#include "mac_simulation.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

using namespace noita;

namespace {
constexpr float ACTIVE_MASS_EPSILON = 0.01f;
constexpr float SAND_PRESSURE_PUSH_THRESHOLD = 0.35f;
constexpr float SAND_PRESSURE_PUSH_STRENGTH = 0.025f;
constexpr float SAND_MAX_PRESSURE_PUSH_SPEED = 1.25f;
constexpr float RIGID_LIQUID_EPSILON = 0.001f;
constexpr float RIGID_DISPLACED_LIQUID_MAX_FILL = 2.0f;
constexpr int32_t RIGID_LIQUID_TARGET_RADIUS = 2;

float clampf(float v, float lo, float hi) {
	return std::max(lo, std::min(v, hi));
}

int sign_nonzero(float v) {
	return v > 0.0f ? 1 : (v < 0.0f ? -1 : 0);
}

uint32_t hash_u32(uint32_t x) {
	x ^= x >> 16;
	x *= 0x7feb352du;
	x ^= x >> 15;
	x *= 0x846ca68bu;
	x ^= x >> 16;
	return x;
}

void update_liquid_cell_identity(WorldGrid &grid, int32_t i) {
	const float fill = std::max(0.0f, grid.volume_fraction[i]);
	if (fill <= RIGID_LIQUID_EPSILON) {
		const int32_t x = i % grid.width;
		const int32_t y = i / grid.width;
		grid.make_air(x, y);
		return;
	}
	grid.oil[i] = clampf(grid.oil[i], 0.0f, fill);
	grid.toxic[i] = clampf(grid.toxic[i], 0.0f, fill);
	const float oil_f = clampf(grid.oil[i] / fill, 0.0f, 1.0f);
	const float toxic_f = clampf(grid.toxic[i] / fill, 0.0f, 1.0f);
	grid.material[i] = (toxic_f > 0.25f && oil_f < 0.5f) ? MATERIAL_TOXIC : (oil_f >= 0.5f ? MATERIAL_OIL : MATERIAL_WATER);
	grid.density[i] = get_material_def(MATERIAL_WATER).density * (1.0f - oil_f) + get_material_def(MATERIAL_OIL).density * oil_f;
}
} // namespace

MacSimulation::MacSimulation() {
	grid.set_size(320, 180);
	fluid_solver.bind_world(grid);
}

void MacSimulation::set_world_size(int32_t p_width, int32_t p_height) { fluid_solver.set_world_size(p_width, p_height); rigid_solver.clear(); }
int32_t MacSimulation::get_width() const { return fluid_solver.get_width(); }
int32_t MacSimulation::get_height() const { return fluid_solver.get_height(); }

void MacSimulation::set_dt(double p_dt) { fluid_solver.set_dt(p_dt); }
double MacSimulation::get_dt() const { return fluid_solver.get_dt(); }
void MacSimulation::set_gravity(double p_gravity) { fluid_solver.set_gravity(p_gravity); }
double MacSimulation::get_gravity() const { return fluid_solver.get_gravity(); }
void MacSimulation::set_viscosity(double p_viscosity) { fluid_solver.set_viscosity(p_viscosity); }
double MacSimulation::get_viscosity() const { return fluid_solver.get_viscosity(); }
void MacSimulation::set_pressure_iterations(int32_t p_iterations) { fluid_solver.set_pressure_iterations(p_iterations); }
int32_t MacSimulation::get_pressure_iterations() const { return fluid_solver.get_pressure_iterations(); }
void MacSimulation::set_pressure_active_mass(double p_mass) { fluid_solver.set_pressure_active_mass(p_mass); }
double MacSimulation::get_pressure_active_mass() const { return fluid_solver.get_pressure_active_mass(); }
void MacSimulation::set_density_correction_strength(double p_strength) { fluid_solver.set_density_correction_strength(p_strength); }
double MacSimulation::get_density_correction_strength() const { return fluid_solver.get_density_correction_strength(); }
void MacSimulation::set_underfill_correction_strength(double p_strength) { fluid_solver.set_underfill_correction_strength(p_strength); }
double MacSimulation::get_underfill_correction_strength() const { return fluid_solver.get_underfill_correction_strength(); }
void MacSimulation::set_rigid_liquid_impulse_strength(double p_strength) { rigid_liquid_impulse_strength = clampf(static_cast<float>(p_strength), 0.0f, 3.0f); }
double MacSimulation::get_rigid_liquid_impulse_strength() const { return rigid_liquid_impulse_strength; }

void MacSimulation::step() {
	begin_budgeted_step();
	while (has_pending_budgeted_step()) {
		advance_budgeted_step(1000000.0);
	}
}

void MacSimulation::run_pre_fluid_solvers() {
	rigid_solver.step(grid);
	resolve_rigid_liquid_overlaps();
	// Order for coupled materials:
	// 1. Powder/sand moves first and becomes a solid-like boundary for fluid.
	// 2. MAC fluid solves pressure/advection against the new powder boundary.
	// 3. Fluid pressure writes small velocity impulses back to active powder.
	budgeted_step_had_powder = powder_present;
	if (budgeted_step_had_powder) {
		grid.rebuild_active_cells(ACTIVE_MASS_EPSILON);
		budgeted_step_had_powder = !grid.active_powder_cells.empty();
		powder_present = budgeted_step_had_powder;
	}
	if (budgeted_step_had_powder) {
		powder_solver.step(grid);
		grid.rebuild_active_cells(ACTIVE_MASS_EPSILON);
	}
}

void MacSimulation::run_post_fluid_solvers() {
	if (budgeted_step_had_powder) {
		resolve_fluid_powder_interactions();
	}
	bool had_gas = gas_present;
	if (had_gas) {
		grid.rebuild_active_cells(ACTIVE_MASS_EPSILON);
		had_gas = !grid.active_gas_cells.empty();
		gas_present = had_gas;
	}
	if (had_gas) {
		gas_solver.step(grid);
	}
	grid.rebuild_active_cells(ACTIVE_MASS_EPSILON);
	bool had_fire = fire_present || !grid.active_fire_cells.empty();
	fire_present = had_fire;
	if (had_fire) {
		fire_solver.step(grid);
	}
	reaction_solver.step(grid);
	grid.rebuild_active_cells(ACTIVE_MASS_EPSILON);
	gas_present = !grid.active_gas_cells.empty();
	fire_present = !grid.active_fire_cells.empty();
	powder_present = !grid.active_powder_cells.empty();
	budgeted_step_had_powder = false;
}

void MacSimulation::begin_budgeted_step() {
	if (budgeted_step_active) {
		return;
	}
	run_pre_fluid_solvers();
	fluid_solver.begin_step_job();
	// Keep material interaction responsive: post-fluid systems run once per
	// 30 Hz simulation tick immediately.  If the budgeted fluid job is still
	// pending, they read the last committed fluid state; if it finished in
	// begin_step_job() (e.g. no liquid), they read the just-updated state.
	run_post_fluid_solvers();
	budgeted_step_active = fluid_solver.has_pending_step_job();
}

bool MacSimulation::advance_budgeted_step(double p_fluid_budget_ms) {
	if (!budgeted_step_active) {
		return true;
	}
	const bool fluid_done = fluid_solver.advance_step_job(p_fluid_budget_ms);
	if (!fluid_done) {
		return false;
	}
	budgeted_step_active = false;
	return true;
}

bool MacSimulation::has_pending_budgeted_step() const {
	return budgeted_step_active;
}

void MacSimulation::resolve_rigid_liquid_overlaps() {
	const auto &occupied = rigid_solver.get_current_occupied_cells();
	if (occupied.empty()) {
		return;
	}

	struct BodyLiquidTargets {
		std::vector<int32_t> targets;
	};

	std::unordered_map<int32_t, int32_t> body_to_target_index;
	std::vector<BodyLiquidTargets> body_targets;

	auto is_liquid_index = [&](int32_t idx) -> bool {
		if (idx < 0 || idx >= grid.cell_count()) {
			return false;
		}
		const MaterialDef &def = get_material_def(grid.material[idx]);
		return def.liquid && std::isfinite(grid.volume_fraction[idx]) && grid.volume_fraction[idx] > RIGID_LIQUID_EPSILON;
	};

	auto target_list_for_body = [&](int32_t body_id) -> BodyLiquidTargets & {
		auto it = body_to_target_index.find(body_id);
		if (it != body_to_target_index.end()) {
			return body_targets[it->second];
		}
		const int32_t new_index = static_cast<int32_t>(body_targets.size());
		body_to_target_index[body_id] = new_index;
		body_targets.push_back({});
		return body_targets.back();
	};

	// Fast broad/narrow hybrid: only moving boundary pixels test for nearby
	// liquid.  Bodies whose boundary never touches liquid are ignored entirely.
	for (const RigidBodySolver::RigidCellOccupancy &occ : occupied) {
		if (!occ.moving || !occ.boundary || !grid.in_bounds(occ.x, occ.y)) {
			continue;
		}

		bool touches_liquid = false;
		for (int32_t oy = -1; oy <= 1 && !touches_liquid; oy++) {
			for (int32_t ox = -1; ox <= 1; ox++) {
				const int32_t nx = occ.x + ox;
				const int32_t ny = occ.y + oy;
				if (!grid.in_bounds(nx, ny)) {
					continue;
				}
				if (is_liquid_index(grid.cell_index(nx, ny))) {
					touches_liquid = true;
					break;
				}
			}
		}
		if (!touches_liquid) {
			continue;
		}

		BodyLiquidTargets &targets = target_list_for_body(occ.body_id);
		for (int32_t oy = -RIGID_LIQUID_TARGET_RADIUS; oy <= RIGID_LIQUID_TARGET_RADIUS; oy++) {
			for (int32_t ox = -RIGID_LIQUID_TARGET_RADIUS; ox <= RIGID_LIQUID_TARGET_RADIUS; ox++) {
				if (ox == 0 && oy == 0) {
					continue;
				}
				const int32_t manhattan = std::abs(ox) + std::abs(oy);
				if (manhattan <= 0 || manhattan > RIGID_LIQUID_TARGET_RADIUS) {
					continue;
				}
				const int32_t tx = occ.x + ox;
				const int32_t ty = occ.y + oy;
				if (!grid.in_bounds(tx, ty)) {
					continue;
				}
				const int32_t ti = grid.cell_index(tx, ty);
				if (grid.rigid_body_id[ti] != 0 || get_material_def(grid.material[ti]).blocks_velocity) {
					continue;
				}
				const MaterialDef &dst_def = get_material_def(grid.material[ti]);
				if (grid.material[ti] == MATERIAL_AIR || dst_def.gas || dst_def.liquid) {
					targets.targets.push_back(ti);
				}
			}
		}
	}

	if (body_targets.empty()) {
		return;
	}

	for (BodyLiquidTargets &targets : body_targets) {
		std::sort(targets.targets.begin(), targets.targets.end());
		targets.targets.erase(std::unique(targets.targets.begin(), targets.targets.end()), targets.targets.end());
	}

	bool changed = false;
	for (const RigidBodySolver::RigidCellOccupancy &occ : occupied) {
		if (!occ.moving || !grid.in_bounds(occ.x, occ.y)) {
			continue;
		}
		auto map_it = body_to_target_index.find(occ.body_id);
		if (map_it == body_to_target_index.end()) {
			continue;
		}
		const std::vector<int32_t> &targets = body_targets[map_it->second].targets;
		if (targets.empty()) {
			continue;
		}

		const int32_t from = grid.cell_index(occ.x, occ.y);
		if (!is_liquid_index(from)) {
			continue;
		}

		float payload_fill = std::max(0.0f, grid.volume_fraction[from]);
		float payload_oil = clampf(grid.oil[from], 0.0f, payload_fill);
		float payload_toxic = clampf(grid.toxic[from], 0.0f, payload_fill);
		const float payload_temp = grid.temperature[from];
		const float payload_vx = occ.vx;
		const float payload_vy = occ.vy;

		const uint32_t h = hash_u32(static_cast<uint32_t>(from) ^ (static_cast<uint32_t>(occ.body_id) * 747796405u));
		const int32_t start = static_cast<int32_t>(h % static_cast<uint32_t>(targets.size()));
		// Biased pseudo-random target side: 30% above the overlapped cell
		// (smaller screen-y), 70% below it (larger screen-y).  If the preferred
		// side has no capacity, a second pass falls back to any valid target so
		// liquid is not deleted just because all preferred slots are full.
		const bool prefer_above = (h % 10u) < 3u;
		for (int32_t bias_pass = 0; bias_pass < 2 && payload_fill > RIGID_LIQUID_EPSILON; bias_pass++) {
			const bool enforce_vertical_bias = bias_pass == 0;
			for (int32_t pass = 0; pass < static_cast<int32_t>(targets.size()) && payload_fill > RIGID_LIQUID_EPSILON; pass++) {
				const int32_t to = targets[(start + pass) % targets.size()];
				if (enforce_vertical_bias) {
					const int32_t target_y = to / grid.width;
					if (prefer_above) {
						if (target_y >= occ.y) {
							continue;
						}
					} else if (target_y <= occ.y) {
						continue;
					}
				}
			if (to == from || to < 0 || to >= grid.cell_count() || grid.rigid_body_id[to] != 0) {
				continue;
			}
			const MaterialDef &dst_def = get_material_def(grid.material[to]);
			if (!(grid.material[to] == MATERIAL_AIR || dst_def.gas || dst_def.liquid) || dst_def.blocks_velocity) {
				continue;
			}
			const float old_fill = dst_def.liquid ? std::max(0.0f, grid.volume_fraction[to]) : 0.0f;
			const float capacity = std::max(0.0f, RIGID_DISPLACED_LIQUID_MAX_FILL - old_fill);
			if (capacity <= RIGID_LIQUID_EPSILON) {
				continue;
			}

			const float moved = std::min(payload_fill, capacity);
			const float frac = moved / std::max(payload_fill, RIGID_LIQUID_EPSILON);
			const float moved_oil = payload_oil * frac;
			const float moved_toxic = payload_toxic * frac;
			const float old_oil = dst_def.liquid ? clampf(grid.oil[to], 0.0f, old_fill) : 0.0f;
			const float old_toxic = dst_def.liquid ? clampf(grid.toxic[to], 0.0f, old_fill) : 0.0f;
			const float new_fill = old_fill + moved;
			const int32_t to_x = to % grid.width;
			const int32_t to_y = to / grid.width;
			float dir_x = static_cast<float>(to_x - occ.x);
			float dir_y = static_cast<float>(to_y - occ.y);
			const float dir_len = std::sqrt(dir_x * dir_x + dir_y * dir_y);
			if (dir_len > 0.0001f) {
				dir_x /= dir_len;
				dir_y /= dir_len;
			} else {
				dir_x = 0.0f;
				dir_y = -1.0f;
			}
			const float rigid_speed = std::sqrt(payload_vx * payload_vx + payload_vy * payload_vy);
			const float splash_speed = rigid_speed * rigid_liquid_impulse_strength;
			const float displaced_vx = payload_vx + dir_x * splash_speed;
			const float displaced_vy = payload_vy + dir_y * splash_speed;

			grid.volume_fraction[to] = new_fill;
			grid.oil[to] = old_oil + moved_oil;
			grid.toxic[to] = old_toxic + moved_toxic;
			grid.temperature[to] = new_fill > RIGID_LIQUID_EPSILON ?
					(grid.temperature[to] * old_fill + payload_temp * moved) / new_fill :
					payload_temp;
			grid.velocity_x[to] = new_fill > RIGID_LIQUID_EPSILON ?
					(grid.velocity_x[to] * old_fill + displaced_vx * moved) / new_fill :
					displaced_vx;
			grid.velocity_y[to] = new_fill > RIGID_LIQUID_EPSILON ?
					(grid.velocity_y[to] * old_fill + displaced_vy * moved) / new_fill :
					displaced_vy;
			update_liquid_cell_identity(grid, to);

			payload_fill -= moved;
			payload_oil = std::max(0.0f, payload_oil - moved_oil);
			payload_toxic = std::max(0.0f, payload_toxic - moved_toxic);
			changed = true;
			}
		}

		if (payload_fill <= RIGID_LIQUID_EPSILON) {
			grid.make_air(occ.x, occ.y);
			changed = true;
		} else if (payload_fill < grid.volume_fraction[from]) {
			grid.volume_fraction[from] = payload_fill;
			grid.oil[from] = payload_oil;
			grid.toxic[from] = payload_toxic;
			update_liquid_cell_identity(grid, from);
			changed = true;
		}
	}

	if (changed) {
		grid.rebuild_active_cells(ACTIVE_MASS_EPSILON);
	}
}

void MacSimulation::resolve_fluid_powder_interactions() {
	const int32_t dirs[4][2] = {
		{ -1, 0 },
		{ 1, 0 },
		{ 0, -1 },
		{ 0, 1 },
	};

	for (int32_t i : grid.active_powder_cells) {
		if (i < 0 || i >= grid.cell_count() || !get_material_def(grid.material[i]).powder) {
			continue;
		}
		const int32_t x = i % grid.width;
		const int32_t y = i / grid.width;
		float fx = 0.0f;
		float fy = 0.0f;
		for (const auto &dir : dirs) {
			const int32_t nx = x + dir[0];
			const int32_t ny = y + dir[1];
			if (!grid.in_bounds(nx, ny) || !grid.is_liquid_cell(nx, ny, ACTIVE_MASS_EPSILON)) {
				continue;
			}
			const int32_t ni = grid.cell_index(nx, ny);
			const float p = std::max(0.0f, grid.pressure[ni] - SAND_PRESSURE_PUSH_THRESHOLD);
			if (p <= 0.0f) {
				continue;
			}
			// Pressure pushes the sand away from the liquid neighbor.
			fx -= static_cast<float>(dir[0]) * p;
			fy -= static_cast<float>(dir[1]) * p;
		}

		if (fx == 0.0f && fy == 0.0f) {
			continue;
		}

		const int sx = sign_nonzero(fx);
		const int sy = sign_nonzero(fy);
		if (sx != 0 && grid.blocks_velocity(x + sx, y)) {
			fx = 0.0f;
		}
		if (sy != 0 && grid.blocks_velocity(x, y + sy)) {
			fy = 0.0f;
		}
		grid.velocity_x[i] = clampf(grid.velocity_x[i] + fx * SAND_PRESSURE_PUSH_STRENGTH, -SAND_MAX_PRESSURE_PUSH_SPEED, SAND_MAX_PRESSURE_PUSH_SPEED);
		grid.velocity_y[i] = clampf(grid.velocity_y[i] + fy * SAND_PRESSURE_PUSH_STRENGTH, -SAND_MAX_PRESSURE_PUSH_SPEED, SAND_MAX_PRESSURE_PUSH_SPEED);
	}
}

void MacSimulation::clear() { powder_present = false; gas_present = false; fire_present = false; rigid_paint_stroke_active = false; budgeted_step_active = false; budgeted_step_had_powder = false; rigid_solver.clear(); fluid_solver.clear(); }
void MacSimulation::generate_basin() { powder_present = false; gas_present = false; fire_present = false; rigid_paint_stroke_active = false; budgeted_step_active = false; budgeted_step_had_powder = false; rigid_solver.clear(); fluid_solver.generate_basin(); rigid_solver.rebuild_all(grid); }
void MacSimulation::generate_rigid_collision_test() {
	powder_present = false;
	gas_present = false;
	fire_present = false;
	rigid_paint_stroke_active = false;
	budgeted_step_active = false;
	budgeted_step_had_powder = false;
	rigid_solver.spawn_collision_test(grid);
	grid.rebuild_active_cells(ACTIVE_MASS_EPSILON);
}

void MacSimulation::begin_rigid_paint_stroke() {
	rigid_paint_stroke_active = true;
	rigid_solver.set_auto_process_dirty(false);
}

void MacSimulation::end_rigid_paint_stroke() {
	if (!rigid_paint_stroke_active) {
		return;
	}
	rigid_paint_stroke_active = false;
	rigid_solver.set_auto_process_dirty(true);
	rigid_solver.process_dirty(grid);
}

void MacSimulation::paint_circle(double p_x, double p_y, double p_radius, int32_t p_material) {
	if (p_material != MATERIAL_SAND && p_material != MATERIAL_SMOKE && p_material != MATERIAL_TOXIC &&
			p_material != MATERIAL_FIRE && p_material != MATERIAL_STEAM && p_material != MATERIAL_TOXIC_GAS &&
			p_material != MATERIAL_FLAMMABLE_GAS) {
		fluid_solver.paint_circle(p_x, p_y, p_radius, p_material);
		if (p_material == MATERIAL_ROCK || p_material == MATERIAL_AIR) {
			const int32_t cx = static_cast<int32_t>(std::floor(p_x));
			const int32_t cy = static_cast<int32_t>(std::floor(p_y));
			const int32_t r = std::max(1, static_cast<int32_t>(std::ceil(p_radius))) + 3;
			if (p_material == MATERIAL_AIR) {
				rigid_solver.destroy_circle(grid, static_cast<float>(p_x), static_cast<float>(p_y), static_cast<float>(p_radius));
			}
			rigid_solver.mark_dirty_rect(grid, cx - r, cy - r, cx + r, cy + r);
			if (!rigid_paint_stroke_active) {
				rigid_solver.process_dirty(grid);
			}
		}
		return;
	}
	const int32_t cx = static_cast<int32_t>(std::floor(p_x));
	const int32_t cy = static_cast<int32_t>(std::floor(p_y));
	const int32_t r = std::max(1, static_cast<int32_t>(std::ceil(p_radius)));
	const int32_t r2 = r * r;
	for (int32_t y = cy - r; y <= cy + r; y++) {
		for (int32_t x = cx - r; x <= cx + r; x++) {
			const int32_t dx = x - cx;
			const int32_t dy = y - cy;
			if (dx * dx + dy * dy <= r2) {
				if (p_material == MATERIAL_SAND) {
					grid.make_sand(x, y, 1.0f);
					powder_present = true;
				} else if (p_material == MATERIAL_SMOKE) {
					grid.make_smoke(x, y, 1.0f);
					gas_present = true;
				} else if (p_material == MATERIAL_TOXIC) {
					grid.make_toxic(x, y, 1.0f);
				} else if (p_material == MATERIAL_FIRE) {
					grid.make_fire(x, y, 1.0f);
					fire_present = true;
				} else if (p_material == MATERIAL_STEAM) {
					grid.make_steam(x, y, 1.0f);
					gas_present = true;
				} else if (p_material == MATERIAL_TOXIC_GAS) {
					grid.make_toxic_gas(x, y, 1.0f);
					gas_present = true;
				} else if (p_material == MATERIAL_FLAMMABLE_GAS) {
					grid.make_flammable_gas(x, y, 1.0f);
					gas_present = true;
				}
			}
		}
	}
	grid.rebuild_active_cells(ACTIVE_MASS_EPSILON);
}

void MacSimulation::inject_water(double p_x, double p_y, double p_radius, double p_mass_per_cell, double p_velocity_x, double p_velocity_y) { fluid_solver.inject_water(p_x, p_y, p_radius, p_mass_per_cell, p_velocity_x, p_velocity_y); }
void MacSimulation::fill_rgba_pixels(std::vector<uint8_t> &p_pixels) const { fluid_solver.fill_rgba_pixels(p_pixels); rigid_solver.draw_overlay_rgba(grid, p_pixels); }
bool MacSimulation::start_rigid_drag(double p_x, double p_y) { return rigid_solver.start_drag(static_cast<float>(p_x), static_cast<float>(p_y)); }
void MacSimulation::update_rigid_drag(double p_x, double p_y, bool p_rotate) { rigid_solver.update_drag(static_cast<float>(p_x), static_cast<float>(p_y), p_rotate); }
void MacSimulation::end_rigid_drag() { rigid_solver.end_drag(); }
int32_t MacSimulation::get_rigid_body_count() const { return rigid_solver.get_body_count(); }
int32_t MacSimulation::get_rigid_awake_count() const { return rigid_solver.get_awake_body_count(); }
int32_t MacSimulation::get_rigid_sleeping_count() const { return rigid_solver.get_sleeping_body_count(); }

double MacSimulation::get_total_water_mass() const { return fluid_solver.get_total_water_mass(); }
int64_t MacSimulation::get_water_cell_count() const { return fluid_solver.get_water_cell_count(); }
double MacSimulation::get_average_water_mass() const { return fluid_solver.get_average_water_mass(); }
double MacSimulation::get_last_step_ms() const { return fluid_solver.get_last_step_ms(); }
double MacSimulation::get_last_predict_ms() const { return fluid_solver.get_last_predict_ms(); }
double MacSimulation::get_last_build_ms() const { return fluid_solver.get_last_build_ms(); }
double MacSimulation::get_last_pcg_ms() const { return fluid_solver.get_last_pcg_ms(); }
double MacSimulation::get_last_project_ms() const { return fluid_solver.get_last_project_ms(); }
double MacSimulation::get_last_advect_ms() const { return fluid_solver.get_last_advect_ms(); }
double MacSimulation::get_last_clamp_ms() const { return fluid_solver.get_last_clamp_ms(); }
int32_t MacSimulation::get_active_region_min_x() const { return fluid_solver.get_active_region_min_x(); }
int32_t MacSimulation::get_active_region_min_y() const { return fluid_solver.get_active_region_min_y(); }
int32_t MacSimulation::get_active_region_max_x() const { return fluid_solver.get_active_region_max_x(); }
int32_t MacSimulation::get_active_region_max_y() const { return fluid_solver.get_active_region_max_y(); }
int32_t MacSimulation::get_active_region_pad() const { return fluid_solver.get_active_region_pad(); }
double MacSimulation::get_active_region_max_speed() const { return fluid_solver.get_active_region_max_speed(); }
int64_t MacSimulation::get_step_count() const { return fluid_solver.get_step_count(); }
int32_t MacSimulation::get_last_pcg_iterations() const { return fluid_solver.get_last_pcg_iterations(); }
double MacSimulation::get_last_pcg_residual() const { return fluid_solver.get_last_pcg_residual(); }

