#include "mac_simulation.h"

#include <algorithm>
#include <cmath>

using namespace noita;

namespace {
constexpr float ACTIVE_MASS_EPSILON = 0.01f;
constexpr float SAND_PRESSURE_PUSH_THRESHOLD = 0.35f;
constexpr float SAND_PRESSURE_PUSH_STRENGTH = 0.025f;
constexpr float SAND_MAX_PRESSURE_PUSH_SPEED = 1.25f;

float clampf(float v, float lo, float hi) {
	return std::max(lo, std::min(v, hi));
}

int sign_nonzero(float v) {
	return v > 0.0f ? 1 : (v < 0.0f ? -1 : 0);
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

void MacSimulation::step() {
	rigid_solver.step(grid);
	// Order for coupled materials:
	// 1. Powder/sand moves first and becomes a solid-like boundary for fluid.
	// 2. MAC fluid solves pressure/advection against the new powder boundary.
	// 3. Fluid pressure writes small velocity impulses back to active powder.
	bool had_powder = powder_present;
	if (had_powder) {
		grid.rebuild_active_cells(ACTIVE_MASS_EPSILON);
		had_powder = !grid.active_powder_cells.empty();
		powder_present = had_powder;
	}
	if (had_powder) {
		powder_solver.step(grid);
		grid.rebuild_active_cells(ACTIVE_MASS_EPSILON);
	}
	fluid_solver.step();
	if (had_powder) {
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

void MacSimulation::clear() { powder_present = false; gas_present = false; fire_present = false; rigid_paint_stroke_active = false; rigid_solver.clear(); fluid_solver.clear(); }
void MacSimulation::generate_basin() { powder_present = false; gas_present = false; fire_present = false; rigid_paint_stroke_active = false; rigid_solver.clear(); fluid_solver.generate_basin(); rigid_solver.rebuild_all(grid); }
void MacSimulation::generate_rigid_collision_test() {
	powder_present = false;
	gas_present = false;
	fire_present = false;
	rigid_paint_stroke_active = false;
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
int64_t MacSimulation::get_step_count() const { return fluid_solver.get_step_count(); }
int32_t MacSimulation::get_last_pcg_iterations() const { return fluid_solver.get_last_pcg_iterations(); }
double MacSimulation::get_last_pcg_residual() const { return fluid_solver.get_last_pcg_residual(); }
