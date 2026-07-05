#include "mac_fluid_solver.h"

#include <algorithm>
#include <chrono>
#include <cmath>

using namespace noita;

namespace {
constexpr float MASS_EPSILON = 0.01f;
constexpr float PCG_EPSILON = 1.0e-7f;
constexpr float DEEPEST_COLOR_THRESHOLD = 0.60f;

float clampf(float v, float lo, float hi) {
	return std::max(lo, std::min(v, hi));
}

float hash01(int32_t x, int32_t y, int32_t seed) {
	uint32_t h = static_cast<uint32_t>(x) * 374761393u +
			static_cast<uint32_t>(y) * 668265263u +
			static_cast<uint32_t>(seed) * 2246822519u;
	h = (h ^ (h >> 13)) * 1274126177u;
	h ^= h >> 16;
	return static_cast<float>(h & 0x00ffffffu) / static_cast<float>(0x01000000u);
}

double elapsed_ms(std::chrono::high_resolution_clock::time_point p_start, std::chrono::high_resolution_clock::time_point p_end) {
	return std::chrono::duration<double, std::milli>(p_end - p_start).count();
}
} // namespace

MacFluidSolver::MacFluidSolver() = default;

void MacFluidSolver::bind_world(WorldGrid &p_world) {
	world = &p_world;
	width = world->width;
	height = world->height;
	ensure_buffers();
}

int32_t MacFluidSolver::cell_index(int32_t p_x, int32_t p_y) const { return p_y * width + p_x; }
int32_t MacFluidSolver::u_index(int32_t p_x, int32_t p_y) const { return p_y * (width + 1) + p_x; }
int32_t MacFluidSolver::v_index(int32_t p_x, int32_t p_y) const { return p_y * width + p_x; }

const MaterialDef &MacFluidSolver::cell_material_def(int32_t p_x, int32_t p_y) const {
	if (!in_bounds(p_x, p_y)) {
		// Treat the outside of the simulation domain as rock for boundary tests.
		return solid_boundary_material_def();
	}
	if (world->has_dynamic_rigid_cell(p_x, p_y)) {
		return solid_boundary_material_def();
	}
	return get_material_def(world->material[cell_index(p_x, p_y)]);
}

bool MacFluidSolver::in_bounds(int32_t p_x, int32_t p_y) const {
	return p_x >= 0 && p_y >= 0 && p_x < width && p_y < height;
}

bool MacFluidSolver::is_solid_cell(int32_t p_x, int32_t p_y) const {
	return cell_material_def(p_x, p_y).solid;
}

bool MacFluidSolver::is_liquid_cell(int32_t p_x, int32_t p_y) const {
	if (!in_bounds(p_x, p_y)) {
		return false;
	}
	const int32_t i = cell_index(p_x, p_y);
	if (world->rigid_body_id[i] != 0) {
		return false;
	}
	const MaterialDef &def = get_material_def(world->material[i]);
	return def.liquid && std::isfinite(world->volume_fraction[i]) && world->volume_fraction[i] > MASS_EPSILON;
}

bool MacFluidSolver::is_internal_liquid_cell(int32_t p_x, int32_t p_y) const {
	if (!is_liquid_cell(p_x, p_y)) {
		return false;
	}
	const int32_t nx[4] = { p_x - 1, p_x + 1, p_x, p_x };
	const int32_t ny[4] = { p_y, p_y, p_y - 1, p_y + 1 };
	for (int32_t k = 0; k < 4; k++) {
		const MaterialDef &neighbor_def = cell_material_def(nx[k], ny[k]);
		if (neighbor_def.blocks_velocity) {
			continue;
		}
		if (!in_bounds(nx[k], ny[k])) {
			continue;
		}
		int32_t ni = cell_index(nx[k], ny[k]);
		if (!std::isfinite(world->volume_fraction[ni]) || world->volume_fraction[ni] <= pressure_active_mass) {
			return false;
		}
	}
	return true;
}

bool MacFluidSolver::is_pressure_active_cell(int32_t p_x, int32_t p_y) const {
	if (!in_bounds(p_x, p_y)) {
		return false;
	}
	const int32_t i = cell_index(p_x, p_y);
	if (world->rigid_body_id[i] != 0) {
		return false;
	}
	const MaterialDef &def = get_material_def(world->material[i]);
	if (def.blocks_velocity) {
		return false;
	}
	float m = world->volume_fraction[i];
	// Time-layer guard for semi-Lagrangian velocity advection:
	// velocity has already been transported by dt before pressure projection,
	// but volume will be advected after projection.  A fast-moving liquid front
	// can therefore arrive at a cell whose current volume is still below the
	// threshold.  Treat the cell as pressure-active if either the current cell
	// is above threshold or its backtraced source position was above threshold.
	const float source_m = backtraced_pressure_volume(p_x, p_y);
	const bool current_active = def.pressure_solved && std::isfinite(m) &&
			(m > pressure_active_mass || is_internal_liquid_cell(p_x, p_y));
	return current_active ||
			(std::isfinite(source_m) && source_m > pressure_active_mass);
}

bool MacFluidSolver::is_pressure_active_masked(int32_t p_x, int32_t p_y) const {
	if (!in_bounds(p_x, p_y)) {
		return false;
	}
	const int32_t i = cell_index(p_x, p_y);
	return active_region_mask[i] != 0 && pressure_active_mask[i] != 0;
}

bool MacFluidSolver::u_face_blocked(int32_t p_x, int32_t p_y) const {
	if (p_y < 0 || p_y >= height || p_x < 0 || p_x > width) {
		return true;
	}
	return cell_material_def(p_x - 1, p_y).blocks_velocity || cell_material_def(p_x, p_y).blocks_velocity;
}

bool MacFluidSolver::v_face_blocked(int32_t p_x, int32_t p_y) const {
	if (p_x < 0 || p_x >= width || p_y < 0 || p_y > height) {
		return true;
	}
	return cell_material_def(p_x, p_y - 1).blocks_velocity || cell_material_def(p_x, p_y).blocks_velocity;
}

float MacFluidSolver::cell_density(int32_t p_x, int32_t p_y) const {
	if (!in_bounds(p_x, p_y)) {
		return std::max(density, 1.0e-6f);
	}
	const int32_t i = cell_index(p_x, p_y);
	const MaterialDef &def = get_material_def(world->material[i]);
	if (def.liquid && std::isfinite(world->volume_fraction[i]) && world->volume_fraction[i] > MASS_EPSILON) {
		const float oil_f = oil_fraction_at(i);
		const float water_rho = get_material_def(MATERIAL_WATER).density;
		const float oil_rho = get_material_def(MATERIAL_OIL).density;
		return std::max((water_rho * (1.0f - oil_f) + oil_rho * oil_f) * density, 1.0e-6f);
	}
	if (!def.blocks_velocity && backtraced_pressure_volume(p_x, p_y) > pressure_active_mass) {
		// If this is an air/low-volume destination cell made active by the
		// backtraced-source OR gate, use liquid density for the pressure matrix.
		// Otherwise alpha=dt/rho_air would over-amplify projection impulses.
		return std::max(get_material_def(MATERIAL_WATER).density * density, 1.0e-6f);
	}
	return std::max(world->density[i] * density, 1.0e-6f);
}

float MacFluidSolver::oil_fraction_at(int32_t p_index) const {
	if (world == nullptr || p_index < 0 || p_index >= width * height) {
		return 0.0f;
	}
	const float vol = std::max(world->volume_fraction[p_index], MASS_EPSILON);
	return clampf(world->oil[p_index] / vol, 0.0f, 1.0f);
}
float MacFluidSolver::u_face_alpha(int32_t p_x, int32_t p_y) const {
	if (u_face_blocked(p_x, p_y)) {
		return 0.0f;
	}
	bool left_active = is_pressure_active_masked(p_x - 1, p_y);
	bool right_active = is_pressure_active_masked(p_x, p_y);
	if (!left_active && !right_active) {
		return 0.0f;
	}
	float rho_sum = 0.0f;
	float rho_count = 0.0f;
	if (left_active) {
		rho_sum += cell_density(p_x - 1, p_y);
		rho_count += 1.0f;
	}
	if (right_active) {
		rho_sum += cell_density(p_x, p_y);
		rho_count += 1.0f;
	}
	return dt / std::max(rho_sum / std::max(rho_count, 1.0f), 1.0e-6f);
}

float MacFluidSolver::v_face_alpha(int32_t p_x, int32_t p_y) const {
	if (v_face_blocked(p_x, p_y)) {
		return 0.0f;
	}
	bool up_active = is_pressure_active_masked(p_x, p_y - 1);
	bool down_active = is_pressure_active_masked(p_x, p_y);
	if (!up_active && !down_active) {
		return 0.0f;
	}
	float rho_sum = 0.0f;
	float rho_count = 0.0f;
	if (up_active) {
		rho_sum += cell_density(p_x, p_y - 1);
		rho_count += 1.0f;
	}
	if (down_active) {
		rho_sum += cell_density(p_x, p_y);
		rho_count += 1.0f;
	}
	return dt / std::max(rho_sum / std::max(rho_count, 1.0f), 1.0e-6f);
}

void MacFluidSolver::ensure_buffers() {
	if (world == nullptr) {
		return;
	}
	width = world->width;
	height = world->height;
	const int32_t cell_count = width * height;
	const int32_t u_count = (width + 1) * height;
	const int32_t v_count = width * (height + 1);

	next_mass.resize(cell_count, 0.0f);
	next_toxic.resize(cell_count, 0.0f);
	next_oil.resize(cell_count, 0.0f);
	rhs.resize(cell_count, 0.0f);
	residual.resize(cell_count, 0.0f);
	direction.resize(cell_count, 0.0f);
	q_vec.resize(cell_count, 0.0f);
	z_vec.resize(cell_count, 0.0f);
	diag_inv.resize(cell_count, 0.0f);
	active_region_mask.resize(cell_count, 0);
	active_region_indices.clear();
	active_region_indices.reserve(cell_count);
	pressure_active_mask.resize(cell_count, 0);
	pressure_row_index.resize(cell_count, -1);
	pressure_rows.clear();
	pressure_rows.reserve(cell_count);
	mic_rows.clear();
	mic_rows.reserve(cell_count);
	mic_diag.clear();
	mic_diag.reserve(cell_count);
	mic_temp.clear();
	mic_temp.reserve(cell_count);
	outflow_mass.resize(cell_count, 0.0f);
	active_cells.clear();
	active_cells.reserve(cell_count);
	active_liquid_source_indices.clear();
	active_liquid_source_indices.reserve(cell_count);

	u.resize(u_count, 0.0f);
	u_tmp.resize(u_count, 0.0f);
	u_advected.resize(u_count, 0.0f);
	u_alpha.resize(u_count, 0.0f);
	u_mass_flux.resize(u_count, 0.0f);
	u_active_face_mask.resize(u_count, 0);
	v.resize(v_count, 0.0f);
	v_tmp.resize(v_count, 0.0f);
	v_advected.resize(v_count, 0.0f);
	v_alpha.resize(v_count, 0.0f);
	v_mass_flux.resize(v_count, 0.0f);
	v_active_face_mask.resize(v_count, 0);
	active_u_faces.clear();
	active_u_faces.reserve(u_count);
	active_v_faces.clear();
	active_v_faces.reserve(v_count);
}

void MacFluidSolver::clear_fields() {
	if (world == nullptr) {
		return;
	}
	world->clear();
	std::fill(next_mass.begin(), next_mass.end(), 0.0f);
	std::fill(next_toxic.begin(), next_toxic.end(), 0.0f);
	std::fill(next_oil.begin(), next_oil.end(), 0.0f);
	std::fill(active_region_mask.begin(), active_region_mask.end(), 0);
	std::fill(u_active_face_mask.begin(), u_active_face_mask.end(), 0);
	std::fill(v_active_face_mask.begin(), v_active_face_mask.end(), 0);
	std::fill(rhs.begin(), rhs.end(), 0.0f);
	std::fill(outflow_mass.begin(), outflow_mass.end(), 0.0f);
	std::fill(u.begin(), u.end(), 0.0f);
	std::fill(v.begin(), v.end(), 0.0f);
	std::fill(u_tmp.begin(), u_tmp.end(), 0.0f);
	std::fill(v_tmp.begin(), v_tmp.end(), 0.0f);
	std::fill(u_advected.begin(), u_advected.end(), 0.0f);
	std::fill(v_advected.begin(), v_advected.end(), 0.0f);
	std::fill(u_alpha.begin(), u_alpha.end(), 0.0f);
	std::fill(v_alpha.begin(), v_alpha.end(), 0.0f);
	std::fill(u_mass_flux.begin(), u_mass_flux.end(), 0.0f);
	std::fill(v_mass_flux.begin(), v_mass_flux.end(), 0.0f);
	active_cells.clear();
	active_liquid_source_indices.clear();
	active_region_indices.clear();
	active_u_faces.clear();
	active_v_faces.clear();
	pressure_rows.clear();
	mic_rows.clear();
	mic_diag.clear();
	mic_temp.clear();
	has_active_liquid = false;
	had_active_liquid_last_step = false;
	active_min_x = 0;
	active_min_y = 0;
	active_max_x = -1;
	active_max_y = -1;
	active_pad = 0;
	active_max_speed = 0.0f;
	step_phase = StepPhase::IDLE;
	step_job_active = false;
}

bool MacFluidSolver::compute_active_liquid_region() {
	for (int32_t i : active_region_indices) {
		active_region_mask[i] = 0;
	}
	active_region_indices.clear();
	for (int32_t idx : active_u_faces) {
		u_active_face_mask[idx] = 0;
	}
	active_u_faces.clear();
	for (int32_t idx : active_v_faces) {
		v_active_face_mask[idx] = 0;
	}
	active_v_faces.clear();
	active_liquid_source_indices.clear();

	has_active_liquid = false;
	active_min_x = width;
	active_min_y = height;
	active_max_x = -1;
	active_max_y = -1;
	active_pad = 0;
	active_max_speed = 0.0f;

	for (int32_t y = 0; y < height; y++) {
		for (int32_t x = 0; x < width; x++) {
			const int32_t i = cell_index(x, y);
			if (world->rigid_body_id[i] != 0) {
				continue;
			}
			const MaterialDef &def = get_material_def(world->material[i]);
			if (!def.liquid || !std::isfinite(world->volume_fraction[i]) || world->volume_fraction[i] <= MASS_EPSILON) {
				continue;
			}
			has_active_liquid = true;
			active_liquid_source_indices.push_back(i);
			active_min_x = std::min(active_min_x, x);
			active_min_y = std::min(active_min_y, y);
			active_max_x = std::max(active_max_x, x);
			active_max_y = std::max(active_max_y, y);

			const float vx = 0.5f * (u[u_index(x, y)] + u[u_index(x + 1, y)]);
			const float vy = 0.5f * (v[v_index(x, y)] + v[v_index(x, y + 1)]);
			if (std::isfinite(vx) && std::isfinite(vy)) {
				active_max_speed = std::max(active_max_speed, std::sqrt(vx * vx + vy * vy));
			}
		}
	}

	if (!has_active_liquid) {
		active_min_x = 0;
		active_min_y = 0;
		active_max_x = -1;
		active_max_y = -1;
		return false;
	}

	active_pad = std::max(3, static_cast<int32_t>(std::ceil(active_max_speed * dt * 1.5f)) + 3);

	active_min_x = width;
	active_min_y = height;
	active_max_x = -1;
	active_max_y = -1;

	auto add_active_cell = [&](int32_t x, int32_t y) {
		if (!in_bounds(x, y)) {
			return;
		}
		const int32_t i = cell_index(x, y);
		if (active_region_mask[i] != 0) {
			return;
		}
		active_region_mask[i] = 1;
		active_region_indices.push_back(i);
		active_min_x = std::min(active_min_x, x);
		active_min_y = std::min(active_min_y, y);
		active_max_x = std::max(active_max_x, x);
		active_max_y = std::max(active_max_y, y);
	};

	for (int32_t i : active_liquid_source_indices) {
		const int32_t cx = i % width;
		const int32_t cy = i / width;
		for (int32_t y = cy - active_pad; y <= cy + active_pad; y++) {
			for (int32_t x = cx - active_pad; x <= cx + active_pad; x++) {
				add_active_cell(x, y);
			}
		}
	}

	auto add_u_face = [&](int32_t x, int32_t y) {
		if (x < 0 || x > width || y < 0 || y >= height) {
			return;
		}
		const int32_t idx = u_index(x, y);
		if (u_active_face_mask[idx] == 0) {
			u_active_face_mask[idx] = 1;
			active_u_faces.push_back(idx);
		}
	};

	auto add_v_face = [&](int32_t x, int32_t y) {
		if (x < 0 || x >= width || y < 0 || y > height) {
			return;
		}
		const int32_t idx = v_index(x, y);
		if (v_active_face_mask[idx] == 0) {
			v_active_face_mask[idx] = 1;
			active_v_faces.push_back(idx);
		}
	};

	for (int32_t i : active_region_indices) {
		const int32_t x = i % width;
		const int32_t y = i / width;
		add_u_face(x, y);
		add_u_face(x + 1, y);
		add_v_face(x, y);
		add_v_face(x, y + 1);
	}

	if (active_region_indices.empty()) {
		active_min_x = 0;
		active_min_y = 0;
		active_max_x = -1;
		active_max_y = -1;
		return false;
	}

	std::sort(active_region_indices.begin(), active_region_indices.end());
	std::sort(active_u_faces.begin(), active_u_faces.end());
	std::sort(active_v_faces.begin(), active_v_faces.end());

	for (int32_t i : active_region_indices) {
		const int32_t x = i % width;
		const int32_t y = i / width;
		active_min_x = std::min(active_min_x, x);
		active_min_y = std::min(active_min_y, y);
		active_max_x = std::max(active_max_x, x);
		active_max_y = std::max(active_max_y, y);
	}

	// Clear stale per-step scalar fields only on cells/faces that will be read
	// this step.  Values outside the active mask are ignored by mask checks.
	for (int32_t i : active_region_indices) {
		rhs[i] = 0.0f;
		diag_inv[i] = 0.0f;
		pressure_active_mask[i] = 0;
		outflow_mass[i] = 0.0f;
	}
	for (int32_t idx : active_u_faces) {
		u_alpha[idx] = 0.0f;
		u_mass_flux[idx] = 0.0f;
	}
	for (int32_t idx : active_v_faces) {
		v_alpha[idx] = 0.0f;
		v_mass_flux[idx] = 0.0f;
		}
	return true;
}

void MacFluidSolver::apply_solid_boundaries(std::vector<float> &p_u, std::vector<float> &p_v) {
	for (int32_t y = 0; y < height; y++) {
		for (int32_t x = 0; x <= width; x++) {
			if (u_face_blocked(x, y)) {
				p_u[u_index(x, y)] = 0.0f;
			}
		}
	}
	for (int32_t y = 0; y <= height; y++) {
		for (int32_t x = 0; x < width; x++) {
			if (v_face_blocked(x, y)) {
				p_v[v_index(x, y)] = 0.0f;
			}
		}
	}
}

void MacFluidSolver::apply_solid_boundaries_active(std::vector<float> &p_u, std::vector<float> &p_v) {
	if (!has_active_liquid) {
		return;
	}
	for (int32_t idx : active_u_faces) {
		const int32_t y = idx / (width + 1);
		const int32_t x = idx - y * (width + 1);
		if (u_face_blocked(x, y)) {
			p_u[idx] = 0.0f;
		}
	}
	for (int32_t idx : active_v_faces) {
		const int32_t y = idx / width;
		const int32_t x = idx - y * width;
		if (v_face_blocked(x, y)) {
			p_v[idx] = 0.0f;
		}
	}
}

float MacFluidSolver::sample_u_clamped(const std::vector<float> &p_u, int32_t p_x, int32_t p_y) const {
	p_x = std::max(0, std::min(p_x, width));
	p_y = std::max(0, std::min(p_y, height - 1));
	return p_u[u_index(p_x, p_y)];
}

float MacFluidSolver::sample_v_clamped(const std::vector<float> &p_v, int32_t p_x, int32_t p_y) const {
	p_x = std::max(0, std::min(p_x, width - 1));
	p_y = std::max(0, std::min(p_y, height));
	return p_v[v_index(p_x, p_y)];
}

float MacFluidSolver::sample_pressure_volume_at(float p_world_x, float p_world_y) const {
	// Cell-centered bilinear sample.  Unlike velocity sampling, samples outside
	// the grid or inside non-pressure materials contribute 0 instead of clamping
	// to the boundary, because outside/solid is not an incoming liquid source.
	const float fx = p_world_x - 0.5f;
	const float fy = p_world_y - 0.5f;
	const int32_t x0 = static_cast<int32_t>(std::floor(fx));
	const int32_t y0 = static_cast<int32_t>(std::floor(fy));
	const float tx = fx - static_cast<float>(x0);
	const float ty = fy - static_cast<float>(y0);

	auto sample = [&](int32_t sx, int32_t sy) -> float {
		if (!in_bounds(sx, sy)) {
			return 0.0f;
		}
		const int32_t si = cell_index(sx, sy);
		if (world->rigid_body_id[si] != 0) {
			return 0.0f;
		}
		const MaterialDef &def = get_material_def(world->material[si]);
		if (!def.pressure_solved || def.blocks_velocity || !std::isfinite(world->volume_fraction[si])) {
			return 0.0f;
		}
		return std::max(0.0f, world->volume_fraction[si]);
	};

	const float a = sample(x0, y0);
	const float b = sample(x0 + 1, y0);
	const float c = sample(x0, y0 + 1);
	const float d = sample(x0 + 1, y0 + 1);
	const float ab = a + (b - a) * tx;
	const float cd = c + (d - c) * tx;
	return ab + (cd - ab) * ty;
}

float MacFluidSolver::backtraced_pressure_volume(int32_t p_x, int32_t p_y) const {
	if (!in_bounds(p_x, p_y)) {
		return 0.0f;
	}
	// Use the same simulation dt as the velocity advection step.  If pressure
	// projection is later split into its own 0.05s cadence/substep, this
	// displacement must use that substep's effective dt instead.
	const float cx = static_cast<float>(p_x) + 0.5f;
	const float cy = static_cast<float>(p_y) + 0.5f;
	const float vx = 0.5f * (u_tmp[u_index(p_x, p_y)] + u_tmp[u_index(p_x + 1, p_y)]);
	const float vy = 0.5f * (v_tmp[v_index(p_x, p_y)] + v_tmp[v_index(p_x, p_y + 1)]);
	if (!std::isfinite(vx) || !std::isfinite(vy)) {
		return 0.0f;
	}
	return sample_pressure_volume_at(cx - dt * vx, cy - dt * vy);
}

float MacFluidSolver::sample_u_bilinear(const std::vector<float> &p_u, float p_world_x, float p_world_y) const {
	// u samples live on vertical faces at world positions (x, y + 0.5).
	float fx = clampf(p_world_x, 0.0f, static_cast<float>(width));
	float fy = clampf(p_world_y - 0.5f, 0.0f, static_cast<float>(height - 1));
	const int32_t x0 = static_cast<int32_t>(std::floor(fx));
	const int32_t y0 = static_cast<int32_t>(std::floor(fy));
	const int32_t x1 = std::min(x0 + 1, width);
	const int32_t y1 = std::min(y0 + 1, height - 1);
	const float tx = fx - static_cast<float>(x0);
	const float ty = fy - static_cast<float>(y0);
	const float a = sample_u_clamped(p_u, x0, y0);
	const float b = sample_u_clamped(p_u, x1, y0);
	const float c = sample_u_clamped(p_u, x0, y1);
	const float d = sample_u_clamped(p_u, x1, y1);
	const float ab = a + (b - a) * tx;
	const float cd = c + (d - c) * tx;
	return ab + (cd - ab) * ty;
}

float MacFluidSolver::sample_v_bilinear(const std::vector<float> &p_v, float p_world_x, float p_world_y) const {
	// v samples live on horizontal faces at world positions (x + 0.5, y).
	float fx = clampf(p_world_x - 0.5f, 0.0f, static_cast<float>(width - 1));
	float fy = clampf(p_world_y, 0.0f, static_cast<float>(height));
	const int32_t x0 = static_cast<int32_t>(std::floor(fx));
	const int32_t y0 = static_cast<int32_t>(std::floor(fy));
	const int32_t x1 = std::min(x0 + 1, width - 1);
	const int32_t y1 = std::min(y0 + 1, height);
	const float tx = fx - static_cast<float>(x0);
	const float ty = fy - static_cast<float>(y0);
	const float a = sample_v_clamped(p_v, x0, y0);
	const float b = sample_v_clamped(p_v, x1, y0);
	const float c = sample_v_clamped(p_v, x0, y1);
	const float d = sample_v_clamped(p_v, x1, y1);
	const float ab = a + (b - a) * tx;
	const float cd = c + (d - c) * tx;
	return ab + (cd - ab) * ty;
}

float MacFluidSolver::vertical_velocity_at_u_face(const std::vector<float> &p_v, int32_t p_x, int32_t p_y) const {
	// Average the four vertical faces touching the u-face sample point.
	return 0.25f * (
			sample_v_clamped(p_v, p_x - 1, p_y) +
			sample_v_clamped(p_v, p_x, p_y) +
			sample_v_clamped(p_v, p_x - 1, p_y + 1) +
			sample_v_clamped(p_v, p_x, p_y + 1));
}

float MacFluidSolver::horizontal_velocity_at_v_face(const std::vector<float> &p_u, int32_t p_x, int32_t p_y) const {
	// Average the four horizontal faces touching the v-face sample point.
	return 0.25f * (
			sample_u_clamped(p_u, p_x, p_y - 1) +
			sample_u_clamped(p_u, p_x + 1, p_y - 1) +
			sample_u_clamped(p_u, p_x, p_y) +
			sample_u_clamped(p_u, p_x + 1, p_y));
}

void MacFluidSolver::predict_velocity_explicit() {
	// Semi-Lagrangian velocity advection.  This is the numerical version of the
	// convective term (U · grad)U: trace each MAC face backward through the old
	// velocity field and copy the velocity carried by the fluid parcel that
	// arrives at this face.
	for (int32_t idx : active_u_faces) {
			const int32_t y = idx / (width + 1);
			const int32_t x = idx - y * (width + 1);
			if (u_face_blocked(x, y)) {
				u_advected[idx] = 0.0f;
				continue;
			}
			if (!is_liquid_cell(x - 1, y) && !is_liquid_cell(x, y)) {
				u_advected[idx] = 0.0f;
				continue;
			}
			const float face_x = static_cast<float>(x);
			const float face_y = static_cast<float>(y) + 0.5f;
			const float vel_x = u[idx];
			const float vel_y = vertical_velocity_at_u_face(v, x, y);
			u_advected[idx] = sample_u_bilinear(u, face_x - dt * vel_x, face_y - dt * vel_y);
	}

	for (int32_t idx : active_v_faces) {
			const int32_t y = idx / width;
			const int32_t x = idx - y * width;
			if (v_face_blocked(x, y)) {
				v_advected[idx] = 0.0f;
				continue;
			}
			if (!is_liquid_cell(x, y - 1) && !is_liquid_cell(x, y)) {
				v_advected[idx] = 0.0f;
				continue;
			}
			const float face_x = static_cast<float>(x) + 0.5f;
			const float face_y = static_cast<float>(y);
			const float vel_x = horizontal_velocity_at_v_face(u, x, y);
			const float vel_y = v[idx];
			v_advected[idx] = sample_v_bilinear(v, face_x - dt * vel_x, face_y - dt * vel_y);
	}

	apply_solid_boundaries_active(u_advected, v_advected);

	for (int32_t idx : active_u_faces) {
			const int32_t y = idx / (width + 1);
			const int32_t x = idx - y * (width + 1);
			if (x <= 0 || x >= width) {
				u_tmp[idx] = 0.0f;
				continue;
			}
			if (!is_liquid_cell(x - 1, y) && !is_liquid_cell(x, y)) {
				u_tmp[idx] = 0.0f;
				continue;
			}
			float u0 = u_advected[idx];
			float lap = sample_u_clamped(u_advected, x + 1, y) + sample_u_clamped(u_advected, x - 1, y) +
					sample_u_clamped(u_advected, x, y + 1) + sample_u_clamped(u_advected, x, y - 1) - 4.0f * u0;
			u_tmp[idx] = u0 + dt * (viscosity * lap);
	}

	for (int32_t idx : active_v_faces) {
			const int32_t y = idx / width;
			const int32_t x = idx - y * width;
			if (y <= 0 || y >= height) {
				v_tmp[idx] = 0.0f;
				continue;
			}
			if (!is_liquid_cell(x, y - 1) && !is_liquid_cell(x, y)) {
				v_tmp[idx] = 0.0f;
				continue;
			}
			float v0 = v_advected[idx];
			float lap = sample_v_clamped(v_advected, x + 1, y) + sample_v_clamped(v_advected, x - 1, y) +
					sample_v_clamped(v_advected, x, y + 1) + sample_v_clamped(v_advected, x, y - 1) - 4.0f * v0;
			v_tmp[idx] = v0 + dt * (viscosity * lap + gravity);
	}

	apply_solid_boundaries_active(u_tmp, v_tmp);
}

void MacFluidSolver::build_pressure_system() {
	active_cells.clear();

	for (int32_t i : active_region_indices) {
		const int32_t x = i % width;
		const int32_t y = i / width;
		world->pressure[i] = 0.0f;
		if (is_pressure_active_cell(x, y)) {
			pressure_active_mask[i] = 1;
			active_cells.push_back(i);
		}
	}

	for (int32_t idx : active_u_faces) {
		const int32_t y = idx / (width + 1);
		const int32_t x = idx - y * (width + 1);
		u_alpha[idx] = u_face_alpha(x, y);
	}
	for (int32_t idx : active_v_faces) {
		const int32_t y = idx / width;
		const int32_t x = idx - y * width;
		v_alpha[idx] = v_face_alpha(x, y);
	}

	int32_t write_count = 0;
	for (int32_t read_pos = 0; read_pos < static_cast<int32_t>(active_cells.size()); read_pos++) {
			int32_t i = active_cells[read_pos];
			int32_t x = i % width;
			int32_t y = i / width;

			float div = (u_tmp[u_index(x + 1, y)] - u_tmp[u_index(x, y)] +
					v_tmp[v_index(x, y + 1)] - v_tmp[v_index(x, y)]);

			float diag = u_alpha[u_index(x, y)] + u_alpha[u_index(x + 1, y)] + v_alpha[v_index(x, y)] + v_alpha[v_index(x, y + 1)];
			if (diag <= 0.0f) {
				pressure_active_mask[i] = 0;
				continue;
			}
			active_cells[write_count++] = i;
			diag_inv[i] = 1.0f / diag;

			// Density/volume correction target:
			// div(U_new) = s.
			// - over-full cells ask for positive divergence, pushing volume out.
			// - internal under-filled cells ask for weak negative divergence,
			//   pulling volume in.  Free-surface cells do NOT get this underfill
			//   term, otherwise the surface would constantly suck in water.
			const float pressure_fill_fraction = std::max(world->volume_fraction[i], backtraced_pressure_volume(x, y));
			float excess = std::max(0.0f, pressure_fill_fraction - target_fill_fraction);
			float deficit = std::max(0.0f, target_fill_fraction - world->volume_fraction[i]);
			float s = density_correction_strength * excess / std::max(dt, 1.0e-5f);
			if (deficit > 0.0f && is_internal_liquid_cell(x, y)) {
				s -= underfill_correction_strength * deficit / std::max(dt, 1.0e-5f);
			}

			// With the projection U_new = U* - alpha * grad(p), and with our
			// discrete operator A(p)=sum alpha_f*(p_C-p_N), the consistent RHS is:
			// A p = s - div(U*).
			rhs[i] = s - div;
	}
	active_cells.resize(write_count);

	// Pack the pressure matrix stencil once per simulation step.  During PCG,
	// alpha values and active-neighbor topology are constant, so apply_laplian()
	// can avoid integer division, face-index recomputation, bounds checks, and
	// active-mask branches in its hottest loop.
	pressure_rows.clear();
	pressure_rows.reserve(active_cells.size());
	for (int32_t i : active_cells) {
		const int32_t x = i % width;
		const int32_t y = i / width;
		const float a_left = u_alpha[u_index(x, y)];
		const float a_right = u_alpha[u_index(x + 1, y)];
		const float a_up = v_alpha[v_index(x, y)];
		const float a_down = v_alpha[v_index(x, y + 1)];

		PressureStencilRow row;
		row.self = i;
		row.diag = a_left + a_right + a_up + a_down;

		const int32_t left = i - 1;
		const int32_t right = i + 1;
		const int32_t up = i - width;
		const int32_t down = i + width;

		row.nbr[0] = (x > 0 && pressure_active_mask[left] != 0) ? left : i;
		row.nbr[1] = (x + 1 < width && pressure_active_mask[right] != 0) ? right : i;
		row.nbr[2] = (y > 0 && pressure_active_mask[up] != 0) ? up : i;
		row.nbr[3] = (y + 1 < height && pressure_active_mask[down] != 0) ? down : i;

		row.offdiag[0] = row.nbr[0] != i ? a_left : 0.0f;
		row.offdiag[1] = row.nbr[1] != i ? a_right : 0.0f;
		row.offdiag[2] = row.nbr[2] != i ? a_up : 0.0f;
		row.offdiag[3] = row.nbr[3] != i ? a_down : 0.0f;

		pressure_rows.push_back(row);
	}
}

void MacFluidSolver::apply_laplacian(const std::vector<float> &p_x, std::vector<float> &r_ax) const {
	for (const PressureStencilRow &row : pressure_rows) {
			const float p0 = p_x[row.self];
			float ax = row.diag * p0;
			ax -= row.offdiag[0] * p_x[row.nbr[0]];
			ax -= row.offdiag[1] * p_x[row.nbr[1]];
			ax -= row.offdiag[2] * p_x[row.nbr[2]];
			ax -= row.offdiag[3] * p_x[row.nbr[3]];
			r_ax[row.self] = ax;
	}
}

void MacFluidSolver::build_mic_preconditioner() {
	const int32_t row_count = static_cast<int32_t>(pressure_rows.size());
	std::fill(pressure_row_index.begin(), pressure_row_index.end(), -1);
	mic_rows.assign(row_count, MicPreconditionerRow());
	mic_diag.assign(row_count, 1.0f);
	mic_temp.assign(row_count, 0.0f);

	for (int32_t r = 0; r < row_count; r++) {
		pressure_row_index[pressure_rows[r].self] = r;
		mic_rows[r].self = pressure_rows[r].self;
	}

	// IC(0)/MIC-style LDL^T factorization with the same left/up sparsity as the
	// pressure matrix. This is rebuilt once per simulation step and then reused
	// by every PCG iteration in that step.  It is serial, but much stronger than
	// Jacobi for large connected water regions.
	for (int32_t r = 0; r < row_count; r++) {
		const PressureStencilRow &row = pressure_rows[r];
		float d = row.diag;
		int32_t lower_count = 0;

		// Lower-neighbor directions in our scan order are left and up.
		const int32_t lower_dirs[2] = { 0, 2 };
		for (int32_t dir : lower_dirs) {
			if (row.offdiag[dir] <= 0.0f || row.nbr[dir] == row.self) {
				continue;
			}
			const int32_t lower_row = pressure_row_index[row.nbr[dir]];
			if (lower_row < 0 || lower_row >= r) {
				continue;
			}
			const float d_lower = std::max(mic_diag[lower_row], 1.0e-8f);
			const float l = -row.offdiag[dir] / d_lower;
			if (lower_count < 2) {
				mic_rows[r].lower_row[lower_count] = lower_row;
				mic_rows[r].lower_l[lower_count] = l;
				lower_count++;
			}
			d -= l * l * d_lower;
		}

		// Robustness guard: incomplete factors can break down when the active
		// free-surface topology is thin/irregular. Falling back toward Jacobi for
		// this row keeps the preconditioner SPD.
		const float min_d = std::max(1.0e-6f, 0.25f * std::max(row.diag, 1.0e-6f));
		if (!std::isfinite(d) || d < min_d) {
			d = std::max(row.diag, 1.0e-6f);
			mic_rows[r].lower_row[0] = -1;
			mic_rows[r].lower_row[1] = -1;
			mic_rows[r].lower_l[0] = 0.0f;
			mic_rows[r].lower_l[1] = 0.0f;
		}
		mic_diag[r] = d;
		mic_rows[r].d_inv = 1.0f / d;
	}

	for (int32_t r = 0; r < row_count; r++) {
		for (int32_t k = 0; k < 2; k++) {
			const int32_t lower = mic_rows[r].lower_row[k];
			if (lower < 0) {
				continue;
			}
			for (int32_t slot = 0; slot < 2; slot++) {
				if (mic_rows[lower].upper_row[slot] < 0) {
					mic_rows[lower].upper_row[slot] = r;
					mic_rows[lower].upper_l[slot] = mic_rows[r].lower_l[k];
					break;
				}
			}
		}
	}
}

void MacFluidSolver::apply_preconditioner(const std::vector<float> &p_r, std::vector<float> &r_z) {
	const int32_t row_count = static_cast<int32_t>(mic_rows.size());
	if (row_count == 0) {
		return;
	}

	// Forward solve: L y = r, then diagonal solve y = D^-1 y.
	for (int32_t r = 0; r < row_count; r++) {
		const MicPreconditionerRow &row = mic_rows[r];
		float y = p_r[row.self];
		for (int32_t k = 0; k < 2; k++) {
			const int32_t lower = row.lower_row[k];
			if (lower >= 0) {
				y -= row.lower_l[k] * mic_temp[lower];
			}
		}
		mic_temp[r] = y * row.d_inv;
	}

	// Backward solve: L^T z = y.
	for (int32_t r = row_count - 1; r >= 0; r--) {
		const MicPreconditionerRow &row = mic_rows[r];
		float z = mic_temp[r];
		for (int32_t k = 0; k < 2; k++) {
			const int32_t upper = row.upper_row[k];
			if (upper >= 0) {
				z -= row.upper_l[k] * r_z[mic_rows[upper].self];
			}
		}
		r_z[row.self] = z;
	}
}

double MacFluidSolver::dot_liquid(const std::vector<float> &p_a, const std::vector<float> &p_b) const {
	double sum = 0.0;
	for (int32_t i : active_cells) {
		sum += static_cast<double>(p_a[i]) * static_cast<double>(p_b[i]);
	}
	return sum;
}

void MacFluidSolver::solve_pressure_pcg() {
	apply_laplacian(world->pressure, q_vec);
	for (int32_t i : active_cells) {
		residual[i] = rhs[i] - q_vec[i];
		z_vec[i] = residual[i] * diag_inv[i];
		direction[i] = z_vec[i];
	}

	double rz_old = dot_liquid(residual, z_vec);
	double b_norm = std::sqrt(std::max(dot_liquid(rhs, rhs), 1.0e-30));
	last_pcg_iterations = 0;
	last_pcg_residual = std::sqrt(std::max(dot_liquid(residual, residual), 0.0)) / b_norm;
	if (rz_old <= 1.0e-30) {
		return;
	}

	for (int32_t iter = 0; iter < pressure_iterations; iter++) {
		apply_laplacian(direction, q_vec);
		double denom = dot_liquid(direction, q_vec);
		if (std::abs(denom) < 1.0e-30) {
			break;
		}
		double alpha = rz_old / denom;
		for (int32_t i : active_cells) {
			world->pressure[i] += static_cast<float>(alpha) * direction[i];
			residual[i] -= static_cast<float>(alpha) * q_vec[i];
		}

		double rel_res = std::sqrt(std::max(dot_liquid(residual, residual), 0.0)) / b_norm;
		last_pcg_iterations = iter + 1;
		last_pcg_residual = rel_res;
		if (rel_res < PCG_EPSILON) {
			break;
		}

		for (int32_t i : active_cells) {
			z_vec[i] = residual[i] * diag_inv[i];
		}
		double rz_new = dot_liquid(residual, z_vec);
		double beta = rz_new / rz_old;
		for (int32_t i : active_cells) {
			direction[i] = z_vec[i] + static_cast<float>(beta) * direction[i];
		}
		rz_old = rz_new;
	}
}

void MacFluidSolver::apply_pressure_projection() {
	for (int32_t idx : active_u_faces) {
			const int32_t y = idx / (width + 1);
			const int32_t x = idx - y * (width + 1);
			u[idx] = u_tmp[idx];
			if (u_face_blocked(x, y)) {
				u[idx] = 0.0f;
				continue;
			}
			bool left_liquid = is_pressure_active_masked(x - 1, y);
			bool right_liquid = is_pressure_active_masked(x, y);
			if (!left_liquid && !right_liquid) {
				continue;
			}
			float p_left = left_liquid ? world->pressure[cell_index(x - 1, y)] : 0.0f;
			float p_right = right_liquid ? world->pressure[cell_index(x, y)] : 0.0f;
			u[idx] -= u_alpha[idx] * (p_right - p_left);
	}

	for (int32_t idx : active_v_faces) {
			const int32_t y = idx / width;
			const int32_t x = idx - y * width;
			v[idx] = v_tmp[idx];
			if (v_face_blocked(x, y)) {
				v[idx] = 0.0f;
				continue;
			}
			bool up_liquid = is_pressure_active_masked(x, y - 1);
			bool down_liquid = is_pressure_active_masked(x, y);
			if (!up_liquid && !down_liquid) {
				continue;
			}
			float p_up = up_liquid ? world->pressure[cell_index(x, y - 1)] : 0.0f;
			float p_down = down_liquid ? world->pressure[cell_index(x, y)] : 0.0f;
			v[idx] -= v_alpha[idx] * (p_down - p_up);
	}

	apply_solid_boundaries_active(u, v);
}

void MacFluidSolver::advect_mass_finite_volume() {
	const int32_t cell_count = width * height;
	for (int32_t i : active_region_indices) {
		next_mass[i] = 0.0f;
		next_toxic[i] = 0.0f;
		next_oil[i] = 0.0f;
		outflow_mass[i] = 0.0f;
		if (get_material_def(world->material[i]).liquid && std::isfinite(world->volume_fraction[i]) && world->volume_fraction[i] > 0.0f) {
			next_mass[i] = world->volume_fraction[i];
			next_toxic[i] = clampf(world->toxic[i], 0.0f, world->volume_fraction[i]);
			next_oil[i] = clampf(world->oil[i], 0.0f, world->volume_fraction[i]);
		}
	}

	// First pass: raw volume fluxes and total outgoing volume per donor.
	for (int32_t face_idx : active_u_faces) {
			const int32_t y = face_idx / (width + 1);
			const int32_t x = face_idx - y * (width + 1);
			if (x <= 0 || x >= width) {
				u_mass_flux[face_idx] = 0.0f;
				continue;
			}
			if (u_face_blocked(x, y)) {
				continue;
			}
			const int32_t li = cell_index(x - 1, y);
			const int32_t ri = cell_index(x, y);
			if (active_region_mask[li] == 0 || active_region_mask[ri] == 0) {
				continue;
			}
			const float vel = u[u_index(x, y)];
			float donor_volume = 0.0f;
			if (vel > 0.0f && get_material_def(world->material[li]).liquid) {
				donor_volume = std::max(0.0f, world->volume_fraction[li]);
			} else if (vel < 0.0f && get_material_def(world->material[ri]).liquid) {
				donor_volume = std::max(0.0f, world->volume_fraction[ri]);
			}
			float flux = vel * dt * donor_volume;
			if (!std::isfinite(flux)) {
				flux = 0.0f;
			}
			u_mass_flux[face_idx] = flux;
			if (flux > 0.0f) {
				outflow_mass[li] += flux;
			} else if (flux < 0.0f) {
				outflow_mass[ri] += -flux;
			}
	}

	for (int32_t face_idx : active_v_faces) {
			const int32_t y = face_idx / width;
			const int32_t x = face_idx - y * width;
			if (y <= 0 || y >= height) {
				v_mass_flux[face_idx] = 0.0f;
				continue;
			}
			if (v_face_blocked(x, y)) {
				continue;
			}
			const int32_t ui = cell_index(x, y - 1);
			const int32_t di = cell_index(x, y);
			if (active_region_mask[ui] == 0 || active_region_mask[di] == 0) {
				continue;
			}
			const float vel = v[v_index(x, y)];
			float donor_volume = 0.0f;
			if (vel > 0.0f && get_material_def(world->material[ui]).liquid) {
				donor_volume = std::max(0.0f, world->volume_fraction[ui]);
			} else if (vel < 0.0f && get_material_def(world->material[di]).liquid) {
				donor_volume = std::max(0.0f, world->volume_fraction[di]);
			}
			float flux = vel * dt * donor_volume;
			if (!std::isfinite(flux)) {
				flux = 0.0f;
			}
			v_mass_flux[face_idx] = flux;
			if (flux > 0.0f) {
				outflow_mass[ui] += flux;
			} else if (flux < 0.0f) {
				outflow_mass[di] += -flux;
			}
	}

	auto scale_flux_for_donor = [&](float flux, int32_t donor) -> float {
		const float raw_out = outflow_mass[donor];
		const float available = std::max(0.0f, world->volume_fraction[donor]);
		if (raw_out > available && raw_out > MASS_EPSILON) {
			return flux * available / raw_out;
		}
		return flux;
	};

	auto face_oil_fraction = [&](int32_t donor, int32_t receiver, float scaled_flux_abs) -> float {
		const float donor_volume = std::max(world->volume_fraction[donor], MASS_EPSILON);
		const float donor_oil = clampf(world->oil[donor], 0.0f, donor_volume);
		const float donor_water = std::max(0.0f, donor_volume - donor_oil);
		const float donor_f = donor_oil / donor_volume;

		float receiver_volume = 0.0f;
		float receiver_oil = 0.0f;
		if (receiver >= 0 && receiver < cell_count && get_material_def(world->material[receiver]).liquid && world->volume_fraction[receiver] > MASS_EPSILON) {
			receiver_volume = std::max(0.0f, world->volume_fraction[receiver]);
			receiver_oil = clampf(world->oil[receiver], 0.0f, receiver_volume);
		}

		// Direct ratio rule: do not change the face velocity.  The already-computed
		// volume flux is only split into oil/water by the two cells touching this
		// face, i.e. phi_face = (oil_A + oil_B) / (volume_A + volume_B).
		float f = (donor_oil + receiver_oil) / std::max(donor_volume + receiver_volume, MASS_EPSILON);


		// Component-availability bounds.  Across all outgoing faces of this donor,
		// oil cannot exceed donor_oil and water cannot exceed donor_water.
		const float donor_scaled_out = std::max(MASS_EPSILON, std::min(outflow_mass[donor], donor_volume));
		const float lower = clampf((donor_scaled_out - donor_water) / donor_scaled_out, 0.0f, 1.0f);
		const float upper = clampf(donor_oil / donor_scaled_out, 0.0f, 1.0f);
		if (scaled_flux_abs <= MASS_EPSILON) {
			return donor_f;
		}
		return clampf(f, lower, upper);
	};

	auto apply_signed_flux = [&](int32_t from, int32_t to, float flux, int32_t dy) {
		if (std::abs(flux) <= 0.0f) {
			return;
		}
		const float abs_flux = std::abs(flux);
		const float oil_f = face_oil_fraction(from, to, abs_flux);
		const float oil_flux = abs_flux * oil_f;
		float toxic_flux = 0.0f;
		if (world->volume_fraction[from] > MASS_EPSILON) {
			toxic_flux = abs_flux * clampf(world->toxic[from] / world->volume_fraction[from], 0.0f, 1.0f);
		}
		next_mass[from] -= abs_flux;
		next_mass[to] += abs_flux;
		next_oil[from] -= oil_flux;
		next_oil[to] += oil_flux;
		next_toxic[from] -= toxic_flux;
		next_toxic[to] += toxic_flux;
	};

	// Second pass: scale volume fluxes per donor, then split each conserved volume
	// flux into oil/water components with the local immiscible bias.
	for (int32_t face_idx : active_u_faces) {
			const int32_t y = face_idx / (width + 1);
			const int32_t x = face_idx - y * (width + 1);
			if (x <= 0 || x >= width) {
				continue;
			}
			const int32_t li = cell_index(x - 1, y);
			const int32_t ri = cell_index(x, y);
			if (active_region_mask[li] == 0 || active_region_mask[ri] == 0) {
				continue;
			}
			float flux = u_mass_flux[face_idx];
			if (flux > 0.0f) {
				flux = scale_flux_for_donor(flux, li);
				apply_signed_flux(li, ri, flux, 0);
			} else if (flux < 0.0f) {
				flux = scale_flux_for_donor(flux, ri);
				apply_signed_flux(ri, li, flux, 0);
			}
	}

	for (int32_t face_idx : active_v_faces) {
			const int32_t y = face_idx / width;
			const int32_t x = face_idx - y * width;
			if (y <= 0 || y >= height) {
				continue;
			}
			const int32_t ui = cell_index(x, y - 1);
			const int32_t di = cell_index(x, y);
			if (active_region_mask[ui] == 0 || active_region_mask[di] == 0) {
				continue;
			}
			float flux = v_mass_flux[face_idx];
			if (flux > 0.0f) {
				flux = scale_flux_for_donor(flux, ui);
				apply_signed_flux(ui, di, flux, 1);
			} else if (flux < 0.0f) {
				flux = scale_flux_for_donor(flux, di);
				apply_signed_flux(di, ui, flux, -1);
			}
	}

	for (int32_t i : active_region_indices) {
			const int32_t x = i % width;
			const int32_t y = i / width;
			const MaterialDef &def = get_material_def(world->material[i]);
			if (!std::isfinite(next_mass[i]) || next_mass[i] < MASS_EPSILON) {
				if (def.liquid) {
					world->volume_fraction[i] = 0.0f;
					world->density[i] = get_material_def(MATERIAL_AIR).density;
					world->toxic[i] = 0.0f;
					world->oil[i] = 0.0f;
					world->material[i] = MATERIAL_AIR;
				}
			} else if (!def.blocks_velocity) {
				world->volume_fraction[i] = std::max(0.0f, next_mass[i]);
				world->oil[i] = clampf(next_oil[i], 0.0f, world->volume_fraction[i]);
				world->toxic[i] = clampf(next_toxic[i], 0.0f, world->volume_fraction[i]);
				const float oil_f = oil_fraction_at(i);
				const float toxic_concentration = world->toxic[i] / std::max(world->volume_fraction[i], MASS_EPSILON);
				world->density[i] = cell_density(x, y);
				if (toxic_concentration >= 0.25f && oil_f < 0.50f) {
					world->material[i] = MATERIAL_TOXIC;
				} else {
					world->material[i] = oil_f >= 0.50f ? MATERIAL_OIL : MATERIAL_WATER;
				}
			}
	}
}
void MacFluidSolver::clamp_velocities() {
	for (int32_t idx : active_u_faces) {
		float &value = u[idx];
		value = clampf(value * velocity_damping, -max_velocity, max_velocity);
	}
	for (int32_t idx : active_v_faces) {
		float &value = v[idx];
		value = clampf(value * velocity_damping, -max_velocity, max_velocity);
	}
	apply_solid_boundaries_active(u, v);
	update_world_velocity_field();
}

void MacFluidSolver::update_world_velocity_field() {
	if (world == nullptr) {
		return;
	}
	for (int32_t i : active_region_indices) {
		const int32_t x = i % width;
		const int32_t y = i / width;
		world->velocity_x[i] = 0.5f * (u[u_index(x, y)] + u[u_index(x + 1, y)]);
		world->velocity_y[i] = 0.5f * (v[v_index(x, y)] + v[v_index(x, y + 1)]);
	}
}

bool MacFluidSolver::step(double p_budget_ms) {
	if (!step_job_active) {
		begin_step_job();
	}
	if (!step_job_active) {
		return true;
	}
	return advance_step_job(p_budget_ms);
}

void MacFluidSolver::begin_step_job() {
	if (world == nullptr) {
		return;
	}
	if (step_job_active) {
		return;
	}
	if (world->volume_fraction.empty()) {
		set_world_size(width, height);
	}
	step_job_start_time = std::chrono::high_resolution_clock::now();
	last_predict_ms = 0.0;
	last_build_ms = 0.0;
	last_pcg_ms = 0.0;
	last_project_ms = 0.0;
	last_advect_ms = 0.0;
	last_clamp_ms = 0.0;
	last_pcg_iterations = 0;
	last_pcg_residual = 0.0;

	if (!compute_active_liquid_region()) {
		if (had_active_liquid_last_step) {
			std::fill(u.begin(), u.end(), 0.0f);
			std::fill(v.begin(), v.end(), 0.0f);
			std::fill(u_tmp.begin(), u_tmp.end(), 0.0f);
			std::fill(v_tmp.begin(), v_tmp.end(), 0.0f);
			std::fill(u_advected.begin(), u_advected.end(), 0.0f);
			std::fill(v_advected.begin(), v_advected.end(), 0.0f);
			std::fill(world->velocity_x.begin(), world->velocity_x.end(), 0.0f);
			std::fill(world->velocity_y.begin(), world->velocity_y.end(), 0.0f);
		}
		active_cells.clear();
		pressure_rows.clear();
		had_active_liquid_last_step = false;
		step_count++;
		auto t1 = std::chrono::high_resolution_clock::now();
		last_step_ms = elapsed_ms(step_job_start_time, t1);
		step_phase = StepPhase::IDLE;
		step_job_active = false;
		return;
	}

	step_phase = StepPhase::PREDICT;
	step_job_active = true;
}

bool MacFluidSolver::advance_step_job(double p_budget_ms) {
	if (!step_job_active) {
		return true;
	}
	const auto budget_start = std::chrono::high_resolution_clock::now();
	const double budget_ms = std::max(0.0, p_budget_ms);
	while (step_job_active) {
		const auto phase_start = std::chrono::high_resolution_clock::now();
		switch (step_phase) {
			case StepPhase::PREDICT:
				predict_velocity_explicit();
				last_predict_ms += elapsed_ms(phase_start, std::chrono::high_resolution_clock::now());
				step_phase = StepPhase::BUILD;
				break;
			case StepPhase::BUILD:
				build_pressure_system();
				last_build_ms += elapsed_ms(phase_start, std::chrono::high_resolution_clock::now());
				step_phase = StepPhase::PCG;
				break;
			case StepPhase::PCG:
				solve_pressure_pcg();
				last_pcg_ms += elapsed_ms(phase_start, std::chrono::high_resolution_clock::now());
				step_phase = StepPhase::PROJECT;
				break;
			case StepPhase::PROJECT:
				apply_pressure_projection();
				last_project_ms += elapsed_ms(phase_start, std::chrono::high_resolution_clock::now());
				step_phase = StepPhase::ADVECT;
				break;
			case StepPhase::ADVECT:
				advect_mass_finite_volume();
				last_advect_ms += elapsed_ms(phase_start, std::chrono::high_resolution_clock::now());
				step_phase = StepPhase::CLAMP;
				break;
			case StepPhase::CLAMP:
				clamp_velocities();
				last_clamp_ms += elapsed_ms(phase_start, std::chrono::high_resolution_clock::now());
				had_active_liquid_last_step = true;
				step_count++;
				last_step_ms = elapsed_ms(step_job_start_time, std::chrono::high_resolution_clock::now());
				step_phase = StepPhase::IDLE;
				step_job_active = false;
				return true;
			case StepPhase::IDLE:
			default:
				step_job_active = false;
				return true;
		}
		const auto now = std::chrono::high_resolution_clock::now();
		if (elapsed_ms(budget_start, now) >= budget_ms) {
			break;
		}
	}
	return !step_job_active;
}

bool MacFluidSolver::has_pending_step_job() const {
	return step_job_active;
}

void MacFluidSolver::set_world_size(int32_t p_width, int32_t p_height) {
	if (world == nullptr) {
		width = std::max(16, p_width);
		height = std::max(16, p_height);
		return;
	}
	world->set_size(p_width, p_height);
	width = world->width;
	height = world->height;
	ensure_buffers();
	clear_fields();
	step_count = 0;
}

int32_t MacFluidSolver::get_width() const { return width; }
int32_t MacFluidSolver::get_height() const { return height; }
void MacFluidSolver::set_dt(double p_dt) { dt = clampf(static_cast<float>(p_dt), 1.0f / 240.0f, 0.25f); }
double MacFluidSolver::get_dt() const { return dt; }
void MacFluidSolver::set_gravity(double p_gravity) { gravity = clampf(static_cast<float>(p_gravity), -1000.0f, 1000.0f); }
double MacFluidSolver::get_gravity() const { return gravity; }
void MacFluidSolver::set_viscosity(double p_viscosity) { viscosity = clampf(static_cast<float>(p_viscosity), 0.0f, 1.0f); }
double MacFluidSolver::get_viscosity() const { return viscosity; }
void MacFluidSolver::set_pressure_iterations(int32_t p_iterations) { pressure_iterations = std::max(1, std::min(p_iterations, 400)); }
int32_t MacFluidSolver::get_pressure_iterations() const { return pressure_iterations; }
void MacFluidSolver::set_pressure_active_mass(double p_mass) { pressure_active_mass = clampf(static_cast<float>(p_mass), MASS_EPSILON, 1.0f); }
double MacFluidSolver::get_pressure_active_mass() const { return pressure_active_mass; }
void MacFluidSolver::set_density_correction_strength(double p_strength) { density_correction_strength = clampf(static_cast<float>(p_strength), 0.0f, 2.0f); }
double MacFluidSolver::get_density_correction_strength() const { return density_correction_strength; }
void MacFluidSolver::set_underfill_correction_strength(double p_strength) { underfill_correction_strength = clampf(static_cast<float>(p_strength), 0.0f, 2.0f); }
double MacFluidSolver::get_underfill_correction_strength() const { return underfill_correction_strength; }

void MacFluidSolver::make_rock(int32_t p_x, int32_t p_y) {
	if (!in_bounds(p_x, p_y)) {
		return;
	}
	int32_t i = cell_index(p_x, p_y);
	world->material[i] = MATERIAL_ROCK;
	world->volume_fraction[i] = 0.0f;
	world->density[i] = get_material_def(MATERIAL_ROCK).density;
	world->toxic[i] = 0.0f;
	world->oil[i] = 0.0f;
	world->pressure[i] = 0.0f;
}

void MacFluidSolver::make_air(int32_t p_x, int32_t p_y) {
	if (!in_bounds(p_x, p_y)) {
		return;
	}
	int32_t i = cell_index(p_x, p_y);
	world->material[i] = MATERIAL_AIR;
	world->volume_fraction[i] = 0.0f;
	world->density[i] = get_material_def(MATERIAL_AIR).density;
	world->toxic[i] = 0.0f;
	world->oil[i] = 0.0f;
	world->pressure[i] = 0.0f;
}

void MacFluidSolver::make_water(int32_t p_x, int32_t p_y, float p_mass) {
	if (!in_bounds(p_x, p_y) || world->blocks_velocity(p_x, p_y)) {
		return;
	}
	int32_t i = cell_index(p_x, p_y);
	if (!get_material_def(world->material[i]).liquid) {
		world->toxic[i] = 0.0f;
		world->oil[i] = 0.0f;
	}
	world->volume_fraction[i] = clampf(world->volume_fraction[i] + p_mass, 0.0f, 1.0f);
	world->oil[i] = clampf(world->oil[i], 0.0f, world->volume_fraction[i]);
	world->toxic[i] = clampf(world->toxic[i], 0.0f, world->volume_fraction[i]);
	const float oil_f = oil_fraction_at(i);
	world->density[i] = get_material_def(MATERIAL_WATER).density * (1.0f - oil_f) + get_material_def(MATERIAL_OIL).density * oil_f;
	world->material[i] = oil_f >= 0.50f ? MATERIAL_OIL : MATERIAL_WATER;
}

void MacFluidSolver::make_oil(int32_t p_x, int32_t p_y, float p_mass) {
	if (!in_bounds(p_x, p_y) || world->blocks_velocity(p_x, p_y)) {
		return;
	}
	int32_t i = cell_index(p_x, p_y);
	if (!get_material_def(world->material[i]).liquid) {
		world->volume_fraction[i] = 0.0f;
		world->toxic[i] = 0.0f;
		world->oil[i] = 0.0f;
	}
	const float add_volume = clampf(p_mass, 0.0f, 1.0f);
	world->volume_fraction[i] = clampf(world->volume_fraction[i] + add_volume, 0.0f, 1.0f);
	world->oil[i] = clampf(world->oil[i] + add_volume, 0.0f, world->volume_fraction[i]);
	world->toxic[i] = clampf(world->toxic[i], 0.0f, world->volume_fraction[i]);
	const float oil_f = oil_fraction_at(i);
	world->density[i] = get_material_def(MATERIAL_WATER).density * (1.0f - oil_f) + get_material_def(MATERIAL_OIL).density * oil_f;
	world->material[i] = oil_f >= 0.50f ? MATERIAL_OIL : MATERIAL_WATER;
}

void MacFluidSolver::clear() {
	clear_fields();
}

void MacFluidSolver::generate_basin() {
	clear();
	for (int32_t x = 0; x < width; x++) {
		for (int32_t y = height - 7; y < height; y++) {
			make_rock(x, y);
		}
	}
	for (int32_t y = 0; y < height; y++) {
		for (int32_t t = 0; t < 4; t++) {
			make_rock(t, y);
			make_rock(width - 1 - t, y);
		}
	}
	int32_t cup_top = height / 3;
	int32_t cup_bottom = height - 8;
	for (int32_t y = cup_top; y < cup_bottom; y++) {
		float t = static_cast<float>(y - cup_top) / static_cast<float>(std::max(1, cup_bottom - cup_top));
		int32_t left_wall = static_cast<int32_t>(width * 0.16f + t * width * 0.15f + 3.0f * std::sin(y * 0.12f));
		int32_t right_wall = static_cast<int32_t>(width * 0.84f - t * width * 0.15f + 3.0f * std::sin(y * 0.10f + 2.0f));
		for (int32_t k = -2; k <= 2; k++) {
			make_rock(left_wall + k, y);
			make_rock(right_wall + k, y);
		}
	}
	for (int32_t x = 0; x < width; x++) {
		float curve = std::abs(static_cast<float>(x) / static_cast<float>(width) - 0.5f) * 2.0f;
		int32_t bed_y = static_cast<int32_t>(height * 0.74f + curve * curve * height * 0.13f + 3.0f * std::sin(x * 0.13f));
		for (int32_t y = bed_y; y < height; y++) {
			make_rock(x, y);
		}
	}
	int32_t blob_x0 = width / 2 - width / 9;
	int32_t blob_x1 = width / 2 + width / 9;
	int32_t blob_y0 = 12;
	int32_t blob_y1 = height / 3 - 10;
	for (int32_t y = blob_y0; y < blob_y1; y++) {
		for (int32_t x = blob_x0; x < blob_x1; x++) {
			float nx = (x - width / 2.0f) / (width / 9.5f);
			float ny = (y - (blob_y0 + blob_y1) * 0.5f) / ((blob_y1 - blob_y0) * 0.55f);
			if (nx * nx + ny * ny <= 1.0f) {
				make_water(x, y, 1.0f);
			}
		}
	}
	apply_solid_boundaries(u, v);
}

void MacFluidSolver::paint_circle(double p_x, double p_y, double p_radius, int32_t p_material) {
	int32_t cx = static_cast<int32_t>(std::floor(p_x));
	int32_t cy = static_cast<int32_t>(std::floor(p_y));
	int32_t r = std::max(1, static_cast<int32_t>(std::ceil(p_radius)));
	int32_t r2 = r * r;
	for (int32_t y = cy - r; y <= cy + r; y++) {
		for (int32_t x = cx - r; x <= cx + r; x++) {
			int32_t dx = x - cx;
			int32_t dy = y - cy;
			if (dx * dx + dy * dy > r2 || !in_bounds(x, y)) {
				continue;
			}
			if (p_material == MATERIAL_ROCK) {
				make_rock(x, y);
			} else if (p_material == MATERIAL_WATER) {
				make_water(x, y, 1.0f);
			} else if (p_material == MATERIAL_OIL) {
				make_oil(x, y, 1.0f);
			} else {
				make_air(x, y);
			}
		}
	}
	apply_solid_boundaries(u, v);
}

void MacFluidSolver::inject_water(double p_x, double p_y, double p_radius, double p_mass_per_cell, double p_velocity_x, double p_velocity_y) {
	int32_t cx = static_cast<int32_t>(std::floor(p_x));
	int32_t cy = static_cast<int32_t>(std::floor(p_y));
	int32_t r = std::max(1, static_cast<int32_t>(std::ceil(p_radius)));
	int32_t r2 = r * r;
	float add_mass = clampf(static_cast<float>(p_mass_per_cell), 0.0f, 1.0f);
	float vx = clampf(static_cast<float>(p_velocity_x), -max_velocity, max_velocity);
	float vy = clampf(static_cast<float>(p_velocity_y), -max_velocity, max_velocity);
	for (int32_t y = cy - r; y <= cy + r; y++) {
		for (int32_t x = cx - r; x <= cx + r; x++) {
			int32_t dx = x - cx;
			int32_t dy = y - cy;
			if (dx * dx + dy * dy > r2 || !in_bounds(x, y) || world->blocks_velocity(x, y)) {
				continue;
			}
			int32_t i = cell_index(x, y);
			if (!get_material_def(world->material[i]).liquid) {
				world->toxic[i] = 0.0f;
				world->oil[i] = 0.0f;
			}
			world->volume_fraction[i] = clampf(world->volume_fraction[i] + add_mass, 0.0f, 1.0f);
			world->oil[i] = clampf(world->oil[i], 0.0f, world->volume_fraction[i]);
			world->toxic[i] = clampf(world->toxic[i], 0.0f, world->volume_fraction[i]);
			const float oil_f = oil_fraction_at(i);
			world->density[i] = get_material_def(MATERIAL_WATER).density * (1.0f - oil_f) + get_material_def(MATERIAL_OIL).density * oil_f;
			world->material[i] = oil_f >= 0.50f ? MATERIAL_OIL : MATERIAL_WATER;
			if (x > 0) {
				u[u_index(x, y)] += vx;
			}
			if (x + 1 < width) {
				u[u_index(x + 1, y)] += vx;
			}
			if (y > 0) {
				v[v_index(x, y)] += vy;
			}
			if (y + 1 < height) {
				v[v_index(x, y + 1)] += vy;
			}
		}
	}
	apply_solid_boundaries(u, v);
}

void MacFluidSolver::fill_rgba_pixels(std::vector<uint8_t> &p_pixels) const {
	if (width <= 0 || height <= 0) {
		p_pixels.clear();
		return;
	}
	p_pixels.resize(static_cast<size_t>(width) * static_cast<size_t>(height) * 4);
	uint8_t *dst = p_pixels.data();
	for (int32_t y = 0; y < height; y++) {
		for (int32_t x = 0; x < width; x++) {
			int32_t i = cell_index(x, y);
			int32_t p = i * 4;
			uint8_t r = 5;
			uint8_t g = 6;
			uint8_t b = 9;
			const MaterialDef &def = get_material_def(world->material[i]);
			if (def.blocks_velocity) {
				uint8_t shade = static_cast<uint8_t>(70 + ((x * 17 + y * 9) & 35));
				if (world->material[i] == MATERIAL_SAND) {
					r = static_cast<uint8_t>(150 + ((x * 11 + y * 7) & 31));
					g = static_cast<uint8_t>(120 + ((x * 5 + y * 13) & 23));
					b = 55;
				} else if (world->material[i] == MATERIAL_GLASS) {
					r = static_cast<uint8_t>(105 + ((x * 3 + y * 11) & 23));
					g = static_cast<uint8_t>(150 + ((x * 7 + y * 5) & 25));
					b = static_cast<uint8_t>(170 + ((x * 13 + y * 3) & 35));
				} else {
					r = shade;
					g = shade;
					b = static_cast<uint8_t>(std::min<int32_t>(255, shade + 8));
				}
			} else if (world->material[i] == MATERIAL_FIRE) {
				float age = clampf(world->lifetime[i] / 100.0f, 0.0f, 1.0f);
				float flicker = 0.75f + 0.25f * hash01(x, y, static_cast<int32_t>(world->lifetime[i]));
				r = static_cast<uint8_t>(255.0f * flicker);
				g = static_cast<uint8_t>((92.0f + 120.0f * (1.0f - age)) * flicker);
				b = static_cast<uint8_t>(18.0f + 30.0f * (1.0f - age));
			} else if (def.gas && std::isfinite(world->volume_fraction[i]) && world->volume_fraction[i] > MASS_EPSILON) {
				float m = clampf(world->volume_fraction[i], 0.0f, 1.5f);
				float age = clampf(world->lifetime[i] / 1400.0f, 0.0f, 1.0f);
				float left = (x > 0 && get_material_def(world->material[i - 1]).gas) ? world->volume_fraction[i - 1] : 0.0f;
				float right = (x + 1 < width && get_material_def(world->material[i + 1]).gas) ? world->volume_fraction[i + 1] : 0.0f;
				float up = (y > 0 && get_material_def(world->material[i - width]).gas) ? world->volume_fraction[i - width] : 0.0f;
				float down = (y + 1 < height && get_material_def(world->material[i + width]).gas) ? world->volume_fraction[i + width] : 0.0f;
				float neighbor_avg = 0.25f * (left + right + up + down);
				float edge = clampf(std::abs(m - neighbor_avg) * 1.4f, 0.0f, 1.0f);
				float n1 = hash01(x, y, static_cast<int32_t>(world->lifetime[i]) / 7);
				float n2 = hash01(x / 3, y / 3, static_cast<int32_t>(world->lifetime[i]) / 19);
				float puff = clampf(0.72f + 0.34f * n1 + 0.22f * n2 + 0.18f * edge, 0.0f, 1.35f);
				float visual = clampf(std::sqrt(std::min(m * 1.7f, 1.0f)) * puff * (1.0f - 0.35f * age), 0.0f, 1.0f);
				// Fake transparency by blending gas color over the dark background.
				float warm = hash01(x / 5, y / 5, 17) * 10.0f;
				float gas_r = 88.0f + 118.0f * visual + warm;
				float gas_g = 92.0f + 112.0f * visual + warm;
				float gas_b = 102.0f + 120.0f * visual;
				if (world->material[i] == MATERIAL_STEAM) {
					gas_r = 155.0f + 80.0f * visual;
					gas_g = 178.0f + 65.0f * visual;
					gas_b = 190.0f + 55.0f * visual;
				} else if (world->material[i] == MATERIAL_TOXIC_GAS) {
					gas_r = 70.0f + 65.0f * visual;
					gas_g = 170.0f + 80.0f * visual;
					gas_b = 45.0f + 40.0f * visual;
				} else if (world->material[i] == MATERIAL_FLAMMABLE_GAS) {
					gas_r = 145.0f + 90.0f * visual;
					gas_g = 92.0f + 75.0f * visual;
					gas_b = 135.0f + 70.0f * visual;
				}
				r = static_cast<uint8_t>(5.0f * (1.0f - visual) + gas_r * visual);
				g = static_cast<uint8_t>(6.0f * (1.0f - visual) + gas_g * visual);
				b = static_cast<uint8_t>(9.0f * (1.0f - visual) + gas_b * visual);
			} else if (def.liquid && std::isfinite(world->volume_fraction[i]) && world->volume_fraction[i] > MASS_EPSILON) {
				float depth = clampf(world->volume_fraction[i] / DEEPEST_COLOR_THRESHOLD, 0.0f, 1.0f);
				float pvis = clampf(std::abs(world->pressure[i]) * 0.04f, 0.0f, 0.45f);
				float toxic_concentration = clampf(world->toxic[i] / std::max(world->volume_fraction[i], MASS_EPSILON), 0.0f, 1.0f);
				float toxic_visible = clampf((toxic_concentration - 0.25f) / 0.75f, 0.0f, 1.0f);
				float oil_visible = oil_fraction_at(i);
				float water_r = 10.0f + 45.0f * pvis;
				float water_g = 54.0f + 125.0f * depth + 35.0f * pvis;
				float water_b = 120.0f + 125.0f * depth;
				float oil_r = 108.0f + 105.0f * depth + 20.0f * pvis;
				float oil_g = 82.0f + 75.0f * depth + 15.0f * pvis;
				float oil_b = 22.0f + 35.0f * depth;
				float base_r = water_r * (1.0f - oil_visible) + oil_r * oil_visible;
				float base_g = water_g * (1.0f - oil_visible) + oil_g * oil_visible;
				float base_b = water_b * (1.0f - oil_visible) + oil_b * oil_visible;
				float toxic_r = 45.0f + 42.0f * depth + 40.0f * pvis;
				float toxic_g = 135.0f + 105.0f * depth + 20.0f * pvis;
				float toxic_b = 32.0f + 45.0f * depth;
				r = static_cast<uint8_t>(base_r * (1.0f - toxic_visible) + toxic_r * toxic_visible);
				g = static_cast<uint8_t>(base_g * (1.0f - toxic_visible) + toxic_g * toxic_visible);
				b = static_cast<uint8_t>(base_b * (1.0f - toxic_visible) + toxic_b * toxic_visible);
			}
			dst[p + 0] = r;
			dst[p + 1] = g;
			dst[p + 2] = b;
			dst[p + 3] = 255;
		}
	}
}

double MacFluidSolver::get_total_water_mass() const {
	double total = 0.0;
	for (int32_t i = 0; i < width * height; i++) {
		if (get_material_def(world->material[i]).liquid && std::isfinite(world->volume_fraction[i]) && world->volume_fraction[i] > 0.0f) {
			total += world->volume_fraction[i];
		}
	}
	return total;
}

int64_t MacFluidSolver::get_water_cell_count() const {
	int64_t count = 0;
	for (int32_t i = 0; i < width * height; i++) {
		if (get_material_def(world->material[i]).liquid && std::isfinite(world->volume_fraction[i]) && world->volume_fraction[i] > MASS_EPSILON) {
			count++;
		}
	}
	return count;
}

double MacFluidSolver::get_average_water_mass() const {
	double total = 0.0;
	int64_t count = 0;
	for (int32_t i = 0; i < width * height; i++) {
		if (get_material_def(world->material[i]).liquid && std::isfinite(world->volume_fraction[i]) && world->volume_fraction[i] > MASS_EPSILON) {
			total += world->volume_fraction[i];
			count++;
		}
	}
	return count == 0 ? 0.0 : total / static_cast<double>(count);
}

double MacFluidSolver::get_last_step_ms() const { return last_step_ms; }
double MacFluidSolver::get_last_predict_ms() const { return last_predict_ms; }
double MacFluidSolver::get_last_build_ms() const { return last_build_ms; }
double MacFluidSolver::get_last_pcg_ms() const { return last_pcg_ms; }
double MacFluidSolver::get_last_project_ms() const { return last_project_ms; }
double MacFluidSolver::get_last_advect_ms() const { return last_advect_ms; }
double MacFluidSolver::get_last_clamp_ms() const { return last_clamp_ms; }
int32_t MacFluidSolver::get_active_region_min_x() const { return active_min_x; }
int32_t MacFluidSolver::get_active_region_min_y() const { return active_min_y; }
int32_t MacFluidSolver::get_active_region_max_x() const { return active_max_x; }
int32_t MacFluidSolver::get_active_region_max_y() const { return active_max_y; }
int32_t MacFluidSolver::get_active_region_pad() const { return active_pad; }
double MacFluidSolver::get_active_region_max_speed() const { return active_max_speed; }
int64_t MacFluidSolver::get_step_count() const { return static_cast<int64_t>(step_count); }
int32_t MacFluidSolver::get_last_pcg_iterations() const { return last_pcg_iterations; }
double MacFluidSolver::get_last_pcg_residual() const { return last_pcg_residual; }





