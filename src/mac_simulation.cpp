#include "mac_simulation.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace {
constexpr float MASS_EPSILON = 0.01f;
constexpr float PCG_EPSILON = 1.0e-7f;
constexpr float DEEPEST_COLOR_THRESHOLD = 0.60f;

float clampf(float v, float lo, float hi) {
	return std::max(lo, std::min(v, hi));
}
} // namespace

MacSimulation::MacSimulation() {
	set_world_size(width, height);
}

int32_t MacSimulation::cell_index(int32_t p_x, int32_t p_y) const { return p_y * width + p_x; }
int32_t MacSimulation::u_index(int32_t p_x, int32_t p_y) const { return p_y * (width + 1) + p_x; }
int32_t MacSimulation::v_index(int32_t p_x, int32_t p_y) const { return p_y * width + p_x; }

bool MacSimulation::in_bounds(int32_t p_x, int32_t p_y) const {
	return p_x >= 0 && p_y >= 0 && p_x < width && p_y < height;
}

bool MacSimulation::is_solid_cell(int32_t p_x, int32_t p_y) const {
	if (!in_bounds(p_x, p_y)) {
		return true;
	}
	return material[cell_index(p_x, p_y)] == MATERIAL_ROCK;
}

bool MacSimulation::is_liquid_cell(int32_t p_x, int32_t p_y) const {
	if (!in_bounds(p_x, p_y) || is_solid_cell(p_x, p_y)) {
		return false;
	}
	return mass[cell_index(p_x, p_y)] > MASS_EPSILON;
}

bool MacSimulation::is_internal_liquid_cell(int32_t p_x, int32_t p_y) const {
	if (!is_liquid_cell(p_x, p_y)) {
		return false;
	}
	const int32_t nx[4] = { p_x - 1, p_x + 1, p_x, p_x };
	const int32_t ny[4] = { p_y, p_y, p_y - 1, p_y + 1 };
	for (int32_t k = 0; k < 4; k++) {
		if (is_solid_cell(nx[k], ny[k])) {
			continue;
		}
		if (!in_bounds(nx[k], ny[k])) {
			continue;
		}
		int32_t ni = cell_index(nx[k], ny[k]);
		if (!std::isfinite(mass[ni]) || mass[ni] <= pressure_active_mass) {
			return false;
		}
	}
	return true;
}

bool MacSimulation::is_pressure_active_cell(int32_t p_x, int32_t p_y) const {
	if (!in_bounds(p_x, p_y) || is_solid_cell(p_x, p_y)) {
		return false;
	}
	float m = mass[cell_index(p_x, p_y)];
	return std::isfinite(m) && (m > pressure_active_mass || is_internal_liquid_cell(p_x, p_y));
}

bool MacSimulation::u_face_blocked(int32_t p_x, int32_t p_y) const {
	if (p_y < 0 || p_y >= height || p_x < 0 || p_x > width) {
		return true;
	}
	return is_solid_cell(p_x - 1, p_y) || is_solid_cell(p_x, p_y);
}

bool MacSimulation::v_face_blocked(int32_t p_x, int32_t p_y) const {
	if (p_x < 0 || p_x >= width || p_y < 0 || p_y > height) {
		return true;
	}
	return is_solid_cell(p_x, p_y - 1) || is_solid_cell(p_x, p_y);
}

float MacSimulation::cell_density(int32_t p_x, int32_t p_y) const {
	return density;
}

float MacSimulation::u_face_alpha(int32_t p_x, int32_t p_y) const {
	if (u_face_blocked(p_x, p_y)) {
		return 0.0f;
	}
	bool left_active = is_pressure_active_cell(p_x - 1, p_y);
	bool right_active = is_pressure_active_cell(p_x, p_y);
	if (!left_active && !right_active) {
		return 0.0f;
	}
	return dt / std::max(density, 1.0e-6f);
}

float MacSimulation::v_face_alpha(int32_t p_x, int32_t p_y) const {
	if (v_face_blocked(p_x, p_y)) {
		return 0.0f;
	}
	bool up_active = is_pressure_active_cell(p_x, p_y - 1);
	bool down_active = is_pressure_active_cell(p_x, p_y);
	if (!up_active && !down_active) {
		return 0.0f;
	}
	return dt / std::max(density, 1.0e-6f);
}

void MacSimulation::ensure_buffers() {
	const int32_t cell_count = width * height;
	const int32_t u_count = (width + 1) * height;
	const int32_t v_count = width * (height + 1);

	material.resize(cell_count, MATERIAL_AIR);
	mass.resize(cell_count, 0.0f);
	next_mass.resize(cell_count, 0.0f);
	pressure.resize(cell_count, 0.0f);
	rhs.resize(cell_count, 0.0f);
	residual.resize(cell_count, 0.0f);
	direction.resize(cell_count, 0.0f);
	q_vec.resize(cell_count, 0.0f);
	z_vec.resize(cell_count, 0.0f);
	diag_inv.resize(cell_count, 0.0f);
	outflow_mass.resize(cell_count, 0.0f);
	active_cells.clear();
	active_cells.reserve(cell_count);

	u.resize(u_count, 0.0f);
	u_tmp.resize(u_count, 0.0f);
	u_alpha.resize(u_count, 0.0f);
	u_mass_flux.resize(u_count, 0.0f);
	v.resize(v_count, 0.0f);
	v_tmp.resize(v_count, 0.0f);
	v_alpha.resize(v_count, 0.0f);
	v_mass_flux.resize(v_count, 0.0f);
}

void MacSimulation::clear_fields() {
	std::fill(material.begin(), material.end(), MATERIAL_AIR);
	std::fill(mass.begin(), mass.end(), 0.0f);
	std::fill(next_mass.begin(), next_mass.end(), 0.0f);
	std::fill(pressure.begin(), pressure.end(), 0.0f);
	std::fill(rhs.begin(), rhs.end(), 0.0f);
	std::fill(outflow_mass.begin(), outflow_mass.end(), 0.0f);
	std::fill(u.begin(), u.end(), 0.0f);
	std::fill(v.begin(), v.end(), 0.0f);
	std::fill(u_tmp.begin(), u_tmp.end(), 0.0f);
	std::fill(v_tmp.begin(), v_tmp.end(), 0.0f);
	std::fill(u_alpha.begin(), u_alpha.end(), 0.0f);
	std::fill(v_alpha.begin(), v_alpha.end(), 0.0f);
	std::fill(u_mass_flux.begin(), u_mass_flux.end(), 0.0f);
	std::fill(v_mass_flux.begin(), v_mass_flux.end(), 0.0f);
	active_cells.clear();
}

void MacSimulation::apply_solid_boundaries(std::vector<float> &p_u, std::vector<float> &p_v) {
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

float MacSimulation::sample_u_clamped(const std::vector<float> &p_u, int32_t p_x, int32_t p_y) const {
	p_x = std::max(0, std::min(p_x, width));
	p_y = std::max(0, std::min(p_y, height - 1));
	return p_u[u_index(p_x, p_y)];
}

float MacSimulation::sample_v_clamped(const std::vector<float> &p_v, int32_t p_x, int32_t p_y) const {
	p_x = std::max(0, std::min(p_x, width - 1));
	p_y = std::max(0, std::min(p_y, height));
	return p_v[v_index(p_x, p_y)];
}

float MacSimulation::vertical_velocity_at_u_face(const std::vector<float> &p_v, int32_t p_x, int32_t p_y) const {
	// Average the four vertical faces touching the u-face sample point.
	return 0.25f * (
			sample_v_clamped(p_v, p_x - 1, p_y) +
			sample_v_clamped(p_v, p_x, p_y) +
			sample_v_clamped(p_v, p_x - 1, p_y + 1) +
			sample_v_clamped(p_v, p_x, p_y + 1));
}

float MacSimulation::horizontal_velocity_at_v_face(const std::vector<float> &p_u, int32_t p_x, int32_t p_y) const {
	// Average the four horizontal faces touching the v-face sample point.
	return 0.25f * (
			sample_u_clamped(p_u, p_x, p_y - 1) +
			sample_u_clamped(p_u, p_x + 1, p_y - 1) +
			sample_u_clamped(p_u, p_x, p_y) +
			sample_u_clamped(p_u, p_x + 1, p_y));
}

void MacSimulation::predict_velocity_explicit() {
	u_tmp = u;
	v_tmp = v;

	for (int32_t y = 0; y < height; y++) {
		for (int32_t x = 1; x < width; x++) {
			int32_t idx = u_index(x, y);
			if (!is_liquid_cell(x - 1, y) && !is_liquid_cell(x, y)) {
				u_tmp[idx] = 0.0f;
				continue;
			}
			float u0 = u[idx];
			float lap = sample_u_clamped(u, x + 1, y) + sample_u_clamped(u, x - 1, y) +
					sample_u_clamped(u, x, y + 1) + sample_u_clamped(u, x, y - 1) - 4.0f * u0;
			// Temporarily remove the explicit convective term -(U 路 grad)u.
			// This isolates whether self-advection is injecting residual motion.
			u_tmp[idx] = u0 + dt * (viscosity * lap);
		}
	}

	for (int32_t y = 1; y < height; y++) {
		for (int32_t x = 0; x < width; x++) {
			int32_t idx = v_index(x, y);
			if (!is_liquid_cell(x, y - 1) && !is_liquid_cell(x, y)) {
				v_tmp[idx] = 0.0f;
				continue;
			}
			float v0 = v[idx];
			float lap = sample_v_clamped(v, x + 1, y) + sample_v_clamped(v, x - 1, y) +
					sample_v_clamped(v, x, y + 1) + sample_v_clamped(v, x, y - 1) - 4.0f * v0;
			// Temporarily remove the explicit convective term -(U 路 grad)v.
			// Gravity and viscosity remain active.
			v_tmp[idx] = v0 + dt * (viscosity * lap + gravity);
		}
	}

	apply_solid_boundaries(u_tmp, v_tmp);
}

void MacSimulation::build_pressure_system() {
	std::fill(rhs.begin(), rhs.end(), 0.0f);
	std::fill(diag_inv.begin(), diag_inv.end(), 0.0f);
	std::fill(u_alpha.begin(), u_alpha.end(), 0.0f);
	std::fill(v_alpha.begin(), v_alpha.end(), 0.0f);
	active_cells.clear();

	for (int32_t y = 0; y < height; y++) {
		for (int32_t x = 0; x <= width; x++) {
			u_alpha[u_index(x, y)] = u_face_alpha(x, y);
		}
	}
	for (int32_t y = 0; y <= height; y++) {
		for (int32_t x = 0; x < width; x++) {
			v_alpha[v_index(x, y)] = v_face_alpha(x, y);
		}
	}

	for (int32_t y = 0; y < height; y++) {
		for (int32_t x = 0; x < width; x++) {
			int32_t i = cell_index(x, y);
			pressure[i] = 0.0f;
			if (!is_pressure_active_cell(x, y)) {
				continue;
			}

			float div = (u_tmp[u_index(x + 1, y)] - u_tmp[u_index(x, y)] +
					v_tmp[v_index(x, y + 1)] - v_tmp[v_index(x, y)]);

			float diag = u_alpha[u_index(x, y)] + u_alpha[u_index(x + 1, y)] + v_alpha[v_index(x, y)] + v_alpha[v_index(x, y + 1)];
			if (diag <= 0.0f) {
				continue;
			}
			active_cells.push_back(i);
			diag_inv[i] = 1.0f / diag;

			// Density/volume correction target:
			// div(U_new) = s.
			// - over-full cells ask for positive divergence, pushing volume out.
			// - internal under-filled cells ask for weak negative divergence,
			//   pulling volume in.  Free-surface cells do NOT get this underfill
			//   term, otherwise the surface would constantly suck in water.
			float excess = std::max(0.0f, mass[i] - target_mass);
			float deficit = std::max(0.0f, target_mass - mass[i]);
			float s = density_correction_strength * excess / std::max(dt, 1.0e-5f);
			if (deficit > 0.0f && is_internal_liquid_cell(x, y)) {
				s -= underfill_correction_strength * deficit / std::max(dt, 1.0e-5f);
			}

			// With the projection U_new = U* - alpha * grad(p), and with our
			// discrete operator A(p)=sum alpha_f*(p_C-p_N), the consistent RHS is:
			// A p = s - div(U*).
			rhs[i] = s - div;
		}
	}
}

void MacSimulation::apply_laplacian(const std::vector<float> &p_x, std::vector<float> &r_ax) const {
	std::fill(r_ax.begin(), r_ax.end(), 0.0f);
	for (int32_t i : active_cells) {
			int32_t x = i % width;
			int32_t y = i / width;

			float ax = 0.0f;
			float a_left = u_alpha[u_index(x, y)];
			float a_right = u_alpha[u_index(x + 1, y)];
			float a_up = v_alpha[v_index(x, y)];
			float a_down = v_alpha[v_index(x, y + 1)];

			if (a_left > 0.0f) {
				ax += a_left * (p_x[i] - (is_pressure_active_cell(x - 1, y) ? p_x[cell_index(x - 1, y)] : 0.0f));
			}
			if (a_right > 0.0f) {
				ax += a_right * (p_x[i] - (is_pressure_active_cell(x + 1, y) ? p_x[cell_index(x + 1, y)] : 0.0f));
			}
			if (a_up > 0.0f) {
				ax += a_up * (p_x[i] - (is_pressure_active_cell(x, y - 1) ? p_x[cell_index(x, y - 1)] : 0.0f));
			}
			if (a_down > 0.0f) {
				ax += a_down * (p_x[i] - (is_pressure_active_cell(x, y + 1) ? p_x[cell_index(x, y + 1)] : 0.0f));
			}
			r_ax[i] = ax;
	}
}

double MacSimulation::dot_liquid(const std::vector<float> &p_a, const std::vector<float> &p_b) const {
	double sum = 0.0;
	for (int32_t i : active_cells) {
		sum += static_cast<double>(p_a[i]) * static_cast<double>(p_b[i]);
	}
	return sum;
}

void MacSimulation::solve_pressure_pcg() {
	apply_laplacian(pressure, q_vec);
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
			pressure[i] += static_cast<float>(alpha) * direction[i];
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

void MacSimulation::apply_pressure_projection() {
	u = u_tmp;
	v = v_tmp;

	for (int32_t y = 0; y < height; y++) {
		for (int32_t x = 0; x <= width; x++) {
			int32_t idx = u_index(x, y);
			if (u_face_blocked(x, y)) {
				u[idx] = 0.0f;
				continue;
			}
			bool left_liquid = is_pressure_active_cell(x - 1, y);
			bool right_liquid = is_pressure_active_cell(x, y);
			if (!left_liquid && !right_liquid) {
				continue;
			}
			float p_left = left_liquid ? pressure[cell_index(x - 1, y)] : 0.0f;
			float p_right = right_liquid ? pressure[cell_index(x, y)] : 0.0f;
			u[idx] -= u_alpha[idx] * (p_right - p_left);
		}
	}

	for (int32_t y = 0; y <= height; y++) {
		for (int32_t x = 0; x < width; x++) {
			int32_t idx = v_index(x, y);
			if (v_face_blocked(x, y)) {
				v[idx] = 0.0f;
				continue;
			}
			bool up_liquid = is_pressure_active_cell(x, y - 1);
			bool down_liquid = is_pressure_active_cell(x, y);
			if (!up_liquid && !down_liquid) {
				continue;
			}
			float p_up = up_liquid ? pressure[cell_index(x, y - 1)] : 0.0f;
			float p_down = down_liquid ? pressure[cell_index(x, y)] : 0.0f;
			v[idx] -= v_alpha[idx] * (p_down - p_up);
		}
	}

	apply_solid_boundaries(u, v);
}

void MacSimulation::advect_mass_finite_volume() {
	next_mass = mass;
	std::fill(u_mass_flux.begin(), u_mass_flux.end(), 0.0f);
	std::fill(v_mass_flux.begin(), v_mass_flux.end(), 0.0f);
	std::fill(outflow_mass.begin(), outflow_mass.end(), 0.0f);

	for (int32_t i = 0; i < width * height; i++) {
		if (material[i] == MATERIAL_ROCK) {
			next_mass[i] = 0.0f;
		} else if (!std::isfinite(next_mass[i]) || next_mass[i] < 0.0f) {
			next_mass[i] = 0.0f;
		}
	}

	// First pass: compute raw conservative face fluxes and accumulate each
	// donor cell's total outgoing mass.  Do not apply them yet, because applying
	// each face independently can overdraw one cell from several directions.
	for (int32_t y = 0; y < height; y++) {
		for (int32_t x = 1; x < width; x++) {
			if (u_face_blocked(x, y)) {
				continue;
			}
			int32_t li = cell_index(x - 1, y);
			int32_t ri = cell_index(x, y);
			float vel = u[u_index(x, y)];
			float donor_mass = vel > 0.0f ? std::max(0.0f, mass[li]) : std::max(0.0f, mass[ri]);
			float flux = vel * dt * donor_mass;
			if (!std::isfinite(flux)) {
				flux = 0.0f;
			}
			u_mass_flux[u_index(x, y)] = flux;
			if (flux > 0.0f) {
				outflow_mass[li] += flux;
			} else if (flux < 0.0f) {
				outflow_mass[ri] += -flux;
			}
		}
	}

	for (int32_t y = 1; y < height; y++) {
		for (int32_t x = 0; x < width; x++) {
			if (v_face_blocked(x, y)) {
				continue;
			}
			int32_t ui = cell_index(x, y - 1);
			int32_t di = cell_index(x, y);
			float vel = v[v_index(x, y)];
			float donor_mass = vel > 0.0f ? std::max(0.0f, mass[ui]) : std::max(0.0f, mass[di]);
			float flux = vel * dt * donor_mass;
			if (!std::isfinite(flux)) {
				flux = 0.0f;
			}
			v_mass_flux[v_index(x, y)] = flux;
			if (flux > 0.0f) {
				outflow_mass[ui] += flux;
			} else if (flux < 0.0f) {
				outflow_mass[di] += -flux;
			}
		}
	}

	// Second pass: scale outgoing fluxes per donor so no cell can become
	// negative. This is the important positivity/conservation fix.
	for (int32_t y = 0; y < height; y++) {
		for (int32_t x = 1; x < width; x++) {
			int32_t li = cell_index(x - 1, y);
			int32_t ri = cell_index(x, y);
			float flux = u_mass_flux[u_index(x, y)];
			if (flux > 0.0f && outflow_mass[li] > mass[li] && outflow_mass[li] > MASS_EPSILON) {
				flux *= std::max(0.0f, mass[li]) / outflow_mass[li];
			} else if (flux < 0.0f && outflow_mass[ri] > mass[ri] && outflow_mass[ri] > MASS_EPSILON) {
				flux *= std::max(0.0f, mass[ri]) / outflow_mass[ri];
			}
			next_mass[li] -= flux;
			next_mass[ri] += flux;
		}
	}

	for (int32_t y = 1; y < height; y++) {
		for (int32_t x = 0; x < width; x++) {
			int32_t ui = cell_index(x, y - 1);
			int32_t di = cell_index(x, y);
			float flux = v_mass_flux[v_index(x, y)];
			if (flux > 0.0f && outflow_mass[ui] > mass[ui] && outflow_mass[ui] > MASS_EPSILON) {
				flux *= std::max(0.0f, mass[ui]) / outflow_mass[ui];
			} else if (flux < 0.0f && outflow_mass[di] > mass[di] && outflow_mass[di] > MASS_EPSILON) {
				flux *= std::max(0.0f, mass[di]) / outflow_mass[di];
			}
			next_mass[ui] -= flux;
			next_mass[di] += flux;
		}
	}

	for (int32_t i = 0; i < width * height; i++) {
		if (material[i] == MATERIAL_ROCK || !std::isfinite(next_mass[i]) || next_mass[i] < MASS_EPSILON) {
			mass[i] = 0.0f;
		} else {
			mass[i] = next_mass[i];
		}
	}
}

void MacSimulation::clamp_velocities() {
	for (float &value : u) {
		value = clampf(value * velocity_damping, -max_velocity, max_velocity);
	}
	for (float &value : v) {
		value = clampf(value * velocity_damping, -max_velocity, max_velocity);
	}
	apply_solid_boundaries(u, v);
}

void MacSimulation::step() {
	if (mass.empty()) {
		set_world_size(width, height);
	}
	auto t0 = std::chrono::high_resolution_clock::now();
	predict_velocity_explicit();
	build_pressure_system();
	solve_pressure_pcg();
	apply_pressure_projection();
	advect_mass_finite_volume();
	clamp_velocities();
	step_count++;
	auto t1 = std::chrono::high_resolution_clock::now();
	last_step_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
}

void MacSimulation::set_world_size(int32_t p_width, int32_t p_height) {
	width = std::max(16, p_width);
	height = std::max(16, p_height);
	clear_fields();
	ensure_buffers();
	step_count = 0;
}

int32_t MacSimulation::get_width() const { return width; }
int32_t MacSimulation::get_height() const { return height; }
void MacSimulation::set_dt(double p_dt) { dt = clampf(static_cast<float>(p_dt), 0.01f, 1.0f); }
double MacSimulation::get_dt() const { return dt; }
void MacSimulation::set_gravity(double p_gravity) { gravity = clampf(static_cast<float>(p_gravity), -10.0f, 10.0f); }
double MacSimulation::get_gravity() const { return gravity; }
void MacSimulation::set_viscosity(double p_viscosity) { viscosity = clampf(static_cast<float>(p_viscosity), 0.0f, 1.0f); }
double MacSimulation::get_viscosity() const { return viscosity; }
void MacSimulation::set_pressure_iterations(int32_t p_iterations) { pressure_iterations = std::max(1, std::min(p_iterations, 400)); }
int32_t MacSimulation::get_pressure_iterations() const { return pressure_iterations; }
void MacSimulation::set_pressure_active_mass(double p_mass) { pressure_active_mass = clampf(static_cast<float>(p_mass), MASS_EPSILON, 1.0f); }
double MacSimulation::get_pressure_active_mass() const { return pressure_active_mass; }
void MacSimulation::set_density_correction_strength(double p_strength) { density_correction_strength = clampf(static_cast<float>(p_strength), 0.0f, 2.0f); }
double MacSimulation::get_density_correction_strength() const { return density_correction_strength; }
void MacSimulation::set_underfill_correction_strength(double p_strength) { underfill_correction_strength = clampf(static_cast<float>(p_strength), 0.0f, 2.0f); }
double MacSimulation::get_underfill_correction_strength() const { return underfill_correction_strength; }

void MacSimulation::make_rock(int32_t p_x, int32_t p_y) {
	if (!in_bounds(p_x, p_y)) {
		return;
	}
	int32_t i = cell_index(p_x, p_y);
	material[i] = MATERIAL_ROCK;
	mass[i] = 0.0f;
	pressure[i] = 0.0f;
}

void MacSimulation::make_air(int32_t p_x, int32_t p_y) {
	if (!in_bounds(p_x, p_y)) {
		return;
	}
	int32_t i = cell_index(p_x, p_y);
	material[i] = MATERIAL_AIR;
	mass[i] = 0.0f;
	pressure[i] = 0.0f;
}

void MacSimulation::make_water(int32_t p_x, int32_t p_y, float p_mass) {
	if (!in_bounds(p_x, p_y) || is_solid_cell(p_x, p_y)) {
		return;
	}
	int32_t i = cell_index(p_x, p_y);
	material[i] = MATERIAL_AIR;
	mass[i] = clampf(mass[i] + p_mass, 0.0f, 1.0f);
}

void MacSimulation::clear() {
	clear_fields();
}

void MacSimulation::generate_basin() {
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

void MacSimulation::paint_circle(double p_x, double p_y, double p_radius, int32_t p_material) {
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
			} else {
				make_air(x, y);
			}
		}
	}
	apply_solid_boundaries(u, v);
}

void MacSimulation::inject_water(double p_x, double p_y, double p_radius, double p_mass_per_cell, double p_velocity_x, double p_velocity_y) {
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
			if (dx * dx + dy * dy > r2 || !in_bounds(x, y) || is_solid_cell(x, y)) {
				continue;
			}
			int32_t i = cell_index(x, y);
			material[i] = MATERIAL_AIR;
			mass[i] = clampf(mass[i] + add_mass, 0.0f, 1.0f);
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

void MacSimulation::fill_rgba_pixels(std::vector<uint8_t> &p_pixels) const {
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
			if (material[i] == MATERIAL_ROCK) {
				uint8_t shade = static_cast<uint8_t>(70 + ((x * 17 + y * 9) & 35));
				r = shade;
				g = shade;
				b = static_cast<uint8_t>(std::min<int32_t>(255, shade + 8));
			} else if (std::isfinite(mass[i]) && mass[i] > MASS_EPSILON) {
				float depth = clampf(mass[i] / DEEPEST_COLOR_THRESHOLD, 0.0f, 1.0f);
				float pvis = clampf(std::abs(pressure[i]) * 0.04f, 0.0f, 0.45f);
				r = static_cast<uint8_t>(10 + 45.0f * pvis);
				g = static_cast<uint8_t>(54 + 125.0f * depth + 35.0f * pvis);
				b = static_cast<uint8_t>(120 + 125.0f * depth);
			}
			dst[p + 0] = r;
			dst[p + 1] = g;
			dst[p + 2] = b;
			dst[p + 3] = 255;
		}
	}
}

double MacSimulation::get_total_water_mass() const {
	double total = 0.0;
	for (int32_t i = 0; i < width * height; i++) {
		if (material[i] != MATERIAL_ROCK && std::isfinite(mass[i]) && mass[i] > 0.0f) {
			total += mass[i];
		}
	}
	return total;
}

int64_t MacSimulation::get_water_cell_count() const {
	int64_t count = 0;
	for (int32_t i = 0; i < width * height; i++) {
		if (material[i] != MATERIAL_ROCK && std::isfinite(mass[i]) && mass[i] > MASS_EPSILON) {
			count++;
		}
	}
	return count;
}

double MacSimulation::get_average_water_mass() const {
	double total = 0.0;
	int64_t count = 0;
	for (int32_t i = 0; i < width * height; i++) {
		if (material[i] != MATERIAL_ROCK && std::isfinite(mass[i]) && mass[i] > MASS_EPSILON) {
			total += mass[i];
			count++;
		}
	}
	return count == 0 ? 0.0 : total / static_cast<double>(count);
}

double MacSimulation::get_last_step_ms() const { return last_step_ms; }
int64_t MacSimulation::get_step_count() const { return static_cast<int64_t>(step_count); }
int32_t MacSimulation::get_last_pcg_iterations() const { return last_pcg_iterations; }
double MacSimulation::get_last_pcg_residual() const { return last_pcg_residual; }



