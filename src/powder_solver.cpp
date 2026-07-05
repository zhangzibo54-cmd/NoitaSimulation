#include "powder_solver.h"

#include <algorithm>
#include <cmath>

using namespace noita;

namespace {
float clampf(float v, float lo, float hi) {
	return std::max(lo, std::min(v, hi));
}
} // namespace

uint32_t PowderSolver::next_random_u32() {
	// Small deterministic xorshift. Good enough for scan direction / tie breaking.
	uint32_t x = rng_state;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	rng_state = x;
	return x;
}

bool PowderSolver::frame_scans_left_to_right() {
	return (next_random_u32() & 1u) == 0u;
}

bool PowderSolver::can_powder_enter(const WorldGrid &p_grid, int32_t p_x, int32_t p_y) const {
	if (!p_grid.in_bounds(p_x, p_y)) {
		return false;
	}
	const int32_t i = p_grid.cell_index(p_x, p_y);
	if (p_grid.rigid_body_id[i] != 0) {
		return false;
	}
	const MaterialDef &def = get_material_def(p_grid.material[i]);
	// Sand cannot enter rock or another powder. It may enter air/smoke, and for
	// now it may swap with liquid so water is displaced upward/sideways.
	return !def.solid && !def.powder;
}

bool PowderSolver::try_move_sand(WorldGrid &p_grid, int32_t p_from_x, int32_t p_from_y, int32_t p_to_x, int32_t p_to_y, float p_new_vx, float p_new_vy) {
	if (!can_powder_enter(p_grid, p_to_x, p_to_y)) {
		return false;
	}
	const int32_t from = p_grid.cell_index(p_from_x, p_from_y);
	const int32_t to = p_grid.cell_index(p_to_x, p_to_y);
	if (!get_material_def(p_grid.material[from]).powder) {
		return false;
	}

	const uint8_t dst_material = p_grid.material[to];
	const float dst_mass = p_grid.volume_fraction[to];
	const float dst_density = p_grid.density[to];
	const float dst_oil = p_grid.oil[to];
	const float dst_toxic = p_grid.toxic[to];
	const float dst_pressure = p_grid.pressure[to];
	const float dst_vx = p_grid.velocity_x[to];
	const float dst_vy = p_grid.velocity_y[to];
	const float dst_temp = p_grid.temperature[to];
	const float dst_life = p_grid.lifetime[to];

	p_grid.material[to] = p_grid.material[from];
	p_grid.volume_fraction[to] = p_grid.volume_fraction[from] > 0.0f ? p_grid.volume_fraction[from] : 1.0f;
	p_grid.toxic[to] = 0.0f;
	p_grid.pressure[to] = 0.0f;
	p_grid.velocity_x[to] = p_new_vx;
	p_grid.velocity_y[to] = p_new_vy;
	p_grid.temperature[to] = p_grid.temperature[from];
	p_grid.lifetime[to] = p_grid.lifetime[from] + 1.0f;

	// If the target was liquid, displace it into the old sand cell. Otherwise the
	// old sand cell becomes air. This gives a simple sand-sinks-through-water rule.
	if (get_material_def(dst_material).liquid) {
		p_grid.material[from] = dst_material;
		p_grid.volume_fraction[from] = dst_mass;
		p_grid.toxic[from] = dst_toxic;
		p_grid.pressure[from] = dst_pressure;
		p_grid.velocity_x[from] = dst_vx;
		p_grid.velocity_y[from] = dst_vy;
		p_grid.temperature[from] = dst_temp;
		p_grid.lifetime[from] = dst_life;
	} else {
		p_grid.make_air(p_from_x, p_from_y);
	}
	return true;
}

void PowderSolver::step(WorldGrid &p_grid) {
	powder_order = p_grid.active_powder_cells;
	if (powder_order.empty()) {
		return;
	}
	const bool left_to_right = frame_scans_left_to_right();
	const int32_t width = p_grid.width;

	std::sort(powder_order.begin(), powder_order.end(), [width, left_to_right](int32_t a, int32_t b) {
		const int32_t ay = a / width;
		const int32_t by = b / width;
		if (ay != by) {
			return ay > by; // bottom to top: y grows downward in screen/grid coordinates.
		}
		const int32_t ax = a % width;
		const int32_t bx = b % width;
		return left_to_right ? ax < bx : ax > bx;
	});

	for (int32_t idx : powder_order) {
		if (idx < 0 || idx >= p_grid.cell_count() || !get_material_def(p_grid.material[idx]).powder) {
			continue;
		}
		const int32_t x = idx % width;
		const int32_t y = idx / width;

		if (p_grid.rigid_body_id[idx] != 0) {
			const float rvx = p_grid.rigid_velocity_x[idx];
			const float rvy = p_grid.rigid_velocity_y[idx];
			const int sx = rvx > 0.05f ? 1 : (rvx < -0.05f ? -1 : (left_to_right ? 1 : -1));
			const int sy = rvy > 0.05f ? 1 : (rvy < -0.05f ? -1 : 0);
			const int dirs[6][2] = {
				{ sx, sy },
				{ sx, 0 },
				{ -sx, 0 },
				{ sx, -1 },
				{ -sx, -1 },
				{ 0, -1 },
			};
			bool ejected = false;
			for (const auto &dir : dirs) {
				const int32_t tx = x + dir[0];
				const int32_t ty = y + dir[1];
				// Approximate elastic contact with a moving rigid cell:
				// v_after = v_sand + 2 * (v_rigid - v_sand), then add a tiny
				// directional separation bias so the powder leaves the rigid tag.
				const float out_vx = clampf(p_grid.velocity_x[idx] + 2.0f * (rvx - p_grid.velocity_x[idx]) + dir[0] * 0.20f, -max_velocity, max_velocity);
				const float out_vy = clampf(p_grid.velocity_y[idx] + 2.0f * (rvy - p_grid.velocity_y[idx]) + dir[1] * 0.20f, -max_velocity, max_velocity);
				if (try_move_sand(p_grid, x, y, tx, ty, out_vx, out_vy)) {
					ejected = true;
					break;
				}
			}
			if (ejected) {
				continue;
			}
		}

		float vx = clampf(p_grid.velocity_x[idx] * velocity_damping, -max_velocity, max_velocity);
		float vy = clampf(p_grid.velocity_y[idx] + gravity, -max_velocity, max_velocity);

		if (try_move_sand(p_grid, x, y, x, y + 1, vx, vy)) {
			continue;
		}

		const int32_t preferred_dir = std::abs(vx) > 0.10f ? (vx > 0.0f ? 1 : -1) : (left_to_right ? -1 : 1);
		const int32_t other_dir = -preferred_dir;

		if (try_move_sand(p_grid, x, y, x + preferred_dir, y + 1, clampf(vx + preferred_dir * 0.15f, -max_velocity, max_velocity), vy * 0.55f)) {
			continue;
		}
		if (try_move_sand(p_grid, x, y, x + other_dir, y + 1, clampf(vx + other_dir * 0.15f, -max_velocity, max_velocity), vy * 0.55f)) {
			continue;
		}

		// If pressure or a previous collision has given horizontal velocity, allow a
		// slow sideways creep. This helps piles relax without making sand liquid.
		if (std::abs(vx) > 0.35f) {
			const int32_t side_dir = vx > 0.0f ? 1 : -1;
			if (try_move_sand(p_grid, x, y, x + side_dir, y, vx * 0.70f, 0.0f)) {
				continue;
			}
		}

		p_grid.velocity_x[idx] = 0.0f;
		p_grid.velocity_y[idx] = 0.0f;
	}
}

