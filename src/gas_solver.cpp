#include "gas_solver.h"

#include <algorithm>
#include <cmath>

using namespace noita;

namespace {
float clampf(float v, float lo, float hi) {
	return std::max(lo, std::min(v, hi));
}
}

uint32_t GasSolver::next_random_u32() {
	uint32_t x = rng_state;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	rng_state = x;
	return x;
}

bool GasSolver::frame_scans_left_to_right() {
	return (next_random_u32() & 1u) == 0u;
}

bool GasSolver::can_gas_enter(const WorldGrid &p_grid, int32_t p_x, int32_t p_y) const {
	if (!p_grid.in_bounds(p_x, p_y)) {
		return false;
	}
	const int32_t i = p_grid.cell_index(p_x, p_y);
	const MaterialDef &def = get_material_def(p_grid.material[i]);
	if (def.liquid || def.solid || def.powder || def.blocks_velocity || p_grid.material[i] == MATERIAL_FIRE) {
		return false;
	}
	return def.gas || p_grid.material[i] == MATERIAL_AIR;
}


bool GasSolver::can_rise_into(const WorldGrid &p_grid, int32_t p_from, int32_t p_to_x, int32_t p_to_y) const {
	if (!can_gas_enter(p_grid, p_to_x, p_to_y)) {
		return false;
	}
	const int32_t to = p_grid.cell_index(p_to_x, p_to_y);
	if (!get_material_def(p_grid.material[to]).gas) {
		return true;
	}
	// Gas rises only into a lower-concentration gas cell. If the cell above is
	// equally dense or denser, rising is blocked and the source can only diffuse
	// sideways/down this frame. This prevents unlimited vertical over-stacking.
	return p_grid.volume_fraction[to] + 0.002f < p_grid.volume_fraction[p_from];
}

bool GasSolver::dissolve_toxic_gas_into_liquid(WorldGrid &p_grid, int32_t p_x, int32_t p_y) {
	const int32_t dirs[4][2] = {
		{ -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 },
	};
	for (const auto &dir : dirs) {
		const int32_t x = p_x + dir[0];
		const int32_t y = p_y + dir[1];
		if (!p_grid.in_bounds(x, y) || !p_grid.is_liquid_cell(x, y, 0.01f)) {
			continue;
		}
		const int32_t li = p_grid.cell_index(x, y);
		p_grid.toxic[li] = clampf(p_grid.toxic[li] + 0.06f, 0.0f, std::max(p_grid.volume_fraction[li], 0.15f));
		if (p_grid.toxic[li] / std::max(p_grid.volume_fraction[li], 0.01f) > 0.25f) {
			p_grid.material[li] = MATERIAL_TOXIC;
		}
		return true;
	}
	return false;
}

float GasSolver::gas_max_lifetime(uint8_t p_material) const {
	if (p_material == MATERIAL_STEAM) {
		return max_lifetime * 0.85f;
	}
	if (p_material == MATERIAL_TOXIC_GAS) {
		return max_lifetime * 1.60f;
	}
	if (p_material == MATERIAL_FLAMMABLE_GAS) {
		return max_lifetime * 1.20f;
	}
	return max_lifetime;
}

float GasSolver::gas_up_multiplier(uint8_t p_material) const {
	if (p_material == MATERIAL_STEAM) {
		return 2.8f;
	}
	if (p_material == MATERIAL_TOXIC_GAS) {
		return 1.6f;
	}
	if (p_material == MATERIAL_FLAMMABLE_GAS) {
		return 2.1f;
	}
	return 2.0f;
}

void GasSolver::add_gas_mass(WorldGrid &p_grid, int32_t p_to_x, int32_t p_to_y, uint8_t p_material, float p_amount, float p_vx, float p_vy, float p_lifetime, float p_temperature) {
	if (p_amount <= 0.0f || !p_grid.in_bounds(p_to_x, p_to_y)) {
		return;
	}
	const int32_t to = p_grid.cell_index(p_to_x, p_to_y);
	if (get_material_def(p_grid.material[to]).gas) {
		const float old_mass = std::max(0.0f, p_grid.volume_fraction[to]);
		const float new_mass = old_mass + p_amount;
		if (new_mass <= 0.0f) {
			return;
		}
		// If gases mix, keep the dominant incoming type only when it contributes
		// enough mass. This is a cheap game-physics approximation.
		if (p_grid.material[to] != p_material && p_amount > old_mass * 0.35f) {
			p_grid.material[to] = p_material;
		}
		p_grid.velocity_x[to] = clampf((p_grid.velocity_x[to] * old_mass + p_vx * p_amount) / new_mass, -max_velocity, max_velocity);
		p_grid.velocity_y[to] = clampf((p_grid.velocity_y[to] * old_mass + p_vy * p_amount) / new_mass, -max_velocity, max_velocity);
		p_grid.lifetime[to] = (p_grid.lifetime[to] * old_mass + p_lifetime * p_amount) / new_mass;
		p_grid.temperature[to] = std::max(p_grid.temperature[to], p_temperature);
		p_grid.volume_fraction[to] = new_mass;
		return;
	}

	p_grid.material[to] = p_material;
	p_grid.volume_fraction[to] = p_amount;
	p_grid.density[to] = get_material_def(p_material).density;
	p_grid.toxic[to] = p_material == MATERIAL_TOXIC_GAS ? p_amount : 0.0f;
	p_grid.oil[to] = 0.0f;
	p_grid.pressure[to] = 0.0f;
	p_grid.velocity_x[to] = clampf(p_vx, -max_velocity, max_velocity);
	p_grid.velocity_y[to] = clampf(p_vy, -max_velocity, max_velocity);
	p_grid.temperature[to] = p_temperature;
	p_grid.lifetime[to] = p_lifetime;
}

float GasSolver::diffuse_gas_mass(WorldGrid &p_grid, int32_t p_to_x, int32_t p_to_y, uint8_t p_material, float p_amount, float p_vx, float p_vy, float p_lifetime, float p_temperature) {
	if (p_amount <= 0.0f || !can_gas_enter(p_grid, p_to_x, p_to_y)) {
		return 0.0f;
	}
	add_gas_mass(p_grid, p_to_x, p_to_y, p_material, p_amount, p_vx, p_vy, p_lifetime, p_temperature);
	return p_amount;
}

void GasSolver::step(WorldGrid &p_grid) {
	gas_order = p_grid.active_gas_cells;
	if (gas_order.empty()) {
		return;
	}
	const bool left_to_right = frame_scans_left_to_right();
	const int32_t width = p_grid.width;

	std::sort(gas_order.begin(), gas_order.end(), [width, left_to_right](int32_t a, int32_t b) {
		const int32_t ay = a / width;
		const int32_t by = b / width;
		if (ay != by) {
			return ay < by; // top to bottom: gases rise.
		}
		const int32_t ax = a % width;
		const int32_t bx = b % width;
		return left_to_right ? ax < bx : ax > bx;
	});

	for (int32_t idx : gas_order) {
		if (idx < 0 || idx >= p_grid.cell_count() || !get_material_def(p_grid.material[idx]).gas) {
			continue;
		}
		const int32_t x = idx % width;
		const int32_t y = idx / width;
		const uint8_t mat = p_grid.material[idx];

		p_grid.lifetime[idx] += 1.0f;
		p_grid.temperature[idx] = std::max(0.0f, p_grid.temperature[idx] - 0.004f);

		if (mat == MATERIAL_STEAM && p_grid.temperature[idx] < 0.22f) {
			p_grid.make_water(x, y, std::max(0.15f, p_grid.volume_fraction[idx]));
			continue;
		}
		if (mat == MATERIAL_TOXIC_GAS && dissolve_toxic_gas_into_liquid(p_grid, x, y)) {
			p_grid.make_air(x, y);
			continue;
		}
		if (mat == MATERIAL_FLAMMABLE_GAS && p_grid.temperature[idx] > 0.80f) {
			p_grid.make_fire(x, y, 1.0f);
			continue;
		}

		float source_mass = std::max(0.0f, p_grid.volume_fraction[idx]);
		const float effective_max_life = gas_max_lifetime(mat) * (0.45f + 0.55f * clampf(source_mass, 0.0f, 1.0f));
		if (source_mass < min_mass || p_grid.lifetime[idx] > effective_max_life) {
			p_grid.make_air(x, y);
			continue;
		}

		float vx = clampf(p_grid.velocity_x[idx] * velocity_damping, -max_velocity, max_velocity);
		float vy = clampf(p_grid.velocity_y[idx] * velocity_damping, -max_velocity, max_velocity);
		const int first_side = (next_random_u32() & 1u) ? -1 : 1;
		const float up_mul = gas_up_multiplier(mat);

		struct Candidate {
			int32_t dx;
			int32_t dy;
			float fraction;
			float vx;
			float vy;
		};

		Candidate candidates[4] = {
			{ 0, -1, diffusion_fraction * up_mul, vx * 0.25f, -diffusion_speed * up_mul },
			{ first_side, 0, diffusion_fraction, diffusion_speed * static_cast<float>(first_side), 0.0f },
			{ -first_side, 0, diffusion_fraction, -diffusion_speed * static_cast<float>(first_side), 0.0f },
			{ 0, 1, diffusion_fraction * 0.55f, vx * 0.15f, diffusion_speed * 0.35f },
		};

		float remaining = source_mass;
		for (const Candidate &c : candidates) {
			if (remaining <= min_mass) {
				break;
			}
			const int32_t tx = x + c.dx;
			const int32_t ty = y + c.dy;
			if (c.dy < 0 && !can_rise_into(p_grid, idx, tx, ty)) {
				continue;
			}
			const float amount = std::min(remaining, source_mass * c.fraction);
			const float moved = diffuse_gas_mass(p_grid, tx, ty, mat, amount, c.vx, c.vy, p_grid.lifetime[idx], p_grid.temperature[idx]);
			remaining -= moved;
		}

		if (remaining < min_mass || p_grid.lifetime[idx] > gas_max_lifetime(mat) * (0.45f + 0.55f * clampf(remaining, 0.0f, 1.0f))) {
			p_grid.make_air(x, y);
		} else {
			p_grid.volume_fraction[idx] = remaining;
			p_grid.velocity_x[idx] = vx * 0.35f;
			p_grid.velocity_y[idx] = vy * 0.35f;
		}
	}
}

