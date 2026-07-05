#include "fire_solver.h"

#include <algorithm>
#include <cmath>

using namespace noita;

namespace {
float clampf(float v, float lo, float hi) {
	return std::max(lo, std::min(v, hi));
}
}

uint32_t FireSolver::next_random_u32() {
	uint32_t x = rng_state;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	rng_state = x;
	return x;
}

bool FireSolver::touches_liquid_material(const WorldGrid &p_grid, int32_t p_x, int32_t p_y, uint8_t p_material) const {
	const int32_t dirs[5][2] = {
		{ 0, 0 }, { -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 },
	};
	for (const auto &dir : dirs) {
		const int32_t x = p_x + dir[0];
		const int32_t y = p_y + dir[1];
		if (!p_grid.in_bounds(x, y)) {
			continue;
		}
		const int32_t i = p_grid.cell_index(x, y);
		if (p_grid.material[i] == p_material && p_grid.volume[i] > 0.01f) {
			return true;
		}
	}
	return false;
}

bool FireSolver::try_spread_to_fuel(WorldGrid &p_grid, int32_t p_x, int32_t p_y) {
	const int32_t dirs[4][2] = {
		{ -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 },
	};
	bool fed = false;
	for (const auto &dir : dirs) {
		const int32_t x = p_x + dir[0];
		const int32_t y = p_y + dir[1];
		if (!p_grid.in_bounds(x, y)) {
			continue;
		}
		const int32_t i = p_grid.cell_index(x, y);
		const uint8_t mat = p_grid.material[i];
		if (mat == MATERIAL_OIL || mat == MATERIAL_FLAMMABLE_GAS) {
			if ((next_random_u32() & 3u) != 0u) {
				p_grid.make_fire(x, y, 1.0f);
				fed = true;
			}
		} else if (mat == MATERIAL_TOXIC) {
			p_grid.make_toxic_gas(x, y, std::max(0.25f, p_grid.volume[i]));
			fed = true;
		}
	}
	return fed;
}

void FireSolver::step(WorldGrid &p_grid) {
	fire_order = p_grid.active_fire_cells;
	if (fire_order.empty()) {
		return;
	}
	const int32_t width = p_grid.width;
	const int32_t dirs[4][2] = {
		{ -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 },
	};

	for (int32_t idx : fire_order) {
		if (idx < 0 || idx >= p_grid.cell_count() || p_grid.material[idx] != MATERIAL_FIRE) {
			continue;
		}
		const int32_t x = idx % width;
		const int32_t y = idx / width;

		if (touches_liquid_material(p_grid, x, y, MATERIAL_WATER)) {
			p_grid.make_steam(x, y, 0.80f);
			continue;
		}

		p_grid.lifetime[idx] += 1.0f;
		p_grid.temperature[idx] = std::max(p_grid.temperature[idx], heat_source);
		bool fed = try_spread_to_fuel(p_grid, x, y);

		for (const auto &dir : dirs) {
			const int32_t nx = x + dir[0];
			const int32_t ny = y + dir[1];
			if (!p_grid.in_bounds(nx, ny)) {
				continue;
			}
			const int32_t ni = p_grid.cell_index(nx, ny);
			if (p_grid.material[ni] == MATERIAL_AIR || get_material_def(p_grid.material[ni]).gas) {
				p_grid.temperature[ni] = std::max(p_grid.temperature[ni], neighbor_heat * 0.55f);
			} else {
				p_grid.temperature[ni] = clampf(p_grid.temperature[ni] + neighbor_heat, 0.0f, 2.0f);
			}
		}

		const float life_limit = fed ? max_lifetime * 1.8f : max_lifetime;
		p_grid.temperature[idx] = std::max(0.0f, p_grid.temperature[idx] - cooling_per_step);
		if (p_grid.lifetime[idx] > life_limit || p_grid.temperature[idx] < water_extinguish_temperature) {
			p_grid.make_smoke(x, y, 0.75f);
		}
	}
}
