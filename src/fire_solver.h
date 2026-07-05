#pragma once

#include "world_grid.h"

#include <cstdint>
#include <vector>

class FireSolver {
private:
	float heat_source = 1.0f;
	float neighbor_heat = 0.18f;
	float cooling_per_step = 0.010f;
	float max_lifetime = 95.0f;
	float water_extinguish_temperature = 0.28f;
	uint32_t rng_state = 0x13572468u;
	std::vector<int32_t> fire_order;

	uint32_t next_random_u32();
	bool touches_liquid_material(const WorldGrid &p_grid, int32_t p_x, int32_t p_y, uint8_t p_material) const;
	bool try_spread_to_fuel(WorldGrid &p_grid, int32_t p_x, int32_t p_y);

public:
	void step(WorldGrid &p_grid);
};

