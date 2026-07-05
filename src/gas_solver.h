#pragma once

#include "world_grid.h"

#include <cstdint>
#include <vector>

class GasSolver {
private:
	float diffusion_speed = 0.045f;
	float velocity_damping = 0.92f;
	float diffusion_fraction = 0.045f;
	float min_mass = 0.004f;
	float max_lifetime = 1600.0f;
	float max_velocity = 2.0f;
	uint32_t rng_state = 0x87654321u;
	std::vector<int32_t> gas_order;

	uint32_t next_random_u32();
	bool frame_scans_left_to_right();
	bool can_gas_enter(const WorldGrid &p_grid, int32_t p_x, int32_t p_y) const;
	bool can_rise_into(const WorldGrid &p_grid, int32_t p_from, int32_t p_to_x, int32_t p_to_y) const;
	bool dissolve_toxic_gas_into_liquid(WorldGrid &p_grid, int32_t p_x, int32_t p_y);
	void add_gas_mass(WorldGrid &p_grid, int32_t p_to_x, int32_t p_to_y, uint8_t p_material, float p_amount, float p_vx, float p_vy, float p_lifetime, float p_temperature);
	float diffuse_gas_mass(WorldGrid &p_grid, int32_t p_to_x, int32_t p_to_y, uint8_t p_material, float p_amount, float p_vx, float p_vy, float p_lifetime, float p_temperature);
	float gas_max_lifetime(uint8_t p_material) const;
	float gas_up_multiplier(uint8_t p_material) const;

public:
	void step(WorldGrid &p_grid);
};

