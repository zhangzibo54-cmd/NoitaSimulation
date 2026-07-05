#pragma once

#include "world_grid.h"

#include <cstdint>
#include <vector>

class PowderSolver {
private:
	float gravity = 0.45f;
	float velocity_damping = 0.96f;
	float max_velocity = 3.0f;
	uint32_t rng_state = 0x12345678u;
	std::vector<int32_t> powder_order;

	uint32_t next_random_u32();
	bool frame_scans_left_to_right();
	bool can_powder_enter(const WorldGrid &p_grid, int32_t p_x, int32_t p_y) const;
	bool try_move_sand(WorldGrid &p_grid, int32_t p_from_x, int32_t p_from_y, int32_t p_to_x, int32_t p_to_y, float p_new_vx, float p_new_vy);

public:
	void step(WorldGrid &p_grid);
};
