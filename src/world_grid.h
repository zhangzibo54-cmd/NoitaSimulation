#pragma once

#include "material_defs.h"

#include <cstdint>
#include <vector>

class WorldGrid {
public:
	int32_t width = 320;
	int32_t height = 180;

	// Shared cell-centered state used by all solvers.
	std::vector<uint8_t> material;
	// Cell fill amount / occupied volume. This is not physical mass.
	std::vector<float> volume_fraction;
	// Physical density-like material amount. For liquids this is derived from
	// phase fractions; for sand/smoke it can store the old "amount" meaning.
	std::vector<float> density;
	std::vector<float> toxic;
	// Immiscible oil phase volume. Water phase volume is volume - oil.
	std::vector<float> oil;
	std::vector<float> pressure;
	std::vector<float> velocity_x;
	std::vector<float> velocity_y;
	std::vector<float> temperature;
	std::vector<float> lifetime;
	// Dynamic rigid-body overlay. These cells are not material cells, but moving
	// solid occupancy projected from free rigid chunks onto the simulation grid.
	std::vector<int32_t> rigid_body_id;
	std::vector<float> rigid_velocity_x;
	std::vector<float> rigid_velocity_y;
	std::vector<int32_t> active_liquid_cells;
	std::vector<int32_t> active_powder_cells;
	std::vector<int32_t> active_gas_cells;
	std::vector<int32_t> active_fire_cells;
	std::vector<int32_t> active_reaction_cells;

	void set_size(int32_t p_width, int32_t p_height);
	void clear();
	void clear_dynamic_fields();
	void clear_rigid_fields();

	int32_t cell_count() const;
	int32_t cell_index(int32_t p_x, int32_t p_y) const;
	bool in_bounds(int32_t p_x, int32_t p_y) const;

	const noita::MaterialDef &cell_material_def(int32_t p_x, int32_t p_y) const;
	bool is_solid_cell(int32_t p_x, int32_t p_y) const;
	bool has_dynamic_rigid_cell(int32_t p_x, int32_t p_y) const;
	bool blocks_velocity(int32_t p_x, int32_t p_y) const;
	bool is_liquid_cell(int32_t p_x, int32_t p_y, float p_mass_epsilon) const;
	bool is_powder_cell(int32_t p_x, int32_t p_y) const;
	bool is_gas_cell(int32_t p_x, int32_t p_y) const;
	float cell_density(int32_t p_x, int32_t p_y, float p_density_scale = 1.0f) const;
	void rebuild_active_cells(float p_liquid_mass_epsilon);

	void make_rock(int32_t p_x, int32_t p_y);
	void make_air(int32_t p_x, int32_t p_y);
	void make_water(int32_t p_x, int32_t p_y, float p_mass);
	void make_toxic(int32_t p_x, int32_t p_y, float p_mass = 1.0f);
	void make_oil(int32_t p_x, int32_t p_y, float p_mass = 1.0f);
	void make_sand(int32_t p_x, int32_t p_y, float p_mass = 1.0f);
	void make_smoke(int32_t p_x, int32_t p_y, float p_mass = 1.0f);
	void make_fire(int32_t p_x, int32_t p_y, float p_heat = 1.0f);
	void make_steam(int32_t p_x, int32_t p_y, float p_mass = 1.0f);
	void make_toxic_gas(int32_t p_x, int32_t p_y, float p_mass = 1.0f);
	void make_flammable_gas(int32_t p_x, int32_t p_y, float p_mass = 1.0f);
	void make_glass(int32_t p_x, int32_t p_y);
};

