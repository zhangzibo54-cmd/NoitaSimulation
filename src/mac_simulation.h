#pragma once

#include "gas_solver.h"
#include "fire_solver.h"
#include "mac_fluid_solver.h"
#include "material_reaction_solver.h"
#include "material_defs.h"
#include "powder_solver.h"
#include "rigid_body_solver.h"

#include <cstdint>
#include <vector>

class MacSimulation {
public:
	// Backward-compatible names used by MacWorld/GDScript tools.
	static constexpr uint8_t MATERIAL_AIR = noita::MATERIAL_AIR;
	static constexpr uint8_t MATERIAL_ROCK = noita::MATERIAL_ROCK;
	static constexpr uint8_t MATERIAL_WATER = noita::MATERIAL_WATER;
	static constexpr uint8_t MATERIAL_SAND = noita::MATERIAL_SAND;
	static constexpr uint8_t MATERIAL_SMOKE = noita::MATERIAL_SMOKE;
	static constexpr uint8_t MATERIAL_TOXIC = noita::MATERIAL_TOXIC;
	static constexpr uint8_t MATERIAL_OIL = noita::MATERIAL_OIL;
	static constexpr uint8_t MATERIAL_FIRE = noita::MATERIAL_FIRE;
	static constexpr uint8_t MATERIAL_STEAM = noita::MATERIAL_STEAM;
	static constexpr uint8_t MATERIAL_TOXIC_GAS = noita::MATERIAL_TOXIC_GAS;
	static constexpr uint8_t MATERIAL_FLAMMABLE_GAS = noita::MATERIAL_FLAMMABLE_GAS;
	static constexpr uint8_t MATERIAL_GLASS = noita::MATERIAL_GLASS;

private:
	WorldGrid grid;
	PowderSolver powder_solver;
	GasSolver gas_solver;
	FireSolver fire_solver;
	MaterialReactionSolver reaction_solver;
	RigidBodySolver rigid_solver;
	MacFluidSolver fluid_solver;
	bool powder_present = false;
	bool gas_present = false;
	bool fire_present = false;
	bool rigid_paint_stroke_active = false;

	void resolve_fluid_powder_interactions();

public:
	MacSimulation();

	void set_world_size(int32_t p_width, int32_t p_height);
	int32_t get_width() const;
	int32_t get_height() const;

	void set_dt(double p_dt);
	double get_dt() const;
	void set_gravity(double p_gravity);
	double get_gravity() const;
	void set_viscosity(double p_viscosity);
	double get_viscosity() const;
	void set_pressure_iterations(int32_t p_iterations);
	int32_t get_pressure_iterations() const;
	void set_pressure_active_mass(double p_mass);
	double get_pressure_active_mass() const;
	void set_density_correction_strength(double p_strength);
	double get_density_correction_strength() const;
	void set_underfill_correction_strength(double p_strength);
	double get_underfill_correction_strength() const;

	void step();
	void clear();
	void generate_basin();
	void generate_rigid_collision_test();
	void begin_rigid_paint_stroke();
	void end_rigid_paint_stroke();
	void paint_circle(double p_x, double p_y, double p_radius, int32_t p_material);
	void inject_water(double p_x, double p_y, double p_radius, double p_mass_per_cell, double p_velocity_x, double p_velocity_y);
	void fill_rgba_pixels(std::vector<uint8_t> &p_pixels) const;
	bool start_rigid_drag(double p_x, double p_y);
	void update_rigid_drag(double p_x, double p_y, bool p_rotate);
	void end_rigid_drag();
	int32_t get_rigid_body_count() const;
	int32_t get_rigid_awake_count() const;
	int32_t get_rigid_sleeping_count() const;

	double get_total_water_mass() const;
	int64_t get_water_cell_count() const;
	double get_average_water_mass() const;
	double get_last_step_ms() const;
	int64_t get_step_count() const;
	int32_t get_last_pcg_iterations() const;
	double get_last_pcg_residual() const;
};
