#pragma once

#include "mac_simulation.h"

#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/classes/image_texture.hpp>
#include <godot_cpp/classes/node2d.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>

#include <cstdint>
#include <vector>

using namespace godot;

class MacWorld : public Node2D {
	GDCLASS(MacWorld, Node2D)

public:
	enum Material : uint8_t {
		MATERIAL_AIR = MacSimulation::MATERIAL_AIR,
		MATERIAL_ROCK = MacSimulation::MATERIAL_ROCK,
		MATERIAL_WATER = MacSimulation::MATERIAL_WATER,
		MATERIAL_SAND = MacSimulation::MATERIAL_SAND,
		MATERIAL_SMOKE = MacSimulation::MATERIAL_SMOKE,
		MATERIAL_TOXIC = MacSimulation::MATERIAL_TOXIC,
		MATERIAL_OIL = MacSimulation::MATERIAL_OIL,
		MATERIAL_FIRE = MacSimulation::MATERIAL_FIRE,
		MATERIAL_STEAM = MacSimulation::MATERIAL_STEAM,
		MATERIAL_TOXIC_GAS = MacSimulation::MATERIAL_TOXIC_GAS,
		MATERIAL_FLAMMABLE_GAS = MacSimulation::MATERIAL_FLAMMABLE_GAS,
		MATERIAL_GLASS = MacSimulation::MATERIAL_GLASS,
	};

private:
	MacSimulation sim;

	float display_scale = 3.0f;
	bool paused = false;
	float simulation_speed = 1.0f;
	float step_accumulator = 0.0f;
	double last_sim_ms = 0.0;
	double last_fill_ms = 0.0;
	double last_texture_ms = 0.0;
	double last_frame_sim_ms = 0.0;
	int32_t last_frame_sim_steps = 0;

	std::vector<uint8_t> rgba_pixels;
	PackedByteArray pixels;
	Ref<Image> image;
	Ref<ImageTexture> texture;

	void run_sim_step();
	void update_texture();

protected:
	static void _bind_methods();

public:
	MacWorld();
	~MacWorld() override = default;

	void _ready() override;
	void _process(double p_delta) override;
	void _draw() override;

	void set_world_size(int32_t p_width, int32_t p_height);
	int32_t get_width() const;
	int32_t get_height() const;
	void set_display_scale(double p_scale);
	double get_display_scale() const;

	void set_paused(bool p_paused);
	bool is_paused() const;
	void set_simulation_speed(double p_speed);
	double get_simulation_speed() const;
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
	double get_last_sim_ms() const;
	double get_last_fill_ms() const;
	double get_last_texture_ms() const;
	double get_last_frame_sim_ms() const;
	int32_t get_last_frame_sim_steps() const;
	int64_t get_step_count() const;
	int32_t get_last_pcg_iterations() const;
	double get_last_pcg_residual() const;
};
