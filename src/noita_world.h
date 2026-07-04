#pragma once

#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/classes/image_texture.hpp>
#include <godot_cpp/classes/node2d.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>
#include <godot_cpp/variant/string.hpp>

#include <cstdint>
#include <vector>

using namespace godot;

class NoitaWorld : public Node2D {
	GDCLASS(NoitaWorld, Node2D)

public:
	enum Material : uint8_t {
		MATERIAL_AIR = 0,
		MATERIAL_ROCK = 1,
		MATERIAL_WATER = 2,
	};

private:
	struct Cell {
		uint8_t material = MATERIAL_AIR;
		float mass = 0.0f;
		float momentum_x = 0.0f;
		float momentum_y = 0.0f;
	};

	int32_t width = 320;
	int32_t height = 180;
	float display_scale = 3.0f;
	bool paused = false;
	float simulation_speed = 1.0f;
	float step_accumulator = 0.0f;

	float gravity = 0.28f;
	float max_velocity = 5.0f;
	float max_stable_mass = 1.05f;
	float pressure_release_fraction = 1.0f;
	int32_t pressure_release_radius = 1;
	float pressure_distance_decay = 0.1f;
	float pressure_density_speed = 1.0f;
	float pressure_target_max_mass = 1.3f;
	float pressure_impulse_strength = 1.0f;
	float velocity_damping = 0.985f;
	int32_t pressure_iterations = 8;

	double last_step_ms = 0.0;
	uint64_t step_count = 0;

	std::vector<Cell> cells;
	std::vector<Cell> next_cells;
	std::vector<float> delta_mass;
	std::vector<float> delta_momentum_x;
	std::vector<float> delta_momentum_y;

	PackedByteArray pixels;
	Ref<Image> image;
	Ref<ImageTexture> texture;

	int32_t index(int32_t p_x, int32_t p_y) const;
	bool in_bounds(int32_t p_x, int32_t p_y) const;
	bool is_rock(int32_t p_x, int32_t p_y) const;
	bool is_blocked_for_motion(int32_t p_x, int32_t p_y) const;
	void ensure_buffers();
	void clear_dynamic_buffers();
	void add_water_to_cell(int32_t p_idx, float p_mass, float p_momentum_x, float p_momentum_y);
	void advect_with_overlap();
	void trace_motion(int32_t p_x, int32_t p_y, float p_vx, float p_vy, int32_t &r_x, int32_t &r_y, float &r_vx, float &r_vy) const;
	void pressure_relax_once();
	void damp_and_sanitize();
	void update_texture();
	void make_rock(int32_t p_x, int32_t p_y);
	void make_air(int32_t p_x, int32_t p_y);
	void make_water(int32_t p_x, int32_t p_y, float p_mass = 1.0f);

protected:
	static void _bind_methods();

public:
	NoitaWorld();
	~NoitaWorld() override = default;

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

	void set_pressure_iterations(int32_t p_iterations);
	int32_t get_pressure_iterations() const;
	void set_pressure_release_radius(int32_t p_radius);
	int32_t get_pressure_release_radius() const;
	void set_pressure_distance_decay(double p_decay);
	double get_pressure_distance_decay() const;
	void set_pressure_density_speed(double p_speed);
	double get_pressure_density_speed() const;
	void set_pressure_target_max_mass(double p_mass);
	double get_pressure_target_max_mass() const;
	void set_pressure_impulse_strength(double p_strength);
	double get_pressure_impulse_strength() const;
	void set_gravity(double p_gravity);
	double get_gravity() const;

	void step();
	void clear();
	void generate_basin();
	void paint_circle(double p_x, double p_y, double p_radius, int32_t p_material);
	void inject_water(double p_x, double p_y, double p_radius, double p_mass_per_cell, double p_velocity_x, double p_velocity_y);

	bool load_from_png(const String &p_path);
	bool save_to_file(const String &p_path) const;
	bool load_from_file(const String &p_path);

	double get_total_water_mass() const;
	int64_t get_water_cell_count() const;
	double get_average_water_mass() const;
	double get_last_step_ms() const;
	int64_t get_step_count() const;
};
