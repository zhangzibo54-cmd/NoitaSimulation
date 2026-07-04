#pragma once

#include <cstdint>
#include <vector>

class MacSimulation {
public:
	enum Material : uint8_t {
		MATERIAL_AIR = 0,
		MATERIAL_ROCK = 1,
		MATERIAL_WATER = 2,
	};

private:
	int32_t width = 320;
	int32_t height = 180;

	float dt = 0.18f;
	float gravity = 0.75f;
	float viscosity = 0.04f;
	float density = 1.0f;
	float target_mass = 1.0f;
	float pressure_active_mass = 0.30f;
	float density_correction_strength = 0.5f;
	float underfill_correction_strength = 0.2f;
	float velocity_damping = 0.998f;
	float max_velocity = 6.0f;
	int32_t pressure_iterations = 20;

	double last_step_ms = 0.0;
	uint64_t step_count = 0;
	int32_t last_pcg_iterations = 0;
	double last_pcg_residual = 0.0;

	std::vector<uint8_t> material;
	std::vector<float> mass;
	std::vector<float> next_mass;
	std::vector<float> pressure;
	std::vector<float> rhs;
	std::vector<float> residual;
	std::vector<float> direction;
	std::vector<float> q_vec;
	std::vector<float> z_vec;
	std::vector<float> diag_inv;
	std::vector<int32_t> active_cells;
	std::vector<float> outflow_mass;

	// MAC staggered velocities:
	// u(x,y), x=0..width, y=0..height-1, is the horizontal velocity on the
	// vertical face between cells x-1 and x.
	// v(x,y), x=0..width-1, y=0..height, is the vertical velocity on the
	// horizontal face between cells y-1 and y.
	std::vector<float> u;
	std::vector<float> v;
	std::vector<float> u_tmp;
	std::vector<float> v_tmp;
	std::vector<float> u_alpha;
	std::vector<float> v_alpha;
	std::vector<float> u_mass_flux;
	std::vector<float> v_mass_flux;

	int32_t cell_index(int32_t p_x, int32_t p_y) const;
	int32_t u_index(int32_t p_x, int32_t p_y) const;
	int32_t v_index(int32_t p_x, int32_t p_y) const;
	bool in_bounds(int32_t p_x, int32_t p_y) const;
	bool is_solid_cell(int32_t p_x, int32_t p_y) const;
	bool is_liquid_cell(int32_t p_x, int32_t p_y) const;
	bool is_internal_liquid_cell(int32_t p_x, int32_t p_y) const;
	bool is_pressure_active_cell(int32_t p_x, int32_t p_y) const;
	bool u_face_blocked(int32_t p_x, int32_t p_y) const;
	bool v_face_blocked(int32_t p_x, int32_t p_y) const;
	float cell_density(int32_t p_x, int32_t p_y) const;
	float u_face_alpha(int32_t p_x, int32_t p_y) const;
	float v_face_alpha(int32_t p_x, int32_t p_y) const;

	void ensure_buffers();
	void clear_fields();
	void apply_solid_boundaries(std::vector<float> &p_u, std::vector<float> &p_v);
	float sample_u_clamped(const std::vector<float> &p_u, int32_t p_x, int32_t p_y) const;
	float sample_v_clamped(const std::vector<float> &p_v, int32_t p_x, int32_t p_y) const;
	float vertical_velocity_at_u_face(const std::vector<float> &p_v, int32_t p_x, int32_t p_y) const;
	float horizontal_velocity_at_v_face(const std::vector<float> &p_u, int32_t p_x, int32_t p_y) const;
	void predict_velocity_explicit();
	void build_pressure_system();
	void apply_laplacian(const std::vector<float> &p_x, std::vector<float> &r_ax) const;
	double dot_liquid(const std::vector<float> &p_a, const std::vector<float> &p_b) const;
	void solve_pressure_pcg();
	void apply_pressure_projection();
	void advect_mass_finite_volume();
	void clamp_velocities();

	void make_rock(int32_t p_x, int32_t p_y);
	void make_air(int32_t p_x, int32_t p_y);
	void make_water(int32_t p_x, int32_t p_y, float p_mass = 1.0f);

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
	void paint_circle(double p_x, double p_y, double p_radius, int32_t p_material);
	void inject_water(double p_x, double p_y, double p_radius, double p_mass_per_cell, double p_velocity_x, double p_velocity_y);
	void fill_rgba_pixels(std::vector<uint8_t> &p_pixels) const;

	double get_total_water_mass() const;
	int64_t get_water_cell_count() const;
	double get_average_water_mass() const;
	double get_last_step_ms() const;
	int64_t get_step_count() const;
	int32_t get_last_pcg_iterations() const;
	double get_last_pcg_residual() const;
};
