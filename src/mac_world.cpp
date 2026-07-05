#include "mac_world.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/rect2.hpp>

#include <algorithm>
#include <chrono>
#include <cstring>

namespace {
float clampf(float v, float lo, float hi) {
	return std::max(lo, std::min(v, hi));
}

double elapsed_ms(std::chrono::steady_clock::time_point p_start, std::chrono::steady_clock::time_point p_end) {
	return std::chrono::duration<double, std::milli>(p_end - p_start).count();
}
} // namespace

MacWorld::MacWorld() = default;

void MacWorld::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_world_size", "width", "height"), &MacWorld::set_world_size);
	ClassDB::bind_method(D_METHOD("get_width"), &MacWorld::get_width);
	ClassDB::bind_method(D_METHOD("get_height"), &MacWorld::get_height);
	ClassDB::bind_method(D_METHOD("set_display_scale", "scale"), &MacWorld::set_display_scale);
	ClassDB::bind_method(D_METHOD("get_display_scale"), &MacWorld::get_display_scale);
	ClassDB::bind_method(D_METHOD("set_paused", "paused"), &MacWorld::set_paused);
	ClassDB::bind_method(D_METHOD("is_paused"), &MacWorld::is_paused);
	ClassDB::bind_method(D_METHOD("set_simulation_speed", "speed"), &MacWorld::set_simulation_speed);
	ClassDB::bind_method(D_METHOD("get_simulation_speed"), &MacWorld::get_simulation_speed);
	ClassDB::bind_method(D_METHOD("set_dt", "dt"), &MacWorld::set_dt);
	ClassDB::bind_method(D_METHOD("get_dt"), &MacWorld::get_dt);
	ClassDB::bind_method(D_METHOD("set_gravity", "gravity"), &MacWorld::set_gravity);
	ClassDB::bind_method(D_METHOD("get_gravity"), &MacWorld::get_gravity);
	ClassDB::bind_method(D_METHOD("set_viscosity", "viscosity"), &MacWorld::set_viscosity);
	ClassDB::bind_method(D_METHOD("get_viscosity"), &MacWorld::get_viscosity);
	ClassDB::bind_method(D_METHOD("set_pressure_iterations", "iterations"), &MacWorld::set_pressure_iterations);
	ClassDB::bind_method(D_METHOD("get_pressure_iterations"), &MacWorld::get_pressure_iterations);
	ClassDB::bind_method(D_METHOD("set_pressure_active_mass", "mass"), &MacWorld::set_pressure_active_mass);
	ClassDB::bind_method(D_METHOD("get_pressure_active_mass"), &MacWorld::get_pressure_active_mass);
	ClassDB::bind_method(D_METHOD("set_density_correction_strength", "strength"), &MacWorld::set_density_correction_strength);
	ClassDB::bind_method(D_METHOD("get_density_correction_strength"), &MacWorld::get_density_correction_strength);
	ClassDB::bind_method(D_METHOD("set_underfill_correction_strength", "strength"), &MacWorld::set_underfill_correction_strength);
	ClassDB::bind_method(D_METHOD("get_underfill_correction_strength"), &MacWorld::get_underfill_correction_strength);

	ClassDB::bind_method(D_METHOD("step"), &MacWorld::step);
	ClassDB::bind_method(D_METHOD("clear"), &MacWorld::clear);
	ClassDB::bind_method(D_METHOD("generate_basin"), &MacWorld::generate_basin);
	ClassDB::bind_method(D_METHOD("generate_rigid_collision_test"), &MacWorld::generate_rigid_collision_test);
	ClassDB::bind_method(D_METHOD("begin_rigid_paint_stroke"), &MacWorld::begin_rigid_paint_stroke);
	ClassDB::bind_method(D_METHOD("end_rigid_paint_stroke"), &MacWorld::end_rigid_paint_stroke);
	ClassDB::bind_method(D_METHOD("paint_circle", "x", "y", "radius", "material"), &MacWorld::paint_circle);
	ClassDB::bind_method(D_METHOD("inject_water", "x", "y", "radius", "mass_per_cell", "velocity_x", "velocity_y"), &MacWorld::inject_water);
	ClassDB::bind_method(D_METHOD("start_rigid_drag", "x", "y"), &MacWorld::start_rigid_drag);
	ClassDB::bind_method(D_METHOD("update_rigid_drag", "x", "y", "rotate"), &MacWorld::update_rigid_drag);
	ClassDB::bind_method(D_METHOD("end_rigid_drag"), &MacWorld::end_rigid_drag);
	ClassDB::bind_method(D_METHOD("get_rigid_body_count"), &MacWorld::get_rigid_body_count);
	ClassDB::bind_method(D_METHOD("get_rigid_awake_count"), &MacWorld::get_rigid_awake_count);
	ClassDB::bind_method(D_METHOD("get_rigid_sleeping_count"), &MacWorld::get_rigid_sleeping_count);
	ClassDB::bind_method(D_METHOD("get_total_water_mass"), &MacWorld::get_total_water_mass);
	ClassDB::bind_method(D_METHOD("get_water_cell_count"), &MacWorld::get_water_cell_count);
	ClassDB::bind_method(D_METHOD("get_average_water_mass"), &MacWorld::get_average_water_mass);
	ClassDB::bind_method(D_METHOD("get_last_step_ms"), &MacWorld::get_last_step_ms);
	ClassDB::bind_method(D_METHOD("get_last_sim_ms"), &MacWorld::get_last_sim_ms);
	ClassDB::bind_method(D_METHOD("get_last_fill_ms"), &MacWorld::get_last_fill_ms);
	ClassDB::bind_method(D_METHOD("get_last_texture_ms"), &MacWorld::get_last_texture_ms);
	ClassDB::bind_method(D_METHOD("get_last_frame_sim_ms"), &MacWorld::get_last_frame_sim_ms);
	ClassDB::bind_method(D_METHOD("get_last_frame_sim_steps"), &MacWorld::get_last_frame_sim_steps);
	ClassDB::bind_method(D_METHOD("get_step_count"), &MacWorld::get_step_count);
	ClassDB::bind_method(D_METHOD("get_last_pcg_iterations"), &MacWorld::get_last_pcg_iterations);
	ClassDB::bind_method(D_METHOD("get_last_pcg_residual"), &MacWorld::get_last_pcg_residual);

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "display_scale"), "set_display_scale", "get_display_scale");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "paused"), "set_paused", "is_paused");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "simulation_speed"), "set_simulation_speed", "get_simulation_speed");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "dt"), "set_dt", "get_dt");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "gravity"), "set_gravity", "get_gravity");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "viscosity"), "set_viscosity", "get_viscosity");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "pressure_iterations"), "set_pressure_iterations", "get_pressure_iterations");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "pressure_active_mass"), "set_pressure_active_mass", "get_pressure_active_mass");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "density_correction_strength"), "set_density_correction_strength", "get_density_correction_strength");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "underfill_correction_strength"), "set_underfill_correction_strength", "get_underfill_correction_strength");
}

void MacWorld::_ready() {
	update_texture();
	queue_redraw();
}

void MacWorld::_process(double p_delta) {
	if (paused) {
		return;
	}
	step_accumulator += static_cast<float>(p_delta * 60.0 * simulation_speed);
	int32_t steps_this_frame = 0;
	double sim_ms_this_frame = 0.0;
	while (step_accumulator >= 1.0f && steps_this_frame < 6) {
		run_sim_step();
		sim_ms_this_frame += last_sim_ms;
		step_accumulator -= 1.0f;
		steps_this_frame++;
	}
	if (steps_this_frame == 6) {
		step_accumulator = 0.0f;
	}
	last_frame_sim_steps = steps_this_frame;
	last_frame_sim_ms = sim_ms_this_frame;
	if (steps_this_frame > 0) {
		update_texture();
		queue_redraw();
	}
}

void MacWorld::_draw() {
	if (texture.is_valid()) {
		draw_texture_rect(texture, Rect2(0, 0, sim.get_width() * display_scale, sim.get_height() * display_scale), false);
	}
}

void MacWorld::run_sim_step() {
	const auto sim_start = std::chrono::steady_clock::now();
	sim.step();
	const auto sim_end = std::chrono::steady_clock::now();
	last_sim_ms = elapsed_ms(sim_start, sim_end);
}

void MacWorld::update_texture() {
	const auto fill_start = std::chrono::steady_clock::now();
	sim.fill_rgba_pixels(rgba_pixels);
	const auto fill_end = std::chrono::steady_clock::now();
	last_fill_ms = elapsed_ms(fill_start, fill_end);
	if (rgba_pixels.empty()) {
		last_texture_ms = 0.0;
		return;
	}

	const auto texture_start = std::chrono::steady_clock::now();
	pixels.resize(static_cast<int64_t>(rgba_pixels.size()));
	std::memcpy(pixels.ptrw(), rgba_pixels.data(), rgba_pixels.size());
	const int32_t width = sim.get_width();
	const int32_t height = sim.get_height();
	if (!image.is_valid()) {
		image = Image::create_from_data(width, height, false, Image::FORMAT_RGBA8, pixels);
	} else {
		image->set_data(width, height, false, Image::FORMAT_RGBA8, pixels);
	}
	if (!texture.is_valid()) {
		texture = ImageTexture::create_from_image(image);
	} else {
		texture->update(image);
	}
	const auto texture_end = std::chrono::steady_clock::now();
	last_texture_ms = elapsed_ms(texture_start, texture_end);
}

void MacWorld::set_world_size(int32_t p_width, int32_t p_height) {
	sim.set_world_size(p_width, p_height);
	image.unref();
	texture.unref();
	update_texture();
	queue_redraw();
}
int32_t MacWorld::get_width() const { return sim.get_width(); }
int32_t MacWorld::get_height() const { return sim.get_height(); }
void MacWorld::set_display_scale(double p_scale) { display_scale = clampf(static_cast<float>(p_scale), 0.25f, 64.0f); queue_redraw(); }
double MacWorld::get_display_scale() const { return display_scale; }
void MacWorld::set_paused(bool p_paused) { paused = p_paused; }
bool MacWorld::is_paused() const { return paused; }
void MacWorld::set_simulation_speed(double p_speed) { simulation_speed = clampf(static_cast<float>(p_speed), 0.0f, 20.0f); }
double MacWorld::get_simulation_speed() const { return simulation_speed; }
void MacWorld::set_dt(double p_dt) { sim.set_dt(p_dt); }
double MacWorld::get_dt() const { return sim.get_dt(); }
void MacWorld::set_gravity(double p_gravity) { sim.set_gravity(p_gravity); }
double MacWorld::get_gravity() const { return sim.get_gravity(); }
void MacWorld::set_viscosity(double p_viscosity) { sim.set_viscosity(p_viscosity); }
double MacWorld::get_viscosity() const { return sim.get_viscosity(); }
void MacWorld::set_pressure_iterations(int32_t p_iterations) { sim.set_pressure_iterations(p_iterations); }
int32_t MacWorld::get_pressure_iterations() const { return sim.get_pressure_iterations(); }
void MacWorld::set_pressure_active_mass(double p_mass) { sim.set_pressure_active_mass(p_mass); }
double MacWorld::get_pressure_active_mass() const { return sim.get_pressure_active_mass(); }
void MacWorld::set_density_correction_strength(double p_strength) { sim.set_density_correction_strength(p_strength); }
double MacWorld::get_density_correction_strength() const { return sim.get_density_correction_strength(); }
void MacWorld::set_underfill_correction_strength(double p_strength) { sim.set_underfill_correction_strength(p_strength); }
double MacWorld::get_underfill_correction_strength() const { return sim.get_underfill_correction_strength(); }

void MacWorld::step() {
	run_sim_step();
	last_frame_sim_steps = 1;
	last_frame_sim_ms = last_sim_ms;
	update_texture();
	queue_redraw();
}

void MacWorld::clear() {
	sim.clear();
	update_texture();
	queue_redraw();
}

void MacWorld::generate_rigid_collision_test() {
	sim.generate_rigid_collision_test();
	update_texture();
	queue_redraw();
}

void MacWorld::begin_rigid_paint_stroke() {
	sim.begin_rigid_paint_stroke();
}

void MacWorld::end_rigid_paint_stroke() {
	sim.end_rigid_paint_stroke();
	update_texture();
	queue_redraw();
}

void MacWorld::generate_basin() {
	sim.generate_basin();
	update_texture();
	queue_redraw();
}

void MacWorld::paint_circle(double p_x, double p_y, double p_radius, int32_t p_material) {
	sim.paint_circle(p_x, p_y, p_radius, p_material);
	update_texture();
	queue_redraw();
}

void MacWorld::inject_water(double p_x, double p_y, double p_radius, double p_mass_per_cell, double p_velocity_x, double p_velocity_y) {
	sim.inject_water(p_x, p_y, p_radius, p_mass_per_cell, p_velocity_x, p_velocity_y);
	update_texture();
	queue_redraw();
}

bool MacWorld::start_rigid_drag(double p_x, double p_y) {
	return sim.start_rigid_drag(p_x, p_y);
}

void MacWorld::update_rigid_drag(double p_x, double p_y, bool p_rotate) {
	sim.update_rigid_drag(p_x, p_y, p_rotate);
	update_texture();
	queue_redraw();
}

void MacWorld::end_rigid_drag() {
	sim.end_rigid_drag();
}

int32_t MacWorld::get_rigid_body_count() const { return sim.get_rigid_body_count(); }
int32_t MacWorld::get_rigid_awake_count() const { return sim.get_rigid_awake_count(); }
int32_t MacWorld::get_rigid_sleeping_count() const { return sim.get_rigid_sleeping_count(); }

double MacWorld::get_total_water_mass() const { return sim.get_total_water_mass(); }
int64_t MacWorld::get_water_cell_count() const { return sim.get_water_cell_count(); }
double MacWorld::get_average_water_mass() const { return sim.get_average_water_mass(); }
double MacWorld::get_last_step_ms() const { return sim.get_last_step_ms(); }
double MacWorld::get_last_sim_ms() const { return last_sim_ms; }
double MacWorld::get_last_fill_ms() const { return last_fill_ms; }
double MacWorld::get_last_texture_ms() const { return last_texture_ms; }
double MacWorld::get_last_frame_sim_ms() const { return last_frame_sim_ms; }
int32_t MacWorld::get_last_frame_sim_steps() const { return last_frame_sim_steps; }
int64_t MacWorld::get_step_count() const { return sim.get_step_count(); }
int32_t MacWorld::get_last_pcg_iterations() const { return sim.get_last_pcg_iterations(); }
double MacWorld::get_last_pcg_residual() const { return sim.get_last_pcg_residual(); }
