#include "mac_world.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/rect2.hpp>

#include <algorithm>
#include <cstring>

namespace {
float clampf(float v, float lo, float hi) {
	return std::max(lo, std::min(v, hi));
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
	ClassDB::bind_method(D_METHOD("paint_circle", "x", "y", "radius", "material"), &MacWorld::paint_circle);
	ClassDB::bind_method(D_METHOD("inject_water", "x", "y", "radius", "mass_per_cell", "velocity_x", "velocity_y"), &MacWorld::inject_water);
	ClassDB::bind_method(D_METHOD("get_total_water_mass"), &MacWorld::get_total_water_mass);
	ClassDB::bind_method(D_METHOD("get_water_cell_count"), &MacWorld::get_water_cell_count);
	ClassDB::bind_method(D_METHOD("get_average_water_mass"), &MacWorld::get_average_water_mass);
	ClassDB::bind_method(D_METHOD("get_last_step_ms"), &MacWorld::get_last_step_ms);
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
	while (step_accumulator >= 1.0f && steps_this_frame < 6) {
		step();
		step_accumulator -= 1.0f;
		steps_this_frame++;
	}
	if (steps_this_frame == 6) {
		step_accumulator = 0.0f;
	}
}

void MacWorld::_draw() {
	if (texture.is_valid()) {
		draw_texture_rect(texture, Rect2(0, 0, sim.get_width() * display_scale, sim.get_height() * display_scale), false);
	}
}

void MacWorld::update_texture() {
	sim.fill_rgba_pixels(rgba_pixels);
	if (rgba_pixels.empty()) {
		return;
	}
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
	sim.step();
	update_texture();
	queue_redraw();
}

void MacWorld::clear() {
	sim.clear();
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

double MacWorld::get_total_water_mass() const { return sim.get_total_water_mass(); }
int64_t MacWorld::get_water_cell_count() const { return sim.get_water_cell_count(); }
double MacWorld::get_average_water_mass() const { return sim.get_average_water_mass(); }
double MacWorld::get_last_step_ms() const { return sim.get_last_step_ms(); }
int64_t MacWorld::get_step_count() const { return sim.get_step_count(); }
int32_t MacWorld::get_last_pcg_iterations() const { return sim.get_last_pcg_iterations(); }
double MacWorld::get_last_pcg_residual() const { return sim.get_last_pcg_residual(); }
