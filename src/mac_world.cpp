#include "mac_world.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/rect2.hpp>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <thread>

namespace {
float clampf(float v, float lo, float hi) {
	return std::max(lo, std::min(v, hi));
}

double elapsed_ms(std::chrono::steady_clock::time_point p_start, std::chrono::steady_clock::time_point p_end) {
	return std::chrono::duration<double, std::milli>(p_end - p_start).count();
}
} // namespace

MacWorld::MacWorld() = default;

MacWorld::~MacWorld() {
	stop_simulation_thread();
}

void MacWorld::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_world_size", "width", "height"), &MacWorld::set_world_size);
	ClassDB::bind_method(D_METHOD("get_width"), &MacWorld::get_width);
	ClassDB::bind_method(D_METHOD("get_height"), &MacWorld::get_height);
	ClassDB::bind_method(D_METHOD("set_display_scale", "scale"), &MacWorld::set_display_scale);
	ClassDB::bind_method(D_METHOD("get_display_scale"), &MacWorld::get_display_scale);
	ClassDB::bind_method(D_METHOD("set_paused", "paused"), &MacWorld::set_paused);
	ClassDB::bind_method(D_METHOD("is_paused"), &MacWorld::is_paused);
	ClassDB::bind_method(D_METHOD("set_threaded_simulation_enabled", "enabled"), &MacWorld::set_threaded_simulation_enabled);
	ClassDB::bind_method(D_METHOD("is_threaded_simulation_enabled"), &MacWorld::is_threaded_simulation_enabled);
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
	ClassDB::bind_method(D_METHOD("set_rigid_liquid_impulse_strength", "strength"), &MacWorld::set_rigid_liquid_impulse_strength);
	ClassDB::bind_method(D_METHOD("get_rigid_liquid_impulse_strength"), &MacWorld::get_rigid_liquid_impulse_strength);

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
	ClassDB::bind_method(D_METHOD("get_last_predict_ms"), &MacWorld::get_last_predict_ms);
	ClassDB::bind_method(D_METHOD("get_last_build_ms"), &MacWorld::get_last_build_ms);
	ClassDB::bind_method(D_METHOD("get_last_pcg_ms"), &MacWorld::get_last_pcg_ms);
	ClassDB::bind_method(D_METHOD("get_last_project_ms"), &MacWorld::get_last_project_ms);
	ClassDB::bind_method(D_METHOD("get_last_advect_ms"), &MacWorld::get_last_advect_ms);
	ClassDB::bind_method(D_METHOD("get_last_clamp_ms"), &MacWorld::get_last_clamp_ms);
	ClassDB::bind_method(D_METHOD("get_active_region_min_x"), &MacWorld::get_active_region_min_x);
	ClassDB::bind_method(D_METHOD("get_active_region_min_y"), &MacWorld::get_active_region_min_y);
	ClassDB::bind_method(D_METHOD("get_active_region_max_x"), &MacWorld::get_active_region_max_x);
	ClassDB::bind_method(D_METHOD("get_active_region_max_y"), &MacWorld::get_active_region_max_y);
	ClassDB::bind_method(D_METHOD("get_active_region_pad"), &MacWorld::get_active_region_pad);
	ClassDB::bind_method(D_METHOD("get_active_region_max_speed"), &MacWorld::get_active_region_max_speed);
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
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "threaded_simulation_enabled"), "set_threaded_simulation_enabled", "is_threaded_simulation_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "simulation_speed"), "set_simulation_speed", "get_simulation_speed");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "dt"), "set_dt", "get_dt");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "gravity"), "set_gravity", "get_gravity");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "viscosity"), "set_viscosity", "get_viscosity");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "pressure_iterations"), "set_pressure_iterations", "get_pressure_iterations");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "pressure_active_mass"), "set_pressure_active_mass", "get_pressure_active_mass");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "density_correction_strength"), "set_density_correction_strength", "get_density_correction_strength");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "underfill_correction_strength"), "set_underfill_correction_strength", "get_underfill_correction_strength");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "rigid_liquid_impulse_strength"), "set_rigid_liquid_impulse_strength", "get_rigid_liquid_impulse_strength");
}

void MacWorld::_ready() {
	update_texture();
	queue_redraw();
}

void MacWorld::_process(double p_delta) {
	if (paused.load()) {
		return;
	}
	if (threaded_simulation_enabled.load()) {
		update_texture();
		queue_redraw();
		return;
	}
	double sim_dt = 1.0 / 30.0;
	float sim_speed = 1.0f;
	{
		std::lock_guard<std::mutex> lock(sim_mutex);
		sim_dt = sim.get_dt();
		sim_speed = simulation_speed;
	}
	const double target_interval = std::max(1.0 / 240.0, sim_dt);
	step_accumulator += static_cast<float>(p_delta * sim_speed);
	int32_t steps_this_frame = 0;
	double sim_ms_this_frame = 0.0;
	const auto frame_sim_start = std::chrono::steady_clock::now();
	while (elapsed_ms(frame_sim_start, std::chrono::steady_clock::now()) < fluid_budget_ms) {
		const auto sim_start = std::chrono::steady_clock::now();
		const double remaining_budget = std::max(0.0, fluid_budget_ms - elapsed_ms(frame_sim_start, std::chrono::steady_clock::now()));
		bool completed = false;
		{
			std::lock_guard<std::mutex> lock(sim_mutex);
			if (!sim.has_pending_budgeted_step()) {
				if (step_accumulator < target_interval) {
					break;
				}
				sim.begin_budgeted_step();
				step_accumulator -= static_cast<float>(target_interval);
			}
			completed = sim.advance_budgeted_step(remaining_budget);
		}
		const auto sim_end = std::chrono::steady_clock::now();
		last_sim_ms = elapsed_ms(sim_start, sim_end);
		sim_ms_this_frame += last_sim_ms;
		if (completed) {
			steps_this_frame++;
		} else {
			break;
		}
	}
	if (step_accumulator > static_cast<float>(target_interval * 4.0)) {
		step_accumulator = static_cast<float>(target_interval * 4.0);
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
		int32_t width = 0;
		int32_t height = 0;
		{
			std::lock_guard<std::mutex> lock(sim_mutex);
			width = sim.get_width();
			height = sim.get_height();
		}
		draw_texture_rect(texture, Rect2(0, 0, width * display_scale, height * display_scale), false);
	}
}

void MacWorld::run_sim_step() {
	const auto sim_start = std::chrono::steady_clock::now();
	{
		std::lock_guard<std::mutex> lock(sim_mutex);
		sim.step();
	}
	const auto sim_end = std::chrono::steady_clock::now();
	last_sim_ms = elapsed_ms(sim_start, sim_end);
}

void MacWorld::update_texture() {
	const auto fill_start = std::chrono::steady_clock::now();
	int32_t width = 0;
	int32_t height = 0;
	{
		std::lock_guard<std::mutex> lock(sim_mutex);
		sim.fill_rgba_pixels(rgba_pixels);
		width = sim.get_width();
		height = sim.get_height();
	}
	const auto fill_end = std::chrono::steady_clock::now();
	last_fill_ms = elapsed_ms(fill_start, fill_end);
	if (rgba_pixels.empty()) {
		last_texture_ms = 0.0;
		return;
	}

	const auto texture_start = std::chrono::steady_clock::now();
	pixels.resize(static_cast<int64_t>(rgba_pixels.size()));
	std::memcpy(pixels.ptrw(), rgba_pixels.data(), rgba_pixels.size());
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

void MacWorld::simulation_thread_loop() {
	auto next_tick = std::chrono::steady_clock::now();
	while (simulation_thread_running.load()) {
		double dt_seconds = 1.0 / 30.0;
		float speed = 1.0f;
		{
			std::lock_guard<std::mutex> lock(sim_mutex);
			dt_seconds = sim.get_dt();
			speed = simulation_speed;
		}
		const double interval = std::max(1.0 / 240.0, dt_seconds / std::max(0.001f, speed));
		next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(interval));
		if (!paused.load()) {
			const auto sim_start = std::chrono::steady_clock::now();
			{
				std::lock_guard<std::mutex> lock(sim_mutex);
				sim.step();
			}
			const auto sim_end = std::chrono::steady_clock::now();
			const double sim_elapsed = elapsed_ms(sim_start, sim_end);
			last_sim_ms = sim_elapsed;
			last_frame_sim_steps = 1;
			last_frame_sim_ms = sim_elapsed;
		}
		std::this_thread::sleep_until(next_tick);
		if (std::chrono::steady_clock::now() > next_tick + std::chrono::milliseconds(250)) {
			next_tick = std::chrono::steady_clock::now();
		}
	}
}

void MacWorld::start_simulation_thread() {
	if (simulation_thread_running.load()) {
		return;
	}
	simulation_thread_running.store(true);
	simulation_thread = std::thread(&MacWorld::simulation_thread_loop, this);
}

void MacWorld::stop_simulation_thread() {
	simulation_thread_running.store(false);
	if (simulation_thread.joinable()) {
		simulation_thread.join();
	}
}

void MacWorld::set_world_size(int32_t p_width, int32_t p_height) {
	{
		std::lock_guard<std::mutex> lock(sim_mutex);
		sim.set_world_size(p_width, p_height);
	}
	image.unref();
	texture.unref();
	update_texture();
	queue_redraw();
}
int32_t MacWorld::get_width() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_width(); }
int32_t MacWorld::get_height() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_height(); }
void MacWorld::set_display_scale(double p_scale) { display_scale = clampf(static_cast<float>(p_scale), 0.25f, 64.0f); queue_redraw(); }
double MacWorld::get_display_scale() const { return display_scale; }
void MacWorld::set_paused(bool p_paused) { paused.store(p_paused); }
bool MacWorld::is_paused() const { return paused.load(); }
void MacWorld::set_threaded_simulation_enabled(bool p_enabled) {
	if (p_enabled == threaded_simulation_enabled.load()) {
		return;
	}
	threaded_simulation_enabled.store(p_enabled);
	if (p_enabled) {
		start_simulation_thread();
	} else {
		stop_simulation_thread();
	}
}
bool MacWorld::is_threaded_simulation_enabled() const { return threaded_simulation_enabled.load(); }
void MacWorld::set_simulation_speed(double p_speed) { std::lock_guard<std::mutex> lock(sim_mutex); simulation_speed = clampf(static_cast<float>(p_speed), 0.0f, 20.0f); }
double MacWorld::get_simulation_speed() const { std::lock_guard<std::mutex> lock(sim_mutex); return simulation_speed; }
void MacWorld::set_dt(double p_dt) { std::lock_guard<std::mutex> lock(sim_mutex); sim.set_dt(p_dt); }
double MacWorld::get_dt() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_dt(); }
void MacWorld::set_gravity(double p_gravity) { std::lock_guard<std::mutex> lock(sim_mutex); sim.set_gravity(p_gravity); }
double MacWorld::get_gravity() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_gravity(); }
void MacWorld::set_viscosity(double p_viscosity) { std::lock_guard<std::mutex> lock(sim_mutex); sim.set_viscosity(p_viscosity); }
double MacWorld::get_viscosity() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_viscosity(); }
void MacWorld::set_pressure_iterations(int32_t p_iterations) { std::lock_guard<std::mutex> lock(sim_mutex); sim.set_pressure_iterations(p_iterations); }
int32_t MacWorld::get_pressure_iterations() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_pressure_iterations(); }
void MacWorld::set_pressure_active_mass(double p_mass) { std::lock_guard<std::mutex> lock(sim_mutex); sim.set_pressure_active_mass(p_mass); }
double MacWorld::get_pressure_active_mass() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_pressure_active_mass(); }
void MacWorld::set_density_correction_strength(double p_strength) { std::lock_guard<std::mutex> lock(sim_mutex); sim.set_density_correction_strength(p_strength); }
double MacWorld::get_density_correction_strength() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_density_correction_strength(); }
void MacWorld::set_underfill_correction_strength(double p_strength) { std::lock_guard<std::mutex> lock(sim_mutex); sim.set_underfill_correction_strength(p_strength); }
double MacWorld::get_underfill_correction_strength() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_underfill_correction_strength(); }
void MacWorld::set_rigid_liquid_impulse_strength(double p_strength) { std::lock_guard<std::mutex> lock(sim_mutex); sim.set_rigid_liquid_impulse_strength(p_strength); }
double MacWorld::get_rigid_liquid_impulse_strength() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_rigid_liquid_impulse_strength(); }

void MacWorld::step() {
	run_sim_step();
	last_frame_sim_steps = 1;
	last_frame_sim_ms = last_sim_ms.load();
	update_texture();
	queue_redraw();
}

void MacWorld::clear() {
	{
		std::lock_guard<std::mutex> lock(sim_mutex);
		sim.clear();
	}
	update_texture();
	queue_redraw();
}

void MacWorld::generate_rigid_collision_test() {
	{
		std::lock_guard<std::mutex> lock(sim_mutex);
		sim.generate_rigid_collision_test();
	}
	update_texture();
	queue_redraw();
}

void MacWorld::begin_rigid_paint_stroke() {
	std::lock_guard<std::mutex> lock(sim_mutex);
	sim.begin_rigid_paint_stroke();
}

void MacWorld::end_rigid_paint_stroke() {
	{
		std::lock_guard<std::mutex> lock(sim_mutex);
		sim.end_rigid_paint_stroke();
	}
	update_texture();
	queue_redraw();
}

void MacWorld::generate_basin() {
	{
		std::lock_guard<std::mutex> lock(sim_mutex);
		sim.generate_basin();
	}
	update_texture();
	queue_redraw();
}

void MacWorld::paint_circle(double p_x, double p_y, double p_radius, int32_t p_material) {
	{
		std::lock_guard<std::mutex> lock(sim_mutex);
		sim.paint_circle(p_x, p_y, p_radius, p_material);
	}
	update_texture();
	queue_redraw();
}

void MacWorld::inject_water(double p_x, double p_y, double p_radius, double p_mass_per_cell, double p_velocity_x, double p_velocity_y) {
	{
		std::lock_guard<std::mutex> lock(sim_mutex);
		sim.inject_water(p_x, p_y, p_radius, p_mass_per_cell, p_velocity_x, p_velocity_y);
	}
	update_texture();
	queue_redraw();
}

bool MacWorld::start_rigid_drag(double p_x, double p_y) {
	std::lock_guard<std::mutex> lock(sim_mutex);
	return sim.start_rigid_drag(p_x, p_y);
}

void MacWorld::update_rigid_drag(double p_x, double p_y, bool p_rotate) {
	{
		std::lock_guard<std::mutex> lock(sim_mutex);
		sim.update_rigid_drag(p_x, p_y, p_rotate);
	}
	update_texture();
	queue_redraw();
}

void MacWorld::end_rigid_drag() {
	std::lock_guard<std::mutex> lock(sim_mutex);
	sim.end_rigid_drag();
}

int32_t MacWorld::get_rigid_body_count() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_rigid_body_count(); }
int32_t MacWorld::get_rigid_awake_count() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_rigid_awake_count(); }
int32_t MacWorld::get_rigid_sleeping_count() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_rigid_sleeping_count(); }

double MacWorld::get_total_water_mass() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_total_water_mass(); }
int64_t MacWorld::get_water_cell_count() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_water_cell_count(); }
double MacWorld::get_average_water_mass() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_average_water_mass(); }
double MacWorld::get_last_step_ms() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_last_step_ms(); }
double MacWorld::get_last_predict_ms() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_last_predict_ms(); }
double MacWorld::get_last_build_ms() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_last_build_ms(); }
double MacWorld::get_last_pcg_ms() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_last_pcg_ms(); }
double MacWorld::get_last_project_ms() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_last_project_ms(); }
double MacWorld::get_last_advect_ms() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_last_advect_ms(); }
double MacWorld::get_last_clamp_ms() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_last_clamp_ms(); }
int32_t MacWorld::get_active_region_min_x() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_active_region_min_x(); }
int32_t MacWorld::get_active_region_min_y() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_active_region_min_y(); }
int32_t MacWorld::get_active_region_max_x() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_active_region_max_x(); }
int32_t MacWorld::get_active_region_max_y() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_active_region_max_y(); }
int32_t MacWorld::get_active_region_pad() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_active_region_pad(); }
double MacWorld::get_active_region_max_speed() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_active_region_max_speed(); }
double MacWorld::get_last_sim_ms() const { return last_sim_ms; }
double MacWorld::get_last_fill_ms() const { return last_fill_ms; }
double MacWorld::get_last_texture_ms() const { return last_texture_ms; }
double MacWorld::get_last_frame_sim_ms() const { return last_frame_sim_ms; }
int32_t MacWorld::get_last_frame_sim_steps() const { return last_frame_sim_steps; }
int64_t MacWorld::get_step_count() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_step_count(); }
int32_t MacWorld::get_last_pcg_iterations() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_last_pcg_iterations(); }
double MacWorld::get_last_pcg_residual() const { std::lock_guard<std::mutex> lock(sim_mutex); return sim.get_last_pcg_residual(); }


