#include "noita_world.h"

#include <godot_cpp/classes/file_access.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/rect2.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/variant/vector2.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace {
constexpr float MASS_EPSILON = 0.0005f;
constexpr uint32_t SAVE_MAGIC = 0x4E574C44; // NWLD
constexpr uint32_t SAVE_VERSION = 1;

float clampf(float v, float lo, float hi) {
	return std::max(lo, std::min(v, hi));
}

int32_t round_to_cell(float v) {
	return static_cast<int32_t>(std::floor(v + 0.5f));
}
} // namespace

NoitaWorld::NoitaWorld() {
}

void NoitaWorld::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_world_size", "width", "height"), &NoitaWorld::set_world_size);
	ClassDB::bind_method(D_METHOD("get_width"), &NoitaWorld::get_width);
	ClassDB::bind_method(D_METHOD("get_height"), &NoitaWorld::get_height);

	ClassDB::bind_method(D_METHOD("set_display_scale", "scale"), &NoitaWorld::set_display_scale);
	ClassDB::bind_method(D_METHOD("get_display_scale"), &NoitaWorld::get_display_scale);
	ClassDB::bind_method(D_METHOD("set_paused", "paused"), &NoitaWorld::set_paused);
	ClassDB::bind_method(D_METHOD("is_paused"), &NoitaWorld::is_paused);
	ClassDB::bind_method(D_METHOD("set_simulation_speed", "speed"), &NoitaWorld::set_simulation_speed);
	ClassDB::bind_method(D_METHOD("get_simulation_speed"), &NoitaWorld::get_simulation_speed);
	ClassDB::bind_method(D_METHOD("set_pressure_iterations", "iterations"), &NoitaWorld::set_pressure_iterations);
	ClassDB::bind_method(D_METHOD("get_pressure_iterations"), &NoitaWorld::get_pressure_iterations);
	ClassDB::bind_method(D_METHOD("set_pressure_release_radius", "radius"), &NoitaWorld::set_pressure_release_radius);
	ClassDB::bind_method(D_METHOD("get_pressure_release_radius"), &NoitaWorld::get_pressure_release_radius);
	ClassDB::bind_method(D_METHOD("set_pressure_distance_decay", "decay"), &NoitaWorld::set_pressure_distance_decay);
	ClassDB::bind_method(D_METHOD("get_pressure_distance_decay"), &NoitaWorld::get_pressure_distance_decay);
	ClassDB::bind_method(D_METHOD("set_pressure_density_speed", "speed"), &NoitaWorld::set_pressure_density_speed);
	ClassDB::bind_method(D_METHOD("get_pressure_density_speed"), &NoitaWorld::get_pressure_density_speed);
	ClassDB::bind_method(D_METHOD("set_pressure_target_max_mass", "mass"), &NoitaWorld::set_pressure_target_max_mass);
	ClassDB::bind_method(D_METHOD("get_pressure_target_max_mass"), &NoitaWorld::get_pressure_target_max_mass);
	ClassDB::bind_method(D_METHOD("set_pressure_impulse_strength", "strength"), &NoitaWorld::set_pressure_impulse_strength);
	ClassDB::bind_method(D_METHOD("get_pressure_impulse_strength"), &NoitaWorld::get_pressure_impulse_strength);
	ClassDB::bind_method(D_METHOD("set_gravity", "gravity"), &NoitaWorld::set_gravity);
	ClassDB::bind_method(D_METHOD("get_gravity"), &NoitaWorld::get_gravity);

	ClassDB::bind_method(D_METHOD("step"), &NoitaWorld::step);
	ClassDB::bind_method(D_METHOD("clear"), &NoitaWorld::clear);
	ClassDB::bind_method(D_METHOD("generate_basin"), &NoitaWorld::generate_basin);
	ClassDB::bind_method(D_METHOD("paint_circle", "x", "y", "radius", "material"), &NoitaWorld::paint_circle);
	ClassDB::bind_method(D_METHOD("inject_water", "x", "y", "radius", "mass_per_cell", "velocity_x", "velocity_y"), &NoitaWorld::inject_water);
	ClassDB::bind_method(D_METHOD("load_from_png", "path"), &NoitaWorld::load_from_png);
	ClassDB::bind_method(D_METHOD("save_to_file", "path"), &NoitaWorld::save_to_file);
	ClassDB::bind_method(D_METHOD("load_from_file", "path"), &NoitaWorld::load_from_file);
	ClassDB::bind_method(D_METHOD("get_total_water_mass"), &NoitaWorld::get_total_water_mass);
	ClassDB::bind_method(D_METHOD("get_water_cell_count"), &NoitaWorld::get_water_cell_count);
	ClassDB::bind_method(D_METHOD("get_average_water_mass"), &NoitaWorld::get_average_water_mass);
	ClassDB::bind_method(D_METHOD("get_last_step_ms"), &NoitaWorld::get_last_step_ms);
	ClassDB::bind_method(D_METHOD("get_step_count"), &NoitaWorld::get_step_count);

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "display_scale"), "set_display_scale", "get_display_scale");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "paused"), "set_paused", "is_paused");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "simulation_speed"), "set_simulation_speed", "get_simulation_speed");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "pressure_iterations"), "set_pressure_iterations", "get_pressure_iterations");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "pressure_release_radius"), "set_pressure_release_radius", "get_pressure_release_radius");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "pressure_distance_decay"), "set_pressure_distance_decay", "get_pressure_distance_decay");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "pressure_density_speed"), "set_pressure_density_speed", "get_pressure_density_speed");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "pressure_target_max_mass"), "set_pressure_target_max_mass", "get_pressure_target_max_mass");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "pressure_impulse_strength"), "set_pressure_impulse_strength", "get_pressure_impulse_strength");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "gravity"), "set_gravity", "get_gravity");
}

void NoitaWorld::_ready() {
	if (cells.empty()) {
		set_world_size(width, height);
		generate_basin();
	} else {
		update_texture();
	}
	queue_redraw();
}

void NoitaWorld::_process(double p_delta) {
	if (paused) {
		return;
	}
	step_accumulator += static_cast<float>(p_delta * 60.0 * simulation_speed);
	int32_t steps_this_frame = 0;
	while (step_accumulator >= 1.0f && steps_this_frame < 8) {
		step();
		step_accumulator -= 1.0f;
		steps_this_frame++;
	}
	if (steps_this_frame == 8) {
		step_accumulator = 0.0f;
	}
}

void NoitaWorld::_draw() {
	if (texture.is_valid()) {
		draw_texture_rect(texture, Rect2(0, 0, width * display_scale, height * display_scale), false);
	}
}

int32_t NoitaWorld::index(int32_t p_x, int32_t p_y) const {
	return p_y * width + p_x;
}

bool NoitaWorld::in_bounds(int32_t p_x, int32_t p_y) const {
	return p_x >= 0 && p_y >= 0 && p_x < width && p_y < height;
}

bool NoitaWorld::is_rock(int32_t p_x, int32_t p_y) const {
	if (!in_bounds(p_x, p_y)) {
		return true;
	}
	return cells[index(p_x, p_y)].material == MATERIAL_ROCK;
}

bool NoitaWorld::is_blocked_for_motion(int32_t p_x, int32_t p_y) const {
	return is_rock(p_x, p_y);
}

void NoitaWorld::ensure_buffers() {
	const int32_t count = width * height;
	cells.resize(count);
	next_cells.resize(count);
	delta_mass.resize(count);
	delta_momentum_x.resize(count);
	delta_momentum_y.resize(count);
}

void NoitaWorld::clear_dynamic_buffers() {
	const int32_t count = width * height;
	for (int32_t i = 0; i < count; i++) {
		next_cells[i].material = cells[i].material;
		next_cells[i].mass = 0.0f;
		next_cells[i].momentum_x = 0.0f;
		next_cells[i].momentum_y = 0.0f;
		if (next_cells[i].material == MATERIAL_ROCK) {
			next_cells[i].mass = 0.0f;
		}
	}
}

void NoitaWorld::add_water_to_cell(int32_t p_idx, float p_mass, float p_momentum_x, float p_momentum_y) {
	if (p_idx < 0 || p_idx >= static_cast<int32_t>(next_cells.size())) {
		return;
	}
	Cell &dst = next_cells[p_idx];
	if (dst.material == MATERIAL_ROCK) {
		return;
	}
	dst.material = MATERIAL_AIR;
	dst.mass += p_mass;
	dst.momentum_x += p_momentum_x;
	dst.momentum_y += p_momentum_y;
}

void NoitaWorld::trace_motion(int32_t p_x, int32_t p_y, float p_vx, float p_vy, int32_t &r_x, int32_t &r_y, float &r_vx, float &r_vy) const {
	r_x = p_x;
	r_y = p_y;
	r_vx = p_vx;
	r_vy = p_vy;

	float max_component = std::max(std::abs(p_vx), std::abs(p_vy));
	int32_t substeps = std::max(1, static_cast<int32_t>(std::ceil(max_component)));
	float step_x = p_vx / static_cast<float>(substeps);
	float step_y = p_vy / static_cast<float>(substeps);
	float pos_x = static_cast<float>(p_x);
	float pos_y = static_cast<float>(p_y);

	for (int32_t s = 0; s < substeps; s++) {
		float wanted_x = pos_x + step_x;
		float wanted_y = pos_y + step_y;
		int32_t cell_x = round_to_cell(wanted_x);
		int32_t cell_y = round_to_cell(wanted_y);

		if (is_blocked_for_motion(cell_x, cell_y)) {
			bool x_blocked = is_blocked_for_motion(round_to_cell(pos_x + step_x), round_to_cell(pos_y));
			bool y_blocked = is_blocked_for_motion(round_to_cell(pos_x), round_to_cell(pos_y + step_y));

			if (x_blocked) {
				step_x = 0.0f;
				r_vx = 0.0f;
			}
			if (y_blocked) {
				step_y = 0.0f;
				r_vy = 0.0f;
			}

			wanted_x = pos_x + step_x;
			wanted_y = pos_y + step_y;
			cell_x = round_to_cell(wanted_x);
			cell_y = round_to_cell(wanted_y);
			if (is_blocked_for_motion(cell_x, cell_y)) {
				break;
			}
		}

		pos_x = wanted_x;
		pos_y = wanted_y;
		r_x = clampf(static_cast<float>(round_to_cell(pos_x)), 0.0f, static_cast<float>(width - 1));
		r_y = clampf(static_cast<float>(round_to_cell(pos_y)), 0.0f, static_cast<float>(height - 1));
	}
}

void NoitaWorld::advect_with_overlap() {
	clear_dynamic_buffers();
	const int32_t count = width * height;
	for (int32_t i = 0; i < count; i++) {
		const Cell &src = cells[i];
		if (src.material == MATERIAL_ROCK || src.mass <= MASS_EPSILON) {
			continue;
		}

		int32_t x = i % width;
		int32_t y = i / width;
		float mass = src.mass;
		float mx = src.momentum_x;
		float my = src.momentum_y + mass * gravity;
		float vx = mx / mass;
		float vy = my / mass;

		float speed = std::sqrt(vx * vx + vy * vy);
		if (speed > max_velocity) {
			float scale = max_velocity / speed;
			vx *= scale;
			vy *= scale;
			mx = vx * mass;
			my = vy * mass;
		}

		int32_t tx = x;
		int32_t ty = y;
		float out_vx = vx;
		float out_vy = vy;
		trace_motion(x, y, vx, vy, tx, ty, out_vx, out_vy);

		int32_t target = index(tx, ty);
		add_water_to_cell(target, mass, out_vx * mass, out_vy * mass);
	}
	cells.swap(next_cells);
}

void NoitaWorld::pressure_relax_once() {
	const int32_t count = width * height;
	std::fill(delta_mass.begin(), delta_mass.end(), 0.0f);
	std::fill(delta_momentum_x.begin(), delta_momentum_x.end(), 0.0f);
	std::fill(delta_momentum_y.begin(), delta_momentum_y.end(), 0.0f);

	struct PressureTarget {
		int32_t target_index = 0;
		float weight = 0.0f;
		float dir_x = 0.0f;
		float dir_y = 0.0f;
	};
	std::vector<PressureTarget> targets;

	for (int32_t y = 0; y < height; y++) {
		for (int32_t x = 0; x < width; x++) {
			int32_t i = index(x, y);
			const Cell &src = cells[i];
			if (src.material == MATERIAL_ROCK || src.mass <= max_stable_mass + MASS_EPSILON) {
				continue;
			}

			float pressure = src.mass - max_stable_mass;
			int32_t radius = std::max(1, pressure_release_radius);
			targets.clear();
			targets.reserve(static_cast<size_t>((radius * 2 + 1) * (radius * 2 + 1) - 1));
			float total_weight = 0.0f;

			// Local pressure release only:
			// scan the finite neighborhood and directly distribute excess mass to
			// lower-pressure cells inside this radius.  This intentionally does NOT
			// relay/fill through already-full cells in up/down/left/right directions.
			for (int32_t oy = -radius; oy <= radius; oy++) {
				for (int32_t ox = -radius; ox <= radius; ox++) {
					if (ox == 0 && oy == 0) {
						continue;
					}
					int32_t nx = x + ox;
					int32_t ny = y + oy;
					if (!in_bounds(nx, ny)) {
						continue;
					}
					const Cell &dst = cells[index(nx, ny)];
					if (dst.material == MATERIAL_ROCK) {
						continue;
					}

					float neighbor_pressure = std::max(0.0f, dst.mass - max_stable_mass);
					float diff = pressure - neighbor_pressure;
					if (diff <= MASS_EPSILON) {
						continue;
					}

					float distance = std::sqrt(static_cast<float>(ox * ox + oy * oy));
					float weight = diff / (1.0f + distance * pressure_distance_decay);
					if (weight <= MASS_EPSILON) {
						continue;
					}

					PressureTarget target;
					target.target_index = index(nx, ny);
					target.weight = weight;
					if (distance > MASS_EPSILON) {
						target.dir_x = static_cast<float>(ox) / distance;
						target.dir_y = static_cast<float>(oy) / distance;
					}
					targets.push_back(target);
					total_weight += weight;
				}
			}

			float excess = std::max(0.0f, src.mass - max_stable_mass);
			float releasable = std::min(excess * pressure_release_fraction, src.mass);
			if (targets.empty() || total_weight <= MASS_EPSILON || releasable <= MASS_EPSILON) {
				continue;
			}

			for (const PressureTarget &target : targets) {
				float flow = releasable * (target.weight / total_weight);
				if (flow <= MASS_EPSILON) {
					continue;
				}

				float ratio = flow / std::max(src.mass, MASS_EPSILON);
				float flow_mx = src.momentum_x * ratio;
				float flow_my = src.momentum_y * ratio;
				float impulse_x = target.dir_x * flow * pressure_impulse_strength;
				float impulse_y = target.dir_y * flow * pressure_impulse_strength;

				delta_mass[i] -= flow;
				delta_mass[target.target_index] += flow;
				delta_momentum_x[i] -= flow_mx;
				delta_momentum_y[i] -= flow_my;
				delta_momentum_x[target.target_index] += flow_mx + impulse_x;
				delta_momentum_y[target.target_index] += flow_my + impulse_y;
			}
		}
	}

	for (int32_t i = 0; i < count; i++) {
		Cell &c = cells[i];
		if (c.material == MATERIAL_ROCK) {
			c.mass = 0.0f;
			c.momentum_x = 0.0f;
			c.momentum_y = 0.0f;
			continue;
		}
		c.mass += delta_mass[i];
		c.momentum_x += delta_momentum_x[i];
		c.momentum_y += delta_momentum_y[i];
		if (c.mass <= MASS_EPSILON) {
			c.mass = 0.0f;
			c.momentum_x = 0.0f;
			c.momentum_y = 0.0f;
		} else if (c.mass < 0.0f) {
			c.mass = 0.0f;
			c.momentum_x = 0.0f;
			c.momentum_y = 0.0f;
		}
	}
}

void NoitaWorld::damp_and_sanitize() {
	for (Cell &c : cells) {
		if (c.material == MATERIAL_ROCK) {
			c.mass = 0.0f;
			c.momentum_x = 0.0f;
			c.momentum_y = 0.0f;
			continue;
		}
		if (c.mass <= MASS_EPSILON) {
			c.mass = 0.0f;
			c.momentum_x = 0.0f;
			c.momentum_y = 0.0f;
			continue;
		}
		float vx = c.momentum_x / c.mass;
		float vy = c.momentum_y / c.mass;
		float speed = std::sqrt(vx * vx + vy * vy);
		if (speed > max_velocity) {
			float scale = max_velocity / speed;
			vx *= scale;
			vy *= scale;
		}
		c.momentum_x = vx * c.mass * velocity_damping;
		c.momentum_y = vy * c.mass * velocity_damping;
	}
}

void NoitaWorld::step() {
	if (cells.empty()) {
		set_world_size(width, height);
	}

	auto t0 = std::chrono::high_resolution_clock::now();
	advect_with_overlap();
	for (int32_t i = 0; i < pressure_iterations; i++) {
		pressure_relax_once();
	}
	damp_and_sanitize();
	update_texture();
	queue_redraw();
	step_count++;
	auto t1 = std::chrono::high_resolution_clock::now();
	last_step_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
}

void NoitaWorld::set_world_size(int32_t p_width, int32_t p_height) {
	width = std::max(16, p_width);
	height = std::max(16, p_height);
	cells.assign(width * height, Cell());
	next_cells.assign(width * height, Cell());
	delta_mass.assign(width * height, 0.0f);
	delta_momentum_x.assign(width * height, 0.0f);
	delta_momentum_y.assign(width * height, 0.0f);
	pixels.resize(width * height * 4);
	image.unref();
	texture.unref();
	step_count = 0;
	update_texture();
	queue_redraw();
}

int32_t NoitaWorld::get_width() const { return width; }
int32_t NoitaWorld::get_height() const { return height; }

void NoitaWorld::set_display_scale(double p_scale) {
	display_scale = clampf(static_cast<float>(p_scale), 0.25f, 32.0f);
	queue_redraw();
}

double NoitaWorld::get_display_scale() const { return display_scale; }

void NoitaWorld::set_paused(bool p_paused) { paused = p_paused; }
bool NoitaWorld::is_paused() const { return paused; }

void NoitaWorld::set_simulation_speed(double p_speed) {
	simulation_speed = clampf(static_cast<float>(p_speed), 0.0f, 20.0f);
}

double NoitaWorld::get_simulation_speed() const { return simulation_speed; }

void NoitaWorld::set_pressure_iterations(int32_t p_iterations) {
	pressure_iterations = std::max(0, std::min(p_iterations, 64));
}

int32_t NoitaWorld::get_pressure_iterations() const { return pressure_iterations; }

void NoitaWorld::set_pressure_release_radius(int32_t p_radius) {
	pressure_release_radius = std::max(1, std::min(p_radius, 32));
}

int32_t NoitaWorld::get_pressure_release_radius() const { return pressure_release_radius; }

void NoitaWorld::set_pressure_distance_decay(double p_decay) {
	pressure_distance_decay = clampf(static_cast<float>(p_decay), 0.0f, 10.0f);
}

double NoitaWorld::get_pressure_distance_decay() const { return pressure_distance_decay; }

void NoitaWorld::set_pressure_density_speed(double p_speed) {
	pressure_density_speed = clampf(static_cast<float>(p_speed), 0.0f, 10.0f);
}

double NoitaWorld::get_pressure_density_speed() const { return pressure_density_speed; }

void NoitaWorld::set_pressure_target_max_mass(double p_mass) {
	pressure_target_max_mass = clampf(static_cast<float>(p_mass), max_stable_mass + 0.001f, 10.0f);
}

double NoitaWorld::get_pressure_target_max_mass() const { return pressure_target_max_mass; }

void NoitaWorld::set_pressure_impulse_strength(double p_strength) {
	pressure_impulse_strength = clampf(static_cast<float>(p_strength), 0.0f, 20.0f);
}

double NoitaWorld::get_pressure_impulse_strength() const { return pressure_impulse_strength; }

void NoitaWorld::set_gravity(double p_gravity) {
	gravity = clampf(static_cast<float>(p_gravity), -5.0f, 5.0f);
}

double NoitaWorld::get_gravity() const { return gravity; }

void NoitaWorld::make_rock(int32_t p_x, int32_t p_y) {
	if (!in_bounds(p_x, p_y)) {
		return;
	}
	Cell &c = cells[index(p_x, p_y)];
	c.material = MATERIAL_ROCK;
	c.mass = 0.0f;
	c.momentum_x = 0.0f;
	c.momentum_y = 0.0f;
}

void NoitaWorld::make_air(int32_t p_x, int32_t p_y) {
	if (!in_bounds(p_x, p_y)) {
		return;
	}
	Cell &c = cells[index(p_x, p_y)];
	c.material = MATERIAL_AIR;
	c.mass = 0.0f;
	c.momentum_x = 0.0f;
	c.momentum_y = 0.0f;
}

void NoitaWorld::make_water(int32_t p_x, int32_t p_y, float p_mass) {
	if (!in_bounds(p_x, p_y)) {
		return;
	}
	Cell &c = cells[index(p_x, p_y)];
	if (c.material == MATERIAL_ROCK) {
		return;
	}
	c.material = MATERIAL_AIR;
	c.mass += p_mass;
}

void NoitaWorld::clear() {
	for (Cell &c : cells) {
		c.material = MATERIAL_AIR;
		c.mass = 0.0f;
		c.momentum_x = 0.0f;
		c.momentum_y = 0.0f;
	}
	update_texture();
	queue_redraw();
}

void NoitaWorld::generate_basin() {
	clear();
	// Outer walls and floor.
	for (int32_t x = 0; x < width; x++) {
		for (int32_t y = height - 8; y < height; y++) {
			make_rock(x, y);
		}
	}
	for (int32_t y = 0; y < height; y++) {
		for (int32_t t = 0; t < 5; t++) {
			make_rock(t, y);
			make_rock(width - 1 - t, y);
		}
	}

	// Irregular cup/basin. The sides are thick rock slopes; the bottom is bumpy.
	int32_t cup_top = height / 3;
	int32_t cup_bottom = height - 10;
	for (int32_t y = cup_top; y < cup_bottom; y++) {
		float t = static_cast<float>(y - cup_top) / static_cast<float>(std::max(1, cup_bottom - cup_top));
		int32_t left_wall = static_cast<int32_t>(width * 0.17f + t * width * 0.14f + 4.0f * std::sin(y * 0.11f));
		int32_t right_wall = static_cast<int32_t>(width * 0.83f - t * width * 0.14f + 5.0f * std::sin(y * 0.09f + 2.0f));
		for (int32_t k = -3; k <= 3; k++) {
			make_rock(left_wall + k, y);
			make_rock(right_wall + k, y);
		}
	}
	for (int32_t x = 0; x < width; x++) {
		float basin_curve = std::abs((static_cast<float>(x) / width) - 0.5f) * 2.0f;
		int32_t bed_y = static_cast<int32_t>(height * 0.74f + basin_curve * basin_curve * height * 0.14f + 5.0f * std::sin(x * 0.08f) + 3.0f * std::sin(x * 0.21f));
		for (int32_t y = bed_y; y < height; y++) {
			make_rock(x, y);
		}
	}

	// A few inner rock bumps to make the flow visibly split.
	int32_t mound_cx = width / 2;
	int32_t mound_cy = static_cast<int32_t>(height * 0.68f);
	for (int32_t y = mound_cy - 8; y <= mound_cy + 6; y++) {
		for (int32_t x = mound_cx - 18; x <= mound_cx + 18; x++) {
			float nx = (x - mound_cx) / 18.0f;
			float ny = (y - mound_cy) / 9.0f;
			if (nx * nx + ny * ny < 1.0f) {
				make_rock(x, y);
			}
		}
	}

	// Initial falling water blob.
	int32_t blob_x0 = width / 2 - width / 9;
	int32_t blob_x1 = width / 2 + width / 9;
	int32_t blob_y0 = 14;
	int32_t blob_y1 = height / 3 - 16;
	for (int32_t y = blob_y0; y < blob_y1; y++) {
		for (int32_t x = blob_x0; x < blob_x1; x++) {
			float nx = (x - width / 2.0f) / (width / 9.5f);
			float ny = (y - (blob_y0 + blob_y1) * 0.5f) / ((blob_y1 - blob_y0) * 0.55f);
			if (nx * nx + ny * ny <= 1.0f) {
				make_water(x, y, 1.0f);
			}
		}
	}
	update_texture();
	queue_redraw();
}

void NoitaWorld::paint_circle(double p_x, double p_y, double p_radius, int32_t p_material) {
	int32_t cx = static_cast<int32_t>(std::floor(p_x));
	int32_t cy = static_cast<int32_t>(std::floor(p_y));
	int32_t r = std::max(1, static_cast<int32_t>(std::ceil(p_radius)));
	int32_t r2 = r * r;
	for (int32_t y = cy - r; y <= cy + r; y++) {
		for (int32_t x = cx - r; x <= cx + r; x++) {
			int32_t dx = x - cx;
			int32_t dy = y - cy;
			if (dx * dx + dy * dy > r2 || !in_bounds(x, y)) {
				continue;
			}
			if (p_material == MATERIAL_ROCK) {
				make_rock(x, y);
			} else if (p_material == MATERIAL_WATER) {
				make_water(x, y, 1.0f);
			} else {
				make_air(x, y);
			}
		}
	}
	update_texture();
	queue_redraw();
}

void NoitaWorld::inject_water(double p_x, double p_y, double p_radius, double p_mass_per_cell, double p_velocity_x, double p_velocity_y) {
	int32_t cx = static_cast<int32_t>(std::floor(p_x));
	int32_t cy = static_cast<int32_t>(std::floor(p_y));
	int32_t r = std::max(1, static_cast<int32_t>(std::ceil(p_radius)));
	int32_t r2 = r * r;
	float mass_per_cell = clampf(static_cast<float>(p_mass_per_cell), 0.0f, 10.0f);
	float vx = clampf(static_cast<float>(p_velocity_x), -max_velocity, max_velocity);
	float vy = clampf(static_cast<float>(p_velocity_y), -max_velocity, max_velocity);
	if (mass_per_cell <= MASS_EPSILON) {
		return;
	}

	for (int32_t y = cy - r; y <= cy + r; y++) {
		for (int32_t x = cx - r; x <= cx + r; x++) {
			int32_t dx = x - cx;
			int32_t dy = y - cy;
			if (dx * dx + dy * dy > r2 || !in_bounds(x, y)) {
				continue;
			}

			Cell &c = cells[index(x, y)];
			if (c.material == MATERIAL_ROCK) {
				continue;
			}

			c.material = MATERIAL_AIR;
			c.mass += mass_per_cell;
			c.momentum_x += mass_per_cell * vx;
			c.momentum_y += mass_per_cell * vy;
		}
	}
	update_texture();
	queue_redraw();
}

bool NoitaWorld::load_from_png(const String &p_path) {
	Ref<Image> source = Image::load_from_file(p_path);
	if (!source.is_valid() || source->is_empty()) {
		return false;
	}
	set_world_size(source->get_width(), source->get_height());
	for (int32_t y = 0; y < height; y++) {
		for (int32_t x = 0; x < width; x++) {
			Color c = source->get_pixel(x, y);
			float brightness = (c.r + c.g + c.b) / 3.0f;
			if (c.a < 0.1f || brightness < 0.08f) {
				make_air(x, y);
			} else if (c.b > c.r * 1.25f && c.b > c.g * 1.05f) {
				make_water(x, y, clampf(c.a, 0.2f, 1.0f));
			} else {
				make_rock(x, y);
			}
		}
	}
	update_texture();
	queue_redraw();
	return true;
}

bool NoitaWorld::save_to_file(const String &p_path) const {
	Ref<FileAccess> file = FileAccess::open(p_path, FileAccess::WRITE);
	if (!file.is_valid() || !file->is_open()) {
		return false;
	}
	file->store_32(SAVE_MAGIC);
	file->store_32(SAVE_VERSION);
	file->store_32(static_cast<uint32_t>(width));
	file->store_32(static_cast<uint32_t>(height));
	for (const Cell &c : cells) {
		file->store_8(c.material);
		file->store_float(c.mass);
		file->store_float(c.momentum_x);
		file->store_float(c.momentum_y);
	}
	file->close();
	return true;
}

bool NoitaWorld::load_from_file(const String &p_path) {
	Ref<FileAccess> file = FileAccess::open(p_path, FileAccess::READ);
	if (!file.is_valid() || !file->is_open()) {
		return false;
	}
	uint32_t magic = file->get_32();
	uint32_t version = file->get_32();
	if (magic != SAVE_MAGIC || version != SAVE_VERSION) {
		return false;
	}
	int32_t new_width = static_cast<int32_t>(file->get_32());
	int32_t new_height = static_cast<int32_t>(file->get_32());
	set_world_size(new_width, new_height);
	for (Cell &c : cells) {
		c.material = static_cast<uint8_t>(file->get_8());
		c.mass = file->get_float();
		c.momentum_x = file->get_float();
		c.momentum_y = file->get_float();
		if (c.material == MATERIAL_ROCK) {
			c.mass = 0.0f;
			c.momentum_x = 0.0f;
			c.momentum_y = 0.0f;
		}
	}
	file->close();
	update_texture();
	queue_redraw();
	return true;
}

void NoitaWorld::update_texture() {
	if (width <= 0 || height <= 0) {
		return;
	}
	pixels.resize(width * height * 4);
	uint8_t *dst = pixels.ptrw();
	for (int32_t y = 0; y < height; y++) {
		for (int32_t x = 0; x < width; x++) {
			const Cell &c = cells[index(x, y)];
			int32_t p = (y * width + x) * 4;
			uint8_t r = 6;
			uint8_t g = 7;
			uint8_t b = 10;
			if (c.material == MATERIAL_ROCK) {
				uint8_t shade = static_cast<uint8_t>(72 + ((x * 13 + y * 7) & 31));
				r = shade;
				g = shade;
				b = static_cast<uint8_t>(shade + 8);
			} else if (c.mass > MASS_EPSILON) {
				float m = clampf(c.mass, 0.0f, 2.5f);
				float pressure = std::max(0.0f, c.mass - max_stable_mass);
				float sat = clampf(m, 0.0f, 1.0f);
				r = static_cast<uint8_t>(18 + 65.0f * clampf(pressure, 0.0f, 1.0f));
				g = static_cast<uint8_t>(70 + 95.0f * sat + 40.0f * clampf(pressure, 0.0f, 1.0f));
				b = static_cast<uint8_t>(150 + 95.0f * sat);
			}
			dst[p + 0] = r;
			dst[p + 1] = g;
			dst[p + 2] = b;
			dst[p + 3] = 255;
		}
	}

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

double NoitaWorld::get_total_water_mass() const {
	double total = 0.0;
	for (const Cell &c : cells) {
		if (c.material != MATERIAL_ROCK) {
			total += c.mass;
		}
	}
	return total;
}

int64_t NoitaWorld::get_water_cell_count() const {
	int64_t count = 0;
	for (const Cell &c : cells) {
		if (c.material != MATERIAL_ROCK && c.mass > MASS_EPSILON) {
			count++;
		}
	}
	return count;
}

double NoitaWorld::get_average_water_mass() const {
	double total = 0.0;
	int64_t count = 0;
	for (const Cell &c : cells) {
		if (c.material != MATERIAL_ROCK && c.mass > MASS_EPSILON) {
			total += c.mass;
			count++;
		}
	}
	if (count == 0) {
		return 0.0;
	}
	return total / static_cast<double>(count);
}

double NoitaWorld::get_last_step_ms() const { return last_step_ms; }

int64_t NoitaWorld::get_step_count() const { return static_cast<int64_t>(step_count); }
