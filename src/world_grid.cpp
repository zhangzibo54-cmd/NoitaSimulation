#include "world_grid.h"

#include <algorithm>
#include <cmath>

using namespace noita;

void WorldGrid::set_size(int32_t p_width, int32_t p_height) {
	width = std::max(16, p_width);
	height = std::max(16, p_height);
	const int32_t count = cell_count();
	material.assign(count, MATERIAL_AIR);
	volume_fraction.assign(count, 0.0f);
	density.assign(count, get_material_def(MATERIAL_AIR).density);
	toxic.assign(count, 0.0f);
	oil.assign(count, 0.0f);
	pressure.assign(count, 0.0f);
	velocity_x.assign(count, 0.0f);
	velocity_y.assign(count, 0.0f);
	temperature.assign(count, 0.0f);
	lifetime.assign(count, 0.0f);
	rigid_body_id.assign(count, 0);
	rigid_velocity_x.assign(count, 0.0f);
	rigid_velocity_y.assign(count, 0.0f);
	active_liquid_cells.clear();
	active_powder_cells.clear();
	active_gas_cells.clear();
	active_fire_cells.clear();
	active_reaction_cells.clear();
	active_liquid_cells.reserve(count);
	active_powder_cells.reserve(count);
	active_gas_cells.reserve(count);
	active_fire_cells.reserve(count / 16);
	active_reaction_cells.reserve(count / 4);
}

void WorldGrid::clear() {
	std::fill(material.begin(), material.end(), MATERIAL_AIR);
	clear_dynamic_fields();
	active_liquid_cells.clear();
	active_powder_cells.clear();
	active_gas_cells.clear();
	active_fire_cells.clear();
	active_reaction_cells.clear();
}

void WorldGrid::clear_dynamic_fields() {
	std::fill(volume_fraction.begin(), volume_fraction.end(), 0.0f);
	std::fill(density.begin(), density.end(), get_material_def(MATERIAL_AIR).density);
	std::fill(toxic.begin(), toxic.end(), 0.0f);
	std::fill(oil.begin(), oil.end(), 0.0f);
	std::fill(pressure.begin(), pressure.end(), 0.0f);
	std::fill(velocity_x.begin(), velocity_x.end(), 0.0f);
	std::fill(velocity_y.begin(), velocity_y.end(), 0.0f);
	std::fill(temperature.begin(), temperature.end(), 0.0f);
	std::fill(lifetime.begin(), lifetime.end(), 0.0f);
	clear_rigid_fields();
}

void WorldGrid::clear_rigid_fields() {
	std::fill(rigid_body_id.begin(), rigid_body_id.end(), 0);
	std::fill(rigid_velocity_x.begin(), rigid_velocity_x.end(), 0.0f);
	std::fill(rigid_velocity_y.begin(), rigid_velocity_y.end(), 0.0f);
}

int32_t WorldGrid::cell_count() const { return width * height; }
int32_t WorldGrid::cell_index(int32_t p_x, int32_t p_y) const { return p_y * width + p_x; }

bool WorldGrid::in_bounds(int32_t p_x, int32_t p_y) const {
	return p_x >= 0 && p_y >= 0 && p_x < width && p_y < height;
}

const MaterialDef &WorldGrid::cell_material_def(int32_t p_x, int32_t p_y) const {
	if (!in_bounds(p_x, p_y)) {
		return solid_boundary_material_def();
	}
	return get_material_def(material[cell_index(p_x, p_y)]);
}

bool WorldGrid::is_solid_cell(int32_t p_x, int32_t p_y) const {
	return cell_material_def(p_x, p_y).solid;
}

bool WorldGrid::has_dynamic_rigid_cell(int32_t p_x, int32_t p_y) const {
	if (!in_bounds(p_x, p_y)) {
		return false;
	}
	const int32_t i = cell_index(p_x, p_y);
	return i >= 0 && i < static_cast<int32_t>(rigid_body_id.size()) && rigid_body_id[i] != 0;
}

bool WorldGrid::blocks_velocity(int32_t p_x, int32_t p_y) const {
	return has_dynamic_rigid_cell(p_x, p_y) || cell_material_def(p_x, p_y).blocks_velocity;
}

bool WorldGrid::is_liquid_cell(int32_t p_x, int32_t p_y, float p_mass_epsilon) const {
	if (!in_bounds(p_x, p_y)) {
		return false;
	}
	const int32_t i = cell_index(p_x, p_y);
	if (rigid_body_id[i] != 0) {
		return false;
	}
	const MaterialDef &def = get_material_def(material[i]);
	return def.liquid && std::isfinite(volume_fraction[i]) && volume_fraction[i] > p_mass_epsilon;
}

bool WorldGrid::is_powder_cell(int32_t p_x, int32_t p_y) const {
	if (!in_bounds(p_x, p_y)) {
		return false;
	}
	return get_material_def(material[cell_index(p_x, p_y)]).powder;
}

bool WorldGrid::is_gas_cell(int32_t p_x, int32_t p_y) const {
	if (!in_bounds(p_x, p_y)) {
		return false;
	}
	return get_material_def(material[cell_index(p_x, p_y)]).gas;
}

float WorldGrid::cell_density(int32_t p_x, int32_t p_y, float p_density_scale) const {
	if (!in_bounds(p_x, p_y)) {
		return std::max(p_density_scale, 1.0e-6f);
	}
	const int32_t i = cell_index(p_x, p_y);
	return std::max(density[i] * p_density_scale, 1.0e-6f);
}

void WorldGrid::rebuild_active_cells(float p_liquid_mass_epsilon) {
	active_liquid_cells.clear();
	active_powder_cells.clear();
	active_gas_cells.clear();
	active_fire_cells.clear();
	active_reaction_cells.clear();
	const int32_t count = cell_count();
	for (int32_t i = 0; i < count; i++) {
		const MaterialDef &def = get_material_def(material[i]);
		if (rigid_body_id[i] == 0 && def.liquid && std::isfinite(volume_fraction[i]) && volume_fraction[i] > p_liquid_mass_epsilon) {
			active_liquid_cells.push_back(i);
		}
		if (def.powder) {
			active_powder_cells.push_back(i);
		}
		if (rigid_body_id[i] == 0 && def.gas && std::isfinite(volume_fraction[i]) && volume_fraction[i] > 0.001f) {
			active_gas_cells.push_back(i);
		}
		if (material[i] == MATERIAL_FIRE) {
			active_fire_cells.push_back(i);
		}
		if (material[i] == MATERIAL_FIRE || temperature[i] > 0.05f || def.liquid || def.gas || def.powder) {
			active_reaction_cells.push_back(i);
		}
	}
}

void WorldGrid::make_rock(int32_t p_x, int32_t p_y) {
	if (!in_bounds(p_x, p_y)) {
		return;
	}
	const int32_t i = cell_index(p_x, p_y);
	material[i] = MATERIAL_ROCK;
	volume_fraction[i] = 0.0f;
	density[i] = get_material_def(MATERIAL_ROCK).density;
	toxic[i] = 0.0f;
	oil[i] = 0.0f;
	pressure[i] = 0.0f;
	velocity_x[i] = 0.0f;
	velocity_y[i] = 0.0f;
	temperature[i] = 0.0f;
	lifetime[i] = 0.0f;
}

void WorldGrid::make_air(int32_t p_x, int32_t p_y) {
	if (!in_bounds(p_x, p_y)) {
		return;
	}
	const int32_t i = cell_index(p_x, p_y);
	material[i] = MATERIAL_AIR;
	volume_fraction[i] = 0.0f;
	density[i] = get_material_def(MATERIAL_AIR).density;
	toxic[i] = 0.0f;
	oil[i] = 0.0f;
	pressure[i] = 0.0f;
	velocity_x[i] = 0.0f;
	velocity_y[i] = 0.0f;
	temperature[i] = 0.0f;
	lifetime[i] = 0.0f;
}

void WorldGrid::make_water(int32_t p_x, int32_t p_y, float p_mass) {
	if (!in_bounds(p_x, p_y) || blocks_velocity(p_x, p_y)) {
		return;
	}
	const int32_t i = cell_index(p_x, p_y);
	material[i] = MATERIAL_WATER;
	volume_fraction[i] = std::max(0.0f, p_mass);
	density[i] = get_material_def(MATERIAL_WATER).density;
	toxic[i] = 0.0f;
	oil[i] = 0.0f;
}

void WorldGrid::make_toxic(int32_t p_x, int32_t p_y, float p_mass) {
	if (!in_bounds(p_x, p_y) || blocks_velocity(p_x, p_y)) {
		return;
	}
	const int32_t i = cell_index(p_x, p_y);
	material[i] = MATERIAL_TOXIC;
	volume_fraction[i] = std::max(0.0f, p_mass);
	density[i] = get_material_def(MATERIAL_TOXIC).density;
	toxic[i] = volume_fraction[i];
	oil[i] = 0.0f;
}

void WorldGrid::make_oil(int32_t p_x, int32_t p_y, float p_mass) {
	if (!in_bounds(p_x, p_y) || blocks_velocity(p_x, p_y)) {
		return;
	}
	const int32_t i = cell_index(p_x, p_y);
	material[i] = MATERIAL_OIL;
	volume_fraction[i] = std::max(0.0f, p_mass);
	density[i] = get_material_def(MATERIAL_OIL).density;
	toxic[i] = 0.0f;
	oil[i] = volume_fraction[i];
}

void WorldGrid::make_sand(int32_t p_x, int32_t p_y, float p_mass) {
	if (!in_bounds(p_x, p_y) || is_solid_cell(p_x, p_y)) {
		return;
	}
	const int32_t i = cell_index(p_x, p_y);
	material[i] = MATERIAL_SAND;
	volume_fraction[i] = 1.0f;
	density[i] = std::max(get_material_def(MATERIAL_SAND).density, p_mass);
	toxic[i] = 0.0f;
	oil[i] = 0.0f;
	pressure[i] = 0.0f;
	velocity_x[i] = 0.0f;
	velocity_y[i] = 0.0f;
}

void WorldGrid::make_smoke(int32_t p_x, int32_t p_y, float p_mass) {
	if (!in_bounds(p_x, p_y) || blocks_velocity(p_x, p_y)) {
		return;
	}
	const int32_t i = cell_index(p_x, p_y);
	material[i] = MATERIAL_SMOKE;
	volume_fraction[i] = std::max(0.0f, p_mass);
	density[i] = std::max(get_material_def(MATERIAL_SMOKE).density, p_mass);
	toxic[i] = 0.0f;
	oil[i] = 0.0f;
	pressure[i] = 0.0f;
	velocity_x[i] = 0.0f;
	velocity_y[i] = -0.35f;
	lifetime[i] = 0.0f;
}

void WorldGrid::make_fire(int32_t p_x, int32_t p_y, float p_heat) {
	if (!in_bounds(p_x, p_y) || blocks_velocity(p_x, p_y)) {
		return;
	}
	const int32_t i = cell_index(p_x, p_y);
	material[i] = MATERIAL_FIRE;
	volume_fraction[i] = 1.0f;
	density[i] = get_material_def(MATERIAL_FIRE).density;
	toxic[i] = 0.0f;
	oil[i] = 0.0f;
	pressure[i] = 0.0f;
	velocity_x[i] = 0.0f;
	velocity_y[i] = -0.25f;
	temperature[i] = std::max(temperature[i], p_heat);
	lifetime[i] = 0.0f;
}

void WorldGrid::make_steam(int32_t p_x, int32_t p_y, float p_mass) {
	if (!in_bounds(p_x, p_y) || blocks_velocity(p_x, p_y)) {
		return;
	}
	const int32_t i = cell_index(p_x, p_y);
	material[i] = MATERIAL_STEAM;
	volume_fraction[i] = std::max(0.0f, p_mass);
	density[i] = get_material_def(MATERIAL_STEAM).density;
	toxic[i] = 0.0f;
	oil[i] = 0.0f;
	pressure[i] = 0.0f;
	velocity_x[i] = 0.0f;
	velocity_y[i] = -0.45f;
	temperature[i] = std::max(temperature[i], 0.75f);
	lifetime[i] = 0.0f;
}

void WorldGrid::make_toxic_gas(int32_t p_x, int32_t p_y, float p_mass) {
	if (!in_bounds(p_x, p_y) || blocks_velocity(p_x, p_y)) {
		return;
	}
	const int32_t i = cell_index(p_x, p_y);
	material[i] = MATERIAL_TOXIC_GAS;
	volume_fraction[i] = std::max(0.0f, p_mass);
	density[i] = get_material_def(MATERIAL_TOXIC_GAS).density;
	toxic[i] = volume_fraction[i];
	oil[i] = 0.0f;
	pressure[i] = 0.0f;
	velocity_x[i] = 0.0f;
	velocity_y[i] = -0.30f;
	temperature[i] = std::max(temperature[i], 0.65f);
	lifetime[i] = 0.0f;
}

void WorldGrid::make_flammable_gas(int32_t p_x, int32_t p_y, float p_mass) {
	if (!in_bounds(p_x, p_y) || blocks_velocity(p_x, p_y)) {
		return;
	}
	const int32_t i = cell_index(p_x, p_y);
	material[i] = MATERIAL_FLAMMABLE_GAS;
	volume_fraction[i] = std::max(0.0f, p_mass);
	density[i] = get_material_def(MATERIAL_FLAMMABLE_GAS).density;
	toxic[i] = 0.0f;
	oil[i] = 0.0f;
	pressure[i] = 0.0f;
	velocity_x[i] = 0.0f;
	velocity_y[i] = -0.22f;
	temperature[i] = std::max(temperature[i], 0.25f);
	lifetime[i] = 0.0f;
}

void WorldGrid::make_glass(int32_t p_x, int32_t p_y) {
	if (!in_bounds(p_x, p_y)) {
		return;
	}
	const int32_t i = cell_index(p_x, p_y);
	material[i] = MATERIAL_GLASS;
	volume_fraction[i] = 0.0f;
	density[i] = get_material_def(MATERIAL_GLASS).density;
	toxic[i] = 0.0f;
	oil[i] = 0.0f;
	pressure[i] = 0.0f;
	velocity_x[i] = 0.0f;
	velocity_y[i] = 0.0f;
	temperature[i] = std::max(temperature[i], 0.45f);
	lifetime[i] = 0.0f;
}


