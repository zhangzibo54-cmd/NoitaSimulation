#include "material_reaction_solver.h"

#include <algorithm>
#include <cmath>

using namespace noita;

namespace {
constexpr float WATER_BOIL_TEMP = 0.92f;
constexpr float STEAM_CONDENSE_TEMP = 0.22f;
constexpr float TOXIC_EVAP_TEMP = 0.82f;
constexpr float OIL_IGNITE_TEMP = 0.72f;
constexpr float SAND_GLASS_TEMP = 1.20f;
constexpr float ROCK_HOT_TEMP = 1.45f;
constexpr float AMBIENT_COOLING = 0.0025f;
constexpr float HOT_COOLING = 0.0060f;

float clampf(float v, float lo, float hi) {
	return std::max(lo, std::min(v, hi));
}
}

MaterialReactionSolver::MaterialReactionSolver() {
	build_default_rules();
}

void MaterialReactionSolver::build_default_rules() {
	rules.clear();
	// Self temperature rules.
	rules.push_back({ MATERIAL_WATER, TriggerKind::SelfTemperature, MATERIAL_AIR, WATER_BOIL_TEMP, MATERIAL_STEAM, 1.6f });
	rules.push_back({ MATERIAL_STEAM, TriggerKind::SelfTemperature, MATERIAL_AIR, -STEAM_CONDENSE_TEMP, MATERIAL_WATER, 1.0f });
	rules.push_back({ MATERIAL_TOXIC, TriggerKind::SelfTemperature, MATERIAL_AIR, TOXIC_EVAP_TEMP, MATERIAL_TOXIC_GAS, 1.8f });
	rules.push_back({ MATERIAL_OIL, TriggerKind::SelfTemperature, MATERIAL_AIR, OIL_IGNITE_TEMP, MATERIAL_FIRE, 1.0f });
	rules.push_back({ MATERIAL_SAND, TriggerKind::SelfTemperature, MATERIAL_AIR, SAND_GLASS_TEMP, MATERIAL_GLASS, 1.0f });
	// Neighbor-triggered rules.  These are intentionally applied only once per
	// cell per frame; chain reactions continue next frame.
	rules.push_back({ MATERIAL_WATER, TriggerKind::NeighborMaterial, MATERIAL_FIRE, 0.0f, MATERIAL_STEAM, 1.5f });
	rules.push_back({ MATERIAL_OIL, TriggerKind::NeighborMaterial, MATERIAL_FIRE, 0.0f, MATERIAL_FIRE, 1.0f });
	rules.push_back({ MATERIAL_TOXIC, TriggerKind::NeighborMaterial, MATERIAL_FIRE, 0.0f, MATERIAL_TOXIC_GAS, 1.8f });
	rules.push_back({ MATERIAL_FLAMMABLE_GAS, TriggerKind::NeighborMaterial, MATERIAL_FIRE, 0.0f, MATERIAL_FIRE, 1.0f });
}

bool MaterialReactionSolver::has_neighbor_material(const WorldGrid &p_grid, int32_t p_x, int32_t p_y, uint8_t p_material) const {
	const int32_t dirs[4][2] = {
		{ -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 },
	};
	for (const auto &dir : dirs) {
		const int32_t x = p_x + dir[0];
		const int32_t y = p_y + dir[1];
		if (!p_grid.in_bounds(x, y)) {
			continue;
		}
		if (p_grid.material[p_grid.cell_index(x, y)] == p_material) {
			return true;
		}
	}
	return false;
}

bool MaterialReactionSolver::try_place_gas_near(WorldGrid &p_grid, int32_t p_x, int32_t p_y, uint8_t p_gas, float p_volume, float p_temperature) {
	const int32_t dirs[5][2] = {
		{ 0, 0 }, { 0, -1 }, { -1, 0 }, { 1, 0 }, { 0, 1 },
	};
	for (const auto &dir : dirs) {
		const int32_t x = p_x + dir[0];
		const int32_t y = p_y + dir[1];
		if (!p_grid.in_bounds(x, y) || p_grid.blocks_velocity(x, y)) {
			continue;
		}
		const int32_t i = p_grid.cell_index(x, y);
		const MaterialDef &def = get_material_def(p_grid.material[i]);
		if (p_grid.material[i] != MATERIAL_AIR && !def.gas) {
			continue;
		}
		if (p_gas == MATERIAL_STEAM) {
			p_grid.make_steam(x, y, p_volume);
		} else if (p_gas == MATERIAL_TOXIC_GAS) {
			p_grid.make_toxic_gas(x, y, p_volume);
		} else if (p_gas == MATERIAL_FLAMMABLE_GAS) {
			p_grid.make_flammable_gas(x, y, p_volume);
		} else {
			p_grid.make_smoke(x, y, p_volume);
		}
		const int32_t gi = p_grid.cell_index(x, y);
		p_grid.temperature[gi] = std::max(p_grid.temperature[gi], p_temperature);
		return true;
	}
	return false;
}

void MaterialReactionSolver::apply_output(WorldGrid &p_grid, int32_t p_x, int32_t p_y, uint8_t p_output, float p_volume) {
	const int32_t i = p_grid.cell_index(p_x, p_y);
	const float old_temp = p_grid.temperature[i];
	if (p_output == MATERIAL_STEAM) {
		p_grid.make_air(p_x, p_y);
		try_place_gas_near(p_grid, p_x, p_y, MATERIAL_STEAM, p_volume, std::max(old_temp, 0.75f));
	} else if (p_output == MATERIAL_TOXIC_GAS) {
		p_grid.make_air(p_x, p_y);
		try_place_gas_near(p_grid, p_x, p_y, MATERIAL_TOXIC_GAS, p_volume, std::max(old_temp, 0.65f));
	} else if (p_output == MATERIAL_FIRE) {
		// Burning liquid consumes the local liquid cell and leaves a flame plus a
		// small smoke puff nearby.
		p_grid.make_fire(p_x, p_y, std::max(old_temp, 1.0f));
		try_place_gas_near(p_grid, p_x, p_y - 1, MATERIAL_SMOKE, 0.25f, 0.45f);
	} else if (p_output == MATERIAL_WATER) {
		p_grid.make_water(p_x, p_y, p_volume);
		p_grid.temperature[i] = 0.18f;
	} else if (p_output == MATERIAL_GLASS) {
		p_grid.make_glass(p_x, p_y);
	}
}

void MaterialReactionSolver::step(WorldGrid &p_grid) {
	reaction_order = p_grid.active_reaction_cells;
	if (reaction_order.empty()) {
		return;
	}
	const int32_t width = p_grid.width;

	for (int32_t idx : reaction_order) {
		if (idx < 0 || idx >= p_grid.cell_count()) {
			continue;
		}
		const uint8_t mat = p_grid.material[idx];
		const int32_t x = idx % width;
		const int32_t y = idx / width;

		// Passive cooling. Rock/glass keep heat longer; gases cool quickly.
		const MaterialDef &def = get_material_def(mat);
		float cooling = def.solid ? AMBIENT_COOLING : HOT_COOLING;
		if (mat == MATERIAL_FIRE) {
			cooling = 0.0f;
		}
		p_grid.temperature[idx] = std::max(0.0f, p_grid.temperature[idx] - cooling);

		// Rock currently only stores heat; no lava yet.
		if (mat == MATERIAL_ROCK && p_grid.temperature[idx] > ROCK_HOT_TEMP) {
			p_grid.temperature[idx] = clampf(p_grid.temperature[idx], 0.0f, 1.8f);
			continue;
		}

		bool reacted = false;
		for (const ReactionRule &rule : rules) {
			if (mat != rule.subject) {
				continue;
			}
			if (rule.trigger == TriggerKind::SelfTemperature) {
				if (rule.min_temperature >= 0.0f) {
					if (p_grid.temperature[idx] < rule.min_temperature) {
						continue;
					}
				} else {
					if (p_grid.temperature[idx] > -rule.min_temperature) {
						continue;
					}
				}
			} else if (!has_neighbor_material(p_grid, x, y, rule.neighbor)) {
				continue;
			}
			apply_output(p_grid, x, y, rule.output, std::max(0.05f, p_grid.volume_fraction[idx] * rule.output_volume));
			reacted = true;
			break;
		}
		if (reacted) {
			continue;
		}
	}
}

