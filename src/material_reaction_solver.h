#pragma once

#include "world_grid.h"

#include <cstdint>
#include <vector>

class MaterialReactionSolver {
private:
	enum class TriggerKind : uint8_t {
		SelfTemperature,
		NeighborMaterial,
	};

	struct ReactionRule {
		uint8_t subject = noita::MATERIAL_AIR;
		TriggerKind trigger = TriggerKind::SelfTemperature;
		uint8_t neighbor = noita::MATERIAL_AIR;
		float min_temperature = 0.0f;
		uint8_t output = noita::MATERIAL_AIR;
		float output_volume = 1.0f;
	};

	std::vector<ReactionRule> rules;
	std::vector<int32_t> reaction_order;

	void build_default_rules();
	bool has_neighbor_material(const WorldGrid &p_grid, int32_t p_x, int32_t p_y, uint8_t p_material) const;
	bool try_place_gas_near(WorldGrid &p_grid, int32_t p_x, int32_t p_y, uint8_t p_gas, float p_volume, float p_temperature);
	void apply_output(WorldGrid &p_grid, int32_t p_x, int32_t p_y, uint8_t p_output, float p_volume);

public:
	MaterialReactionSolver();
	void step(WorldGrid &p_grid);
};

