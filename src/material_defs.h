#pragma once

#include <cstdint>

namespace noita {

enum MaterialId : uint8_t {
	AIR = 0,
	ROCK = 1,
	WATER = 2,
	SAND = 3,
	SMOKE = 4,
	TOXIC = 5,
	OIL = 6,
	FIRE = 7,
	STEAM = 8,
	TOXIC_GAS = 9,
	FLAMMABLE_GAS = 10,
	GLASS = 11,
	MATERIAL_COUNT = 12,
};

// Backward-compatible numeric material ids used by Godot tools.
static constexpr uint8_t MATERIAL_AIR = AIR;
static constexpr uint8_t MATERIAL_ROCK = ROCK;
static constexpr uint8_t MATERIAL_WATER = WATER;
static constexpr uint8_t MATERIAL_SAND = SAND;
static constexpr uint8_t MATERIAL_SMOKE = SMOKE;
static constexpr uint8_t MATERIAL_TOXIC = TOXIC;
static constexpr uint8_t MATERIAL_OIL = OIL;
static constexpr uint8_t MATERIAL_FIRE = FIRE;
static constexpr uint8_t MATERIAL_STEAM = STEAM;
static constexpr uint8_t MATERIAL_TOXIC_GAS = TOXIC_GAS;
static constexpr uint8_t MATERIAL_FLAMMABLE_GAS = FLAMMABLE_GAS;
static constexpr uint8_t MATERIAL_GLASS = GLASS;

struct MaterialDef {
	const char *name;
	bool solid;
	bool liquid;
	bool gas;
	bool powder;
	bool pressure_solved;
	bool blocks_velocity;
	float density;
	float viscosity;
};

inline constexpr MaterialDef MATERIAL_DEFS[MATERIAL_COUNT] = {
	// name    solid  liquid gas    powder pressure blocks  density viscosity
	{ "air",   false, false, false, false, false,   false,  1.0e-3f, 0.0f },
	{ "rock",  true,  false, false, false, false,   true,   1000.0f, 0.0f },
	{ "water", false, true,  false, false, true,    false,  1.0f,    0.04f },
	{ "sand",  false, false, false, true,  false,   true,   1.6f,    0.0f },
	{ "smoke", false, false, true,  false, false,   false,  0.05f,   0.02f },
	{ "toxic", false, true,  false, false, true,    false,  1.02f,   0.05f },
	{ "oil",   false, true,  false, false, true,    false,  0.80f,   0.06f },
	{ "fire",  false, false, false, false, false,   false,  0.02f,   0.0f },
	{ "steam", false, false, true,  false, false,   false,  0.02f,   0.01f },
	{ "toxic_gas", false, false, true, false, false, false, 0.04f,   0.02f },
	{ "flammable_gas", false, false, true, false, false, false, 0.03f, 0.015f },
	{ "glass", true,  false, false, false, false,   true,   2.4f,    0.0f },
};

inline const MaterialDef &get_material_def(uint8_t p_material) {
	if (p_material >= MATERIAL_COUNT) {
		return MATERIAL_DEFS[AIR];
	}
	return MATERIAL_DEFS[p_material];
}

inline const MaterialDef &solid_boundary_material_def() {
	return MATERIAL_DEFS[ROCK];
}

} // namespace noita
