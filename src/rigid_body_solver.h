#pragma once

#include "world_grid.h"

#include <cstdint>
#include <unordered_map>
#include <vector>

class RigidBodySolver {
public:
	struct RigidCellOccupancy {
		int32_t x = 0;
		int32_t y = 0;
		int32_t body_id = 0;
		float vx = 0.0f;
		float vy = 0.0f;
		bool moving = false;
		bool boundary = false;
	};

private:
	struct BodyCell {
		float lx = 0.0f;
		float ly = 0.0f;
		uint8_t material = noita::MATERIAL_ROCK;
	};

	struct RigidBody {
		int32_t id = 0;
		std::vector<BodyCell> cells;
		// Indices into cells. Full cells are still used for rendering / dynamic
		// occupancy, but collision only samples the boundary to avoid O(N^2)
		// pixel-pixel checks.
		std::vector<int32_t> boundary_indices;
		std::vector<int32_t> collision_sample_indices;
		float x = 0.0f;
		float y = 0.0f;
		float angle = 0.0f;
		float vx = 0.0f;
		float vy = 0.0f;
		float angular_velocity = 0.0f;
		float mass = 1.0f;
		float inv_mass = 1.0f;
		float inertia = 1.0f;
		float inv_inertia = 1.0f;
		int32_t min_x = 0;
		int32_t min_y = 0;
		int32_t max_x = 0;
		int32_t max_y = 0;
		int32_t wake_min_x = 0;
		int32_t wake_min_y = 0;
		int32_t wake_max_x = 0;
		int32_t wake_max_y = 0;
		float local_min_x = 0.0f;
		float local_min_y = 0.0f;
		float local_max_x = 0.0f;
		float local_max_y = 0.0f;
		float still_time = 0.0f;
		bool had_contact = false;
		bool active = true;
		bool sleeping = false;
	};

	struct BoundarySample {
		int32_t x = 0;
		int32_t y = 0;
		float wx = 0.0f;
		float wy = 0.0f;
		float nx = 0.0f;
		float ny = -1.0f;
	};

	struct StaticCollisionChunk {
		int32_t chunk_x = 0;
		int32_t chunk_y = 0;
		int32_t min_x = 0;
		int32_t min_y = 0;
		int32_t max_x = 0;
		int32_t max_y = 0;
		bool has_solid = false;
		std::vector<BoundarySample> samples;
		std::vector<int64_t> sample_keys;
	};

	struct BodyPair {
		int32_t a_id = 0;
		int32_t b_id = 0;
	};

	struct StaticPair {
		int32_t body_id = 0;
		int32_t chunk_index = 0;
	};

	struct Contact {
		float px = 0.0f;
		float py = 0.0f;
		float nx = 0.0f;
		float ny = -1.0f;
		float penetration = 0.0f;
	};

	struct CoveredCell {
		int32_t x = 0;
		int32_t y = 0;
		int32_t source_index = -1;
		int64_t key = 0;
	};

	float gravity = 0.035f;
	float linear_damping = 0.995f;
	float angular_damping = 0.990f;
	float settle_speed = 0.45f;
	float settle_angular_speed = 0.080f;
	float restitution = 0.22f;
	float friction = 0.55f;
	float positional_correction_fraction = 0.70f;
	float collision_slop = 0.03f;
	int32_t collision_iterations = 4;
	float boundary_sample_ratio = 0.50f;
	int32_t max_collision_samples = 160;
	int32_t wake_aabb_padding = 10;
	float fixed_step_seconds = 1.0f / 60.0f;
	float sleep_after_seconds = 0.20f;
	int32_t broad_phase_interval_steps = 6;
	int32_t broad_phase_counter = 0;
	int32_t broad_phase_cell_size = 64;
	int32_t static_chunk_size = 32;
	int32_t static_bucket_size = 4;
	int32_t dynamic_bucket_size = 4;
	int32_t max_dynamic_cells = 3000;
	int32_t max_flood_cells = 12000;
	int32_t next_body_id = 1;
	int32_t dragged_body = -1;
	bool auto_process_dirty = true;
	float drag_offset_x = 0.0f;
	float drag_offset_y = 0.0f;
	float drag_start_mouse_angle = 0.0f;
	float drag_start_body_angle = 0.0f;

	std::vector<RigidBody> bodies;
	mutable std::unordered_map<int32_t, int32_t> body_index_by_id;
	mutable bool body_index_dirty = true;
	std::vector<StaticCollisionChunk> static_chunks;
	int32_t static_chunks_x = 0;
	int32_t static_chunks_y = 0;
	bool static_chunks_dirty = true;
	std::vector<BodyPair> active_body_pairs;
	std::vector<StaticPair> active_static_pairs;
	std::vector<int64_t> sleeping_bucket_keys;
	std::vector<int32_t> sleeping_bucket_body_ids;
	bool sleeping_buckets_dirty = true;
	std::vector<int64_t> temp_keys;
	std::vector<int32_t> temp_indices;
	std::vector<int64_t> temp_pair_keys;
	std::vector<RigidCellOccupancy> current_occupied_cells;
	std::vector<int32_t> dirty_cells;
	std::vector<int32_t> flood_queue;
	std::vector<int32_t> island_cells;
	std::vector<int32_t> visit_stamp;
	int32_t current_stamp = 1;

	void ensure_buffers(const WorldGrid &p_grid);
	void mark_body_index_dirty();
	void rebuild_body_index_map() const;
	bool is_dynamic_solid_material(uint8_t p_material) const;
	bool is_anchor_cell(const WorldGrid &p_grid, int32_t p_x, int32_t p_y) const;
	void mark_visited(int32_t p_index);
	bool was_visited(int32_t p_index) const;
	bool flood_solid_island(WorldGrid &p_grid, int32_t p_seed, bool &r_anchored, bool &r_too_large);
	void create_body_from_island(WorldGrid &p_grid);
	void rebuild_boundary_samples(RigidBody &r_body);
	void rebuild_static_collision_chunks(const WorldGrid &p_grid);
	void ensure_static_collision_chunks(const WorldGrid &p_grid);
	void rebuild_sleeping_body_buckets();
	void rebuild_broad_phase_pairs(const WorldGrid &p_grid);
	bool body_has_support(const WorldGrid &p_grid, const RigidBody &p_body) const;
	void wake_sleeping_bodies_without_support_or_with_velocity(const WorldGrid &p_grid);
	void update_body_aabb(RigidBody &r_body) const;
	void recompute_mass_properties(RigidBody &r_body, bool p_recenter);
	void cell_world_position(const RigidBody &p_body, const BodyCell &p_cell, float &r_x, float &r_y) const;
	bool raster_cell(const RigidBody &p_body, const BodyCell &p_cell, int32_t &r_x, int32_t &r_y) const;
	void collect_body_covered_cells(const RigidBody &p_body, const WorldGrid *p_grid, const std::vector<int32_t> *p_indices, float p_half_extent, std::vector<CoveredCell> &r_cells) const;
	bool static_solid_or_boundary(const WorldGrid &p_grid, int32_t p_x, int32_t p_y) const;
	void estimate_contact_normal(const WorldGrid &p_grid, int32_t p_x, int32_t p_y, float p_wx, float p_wy, float p_fallback_vx, float p_fallback_vy, float &r_nx, float &r_ny) const;
	void collect_contacts(const WorldGrid &p_grid, const RigidBody &p_body, std::vector<Contact> &r_contacts) const;
	void build_body_cell_hash(const RigidBody &p_body, std::vector<int64_t> &r_keys) const;
	bool boundary_overlaps_body_cells(const RigidBody &p_boundary_body, const RigidBody &p_solid_body, const std::vector<int64_t> *p_solid_keys = nullptr, int32_t *r_overlap_count = nullptr) const;
	bool boundary_overlaps_static(const WorldGrid &p_grid, const RigidBody &p_body, int32_t *r_overlap_count = nullptr) const;
	bool has_overlap_blocking_sleep(const WorldGrid &p_grid, const RigidBody &p_body);
	void add_body_overlap_contacts(const RigidBody &p_boundary_body, const RigidBody &p_solid_body, bool p_normal_b_to_a, const std::vector<int64_t> &p_solid_keys, std::vector<Contact> &r_contacts) const;
	void add_static_overlap_contacts(const WorldGrid &p_grid, const RigidBody &p_body, const StaticCollisionChunk &p_chunk, std::vector<Contact> &r_contacts) const;
	void collect_static_chunk_contacts(const WorldGrid &p_grid, const RigidBody &p_body, const StaticCollisionChunk &p_chunk, std::vector<Contact> &r_contacts) const;
	void resolve_contacts(RigidBody &r_body, const std::vector<Contact> &p_contacts);
	void resolve_body_pair_contacts(RigidBody &r_a, RigidBody &r_b);
	void resolve_static_pair_contacts(const WorldGrid &p_grid, RigidBody &r_body, const StaticCollisionChunk &p_chunk);
	void resolve_dynamic_body_contacts(const WorldGrid &p_grid);
	RigidBody *find_body_by_id(int32_t p_id);
	const RigidBody *find_body_by_id(int32_t p_id) const;
	bool aabb_overlap_expanded(const RigidBody &p_a, const RigidBody &p_b, int32_t p_padding) const;
	void update_sleep_state(const WorldGrid &p_grid, RigidBody &r_body, bool p_had_contact);
	void bake_body_to_grid(WorldGrid &p_grid, RigidBody &r_body);
	void populate_dynamic_cells(WorldGrid &p_grid);
	int32_t find_body_at(float p_x, float p_y) const;

public:
	void clear();
	void rebuild_all(WorldGrid &p_grid);
	void set_auto_process_dirty(bool p_enabled);
	bool is_auto_process_dirty() const;
	void mark_dirty_cell(const WorldGrid &p_grid, int32_t p_x, int32_t p_y);
	void mark_dirty_rect(const WorldGrid &p_grid, int32_t p_min_x, int32_t p_min_y, int32_t p_max_x, int32_t p_max_y);
	void process_dirty(WorldGrid &p_grid);
	void step(WorldGrid &p_grid);
	void destroy_circle(WorldGrid &p_grid, float p_x, float p_y, float p_radius);
	void spawn_collision_test(WorldGrid &p_grid);
	void draw_overlay_rgba(const WorldGrid &p_grid, std::vector<uint8_t> &r_pixels) const;
	const std::vector<RigidCellOccupancy> &get_current_occupied_cells() const;

	bool start_drag(float p_x, float p_y);
	void update_drag(float p_x, float p_y, bool p_rotate);
	void end_drag();
	int32_t get_body_count() const;
	int32_t get_awake_body_count() const;
	int32_t get_sleeping_body_count() const;
};

