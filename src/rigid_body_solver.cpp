#include "rigid_body_solver.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdint>

using namespace noita;

namespace {
constexpr int32_t SLEEP_BLOCKING_OVERLAP_CELLS = 12;

float clampf(float v, float lo, float hi) {
	return std::max(lo, std::min(v, hi));
}

uint8_t rock_shade(int32_t x, int32_t y, uint8_t base) {
	return static_cast<uint8_t>(std::min<int32_t>(255, base + ((x * 17 + y * 9) & 35)));
}

float cross2(float ax, float ay, float bx, float by) {
	return ax * by - ay * bx;
}

int64_t local_cell_key(float lx, float ly) {
	const int32_t qx = static_cast<int32_t>(std::llround(lx * 4.0f));
	const int32_t qy = static_cast<int32_t>(std::llround(ly * 4.0f));
	return (static_cast<int64_t>(qx) << 32) ^ static_cast<uint32_t>(qy);
}

int64_t grid_cell_key(int32_t x, int32_t y) {
	return (static_cast<int64_t>(x) << 32) ^ static_cast<uint32_t>(y);
}

uint32_t hash_u32(uint32_t x) {
	x ^= x >> 16;
	x *= 0x7feb352du;
	x ^= x >> 15;
	x *= 0x846ca68bu;
	x ^= x >> 16;
	return x;
}
} // namespace

void RigidBodySolver::clear() {
	bodies.clear();
	body_index_by_id.clear();
	body_index_dirty = false;
	static_chunks.clear();
	static_chunks_x = 0;
	static_chunks_y = 0;
	static_chunks_dirty = true;
	active_body_pairs.clear();
	active_static_pairs.clear();
	sleeping_bucket_keys.clear();
	sleeping_bucket_body_ids.clear();
	sleeping_buckets_dirty = true;
	temp_keys.clear();
	temp_indices.clear();
	temp_pair_keys.clear();
	current_occupied_cells.clear();
	dirty_cells.clear();
	flood_queue.clear();
	island_cells.clear();
	dragged_body = -1;
	auto_process_dirty = true;
	broad_phase_counter = 0;
}

void RigidBodySolver::mark_body_index_dirty() {
	body_index_dirty = true;
}

void RigidBodySolver::rebuild_body_index_map() const {
	if (!body_index_dirty) {
		return;
	}
	body_index_by_id.clear();
	body_index_by_id.reserve(bodies.size() * 2 + 1);
	for (int32_t i = 0; i < static_cast<int32_t>(bodies.size()); i++) {
		if (bodies[i].active) {
			body_index_by_id[bodies[i].id] = i;
		}
	}
	body_index_dirty = false;
}

void RigidBodySolver::set_auto_process_dirty(bool p_enabled) {
	auto_process_dirty = p_enabled;
}

bool RigidBodySolver::is_auto_process_dirty() const {
	return auto_process_dirty;
}

void RigidBodySolver::ensure_buffers(const WorldGrid &p_grid) {
	const int32_t count = p_grid.cell_count();
	if (static_cast<int32_t>(visit_stamp.size()) != count) {
		visit_stamp.assign(count, 0);
		current_stamp = 1;
	}
}

bool RigidBodySolver::is_dynamic_solid_material(uint8_t p_material) const {
	return p_material == MATERIAL_ROCK || p_material == MATERIAL_GLASS;
}

bool RigidBodySolver::is_anchor_cell(const WorldGrid &p_grid, int32_t p_x, int32_t p_y) const {
	return p_x <= 0 || p_y <= 0 || p_x >= p_grid.width - 1 || p_y >= p_grid.height - 1;
}

void RigidBodySolver::mark_visited(int32_t p_index) {
	visit_stamp[p_index] = current_stamp;
}

bool RigidBodySolver::was_visited(int32_t p_index) const {
	return p_index >= 0 && p_index < static_cast<int32_t>(visit_stamp.size()) && visit_stamp[p_index] == current_stamp;
}

void RigidBodySolver::mark_dirty_cell(const WorldGrid &p_grid, int32_t p_x, int32_t p_y) {
	if (!p_grid.in_bounds(p_x, p_y)) {
		return;
	}
	static_chunks_dirty = true;
	dirty_cells.push_back(p_grid.cell_index(p_x, p_y));
	for (RigidBody &body : bodies) {
		if (p_x >= body.min_x - 3 && p_x <= body.max_x + 3 && p_y >= body.min_y - 3 && p_y <= body.max_y + 3) {
			if (body.sleeping) {
				body.sleeping = false;
				body.still_time = 0.0f;
				sleeping_buckets_dirty = true;
			}
		}
	}
}

void RigidBodySolver::mark_dirty_rect(const WorldGrid &p_grid, int32_t p_min_x, int32_t p_min_y, int32_t p_max_x, int32_t p_max_y) {
	p_min_x = std::max(0, p_min_x);
	p_min_y = std::max(0, p_min_y);
	p_max_x = std::min(p_grid.width - 1, p_max_x);
	p_max_y = std::min(p_grid.height - 1, p_max_y);
	for (int32_t y = p_min_y; y <= p_max_y; y++) {
		for (int32_t x = p_min_x; x <= p_max_x; x++) {
			if (x == p_min_x || x == p_max_x || y == p_min_y || y == p_max_y) {
				mark_dirty_cell(p_grid, x, y);
			}
		}
	}
}

bool RigidBodySolver::flood_solid_island(WorldGrid &p_grid, int32_t p_seed, bool &r_anchored, bool &r_too_large) {
	r_anchored = false;
	r_too_large = false;
	island_cells.clear();
	flood_queue.clear();
	if (p_seed < 0 || p_seed >= p_grid.cell_count() || was_visited(p_seed) || !is_dynamic_solid_material(p_grid.material[p_seed])) {
		return false;
	}
	mark_visited(p_seed);
	flood_queue.push_back(p_seed);
	for (size_t head = 0; head < flood_queue.size(); head++) {
		const int32_t i = flood_queue[head];
		island_cells.push_back(i);
		const int32_t x = i % p_grid.width;
		const int32_t y = i / p_grid.width;
		if (is_anchor_cell(p_grid, x, y)) {
			r_anchored = true;
		}
		if (static_cast<int32_t>(island_cells.size()) > max_flood_cells) {
			r_too_large = true;
			return true;
		}
		const int32_t n[4] = { i - 1, i + 1, i - p_grid.width, i + p_grid.width };
		for (int32_t k = 0; k < 4; k++) {
			const int32_t ni = n[k];
			if (ni < 0 || ni >= p_grid.cell_count() || was_visited(ni)) {
				continue;
			}
			const int32_t nx = ni % p_grid.width;
			const int32_t ny = ni / p_grid.width;
			if (std::abs(nx - x) + std::abs(ny - y) != 1) {
				continue;
			}
			if (!is_dynamic_solid_material(p_grid.material[ni])) {
				continue;
			}
			mark_visited(ni);
			flood_queue.push_back(ni);
		}
	}
	return !island_cells.empty();
}

void RigidBodySolver::create_body_from_island(WorldGrid &p_grid) {
	if (island_cells.size() <= 1 || static_cast<int32_t>(island_cells.size()) > max_dynamic_cells) {
		return;
	}
	float cx = 0.0f;
	float cy = 0.0f;
	for (int32_t i : island_cells) {
		cx += static_cast<float>(i % p_grid.width) + 0.5f;
		cy += static_cast<float>(i / p_grid.width) + 0.5f;
	}
	const float inv = 1.0f / static_cast<float>(island_cells.size());
	cx *= inv;
	cy *= inv;

	RigidBody body;
	body.id = next_body_id++;
	body.x = cx;
	body.y = cy;
	body.cells.reserve(island_cells.size());
	for (int32_t i : island_cells) {
		const int32_t x = i % p_grid.width;
		const int32_t y = i / p_grid.width;
		BodyCell c;
		c.lx = (static_cast<float>(x) + 0.5f) - cx;
		c.ly = (static_cast<float>(y) + 0.5f) - cy;
		c.material = p_grid.material[i];
		body.cells.push_back(c);
		p_grid.make_air(x, y);
	}
	static_chunks_dirty = true;
	body.vy = 0.05f;
	body.angular_velocity = clampf((static_cast<float>((body.id * 1103515245u) & 1023u) / 1023.0f - 0.5f) * 0.018f, -0.02f, 0.02f);
	recompute_mass_properties(body, false);
	rebuild_boundary_samples(body);
	update_body_aabb(body);
	bodies.push_back(std::move(body));
	mark_body_index_dirty();
}

void RigidBodySolver::process_dirty(WorldGrid &p_grid) {
	if (dirty_cells.empty()) {
		return;
	}
	ensure_buffers(p_grid);
	current_stamp++;
	if (current_stamp == std::numeric_limits<int32_t>::max()) {
		std::fill(visit_stamp.begin(), visit_stamp.end(), 0);
		current_stamp = 1;
	}
	std::vector<int32_t> seeds;
	seeds.reserve(dirty_cells.size() * 5);
	for (int32_t i : dirty_cells) {
		if (i < 0 || i >= p_grid.cell_count()) {
			continue;
		}
		seeds.push_back(i);
		const int32_t x = i % p_grid.width;
		const int32_t y = i / p_grid.width;
		if (x > 0) seeds.push_back(i - 1);
		if (x + 1 < p_grid.width) seeds.push_back(i + 1);
		if (y > 0) seeds.push_back(i - p_grid.width);
		if (y + 1 < p_grid.height) seeds.push_back(i + p_grid.width);
	}
	dirty_cells.clear();
	for (int32_t seed : seeds) {
		if (seed < 0 || seed >= p_grid.cell_count() || was_visited(seed) || !is_dynamic_solid_material(p_grid.material[seed])) {
			continue;
		}
		bool anchored = false;
		bool too_large = false;
		if (!flood_solid_island(p_grid, seed, anchored, too_large)) {
			continue;
		}
		if (!anchored && !too_large && static_cast<int32_t>(island_cells.size()) <= max_dynamic_cells) {
			create_body_from_island(p_grid);
		}
	}
}

void RigidBodySolver::rebuild_all(WorldGrid &p_grid) {
	clear();
	ensure_buffers(p_grid);
	current_stamp++;
	for (int32_t i = 0; i < p_grid.cell_count(); i++) {
		if (was_visited(i) || !is_dynamic_solid_material(p_grid.material[i])) {
			continue;
		}
		bool anchored = false;
		bool too_large = false;
		if (!flood_solid_island(p_grid, i, anchored, too_large)) {
			continue;
		}
		if (!anchored && !too_large && static_cast<int32_t>(island_cells.size()) <= max_dynamic_cells) {
			create_body_from_island(p_grid);
		}
	}
}

void RigidBodySolver::rebuild_boundary_samples(RigidBody &r_body) {
	r_body.boundary_indices.clear();
	r_body.collision_sample_indices.clear();
	const int32_t count = static_cast<int32_t>(r_body.cells.size());
	if (count <= 0) {
		return;
	}

	// Build a quantized local occupancy map. This is intentionally done only
	// when the body topology changes (creation/destruction), not every frame.
	std::vector<int64_t> keys;
	keys.reserve(count);
	for (const BodyCell &c : r_body.cells) {
		keys.push_back(local_cell_key(c.lx, c.ly));
	}
	std::sort(keys.begin(), keys.end());

	auto has_key = [&](int64_t key) {
		return std::binary_search(keys.begin(), keys.end(), key);
	};

	for (int32_t i = 0; i < count; i++) {
		const BodyCell &c = r_body.cells[i];
		const int32_t qx = static_cast<int32_t>(std::llround(c.lx * 4.0f));
		const int32_t qy = static_cast<int32_t>(std::llround(c.ly * 4.0f));
		const int64_t left = (static_cast<int64_t>(qx - 4) << 32) ^ static_cast<uint32_t>(qy);
		const int64_t right = (static_cast<int64_t>(qx + 4) << 32) ^ static_cast<uint32_t>(qy);
		const int64_t up = (static_cast<int64_t>(qx) << 32) ^ static_cast<uint32_t>(qy - 4);
		const int64_t down = (static_cast<int64_t>(qx) << 32) ^ static_cast<uint32_t>(qy + 4);
		if (!has_key(left) || !has_key(right) || !has_key(up) || !has_key(down)) {
			r_body.boundary_indices.push_back(i);
		}
	}

	if (r_body.boundary_indices.empty()) {
		for (int32_t i = 0; i < count; i++) {
			r_body.boundary_indices.push_back(i);
		}
	}

	struct BucketPick {
		int64_t key = 0;
		int32_t index = 0;
		uint32_t hash = 0;
	};
	std::vector<BucketPick> bucket_picks;
	bucket_picks.reserve(r_body.boundary_indices.size());
	const int32_t bucket_size = std::max(1, dynamic_bucket_size);
	for (int32_t idx : r_body.boundary_indices) {
		const BodyCell &c = r_body.cells[idx];
		const int32_t bx = static_cast<int32_t>(std::floor(c.lx / static_cast<float>(bucket_size)));
		const int32_t by = static_cast<int32_t>(std::floor(c.ly / static_cast<float>(bucket_size)));
		const int64_t bkey = (static_cast<int64_t>(bx) << 32) ^ static_cast<uint32_t>(by);
		const uint32_t h = hash_u32(static_cast<uint32_t>(idx) ^ (static_cast<uint32_t>(r_body.id) * 747796405u));
		bucket_picks.push_back({ bkey, idx, h });
	}
	std::sort(bucket_picks.begin(), bucket_picks.end(), [](const BucketPick &a, const BucketPick &b) {
		if (a.key != b.key) {
			return a.key < b.key;
		}
		return a.hash < b.hash;
	});

	for (size_t i = 0; i < bucket_picks.size();) {
		size_t j = i + 1;
		while (j < bucket_picks.size() && bucket_picks[j].key == bucket_picks[i].key) {
			j++;
		}
		// Pick the best hash in every spatial bucket. This is still randomized
		// but spatially uniform, so an entire side cannot disappear by chance.
		r_body.collision_sample_indices.push_back(bucket_picks[i].index);
		if (boundary_sample_ratio > 0.65f && j - i > 2) {
			r_body.collision_sample_indices.push_back(bucket_picks[i + (j - i) / 2].index);
		}
		i = j;
	}

	if (r_body.collision_sample_indices.empty()) {
		r_body.collision_sample_indices.push_back(r_body.boundary_indices[0]);
	}

	// Keep a hard cap for collision cost. Use deterministic stride downsampling
	// so results are stable for a given body id.
	if (static_cast<int32_t>(r_body.collision_sample_indices.size()) > max_collision_samples) {
		std::vector<int32_t> reduced;
		reduced.reserve(max_collision_samples);
		const int32_t n = static_cast<int32_t>(r_body.collision_sample_indices.size());
		const int32_t offset = r_body.id % std::max(1, n);
		for (int32_t k = 0; k < max_collision_samples; k++) {
			const int32_t src = (offset + (k * n) / max_collision_samples) % n;
			reduced.push_back(r_body.collision_sample_indices[src]);
		}
		r_body.collision_sample_indices.swap(reduced);
	}
}

void RigidBodySolver::rebuild_static_collision_chunks(const WorldGrid &p_grid) {
	static_chunks.clear();
	const int32_t cs = std::max(4, static_chunk_size);
	static_chunks_x = (p_grid.width + cs - 1) / cs;
	static_chunks_y = (p_grid.height + cs - 1) / cs;
	static_chunks.resize(static_cast<size_t>(static_chunks_x * static_chunks_y));

	for (int32_t cy = 0; cy < static_chunks_y; cy++) {
		for (int32_t cx = 0; cx < static_chunks_x; cx++) {
			StaticCollisionChunk &chunk = static_chunks[static_cast<size_t>(cy * static_chunks_x + cx)];
			chunk.chunk_x = cx;
			chunk.chunk_y = cy;
			chunk.min_x = cx * cs;
			chunk.min_y = cy * cs;
			chunk.max_x = std::min(p_grid.width - 1, (cx + 1) * cs - 1);
			chunk.max_y = std::min(p_grid.height - 1, (cy + 1) * cs - 1);
			chunk.has_solid = false;
			chunk.samples.clear();
			chunk.sample_keys.clear();

			struct StaticPick {
				int64_t key = 0;
				BoundarySample sample;
				uint32_t hash = 0;
			};
			std::vector<StaticPick> picks;

			for (int32_t y = chunk.min_y; y <= chunk.max_y; y++) {
				for (int32_t x = chunk.min_x; x <= chunk.max_x; x++) {
					const int32_t i = p_grid.cell_index(x, y);
					if (!get_material_def(p_grid.material[i]).solid) {
						continue;
					}
					chunk.has_solid = true;
					const bool left_open = !static_solid_or_boundary(p_grid, x - 1, y);
					const bool right_open = !static_solid_or_boundary(p_grid, x + 1, y);
					const bool up_open = !static_solid_or_boundary(p_grid, x, y - 1);
					const bool down_open = !static_solid_or_boundary(p_grid, x, y + 1);
					if (!left_open && !right_open && !up_open && !down_open) {
						continue;
					}

					float nx = 0.0f;
					float ny = 0.0f;
					if (left_open) nx -= 1.0f;
					if (right_open) nx += 1.0f;
					if (up_open) ny -= 1.0f;
					if (down_open) ny += 1.0f;
					float len = std::sqrt(nx * nx + ny * ny);
					if (len < 0.0001f) {
						nx = 0.0f;
						ny = -1.0f;
						len = 1.0f;
					}

					BoundarySample s;
					s.x = x;
					s.y = y;
					s.wx = static_cast<float>(x) + 0.5f;
					s.wy = static_cast<float>(y) + 0.5f;
					// Normal points from static solid into open space / body.
					s.nx = nx / len;
					s.ny = ny / len;

					const int32_t bx = x / std::max(1, static_bucket_size);
					const int32_t by = y / std::max(1, static_bucket_size);
					const int64_t bkey = (static_cast<int64_t>(bx) << 32) ^ static_cast<uint32_t>(by);
					const uint32_t h = hash_u32(static_cast<uint32_t>(x * 73856093u) ^ static_cast<uint32_t>(y * 19349663u));
					picks.push_back({ bkey, s, h });
				}
			}

			std::sort(picks.begin(), picks.end(), [](const StaticPick &a, const StaticPick &b) {
				if (a.key != b.key) {
					return a.key < b.key;
				}
				return a.hash < b.hash;
			});
			for (size_t i = 0; i < picks.size();) {
				size_t j = i + 1;
				while (j < picks.size() && picks[j].key == picks[i].key) {
					j++;
				}
				chunk.samples.push_back(picks[i].sample);
				i = j;
			}
			chunk.sample_keys.reserve(chunk.samples.size());
			for (const BoundarySample &s : chunk.samples) {
				chunk.sample_keys.push_back(grid_cell_key(s.x, s.y));
			}
			std::sort(chunk.sample_keys.begin(), chunk.sample_keys.end());
		}
	}
	static_chunks_dirty = false;
}

void RigidBodySolver::ensure_static_collision_chunks(const WorldGrid &p_grid) {
	const int32_t cs = std::max(4, static_chunk_size);
	const int32_t expected_x = (p_grid.width + cs - 1) / cs;
	const int32_t expected_y = (p_grid.height + cs - 1) / cs;
	if (static_chunks_dirty || static_chunks_x != expected_x || static_chunks_y != expected_y ||
			static_cast<int32_t>(static_chunks.size()) != expected_x * expected_y) {
		rebuild_static_collision_chunks(p_grid);
	}
}

void RigidBodySolver::rebuild_sleeping_body_buckets() {
	sleeping_bucket_keys.clear();
	sleeping_bucket_body_ids.clear();
	struct SleepEntry {
		int64_t key = 0;
		int32_t body_id = 0;
	};
	std::vector<SleepEntry> entries;
	const int32_t bs = std::max(4, broad_phase_cell_size);
	for (const RigidBody &body : bodies) {
		if (!body.active || !body.sleeping) {
			continue;
		}
		const int32_t min_bx = static_cast<int32_t>(std::floor(static_cast<float>(body.wake_min_x) / static_cast<float>(bs)));
		const int32_t max_bx = static_cast<int32_t>(std::floor(static_cast<float>(body.wake_max_x) / static_cast<float>(bs)));
		const int32_t min_by = static_cast<int32_t>(std::floor(static_cast<float>(body.wake_min_y) / static_cast<float>(bs)));
		const int32_t max_by = static_cast<int32_t>(std::floor(static_cast<float>(body.wake_max_y) / static_cast<float>(bs)));
		for (int32_t by = min_by; by <= max_by; by++) {
			for (int32_t bx = min_bx; bx <= max_bx; bx++) {
				entries.push_back({ grid_cell_key(bx, by), body.id });
			}
		}
	}
	std::sort(entries.begin(), entries.end(), [](const SleepEntry &a, const SleepEntry &b) {
		if (a.key != b.key) {
			return a.key < b.key;
		}
		return a.body_id < b.body_id;
	});
	sleeping_bucket_keys.reserve(entries.size());
	sleeping_bucket_body_ids.reserve(entries.size());
	for (const SleepEntry &e : entries) {
		sleeping_bucket_keys.push_back(e.key);
		sleeping_bucket_body_ids.push_back(e.body_id);
	}
	sleeping_buckets_dirty = false;
}

void RigidBodySolver::recompute_mass_properties(RigidBody &r_body, bool p_recenter) {
	if (r_body.cells.empty()) {
		r_body.mass = 1.0f;
		r_body.inv_mass = 1.0f;
		r_body.inertia = 1.0f;
		r_body.inv_inertia = 1.0f;
		r_body.local_min_x = r_body.local_min_y = 0.0f;
		r_body.local_max_x = r_body.local_max_y = 0.0f;
		return;
	}

	if (p_recenter) {
		float cx = 0.0f;
		float cy = 0.0f;
		for (const BodyCell &c : r_body.cells) {
			cx += c.lx;
			cy += c.ly;
		}
		const float inv_count = 1.0f / static_cast<float>(r_body.cells.size());
		cx *= inv_count;
		cy *= inv_count;
		if (std::abs(cx) > 0.0001f || std::abs(cy) > 0.0001f) {
			const float ca = std::cos(r_body.angle);
			const float sa = std::sin(r_body.angle);
			r_body.x += ca * cx - sa * cy;
			r_body.y += sa * cx + ca * cy;
			for (BodyCell &c : r_body.cells) {
				c.lx -= cx;
				c.ly -= cy;
			}
		}
	}

	constexpr float LOCAL_CELL_HALF_EXTENT = 0.52f;
	r_body.local_min_x = std::numeric_limits<float>::max();
	r_body.local_min_y = std::numeric_limits<float>::max();
	r_body.local_max_x = std::numeric_limits<float>::lowest();
	r_body.local_max_y = std::numeric_limits<float>::lowest();
	for (const BodyCell &c : r_body.cells) {
		r_body.local_min_x = std::min(r_body.local_min_x, c.lx - LOCAL_CELL_HALF_EXTENT);
		r_body.local_min_y = std::min(r_body.local_min_y, c.ly - LOCAL_CELL_HALF_EXTENT);
		r_body.local_max_x = std::max(r_body.local_max_x, c.lx + LOCAL_CELL_HALF_EXTENT);
		r_body.local_max_y = std::max(r_body.local_max_y, c.ly + LOCAL_CELL_HALF_EXTENT);
	}

	r_body.mass = std::max(1.0f, static_cast<float>(r_body.cells.size()));
	r_body.inv_mass = 1.0f / r_body.mass;
	float inertia = 0.0f;
	for (const BodyCell &c : r_body.cells) {
		// Unit square pixel inertia around its own center is 1/6.
		inertia += c.lx * c.lx + c.ly * c.ly + 1.0f / 6.0f;
	}
	r_body.inertia = std::max(1.0f, inertia);
	r_body.inv_inertia = 1.0f / r_body.inertia;
}

void RigidBodySolver::cell_world_position(const RigidBody &p_body, const BodyCell &p_cell, float &r_x, float &r_y) const {
	const float ca = std::cos(p_body.angle);
	const float sa = std::sin(p_body.angle);
	r_x = p_body.x + ca * p_cell.lx - sa * p_cell.ly;
	r_y = p_body.y + sa * p_cell.lx + ca * p_cell.ly;
}

bool RigidBodySolver::raster_cell(const RigidBody &p_body, const BodyCell &p_cell, int32_t &r_x, int32_t &r_y) const {
	float wx = 0.0f;
	float wy = 0.0f;
	cell_world_position(p_body, p_cell, wx, wy);
	r_x = static_cast<int32_t>(std::floor(wx));
	r_y = static_cast<int32_t>(std::floor(wy));
	return true;
}

void RigidBodySolver::collect_body_covered_cells(const RigidBody &p_body, const WorldGrid *p_grid, const std::vector<int32_t> *p_indices, float p_half_extent, std::vector<CoveredCell> &r_cells) const {
	r_cells.clear();
	if (p_body.cells.empty()) {
		return;
	}
	const float ca = std::cos(p_body.angle);
	const float sa = std::sin(p_body.angle);
	const float half = std::max(0.45f, p_half_extent);
	const float extent_x = half * (std::abs(ca) + std::abs(sa)) + 0.001f;
	const float extent_y = extent_x;
	const int32_t count = p_indices == nullptr ? static_cast<int32_t>(p_body.cells.size()) : static_cast<int32_t>(p_indices->size());
	r_cells.reserve(static_cast<size_t>(count * 2));

	for (int32_t ii = 0; ii < count; ii++) {
		const int32_t source_index = p_indices == nullptr ? ii : (*p_indices)[ii];
		if (source_index < 0 || source_index >= static_cast<int32_t>(p_body.cells.size())) {
			continue;
		}
		const BodyCell &cell = p_body.cells[source_index];
		const float wx = p_body.x + ca * cell.lx - sa * cell.ly;
		const float wy = p_body.y + sa * cell.lx + ca * cell.ly;
		const int32_t min_x = static_cast<int32_t>(std::floor(wx - extent_x));
		const int32_t max_x = static_cast<int32_t>(std::floor(wx + extent_x));
		const int32_t min_y = static_cast<int32_t>(std::floor(wy - extent_y));
		const int32_t max_y = static_cast<int32_t>(std::floor(wy + extent_y));

		for (int32_t y = min_y; y <= max_y; y++) {
			for (int32_t x = min_x; x <= max_x; x++) {
				if (p_grid != nullptr && !p_grid->in_bounds(x, y)) {
					continue;
				}
				const float dx = (static_cast<float>(x) + 0.5f) - wx;
				const float dy = (static_cast<float>(y) + 0.5f) - wy;
				const float local_x = ca * dx + sa * dy;
				const float local_y = -sa * dx + ca * dy;
				if (std::abs(local_x) > half || std::abs(local_y) > half) {
					continue;
				}
				CoveredCell covered;
				covered.x = x;
				covered.y = y;
				covered.source_index = source_index;
				covered.key = grid_cell_key(x, y);
				r_cells.push_back(covered);
			}
		}
	}

	std::sort(r_cells.begin(), r_cells.end(), [](const CoveredCell &a, const CoveredCell &b) {
		if (a.key != b.key) {
			return a.key < b.key;
		}
		return a.source_index < b.source_index;
	});
	r_cells.erase(std::unique(r_cells.begin(), r_cells.end(), [](const CoveredCell &a, const CoveredCell &b) {
		return a.key == b.key;
	}), r_cells.end());
}

void RigidBodySolver::update_body_aabb(RigidBody &r_body) const {
	if (r_body.cells.empty()) {
		r_body.min_x = r_body.max_x = static_cast<int32_t>(std::floor(r_body.x));
		r_body.min_y = r_body.max_y = static_cast<int32_t>(std::floor(r_body.y));
		r_body.wake_min_x = r_body.min_x - wake_aabb_padding;
		r_body.wake_min_y = r_body.min_y - wake_aabb_padding;
		r_body.wake_max_x = r_body.max_x + wake_aabb_padding;
		r_body.wake_max_y = r_body.max_y + wake_aabb_padding;
		return;
	}
	r_body.min_x = std::numeric_limits<int32_t>::max();
	r_body.min_y = std::numeric_limits<int32_t>::max();
	r_body.max_x = std::numeric_limits<int32_t>::min();
	r_body.max_y = std::numeric_limits<int32_t>::min();

	const float ca = std::cos(r_body.angle);
	const float sa = std::sin(r_body.angle);
	const float xs[2] = { r_body.local_min_x, r_body.local_max_x };
	const float ys[2] = { r_body.local_min_y, r_body.local_max_y };
	float min_x = std::numeric_limits<float>::max();
	float min_y = std::numeric_limits<float>::max();
	float max_x = std::numeric_limits<float>::lowest();
	float max_y = std::numeric_limits<float>::lowest();
	for (float lx : xs) {
		for (float ly : ys) {
			const float wx = r_body.x + ca * lx - sa * ly;
			const float wy = r_body.y + sa * lx + ca * ly;
			min_x = std::min(min_x, wx);
			min_y = std::min(min_y, wy);
			max_x = std::max(max_x, wx);
			max_y = std::max(max_y, wy);
		}
	}
	// The local bounds already include the conservative per-pixel half extent.
	// Pad by one grid cell because rasterization tests pixel centers and because
	// broad-phase false positives are much cheaper than missed contacts.
	r_body.min_x = static_cast<int32_t>(std::floor(min_x)) - 1;
	r_body.min_y = static_cast<int32_t>(std::floor(min_y)) - 1;
	r_body.max_x = static_cast<int32_t>(std::floor(max_x)) + 1;
	r_body.max_y = static_cast<int32_t>(std::floor(max_y)) + 1;
	r_body.wake_min_x = r_body.min_x - wake_aabb_padding;
	r_body.wake_min_y = r_body.min_y - wake_aabb_padding;
	r_body.wake_max_x = r_body.max_x + wake_aabb_padding;
	r_body.wake_max_y = r_body.max_y + wake_aabb_padding;
}

bool RigidBodySolver::static_solid_or_boundary(const WorldGrid &p_grid, int32_t p_x, int32_t p_y) const {
	if (!p_grid.in_bounds(p_x, p_y)) {
		return true;
	}
	const int32_t i = p_grid.cell_index(p_x, p_y);
	return get_material_def(p_grid.material[i]).solid;
}

void RigidBodySolver::estimate_contact_normal(const WorldGrid &p_grid, int32_t p_x, int32_t p_y, float p_wx, float p_wy, float p_fallback_vx, float p_fallback_vy, float &r_nx, float &r_ny) const {
	if (p_x < 0) {
		r_nx = 1.0f;
		r_ny = 0.0f;
		return;
	}
	if (p_x >= p_grid.width) {
		r_nx = -1.0f;
		r_ny = 0.0f;
		return;
	}
	if (p_y < 0) {
		r_nx = 0.0f;
		r_ny = 1.0f;
		return;
	}
	if (p_y >= p_grid.height) {
		r_nx = 0.0f;
		r_ny = -1.0f;
		return;
	}

	const float solid_left = static_solid_or_boundary(p_grid, p_x - 1, p_y) ? 1.0f : 0.0f;
	const float solid_right = static_solid_or_boundary(p_grid, p_x + 1, p_y) ? 1.0f : 0.0f;
	const float solid_up = static_solid_or_boundary(p_grid, p_x, p_y - 1) ? 1.0f : 0.0f;
	const float solid_down = static_solid_or_boundary(p_grid, p_x, p_y + 1) ? 1.0f : 0.0f;

	float nx = -(solid_right - solid_left);
	float ny = -(solid_down - solid_up);
	float len = std::sqrt(nx * nx + ny * ny);
	if (len < 0.0001f) {
		// When deeply embedded in a flat raster collider the local gradient can
		// be zero. Use the opposite of contact velocity as a stable fallback.
		nx = -p_fallback_vx;
		ny = -p_fallback_vy;
		len = std::sqrt(nx * nx + ny * ny);
	}
	if (len < 0.0001f) {
		nx = p_wx - (static_cast<float>(p_x) + 0.5f);
		ny = p_wy - (static_cast<float>(p_y) + 0.5f);
		len = std::sqrt(nx * nx + ny * ny);
	}
	if (len < 0.0001f) {
		nx = 0.0f;
		ny = -1.0f;
		len = 1.0f;
	}
	r_nx = nx / len;
	r_ny = ny / len;
}

void RigidBodySolver::collect_contacts(const WorldGrid &p_grid, const RigidBody &p_body, std::vector<Contact> &r_contacts) const {
	r_contacts.clear();
	const std::vector<int32_t> &samples = p_body.collision_sample_indices.empty() ? p_body.boundary_indices : p_body.collision_sample_indices;
	if (samples.empty()) {
		return;
	}
	for (int32_t sample_index : samples) {
		if (sample_index < 0 || sample_index >= static_cast<int32_t>(p_body.cells.size())) {
			continue;
		}
		const BodyCell &c = p_body.cells[sample_index];
		float wx = 0.0f;
		float wy = 0.0f;
		cell_world_position(p_body, c, wx, wy);
		const int32_t x = static_cast<int32_t>(std::floor(wx));
		const int32_t y = static_cast<int32_t>(std::floor(wy));
		if (!static_solid_or_boundary(p_grid, x, y)) {
			continue;
		}

		const float rx = wx - p_body.x;
		const float ry = wy - p_body.y;
		const float fallback_vx = p_body.vx - p_body.angular_velocity * ry;
		const float fallback_vy = p_body.vy + p_body.angular_velocity * rx;
		Contact contact;
		contact.px = wx;
		contact.py = wy;
		estimate_contact_normal(p_grid, x, y, wx, wy, fallback_vx, fallback_vy, contact.nx, contact.ny);
		if (x < 0) {
			contact.penetration = std::max(0.0f, -wx);
		} else if (x >= p_grid.width) {
			contact.penetration = std::max(0.0f, wx - static_cast<float>(p_grid.width - 1));
		} else if (y < 0) {
			contact.penetration = std::max(0.0f, -wy);
		} else if (y >= p_grid.height) {
			contact.penetration = std::max(0.0f, wy - static_cast<float>(p_grid.height - 1));
		} else {
			// Pixel bodies are rasterized by center points. Once a center falls
			// into a solid cell, pushing by about half a cell along the surface
			// normal is enough to escape the overlap without snapping too hard.
			contact.penetration = 0.55f;
		}
		r_contacts.push_back(contact);
	}
}

void RigidBodySolver::build_body_cell_hash(const RigidBody &p_body, std::vector<int64_t> &r_keys) const {
	r_keys.clear();
	std::vector<CoveredCell> covered;
	collect_body_covered_cells(p_body, nullptr, nullptr, 0.52f, covered);
	r_keys.reserve(covered.size());
	for (const CoveredCell &c : covered) {
		r_keys.push_back(c.key);
	}
	std::sort(r_keys.begin(), r_keys.end());
	r_keys.erase(std::unique(r_keys.begin(), r_keys.end()), r_keys.end());
}

bool RigidBodySolver::boundary_overlaps_body_cells(const RigidBody &p_boundary_body, const RigidBody &p_solid_body, const std::vector<int64_t> *p_solid_keys, int32_t *r_overlap_count) const {
	if (r_overlap_count != nullptr) {
		*r_overlap_count = 0;
	}
	if (p_boundary_body.boundary_indices.empty() || p_solid_body.cells.empty()) {
		return false;
	}

	std::vector<int64_t> local_keys;
	const std::vector<int64_t> *keys = p_solid_keys;
	if (keys == nullptr) {
		build_body_cell_hash(p_solid_body, local_keys);
		keys = &local_keys;
	}
	if (keys->empty()) {
		return false;
	}

	int32_t overlaps = 0;
	std::vector<CoveredCell> boundary_cells;
	collect_body_covered_cells(p_boundary_body, nullptr, &p_boundary_body.boundary_indices, 0.52f, boundary_cells);
	for (const CoveredCell &c : boundary_cells) {
		if (std::binary_search(keys->begin(), keys->end(), c.key)) {
			overlaps++;
			if (r_overlap_count == nullptr) {
				return true;
			}
		}
	}
	if (r_overlap_count != nullptr) {
		*r_overlap_count = overlaps;
	}
	return overlaps > 0;
}

bool RigidBodySolver::boundary_overlaps_static(const WorldGrid &p_grid, const RigidBody &p_body, int32_t *r_overlap_count) const {
	if (r_overlap_count != nullptr) {
		*r_overlap_count = 0;
	}
	int32_t overlaps = 0;
	std::vector<CoveredCell> boundary_cells;
	collect_body_covered_cells(p_body, &p_grid, &p_body.boundary_indices, 0.52f, boundary_cells);
	for (const CoveredCell &c : boundary_cells) {
		if (static_solid_or_boundary(p_grid, c.x, c.y)) {
			overlaps++;
			if (r_overlap_count == nullptr) {
				return true;
			}
		}
	}
	if (r_overlap_count != nullptr) {
		*r_overlap_count = overlaps;
	}
	return overlaps > 0;
}

bool RigidBodySolver::has_overlap_blocking_sleep(const WorldGrid &p_grid, const RigidBody &p_body) {
	int32_t static_overlap_count = 0;
	if (boundary_overlaps_static(p_grid, p_body, &static_overlap_count) && static_overlap_count > SLEEP_BLOCKING_OVERLAP_CELLS) {
		return true;
	}

	auto overlaps_candidate = [&](const RigidBody *other) -> bool {
		if (other == nullptr || !other->active || other->id == p_body.id) {
			return false;
		}
		if (p_body.max_x < other->min_x || other->max_x < p_body.min_x ||
				p_body.max_y < other->min_y || other->max_y < p_body.min_y) {
			return false;
		}
		int32_t overlap_count = 0;
		return boundary_overlaps_body_cells(p_body, *other, nullptr, &overlap_count) && overlap_count > SLEEP_BLOCKING_OVERLAP_CELLS;
	};

	for (const BodyPair &pair : active_body_pairs) {
		if (pair.a_id == p_body.id) {
			if (overlaps_candidate(find_body_by_id(pair.b_id))) {
				return true;
			}
		} else if (pair.b_id == p_body.id) {
			if (overlaps_candidate(find_body_by_id(pair.a_id))) {
				return true;
			}
		}
	}

	if (sleeping_buckets_dirty) {
		rebuild_sleeping_body_buckets();
	}
	const int32_t bs = std::max(4, broad_phase_cell_size);
	const int32_t min_bx = static_cast<int32_t>(std::floor(static_cast<float>(p_body.min_x - 1) / static_cast<float>(bs)));
	const int32_t max_bx = static_cast<int32_t>(std::floor(static_cast<float>(p_body.max_x + 1) / static_cast<float>(bs)));
	const int32_t min_by = static_cast<int32_t>(std::floor(static_cast<float>(p_body.min_y - 1) / static_cast<float>(bs)));
	const int32_t max_by = static_cast<int32_t>(std::floor(static_cast<float>(p_body.max_y + 1) / static_cast<float>(bs)));
	std::vector<int32_t> checked_ids;
	for (int32_t by = min_by; by <= max_by; by++) {
		for (int32_t bx = min_bx; bx <= max_bx; bx++) {
			const int64_t key = grid_cell_key(bx, by);
			auto it = std::lower_bound(sleeping_bucket_keys.begin(), sleeping_bucket_keys.end(), key);
			for (; it != sleeping_bucket_keys.end() && *it == key; ++it) {
				const int32_t idx = static_cast<int32_t>(it - sleeping_bucket_keys.begin());
				const int32_t other_id = sleeping_bucket_body_ids[idx];
				if (other_id == p_body.id || std::find(checked_ids.begin(), checked_ids.end(), other_id) != checked_ids.end()) {
					continue;
				}
				checked_ids.push_back(other_id);
				const RigidBody *other = find_body_by_id(other_id);
				if (other == nullptr || !other->sleeping) {
					continue;
				}
				if (overlaps_candidate(other)) {
					return true;
				}
			}
		}
	}
	return false;
}

void RigidBodySolver::add_body_overlap_contacts(const RigidBody &p_boundary_body, const RigidBody &p_solid_body, bool p_normal_b_to_a, const std::vector<int64_t> &p_solid_keys, std::vector<Contact> &r_contacts) const {
	if (p_boundary_body.boundary_indices.empty() || p_solid_keys.empty()) {
		return;
	}
	const size_t start_count = r_contacts.size();
	int32_t overlap_count = 0;
	float sum_x = 0.0f;
	float sum_y = 0.0f;
	const int32_t max_extra_contacts = 16;

	std::vector<CoveredCell> boundary_cells;
	collect_body_covered_cells(p_boundary_body, nullptr, &p_boundary_body.boundary_indices, 0.52f, boundary_cells);
	for (const CoveredCell &covered : boundary_cells) {
		if (!std::binary_search(p_solid_keys.begin(), p_solid_keys.end(), covered.key)) {
			continue;
		}
		const float wx = static_cast<float>(covered.x) + 0.5f;
		const float wy = static_cast<float>(covered.y) + 0.5f;
		overlap_count++;
		sum_x += wx;
		sum_y += wy;
		if (static_cast<int32_t>(r_contacts.size() - start_count) < max_extra_contacts) {
			Contact c;
			c.px = wx;
			c.py = wy;
			float nx = p_normal_b_to_a ? (p_boundary_body.x - p_solid_body.x) : (p_solid_body.x - p_boundary_body.x);
			float ny = p_normal_b_to_a ? (p_boundary_body.y - p_solid_body.y) : (p_solid_body.y - p_boundary_body.y);
			float len = std::sqrt(nx * nx + ny * ny);
			if (len < 0.0001f) {
				nx = p_normal_b_to_a ? (wx - p_solid_body.x) : (p_solid_body.x - wx);
				ny = p_normal_b_to_a ? (wy - p_solid_body.y) : (p_solid_body.y - wy);
				len = std::sqrt(nx * nx + ny * ny);
			}
			if (len < 0.0001f) {
				nx = 1.0f;
				ny = 0.0f;
				len = 1.0f;
			}
			c.nx = nx / len;
			c.ny = ny / len;
			c.penetration = 0.75f;
			r_contacts.push_back(c);
		}
	}

	if (overlap_count > 0) {
		Contact c;
		c.px = sum_x / static_cast<float>(overlap_count);
		c.py = sum_y / static_cast<float>(overlap_count);
		float nx = p_normal_b_to_a ? (p_boundary_body.x - p_solid_body.x) : (p_solid_body.x - p_boundary_body.x);
		float ny = p_normal_b_to_a ? (p_boundary_body.y - p_solid_body.y) : (p_solid_body.y - p_boundary_body.y);
		float len = std::sqrt(nx * nx + ny * ny);
		if (len < 0.0001f) {
			nx = p_normal_b_to_a ? (c.px - p_solid_body.x) : (p_solid_body.x - c.px);
			ny = p_normal_b_to_a ? (c.py - p_solid_body.y) : (p_solid_body.y - c.py);
			len = std::sqrt(nx * nx + ny * ny);
		}
		if (len < 0.0001f) {
			nx = 1.0f;
			ny = 0.0f;
			len = 1.0f;
		}
		c.nx = nx / len;
		c.ny = ny / len;
		// Overlap count approximates "overlap area" without all-cell pairwise
		// checks. Feed it mostly into positional correction, capped to avoid
		// explosive impulses when two chunks are deeply embedded.
		c.penetration = clampf(0.65f + 0.045f * std::sqrt(static_cast<float>(overlap_count)), 0.65f, 1.65f);
		r_contacts.push_back(c);
	}
}

void RigidBodySolver::add_static_overlap_contacts(const WorldGrid &p_grid, const RigidBody &p_body, const StaticCollisionChunk &p_chunk, std::vector<Contact> &r_contacts) const {
	const size_t start_count = r_contacts.size();
	int32_t overlap_count = 0;
	float sum_x = 0.0f;
	float sum_y = 0.0f;
	float sum_nx = 0.0f;
	float sum_ny = 0.0f;
	const int32_t max_extra_contacts = 16;
	std::vector<CoveredCell> boundary_cells;
	collect_body_covered_cells(p_body, &p_grid, &p_body.boundary_indices, 0.52f, boundary_cells);
	for (const CoveredCell &covered : boundary_cells) {
		const int32_t x = covered.x;
		const int32_t y = covered.y;
		if (x < p_chunk.min_x - 1 || x > p_chunk.max_x + 1 || y < p_chunk.min_y - 1 || y > p_chunk.max_y + 1) {
			continue;
		}
		if (!static_solid_or_boundary(p_grid, x, y)) {
			continue;
		}
		const float wx = static_cast<float>(x) + 0.5f;
		const float wy = static_cast<float>(y) + 0.5f;
		const float rx = wx - p_body.x;
		const float ry = wy - p_body.y;
		const float fallback_vx = p_body.vx - p_body.angular_velocity * ry;
		const float fallback_vy = p_body.vy + p_body.angular_velocity * rx;
		float nx = 0.0f;
		float ny = -1.0f;
		estimate_contact_normal(p_grid, x, y, wx, wy, fallback_vx, fallback_vy, nx, ny);

		// If the boundary point is already deep inside a raster solid, the
		// immediate 4-neighbor gradient can be zero or point in a poor fallback
		// direction. Search for the nearest open cell and use the direction from
		// solid to open as a stronger depenetration normal.
		float escape_nx = nx;
		float escape_ny = ny;
		float best_score = 999999.0f;
		const int32_t dirs[8][2] = {
			{ 0, -1 }, { 0, 1 }, { -1, 0 }, { 1, 0 },
			{ -1, -1 }, { 1, -1 }, { -1, 1 }, { 1, 1 },
		};
		for (int32_t dist = 1; dist <= 10; dist++) {
			for (const auto &dir : dirs) {
				const int32_t tx = x + dir[0] * dist;
				const int32_t ty = y + dir[1] * dist;
				if (static_solid_or_boundary(p_grid, tx, ty)) {
					continue;
				}
				const float diag_penalty = (dir[0] != 0 && dir[1] != 0) ? 0.41f : 0.0f;
				const float score = static_cast<float>(dist) + diag_penalty;
				if (score < best_score) {
					best_score = score;
					escape_nx = static_cast<float>(dir[0]);
					escape_ny = static_cast<float>(dir[1]);
				}
			}
			if (best_score < 999999.0f) {
				break;
			}
		}
		float escape_len = std::sqrt(escape_nx * escape_nx + escape_ny * escape_ny);
		if (escape_len > 0.0001f) {
			nx = escape_nx / escape_len;
			ny = escape_ny / escape_len;
		}
		overlap_count++;
		sum_x += wx;
		sum_y += wy;
		sum_nx += nx;
		sum_ny += ny;
		if (static_cast<int32_t>(r_contacts.size() - start_count) < max_extra_contacts) {
			Contact c;
			c.px = wx;
			c.py = wy;
			c.nx = nx;
			c.ny = ny;
			c.penetration = 1.15f;
			r_contacts.push_back(c);
		}
	}
	if (overlap_count > 0) {
		Contact c;
		c.px = sum_x / static_cast<float>(overlap_count);
		c.py = sum_y / static_cast<float>(overlap_count);
		float len = std::sqrt(sum_nx * sum_nx + sum_ny * sum_ny);
		if (len < 0.0001f) {
			c.nx = 0.0f;
			c.ny = -1.0f;
		} else {
			c.nx = sum_nx / len;
			c.ny = sum_ny / len;
		}
		c.penetration = clampf(1.10f + 0.12f * std::sqrt(static_cast<float>(overlap_count)), 1.10f, 4.00f);
		r_contacts.push_back(c);
	}
}

void RigidBodySolver::collect_static_chunk_contacts(const WorldGrid &p_grid, const RigidBody &p_body, const StaticCollisionChunk &p_chunk, std::vector<Contact> &r_contacts) const {
	r_contacts.clear();
	if (!p_chunk.has_solid || p_chunk.samples.empty()) {
		return;
	}
	const std::vector<int32_t> &samples = p_body.collision_sample_indices.empty() ? p_body.boundary_indices : p_body.collision_sample_indices;
	if (samples.empty()) {
		return;
	}

	for (int32_t sample_index : samples) {
		if (sample_index < 0 || sample_index >= static_cast<int32_t>(p_body.cells.size())) {
			continue;
		}
		const BodyCell &bc = p_body.cells[sample_index];
		float wx = 0.0f;
		float wy = 0.0f;
		cell_world_position(p_body, bc, wx, wy);
		const int32_t gx = static_cast<int32_t>(std::floor(wx));
		const int32_t gy = static_cast<int32_t>(std::floor(wy));
		if (gx < p_chunk.min_x - 1 || gx > p_chunk.max_x + 1 || gy < p_chunk.min_y - 1 || gy > p_chunk.max_y + 1) {
			continue;
		}

		for (int32_t oy = -1; oy <= 1; oy++) {
			bool found = false;
			for (int32_t ox = -1; ox <= 1; ox++) {
				const int64_t key = grid_cell_key(gx + ox, gy + oy);
				if (!std::binary_search(p_chunk.sample_keys.begin(), p_chunk.sample_keys.end(), key)) {
					continue;
				}
				for (const BoundarySample &ss : p_chunk.samples) {
					if (ss.x != gx + ox || ss.y != gy + oy) {
						continue;
					}
					Contact c;
					c.px = wx;
					c.py = wy;
					c.nx = ss.nx;
					c.ny = ss.ny;
					c.penetration = (ox == 0 && oy == 0) ? 0.60f : 0.25f;
					r_contacts.push_back(c);
					found = true;
					break;
				}
				if (found || r_contacts.size() >= 24) {
					break;
				}
			}
			if (found || r_contacts.size() >= 24) {
				break;
			}
		}
		if (r_contacts.size() >= 24) {
			break;
		}
	}
	add_static_overlap_contacts(p_grid, p_body, p_chunk, r_contacts);
}

void RigidBodySolver::resolve_contacts(RigidBody &r_body, const std::vector<Contact> &p_contacts) {
	if (p_contacts.empty() || r_body.inv_mass <= 0.0f) {
		return;
	}

	for (const Contact &c : p_contacts) {
		const float rx = c.px - r_body.x;
		const float ry = c.py - r_body.y;
		const float vcx = r_body.vx - r_body.angular_velocity * ry;
		const float vcy = r_body.vy + r_body.angular_velocity * rx;
		const float vn = vcx * c.nx + vcy * c.ny;
		if (vn < 0.0f) {
			const float rn = cross2(rx, ry, c.nx, c.ny);
			const float denom = r_body.inv_mass + rn * rn * r_body.inv_inertia;
			if (denom > 0.000001f) {
				const float j = -(1.0f + restitution) * vn / denom;
				const float jx = j * c.nx;
				const float jy = j * c.ny;
				r_body.vx += jx * r_body.inv_mass;
				r_body.vy += jy * r_body.inv_mass;
				r_body.angular_velocity += cross2(rx, ry, jx, jy) * r_body.inv_inertia;

				const float tx = -c.ny;
				const float ty = c.nx;
				const float vt = vcx * tx + vcy * ty;
				const float rt = cross2(rx, ry, tx, ty);
				const float tdenom = r_body.inv_mass + rt * rt * r_body.inv_inertia;
				if (tdenom > 0.000001f) {
					float jt = -vt / tdenom;
					const float max_friction = friction * j;
					jt = clampf(jt, -max_friction, max_friction);
					const float fjx = jt * tx;
					const float fjy = jt * ty;
					r_body.vx += fjx * r_body.inv_mass;
					r_body.vy += fjy * r_body.inv_mass;
					r_body.angular_velocity += cross2(rx, ry, fjx, fjy) * r_body.inv_inertia;
				}
			}
		}
	}

	float push_x = 0.0f;
	float push_y = 0.0f;
	int32_t push_count = 0;
	for (const Contact &c : p_contacts) {
		const float correction = std::max(0.0f, c.penetration - collision_slop);
		if (correction <= 0.0f) {
			continue;
		}
		push_x += c.nx * correction;
		push_y += c.ny * correction;
		push_count++;
	}
	if (push_count > 0) {
		const float scale = positional_correction_fraction / static_cast<float>(push_count);
		r_body.x += push_x * scale;
		r_body.y += push_y * scale;
	}
}

void RigidBodySolver::resolve_static_pair_contacts(const WorldGrid &p_grid, RigidBody &r_body, const StaticCollisionChunk &p_chunk) {
	if (!r_body.active || r_body.sleeping || !p_chunk.has_solid) {
		return;
	}
	std::vector<Contact> contacts;
	for (int32_t iter = 0; iter < std::max(1, collision_iterations); iter++) {
		collect_static_chunk_contacts(p_grid, r_body, p_chunk, contacts);
		if (contacts.empty()) {
			break;
		}
		r_body.had_contact = true;
		resolve_contacts(r_body, contacts);
		update_body_aabb(r_body);
	}
}

void RigidBodySolver::resolve_body_pair_contacts(RigidBody &r_a, RigidBody &r_b) {
	if (!r_a.active || !r_b.active || (r_a.sleeping && r_b.sleeping)) {
		return;
	}
	if (r_a.max_x < r_b.min_x - 1 || r_b.max_x < r_a.min_x - 1 || r_a.max_y < r_b.min_y - 1 || r_b.max_y < r_a.min_y - 1) {
		return;
	}

	std::vector<Contact> contacts;
	contacts.reserve(16);

	const std::vector<int32_t> &samples_a = r_a.collision_sample_indices.empty() ? r_a.boundary_indices : r_a.collision_sample_indices;
	const std::vector<int32_t> &samples_b = r_b.collision_sample_indices.empty() ? r_b.boundary_indices : r_b.collision_sample_indices;
	if (samples_a.empty() || samples_b.empty()) {
		return;
	}

	struct BodySampleWorld {
		int32_t x = 0;
		int32_t y = 0;
		float wx = 0.0f;
		float wy = 0.0f;
	};
	struct KeyedBodySample {
		int64_t key = 0;
		BodySampleWorld sample;
	};
	std::vector<KeyedBodySample> b_hash;
	b_hash.reserve(samples_b.size());
	for (int32_t sample_b : samples_b) {
		if (sample_b < 0 || sample_b >= static_cast<int32_t>(r_b.cells.size())) {
			continue;
		}
		const BodyCell &cb = r_b.cells[sample_b];
		BodySampleWorld bs;
		cell_world_position(r_b, cb, bs.wx, bs.wy);
		bs.x = static_cast<int32_t>(std::floor(bs.wx));
		bs.y = static_cast<int32_t>(std::floor(bs.wy));
		b_hash.push_back({ grid_cell_key(bs.x, bs.y), bs });
	}
	std::sort(b_hash.begin(), b_hash.end(), [](const KeyedBodySample &a, const KeyedBodySample &b) {
		return a.key < b.key;
	});

	for (int32_t sample_a : samples_a) {
		if (sample_a < 0 || sample_a >= static_cast<int32_t>(r_a.cells.size())) {
			continue;
		}
		const BodyCell &ca = r_a.cells[sample_a];
		float ax = 0.0f;
		float ay = 0.0f;
		cell_world_position(r_a, ca, ax, ay);
		const int32_t agx = static_cast<int32_t>(std::floor(ax));
		const int32_t agy = static_cast<int32_t>(std::floor(ay));
		for (int32_t oy = -1; oy <= 1; oy++) {
			bool found = false;
			for (int32_t ox = -1; ox <= 1; ox++) {
				const int64_t key = grid_cell_key(agx + ox, agy + oy);
				auto it = std::lower_bound(b_hash.begin(), b_hash.end(), key, [](const KeyedBodySample &a, const int64_t &k) {
					return a.key < k;
				});
				if (it == b_hash.end() || it->key != key) {
					continue;
				}
				const BodySampleWorld &bs = it->sample;
				Contact c;
				c.px = 0.5f * (ax + bs.wx);
				c.py = 0.5f * (ay + bs.wy);
				float nx = ax - bs.wx;
				float ny = ay - bs.wy;
				float len = std::sqrt(nx * nx + ny * ny);
				if (len < 0.0001f) {
					nx = r_a.x - r_b.x;
					ny = r_a.y - r_b.y;
					len = std::sqrt(nx * nx + ny * ny);
				}
				if (len < 0.0001f) {
					nx = 1.0f;
					ny = 0.0f;
					len = 1.0f;
				}
				c.nx = nx / len;
				c.ny = ny / len;
				c.penetration = (ox == 0 && oy == 0) ? 0.65f : 0.28f;
				contacts.push_back(c);
				found = true;
				if (contacts.size() >= 24) {
					break;
				}
			}
			if (found || contacts.size() >= 24) {
				break;
			}
		}
		if (contacts.size() >= 24) {
			break;
		}
	}

	std::vector<int64_t> a_full_keys;
	std::vector<int64_t> b_full_keys;
	build_body_cell_hash(r_a, a_full_keys);
	build_body_cell_hash(r_b, b_full_keys);
	// Add robust deep-overlap contacts: full boundary of one body against all
	// occupied cells of the other body. This catches cases missed by the cheap
	// sampled-boundary narrow phase without degenerating to all-cells vs all-cells.
	add_body_overlap_contacts(r_a, r_b, true, b_full_keys, contacts);
	add_body_overlap_contacts(r_b, r_a, false, a_full_keys, contacts);

	if (contacts.empty()) {
		return;
	}
	r_a.had_contact = true;
	r_b.had_contact = true;

	const bool a_was_sleeping = r_a.sleeping;
	const bool b_was_sleeping = r_b.sleeping;
	const float speed_a = std::sqrt(r_a.vx * r_a.vx + r_a.vy * r_a.vy);
	const float speed_b = std::sqrt(r_b.vx * r_b.vx + r_b.vy * r_b.vy);
	const bool a_can_wake_b = speed_a > settle_speed * 1.5f || std::abs(r_a.angular_velocity) > settle_angular_speed * 1.5f;
	const bool b_can_wake_a = speed_b > settle_speed * 1.5f || std::abs(r_b.angular_velocity) > settle_angular_speed * 1.5f;
	if ((a_was_sleeping && b_can_wake_a) || (b_was_sleeping && a_can_wake_b)) {
		sleeping_buckets_dirty = true;
	}
	if (a_was_sleeping && b_can_wake_a) {
		r_a.sleeping = false;
		r_a.still_time = 0.0f;
	}
	if (b_was_sleeping && a_can_wake_b) {
		r_b.sleeping = false;
		r_b.still_time = 0.0f;
	}

	for (const Contact &c : contacts) {
		const float rax = c.px - r_a.x;
		const float ray = c.py - r_a.y;
		const float rbx = c.px - r_b.x;
		const float rby = c.py - r_b.y;
		const float vax = r_a.vx - r_a.angular_velocity * ray;
		const float vay = r_a.vy + r_a.angular_velocity * rax;
		const float vbx = r_b.vx - r_b.angular_velocity * rby;
		const float vby = r_b.vy + r_b.angular_velocity * rbx;
		const float rvx = vax - vbx;
		const float rvy = vay - vby;
		const float vn = rvx * c.nx + rvy * c.ny;
		if (vn >= 0.0f) {
			continue;
		}

		const float ran = cross2(rax, ray, c.nx, c.ny);
		const float rbn = cross2(rbx, rby, c.nx, c.ny);
		const float denom = r_a.inv_mass + r_b.inv_mass + ran * ran * r_a.inv_inertia + rbn * rbn * r_b.inv_inertia;
		if (denom <= 0.000001f) {
			continue;
		}
		const float j = -(1.0f + restitution) * vn / denom;
		const float jx = j * c.nx;
		const float jy = j * c.ny;
		r_a.vx += jx * r_a.inv_mass;
		r_a.vy += jy * r_a.inv_mass;
		r_a.angular_velocity += cross2(rax, ray, jx, jy) * r_a.inv_inertia;
		r_b.vx -= jx * r_b.inv_mass;
		r_b.vy -= jy * r_b.inv_mass;
		r_b.angular_velocity -= cross2(rbx, rby, jx, jy) * r_b.inv_inertia;

		const float tx = -c.ny;
		const float ty = c.nx;
		const float vt = rvx * tx + rvy * ty;
		const float rat = cross2(rax, ray, tx, ty);
		const float rbt = cross2(rbx, rby, tx, ty);
		const float tdenom = r_a.inv_mass + r_b.inv_mass + rat * rat * r_a.inv_inertia + rbt * rbt * r_b.inv_inertia;
		if (tdenom > 0.000001f) {
			float jt = -vt / tdenom;
			const float max_friction = friction * j;
			jt = clampf(jt, -max_friction, max_friction);
			const float fjx = jt * tx;
			const float fjy = jt * ty;
			r_a.vx += fjx * r_a.inv_mass;
			r_a.vy += fjy * r_a.inv_mass;
			r_a.angular_velocity += cross2(rax, ray, fjx, fjy) * r_a.inv_inertia;
			r_b.vx -= fjx * r_b.inv_mass;
			r_b.vy -= fjy * r_b.inv_mass;
			r_b.angular_velocity -= cross2(rbx, rby, fjx, fjy) * r_b.inv_inertia;
		}
	}

	float push_x = 0.0f;
	float push_y = 0.0f;
	for (const Contact &c : contacts) {
		const float correction = std::max(0.0f, c.penetration - collision_slop);
		push_x += c.nx * correction;
		push_y += c.ny * correction;
	}
	const float inv_sum = std::max(0.0001f, r_a.inv_mass + r_b.inv_mass);
	const float scale = positional_correction_fraction / static_cast<float>(contacts.size());
	r_a.x += push_x * scale * (r_a.inv_mass / inv_sum);
	r_a.y += push_y * scale * (r_a.inv_mass / inv_sum);
	r_b.x -= push_x * scale * (r_b.inv_mass / inv_sum);
	r_b.y -= push_y * scale * (r_b.inv_mass / inv_sum);
	update_body_aabb(r_a);
	update_body_aabb(r_b);
}

void RigidBodySolver::resolve_dynamic_body_contacts(const WorldGrid &p_grid) {
	for (int32_t iter = 0; iter < std::max(1, collision_iterations); iter++) {
		for (const BodyPair &pair : active_body_pairs) {
			RigidBody *a = find_body_by_id(pair.a_id);
			RigidBody *b = find_body_by_id(pair.b_id);
			if (a == nullptr || b == nullptr) {
				continue;
			}
			resolve_body_pair_contacts(*a, *b);
		}
	}
	for (const StaticPair &pair : active_static_pairs) {
		RigidBody *body = find_body_by_id(pair.body_id);
		if (body == nullptr || pair.chunk_index < 0 || pair.chunk_index >= static_cast<int32_t>(static_chunks.size())) {
			continue;
		}
		resolve_static_pair_contacts(p_grid, *body, static_chunks[pair.chunk_index]);
	}
}

RigidBodySolver::RigidBody *RigidBodySolver::find_body_by_id(int32_t p_id) {
	rebuild_body_index_map();
	auto it = body_index_by_id.find(p_id);
	if (it == body_index_by_id.end()) {
		return nullptr;
	}
	const int32_t idx = it->second;
	if (idx < 0 || idx >= static_cast<int32_t>(bodies.size()) || !bodies[idx].active || bodies[idx].id != p_id) {
		body_index_dirty = true;
		return nullptr;
	}
	return &bodies[idx];
}

const RigidBodySolver::RigidBody *RigidBodySolver::find_body_by_id(int32_t p_id) const {
	rebuild_body_index_map();
	auto it = body_index_by_id.find(p_id);
	if (it == body_index_by_id.end()) {
		return nullptr;
	}
	const int32_t idx = it->second;
	if (idx < 0 || idx >= static_cast<int32_t>(bodies.size()) || !bodies[idx].active || bodies[idx].id != p_id) {
		body_index_dirty = true;
		return nullptr;
	}
	return &bodies[idx];
}

bool RigidBodySolver::aabb_overlap_expanded(const RigidBody &p_a, const RigidBody &p_b, int32_t p_padding) const {
	return !(p_a.max_x + p_padding < p_b.min_x - p_padding ||
			p_b.max_x + p_padding < p_a.min_x - p_padding ||
			p_a.max_y + p_padding < p_b.min_y - p_padding ||
			p_b.max_y + p_padding < p_a.min_y - p_padding);
}

void RigidBodySolver::rebuild_broad_phase_pairs(const WorldGrid &p_grid) {
	ensure_static_collision_chunks(p_grid);
	wake_sleeping_bodies_without_support_or_with_velocity(p_grid);
	if (sleeping_buckets_dirty) {
		rebuild_sleeping_body_buckets();
	}
	active_body_pairs.clear();
	active_static_pairs.clear();
	temp_pair_keys.clear();

	struct BucketEntry {
		int64_t key = 0;
		int32_t body_id = 0;
	};
	std::vector<BucketEntry> active_entries;
	const int32_t bs = std::max(4, broad_phase_cell_size);

	auto add_body_pair_key = [&](int32_t id_a, int32_t id_b) {
		if (id_a == id_b) {
			return;
		}
		const int32_t a = std::min(id_a, id_b);
		const int32_t b = std::max(id_a, id_b);
		temp_pair_keys.push_back((static_cast<int64_t>(a) << 32) ^ static_cast<uint32_t>(b));
	};

	auto insert_active_body = [&](const RigidBody &body) {
		const int32_t min_bx = static_cast<int32_t>(std::floor(static_cast<float>(body.min_x - 2) / static_cast<float>(bs)));
		const int32_t max_bx = static_cast<int32_t>(std::floor(static_cast<float>(body.max_x + 2) / static_cast<float>(bs)));
		const int32_t min_by = static_cast<int32_t>(std::floor(static_cast<float>(body.min_y - 2) / static_cast<float>(bs)));
		const int32_t max_by = static_cast<int32_t>(std::floor(static_cast<float>(body.max_y + 2) / static_cast<float>(bs)));
		for (int32_t by = min_by; by <= max_by; by++) {
			for (int32_t bx = min_bx; bx <= max_bx; bx++) {
				active_entries.push_back({ grid_cell_key(bx, by), body.id });
			}
		}
	};

	for (RigidBody &body : bodies) {
		if (!body.active || body.sleeping) {
			continue;
		}
		insert_active_body(body);
	}

	std::sort(active_entries.begin(), active_entries.end(), [](const BucketEntry &a, const BucketEntry &b) {
		if (a.key != b.key) {
			return a.key < b.key;
		}
		return a.body_id < b.body_id;
	});

	for (size_t i = 0; i < active_entries.size();) {
		size_t j = i + 1;
		while (j < active_entries.size() && active_entries[j].key == active_entries[i].key) {
			j++;
		}
		for (size_t a = i; a < j; a++) {
			const RigidBody *body_a = find_body_by_id(active_entries[a].body_id);
			if (body_a == nullptr || body_a->sleeping) {
				continue;
			}
			for (size_t b = a + 1; b < j; b++) {
				const RigidBody *body_b = find_body_by_id(active_entries[b].body_id);
				if (body_b == nullptr || body_b->sleeping) {
					continue;
				}
				if (aabb_overlap_expanded(*body_a, *body_b, 2)) {
					add_body_pair_key(body_a->id, body_b->id);
				}
			}
		}
		i = j;
	}

	// Active bodies query the persistent sleeping bucket. Sleeping bodies do
	// not need to be reinserted every broad phase because their transform is
	// frozen until they are woken.
	for (RigidBody &body : bodies) {
		if (!body.active || body.sleeping) {
			continue;
		}
		const int32_t min_bx = static_cast<int32_t>(std::floor(static_cast<float>(body.min_x - wake_aabb_padding) / static_cast<float>(bs)));
		const int32_t max_bx = static_cast<int32_t>(std::floor(static_cast<float>(body.max_x + wake_aabb_padding) / static_cast<float>(bs)));
		const int32_t min_by = static_cast<int32_t>(std::floor(static_cast<float>(body.min_y - wake_aabb_padding) / static_cast<float>(bs)));
		const int32_t max_by = static_cast<int32_t>(std::floor(static_cast<float>(body.max_y + wake_aabb_padding) / static_cast<float>(bs)));
		for (int32_t by = min_by; by <= max_by; by++) {
			for (int32_t bx = min_bx; bx <= max_bx; bx++) {
				const int64_t key = grid_cell_key(bx, by);
				auto it = std::lower_bound(sleeping_bucket_keys.begin(), sleeping_bucket_keys.end(), key);
				for (; it != sleeping_bucket_keys.end() && *it == key; ++it) {
					const int32_t idx = static_cast<int32_t>(it - sleeping_bucket_keys.begin());
					RigidBody *other = find_body_by_id(sleeping_bucket_body_ids[idx]);
					if (other == nullptr || !other->sleeping) {
						continue;
					}
					if (body.max_x < other->wake_min_x || body.min_x > other->wake_max_x ||
							body.max_y < other->wake_min_y || body.min_y > other->wake_max_y) {
						continue;
					}
					add_body_pair_key(body.id, other->id);
				}
			}
		}
	}

	std::sort(temp_pair_keys.begin(), temp_pair_keys.end());
	temp_pair_keys.erase(std::unique(temp_pair_keys.begin(), temp_pair_keys.end()), temp_pair_keys.end());
	for (int64_t key : temp_pair_keys) {
		active_body_pairs.push_back({ static_cast<int32_t>(key >> 32), static_cast<int32_t>(key & 0xffffffffu) });
	}

	const int32_t cs = std::max(4, static_chunk_size);
	for (RigidBody &body : bodies) {
		if (!body.active || body.sleeping) {
			continue;
		}
		const int32_t min_cx = std::max(0, (body.min_x - 2) / cs);
		const int32_t max_cx = std::min(static_chunks_x - 1, (body.max_x + 2) / cs);
		const int32_t min_cy = std::max(0, (body.min_y - 2) / cs);
		const int32_t max_cy = std::min(static_chunks_y - 1, (body.max_y + 2) / cs);
		if (min_cx > max_cx || min_cy > max_cy) {
			continue;
		}
		for (int32_t cy = min_cy; cy <= max_cy; cy++) {
			for (int32_t cx = min_cx; cx <= max_cx; cx++) {
				const int32_t ci = cy * static_chunks_x + cx;
				if (ci < 0 || ci >= static_cast<int32_t>(static_chunks.size())) {
					continue;
				}
				const StaticCollisionChunk &chunk = static_chunks[ci];
				if (chunk.has_solid && !chunk.samples.empty()) {
					active_static_pairs.push_back({ body.id, ci });
				}
			}
		}
	}
}

void RigidBodySolver::update_sleep_state(const WorldGrid &p_grid, RigidBody &r_body, bool p_had_contact) {
	const float speed = std::sqrt(r_body.vx * r_body.vx + r_body.vy * r_body.vy);
	const bool has_rest_support = p_had_contact || body_has_support(p_grid, r_body);
	if (has_rest_support && speed < settle_speed && std::abs(r_body.angular_velocity) < settle_angular_speed) {
		if (has_overlap_blocking_sleep(p_grid, r_body)) {
			// A sampled narrow phase can leave two pixel chunks slightly
			// interpenetrating. Do not freeze that configuration; keep it awake
			// until the full-boundary overlap test says it is separated.
			r_body.still_time = 0.0f;
		} else {
			r_body.still_time += fixed_step_seconds;
		}
	} else {
		r_body.still_time = 0.0f;
	}
	if (r_body.still_time >= sleep_after_seconds) {
		r_body.vx = 0.0f;
		r_body.vy = 0.0f;
		r_body.angular_velocity = 0.0f;
		if (!r_body.sleeping) {
			r_body.sleeping = true;
			sleeping_buckets_dirty = true;
		}
		r_body.still_time = sleep_after_seconds;
	}
}

bool RigidBodySolver::body_has_support(const WorldGrid &p_grid, const RigidBody &p_body) const {
	const std::vector<int32_t> &samples = p_body.boundary_indices;
	if (samples.empty()) {
		return false;
	}
	auto is_support_cell = [&](int32_t sx, int32_t sy) -> bool {
		if (static_solid_or_boundary(p_grid, sx, sy)) {
			return true;
		}
		if (!p_grid.in_bounds(sx, sy)) {
			return false;
		}
		const int32_t support_i = p_grid.cell_index(sx, sy);
		const int32_t support_body_id = p_grid.rigid_body_id[support_i];
		if (support_body_id == 0 || support_body_id == p_body.id) {
			return false;
		}
		return find_body_by_id(support_body_id) != nullptr;
	};
	for (int32_t sample_index : samples) {
		if (sample_index < 0 || sample_index >= static_cast<int32_t>(p_body.cells.size())) {
			continue;
		}
		float wx = 0.0f;
		float wy = 0.0f;
		cell_world_position(p_body, p_body.cells[sample_index], wx, wy);
		const int32_t x = static_cast<int32_t>(std::floor(wx));
		const int32_t y = static_cast<int32_t>(std::floor(wy));
		if (is_support_cell(x, y + 1) ||
				is_support_cell(x - 1, y + 1) ||
				is_support_cell(x + 1, y + 1)) {
			return true;
		}
	}
	return false;
}

void RigidBodySolver::wake_sleeping_bodies_without_support_or_with_velocity(const WorldGrid &p_grid) {
	for (RigidBody &body : bodies) {
		if (!body.active || !body.sleeping) {
			continue;
		}
		const float speed = std::sqrt(body.vx * body.vx + body.vy * body.vy);
		if (speed > settle_speed || std::abs(body.angular_velocity) > settle_angular_speed || !body_has_support(p_grid, body)) {
			body.sleeping = false;
			body.still_time = 0.0f;
			sleeping_buckets_dirty = true;
		}
	}
}

void RigidBodySolver::bake_body_to_grid(WorldGrid &p_grid, RigidBody &r_body) {
	for (const BodyCell &c : r_body.cells) {
		int32_t x = 0;
		int32_t y = 0;
		raster_cell(r_body, c, x, y);
		if (!p_grid.in_bounds(x, y)) {
			continue;
		}
		if (c.material == MATERIAL_GLASS) {
			p_grid.make_glass(x, y);
		} else {
			p_grid.make_rock(x, y);
		}
		mark_dirty_cell(p_grid, x, y);
	}
	r_body.active = false;
}

void RigidBodySolver::populate_dynamic_cells(WorldGrid &p_grid) {
	p_grid.clear_rigid_fields();
	current_occupied_cells.clear();
	std::vector<CoveredCell> covered;
	for (const RigidBody &body : bodies) {
		if (!body.active) {
			continue;
		}
		collect_body_covered_cells(body, &p_grid, nullptr, 0.52f, covered);
		for (const CoveredCell &c : covered) {
			const int32_t x = c.x;
			const int32_t y = c.y;
			const float rx = (static_cast<float>(x) + 0.5f) - body.x;
			const float ry = (static_cast<float>(y) + 0.5f) - body.y;
			const float cvx = body.vx - body.angular_velocity * ry;
			const float cvy = body.vy + body.angular_velocity * rx;
			const int32_t i = p_grid.cell_index(x, y);
			p_grid.rigid_body_id[i] = body.id;
			p_grid.rigid_velocity_x[i] = cvx;
			p_grid.rigid_velocity_y[i] = cvy;
			const float speed2 = cvx * cvx + cvy * cvy;
			const bool is_boundary = c.source_index >= 0 &&
					std::binary_search(body.boundary_indices.begin(), body.boundary_indices.end(), c.source_index);
			current_occupied_cells.push_back({ x, y, body.id, cvx, cvy, !body.sleeping && (body.id == dragged_body || speed2 > 1.0e-5f), is_boundary });
		}
	}
}

void RigidBodySolver::step(WorldGrid &p_grid) {
	if (auto_process_dirty) {
		process_dirty(p_grid);
	}
	for (RigidBody &body : bodies) {
		body.had_contact = false;
	}
	for (RigidBody &body : bodies) {
		if (!body.active || body.id == dragged_body) {
			continue;
		}
		if (body.sleeping) {
			continue;
		}
		body.vy += gravity;
		body.x += body.vx;
		body.y += body.vy;
		body.angle += body.angular_velocity;
		body.vx *= linear_damping;
		body.vy *= linear_damping;
		body.angular_velocity *= angular_damping;
		update_body_aabb(body);
	}

	broad_phase_counter--;
	if (broad_phase_counter <= 0) {
		rebuild_broad_phase_pairs(p_grid);
		broad_phase_counter = std::max(1, broad_phase_interval_steps);
	}

	resolve_dynamic_body_contacts(p_grid);

	for (RigidBody &body : bodies) {
		if (!body.active || body.sleeping || body.id == dragged_body) {
			continue;
		}
		update_sleep_state(p_grid, body, body.had_contact);
	}
	const size_t before_count = bodies.size();
	bodies.erase(std::remove_if(bodies.begin(), bodies.end(), [](const RigidBody &b) { return !b.active; }), bodies.end());
	if (bodies.size() != before_count) {
		mark_body_index_dirty();
	}
	populate_dynamic_cells(p_grid);
}

void RigidBodySolver::destroy_circle(WorldGrid &p_grid, float p_x, float p_y, float p_radius) {
	const float r2 = p_radius * p_radius;
	for (RigidBody &body : bodies) {
		if (!body.active) {
			continue;
		}
		if (p_x < body.min_x - p_radius || p_x > body.max_x + p_radius || p_y < body.min_y - p_radius || p_y > body.max_y + p_radius) {
			continue;
		}
		std::vector<BodyCell> kept;
		kept.reserve(body.cells.size());
		bool removed_any = false;
		const float ca = std::cos(body.angle);
		const float sa = std::sin(body.angle);
		for (const BodyCell &c : body.cells) {
			const float wx = body.x + ca * c.lx - sa * c.ly;
			const float wy = body.y + sa * c.lx + ca * c.ly;
			const float dx = wx - p_x;
			const float dy = wy - p_y;
			if (dx * dx + dy * dy <= r2) {
				removed_any = true;
				continue;
			}
			kept.push_back(c);
		}
		if (!removed_any) {
			continue;
		}
		body.cells.swap(kept);
		if (body.cells.empty()) {
			body.active = false;
		} else {
			recompute_mass_properties(body, true);
			rebuild_boundary_samples(body);
			update_body_aabb(body);
			if (body.sleeping) {
				sleeping_buckets_dirty = true;
			}
			body.sleeping = false;
			body.still_time = 0.0f;
			body.vx *= 0.7f;
			body.vy *= 0.7f;
			body.angular_velocity *= 0.8f;
		}
	}
	const size_t before_count = bodies.size();
	bodies.erase(std::remove_if(bodies.begin(), bodies.end(), [](const RigidBody &b) { return !b.active; }), bodies.end());
	if (bodies.size() != before_count) {
		mark_body_index_dirty();
	}
	mark_dirty_rect(p_grid,
			static_cast<int32_t>(std::floor(p_x - p_radius)) - 1,
			static_cast<int32_t>(std::floor(p_y - p_radius)) - 1,
			static_cast<int32_t>(std::ceil(p_x + p_radius)) + 1,
			static_cast<int32_t>(std::ceil(p_y + p_radius)) + 1);
}

void RigidBodySolver::spawn_collision_test(WorldGrid &p_grid) {
	clear();
	p_grid.clear();

	// Static arena: a floor and two side walls.  These stay in WorldGrid and
	// are resolved by static contact impulses.
	const int32_t floor_y = p_grid.height - 18;
	for (int32_t x = 0; x < p_grid.width; x++) {
		for (int32_t y = floor_y; y < p_grid.height; y++) {
			p_grid.make_rock(x, y);
		}
	}
	for (int32_t y = 0; y < p_grid.height; y++) {
		for (int32_t x = 0; x < 5; x++) {
			p_grid.make_rock(x, y);
			p_grid.make_rock(p_grid.width - 1 - x, y);
		}
	}

	auto make_box_body = [&](float cx, float cy, float angle, float vx, float vy, float av, uint8_t mat) {
		RigidBody body;
		body.id = next_body_id++;
		body.x = cx;
		body.y = cy;
		body.angle = angle;
		body.vx = vx;
		body.vy = vy;
		body.angular_velocity = av;
		body.sleeping = false;
		body.active = true;
		for (int32_t y = -5; y <= 5; y++) {
			for (int32_t x = -11; x <= 11; x++) {
				// Slightly irregular chunks make angular collision visible.
				if ((x < -8 && y < -2) || (x > 8 && y > 2)) {
					continue;
				}
				BodyCell c;
				c.lx = static_cast<float>(x);
				c.ly = static_cast<float>(y);
				c.material = mat;
				body.cells.push_back(c);
			}
		}
		recompute_mass_properties(body, true);
		rebuild_boundary_samples(body);
		update_body_aabb(body);
		bodies.push_back(std::move(body));
		mark_body_index_dirty();
	};

	const float cy = static_cast<float>(p_grid.height) * 0.38f;
	make_box_body(static_cast<float>(p_grid.width) * 0.34f, cy, 0.20f, 0.95f, 0.02f, 0.045f, MATERIAL_ROCK);
	make_box_body(static_cast<float>(p_grid.width) * 0.66f, cy + 1.0f, -0.18f, -0.95f, 0.02f, -0.050f, MATERIAL_GLASS);
	populate_dynamic_cells(p_grid);
}


const std::vector<RigidBodySolver::RigidCellOccupancy> &RigidBodySolver::get_current_occupied_cells() const {
	return current_occupied_cells;
}

void RigidBodySolver::draw_overlay_rgba(const WorldGrid &p_grid, std::vector<uint8_t> &r_pixels) const {
	if (r_pixels.empty()) {
		return;
	}
	std::vector<CoveredCell> covered;
	for (const RigidBody &body : bodies) {
		if (!body.active) {
			continue;
		}
		collect_body_covered_cells(body, &p_grid, nullptr, 0.52f, covered);
		for (const CoveredCell &cell : covered) {
			if (cell.source_index < 0 || cell.source_index >= static_cast<int32_t>(body.cells.size())) {
				continue;
			}
			const BodyCell &source = body.cells[cell.source_index];
			const int32_t p = p_grid.cell_index(cell.x, cell.y) * 4;
			if (source.material == MATERIAL_GLASS) {
				r_pixels[p + 0] = 115;
				r_pixels[p + 1] = 170;
				r_pixels[p + 2] = 195;
			} else {
				const uint8_t shade = rock_shade(static_cast<int32_t>(std::llround(source.lx * 4.0f)), static_cast<int32_t>(std::llround(source.ly * 4.0f)), 70);
				r_pixels[p + 0] = shade;
				r_pixels[p + 1] = shade;
				r_pixels[p + 2] = static_cast<uint8_t>(std::min<int32_t>(255, shade + 10));
			}
			r_pixels[p + 3] = 255;
		}
	}
}

int32_t RigidBodySolver::find_body_at(float p_x, float p_y) const {
	const int32_t gx = static_cast<int32_t>(std::floor(p_x));
	const int32_t gy = static_cast<int32_t>(std::floor(p_y));
	for (int32_t bi = static_cast<int32_t>(bodies.size()) - 1; bi >= 0; bi--) {
		const RigidBody &body = bodies[bi];
		if (!body.active || gx < body.min_x - 1 || gx > body.max_x + 1 || gy < body.min_y - 1 || gy > body.max_y + 1) {
			continue;
		}
		std::vector<CoveredCell> covered;
		collect_body_covered_cells(body, nullptr, nullptr, 0.52f, covered);
		for (const CoveredCell &c : covered) {
			if (c.x == gx && c.y == gy) {
				return body.id;
			}
		}
	}
	return -1;
}

bool RigidBodySolver::start_drag(float p_x, float p_y) {
	dragged_body = find_body_at(p_x, p_y);
	if (dragged_body < 0) {
		return false;
	}
	for (RigidBody &body : bodies) {
		if (body.id != dragged_body) {
			continue;
		}
		drag_offset_x = body.x - p_x;
		drag_offset_y = body.y - p_y;
		drag_start_mouse_angle = std::atan2(p_y - body.y, p_x - body.x);
		drag_start_body_angle = body.angle;
		body.vx = 0.0f;
		body.vy = 0.0f;
		body.angular_velocity = 0.0f;
		if (body.sleeping) {
			sleeping_buckets_dirty = true;
		}
		body.sleeping = false;
		body.still_time = 0.0f;
		return true;
	}
	return false;
}

void RigidBodySolver::update_drag(float p_x, float p_y, bool p_rotate) {
	if (dragged_body < 0) {
		return;
	}
	for (RigidBody &body : bodies) {
		if (body.id != dragged_body) {
			continue;
		}
		if (p_rotate) {
			const float a = std::atan2(p_y - body.y, p_x - body.x);
			body.angle = drag_start_body_angle + (a - drag_start_mouse_angle);
		} else {
			body.x = p_x + drag_offset_x;
			body.y = p_y + drag_offset_y;
		}
		body.vx = 0.0f;
		body.vy = 0.0f;
		body.angular_velocity = 0.0f;
		if (body.sleeping) {
			sleeping_buckets_dirty = true;
		}
		body.sleeping = false;
		body.still_time = 0.0f;
		update_body_aabb(body);
		return;
	}
}

void RigidBodySolver::end_drag() {
	dragged_body = -1;
}

int32_t RigidBodySolver::get_body_count() const {
	return static_cast<int32_t>(bodies.size());
}

int32_t RigidBodySolver::get_awake_body_count() const {
	int32_t count = 0;
	for (const RigidBody &body : bodies) {
		if (body.active && !body.sleeping) {
			count++;
		}
	}
	return count;
}

int32_t RigidBodySolver::get_sleeping_body_count() const {
	int32_t count = 0;
	for (const RigidBody &body : bodies) {
		if (body.active && body.sleeping) {
			count++;
		}
	}
	return count;
}


