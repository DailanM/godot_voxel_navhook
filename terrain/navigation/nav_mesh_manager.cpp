#include "nav_mesh_manager.h"

#ifdef VOXEL_ENABLE_NAVIGATION

#include "nav_mesh_build_task.h"
#include "../../engine/voxel_engine.h"
#include "../../util/godot/classes/engine.h"
#include "../../util/io/log.h"
#include "../../util/math/conv.h"
#include "../../util/string/format.h"
#include "servers/navigation_3d/navigation_server_3d.h"

namespace zylann::voxel {

// All 26 neighbors in the 3x3x3 cube around the center chunk.
// Required so that Y-padded heightfield bounds have geometry coverage
// from all directions — without full coverage, erosion eats into the
// walkable area at chunk edges/corners where geometry is missing.
static const Vector3i neighbor_offsets[26] = {
	// 6 face neighbors
	Vector3i(-1, 0, 0), Vector3i(1, 0, 0),
	Vector3i(0, -1, 0), Vector3i(0, 1, 0),
	Vector3i(0, 0, -1), Vector3i(0, 0, 1),
	// 12 edge neighbors
	Vector3i(-1, -1, 0), Vector3i(-1, 1, 0),
	Vector3i(1, -1, 0), Vector3i(1, 1, 0),
	Vector3i(-1, 0, -1), Vector3i(-1, 0, 1),
	Vector3i(1, 0, -1), Vector3i(1, 0, 1),
	Vector3i(0, -1, -1), Vector3i(0, -1, 1),
	Vector3i(0, 1, -1), Vector3i(0, 1, 1),
	// 8 corner neighbors
	Vector3i(-1, -1, -1), Vector3i(-1, -1, 1),
	Vector3i(-1, 1, -1), Vector3i(-1, 1, 1),
	Vector3i(1, -1, -1), Vector3i(1, -1, 1),
	Vector3i(1, 1, -1), Vector3i(1, 1, 1),
};
static const int NEIGHBOR_COUNT = 26;

// --- Worker thread ---

void NavMeshManager::on_mesh_built(NavChunkData &&data) {
	MutexLock lock(_cache_mutex);

	Vector3i chunk_pos = data.chunk_position;

	// Cache/replace data, bump generation counter
	NavChunkEntry &entry = _chunk_cache[chunk_pos];
	entry.data = std::move(data);
	entry.generation++;

	ZN_PRINT_VERBOSE(format("NavMeshManager::on_mesh_built() chunk=({},{},{}) verts={} indices={}",
			chunk_pos.x, chunk_pos.y, chunk_pos.z,
			entry.data.positions.size(), entry.data.indices.size()));

	// Check this chunk and all neighbors for readiness.
	// When chunk A arrives, it may complete the neighborhood for
	// itself AND for any neighbor that was waiting on A.
	_try_dispatch_nav_build(chunk_pos);
	for (int i = 0; i < NEIGHBOR_COUNT; i++) {
		_try_dispatch_nav_build(chunk_pos + neighbor_offsets[i]);
	}
}

// --- Main thread: obstacles ---

int NavMeshManager::add_obstacle(Ref<Mesh> collision_mesh, Transform3D transform, bool walkable) {
	// Stub — Phase 5
	return 0;
}

void NavMeshManager::remove_obstacle(int obstacle_id) {
	// Stub — Phase 5
}

void NavMeshManager::update_obstacle_transform(int obstacle_id, Transform3D new_transform) {
	// Stub — Phase 5
}

// --- Main thread: results ---

void NavMeshManager::apply_nav_result(Vector3i chunk_pos, Ref<NavigationMesh> nav_mesh, uint32_t build_generation) {
	// Skip stale results — a newer build was already applied
	auto *gen_ptr = _applied_generations.getptr(chunk_pos);
	if (gen_ptr != nullptr && build_generation <= *gen_ptr) {
		return;
	}
	_applied_generations[chunk_pos] = build_generation;

	RegionEntry &region = _regions[chunk_pos];
	region.nav_mesh = nav_mesh;

	if (register_with_server) {
		NavigationServer3D *ns = NavigationServer3D::get_singleton();
		if (!region.rid.is_valid()) {
			region.rid = ns->region_create();
			ns->region_set_map(region.rid, _nav_map_rid);
			ns->region_set_navigation_layers(region.rid, navigation_layers);
			ns->region_set_enabled(region.rid, true);
		}
		ns->region_set_transform(region.rid, _chunk_to_world(chunk_pos));
		ns->region_set_navigation_mesh(region.rid, nav_mesh);
	}
}

// --- Main thread: cleanup ---

void NavMeshManager::remove_chunk(Vector3i chunk_pos) {
	// Free region RID if it exists
	auto region_it = _regions.find(chunk_pos);
	if (region_it != _regions.end()) {
		NavigationServer3D::get_singleton()->free_rid(region_it->value.rid);
		_regions.remove(region_it);
	}

	_applied_generations.erase(chunk_pos);

	{
		MutexLock lock(_cache_mutex);
		_chunk_cache.erase(chunk_pos);
	}
}

void NavMeshManager::clear_all() {
	// Free all NavigationServer3D region RIDs
	for (const KeyValue<Vector3i, RegionEntry> &kv : _regions) {
		NavigationServer3D::get_singleton()->free_rid(kv.value.rid);
	}
	_regions.clear();
	_applied_generations.clear();

	{
		MutexLock lock(_cache_mutex);
		_chunk_cache.clear();
	}
}

// --- Debug ---

Array NavMeshManager::debug_get_nav_meshes() const {
	Array result;
	for (const KeyValue<Vector3i, RegionEntry> &kv : _regions) {
		if (kv.value.nav_mesh.is_valid()) {
			Array entry;
			entry.resize(2);
			entry[0] = _chunk_to_world(kv.key);
			entry[1] = kv.value.nav_mesh;
			result.push_back(entry);
		}
	}
	return result;
}

// --- Helpers ---

bool NavMeshManager::_are_neighbors_ready(Vector3i chunk_pos) const {
	// Must be called under _cache_mutex
	for (int i = 0; i < NEIGHBOR_COUNT; i++) {
		if (_chunk_cache.find(chunk_pos + neighbor_offsets[i]) == _chunk_cache.end()) {
			return false;
		}
	}
	return true;
}

bool NavMeshManager::_is_within_nav_range(Vector3i chunk_pos) const {
	// Must be called under _cache_mutex (caller holds it)

	// In the editor, all meshed chunks are nav candidates (no viewer needed)
	if (Engine::get_singleton()->is_editor_hint()) {
		return true;
	}

	Vector3 chunk_center = _chunk_center_world(chunk_pos);

	bool any_viewer = false;
	VoxelEngine::get_singleton().for_each_nav_viewer([&](const VoxelEngine::NavViewer &viewer) {
		if (any_viewer) {
			return;
		}
		float dist = chunk_center.distance_to(viewer.world_position);
		if (dist <= static_cast<float>(viewer.nav_distance)) {
			any_viewer = true;
		}
	});

	return any_viewer;
}

void NavMeshManager::_try_dispatch_nav_build(Vector3i chunk_pos) {
	// Must be called under _cache_mutex

	// Skip chunks outside nav range (but their data stays cached —
	// they may be needed as border geometry for in-range neighbors)
	if (!_is_within_nav_range(chunk_pos)) {
		return;
	}

	// Skip if this chunk itself has no data yet
	auto it = _chunk_cache.find(chunk_pos);
	if (it == _chunk_cache.end()) {
		return;
	}

	// Check that all neighbors have cached data
	if (!_are_neighbors_ready(chunk_pos)) {
		return;
	}

	ZN_PRINT_VERBOSE(format("NavMesh: dispatching build for chunk ({},{},{})",
			chunk_pos.x, chunk_pos.y, chunk_pos.z));
	_dispatch_nav_build(chunk_pos, it->value.generation);
}

void NavMeshManager::_dispatch_nav_build(Vector3i chunk_pos, uint32_t generation) {
	// Must be called under _cache_mutex

	auto *task = ZN_NEW(NavMeshBuildTask);
	task->chunk_position = chunk_pos;
	task->build_generation = generation;
	task->cfg = recast_config;
	task->filter_low_hanging = filter_low_hanging;
	task->filter_ledge_spans = filter_ledge_spans;
	task->filter_low_height_spans = filter_low_height_spans;
	task->use_erosion = use_erosion;
	task->use_detail_mesh = use_detail_mesh;

	// Copy this chunk's triangles
	task->chunk_triangles = _chunk_cache[chunk_pos].data;

	// Copy neighbor triangles
	for (int i = 0; i < NEIGHBOR_COUNT; i++) {
		auto it = _chunk_cache.find(chunk_pos + neighbor_offsets[i]);
		if (it != _chunk_cache.end()) {
			task->neighbor_triangles.push_back(it->value.data);
		}
	}

	// Snapshot overlapping obstacles (lock ordering: _cache_mutex already held -> _obstacle_mutex)
	AABB expanded_aabb = _chunk_aabb_world(chunk_pos).grow(recast_config.borderSize * recast_config.cs);
	{
		MutexLock obs_lock(_obstacle_mutex);
		for (const KeyValue<int, ObstacleEntry> &kv : _obstacles) {
			if (expanded_aabb.intersects(kv.value.world_aabb)) {
				task->obstacles.push_back(kv.value);
			}
		}
	}

	task->nav_mesh_manager = shared_from_this();
	VoxelEngine::get_singleton().push_async_task(task);
}

StdVector<Vector3i> NavMeshManager::_get_affected_chunks(const AABB &aabb) {
	// Stub — Phase 5
	return {};
}

Vector3 NavMeshManager::_chunk_center_world(Vector3i chunk_pos) const {
	return to_vec3(chunk_pos * mesh_block_size) + Vector3(mesh_block_size, mesh_block_size, mesh_block_size) * 0.5f;
}

AABB NavMeshManager::_chunk_aabb_world(Vector3i chunk_pos) const {
	return AABB(
			to_vec3(chunk_pos * mesh_block_size),
			Vector3(mesh_block_size, mesh_block_size, mesh_block_size));
}

Transform3D NavMeshManager::_chunk_to_world(Vector3i chunk_pos) const {
	return Transform3D(Basis(), to_vec3(chunk_pos * mesh_block_size));
}

} // namespace zylann::voxel

#endif // VOXEL_ENABLE_NAVIGATION
