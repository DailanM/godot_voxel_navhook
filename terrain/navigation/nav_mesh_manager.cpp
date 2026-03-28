#include "nav_mesh_manager.h"

#ifdef VOXEL_ENABLE_NAVIGATION

#include "../../util/io/log.h"
#include "../../util/math/conv.h"
#include "../../util/string/format.h"
#include "servers/navigation_3d/navigation_server_3d.h"

namespace zylann::voxel {

// --- Worker thread ---

void NavMeshManager::on_mesh_built(NavChunkData &&data) {
	MutexLock lock(_cache_mutex);

	Vector3i chunk_pos = data.chunk_position;

	// Cache/replace data, bump generation counter
	NavChunkEntry &entry = _chunk_cache[chunk_pos];
	entry.data = std::move(data);
	entry.generation++;

	// TODO: Remove debug logging after verification
	ZN_PRINT_VERBOSE(format("NavMeshManager::on_mesh_built() chunk=({},{},{}) verts={} indices={}",
			chunk_pos.x, chunk_pos.y, chunk_pos.z,
			entry.data.positions.size(), entry.data.indices.size()));

	// Dispatch logic will be added in Phase 3
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
	// Stub — Phase 4
}

// --- Main thread: cleanup ---

void NavMeshManager::remove_chunk(Vector3i chunk_pos) {
	// Free region RID if it exists
	auto region_it = _region_rids.find(chunk_pos);
	if (region_it != _region_rids.end()) {
		NavigationServer3D::get_singleton()->free_rid(region_it->value);
		_region_rids.remove(region_it);
	}

	_applied_generations.erase(chunk_pos);

	{
		MutexLock lock(_cache_mutex);
		_chunk_cache.erase(chunk_pos);
	}
}

void NavMeshManager::clear_all() {
	// Free all NavigationServer3D region RIDs
	for (const KeyValue<Vector3i, RID> &kv : _region_rids) {
		NavigationServer3D::get_singleton()->free_rid(kv.value);
	}
	_region_rids.clear();
	_applied_generations.clear();

	{
		MutexLock lock(_cache_mutex);
		_chunk_cache.clear();
	}
}

// --- Navigation viewer registry ---

NavViewerID NavMeshManager::add_nav_viewer() {
	MutexLock lock(_viewer_mutex);
	NavViewerState state;
	return _nav_viewers.add(state);
}

void NavMeshManager::remove_nav_viewer(NavViewerID viewer_id) {
	MutexLock lock(_viewer_mutex);
	_nav_viewers.remove(viewer_id);
}

void NavMeshManager::update_nav_viewer_position(NavViewerID viewer_id, Vector3 position) {
	MutexLock lock(_viewer_mutex);
	NavViewerState *state = _nav_viewers.try_get(viewer_id);
	if (state != nullptr) {
		state->world_position = position;
	}
}

void NavMeshManager::update_nav_viewer_distance(NavViewerID viewer_id, unsigned int distance) {
	MutexLock lock(_viewer_mutex);
	NavViewerState *state = _nav_viewers.try_get(viewer_id);
	if (state != nullptr) {
		state->nav_distance = distance;
	}
}

// --- Helpers ---

bool NavMeshManager::_are_neighbors_ready(Vector3i chunk_pos) const {
	// Stub — Phase 3
	return false;
}

bool NavMeshManager::_is_within_nav_range(Vector3i chunk_pos) const {
	// Stub — Phase 3
	return false;
}

void NavMeshManager::_try_dispatch_nav_build(Vector3i chunk_pos) {
	// Stub — Phase 3
}

void NavMeshManager::_dispatch_nav_build(Vector3i chunk_pos, uint32_t generation) {
	// Stub — Phase 3
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
