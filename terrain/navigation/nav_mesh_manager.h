#ifndef VOXEL_NAV_MESH_MANAGER_H
#define VOXEL_NAV_MESH_MANAGER_H

#ifdef VOXEL_ENABLE_NAVIGATION

#include "../../engine/ids.h"
#include "../../util/containers/slot_map.h"
#include "../../util/containers/std_vector.h"
#include "../../util/math/vector3f.h"
#include "../../util/memory/memory.h"
#include "../../util/thread/mutex.h"

#include "core/math/aabb.h"
#include "core/math/transform_3d.h"
#include "core/math/vector3i.h"
#include "core/templates/hash_map.h"
#include "core/variant/variant.h"
#include "scene/resources/navigation_mesh.h"
#include "scene/resources/mesh.h"

#include <Recast.h>

#include <memory>

namespace zylann::voxel {

struct NavChunkData {
	StdVector<Vector3f> positions;
	StdVector<int32_t> indices;
	Vector3i chunk_position;
	AABB world_aabb;
};

class NavMeshManager : public std::enable_shared_from_this<NavMeshManager> {
public:
	// --- Called from worker threads (thread-safe) ---

	void on_mesh_built(NavChunkData &&data);

	// --- Called from main thread ---

	int add_obstacle(Ref<Mesh> collision_mesh, Transform3D transform, bool walkable = false);
	void remove_obstacle(int obstacle_id);
	void update_obstacle_transform(int obstacle_id, Transform3D new_transform);

	void apply_nav_result(Vector3i chunk_pos, Ref<NavigationMesh> nav_mesh, PackedInt32Array poly_regions,
			uint32_t build_generation);

	void remove_chunk(Vector3i chunk_pos);
	void clear_all();

	// Debug: returns Array of [Transform3D, NavigationMesh, PackedInt32Array] triples.
	// The int32 array is a per-polygon region ID (one entry per NavigationMesh
	// polygon — each polygon is a single triangle).  Temporary diagnostic.
	Array debug_get_nav_meshes() const;

	// Cancellation
	bool valid = true;

	// --- Configuration (derived from terrain node properties) ---
	rcConfig recast_config = {};
	uint32_t navigation_layers = 1;
	bool filter_low_hanging = true;
	bool filter_ledge_spans = true;
	bool filter_low_height_spans = true;
	bool use_erosion = true;
	bool use_detail_mesh = true;
	bool register_with_server = true;
	int y_band_strip_radius = 2;
	Vector3i debug_chunk = Vector3i(INT32_MAX, INT32_MAX, INT32_MAX);

	// Chunk geometry info
	int mesh_block_size = 16;

	// Navigation map RID
	RID _nav_map_rid;

	// --- Obstacle entry (public for task snapshot) ---
	struct ObstacleEntry {
		int id = 0;
		Ref<Mesh> collision_mesh;
		Transform3D transform;
		AABB world_aabb;
		bool walkable = false;
	};

private:
	// --- Lock ordering: _cache_mutex -> _obstacle_mutex ---

	struct NavChunkEntry {
		NavChunkData data;
		uint32_t generation = 0;
	};
	Mutex _cache_mutex;
	HashMap<Vector3i, NavChunkEntry> _chunk_cache;

	Mutex _obstacle_mutex;
	HashMap<int, ObstacleEntry> _obstacles;
	int _next_obstacle_id = 0;

	// Active NavigationServer3D regions (main thread only)
	struct RegionEntry {
		RID rid;
		Ref<NavigationMesh> nav_mesh;
		// One Recast region ID per polygon of nav_mesh (debug viewer).
		PackedInt32Array poly_regions;
	};
	HashMap<Vector3i, RegionEntry> _regions;

	// Generation tracking for stale result detection (main thread only)
	HashMap<Vector3i, uint32_t> _applied_generations;

	// Helpers
	bool _are_neighbors_ready(Vector3i chunk_pos) const;
	bool _is_within_nav_range(Vector3i chunk_pos) const;
	void _try_dispatch_nav_build(Vector3i chunk_pos);
	void _dispatch_nav_build(Vector3i chunk_pos, uint32_t generation);
	StdVector<Vector3i> _get_affected_chunks(const AABB &aabb);

	Vector3 _chunk_center_world(Vector3i chunk_pos) const;
	AABB _chunk_aabb_world(Vector3i chunk_pos) const;
	Transform3D _chunk_to_world(Vector3i chunk_pos) const;
};

} // namespace zylann::voxel

#endif // VOXEL_ENABLE_NAVIGATION

#endif // VOXEL_NAV_MESH_MANAGER_H
