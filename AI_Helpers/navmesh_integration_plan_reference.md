# Godot-Voxel Recast Navigation Integration — Plan v2

## Overview

This plan describes how to generate navigation meshes for smooth (Transvoxel) voxel terrain by feeding the triangle mesh already produced by the godot-voxel mesher into Recast's pipeline, then registering the result with Godot's NavigationServer3D.

**Scope:** This plan targets `VoxelTerrain` (fixed LOD) only. `VoxelLodTerrain` support is deferred to future work.

The core architectural decision is **lightly-coupled post-mesher dispatch**: inside `MeshBlockTask::build_mesh()` (on a worker thread), the collision triangle data is extracted and handed to a `NavMeshManager`. The manager caches per-chunk triangle data, tracks neighbor readiness, manages an obstacle registry, and dispatches `NavMeshBuildTask`s to the thread pool when a chunk and its neighbors are all available. Final NavigationServer3D calls happen on the main thread.

This approach:
- **Supports full threading** — nav data capture and Recast builds happen on worker threads; only NavigationServer3D calls touch the main thread.
- **Supports per-chunk incremental updates** — when terrain changes or obstacles move, only affected chunks are rebuilt.
- **Handles border overlap naturally** — chunks wait for neighbor data before building, so border geometry is always available.

---

## Developer-Facing Design

### Design Principles

Navigation follows the same monolithic pattern as the rest of the module. Chunks are internal data structures managed by the terrain node — not scene tree nodes. Just as rendering uses `DirectMeshInstance` (wrapping `RenderingServer` RIDs) and physics uses `DirectStaticBody` (wrapping `PhysicsServer3D` RIDs), navigation manages `NavigationServer3D` region RIDs directly inside `NavMeshManager`. No `NavigationRegion3D` nodes are created.

This works because `NavigationAgent3D` queries the server directly via `query_path()` using the World3D's default navigation map. Agents and regions find each other by being registered on the same map RID — agents have zero knowledge of region nodes. Standard Godot navigation workflows (adding a `NavigationAgent3D` to a character) work automatically once our regions are registered.

### Exposed Properties

Properties are flat on the terrain node (`VoxelTerrain`), organized into inspector groups via `ADD_GROUP`. This matches the existing pattern for collision settings (`generate_collisions`, `collision_layer`, `collision_mask`, `collision_margin`).

Navigation range is NOT a terrain property — it is controlled by `VoxelNavViewer` nodes (see [Navigation Viewer](#navigation-viewer) below).

```
ADD_GROUP("Navigation", "")
  generate_navigation         : bool    // Master toggle (like generate_collisions)
  navigation_layers           : int     // PROPERTY_HINT_LAYERS_3D_NAVIGATION

ADD_GROUP("Navigation Agent", "nav_agent_")
  nav_agent_radius            : float   // Default 0.4 → drives cfg.cs and cfg.walkableRadius
  nav_agent_height            : float   // Default 1.8 → drives cfg.walkableHeight
  nav_agent_max_climb         : float   // Default 0.3 → drives cfg.walkableClimb
  nav_agent_max_slope         : float   // Default 45.0 (degrees) → drives cfg.walkableSlopeAngle

ADD_GROUP("Navigation Advanced", "nav_")
  nav_cell_size               : float   // Default 0.2 (= agent_radius / 2)
  nav_cell_height             : float   // Default 0.1 (= cell_size / 2)
  nav_filter_low_hanging      : bool    // Default true
  nav_filter_ledge_spans      : bool    // Default true
  nav_filter_low_height       : bool    // Default true
  nav_region_min_size         : int     // Default 8 (rcConfig.minRegionArea)
  nav_region_merge_size       : int     // Default 20 (rcConfig.mergeRegionArea)
  nav_edge_max_length         : float   // Default 3.2 (= walkableRadius * 8 * cs)
  nav_edge_max_error          : float   // Default 1.3 (rcConfig.maxSimplificationError)
  nav_detail_sample_dist      : float   // Default 1.2 (= 6 * cell_size)
  nav_detail_sample_max_error : float   // Default 0.1 (= cell_height)
```

All advanced properties show their actual computed defaults. When agent parameters change, the advanced defaults are NOT automatically recomputed — they stay at whatever value the developer set (or the initial defaults). This keeps behavior transparent. A setter warning (`WARN_PRINT`) is emitted if a value is set to zero or negative, since no property uses zero as a meaningful value.

**Property derivation:** The "Navigation Agent" group contains the user-friendly parameters that describe the agent. The Recast `rcConfig` values are derived from all properties at build time:

```cpp
// Computed when properties change, stored on NavMeshManager::recast_config
cfg.cs = nav_cell_size;
cfg.ch = nav_cell_height;
cfg.walkableSlopeAngle = nav_agent_max_slope;
cfg.walkableHeight = (int)ceilf(nav_agent_height / cfg.ch);
cfg.walkableClimb  = (int)ceilf(nav_agent_max_climb / cfg.ch);
cfg.walkableRadius = (int)ceilf(nav_agent_radius / cfg.cs);
cfg.maxEdgeLen = (int)(nav_edge_max_length / cfg.cs);
cfg.maxSimplificationError = nav_edge_max_error;
cfg.minRegionArea = nav_region_min_size;
cfg.mergeRegionArea = nav_region_merge_size;
cfg.maxVertsPerPoly = 6;
cfg.detailSampleDist = nav_detail_sample_dist;
cfg.detailSampleMaxError = nav_detail_sample_max_error;
cfg.borderSize = cfg.walkableRadius + 3;
```

When any navigation property changes, the derived `rcConfig` is recomputed and stored on the `NavMeshManager`. Existing cached triangle data remains valid — only the Recast build parameters change, so all chunks are marked for rebuild.

### Obstacle API (Methods)

Obstacles are runtime-only and exposed as methods on the terrain node, forwarding to `NavMeshManager`:

```cpp
// In VoxelTerrain _bind_methods():
ClassDB::bind_method(D_METHOD("add_nav_obstacle", "collision_mesh", "transform", "walkable"),
                     &Self::add_nav_obstacle, DEFVAL(false));
ClassDB::bind_method(D_METHOD("remove_nav_obstacle", "obstacle_id"), &Self::remove_nav_obstacle);
ClassDB::bind_method(D_METHOD("update_nav_obstacle_transform", "obstacle_id", "transform"),
                     &Self::update_nav_obstacle_transform);
```

```gdscript
# GDScript usage:
var obstacle_id = terrain.add_nav_obstacle(rock_collision_mesh, rock_transform)
# Walkable obstacle (bridge, ramp) — slope-based walkability is evaluated:
var bridge_id = terrain.add_nav_obstacle(bridge_mesh, bridge_transform, true)
# Later, if the obstacle moves:
terrain.update_nav_obstacle_transform(obstacle_id, new_transform)
# Or remove it:
terrain.remove_nav_obstacle(obstacle_id)
```

No special obstacle node is needed. The terrain manages obstacle lifecycle internally.

### Developer Workflow

The minimal setup for a developer:

1. **Enable navigation** on the terrain node: set `generate_navigation = true`.
2. **Set agent parameters**: `nav_agent_radius`, `nav_agent_height`, `nav_agent_max_climb`, `nav_agent_max_slope`.
3. **Add a `VoxelNavViewer`** to the character (or any node near where navigation is needed). This controls where navmesh is generated.
4. **Add a `NavigationAgent3D`** to the character (standard Godot workflow).
5. Pathfinding works automatically — the terrain's internal regions are on the World3D's default navigation map, same map the agent queries.

Optional:
- Tune advanced Recast parameters if default quality isn't sufficient.
- Call `add_nav_obstacle()` for dynamic obstacles (trees, buildings, etc.).
- Adjust `VoxelNavViewer::nav_distance` to control how far around each viewer navmesh is generated.

```
Developer                    Terrain Node              NavMeshManager         NavigationServer3D
─────────                    ────────────              ──────────────         ──────────────────
Sets generate_navigation=true
Sets nav_agent_radius etc.
                             Creates NavMeshManager
                             Passes it via MeshingDependency

                             (chunks mesh on workers)
                                                       on_mesh_built()
                                                       → caches data
                                                       → dispatches builds
                                                       → apply_nav_result()
                                                         → region_create()
                                                         → region_set_map(world_nav_map)
                                                         → region_set_navigation_mesh()

NavigationAgent3D queries ──────────────────────────────────────────────► map_get_path()
  (uses World3D default map,                                              (finds our regions
   same map our regions use)                                               automatically)

add_nav_obstacle(mesh, xform)
                             → nav_mesh_manager
                               ->add_obstacle()        → marks dirty chunks
                                                       → dispatches rebuilds
```

### Navigation Viewer

Navigation range is controlled by a dedicated `VoxelNavViewer` node rather than a flat property on the terrain. This follows the `VoxelViewer` pattern (`terrain/voxel_viewer.h`) — developers place a `VoxelNavViewer` on any agent that needs navigation, and navmesh is generated within that viewer's range.

**Rationale:** A developer may want navigation around specific agents without the cost of generating navmesh everywhere the terrain is visible. Placing a `VoxelNavViewer` on an agent is a lightweight way to request navigation — it's distinct from `VoxelViewer` (which controls mesh generation) because you don't want the performance penalty of actually meshing around every agent that needs pathfinding.

**Properties:**
```
VoxelNavViewer (inherits Node3D)
  nav_distance       : unsigned int  // Default 64; radius in world units
  enabled_in_editor  : bool          // Default false
```

**Lifecycle** (mirrors `VoxelViewer` — see `terrain/voxel_viewer.cpp`):
- `NOTIFICATION_ENTER_TREE`: registers with `NavMeshManager`'s nav viewer registry via `NavMeshManager::add_nav_viewer()`. Skips registration if in editor and `!_enabled_in_editor`. If `_pending_deferred_unregistration` is set (node was reparented), reuses the existing ID instead of allocating a new one, and calls `sync_all_parameters()`.
- `NOTIFICATION_EXIT_TREE`: **deferred** unregistration — sets `_pending_deferred_unregistration = true` and schedules a `call_deferred` callback. The callback checks `is_inside_tree()` before actually removing the viewer. This prevents unnecessary navmesh churn when a node is reparented (EXIT_TREE → ENTER_TREE), exactly matching `VoxelViewer::unregister_deferred_callback()`.
- `NOTIFICATION_TRANSFORM_CHANGED`: updates position in registry via `NavMeshManager::update_nav_viewer_position(_viewer_id, get_global_transform().origin)`

**ID type:** `NavViewerID` — a `SlotMapKey<uint16_t, uint16_t>` matching the `ViewerID` pattern in `engine/ids.h`. Generational IDs prevent use-after-free when slots are reused.

**How `VoxelNavViewer` finds the `NavMeshManager`:** Uses the same **global registry pattern** as `VoxelViewer`. `VoxelNavViewer` registers directly with the `NavMeshManager` singleton (or a registry on `VoxelEngine`). Terrain nodes discover nav viewers by iterating the registry — no scene tree traversal. If no `NavMeshManager` is active (no terrain with `generate_navigation = true`), registration is a no-op.

**NavMeshManager integration:**
- `NavMeshManager` maintains a `SlotMap<NavViewerState>` with each viewer's position and distance (matching `VoxelEngine`'s `SlotMap<Viewer>` pattern)
- `add_nav_viewer()` / `remove_nav_viewer()` / `update_nav_viewer_position()` methods manage the registry
- A `Mutex _viewer_mutex` protects the registry (viewers update on main thread, `_is_within_nav_range()` reads on worker threads)
- `_is_within_nav_range(chunk_pos)` iterates all registered nav viewers and returns true if the chunk's world center is within `nav_distance` of **any** viewer
- When a `VoxelNavViewer` moves or its distance changes, the manager can dispatch builds for newly-in-range chunks and remove regions for chunks that left range

---

## File Organization

New files live under `terrain/navigation/`, mirroring the organization of other terrain subsystems (`terrain/instancing/`, `engine/detail_rendering/`):

```
terrain/navigation/
  nav_mesh_manager.h       — NavMeshManager class (coordinator)
  nav_mesh_manager.cpp
  nav_mesh_build_task.h    — NavMeshBuildTask (IThreadedTask, Recast pipeline)
  nav_mesh_build_task.cpp
  voxel_nav_viewer.h       — VoxelNavViewer node (controls nav generation range)
  voxel_nav_viewer.cpp
```

Add `"terrain/navigation/*.cpp"` to the source list in `common.py`, gated on a new build option `voxel_navigation`:

```python
# In common.py get_sources():
if env["voxel_navigation"]:
    sources += ["terrain/navigation/*.cpp"]
```

This keeps navigation code co-located, clearly scoped under `terrain/`, and automatically picked up by the build system.

---

## Architecture

```
Worker Thread                          NavMeshManager                    Main Thread
─────────────                          ──────────────                    ───────────
MeshBlockTask::run()
  ├─ gather_voxels_cpu()
  ├─ mesher->build()
  ├─ build_mesh() (Godot mesh)
  └─ Extract collision triangles ──►  on_mesh_built(NavChunkData)
                                        ├─ Cache triangle data + bump generation
                                        ├─ Check neighbor readiness + nav range
                                        └─ If ready ──► push NavMeshBuildTask
                                                              │
NavMeshBuildTask::run()  ◄────────────────────────────────────┘
  ├─ Read snapshot triangles (chunk + neighbors)
  ├─ Read snapshot obstacles
  ├─ Recast pipeline (rasterize → filter → compact →
  │    erode → regions → contours → polymesh → detail)
  └─ Produce NavigationMesh
        │
        └──► apply_result() ─────────────────────────────►  region_set_navigation_mesh()
               (called on main thread by                     (NavigationServer3D)
                VoxelEngine::process() dequeue;
                checks generation counter —
                skips if stale)

Game Code (main thread):
  NavMeshManager::add_obstacle() ──► mark affected chunks dirty ──► push NavMeshBuildTask
  NavMeshManager::remove_obstacle() / update_obstacle_transform()  (same pattern)

Terrain Node (main thread):
  On chunk unload ──► NavMeshManager::remove_chunk()
  On node removal ──► NavMeshManager::clear_all()
```

---

## NavMeshManager

Central coordinator for all navmesh state. Owned by the terrain node. A `shared_ptr<NavMeshManager>` is passed through `MeshingDependency` so worker threads can reach it.

```cpp
class NavMeshManager : public std::enable_shared_from_this<NavMeshManager> {
public:
    // --- Called from worker threads (thread-safe) ---

    // Called at end of MeshBlockTask::run() with the collision triangle data.
    // Caches the data and dispatches NavMeshBuildTasks for ready chunks.
    void on_mesh_built(NavChunkData &&data);

    // --- Called from main thread ---

    // Obstacle management. Marks overlapping chunks dirty and dispatches rebuilds.
    // walkable: if false, obstacle triangles are marked RC_NULL_AREA (blocks nav).
    //           if true, rcMarkWalkableTriangles() evaluates slope (bridges, ramps).
    int add_obstacle(Ref<Mesh> collision_mesh, Transform3D transform, bool walkable = false);
    void remove_obstacle(int obstacle_id);
    void update_obstacle_transform(int obstacle_id, Transform3D new_transform);

    // Called from NavMeshBuildTask::apply_result().
    // Checks generation counter before applying to avoid stale results.
    void apply_nav_result(Vector3i chunk_pos, Ref<NavigationMesh> nav_mesh,
                          uint32_t build_generation);

    // Cleanup when chunks leave nav range or terrain node is removed.
    void remove_chunk(Vector3i chunk_pos);
    void clear_all();

    // Cancellation — checked by NavMeshBuildTask::is_cancelled().
    bool valid = true;

    // --- Configuration (derived from terrain node properties) ---
    // Recomputed by the terrain node when any nav property changes,
    // then stored here. See "Developer-Facing Design > Property derivation".
    rcConfig recast_config;         // Derived Recast build parameters
    uint32_t navigation_layers = 1;
    bool filter_low_hanging = true;
    bool filter_ledge_spans = true;
    bool filter_low_height_spans = true;

    // Chunk geometry info (set by terrain node on manager creation)
    int mesh_block_size = 16;      // In voxels (from VoxelTerrain::get_mesh_block_size())

    // Reference to the World3D navigation map (set by terrain node on enter_tree)
    RID _nav_map_rid;

    // --- Navigation viewer registry ---
    // VoxelNavViewer nodes register/unregister here (main thread).
    // _is_within_nav_range() reads this under _viewer_mutex (worker threads).
    // Uses SlotMap with generational IDs matching the VoxelViewer pattern.
    struct NavViewerState {
        Vector3 world_position;
        unsigned int nav_distance = 64;
    };
    NavViewerID add_nav_viewer();
    void remove_nav_viewer(NavViewerID viewer_id);
    void update_nav_viewer_position(NavViewerID viewer_id, Vector3 position);
    void update_nav_viewer_distance(NavViewerID viewer_id, unsigned int distance);

private:
    // --- Lock ordering: _cache_mutex → _obstacle_mutex → _viewer_mutex ---
    // All code paths MUST acquire locks in this order to prevent deadlocks.
    // See "Lock Ordering" section below.

    // Per-chunk cached triangle data. Written by worker threads after meshing,
    // read during dispatch (both under _cache_mutex).
    struct NavChunkEntry {
        NavChunkData data;
        uint32_t generation = 0;    // Bumped on each cache update
    };
    Mutex _cache_mutex;
    HashMap<Vector3i, NavChunkEntry> _chunk_cache;

    // Obstacle registry. Modified on main thread, read (under lock) by dispatch.
    struct ObstacleEntry {
        int id;
        Ref<Mesh> collision_mesh;
        Transform3D transform;
        AABB world_aabb;
        bool walkable = false;  // false = RC_NULL_AREA; true = slope-evaluated walkability
    };
    Mutex _obstacle_mutex;
    HashMap<int, ObstacleEntry> _obstacles;
    int _next_obstacle_id = 0;

    // Navigation viewer positions. Modified on main thread, read by worker threads.
    // SlotMap provides generational IDs matching the VoxelViewer pattern (engine/ids.h).
    Mutex _viewer_mutex;
    SlotMap<NavViewerState, uint16_t, uint16_t> _nav_viewers;

    // Active NavigationServer3D regions. Main thread only.
    HashMap<Vector3i, RID> _region_rids;

    // Tracks the generation that was last applied per chunk (main thread only).
    // Used to skip stale NavMeshBuildTask results.
    HashMap<Vector3i, uint32_t> _applied_generations;

    // Helpers (all called under _cache_mutex)
    bool _are_neighbors_ready(Vector3i chunk_pos) const;
    bool _is_within_nav_range(Vector3i chunk_pos) const; // also acquires _viewer_mutex
    void _try_dispatch_nav_build(Vector3i chunk_pos);
    void _dispatch_nav_build(Vector3i chunk_pos, uint32_t generation); // also acquires _obstacle_mutex
    StdVector<Vector3i> _get_affected_chunks(const AABB &aabb);

    // Coordinate conversion helpers
    Vector3 _chunk_center_world(Vector3i chunk_pos) const;
    AABB _chunk_aabb_world(Vector3i chunk_pos) const;
    Transform3D _chunk_to_world(Vector3i chunk_pos) const;
};
```

### NavChunkData (passed between MeshBlockTask and NavMeshManager)

```cpp
struct NavChunkData {
    StdVector<Vector3f> positions;  // collision subset [0..submesh_vertex_end)
    StdVector<int32_t> indices;     // collision subset [0..submesh_index_end)
    Vector3i chunk_position;
    AABB world_aabb;
};
```

### Key Behaviors

**`on_mesh_built()`** (worker thread):

This is the central dispatch point. The entire check-and-dispatch sequence runs under a single mutex lock to ensure consistency. Multiple worker threads may call this concurrently — the mutex serializes them.

```cpp
void NavMeshManager::on_mesh_built(NavChunkData &&data) {
    MutexLock lock(_cache_mutex);

    Vector3i chunk_pos = data.chunk_position;

    // 1. Cache/replace data, bump generation counter
    NavChunkEntry &entry = _chunk_cache[chunk_pos];
    entry.data = std::move(data);
    entry.generation++;

    // 2. Check this chunk and all neighbors for readiness.
    //    When chunk A arrives, it may complete the neighborhood for
    //    itself AND for any neighbor that was waiting on A.
    _try_dispatch_nav_build(chunk_pos);
    for (Vector3i offset : neighbor_offsets) {
        _try_dispatch_nav_build(chunk_pos + offset);
    }
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

    // Check that all neighbors within borderSize distance have cached data
    if (!_are_neighbors_ready(chunk_pos)) {
        return;
    }

    _dispatch_nav_build(chunk_pos, it->second.generation);
}
```

**Note on cascading dispatch:** When chunk A's data arrives, `on_mesh_built()` checks up to 27 positions (itself + 26 neighbors). Each readiness check looks at up to 26 more neighbors in `_chunk_cache` — all HashMap lookups, all under the same lock. This is fast. A single `on_mesh_built()` call can dispatch up to 27 NavMeshBuildTasks in burst, which is desirable — it means you get rapid progress when a neighborhood completes during initial load.

**`add_obstacle()` / `remove_obstacle()`** (main thread):
1. Update the obstacle registry.
2. Compute which chunks overlap the obstacle's AABB.
3. Mark those chunks dirty and dispatch `NavMeshBuildTask`s for any that have cached triangle data and ready neighbors.

**`apply_nav_result()`** (main thread, called from `apply_result()` which is dequeued by `VoxelEngine::process()`):
1. Compare `build_generation` against `_applied_generations[chunk_pos]`. If the build is for an older generation than what's already applied, skip it (a newer build is in flight or already applied).
2. If no region RID exists for this chunk, create one via `NavigationServer3D::region_create()`.
3. Call `region_set_navigation_mesh()` with the new data.
4. Update `_applied_generations[chunk_pos] = build_generation`.

**`remove_chunk()`** (main thread):
1. Free the region RID via `NavigationServer3D::free()`.
2. Remove from `_region_rids` and `_applied_generations`.
3. Remove from `_chunk_cache` (under `_cache_mutex`).

### Lock Ordering

The NavMeshManager uses three mutexes. To prevent deadlocks, all code paths **must** acquire locks in this fixed order:

```
_cache_mutex → _obstacle_mutex → _viewer_mutex
```

**Why this matters:** Two code paths touch multiple mutexes:
1. `on_mesh_built()` (worker thread): acquires `_cache_mutex`, then `_dispatch_nav_build()` acquires `_obstacle_mutex` for the obstacle snapshot, and `_is_within_nav_range()` acquires `_viewer_mutex`.
2. `add_obstacle()` (main thread): needs to update the obstacle registry AND dispatch nav builds (which requires `_cache_mutex` for the chunk cache).

If `add_obstacle()` locked `_obstacle_mutex` first and then `_cache_mutex`, it would deadlock with a concurrent `on_mesh_built()` that holds `_cache_mutex` and wants `_obstacle_mutex`. By always acquiring `_cache_mutex` first, both paths use the same order:

```cpp
// add_obstacle() — correct lock order
void NavMeshManager::add_obstacle(...) {
    MutexLock cache_lock(_cache_mutex);
    {
        MutexLock obs_lock(_obstacle_mutex);
        // update obstacle registry
    }
    // dispatch nav builds for affected chunks (cache_mutex still held)
}
```

### Chunk Coordinate Helpers

`NavMeshManager` stores `mesh_block_size` (set by the terrain node at creation time from `VoxelTerrain::get_mesh_block_size()`). This enables chunk-to-world coordinate conversions:

```cpp
// mesh_block_size is in voxels (typically 16 or 32).
// In godot-voxel, 1 voxel = 1 world unit (no scale factor).

Vector3 NavMeshManager::_chunk_center_world(Vector3i chunk_pos) const {
    return to_vec3(chunk_pos * mesh_block_size) + Vector3(mesh_block_size, mesh_block_size, mesh_block_size) * 0.5f;
}

AABB NavMeshManager::_chunk_aabb_world(Vector3i chunk_pos) const {
    return AABB(to_vec3(chunk_pos * mesh_block_size), Vector3(mesh_block_size, mesh_block_size, mesh_block_size));
}

Transform3D NavMeshManager::_chunk_to_world(Vector3i chunk_pos) const {
    return Transform3D(Basis(), to_vec3(chunk_pos * mesh_block_size));
}
```

These follow the same pattern as `VoxelMeshBlockVT` (`terrain/fixed_lod/voxel_mesh_block_vt.h:35`), which computes `_position_in_voxels = bpos * size` and uses it as a translation in `Transform3D`.

---

## MeshingDependency Modification

`MeshingDependency` gains a nullable `nav_mesh_manager` field (gated on `#ifdef VOXEL_ENABLE_NAVIGATION`). This is the mechanism by which worker threads access the NavMeshManager:

```cpp
struct MeshingDependency {
    Ref<VoxelMesher> mesher;
    Ref<VoxelGenerator> generator;
#ifdef VOXEL_ENABLE_NAVIGATION
    std::shared_ptr<NavMeshManager> nav_mesh_manager; // NEW — nullable for opt-in
#endif
    bool valid = true;

    static void reset(std::shared_ptr<MeshingDependency> &ref,
                      Ref<VoxelMesher> mesher,
                      Ref<VoxelGenerator> generator,
                      std::shared_ptr<NavMeshManager> nav_mesh_manager = nullptr) {
        if (ref != nullptr) {
            ref->valid = false;
        }
        ref = make_shared_instance<MeshingDependency>();
        ref->mesher = mesher;
        ref->generator = generator;
        ref->nav_mesh_manager = nav_mesh_manager;
        ref->valid = true;
    }
};
```

When `MeshingDependency::reset()` is called (mesher/generator changed), the old dependency is invalidated. In-flight `MeshBlockTask`s self-cancel via `is_cancelled()`. The NavMeshManager itself remains valid — only the meshing pipeline is invalidated. New mesh tasks with the new dependency will produce fresh triangle data and feed it to the same NavMeshManager.

`VoxelTerrain` passes its `NavMeshManager` (if any) through `MeshingDependency::reset()` calls.

---

## MeshBlockTask Hook Point

Inside `MeshBlockTask::build_mesh()`, after the detail texture scheduling block (~line 579), a small block extracts collision triangle data and forwards it to the NavMeshManager. This placement matches the existing pattern where detail texture scheduling also lives inside `build_mesh()`. The nav_mesh_manager pointer is nullable — if null, no nav work happens, keeping the coupling optional.

**Why the Transvoxel thread-local cache instead of `collision_surface.positions`:** The Transvoxel mesher does NOT populate `CollisionSurface::positions` or `CollisionSurface::indices` directly. Instead, it sets `submesh_vertex_end` and `submesh_index_end` as bounds into the render mesh arrays (see `voxel_mesher_transvoxel.cpp:350`). The thread-local cache (`get_mesh_cache_from_current_thread()`) provides direct access to raw `StdVector<Vector3f>` vertices — avoiding Godot Variant array overhead. The bounds from `submesh_vertex_end/submesh_index_end` exclude LOD transition mesh vertices.

```cpp
// In MeshBlockTask::build_mesh(), after the detail texture scheduling block:
#ifdef VOXEL_ENABLE_NAVIGATION
if (meshing_dependency->nav_mesh_manager != nullptr && lod_index == 0) {
    // Explicit Transvoxel mesher check — do NOT rely on implicit mesher type.
    // Without this, the thread-local cache could contain stale data from a
    // previous task if a non-Transvoxel mesher is used with navigation enabled.
    Ref<VoxelMesherTransvoxel> nav_transvoxel_mesher;
    if (zylann::godot::try_get_as(mesher, nav_transvoxel_mesher)) {
        const VoxelMesher::Output::CollisionSurface &col = _surfaces_output.collision_surface;

        if (_surfaces_output.surfaces.size() > 0 && col.submesh_vertex_end > 0) {
            NavChunkData nav_data;
            nav_data.chunk_position = mesh_block_position;
            nav_data.world_aabb = AABB(
                to_vec3(mesh_block_position * mesh_block_size),
                to_vec3(mesh_block_size));

            // Use thread-local mesh cache for raw float data (no Variant overhead).
            // This follows the same pattern as RenderDetailTextureTask (mesh_block_task.cpp:539).
            const transvoxel::MeshArrays &mesh_arrays =
                VoxelMesherTransvoxel::get_mesh_cache_from_current_thread();

            int vert_end = col.submesh_vertex_end;
            int idx_end = col.submesh_index_end;

            nav_data.positions.assign(
                mesh_arrays.vertices.begin(),
                mesh_arrays.vertices.begin() + vert_end);
            nav_data.indices.assign(
                mesh_arrays.indices.begin(),
                mesh_arrays.indices.begin() + idx_end);

            meshing_dependency->nav_mesh_manager->on_mesh_built(std::move(nav_data));
        }
    }
}
#endif
```

**Notes:**
- Only LOD 0 chunks produce nav data (higher LODs are too coarse for accurate navigation).
- The `submesh_vertex_end` / `submesh_index_end` bounds exclude transition mesh vertices used for LOD stitching — these must NOT be included in nav generation.
- `get_mesh_cache_from_current_thread()` returns data that's only valid until the next `build()` call on that thread. We copy immediately via `assign()`, which is fine.
- In double-precision builds, the thread cache already uses `Vector3f` (32-bit), which is what Recast expects. No conversion needed.
- Gated on `VOXEL_ENABLE_NAVIGATION` for builds without the navigation module.
- The Transvoxel mesher check is **explicit** via `try_get_as()`, matching the detail texture code pattern at line 531. This prevents reading stale thread-local cache data if a non-Transvoxel mesher is used.
- **Vertex coordinate space:** The Transvoxel mesher outputs vertices in **chunk-local space** (origin at the chunk's corner, range 0 to `mesh_block_size`). These must be converted to world space before being used in the Recast pipeline, since chunks and their neighbors need to share a common coordinate system. The conversion is a simple inline offset: `world_vertex = local_vertex + chunk_position * mesh_block_size`. This follows the same convention used throughout godot-voxel (e.g., `VoxelMeshBlockVT` computes `_position_in_voxels = bpos * size` for its world-space translation). The implementer should apply this offset either at capture time (in this hook, before storing in `NavChunkData`) or at rasterization time (in `NavMeshBuildTask::run()`). Doing it at capture time is simpler since each chunk only needs its own offset.

---

## NavMeshBuildTask

An `IThreadedTask` that runs the full Recast pipeline for a single chunk.

```cpp
class NavMeshBuildTask : public IThreadedTask {
public:
    // Input (set before dispatch, immutable during run — all data is copied/snapshotted)
    Vector3i chunk_position;
    uint32_t build_generation;                           // from NavChunkEntry at dispatch time
    NavChunkData chunk_triangles;                        // this chunk (copied)
    StdVector<NavChunkData> neighbor_triangles;           // border overlap (copied)
    StdVector<NavMeshManager::ObstacleEntry> obstacles;  // snapshot from registry
    rcConfig cfg;
    bool filter_low_hanging;
    bool filter_ledge_spans;
    bool filter_low_height_spans;

    // Output
    Ref<NavigationMesh> result_nav_mesh;

    // Dependency for priority calculation and cancellation
    std::shared_ptr<NavMeshManager> nav_mesh_manager;
    PriorityDependency priority_dependency;

    void run(ThreadedTaskContext &ctx) override;
    void apply_result() override;
    TaskPriority get_priority() override;
    bool is_cancelled() override;
    const char *get_debug_name() const override { return "NavMeshBuild"; }
};
```

### Cancellation

```cpp
bool NavMeshBuildTask::is_cancelled() {
    return nav_mesh_manager == nullptr || !nav_mesh_manager->valid;
}
```

When the terrain node is removed or the NavMeshManager is invalidated, `valid` is set to `false`, causing all in-flight NavMeshBuildTasks to self-cancel. This mirrors the `MeshingDependency::valid` pattern used by `MeshBlockTask`.

### `run()` — Recast Pipeline

**Step 1: Setup heightfield**

```cpp
rcContext recast_ctx;

// Compute bounds from chunk world position, expanded by borderSize
// (chunk_position * chunk_size_world gives bmin; + chunk_size gives bmax)
// Expand by cfg.borderSize * cfg.cs on X/Z
rcCalcGridSize(cfg.bmin, cfg.bmax, cfg.cs, &cfg.width, &cfg.height);

rcHeightfield *hf = rcAllocHeightfield();
ERR_FAIL_NULL(hf);
if (!rcCreateHeightfield(&recast_ctx, *hf, cfg.width, cfg.height,
                    cfg.bmin, cfg.bmax, cfg.cs, cfg.ch)) {
    rcFreeHeightField(hf);
    return;
}
```

**Step 2: Rasterize terrain triangles**

Rasterize this chunk's triangles plus neighbor triangles that fall within the expanded bounds.

```cpp
auto rasterize = [&](const NavChunkData &data) {
    if (data.indices.size() == 0) return;
    int num_tris = data.indices.size() / 3;
    StdVector<unsigned char> tri_areas(num_tris, 0);
    rcMarkWalkableTriangles(&recast_ctx, cfg.walkableSlopeAngle,
        (const float *)data.positions.data(), data.positions.size(),
        data.indices.data(), num_tris, tri_areas.data());
    rcRasterizeTriangles(&recast_ctx,
        (const float *)data.positions.data(), data.positions.size(),
        data.indices.data(), tri_areas.data(), num_tris, *hf, cfg.walkableClimb);
};

rasterize(chunk_triangles);
for (const auto &neighbor : neighbor_triangles) {
    rasterize(neighbor);
}
```

**Step 3: Rasterize obstacles**

Obstacles use full mesh rasterization (not projected obstructions). This produces accurate clearance around complex 3D shapes.

```cpp
for (const auto &obs : obstacles) {
    // Extract and transform obstacle mesh vertices to world space
    // ...
    StdVector<unsigned char> obs_areas(obs_num_tris, 0);
    if (obs.walkable) {
        // Walkable obstacles (bridges, ramps): evaluate slope-based walkability
        rcMarkWalkableTriangles(&recast_ctx, cfg.walkableSlopeAngle,
            obs_verts, obs_num_verts, obs_tris, obs_num_tris, obs_areas.data());
    } else {
        // Non-walkable obstacles: mark all triangles as RC_NULL_AREA (blocks nav)
        memset(obs_areas.data(), RC_NULL_AREA, obs_areas.size());
    }
    rcRasterizeTriangles(&recast_ctx,
        obs_verts, obs_num_verts, obs_tris, obs_areas.data(),
        obs_num_tris, *hf, cfg.walkableClimb);
}
```

**Step 4: Filtering (configurable)**

```cpp
if (filter_low_hanging) {
    rcFilterLowHangingWalkableObstacles(&recast_ctx, cfg.walkableClimb, *hf);
}
if (filter_ledge_spans) {
    // Must run AFTER LowHangingWalkableObstacles (ordering matters per Recast.h)
    rcFilterLedgeSpans(&recast_ctx, cfg.walkableHeight, cfg.walkableClimb, *hf);
}
if (filter_low_height_spans) {
    rcFilterWalkableLowHeightSpans(&recast_ctx, cfg.walkableHeight, *hf);
}
```

**Step 5: Compact, erode, build regions**

```cpp
rcCompactHeightfield *chf = rcAllocCompactHeightfield();
ERR_FAIL_NULL(chf);
rcBuildCompactHeightfield(&recast_ctx, cfg.walkableHeight,
                          cfg.walkableClimb, *hf, *chf);
rcFreeHeightField(hf);

rcErodeWalkableArea(&recast_ctx, cfg.walkableRadius, *chf);
rcBuildDistanceField(&recast_ctx, *chf);
// Monotone partitioning — faster, good for runtime/streaming use
rcBuildRegionsMonotone(&recast_ctx, *chf, cfg.borderSize,
                       cfg.minRegionArea, cfg.mergeRegionArea);
```

**Step 6: Contours, polymesh, detail mesh**

```cpp
rcContourSet *cset = rcAllocContourSet();
ERR_FAIL_NULL(cset);
rcBuildContours(&recast_ctx, *chf, cfg.maxSimplificationError,
                cfg.maxEdgeLen, *cset);

rcPolyMesh *pmesh = rcAllocPolyMesh();
ERR_FAIL_NULL(pmesh);
rcBuildPolyMesh(&recast_ctx, *cset, cfg.maxVertsPerPoly, *pmesh);

rcPolyMeshDetail *dmesh = rcAllocPolyMeshDetail();
ERR_FAIL_NULL(dmesh);
rcBuildPolyMeshDetail(&recast_ctx, *pmesh, *chf,
                      cfg.detailSampleDist, cfg.detailSampleMaxError, *dmesh);

rcFreeCompactHeightfield(chf);
rcFreeContourSet(cset);
```

**Step 7: Convert rcPolyMeshDetail to NavigationMesh**

We use `rcPolyMeshDetail` (not `rcPolyMesh`) to match Godot's own baker and get float-precision vertices with higher vertical accuracy.

```cpp
result_nav_mesh.instantiate();

// Extract vertices — dmesh->verts are already float world-space coordinates
PackedVector3Array vertices;
vertices.resize(dmesh->nverts);
for (int i = 0; i < dmesh->nverts; i++) {
    const float *v = &dmesh->verts[i * 3];
    vertices.write[i] = Vector3(v[0], v[1], v[2]);
}

// Deduplicate vertices and build triangle polygons
// Note: detail mesh vertices include both shared (from pmesh) and
// interpolated vertices. Deduplication matches Godot's approach.
HashMap<Vector3, int> vertex_map;
PackedVector3Array unique_verts;
// ... (dedup loop)

// Extract triangles with REVERSED winding order (Godot convention)
for (int i = 0; i < dmesh->nmeshes; i++) {
    const unsigned int *m = &dmesh->meshes[i * 4];
    const unsigned int bverts = m[0];   // vertex start index
    const unsigned int btris = m[2];    // triangle start index
    const unsigned int ntris = m[3];    // triangle count

    for (unsigned int j = 0; j < ntris; j++) {
        const unsigned char *t = &dmesh->tris[(btris + j) * 4];
        PackedInt32Array polygon;
        polygon.resize(3);
        // IMPORTANT: Reversed winding — indices [0, 2, 1] not [0, 1, 2]
        // This matches Godot's nav_mesh_generator_3d.cpp:565-567
        polygon.write[0] = deduped_index[bverts + t[0]];
        polygon.write[1] = deduped_index[bverts + t[2]];  // swapped
        polygon.write[2] = deduped_index[bverts + t[1]];  // swapped
        result_nav_mesh->add_polygon(polygon);
    }
}
result_nav_mesh->set_vertices(unique_verts);

// Set cell size/height on the resource for debug visualization and edge matching
result_nav_mesh->set_cell_size(cfg.cs);
result_nav_mesh->set_cell_height(cfg.ch);

rcFreePolyMesh(pmesh);
rcFreePolyMeshDetail(dmesh);
```

### `apply_result()` — Main Thread

Called automatically on the main thread by `VoxelEngine::process()`, which dequeues completed tasks from the thread pool and calls `apply_result()` then deletes them. This is the standard `IThreadedTask` lifecycle — no special registration needed.

```cpp
void NavMeshBuildTask::apply_result() {
    if (nav_mesh_manager && nav_mesh_manager->valid && result_nav_mesh.is_valid()) {
        nav_mesh_manager->apply_nav_result(chunk_position, result_nav_mesh, build_generation);
    }
}
```

---

## Recast Configuration

The `rcConfig` is not set directly by developers. It is derived from the terrain node's inspector properties (see "Developer-Facing Design > Property derivation"). The terrain node recomputes `NavMeshManager::recast_config` whenever any navigation property changes.

Default values (before derivation from agent properties):

```cpp
rcConfig cfg = {};
cfg.cs = agent_radius / 2.0f;          // nav cell size (horizontal)
cfg.ch = cfg.cs / 2.0f;                // nav cell height (vertical, finer)
cfg.walkableSlopeAngle = 45.0f;
cfg.walkableHeight = (int)ceilf(agent_height / cfg.ch);
cfg.walkableClimb  = (int)ceilf(agent_max_climb / cfg.ch);
cfg.walkableRadius = (int)ceilf(agent_radius / cfg.cs);
cfg.maxEdgeLen = cfg.walkableRadius * 8;
cfg.maxSimplificationError = 1.3f;
cfg.minRegionArea = 8;
cfg.mergeRegionArea = 20;
cfg.maxVertsPerPoly = 6;
cfg.detailSampleDist = 6.0f * cfg.cs;
cfg.detailSampleMaxError = cfg.ch;
cfg.borderSize = cfg.walkableRadius + 3;
```

All advanced parameters display their actual defaults. There is no "0 = auto" convention — every value is explicit and meaningful. A `WARN_PRINT` is emitted if a value is set to zero or negative.

---

## Task Start Lifecycle

### When do NavMeshBuildTasks start?

Tasks are dispatched **as soon as they're ready** — there is no global barrier or deferred batch. The NavMeshManager is the sole coordinator that decides when a task can start.

### The readiness check flow

1. `MeshBlockTask::run()` completes on a worker thread, producing collision triangles.
2. It calls `NavMeshManager::on_mesh_built()` with the triangle data.
3. `on_mesh_built()` acquires `_cache_mutex` and:
   a. Caches the data and bumps the generation counter.
   b. Checks **this chunk** and **all its neighbors** for readiness.
   c. For each position checked: if it has cached data, is within nav range, and all *its* neighbors have cached data → dispatch a NavMeshBuildTask.
4. The mutex is held for the entire check-and-dispatch sequence (steps a-c). This ensures consistency — no other thread can modify the cache between the readiness check and the data snapshot for dispatch.

### Initial load behavior

On initial world load, chunks mesh concurrently in priority order (closest first). Each `on_mesh_built()` call checks its neighborhood:
- Early chunks: neighbors aren't cached yet → no dispatch, just cache.
- As more chunks complete: interior neighborhoods fill first → burst of nav builds for interior chunks.
- Edge chunks: wait until their boundary neighbors arrive.

There is an inherent latency between the first mesh appearing and the first navmesh being available — this is the cost of waiting for neighbors. For most use cases this is acceptable since agents aren't typically active before the terrain around them has loaded.

### Terrain edit behavior

When a chunk's voxels are edited, the mesher re-meshes it. The new `on_mesh_built()` call:
1. Replaces cached data and bumps generation.
2. Neighbors are already cached → immediate dispatch.
3. If the old build is still in flight, its `apply_result()` will apply stale data, but the new build will overwrite it. The generation counter prevents the stale result from overwriting a newer one if ordering is reversed.

### Obstacle edit behavior

Obstacle add/remove/move happens on the main thread:
1. The obstacle registry is updated.
2. Affected chunks are identified by AABB overlap.
3. For each affected chunk with cached data and ready neighbors → dispatch NavMeshBuildTask.
4. The generation counter ensures consistency if a mesh update races with an obstacle update.

---

## Border Geometry and Neighbor Readiness

### The Problem

Recast's `borderSize` expansion means each chunk's heightfield extends into neighboring chunks. Without neighbor geometry in the overlap zone, `rcErodeWalkableArea` incorrectly erodes edges at chunk boundaries, creating gaps.

### How This Is Solved

The NavMeshManager's per-chunk triangle cache is the key enabler:

1. When `on_mesh_built()` is called for chunk A, the triangle data is cached.
2. Before dispatching a NavMeshBuildTask for A, the manager checks that all neighbors within `borderSize * cs` distance also have cached data.
3. If not all neighbors are ready, nothing happens — the chunk simply waits.
4. When the final neighbor's mesh completes, its `on_mesh_built()` call triggers readiness checks for itself *and* all its neighbors, dispatching builds for everything that's now ready.

On initial load, all chunks mesh concurrently. Nav builds begin as soon as local neighborhoods complete — no global barrier needed. Interior chunks (surrounded by already-meshed chunks) build first; edge chunks build as their neighbors arrive.

### NavMeshBuildTask Data Assembly

When dispatching, the manager snapshots all relevant data under the cache mutex. The task receives **copies** and is fully self-contained:

```cpp
void NavMeshManager::_dispatch_nav_build(Vector3i chunk_pos, uint32_t generation) {
    // Must be called under _cache_mutex

    auto *task = ZN_NEW(NavMeshBuildTask);
    task->chunk_position = chunk_pos;
    task->build_generation = generation;
    task->cfg = recast_config;
    task->filter_low_hanging = filter_low_hanging;
    task->filter_ledge_spans = filter_ledge_spans;
    task->filter_low_height_spans = filter_low_height_spans;

    // Copy this chunk's triangles
    task->chunk_triangles = _chunk_cache[chunk_pos].data; // copy

    // Copy neighbor triangles that fall within the border overlap zone
    // (typically 6 axis neighbors, possibly up to 26 for corner overlap)
    for (Vector3i offset : neighbor_offsets) {
        auto it = _chunk_cache.find(chunk_pos + offset);
        if (it != _chunk_cache.end()) {
            task->neighbor_triangles.push_back(it->second.data); // copy
        }
    }

    // Snapshot overlapping obstacles (lock ordering: _cache_mutex already held → _obstacle_mutex)
    AABB expanded_aabb = _chunk_aabb_world(chunk_pos).grow(cfg.borderSize * cfg.cs);
    {
        MutexLock lock(_obstacle_mutex);
        for (const auto &[id, obs] : _obstacles) {
            if (expanded_aabb.intersects(obs.world_aabb)) {
                task->obstacles.push_back(obs); // copy
            }
        }
    }

    task->nav_mesh_manager = shared_from_this();
    // TODO: set up task->priority_dependency
    VoxelEngine::get_singleton().push_async_task(task);
}
```

### Consistency

All data passed to the build task is **copied at dispatch time**, so the task is fully self-contained. The cache and obstacle registry can continue to be updated by other threads without affecting in-flight builds.

If a chunk's data changes while a build is in flight:
1. The new `on_mesh_built()` bumps the generation counter and dispatches a new build.
2. When the old build's `apply_result()` runs, `apply_nav_result()` compares `build_generation` against `_applied_generations[chunk_pos]`.
3. If a newer result was already applied, the stale result is silently dropped.

**Concurrent updates to the same chunk:** If two worker threads call `on_mesh_built()` for the same chunk nearly simultaneously (possible if voxels are edited during initial load), the second call will bump the generation and dispatch a new build, while the first build is already in flight with the older data. The generation counter in `apply_nav_result()` handles this correctly — the stale result is dropped when the newer one arrives (or is already applied). No special handling is needed; the existing generation counter mechanism ensures correctness.

---

## Region Registration and Edge Stitching

### One Region Per Chunk

Each LOD 0 chunk within nav range maps to one `NavigationServer3D` region. Region RIDs are stored in `NavMeshManager::_region_rids`.

```cpp
void NavMeshManager::apply_nav_result(Vector3i chunk_pos, Ref<NavigationMesh> nav_mesh,
                                       uint32_t build_generation) {
    // Skip stale results — a newer build was already applied
    auto gen_it = _applied_generations.find(chunk_pos);
    if (gen_it != _applied_generations.end() && build_generation <= gen_it->second) {
        return;
    }
    _applied_generations[chunk_pos] = build_generation;

    RID &rid = _region_rids[chunk_pos];
    if (!rid.is_valid()) {
        rid = NavigationServer3D::get_singleton()->region_create();
        NavigationServer3D::get_singleton()->region_set_map(rid, _nav_map_rid);
        NavigationServer3D::get_singleton()->region_set_navigation_layers(rid, navigation_layers);
        NavigationServer3D::get_singleton()->region_set_enabled(rid, true);
    }
    NavigationServer3D::get_singleton()->region_set_transform(rid, _chunk_to_world(chunk_pos));
    NavigationServer3D::get_singleton()->region_set_navigation_mesh(rid, nav_mesh);
}
```

### Cell Size Synchronization

The navigation map's cell size must match Recast's:

```cpp
NavigationServer3D::get_singleton()->map_set_cell_size(_nav_map_rid, recast_config.cs);
NavigationServer3D::get_singleton()->map_set_cell_height(_nav_map_rid, recast_config.ch);
```

### Edge Stitching

NavigationServer3D auto-merges edges between adjacent regions when vertices match. For this to work, all chunks must use a consistent Recast coordinate grid — same `cs`, `ch`, and `bmin` values aligned to multiples of `cs`. This happens naturally when chunk positions are computed from integer coordinates multiplied by chunk world size.

If floating-point quantization causes mismatches, `map_set_edge_connection_margin()` can be set to `cfg.cs * 0.5` as a fallback.

---

## Chunk Lifecycle Integration

### Who calls `remove_chunk()`?

The terrain node is responsible for calling `NavMeshManager::remove_chunk()` when chunks leave the loaded area.

**VoxelTerrain:** In `voxel_terrain.cpp`, `unload_mesh_block()` (line 528) is called when a mesh block has no remaining viewers. This is the integration point — call `_nav_mesh_manager->remove_chunk(bpos)` here if the manager exists.

### Terrain node removal (`clear_all`)

When a `VoxelTerrain` node is destroyed:
1. `nav_mesh_manager->valid = false` — cancels all in-flight NavMeshBuildTasks.
2. `nav_mesh_manager->clear_all()` — frees all NavigationServer3D region RIDs and clears internal state.
3. This happens in `~VoxelTerrain()`, matching the existing pattern where `_meshing_dependency->valid = false` is set in the destructor (line 107).

---

## Obstacle Handling

Obstacles use **full mesh rasterization** — the actual obstacle geometry is rasterized into the heightfield alongside terrain. This produces accurate clearance around complex 3D shapes including overhangs, unlike Godot's projected obstruction approach which only handles vertical extrusions.

### API

Developers call methods on the terrain node (see "Developer-Facing Design > Obstacle API"). The terrain node forwards to `NavMeshManager`:

```cpp
// NavMeshManager internal API (called by terrain node methods):
int NavMeshManager::add_obstacle(Ref<Mesh> collision_mesh, Transform3D transform, bool walkable = false);
void NavMeshManager::remove_obstacle(int obstacle_id);
void NavMeshManager::update_obstacle_transform(int obstacle_id, Transform3D new_transform);
```

### Walkable vs Non-Walkable Obstacles

The `walkable` parameter on `add_obstacle()` controls how obstacle triangles are classified during rasterization:

- **`walkable = false` (default):** All obstacle triangles are marked `RC_NULL_AREA` — they block navigation entirely. Use for walls, rocks, trees.
- **`walkable = true`:** `rcMarkWalkableTriangles()` evaluates slope-based walkability on the obstacle mesh, allowing navigation on surfaces within `walkableSlopeAngle`. Use for bridges, ramps, rooftops, or any obstacle with walkable surfaces.

### Dirty Propagation

When an obstacle is added, removed, or moved:
1. Compute which chunks' expanded AABBs overlap the obstacle.
2. Mark those chunks dirty.
3. Dispatch `NavMeshBuildTask`s for dirty chunks that have cached triangle data and ready neighbors.

For obstacle moves, both the old and new positions' chunks are marked dirty.

---

## Build System

### Recast Header Access (No Recompilation Needed)

Godot compiles Recast as part of the `navigation_3d` module into `libmodule_navigation_3d.a`. All modules link against the same final binary, so **Recast symbols are globally available at link time**. The godot-voxel module only needs to add the include path — no Recast source recompilation is required.

In `SCsub` or `common.py`, add when `voxel_navigation` is enabled:

```python
# Add Recast include path (headers only — symbols come from libmodule_navigation_3d.a)
env_voxel.Prepend(CPPPATH=["#thirdparty/recastnavigation/Recast/Include"])
```

### Compile-Time Guards

All navigation code is gated on `VOXEL_ENABLE_NAVIGATION`, which is defined as a CPPDEFINE by `common.py` when the `voxel_navigation` build option is enabled. This is the module's own define — we do **not** use `MODULE_NAVIGATION_3D_ENABLED` from Godot's `modules/modules_enabled.gen.h`, because that header is not automatically included in module compilation units.

```cpp
#ifdef VOXEL_ENABLE_NAVIGATION
#include <Recast.h>
// ... nav code ...
#endif
```

To disable navigation compilation, build with `voxel_navigation=no`. This ensures the module compiles cleanly even if Recast headers are unavailable.

**Note:** The `voxel_navigation` option requires that Godot's `navigation_3d` module is enabled (provides Recast symbols at link time). If `navigation_3d` is disabled, `voxel_navigation` should also be disabled.

**Files requiring `#ifdef VOXEL_ENABLE_NAVIGATION` guards:**
- `terrain/navigation/*.h` and `*.cpp` — entire files (nav-only code)
- `engine/meshing_dependency.h` — the `nav_mesh_manager` field
- `meshers/mesh_block_task.cpp` — the MeshBlockTask hook block
- `terrain/fixed_lod/voxel_terrain.h` — nav member variables, `_nav_mesh_manager`, nav property getters/setters
- `terrain/fixed_lod/voxel_terrain.cpp` — nav `_bind_methods()` entries, lifecycle calls to NavMeshManager

---

## Interface Points Summary

| Interface | From | To | Data |
|-----------|------|----|------|
| **1. Mesh capture** | `MeshBlockTask::build_mesh()` | `NavMeshManager::on_mesh_built()` | Collision vertices/indices (from thread cache), chunk position, world AABB |
| **2. Nav build dispatch** | `NavMeshManager::on_mesh_built()` | `NavMeshBuildTask` (thread pool) | Copied triangles (chunk + neighbors), obstacle snapshot, rcConfig, generation |
| **3. Terrain rasterization** | `NavMeshBuildTask` | Recast | `float*` vertices, `int*` indices → `rcHeightfield` |
| **4. Obstacle rasterization** | `NavMeshBuildTask` | Recast | Obstacle mesh vertices/indices → same `rcHeightfield` |
| **5. Recast → NavigationMesh** | `NavMeshBuildTask` | Godot | `rcPolyMeshDetail` → `NavigationMesh` (reversed winding) |
| **6. Region registration** | `NavMeshBuildTask::apply_result()` | `NavigationServer3D` | `region_set_navigation_mesh()` on main thread (generation-checked) |
| **7. Obstacle management** | Game code | `NavMeshManager` | `add/remove/update_obstacle()` → dirty chunks → rebuild |
| **8. Chunk unload** | Terrain node | `NavMeshManager::remove_chunk()` | Free region RID, clear cache entry |
| **9. Dependency threading** | Terrain node | `MeshingDependency` | `shared_ptr<NavMeshManager>` propagated to worker threads |

---

## Future Improvements

These items are noted for later consideration and are not part of the initial implementation scope.

**VoxelLodTerrain Support:** The initial implementation targets `VoxelTerrain` only. Extending to `VoxelLodTerrain` requires adapting the lifecycle integration to its update task system, LOD-aware chunk management, and different MeshingDependency propagation path.

**NavMeshBuildTask Priority Tuning:** Currently `get_priority()` returns `TaskPriority::max()` (lowest priority), meaning nav tasks run after all mesh tasks. This can cause noticeable latency before navmesh appears. A proper priority scheme should consider viewer distance and navigation urgency.

**Performance — Projected Area Marking:** Expose a collection of convex polygon footprints passed to `rcMarkConvexPolyArea()` on the `rcCompactHeightfield`. This is a lighter-weight alternative to full mesh rasterization for simple vertical obstacles (walls, fences), and can be used alongside the full rasterization approach. Could speed up rebuilds for common obstacle types.

**LOD-Aware Navigation:** Currently only LOD 0 chunks produce navmesh data. For large worlds, coarser LODs could provide approximate long-range pathfinding while LOD 0 handles detailed local navigation. The nav range must be <= the LOD 0 loading range.

**Error Handling:** The Recast pipeline can fail at various stages (allocation failure, empty input, degenerate geometry from empty/near-empty chunks). Needs `ERR_FAIL_COND` guards throughout the build pipeline. Basic guards are included in the plan above; a comprehensive pass should be done during implementation.

**Debouncing Rapid Edits:** When voxels are edited rapidly (painting terrain), the mesher already re-meshes affected chunks. The nav builder should debounce to avoid excessive Recast rebuilds — e.g., a short delay after the last edit before dispatching a nav build, or cancelling in-flight builds for chunks that have been re-dirtied. The generation counter already prevents stale results from being applied, but the redundant Recast compute is still wasted work.

**Cleanup on Terrain Node Removal:** When a `VoxelTerrain` node is destroyed, all nav regions must be freed in bulk. `NavMeshManager::clear_all()` handles this. See "Chunk Lifecycle Integration" section above.

**Editor UI:** Debug visualization of nav regions (Godot already supports navmesh debug drawing). Property inspector for Recast parameters on the VoxelNavBuilder node. Gizmos showing nav range. Toggle to enable/disable nav building in-editor.

**Custom rcContext:** Default no-op `rcContext` is fine initially. Later, subclass to forward `doLog()` to Godot's print functions for profiling and debugging the Recast pipeline.
