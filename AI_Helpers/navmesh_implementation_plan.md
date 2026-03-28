# Navmesh Integration — Implementation Plan

This document outlines the phased implementation process. Each phase is scoped to be completable in a single session. Code snippets and architectural details are in the [plan reference](navmesh_integration_plan_reference.md).

**Scope:** This plan targets `VoxelTerrain` (fixed LOD) only. `VoxelLodTerrain` support is deferred to future work — the initial implementation focuses on LOD 0 chunks to establish the system before extending to variable-LOD terrain.

---

## Phase 1: Skeleton Classes, Build System, and UI

**Goal:** Create the new files, add Recast include path, expose all properties in the inspector, and verify the build compiles. No navigation logic yet — just the framework.

### 1.1 Build system setup

- [x] Create `terrain/navigation/` directory
- [x] Add `"terrain/navigation/*.cpp"` to `common.py` source list, gated on a new `voxel_navigation` build option
- [x] Add `voxel_navigation` option to `config.py` (default enabled)
- [x] Add Recast include path to `SCsub`: `env_voxel.Prepend(CPPPATH=["#thirdparty/recastnavigation/Recast/Include"])`
- [x] Gate the include path on `voxel_navigation` being enabled
- [x] Verify build compiles with `#include <Recast.h>` in a new file

### 1.2 NavMeshManager skeleton

Central coordinator for navmesh state, caching, and dispatch. See [plan reference — NavMeshManager](navmesh_integration_plan_reference.md#navmeshmanager) for the full class definition.

- [x] Create `terrain/navigation/nav_mesh_manager.h`
  - Class declaration with all public/private members from the plan reference
  - All method signatures declared but not implemented (empty bodies or stubs)
  - `#ifdef VOXEL_ENABLE_NAVIGATION` guard around the whole file
  - `NavChunkData` struct definition
  - `NavChunkEntry` struct (with generation counter)
  - `ObstacleEntry` struct
  - All member variables: `_chunk_cache`, `_obstacles`, `_region_rids`, `_applied_generations`, mutexes, config fields
- [x] Create `terrain/navigation/nav_mesh_manager.cpp`
  - Stub implementations for all methods (just return defaults / do nothing)
  - `on_mesh_built()`: empty (will be implemented in Phase 3)
  - `add_obstacle()` / `remove_obstacle()` / `update_obstacle_transform()`: empty stubs returning 0 / doing nothing
  - `apply_nav_result()`: empty
  - `remove_chunk()`: empty
  - `clear_all()`: empty

### 1.3 NavMeshBuildTask skeleton

Threaded task that runs the Recast pipeline for a single chunk. See [plan reference — NavMeshBuildTask](navmesh_integration_plan_reference.md#navmeshbuildtask) for the full class definition.

- [x] Create `terrain/navigation/nav_mesh_build_task.h`
  - Class declaration inheriting `IThreadedTask`
  - All input/output fields from the plan reference
  - `build_generation` field
  - `std::shared_ptr<NavMeshManager>` field
- [x] Create `terrain/navigation/nav_mesh_build_task.cpp`
  - `run()`: empty stub
  - `apply_result()`: empty stub
  - `get_priority()`: return `TaskPriority::max()`
  - `is_cancelled()`: return `!nav_mesh_manager || !nav_mesh_manager->valid`
  - `get_debug_name()`: return `"NavMeshBuild"`

### 1.4 MeshingDependency modification

Propagates the NavMeshManager to worker threads. See [plan reference — MeshingDependency Modification](navmesh_integration_plan_reference.md#meshingdependency-modification) for the modified struct.

- [x] Add `std::shared_ptr<NavMeshManager> nav_mesh_manager;` field to `MeshingDependency` in `engine/meshing_dependency.h` (gated on `#ifdef VOXEL_ENABLE_NAVIGATION`)
- [x] Update `MeshingDependency::reset()` signature to accept an optional `nav_mesh_manager` parameter (default `nullptr`)
- [x] Update all existing call sites of `MeshingDependency::reset()` in:
  - `terrain/fixed_lod/voxel_terrain.cpp` (lines 184, 327, 727 — search for `MeshingDependency::reset`)
  - `terrain/variable_lod/voxel_lod_terrain.cpp` (lines 337, 408, 710 — search for `MeshingDependency::reset`)
  - Pass `nullptr` for nav_mesh_manager at existing call sites for now

### 1.5 Navigation properties on VoxelTerrain

Inspector-exposed Recast parameters on the terrain node. See [plan reference — Exposed Properties](navmesh_integration_plan_reference.md#exposed-properties) for property groups and derivation logic.

- [x] Add navigation member variables to `VoxelTerrain` (in `terrain/fixed_lod/voxel_terrain.h`), gated on `#ifdef VOXEL_ENABLE_NAVIGATION`:
  ```cpp
  // Navigation
  bool _generate_navigation = false;
  uint32_t _navigation_layers = 1;
  float _nav_agent_radius = 0.4f;
  float _nav_agent_height = 1.8f;
  float _nav_agent_max_climb = 0.3f;
  float _nav_agent_max_slope = 45.0f;
  float _nav_cell_size = 0.2f;
  float _nav_cell_height = 0.1f;
  bool _nav_filter_low_hanging = true;
  bool _nav_filter_ledge_spans = true;
  bool _nav_filter_low_height = true;
  int _nav_region_min_size = 8;
  int _nav_region_merge_size = 20;
  float _nav_edge_max_length = 3.2f;
  float _nav_edge_max_error = 1.3f;
  float _nav_detail_sample_dist = 1.2f;
  float _nav_detail_sample_max_error = 0.1f;
  std::shared_ptr<NavMeshManager> _nav_mesh_manager;
  ```
- [x] Add getter/setter methods for each property
  - Setters should `WARN_PRINT` if a float is set to <= 0 (for properties where that's invalid)
  - Setters don't trigger rebuilds yet (Phase 3)
- [x] Add `_bind_methods()` entries for all properties with `ADD_GROUP` inspector grouping (gated on `#ifdef VOXEL_ENABLE_NAVIGATION`)
  - `"Navigation"` group: `generate_navigation`, `navigation_layers` (PROPERTY_HINT_LAYERS_3D_NAVIGATION)
  - `"Navigation Agent"` group with prefix `"nav_agent_"`: radius, height, max_climb, max_slope
  - `"Navigation Advanced"` group with prefix `"nav_"`: cell_size, cell_height, filters, region sizes, edge params, detail params
- [x] Add obstacle API method bindings: `add_nav_obstacle` (with `walkable` parameter, default `false`), `remove_nav_obstacle`, `update_nav_obstacle_transform`
  - Stub implementations that return 0 / do nothing for now

### 1.6 VoxelNavViewer skeleton

Controls where navmesh is generated by proximity. Uses the same **global registry pattern** as `VoxelViewer` — registers with a central manager, not by walking the scene tree. See [plan reference — Navigation Viewer](navmesh_integration_plan_reference.md#navigation-viewer).

- [x] Create `terrain/navigation/voxel_nav_viewer.h`
  - Inherits `Node3D`
  - Properties: `nav_distance` (unsigned int, default 64), `enabled_in_editor` (bool, default false)
  - Internal: `NavViewerID _viewer_id` — a `SlotMapKey` type matching the `ViewerID` pattern in `engine/ids.h`
- [x] Create `terrain/navigation/voxel_nav_viewer.cpp`
  - `NOTIFICATION_ENTER_TREE`: register with `NavMeshManager`'s global nav viewer registry via `NavMeshManager::add_nav_viewer()` (skip if in editor and `!_enabled_in_editor`). If `_pending_deferred_unregistration`, reuse existing ID instead of creating a new one.
  - `NOTIFICATION_EXIT_TREE`: **deferred** unregistration matching `VoxelViewer`'s pattern — set `_pending_deferred_unregistration = true` and schedule a deferred callback. The callback checks `is_inside_tree()` before actually removing (handles reparenting safely).
  - `NOTIFICATION_TRANSFORM_CHANGED`: update position in registry via `NavMeshManager::update_nav_viewer_position()`
  - `_bind_methods()`: expose `nav_distance`, `enabled_in_editor`
- [x] Add nav viewer registry to `NavMeshManager`:
  - `SlotMap<NavViewerState>` with position and distance per viewer (matching the `VoxelEngine::Viewer` storage pattern)
  - `add_nav_viewer()` / `remove_nav_viewer()` / `update_nav_viewer_position()` methods
  - `_is_within_nav_range(Vector3i chunk_pos)` iterates all registered nav viewers
  - Thread-safe read access (nav viewers update on main thread, readiness checks happen on worker threads via `_viewer_mutex`)

### 1.7 Verification

- [x] Build the project successfully with `scons` (no compile errors)
- [x] Open the editor and verify:
  - Navigation properties appear in the inspector for VoxelTerrain
  - Properties are grouped correctly (Navigation, Navigation Agent, Navigation Advanced)
  - `navigation_layers` shows the layer mask editor
  - Default values display correctly in the inspector
- [x] Verify that setting `generate_navigation = true` doesn't crash (it's a no-op at this point)

---

## Phase 2: NavMeshManager Lifecycle and MeshBlockTask Hook

**Goal:** Wire up the NavMeshManager creation/destruction lifecycle and the MeshBlockTask data extraction hook. Triangle data flows from the mesher into the NavMeshManager's cache, but no Recast builds happen yet.

### 2.1 NavMeshManager lifecycle in VoxelTerrain

- [x] In `set_generate_navigation(bool)`:
  - If enabling: create `_nav_mesh_manager = make_shared_instance<NavMeshManager>()`
  - Copy all current nav properties into the manager (recast_config, filters, etc.)
  - Store `mesh_block_size` on the manager (from `get_mesh_block_size()`) so it can compute world positions
  - Set `_nav_mesh_manager->_nav_map_rid` from `get_world_3d()->get_navigation_map()`
  - Update `MeshingDependency` to include the nav_mesh_manager
  - If disabling: `_nav_mesh_manager->valid = false`, `_nav_mesh_manager->clear_all()`, set to nullptr
  - Update `MeshingDependency` with nullptr
- [x] In `~VoxelTerrain()` destructor (matching the existing pattern where `_meshing_dependency->valid = false` is set in the destructor, not in `NOTIFICATION_EXIT_TREE`):
  - If `_nav_mesh_manager`: set `valid = false`, call `clear_all()`, reset to nullptr
- [x] Pass `_nav_mesh_manager` in all `MeshingDependency::reset()` calls
- [x] Implement `_recompute_nav_config()` helper that derives rcConfig from properties and stores on the manager (see [plan reference — Property derivation](navmesh_integration_plan_reference.md#exposed-properties))
  - Call this from `set_generate_navigation()` and from all nav property setters when manager exists

### 2.2 MeshBlockTask hook — triangle data extraction

Extracts collision triangles from the mesher and forwards them to `NavMeshManager`. See [plan reference — MeshBlockTask Hook Point](navmesh_integration_plan_reference.md#meshblocktask-hook-point) for code snippet and rationale.

- [x] Inside `MeshBlockTask::build_mesh()` in `meshers/mesh_block_task.cpp`, after the detail texture scheduling block (~line 579), add the nav hook:
  - Gate on `#ifdef VOXEL_ENABLE_NAVIGATION`
  - Check `meshing_dependency->nav_mesh_manager != nullptr && lod_index == 0`
  - Explicitly check the mesher is Transvoxel via `try_get_as(mesher, transvoxel_mesher)`
  - Check `_surfaces_output.collision_surface.submesh_vertex_end > 0` (non-empty mesh)
  - Use `VoxelMesherTransvoxel::get_mesh_cache_from_current_thread()` for raw `Vector3f` vertex data
  - Copy vertices and indices up to `submesh_vertex_end` / `submesh_index_end`
  - **Convert vertices from chunk-local space to world space** by offsetting each vertex by `chunk_position * mesh_block_size` (Transvoxel outputs in local space; Recast needs a common coordinate system across chunks — see plan reference notes)
  - Compute `nav_data.world_aabb` from `mesh_block_position` and `mesh_block_size`
  - Set `nav_data.chunk_position = mesh_block_position`
  - Call `nav_mesh_manager->on_mesh_built(std::move(nav_data))`

### 2.3 NavMeshManager::on_mesh_built() — cache only

- [x] Implement `on_mesh_built()` to cache data but NOT dispatch builds yet:
  - Lock `_cache_mutex`
  - Store/replace `NavChunkEntry` for this chunk position
  - Bump `entry.generation++`
  - (Dispatch logic added in Phase 3)
- [x] Implement `remove_chunk()`:
  - Free region RID if it exists
  - Remove from `_region_rids`, `_applied_generations`
  - Remove from `_chunk_cache` under lock
- [x] Implement `clear_all()`:
  - Free all region RIDs
  - Clear all hashmaps

### 2.4 Chunk unload integration

- [x] In VoxelTerrain: when a mesh block is unloaded in `unload_mesh_block()` (`voxel_terrain.cpp:528`), call `_nav_mesh_manager->remove_chunk(bpos)` if manager exists

### 2.5 Verification

- [x] Build compiles
- [x] With `generate_navigation = true`, terrain loads without crashes
- [x] Add temporary debug logging to `on_mesh_built()` to confirm triangle data arrives:
  - Log chunk position, vertex count, index count
  - Verify data stops arriving when `generate_navigation` is toggled off
- [x] Verify no crashes when toggling `generate_navigation` on/off at runtime
- [x] Verify no crashes when removing the terrain node from the scene tree

---

## Phase 3: Neighbor Readiness and NavMeshBuildTask Dispatch

**Goal:** Implement the readiness checking and dispatch logic. NavMeshBuildTasks are pushed to the thread pool with real data, but the Recast pipeline itself is still a stub — tasks just log that they would build.

### 3.1 Neighbor readiness logic

Ensures border geometry is available before building. See [plan reference — Border Geometry and Neighbor Readiness](navmesh_integration_plan_reference.md#border-geometry-and-neighbor-readiness).

- [ ] Define `neighbor_offsets` — the set of Vector3i offsets to check (start with 6 axis neighbors; expand to 26 if border overlap requires corners)
- [ ] Implement `_are_neighbors_ready(Vector3i chunk_pos)`:
  - For each offset in `neighbor_offsets`, check if `_chunk_cache.find(chunk_pos + offset)` exists
  - Return true only if all neighbors have cached data
- [ ] Implement `_is_within_nav_range(Vector3i chunk_pos)`:
  - Compute chunk world center from `chunk_pos * mesh_block_size + mesh_block_size / 2`
  - Check against all registered navigation viewers: return true if the chunk is within `nav_distance` of **any** `VoxelNavViewer`
  - Read the nav viewer registry under its mutex (viewers update on main thread, this runs on worker threads)

### 3.2 Dispatch logic in on_mesh_built()

Checks readiness and dispatches build tasks when a chunk and its neighbors are cached. See [plan reference — Key Behaviors](navmesh_integration_plan_reference.md#key-behaviors) for code and dispatch flow.

- [ ] Complete `on_mesh_built()` implementation:
  - After caching data, call `_try_dispatch_nav_build(chunk_pos)`
  - Also call `_try_dispatch_nav_build(chunk_pos + offset)` for each neighbor offset
  - All under `_cache_mutex`
- [ ] Implement `_try_dispatch_nav_build(Vector3i)`:
  - Skip if not in cache, not in nav range, or neighbors not ready
  - Otherwise call `_dispatch_nav_build()`
- [ ] Implement `_dispatch_nav_build(Vector3i, uint32_t generation)`:
  - Create `NavMeshBuildTask` via `ZN_NEW`
  - Copy chunk triangles and neighbor triangles (already under `_cache_mutex`)
  - Acquire `_obstacle_mutex` for obstacle snapshot. **Lock ordering: `_cache_mutex` must always be acquired before `_obstacle_mutex`** — see [plan reference — Lock Ordering](navmesh_integration_plan_reference.md#lock-ordering) for rationale
  - Copy config, filter flags, generation
  - Set `nav_mesh_manager = shared_from_this()`
  - Push via `VoxelEngine::get_singleton().push_async_task(task)`

### 3.3 NavMeshBuildTask — stub with logging

- [ ] In `NavMeshBuildTask::run()`:
  - Log: chunk position, triangle count, neighbor count, obstacle count
  - Set `result_nav_mesh` to null (no actual Recast work yet)
- [ ] In `NavMeshBuildTask::apply_result()`:
  - Check `nav_mesh_manager && nav_mesh_manager->valid && result_nav_mesh.is_valid()`
  - Call `nav_mesh_manager->apply_nav_result(...)` (which will be a no-op since result is null)
- [ ] **Note on priority:** `get_priority()` returns `TaskPriority::max()` (lowest priority) for now. This means nav tasks run after all mesh tasks, which may cause noticeable latency before navmesh appears. Priority tuning is deferred to future optimization.

### 3.4 Verification

- [ ] Build compiles
- [ ] With `generate_navigation = true`, verify dispatch logging shows:
  - Tasks dispatch once neighborhoods complete
  - Interior chunks dispatch first, edge chunks later
  - Each chunk dispatches exactly once on initial load
- [ ] Verify terrain edits trigger re-dispatch for the edited chunk
- [ ] Verify no duplicate dispatches for the same chunk in steady state
- [ ] Verify no crashes under concurrent meshing (multiple worker threads calling `on_mesh_built()`)

---

## Phase 4: Recast Pipeline

**Goal:** Implement the full Recast build inside `NavMeshBuildTask::run()`. Output is a `Ref<NavigationMesh>` that gets registered with NavigationServer3D.

### 4.1 Recast heightfield setup

- [ ] In `NavMeshBuildTask::run()`:
  - Compute `cfg.bmin` / `cfg.bmax` from chunk world position, expanded by `borderSize * cs`
  - Call `rcCalcGridSize()` to set `cfg.width` / `cfg.height`
  - Allocate heightfield with `rcAllocHeightfield()` + `rcCreateHeightfield()`
  - Add `ERR_FAIL_NULL` / error return guards

### 4.2 Terrain rasterization

- [ ] Implement the `rasterize` lambda (see plan reference):
  - `rcMarkWalkableTriangles()` to classify by slope
  - `rcRasterizeTriangles()` into the heightfield
  - Handle empty data gracefully (zero indices → skip)
- [ ] Rasterize this chunk's triangles
- [ ] Rasterize all neighbor triangles

### 4.3 Obstacle rasterization

- [ ] For each obstacle in the snapshot:
  - Extract mesh vertices and indices from `Ref<Mesh>`
  - Transform vertices to world space using obstacle transform
  - If `walkable == false`: mark all triangles as `RC_NULL_AREA` (blocks navigation)
  - If `walkable == true`: run `rcMarkWalkableTriangles()` to evaluate slope-based walkability (allows bridges, ramps, rooftops)
  - Rasterize into the same heightfield

### 4.4 Filtering

- [ ] Apply filters based on config flags (order matters — see plan reference):
  - `rcFilterLowHangingWalkableObstacles` (if enabled)
  - `rcFilterLedgeSpans` (if enabled, must be AFTER low hanging)
  - `rcFilterWalkableLowHeightSpans` (if enabled)

### 4.5 Compact, erode, regions

- [ ] `rcBuildCompactHeightfield()`
- [ ] Free heightfield
- [ ] `rcErodeWalkableArea()`
- [ ] `rcBuildDistanceField()`
- [ ] `rcBuildRegionsMonotone()` (monotone for runtime/streaming performance)
- [ ] Add error guards at each step

### 4.6 Contours, polymesh, detail mesh

- [ ] `rcBuildContours()`
- [ ] `rcBuildPolyMesh()`
- [ ] `rcBuildPolyMeshDetail()`
- [ ] Free intermediate structures (compact heightfield, contour set)
- [ ] Add error guards

### 4.7 Convert rcPolyMeshDetail to NavigationMesh

Converts Recast output to Godot's NavigationMesh format. See [plan reference — run() Step 7](navmesh_integration_plan_reference.md#run--recast-pipeline) for conversion code and winding order details.

- [ ] Extract vertices from `dmesh->verts` (float world-space coordinates)
- [ ] Deduplicate vertices (HashMap-based, matching Godot's approach)
- [ ] Extract triangles with REVERSED winding order (`[0, 2, 1]` not `[0, 1, 2]`)
- [ ] Set `cell_size` and `cell_height` on the NavigationMesh resource
- [ ] Free polymesh and detail mesh
- [ ] Store result in `result_nav_mesh`

### 4.8 apply_result() and region registration

Registers the built navmesh with NavigationServer3D. See [plan reference — Region Registration and Edge Stitching](navmesh_integration_plan_reference.md#region-registration-and-edge-stitching) for registration code and edge matching.

- [ ] Implement `NavMeshBuildTask::apply_result()`:
  - Check validity and non-null result
  - Call `nav_mesh_manager->apply_nav_result(chunk_pos, result, build_generation)`
- [ ] Implement `NavMeshManager::apply_nav_result()`:
  - Generation counter check (skip if stale)
  - Create region RID if needed (`region_create`, `region_set_map`, `region_set_navigation_layers`, `region_set_enabled`)
  - Set transform and navigation mesh on the region
  - Update `_applied_generations`
- [ ] Set navigation map cell size/height on manager initialization:
  - `map_set_cell_size(_nav_map_rid, cfg.cs)`
  - `map_set_cell_height(_nav_map_rid, cfg.ch)`

### 4.9 Verification

- [ ] Build compiles
- [ ] With `generate_navigation = true` and a Transvoxel terrain:
  - Navigation mesh regions appear (use Godot's debug navigation visualization)
  - Navmesh covers the terrain surface within nav_range
  - No gaps at chunk boundaries (edge stitching works)
  - No navmesh on steep slopes
  - Correct erosion away from ledges
- [ ] Add a NavigationAgent3D to a character and verify pathfinding works
- [ ] Test terrain edits — navmesh rebuilds for affected chunks
- [ ] Test performance — no main thread hitches from Recast builds

---

## Phase 5: Obstacle System and Polish

**Goal:** Implement the obstacle API and handle remaining edge cases.

### 5.1 Obstacle API implementation

Rasterizes obstacle geometry into the heightfield to carve navigation. See [plan reference — Obstacle Handling](navmesh_integration_plan_reference.md#obstacle-handling) for the API and dirty propagation logic.

- [ ] Implement `NavMeshManager::add_obstacle()`:
  - Accepts `Ref<Mesh> collision_mesh`, `Transform3D transform`, and `bool walkable` (default `false`)
  - **Lock ordering:** acquire `_cache_mutex` first, then `_obstacle_mutex` (consistent with `_dispatch_nav_build`)
  - Store in `_obstacles` registry under `_obstacle_mutex` (including the `walkable` flag)
  - Compute world AABB from mesh + transform
  - Find affected chunks via `_get_affected_chunks()`
  - Dispatch nav builds for affected chunks with cached data (under `_cache_mutex`)
  - Return obstacle ID
- [ ] Implement `NavMeshManager::remove_obstacle()`:
  - Remove from registry
  - Dispatch rebuilds for previously-affected chunks
- [ ] Implement `NavMeshManager::update_obstacle_transform()`:
  - Update transform and recompute AABB
  - Dispatch rebuilds for both old and new affected chunks
- [ ] Wire up terrain node methods to forward to NavMeshManager

### 5.2 _get_affected_chunks()

- [ ] Implement AABB-to-chunk-positions conversion
  - Account for borderSize expansion

### 5.3 Property change handling

- [ ] When agent or advanced nav properties change while manager is active:
  - Recompute rcConfig via `_recompute_nav_config()`
  - Mark all cached chunks for rebuild (or clear and let them rebuild naturally)
- [ ] When nav viewer positions/distances change: potentially add/remove region RIDs for chunks entering/leaving range
- [ ] When `navigation_layers` changes: update all existing region RIDs

### 5.4 Edge cases and cleanup

- [ ] Handle empty chunks gracefully (zero triangles from mesher → skip nav, don't cache empty data)
- [ ] Handle terrain with no Transvoxel mesher (nav_mesh_manager should be null)
- [ ] Verify `clear_all()` properly frees all NavigationServer3D resources
- [ ] Test toggling `generate_navigation` off and on at runtime
- [ ] Test removing terrain node while nav builds are in flight

### 5.5 Verification

- [ ] Obstacles carve into the navmesh correctly
- [ ] Moving obstacles update affected chunks
- [ ] Removing obstacles restores original navmesh
- [ ] No crashes during rapid obstacle add/remove
- [ ] No memory leaks (check with sanitizers or Godot debug build)

---

## Out of Scope (Future Work)

These are documented in the plan reference under "Future Improvements" and are explicitly not part of the initial implementation:

- VoxelLodTerrain support (navigation on variable-LOD terrain)
- Debouncing rapid terrain edits
- LOD-aware navigation (coarse LODs for long-range pathfinding)
- Projected area marking (lightweight obstacle alternative)
- Custom rcContext for debug logging
- Editor UI (gizmos, nav range visualization)
- NavMeshBuildTask priority tuning (currently uses lowest priority)
