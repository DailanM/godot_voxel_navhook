# Navmesh Integration — Implementation Plan

This document outlines the phased implementation process. Each phase is scoped to be completable in a single session. Code snippets and architectural details are in the [plan reference](navmesh_integration_plan_reference.md).

---

## Phase 1: Skeleton Classes, Build System, and UI

**Goal:** Create the new files, add Recast include path, expose all properties in the inspector, and verify the build compiles. No navigation logic yet — just the framework.

### 1.1 Build system setup

- [ ] Create `terrain/navigation/` directory
- [ ] Add `"terrain/navigation/*.cpp"` to `common.py` source list, gated on a new `voxel_navigation` build option
- [ ] Add `voxel_navigation` option to `config.py` (default enabled)
- [ ] Add Recast include path to `SCsub`: `env_voxel.Prepend(CPPPATH=["#thirdparty/recastnavigation/Recast/Include"])`
- [ ] Gate the include path on `voxel_navigation` being enabled
- [ ] Verify build compiles with `#include <Recast.h>` in a new file

### 1.2 NavMeshManager skeleton

- [ ] Create `terrain/navigation/nav_mesh_manager.h`
  - Class declaration with all public/private members from the plan reference
  - All method signatures declared but not implemented (empty bodies or stubs)
  - `#ifdef MODULE_NAVIGATION_3D_ENABLED` guard around the whole file
  - `NavChunkData` struct definition
  - `NavChunkEntry` struct (with generation counter)
  - `ObstacleEntry` struct
  - All member variables: `_chunk_cache`, `_obstacles`, `_region_rids`, `_applied_generations`, mutexes, config fields
- [ ] Create `terrain/navigation/nav_mesh_manager.cpp`
  - Stub implementations for all methods (just return defaults / do nothing)
  - `on_mesh_built()`: empty (will be implemented in Phase 3)
  - `add_obstacle()` / `remove_obstacle()` / `update_obstacle_transform()`: empty stubs returning 0 / doing nothing
  - `apply_nav_result()`: empty
  - `remove_chunk()`: empty
  - `clear_all()`: empty

### 1.3 NavMeshBuildTask skeleton

- [ ] Create `terrain/navigation/nav_mesh_build_task.h`
  - Class declaration inheriting `IThreadedTask`
  - All input/output fields from the plan reference
  - `build_generation` field
  - `std::shared_ptr<NavMeshManager>` field
- [ ] Create `terrain/navigation/nav_mesh_build_task.cpp`
  - `run()`: empty stub
  - `apply_result()`: empty stub
  - `get_priority()`: return `TaskPriority::max()`
  - `is_cancelled()`: return `!nav_mesh_manager || !nav_mesh_manager->valid`
  - `get_debug_name()`: return `"NavMeshBuild"`

### 1.4 MeshingDependency modification

- [ ] Add `std::shared_ptr<NavMeshManager> nav_mesh_manager;` field to `MeshingDependency` in `engine/meshing_dependency.h`
- [ ] Update `MeshingDependency::reset()` signature to accept an optional `nav_mesh_manager` parameter (default `nullptr`)
- [ ] Update all existing call sites of `MeshingDependency::reset()` in:
  - `terrain/fixed_lod/voxel_terrain.cpp` (search for `MeshingDependency::reset`)
  - `terrain/variable_lod/voxel_lod_terrain.cpp` (search for `MeshingDependency::reset`)
  - Pass `nullptr` for nav_mesh_manager at existing call sites for now

### 1.5 Navigation properties on VoxelTerrain

- [ ] Add navigation member variables to `VoxelTerrain` (in `terrain/fixed_lod/voxel_terrain.h`):
  ```cpp
  // Navigation
  bool _generate_navigation = false;
  uint32_t _navigation_layers = 1;
  float _nav_range = 128.0f;
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
- [ ] Add getter/setter methods for each property
  - Setters should `WARN_PRINT` if a float is set to <= 0 (for properties where that's invalid)
  - Setters don't trigger rebuilds yet (Phase 3)
- [ ] Add `_bind_methods()` entries for all properties with `ADD_GROUP` inspector grouping
  - `"Navigation"` group: `generate_navigation`, `navigation_layers` (PROPERTY_HINT_LAYERS_3D_NAVIGATION), `nav_range`
  - `"Navigation Agent"` group with prefix `"nav_agent_"`: radius, height, max_climb, max_slope
  - `"Navigation Advanced"` group with prefix `"nav_"`: cell_size, cell_height, filters, region sizes, edge params, detail params
- [ ] Add obstacle API method bindings: `add_nav_obstacle`, `remove_nav_obstacle`, `update_nav_obstacle_transform`
  - Stub implementations that return 0 / do nothing for now

### 1.6 Navigation properties on VoxelLodTerrain

- [ ] Repeat the same property additions for `VoxelLodTerrain` (in `terrain/variable_lod/voxel_lod_terrain.h/.cpp`)
  - Same member variables, getters/setters, and `_bind_methods()` entries
  - Same obstacle API bindings

### 1.7 Verification

- [ ] Build the project successfully with `scons` (no compile errors)
- [ ] Open the editor and verify:
  - Navigation properties appear in the inspector for VoxelTerrain
  - Navigation properties appear in the inspector for VoxelLodTerrain
  - Properties are grouped correctly (Navigation, Navigation Agent, Navigation Advanced)
  - `navigation_layers` shows the layer mask editor
  - Default values display correctly in the inspector
- [ ] Verify that setting `generate_navigation = true` doesn't crash (it's a no-op at this point)

---

## Phase 2: NavMeshManager Lifecycle and MeshBlockTask Hook

**Goal:** Wire up the NavMeshManager creation/destruction lifecycle and the MeshBlockTask data extraction hook. Triangle data flows from the mesher into the NavMeshManager's cache, but no Recast builds happen yet.

### 2.1 NavMeshManager lifecycle in VoxelTerrain

- [ ] In `set_generate_navigation(bool)`:
  - If enabling: create `_nav_mesh_manager = make_shared_instance<NavMeshManager>()`
  - Copy all current nav properties into the manager (recast_config, filters, nav_range, etc.)
  - Set `_nav_mesh_manager->_nav_map_rid` from `get_world_3d()->get_navigation_map()`
  - Update `MeshingDependency` to include the nav_mesh_manager
  - If disabling: `_nav_mesh_manager->valid = false`, `_nav_mesh_manager->clear_all()`, set to nullptr
  - Update `MeshingDependency` with nullptr
- [ ] In `_notification(NOTIFICATION_EXIT_TREE)`:
  - If `_nav_mesh_manager`: set `valid = false`, call `clear_all()`, reset to nullptr
- [ ] Pass `_nav_mesh_manager` in all `MeshingDependency::reset()` calls
- [ ] Implement `_recompute_nav_config()` helper that derives rcConfig from properties and stores on the manager
  - Call this from `set_generate_navigation()` and from all nav property setters when manager exists

### 2.2 NavMeshManager lifecycle in VoxelLodTerrain

- [ ] Same lifecycle integration as VoxelTerrain (adapted to VoxelLodTerrain's structure)

### 2.3 MeshBlockTask hook — triangle data extraction

- [ ] At end of `MeshBlockTask::build_mesh()` in `meshers/mesh_block_task.cpp`, add the hook:
  - Gate on `#ifdef MODULE_NAVIGATION_3D_ENABLED`
  - Check `meshing_dependency->nav_mesh_manager != nullptr && lod_index == 0`
  - Check `_surfaces_output.collision_surface.submesh_vertex_end > 0`
  - Use `VoxelMesherTransvoxel::get_mesh_cache_from_current_thread()` for raw vertex data
  - Copy positions and indices up to `submesh_vertex_end` / `submesh_index_end`
  - Set `nav_data.chunk_position = mesh_block_position`
  - Compute `nav_data.world_aabb` from chunk position and mesh block size
  - Call `nav_mesh_manager->on_mesh_built(std::move(nav_data))`
- [ ] Include the Transvoxel header guard — only extract data if the mesher is Transvoxel

### 2.4 NavMeshManager::on_mesh_built() — cache only

- [ ] Implement `on_mesh_built()` to cache data but NOT dispatch builds yet:
  - Lock `_cache_mutex`
  - Store/replace `NavChunkEntry` for this chunk position
  - Bump `entry.generation++`
  - (Dispatch logic added in Phase 3)
- [ ] Implement `remove_chunk()`:
  - Free region RID if it exists
  - Remove from `_region_rids`, `_applied_generations`
  - Remove from `_chunk_cache` under lock
- [ ] Implement `clear_all()`:
  - Free all region RIDs
  - Clear all hashmaps

### 2.5 Chunk unload integration

- [ ] In VoxelTerrain: when a mesh block is freed (look for `FreeMeshBlockTask` usage), call `_nav_mesh_manager->remove_chunk(pos)` if manager exists
- [ ] In VoxelLodTerrain: same integration at the appropriate chunk unload point

### 2.6 Verification

- [ ] Build compiles
- [ ] With `generate_navigation = true`, terrain loads without crashes
- [ ] Add temporary debug logging to `on_mesh_built()` to confirm triangle data arrives:
  - Log chunk position, vertex count, index count
  - Verify data arrives for LOD 0 chunks only
  - Verify data stops arriving when `generate_navigation` is toggled off
- [ ] Verify no crashes when toggling `generate_navigation` on/off at runtime
- [ ] Verify no crashes when removing the terrain node from the scene tree

---

## Phase 3: Neighbor Readiness and NavMeshBuildTask Dispatch

**Goal:** Implement the readiness checking and dispatch logic. NavMeshBuildTasks are pushed to the thread pool with real data, but the Recast pipeline itself is still a stub — tasks just log that they would build.

### 3.1 Neighbor readiness logic

- [ ] Define `neighbor_offsets` — the set of Vector3i offsets to check (start with 6 axis neighbors; expand to 26 if border overlap requires corners)
- [ ] Implement `_are_neighbors_ready(Vector3i chunk_pos)`:
  - For each offset in `neighbor_offsets`, check if `_chunk_cache.find(chunk_pos + offset)` exists
  - Return true only if all neighbors have cached data
- [ ] Implement `_is_within_nav_range(Vector3i chunk_pos)`:
  - Compare chunk world position against nav_range (relative to... viewer? terrain origin? — decide and document)

### 3.2 Dispatch logic in on_mesh_built()

- [ ] Complete `on_mesh_built()` implementation:
  - After caching data, call `_try_dispatch_nav_build(chunk_pos)`
  - Also call `_try_dispatch_nav_build(chunk_pos + offset)` for each neighbor offset
  - All under `_cache_mutex`
- [ ] Implement `_try_dispatch_nav_build(Vector3i)`:
  - Skip if not in cache, not in nav range, or neighbors not ready
  - Otherwise call `_dispatch_nav_build()`
- [ ] Implement `_dispatch_nav_build(Vector3i, uint32_t generation)`:
  - Create `NavMeshBuildTask` via `ZN_NEW`
  - Copy chunk triangles, neighbor triangles, obstacle snapshot (under `_obstacle_mutex`)
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
  - Mark all triangles as `RC_NULL_AREA`
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

- [ ] Extract vertices from `dmesh->verts` (float world-space coordinates)
- [ ] Deduplicate vertices (HashMap-based, matching Godot's approach)
- [ ] Extract triangles with REVERSED winding order (`[0, 2, 1]` not `[0, 1, 2]`)
- [ ] Set `cell_size` and `cell_height` on the NavigationMesh resource
- [ ] Free polymesh and detail mesh
- [ ] Store result in `result_nav_mesh`

### 4.8 apply_result() and region registration

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

- [ ] Implement `NavMeshManager::add_obstacle()`:
  - Store in `_obstacles` registry under `_obstacle_mutex`
  - Compute world AABB from mesh + transform
  - Find affected chunks via `_get_affected_chunks()`
  - Dispatch nav builds for affected chunks with cached data
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
- [ ] When `nav_range` changes: potentially add/remove region RIDs for chunks entering/leaving range
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

- Debouncing rapid terrain edits
- LOD-aware navigation
- Projected area marking (lightweight obstacle alternative)
- Custom rcContext for debug logging
- Editor UI (gizmos, nav range visualization)
- Walkable obstacle flag
