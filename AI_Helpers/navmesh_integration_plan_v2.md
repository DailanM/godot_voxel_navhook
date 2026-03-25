# Godot-Voxel Recast Navigation Integration — Plan v2

## Overview

This plan describes how to generate navigation meshes for smooth (Transvoxel) voxel terrain by feeding the triangle mesh already produced by the godot-voxel mesher into Recast's pipeline, then registering the result with Godot's NavigationServer3D.

The core architectural decision is **lightly-coupled post-mesher dispatch**: at the end of `MeshBlockTask::run()` (on a worker thread), the collision triangle data is extracted and handed to a `NavMeshManager`. The manager caches per-chunk triangle data, tracks neighbor readiness, manages an obstacle registry, and dispatches `NavMeshBuildTask`s to the thread pool when a chunk and its neighbors are all available. Final NavigationServer3D calls happen on the main thread.

This approach:
- **Supports full threading** — nav data capture and Recast builds happen on worker threads; only NavigationServer3D calls touch the main thread.
- **Supports per-chunk incremental updates** — when terrain changes or obstacles move, only affected chunks are rebuilt.
- **Handles border overlap naturally** — chunks wait for neighbor data before building, so border geometry is always available.

---

## File Organization

New files live under `terrain/navigation/`, mirroring the organization of other terrain subsystems (`terrain/instancing/`, `engine/detail_rendering/`):

```
terrain/navigation/
  nav_mesh_manager.h       — NavMeshManager class (coordinator)
  nav_mesh_manager.cpp
  nav_mesh_build_task.h    — NavMeshBuildTask (IThreadedTask, Recast pipeline)
  nav_mesh_build_task.cpp
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
               (checks generation counter —                  (via TimeSpreadTaskRunner)
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
    int add_obstacle(Ref<Mesh> collision_mesh, Transform3D transform);
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

    // --- Configuration (set before use, typically from editor properties) ---
    rcConfig recast_config;         // Recast build parameters
    float nav_range = 128.0f;      // Independent from mesh loading range
    bool filter_low_hanging = true;
    bool filter_ledge_spans = true;
    bool filter_low_height_spans = true;

private:
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
    };
    Mutex _obstacle_mutex;
    HashMap<int, ObstacleEntry> _obstacles;
    int _next_obstacle_id = 0;

    // Active NavigationServer3D regions. Main thread only.
    HashMap<Vector3i, RID> _region_rids;

    // Tracks the generation that was last applied per chunk (main thread only).
    // Used to skip stale NavMeshBuildTask results.
    HashMap<Vector3i, uint32_t> _applied_generations;

    // Helpers (all called under _cache_mutex)
    bool _are_neighbors_ready(Vector3i chunk_pos) const;
    bool _is_within_nav_range(Vector3i chunk_pos) const;
    void _try_dispatch_nav_build(Vector3i chunk_pos);
    void _dispatch_nav_build(Vector3i chunk_pos, uint32_t generation);
    StdVector<Vector3i> _get_affected_chunks(const AABB &aabb);
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

**`apply_nav_result()`** (main thread, via TimeSpreadTaskRunner):
1. Compare `build_generation` against `_applied_generations[chunk_pos]`. If the build is for an older generation than what's already applied, skip it (a newer build is in flight or already applied).
2. If no region RID exists for this chunk, create one via `NavigationServer3D::region_create()`.
3. Call `region_set_navigation_mesh()` with the new data.
4. Update `_applied_generations[chunk_pos] = build_generation`.

**`remove_chunk()`** (main thread):
1. Free the region RID via `NavigationServer3D::free()`.
2. Remove from `_region_rids` and `_applied_generations`.
3. Remove from `_chunk_cache` (under `_cache_mutex`).

---

## MeshingDependency Modification

`MeshingDependency` gains a nullable `nav_mesh_manager` field. This is the mechanism by which worker threads access the NavMeshManager:

```cpp
struct MeshingDependency {
    Ref<VoxelMesher> mesher;
    Ref<VoxelGenerator> generator;
    std::shared_ptr<NavMeshManager> nav_mesh_manager; // NEW — nullable for opt-in
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

Both `VoxelTerrain` and `VoxelLodTerrain` need to pass their `NavMeshManager` (if any) through `MeshingDependency::reset()` calls.

---

## MeshBlockTask Hook Point

At the end of `MeshBlockTask::run()`, after `build_mesh()` completes, a small block extracts collision triangle data and forwards it to the NavMeshManager. The nav_mesh_manager pointer is nullable — if null, no nav work happens, keeping the coupling optional.

**Data extraction uses the Transvoxel thread-local cache** (`get_mesh_cache_from_current_thread()`) for direct access to raw `StdVector<Vector3f>` vertices — avoiding Godot Variant array overhead. The bounds come from `_surfaces_output.collision_surface.submesh_vertex_end/submesh_index_end` to exclude LOD transition mesh vertices.

```cpp
// In MeshBlockTask::run(), after build_mesh():
#ifdef MODULE_NAVIGATION_3D_ENABLED
if (meshing_dependency->nav_mesh_manager != nullptr && lod_index == 0) {
    const VoxelMesher::Output::CollisionSurface &col = _surfaces_output.collision_surface;

    if (_surfaces_output.surfaces.size() > 0 && col.submesh_vertex_end > 0) {
        NavChunkData nav_data;
        nav_data.chunk_position = mesh_block_position;
        // TODO: compute world_aabb from chunk position and mesh block size

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
#endif
```

**Notes:**
- Only LOD 0 chunks produce nav data (higher LODs are too coarse for accurate navigation).
- The `submesh_vertex_end` / `submesh_index_end` bounds exclude transition mesh vertices used for LOD stitching — these must NOT be included in nav generation.
- `get_mesh_cache_from_current_thread()` returns data that's only valid until the next `build()` call on that thread. We copy immediately via `assign()`, which is fine.
- In double-precision builds, the thread cache already uses `Vector3f` (32-bit), which is what Recast expects. No conversion needed.
- Gated on `MODULE_NAVIGATION_3D_ENABLED` for builds without the navigation module.
- The Transvoxel mesher check is implicit — `get_mesh_cache_from_current_thread()` is only meaningful after a Transvoxel `build()`. If a different mesher is in use, the nav_mesh_manager should be null (navigation is only supported with Transvoxel for now).

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
    // Non-walkable obstacles: mark all triangles as RC_NULL_AREA
    StdVector<unsigned char> obs_areas(obs_num_tris, RC_NULL_AREA);
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

```cpp
void NavMeshBuildTask::apply_result() {
    if (nav_mesh_manager && nav_mesh_manager->valid && result_nav_mesh.is_valid()) {
        nav_mesh_manager->apply_nav_result(chunk_position, result_nav_mesh, build_generation);
    }
}
```

---

## Recast Configuration

Default parameters, exposed as properties on the NavMeshManager (or a future VoxelNavBuilder node):

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

    // Snapshot overlapping obstacles (separate mutex)
    AABB expanded_aabb = /* chunk AABB expanded by borderSize * cs */;
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

The terrain node is responsible for calling `NavMeshManager::remove_chunk()` when chunks leave the loaded area. The integration points differ by terrain type:

**VoxelLodTerrain:** In the LOD update task (`voxel_lod_terrain_update_task.cpp`), when LOD 0 blocks are unloaded, a callback or deferred main-thread action should call `remove_chunk()`. This parallels how `FreeMeshBlockTask` is already queued via `TimeSpreadTaskRunner` when mesh blocks are freed.

**VoxelTerrain:** In `voxel_terrain.cpp`, the `_on_block_exit()` or equivalent cleanup path should call `remove_chunk()`.

### Terrain node removal (`clear_all`)

When a `VoxelTerrain`/`VoxelLodTerrain` node exits the scene tree:
1. `nav_mesh_manager->valid = false` — cancels all in-flight NavMeshBuildTasks.
2. `nav_mesh_manager->clear_all()` — frees all NavigationServer3D region RIDs and clears internal state.
3. The terrain node's destructor (or `_exit_tree()`) handles this.

This mirrors how both terrains already set `_meshing_dependency->valid = false` in their destructors.

---

## Obstacle Handling

Obstacles use **full mesh rasterization** — the actual obstacle geometry is rasterized into the heightfield alongside terrain. This produces accurate clearance around complex 3D shapes including overhangs, unlike Godot's projected obstruction approach which only handles vertical extrusions.

### API

```cpp
// Add an obstacle. Returns ID for later removal.
// Collision mesh is rasterized into any chunk whose nav bounds overlap the obstacle.
int NavMeshManager::add_obstacle(Ref<Mesh> collision_mesh, Transform3D transform);

// Remove obstacle and rebuild affected chunks.
void NavMeshManager::remove_obstacle(int obstacle_id);

// Update transform (e.g. obstacle moved). Rebuilds chunks affected by both
// old and new positions.
void NavMeshManager::update_obstacle_transform(int obstacle_id, Transform3D new_transform);
```

### Walkable vs Non-Walkable Obstacles

By default, obstacle triangles are marked `RC_NULL_AREA` (non-walkable — they block navigation). For obstacles with walkable surfaces (bridges, ramps, rooftops), the API should support a flag to run `rcMarkWalkableTriangles` on the obstacle mesh instead, so slope-based walkability is evaluated.

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

All navigation code should be gated on `MODULE_NAVIGATION_3D_ENABLED` (defined in `modules/modules_enabled.gen.h` when the navigation_3d module is active):

```cpp
#ifdef MODULE_NAVIGATION_3D_ENABLED
#include <Recast.h>
// ... nav code ...
#endif
```

This ensures the module compiles cleanly when:
- `disable_navigation_3d=yes` is set
- `disable_3d=yes` is set (cascades into disabling navigation_3d)
- `builtin_recastnavigation=no` with missing system library

The `voxel_navigation` build option in `common.py` should also check that `MODULE_NAVIGATION_3D_ENABLED` would be true, or at minimum document the dependency.

---

## Interface Points Summary

| Interface | From | To | Data |
|-----------|------|----|------|
| **1. Mesh capture** | `MeshBlockTask::run()` | `NavMeshManager::on_mesh_built()` | Collision vertices/indices (from thread cache), chunk position |
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

**Performance — Projected Area Marking:** Expose a collection of convex polygon footprints passed to `rcMarkConvexPolyArea()` on the `rcCompactHeightfield`. This is a lighter-weight alternative to full mesh rasterization for simple vertical obstacles (walls, fences), and can be used alongside the full rasterization approach. Could speed up rebuilds for common obstacle types.

**LOD-Aware Navigation:** Currently only LOD 0 chunks produce navmesh data. For large worlds, coarser LODs could provide approximate long-range pathfinding while LOD 0 handles detailed local navigation. The nav range must be <= the LOD 0 loading range.

**Error Handling:** The Recast pipeline can fail at various stages (allocation failure, empty input, degenerate geometry from empty/near-empty chunks). Needs `ERR_FAIL_COND` guards throughout the build pipeline. Basic guards are included in the plan above; a comprehensive pass should be done during implementation.

**Debouncing Rapid Edits:** When voxels are edited rapidly (painting terrain), the mesher already re-meshes affected chunks. The nav builder should debounce to avoid excessive Recast rebuilds — e.g., a short delay after the last edit before dispatching a nav build, or cancelling in-flight builds for chunks that have been re-dirtied. The generation counter already prevents stale results from being applied, but the redundant Recast compute is still wasted work.

**Navigation Range Configuration:** The nav range should be configurable independently from terrain mesh loading range. Players may need navigation within 64m even if terrain is visible at 256m.

**Cleanup on Terrain Node Removal:** When a `VoxelTerrain`/`VoxelLodTerrain` node is removed from the scene tree, all nav regions must be freed in bulk. `NavMeshManager::clear_all()` handles this. See "Chunk Lifecycle Integration" section above.

**Editor UI:** Debug visualization of nav regions (Godot already supports navmesh debug drawing). Property inspector for Recast parameters on the VoxelNavBuilder node. Gizmos showing nav range. Toggle to enable/disable nav building in-editor.

**Custom rcContext:** Default no-op `rcContext` is fine initially. Later, subclass to forward `doLog()` to Godot's print functions for profiling and debugging the Recast pipeline.
