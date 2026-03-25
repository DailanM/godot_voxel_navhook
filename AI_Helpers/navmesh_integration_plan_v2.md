# Godot-Voxel Recast Navigation Integration — Plan v2

## Overview

This plan describes how to generate navigation meshes for smooth (Transvoxel) voxel terrain by feeding the triangle mesh already produced by the godot-voxel mesher into Recast's pipeline, then registering the result with Godot's NavigationServer3D.

The core architectural decision is **lightly-coupled post-mesher dispatch**: at the end of `MeshBlockTask::run()` (on a worker thread), the collision triangle data is extracted and handed to a `NavMeshManager`. The manager caches per-chunk triangle data, tracks neighbor readiness, manages an obstacle registry, and dispatches `NavMeshBuildTask`s to the thread pool when a chunk and its neighbors are all available. Final NavigationServer3D calls happen on the main thread.

This approach:
- **Supports full threading** — nav data capture and Recast builds happen on worker threads; only NavigationServer3D calls touch the main thread.
- **Supports per-chunk incremental updates** — when terrain changes or obstacles move, only affected chunks are rebuilt.
- **Handles border overlap naturally** — chunks wait for neighbor data before building, so border geometry is always available.

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
                                        ├─ Cache triangle data
                                        ├─ Check neighbor readiness
                                        └─ If ready ──► push NavMeshBuildTask
                                                              │
NavMeshBuildTask::run()  ◄────────────────────────────────────┘
  ├─ Read cached triangles (chunk + neighbors)
  ├─ Read obstacle registry
  ├─ Recast pipeline (rasterize → filter → compact →
  │    erode → regions → contours → polymesh → detail)
  └─ Produce NavigationMesh
        │
        └──► apply_result() ─────────────────────────────►  region_set_navigation_mesh()
                                                             (via TimeSpreadTaskRunner)

Game Code (main thread):
  NavMeshManager::add_obstacle() ──► mark affected chunks dirty ──► push NavMeshBuildTask
  NavMeshManager::remove_obstacle() / update_obstacle_transform()  (same pattern)
```

---

## NavMeshManager

Central coordinator for all navmesh state. Owned by the terrain node (or a sibling node). A `shared_ptr<NavMeshManager>` is passed through `MeshingDependency` so worker threads can reach it.

```cpp
class NavMeshManager {
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

    // Called from NavMeshBuildTask::apply_result() via TimeSpreadTaskRunner.
    void apply_nav_result(Vector3i chunk_pos, Ref<NavigationMesh> nav_mesh);

    // Cleanup when chunks leave nav range or terrain node is removed.
    void remove_chunk(Vector3i chunk_pos);
    void clear_all();

    // --- Configuration (set before use, typically from editor properties) ---
    rcConfig recast_config;         // Recast build parameters
    float nav_range = 128.0f;      // Independent from mesh loading range
    bool filter_low_hanging = true;
    bool filter_ledge_spans = true;
    bool filter_low_height_spans = true;

private:
    // Per-chunk cached triangle data. Written by worker threads after meshing,
    // read by NavMeshBuildTasks. Protected by mutex (writes are infrequent —
    // only when a chunk mesh is built or rebuilt).
    struct NavChunkData {
        StdVector<Vector3f> positions;  // collision subset [0..submesh_vertex_end)
        StdVector<int32_t> indices;     // collision subset [0..submesh_index_end)
        Vector3i chunk_position;
        AABB world_aabb;
    };
    Mutex _cache_mutex;
    HashMap<Vector3i, NavChunkData> _chunk_cache;

    // Obstacle registry. Modified on main thread, read (under lock) by build tasks.
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

    // Helpers
    bool _are_neighbors_ready(Vector3i chunk_pos);
    void _dispatch_nav_build(Vector3i chunk_pos);
    StdVector<Vector3i> _get_affected_chunks(const AABB &aabb);
};
```

### Key Behaviors

**`on_mesh_built()`** (worker thread):
1. Lock `_cache_mutex`, store/replace `NavChunkData` for this chunk position.
2. Check if this chunk and its axis neighbors (6 or up to 26 depending on border config) all have cached data.
3. If ready, push a `NavMeshBuildTask` via `VoxelEngine::push_async_task()`.
4. Also check if any neighbors were waiting on *this* chunk — if a neighbor's readiness just became complete, dispatch its build too.

**`add_obstacle()` / `remove_obstacle()`** (main thread):
1. Update the obstacle registry.
2. Compute which chunks overlap the obstacle's AABB.
3. Mark those chunks dirty and dispatch `NavMeshBuildTask`s for any that have cached triangle data.

**`apply_nav_result()`** (main thread, via TimeSpreadTaskRunner):
1. If no region RID exists for this chunk, create one via `NavigationServer3D::region_create()`.
2. Call `region_set_navigation_mesh()` with the new data.

**`remove_chunk()`** (main thread):
1. Free the region RID via `NavigationServer3D::free()`.
2. Remove from `_chunk_cache` (under lock).

---

## MeshBlockTask Hook Point

At the end of `MeshBlockTask::run()`, after `build_mesh()` completes, a small block extracts collision triangle data and forwards it to the NavMeshManager. The nav_mesh_manager pointer is nullable — if null, no nav work happens, keeping the coupling optional.

```cpp
// In MeshBlockTask::run(), after build_mesh():
if (meshing_dependency->nav_mesh_manager != nullptr && lod_index == 0) {
    NavMeshManager::NavChunkData nav_data;
    nav_data.chunk_position = mesh_block_position;

    // Extract collision-relevant subset from first surface
    const VoxelMesher::Output &out = _surfaces_output;
    if (out.surfaces.size() > 0 && out.collision_surface.submesh_vertex_end > 0) {
        const PackedVector3Array verts = out.surfaces[0].arrays[Mesh::ARRAY_VERTEX];
        const PackedInt32Array idxs = out.surfaces[0].arrays[Mesh::ARRAY_INDEX];
        int vert_end = out.collision_surface.submesh_vertex_end;
        int idx_end = out.collision_surface.submesh_index_end;

        nav_data.positions.resize(vert_end);
        for (int i = 0; i < vert_end; i++) {
            const Vector3 &v = verts[i];
            nav_data.positions[i] = Vector3f(v.x, v.y, v.z);
        }
        nav_data.indices.assign(idxs.ptr(), idxs.ptr() + idx_end);
    }

    meshing_dependency->nav_mesh_manager->on_mesh_built(std::move(nav_data));
}
```

**Notes:**
- Only LOD 0 chunks produce nav data (higher LODs are too coarse for accurate navigation).
- The `submesh_vertex_end` / `submesh_index_end` bounds exclude transition mesh vertices used for LOD stitching — these must NOT be included in nav generation.
- In double-precision builds, `Vector3` uses `real_t = double` but Recast expects `float`. The explicit `Vector3f` conversion handles this.

---

## NavMeshBuildTask

An `IThreadedTask` that runs the full Recast pipeline for a single chunk.

```cpp
class NavMeshBuildTask : public IThreadedTask {
public:
    // Input (set before dispatch, immutable during run)
    Vector3i chunk_position;
    NavMeshManager::NavChunkData chunk_triangles;        // this chunk
    StdVector<NavMeshManager::NavChunkData> neighbor_triangles; // border overlap
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
};
```

### `run()` — Recast Pipeline

**Step 1: Setup heightfield**

```cpp
rcContext recast_ctx;

// Compute bounds from chunk world position, expanded by borderSize
// (chunk_position * chunk_size_world gives bmin; + chunk_size gives bmax)
// Expand by cfg.borderSize * cfg.cs on X/Z
rcCalcGridSize(cfg.bmin, cfg.bmax, cfg.cs, &cfg.width, &cfg.height);

rcHeightfield *hf = rcAllocHeightfield();
rcCreateHeightfield(&recast_ctx, *hf, cfg.width, cfg.height,
                    cfg.bmin, cfg.bmax, cfg.cs, cfg.ch);
```

**Step 2: Rasterize terrain triangles**

Rasterize this chunk's triangles plus neighbor triangles that fall within the expanded bounds.

```cpp
auto rasterize = [&](const NavChunkData &data) {
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
rcBuildContours(&recast_ctx, *chf, cfg.maxSimplificationError,
                cfg.maxEdgeLen, *cset);

rcPolyMesh *pmesh = rcAllocPolyMesh();
rcBuildPolyMesh(&recast_ctx, *cset, cfg.maxVertsPerPoly, *pmesh);

rcPolyMeshDetail *dmesh = rcAllocPolyMeshDetail();
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
    if (nav_mesh_manager && result_nav_mesh.is_valid()) {
        // Queued through TimeSpreadTaskRunner for time-budgeted main thread work
        nav_mesh_manager->apply_nav_result(chunk_position, result_nav_mesh);
    }
}
```

---

## Recast Configuration

Default parameters, exposed as properties on a VoxelNavBuilder node or on the NavMeshManager:

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

## Border Geometry and Neighbor Readiness

### The Problem

Recast's `borderSize` expansion means each chunk's heightfield extends into neighboring chunks. Without neighbor geometry in the overlap zone, `rcErodeWalkableArea` incorrectly erodes edges at chunk boundaries, creating gaps.

### How Approach B Solves This

The NavMeshManager's per-chunk triangle cache is the key enabler:

1. When `on_mesh_built()` is called for chunk A, the triangle data is cached.
2. Before dispatching a NavMeshBuildTask for A, the manager checks that all neighbors within `borderSize * cs` distance also have cached data.
3. If not all neighbors are ready, nothing happens — the chunk simply waits.
4. When the final neighbor's mesh completes, its `on_mesh_built()` call triggers readiness checks for itself *and* all its neighbors, dispatching builds for everything that's now ready.

On initial load, all chunks mesh concurrently. Nav builds begin as soon as local neighborhoods complete — no global barrier needed. Interior chunks (surrounded by already-meshed chunks) build first; edge chunks build as their neighbors arrive.

### NavMeshBuildTask Data Assembly

When dispatching, the manager snapshots the relevant triangle data:

```cpp
void NavMeshManager::_dispatch_nav_build(Vector3i chunk_pos) {
    auto *task = new NavMeshBuildTask();
    task->chunk_position = chunk_pos;
    task->cfg = recast_config;

    // Copy this chunk's triangles
    task->chunk_triangles = _chunk_cache[chunk_pos]; // copy

    // Copy neighbor triangles that fall within the border overlap zone
    // (typically 6 axis neighbors, possibly up to 26 for corner overlap)
    for (Vector3i offset : neighbor_offsets) {
        auto it = _chunk_cache.find(chunk_pos + offset);
        if (it != _chunk_cache.end()) {
            task->neighbor_triangles.push_back(it->second); // copy
        }
    }

    // Snapshot overlapping obstacles
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
    VoxelEngine::get_singleton().push_async_task(task);
}
```

### Consistency

All data passed to the build task is **copied at dispatch time**, so the task is fully self-contained. The cache and obstacle registry can continue to be updated by other threads without affecting in-flight builds. If a chunk's data changes while a build is in flight, the result will be for the old data — the chunk will simply be rebuilt again with the new data.

---

## Region Registration and Edge Stitching

### One Region Per Chunk

Each LOD 0 chunk within nav range maps to one `NavigationServer3D` region. Region RIDs are stored in `NavMeshManager::_region_rids`.

```cpp
void NavMeshManager::apply_nav_result(Vector3i chunk_pos, Ref<NavigationMesh> nav_mesh) {
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
3. Dispatch `NavMeshBuildTask`s for dirty chunks that have cached triangle data.

For obstacle moves, both the old and new positions' chunks are marked dirty.

---

## Build System

The godot-voxel module needs access to Recast headers. Godot compiles Recast as part of the `navigation_3d` module. The include path `thirdparty/recastnavigation/Recast/Include/` must be added to godot-voxel's `SCsub`.

The Recast object files compiled by the navigation module should be linkable from godot-voxel without recompilation. If not, the Recast sources may need to be added to godot-voxel's build directly.

---

## Interface Points Summary

| Interface | From | To | Data |
|-----------|------|----|------|
| **1. Mesh capture** | `MeshBlockTask::run()` | `NavMeshManager::on_mesh_built()` | Collision vertices/indices, chunk position |
| **2. Nav build dispatch** | `NavMeshManager` | `NavMeshBuildTask` (thread pool) | Cached triangles (chunk + neighbors), obstacle snapshot, rcConfig |
| **3. Terrain rasterization** | `NavMeshBuildTask` | Recast | `float*` vertices, `int*` indices → `rcHeightfield` |
| **4. Obstacle rasterization** | `NavMeshBuildTask` | Recast | Obstacle mesh vertices/indices → same `rcHeightfield` |
| **5. Recast → NavigationMesh** | `NavMeshBuildTask` | Godot | `rcPolyMeshDetail` → `NavigationMesh` (reversed winding) |
| **6. Region registration** | `NavMeshBuildTask::apply_result()` | `NavigationServer3D` | `region_set_navigation_mesh()` on main thread |
| **7. Obstacle management** | Game code | `NavMeshManager` | `add/remove/update_obstacle()` → dirty chunks → rebuild |

---

## Future Improvements

These items are noted for later consideration and are not part of the initial implementation scope.

**Performance — Projected Area Marking:** Expose a collection of convex polygon footprints passed to `rcMarkConvexPolyArea()` on the `rcCompactHeightfield`. This is a lighter-weight alternative to full mesh rasterization for simple vertical obstacles (walls, fences), and can be used alongside the full rasterization approach. Could speed up rebuilds for common obstacle types.

**LOD-Aware Navigation:** Currently only LOD 0 chunks produce navmesh data. For large worlds, coarser LODs could provide approximate long-range pathfinding while LOD 0 handles detailed local navigation. The nav range must be <= the LOD 0 loading range.

**Error Handling:** The Recast pipeline can fail at various stages (allocation failure, empty input, degenerate geometry from empty/near-empty chunks). Needs `ERR_FAIL_COND` guards throughout the build pipeline.

**Debouncing Rapid Edits:** When voxels are edited rapidly (painting terrain), the mesher already re-meshes affected chunks. The nav builder should debounce to avoid excessive Recast rebuilds — e.g., a short delay after the last edit before dispatching a nav build, or cancelling in-flight builds for chunks that have been re-dirtied.

**Navigation Range Configuration:** The nav range should be configurable independently from terrain mesh loading range. Players may need navigation within 64m even if terrain is visible at 256m.

**Cleanup on Terrain Node Removal:** When a `VoxelTerrain`/`VoxelLodTerrain` node is removed from the scene tree, all nav regions must be freed in bulk. `NavMeshManager::clear_all()` handles this.

**Editor UI:** Debug visualization of nav regions (Godot already supports navmesh debug drawing). Property inspector for Recast parameters on the VoxelNavBuilder node. Gizmos showing nav range. Toggle to enable/disable nav building in-editor.

**Custom rcContext:** Default no-op `rcContext` is fine initially. Later, subclass to forward `doLog()` to Godot's print functions for profiling and debugging the Recast pipeline.
