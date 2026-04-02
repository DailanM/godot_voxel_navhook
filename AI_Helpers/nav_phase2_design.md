# NavMesh Phase 2: Boundary Stitching and Connected Components

## Motivation

Profiling confirms that Godot's NavigationServer edge merging is bottlenecked by **Phase 2** (O(F^2) brute-force margin matching on free/unmatched edges). See `navigation_server_analysis.md` for details.

The fix is to ensure boundary edges between adjacent chunk navmeshes share exact vertex positions, so that they match in the server's O(E) quantized HashMap phase and never enter the O(F^2) path.

A second benefit: once we have boundary connectivity, computing connected components across chunks is nearly free. This enables the region API proposed in `region_navigation_api.md`.

## Overview

After the existing Recast build (Phase 1) produces a per-chunk `NavigationMesh`, a new **Phase 2 task** runs that:

1. Identifies boundary edges of each chunk's navmesh.
2. Matches boundary edges between adjacent chunks by snapping shared vertices to canonical positions.
3. Rewrites the navmesh vertices so matched edges share identical coordinates.
4. Computes connected components across the stitched chunk group.
5. Stores component IDs and unmerged boundary edges for the region API.

Phase 2 runs as a separate `IThreadedTask` and produces a modified `NavigationMesh` plus component metadata. The existing `apply_nav_result()` path is reused to register the stitched mesh with the server.

---

## Readiness Model

Phase 2 for chunk C requires:
- C's Phase 1 navmesh is complete.
- All 6 axis neighbors of C have completed Phase 1.

This is the same neighbor readiness check already used by the Phase 1 dispatch, so the infrastructure exists. The difference is that Phase 2 operates on **navmesh output** (vertices + polygons), not raw triangles.

### Data Flow

```
Phase 1 (existing)                    Phase 2 (new)                      Main Thread
──────────────────                    ─────────────                      ───────────
NavMeshBuildTask::run()
  Recast pipeline
  → produces NavigationMesh
  → stores in NavMeshManager
    chunk navmesh cache
                                      NavStitchTask::run()
                                        reads chunk + neighbor navmeshes
                                        identifies boundary edges
                                        snaps shared vertices
                                        computes connected components
                                        → produces stitched NavigationMesh
                                          + component metadata
                                                │
                                                └──► apply_result() ──► region_set_navigation_mesh()
                                                                        + store component data
```

### Cache Addition

`NavMeshManager` gains a new cache alongside the existing triangle cache:

```cpp
struct NavMeshCacheEntry {
    Ref<NavigationMesh> nav_mesh;      // Phase 1 output (chunk-local coords)
    PackedVector3Array vertices;        // cached copy for Phase 2 reads
    Vector<PackedInt32Array> polygons;  // cached copy for Phase 2 reads
    uint32_t generation = 0;
};
HashMap<Vector3i, NavMeshCacheEntry> _navmesh_cache;  // under _cache_mutex
```

Phase 1's `apply_result()` is split: instead of immediately registering with the server, it stores the navmesh in `_navmesh_cache` and triggers Phase 2 dispatch (same neighbor-readiness pattern). Phase 2's `apply_result()` does the actual server registration.

---

## Algorithm: Boundary Edge Identification

A boundary edge is an edge of a navmesh polygon that lies on a face of the chunk's AABB. Since navmeshes are in chunk-local coordinates (origin at chunk corner), boundary edges are identified by vertex position:

```
Given chunk_size (e.g., 16):
  -x face: vertex.x ≈ 0
  +x face: vertex.x ≈ chunk_size
  -y face: vertex.y ≈ 0
  +y face: vertex.y ≈ chunk_size
  -z face: vertex.z ≈ 0
  +z face: vertex.z ≈ chunk_size
```

An edge is a boundary edge for face F if **both** vertices are within tolerance of face F. The tolerance should be `cell_size` (the Recast quantization step) to account for Recast's grid snapping.

### Edge Cases: Edges and Corners

A vertex can lie on multiple chunk faces simultaneously:

- **Face vertex**: on exactly 1 face (e.g., x=0, 0<y<16, 0<z<16). Shared with 1 neighbor.
- **Edge-of-chunk vertex**: on exactly 2 faces (e.g., x=0, z=0, 0<y<16). Shared with 3 neighbors (the two face-adjacent chunks plus the edge-diagonal chunk).
- **Corner vertex**: on 3 faces (e.g., x=0, y=0, z=0). Shared with 7 neighbors.

For Phase 2, we only need to match with the 6 axis neighbors. An edge-of-chunk or corner vertex will be snapped to a canonical position that is consistent across all chunks that share it, because the canonical position is computed from the vertex's world-space coordinates (see below), not from any particular chunk's perspective.

**A boundary edge between chunks A and B** is an edge where both vertices lie on the shared face between A and B. Edges where one vertex is on the shared face and the other is on a perpendicular face (i.e., the edge runs along a chunk edge/corner) still count — what matters is that both vertices are on (or within tolerance of) the plane of the shared face.

---

## Algorithm: Vertex Matching and Snapping

### Canonical Position via Lexicographic Tie-Breaking

When two vertices from adjacent chunks should be merged, we need a deterministic rule for which position wins. We use **lexicographic ordering on world-space coordinates**: the vertex with the smaller (x, y, z) tuple in lexicographic order provides the canonical position.

```cpp
// Canonical vertex selection
Vector3 canonical_position(Vector3 world_a, Vector3 world_b) {
    // Lexicographic comparison: x first, then y, then z
    if (world_a.x < world_b.x) return world_a;
    if (world_a.x > world_b.x) return world_b;
    if (world_a.y < world_b.y) return world_a;
    if (world_a.y > world_b.y) return world_b;
    if (world_a.z < world_b.z) return world_a;
    return world_b;
}
```

In practice, since Recast quantizes to a grid (`cell_size` × `cell_height`), matching vertices from adjacent chunks will often have identical coordinates on the shared face plane and differ only in the perpendicular axis (by floating point noise from independent Recast runs). The canonical selection resolves this noise deterministically.

### Matching Process

For each pair of adjacent chunks (C, N) sharing face F:

1. Collect boundary vertices of C on face F → set `V_C`.
2. Collect boundary vertices of N on face F → set `V_N` (converted to C's local coordinate space, or both converted to world space).
3. For each vertex `v_c` in `V_C`, find the closest vertex `v_n` in `V_N` within tolerance `cell_size`.
4. If matched: compute canonical world position from the pair. Convert back to each chunk's local space and update both chunks' vertex arrays.

**Spatial lookup**: Since vertices are on a shared plane, project them onto the 2D plane of face F and use a grid hash (cell = `cell_size`) for O(1) average lookup. This keeps matching O(V) per face rather than O(V^2).

### Implementation Sketch

```cpp
struct BoundaryVertex {
    int vertex_index;       // index in the chunk's vertex array
    Vector3 world_position; // chunk-local + chunk_origin
    int face;               // which chunk face (0-5 for ±x, ±y, ±z)
};

// For chunk C and neighbor N sharing face F:
// 1. Build a spatial hash of N's boundary vertices on face F
//    Key: quantized 2D position on the face plane
//    Value: list of BoundaryVertex

// 2. For each boundary vertex of C on face F:
//    - Look up in N's spatial hash
//    - If match found within cell_size tolerance:
//      - Compute canonical position (lexicographic min of world coords)
//      - Record: (C_vertex_index → canonical_pos, N_vertex_index → canonical_pos)

// 3. Apply all recorded snaps to both vertex arrays
```

### Quantization Alignment with NavigationServer

The server's `PointKey` quantizes via `floor(pos / merge_cell)` where `merge_cell = cell_size * merge_rasterizer_cell_scale` (default 0.1). For two vertices to hash identically, they must land in the same bucket. After snapping to the canonical position, both chunks' vertices at that location are bitwise identical, guaranteeing they produce the same `PointKey` and `EdgeKey`. This eliminates Phase 2 (margin matching) in the server entirely.

---

## Algorithm: Connected Components

Once boundary edges are matched, we can compute connected components across the chunk neighborhood. This is a union-find over navmesh polygons:

### Per-Chunk Internal Connectivity (computed once per Phase 1 output)

Within a single chunk's navmesh, two polygons are connected if they share an edge (both internal edges identified during Phase 1, and boundary edges that have been matched to a neighbor). Standard union-find on polygon indices.

### Cross-Chunk Connectivity (computed in Phase 2)

A matched boundary edge connects a polygon in chunk C to a polygon in chunk N. The union-find merges across chunk boundaries.

### Algorithm

```
1. Initialize union-find with all polygons from chunk C
   (only C's components are the output — neighbors are context)

2. For each internal edge of C (edge shared by two polygons within C):
   union(poly_a, poly_b)

3. For each matched boundary edge between C and neighbor N:
   union(C_poly, N_poly)
   (N_poly participates in the union-find but its component ID
    is not part of C's output)

4. Extract components: group C's polygons by their root.
   Assign each group a component ID.
```

### Component ID Stability

Component IDs are local to a chunk and regenerated on each Phase 2 run. They are **not** stable across rebuilds. The `region_navigation_api.md` spec states: "Region IDs are globally unique and stable for the lifetime of the chunk (they are invalidated when a chunk's navmesh is regenerated)." This is satisfied — IDs are stable until the chunk rebuilds, at which point downstream code should invalidate caches (a generation counter or signal can notify).

Global uniqueness is achieved by combining chunk position + local component index into a single int (e.g., pack chunk coords into upper bits, component index into lower bits), or by maintaining a global ID counter in `NavMeshManager`.

### Component Storage

```cpp
struct ChunkComponentData {
    // Per-polygon: which component does this polygon belong to?
    PackedInt32Array polygon_to_component;  // indexed by polygon index

    // Per-component: what are the unmerged boundary edges?
    // Stored as packed Vector3 pairs (start, end) in world space.
    HashMap<int, PackedVector3Array> component_boundary_edges;

    // Per-component: which neighbor components is this connected to?
    // (for building the region graph)
    HashMap<int, PackedInt32Array> component_neighbors;

    uint32_t generation = 0;
};
HashMap<Vector3i, ChunkComponentData> _component_cache;  // main thread only
```

---

## Unmerged Boundary Edges

After stitching, boundary edges fall into two categories:

1. **Matched/merged**: both vertices snapped to canonical positions, will merge in the server's HashMap phase. These are internal to the navigation graph.
2. **Unmerged**: boundary edges with no matching edge in the neighbor. These are the perimeter of navigable surface — cliff edges, gaps, terrain borders.

Unmerged boundary edges are stored per-component for the `get_region_boundary_edges()` API. They are identified as boundary edges of chunk C on face F that have no matching edge in neighbor N (either no vertex match, or only one of the two edge vertices matched).

---

## NavStitchTask

```cpp
class NavStitchTask : public IThreadedTask {
public:
    // Input
    Vector3i chunk_position;
    uint32_t build_generation;

    // Snapshot of chunk + neighbor navmeshes (chunk-local vertices + polygons)
    struct ChunkNavData {
        Vector3i position;
        PackedVector3Array vertices;
        Vector<PackedInt32Array> polygons;
    };
    ChunkNavData center_chunk;
    StdVector<ChunkNavData> neighbors;  // 6 axis neighbors

    // Config
    float cell_size;
    float cell_height;
    int mesh_block_size;

    // Output
    Ref<NavigationMesh> stitched_nav_mesh;  // center chunk with snapped vertices
    ChunkComponentData component_data;

    // Dependency
    std::shared_ptr<NavMeshManager> nav_mesh_manager;

    void run(ThreadedTaskContext &ctx) override;
    void apply_result() override;
    TaskPriority get_priority() override;
    bool is_cancelled() override;
    const char *get_debug_name() const override { return "NavStitch"; }
};
```

### `run()` Outline

```
1. For each of the 6 axis neighbors:
   a. Identify boundary vertices of center chunk on the shared face
   b. Identify boundary vertices of neighbor on the shared face
   c. Build spatial hash of neighbor boundary verts (2D, on face plane)
   d. Match center boundary verts against neighbor hash
   e. For matched pairs: compute canonical position, record snap
   f. Record matched edges (for cross-chunk connectivity)
   g. Record unmatched boundary edges (for region API)

2. Apply all vertex snaps to center chunk's vertex array
   (produce a new PackedVector3Array with modified positions)

3. Build union-find over center chunk polygons:
   a. Union polygons sharing internal edges
   b. Union polygons connected via matched boundary edges to neighbor polygons
   c. Extract component assignments for center chunk polygons

4. For each component: collect its unmerged boundary edges

5. Build stitched_nav_mesh from snapped vertices + original polygons

6. Package component_data
```

### `apply_result()`

```cpp
void NavStitchTask::apply_result() {
    if (!nav_mesh_manager || !nav_mesh_manager->valid) return;
    if (!stitched_nav_mesh.is_valid()) return;

    // Register stitched mesh with NavigationServer (replaces Phase 1's direct registration)
    nav_mesh_manager->apply_stitched_result(
        chunk_position, stitched_nav_mesh, component_data, build_generation);
}
```

---

## Dispatch Flow

### Modified Phase 1 Flow

Phase 1 (`NavMeshBuildTask`) no longer registers with the server directly. Instead:

```
NavMeshBuildTask::apply_result()
  → nav_mesh_manager->cache_nav_result(chunk_pos, nav_mesh, generation)
    → stores in _navmesh_cache
    → calls _try_dispatch_stitch(chunk_pos)
    → calls _try_dispatch_stitch(neighbor) for each neighbor
```

### Phase 2 Dispatch

```cpp
void NavMeshManager::_try_dispatch_stitch(Vector3i chunk_pos) {
    // Must be called on main thread (navmesh cache is main-thread only)

    // Check center chunk has Phase 1 result
    if (!_navmesh_cache.has(chunk_pos)) return;

    // Check all 6 axis neighbors have Phase 1 results
    for (int i = 0; i < 6; i++) {
        if (!_navmesh_cache.has(chunk_pos + neighbor_offsets[i])) return;
    }

    // Check within nav range
    if (!_is_within_nav_range(chunk_pos)) return;

    _dispatch_stitch(chunk_pos, _navmesh_cache[chunk_pos].generation);
}

void NavMeshManager::_dispatch_stitch(Vector3i chunk_pos, uint32_t generation) {
    auto *task = ZN_NEW(NavStitchTask);
    task->chunk_position = chunk_pos;
    task->build_generation = generation;
    task->cell_size = recast_config.cs;
    task->cell_height = recast_config.ch;
    task->mesh_block_size = mesh_block_size;

    // Snapshot center chunk navmesh
    const auto &center = _navmesh_cache[chunk_pos];
    task->center_chunk = { chunk_pos, center.vertices, center.polygons };

    // Snapshot neighbor navmeshes
    for (int i = 0; i < 6; i++) {
        Vector3i npos = chunk_pos + neighbor_offsets[i];
        const auto &nbr = _navmesh_cache[npos];
        task->neighbors.push_back({ npos, nbr.vertices, nbr.polygons });
    }

    task->nav_mesh_manager = shared_from_this();
    VoxelEngine::get_singleton().push_async_task(task);
}
```

### Generation Tracking

The generation counter from Phase 1 propagates through Phase 2. If a chunk's Phase 1 rebuilds while a Phase 2 task is in flight, the stale Phase 2 result is discarded by the same generation check in `apply_stitched_result()`.

---

## Incremental Updates

When a single chunk C is re-meshed (terrain edit):

1. Phase 1 rebuilds C's navmesh (existing behavior).
2. Phase 1 result is cached, triggering Phase 2 dispatch for C and all neighbors of C.
3. Phase 2 for C: re-stitches C with its (unchanged) neighbors. Fast — only C's boundary edges are recomputed.
4. Phase 2 for C's neighbors: each neighbor re-stitches with C's new navmesh. Also fast — only the face shared with C needs re-matching.

Total Phase 2 work for a single chunk edit: up to 7 stitch tasks (C + 6 neighbors), each processing one chunk's boundary edges. This is proportional to the boundary, not the total navmesh.

---

## Region API Implementation

With component data stored, the three API functions from `region_navigation_api.md` are straightforward:

### `get_regions_in_chunk(chunk_position)`
Look up `_component_cache[chunk_position]`, return the set of unique component IDs.

### `get_region_for_point(position)`
1. Compute which chunk the point falls in.
2. Find the nearest polygon in that chunk's navmesh (spatial query on the triangle soup).
3. Return `polygon_to_component[polygon_index]` from the chunk's component data.

### `get_region_boundary_edges(region_id, neighborhood)`
1. Decode chunk position from `region_id` (or search chunks in the AABB).
2. Return `component_boundary_edges[local_component_id]` filtered to the AABB.

---

## Summary of New/Modified Files

| File | Change |
|------|--------|
| `terrain/navigation/nav_stitch_task.h/.cpp` | **New** — Phase 2 task |
| `terrain/navigation/nav_mesh_manager.h` | Add `_navmesh_cache`, `_component_cache`, stitch dispatch methods, region API methods |
| `terrain/navigation/nav_mesh_manager.cpp` | Implement stitch dispatch, `cache_nav_result()`, `apply_stitched_result()`, region API |
| `terrain/navigation/nav_mesh_build_task.cpp` | Modify `apply_result()` to cache instead of register directly |
| `terrain/fixed_lod/voxel_terrain.h/.cpp` | Bind region API methods |

---

## Open Questions

1. **Should Phase 2 run on worker threads or main thread?** The stitching work (vertex matching + union-find) is lightweight per-chunk. Running on a worker thread avoids any main-thread cost but adds a frame of latency. Running on main thread during `apply_result()` is simpler but blocks. **Recommendation: worker thread** — keeps the main thread clean and the latency is acceptable since Phase 1 already has latency.

2. **Should we stitch corners (26-neighborhood)?** The current design only matches with 6 axis neighbors. A vertex on a chunk edge (shared by 3+ chunks) gets snapped independently by each pair. Since the canonical position is deterministic (lexicographic world-space), all pairs produce the same result, so corner consistency is maintained without explicit 26-neighbor handling. This should be verified empirically.

3. **Component ID encoding.** The simplest approach: global atomic counter in `NavMeshManager`. Each Phase 2 result gets fresh IDs. Alternatively, pack `(chunk_pos_hash, local_index)` for spatial locality. The choice affects downstream cache invalidation patterns.
