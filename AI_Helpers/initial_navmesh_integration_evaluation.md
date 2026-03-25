# Navmesh Integration Draft — Technical Evaluation

This document evaluates the feasibility of the plan described in `navmesh_integration_draft.md`, identifies gaps and issues, and provides recommendations based on a thorough reading of the Recast library (as bundled in Godot 4.6), the godot-voxel module source, and Godot's NavigationServer3D internals.

**Verdict: The plan is fundamentally sound and feasible.** The Recast API, godot-voxel mesher output, and Godot NavigationServer3D all support the proposed integration. There are several concrete issues that need to be addressed before implementation, detailed below.

---

## 1. Critical Issue: rcPolyMesh vs rcPolyMeshDetail Conversion

**The draft proposes converting `rcPolyMesh` directly to `NavigationMesh`. Godot's own baker uses `rcPolyMeshDetail` instead.**

In `nav_mesh_generator_3d.cpp` (lines 534-577), Godot iterates `detail_mesh->nmeshes`, extracting triangles from `detail_mesh->tris` and vertices from `detail_mesh->verts` (which are `float*` in world space), deduplicates vertices, and feeds triangle polygons (always 3-sided) to `NavigationMesh::set_data()`.

The draft's approach of using `rcPolyMesh` directly is *not wrong* — convex polygons with up to `nvp` vertices are a valid representation and NavigationMesh::add_polygon() accepts arbitrary convex polygons. However, there are tradeoffs:

| Approach | Pros | Cons |
|----------|------|------|
| **rcPolyMesh** (draft) | Fewer polygons, more efficient pathfinding | Vertices are `unsigned short` (voxel-space), require manual conversion to world-space floats. Lower vertical accuracy. |
| **rcPolyMeshDetail** (Godot) | Higher vertical accuracy, vertices already in `float` world-space. Proven working approach. | More triangles = more polygons in the navmesh. |

**Key detail the draft gets right but is worth highlighting:** `rcPolyMesh::verts` are `unsigned short*` — they are quantized voxel-space coordinates. The conversion formula `bmin + v * cs/ch` in the draft is correct:
```
x = pmesh->bmin[0] + v[0] * pmesh->cs
y = pmesh->bmin[1] + v[1] * pmesh->ch
z = pmesh->bmin[2] + v[2] * pmesh->cs
```

**Recommendation:** Start with the `rcPolyMeshDetail` approach (matching Godot's proven path) for correctness. The `rcPolyMesh` approach can be evaluated later as an optimization if polygon count becomes a concern for pathfinding performance.

**Important vertex winding note:** Godot's baker reverses triangle winding order — index order is `[0, 2, 1]` not `[0, 1, 2]` (see `nav_mesh_generator_3d.cpp:565-567`). This will need to be replicated:
```cpp
nav_indices.write[0] = recast_index_to_native_index[index1];
nav_indices.write[1] = recast_index_to_native_index[index2]; // detail_mesh_tris[j*4 + 2]
nav_indices.write[2] = recast_index_to_native_index[index3]; // detail_mesh_tris[j*4 + 1]
```

---

## 2. Mesh Data Access: CollisionSurface vs Surface Arrays

The draft says we need vertex positions as `float[]` and triangle indices as `int[]`. The godot-voxel mesher provides these in two forms:

### Option A: `VoxelMesher::Output::CollisionSurface`
```cpp
struct CollisionSurface {
    StdVector<Vector3f> positions;  // float x,y,z triples
    StdVector<int> indices;
    int32_t submesh_vertex_end = -1;  // collision uses subset of render mesh
    int32_t submesh_index_end = -1;
};
```

**Problem:** The Transvoxel mesher does NOT populate `CollisionSurface::positions` or `CollisionSurface::indices` directly. Instead, it sets `submesh_vertex_end` and `submesh_index_end` as pointers into the *render* mesh arrays. The actual collision geometry is extracted later from the first surface's `ARRAY_VERTEX` and `ARRAY_INDEX` up to these bounds. This means we can't just use `collision_surface` directly.

### Option B: Surface Arrays (render mesh)
```cpp
struct Surface {
    Array arrays;  // Godot Array containing ARRAY_VERTEX (PackedVector3Array), ARRAY_INDEX (PackedInt32Array), etc.
    uint16_t material_index = 0;
};
```

The first surface's `arrays[Mesh::ARRAY_VERTEX]` (as `PackedVector3Array`) and `arrays[Mesh::ARRAY_INDEX]` (as `PackedInt32Array`) contain the full rendering mesh data. The collision portion is the subset `[0..submesh_vertex_end)` / `[0..submesh_index_end)`.

### Option C: Internal transvoxel::MeshArrays (best for threading)

The internal `transvoxel::MeshArrays` struct has `StdVector<Vector3f> vertices` and `StdVector<int> indices` — raw C++ vectors that are ideal for feeding to Recast. However, these are consumed during `fill_surface_arrays()` to produce Godot `Array` objects, and are not retained in the output.

**Option C detail:** The Transvoxel mesher exposes thread-local cache access:
```cpp
// In voxel_mesher_transvoxel.h — only valid on the thread that just called build()
static const transvoxel::MeshArrays &get_mesh_cache_from_current_thread();
```
This provides direct access to `StdVector<Vector3f> vertices` and `StdVector<int32_t> indices` before they're copied into Godot arrays. However, this is only valid until the next `build()` call on that thread, so data must be copied immediately.

**Recommendation:** For a first pass, extract from the Godot Surface arrays (Option B) after meshing. For performance, modify the mesher to retain or copy the raw collision vertices/indices alongside the existing output (Option C style). The raw `float*` and `int*` are what Recast expects — `rcRasterizeTriangles` takes `const float* verts` and `const int* tris`.

**Collision submesh bounds:** For Transvoxel, the collision-relevant portion of the mesh is `[0..submesh_vertex_end)` vertices and `[0..submesh_index_end)` indices from the first surface. Vertices beyond these bounds are transition mesh vertices for LOD stitching and should NOT be included in navmesh generation.

**Data format compatibility with Recast:**
- Recast expects `float*` as `(x, y, z)` triples — Godot `Vector3` is also `{float x, y, z}`, so `PackedVector3Array.ptr()` can be cast to `const float*`.
- Recast expects `int*` as triangle index triples — `PackedInt32Array.ptr()` works directly as `const int*`.
- However, there's a type precision concern: godot-voxel uses `Vector3f` internally (32-bit float), and Godot's `Vector3` may use `real_t` (which is `double` in double-precision builds). This would need a conversion step in double-precision builds.

---

## 3. Hook Point: Where to Intercept Mesh Updates

The draft references a `mesh_update_notification` feature branch. Looking at the actual code, the natural hook point is `VoxelLodTerrain::apply_mesh_update()` (`voxel_lod_terrain.cpp:1831`).

This method:
1. Runs on the **main thread**
2. Receives `VoxelEngine::BlockMeshOutput` containing the mesher output, LOD index, and block position
3. Has access to `ob.surfaces` (the `VoxelMesher::Output`)
4. Is called for every chunk mesh create/update

**The actual callback chain is:**
1. `MeshBlockTask::run()` executes on worker thread (builds mesh)
2. `MeshBlockTask::apply_result()` fires on main thread
3. This invokes `mesh_output_callback` which queues an `ApplyMeshUpdateTask`
4. `ApplyMeshUpdateTask` is processed via `TimeSpreadTaskRunner` (main thread, time-budgeted)
5. This calls `apply_mesh_update()` with a `VoxelEngine::BlockMeshOutput` containing:
   - `position` (Vector3i mesh block coordinates)
   - `lod` (uint8_t LOD index)
   - `surfaces` (VoxelMesher::Output with triangle data)
   - `visual_was_required`, `has_mesh_resource`, etc.

**The key question: should nav building happen here (main thread) or be dispatched to a worker?**

The answer is clear: dispatch to a worker thread. The `apply_mesh_update` function should:
1. Extract/copy the triangle data needed for Recast (use `submesh_vertex_end`/`submesh_index_end` to exclude transition mesh vertices)
2. Queue a nav build task via `VoxelEngine::push_async_task()` (the existing async task system)
3. When the task completes, apply the result on the main thread via `push_main_thread_time_spread_task()` (calling NavigationServer3D)

This matches godot-voxel's existing threading model where meshing happens on workers and results are applied on the main thread.

**For `VoxelTerrain` (non-LOD fixed terrain):** A similar function `VoxelTerrain::apply_mesh_update()` exists in `terrain/fixed_lod/voxel_terrain.cpp`. Both terrain types will need the hook.

---

## 4. Border Geometry / Neighbor Overlap

The draft correctly identifies that `borderSize` expansion requires triangle geometry from neighboring chunks. This is one of the trickier implementation challenges.

**How the Transvoxel mesher handles neighbors:** The mesher already accesses neighbor voxel data via padding. The `VoxelMesher::Input` provides voxels with padding (`get_minimum_padding()` / `get_maximum_padding()`). But this padding is for *voxel meshing continuity*, not for navmesh border overlap, and is typically only 1-2 voxels.

**The navmesh border overlap is different and larger.** With `borderSize = walkableRadius + 3` and a typical `walkableRadius` of 2-3 cells, the border needs ~5-6 nav cells of overlap. At a nav cell size of 0.2m, this is only ~1.2m — but depending on terrain voxel size, this could span into 1-2 neighboring chunks.

**Options for obtaining border geometry:**

1. **Merge neighbor mesh data:** When building a chunk's navmesh, also request the triangle meshes from adjacent chunks and include triangles that fall within the expanded AABB. This is the most straightforward approach but requires neighbor meshes to be available (they may not be if the neighbor hasn't been meshed yet).

2. **Re-mesh the border region:** Use neighbor VoxelBuffer data with the Transvoxel mesher to generate geometry for just the border area. This is more complex but guarantees availability.

3. **Two-pass approach:** Build each chunk's navmesh without border data, then stitch at boundaries. This is simpler but produces worse results at boundaries (erosion artifacts).

**Recommendation:** Option 1 (merge neighbor meshes). Cache each chunk's collision-relevant triangle data so neighbors can access it. When a chunk's navmesh is built, check if all needed neighbors have mesh data available. If not, defer the build or build without the border (marking it for rebuild when neighbors become available).

---

## 5. Recast Filter Flags — Draft vs Godot's Approach

The draft enables all three filters unconditionally:
```cpp
rcFilterLowHangingWalkableObstacles(ctx, cfg.walkableClimb, *hf);
rcFilterLedgeSpans(ctx, cfg.walkableHeight, cfg.walkableClimb, *hf);
rcFilterWalkableLowHeightSpans(ctx, cfg.walkableHeight, *hf);
```

Godot's baker makes these **optional** via `NavigationMesh` properties:
```cpp
if (p_navigation_mesh->get_filter_low_hanging_obstacles()) { ... }
if (p_navigation_mesh->get_filter_ledge_spans()) { ... }
if (p_navigation_mesh->get_filter_walkable_low_height_spans()) { ... }
```

**Recommendation:** Expose these as configurable properties on the VoxelNavBuilder node, defaulting to all enabled (they're generally all wanted for terrain navigation). This gives users control for edge cases.

**Important filter ordering note from Recast.h:** `rcFilterLowHangingWalkableObstacles` will override the effect of `rcFilterLedgeSpans`, so if both are used, `rcFilterLedgeSpans` must be called AFTER `rcFilterLowHangingWalkableObstacles`. The draft's ordering is correct.

---

## 6. Obstacles: Draft's Approach vs Godot's ProjectedObstruction

The draft proposes an `ObstacleSource` registry with full mesh rasterization. Godot's own baker uses a different mechanism: `ProjectedObstruction`, which are convex polygon footprints projected vertically (2D outlines + height range), applied via `rcMarkConvexPolyArea()` on the `rcCompactHeightfield`.

**Key difference:** Godot applies obstacles *after* compacting and *after* erosion for "carve" obstacles, or *before* erosion for regular obstacles. The draft rasterizes obstacle meshes into the heightfield alongside terrain, which means they'll be affected by all subsequent filters including erosion.

Both approaches are valid, but they produce different results:
- **Draft's full rasterization:** More accurate — the actual obstacle mesh geometry creates solid spans, so Recast properly handles clearance around complex shapes. Better for 3D obstacles with overhangs.
- **Godot's projected approach:** Simpler, faster, works well for buildings/walls that can be represented as vertical extrusions.

**Recommendation:** The draft's approach is better for a voxel terrain game where obstacles may have complex 3D geometry. Keep the full rasterization approach. Consider *also* supporting projected obstructions as a lighter-weight option.

---

## 7. NavigationServer3D Thread Safety

The draft says: *"Only the final NavigationServer3D calls must happen on the main thread."*

This is correct. Looking at `NavigationServer3D`, all methods are virtual and the concrete implementation uses a command queue pattern. However, `region_set_navigation_mesh()` involves resource handling that should be done on the main thread for safety.

**Godot's own async baker (`bake_from_source_geometry_data_async`)** does the Recast pipeline on a worker thread via `WorkerThreadPool`, but applies the result through a sync callback on the main thread. Our integration should follow the same pattern.

---

## 8. Cell Size Synchronization — Potential Pitfall

The draft correctly notes that the nav map's cell size must match. There's a subtlety: `map_set_merge_rasterizer_cell_scale` exists in the NavigationServer3D API and controls how aggressively edges are merged. The default is typically adequate, but if edge stitching between chunks is problematic, this may need tuning.

Also noted: `NavigationMesh` has its own `cell_size` and `cell_height` properties (used during baking). Since we're bypassing the normal baking pipeline and setting vertices/polygons directly, we should still set these properties on the NavigationMesh resource to match our Recast cfg values — Godot may use them internally for debug visualization or edge matching.

---

## 9. Region Edge Stitching — Additional Details

The draft's approach of one region per chunk is sound. NavigationServer3D uses `edge_connection_margin` to match edges between regions. By default, edges within this margin distance are connected.

**Critical detail for tiled Recast builds:** Recast's `borderSize` parameter causes the heightfield to extend beyond the tile boundary, and then `rcBuildContours` clips contours to the tile boundary, producing vertices exactly on the tile edge. **However**, after clipping, the vertices are stored as `unsigned short` in `rcPolyMesh`, which quantizes positions. Two adjacent tiles will produce the *same* quantized boundary vertices only if they share the same `bmin` alignment and `cs`/`ch` values.

**Recommendation:** Ensure all chunks use a consistent coordinate grid for Recast (same `cs`, `ch`, and aligned `bmin` values that are multiples of `cs`). This should happen naturally if chunk positions are computed from integer chunk coordinates multiplied by chunk size.

If exact vertex matching at boundaries proves problematic, `map_set_edge_connection_margin` can be set to `cfg.cs * 0.5` as the draft suggests. This is a reasonable fallback.

---

## 10. LOD Considerations

The draft mentions: *"Only LOD 0 chunks within a configurable navigation range need regions."*

This is a good starting point but may be limiting for large worlds. Consider:

- **LOD 0** for detailed navigation near the player
- **Coarser LODs** for approximate long-range pathfinding (optional future enhancement)

For now, LOD 0 only is the right initial approach. The Transvoxel mesher output at LOD 0 has the highest resolution geometry and is the most suitable for accurate navigation.

**One concern with `VoxelLodTerrain`:** LOD 0 blocks may not be loaded at all distances. The navigation range should be <= the LOD 0 loading range. This is implicitly handled if we only hook into `apply_mesh_update` for `ob.lod == 0`.

---

## 11. Memory and Performance Considerations

### Heightfield Memory
For a chunk of size 16 voxels at 1m, with `cs = 0.2m`, the heightfield grid would be approximately 80x80 cells plus border. At ~24 bytes per span, this is manageable. But with many chunks being rebuilt simultaneously, memory should be pooled or limited.

### Recast Pipeline Cost
The full Recast pipeline for a single tile is typically fast (1-10ms depending on complexity). This is well within worker thread budget. The bottleneck will likely be the chunk-to-chunk stall waiting for neighbor meshes for border overlap.

### NavigationServer Region Count
Each chunk = one region. With a typical navigation range of 128m and 16m chunks, that's ~64 regions at LOD 0. NavigationServer3D handles this comfortably. At larger ranges or smaller chunks, this could become a concern.

---

## 12. `rcContext` Implementation

The draft doesn't address `rcContext`. Godot's baker uses a default `rcContext ctx;` (on the stack) with no custom logging — the base class has no-op implementations for all virtual methods.

**Recommendation:** Same approach. Use default `rcContext` for now. If profiling or debugging is needed later, subclass `rcContext` to forward `doLog` to Godot's print functions.

---

## 13. Include Path and Build System

Godot's `nav_mesh_generator_3d.cpp` includes Recast with `#include <Recast.h>` and the build system adds the include path `thirdparty/recastnavigation/Recast/Include/`. Since godot-voxel is a module, it will need to add this include path to its `SCsub` or `SConstruct`.

The Recast source files are compiled as part of the `navigation_3d` module. The godot-voxel module should be able to link against the same Recast object files without recompilation, by just including the header and relying on the already-compiled Recast library.

**Verification needed:** Check that the `navigation_3d` module's Recast objects are linked into the final binary in a way that's accessible to other modules. If not, the include path and source compilation may need to be added to godot-voxel's build.

---

## 14. Missing from the Draft

### 14a. Error Handling
The Recast pipeline can fail at various stages (allocation failure, empty input, degenerate geometry). Godot's baker uses `ERR_FAIL_COND` macros. The VoxelNavBuilder needs similar error handling, particularly since voxel terrain can produce degenerate meshes (empty chunks, single-triangle chunks, etc.).

### 14b. Dirty Region Tracking
The draft mentions obstacle add/remove marking chunks dirty, but doesn't detail how terrain edits propagate. When voxels are edited, godot-voxel already re-meshes affected chunks. The nav builder should respond to re-meshing (which it will if hooked into `apply_mesh_update`), but it should also debounce rapid edits to avoid excessive rebuilds.

### 14c. Navigation Range vs Mesh Range
The nav range should be configurable independently from the terrain mesh loading range. Players may need navigation only within, say, 64m even if terrain is visible at 256m. This is acknowledged in the draft but not detailed.

### 14d. Cleanup on Terrain Node Removal
When a `VoxelLodTerrain` node is removed from the scene tree, all nav regions must be freed. The draft mentions `free(region_rid)` on chunk unload but doesn't address bulk cleanup.

---

## Summary of Action Items

| Priority | Item | Effort |
|----------|------|--------|
| **High** | Switch to rcPolyMeshDetail conversion (matching Godot's approach) or validate rcPolyMesh approach thoroughly | Low |
| **High** | Determine mesh data extraction path (Surface arrays vs. adding raw data retention) | Medium |
| **High** | Design border geometry neighbor access system | High |
| **High** | Add Recast include path to godot-voxel build system | Low |
| **Medium** | Make filter flags configurable | Low |
| **Medium** | Implement debouncing for rapid terrain edits | Medium |
| **Medium** | Design nav range independent from mesh loading range | Medium |
| **Low** | Support for projected obstructions alongside full mesh rasterization | Medium |
| **Low** | LOD-aware navigation for long-range pathfinding | High |
| **Low** | Custom rcContext for debug logging | Low |

---

## Conclusion

The draft is a well-researched and technically accurate plan. The Recast pipeline integration is standard and well-proven. The main engineering challenges are:

1. **Data plumbing** — getting triangle data from godot-voxel's mesher output into Recast's expected format (solvable, multiple paths available)
2. **Border geometry** — ensuring neighbor chunk data is available for proper edge stitching (the hardest single problem)
3. **Threading** — dispatching Recast work to workers and applying results on main thread (well-understood pattern in godot-voxel)

None of these are blockers. The plan is ready to move to detailed design and implementation.
