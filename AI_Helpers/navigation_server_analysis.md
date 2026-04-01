# Navigation Server Performance Analysis

## Context

After confirming that Godot's NavigationServer is the source of performance issues (by toggling `nav_register_with_server` on VoxelTerrain), we researched known issues from Zylann's godot_voxel repo and Godot upstream.

## Zylann's Key Navigation Issues (godot_voxel #610)

1. **`NavMap::sync` runs on main thread and is very slow** — nearly as expensive as baking itself, stalls physics every time a navmesh changes
2. **O(n^2) edge connection scaling** — no bounds checking when connecting regions, so many chunk regions = quadratic cost (godot-proposals#9381)
3. **Edge connections between chunks are unreliable** — vertices must match exactly or within a tiny margin, so visually-touching navmeshes often don't connect
4. **Thin polygon sync errors** — Recast produces polygons that trigger `"Attempted to merge an already-merged edge"` errors (godot#85548)
5. **Geometry gathering forced onto main thread** — API assumes scene tree access
6. **Async bake disabled in editor** — hardcoded off

Zylann's nuclear option quote:
> "one extreme workaround that starts flying in my head is to implement a whole navigation query system entirely in the voxel module to be better tailored to streaming procedural open-worlds, and completely skip Godot's regions system... but that would be really, really sad"

## Godot Upstream Progress (as of 2026-03-30)

| Issue | Status | Notes |
|-------|--------|-------|
| Main thread sync bottleneck | **Largely fixed** | PR #100497 (merged Dec 2024) moves sync to background thread |
| Thin polygon errors (#85548) | **Fixed** | PR #87959 merged in Godot 4.3 |
| Custom geometry providers (#5138) | **Implemented** | PR #90876 merged Apr 2024 |
| O(n^2) edge connections (#9381) | **Still open** | No PR exists, just a proposal |
| Further sync improvements (#112908) | **Open PR** | Would eliminate command queue delays |

### Other relevant merged PRs:
- **#98866** — Reduce allocations for NavMap synchronization (Nov 2024)
- **#99646** — NavMap objects request sync only on demand (Nov 2024)
- **#101037** — NavMeshQueries use region iteration polygons directly (Jan 2025)
- **#93005** — Replace distance checks with square distance checks (Dec 2024)

## What We Already Sidestep with Our Recast Pipeline

Since we run Recast directly on worker threads:

- **Baking performance** — solved, we run Recast on worker threads via `NavMeshBuildTask`
- **Geometry gathering** — we feed voxel mesh data directly to Recast, no scene tree needed
- **Thin polygon errors** — we control Recast config and can tune parameters
- **Async bake in editor** — our pipeline doesn't use Godot's bake API
- **Neighbor data for caves/ceilings** — our build task already collects neighbor chunk triangles

## Remaining Problems (require Godot's NavigationServer)

- **O(n^2) edge connections** — still unsolved upstream. Could disable `use_edge_connections` and ensure chunks overlap slightly so Recast handles connectivity internally
- **Main thread sync cost** — largely fixed in Godot 4.3+ but open PR #112908 would help further

## Edge Merging Algorithm Deep Dive (from Godot source)

Source: `godot/modules/navigation_3d/3d/nav_map_builder_3d.cpp`

### Two-Phase Edge Matching

When the navigation map rebuilds, external edges (boundary edges of each region) are matched in two phases:

**Phase 1 — Quantized HashMap (O(E)):**
Each edge vertex is quantized via `floor(pos / merge_cell)` into a 64-bit `PointKey` (21+22+21 bits for x/y/z). Two vertices form an `EdgeKey` (normalized so a.key <= b.key). All external edges are inserted into a `HashMap<EdgeKey, EdgeConnectionPair>`. If two edges from different regions hash to the same key, they're connected. Unmatched edges become "free edges".

- Quantization cell: `merge_cell = cell_size * merge_rasterizer_cell_scale` (default scale: 0.1)
- See `NavRegionBuilder3D::get_point_key()` in `nav_region_builder_3d.cpp:151`

**Phase 2 — Brute-force margin matching (O(F²)):**
All free edges are compared pairwise (`nav_map_builder_3d.cpp:214-269`). For each pair, edges are projected onto each other and checked against `edge_connection_margin` (default 0.1). This is the expensive path — F² comparisons where F = free edge count.

### Rebuild Trigger Frequency

`region_set_navigation_mesh()` is a deferred command (queued, not immediate). Commands flush once per physics frame in `GodotNavigationServer3D::process()`, then `NavMap3D::sync()` runs. Key flow:

1. Multiple `region_set_navigation_mesh` calls queue up
2. `process()` flushes all queued commands in one batch
3. `sync()` → `_sync_dirty_map_update_requests()` sets `iteration_dirty = true`
4. `_build_iteration()` runs **once** per frame (guarded by `iteration_dirty && !iteration_building && !iteration_ready`)
5. With async iterations enabled, build runs on a worker thread; next build won't start until current one finishes

**Effective cost:** O(frames_with_dirty_regions × E_total). If chunks arrive spread across N frames, you get N full rebuilds, each processing all edges from all regions. Multiple chunks arriving in the same frame batch into one rebuild.

### Implications for Optimization

- **Vertex welding** (snapping boundary vertices to match quantization buckets) would ensure edges match in Phase 1, eliminating Phase 2's O(F²). But Phase 1 still runs over all edges every rebuild.
- **Mesh merging** (combining chunk meshes into fewer regions) reduces E_total and the number of regions that trigger rebuilds. But reproduces similar batched-rebuild complexity in our module.
- **Buffering submissions** (waiting for multiple chunks before calling `region_set_navigation_mesh`) reduces the number of rebuilds but adds latency.
- The navigation server already threads the rebuild (async iterations), so doing our own threaded merge doesn't necessarily gain anything — we'd just be moving the same O(E) work from their thread to ours.

### Key NavigationServer Performance Counters

Available via `NavigationServer3D.get_process_info()`:
- `INFO_EDGE_COUNT` — total external edges (Phase 1 cost)
- `INFO_EDGE_FREE_COUNT` — edges that failed quantization matching (Phase 2 cost)
- `INFO_EDGE_CONNECTION_COUNT` — successfully connected edges
- `INFO_EDGE_MERGE_COUNT` — edges merged within regions

If `edge_free_count / edge_count` is high, Phase 2 dominates. If low, Phase 1 (linear scan) or sheer rebuild frequency is the bottleneck.

## Reference Links

- Zylann's main tracking issue: https://github.com/Zylann/godot_voxel/issues/610
- O(n^2) proposal: https://github.com/godotengine/godot-proposals/issues/9381
- Async sync PR: https://github.com/godotengine/godot/pull/100497
- Further sync improvements: https://github.com/godotengine/godot/pull/112908
