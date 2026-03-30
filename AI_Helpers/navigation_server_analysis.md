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

## Potential Next Step: Detour for Pathfinding

The most complete solution would be to use **Detour** (Recast's companion pathfinding library) for queries directly, bypassing NavigationServer entirely. This would eliminate both the sync cost and edge connection scaling. This is essentially what Zylann was contemplating, and what one user in issue #187 successfully did (generating tiled navmesh from heightmap + passability mask with Recast/Detour directly).

This would involve:
- Building Detour tiled navmeshes from our existing Recast output
- Managing a `dtNavMesh` + `dtNavMeshQuery` instance in `NavMeshManager`
- Exposing pathfinding queries (find_path, nearest_point, etc.) on VoxelTerrain
- Tile add/remove as chunks stream in/out

## Reference Links

- Zylann's main tracking issue: https://github.com/Zylann/godot_voxel/issues/610
- O(n^2) proposal: https://github.com/godotengine/godot-proposals/issues/9381
- Async sync PR: https://github.com/godotengine/godot/pull/100497
- Further sync improvements: https://github.com/godotengine/godot/pull/112908
