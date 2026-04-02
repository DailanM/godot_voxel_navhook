# Navmesh Region API for godot-voxel

## Context

Procedurally generated terrain frequently produces navmeshes with disconnected regions. Agents navigating this terrain need a way to detect disconnections and reason about how to cross them (jumping, flying, teleporting, etc.). Rather than baking agent-specific traversal logic into the module, we expose minimal topological and geometric data so that downstream developers can implement their own traversal strategies.

## Design Rationale

The module has unique, cheap access to navmesh geometry at generation time. No downstream code can recover this information without redundant spatial queries or a much larger API surface. The module should therefore expose what it uniquely knows — connectivity topology and boundary geometry — and nothing more.

Specifically, the module should **not** own:

- **Jump envelopes or traversal feasibility.** These depend on agent physics (jump velocity, gravity, agent radius) and game design decisions that vary per project. A platformer, an RTS, and a creature simulation all define "reachable" differently.
- **Region graph pathfinding.** Runtime pathfinding may need to incorporate dynamic costs (invalidated edges, agent-specific penalties, threat avoidance). Exposing the topology and letting users pathfind in GDScript or a GDExtension keeps this flexible.
- **Validated jump points or navigation links.** These require physics raycasts the module doesn't have access to, and caching strategies that depend on the game's update patterns.

Developers who need these capabilities can compute them in GDScript or a GDExtension using the exposed primitives, with full access to their own game's parameters. They can cache results using the module's existing local storage facilities.

## A Note on Boundary Edges

The module stitches chunk navmeshes by aligning boundary vertices so that the Godot NavigationServer can merge shared edges via hashing. This means boundary edges within a chunk fall into two categories:

1. **Merged edges** — edges shared with an adjacent chunk's navmesh. After stitching, the NavigationServer treats these as internal edges and navigation crosses them seamlessly.
2. **Unmerged boundary edges** — edges at the boundary of navigable surface that have no matching edge in a neighbor. These represent the actual perimeter of a connected navmesh region: cliff edges, gaps, the border between navigable and non-navigable terrain.

It is the **unmerged boundary edges** that matter for traversal. These are the edges an agent would stand on before jumping, and the edges it would land on after. The API exposes these specifically.

## Proposed API

### `get_regions_in_chunk(chunk_position: Vector3i) -> PackedInt32Array`

Returns the IDs of all connected navmesh regions present in the given chunk. Region IDs are globally unique and stable for the lifetime of the chunk (they are invalidated when a chunk's navmesh is regenerated).

This is useful for understanding local navmesh fragmentation and for building a region graph in user code.

### `get_region_for_point(position: Vector3) -> int`

Returns the region ID of the connected navmesh region containing (or nearest to) the given world-space position. Returns -1 if no region is within range.

This is the primary runtime query. An agent checks its own region and its goal's region; if they differ, traversal logic is needed.

### `get_region_boundary_edges(region_id: int, neighborhood: AABB) -> PackedVector3Array`

Returns the unmerged boundary edges of the specified region within the given spatial extent, as a packed array of point pairs (every two consecutive Vector3 values define one edge segment: start, end).

The `neighborhood` parameter limits the query spatially. A typical use would be a 3×3×3 cube of chunks around the agent's current chunk. This keeps the query local and the result set manageable.

This is the workhorse for traversal computation. Developers iterate these edges to find candidate launch/landing points for their specific traversal mechanic, then validate feasibility using the physics server.

### Future Considerations

- **Edge normals.** Exposing the face normal adjacent to each boundary edge would let developers reason about surface orientation (e.g., rejecting edges on surfaces too steep to stand on). This could be added as a parallel packed array or by expanding the return format.
- **Region merging across chunks.** The module internally knows when stitching connects two per-chunk regions into a single global region. Currently this is reflected in the assigned region IDs. If use cases arise where developers need the per-chunk decomposition as well, a separate query could expose it.
- **Change notification.** When a chunk is regenerated, any region IDs associated with that chunk become invalid. A signal or versioning mechanism may be needed so downstream code can invalidate caches.
