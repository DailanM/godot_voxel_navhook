# Y-Boundary Seam Analysis

## Current State

The `rcMarkBoxArea` approach (applied after `rcErodeWalkableArea`) correctly assigns walkable area ownership at Y chunk boundaries: each chunk claims its portion of the terrain with no duplication and no gap. However, the resulting nav mesh edges at Y boundaries don't align between adjacent chunks, producing visible seam artifacts.

## How Recast Ensures Edge Consistency in the Detail Mesh

The detail mesh edge sampling has a deliberate seam-prevention mechanism (`RecastMeshDetail.cpp:664-767`):

```
// Tessellate outlines.
// This is done in separate pass in order to ensure
// seamless height values across the ply boundaries.
```

```
// Make sure the segments are always handled in same order
// using lexological sort or else there will be seams.
```

For each polygon edge, the detail mesh:
1. **Lexicographically sorts** the edge endpoints so the same edge is always walked in the same direction regardless of which polygon "owns" it
2. Places vertices at **regular `sampleDist` intervals** along the XZ distance
3. Overwrites every vertex's Y with `getHeight() * ch` — a **cell-quantized lookup** into the compact heightfield (no interpolation, just nearest-cell sampling)
4. Simplifies intermediate edge vertices via Douglas-Peucker with `sampleMaxError`

Two adjacent polygons sharing an edge produce **bit-identical** edge vertices because both process the same edge direction, same sample count, same heights. This holds both within a tile and across XZ tile boundaries (because the heightfield data in the border zone is identical in both tiles).

## How XZ Tile Seams Match

The polymesh stores vertices as **integer cell indices** (`unsigned short[3]`), not world-space floats. `addVertex()` deduplicates by exact XZ match + Y within ±2 cells. Vertices are on the `cs`/`ch` grid by construction.

At XZ tile borders, two things ensure matching:
1. **`RC_BORDER_REG`** paints the outer ring with border-region IDs. Contour vertices at the border are flagged `RC_BORDER_VERTEX`. These are then **removed by `removeVertex()`**, which retriangulates the hole — leaving only the "corner" vertices where the region's shape bends at the tile edge. Adjacent tiles produce the same corners because they see the same rasterized data in their overlapping border zones.
2. **Detail mesh edge sampling** then produces identical edge samples along those shared border edges, because the edge endpoints are the same quantized cell positions and the heightfield data is the same.

XZ matching = **quantized polymesh corners + deterministic detail sampling + identical heightfield data = bit-identical seam edges**.

## Why Y Boundaries Don't Get Matching Edges

Our Y boundary mechanism (`rcMarkBoxArea` after erode) achieves correct walkable ownership but **skips the entire seam-matching pipeline**:

1. **No `RC_BORDER_REG` painting for Y.** The Y-border cells get `area = RC_NULL_AREA` (non-walkable), not `reg = id | RC_BORDER_REG`. So contour vertices at the Y boundary are NOT flagged `RC_BORDER_VERTEX` and are NOT removed by `removeVertex()`. Each chunk retains its own independent vertex set along the Y boundary.

2. **Contours are walked independently.** Each chunk's contour walker traces its region's outline in 2D (XZ). At the Y boundary, the walkable/non-walkable transition in XZ follows the line where the terrain surface crosses the chunk's Y limit. Both chunks follow approximately the same line — but each chunk's contour is simplified independently (Douglas-Peucker on each chunk's full contour polygon), and the simplification depends on the entire contour shape, not just the boundary segment. **Different total contour shapes -> different simplification decisions -> different vertex positions along the boundary.**

3. **Detail mesh can't fix misaligned contour vertices.** The detail mesh faithfully samples edges between the polymesh's corner vertices. If chunk A's contour corner is at XZ = (15.3, 8.7) and chunk B's corner is at XZ = (15.6, 8.4), the detail meshes produce two different edge polylines that don't coincide, regardless of how deterministic the sampling is.

4. **The `twoSameExts` border-vertex detection is intrinsically 2D.** It checks a 2x2 XZ cell neighborhood for transitions from interior to `RC_BORDER_REG`. Y-border cells are in the **same XZ column** as interior cells (stacked above/below, not adjacent in XZ), so the 2D contour walker never sees them as neighbors. Even if we set `reg |= RC_BORDER_REG` on Y-border spans, the contour walker wouldn't detect the transition.

## Remaining Hypotheses

**Primary cause (high confidence):** Independent contour simplification at Y boundaries. Each chunk's contour is a different polygon with a different vertex count and shape. Douglas-Peucker simplification retains different vertices on each side. The resulting polymesh corners don't coincide, and the detail mesh faithfully follows them, producing visible misalignment.

**Secondary contributor:** The `getHeight()` fallback. When a detail edge sample falls on a cell with `RC_UNSET_HEIGHT`, it spiral-searches for the nearest valid height. At Y boundaries, some cells may be marked non-walkable (our `rcMarkBoxArea` cells), and if the height patch doesn't include them, the spiral search may diverge between chunks, causing Y jitter even when XZ positions agree.

**Tertiary (unlikely):** The `getJitterX/Y` calls for interior detail samples add ±0.1*cs noise. This shouldn't affect seam edges (jitter is only on interior samples).

## Viable Approaches

### A. Force Y-boundary contour vertices to cell grid (moderate effort)

After `rcBuildContours` and before `rcBuildPolyMesh`, iterate each contour's raw vertices. For any vertex whose world-Y (computed from the contour's integer cell position) is within 1 cell of the chunk's Y boundary, snap its XZ to the nearest `cs` cell boundary. Both chunks would independently snap to the same global cell grid, producing matching contour vertices, which leads to matching detail mesh edges.

This is the closest analog to what `RC_BORDER_VERTEX` removal achieves: it forces boundary vertices to deterministic positions that both chunks agree on.

**Pros:** Works within Recast's existing data structures. No need to modify Recast internals.
**Cons:** Need to identify which contour vertices are "at the Y boundary" — requires computing the world-Y of the contour's span floor and comparing to the chunk boundary.

### B. Post-process polymesh: snap Y-boundary vertices (moderate effort)

After `rcBuildPolyMesh`, iterate `pmesh->verts` (integer cell coordinates). For vertices whose Y cell is within 1 of the chunk's Y boundary cell, snap their XZ to multiples of some larger grid (e.g., every N cells). Both chunks snap to the same grid, producing matching vertices, which leads to matching detail edges.

**Pros:** Simpler than contour-level manipulation. Polymesh vertices are just integer arrays.
**Cons:** Snapping might distort polygon shapes. The detail mesh still samples independently.

### C. Remove border vertices at Y boundary (moderate-high effort)

After contour build but before polymesh, flag vertices at the Y boundary with `RC_BORDER_VERTEX` in the contour data (`cont.verts[j*4+3] |= RC_BORDER_VERTEX`). Then `rcBuildPolyMesh`'s existing vertex-removal pass aggressively simplifies Y boundary edges the same way it does XZ borders.

Both chunks would remove the same vertices (because the `canRemoveVertex` + `removeVertex` logic is deterministic given the same polymesh topology, and both chunks have the same geometry at the boundary), leaving only essential corner vertices.

**Pros:** Uses Recast's existing `removeVertex` infrastructure. Produces the same kind of clean boundary edges that XZ tiles get.
**Cons:** "Both chunks have the same polymesh topology at the boundary" is an assumption that needs verification — if false, they'd remove different vertices and the fix backfires. Requires careful identification of which contour vertices are Y-boundary.

### D. Build full Y columns as one tile (large refactor)

Don't tile in Y at all. For each XZ tile position, build one heightfield that covers the full Y range (union of all Y chunks at that XZ position). Tile only in XZ, where Recast's seam matching works natively.

**Pros:** Eliminates Y seam issues entirely. Uses Recast's native tile mechanism for all boundaries.
**Cons:** Heightfields become much taller (more memory, more rasterization work). Requires restructuring the dispatch: instead of "one nav build per mesh block," it's "one nav build per XZ column of mesh blocks." Breaks the current 1:1 mapping between mesh blocks and nav builds.

### E. Shared-edge reconciliation between chunks (high effort)

After building both chunk A's and chunk B's nav meshes, find edge pairs that run along the shared Y boundary and force them to use identical vertex positions (e.g., average or snap to one side). This is a post-build cross-chunk operation.

**Pros:** Directly addresses the symptom.
**Cons:** Requires coordinating between two independently-built nav meshes. Hard to do in a streaming/async system where chunks build at different times. Would need a "reconciliation pass" that runs after both chunks are ready.
