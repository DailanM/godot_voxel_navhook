# Ear-Clipping Border Contour Building Algorithm

## Overview

This algorithm replaces `rcBuildContours` entirely with a new function that walks
contours and simplifies them in one sweep, using `RC_BORDER_VERTEX` filtering to
produce chunk-boundary portal edges that match between adjacent chunks. No banding
system is needed — the algorithm runs on a single `rcBuildRegionsMonotone` pass per
chunk.

The key insight: vertices along chunk boundaries are flagged with `RC_BORDER_VERTEX`
and excluded from all contour segments entirely. Portal edges between mandatory
vertices become straight lines with no intermediate vertices — both chunks produce
identical portal edges because mandatory vertices occur at region transitions that
the overlapping rasterization guarantees are the same on both sides. Interior
simplification uses lexicographic XZ ordering (RecastContour.cpp:309-322),
maintaining direction independence across regions.

## Region Setup

### Single Monotone Pass

Run one `rcBuildRegionsMonotone` call per chunk with the standard `borderSize`
parameter. This assigns interior region IDs and paints the XZ border strip with
`RC_BORDER_REG`.

### Y Boundary Handling

Spans whose Y coordinate falls outside the chunk's Y range need to be marked as
border regions so that the contour building treats them the same as XZ borders.

After `rcBuildRegionsMonotone`:
- For each span where `span.y < chunk_y_min_voxel` or `span.y >= chunk_y_max_voxel`:
  - If the span already has `RC_BORDER_REG` (from XZ border painting): keep as-is.
    This allows correct boundary corner detection where XZ and Y borders meet.
  - If the span has an interior region ID: overwrite with a dedicated Y-border
    region ID OR'd with `RC_BORDER_REG`.

This gives border regions on all 6 faces of the chunk. The XZ borders come from
`rcBuildRegionsMonotone`'s built-in border handling; the Y borders are applied
afterward. At corners where XZ and Y borders overlap, the XZ border ID takes
precedence, which is needed for correct corner detection (the contour walk sees a
transition between the XZ border region and the Y border region at the corner).

## Border Vertex Detection

The stock `isBorderVertex` check in `getCornerHeight` (RecastContour.cpp:79-98)
is broadened to flag all vertices where a chunk boundary meets interior regions.
These vertices are excluded from contour segments entirely — this is the mechanism
that produces matching portal edges between chunks.

### The RC_BORDER_VERTEX Check

The check examines the 4 cells meeting at a corner vertex:
```
regs[0] = current span
regs[1] = span in dir
regs[2] = diagonal span
regs[3] = span in dirp
```

A vertex is flagged `RC_BORDER_VERTEX` when:

```
for j in 0..3:
    a, b, c, d = j, (j+1)&3, (j+2)&3, (j+3)&3

    twoSameExts = (regs[a] & regs[b] & RC_BORDER_REG) != 0 and regs[a] == regs[b]
    twoInts     = ((regs[c] | regs[d]) & RC_BORDER_REG) == 0
    noZeros     = regs[a] != 0 and regs[b] != 0 and regs[c] != 0 and regs[d] != 0

    if twoSameExts and twoInts and noZeros:
        isBorderVertex = true
        break
```

Compared to stock Recast, the `intsSameArea` condition is removed — any two
non-border cells meeting at a border trigger the flag, regardless of area type.
This catches T-junctions (two different interior regions meeting at a border)
without requiring a separate detection mechanism.

### Role of RC_BORDER_VERTEX

Vertices with this flag are **never added to any contour segment** and **do not
participate in simplification**. This is the core mechanism for producing matching
portal edges:

1. Mandatory vertices (region changes) delimit segments and ARE emitted if not
   `RC_BORDER_VERTEX`.
2. Between two mandatory vertices on the boundary, ALL intermediate vertices are
   `RC_BORDER_VERTEX` and excluded.
3. The resulting portal edge is a straight line from one mandatory vertex to the
   next — no intermediate vertices that could disagree between chunks.

The ear-clipping effect emerges naturally from this filtering: when a segment ends
at a border mandatory vertex (a T-junction), the segment's last non-border vertex
becomes the effective clip point. The contour jumps from this clip point to the
next segment's first non-border vertex, clipping the "ear" that protrudes toward
the chunk boundary.

### Chunk Corner Detection

Chunk corners (where two different border regions meet, e.g., XZ-Y corners) are
detected during the contour walk as region changes between two different
`RC_BORDER_REG` regions. These are NOT `RC_BORDER_VERTEX` — the 4-cell check
requires two non-border cells, but chunk corners typically have 3+ border cells.
Chunk corners are mandatory vertices that are always kept.

### Hole-Border Transitions

Where a hole (region 0) meets a border region, the `noZeros` condition in the
`RC_BORDER_VERTEX` check fails — the vertex is NOT flagged. This vertex is a
mandatory endpoint in the contour, marking where a portal edge terminates at a
gap in the geometry. It receives the same treatment as a chunk corner: always
kept, never clipped.

## The nav_build_contours Function

Replaces `rcBuildContours` entirely. Walks and simplifies contours in one function,
processing both interior and border regions in a single loop.

### Step 1: Flag Marking

Modified from stock (RecastContour.cpp:869-899). The stock code zeros flags for all
border-region spans. We instead retain border spans that are adjacent to at least
one interior region:

```
for each span i:
    let reg = chf.spans[i].reg

    if reg == 0:
        flags[i] = 0
        continue

    if reg & RC_BORDER_REG:
        // Check adjacency to interior regions.
        let adjacent_to_interior = false
        for dir in 0..3:
            if rcGetCon(s, dir) != RC_NOT_CONNECTED:
                let neighbor_reg = neighbor span's reg
                if neighbor_reg != 0 and (neighbor_reg & RC_BORDER_REG) == 0:
                    adjacent_to_interior = true
                    break

        if not adjacent_to_interior:
            flags[i] = 0    // deep exterior — skip
            continue

        // Fall through to compute boundary flags normally.

    // Compute flags: mark edges where neighbor differs from this span's region.
    res = 0
    for dir in 0..3:
        r = 0
        if connected in dir:
            r = neighbor span's reg
        if r == reg:
            res |= (1 << dir)
    flags[i] = res ^ 0xf
```

### Step 2: Contour Walking and Simplification Loop

The main loop iterates over all spans, walking contours for both interior and
border regions. The key difference from stock: border regions are NOT skipped.

For each contour, the walk and simplification are fused into one streaming pass.
The function maintains:
- `contour[]`: the output simplified vertex array (grows as segments are committed)
- `segment[]`: non-border raw vertices of the segment currently being walked
  (reset at each endpoint)
- `prefix[]`: non-border raw vertices encountered before the first mandatory vertex
  (used for the wrap-around segment)
- `is_border_walk`: whether we are walking a border region or an interior region

```
for each span i with flags[i] != 0 and flags[i] != 0xf:
    let reg = chf.spans[i].reg
    if reg == 0:
        continue

    let is_border_walk = (reg & RC_BORDER_REG) != 0

    contour.clear()
    segment.clear()
    prefix.clear()

    // Walk + simplify in one streaming pass
    walkAndSimplify(x, y, i, chf, flags, is_border_walk,
                    contour, segment, prefix, maxError, maxEdgeLen)

    // Edge tessellation as post-processing (Step 2.4)
    tessellate(contour, maxEdgeLen)

    // Store contour if valid (>= 3 vertices)
    if contour.size() / 4 >= 3:
        store contour in rcContourSet
```

### Walk-and-Simplify State Machine

The walk processes raw vertices one at a time through three phases:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     WALK-AND-SIMPLIFY STATE MACHINE                     │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌──────────┐  non-border vertex   ┌──────────┐                        │
│  │          │ ───────────────────► │          │                        │
│  │ SCANNING │  append to prefix[]  │ SCANNING │                        │
│  │          │ ◄─────────────────── │          │                        │
│  └────┬─────┘  (skip if border)    └──────────┘                        │
│       │                                                                 │
│       │ mandatory vertex M₀ found (the anchor)                          │
│       │ → if M₀ not RC_BORDER_VERTEX: append to segment[]               │
│       ▼                                                                 │
│  ┌───────────────┐  non-border vertex                                   │
│  │               │ ────────────────────┐                                │
│  │  COLLECTING   │  append to segment[] │                               │
│  │               │ ◄───────────────────┘                                │
│  │               │  (skip if border)                                    │
│  └──┬────────┬───┘                                                      │
│     │        │                                                          │
│     │        │ walk complete (returned to start span + start dir)        │
│     │        ▼                                                          │
│     │   ┌─────────────┐                                                 │
│     │   │ WRAP-AROUND  │  combined = segment[] + prefix[]               │
│     │   │   SEGMENT    │  if M₀ not border: append M₀, simplify,       │
│     │   │              │    emit S₀..Sₖ (omit M₀ — already emitted)    │
│     │   │              │  if M₀ border: simplify, emit all              │
│     │   └──────┬───────┘                                                │
│     │          ▼                                                        │
│     │       ┌──────┐                                                    │
│     │       │ DONE │                                                    │
│     │       └──────┘                                                    │
│     │                                                                   │
│     │ mandatory endpoint E detected                                     │
│     ▼                                                                   │
│  ┌──────────────────────┐                                               │
│  │   SIMPLIFY + EMIT    │                                               │
│  │                      │  if segment[] empty: skip                     │
│  │                      │  if E not border:                             │
│  │                      │    append E to segment[]                      │
│  │                      │    simplify segment[]                         │
│  │                      │    emit S₀..Sₖ (omit Sₖ₊₁ = E)              │
│  │                      │  if E border:                                 │
│  │                      │    simplify segment[]                         │
│  │                      │    emit all simplified vertices               │
│  │                      │  reset segment[]                              │
│  │                      │  if E not border: append E (starts next)      │
│  └──────────┬───────────┘                                               │
│             │                                                           │
│             └──────────────────────────────► COLLECTING                  │
│                                                                         │
│  ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─   │
│  FALLBACK: if walk completes in SCANNING (no mandatory vertex found),   │
│  use stock lower-left/upper-right bounding box initialization.          │
└─────────────────────────────────────────────────────────────────────────┘
```

**Endpoint detection** (at each vertex during COLLECTING):

A vertex is a mandatory endpoint when the neighbor region changes or the area
border flag changes from the previous vertex (same detection as stock
simplifyContour lines 227-232). All mandatory endpoints are treated uniformly —
the `RC_BORDER_VERTEX` flag determines whether the endpoint vertex is included
in the contour (not border) or excluded (border). No separate T-junction
classification is needed.

### Step 2.1: Walking a Contour

The walk follows stock `walkContour` mechanics (RecastContour.cpp:103-184): start
at a span, find the first boundary edge, walk clockwise around the region.

At each step, when an edge is a boundary edge (the flag bit for direction `dir` is
set):

1. Call `getCornerHeight` to compute corner height and the `RC_BORDER_VERTEX` flag
   (see Border Vertex Detection above).

2. Determine `r` flags for the raw vertex:

   ```
   r = neighbor_region
   if isBorderVertex:
       r |= RC_BORDER_VERTEX
   if isAreaBorder:
       r |= RC_AREA_BORDER
   ```

3. Check if this vertex is a **mandatory endpoint** — the neighbor region changed
   or the area border flag changed from the previous vertex:

   ```
   let is_endpoint = false
   if neighbor_reg changed from previous vertex or area_border changed:
       is_endpoint = true
   ```

4. Process the vertex according to the current walk phase (see state machine above):

   **SCANNING phase:** If `is_endpoint`, this is the anchor M₀ — enter COLLECTING.
   If M₀ is not `RC_BORDER_VERTEX`, append it to `segment[]`.
   If not an endpoint: append to `prefix[]` only if not `RC_BORDER_VERTEX`.

   **COLLECTING phase:** If `is_endpoint`, finalize the current segment
   (simplify + emit per state machine). Reset `segment[]`. If E is not
   `RC_BORDER_VERTEX`, append it to the new `segment[]` as its first vertex.
   If not an endpoint: append to `segment[]` only if not `RC_BORDER_VERTEX`.

5. **Flag clearing** after processing the vertex:
   ```
   if is_border_walk:
       if chf.spans[i].reg & RC_BORDER_REG:
           flags[i] &= ~(1 << dir)
   else:
       flags[i] &= ~(1 << dir)
   ```

   **Interior walk:** Always clear the flag for this edge (stock behavior).
   **Border walk:** Only clear if the current span is a border region. The border
   contour's boundary includes edges that face interior regions. The spans on the
   interior side of those edges have their own flags that must be preserved for
   the interior contour walks. The guard `chf.spans[i].reg & RC_BORDER_REG` is
   defensive — the stock walk mechanic stays on spans of the region being traced,
   so border walks only visit border spans. But the guard documents the intent:
   interior span flags must never be cleared during a border walk.

6. Walk termination: stop when we return to the starting span `starti` with the
   starting direction `startDir`. Process the wrap-around segment
   (`segment[] + prefix[]`) using M₀ as the closing reference (see state machine).

If no mandatory endpoints are found during the entire walk (SCANNING completes
without finding one), fall back to the stock lower-left / upper-right bounding box
initialization (stock simplifyContour lines 242-284).

### Step 2.2: Segment Simplification (max-deviation)

When a segment is completed (endpoint reached), simplify the non-border raw
vertices in `segment[]` using the max-deviation loop. The first and last vertices
of `segment[]` seed the simplified array, then intermediate vertices are inserted
iteratively.

When the ending mandatory endpoint E is non-border, it is appended to `segment[]`
before simplification — this extends the reference line to the true segment
boundary. When E is a border mandatory vertex (a T-junction), it is not in
`segment[]`, and the reference line ends at the last non-border vertex. This means
the simplification approximates only the non-border portion of the contour; the
border portion is clipped entirely.

**Force simplification on all segments.** Stock simplifyContour (line 325-326) only
simplifies segments where the neighbor is region 0 (wall) or has `RC_AREA_BORDER`.
We remove this condition entirely — simplify all segments regardless of neighbor
type.

The simplification loop is unchanged from stock (RecastContour.cpp:286-365):

1. **Seed** the simplified array with the two endpoints (segment[0] and
   segment[last]).
2. **Max-deviation loop:** For each pair of consecutive simplified vertices, traverse
   raw vertices between them in **lexicographic XZ order**
   (RecastContour.cpp:309-322). Find the vertex with maximum perpendicular distance
   (`distancePtSeg`, which operates in XZ only). If distance exceeds `maxError²`,
   insert it. Repeat until all deviations are within tolerance.

Result: a simplified vertex sequence `S_0, S_1, ..., S_{k+1}` where all vertices
are non-border raw vertices from the segment.

#### Direction Independence

The lexicographic ordering at RecastContour.cpp:309-322:
```cpp
if (bx > ax || (bx == ax && bz > az))
    // traverse forward
else
    // traverse backward, swap endpoints
```

This ensures the same raw vertex is selected as the maximum-deviation point
regardless of walk direction. Two regions sharing an interior boundary walk that
boundary in opposite directions but produce the same simplified vertices. Since
`RC_BORDER_VERTEX` vertices are excluded from both regions' segments, the shared
boundary contains only interior vertices — the simplification inputs are identical
on both sides.

### Step 2.3: Segment Commit

After simplification, emit the simplified vertices to `contour[]`:

- If the segment ends at a **non-border** mandatory endpoint E (E is the last
  vertex in `segment[]`): emit `S_0` through `S_k`, omitting `S_{k+1}` = E.
  E will be `S_0` of the next segment.
- If the segment ends at a **border** mandatory endpoint (not in `segment[]`):
  emit all simplified vertices `S_0` through `S_{k+1}`.
- If `segment[]` is empty (all vertices between the two mandatory endpoints were
  `RC_BORDER_VERTEX`): skip. Nothing is emitted.

For each vertex emitted to `contour[]`, assign the neighbor region and flags using
stock behavior (RecastContour.cpp:442-449):
```
let ai = (raw_index + 1) % n
let bi = raw_index
reg = raw_verts[ai*4+3] & (RC_CONTOUR_REG_MASK | RC_AREA_BORDER)
```

Since all emitted vertices are non-border, the `RC_BORDER_VERTEX` bit is always
0 and does not need to be OR'd in from `raw_verts[bi]`.

### Step 2.4: Edge Tessellation

After all segments are committed, run the edge tessellation pass over `contour[]`
as a post-processing step. This splits simplified edges that exceed `maxEdgeLen` by
inserting midpoint raw vertices.

The tessellation uses lexicographic XZ ordering for midpoint selection
(RecastContour.cpp:407-411):
```cpp
if (bx > ax || (bx == ax && bz > az))
    maxi = (ai + n/2) % pn;
else
    maxi = (ai + (n+1)/2) % pn;
```

This ensures the same midpoint is selected regardless of walk direction, maintaining
direction independence across regions and chunks.

Force tessellation on all edge types (override the stock condition at lines 389-393
that restricts to wall edges and area borders). However, do not insert midpoints
that are `RC_BORDER_VERTEX` — portal edges (straight lines between mandatory
vertices) must remain free of intermediate vertices to preserve cross-chunk matching.

Tessellation can also be applied during segment commit (Step 2.3) as each simplified
edge is appended to `contour[]` — this is an implementation choice that doesn't
affect the result.

### Step 2.5: Degenerate Handling

After the walk completes, contours with fewer than 3 vertices are discarded (same
as stock, RecastContour.cpp:938). This can happen when a region is almost entirely
border-facing and most segments produce no non-border vertices.

## Direction Independence Analysis

The algorithm's correctness depends on adjacent regions (within a chunk) and
adjacent chunks producing matching vertices at shared boundaries.

### Across Regions (Within a Chunk)

Two interior regions R1 and R2 sharing a boundary walk that boundary in opposite
directions. The max-deviation simplification uses lexicographic XZ ordering
(RecastContour.cpp:309-322) to traverse raw vertices in a canonical direction,
regardless of which region's contour we're tracing.

The `RC_BORDER_VERTEX` filtering does not affect shared interior boundaries:
vertices on a shared boundary between two interior regions are not
`RC_BORDER_VERTEX` (the 4-cell check requires two same-border cells, which are
absent at an interior-only boundary). Both regions include the full set of raw
boundary vertices in their segments, and the direction-independent simplification
produces identical simplified vertices. This is stock Recast behavior.

### Across Chunks

Portal edges are straight lines between mandatory vertices with no intermediate
vertices. Both chunks produce the same mandatory vertices at the chunk boundary
because:
1. Mandatory vertices occur at region transitions
2. The overlapping `borderSize`-wide strip has identical span data on both chunks
3. Region transitions in the overlap are at the same cell-corner positions

Since there are no intermediate vertices on portal edges, there is nothing to
disagree on — the portal edge is fully defined by its two endpoint positions.

For border contours, the outer-edge vertices (facing void, not `RC_BORDER_VERTEX`
because `noZeros` fails) are simplified normally. Both chunks have identical raw
vertices on the outer edge (from overlapping rasterization), and the
direction-independent simplification produces matching simplified vertices.

### Edge Tessellation

Edge tessellation (RecastContour.cpp:367-440) selects midpoints using lexicographic
ordering (lines 407-411):
```cpp
if (bx > ax || (bx == ax && bz > az))
    maxi = (ai + n/2) % pn;
else
    maxi = (ai + (n+1)/2) % pn;
```

For even vertex counts, both directions select the same midpoint. For odd counts,
the `n/2` vs `(n+1)/2` adjustment selects the same vertex regardless of direction.
This is also stock behavior. Portal edges skip tessellation (midpoints would be
`RC_BORDER_VERTEX`), so this analysis applies only to interior and outer-edge
segments.

## Edge Cases

### Empty Segments

A segment between two mandatory endpoints may contain zero non-border vertices —
all raw vertices between the endpoints are `RC_BORDER_VERTEX`. This occurs for
boundary-facing segments (between two T-junctions, or between a T-junction and a
chunk corner along the inner edge of the border strip). Nothing is emitted; the
contour jumps directly between adjacent non-empty segments.

### Short Segments Near T-Junctions

A segment between a non-border mandatory vertex M and a border mandatory vertex
(T-junction) may have very few non-border raw vertices. If only one non-border
vertex exists, it becomes both `S_0` and `S_{k+1}` — a single-vertex segment. That
vertex is emitted and serves as the effective clip point. If no non-border vertices
exist between M and the T-junction (empty segment), M was the last emitted vertex
from the previous segment, and the contour continues from M directly to the next
non-empty segment's first vertex.

### Consecutive Border Mandatory Endpoints

When the contour runs along the border through multiple T-junctions, consecutive
border mandatory endpoints produce consecutive empty segments. No vertices are
emitted for any of them. The contour clips the entire border-facing stretch,
connecting the last non-border vertex before the stretch to the first non-border
vertex after it.

### Degenerate Contours

A region that is almost entirely border-facing may have most segments empty,
leaving fewer than 3 vertices total. These contours are discarded (same as stock,
RecastContour.cpp:938). The space is covered by the adjacent boundary contours.

### Chunk Corners

Where two chunk borders meet (e.g., XZ-Y corners), the contour walk encounters a
transition between two different border regions. This is detected as a mandatory
endpoint. Chunk corner vertices are NOT `RC_BORDER_VERTEX` (the 4-cell check fails
because 3+ cells are border), so they are always emitted.

### Y Boundaries

Spans outside the chunk's Y range are marked as border regions (see Region Setup).
Border vertices at Y boundaries are detected and excluded identically to XZ border
vertices. At corners where Y and XZ borders meet, the XZ border ID takes
precedence (it was assigned first by `rcBuildRegionsMonotone` and not overwritten),
allowing the contour walk to detect the corner as a transition between two different
border regions (XZ-border and Y-border) — a mandatory vertex that is always kept.

### Hole-Border Transitions

Where a hole (region 0) meets a border region at the chunk boundary, the
`noZeros` condition prevents the vertex from being flagged as `RC_BORDER_VERTEX`.
The vertex is a mandatory endpoint and is always emitted, marking where the portal
edge terminates. This receives the same treatment as a chunk corner.

### Self-Intersection Risk

For non-convex regions, the chord from one clip point to the next (in the trimmed
interior contour) could theoretically intersect the contour. In practice this is
extremely unlikely: the ear is bounded by `maxError` perpendicular to any simplified
edge, so the contour would need to curve back within a band of `maxError` width
within a single segment's span. The ear-clipping analogy holds: we are removing
small protrusions bounded by one simplified segment, which is safe regardless of the
overall region's convexity.
