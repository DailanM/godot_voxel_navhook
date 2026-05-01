# Implementation Checklist: `nav_build_contours`

Replace `nav_build_contours_raw` with a cross-chunk-aware contour builder that walks both interior and border regions, uses `RC_BORDER_VERTEX` filtering to produce matching portal edges between chunks, and fuses walk + simplification into one streaming pass.

**Creates:** `terrain/navigation/nav_build_contours.h` and `.cpp`
**Build task integration** (`nav_mesh_build_task.cpp:361`): out of scope

---

## Progress

- [x] **Step 1** — Create header file
- [x] **Step 2** — Copy boilerplate and unchanged helpers
- [x] **Step 3** — Implement modified `getCornerHeight`
- [x] **Step 4** — Implement modified flag marking
- [x] **Step 5** — Implement the contour walk (interior + border with reversal)
- [x] **Step 6** — Implement segment identification and simplification
- [x] **Step 7** — Implement tessellation
- [x] **Step 8** — Implement degenerate handling and contour storage
- [x] **Step 9** — Implement hole merging
- [x] **Step 10** — Assemble the main function

---

## Step 1: Create the header file

**File:** `terrain/navigation/nav_build_contours.h`

```cpp
bool nav_build_contours(rcContext *ctx, const rcCompactHeightfield &chf,
                        float maxError, int maxEdgeLen, rcContourSet &cset);
```

Follow the pattern of `nav_build_contours_raw.h`: include guard `VOXEL_NAV_BUILD_CONTOURS_H`, gated on `VOXEL_ENABLE_NAVIGATION`, `#include <Recast.h>`, namespace `zylann::voxel`.

---

## Step 2: Copy boilerplate and unchanged helpers

**File:** `terrain/navigation/nav_build_contours.cpp`

Copy from `nav_build_contours_raw.cpp` (already adapted to module code style):

- **Geometry helpers** (raw lines 178-247): `prev`, `next`, `area2`, `xorb`, `left`, `leftOn`, `collinear`, `intersectProp`, `between`, `intersect`, `vequal`, `intersectSegContour`, `inCone`
- **Polygon area** (raw lines 168-176): `calcAreaOfPolygon2D`
- **Hole merging** (raw lines 249-412): `mergeContours`, `findLeftMostVertex`, `compareHoles`, `compareDiagDist`, `mergeRegionHoles`, plus structs `rcContourHole`, `rcContourRegion`, `rcPotentialDiagonal`

Copy from stock `RecastContour.cpp`:
- **`distancePtSeg`** (stock lines 186-207): perpendicular XZ distance for simplification
- **`removeDegenerateSegments`** (stock lines 579-602): removes adjacent XZ-equal vertices

All in an anonymous namespace inside `zylann::voxel`.

---

## Step 3: Implement modified `getCornerHeight`

**Ref:** `RecastContour.cpp:28-101`

Copy the stock function. Modify the border vertex check loop (stock lines 79-98):

**Keep stock check as-is** (lines 89-93):
```
twoSameExts && twoInts && intsSameArea && noZeros → isBorderVertex = true
```

**Add T-vertex check** immediately after:
```cpp
const bool intsDiffReg = (regs[c] & RC_CONTOUR_REG_MASK) != (regs[d] & RC_CONTOUR_REG_MASK);
if (twoSameExts && twoInts && intsDiffReg && noZeros) {
    isBorderVertex = true;
    break;
}
```

**Why two checks:** The stock check catches border vertices where interior cells share the same area type. The T-vertex check catches T-junctions where two different interior regions meet at the border with different area types. Chunk corners are safe — they fail `twoSameExts` because the two border cells have different region IDs from different `paintRectRegion` calls (`RecastRegion.cpp:1575-1578`).

---

## Step 4: Implement modified flag marking

**Ref:** `RecastContour.cpp:869-899` (stock flag marking)

Stock zeros flags for ALL border-region spans (line 878-879). Modify to retain border spans adjacent to at least one interior region:

```
for each span i:
    reg = chf.spans[i].reg

    if reg == 0:
        flags[i] = 0; continue

    if reg & RC_BORDER_REG:
        adjacent_to_interior = false
        for dir in 0..3:
            if rcGetCon(s, dir) != RC_NOT_CONNECTED:
                neighbor_reg = neighbor span's reg
                if neighbor_reg != 0 and !(neighbor_reg & RC_BORDER_REG):
                    adjacent_to_interior = true; break
        if !adjacent_to_interior:
            flags[i] = 0; continue    // deep exterior — skip

    // Standard flag computation (stock lines 883-896)
    res = 0
    for dir in 0..3:
        r = 0; if connected: r = neighbor reg
        if r == reg: res |= (1 << dir)
    flags[i] = res ^ 0xf
```

Only inner-edge border spans (adjacent to interior regions) get flags. Deep exterior border spans have `flags=0` and are never walked.

---

## Step 5: Implement the contour walk

**Ref:** `RecastContour.cpp:103-184` (stock `walkContour`)

The walk collects raw vertices into `raw_verts[]` (working array, not stored in output). Works for both interior and border regions.

### Interior walks

Identical to stock `walkContour`:
- Clear flags on all visited boundary edges (stock line 154)
- Rotate CW after boundary edge (stock line 155: `dir = (dir+1) & 0x3`)
- Rotate CCW when moving to neighbor (stock line 176: `dir = (dir+3) & 0x3`)

### Border walks — two modifications

**1. Flag clearing guard** — only clear flags on border spans:
```cpp
// Stock line 154: flags[i] &= ~(1 << dir);
// Modified:
if (chf.spans[i].reg & RC_BORDER_REG)
    flags[i] &= ~(1 << dir);
```

**2. Direction reversal at mandatory vertices** — when a mandatory non-`RC_BORDER_VERTEX` vertex is hit during a border walk, swap the rotation directions:
```cpp
// Stock line 155: dir = (dir+1) & 0x3;  // Rotate CW
// Reversed:       dir = (dir-1) & 0x3;  // Rotate CCW (equiv: (dir+3) & 0x3)

// Stock line 176: dir = (dir+3) & 0x3;  // Rotate CCW
// Reversed:       dir = (dir+1) & 0x3;  // Rotate CW
```

Track a `bool reversed` flag. Flip it at each mandatory non-border vertex. Use it to choose between stock and swapped rotation directions.

During the return (reversed) phase:
- T-junctions are NOT followed (don't step inward at boundary changes)
- Flags are NOT cleared

### Walk output

`raw_verts[]` — all vertices, 4 ints each `(x, y, z, r)` where `r` includes `RC_BORDER_VERTEX` and `RC_AREA_BORDER` flags. Same format as stock `walkContour`.

**Termination:** Same as stock (line 179): `starti == i && startDir == dir`.

---

## Step 6: Implement segment identification and simplification

**Ref:** `RecastContour.cpp:209-365` (stock `simplifyContour`)

After the walk produces `raw_verts[]`, process it: identify mandatory endpoints, split into segments of non-border vertices, simplify each, assemble output `contour[]`.

### Mandatory endpoint detection

Ref: stock lines 227-232:
```cpp
const bool differentRegs = (raw_verts[i*4+3] & RC_CONTOUR_REG_MASK) !=
                           (raw_verts[ii*4+3] & RC_CONTOUR_REG_MASK);
const bool areaBorders = (raw_verts[i*4+3] & RC_AREA_BORDER) !=
                         (raw_verts[ii*4+3] & RC_AREA_BORDER);
bool is_mandatory = differentRegs || areaBorders;
```

### State machine

| Phase | Non-border, non-mandatory | Non-border, mandatory | RC_BORDER_VERTEX |
|-------|---------------------------|----------------------|------------------|
| SCANNING | Append to `prefix[]` | Anchor M0 → enter COLLECTING. Append M0 to `segment[]` | Skip |
| COLLECTING | Append to `segment[]` | Endpoint E → simplify+emit, reset. Append E to new `segment[]` if non-border | Skip |

### Simplify a segment

Adapted from stock max-deviation loop (lines 286-365):
- Seed simplified array with segment endpoints
- Max-deviation loop with lexicographic XZ ordering (lines 309-322) for direction independence
- **Force simplification on ALL segments** — remove the edge-type condition at stock lines 325-326

### Emit logic at endpoint E

- `segment[]` empty → emit nothing
- E is non-border → append E to segment, simplify, emit S0..Sk (omit Sk+1=E, it starts next segment)
- E is `RC_BORDER_VERTEX` → simplify segment as-is, emit ALL simplified vertices

### Wrap-around (walk complete)

- Combine remaining `segment[] + prefix[]`
- Simplify with M0 as closing reference
- Emit (omit M0 duplicate if M0 was non-border)

### Fallback (no mandatory vertex found)

Use stock lower-left/upper-right bounding box initialization (stock lines 242-284).

### Vertex format

Each emitted vertex stores `(x, y, z, raw_index)` — the index in `raw_verts[]` is needed by tessellation (Step 7) and final reg assignment.

---

## Step 7: Implement tessellation

**Ref:** `RecastContour.cpp:267-449` (simplification + tessellation in `simplifyContour`)

Post-processing pass on the completed `contour[]`. Splits long simplified edges by inserting midpoint raw vertices.

**Tessellation loop ref:** stock lines 367-440.

### Modifications from stock

1. **Force on ALL edge types** — remove `buildFlags` conditions at stock lines 388-393. Always tessellate.

2. **Direction-independent midpoint selection** — preserved exactly from stock (lines 407-411):
   ```cpp
   if (bx > ax || (bx == ax && bz > az))
       maxi = (ai + n/2) % pn;
   else
       maxi = (ai + (n+1)/2) % pn;
   ```
   Both chunks select the same midpoint on shared portal edges.

3. **Midpoint lookup from `raw_verts[]`** — `contour[i*4+3]` stores raw index `ai`. Tessellation computes midpoint index `maxi` in `raw_verts[]` and inserts `raw_verts[maxi*4 + 0..2]`. Same mechanism as stock (lines 430-433).

### Final reg assignment

After tessellation, assign final reg values (ref: stock lines 442-449):
```cpp
for each vertex i in contour[]:
    const int ai = (contour[i*4+3] + 1) % pn;
    const int bi = contour[i*4+3];
    contour[i*4+3] = (raw_verts[ai*4+3] & (RC_CONTOUR_REG_MASK | RC_AREA_BORDER))
                   | (raw_verts[bi*4+3] & RC_BORDER_VERTEX);
```

Where `pn = raw_verts.size() / 4`.

---

## Step 8: Implement degenerate handling and contour storage

**Ref:** `RecastContour.cpp:936-1002` (contour storage in `rcBuildContours`)

### After tessellation

1. `removeDegenerateSegments` (stock lines 579-602)
2. Discard contours with `< 3` vertices (stock line 938)
3. Discard contours where `calcAreaOfPolygon2D == 0` (stock line 453) — must run AFTER tessellation

### Storage (stock lines 960-1002)

- Allocate `cont->verts`, copy from `contour[]`
- Set `cont->rverts = nullptr`, `cont->nrverts = 0`
- Apply borderSize coordinate offset: subtract `borderSize` from `v[0]` and `v[2]` (stock lines 970-978, same as `nav_build_contours_raw.cpp:528-533`)
- Set `cont->reg = reg`, `cont->area = area`
- Dynamic contour array growth: stock lines 940-957

---

## Step 9: Implement hole merging

**Ref:** `RecastContour.cpp:1007-1101` (hole merging in `rcBuildContours`)

Same structure as stock, one change: **skip border region contours** (`cont.reg & RC_BORDER_REG`). Only classify interior region contours as outlines (positive winding) or holes (negative winding).

Border regions don't have meaningful outline/hole topology. Skipping avoids "Bad outline for region" warnings.

---

## Step 10: Assemble the main function

**Ref:** `RecastContour.cpp:823-1104` (stock `rcBuildContours`)

```
1. Setup (stock lines 831-865)
   - Copy bmin/bmax with borderSize offset
   - Set cs/ch/width/height/borderSize/maxError on cset
   - Allocate contour array and flags array

2. Flag marking — Step 4 (stock lines 869-899, modified)

3. Contour walk loop (stock lines 906-1005)
   for each span with flags[i] != 0 && flags[i] != 0xf:
       Skip reg == 0
       DO NOT skip border regions (remove stock line 919 check)
       is_border_walk = (reg & RC_BORDER_REG) != 0

       Walk → raw_verts[]                      (Step 5)
       Segment identify + simplify → contour[]  (Step 6)
       Tessellate contour[] using raw_verts[]    (Step 7)
       Degenerate checks + store                (Step 8)

4. Hole merging — Step 9 (stock lines 1007-1101, skip border contours)
```

---

## Key Recast Source References

| What | File | Lines | Notes |
|------|------|-------|-------|
| `getCornerHeight` | RecastContour.cpp | 28-101 | Border vertex detection; modify lines 79-98 |
| `walkContour` | RecastContour.cpp | 103-184 | CW rotation line 155, CCW line 176 |
| `distancePtSeg` | RecastContour.cpp | 186-207 | XZ distance for simplification |
| `simplifyContour` | RecastContour.cpp | 209-451 | Mandatory detection 227-232, fallback 242-284, max-deviation 286-365, tessellation 367-440, reg assign 442-449 |
| `calcAreaOfPolygon2D` | RecastContour.cpp | 453-463 | Shoelace zero-area check |
| `removeDegenerateSegments` | RecastContour.cpp | 579-602 | Adjacent XZ-equal removal |
| `rcBuildContours` | RecastContour.cpp | 823-1104 | Flag marking 869-899, walk loop 906-1005, hole merge 1007-1101 |
| `paintRectRegion` | RecastRegion.cpp | 1575-1578 | 4 different border region IDs |
| Key constants | Recast.h | 585-623 | `RC_BORDER_REG`, `RC_BORDER_VERTEX`, `RC_AREA_BORDER`, `RC_CONTOUR_REG_MASK` |
| Data structures | Recast.h | 341-426 | `rcCompactSpan`, `rcContour`, `rcContourSet` |

---

## Design Decisions

1. **`cont->rverts`** — `nullptr` / `nrverts = 0`. Raw verts are a working array only.
2. **Hole merging** — Interior regions only. Border contours skipped.
3. **Tessellation** — Forced on ALL edge types. Direction-independent midpoint selection ensures cross-chunk matching.
4. **Simplification** — Forced on ALL segments (stock restricts to wall/area-border).
5. **No `buildFlags` parameter** — Tessellation and simplification always applied.
6. **`const chf`** — Y boundary marking is the caller's responsibility.
