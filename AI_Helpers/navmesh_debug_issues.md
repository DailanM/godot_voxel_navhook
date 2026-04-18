# Nav Mesh Debug Issues (2026-04-12)

## Current Test State
- Contour simplification: disabled (maxSimplificationError = 0)
- Erosion: disabled
- Low-height filter: disabled
- Ledge filter: enabled
- Low-hanging filter: enabled

## Issue 1: Y-Boundary Gap

**Symptom:** Horizontal gap between nav meshes of vertically adjacent chunks. The upper chunk's walkable area doesn't extend to the bottom of its owned Y range, leaving uncovered terrain at the boundary.

**Confirmed:** Disabling erosion mostly fixes this. The gap is caused by erosion eating into the walkable area at the Y boundary.

**Root cause (confirmed via testing):** Erosion marks cells as boundary (distance=0) when they have fewer than 4 connected walkable XZ neighbors. At the Y boundary, padding-zone spans lose walkability through some filter interaction, which disconnects boundary cells in the downslope direction, causing erosion to remove them.

**What we know:**
- Disabling erosion alone is a huge improvement — fixes the issue in most locations
- Some locations still have gaps even without erosion (remaining issue unclear)
- The rcMarkBoxArea cell partition itself appears to be clean (no overlap, no gap at the cell level)

**Status:** Erosion disabled as workaround. Need to investigate why padding-zone spans lose connectivity before erosion runs, likely due to interaction between the filters and the limited Y padding (pad_below = walkableClimb * ch).

---

## Issue 2: XZ-Boundary Vertex Mismatch (Ledge Filter Related)

**Symptom:** At horizontal (XZ) chunk boundaries, the nav mesh edges from adjacent chunks agree on position but have different vertex placement along shared edges. This prevents clean merging.

**Confirmed:** Disabling the ledge filter fixed the vertex mismatch at XZ boundaries in at least one observed location.

**Hypothesis:** The ledge filter (`rcFilterLedgeSpans`) operates on the raw heightfield and checks 4 XZ neighbors. At the XZ chunk boundary, the heightfield extends `borderSize` cells beyond the chunk. Near the boundary edge, the ledge filter may produce different results in adjacent chunks because:
- Each chunk's heightfield has different geometry beyond the border overlap zone
- The ledge filter's neighbor checks near the heightfield edge use a virtual ground at `-walkableClimb`, which can trigger false ledge detection
- These false ledge markings propagate inward through erosion (when enabled) or affect region building directly
- We had previouslu discussed the fact that it might have to do with simplifying the contour, and this seems likely since the edges between boundaries and parts that don't get merges have verticies that don't line up. How this interacts with the filterLedgeSpans function is unclear.

**What we know:**
- The issue is visible with contour simplification disabled (cell-level detail)
- Separate from the Y-boundary issue — occurs at same-Y-level XZ boundaries
- Disabling the ledge filter removes it (at least in the tested location)

**Status:** Not yet fixed. Needs further investigation into how the ledge filter interacts with border-zone geometry at XZ boundaries.

---

## Issue 3: Nav Mesh at Extreme Heights (Low-Height Filter Disabled)

**Symptom:** With the low-height filter disabled, some nav mesh polygons extend to enormous heights — entire chunks appear to get pushed up to extreme Y positions. This is NOT localized to Y boundaries; it happens in the interior of chunks as well.

**Observation:** The low-height filter (`rcFilterWalkableLowHeightSpans`) removes walkable spans that have insufficient clearance above them (gap < walkableHeight). When disabled, spans that would normally be filtered remain walkable.

**Hypothesis:** When neighbor chunks' geometry is rasterized, it creates thin solid shells at various heights throughout the heightfield. Normally the low-height filter kills walkable spans that are "trapped" below these shells (insufficient clearance). Without the filter, those phantom spans survive and produce nav mesh polygons. The detail mesh then connects vertices at wildly different Y positions, creating the extreme-height polygons.

**Key question:** Why would this affect entire chunks, not just boundary regions? Possible explanations:
- Neighbor mesh geometry from chunks at very different Y levels creates phantom solid spans throughout the heightfield, not just at boundaries
- The compact heightfield's `s.y` value (walkable floor) for these phantom spans could be at the maximum cell index, placing the nav mesh at the top of the heightfield
- The `getHeight()` function in the detail mesh might fall back to extreme values when looking up heights for these phantom spans

**Status:** Not investigated. The low-height filter cannot simply be disabled as a workaround because it causes this issue. If we need to disable it for Y-boundary fixes, we'd need to either:
1. Only disable it in the Y-padding zone (not the full heightfield)
2. Merge the overlapping solid shells before the filter runs so there's no false ceiling
3. Find and address the actual root cause of why entire chunks are affected

---
