#include "nav_build_contours.h"

#ifdef VOXEL_ENABLE_NAVIGATION

#include <RecastAlloc.h>
#include <RecastAssert.h>

#include <math.h>
#include <string.h>

// Cross-chunk-aware contour builder.
//
// Based on godot/thirdparty/recastnavigation/Recast/Source/RecastContour.cpp
// with modifications for tiled navmesh chunk boundary matching. See
// AI_Helpers/border_clipping_algorithm.md for the full algorithm specification.
//
// Key differences from stock rcBuildContours:
//
// - getCornerHeight: stock check kept verbatim (twoSameExts && twoInts &&
//   intsSameArea && noZeros) for RC_BORDER_VERTEX. This intsSameArea
//   condition is load-bearing for cross-chunk symmetry on area transitions:
//   at the area-T-junction along a seam, the two adjacent border cells have
//   different rasterized areas, twoSameExts fails (area is in the upper bits
//   of regs[]), and the vertex falls through to the regular mandatory path.
//   See getCornerHeight comments for the full case analysis.
//
// - Flag marking: border spans adjacent to interior regions retain flags
//   (stock zeros ALL border region flags).
//
// - walkContour: border walks use flag clearing guard and direction reversal
//   at mandatory non-RC_BORDER_VERTEX vertices (chunk corners, hole-boundary
//   transitions).
//
// - Contour building processes raw vertices segment-by-segment, equivalent
//   to the algorithm doc's SCANNING/COLLECTING streaming state machine but
//   expressed as a two-phase pass: first scan raw_verts for all mandatory
//   delimiters, then iterate consecutive delimiter pairs and simplify each
//   segment. ALL mandatory vertices (both border and non-border) act as
//   segment delimiters. Border mandatory vertices are silent delimiters —
//   they end/start segments but are not emitted. RC_BORDER_VERTEX vertices
//   within segments are excluded entirely. This produces the ear-clipping
//   effect: portal edges become straight lines between mandatory vertices
//   with no intermediate vertices.
//
// - Simplification forced on all segments (no edge-type restriction).
//   Tessellation forced on all edges, but skips RC_BORDER_VERTEX midpoints
//   to keep portal edges clean.
//
// - Hole merging skips border region contours.
// - cont->rverts = nullptr (raw verts are working array only).

namespace zylann::voxel {

namespace {

// ============================================================================
// getCornerHeight — stock border vertex check
// ============================================================================
// Ref: RecastContour.cpp:28-101
//
// Computes the maximum corner height at the vertex shared by cell (x,y) and
// its dir/dirp neighbors. Sets isBorderVertex when the vertex lies on a chunk
// boundary seam where two interior cells share the same area type.
//
// The intsSameArea condition is intentional and load-bearing for cross-chunk
// symmetry. At an area-transition T-junction along the seam, the two border
// cells have different rasterized area types — twoSameExts compares full
// reg|area words, so the area-typed difference makes twoSameExts FAIL. The
// vertex is correctly NOT flagged, allowing the simplifyContour mandatory
// detection (differentRegs || areaBorders) to emit it as a regular non-border
// mandatory vertex. Both adjacent chunks see this transition consistently
// because each chunk's border strip rasterizes from the same world voxels as
// the neighboring chunk's interior, and paintRectRegion (RecastRegion.cpp:1304)
// only writes srcReg — chf.areas is preserved.
//
// Vertices that are NOT flagged here and become regular mandatory vertices:
// - Chunk corners (different border region IDs across the seam): fail twoSameExts
// - Hole-boundary transitions (region 0 in the 4 cells): fail noZeros
// - Area-transition T-junctions (different rasterized areas in the border cells):
//   fail twoSameExts because area is part of the regs[] word
// - Region-only T-junctions (same area but different interior regions on either
//   side of the seam): NOT caught here — see Step 6 (buildContourFromSegments)
//   for the additional mandatory delimiter on RC_BORDER_VERTEX positions.
static int getCornerHeight(int x, int y, int i, int dir, const rcCompactHeightfield &chf, bool &isBorderVertex) {
	const rcCompactSpan &s = chf.spans[i];
	int ch = (int)s.y;
	int dirp = (dir + 1) & 0x3;

	unsigned int regs[4] = { 0, 0, 0, 0 };

	regs[0] = chf.spans[i].reg | (chf.areas[i] << 16);

	if (rcGetCon(s, dir) != RC_NOT_CONNECTED) {
		const int ax = x + rcGetDirOffsetX(dir);
		const int ay = y + rcGetDirOffsetY(dir);
		const int ai = (int)chf.cells[ax + ay * chf.width].index + rcGetCon(s, dir);
		const rcCompactSpan &as = chf.spans[ai];
		ch = rcMax(ch, (int)as.y);
		regs[1] = chf.spans[ai].reg | (chf.areas[ai] << 16);
		if (rcGetCon(as, dirp) != RC_NOT_CONNECTED) {
			const int ax2 = ax + rcGetDirOffsetX(dirp);
			const int ay2 = ay + rcGetDirOffsetY(dirp);
			const int ai2 = (int)chf.cells[ax2 + ay2 * chf.width].index + rcGetCon(as, dirp);
			const rcCompactSpan &as2 = chf.spans[ai2];
			ch = rcMax(ch, (int)as2.y);
			regs[2] = chf.spans[ai2].reg | (chf.areas[ai2] << 16);
		}
	}
	if (rcGetCon(s, dirp) != RC_NOT_CONNECTED) {
		const int ax = x + rcGetDirOffsetX(dirp);
		const int ay = y + rcGetDirOffsetY(dirp);
		const int ai = (int)chf.cells[ax + ay * chf.width].index + rcGetCon(s, dirp);
		const rcCompactSpan &as = chf.spans[ai];
		ch = rcMax(ch, (int)as.y);
		regs[3] = chf.spans[ai].reg | (chf.areas[ai] << 16);
		if (rcGetCon(as, dir) != RC_NOT_CONNECTED) {
			const int ax2 = ax + rcGetDirOffsetX(dir);
			const int ay2 = ay + rcGetDirOffsetY(dir);
			const int ai2 = (int)chf.cells[ax2 + ay2 * chf.width].index + rcGetCon(as, dir);
			const rcCompactSpan &as2 = chf.spans[ai2];
			ch = rcMax(ch, (int)as2.y);
			regs[2] = chf.spans[ai2].reg | (chf.areas[ai2] << 16);
		}
	}

	for (int j = 0; j < 4; ++j) {
		const int a = j;
		const int b = (j + 1) & 0x3;
		const int c = (j + 2) & 0x3;
		const int d = (j + 3) & 0x3;

		const bool twoSameExts = (regs[a] & regs[b] & RC_BORDER_REG) != 0 && regs[a] == regs[b];
		const bool twoInts = ((regs[c] | regs[d]) & RC_BORDER_REG) == 0;
		const bool intsSameArea = (regs[c] >> 16) == (regs[d] >> 16);
		const bool noZeros = regs[a] != 0 && regs[b] != 0 && regs[c] != 0 && regs[d] != 0;

		// Stock check: two same-ID exterior cells opposite two interior cells
		// of the same area type, all non-zero.
		if (twoSameExts && twoInts && intsSameArea && noZeros) {
			isBorderVertex = true;
			break;
		}
	}

	return ch;
}

// ============================================================================
// Helpers copied from stock RecastContour.cpp
// ============================================================================

// Ref: RecastContour.cpp:186-207
static float distancePtSeg(const int x, const int z, const int px, const int pz, const int qx, const int qz) {
	float pqx = (float)(qx - px);
	float pqz = (float)(qz - pz);
	float dx = (float)(x - px);
	float dz = (float)(z - pz);
	float d = pqx * pqx + pqz * pqz;
	float t = pqx * dx + pqz * dz;
	if (d > 0)
		t /= d;
	if (t < 0)
		t = 0;
	else if (t > 1)
		t = 1;

	dx = px + t * pqx - x;
	dz = pz + t * pqz - z;

	return dx * dx + dz * dz;
}

// Ref: RecastContour.cpp:579-602
static void removeDegenerateSegments(rcIntArray &simplified) {
	int npts = simplified.size() / 4;
	for (int i = 0; i < npts; ++i) {
		int ni = (i + 1 < npts) ? i + 1 : 0;

		if (simplified[i * 4 + 0] == simplified[ni * 4 + 0] &&
				simplified[i * 4 + 2] == simplified[ni * 4 + 2]) {
			for (int j = i; j < simplified.size() / 4 - 1; ++j) {
				simplified[j * 4 + 0] = simplified[(j + 1) * 4 + 0];
				simplified[j * 4 + 1] = simplified[(j + 1) * 4 + 1];
				simplified[j * 4 + 2] = simplified[(j + 1) * 4 + 2];
				simplified[j * 4 + 3] = simplified[(j + 1) * 4 + 3];
			}
			simplified.resize(simplified.size() - 4);
			npts--;
		}
	}
}

// ============================================================================
// Helpers copied from nav_build_contours_raw.cpp
// ============================================================================

static int calcAreaOfPolygon2D(const int *verts, const int nverts) {
	int area = 0;
	for (int i = 0, j = nverts - 1; i < nverts; j = i++) {
		const int *vi = &verts[i * 4];
		const int *vj = &verts[j * 4];
		area += vi[0] * vj[2] - vj[0] * vi[2];
	}
	return (area + 1) / 2;
}

inline int prev(int i, int n) { return i - 1 >= 0 ? i - 1 : n - 1; }
inline int next(int i, int n) { return i + 1 < n ? i + 1 : 0; }

inline int area2(const int *a, const int *b, const int *c) {
	return (b[0] - a[0]) * (c[2] - a[2]) - (c[0] - a[0]) * (b[2] - a[2]);
}

inline bool xorb(bool x, bool y) { return !x ^ !y; }
inline bool left(const int *a, const int *b, const int *c) { return area2(a, b, c) < 0; }
inline bool leftOn(const int *a, const int *b, const int *c) { return area2(a, b, c) <= 0; }
inline bool collinear(const int *a, const int *b, const int *c) { return area2(a, b, c) == 0; }

static bool intersectProp(const int *a, const int *b, const int *c, const int *d) {
	if (collinear(a, b, c) || collinear(a, b, d) || collinear(c, d, a) || collinear(c, d, b)) {
		return false;
	}
	return xorb(left(a, b, c), left(a, b, d)) && xorb(left(c, d, a), left(c, d, b));
}

static bool between(const int *a, const int *b, const int *c) {
	if (!collinear(a, b, c)) {
		return false;
	}
	if (a[0] != b[0]) {
		return ((a[0] <= c[0]) && (c[0] <= b[0])) || ((a[0] >= c[0]) && (c[0] >= b[0]));
	} else {
		return ((a[2] <= c[2]) && (c[2] <= b[2])) || ((a[2] >= c[2]) && (c[2] >= b[2]));
	}
}

static bool intersect(const int *a, const int *b, const int *c, const int *d) {
	if (intersectProp(a, b, c, d)) {
		return true;
	} else if (between(a, b, c) || between(a, b, d) || between(c, d, a) || between(c, d, b)) {
		return true;
	}
	return false;
}

static bool vequal(const int *a, const int *b) {
	return a[0] == b[0] && a[2] == b[2];
}

static bool intersectSegContour(const int *d0, const int *d1, int i, int n, const int *verts) {
	for (int k = 0; k < n; k++) {
		int k1 = next(k, n);
		if (i == k || i == k1) {
			continue;
		}
		const int *p0 = &verts[k * 4];
		const int *p1 = &verts[k1 * 4];
		if (vequal(d0, p0) || vequal(d1, p0) || vequal(d0, p1) || vequal(d1, p1)) {
			continue;
		}
		if (intersect(d0, d1, p0, p1)) {
			return true;
		}
	}
	return false;
}

static bool inCone(int i, int n, const int *verts, const int *pj) {
	const int *pi = &verts[i * 4];
	const int *pi1 = &verts[next(i, n) * 4];
	const int *pin1 = &verts[prev(i, n) * 4];
	if (leftOn(pin1, pi, pi1)) {
		return left(pi, pj, pin1) && left(pj, pi, pi1);
	}
	return !(leftOn(pi, pj, pi1) && leftOn(pj, pi, pin1));
}

// ============================================================================
// Hole merging machinery (copied from nav_build_contours_raw.cpp)
// ============================================================================

static bool mergeContours(rcContour &ca, rcContour &cb, int ia, int ib) {
	const int maxVerts = ca.nverts + cb.nverts + 2;
	int *verts = (int *)rcAlloc(sizeof(int) * maxVerts * 4, RC_ALLOC_PERM);
	if (!verts) {
		return false;
	}
	int nv = 0;
	for (int i = 0; i <= ca.nverts; ++i) {
		int *dst = &verts[nv * 4];
		const int *src = &ca.verts[((ia + i) % ca.nverts) * 4];
		dst[0] = src[0];
		dst[1] = src[1];
		dst[2] = src[2];
		dst[3] = src[3];
		nv++;
	}
	for (int i = 0; i <= cb.nverts; ++i) {
		int *dst = &verts[nv * 4];
		const int *src = &cb.verts[((ib + i) % cb.nverts) * 4];
		dst[0] = src[0];
		dst[1] = src[1];
		dst[2] = src[2];
		dst[3] = src[3];
		nv++;
	}
	rcFree(ca.verts);
	ca.verts = verts;
	ca.nverts = nv;
	rcFree(cb.verts);
	cb.verts = 0;
	cb.nverts = 0;
	return true;
}

struct rcContourHole {
	rcContour *contour;
	int minx, minz, leftmost;
};

struct rcContourRegion {
	rcContour *outline;
	rcContourHole *holes;
	int nholes;
};

struct rcPotentialDiagonal {
	int vert;
	int dist;
};

static void findLeftMostVertex(rcContour *contour, int *minx, int *minz, int *leftmost) {
	*minx = contour->verts[0];
	*minz = contour->verts[2];
	*leftmost = 0;
	for (int i = 1; i < contour->nverts; i++) {
		const int x = contour->verts[i * 4 + 0];
		const int z = contour->verts[i * 4 + 2];
		if (x < *minx || (x == *minx && z < *minz)) {
			*minx = x;
			*minz = z;
			*leftmost = i;
		}
	}
}

static int compareHoles(const void *va, const void *vb) {
	const rcContourHole *a = (const rcContourHole *)va;
	const rcContourHole *b = (const rcContourHole *)vb;
	if (a->minx == b->minx) {
		if (a->minz < b->minz) {
			return -1;
		}
		if (a->minz > b->minz) {
			return 1;
		}
	} else {
		if (a->minx < b->minx) {
			return -1;
		}
		if (a->minx > b->minx) {
			return 1;
		}
	}
	return 0;
}

static int compareDiagDist(const void *va, const void *vb) {
	const rcPotentialDiagonal *a = (const rcPotentialDiagonal *)va;
	const rcPotentialDiagonal *b = (const rcPotentialDiagonal *)vb;
	if (a->dist < b->dist) {
		return -1;
	}
	if (a->dist > b->dist) {
		return 1;
	}
	return 0;
}

static void mergeRegionHoles(rcContext *ctx, rcContourRegion &region) {
	for (int i = 0; i < region.nholes; i++) {
		findLeftMostVertex(region.holes[i].contour, &region.holes[i].minx, &region.holes[i].minz,
				&region.holes[i].leftmost);
	}
	qsort(region.holes, region.nholes, sizeof(rcContourHole), compareHoles);

	int maxVerts = region.outline->nverts;
	for (int i = 0; i < region.nholes; i++) {
		maxVerts += region.holes[i].contour->nverts;
	}

	rcScopedDelete<rcPotentialDiagonal> diags(
			(rcPotentialDiagonal *)rcAlloc(sizeof(rcPotentialDiagonal) * maxVerts, RC_ALLOC_TEMP));
	if (!diags) {
		ctx->log(RC_LOG_WARNING, "mergeRegionHoles: Failed to allocated diags %d.", maxVerts);
		return;
	}

	rcContour *outline = region.outline;

	for (int i = 0; i < region.nholes; i++) {
		rcContour *hole = region.holes[i].contour;
		int index = -1;
		int bestVertex = region.holes[i].leftmost;
		for (int iter = 0; iter < hole->nverts; iter++) {
			int ndiags = 0;
			const int *corner = &hole->verts[bestVertex * 4];
			for (int j = 0; j < outline->nverts; j++) {
				if (inCone(j, outline->nverts, outline->verts, corner)) {
					int dx = outline->verts[j * 4 + 0] - corner[0];
					int dz = outline->verts[j * 4 + 2] - corner[2];
					diags[ndiags].vert = j;
					diags[ndiags].dist = dx * dx + dz * dz;
					ndiags++;
				}
			}
			qsort(diags, ndiags, sizeof(rcPotentialDiagonal), compareDiagDist);
			index = -1;
			for (int j = 0; j < ndiags; j++) {
				const int *pt = &outline->verts[diags[j].vert * 4];
				bool intersectR = intersectSegContour(pt, corner, diags[i].vert, outline->nverts, outline->verts);
				for (int k = i; k < region.nholes && !intersectR; k++) {
					intersectR |= intersectSegContour(pt, corner, -1, region.holes[k].contour->nverts,
							region.holes[k].contour->verts);
				}
				if (!intersectR) {
					index = diags[j].vert;
					break;
				}
			}
			if (index != -1) {
				break;
			}
			bestVertex = (bestVertex + 1) % hole->nverts;
		}
		if (index == -1) {
			ctx->log(RC_LOG_WARNING, "mergeHoles: Failed to find merge points for %p and %p.", region.outline, hole);
			continue;
		}
		if (!mergeContours(*region.outline, *hole, index, bestVertex)) {
			ctx->log(RC_LOG_WARNING, "mergeHoles: Failed to merge contours %p and %p.", region.outline, hole);
			continue;
		}
	}
}

// ============================================================================
// walkContour — modified for border walks
// ============================================================================
// Ref: RecastContour.cpp:103-184
//
// Walks the contour of a single region, collecting raw vertices into points[].
// Each vertex is 4 ints: (x, y, z, r) where r encodes the neighbor region
// and RC_BORDER_VERTEX / RC_AREA_BORDER flags.
//
// Interior walks (is_border_walk=false) are identical to stock walkContour.
//
// Border walks (is_border_walk=true) hug only the inner edge — the boundary
// where the border region meets interior regions — walking back and forth
// between mandatory vertices. Only border spans adjacent to at least one
// interior region have flags set (Step 1), so the walk is confined to the
// innermost row of the border strip.
//
// Outgoing phase (CW): traces the inner edge clockwise. Flags are cleared
//   on border spans only (guard: chf.spans[i].reg & RC_BORDER_REG).
//
// Direction reversal: when the walk reaches a mandatory non-RC_BORDER_VERTEX
//   vertex (chunk corner or hole-boundary transition), the reversed flag
//   flips, swapping CW↔CCW rotation.
//
// Return phase (CCW): traces back along the inner edge. T-junctions into
//   interior cells are NOT followed. Flags are NOT cleared.
static void walkContour(int x, int y, int i, const rcCompactHeightfield &chf,
		unsigned char *flags, rcIntArray &points, bool is_border_walk) {
	unsigned char dir = 0;
	while ((flags[i] & (1 << dir)) == 0) {
		dir++;
	}

	unsigned char startDir = dir;
	int starti = i;

	const unsigned char area = chf.areas[i];

	bool reversed = false;
	int prev_r = -1;

	int iter = 0;
	while (++iter < 40000) {
		if (flags[i] & (1 << dir)) {
			// Boundary edge — emit vertex
			bool isBorderVertex = false;
			bool isAreaBorder = false;
			int px = x;
			int py = getCornerHeight(x, y, i, dir, chf, isBorderVertex);
			int pz = y;
			switch (dir) {
				case 0:
					pz++;
					break;
				case 1:
					px++;
					pz++;
					break;
				case 2:
					px++;
					break;
			}
			int r = 0;
			const rcCompactSpan &s = chf.spans[i];
			if (rcGetCon(s, dir) != RC_NOT_CONNECTED) {
				const int ax = x + rcGetDirOffsetX(dir);
				const int ay = y + rcGetDirOffsetY(dir);
				const int ai = (int)chf.cells[ax + ay * chf.width].index + rcGetCon(s, dir);
				r = (int)chf.spans[ai].reg;
				if (area != chf.areas[ai]) {
					isAreaBorder = true;
				}
			}
			if (isBorderVertex) {
				r |= RC_BORDER_VERTEX;
			}
			if (isAreaBorder) {
				r |= RC_AREA_BORDER;
			}
			points.push(px);
			points.push(py);
			points.push(pz);
			points.push(r);

			if (is_border_walk) {
				// Detect mandatory non-RC_BORDER_VERTEX transition → flip reversed.
				// Reversal triggers at chunk corners (transition between different
				// RC_BORDER_REG region IDs) and hole-boundary vertices (where
				// region 0 meets the border). Both are mandatory (r changes) and
				// NOT RC_BORDER_VERTEX (corners fail twoSameExts, holes fail
				// noZeros).
				if (prev_r >= 0) {
					const bool differentRegs =
							(prev_r & RC_CONTOUR_REG_MASK) != (r & RC_CONTOUR_REG_MASK);
					const bool areaBorderChange =
							((prev_r & RC_AREA_BORDER) != 0) != ((r & RC_AREA_BORDER) != 0);
					if ((differentRegs || areaBorderChange) && !(r & RC_BORDER_VERTEX)) {
						reversed = !reversed;
					}
				}
				prev_r = r;

				// Flag clearing: only on border spans, only during forward phase.
				if (!reversed && (chf.spans[i].reg & RC_BORDER_REG)) {
					flags[i] &= ~(1 << dir);
				}

				// Rotation: stock CW when forward, CCW when reversed
				if (reversed) {
					dir = (dir + 3) & 0x3; // CCW (reversed)
				} else {
					dir = (dir + 1) & 0x3; // CW (stock)
				}
			} else {
				// Interior walk: stock behavior
				flags[i] &= ~(1 << dir);
				dir = (dir + 1) & 0x3; // CW
			}
		} else {
			// No boundary edge — move to neighbor
			int ni = -1;
			const int nx = x + rcGetDirOffsetX(dir);
			const int ny = y + rcGetDirOffsetY(dir);
			const rcCompactSpan &s = chf.spans[i];
			if (rcGetCon(s, dir) != RC_NOT_CONNECTED) {
				const rcCompactCell &nc = chf.cells[nx + ny * chf.width];
				ni = (int)nc.index + rcGetCon(s, dir);
			}
			if (ni == -1) {
				return;
			}

			// T-junction guard: during reversed phase, don't step into
			// interior cells. The walk stays on border cells and rotates
			// to find the next boundary edge or border-cell neighbor.
			if (is_border_walk && reversed && !(chf.spans[ni].reg & RC_BORDER_REG)) {
				dir = (dir + 1) & 0x3; // Rotate CW (reversed move direction)
			} else {
				x = nx;
				y = ny;
				i = ni;
				if (is_border_walk && reversed) {
					dir = (dir + 1) & 0x3; // CW (reversed)
				} else {
					dir = (dir + 3) & 0x3; // CCW (stock)
				}
			}
		}

		if (starti == i && startDir == dir) {
			break;
		}
	}
}

// ============================================================================
// simplifySegment — max-deviation simplification
// ============================================================================
// Ref: RecastContour.cpp:286-365
//
// Simplifies the raw vertices between two anchor points (raw indices s0 and
// s_end in raw_verts). Seeds the output with the two anchors, then iteratively
// inserts the raw vertex with maximum perpendicular deviation until all
// deviations are within maxError. RC_BORDER_VERTEX vertices are skipped during
// deviation evaluation.
//
// Lexicographic XZ ordering (RecastContour.cpp:309-322) ensures both chunks
// traversing a shared edge in opposite directions select the same
// max-deviation vertex.
//
// wrap_around controls whether the (last, first) pair is also processed:
//
// - wrap_around=false (per-segment): only refines pairs (i, i+1) for
//   i in [0, size-2]. The traversal stays within the single segment from
//   s0 forward to s_end. Used when invoked once per segment between mandatory
//   delimiters — vertices outside the segment must NOT be considered.
//
// - wrap_around=true (full contour): also refines the (last, first) pair,
//   covering both halves of the closed loop. Used by the bounding-box
//   fallback when no mandatory vertices exist and the entire contour is
//   one big segment.
static void simplifySegment(const rcIntArray &raw_verts, int pn, int s0, int s_end,
		float maxError, rcIntArray &out, bool wrap_around) {
	out.clear();

	// Seed with the two endpoints
	out.push(raw_verts[s0 * 4 + 0]);
	out.push(raw_verts[s0 * 4 + 1]);
	out.push(raw_verts[s0 * 4 + 2]);
	out.push(s0);

	if (s0 != s_end) {
		out.push(raw_verts[s_end * 4 + 0]);
		out.push(raw_verts[s_end * 4 + 1]);
		out.push(raw_verts[s_end * 4 + 2]);
		out.push(s_end);
	}

	// Max-deviation loop. The bound (out.size()/4 - (wrap_around ? 0 : 1))
	// is recomputed each iteration so insertions extending the array
	// continue to be processed. With wrap_around=false the (last, first)
	// pair is skipped — the loop only refines pairs strictly within the
	// segment.
	for (int i = 0; i < (int)(out.size() / 4) - (wrap_around ? 0 : 1);) {
		int ii = (i + 1) % (out.size() / 4);

		int ax = out[i * 4 + 0];
		int az = out[i * 4 + 2];
		int ai = out[i * 4 + 3];

		int bx = out[ii * 4 + 0];
		int bz = out[ii * 4 + 2];
		int bi = out[ii * 4 + 3];

		float maxd = 0;
		int maxi = -1;
		int ci, cinc, endi;

		// Traverse in lexicographic XZ order for direction independence
		if (bx > ax || (bx == ax && bz > az)) {
			cinc = 1;
			ci = (ai + cinc) % pn;
			endi = bi;
		} else {
			cinc = pn - 1;
			ci = (bi + cinc) % pn;
			endi = ai;
			rcSwap(ax, bx);
			rcSwap(az, bz);
		}

		while (ci != endi) {
			if (!(raw_verts[ci * 4 + 3] & RC_BORDER_VERTEX)) {
				float d = distancePtSeg(raw_verts[ci * 4 + 0], raw_verts[ci * 4 + 2], ax, az, bx, bz);
				if (d > maxd) {
					maxd = d;
					maxi = ci;
				}
			}
			ci = (ci + cinc) % pn;
		}

		if (maxi != -1 && maxd > (maxError * maxError)) {
			out.resize(out.size() + 4);
			const int n = out.size() / 4;
			for (int j = n - 1; j > i; --j) {
				out[j * 4 + 0] = out[(j - 1) * 4 + 0];
				out[j * 4 + 1] = out[(j - 1) * 4 + 1];
				out[j * 4 + 2] = out[(j - 1) * 4 + 2];
				out[j * 4 + 3] = out[(j - 1) * 4 + 3];
			}
			out[(i + 1) * 4 + 0] = raw_verts[maxi * 4 + 0];
			out[(i + 1) * 4 + 1] = raw_verts[maxi * 4 + 1];
			out[(i + 1) * 4 + 2] = raw_verts[maxi * 4 + 2];
			out[(i + 1) * 4 + 3] = maxi;
		} else {
			++i;
		}
	}
}

// ============================================================================
// buildContourFromSegments — segment-by-segment simplification
// ============================================================================
// Ref: border_clipping_algorithm.md "Walk-and-Simplify State Machine"
//
// Functionally equivalent to the algorithm document's SCANNING/COLLECTING
// streaming state machine, expressed as two phases on the already-walked
// raw_verts array:
//
//   1. Scan raw_verts for ALL mandatory delimiters (region or area-border
//      transitions). Each delimiter is recorded with its border-vertex flag.
//      Mandatory wrap-around (last vertex vs first vertex) is handled
//      naturally by the modular comparison r[i] != r[(i+1)%pn].
//
//   2. For each consecutive pair of delimiters (mandatory[m], mandatory[m+1]),
//      build the segment: collect non-border raw vertex indices between (and
//      including, if non-border) the two delimiters. Simplify each segment
//      independently with simplifySegment. Emit simplified vertices to the
//      output contour, omitting the last vertex when the end delimiter is
//      non-border (it will be emitted as the start of the next segment).
//
// Border mandatory vertices ("silent delimiters" in the algorithm doc) end
// and start segments without being included in either. RC_BORDER_VERTEX
// non-mandatory vertices are excluded from all segments. This produces the
// ear-clipping effect: portal edges become straight lines between mandatory
// vertices with no intermediate vertices that could disagree between chunks.
//
// The two-phase approach produces the same output as the streaming state
// machine. Iterating over delimiter pairs makes the wrap-around case implicit
// via mandatory[(m+1) % nm], avoiding a separate prefix[] for the SCANNING
// remainder.
//
// Contour output stores raw vertex indices in the 4th component. Tessellation
// and reg assignment are applied afterward by the caller.
static void buildContourFromSegments(const rcIntArray &raw_verts, rcIntArray &contour, float maxError) {
	const int pn = raw_verts.size() / 4;
	if (pn < 3) {
		return;
	}

	// Find ALL mandatory vertices — both border and non-border.
	// Vertex i is mandatory when r[i] != r[(i+1)%pn] (stock detection,
	// RecastContour.cpp:227-232). This handles wrap-around naturally.
	// Border mandatory vertices are silent delimiters; non-border ones are
	// emitted and included in segments.
	rcIntArray mandatory_raw_idx(16);
	rcIntArray mandatory_is_bv(16);

	for (int i = 0; i < pn; ++i) {
		int ii = (i + 1) % pn;
		const bool differentRegs =
				(raw_verts[i * 4 + 3] & RC_CONTOUR_REG_MASK) != (raw_verts[ii * 4 + 3] & RC_CONTOUR_REG_MASK);
		const bool areaBorders =
				(raw_verts[i * 4 + 3] & RC_AREA_BORDER) != (raw_verts[ii * 4 + 3] & RC_AREA_BORDER);
		if (differentRegs || areaBorders) {
			mandatory_raw_idx.push(i);
			mandatory_is_bv.push((raw_verts[i * 4 + 3] & RC_BORDER_VERTEX) ? 1 : 0);
		}
	}

	// --- Fallback: no mandatory vertices found (stock lines 242-284) ---
	// Seed with lower-left and upper-right bounding box extremes, then run
	// max-deviation on the entire contour using wrap_around=true so both
	// halves of the closed loop are simplified.
	if (mandatory_raw_idx.size() == 0) {
		int llx = raw_verts[0];
		int llz = raw_verts[2];
		int lli = 0;
		int urx = raw_verts[0];
		int urz = raw_verts[2];
		int uri = 0;
		for (int i = 0; i < raw_verts.size(); i += 4) {
			int x = raw_verts[i + 0];
			int z = raw_verts[i + 2];
			if (x < llx || (x == llx && z < llz)) {
				llx = x;
				llz = z;
				lli = i / 4;
			}
			if (x > urx || (x == urx && z > urz)) {
				urx = x;
				urz = z;
				uri = i / 4;
			}
		}

		simplifySegment(raw_verts, pn, lli, uri, maxError, contour, /*wrap_around*/ true);
		return;
	}

	// --- Process segments between consecutive mandatory vertices ---
	// Each segment spans raw_verts from mandatory[m] to mandatory[(m+1)%nm].
	// Non-border mandatory vertices are included in their segment as
	// endpoints. Border mandatory vertices are excluded (silent delimiters).
	// All RC_BORDER_VERTEX intermediates are excluded.
	//
	// For each segment:
	// - Build seg[]: the non-border raw vertex indices in this segment
	// - If empty: skip (all vertices were RC_BORDER_VERTEX)
	// - If 1 vertex: emit directly (unless it's the end vertex of a
	//   non-border-ended segment, which the next segment will emit)
	// - If 2+ vertices: simplify with seg[0] and seg[last] as seeds
	// - Emit rule: if end delimiter is non-border, omit last simplified
	//   vertex (it starts the next segment). If end is border, emit all.
	const int nm = mandatory_raw_idx.size();
	rcIntArray simplified(32);

	for (int m = 0; m < nm; ++m) {
		const int start_raw = mandatory_raw_idx[m];
		const bool start_is_bv = mandatory_is_bv[m] != 0;
		const int end_raw = mandatory_raw_idx[(m + 1) % nm];
		const bool end_is_bv = mandatory_is_bv[(m + 1) % nm] != 0;

		// Collect non-border raw vertex indices in this segment
		rcIntArray seg(32);

		// Include start mandatory vertex if non-border
		if (!start_is_bv) {
			seg.push(start_raw);
		}

		// Add intermediate non-border vertices
		int ci = (start_raw + 1) % pn;
		while (ci != end_raw) {
			if (!(raw_verts[ci * 4 + 3] & RC_BORDER_VERTEX)) {
				seg.push(ci);
			}
			ci = (ci + 1) % pn;
		}

		// Include end mandatory vertex if non-border
		if (!end_is_bv) {
			seg.push(end_raw);
		}

		if (seg.size() == 0) {
			continue;
		}

		if (seg.size() == 1) {
			// Single vertex — emit only if the next segment won't emit it.
			// If end is non-border and this vertex IS end_raw, the next
			// segment will include it as its start. Skip to avoid duplication.
			if (!end_is_bv && seg[0] == end_raw) {
				continue;
			}
			int ri = seg[0];
			contour.push(raw_verts[ri * 4 + 0]);
			contour.push(raw_verts[ri * 4 + 1]);
			contour.push(raw_verts[ri * 4 + 2]);
			contour.push(ri);
			continue;
		}

		// 2+ vertices — simplify segment (per-segment, no wrap-around).
		// The traversal must stay within the segment from seg[0] forward to
		// seg[last]; the (last, first) wrap-around pair would step outside
		// the segment into vertices belonging to other segments.
		simplifySegment(raw_verts, pn, seg[0], seg[seg.size() - 1], maxError, simplified, /*wrap_around*/ false);

		// Emit: omit last vertex if end delimiter is non-border (it starts
		// the next segment and will be emitted there).
		int emit_count = end_is_bv ? simplified.size() / 4 : simplified.size() / 4 - 1;
		for (int j = 0; j < emit_count; ++j) {
			contour.push(simplified[j * 4 + 0]);
			contour.push(simplified[j * 4 + 1]);
			contour.push(simplified[j * 4 + 2]);
			contour.push(simplified[j * 4 + 3]);
		}
	}
}

// ============================================================================
// tessellateContour — split long edges by inserting raw midpoints
// ============================================================================
// Ref: RecastContour.cpp:367-440
//
// Post-processing pass on the completed contour[]. Splits simplified edges
// that exceed maxEdgeLen by inserting midpoint raw vertices. Forced on ALL
// edge types (no buildFlags restriction). Midpoints that are RC_BORDER_VERTEX
// are skipped to keep portal edges free of intermediate vertices.
//
// Direction-independent midpoint selection (RecastContour.cpp:407-411) ensures
// both chunks on a shared edge pick the same split vertex.
static void tessellateContour(rcIntArray &contour, const rcIntArray &raw_verts, int pn, int maxEdgeLen) {
	if (maxEdgeLen <= 0) {
		return;
	}

	for (int i = 0; i < contour.size() / 4;) {
		const int ii = (i + 1) % (contour.size() / 4);

		const int ax = contour[i * 4 + 0];
		const int az = contour[i * 4 + 2];
		const int ai = contour[i * 4 + 3];

		const int bx = contour[ii * 4 + 0];
		const int bz = contour[ii * 4 + 2];
		const int bi = contour[ii * 4 + 3];

		int maxi = -1;

		int dx = bx - ax;
		int dz = bz - az;
		if (dx * dx + dz * dz > maxEdgeLen * maxEdgeLen) {
			const int n = bi < ai ? (bi + pn - ai) : (bi - ai);
			if (n > 1) {
				// Direction-independent midpoint selection
				if (bx > ax || (bx == ax && bz > az)) {
					maxi = (ai + n / 2) % pn;
				} else {
					maxi = (ai + (n + 1) / 2) % pn;
				}
			}
		}

		// Skip midpoints that are RC_BORDER_VERTEX — portal edges must remain
		// straight lines with no intermediate vertices.
		if (maxi != -1 && (raw_verts[maxi * 4 + 3] & RC_BORDER_VERTEX)) {
			maxi = -1;
		}

		if (maxi != -1) {
			contour.resize(contour.size() + 4);
			const int n = contour.size() / 4;
			for (int j = n - 1; j > i; --j) {
				contour[j * 4 + 0] = contour[(j - 1) * 4 + 0];
				contour[j * 4 + 1] = contour[(j - 1) * 4 + 1];
				contour[j * 4 + 2] = contour[(j - 1) * 4 + 2];
				contour[j * 4 + 3] = contour[(j - 1) * 4 + 3];
			}
			contour[(i + 1) * 4 + 0] = raw_verts[maxi * 4 + 0];
			contour[(i + 1) * 4 + 1] = raw_verts[maxi * 4 + 1];
			contour[(i + 1) * 4 + 2] = raw_verts[maxi * 4 + 2];
			contour[(i + 1) * 4 + 3] = maxi;
		} else {
			++i;
		}
	}
}

// ============================================================================
// assignContourRegValues — replace raw indices with region/flag values
// ============================================================================
// Ref: RecastContour.cpp:442-449
//
// Replaces the raw vertex index in contour[i*4+3] with the actual
// neighbor-region and flag value. The neighbor region and RC_AREA_BORDER
// come from the NEXT raw vertex (ai), matching stock behavior.
//
// Since all emitted contour vertices are non-RC_BORDER_VERTEX (border
// vertices are excluded from segments entirely), the RC_BORDER_VERTEX bit
// is always 0 and does not need to be OR'd in from raw_verts[bi].
static void assignContourRegValues(rcIntArray &contour, const rcIntArray &raw_verts, int pn) {
	for (int i = 0; i < contour.size() / 4; ++i) {
		const int ai = (contour[i * 4 + 3] + 1) % pn;
		contour[i * 4 + 3] = raw_verts[ai * 4 + 3] & (RC_CONTOUR_REG_MASK | RC_AREA_BORDER);
	}
}

} // anonymous namespace

// ============================================================================
// Main function
// ============================================================================
bool nav_build_contours(rcContext *ctx, const rcCompactHeightfield &chf,
		float maxError, int maxEdgeLen, rcContourSet &cset) {
	rcAssert(ctx);

	const int w = chf.width;
	const int h = chf.height;
	const int borderSize = chf.borderSize;

	// --- Setup (stock lines 831-865) ---
	rcVcopy(cset.bmin, chf.bmin);
	rcVcopy(cset.bmax, chf.bmax);
	if (borderSize > 0) {
		const float pad = borderSize * chf.cs;
		cset.bmin[0] += pad;
		cset.bmin[2] += pad;
		cset.bmax[0] -= pad;
		cset.bmax[2] -= pad;
	}
	cset.cs = chf.cs;
	cset.ch = chf.ch;
	cset.width = chf.width - chf.borderSize * 2;
	cset.height = chf.height - chf.borderSize * 2;
	cset.borderSize = chf.borderSize;
	cset.maxError = maxError;

	int maxContours = rcMax((int)chf.maxRegions, 8);
	cset.conts = (rcContour *)rcAlloc(sizeof(rcContour) * maxContours, RC_ALLOC_PERM);
	if (!cset.conts) {
		return false;
	}
	cset.nconts = 0;

	rcScopedDelete<unsigned char> flags((unsigned char *)rcAlloc(sizeof(unsigned char) * chf.spanCount, RC_ALLOC_TEMP));
	if (!flags) {
		ctx->log(RC_LOG_ERROR, "nav_build_contours: Out of memory 'flags' (%d).", chf.spanCount);
		return false;
	}

	// --- Modified flag marking (stock lines 869-899) ---
	// Stock zeros flags for ALL border-region spans. Modified to retain
	// flags on border spans adjacent to at least one interior region.
	for (int y = 0; y < h; ++y) {
		for (int x = 0; x < w; ++x) {
			const rcCompactCell &c = chf.cells[x + y * w];
			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i) {
				const rcCompactSpan &s = chf.spans[i];
				const unsigned short reg = chf.spans[i].reg;

				if (reg == 0) {
					flags[i] = 0;
					continue;
				}

				if (reg & RC_BORDER_REG) {
					bool adjacent_to_interior = false;
					for (int dir = 0; dir < 4; ++dir) {
						if (rcGetCon(s, dir) != RC_NOT_CONNECTED) {
							const int ax = x + rcGetDirOffsetX(dir);
							const int ay = y + rcGetDirOffsetY(dir);
							const int ai = (int)chf.cells[ax + ay * w].index + rcGetCon(s, dir);
							const unsigned short neighbor_reg = chf.spans[ai].reg;
							if (neighbor_reg != 0 && !(neighbor_reg & RC_BORDER_REG)) {
								adjacent_to_interior = true;
								break;
							}
						}
					}
					if (!adjacent_to_interior) {
						flags[i] = 0;
						continue;
					}
				}

				unsigned char res = 0;
				for (int dir = 0; dir < 4; ++dir) {
					unsigned short r = 0;
					if (rcGetCon(s, dir) != RC_NOT_CONNECTED) {
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.cells[ax + ay * w].index + rcGetCon(s, dir);
						r = chf.spans[ai].reg;
					}
					if (r == reg) {
						res |= (1 << dir);
					}
				}
				flags[i] = res ^ 0xf;
			}
		}
	}

	// --- Contour walk + simplify + tessellate + store loop ---
	rcIntArray verts(256);
	rcIntArray simplified(64);

	for (int y = 0; y < h; ++y) {
		for (int x = 0; x < w; ++x) {
			const rcCompactCell &c = chf.cells[x + y * w];
			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i) {
				if (flags[i] == 0 || flags[i] == 0xf) {
					flags[i] = 0;
					continue;
				}
				const unsigned short reg = chf.spans[i].reg;
				if (!reg) {
					continue;
				}
				const unsigned char area = chf.areas[i];
				const bool is_border_walk = (reg & RC_BORDER_REG) != 0;

				verts.clear();
				simplified.clear();

				// Walk the contour — produces raw_verts
				walkContour(x, y, i, chf, flags, verts, is_border_walk);

				const int pn = verts.size() / 4;
				if (pn < 3) {
					continue;
				}

				// Build contour from segments (state machine + per-segment
				// simplification). Output has raw indices in 4th component.
				buildContourFromSegments(verts, simplified, maxError);

				// Tessellate long edges (skip RC_BORDER_VERTEX midpoints)
				tessellateContour(simplified, verts, pn, maxEdgeLen);

				// Replace raw indices with region/flag values
				assignContourRegValues(simplified, verts, pn);

				// Remove degenerate segments
				removeDegenerateSegments(simplified);

				// Discard contours with < 3 vertices or zero area
				if (simplified.size() / 4 < 3) {
					continue;
				}
				if (calcAreaOfPolygon2D(&simplified[0], simplified.size() / 4) == 0) {
					continue;
				}

				// Store contour
				if (cset.nconts >= maxContours) {
					const int oldMax = maxContours;
					maxContours *= 2;
					rcContour *newConts = (rcContour *)rcAlloc(sizeof(rcContour) * maxContours, RC_ALLOC_PERM);
					for (int j = 0; j < cset.nconts; ++j) {
						newConts[j] = cset.conts[j];
						cset.conts[j].verts = 0;
						cset.conts[j].rverts = 0;
					}
					rcFree(cset.conts);
					cset.conts = newConts;
					ctx->log(RC_LOG_WARNING, "nav_build_contours: Expanding max contours from %d to %d.",
							oldMax, maxContours);
				}

				rcContour *cont = &cset.conts[cset.nconts++];

				cont->nverts = simplified.size() / 4;
				cont->verts = (int *)rcAlloc(sizeof(int) * cont->nverts * 4, RC_ALLOC_PERM);
				if (!cont->verts) {
					ctx->log(RC_LOG_ERROR, "nav_build_contours: Out of memory 'verts' (%d).", cont->nverts);
					return false;
				}
				memcpy(cont->verts, &simplified[0], sizeof(int) * cont->nverts * 4);
				if (borderSize > 0) {
					for (int j = 0; j < cont->nverts; ++j) {
						int *v = &cont->verts[j * 4];
						v[0] -= borderSize;
						v[2] -= borderSize;
					}
				}

				cont->rverts = 0;
				cont->nrverts = 0;

				cont->reg = reg;
				cont->area = area;
			}
		}
	}

	// --- Hole merging (stock lines 1007-1101) ---
	// Border region contours are skipped — they have no meaningful
	// outline/hole topology.
	if (cset.nconts > 0) {
		rcScopedDelete<signed char> winding((signed char *)rcAlloc(sizeof(signed char) * cset.nconts, RC_ALLOC_TEMP));
		if (!winding) {
			ctx->log(RC_LOG_ERROR, "nav_build_contours: Out of memory 'hole' (%d).", cset.nconts);
			return false;
		}
		int nholes = 0;
		for (int i = 0; i < cset.nconts; ++i) {
			rcContour &cont = cset.conts[i];
			if (cont.reg & RC_BORDER_REG) {
				winding[i] = 0;
				continue;
			}
			winding[i] = calcAreaOfPolygon2D(cont.verts, cont.nverts) < 0 ? -1 : 1;
			if (winding[i] < 0) {
				nholes++;
			}
		}

		if (nholes > 0) {
			const int nregions = chf.maxRegions + 1;
			rcScopedDelete<rcContourRegion> regions(
					(rcContourRegion *)rcAlloc(sizeof(rcContourRegion) * nregions, RC_ALLOC_TEMP));
			if (!regions) {
				ctx->log(RC_LOG_ERROR, "nav_build_contours: Out of memory 'regions' (%d).", nregions);
				return false;
			}
			memset(regions, 0, sizeof(rcContourRegion) * nregions);

			rcScopedDelete<rcContourHole> holes(
					(rcContourHole *)rcAlloc(sizeof(rcContourHole) * cset.nconts, RC_ALLOC_TEMP));
			if (!holes) {
				ctx->log(RC_LOG_ERROR, "nav_build_contours: Out of memory 'holes' (%d).", cset.nconts);
				return false;
			}
			memset(holes, 0, sizeof(rcContourHole) * cset.nconts);

			for (int i = 0; i < cset.nconts; ++i) {
				rcContour &cont = cset.conts[i];
				if (cont.reg & RC_BORDER_REG) {
					continue;
				}
				if (winding[i] > 0) {
					if (regions[cont.reg].outline) {
						ctx->log(RC_LOG_ERROR, "nav_build_contours: Multiple outlines for region %d.", cont.reg);
					}
					regions[cont.reg].outline = &cont;
				} else {
					regions[cont.reg].nholes++;
				}
			}
			int index = 0;
			for (int i = 0; i < nregions; i++) {
				if (regions[i].nholes > 0) {
					regions[i].holes = &holes[index];
					index += regions[i].nholes;
					regions[i].nholes = 0;
				}
			}
			for (int i = 0; i < cset.nconts; ++i) {
				rcContour &cont = cset.conts[i];
				if (cont.reg & RC_BORDER_REG) {
					continue;
				}
				rcContourRegion &reg = regions[cont.reg];
				if (winding[i] < 0) {
					reg.holes[reg.nholes++].contour = &cont;
				}
			}

			for (int i = 0; i < nregions; i++) {
				rcContourRegion &reg = regions[i];
				if (!reg.nholes) {
					continue;
				}
				if (reg.outline) {
					mergeRegionHoles(ctx, reg);
				} else {
					ctx->log(RC_LOG_ERROR, "nav_build_contours: Bad outline for region %d.", i);
				}
			}
		}
	}

	return true;
}

} // namespace zylann::voxel

#endif // VOXEL_ENABLE_NAVIGATION
