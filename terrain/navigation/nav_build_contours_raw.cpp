#include "nav_build_contours_raw.h"

#ifdef VOXEL_ENABLE_NAVIGATION

#include <RecastAlloc.h>
#include <RecastAssert.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// This file is a trimmed copy of
// godot/thirdparty/recastnavigation/Recast/Source/RecastContour.cpp
// with simplifyContour() and removeDegenerateSegments() removed.  The
// raw output of walkContour() is written straight into rcContour::verts
// so we can see every per-edge vertex that the contour walk produced,
// before any simplification collapses them.
//
// Functions that simplifyContour/removeDegenerateSegments were the sole
// users of (distancePtSeg) are omitted.  Hole-merging machinery is kept
// so the output is still a well-formed rcContourSet.

namespace zylann::voxel {

namespace {

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
		if (twoSameExts && twoInts && intsSameArea && noZeros) {
			isBorderVertex = true;
			break;
		}
	}

	return ch;
}

static void walkContour(int x, int y, int i, const rcCompactHeightfield &chf, unsigned char *flags, rcIntArray &points) {
	unsigned char dir = 0;
	while ((flags[i] & (1 << dir)) == 0) {
		dir++;
	}

	unsigned char startDir = dir;
	int starti = i;

	const unsigned char area = chf.areas[i];

	int iter = 0;
	while (++iter < 40000) {
		if (flags[i] & (1 << dir)) {
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

			flags[i] &= ~(1 << dir);
			dir = (dir + 1) & 0x3;
		} else {
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
			x = nx;
			y = ny;
			i = ni;
			dir = (dir + 3) & 0x3;
		}

		if (starti == i && startDir == dir) {
			break;
		}
	}
}

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

} // namespace

bool nav_build_contours_raw(rcContext *ctx, const rcCompactHeightfield &chf, rcContourSet &cset) {
	rcAssert(ctx);

	const int w = chf.width;
	const int h = chf.height;
	const int borderSize = chf.borderSize;

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
	cset.maxError = 0.0f;

	int maxContours = rcMax((int)chf.maxRegions, 8);
	cset.conts = (rcContour *)rcAlloc(sizeof(rcContour) * maxContours, RC_ALLOC_PERM);
	if (!cset.conts) {
		return false;
	}
	cset.nconts = 0;

	rcScopedDelete<unsigned char> flags((unsigned char *)rcAlloc(sizeof(unsigned char) * chf.spanCount, RC_ALLOC_TEMP));
	if (!flags) {
		ctx->log(RC_LOG_ERROR, "nav_build_contours_raw: Out of memory 'flags' (%d).", chf.spanCount);
		return false;
	}

	// Mark boundaries — bit set per dir where neighbor is in a different region.
	for (int y = 0; y < h; ++y) {
		for (int x = 0; x < w; ++x) {
			const rcCompactCell &c = chf.cells[x + y * w];
			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i) {
				unsigned char res = 0;
				const rcCompactSpan &s = chf.spans[i];
				if (!chf.spans[i].reg || (chf.spans[i].reg & RC_BORDER_REG)) {
					flags[i] = 0;
					continue;
				}
				for (int dir = 0; dir < 4; ++dir) {
					unsigned short r = 0;
					if (rcGetCon(s, dir) != RC_NOT_CONNECTED) {
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.cells[ax + ay * w].index + rcGetCon(s, dir);
						r = chf.spans[ai].reg;
					}
					if (r == chf.spans[i].reg) {
						res |= (1 << dir);
					}
				}
				flags[i] = res ^ 0xf;
			}
		}
	}

	rcIntArray verts(256);

	for (int y = 0; y < h; ++y) {
		for (int x = 0; x < w; ++x) {
			const rcCompactCell &c = chf.cells[x + y * w];
			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i) {
				if (flags[i] == 0 || flags[i] == 0xf) {
					flags[i] = 0;
					continue;
				}
				const unsigned short reg = chf.spans[i].reg;
				if (!reg || (reg & RC_BORDER_REG)) {
					continue;
				}
				const unsigned char area = chf.areas[i];

				verts.clear();
				walkContour(x, y, i, chf, flags, verts);

				// === simplify / removeDegenerateSegments intentionally skipped ===
				// The raw verts array is used directly as the contour output.

				if (verts.size() / 4 >= 3) {
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
						ctx->log(RC_LOG_WARNING, "nav_build_contours_raw: Expanding max contours from %d to %d.",
								oldMax, maxContours);
					}

					rcContour *cont = &cset.conts[cset.nconts++];

					cont->nverts = verts.size() / 4;
					cont->verts = (int *)rcAlloc(sizeof(int) * cont->nverts * 4, RC_ALLOC_PERM);
					if (!cont->verts) {
						ctx->log(RC_LOG_ERROR, "nav_build_contours_raw: Out of memory 'verts' (%d).", cont->nverts);
						return false;
					}
					memcpy(cont->verts, &verts[0], sizeof(int) * cont->nverts * 4);
					if (borderSize > 0) {
						for (int j = 0; j < cont->nverts; ++j) {
							int *v = &cont->verts[j * 4];
							v[0] -= borderSize;
							v[2] -= borderSize;
						}
					}

					cont->nrverts = verts.size() / 4;
					cont->rverts = (int *)rcAlloc(sizeof(int) * cont->nrverts * 4, RC_ALLOC_PERM);
					if (!cont->rverts) {
						ctx->log(RC_LOG_ERROR, "nav_build_contours_raw: Out of memory 'rverts' (%d).", cont->nrverts);
						return false;
					}
					memcpy(cont->rverts, &verts[0], sizeof(int) * cont->nrverts * 4);
					if (borderSize > 0) {
						for (int j = 0; j < cont->nrverts; ++j) {
							int *v = &cont->rverts[j * 4];
							v[0] -= borderSize;
							v[2] -= borderSize;
						}
					}

					cont->reg = reg;
					cont->area = area;
				}
			}
		}
	}

	// Merge holes.
	if (cset.nconts > 0) {
		rcScopedDelete<signed char> winding((signed char *)rcAlloc(sizeof(signed char) * cset.nconts, RC_ALLOC_TEMP));
		if (!winding) {
			ctx->log(RC_LOG_ERROR, "nav_build_contours_raw: Out of memory 'hole' (%d).", cset.nconts);
			return false;
		}
		int nholes = 0;
		for (int i = 0; i < cset.nconts; ++i) {
			rcContour &cont = cset.conts[i];
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
				ctx->log(RC_LOG_ERROR, "nav_build_contours_raw: Out of memory 'regions' (%d).", nregions);
				return false;
			}
			memset(regions, 0, sizeof(rcContourRegion) * nregions);

			rcScopedDelete<rcContourHole> holes(
					(rcContourHole *)rcAlloc(sizeof(rcContourHole) * cset.nconts, RC_ALLOC_TEMP));
			if (!holes) {
				ctx->log(RC_LOG_ERROR, "nav_build_contours_raw: Out of memory 'holes' (%d).", cset.nconts);
				return false;
			}
			memset(holes, 0, sizeof(rcContourHole) * cset.nconts);

			for (int i = 0; i < cset.nconts; ++i) {
				rcContour &cont = cset.conts[i];
				if (winding[i] > 0) {
					if (regions[cont.reg].outline) {
						ctx->log(RC_LOG_ERROR, "nav_build_contours_raw: Multiple outlines for region %d.", cont.reg);
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
					ctx->log(RC_LOG_ERROR, "nav_build_contours_raw: Bad outline for region %d.", i);
				}
			}
		}
	}

	return true;
}

} // namespace zylann::voxel

#endif // VOXEL_ENABLE_NAVIGATION
