#include "build_chunk_contours.h"

//#ifdef VOXEL_ENABLE_NAVIGATION

#include <RecastAlloc.h>
#include <RecastAssert.h>

#include <math.h>
#include <string.h>

//temp for intellisense
#include <Recast.h>




namespace zylann::voxel {

namespace {

// ============================================================================
// getCornerHeight — modified for a broader border vertex check
// ============================================================================
//
// TODO: Double check this.

static int getCornerHeight(int x, int y, int i, int dir,
						   const rcCompactHeightfield& chf,
						   bool& isBorderVertex)
	{
	const rcCompactSpan& s = chf.spans[i];
	int ch = (int)s.y;
	int dirp = (dir+1) & 0x3;

	unsigned int regs[4] = {0,0,0,0};

	// Combine region and area codes in order to prevent
	// border vertices which are in between two areas to be removed.
	regs[0] = chf.spans[i].reg | (chf.areas[i] << 16);

	if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
	{
		const int ax = x + rcGetDirOffsetX(dir);
		const int ay = y + rcGetDirOffsetY(dir);
		const int ai = (int)chf.cells[ax+ay*chf.width].index + rcGetCon(s, dir);
		const rcCompactSpan& as = chf.spans[ai];
		ch = rcMax(ch, (int)as.y);
		regs[1] = chf.spans[ai].reg | (chf.areas[ai] << 16);
		if (rcGetCon(as, dirp) != RC_NOT_CONNECTED)
		{
			const int ax2 = ax + rcGetDirOffsetX(dirp);
			const int ay2 = ay + rcGetDirOffsetY(dirp);
			const int ai2 = (int)chf.cells[ax2+ay2*chf.width].index + rcGetCon(as, dirp);
			const rcCompactSpan& as2 = chf.spans[ai2];
			ch = rcMax(ch, (int)as2.y);
			regs[2] = chf.spans[ai2].reg | (chf.areas[ai2] << 16);
		}
	}
	if (rcGetCon(s, dirp) != RC_NOT_CONNECTED)
	{
		const int ax = x + rcGetDirOffsetX(dirp);
		const int ay = y + rcGetDirOffsetY(dirp);
		const int ai = (int)chf.cells[ax+ay*chf.width].index + rcGetCon(s, dirp);
		const rcCompactSpan& as = chf.spans[ai];
		ch = rcMax(ch, (int)as.y);
		regs[3] = chf.spans[ai].reg | (chf.areas[ai] << 16);
		if (rcGetCon(as, dir) != RC_NOT_CONNECTED)
		{
			const int ax2 = ax + rcGetDirOffsetX(dir);
			const int ay2 = ay + rcGetDirOffsetY(dir);
			const int ai2 = (int)chf.cells[ax2+ay2*chf.width].index + rcGetCon(as, dir);
			const rcCompactSpan& as2 = chf.spans[ai2];
			ch = rcMax(ch, (int)as2.y);
			regs[2] = chf.spans[ai2].reg | (chf.areas[ai2] << 16);
		}
	}

	for (int j = 0; j < 4; ++j)
	{
		const int a = j;
		const int b = (j+1) & 0x3;
		const int c = (j+2) & 0x3;
		const int d = (j+3) & 0x3;
		
		// What we're looking for is a vertex on the border that is not manditory
		// that is, not by an area change, hole, or corner of two boundary regions.
		//
		// If we are a boundary vertex, we should be able to continue a boundary walk.

		// First case: one border span, three interior spans.
		const bool oneExt = (regs[a] & RC_BORDER_REG) != 0;
		const bool threeInts = ((regs[b] | regs[c] | regs[d]) & RC_BORDER_REG) == 0;
		const bool threeIntsSameArea = (regs[b]>>16) == (regs[c]>>16) && (regs[b]>>16) == (regs[d]>>16);
		const bool threeIntsNonZero = regs[b] != 0 && regs[c] != 0 && regs[d] != 0;

		// Second case: two border spans, two interior spans.
		const bool twoSameExts = (regs[a] & regs[b] & RC_BORDER_REG) != 0 && regs[a] == regs[b];
		const bool twoInts = ((regs[c] | regs[d]) & RC_BORDER_REG) == 0;
		const bool twoIntsSameArea = (regs[c]>>16) == (regs[d]>>16);
		const bool twoIntsNonZero = regs[c] != 0 && regs[d] != 0;

		// Third case: three border spans, one interior span.
		const bool threeSameExts = (regs[a] & regs[b] & regs[c] & RC_BORDER_REG) != 0 && regs[a] == regs[b] && regs[a] == regs[c];
		const bool oneInt = (regs[d] & RC_BORDER_REG) == 0;
		const bool oneIntNonZero = regs[d] != 0;

		bool firstCase = oneExt && threeInts && threeIntsSameArea && threeIntsNonZero;
		bool secondCase = twoSameExts && twoInts && twoIntsSameArea && twoIntsNonZero;
		bool thirdCase = threeSameExts && oneInt && oneIntNonZero;

		if (firstCase || secondCase || thirdCase)
		{
			isBorderVertex = true;
			break;
		}
	}

	return ch;
}

// ============================================================================
// Helpers copied from stock RecastContour.cpp
// ============================================================================

static float distancePtSeg(const int x, const int z,
	const int px, const int pz,
	const int qx, const int qz)
{
float pqx = (float)(qx - px);
float pqz = (float)(qz - pz);
float dx = (float)(x - px);
float dz = (float)(z - pz);
float d = pqx*pqx + pqz*pqz;
float t = pqx*dx + pqz*dz;
if (d > 0)
t /= d;
if (t < 0)
t = 0;
else if (t > 1)
t = 1;

dx = px + t*pqx - x;
dz = pz + t*pqz - z;

return dx*dx + dz*dz;
}

static int calcAreaOfPolygon2D(const int* verts, const int nverts)
{
	int area = 0;
	for (int i = 0, j = nverts-1; i < nverts; j=i++)
	{
		const int* vi = &verts[i*4];
		const int* vj = &verts[j*4];
		area += vi[0] * vj[2] - vj[0] * vi[2];
	}
	return (area+1) / 2;
}

static int calcAreaOfPolygon2D(const int* verts, const int nverts)
{
	int area = 0;
	for (int i = 0, j = nverts-1; i < nverts; j=i++)
	{
		const int* vi = &verts[i*4];
		const int* vj = &verts[j*4];
		area += vi[0] * vj[2] - vj[0] * vi[2];
	}
	return (area+1) / 2;
}

// TODO: these are the same as in RecastMesh.cpp, consider using the same.
// Last time I checked the if version got compiled using cmov, which was a lot faster than module (with idiv).
inline int prev(int i, int n) { return i-1 >= 0 ? i-1 : n-1; }
inline int next(int i, int n) { return i+1 < n ? i+1 : 0; }

inline int area2(const int* a, const int* b, const int* c)
{
	return (b[0] - a[0]) * (c[2] - a[2]) - (c[0] - a[0]) * (b[2] - a[2]);
}

//	Exclusive or: true iff exactly one argument is true.
//	The arguments are negated to ensure that they are 0/1
//	values.  Then the bitwise Xor operator may apply.
//	(This idea is due to Michael Baldwin.)
inline bool xorb(bool x, bool y)
{
	return !x ^ !y;
}

// Returns true iff c is strictly to the left of the directed
// line through a to b.
inline bool left(const int* a, const int* b, const int* c)
{
	return area2(a, b, c) < 0;
}

inline bool leftOn(const int* a, const int* b, const int* c)
{
	return area2(a, b, c) <= 0;
}

inline bool collinear(const int* a, const int* b, const int* c)
{
	return area2(a, b, c) == 0;
}

//	Returns true iff ab properly intersects cd: they share
//	a point interior to both segments.  The properness of the
//	intersection is ensured by using strict leftness.
static bool intersectProp(const int* a, const int* b, const int* c, const int* d)
{
	// Eliminate improper cases.
	if (collinear(a,b,c) || collinear(a,b,d) ||
		collinear(c,d,a) || collinear(c,d,b))
		return false;
	
	return xorb(left(a,b,c), left(a,b,d)) && xorb(left(c,d,a), left(c,d,b));
}

// Returns T iff (a,b,c) are collinear and point c lies
// on the closed segement ab.
static bool between(const int* a, const int* b, const int* c)
{
	if (!collinear(a, b, c))
		return false;
	// If ab not vertical, check betweenness on x; else on y.
	if (a[0] != b[0])
		return	((a[0] <= c[0]) && (c[0] <= b[0])) || ((a[0] >= c[0]) && (c[0] >= b[0]));
	else
		return	((a[2] <= c[2]) && (c[2] <= b[2])) || ((a[2] >= c[2]) && (c[2] >= b[2]));
}

// Returns true iff segments ab and cd intersect, properly or improperly.
static bool intersect(const int* a, const int* b, const int* c, const int* d)
{
	if (intersectProp(a, b, c, d))
		return true;
	else if (between(a, b, c) || between(a, b, d) ||
			 between(c, d, a) || between(c, d, b))
		return true;
	else
		return false;
}

static bool vequal(const int* a, const int* b)
{
	return a[0] == b[0] && a[2] == b[2];
}

static bool intersectSegContour(const int* d0, const int* d1, int i, int n, const int* verts)
{
	// For each edge (k,k+1) of P
	for (int k = 0; k < n; k++)
	{
		int k1 = next(k, n);
		// Skip edges incident to i.
		if (i == k || i == k1)
			continue;
		const int* p0 = &verts[k * 4];
		const int* p1 = &verts[k1 * 4];
		if (vequal(d0, p0) || vequal(d1, p0) || vequal(d0, p1) || vequal(d1, p1))
			continue;
		
		if (intersect(d0, d1, p0, p1))
			return true;
	}
	return false;
}

static bool	inCone(int i, int n, const int* verts, const int* pj)
{
	const int* pi = &verts[i * 4];
	const int* pi1 = &verts[next(i, n) * 4];
	const int* pin1 = &verts[prev(i, n) * 4];
	
	// If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
	if (leftOn(pin1, pi, pi1))
		return left(pi, pj, pin1) && left(pj, pi, pi1);
	// Assume (i-1,i,i+1) not collinear.
	// else P[i] is reflex.
	return !(leftOn(pi, pj, pi1) && leftOn(pj, pi, pin1));
}

static void removeDegenerateSegments(rcIntArray& simplified)
{
	// Remove adjacent vertices which are equal on xz-plane,
	// or else the triangulator will get confused.
	int npts = simplified.size()/4;
	for (int i = 0; i < npts; ++i)
	{
		int ni = next(i, npts);
		
		if (vequal(&simplified[i*4], &simplified[ni*4]))
		{
			// Degenerate segment, remove.
			for (int j = i; j < simplified.size()/4-1; ++j)
			{
				simplified[j*4+0] = simplified[(j+1)*4+0];
				simplified[j*4+1] = simplified[(j+1)*4+1];
				simplified[j*4+2] = simplified[(j+1)*4+2];
				simplified[j*4+3] = simplified[(j+1)*4+3];
			}
			simplified.resize(simplified.size()-4);
			npts--;
		}
	}
}

static bool mergeContours(rcContour& ca, rcContour& cb, int ia, int ib)
{
	const int maxVerts = ca.nverts + cb.nverts + 2;
	int* verts = (int*)rcAlloc(sizeof(int)*maxVerts*4, RC_ALLOC_PERM);
	if (!verts)
		return false;
	
	int nv = 0;
	
	// Copy contour A.
	for (int i = 0; i <= ca.nverts; ++i)
	{
		int* dst = &verts[nv*4];
		const int* src = &ca.verts[((ia+i)%ca.nverts)*4];
		dst[0] = src[0];
		dst[1] = src[1];
		dst[2] = src[2];
		dst[3] = src[3];
		nv++;
	}

	// Copy contour B
	for (int i = 0; i <= cb.nverts; ++i)
	{
		int* dst = &verts[nv*4];
		const int* src = &cb.verts[((ib+i)%cb.nverts)*4];
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

struct rcContourHole
{
	rcContour* contour;
	int minx, minz, leftmost;
};

struct rcContourRegion
{
	rcContour* outline;
	rcContourHole* holes;
	int nholes;
};

struct rcPotentialDiagonal
{
	int vert;
	int dist;
};

// Finds the lowest leftmost vertex of a contour.
static void findLeftMostVertex(rcContour* contour, int* minx, int* minz, int* leftmost)
{
	*minx = contour->verts[0];
	*minz = contour->verts[2];
	*leftmost = 0;
	for (int i = 1; i < contour->nverts; i++)
	{
		const int x = contour->verts[i*4+0];
		const int z = contour->verts[i*4+2];
		if (x < *minx || (x == *minx && z < *minz))
		{
			*minx = x;
			*minz = z;
			*leftmost = i;
		}
	}
}

static int compareHoles(const void* va, const void* vb)
{
	const rcContourHole* a = (const rcContourHole*)va;
	const rcContourHole* b = (const rcContourHole*)vb;
	if (a->minx == b->minx)
	{
		if (a->minz < b->minz)
			return -1;
		if (a->minz > b->minz)
			return 1;
	}
	else
	{
		if (a->minx < b->minx)
			return -1;
		if (a->minx > b->minx)
			return 1;
	}
	return 0;
}


static int compareDiagDist(const void* va, const void* vb)
{
	const rcPotentialDiagonal* a = (const rcPotentialDiagonal*)va;
	const rcPotentialDiagonal* b = (const rcPotentialDiagonal*)vb;
	if (a->dist < b->dist)
		return -1;
	if (a->dist > b->dist)
		return 1;
	return 0;
}


static void mergeRegionHoles(rcContext* ctx, rcContourRegion& region)
{
	// Sort holes from left to right.
	for (int i = 0; i < region.nholes; i++)
		findLeftMostVertex(region.holes[i].contour, &region.holes[i].minx, &region.holes[i].minz, &region.holes[i].leftmost);
	
	qsort(region.holes, region.nholes, sizeof(rcContourHole), compareHoles);
	
	int maxVerts = region.outline->nverts;
	for (int i = 0; i < region.nholes; i++)
		maxVerts += region.holes[i].contour->nverts;
	
	rcScopedDelete<rcPotentialDiagonal> diags((rcPotentialDiagonal*)rcAlloc(sizeof(rcPotentialDiagonal)*maxVerts, RC_ALLOC_TEMP));
	if (!diags)
	{
		ctx->log(RC_LOG_WARNING, "mergeRegionHoles: Failed to allocated diags %d.", maxVerts);
		return;
	}
	
	rcContour* outline = region.outline;
	
	// Merge holes into the outline one by one.
	for (int i = 0; i < region.nholes; i++)
	{
		rcContour* hole = region.holes[i].contour;
		
		int index = -1;
		int bestVertex = region.holes[i].leftmost;
		for (int iter = 0; iter < hole->nverts; iter++)
		{
			// Find potential diagonals.
			// The 'best' vertex must be in the cone described by 3 cosequtive vertices of the outline.
			// ..o j-1
			//   |
			//   |   * best
			//   |
			// j o-----o j+1
			//         :
			int ndiags = 0;
			const int* corner = &hole->verts[bestVertex*4];
			for (int j = 0; j < outline->nverts; j++)
			{
				if (inCone(j, outline->nverts, outline->verts, corner))
				{
					int dx = outline->verts[j*4+0] - corner[0];
					int dz = outline->verts[j*4+2] - corner[2];
					diags[ndiags].vert = j;
					diags[ndiags].dist = dx*dx + dz*dz;
					ndiags++;
				}
			}
			// Sort potential diagonals by distance, we want to make the connection as short as possible.
			qsort(diags, ndiags, sizeof(rcPotentialDiagonal), compareDiagDist);
			
			// Find a diagonal that is not intersecting the outline not the remaining holes.
			index = -1;
			for (int j = 0; j < ndiags; j++)
			{
				const int* pt = &outline->verts[diags[j].vert*4];
				bool intersect = intersectSegContour(pt, corner, diags[i].vert, outline->nverts, outline->verts);
				for (int k = i; k < region.nholes && !intersect; k++)
					intersect |= intersectSegContour(pt, corner, -1, region.holes[k].contour->nverts, region.holes[k].contour->verts);
				if (!intersect)
				{
					index = diags[j].vert;
					break;
				}
			}
			// If found non-intersecting diagonal, stop looking.
			if (index != -1)
				break;
			// All the potential diagonals for the current vertex were intersecting, try next vertex.
			bestVertex = (bestVertex + 1) % hole->nverts;
		}
		
		if (index == -1)
		{
			ctx->log(RC_LOG_WARNING, "mergeHoles: Failed to find merge points for %p and %p.", region.outline, hole);
			continue;
		}
		if (!mergeContours(*region.outline, *hole, index, bestVertex))
		{
			ctx->log(RC_LOG_WARNING, "mergeHoles: Failed to merge contours %p and %p.", region.outline, hole);
			continue;
		}
	}
}

// ============================================================================
// Our helpers
// ============================================================================

enum endpointType {
	INITIAL_VERTEX,
	MANDATORY_VERTEX,
	BORDER_T_VERTEX
};

struct contourSegmentInfo
{
	// To check for loop closure, we need to stash the initial span, and direction of this
	// segment and of the contour walk.
	int contourStarti;
	unsigned char contourStartDir;
	bool contourStartRevered = false;

	endpointType segmentEndType = INITIAL_VERTEX;

	// Offsets of this segment in the vertex and simplified arrays.
	int length = 0;

	// walk state:
	int i, x, z, dir;
	int prev_r;
	bool reversed = false;
	bool clearFlags = true;

	contourSegmentInfo( int _starti, unsigned char _startDir, int _i, int _x, int _z, int _prev_r)
	: contourStarti(_starti), contourStartDir(_startDir), dir(_startDir), i(_i), x(_x), z(_z), prev_r(_prev_r) {}
};

struct endPoint
{
	int edgeLength = 0;
	endpointType type = INITIAL_VERTEX;

	endPoint(int _edgeLength, endpointType _type)
	: edgeLength(_edgeLength), type(_type) {}
};


// ============================================================================
// Modified functions
// ============================================================================

// Our contour walking function differs from recast in that we walk and simplify
// one segment of each contour at a time, instead of simplifying after the walk.
//
// A segment ends at a mandatory vertex, which is a vertex adjacent to a chunk corner,
// a new hole or region, or an area border, or else ends before it's initial vertex.
//
// If two regions of a chunk meet at the boundary, at what we call a T boundary vertex,
// our algorithm will not emit the segment along the boundary, 'clipping' these vertices
// off the contour. The clipped part of the interior regions will get added to a new
// 'boundary' contour, which stitches together these clipped segments into a single
//  contour, with a cross-chunk stable segment over the boundary.
//
// This seems more or less like what the original algorithm wanted to do.
//
// rambling TODO: 
// We need to gaurentee that all edges of the boundary contours appear in the triangulation.
//
// If we do maxlength splitting on all edges of the boundary contours instead
// of reducing error, there should be about the same number of edges on the boundary and
// interior side. BUT this would break vertical chunk seams, which need to minimize errors
// or else they might make invalid geometry. I need to look at the triangulation code and see
// if any more changes need to be made there.

bool build_chunk_contours(rcContext *ctx, const rcCompactHeightfield &chf,
						  float maxError, int maxEdgeLen, rcContourSet &cset,
						  int yLowerPad = 0, int yUpperPad = 0)
	{
	rcAssert(ctx);
	
	const int w = chf.width;
	const int h = chf.height;
	const int borderSize = chf.borderSize;
	
	rcScopedTimer timer(ctx, RC_TIMER_BUILD_CONTOURS);
	
	rcVcopy(cset.bmin, chf.bmin);
	rcVcopy(cset.bmax, chf.bmax);
	if (borderSize > 0)
	{
		// If the heightfield was build with bordersize, remove the offset.
		const float pad = borderSize*chf.cs;
		cset.bmin[0] += pad;
		cset.bmin[1] += yLowerPad * chf.ch;
		cset.bmin[2] += pad;
		cset.bmax[0] -= pad;
		cset.bmax[1] -= yUpperPad * chf.ch;
		cset.bmax[2] -= pad;
	}
	cset.cs = chf.cs;
	cset.ch = chf.ch;
	cset.width = chf.width - chf.borderSize*2;
	cset.height = chf.height - chf.borderSize*2;
	cset.borderSize = chf.borderSize;
	cset.maxError = maxError;
	
	int maxContours = rcMax((int)chf.maxRegions, 8);
	cset.conts = (rcContour*)rcAlloc(sizeof(rcContour)*maxContours, RC_ALLOC_PERM);
	if (!cset.conts)
		return false;
	cset.nconts = 0;
	
	rcScopedDelete<unsigned char> flags((unsigned char*)rcAlloc(sizeof(unsigned char)*chf.spanCount, RC_ALLOC_TEMP));
	if (!flags)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'flags' (%d).", chf.spanCount);
		return false;
	}
	
	
	// Mark disconnected boundaries.
	for (int z = 0; z < h; ++z)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+z*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				unsigned char res = 0;
				const rcCompactSpan& s = chf.spans[i];

				if (!chf.spans[i].reg )
				{
					flags[i] = 0;
					continue;
				}

				//check if boundary region
				bool isInBoundary = (chf.spans[i].reg & RC_BORDER_REG) != 0;

				for (int dir = 0; dir < 4; ++dir)
				{
					unsigned short r = 0;
					if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirOffsetX(dir);
						const int az = z + rcGetDirOffsetY(dir);
						const int ai = (int)chf.cells[ax+az*w].index + rcGetCon(s, dir);
						r = chf.spans[ai].reg;
					}

					// Mark directions that look towards boundary of interior region,
					// or towards the interior from a boundary region
					if (r == chf.spans[i].reg || ( isInBoundary && ((r & RC_BORDER_REG) != 0) ) )
						res |= (1 << dir);
				}

				res = res ^ 0xf; // The above checks are inverted from what we want.
				res |= res << 4; // Add extra flags for boundary walks
					  	 	 	 // eg reverse walks in boundary, interior walks for pushoff.
			}
		}
	}
	
	// Try to start walking at the next cell
	rcIntArray verts(256);
	rcIntArray simplified(64);
	rcTempVector<endPoint> segmentEnds;
	segmentEnds.reserve(16); //TODO: tune
	
	for (int z = 0; z < h; ++z)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+z*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				//skip cells that aren't marked. // TODO: I only care about cw walk flags
				if (flags[i] == 0 || flags[i] == 0xf)
				{
					flags[i] = 0;
					continue;
				}

				const unsigned short reg = chf.spans[i].reg;
				if (!reg)
					continue; // skips zero region cells (shouldn't happen)

				// at this point, we know we are in a valid cell to start a walk!
				const unsigned char area = chf.areas[i];
				
				// Clear arrays from last contour walk.
				verts.clear();
				simplified.clear();
				segmentEnds.clear();

				//commit initial info for the first vertex:

				// set initial direction
				unsigned char dir = 0;
				while ((flags[i] & (1 << dir)) == 0)
					dir++;

				// Clear flag for this direction
				flags[i] &= ~(1 << dir);
				
				// push and classify initial vertex.
				bool isBorderVertex = false;
				bool isAreaBorder = false;
				int px = x;
				int py = getCornerHeight(x, z, i, dir, chf, isBorderVertex);
				int pz = z;
				switch(dir)
				{
					case 0: pz++; break;
					case 1: px++; pz++; break;
					case 2: px++; break;
				}
				int r = 0;
				const rcCompactSpan& s = chf.spans[i];
				if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(dir);
					const int az = z + rcGetDirOffsetY(dir);
					const int ai = (int)chf.cells[ax+az*chf.width].index + rcGetCon(s, dir);
					r = (int)chf.spans[ai].reg;
					if (area != chf.areas[ai])
						isAreaBorder = true;
				}
				if (isBorderVertex)
					r |= RC_BORDER_VERTEX;
				if (isAreaBorder)
					r |= RC_AREA_BORDER;

				verts.push(px);
				verts.push(py);
				verts.push(pz);
				verts.push(r);

				// The first vertex is necessarily an end.
				segmentEnds.push_back( endPoint(0, INITIAL_VERTEX) );

				// Initialize segment info, state is given by the last pushed vertex.
				contourSegmentInfo segmentInfo = contourSegmentInfo(i, dir, i, x, z, r);

				// We build one segment at a time starting with the first interior vertex, so we always start with
				// the first vertex in the first edge of the segment we are building, and walk clockwise from there.
				int walkIter = 0; //safety limit
				while (walkIter < 40000)
				{
					walkIter++;
					
					segmentInfo = walkSegment(segmentInfo, chf, flags, verts, walkIter);
					
					//collect segment info for the segment simplification process:
					endpointType segmentEndType = segmentInfo.segmentEndType; // The classification of the last endpoint.
					int segmentEdgeLength = segmentInfo.length;
					
					switch(segmentEndType)
					{
						case INITIAL_VERTEX:
							// The contour is complete!
							segmentEnds.push_back( endPoint(segmentEdgeLength, INITIAL_VERTEX) );
							break;
						case MANDATORY_VERTEX:
							segmentEnds.push_back( endPoint(segmentEdgeLength, segmentEndType) );
							continue;
						case BORDER_T_VERTEX:
							// We have a new endpoint to track:
							segmentEnds.push_back( endPoint(segmentEdgeLength, segmentEndType) ); //TODO: double check this when contour simplification is complete.
							if( (r & RC_BORDER_VERTEX) != 0) // Boundary walk
							{
								// if we are walking the boundary, we need to inject an interior segment at the T of our boundary walk.
								// segmentInfo is tracking the state of the boundary walk for us, so we need to initialize the interior
								// walk info. This walk is ccw, and uses seperate flag bits from interior walks so they don't interfere.

								int T_i = segmentInfo.i;
								int T_x = segmentInfo.x;
								int T_z = segmentInfo.z;
								unsigned char T_dir = segmentInfo.dir;
								int T_region = chf.spans[T_i].reg;

								// step to next span (Should be gaurenteed to exist, but we should double check)
								const int interior_x = T_x + rcGetDirOffsetX(T_dir); 
								const int interior_z = T_z + rcGetDirOffsetY(T_dir);

								const rcCompactCell& T_s = chf.cells[interior_x+interior_z*chf.width]; // grab offset of spans in the adjacent cell.
								int interior_i = (int)T_s.index + rcGetCon(chf.spans[T_i], T_dir); // rcGetCon gives index amoung spans with footprint in the next cell. FIX ME
								
								unsigned char interior_dir = (T_dir + 2) & 0x3; // now dacing the T edge from interior.

								int interior_r = T_region;
								interior_r |= RC_BORDER_VERTEX; // Shouldn't be an area border since then it would be manditory.

								// The state we set below would be just after pushing the T vertex from the interior.
								contourSegmentInfo interiorSegmentInfo = contourSegmentInfo(interior_i, interior_dir, interior_i, interior_x, interior_z, interior_r);
								interiorSegmentInfo.clearFlags = false;
								interiorSegmentInfo.reversed = true;
								interiorSegmentInfo.contourStartRevered = true;
								interiorSegmentInfo.segmentEndType = BORDER_T_VERTEX;

								interiorSegmentInfo = walkSegment(interiorSegmentInfo, chf, flags, verts, walkIter);
								segmentEnds.push_back( endPoint(interiorSegmentInfo.length, interiorSegmentInfo.segmentEndType) );

								// push the T vertex again 
								verts.push(T_x);
								verts.push(getCornerHeight(T_x, T_z, T_i, T_dir, chf, isBorderVertex));
								verts.push(T_z);
								verts.push(segmentInfo.prev_r);
								segmentEnds.push_back( endPoint(0, BORDER_T_VERTEX) ); // dummy segment for contour simplification

							}
							continue;
					}
				}

				// Now we have all vertices of the contour, and we start simplifying per segment.
				if(verts.size() > 2)
				{
					int simplified_offset = 0;
					int simplified_segment_length = 0;

					if(segmentEnds.size() != 1)
					{
						// If we have more than one segment, then we have at least two manditory vertices, and can simplify the segments between them.

						// Save the initial segment for the end when it wraps.
						int initial_segment_length = segmentEnds[0].edgeLength;
						int segment_offset = initial_segment_length;
						
						//iterate through all segments except the warpped segment.
						for(int i = 1; i < segmentEnds.size() - 1; i++)
						{
							int segment_length = segmentEnds[i].edgeLength;
							int segment_start = segment_offset;
							int segment_end = segment_start + segment_length;

							simplified.push(verts[ segment_start*4 + 0]); // x
							simplified.push(verts[ segment_start*4 + 1]); // y
							simplified.push(verts[ segment_start*4 + 2]); // z
							simplified.push( segment_start ); // index

							simplified.push(verts[ segment_end*4 + 0]); // x
							simplified.push(verts[ segment_end*4 + 1]); // y
							simplified.push(verts[ segment_end*4 + 2]); // z
							simplified.push( segment_end ); // index

							simplified_segment_length += 2;

							for(int j = 0; j < simplified_segment_length; j++)
							{
								int jj = (j+1) % simplified_segment_length;

								int ax = simplified[ (j + segment_offset)*4 + 0];
								int az = simplified[ (j + segment_offset)*4 + 2];
								int ai = simplified[ (j + segment_offset)*4 + 3];

								int bx = simplified[ (jj + segment_offset)*4 + 0];
								int bz = simplified[ (jj + segment_offset)*4 + 2];
								int bi = simplified[ (jj + segment_offset)*4 + 3];
								
								// Find maximum deviation from the current edge of this segment.
								float maxd = 0;
								int maxi = -1;
								int ci, cinc, endi;

								// Traverse the segment in lexilogical order so that the
								// max deviation is calculated similarly when traversing
								// opposite segments.
								if (bx > ax || (bx == ax && bz > az))
								{
									cinc = 1;
									ci = (ai+cinc); // % pn; // We don't need to account for wrapping in this case.
									endi = bi;
								}
								else
								{
									cinc = -1; // pn-1; // not cyclic class of -1.
									ci = (bi+cinc); // % pn;
									endi = ai;
									rcSwap(ax, bx);
									rcSwap(az, bz);
								}

								// Should be tessalating only the holes and area boundaries,
								//  so we shouldn't tessalate the borders.
								if ( ( (verts[ci*4+3] & RC_CONTOUR_REG_MASK) == 0 ||
									(verts[ci*4+3] & RC_AREA_BORDER)) )
								{
									while (ci != endi)
									{
										float d = distancePtSeg(verts[ci*4+0], verts[ci*4+2], ax, az, bx, bz);
										if (d > maxd)
										{
											maxd = d;
											maxi = ci;
										}
										ci = (ci+cinc); // % pn;
									}
								}


							}
							
						}
					}
					else
					{
						// There are no connections to other contours, so we simplify the entire contour from a fixed basepoint.
						// TODO: copy hasConnections = false case.
					}
				}

				removeDegenerateSegments(simplified);
				/*
				
				// Store region->contour remap info.
				// Create contour.
				if (simplified.size()/4 >= 3)
				{
					if (cset.nconts >= maxContours)
					{
						// Allocate more contours.
						// This happens when a region has holes.
						const int oldMax = maxContours;
						maxContours *= 2;
						rcContour* newConts = (rcContour*)rcAlloc(sizeof(rcContour)*maxContours, RC_ALLOC_PERM);
						for (int j = 0; j < cset.nconts; ++j)
						{
							newConts[j] = cset.conts[j];
							// Reset source pointers to prevent data deletion.
							cset.conts[j].verts = 0;
							cset.conts[j].rverts = 0;
						}
						rcFree(cset.conts);
						cset.conts = newConts;
						
					}
					
					rcContour* cont = &cset.conts[cset.nconts++];
					
					cont->nverts = simplified.size()/4;
					cont->verts = (int*)rcAlloc(sizeof(int)*cont->nverts*4, RC_ALLOC_PERM);
					if (!cont->verts)
					{
						return false;
					}
					memcpy(cont->verts, &simplified[0], sizeof(int)*cont->nverts*4);
					if (borderSize > 0)
					{
						// If the heightfield was build with bordersize, remove the offset.
						for (int j = 0; j < cont->nverts; ++j)
						{
							int* v = &cont->verts[j*4];
							v[0] -= borderSize;
							v[2] -= borderSize;
						}
					}
					
					cont->nrverts = verts.size()/4;
					cont->rverts = (int*)rcAlloc(sizeof(int)*cont->nrverts*4, RC_ALLOC_PERM);
					if (!cont->rverts)
					{
						return false;
					}
					memcpy(cont->rverts, &verts[0], sizeof(int)*cont->nrverts*4);
					if (borderSize > 0)
					{
						// If the heightfield was build with bordersize, remove the offset.
						for (int j = 0; j < cont->nrverts; ++j)
						{
							int* v = &cont->rverts[j*4];
							v[0] -= borderSize;
							v[2] -= borderSize;
						}
					}
					
					cont->reg = reg;
					cont->area = area;
				
				}
				// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
			}
		}
	}

	/*
	// Merge holes if needed.
	if (cset.nconts > 0)
	{
		// Calculate winding of all polygons.
		rcScopedDelete<signed char> winding((signed char*)rcAlloc(sizeof(signed char)*cset.nconts, RC_ALLOC_TEMP));
		if (!winding)
		{
			ctx->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'hole' (%d).", cset.nconts);
			return false;
		}
		int nholes = 0;
		for (int i = 0; i < cset.nconts; ++i)
		{
			rcContour& cont = cset.conts[i];
			// If the contour is wound backwards, it is a hole.
			winding[i] = calcAreaOfPolygon2D(cont.verts, cont.nverts) < 0 ? -1 : 1;
			if (winding[i] < 0)
				nholes++;
		}
		
		if (nholes > 0)
		{
			// Collect outline contour and holes contours per region.
			// We assume that there is one outline and multiple holes.
			const int nregions = chf.maxRegions+1;
			rcScopedDelete<rcContourRegion> regions((rcContourRegion*)rcAlloc(sizeof(rcContourRegion)*nregions, RC_ALLOC_TEMP));
			if (!regions)
			{
				ctx->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'regions' (%d).", nregions);
				return false;
			}
			memset(regions, 0, sizeof(rcContourRegion)*nregions);
			
			rcScopedDelete<rcContourHole> holes((rcContourHole*)rcAlloc(sizeof(rcContourHole)*cset.nconts, RC_ALLOC_TEMP));
			if (!holes)
			{
				ctx->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'holes' (%d).", cset.nconts);
				return false;
			}
			memset(holes, 0, sizeof(rcContourHole)*cset.nconts);
			
			for (int i = 0; i < cset.nconts; ++i)
			{
				rcContour& cont = cset.conts[i];
				// Positively would contours are outlines, negative holes.
				if (winding[i] > 0)
				{
					if (regions[cont.reg].outline)
						ctx->log(RC_LOG_ERROR, "rcBuildContours: Multiple outlines for region %d.", cont.reg);
					regions[cont.reg].outline = &cont;
				}
				else
				{
					regions[cont.reg].nholes++;
				}
			}
			int index = 0;
			for (int i = 0; i < nregions; i++)
			{
				if (regions[i].nholes > 0)
				{
					regions[i].holes = &holes[index];
					index += regions[i].nholes;
					regions[i].nholes = 0;
				}
			}
			for (int i = 0; i < cset.nconts; ++i)
			{
				rcContour& cont = cset.conts[i];
				rcContourRegion& reg = regions[cont.reg];
				if (winding[i] < 0)
					reg.holes[reg.nholes++].contour = &cont;
			}
			
			// Finally merge each regions holes into the outline.
			for (int i = 0; i < nregions; i++)
			{
				rcContourRegion& reg = regions[i];
				if (!reg.nholes) continue;
				
				if (reg.outline)
				{
					mergeRegionHoles(ctx, reg);
				}
				else
				{
					// The region does not have an outline.
					// This can happen if the contour becaomes selfoverlapping because of
					// too aggressive simplification settings.
					ctx->log(RC_LOG_ERROR, "rcBuildContours: Bad outline for region %d, contour simplification is likely too aggressive.", i);
				}
			}
		}
		
	} */
	
	return true;
}















contourSegmentInfo walkSegment(contourSegmentInfo &segmentInfo, const rcCompactHeightfield &chf, unsigned char *flags, rcIntArray &verts, int& walkIter)
{
	// TODO: how do we know if we are walking from the boundary to the interior from a T vertex?
	unsigned char dir = segmentInfo.dir;
	int i = segmentInfo.i;
	int x = segmentInfo.x;
	int z = segmentInfo.z;
	int prev_r = segmentInfo.prev_r;
	int pushCount = 0;

	unsigned char segmentStartDir = dir;
	int segmentStarti = i;

	contourSegmentInfo segmentEndInfo = segmentInfo;


	while (walkIter < 40000)
	{
		// find the next vertex
		if ( !segmentInfo.reversed )
		{
			dir = (dir + 1) & 0x3; //rotate cw.
		}
		else
		{
			dir = (dir + 3) & 0x3; //rotate ccw.
		}

		// Back to the start of this segment (implicitely has same orientation).
		bool backToStartOfSeg = (i == segmentStarti && dir == segmentStartDir); 
		// Back to the start of the contour in the same orientation
		bool backToStartOfContour = (i == segmentInfo.contourStarti &&
									 dir == segmentInfo.contourStartDir &&
									 segmentInfo.contourStartRevered == segmentInfo.reversed);

		// Check for loop closure
		if( backToStartOfSeg || backToStartOfContour ) // Note the first condition should imply the second.
		{
			segmentEndInfo.segmentEndType = INITIAL_VERTEX;
			break; // No need to push a vertex
		}

		if ( (( (flags[i] & (1 << dir) ) != 0 ) && !segmentInfo.reversed ) ||
			 (( (flags[i] & (1 << (dir + 4) ) ) != 0 ) && segmentInfo.reversed ) ) // TODO: ugly
		{
			// We found the next vertex!
			if( segmentInfo.clearFlags ) // We don't clear flags when walking into the interior from the boundary.
			{
				if( !segmentInfo.reversed )
				{
					flags[i] &= ~(1 << dir); // Remove visited edge
				}
				else
				{
					flags[i] &= ~(1 << (dir + 4)); // Remove visited edge in reverse direction
				}
			}

			// Classify vertex.
			bool isBorderVertex = false;
			bool isAreaBorder = false;
			int px = x;
			int py = getCornerHeight(x, z, i, dir, chf, isBorderVertex);
			int pz = z;
			// Vertex position is the closest point in the edge of dir to the direction we are traveling.
			if(!segmentInfo.reversed)
			{
				switch(dir)
				{
					case 0: pz++; break;
					case 1: px++; pz++; break;
					case 2: px++; break;
				}
			}
			else
			{
				switch(dir)
				{
					case 1: pz++; break;
					case 2: px++; pz++; break;
					case 3: px++; break;
				}
			}
			int r = 0;
			const rcCompactSpan& s = chf.spans[i];
			if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
			{
				const int ax = x + rcGetDirOffsetX(dir);
				const int az = z + rcGetDirOffsetY(dir); // recast renames z as y for walks. I hate it.
				const int ai = (int)chf.cells[ax+az*chf.width].index + rcGetCon(s, dir);
				r = (int)chf.spans[ai].reg;
				if (chf.areas[i] != chf.areas[ai])
					isAreaBorder = true;
			}
			if (isBorderVertex)
				r |= RC_BORDER_VERTEX;
			if (isAreaBorder)
				r |= RC_AREA_BORDER;

			// We now have all the info we need about the vertex, use this info to check for an endpoint.

			// Check for segment end conditions: changes in region or area:
			bool differentRegs = (prev_r & RC_CONTOUR_REG_MASK) != (r & RC_CONTOUR_REG_MASK);
			bool areaBorderChange = (prev_r & RC_AREA_BORDER) != (r & RC_AREA_BORDER); 
			// TODO: the above might be bugged: are we using RC_AREA_BORDER a flag or bitmask? This is expecting a bitmask. If wrong, I think isAreaBorder would work?
			if( differentRegs && !areaBorderChange &&  (r & RC_BORDER_VERTEX) != 0 ) // TODO: these if statements are confusing. 
			{
				segmentEndInfo.segmentEndType = BORDER_T_VERTEX; break;
			}
			else if(differentRegs || areaBorderChange)
			{
				segmentEndInfo.segmentEndType = MANDATORY_VERTEX; break;
			}

			// If the last vertex was not an endpoint, this vertex belongs to the segment and so we can push it: 
			verts.push(px);
			verts.push(py);
			verts.push(pz);
			verts.push(r);
			pushCount++;

			// Update prev_r
			prev_r = r;
			
			// Point to the new furthest vertex along the segment.
			segmentEndInfo.dir = dir;
			segmentEndInfo.i = i;
			segmentEndInfo.x = x;
			segmentEndInfo.z = z;
			segmentEndInfo.prev_r = r;
			segmentEndInfo.length = pushCount; // Effectively the number of edges walked so far in this segment.

		} else {
			// This span isn't flagged for a walk, we need to find the next span that is, or end the segment.

			// step to next span if possible
			int next_i = -1;
			const int next_x = x + rcGetDirOffsetX(dir);
			const int next_z = z + rcGetDirOffsetY(dir);

			// check if we can walk to the next span in this direction
			const rcCompactSpan& s = chf.spans[i];
			bool inBoundary = (s.reg & RC_BORDER_REG) != 0;
			bool wasBorderVertex = (prev_r & RC_BORDER_VERTEX) != 0;
			if ( (rcGetCon(s, dir) != RC_NOT_CONNECTED) || (inBoundary && wasBorderVertex) )
			{	
				// note: if wasBorderVertex is set, we must be connected. However, if we
				// follow this branch for two iterations in the boundary, prev_r no longer
				// gaurentees we are connected.
				// We'll generically follow once so I'll leave this issue open for now.
				// TODO: refresh prev_r from the perspective of the next span.

				const rcCompactCell& next_cell = chf.cells[next_x+next_z*chf.width]; // grab offset of spans in the adjacent cell.
				next_i = (int)next_cell.index + rcGetCon(s, dir); // rcGetCon gives index amoung spans with footprint in the next cell.

				i = next_i;
				dir = (dir + 2) & 0x3; // will effectively rotate the reverse direction on next iteration.
				x = next_x;
				z = next_z;

			}
			else
			{
				// no connected span in this direction, we are at a dead end. This shouldn't happen in the interior,
				// but does happen when walking a boundary and hitting a hole in the boundary. This is a mandatory
				// at the edge of a boundary walk, so we turn around to walk back along the boundary.
				segmentEndInfo.reversed = !segmentInfo.reversed;
				segmentEndInfo.segmentEndType = MANDATORY_VERTEX;
				break;
			}
		}

		walkIter++;
	}

	return segmentEndInfo;
}

} // namespace zylann::voxel

//#endif // VOXEL_ENABLE_NAVIGATION
