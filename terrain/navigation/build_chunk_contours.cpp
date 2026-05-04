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
	int segStarti;
	unsigned char segStartDir;
	int contourStarti;
	unsigned char contourStartDir;
	bool contourStartRevered = false;

	endpointType segmentStartType = INITIAL_VERTEX;

	// Offsets of this segment in the vertex and simplified arrays.
	int vertexOffset = 0;

	// walk state:
	int x, z, i;
	int prev_r;
	bool reversed = false;

	contourSegmentInfo( int _starti, unsigned char _startDir, int _x, int _z, int _i, int _prev_r)
	: segStarti(_starti), segStartDir(_startDir), contourStarti(_starti), contourStartDir(_startDir), x(_x), z(_z), i(_i), prev_r(_prev_r) {}
};

struct endPoints
{
	int vertexIndex = 0;
	endpointType type = INITIAL_VERTEX;
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
				flags[i] = res ^ 0xf; // The above checks are inverted from what we want.
			}
		}
	}
	
	// Try to start walking at the next cell
	rcIntArray verts(256);
	rcIntArray simplified(64);
	
	for (int z = 0; z < h; ++z)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+z*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				//skip cells that aren't marked.
				if (flags[i] == 0 || flags[i] == 0xf)
				{
					flags[i] = 0;
					continue;
				}

				const unsigned short reg = chf.spans[i].reg;
				if (!reg)
					continue; // skips zero region cells (shouldn't happen)
				const unsigned char area = chf.areas[i];
				
				// Clear arrays from last contour walk.
				verts.clear();
				simplified.clear();

				int walkIter = 0; //safety limit
				while (walkIter < 40000)
				{
					walkIter++;

					// We build one segment at a time starting with the first interior vertex.
					// We push the vertex of the next segment to check for manditory vertices.

					// set initial direction
					unsigned char dir = 0;
					while ((flags[i] & (1 << dir)) == 0)
						dir++;
					
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

					// Clear flag for this direction if we aren't in the border
					if( r & RC_BORDER_VERTEX == 0 )
					{
						flags[i] &= ~(1 << dir);
					}

					contourSegmentInfo segmentInfo = contourSegmentInfo(i, dir, x, z, i, r);
					
					//todo: we need to know when to stop
					segmentInfo = walkSegment(segmentInfo, chf, flags, verts, walkIter);
					
					// simplify segment

				}
				
				/* ~~~~~~~~~~~~ what we are replacing ~~~~~~~~~~~~ //
				walkContour(x, y, i, chf, flags, verts);

				simplifyContour(verts, simplified, maxError, maxEdgeLen, buildFlags);
				removeDegenerateSegments(simplified);
				
				
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
		
	}
	
	return true;
}















contourSegmentInfo walkSegment(contourSegmentInfo &segmentInfo, const rcCompactHeightfield &chf, unsigned char *flags, rcIntArray &verts, int& walkIter)
{
	unsigned char dir = segmentInfo.segStartDir;
	int i = segmentInfo.i;
	int x = segmentInfo.x;
	int z = segmentInfo.z;
	int prev_r = segmentInfo.prev_r;
	int pushCount = 0;

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

		// back to the start of this segment
		bool backToStartOfSeg = (i == segmentInfo.segStarti && dir == segmentInfo.segStartDir); 
		// back to the start of the contour in the same orientation
		bool backToStartOfContour = (i == segmentInfo.contourStarti &&
									 dir == segmentInfo.contourStartDir &&
									 segmentInfo.contourStartRevered == segmentInfo.reversed);

		// check for loop closure
		if( backToStartOfSeg || backToStartOfContour )
		{
			segmentEndInfo.segmentStartType = INITIAL_VERTEX;
			break; // No need to push a vertex
		}

		if (flags[i] & (1 << dir))
		{
			// We found the next vertex!
			flags[i] &= ~(1 << dir); // Remove visited edge

			// push and classify vertex.
			bool isBorderVertex = false;
			bool isAreaBorder = false;
			int px = x;
			int py = getCornerHeight(x, z, i, dir, chf, isBorderVertex);
			int pz = z;
			// vertex position is the closest point in the edge of dir to the direction we are traveling.
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

			verts.push(px);
			verts.push(py);
			verts.push(pz);
			verts.push(r);
			pushCount++;

			// check for segment end conditions: changes in region or area:
			bool differentRegs = (prev_r & RC_CONTOUR_REG_MASK) != (r & RC_CONTOUR_REG_MASK);
			bool areaBorderChange = (prev_r & RC_AREA_BORDER) != (r & RC_AREA_BORDER);
			if( differentRegs || areaBorderChange )
			{
				// we have observed a region or area change.
				if( (r & RC_BORDER_VERTEX) != 0 )
				{
					segmentEndInfo.segmentStartType = BORDER_T_VERTEX;
				}
				else
				{
					segmentEndInfo.segmentStartType = MANDATORY_VERTEX;
				}

				segmentEndInfo.prev_r = r;
				segmentEndInfo.segStarti = i;
				segmentEndInfo.segStartDir = dir;
				segmentEndInfo.vertexOffset = pushCount;
				break;
			}

			// update prev_r
			prev_r = r;
		} else {
			//check if we are at the end of a boundary walk.


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

				// Push first vertex in other direction
				if ( !segmentEndInfo.reversed )  // we should be looking in the previous direction again.
				{
					dir = (dir + 1) & 0x3; //rotate cw.
				}
				else
				{
					dir = (dir + 3) & 0x3; //rotate ccw.
				}

				bool isBorderVertex = false;
				bool isAreaBorder = false;
				int px = x;
				int py = getCornerHeight(x, z, i, dir, chf, isBorderVertex);
				int pz = z;
				// vertex position is the closest point in the edge of dir to the direction we are traveling.
				if(  !segmentEndInfo.reversed )
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

				verts.push(px);
				verts.push(py);
				verts.push(pz);
				verts.push(r);

				segmentEndInfo.prev_r = r;
				segmentEndInfo.segmentStartType = MANDATORY_VERTEX;
				segmentEndInfo.segStarti = i;
				segmentEndInfo.segStartDir = dir;
				break;
			}
		}

		walkIter++;
	}

	return segmentEndInfo;
}



















// Old garbage opus implementation




























// Walks one segment of a contour starting from `state`, appending raw vertices
// to `points`. Updates `state` in-place. Returns the reason for stopping.
//
// Loop closure is detected against (loop_start_i, loop_start_dir) — the parent
// walk's start, not necessarily this segment's start. The orchestrator passes
// these through unchanged across re-entries so closure detection is stable.
static WalkStopReason walkSegmentOld(
		WalkState &state,
		int loop_start_i, unsigned char loop_start_dir,
		const rcCompactHeightfield &chf,
		unsigned char *flags,
		rcIntArray &points,
		const WalkSegmentConfig &cfg) {
	while (++state.iter < 40000) {
		if (flags[state.i] & (1 << state.dir)) {
			// --- Boundary edge: emit vertex ---
			bool isBorderVertex = false;
			bool isAreaBorder = false;
			int px = state.x;
			int py = getCornerHeight(state.x, state.y, state.i, state.dir, chf, isBorderVertex);
			int pz = state.y;
			switch (state.dir) {
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
			const rcCompactSpan &s = chf.spans[state.i];
			if (rcGetCon(s, state.dir) != RC_NOT_CONNECTED) {
				const int ax = state.x + rcGetDirOffsetX(state.dir);
				const int ay = state.y + rcGetDirOffsetY(state.dir);
				const int ai = (int)chf.cells[ax + ay * chf.width].index + rcGetCon(s, state.dir);
				r = (int)chf.spans[ai].reg;
				if (cfg.area != chf.areas[ai]) {
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

			// --- Detect mandatory transitions ---
			// Mandatory: region or area-border flag changed from previous emission.
			// Bug: since vertices are emmited on the side of the closest to the direvction we are traveling,
			//      we need to check if the region of the next vertex is different from this one. I think we
			//      can do this by breaking if either of these are true and not emiting this vertex.
			//      (I need to think about this carefully)
			bool isMandatoryNonBV = false;
			bool isMandatoryBV = false;
			if (state.prev_r >= 0) {
				const bool differentRegs =
						(state.prev_r & RC_CONTOUR_REG_MASK) != (r & RC_CONTOUR_REG_MASK);
				const bool areaBorderChange =
						((state.prev_r & RC_AREA_BORDER) != 0) != ((r & RC_AREA_BORDER) != 0);
				if (differentRegs || areaBorderChange) {
					if (r & RC_BORDER_VERTEX) {
						isMandatoryBV = true;
					} else {
						isMandatoryNonBV = true;
					}
				}
			}

			// --- Reversal flip (border walks only) ---
			// Done BEFORE flag clearing so the flag-clear check uses the new
			// reversed value (matches stock walkContour behavior).
			if (cfg.flag_policy == FLAG_CLEAR_BORDER && isMandatoryNonBV) {
				state.reversed = !state.reversed;
			}

			state.prev_r = r;

			// --- Flag clearing per policy ---
			if (cfg.flag_policy == FLAG_CLEAR_ALWAYS) {
				flags[state.i] &= ~(1 << state.dir);
			} else if (cfg.flag_policy == FLAG_CLEAR_BORDER) {
				if (!state.reversed && (chf.spans[state.i].reg & RC_BORDER_REG)) {
					flags[state.i] &= ~(1 << state.dir);
				}
			}
			// FLAG_CLEAR_NEVER: skip flag clearing entirely.

			// --- Stop conditions ---
			// Return BEFORE dir rotation. state.dir holds the just-emitted
			// boundary edge direction. Caller rotates per state.reversed.
			if (isMandatoryNonBV && cfg.stop_at_mandatory_non_bv) {
				return WALK_STOP_AT_MANDATORY_NON_BV;
			}
			if (isMandatoryBV && cfg.stop_at_mandatory_bv) {
				return WALK_STOP_AT_MANDATORY_BV;
			}

			// --- Rotate ---
			// Forward: CW. Reversed (border phase): CCW.
			state.dir = state.reversed ? (state.dir + 3) & 0x3 : (state.dir + 1) & 0x3;
		} else {
			// --- No boundary edge: move to neighbor ---
			int ni = -1;
			const int nx = state.x + rcGetDirOffsetX(state.dir);
			const int ny = state.y + rcGetDirOffsetY(state.dir);
			const rcCompactSpan &s = chf.spans[state.i];
			if (rcGetCon(s, state.dir) != RC_NOT_CONNECTED) {
				const rcCompactCell &nc = chf.cells[nx + ny * chf.width];
				ni = (int)nc.index + rcGetCon(s, state.dir);
			}
			if (ni == -1) {
				return WALK_STOP_AT_DEAD_END;
			}

			// T-junction guard: during the reversed phase of a border walk,
			// don't step into non-border cells. Rotate CW (reversed move
			// direction) and continue.
			if (cfg.block_t_junction_steps && state.reversed &&
					!(chf.spans[ni].reg & RC_BORDER_REG)) {
				state.dir = (state.dir + 1) & 0x3;
			} else {
				state.x = nx;
				state.y = ny;
				state.i = ni;
				// Forward move: CCW. Reversed move: CW.
				state.dir = state.reversed ? (state.dir + 1) & 0x3 : (state.dir + 3) & 0x3;
			}
		}

		// Loop closure: returned to starting span and direction.
		if (state.i == loop_start_i && state.dir == loop_start_dir) {
			return WALK_STOP_LOOP_CLOSED;
		}
	}
	return WALK_STOP_AT_DEAD_END;
}

// Orchestrator for interior region contours. Single walkSegment call with
// FLAG_CLEAR_ALWAYS and no stop conditions — equivalent to the stock walk.
static void walkInteriorContour(int x, int y, int i,
		const rcCompactHeightfield &chf,
		unsigned char *flags, rcIntArray &points) {
	unsigned char dir = 0;
	while ((flags[i] & (1 << dir)) == 0) {
		dir++;
	}

	WalkState state = { x, y, i, dir, -1, false, 0 };
	WalkSegmentConfig cfg = {
		chf.areas[i],
		FLAG_CLEAR_ALWAYS,
		/*stop_at_mandatory_non_bv*/ false,
		/*stop_at_mandatory_bv*/ false,
		/*block_t_junction_steps*/ false,
	};
	walkSegment(state, i, dir, chf, flags, points, cfg);
}

// Forward declaration: simplifySegment is defined later in this file.
// findAndInjectClipPoint needs it for simplifying the inland sub-walk's raw
// vertices.
static void simplifySegment(const rcIntArray &raw_verts, int pn, int s0, int s_end,
		float maxError, rcIntArray &out, bool wrap_around);

// findAndInjectClipPoint — T-vertex side-trip implementation.
//
// Called by walkBorderContour when walkSegment returns WALK_STOP_AT_MANDATORY_BV.
// At this point, parent_points[]'s tail is the just-emitted T-vertex's 4 ints,
// state.dir holds the boundary edge direction we emitted across (pre-rotation),
// and state.prev_r equals the T-vertex's r value.
//
// The side-trip walks the R1-R2 inland boundary on R2's side (the cell
// diagonally opposite the parent across T) so that walkSegment's natural
// rotation/emission produces vertices going inland from T (the first emission
// is the next non-bv inland vertex past T). Walking on R1's side does NOT
// work the same way because R1's natural CW walk at the T-corner cell would
// emit T itself first (a duplicate of parent's T) instead of the desired
// inland vertex.
//
// Inland setup:
//   1. Step from parent across the just-emitted edge → cell A (one of R1/R2).
//   2. From cell A, find the perpendicular dir whose flag is set — that is
//      the R1-R2 boundary edge from A's side.
//   3. Step from cell A across the R1-R2 boundary edge → cell B (the other
//      interior region's cell, sharing T as a corner with A).
//   4. Inland walker on cell B faces back toward A (= (perp_dir + 2) & 3).
//      Its first emission is at the corner shared by A and B, one step
//      inland from T along the R1-R2 boundary.
//
// After walkSegment terminates at the first inland mandatory, the resulting
// raw verts are simplified using the same buildContourFromSegments rules as
// would apply to a segment with bv start (T, not in seg) and the inland
// mandatory as the end. simplified[0] is the inland clip-point — the first
// non-bv inland vertex (post-max-deviation).
//
// Injection: clip-point and a re-injection of the T-vertex are appended to
// parent_points[]. The dual-T pattern produces a points[] structure that
// buildContourFromSegments processes into the desired
// [chunk-corner-A, clip-point, chunk-corner-B] 3-vertex border contour
// (per the plan's "Why dual-T injection works" analysis).
//
// Degenerate cases (skip injection, leaving the T as a silent bv delimiter
// — preserves the prior 2-vert-discard behavior for that one T):
//   - parent's just-emitted edge is RC_NOT_CONNECTED
//   - cell A has no flagged perpendicular edge (R1 doesn't extend inland)
//   - cell A's perpendicular neighbor is RC_NOT_CONNECTED
//   - inland walk produces no raw verts (immediate dead-end)
//   - synthetic seg is empty (all inland verts were bv)
//   - simplifySegment returns empty
static void findAndInjectClipPoint(
		rcContext *ctx,
		WalkState &state,
		const rcCompactHeightfield &chf,
		unsigned char *flags,
		rcIntArray &parent_points,
		float maxError) {
	// 1. Save the just-emitted T-vertex from parent_points tail.
	const int n = parent_points.size();
	if (n < 4) {
		return; // Should not happen — T was just emitted.
	}
	const int T_x = parent_points[n - 4];
	const int T_y = parent_points[n - 3];
	const int T_z = parent_points[n - 2];
	const int T_r = parent_points[n - 1];

	// 2. Step across the just-emitted boundary edge → cell A.
	const rcCompactSpan &parent_span = chf.spans[state.i];
	if (rcGetCon(parent_span, state.dir) == RC_NOT_CONNECTED) {
		ctx->log(RC_LOG_WARNING,
				"T-side-trip degenerate at (%d,%d): parent's emit edge has no neighbor.",
				state.x, state.y);
		return;
	}
	const int A_x = state.x + rcGetDirOffsetX(state.dir);
	const int A_y = state.y + rcGetDirOffsetY(state.dir);
	const int A_i = (int)chf.cells[A_x + A_y * chf.width].index +
			rcGetCon(parent_span, state.dir);

	// 3. Find perpendicular dir from A to B. Try both perpendiculars; pick the
	//    one with a flag set on cell A — that's the R1-R2 boundary edge.
	const unsigned char d1 = (state.dir + 1) & 0x3;
	const unsigned char d3 = (state.dir + 3) & 0x3;
	unsigned char perp_dir;
	if (flags[A_i] & (1 << d1)) {
		perp_dir = d1;
	} else if (flags[A_i] & (1 << d3)) {
		perp_dir = d3;
	} else {
		ctx->log(RC_LOG_WARNING,
				"T-side-trip degenerate at (%d,%d): no flagged perpendicular edge "
				"on inland cell — R1 doesn't extend inland at this T.",
				state.x, state.y);
		return;
	}

	// 4. Step from cell A across the R1-R2 boundary → cell B.
	const rcCompactSpan &A_span = chf.spans[A_i];
	if (rcGetCon(A_span, perp_dir) == RC_NOT_CONNECTED) {
		ctx->log(RC_LOG_WARNING,
				"T-side-trip degenerate at (%d,%d): R1-R2 boundary neighbor RC_NOT_CONNECTED.",
				state.x, state.y);
		return;
	}
	const int B_x = A_x + rcGetDirOffsetX(perp_dir);
	const int B_y = A_y + rcGetDirOffsetY(perp_dir);
	const int B_i = (int)chf.cells[B_x + B_y * chf.width].index +
			rcGetCon(A_span, perp_dir);

	// 5. Inland walker on cell B faces back toward cell A. The R1-R2 boundary
	//    edge from B's side has the flag set (cell B's neighbor in this dir
	//    is cell A, a different region from B).
	const unsigned char inland_dir = (perp_dir + 2) & 0x3;

	// 6. Run inland walkSegment. FLAG_CLEAR_NEVER preserves flags so the
	//    eventual interior contour walk on R1/R2 isn't starved by our
	//    side-trip. Stop on first mandatory (either bv or non-bv).
	WalkState inland_state = { B_x, B_y, B_i, inland_dir, -1, false, state.iter };
	WalkSegmentConfig inland_cfg = {
		chf.areas[B_i],
		FLAG_CLEAR_NEVER,
		/*stop_at_mandatory_non_bv*/ true,
		/*stop_at_mandatory_bv*/ true,
		/*block_t_junction_steps*/ false,
	};

	rcIntArray inland_raw(64);
	walkSegmentOld(inland_state, B_i, inland_dir, chf, flags, inland_raw, inland_cfg);

	// Carry the iter counter back to parent so the 40000 cap covers the
	// combined walk + side-trip.
	state.iter = inland_state.iter;

	// 7. Build a synthetic seg following buildContourFromSegments rules for a
	//    segment with bv start (T, not in seg) and end mandatory (pushed if
	//    non-bv). seg holds raw indices into inland_raw.
	const int inland_count = inland_raw.size() / 4;
	if (inland_count < 1) {
		ctx->log(RC_LOG_WARNING,
				"T-side-trip degenerate at (%d,%d): inland walk produced no raw verts.",
				state.x, state.y);
		return;
	}

	rcIntArray seg(32);
	for (int j = 0; j < inland_count - 1; ++j) {
		if (!(inland_raw[j * 4 + 3] & RC_BORDER_VERTEX)) {
			seg.push(j);
		}
	}
	if (!(inland_raw[(inland_count - 1) * 4 + 3] & RC_BORDER_VERTEX)) {
		seg.push(inland_count - 1);
	}

	if (seg.size() < 1) {
		ctx->log(RC_LOG_WARNING,
				"T-side-trip degenerate at (%d,%d): all inland raw verts were RC_BORDER_VERTEX.",
				state.x, state.y);
		return;
	}

	// 8. simplifySegment on the synthetic seg (lex-XZ direction-independent).
	rcIntArray simplified(16);
	simplifySegment(inland_raw, inland_count, seg[0], seg[seg.size() - 1],
			maxError, simplified, /*wrap_around*/ false);

	if (simplified.size() < 4) {
		ctx->log(RC_LOG_WARNING,
				"T-side-trip degenerate at (%d,%d): simplifySegment returned empty.",
				state.x, state.y);
		return;
	}

	// 9. Inject clip-point (simplified[0], the first non-bv inland vertex past
	//    T after max-deviation thinning) into parent_points[].
	const int clip_x = simplified[0];
	const int clip_y = simplified[1];
	const int clip_z = simplified[2];
	const int clip_raw_idx = simplified[3];
	const int clip_r = inland_raw[clip_raw_idx * 4 + 3];

	parent_points.push(clip_x);
	parent_points.push(clip_y);
	parent_points.push(clip_z);
	parent_points.push(clip_r);

	// 10. Re-inject the T-vertex as the return marker. The dual-T structure
	//     gives buildContourFromSegments the segment shape:
	//       [..., T(bv), clip(non-bv), T(bv), ...]
	//     which produces the correct 3-vertex border contour for a single-T
	//     case. state.prev_r is left equal to T_r (set by walkSegment), which
	//     matches the most-recently-appended vertex.
	parent_points.push(T_x);
	parent_points.push(T_y);
	parent_points.push(T_z);
	parent_points.push(T_r);
}

// Orchestrator for border region contours. walkSegment runs with
// FLAG_CLEAR_BORDER (which also enables internal reversal at mandatory
// non-bv emissions) and the T-junction guard. T-vertex stops trigger
// findAndInjectClipPoint (Phase 3 stub; Phase 4 implementation does the
// actual inland side-trip).
//
// The orchestrator manually rotates state.dir after a stop (walkSegment
// returns BEFORE rotation on stops) and re-checks loop closure after
// rotation, since walkSegment's end-of-iteration closure check is skipped
// when it returns early.
static void walkChunkContours(rcContext *ctx, int x, int y, int i,
		const rcCompactHeightfield &chf,
		unsigned char *flags, rcIntArray &points,
		float maxError) {
	unsigned char dir = 0;
	while ((flags[i] & (1 << dir)) == 0) {
		dir++;
	}

	WalkState state = { x, y, i, dir, -1, false, 0 };
	const int loop_start_i = i;
	const unsigned char loop_start_dir = dir;

	WalkSegmentConfig cfg = {
		chf.areas[i],
		FLAG_CLEAR_BORDER,
		/*stop_at_mandatory_non_bv*/ false, // reversal handled internally by walkSegment
		/*stop_at_mandatory_bv*/ true, // T-vertex: trigger side-trip
		/*block_t_junction_steps*/ true,
	};

	while (true) {
		WalkStopReason r = walkSegment(state, loop_start_i, loop_start_dir,
				chf, flags, points, cfg);

		if (r == WALK_STOP_LOOP_CLOSED || r == WALK_STOP_AT_DEAD_END) {
			break;
		}

		if (r == WALK_STOP_AT_MANDATORY_BV) {
			// T-vertex emitted. Phase 3: stub call (no-op).
			// Reversed-phase guard: only side-trip during forward phase.
			// During reversed phase the forward phase already injected for
			// this T — re-injecting would duplicate. Phase 3 stub doesn't
			// inject anything anyway, but the guard is added now for Phase 4
			// readiness.
			if (!state.reversed) {
				findAndInjectClipPoint(ctx, state, chf, flags, points, maxError);
			}

			// Rotate per state.reversed — matches walkSegment's internal
			// rotation that would have happened had we not stopped.
			state.dir = state.reversed ? (state.dir + 3) & 0x3 : (state.dir + 1) & 0x3;

			// Loop closure check after rotation (walkSegment skipped the
			// end-of-iter check because it returned early on the stop).
			if (state.i == loop_start_i && state.dir == loop_start_dir) {
				break;
			}
			continue;
		}

		// WALK_STOP_AT_MANDATORY_NON_BV is not expected here (cfg disables it).
		break;
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

	const int w = chf.width;  // x width in xz plane
	const int h = chf.height; // z width in xz plane
	const int borderSize = chf.borderSize;

	// --- Setup (stock lines 831-865) ---
	// TODO: In our use case we should probably adjust the y coords for our chunk padding
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
	for (int y = 0; y < h; ++y) {                                                        //
		for (int x = 0; x < w; ++x) {                                                    // Iterate over all spans by iterating over
			const rcCompactCell &c = chf.cells[x + y * w];                               // all spans in every xz (yx) column.
			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i) {     //
				
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
				for (int dir = 0; dir < 4; ++dir) { // Check what spans connected to this span is connected to (we also check border spans here)
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

	for (int y = 0; y < h; ++y) {                                                        //
		for (int x = 0; x < w; ++x) {                                                    // Iterate over all spans by iterating over
			const rcCompactCell &c = chf.cells[x + y * w];                               // all spans in every xz (yx) column.
			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i) {     //
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

				// Walk the contour — produces raw_verts. Border walks may grow
				// the points[] array via T-vertex side-trips (Phase 4+); for
				// now, behavior is identical to the previous monolithic
				// walkContour.
				if (is_border_walk) {
					walkBorderContour(ctx, x, y, i, chf, flags, verts, maxError);
				} else {
					walkInteriorContour(x, y, i, chf, flags, verts);
				}

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

//#endif // VOXEL_ENABLE_NAVIGATION
