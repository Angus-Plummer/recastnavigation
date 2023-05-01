//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"
#include "RecastOptimisationToggle.h" //@HG


/// Check whether two bounding boxes overlap
///
/// @param[in]	aMin	Min axis extents of bounding box A
/// @param[in]	aMax	Max axis extents of bounding box A
/// @param[in]	bMin	Min axis extents of bounding box B
/// @param[in]	bMax	Max axis extents of bounding box B
/// @returns true if the two bounding boxes overlap.  False otherwise.
inline bool overlapBounds(const rcReal* amin, const rcReal* amax, const rcReal* bmin, const rcReal* bmax)
{
	bool overlap = true;
	overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
	overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
	overlap = (amin[2] > bmax[2] || amax[2] < bmin[2]) ? false : overlap;
	return overlap;
}

inline bool overlapInterval(unsigned short amin, unsigned short amax,
							unsigned short bmin, unsigned short bmax)
{
	if (amax < bmin) return false;
	if (amin > bmax) return false;
	return true;
}


/// Allocates a new span in the heightfield.
/// Use a memory pool and free list to minimize actual allocations.
/// 
/// @param[in]	hf		The heightfield
/// @returns A pointer to the allocated or re-used span memory. 
static rcSpan* allocSpan(rcHeightfield& hf)
{
	// If necessary, allocate new page and update the freelist.
	if (hf.freelist == NULL || hf.freelist->next == NULL)
	{
		// Create new page.
		// Allocate memory for the new pool.
		rcSpanPool* spanPool = (rcSpanPool*)rcAlloc(sizeof(rcSpanPool), RC_ALLOC_PERM);
		if (spanPool == NULL)
		{
			return NULL;
		}

		// Add the pool into the list of pools.
		spanPool->next = hf.pools;
		hf.pools = spanPool;
		
		// Add new spans to the free list.
		rcSpan* freeList = hf.freelist;
		rcSpan* head = &spanPool->items[0];
		rcSpan* it = &spanPool->items[RC_SPANS_PER_POOL];
		do
		{
			--it;
			it->next = freeList;
			freeList = it;
		}
		while (it != head);
		hf.freelist = it;
	}

	// Pop item from the front of the free list.
	rcSpan* newSpan = hf.freelist;
	hf.freelist = hf.freelist->next;
	return newSpan;
}

/// Releases the memory used by the span back to the heightfield, so it can be re-used for new spans.
/// @param[in]	hf		The heightfield.
/// @param[in]	span	A pointer to the span to free
static void freeSpan(rcHeightfield& hf, rcSpan* span)
{
	if (span == NULL)
	{
		return;
	}
	// Add the span to the front of the free list.
	span->next = hf.freelist;
	hf.freelist = span;
}

/// Adds a span to the heightfield.  If the new span overlaps existing spans,
/// it will merge the new span with the existing ones.
///
/// @param[in]	hf					Heightfield to add spans to
/// @param[in]	x					The new span's column cell x index
/// @param[in]	z					The new span's column cell z index
/// @param[in]	min					The new span's minimum cell index
/// @param[in]	max					The new span's maximum cell index
/// @param[in]	areaID				The new span's area type ID
/// @param[in]	flagMergeThreshold	How close two spans maximum extents need to be to merge area type IDs
static bool addSpan(rcHeightfield& hf,
                    const int x, const int z,
                    const unsigned short min, const unsigned short max,
                    const unsigned char areaID, const int flagMergeThreshold)
{
	// Create the new span.
	rcSpan* newSpan = allocSpan(hf);
	if (newSpan == NULL)
	{
		return false;
	}
	newSpan->data.smin = min;
	newSpan->data.smax = max;
	newSpan->data.area = areaID;
	newSpan->next = NULL;
	
	const int columnIndex = x + z * hf.width;
	rcSpan* previousSpan = NULL;
	rcSpan* currentSpan = hf.spans[columnIndex];
	
	// Insert the new span, possibly merging it with existing spans.
	while (currentSpan != NULL)
	{
		if (currentSpan->data.smin > newSpan->data.smax)
		{
			// Current span is completely after the new span, break.
			break;
		}
		
		if (currentSpan->data.smax < newSpan->data.smin)
		{
			// Current span is completely before the new span.  Keep going.
			previousSpan = currentSpan;
			currentSpan = currentSpan->next;
		}
		else
		{

			// Merge height intervals.
			if (currentSpan->data.smin < newSpan->data.smin)
			{
				newSpan->data.smin = currentSpan->data.smin;
			}
			if (currentSpan->data.smax > newSpan->data.smax)
			{
				newSpan->data.smax = currentSpan->data.smax;
			}
			

			// Merge flags.

			// For spans whose tops are really close to each other, prefer walkable areas.
			// This is done in order to remove aliasing (similar to z-fighting) on surfaces close to each other.
			if (rcAbs((int)newSpan->data.smax - (int)currentSpan->data.smax) <= flagMergeThreshold)
			{
				// Higher area ID numbers indicate higher resolution priority.
				newSpan->data.area = rcMax(newSpan->data.area, currentSpan->data.area);
			}
			// @HG BEGIN
			else if (currentSpan->data.smax > newSpan->data.smax)
			{
				// Use the new spans area if it will become the top.
				newSpan->data.area = currentSpan->data.area;
			}
			// @HG END
			
			// Remove the current span since it's now merged with newSpan.
			// Keep going because there might be other overlapping spans that also need to be merged.
			rcSpan* next = currentSpan->next;
			freeSpan(hf, currentSpan);
			if (previousSpan)
			{
				previousSpan->next = next;
			}
			else
			{
				hf.spans[columnIndex] = next;
			}
			currentSpan = next;
		}
	}
	
	// Insert new span after prev
	if (previousSpan != NULL)
	{
		newSpan->next = previousSpan->next;
		previousSpan->next = newSpan;
	}
	else
	{
		// This span should go before the others in the list
		newSpan->next = hf.spans[columnIndex];
		hf.spans[columnIndex] = newSpan;
	}

	return true;
}

/// @par
///
/// The span addition can be set to favor flags. If the span is merged to
/// another span and the new @p smax is within @p flagMergeThr units
/// from the existing span, the span flags are merged.
///
/// @see rcHeightfield, rcSpan.
bool rcAddSpan(rcContext* context, rcHeightfield& heightfield,
               const int x, const int z,
               const unsigned short spanMin, const unsigned short spanMax,
               const unsigned char areaID, const int flagMergeThreshold)

{
	if (!addSpan(heightfield, x, z, spanMin, spanMax, areaID, flagMergeThreshold))
	{
		context->log(RC_LOG_ERROR, "rcAddSpan: Out of memory.");
		return false;
	}

	return true;
}


void rcAddSpans(rcContext* /*ctx*/, rcHeightfield& heightfield, const int flagMergeThr,
				const rcSpanCache* cachedSpans, const int nspans)
{
	const rcSpanCache* cachedInfo = cachedSpans;
	for (int i = 0; i < nspans; i++, cachedInfo++)
	{
		addSpan(heightfield, cachedInfo->x, cachedInfo->y, cachedInfo->data.smin, cachedInfo->data.smax, cachedInfo->data.area, flagMergeThr);
	}
}

int rcCountSpans(rcContext* /*ctx*/, rcHeightfield& heightfield)
{
	if (heightfield.width > 0xffff || heightfield.height > 0xffff)
	{
		return 0;
	}

	int numSpans = 0;
	for (rcSpanPool* pool = heightfield.pools; pool; pool = pool->next)
	{
		numSpans += RC_SPANS_PER_POOL;
	}

	for (rcSpan* s = heightfield.freelist; s; s = s->next)
	{
		numSpans--;
	}

	return numSpans;
}

void rcCacheSpans(rcContext* /*ctx*/, rcHeightfield& heightfield, rcSpanCache* cachedSpans)
{
	rcSpanCache* cachedInfo = cachedSpans;
	for (int iz = 0; iz < heightfield.height; iz++)
	{
		for (int ix = 0; ix < heightfield.width; ix++)
		{
			const int idx = ix + (iz * heightfield.width);
			for (rcSpan* s = heightfield.spans[idx]; s; s = s->next)
			{
				cachedInfo->x = (unsigned short)ix;
				cachedInfo->y = (unsigned short)iz;
				cachedInfo->data = s->data;
				cachedInfo++;
			}
		}
	}
}

enum rcAxis
{
	RC_AXIS_X = 0,
	RC_AXIS_Y = 1,
	RC_AXIS_Z = 2
};


/// Divides a convex polygon of max 12 vertices into two convex polygons
/// across a separating axis.
/// 
/// @param[in]	inVerts			The input polygon vertices
/// @param[in]	inVertsCount	The number of input polygon vertices
/// @param[out]	outVerts1		Resulting polygon 1's vertices
/// @param[out]	outVerts1Count	The number of resulting polygon 1 vertices
/// @param[out]	outVerts2		Resulting polygon 2's vertices
/// @param[out]	outVerts2Count	The number of resulting polygon 2 vertices
/// @param[in]	axisOffset		THe offset along the specified axis
/// @param[in]	axis			The separating axis
static void dividePoly(const rcReal* inVerts, int inVertsCount,
                       rcReal* outVerts1, int* outVerts1Count,
                       rcReal* outVerts2, int* outVerts2Count,
                       rcReal axisOffset, rcAxis axis)
{
	rcAssert(inVertsCount <= 12);
	
	// How far positive or negative away from the separating axis is each vertex.
	rcReal inVertAxisDelta[12];
	for (int inVert = 0; inVert < inVertsCount; ++inVert)
	{
		inVertAxisDelta[inVert] = axisOffset - inVerts[inVert * 3 + axis];
	}

	int poly1Vert = 0;
	int poly2Vert = 0;
	for (int inVertA = 0, inVertB = inVertsCount - 1; inVertA < inVertsCount; inVertB = inVertA, ++inVertA)
	{
		// If the two vertices are on the same side of the separating axis
		bool sameSide = (inVertAxisDelta[inVertA] >= 0) == (inVertAxisDelta[inVertB] >= 0);

		if (!sameSide)
		{
			rcReal s = inVertAxisDelta[inVertB] / (inVertAxisDelta[inVertB] - inVertAxisDelta[inVertA]);
			outVerts1[poly1Vert * 3 + 0] = inVerts[inVertB * 3 + 0] + (inVerts[inVertA * 3 + 0] - inVerts[inVertB * 3 + 0]) * s;
			outVerts1[poly1Vert * 3 + 1] = inVerts[inVertB * 3 + 1] + (inVerts[inVertA * 3 + 1] - inVerts[inVertB * 3 + 1]) * s;
			outVerts1[poly1Vert * 3 + 2] = inVerts[inVertB * 3 + 2] + (inVerts[inVertA * 3 + 2] - inVerts[inVertB * 3 + 2]) * s;
			rcVcopy(&outVerts2[poly2Vert * 3], &outVerts1[poly1Vert * 3]);
			poly1Vert++;
			poly2Vert++;
			
			// add the inVertA point to the right polygon. Do NOT add points that are on the dividing line
			// since these were already added above
			if (inVertAxisDelta[inVertA] > 0)
			{
				rcVcopy(&outVerts1[poly1Vert * 3], &inVerts[inVertA * 3]);
				poly1Vert++;
			}
			else if (inVertAxisDelta[inVertA] < 0)
			{
				rcVcopy(&outVerts2[poly2Vert * 3], &inVerts[inVertA * 3]);
				poly2Vert++;
			}
		}
		else
		{
			// add the inVertA point to the right polygon. Addition is done even for points on the dividing line
			if (inVertAxisDelta[inVertA] >= 0)
			{
				rcVcopy(&outVerts1[poly1Vert * 3], &inVerts[inVertA * 3]);
				poly1Vert++;
				if (inVertAxisDelta[inVertA] != 0)
				{
					continue;
				}
			}
			rcVcopy(&outVerts2[poly2Vert * 3], &inVerts[inVertA * 3]);
			poly2Vert++;
		}
	}

	*outVerts1Count = poly1Vert;
	*outVerts2Count = poly2Vert;
}

static int clipPoly(const rcReal* in, int n, rcReal* out, rcReal pnx, rcReal pnz, rcReal pd)
{
	rcReal d[12];
	for (int i = 0; i < n; ++i)
		d[i] = pnx*in[i*3+0] + pnz*in[i*3+2] + pd;
	
	int m = 0;
	for (int i = 0, j = n-1; i < n; j=i, ++i)
	{
		bool ina = d[j] >= 0;
		bool inb = d[i] >= 0;
		if (ina != inb)
		{
			rcReal s = d[j] / (d[j] - d[i]);
			out[m*3+0] = in[j*3+0] + (in[i*3+0] - in[j*3+0])*s;
			out[m*3+1] = in[j*3+1] + (in[i*3+1] - in[j*3+1])*s;
			out[m*3+2] = in[j*3+2] + (in[i*3+2] - in[j*3+2])*s;
			m++;
		}
		if (inb)
		{
			out[m*3+0] = in[i*3+0];
			out[m*3+1] = in[i*3+1];
			out[m*3+2] = in[i*3+2];
			m++;
		}
	}
	return m;
}



#if HG_ADDITION_USE_NEW_RECAST_RASTERIZER

static inline int intMax(int a, int b)
{
	return a < b ? b : a;
}

static inline int intMin(int a, int b)
{
	return a < b ? a : b;
}


static inline void addFlatSpanSample(rcHeightfield& hf, const int x, const int y)
{
	hf.RowExt[y + 1].MinCol = intMin(hf.RowExt[y + 1].MinCol, x);
	hf.RowExt[y + 1].MaxCol = intMax(hf.RowExt[y + 1].MaxCol, x);

}

static inline int SampleIndex(rcHeightfield const& hf, const int x, const int y)
{
	return x + 1 + (y + 1)*(hf.width + 2);
}

static inline void addSpanSample(rcHeightfield& hf, const int x, const int y, short int sint)
{
	addFlatSpanSample(hf, x, y);
	int idx = SampleIndex(hf, x, y);
	rcTempSpan& Temp = hf.tempspans[idx];

	Temp.sminmax[0] = Temp.sminmax[0] > sint ? sint : Temp.sminmax[0];
	Temp.sminmax[1] = Temp.sminmax[1] < sint ? sint : Temp.sminmax[1];
}


static inline void intersectX(const rcReal* v0, const rcReal* edge, rcReal cx, rcReal *pnt)
{
	rcReal t = rcClamp((cx - v0[0]) * edge[9 + 0], 0.0f, 1.0f);  // inverses

	pnt[0] = v0[0] + t * edge[0];
	pnt[1] = v0[1] + t * edge[1];
	pnt[2] = v0[2] + t * edge[2];
}

static inline void intersectZ(const rcReal* v0, const rcReal* edge, rcReal cz, rcReal *pnt)
{
	rcReal t = rcClamp((cz - v0[2]) * edge[9 + 2], 0.0f, 1.0f); //inverses

	pnt[0] = v0[0] + t * edge[0];
	pnt[1] = v0[1] + t * edge[1];
	pnt[2] = v0[2] + t * edge[2];
}


static bool rasterizeTri(const rcReal* v0, const rcReal* v1, const rcReal* v2,
						 const unsigned char area, rcHeightfield& hf,
						 const rcReal* bmin, const rcReal* bmax,
						 const rcReal cs, const rcReal ics, const rcReal ich, 
						 const int flagMergeThr,
						 const int rasterizationFlags,
	                     const int* rasterizationMasks )
{
	rcEdgeHit* const hfEdgeHits = hf.EdgeHits; //this prevents a static analysis warning

	const int w = hf.width;
	const int h = hf.height;
	const rcReal by = bmax[1] - bmin[1];
	const int projectTriToBottom = rasterizationFlags & RC_PROJECT_TO_BOTTOM;

	int intverts[3][2];

	intverts[0][0] = (int)rcFloor((v0[0] - bmin[0])*ics);
	intverts[0][1] = (int)rcFloor((v0[2] - bmin[2])*ics);
	intverts[1][0] = (int)rcFloor((v1[0] - bmin[0])*ics);
	intverts[1][1] = (int)rcFloor((v1[2] - bmin[2])*ics);
	intverts[2][0] = (int)rcFloor((v2[0] - bmin[0])*ics);
	intverts[2][1] = (int)rcFloor((v2[2] - bmin[2])*ics);

	int x0 = intMin(intverts[0][0], intMin(intverts[1][0], intverts[2][0]));
	int x1 = intMax(intverts[0][0], intMax(intverts[1][0], intverts[2][0]));
	int y0 = intMin(intverts[0][1], intMin(intverts[1][1], intverts[2][1]));
	int y1 = intMax(intverts[0][1], intMax(intverts[1][1], intverts[2][1]));

	if (x1 < 0 || x0 >= w || y1 < 0 || y0 >= h)
		return true;

	// Calculate min and max of the triangle

	rcReal triangle_smin = rcMin(rcMin(v0[1], v1[1]), v2[1]);
	rcReal triangle_smax = rcMax(rcMax(v0[1], v1[1]), v2[1]);
	triangle_smin -= bmin[1];
	triangle_smax -= bmin[1];
	// Skip the span if it is outside the heightfield bbox
	if (triangle_smax < 0.0f) return true;
	if (triangle_smin > by) return true;

	if (x0 == x1 && y0 == y1)
	{
		// Clamp the span to the heightfield bbox.
		if (triangle_smin < 0.0f) triangle_smin = 0.0f;
		if (triangle_smax > by) triangle_smax = by;

		// Snap the span to the heightfield height grid.
		unsigned short triangle_ismin = (unsigned short)rcClamp((int)rcFloor(triangle_smin * ich), 0, RC_SPAN_MAX_HEIGHT);
		unsigned short triangle_ismax = (unsigned short)rcClamp((int)rcCeil(triangle_smax * ich), (int)triangle_ismin+1, RC_SPAN_MAX_HEIGHT);
		const int projectSpanToBottom = rasterizationMasks != nullptr ? (projectTriToBottom & rasterizationMasks[x0+y0*w]) : projectTriToBottom;
		if (projectSpanToBottom)
		{
			triangle_ismin = 0;
		}

		addSpan(hf, x0, y0, triangle_ismin, triangle_ismax, area, flagMergeThr);
		return true;
	}

	const short int triangle_ismin = (short int)rcClamp((int)rcFloor(triangle_smin * ich), -32000, 32000);
	const short int triangle_ismax = (short int)rcClamp((int)rcFloor(triangle_smax * ich), -32000, 32000);

	x0 = intMax(x0, 0);
	int x1_edge = intMin(x1, w);
	x1 = intMin(x1, w - 1);
	y0 = intMax(y0, 0);
	int y1_edge = intMin(y1, h);
	y1 = intMin(y1, h - 1);


	rcReal edges[6][3];

	rcReal vertarray[3][3];
	rcVcopy(vertarray[0], v0);
	rcVcopy(vertarray[1], v1);
	rcVcopy(vertarray[2], v2);

	bool doFlat = true;
	if (doFlat && triangle_ismin == triangle_ismax)
	{
		// flat horizontal, much faster
		for (int basevert = 0; basevert < 3; basevert++)
		{
			int othervert = basevert == 2 ? 0 : basevert + 1;
			int edge = basevert == 0 ? 2 : basevert - 1;

			rcVsub(&edges[edge][0], vertarray[othervert], vertarray[basevert]);
			//rcVnormalize(&edges[edge][0]);
			edges[3 + edge][0] = 1.0f / edges[edge][0];
			edges[3 + edge][1] = 1.0f / edges[edge][1];
			edges[3 + edge][2] = 1.0f / edges[edge][2];

			// drop the vert into the temp span area
			if (intverts[basevert][0] >= x0 && intverts[basevert][0] <= x1 && intverts[basevert][1] >= y0 && intverts[basevert][1] <= y1)
			{
				addFlatSpanSample(hf, intverts[basevert][0], intverts[basevert][1]);
			}
			// set up the edge intersections with horizontal planes
			if (intverts[basevert][1] != intverts[othervert][1])
			{
				int edge0 = intMin(intverts[basevert][1], intverts[othervert][1]);
				int edge1 = intMax(intverts[basevert][1], intverts[othervert][1]);
				int loop0 = intMax(edge0 + 1, y0);
				int loop1 = intMin(edge1, y1_edge);

				unsigned char edgeBits = (edge << 4) | (othervert << 2) | basevert;
				for (int y = loop0; y <= loop1; y++)
				{
					int HitIndex = !!hfEdgeHits[y].Hits[0];
					hfEdgeHits[y].Hits[HitIndex] = edgeBits;
				}
			}
			// do the edge intersections with vertical planes
			if (intverts[basevert][0] != intverts[othervert][0])
			{
				int edge0 = intMin(intverts[basevert][0], intverts[othervert][0]);
				int edge1 = intMax(intverts[basevert][0], intverts[othervert][0]);
				int loop0 = intMax(edge0 + 1, x0);
				int loop1 = intMin(edge1, x1_edge);

				rcReal temppnt[3];
				rcReal cx = bmin[0] + cs * loop0;
				for (int x = loop0; x <= loop1; x++, cx += cs)
				{
					intersectX(vertarray[basevert], &edges[edge][0], cx, temppnt);
					int y = (int)rcFloor((temppnt[2] - bmin[2])*ics);
					if (y >= y0 && y <= y1)
					{
						addFlatSpanSample(hf, x, y);
						addFlatSpanSample(hf, x - 1, y);
					}
				}
			}
		}
		{
			// deal with the horizontal intersections 
			int edge0 = intMin(intverts[0][1], intMin(intverts[1][1],intverts[2][1]));
			int edge1 = intMax(intverts[0][1], intMax(intverts[1][1],intverts[2][1]));
			int loop0 = intMax(edge0 + 1, y0);
			int loop1 = intMin(edge1, y1_edge);

			rcReal Inter[2][3];
			int xInter[2];

			rcReal cz = bmin[2] + cs * loop0;
			for (int y = loop0; y <= loop1; y++, cz += cs)
			{
				rcEdgeHit& Hits = hfEdgeHits[y];
				if (Hits.Hits[0])
				{
					rcAssert(Hits.Hits[1]); // must have two hits

					for (int i = 0; i < 2; i++)
					{
						int edge = Hits.Hits[i] >> 4;
						int othervert = (Hits.Hits[i] >> 2) & 3;
						int basevert = Hits.Hits[i] & 3;

						intersectZ(vertarray[basevert], &edges[edge][0], cz, Inter[i]);
						int x = (int)rcFloor((Inter[i][0] - bmin[0])*ics);
						xInter[i] = x;
						if (x >= x0 && x <= x1)
						{
							addFlatSpanSample(hf, x, y);
							addFlatSpanSample(hf, x, y - 1);
						}
					}
					if (xInter[0] != xInter[1])
					{
						// now fill in the fully contained ones.
						int left = Inter[1][0] < Inter[0][0];  
						int xloop0 = intMax(xInter[left] + 1, x0);
						int xloop1 = intMin(xInter[1 - left], x1);
						if (xloop0 <= xloop1)
						{
							addFlatSpanSample(hf, xloop0, y);
							addFlatSpanSample(hf, xloop1, y);
							addFlatSpanSample(hf, xloop0 - 1, y);
							addFlatSpanSample(hf, xloop1 - 1, y);
							addFlatSpanSample(hf, xloop0, y - 1);
							addFlatSpanSample(hf, xloop1, y - 1);
							addFlatSpanSample(hf, xloop0 - 1, y - 1);
							addFlatSpanSample(hf, xloop1 - 1, y - 1);
						}
					}
					// reset for next triangle
					Hits.Hits[0] = 0;
					Hits.Hits[1] = 0;
				}
			}
		}

		if (rasterizationMasks == nullptr)
		{
			// Snap the span to the heightfield height grid.
			unsigned short triangle_ismin_clamp = (unsigned short)rcClamp((int)triangle_ismin, 0, RC_SPAN_MAX_HEIGHT);
			const unsigned short triangle_ismax_clamp = (unsigned short)rcClamp((int)triangle_ismax, (int)triangle_ismin_clamp+1, RC_SPAN_MAX_HEIGHT);
			if (projectTriToBottom)
			{
				triangle_ismin_clamp = 0;
			}

			for (int y = y0; y <= y1; y++)
			{
				int xloop0 = intMax(hf.RowExt[y + 1].MinCol, x0);
				int xloop1 = intMin(hf.RowExt[y + 1].MaxCol, x1);
				for (int x = xloop0; x <= xloop1; x++)
				{
					addSpan(hf, x, y, triangle_ismin_clamp, triangle_ismax_clamp, area, flagMergeThr);
				}

				// reset for next triangle
				hf.RowExt[y + 1].MinCol = hf.width + 2;
				hf.RowExt[y + 1].MaxCol = -2;
			}
		}
		else
		{
			for (int y = y0; y <= y1; y++)
			{
				int xloop0 = intMax(hf.RowExt[y + 1].MinCol, x0);
				int xloop1 = intMin(hf.RowExt[y + 1].MaxCol, x1);
				for (int x = xloop0; x <= xloop1; x++)
				{
					// Snap the span to the heightfield height grid.
					unsigned short triangle_ismin_clamp = (unsigned short)rcClamp((int)triangle_ismin, 0, RC_SPAN_MAX_HEIGHT);
					const unsigned short triangle_ismax_clamp = (unsigned short)rcClamp((int)triangle_ismax, (int)triangle_ismin_clamp+1, RC_SPAN_MAX_HEIGHT);
					const int projectSpanToBottom = projectTriToBottom & rasterizationMasks[x+y*w];
					if (projectSpanToBottom)
					{
						triangle_ismin_clamp = 0;
					}
					addSpan(hf, x, y, triangle_ismin_clamp, triangle_ismax_clamp, area, flagMergeThr);
				}

				// reset for next triangle
				hf.RowExt[y + 1].MinCol = hf.width + 2;
				hf.RowExt[y + 1].MaxCol = -2;
			}
		}
	}
	else
	{
		//non-flat case
		for (int basevert = 0; basevert < 3; basevert++)
		{
			int othervert = basevert == 2 ? 0 : basevert + 1;
			int edge = basevert == 0 ? 2 : basevert - 1;

			rcVsub(&edges[edge][0], vertarray[othervert], vertarray[basevert]);
			//rcVnormalize(&edges[edge][0]);
			edges[3 + edge][0] = 1.0f / edges[edge][0];
			edges[3 + edge][1] = 1.0f / edges[edge][1];
			edges[3 + edge][2] = 1.0f / edges[edge][2];

			// drop the vert into the temp span area
			if (intverts[basevert][0] >= x0 && intverts[basevert][0] <= x1 && intverts[basevert][1] >= y0 && intverts[basevert][1] <= y1)
			{
				rcReal sfloat = vertarray[basevert][1] - bmin[1];
				short int sint = (short int)rcClamp((int)rcFloor(sfloat * ich), -32000, 32000);
				addSpanSample(hf, intverts[basevert][0], intverts[basevert][1], sint);
			}
			// set up the edge intersections with horizontal planes
			if (intverts[basevert][1] != intverts[othervert][1])
			{
				int edge0 = intMin(intverts[basevert][1], intverts[othervert][1]);
				int edge1 = intMax(intverts[basevert][1], intverts[othervert][1]);
				int loop0 = intMax(edge0 + 1, y0);
				int loop1 = intMin(edge1, y1_edge);

				unsigned char edgeBits = (edge << 4) | (othervert << 2) | basevert;
				for (int y = loop0; y <= loop1; y++)
				{
					int HitIndex = !!hfEdgeHits[y].Hits[0];
					hfEdgeHits[y].Hits[HitIndex] = edgeBits;
				}
			}
			// do the edge intersections with vertical planes
			if (intverts[basevert][0] != intverts[othervert][0])
			{
				int edge0 = intMin(intverts[basevert][0], intverts[othervert][0]);
				int edge1 = intMax(intverts[basevert][0], intverts[othervert][0]);
				int loop0 = intMax(edge0 + 1, x0);
				int loop1 = intMin(edge1, x1_edge);

				rcReal temppnt[3];
				rcReal cx = bmin[0] + cs * loop0;
				for (int x = loop0; x <= loop1; x++, cx += cs)
				{
					intersectX(vertarray[basevert], &edges[edge][0], cx, temppnt);
					int y = (int)rcFloor((temppnt[2] - bmin[2])*ics);
					if (y >= y0 && y <= y1)
					{
						rcReal sfloat = temppnt[1] - bmin[1];
						short int sint = (short int)rcClamp((int)rcFloor(sfloat * ich), -32000, 32000);
						addSpanSample(hf, x, y, sint);
						addSpanSample(hf, x - 1, y, sint);
					}
				}
			}
		}
		{
			// deal with the horizontal intersections 
			int edge0 = intMin(intverts[0][1], intMin(intverts[1][1],intverts[2][1]));
			int edge1 = intMax(intverts[0][1], intMax(intverts[1][1],intverts[2][1]));
			int loop0 = intMax(edge0 + 1, y0);
			int loop1 = intMin(edge1, y1_edge);

			rcReal Inter[2][3];
			int xInter[2];

			rcReal cz = bmin[2] + cs * loop0;
			for (int y = loop0; y <= loop1; y++, cz += cs)
			{
				rcEdgeHit& Hits = hfEdgeHits[y];
				if (Hits.Hits[0])
				{
					rcAssert(Hits.Hits[1]); // must have two hits

					for (int i = 0; i < 2; i++)
					{
						int edge = Hits.Hits[i] >> 4;
						int othervert = (Hits.Hits[i] >> 2) & 3;
						int basevert = Hits.Hits[i] & 3;

						intersectZ(vertarray[basevert], &edges[edge][0], cz, Inter[i]);
						int x = (int)rcFloor((Inter[i][0] - bmin[0])*ics);
						xInter[i] = x;
						if (x >= x0 && x <= x1)
						{
							rcReal sfloat = Inter[i][1] - bmin[1];
							short int sint = (short int)rcClamp((int)rcFloor(sfloat * ich), -32000, 32000);

							addSpanSample(hf, x, y, sint);
							addSpanSample(hf, x, y - 1, sint);
						}
					}
					if (xInter[0] != xInter[1])
					{
						// now fill in the fully contained ones.
						int left = Inter[1][0] < Inter[0][0];  
						int xloop0 = intMax(xInter[left] + 1, x0);
						int xloop1 = intMin(xInter[1 - left], x1_edge);

						rcReal d = 1.0f / (Inter[1-left][0] - Inter[left][0]);
						rcReal dy = Inter[1-left][1] - Inter[left][1];
						//rcReal ds = dy * d;
						rcReal ds = 0.0f;
						rcReal t = rcClamp((rcReal(xloop0)*cs + bmin[0] - Inter[left][0]) * d, 0.0f, 1.0f);
						rcReal sfloat = (Inter[left][1] + t * dy) - bmin[1];
						if (xloop1 - xloop0 > 0)
						{
							rcReal t2 = rcClamp((rcReal(xloop1)*cs + bmin[0] - Inter[left][0]) * d, 0.0f, 1.0f);
							rcReal sfloat2 = (Inter[left][1] + t2 * dy) - bmin[1];
							ds = (sfloat2 - sfloat) / rcReal(xloop1 - xloop0);
						}
						for (int x = xloop0; x <= xloop1; x++, sfloat += ds)
						{
							short int sint = (short int)rcClamp((int)rcFloor(sfloat * ich), -32000, 32000);

							addSpanSample(hf, x, y, sint);
							addSpanSample(hf, x - 1, y, sint);
							addSpanSample(hf, x, y - 1, sint);
							addSpanSample(hf, x - 1, y - 1, sint);
						}
					}
					// reset for next triangle
					Hits.Hits[0] = 0;
					Hits.Hits[1] = 0;
				}
			}
		}
		for (int y = y0; y <= y1; y++)
		{
			int xloop0 = intMax(hf.RowExt[y + 1].MinCol, x0);
			int xloop1 = intMin(hf.RowExt[y + 1].MaxCol, x1);
			for (int x = xloop0; x <= xloop1; x++)
			{
				int idx = SampleIndex(hf, x, y);
				rcTempSpan& Temp = hf.tempspans[idx];

				short int smin = Temp.sminmax[0];
				short int smax = Temp.sminmax[1];
				
				// reset for next triangle
				Temp.sminmax[0] = 32000;
				Temp.sminmax[1] = -32000;

				// Skip the span if it is outside the heightfield bbox
				if (smin >= RC_SPAN_MAX_HEIGHT || smax < 0) continue;

				smin = intMax(smin, 0);
				smax = intMin(intMax(smax,smin+1), RC_SPAN_MAX_HEIGHT);
				const int projectSpanToBottom = rasterizationMasks != nullptr ? (projectTriToBottom & rasterizationMasks[x+y*w]) : projectTriToBottom;
				if (projectSpanToBottom)
				{
					smin = 0;
				}

				if (!addSpan(hf, x, y, smin, smax, area, flagMergeThr))
				{
					return false;
				}
			}

			// reset for next triangle
			hf.RowExt[y + 1].MinCol = hf.width + 2;
			hf.RowExt[y + 1].MaxCol = -2;
		}
	}

    return true;
}

#else

///	Rasterize a single triangle to the heightfield.
///
///	This code is extremely hot, so much care should be given to maintaining maximum perf here.
/// 
/// @param[in] 	v0					Triangle vertex 0
/// @param[in] 	v1					Triangle vertex 1
/// @param[in] 	v2					Triangle vertex 2
/// @param[in] 	areaID				The area ID to assign to the rasterized spans
/// @param[in] 	hf					Heightfield to rasterize into
/// @param[in] 	hfBBMin				The min extents of the heightfield bounding box
/// @param[in] 	hfBBMax				The max extents of the heightfield bounding box
/// @param[in] 	cellSize			The x and z axis size of a voxel in the heightfield
/// @param[in] 	inverseCellSize		1 / cellSize
/// @param[in] 	inverseCellHeight	1 / cellHeight
/// @param[in] 	flagMergeThreshold	The threshold in which area flags will be merged 
/// @returns true if the operation completes successfully.  false if there was an error adding spans to the heightfield.
static bool rasterizeTri(const rcReal* v0, const rcReal* v1, const rcReal* v2,
						 const unsigned char areaID, rcHeightfield& hf,
						 const rcReal* hfBBMin, const rcReal* hfBBMax,
						 const rcReal cellSize, const rcReal inverseCellSize, const rcReal inverseCellHeight,
						 const int flagMergeThreshold,
						 const int rasterizationFlags,  //HG
						 const int* rasterizationMasks) //HG
{
	// Calculate the bounding box of the triangle.
	rcReal triBBMin[3];
	rcVcopy(triBBMin, v0);
	rcVmin(triBBMin, v1);
	rcVmin(triBBMin, v2);

	rcReal triBBMax[3];
	rcVcopy(triBBMax, v0);
	rcVmax(triBBMax, v1);
	rcVmax(triBBMax, v2);

	// If the triangle does not touch the bounding box of the heightfield, skip the triangle.
	if (!overlapBounds(triBBMin, triBBMax, hfBBMin, hfBBMax))
	{
		return true;
	}

	const int w = hf.width;
	const int h = hf.height;
	const rcReal by = hfBBMax[1] - hfBBMin[1];
	const int projectTriToBottom = rasterizationFlags & RC_PROJECT_TO_BOTTOM; //HG
	
	// Calculate the footprint of the triangle on the grid.
	int x0 = (int)((triBBMin[0] - hfBBMin[0]) * inverseCellSize);
	int z0 = (int)((triBBMin[2] - hfBBMin[2]) * inverseCellSize);
	int x1 = (int)((triBBMax[0] - hfBBMin[0]) * inverseCellSize);
	int z1 = (int)((triBBMax[2] - hfBBMin[2]) * inverseCellSize);
	x0 = rcClamp(x0, 0, w-1);
	z0 = rcClamp(z0, -1, h-1);
	x1 = rcClamp(x1, 0, w-1);
	z1 = rcClamp(z1, 0, h-1);
	
	// Clip the triangle into all grid cells it touches.
	rcReal in[7*3], out[7*3], inrow[7*3];
	
	for (int z = z0; z <= z1; ++z)
	{
		// Clip polygon to row.
		rcVcopy(&in[0], v0);
		rcVcopy(&in[1*3], v1);
		rcVcopy(&in[2*3], v2);
		int nvRow = 3;
		const rcReal cellZ = hfBBMin[2] + (float)z * cellSize;
		nvRow = clipPoly(in, nvRow, out, 0, 1, -cellZ);
		if (nvrow < 3)
		{
			continue;
		}
		nvrow = clipPoly(out, nvRow, inrow, 0, -1, cellZ+cellSize);
		if (nvRow < 3) 
		{
			continue;
		}
		if (z < 0)
		{
			continue;
		}
		
		for (int x = x0; x <= x1; ++x)
		{
			// Clip polygon to column.
			int nv = nvRow;
			const rcReal cx = hfBBMin[0] + x*cellSize;
			nv = clipPoly(inrow, nv, out, 1, 0, -cx);
			if (nv < 3) 
			{
				continue;
			}
			nv = clipPoly(out, nv, in, -1, 0, cx+cs);
			if (nv < 3) 
			{
				continue;
			}
			
			// Calculate min and max of the span.
			rcReal spanMin = in[1], spanMax = in[1];
			for (int vert = 1; vert < nv; ++vert)
			{
				spanMin = rcMin(spanMin, in[vert*3+1]);
				spanMax = rcMax(spanMax, in[vert*3+1]);
			}
			spanMin -= hfBBMin[1];
			spanMax -= hfBBMin[1];
			
			// Skip the span if it's completely outside the heightfield bounding box
			if (spanMax < 0.0f)
			{
				continue;
			}
			if (spanMin > by)
			{
				continue;
			}
			
			// Clamp the span to the heightfield bounding box.
			if (spanMin < 0.0f)
			{
				spanMin = 0;
			}
			if (spanMax > by)
			{
				spanMax = by;
			}
			
			// Snap the span to the heightfield height grid.
			unsigned short spanMinCellIndex = (unsigned short)rcClamp((int)rcFloor(spanMin * inverseCellHeight), 0, RC_SPAN_MAX_HEIGHT);
			unsigned short spanMaxCellIndex = (unsigned short)rcClamp((int)rcCeil(spanMax * inverseCellHeight), (int)spanMinCellIndex+1, RC_SPAN_MAX_HEIGHT);
			const int projectSpanToBottom = rasterizationMasks != nullptr ? (projectTriToBottom & rasterizationMasks[x+z*w]) : projectTriToBottom;	//HG
			if (projectSpanToBottom) //HG
			{
				spanMinCellIndex = 0; //HG
			}

			if (!addSpan(hf, x, z, spanMinCellIndex, spanMaxCellIndex, areaID, flagMergeThreshold))
			{
				return false;
			}
		}
	}
	return true;
}

#endif //HG_ADDITION_USE_NEW_RECAST_RASTERIZER

/// @par
///
/// No spans will be added if the triangle does not overlap the heightfield grid.
///
/// @see rcHeightfield
bool rcRasterizeTriangle(rcContext* context,
                         const rcReal* v0, const rcReal* v1, const rcReal* v2,
                         const unsigned char areaID, rcHeightfield& heightfield, const int flagMergeThreshold,
						 const int rasterizationFlags, const int* rasterizationMasks) //HG
{
	rcAssert(context != NULL);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);

	// Rasterize the single triangle.
	const rcReal inverseCellSize = 1.0f / heightfield.cs;
	const rcReal inverseCellHeight = 1.0f / heightfield.ch;
	if (!rasterizeTri(v0, v1, v2, areaID, heightfield, heightfield.bmin, heightfield.bmax, heightfield.cs, inverseCellSize, inverseCellHeight, flagMergeThreshold, rasterizationFlags, rasterizationMasks))
	{
		context->log(RC_LOG_ERROR, "rcRasterizeTriangle: Out of memory.");
		return false;
	}
	
	return true;
}

bool rcRasterizeTriangles(rcContext* context,
                          const rcReal* verts, const int /*nv*/,
                          const int* tris, const unsigned char* triAreaIDs, const int numTris,
                          rcHeightfield& heightfield, const int flagMergeThreshold, 
						  const int rasterizationFlags, const int* rasterizationMasks)
{
	rcAssert(context != NULL);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);
	
	// Rasterize the triangles.
	const rcReal inverseCellSize = 1.0f / heightfield.cs;
	const rcReal inverseCellHeight = 1.0f / heightfield.ch;
	for (int triIndex = 0; triIndex < numTris; ++triIndex)
	{
		const rcReal* v0 = &verts[tris[triIndex*3+0]*3];
		const rcReal* v1 = &verts[tris[triIndex*3+1]*3];
		const rcReal* v2 = &verts[tris[triIndex*3+2]*3];
		// Rasterize.
		if (!rasterizeTri(v0, v1, v2, triAreaIDs[triIndex], heightfield, heightfield.bmin, heightfield.bmax, heightfield.cs, inverseCellSize, inverseCellHeight, flagMergeThreshold, rasterizationFlags, rasterizationMasks))
		{
			context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
			return false;
		}
	}

	return true;
}

/// @par
///
/// Spans will only be added for triangles that overlap the heightfield grid.
///
/// @see rcHeightfield
bool rcRasterizeTriangles(rcContext* context, const rcReal* verts, const int /*nv*/,
						  const unsigned short* tris, const unsigned char* triAreaIDs, const int numTris,
						  rcHeightfield& heightfield, const int flagMergeThreshold, 
						  const int rasterizationFlags, const int* rasterizationMasks) //HG
{
	rcAssert(context != NULL);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);

	// Rasterize the triangles.
	const rcReal inverseCellSize = 1.0f / heightfield.cs;
	const rcReal inverseCellHeight = 1.0f / heightfield.ch;
	for (int triIndex = 0; triIndex < numTris; ++triIndex)
	{
		const rcReal* v0 = &verts[tris[triIndex*3+0]*3];
		const rcReal* v1 = &verts[tris[triIndex*3+1]*3];
		const rcReal* v2 = &verts[tris[triIndex*3+2]*3];
		// Rasterize.
		if (!rasterizeTri(v0, v1, v2, triAreaIDs[triIndex], heightfield, heightfield.bmin, heightfield.bmax, heightfield.cs, inverseCellSize, inverseCellHeight, flagMergeThreshold, rasterizationFlags, rasterizationMasks))
		{
			context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
			return false;
		}
	}

	return true;

}

/// @par
///
/// Spans will only be added for triangles that overlap the heightfield grid.
///
/// @see rcHeightfield
bool rcRasterizeTriangles(rcContext* context, 
						  const rcReal* verts, const unsigned char* triAreaIDs, const int numTris,
						  rcHeightfield& heightfield, const int flagMergeThreshold, 
						  const int rasterizationFlags, const int* rasterizationMasks) //HG
{
	rcAssert(context != NULL);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);
	
	// Rasterize the triangles.
	const rcReal inverseCellSize = 1.0f / heightfield.cs;
	const rcReal inverseCellHeight = 1.0f / heightfield.ch;
	for (int triIndex = 0; triIndex < numTris; ++triIndex)
	{
		const rcReal* v0 = &verts[(triIndex*3+0)*3];
		const rcReal* v1 = &verts[(triIndex*3+1)*3];
		const rcReal* v2 = &verts[(triIndex*3+2)*3];
		// Rasterize.
		if (!rasterizeTri(v0, v1, v2, triAreaIDs[triIndex], heightfield, heightfield.bmin, heightfield.bmax, heightfield.cs, inverseCellSize, inverseCellHeight, flagMergeThreshold, rasterizationFlags, rasterizationMasks))
		{
			context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
			return false;
		}
	}

	return true;
}
