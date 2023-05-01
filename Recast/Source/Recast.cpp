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

#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"
#include "RecastOptimisationToggle.h" //@HG
#include "RecastSharedUtilFuncs.h" //@HG

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

namespace
{
/// Allocates and constructs an object of the given type, returning a pointer.
/// @param[in]		allocLifetime	Allocation lifetime hint
template<typename T>
T* rcNew(const rcAllocHint allocLifetime)
{
	T* ptr = (T*)rcAlloc(sizeof(T), allocLifetime);
	::new(rcNewTag(), (void*)ptr) T();
	return ptr;
}

/// Destroys and frees an object allocated with rcNew.
/// @param[in]     ptr    The object pointer to delete.
template<typename T>
void rcDelete(T* ptr)
{
	if (ptr)
	{
		ptr->~T();
		rcFree((void*)ptr);
	}
}
} // anonymous namespace

float rcSqrt(float x)
{
	return sqrtf(x);
}

//@HG BEGIN Adding support for double precision
double rcSqrt(double x)
{
	return sqrt(x);
}
//@HG END

/// @class rcContext
/// @par
///
/// This class does not provide logging or timer functionality on its 
/// own.  Both must be provided by a concrete implementation 
/// by overriding the protected member functions.  Also, this class does not 
/// provide an interface for extracting log messages. (Only adding them.) 
/// So concrete implementations must provide one.
///
/// If no logging or timers are required, just pass an instance of this 
/// class through the Recast build process.
///

/// @par
///
/// Example:
/// @code
/// // Where ctx is an instance of rcContext and filepath is a char array.
/// ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not load '%s'", filepath);
/// @endcode
void rcContext::log(const rcLogCategory category, const char* format, ...)
{
	if (!m_logEnabled)
	{
		return;
	}
	static const int MSG_SIZE = 512;
	char msg[MSG_SIZE];
	va_list argList;
	va_start(argList, format);
	int len = vsnprintf(msg, MSG_SIZE, format, argList);
	if (len >= MSG_SIZE)
	{
		len = MSG_SIZE - 1;
		msg[MSG_SIZE - 1] = '\0';

		const char* errorMessage = "Log message was truncated";
		doLog(RC_LOG_ERROR, errorMessage, (int)strlen(errorMessage));
	}
	va_end(argList);
	doLog(category, msg, len);
}

void rcContext::doResetLog()
{
	// Defined out of line to fix the weak v-tables warning
}

rcAreaModification::rcAreaModification(unsigned char value) :
	m_value(value),
	m_mask(RC_AREA_FLAGS_MASK)
{
}

rcAreaModification::rcAreaModification(unsigned char value, unsigned char mask) :
	m_value(value),
	m_mask(mask)
{
}

rcAreaModification::rcAreaModification(const rcAreaModification& other) :
	m_value(other.m_value),
	m_mask(other.m_mask)
{
}

void rcAreaModification::operator = (const rcAreaModification& other)
{
	m_value = other.m_value;
	m_mask = other.m_mask;
}

bool rcAreaModification::operator == (const rcAreaModification& other) const
{
	return ((m_value == other.m_value) && (m_mask == other.m_mask));
}

bool rcAreaModification::operator != (const rcAreaModification& other) const
{
	return ((m_value == other.m_value) && (m_mask == other.m_mask));
}

void rcAreaModification::apply(unsigned char& area) const
{
	area = ((m_value & m_mask) | (area & ~m_mask));
}

unsigned char rcAreaModification::getMaskedValue() const
{
	return (m_value & m_mask);
}

rcHeightfield* rcAllocHeightfield()
{
	return rcNew<rcHeightfield>(RC_ALLOC_PERM);
}

void rcFreeHeightField(rcHeightfield* heightfield)
{
	rcDelete(heightfield);
}

rcHeightfield::rcHeightfield()
: width()
, height()
, bmin()
, bmax()
, cs()
, ch()
, spans()
, pools()
, freelist()
#if HG_ADDITION_USE_NEW_RECAST_RASTERIZER
, EdgeHits()
, RowExt()
, tempspans()
#endif
{
}

rcHeightfield::~rcHeightfield()
{
	// Delete span array.
	rcFree(spans);
	// Delete span pools.
	while (pools)
	{
		rcSpanPool* next = pools->next;
		rcFree(pools);
		pools = next;
	}
#if HG_ADDITION_USE_NEW_RECAST_RASTERIZER
	rcFree(EdgeHits);
	rcFree(RowExt);
	rcFree(tempspans);
#endif
}

rcCompactHeightfield* rcAllocCompactHeightfield()
{
	return rcNew<rcCompactHeightfield>(RC_ALLOC_PERM);
}

void rcFreeCompactHeightfield(rcCompactHeightfield* compactHeightfield)
{
	rcDelete(compactHeightfield);
}

rcCompactHeightfield::rcCompactHeightfield()
: width()
, height()
, spanCount()
, maxCellSpanCount() //@HG - track max cell span count for safer mem allocations
, walkableHeight()
, walkableClimb()
, borderSize()
, maxDistance()
, maxRegions()
, bmin()
, bmax()
, cs()
, ch()
, cells()
, spans()
, dist()
, areas()
{
}

rcCompactHeightfield::~rcCompactHeightfield()
{
	rcFree(cells);
	rcFree(spans);
	rcFree(dist);
	rcFree(areas);
}


rcHeightfieldLayerSet* rcAllocHeightfieldLayerSet()
{
	return rcNew<rcHeightfieldLayerSet>(RC_ALLOC_PERM);
}

void rcFreeHeightfieldLayerSet(rcHeightfieldLayerSet* layerSet)
{
	rcDelete(layerSet);
}

rcHeightfieldLayerSet::rcHeightfieldLayerSet()
: layers()
, nlayers()
{
}

rcHeightfieldLayerSet::~rcHeightfieldLayerSet()
{
	for (int i = 0; i < nlayers; ++i)
	{
		rcFree(layers[i].heights);
		rcFree(layers[i].areas);
		rcFree(layers[i].cons);
	}
	rcFree(layers);
}


rcContourSet* rcAllocContourSet()
{
	return rcNew<rcContourSet>(RC_ALLOC_PERM);
}

void rcFreeContourSet(rcContourSet* contourSet)
{
	rcDelete(contourSet);
}

rcContourSet::rcContourSet()
: conts()
, nconts()
, bmin()
, bmax()
, cs()
, ch()
, width()
, height()
, borderSize()
, maxError()
{
}

rcContourSet::~rcContourSet()
{
	for (int i = 0; i < nconts; ++i)
	{
		rcFree(conts[i].verts);
		rcFree(conts[i].rverts);
	}
	rcFree(conts);
}

//@HG BEGIN
#if WITH_NAVMESH_CLUSTER_LINKS
rcClusterSet* rcAllocClusterSet()
{
	return rcNew<rcClusterSet>(RC_ALLOC_PERM);
}

void rcFreeClusterSet(rcClusterSet* clusters)
{
	rcDelete(clusters);
}

rcClusterSet::rcClusterSet()
    : nclusters()
    , center()
    , nlinks()
    , links()
{}

rcClusterSet::~rcClusterSet()
{
    rcFree(center);
    rcFree(links);
    rcFree(nlinks);
}

#endif // WITH_NAVMESH_CLUSTER_LINKS
//@HG END

rcPolyMesh* rcAllocPolyMesh()
{
	return rcNew<rcPolyMesh>(RC_ALLOC_PERM);
}

void rcFreePolyMesh(rcPolyMesh* polyMesh)
{
	rcDelete(polyMesh);
}

rcPolyMesh::rcPolyMesh()
: verts()
, polys()
, regs()
, flags()
, areas()
, nverts()
, npolys()
, maxpolys()
, nvp()
, bmin()
, bmax()
, cs()
, ch()
, borderSize()
, maxEdgeError()
{
}

rcPolyMesh::~rcPolyMesh()
{
	rcFree(verts);
	rcFree(polys);
	rcFree(regs);
	rcFree(flags);
	rcFree(areas);
}

rcPolyMeshDetail* rcAllocPolyMeshDetail()
{
	return rcNew<rcPolyMeshDetail>(RC_ALLOC_PERM);
}

void rcFreePolyMeshDetail(rcPolyMeshDetail* detailMesh)
{
	rcDelete(detailMesh);
}

rcPolyMeshDetail::rcPolyMeshDetail()
: meshes()
, verts()
, tris()
, nmeshes()
, nverts()
, ntris()
{
}

rcPolyMeshDetail::~rcPolyMeshDetail()
{
	rcFree(meshes);
	rcFree(verts);
	rcFree(tris);
}

void rcCalcBounds(const rcReal* verts, int numVerts, rcReal* minBounds, rcReal* maxBounds)
{
	// Calculate bounding box.
	rcVcopy(minBounds, verts);
	rcVcopy(maxBounds, verts);
	for (int i = 1; i < numVerts; ++i)
	{
		const rcReal* v = &verts[i * 3];
		rcVmin(minBounds, v);
		rcVmax(maxBounds, v);
	}
}

void rcCalcGridSize(const rcReal* minBounds, const rcReal* maxBounds, const rcReal cellSize, int* sizeX, int* sizeZ)
{
	*sizeX = (int)((maxBounds[0] - minBounds[0]) / cellSize + 0.5f);
	*sizeZ = (int)((maxBounds[2] - minBounds[2]) / cellSize + 0.5f);
}

bool rcCreateHeightfield(rcContext* context, rcHeightfield& heightfield, int sizeX, int sizeZ,
                         const rcReal* minBounds, const rcReal* maxBounds,
                         rcReal cellSize, rcReal cellHeight)
{
//	rcAssert(ctx);
	
	heightfield.width = sizeX;
	heightfield.height = sizeZ;
	rcVcopy(heightfield.bmin, minBounds);
	rcVcopy(heightfield.bmax, maxBounds);
	heightfield.cs = cellSize;
	heightfield.ch = cellHeight;
	heightfield.spans = (rcSpan**)rcAlloc(sizeof(rcSpan*) * heightfield.width * heightfield.height, RC_ALLOC_PERM);
	if (!heightfield.spans)
	{
		return false;
	}
	memset(heightfield.spans, 0, sizeof(rcSpan*) * heightfield.width * heightfield.height);

#if HG_ADDITION_USE_NEW_RECAST_RASTERIZER
	heightfield.EdgeHits = (rcEdgeHit*)rcAlloc(sizeof(rcEdgeHit) * (heightfield.height + 1), RC_ALLOC_PERM); 
	if (!heightfield.EdgeHits)
		return false;
	memset(heightfield.EdgeHits, 0, sizeof(rcEdgeHit) * (heightfield.height + 1));

	heightfield.RowExt = (rcRowExt*)rcAlloc(sizeof(rcRowExt) * (heightfield.height + 2), RC_ALLOC_PERM); 

	for (int i = 0; i < heightfield.height + 2; i++)
	{
		heightfield.RowExt[i].MinCol = heightfield.width + 2;
		heightfield.RowExt[i].MaxCol = -2;
	}

	heightfield.tempspans = (rcTempSpan*)rcAlloc(sizeof(rcTempSpan)*(heightfield.width + 2) * (heightfield.height + 2), RC_ALLOC_PERM); 
	if (!heightfield.tempspans)
		return false;

	for (int i = 0; i < heightfield.height + 2; i++)
	{
		for (int j = 0; j < heightfield.width + 2; j++)
		{
			heightfield.tempspans[i * (heightfield.width + 2) + j].sminmax[0] = 32000;
			heightfield.tempspans[i * (heightfield.width + 2) + j].sminmax[1] = -32000;
		}
	}
#endif

	return true;
}

void rcResetHeightfield(rcHeightfield& hf)
{
	// reset all spans in allocated pools
	hf.freelist = 0;
	for (rcSpanPool* ipool = hf.pools; ipool; ipool = ipool->next)
	{
		rcSpan* freelist = hf.freelist;
		rcSpan* head = &ipool->items[0];
		rcSpan* it = &ipool->items[RC_SPANS_PER_POOL];
		do
		{
			--it;
			it->next = freelist;
			freelist = it;
		}
		while (it != head);
		hf.freelist = it;
	}

	// reset grid
	memset(hf.spans, 0, sizeof(rcSpan*)*hf.width*hf.height);
}

static void calcTriNormal(const rcReal* v0, const rcReal* v1, const rcReal* v2, rcReal* faceNormal)
{
	rcReal e0[3], e1[3];
	rcVsub(e0, v1, v0);
	rcVsub(e1, v2, v0);
	rcVcross(faceNormal, e0, e1);
	rcVnormalize(faceNormal);
}

/// @par
///
/// Only sets the aread id's for the walkable triangles.  Does not alter the
/// area id's for unwalkable triangles.
/// 
/// See the #rcConfig documentation for more information on the configuration parameters.
/// 
/// @see rcHeightfield, rcClearUnwalkableTriangles, rcRasterizeTriangles
void rcMarkWalkableTriangles(rcContext* /*ctx*/, const rcReal walkableSlopeAngle,
							 const rcReal* verts, int /*nv*/,
							 const int* tris, const int numTris, 
                             unsigned char* triAreaIDs, rcAreaModification areaMod )
{
	// TODO: VC complains about unref formal variable, figure out a way to handle this better.
//	rcAssert(ctx);
	
	const rcReal walkableThr = rcCos(walkableSlopeAngle/180.0f*RC_PI);
	rcMarkWalkableTrianglesCos(0, walkableThr, verts, 0, tris, numTris, triAreaIDs, areaMod);
}

void rcMarkWalkableTrianglesCos(rcContext* /*ctx*/, const rcReal walkableSlopeCos,
								const rcReal* verts, int /*nv*/,
								const int* tris, int numTris, 
                                unsigned char* triAreaIDs, rcAreaModification areaMod)
{
	rcReal norm[3];
	for (int i = 0; i < numTris; ++i)
	{
		const int* tri = &tris[i*3];
		calcTriNormal(&verts[tri[0]*3], &verts[tri[1]*3], &verts[tri[2]*3], norm);
		// Check if the face is walkable.
		if (norm[1] > walkableSlopeCos)
		{
			areaMod.apply(triAreaIDs[i]);
		}
	}
}

/// @par
///
/// Only sets the aread id's for the unwalkable triangles.  Does not alter the
/// area id's for walkable triangles.
/// 
/// See the #rcConfig documentation for more information on the configuration parameters.
/// 
/// @see rcHeightfield, rcClearUnwalkableTriangles, rcRasterizeTriangles
void rcClearUnwalkableTriangles(rcContext* /*ctx*/, const rcReal walkableSlopeAngle,
								const rcReal* verts, int /*numVerts*/,
								const int* tris, int numTris,
								unsigned char* triAreaIDs)
{
	// The minimum Y value for a face normal of a triangle with a walkable slope.
//	rcAssert(ctx);
	
	const rcReal walkableLimitY = rcCos(walkableSlopeAngle/180.0f*RC_PI);
	
	rcReal faceNormal[3];
	
	for (int i = 0; i < numTris; ++i)
	{
		const int* tri = &tris[i * 3];
		calcTriNormal(&verts[tri[0] * 3], &verts[tri[1] * 3], &verts[tri[2] * 3], faceNormal);
		// Check if the face is walkable.
		if (faceNormal[1] <= walkableLimitY)
		{
			triAreaIDs[i] = RC_NULL_AREA;
		}
	}
}

int rcGetHeightFieldSpanCount(rcContext* /*ctx*/, rcHeightfield& heightfield)
{
	//rcIgnoreUnused(context);

	const int numCols = heightfield.width * heightfield.height;
	int spanCount = 0;
	for (int columnIndex = 0; columnIndex < numCols; ++columnIndex)
	{
		for (rcSpan* span = heightfield.spans[columnIndex]; span != NULL; span = span->next)
		{
			if (span->data.area != RC_NULL_AREA)
			{
				spanCount++;
			}
		}
	}
	return spanCount;
}

/// @par
///
/// This is just the beginning of the process of fully building a compact heightfield.
/// Various filters may be applied applied, then the distance field and regions built.
/// E.g: #rcBuildDistanceField and #rcBuildRegions
///
/// See the #rcConfig documentation for more information on the configuration parameters.
///
/// @see rcAllocCompactHeightfield, rcHeightfield, rcCompactHeightfield, rcConfig
bool rcBuildCompactHeightfield(rcContext* context, const int walkableHeight, const int walkableClimb,
							   rcHeightfield& heightfield, rcCompactHeightfield& compactHeightfield)
{
	rcAssert(context);
	
// @HG BEGIN: early-out when no walkable spans 
	const int spanCount = rcGetHeightFieldSpanCount(context, heightfield);
	if (spanCount == 0)
	{
		// no spans to speak of, bail out.
		return false;
	}
// @HG END
	rcScopedTimer timer(context, RC_TIMER_BUILD_COMPACTHEIGHTFIELD);

	const int xSize = heightfield.width;
	const int zSize = heightfield.height;

	// Fill in header.
	compactHeightfield.width = xSize;
	compactHeightfield.height = zSize;
	compactHeightfield.spanCount = spanCount;
	compactHeightfield.walkableHeight = walkableHeight;
	compactHeightfield.walkableClimb = walkableClimb;
	compactHeightfield.maxRegions = 0;
	rcVcopy(compactHeightfield.bmin, heightfield.bmin);
	rcVcopy(compactHeightfield.bmax, heightfield.bmax);
	compactHeightfield.bmax[1] += walkableHeight * heightfield.ch;
	compactHeightfield.cs = heightfield.cs;
	compactHeightfield.ch = heightfield.ch;
	compactHeightfield.cells = (rcCompactCell*)rcAlloc(sizeof(rcCompactCell) * xSize * zSize, RC_ALLOC_PERM);
	if (!compactHeightfield.cells)
	{
		context->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Out of memory 'chf.cells' (%d)", xSize * zSize);
		return false;
	}
	memset(compactHeightfield.cells, 0, sizeof(rcCompactCell) * xSize * zSize);
	compactHeightfield.spans = (rcCompactSpan*)rcAlloc(sizeof(rcCompactSpan) * spanCount, RC_ALLOC_PERM);
	if (!compactHeightfield.spans)
	{
		context->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Out of memory 'chf.spans' (%d)", spanCount);
		return false;
	}
	memset(compactHeightfield.spans, 0, sizeof(rcCompactSpan) * spanCount);
	compactHeightfield.areas = (unsigned char*)rcAlloc(sizeof(unsigned char) * spanCount, RC_ALLOC_PERM);
	if (!compactHeightfield.areas)
	{
		context->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Out of memory 'chf.areas' (%d)", spanCount);
		return false;
	}
	memset(compactHeightfield.areas, RC_NULL_AREA, sizeof(unsigned char) * spanCount);

	const int MAX_HEIGHT = 0xffff;

	compactHeightfield.maxCellSpanCount = 0; //@HG - track max cell span count for safer mem allocations

	// Fill in cells and spans.
	int currentCellIndex = 0;
	const int numColumns = xSize * zSize;
	for (int columnIndex = 0; columnIndex < numColumns; ++columnIndex)
	{
		const rcSpan* span = heightfield.spans[columnIndex];
			
		// If there are no spans at this cell, just leave the data to index=0, count=0.
		if (span == NULL)
		{
			continue;
		}
			
		rcCompactCell& cell = compactHeightfield.cells[columnIndex];
		cell.index = currentCellIndex;
		cell.count = 0;

		for (; span != NULL; span = span->next)
		{
			if (span->data.area != RC_NULL_AREA)
			{
				const int bot = (int)span->data.smax;
				const int top = span->next ? (int)span->next->data.smin : MAX_HEIGHT;
				compactHeightfield.spans[currentCellIndex].y = (unsigned short)rcClamp(bot, 0, 0xffff);
				compactHeightfield.spans[currentCellIndex].h = (unsigned char)rcClamp(top - bot, 0, 0xff);
				compactHeightfield.areas[currentCellIndex] = span->data.area;
				currentCellIndex++;
				cell.count++;
			}
		}

		//@HG - track max cell span count for safer mem allocations
		compactHeightfield.maxCellSpanCount = rcMax( compactHeightfield.maxCellSpanCount, cell.count );
	}
	
	// Find neighbour connections.
	const int MAX_LAYERS = RC_NOT_CONNECTED - 1;
	int maxLayerIndex = 0;
	const int zStride = xSize; // for readability
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			const rcCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			for (int i = (int)cell.index, ni = (int)(cell.index + cell.count); i < ni; ++i)
			{
				rcCompactSpan& span = compactHeightfield.spans[i];

				for (int dir = 0; dir < 4; ++dir)
				{
					rcSetCon(span, dir, RC_NOT_CONNECTED);
					const int neighborX = x + rcGetDirOffsetX(dir);
					const int neighborZ = z + rcGetDirOffsetY(dir);
					// First check that the neighbour cell is in bounds.
					if (neighborX < 0 || neighborZ < 0 || neighborX >= xSize || neighborZ >= zSize)
					{
						continue;
					}

					// Iterate over all neighbour spans and check if any of the is
					// accessible from current cell.
					const rcCompactCell& neighborCell = compactHeightfield.cells[neighborX + neighborZ * zStride];
					for (int k = (int)neighborCell.index, nk = (int)(neighborCell.index + neighborCell.count); k < nk; ++k)
					{
						const rcCompactSpan& neighborSpan = compactHeightfield.spans[k];
						const int bot = rcMax(span.y, neighborSpan.y);
						const int top = rcMin(span.y + span.h, neighborSpan.y + neighborSpan.h);

						// Check that the gap between the spans is walkable,
						// and that the climb height between the gaps is not too high.
						if ((top - bot) >= walkableHeight && rcAbs((int)neighborSpan.y - (int)span.y) <= walkableClimb)
						{
							// Mark direction as walkable.
							const int layerIndex = k - (int)neighborCell.index;
							if (layerIndex < 0 || layerIndex > MAX_LAYERS)
							{
								maxLayerIndex = rcMax(maxLayerIndex, layerIndex);
								continue;
							}
							rcSetCon(span, dir, layerIndex);
							break;
						}
					}
				}
			}
		}
	}

	if (maxLayerIndex > MAX_LAYERS)
	{
		context->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Heightfield has too many layers %d (max: %d)",
		         maxLayerIndex, MAX_LAYERS);
	}

	return true;
}