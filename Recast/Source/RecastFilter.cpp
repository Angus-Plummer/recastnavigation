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
#include "Recast.h"
#include "RecastAssert.h"
#include "RecastOptimisationToggle.h" //@HG
#include <stdlib.h>


/// @par
///
/// Allows the formation of walkable regions that will flow over low lying 
/// objects such as curbs, and up structures such as stairways. 
/// 
/// Two neighboring spans are walkable if: <tt>rcAbs(currentSpan.smax - neighborSpan.smax) < waklableClimb</tt>
/// 
/// @warning Will override the effect of #rcFilterLedgeSpans.  So if both filters are used, call
/// #rcFilterLedgeSpans after calling this filter. 
///
/// @see rcHeightfield, rcConfig
void rcFilterLowHangingWalkableObstacles(rcContext* context, const int walkableClimb, rcHeightfield& heightfield)
{
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_FILTER_LOW_OBSTACLES);

	const int xSize = heightfield.width;
	const int zSize = heightfield.height;

	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			rcSpan* previousSpan = NULL;
			bool previousWasWalkable = false;
			unsigned char previousArea = RC_NULL_AREA;

			for (rcSpan* span = heightfield.spans[x + z * xSize]; span != NULL; previousSpan = span, span = span->next)
			{
				const bool walkable = span->data.area != RC_NULL_AREA;
				// If current span is not walkable, but there is walkable
				// span just below it, mark the span above it walkable too.
				if (!walkable && previousWasWalkable)
				{
					if (rcAbs((int)span->data.smax - (int)previousSpan->data.smax) <= walkableClimb)
					{
						span->data.area = previousArea;
					}
				}
				// Copy walkable flag so that it cannot propagate
				// past multiple non-walkable objects.
				previousWasWalkable = walkable;
				previousArea = span->data.area;
			}
		}
	}
}

void rcFilterLedgeSpansImp(rcContext* context, const int walkableHeight, const int walkableClimb, const int filterLedgeSpansAtZ,
						   rcHeightfield& heightfield)
{
	rcAssert(context);

	const int xSize = heightfield.width;
	const int zSize = heightfield.height;
	const int MAX_HEIGHT = 0xffff; // TODO (graham): Move this to a more visible constant and update usages.
	
	// Mark border spans.
	for (int x = 0; x < xSize; ++x)
	{
		for (rcSpan* span = heightfield.spans[x + filterLedgeSpansAtZ * xSize]; span; span = span->next)
		{
			// Skip non walkable spans.
			if (span->data.area == RC_NULL_AREA)
			{
				continue;
			}

			const int bot = (int)(span->data.smax);
			const int top = span->next ? (int)(span->next->data.smin) : MAX_HEIGHT;

			// Find neighbours minimum height.
			int minNeighborHeight = MAX_HEIGHT;

			// Min and max height of accessible neighbours.
			int accessibleNeighborMinHeight = span->data.smax;
			int accessibleNeighborMaxHeight = span->data.smax;

			for (int direction = 0; direction < 4; ++direction)
			{
				int dx = x + rcGetDirOffsetX(direction);
				int dz = filterLedgeSpansAtZ + rcGetDirOffsetY(direction);
				// Skip neighbours which are out of bounds.
				if (dx < 0 || dz < 0 || dx >= xSize || dz >= zSize)
				{
					minNeighborHeight = rcMin(minNeighborHeight, -walkableClimb - bot);
					continue;
				}

				// From minus infinity to the first span.
				const rcSpan* neighborSpan = heightfield.spans[dx + dz*xSize];
				int neighborBot = -walkableClimb;
				int neighborTop = neighborSpan ? (int)neighborSpan->data.smin : MAX_HEIGHT;
				// Skip neightbour if the gap between the spans is too small.
				if (rcMin(top, neighborTop) - rcMax(bot, neighborBot) > walkableHeight)
				{
					minNeighborHeight = rcMin(minNeighborHeight, neighborBot - bot);
				}

				// Rest of the spans.
				for (neighborSpan = heightfield.spans[dx + dz * xSize]; neighborSpan; neighborSpan = neighborSpan->next)
				{
					neighborBot = (int)neighborSpan->data.smax;
					neighborTop = neighborSpan->next ? (int)neighborSpan->next->data.smin : MAX_HEIGHT;
					// Skip neightbour if the gap between the spans is too small.
					if (rcMin(top, neighborTop) - rcMax(bot, neighborBot) > walkableHeight)
					{
						minNeighborHeight = rcMin(minNeighborHeight, neighborBot - bot);

						// Find min/max accessible neighbour height. 
						if (rcAbs(neighborBot - bot) <= walkableClimb)
						{
							if (neighborBot < accessibleNeighborMinHeight) accessibleNeighborMinHeight = neighborBot;
							if (neighborBot > accessibleNeighborMaxHeight) accessibleNeighborMaxHeight = neighborBot;
						}

					}
				}
			}

			// The current span is close to a ledge if the drop to any
			// neighbour span is less than the walkableClimb.
			if (minNeighborHeight < -walkableClimb)
			{
				span->data.area = RC_NULL_AREA;
			}

			// If the difference between all neighbours is too large,
			// we are at steep slope, mark the span as ledge.
			else if ((accessibleNeighborMaxHeight - accessibleNeighborMinHeight) > walkableClimb)
			{
				span->data.area = RC_NULL_AREA;
			}
		}
	}
}

/// @par
///
/// A ledge is a span with one or more neighbors whose maximum is further away than @p walkableClimb
/// from the current span's maximum.
/// This method removes the impact of the overestimation of conservative voxelization 
/// so the resulting mesh will not have regions hanging in the air over ledges.
/// 
/// A span is a ledge if: <tt>rcAbs(currentSpan.smax - neighborSpan.smax) > walkableClimb</tt>
/// 
/// @see rcHeightfield, rcConfig
void rcFilterLedgeSpans(rcContext* context, const int walkableHeight, const int walkableClimb,
						rcHeightfield& heightfield)
{
	rcAssert(context);
	
	rcScopedTimer timer(context, RC_TIMER_FILTER_BORDER);

	const int w = heightfield.width;
	const int h = heightfield.height;
	const int MAX_HEIGHT = 0xffff;
	
	// Mark border spans.
	for (int z = 0; z < h; ++z)
	{
		rcFilterLedgeSpansImp(context, walkableHeight, walkableClimb, z, heightfield);
	}
}	


/// @see rcHeightfield, rcConfig
void rcFilterLedgeSpans(rcContext* context, const int walkableHeight, const int walkableClimb, const int zStart, const int maxZProcess,
					    rcHeightfield& heightfield)
{
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_FILTER_BORDER);

	const int w = heightfield.width;
	const int h = rcMin(zStart + maxZProcess, heightfield.height);

	for (int z = zStart; z < h; ++z)
	{
		rcFilterLedgeSpansImp(context, walkableHeight, walkableClimb, z, heightfield);
	}
}

/// @par
///
/// For this filter, the clearance above the span is the distance from the span's 
/// maximum to the next higher span's minimum. (Same grid column.)
/// 
/// @see rcHeightfield, rcConfig
void rcFilterWalkableLowHeightSpans(rcContext* context, int walkableHeight, rcHeightfield& heightfield)
{
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_FILTER_WALKABLE);
	
	const int xSize = heightfield.width;
	const int zSize = heightfield.height;
	const int MAX_HEIGHT = 0xffff;
	
	// Remove walkable flag from spans which do not have enough
	// space above them for the agent to stand there.
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			for (rcSpan* span = heightfield.spans[x + z*xSize]; span; span = span->next)
			{
				const int bot = (int)(span->data.smax);
				const int top = span->next ? (int)(span->next->data.smin) : MAX_HEIGHT;
				if ((top - bot) < walkableHeight)
				{
					span->data.area = RC_NULL_AREA;
				}
			}
		}
	}
	
}

void rcFilterWalkableLowHeightSpansSequences(rcContext* context, int walkableHeight, rcHeightfield& heightfield)
{
	rcAssert(context);
	
	rcScopedTimer timer(context, RC_TIMER_FILTER_WALKABLE);

	const int w = heightfield.width;
	const int h = heightfield.height;
	const int MAX_HEIGHT = 0xffff;

	const int MaxSpans = 64;
	rcCompactSpan SpanList[MaxSpans];
    int NumSpans;
	memset(SpanList, 0, sizeof(SpanList));

	// leave only single low span below valid one (null area doesn't count) or after leaving walkableHeight space between them

	// Remove walkable flag from spans which do not have enough
	// space above them for the agent to stand there.
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			// build compact span list, we need to iterate from top to bottom
			NumSpans = 0;
			for (rcSpan* span = heightfield.spans[x + y*w]; span; span = span->next)
			{
				const int bot = (int)span->data.smax;
				const int top = span->next ? (int)span->next->data.smin : MAX_HEIGHT;
				SpanList[NumSpans].y = (unsigned short)rcClamp(bot, 0, 0xffff);
				SpanList[NumSpans].h = (unsigned char)rcClamp(top - bot, 0, 0xff);
				SpanList[NumSpans].reg = span->data.area;
				
				NumSpans++;
				if (NumSpans >= MaxSpans)
				{
					break;
				}
			}

			int NextAllowedBase = 0xffff;
			for (int Idx = NumSpans - 1; Idx >= 0; Idx--)
			{
				if (SpanList[Idx].h < walkableHeight)
				{
					if (SpanList[Idx].y < NextAllowedBase)
					{
						NextAllowedBase = rcMax(0, SpanList[Idx].y - walkableHeight);
					}
					else
					{
						SpanList[Idx].reg = RC_NULL_AREA;
					}
				}
				else if (SpanList[Idx].reg != RC_NULL_AREA)
				{
					NextAllowedBase = SpanList[Idx].y;
				}
			}

            int SpanIdx = 0;
			for (rcSpan* span = heightfield.spans[x + y*w]; span; span = span->next)
			{
				if (SpanIdx < MaxSpans)
				{
					span->data.area = SpanList[SpanIdx].reg;
				}

				SpanIdx++;
			}
		}
	}
}
