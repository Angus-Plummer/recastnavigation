#pragma once

#include <limits>

#define DU_LARGE_WORLD_COORDINATES_ENABLED 0

#if DU_LARGE_WORLD_COORDINATES_ENABLED

typedef float duReal;

#else // DU_LARGE_WORLD_COORDINATES_ENABLED

typedef double duReal;

#endif // DU_LARGE_WORLD_COORDINATES_ENABLED