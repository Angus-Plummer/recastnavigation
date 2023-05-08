#pragma once

#include <limits>

#define RC_LARGE_WORLD_COORDINATES_ENABLED 0

#if RC_LARGE_WORLD_COORDINATES_ENABLED

typedef double rcReal;

#else // RC_LARGE_WORLD_COORDINATES_ENABLED

typedef float rcReal;

#endif // RC_LARGE_WORLD_COORDINATES_ENABLED

constexpr rcReal RC_REAL_MAX = std::numeric_limits<rcReal>::max();

