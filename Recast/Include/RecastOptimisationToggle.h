////////////////////////////////////////////////////////////////////////////////
///
///     @file       RecastOptimisationToggle.h
///     @author     Angus
///     @date       Apr 2023
///
///     @brief      RecastOptimisationToggle
///
///     Copyright (c) 2023 Hello Games Ltd. All Rights Reserved.
///
////////////////////////////////////////////////////////////////////////////////

// this is in its own .h so that we can edit it without triggering large build from Recast.h edit

#pragma once

#define RC_ALLOW_TOGGLE_OPTIMIZATION 0
#if RC_ALLOW_TOGGLE_OPTIMIZATION
#define RC_DISABLE_OPTIMIZATION __pragma(optimize( "", off ))
#define RC_ENABLE_OPTIMIZATION __pragma(optimize( "", on ))
#define RC_DISABLE_INLINING	__pragma(inline_depth(0))
#else
#define RC_DISABLE_OPTIMIZATION
#define RC_ENABLE_OPTIMIZATION
#define RC_DISABLE_INLINING
#endif

// disable all
//RC_DISABLE_OPTIMIZATION