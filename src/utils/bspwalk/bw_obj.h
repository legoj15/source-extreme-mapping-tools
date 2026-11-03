//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: -dumpobj. Writes <base>_raw.obj (source convexes, every category),
//          <base>_stand.obj and <base>_crouch.obj (grown polytopes).
//
//          One "o"/"g" pair per convex, named <category>_<n>[_<label>] where
//          n counts within the category, so a convex keeps its name across
//          the three files and Blender can filter by category prefix.
//          Coordinates are Source units, Z up. Blender's OBJ importer
//          defaults to Y up: import with Up Axis = Z, Forward Axis = Y to
//          keep map coordinates (scale 0.0254 gives metres at 1u = 1in).
//
//=============================================================================//

#ifndef BW_OBJ_H
#define BW_OBJ_H
#pragma once

#include "bw_world.h"

// Returns false if a file could not be written.
bool DumpWorldObj( const CollisionWorld &world, const char *pszBase );

#endif // BW_OBJ_H
