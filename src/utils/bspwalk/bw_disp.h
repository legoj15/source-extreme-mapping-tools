//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: displacement collision triangles. Each triangle of the full-power
//          grid (what the engine's CDispCollTree traces against) becomes one
//          flat convex.
//
//=============================================================================//

#ifndef BW_DISP_H
#define BW_DISP_H
#pragma once

#include "bw_world.h"

// Adds every displacement that collides with the player hull as one convex per
// triangle, and fills world.dispRenderTris with the render triangles of every
// displacement (the same full-power grid).
void CollectDisplacements( CollisionWorld &world );

#endif // BW_DISP_H
