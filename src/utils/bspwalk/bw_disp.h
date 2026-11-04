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

void CollectDisplacements( CollisionWorld &world );

// The render triangles of every displacement (the same full-power grid), three
// points each, per displacement index, and each displacement's face (-1: none).
void DispRenderTriangles( std::vector< std::vector<dvec3> > &tris, std::vector<int> &dispFace );

#endif // BW_DISP_H
