//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: the OptiX arc tracer (host side). Modelled on vvis_optix.cpp, with
//          its known defects left out: no dead any-hit program, the GAS is
//          compacted, a failed init frees everything it made, and the PTX is
//          read only from the exe's own directory.
//
//=============================================================================//

#ifndef BW_OPTIX_H
#define BW_OPTIX_H
#pragma once

#include <vector>
#include "bw_arc_types.h"

// CUDA device + OptiX context + bspwalk_arcs.ptx + pipeline. False (message
// printed, nothing left allocated) when any part is missing.
bool BwOptix_Init();
const char *BwOptix_DeviceName();
// Builds (replaces) the triangle GAS of one mesh: xyz float triples, index triples.
bool BwOptix_BuildMesh( int mesh, const std::vector<float> &verts, const std::vector<unsigned int> &tris );
size_t BwOptix_MeshBytes( int mesh );
// One launch (one thread per arc) over arcs[0, n). blocked[i] = 1 on a hit.
bool BwOptix_TraceArcs( const BwArc *arcs, unsigned int n, float gravity, unsigned char *blocked );
void BwOptix_Shutdown();

#endif // BW_OPTIX_H
