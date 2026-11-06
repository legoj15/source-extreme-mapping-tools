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

// #14 eye fan (bspwalk_fan.ptx, BwFanLaunchParams). Setup loads the fan
// pipeline (after BwOptix_Init) and uploads: the occluder mesh, the face slab
// mesh with its triangle -> face ids, per face the normal and a one-sided flag
// (4 floats), the zero-area face plates (BwFanLaunchParams::plates), the fan
// directions and every fan eye (xyz each), and the per face marks to start
// from (nonzero: never marked again). False (fan data freed) on any failure.
bool BwOptix_FanSetup( const std::vector<float> &occVerts, const std::vector<unsigned int> &occTris,
					   const std::vector<float> &faceVerts, const std::vector<unsigned int> &faceTris,
					   const std::vector<unsigned int> &primFace, const std::vector<float> &faceN,
					   const std::vector<float> &plates, unsigned int numPlates, const std::vector<float> &dirs,
					   const std::vector<float> &eyes, const std::vector<unsigned char> &visible );
// One launch: eyes [first, first + count) x every direction. tOut, when not
// null, gets each ray's occluder distance (count x directions, eye-major).
// 'pad' is the plates' margin.
bool BwOptix_FanTrace( unsigned int first, unsigned int count, float maxT, float past, float facingTol, float pad,
					   float *tOut );
// The per face marks so far.
bool BwOptix_FanVisible( unsigned char *visible );
void BwOptix_FanFree();
void BwOptix_Shutdown();

#endif // BW_OPTIX_H
