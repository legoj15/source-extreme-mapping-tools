//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: the OptiX arc tracer (host side). Modelled on VVIS's former vvis_optix.cpp (removed, #46), with
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

// #35 sweep (__raygen__sweep in bspwalk_arcs.ptx, after BwOptix_Init and the
// BW_MESH_VIS mesh). SweepEyes uploads one block of eye samples, replacing the last
// block (fill(ctx, first, count, out) writes the xyz of samples [first, first + count));
// false (no block kept) when the device cannot hold them. SweepSamples sets a slot's
// confirmation samples (p xyz, n xyz in double each; any slot >= 0, one per face of the
// sweep); the slots survive a new block. SweepTrace launches (rays x n) pairs of the
// block's eyes idx[0, n), eye idx[i] taking rank[i] as its place in the hash
// (BwVis_ConfirmSample's 'eye'), with that slot's samples, as BwArcLaunchParams
// describes; clear is true when one is clear. Calls are not thread safe: the caller
// serializes them. With pairs (#114), every clear pair (i * rays + r) is listed in
// *pairs when clear, in no particular order; *pOverflow is true (and *pairs empty) when
// more than BW_SWEEP_OUT_CAP were clear, the list then incomplete. SweepFree frees the
// block and every slot.
#define BW_SWEEP_OUT_CAP	( 1u << 22 )
bool BwOptix_SweepEyes( size_t n, void ( *fill )( void *, size_t, size_t, float * ), void *ctx );
bool BwOptix_SweepSamples( int slot, const double *samples, unsigned int n );
bool BwOptix_SweepTrace( int slot, const unsigned int *idx, const unsigned long long *rank, unsigned int n, unsigned int rays,
						 bool twoSided, unsigned long long key, bool &clear, std::vector<unsigned long long> *pairs = NULL,
						 bool *pOverflow = NULL );
void BwOptix_SweepFree();

// #14 eye fan (bspwalk_fan.ptx, BwFanLaunchParams). Setup loads the fan
// pipeline (after BwOptix_Init) and uploads: the occluder mesh, the face slab
// mesh with its triangle -> face ids, per face the normal and a one-sided flag
// (4 floats), the zero-area face plates (BwFanLaunchParams::plates), the fan
// directions and every fan eye (xyz each), and the per face marks to start
// from (nonzero: never marked again), and the PVS gate (BwFanPvs). False (fan data freed)
// on any failure.
struct BwFanPvs	// BwFanLaunchParams' PVS gate: eyeCl per fan eye, faceClOfs per face + 1
{
	std::vector<int> eyeCl, faceClOfs, faceCl;
	std::vector<unsigned char> rows;
	unsigned int rowBytes;
};
// The one-way occluders (#77, bw_vis.cpp FanOneWayMesh) for BwFanLaunchParams::dirGas: their
// hulls as one triangle mesh with the occluder of each triangle (prim), and per occluder its
// real planes (n xyz, d: 4 floats each, from ofs[o] to ofs[o + 1]) and per plane drawn.
struct BwFanOneWay
{
	std::vector<float> verts, planes;
	std::vector<unsigned int> tris, prim, ofs;
	std::vector<unsigned char> drawn;
};
bool BwOptix_FanSetup( const std::vector<float> &occVerts, const std::vector<unsigned int> &occTris,
					   const std::vector<float> &faceVerts, const std::vector<unsigned int> &faceTris,
					   const std::vector<unsigned int> &primFace, const std::vector<float> &faceN,
					   const std::vector<float> &plates, unsigned int numPlates, const std::vector<float> &dirs,
					   const std::vector<float> &eyes, const std::vector<unsigned char> &visible, const BwFanPvs &pvs,
					   const BwFanOneWay &dir );
// One launch: eyes [first, first + count) x every direction. tOut, when not
// null, gets each ray's occluder distance (count x directions, eye-major).
// 'pad' is the plates' margin; dirSlop the one-way occluders' (BwFanLaunchParams).
bool BwOptix_FanTrace( unsigned int first, unsigned int count, float maxT, float past, float facingTol, float pad,
					   float dirSlop, float *tOut );
// The per face marks so far.
bool BwOptix_FanVisible( unsigned char *visible );
void BwOptix_FanFree();
void BwOptix_Shutdown();

#endif // BW_OPTIX_H
