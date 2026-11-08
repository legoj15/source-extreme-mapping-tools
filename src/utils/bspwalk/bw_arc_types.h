//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: plain types shared by the host (MSVC) and the OptiX kernels
//          (bw_arcs.cu, bw_fan.cu; nvcc). No includes, no C++ beyond POD structs.
//
//=============================================================================//

#ifndef BW_ARC_TYPES_H
#define BW_ARC_TYPES_H

#define BW_MESH_STAND	0
#define BW_MESH_CROUCH	1
#define BW_MESH_VIS		2	// #14 tier B: the shrunk opaque solid brushes (sight occluders)
#define BW_MESH_COUNT	3

// Consecutive chords overlap by this much (units) at each interior joint, so a
// face crossed exactly at a joint is strictly inside one of the two rays.
#define BW_ARC_JOINT_OVERLAP	( 1.0f / 64.0f )

// One ballistic feet-point arc, in float for the GPU. Before tKick the height
// is oz + vz t - g0 t^2 / 2 with g0 = gravity + dg0 (dg0 is 0 except inside a
// trigger_push, where g0 = gravity - push z speed may be negative); at tKick
// (an air jump or leaving a push, 0 = none) the vertical speed is reset to
// vzKick and gravity is the plain one. XY moves at (vx, vy) throughout. The arc
// is traced as nSeg[0] equal chords over [0, tKick] (or [0, tEnd] without a
// kick) and nSeg[1] over [tKick, tEnd]; the host picks the counts so every
// chord stays within the sagitta bound of the parabola. A straight segment a -> b
// is the arc o = a, v = b - a, dg0 = -gravity, tEnd = 1, nSeg = { 1, 0 } (tier B
// visibility rays, bw_vis.cpp).
struct BwArc
{
	float ox, oy, oz;
	float vx, vy, vz;
	float tKick, vzKick;
	float dg0;
	float tEnd;
	unsigned short nSeg[2];
	unsigned int mesh;		// BW_MESH_*
};

struct BwArcLaunchParams
{
	const BwArc *arcs;
	unsigned char *blocked;			// out: 1 when the arc hits its mesh
	unsigned int numArcs;
	float gravity;
	unsigned long long gas[BW_MESH_COUNT];	// OptixTraversableHandle per mesh

	// The #35 sweep (__raygen__sweep, same module): thread (r, i) pairs eye
	// sweepIdx[i] with the face sample bw_vis.cpp's ConfirmPairs picks for
	// confirmation ray r of that eye, and traces the segment on BW_MESH_VIS
	// exactly as a zero-gravity arc of one chord. Any clear pair sets *sweepClear.
	const float *eyes;				// xyz per eye sample of the run
	const unsigned int *sweepIdx;	// this launch's eyes (indices into 'eyes')
	const double *sweepSamples;		// per confirmation sample: p xyz, n xyz
	unsigned int *sweepClear;		// out: 1 when some pair is clear (in: 0)
	unsigned int sweepEyes, sweepRays, sweepNumSamples;
	unsigned int sweepTwoSided;		// 1: no back-face test of the sample
	unsigned long long sweepKey;	// face index << 32 (the hash key, mirror + 1 = 0 for direct rays)
	unsigned long long sweepBase;	// sweepIdx[0]'s place in the face's full candidate list
};

// The #14 tier B eye fan on the GPU (bw_fan.cu, its own module and pipeline).
// Launch (numRays, eyes in this batch): ray x of eye y finds its first occluder
// (closest hit on occGas, maxT when none), then marks every face whose padded
// slab on faceGas it meets before that distance plus 'past'.
struct BwFanLaunchParams
{
	const float *eyes;				// xyz per eye of this batch
	const float *dirs;				// xyz per fan direction
	const unsigned int *primFace;	// faceGas triangle -> face index
	const float *faceN;				// per face: the plane normal xyz and w = 1 to test the facing (one-sided)
	unsigned char *visible;			// per face: set to 1 when marked (in and out)
	float *tOut;					// -vischeck: the occluder distance per ray, ray-major per eye; or null
	unsigned int numRays;
	float maxT;						// no occluder: the ray runs this far
	float past;						// faces are marked up to the occluder distance plus this
	float facingTol;				// a one-sided face is marked while n . dir < this
	// Faces whose crossing region is unbounded (zero-area faces), tested in
	// the raygen program instead of as meshes. Per plate, in order: the face
	// index and the half-plane count h (both as uint bits), the plane n xyz and
	// d (n . x = d), then h half-planes m xyz and c (m . x <= c). A ray marks
	// the face when some point of it up to the occluder distance plus 'past'
	// is within 'pad' of the plane and of every half-plane.
	const float *plates;
	unsigned int numPlates;
	float pad;
	// The PVS gate (#77, bw_vis.cpp FanEyeMaySee): a ray marks a face only when its eye's
	// cluster's PVS holds one of the face's clusters. eyeCl per eye of this batch (-1: no
	// gate); per face its clusters faceCl[faceClOfs[f] .. faceClOfs[f + 1]) (none: no gate);
	// pvs: a row of pvsRowBytes per cluster.
	const int *eyeCl;
	const int *faceClOfs;
	const int *faceCl;
	const unsigned char *pvs;
	unsigned int pvsRowBytes;
	// The one-way occluders (#77, bw_vis.cpp FanOneWayMesh) on dirGas, traced up to the occGas
	// distance: a hit on occluder dirPrim[triangle] stops the ray only where bw_vis.cpp
	// NearestOccluder surely stops it, by dirSlop margins (__anyhit__dir). Per occluder its real
	// planes dirPlanes[dirOfs[o] .. dirOfs[o + 1]) (n xyz, d: 4 floats each) and dirDrawn per plane.
	const unsigned int *dirPrim;
	const float *dirPlanes;
	const unsigned char *dirDrawn;
	const unsigned int *dirOfs;
	float dirSlop;
	unsigned long long occGas, faceGas, dirGas;	// OptixTraversableHandle
};

#endif // BW_ARC_TYPES_H
