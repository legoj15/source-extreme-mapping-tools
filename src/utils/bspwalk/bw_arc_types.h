//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: plain types shared by the host (MSVC) and the OptiX arc kernel
//          (bw_arcs.cu, nvcc). No includes, no C++ beyond POD structs.
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
};

#endif // BW_ARC_TYPES_H
