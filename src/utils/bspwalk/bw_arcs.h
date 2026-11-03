//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: Stage 3 ballistic arcs (#13): horizontal jump / fall edges, found
//          target-driven from the flood frontier and traced as chords against
//          the grown obstacles (GPU: OptiX triangle mesh; CPU: the reference,
//          exact half-spaces in double).
//
//          An arc is a feet-point parabola. The point against a grown obstacle
//          is the player box against the source solid, so a free arc is a free
//          box trajectory up to the chord error (sagitta <= ARC_SAGITTA).
//
//=============================================================================//

#ifndef BW_ARCS_H
#define BW_ARCS_H
#pragma once

#include <vector>
#include "bw_walk.h"
#include "bw_arc_types.h"

// Every chord stays within this of its parabola (vertical sagitta g dt^2 / 8).
#define ARC_SAGITTA		1.0
// Every arc is traced lifted this far (start and landing both): the rays then
// start and end in free space above the launch and landing floors, and are
// traced over their full length with no tolerance at the ends. A ray sees an
// obstacle only where it crosses a face, so a tolerance window at an end would
// hide that end's crossing (an arc through a wall between two slivers lying
// within the window of both faces, or one diving into the floor it leaves).
// The price: clearance is judged with the hull 1/32u higher.
#define ARC_LIFT		( 1.0 / 32.0 )
// Jump launch points and landing points are moved this far toward their
// polygon's centroid: a polygon edge cut by an obstacle lies 1/32u inside it.
#define ARC_INSET		0.25
// CPU reference: a chord is blocked only where it reaches this far into a
// grown obstacle; touching is free.
#define ARC_TOUCH		1e-6
// An arc whose (lifted) launch point is not at least this far outside every
// obstacle of its mesh is not tried (on either path). A ray that starts inside
// a closed mesh crosses no entry face, and one that starts on a face can miss
// the crossing at t = 0, so the GPU would call an arc through that obstacle
// free. A walk-off starts on the exact polygon edge, and an edge cut by an
// obstacle lies 1/32u inside it. The lift clears the launch floor's own top
// plane by at least ARC_LIFT * 0.7 = 0.022; this is 4 float ulps at 16384.
#define ARC_START_CLEAR	( 1.0 / 128.0 )
// -arccheck: a GPU / CPU disagreement whose grazing distance is at most this
// is float rounding of the mesh and the ray (coordinates up to 2^15, float
// ulp 0.004 there).
#define ARC_FLOAT_BOUND	( 1.0 / 64.0 )
// Launch / landing point combinations tried per node pair (bw_arcs.cpp).
#define ARC_COMBOS		9

// Most chords one arc may have (the tracers' stack buffers); profiles whose
// longest arc needs more are refused at load (bw_profile.cpp).
#define ARC_MAX_CHORDS	1024

// A ballistic arc in double precision (the GPU gets a float copy).
struct ArcD
{
	dvec3 start;		// feet point at t = 0 (lifted already for a duck-jump)
	double vx, vy, vz;	// initial velocity
	double tKick;		// one air jump: vz is reset to vzKick at tKick (0 = none)
	double vzKick;
	double tEnd;		// landing time
	int mesh;			// BW_MESH_*
	int nSeg[2];		// chords over [0, tKick or tEnd] and [tKick, tEnd]
};

// The arc API. An ArcD is the launch API: start point, initial velocity, an
// optional air jump, posture (mesh), and the time to trace to; the tracers
// take any batch of them. Phase B launch sources build ArcD the same way:
// TODO(#13B) pushes (trigger_push / trigger_catapult / func_conveyor /
//   trigger_wind: reached nodes inside the volume, push velocity added),
//   ladder jump-offs along the ladder, water exits, teleport destinations as
//   seeds, mover ride edges between poses (bw_walk.cpp EntityPoses), vehicles.
//   A velocity-driven launch has no target, so it needs a first-hit trace that
//   returns the hit point and normal (closest-hit on the GPU) instead of
//   blocked / free, and the landing node is looked up from the hit.
// TODO(#13B) slides: a landing on a steep face (0 < n.z < 0.7) is not a
//   landing (targets are walkable nodes only, so a steep face only ever blocks
//   an arc); continuing as a slide down the face needs the first-hit trace.
dvec3 ArcPoint( const ArcD &a, double g, double t );
// Fills the chord counts of an arc whose velocity and times are set.
void ArcSetSegments( ArcD &a, double g );
// How far both tracers raise an arc's chords: half the largest chord sagitta,
// g dt^2 / 16 (at most ARC_SAGITTA / 2). The parabola then lies within
// +-ARC_SAGITTA / 2 of the traced chords, above or below.
double ArcTraceRaise( const ArcD &a, double g );
// The arc that leaves 'start' with vertical speed vz (with airJump, one more
// jump of vz at the apex) and comes down on 'land' on its descending branch,
// straight in XY. False when 'land' is above the apex or the horizontal speed
// it needs exceeds maxSpeed.
bool SolveArcToPoint( const dvec3 &start, double vz, bool airJump, double g, const dvec3 &land, double maxSpeed,
					  int mesh, ArcD &out );
// The arcs of launch/landing combination 'combo' (0 .. ARC_COMBOS-1) of the
// pair A -> B, appended to out. Returns how many were added.
int PairArcs( const WalkNode &A, const WalkNode &B, const PlayerProfile &p, int combo, std::vector<ArcD> &out );

// Arc tracing. blocked[i] = 1 when arc i hits an obstacle of its mesh.
class CArcTracer
{
public:
	CArcTracer();
	~CArcTracer();
	// Obstacle sets (CPU) and, with wantGpu, OptiX plus both meshes. HasGpu()
	// says whether the GPU path came up.
	void Init( const CollisionWorld &world, const PlayerProfile &profile, bool wantGpu );
	void Shutdown();
	bool HasGpu() const { return m_bGpu; }
	void TraceCpu( const std::vector<ArcD> &arcs, std::vector<unsigned char> &blocked );
	bool TraceGpu( const std::vector<ArcD> &arcs, std::vector<unsigned char> &blocked );
	// The same on arcs already in the GPU form (bw_arc_types.h).
	bool TraceGpuRaw( const BwArc *arcs, size_t n, std::vector<unsigned char> &blocked );
	// Deepest reach of the arc's chords into any obstacle of its mesh, in plane
	// distance (> 0 inside, < 0: the closest clearance). For -arccheck.
	double Depth( const ArcD &a ) const;

	double m_tBuild;					// obstacle sets + meshes + GAS
	int m_nTris[BW_MESH_COUNT];
	size_t m_gasBytes[BW_MESH_COUNT];

private:
	bool m_bGpu;
	double m_g;
};

#endif // BW_ARCS_H
