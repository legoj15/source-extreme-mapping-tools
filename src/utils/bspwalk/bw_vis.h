//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: Stages 4 and 5 (#14): the eye volume, and face visibility tiered by
//          proof strength.
//
//          Stage 4 (bw_eyes.cpp). Every place a player's eye can be, as convex
//          pieces in world space: the free crouch column over each reached
//          walkable polygon (feet up to max_jump_rise, or higher under an
//          incoming fall) moved up by the crouch eye height, water and ladder
//          volumes, the traced free arcs, mover ride sweeps, teleport arrivals
//          and the extra viewpoints (env_cubemap, point_camera,
//          point_viewcontrol, -viewpoint). With a sky_camera every piece is
//          also mapped into the 3D skybox. Each piece is split down the BSP to
//          the leaves it touches (the eye clusters) and sampled for tier B.
//
//          Stage 5 (bw_vis.cpp). Tier A: a face none of whose leaves is in the
//          PVS of any eye cluster (reflective faces that stay visible add their
//          own clusters). Tier B: the CPU ray reference, an eye-driven fan and a
//          face-driven pass with angular binning and mirrored eyes, against
//          opaque solid brushes only.
//
//=============================================================================//

#ifndef BW_VIS_H
#define BW_VIS_H
#pragma once

#include <vector>
#include "bw_walk.h"
#include "bw_arc_types.h"
#include "bw_arcs.h"

// What put an eye piece into the volume (JSON "eyes.pieces").
enum EyeSource
{
	EYESRC_NODE = 0,		// free crouch column over a reached walkable polygon
	EYESRC_WATER,			// reached water volume
	EYESRC_LADDER,			// reached ladder segment or ladder brush volume
	EYESRC_ARC,				// a traced free arc (node arc, special launch, trigger target)
	EYESRC_RIDE,			// a mover top swept between two linked poses
	EYESRC_TELEPORT,		// a triggered teleport's destination
	EYESRC_CUBEMAP,			// env_cubemap sample origin (lump 42)
	EYESRC_CAMERA,			// point_camera origin
	EYESRC_VIEWCONTROL,		// point_viewcontrol origin
	EYESRC_CLI,				// -viewpoint
	EYESRC_SKYCAMERA,		// sky_camera origin: the 3D skybox pass takes its PVS from there
	EYESRC_COUNT
};
const char *EyeSourceName( int source );
// A camera the engine draws from (cubemap, point_camera, point_viewcontrol, -viewpoint, sky_camera).
bool IsViewpointSource( int source );

enum EyePieceKind
{
	EYE_POINT = 0,
	EYE_SEGMENT,
	EYE_POLY		// convex polyhedron: planes (n . x <= d) and its vertices
};

struct EyePiece
{
	int kind;
	int source;					// EyeSource
	std::vector<DPlane> planes;	// EYE_POLY
	std::vector<dvec3> verts;	// EYE_POLY: vertices; EYE_SEGMENT: the two ends; EYE_POINT: the point
	dvec3 mins, maxs;
};

// A convex polyhedron from half-spaces (n . x <= d): its vertices and bounds.
// Drops planes that bound nothing. False when the set is empty or flat.
bool EyePolyFromPlanes( std::vector<DPlane> &planes, EyePiece &out );
// The convex hull of points as an EYE_POLY. False when the points are flat.
bool EyePolyFromPoints( const std::vector<dvec3> &pts, EyePiece &out );
// The part of a polyhedron on the back of (n, d) (n . x <= d). False when empty.
bool EyePolyClip( const EyePiece &in, const dvec3 &n, double d, EyePiece &out );
// poly minus convex polytope q (planes n . x <= d): the convex pieces of poly
// outside q (a peel, one plane of q at a time). Appended to out.
void EyePolySubtract( const EyePiece &poly, const std::vector<DPlane> &q, std::vector<EyePiece> &out );
// The free crouch-feet column over a walkable polygon: the prism from its plane
// up by 'rise' (along z) minus, for every obstacle (each shrunk by 'shrink'),
// the part of the prism at or above some point of the obstacle inside the
// prism (the upward extrusion of obstacle n prism, which is exact because the
// prism is a union of vertical segments from its floor). An obstacle below the
// floor meets the prism nowhere, so it removes nothing. Pieces in feet space.
void FeetColumn( const DWinding &poly, const DPlane &plane, double rise,
				 const std::vector<const GrownPolytope *> &obstacles, double shrink, std::vector<EyePiece> &out );

// Eye samples for tier B, in world space (real or skybox-mapped).
struct EyeSample
{
	float x, y, z;
	int cluster;		// leaf cluster of the point (-1: solid or outside)
	int source;			// EyeSource
	int sky;			// -1: a real eye; k >= 0: the copy in sky camera k's 3D skybox (skyOrigins[k] + eye / skyScales[k])
};

struct EyeVolume
{
	std::vector<unsigned char> clusters;		// eye clusters (real and skybox-mapped): 1 per cluster
	std::vector<unsigned char> pieceClusters;	// the clusters the eye pieces touch, real and skybox-mapped
												// ('clusters' as merged from the threads, before the
												// solid-viewpoint fill sets every cluster)
	std::vector<EyeSample> samples;				// tier B eye set
	std::vector<EyePiece> keep;					// every piece, when kept for -dumpobj
	int pieces[EYESRC_COUNT];
	int mappedPieces;							// skybox copies
	int numClusters;							// clusters in the BSP's vis data (0: no vis)
	int eyeClusters;							// clusters set in 'clusters'
	int viewpoints;								// extra viewpoints added
	std::vector<int> bareClusters;				// piece clusters with no sample (the lattice missed them)
	std::vector<dvec3> bareLo, bareHi;			// per bare cluster: box of every eye piece part in it
	bool sky;									// a sky_camera was found
	dvec3 skyOrigin;							// the first sky_camera (JSON)
	double skyScale;
	std::vector<dvec3> skyOrigins;				// every sky_camera: each maps every piece
	std::vector<double> skyScales;
	bool solidEye;								// a viewpoint lies wholly in a solid leaf: the engine
												// draws every cluster from there, so all are eye clusters
	double spacing;								// sample spacing in use (EyeOptions::spacing or wider)
	double tBuild;

	EyeVolume() : mappedPieces( 0 ), numClusters( 0 ), eyeClusters( 0 ), viewpoints( 0 ), sky( false ),
				  skyOrigin( 0, 0, 0 ), skyScale( 16 ), solidEye( false ), spacing( 16 ), tBuild( 0 )
	{
		memset( pieces, 0, sizeof( pieces ) );
	}
};

struct EyeOptions
{
	double spacing;						// tier B sample spacing (units)
	double maxSamples;					// widen the spacing until about this many samples or fewer
	bool keepPieces;					// keep every piece (for -dumpobj)
	std::vector<dvec3> viewpoints;		// -viewpoint

	EyeOptions() : spacing( 8 ), maxSamples( 24e6 ), keepPieces( false ) {}
};

// Stage 4. Needs a flooded graph (FloodWalkGraph).
void BuildEyeVolume( const CollisionWorld &world, const PlayerProfile &profile, const WalkGraph &graph,
					 const EyeOptions &opts, EyeVolume &eyes );
// The leaf of a point (world BSP), and the clusters a piece touches (bits set
// in 'clusters', sized to the vis cluster count). Main or worker threads.
int PointLeaf( const dvec3 &p );
// Returns 1 when the piece touched a leaf with a cluster, | 2 when it touched a
// leaf without one (solid or outside the map).
int PieceClusters( const EyePiece &piece, std::vector<unsigned char> &clusters );
// Maps a piece into the 3D skybox: x -> origin + x / scale.
void SkyMapPiece( const EyePiece &in, const dvec3 &origin, double scale, EyePiece &out );
// HL2's VEC_DEAD_VIEWHEIGHT: a dead player's eye, 14u above the floor.
#define EYE_DEAD_HEIGHT	14.0

//-----------------------------------------------------------------------------
// Stage 5
//-----------------------------------------------------------------------------
enum FaceTier
{
	TIER_VISIBLE = 0,
	TIER_B,				// ray-sampled hidden
	TIER_A				// PVS-proven hidden
};

// Bounds of the user-settable tier B options (bspwalk.cpp clamps to them).
// Face candidates are capped at 65536 (bw_vis.cpp), so 256 bins per axis (65536
// bins) already take every candidate. The doubled bin levels stop at 1024 bins
// per axis whatever the level count; the level bound keeps the loop short.
#define VIS_MAX_BINS			256
#define VIS_MAX_BIN_LEVELS		8
// -facespacing: at most the design's 4u (a coarser lattice can step over a thin
// sight line); the floor keeps a typo from asking for billions of samples (a face
// is capped at 2^18 samples by widening the grid anyway).
#define VIS_MAX_FACE_SPACING	4.0
#define VIS_MIN_FACE_SPACING	0.25
// -fanspacing floor: a fan cell key packs 21 bits per axis around a 2^20 offset,
// so a cell must be at least 16384 / 2^20 = 1/64u wide; a skybox copy's cells
// shrink by its sky camera's scale, and 1u stays clear of that up to scale 64.
#define VIS_MIN_FAN_SPACING		1.0

struct VisOptions
{
	bool tierB;			// run the ray passes (-notierb: everything not tier A is visible)
	int fanRays;		// rays per fan eye
	double fanSpacing;	// fan eyes: one sample per this cell (units)
	double faceSpacing;	// face samples at most this far apart (units, <= 4 per the design)
	int bins;			// angular bins per axis of the octahedral map (bins^2 eyes at most per sample tile)
						// (1 .. VIS_MAX_BINS: more bins than the face candidate cap changes nothing)
	int binLevels;		// a tile with no clear ray retries at 2x, 4x, ... the bins over untried eyes
						// (1 .. VIS_MAX_BIN_LEVELS)
	bool cpuOnly;		// -cpuvis: never try the GPU for the face and mirror passes
	bool check;			// -vischeck: trace those passes on both paths, follow the CPU, report disagreements
						// beyond float rounding (not with gpuSim: there is no second path to compare)
	bool gpuSim;		// -visgpusim: the GPU path's batching, each segment traced on the CPU in double

	VisOptions() : tierB( true ), fanRays( 1024 ), fanSpacing( 64 ), faceSpacing( 4 ), bins( 32 ), binLevels( 2 ),
				   cpuOnly( false ), check( false ), gpuSim( false ) {}
};

struct VisStats
{
	int faces;
	int worldFaces, entityFaces, dispFaces;
	int tierA, tierB, visible;
	int reflective;					// reflective faces (water shader, func_reflective_glass)
	int reflectClusters;			// clusters tier A added for visible reflective faces
	int occluders;					// opaque solid brushes
	int occludersSkipped;			// solid brushes left out: see-through material or contents
	int faceNoLeaf;					// world faces in no leaf list (leaves from their bounds)
	long long fanRays, faceRays, mirrorRays;
	const char *rayUnit;			// what faceRays and mirrorRays count: "segments_launched" on the GPU paths
									// (a mirror pair is two), "pairs_tested" on the CPU path (tests up to the
									// first clear one), "mixed" after a GPU fall-back. Not comparable across them.
	const char *path;				// face / mirror passes: "gpu", "cpu" (-cpuvis, or no GPU path), "gpusim"
									// (-visgpusim) or "cpu-fallback" (the GPU path was rejected or a launch failed)
	bool tierBRan;					// the ray passes ran (false: -notierb, or a viewpoint in solid)
	int meshTris;					// occluder triangles on the GPU
	long long gpuLaunchFailures;	// launches that failed: that batch and every later one ran on the CPU
	long long checkSegments, gpuOnlyBlocked, cpuOnlyBlocked;	// -vischeck, per segment
	long long checkUnexplained;		// -vischeck: disagreements beyond VIS_FLOAT_BOUND (exit 1)
	double checkMaxDist;			// -vischeck: largest grazing distance of a disagreement
	int checkTileDiffer;			// -vischeck: tile levels whose any-free outcome differs
	int visibleFan, visibleFace, visibleMirror;
	int visibleNoEye;				// kept visible with no ray test (WHY_NOEYE)
	int noEyeClusters;				// eye clusters the pieces touch that hold no eye sample
	int solidEyes;					// real (not skybox-mapped) viewpoint samples in a solid leaf: they keep the faces in
									// front of them visible; counted even when a solid viewpoint skips tier B
	int cappedFaces;				// faces whose candidate eyes passed the cap (65536, bw_vis.cpp) and were thinned: tier B
									// tries only the kept ones, and fewer eyes can only make a face look more hidden
	double tFaceClusters;			// BuildFaceClusters, all of it
	double tFaceBounds;				// ... of which the faces with no leaf listing them (their bounds -> clusters)
	double tTierA, tOccluders, tFan, tFace, tTotal;

	VisStats() { memset( this, 0, sizeof( *this ) ); path = "cpu"; rayUnit = "pairs_tested"; }
};

struct VisResult
{
	std::vector<unsigned char> tier;	// per face
	std::vector<unsigned char> why;		// per face: VisReason
	bool pvs;							// the BSP has vis data (tier A ran)
	VisStats stats;

	VisResult() : pvs( false ) {}
};

enum VisReason
{
	WHY_PVS = 0,		// tier A: no leaf in any eye cluster's PVS
	WHY_RAYS,			// tier B: every sampled ray blocked
	WHY_FAN,			// visible: hit by an eye fan ray
	WHY_RAY,			// visible: a clear face-to-eye ray
	WHY_MIRROR,			// visible: a clear ray through a reflective face
	WHY_ENTITY,			// visible: a brush entity face (not evaluated)
	WHY_NOTIERB,		// visible: in the PVS and -notierb
	WHY_NOEYE,			// visible: no ray test possible (kept visible): the face has no sample, or it
						// (or a mirror it shows in) is in the PVS of an eye cluster that holds no
						// eye sample (any such cluster without vis data), with part of that
						// cluster's eye region in front of it, or a viewpoint in solid is in front of it
	WHY_COUNT
};
const char *VisReasonName( int why );

void RunVisibility( const CollisionWorld &world, const PlayerProfile &profile, const WalkGraph &graph,
					const EyeVolume &eyes, const VisOptions &opts, VisResult &out );

// -vischeck: a GPU / CPU disagreement whose grazing distance is at most this is
// float rounding of the mesh and the ray, the same bound as -arccheck.
#define VIS_FLOAT_BOUND	ARC_FLOAT_BOUND

// A straight sight segment in double: the form the CPU tracer takes.
struct VisSegment
{
	dvec3 a, b;
};

// GPU face / mirror ray encoding (bw_vis.cpp), exposed for -selftest. A straight
// segment a -> b is launched as a zero-gravity arc of the arc kernel: o = a,
// v = b - a, dg0 = -gravity, tEnd = 1, one chord, mesh BW_MESH_VIS. 'gravity'
// must be the float later passed to BwOptix_TraceArcs; it cancels exactly. The
// endpoints are rounded to float here, so a GPU answer can differ from the
// double trace (SegmentCrossesConvex) for a segment grazing an occluder.
void BwVis_EncodeSegment( BwArc &out, const dvec3 &a, const dvec3 &b, float gravity );
// Traces segs against the BW_MESH_VIS mesh in one launch (BwOptix_Init and
// BwOptix_BuildMesh( BW_MESH_VIS, ... ) first), serialized with the face pass's
// own launches. blocked[i] = 1 when segment i hits the mesh. False when the
// launch fails (blocked is then meaningless). RunVisibility's own known-answer
// check of the GPU path goes through this.
bool BwVis_TraceSegmentsGpu( const std::vector<VisSegment> &segs, float gravity, std::vector<unsigned char> &blocked );

// Ray helpers (bw_vis.cpp), exposed for -selftest.
// Does the open segment a -> b pass through the interior of the convex
// polytope (planes n . x <= d, already shrunk by the caller)? Touching is free.
bool SegmentCrossesConvex( const dvec3 &a, const dvec3 &b, const DPlane *planes, int numPlanes );
// The reflection of e in the plane (n, d).
dvec3 MirrorPoint( const dvec3 &e, const dvec3 &n, double d );
// Unit direction i of n on a Fibonacci sphere.
dvec3 FibonacciDir( int i, int n );
// Octahedral bin of a unit direction, 0 .. bins * bins - 1.
int OctBin( const dvec3 &dir, int bins );
// Thins a face's candidate eyes to at most 'cap'. The eyes are listed cluster by cluster:
// runStart[i] is where the i-th non-empty cluster's run begins (ascending, runs end where
// the next begins). Every cluster keeps at least one eye (while there are no more clusters
// than 'cap'; past that an even subset of the clusters keeps one each), and the rest of the
// budget is shared out evenly: a cluster with fewer eyes than its share keeps them all and
// the others split what is left. Within a cluster the kept eyes are evenly spaced in
// list order. Order is kept; nothing happens when the eyes already fit.
void CapCandidateEyes( std::vector<int> &eyes, const std::vector<int> &runStart, int cap );

#endif // BW_VIS_H
