//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: Stages 4 and 5 (#14): the eye volume, and face visibility tiered by
//          proof strength.
//
//          Stage 4 (bw_eyes.cpp). Where a player's eye can be, as convex
//          pieces in world space: the free crouch column over each reached
//          walkable polygon (feet up to max_jump_rise, or higher where a fall
//          from a ledge, ladder, seed or teleport arrival can pass over it)
//          moved up by the crouch eye height (and a stand column when the
//          profile needs one, and the dead-eye prism), water and ladder
//          volumes, the kept free arcs, mover ride sweeps, teleport arrivals
//          and the extra viewpoints (env_cubemap, point_camera,
//          point_viewcontrol and its moveto rail, -viewpoint). Flights between
//          already-reached floors are not in it (docs: accepted limitations). With a sky_camera every piece is
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
	int sky = 0;				// 0: real; k + 1: the copy mapped into skybox k (kept pieces, -dumpobj)
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
// Returns 1 when the column was cut into so many pieces that the rest of the
// obstacles were skipped (the pieces then over-reach), else 0.
int FeetColumn( const DWinding &poly, const DPlane &plane, double rise,
				const std::vector<const GrownPolytope *> &obstacles, double shrink, std::vector<EyePiece> &out );

// Eye samples for tier B, in world space (real or skybox-mapped).
struct EyeSample
{
	float x, y, z;
	int cluster;		// leaf cluster of the point (-1: solid or outside)
	int source;			// EyeSource
	int mapped;			// 1: a 3D skybox copy (origin + eye / scale)
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
	std::vector<dvec3> regionLo, regionHi;		// per cluster the pieces touch: box of every eye piece part in it
												// (+-1e30 when none was recorded; 0 for other clusters)
	std::vector<int> bareClusters;				// piece clusters with no sample (the lattice missed them)
	dvec3 allLo, allHi;							// box of every eye piece (the region of the one no-vis bucket)
	bool allAny = false;
	std::vector<dvec3> bareLo, bareHi;			// per bare cluster: its region box
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
	double maxSamples;					// widen the spacing until the node columns' estimate is about this or fewer
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
// Tier B samples of a piece (bw_eyes.cpp SamplePiece), exposed for -selftest.
void SampleEyePiece( const EyePiece &p, double spacing, std::vector<EyeSample> &out );
// Maps a piece into the 3D skybox: x -> origin + x / scale.
void SkyMapPiece( const EyePiece &in, const dvec3 &origin, double scale, EyePiece &out );

//-----------------------------------------------------------------------------
// Stage 5
//-----------------------------------------------------------------------------
enum FaceTier
{
	TIER_VISIBLE = 0,
	TIER_B,				// ray-sampled hidden
	TIER_A				// PVS-proven hidden
};

struct VisOptions
{
	bool tierB;			// run the ray passes (-notierb: everything not tier A is visible)
	int fanRays;		// rays per fan eye
	double fanSpacing;	// fan eyes: one sample per this cell (units)
	double faceSpacing;	// face samples at most this far apart (units, <= 4 per the design)
	int bins;			// angular bins per axis of the octahedral map (bins^2 eyes at most per sample tile)
	int binLevels;		// a tile with no clear ray retries at 2x, 4x, ... the bins over untried eyes
	bool cpuOnly;		// -cpuvis: never try the GPU for the face and mirror passes
	bool check;			// -vischeck: trace those passes on both paths, follow the CPU, report disagreements
	bool gpuSim;		// -visgpusim: the GPU path's batching, each segment traced on the CPU from its float form

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
	const char *path;				// face / mirror passes: "gpu" or "cpu"
	int meshTris;					// occluder triangles on the GPU
	long long checkSegments, gpuOnlyBlocked, cpuOnlyBlocked;	// -vischeck, per segment
	int checkTileDiffer;			// -vischeck: tile levels whose any-free outcome differs
	int visibleFan, visibleFace, visibleMirror;
	int visibleNoEye;				// kept visible with no ray test (WHY_NOEYE)
	int noEyeClusters;				// eye clusters the pieces touch that hold no eye sample
	int solidEyes;					// unmapped viewpoint samples in a solid leaf: candidates of every face
	double tTierA, tOccluders, tFan, tFace, tTotal;

	VisStats() { memset( this, 0, sizeof( *this ) ); path = "cpu"; }
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
						// cluster's eye region in front of it, or it is in the PVS of an eye cluster
						// whose samples all lie behind it while its eye region reaches in front,
						// or a viewpoint in solid is in front of it
	WHY_COUNT
};
const char *VisReasonName( int why );

void RunVisibility( const CollisionWorld &world, const PlayerProfile &profile, const WalkGraph &graph,
					const EyeVolume &eyes, const VisOptions &opts, VisResult &out );

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

#endif // BW_VIS_H
