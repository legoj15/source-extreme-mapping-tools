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
#include "bw_arc_types.h"
#include "bw_arcs.h"
#include "bw_cubemap.h"

// What put an eye piece into the volume (JSON "eyes.pieces").
enum EyeSource
{
	EYESRC_NODE = 0,		// free crouch column over a reached node polygon (slides included)
	EYESRC_WATER,			// reached water volume
	EYESRC_LADDER,			// reached ladder segment or ladder brush volume
	EYESRC_ARC,				// a traced free arc (node arc, special launch, trigger target)
	EYESRC_RIDE,			// a mover top swept between two linked poses
	EYESRC_TELEPORT,		// a triggered teleport's destination
	EYESRC_CUBEMAP,			// env_cubemap sample origin (lump 42) that passed the CheckCubemaps rules
	EYESRC_CAMERA,			// point_camera origin
	EYESRC_VIEWCONTROL,		// point_viewcontrol origin
	EYESRC_CLI,				// -viewpoint
	EYESRC_SKYCAMERA,		// sky_camera origin: the 3D skybox pass takes its PVS from there
	EYESRC_SURF,			// the free part of a surf flight off a surf lip (#26)
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
	int viewpoint = -1;			// the camera a viewpoint piece belongs to (-1: not one); viewPart 0 is its
	int viewPart = 0;			// own point, 1 on a point_viewcontrol's rail segments in order
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
	int sky;			// -1: a real eye; k >= 0: the copy in sky camera k's 3D skybox (skyOrigins[k] + eye / skyScales[k])
};

// The region of eye piece vertices that bw_vis asks "does it reach in front of this
// face's plane?" (CountCandidates, BareMaySee). A box overstates the reach along any
// direction off the axes: against a wall at 19 degrees to the grid a 400 x 128 region
// reached 13u past the plane it never gets near (d1_trainstation_02 faces 1159/1160,
// #77). So the bound is a k-DOP: the box (the six axis directions, in double) and the
// most d . v over the vertices for REGION_EXTRA more directions (RegionDir). Every
// support value is rounded up, so the hull of the recorded vertices (every piece part's,
// and for a side that does not clip, its vertices and band-edge crossings, which span
// that part; plus the cluster's samples), and so every eye position of the pieces,
// lies inside it.
enum { REGION_EXTRA = 28, REGION_DIRS = 6 + REGION_EXTRA };
// Unit direction i: 0..5 are +x -x +y -y +z -z (the box), 6.. the extra ones (the 12 edge
// diagonals, the 8 corner diagonals, 8 more across the xy plane at odd multiples of 22.5).
const dvec3 &RegionDir( int i );

struct EyeBound
{
	dvec3 lo, hi;					// the box
	float h[REGION_EXTRA];			// per extra direction: max d . v, rounded up (-FLT_MAX: no vertex yet)
	bool any;						// at least one vertex (Unbounded() counts)

	EyeBound();						// empty
	void Add( const dvec3 &v );
	void Merge( const EyeBound &o );
	void Unbounded();				// everything: the pieces' region was not recorded
	void AtOrigin();				// the single point (0, 0, 0)
	// An upper bound of n . x over the region, the least the k-DOP gives (a linear program
	// over the directions, never above the box's bound n . (corner of the box). Stops once
	// the bound is <= stopAt (a caller who only compares with stopAt); any value it returns
	// is still an upper bound. 'any' must be set.
	double Support( const dvec3 &n, double stopAt = -1e300 ) const;
	double BoxSupport( const dvec3 &n ) const;	// the box alone
};

struct EyeVolume
{
	std::vector<unsigned char> clusters;		// eye clusters (real and skybox-mapped): 1 per cluster
	std::vector<unsigned char> pieceClusters;	// the clusters the eye pieces touch, real and skybox-mapped
												// ('clusters' as merged from the threads)
	std::vector<EyeSample> samples;				// tier B eye set
	std::vector<EyePiece> keep;					// every piece, when kept for -dumpobj
	int pieces[EYESRC_COUNT];
	int mappedPieces;							// skybox copies
	int numClusters;							// clusters in the BSP's vis data (0: no vis)
	int eyeClusters;							// clusters set in 'clusters'
	int viewpoints;								// extra viewpoints added
	std::vector<EyeBound> region;				// per cluster the pieces touch: bound of every eye piece part's vertices in it
												// (unbounded when none was recorded; the point 0 for other clusters)
	std::vector<int> bareClusters;				// piece clusters with no sample (the lattice missed them)
	EyeBound all;								// bound of every eye piece (the region checked once without vis data);
												// all.any false: no piece
	std::vector<EyeBound> bare;					// per bare cluster: its region bound
	bool sky;									// a sky_camera was found
	dvec3 skyOrigin;							// the first sky_camera (JSON)
	double skyScale;
	std::vector<dvec3> skyOrigins;				// every sky_camera: each maps every piece
	std::vector<double> skyScales;
	std::vector< std::pair<int, dvec3> > solidViews;	// viewpoints with a sample in a solid leaf, one per
												// camera: source and its first such sample, sorted (JSON
												// eyes.solid_viewpoints and solid_eye, the warning; the
												// engine draws every cluster from there, and bw_vis keeps the
												// faces those samples see visible, WHY_SOLIDVIEW)
	CubemapReport cubemaps;						// the verdict on every env_cubemap (bw_cubemap.h)
	double spacing;								// sample spacing in use (EyeOptions::spacing or wider)
	double spacingRequested;					// EyeOptions::spacing
	bool widened;								// spacing > spacingRequested: tier B is sampled coarser than asked
	const char *capReason;						// what set sampleCap: "eyemaxsamples", "memory" (the guard: a share of
												// the free memory), "memory_unknown" (free memory was unreadable: a
												// fixed cap) or "index" (bw_vis indexes the samples with int)
	double sampleCap;							// the most samples the estimate may reach before the spacing widens
	double estimate;							// estimated samples at 'spacing'
	bool capMet;								// estimate <= sampleCap (false: widening stalled or hit its limit)
	double tBuild;

	EyeVolume() : mappedPieces( 0 ), numClusters( 0 ), eyeClusters( 0 ), viewpoints( 0 ), sky( false ),
				  skyOrigin( 0, 0, 0 ), skyScale( 16 ), spacing( 16 ), spacingRequested( 16 ), widened( false ),
				  capReason( "index" ), sampleCap( 0 ), estimate( 0 ), capMet( true ), tBuild( 0 )
	{
		memset( pieces, 0, sizeof( pieces ) );
	}
};

struct EyeOptions
{
	double spacing;						// tier B sample spacing (units)
	double maxSamples;					// -eyemaxsamples (0: none, the default): widen the spacing (x1.25 steps) until the
										// estimated number of eye samples is about this many or fewer. The estimate covers
										// every eye sample the volume emits (all piece sources, each with its skybox
										// copies), not just the node columns; it is an estimate, so the real count can
										// differ a little. Without it a memory guard sets the cap (bw_eyes.cpp
										// EYE_GUARD_BYTES); with it the user's number replaces the guard
	bool keepPieces;					// keep every piece (for -dumpobj)
	std::vector<dvec3> viewpoints;		// -viewpoint
	std::vector<dvec3> traceEyes;		// -traceeye: print the piece (and node) that emits a sample at each point
	CubemapOptions cubemaps;			// which env_cubemaps may be viewpoints

	EyeOptions() : spacing( 8 ), maxSamples( 0 ), keepPieces( false ) {}
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
// shrink by its sky camera's scale, and 1u stays clear of that below scale 64. A
// larger scale only merges fan cells (fewer fan eyes; the ray passes still decide).
#define VIS_MIN_FAN_SPACING		1.0
// -eyeconfirm: every candidate eye of a face that the tiles left hidden traces
// this many pairs (past the 65536 cap every candidate too, a chunk at a time:
// bw_vis.cpp SweepCandidates), so the bound keeps a face's confirmation within
// 256 pairs per candidate.
#define VIS_MAX_CONFIRM_RAYS	256

struct VisOptions
{
	bool tierB;			// run the ray passes (-notierb: everything not tier A is visible)
	int fanRays;		// rays per fan eye
	double fanSpacing;	// fan eyes: one sample per this cell (units)
	double faceSpacing;	// face samples at most this far apart (units, default 4; the command line clamps it to
						// VIS_MIN_FACE_SPACING .. VIS_MAX_FACE_SPACING: a
						// larger value samples faces more coarsely and can hide a face a finer grid finds
						// visible; a face too big for FACE_MAX_SAMPLES gets a doubled grid anyway)
	int bins;			// angular bins per axis of the octahedral map (bins^2 eyes at most per sample tile)
						// (1 .. VIS_MAX_BINS: more bins than the face candidate cap changes nothing)
	int binLevels;		// a tile with no clear ray retries at 2x, 4x, ... the bins over untried eyes
						// (1 .. VIS_MAX_BIN_LEVELS)
	int confirmRays;	// then every candidate eye traces to this many samples spread over the face (0: off)
						// (0 .. VIS_MAX_CONFIRM_RAYS)
	bool cpuOnly;		// -cpuvis: never try the GPU for the fan, face and mirror passes
	bool check;			// -vischeck: trace those passes on both paths, follow the CPU, report disagreements
						// beyond float rounding (not with gpuSim: there is no second path to compare)
	bool gpuSim;		// -visgpusim: the GPU path's batching, each segment traced on the CPU in double
	std::vector<int> traceFaces;	// -traceface: print which pass marked these faces, with the eye and ray
									// (fan rays and face-pass pairs only on the CPU paths: -cpuvis)

	VisOptions(): tierB( true ), fanRays( 1024 ), fanSpacing( 64 ), faceSpacing( 4 ), bins( 32 ), binLevels( 2 ), confirmRays( 8 ),
				   cpuOnly( false ), check( false ), gpuSim( false ) {}
};

struct VisStats
{
	int faces;
	int worldFaces, entityFaces, dispFaces, badFaces;
	int tierA, tierB, visible;
	int reflective;					// reflective faces (water shader, func_reflective_glass)
	int reflectClusters;			// clusters tier A added for visible reflective faces
	int dispMirrors, reflectPlanes;	// visible reflective displacements, each a mirror per reflection plane (#78)
	int occluders;					// opaque solid brushes
	int occludersSkipped;			// solid brushes left out: see-through material or contents, or every side nodraw
	int occludersOneWay;			// opaque world brushes with a nodraw side open to the map: they block only a
									// segment that enters through a drawn side from open space (#77)
	int seamContacts;				// pairs of two-sided occluders with opposite sides on one plane (#33, #40)
	int occludersSeam;				// seam fillers made for them (the rest could not be proven inside solid)
	int seamExtended;				// ... of which reach past an edge where a third brush continues the solid
	int faceNoLeaf;					// world faces in no leaf list (leaves from their bounds)
	long long fanRays, faceRays, mirrorRays;
	const char *rayUnit;			// what faceRays and mirrorRays count: "segments_launched" on the GPU paths
									// (a mirror pair is two), "pairs_tested" on the CPU path (tests up to the
									// first clear one), "mixed" after a GPU fall-back. Not comparable across them.
	const char *path;				// face / mirror passes: "gpu", "cpu" (-cpuvis, or no GPU path), "gpusim"
									// (-visgpusim) or "cpu-fallback" (the GPU path was rejected or a launch failed)
	bool tierBRan;					// the ray passes ran (false: -notierb)
	int meshTris;					// occluder triangles on the GPU
	long long gpuLaunchFailures;	// face / mirror launches that failed (the canary's included): that batch and
									// every later one ran on the CPU
	int fanLaunchFailures;			// 1 when the GPU fan's setup or a launch failed and the CPU fan ran instead
	long long checkSegments, gpuOnlyBlocked, cpuOnlyBlocked;	// -vischeck, per segment
	long long checkUnexplained;		// -vischeck: disagreements beyond VIS_FLOAT_BOUND (exit 1)
	double checkMaxDist;			// -vischeck: largest grazing distance of a disagreement
	int checkTileDiffer;			// -vischeck: tile levels whose any-free outcome differs
	const char *fanPath;			// fan pass: "gpu", "cpu" (-cpuvis, -visgpusim, or no GPU path) or "cpu-fallback"
									// (the GPU fan failed to set up or launch; the CPU fan ran instead)
	long long fanMeshTris;			// GPU fan: occluder plus face slab triangles
	int fanDegenerate;				// GPU fan: faces no slab can be built for, marked without a ray
	int fanPlates;					// GPU fan: zero-area faces tested as plates (unbounded crossing region)
	int fanEyeInSlab;				// GPU fan: faces whose slab may hold a fan eye, marked without a ray
	long long fanCheckRays;			// -vischeck: fan rays traced on both paths
	long long fanGpuShort;			// -vischeck: rays whose GPU occluder distance is shorter than the CPU's
	double fanMaxShort;				// -vischeck: the largest such shortfall (units)
	int fanGpuOnlyFaces, fanCpuOnlyFaces;	// -vischeck: faces only the GPU fan / only the CPU fan marks (the latter exit 1)
	int visibleFan, visibleFace, visibleMirror;
	int visibleConfirm;				// of visibleFace + visibleMirror: the first clear ray came from the confirmation pass
	int visibleNoEye;				// kept visible with no ray test (WHY_NOEYE)
	int noEyeClusters;				// eye clusters the pieces touch that hold no eye sample
	int solidEyes;					// real (not skybox-mapped) viewpoint samples in a solid leaf: they keep the faces
									// they see visible (WHY_SOLIDVIEW); counted under -notierb too
	int solidViewPvs;				// faces the PVS hid that tier A kept visible: a solid viewpoint sample sees them
	int solidViewRays;				// faces tier A did not hide (in the PVS of the eye clusters, the clusters tier A
									// added for reflective faces included) that no ray from an eye cluster reached
									// and the face pass kept visible (WHY_SOLIDVIEW): a solid viewpoint sample sees
									// them, directly or through a mirror
	int facesWidened;				// faces sampled coarser than faceSpacing (the per-face sample cap)
	double faceSpacingMax;			// the widest spacing any face used (0: none widened)
	int cappedFaces;				// faces (mirrors included) whose candidate eyes passed the cap (65536, bw_vis.cpp): the
									// tiles bin a thinned set, then every candidate is swept (sweptFaces)
	int sweptFaces;					// faces the tiles left hidden whose candidates (or a mirror's) were all paired with the
									// -eyeconfirm samples, a chunk at a time (bw_vis.cpp SweepCandidates)
	int sweptVisible;				// ... of which the sweep found a clear ray: the thinned set alone would have hidden them
	int cappedKept;					// faces past the cap kept visible untested (-eyeconfirm 0: nothing to sweep with)
	long long sweepEyes;			// eyes the sweeps paired
	double tFaceClusters;			// BuildFaceClusters, all of it
	double tFaceBounds;				// ... of which the faces with no leaf listing them (their bounds -> clusters)
	double tTierA, tOccluders, tFan, tMirror, tFace, tTotal;	// tOccluders: the occluder build alone; tMirror: the
																// mirror eye setup before the face pass
	double tFanSetup;				// GPU fan: meshes and upload (inside tFan)

	VisStats()
	{
		memset( this, 0, sizeof( *this ) );
		path = "cpu";
		fanPath = "cpu";
		rayUnit = "pairs_tested";
	}
};

// -traceface (#85): an eye sample that sees a traced visible face, and the pass that found it
// ("fan", "face pass", "mirror pass"); written as the face's "seen_from" in the JSON. For a GPU
// verdict it is the first clear pair of the CPU replay, which need not be the ray the GPU found.
struct VisSeenFrom
{
	int face;
	const char *pass;
	int eye;			// index into the eye samples
	float origin[3];
	int cluster;
	const char *source;	// EyeSourceName
};

struct VisResult
{
	std::vector<unsigned char> tier;	// per face
	std::vector<unsigned char> why;		// per face: VisReason
	bool pvs;							// the BSP has vis data (tier A ran)
	VisStats stats;
	std::vector<VisSeenFrom> seenFrom;	// -traceface: per traced visible face whose eye was named

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
	WHY_BADLUMP,		// visible: the face's lump data is out of range (not evaluated)
	WHY_NOEYE,			// visible: kept visible because no ray test is possible (set in bw_vis.cpp FacePassOne):
						//  - the face has no sample point, or
						//  - it is in the PVS of a bare eye cluster (an eye piece cluster that holds no eye
						//    sample: the lattice missed it; with no vis data any bare cluster counts) and part
						//    of that cluster's eye region is in front of its plane, or
						//  - it is in the PVS of an eye cluster whose samples all lie behind it while that
						//    cluster's eye region reaches in front (without vis data: no eye in front while
						//    the region bound of all pieces reaches it), or
						//  - a visible reflective face R it shows in (part of it lies in front of R) meets
						//    any of the conditions just above, or
						//  - with -eyeconfirm 0, its candidate eyes (or a mirror's) passed the cap and the
						//    thinned set found no clear ray: the rest cannot be swept without confirmation
						//    pairs.
	WHY_SOLIDVIEW,		// visible: a viewpoint sample in a solid leaf (cluster -1: the engine draws every cluster
						//  from it) sees the face: it lies in front of it (any such sample for a two-sided face
						//  or a displacement) and a segment from it to one of the face's samples is clear of the
						//  occluders, skipping those that hold it and those entered from a solid leaf (the
						//  engine draws a brush side only toward open leaves); or
						//  it sees the face through a visible reflective face R it lies in front of, by the
						//  mirrored path (SolidSeesFace). Kept by tier A (the direct test) and by the face pass
						//  (both) when no ray from an eye cluster reached the face, and when it cannot tell (no
						//  face samples, or past SOLID_MAX_PAIRS segments) (#76).
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
// The confirmation pass's choice (bw_vis.cpp ConfirmPairs, and the GPU sweep
// kernel in bw_arcs.cu, which must match it): of nS samples, the one ray r of
// 'rays' pairs with the eye numbered 'eye' (its place in a face's candidate list
// for a direct sweep; in a mirror sweep its place among the eyes kept after the
// culls, a number unique per face and mirror). 'key' is
// face index << 32 ^ (mirror + 1) << 48. Exposed for -selftest.
size_t BwVis_ConfirmSample( unsigned long long key, unsigned long long eye, int r, int rays, size_t nS );
// Traces segs against the BW_MESH_VIS mesh in one launch (BwOptix_Init and
// BwOptix_BuildMesh( BW_MESH_VIS, ... ) first), serialized with the face pass's
// own launches. blocked[i] = 1 when segment i hits the mesh. False when the
// launch fails (blocked is then meaningless). RunVisibility's own known-answer
// check of the GPU path goes through this.
bool BwVis_TraceSegmentsGpu( const std::vector<VisSegment> &segs, float gravity, std::vector<unsigned char> &blocked );
// A displacement mirror's window (#78) for face sample sp, reflected path crossing the
// reflection plane at q (mirEye: the mirrored eye, realEye: the real one), the
// displacement's render triangles stood in for by mins .. maxs grown by tol: pair 0 when
// the real eye's ray through q meets the bounds, a further pair when the segment
// sp -> mirEye does. Each pair is a sample leg and an eye leg (legs[2k], legs[2k + 1]),
// any pair with both clear a path. Returns the pairs (0: no window), -1 for a window with
// nothing to trace. Exposed for -selftest.
int BwVis_DispMirrorPairs( const dvec3 &mins, const dvec3 &maxs, double tol, const dvec3 &sp, const dvec3 &q,
						   const dvec3 &mirEye, const dvec3 &realEye, VisSegment legs[4] );

// A world brush that blocks sight: CONTENTS_SOLID without WINDOW / GRATE / TRANSLUCENT /
// water contents (bw_vis.cpp; bw_cubemap.cpp uses it for its rays).
bool OpaqueSolid( int brush );

// Ray helpers (bw_vis.cpp), exposed for -selftest.
// Does the open segment a -> b pass through the interior of the convex
// polytope (planes n . x <= d, already shrunk by the caller)? Touching is free.
bool SegmentCrossesConvex( const dvec3 &a, const dvec3 &b, const DPlane *planes, int numPlanes );
// The reflection of e in the plane (n, d).
dvec3 MirrorPoint( const dvec3 &e, const dvec3 &n, double d );

// Occluders are shrunk by this (the safe direction: more is visible).
static const double OCC_SHRINK = 1.0 / 32.0;
// The GPU fan's occluder meshes are shrunk this much more (bw_vis.cpp: float rounding
// there can then only mark more).
static const double FAN_GPU_SHRINK = 1.0 / 32.0;

// Seam fillers (#33, #40; bw_vis.cpp, the rule they keep is there), exposed for -selftest: for
// the real (unshrunk) occluders 'shapes', appends to 'fillers' the convexes (planes n . x <= d,
// used as they are, not shrunk again) that close the slot their OCC_SHRINK copies leave where
// two of them meet on a plane. Returns the contacts found (pairs of opposite sides on one
// plane); *pExtended, when given, gets how many fillers reach past an edge of their contact,
// and *pPairs the two shapes (indices into 'shapes') each filler joins, two ints per filler.
int BwVis_SeamFillers( const std::vector<ConvexShape> &shapes, std::vector< std::vector<DPlane> > &fillers, int *pExtended,
					   std::vector<int> *pPairs = NULL );

// The seam fillers a viewpoint in solid (#76) skips besides the occluders that hold it ('skip'):
// filler i is occluder first + i and joins occluders pairs[2i] and pairs[2i + 1]; every filler
// that joins a skipped occluder is appended to 'skip' (its half inside that brush would block a
// view the skip lets through).
void BwVis_SeamSkips( const std::vector<int> &pairs, int first, std::vector<int> &skip );

// Viewpoints in solid (#76), exposed for -selftest. A camera in a solid leaf sees
// through every brush side that faces another solid leaf (vbsp draws a side only
// toward open leaves) and through the occluders that hold it.
struct FaceSample
{
	dvec3 p;		// lifted off the face
	dvec3 n;		// the surface normal there
};
typedef bool ( *BwSolidAtFn )( const dvec3 &p, void *ctx );					// does p lie in a solid leaf?
typedef bool ( *BwSolidBlockedFn )( const dvec3 &v, const dvec3 &p, void *ctx );	// is viewer v -> p blocked?
struct BwSolidMirror
{
	dvec3 n;			// the mirror's plane (n . x = d), its front side +n
	double d;
	const DWinding *w;	// its polygon (CCW about n)
	double tol;			// how far outside the polygon Q may lie
};
// Does the convex occluder (planes n . x <= d, shrunk by OCC_SHRINK) block the
// segment from viewer v to p? Only when the segment crosses it and enters it, coming
// from v, from an open leaf: the leaf is probed OCC_SHRINK outside the real side
// (at most 1u back along a grazing segment); a viewer inside it probes its own point.
bool BwVis_SolidEntryBlocks( const dvec3 &v, const dvec3 &p, const DPlane *planes, int numPlanes, BwSolidAtFn solidAt,
							 void *ctx );
// BwVis_SolidEntryBlocks over the occluders listed in cand (occluder o is planes[ofs[o]]
// up to planes[ofs[o + 1]]), less those in skip (the ones that hold the camera).
bool BwVis_SolidSegmentBlocked( const dvec3 &v, const dvec3 &p, const DPlane *planes, const int *ofs, const int *cand,
								int numCand, const std::vector<int> &skip, BwSolidAtFn solidAt, void *ctx );
// Does camera cam see the face (plane fn . x = fd, front side +fn; twoSided: no
// back-face culls) at one of its samples? Directly when mirror is NULL, else by the
// path sample -> Q -> cam with Q on the mirror's plane inside its polygon (the camera
// must be in front of the mirror: the caller checks). Every segment goes through
// blocked( viewer, target ): the camera to a sample or Q, and Q to a sample. pairs
// counts the face samples tried (a mirrored sample's two legs count once); past maxPairs it gives up and answers true.
bool BwVis_SolidCamSees( const dvec3 &cam, const dvec3 &fn, double fd, bool twoSided, const FaceSample *samples,
						 int numSamples, const BwSolidMirror *mirror, BwSolidBlockedFn blocked, void *ctx, long long &pairs,
						 long long maxPairs );
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
