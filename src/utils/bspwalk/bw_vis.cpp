//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: Stage 5 (#14), face visibility tiered by proof strength.
//
//          Tier A: a face none of whose leaves (dleaffaces, all of them) is in
//          the PVS of any eye cluster. Reflective faces (the Water shader,
//          func_reflective_glass) that stay visible add their own clusters to
//          the eye clusters, until nothing changes: a reflection is a second
//          camera, so its view joins the proof.
//
//          Tier B (CPU reference): occluders are the opaque solid brushes only
//          (CONTENTS_SOLID without WINDOW / GRATE / TRANSLUCENT / water; vbsp
//          already strips SOLID from any brush with a see-through material),
//          shrunk 1/32u. An eye fan marks every face its rays hit first; then
//          each face still unmarked is sampled (<= 4u grid, edges inset 1/16u)
//          and traced to eyes in clusters whose PVS holds it, one eye per
//          angular bin per 32u tile, and through each visible reflective face
//          to mirrored eyes. A face no ray reaches is tier B.
//
//=============================================================================//

#include "cmdlib.h"
#include "bsplib.h"
#include "bspflags.h"
#include "threads.h"
#include "filesystem_tools.h"
#include "tier0/platform.h"
#include "tier1/KeyValues.h"
#include "tier1/utlbuffer.h"
#include "bw_vis.h"
#include "bw_eyestore.h"
#include "bw_optix.h"
#include <mutex>
#include "bw_hash.h"
#include "bw_util.h"
#include "bw_bvh.h"

#include <algorithm>
#include <atomic>
#include <limits.h>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <unordered_set>

// Occluders are shrunk by OCC_SHRINK (bw_vis.h; the safe direction: more is visible).
// A ray crosses an occluder only when it runs at least this far inside it.
static const double OCC_DEPTH = 1e-6;
// Face samples: edges inset by this; lifted off the face by this.
static const double FACE_INSET = 1.0 / 16.0;
static const double FACE_LIFT = 1.0 / 256.0;
// Samples are grouped in cubes this big for the angular eye choice.
static const double FACE_TILE = 32.0;
// At most this many samples on one face; a larger face gets a coarser grid.
static const int FACE_MAX_SAMPLES = 1 << 18;
// A fan ray marks the faces it crosses up to this far past its first occluder.
static const double FAN_PAST = 0.05;
static const double FAN_MAX_T = 131072.0;
// At most this many candidate eyes per face go to the binned tiles (CapPositions
// thins them beyond): many times the eyes the finest bin level can use. Past it the
// confirmation still pairs every candidate (the candidate sweep, RunSweep).
static const int FACE_MAX_CANDIDATES = 1 << 16;
// Segments per GPU launch of the face and mirror passes: the arc path's ARC_GPU_BATCH
// (bw_arcs.cpp), a memory bound only.
static const int VIS_GPU_BATCH = 8 * 1024 * 1024;
// A mirror hit point may lie this far outside the reflective face (safe side).
static const double MIRROR_TOL = 1e-2;

const char *VisReasonName( int why )
{
	static const char *s_Names[WHY_COUNT] = { "pvs", "rays", "fan", "ray", "mirror", "brush_entity", "no_tierb", "bad_lump",
											  "no_eye", "solid_view" };
	return ( why >= 0 && why < WHY_COUNT ) ? s_Names[why] : "?";
}

//-----------------------------------------------------------------------------
// Small geometry
//-----------------------------------------------------------------------------
bool SegmentCrossesConvex( const dvec3 &a, const dvec3 &b, const DPlane *planes, int numPlanes )
{
	dvec3 d = b - a;
	double t0 = 0.0, t1 = 1.0;
	for ( int k = 0; k < numPlanes; k++ )
	{
		double denom = DDot( planes[k].n, d );
		double num = planes[k].d - DDot( planes[k].n, a );
		if ( fabs( denom ) < 1e-300 )
		{
			if ( num <= 0 )
				return false;	// outside, or running along the face: touching is free
			continue;
		}
		double t = num / denom;
		if ( denom < 0 )
			t0 = MAX( t0, t );
		else
			t1 = MIN( t1, t );
		if ( t0 >= t1 )
			return false;
	}
	return ( t1 - t0 ) * DLength( d ) > OCC_DEPTH;
}

// The entry parameter of the ray o + t dir (t in [0, tMax]) into the convex,
// or -1 when it does not pass through it.
static double RayEnterConvex( const dvec3 &o, const dvec3 &dir, double tMax, const DPlane *planes, int numPlanes )
{
	double t0 = 0.0, t1 = tMax;
	for ( int k = 0; k < numPlanes; k++ )
	{
		double denom = DDot( planes[k].n, dir );
		double num = planes[k].d - DDot( planes[k].n, o );
		if ( fabs( denom ) < 1e-300 )
		{
			if ( num <= 0 )
				return -1;	// outside, or running along the face: touching is free
			continue;
		}
		double t = num / denom;
		if ( denom < 0 )
			t0 = MAX( t0, t );
		else
			t1 = MIN( t1, t );
		if ( t0 >= t1 )
			return -1;
	}
	return ( t1 - t0 ) > OCC_DEPTH ? t0 : -1;
}

dvec3 MirrorPoint( const dvec3 &e, const dvec3 &n, double d )
{
	return e - n * ( 2.0 * ( DDot( n, e ) - d ) );
}

dvec3 FibonacciDir( int i, int n )
{
	const double golden = 3.14159265358979323846 * ( 3.0 - sqrt( 5.0 ) );
	double z = 1.0 - ( 2.0 * i + 1.0 ) / n;
	double r = sqrt( MAX( 0.0, 1.0 - z * z ) );
	double phi = golden * i;
	return dvec3( r * cos( phi ), r * sin( phi ), z );
}

int OctBin( const dvec3 &dir, int bins )
{
	double s = fabs( dir.x ) + fabs( dir.y ) + fabs( dir.z );
	if ( !( s > 0 ) )
		return 0;
	double x = dir.x / s, y = dir.y / s;
	if ( dir.z < 0 )
	{
		double ox = ( 1.0 - fabs( y ) ) * ( x >= 0 ? 1.0 : -1.0 );
		double oy = ( 1.0 - fabs( x ) ) * ( y >= 0 ? 1.0 : -1.0 );
		x = ox;
		y = oy;
	}
	int u = (int)( ( x + 1.0 ) * 0.5 * bins ), v = (int)( ( y + 1.0 ) * 0.5 * bins );
	u = MAX( 0, MIN( bins - 1, u ) );
	v = MAX( 0, MIN( bins - 1, v ) );
	return u + v * bins;
}

//-----------------------------------------------------------------------------
// State
//-----------------------------------------------------------------------------
struct VisFace
{
	int model;
	dvec3 n;
	double d;
	DWinding w;				// world space
	const std::vector<dvec3> *tris;	// displacement render triangles (3 points each), CollisionWorld's
	dvec3 mins, maxs;
	bool reflective;
	bool twoSided;			// $nocull or water: no back-face culling
	bool disp;
	bool bad;				// lump data out of range (a rejected displacement): kept visible
};

static std::vector<VisFace> s_Faces;
static std::vector<int> s_FaceClOfs, s_FaceCl;		// face -> clusters (CSR, -1 excluded)
static std::vector<DPlane> s_OccPlanes;
static std::vector<int> s_OccOfs;
static Bvh s_OccBvh, s_FaceBvh;
// One-way occluders (#77): opaque world brushes with a nodraw side open to the map. The
// engine draws nothing on that side, so a segment entering there sees through the brush
// (its other sides face away); one entering through a drawn side from open space meets a
// drawn face (DirSegmentBlocks decides where). Their planes are shrunk like s_OccPlanes;
// s_DirDrawn says per plane whether its brush side is drawn. Not on the GPU face-pass mesh: a
// clear GPU segment is checked against them on the CPU (GpuAnyFree, SweepChunk); the GPU
// fan has them on a mesh of their own, decided per hit as NearestOccluder decides (FanOneWayMesh).
static std::vector<DPlane> s_DirPlanes;
static std::vector<unsigned char> s_DirDrawn;
static std::vector<int> s_DirOfs, s_DirSrc;			// s_DirSrc: the world brush of each
static Bvh s_DirBvh;
// Every world brush, unshrunk, when there are one-way occluders: a drawn side entered where
// another brush covers it emits no face there (DirSegmentBlocks' run walk and probe).
// s_BrushDrawn says per plane whether its brush side is drawn.
static std::vector<DPlane> s_BrushPlanes;
static std::vector<unsigned char> s_BrushDrawn;
static std::vector<int> s_BrushOfs;
// per brush of the table: opaque solid (OpaqueSolid), the only kind a run is made of
static std::vector<unsigned char> s_BrushRun;
static Bvh s_BrushBvh;
static inline bool HasDirOccluders()
{
	return s_DirOfs.size() > 1;
}
// Per face, set by the ray passes. The fan pass has many threads testing and marking the same
// faces, so every access is a relaxed atomic: a face only ever goes from 0 to a reason, and a
// thread that misses a fresh mark repeats a test.
static std::unique_ptr< std::atomic<unsigned char>[] > s_Visible;
static inline int VisibleGet( int f )
{
	return s_Visible[f].load( std::memory_order_relaxed );
}
static inline void VisibleSet( int f, int v )
{
	s_Visible[f].store( (unsigned char)v, std::memory_order_relaxed );
}
static int s_nClusters, s_RowBytes;
static std::vector<unsigned char> s_Pvs;			// numClusters rows
static const EyeVolume *s_pEyes;
static const VisOptions *s_pVisOpts;
// The eye set (#39): its store, and its eyes in buckets. With vis data a bucket is a cluster
// (bucket c = cluster c: its runs, EyeClusterFacts), without one an eye tile that holds samples
// (one run, the whole tile). Per bucket its runs (CSR), how many eyes, and their float box.
static EyeStore *s_pStore;
// #39 stage C: the refinement level the face pass runs at (0: the eye set's spacing). At a level
// s_pStore is that level's store and s_PassFaces the faces still hidden.
static int s_RefineLevel;
static std::vector<int> s_BucketOfs;
static std::vector<EyeRun> s_BucketRuns;
static std::vector<long long> s_BucketCount;
static std::vector<float> s_BucketBox;				// per bucket: its eyes' mins xyz, maxs xyz
static bool s_bBarePvs;				// vis data present: a bare cluster sees only its PVS (else it may see every face)
// real (not skybox-mapped) samples of viewpoint sources whose point is in a solid
// leaf (cluster -1, so no PVS row): a camera there draws every cluster, so a face it
// sees stays visible (SolidSeesFace, WHY_SOLIDVIEW). EyeVolume::solidEyes.
static std::vector<EyeSample> s_SolidEyes;
static std::vector< std::vector<int> > s_SolidSkip;	// per s_SolidEyes: the occluders that hold it
// At most this many (face sample, solid sample) segments per face and path in
// SolidSeesFace; past it the face stays visible untested.
static const long long SOLID_MAX_PAIRS = 1 << 20;
// An eye region counts as in front of a face only past this: the eye
// descent hands each side of a node plane the piece parts up to 1e-4 beyond it
// (bw_eyes EYE_SPLIT_EPS), so regions reach that far past faces on node planes.
// A sample counts as in front past 1e-6: an eye region reaching (1e-6, 1e-3]
// in front of a face whose samples are all behind it is accepted as hidden.
static const double REGION_MARGIN = 1e-3;
static const FanEye *s_FanEyes;						// EyeVolume::fanEyes
static int s_nFanEyes;
static std::vector<int> s_Reflective;				// reflective faces that stay visible to tier A
// per s_Reflective: the mirror as traced, a copy of its face; a displacement's (one entry per
// reflection plane it may show, #78) carries that plane as n, d (BwVis_DispMirrorPairs)
static std::vector<VisFace> s_MirrorFace;
// per s_Reflective: the eyes in front of it that may see it (past the cap the thinned set), gathered
// when a face still hidden in front of it needs them (GatherMirrorEyes)
static std::vector< std::vector<dvec3> > s_MirrorEyes;
static std::vector< std::vector<long long> > s_MirrorIds;	// ... and their eye ids
// per s_Reflective: MIRROR_BARE when no ray test can speak for it (a bare eye cluster may
// see it, or its eye region is untestable), else MIRROR_SOLID when a viewpoint sample in
// solid lies in front of it (the mirror loop then traces through it from those samples), else
// MIRROR_TESTED (MirrorStatusPass)
enum { MIRROR_TESTED = 0, MIRROR_BARE = 1, MIRROR_SOLID = 2 };
static std::vector<unsigned char> s_MirrorBare;
static std::vector<int> s_PassFaces;
// per face: its candidates (or, for a mirror's face, the mirror's) passed the cap and were thinned.
// Set from the main thread, after a count pass; atomic as it always was. Not reset between the
// refinement levels (#39 stage C): vis.capped_faces counts the faces capped at any level, the union.
static std::vector< std::atomic<unsigned char> > s_FaceCapped;
static std::vector<unsigned char> s_MirrorCapped;	// per s_Reflective: s_MirrorEyes is a thinned set
// Per-thread counters, a row to a thread on its own cache lines (as bw_walk.cpp's PaddedInt):
// rows written by different threads must not share one.
struct alignas( 64 ) VisCounters
{
	long long rays[3];			// fan rays; face and mirror: VisStats::rayUnit
	long long check[5];			// -vischeck: segments, gpu-only blocked, cpu-only blocked, failed launches, unexplained
	double checkMaxDist;		// largest grazing distance of a disagreement
	int checkTiles;				// tile levels whose any-free outcome differs
	int confirmHits;			// faces ConfirmPairs found visible
	int sweptFaces;				// faces a candidate sweep ran for (their candidates, or a mirror's, passed the cap)
	int sweptVisible;			// ... of which it found a clear ray: the thinned set alone would have hidden them
	int sweptMirror;			// sweeps through a capped mirror (SweepThroughMirror), one per face and mirror
	int cappedKept;				// faces past the cap kept visible untested (-eyeconfirm 0: nothing to sweep with)
	long long sweepEyes;		// eyes the candidate sweeps paired
	long long fanCheck[2];		// -vischeck with the GPU fan: rays compared, rays the GPU stopped short on
	double fanShort;			// the largest such shortfall

	VisCounters() { memset( this, 0, sizeof( *this ) ); }
};
static VisCounters s_Counters[MAX_TOOL_THREADS + 1];
// GPU face / mirror passes (the arc kernel on BW_MESH_VIS, segments as zero-gravity arcs)
static bool s_bVisGpu, s_bVisCheck, s_bVisSim;
static float s_VisG;
static std::mutex s_GpuMutex;
// Set by the first failed launch: every later batch goes to the CPU without retrying.
static std::atomic<bool> s_bVisGpuOff;
static std::atomic<int> s_CheckReported;				// disagreements named in a Warning

static inline bool PvsBit( int from, int to )
{
	return ( s_Pvs[(size_t)from * s_RowBytes + ( to >> 3 )] >> ( to & 7 ) ) & 1;
}

// The fan's PVS gate (#77): may a fan eye in cluster c see face fi at all? The engine draws
// a face only from a cluster whose PVS holds one of the face's clusters, which tier A and
// the face pass's candidate eyes already assume; a ray through a nodraw side can cross a
// face no such eye sees. No gate (true) for c < 0 (no cluster, or a skybox copy: the
// engine draws the 3D skybox with the sky_camera's PVS, not the copy's), without vis
// data, or for a face with no cluster. The GPU fan has the same gate (bw_fan.cu EyeMaySee).
static inline bool FanEyeMaySee( int c, int fi )
{
	if ( c < 0 || c >= s_nClusters || s_Pvs.empty() || s_FaceClOfs[fi] == s_FaceClOfs[fi + 1] )
		return true;
	for ( int k = s_FaceClOfs[fi]; k < s_FaceClOfs[fi + 1]; k++ )
	{
		if ( s_FaceCl[k] == c || PvsBit( c, s_FaceCl[k] ) )
			return true;
	}
	return false;
}

// The cluster FanEyeMaySee gates a fan eye by: -1 for a skybox copy.
static inline int FanGateCluster( const FanEye &e )
{
	return e.sky >= 0 ? -1 : e.cluster;
}

//-----------------------------------------------------------------------------
// -traceface: which pass marked a face, with the eye and the ray. A debugging
// aid: the passes append to a per-face log (any thread), RunVisibility prints it
// at the end. Fan rays are recorded by the CPU fan only (FanOne); the GPU fan
// marks faces without saying which ray did.
//-----------------------------------------------------------------------------
static std::vector<int> s_TraceSlot;				// per face: its log, or -1 (empty: no -traceface)
static std::vector<std::string> s_TraceLog;
static std::mutex s_TraceMutex;
struct TraceFanRay
{
	int face, eye, ray;		// the face, the eye sample, the ray of its fan
	dvec3 dir;
	double tOcc, tHit;		// the first occluder entry along the ray, and where it crosses the face (-1: displacement)
};
static std::vector<TraceFanRay> s_TraceFan;
// -traceface: an eye as the log and the JSON name it (#85): the fan's (id: its index in
// EyeVolume::fanEyes) or a face or mirror pass candidate's (id: its eye id)
struct TraceEyeInfo
{
	long long id;
	float x, y, z;
	int cluster, source, sky;
	int level;			// the refinement level whose store holds the eye (#39 stage C; the fan's: 0)
};
static std::vector<TraceEyeInfo> s_TraceSeen;		// per log: the eye the face was first seen from (id -1: none) (#85)
static std::vector<const char *> s_TraceSeenPass;	// ... and the pass that saw it
static std::vector<long long> s_TraceFanCount;		// per log: fan rays that crossed the face
static std::vector<long long> s_TraceFanGated;		// per log: of those, from eyes FanEyeMaySee turns away
static const int TRACE_KEEP_RAYS = 256;				// fan rays kept per face (the first few by eye are printed)
static const int TRACE_PRINT_RAYS = 4;
static const int TRACE_PRINT_BRUSHES = 12;			// brushes listed along one traced segment

static inline bool Traced( int fi )
{
	return !s_TraceSlot.empty() && s_TraceSlot[fi] >= 0;
}

static void TraceF( int fi, const char *fmt, ... )
{
	char buf[1024];
	va_list args;
	va_start( args, fmt );
	V_vsnprintf( buf, sizeof( buf ), fmt, args );
	va_end( args );
	std::lock_guard<std::mutex> lock( s_TraceMutex );
	s_TraceLog[s_TraceSlot[fi]] += buf;
}

static void TraceFanHit( int fi, int eye, int ray, const dvec3 &dir, double tOcc, double tHit, bool gated )
{
	std::lock_guard<std::mutex> lock( s_TraceMutex );
	int slot = s_TraceSlot[fi];
	if ( gated )
	{
		s_TraceFanGated[slot]++;
		return;
	}
	s_TraceFanCount[slot]++;
	if ( s_TraceFanCount[slot] > TRACE_KEEP_RAYS )
		return;
	TraceFanRay r;
	r.face = fi;
	r.eye = eye;
	r.ray = ray;
	r.dir = dir;
	r.tOcc = tOcc;
	r.tHit = tHit;
	s_TraceFan.push_back( r );
}

// -traceface: the first eye a traced face is seen from (#85; kept for the JSON).
static void TraceSeen( int fi, const char *pass, const TraceEyeInfo &eye )
{
	std::lock_guard<std::mutex> lock( s_TraceMutex );
	int slot = s_TraceSlot[fi];
	if ( s_TraceSeen[slot].id >= 0 )
		return;
	s_TraceSeen[slot] = eye;
	s_TraceSeenPass[slot] = pass;
}

static TraceEyeInfo FanTraceInfo( int eye )
{
	const FanEye &e = s_FanEyes[eye];
	TraceEyeInfo w = { eye, e.x, e.y, e.z, e.cluster, e.source, e.sky, 0 };
	return w;
}

// An eye of the store by its id (-traceface only: it pins the eye's tile for the lookup).
static TraceEyeInfo SampleTraceInfo( long long id )
{
	const int tile = EyeIdTile( id ), i = EyeIdIndex( id );
	const EyeTileData &d = s_pStore->Pin( tile );
	TraceEyeInfo w = { id, d.xyz[(size_t)i * 3], d.xyz[(size_t)i * 3 + 1], d.xyz[(size_t)i * 3 + 2], d.ClusterOf( i ), d.source[i],
					   s_pStore->TileSky( tile ), s_RefineLevel };
	s_pStore->Unpin( tile );
	return w;
}

//-----------------------------------------------------------------------------
// Occlusion queries
//-----------------------------------------------------------------------------
// The full occluders alone (what the GPU face-pass mesh holds).
static bool SegmentBlockedFull( const dvec3 &a, const dvec3 &b )
{
	if ( s_OccBvh.nodes.empty() )
		return false;
	dvec3 d = b - a;
	dvec3 inv = SafeInv( d );
	int stack[128];
	int sp = 0;
	stack[sp++] = 0;
	while ( sp )
	{
		const BvhNode &nd = s_OccBvh.nodes[stack[--sp]];
		if ( !SegBox( a, inv, 0.0, 1.0, nd.mins, nd.maxs ) )
			continue;
		if ( nd.left >= 0 )
		{
			if ( sp < 126 )
			{
				stack[sp++] = nd.left;
				stack[sp++] = nd.right;
			}
			continue;
		}
		for ( int i = nd.first; i < nd.first + nd.count; i++ )
		{
			int o = s_OccBvh.items[i];
			if ( SegmentCrossesConvex( a, b, &s_OccPlanes[s_OccOfs[o]], s_OccOfs[o + 1] - s_OccOfs[o] ) )
				return true;
		}
	}
	return false;
}

// Nearest full occluder entry along the ray, FAN_MAX_T if none.
static double NearestFullOccluder( const dvec3 &o, const dvec3 &dir )
{
	double best = FAN_MAX_T;
	if ( s_OccBvh.nodes.empty() )
		return best;
	dvec3 inv = SafeInv( dir );
	int stack[128];
	int sp = 0;
	stack[sp++] = 0;
	while ( sp )
	{
		const BvhNode &nd = s_OccBvh.nodes[stack[--sp]];
		if ( !SegBox( o, inv, 0.0, best, nd.mins, nd.maxs ) )
			continue;
		if ( nd.left >= 0 )
		{
			if ( sp < 126 )
			{
				stack[sp++] = nd.left;
				stack[sp++] = nd.right;
			}
			continue;
		}
		for ( int i = nd.first; i < nd.first + nd.count; i++ )
		{
			int k = s_OccBvh.items[i];
			double t = RayEnterConvex( o, dir, best, &s_OccPlanes[s_OccOfs[k]], s_OccOfs[k + 1] - s_OccOfs[k] );
			if ( t >= 0 && t < best )
				best = t;
		}
	}
	return best;
}

// Is p inside (or within 1e-3u of) any world brush, unshrunk?
static bool PointInAnyBrush( const dvec3 &p )
{
	if ( s_BrushBvh.nodes.empty() )
		return false;
	const double tol = 1e-3;
	int stack[128];
	int sp = 0;
	stack[sp++] = 0;
	while ( sp )
	{
		const BvhNode &nd = s_BrushBvh.nodes[stack[--sp]];
		if ( p.x < nd.mins.x - tol || p.y < nd.mins.y - tol || p.z < nd.mins.z - tol || p.x > nd.maxs.x + tol ||
			 p.y > nd.maxs.y + tol || p.z > nd.maxs.z + tol )
			continue;
		if ( nd.left >= 0 )
		{
			if ( sp < 126 )
			{
				stack[sp++] = nd.left;
				stack[sp++] = nd.right;
			}
			continue;
		}
		for ( int i = nd.first; i < nd.first + nd.count; i++ )
		{
			int b = s_BrushBvh.items[i];
			bool inside = true;
			for ( int k = s_BrushOfs[b]; k < s_BrushOfs[b + 1] && inside; k++ )
				inside = DDot( s_BrushPlanes[k].n, p ) <= s_BrushPlanes[k].d + tol;
			if ( inside )
				return true;
		}
	}
	return false;
}

// Where the ray v + t d first enters one-way occluder o's real (unshrunk) volume: the
// largest t over the planes it crosses inward, with the plane in *pPlane. *pDrawn is
// false when that side is nodraw, or when a nodraw side ties with it (an edge: the safe
// side). Returns -1 when v is inside or on the volume (a viewer there sees out through
// sides facing away from it) or the ray runs outside along a plane.
static double DirEntry( const dvec3 &v, const dvec3 &d, int o, bool *pDrawn, int *pPlane )
{
	const double len = DLength( d );
	const double tieT = len > 0 ? 1e-6 / len : 0.0;
	double t0 = -1e300;
	bool drawn = false;
	int plane = -1;
	for ( int k = s_DirOfs[o]; k < s_DirOfs[o + 1]; k++ )
	{
		const DPlane &pl = s_DirPlanes[k];
		double denom = DDot( pl.n, d );
		double num = pl.d + OCC_SHRINK - DDot( pl.n, v );	// >= 0: v on the inner side
		if ( fabs( denom ) < 1e-300 )
		{
			if ( num < 0 )
				return -1;
			continue;
		}
		if ( denom > 0 )
			continue;
		double t = num / denom;
		if ( t > t0 + tieT )
		{
			t0 = t;
			drawn = s_DirDrawn[k] != 0;
			plane = k;
		}
		else if ( t > t0 - tieT )
			drawn = drawn && s_DirDrawn[k];
	}
	if ( plane < 0 || t0 <= 0 )
		return -1;
	*pDrawn = drawn;
	*pPlane = plane;
	return t0;
}

// The most DirSegmentBlocks steps back along a grazing segment from where it enters solid.
static const double DIR_PROBE_BACK_MAX = 1.0;
// Brushes closer than this along a segment (abutting, up to float noise) are one solid run,
// and entry planes this close tie.
static const double DIR_RUN_GAP = 1e-3;
// The most brushes a run walk collects; more gives up (no block: the safe side).
static const int DIR_RUN_MAX_SPANS = 64;

// Where the line v + s u (s in length units, u unit) runs inside world brush b of the
// s_Brush table: [*pIn, *pOut]. False when it misses it.
static bool BrushSpan( const dvec3 &v, const dvec3 &u, int b, double *pIn, double *pOut )
{
	double in = -1e300, out = 1e300;
	for ( int k = s_BrushOfs[b]; k < s_BrushOfs[b + 1]; k++ )
	{
		const DPlane &pl = s_BrushPlanes[k];
		double c = DDot( pl.n, u );
		double num = pl.d - DDot( pl.n, v );
		if ( fabs( c ) < 1e-300 )
		{
			if ( num < 0 )
				return false;
			continue;
		}
		double s = num / c;
		if ( c < 0 )
			in = MAX( in, s );
		else
			out = MIN( out, s );
	}
	if ( in > out )
		return false;
	*pIn = in;
	*pOut = out;
	return true;
}

struct DirWhy	// -traceface: how DirSegmentBlocks decided
{
	bool crosses = false, viewerInSolid = false, overflow = false, drawn = false, inBrush = false;
	double t0 = -1, entry = 0, start = 0;
	int entering = 0, ties = 0, cluster = -1;
	dvec3 q;
};

// Does one-way occluder o block the view from v to p? Only when the segment runs more than
// OCC_DEPTH through its shrunk volume, and where it came into solid on its way there it
// came from open space through a drawn side. That entry is found by walking back from o's
// real entry through the world brushes the segment is inside without a gap (a run): the
// sides crossed where the run starts must all be drawn (a tied nodraw side is an edge: the
// safe side), and the point OCC_SHRINK outside them, back along the segment (at most
// DIR_PROBE_BACK_MAX), must lie in a leaf with a cluster and in no world brush. A run that
// starts at a nodraw side sees through every brush in it: vbsp emits no face inside solid,
// so a drawn side another brush covers shows nothing. Where the void lies beyond a side,
// vbsp emits no face either. A run that reaches back to the viewer means it is in solid,
// and sees out.
static bool DirSegmentBlocks( const dvec3 &v, const dvec3 &p, int o, DirWhy *why = NULL )
{
	DirWhy w;
	if ( !why )
		why = &w;
	why->crosses = SegmentCrossesConvex( v, p, &s_DirPlanes[s_DirOfs[o]], s_DirOfs[o + 1] - s_DirOfs[o] );
	if ( !why->crosses )
		return false;
	const dvec3 d = p - v;
	bool drawn;
	int plane;
	why->t0 = DirEntry( v, d, o, &drawn, &plane );
	if ( why->t0 < 0 )
		return false;
	const double len = DLength( d );
	const dvec3 u = d * ( 1.0 / len );
	const dvec3 inv = SafeInv( u );
	const double s0 = why->t0 * len;
	why->entry = s0;

	// the run: brushes met within 'reach' before s0, widened while the run reaches the window's edge
	int span[DIR_RUN_MAX_SPANS];
	double sIn[DIR_RUN_MAX_SPANS], sOut[DIR_RUN_MAX_SPANS];
	int n = 0;
	double start = s0;
	for ( double reach = DIR_PROBE_BACK_MAX;; reach *= 4 )
	{
		const double lo = MAX( 0.0, s0 - reach );
		n = 0;
		int stack[128];
		int sp = 0;
		stack[sp++] = 0;
		while ( sp )
		{
			const BvhNode &nd = s_BrushBvh.nodes[stack[--sp]];
			if ( !SegBox( v, inv, lo, s0 + DIR_RUN_GAP, nd.mins, nd.maxs ) )
				continue;
			if ( nd.left >= 0 )
			{
				if ( sp < 126 )
				{
					stack[sp++] = nd.left;
					stack[sp++] = nd.right;
				}
				continue;
			}
			for ( int i = nd.first; i < nd.first + nd.count; i++ )
			{
				double a, b;
				int k = s_BrushBvh.items[i];
				// a see-through brush (window, grate, water, ...) hides nothing behind it:
				// a run through it starts wherever the opaque brushes start
				if ( !s_BrushRun[k] )
					continue;
				if ( !BrushSpan( v, u, k, &a, &b ) || b < lo || a > s0 + DIR_RUN_GAP )
					continue;
				if ( n == DIR_RUN_MAX_SPANS )
				{
					why->overflow = true;
					return false;
				}
				span[n] = k;
				sIn[n] = a;
				sOut[n] = b;
				n++;
			}
		}
		start = s0;
		for ( bool grew = true; grew; )
		{
			grew = false;
			for ( int i = 0; i < n; i++ )
			{
				// a brush the segment is inside (not just touching) up to the run's start
				if ( sIn[i] < start - DIR_RUN_GAP && sOut[i] >= start - DIR_RUN_GAP && sOut[i] > sIn[i] )
				{
					start = sIn[i];
					grew = true;
				}
			}
		}
		if ( start <= DIR_RUN_GAP )
		{
			why->viewerInSolid = true;
			return false;
		}
		if ( lo <= 0 || start > lo + DIR_RUN_GAP )
			break;
	}
	why->start = start;

	// the sides crossed where the run starts, and the probe OCC_SHRINK outside each of
	// them: a probe barely outside an edge's other plane would land within
	// PointInAnyBrush's tolerance of a brush sharing that plane
	why->drawn = true;
	double back = 0;
	for ( int i = 0; i < n; i++ )
	{
		if ( fabs( sIn[i] - start ) >= DIR_RUN_GAP || sOut[i] < sIn[i] )
			continue;
		why->entering++;
		int k = span[i];
		for ( int j = s_BrushOfs[k]; j < s_BrushOfs[k + 1]; j++ )
		{
			const DPlane &pl = s_BrushPlanes[j];
			double c = -DDot( pl.n, u );
			if ( c <= 0 )
				continue;
			double sj = ( DDot( pl.n, v ) - pl.d ) / c;
			if ( start - sj < DIR_RUN_GAP )
			{
				why->ties++;
				why->drawn = why->drawn && s_BrushDrawn[j];
			}
			if ( start - sj < OCC_SHRINK )
				back = MAX( back, ( start - sj ) + OCC_SHRINK / c );
		}
	}
	if ( !why->drawn )
		return false;
	back = MIN( back, DIR_PROBE_BACK_MAX );
	why->q = back < start ? v + u * ( start - back ) : v;
	why->cluster = dleafs[PointLeaf( why->q )].cluster;
	if ( why->cluster < 0 )
		return false;
	why->inBrush = PointInAnyBrush( why->q );
	return !why->inBrush;
}

// Is the view from 'viewer' to p blocked by a one-way occluder?
static bool DirBlocked( const dvec3 &viewer, const dvec3 &p )
{
	if ( s_DirBvh.nodes.empty() )
		return false;
	// The answer is whether any occluder blocks, so the one that blocked this thread's last
	// segment can go first: neighbouring segments of a face's sweep mostly end at the same
	// wall (#114), and that saves the walk to it. The set tested is the same.
	thread_local int lastBlocker = -1;
	if ( lastBlocker >= 0 && lastBlocker < (int)s_DirOfs.size() - 1 && DirSegmentBlocks( viewer, p, lastBlocker ) )
		return true;
	dvec3 inv = SafeInv( p - viewer );
	int stack[128];
	int sp = 0;
	stack[sp++] = 0;
	while ( sp )
	{
		const BvhNode &nd = s_DirBvh.nodes[stack[--sp]];
		if ( !SegBox( viewer, inv, 0.0, 1.0, nd.mins, nd.maxs ) )
			continue;
		if ( nd.left >= 0 )
		{
			if ( sp < 126 )
			{
				stack[sp++] = nd.left;
				stack[sp++] = nd.right;
			}
			continue;
		}
		for ( int i = nd.first; i < nd.first + nd.count; i++ )
		{
			if ( DirSegmentBlocks( viewer, p, s_DirBvh.items[i] ) )
			{
				lastBlocker = s_DirBvh.items[i];
				return true;
			}
		}
	}
	return false;
}

// Is the segment a -> b blocked, seen from b? Every face-pass segment ends at its viewer:
// the eye, the real eye behind a mirror, or the mirror point for the leg to the face. The
// one-way occluders depend on that direction.
static bool SegmentBlocked( const dvec3 &a, const dvec3 &b )
{
	return SegmentBlockedFull( a, b ) || DirBlocked( b, a );
}

// Nearest occluder entry along the fan ray from eye o, FAN_MAX_T if none. A one-way
// occluder counts where the ray enters its real volume through a drawn side; unlike
// DirSegmentBlocks there is no run walk or probe (the GPU fan cannot make them). That
// stops a ray early only where a covered or void-facing side is entered, and lets one
// through only where a drawn side of another brush comes first (no stop there; the
// faces it marks are only more visible). The face pass then decides: the fan only ever
// marks faces visible.
static double NearestOccluder( const dvec3 &o, const dvec3 &dir )
{
	double best = NearestFullOccluder( o, dir );
	if ( s_DirBvh.nodes.empty() )
		return best;
	dvec3 inv = SafeInv( dir );
	int stack[128];
	int sp = 0;
	stack[sp++] = 0;
	while ( sp )
	{
		const BvhNode &nd = s_DirBvh.nodes[stack[--sp]];
		if ( !SegBox( o, inv, 0.0, best, nd.mins, nd.maxs ) )
			continue;
		if ( nd.left >= 0 )
		{
			if ( sp < 126 )
			{
				stack[sp++] = nd.left;
				stack[sp++] = nd.right;
			}
			continue;
		}
		for ( int i = nd.first; i < nd.first + nd.count; i++ )
		{
			int k = s_DirBvh.items[i];
			double t = RayEnterConvex( o, dir, best, &s_DirPlanes[s_DirOfs[k]], s_DirOfs[k + 1] - s_DirOfs[k] );
			if ( t < 0 || t >= best )
				continue;
			bool drawn;
			int plane;
			if ( DirEntry( o, dir, k, &drawn, &plane ) >= 0 && drawn )
				best = t;
		}
	}
	return best;
}

// Is p (on the plane with normal nrm) inside the convex winding w, within tol outward?
static bool InsideWinding( const DWinding &w, const dvec3 &nrm, const dvec3 &p, double tol )
{
	int n = (int)w.size();
	for ( int i = 0; i < n; i++ )
	{
		const dvec3 &a = w[i];
		const dvec3 &b = w[( i + 1 ) % n];
		dvec3 e = b - a;
		dvec3 m = DCross( e, nrm );	// outward when the winding is CCW about nrm
		double len = DLength( m );
		if ( len < 1e-12 )
			continue;
		if ( DDot( m, p - a ) > tol * len )
			return false;
	}
	return true;
}

// Is p (on the face's plane) inside the face's convex polygon, within tol outward?
static bool InsidePolygon( const VisFace &f, const dvec3 &p, double tol )
{
	return InsideWinding( f.w, f.n, p, tol );
}

static bool RayHitsTriangle( const dvec3 &o, const dvec3 &dir, double tMax, const dvec3 &a, const dvec3 &b,
							 const dvec3 &c )
{
	dvec3 e1 = b - a, e2 = c - a;
	dvec3 p = DCross( dir, e2 );
	double det = DDot( e1, p );
	if ( fabs( det ) < 1e-14 )
		return false;
	double inv = 1.0 / det;
	dvec3 s = o - a;
	double u = DDot( s, p ) * inv;
	if ( u < -1e-6 || u > 1 + 1e-6 )
		return false;
	dvec3 q = DCross( s, e1 );
	double v = DDot( dir, q ) * inv;
	if ( v < -1e-6 || u + v > 1 + 1e-6 )
		return false;
	double t = DDot( e2, q ) * inv;
	return t >= 0 && t <= tMax;
}

// Does the ray o + t dir, t in [0, tMax], cross face fi on its front side (either side of a
// two-sided face)? tHit: where, or -1 for a displacement.
static bool FanCrosses( int fi, const dvec3 &o, const dvec3 &dir, double tMax, double &tHit )
{
	const VisFace &f = s_Faces[fi];
	tHit = -1.0;
	if ( f.disp )
	{
		const std::vector<dvec3> &tris = *f.tris;
		for ( size_t k = 0; k + 2 < tris.size(); k += 3 )
		{
			if ( RayHitsTriangle( o, dir, tMax, tris[k], tris[k + 1], tris[k + 2] ) )
				return true;
		}
		return false;
	}
	double denom = DDot( f.n, dir );
	if ( denom >= 0 && !f.twoSided )
		return false;
	if ( fabs( denom ) < 1e-12 )
		return false;
	double t = ( f.d - DDot( f.n, o ) ) / denom;
	if ( t < 0 || t > tMax )
		return false;
	tHit = t;
	return InsidePolygon( f, o + dir * t, 1e-3 );
}

// Marks every face the ray o + t dir from eye sample 'eye' crosses for t in [0, tMax], front
// side (or either side of a two-sided face), that the eye's PVS holds (FanEyeMaySee). ray and
// tOcc only feed -traceface.
static void MarkFacesAlong( const dvec3 &o, const dvec3 &dir, double tMax, int eye, int ray, double tOcc )
{
	if ( s_FaceBvh.nodes.empty() )
		return;
	const int gateCl = FanGateCluster( s_FanEyes[eye] );
	dvec3 inv = SafeInv( dir );
	int stack[128];
	int sp = 0;
	stack[sp++] = 0;
	while ( sp )
	{
		const BvhNode &nd = s_FaceBvh.nodes[stack[--sp]];
		if ( !SegBox( o, inv, 0.0, tMax, nd.mins - dvec3( 0.01, 0.01, 0.01 ), nd.maxs + dvec3( 0.01, 0.01, 0.01 ) ) )
			continue;
		if ( nd.left >= 0 )
		{
			if ( sp < 126 )
			{
				stack[sp++] = nd.left;
				stack[sp++] = nd.right;
			}
			continue;
		}
		for ( int i = nd.first; i < nd.first + nd.count; i++ )
		{
			int fi = s_FaceBvh.items[i];
			// a traced face is tested past its first mark, so every ray that crosses it is counted
			const bool traced = Traced( fi );
			if ( VisibleGet( fi ) && !traced )
				continue;
			double tHit;
			if ( !FanCrosses( fi, o, dir, tMax, tHit ) )
				continue;
			const bool see = FanEyeMaySee( gateCl, fi );
			if ( see && !VisibleGet( fi ) )
				VisibleSet( fi, WHY_FAN + 1 );
			if ( traced )
				TraceFanHit( fi, eye, ray, dir, tOcc, tHit, !see );
		}
	}
}

//-----------------------------------------------------------------------------
// Materials: the Water shader and $nocull. Cached per texdata.
//-----------------------------------------------------------------------------
struct MatInfo
{
	bool water, nocull;
};

// The VMTs that could not be read, each once, in the order met.
struct VmtMisses
{
	std::unordered_set<std::string> seen;
	std::vector<std::string> names;

	void Add( const char *pszName )
	{
		if ( seen.insert( pszName ).second )
			names.push_back( pszName );
	}
};

// materials/<name>.vmt from the map's own pakfile (LoadBSPFile loaded it), else
// from the game's search paths. The engine mounts the pakfile ahead of the game
// paths, so a packed material wins over a stock one of the same name.
static bool ReadVmtFile( const char *pszPath, CUtlBuffer &buf )
{
	CUtlBuffer packed;
	if ( ReadFileFromPak( GetPakFile(), pszPath, false, packed ) )
	{
		// into the caller's text buffer, as the file system fills it (the pak read
		// leaves 'packed' a binary buffer, which KeyValues cannot parse as text)
		buf.Put( packed.Base(), packed.TellPut() );
		return true;
	}
	return g_pFullFileSystem && g_pFullFileSystem->ReadFile( pszPath, NULL, buf );
}

static bool ReadVmt( const char *pszName, int depth, MatInfo &mi, VmtMisses &misses )
{
	char path[MAX_PATH];
	Q_snprintf( path, sizeof( path ), "materials/%s.vmt", pszName );
	Q_FixSlashes( path, '/' );
	CUtlBuffer buf( 0, 0, CUtlBuffer::TEXT_BUFFER );
	if ( !ReadVmtFile( path, buf ) )
	{
		misses.Add( pszName );
		return false;
	}
	KeyValues *kv = new KeyValues( "vmt" );
	bool ok = kv->LoadFromBuffer( path, buf );
	if ( !ok )
		misses.Add( pszName );
	else
	{
		const char *shader = kv->GetName();
		if ( !Q_stricmp( shader, "patch" ) )
		{
			const char *inc = kv->GetString( "include", "" );
			if ( inc[0] && depth < 4 )
			{
				char base[MAX_PATH];
				Q_strncpy( base, inc, sizeof( base ) );
				// "materials/x.vmt" -> "x"
				const char *p = base;
				if ( !Q_strnicmp( p, "materials/", 10 ) || !Q_strnicmp( p, "materials\\", 10 ) )
					p += 10;
				char stripped[MAX_PATH];
				Q_StripExtension( p, stripped, sizeof( stripped ) );
				ReadVmt( stripped, depth + 1, mi, misses );
			}
			for ( const char *sect : { "insert", "replace" } )
			{
				KeyValues *s = kv->FindKey( sect );
				if ( s && s->GetInt( "$nocull", 0 ) )
					mi.nocull = true;
			}
		}
		else
		{
			if ( Q_stristr( shader, "water" ) )
				mi.water = true;
			if ( kv->GetInt( "$nocull", 0 ) )
				mi.nocull = true;
		}
	}
	kv->deleteThis();
	return ok;
}

static void LoadMaterials( std::vector<MatInfo> &mats )
{
	mats.assign( numtexdata, MatInfo() );
	VmtMisses misses;
	for ( int i = 0; i < numtexdata; i++ )
	{
		MatInfo mi = { false, false };
		ReadVmt( TexDataStringTable_GetString( dtexdata[i].nameStringTableID ), 0, mi, misses );
		mats[i] = mi;
	}
	if ( misses.names.empty() )
		return;
	std::string first;
	for ( size_t k = 0; k < misses.names.size() && k < 3; k++ )
		first += ( k ? ", " : "" ) + misses.names[k];
	Warning( "bspwalk: %d material file(s) could not be read from the game's search paths or the map's pakfile, or do not "
			 "parse (%s%s); a Water shader or $nocull on them is not seen, so their faces are two-sided or reflective only "
			 "where the texinfo says so\n",
			 (int)misses.names.size(), first.c_str(), misses.names.size() > 3 ? ", ..." : "" );
}

//-----------------------------------------------------------------------------
// Faces
//-----------------------------------------------------------------------------
static void BuildFaces( const CollisionWorld &world, VisStats &st )
{
	int nf = numfaces;
	// CollectDisplacements ran in Stage 1: a displacement missing from its list would be sampled as a flat quad
	Assert( world.dispRenderTris.size() == (size_t)g_dispinfo.Count() );
	s_Faces.assign( nf, VisFace() );
	std::vector<int> faceModel( nf, 0 );
	for ( int m = 1; m < nummodels; m++ )
	{
		for ( int k = 0; k < dmodels[m].numfaces; k++ )
		{
			int f = dmodels[m].firstface + k;
			if ( f >= 0 && f < nf )
				faceModel[f] = m;
		}
	}

	// model -> entity transform: func_reflective_glass and friends sit at the
	// entity's origin, turned by its angles (as their collision brushes are)
	std::vector<DTransform> modelXf( nummodels );
	std::vector<unsigned char> modelGlass( nummodels, 0 );
	for ( int e = 0; e < num_entities; e++ )
	{
		const char *model = EntValue( &entities[e], "model" );
		if ( model[0] != '*' )
			continue;
		int m = atoi( model + 1 );
		if ( m <= 0 || m >= nummodels )
			continue;
		BrushEntityTransform( &entities[e], modelXf[m] );
		if ( !Q_stricmp( EntValue( &entities[e], "classname" ), "func_reflective_glass" ) )
			modelGlass[m] = 1;
	}

	std::vector<MatInfo> mats;
	LoadMaterials( mats );

	for ( int fi = 0; fi < nf; fi++ )
	{
		const dface_t &df = dfaces[fi];
		VisFace &f = s_Faces[fi];
		f.model = faceModel[fi];
		const bool planeOk = df.planenum < numplanes;	// unsigned short
		f.n = planeOk ? dvec3( dplanes[df.planenum].normal.x, dplanes[df.planenum].normal.y, dplanes[df.planenum].normal.z )
					  : dvec3( 0, 0, 0 );
		f.d = planeOk ? dplanes[df.planenum].dist : 0.0;
		// the plane and winding are in model space (the identity for the world)
		const DTransform &xf = modelXf[f.model];
		const dvec3 n0 = f.n;
		f.n = dvec3( xf.m[0][0] * n0.x + xf.m[0][1] * n0.y + xf.m[0][2] * n0.z,
					 xf.m[1][0] * n0.x + xf.m[1][1] * n0.y + xf.m[1][2] * n0.z,
					 xf.m[2][0] * n0.x + xf.m[2][1] * n0.y + xf.m[2][2] * n0.z );
		f.d += DDot( f.n, xf.origin );
		// a face whose lump data is out of range (bad plane, DispFaceProblem rejects
		// its displacement, fewer than 3 edges, or a bad edge index) is never evaluated
		f.bad = !planeOk || ( df.dispinfo >= 0 && DispFaceProblem( &df ) ) || df.firstedge < 0 || df.numedges < 3 ||
				df.firstedge > numsurfedges - df.numedges;
		for ( int k = 0; k < df.numedges && !f.bad; k++ )
		{
			int se = dsurfedges[df.firstedge + k];
			// range-check before negating: -INT_MIN overflows
			int vi = ( se > -numedges && se < numedges ) ? dedges[se < 0 ? -se : se].v[se < 0 ? 1 : 0] : -1;
			if ( vi < 0 || vi >= numvertexes )
			{
				f.bad = true;
				break;
			}
			const dvertex_t &v = dvertexes[vi];
			f.w.push_back( xf.Apply( dvec3( v.point.x, v.point.y, v.point.z ) ) );
		}
		if ( f.bad )
			f.w.clear();
		f.disp = !f.bad && df.dispinfo >= 0 && df.dispinfo < (int)world.dispRenderTris.size() && !world.dispRenderTris[df.dispinfo].empty();
		if ( f.disp )
			f.tris = &world.dispRenderTris[df.dispinfo];
		// windings come out either way round: make them CCW about n
		if ( f.w.size() >= 3 )
		{
			dvec3 an( 0, 0, 0 );
			for ( size_t k = 1; k + 1 < f.w.size(); k++ )
				an += DCross( f.w[k] - f.w[0], f.w[k + 1] - f.w[0] );
			if ( DDot( an, f.n ) < 0 )
				std::reverse( f.w.begin(), f.w.end() );
		}
		f.mins = dvec3( 1e300, 1e300, 1e300 );
		f.maxs = dvec3( -1e300, -1e300, -1e300 );
		const std::vector<dvec3> &pts = f.disp ? *f.tris : f.w;
		for ( size_t k = 0; k < pts.size(); k++ )
		{
			f.mins = DMin( f.mins, pts[k] );
			f.maxs = DMax( f.maxs, pts[k] );
		}
		int ti = df.texinfo;
		bool warp = false;
		MatInfo mi = { false, false };
		if ( ti >= 0 && ti < texinfo.Count() )
		{
			warp = ( texinfo[ti].flags & SURF_WARP ) != 0;
			int td = texinfo[ti].texdata;
			if ( td >= 0 && td < (int)mats.size() )
				mi = mats[td];
		}
		f.reflective = mi.water || warp || modelGlass[f.model];
		f.twoSided = mi.nocull || mi.water || warp;
		st.faces++;
		if ( f.model )
			st.entityFaces++;
		else
			st.worldFaces++;
		st.dispFaces += f.disp;
		st.badFaces += f.bad && !f.model;	// a bad brush entity face reports brush_entity
		st.reflective += f.reflective;
	}
}

static void BoxClusters( const dvec3 &mins, const dvec3 &maxs, std::vector<unsigned char> &bits )
{
	std::vector<DPlane> planes;
	for ( int a = 0; a < 3; a++ )
	{
		DPlane p;
		p.n = dvec3( 0, 0, 0 );
		p.n[a] = 1;
		p.d = maxs[a] + 0.5;
		planes.push_back( p );
		p.n[a] = -1;
		p.d = -( mins[a] - 0.5 );
		planes.push_back( p );
	}
	EyePiece box;
	if ( EyePolyFromPlanes( planes, box ) )
		PieceClusters( box, bits );
}

// BuildFaceClusters' per-face work: the clusters a face's bounds touch, into its own
// output, so the faces run on any thread.
static const std::vector<int> *s_pBoundedFaces;
static std::vector< std::vector<int> > *s_pBoundedClusters;
static std::vector<unsigned char> s_BoundedBits[MAX_TOOL_THREADS + 1];

static void FaceBoundsOne( int iThread, int item )
{
	const VisFace &f = s_Faces[( *s_pBoundedFaces )[item]];
	std::vector<unsigned char> &bits = s_BoundedBits[iThread];
	bits.assign( s_nClusters, 0 );
	BoxClusters( f.mins, f.maxs, bits );
	std::vector<int> &out = ( *s_pBoundedClusters )[item];
	for ( int c = 0; c < s_nClusters; c++ )
	{
		if ( bits[c] )
			out.push_back( c );
	}
}

static void BuildFaceClusters( VisStats &st )
{
	double tStart = Plat_FloatTime();
	int nf = numfaces;
	int badClusters = 0;
	std::vector< std::pair<int, int> > pairs;
	for ( int l = 0; l < numleafs; l++ )
	{
		const dleaf_t &lf = dleafs[l];
		for ( int k = 0; k < lf.numleaffaces; k++ )
		{
			int idx = lf.firstleafface + k;
			if ( idx < 0 || idx >= numleaffaces )
				continue;
			int f = dleaffaces[idx];
			if ( f >= 0 && f < nf )
				pairs.push_back( std::make_pair( f, (int)lf.cluster ) );
		}
	}
	std::sort( pairs.begin(), pairs.end() );
	pairs.erase( std::unique( pairs.begin(), pairs.end() ), pairs.end() );

	std::vector<unsigned char> listed( nf, 0 );
	for ( size_t i = 0; i < pairs.size(); i++ )
		listed[pairs[i].first] = 1;
	// world faces in no leaf list (displacements, detail faces): the leaves
	// their bounds touch, a superset
	double tBounds = Plat_FloatTime();
	std::vector<int> bounded;
	for ( int f = 0; f < nf; f++ )
	{
		// a brush entity's faces sit in its submodel's leaves, which have no
		// cluster: a reflective one takes the clusters its bounds touch
		if ( s_Faces[f].model ? !s_Faces[f].reflective : listed[f] )
			continue;
		st.faceNoLeaf += !s_Faces[f].model;
		bounded.push_back( f );
	}
	std::vector< std::vector<int> > boundedCl( bounded.size() );
	s_pBoundedFaces = &bounded;
	s_pBoundedClusters = &boundedCl;
	RunThreadsBatched( (int)bounded.size(), FaceBoundsOne );
	s_pBoundedFaces = NULL;
	s_pBoundedClusters = NULL;
	for ( int t = 0; t <= MAX_TOOL_THREADS; t++ )
		std::vector<unsigned char>().swap( s_BoundedBits[t] );
	for ( size_t i = 0; i < bounded.size(); i++ )
	{
		for ( size_t k = 0; k < boundedCl[i].size(); k++ )
			pairs.push_back( std::make_pair( bounded[i], boundedCl[i][k] ) );
	}
	st.tFaceBounds = Plat_FloatTime() - tBounds;
	std::sort( pairs.begin(), pairs.end() );
	s_FaceClOfs.assign( nf + 1, 0 );
	s_FaceCl.clear();
	size_t k = 0;
	for ( int f = 0; f < nf; f++ )
	{
		s_FaceClOfs[f] = (int)s_FaceCl.size();
		for ( ; k < pairs.size() && pairs[k].first == f; k++ )
		{
			// -1 is a solid leaf; anything else outside the vis data's clusters would
			// index past the PVS rows and the eye cluster flags
			int c = pairs[k].second;
			if ( c >= 0 && c < s_nClusters )
				s_FaceCl.push_back( c );
			else if ( c != -1 )
				badClusters++;
		}
	}
	s_FaceClOfs[nf] = (int)s_FaceCl.size();
	if ( badClusters )
		Warning( "bspwalk: %d face-to-cluster entries name a cluster outside 0..%d (the leaf and vis lumps disagree); "
				 "skipped. A face left with no cluster is not called hidden by the PVS: the ray tier decides it\n",
				 badClusters, s_nClusters - 1 );
	st.tFaceClusters = Plat_FloatTime() - tStart;
}

//-----------------------------------------------------------------------------
// Occluders: opaque solid brushes of the world, and world-like func_brush.
//-----------------------------------------------------------------------------
bool OpaqueSolid( int b )
{
	int c = dbrushes[b].contents;
	return ( c & CONTENTS_SOLID ) &&
		   !( c & ( CONTENTS_WINDOW | CONTENTS_GRATE | CONTENTS_TRANSLUCENT | CONTENTS_WATER | CONTENTS_SLIME ) );
}

// Every non-bevel side nodraw (and none sky): the brush draws nothing, so what is
// behind it shows through.
static bool AllNodraw( int b )
{
	const dbrush_t &br = dbrushes[b];
	bool any = false;
	for ( int k = 0; k < br.numsides; k++ )
	{
		const dbrushside_t &s = dbrushsides[br.firstside + k];
		if ( BrushSideIsBevel( s.bevel ) )
			continue;
		int flags = ( s.texinfo >= 0 && s.texinfo < texinfo.Count() ) ? texinfo[s.texinfo].flags : 0;
		if ( !( flags & SURF_NODRAW ) || ( flags & SURF_SKY ) )
			return false;
		any = true;
	}
	return any;
}

// A nodraw side (and not sky) of a brush: vbsp emits no face for it.
static bool SideNodraw( const dbrushside_t &s )
{
	int flags = ( s.texinfo >= 0 && s.texinfo < texinfo.Count() ) ? texinfo[s.texinfo].flags : 0;
	return ( flags & SURF_NODRAW ) && !( flags & SURF_SKY );
}

static bool AnyNodraw( int b )
{
	const dbrush_t &br = dbrushes[b];
	for ( int k = 0; k < br.numsides; k++ )
	{
		const dbrushside_t &s = dbrushsides[br.firstside + k];
		if ( !BrushSideIsBevel( s.bevel ) && SideNodraw( s ) )
			return true;
	}
	return false;
}

// Does any part of the winding lie in a world leaf with a cluster (inside the map)?
static bool WindingInMap( int node, const DWinding &w )
{
	while ( node >= 0 )
	{
		const dnode_t &dn = dnodes[node];
		const dplane_t &pl = dplanes[dn.planenum];
		const dvec3 n( pl.normal );
		double lo = 1e300, hi = -1e300;
		for ( size_t i = 0; i < w.size(); i++ )
		{
			double d = DDot( n, w[i] ) - pl.dist;
			lo = MIN( lo, d );
			hi = MAX( hi, d );
		}
		const double eps = 1e-3;
		if ( lo >= -eps )
			node = dn.children[0];	// in front (or on the plane)
		else if ( hi <= eps )
			node = dn.children[1];
		else
		{
			DWinding front( w ), back( w );
			DChopWindingInPlace( front, n, pl.dist, eps );
			DChopWindingInPlace( back, -n, -pl.dist, eps );
			if ( front.size() >= 3 && WindingInMap( dn.children[0], front ) )
				return true;
			return back.size() >= 3 && WindingInMap( dn.children[1], back );
		}
	}
	return dleafs[-1 - node].cluster >= 0;
}

// A world brush with a nodraw side that borders the map's open space: a ray can
// enter the brush through that side, where the engine draws nothing, and the
// drawn side it leaves by faces away from the eye (back-face culled). Sides
// against another brush or the void are not open (they land in a solid leaf,
// cluster -1, 1/8u out).
static bool OpenNodrawSide( int b )
{
	const dbrush_t &br = dbrushes[b];
	DPlane planes[256];
	bool nodraw[256];
	int n = 0;
	for ( int k = 0; k < br.numsides; k++ )
	{
		const dbrushside_t &s = dbrushsides[br.firstside + k];
		if ( BrushSideIsBevel( s.bevel ) )
			continue;
		if ( n == ARRAYSIZE( planes ) )
			return true;					// too many sides to test: assume open (a one-way occluder at most)
		const dplane_t &pl = dplanes[s.planenum];
		planes[n].n = dvec3( pl.normal );
		planes[n].d = pl.dist;
		nodraw[n] = SideNodraw( s );
		n++;
	}
	DWinding w;
	for ( int i = 0; i < n; i++ )
	{
		if ( !nodraw[i] )
			continue;
		DBaseWindingForPlane( planes[i].n, planes[i].d, 131072.0, w );
		for ( int j = 0; j < n && w.size() >= 3; j++ )
		{
			// a second side on the same plane would chop the whole winding away
			// (no point strictly behind it): skip it
			if ( j == i || ( DDot( planes[i].n, planes[j].n ) > 1.0 - 1e-9 && fabs( planes[i].d - planes[j].d ) < 1e-3 ) )
				continue;
			DChopWindingInPlace( w, -planes[j].n, -planes[j].d, 1e-6 );
		}
		if ( w.size() < 3 )
			continue;
		for ( size_t k = 0; k < w.size(); k++ )
			w[k] += planes[i].n * 0.125;
		if ( WindingInMap( dmodels[0].headnode, w ) )
			return true;
	}
	return false;
}

// The source of each occluder (-traceface): the world brush index, -1 - the
// CollisionWorld convex of a func_brush, or OCC_SRC_SEAM for a seam filler.
static std::vector<int> s_OccSrc;
static const int OCC_SRC_SEAM = INT_MIN;
// The seam fillers are occluders s_FirstSeam on; filler i joins occluders s_SeamPairs[2i] and
// s_SeamPairs[2i + 1].
static int s_FirstSeam;
static std::vector<int> s_SeamPairs;

static void AddOccluder( const ConvexShape &shape, std::vector<dvec3> &mins, std::vector<dvec3> &maxs, int src )
{
	s_OccSrc.push_back( src );
	for ( size_t f = 0; f < shape.faces.size(); f++ )
	{
		DPlane p;
		p.n = shape.faceNormals[f];
		double d = -1e300;
		for ( size_t k = 0; k < shape.faces[f].size(); k++ )
			d = MAX( d, DDot( p.n, shape.verts[shape.faces[f][k]] ) );
		p.d = d - OCC_SHRINK;
		s_OccPlanes.push_back( p );
	}
	s_OccOfs.push_back( (int)s_OccPlanes.size() );
	mins.push_back( shape.mins );
	maxs.push_back( shape.maxs );
}

// A brush side the engine draws on its plane: not nodraw, hint or skip (sky is drawn),
// and not translucent. Displacements need no test: vbsp drops a brush with a displacement
// side from the brush lump (map.cpp, HasDispInfo), and it never fills dbrushside_t's
// dispinfo (0 on every side), so that field cannot be read.
static bool SideDrawn( const dbrushside_t &s )
{
	int flags = ( s.texinfo >= 0 && s.texinfo < texinfo.Count() ) ? texinfo[s.texinfo].flags : 0;
	if ( flags & SURF_SKY )
		return true;
	return !( flags & ( SURF_NODRAW | SURF_HINT | SURF_SKIP | SURF_TRANS ) );
}

// Is brush b's shape plane n . x = d drawn? Only when every brush side on it is (a plane
// no side matches counts as nodraw: the safe side).
static bool ShapePlaneDrawn( int b, const dvec3 &n, double d )
{
	const dbrush_t &br = dbrushes[b];
	bool any = false, drawn = true;
	for ( int k = 0; k < br.numsides; k++ )
	{
		const dbrushside_t &bs = dbrushsides[br.firstside + k];
		if ( BrushSideIsBevel( bs.bevel ) )
			continue;
		const dplane_t &pl = dplanes[bs.planenum];
		if ( DDot( dvec3( pl.normal ), n ) > 1.0 - 1e-6 && fabs( pl.dist - d ) < 1e-2 )
		{
			any = true;
			drawn = drawn && SideDrawn( bs );
		}
	}
	return any && drawn;
}

// A one-way occluder (s_DirPlanes): the shape's planes shrunk as AddOccluder does, each
// with its ShapePlaneDrawn.
static void AddDirOccluder( int b, const ConvexShape &shape, std::vector<dvec3> &mins, std::vector<dvec3> &maxs )
{
	s_DirSrc.push_back( b );
	for ( size_t f = 0; f < shape.faces.size(); f++ )
	{
		DPlane p;
		p.n = shape.faceNormals[f];
		double d = -1e300;
		for ( size_t k = 0; k < shape.faces[f].size(); k++ )
			d = MAX( d, DDot( p.n, shape.verts[shape.faces[f][k]] ) );
		s_DirDrawn.push_back( ShapePlaneDrawn( b, p.n, d ) );
		p.d = d - OCC_SHRINK;
		s_DirPlanes.push_back( p );
	}
	s_DirOfs.push_back( (int)s_DirPlanes.size() );
	mins.push_back( shape.mins );
	maxs.push_back( shape.maxs );
}

//-----------------------------------------------------------------------------
// Seam fillers (#33, #40)
//
// Each occluder is shrunk on its own by OCC_SHRINK, so where two opaque brushes meet on a
// plane (a wall built of two brushes, a block standing on a floor or against a wall) their
// shrunk copies leave a 2 * OCC_SHRINK slot along it, and a segment lying in that slot passes
// through solid: fan rays hundreds of units long, and face rays grazing the face's own plane
// under a block that stands on it. A seam filler closes the slot: for two occluders with
// opposite sides on one plane, a prism over the region where both sides meet (inset from its
// edges), reaching SEAM_HALF into both brushes, past their shrunk copies.
//
// The rule every filler keeps (the safety invariant): it lies inside the union of the real,
// unshrunk occluder brushes, and at least OCC_SHRINK (less SEAM_TOL of rounding) from every
// point of that union's surface that faces air. A segment that enters a filler therefore runs
// through solid at least that deep and was never clear, while a segment that only touches or
// grazes an exposed surface (within OCC_SHRINK of it, what the shrunk copies already let
// through) never reaches one and stays as clear as before. It is checked, not assumed: the
// region R the filler stands on is clipped so that the column over each point of R, reaching
// h + OCC_SHRINK below the plane, lies inside the brush below, and the one reaching as far
// above inside the brush above (SeamColumn; then every vertex is tested against every plane of
// both brushes). The filler is R inset by OCC_SHRINK + SEAM_SLACK and h thick on each side, so a
// ball of radius OCC_SHRINK around any of its points stays inside those columns. Where the plane
// runs on past an edge of R into solid on both sides (a third brush below, a fourth above: a T
// or a cross of brushes), R is pushed out over that edge only after the piece it gains is shown
// to lie in one brush below and one above (SeamEdgePiece), so neighbouring fillers overlap
// there instead of leaving a tunnel along the edge. Anything the checks cannot prove (sloped
// neighbours that leave no room, a third brush that is not one convex on each side, sides whose
// planes do not match within SEAM_GAP) gets no filler and keeps its leak: the safe side.
//-----------------------------------------------------------------------------
// Opposite sides are one contact when the second lies at most SEAM_GAP beyond the first (float
// noise: abutting brushes share one BSP plane) or at most 2 * OCC_SHRINK into it (overlapping
// brushes whose shrunk copies still leave a slot).
static const double SEAM_GAP = 1e-4;
// Containment checks accept a point this far outside a plane (vertex rounding).
static const double SEAM_TOL = 1e-6;
// A filler's region is inset this much more than OCC_SHRINK from where it is proven, so
// rounding of the region's vertices cannot bring it nearer an exposed surface.
static const double SEAM_SLACK = 1.0 / 1024.0;
// Half-thicknesses tried, thickest first; a pass whose region is empty or whose column a
// brush plane cuts falls through to the next. 4 * OCC_SHRINK still covers the slot of the GPU
// fan's own copies (shrunk FAN_GPU_SHRINK more, filler included: a slot of 2 * (OCC_SHRINK +
// FAN_GPU_SHRINK) against 2 * (4 * OCC_SHRINK - FAN_GPU_SHRINK)); 2 * OCC_SHRINK, for brushes
// too thin for that, closes the CPU's slot only (the GPU fan's mesh then keeps a slot there,
// which only marks more).
static const double SEAM_HALF[2] = { 4.0 * OCC_SHRINK, 2.0 * OCC_SHRINK };
// How far an edge of a filler's region is pushed out where the plane runs on into solid on
// both sides: the filler then reaches 2 * OCC_SHRINK + SEAM_SLACK past the edge (OCC_SHRINK +
// SEAM_SLACK past it after the GPU fan's further shrink), past the neighbouring fillers' own
// inset, so they overlap.
static const double SEAM_EXTEND = 3.0 * OCC_SHRINK + 2.0 * SEAM_SLACK;
static const double SEAM_BASE = 131072.0;

// A line in the contact plane: t . x <= o (t unit, perpendicular to the plane's normal).
struct SeamLine
{
	dvec3 t;
	double o;
};

struct SeamBrush
{
	std::vector<DPlane> planes;		// real (unshrunk), one per face
	dvec3 mins, maxs;
};

// The constraints brush 'planes' puts on the contact plane n . x = c for the column from each
// point x to x + sgn * H * n (sgn -1: the brush below the plane, +1: above) to lie inside it.
// For a plane of normal m = n_k . n, x + s sgn n (s in [0, H]) is inside when n_k . x <=
// d_k - max(0, sgn m H); over the contact plane n_k . x = t_k . x + m c. Planes along n (no
// line) are left to SeamColumnFits.
static void SeamColumn( const std::vector<DPlane> &planes, const dvec3 &n, double c, double H, double sgn,
						std::vector<SeamLine> &lines )
{
	for ( size_t k = 0; k < planes.size(); k++ )
	{
		const DPlane &p = planes[k];
		double m = DDot( p.n, n );
		dvec3 t = p.n - n * m;
		double len = DLength( t );
		if ( len < 1e-12 )
			continue;
		SeamLine L;
		L.t = t * ( 1.0 / len );
		L.o = ( p.d - MAX( 0.0, sgn * m * H ) - m * c ) / len;
		lines.push_back( L );
	}
}

// Does the column from every point of w (on the contact plane) to it + sgn * H * n lie inside
// the brush? Both ends of every vertex's column against every plane: the region is convex
// and each constraint linear, so that is all of it.
static bool SeamColumnFits( const SeamBrush &b, const DWinding &w, const dvec3 &n, double H, double sgn )
{
	for ( size_t i = 0; i < w.size(); i++ )
	{
		const dvec3 top = w[i] + n * ( sgn * H );
		for ( size_t k = 0; k < b.planes.size(); k++ )
		{
			if ( DDot( b.planes[k].n, w[i] ) > b.planes[k].d + SEAM_TOL || DDot( b.planes[k].n, top ) > b.planes[k].d + SEAM_TOL )
				return false;
		}
	}
	return true;
}

// The polygon on the plane n . x = c inside every line moved out by off[j] (off NULL: none).
static bool SeamRegion( const dvec3 &n, double c, const std::vector<SeamLine> &lines, const double *off, double shift,
						DWinding &w )
{
	DBaseWindingForPlane( n, c, SEAM_BASE, w );
	for ( size_t j = 0; j < lines.size() && w.size() >= 3; j++ )
		DChopWindingInPlace( w, -lines[j].t, -( lines[j].o + ( off ? off[j] : 0.0 ) + shift ), 0.0 );
	return w.size() >= 3 && DWindingArea( w ) > 1e-9;
}

// Is there one brush whose planes hold the column from every point of w to it + sgn * H * n?
static bool SeamCovered( const std::vector<SeamBrush> &brushes, const Bvh &bvh, const DWinding &w, const dvec3 &n, double H,
						 double sgn )
{
	dvec3 lo( 1e300, 1e300, 1e300 ), hi( -1e300, -1e300, -1e300 );
	for ( size_t i = 0; i < w.size(); i++ )
	{
		const dvec3 top = w[i] + n * ( sgn * H );
		lo = DMin( lo, DMin( w[i], top ) );
		hi = DMax( hi, DMax( w[i], top ) );
	}
	if ( bvh.nodes.empty() )
		return false;
	int stack[128];
	int sp = 0;
	stack[sp++] = 0;
	while ( sp )
	{
		const BvhNode &nd = bvh.nodes[stack[--sp]];
		// a brush holding the box contains it: its box does too
		if ( nd.mins.x > lo.x + SEAM_TOL || nd.mins.y > lo.y + SEAM_TOL || nd.mins.z > lo.z + SEAM_TOL ||
			 nd.maxs.x < hi.x - SEAM_TOL || nd.maxs.y < hi.y - SEAM_TOL || nd.maxs.z < hi.z - SEAM_TOL )
			continue;
		if ( nd.left >= 0 )
		{
			if ( sp < 126 )
			{
				stack[sp++] = nd.left;
				stack[sp++] = nd.right;
			}
			continue;
		}
		for ( int i = nd.first; i < nd.first + nd.count; i++ )
		{
			if ( SeamColumnFits( brushes[bvh.items[i]], w, n, H, sgn ) )
				return true;
		}
	}
	return false;
}

// The filler for brushes a (below: its side faces +n, on n . x = ca) and b (above: its side
// faces -n, on n . x = cb), or false when none can be proven (see the rule above).
static bool BuildSeamFiller( const std::vector<SeamBrush> &brushes, const Bvh &bvh, int a, int b, const dvec3 &n, double ca,
							 double cb, std::vector<DPlane> &out, bool *pExtended )
{
	const double c = 0.5 * ( ca + cb );
	std::vector<SeamLine> lines;
	std::vector<double> ext;
	DWinding w, piece;
	for ( int pass = 0; pass < (int)ARRAYSIZE( SEAM_HALF ); pass++ )
	{
		const double h = SEAM_HALF[pass], H = h + OCC_SHRINK;
		lines.clear();
		SeamColumn( brushes[a].planes, n, c, H, -1.0, lines );
		SeamColumn( brushes[b].planes, n, c, H, 1.0, lines );
		// the sides do not meet at this thickness. A thinner column can still give a region:
		// a sloped side of either brush moves its line out as H shrinks (#115), so try thinner;
		// every later proof runs again on what that pass finds.
		if ( !SeamRegion( n, c, lines, NULL, 0.0, w ) )
			continue;
		if ( !SeamColumnFits( brushes[a], w, n, H, -1.0 ) || !SeamColumnFits( brushes[b], w, n, H, 1.0 ) )
			continue;		// a plane along n cuts the column (a brush thinner than H): try thinner
		// Push out each edge where the plane runs on into solid on both sides, one line at a
		// time: what the region gains is the part of it past that line's first place, proven
		// to lie in one brush below and one above before it is kept. A line that gains nothing
		// past rounding (another line makes the same edge, or it makes none) moves freely.
		ext.assign( lines.size(), 0.0 );
		bool extended = false;
		for ( size_t j = 0; j < lines.size(); j++ )
		{
			ext[j] = SEAM_EXTEND;
			if ( !SeamRegion( n, c, lines, ext.data(), 0.0, piece ) )
			{
				ext[j] = 0.0;
				continue;
			}
			DChopWindingInPlace( piece, lines[j].t, lines[j].o, 0.0 );	// the part past the edge
			double beyond = 0.0;
			for ( size_t i = 0; i < piece.size(); i++ )
				beyond = MAX( beyond, DDot( lines[j].t, piece[i] ) - lines[j].o );
			if ( piece.size() < 3 || beyond <= SEAM_TOL )
				continue;
			if ( SeamCovered( brushes, bvh, piece, n, H, -1.0 ) && SeamCovered( brushes, bvh, piece, n, H, 1.0 ) )
				extended = true;
			else
				ext[j] = 0.0;
		}
		// the filler: the proven region inset by OCC_SHRINK + SEAM_SLACK, h either side. A
		// region too small for the inset ends the pair (no thinner pass is tried for it: that
		// only gives fewer fillers, so fewer hidden faces)
		if ( !SeamRegion( n, c, lines, ext.data(), -( OCC_SHRINK + SEAM_SLACK ), w ) )
			return false;
		out.clear();
		for ( size_t j = 0; j < lines.size(); j++ )
		{
			// the lines the filler's polygon lies on (a line no vertex is near cuts nothing;
			// the loose test only ever keeps one more)
			const double o = lines[j].o + ext[j] - ( OCC_SHRINK + SEAM_SLACK );
			bool on = false;
			for ( size_t i = 0; i < w.size() && !on; i++ )
				on = DDot( lines[j].t, w[i] ) - o >= -1e-3;
			if ( !on )
				continue;
			DPlane p;
			p.n = lines[j].t;
			p.d = o;
			out.push_back( p );
		}
		DPlane cap;
		cap.n = n;
		cap.d = c + h;
		out.push_back( cap );
		cap.n = -n;
		cap.d = -( c - h );
		out.push_back( cap );
		*pExtended = extended;
		return out.size() >= 5;
	}
	return false;
}

// One face of an occluder, for the contact search.
struct SeamFace
{
	int brush;
	dvec3 n;		// the canonical normal of its plane (the face's own, or its opposite)
	double c;		// its plane: n . x = c
	bool up;		// its own normal is n (the brush lies below the plane)
	dvec3 mins, maxs;
};

int BwVis_SeamFillers( const std::vector<ConvexShape> &shapes, std::vector< std::vector<DPlane> > &fillers, int *pExtended,
					   std::vector<int> *pPairs )
{
	std::vector<SeamBrush> brushes( shapes.size() );
	std::vector<SeamFace> faces;
	std::vector<dvec3> mins, maxs;
	for ( size_t i = 0; i < shapes.size(); i++ )
	{
		const ConvexShape &s = shapes[i];
		SeamBrush &b = brushes[i];
		b.mins = s.mins;
		b.maxs = s.maxs;
		mins.push_back( s.mins );
		maxs.push_back( s.maxs );
		for ( size_t f = 0; f < s.faces.size(); f++ )
		{
			DPlane p;
			p.n = s.faceNormals[f];
			p.d = -1e300;
			SeamFace sf;
			sf.mins = dvec3( 1e300, 1e300, 1e300 );
			sf.maxs = dvec3( -1e300, -1e300, -1e300 );
			for ( size_t k = 0; k < s.faces[f].size(); k++ )
			{
				const dvec3 &v = s.verts[s.faces[f][k]];
				p.d = MAX( p.d, DDot( p.n, v ) );
				sf.mins = DMin( sf.mins, v );
				sf.maxs = DMax( sf.maxs, v );
			}
			b.planes.push_back( p );
			// canonical facing: the largest component positive
			int ax = fabs( p.n.x ) >= fabs( p.n.y ) && fabs( p.n.x ) >= fabs( p.n.z ) ? 0 : fabs( p.n.y ) >= fabs( p.n.z ) ? 1 : 2;
			sf.up = p.n[ax] > 0;
			sf.n = sf.up ? p.n : -p.n;
			sf.c = sf.up ? p.d : -p.d;
			sf.brush = (int)i;
			faces.push_back( sf );
		}
	}
	Bvh bvh;
	bvh.Build( mins, maxs );

	// bucket by exact normal (abutting brushes share one BSP plane, so their normals are exact
	// opposites) and plane distance in quarter units; a contact is looked up in its own bucket
	// and the next ones (a contact spans at most 2 * OCC_SHRINK + SEAM_GAP)
	struct Key
	{
		double nx, ny, nz;
		long long q;
		bool operator<( const Key &o ) const
		{
			if ( nx != o.nx )
				return nx < o.nx;
			if ( ny != o.ny )
				return ny < o.ny;
			if ( nz != o.nz )
				return nz < o.nz;
			return q < o.q;
		}
	};
	const double SEAM_BUCKET = 0.25;
	std::map< Key, std::vector<int> > buckets;
	for ( size_t i = 0; i < faces.size(); i++ )
	{
		Key k = { faces[i].n.x, faces[i].n.y, faces[i].n.z, (long long)floor( faces[i].c / SEAM_BUCKET ) };
		buckets[k].push_back( (int)i );
	}
	int contacts = 0, extended = 0;
	std::vector<DPlane> planes;
	for ( std::map< Key, std::vector<int> >::const_iterator it = buckets.begin(); it != buckets.end(); ++it )
	{
		for ( size_t u = 0; u < it->second.size(); u++ )
		{
			const SeamFace &fa = faces[it->second[u]];
			if ( !fa.up )
				continue;
			for ( long long dq = -1; dq <= 1; dq++ )
			{
				Key k = it->first;
				k.q += dq;
				std::map< Key, std::vector<int> >::const_iterator jt = buckets.find( k );
				if ( jt == buckets.end() )
					continue;
				for ( size_t v = 0; v < jt->second.size(); v++ )
				{
					const SeamFace &fb = faces[jt->second[v]];
					if ( fb.up || fb.brush == fa.brush )
						continue;
					// b's side at most SEAM_GAP beyond a's, at most 2 * OCC_SHRINK into it
					const double gap = fb.c - fa.c;
					if ( gap > SEAM_GAP || gap < -2.0 * OCC_SHRINK )
						continue;
					const double pad = 2.0 * OCC_SHRINK + SEAM_GAP;
					if ( fa.mins.x > fb.maxs.x + pad || fb.mins.x > fa.maxs.x + pad || fa.mins.y > fb.maxs.y + pad ||
						 fb.mins.y > fa.maxs.y + pad || fa.mins.z > fb.maxs.z + pad || fb.mins.z > fa.maxs.z + pad )
						continue;
					contacts++;
					bool ext = false;
					if ( BuildSeamFiller( brushes, bvh, fa.brush, fb.brush, fa.n, fa.c, fb.c, planes, &ext ) )
					{
						fillers.push_back( planes );
						extended += ext;
						if ( pPairs )
						{
							pPairs->push_back( fa.brush );
							pPairs->push_back( fb.brush );
						}
					}
				}
			}
		}
	}
	if ( pExtended )
		*pExtended = extended;
	return contacts;
}

void BwVis_SeamSkips( const std::vector<int> &pairs, int first, std::vector<int> &skip )
{
	const size_t held = skip.size();	// the occluders that hold the viewpoint (fillers among them too)
	for ( size_t i = 0; i * 2 + 1 < pairs.size(); i++ )
	{
		const int o = first + (int)i;
		if ( std::find( skip.begin(), skip.end(), o ) != skip.end() )
			continue;
		if ( std::find( skip.begin(), skip.begin() + held, pairs[i * 2] ) != skip.begin() + held ||
			 std::find( skip.begin(), skip.begin() + held, pairs[i * 2 + 1] ) != skip.begin() + held )
			skip.push_back( o );
	}
}

int BwVis_OccluderSolids( const CollisionWorld &world, const std::function<void( const ConvexShape &, int, bool )> &fn )
{
	int skipped = 0;
	std::vector<int> brushes;
	CollectModelBrushes( 0, brushes );
	std::vector<unsigned char> entOpaque;
	BrushEntitiesDrawnOpaque( entOpaque );
	ConvexShape shape;
	for ( size_t i = 0; i < brushes.size(); i++ )
	{
		int b = brushes[i];
		if ( !( dbrushes[b].contents & CONTENTS_SOLID ) )
			continue;
		if ( !OpaqueSolid( b ) || AllNodraw( b ) )
		{
			skipped++;
			continue;
		}
		if ( !BuildBrushShape( b, shape ) )
			continue;
		// a nodraw side open to the map: blocks only what enters through a drawn side (#77)
		fn( shape, b, OpenNodrawSide( b ) );
	}
	for ( size_t i = 0; i < world.convexes.size(); i++ )
	{
		const SourceConvex &sc = world.convexes[i];
		if ( sc.category != CAT_FUNC_BRUSH_SOLID )
			continue;
		// a func_brush occludes only when drawn opaque all game long, and not
		// through any nodraw side (its planes are in model space: no leaf test)
		bool drawn = sc.entityIndex >= 0 && sc.entityIndex < (int)entOpaque.size() && entOpaque[sc.entityIndex];
		if ( !drawn || !OpaqueSolid( sc.sourceIndex ) || AnyNodraw( sc.sourceIndex ) )
		{
			skipped++;
			continue;
		}
		fn( sc.shape, -1 - (int)i, false );
	}
	return skipped;
}

static void BuildOccluders( const CollisionWorld &world, VisStats &st )
{
	s_OccPlanes.clear();
	s_OccOfs.assign( 1, 0 );
	s_OccSrc.clear();
	s_SeamPairs.clear();
	s_DirPlanes.clear();
	s_DirDrawn.clear();
	s_DirOfs.assign( 1, 0 );
	s_DirSrc.clear();
	s_BrushPlanes.clear();
	s_BrushDrawn.clear();
	s_BrushOfs.assign( 1, 0 );
	s_BrushRun.clear();
	std::vector<dvec3> mins, maxs, dirMins, dirMaxs;
	std::vector<ConvexShape> twoSided;	// the shapes of the two-sided occluders, for the seam fillers
	std::vector<int> brushes;
	CollectModelBrushes( 0, brushes );
	ConvexShape shape;
	st.occludersSkipped += BwVis_OccluderSolids( world, [&]( const ConvexShape &s, int src, bool oneWay ) {
		if ( oneWay )
			AddDirOccluder( src, s, dirMins, dirMaxs );
		else
		{
			AddOccluder( s, mins, maxs, src );
			twoSided.push_back( s );
		}
	} );
	st.occluders = (int)mins.size();
	st.occludersOneWay = (int)dirMins.size();
	// Seam fillers between the two-sided occluders, after them in s_OccPlanes. They block
	// as any occluder does (the face pass, the fan, the GPU meshes, the solid-viewpoint
	// test), so they come from those alone: a one-way occluder sees through from its open
	// nodraw side, and a filler against one would block a view it lets through. Contacts
	// with a one-way occluder keep their slot (the safe side). twoSided[i] is occluder i
	// (AddOccluder adds one each, in the same order).
	{
		std::vector< std::vector<DPlane> > fill;
		std::vector<int> pairs;
		st.seamContacts = BwVis_SeamFillers( twoSided, fill, &st.seamExtended, &pairs );
		std::vector<ConvexShape>().swap( twoSided );
		s_FirstSeam = (int)s_OccSrc.size();
		for ( size_t i = 0; i < fill.size(); i++ )
		{
			if ( !ConvexFromPlanes( fill[i].data(), (int)fill[i].size(), shape ) )
				continue;
			s_OccSrc.push_back( OCC_SRC_SEAM );
			s_SeamPairs.push_back( pairs[i * 2] );
			s_SeamPairs.push_back( pairs[i * 2 + 1] );
			s_OccPlanes.insert( s_OccPlanes.end(), fill[i].begin(), fill[i].end() );
			s_OccOfs.push_back( (int)s_OccPlanes.size() );
			mins.push_back( shape.mins );
			maxs.push_back( shape.maxs );
			st.occludersSeam++;
		}
	}
	s_OccBvh.Build( mins, maxs );
	s_DirBvh.Build( dirMins, dirMaxs );
	if ( dirMins.empty() )
		return;
	// every world brush, whatever its contents: DirSegmentBlocks asks whether a side is
	// covered, and any brush may cover one (more cover only shows more). Only the opaque
	// solid ones make a run (s_BrushRun): a run through a see-through brush that a drawn
	// side starts would block a view the player has through it.
	std::vector<dvec3> bMins, bMaxs;
	for ( size_t i = 0; i < brushes.size(); i++ )
	{
		if ( !BuildBrushShape( brushes[i], shape ) )
			continue;
		for ( size_t f = 0; f < shape.faces.size(); f++ )
		{
			DPlane p;
			p.n = shape.faceNormals[f];
			p.d = -1e300;
			for ( size_t k = 0; k < shape.faces[f].size(); k++ )
				p.d = MAX( p.d, DDot( p.n, shape.verts[shape.faces[f][k]] ) );
			s_BrushPlanes.push_back( p );
			s_BrushDrawn.push_back( ShapePlaneDrawn( brushes[i], p.n, p.d ) );
		}
		s_BrushOfs.push_back( (int)s_BrushPlanes.size() );
		s_BrushRun.push_back( OpaqueSolid( brushes[i] ) ? 1 : 0 );
		bMins.push_back( shape.mins );
		bMaxs.push_back( shape.maxs );
	}
	s_BrushBvh.Build( bMins, bMaxs );
}

// How deep the segment a -> b runs into the convex (planes n . x <= d): the max
// over t of min_k (d_k - n_k . x(t)), at parameter *pAt. Below zero it misses by
// about that. Concave in t, so a ternary search finds it.
static double GrazeConvex( const dvec3 &a, const dvec3 &b, const DPlane *planes, int numPlanes, double *pAt = NULL )
{
	const dvec3 d = b - a;
	auto depthAt = [&]( double t ) {
		dvec3 x = a + d * t;
		double m = 1e300;
		for ( int k = 0; k < numPlanes; k++ )
			m = MIN( m, planes[k].d - DDot( planes[k].n, x ) );
		return m;
	};
	double lo = 0.0, hi = 1.0;
	for ( int it = 0; it < 100; it++ )
	{
		double m1 = lo + ( hi - lo ) / 3, m2 = hi - ( hi - lo ) / 3;
		if ( depthAt( m1 ) < depthAt( m2 ) )
			lo = m1;
		else
			hi = m2;
	}
	if ( pAt )
		*pAt = 0.5 * ( lo + hi );
	return depthAt( 0.5 * ( lo + hi ) );
}

// -traceface: every world brush (model 0) with its real (unshrunk) planes, so a
// traced segment can list what it passes through or grazes, occluder or not.
struct TraceBrush
{
	int brush;
	std::vector<DPlane> planes;
	dvec3 mins, maxs;
};
static std::vector<TraceBrush> s_TraceBrushes;

static void TraceBuildBrushes()
{
	if ( !s_TraceBrushes.empty() )
		return;
	std::vector<int> brushes;
	CollectModelBrushes( 0, brushes );
	ConvexShape shape;
	for ( size_t i = 0; i < brushes.size(); i++ )
	{
		if ( !BuildBrushShape( brushes[i], shape ) )
			continue;
		TraceBrush tb;
		tb.brush = brushes[i];
		for ( size_t f = 0; f < shape.faces.size(); f++ )
		{
			DPlane p;
			p.n = shape.faceNormals[f];
			p.d = -1e300;
			for ( size_t k = 0; k < shape.faces[f].size(); k++ )
				p.d = MAX( p.d, DDot( p.n, shape.verts[shape.faces[f][k]] ) );
			tb.planes.push_back( p );
		}
		tb.mins = shape.mins;
		tb.maxs = shape.maxs;
		s_TraceBrushes.push_back( tb );
	}
}

// Why a world brush is or is not an occluder (BuildOccluders' rules).
static const char *TraceBrushRole( int b )
{
	for ( size_t o = 0; o < s_OccSrc.size(); o++ )
	{
		if ( s_OccSrc[o] == b )
			return "occluder";
	}
	for ( size_t o = 0; o < s_DirSrc.size(); o++ )
	{
		if ( s_DirSrc[o] == b )
		{
			// the drawn count shows at a glance whether the side matching worked
			static thread_local char buf[160];
			int drawn = 0;
			for ( int k = s_DirOfs[o]; k < s_DirOfs[o + 1]; k++ )
				drawn += s_DirDrawn[k] != 0;
			V_snprintf( buf, sizeof( buf ),
						"one-way occluder: a nodraw side open to the map, blocks through drawn sides (%d of %d planes drawn)",
						drawn, s_DirOfs[o + 1] - s_DirOfs[o] );
			return buf;
		}
	}
	if ( !( dbrushes[b].contents & CONTENTS_SOLID ) )
		return "not solid";
	if ( !OpaqueSolid( b ) )
		return "not occluder: see-through contents";
	if ( AllNodraw( b ) )
		return "not occluder: every side nodraw";
	if ( OpenNodrawSide( b ) )
		return "not occluder: a nodraw side open to the map, and no one-way shape";
	return "not occluder: no shape";
}

// -traceface: the occluder verdict on the segment a -> b, seen from b (SegmentBlocked's
// viewer), and the world brushes it passes within 1u of, deepest first: the real brush
// depth (above 0: it enters the brush) and the shrunk occluder's (above 1e-6: blocked).
// A segment that runs up to 1/32u into a real occluder passes its shrunk copy (#33); two
// abutting occluders each entered by less leave a 1/16u seam (#40).
static void TraceSegment( int fi, const char *what, const dvec3 &a, const dvec3 &b )
{
	TraceBuildBrushes();
	struct Near
	{
		double depth, at;
		int brush;
	};
	std::vector<Near> near;
	const dvec3 pad( 1, 1, 1 );
	const dvec3 lo = DMin( a, b ) - pad, hi = DMax( a, b ) + pad;
	for ( size_t i = 0; i < s_TraceBrushes.size(); i++ )
	{
		const TraceBrush &tb = s_TraceBrushes[i];
		if ( tb.mins.x > hi.x || tb.mins.y > hi.y || tb.mins.z > hi.z || tb.maxs.x < lo.x || tb.maxs.y < lo.y || tb.maxs.z < lo.z )
			continue;
		Near n;
		n.depth = GrazeConvex( a, b, tb.planes.data(), (int)tb.planes.size(), &n.at );
		n.brush = tb.brush;
		if ( n.depth > -1.0 )
			near.push_back( n );
	}
	std::sort( near.begin(), near.end(), []( const Near &x, const Near &y ) { return x.depth > y.depth; } );
	TraceF( fi, "    %s (%.4f %.4f %.4f) -> (%.4f %.4f %.4f), length %.4f: occluders %s it; %d world brush(es) within 1u\n",
			what, a.x, a.y, a.z, b.x, b.y, b.z, DLength( b - a ), SegmentBlocked( a, b ) ? "BLOCK" : "do not block",
			(int)near.size() );
	for ( size_t k = 0; k < near.size() && (int)k < TRACE_PRINT_BRUSHES; k++ )
	{
		const Near &n = near[k];
		dvec3 x = a + ( b - a ) * n.at;
		TraceF( fi, "      brush %d (contents 0x%x, %s): depth %+.5f (shrunk %+.5f) at (%.4f %.4f %.4f)\n", n.brush,
				dbrushes[n.brush].contents, TraceBrushRole( n.brush ), n.depth, n.depth - OCC_SHRINK, x.x, x.y, x.z );
		for ( size_t o = 0; o < s_DirSrc.size(); o++ )
		{
			if ( s_DirSrc[o] != n.brush )
				continue;
			// the one-way verdict, seen from b (SegmentBlocked's viewer)
			DirWhy why;
			bool blocks = DirSegmentBlocks( b, a, (int)o, &why );
			if ( !why.crosses )
				TraceF( fi, "        one-way: no crossing deeper than %g of its shrunk volume\n", OCC_DEPTH );
			else if ( why.t0 < 0 )
				TraceF( fi, "        one-way: the viewer is inside or on it (sees out)\n" );
			else if ( why.overflow )
				TraceF( fi, "        one-way: more than %d brushes before its entry (gave up: not blocked)\n", DIR_RUN_MAX_SPANS );
			else if ( why.viewerInSolid )
				TraceF( fi, "        one-way: the solid run before its entry reaches back to the viewer (in solid: sees out)\n" );
			else if ( !why.drawn )
				TraceF( fi, "        one-way: entered at %.4f; the solid run starts %.4f before it, at %d brush(es) through %d side(s), "
						"one nodraw: sees through\n", why.entry, why.entry - why.start, why.entering, why.ties );
			else
				TraceF( fi, "        one-way: entered at %.4f; the solid run starts %.4f before it, at %d brush(es) through %d drawn side(s); "
						"probe (%.4f %.4f %.4f) cluster %d%s: %s\n", why.entry, why.entry - why.start, why.entering, why.ties,
						why.q.x, why.q.y, why.q.z, why.cluster, why.inBrush ? ", inside a brush" : "",
						blocks ? "BLOCKS" : "sides covered or facing the void, no face there" );
		}
	}
}

// -traceface: an eye as text ('kind': "fan eye" or "eye sample").
static void TraceEye( int fi, const char *what, const char *kind, const TraceEyeInfo &e )
{
	char lv[48] = "";
	if ( e.level > 0 )
		V_snprintf( lv, sizeof( lv ), ", refinement level %d (%gu)", e.level, RefinedEyeSpacing( e.level ) );
	TraceF( fi, "    %s: %s %lld at (%.4f %.4f %.4f), source %s, cluster %d%s%s\n", what, kind, e.id, e.x, e.y, e.z,
			EyeSourceName( e.source ), e.cluster, e.sky >= 0 ? ", skybox copy" : "", lv );
}

//-----------------------------------------------------------------------------
// Face samples
//-----------------------------------------------------------------------------

// 2-D convex clip: keeps the part of poly with m . p <= c.
static void Clip2D( std::vector<dvec3> &poly, const dvec3 &m, double c )
{
	std::vector<dvec3> out;
	int n = (int)poly.size();
	for ( int i = 0; i < n; i++ )
	{
		const dvec3 &a = poly[i];
		const dvec3 &b = poly[( i + 1 ) % n];
		double da = DDot( m, a ) - c, db = DDot( m, b ) - c;
		if ( da <= 0 )
			out.push_back( a );
		if ( ( da < 0 && db > 0 ) || ( da > 0 && db < 0 ) )
			out.push_back( a + ( b - a ) * ( da / ( da - db ) ) );
	}
	poly.swap( out );
}

// The spacing a displacement is sampled at: sp, doubled until its triangles hold
// at most FACE_MAX_SAMPLES samples (the planar path's budget), or every triangle
// is a single step.
static double DispSpacing( const std::vector<dvec3> &tris, double sp )
{
	for ( ;; )
	{
		long long total = 0;
		bool coarsest = true;
		for ( size_t k = 0; k + 2 < tris.size(); k += 3 )
		{
			const dvec3 &a = tris[k], &b = tris[k + 1], &c = tris[k + 2];
			double len = MAX( DLength( b - a ), MAX( DLength( c - a ), DLength( c - b ) ) );
			long long steps = MAX( 1LL, (long long)ceil( len / sp ) );
			total += ( steps + 1 ) * ( steps + 2 ) / 2 + 1;	// the triangle lattice and its centroid
			coarsest = coarsest && steps == 1;
		}
		if ( total <= FACE_MAX_SAMPLES || coarsest )
			return sp;
		sp *= 2;
	}
}

// spUsed: the grid spacing the face got (faceSpacing, or coarser for a huge face).
static void SampleFace( const VisFace &f, std::vector<FaceSample> &out, double &spUsed )
{
	out.clear();
	double sp = s_pVisOpts->faceSpacing;
	spUsed = sp;
	if ( f.disp )
	{
		const std::vector<dvec3> &tris = *f.tris;
		sp = DispSpacing( tris, sp );
		spUsed = sp;
		for ( size_t k = 0; k + 2 < tris.size(); k += 3 )
		{
			const dvec3 &a = tris[k], &b = tris[k + 1], &c = tris[k + 2];
			dvec3 n = DCross( b - a, c - a );
			if ( DNormalize( n ) < 1e-12 )
				continue;
			if ( DDot( n, f.n ) < 0 )
				n = -n;
			double len = MAX( DLength( b - a ), MAX( DLength( c - a ), DLength( c - b ) ) );
			int steps = MAX( 1, (int)ceil( len / sp ) );
			for ( int i = 0; i <= steps; i++ )
			{
				for ( int j = 0; i + j <= steps; j++ )
				{
					double u = (double)i / steps, v = (double)j / steps;
					FaceSample s;
					s.p = a + ( b - a ) * u + ( c - a ) * v + n * FACE_LIFT;
					s.n = n;
					out.push_back( s );
				}
			}
			FaceSample s;
			s.p = ( a + b + c ) * ( 1.0 / 3.0 ) + n * FACE_LIFT;
			s.n = n;
			out.push_back( s );
		}
		return;
	}
	int n = (int)f.w.size();
	if ( n < 3 )
		return;
	dvec3 c( 0, 0, 0 );
	for ( int i = 0; i < n; i++ )
		c += f.w[i];
	c = c * ( 1.0 / n );
	std::vector<dvec3> in = f.w;
	for ( int i = 0; i < n; i++ )
	{
		dvec3 e = f.w[( i + 1 ) % n] - f.w[i];
		dvec3 m = DCross( e, f.n );
		if ( DNormalize( m ) < 1e-12 )
			continue;
		Clip2D( in, m, DDot( m, f.w[i] ) - FACE_INSET );
		if ( in.size() < 3 )
			break;
	}
	FaceSample s;
	s.n = f.n;
	s.p = c + f.n * FACE_LIFT;
	out.push_back( s );
	if ( in.size() < 3 )
		return;	// a sliver: its centroid only
	for ( size_t i = 0; i < in.size(); i++ )
	{
		s.p = in[i] + f.n * FACE_LIFT;
		out.push_back( s );
		s.p = ( in[i] + in[( i + 1 ) % in.size()] ) * 0.5 + f.n * FACE_LIFT;
		out.push_back( s );
	}
	dvec3 u = f.w[1] - f.w[0];
	DNormalize( u );
	dvec3 v = DCross( f.n, u );
	double u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
	for ( size_t i = 0; i < in.size(); i++ )
	{
		double pu = DDot( in[i], u ), pv = DDot( in[i], v );
		u0 = MIN( u0, pu );
		u1 = MAX( u1, pu );
		v0 = MIN( v0, pv );
		v1 = MAX( v1, pv );
	}
	// a huge face gets a coarser grid
	while ( ( ( u1 - u0 ) / sp + 1 ) * ( ( v1 - v0 ) / sp + 1 ) > FACE_MAX_SAMPLES )
		sp *= 2;
	spUsed = sp;
	// The lattice starts at the inset polygon's minimum corner, not at multiples of sp: a thin
	// strip's own edge then carries a sample line wherever the face sits in the world.
	double w0 = DDot( f.w[0], f.n );
	const int nu = (int)floor( ( u1 - u0 ) / sp ), nv = (int)floor( ( v1 - v0 ) / sp );
	for ( int iu = 0; iu <= nu; iu++ )
	{
		const double pu = u0 + iu * sp;
		for ( int iv = 0; iv <= nv; iv++ )
		{
			const double pv = v0 + iv * sp;
			dvec3 p = u * pu + v * pv + f.n * w0;
			bool inside = true;
			int m = (int)in.size();
			for ( int i = 0; i < m && inside; i++ )
			{
				dvec3 e = in[( i + 1 ) % m] - in[i];
				dvec3 o = DCross( e, f.n );
				if ( DDot( o, p - in[i] ) > 1e-9 * DLength( o ) )
					inside = false;
			}
			if ( inside )
			{
				s.p = p + f.n * FACE_LIFT;
				out.push_back( s );
			}
		}
	}
}

//-----------------------------------------------------------------------------
// The passes
//-----------------------------------------------------------------------------
// CPU fan of eye s_FanEyes[s_FanFirst + item]. Under -vischeck with the GPU
// fan, s_FanGpuT holds the GPU's occluder distance of each ray of the batch.
static const float *s_FanGpuT;
static int s_FanFirst;

static void FanOne( int iThread, int item )
{
	const FanEye &e = s_FanEyes[s_FanFirst + item];
	dvec3 o( e.x, e.y, e.z );
	int n = s_pVisOpts->fanRays;
	const float *gpuT = s_FanGpuT ? s_FanGpuT + (size_t)item * n : NULL;
	VisCounters &cnt = s_Counters[iThread];
	for ( int i = 0; i < n; i++ )
	{
		dvec3 dir = FibonacciDir( i, n );
		double t = NearestOccluder( o, dir );
		if ( gpuT )
		{
			// a ray the GPU stops short on may miss faces the CPU marks;
			// fan_cpu_only_faces is what tells whether any face was lost
			cnt.fanCheck[0]++;
			if ( gpuT[i] < t )
			{
				cnt.fanCheck[1]++;
				cnt.fanShort = MAX( cnt.fanShort, t - gpuT[i] );
			}
		}
		MarkFacesAlong( o, dir, t + FAN_PAST, s_FanFirst + item, i, t );
	}
	cnt.rays[0] += n;
}

// -traceface with the GPU fan (#85): the GPU marks faces without saying which
// ray did, so the traced faces it marked (s_FanReplay) have their fan rays
// replayed on the CPU for the log: the CPU's occluders (NearestOccluder) and
// crossing test (FanCrosses), only on rays that reach one of those faces' boxes.
// Nothing is marked; the verdict stays the GPU's.
static std::vector<int> s_FanReplay;
static void FanReplayOne( int iThread, int item )
{
	const int eye = item;
	const FanEye &e = s_FanEyes[eye];
	const dvec3 o( e.x, e.y, e.z );
	const int gateCl = FanGateCluster( e );
	const int n = s_pVisOpts->fanRays;
	const dvec3 pad( 0.01, 0.01, 0.01 );
	for ( int i = 0; i < n; i++ )
	{
		const dvec3 dir = FibonacciDir( i, n );
		const dvec3 inv = SafeInv( dir );
		double t = -1.0;
		for ( size_t k = 0; k < s_FanReplay.size(); k++ )
		{
			const int fi = s_FanReplay[k];
			if ( !SegBox( o, inv, 0.0, FAN_MAX_T + FAN_PAST, s_Faces[fi].mins - pad, s_Faces[fi].maxs + pad ) )
				continue;
			if ( t < 0 )
				t = NearestOccluder( o, dir );
			double tHit;
			if ( FanCrosses( fi, o, dir, t + FAN_PAST, tHit ) )
				TraceFanHit( fi, eye, i, dir, t, tHit, !FanEyeMaySee( gateCl, fi ) );
		}
	}
}

// CapCandidateEyes' choice as positions into the list: runSize[r] eyes in run r,
// the runs one after another. 'pos' gets the kept positions, ascending. The gather
// pass (GatherTileOne) takes them while it reads the eyes, without listing every one first.
static void CapPositions( const std::vector<size_t> &runSize, int cap, std::vector<size_t> &pos )
{
	pos.clear();
	const size_t m = runSize.size();
	size_t n = 0;
	std::vector<size_t> start( m );
	for ( size_t r = 0; r < m; r++ )
	{
		start[r] = n;
		n += runSize[r];
	}
	if ( cap <= 0 || !m )
		return;
	if ( n <= (size_t)cap )
	{
		for ( size_t i = 0; i < n; i++ )
			pos.push_back( i );
		return;
	}
	pos.reserve( cap );
	if ( m > (size_t)cap )
	{
		// more clusters than eyes to keep: the middle eye of each of an even subset of the clusters
		for ( size_t j = 0; j < (size_t)cap; j++ )
		{
			size_t r = ( 2 * j + 1 ) * m / ( 2 * (size_t)cap );
			if ( runSize[r] )		// callers pass non-empty runs; an empty one has no middle eye
				pos.push_back( start[r] + runSize[r] / 2 );
		}
		return;
	}
	// Smallest cluster first, each takes the lesser of its eyes and an even split of what is
	// left: a small cluster keeps all it has, the big ones divide the rest, and every cluster
	// keeps at least one (cap >= m, and a split never falls below the first one).
	std::vector<size_t> order( m );
	for ( size_t r = 0; r < m; r++ )
		order[r] = r;
	std::sort( order.begin(), order.end(), [&]( size_t a, size_t b ) {
		return runSize[a] != runSize[b] ? runSize[a] < runSize[b] : a < b;
	} );
	std::vector<size_t> quota( m );
	size_t left = (size_t)cap;
	for ( size_t k = 0; k < m; k++ )
	{
		size_t q = MIN( runSize[order[k]], left / ( m - k ) );
		quota[order[k]] = q;
		left -= q;
	}
	for ( size_t r = 0; r < m; r++ )
	{
		// q evenly spaced eyes of the cluster, each in the middle of its stretch of the list
		const size_t size = runSize[r], q = quota[r];
		for ( size_t j = 0; j < q; j++ )
			pos.push_back( start[r] + ( 2 * j + 1 ) * size / ( 2 * q ) );
	}
}

void CapCandidateEyes( std::vector<int> &eyes, const std::vector<int> &runStart, int cap )
{
	const size_t n = eyes.size(), m = runStart.size();
	if ( cap <= 0 || n <= (size_t)cap || !m )
		return;
	std::vector<size_t> runSize( m ), pos;
	for ( size_t r = 0; r < m; r++ )
		runSize[r] = ( r + 1 < m ? (size_t)runStart[r + 1] : n ) - (size_t)runStart[r];
	CapPositions( runSize, cap, pos );
	std::vector<int> kept( pos.size() );
	for ( size_t k = 0; k < pos.size(); k++ )
		kept[k] = eyes[pos[k]];
	eyes.swap( kept );
}

// The GPU fan's meshes. Occluders: the CPU's, shrunk FAN_GPU_SHRINK more.
// Faces: each candidate face as a slab holding every point within FAN_GPU_PAD
// of the region the CPU's crossing test accepts (SlabOne). Both make the GPU
// mark more, never less: a ray that enters a CPU occluder at t runs at least
// FAN_GPU_SHRINK deeper before it meets the GPU's, and a ray that the CPU
// test accepts before t + FAN_PAST passes through the face's slab before that
// too. Float rounding (vertices, eye,
// direction and hit distance: under 0.01u at |coord| <= 16384 over the
// world's diagonal) stays well inside both margins. The margins cannot help a
// ray that starts inside a face's slab: it meets only the slab's far side,
// which may lie past the occluder when the ray runs nearly along the face
// toward an occluder standing in the slab (a corner). The host marks every
// face whose slab may hold a fan eye (FanEyeInSlab) instead.
// FAN_GPU_SHRINK: bw_vis.h
static const double FAN_GPU_PAD = 1.0 / 32.0;
// A face's crossing region is first cut from a square this far around the
// plane's centre (the point of the plane nearest the origin), past any point a
// fan ray reaches from an eye in the +-16384 world. A region with a corner
// still past FAN_UNBOUNDED from the origin was never cut back there: it is
// unbounded (a zero-area face), and no face of the map reaches that far.
static const double FAN_WORLD = 262144.0;
static const double FAN_UNBOUNDED = 131072.0;
// A point of a face's slab is within sqrt(3) * FAN_GPU_PAD of the region the
// CPU's test accepts: of its plane, and of each edge half-plane moved out by
// 1e-3. Twice the pad covers that, and 1/64u covers the float rounding of the
// slab's vertices and of the eye (under 0.01u).
static const double FAN_EYE_SLAB = 2.0 * FAN_GPU_PAD + 1.0 / 64.0;
// a one-sided face is marked while n . dir < this (the CPU: n . dir <= -1e-12)
static const float FAN_GPU_FACING = 1e-5f;
// rays per launch: short kernels, well under the display driver's watchdog
static const size_t FAN_GPU_LAUNCH_RAYS = (size_t)1 << 26;

// A convex as float vertices and fan-triangulated faces, appended to a mesh.
static void AppendConvex( const ConvexShape &cs, std::vector<float> &mv, std::vector<unsigned int> &mt )
{
	unsigned int base = (unsigned int)( mv.size() / 3 );
	for ( size_t k = 0; k < cs.verts.size(); k++ )
	{
		mv.push_back( (float)cs.verts[k].x );
		mv.push_back( (float)cs.verts[k].y );
		mv.push_back( (float)cs.verts[k].z );
	}
	for ( size_t q = 0; q < cs.faces.size(); q++ )
	{
		for ( size_t k = 1; k + 1 < cs.faces[q].size(); k++ )
		{
			mt.push_back( base + cs.faces[q][0] );
			mt.push_back( base + cs.faces[q][k] );
			mt.push_back( base + cs.faces[q][k + 1] );
		}
	}
}

// What the GPU canary needs of the occluder mesh: its bounds, and the middle and
// normal of its largest face.
struct VisCanary
{
	dvec3 mins, maxs;
	dvec3 centre, normal;
	double area;

	VisCanary() : mins( 1e300, 1e300, 1e300 ), maxs( -1e300, -1e300, -1e300 ), area( 0 ) {}

	void Add( const ConvexShape &cs )
	{
		mins = DMin( mins, cs.mins );
		maxs = DMax( maxs, cs.maxs );
		for ( size_t f = 0; f < cs.faces.size() && f < cs.faceNormals.size(); f++ )
		{
			const std::vector<int> &face = cs.faces[f];
			if ( face.size() < 3 )
				continue;
			dvec3 sum( 0, 0, 0 ), cross( 0, 0, 0 );
			for ( size_t k = 0; k < face.size(); k++ )
				sum += cs.verts[face[k]];
			for ( size_t k = 1; k + 1 < face.size(); k++ )
				cross += DCross( cs.verts[face[k]] - cs.verts[face[0]], cs.verts[face[k + 1]] - cs.verts[face[0]] );
			double a = 0.5 * DLength( cross );
			if ( a > area )
			{
				area = a;
				centre = sum * ( 1.0 / face.size() );	// strictly inside a convex polygon
				normal = cs.faceNormals[f];
			}
		}
	}
};

// The occluders as a triangle mesh, each convex rebuilt from its planes moved
// in by 'shrink'. 'canary', when given, collects every convex of the mesh.
static void OccluderMesh( double shrink, std::vector<float> &mv, std::vector<unsigned int> &mt, VisCanary *canary = NULL )
{
	ConvexShape cs;
	std::vector<DPlane> planes;
	for ( size_t o = 0; o + 1 < s_OccOfs.size(); o++ )
	{
		planes.assign( s_OccPlanes.begin() + s_OccOfs[o], s_OccPlanes.begin() + s_OccOfs[o + 1] );
		for ( size_t k = 0; k < planes.size(); k++ )
			planes[k].d -= shrink;
		if ( !ConvexFromPlanes( planes.data(), (int)planes.size(), cs ) )
			continue;	// a sliver thinner than the shrink: gone, which only shows more
		if ( canary )
			canary->Add( cs );
		AppendConvex( cs, mv, mt );
	}
}

// The GPU fan's one-way occluders (#77; BwFanOneWay): each hull shrunk by FAN_GPU_SHRINK past
// the CPU's, so a ray that stops at it stops no earlier than NearestOccluder stops it, with the
// occluder of each triangle and the occluder's real planes and drawn flags, from which
// bw_fan.cu __anyhit__dir decides as NearestOccluder does, letting a ray run on wherever float
// rounding (FAN_DIR_SLOP) could change the answer: the safe side (a face only the GPU marks).
static const float FAN_DIR_SLOP = 1.0f / 32.0f;
static void FanOneWayMesh( BwFanOneWay &out )
{
	ConvexShape cs;
	std::vector<DPlane> planes;
	out.ofs.push_back( 0 );
	for ( size_t o = 0; o + 1 < s_DirOfs.size(); o++ )
	{
		planes.assign( s_DirPlanes.begin() + s_DirOfs[o], s_DirPlanes.begin() + s_DirOfs[o + 1] );
		for ( size_t k = 0; k < planes.size(); k++ )
		{
			out.planes.push_back( (float)planes[k].n.x );
			out.planes.push_back( (float)planes[k].n.y );
			out.planes.push_back( (float)planes[k].n.z );
			out.planes.push_back( (float)( planes[k].d + OCC_SHRINK ) );
			out.drawn.push_back( s_DirDrawn[s_DirOfs[o] + k] );
			planes[k].d -= FAN_GPU_SHRINK;
		}
		out.ofs.push_back( (unsigned int)( out.drawn.size() ) );
		if ( !ConvexFromPlanes( planes.data(), (int)planes.size(), cs ) )
			continue;	// a sliver thinner than the shrink: gone, which only shows more
		unsigned int base = (unsigned int)( out.verts.size() / 3 );
		for ( size_t k = 0; k < cs.verts.size(); k++ )
		{
			out.verts.push_back( (float)cs.verts[k].x );
			out.verts.push_back( (float)cs.verts[k].y );
			out.verts.push_back( (float)cs.verts[k].z );
		}
		for ( size_t q = 0; q < cs.faces.size(); q++ )
		{
			const std::vector<int> &f = cs.faces[q];
			for ( size_t k = 1; k + 1 < f.size(); k++ )
			{
				out.tris.push_back( base + f[0] );
				out.tris.push_back( base + f[k] );
				out.tris.push_back( base + f[k + 1] );
				out.prim.push_back( (unsigned int)o );
			}
		}
	}
}

// Fan slabs per candidate face (s_SlabFaces[i]): local vertex and index lists,
// or a plate record (BwFanLaunchParams::plates) for a zero-area face, or
// s_SlabMark set when the face is marked without a ray.
static std::vector<int> s_SlabFaces;
static std::vector< std::vector<float> > s_SlabVerts;
static std::vector< std::vector<unsigned int> > s_SlabTris;
static std::vector< std::vector<float> > s_SlabPlate;
static std::vector<unsigned char> s_SlabMark;
// the bounds of each mesh slab's region (before the pad); mins > maxs for none
static std::vector<dvec3> s_SlabMins, s_SlabMaxs;

static void PushBits( std::vector<float> &v, unsigned int u )
{
	float x;
	memcpy( &x, &u, sizeof( x ) );
	v.push_back( x );
}

static void SlabOne( int iThread, int item )
{
	const VisFace &f = s_Faces[s_SlabFaces[item]];
	std::vector<float> &mv = s_SlabVerts[item];
	std::vector<unsigned int> &mt = s_SlabTris[item];
	ConvexShape cs;
	if ( !f.disp )
	{
		// The region the CPU's inside test (InsidePolygon) accepts: the face
		// plane cut by every edge whose outward normal is not degenerate, moved
		// out by its 1e-3u tolerance. The slab is the hull of that polygon's
		// corners moved by +-pad on every axis. A zero-area face (its points on
		// one point or one line: vbsp leaves a few, 161 on fightspace3) passes
		// the test on an unbounded strip or the whole plane. A slab for that
		// spans the map, and its triangles' boxes overlap every ray (161 such
		// slabs took fightspace3's fan from 14 s to 99 s), so it becomes a
		// plate: the plane and the same edge half-planes, which the raygen
		// program tests directly.
		DWinding w;
		DBaseWindingForPlane( f.n, f.d, FAN_WORLD, w );
		std::vector<float> halves;
		int n = (int)f.w.size();
		for ( int i = 0; i < n && w.size() >= 3; i++ )
		{
			dvec3 m = DCross( f.w[( i + 1 ) % n] - f.w[i], f.n );
			double len = DLength( m );
			if ( len < 1e-12 )
				continue;
			m = m * ( 1.0 / len );
			// keep m . x <= m . a + 1e-3
			double c = DDot( m, f.w[i] ) + 1e-3;
			DChopWindingInPlace( w, -m, -c, 0.0 );
			halves.push_back( (float)m.x );
			halves.push_back( (float)m.y );
			halves.push_back( (float)m.z );
			halves.push_back( (float)c );
		}
		if ( w.size() < 3 )
		{
			s_SlabMark[item] = 1;	// nothing to build a slab from: never lose the face
			return;
		}
		bool unbounded = false;
		for ( size_t k = 0; k < w.size() && !unbounded; k++ )
			unbounded = DLength( w[k] ) > FAN_UNBOUNDED;
		if ( unbounded )
		{
			std::vector<float> &pl = s_SlabPlate[item];
			PushBits( pl, (unsigned int)s_SlabFaces[item] );
			PushBits( pl, (unsigned int)( halves.size() / 4 ) );
			pl.push_back( (float)f.n.x );
			pl.push_back( (float)f.n.y );
			pl.push_back( (float)f.n.z );
			pl.push_back( (float)f.d );
			pl.insert( pl.end(), halves.begin(), halves.end() );
			return;
		}
		std::vector<dvec3> pts;
		for ( size_t k = 0; k < w.size(); k++ )
		{
			s_SlabMins[item] = DMin( s_SlabMins[item], w[k] );
			s_SlabMaxs[item] = DMax( s_SlabMaxs[item], w[k] );
			for ( int c = 0; c < 8; c++ )
			{
				pts.push_back( w[k] + dvec3( c & 1 ? FAN_GPU_PAD : -FAN_GPU_PAD, c & 2 ? FAN_GPU_PAD : -FAN_GPU_PAD,
											 c & 4 ? FAN_GPU_PAD : -FAN_GPU_PAD ) );
			}
		}
		if ( ConvexFromPoints( pts.data(), (int)pts.size(), cs ) )
			AppendConvex( cs, mv, mt );
		else
			s_SlabMark[item] = 1;	// nothing to build a slab from: never lose the face
		return;
	}
	// a displacement: each render triangle, whose CPU test (RayHitsTriangle)
	// has a barycentric tolerance of 1e-6 and skips degenerate triangles, as
	// the hull of its corners moved by +-pad on every axis
	const std::vector<dvec3> &tris = *f.tris;
	std::vector<dvec3> pts;
	for ( size_t g = 0; g + 2 < tris.size(); g += 3 )
	{
		pts.clear();
		for ( size_t k = 0; k < 3; k++ )
		{
			s_SlabMins[item] = DMin( s_SlabMins[item], tris[g + k] );
			s_SlabMaxs[item] = DMax( s_SlabMaxs[item], tris[g + k] );
			for ( int c = 0; c < 8; c++ )
			{
				pts.push_back( tris[g + k] + dvec3( c & 1 ? FAN_GPU_PAD : -FAN_GPU_PAD, c & 2 ? FAN_GPU_PAD : -FAN_GPU_PAD,
													  c & 4 ? FAN_GPU_PAD : -FAN_GPU_PAD ) );
			}
		}
		if ( ConvexFromPoints( pts.data(), (int)pts.size(), cs ) )
			AppendConvex( cs, mv, mt );
		else
			s_SlabMark[item] = 1;	// cannot happen for a padded triangle; never lose the face
	}
}

// Is p within m of triangle abc: of its plane and of each edge's half-plane?
// That holds every point of the triangle's slab (its corners moved by +-pad on
// every axis) when m >= sqrt(3) * pad. A degenerate triangle never passes the
// CPU's test; its box decides.
static bool NearTriangle( const dvec3 &p, const dvec3 &a, const dvec3 &b, const dvec3 &c, double m )
{
	dvec3 n = DCross( b - a, c - a );
	double len = DLength( n );
	if ( len < 1e-12 )
	{
		dvec3 lo = DMin( a, DMin( b, c ) ), hi = DMax( a, DMax( b, c ) );
		return p.x >= lo.x - m && p.y >= lo.y - m && p.z >= lo.z - m && p.x <= hi.x + m && p.y <= hi.y + m &&
			   p.z <= hi.z + m;
	}
	n = n * ( 1.0 / len );
	if ( fabs( DDot( n, p - a ) ) > m )
		return false;
	const dvec3 *v[3] = { &a, &b, &c };
	for ( int i = 0; i < 3; i++ )
	{
		const dvec3 &s = *v[i], &e = *v[( i + 1 ) % 3];
		dvec3 out = DCross( e - s, n );	// outward: the triangle winds counter-clockwise about n
		double ol = DLength( out );
		if ( ol > 0 && DDot( out, p - s ) / ol > m )
			return false;
	}
	return true;
}

// FanEyeInSlab: the slab items' BVH (boxes padded by FAN_EYE_SLAB) and the
// faces it marks. Many fan eyes test and mark the same face at once, so the marks
// are relaxed atomics, as s_Visible is.
static Bvh s_SlabBvh;
static std::unique_ptr< std::atomic<unsigned char>[] > s_FanInSlab;

// Marks every face whose slab may hold fan eye s_FanEyes[item] (see the GPU
// meshes' comment), a superset: within FAN_EYE_SLAB of the plane and of every
// edge's half-plane moved out by 1e-3, as the CPU's test (InsidePolygon) does.
static void FanEyeInSlab( int iThread, int item )
{
	if ( s_SlabBvh.nodes.empty() )
		return;
	const FanEye &e = s_FanEyes[item];
	dvec3 p( e.x, e.y, e.z );
	const double m = FAN_EYE_SLAB;
	int stack[128];
	int sp = 0;
	stack[sp++] = 0;
	while ( sp )
	{
		const BvhNode &nd = s_SlabBvh.nodes[stack[--sp]];
		if ( p.x < nd.mins.x || p.y < nd.mins.y || p.z < nd.mins.z || p.x > nd.maxs.x || p.y > nd.maxs.y || p.z > nd.maxs.z )
			continue;
		if ( nd.left >= 0 )
		{
			if ( sp < 126 )
			{
				stack[sp++] = nd.left;
				stack[sp++] = nd.right;
			}
			continue;
		}
		for ( int i = nd.first; i < nd.first + nd.count; i++ )
		{
			int fi = s_SlabFaces[s_SlabBvh.items[i]];
			if ( s_FanInSlab[fi].load( std::memory_order_relaxed ) )
				continue;
			const VisFace &f = s_Faces[fi];
			bool in = false;
			if ( f.disp )
			{
				const std::vector<dvec3> &tris = *f.tris;
				for ( size_t k = 0; k + 2 < tris.size() && !in; k += 3 )
					in = NearTriangle( p, tris[k], tris[k + 1], tris[k + 2], m );
			}
			else
				in = fabs( DDot( f.n, p ) - f.d ) <= m && InsidePolygon( f, p, 1e-3 + m );
			if ( in )
				s_FanInSlab[fi].store( 1, std::memory_order_relaxed );
		}
	}
}

// The fan on the GPU (bw_fan.cu): every candidate face (s_Visible 0) it marks
// becomes WHY_FAN. With -vischeck the CPU fan runs too, batch by batch against
// the GPU's occluder distances, and its marks are kept. False when the GPU
// could not run it; s_Visible then holds no GPU mark (the caller runs the CPU
// fan, which repeats any CPU marks already made).
static bool FanGpu( VisStats &st, bool check )
{
	double t0 = Plat_FloatTime();
	int nf = (int)s_Faces.size();
	int n = s_pVisOpts->fanRays;
	std::vector<float> ov, fv;
	std::vector<unsigned int> ot, ft;
	OccluderMesh( FAN_GPU_SHRINK, ov, ot );
	BwFanOneWay dirMesh;
	FanOneWayMesh( dirMesh );

	// the GPU starts from a plain copy of the marks; nothing else runs during the copy
	std::vector<unsigned char> init( nf );
	for ( int f = 0; f < nf; f++ )
		init[f] = (unsigned char)VisibleGet( f );
	s_SlabFaces.clear();
	for ( int f = 0; f < nf; f++ )
	{
		if ( !init[f] )
			s_SlabFaces.push_back( f );
	}
	s_SlabVerts.assign( s_SlabFaces.size(), std::vector<float>() );
	s_SlabTris.assign( s_SlabFaces.size(), std::vector<unsigned int>() );
	s_SlabPlate.assign( s_SlabFaces.size(), std::vector<float>() );
	s_SlabMark.assign( s_SlabFaces.size(), 0 );
	s_SlabMins.assign( s_SlabFaces.size(), dvec3( 1e300, 1e300, 1e300 ) );
	s_SlabMaxs.assign( s_SlabFaces.size(), dvec3( -1e300, -1e300, -1e300 ) );
	RunThreadsBatched( (int)s_SlabFaces.size(), SlabOne );

	// faces whose slab may hold a fan eye: marked here (FanEyeInSlab)
	{
		std::vector<dvec3> bmins, bmaxs;
		std::vector<int> bItem;
		dvec3 g( FAN_EYE_SLAB + 1e-3, FAN_EYE_SLAB + 1e-3, FAN_EYE_SLAB + 1e-3 );
		for ( size_t i = 0; i < s_SlabFaces.size(); i++ )
		{
			if ( s_SlabMins[i].x > s_SlabMaxs[i].x )
				continue;	// no mesh slab: a plate (its test starts at the eye) or marked already
			bItem.push_back( (int)i );
			bmins.push_back( s_SlabMins[i] - g );
			bmaxs.push_back( s_SlabMaxs[i] + g );
		}
		s_SlabBvh.Build( bmins, bmaxs );
		for ( size_t i = 0; i < s_SlabBvh.items.size(); i++ )
			s_SlabBvh.items[i] = bItem[s_SlabBvh.items[i]];
		s_FanInSlab.reset( new std::atomic<unsigned char>[nf] );
		for ( int f = 0; f < nf; f++ )
			s_FanInSlab[f].store( 0, std::memory_order_relaxed );
		RunThreadsBatched( s_nFanEyes, FanEyeInSlab );
		s_SlabBvh = Bvh();
	}
	std::vector<unsigned char> markNoRay( nf, 0 );
	int degenerate = 0, inSlab = 0;
	for ( size_t i = 0; i < s_SlabFaces.size(); i++ )
	{
		if ( s_SlabMark[i] )
		{
			markNoRay[s_SlabFaces[i]] = 1;
			degenerate++;
		}
		else if ( s_FanInSlab[s_SlabFaces[i]].load( std::memory_order_relaxed ) )
		{
			markNoRay[s_SlabFaces[i]] = 1;
			inSlab++;
		}
	}
	s_FanInSlab.reset();
	std::vector<dvec3>().swap( s_SlabMins );
	std::vector<dvec3>().swap( s_SlabMaxs );
	// the GPU starts from these marks too, so it skips those faces
	std::vector<unsigned char> start( init );
	for ( int f = 0; f < nf; f++ )
		start[f] |= markNoRay[f];
	std::vector<unsigned int> prim;
	std::vector<float> plates;
	unsigned int numPlates = 0;
	for ( size_t i = 0; i < s_SlabFaces.size(); i++ )
	{
		if ( !s_SlabPlate[i].empty() )
		{
			plates.insert( plates.end(), s_SlabPlate[i].begin(), s_SlabPlate[i].end() );
			numPlates++;
		}
		unsigned int base = (unsigned int)( fv.size() / 3 );
		fv.insert( fv.end(), s_SlabVerts[i].begin(), s_SlabVerts[i].end() );
		for ( size_t k = 0; k < s_SlabTris[i].size(); k++ )
			ft.push_back( base + s_SlabTris[i][k] );
		prim.insert( prim.end(), s_SlabTris[i].size() / 3, (unsigned int)s_SlabFaces[i] );
	}
	std::vector< std::vector<float> >().swap( s_SlabVerts );
	std::vector< std::vector<unsigned int> >().swap( s_SlabTris );
	std::vector< std::vector<float> >().swap( s_SlabPlate );

	std::vector<float> faceN( (size_t)nf * 4 ), dirs( (size_t)n * 3 ), eyes( (size_t)s_nFanEyes * 3 );
	for ( int f = 0; f < nf; f++ )
	{
		const VisFace &vf = s_Faces[f];
		faceN[f * 4 + 0] = (float)vf.n.x;
		faceN[f * 4 + 1] = (float)vf.n.y;
		faceN[f * 4 + 2] = (float)vf.n.z;
		faceN[f * 4 + 3] = ( vf.twoSided || vf.disp ) ? 0.0f : 1.0f;
	}
	for ( int i = 0; i < n; i++ )
	{
		dvec3 d = FibonacciDir( i, n );
		dirs[i * 3 + 0] = (float)d.x;
		dirs[i * 3 + 1] = (float)d.y;
		dirs[i * 3 + 2] = (float)d.z;
	}
	for ( int i = 0; i < s_nFanEyes; i++ )
	{
		const FanEye &e = s_FanEyes[i];
		eyes[i * 3 + 0] = e.x;
		eyes[i * 3 + 1] = e.y;
		eyes[i * 3 + 2] = e.z;
	}
	// the PVS gate, as FanEyeMaySee: no vis data leaves every eye ungated
	BwFanPvs pvs;
	pvs.eyeCl.assign( s_nFanEyes, -1 );
	pvs.faceClOfs = s_FaceClOfs;
	pvs.faceCl = s_FaceCl;
	pvs.rowBytes = (unsigned int)s_RowBytes;
	if ( !s_Pvs.empty() )
	{
		pvs.rows = s_Pvs;
		for ( int i = 0; i < s_nFanEyes; i++ )
		{
			int c = FanGateCluster( s_FanEyes[i] );
			pvs.eyeCl[i] = c < s_nClusters ? c : -1;
		}
	}
	bool ok = BwOptix_FanSetup( ov, ot, fv, ft, prim, faceN, plates, numPlates, dirs, eyes, start, pvs, dirMesh );
	if ( !ok )
		return false;
	double tSetup = Plat_FloatTime() - t0;

	unsigned int nEyes = (unsigned int)s_nFanEyes;
	unsigned int perLaunch = (unsigned int)MAX( (size_t)1, FAN_GPU_LAUNCH_RAYS / MAX( n, 1 ) );
	std::vector<float> tbuf;
	for ( unsigned int first = 0; first < nEyes && ok; first += perLaunch )
	{
		unsigned int count = MIN( perLaunch, nEyes - first );
		if ( check )
			tbuf.resize( (size_t)count * n );
		ok = BwOptix_FanTrace( first, count, (float)FAN_MAX_T, (float)( FAN_PAST + FAN_GPU_PAD ), FAN_GPU_FACING,
							   (float)FAN_GPU_PAD, FAN_DIR_SLOP, check ? tbuf.data() : NULL );
		if ( ok && !check )
			s_Counters[0].rays[0] += (long long)count * n;	// with -vischeck the CPU fan counts them
		if ( ok && check )
		{
			s_FanGpuT = tbuf.data();
			s_FanFirst = (int)first;
			RunThreadsBatched( (int)count, FanOne );
			s_FanGpuT = NULL;
			s_FanFirst = 0;
		}
	}
	std::vector<unsigned char> gpuVis( nf, 0 );
	ok = ok && BwOptix_FanVisible( gpuVis.data() );
	BwOptix_FanFree();
	if ( !ok )
	{
		// the CPU fan runs instead: forget the GPU's ray count and any -vischeck comparison
		for ( int t = 0; t <= MAX_TOOL_THREADS; t++ )
		{
			s_Counters[t].rays[0] = 0;
			s_Counters[t].fanCheck[0] = s_Counters[t].fanCheck[1] = 0;
			s_Counters[t].fanShort = 0;
		}
		return false;
	}
	for ( int f = 0; f < nf; f++ )
	{
		if ( init[f] )
			continue;
		bool g = gpuVis[f] != 0 || markNoRay[f];
		if ( check )
		{
			// the CPU fan has marked s_Visible: it decides
			bool c = VisibleGet( f ) != 0;
			st.fanGpuOnlyFaces += g && !c;
			st.fanCpuOnlyFaces += c && !g;
		}
		else if ( g )
			VisibleSet( f, WHY_FAN + 1 );
	}
	if ( check )
	{
		// the CPU fan counted its rays too
		for ( int t = 0; t <= MAX_TOOL_THREADS; t++ )
		{
			st.fanCheckRays += s_Counters[t].fanCheck[0];
			st.fanGpuShort += s_Counters[t].fanCheck[1];
			st.fanMaxShort = MAX( st.fanMaxShort, s_Counters[t].fanShort );
		}
	}
	// only now: a failed GPU fan reports no GPU meshes (the CPU fan ran)
	st.fanDegenerate = degenerate;
	st.fanEyeInSlab = inSlab;
	st.fanPlates = (int)numPlates;
	st.fanMeshTris = (long long)( ot.size() + ft.size() ) / 3;
	st.tFanSetup = tSetup;
	return true;
}

static inline bool EyeInFront( const VisFace &f, const EyeSample &e )
{
	return f.n.x * e.x + f.n.y * e.y + f.n.z * e.z > f.d + 1e-6;
}

// The per-eye back-face test against the plane n . x = d (a face's, or a candidate job's).
static inline bool EyeInFront( const dvec3 &n, double d, const float *e )
{
	return n.x * e[0] + n.y * e[1] + n.z * e[2] > d + 1e-6;
}

static inline bool EyeInFront( const VisFace &f, const float *e )
{
	return EyeInFront( f.n, f.d, e );
}

// Does a viewpoint sample in a solid leaf (s_SolidEyes) lie in front of the face
// (any such sample when !cull: a two-sided face or a displacement)? The cheap
// prefilter of SolidSeesFace.
static bool SolidViewInFront( const VisFace &f, bool cull )
{
	for ( size_t i = 0; i < s_SolidEyes.size(); i++ )
	{
		if ( !cull || EyeInFront( f, s_SolidEyes[i] ) )
			return true;
	}
	return false;
}

// The occluders that hold each solid viewpoint sample (s_SolidSkip). The engine
// draws from such a camera with its brush's sides facing away, so that brush
// hides nothing from it. Tested against the unshrunk planes.
static void FindSolidSkips()
{
	s_SolidSkip.assign( s_SolidEyes.size(), std::vector<int>() );
	for ( size_t i = 0; i < s_SolidEyes.size(); i++ )
	{
		const EyeSample &e = s_SolidEyes[i];
		dvec3 p( e.x, e.y, e.z );
		for ( int o = 0; o + 1 < (int)s_OccOfs.size(); o++ )
		{
			bool inside = true;
			for ( int k = s_OccOfs[o]; k < s_OccOfs[o + 1] && inside; k++ )
				inside = DDot( s_OccPlanes[k].n, p ) < s_OccPlanes[k].d + OCC_SHRINK;
			if ( inside )
				s_SolidSkip[i].push_back( o );
		}
		BwVis_SeamSkips( s_SeamPairs, s_FirstSeam, s_SolidSkip[i] );
	}
}

// The most SolidEntryBlocks steps back along a grazing segment to find the leaf
// an occluder is entered from. A probe still inside the brush reads solid, which
// keeps the face visible (the safe side).
static const double SOLID_ENTRY_BACK_MAX = 1.0;

bool BwVis_SolidEntryBlocks( const dvec3 &v, const dvec3 &p, const DPlane *planes, int numPlanes, BwSolidAtFn solidAt,
							 void *ctx )
{
	if ( !SegmentCrossesConvex( v, p, planes, numPlanes ) )
		return false;
	dvec3 d = p - v;
	double len = DLength( d );
	// where the segment enters it, coming from the viewer, and through which plane
	double t0 = 0.0;
	int kIn = -1;
	for ( int k = 0; k < numPlanes; k++ )
	{
		double den = DDot( planes[k].n, d );
		if ( den < -1e-300 )
		{
			double t = ( planes[k].d - DDot( planes[k].n, v ) ) / den;
			if ( t > t0 )
			{
				t0 = t;
				kIn = k;
			}
		}
	}
	// The probe sits OCC_SHRINK outside the brush's real side (the shrunk plane is
	// OCC_SHRINK inside it), so a solid sliver thicker than that still reads solid.
	// A viewer inside the occluder (no entry) probes its own point.
	dvec3 probe = v;
	if ( kIn >= 0 )
	{
		double cosv = -DDot( planes[kIn].n, d ) / len;
		double back = cosv > 1e-9 ? MIN( 2.0 * OCC_SHRINK / cosv, SOLID_ENTRY_BACK_MAX ) : SOLID_ENTRY_BACK_MAX;
		double tb = t0 - back / len;
		probe = tb > 0 ? v + d * tb : v;
	}
	return !solidAt( probe, ctx );	// entered from open space: through a drawn side
}

bool BwVis_SolidSegmentBlocked( const dvec3 &v, const dvec3 &p, const DPlane *planes, const int *ofs, const int *cand,
								int numCand, const std::vector<int> &skip, BwSolidAtFn solidAt, void *ctx )
{
	if ( DLength( p - v ) < 1e-9 )
		return false;
	for ( int i = 0; i < numCand; i++ )
	{
		int o = cand[i];
		if ( std::find( skip.begin(), skip.end(), o ) != skip.end() )
			continue;
		if ( BwVis_SolidEntryBlocks( v, p, &planes[ofs[o]], ofs[o + 1] - ofs[o], solidAt, ctx ) )
			return true;
	}
	return false;
}

static bool LeafSolidAt( const dvec3 &p, void * )
{
	return ( dleafs[PointLeaf( p )].contents & CONTENTS_SOLID ) != 0;
}

// Is the segment from viewer v to p blocked for a solid viewpoint sample whose
// containing occluders are 'skip'? BwVis_SolidSegmentBlocked over the occluder
// BVH, with the BSP's leaves for "solid": vbsp draws a brush side only toward
// open leaves, so a camera in a solid leaf sees through every side that faces
// another solid leaf (outside the sealed world, which vbsp fills solid, that is
// every outer wall; d1_trainstation_02 has two cameras there).
static bool SolidRayBlocked( const dvec3 &v, const dvec3 &p, const std::vector<int> &skip )
{
	if ( s_OccBvh.nodes.empty() )
		return false;
	dvec3 inv = SafeInv( p - v );
	int stack[128];
	int sp = 0;
	stack[sp++] = 0;
	while ( sp )
	{
		const BvhNode &nd = s_OccBvh.nodes[stack[--sp]];
		if ( !SegBox( v, inv, 0.0, 1.0, nd.mins, nd.maxs ) )
			continue;
		if ( nd.left >= 0 )
		{
			if ( sp < 126 )
			{
				stack[sp++] = nd.left;
				stack[sp++] = nd.right;
			}
			continue;
		}
		if ( BwVis_SolidSegmentBlocked( v, p, s_OccPlanes.data(), s_OccOfs.data(), &s_OccBvh.items[nd.first], nd.count,
										skip, LeafSolidAt, NULL ) )
			return true;
	}
	return false;
}

bool BwVis_SolidCamSees( const dvec3 &cam, const dvec3 &fn, double fd, bool twoSided, const FaceSample *samples,
						 int numSamples, const BwSolidMirror *mirror, BwSolidBlockedFn blocked, void *ctx, long long &pairs,
						 long long maxPairs )
{
	// the camera, or its image in the mirror, must be in front of the face
	const dvec3 eye = mirror ? MirrorPoint( cam, mirror->n, mirror->d ) : cam;
	if ( !twoSided && DDot( fn, eye ) <= fd + 1e-6 )
		return false;
	for ( int k = 0; k < numSamples; k++ )
	{
		const FaceSample &s = samples[k];
		dvec3 d = eye - s.p;
		if ( !twoSided && DDot( d, s.n ) <= 0 )
			continue;
		if ( ++pairs > maxPairs )
			return true;
		if ( !mirror )
		{
			if ( !blocked( cam, s.p, ctx ) )
				return true;
			continue;
		}
		if ( DDot( mirror->n, s.p ) <= mirror->d )
			continue;	// behind the mirror: not in its reflection
		double den = DDot( mirror->n, d );
		if ( fabs( den ) < 1e-12 )
			continue;
		double u = ( mirror->d - DDot( mirror->n, s.p ) ) / den;
		if ( u <= 0 || u >= 1 )
			continue;
		dvec3 q = s.p + d * u;
		// both legs by the camera's rule: the sample seen from Q, and Q from the camera
		if ( InsideWinding( *mirror->w, mirror->n, q, mirror->tol ) && !blocked( q, s.p, ctx ) &&
			 !blocked( cam, q, ctx ) )
			return true;
	}
	return false;
}

static bool SolidSkipBlocked( const dvec3 &v, const dvec3 &p, void *ctx )
{
	return SolidRayBlocked( v, p, *(const std::vector<int> *)ctx );
}

// Does a viewpoint sample in a solid leaf see the face (#76)? The engine model the
// fixture verifier uses: the sample is in front of the face (any sample for a
// two-sided face or a displacement; per face sample, as TestPair culls) and some
// segment from one of the face's samples to it is clear of the occluders, skipping
// those that hold the camera (s_SolidSkip); beyond the verifier, an occluder
// entered from a solid leaf does not block either (SolidRayBlocked). Through
// mirror R (mirror >= 0, an index into s_Reflective): the samples in front of R,
// the path face sample -> Q on R -> camera, Q inside R's polygon, both legs by the
// same rule, as TestPair traces a mirrored eye (BwVis_SolidCamSees). On the CPU:
// the solid samples are few. True as well when it cannot tell: no face samples, or
// past SOLID_MAX_PAIRS (camera sample, face sample) pairs.
static bool SolidSeesFace( const VisFace &f, const std::vector<FaceSample> &samples, int mirror )
{
	if ( samples.empty() )
		return true;
	BwSolidMirror m;
	const BwSolidMirror *pm = NULL;
	const VisFace *pR = NULL;
	if ( mirror >= 0 )
	{
		pR = &s_MirrorFace[mirror];
		m.n = pR->n;
		m.d = pR->d;
		m.w = &pR->w;
		m.tol = MIRROR_TOL;
		pm = &m;
	}
	long long pairs = 0;
	for ( size_t i = 0; i < s_SolidEyes.size(); i++ )
	{
		const EyeSample &e = s_SolidEyes[i];
		if ( pR && !EyeInFront( *pR, e ) )
			continue;
		// a displacement mirror's window is not a polygon on its plane (BwVis_DispMirrorPairs): a solid
		// viewpoint in front of the plane it shows keeps the face, untested (rare; the safe side)
		if ( pR && pR->disp )
			return true;
		if ( BwVis_SolidCamSees( dvec3( e.x, e.y, e.z ), f.n, f.d, f.twoSided || f.disp, samples.data(),
								 (int)samples.size(), pm, SolidSkipBlocked, &s_SolidSkip[i], pairs, SOLID_MAX_PAIRS ) )
			return true;
	}
	return false;
}

// Which eyes of bucket c pass the back-face test: 0 none, 1 all, 2 test each.
// The bucket's eye box decides most buckets without touching their eyes. The
// per-eye test is n . e > d + 1e-6. The box holds the exact min and max of the
// same float eye coordinates, and its dot products are computed in double in
// the same term order. Rounding is monotone, so every eye's value lies in
// [lo, hi] (the extremes sit at box corners, usually no eye's own value): 'all'
// means every eye passes and 'none' that none does, exactly as the per-eye
// test would answer. The extra 1e-6 either side
// ('all' above d + 2e-6, 'none' at or below d) is slack, not needed for that.
static int BucketSide( const dvec3 &pn, double pd, int c )
{
	const float *b = &s_BucketBox[(size_t)c * 6];
	double lo = 0, hi = 0;
	for ( int a = 0; a < 3; a++ )
	{
		double n = pn[a];
		lo += n * ( n > 0 ? b[a] : b[3 + a] );
		hi += n * ( n > 0 ? b[3 + a] : b[a] );
	}
	if ( lo > pd + 2e-6 )
		return 1;
	if ( hi <= pd )
		return 0;
	return 2;
}

static inline int BucketSide( const VisFace &f, int c )
{
	return BucketSide( f.n, f.d, c );
}

// Does eye cluster c's region (the k-DOP bound of its pieces' parts and samples, EyeBound)
// reach in front of the plane n . x = d? c < 0: without vis data the buckets are eye tiles, not
// clusters, so the test is made once for all eyes against the bound of all pieces
// (vbsp numbers clusters without vvis too, so the regions are not empty then).
static bool RegionInFront( const dvec3 &n, double d, int c )
{
	const EyeVolume &ev = *s_pEyes;
	bool noPvs = c < 0;
	if ( noPvs ? !ev.all.any : c >= s_pStore->NumClusters() )
		return false;
	const EyeBound &rb = noPvs ? ev.all : s_pStore->Facts( c ).region;
	return rb.Support( n, d + REGION_MARGIN ) > d + REGION_MARGIN;
}

static inline bool RegionInFront( const VisFace &f, int c )
{
	return RegionInFront( f.n, f.d, c );
}

static inline int BucketRunCount( int q )
{
	return s_BucketOfs[q + 1] - s_BucketOfs[q];
}

// -traceface: how far eye cluster c's region reaches past the plane n . x = d, by its
// box (what a box alone would have said), by the k-DOP bound the face pass uses,
// and the most any of the cluster's samples does ('sample', which the count pass
// measured: the region's true reach is between that and the bound when the samples
// include the piece vertices).
static void TraceRegion( int fi, const dvec3 &n, double d, int c, bool untestable, double sample )
{
	const EyeBound &rb = s_pStore->Facts( c ).region;
	double box = rb.BoxSupport( n ) - d, kdop = rb.Support( n ) - d;
	if ( box <= REGION_MARGIN && !untestable )
		return;
	TraceF( fi, "  face pass: eye cluster %d (%lld samples, none in front) has its region (%.2f %.2f %.2f) - (%.2f %.2f %.2f): "
				"%s; reach past the plane: box %.3f, k-DOP %.3f, best sample %.3f\n",
			c, s_BucketCount[c], rb.lo.x, rb.lo.y, rb.lo.z, rb.hi.x, rb.hi.y, rb.hi.z,
			untestable ? "in front, the face stays visible (no_eye)" : "the box reaches in front but the k-DOP bound does not",
			box, kdop, sample );
}

// A mirror's candidate eyes as CountCandidates finds them, for the sweep through a capped mirror
// (SweepThroughMirror, left face-major on purpose: see there): per eye bucket that holds one, its
// BucketSide answer and how many of its eyes pass. CandidateChunk walks them from a cursor, a chunk
// at a time, without listing them all. Both pin one tile at a time, only while they read it.
struct CandBucket
{
	int c, side;	// eye bucket, BucketSide (1: every eye passes, 2: test each)
	size_t pass;	// its eyes that pass
};
struct CandSet
{
	int fi;
	std::vector<CandBucket> buckets;
	size_t n;			// candidates in all
	size_t q;			// CandidateChunk's cursor: bucket ...
	int r, k;			// ... its run, and the place in the run
	CandSet() : fi( -1 ), n( 0 ), q( 0 ), r( 0 ), k( 0 ) {}
};

// Fills cs (cursor at the start) with the candidates of face fi in the candidate order: the buckets
// that may see it (the PVS test of JobSetup), culled by its plane unless it is two-sided or a
// displacement ('front': always). The untestable-region rule is not needed here: the mirror's
// status has it (MirrorStatusPass).
static void CountCandidates( int fi, bool front, CandSet &cs )
{
	const VisFace &f = s_Faces[fi];
	int nB = (int)s_BucketCount.size();
	bool cull = front || ( !f.twoSided && !f.disp );
	cs.fi = fi;
	cs.buckets.clear();
	cs.n = 0;
	cs.q = 0;
	cs.r = 0;
	cs.k = 0;
	for ( int c = 0; c < nB; c++ )
	{
		if ( !s_BucketCount[c] )
			continue;
		// a face with no cluster (every leaf it lies in is cluster -1, or its clusters were out
		// of range) has no PVS row to match: as with no vis data, every eye cluster is a candidate
		if ( !s_Pvs.empty() && s_FaceClOfs[fi] < s_FaceClOfs[fi + 1] )
		{
			bool any = false;
			for ( int k = s_FaceClOfs[fi]; k < s_FaceClOfs[fi + 1] && !any; k++ )
				any = PvsBit( c, s_FaceCl[k] );
			if ( !any )
				continue;
		}
		CandBucket bk;
		bk.c = c;
		bk.side = cull ? BucketSide( f, c ) : 1;
		bk.pass = 0;
		if ( bk.side == 1 )
			bk.pass = (size_t)s_BucketCount[c];
		else if ( bk.side == 2 )
		{
			for ( int r = 0; r < BucketRunCount( c ); r++ )
			{
				const EyeRun &run = s_BucketRuns[s_BucketOfs[c] + r];
				const float *e = &s_pStore->Pin( run.tile ).xyz[(size_t)run.start * 3];
				for ( int i = 0; i < run.count; i++, e += 3 )
					bk.pass += EyeInFront( f, e );
				s_pStore->Unpin( run.tile );
			}
		}
		if ( !bk.pass )
			continue;
		cs.buckets.push_back( bk );
		cs.n += bk.pass;
	}
}

// The next candidates of cs from its cursor, at most maxEyes, in list order (ids and
// positions); false when none are left.
static bool CandidateChunk( CandSet &cs, size_t maxEyes, std::vector<long long> &ids, std::vector<dvec3> &pts )
{
	ids.clear();
	pts.clear();
	const VisFace &f = s_Faces[cs.fi];
	while ( cs.q < cs.buckets.size() && ids.size() < maxEyes )
	{
		const CandBucket &bk = cs.buckets[cs.q];
		if ( cs.r >= BucketRunCount( bk.c ) )
		{
			cs.q++;
			cs.r = 0;
			cs.k = 0;
			continue;
		}
		const EyeRun &run = s_BucketRuns[s_BucketOfs[bk.c] + cs.r];
		const float *e = &s_pStore->Pin( run.tile ).xyz[(size_t)run.start * 3];
		const int n = run.count;
		for ( ; cs.k < n && ids.size() < maxEyes; cs.k++ )
		{
			const float *p = e + (size_t)cs.k * 3;
			if ( bk.side == 1 || EyeInFront( f, p ) )
			{
				ids.push_back( EyeId( run.tile, run.start + cs.k ) );
				pts.push_back( dvec3( p[0], p[1], p[2] ) );
			}
		}
		s_pStore->Unpin( run.tile );
		if ( cs.k >= n )
		{
			cs.r++;
			cs.k = 0;
		}
	}
	return !ids.empty();
}

struct TileEye
{
	dvec3 p;
	double dist2;
	dvec3 viaEye;	// mirror: the real eye
	long long id;	// the (real) eye's id in the store (-traceface)
};

// One eye per angular bin from c (the nearest not used yet); every unused one
// when they fit. The chosen ones are marked used. Returns the number left.
static int BinEyes( const dvec3 &c, const std::vector<dvec3> &pts, const std::vector<dvec3> *via, const std::vector<long long> &ids,
					int bins, std::vector<unsigned char> &used, int &unused, std::vector<TileEye> &out )
{
	out.clear();
	const size_t numBins = (size_t)bins * bins;
	if ( (size_t)unused <= numBins )
	{
		for ( size_t i = 0; i < pts.size(); i++ )
		{
			if ( used[i] )
				continue;
			used[i] = 1;
			TileEye te;
			te.p = pts[i];
			dvec3 d = pts[i] - c;
			te.dist2 = DDot( d, d );
			te.viaEye = via ? ( *via )[i] : pts[i];
			te.id = ids[i];
			out.push_back( te );
		}
		unused = 0;
	}
	else
	{
		std::vector<int> best( numBins, -1 );
		std::vector<double> bestD( numBins, 1e300 );
		for ( size_t i = 0; i < pts.size(); i++ )
		{
			if ( used[i] )
				continue;
			dvec3 d = pts[i] - c;
			double d2 = DDot( d, d );
			int bin = OctBin( d, bins );
			if ( d2 < bestD[bin] )
			{
				bestD[bin] = d2;
				best[bin] = (int)i;
			}
		}
		for ( size_t b = 0; b < best.size(); b++ )
		{
			if ( best[b] < 0 )
				continue;
			used[best[b]] = 1;
			unused--;
			TileEye te;
			te.p = pts[best[b]];
			te.dist2 = bestD[b];
			te.viaEye = via ? ( *via )[best[b]] : pts[best[b]];
			te.id = ids[best[b]];
			out.push_back( te );
		}
	}
	std::sort( out.begin(), out.end(), []( const TileEye &a, const TileEye &b ) { return a.dist2 < b.dist2; } );
	return unused;
}

struct FaceThread
{
	bool twoSided;		// the face being traced has no back side
	int facesWidened;	// faces whose grid SampleFace coarsened (summed into VisStats)
	double spacingMax;	// the widest grid any of them got
	std::vector<BwArc> arcs;			// GPU: this tile level's segments
	std::vector<VisSegment> segs;		// the same segments in double: what the CPU legs trace
	std::vector<unsigned char> blocked;
	std::vector<FaceSample> samples;	// the face's (PassFace::samples, lent for one step)
	CandSet sweep;						// SweepThroughMirror: the mirror's candidates
	std::vector<long long> chunk;		// SweepThroughMirror: one chunk of candidates (ids) ...
	std::vector<dvec3> chunkPts;		// ... and their positions
	// the block sweep (SweepBlockOne): per eye of a chunk its index in the block's upload, its rank,
	// its position (in the pinned block) and its id
	std::vector<unsigned int> chunkIdx;
	std::vector<unsigned long long> chunkRank;
	std::vector<const float *> chunkEye;
	std::vector<long long> chunkId;
	std::vector<unsigned long long> sweepPairs;	// the sweep kernel's clear pairs (#114)
	std::vector<size_t> dirGroups;		// DirFreeGroups: the groups clear on the GPU
	std::vector<dvec3> pts, via;		// the eyes a pass traces to (mirrored: the image, via the real eye)
	std::vector<long long> ids;			// ... and the (real) eyes' ids
	std::vector<TileEye> tileEyes;
	std::vector<unsigned char> used;
	std::vector< std::pair<long long, int> > tiles;
	std::vector<int> confirm;			// ConfirmPairs: the samples that can take part (ConfirmSamples)
	int trace = -1;						// -traceface: the face being traced, else -1
	const char *phase = "";				// -traceface: the step TestPair runs for (tiles, confirm, sweep)
	bool cpuReplay = false;				// -traceface: TestPair traces on the CPU, for the log only (#85)
};

// -traceface: names eye 'id' (the eye a clear pair reached) and keeps it as the face's
// seen-from eye.
static void TraceSeenEye( FaceThread &t, const char *pass, long long id )
{
	TraceEyeInfo info = SampleTraceInfo( id );
	TraceEye( t.trace, "seen from", "eye sample", info );
	TraceSeen( t.trace, pass, info );
}
static FaceThread s_FaceThreads[MAX_TOOL_THREADS + 1];

static long long TileKey( const dvec3 &p )
{
	long long x = (long long)floor( p.x / FACE_TILE ) + ( 1 << 20 );
	long long y = (long long)floor( p.y / FACE_TILE ) + ( 1 << 20 );
	long long z = (long long)floor( p.z / FACE_TILE ) + ( 1 << 20 );
	return ( x << 42 ) | ( y << 21 ) | z;
}

void BwVis_EncodeSegment( BwArc &s, const dvec3 &a, const dvec3 &b, float gravity )
{
	s.ox = (float)a.x;
	s.oy = (float)a.y;
	s.oz = (float)a.z;
	s.vx = (float)( b.x - a.x );
	s.vy = (float)( b.y - a.y );
	s.vz = (float)( b.z - a.z );
	s.tKick = 0.0f;
	s.vzKick = 0.0f;
	s.dg0 = -gravity;	// gravity + dg0 = 0 exactly: a straight chord
	s.tEnd = 1.0f;
	s.nSeg[0] = 1;
	s.nSeg[1] = 0;
	s.mesh = BW_MESH_VIS;
}

bool BwVis_TraceSegmentsGpu( const std::vector<VisSegment> &segs, float gravity, std::vector<unsigned char> &blocked )
{
	blocked.assign( segs.size(), 1 );
	if ( segs.empty() )
		return true;
	std::vector<BwArc> arcs( segs.size() );
	for ( size_t i = 0; i < segs.size(); i++ )
		BwVis_EncodeSegment( arcs[i], segs[i].a, segs[i].b, gravity );
	std::lock_guard<std::mutex> lock( s_GpuMutex );
	return BwOptix_TraceArcs( arcs.data(), (unsigned int)arcs.size(), gravity, blocked.data() );
}

// Queues segment a -> b of this tile level: its float launch form, and the
// double endpoints the CPU legs (-visgpusim, a failed launch, -vischeck) trace,
// so they trace what -cpuvis traces.
static void PushSegment( FaceThread &t, const dvec3 &a, const dvec3 &b )
{
	BwArc s;
	BwVis_EncodeSegment( s, a, b, s_VisG );
	t.arcs.push_back( s );
	VisSegment seg = { a, b };
	t.segs.push_back( seg );
}

// How far the segment a -> b is from deciding the other way: the largest, over
// the occluders within 1u of it, of max over t of min_k (d_k - n_k . x(t)). Above
// zero it crosses that occluder by that depth, at or below it misses by about
// that. Concave in t, so a ternary search finds it. -1e300 when no occluder is
// near (a disagreement there is no rounding at all). The grazing distance of
// -arccheck, for a straight segment.
static double SegmentGraze( const dvec3 &a, const dvec3 &b )
{
	double best = -1e300;
	if ( s_OccBvh.nodes.empty() )
		return best;
	dvec3 d = b - a;
	dvec3 inv = SafeInv( d );
	const dvec3 pad( 1, 1, 1 );
	int stack[128];
	int sp = 0;
	stack[sp++] = 0;
	while ( sp )
	{
		const BvhNode &nd = s_OccBvh.nodes[stack[--sp]];
		if ( !SegBox( a, inv, 0.0, 1.0, nd.mins - pad, nd.maxs + pad ) )
			continue;
		if ( nd.left >= 0 )
		{
			if ( sp < 126 )
			{
				stack[sp++] = nd.left;
				stack[sp++] = nd.right;
			}
			continue;
		}
		for ( int i = nd.first; i < nd.first + nd.count; i++ )
		{
			int o = s_OccBvh.items[i];
			best = MAX( best, GrazeConvex( a, b, &s_OccPlanes[s_OccOfs[o]], s_OccOfs[o + 1] - s_OccOfs[o] ) );
		}
	}
	return best;
}

// A failed launch: counted, and the GPU is latched off so later batches go to
// the CPU without retrying (the arc path does the same).
static void GpuLaunchFailed( int iThread )
{
	s_Counters[iThread].check[3]++;
	if ( !s_bVisGpuOff.exchange( true ) )
		Warning( "bspwalk: GPU visibility launch failed; tier B face and mirror rays continue on the CPU\n" );
}

// Groups clear on the GPU that DirFreeGroups tests on helper threads, past this many:
// below it the thread start costs more than the tests (a DirBlocked test is a few us).
static const size_t DIR_PARALLEL_MIN = 4096;
// Groups a helper takes at a time.
static const size_t DIR_PARALLEL_BLOCK = 256;
// Helper threads running for all DirFreeGroups calls together: each call takes at most an even
// share of the budget (hardware threads - 1) among the calls large enough to use helpers that are
// running at that moment, and no more than is left of it, so face pass threads calling at once do
// not multiply it. A call that took the whole budget left every face that called after it on its
// own thread: the candidate sweep's faces of a block call at once, and on d1_trainstation_02 at 4u
// one face spent 2.5 s on its own thread while the others held the helpers (#39 stage B part 2b).
static std::atomic<int> s_DirHelpersBusy( 0 );
static std::atomic<int> s_DirCallers( 0 );

// Is some group of t.arcs (per consecutive segments), all clear on the GPU (t.blocked), also
// clear of the one-way occluders (DirBlocked, viewer at each segment's end)? The answer is an
// OR over the groups, so testing them in any order, on any number of threads, gives the same
// answer as one thread in order. Many GPU-clear groups (#114: behind one-way occluders the GPU
// sees through, d1_trainstation_02 under hl2 sent one face 40.8M segments, all of them blocked,
// 41 s on the one thread that held the face) are tested on helper threads as well; the face
// pass's own threads are mostly idle by then, waiting on that face.
static bool DirFreeGroups( FaceThread &t, int per )
{
	const size_t n = t.segs.size();
	t.dirGroups.clear();
	for ( size_t g = 0; g + per <= n; g += per )
	{
		bool f = true;
		for ( int k = 0; k < per && f; k++ )
			f = !t.blocked[g + k];
		if ( f )
			t.dirGroups.push_back( g );
	}
	const std::vector<size_t> &groups = t.dirGroups;
	auto groupFree = [&]( size_t g ) {
		for ( int k = 0; k < per; k++ )
		{
			if ( DirBlocked( t.segs[g + k].b, t.segs[g + k].a ) )
				return false;
		}
		return true;
	};
	const int budget = MIN( numthreads, (int)std::thread::hardware_concurrency() ) - 1;
	int helpers = 0;
	if ( groups.size() < DIR_PARALLEL_MIN || budget <= 0 )
	{
		for ( size_t i = 0; i < groups.size(); i++ )
		{
			if ( groupFree( groups[i] ) )
				return true;
		}
		return false;
	}
	struct Caller
	{
		Caller() { s_DirCallers.fetch_add( 1 ); }
		~Caller() { s_DirCallers.fetch_sub( 1 ); }
	} caller;
	// the even share (rounded up) of the budget among the calls running now, this one included
	const int callers = MAX( 1, s_DirCallers.load() );
	const int share = ( budget + callers - 1 ) / callers;
	int busy = s_DirHelpersBusy.load();
	do
		helpers = MIN( share, budget - busy );
	while ( helpers > 0 && !s_DirHelpersBusy.compare_exchange_weak( busy, busy + helpers ) );
	if ( helpers <= 0 )
	{
		for ( size_t i = 0; i < groups.size(); i++ )
		{
			if ( groupFree( groups[i] ) )
				return true;
		}
		return false;
	}
	std::atomic<size_t> next( 0 );
	std::atomic<bool> found( false );
	auto work = [&]() {
		for ( ;; )
		{
			if ( found.load( std::memory_order_relaxed ) )
				return;
			size_t a = next.fetch_add( DIR_PARALLEL_BLOCK );
			if ( a >= groups.size() )
				return;
			size_t b = MIN( groups.size(), a + DIR_PARALLEL_BLOCK );
			for ( size_t i = a; i < b; i++ )
			{
				if ( groupFree( groups[i] ) )
				{
					found = true;
					return;
				}
			}
		}
	};
	std::vector<std::thread> pool;
	try
	{
		pool.reserve( helpers );
		for ( int h = 0; h < helpers; h++ )
			pool.emplace_back( work );
	}
	catch ( ... )
	{
		// no more threads: this one and those started share the work
	}
	work();
	for ( size_t h = 0; h < pool.size(); h++ )
		pool[h].join();
	s_DirHelpersBusy.fetch_sub( helpers );
	return found.load();
}

// GPU: is some path of t.arcs (groups of 'per' consecutive segments, all of a
// group clear) free? With -vischeck the CPU traces the same segments in double,
// the answer follows it, and the disagreements are counted and, beyond float
// rounding, reported.
static bool GpuAnyFree( int iThread, FaceThread &t, int per )
{
	size_t n = t.arcs.size();
	bool launched = false;
	if ( !s_bVisSim && !s_bVisGpuOff.load( std::memory_order_relaxed ) )
	{
		t.blocked.assign( n, 1 );
		// slices of VIS_GPU_BATCH segments, each writing its answers at its own offset; one failed
		// slice sends the whole batch to the CPU
		launched = true;
		for ( size_t first = 0; first < n && launched; first += VIS_GPU_BATCH )
		{
			size_t k = MIN( (size_t)VIS_GPU_BATCH, n - first );
			std::lock_guard<std::mutex> lock( s_GpuMutex );
			launched = BwOptix_TraceArcs( t.arcs.data() + first, (unsigned int)k, s_VisG, t.blocked.data() + first );
		}
		if ( !launched )
			GpuLaunchFailed( iThread );
	}
	if ( !launched )
	{
		// -visgpusim, a failed launch or a latched-off GPU: the CPU reference on the
		// same double segments -cpuvis traces, up to the first clear group
		for ( size_t g = 0; g + per <= n; g += per )
		{
			bool f = true;
			for ( int k = 0; k < per && f; k++ )
				f = !SegmentBlocked( t.segs[g + k].a, t.segs[g + k].b );
			if ( f )
				return true;
		}
		return false;
	}
	// The mesh holds the full occluders only: a group clear on the GPU still has to pass the
	// one-way occluders, on the CPU (DirBlocked; viewer at each segment's end, as SegmentBlocked)
	bool gpuFree = false, cpuFree = false;
	if ( !s_bVisCheck && HasDirOccluders() )
		return DirFreeGroups( t, per );
	for ( size_t g = 0; g + per <= n && !( gpuFree && !s_bVisCheck ); g += per )
	{
		bool f = true;
		for ( int k = 0; k < per; k++ )
			f = f && !t.blocked[g + k];
		for ( int k = 0; k < per && f && HasDirOccluders(); k++ )
			f = !DirBlocked( t.segs[g + k].b, t.segs[g + k].a );
		gpuFree = gpuFree || f;
	}
	if ( !s_bVisCheck )
		return gpuFree;
	for ( size_t g = 0; g + per <= n; g += per )
	{
		bool f = true;
		for ( int k = 0; k < per; k++ )
		{
			const VisSegment &sg = t.segs[g + k];
			// compared with what the mesh holds: the full occluders
			bool cb = SegmentBlockedFull( sg.a, sg.b );
			s_Counters[iThread].check[0]++;
			if ( ( t.blocked[g + k] != 0 ) != cb )
			{
				s_Counters[iThread].check[t.blocked[g + k] ? 1 : 2]++;
				double depth = SegmentGraze( sg.a, sg.b );
				// -1e300: no occluder within 1u, so no rounding explains it and there is no distance to report
				bool nearNone = depth < -1e299;
				double dist = fabs( depth );
				if ( !nearNone )
					s_Counters[iThread].checkMaxDist = MAX( s_Counters[iThread].checkMaxDist, dist );
				if ( nearNone || dist > VIS_FLOAT_BOUND )
				{
					s_Counters[iThread].check[4]++;
					if ( s_CheckReported++ < 8 )
					{
						char why[64];
						if ( nearNone )
							Q_strncpy( why, "no occluder within 1u", sizeof( why ) );
						else
							Q_snprintf( why, sizeof( why ), "grazing distance %.6f", depth );
						Warning( "bspwalk: vischeck: segment (%.3f %.3f %.3f) -> (%.3f %.3f %.3f) %s on the GPU, %s on the CPU, %s\n",
								 sg.a.x, sg.a.y, sg.a.z, sg.b.x, sg.b.y, sg.b.z, t.blocked[g + k] ? "blocked" : "free",
								 cb ? "blocked" : "free", why );
					}
				}
			}
			f = f && !cb && !DirBlocked( sg.b, sg.a );
		}
		cpuFree = cpuFree || f;
	}
	if ( cpuFree != gpuFree )
		s_Counters[iThread].checkTiles++;
	return cpuFree;
}

// Pairs go to the GPU queue while the GPU path is on and no launch has failed.
static inline bool VisGpuQueue()
{
	return s_bVisGpu && !s_bVisGpuOff.load( std::memory_order_relaxed );
}

// Does o + t dir, t in [0, tMax], meet the box mins .. maxs grown by tol? 'enter' is the
// first t inside it, 0 for an o inside.
static bool RayEntersBox( const dvec3 &o, const dvec3 &dir, double tMax, const dvec3 &mins, const dvec3 &maxs, double tol,
						  double &enter )
{
	enter = 0.0;
	double t0 = 0.0, t1 = tMax;
	for ( int a = 0; a < 3; a++ )
	{
		double lo = mins[a] - tol, hi = maxs[a] + tol;
		if ( fabs( dir[a] ) < 1e-300 )
		{
			if ( o[a] < lo || o[a] > hi )
				return false;
			continue;
		}
		double ta = ( lo - o[a] ) / dir[a], tb = ( hi - o[a] ) / dir[a];
		if ( ta > tb )
			std::swap( ta, tb );
		t0 = MAX( t0, ta );
		t1 = MIN( t1, tb );
		if ( t0 > t1 )
			return false;
	}
	enter = t0;
	return true;
}

// A displacement mirror's window (#78) for face sample sp whose reflected path to the
// mirrored eye mirEye crosses the reflection plane at q (realEye: the real eye). The
// engine draws, at each pixel the displacement covers, the one reflection texture rendered
// about another surface's plane, looked up at that pixel's screen position: sp shows where
// the real eye's ray through q (on to sp's image) meets the displacement, wherever along it
// the displacement lies. That ray is pair 0. A third-party review read the lookup as the
// mirrored eye's line to sp instead (pair 1, where the segment sp -> mirEye meets the
// displacement); the two agree for a displacement in the plane and differ for one off it.
// Both are kept: a window is either ray meeting the render triangles' bounds grown by tol,
// so the pairs over-cover each reading (the safe side). Each pair is a scene leg and an eye
// leg (legs[2k], legs[2k + 1]), cut short where they enter the bounds (short of the surface:
// less tested, more seen); a leg that has no length (its end inside the bounds) gives way to
// the pair's other leg, so a pair is still two segments. Returns the pairs written (0: no
// window), or -1 when sp and the real eye both lie at the bounds: a window with nothing to
// trace, so seen.
int BwVis_DispMirrorPairs( const dvec3 &mins, const dvec3 &maxs, double tol, const dvec3 &sp, const dvec3 &q,
						   const dvec3 &mirEye, const dvec3 &realEye, VisSegment legs[4] )
{
	int n = 0;
	double enter;
	if ( RayEntersBox( realEye, q - realEye, 1e300, mins, maxs, tol, enter ) )
	{
		legs[0].a = sp;
		legs[0].b = q;
		legs[1] = legs[0];
		if ( enter > 1e-9 )
		{
			legs[1].a = realEye + ( q - realEye ) * enter;
			legs[1].b = realEye;
		}
		n = 1;
	}
	if ( RayEntersBox( sp, mirEye - sp, 1.0, mins, maxs, tol, enter ) )
	{
		// w: where sp's line to the mirrored eye enters the bounds; the real eye sees it along
		// realEye -> w, entering the bounds at or before w
		dvec3 w = sp + ( mirEye - sp ) * enter;
		double e2 = 1.0;
		if ( !RayEntersBox( realEye, w - realEye, 1.0, mins, maxs, tol, e2 ) )
			e2 = 1.0;	// w lies on the bounds, so only rounding misses: w itself
		VisSegment scene = { sp, w }, eye = { realEye + ( w - realEye ) * e2, realEye };
		bool sceneLen = enter > 1e-9, eyeLen = e2 > 1e-9;
		if ( !sceneLen && !eyeLen )
			return -1;	// sample and eye both at the bounds: nothing between them to block
		legs[n * 2] = sceneLen ? scene : eye;
		legs[n * 2 + 1] = eyeLen ? eye : scene;
		n++;
	}
	return n;
}

// One sample-to-eye pair (eye: the point traced to; viaEye: the real eye
// behind the mirror pR, or eye itself). On the GPU path its segments are
// queued in t.arcs / t.segs and the result is 0. Returns -1 when the pair cannot
// see (back side, behind the mirror, misses the mirror), 1 for a clear CPU ray.
static int TestPair( int iThread, FaceThread &t, const FaceSample &s, const dvec3 &eye, const dvec3 &viaEye,
					 const VisFace *pR, long long id )
{
	if ( pR && DDot( pR->n, s.p ) <= pR->d )
		return -1;	// behind the mirror: not in its reflection
	dvec3 d = eye - s.p;
	if ( !t.twoSided && DDot( d, s.n ) <= 0 )
		return -1;	// the sample's back side
	if ( !pR )
	{
		if ( VisGpuQueue() && !t.cpuReplay )
		{
			PushSegment( t, s.p, eye );
			return 0;
		}
		if ( !t.cpuReplay )
			s_Counters[iThread].rays[1]++;
		if ( SegmentBlocked( s.p, eye ) )
			return 0;
		if ( t.trace >= 0 )
		{
			TraceF( t.trace, "  face pass (%s): clear ray to the eye at (%.4f %.4f %.4f) (leaf cluster %d)\n", t.phase, eye.x,
					eye.y, eye.z, dleafs[PointLeaf( eye )].cluster );
			TraceSeenEye( t, "face pass", id );
			TraceSegment( t.trace, "sample -> eye", s.p, eye );
		}
		return 1;
	}
	double den = DDot( pR->n, d );
	if ( fabs( den ) < 1e-12 )
		return -1;
	double u = ( pR->d - DDot( pR->n, s.p ) ) / den;
	if ( u <= 0 || u >= 1 )
		return -1;
	dvec3 q = s.p + d * u;
	// (sample leg, eye leg) pairs, any clear one a path: a face mirror's one, sample -> q -> eye;
	// a displacement's from BwVis_DispMirrorPairs
	VisSegment legs[4];
	int n = 1;
	if ( pR->disp )
	{
		n = BwVis_DispMirrorPairs( pR->mins, pR->maxs, MIRROR_TOL, s.p, q, eye, viaEye, legs );
		if ( n < 0 )
		{
			if ( t.trace >= 0 )
				TraceF( t.trace, "  mirror pass (%s): sample and eye (%.4f %.4f %.4f) both at the displacement's bounds\n",
						t.phase, viaEye.x, viaEye.y, viaEye.z );
			return 1;
		}
	}
	else
	{
		// a face mirror's window: q inside its polygon
		if ( !InsidePolygon( *pR, q, MIRROR_TOL ) )
			return -1;
		legs[0].a = s.p;
		legs[0].b = q;
		legs[1].a = q;
		legs[1].b = viaEye;
	}
	if ( n == 0 )
		return -1;
	if ( VisGpuQueue() && !t.cpuReplay )
	{
		for ( int k = 0; k < n * 2; k++ )
			PushSegment( t, legs[k].a, legs[k].b );
		return 0;
	}
	for ( int k = 0; k < n; k++ )
	{
		if ( !t.cpuReplay )
			s_Counters[iThread].rays[2]++;
		const VisSegment &ls = legs[k * 2], &le = legs[k * 2 + 1];
		if ( SegmentBlocked( ls.a, ls.b ) || SegmentBlocked( le.a, le.b ) )
			continue;
		if ( t.trace >= 0 )
		{
			TraceF( t.trace, "  mirror pass (%s): clear path to the eye at (%.4f %.4f %.4f) (leaf cluster %d) through the mirror at "
					"(%.4f %.4f %.4f), pair %d of %d\n", t.phase, viaEye.x, viaEye.y, viaEye.z, dleafs[PointLeaf( viaEye )].cluster,
					q.x, q.y, q.z, k + 1, n );
			TraceSeenEye( t, "mirror pass", id );
			TraceSegment( t.trace, "sample -> mirror", ls.a, ls.b );
			TraceSegment( t.trace, pR->disp ? "displacement bounds -> eye" : "mirror -> eye", le.a, le.b );
		}
		return 1;
	}
	return 0;
}

// Traces the queued pairs (GpuAnyFree: sliced launches, or the CPU on the same
// double segments after a failed launch, with -vischeck's comparison); true when
// one is clear. The queue is emptied.
static bool FlushPairs( int iThread, FaceThread &t, bool mirror )
{
	if ( t.arcs.empty() )
		return false;
	s_Counters[iThread].rays[mirror ? 2 : 1] += (long long)t.arcs.size();
	bool any = GpuAnyFree( iThread, t, mirror ? 2 : 1 );
	t.arcs.clear();
	t.segs.clear();
	return any;
}

static unsigned long long Mix64( unsigned long long x )
{
	x += 0x9e3779b97f4a7c15ULL;
	x = ( x ^ ( x >> 30 ) ) * 0xbf58476d1ce4e5b9ULL;
	x = ( x ^ ( x >> 27 ) ) * 0x94d049bb133111ebULL;
	return x ^ ( x >> 31 );
}

size_t BwVis_ConfirmSample( unsigned long long key, unsigned long long eye, int r, int rays, size_t nS )
{
	// stratified with a hashed jitter per stratum: an even stride alone can
	// match the grid's row length and put every ray on one row
	unsigned long long h = Mix64( key ^ eye );
	double u = (double)( Mix64( h + (unsigned long long)r ) >> 11 ) * ( 1.0 / 9007199254740992.0 );
	return MIN( nS - 1, (size_t)( ( r + u ) * (double)nS / rays ) );
}

// The samples a confirmation pairs eyes with (t.confirm): every sample of the
// face, or for a mirror pass those in front of the mirror.
static void ConfirmSamples( FaceThread &t, int mirror )
{
	const VisFace *pR = mirror >= 0 ? &s_MirrorFace[mirror] : NULL;
	t.confirm.clear();
	for ( size_t k = 0; k < t.samples.size(); k++ )
	{
		if ( !pR || DDot( pR->n, t.samples[k].p ) > pR->d )
			t.confirm.push_back( (int)k );
	}
}

// The last word before a face is called hidden (#25): the binned tiles pick
// the nearest eye per direction, so an eye that sees the face through a gap
// past nearer, blocked eyes is never tried. Here every eye of t.pts (binned
// or not) is paired with confirmRays samples of t.confirm (ConfirmSamples:
// the whole face, or the part in front of the mirror for a mirror pass), one
// per stratum of the sample list at a hashed offset: a different,
// deterministic set per eye. The hash takes each eye's place in the list it
// belongs to: ranks[i] when given (the block sweep: the eye's rank among the
// face's candidates, #39 stage B part 2b), else base + i, where 'base' is
// t.pts[0]'s place in the list of eyes SweepThroughMirror has fed to this pass
// so far (0 for the binned tiles' confirmation). A mirror sweep drops the
// candidates behind the mirror, or whose image is behind the face, before they
// become t.pts, so its base advances by the eyes kept, not by the place in the
// full list; the ids are still unique per (face, mirror), which is all the hash
// needs. True at the first clear ray.
static bool ConfirmPairs( int iThread, FaceThread &t, int mirror, int fi, size_t base = 0, const unsigned long long *ranks = NULL )
{
	int rays = MIN( s_pVisOpts->confirmRays, VIS_MAX_CONFIRM_RAYS );
	if ( rays <= 0 )
		return false;
	const VisFace *pR = mirror >= 0 ? &s_MirrorFace[mirror] : NULL;
	size_t nS = t.confirm.size();
	if ( nS == 0 )
		return false;
	// early exits between flushes; GpuAnyFree slices each flush to VIS_GPU_BATCH
	const size_t FLUSH = 1 << 18;
	t.arcs.clear();
	t.segs.clear();
	for ( size_t i = 0; i < t.pts.size(); i++ )
	{
		const dvec3 &viaEye = pR ? t.via[i] : t.pts[i];
		const unsigned long long key = ( (unsigned long long)fi << 32 ) ^ ( (unsigned long long)( mirror + 1 ) << 48 );
		for ( int r = 0; r < rays; r++ )
		{
			size_t k = BwVis_ConfirmSample( key, ranks ? ranks[i] : base + i, r, rays, nS );
			if ( TestPair( iThread, t, t.samples[t.confirm[k]], t.pts[i], viaEye, pR, t.ids[i] ) == 1 )
				return true;
			// a pair's segments are queued in one call: a flush never splits one
			if ( t.arcs.size() >= FLUSH && FlushPairs( iThread, t, pR != NULL ) )
				return true;
		}
	}
	return FlushPairs( iThread, t, pR != NULL );
}

// Tests the samples tile by tile against the binned points; true at the first
// clear ray. A tile that finds none tries again with twice the bins over the
// eyes it has not tried, up to binLevels times. Then ConfirmPairs, unless
// 'confirm' is false: t.pts is then a thinned set and the candidate sweep
// (RunSweep, SweepThroughMirror) confirms over every candidate instead. mirror >= 0: points are mirrored eyes behind
// s_Reflective[mirror].
static bool TraceTiles( int iThread, FaceThread &t, int mirror, int fi, bool confirm )
{
	// bspwalk.cpp clamps both options; the limits are repeated here because the shift below is
	// undefined for a level count past 63. Bins per axis stay within 1024 (bins * bins within
	// int): the doubled levels are trimmed to that, and the first level is at most VIS_MAX_BINS.
	int bins0 = MAX( 1, MIN( VIS_MAX_BINS, s_pVisOpts->bins ) );
	int levels = MAX( 1, MIN( VIS_MAX_BIN_LEVELS, s_pVisOpts->binLevels ) );
	while ( levels > 1 && ( (long long)bins0 << ( levels - 1 ) ) > 1024 )
		levels--;
	const VisFace *pR = mirror >= 0 ? &s_MirrorFace[mirror] : NULL;
	t.phase = "binned tiles";
	for ( size_t a = 0; a < t.tiles.size(); )
	{
		size_t b = a;
		dvec3 c( 0, 0, 0 );
		while ( b < t.tiles.size() && t.tiles[b].first == t.tiles[a].first )
		{
			c += t.samples[t.tiles[b].second].p;
			b++;
		}
		c = c * ( 1.0 / ( b - a ) );
		t.used.assign( t.pts.size(), 0 );
		int unused = (int)t.pts.size();
		for ( int level = 0; level < levels && unused > 0; level++ )
		{
			BinEyes( c, t.pts, mirror >= 0 ? &t.via : NULL, t.ids, bins0 << level, t.used, unused, t.tileEyes );
			// every pair of this tile level (GPU: in one launch)
			t.arcs.clear();
			t.segs.clear();
			for ( size_t k = a; k < b; k++ )
			{
				const FaceSample &s = t.samples[t.tiles[k].second];
				for ( size_t e = 0; e < t.tileEyes.size(); e++ )
				{
					if ( TestPair( iThread, t, s, t.tileEyes[e].p, t.tileEyes[e].viaEye, pR, t.tileEyes[e].id ) == 1 )
						return true;
				}
			}
			if ( FlushPairs( iThread, t, pR != NULL ) )
				return true;
		}
		a = b;
	}
	if ( !confirm )
		return false;
	t.phase = "-eyeconfirm pairs";
	ConfirmSamples( t, mirror );
	if ( !ConfirmPairs( iThread, t, mirror, fi ) )
		return false;
	if ( !t.cpuReplay )
		s_Counters[iThread].confirmHits++;
	return true;
}

// Eyes per chunk of a candidate sweep (the block sweep's launches and ConfirmPairs calls, and the
// mirror sweep's CandidateChunk): memory stays bounded and a clear pair ends a face early.
static const size_t SWEEP_CHUNK = 1 << 18;

// A face hidden so far in front of mirror r whose own candidates passed the cap (#35): the tiles
// binned the mirror's thinned set, so before the face can be tier B every candidate of the mirror
// goes through the confirmation pairs, counted again here (t.sweep) and read SWEEP_CHUNK at a time,
// mirrored and culled as the mirror loop culls s_MirrorEyes. True at the first clear ray. Nothing
// calls this with -eyeconfirm 0 (the mirror loop then keeps the face visible). Left face-major on
// purpose (#39 stage B part 2b): its hash position advances by the eyes kept after mirroring, which
// has no prefix sum without another count pass per (face, mirror); it pins one tile at a time. Every
// chunk goes through ConfirmPairs (its segments on the GPU path when there is one).
static bool SweepThroughMirror( int iThread, FaceThread &t, int mirror, int fi )
{
	const VisFace &f = s_Faces[fi];
	const VisFace *pR = &s_MirrorFace[mirror];
	// a displacement mirror's eyes are culled by the plane it shows, below, not its own
	CountCandidates( s_Reflective[mirror], !pR->disp, t.sweep );
	t.phase = "candidate sweep";
	ConfirmSamples( t, mirror );
	const bool cull = !f.twoSided && !f.disp;
	size_t base = 0;
	s_Counters[iThread].sweptMirror++;
	while ( CandidateChunk( t.sweep, SWEEP_CHUNK, t.chunk, t.chunkPts ) )
	{
		t.pts.clear();
		t.via.clear();
		t.ids.clear();
		for ( size_t k = 0; k < t.chunk.size(); k++ )
		{
			const dvec3 &p = t.chunkPts[k];
			// as the mirror eyes and the mirror loop: in front of the mirror, and its image in front of the face
			if ( DDot( pR->n, p ) <= pR->d + 1e-6 )
				continue;
			dvec3 m = MirrorPoint( p, pR->n, pR->d );
			if ( cull && DDot( f.n, m ) <= f.d + 1e-6 )
				continue;
			t.pts.push_back( m );
			t.via.push_back( p );
			t.ids.push_back( t.chunk[k] );
		}
		s_Counters[iThread].sweepEyes += (long long)t.pts.size();
		if ( !t.pts.empty() && ConfirmPairs( iThread, t, mirror, fi, base ) )
			return true;
		base += t.pts.size();
	}
	return false;
}

static bool FaceInPvsOf( int fi, int c )
{
	for ( int k = s_FaceClOfs[fi]; k < s_FaceClOfs[fi + 1]; k++ )
	{
		if ( s_FaceCl[k] == c || PvsBit( c, s_FaceCl[k] ) )
			return true;
	}
	return false;
}

// May an eye cluster the lattice missed (no sample to ray test from) see the
// face: is one of the face's clusters in its PVS (or the cluster itself), with
// part of the cluster's eye region (its k-DOP bound) in front of the face's plane
// (any part for a two-sided face or a displacement)?
static bool BareMaySee( const VisFace &f, int fi, bool cull )
{
	const std::vector<int> &bare = s_pStore->BareClusters();
	const std::vector<EyeBound> &region = s_pStore->BareRegions();
	for ( size_t i = 0; i < bare.size(); i++ )
	{
		if ( cull && region[i].Support( f.n, f.d + REGION_MARGIN ) <= f.d + REGION_MARGIN )
			continue;
		if ( !s_bBarePvs || FaceInPvsOf( fi, bare[i] ) )
			return true;
	}
	return false;
}

// Some part of face fi lies in front of reflective face R.
static bool InFrontOfMirror( const VisFace &f, const VisFace &R )
{
	const std::vector<dvec3> &pts = f.disp ? *f.tris : f.w;
	for ( size_t k = 0; k < pts.size(); k++ )
	{
		if ( DDot( R.n, pts[k] ) > R.d + 1e-6 )
			return true;
	}
	return false;
}

//-----------------------------------------------------------------------------
// Candidate jobs (#39 stage B part 2b). A job is something that needs candidate eyes: a face of
// the face pass, a mirror's status, or a mirror's eyes. Its buckets are the eye buckets that may
// see it (CountCandidates' PVS test), each with its BucketSide answer against the job's plane; a
// bucket's runs are its samples tile by tile. The candidate order is bucket ascending, run by run,
// index ascending, and an eye's rank is its place among the job's passing eyes in that order.
// Once every (job, run) knows how many of its eyes pass, every rank is known without walking
// anything in order (a run's first rank is a prefix sum), so the passes that read eye samples
// take the tiles in any order: each thread pins one tile, serves every job with a run in it, and
// unpins it. No loop over eye samples is job-major, and a tile is pinned only while it is read.
//-----------------------------------------------------------------------------
// Batches (faces, or mirror jobs) are closed once one of these is reached, so the buckets, runs,
// candidate lists and face samples the passes hold besides the eye cache are bounded by constants,
// never by the map or by free memory; the last group set up may pass a bound by one group. The
// mirror eyes are not under these bounds: every mirror a batch's hidden faces lie in front of is
// gathered at once and held through that batch's mirror loop (at most the cap's eyes a mirror, about
// 2 MB); MIRROR_EYE_BYTES only decides what is kept across batches (#140).
static const double JOB_RUN_BYTES = 512.0 * 1024 * 1024;		// buckets, runs and tile entries
static const double JOB_CAND_BYTES = 1024.0 * 1024 * 1024;		// candidate lists (an id and float xyz each)
static const double JOB_SAMPLE_BYTES = 1024.0 * 1024 * 1024;	// face samples, kept from setup to the mirror loop
static const int JOB_SETUP_GROUP = 64;							// jobs (faces) set up at a time
static const int FACE_BATCH_MAX = 8192;							// faces in a batch at most

struct JobRun
{
	long long first;		// the rank of its first passing eye (JobFinishOne)
	int run;				// index into s_BucketRuns
	int pass;				// its passing eyes (side 1: all of them; side 2: CountTileOne)
	int side;				// 1 every eye passes, 2 each is tested
};
struct JobBucket
{
	int c, side;			// the bucket and its BucketSide answer (1 when the job does not cull)
	int firstRun, numRuns;	// its runs in CandJob::runs: none for side 0 unless the job is traced
	long long pass;			// its passing eyes (JobFinishOne)
};
struct JobEntry				// one (job, run) a tile-major pass serves
{
	int job, jr;
};
struct CandJob
{
	int fi;					// the face whose clusters pick the buckets (a mirror's own face)
	int trace;				// -traceface: the log TraceRegion writes to, or -1
	DPlane plane;			// the back-face test: an eye passes when plane.n . e > plane.d + 1e-6
	bool cull;				// false: every eye of a bucket passes (side 1)
	bool gather;			// list the candidates (GatherTileOne)
	int planeId;			// jobs on one plane (bit for bit) share a run's count (CountTileOne)
	long long total;		// the eyes of its buckets, passing or not
	long long maySee;		// ... of the buckets whose side is not 0: at least the candidates
	std::vector<JobBucket> buckets;
	std::vector<JobRun> runs;
	std::vector<double> best;	// traced: per run, the most an eye reaches past the plane
	long long n;			// candidates (JobFinishOne)
	bool thinned;			// n > FACE_MAX_CANDIDATES: the list holds CapPositions' picks only
	bool untestable;		// CountCandidates' untestable-region rule holds
	std::vector<size_t> keep;	// thinned: the picks' ranks, ascending
	std::vector<long long> ids;	// the candidates (GatherTileOne): eye ids ...
	std::vector<float> xyz;		// ... and positions, in candidate order
	CandJob()
		: fi( -1 ), trace( -1 ), cull( false ), gather( false ), planeId( 0 ), total( 0 ), maySee( 0 ), n( 0 ), thinned( false ),
		  untestable( false )
	{
		plane.n = dvec3( 0, 0, 0 );
		plane.d = 0;
	}
	// what JOB_RUN_BYTES counts
	double RunBytes() const
	{
		return (double)buckets.size() * sizeof( JobBucket ) + (double)runs.size() * ( sizeof( JobRun ) + sizeof( JobEntry ) ) +
			   (double)best.size() * sizeof( double );
	}
	// what JOB_CAND_BYTES counts: at most this many candidates are listed
	double CandBytes() const
	{
		return gather ? (double)MIN( maySee, (long long)FACE_MAX_CANDIDATES ) * ( sizeof( long long ) + 3 * sizeof( float ) ) : 0.0;
	}
};

// The job's buckets with their sides and runs, from the bucket boxes alone (no eye data).
static void JobSetup( CandJob &j )
{
	j.buckets.clear();
	j.runs.clear();
	j.best.clear();
	j.keep.clear();
	j.ids.clear();
	j.xyz.clear();
	j.total = j.maySee = j.n = 0;
	j.thinned = j.untestable = false;
	const int nB = (int)s_BucketCount.size();
	// a face with no cluster has no PVS row to match: as without vis data, every bucket may see it
	const bool pvs = !s_Pvs.empty() && s_FaceClOfs[j.fi] < s_FaceClOfs[j.fi + 1];
	for ( int c = 0; c < nB; c++ )
	{
		if ( !s_BucketCount[c] )
			continue;
		if ( pvs )
		{
			bool any = false;
			for ( int k = s_FaceClOfs[j.fi]; k < s_FaceClOfs[j.fi + 1] && !any; k++ )
				any = PvsBit( c, s_FaceCl[k] );
			if ( !any )
				continue;
		}
		JobBucket b;
		b.c = c;
		b.side = j.cull ? BucketSide( j.plane.n, j.plane.d, c ) : 1;
		b.firstRun = (int)j.runs.size();
		b.pass = 0;
		j.total += s_BucketCount[c];
		if ( b.side )
			j.maySee += s_BucketCount[c];
		// a traced job counts its side-0 buckets' eyes too (none passes, BucketSide's guarantee), for
		// the reach TraceRegion prints
		if ( b.side || j.trace >= 0 )
		{
			for ( int r = 0; r < BucketRunCount( c ); r++ )
			{
				JobRun jr;
				jr.first = 0;
				jr.run = s_BucketOfs[c] + r;
				jr.side = b.side == 1 ? 1 : 2;
				jr.pass = b.side == 1 ? s_BucketRuns[jr.run].count : 0;
				j.runs.push_back( jr );
			}
		}
		b.numRuns = (int)j.runs.size() - b.firstRun;
		j.buckets.push_back( b );
	}
	if ( j.trace >= 0 )
		j.best.assign( j.runs.size(), -1e300 );
}

// The jobs of the tile-major pass running, and its (job, run) entries grouped by tile (CSR).
static std::vector<CandJob *> s_PassJobs;
static std::vector<int> s_EntryOfs;			// per tile: its first entry; NumTiles() + 1 entries
static std::vector<JobEntry> s_Entries;
static std::vector<int> s_EntryTiles;		// the tiles with an entry, ascending

// Count pass (gather false): every run each job tests eye by eye (side 2). Gather pass: every run
// that adds to a job's list (passing eyes, and when thinned a pick among them).
static void BuildEntries( bool gather )
{
	const int nT = s_pStore->NumTiles();
	s_EntryOfs.assign( (size_t)nT + 1, 0 );
	auto want = [gather]( const CandJob &j, const JobRun &jr ) -> bool {
		if ( !gather )
			return jr.side == 2;
		if ( jr.pass <= 0 )
			return false;
		if ( !j.thinned )
			return true;
		std::vector<size_t>::const_iterator it = std::lower_bound( j.keep.begin(), j.keep.end(), (size_t)jr.first );
		return it != j.keep.end() && *it < (size_t)( jr.first + jr.pass );
	};
	for ( size_t q = 0; q < s_PassJobs.size(); q++ )
	{
		const CandJob &j = *s_PassJobs[q];
		if ( gather && !j.gather )
			continue;
		for ( size_t k = 0; k < j.runs.size(); k++ )
		{
			if ( want( j, j.runs[k] ) )
				s_EntryOfs[(size_t)s_BucketRuns[j.runs[k].run].tile + 1]++;
		}
	}
	for ( int t = 0; t < nT; t++ )
		s_EntryOfs[(size_t)t + 1] += s_EntryOfs[t];
	s_Entries.resize( (size_t)s_EntryOfs[nT] );
	std::vector<int> cur( s_EntryOfs.begin(), s_EntryOfs.end() - 1 );
	for ( size_t q = 0; q < s_PassJobs.size(); q++ )
	{
		const CandJob &j = *s_PassJobs[q];
		if ( gather && !j.gather )
			continue;
		for ( size_t k = 0; k < j.runs.size(); k++ )
		{
			if ( !want( j, j.runs[k] ) )
				continue;
			JobEntry e = { (int)q, (int)k };
			s_Entries[cur[s_BucketRuns[j.runs[k].run].tile]++] = e;
		}
	}
	s_EntryTiles.clear();
	for ( int t = 0; t < nT; t++ )
	{
		if ( s_EntryOfs[(size_t)t + 1] > s_EntryOfs[t] )
			s_EntryTiles.push_back( t );
	}
}

static void FreeEntries()
{
	std::vector<int>().swap( s_EntryOfs );
	std::vector<JobEntry>().swap( s_Entries );
	std::vector<int>().swap( s_EntryTiles );
}

// Jobs on one plane, bit for bit, get one plane id: their side-2 runs in a tile are counted once.
static void AssignPlaneIds()
{
	std::map< std::vector<unsigned long long>, int > ids;
	std::vector<unsigned long long> key( 4 );
	for ( size_t q = 0; q < s_PassJobs.size(); q++ )
	{
		CandJob &j = *s_PassJobs[q];
		const double v[4] = { j.plane.n.x, j.plane.n.y, j.plane.n.z, j.plane.d };
		memcpy( key.data(), v, sizeof( v ) );
		std::map< std::vector<unsigned long long>, int >::iterator it = ids.find( key );
		if ( it == ids.end() )
			it = ids.insert( std::make_pair( key, (int)ids.size() ) ).first;
		j.planeId = it->second;
	}
}

// Count pass, one tile: the passing eyes of every (job, side-2 run) in it (and for a traced job the
// most an eye reaches past its plane). The entries of one run and one plane share the count.
static void CountTileOne( int, int item )
{
	const int tile = s_EntryTiles[item];
	JobEntry *e = s_Entries.data() + s_EntryOfs[tile];
	const int m = s_EntryOfs[(size_t)tile + 1] - s_EntryOfs[tile];
	std::sort( e, e + m, []( const JobEntry &a, const JobEntry &b ) {
		const CandJob &ja = *s_PassJobs[a.job], &jb = *s_PassJobs[b.job];
		const int ra = ja.runs[a.jr].run, rb = jb.runs[b.jr].run;
		if ( ra != rb )
			return ra < rb;
		if ( ja.planeId != jb.planeId )
			return ja.planeId < jb.planeId;
		return a.job != b.job ? a.job < b.job : a.jr < b.jr;
	} );
	const EyeTileData &d = s_pStore->Pin( tile );
	for ( int a = 0; a < m; )
	{
		const CandJob &j0 = *s_PassJobs[e[a].job];
		const int run = j0.runs[e[a].jr].run, plane = j0.planeId;
		int b = a;
		bool traced = false;
		while ( b < m && s_PassJobs[e[b].job]->runs[e[b].jr].run == run && s_PassJobs[e[b].job]->planeId == plane )
		{
			traced = traced || s_PassJobs[e[b].job]->trace >= 0;
			b++;
		}
		const EyeRun &er = s_BucketRuns[run];
		const float *p = &d.xyz[(size_t)er.start * 3];
		const dvec3 &n = j0.plane.n;
		const double pd = j0.plane.d;
		int count = 0;
		double best = -1e300;
		if ( traced )
		{
			for ( int i = 0; i < er.count; i++, p += 3 )
			{
				count += EyeInFront( n, pd, p );
				best = MAX( best, n.x * p[0] + n.y * p[1] + n.z * p[2] - pd );
			}
		}
		else
		{
			for ( int i = 0; i < er.count; i++, p += 3 )
				count += EyeInFront( n, pd, p );
		}
		for ( int k = a; k < b; k++ )
		{
			CandJob &j = *s_PassJobs[e[k].job];
			j.runs[e[k].jr].pass = count;
			if ( j.trace >= 0 )
				j.best[e[k].jr] = best;
		}
		a = b;
	}
	s_pStore->Unpin( tile );
}

// After the count: per bucket its passing eyes, CountCandidates' untestable-region rule (in bucket
// order, with its -traceface lines), n, every run's first rank, and past the cap CapPositions'
// picks (a gather job only).
static void JobFinishOne( int, int item )
{
	CandJob &j = *s_PassJobs[item];
	const bool pvs = !s_Pvs.empty();
	j.n = 0;
	j.untestable = false;
	for ( size_t q = 0; q < j.buckets.size(); q++ )
	{
		JobBucket &b = j.buckets[q];
		long long pass = 0;
		if ( b.side == 1 )
			pass = s_BucketCount[b.c];
		else
		{
			for ( int k = 0; k < b.numRuns; k++ )
				pass += j.runs[(size_t)b.firstRun + k].pass;
		}
		b.pass = pass;
		if ( !pass )
		{
			// no sample in front: the bucket's pieces may still reach in front
			if ( j.cull && !j.untestable && pvs )
			{
				j.untestable = RegionInFront( j.plane.n, j.plane.d, b.c );
				if ( j.trace >= 0 && b.c < s_pStore->NumClusters() )
				{
					double best = -1e300;
					for ( int k = 0; k < b.numRuns; k++ )
						best = MAX( best, j.best[(size_t)b.firstRun + k] );
					TraceRegion( j.trace, j.plane.n, j.plane.d, b.c, j.untestable, best );
				}
			}
			continue;
		}
		j.n += pass;
	}
	if ( j.cull && j.n == 0 && !pvs )
		j.untestable = RegionInFront( j.plane.n, j.plane.d, -1 );	// no eye in front, the pieces may reach it
	long long rank = 0;
	for ( size_t q = 0; q < j.buckets.size(); q++ )
	{
		const JobBucket &b = j.buckets[q];
		if ( !b.pass )
			continue;
		for ( int k = 0; k < b.numRuns; k++ )
		{
			JobRun &jr = j.runs[(size_t)b.firstRun + k];
			jr.first = rank;
			rank += jr.pass;
		}
	}
	j.thinned = j.n > FACE_MAX_CANDIDATES;
	j.keep.clear();
	if ( j.thinned && j.gather )
	{
		// each bucket with a passing eye is one cluster run of CapCandidateEyes
		std::vector<size_t> runSize;
		for ( size_t q = 0; q < j.buckets.size(); q++ )
		{
			if ( j.buckets[q].pass )
				runSize.push_back( (size_t)j.buckets[q].pass );
		}
		CapPositions( runSize, FACE_MAX_CANDIDATES, j.keep );
	}
}

// The count pass over the jobs: tile by tile, then each job finished.
static void RunCount( std::vector<CandJob *> &jobs )
{
	s_PassJobs.swap( jobs );
	AssignPlaneIds();
	BuildEntries( false );
	RunThreadsBatched( (int)s_EntryTiles.size(), CountTileOne );
	FreeEntries();
	// a tile remade here (past the cache budget) left the generator's scratch, one per thread that made
	// one: freed now, so it never sits beside the next pass's buffers
	TrimEyeScratch();
	RunThreadsBatched( (int)s_PassJobs.size(), JobFinishOne );
	s_PassJobs.swap( jobs );
	s_PassJobs.clear();
}

// Jobs made by a callback and run in chunks (RunJobsChunked).
static void ( *s_JobMake )( int k, CandJob &j );
static std::vector<CandJob> s_ChunkJobs;
static int s_ChunkAt, s_ChunkBase;

static void JobMakeOne( int, int item )
{
	s_JobMake( s_ChunkBase + item, s_ChunkJobs[(size_t)s_ChunkAt + item] );
}

static void RunGather( std::vector<CandJob *> &jobs );

// Jobs 0 .. count-1, made by make(k, job) (JobSetup inside; any thread) JOB_SETUP_GROUP at a time and
// run in chunks within the batch bounds: the count pass, with 'gather' the gather pass, then
// done(k, job) on the main thread in order.
static void RunJobsChunked( int count, bool gather, void ( *make )( int, CandJob & ), void ( *done )( int, CandJob & ) )
{
	s_JobMake = make;
	int next = 0;
	while ( next < count )
	{
		s_ChunkJobs.clear();
		const int first = next;
		double runBytes = 0, candBytes = 0;
		while ( next < count && runBytes <= JOB_RUN_BYTES && candBytes <= JOB_CAND_BYTES )
		{
			const int k = MIN( JOB_SETUP_GROUP, count - next );
			const size_t at = s_ChunkJobs.size();
			s_ChunkJobs.resize( at + k );
			s_ChunkAt = (int)at;
			s_ChunkBase = next;
			RunThreadsBatched( k, JobMakeOne );
			for ( int i = 0; i < k; i++ )
			{
				runBytes += s_ChunkJobs[at + i].RunBytes();
				candBytes += s_ChunkJobs[at + i].CandBytes();
			}
			next += k;
		}
		std::vector<CandJob *> jobs;
		for ( size_t i = 0; i < s_ChunkJobs.size(); i++ )
			jobs.push_back( &s_ChunkJobs[i] );
		RunCount( jobs );
		if ( gather )
			RunGather( jobs );
		for ( size_t i = 0; i < s_ChunkJobs.size(); i++ )
			done( first + (int)i, s_ChunkJobs[i] );
	}
	std::vector<CandJob>().swap( s_ChunkJobs );
	s_JobMake = NULL;
}

// Gather pass, one tile: every listed candidate of every (job, run) entry in it, written at the
// place its rank gives (past the cap: the place of its pick). A side-1 run needs no plane test.
static void GatherTileOne( int, int item )
{
	const int tile = s_EntryTiles[item];
	const JobEntry *e = s_Entries.data() + s_EntryOfs[tile];
	const int m = s_EntryOfs[(size_t)tile + 1] - s_EntryOfs[tile];
	const EyeTileData &d = s_pStore->Pin( tile );
	for ( int a = 0; a < m; a++ )
	{
		CandJob &j = *s_PassJobs[e[a].job];
		const JobRun &jr = j.runs[e[a].jr];
		const EyeRun &er = s_BucketRuns[jr.run];
		const float *p = &d.xyz[(size_t)er.start * 3];
		const dvec3 &n = j.plane.n;
		const double pd = j.plane.d;
		if ( !j.thinned )
		{
			size_t pos = (size_t)jr.first;
			const size_t end = (size_t)( jr.first + jr.pass );
			for ( int i = 0; i < er.count; i++ )
			{
				const float *q = p + (size_t)i * 3;
				if ( jr.side == 2 && !EyeInFront( n, pd, q ) )
					continue;
				// checked before the write: one eye past the count pass's would land in the next run's place
				if ( pos == end )
					Error( "bspwalk: eye tile %d run gave more passing eyes on the gather pass than the %d of the count pass\n",
						   tile, jr.pass );
				j.ids[pos] = EyeId( tile, er.start + i );
				memcpy( &j.xyz[pos * 3], q, 3 * sizeof( float ) );
				pos++;
			}
			if ( pos != end )
				Error( "bspwalk: eye tile %d run gave %lld passing eyes on the gather pass, %d on the count pass\n", tile,
					   (long long)pos - jr.first, jr.pass );
			continue;
		}
		const size_t lo = (size_t)jr.first, hi = (size_t)( jr.first + jr.pass );
		size_t k = std::lower_bound( j.keep.begin(), j.keep.end(), lo ) - j.keep.begin();
		const size_t kEnd = std::lower_bound( j.keep.begin(), j.keep.end(), hi ) - j.keep.begin();
		if ( jr.side == 1 )
		{
			for ( ; k < kEnd; k++ )
			{
				const int i = (int)( j.keep[k] - lo );
				j.ids[k] = EyeId( tile, er.start + i );
				memcpy( &j.xyz[k * 3], p + (size_t)i * 3, 3 * sizeof( float ) );
			}
			continue;
		}
		size_t jj = 0;	// place among the run's passing eyes
		for ( int i = 0; i < er.count && k < kEnd; i++ )
		{
			const float *q = p + (size_t)i * 3;
			if ( !EyeInFront( n, pd, q ) )
				continue;
			if ( lo + jj == j.keep[k] )
			{
				j.ids[k] = EyeId( tile, er.start + i );
				memcpy( &j.xyz[k * 3], q, 3 * sizeof( float ) );
				k++;
			}
			jj++;
		}
		if ( k != kEnd )
			Error( "bspwalk: eye tile %d run lost %d picks on the gather pass\n", tile, (int)( kEnd - k ) );
	}
	s_pStore->Unpin( tile );
}

// The gather pass over the jobs that list their candidates (CandJob::gather): their lists sized,
// then filled tile by tile.
static void RunGather( std::vector<CandJob *> &jobs )
{
	s_PassJobs.swap( jobs );
	for ( size_t q = 0; q < s_PassJobs.size(); q++ )
	{
		CandJob &j = *s_PassJobs[q];
		if ( !j.gather )
			continue;
		const size_t len = j.thinned ? j.keep.size() : (size_t)j.n;
		j.ids.assign( len, -1 );
		j.xyz.assign( len * 3, 0.0f );
	}
	BuildEntries( true );
	RunThreadsBatched( (int)s_EntryTiles.size(), GatherTileOne );
	FreeEntries();
	TrimEyeScratch();	// as RunCount
	s_PassJobs.swap( jobs );
	s_PassJobs.clear();
}

// Mirror status (#39 stage B part 2b): s_MirrorBare and s_MirrorCapped of every mirror a face of the
// face pass lies in front of, from counts alone, in one tile-major count pass before the face pass.
// Each is a pure function of the mirror (MirrorStatusDone has the rules), so working out more of
// them than the face pass reads changes no verdict.
static std::vector<unsigned char> s_MirrorStatus;	// per s_Reflective: its status is set
static std::vector<int> s_StatusMirrors;			// the mirrors MirrorStatusPass works out
static std::vector<unsigned char> s_MirrorNeed;

static void MirrorNeedOne( int, int r )
{
	const VisFace &R = s_MirrorFace[r];
	for ( size_t i = 0; i < s_PassFaces.size(); i++ )
	{
		const int fi = s_PassFaces[i];
		if ( fi != s_Reflective[r] && InFrontOfMirror( s_Faces[fi], R ) )
		{
			s_MirrorNeed[r] = 1;
			return;
		}
	}
}

// The status job: the eyes that may see the mirror, culled by the plane it shows (a displacement's
// candidates are not culled by their own plane: JobFinishOne's untestable rule on the plane it
// shows is what the status needs, and the count of every eye that may see it decides the cap).
static void MirrorStatusMake( int k, CandJob &j )
{
	const int r = s_StatusMirrors[k];
	const VisFace &R = s_MirrorFace[r];
	j.fi = s_Reflective[r];
	j.trace = !R.disp && Traced( j.fi ) ? j.fi : -1;
	j.plane.n = R.n;
	j.plane.d = R.d;
	j.cull = true;
	j.gather = false;
	JobSetup( j );
}

// MIRROR_BARE when a bare eye cluster may see the mirror from in front, or an eye region that may
// see it is untestable against the plane it shows (JobFinishOne's rule); else MIRROR_SOLID when a
// viewpoint sample in solid lies in front of it; else MIRROR_TESTED. Capped when more eyes may see
// it than the cap: those in front of it, or for a displacement every one (its candidates are not
// culled by its own plane).
static void MirrorStatusDone( int k, CandJob &j )
{
	const int r = s_StatusMirrors[k], f = s_Reflective[r];
	const VisFace &R = s_MirrorFace[r];
	unsigned char bare = BareMaySee( R, f, true ) || j.untestable ? MIRROR_BARE : MIRROR_TESTED;
	const bool capped = ( R.disp ? j.total : j.n ) > FACE_MAX_CANDIDATES;
	s_MirrorCapped[r] = capped;
	if ( capped )
		s_FaceCapped[f].store( 1, std::memory_order_relaxed );
	if ( bare == MIRROR_TESTED && SolidViewInFront( R, true ) )
		bare = MIRROR_SOLID;
	s_MirrorBare[r] = bare;
	s_MirrorStatus[r] = 1;
}

static void MirrorStatusPass()
{
	const int nR = (int)s_Reflective.size();
	s_MirrorStatus.assign( nR, 0 );
	s_MirrorNeed.assign( nR, 0 );
	RunThreadsBatched( nR, MirrorNeedOne );
	s_StatusMirrors.clear();
	for ( int r = 0; r < nR; r++ )
	{
		if ( s_MirrorNeed[r] )
			s_StatusMirrors.push_back( r );
	}
	RunJobsChunked( (int)s_StatusMirrors.size(), false, MirrorStatusMake, MirrorStatusDone );
	std::vector<int>().swap( s_StatusMirrors );
	std::vector<unsigned char>().swap( s_MirrorNeed );
}

// Does face fi lie partly in front of a mirror that a bare eye cluster may see
// (s_MirrorBare MIRROR_BARE)? The mirror loop (FaceMirrorOne) keeps such a face visible with
// no ray test whenever the face's own rays all fail, so the face is visible
// whatever they find: deciding that first skips them. On fightspace3 one large
// face spent 148 s in direct rays that all failed before the mirror loop kept it
// visible this way. The face's reason is then no_eye where a clear ray would
// have made it ray; its tier is the same. A MIRROR_SOLID mirror is left to the
// mirror loop, which tries the rays first so that solid_view counts only the
// faces the rule keeps.
static bool InFrontOfBareMirror( int fi )
{
	for ( size_t r = 0; r < s_Reflective.size(); r++ )
	{
		if ( s_Reflective[r] == fi || !InFrontOfMirror( s_Faces[fi], s_MirrorFace[r] ) )
			continue;
		if ( !s_MirrorStatus[r] )
			Error( "bspwalk: mirror %d's status read before MirrorStatusPass set it\n", (int)r );
		if ( s_MirrorBare[r] == MIRROR_BARE )
			return true;
	}
	return false;
}

//-----------------------------------------------------------------------------
// The face pass (#39 stage B part 2b). The faces of s_PassFaces go in batches of consecutive faces,
// closed by the JOB_* bounds (BSPWALK_FACE_BATCH forces a smaller one, a test hook). Per batch:
//   A (FaceSetupOne, per face, no eye data): the bare-cluster and bare-mirror rules, the face's
//     samples, its job (buckets, sides, runs);
//   B (RunCount, tile-major): the passing eyes of every (face, side-2 run); per face then the
//     untestable-region rule, n, the thinning picks;
//   C (RunGather, tile-major): each face's candidate list, every passing eye or the picks;
//   D (FaceTraceOne, per face): the binned tiles, with the confirmation pairs unless thinned;
//   E (RunSweep, block-major): the thinned faces D left hidden, every candidate paired;
//   F (GatherMirrorEyes, then FaceMirrorOne per face): the mirror loop and the viewpoints in solid.
// Each face meets the rules in the order the one-face-at-a-time pass had them, with the same
// candidates, picks and pair hashes, so the same verdict; only the order of the work changed.
//-----------------------------------------------------------------------------
// A face of the face pass, from its setup to the end of its batch.
struct PassFace
{
	int fi;
	bool done;							// decided (VisibleSet called)
	bool swept;							// its candidates passed the cap: the candidate sweep pairs them all
	int slot;							// the sweep: its GPU sample slot, or -1
	size_t cursor;						// the sweep: its next entry of byTile
	std::vector<FaceSample> samples;	// SampleFace's, kept to the mirror loop
	std::vector<int> byTile;			// the sweep: its runs (into job.runs) with a passing eye, by tile
	CandJob job;
	PassFace() : fi( -1 ), done( false ), swept( false ), slot( -1 ), cursor( 0 ) {}
};
static std::vector<PassFace> s_Batch;
static size_t s_BatchAt;			// FaceSetupOne: the batch's first face of the group being set up

// Hands a face's samples to the thread's FaceThread (TestPair, TraceTiles and ConfirmPairs read
// t.samples) for one step, and takes them back.
struct FaceSamples
{
	FaceThread &t;
	PassFace &p;
	FaceSamples( FaceThread &ft, PassFace &pf ) : t( ft ), p( pf ) { t.samples.swap( p.samples ); }
	~FaceSamples() { t.samples.swap( p.samples ); }
	FaceSamples( const FaceSamples & ) = delete;
	FaceSamples &operator=( const FaceSamples & ) = delete;
};

static void FaceDecide( PassFace &p, int why )
{
	VisibleSet( p.fi, why + 1 );
	p.done = true;
}

// The face's samples grouped in FACE_TILE cubes (t.tiles), for TraceTiles.
static void FaceTiles( FaceThread &t )
{
	t.tiles.clear();
	for ( size_t i = 0; i < t.samples.size(); i++ )
		t.tiles.push_back( std::make_pair( TileKey( t.samples[i].p ), (int)i ) );
	std::sort( t.tiles.begin(), t.tiles.end() );
}

// A: the rules that need no candidate, the samples and the job.
static void FaceSetupOne( int iThread, int item )
{
	PassFace &p = s_Batch[s_BatchAt + item];
	const int fi = p.fi;
	if ( VisibleGet( fi ) )
	{
		p.done = true;
		return;
	}
	FaceThread &t = s_FaceThreads[iThread];
	const VisFace &f = s_Faces[fi];
	const bool traced = Traced( fi );
	// No ray test is possible from an eye cluster without samples (a piece that only grazes a thin
	// leaf, missing every vertex and lattice point) that may see the face, or for a face without
	// samples: it stays visible. Clusters added for reflective faces are not eye piece clusters; the
	// mirror pass decides those. The first two tests need no samples, so they run before the face is
	// sampled.
	if ( BareMaySee( f, fi, !f.twoSided && !f.disp ) || InFrontOfBareMirror( fi ) )
	{
		if ( traced )
			TraceF( fi, "  face pass: no_eye: a bare eye cluster may see it, or it lies in front of a mirror one may see\n" );
		FaceDecide( p, WHY_NOEYE );
		return;
	}
	double spUsed;
	SampleFace( f, p.samples, spUsed );
	if ( spUsed > s_pVisOpts->faceSpacing )
	{
		t.facesWidened++;
		t.spacingMax = MAX( t.spacingMax, spUsed );
	}
	if ( p.samples.empty() )
	{
		if ( traced )
			TraceF( fi, "  face pass: no_eye: no face sample\n" );
		FaceDecide( p, WHY_NOEYE );
		return;
	}
	// The eyes that may see the face: those in clusters whose PVS holds one of its clusters (all eyes
	// without vis data, or for a face with no cluster), in front of its plane unless it is two-sided
	// or a displacement. With vis data the samples of viewpoints in solid hold cluster -1, so they are
	// in no bucket and not candidates (SolidSeesFace decides for them); without vis data they are
	// bucketed with every other eye and are candidates too.
	CandJob &j = p.job;
	j.fi = fi;
	j.trace = traced ? fi : -1;
	j.plane.n = f.n;
	j.plane.d = f.d;
	j.cull = !f.twoSided && !f.disp;
	j.gather = true;
	JobSetup( j );
}

// D: the binned tiles over the face's candidates (all of them, or past the cap CapPositions' picks),
// with the confirmation pairs unless the list is thinned.
static void FaceTraceOne( int iThread, int item )
{
	PassFace &p = s_Batch[item];
	if ( p.done )
		return;
	const int fi = p.fi;
	FaceThread &t = s_FaceThreads[iThread];
	FaceSamples hold( t, p );
	t.trace = Traced( fi ) ? fi : -1;
	t.phase = "";
	// Displacement samples are not back-face culled: a folded displacement's triangles can face away
	// from its base face, so a sample normal forced to the base side would cull real front-side eyes.
	t.twoSided = s_Faces[fi].twoSided || s_Faces[fi].disp;
	FaceTiles( t );
	CandJob &j = p.job;
	const size_t n = j.ids.size();
	t.ids.assign( j.ids.begin(), j.ids.end() );
	t.pts.resize( n );
	for ( size_t k = 0; k < n; k++ )
		t.pts[k] = dvec3( j.xyz[k * 3], j.xyz[k * 3 + 1], j.xyz[k * 3 + 2] );
	std::vector<long long>().swap( j.ids );
	std::vector<float>().swap( j.xyz );
	if ( t.trace >= 0 )
		TraceF( fi, "  face pass: %d samples, %d candidate eyes%s; GPU path: %s\n", (int)t.samples.size(), (int)t.ids.size(),
				j.thinned ? " (thinned past the cap)" : "", VisGpuQueue() ? "yes (pairs not named; -cpuvis names them)" : "no" );
	if ( !t.pts.empty() && TraceTiles( iThread, t, -1, fi, !j.thinned ) )
	{
		if ( t.trace >= 0 )
		{
			TraceF( fi, "  face pass: ray (%s)\n", t.phase );
			if ( VisGpuQueue() )
			{
				// the GPU names no pair (#85): the same pairs again on the CPU, for the log only
				TraceF( fi, "  face pass: the same pairs replayed on the CPU for this log (the verdict is the GPU's):\n" );
				t.cpuReplay = true;
				const bool cpu = TraceTiles( iThread, t, -1, fi, !j.thinned );
				t.cpuReplay = false;
				t.arcs.clear();
				t.segs.clear();
				if ( !cpu )
					TraceF( fi, "  face pass: the CPU replay found no clear pair (the GPU's margins let one through)\n" );
			}
		}
		FaceDecide( p, WHY_RAY );
		return;
	}
	// Past the cap the tiles bin a thinned set; no face may be called hidden from that alone (#35):
	// every candidate then goes through the candidate sweep (E), or with -eyeconfirm 0 (no
	// confirmation pairs to sweep with) the face stays visible untested.
	if ( j.thinned )
	{
		if ( s_pVisOpts->confirmRays <= 0 )
		{
			s_Counters[iThread].cappedKept++;
			FaceDecide( p, WHY_NOEYE );
			return;
		}
		p.swept = true;
		if ( t.trace >= 0 )
			TraceF( fi, "  face pass: no clear pair in the binned tiles (a thinned set): every candidate goes to the candidate "
						"sweep\n" );
	}
}

//-----------------------------------------------------------------------------
// E: the candidate sweep (#35), block-major. The faces the binned tiles left hidden whose
// candidates passed the cap: before such a face can be tier B every candidate goes through the
// confirmation pairs (ConfirmPairs' rule: confirmRays face samples per eye, picked by the hash of
// the face, the eye's rank and the ray). The eye tiles their candidates lie in are walked in tile
// order in blocks of at most SWEEP_BLOCK_EYES samples (or one larger tile); per block they are
// pinned once and, for the sweep kernel, uploaded once, and each pending face pairs its passing
// eyes of the block, SWEEP_CHUNK at a time, by rank (JobRun::first and on). The verdict is an OR
// over a fixed set of pairs, so the order changes only where a face stops: at its first clear pair.
// Blocks in tile order and each face's work in a block on one thread keep the ray counters the same
// at any thread count. On the GPU path the sweep kernel (__raygen__sweep) pairs on the device, 4 B
// an eye and 8 B a rank instead of 48 B a segment; -cpuvis, -vischeck, -visgpusim, a failed launch
// or a device that cannot hold a block take the same blocks through ConfirmPairs.
//-----------------------------------------------------------------------------
// Samples per block: 48 MB of float positions to upload, the most the pinned block holds.
static const size_t SWEEP_BLOCK_EYES = (size_t)1 << 22;
// The kernel's listed clear pairs are checked against the one-way occluders this many at a time.
static const size_t SWEEP_PAIR_WINDOW = 1 << 18;
static std::vector<PassFace *> s_Pending;
static std::vector<int> s_BlockTiles;		// the block's tiles, ascending (pinned by the main thread)
static std::vector<size_t> s_BlockOfs;		// ... each one's first sample in the block's upload (+ the total)
static bool s_BlockGpu;						// the block's eyes are on the device
static int s_BlockIndex;
static bool s_SweepWarned;

// SweepEyes' fill: samples [first, first + count) of the block, its tiles one after another.
static void BlockEyeFill( void *, size_t first, size_t count, float *out )
{
	size_t k = std::upper_bound( s_BlockOfs.begin(), s_BlockOfs.end(), first ) - s_BlockOfs.begin() - 1;
	while ( count && k < s_BlockTiles.size() )
	{
		const EyeTileData &d = s_pStore->Pinned( s_BlockTiles[k] );
		const size_t at = first - s_BlockOfs[k];
		const size_t take = MIN( count, (size_t)d.Count() - at );
		memcpy( out, &d.xyz[at * 3], take * 3 * sizeof( float ) );
		out += take * 3;
		first += take;
		count -= take;
		k++;
	}
}

// One chunk of a face's sweep (t.chunkIdx, chunkRank, chunkEye, chunkId: each eye's index in the
// block's upload, its rank, its position and its id). True at a clear pair the one-way occluders
// let through.
static bool SweepChunk( int iThread, FaceThread &t, PassFace &p )
{
	const size_t n = t.chunkRank.size();
	if ( !n )
		return false;
	const int fi = p.fi;
	const int rays = MIN( s_pVisOpts->confirmRays, VIS_MAX_CONFIRM_RAYS );
	const unsigned long long key = (unsigned long long)fi << 32;
	if ( s_BlockGpu && p.slot >= 0 && !s_bVisGpuOff.load( std::memory_order_relaxed ) )
	{
		bool clear = false, overflow = false, ok;
		{
			std::lock_guard<std::mutex> lock( s_GpuMutex );
			ok = BwOptix_SweepTrace( p.slot, t.chunkIdx.data(), t.chunkRank.data(), (unsigned int)n, (unsigned int)rays, t.twoSided,
									 key, clear, HasDirOccluders() ? &t.sweepPairs : NULL, &overflow );
		}
		// a clear pair with one-way occluders and an overflowing list: the chunk goes through
		// ConfirmPairs below, whose GPU groups GpuAnyFree checks against them
		if ( ok && !( clear && HasDirOccluders() && overflow ) )
		{
			s_Counters[iThread].rays[1] += (long long)n * rays;
			s_Counters[iThread].sweepEyes += (long long)n;
			bool seen = clear;
			if ( clear && HasDirOccluders() )
			{
				// #114: the kernel listed its clear pairs; only they can be free, and only those the
				// one-way occluders let through are. The pair's sample is the one the kernel and
				// ConfirmPairs pick (BwVis_ConfirmSample), its segment the one TestPair queues. A
				// window at a time: the segments stay small and the first free one ends the face. An
				// OR over the pairs, so the windows give the answer the whole list does.
				seen = false;
				for ( size_t w = 0; w < t.sweepPairs.size() && !seen; w += SWEEP_PAIR_WINDOW )
				{
					const size_t wEnd = MIN( t.sweepPairs.size(), w + SWEEP_PAIR_WINDOW );
					t.segs.clear();
					for ( size_t q = w; q < wEnd; q++ )
					{
						const size_t i = (size_t)( t.sweepPairs[q] / (unsigned long long)rays );
						const int r = (int)( t.sweepPairs[q] % (unsigned long long)rays );
						const size_t k = BwVis_ConfirmSample( key, t.chunkRank[i], r, rays, t.confirm.size() );
						const float *e = t.chunkEye[i];
						VisSegment seg = { t.samples[t.confirm[k]].p, dvec3( e[0], e[1], e[2] ) };
						t.segs.push_back( seg );
					}
					t.blocked.assign( t.segs.size(), 0 );
					seen = DirFreeGroups( t, 1 );
				}
				t.segs.clear();
				// FaceThread storage is static: a long list must not keep its memory for the run
				if ( t.sweepPairs.capacity() > SWEEP_PAIR_WINDOW )
					std::vector<unsigned long long>().swap( t.sweepPairs );
			}
			if ( seen && t.trace >= 0 )
			{
				// the kernel names no pair (#85): the chunk's pairs again on the CPU, for the log only
				TraceF( fi, "  face pass: candidate sweep block %d: the sweep kernel found a clear pair; the chunk's pairs "
							"replayed on the CPU for this log (the verdict is the GPU's):\n", s_BlockIndex );
				t.pts.resize( n );
				t.ids.assign( t.chunkId.begin(), t.chunkId.end() );
				for ( size_t i = 0; i < n; i++ )
					t.pts[i] = dvec3( t.chunkEye[i][0], t.chunkEye[i][1], t.chunkEye[i][2] );
				t.cpuReplay = true;
				const bool cpu = ConfirmPairs( iThread, t, -1, fi, 0, t.chunkRank.data() );
				t.cpuReplay = false;
				t.arcs.clear();
				t.segs.clear();
				if ( !cpu )
					TraceF( fi, "  face pass: the CPU replay found no clear pair (the GPU's margins let one through)\n" );
			}
			return seen;
		}
		if ( !ok )
			GpuLaunchFailed( iThread );	// this chunk and the rest go through ConfirmPairs (on the CPU from now on)
	}
	t.pts.resize( n );
	t.ids.assign( t.chunkId.begin(), t.chunkId.end() );
	for ( size_t i = 0; i < n; i++ )
		t.pts[i] = dvec3( t.chunkEye[i][0], t.chunkEye[i][1], t.chunkEye[i][2] );
	s_Counters[iThread].sweepEyes += (long long)n;
	return ConfirmPairs( iThread, t, -1, fi, 0, t.chunkRank.data() );
}

static void SweepChunkClear( FaceThread &t )
{
	t.chunkIdx.clear();
	t.chunkRank.clear();
	t.chunkEye.clear();
	t.chunkId.clear();
}

// E, one block, one pending face: its passing eyes of the block's tiles, by rank, a chunk at a time.
static void SweepBlockOne( int iThread, int item )
{
	PassFace &p = *s_Pending[item];
	if ( p.done )
		return;
	const int fi = p.fi;
	FaceThread &t = s_FaceThreads[iThread];
	FaceSamples hold( t, p );
	t.trace = Traced( fi ) ? fi : -1;
	t.phase = "candidate sweep";
	t.twoSided = s_Faces[fi].twoSided || s_Faces[fi].disp;
	ConfirmSamples( t, -1 );
	const CandJob &j = p.job;
	const int lastTile = s_BlockTiles.back();
	SweepChunkClear( t );
	bool seen = false;
	while ( !seen && p.cursor < p.byTile.size() )
	{
		const JobRun &jr = j.runs[p.byTile[p.cursor]];
		const EyeRun &er = s_BucketRuns[jr.run];
		if ( er.tile > lastTile )
			break;
		p.cursor++;
		const size_t bt = std::lower_bound( s_BlockTiles.begin(), s_BlockTiles.end(), er.tile ) - s_BlockTiles.begin();
		if ( bt >= s_BlockTiles.size() || s_BlockTiles[bt] != er.tile )
			Error( "bspwalk: the candidate sweep's block lacks eye tile %d\n", er.tile );
		const float *e = &s_pStore->Pinned( er.tile ).xyz[(size_t)er.start * 3];
		unsigned long long rank = (unsigned long long)jr.first;
		for ( int i = 0; i < er.count && !seen; i++, e += 3 )
		{
			if ( jr.side == 2 && !EyeInFront( j.plane.n, j.plane.d, e ) )
				continue;
			t.chunkIdx.push_back( (unsigned int)( s_BlockOfs[bt] + er.start + i ) );
			t.chunkRank.push_back( rank++ );
			t.chunkEye.push_back( e );
			t.chunkId.push_back( EyeId( er.tile, er.start + i ) );
			if ( t.chunkRank.size() >= SWEEP_CHUNK )
			{
				seen = SweepChunk( iThread, t, p );
				SweepChunkClear( t );
			}
		}
		if ( !seen && rank != (unsigned long long)( jr.first + jr.pass ) )
			Error( "bspwalk: eye tile %d run gave %lld passing eyes on the candidate sweep, %d on the count pass\n", er.tile,
				   (long long)( rank - jr.first ), jr.pass );
	}
	if ( !seen )
		seen = SweepChunk( iThread, t, p );
	SweepChunkClear( t );
	if ( !seen )
		return;
	if ( t.trace >= 0 )
		TraceF( fi, "  face pass: ray (candidate sweep, block %d)\n", s_BlockIndex );
	s_Counters[iThread].sweptFaces++;
	s_Counters[iThread].sweptVisible++;
	s_Counters[iThread].confirmHits++;
	FaceDecide( p, WHY_RAY );
}

static void RunSweep()
{
	s_Pending.clear();
	for ( size_t i = 0; i < s_Batch.size(); i++ )
	{
		if ( !s_Batch[i].done && s_Batch[i].swept )
			s_Pending.push_back( &s_Batch[i] );
	}
	if ( s_Pending.empty() )
		return;
	const int rays = MIN( s_pVisOpts->confirmRays, VIS_MAX_CONFIRM_RAYS );
	// the sweep kernel is the arc kernel's segments paired on the device: not with -vischeck or
	// -visgpusim, which compare or simulate the arc kernel's segments
	const bool kernel = s_bVisGpu && !s_bVisSim && !s_bVisCheck && rays > 0 && !s_bVisGpuOff.load();
	std::vector<int> tiles;
	std::vector<double> host;
	for ( size_t q = 0; q < s_Pending.size(); q++ )
	{
		PassFace &p = *s_Pending[q];
		const CandJob &j = p.job;
		p.byTile.clear();
		p.cursor = 0;
		p.slot = -1;
		for ( size_t k = 0; k < j.runs.size(); k++ )
		{
			if ( j.runs[k].pass > 0 )
				p.byTile.push_back( (int)k );
		}
		std::sort( p.byTile.begin(), p.byTile.end(), [&j]( int a, int b ) {
			const int ta = s_BucketRuns[j.runs[a].run].tile, tb = s_BucketRuns[j.runs[b].run].tile;
			return ta != tb ? ta < tb : a < b;
		} );
		for ( size_t k = 0; k < p.byTile.size(); k++ )
			tiles.push_back( s_BucketRuns[j.runs[p.byTile[k]].run].tile );
		if ( kernel )
		{
			// the confirmation samples of a direct sweep are every sample of the face (ConfirmSamples)
			host.resize( p.samples.size() * 6 );
			for ( size_t k = 0; k < p.samples.size(); k++ )
			{
				const FaceSample &s = p.samples[k];
				double *o = &host[k * 6];
				o[0] = s.p.x, o[1] = s.p.y, o[2] = s.p.z, o[3] = s.n.x, o[4] = s.n.y, o[5] = s.n.z;
			}
			std::lock_guard<std::mutex> lock( s_GpuMutex );
			if ( BwOptix_SweepSamples( (int)q, host.data(), (unsigned int)p.samples.size() ) )
				p.slot = (int)q;
		}
		if ( Traced( p.fi ) )
			TraceF( p.fi, "  face pass: candidate sweep: %lld candidates, block by block in tile order (%s)\n", j.n,
					p.slot >= 0 ? "the sweep kernel" : "ConfirmPairs" );
	}
	std::vector<double>().swap( host );
	std::sort( tiles.begin(), tiles.end() );
	tiles.erase( std::unique( tiles.begin(), tiles.end() ), tiles.end() );
	s_BlockIndex = 0;
	for ( size_t a = 0; a < tiles.size(); )
	{
		bool open = false;
		for ( size_t q = 0; q < s_Pending.size() && !open; q++ )
			open = !s_Pending[q]->done;
		if ( !open )
			break;
		size_t b = a, eyes = 0;
		while ( b < tiles.size() && ( b == a || eyes + (size_t)s_pStore->TileCount( tiles[b] ) <= SWEEP_BLOCK_EYES ) )
			eyes += (size_t)s_pStore->TileCount( tiles[b++] );
		s_BlockTiles.assign( tiles.begin() + a, tiles.begin() + b );
		s_BlockOfs.assign( 1, 0 );
		for ( size_t k = 0; k < s_BlockTiles.size(); k++ )
			s_BlockOfs.push_back( s_BlockOfs.back() + (size_t)s_pStore->Pin( s_BlockTiles[k] ).Count() );
		s_BlockGpu = false;
		bool anySlot = false;
		for ( size_t q = 0; q < s_Pending.size() && !anySlot; q++ )
			anySlot = !s_Pending[q]->done && s_Pending[q]->slot >= 0;
		if ( anySlot && !s_bVisGpuOff.load() )
		{
			std::lock_guard<std::mutex> lock( s_GpuMutex );
			s_BlockGpu = BwOptix_SweepEyes( s_BlockOfs.back(), BlockEyeFill, NULL );
		}
		if ( anySlot && !s_BlockGpu && !s_SweepWarned )
		{
			s_SweepWarned = true;
			Warning( "bspwalk: the GPU cannot hold a candidate sweep block (%.0f eye samples); it traces through the arc kernel "
					 "instead (slower)\n", (double)s_BlockOfs.back() );
		}
		RunThreadsBatched( (int)s_Pending.size(), SweepBlockOne );
		for ( size_t k = 0; k < s_BlockTiles.size(); k++ )
			s_pStore->Unpin( s_BlockTiles[k] );
		TrimEyeScratch();	// as RunCount (the block's tiles were made on this thread)
		a = b;
		s_BlockIndex++;
	}
	if ( kernel )
	{
		std::lock_guard<std::mutex> lock( s_GpuMutex );
		BwOptix_SweepFree();
	}
	for ( size_t q = 0; q < s_Pending.size(); q++ )
	{
		PassFace &p = *s_Pending[q];
		if ( !p.done && Traced( p.fi ) )
			TraceF( p.fi, "  face pass: the candidate sweep found no clear pair (%d blocks)\n", s_BlockIndex );
		std::vector<int>().swap( p.byTile );
	}
	s_Pending.clear();
	std::vector<int>().swap( s_BlockTiles );
	std::vector<size_t>().swap( s_BlockOfs );
}

//-----------------------------------------------------------------------------
// F: the mirror loop and the viewpoints in solid, for each face still hidden. Before it, the eyes
// of the mirrors those faces lie in front of (and may trace through) that are not gathered yet, as
// gather jobs (s_MirrorEyes, s_MirrorIds: the candidates, or past the cap the picks, in front of
// the mirror). Every mirror the batch needs is gathered in this one call and held through the
// batch's mirror loop, so the batch holds the eyes of all of them together (at most the cap's
// 65,536 a mirror, about 2 MB with their ids); no constant bounds that sum (#140). After the loop
// the eyes are kept for later batches while they total at most MIRROR_EYE_BYTES, else all dropped
// and gathered again when asked for.
//-----------------------------------------------------------------------------
static const double MIRROR_EYE_BYTES = 512.0 * 1024 * 1024;
static std::vector<unsigned char> s_MirrorGathered;	// per s_Reflective: s_MirrorEyes holds its eyes
static std::vector<int> s_GatherMirrors;

// The mirror's candidates: those in front of it, except a displacement's, which are every eye that
// may see it (culled below by the plane it shows), as the status counted them.
static void MirrorEyesMake( int k, CandJob &j )
{
	const int r = s_GatherMirrors[k];
	const VisFace &R = s_MirrorFace[r];
	j.fi = s_Reflective[r];
	j.trace = -1;
	j.plane.n = R.n;
	j.plane.d = R.d;
	j.cull = !R.disp;
	j.gather = true;
	JobSetup( j );
}

static void MirrorEyesDone( int k, CandJob &j )
{
	const int r = s_GatherMirrors[k];
	const VisFace &R = s_MirrorFace[r];
	if ( j.thinned != ( s_MirrorCapped[r] != 0 ) )
		Error( "bspwalk: mirror %d's eyes are %s, its status says %s\n", r, j.thinned ? "thinned" : "all", s_MirrorCapped[r] ? "capped" : "not" );
	std::vector<dvec3> &me = s_MirrorEyes[r];
	std::vector<long long> &mid = s_MirrorIds[r];
	me.clear();
	mid.clear();
	for ( size_t q = 0; q < j.ids.size(); q++ )
	{
		const dvec3 p( j.xyz[q * 3], j.xyz[q * 3 + 1], j.xyz[q * 3 + 2] );
		if ( DDot( R.n, p ) > R.d + 1e-6 )
		{
			me.push_back( p );
			mid.push_back( j.ids[q] );
		}
	}
	s_MirrorGathered[r] = 1;
	std::vector<long long>().swap( j.ids );
	std::vector<float>().swap( j.xyz );
}

static void GatherMirrorEyes()
{
	const int nR = (int)s_Reflective.size();
	std::vector<unsigned char> need( nR, 0 );
	for ( size_t i = 0; i < s_Batch.size(); i++ )
	{
		const PassFace &p = s_Batch[i];
		if ( p.done )
			continue;
		for ( int r = 0; r < nR; r++ )
		{
			if ( need[r] || s_MirrorGathered[r] || s_Reflective[r] == p.fi || !InFrontOfMirror( s_Faces[p.fi], s_MirrorFace[r] ) )
				continue;
			if ( !s_MirrorStatus[r] )
				Error( "bspwalk: mirror %d's status read before MirrorStatusPass set it\n", r );
			// a bare mirror's faces were decided in A (InFrontOfBareMirror)
			need[r] = s_MirrorBare[r] != MIRROR_BARE;
		}
	}
	s_GatherMirrors.clear();
	for ( int r = 0; r < nR; r++ )
	{
		if ( need[r] )
			s_GatherMirrors.push_back( r );
	}
	RunJobsChunked( (int)s_GatherMirrors.size(), true, MirrorEyesMake, MirrorEyesDone );
	s_GatherMirrors.clear();
}

// F, one face still hidden: through each reflective face that stays visible, then the viewpoints in
// solid.
static void FaceMirrorOne( int iThread, int item )
{
	PassFace &p = s_Batch[item];
	if ( p.done )
		return;
	const int fi = p.fi;
	FaceThread &t = s_FaceThreads[iThread];
	FaceSamples hold( t, p );
	t.trace = Traced( fi ) ? fi : -1;
	t.phase = "";
	t.twoSided = s_Faces[fi].twoSided || s_Faces[fi].disp;
	const bool sweep = s_pVisOpts->confirmRays > 0;
	bool swept = p.swept;
	const VisFace &f = s_Faces[fi];
	bool tiled = false;
	for ( size_t r = 0; r < s_Reflective.size(); r++ )
	{
		if ( s_Reflective[r] == fi )
			continue;
		const VisFace &R = s_MirrorFace[r];
		// some part of the face must lie in front of the mirror
		if ( !InFrontOfMirror( f, R ) )
			continue;
		if ( s_MirrorBare[r] != MIRROR_BARE && !s_MirrorGathered[r] )
			Error( "bspwalk: mirror %d's eyes read before they were gathered\n", (int)r );
		// no gathered eye and nothing else to try: skip the mirror. A capped mirror goes on with an
		// empty list: its gathered eyes are only the thinned picks, which can all lie behind the plane
		// it shows (a displacement mirror) while other candidates are in front, so the sweep below
		// (SweepThroughMirror, every candidate) or, with -eyeconfirm 0, the untested keep still applies
		if ( s_MirrorEyes[r].empty() && !s_MirrorBare[r] && !s_MirrorCapped[r] )
			continue;
		if ( s_MirrorBare[r] == MIRROR_BARE )
		{
			// a bare eye cluster may see the mirror: no ray test is possible
			if ( t.trace >= 0 )
				TraceF( fi, "  mirror pass: no_eye: it lies in front of mirror face %d, which no ray test can speak for\n",
						s_Reflective[r] );
			s_Counters[iThread].sweptFaces += swept;
			FaceDecide( p, WHY_NOEYE );
			return;
		}
		if ( !tiled )
		{
			FaceTiles( t );
			tiled = true;
		}
		t.pts.clear();
		t.via.clear();
		t.ids.clear();
		bool cull = !f.twoSided && !f.disp;
		const std::vector<dvec3> &me = s_MirrorEyes[r];
		const std::vector<long long> &mid = s_MirrorIds[r];
		for ( size_t k = 0; k < me.size(); k++ )
		{
			dvec3 m = MirrorPoint( me[k], R.n, R.d );
			if ( cull && DDot( f.n, m ) <= f.d + 1e-6 )
				continue;
			t.pts.push_back( m );
			t.via.push_back( me[k] );
			t.ids.push_back( mid[k] );
		}
		if ( !t.pts.empty() && TraceTiles( iThread, t, (int)r, fi, !s_MirrorCapped[r] ) )
		{
			if ( t.trace >= 0 && VisGpuQueue() )
			{
				// as the face pass: the GPU names no pair (#85), so replay them on the CPU for the log
				TraceF( fi, "  mirror pass: mirror face %d's pairs replayed on the CPU for this log (the verdict is the GPU's):\n",
						s_Reflective[r] );
				t.cpuReplay = true;
				const bool cpu = TraceTiles( iThread, t, (int)r, fi, !s_MirrorCapped[r] );
				t.cpuReplay = false;
				t.arcs.clear();
				t.segs.clear();
				if ( !cpu )
					TraceF( fi, "  mirror pass: the CPU replay found no clear pair (the GPU's margins let one through)\n" );
			}
			s_Counters[iThread].sweptFaces += swept;
			FaceDecide( p, WHY_MIRROR );
			return;
		}
		if ( !s_MirrorCapped[r] )
			continue;
		// the mirror's own candidates passed the cap: s_MirrorEyes[r] is its thinned set
		if ( !sweep )
		{
			s_Counters[iThread].sweptFaces += swept;
			s_Counters[iThread].cappedKept++;
			FaceDecide( p, WHY_NOEYE );
			return;
		}
		swept = true;
		if ( t.trace >= 0 )
			TraceF( fi, "  mirror pass: mirror face %d's candidates passed the cap: every one goes to the sweep through it\n",
					s_Reflective[r] );
		if ( SweepThroughMirror( iThread, t, (int)r, fi ) )
		{
			s_Counters[iThread].sweptFaces++;
			s_Counters[iThread].sweptVisible++;
			s_Counters[iThread].confirmHits++;
			FaceDecide( p, WHY_MIRROR );
			return;
		}
	}
	s_Counters[iThread].sweptFaces += swept;
	// Every ray from the eye clusters failed. A viewpoint in a solid leaf that sees
	// the face, directly or through a mirror it shows in, still keeps it visible
	// (SolidSeesFace). Tested last so that solid_view marks only the faces this rule
	// keeps; tier A applied the direct test to the faces the PVS hid. Its samples do not
	// depend on the eye spacing, and a face at a refinement level already failed it at level
	// 0 (through the same mirrors: a mirror bare at level 0 left no face in front of it
	// hidden), so a level skips it (#39 stage C).
	if ( s_SolidEyes.empty() || s_RefineLevel > 0 )
		return;
	bool seen = SolidSeesFace( f, t.samples, -1 );
	for ( size_t r = 0; r < s_Reflective.size() && !seen; r++ )
	{
		if ( s_Reflective[r] == fi || !InFrontOfMirror( f, s_MirrorFace[r] ) )
			continue;
		if ( s_MirrorBare[r] == MIRROR_SOLID )
			seen = SolidSeesFace( f, t.samples, (int)r );
	}
	if ( seen )
		FaceDecide( p, WHY_SOLIDVIEW );
	if ( t.trace >= 0 )
		TraceF( fi, "  face pass: no ray from an eye cluster; a viewpoint in solid %s it\n", seen ? "sees" : "does not see" );
}

// The face pass's time by step, summed over the batches (the log line after it).
static double s_StepTime[6];
static int s_Batches;

// One batch: B, C, D, E, F (A ran as its faces were added).
static void RunBatch()
{
	s_Batches++;
	double t = Plat_FloatTime(), t1;
	std::vector<CandJob *> jobs;
	for ( size_t i = 0; i < s_Batch.size(); i++ )
	{
		if ( !s_Batch[i].done )
			jobs.push_back( &s_Batch[i].job );
	}
	RunCount( jobs );
	t1 = Plat_FloatTime(), s_StepTime[1] += t1 - t, t = t1;
	jobs.clear();
	for ( size_t i = 0; i < s_Batch.size(); i++ )
	{
		PassFace &p = s_Batch[i];
		if ( p.done )
			continue;
		if ( p.job.thinned )
			s_FaceCapped[p.fi].store( 1, std::memory_order_relaxed );
		// an eye cluster that may see the face has no sample in front of it while its eye region
		// reaches more than REGION_MARGIN in front (the lattice and the piece vertices can all lie
		// behind a plane the pieces cross by up to a spacing): no ray test can settle the face
		if ( p.job.untestable )
		{
			if ( Traced( p.fi ) )
				TraceF( p.fi, "  face pass: no_eye: an eye cluster that may see it has its region in front but no sample there\n" );
			FaceDecide( p, WHY_NOEYE );
			continue;
		}
		jobs.push_back( &p.job );
	}
	RunGather( jobs );
	t1 = Plat_FloatTime(), s_StepTime[2] += t1 - t, t = t1;
	RunThreadsBatched( (int)s_Batch.size(), FaceTraceOne );
	t1 = Plat_FloatTime(), s_StepTime[3] += t1 - t, t = t1;
	RunSweep();
	t1 = Plat_FloatTime(), s_StepTime[4] += t1 - t, t = t1;
	GatherMirrorEyes();
	RunThreadsBatched( (int)s_Batch.size(), FaceMirrorOne );
	s_StepTime[5] += Plat_FloatTime() - t;
	s_Batch.clear();
	double bytes = 0;
	for ( size_t r = 0; r < s_MirrorEyes.size(); r++ )
		bytes += (double)s_MirrorEyes[r].capacity() * sizeof( dvec3 ) + (double)s_MirrorIds[r].capacity() * sizeof( long long );
	if ( bytes > MIRROR_EYE_BYTES )
	{
		for ( size_t r = 0; r < s_MirrorEyes.size(); r++ )
		{
			std::vector<dvec3>().swap( s_MirrorEyes[r] );
			std::vector<long long>().swap( s_MirrorIds[r] );
		}
		std::fill( s_MirrorGathered.begin(), s_MirrorGathered.end(), (unsigned char)0 );
	}
}

static void RunFacePass()
{
	// BSPWALK_FACE_BATCH=<faces>: a test hook, read once per run, that caps the faces of a batch (the
	// tiers must not depend on the batch: run_bspwalk_cache_tests.py runs it at 1)
	int hook = 0;
	if ( const char *env = getenv( "BSPWALK_FACE_BATCH" ) )
		hook = MAX( 0, atoi( env ) );
	const size_t maxFaces = hook > 0 ? (size_t)hook : (size_t)FACE_BATCH_MAX;
	const size_t group = MIN( (size_t)JOB_SETUP_GROUP, maxFaces );
	const size_t nPass = s_PassFaces.size();
	if ( s_BucketOfs.empty() )
		Error( "bspwalk: the face pass ran with no candidate buckets (BuildBuckets not run, or the refinement levels freed them)\n" );
	s_MirrorGathered.assign( s_Reflective.size(), 0 );
	s_SweepWarned = false;
	memset( s_StepTime, 0, sizeof( s_StepTime ) );
	s_Batches = 0;
	size_t next = 0;
	while ( next < nPass )
	{
		s_Batch.clear();
		double runBytes = 0, candBytes = 0, sampleBytes = 0;
		while ( next < nPass && s_Batch.size() < maxFaces && runBytes <= JOB_RUN_BYTES && candBytes <= JOB_CAND_BYTES &&
				sampleBytes <= JOB_SAMPLE_BYTES )
		{
			const size_t at = s_Batch.size(), k = MIN( group, MIN( maxFaces - at, nPass - next ) );
			s_Batch.resize( at + k );
			for ( size_t i = 0; i < k; i++ )
				s_Batch[at + i].fi = s_PassFaces[next + i];
			s_BatchAt = at;
			const double ta = Plat_FloatTime();
			RunThreadsBatched( (int)k, FaceSetupOne );
			s_StepTime[0] += Plat_FloatTime() - ta;
			for ( size_t i = 0; i < k; i++ )
			{
				const PassFace &p = s_Batch[at + i];
				runBytes += p.job.RunBytes();
				candBytes += p.job.CandBytes();
				sampleBytes += (double)p.samples.size() * sizeof( FaceSample );
			}
			next += k;
		}
		RunBatch();
	}
	std::vector<PassFace>().swap( s_Batch );
	std::vector<unsigned char>().swap( s_MirrorGathered );
	if ( nPass )
		Msg( "vis: face pass in %d batch(es): setup %.3f s, count %.3f, gather %.3f, binned tiles %.3f, candidate sweep %.3f, "
			 "mirrors and viewpoints in solid %.3f\n", s_Batches, s_StepTime[0], s_StepTime[1], s_StepTime[2], s_StepTime[3],
			 s_StepTime[4], s_StepTime[5] );
}

// Known answers for the GPU path, traced as the face pass encodes its segments: one
// through the middle of an occluder face must come back blocked, one above every
// occluder clear. Catches a kernel, mesh or encoding that blocks everything or
// nothing before it decides a face. True (nothing to test) with no usable face.
enum { CANARY_OK, CANARY_WRONG, CANARY_LAUNCH_FAILED };
static int VisGpuCanary( const VisCanary &c )
{
	if ( !( c.area > 0 ) )
		return CANARY_OK;
	std::vector<VisSegment> segs( 2 );
	segs[0].a = c.centre + c.normal * 4.0;
	segs[0].b = c.centre - c.normal * 4.0;
	segs[1].a = dvec3( c.mins.x - 64.0, c.mins.y - 64.0, c.maxs.z + 64.0 );
	segs[1].b = dvec3( c.maxs.x + 64.0, c.maxs.y + 64.0, c.maxs.z + 64.0 );
	std::vector<unsigned char> blocked;
	if ( !BwVis_TraceSegmentsGpu( segs, s_VisG, blocked ) || blocked.size() != 2 )
		return CANARY_LAUNCH_FAILED;
	return blocked[0] && !blocked[1] ? CANARY_OK : CANARY_WRONG;
}

// The candidate buckets of the store in s_pStore (#39). With vis data a bucket is an eye cluster: its
// runs, count and box from its facts (cluster -1, the samples of viewpoints in solid and the unplaced
// ones, is in no bucket; a cluster the face pass does not read was not computed and is an empty
// bucket, whose zero box nothing reads: a bucket with no eye is skipped before its box is looked at).
// Without vis data a bucket is an eye tile with samples, all of them (every cluster, -1 first): its
// box settles the back-face test for most eyes without a cube regrouping, and the order is the
// tiles' whatever the cache holds.
static void BuildBuckets( bool pvs )
{
	s_BucketOfs.assign( 1, 0 );
	s_BucketRuns.clear();
	s_BucketCount.clear();
	s_BucketBox.clear();
	if ( pvs )
	{
		static const float s_NoBox[6] = { 0, 0, 0, 0, 0, 0 };
		for ( int c = 0; c < s_pStore->NumClusters(); c++ )
		{
			const bool have = s_pStore->Computed( c );
			if ( have )
			{
				const EyeClusterFacts &fc = s_pStore->Facts( c );
				s_BucketRuns.insert( s_BucketRuns.end(), fc.runs.begin(), fc.runs.end() );
				s_BucketCount.push_back( fc.count );
				s_BucketBox.insert( s_BucketBox.end(), fc.box, fc.box + 6 );
			}
			else
			{
				s_BucketCount.push_back( 0 );
				s_BucketBox.insert( s_BucketBox.end(), s_NoBox, s_NoBox + 6 );
			}
			s_BucketOfs.push_back( (int)s_BucketRuns.size() );
		}
	}
	else
	{
		for ( int tile = 0; tile < s_pStore->NumTiles(); tile++ )
		{
			int n = s_pStore->TileCount( tile );
			if ( n <= 0 )
				continue;
			EyeRun run = { tile, 0, n };
			s_BucketRuns.push_back( run );
			s_BucketOfs.push_back( (int)s_BucketRuns.size() );
			s_BucketCount.push_back( n );
			const float *b = s_pStore->TileBox( tile );
			s_BucketBox.insert( s_BucketBox.end(), b, b + 6 );
		}
	}
}

// The eye clusters whose facts the face pass reads (#39), with vis data: the ones that can be a
// candidate of, or a bare cluster for, a face of the face pass or a reflective face (the mirror
// status and eye jobs read their candidates). Eye cluster c is one for face f when it is a piece cluster and f has no
// cluster (every eye cluster is then a candidate: JobSetup), or c is one of f's clusters, or
// the PVS row of c holds one of them (the tests of JobSetup, BareMaySee and FaceInPvsOf;
// RegionInFront and TraceRegion read the facts of clusters those two found). A superset costs
// tiles, never an answer: a cluster left out is one no face asks about. Needs s_PassFaces, s_Reflective,
// the face clusters and the PVS. At a refinement level (#39 stage C) only the mirrors a face of the
// pass lies in front of count: the status and eye jobs run for those alone (MirrorStatusPass,
// GatherMirrorEyes), so the level's finer tiles are made for no other mirror.
static void FacePassEyeClusters( std::vector<int> &needed )
{
	needed.clear();
	// the clusters of those faces as one more PVS row
	std::vector<unsigned char> faceCl( s_RowBytes, 0 );
	bool every = false;
	auto addFace = [&]( int f ) {
		if ( s_FaceClOfs[f] == s_FaceClOfs[f + 1] )
		{
			every = true;
			return;
		}
		for ( int k = s_FaceClOfs[f]; k < s_FaceClOfs[f + 1]; k++ )
			faceCl[s_FaceCl[k] >> 3] |= (unsigned char)( 1 << ( s_FaceCl[k] & 7 ) );
	};
	for ( size_t i = 0; i < s_PassFaces.size(); i++ )
		addFace( s_PassFaces[i] );
	for ( size_t r = 0; r < s_Reflective.size(); r++ )
	{
		bool front = s_RefineLevel == 0;
		for ( size_t i = 0; i < s_PassFaces.size() && !front; i++ )
			front = s_PassFaces[i] != s_Reflective[r] && InFrontOfMirror( s_Faces[s_PassFaces[i]], s_MirrorFace[r] );
		if ( front )
			addFace( s_Reflective[r] );
	}
	for ( int c = 0; c < s_pStore->NumClusters(); c++ )
	{
		if ( !s_pStore->PieceCluster( c ) )
			continue;
		bool need = every || ( ( faceCl[c >> 3] >> ( c & 7 ) ) & 1 );
		const unsigned char *row = &s_Pvs[(size_t)c * s_RowBytes];
		for ( int b = 0; !need && b < s_RowBytes; b++ )
			need = ( row[b] & faceCl[b] ) != 0;
		if ( need )
			needed.push_back( c );
	}
}

//-----------------------------------------------------------------------------
// Refinement levels (#39 stage C). The eye set is the shell of the lattice, not the full lattice:
// exact for sight lines wider than the spacing, weaker for a thin pencil of them, which a full
// lattice of spacing S hits about a L / S^3 times along a length L inside the eye volume and the
// shell about 2 a / S^2 times (a: its cross-section). Each halving of S gives the shell four times
// the hits. So after the face pass, the faces it left hidden (H) go through the face pass again at
// S / 2, then S / 4, ... (-eyerefine levels, default 2), each level's eye set made only for the
// clusters H's faces, and the mirrors they lie in front of, can be seen from. The levels run until
// -eyerefine is reached or H is empty (#39 stage D): a level that sees no face says little about the
// next (each halving gives the shell four times the hits on a thin pencil), so it does not stop them.
// A level only moves faces from hidden to visible, for the
// same reasons the face pass gives (ray, mirror, no_eye when a bare cluster or the untestable-region
// rule fires at that level); the fan and the viewpoints in solid are not redone. One level's store
// at a time: the eye set's tiles are dropped before level 1, each level's store freed before the next.
// Afterwards s_pStore is the eye set's store again, and the state the levels built from their own
// stores is cleared rather than restored, so a later read fails loudly instead of using another
// level's data: the buckets (RunFacePass stops on empty ones), the mirror status (InFrontOfBareMirror
// and GatherMirrorEyes stop on an unset one) and the mirror eyes. Nothing reads them after the levels.
//-----------------------------------------------------------------------------
static void RunRefineLevels( const EyeVolume &eyes, const VisOptions &opts, VisResult &out, const std::vector<int> &traced )
{
	std::vector<int> hidden;
	for ( size_t i = 0; i < s_PassFaces.size(); i++ )
	{
		if ( !VisibleGet( s_PassFaces[i] ) )
			hidden.push_back( s_PassFaces[i] );
	}
	if ( opts.eyeRefine <= 0 || hidden.empty() || !eyes.store )
		return;
	EyeStore *const base = s_pStore;
	// the eye set's tiles are not read again here: free them before a level makes its own
	base->DropResident();
	std::unique_ptr<EyeStore> level;
	const double budget = opts.eyeCacheMB * 1024.0 * 1024.0;
	for ( int k = 1; k <= opts.eyeRefine && k <= EYE_MAX_REFINE && !hidden.empty(); k++ )
	{
		const double t0 = Plat_FloatTime();
		FreeRefinedEyeStore( level );
		level = MakeRefinedEyeStore( eyes, k, budget );
		s_pStore = level.get();
		s_RefineLevel = k;
		s_PassFaces = hidden;
		for ( size_t i = 0; i < traced.size(); i++ )
		{
			if ( std::binary_search( hidden.begin(), hidden.end(), traced[i] ) )
				TraceF( traced[i], "  refinement level %d (eyes %gu apart; %d faces still hidden):\n", k, RefinedEyeSpacing( k ),
						(int)hidden.size() );
		}
		if ( out.pvs )
		{
			std::vector<int> needed;
			FacePassEyeClusters( needed );
			s_pStore->ComputeFor( needed );
		}
		else
			s_pStore->ComputeAll();
		TrimEyeScratch();
		BuildBuckets( out.pvs );
		// the mirror state is the level's: its status and eyes come from its store
		const size_t nR = s_Reflective.size();
		for ( size_t r = 0; r < nR; r++ )
		{
			std::vector<dvec3>().swap( s_MirrorEyes[r] );
			std::vector<long long>().swap( s_MirrorIds[r] );
		}
		s_MirrorBare.assign( nR, 0 );
		s_MirrorCapped.assign( nR, 0 );
		MirrorStatusPass();
		RunFacePass();
		std::vector<int> still;
		for ( size_t i = 0; i < hidden.size(); i++ )
		{
			if ( !VisibleGet( hidden[i] ) )
				still.push_back( hidden[i] );
		}
		const EyeStoreStats es = s_pStore->Stats();
		VisRefineLevel lv;
		lv.level = k;
		lv.spacing = RefinedEyeSpacing( k );
		lv.shell = EyeShellOn( lv.spacing );
		lv.facesIn = (int)hidden.size();
		lv.facesSeen = (int)( hidden.size() - still.size() );
		lv.clustersNeeded = s_pStore->NeededClusters();
		lv.tilesGenerated = es.tilesGenerated;
		lv.samples = es.samples;
		lv.unrecorded = es.unrecorded;
		lv.unplaced = es.unplaced;
		for ( size_t L = 0; L <= eyes.skyOrigins.size() && L < EYE_STATS_LAYERS; L++ )
			lv.occKept.push_back( es.occKept[L] );
		lv.seconds = Plat_FloatTime() - t0;
		out.refine.push_back( lv );
		Msg( "vis: refinement level %d (eyes %gu apart, shell %s): %d of %d hidden faces seen; %d eye clusters needed (%d with "
			 "no sample), %d tiles, %lld samples, peak %.1f MB resident; %.2f s\n", k, lv.spacing, lv.shell ? "on" : "off",
			 lv.facesSeen, lv.facesIn, lv.clustersNeeded,
			 (int)s_pStore->BareClusters().size(), lv.tilesGenerated, lv.samples, es.peakBytes / ( 1024.0 * 1024.0 ), lv.seconds );
		hidden.swap( still );
	}
	s_pStore = base;
	s_RefineLevel = 0;
	FreeRefinedEyeStore( level );
	TrimEyeScratch();
	// the levels' buckets and mirror state are cleared, not restored (see above)
	std::vector<int>().swap( s_BucketOfs );
	std::vector<EyeRun>().swap( s_BucketRuns );
	std::vector<long long>().swap( s_BucketCount );
	std::vector<float>().swap( s_BucketBox );
	for ( size_t r = 0; r < s_Reflective.size(); r++ )
	{
		std::vector<dvec3>().swap( s_MirrorEyes[r] );
		std::vector<long long>().swap( s_MirrorIds[r] );
	}
	s_MirrorStatus.assign( s_Reflective.size(), 0 );
	s_MirrorGathered.assign( s_Reflective.size(), 0 );
}

void RunVisibility( const CollisionWorld &world, const PlayerProfile &profile, const WalkGraph &graph,
					const EyeVolume &eyes, const VisOptions &opts, VisResult &out )
{
	double t0 = Plat_FloatTime();
	out = VisResult();
	VisStats &st = out.stats;
	s_pEyes = &eyes;
	s_pVisOpts = &opts;
	int nf = numfaces;
	s_nClusters = (int)eyes.clusters.size();
	out.pvs = eyes.numClusters > 0;

	BuildFaces( world, st );
	BuildFaceClusters( st );
	out.tier.assign( nf, TIER_VISIBLE );
	out.why.assign( nf, WHY_RAY );

	// -traceface: a log per traced face, printed at the end
	s_TraceSlot.clear();
	s_TraceLog.clear();
	s_TraceFan.clear();
	std::vector<int> traced;
	for ( size_t i = 0; i < opts.traceFaces.size(); i++ )
	{
		int f = opts.traceFaces[i];
		if ( f < 0 || f >= nf )
		{
			Warning( "bspwalk: -traceface %d is not a face of this BSP (0 to %d); ignored\n", f, nf - 1 );
			continue;
		}
		if ( s_TraceSlot.empty() )
			s_TraceSlot.assign( nf, -1 );
		if ( s_TraceSlot[f] >= 0 )
			continue;
		s_TraceSlot[f] = (int)s_TraceLog.size();
		s_TraceLog.push_back( std::string() );
		traced.push_back( f );
	}
	s_TraceFanCount.assign( s_TraceLog.size(), 0 );
	s_TraceFanGated.assign( s_TraceLog.size(), 0 );
	s_TraceSeen.assign( s_TraceLog.size(), TraceEyeInfo{ -1, 0, 0, 0, -1, 0, -1, 0 } );
	s_TraceSeenPass.assign( s_TraceLog.size(), "" );

	// Real viewpoint samples in a solid leaf (cluster -1, so no PVS row): a camera there draws every
	// cluster, so the faces it sees stay visible in both tiers (SolidSeesFace, #76). They
	// add no eye cluster: one misplaced cubemap protects only what it faces.
	s_SolidEyes = eyes.solidEyes;
	s_pStore = eyes.store.get();
	s_RefineLevel = 0;
	st.solidEyes = (int)s_SolidEyes.size();
	std::vector<unsigned char> solidKeptA( nf, 0 );	// faces the PVS hid that a solid viewpoint kept
	// SolidSeesFace traces against the occluders: build them now when tier A needs them
	// (tier B then reuses them), even under -notierb.
	bool occBuilt = false;
	if ( !s_SolidEyes.empty() )
	{
		double to = Plat_FloatTime();
		BuildOccluders( world, st );
		st.tOccluders = Plat_FloatTime() - to;
		FindSolidSkips();
		occBuilt = true;
	}
	// Tier A's verdict per face for the viewpoints in solid, tested once (the reflective
	// step repeats its loop): -1 untested, 0 not seen, 1 seen. The cull is the face pass's.
	std::vector<signed char> solidSeen( nf, -1 );
	std::vector<FaceSample> solidSamples;
	auto solidSees = [&]( int f ) -> bool
	{
		if ( solidSeen[f] < 0 )
		{
			const VisFace &vf = s_Faces[f];
			bool seen = false;
			if ( !s_SolidEyes.empty() && SolidViewInFront( vf, !vf.twoSided && !vf.disp ) )
			{
				double spUsed;
				SampleFace( vf, solidSamples, spUsed );
				seen = SolidSeesFace( vf, solidSamples, -1 );
			}
			solidSeen[f] = seen ? 1 : 0;
		}
		return solidSeen[f] != 0;
	};

	//
	// Tier A
	//
	double ta = Plat_FloatTime();
	std::vector<unsigned char> eyeCl( eyes.clusters );
	s_Pvs.clear();
	if ( out.pvs )
	{
		s_RowBytes = ( s_nClusters + 7 ) >> 3;
		s_Pvs.assign( (size_t)s_nClusters * s_RowBytes, 0 );
		for ( int c = 0; c < s_nClusters; c++ )
		{
			int ofs = dvis->bitofs[c][DVIS_PVS];
			if ( ofs > 0 && ofs < visdatasize )
				DecompressVis( &dvisdata[ofs], &s_Pvs[(size_t)c * s_RowBytes] );
			else
				memset( &s_Pvs[(size_t)c * s_RowBytes], 0xff, s_RowBytes );	// no row: everything
		}
		// Reflective faces that stay visible see like eyes: iterate. 'seen' holds the clusters in
		// the PVS of an eye cluster, bit k at word k >> 6 (a PVS row's bit order read as little-endian
		// 64-bit words). It lasts across the passes: each ORs in only the rows of the clusters the
		// pass before added to eyeCl, which are 'fresh'.
		const int rowWords = ( s_RowBytes + 7 ) >> 3;
		std::vector<unsigned long long> seen( rowWords, 0 );
		auto isSeen = [&]( int k ) { return ( ( seen[k >> 6] >> ( k & 63 ) ) & 1 ) != 0; };
		std::vector<int> fresh;
		for ( int c = 0; c < s_nClusters; c++ )
		{
			if ( eyeCl[c] )
				fresh.push_back( c );
		}
		std::vector<unsigned char> reflAdded( nf, 0 );
		bool borrow = false;	// a reflective displacement was added
		for ( ;; )
		{
			for ( size_t i = 0; i < fresh.size(); i++ )
			{
				const int c = fresh[i];
				const unsigned char *row = &s_Pvs[(size_t)c * s_RowBytes];
				for ( int w = 0; w < rowWords; w++ )
				{
					// the last word may run past the row: take only the bytes there
					unsigned long long bits = 0;
					memcpy( &bits, row + w * 8, MIN( 8, s_RowBytes - w * 8 ) );
					seen[w] |= bits;
				}
				seen[c >> 6] |= 1ULL << ( c & 63 );
			}
			fresh.clear();
			bool borrowNew = false;
			for ( int f = 0; f < nf; f++ )
			{
				if ( !s_Faces[f].reflective || s_Faces[f].bad || reflAdded[f] )
					continue;
				bool vis = false;
				for ( int k = s_FaceClOfs[f]; k < s_FaceClOfs[f + 1] && !vis; k++ )
					vis = isSeen( s_FaceCl[k] );
				// a visible reflective displacement shows the reflection the engine renders for
				// another reflective face (#78, at the mirror setup below): every such face's
				// view joins, as if it were visible
				if ( !vis && borrow && !s_Faces[f].disp )
					vis = true;
				// a brush entity's mirror (func_reflective_glass) is never tier A
				// a solid viewpoint that sees it keeps it visible below, so it shows its reflection too
				if ( !vis && s_Faces[f].model == 0 )
					vis = solidSees( f );
				if ( !vis && s_Faces[f].model == 0 )
					continue;
				reflAdded[f] = 1;
				if ( s_Faces[f].disp && !borrow )
					borrow = borrowNew = true;
				// the reflection shows the 3D skybox too, drawn from the mirrored
				// camera's skybox copy: the face's bounds mapped into each skybox
				std::vector<unsigned char> sk( s_nClusters, 0 );
				for ( size_t k = 0; k < eyes.skyOrigins.size(); k++ )
				{
					double sc = 1.0 / eyes.skyScales[k];
					BoxClusters( eyes.skyOrigins[k] + s_Faces[f].mins * sc, eyes.skyOrigins[k] + s_Faces[f].maxs * sc, sk );
				}
				for ( int c = 0; c < s_nClusters; c++ )
				{
					if ( sk[c] && !eyeCl[c] )
					{
						eyeCl[c] = 1;
						st.reflectClusters++;
						fresh.push_back( c );
					}
				}
				for ( int k = s_FaceClOfs[f]; k < s_FaceClOfs[f + 1]; k++ )
				{
					if ( !eyeCl[s_FaceCl[k]] )
					{
						eyeCl[s_FaceCl[k]] = 1;
						st.reflectClusters++;
						fresh.push_back( s_FaceCl[k] );
					}
				}
			}
			// a displacement added this pass: one more, for the faces it borrows planes from
			if ( fresh.empty() && !borrowNew )
				break;
		}
		for ( int f = 0; f < nf; f++ )
		{
			if ( s_Faces[f].model || s_Faces[f].bad )
				continue;
			// No cluster to look up (a face in no leaf with a cluster, or whose clusters
			// were all out of range): "none of its clusters is seen" would hold vacuously
			// and call it hidden. The PVS proves nothing about it; the ray tier decides.
			if ( s_FaceClOfs[f] == s_FaceClOfs[f + 1] )
				continue;
			bool vis = false;
			for ( int k = s_FaceClOfs[f]; k < s_FaceClOfs[f + 1] && !vis; k++ )
				vis = isSeen( s_FaceCl[k] );
			// a viewpoint in solid draws every cluster: no PVS row speaks for what it sees
			if ( !vis && solidSees( f ) )
			{
				solidKeptA[f] = 1;
				out.why[f] = WHY_SOLIDVIEW;		// tier B may still find it visible by a ray
				st.solidViewPvs++;
				continue;
			}
			if ( !vis )
			{
				out.tier[f] = TIER_A;
				out.why[f] = WHY_PVS;
			}
		}
	}
	else
		Warning( "bspwalk: the BSP has no vis data (run vvis): tier A is empty, every face goes to the ray tier\n" );
	for ( int f = 0; f < nf; f++ )
	{
		if ( s_Faces[f].model || s_Faces[f].bad )
		{
			out.tier[f] = TIER_VISIBLE;
			out.why[f] = s_Faces[f].model ? WHY_ENTITY : WHY_BADLUMP;
		}
	}
	st.tTierA = Plat_FloatTime() - ta;
	for ( size_t i = 0; i < traced.size(); i++ )
	{
		const int f = traced[i];
		const VisFace &vf = s_Faces[f];
		dvec3 c( 0, 0, 0 );
		for ( size_t k = 0; k < vf.w.size(); k++ )
			c += vf.w[k];
		if ( !vf.w.empty() )
			c = c * ( 1.0 / vf.w.size() );
		TraceF( f, "  face: model %d%s%s%s%s, normal (%.4f %.4f %.4f), d %.4f, centre (%.2f %.2f %.2f), box (%.2f %.2f %.2f) - "
				"(%.2f %.2f %.2f); clusters", vf.model, vf.disp ? ", displacement" : "", vf.twoSided ? ", two-sided" : "",
				vf.reflective ? ", reflective" : "", vf.bad ? ", bad lump" : "", vf.n.x, vf.n.y, vf.n.z, vf.d, c.x, c.y, c.z,
				vf.mins.x, vf.mins.y, vf.mins.z, vf.maxs.x, vf.maxs.y, vf.maxs.z );
		for ( int k = s_FaceClOfs[f]; k < s_FaceClOfs[f + 1]; k++ )
			TraceF( f, " %d", s_FaceCl[k] );
		TraceF( f, "\n" );
		if ( vf.model || vf.bad || !out.pvs )
			continue;
		// the eye clusters whose PVS holds the face (the reflective step's clusters included)
		int seers = 0;
		std::string first;
		for ( int cl = 0; cl < s_nClusters; cl++ )
		{
			if ( !eyeCl[cl] || !FaceInPvsOf( f, cl ) )
				continue;
			if ( seers++ < 8 )
				first += " " + std::to_string( cl );
		}
		TraceF( f, "  tier A: %s; %d eye cluster(s) have it in their PVS%s%s\n",
				out.tier[f] == TIER_A ? "hidden by the PVS" : solidKeptA[f] ? "the PVS hides it, a viewpoint in solid sees it"
																			  : "in the PVS of an eye cluster",
				seers, seers ? ":" : "", first.c_str() );
	}

	//
	// Tier B
	//
	// A viewpoint in a solid leaf does not stop it: the ray tier's eyes (per cluster,
	// with their PVS) do not describe what that camera sees, so the face pass traces
	// from it for every face no other ray reaches (SolidSeesFace, WHY_SOLIDVIEW).
	const bool tierB = opts.tierB;
	st.tierBRan = tierB;
	s_Visible.reset( new std::atomic<unsigned char>[nf] );
	for ( int f = 0; f < nf; f++ )
		VisibleSet( f, 0 );
	if ( tierB )
	{
		// occluders_s is the build alone, in tier A or here
		if ( !occBuilt )
		{
			double to = Plat_FloatTime();
			BuildOccluders( world, st );
			st.tOccluders = Plat_FloatTime() - to;
		}
		if ( !traced.empty() )
			TraceBuildBrushes();	// before the worker threads trace segments
		// GPU path for the face and mirror passes: the occluders as one mesh
		s_bVisGpu = s_bVisCheck = false;
		s_bVisSim = opts.gpuSim;
		s_bVisGpuOff = false;
		s_CheckReported = 0;
		s_VisG = (float)profile.gravity;
		bool wantCheck = opts.check;
		if ( wantCheck && opts.gpuSim )
		{
			Warning( "bspwalk: -vischeck is ignored with -visgpusim: the simulator traces every segment on the CPU, so "
					 "there is no second path to compare\n" );
			wantCheck = false;
		}
		bool canaryFailed = false;
		int canaryLaunchFailures = 0;	// added once the counters are summed (they are reset below)
		if ( s_bVisSim )
		{
			s_bVisGpu = true;
			Msg( "vis: -visgpusim: GPU batching, each segment traced on the CPU\n" );
		}
		else if ( !opts.cpuOnly )
		{
			if ( BwOptix_Init() )
			{
				std::vector<float> mv;
				std::vector<unsigned int> mt;
				VisCanary canary;
				OccluderMesh( 0.0, mv, mt, &canary );
				st.meshTris = (int)( mt.size() / 3 );
				if ( mt.empty() || BwOptix_BuildMesh( BW_MESH_VIS, mv, mt ) )
				{
					s_bVisGpu = !mt.empty();
					s_bVisCheck = s_bVisGpu && wantCheck;
				}
				int canaryResult = s_bVisGpu ? VisGpuCanary( canary ) : CANARY_OK;
				if ( canaryResult == CANARY_WRONG )
					Warning( "bspwalk: the GPU visibility path failed its known-answer check (a segment through an occluder "
							 "face must be blocked, one above every occluder clear); tier B fan, face and mirror rays on the CPU\n" );
				else if ( canaryResult == CANARY_LAUNCH_FAILED )
				{
					Warning( "bspwalk: the GPU visibility known-answer launch failed; tier B fan, face and mirror rays on the CPU\n" );
					canaryLaunchFailures = 1;
				}
				if ( canaryResult != CANARY_OK )
				{
					s_bVisGpu = s_bVisCheck = false;
					canaryFailed = true;
				}
				if ( !s_bVisGpu )
					BwOptix_Shutdown();
			}
			if ( !s_bVisGpu && !canaryFailed && !s_OccOfs.empty() && s_OccOfs.size() > 1 )
				Warning( "bspwalk: no GPU visibility path (CUDA device, OptiX or bspwalk_arcs.ptx missing); the tier B fan and face rays run on the CPU\n" );
		}
		if ( wantCheck && !s_bVisGpu )
			Warning( "bspwalk: -vischeck did not run: there is no GPU visibility path to compare against; tier B is traced "
					 "on the CPU only\n" );
		st.path = s_bVisSim ? "gpusim" : s_bVisGpu ? "gpu" : canaryFailed ? "cpu-fallback" : "cpu";
		st.rayUnit = s_bVisGpu ? "segments_launched" : "pairs_tested";
		for ( int t = 0; t <= MAX_TOOL_THREADS; t++ )
			s_Counters[t] = VisCounters();
		std::vector<dvec3> fmins, fmaxs;
		std::vector<int> faceIds;
		for ( int f = 0; f < nf; f++ )
		{
			if ( out.tier[f] != TIER_VISIBLE || s_Faces[f].model || s_Faces[f].bad )
				continue;
			faceIds.push_back( f );
			fmins.push_back( s_Faces[f].mins );
			fmaxs.push_back( s_Faces[f].maxs );
		}
		s_FaceBvh.Build( fmins, fmaxs );
		for ( size_t i = 0; i < s_FaceBvh.items.size(); i++ )
			s_FaceBvh.items[i] = faceIds[s_FaceBvh.items[i]];
		// faces already tier A never take a fan mark
		for ( int f = 0; f < nf; f++ )
		{
			if ( out.tier[f] != TIER_VISIBLE || s_Faces[f].model || s_Faces[f].bad )
				VisibleSet( f, 1 );
		}
		s_bBarePvs = out.pvs;
		{
			std::vector< std::atomic<unsigned char> > capped( nf );
			for ( int f = 0; f < nf; f++ )
				capped[f].store( 0, std::memory_order_relaxed );
			s_FaceCapped.swap( capped );
		}
		// fan: one eye per fanSpacing cell
		double tf = Plat_FloatTime();
		s_FanEyes = eyes.fanEyes.data();
		s_nFanEyes = (int)eyes.fanEyes.size();
		s_FanGpuT = NULL;
		s_FanFirst = 0;
		// The GPU fan rides on the face pass's OptiX setup, so it runs only when that passed its
		// canary (not under -visgpusim). It marks s_Visible itself, after its launches, from the
		// main thread; s_FanEyes already holds one eye per fan cell of each sky camera's set.
		bool fanGpu = s_bVisGpu && !s_bVisSim && s_nFanEyes > 0 && opts.fanRays > 0;
		bool fanFailed = fanGpu && !FanGpu( st, s_bVisCheck );
		if ( fanFailed )
		{
			Warning( "bspwalk: the GPU fan failed (the error is above); the fan runs on the CPU\n" );
			fanGpu = false;
		}
		if ( !fanGpu )
			RunThreadsBatched( s_nFanEyes, FanOne );
		st.fanPath = fanGpu ? "gpu" : fanFailed ? "cpu-fallback" : "cpu";
		st.fanLaunchFailures = fanFailed ? 1 : 0;
		st.tFan = Plat_FloatTime() - tf;
		if ( !traced.empty() )
		{
			s_FanReplay.clear();
			for ( size_t i = 0; fanGpu && i < traced.size(); i++ )
			{
				if ( VisibleGet( traced[i] ) == WHY_FAN + 1 )
					s_FanReplay.push_back( traced[i] );
			}
			if ( !s_FanReplay.empty() )
				RunThreadsBatched( s_nFanEyes, FanReplayOne );
			std::sort( s_TraceFan.begin(), s_TraceFan.end(), []( const TraceFanRay &a, const TraceFanRay &b ) {
				return a.face != b.face ? a.face < b.face : a.eye != b.eye ? a.eye < b.eye : a.ray < b.ray;
			} );
			for ( size_t i = 0; i < traced.size(); i++ )
			{
				const int f = traced[i];
				if ( out.tier[f] != TIER_VISIBLE || s_Faces[f].model || s_Faces[f].bad )
					continue;
				const long long n = s_TraceFanCount[s_TraceSlot[f]];
				if ( fanGpu && VisibleGet( f ) != WHY_FAN + 1 )
				{
					TraceF( f, "  fan (GPU): did not mark it\n" );
					continue;
				}
				if ( fanGpu )
					TraceF( f, "  fan (GPU): marked it; its rays replayed on the CPU for this log (nothing marked):\n" );
				TraceF( f, "  fan (CPU, %d eyes x %d rays): %lld ray(s) crossed it from eyes whose cluster's PVS holds it, "
						"%lld more from eyes whose PVS does not (not marked)%s\n", s_nFanEyes, opts.fanRays, n,
						s_TraceFanGated[s_TraceSlot[f]], n ? "; the first by eye:" : fanGpu ? " (the GPU's wider margins marked it)" : "" );
				int printed = 0;
				for ( size_t k = 0; k < s_TraceFan.size() && printed < TRACE_PRINT_RAYS; k++ )
				{
					const TraceFanRay &r = s_TraceFan[k];
					if ( r.face != f )
						continue;
					if ( !printed )
						TraceSeen( f, "fan", FanTraceInfo( r.eye ) );
					printed++;
					const FanEye &e = s_FanEyes[r.eye];
					const dvec3 o( e.x, e.y, e.z );
					TraceEye( f, "fan ray from", "fan eye", FanTraceInfo( r.eye ) );
					// the occluder the ray enters first
					int occ = -1;
					for ( size_t q = 0; q + 1 < s_OccOfs.size(); q++ )
					{
						double tq = RayEnterConvex( o, r.dir, FAN_MAX_T, &s_OccPlanes[s_OccOfs[q]], s_OccOfs[q + 1] - s_OccOfs[q] );
						if ( tq >= 0 && fabs( tq - r.tOcc ) < 1e-9 )
						{
							occ = (int)q;
							break;
						}
					}
					// or the one-way occluder it enters through a drawn side (NearestOccluder)
					int dirOcc = -1;
					for ( size_t q = 0; occ < 0 && q + 1 < s_DirOfs.size(); q++ )
					{
						double tq = RayEnterConvex( o, r.dir, FAN_MAX_T, &s_DirPlanes[s_DirOfs[q]], s_DirOfs[q + 1] - s_DirOfs[q] );
						if ( tq >= 0 && fabs( tq - r.tOcc ) < 1e-9 )
						{
							dirOcc = (int)q;
							break;
						}
					}
					const double tHit = r.tHit >= 0 ? r.tHit : r.tOcc;
					const dvec3 hit = o + r.dir * tHit;
					TraceF( f, "      ray %d, direction (%.6f %.6f %.6f); first occluder entry at t %.4f (%s %d, %s %d); "
							"crosses the face at t %.4f (%.4f %.4f %.4f), %+.4fu from that entry (marks reach %.2fu past it); "
							"the eye cluster's PVS %s the face\n", r.ray, r.dir.x, r.dir.y, r.dir.z, r.tOcc,
							dirOcc >= 0 ? "one-way occluder" : "occluder", dirOcc >= 0 ? dirOcc : occ,
							occ >= 0 && s_OccSrc[occ] == OCC_SRC_SEAM ? "seam filler" : occ >= 0 && s_OccSrc[occ] < 0 ? "func_brush convex" : "brush",
							dirOcc >= 0 ? s_DirSrc[dirOcc] : occ >= 0 ? ( s_OccSrc[occ] == OCC_SRC_SEAM ? occ : s_OccSrc[occ] < 0 ? -1 - s_OccSrc[occ] : s_OccSrc[occ] ) : -1, tHit, hit.x, hit.y,
							hit.z, tHit - r.tOcc, FAN_PAST,
							!out.pvs || e.cluster < 0 ? "(no PVS row) may hold" : FaceInPvsOf( f, e.cluster ) ? "holds" : "does NOT hold" );
					// TraceSegment's viewer is its second point, and the fan's is the eye
					TraceSegment( f, "face -> eye", hit, o );
				}
			}
		}

		// mirrors: reflective faces still visible, and the eyes in front of each
		// that can see it
		double tm = Plat_FloatTime();
		s_Reflective.clear();
		s_MirrorFace.clear();
		// A displacement with a reflective material is no fog volume: vbsp drops its brush from
		// the world and never gives its face a water leaf (surfaceFogVolumeID -1). The client
		// renders one water reflection per view, about the visible fog volume's surface plane
		// (viewrender.cpp CAboveWaterView), or a func_reflective_glass plane
		// (CReflectiveGlassView), into the one texture every reflective Water material samples
		// in screen space. So a displacement shows the reflection about another surface's
		// plane through its own footprint (#78): which one the engine picks is in the engine
		// binary, so it is a mirror once per plane the engine can reflect about, with its
		// bounds for a window (BwVis_DispMirrorPairs). Those planes (tier A faces' too: the pick follows
		// the eye, not what is drawn): each fog volume's surface, facing up
		// (dleafwaterdata surfaceZ; AdjustView mirrors the view origin's z about it, and
		// CUnderWaterView renders refraction only), the up-facing planes of the other
		// reflective faces (a brush entity's water has no fog volume), and every plane of a
		// brush entity's reflective faces (func_reflective_glass). A world water face's
		// side or underside is no reflection plane.
		std::vector<DPlane> reflPlanes;
		std::vector<DPlane> cand;
		for ( int w = 0; w < numleafwaterdata; w++ )
		{
			DPlane p;
			p.n = dvec3( 0, 0, 1 );
			p.d = dleafwaterdata[w].surfaceZ;
			cand.push_back( p );
		}
		for ( int f = 0; f < nf; f++ )
		{
			const VisFace &vf = s_Faces[f];
			if ( !vf.reflective || vf.disp || vf.bad || vf.w.size() < 3 || ( !vf.model && vf.n.z < 1.0 - 1e-6 ) )
				continue;
			DPlane p;
			p.n = vf.n;
			p.d = vf.d;
			cand.push_back( p );
		}
		for ( size_t c = 0; c < cand.size(); c++ )
		{
			bool dup = false;
			for ( size_t k = 0; k < reflPlanes.size() && !dup; k++ )
				dup = DDot( reflPlanes[k].n, cand[c].n ) > 1.0 - 1e-9 && fabs( reflPlanes[k].d - cand[c].d ) < 1e-3;
			if ( !dup )
				reflPlanes.push_back( cand[c] );
		}
		for ( int f = 0; f < nf; f++ )
		{
			if ( !s_Faces[f].reflective || out.tier[f] == TIER_A || s_Faces[f].w.size() < 3 )
				continue;
			if ( !s_Faces[f].disp )
			{
				s_Reflective.push_back( f );
				s_MirrorFace.push_back( s_Faces[f] );
				continue;
			}
			st.dispMirrors++;
			for ( size_t k = 0; k < reflPlanes.size(); k++ )
			{
				s_Reflective.push_back( f );
				s_MirrorFace.push_back( s_Faces[f] );
				s_MirrorFace.back().n = reflPlanes[k].n;
				s_MirrorFace.back().d = reflPlanes[k].d;
			}
		}
		st.reflectPlanes = (int)reflPlanes.size();
		s_MirrorEyes.assign( s_Reflective.size(), std::vector<dvec3>() );
		s_MirrorIds.assign( s_Reflective.size(), std::vector<long long>() );
		s_MirrorBare.assign( s_Reflective.size(), 0 );
		s_MirrorCapped.assign( s_Reflective.size(), 0 );
		// The mirrors' status (bare, solid, capped) is worked out from counts alone, before the
		// face pass, for every mirror a face of the face pass lies in front of (MirrorStatusPass);
		// their eyes only when a face still hidden after its own rays needs them (#39 stage B
		// part 2b; before it, #50 set a mirror up in full the first time a face asked, one
		// thread each: 380 s of fightspace3's face pass). The status marks nothing; the only
		// trace is capped_faces, which lists every capped mirror it worked out.
		s_MirrorStatus.assign( s_Reflective.size(), 0 );
		s_PassFaces.clear();
		for ( int f = 0; f < nf; f++ )
		{
			if ( !VisibleGet( f ) )
				s_PassFaces.push_back( f );
		}
		st.tMirror = Plat_FloatTime() - tm;

		// The eye tiles the face pass reads (#39), made now that tier A, the fan and the list of
		// faces going to the face pass are known. With vis data that is the tiles of the clusters
		// FacePassEyeClusters names, without it every tile (the buckets are the tiles); no face to
		// test, no tile. The generator's scratch is freed after (TrimEyeScratch). The tiles are
		// made while the occluders and the GPU mesh exist; the face pass would make them on demand
		// otherwise, one thread each.
		double tTiles = Plat_FloatTime();
		if ( !s_PassFaces.empty() )
		{
			if ( out.pvs )
			{
				std::vector<int> needed;
				FacePassEyeClusters( needed );
				s_pStore->ComputeFor( needed );
			}
			else
				s_pStore->ComputeAll();
			TrimEyeScratch();
		}
		st.tEyeTiles = Plat_FloatTime() - tTiles;
		st.noEyeClusters = (int)s_pStore->BareClusters().size();
		BuildBuckets( out.pvs );

		// every mirror's status the face pass may read, before it (counted in the mirror setup time)
		double tms = Plat_FloatTime();
		MirrorStatusPass();
		st.tMirror += Plat_FloatTime() - tms;
		double tp = Plat_FloatTime();
		for ( int t = 0; t <= MAX_TOOL_THREADS; t++ )
		{
			s_FaceThreads[t].facesWidened = 0;
			s_FaceThreads[t].spacingMax = 0;
		}
		RunFacePass();
		st.tFace = Plat_FloatTime() - tp;
		// the refinement levels (#39 stage C), on the faces the face pass left hidden; their rays
		// are counted in the totals below, the face sample widening only once (level 0's)
		{
			int widened = 0;
			double spMax = 0;
			for ( int t = 0; t <= MAX_TOOL_THREADS; t++ )
			{
				widened += s_FaceThreads[t].facesWidened;
				spMax = MAX( spMax, s_FaceThreads[t].spacingMax );
			}
			RunRefineLevels( eyes, opts, out, traced );
			for ( int t = 0; t <= MAX_TOOL_THREADS; t++ )
			{
				s_FaceThreads[t].facesWidened = t ? 0 : widened;
				s_FaceThreads[t].spacingMax = t ? 0 : spMax;
			}
		}

		for ( int t = 0; t <= MAX_TOOL_THREADS; t++ )
		{
			st.facesWidened += s_FaceThreads[t].facesWidened;
			st.faceSpacingMax = MAX( st.faceSpacingMax, s_FaceThreads[t].spacingMax );
			const VisCounters &c = s_Counters[t];
			st.fanRays += c.rays[0];
			st.faceRays += c.rays[1];
			st.mirrorRays += c.rays[2];
			st.checkSegments += c.check[0];
			st.gpuOnlyBlocked += c.check[1];
			st.cpuOnlyBlocked += c.check[2];
			st.gpuLaunchFailures += c.check[3] + ( t == 0 ? canaryLaunchFailures : 0 );
			st.checkUnexplained += c.check[4];
			st.checkMaxDist = MAX( st.checkMaxDist, c.checkMaxDist );
			st.checkTileDiffer += c.checkTiles;
			st.visibleConfirm += c.confirmHits;	// the fan's -vischeck counters were summed by FanGpu
			st.sweptFaces += c.sweptFaces;
			st.sweptVisible += c.sweptVisible;
			st.sweptMirror += c.sweptMirror;
			st.cappedKept += c.cappedKept;
			st.sweepEyes += c.sweepEyes;
		}
		for ( int f = 0; f < nf; f++ )
			st.cappedFaces += s_FaceCapped[f].load( std::memory_order_relaxed );
		if ( st.gpuLaunchFailures )
		{
			// the batches from the first failure on ran on the CPU; a failed canary launch
			// sends every ray to the CPU, so the counts stay pairs
			st.path = "cpu-fallback";
			if ( st.gpuLaunchFailures > canaryLaunchFailures )
				st.rayUnit = "mixed";
		}
		if ( s_bVisGpu && !s_bVisSim )
			BwOptix_Shutdown();
		// -visgpusim traces its "GPU" segments on the CPU from the same float form the check then traces
		// again, so the two sides are one computation and the counters are always 0: say that, not a comparison
		if ( s_bVisCheck && s_bVisSim )
		{
			Msg( "vis: -vischeck with -visgpusim compares the CPU with itself (%lld segments): nothing to report\n",
				 st.checkSegments );
		}
		else if ( s_bVisCheck )
		{
			Msg( "vis: -vischeck: %lld segments on both paths, %lld blocked only on the GPU, %lld only on the CPU, max "
				 "grazing distance %.6f (float bound %.6f), %lld unexplained; %d tile levels decided differently (the "
				 "CPU's answer kept)\n",
				 st.checkSegments, st.gpuOnlyBlocked, st.cpuOnlyBlocked, st.checkMaxDist, VIS_FLOAT_BOUND,
				 st.checkUnexplained, st.checkTileDiffer );
		}
		if ( s_bVisCheck && !strcmp( st.fanPath, "gpu" ) )
			Msg( "vis: -vischeck fan: %lld rays on both paths, %lld stopped short only on the GPU (max %.4fu); faces marked "
				 "%d only on the GPU, %d only on the CPU (the CPU's marks kept; any only on the CPU fails the run)\n",
				 st.fanCheckRays, st.fanGpuShort, st.fanMaxShort, st.fanGpuOnlyFaces, st.fanCpuOnlyFaces );
		s_bVisGpu = s_bVisCheck = s_bVisSim = false;
		for ( int f = 0; f < nf; f++ )
		{
			if ( out.tier[f] != TIER_VISIBLE || s_Faces[f].model || s_Faces[f].bad )
				continue;
			int v = VisibleGet( f );
			if ( !v )
			{
				out.tier[f] = TIER_B;
				out.why[f] = WHY_RAYS;
				continue;
			}
			out.why[f] = (unsigned char)( v - 1 );
			st.visibleFan += v - 1 == WHY_FAN;
			st.visibleFace += v - 1 == WHY_RAY;
			st.visibleMirror += v - 1 == WHY_MIRROR;
			st.visibleNoEye += v - 1 == WHY_NOEYE;
			// the faces the PVS hid were counted by tier A (st.solidViewPvs)
			st.solidViewRays += v - 1 == WHY_SOLIDVIEW && !solidKeptA[f];
		}
	}
	else
	{
		for ( int f = 0; f < nf; f++ )
		{
			// a face the PVS hid keeps the reason a solid viewpoint gave it in tier A
			if ( out.tier[f] == TIER_VISIBLE && !s_Faces[f].model && !s_Faces[f].bad && !solidKeptA[f] )
				out.why[f] = WHY_NOTIERB;
		}
		if ( opts.check )
			Warning( "bspwalk: -vischeck did not run: there are no tier B ray passes (-notierb)\n" );
	}

	for ( int f = 0; f < nf; f++ )
	{
		st.tierA += out.tier[f] == TIER_A;
		st.tierB += out.tier[f] == TIER_B;
		st.visible += out.tier[f] == TIER_VISIBLE;
	}
	out.seenFrom.clear();
	for ( size_t i = 0; i < traced.size(); i++ )
	{
		const int f = traced[i];
		const TraceEyeInfo &e = s_TraceSeen[s_TraceSlot[f]];
		if ( out.tier[f] == TIER_VISIBLE && e.id >= 0 )
		{
			VisSeenFrom w = { f, s_TraceSeenPass[s_TraceSlot[f]], e.id, { e.x, e.y, e.z }, e.cluster,
							  EyeSourceName( e.source ), e.level };
			out.seenFrom.push_back( w );
		}
		Msg( "trace face %d: tier %s, reason %s\n", f, out.tier[f] == TIER_A ? "A" : out.tier[f] == TIER_B ? "B" : "visible",
			 VisReasonName( out.why[f] ) );
		// a line per Msg: tier0's spew buffer is a few KB, and a log past it printed nothing at all
		const std::string &log = s_TraceLog[s_TraceSlot[f]];
		for ( size_t a = 0; a < log.size(); )
		{
			size_t b = log.find( '\n', a );
			b = b == std::string::npos ? log.size() : b + 1;
			Msg( "%s", log.substr( a, b - a ).c_str() );
			a = b;
		}
	}
	st.tTotal = Plat_FloatTime() - t0;
	Msg( "vis: %d faces (%d world, %d brush entity, %d displacement, %d reflective); tier A %d, tier B %d, visible %d%s\n",
		 st.faces, st.worldFaces, st.entityFaces, st.dispFaces, st.reflective, st.tierA, st.tierB, st.visible,
		 out.pvs ? "" : " (no vis data: no tier A)" );
	if ( st.dispMirrors )
		Msg( "vis: %d visible reflective displacement(s), each a mirror about the %d plane(s) the engine may reflect about "
			 "(#78)\n", st.dispMirrors, st.reflectPlanes );
	if ( st.solidEyes )
		Msg( "vis: %d samples of viewpoints in solid kept visible %d faces the PVS hid and %d faces no ray reached "
			 "(solid_view)%s\n", st.solidEyes, st.solidViewPvs, st.solidViewRays, tierB ? "" : " (-notierb: no ray tier)" );
	if ( tierB )
	{
		Msg( "vis: fan on the %s%s, face and mirror passes on the %s%s\n", st.fanPath,
			 !strcmp( st.fanPath, "gpu" ) ? " (OptiX)" : " (reference)", st.path,
			 !strcmp( st.path, "gpu" ) ? " (OptiX)" : " (reference)" );
		// the GPU counts segments launched, the CPU pairs tested up to the first clear one: not comparable
		const char *unit = !strcmp( st.rayUnit, "segments_launched" ) ? "segments launched"
						 : !strcmp( st.rayUnit, "pairs_tested" )      ? "pairs tested"
																	  : "segments launched and pairs tested";
		Msg( "vis: %d occluders, %d one-way (a nodraw side open to the map), %d solids left out (see-through or all nodraw), "
			 "%d seam fillers (%d contacts, %d reaching past an edge); "
			 "fan %d eyes x %d rays -> %d faces; face %s %lld -> %d, mirror %s %lld -> %d; %d reflective clusters added to tier A\n",
			 st.occluders, st.occludersOneWay, st.occludersSkipped, st.occludersSeam, st.seamContacts, st.seamExtended,
			 s_nFanEyes, opts.fanRays, st.visibleFan, unit, st.faceRays,
			 st.visibleFace, unit, st.mirrorRays, st.visibleMirror, st.reflectClusters );
		Msg( "vis: %d of the ray-visible faces first found by the -eyeconfirm pass (%d rays per eye)\n", st.visibleConfirm,
			 opts.confirmRays );
		Msg( "vis: %d faces kept visible with no ray test (%d of the eye clusters the face pass needed hold no eye sample)\n",
			 st.visibleNoEye, st.noEyeClusters );
		if ( st.facesWidened )
			Warning( "bspwalk: %d faces were sampled coarser than -facespacing %gu (up to %gu; the per-face sample cap)\n",
					 st.facesWidened, opts.faceSpacing, st.faceSpacingMax );
		if ( st.cappedFaces )
			Msg( "vis: %d face(s) (mirrors included) had more than %d candidate eyes: the tiles binned an even share of "
				 "every eye cluster, then %d face(s) the tiles left hidden paired every candidate (%lld eyes) with the "
				 "-eyeconfirm samples, and %d of those were seen only that way (%d sweeps through a capped mirror); %d kept "
				 "visible untested (-eyeconfirm 0)\n",
				 st.cappedFaces, FACE_MAX_CANDIDATES, st.sweptFaces, st.sweepEyes, st.sweptVisible, st.sweptMirror, st.cappedKept );
	}
	Msg( "vis timings (s): face clusters %.4f (leaf-less faces %.4f), eye tiles %.3f, tier A %.3f, occluders %.3f, fan %.3f "
		 "(GPU setup %.3f), mirror setup %.3f, face pass %.3f, total %.3f\n", st.tFaceClusters, st.tFaceBounds, st.tEyeTiles,
		 st.tTierA, st.tOccluders, st.tFan, st.tFanSetup, st.tMirror, st.tFace, st.tTotal );

	// Release everything this run left in the file statics, and the pointers to its arguments.
	std::vector<VisFace>().swap( s_Faces );
	std::vector<int>().swap( s_FaceClOfs );
	std::vector<int>().swap( s_FaceCl );
	std::vector<DPlane>().swap( s_OccPlanes );
	std::vector<int>().swap( s_OccOfs );
	std::vector<int>().swap( s_OccSrc );
	std::vector<int>().swap( s_SeamPairs );
	std::vector<DPlane>().swap( s_DirPlanes );
	std::vector<unsigned char>().swap( s_DirDrawn );
	std::vector<int>().swap( s_DirOfs );
	std::vector<int>().swap( s_DirSrc );
	s_DirBvh = Bvh();
	std::vector<DPlane>().swap( s_BrushPlanes );
	std::vector<unsigned char>().swap( s_BrushDrawn );
	std::vector<int>().swap( s_BrushOfs );
	std::vector<unsigned char>().swap( s_BrushRun );
	s_BrushBvh = Bvh();
	std::vector<int>().swap( s_TraceSlot );
	std::vector<std::string>().swap( s_TraceLog );
	std::vector<TraceFanRay>().swap( s_TraceFan );
	std::vector<long long>().swap( s_TraceFanCount );
	std::vector<long long>().swap( s_TraceFanGated );
	std::vector<TraceBrush>().swap( s_TraceBrushes );
	s_OccBvh = Bvh();
	s_FaceBvh = Bvh();
	s_Visible.reset();
	std::vector<unsigned char>().swap( s_Pvs );
	std::vector<int>().swap( s_BucketOfs );
	std::vector<EyeRun>().swap( s_BucketRuns );
	std::vector<long long>().swap( s_BucketCount );
	std::vector<float>().swap( s_BucketBox );
	std::vector<EyeSample>().swap( s_SolidEyes );
	std::vector< std::vector<long long> >().swap( s_MirrorIds );
	s_pStore = NULL;
	std::vector<int>().swap( s_SlabFaces );
	std::vector<unsigned char>().swap( s_SlabMark );
	s_FanEyes = NULL;
	s_nFanEyes = 0;
	std::vector<int>().swap( s_Reflective );
	std::vector<VisFace>().swap( s_MirrorFace );
	std::vector< std::vector<dvec3> >().swap( s_MirrorEyes );
	std::vector< std::vector<int> >().swap( s_SolidSkip );
	std::vector<unsigned char>().swap( s_MirrorBare );
	std::vector<unsigned char>().swap( s_MirrorCapped );
	std::vector<int>().swap( s_PassFaces );
	std::vector<unsigned char>().swap( s_MirrorStatus );
	std::vector< std::atomic<unsigned char> >().swap( s_FaceCapped );
	for ( int t = 0; t <= MAX_TOOL_THREADS; t++ )
		s_FaceThreads[t] = FaceThread();
	s_pEyes = NULL;
	s_pVisOpts = NULL;
	(void)profile;
	(void)graph;
}
