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
// At most this many candidate eyes per face go to the binned tiles (CapCandidateEyes
// thins them beyond): many times the eyes the finest bin level can use. Past it the
// confirmation still pairs every candidate (SweepCandidates).
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
// clear GPU segment is checked against them on the CPU (GpuAnyFree, SweepCandidates); the GPU
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
static std::vector<int> s_EyeClOfs, s_EyeByCl;		// cluster -> eye samples (CSR)
static const double NOVIS_BUCKET = 512.0;			// eye bucket cube size without vis data
static std::vector<float> s_EyeClBox;				// per s_EyeClOfs bucket: its eyes' mins xyz, maxs xyz
static bool s_bBarePvs;				// vis data present: a bare cluster sees only its PVS (else it may see every face)
// real (not skybox-mapped) samples of viewpoint sources whose point is in a solid
// leaf (cluster -1, so no PVS row): a camera there draws every cluster, so a face it
// sees stays visible (SolidSeesFace, WHY_SOLIDVIEW)
static std::vector<int> s_SolidEyes;
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
static std::vector<int> s_FanEyes;
static std::vector<int> s_Reflective;				// reflective faces that stay visible to tier A
static std::vector< std::vector<dvec3> > s_MirrorEyes;	// per s_Reflective: mirrored eyes in front of it
// per s_Reflective: MIRROR_BARE when no ray test can speak for it (a bare eye cluster may
// see it, or its eye region is untestable), else MIRROR_SOLID when a viewpoint sample in
// solid lies in front of it (FacePassOne then traces through it from those samples), else
// MIRROR_TESTED
enum { MIRROR_TESTED = 0, MIRROR_BARE = 1, MIRROR_SOLID = 2 };
static std::vector<unsigned char> s_MirrorBare;
static std::vector<int> s_PassFaces;
static std::vector<unsigned char> s_FaceCapped;		// per face: CandidateEyes thinned its candidates
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
	int sweptFaces;				// faces SweepCandidates ran for (their candidates, or a mirror's, passed the cap)
	int sweptVisible;			// ... of which it found a clear ray: the thinned set alone would have hidden them
	int cappedKept;				// faces past the cap kept visible untested (-eyeconfirm 0: nothing to sweep with)
	long long sweepEyes;		// eyes SweepCandidates paired
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
static inline int FanGateCluster( const EyeSample &e )
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
				return true;
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

// Marks every face the ray o + t dir from eye sample 'eye' crosses for t in [0, tMax], front
// side (or either side of a two-sided face), that the eye's PVS holds (FanEyeMaySee). ray and
// tOcc only feed -traceface.
static void MarkFacesAlong( const dvec3 &o, const dvec3 &dir, double tMax, int eye, int ray, double tOcc )
{
	if ( s_FaceBvh.nodes.empty() )
		return;
	const int gateCl = FanGateCluster( s_pEyes->samples[eye] );
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
			const VisFace &f = s_Faces[fi];
			if ( f.disp )
			{
				const std::vector<dvec3> &tris = *f.tris;
				for ( size_t k = 0; k + 2 < tris.size(); k += 3 )
				{
					if ( RayHitsTriangle( o, dir, tMax, tris[k], tris[k + 1], tris[k + 2] ) )
					{
						const bool see = FanEyeMaySee( gateCl, fi );
						if ( see && !VisibleGet( fi ) )
							VisibleSet( fi, WHY_FAN + 1 );
						if ( traced )
							TraceFanHit( fi, eye, ray, dir, tOcc, -1.0, !see );
						break;
					}
				}
				continue;
			}
			double denom = DDot( f.n, dir );
			if ( denom >= 0 && !f.twoSided )
				continue;
			if ( fabs( denom ) < 1e-12 )
				continue;
			double t = ( f.d - DDot( f.n, o ) ) / denom;
			if ( t < 0 || t > tMax )
				continue;
			if ( InsidePolygon( f, o + dir * t, 1e-3 ) )
			{
				const bool see = FanEyeMaySee( gateCl, fi );
				if ( see && !VisibleGet( fi ) )
					VisibleSet( fi, WHY_FAN + 1 );
				if ( traced )
					TraceFanHit( fi, eye, ray, dir, tOcc, t, !see );
			}
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
// Half-thicknesses tried, thickest first. 4 * OCC_SHRINK still covers the slot of the GPU
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
		if ( !SeamRegion( n, c, lines, NULL, 0.0, w ) )
			return false;	// the sides do not meet (a thicker filler only narrows the region)
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
		// the filler: the proven region inset by OCC_SHRINK + SEAM_SLACK, h either side
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
			st.occludersSkipped++;
			continue;
		}
		if ( !BuildBrushShape( b, shape ) )
			continue;
		// a nodraw side open to the map: blocks only what enters through a drawn side (#77)
		if ( OpenNodrawSide( b ) )
			AddDirOccluder( b, shape, dirMins, dirMaxs );
		else
		{
			AddOccluder( shape, mins, maxs, b );
			twoSided.push_back( shape );
		}
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
			st.occludersSkipped++;
			continue;
		}
		AddOccluder( sc.shape, mins, maxs, -1 - (int)i );
		twoSided.push_back( sc.shape );
	}
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

// -traceface: an eye sample as text.
static void TraceEye( int fi, const char *what, const EyeSample &e, int index )
{
	TraceF( fi, "    %s: eye sample %d at (%.4f %.4f %.4f), source %s, cluster %d%s\n", what, index, e.x, e.y, e.z,
			EyeSourceName( e.source ), e.cluster, e.sky >= 0 ? ", skybox copy" : "" );
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
	const EyeSample &e = s_pEyes->samples[s_FanEyes[s_FanFirst + item]];
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
		MarkFacesAlong( o, dir, t + FAN_PAST, s_FanEyes[s_FanFirst + item], i, t );
	}
	cnt.rays[0] += n;
}

// CapCandidateEyes' choice as positions into the list: runSize[r] eyes in run r,
// the runs one after another. 'pos' gets the kept positions, ascending. CandidateEyes
// takes them while it streams the eyes, without listing every one first.
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
	const EyeSample &e = s_pEyes->samples[s_FanEyes[item]];
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
		RunThreadsBatched( (int)s_FanEyes.size(), FanEyeInSlab );
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

	std::vector<float> faceN( (size_t)nf * 4 ), dirs( (size_t)n * 3 ), eyes( s_FanEyes.size() * 3 );
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
	for ( size_t i = 0; i < s_FanEyes.size(); i++ )
	{
		const EyeSample &e = s_pEyes->samples[s_FanEyes[i]];
		eyes[i * 3 + 0] = e.x;
		eyes[i * 3 + 1] = e.y;
		eyes[i * 3 + 2] = e.z;
	}
	// the PVS gate, as FanEyeMaySee: no vis data leaves every eye ungated
	BwFanPvs pvs;
	pvs.eyeCl.assign( s_FanEyes.size(), -1 );
	pvs.faceClOfs = s_FaceClOfs;
	pvs.faceCl = s_FaceCl;
	pvs.rowBytes = (unsigned int)s_RowBytes;
	if ( !s_Pvs.empty() )
	{
		pvs.rows = s_Pvs;
		for ( size_t i = 0; i < s_FanEyes.size(); i++ )
		{
			int c = FanGateCluster( s_pEyes->samples[s_FanEyes[i]] );
			pvs.eyeCl[i] = c < s_nClusters ? c : -1;
		}
	}
	bool ok = BwOptix_FanSetup( ov, ot, fv, ft, prim, faceN, plates, numPlates, dirs, eyes, start, pvs, dirMesh );
	if ( !ok )
		return false;
	double tSetup = Plat_FloatTime() - t0;

	unsigned int nEyes = (unsigned int)s_FanEyes.size();
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

// Does a viewpoint sample in a solid leaf (s_SolidEyes) lie in front of the face
// (any such sample when !cull: a two-sided face or a displacement)? The cheap
// prefilter of SolidSeesFace.
static bool SolidViewInFront( const VisFace &f, bool cull )
{
	const std::vector<EyeSample> &eyes = s_pEyes->samples;
	for ( size_t i = 0; i < s_SolidEyes.size(); i++ )
	{
		if ( !cull || EyeInFront( f, eyes[s_SolidEyes[i]] ) )
			return true;
	}
	return false;
}

// The occluders that hold each solid viewpoint sample (s_SolidSkip). The engine
// draws from such a camera with its brush's sides facing away, so that brush
// hides nothing from it. Tested against the unshrunk planes.
static void FindSolidSkips()
{
	const std::vector<EyeSample> &eyes = s_pEyes->samples;
	s_SolidSkip.assign( s_SolidEyes.size(), std::vector<int>() );
	for ( size_t i = 0; i < s_SolidEyes.size(); i++ )
	{
		const EyeSample &e = eyes[s_SolidEyes[i]];
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
	const std::vector<EyeSample> &eyes = s_pEyes->samples;
	BwSolidMirror m;
	const BwSolidMirror *pm = NULL;
	const VisFace *pR = NULL;
	if ( mirror >= 0 )
	{
		pR = &s_Faces[s_Reflective[mirror]];
		m.n = pR->n;
		m.d = pR->d;
		m.w = &pR->w;
		m.tol = MIRROR_TOL;
		pm = &m;
	}
	long long pairs = 0;
	for ( size_t i = 0; i < s_SolidEyes.size(); i++ )
	{
		const EyeSample &e = eyes[s_SolidEyes[i]];
		if ( pR && !EyeInFront( *pR, e ) )
			continue;
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
static int BucketSide( const VisFace &f, int c )
{
	const float *b = &s_EyeClBox[(size_t)c * 6];
	double lo = 0, hi = 0;
	for ( int a = 0; a < 3; a++ )
	{
		double n = f.n[a];
		lo += n * ( n > 0 ? b[a] : b[3 + a] );
		hi += n * ( n > 0 ? b[3 + a] : b[a] );
	}
	if ( lo > f.d + 2e-6 )
		return 1;
	if ( hi <= f.d )
		return 0;
	return 2;
}

// Does eye cluster c's region (the k-DOP bound of its pieces' parts, EyeBound)
// reach in front of the face? c < 0: without vis data the buckets are cubes, not
// clusters, so the test is made once for all eyes against the bound of all pieces
// (vbsp numbers clusters without vvis too, so the regions are not empty then).
static bool RegionInFront( const VisFace &f, int c )
{
	const EyeVolume &ev = *s_pEyes;
	bool noPvs = c < 0;
	if ( noPvs ? !ev.all.any : c >= (int)ev.region.size() )
		return false;
	const EyeBound &rb = noPvs ? ev.all : ev.region[c];
	return rb.Support( f.n, f.d + REGION_MARGIN ) > f.d + REGION_MARGIN;
}

// -traceface: how far eye cluster c's region reaches past the face's plane, by its
// box (what a box alone would have said), by the k-DOP bound the face pass uses,
// and the most any of the cluster's samples does (the region's true reach is
// between that and the bound when the samples include the piece vertices).
static void TraceRegion( int fi, const VisFace &f, int c, bool untestable )
{
	const EyeBound &rb = s_pEyes->region[c];
	double box = rb.BoxSupport( f.n ) - f.d, kdop = rb.Support( f.n ) - f.d;
	if ( box <= REGION_MARGIN && !untestable )
		return;
	const std::vector<EyeSample> &eyes = s_pEyes->samples;
	double sample = -1e300;
	for ( int i = s_EyeClOfs[c]; i < s_EyeClOfs[c + 1]; i++ )
	{
		const EyeSample &e = eyes[s_EyeByCl[i]];
		sample = MAX( sample, f.n.x * e.x + f.n.y * e.y + f.n.z * e.z - f.d );
	}
	TraceF( fi, "  face pass: eye cluster %d (%d samples, none in front) has its region (%.2f %.2f %.2f) - (%.2f %.2f %.2f): "
				"%s; reach past the plane: box %.3f, k-DOP %.3f, best sample %.3f\n",
			c, s_EyeClOfs[c + 1] - s_EyeClOfs[c], rb.lo.x, rb.lo.y, rb.lo.z, rb.hi.x, rb.hi.y, rb.hi.z,
			untestable ? "in front, the face stays visible (no_eye)" : "the box reaches in front but the k-DOP bound does not",
			box, kdop, sample );
}

// A face's candidate eyes as CountCandidates finds them: per eye bucket that
// holds one, its BucketSide answer and how many of its eyes pass. CandidateChunk
// walks them from a cursor, so a face with more candidates than the cap can have
// every one tested a chunk at a time without listing them all.
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
	bool thinned;		// n > FACE_MAX_CANDIDATES: CandidateEyes listed the cap's choice only
	size_t q;			// CandidateChunk's cursor: bucket ...
	int i;				// ... and position in s_EyeByCl
	CandSet() : fi( -1 ), n( 0 ), thinned( false ), q( 0 ), i( 0 ) {}
};

// The counting half of CandidateEyes: fills cs (cursor at the start) and returns
// true when an eye cluster that may see the face has no sample in front of it
// while its region reaches in front (see CandidateEyes).
static bool CountCandidates( int fi, bool front, CandSet &cs )
{
	const VisFace &f = s_Faces[fi];
	const std::vector<EyeSample> &eyes = s_pEyes->samples;
	int nCl = (int)s_EyeClOfs.size() - 1;
	bool cull = front || ( !f.twoSided && !f.disp );
	bool untestable = false;
	cs.fi = fi;
	cs.buckets.clear();
	cs.n = 0;
	cs.thinned = false;
	cs.q = 0;
	cs.i = 0;
	for ( int c = 0; c < nCl; c++ )
	{
		int a = s_EyeClOfs[c], b = s_EyeClOfs[c + 1];
		if ( a == b )
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
			bk.pass = b - a;
		else if ( bk.side == 2 )
		{
			for ( int i = a; i < b; i++ )
				bk.pass += EyeInFront( f, eyes[s_EyeByCl[i]] );
		}
		if ( !bk.pass )
		{
			// no sample in front: the bucket's pieces may still reach in front
			if ( cull && !untestable && !s_Pvs.empty() )
			{
				untestable = RegionInFront( f, c );
				if ( Traced( fi ) && c < (int)s_pEyes->region.size() )
					TraceRegion( fi, f, c, untestable );
			}
			continue;
		}
		cs.buckets.push_back( bk );
		cs.n += bk.pass;
	}
	if ( cull && cs.n == 0 && s_Pvs.empty() )
		untestable = RegionInFront( f, -1 );	// no eye in front, the pieces may reach it
	return untestable;
}

// The next candidates of cs from its cursor, at most maxEyes, in list order;
// false when none are left.
static bool CandidateChunk( CandSet &cs, size_t maxEyes, std::vector<int> &out )
{
	out.clear();
	const VisFace &f = s_Faces[cs.fi];
	const std::vector<EyeSample> &eyes = s_pEyes->samples;
	while ( cs.q < cs.buckets.size() && out.size() < maxEyes )
	{
		const CandBucket &bk = cs.buckets[cs.q];
		int a = s_EyeClOfs[bk.c], b = s_EyeClOfs[bk.c + 1];
		if ( cs.i < a )
			cs.i = a;
		for ( ; cs.i < b && out.size() < maxEyes; cs.i++ )
		{
			int e = s_EyeByCl[cs.i];
			if ( bk.side == 1 || EyeInFront( f, eyes[e] ) )
				out.push_back( e );
		}
		if ( cs.i >= b )
			cs.q++;
	}
	return !out.empty();
}

// The eyes that may see a face: those in clusters whose PVS holds one of its
// clusters (all eyes without vis data, or for a face with no cluster), in front
// of its plane unless it is two-sided or a displacement ('front': always; a
// mirror is seen only from in front). The lattice samples go to 'out': all of
// them, or past the cap CapCandidateEyes' choice of them (cs.thinned; the
// caller then tests the rest through CandidateChunk, SweepCandidates). With vis
// data the samples of viewpoints in solid hold cluster -1, so they are in no
// bucket and not candidates (SolidSeesFace decides for them); without vis data
// they are bucketed with every other eye and are candidates too. Returns true
// when no ray test can settle the face, so the caller keeps it visible: an eye
// cluster that may see the face has no sample in front of it while its eye
// region (the k-DOP bound of its pieces' parts) reaches more than REGION_MARGIN in
// front (the lattice and the piece vertices can all lie behind a plane the pieces
// cross by up to a spacing). Without vis data the eye
// buckets are 512u cubes with no region of their own: the check is made once,
// against the bound of all pieces, when no eye at all is in front.
static bool CandidateEyes( int fi, std::vector<int> &out, CandSet &cs, bool front = false )
{
	out.clear();
	const VisFace &f = s_Faces[fi];
	const std::vector<EyeSample> &eyes = s_pEyes->samples;
	// Count the eyes that pass per bucket, then take them, or the cap's choice of
	// them, straight into 'out'. Listing every passing eye first and thinning
	// afterwards cost over 2 minutes on fightspace3 (28M eyes, 900 mirrors set up
	// one by one).
	bool untestable = CountCandidates( fi, front, cs );
	static thread_local std::vector<size_t> runSize, keep;
	const size_t n = cs.n;
	const bool thin = n > (size_t)FACE_MAX_CANDIDATES;
	if ( thin )
	{
		// each bucket with a passing eye is one cluster run of CapCandidateEyes
		runSize.resize( cs.buckets.size() );
		for ( size_t q = 0; q < cs.buckets.size(); q++ )
			runSize[q] = cs.buckets[q].pass;
		CapPositions( runSize, FACE_MAX_CANDIDATES, keep );
		cs.thinned = true;
		s_FaceCapped[fi] = 1;
	}
	out.reserve( thin ? keep.size() : n );
	size_t j = 0, next = 0;	// position among the passing eyes; next index into 'keep'
	for ( size_t q = 0; q < cs.buckets.size() && ( !thin || next < keep.size() ); q++ )
	{
		int a = s_EyeClOfs[cs.buckets[q].c], b = s_EyeClOfs[cs.buckets[q].c + 1];
		if ( cs.buckets[q].side == 1 )
		{
			size_t end = j + ( b - a );
			if ( !thin )
				out.insert( out.end(), s_EyeByCl.begin() + a, s_EyeByCl.begin() + b );
			else
			{
				for ( ; next < keep.size() && keep[next] < end; next++ )
					out.push_back( s_EyeByCl[a + ( keep[next] - j )] );
			}
			j = end;
			continue;
		}
		for ( int i = a; i < b; i++ )
		{
			if ( !EyeInFront( f, eyes[s_EyeByCl[i]] ) )
				continue;
			if ( !thin )
				out.push_back( s_EyeByCl[i] );
			else if ( next < keep.size() && keep[next] == j )
			{
				out.push_back( s_EyeByCl[i] );
				next++;
			}
			j++;
		}
	}
	return untestable;
}

struct TileEye
{
	dvec3 p;
	double dist2;
	dvec3 viaEye;	// mirror: the real eye
};

// One eye per angular bin from c (the nearest not used yet); every unused one
// when they fit. The chosen ones are marked used. Returns the number left.
static int BinEyes( const dvec3 &c, const std::vector<dvec3> &pts, const std::vector<dvec3> *via, int bins,
					std::vector<unsigned char> &used, int &unused, std::vector<TileEye> &out )
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
	std::vector<FaceSample> samples;
	std::vector<int> cand;				// CandidateEyes: the lattice eyes
	CandSet cands;						// ... and the face's candidates in full (or a mirror's, in MirrorSetupOne)
	CandSet sweep;						// SweepCandidates through a mirror: the mirror's candidates
	std::vector<int> chunk;				// SweepCandidates: one chunk of candidates
	std::vector<double> sweepSamples;	// ... and the confirmation samples for the sweep kernel (p, n)
	std::vector<dvec3> pts, via;
	std::vector<TileEye> tileEyes;
	std::vector<unsigned char> used;
	std::vector< std::pair<long long, int> > tiles;
	std::vector<int> confirm;			// ConfirmPairs: the samples that can take part (ConfirmSamples)
	int trace = -1;						// -traceface: the face being traced, else -1
	const char *phase = "";				// -traceface: the step TestPair runs for (tiles, confirm, sweep)
};
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

// One sample-to-eye pair (eye: the point traced to; viaEye: the real eye
// behind the mirror pR, or eye itself). On the GPU path its segments are
// queued in t.arcs / t.segs and the result is 0. Returns -1 when the pair cannot
// see (back side, behind the mirror, misses the mirror), 1 for a clear CPU ray.
static int TestPair( int iThread, FaceThread &t, const FaceSample &s, const dvec3 &eye, const dvec3 &viaEye,
					 const VisFace *pR )
{
	if ( pR && DDot( pR->n, s.p ) <= pR->d )
		return -1;	// behind the mirror: not in its reflection
	dvec3 d = eye - s.p;
	if ( !t.twoSided && DDot( d, s.n ) <= 0 )
		return -1;	// the sample's back side
	if ( !pR )
	{
		if ( VisGpuQueue() )
		{
			PushSegment( t, s.p, eye );
			return 0;
		}
		s_Counters[iThread].rays[1]++;
		if ( SegmentBlocked( s.p, eye ) )
			return 0;
		if ( t.trace >= 0 )
		{
			TraceF( t.trace, "  face pass (%s): clear ray to the eye at (%.4f %.4f %.4f) (leaf cluster %d)\n", t.phase, eye.x,
					eye.y, eye.z, dleafs[PointLeaf( eye )].cluster );
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
	if ( !InsidePolygon( *pR, q, MIRROR_TOL ) )
		return -1;
	if ( VisGpuQueue() )
	{
		PushSegment( t, s.p, q );
		PushSegment( t, q, viaEye );
		return 0;
	}
	s_Counters[iThread].rays[2]++;
	if ( SegmentBlocked( s.p, q ) || SegmentBlocked( q, viaEye ) )
		return 0;
	if ( t.trace >= 0 )
	{
		TraceF( t.trace, "  mirror pass (%s): clear path to the eye at (%.4f %.4f %.4f) (leaf cluster %d) through the mirror at "
				"(%.4f %.4f %.4f)\n", t.phase, viaEye.x, viaEye.y, viaEye.z, dleafs[PointLeaf( viaEye )].cluster, q.x, q.y, q.z );
		TraceSegment( t.trace, "sample -> mirror", s.p, q );
		TraceSegment( t.trace, "mirror -> eye", q, viaEye );
	}
	return 1;
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
	const VisFace *pR = mirror >= 0 ? &s_Faces[s_Reflective[mirror]] : NULL;
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
// deterministic set per eye. 'base' is t.pts[0]'s place in the list of eyes
// SweepCandidates has fed to this pass so far, a chunk at a time (0 otherwise),
// so each eye keeps its own hash. For a direct sweep that is the place in the
// face's full candidate list. A mirror sweep drops the candidates behind the
// mirror, or whose image is behind the face, before they become t.pts, so its
// base advances by the eyes kept, not by the place in the full list; the ids
// are still unique per (face, mirror), which is all the hash needs. True at
// the first clear ray.
static bool ConfirmPairs( int iThread, FaceThread &t, int mirror, int fi, size_t base = 0 )
{
	int rays = MIN( s_pVisOpts->confirmRays, VIS_MAX_CONFIRM_RAYS );
	if ( rays <= 0 )
		return false;
	const VisFace *pR = mirror >= 0 ? &s_Faces[s_Reflective[mirror]] : NULL;
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
			size_t k = BwVis_ConfirmSample( key, base + i, r, rays, nS );
			if ( TestPair( iThread, t, t.samples[t.confirm[k]], t.pts[i], viaEye, pR ) == 1 )
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
// 'confirm' is false: t.pts is then a thinned set and SweepCandidates confirms
// over every candidate instead. mirror >= 0: points are mirrored eyes behind
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
	const VisFace *pR = mirror >= 0 ? &s_Faces[s_Reflective[mirror]] : NULL;
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
			BinEyes( c, t.pts, mirror >= 0 ? &t.via : NULL, bins0 << level, t.used, unused, t.tileEyes );
			// every pair of this tile level (GPU: in one launch)
			t.arcs.clear();
			t.segs.clear();
			for ( size_t k = a; k < b; k++ )
			{
				const FaceSample &s = t.samples[t.tiles[k].second];
				for ( size_t e = 0; e < t.tileEyes.size(); e++ )
				{
					if ( TestPair( iThread, t, s, t.tileEyes[e].p, t.tileEyes[e].viaEye, pR ) == 1 )
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
	s_Counters[iThread].confirmHits++;
	return true;
}

// A face whose candidates passed the cap (#35): the tiles binned a thinned set,
// so before the face can be tier B every candidate goes through the
// confirmation pairs, read SWEEP_CHUNK at a time so memory stays bounded. The
// face's own candidates are t.cands (FacePassOne counted them); through mirror
// r they are the mirror's (counted again here, as MirrorSetupOne did, into
// t.sweep), mirrored and culled as FacePassOne culls s_MirrorEyes. True at the
// first clear ray. Nothing calls this with -eyeconfirm 0 (FacePassOne then
// keeps the face visible).
static const size_t SWEEP_CHUNK = 1 << 18;

// The sweep kernel's eyes: every sample, uploaded by the first sweep that needs
// them (0 not tried, 1 on the device, 2 not available: no GPU path, -vischeck
// or -visgpusim, which compare or simulate the arc kernel's segments, or the
// device could not hold them). Reset by RunVisibility.
static std::atomic<int> s_SweepGpu;

static void SweepEyeFill( void *ctx, size_t first, size_t count, float *out )
{
	const std::vector<EyeSample> &eyes = *(const std::vector<EyeSample> *)ctx;
	for ( size_t k = 0; k < count; k++ )
	{
		const EyeSample &e = eyes[first + k];
		out[3 * k] = e.x;
		out[3 * k + 1] = e.y;
		out[3 * k + 2] = e.z;
	}
}

static bool SweepGpuReady()
{
	int s = s_SweepGpu.load();
	if ( s )
		return s == 1;
	std::lock_guard<std::mutex> lock( s_GpuMutex );
	s = s_SweepGpu.load();
	if ( s )
		return s == 1;
	bool ok = s_bVisGpu && !s_bVisSim && !s_bVisCheck && !s_bVisGpuOff.load() &&
			  BwOptix_SweepEyes( s_pEyes->samples.size(), SweepEyeFill, (void *)&s_pEyes->samples );
	if ( !ok && s_bVisGpu && !s_bVisSim && !s_bVisCheck )
		Warning( "bspwalk: the GPU cannot hold the %.0f eye samples for the candidate sweep; it traces through the arc "
				 "kernel instead (slower)\n", (double)s_pEyes->samples.size() );
	s_SweepGpu = ok ? 1 : 2;
	return ok;
}
static bool SweepCandidates( int iThread, FaceThread &t, int mirror, int fi )
{
	const VisFace &f = s_Faces[fi];
	const VisFace *pR = mirror >= 0 ? &s_Faces[s_Reflective[mirror]] : NULL;
	CandSet *pCs = &t.cands;
	if ( pR )
	{
		CountCandidates( s_Reflective[mirror], true, t.sweep );
		pCs = &t.sweep;
	}
	pCs->q = 0;
	pCs->i = 0;
	t.phase = "candidate sweep";
	ConfirmSamples( t, mirror );
	const bool cull = !f.twoSided && !f.disp;
	const std::vector<EyeSample> &eyes = s_pEyes->samples;
	size_t base = 0;
	// direct rays on the GPU path: the sweep kernel pairs and traces on the device, so
	// a chunk costs 4 B an eye to send instead of 48 B a segment (d1_trainstation_02 at
	// 8u: 6.1G segments, 65 s through the arc kernel)
	const int rays = MIN( s_pVisOpts->confirmRays, VIS_MAX_CONFIRM_RAYS );
	bool gpu = !pR && rays > 0 && !t.confirm.empty() && SweepGpuReady();
	if ( gpu )
	{
		t.sweepSamples.resize( t.confirm.size() * 6 );
		for ( size_t k = 0; k < t.confirm.size(); k++ )
		{
			const FaceSample &s = t.samples[t.confirm[k]];
			double *o = &t.sweepSamples[k * 6];
			o[0] = s.p.x, o[1] = s.p.y, o[2] = s.p.z, o[3] = s.n.x, o[4] = s.n.y, o[5] = s.n.z;
		}
		std::lock_guard<std::mutex> lock( s_GpuMutex );
		if ( !BwOptix_SweepSamples( iThread, t.sweepSamples.data(), (unsigned int)t.confirm.size() ) )
		{
			GpuLaunchFailed( iThread );
			gpu = false;
		}
	}
	while ( CandidateChunk( *pCs, SWEEP_CHUNK, t.chunk ) )
	{
		if ( gpu && !s_bVisGpuOff.load( std::memory_order_relaxed ) )
		{
			bool clear = false, ok;
			{
				std::lock_guard<std::mutex> lock( s_GpuMutex );
				ok = BwOptix_SweepTrace( iThread, (const unsigned int *)t.chunk.data(), (unsigned int)t.chunk.size(),
										 (unsigned int)rays, t.twoSided, (unsigned long long)fi << 32, base, clear );
			}
			if ( ok && !( clear && HasDirOccluders() ) )
			{
				s_Counters[iThread].rays[1] += (long long)t.chunk.size() * rays;
				s_Counters[iThread].sweepEyes += (long long)t.chunk.size();
				if ( clear )
					return true;
				base += t.chunk.size();
				continue;
			}
			// A clear ray with one-way occluders, which the sweep kernel does not hold: the chunk
			// goes through ConfirmPairs below, whose GPU groups GpuAnyFree checks against them
			if ( !ok )
				GpuLaunchFailed( iThread );	// this chunk and the rest go through ConfirmPairs (on the CPU from now on)
		}
		t.pts.clear();
		t.via.clear();
		for ( size_t k = 0; k < t.chunk.size(); k++ )
		{
			const EyeSample &e = eyes[t.chunk[k]];
			dvec3 p( e.x, e.y, e.z );
			if ( !pR )
			{
				t.pts.push_back( p );
				continue;
			}
			// as MirrorSetupOne and FacePassOne: in front of the mirror, and its image in front of the face
			if ( DDot( pR->n, p ) <= pR->d + 1e-6 )
				continue;
			dvec3 m = MirrorPoint( p, pR->n, pR->d );
			if ( cull && DDot( f.n, m ) <= f.d + 1e-6 )
				continue;
			t.pts.push_back( m );
			t.via.push_back( p );
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
	const EyeVolume &ev = *s_pEyes;
	for ( size_t i = 0; i < ev.bareClusters.size(); i++ )
	{
		if ( cull && ev.bare[i].Support( f.n, f.d + REGION_MARGIN ) <= f.d + REGION_MARGIN )
			continue;
		if ( !s_bBarePvs || FaceInPvsOf( fi, ev.bareClusters[i] ) )
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

// Does face fi lie partly in front of a mirror that a bare eye cluster may see
// (s_MirrorBare MIRROR_BARE)? The mirror loop of FacePassOne keeps such a face visible with
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
		if ( s_MirrorBare[r] == MIRROR_BARE && s_Reflective[r] != fi && InFrontOfMirror( s_Faces[fi], s_Faces[s_Reflective[r]] ) )
			return true;
	}
	return false;
}

static void FacePassOne( int iThread, int item )
{
	int fi = s_PassFaces[item];
	if ( VisibleGet( fi ) )
		return;
	FaceThread &t = s_FaceThreads[iThread];
	t.trace = Traced( fi ) ? fi : -1;
	t.phase = "";
	// Displacement samples are not back-face culled: a folded displacement's
	// triangles can face away from its base face, so a sample normal forced to
	// the base side would cull real front-side eyes.
	t.twoSided = s_Faces[fi].twoSided || s_Faces[fi].disp;
	// No ray test is possible from an eye cluster without samples (a piece
	// that only grazes a thin leaf, missing every vertex and lattice point)
	// that may see the face, or for a face without samples: it stays visible.
	// Clusters added for reflective faces are not eye piece clusters; the
	// mirror pass decides those. The first two tests need no samples, so they
	// run before the face is sampled.
	if ( BareMaySee( s_Faces[fi], fi, !s_Faces[fi].twoSided && !s_Faces[fi].disp ) || InFrontOfBareMirror( fi ) )
	{
		if ( t.trace >= 0 )
			TraceF( fi, "  face pass: no_eye: a bare eye cluster may see it, or it lies in front of a mirror one may see\n" );
		VisibleSet( fi, WHY_NOEYE + 1 );
		return;
	}
	double spUsed;
	SampleFace( s_Faces[fi], t.samples, spUsed );
	if ( spUsed > s_pVisOpts->faceSpacing )
	{
		t.facesWidened++;
		t.spacingMax = MAX( t.spacingMax, spUsed );
	}
	if ( t.samples.empty() )
	{
		if ( t.trace >= 0 )
			TraceF( fi, "  face pass: no_eye: no face sample\n" );
		VisibleSet( fi, WHY_NOEYE + 1 );
		return;
	}
	t.tiles.clear();
	for ( size_t i = 0; i < t.samples.size(); i++ )
		t.tiles.push_back( std::make_pair( TileKey( t.samples[i].p ), (int)i ) );
	std::sort( t.tiles.begin(), t.tiles.end() );

	if ( CandidateEyes( fi, t.cand, t.cands ) )
	{
		if ( t.trace >= 0 )
			TraceF( fi, "  face pass: no_eye: an eye cluster that may see it has its region in front but no sample there\n" );
		VisibleSet( fi, WHY_NOEYE + 1 );
		return;
	}
	if ( t.trace >= 0 )
		TraceF( fi, "  face pass: %d samples, %d candidate eyes%s; GPU path: %s\n", (int)t.samples.size(), (int)t.cand.size(),
				t.cands.thinned ? " (thinned past the cap)" : "", VisGpuQueue() ? "yes (pairs not named; -cpuvis names them)" : "no" );
	// Past the cap the tiles bin a thinned set; no face may be called hidden
	// from that alone (#35): every candidate then goes through SweepCandidates,
	// or with -eyeconfirm 0 (no confirmation pairs to sweep with) the face stays
	// visible untested.
	const bool sweep = s_pVisOpts->confirmRays > 0;
	bool swept = false;
	t.pts.clear();
	for ( size_t i = 0; i < t.cand.size(); i++ )
	{
		const EyeSample &e = s_pEyes->samples[t.cand[i]];
		t.pts.push_back( dvec3( e.x, e.y, e.z ) );
	}
	if ( !t.pts.empty() && TraceTiles( iThread, t, -1, fi, !t.cands.thinned ) )
	{
		if ( t.trace >= 0 )
			TraceF( fi, "  face pass: ray (%s)\n", t.phase );
		VisibleSet( fi, WHY_RAY + 1 );
		return;
	}
	if ( t.cands.thinned )
	{
		if ( !sweep )
		{
			s_Counters[iThread].cappedKept++;
			VisibleSet( fi, WHY_NOEYE + 1 );
			return;
		}
		swept = true;
		if ( SweepCandidates( iThread, t, -1, fi ) )
		{
			s_Counters[iThread].sweptFaces++;
			s_Counters[iThread].sweptVisible++;
			s_Counters[iThread].confirmHits++;
			VisibleSet( fi, WHY_RAY + 1 );
			return;
		}
	}

	// through each reflective face that stays visible
	const VisFace &f = s_Faces[fi];
	for ( size_t r = 0; r < s_Reflective.size(); r++ )
	{
		if ( s_Reflective[r] == fi || ( s_MirrorEyes[r].empty() && !s_MirrorBare[r] ) )
			continue;
		const VisFace &R = s_Faces[s_Reflective[r]];
		// some part of the face must lie in front of the mirror
		if ( !InFrontOfMirror( f, R ) )
			continue;
		if ( s_MirrorBare[r] == MIRROR_BARE )
		{
			// a bare eye cluster may see the mirror: no ray test is possible
			if ( t.trace >= 0 )
				TraceF( fi, "  mirror pass: no_eye: it lies in front of mirror face %d, which no ray test can speak for\n",
						s_Reflective[r] );
			s_Counters[iThread].sweptFaces += swept;
			VisibleSet( fi, WHY_NOEYE + 1 );
			return;
		}
		t.pts.clear();
		t.via.clear();
		bool cull = !f.twoSided && !f.disp;
		const std::vector<dvec3> &me = s_MirrorEyes[r];
		for ( size_t k = 0; k < me.size(); k++ )
		{
			dvec3 m = MirrorPoint( me[k], R.n, R.d );
			if ( cull && DDot( f.n, m ) <= f.d + 1e-6 )
				continue;
			t.pts.push_back( m );
			t.via.push_back( me[k] );
		}
		if ( !t.pts.empty() && TraceTiles( iThread, t, (int)r, fi, !s_MirrorCapped[r] ) )
		{
			s_Counters[iThread].sweptFaces += swept;
			VisibleSet( fi, WHY_MIRROR + 1 );
			return;
		}
		if ( !s_MirrorCapped[r] )
			continue;
		// the mirror's own candidates passed the cap: s_MirrorEyes[r] is its thinned set
		if ( !sweep )
		{
			s_Counters[iThread].sweptFaces += swept;
			s_Counters[iThread].cappedKept++;
			VisibleSet( fi, WHY_NOEYE + 1 );
			return;
		}
		swept = true;
		if ( SweepCandidates( iThread, t, (int)r, fi ) )
		{
			s_Counters[iThread].sweptFaces++;
			s_Counters[iThread].sweptVisible++;
			s_Counters[iThread].confirmHits++;
			VisibleSet( fi, WHY_MIRROR + 1 );
			return;
		}
	}
	s_Counters[iThread].sweptFaces += swept;
	// Every ray from the eye clusters failed. A viewpoint in a solid leaf that sees
	// the face, directly or through a mirror it shows in, still keeps it visible
	// (SolidSeesFace). Tested last so that solid_view marks only the faces this rule
	// keeps; tier A applied the direct test to the faces the PVS hid.
	if ( s_SolidEyes.empty() )
		return;
	bool seen = SolidSeesFace( f, t.samples, -1 );
	for ( size_t r = 0; r < s_Reflective.size() && !seen; r++ )
	{
		if ( s_MirrorBare[r] == MIRROR_SOLID && s_Reflective[r] != fi && InFrontOfMirror( f, s_Faces[s_Reflective[r]] ) )
			seen = SolidSeesFace( f, t.samples, (int)r );
	}
	if ( seen )
		VisibleSet( fi, WHY_SOLIDVIEW + 1 );
	if ( t.trace >= 0 )
		TraceF( fi, "  face pass: no ray from an eye cluster; a viewpoint in solid %s it\n", seen ? "sees" : "does not see" );
}

// Mirror r (s_Reflective[r]): the eyes in front of it that may see it (past the
// cap the thinned set; s_MirrorCapped, and FacePassOne sweeps the rest). Each r is
// one face, so the CandidateEyes calls of different threads write different
// s_FaceCapped entries.
static void MirrorSetupOne( int iThread, int r )
{
	int f = s_Reflective[r];
	const VisFace &R = s_Faces[f];
	const std::vector<EyeSample> &samples = s_pEyes->samples;
	// a reflection is seen only from in front of the mirror
	unsigned char bare = BareMaySee( R, f, true ) ? MIRROR_BARE : MIRROR_TESTED;
	FaceThread &t = s_FaceThreads[iThread];
	// only the eyes in front of the mirror (culled before the thinning)
	if ( CandidateEyes( f, t.cand, t.cands, true ) )
		bare = MIRROR_BARE;		// an untestable eye region
	s_MirrorCapped[r] = t.cands.thinned;
	// a viewpoint in solid in front of it: FacePassOne traces from it through the mirror
	// (SolidSeesFace) for the faces no ray from the eye clusters reaches
	if ( bare == MIRROR_TESTED && SolidViewInFront( R, true ) )
		bare = MIRROR_SOLID;
	s_MirrorBare[r] = bare;
	std::vector<dvec3> &me = s_MirrorEyes[r];
	for ( size_t k = 0; k < t.cand.size(); k++ )
	{
		const EyeSample &e = samples[t.cand[k]];
		dvec3 p( e.x, e.y, e.z );
		if ( DDot( R.n, p ) > R.d + 1e-6 )
			me.push_back( p );
	}
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

	// Real viewpoint samples in a solid leaf (cluster -1, so no PVS row): a camera there draws every
	// cluster, so the faces it sees stay visible in both tiers (SolidSeesFace, #76). They
	// add no eye cluster: one misplaced cubemap protects only what it faces.
	s_SolidEyes.clear();
	for ( size_t i = 0; i < eyes.samples.size(); i++ )
	{
		const EyeSample &e = eyes.samples[i];
		if ( e.cluster < 0 && e.sky < 0 && IsViewpointSource( e.source ) )
			s_SolidEyes.push_back( (int)i );
	}
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
			for ( int f = 0; f < nf; f++ )
			{
				if ( !s_Faces[f].reflective || s_Faces[f].bad || reflAdded[f] )
					continue;
				bool vis = false;
				for ( int k = s_FaceClOfs[f]; k < s_FaceClOfs[f + 1] && !vis; k++ )
					vis = isSeen( s_FaceCl[k] );
				// a brush entity's mirror (func_reflective_glass) is never tier A
				// a solid viewpoint that sees it keeps it visible below, so it shows its reflection too
				if ( !vis && s_Faces[f].model == 0 )
					vis = solidSees( f );
				if ( !vis && s_Faces[f].model == 0 )
					continue;
				reflAdded[f] = 1;
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
			if ( fresh.empty() )
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
		s_SweepGpu = 0;
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
		// eyes by cluster
		int nCl = MAX( s_nClusters, 1 );
		s_EyeClOfs.assign( nCl + 1, 0 );
		for ( size_t i = 0; i < eyes.samples.size(); i++ )
		{
			int c = eyes.samples[i].cluster;
			if ( c >= 0 && c < nCl )
				s_EyeClOfs[c + 1]++;
		}
		for ( int c = 0; c < nCl; c++ )
			s_EyeClOfs[c + 1] += s_EyeClOfs[c];
		s_EyeByCl.assign( s_EyeClOfs[nCl], 0 );
		{
			std::vector<int> fill( s_EyeClOfs.begin(), s_EyeClOfs.end() - 1 );
			for ( size_t i = 0; i < eyes.samples.size(); i++ )
			{
				int c = eyes.samples[i].cluster;
				if ( c >= 0 && c < nCl )
					s_EyeByCl[fill[c]++] = (int)i;
			}
		}
		s_bBarePvs = out.pvs;
		st.noEyeClusters = (int)eyes.bareClusters.size();
		s_FaceCapped.assign( nf, 0 );
		if ( !out.pvs )
		{
			// No clusters to bucket by: bucket the eyes by NOVIS_BUCKET cubes, so
			// the bucket boxes settle the back-face test for most eyes (one bucket
			// of all 39M eyes cost fightspace3's mirror setup 14 s).
			std::vector< std::pair<long long, int> > keyed( eyes.samples.size() );
			for ( size_t i = 0; i < eyes.samples.size(); i++ )
			{
				const EyeSample &e = eyes.samples[i];
				long long kx = (long long)floor( e.x / NOVIS_BUCKET ) & 0x1fffff, ky = (long long)floor( e.y / NOVIS_BUCKET ) & 0x1fffff,
						  kz = (long long)floor( e.z / NOVIS_BUCKET ) & 0x1fffff;
				keyed[i] = std::make_pair( ( kx << 42 ) | ( ky << 21 ) | kz, (int)i );
			}
			std::sort( keyed.begin(), keyed.end() );
			s_EyeClOfs.assign( 1, 0 );
			s_EyeByCl.resize( keyed.size() );
			for ( size_t i = 0; i < keyed.size(); i++ )
			{
				if ( i > 0 && keyed[i].first != keyed[i - 1].first )
					s_EyeClOfs.push_back( (int)i );
				s_EyeByCl[i] = keyed[i].second;
			}
			s_EyeClOfs.push_back( (int)keyed.size() );
		}
		s_EyeClBox.assign( ( s_EyeClOfs.size() - 1 ) * 6, 0.0f );
		for ( size_t c = 0; c + 1 < s_EyeClOfs.size(); c++ )
		{
			float *b = &s_EyeClBox[c * 6];
			b[0] = b[1] = b[2] = FLT_MAX;
			b[3] = b[4] = b[5] = -FLT_MAX;
			for ( int i = s_EyeClOfs[c]; i < s_EyeClOfs[c + 1]; i++ )
			{
				const EyeSample &e = eyes.samples[s_EyeByCl[i]];
				b[0] = MIN( b[0], e.x );
				b[1] = MIN( b[1], e.y );
				b[2] = MIN( b[2], e.z );
				b[3] = MAX( b[3], e.x );
				b[4] = MAX( b[4], e.y );
				b[5] = MAX( b[5], e.z );
			}
		}

		// fan: one eye per fanSpacing cell
		double tf = Plat_FloatTime();
		s_FanEyes.clear();
		{
			// A skybox copy is 1/scale the size, so its fan cells are too, at its own camera's scale;
			// each sky camera's copies have a cell set of their own (set 0: the real eyes).
			std::vector< std::unordered_set<long long> > cells( eyes.skyScales.size() + 1 );
			for ( size_t i = 0; i < eyes.samples.size(); i++ )
			{
				const EyeSample &e = eyes.samples[i];
				if ( e.cluster < 0 && out.pvs )
					continue;
				double fs = e.sky >= 0 ? opts.fanSpacing / eyes.skyScales[e.sky] : opts.fanSpacing;
				long long x = (long long)floor( e.x / fs ) + ( 1 << 20 ), y = (long long)floor( e.y / fs ) + ( 1 << 20 ),
						  z = (long long)floor( e.z / fs ) + ( 1 << 20 );
				if ( cells[e.sky + 1].insert( ( x << 42 ) | ( y << 21 ) | z ).second )
					s_FanEyes.push_back( (int)i );
			}
		}
		s_FanGpuT = NULL;
		s_FanFirst = 0;
		// The GPU fan rides on the face pass's OptiX setup, so it runs only when that passed its
		// canary (not under -visgpusim). It marks s_Visible itself, after its launches, from the
		// main thread; s_FanEyes already holds one eye per fan cell of each sky camera's set.
		bool fanGpu = s_bVisGpu && !s_bVisSim && !s_FanEyes.empty() && opts.fanRays > 0;
		bool fanFailed = fanGpu && !FanGpu( st, s_bVisCheck );
		if ( fanFailed )
		{
			Warning( "bspwalk: the GPU fan failed (the error is above); the fan runs on the CPU\n" );
			fanGpu = false;
		}
		if ( !fanGpu )
			RunThreadsBatched( (int)s_FanEyes.size(), FanOne );
		st.fanPath = fanGpu ? "gpu" : fanFailed ? "cpu-fallback" : "cpu";
		st.fanLaunchFailures = fanFailed ? 1 : 0;
		st.tFan = Plat_FloatTime() - tf;
		if ( !traced.empty() )
		{
			std::sort( s_TraceFan.begin(), s_TraceFan.end(), []( const TraceFanRay &a, const TraceFanRay &b ) {
				return a.face != b.face ? a.face < b.face : a.eye != b.eye ? a.eye < b.eye : a.ray < b.ray;
			} );
			for ( size_t i = 0; i < traced.size(); i++ )
			{
				const int f = traced[i];
				if ( out.tier[f] != TIER_VISIBLE || s_Faces[f].model || s_Faces[f].bad )
					continue;
				const long long n = s_TraceFanCount[s_TraceSlot[f]];
				if ( fanGpu )
				{
					TraceF( f, "  fan (GPU): %s (the GPU fan names no ray; -cpuvis does)\n",
							VisibleGet( f ) == WHY_FAN + 1 ? "marked it" : "did not mark it" );
					continue;
				}
				TraceF( f, "  fan (CPU, %d eyes x %d rays): %lld ray(s) crossed it from eyes whose cluster's PVS holds it, "
						"%lld more from eyes whose PVS does not (not marked)%s\n", (int)s_FanEyes.size(), opts.fanRays, n,
						s_TraceFanGated[s_TraceSlot[f]], n ? "; the first by eye:" : "" );
				int printed = 0;
				for ( size_t k = 0; k < s_TraceFan.size() && printed < TRACE_PRINT_RAYS; k++ )
				{
					const TraceFanRay &r = s_TraceFan[k];
					if ( r.face != f )
						continue;
					printed++;
					const EyeSample &e = eyes.samples[r.eye];
					const dvec3 o( e.x, e.y, e.z );
					TraceEye( f, "fan ray from", e, r.eye );
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
		for ( int f = 0; f < nf; f++ )
		{
			if ( !s_Faces[f].reflective || out.tier[f] == TIER_A || s_Faces[f].disp || s_Faces[f].w.size() < 3 )
				continue;
			s_Reflective.push_back( f );
		}
		s_MirrorEyes.assign( s_Reflective.size(), std::vector<dvec3>() );
		s_MirrorBare.assign( s_Reflective.size(), 0 );
		s_MirrorCapped.assign( s_Reflective.size(), 0 );
		RunThreadsBatched( (int)s_Reflective.size(), MirrorSetupOne );
		st.tMirror = Plat_FloatTime() - tm;

		double tp = Plat_FloatTime();
		s_PassFaces.clear();
		for ( int f = 0; f < nf; f++ )
		{
			if ( !VisibleGet( f ) )
				s_PassFaces.push_back( f );
		}
		for ( int t = 0; t <= MAX_TOOL_THREADS; t++ )
		{
			s_FaceThreads[t].facesWidened = 0;
			s_FaceThreads[t].spacingMax = 0;
		}
		RunThreadsBatched( (int)s_PassFaces.size(), FacePassOne );
		st.tFace = Plat_FloatTime() - tp;

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
			st.cappedKept += c.cappedKept;
			st.sweepEyes += c.sweepEyes;
		}
		for ( int f = 0; f < nf; f++ )
			st.cappedFaces += s_FaceCapped[f];
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
	for ( size_t i = 0; i < traced.size(); i++ )
	{
		const int f = traced[i];
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
			 (int)s_FanEyes.size(), opts.fanRays, st.visibleFan, unit, st.faceRays,
			 st.visibleFace, unit, st.mirrorRays, st.visibleMirror, st.reflectClusters );
		Msg( "vis: %d of the ray-visible faces first found by the -eyeconfirm pass (%d rays per eye)\n", st.visibleConfirm,
			 opts.confirmRays );
		Msg( "vis: %d faces kept visible with no ray test (%d eye clusters hold no eye sample)\n", st.visibleNoEye,
			 st.noEyeClusters );
		if ( st.facesWidened )
			Warning( "bspwalk: %d faces were sampled coarser than -facespacing %gu (up to %gu; the per-face sample cap)\n",
					 st.facesWidened, opts.faceSpacing, st.faceSpacingMax );
		if ( st.cappedFaces )
			Msg( "vis: %d face(s) (mirrors included) had more than %d candidate eyes: the tiles binned an even share of "
				 "every eye cluster, then %d face(s) the tiles left hidden paired every candidate (%lld eyes) with the "
				 "-eyeconfirm samples, and %d of those were seen only that way; %d kept visible untested (-eyeconfirm 0)\n",
				 st.cappedFaces, FACE_MAX_CANDIDATES, st.sweptFaces, st.sweepEyes, st.sweptVisible, st.cappedKept );
	}
	Msg( "vis timings (s): face clusters %.4f (leaf-less faces %.4f), tier A %.3f, occluders %.3f, fan %.3f (GPU setup "
		 "%.3f), mirror setup %.3f, face pass %.3f, total %.3f\n", st.tFaceClusters, st.tFaceBounds, st.tTierA,
		 st.tOccluders, st.tFan, st.tFanSetup, st.tMirror, st.tFace, st.tTotal );

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
	std::vector<int>().swap( s_EyeClOfs );
	std::vector<int>().swap( s_EyeByCl );
	std::vector<float>().swap( s_EyeClBox );
	std::vector<int>().swap( s_SlabFaces );
	std::vector<unsigned char>().swap( s_SlabMark );
	std::vector<int>().swap( s_SolidEyes );
	std::vector<int>().swap( s_FanEyes );
	std::vector<int>().swap( s_Reflective );
	std::vector< std::vector<dvec3> >().swap( s_MirrorEyes );
	std::vector< std::vector<int> >().swap( s_SolidSkip );
	std::vector<unsigned char>().swap( s_MirrorBare );
	std::vector<unsigned char>().swap( s_MirrorCapped );
	std::vector<int>().swap( s_PassFaces );
	std::vector<unsigned char>().swap( s_FaceCapped );
	for ( int t = 0; t <= MAX_TOOL_THREADS; t++ )
		s_FaceThreads[t] = FaceThread();
	s_pEyes = NULL;
	s_pVisOpts = NULL;
	(void)profile;
	(void)graph;
}
