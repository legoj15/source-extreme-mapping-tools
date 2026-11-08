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

#include <algorithm>
#include <atomic>
#include <memory>
#include <string>
#include <unordered_set>

// Occluders are shrunk by this (the safe direction: more is visible).
static const double OCC_SHRINK = 1.0 / 32.0;
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
// BVH over boxes
//-----------------------------------------------------------------------------
struct BvhNode
{
	dvec3 mins, maxs;
	int left, right;	// children, or -1
	int first, count;	// leaf items in 'items'
};

struct Bvh
{
	std::vector<BvhNode> nodes;
	std::vector<int> items;

	void Build( const std::vector<dvec3> &mins, const std::vector<dvec3> &maxs )
	{
		nodes.clear();
		items.resize( mins.size() );
		for ( size_t i = 0; i < items.size(); i++ )
			items[i] = (int)i;
		if ( !items.empty() )
			BuildNode( mins, maxs, 0, (int)items.size() );
	}

	int BuildNode( const std::vector<dvec3> &mins, const std::vector<dvec3> &maxs, int first, int count )
	{
		int idx = (int)nodes.size();
		nodes.push_back( BvhNode() );
		BvhNode nd;
		nd.mins = dvec3( 1e300, 1e300, 1e300 );
		nd.maxs = dvec3( -1e300, -1e300, -1e300 );
		for ( int i = first; i < first + count; i++ )
		{
			nd.mins = DMin( nd.mins, mins[items[i]] );
			nd.maxs = DMax( nd.maxs, maxs[items[i]] );
		}
		nd.left = nd.right = -1;
		nd.first = first;
		nd.count = count;
		if ( count > 4 )
		{
			dvec3 ext = nd.maxs - nd.mins;
			int axis = ( ext.x >= ext.y && ext.x >= ext.z ) ? 0 : ( ext.y >= ext.z ? 1 : 2 );
			int mid = first + count / 2;
			std::nth_element( items.begin() + first, items.begin() + mid, items.begin() + first + count, [&]( int a, int b ) {
				return mins[a][axis] + maxs[a][axis] < mins[b][axis] + maxs[b][axis];
			} );
			nd.left = BuildNode( mins, maxs, first, mid - first );
			nd.right = BuildNode( mins, maxs, mid, first + count - mid );
			nd.count = 0;
		}
		nodes[idx] = nd;
		return idx;
	}
};

// Does the segment o + t dir, t in [t0, t1], meet the box?
static inline bool SegBox( const dvec3 &o, const dvec3 &inv, double t0, double t1, const dvec3 &mins, const dvec3 &maxs )
{
	for ( int a = 0; a < 3; a++ )
	{
		double ta = ( mins[a] - o[a] ) * inv[a], tb = ( maxs[a] - o[a] ) * inv[a];
		if ( ta > tb )
		{
			double t = ta;
			ta = tb;
			tb = t;
		}
		// SafeInv clamps a zero direction to 1e300, so 0 * inv is 0; a NaN bound
		// (1/d overflowing to inf for a denormal d) must not narrow [t0, t1]
		if ( ta == ta )
			t0 = MAX( t0, ta );
		if ( tb == tb )
			t1 = MIN( t1, tb );
		if ( t0 > t1 )
			return false;
	}
	return true;
}

static inline dvec3 SafeInv( const dvec3 &d )
{
	return dvec3( d.x != 0 ? 1.0 / d.x : 1e300, d.y != 0 ? 1.0 / d.y : 1e300, d.z != 0 ? 1.0 / d.z : 1e300 );
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
// leaf (cluster -1, so no PVS row): a camera there draws every cluster, so a face in
// front of one stays visible (SolidViewInFront, WHY_SOLIDVIEW)
static std::vector<int> s_SolidEyes;
// An eye region box counts as in front of a face only past this: the eye
// descent hands each side of a node plane the piece parts up to 1e-4 beyond it
// (bw_eyes EYE_SPLIT_EPS), so boxes reach that far past faces on node planes.
// A sample counts as in front past 1e-6: an eye region reaching (1e-6, 1e-3]
// in front of a face whose samples are all behind it is accepted as hidden.
static const double REGION_MARGIN = 1e-3;
static std::vector<int> s_FanEyes;
static std::vector<int> s_Reflective;				// reflective faces that stay visible to tier A
static std::vector< std::vector<dvec3> > s_MirrorEyes;	// per s_Reflective: mirrored eyes in front of it
// per s_Reflective: MIRROR_BARE when no ray test can speak for it (a bare eye cluster may
// see it, or its eye region is untestable), else MIRROR_SOLID when a viewpoint sample in
// solid lies in front of it, else MIRROR_TESTED
enum { MIRROR_TESTED = 0, MIRROR_BARE = 1, MIRROR_SOLID = 2 };
static std::vector<unsigned char> s_MirrorBare;
static std::vector< std::vector<dvec3> > s_MirrorSolid;	// per MIRROR_SOLID mirror: those samples (real positions)
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

//-----------------------------------------------------------------------------
// Occlusion queries
//-----------------------------------------------------------------------------
static bool SegmentBlocked( const dvec3 &a, const dvec3 &b )
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

// Nearest occluder entry along the ray, FAN_MAX_T if none.
static double NearestOccluder( const dvec3 &o, const dvec3 &dir )
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

// Is p (on the face's plane) inside the convex polygon, within tol outward?
static bool InsidePolygon( const VisFace &f, const dvec3 &p, double tol )
{
	int n = (int)f.w.size();
	for ( int i = 0; i < n; i++ )
	{
		const dvec3 &a = f.w[i];
		const dvec3 &b = f.w[( i + 1 ) % n];
		dvec3 e = b - a;
		dvec3 m = DCross( e, f.n );	// outward when the winding is CCW about n
		double len = DLength( m );
		if ( len < 1e-12 )
			continue;
		if ( DDot( m, p - a ) > tol * len )
			return false;
	}
	return true;
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

// Marks every face the ray o + t dir crosses for t in [0, tMax], front side
// (or either side of a two-sided face).
static void MarkFacesAlong( const dvec3 &o, const dvec3 &dir, double tMax )
{
	if ( s_FaceBvh.nodes.empty() )
		return;
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
			if ( VisibleGet( fi ) )
				continue;
			const VisFace &f = s_Faces[fi];
			if ( f.disp )
			{
				const std::vector<dvec3> &tris = *f.tris;
				for ( size_t k = 0; k + 2 < tris.size(); k += 3 )
				{
					if ( RayHitsTriangle( o, dir, tMax, tris[k], tris[k + 1], tris[k + 2] ) )
					{
						VisibleSet( fi, WHY_FAN + 1 );
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
				VisibleSet( fi, WHY_FAN + 1 );
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
static bool OpaqueSolid( int b )
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
			return true;					// too many sides to test: assume open (not an occluder)
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

static void AddOccluder( const ConvexShape &shape, std::vector<dvec3> &mins, std::vector<dvec3> &maxs )
{
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

static void BuildOccluders( const CollisionWorld &world, VisStats &st )
{
	s_OccPlanes.clear();
	s_OccOfs.assign( 1, 0 );
	std::vector<dvec3> mins, maxs;
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
		if ( !OpaqueSolid( b ) || AllNodraw( b ) || OpenNodrawSide( b ) )
		{
			st.occludersSkipped++;
			continue;
		}
		if ( !BuildBrushShape( b, shape ) )
			continue;
		AddOccluder( shape, mins, maxs );
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
		AddOccluder( sc.shape, mins, maxs );
	}
	st.occluders = (int)mins.size();
	s_OccBvh.Build( mins, maxs );
}

//-----------------------------------------------------------------------------
// Face samples
//-----------------------------------------------------------------------------
struct FaceSample
{
	dvec3 p;		// lifted off the face
	dvec3 n;		// the surface normal there
};

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
		MarkFacesAlong( o, dir, t + FAN_PAST );
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
static const double FAN_GPU_SHRINK = 1.0 / 32.0;
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
	bool ok = BwOptix_FanSetup( ov, ot, fv, ft, prim, faceN, plates, numPlates, dirs, eyes, start );
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
							   (float)FAN_GPU_PAD, check ? tbuf.data() : NULL );
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
// (any such sample when !cull: a two-sided face or a displacement)? A camera there
// draws every cluster, and it sits inside the brush that makes the leaf solid, so
// every ray to it would be blocked by that brush although the engine draws through
// it (its sides face away). What lies between is ignored: a false visible costs
// budget, a false hidden is the error that must not happen (#76).
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

// Does eye cluster c's region (the box of its pieces' parts) reach in front
// of the face? c < 0: without vis data the buckets are cubes, not clusters,
// so the test is made once for all eyes against the box of all pieces (vbsp
// numbers clusters without vvis too, so regionLo is not empty then).
static bool RegionInFront( const VisFace &f, int c )
{
	const EyeVolume &ev = *s_pEyes;
	bool noPvs = c < 0;
	if ( noPvs ? !ev.allAny : c >= (int)ev.regionLo.size() )
		return false;
	const dvec3 &lo = noPvs ? ev.allLo : ev.regionLo[c], &hi = noPvs ? ev.allHi : ev.regionHi[c];
	double far = f.n.x * ( f.n.x > 0 ? hi.x : lo.x ) + f.n.y * ( f.n.y > 0 ? hi.y : lo.y ) +
				 f.n.z * ( f.n.z > 0 ? hi.z : lo.z );
	return far > f.d + REGION_MARGIN;
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
				untestable = RegionInFront( f, c );
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
// caller then tests the rest through CandidateChunk, SweepCandidates). The
// samples of viewpoints in solid are not candidates (SolidViewInFront decides
// for them). Returns true when no ray test can settle the face, so the caller
// keeps it visible: an eye cluster that may see the face has no sample in front of
// it while its eye region (the box of its pieces' parts) reaches more than
// REGION_MARGIN in front (the lattice and the piece vertices can all lie behind
// a plane the pieces cross by up to a spacing). Without vis data the eye
// buckets are 512u cubes with no region of their own: the check is made once,
// against the box of all pieces, when no eye at all is in front.
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
			const DPlane *planes = &s_OccPlanes[s_OccOfs[o]];
			int numPlanes = s_OccOfs[o + 1] - s_OccOfs[o];
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
			best = MAX( best, depthAt( 0.5 * ( lo + hi ) ) );
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
	bool gpuFree = false, cpuFree = false;
	for ( size_t g = 0; g + per <= n && !( gpuFree && !s_bVisCheck ); g += per )
	{
		bool f = true;
		for ( int k = 0; k < per; k++ )
			f = f && !t.blocked[g + k];
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
			bool cb = SegmentBlocked( sg.a, sg.b );
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
			f = f && !cb;
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
		return SegmentBlocked( s.p, eye ) ? 0 : 1;
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
	return ( !SegmentBlocked( s.p, q ) && !SegmentBlocked( q, viaEye ) ) ? 1 : 0;
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
			if ( ok )
			{
				s_Counters[iThread].rays[1] += (long long)t.chunk.size() * rays;
				s_Counters[iThread].sweepEyes += (long long)t.chunk.size();
				if ( clear )
					return true;
				base += t.chunk.size();
				continue;
			}
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
// part of the cluster's eye region (its box) in front of the face's plane (any
// part for a two-sided face or a displacement)?
static bool BareMaySee( const VisFace &f, int fi, bool cull )
{
	const EyeVolume &ev = *s_pEyes;
	for ( size_t i = 0; i < ev.bareClusters.size(); i++ )
	{
		if ( cull )
		{
			const dvec3 &lo = ev.bareLo[i], &hi = ev.bareHi[i];
			// the box corner farthest in front of the plane
			double far = f.n.x * ( f.n.x > 0 ? hi.x : lo.x ) + f.n.y * ( f.n.y > 0 ? hi.y : lo.y ) +
						 f.n.z * ( f.n.z > 0 ? hi.z : lo.z );
			if ( far <= f.d + REGION_MARGIN )
				continue;
		}
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
		VisibleSet( fi, WHY_NOEYE + 1 );
		return;
	}
	t.tiles.clear();
	for ( size_t i = 0; i < t.samples.size(); i++ )
		t.tiles.push_back( std::make_pair( TileKey( t.samples[i].p ), (int)i ) );
	std::sort( t.tiles.begin(), t.tiles.end() );

	if ( CandidateEyes( fi, t.cand, t.cands ) )
	{
		VisibleSet( fi, WHY_NOEYE + 1 );
		return;
	}
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
	bool solidMirror = false;	// a viewpoint sample in solid lies in front of a mirror the face shows in
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
			s_Counters[iThread].sweptFaces += swept;
			VisibleSet( fi, WHY_NOEYE + 1 );
			return;
		}
		// A viewpoint in solid sees the face in R when the face is in front of its mirror
		// image (any image for a two-sided face or a displacement): the face pass's
		// in-front test, taken through the mirror; what lies between is ignored.
		for ( size_t k = 0; k < s_MirrorSolid[r].size() && !solidMirror; k++ )
		{
			dvec3 m = MirrorPoint( s_MirrorSolid[r][k], R.n, R.d );
			solidMirror = ( f.twoSided || f.disp ) || DDot( f.n, m ) > f.d + 1e-6;
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
	// Every ray failed. A viewpoint in a solid leaf in front of the face, or of a
	// mirror it shows in, still keeps it visible (SolidViewInFront). Tested last so
	// that solid_view marks only the faces this rule keeps; tier A applied the same
	// test to the faces the PVS hid.
	if ( solidMirror || SolidViewInFront( f, !f.twoSided && !f.disp ) )
		VisibleSet( fi, WHY_SOLIDVIEW + 1 );
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
	// a viewpoint in solid in front of it sees it whatever lies between (as SolidViewInFront):
	// the faces that face its mirror image and that no ray reaches stay visible (FacePassOne)
	if ( bare == MIRROR_TESTED )
	{
		for ( size_t i = 0; i < s_SolidEyes.size(); i++ )
		{
			const EyeSample &e = samples[s_SolidEyes[i]];
			if ( EyeInFront( R, e ) )
				s_MirrorSolid[r].push_back( dvec3( e.x, e.y, e.z ) );
		}
		if ( !s_MirrorSolid[r].empty() )
			bare = MIRROR_SOLID;
	}
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

	// Real viewpoint samples in a solid leaf (cluster -1, so no PVS row): a camera there draws every
	// cluster, so the faces in front of it stay visible in both tiers (SolidViewInFront, #76). They
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
				// a solid viewpoint in front of it keeps it visible below, so it shows its reflection too
				if ( !vis && s_Faces[f].model == 0 && !SolidViewInFront( s_Faces[f], true ) )
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
			if ( !vis && SolidViewInFront( s_Faces[f], !s_Faces[f].twoSided && !s_Faces[f].disp ) )
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

	//
	// Tier B
	//
	// A viewpoint in a solid leaf does not stop it: the ray tier's eyes (per cluster,
	// with their PVS) do not describe what that camera sees, so the face pass keeps
	// visible every face in front of one that no ray reaches (WHY_SOLIDVIEW).
	const bool tierB = opts.tierB;
	st.tierBRan = tierB;
	s_Visible.reset( new std::atomic<unsigned char>[nf] );
	for ( int f = 0; f < nf; f++ )
		VisibleSet( f, 0 );
	if ( tierB )
	{
		double to = Plat_FloatTime();
		BuildOccluders( world, st );
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
		st.tOccluders = Plat_FloatTime() - to;

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
		s_MirrorSolid.assign( s_Reflective.size(), std::vector<dvec3>() );
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
		Msg( "vis: %d occluders (%d see-through solids left out); fan %d eyes x %d rays -> %d faces; face %s %lld -> %d, "
			 "mirror %s %lld -> %d; %d reflective clusters added to tier A\n",
			 st.occluders, st.occludersSkipped, (int)s_FanEyes.size(), opts.fanRays, st.visibleFan, unit, st.faceRays,
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
	std::vector< std::vector<dvec3> >().swap( s_MirrorSolid );
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
