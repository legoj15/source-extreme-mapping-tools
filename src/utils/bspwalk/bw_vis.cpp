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
#include "bw_disp.h"
#include "bw_optix.h"
#include <mutex>
#include "bw_hash.h"
#include "bw_util.h"

#include <algorithm>
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
// At most this many candidate eyes per face (an even stride over them beyond):
// many times the eyes the finest bin level can use.
static const int FACE_MAX_CANDIDATES = 1 << 16;
// A mirror hit point may lie this far outside the reflective face (safe side).
static const double MIRROR_TOL = 1e-2;

const char *VisReasonName( int why )
{
	static const char *s_Names[WHY_COUNT] = { "pvs", "rays", "fan", "ray", "mirror", "brush_entity", "no_tierb", "bad_lump",
											  "no_eye" };
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
		// NaN (0 * inf) on a flat box: keep the segment
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
	std::vector<dvec3> tris;	// displacement render triangles (3 points each)
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
static std::vector<unsigned char> s_Visible;		// per face, set by the ray passes
static std::vector<unsigned char> s_Tier;
static int s_nClusters, s_RowBytes;
static std::vector<unsigned char> s_Pvs;			// numClusters rows
static const EyeVolume *s_pEyes;
static const VisOptions *s_pVisOpts;
static std::vector<int> s_EyeClOfs, s_EyeByCl;		// cluster -> eye samples (CSR)
static bool s_bBarePvs;				// vis data present: a bare cluster sees only its PVS (else it may see every face)
// unmapped samples of viewpoint sources whose point is in a solid leaf (cluster
// -1, so no PVS row): a camera there draws every cluster, so they are candidate
// eyes of every face
static std::vector<int> s_SolidEyes;
static const int FACE_MAX_EXTRA = 1 << 12;	// at most this many of them per face (CandidateEyes)
static std::vector<int> s_FanEyes;
static std::vector<int> s_Reflective;				// reflective faces that stay visible to tier A
static std::vector< std::vector<dvec3> > s_MirrorEyes;	// per s_Reflective: mirrored eyes in front of it
static std::vector< std::vector<dvec3> > s_MirrorExtra;	// the same for the extra eyes (CandidateEyes)
static std::vector<unsigned char> s_MirrorBare;		// per s_Reflective: a bare eye cluster may see it
static std::vector<int> s_PassFaces;
static long long s_RayCount[MAX_TOOL_THREADS + 1][3];
// GPU face / mirror passes (the arc kernel on BW_MESH_VIS, segments as zero-gravity arcs)
static bool s_bVisGpu, s_bVisCheck, s_bVisSim;
static float s_VisG;
static std::mutex s_GpuMutex;
static long long s_CheckSeg[MAX_TOOL_THREADS + 1][3];	// segments, gpu-only blocked, cpu-only blocked
static int s_CheckTiles[MAX_TOOL_THREADS + 1];

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
			if ( s_Visible[fi] )
				continue;
			const VisFace &f = s_Faces[fi];
			if ( f.disp )
			{
				for ( size_t k = 0; k + 2 < f.tris.size(); k += 3 )
				{
					if ( RayHitsTriangle( o, dir, tMax, f.tris[k], f.tris[k + 1], f.tris[k + 2] ) )
					{
						s_Visible[fi] = WHY_FAN + 1;
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
				s_Visible[fi] = WHY_FAN + 1;
		}
	}
}

//-----------------------------------------------------------------------------
// Materials: the Water shader and $nocull. Cached per texdata.
//-----------------------------------------------------------------------------
struct MatInfo
{
	bool loaded, water, nocull;
};

static bool ReadVmt( const char *pszName, int depth, MatInfo &mi )
{
	char path[MAX_PATH];
	Q_snprintf( path, sizeof( path ), "materials/%s.vmt", pszName );
	Q_FixSlashes( path, '/' );
	CUtlBuffer buf( 0, 0, CUtlBuffer::TEXT_BUFFER );
	if ( !g_pFullFileSystem || !g_pFullFileSystem->ReadFile( path, NULL, buf ) )
		return false;
	KeyValues *kv = new KeyValues( "vmt" );
	bool ok = kv->LoadFromBuffer( path, buf );
	if ( ok )
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
				ReadVmt( stripped, depth + 1, mi );
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
		mi.loaded = true;
	}
	kv->deleteThis();
	return ok;
}

static void LoadMaterials( std::vector<MatInfo> &mats )
{
	mats.assign( numtexdata, MatInfo() );
	for ( int i = 0; i < numtexdata; i++ )
	{
		MatInfo mi = { false, false, false };
		ReadVmt( TexDataStringTable_GetString( dtexdata[i].nameStringTableID ), 0, mi );
		mats[i] = mi;
	}
}

//-----------------------------------------------------------------------------
// Faces
//-----------------------------------------------------------------------------
static void BuildFaces( VisStats &st )
{
	int nf = numfaces;
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

	// model -> entity origin (func_reflective_glass and friends sit at it)
	std::vector<dvec3> modelOrigin( nummodels, dvec3( 0, 0, 0 ) );
	std::vector<unsigned char> modelGlass( nummodels, 0 );
	for ( int e = 0; e < num_entities; e++ )
	{
		const char *model = EntValue( &entities[e], "model" );
		if ( model[0] != '*' )
			continue;
		int m = atoi( model + 1 );
		if ( m <= 0 || m >= nummodels )
			continue;
		modelOrigin[m] = EntOrigin( &entities[e] );
		if ( !Q_stricmp( EntValue( &entities[e], "classname" ), "func_reflective_glass" ) )
			modelGlass[m] = 1;
	}

	std::vector<MatInfo> mats;
	LoadMaterials( mats );

	std::vector< std::vector<dvec3> > dispTris;
	std::vector<int> dispFace;
	DispRenderTriangles( dispTris, dispFace );

	for ( int fi = 0; fi < nf; fi++ )
	{
		const dface_t &df = dfaces[fi];
		VisFace &f = s_Faces[fi];
		f.model = faceModel[fi];
		const dplane_t &pl = dplanes[df.planenum];
		f.n = dvec3( pl.normal.x, pl.normal.y, pl.normal.z );
		f.d = pl.dist;
		dvec3 org = modelOrigin[f.model];
		// a face whose lump data is out of range (DispFaceProblem rejects its
		// displacement, or an edge index is bad) is never evaluated
		f.bad = ( df.dispinfo >= 0 && DispFaceProblem( &df ) ) || df.firstedge < 0 || df.numedges < 0 ||
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
			f.w.push_back( dvec3( v.point.x, v.point.y, v.point.z ) + org );
		}
		if ( f.bad )
			f.w.clear();
		f.d += DDot( f.n, org );
		f.disp = !f.bad && df.dispinfo >= 0 && df.dispinfo < (int)dispTris.size() && !dispTris[df.dispinfo].empty();
		if ( f.disp )
			f.tris = dispTris[df.dispinfo];
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
		const std::vector<dvec3> &pts = f.disp ? f.tris : f.w;
		for ( size_t k = 0; k < pts.size(); k++ )
		{
			f.mins = DMin( f.mins, pts[k] );
			f.maxs = DMax( f.maxs, pts[k] );
		}
		int ti = df.texinfo;
		bool warp = false;
		MatInfo mi = { false, false, false };
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

static void BuildFaceClusters( VisStats &st )
{
	int nf = numfaces;
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
	std::vector<unsigned char> bits( s_nClusters );
	for ( int f = 0; f < nf; f++ )
	{
		// a brush entity's faces sit in its submodel's leaves, which have no
		// cluster: a reflective one takes the clusters its bounds touch
		if ( s_Faces[f].model ? !s_Faces[f].reflective : listed[f] )
			continue;
		st.faceNoLeaf += !s_Faces[f].model;
		std::fill( bits.begin(), bits.end(), 0 );
		BoxClusters( s_Faces[f].mins, s_Faces[f].maxs, bits );
		for ( int c = 0; c < s_nClusters; c++ )
		{
			if ( bits[c] )
				pairs.push_back( std::make_pair( f, c ) );
		}
	}
	std::sort( pairs.begin(), pairs.end() );
	s_FaceClOfs.assign( nf + 1, 0 );
	s_FaceCl.clear();
	size_t k = 0;
	for ( int f = 0; f < nf; f++ )
	{
		s_FaceClOfs[f] = (int)s_FaceCl.size();
		for ( ; k < pairs.size() && pairs[k].first == f; k++ )
		{
			if ( pairs[k].second >= 0 )
				s_FaceCl.push_back( pairs[k].second );
		}
	}
	s_FaceClOfs[nf] = (int)s_FaceCl.size();
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
		if ( s.bevel & 0xFF )	// the high byte is "thin" on BSP v20
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
		if ( !( s.bevel & 0xFF ) && SideNodraw( s ) )
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
		if ( s.bevel & 0xFF )
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

// spUsed: the grid spacing the face got (faceSpacing, or coarser for a huge face).
static void SampleFace( const VisFace &f, std::vector<FaceSample> &out, double &spUsed )
{
	out.clear();
	double sp = s_pVisOpts->faceSpacing;
	spUsed = sp;
	if ( f.disp )
	{
		for ( size_t k = 0; k + 2 < f.tris.size(); k += 3 )
		{
			const dvec3 &a = f.tris[k], &b = f.tris[k + 1], &c = f.tris[k + 2];
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
	double w0 = DDot( f.w[0], f.n );
	for ( double pu = ceil( u0 / sp ) * sp; pu <= u1; pu += sp )
	{
		for ( double pv = ceil( v0 / sp ) * sp; pv <= v1; pv += sp )
		{
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
static void FanOne( int iThread, int item )
{
	const EyeSample &e = s_pEyes->samples[s_FanEyes[item]];
	dvec3 o( e.x, e.y, e.z );
	int n = s_pVisOpts->fanRays;
	for ( int i = 0; i < n; i++ )
	{
		dvec3 dir = FibonacciDir( i, n );
		double t = NearestOccluder( o, dir );
		MarkFacesAlong( o, dir, t + FAN_PAST );
	}
	s_RayCount[iThread][0] += n;
}

// The eyes that may see a face: those in clusters whose PVS holds one of its
// clusters (all eyes without vis data), in front of its plane unless it is
// two-sided or a displacement. The lattice samples go to 'out' (binned and
// thinned out); the samples of viewpoints in solid go to 'extra'. The callers
// keep the face visible when 'extra' is non-empty (such an eye sits inside the
// brush that would block its rays). Returns true when 'extra' overflowed
// FACE_MAX_EXTRA (it is then cleared): the caller keeps the face visible too.
static bool CandidateEyes( int fi, std::vector<int> &out, std::vector<int> &extra )
{
	out.clear();
	extra.clear();
	const VisFace &f = s_Faces[fi];
	const std::vector<EyeSample> &eyes = s_pEyes->samples;
	int nCl = (int)s_EyeClOfs.size() - 1;
	bool cull = !f.twoSided && !f.disp;
	for ( int c = 0; c < nCl; c++ )
	{
		int a = s_EyeClOfs[c], b = s_EyeClOfs[c + 1];
		if ( a == b )
			continue;
		if ( !s_Pvs.empty() )
		{
			bool any = false;
			for ( int k = s_FaceClOfs[fi]; k < s_FaceClOfs[fi + 1] && !any; k++ )
				any = PvsBit( c, s_FaceCl[k] );
			if ( !any )
				continue;
		}
		for ( int i = a; i < b; i++ )
		{
			const EyeSample &e = eyes[s_EyeByCl[i]];
			if ( cull && f.n.x * e.x + f.n.y * e.y + f.n.z * e.z <= f.d + 1e-6 )
				continue;
			out.push_back( s_EyeByCl[i] );
		}
	}
	// a camera in a solid leaf draws every cluster: its samples may see any face
	for ( size_t i = 0; i < s_SolidEyes.size(); i++ )
	{
		const EyeSample &e = eyes[s_SolidEyes[i]];
		if ( cull && f.n.x * e.x + f.n.y * e.y + f.n.z * e.z <= f.d + 1e-6 )
			continue;
		extra.push_back( s_SolidEyes[i] );
	}
	bool overflow = (int)extra.size() > FACE_MAX_EXTRA;
	if ( overflow )
		extra.clear();						// the caller keeps the face visible
	if ( (int)out.size() > FACE_MAX_CANDIDATES )
	{
		size_t n = out.size();
		for ( int k = 0; k < FACE_MAX_CANDIDATES; k++ )
			out[k] = out[(size_t)( (double)k * n / FACE_MAX_CANDIDATES )];
		out.resize( FACE_MAX_CANDIDATES );
	}
	return overflow;
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
	if ( unused <= bins * bins )
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
		std::vector<int> best( bins * bins, -1 );
		std::vector<double> bestD( bins * bins, 1e300 );
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
	std::vector<unsigned char> blocked;
	std::vector<FaceSample> samples;
	std::vector<int> cand, xcand;
	std::vector<dvec3> pts, via;
	std::vector<dvec3> xpts, xvia;		// extra eyes (CandidateEyes): added whole to the first tile level
	std::vector<TileEye> tileEyes;
	std::vector<unsigned char> used;
	std::vector< std::pair<long long, int> > tiles;
};
static FaceThread s_FaceThreads[MAX_TOOL_THREADS + 1];

static long long TileKey( const dvec3 &p )
{
	long long x = (long long)floor( p.x / FACE_TILE ) + ( 1 << 20 );
	long long y = (long long)floor( p.y / FACE_TILE ) + ( 1 << 20 );
	long long z = (long long)floor( p.z / FACE_TILE ) + ( 1 << 20 );
	return ( x << 42 ) | ( y << 21 ) | z;
}

static void PushSegment( std::vector<BwArc> &out, const dvec3 &a, const dvec3 &b )
{
	BwArc s;
	s.ox = (float)a.x;
	s.oy = (float)a.y;
	s.oz = (float)a.z;
	s.vx = (float)( b.x - a.x );
	s.vy = (float)( b.y - a.y );
	s.vz = (float)( b.z - a.z );
	s.tKick = 0.0f;
	s.vzKick = 0.0f;
	s.dg0 = -s_VisG;	// gravity + dg0 = 0 exactly: a straight chord
	s.tEnd = 1.0f;
	s.nSeg[0] = 1;
	s.nSeg[1] = 0;
	s.mesh = BW_MESH_VIS;
	out.push_back( s );
}

// GPU: is some path of t.arcs (groups of 'per' consecutive segments, all of a
// group clear) free? With -vischeck the CPU traces the same segments, the
// answer follows it, and the disagreements are counted.
static bool GpuAnyFree( int iThread, FaceThread &t, int per )
{
	size_t n = t.arcs.size();
	t.blocked.assign( n, 1 );
	bool ok;
	if ( s_bVisSim )
		ok = false;	// -visgpusim: the fallback below traces the float segments
	else
	{
		std::lock_guard<std::mutex> lock( s_GpuMutex );
		ok = BwOptix_TraceArcs( t.arcs.data(), (unsigned int)n, s_VisG, t.blocked.data() );
	}
	if ( !ok )
	{
		// a failed launch: trace these on the CPU (the caller keeps going)
		for ( size_t i = 0; i < n; i++ )
		{
			const BwArc &a = t.arcs[i];
			dvec3 o( a.ox, a.oy, a.oz );
			t.blocked[i] = SegmentBlocked( o, o + dvec3( a.vx, a.vy, a.vz ) );
		}
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
			const BwArc &a = t.arcs[g + k];
			dvec3 o( a.ox, a.oy, a.oz );
			bool cb = SegmentBlocked( o, o + dvec3( a.vx, a.vy, a.vz ) );
			s_CheckSeg[iThread][0]++;
			if ( t.blocked[g + k] && !cb )
				s_CheckSeg[iThread][1]++;
			else if ( !t.blocked[g + k] && cb )
				s_CheckSeg[iThread][2]++;
			f = f && !cb;
		}
		cpuFree = cpuFree || f;
	}
	if ( cpuFree != gpuFree )
		s_CheckTiles[iThread]++;
	return cpuFree;
}

// Tests the samples tile by tile against the binned points; true at the first
// clear ray. A tile that finds none tries again with twice the bins over the
// eyes it has not tried, up to binLevels times. mirror >= 0: points are
// mirrored eyes behind s_Reflective[mirror].
static bool TraceTiles( int iThread, FaceThread &t, int mirror )
{
	int bins0 = s_pVisOpts->bins;
	int levels = s_pVisOpts->binLevels;
	while ( levels > 1 && ( (long long)bins0 << ( levels - 1 ) ) > 1024 )
		levels--;	// at most 1024^2 bins: no int overflow
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
		for ( int level = 0; level < levels && ( unused > 0 || ( level == 0 && !t.xpts.empty() ) ); level++ )
		{
			BinEyes( c, t.pts, mirror >= 0 ? &t.via : NULL, bins0 << level, t.used, unused, t.tileEyes );
			for ( size_t x = 0; level == 0 && x < t.xpts.size(); x++ )
			{
				TileEye te;
				te.p = t.xpts[x];
				dvec3 d = te.p - c;
				te.dist2 = DDot( d, d );
				te.viaEye = mirror >= 0 ? t.xvia[x] : t.xpts[x];
				t.tileEyes.push_back( te );
			}
			if ( s_bVisGpu )
			{
				// every pair of this tile level in one launch
				t.arcs.clear();
				for ( size_t k = a; k < b; k++ )
				{
					const FaceSample &s = t.samples[t.tiles[k].second];
					if ( pR && DDot( pR->n, s.p ) <= pR->d )
						continue;
					for ( size_t e = 0; e < t.tileEyes.size(); e++ )
					{
						const TileEye &te = t.tileEyes[e];
						dvec3 d = te.p - s.p;
						if ( !t.twoSided && DDot( d, s.n ) <= 0 )
							continue;
						if ( !pR )
						{
							PushSegment( t.arcs, s.p, te.p );
							continue;
						}
						double den = DDot( pR->n, d );
						if ( fabs( den ) < 1e-12 )
							continue;
						double u = ( pR->d - DDot( pR->n, s.p ) ) / den;
						if ( u <= 0 || u >= 1 )
							continue;
						dvec3 q = s.p + d * u;
						if ( !InsidePolygon( *pR, q, MIRROR_TOL ) )
							continue;
						PushSegment( t.arcs, s.p, q );
						PushSegment( t.arcs, q, te.viaEye );
					}
				}
				s_RayCount[iThread][pR ? 2 : 1] += (long long)t.arcs.size();
				if ( !t.arcs.empty() && GpuAnyFree( iThread, t, pR ? 2 : 1 ) )
					return true;
				continue;
			}
			for ( size_t k = a; k < b; k++ )
			{
				const FaceSample &s = t.samples[t.tiles[k].second];
				if ( pR && DDot( pR->n, s.p ) <= pR->d )
					continue;	// behind the mirror: not in its reflection
				for ( size_t e = 0; e < t.tileEyes.size(); e++ )
				{
					const TileEye &te = t.tileEyes[e];
					dvec3 d = te.p - s.p;
					if ( !t.twoSided && DDot( d, s.n ) <= 0 )
						continue;	// the sample's back side
					if ( !pR )
					{
						s_RayCount[iThread][1]++;
						if ( !SegmentBlocked( s.p, te.p ) )
							return true;
						continue;
					}
					double den = DDot( pR->n, d );
					if ( fabs( den ) < 1e-12 )
						continue;
					double u = ( pR->d - DDot( pR->n, s.p ) ) / den;
					if ( u <= 0 || u >= 1 )
						continue;
					dvec3 q = s.p + d * u;
					if ( !InsidePolygon( *pR, q, MIRROR_TOL ) )
						continue;
					s_RayCount[iThread][2]++;
					if ( !SegmentBlocked( s.p, q ) && !SegmentBlocked( q, te.viaEye ) )
						return true;
				}
			}
		}
		a = b;
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
			if ( far <= f.d + 1e-6 )
				continue;
		}
		if ( !s_bBarePvs || FaceInPvsOf( fi, ev.bareClusters[i] ) )
			return true;
	}
	return false;
}

static void FacePassOne( int iThread, int item )
{
	int fi = s_PassFaces[item];
	if ( s_Visible[fi] )
		return;
	FaceThread &t = s_FaceThreads[iThread];
	t.twoSided = s_Faces[fi].twoSided;
	double spUsed;
	SampleFace( s_Faces[fi], t.samples, spUsed );
	if ( spUsed > s_pVisOpts->faceSpacing )
	{
		t.facesWidened++;
		t.spacingMax = MAX( t.spacingMax, spUsed );
	}
	// No ray test is possible for a face without samples, or from an eye
	// cluster without samples (a piece that only grazes a thin leaf, missing
	// every vertex and lattice point) that may see it: the face stays visible.
	// Clusters added for reflective faces are not eye piece clusters; the
	// mirror pass decides those.
	if ( t.samples.empty() || BareMaySee( s_Faces[fi], fi, !s_Faces[fi].twoSided && !s_Faces[fi].disp ) )
	{
		s_Visible[fi] = WHY_NOEYE + 1;
		return;
	}
	t.tiles.clear();
	for ( size_t i = 0; i < t.samples.size(); i++ )
		t.tiles.push_back( std::make_pair( TileKey( t.samples[i].p ), (int)i ) );
	std::sort( t.tiles.begin(), t.tiles.end() );

	// A viewpoint in a solid leaf sits inside the brush that makes the leaf
	// solid, so every ray to it would be blocked by that brush, yet the engine
	// draws through it (its sides face away). Such an eye in front of the face
	// keeps the face visible without a ray test.
	if ( CandidateEyes( fi, t.cand, t.xcand ) || !t.xcand.empty() )
	{
		s_Visible[fi] = WHY_NOEYE + 1;
		return;
	}
	t.pts.clear();
	for ( size_t i = 0; i < t.cand.size(); i++ )
	{
		const EyeSample &e = s_pEyes->samples[t.cand[i]];
		t.pts.push_back( dvec3( e.x, e.y, e.z ) );
	}
	t.xpts.clear();
	for ( size_t i = 0; i < t.xcand.size(); i++ )
	{
		const EyeSample &e = s_pEyes->samples[t.xcand[i]];
		t.xpts.push_back( dvec3( e.x, e.y, e.z ) );
	}
	if ( ( !t.pts.empty() || !t.xpts.empty() ) && TraceTiles( iThread, t, -1 ) )
	{
		s_Visible[fi] = WHY_RAY + 1;
		return;
	}

	// through each reflective face that stays visible
	const VisFace &f = s_Faces[fi];
	for ( size_t r = 0; r < s_Reflective.size(); r++ )
	{
		if ( s_Reflective[r] == fi || ( s_MirrorEyes[r].empty() && s_MirrorExtra[r].empty() && !s_MirrorBare[r] ) )
			continue;
		const VisFace &R = s_Faces[s_Reflective[r]];
		// some part of the face must lie in front of the mirror
		bool front = false;
		const std::vector<dvec3> &pts = f.disp ? f.tris : f.w;
		for ( size_t k = 0; k < pts.size() && !front; k++ )
			front = DDot( R.n, pts[k] ) > R.d + 1e-6;
		if ( !front )
			continue;
		if ( s_MirrorBare[r] )
		{
			// a bare eye cluster may see the mirror: no ray test is possible
			s_Visible[fi] = WHY_NOEYE + 1;
			return;
		}
		t.pts.clear();
		t.via.clear();
		t.xpts.clear();
		t.xvia.clear();
		bool cull = !f.twoSided && !f.disp;
		for ( int x = 0; x < 2; x++ )
		{
			const std::vector<dvec3> &me = x ? s_MirrorExtra[r] : s_MirrorEyes[r];
			for ( size_t k = 0; k < me.size(); k++ )
			{
				dvec3 m = MirrorPoint( me[k], R.n, R.d );
				if ( cull && DDot( f.n, m ) <= f.d + 1e-6 )
					continue;
				( x ? t.xpts : t.pts ).push_back( m );
				( x ? t.xvia : t.via ).push_back( me[k] );
			}
		}
		if ( ( !t.pts.empty() || !t.xpts.empty() ) && TraceTiles( iThread, t, (int)r ) )
		{
			s_Visible[fi] = WHY_MIRROR + 1;
			return;
		}
	}
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

	BuildFaces( st );
	BuildFaceClusters( st );
	out.tier.assign( nf, TIER_VISIBLE );
	out.why.assign( nf, WHY_RAY );

	//
	// Tier A
	//
	double ta = Plat_FloatTime();
	std::vector<unsigned char> eyeCl( eyes.clusters );
	std::vector<unsigned char> seen( s_nClusters, 0 );
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
		// reflective faces that stay visible see like eyes: iterate
		std::vector<unsigned char> reflAdded( nf, 0 );
		for ( ;; )
		{
			std::fill( seen.begin(), seen.end(), 0 );
			for ( int c = 0; c < s_nClusters; c++ )
			{
				if ( !eyeCl[c] )
					continue;
				const unsigned char *row = &s_Pvs[(size_t)c * s_RowBytes];
				for ( int k = 0; k < s_nClusters; k++ )
				{
					if ( ( row[k >> 3] >> ( k & 7 ) ) & 1 )
						seen[k] = 1;
				}
				seen[c] = 1;
			}
			bool grew = false;
			for ( int f = 0; f < nf; f++ )
			{
				if ( !s_Faces[f].reflective || reflAdded[f] )
					continue;
				bool vis = false;
				for ( int k = s_FaceClOfs[f]; k < s_FaceClOfs[f + 1] && !vis; k++ )
					vis = seen[s_FaceCl[k]] != 0;
				// a brush entity's mirror (func_reflective_glass) is never tier A
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
						grew = true;
					}
				}
				for ( int k = s_FaceClOfs[f]; k < s_FaceClOfs[f + 1]; k++ )
				{
					if ( !eyeCl[s_FaceCl[k]] )
					{
						eyeCl[s_FaceCl[k]] = 1;
						st.reflectClusters++;
						grew = true;
					}
				}
			}
			if ( !grew )
				break;
		}
		for ( int f = 0; f < nf; f++ )
		{
			if ( s_Faces[f].model || s_Faces[f].bad )
				continue;
			bool vis = false;
			for ( int k = s_FaceClOfs[f]; k < s_FaceClOfs[f + 1] && !vis; k++ )
				vis = seen[s_FaceCl[k]] != 0;
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
	s_Visible.assign( nf, 0 );
	if ( opts.tierB )
	{
		double to = Plat_FloatTime();
		BuildOccluders( world, st );
		// GPU path for the face and mirror passes: the occluders as one mesh
		s_bVisGpu = s_bVisCheck = false;
		s_bVisSim = opts.gpuSim;
		s_VisG = (float)profile.gravity;
		if ( s_bVisSim )
		{
			s_bVisGpu = true;
			s_bVisCheck = opts.check;
			Msg( "vis: -visgpusim: GPU batching, each segment traced on the CPU\n" );
		}
		else if ( !opts.cpuOnly )
		{
			if ( BwOptix_Init() )
			{
				std::vector<float> mv;
				std::vector<unsigned int> mt;
				ConvexShape cs;
				for ( size_t o = 0; o + 1 < s_OccOfs.size(); o++ )
				{
					if ( !ConvexFromPlanes( &s_OccPlanes[s_OccOfs[o]], s_OccOfs[o + 1] - s_OccOfs[o], cs ) )
						continue;
					unsigned int base = (unsigned int)( mv.size() / 3 );
					for ( size_t k = 0; k < cs.verts.size(); k++ )
					{
						mv.push_back( (float)cs.verts[k].x );
						mv.push_back( (float)cs.verts[k].y );
						mv.push_back( (float)cs.verts[k].z );
					}
					for ( size_t f = 0; f < cs.faces.size(); f++ )
					{
						for ( size_t k = 1; k + 1 < cs.faces[f].size(); k++ )
						{
							mt.push_back( base + cs.faces[f][0] );
							mt.push_back( base + cs.faces[f][k] );
							mt.push_back( base + cs.faces[f][k + 1] );
						}
					}
				}
				st.meshTris = (int)( mt.size() / 3 );
				if ( mt.empty() || BwOptix_BuildMesh( BW_MESH_VIS, mv, mt ) )
				{
					s_bVisGpu = !mt.empty();
					s_bVisCheck = s_bVisGpu && opts.check;
				}
				if ( !s_bVisGpu )
					BwOptix_Shutdown();
			}
			if ( !s_bVisGpu && !s_OccOfs.empty() && s_OccOfs.size() > 1 )
				Warning( "bspwalk: no GPU visibility path (CUDA device, OptiX or bspwalk_arcs.ptx missing); tier B face rays on the CPU\n" );
		}
		if ( opts.check && !s_bVisGpu )
			Warning( "bspwalk: -vischeck needs the GPU path; tier B is traced on the CPU only\n" );
		st.path = s_bVisSim ? "gpusim" : s_bVisGpu ? "gpu" : "cpu";
		memset( s_CheckSeg, 0, sizeof( s_CheckSeg ) );
		memset( s_CheckTiles, 0, sizeof( s_CheckTiles ) );
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
				s_Visible[f] = 1;
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
		s_SolidEyes.clear();
		s_bBarePvs = out.pvs;
		st.noEyeClusters = (int)eyes.bareClusters.size();
		// with or without vis data: a viewpoint in solid keeps faces visible
		for ( size_t i = 0; i < eyes.samples.size(); i++ )
		{
			const EyeSample &e = eyes.samples[i];
			if ( e.cluster < 0 && !e.mapped && IsViewpointSource( e.source ) )
				s_SolidEyes.push_back( (int)i );
		}
		st.solidEyes = (int)s_SolidEyes.size();
		if ( !out.pvs )
		{
			// one bucket holds every eye
			s_EyeClOfs.assign( 2, 0 );
			s_EyeByCl.clear();
			for ( size_t i = 0; i < eyes.samples.size(); i++ )
				s_EyeByCl.push_back( (int)i );
			s_EyeClOfs[1] = (int)s_EyeByCl.size();
		}
		st.tOccluders = Plat_FloatTime() - to;

		// fan: one eye per fanSpacing cell
		double tf = Plat_FloatTime();
		s_FanEyes.clear();
		{
			std::unordered_set<long long> cells;
			double minScale = 1e300;
			for ( size_t k = 0; k < eyes.skyScales.size(); k++ )
				minScale = MIN( minScale, eyes.skyScales[k] );
			for ( size_t i = 0; i < eyes.samples.size(); i++ )
			{
				const EyeSample &e = eyes.samples[i];
				if ( e.cluster < 0 && out.pvs )
					continue;
				// a skybox copy is 1/scale the size: its fan cells too (the mapped
				// cells get their own key space)
				double fs = e.mapped ? opts.fanSpacing / minScale : opts.fanSpacing;
				long long x = (long long)floor( e.x / fs ) + ( 1 << 20 ), y = (long long)floor( e.y / fs ) + ( 1 << 20 ),
						  z = (long long)floor( e.z / fs ) + ( 1 << 20 );
				// mapped eyes are tiny: their cells are per source scale apart anyway
				if ( cells.insert( ( ( x << 42 ) | ( y << 21 ) | z ) ^ ( e.mapped ? 0x4000000000000000LL : 0 ) ).second )
					s_FanEyes.push_back( (int)i );
			}
		}
		memset( s_RayCount, 0, sizeof( s_RayCount ) );
		RunThreadsBatched( (int)s_FanEyes.size(), FanOne );
		st.tFan = Plat_FloatTime() - tf;

		// mirrors: reflective faces still visible, and the eyes in front of each
		// that can see it
		s_Reflective.clear();
		s_MirrorEyes.clear();
		s_MirrorExtra.clear();
		s_MirrorBare.clear();
		for ( int f = 0; f < nf; f++ )
		{
			if ( !s_Faces[f].reflective || out.tier[f] == TIER_A || s_Faces[f].disp || s_Faces[f].w.size() < 3 )
				continue;
			s_Reflective.push_back( f );
			s_MirrorEyes.push_back( std::vector<dvec3>() );
			s_MirrorExtra.push_back( std::vector<dvec3>() );
			// a reflection is seen only from in front of the mirror
			s_MirrorBare.push_back( BareMaySee( s_Faces[f], f, true ) );
			std::vector<int> cand[2];
			// as a two-sided face the culling is off: keep the eyes in front by hand
			const VisFace &R = s_Faces[f];
			if ( CandidateEyes( f, cand[0], cand[1] ) )
				s_MirrorBare.back() = 1;	// untried viewpoints in solid
			for ( size_t k = 0; k < cand[1].size(); k++ )
			{
				const EyeSample &e = eyes.samples[cand[1][k]];
				if ( DDot( R.n, dvec3( e.x, e.y, e.z ) ) > R.d + 1e-6 )
					s_MirrorBare.back() = 1;	// a viewpoint in solid in front of it (see FacePassOne)
			}
			for ( int x = 0; x < 2; x++ )
			{
				for ( size_t k = 0; k < cand[x].size(); k++ )
				{
					const EyeSample &e = eyes.samples[cand[x][k]];
					dvec3 p( e.x, e.y, e.z );
					if ( DDot( R.n, p ) > R.d + 1e-6 )
						( x ? s_MirrorExtra : s_MirrorEyes ).back().push_back( p );
				}
			}
		}

		double tp = Plat_FloatTime();
		s_PassFaces.clear();
		for ( int f = 0; f < nf; f++ )
		{
			if ( !s_Visible[f] )
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
			st.fanRays += s_RayCount[t][0];
			st.faceRays += s_RayCount[t][1];
			st.mirrorRays += s_RayCount[t][2];
			st.checkSegments += s_CheckSeg[t][0];
			st.gpuOnlyBlocked += s_CheckSeg[t][1];
			st.cpuOnlyBlocked += s_CheckSeg[t][2];
			st.checkTileDiffer += s_CheckTiles[t];
		}
		if ( s_bVisGpu && !s_bVisSim )
			BwOptix_Shutdown();
		if ( s_bVisCheck )
			Msg( "vis: -vischeck: %lld segments on both paths, %lld blocked only on the GPU, %lld only on the CPU; "
				 "%d tile levels decided differently (the CPU's answer kept)\n",
				 st.checkSegments, st.gpuOnlyBlocked, st.cpuOnlyBlocked, st.checkTileDiffer );
		s_bVisGpu = s_bVisCheck = s_bVisSim = false;
		for ( int f = 0; f < nf; f++ )
		{
			if ( out.tier[f] != TIER_VISIBLE || s_Faces[f].model || s_Faces[f].bad )
				continue;
			int v = s_Visible[f];
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
		}
		for ( int t = 0; t <= MAX_TOOL_THREADS; t++ )
		{
			s_FaceThreads[t] = FaceThread();
		}
	}
	else
	{
		for ( int f = 0; f < nf; f++ )
		{
			if ( out.tier[f] == TIER_VISIBLE && !s_Faces[f].model && !s_Faces[f].bad )
				out.why[f] = WHY_NOTIERB;
		}
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
	if ( opts.tierB )
	{
		Msg( "vis: face and mirror passes on the %s%s\n", st.path, !strcmp( st.path, "gpu" ) ? " (OptiX)" : " (reference)" );
		Msg( "vis: %d occluders (%d see-through solids left out); fan %d eyes x %d rays -> %d faces; face rays %lld -> %d, "
			 "mirror rays %lld -> %d; %d reflective clusters added to tier A\n",
			 st.occluders, st.occludersSkipped, (int)s_FanEyes.size(), opts.fanRays, st.visibleFan, st.faceRays,
			 st.visibleFace, st.mirrorRays, st.visibleMirror, st.reflectClusters );
		Msg( "vis: %d faces kept visible with no ray test (%d eye clusters hold no eye sample; %d samples of "
			 "viewpoints in solid keep the faces in front of them visible)\n", st.visibleNoEye, st.noEyeClusters, st.solidEyes );
		if ( st.facesWidened )
			Warning( "bspwalk: %d faces were sampled coarser than -facespacing %gu (up to %gu; the per-face sample cap)\n",
					 st.facesWidened, opts.faceSpacing, st.faceSpacingMax );
	}
	Msg( "vis timings (s): tier A %.3f, occluders %.3f, fan %.3f, face pass %.3f, total %.3f\n", st.tTierA, st.tOccluders,
		 st.tFan, st.tFace, st.tTotal );

	// release
	std::vector<VisFace>().swap( s_Faces );
	std::vector<unsigned char>().swap( s_Pvs );
	std::vector<DPlane>().swap( s_OccPlanes );
	s_OccBvh = Bvh();
	s_FaceBvh = Bvh();
	(void)profile;
	(void)graph;
}
