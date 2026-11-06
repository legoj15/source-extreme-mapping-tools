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
	static const char *s_Names[WHY_COUNT] = { "pvs", "rays", "fan", "ray", "mirror", "brush_entity", "no_tierb",
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
};

static std::vector<VisFace> s_Faces;
static std::vector<int> s_FaceClOfs, s_FaceCl;		// face -> clusters (CSR, -1 excluded)
static std::vector<DPlane> s_OccPlanes;
static std::vector<int> s_OccOfs;
static Bvh s_OccBvh, s_FaceBvh;
static std::vector<unsigned char> s_Visible;		// per face, set by the ray passes
static int s_nClusters, s_RowBytes;
static std::vector<unsigned char> s_Pvs;			// numClusters rows
static const EyeVolume *s_pEyes;
static const VisOptions *s_pVisOpts;
static std::vector<int> s_EyeClOfs, s_EyeByCl;		// cluster -> eye samples (CSR)
static const double NOVIS_BUCKET = 512.0;			// eye bucket cube size without vis data
static std::vector<float> s_EyeClBox;				// per s_EyeClOfs bucket: its eyes' mins xyz, maxs xyz
static bool s_bBarePvs;				// vis data present: a bare cluster sees only its PVS (else it may see every face)
// unmapped samples of viewpoint sources whose point is in a solid leaf (cluster
// -1, so no PVS row): a camera there draws every cluster, so they are candidate
// eyes of every face
static std::vector<int> s_SolidEyes;
static const int FACE_MAX_EXTRA = 1 << 12;	// at most this many of them per face (CandidateEyes)
// An eye region box counts as in front of a face only past this: the eye
// descent hands each side of a node plane the piece parts up to 1e-4 beyond it
// (bw_eyes EYE_SPLIT_EPS), so boxes reach that far past faces on node planes.
static const double REGION_MARGIN = 1e-3;
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
		for ( int k = 0; k < df.numedges; k++ )
		{
			int se = dsurfedges[df.firstedge + k];
			const dvertex_t &v = dvertexes[se < 0 ? dedges[-se].v[1] : dedges[se].v[0]];
			f.w.push_back( dvec3( v.point.x, v.point.y, v.point.z ) + org );
		}
		f.d += DDot( f.n, org );
		f.disp = df.dispinfo >= 0 && df.dispinfo < (int)dispTris.size() && !dispTris[df.dispinfo].empty();
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

static void SampleFace( const VisFace &f, std::vector<FaceSample> &out )
{
	out.clear();
	double sp = s_pVisOpts->faceSpacing;
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
// CPU fan of eye s_FanEyes[s_FanFirst + item]. Under -vischeck with the GPU
// fan, s_FanGpuT holds the GPU's occluder distance of each ray of the batch.
static const float *s_FanGpuT;
static int s_FanFirst;
static long long s_FanCheck[MAX_TOOL_THREADS + 1][2];	// rays compared, rays the GPU stopped short on
static double s_FanShort[MAX_TOOL_THREADS + 1];			// the largest such shortfall

static void FanOne( int iThread, int item )
{
	const EyeSample &e = s_pEyes->samples[s_FanEyes[s_FanFirst + item]];
	dvec3 o( e.x, e.y, e.z );
	int n = s_pVisOpts->fanRays;
	const float *gpuT = s_FanGpuT ? s_FanGpuT + (size_t)item * n : NULL;
	for ( int i = 0; i < n; i++ )
	{
		dvec3 dir = FibonacciDir( i, n );
		double t = NearestOccluder( o, dir );
		if ( gpuT )
		{
			// a ray the GPU stops short on may miss faces the CPU marks;
			// fan_cpu_only_faces is what tells whether any face was lost
			s_FanCheck[iThread][0]++;
			if ( gpuT[i] < t )
			{
				s_FanCheck[iThread][1]++;
				s_FanShort[iThread] = MAX( s_FanShort[iThread], t - gpuT[i] );
			}
		}
		MarkFacesAlong( o, dir, t + FAN_PAST );
	}
	s_RayCount[iThread][0] += n;
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

// The occluders as a triangle mesh, each convex rebuilt from its planes moved
// in by 'shrink'.
static void OccluderMesh( double shrink, std::vector<float> &mv, std::vector<unsigned int> &mt )
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
	std::vector<dvec3> pts;
	for ( size_t g = 0; g + 2 < f.tris.size(); g += 3 )
	{
		pts.clear();
		for ( size_t k = 0; k < 3; k++ )
		{
			s_SlabMins[item] = DMin( s_SlabMins[item], f.tris[g + k] );
			s_SlabMaxs[item] = DMax( s_SlabMaxs[item], f.tris[g + k] );
			for ( int c = 0; c < 8; c++ )
			{
				pts.push_back( f.tris[g + k] + dvec3( c & 1 ? FAN_GPU_PAD : -FAN_GPU_PAD, c & 2 ? FAN_GPU_PAD : -FAN_GPU_PAD,
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
// faces it marks.
static Bvh s_SlabBvh;
static std::vector<unsigned char> s_FanInSlab;

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
			if ( s_FanInSlab[fi] )
				continue;
			const VisFace &f = s_Faces[fi];
			bool in = false;
			if ( f.disp )
			{
				for ( size_t k = 0; k + 2 < f.tris.size() && !in; k += 3 )
					in = NearTriangle( p, f.tris[k], f.tris[k + 1], f.tris[k + 2], m );
			}
			else
				in = fabs( DDot( f.n, p ) - f.d ) <= m && InsidePolygon( f, p, 1e-3 + m );
			if ( in )
				s_FanInSlab[fi] = 1;	// a racing store writes the same byte
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

	std::vector<unsigned char> init( s_Visible );
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
		s_FanInSlab.assign( nf, 0 );
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
		else if ( s_FanInSlab[s_SlabFaces[i]] )
		{
			markNoRay[s_SlabFaces[i]] = 1;
			inSlab++;
		}
	}
	std::vector<unsigned char>().swap( s_FanInSlab );
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
			s_RayCount[0][0] += (long long)count * n;	// with -vischeck the CPU fan counts them
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
		// the CPU fan runs instead: forget the GPU's ray count
		memset( s_RayCount, 0, sizeof( s_RayCount ) );
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
			bool c = s_Visible[f] != 0;
			st.fanGpuOnlyFaces += g && !c;
			st.fanCpuOnlyFaces += c && !g;
		}
		else if ( g )
			s_Visible[f] = WHY_FAN + 1;
	}
	if ( check )
	{
		// the CPU fan counted its rays too
		for ( int t = 0; t <= MAX_TOOL_THREADS; t++ )
		{
			st.fanCheckRays += s_FanCheck[t][0];
			st.fanGpuShort += s_FanCheck[t][1];
			st.fanMaxShort = MAX( st.fanMaxShort, s_FanShort[t] );
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

// The eyes that may see a face: those in clusters whose PVS holds one of its
// clusters (all eyes without vis data), in front of its plane unless it is
// two-sided or a displacement ('front': always; a mirror is seen only from in
// front). The lattice samples go to 'out' (thinned out past
// FACE_MAX_CANDIDATES); the samples of viewpoints in solid go to 'extra'. The
// callers keep the face visible when 'extra' is non-empty (such an eye sits
// inside the brush that would block its rays). Returns true when no ray test
// can settle the face, so the caller keeps it visible: 'extra' overflowed
// FACE_MAX_EXTRA (it is then cleared), or an eye cluster that may see the face
// has no sample in front of it while its eye region (the box of its pieces'
// parts) reaches in front (the lattice and the piece vertices can all lie
// behind a plane the pieces cross by up to a spacing).
static inline bool EyeInFront( const VisFace &f, const EyeSample &e )
{
	return f.n.x * e.x + f.n.y * e.y + f.n.z * e.z > f.d + 1e-6;
}

// Which eyes of bucket c pass the back-face test: 0 none, 1 all, 2 test each.
// The bucket's eye box decides most buckets without touching their eyes. The
// per-eye test is n . e > d + 1e-6. The box holds the exact min and max of the
// same float eye coordinates, and its dot products are computed in double in
// the same term order, so its extremes are the extreme eyes' own per-eye
// values: the answer is the per-eye test's exactly. The extra 1e-6 either side
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

static bool CandidateEyes( int fi, std::vector<int> &out, std::vector<int> &extra, bool front = false )
{
	out.clear();
	extra.clear();
	const VisFace &f = s_Faces[fi];
	const std::vector<EyeSample> &eyes = s_pEyes->samples;
	int nCl = (int)s_EyeClOfs.size() - 1;
	bool cull = front || ( !f.twoSided && !f.disp );
	bool untestable = false;
	// Two passes over the eye buckets: count the eyes that pass, then take
	// them, or the FACE_MAX_CANDIDATES of them at an even stride, straight
	// into 'out'. Listing every passing eye first and thinning afterwards cost
	// over 2 minutes on fightspace3 (28M eyes, 900 mirrors set up one by one).
	static thread_local std::vector< std::pair<int, int> > buckets;	// bucket, BucketSide
	buckets.clear();
	size_t n = 0;
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
		int side = cull ? BucketSide( f, c ) : 1;
		size_t m = 0;
		if ( side == 1 )
			m = b - a;
		else if ( side == 2 )
		{
			for ( int i = a; i < b; i++ )
				m += EyeInFront( f, eyes[s_EyeByCl[i]] );
		}
		if ( m == 0 )
		{
			// no sample in front: the bucket's pieces may still reach in front
			if ( cull && !untestable && !s_Pvs.empty() )
				untestable = RegionInFront( f, c );
			continue;
		}
		buckets.push_back( std::make_pair( c, side ) );
		n += m;
	}
	if ( cull && n == 0 && s_Pvs.empty() )
		untestable = RegionInFront( f, -1 );	// no eye in front, the pieces may reach it
	// the k-th kept eye is the one at position floor(k n / cap) among those that pass
	const bool thin = n > (size_t)FACE_MAX_CANDIDATES;
	out.reserve( thin ? FACE_MAX_CANDIDATES : n );
	int k = 0;
	size_t next = 0, j = 0;	// next position to keep; position of the bucket's first eye
	for ( size_t q = 0; q < buckets.size() && next < n; q++ )
	{
		int a = s_EyeClOfs[buckets[q].first], b = s_EyeClOfs[buckets[q].first + 1];
		if ( buckets[q].second == 1 )
		{
			size_t end = j + ( b - a );
			if ( !thin )
				out.insert( out.end(), s_EyeByCl.begin() + a, s_EyeByCl.begin() + b );
			else
			{
				while ( next < end )
				{
					out.push_back( s_EyeByCl[a + ( next - j )] );
					k++;
					next = k < FACE_MAX_CANDIDATES ? (size_t)( (double)k * n / FACE_MAX_CANDIDATES ) : n;
				}
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
			else if ( j == next )
			{
				out.push_back( s_EyeByCl[i] );
				k++;
				next = k < FACE_MAX_CANDIDATES ? (size_t)( (double)k * n / FACE_MAX_CANDIDATES ) : n;
			}
			j++;
		}
	}
	// a camera in a solid leaf draws every cluster: its samples may see any face
	for ( size_t i = 0; i < s_SolidEyes.size(); i++ )
	{
		const EyeSample &e = eyes[s_SolidEyes[i]];
		if ( cull && !EyeInFront( f, e ) )
			continue;
		extra.push_back( s_SolidEyes[i] );
	}
	bool overflow = (int)extra.size() > FACE_MAX_EXTRA;
	if ( overflow )
		extra.clear();						// the caller keeps the face visible
	return overflow || untestable;
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
	const std::vector<dvec3> &pts = f.disp ? f.tris : f.w;
	for ( size_t k = 0; k < pts.size(); k++ )
	{
		if ( DDot( R.n, pts[k] ) > R.d + 1e-6 )
			return true;
	}
	return false;
}

// Does face fi lie partly in front of a mirror that a bare eye cluster may see
// (s_MirrorBare)? The mirror loop of FacePassOne keeps such a face visible with
// no ray test whenever the face's own rays all fail, so the face is visible
// whatever they find: deciding that first skips them. On fightspace3 one large
// face spent 148 s in direct rays that all failed before the mirror loop kept it
// visible this way. The face's reason is then no_eye where a clear ray would
// have made it ray; its tier is the same.
static bool InFrontOfBareMirror( int fi )
{
	for ( size_t r = 0; r < s_Reflective.size(); r++ )
	{
		if ( s_MirrorBare[r] && s_Reflective[r] != fi && InFrontOfMirror( s_Faces[fi], s_Faces[s_Reflective[r]] ) )
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
		s_Visible[fi] = WHY_NOEYE + 1;
		return;
	}
	SampleFace( s_Faces[fi], t.samples );
	if ( t.samples.empty() )
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
		if ( !InFrontOfMirror( f, R ) )
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

// Mirror r (s_Reflective[r]): the eyes in front of it that may see it.
static void MirrorSetupOne( int iThread, int r )
{
	int f = s_Reflective[r];
	const VisFace &R = s_Faces[f];
	const std::vector<EyeSample> &samples = s_pEyes->samples;
	// a reflection is seen only from in front of the mirror
	unsigned char bare = BareMaySee( R, f, true );
	FaceThread &t = s_FaceThreads[iThread];
	// only the eyes in front of the mirror (culled before the thinning)
	if ( CandidateEyes( f, t.cand, t.xcand, true ) )
		bare = 1;	// untried viewpoints in solid, or an untestable eye region
	for ( size_t k = 0; k < t.xcand.size(); k++ )
	{
		const EyeSample &e = samples[t.xcand[k]];
		if ( DDot( R.n, dvec3( e.x, e.y, e.z ) ) > R.d + 1e-6 )
			bare = 1;	// a viewpoint in solid in front of it (see FacePassOne)
	}
	s_MirrorBare[r] = bare;
	for ( int x = 0; x < 2; x++ )
	{
		const std::vector<int> &cand = x ? t.xcand : t.cand;
		std::vector<dvec3> &me = x ? s_MirrorExtra[r] : s_MirrorEyes[r];
		for ( size_t k = 0; k < cand.size(); k++ )
		{
			const EyeSample &e = samples[cand[k]];
			dvec3 p( e.x, e.y, e.z );
			if ( DDot( R.n, p ) > R.d + 1e-6 )
				me.push_back( p );
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
			if ( s_Faces[f].model )
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
		if ( s_Faces[f].model )
		{
			out.tier[f] = TIER_VISIBLE;
			out.why[f] = WHY_ENTITY;
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
				OccluderMesh( 0.0, mv, mt );
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
				Warning( "bspwalk: no GPU visibility path (CUDA device, OptiX or bspwalk_arcs.ptx missing); the tier B fan and face rays run on the CPU\n" );
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
			if ( out.tier[f] != TIER_VISIBLE || s_Faces[f].model )
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
			if ( out.tier[f] != TIER_VISIBLE || s_Faces[f].model )
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
		memset( s_FanCheck, 0, sizeof( s_FanCheck ) );
		memset( s_FanShort, 0, sizeof( s_FanShort ) );
		s_FanGpuT = NULL;
		s_FanFirst = 0;
		// the GPU fan rides on the face pass's OptiX setup (not under -visgpusim)
		bool fanGpu = s_bVisGpu && !s_bVisSim && !s_FanEyes.empty() && opts.fanRays > 0;
		if ( fanGpu && !FanGpu( st, s_bVisCheck ) )
		{
			Warning( "bspwalk: the GPU fan failed (the error is above); the fan runs on the CPU\n" );
			fanGpu = false;
		}
		if ( !fanGpu )
			RunThreadsBatched( (int)s_FanEyes.size(), FanOne );
		st.fanPath = fanGpu ? "gpu" : "cpu";
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
		s_MirrorExtra.assign( s_Reflective.size(), std::vector<dvec3>() );
		s_MirrorBare.assign( s_Reflective.size(), 0 );
		RunThreadsBatched( (int)s_Reflective.size(), MirrorSetupOne );
		st.tMirror = Plat_FloatTime() - tm;

		double tp = Plat_FloatTime();
		s_PassFaces.clear();
		for ( int f = 0; f < nf; f++ )
		{
			if ( !s_Visible[f] )
				s_PassFaces.push_back( f );
		}
		RunThreadsBatched( (int)s_PassFaces.size(), FacePassOne );
		st.tFace = Plat_FloatTime() - tp;

		for ( int t = 0; t <= MAX_TOOL_THREADS; t++ )
		{
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
		if ( s_bVisCheck && !strcmp( st.fanPath, "gpu" ) )
			Msg( "vis: -vischeck fan: %lld rays on both paths, %lld stopped short only on the GPU (max %.4fu); faces marked "
				 "%d only on the GPU, %d only on the CPU (the CPU's marks kept)\n",
				 st.fanCheckRays, st.fanGpuShort, st.fanMaxShort, st.fanGpuOnlyFaces, st.fanCpuOnlyFaces );
		s_bVisGpu = s_bVisCheck = s_bVisSim = false;
		for ( int f = 0; f < nf; f++ )
		{
			if ( out.tier[f] != TIER_VISIBLE || s_Faces[f].model )
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
			if ( out.tier[f] == TIER_VISIBLE && !s_Faces[f].model )
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
		Msg( "vis: fan on the %s%s, face and mirror passes on the %s%s\n", st.fanPath,
			 !strcmp( st.fanPath, "gpu" ) ? " (OptiX)" : " (reference)", st.path,
			 !strcmp( st.path, "gpu" ) ? " (OptiX)" : " (reference)" );
		Msg( "vis: %d occluders (%d see-through solids left out); fan %d eyes x %d rays -> %d faces; face rays %lld -> %d, "
			 "mirror rays %lld -> %d; %d reflective clusters added to tier A\n",
			 st.occluders, st.occludersSkipped, (int)s_FanEyes.size(), opts.fanRays, st.visibleFan, st.faceRays,
			 st.visibleFace, st.mirrorRays, st.visibleMirror, st.reflectClusters );
		Msg( "vis: %d faces kept visible with no ray test (%d eye clusters hold no eye sample; %d samples of "
			 "viewpoints in solid keep the faces in front of them visible)\n", st.visibleNoEye, st.noEyeClusters, st.solidEyes );
	}
	Msg( "vis timings (s): tier A %.3f, occluders %.3f, fan %.3f (GPU setup %.3f), mirror setup %.3f, face pass %.3f, "
		 "total %.3f\n", st.tTierA, st.tOccluders, st.tFan, st.tFanSetup, st.tMirror, st.tFace, st.tTotal );

	// release
	std::vector<VisFace>().swap( s_Faces );
	std::vector<unsigned char>().swap( s_Pvs );
	std::vector<DPlane>().swap( s_OccPlanes );
	s_OccBvh = Bvh();
	s_FaceBvh = Bvh();
	(void)profile;
	(void)graph;
}
