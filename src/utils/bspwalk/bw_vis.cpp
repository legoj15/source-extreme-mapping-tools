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
static std::vector<unsigned char> s_Tier;
static int s_nClusters, s_RowBytes;
static std::vector<unsigned char> s_Pvs;			// numClusters rows
static const EyeVolume *s_pEyes;
static const VisOptions *s_pVisOpts;
static std::vector<int> s_EyeClOfs, s_EyeByCl;		// cluster -> eye samples (CSR)
static std::vector<int> s_FanEyes;
static std::vector<int> s_Reflective;				// reflective faces that stay visible to tier A
static std::vector< std::vector<dvec3> > s_MirrorEyes;	// per s_Reflective: mirrored eyes in front of it
static std::vector<int> s_PassFaces;
static long long s_RayCount[MAX_TOOL_THREADS + 1][3];

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
		if ( listed[f] || ( s_Faces[f].model && !s_Faces[f].reflective ) )
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
	ConvexShape shape;
	for ( size_t i = 0; i < brushes.size(); i++ )
	{
		int b = brushes[i];
		if ( !( dbrushes[b].contents & CONTENTS_SOLID ) )
			continue;
		if ( !OpaqueSolid( b ) )
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
		if ( !OpaqueSolid( sc.sourceIndex ) )
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
// two-sided or a displacement.
static void CandidateEyes( int fi, std::vector<int> &out )
{
	out.clear();
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
	std::vector<FaceSample> samples;
	std::vector<int> cand;
	std::vector<dvec3> pts, via;
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

// Tests the samples tile by tile against the binned points; true at the first
// clear ray. A tile that finds none tries again with twice the bins over the
// eyes it has not tried, up to binLevels times. mirror >= 0: points are
// mirrored eyes behind s_Reflective[mirror].
static bool TraceTiles( int iThread, FaceThread &t, int mirror )
{
	int bins0 = s_pVisOpts->bins;
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
		for ( int level = 0; level < s_pVisOpts->binLevels && unused > 0; level++ )
		{
			BinEyes( c, t.pts, mirror >= 0 ? &t.via : NULL, bins0 << level, t.used, unused, t.tileEyes );
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

static void FacePassOne( int iThread, int item )
{
	int fi = s_PassFaces[item];
	if ( s_Visible[fi] )
		return;
	FaceThread &t = s_FaceThreads[iThread];
	t.twoSided = s_Faces[fi].twoSided;
	SampleFace( s_Faces[fi], t.samples );
	if ( t.samples.empty() )
		return;
	t.tiles.clear();
	for ( size_t i = 0; i < t.samples.size(); i++ )
		t.tiles.push_back( std::make_pair( TileKey( t.samples[i].p ), (int)i ) );
	std::sort( t.tiles.begin(), t.tiles.end() );

	CandidateEyes( fi, t.cand );
	t.pts.clear();
	for ( size_t i = 0; i < t.cand.size(); i++ )
	{
		const EyeSample &e = s_pEyes->samples[t.cand[i]];
		t.pts.push_back( dvec3( e.x, e.y, e.z ) );
	}
	if ( !t.pts.empty() && TraceTiles( iThread, t, -1 ) )
	{
		s_Visible[fi] = WHY_RAY + 1;
		return;
	}

	// through each reflective face that stays visible
	const VisFace &f = s_Faces[fi];
	for ( size_t r = 0; r < s_Reflective.size(); r++ )
	{
		if ( s_Reflective[r] == fi || s_MirrorEyes[r].empty() )
			continue;
		const VisFace &R = s_Faces[s_Reflective[r]];
		// some part of the face must lie in front of the mirror
		bool front = false;
		const std::vector<dvec3> &pts = f.disp ? f.tris : f.w;
		for ( size_t k = 0; k < pts.size() && !front; k++ )
			front = DDot( R.n, pts[k] ) > R.d + 1e-6;
		if ( !front )
			continue;
		t.pts.clear();
		t.via.clear();
		const std::vector<dvec3> &me = s_MirrorEyes[r];
		bool cull = !f.twoSided && !f.disp;
		for ( size_t k = 0; k < me.size(); k++ )
		{
			dvec3 m = MirrorPoint( me[k], R.n, R.d );
			if ( cull && DDot( f.n, m ) <= f.d + 1e-6 )
				continue;
			t.pts.push_back( m );
			t.via.push_back( me[k] );
		}
		if ( !t.pts.empty() && TraceTiles( iThread, t, (int)r ) )
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
			double fs = opts.fanSpacing;
			for ( size_t i = 0; i < eyes.samples.size(); i++ )
			{
				const EyeSample &e = eyes.samples[i];
				if ( e.cluster < 0 && out.pvs )
					continue;
				long long x = (long long)floor( e.x / fs ) + ( 1 << 20 ), y = (long long)floor( e.y / fs ) + ( 1 << 20 ),
						  z = (long long)floor( e.z / fs ) + ( 1 << 20 );
				// mapped eyes are tiny: their cells are per source scale apart anyway
				if ( cells.insert( ( x << 42 ) | ( y << 21 ) | z ).second )
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
		for ( int f = 0; f < nf; f++ )
		{
			if ( !s_Faces[f].reflective || out.tier[f] == TIER_A || s_Faces[f].disp || s_Faces[f].w.size() < 3 )
				continue;
			s_Reflective.push_back( f );
			s_MirrorEyes.push_back( std::vector<dvec3>() );
			std::vector<int> cand;
			// as a two-sided face the culling is off: keep the eyes in front by hand
			CandidateEyes( f, cand );
			const VisFace &R = s_Faces[f];
			for ( size_t k = 0; k < cand.size(); k++ )
			{
				const EyeSample &e = eyes.samples[cand[k]];
				dvec3 p( e.x, e.y, e.z );
				if ( DDot( R.n, p ) > R.d + 1e-6 )
					s_MirrorEyes.back().push_back( p );
			}
		}

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
		}
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
		Msg( "vis: %d occluders (%d see-through solids left out); fan %d eyes x %d rays -> %d faces; face rays %lld -> %d, "
			 "mirror rays %lld -> %d; %d reflective clusters added to tier A\n",
			 st.occluders, st.occludersSkipped, (int)s_FanEyes.size(), opts.fanRays, st.visibleFan, st.faceRays,
			 st.visibleFace, st.mirrorRays, st.visibleMirror, st.reflectClusters );
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
