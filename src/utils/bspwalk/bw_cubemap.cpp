//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: which env_cubemap samples may be viewpoint eyes (#82). See
//          bw_cubemap.h for the three rules (in a brush, sees toolsnodraw, in a
//          static prop) and why.
//
//=============================================================================//

#include "cmdlib.h"
#include "bsplib.h"
#include "bspflags.h"
#include "threads.h"
#include "tier0/platform.h"
#include "bw_cubemap.h"
#include "bw_vis.h"
#include "bw_props.h"
#include "bw_hash.h"
#include "bw_bvh.h"
#include "bw_util.h"

#include <algorithm>
#include <string.h>

// The near cube overlaps a convex only when it runs this far into it on every axis (touching is free).
static const double CM_OVERLAP = 1e-3;
// A nodraw side is buried when the point this far out along its normal is in a leaf with no cluster.
static const double CM_BURY = 0.125;
// Entry parameters within this tie; the side that draws wins.
static const double CM_TIE = 1e-4;
// A ray enters a brush only when it runs at least this far inside (bw_vis.cpp OCC_DEPTH).
static const double CM_DEPTH = 1e-6;
static const double CM_RAY_MAX = 131072.0;

const char *CubemapStatusName( int status )
{
	return status == CUBEMAP_OK ? "ok" : status == CUBEMAP_SOLID ? "solid" : status == CUBEMAP_NODRAW ? "nodraw" : "?";
}

//-----------------------------------------------------------------------------
// Convex tests
//-----------------------------------------------------------------------------
// Penetration of the cube [c - h, c + h] and the convex along 'axis' (any length): the overlap of their
// projections, in units. Negative or zero: separated or touching.
static double AxisPenetration( const dvec3 &c, double h, const ConvexShape &s, dvec3 axis )
{
	double len = DNormalize( axis );
	if ( len < 1e-9 )
		return 1e300;		// a degenerate axis (parallel edges) separates nothing
	double lo = 1e300, hi = -1e300;
	for ( size_t i = 0; i < s.verts.size(); i++ )
	{
		double p = DDot( axis, s.verts[i] );
		lo = MIN( lo, p );
		hi = MAX( hi, p );
	}
	double mid = DDot( axis, c ), r = h * ( fabs( axis.x ) + fabs( axis.y ) + fabs( axis.z ) );
	return MIN( mid + r, hi ) - MAX( mid - r, lo );
}

// Is the point c inside the convex by more than CM_OVERLAP?
static bool PointInsideConvex( const dvec3 &c, const ConvexShape &s )
{
	if ( s.faces.empty() )
		return false;
	for ( size_t f = 0; f < s.faces.size(); f++ )
	{
		double d = DDot( s.faceNormals[f], s.verts[s.faces[f][0]] );
		if ( d - DDot( s.faceNormals[f], c ) <= CM_OVERLAP )
			return false;
	}
	return true;
}

// Exact: does the cube [c - h, c + h] overlap the interior of the convex? Separating axes of a box and a
// convex polyhedron: the three world axes, the face normals, and the cross products of a world axis with
// an edge. They overlap only if every axis has more than CM_OVERLAP penetration. h == 0: the point test.
static bool CubeOverlapsConvex( const dvec3 &c, double h, const ConvexShape &s )
{
	if ( s.verts.empty() || s.faces.empty() )
		return false;
	if ( h <= 0 )
		return PointInsideConvex( c, s );
	static const dvec3 e[3] = { dvec3( 1, 0, 0 ), dvec3( 0, 1, 0 ), dvec3( 0, 0, 1 ) };
	for ( int a = 0; a < 3; a++ )
	{
		if ( AxisPenetration( c, h, s, e[a] ) <= CM_OVERLAP )
			return false;
	}
	for ( size_t f = 0; f < s.faceNormals.size(); f++ )
	{
		if ( AxisPenetration( c, h, s, s.faceNormals[f] ) <= CM_OVERLAP )
			return false;
	}
	for ( size_t k = 0; k < s.edges.size(); k++ )
	{
		dvec3 d = s.verts[s.edges[k].second] - s.verts[s.edges[k].first];
		for ( int a = 0; a < 3; a++ )
		{
			if ( AxisPenetration( c, h, s, DCross( e[a], d ) ) <= CM_OVERLAP )
				return false;
		}
	}
	return true;
}

//-----------------------------------------------------------------------------
// World brushes: every model 0 brush with CONTENTS_SOLID, as planes (un-shrunk,
// each with its dbrushside_t index) in a BVH over their boxes
//-----------------------------------------------------------------------------
struct CmBrush
{
	int brush;					// dbrush index
	bool built;
	bool opaque;				// OpaqueSolid: it can block a ray
	dvec3 mins, maxs;
	std::vector<DPlane> planes;	// n . x <= d, the non-bevel sides
	std::vector<int> sides;		// dbrushsides index per plane
	std::vector<char> nodraw;	// per plane: the side's material is toolsnodraw
};

static std::vector<CmBrush> s_Brushes;		// the candidates; the BVH indexes the built ones (s_BvhBrush)
static std::vector<int> s_BvhBrush;			// BVH item -> s_Brushes
static Bvh s_Bvh;
static std::vector<char> s_NodrawTexinfo;	// per texinfo: the material is TOOLS/TOOLSNODRAW
static std::vector<StaticPropInstance> s_Props;
static const CubemapOptions *s_pOpts;
static CubemapReport *s_pReport;

static bool EndsWithNoCase( const char *s, const char *tail )
{
	size_t a = strlen( s ), b = strlen( tail );
	return a >= b && !Q_stricmp( s + a - b, tail );
}

static void BuildBrushOne( int iThread, int item )
{
	CmBrush &cb = s_Brushes[item];
	const dbrush_t &br = dbrushes[cb.brush];
	ConvexShape shape;
	cb.built = BuildBrushShape( cb.brush, shape );
	if ( !cb.built )
		return;
	cb.opaque = OpaqueSolid( cb.brush );
	cb.mins = shape.mins;
	cb.maxs = shape.maxs;
	for ( int k = 0; k < br.numsides; k++ )
	{
		const dbrushside_t &s = dbrushsides[br.firstside + k];
		if ( BrushSideIsBevel( s.bevel ) )
			continue;
		const dplane_t &pl = dplanes[s.planenum];
		DPlane p;
		p.n = dvec3( pl.normal );
		p.d = pl.dist;
		cb.planes.push_back( p );
		cb.sides.push_back( br.firstside + k );
		cb.nodraw.push_back( s.texinfo >= 0 && s.texinfo < (int)s_NodrawTexinfo.size() && s_NodrawTexinfo[s.texinfo] );
	}
}

static void BuildBrushes()
{
	std::vector<int> all;
	CollectModelBrushes( 0, all );
	s_Brushes.clear();
	for ( size_t i = 0; i < all.size(); i++ )
	{
		if ( !( dbrushes[all[i]].contents & CONTENTS_SOLID ) )
			continue;
		CmBrush cb;
		cb.brush = all[i];
		cb.built = cb.opaque = false;
		s_Brushes.push_back( cb );
	}
	s_NodrawTexinfo.assign( texinfo.Count(), 0 );
	for ( int t = 0; t < texinfo.Count(); t++ )
	{
		int td = texinfo[t].texdata;
		if ( td >= 0 && td < numtexdata )
			s_NodrawTexinfo[t] = EndsWithNoCase( TexInfo_TexName( t ), "toolsnodraw" );
	}
	RunThreadsBatched( (int)s_Brushes.size(), BuildBrushOne );

	std::vector<dvec3> mins, maxs;
	s_BvhBrush.clear();
	for ( size_t i = 0; i < s_Brushes.size(); i++ )
	{
		if ( !s_Brushes[i].built )
		{
			s_pReport->brushesUnbuilt++;
			continue;
		}
		s_BvhBrush.push_back( (int)i );
		mins.push_back( s_Brushes[i].mins );
		maxs.push_back( s_Brushes[i].maxs );
	}
	s_Bvh.Build( mins, maxs );
}

// Brushes (s_Brushes indices) whose box meets [lo, hi].
static void BrushesInBox( const dvec3 &lo, const dvec3 &hi, std::vector<int> &out )
{
	out.clear();
	if ( s_Bvh.nodes.empty() )
		return;
	int stack[128];
	int sp = 0;
	stack[sp++] = 0;
	while ( sp )
	{
		const BvhNode &nd = s_Bvh.nodes[stack[--sp]];
		if ( nd.mins.x > hi.x || nd.maxs.x < lo.x || nd.mins.y > hi.y || nd.maxs.y < lo.y || nd.mins.z > hi.z ||
			 nd.maxs.z < lo.z )
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
			out.push_back( s_BvhBrush[s_Bvh.items[i]] );
	}
}

//-----------------------------------------------------------------------------
// Rays
//-----------------------------------------------------------------------------
// The entry of the ray o + t dir (t in [0, tMax]) into the brush: its parameter and the side that bounds it
// there. A ray through an edge enters by two sides at once (parameters within CM_TIE): the one that draws
// is taken. False when the ray misses the brush, only touches it, or starts inside (no entry plane).
static bool RayEnterBrush( const dvec3 &o, const dvec3 &dir, double tMax, const CmBrush &cb, double &tEnter, int &plane )
{
	const std::vector<DPlane> &planes = cb.planes;
	double t0 = 0.0, t1 = tMax;
	int enter = -1;
	for ( size_t k = 0; k < planes.size(); k++ )
	{
		double denom = DDot( planes[k].n, dir );
		double num = planes[k].d - DDot( planes[k].n, o );
		if ( fabs( denom ) < 1e-300 )
		{
			if ( num <= 0 )
				return false;	// outside, or running along the face: touching is free
			continue;
		}
		double t = num / denom;
		if ( denom < 0 )
		{
			if ( t > t0 )
			{
				t0 = t;
				enter = (int)k;
			}
		}
		else
			t1 = MIN( t1, t );
		if ( t0 >= t1 )
			return false;
	}
	if ( enter < 0 || !( t1 - t0 > CM_DEPTH ) )
		return false;
	if ( cb.nodraw[enter] )
	{
		for ( size_t k = 0; k < planes.size(); k++ )
		{
			double denom = DDot( planes[k].n, dir );
			if ( denom >= 0 || cb.nodraw[k] || fabs( denom ) < 1e-300 )
				continue;
			if ( ( planes[k].d - DDot( planes[k].n, o ) ) / denom >= t0 - CM_TIE )
			{
				enter = (int)k;
				break;
			}
		}
	}
	tEnter = t0;
	plane = enter;
	return true;
}

// Where the ray first meets the box (within [0, t1]), or -1.
static inline double BoxEntryT( const dvec3 &o, const dvec3 &inv, double t1, const dvec3 &mins, const dvec3 &maxs )
{
	double t0 = 0.0;
	for ( int a = 0; a < 3; a++ )
	{
		double ta = ( mins[a] - o[a] ) * inv[a], tb = ( maxs[a] - o[a] ) * inv[a];
		if ( ta > tb )
		{
			double t = ta;
			ta = tb;
			tb = t;
		}
		// a NaN bound (an infinite inverse times a zero distance) must not narrow [t0, t1]
		if ( ta == ta )
			t0 = MAX( t0, ta );
		if ( tb == tb )
			t1 = MIN( t1, tb );
		if ( t0 > t1 )
			return -1.0;
	}
	return t0;
}

struct RayHit
{
	double t;
	int brush;			// s_Brushes index, -1: none
	int side;			// dbrushsides index of the entry side
	bool nodraw;		// that side's material is toolsnodraw
};

// The nearest entry into an opaque solid brush. Entries within CM_TIE of each other tie: a side that draws
// beats one that does not.
static void NearestEntry( const dvec3 &o, const dvec3 &dir, RayHit &best )
{
	best.t = CM_RAY_MAX;
	best.brush = best.side = -1;
	best.nodraw = false;
	if ( s_Bvh.nodes.empty() )
		return;
	dvec3 inv( dir.x != 0 ? 1.0 / dir.x : 1e300, dir.y != 0 ? 1.0 / dir.y : 1e300, dir.z != 0 ? 1.0 / dir.z : 1e300 );
	int stack[128];
	double stackT[128];
	int sp = 0;
	stack[sp] = 0;
	stackT[sp++] = 0.0;
	while ( sp )
	{
		sp--;
		if ( stackT[sp] > best.t + CM_TIE )
			continue;
		const BvhNode &nd = s_Bvh.nodes[stack[sp]];
		if ( nd.left >= 0 )
		{
			// nearer child on top of the stack
			const BvhNode &l = s_Bvh.nodes[nd.left], &r = s_Bvh.nodes[nd.right];
			double tl = BoxEntryT( o, inv, best.t + CM_TIE, l.mins, l.maxs );
			double tr = BoxEntryT( o, inv, best.t + CM_TIE, r.mins, r.maxs );
			if ( sp > 124 )
				continue;		// the tree is far deeper than any real map's: skip the rest of this branch
			if ( tl >= 0 && tr >= 0 )
			{
				bool leftFirst = tl <= tr;
				stack[sp] = leftFirst ? nd.right : nd.left;
				stackT[sp++] = leftFirst ? tr : tl;
				stack[sp] = leftFirst ? nd.left : nd.right;
				stackT[sp++] = leftFirst ? tl : tr;
			}
			else if ( tl >= 0 )
			{
				stack[sp] = nd.left;
				stackT[sp++] = tl;
			}
			else if ( tr >= 0 )
			{
				stack[sp] = nd.right;
				stackT[sp++] = tr;
			}
			continue;
		}
		for ( int i = nd.first; i < nd.first + nd.count; i++ )
		{
			int k = s_BvhBrush[s_Bvh.items[i]];
			const CmBrush &cb = s_Brushes[k];
			if ( !cb.opaque )
				continue;
			double t;
			int plane;
			if ( !RayEnterBrush( o, dir, best.t + CM_TIE, cb, t, plane ) )
				continue;
			int side = cb.sides[plane];
			bool nodraw = cb.nodraw[plane] != 0;
			if ( best.brush < 0 || t < best.t - CM_TIE )
			{
				best.t = t;
				best.brush = k;
				best.side = side;
				best.nodraw = nodraw;
			}
			else if ( t <= best.t + CM_TIE && best.nodraw && !nodraw )
			{
				best.t = MIN( t, best.t );
				best.brush = k;
				best.side = side;
				best.nodraw = false;
			}
		}
	}
}

// Is the hit buried: open space is not just outside the side?
static bool HitBuried( const dvec3 &p, int side )
{
	const dplane_t &pl = dplanes[dbrushsides[side].planenum];
	dvec3 out = p + dvec3( pl.normal ) * CM_BURY;
	return dleafs[PointLeaf( out )].cluster < 0;
}

// The six faces of a cube map: forward, and the two axes the pixel offsets run along.
static const double s_CubeFaces[6][9] = {
	{ 1, 0, 0, 0, 1, 0, 0, 0, 1 },	{ -1, 0, 0, 0, 1, 0, 0, 0, 1 },
	{ 0, 1, 0, 1, 0, 0, 0, 0, 1 },	{ 0, -1, 0, 1, 0, 0, 0, 0, 1 },
	{ 0, 0, 1, 1, 0, 0, 0, 1, 0 },	{ 0, 0, -1, 1, 0, 0, 0, 1, 0 },
};

//-----------------------------------------------------------------------------
// One cubemap
//-----------------------------------------------------------------------------
static void CheckOne( int iThread, int item )
{
	const CubemapOptions &opts = *s_pOpts;
	CubemapInfo &info = s_pReport->list[item];
	const dvec3 o = info.origin;
	const double h = opts.nearZ;

	// rule 1: in a brush
	if ( !opts.keepAll )
	{
		if ( dleafs[PointLeaf( o )].cluster < 0 )
		{
			info.status = CUBEMAP_SOLID;
			info.reason = "solid_leaf";
		}
		else
		{
			std::vector<int> cand;
			BrushesInBox( o - dvec3( h, h, h ) - dvec3( CM_OVERLAP, CM_OVERLAP, CM_OVERLAP ),
						  o + dvec3( h, h, h ) + dvec3( CM_OVERLAP, CM_OVERLAP, CM_OVERLAP ), cand );
			int first = -1;
			ConvexShape shape;
			for ( size_t i = 0; i < cand.size(); i++ )
			{
				int b = s_Brushes[cand[i]].brush;
				if ( first >= 0 && b > first )
					continue;
				if ( BuildBrushShape( b, shape ) && CubeOverlapsConvex( o, h, shape ) )
					first = b;
			}
			if ( first >= 0 )
			{
				info.status = CUBEMAP_SOLID;
				info.reason = "brush";
				info.brush = first;
			}
		}

		// rule 2: sees a toolsnodraw face
		if ( info.status == CUBEMAP_OK )
		{
			const int R = opts.raysPerFace;
			for ( int f = 0; f < 6; f++ )
			{
				const double *fd = s_CubeFaces[f];
				const dvec3 fwd( fd[0], fd[1], fd[2] ), ax( fd[3], fd[4], fd[5] ), ay( fd[6], fd[7], fd[8] );
				for ( int j = 0; j < R; j++ )
				{
					for ( int i = 0; i < R; i++ )
					{
						dvec3 dir = fwd + ax * ( 2.0 * ( i + 0.5 ) / R - 1.0 ) + ay * ( 2.0 * ( j + 0.5 ) / R - 1.0 );
						DNormalize( dir );
						info.rays++;
						RayHit hit;
						NearestEntry( o, dir, hit );
						if ( hit.brush < 0 || !hit.nodraw )
							continue;
						dvec3 p = o + dir * hit.t;
						if ( HitBuried( p, hit.side ) )
							continue;
						if ( !info.hasNodrawPoint )
						{
							info.hasNodrawPoint = true;
							info.nodrawPoint = p;
							info.brush = s_Brushes[hit.brush].brush;
						}
						info.nodrawRays++;
					}
				}
			}
			if ( info.nodrawRays >= opts.nodrawMin )
			{
				info.status = CUBEMAP_NODRAW;
				info.reason = "nodraw";
			}
		}
	}

	// rule 3: in a static prop (every cubemap)
	const dvec3 lo = o - dvec3( h, h, h ), hi = o + dvec3( h, h, h );
	std::vector<ConvexShape> shapes;
	for ( size_t p = 0; p < s_Props.size(); p++ )
	{
		const StaticPropInstance &sp = s_Props[p];
		if ( sp.mins.x > hi.x || sp.maxs.x < lo.x || sp.mins.y > hi.y || sp.maxs.y < lo.y || sp.mins.z > hi.z ||
			 sp.maxs.z < lo.z )
			continue;
		StaticPropShapes( sp, shapes );
		for ( size_t c = 0; c < shapes.size(); c++ )
		{
			if ( !CubeOverlapsConvex( o, h, shapes[c] ) )
				continue;
			CubemapProp cp;
			cp.model = sp.model;
			cp.staticIndex = sp.index;
			cp.hullBox = sp.hullBox;
			info.props.push_back( cp );
			break;
		}
	}
}

//-----------------------------------------------------------------------------
// All cubemaps
//-----------------------------------------------------------------------------
void CheckCubemaps( const CubemapOptions &opts, CubemapReport &out )
{
	double t0 = Plat_FloatTime();
	out = CubemapReport();
	out.opts = opts;
	const int n = g_nCubemapSamples;
	out.list.resize( n );
	for ( int i = 0; i < n; i++ )
	{
		out.list[i].index = i;
		out.list[i].origin = dvec3( g_CubemapSamples[i].origin[0], g_CubemapSamples[i].origin[1], g_CubemapSamples[i].origin[2] );
	}
	if ( !n )
		return;

	s_pOpts = &opts;
	s_pReport = &out;
	CollectStaticPropInstances( s_Props, out.propsUnloadable );
	if ( !opts.keepAll )
		BuildBrushes();
	RunThreadsBatched( n, CheckOne );

	for ( int i = 0; i < n; i++ )
	{
		const CubemapInfo &c = out.list[i];
		out.disqualifiedSolid += c.status == CUBEMAP_SOLID;
		out.disqualifiedNodraw += c.status == CUBEMAP_NODRAW;
		out.propWarnings += !c.props.empty();
	}

	for ( int i = 0; i < n; i++ )
	{
		const CubemapInfo &c = out.list[i];
		if ( c.status == CUBEMAP_SOLID && !strcmp( c.reason, "solid_leaf" ) )
			Warning( "bspwalk: env_cubemap %d at (%.0f %.0f %.0f) is not a viewpoint: the camera is in a solid leaf\n", i,
					 c.origin.x, c.origin.y, c.origin.z );
		else if ( c.status == CUBEMAP_SOLID )
			Warning( "bspwalk: env_cubemap %d at (%.0f %.0f %.0f) is not a viewpoint: its near cube (%g units) overlaps "
					 "brush %d\n", i, c.origin.x, c.origin.y, c.origin.z, opts.nearZ, c.brush );
		else if ( c.status == CUBEMAP_NODRAW )
			Warning( "bspwalk: env_cubemap %d at (%.0f %.0f %.0f) is not a viewpoint: it sees toolsnodraw on %d of %d rays "
					 "(first at (%.1f %.1f %.1f), brush %d)\n", i, c.origin.x, c.origin.y, c.origin.z, c.nodrawRays, c.rays,
					 c.nodrawPoint.x, c.nodrawPoint.y, c.nodrawPoint.z, c.brush );
		else
		{
			for ( size_t p = 0; p < c.props.size(); p++ )
				Warning( "bspwalk: env_cubemap %d at (%.0f %.0f %.0f): its near cube overlaps static prop %d (%s)%s\n", i,
						 c.origin.x, c.origin.y, c.origin.z, c.props[p].staticIndex, c.props[p].model.c_str(),
						 c.props[p].hullBox ? "; the shape is the .mdl hull box, which is crude: this may be a false warning" : "" );
		}
	}
	if ( out.brushesUnbuilt )
		Warning( "bspwalk: %d solid world brush(es) did not close into a convex; the cubemap check ignores them\n",
				 out.brushesUnbuilt );
	out.tCheck = Plat_FloatTime() - t0;
	Msg( "cubemaps: %d, disqualified %d in brush, %d see nodraw; %d inside props%s (%.2f s)\n", n, out.disqualifiedSolid,
		 out.disqualifiedNodraw, out.propWarnings, opts.keepAll ? " (-keepcubemaps: none disqualified)" : "", out.tCheck );
	if ( out.propsUnloadable )
		Msg( "cubemaps: %d static prop(s) have neither a usable .phy nor a readable .mdl and were not checked\n",
			 out.propsUnloadable );

	std::vector<CmBrush>().swap( s_Brushes );
	std::vector<int>().swap( s_BvhBrush );
	s_Bvh = Bvh();
	std::vector<StaticPropInstance>().swap( s_Props );
	s_pOpts = NULL;
	s_pReport = NULL;
}
