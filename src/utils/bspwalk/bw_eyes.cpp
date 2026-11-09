//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: Stage 4 (#14), the eye volume. See bw_vis.h for the parts and
//          docs/bsp_walker_rebuild.md ("How #14 visibility works") for why each
//          holds every eye position the walker models (the reached nodes'
//          columns, raised by the fall columns, water, ladders, push volumes,
//          traced arcs, rides, teleports, the viewpoints). What it does not
//          model is listed there under "Accepted limitations (visibility)".
//
//=============================================================================//

#include "cmdlib.h"
#include "bsplib.h"
#include "threads.h"
#include "tier0/platform.h"
#include "bw_vis.h"
#include "bw_hash.h"
#include "bw_util.h"

#include <algorithm>
#include <float.h>
#include <limits.h>
#include <math.h>
#include <string.h>

// Winding helpers: the same base size and chop epsilon as bw_grow.cpp.
static const double EYE_BASE_HALFSIZE = 131072.0;
static const double EYE_CHOP_EPS = 1e-9;
// Vertices closer than this are one.
static const double EYE_WELD = 1e-7;
// A plane separates two convexes when every vertex of one is at least this far
// outside it (strictly outside). A convex that only touches the plane (within
// this) is not separated by it and goes on to the exact clip, where touching
// removes nothing.
static const double EYE_TOUCH = 1e-6;
// BSP descent: a piece goes down one side of a node plane only when it is at
// least this far clear of the plane; nearer, each side gets the part of it
// within this of its half-space, so a piece part within rounding of a plane is
// never discarded.
static const double EYE_SPLIT_EPS = 1e-4;
static const double EYE_PI = 3.14159265358979323846;

const char *EyeSourceName( int source )
{
	static const char *s_Names[EYESRC_COUNT] = { "node", "water", "ladder", "arc", "ride", "teleport", "cubemap",
												 "point_camera", "point_viewcontrol", "cli", "sky_camera", "surf" };
	return ( source >= 0 && source < EYESRC_COUNT ) ? s_Names[source] : "?";
}

static void PieceBounds( EyePiece &p )
{
	p.mins = dvec3( 1e300, 1e300, 1e300 );
	p.maxs = dvec3( -1e300, -1e300, -1e300 );
	for ( size_t i = 0; i < p.verts.size(); i++ )
	{
		p.mins = DMin( p.mins, p.verts[i] );
		p.maxs = DMax( p.maxs, p.verts[i] );
	}
}

static void AddWelded( std::vector<dvec3> &verts, const dvec3 &v )
{
	for ( size_t i = 0; i < verts.size(); i++ )
	{
		dvec3 d = verts[i] - v;
		if ( fabs( d.x ) <= EYE_WELD && fabs( d.y ) <= EYE_WELD && fabs( d.z ) <= EYE_WELD )
			return;
	}
	verts.push_back( v );
}

bool EyePolyFromPlanes( std::vector<DPlane> &planes, EyePiece &out )
{
	out.kind = EYE_POLY;
	out.verts.clear();
	std::vector<DPlane> kept;
	kept.reserve( planes.size() );
	DWinding w;
	int n = (int)planes.size();
	int faces = 0;
	for ( int i = 0; i < n; i++ )
	{
		DBaseWindingForPlane( planes[i].n, planes[i].d, EYE_BASE_HALFSIZE, w );
		for ( int j = 0; j < n && !w.empty(); j++ )
		{
			if ( j == i )
				continue;
			// a later plane with the same facing that w lies on (within the
			// chop epsilon) is a duplicate of this one: it does not chop this
			// face away. The later plane's own face lies on this plane, so
			// this plane chops it to nothing (DChopWindingInPlace empties a
			// winding with no point in front) and it is dropped: one copy stays.
			// Chopping both would drop both and leave the piece open there.
			if ( j > i && DDot( planes[i].n, planes[j].n ) > 0 )
			{
				bool on = true;
				for ( size_t k = 0; k < w.size() && on; k++ )
					on = fabs( DDot( planes[j].n, w[k] ) - planes[j].d ) <= EYE_CHOP_EPS;
				if ( on )
					continue;
			}
			DChopWindingInPlace( w, -planes[j].n, -planes[j].d, EYE_CHOP_EPS );
		}
		// a plane that bounds anything stays, however small its face: dropping
		// the sides of a sliver would leave an unbounded slab
		if ( w.size() < 3 )
			continue;
		kept.push_back( planes[i] );
		faces += DWindingArea( w ) >= 1e-9;
		for ( size_t k = 0; k < w.size(); k++ )
			AddWelded( out.verts, w[k] );
	}
	PieceBounds( out );
	// open (a vertex out at the base winding) or flat: no piece
	bool open = false;
	for ( int a = 0; a < 3 && !out.verts.empty(); a++ )
		open = open || out.maxs[a] - out.mins[a] > EYE_BASE_HALFSIZE * 0.5;
	if ( faces < 4 || out.verts.size() < 4 || open )
	{
		out.verts.clear();
		out.planes.clear();
		return false;
	}
	out.planes.swap( kept );
	return true;
}

bool EyePolyFromPoints( const std::vector<dvec3> &pts, EyePiece &out )
{
	ConvexShape s;
	if ( !ConvexFromPoints( pts.data(), (int)pts.size(), s ) )
		return false;
	out.kind = EYE_POLY;
	out.planes.resize( s.faces.size() );
	for ( size_t f = 0; f < s.faces.size(); f++ )
	{
		out.planes[f].n = s.faceNormals[f];
		// the most outward vertex of the face, so float-flat faces stay outside
		double d = -1e300;
		for ( size_t k = 0; k < s.faces[f].size(); k++ )
			d = MAX( d, DDot( s.faceNormals[f], s.verts[s.faces[f][k]] ) );
		out.planes[f].d = d;
	}
	out.verts = s.verts;
	PieceBounds( out );
	return out.planes.size() >= 4;
}

bool EyePolyClip( const EyePiece &in, const dvec3 &n, double d, EyePiece &out )
{
	std::vector<DPlane> planes( in.planes );
	DPlane p;
	p.n = n;
	p.d = d;
	planes.push_back( p );
	out.source = in.source;
	return EyePolyFromPlanes( planes, out );
}

// Every vertex of 'verts' has n . x - d >= EYE_TOUCH: all strictly outside the
// half-space n . x <= d. A set that touches the plane is not all outside.
static bool AllOutside( const std::vector<dvec3> &verts, const dvec3 &n, double d )
{
	for ( size_t i = 0; i < verts.size(); i++ )
	{
		if ( DDot( n, verts[i] ) < d + EYE_TOUCH )
			return false;
	}
	return true;
}

static bool AllInside( const std::vector<dvec3> &verts, const dvec3 &n, double d )
{
	for ( size_t i = 0; i < verts.size(); i++ )
	{
		if ( DDot( n, verts[i] ) > d + EYE_TOUCH )
			return false;
	}
	return true;
}

void EyePolySubtract( const EyePiece &poly, const std::vector<DPlane> &q, std::vector<EyePiece> &out )
{
	// a separating plane of q: poly is entirely outside it
	for ( size_t k = 0; k < q.size(); k++ )
	{
		if ( AllOutside( poly.verts, q[k].n, q[k].d ) )
		{
			out.push_back( poly );
			return;
		}
	}
	EyePiece cur = poly, front, back;
	for ( size_t k = 0; k < q.size(); k++ )
	{
		if ( AllInside( cur.verts, q[k].n, q[k].d ) )
			continue;	// no part of cur in front of this plane
		if ( EyePolyClip( cur, -q[k].n, -q[k].d, front ) )
			out.push_back( front );
		if ( !EyePolyClip( cur, q[k].n, q[k].d, back ) )
			return;
		cur.planes.swap( back.planes );
		cur.verts.swap( back.verts );
		cur.mins = back.mins;
		cur.maxs = back.maxs;
	}
	// what is left lies inside q
}

// Do the shrunk polytope q and the convex piece p overlap with positive depth?
// A plane of either that has the other entirely outside separates them.
static bool MayOverlap( const GrownPolytope &q, double shrink, const EyePiece &p )
{
	if ( q.maxs.x < p.mins.x || q.mins.x > p.maxs.x || q.maxs.y < p.mins.y || q.mins.y > p.maxs.y ||
		 q.maxs.z < p.mins.z || q.mins.z > p.maxs.z )
		return false;
	for ( size_t k = 0; k < q.planes.size(); k++ )
	{
		if ( AllOutside( p.verts, q.planes[k].n, q.planes[k].d - shrink ) )
			return false;
	}
	// q's own vertices are outside its shrunk self, so they bound it from outside
	for ( size_t k = 0; k < p.planes.size(); k++ )
	{
		if ( AllOutside( q.verts, p.planes[k].n, p.planes[k].d ) )
			return false;
	}
	return true;
}

// A column cut into more pieces than this keeps the rest unclipped: that only
// over-reaches (eyes inside obstacles), and bounds the work per node.
static const size_t FEET_MAX_PIECES = 4096;

int FeetColumn( const DWinding &poly, const DPlane &plane, double rise,
				const std::vector<const GrownPolytope *> &obstacles, double shrink, std::vector<EyePiece> &out )
{
	int n = (int)poly.size();
	if ( n < 3 || !( plane.n.z > 0 ) )
		return 0;
	dvec3 c( 0, 0, 0 );
	for ( int i = 0; i < n; i++ )
		c += poly[i];
	c = c * ( 1.0 / n );

	std::vector<DPlane> planes;
	for ( int i = 0; i < n; i++ )
	{
		const dvec3 &a = poly[i];
		const dvec3 &b = poly[( i + 1 ) % n];
		dvec3 e = b - a;
		DPlane p;
		p.n = dvec3( e.y, -e.x, 0 );
		if ( DNormalize( p.n ) < 1e-12 )
			continue;
		p.d = DDot( p.n, a );
		if ( DDot( p.n, c ) > p.d )
		{
			p.n = -p.n;
			p.d = -p.d;
		}
		planes.push_back( p );
	}
	DPlane bottom, top;
	bottom.n = -plane.n;
	bottom.d = -plane.d;
	top.n = plane.n;
	top.d = plane.d + rise * plane.n.z;
	planes.push_back( bottom );
	planes.push_back( top );

	EyePiece prism;
	prism.source = EYESRC_NODE;
	if ( !EyePolyFromPlanes( planes, prism ) )
		return 0;

	std::vector<EyePiece> pieces( 1, prism ), next;
	std::vector<DPlane> both;
	std::vector<dvec3> ext;
	EyePiece meet, extr;
	double H = rise + 1.0;
	int capped = 0;
	for ( size_t o = 0; o < obstacles.size() && !pieces.empty(); o++ )
	{
		if ( pieces.size() > FEET_MAX_PIECES )
		{
			capped = 1;
			break;
		}
		const GrownPolytope &q = *obstacles[o];
		if ( !MayOverlap( q, shrink, prism ) )
			continue;
		both = prism.planes;
		for ( size_t k = 0; k < q.planes.size(); k++ )
		{
			DPlane s = q.planes[k];
			s.d -= shrink;
			both.push_back( s );
		}
		if ( !EyePolyFromPlanes( both, meet ) )
			continue;	// they only touch
		// the part of the prism at or above the obstacle: the upward extrusion of
		// (obstacle n prism), here up past the prism's top
		ext = meet.verts;
		for ( size_t k = 0; k < meet.verts.size(); k++ )
			ext.push_back( meet.verts[k] + dvec3( 0, 0, H ) );
		if ( !EyePolyFromPoints( ext, extr ) )
			continue;
		next.clear();
		for ( size_t p = 0; p < pieces.size(); p++ )
			EyePolySubtract( pieces[p], extr.planes, next );
		pieces.swap( next );
	}
	for ( size_t p = 0; p < pieces.size(); p++ )
	{
		pieces[p].source = EYESRC_NODE;
		out.push_back( pieces[p] );
	}
	return capped;
}

//-----------------------------------------------------------------------------
// BSP descent
//-----------------------------------------------------------------------------
int PointLeaf( const dvec3 &p )
{
	int node = dmodels[0].headnode;
	while ( node >= 0 )
	{
		const dnode_t &dn = dnodes[node];
		const dplane_t &pl = dplanes[dn.planenum];
		double d = pl.normal.x * p.x + pl.normal.y * p.y + pl.normal.z * p.z - pl.dist;
		node = dn.children[d >= 0 ? 0 : 1];
	}
	return -1 - node;
}

static thread_local int t_LeafFlags;	// PieceClusters' return value, built by MarkLeaf
static thread_local int t_SampleSky = -1;	// the sky camera whose skybox copy is being sampled, else -1 (EmitPiece runs on worker threads)

//-----------------------------------------------------------------------------
// EyeBound: the k-DOP of an eye region (bw_vis.h)
//-----------------------------------------------------------------------------
namespace {
struct RegionDirTable
{
	dvec3 d[REGION_DIRS];

	RegionDirTable()
	{
		int n = 0;
		d[n++] = dvec3( 1, 0, 0 );
		d[n++] = dvec3( -1, 0, 0 );
		d[n++] = dvec3( 0, 1, 0 );
		d[n++] = dvec3( 0, -1, 0 );
		d[n++] = dvec3( 0, 0, 1 );
		d[n++] = dvec3( 0, 0, -1 );
		const double s2 = sqrt( 0.5 ), s3 = sqrt( 1.0 / 3.0 );
		for ( int a = 0; a < 3; a++ )		// the 12 edge diagonals: xy, yz, zx
		{
			int b = ( a + 1 ) % 3;
			for ( int sa = -1; sa <= 1; sa += 2 )
			{
				for ( int sb = -1; sb <= 1; sb += 2 )
				{
					double v[3] = { 0, 0, 0 };
					v[a] = sa * s2;
					v[b] = sb * s2;
					d[n++] = dvec3( v[0], v[1], v[2] );
				}
			}
		}
		for ( int c = 0; c < 8; c++ )		// the 8 corner diagonals
			d[n++] = dvec3( ( c & 1 ) ? s3 : -s3, ( c & 2 ) ? s3 : -s3, ( c & 4 ) ? s3 : -s3 );
		for ( int j = 0; j < 8; j++ )		// across the xy plane at odd multiples of 22.5 degrees: most walls are vertical
		{
			double ang = ( 2 * j + 1 ) * EYE_PI / 8.0;
			d[n++] = dvec3( cos( ang ), sin( ang ), 0 );
		}
		if ( n != REGION_DIRS )
			Error( "bspwalk: eye region directions: built %d, expected %d\n", n, (int)REGION_DIRS );
	}
};
}

static const RegionDirTable &RegionDirs()
{
	static const RegionDirTable s_Table;
	return s_Table;
}

const dvec3 &RegionDir( int i )
{
	return RegionDirs().d[i];
}

// The smallest float at or above s.
static inline float RoundUpFloat( double s )
{
	float f = (float)s;
	if ( (double)f < s )
		f = nextafterf( f, FLT_MAX );
	return f;
}

EyeBound::EyeBound() : lo( 0, 0, 0 ), hi( 0, 0, 0 ), any( false )
{
	for ( int k = 0; k < REGION_EXTRA; k++ )
		h[k] = -FLT_MAX;
}

void EyeBound::Add( const dvec3 &v )
{
	lo = any ? DMin( lo, v ) : v;
	hi = any ? DMax( hi, v ) : v;
	any = true;
	const RegionDirTable &T = RegionDirs();
	for ( int k = 0; k < REGION_EXTRA; k++ )
	{
		const dvec3 &d = T.d[6 + k];
		double s = d.x * v.x + d.y * v.y + d.z * v.z;
		if ( s > (double)h[k] )
			h[k] = RoundUpFloat( s );
	}
}

void EyeBound::Merge( const EyeBound &o )
{
	if ( !o.any )
		return;
	if ( !any )
	{
		*this = o;
		return;
	}
	lo = DMin( lo, o.lo );
	hi = DMax( hi, o.hi );
	for ( int k = 0; k < REGION_EXTRA; k++ )
	{
		if ( o.h[k] > h[k] )
			h[k] = o.h[k];
	}
}

void EyeBound::Unbounded()
{
	lo = dvec3( -1e30, -1e30, -1e30 );
	hi = dvec3( 1e30, 1e30, 1e30 );
	for ( int k = 0; k < REGION_EXTRA; k++ )
		h[k] = 1e30f;
	any = true;
}

void EyeBound::AtOrigin()
{
	lo = hi = dvec3( 0, 0, 0 );
	for ( int k = 0; k < REGION_EXTRA; k++ )
		h[k] = 0.0f;
	any = true;
}

double EyeBound::BoxSupport( const dvec3 &n ) const
{
	return n.x * ( n.x > 0 ? hi.x : lo.x ) + n.y * ( n.y > 0 ? hi.y : lo.y ) + n.z * ( n.z > 0 ? hi.z : lo.z );
}

// The bound is the least of sum(w_i c_i) over the ways to write n = sum(w_i d_i) with
// every w_i >= 0, d_i the region directions and c_i their support values: for any
// point x of the region n . x = sum(w_i d_i . x) <= sum(w_i c_i). That is a linear
// program with three equality rows; the simplex below starts from the box's own
// decomposition (the signed axes, w = |n|), so the first answer is the box bound, and
// every basis it reaches is another valid decomposition, so stopping early (the pivot
// cap, a numerically odd basis) still returns a bound that holds.
static const int SUPPORT_MAX_PIVOTS = 96;

double EyeBound::Support( const dvec3 &n, double stopAt ) const
{
	double best = BoxSupport( n );
	if ( best <= stopAt || hi.x >= 1e29 )
		return best;
	const RegionDirTable &T = RegionDirs();
	double c[REGION_DIRS];
	c[0] = hi.x;
	c[1] = -lo.x;
	c[2] = hi.y;
	c[3] = -lo.y;
	c[4] = hi.z;
	c[5] = -lo.z;
	for ( int k = 0; k < REGION_EXTRA; k++ )
		c[6 + k] = h[k];
	// how far a vertex coordinate reaches: scales the rounding slack of a decomposition
	double M = MAX( MAX( MAX( fabs( lo.x ), fabs( hi.x ) ), MAX( fabs( lo.y ), fabs( hi.y ) ) ), MAX( fabs( lo.z ), fabs( hi.z ) ) );
	int basis[3] = { n.x >= 0 ? 0 : 1, n.y >= 0 ? 2 : 3, n.z >= 0 ? 4 : 5 };
	for ( int it = 0; it < SUPPORT_MAX_PIVOTS; it++ )
	{
		// basis columns a, b, e: the weights are n against the reciprocal vectors
		const dvec3 &a = T.d[basis[0]], &b = T.d[basis[1]], &e = T.d[basis[2]];
		const dvec3 rb = DCross( b, e ), re = DCross( e, a ), ra = DCross( a, b );
		double det = DDot( a, rb );
		if ( fabs( det ) < 1e-9 )
			break;
		double w[3] = { DDot( n, rb ) / det, DDot( n, re ) / det, DDot( n, ra ) / det };
		// this decomposition of n; a weight under 0 by rounding is cut to 0 and the
		// leftover r of n priced at the largest coordinate a vertex has
		double obj = 0;
		dvec3 r = n;
		bool valid = true;
		for ( int k = 0; k < 3; k++ )
		{
			if ( w[k] < 0 )
			{
				if ( w[k] < -1e-9 )
					valid = false;
				w[k] = 0;
			}
			obj += w[k] * c[basis[k]];
			r -= T.d[basis[k]] * w[k];
		}
		if ( !valid )
			break;
		double bound = obj + ( fabs( r.x ) + fabs( r.y ) + fabs( r.z ) ) * M + 1e-9;
		if ( bound < best )
			best = bound;
		if ( best <= stopAt )
			break;
		// the point y with d_i . y = c_i on the basis; the direction it violates most enters
		const dvec3 y = ( rb * c[basis[0]] + re * c[basis[1]] + ra * c[basis[2]] ) * ( 1.0 / det );
		int enter = -1;
		double worst = 1e-7;
		for ( int j = 0; j < REGION_DIRS; j++ )
		{
			if ( j == basis[0] || j == basis[1] || j == basis[2] )
				continue;
			double viol = DDot( T.d[j], y ) - c[j];
			if ( viol > worst )
			{
				worst = viol;
				enter = j;
			}
		}
		if ( enter < 0 )
			break;		// no direction is violated: this decomposition is the least
		const dvec3 &dj = T.d[enter];
		const double dl[3] = { DDot( dj, rb ) / det, DDot( dj, re ) / det, DDot( dj, ra ) / det };
		int leave = -1;
		double tMin = 1e300;
		for ( int k = 0; k < 3; k++ )
		{
			if ( dl[k] > 1e-9 && w[k] / dl[k] < tMin )
			{
				tMin = w[k] / dl[k];
				leave = k;
			}
		}
		if ( leave < 0 )
			break;
		basis[leave] = enter;
	}
	return best;
}

// Per thread and cluster (EmitPiece points this at its EyeThread while it
// runs): the bound of every part of an eye piece the descent put in the cluster
// (a side that fails to build adds that side's vertices), so it holds every eye
// position the pieces give the cluster. An eye cluster with no lattice sample
// in it (a piece that only grazes a thin leaf) cannot be ray tested; its bound
// tells which faces it may still see (bw_vis BareMaySee), and the same bound
// catches a cluster whose samples all lie behind a face its pieces reach in
// front of (bw_vis CandidateEyes).
static thread_local std::vector<EyeBound> *t_pRegion;

static inline void MarkRegion( int leaf, const dvec3 *v, size_t n )
{
	if ( !t_pRegion || !n )
		return;
	int c = dleafs[leaf].cluster;
	if ( c < 0 || c >= (int)t_pRegion->size() )
		return;
	EyeBound &r = ( *t_pRegion )[c];
	for ( size_t i = 0; i < n; i++ )
		r.Add( v[i] );
}

static inline void MarkLeaf( int leaf, std::vector<unsigned char> &clusters )
{
	int c = dleafs[leaf].cluster;
	if ( c >= 0 && c < (int)clusters.size() )
	{
		clusters[c] = 1;
		t_LeafFlags |= 1;
	}
	else
		t_LeafFlags |= 2;
}

// The part of a -> b where a value linear along it (va at a, vb at b) is >= 0.
static bool ClipSegment( dvec3 &a, dvec3 &b, double va, double vb )
{
	if ( va < 0 && vb < 0 )
		return false;
	dvec3 a0 = a, b0 = b;
	if ( va < 0 )
		a = a0 + ( b0 - a0 ) * ( va / ( va - vb ) );
	else if ( vb < 0 )
		b = a0 + ( b0 - a0 ) * ( va / ( va - vb ) );
	return true;
}

static void SegmentClusters( int node, dvec3 a, dvec3 b, std::vector<unsigned char> &clusters )
{
	while ( node >= 0 )
	{
		const dnode_t &dn = dnodes[node];
		const dplane_t &pl = dplanes[dn.planenum];
		dvec3 n( pl.normal.x, pl.normal.y, pl.normal.z );
		double da = DDot( n, a ) - pl.dist, db = DDot( n, b ) - pl.dist;
		if ( da >= EYE_SPLIT_EPS && db >= EYE_SPLIT_EPS )
		{
			node = dn.children[0];
			continue;
		}
		if ( da <= -EYE_SPLIT_EPS && db <= -EYE_SPLIT_EPS )
		{
			node = dn.children[1];
			continue;
		}
		// the front gets the part at most EYE_SPLIT_EPS behind the plane, the
		// back the part at most that far in front: the band goes both ways
		dvec3 fa = a, fb = b;
		if ( ClipSegment( fa, fb, da + EYE_SPLIT_EPS, db + EYE_SPLIT_EPS ) )
			SegmentClusters( dn.children[0], fa, fb, clusters );
		if ( !ClipSegment( a, b, EYE_SPLIT_EPS - da, EYE_SPLIT_EPS - db ) )
			return;
		node = dn.children[1];
	}
	MarkLeaf( -1 - node, clusters );
	const dvec3 ends[2] = { a, b };
	MarkRegion( -1 - node, ends, 2 );
}

static void PolyClusters( int node, const EyePiece &p, std::vector<unsigned char> &clusters )
{
	while ( node >= 0 )
	{
		const dnode_t &dn = dnodes[node];
		const dplane_t &pl = dplanes[dn.planenum];
		dvec3 n( pl.normal.x, pl.normal.y, pl.normal.z );
		double lo = 1e300, hi = -1e300;
		for ( size_t i = 0; i < p.verts.size(); i++ )
		{
			double d = DDot( n, p.verts[i] ) - pl.dist;
			lo = MIN( lo, d );
			hi = MAX( hi, d );
		}
		if ( lo >= EYE_SPLIT_EPS )
		{
			node = dn.children[0];
			continue;
		}
		if ( hi <= -EYE_SPLIT_EPS )
		{
			node = dn.children[1];
			continue;
		}
		// Split with the band both ways: the front gets the part at most
		// EYE_SPLIT_EPS behind the plane, the back the part at most that far in
		// front. A side that does not build (a sliver, or a piece that only
		// touches the band at an edge or corner) gets the piece's vertices on
		// that side as points, so no part of the piece is lost and none is
		// pushed into leaves it does not reach.
		for ( int side = 0; side < 2; side++ )
		{
			EyePiece part;
			bool ok = side == 0 ? EyePolyClip( p, -n, -( (double)pl.dist - EYE_SPLIT_EPS ), part )
								: EyePolyClip( p, n, (double)pl.dist + EYE_SPLIT_EPS, part );
			if ( ok )
			{
				PolyClusters( dn.children[side], part, clusters );
				continue;
			}
			for ( size_t i = 0; i < p.verts.size(); i++ )
			{
				double d = DDot( n, p.verts[i] ) - pl.dist;
				if ( side == 0 ? d >= -EYE_SPLIT_EPS : d <= EYE_SPLIT_EPS )
					SegmentClusters( dn.children[side], p.verts[i], p.verts[i], clusters );
			}
		}
		return;
	}
	MarkLeaf( -1 - node, clusters );
	if ( !p.verts.empty() )
		MarkRegion( -1 - node, &p.verts[0], p.verts.size() );
}

int PieceClusters( const EyePiece &piece, std::vector<unsigned char> &clusters )
{
	t_LeafFlags = 0;
	if ( clusters.empty() || piece.verts.empty() )
		return 0;
	int head = dmodels[0].headnode;
	if ( piece.kind == EYE_POINT )
	{
		int leaf = PointLeaf( piece.verts[0] );
		MarkLeaf( leaf, clusters );
		MarkRegion( leaf, &piece.verts[0], 1 );
	}
	else if ( piece.kind == EYE_SEGMENT )
		SegmentClusters( head, piece.verts[0], piece.verts[1], clusters );
	else
		PolyClusters( head, piece, clusters );
	return t_LeafFlags;
}

void SkyMapPiece( const EyePiece &in, const dvec3 &origin, double scale, EyePiece &out )
{
	out = in;
	double inv = 1.0 / scale;
	for ( size_t i = 0; i < out.verts.size(); i++ )
		out.verts[i] = origin + in.verts[i] * inv;
	// n . x <= d  ->  n . (o + x / s) <= n . o + d / s
	for ( size_t k = 0; k < out.planes.size(); k++ )
		out.planes[k].d = DDot( out.planes[k].n, origin ) + in.planes[k].d * inv;
	out.mins = origin + in.mins * inv;
	out.maxs = origin + in.maxs * inv;
}

static EyePiece Translated( const EyePiece &in, const dvec3 &t )
{
	EyePiece out = in;
	for ( size_t i = 0; i < out.verts.size(); i++ )
		out.verts[i] += t;
	for ( size_t k = 0; k < out.planes.size(); k++ )
		out.planes[k].d += DDot( out.planes[k].n, t );
	out.mins += t;
	out.maxs += t;
	return out;
}

static EyePiece PointPiece( const dvec3 &p, int source )
{
	EyePiece e;
	e.kind = EYE_POINT;
	e.source = source;
	e.verts.push_back( p );
	e.mins = e.maxs = p;
	return e;
}

static EyePiece SegmentPiece( const dvec3 &a, const dvec3 &b, int source )
{
	EyePiece e;
	e.kind = EYE_SEGMENT;
	e.source = source;
	e.verts.push_back( a );
	e.verts.push_back( b );
	PieceBounds( e );
	return e;
}

//-----------------------------------------------------------------------------
// Samples (tier B): every vertex, plus a grid of vertical lines 'spacing' apart
// (world-aligned) with points at both ends of each line's span and every
// 'spacing' between. Segments: both ends and every 'spacing'.
//-----------------------------------------------------------------------------
static void PushSample( std::vector<EyeSample> &out, const dvec3 &p, int source )
{
	EyeSample s;
	s.sky = t_SampleSky;
	s.x = (float)p.x;
	s.y = (float)p.y;
	s.z = (float)p.z;
	s.cluster = -1;
	s.source = source;
	out.push_back( s );
}

// The vertical line through (x, y) across a polyhedron: points at both ends of
// its span and every 'spacing' between. A side plane misses by more than
// EYE_TOUCH, or a span inverted by more than that, before the line is off the
// piece (clip rounding must not drop a boundary line); an inverted span within
// it is one point. False when the line misses.
static bool SampleColumn( const EyePiece &p, double x, double y, double spacing, std::vector<EyeSample> &out )
{
	double zlo = -1e300, zhi = 1e300;
	for ( size_t k = 0; k < p.planes.size(); k++ )
	{
		const DPlane &pl = p.planes[k];
		double r = pl.d - pl.n.x * x - pl.n.y * y;
		if ( pl.n.z > 1e-9 )
			zhi = MIN( zhi, r / pl.n.z );
		else if ( pl.n.z < -1e-9 )
			zlo = MAX( zlo, r / pl.n.z );
		else if ( r < -EYE_TOUCH )
			return false;
	}
	if ( zlo < -1e299 || zhi > 1e299 || zlo > zhi + EYE_TOUCH )
		return false;
	if ( zlo >= zhi )
	{
		PushSample( out, dvec3( x, y, 0.5 * ( zlo + zhi ) ), p.source );
		return true;
	}
	PushSample( out, dvec3( x, y, zlo ), p.source );
	for ( double z = ceil( zlo / spacing ) * spacing; z < zhi; z += spacing )
	{
		if ( z > zlo )
			PushSample( out, dvec3( x, y, z ), p.source );
	}
	PushSample( out, dvec3( x, y, zhi ), p.source );
	return true;
}

static void SamplePiece( const EyePiece &p, double spacing, std::vector<EyeSample> &out )
{
	for ( size_t i = 0; i < p.verts.size(); i++ )
		PushSample( out, p.verts[i], p.source );
	if ( p.kind == EYE_SEGMENT )
	{
		dvec3 d = p.verts[1] - p.verts[0];
		double len = DLength( d );
		int steps = (int)floor( len / spacing );
		for ( int k = 1; k <= steps; k++ )
		{
			double t = k * spacing / len;
			if ( t < 1.0 )
				PushSample( out, p.verts[0] + d * t, p.source );
		}
		return;
	}
	if ( p.kind != EYE_POLY )
		return;
	double x0 = ceil( ( p.mins.x - EYE_TOUCH ) / spacing ) * spacing, y0 = ceil( ( p.mins.y - EYE_TOUCH ) / spacing ) * spacing;
	int cols = 0;
	for ( double x = x0; x <= p.maxs.x + EYE_TOUCH; x += spacing )
	{
		for ( double y = y0; y <= p.maxs.y + EYE_TOUCH; y += spacing )
			cols += SampleColumn( p, x, y, spacing, out );
	}
	// A piece the lattice under-covers (narrower than the spacing, or a thin
	// diagonal strip): also sample the chord between the two vertices farthest
	// apart in XY. The footprint is convex, so the chord lies inside it.
	double best = -1;
	dvec3 a, b;
	for ( size_t i = 0; i < p.verts.size(); i++ )
	{
		for ( size_t j = i + 1; j < p.verts.size(); j++ )
		{
			double dx = p.verts[j].x - p.verts[i].x, dy = p.verts[j].y - p.verts[i].y;
			if ( dx * dx + dy * dy > best )
			{
				best = dx * dx + dy * dy;
				a = p.verts[i];
				b = p.verts[j];
			}
		}
	}
	double len = best > 0 ? sqrt( best ) : 0;
	if ( len <= 0 || cols >= (int)floor( len / spacing ) + 1 )
		return;
	int steps = (int)ceil( len / spacing );	// its ends are vertices, sampled above
	for ( int k = 1; k < steps; k++ )
	{
		double t = (double)k / steps;
		SampleColumn( p, a.x + ( b.x - a.x ) * t, a.y + ( b.y - a.y ) * t, spacing, out );
	}
}

void SampleEyePiece( const EyePiece &p, double spacing, std::vector<EyeSample> &out )
{
	SamplePiece( p, spacing, out );
}

//-----------------------------------------------------------------------------
// Per-thread accumulation
//-----------------------------------------------------------------------------
// A viewpoint piece with a sample in a solid leaf: which camera (EyePiece::viewpoint),
// which of its pieces, and that sample.
struct SolidViewHit
{
	int viewpoint, part, source;
	dvec3 at;
};

struct EyeThread
{
	std::vector<unsigned char> clusters;
	std::vector<EyeSample> samples;
	std::vector<EyeBound> region;		// per cluster, see MarkRegion (sized by the first piece it emits)
	std::vector<EyePiece> keep;
	std::vector<int> cand;
	std::vector<const GrownPolytope *> obs;
	std::vector<EyePiece> pieces, tmp, eye;
	std::vector<float> arcTimes;		// ArcChordEyes' chord end times
	int pieces_[EYESRC_COUNT];
	int mapped;
	std::vector<SolidViewHit> solidViews;				// viewpoint pieces with a sample in solid
	int cappedColumns;									// columns that hit FEET_MAX_PIECES
	EyeBound all;										// bound of every piece (skybox copies too)
	std::vector<EyeBound> sampleBound;					// SampleClusterChunk: per cluster, its samples
	std::vector<int> perCluster;						// ... and how many
};

static EyeThread s_EyeThreads[MAX_TOOL_THREADS + 1];
static const CollisionWorld *s_pEyeWorld;
static const PlayerProfile *s_pEyeProfile;
static const WalkGraph *s_pEyeGraph;
static const EyeOptions *s_pEyeOpts;
static double s_EyeSpacing;					// the sample spacing in use (opts.spacing, or wider)
static EyeVolume *s_pEyeVol;
static ObstacleSet s_EyeCrouchObs, s_EyeStandObs;	// stand: built only for the stand column
static bool s_bStandColumn;					// the profile needs a stand column (see NodeEyesOne)
static std::vector<int> s_EyeNodes;			// reached nodes
static std::vector<double> s_EyeRise;		// per node: feet rise of its column
// Standing eyes get a column of their own when the crouch box raised to a
// standing eye (feet + eye_stand - eye_crouch) sticks out of the top of the
// stand box (eye_stand - eye_crouch > stand_height - crouch_height; tf: 23 >
// 20): a ceiling just over a standing player's head would cut the crouch column
// below that player's eye. Then this is eye_stand - eye_crouch (the stand
// column's rise is the crouch column's minus this, so both reach the same eye
// top); 0 when the crouch column holds every standing eye (hl2: 36 = 36).
static double s_EyeStandLift;
static std::vector<int> s_MoverIndex;		// (convex, pose) -> graph.movers index, by s_MoverKey
static std::vector<long long> s_MoverKey;

// One finished eye piece: its clusters, its skybox copy, its samples.
// A viewpoint in solid is a camera the engine draws from with no cluster: every
// cluster is then drawn (the view cluster -1 has no PVS). It adds no eye cluster:
// bw_vis keeps visible the faces its samples in solid see instead (#76), so one
// misplaced cubemap does not switch hiding off for the whole map. Skybox copies
// of the eye volume are not viewpoints in that sense: the 3D skybox pass takes
// its PVS from the sky_camera origin (added as a viewpoint), and copies that
// reach into the skybox room's solid are common (every CS:S map tried has them)
// without the real world showing in the skybox.
bool IsViewpointSource( int source )
{
	return source == EYESRC_CUBEMAP || source == EYESRC_CAMERA || source == EYESRC_VIEWCONTROL || source == EYESRC_CLI ||
		   source == EYESRC_SKYCAMERA;
}

static void EmitPiece( EyeThread &t, const EyePiece &p )
{
	const EyeVolume &ev = *s_pEyeVol;
	if ( t.region.empty() )
		t.region.assign( t.clusters.size(), EyeBound() );
	t_pRegion = &t.region;
	PieceClusters( p, t.clusters );
	t.pieces_[p.source]++;
	for ( size_t i = 0; i < p.verts.size(); i++ )
		t.all.Add( p.verts[i] );
	size_t first = t.samples.size();
	SamplePiece( p, s_EyeSpacing, t.samples );
	// A viewpoint with a sample in a solid leaf (cluster -1): bw_vis keys its rule for
	// viewpoints in solid off exactly those samples (s_SolidEyes), so the list names
	// every viewpoint that has one, not only those wholly in solid. One entry per
	// piece here (its first such sample); the gather keeps one per viewpoint.
	if ( IsViewpointSource( p.source ) )
	{
		for ( size_t i = first; i < t.samples.size(); i++ )
		{
			dvec3 at( t.samples[i].x, t.samples[i].y, t.samples[i].z );
			if ( dleafs[PointLeaf( at )].cluster < 0 )
			{
				SolidViewHit h;
				h.viewpoint = p.viewpoint;
				h.part = p.viewPart;
				h.source = p.source;
				h.at = at;
				t.solidViews.push_back( h );
				break;
			}
		}
	}
	if ( s_pEyeOpts->keepPieces )
		t.keep.push_back( p );
	for ( size_t k = 0; k < ev.skyOrigins.size(); k++ )
	{
		EyePiece m;
		SkyMapPiece( p, ev.skyOrigins[k], ev.skyScales[k], m );
		m.sky = (int)k + 1;
		t_SampleSky = (int)k;
		PieceClusters( m, t.clusters );
		t.mapped++;
		for ( size_t i = 0; i < m.verts.size(); i++ )
			t.all.Add( m.verts[i] );
		// a mapped piece is 1/scale the size: sample it 1/scale as finely
		SamplePiece( m, s_EyeSpacing / ev.skyScales[k], t.samples );
		t_SampleSky = -1;
		if ( s_pEyeOpts->keepPieces )
			t.keep.push_back( m );
	}
	t_pRegion = NULL;
}

// Obstacles of a posture whose boxes meet [mins, maxs].
static void QueryObstacles( EyeThread &t, const ObstacleSet &obs, const dvec3 &mins, const dvec3 &maxs )
{
	t.cand.clear();
	t.obs.clear();
	obs.hash.Query( mins.x, mins.y, maxs.x, maxs.y, t.cand );
	for ( size_t k = 0; k < t.cand.size(); k++ )
	{
		const GrownPolytope &q = ( *obs.polys )[t.cand[k]];
		if ( q.maxs.z < mins.z || q.mins.z > maxs.z )
			continue;
		t.obs.push_back( &q );
	}
}

static int MoverLookup( int convex, int pose )
{
	long long key = (long long)convex * 65536 + pose;
	std::vector<long long>::const_iterator it = std::lower_bound( s_MoverKey.begin(), s_MoverKey.end(), key );
	if ( it == s_MoverKey.end() || *it != key )
		return -1;
	return s_MoverIndex[it - s_MoverKey.begin()];
}

// x at pose p -> the same model point at pose q
static dvec3 PoseToPose( const DTransform &p, const DTransform &q, const dvec3 &x )
{
	dvec3 r = x - p.origin;
	// R^T r / scale
	dvec3 l( p.m[0][0] * r.x + p.m[1][0] * r.y + p.m[2][0] * r.z, p.m[0][1] * r.x + p.m[1][1] * r.y + p.m[2][1] * r.z,
			 p.m[0][2] * r.x + p.m[1][2] * r.y + p.m[2][2] * r.z );
	l = l * ( 1.0 / p.scale );
	return q.Apply( l );
}

// A hull of points that lie in a plane or on a line (a sliver's segments, or a
// sweep of them along their own plane) is thickened by this along each axis.
static const double EYE_FLAT_PAD = WALK_SHRINK;
// Ride sweeps of rotating movers: the swing is cut into equal steps, at most
// RIDE_MAX_STEPS, so that the chord between two steps' positions of the point
// farthest from the hinge strays at most RIDE_SAGITTA from its arc. The hull of
// the steps is then grown by the sagitta the steps actually leave (at most
// RIDE_SAGITTA unless the step cap binds), so it holds the whole swing.
static const double RIDE_SAGITTA = 1.0;
static const int RIDE_MAX_STEPS = 64;

// The hull of 'pts' as a piece, grown by 'pad' (every plane moved out by it,
// which holds every point within pad of the hull). A flat or collinear set is
// thickened by EYE_FLAT_PAD first. 'pts' may be extended.
static bool HullPiece( std::vector<dvec3> &pts, double pad, int source, EyePiece &out )
{
	if ( pts.empty() )
		return false;
	if ( !EyePolyFromPoints( pts, out ) )
	{
		size_t n = pts.size();
		for ( size_t i = 0; i < n; i++ )
		{
			pts.push_back( pts[i] + dvec3( EYE_FLAT_PAD, 0, 0 ) );
			pts.push_back( pts[i] + dvec3( 0, EYE_FLAT_PAD, 0 ) );
			pts.push_back( pts[i] + dvec3( 0, 0, EYE_FLAT_PAD ) );
		}
		if ( !EyePolyFromPoints( pts, out ) )
			return false;
	}
	if ( pad > 0 )
	{
		std::vector<DPlane> planes( out.planes );
		for ( size_t k = 0; k < planes.size(); k++ )
			planes[k].d += pad;
		EyePiece grown;
		if ( EyePolyFromPlanes( planes, grown ) )
			out = grown;
		else
		{
			// the hull's vertices moved to the corners of a cube of half size
			// pad: the cube holds the ball of radius pad
			std::vector<dvec3> cube;
			for ( size_t i = 0; i < out.verts.size(); i++ )
			{
				for ( int c = 0; c < 8; c++ )
					cube.push_back( out.verts[i] + dvec3( ( c & 1 ) ? pad : -pad, ( c & 2 ) ? pad : -pad, ( c & 4 ) ? pad : -pad ) );
			}
			if ( EyePolyFromPoints( cube, grown ) )
				out = grown;
		}
	}
	out.source = source;
	return true;
}

// The points whose hull holds 'verts' (at pose p, transform xp) carried with
// the mover through link (p, q) to pose q (xq): the two ends, and for a rotating
// mover the steps between them (MoverSweepPose, RIDE_SAGITTA). 'pad' gets how
// far the hull must be grown to hold the swing: 0 for a translation, whose sweep
// of a convex piece is exactly the hull of its two ends.
static void RideSweepPoints( const MoverEntity &me, int p, int q, const DTransform &xp, const DTransform &xq,
							 const std::vector<dvec3> &verts, std::vector<dvec3> &pts, double &pad )
{
	pts = verts;
	for ( size_t k = 0; k < verts.size(); k++ )
		pts.push_back( PoseToPose( xp, xq, verts[k] ) );
	pad = 0;
	if ( !me.rotating || p < 0 || q < 0 || p >= (int)me.poseAngles.size() || q >= (int)me.poseAngles.size() )
		return;
	// the hinge is on the axis, so no point is farther than this from the axis
	double rmax = 0;
	for ( size_t k = 0; k < verts.size(); k++ )
		rmax = MAX( rmax, DLength( verts[k] - me.hingeOrigin ) );
	// one angle changes, so the swing is one rotation by this much
	double swing = DLength( me.poseAngles[q] - me.poseAngles[p] ) * ( EYE_PI / 180.0 );
	if ( !( swing > 0 ) || !( rmax > 0 ) )
		return;
	int steps = 1;
	if ( rmax > RIDE_SAGITTA )
		steps = (int)ceil( swing / ( 2.0 * acos( 1.0 - RIDE_SAGITTA / rmax ) ) );
	steps = MAX( 1, MIN( steps, RIDE_MAX_STEPS ) );
	for ( int k = 1; k < steps; k++ )
	{
		DTransform xm = MoverSweepPose( me, p, q, (double)k / steps );
		for ( size_t i = 0; i < verts.size(); i++ )
			pts.push_back( PoseToPose( xp, xm, verts[i] ) );
	}
	// a vertex on its arc between two steps lies within r (1 - cos(step / 2))
	// of the chord between them; a piece point is a convex combination of the
	// vertices, so it lies within the largest of those of the hull
	double half = 0.5 * swing / steps;
	pad = half < 0.5 * EYE_PI ? rmax * ( 1.0 - cos( half ) ) : 2.0 * rmax;
}

static void RideSweep( EyeThread &t, const MoverEntity &me, int p, int q, const DTransform &xp, const DTransform &xq,
					   const std::vector<dvec3> &verts )
{
	std::vector<dvec3> pts;
	double pad;
	RideSweepPoints( me, p, q, xp, xq, verts, pts, pad );
	EyePiece sw;
	if ( HullPiece( pts, pad, EYESRC_RIDE, sw ) )
		EmitPiece( t, sw );
}

// A node's eye pieces, all collected in t.eye (the ride sweep moves each):
// - the crouch-feet column raised by eye_crouch;
// - with s_EyeStandLift, the stand-feet column raised by eye_stand;
// - the dead view, eye_dead up to the crouch eye: that prism lies
//   inside the crouch box on the floor, so it needs no obstacle clip.
static void NodeEyesOne( int iThread, int item )
{
	EyeThread &t = s_EyeThreads[iThread];
	const WalkGraph &g = *s_pEyeGraph;
	const PlayerProfile &prof = *s_pEyeProfile;
	int ni = s_EyeNodes[item];
	const WalkNode &node = g.nodes[ni];
	double rise = s_EyeRise[ni];
	double dead = MIN( prof.eyeDead, prof.eyeCrouch );
	std::vector<const GrownPolytope *> none;
	t.eye.clear();

	dvec3 mins = node.mins, maxs = node.maxs;
	maxs.z += rise;
	QueryObstacles( t, s_EyeCrouchObs, mins, maxs );
	t.pieces.clear();
	t.cappedColumns += FeetColumn( node.poly, node.plane, rise, t.obs, WALK_SHRINK, t.pieces );
	for ( size_t p = 0; p < t.pieces.size(); p++ )
		t.eye.push_back( Translated( t.pieces[p], dvec3( 0, 0, prof.eyeCrouch ) ) );

	double standRise = rise - s_EyeStandLift;
	if ( s_EyeStandLift > 0 && standRise > 0 )
	{
		maxs = node.maxs;
		maxs.z += standRise;
		QueryObstacles( t, s_EyeStandObs, mins, maxs );
		t.tmp.clear();
		t.cappedColumns += FeetColumn( node.poly, node.plane, standRise, t.obs, WALK_SHRINK, t.tmp );
		for ( size_t p = 0; p < t.tmp.size(); p++ )
			t.eye.push_back( Translated( t.tmp[p], dvec3( 0, 0, prof.eyeStand ) ) );
	}

	t.tmp.clear();
	if ( dead < prof.eyeCrouch )
		FeetColumn( node.poly, node.plane, prof.eyeCrouch - dead, none, WALK_SHRINK, t.tmp );
	for ( size_t p = 0; p < t.tmp.size(); p++ )
		t.eye.push_back( Translated( t.tmp[p], dvec3( 0, 0, dead ) ) );

	// No crouch column piece. Either (a) the prism did not build (a sliver
	// polygon: FeetColumn with no obstacles fails too), or (b) the obstacles
	// removed all of it (an underside within the build tolerance of the floor
	// all over the node).
	// (a): the hull of the polygon's vertical segments from the dead view up to
	// the column top, thickened when flat. It holds the corner segments, the
	// dead-view segments and the column over the sliver's edges, and it ignores
	// the obstacles (over-reach, the safe direction).
	// (b): the column's real eyes are the crouch eyes on the floor (there is no
	// room above them), and those are the dead-view prism's top face; a profile
	// with no dead-view prism (eye_crouch <= eye_dead) gets the crouch-eye
	// polygon instead.
	// Both cases also get the corner segments (below).
	if ( t.pieces.empty() )
	{
		t.tmp.clear();
		FeetColumn( node.poly, node.plane, rise, none, WALK_SHRINK, t.tmp );
		std::vector<dvec3> pts;
		bool sliver = t.tmp.empty();
		if ( sliver )
		{
			for ( size_t k = 0; k < node.poly.size(); k++ )
			{
				pts.push_back( node.poly[k] + dvec3( 0, 0, dead ) );
				pts.push_back( node.poly[k] + dvec3( 0, 0, prof.eyeCrouch + rise ) );
			}
		}
		else if ( !( dead < prof.eyeCrouch ) )
		{
			for ( size_t k = 0; k < node.poly.size(); k++ )
				pts.push_back( node.poly[k] + dvec3( 0, 0, prof.eyeCrouch ) );
		}
		EyePiece e;
		if ( HullPiece( pts, 0.0, EYESRC_NODE, e ) )
			t.eye.push_back( e );
		// Always also the corner segments, dead view to the top. A thin hull is
		// sampled only at its vertices (the grid misses it), so they are what puts
		// samples at every height along the column; and an empty column can also
		// come from a failed clip, not only from obstacles covering it all. They
		// only add eyes (over-reach, safe).
		for ( size_t k = 0; k < node.poly.size(); k++ )
			t.eye.push_back( SegmentPiece( node.poly[k] + dvec3( 0, 0, dead ),
										   node.poly[k] + dvec3( 0, 0, prof.eyeCrouch + rise ), EYESRC_NODE ) );
	}

	for ( size_t p = 0; p < t.eye.size(); p++ )
	{
		t.eye[p].source = EYESRC_NODE;
		EmitPiece( t, t.eye[p] );
	}

	// rides: every eye piece swept with its mover to every linked pose
	if ( node.category != CAT_TOPS_ONLY )
		return;
	const SourceConvex &sc = s_pEyeWorld->convexes[node.convex];
	int mi = MoverLookup( node.convex, node.pose );
	if ( mi < 0 )
		return;
	for ( size_t m = 0; m < g.moverEnts.size(); m++ )
	{
		const MoverEntity &me = g.moverEnts[m];
		if ( me.entity != sc.entityIndex )
			continue;
		for ( size_t l = 0; l < me.links.size(); l++ )
		{
			int other = -1;
			if ( me.links[l].first == node.pose )
				other = me.links[l].second;
			else if ( me.links[l].second == node.pose )
				other = me.links[l].first;
			int oi = other >= 0 ? MoverLookup( node.convex, other ) : -1;
			if ( oi < 0 )
				continue;
			const DTransform &xp = g.movers[mi].xf, &xq = g.movers[oi].xf;
			// The rider's feet move with the floor, but the eye stays world-up of
			// them. Sweeping an eye piece with the mover keeps it right only when the
			// world up does not tilt: a yaw-only change (yaw is the outermost Euler
			// rotation, about world z). A pitch or roll swing instead sweeps the floor
			// polygon and hangs the whole eye column (dead view to the top) world-up
			// off every swept point, ignoring obstacles (over-reach, safe).
			bool tilts = me.rotating && node.pose < (int)me.poseAngles.size() && other < (int)me.poseAngles.size() &&
						 ( fabs( me.poseAngles[other].x - me.poseAngles[node.pose].x ) > 1e-9 ||
						   fabs( me.poseAngles[other].z - me.poseAngles[node.pose].z ) > 1e-9 );
			if ( tilts )
			{
				// Only the feet rotate; the offset is a fixed world vector, so the
				// pad sized from the feet (their distance from the hinge) stays valid
				// after adding it: (hull + ball(pad)) + segment = hull(feet steps +
				// segment) + ball(pad). Do not sweep the raised points instead: that
				// rotates the offset, the very tilt this branch exists to avoid.
				std::vector<dvec3> feet, pts;
				double pad;
				RideSweepPoints( me, node.pose, other, xp, xq, node.poly, feet, pad );
				for ( size_t k = 0; k < feet.size(); k++ )
				{
					pts.push_back( feet[k] + dvec3( 0, 0, dead ) );
					pts.push_back( feet[k] + dvec3( 0, 0, prof.eyeCrouch + rise ) );
				}
				EyePiece sw;
				if ( HullPiece( pts, pad, EYESRC_RIDE, sw ) )
					EmitPiece( t, sw );
				continue;
			}
			for ( size_t p = 0; p < t.eye.size(); p++ )
				RideSweep( t, me, node.pose, other, xp, xq, t.eye[p].verts );
		}
	}
}

// Water, ladder-brush and teleport-arrival volumes: the hull of the volume
// raised by eye_dead, eye_crouch and eye_stand (a swimmer or climber may duck,
// or die there). Obstacles inside are not
// subtracted: an eye inside a lake bed only over-reaches (a solid leaf adds no
// cluster, and rays from inside an occluder are blocked at once), and
// subtracting every displacement triangle of a lake bed cut de_aztec's 12 water
// volumes into 293k pieces (120 s).
static void VolumeEyes( EyeThread &t, const GrownPolytope &vol, int source )
{
	const PlayerProfile &prof = *s_pEyeProfile;
	EyePiece base;
	std::vector<DPlane> planes = vol.planes;
	if ( !EyePolyFromPlanes( planes, base ) )
		return;
	std::vector<dvec3> pts;
	for ( size_t k = 0; k < base.verts.size(); k++ )
	{
		pts.push_back( base.verts[k] + dvec3( 0, 0, MIN( prof.eyeDead, prof.eyeCrouch ) ) );
		pts.push_back( base.verts[k] + dvec3( 0, 0, prof.eyeCrouch ) );
		pts.push_back( base.verts[k] + dvec3( 0, 0, prof.eyeStand ) );
	}
	EyePiece hull;
	if ( EyePolyFromPoints( pts, hull ) )
	{
		hull.source = source;
		EmitPiece( t, hull );
	}
}

static std::vector<int> s_EyeSpecial;	// reached special vertices with volumes

static void SpecialEyesOne( int iThread, int item )
{
	EyeThread &t = s_EyeThreads[iThread];
	const SpecialVertex &v = s_pEyeGraph->special.verts[s_EyeSpecial[item]];
	const PlayerProfile &prof = *s_pEyeProfile;
	if ( v.kind == SPECIAL_WATER )
	{
		for ( size_t k = 0; k < v.volumes.size(); k++ )
			VolumeEyes( t, v.volumes[k], EYESRC_WATER );
	}
	else if ( v.kind == SPECIAL_LADDER_BRUSH )
	{
		for ( size_t k = 0; k < v.volumes.size(); k++ )
			VolumeEyes( t, v.volumes[k], EYESRC_LADDER );
	}
	else if ( v.kind == SPECIAL_PUSH )
	{
		// a push can hold a player in the air (a lift balancing gravity): its
		// whole touch volume, not only the traced flights through it
		for ( size_t k = 0; k < v.volumes.size(); k++ )
			VolumeEyes( t, v.volumes[k], EYESRC_ARC );
	}
	else if ( v.kind == SPECIAL_ARRIVAL )
	{
		// a landmark arrival: anywhere in the shifted touch region
		for ( size_t k = 0; k < v.volumes.size(); k++ )
			VolumeEyes( t, v.volumes[k], EYESRC_TELEPORT );
	}
	else if ( v.kind == SPECIAL_LADDER )
	{
		// the climber stands on the segment; it may duck, or die there
		EmitPiece( t, SegmentPiece( v.a + dvec3( 0, 0, prof.eyeStand ), v.b + dvec3( 0, 0, prof.eyeStand ), EYESRC_LADDER ) );
		EmitPiece( t, SegmentPiece( v.a + dvec3( 0, 0, prof.eyeCrouch ), v.b + dvec3( 0, 0, prof.eyeCrouch ), EYESRC_LADDER ) );
		double dead = MIN( prof.eyeDead, prof.eyeCrouch );
		if ( dead < prof.eyeCrouch )
			EmitPiece( t, SegmentPiece( v.a + dvec3( 0, 0, dead ), v.b + dvec3( 0, 0, dead ), EYESRC_LADDER ) );
	}
}

// Traced free arcs: one segment per chord at the crouch eye (every arc is
// traced crouched). Ducking or unducking in the air moves the feet by the hull
// difference and leaves the eye where it is, so a flight's eye curve is fixed
// at launch: a duck-jump (traced lifted by crouch_lift) has the standing
// jump's eye curve, f + crouch_lift + eye_crouch; a crouched jump is kept only
// when the duck-jump is blocked, which blocks a standing jump too (its box
// holds the lifted crouch box). A walk-off (no launch speed, no kick or push)
// may start standing: its stand-eye curve is added. Where eye_stand - eye_crouch
// exceeds crouch_lift (tf: 23 against 20) a standing jump's eye is above the
// duck-jump's curve by the difference: other arcs get that curve too. The chords are the
// tracer's own: nSeg[0] over [0, tKick] (or [0, tEnd] without a kick),
// nSeg[1] over [tKick, tEnd], so each stays within its sagitta bound.
static float ArcZ( const BwArc &a, float g, float t )
{
	float g0 = g + a.dg0;
	if ( a.tKick > 0.0f && t > a.tKick )
	{
		float zk = a.oz + a.vz * a.tKick - 0.5f * g0 * a.tKick * a.tKick;
		float u = t - a.tKick;
		return zk + a.vzKick * u - 0.5f * g * u * u;
	}
	return a.oz + a.vz * t - 0.5f * g0 * t * t;
}

// A free arc's second eye curve (see above): a walk-off's standing eye, or the
// standing jump's eye gap above the crouch eye. Surf flights are eye paths
// already, one per eye (bw_arcs.cpp SurfFlights), and get none.
static bool ArcStandEye( const BwArc &a, const PlayerProfile &prof, double &z )
{
	bool kick = a.tKick > 0.0f && a.tKick < a.tEnd;
	double gap = prof.eyeStand - prof.eyeCrouch - prof.crouchLift;
	if ( a.vz == 0.0f && !kick && a.dg0 == 0.0f )
		z = prof.eyeStand;	// a walk-off may start standing
	else if ( gap > 0 )
		z = prof.eyeCrouch + gap;	// a standing jump's eye is above the lifted duck-jump's by the gap (tf 3u)
	else
		return false;
	return true;
}

// The tracer's chord end times: nSeg[0] over [0, tKick] (or [0, tEnd] without a
// kick), nSeg[1] over [tKick, tEnd]. Returns the chord count; tt[0] is 0.
static int ArcChordTimes( const BwArc &a, float tt[] )
{
	bool kick = a.tKick > 0.0f && a.tKick < a.tEnd;
	float t0[2] = { 0.0f, kick ? a.tKick : a.tEnd }, t1[2] = { kick ? a.tKick : a.tEnd, a.tEnd };
	int ns[2] = { MAX( 1, (int)a.nSeg[0] ), kick ? MAX( 1, (int)a.nSeg[1] ) : 0 };
	int n = 0;
	tt[0] = 0.0f;
	for ( int piece = 0; piece < 2; piece++ )
		for ( int k = 1; k <= ns[piece]; k++ )
			tt[++n] = k == ns[piece] ? t1[piece] : t0[piece] + ( t1[piece] - t0[piece] ) * (float)k / (float)ns[piece];
	return n;
}

// eyeZ: the eye height above the traced path (0 for a surf flight's eye path);
// standEye adds ArcStandEye's curve.
static void ArcChordEyes( EyeThread &t, const BwArc &a, double eyeZ, int source, bool standEye )
{
	float g = (float)s_pEyeProfile->gravity;
	const PlayerProfile &prof = *s_pEyeProfile;
	dvec3 eyes[2] = { dvec3( 0, 0, eyeZ ), dvec3( 0, 0, 0 ) };
	int nEyes = 1;
	double standZ;
	if ( standEye && ArcStandEye( a, prof, standZ ) )
	{
		eyes[1] = dvec3( 0, 0, standZ );
		nEyes = 2;
	}
	std::vector<float> &tt = t.arcTimes;
	tt.resize( (size_t)a.nSeg[0] + a.nSeg[1] + 3 );
	int n = ArcChordTimes( a, tt.data() );
	dvec3 prev( a.ox, a.oy, ArcZ( a, g, 0.0f ) );
	for ( int k = 1; k <= n; k++ )
	{
		dvec3 p( a.ox + a.vx * tt[k], a.oy + a.vy * tt[k], ArcZ( a, g, tt[k] ) );
		for ( int e = 0; e < nEyes; e++ )
			EmitPiece( t, SegmentPiece( prev + eyes[e], p + eyes[e], source ) );
		prev = p;
	}
}

static void ArcEyesOne( int iThread, int item )
{
	ArcChordEyes( s_EyeThreads[iThread], s_pEyeGraph->freeArcs[item], s_pEyeProfile->eyeCrouch, EYESRC_ARC, true );
}

// Surf flights (bw_arcs.cpp SurfFlights): eye paths already, at either eye.
static void SurfEyesOne( int iThread, int item )
{
	ArcChordEyes( s_EyeThreads[iThread], s_pEyeGraph->surfEyes[item], 0.0, EYESRC_SURF, false );
}

static std::vector<EyePiece> s_EyeLoose;	// points and segments built on the main thread
static const int MAX_VIEW_RAIL = 256;		// path corners followed from a point_viewcontrol's moveto

static void LooseEyesOne( int iThread, int item )
{
	EmitPiece( s_EyeThreads[iThread], s_EyeLoose[item] );
}

static void RunEyePass( int count, ThreadWorkerFn fn )
{
	RunThreadsBatched( count, fn );
}

// The cluster of each kept sample, how many samples each cluster holds, and the bound of
// them (item = a run of SAMPLE_CLUSTER_CHUNK samples; each thread fills its own per-cluster
// lists, merged by the caller).
static const size_t SAMPLE_CLUSTER_CHUNK = 1 << 16;

static void SampleClusterChunk( int iThread, int item )
{
	EyeThread &t = s_EyeThreads[iThread];
	const int nClusters = (int)t.clusters.size();
	if ( t.perCluster.empty() )
	{
		t.perCluster.assign( nClusters, 0 );
		t.sampleBound.assign( nClusters, EyeBound() );
	}
	std::vector<EyeSample> &samples = s_pEyeVol->samples;
	size_t end = MIN( ( (size_t)item + 1 ) * SAMPLE_CLUSTER_CHUNK, samples.size() );
	for ( size_t i = (size_t)item * SAMPLE_CLUSTER_CHUNK; i < end; i++ )
	{
		EyeSample &s = samples[i];
		dvec3 at( s.x, s.y, s.z );
		int c = s.cluster = dleafs[PointLeaf( at )].cluster;
		if ( c >= 0 && c < nClusters )
		{
			t.perCluster[c]++;
			t.sampleBound[c].Add( at );
		}
	}
}

//-----------------------------------------------------------------------------
// Fall columns. A player leaving a ledge (or a ladder, a seed or a teleport
// arrival) falls past every lower node within the XY distance it drifts
// before it lands, not only the one straight below. For a launch at feet
// height 'top', the crouch-feet height over a node at XY distance d is at most
// top + off + max over t' >= t of (vJ t' - g t'^2 / 2), with t = (d - half_width) / v
// the earliest it can get there (the jump apex while t is before it): v is max_ground_speed x reach_margin (the walk-off speed; the
// arc pass covers faster flights), vJ the jump speed and off the duck lift or
// the stand-to-crouch eye difference (baseRise = jump apex + off). With air
// jumps the jump part does not decay (it can be renewed in the air).
//-----------------------------------------------------------------------------
struct FallLaunch
{
	XYBox box;
	double top;			// feet height
};

static std::vector<FallLaunch> s_FallLaunch;
// Launches grouped by feet height in FALL_BAND bands, each with its own hash:
// a node queries a band with the reach of a fall from that band's highest
// launch down to it, not the map's tallest drop.
static const double FALL_BAND = 256.0;
struct FallBand
{
	CXYHash hash;
	std::vector<int> ids;	// band object -> s_FallLaunch index
	double maxTop = -1e300;
};
static std::vector<FallBand> s_FallBands;
static double s_FallOff, s_FallVJ, s_FallV;
static double s_FallExtra;		// air jumps: more time aloft

static double FallTimeTo( double drop )
{
	// time for a jump at vJ to come down 'drop' below its start
	double g = s_pEyeProfile->gravity;
	return ( s_FallVJ + sqrt( s_FallVJ * s_FallVJ + 2.0 * g * MAX( drop, 0.0 ) ) ) / g + s_FallExtra;
}

static void FallEnvelopeOne( int iThread, int item )
{
	EyeThread &t = s_EyeThreads[iThread];
	const PlayerProfile &prof = *s_pEyeProfile;
	int ni = s_EyeNodes[item];
	const WalkNode &B = s_pEyeGraph->nodes[ni];
	double g = prof.gravity, rise = s_EyeRise[ni];
	double J = prof.jumpApex * ( 1 + prof.airJumps );
	// Only the maximum matters, so the work is pruned without changing it. A
	// launch raises B past 'rise' only while its feet are above B.z + rise - off
	// (the floor of what it can still add): its top must be above that less J,
	// and its flight must reach B before it falls below it. The bands go from
	// the highest down (a band's tops are all under the next one's), so the
	// rise found grows early and the rest stop at the first band too low.
	for ( int bi = (int)s_FallBands.size() - 1; bi >= 0; bi-- )
	{
		const FallBand &band = s_FallBands[bi];
		if ( band.ids.empty() )
			continue;	// no maxTop: it must not end the scan
		double floorZ = B.mins.z + rise - s_FallOff;
		if ( band.maxTop <= B.mins.z || band.maxTop <= floorZ - J )
			break;
		// with air jumps the gain does not decay: only landing ends it
		double cutZ = prof.airJumps > 0 ? B.mins.z : floorZ;
		double R = prof.halfWidth + s_FallV * FallTimeTo( band.maxTop - cutZ );
		t.cand.clear();
		band.hash.Query( B.mins.x - R, B.mins.y - R, B.maxs.x + R, B.maxs.y + R, t.cand );
		for ( size_t k = 0; k < t.cand.size(); k++ )
		{
			const FallLaunch &L = s_FallLaunch[band.ids[t.cand[k]]];
			floorZ = B.mins.z + rise - s_FallOff;
			cutZ = prof.airJumps > 0 ? B.mins.z : floorZ;
			if ( L.top <= B.mins.z || L.top <= floorZ - J )
				continue;
			double dx = MAX( 0.0, MAX( L.box.x0 - B.maxs.x, B.mins.x - L.box.x1 ) );
			double dy = MAX( 0.0, MAX( L.box.y0 - B.maxs.y, B.mins.y - L.box.y1 ) );
			double tt = MAX( 0.0, sqrt( dx * dx + dy * dy ) - prof.halfWidth ) / s_FallV;
			if ( tt > FallTimeTo( L.top - cutZ ) )
				continue;	// below what B already has (or landed) before it got there
			double jump = ( prof.airJumps > 0 || tt <= s_FallVJ / g ) ? J : s_FallVJ * tt - 0.5 * g * tt * tt;
			rise = MAX( rise, L.top + s_FallOff + jump - B.mins.z );
		}
	}
	s_EyeRise[ni] = rise;
}

static void AddFallLaunch( double x0, double y0, double x1, double y1, double top )
{
	FallLaunch L;
	L.box.x0 = x0;
	L.box.y0 = y0;
	L.box.x1 = x1;
	L.box.y1 = y1;
	L.top = top;
	s_FallLaunch.push_back( L );
}

// Raises s_EyeRise over every reached node a fall can pass over (s_EyeNodes set).
static void BuildFallColumns( const PlayerProfile &profile, const WalkGraph &graph, double baseRise )
{
	s_FallLaunch.clear();
	int n = (int)graph.nodes.size();
	std::vector<unsigned char> rim( n, 0 );
	for ( size_t i = 0; i < graph.edges.size(); i++ )
	{
		const WalkEdge &e = graph.edges[i];
		// any descent: a jump off a step edge (a walk or slide edge down) drifts
		// over the lower floor at the launch height too
		if ( e.kind != EDGE_JUMP && graph.reached[e.a] && graph.reached[e.b] &&
			 graph.nodes[e.b].mins.z < graph.nodes[e.a].maxs.z )
			rim[e.a] = 1;
		// a walk edge is stored once and runs both ways
		if ( e.kind == EDGE_WALK && graph.reached[e.a] && graph.reached[e.b] &&
			 graph.nodes[e.a].mins.z < graph.nodes[e.b].maxs.z )
			rim[e.b] = 1;
	}
	for ( int i = 0; i < n; i++ )
	{
		if ( rim[i] )
		{
			const WalkNode &A = graph.nodes[i];
			AddFallLaunch( A.mins.x, A.mins.y, A.maxs.x, A.maxs.y, A.maxs.z );
		}
	}
	for ( size_t i = 0; i < graph.seeds.size(); i++ )
	{
		const dvec3 &p = graph.seeds[i].point;
		AddFallLaunch( p.x, p.y, p.x, p.y, p.z );
	}
	const SpecialGraph &sg = graph.special;
	for ( size_t i = 0; i < sg.teleports.size(); i++ )
	{
		const TeleportReport &tr = sg.teleports[i];
		if ( tr.vertex < 0 || tr.vertex >= (int)sg.reached.size() || !sg.reached[tr.vertex] )
			continue;
		for ( size_t d = 0; d < tr.dests.size(); d++ )
		{
			const dvec3 &p = tr.dests[d].point;
			AddFallLaunch( p.x, p.y, p.x, p.y, p.z );
		}
	}
	for ( size_t v = 0; v < sg.verts.size(); v++ )
	{
		const SpecialVertex &sv = sg.verts[v];
		if ( sv.kind != SPECIAL_LADDER || v >= sg.reached.size() || !sg.reached[v] )
			continue;
		// a climber may let go anywhere on the ladder
		AddFallLaunch( MIN( sv.a.x, sv.b.x ), MIN( sv.a.y, sv.b.y ), MAX( sv.a.x, sv.b.x ), MAX( sv.a.y, sv.b.y ),
					   MAX( sv.a.z, sv.b.z ) );
	}
	for ( size_t v = 0; v < sg.verts.size(); v++ )
	{
		// a func_ladder climber may let go anywhere in its touch volume
		const SpecialVertex &sv = sg.verts[v];
		if ( sv.kind != SPECIAL_LADDER_BRUSH || v >= sg.reached.size() || !sg.reached[v] )
			continue;
		for ( size_t k = 0; k < sv.volumes.size(); k++ )
		{
			const GrownPolytope &gp = sv.volumes[k];
			if ( gp.IsValid() )
				AddFallLaunch( gp.mins.x, gp.mins.y, gp.maxs.x, gp.maxs.y, gp.maxs.z );
		}
	}
	if ( s_FallLaunch.empty() || s_EyeNodes.empty() )
		return;

	double J = profile.jumpApex * ( 1 + profile.airJumps );
	s_FallOff = MAX( 0.0, baseRise - J );
	s_FallVJ = sqrt( 2.0 * profile.gravity * profile.jumpApex );
	s_FallV = MAX( 1.0, profile.maxGroundSpeed * profile.reachMargin );
	// an air jump renews the climb: give its flight that much more time
	s_FallExtra = profile.airJumps * 2.0 * s_FallVJ / profile.gravity;
	double topMin = 1e300, topMax = -1e300;
	for ( size_t k = 0; k < s_FallLaunch.size(); k++ )
	{
		topMin = MIN( topMin, s_FallLaunch[k].top );
		topMax = MAX( topMax, s_FallLaunch[k].top );
	}
	int nBands = MIN( 4096, 1 + (int)floor( ( topMax - topMin ) / FALL_BAND ) );
	s_FallBands.assign( nBands, FallBand() );
	std::vector< std::vector<XYBox> > boxes( nBands );
	for ( size_t k = 0; k < s_FallLaunch.size(); k++ )
	{
		int b = MIN( nBands - 1, (int)floor( ( s_FallLaunch[k].top - topMin ) / FALL_BAND ) );
		FallBand &band = s_FallBands[b];
		band.maxTop = band.ids.empty() ? s_FallLaunch[k].top : MAX( band.maxTop, s_FallLaunch[k].top );
		band.ids.push_back( (int)k );
		boxes[b].push_back( s_FallLaunch[k].box );
	}
	for ( int b = 0; b < nBands; b++ )
		s_FallBands[b].hash.Build( boxes[b], std::vector<unsigned char>( boxes[b].size(), 1 ) );
	RunThreadsBatched( (int)s_EyeNodes.size(), FallEnvelopeOne );
	std::vector<FallBand>().swap( s_FallBands );
	std::vector<FallLaunch>().swap( s_FallLaunch );
}

// The cap on the estimate below. The memory guard: an estimated sample costs
// about EYE_GUARD_BYTES at the peak (24 B a sample; the samples made before
// deduplication run up to 1.47x the estimate, and the vis pass adds a 4 B index
// each; d1_trainstation_02 peaked at about 31 B per estimated sample), and the
// estimate may use EYE_GUARD_FRACTION of the memory free when the eyes are
// built (the vis pass and the rest of the run need the remainder). bw_vis
// indexes the kept samples with int, so the estimate also stays under
// EYE_INDEX_CAP (kept / estimate: 0.59 to 0.97 on the fixtures, 0.85 on d1).
// Widening stops when a 1.25x step cuts the estimate by less than
// EYE_STALL_CUT: what is left is per-piece vertices and points, and further
// steps would only coarsen tier B (fightspace3 once ran to 4136u that way).
// When the free memory cannot be read the guard has nothing to go on, and the
// index cap alone would allow about 58 GB (1.2e9 samples at 48 B), so the cap
// is a fixed EYE_UNKNOWN_MEMORY_CAP instead (about 12 GB at the guard's rate).
static const double EYE_GUARD_BYTES = 48.0;
static const double EYE_GUARD_FRACTION = 0.5;
static const double EYE_INDEX_CAP = 1.2e9;
static const double EYE_UNKNOWN_MEMORY_CAP = 256e6;
static const double EYE_STALL_CUT = 0.05;

// The -eyemaxsamples estimate, over every source. SamplePiece gives a poly
// piece about volume / s^3 + 2 xy-area / s^2 samples (a grid line per s^2 of
// its xy shadow, with a point every s up it and one at each end) plus its
// vertices, a segment length / s + 2, a point 1. Each skybox copy repeats them
// (it is 1/scale the size, sampled 1/scale as finely).
struct EyeBudget
{
	double vol, axy, len, pts;
	EyeBudget() : vol( 0 ), axy( 0 ), len( 0 ), pts( 0 ) {}
	double At( double s ) const { return vol / ( s * s * s ) + 2.0 * axy / ( s * s ) + len / s + pts; }
	void Segment( const dvec3 &a, const dvec3 &b )
	{
		len += DLength( b - a );
		pts += 2;
	}
};

// A hull piece's share of the budget.
static void BudgetHull( const std::vector<dvec3> &pts, EyeBudget &b )
{
	ConvexShape c;
	if ( pts.size() < 4 || !ConvexFromPoints( pts.data(), (int)pts.size(), c ) )
		return;
	b.vol += fabs( PolytopeVolume( c.verts, c.faces ) );
	// the xy shadow of a convex: half the summed |z| of its faces' area vectors
	double az = 0;
	for ( size_t f = 0; f < c.faces.size(); f++ )
	{
		const std::vector<int> &face = c.faces[f];
		double z = 0;
		for ( size_t i = 0; i < face.size(); i++ )
		{
			const dvec3 &p = c.verts[face[i]], &q = c.verts[face[( i + 1 ) % face.size()]];
			z += p.x * q.y - q.x * p.y;
		}
		az += fabs( 0.5 * z );
	}
	b.axy += 0.5 * az;
	b.pts += (double)c.verts.size();
}

// Every source the passes below sample: node columns (crouch, standing with
// s_EyeStandLift, dead view), ride sweeps (the box of a node's eye pieces swept
// along each link), water and ladder volumes, ladder segments, arc chords,
// teleport segments and viewpoints. Needs s_EyeNodes, s_EyeRise,
// s_EyeStandLift, s_EyeSpecial, s_EyeLoose and the mover keys.
static void BudgetEyes( const WalkGraph &g, const PlayerProfile &prof, EyeBudget &b )
{
	double dead = MIN( prof.eyeDead, prof.eyeCrouch );
	std::vector<dvec3> corners, pts;
	for ( size_t k = 0; k < s_EyeNodes.size(); k++ )
	{
		int ni = s_EyeNodes[k];
		const WalkNode &node = g.nodes[ni];
		double rise = s_EyeRise[ni];
		double h[3] = { rise, s_EyeStandLift > 0 ? rise - s_EyeStandLift : 0.0, prof.eyeCrouch - dead };
		for ( int c = 0; c < 3; c++ )
		{
			if ( !( h[c] > 0 ) )
				continue;
			b.vol += node.area * h[c];
			b.axy += node.area;
			b.pts += 2.0 * node.poly.size();
		}
		if ( node.category != CAT_TOPS_ONLY )
			continue;
		int mi = MoverLookup( node.convex, node.pose );
		if ( mi < 0 )
			continue;
		const SourceConvex &sc = s_pEyeWorld->convexes[node.convex];
		corners.clear();
		for ( int c = 0; c < 8; c++ )
			corners.push_back( dvec3( ( c & 1 ) ? node.maxs.x : node.mins.x, ( c & 2 ) ? node.maxs.y : node.mins.y,
									  ( c & 4 ) ? node.maxs.z + prof.eyeCrouch + rise : node.mins.z + dead ) );
		for ( size_t m = 0; m < g.moverEnts.size(); m++ )
		{
			const MoverEntity &me = g.moverEnts[m];
			if ( me.entity != sc.entityIndex )
				continue;
			for ( size_t l = 0; l < me.links.size(); l++ )
			{
				int other = -1;
				if ( me.links[l].first == node.pose )
					other = me.links[l].second;
				else if ( me.links[l].second == node.pose )
					other = me.links[l].first;
				int oi = other >= 0 ? MoverLookup( node.convex, other ) : -1;
				if ( oi < 0 )
					continue;
				double pad;
				RideSweepPoints( me, node.pose, other, g.movers[mi].xf, g.movers[oi].xf, corners, pts, pad );
				BudgetHull( pts, b );
			}
		}
	}

	const SpecialGraph &sg = g.special;
	for ( size_t i = 0; i < s_EyeSpecial.size(); i++ )
	{
		const SpecialVertex &v = sg.verts[s_EyeSpecial[i]];
		if ( v.kind == SPECIAL_LADDER )
		{
			b.Segment( v.a + dvec3( 0, 0, prof.eyeStand ), v.b + dvec3( 0, 0, prof.eyeStand ) );
			b.Segment( v.a + dvec3( 0, 0, prof.eyeCrouch ), v.b + dvec3( 0, 0, prof.eyeCrouch ) );
			if ( dead < prof.eyeCrouch )
				b.Segment( v.a + dvec3( 0, 0, dead ), v.b + dvec3( 0, 0, dead ) );
			continue;
		}
		for ( size_t k = 0; k < v.volumes.size(); k++ )
		{
			const GrownPolytope &vol = v.volumes[k];
			pts.clear();
			for ( size_t j = 0; j < vol.verts.size(); j++ )
			{
				pts.push_back( vol.verts[j] + dvec3( 0, 0, dead ) );
				pts.push_back( vol.verts[j] + dvec3( 0, 0, prof.eyeCrouch ) );
				pts.push_back( vol.verts[j] + dvec3( 0, 0, prof.eyeStand ) );
			}
			BudgetHull( pts, b );
		}
	}

	float grav = (float)prof.gravity;
	// free arcs and surf-flight eye paths (bw_arcs.cpp SurfFlights), chord for
	// chord as ArcChordEyes emits them
	std::vector<float> tt;
	for ( size_t i = 0; i < g.freeArcs.size() + g.surfEyes.size(); i++ )
	{
		bool isFree = i < g.freeArcs.size();
		const BwArc &a = isFree ? g.freeArcs[i] : g.surfEyes[i - g.freeArcs.size()];
		double standZ;
		int curves = isFree && ArcStandEye( a, prof, standZ ) ? 2 : 1;
		tt.resize( (size_t)a.nSeg[0] + a.nSeg[1] + 3 );
		int n = ArcChordTimes( a, tt.data() );
		dvec3 prev( a.ox, a.oy, ArcZ( a, grav, 0.0f ) );
		for ( int k = 1; k <= n; k++ )
		{
			dvec3 p( a.ox + a.vx * tt[k], a.oy + a.vy * tt[k], ArcZ( a, grav, tt[k] ) );
			for ( int c = 0; c < curves; c++ )
				b.Segment( prev, p );
			prev = p;
		}
	}

	for ( size_t i = 0; i < s_EyeLoose.size(); i++ )
	{
		const EyePiece &p = s_EyeLoose[i];
		if ( p.kind == EYE_SEGMENT )
			b.Segment( p.verts[0], p.verts[1] );
		else
			b.pts += 1;
	}
}

void BuildEyeVolume( const CollisionWorld &world, const PlayerProfile &profile, const WalkGraph &graph,
					 const EyeOptions &opts, EyeVolume &eyes )
{
	double t0 = Plat_FloatTime();
	eyes = EyeVolume();
	eyes.numClusters = ( visdatasize > 0 && dvis ) ? dvis->numclusters : 0;
	// without vis data the clusters still exist (leaf.cluster), sized from the leaves
	int nClusters = eyes.numClusters;
	if ( !nClusters )
	{
		for ( int i = 0; i < numleafs; i++ )
			nClusters = MAX( nClusters, dleafs[i].cluster + 1 );
	}

	// sky_camera: the 3D skybox is drawn from origin + eye / scale. Every one
	// counts (some maps switch cameras by area or at run time). The engine
	// reads scale as an integer (FIELD_INTEGER) and divides by it only when it
	// is positive: a missing, empty, zero or negative scale draws the skybox
	// 1:1 from origin + eye, so that is 1 here. The camera is kept either way
	// (the engine still draws the sky pass).
	for ( int e = 0; e < num_entities; e++ )
	{
		if ( Q_stricmp( EntValue( &entities[e], "classname" ), "sky_camera" ) )
			continue;
		const char *pszScale = EntValue( &entities[e], "scale" );
		double scale = atoi( pszScale );
		if ( !( scale > 0 ) )
		{
			dvec3 o = EntOrigin( &entities[e] );
			Warning( "bspwalk: sky_camera at (%.1f %.1f %.1f) has scale \"%s\": the engine draws its skybox 1:1, so "
					 "scale 1 is used\n", o.x, o.y, o.z, pszScale );
			scale = 1.0;
		}
		if ( !eyes.sky )
		{
			eyes.skyOrigin = EntOrigin( &entities[e] );
			eyes.skyScale = scale;
		}
		eyes.sky = true;
		eyes.skyOrigins.push_back( EntOrigin( &entities[e] ) );
		eyes.skyScales.push_back( scale );
	}

	s_pEyeWorld = &world;
	s_pEyeProfile = &profile;
	s_pEyeGraph = &graph;
	s_pEyeOpts = &opts;
	s_pEyeVol = &eyes;
	s_EyeCrouchObs.Build( world, world.crouch );
	// a standing eye is a crouch eye of the crouch column only while
	// eye_stand - eye_crouch <= stand_height - crouch_height (NodeEyesOne);
	// past that (tf: 23 > 20) standing eyes get their own column
	s_bStandColumn = profile.eyeStand - profile.eyeCrouch > profile.standHeight - profile.crouchHeight + 1e-9;
	if ( s_bStandColumn )
	{
		s_EyeStandObs.Build( world, world.stand );
		Msg( "eyes: eye_stand - eye_crouch (%g) exceeds stand_height - crouch_height (%g): standing eyes get their own "
			 "column\n", profile.eyeStand - profile.eyeCrouch, profile.standHeight - profile.crouchHeight );
	}
	for ( int i = 0; i <= MAX_TOOL_THREADS; i++ )
	{
		EyeThread &t = s_EyeThreads[i];
		t.clusters.assign( nClusters, 0 );
		t.region.clear();
		t.samples.clear();
		t.keep.clear();
		memset( t.pieces_, 0, sizeof( t.pieces_ ) );
		t.mapped = 0;
		t.solidViews.clear();
		t.cappedColumns = 0;
		t.all = EyeBound();
		t.sampleBound.clear();
		t.perCluster.clear();
	}

	// movers: (convex, pose) -> index
	std::vector< std::pair<long long, int> > mk;
	for ( size_t i = 0; i < graph.movers.size(); i++ )
		mk.push_back( std::make_pair( (long long)graph.movers[i].convex * 65536 + graph.movers[i].pose, (int)i ) );
	std::sort( mk.begin(), mk.end() );
	s_MoverKey.resize( mk.size() );
	s_MoverIndex.resize( mk.size() );
	for ( size_t i = 0; i < mk.size(); i++ )
	{
		s_MoverKey[i] = mk[i].first;
		s_MoverIndex[i] = mk[i].second;
	}

	// node columns: the crouch-feet rise is max_jump_rise, or the rise that puts
	// the crouch eye at a standing jump's eye when that is higher:
	// eye top = feet + eye_crouch + max(max_jump_rise, jump_apex (1 + air_jumps)
	// + eye_stand - eye_crouch). An in-air duck (feet up crouch_lift, eye at
	// eye_crouch) is a crouch box at most max_jump_rise up; a standing eye at
	// the (last air jump's) apex is the second term. Raised over every node a
	// fall can pass over (BuildFallColumns).
	int n = (int)graph.nodes.size();
	double baseRise = MAX( profile.maxJumpRise,
						   profile.jumpApex * ( 1 + profile.airJumps ) + profile.eyeStand - profile.eyeCrouch );
	s_EyeStandLift = 0;
	if ( profile.eyeStand - profile.eyeCrouch > profile.standHeight - profile.crouchHeight + 1e-9 )
		s_EyeStandLift = profile.eyeStand - profile.eyeCrouch;
	s_EyeRise.assign( n, baseRise );
	s_EyeNodes.clear();
	for ( int i = 0; i < n; i++ )
	{
		if ( graph.reached[i] )
			s_EyeNodes.push_back( i );
	}
	double tFall = Plat_FloatTime();
	BuildFallColumns( profile, graph, baseRise );
	int raised = 0;
	for ( size_t k = 0; k < s_EyeNodes.size(); k++ )
		raised += s_EyeRise[s_EyeNodes[k]] > baseRise;
	Msg( "eyes: fall columns raise %d of %d reached nodes (%.2f s)\n", raised, (int)s_EyeNodes.size(),
		 Plat_FloatTime() - tFall );

	// water and ladders
	s_EyeSpecial.clear();
	const SpecialGraph &sg = graph.special;
	for ( size_t v = 0; v < sg.verts.size(); v++ )
	{
		int k = sg.verts[v].kind;
		if ( v < sg.reached.size() && sg.reached[v] &&
			 ( k == SPECIAL_WATER || k == SPECIAL_LADDER || k == SPECIAL_LADDER_BRUSH || k == SPECIAL_PUSH ||
			   ( k == SPECIAL_ARRIVAL && !sg.verts[v].volumes.empty() ) ) )
			s_EyeSpecial.push_back( (int)v );
	}

	// teleport destinations and the extra viewpoints
	s_EyeLoose.clear();
	for ( size_t i = 0; i < sg.teleports.size(); i++ )
	{
		const TeleportReport &tr = sg.teleports[i];
		if ( tr.vertex < 0 || tr.vertex >= (int)sg.reached.size() || !sg.reached[tr.vertex] )
			continue;
		for ( size_t d = 0; d < tr.dests.size(); d++ )
		{
			const dvec3 &p = tr.dests[d].point;
			s_EyeLoose.push_back( SegmentPiece( p + dvec3( 0, 0, MIN( profile.eyeDead, profile.eyeCrouch ) ),
												p + dvec3( 0, 0, profile.eyeStand ), EYESRC_TELEPORT ) );
		}
	}
	// viewpoints: each camera gets an id (EyePiece::viewpoint), shared by a
	// point_viewcontrol's rail pieces (part 1 on); part 0 starts a new camera
	int nView = 0;
	auto pushView = [&]( EyePiece piece, int part ) {
		piece.viewpoint = part ? nView - 1 : nView++;
		piece.viewPart = part;
		s_EyeLoose.push_back( piece );
	};
	for ( int i = 0; i < g_nCubemapSamples; i++ )
	{
		const dcubemapsample_t &c = g_CubemapSamples[i];
		pushView( PointPiece( dvec3( c.origin[0], c.origin[1], c.origin[2] ), EYESRC_CUBEMAP ), 0 );
	}
	for ( int e = 0; e < num_entities; e++ )
	{
		const char *cls = EntValue( &entities[e], "classname" );
		if ( !Q_stricmp( cls, "point_camera" ) )
			pushView( PointPiece( EntOrigin( &entities[e] ), EYESRC_CAMERA ), 0 );
		else if ( !Q_stricmp( cls, "point_viewcontrol" ) )
		{
			// the camera, and its moveto rail: straight from its origin to the
			// moveto corner, then corner to corner along the target links
			dvec3 prev = EntOrigin( &entities[e] );
			pushView( PointPiece( prev, EYESRC_VIEWCONTROL ), 0 );
			int part = 0;
			const char *moveto = EntValue( &entities[e], "moveto" );
			std::vector<int> seen;
			for ( int c = moveto[0] ? FindEntityByTargetname( moveto, -1 ) : -1;
				  c >= 0 && (int)seen.size() < MAX_VIEW_RAIL; c = FindEntityByTargetname( EntValue( &entities[c], "target" ), -1 ) )
			{
				const char *pc = EntValue( &entities[c], "classname" );
				if ( ( Q_stricmp( pc, "path_corner" ) && Q_stricmp( pc, "path_track" ) ) ||
					 std::find( seen.begin(), seen.end(), c ) != seen.end() )
					break;
				seen.push_back( c );
				dvec3 p = EntOrigin( &entities[c] );
				pushView( SegmentPiece( prev, p, EYESRC_VIEWCONTROL ), ++part );
				prev = p;
			}
		}
	}
	for ( size_t i = 0; i < opts.viewpoints.size(); i++ )
		pushView( PointPiece( opts.viewpoints[i], EYESRC_CLI ), 0 );
	for ( size_t k = 0; k < eyes.skyOrigins.size(); k++ )
		pushView( PointPiece( eyes.skyOrigins[k], EYESRC_SKYCAMERA ), 0 );
	for ( size_t i = 0; i < s_EyeLoose.size(); i++ )
	{
		int s = s_EyeLoose[i].source;
		eyes.viewpoints += IsViewpointSource( s );
	}

	// sample spacing: the requested one, widened by 1.25x steps only while the
	// estimated sample count of every source (EyeBudget) is over the cap: the
	// int index limit, and -eyemaxsamples when given, else the memory guard
	EyeBudget budget;
	BudgetEyes( graph, profile, budget );
	double skyCopies = 1.0 + eyes.skyOrigins.size();
	double freeBytes = FreeMemoryBytes();
	double cap = EYE_INDEX_CAP;
	const char *capReason = "index";
	if ( opts.maxSamples > 0 )
	{
		if ( opts.maxSamples < cap )
		{
			cap = opts.maxSamples;
			capReason = "eyemaxsamples";
		}
	}
	else if ( freeBytes <= 0 )
	{
		// FreeMemoryBytes returns 0 when it cannot read the memory status
		Warning( "bspwalk: could not read free memory; using a fixed cap of %.0f samples (-eyemaxsamples <n> sets it)\n",
				 EYE_UNKNOWN_MEMORY_CAP );
		if ( EYE_UNKNOWN_MEMORY_CAP < cap )
		{
			cap = EYE_UNKNOWN_MEMORY_CAP;
			capReason = "memory_unknown";
		}
	}
	else if ( freeBytes * EYE_GUARD_FRACTION / EYE_GUARD_BYTES < cap )
	{
		cap = freeBytes * EYE_GUARD_FRACTION / EYE_GUARD_BYTES;
		capReason = "memory";
	}
	s_EyeSpacing = opts.spacing;
	double est = budget.At( s_EyeSpacing ) * skyCopies;
	const double estAsked = est;
	bool stalled = false;
	while ( est > cap && s_EyeSpacing <= 4096 )
	{
		// a step that no longer cuts the estimate meaningfully means the rest is
		// per-piece vertices and points: widening further only coarsens tier B
		double next = budget.At( s_EyeSpacing * 1.25 ) * skyCopies;
		if ( next > est * ( 1.0 - EYE_STALL_CUT ) )
		{
			stalled = true;
			break;
		}
		s_EyeSpacing *= 1.25;
		est = next;
	}
	eyes.spacing = s_EyeSpacing;
	eyes.spacingRequested = opts.spacing;
	eyes.widened = s_EyeSpacing > opts.spacing;
	eyes.capReason = capReason;
	eyes.sampleCap = cap;
	eyes.estimate = est;
	eyes.capMet = est <= cap;
	const char *capWhat = !strcmp( capReason, "eyemaxsamples" )		? "-eyemaxsamples"
						: !strcmp( capReason, "memory" )			? "the memory guard"
						: !strcmp( capReason, "memory_unknown" )	? "the fixed cap used when free memory is unreadable"
																	: "the int sample index limit";
	if ( eyes.widened )
	{
		Warning( "bspwalk: TIER B IS SAMPLED COARSER THAN ASKED: eye samples %.2fu apart, not the %gu asked (-eyespacing). "
				 "At %gu the estimate is %.0f samples, over the cap of %.0f set by %s", s_EyeSpacing, opts.spacing,
				 opts.spacing, estAsked, cap, capWhat );
		if ( !strcmp( capReason, "memory" ) )
			Warning( " (%.0f%% of the %.1f GB free at %.0f B a sample)", EYE_GUARD_FRACTION * 100, freeBytes / 1e9,
					 EYE_GUARD_BYTES );
		Warning( ". A face seen only from between these samples can be classed tier B hidden%s\n",
				 !strncmp( capReason, "memory", 6 ) ? "; -eyemaxsamples <n> replaces the memory guard" : "" );
	}
	if ( !eyes.capMet )
	{
		Warning( "bspwalk: the eye sample cap (%.0f, %s) is not met: about %.0f samples at %.2fu, %.0f of them piece "
				 "vertices and points, which no spacing removes; %s\n", cap, capWhat, est, s_EyeSpacing,
				 budget.pts * skyCopies,
				 stalled ? "widening stopped where a 1.25x step no longer cut the estimate meaningfully"
						 : "widening stopped past 4096u" );
	}

	RunEyePass( (int)s_EyeNodes.size(), NodeEyesOne );
	RunEyePass( (int)s_EyeSpecial.size(), SpecialEyesOne );
	RunEyePass( (int)graph.freeArcs.size(), ArcEyesOne );
	RunEyePass( (int)graph.surfEyes.size(), SurfEyesOne );
	RunEyePass( (int)s_EyeLoose.size(), LooseEyesOne );

	// gather
	std::vector<SolidViewHit> solidHits;
	int capped = 0;
	eyes.clusters.assign( nClusters, 0 );
	size_t nSamples = 0;
	for ( int i = 0; i <= MAX_TOOL_THREADS; i++ )
		nSamples += s_EyeThreads[i].samples.size();
	eyes.samples.reserve( nSamples );
	for ( int i = 0; i <= MAX_TOOL_THREADS; i++ )
	{
		EyeThread &t = s_EyeThreads[i];
		for ( int c = 0; c < nClusters; c++ )
			eyes.clusters[c] |= t.clusters[c];
		eyes.samples.insert( eyes.samples.end(), t.samples.begin(), t.samples.end() );
		if ( opts.keepPieces )
			eyes.keep.insert( eyes.keep.end(), t.keep.begin(), t.keep.end() );
		for ( int s = 0; s < EYESRC_COUNT; s++ )
			eyes.pieces[s] += t.pieces_[s];
		eyes.mappedPieces += t.mapped;
		solidHits.insert( solidHits.end(), t.solidViews.begin(), t.solidViews.end() );
		capped += t.cappedColumns;
		eyes.all.Merge( t.all );
		std::vector<EyeSample>().swap( t.samples );
		std::vector<EyePiece>().swap( t.keep );
		std::vector<EyePiece>().swap( t.pieces );
		std::vector<EyePiece>().swap( t.tmp );
		std::vector<EyePiece>().swap( t.eye );
	}
	if ( capped )
		Msg( "eyes: %d node columns were cut into more than %d pieces; the rest of their obstacles were skipped "
			 "(those columns over-reach)\n", capped, (int)FEET_MAX_PIECES );
	eyes.pieceClusters = eyes.clusters;
	// one entry per viewpoint (a point_viewcontrol's rail is several pieces): its
	// first piece with a sample in solid, whichever thread found which
	std::sort( solidHits.begin(), solidHits.end(), []( const SolidViewHit &a, const SolidViewHit &b ) {
		return a.viewpoint != b.viewpoint ? a.viewpoint < b.viewpoint : a.part < b.part;
	} );
	std::vector< std::pair<int, dvec3> > &solidViews = eyes.solidViews;
	solidViews.clear();
	for ( size_t i = 0; i < solidHits.size(); i++ )
	{
		if ( i == 0 || solidHits[i].viewpoint != solidHits[i - 1].viewpoint )
			solidViews.push_back( std::make_pair( solidHits[i].source, solidHits[i].at ) );
	}
	if ( !solidViews.empty() )
	{
		std::sort( solidViews.begin(), solidViews.end(), []( const std::pair<int, dvec3> &a, const std::pair<int, dvec3> &b ) {
			if ( a.first != b.first )
				return a.first < b.first;
			if ( a.second.x != b.second.x )
				return a.second.x < b.second.x;
			if ( a.second.y != b.second.y )
				return a.second.y < b.second.y;
			return a.second.z < b.second.z;
		} );
		// the JSON lists the same (eyes.solid_viewpoints); bspwalk_runner.py reads it from there
		Warning( "bspwalk: %d viewpoint(s) lie in a solid leaf; the engine draws every cluster from there and sees "
				 "through every brush side facing solid, so every face one of them sees stays visible (fewer faces "
				 "can be hidden). Move them into open space:\n", (int)solidViews.size() );
		for ( size_t i = 0; i < solidViews.size(); i++ )
			Warning( "  %s at (%.1f %.1f %.1f)\n", EyeSourceName( solidViews[i].first ), solidViews[i].second.x,
					 solidViews[i].second.y, solidViews[i].second.z );
	}
	for ( int c = 0; c < nClusters; c++ )
		eyes.eyeClusters += eyes.clusters[c];

	// dedupe the samples (adjacent pieces share grid points), then their clusters.
	// Of the samples at one point the one kept is a real one (then the lowest sky
	// camera's copy), and of those a viewpoint's ahead of any other source, then the
	// lowest source: the same whichever thread made which. bw_vis relies on the
	// viewpoint's being kept: it finds the viewpoints in solid by their samples'
	// source (s_SolidEyes), and a player sample at the same point would hide one.
	std::sort( eyes.samples.begin(), eyes.samples.end(), []( const EyeSample &a, const EyeSample &b ) {
		if ( a.x != b.x )
			return a.x < b.x;
		if ( a.y != b.y )
			return a.y < b.y;
		if ( a.z != b.z )
			return a.z < b.z;
		if ( a.sky != b.sky )
			return a.sky < b.sky;
		bool va = IsViewpointSource( a.source ), vb = IsViewpointSource( b.source );
		if ( va != vb )
			return va;
		return a.source < b.source;
	} );
	eyes.samples.erase( std::unique( eyes.samples.begin(), eyes.samples.end(), []( const EyeSample &a, const EyeSample &b ) {
		return a.x == b.x && a.y == b.y && a.z == b.z && a.sky == b.sky;
	} ), eyes.samples.end() );
	if ( eyes.samples.size() > (size_t)INT_MAX )
		Error( "bspwalk: %.0f eye samples kept, more than the int indices of the vis pass hold; raise -eyespacing\n",
			   (double)eyes.samples.size() );
	// The samples' own bound per cluster. The descent does not cut a flat piece (one with
	// no thickness; it fails every clip): only its corners reach MarkRegion, so the samples
	// along it in the clusters between lie outside the bound of the parts (on
	// d1_trainstation_02, 120 samples outside the box of the parts, 271 outside their
	// k-DOP). A sample is an eye position, so the region holds it, and with it the stretch
	// between collinear samples.
	std::vector<int> perCluster( nClusters, 0 );
	std::vector<EyeBound> sampleBound( nClusters );
	if ( !eyes.samples.empty() )
		RunEyePass( (int)( ( eyes.samples.size() + SAMPLE_CLUSTER_CHUNK - 1 ) / SAMPLE_CLUSTER_CHUNK ), SampleClusterChunk );
	for ( int i = 0; i <= MAX_TOOL_THREADS; i++ )
	{
		EyeThread &t = s_EyeThreads[i];
		for ( size_t c = 0; c < t.perCluster.size(); c++ )
		{
			perCluster[c] += t.perCluster[c];
			sampleBound[c].Merge( t.sampleBound[c] );
		}
		std::vector<int>().swap( t.perCluster );
		std::vector<EyeBound>().swap( t.sampleBound );
	}
	// per eye cluster the pieces touch: the bound of the piece parts in it and of the
	// samples there (no part found: unbounded, every face may be seen from it). Eye
	// clusters with no sample in them are listed apart (bare).
	EyeBound origin;
	origin.AtOrigin();
	eyes.region.assign( nClusters, origin );
	for ( int c = 0; c < nClusters; c++ )
	{
		if ( !eyes.pieceClusters[c] )
			continue;
		EyeBound m;
		for ( int i = 0; i <= MAX_TOOL_THREADS; i++ )
		{
			const EyeThread &t = s_EyeThreads[i];
			if ( c < (int)t.region.size() )
				m.Merge( t.region[c] );
		}
		if ( !m.any )
			m.Unbounded();
		else
			m.Merge( sampleBound[c] );
		eyes.region[c] = m;
		if ( perCluster[c] )
			continue;
		eyes.bareClusters.push_back( c );
		eyes.bare.push_back( m );
	}
	for ( int i = 0; i <= MAX_TOOL_THREADS; i++ )
		std::vector<EyeBound>().swap( s_EyeThreads[i].region );

	s_EyeCrouchObs = ObstacleSet();
	s_EyeStandObs = ObstacleSet();
	s_bStandColumn = false;
	std::vector<int>().swap( s_EyeNodes );
	std::vector<double>().swap( s_EyeRise );
	std::vector<EyePiece>().swap( s_EyeLoose );
	eyes.tBuild = Plat_FloatTime() - t0;

	Msg( "eyes:" );
	for ( int s = 0; s < EYESRC_COUNT; s++ )
		Msg( " %s %d", EyeSourceName( s ), eyes.pieces[s] );
	Msg( " pieces%s; %d of %d clusters; %d viewpoints; %d samples at %gu (%.0f made, %.0f estimated; %d eye clusters "
		 "the lattice missed); %.2f s\n",
		 eyes.sky ? " (and each mapped into the 3D skybox)" : "", eyes.eyeClusters, nClusters, eyes.viewpoints,
		 (int)eyes.samples.size(), s_EyeSpacing, (double)nSamples, est, (int)eyes.bareClusters.size(), eyes.tBuild );
	for ( size_t k = 0; k < eyes.skyOrigins.size(); k++ )
		Msg( "eyes: sky_camera at (%.1f %.1f %.1f), scale %g\n", eyes.skyOrigins[k].x, eyes.skyOrigins[k].y,
			 eyes.skyOrigins[k].z, eyes.skyScales[k] );
}
