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
#include "bw_eyestore.h"

#include <algorithm>
#include <atomic>
#include <float.h>
#include <limits.h>
#include <math.h>
#include <memory>
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
static thread_local std::vector<int> *t_pPieceCl;	// EmitPiece: every cluster MarkLeaf marks for the piece (repeats kept)

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
		if ( t_pPieceCl )
			t_pPieceCl->push_back( c );
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

// Drops the points of a set that are not corners of its hull, and repeats of a point, so a
// point set split again and again (PointsClusters) stays the size of its hull instead of
// growing with every split. A flat set (every point within EYE_TOUCH of one plane) keeps the
// corners of its hull in that plane; any other set the vertices of ConvexFromPoints' hull
// when that hull is checked to hold every point within 1e-5 (far inside the EYE_SPLIT_EPS
// band and REGION_MARGIN), else the set as it is.
static void ReduceHullPoints( std::vector<dvec3> &pts )
{
	std::vector<dvec3> u;
	for ( size_t i = 0; i < pts.size(); i++ )
		AddWelded( u, pts[i] );
	pts.swap( u );
	if ( pts.size() <= 4 )
		return;
	// the plane through three far-apart points
	const dvec3 p0 = pts[0];
	size_t i1 = 0, i2 = 0;
	double best = 0;
	for ( size_t i = 1; i < pts.size(); i++ )
	{
		double l = DLength( pts[i] - p0 );
		if ( l > best )
		{
			best = l;
			i1 = i;
		}
	}
	if ( best <= EYE_WELD )
		return;
	const dvec3 ax = ( pts[i1] - p0 ) * ( 1.0 / best );
	best = 0;
	for ( size_t i = 1; i < pts.size(); i++ )
	{
		dvec3 q = pts[i] - p0;
		double l = DLength( q - ax * DDot( q, ax ) );
		if ( l > best )
		{
			best = l;
			i2 = i;
		}
	}
	if ( best <= EYE_WELD )
	{
		// collinear: the two ends
		size_t lo = 0, hi = 0;
		for ( size_t i = 1; i < pts.size(); i++ )
		{
			if ( DDot( pts[i] - p0, ax ) < DDot( pts[lo] - p0, ax ) )
				lo = i;
			if ( DDot( pts[i] - p0, ax ) > DDot( pts[hi] - p0, ax ) )
				hi = i;
		}
		dvec3 a = pts[lo], b = pts[hi];
		pts.clear();
		pts.push_back( a );
		pts.push_back( b );
		return;
	}
	dvec3 nrm = DCross( ax, pts[i2] - p0 );
	nrm = nrm * ( 1.0 / DLength( nrm ) );
	bool flat = true;
	for ( size_t i = 0; i < pts.size() && flat; i++ )
		flat = fabs( DDot( pts[i] - p0, nrm ) ) <= EYE_TOUCH;
	if ( !flat )
	{
		ConvexShape hull;
		if ( pts.size() > 8 && ConvexFromPoints( pts.data(), (int)pts.size(), hull ) && hull.verts.size() >= 4 )
		{
			// The hull is taken only when it is checked to hold every point within
			// HULL_KEEP: on a sliver a few 1e-6 thick, quickhull's own epsilon decides
			// its faces by rounding noise and the hull it returns can lie units inside
			// the set (measured: 10u on a 138u sliver), which would shrink a cluster's
			// eye region, the unsafe direction. A set that fails keeps all its points.
			const double HULL_KEEP = 1e-5;
			bool holds = hull.faceNormals.size() == hull.faces.size();
			for ( size_t f = 0; f < hull.faces.size() && holds; f++ )
			{
				if ( hull.faces[f].empty() )
				{
					holds = false;
					break;
				}
				const dvec3 &v0 = hull.verts[hull.faces[f][0]];
				for ( size_t i = 0; i < pts.size() && holds; i++ )
					holds = DDot( pts[i] - v0, hull.faceNormals[f] ) <= HULL_KEEP;
			}
			if ( holds )
				pts.swap( hull.verts );
		}
		return;
	}
	const dvec3 ay = DCross( nrm, ax );
	// monotone chain over (ax, ay) coordinates; collinear points on an edge are dropped
	std::vector<int> order( pts.size() );
	std::vector<double> cx( pts.size() ), cy( pts.size() );
	for ( size_t i = 0; i < pts.size(); i++ )
	{
		order[i] = (int)i;
		cx[i] = DDot( pts[i] - p0, ax );
		cy[i] = DDot( pts[i] - p0, ay );
	}
	std::sort( order.begin(), order.end(), [&]( int a, int b ) { return cx[a] != cx[b] ? cx[a] < cx[b] : cy[a] < cy[b]; } );
	auto turn = [&]( int o, int a, int b ) {
		return ( cx[a] - cx[o] ) * ( cy[b] - cy[o] ) - ( cy[a] - cy[o] ) * ( cx[b] - cx[o] );
	};
	std::vector<int> hull( 2 * order.size() );
	int k = 0;
	for ( size_t i = 0; i < order.size(); i++ )
	{
		while ( k >= 2 && turn( hull[k - 2], hull[k - 1], order[i] ) <= 0 )
			k--;
		hull[k++] = order[i];
	}
	for ( int i = (int)order.size() - 2, t = k + 1; i >= 0; i-- )
	{
		while ( k >= t && turn( hull[k - 2], hull[k - 1], order[i] ) <= 0 )
			k--;
		hull[k++] = order[i];
	}
	std::vector<dvec3> kept;
	for ( int i = 0; i + 1 < k; i++ )
		kept.push_back( pts[hull[i]] );
	if ( kept.size() >= 3 )
		pts.swap( kept );
}

// A piece part that EyePolyClip cannot build as a solid (a flat piece, such as an arc's
// vertical sheet, fails every clip; so does a sliver that only touches the band), as the
// points whose hull it is. The descent splits the set as PolyClusters splits a piece: each
// side gets its points within the band and, for every pair with one point on each side,
// where the segment between them crosses the band's edge; their hull is the hull's part on
// that side. So every leaf the part reaches gets its region, not only the leaves of its
// corners (a flat piece described by its corners alone left clusters with samples and no
// region: on d1_trainstation_02, eye cluster 624 with 688 samples kept its region at the
// origin, #77).
static void PointsClusters( int node, std::vector<dvec3> &pts, std::vector<unsigned char> &clusters )
{
	ReduceHullPoints( pts );
	if ( pts.empty() )
		return;
	while ( node >= 0 )
	{
		const dnode_t &dn = dnodes[node];
		const dplane_t &pl = dplanes[dn.planenum];
		dvec3 n( pl.normal.x, pl.normal.y, pl.normal.z );
		double lo = 1e300, hi = -1e300;
		for ( size_t i = 0; i < pts.size(); i++ )
		{
			double d = DDot( n, pts[i] ) - pl.dist;
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
		for ( int side = 0; side < 2; side++ )
		{
			const double edge = side == 0 ? -EYE_SPLIT_EPS : EYE_SPLIT_EPS;
			const double sgn = side == 0 ? 1.0 : -1.0;
			std::vector<dvec3> part;
			for ( size_t i = 0; i < pts.size(); i++ )
			{
				double di = sgn * ( DDot( n, pts[i] ) - pl.dist - edge );
				if ( di < 0 )
					continue;
				part.push_back( pts[i] );
				for ( size_t j = 0; j < pts.size(); j++ )
				{
					double dj = sgn * ( DDot( n, pts[j] ) - pl.dist - edge );
					if ( dj < 0 )
						part.push_back( pts[i] + ( pts[j] - pts[i] ) * ( di / ( di - dj ) ) );
				}
			}
			if ( !part.empty() )
				PointsClusters( dn.children[side], part, clusters );
		}
		return;
	}
	MarkLeaf( -1 - node, clusters );
	MarkRegion( -1 - node, &pts[0], pts.size() );
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
		// front. A side that does not build (a flat piece, a sliver, or a piece
		// that only touches the band at an edge or corner) goes on as a point
		// set (PointsClusters): the piece's vertices on that side and, for every
		// pair of vertices one on each side, where the segment between them
		// crosses the band's edge. All lie in the piece's part on that side and
		// their hull is that part, so no eye position is lost from the region
		// bound and none is pushed into leaves the piece does not reach.
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
			// signed distance past the band's edge on this side: >= 0 inside
			const double edge = side == 0 ? -EYE_SPLIT_EPS : EYE_SPLIT_EPS;
			const double sgn = side == 0 ? 1.0 : -1.0;
			std::vector<dvec3> pts;
			for ( size_t i = 0; i < p.verts.size(); i++ )
			{
				double di = sgn * ( DDot( n, p.verts[i] ) - pl.dist - edge );
				if ( di < 0 )
					continue;
				pts.push_back( p.verts[i] );
				for ( size_t j = 0; j < p.verts.size(); j++ )
				{
					double dj = sgn * ( DDot( n, p.verts[j] ) - pl.dist - edge );
					if ( dj < 0 )
						pts.push_back( p.verts[i] + ( p.verts[j] - p.verts[i] ) * ( di / ( di - dj ) ) );
				}
			}
			if ( !pts.empty() )
				PointsClusters( dn.children[side], pts, clusters );
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
// Samples (tier B), made one XY tile at a time (#39). Off the lattice: every
// piece's vertices, a segment's points every 'spacing' from its first end, and
// the chord fallback of a poly the lattice under-covers. On the lattice (vertical
// lines 'spacing' apart, world-aligned, indexed by integer): each poly gives a
// line one closed span (ColumnSpan), the spans of a line are merged, and a merged
// span gives both its ends and every lattice height between them that is not
// interior (the shell rule, bw_vis.h EYE_SHELL_MAX_SPACING). A sight line from a
// face outside the eye volume to an eye inside it crosses the volume's boundary
// first, and that crossing point sees the face too, so the shell is enough.
//-----------------------------------------------------------------------------
// Float rounding moves a coordinate under 2^20 by less than this: a piece marks
// its tiles, and a tile gathers its pieces, this far past their bounds, so a
// sample whose float position crosses into the next tile is still made there.
static const double TILE_PAD = 1.0 / 16.0;

struct TileRect
{
	double x0, y0, x1, y1;		// [x0, x1) x [y0, y1), tested on a sample's float position
};

static inline bool InRect( const EyeSample &s, const TileRect &r )
{
	return s.x >= r.x0 && s.x < r.x1 && s.y >= r.y0 && s.y < r.y1;
}

static inline bool BoundsMeet( const dvec3 &mins, const dvec3 &maxs, const TileRect &r, double pad )
{
	return maxs.x >= r.x0 - pad && mins.x <= r.x1 + pad && maxs.y >= r.y0 - pad && mins.y <= r.y1 + pad;
}

static inline EyeSample MakeSample( const dvec3 &p, int source, int sky )
{
	EyeSample s;
	s.sky = sky;
	s.x = (float)p.x;
	s.y = (float)p.y;
	s.z = (float)p.z;
	s.cluster = -1;
	s.source = source;
	s.ux = p.x;
	s.uy = p.y;
	s.uz = p.z;
	return s;
}

static inline void PushIn( std::vector<EyeSample> &out, const dvec3 &p, int source, int sky, const TileRect &r )
{
	EyeSample s = MakeSample( p, source, sky );
	if ( InRect( s, r ) )
		out.push_back( s );
}

// The span of the vertical line through (x, y) across a polyhedron. A side plane
// misses by more than EYE_TOUCH, or a span inverted by more than that, before the
// line is off the piece (clip rounding must not drop a boundary line); a span
// inverted within it is the one point at its middle. False when the line misses.
static bool ColumnSpan( const EyePiece &p, double x, double y, double &lo, double &hi )
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
		lo = hi = 0.5 * ( zlo + zhi );
	else
	{
		lo = zlo;
		hi = zhi;
	}
	return true;
}

// The first lattice height above lo: the least k with k * spacing > lo.
static inline long long FirstAbove( double lo, double spacing )
{
	long long k = (long long)floor( lo / spacing );
	while ( k * spacing <= lo )
		k++;
	return k;
}

// The vertical line through (x, y) across a polyhedron with every point: both
// ends and every lattice height between (the under-cover fallback below).
static void SampleColumn( const EyePiece &p, double x, double y, double spacing, int sky, std::vector<EyeSample> &out )
{
	double lo, hi;
	if ( !ColumnSpan( p, x, y, lo, hi ) )
		return;
	out.push_back( MakeSample( dvec3( x, y, lo ), p.source, sky ) );
	for ( long long k = FirstAbove( lo, spacing ); k * spacing < hi; k++ )
		out.push_back( MakeSample( dvec3( x, y, k * spacing ), p.source, sky ) );
	if ( hi > lo )
		out.push_back( MakeSample( dvec3( x, y, hi ), p.source, sky ) );
}

// A segment's points every 'spacing' from a (b is a vertex, sampled apart), those
// whose float (x, y) lie in r and whose parameter lies in [tlo, thi]. Only the k
// whose points can reach r are visited; each point is a + d (k spacing / len)
// whichever tile asks.
static void SegmentSamples( const dvec3 &a, const dvec3 &b, int source, double spacing, int sky, const TileRect &r,
							std::vector<EyeSample> &out, double tlo = 0.0, double thi = 1.0 )
{
	dvec3 d = b - a;
	double len = DLength( d );
	long long steps = (long long)floor( len / spacing );
	if ( steps < 1 )
		return;
	double t0 = tlo, t1 = thi;
	const double lo[2] = { r.x0 - TILE_PAD, r.y0 - TILE_PAD }, hi[2] = { r.x1 + TILE_PAD, r.y1 + TILE_PAD };
	for ( int ax = 0; ax < 2; ax++ )
	{
		double o = ax ? a.y : a.x, e = ax ? d.y : d.x;
		if ( fabs( e ) < 1e-300 )
		{
			if ( o < lo[ax] || o > hi[ax] )
				return;
			continue;
		}
		double u = ( lo[ax] - o ) / e, v = ( hi[ax] - o ) / e;
		if ( u > v )
			std::swap( u, v );
		t0 = MAX( t0, u );
		t1 = MIN( t1, v );
		if ( t0 > t1 )
			return;
	}
	long long k0 = MAX( 1LL, (long long)floor( t0 * len / spacing ) - 1 );
	long long k1 = MIN( steps, (long long)ceil( t1 * len / spacing ) + 1 );
	for ( long long k = k0; k <= k1; k++ )
	{
		double t = k * spacing / len;
		if ( t < 1.0 && t >= tlo && t <= thi )
			PushIn( out, a + d * t, source, sky, r );
	}
}

// The two vertices of a piece farthest apart in XY (the first such pair), and that
// distance squared.
static double FarthestPairXY( const EyePiece &p, dvec3 &a, dvec3 &b )
{
	double best = -1;
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
	return best;
}

// A poly the lattice under-covers (narrower than the spacing, or a thin diagonal
// strip) also gets the chord between its two vertices farthest apart in XY
// (FarthestPairXY), sampled as columns (the footprint is convex, so the chord lies
// inside it). Returns that chord's step count, or 0 when the piece meets at least
// floor(len / spacing) + 1 lattice lines. The count is over the whole piece, not
// the tile's part of it, so every tile the piece reaches decides alike; it scans
// the piece's whole lattice box, so the piece pass works it out once per piece and
// layer and the tiles read it (#39).
static int UnderCoverSteps( const EyePiece &p, double spacing )
{
	dvec3 a, b;
	double best = FarthestPairXY( p, a, b );
	double len = best > 0 ? sqrt( best ) : 0;
	if ( len <= 0 )
		return 0;
	long long need = (long long)floor( len / spacing ) + 1, cols = 0;
	long long i0 = (long long)ceil( ( p.mins.x - EYE_TOUCH ) / spacing ), i1 = (long long)floor( ( p.maxs.x + EYE_TOUCH ) / spacing );
	long long j0 = (long long)ceil( ( p.mins.y - EYE_TOUCH ) / spacing ), j1 = (long long)floor( ( p.maxs.y + EYE_TOUCH ) / spacing );
	double lo, hi;
	for ( long long i = i0; i <= i1; i++ )
	{
		for ( long long j = j0; j <= j1; j++ )
		{
			if ( ColumnSpan( p, i * spacing, j * spacing, lo, hi ) && ++cols >= need )
				return 0;
		}
	}
	return (int)ceil( len / spacing );
}

struct TileSpan
{
	long long i, j;		// the lattice line (i spacing, j spacing)
	double lo, hi;
	int source;
};

struct TileLine
{
	long long i, j;
	int first, count;	// its merged spans in TileScratch::spans
};

// A path chord a -> b that the poly pieces clip (in one layer), and the stretch
// [u0, u1] of it whose samples are wanted. Its samples are the chord's own (a, b
// and every 'spacing' from a, as the whole chord had them): no longer eyes of
// their own (its kept parts are sampled from their own ends), but candidates for
// SampleTilePieces' cluster rule.
struct SpareChord
{
	dvec3 a, b;
	double u0, u1;
	int source;
};

struct TileScratch
{
	std::vector<TileSpan> spans;
	std::vector<TileLine> lines;
	std::vector<EyeSample> raw;
	std::vector<EyeSample> spare;		// samples the old eye set had and this one drops ...
	size_t spareDone;					// ... of which the first spareDone are compacted (CompactSpare)
	std::vector<int> kept;				// the clusters of the kept samples

	TileScratch() : spareDone( 0 ) {}
};

// The dedupe order: of the samples at one point the one kept is a viewpoint's
// ahead of any other source, then the lowest source, then the least unrounded
// position (so which of two pieces' samples at one float point is kept never
// depends on the order the pieces came in). The viewpoint's being kept is what
// EyeVolume::solidEyes assumes (the viewpoint samples in solid, one per point).
static bool SampleOrder( const EyeSample &a, const EyeSample &b )
{
	if ( a.x != b.x )
		return a.x < b.x;
	if ( a.y != b.y )
		return a.y < b.y;
	if ( a.z != b.z )
		return a.z < b.z;
	bool va = IsViewpointSource( a.source ), vb = IsViewpointSource( b.source );
	if ( va != vb )
		return va;
	if ( a.source != b.source )
		return a.source < b.source;
	if ( a.ux != b.ux )
		return a.ux < b.ux;
	if ( a.uy != b.uy )
		return a.uy < b.uy;
	return a.uz < b.uz;
}

static int FindLine( const std::vector<TileLine> &lines, long long i, long long j )
{
	size_t lo = 0, hi = lines.size();
	while ( lo < hi )
	{
		size_t m = ( lo + hi ) / 2;
		if ( lines[m].i < i || ( lines[m].i == i && lines[m].j < j ) )
			lo = m + 1;
		else
			hi = m;
	}
	return lo < lines.size() && lines[lo].i == i && lines[lo].j == j ? (int)lo : -1;
}

// A merged span of line l holds z strictly inside.
static inline bool LineHolds( const TileScratch &ts, int l, double z )
{
	const TileLine &L = ts.lines[l];
	for ( int s = L.first; s < L.first + L.count; s++ )
	{
		const TileSpan &sp = ts.spans[s];
		if ( sp.lo >= z )
			return false;
		if ( z < sp.hi )
			return true;
	}
	return false;
}

static inline int SampleCluster( const EyeSample &s )
{
	return dleafs[PointLeaf( dvec3( s.x, s.y, s.z ) )].cluster;
}

// The cluster rule (SampleTilePieces) can only keep, per cluster, the first of the
// dropped samples in sample order, so the dropped samples are reduced to that one per
// cluster (with cluster -1 dropped) every SPARE_BATCH of them: a tall piece's interior
// lattice heights would otherwise all be held at once (on d1_trainstation_02 6M in one
// tile, 1 GB over 32 threads).
static const size_t SPARE_BATCH = 1 << 16;

static void CompactSpare( TileScratch &ts )
{
	std::vector<EyeSample> &sp = ts.spare;
	for ( size_t i = ts.spareDone; i < sp.size(); i++ )
		sp[i].cluster = SampleCluster( sp[i] );
	std::sort( sp.begin(), sp.end(), []( const EyeSample &a, const EyeSample &b ) {
		return a.cluster != b.cluster ? a.cluster < b.cluster : SampleOrder( a, b );
	} );
	size_t m = 0;
	for ( size_t i = 0; i < sp.size(); i++ )
	{
		if ( sp[i].cluster >= 0 && ( m == 0 || sp[m - 1].cluster != sp[i].cluster ) )
			sp[m++] = sp[i];
	}
	sp.resize( m );
	ts.spareDone = m;
}

static inline void SpareGrown( TileScratch &ts )
{
	if ( ts.spare.size() >= ts.spareDone + SPARE_BATCH )
		CompactSpare( ts );
}

// The lattice lines whose samples can lie in r (a float position in r: TILE_PAD past
// it), and with the shell rule the ring of lines around them, which the shell test
// reads (#39: one ring, and none without the shell).
static void TileLineRange( const TileRect &r, double spacing, bool shell, double &i0, double &i1, double &j0, double &j1 )
{
	const double ring = shell ? 1 : 0;
	i0 = floor( ( r.x0 - TILE_PAD ) / spacing ) - ring;
	i1 = ceil( ( r.x1 + TILE_PAD ) / spacing ) + ring;
	j0 = floor( ( r.y0 - TILE_PAD ) / spacing ) - ring;
	j1 = ceil( ( r.y1 + TILE_PAD ) / spacing ) + ring;
}

// The samples of 'pieces' (one layer) whose float (x, y) lie in r, sorted and
// deduplicated, appended to 'out'. Every piece that reaches r's lattice lines
// (TileLineRange, EYE_TOUCH past them) or r itself (TILE_PAD past it) must be in
// 'pieces' (and every such clipped chord in 'spare'); others add nothing. A lattice
// sample's source is the lowest of the spans merged into its span (only
// viewpoints, which are never polys, need their own). 'under' (or NULL: worked out
// here) gives each piece's UnderCoverSteps at this spacing.
// 'clusters' (a BSP is loaded): no eye cluster is lost to the smaller set. The
// samples the full lattice and the whole chords had and this set drops (interior
// lattice heights, a piece's span ends inside a merged span, a clipped chord's
// own samples) can be the only ones in a cluster (a thin leaf no lattice line
// crosses), and bw_vis keeps visible every face a cluster with no sample may
// see. So for each cluster those reach in the tile and no kept sample of the
// tile does, the first of them in sample order is kept too (more eyes only keep
// more faces visible: the safe side). The kept samples get their leaf cluster here
// (of the float position). 'spareClusters' (or NULL): the clusters of every
// candidate the rule looked at, sorted, for -eyeverify. Returns the samples made
// before the dedupe.
static size_t SampleTilePieces( const EyePiece *const *pieces, size_t n, const SpareChord *spare, size_t nSpare,
								double spacing, bool shell, bool clusters, int sky, const TileRect &r, TileScratch &ts,
								std::vector<EyeSample> &out, const int *under = NULL, std::vector<int> *spareClusters = NULL )
{
	ts.spans.clear();
	ts.lines.clear();
	ts.raw.clear();
	ts.spare.clear();
	ts.spareDone = 0;
	double ci0, ci1, cj0, cj1;
	TileLineRange( r, spacing, shell, ci0, ci1, cj0, cj1 );
	double lo, hi;
	for ( size_t k = 0; k < n; k++ )
	{
		const EyePiece &p = *pieces[k];
		bool near = BoundsMeet( p.mins, p.maxs, r, TILE_PAD );
		if ( near )
		{
			for ( size_t v = 0; v < p.verts.size(); v++ )
				PushIn( ts.raw, p.verts[v], p.source, sky, r );
			if ( p.kind == EYE_SEGMENT )
				SegmentSamples( p.verts[0], p.verts[1], p.source, spacing, sky, r, ts.raw );
		}
		if ( p.kind != EYE_POLY )
			continue;
		double i0 = MAX( ceil( ( p.mins.x - EYE_TOUCH ) / spacing ), ci0 ), i1 = MIN( floor( ( p.maxs.x + EYE_TOUCH ) / spacing ), ci1 );
		double j0 = MAX( ceil( ( p.mins.y - EYE_TOUCH ) / spacing ), cj0 ), j1 = MIN( floor( ( p.maxs.y + EYE_TOUCH ) / spacing ), cj1 );
		for ( long long i = (long long)i0; i0 <= i1 && i <= (long long)i1; i++ )
		{
			for ( long long j = (long long)j0; j0 <= j1 && j <= (long long)j1; j++ )
			{
				if ( !ColumnSpan( p, i * spacing, j * spacing, lo, hi ) )
					continue;
				TileSpan s;
				s.i = i;
				s.j = j;
				s.lo = lo;
				s.hi = hi;
				s.source = p.source;
				ts.spans.push_back( s );
			}
		}
		if ( !near )
			continue;
		int steps = under ? under[k] : UnderCoverSteps( p, spacing );
		dvec3 a, b;
		if ( steps > 1 )
			FarthestPairXY( p, a, b );
		for ( int s = 1; s < steps; s++ )
		{
			double t = (double)s / steps;
			double x = a.x + ( b.x - a.x ) * t, y = a.y + ( b.y - a.y ) * t;
			if ( InRect( MakeSample( dvec3( x, y, 0 ), 0, sky ), r ) )
				SampleColumn( p, x, y, spacing, sky, ts.raw );
		}
	}
	if ( clusters )
	{
		for ( size_t k = 0; k < nSpare; k++ )
		{
			const SpareChord &c = spare[k];
			if ( c.u0 <= 0 )
				PushIn( ts.spare, c.a, c.source, sky, r );
			if ( c.u1 >= 1 )
				PushIn( ts.spare, c.b, c.source, sky, r );
			SegmentSamples( c.a, c.b, c.source, spacing, sky, r, ts.spare, c.u0, c.u1 );
			SpareGrown( ts );
		}
	}

	// merge the spans of each line (touching within EYE_TOUCH merges)
	std::vector<TileSpan> &sp = ts.spans;
	std::sort( sp.begin(), sp.end(), []( const TileSpan &a, const TileSpan &b ) {
		if ( a.i != b.i )
			return a.i < b.i;
		if ( a.j != b.j )
			return a.j < b.j;
		if ( a.lo != b.lo )
			return a.lo < b.lo;
		if ( a.hi != b.hi )
			return a.hi < b.hi;
		return a.source < b.source;
	} );
	size_t w = 0;
	for ( size_t s = 0; s < sp.size(); )
	{
		size_t e = s + 1;
		while ( e < sp.size() && sp[e].i == sp[s].i && sp[e].j == sp[s].j )
			e++;
		const double x = sp[s].i * spacing, y = sp[s].j * spacing;
		if ( clusters && InRect( MakeSample( dvec3( x, y, 0 ), 0, sky ), r ) )
		{
			// every piece's own span ends: the merged span keeps only its own
			for ( size_t q = s; q < e; q++ )
			{
				ts.spare.push_back( MakeSample( dvec3( x, y, sp[q].lo ), sp[q].source, sky ) );
				ts.spare.push_back( MakeSample( dvec3( x, y, sp[q].hi ), sp[q].source, sky ) );
			}
			SpareGrown( ts );
		}
		TileLine L;
		L.i = sp[s].i;
		L.j = sp[s].j;
		L.first = (int)w;
		TileSpan cur = sp[s];
		for ( size_t q = s + 1; q < e; q++ )
		{
			if ( sp[q].lo <= cur.hi + EYE_TOUCH )
			{
				cur.hi = MAX( cur.hi, sp[q].hi );
				cur.source = MIN( cur.source, sp[q].source );
			}
			else
			{
				sp[w++] = cur;
				cur = sp[q];
			}
		}
		sp[w++] = cur;
		L.count = (int)w - L.first;
		ts.lines.push_back( L );
		s = e;
	}
	sp.resize( w );

	// each line in r: its merged spans' ends, and the lattice heights between
	// them that are not interior
	static const int NB[8][2] = { { -1, -1 }, { -1, 0 }, { -1, 1 }, { 0, -1 }, { 0, 1 }, { 1, -1 }, { 1, 0 }, { 1, 1 } };
	for ( size_t l = 0; l < ts.lines.size(); l++ )
	{
		const TileLine &L = ts.lines[l];
		const double x = L.i * spacing, y = L.j * spacing;
		if ( !InRect( MakeSample( dvec3( x, y, 0 ), 0, sky ), r ) )
			continue;
		int nb[8];
		bool ring = shell;
		for ( int m = 0; m < 8 && ring; m++ )
		{
			nb[m] = FindLine( ts.lines, L.i + NB[m][0], L.j + NB[m][1] );
			ring = nb[m] >= 0;
		}
		for ( int s = L.first; s < L.first + L.count; s++ )
		{
			const TileSpan &span = sp[s];
			ts.raw.push_back( MakeSample( dvec3( x, y, span.lo ), span.source, sky ) );
			if ( span.hi > span.lo )
				ts.raw.push_back( MakeSample( dvec3( x, y, span.hi ), span.source, sky ) );
			for ( long long k = FirstAbove( span.lo, spacing ); k * spacing < span.hi; k++ )
			{
				const double z = k * spacing;
				bool interior = ring;
				for ( int m = 0; m < 8 && interior; m++ )
					interior = LineHolds( ts, nb[m], z );
				if ( !interior )
					ts.raw.push_back( MakeSample( dvec3( x, y, z ), span.source, sky ) );
				else if ( clusters )
					ts.spare.push_back( MakeSample( dvec3( x, y, z ), span.source, sky ) );
			}
		}
		if ( clusters )
			SpareGrown( ts );
	}

	if ( clusters )
	{
		ts.kept.clear();
		for ( size_t i = 0; i < ts.raw.size(); i++ )
		{
			ts.raw[i].cluster = SampleCluster( ts.raw[i] );
			ts.kept.push_back( ts.raw[i].cluster );
		}
		std::sort( ts.kept.begin(), ts.kept.end() );
		ts.kept.erase( std::unique( ts.kept.begin(), ts.kept.end() ), ts.kept.end() );
		// one dropped sample per cluster, ascending, each its cluster's first in sample order
		CompactSpare( ts );
		if ( spareClusters )
			spareClusters->clear();
		for ( size_t i = 0; i < ts.spare.size(); i++ )
		{
			const EyeSample &s = ts.spare[i];
			if ( spareClusters )
				spareClusters->push_back( s.cluster );
			if ( !std::binary_search( ts.kept.begin(), ts.kept.end(), s.cluster ) )
				ts.raw.push_back( s );
		}
	}

	size_t made = ts.raw.size();
	std::sort( ts.raw.begin(), ts.raw.end(), SampleOrder );
	ts.raw.erase( std::unique( ts.raw.begin(), ts.raw.end(), []( const EyeSample &a, const EyeSample &b ) {
		return a.x == b.x && a.y == b.y && a.z == b.z;
	} ), ts.raw.end() );
	out.insert( out.end(), ts.raw.begin(), ts.raw.end() );
	return made;
}

void SampleEyeTile( const std::vector<EyePiece> &pieces, double spacing, double realSpacing, int sky, double x0, double y0,
					double x1, double y1, std::vector<EyeSample> &out )
{
	std::vector<const EyePiece *> ptrs( pieces.size() );
	for ( size_t i = 0; i < pieces.size(); i++ )
		ptrs[i] = &pieces[i];
	TileRect r = { x0, y0, x1, y1 };
	TileScratch ts;
	SampleTilePieces( ptrs.data(), ptrs.size(), NULL, 0, spacing, realSpacing <= EYE_SHELL_MAX_SPACING, false, sky, r, ts,
					  out );
}

void SampleEyePiece( const EyePiece &p, double spacing, std::vector<EyeSample> &out )
{
	std::vector<EyePiece> one( 1, p );
	SampleEyeTile( one, spacing, spacing, -1, -1e30, -1e30, 1e30, 1e30, out );
}

//-----------------------------------------------------------------------------
// Path chords (#39): an arc or surf-flight chord is kept only where it lies
// outside every real poly piece (each grown by EYE_TOUCH); inside one, the
// shell of that piece already holds every eye the chord would add.
//-----------------------------------------------------------------------------
// The parameter interval [t0, t1] of a -> b inside p grown by EYE_TOUCH
// (n . x <= d + EYE_TOUCH; a closed piece, so touching is inside). False when the
// chord misses it.
static bool ChordInside( const dvec3 &a, const dvec3 &b, const EyePiece &p, double &t0, double &t1 )
{
	dvec3 d = b - a;
	t0 = 0;
	t1 = 1;
	for ( size_t k = 0; k < p.planes.size(); k++ )
	{
		const DPlane &pl = p.planes[k];
		double num = pl.d + EYE_TOUCH - DDot( pl.n, a ), den = DDot( pl.n, d );
		if ( fabs( den ) < 1e-300 )
		{
			if ( num < 0 )
				return false;
			continue;
		}
		double t = num / den;
		if ( den > 0 )
			t1 = MIN( t1, t );
		else
			t0 = MAX( t0, t );
		if ( t0 > t1 )
			return false;
	}
	return true;
}

// The parts of a -> b outside the intervals (sorted by start; they merge where
// they meet), appended to 'parts' as pairs of ends; a part shorter than
// EYE_SPLIT_EPS is left out (both its ends lie on the volume). Returns the kept
// length.
static double ChordKeptParts( const dvec3 &a, const dvec3 &b, const std::pair<double, double> *iv, size_t n,
							  std::vector<dvec3> &parts )
{
	const dvec3 d = b - a;
	const double len = DLength( d );
	if ( !n )
	{
		parts.push_back( a );
		parts.push_back( b );
		return len;
	}
	double kept = 0, from = 0;
	size_t k = 0;
	while ( true )
	{
		double to = k < n ? iv[k].first : 1.0;
		if ( to > from && ( to - from ) * len >= EYE_SPLIT_EPS )
		{
			parts.push_back( from <= 0 ? a : a + d * from );
			parts.push_back( to >= 1 ? b : a + d * to );
			kept += ( to - from ) * len;
		}
		if ( k >= n )
			break;
		double end = iv[k].second;
		for ( k++; k < n && iv[k].first <= end; k++ )
			end = MAX( end, iv[k].second );
		from = end;
		if ( from >= 1 )
			break;
	}
	return kept;
}

void ClipEyeChord( const dvec3 &a, const dvec3 &b, const std::vector<EyePiece> &polys, std::vector<dvec3> &parts )
{
	std::vector< std::pair<double, double> > iv;
	double t0, t1;
	for ( size_t i = 0; i < polys.size(); i++ )
	{
		if ( polys[i].kind == EYE_POLY && ChordInside( a, b, polys[i], t0, t1 ) )
			iv.push_back( std::make_pair( t0, t1 ) );
	}
	std::sort( iv.begin(), iv.end() );
	parts.clear();
	ChordKeptParts( a, b, iv.data(), iv.size(), parts );
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

// A path chord (#39): one EYE_SEGMENT of an eye curve along a free arc or a surf
// flight (ArcChordEyes' rule), before it is clipped to its parts outside the
// real poly pieces.
struct PathChord
{
	dvec3 a, b;
	int source;
};

// A real poly piece's hold on a path chord: the parameter interval inside it.
struct ChordHit
{
	int chord;
	double t0, t1;
};

// A segment or point the tiles sample (ladder segments, the kept path parts, the
// loose pieces), or a clipped path chord a -> b (SpareChord: its own samples only
// keep clusters).
enum { TILESEG_SEGMENT = 0, TILESEG_POINT, TILESEG_SPARE };
struct TileSeg
{
	dvec3 a, b;
	double u0, u1;
	int source;
	int kind;
};

// A tile of one layer: 0 the real one, k + 1 sky camera k's copy.
struct TileKey
{
	int layer, tx, ty;
	bool operator<( const TileKey &o ) const
	{
		if ( layer != o.layer )
			return layer < o.layer;
		if ( tx != o.tx )
			return tx < o.tx;
		return ty < o.ty;
	}
	bool operator==( const TileKey &o ) const { return layer == o.layer && tx == o.tx && ty == o.ty; }
};

// A fan eye candidate (#39): a piece's sample at the fan spacing, keyed by its fan cell
// ('set' 0 for the real layer, k + 1 for sky camera k's copies).
struct FanCand
{
	long long cell;
	int set;
	FanEye eye;
};

// The fan cell order, then within a cell the first sample in (x, y, z) order (a viewpoint's
// ahead of another source at one point, then the lowest source, as SampleOrder).
static bool FanCandLess( const FanCand &a, const FanCand &b )
{
	if ( a.set != b.set )
		return a.set < b.set;
	if ( a.cell != b.cell )
		return a.cell < b.cell;
	const FanEye &p = a.eye, &q = b.eye;
	if ( p.x != q.x )
		return p.x < q.x;
	if ( p.y != q.y )
		return p.y < q.y;
	if ( p.z != q.z )
		return p.z < q.z;
	bool vp = IsViewpointSource( p.source ), vq = IsViewpointSource( q.source );
	if ( vp != vq )
		return vp;
	return p.source < q.source;
}

// Sorted by FanCandLess, one candidate kept per cell.
static void CompactFanCands( std::vector<FanCand> &c )
{
	std::sort( c.begin(), c.end(), FanCandLess );
	c.erase( std::unique( c.begin(), c.end(), []( const FanCand &a, const FanCand &b ) {
		return a.set == b.set && a.cell == b.cell;
	} ), c.end() );
}

// A ride sweep (pass 0, item = node item) or special piece (pass 1, item =
// special item), kept for the tiles; one thread makes all of an item's, in order.
struct ListedPiece
{
	int pass, item;
	EyePiece piece;
	std::vector<int> under;		// per layer: its UnderCoverSteps (EmitPiece)
};

// A piece's cluster and one tile of the piece's layer it reaches (#39): the store finds a
// cluster's samples in the tiles listed for it.
struct ClusterTile
{
	int cluster;
	TileKey key;
	bool operator<( const ClusterTile &o ) const { return cluster != o.cluster ? cluster < o.cluster : key < o.key; }
	bool operator==( const ClusterTile &o ) const { return cluster == o.cluster && key == o.key; }
};

struct EyeThread
{
	std::vector<unsigned char> clusters;
	std::vector<EyeBound> region;		// per cluster, see MarkRegion (sized by the first piece it emits)
	std::vector<EyePiece> keep;
	std::vector<int> cand, chordCand;
	std::vector<const GrownPolytope *> obs;
	std::vector<EyePiece> pieces, tmp, eye;
	std::vector<float> arcTimes;		// ArcChordTimes' chord end times
	int pieces_[EYESRC_COUNT];
	int mapped;
	std::vector<SolidViewHit> solidViews;				// viewpoint pieces with a sample in solid
	int cappedColumns;									// columns that hit FEET_MAX_PIECES
	EyeBound all;										// bound of every piece (skybox copies too)
	std::vector<ChordHit> chordHits;					// path chords inside this thread's real polys
	std::vector<ListedPiece> listed;					// ride sweeps and special pieces
	std::vector<TileKey> tileKeys;						// tiles its pieces reach (sorted and unique at times)
	size_t tileCompactAt;
	std::vector<EyeSample> viewSamples;					// a viewpoint piece's samples (the solid check)
	std::vector<FanCand> fanCands;						// fan eye candidates, one per cell at times (FanCandidates)
	size_t fanCompactAt;
	std::vector<EyeSample> fanSamples;					// a piece's samples at the fan spacing
	std::vector<int> pieceCl;							// EmitPiece: the piece's clusters (t_pPieceCl)
	std::vector<ClusterTile> clTiles;					// (cluster, tile) of every piece (sorted and unique at times)
	size_t clTileCompactAt;
	std::vector<EyeSample> solidEyes;					// real viewpoint samples in a solid leaf
	std::vector<EyePiece> tilePieces;					// TileSamples: the pieces of the tile
	std::vector<const EyePiece *> tilePtrs;
	std::vector<int> tileUnder;							// ... their UnderCoverSteps
	std::vector<SpareChord> spare;						// ... the clipped path chords of the tile
	std::vector<EyeSample> tileOut;						// ... its samples
	std::vector<int> order;								// GenerateTile: the samples by cluster
	TileScratch tile;
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
// #39: the path chords (free arcs, then surf flights, in graph order) and their
// XY index, alive while the poly passes record ChordHits
static std::vector<PathChord> s_PathChords;
static CXYHash s_PathHash;
// What the tiles are made from, besides the walk graph, s_EyeRise and the
// obstacle sets (stage B will generate tiles after BuildEyeVolume returns, so
// these are its inputs): the reached nodes by XY (s_NodeHash, by s_EyeNodes
// item; a tile rebuilds their pieces with NodePieces), the ride sweeps and
// special volumes (s_TilePolys), the segments and points (s_TileSegs), one XY
// index over both (s_TileHash: polys first), and the tiles any piece reaches.
static CXYHash s_NodeHash;
// per s_EyeNodes item: the XY box of its pieces, as NodePieces made them in the
// node pass. Taken from the pieces, not the polygon: a sliver's hull can reach
// far past its polygon (d1_trainstation_02 node 51774, a zero-width polygon,
// gives a 13500u piece), and a tile must rebuild every node piece that reaches
// it or its samples are lost (#39).
static std::vector<XYBox> s_NodeBox;
static std::vector<unsigned char> s_NodeUsed;	// the node made a piece
static std::vector< std::vector<int> > s_NodeUnder;	// per s_EyeNodes item: per piece (NodePieces order), per layer, its
													// UnderCoverSteps
static std::vector<EyePiece> s_TilePolys;
static std::vector<int> s_TilePolyUnder;	// per s_TilePolys piece, per layer: its UnderCoverSteps
static std::vector<TileSeg> s_TileSegs;
static CXYHash s_TileHash;
static std::vector<TileKey> s_Tiles;		// every tile any piece reaches, in canonical order: the store's tile ids
static int s_nLayers;						// 1 + sky cameras
static bool s_bShell;						// the shell rule is in force (s_EyeSpacing <= EYE_SHELL_MAX_SPACING)
static EyeStore *s_pStore;					// the store the tiles are made for (GenerateTile)
static std::atomic<int> s_UnrecordedWarned, s_UnplacedWarned;

// One finished eye piece: its clusters, its skybox copies, the tiles it reaches.
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

// The tiles of one layer (tile side 'side') that the bounds reach.
static void MarkTiles( EyeThread &t, int layer, double side, const dvec3 &mins, const dvec3 &maxs )
{
	int tx0 = (int)floor( ( mins.x - TILE_PAD ) / side ), tx1 = (int)floor( ( maxs.x + TILE_PAD ) / side );
	int ty0 = (int)floor( ( mins.y - TILE_PAD ) / side ), ty1 = (int)floor( ( maxs.y + TILE_PAD ) / side );
	for ( int tx = tx0; tx <= tx1; tx++ )
	{
		for ( int ty = ty0; ty <= ty1; ty++ )
		{
			TileKey k = { layer, tx, ty };
			t.tileKeys.push_back( k );
		}
	}
	if ( t.tileKeys.size() >= t.tileCompactAt )
	{
		std::sort( t.tileKeys.begin(), t.tileKeys.end() );
		t.tileKeys.erase( std::unique( t.tileKeys.begin(), t.tileKeys.end() ), t.tileKeys.end() );
		t.tileCompactAt = MAX( (size_t)1 << 16, 2 * t.tileKeys.size() );
	}
	// every cluster the piece marked (t.pieceCl, from its PieceClusters call) lists these tiles
	std::sort( t.pieceCl.begin(), t.pieceCl.end() );
	t.pieceCl.erase( std::unique( t.pieceCl.begin(), t.pieceCl.end() ), t.pieceCl.end() );
	for ( size_t c = 0; c < t.pieceCl.size(); c++ )
	{
		for ( int tx = tx0; tx <= tx1; tx++ )
		{
			for ( int ty = ty0; ty <= ty1; ty++ )
			{
				ClusterTile k = { t.pieceCl[c], { layer, tx, ty } };
				t.clTiles.push_back( k );
			}
		}
	}
	t.pieceCl.clear();
	if ( t.clTiles.size() >= t.clTileCompactAt )
	{
		std::sort( t.clTiles.begin(), t.clTiles.end() );
		t.clTiles.erase( std::unique( t.clTiles.begin(), t.clTiles.end() ), t.clTiles.end() );
		t.clTileCompactAt = MAX( (size_t)1 << 16, 2 * t.clTiles.size() );
	}
}

// The fan eye candidates of one piece of one layer (#39): its samples at the layer's fan
// spacing 'fs' by the plain sampler (vertices, a segment's points, the full lattice, no
// shell), each with its leaf cluster; with vis data one in no cluster is left out. The fan
// cell key packs 21 bits per axis around 2^20 (VIS_MIN_FAN_SPACING keeps it in range).
static void FanCandidates( EyeThread &t, const EyePiece &p, int set, double fs, int sky )
{
	const TileRect everywhere = { -1e30, -1e30, 1e30, 1e30 };
	const EyePiece *pp = &p;
	t.fanSamples.clear();
	SampleTilePieces( &pp, 1, NULL, 0, fs, false, false, sky, everywhere, t.tile, t.fanSamples );
	const bool vis = s_pEyeVol->numClusters > 0;
	for ( size_t i = 0; i < t.fanSamples.size(); i++ )
	{
		const EyeSample &s = t.fanSamples[i];
		int c = SampleCluster( s );
		if ( c < 0 && vis )
			continue;
		long long x = (long long)floor( s.x / fs ) + ( 1 << 20 ), y = (long long)floor( s.y / fs ) + ( 1 << 20 ),
				  z = (long long)floor( s.z / fs ) + ( 1 << 20 );
		FanCand f;
		f.cell = ( x << 42 ) | ( y << 21 ) | z;
		f.set = set;
		f.eye.x = s.x;
		f.eye.y = s.y;
		f.eye.z = s.z;
		f.eye.cluster = c;
		f.eye.sky = sky;
		f.eye.source = s.source;
		t.fanCands.push_back( f );
	}
	if ( t.fanCands.size() >= t.fanCompactAt )
	{
		CompactFanCands( t.fanCands );
		t.fanCompactAt = MAX( (size_t)1 << 16, 2 * t.fanCands.size() );
	}
}

// The path chords a real poly piece holds, and where (#39).
static void RecordChordHits( EyeThread &t, const EyePiece &p )
{
	if ( s_PathChords.empty() )
		return;
	t.chordCand.clear();
	s_PathHash.Query( p.mins.x - EYE_TOUCH, p.mins.y - EYE_TOUCH, p.maxs.x + EYE_TOUCH, p.maxs.y + EYE_TOUCH, t.chordCand );
	for ( size_t k = 0; k < t.chordCand.size(); k++ )
	{
		const PathChord &c = s_PathChords[t.chordCand[k]];
		if ( MIN( c.a.x, c.b.x ) > p.maxs.x + EYE_TOUCH || MAX( c.a.x, c.b.x ) < p.mins.x - EYE_TOUCH ||
			 MIN( c.a.y, c.b.y ) > p.maxs.y + EYE_TOUCH || MAX( c.a.y, c.b.y ) < p.mins.y - EYE_TOUCH ||
			 MIN( c.a.z, c.b.z ) > p.maxs.z + EYE_TOUCH || MAX( c.a.z, c.b.z ) < p.mins.z - EYE_TOUCH )
			continue;
		ChordHit h;
		if ( ChordInside( c.a, c.b, p, h.t0, h.t1 ) )
		{
			h.chord = t.chordCand[k];
			t.chordHits.push_back( h );
		}
	}
}

// 'count': a kept path part is not counted (eyes.pieces counts the chords made). 'under'
// (or NULL): per layer, the piece's UnderCoverSteps at the layer's spacing (0 unless a poly).
static void EmitPiece( EyeThread &t, const EyePiece &p, bool count = true, int *under = NULL )
{
	const EyeVolume &ev = *s_pEyeVol;
	if ( t.region.empty() )
		t.region.assign( t.clusters.size(), EyeBound() );
	t_pRegion = &t.region;
	t_pPieceCl = &t.pieceCl;
	t.pieceCl.clear();
	PieceClusters( p, t.clusters );
	if ( under )
		under[0] = p.kind == EYE_POLY ? UnderCoverSteps( p, s_EyeSpacing ) : 0;
	if ( count )
		t.pieces_[p.source]++;
	for ( size_t i = 0; i < p.verts.size(); i++ )
		t.all.Add( p.verts[i] );
	// A viewpoint with a sample in a solid leaf (cluster -1): bw_vis keys its rule for
	// viewpoints in solid off exactly those samples (s_SolidEyes), so the list names
	// every viewpoint that has one, not only those wholly in solid. One entry per
	// piece here (its first such sample); the gather keeps one per viewpoint.
	// Viewpoint pieces are points and segments: their samples are the tiles' own.
	// Every such sample goes to EyeVolume::solidEyes as well: they are the tile samples of a
	// viewpoint piece (its vertices and a segment's points), and the tiles' dedupe keeps a
	// viewpoint's sample at its point, so the gather's sort and dedupe give the samples of
	// viewpoints in solid of the whole eye set, in its order (#39).
	if ( IsViewpointSource( p.source ) )
	{
		const TileRect everywhere = { -1e30, -1e30, 1e30, 1e30 };
		t.viewSamples.clear();
		for ( size_t i = 0; i < p.verts.size(); i++ )
			PushIn( t.viewSamples, p.verts[i], p.source, -1, everywhere );
		if ( p.kind == EYE_SEGMENT )
			SegmentSamples( p.verts[0], p.verts[1], p.source, s_EyeSpacing, -1, everywhere, t.viewSamples );
		bool hit = false;
		for ( size_t i = 0; i < t.viewSamples.size(); i++ )
		{
			dvec3 at( t.viewSamples[i].x, t.viewSamples[i].y, t.viewSamples[i].z );
			if ( dleafs[PointLeaf( at )].cluster >= 0 )
				continue;
			t.solidEyes.push_back( t.viewSamples[i] );
			if ( hit )
				continue;
			hit = true;
			SolidViewHit h;
			h.viewpoint = p.viewpoint;
			h.part = p.viewPart;
			h.source = p.source;
			h.at = at;
			t.solidViews.push_back( h );
		}
	}
	if ( s_pEyeOpts->keepPieces )
		t.keep.push_back( p );
	if ( p.kind == EYE_POLY )
		RecordChordHits( t, p );
	MarkTiles( t, 0, EYE_TILE, p.mins, p.maxs );
	FanCandidates( t, p, 0, s_pEyeOpts->fanSpacing, -1 );
	for ( size_t k = 0; k < ev.skyOrigins.size(); k++ )
	{
		EyePiece m;
		SkyMapPiece( p, ev.skyOrigins[k], ev.skyScales[k], m );
		m.sky = (int)k + 1;
		t.pieceCl.clear();
		PieceClusters( m, t.clusters );
		if ( under )
			under[k + 1] = m.kind == EYE_POLY ? UnderCoverSteps( m, s_EyeSpacing / ev.skyScales[k] ) : 0;
		FanCandidates( t, m, (int)k + 1, s_pEyeOpts->fanSpacing / ev.skyScales[k], (int)k );
		t.mapped++;
		for ( size_t i = 0; i < m.verts.size(); i++ )
			t.all.Add( m.verts[i] );
		if ( s_pEyeOpts->keepPieces )
			t.keep.push_back( m );
		// a mapped piece is 1/scale the size, and so are its tiles
		MarkTiles( t, (int)k + 1, EYE_TILE / ev.skyScales[k], m.mins, m.maxs );
	}
	t_pRegion = NULL;
	t_pPieceCl = NULL;
}

// A piece the tiles need that no node rebuilds: a ride sweep or special piece.
static void EmitListed( EyeThread &t, int pass, int item, const EyePiece &p )
{
	ListedPiece lp;
	lp.under.assign( s_nLayers, 0 );
	EmitPiece( t, p, true, lp.under.data() );
	lp.pass = pass;
	lp.item = item;
	lp.piece = p;
	t.listed.push_back( lp );
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

static void RideSweep( EyeThread &t, int item, const MoverEntity &me, int p, int q, const DTransform &xp,
					   const DTransform &xq, const std::vector<dvec3> &verts )
{
	std::vector<dvec3> pts;
	double pad;
	RideSweepPoints( me, p, q, xp, xq, verts, pts, pad );
	EyePiece sw;
	if ( HullPiece( pts, pad, EYESRC_RIDE, sw ) )
		EmitListed( t, 0, item, sw );
}

// A node's eye pieces, all collected in t.eye (the ride sweep moves each):
// - the crouch-feet column raised by eye_crouch;
// - with s_EyeStandLift, the stand-feet column raised by eye_stand;
// - the dead view, eye_dead up to the crouch eye: that prism lies
//   inside the crouch box on the floor, so it needs no obstacle clip.
// The piece pass and every tile that needs the node build them with this, so
// both get the same pieces (#39). Returns the columns that hit FEET_MAX_PIECES.
// Every piece lies over the node's polygon, or within EYE_FLAT_PAD of it (a
// thickened hull): inside its XY box grown by that.
static int NodePieces( EyeThread &t, int ni )
{
	const WalkGraph &g = *s_pEyeGraph;
	const PlayerProfile &prof = *s_pEyeProfile;
	const WalkNode &node = g.nodes[ni];
	double rise = s_EyeRise[ni];
	double dead = MIN( prof.eyeDead, prof.eyeCrouch );
	std::vector<const GrownPolytope *> none;
	int capped = 0;
	t.eye.clear();

	dvec3 mins = node.mins, maxs = node.maxs;
	maxs.z += rise;
	QueryObstacles( t, s_EyeCrouchObs, mins, maxs );
	t.pieces.clear();
	capped += FeetColumn( node.poly, node.plane, rise, t.obs, WALK_SHRINK, t.pieces );
	for ( size_t p = 0; p < t.pieces.size(); p++ )
		t.eye.push_back( Translated( t.pieces[p], dvec3( 0, 0, prof.eyeCrouch ) ) );

	double standRise = rise - s_EyeStandLift;
	if ( s_EyeStandLift > 0 && standRise > 0 )
	{
		maxs = node.maxs;
		maxs.z += standRise;
		QueryObstacles( t, s_EyeStandObs, mins, maxs );
		t.tmp.clear();
		capped += FeetColumn( node.poly, node.plane, standRise, t.obs, WALK_SHRINK, t.tmp );
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
		t.eye[p].source = EYESRC_NODE;
	return capped;
}

static void NodeEyesOne( int iThread, int item )
{
	EyeThread &t = s_EyeThreads[iThread];
	const WalkGraph &g = *s_pEyeGraph;
	const PlayerProfile &prof = *s_pEyeProfile;
	int ni = s_EyeNodes[item];
	const WalkNode &node = g.nodes[ni];
	double rise = s_EyeRise[ni];
	double dead = MIN( prof.eyeDead, prof.eyeCrouch );
	t.cappedColumns += NodePieces( t, ni );
	XYBox box = { DBL_MAX, DBL_MAX, -DBL_MAX, -DBL_MAX };
	std::vector<int> &under = s_NodeUnder[item];
	under.assign( t.eye.size() * s_nLayers, 0 );
	for ( size_t p = 0; p < t.eye.size(); p++ )
	{
		EmitPiece( t, t.eye[p], true, &under[p * s_nLayers] );
		box.x0 = MIN( box.x0, t.eye[p].mins.x );
		box.y0 = MIN( box.y0, t.eye[p].mins.y );
		box.x1 = MAX( box.x1, t.eye[p].maxs.x );
		box.y1 = MAX( box.y1, t.eye[p].maxs.y );
	}
	s_NodeBox[item] = box;
	s_NodeUsed[item] = !t.eye.empty();

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
					EmitListed( t, 0, item, sw );
				continue;
			}
			for ( size_t p = 0; p < t.eye.size(); p++ )
				RideSweep( t, item, me, node.pose, other, xp, xq, t.eye[p].verts );
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
static void VolumeEyes( EyeThread &t, int item, const GrownPolytope &vol, int source )
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
		EmitListed( t, 1, item, hull );
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
			VolumeEyes( t, item, v.volumes[k], EYESRC_WATER );
	}
	else if ( v.kind == SPECIAL_LADDER_BRUSH )
	{
		for ( size_t k = 0; k < v.volumes.size(); k++ )
			VolumeEyes( t, item, v.volumes[k], EYESRC_LADDER );
	}
	else if ( v.kind == SPECIAL_PUSH )
	{
		// a push can hold a player in the air (a lift balancing gravity): its
		// whole touch volume, not only the traced flights through it
		for ( size_t k = 0; k < v.volumes.size(); k++ )
			VolumeEyes( t, item, v.volumes[k], EYESRC_ARC );
	}
	else if ( v.kind == SPECIAL_ARRIVAL )
	{
		// a landmark arrival: anywhere in the shifted touch region
		for ( size_t k = 0; k < v.volumes.size(); k++ )
			VolumeEyes( t, item, v.volumes[k], EYESRC_TELEPORT );
	}
	else if ( v.kind == SPECIAL_LADDER )
	{
		// the climber stands on the segment; it may duck, or die there
		EmitListed( t, 1, item, SegmentPiece( v.a + dvec3( 0, 0, prof.eyeStand ), v.b + dvec3( 0, 0, prof.eyeStand ), EYESRC_LADDER ) );
		EmitListed( t, 1, item, SegmentPiece( v.a + dvec3( 0, 0, prof.eyeCrouch ), v.b + dvec3( 0, 0, prof.eyeCrouch ), EYESRC_LADDER ) );
		double dead = MIN( prof.eyeDead, prof.eyeCrouch );
		if ( dead < prof.eyeCrouch )
			EmitListed( t, 1, item, SegmentPiece( v.a + dvec3( 0, 0, dead ), v.b + dvec3( 0, 0, dead ), EYESRC_LADDER ) );
	}
}

// Traced free arcs: one segment per chord at the crouch eye (every arc is
// traced crouched). Ducking or unducking in the air moves the feet by the hull
// difference and leaves the eye where it is, so a flight's eye curve is fixed
// at launch. The rule (#42): an arc gets further curves above its crouch curve
// where a player on it can have his eye, by its posture (WalkGraph::freeArcEye,
// set where the arc is made):
//  - ARC_EYE_LIFTED, a duck-jump (traced lifted by crouch_lift): its crouch curve
//    f + crouch_lift + eye_crouch is the standing jump's eye; only where
//    eye_stand - eye_crouch exceeds crouch_lift (tf: 23 against 20) is the
//    standing jump's eye above it, by the difference.
//  - ARC_EYE_STAND, an arc not lifted whose launch can be standing (a walk-off,
//    a surf arc, a push, catapult, ladder or teleport launch, a special's jump):
//    the stand eye, f + eye_stand.
//  - ARC_EYE_CROUCHED, a plain jump from the feet point, traced crouched: a
//    player standing can jump along that same path, so it gets the stand eye
//    f + eye_stand as well. Its lifted duck-jump twin does not stand in for that:
//    it is solved from crouch_lift higher to the same landing, so it is another
//    flight (time, speed and path), and this arc is kept both when that twin is
//    blocked and when it is merely not the pair's one kept arc. Whether a standing
//    box fits along it is not traced (every arc is traced crouched), so the stand
//    eye is emitted always: more eyes only keep more faces visible. In the air a
//    crouched box may unduck as well: the box grows down by crouch_lift, so its
//    eye is the crouch eye plus the gap above (if any).
// The chords are the tracer's own: nSeg[0] over [0, tKick] (or [0, tEnd]
// without a kick), nSeg[1] over [tKick, tEnd], so each stays within its sagitta bound.
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

// A free arc's further eye curves, by its posture (see above), as heights above
// the traced path into z; returns how many (0 to 2): the stand eye of an arc that
// may start standing or is a plain jump, and the unducking crouch box's gap above
// the crouch eye of a crouched or lifted one. Surf flights are eye paths already,
// one per eye (bw_arcs.cpp SurfFlights), and get none.
static int ArcStandEye( int posture, const PlayerProfile &prof, double z[2] )
{
	int n = 0;
	double gap = prof.eyeStand - prof.eyeCrouch - prof.crouchLift;
	if ( ( posture == ARC_EYE_STAND || posture == ARC_EYE_CROUCHED ) && prof.eyeStand > prof.eyeCrouch )
		z[n++] = prof.eyeStand;
	if ( posture != ARC_EYE_STAND && gap > 0 )
		z[n++] = prof.eyeCrouch + gap;	// a crouch box unducking in the air drops its feet by crouch_lift: its eye rises by the gap (tf 3u)
	return n;
}

// freeArcs[i]'s posture; an arc with none recorded may start standing (the safe side).
static int FreeArcPosture( const WalkGraph &g, size_t i )
{
	return i < g.freeArcEye.size() ? g.freeArcEye[i] : ARC_EYE_STAND;
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
// posture >= 0 adds ArcStandEye's curves for it. Writes the arc's path chords,
// chord by chord and eye by eye, from 'out' on, and returns how many (with out
// NULL it only counts them).
static int ArcChordEyes( std::vector<float> &tt, const BwArc &a, double eyeZ, int source, int posture, PathChord *out )
{
	float g = (float)s_pEyeProfile->gravity;
	const PlayerProfile &prof = *s_pEyeProfile;
	dvec3 eyes[3] = { dvec3( 0, 0, eyeZ ), dvec3( 0, 0, 0 ), dvec3( 0, 0, 0 ) };
	int nEyes = 1;
	double extraZ[2];
	int extra = posture >= 0 ? ArcStandEye( posture, prof, extraZ ) : 0;
	for ( int e = 0; e < extra; e++ )
		eyes[nEyes++] = dvec3( 0, 0, extraZ[e] );
	tt.resize( (size_t)a.nSeg[0] + a.nSeg[1] + 3 );
	int n = ArcChordTimes( a, tt.data() );
	if ( !out )
		return n * nEyes;
	int w = 0;
	dvec3 prev( a.ox, a.oy, ArcZ( a, g, 0.0f ) );
	for ( int k = 1; k <= n; k++ )
	{
		dvec3 p( a.ox + a.vx * tt[k], a.oy + a.vy * tt[k], ArcZ( a, g, tt[k] ) );
		for ( int e = 0; e < nEyes; e++ )
		{
			out[w].a = prev + eyes[e];
			out[w].b = p + eyes[e];
			out[w].source = source;
			w++;
		}
		prev = p;
	}
	return w;
}

// Path item i: free arc i (crouch eye, its posture's curves), then the surf
// flights (bw_arcs.cpp SurfFlights: eye paths already, at either eye).
static int PathItemChords( std::vector<float> &tt, int item, PathChord *out )
{
	const WalkGraph &g = *s_pEyeGraph;
	int nFree = (int)g.freeArcs.size();
	if ( item < nFree )
		return ArcChordEyes( tt, g.freeArcs[item], s_pEyeProfile->eyeCrouch, EYESRC_ARC, FreeArcPosture( g, item ), out );
	return ArcChordEyes( tt, g.surfEyes[item - nFree], 0.0, EYESRC_SURF, -1, out );
}

static std::vector<size_t> s_PathFirst;		// per path item: its first chord in s_PathChords
static std::vector<TileSeg> s_PathParts;	// the chords' kept parts, and each clipped chord whole, in chord order

static void PathChordsOne( int iThread, int item )
{
	PathItemChords( s_EyeThreads[iThread].arcTimes, item, &s_PathChords[s_PathFirst[item]] );
}

static void PathPartsOne( int iThread, int item )
{
	const TileSeg &s = s_PathParts[item];
	if ( s.kind == TILESEG_SEGMENT )
		EmitPiece( s_EyeThreads[iThread], SegmentPiece( s.a, s.b, s.source ), false );
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

// A tile's layer geometry: the rectangle, the lattice step, and the skybox mapping.
struct TileFrame
{
	int L;					// layer: 0 real, k + 1 sky camera k
	double scale, sp;
	dvec3 origin;
	TileRect r;
	double gx0, gy0, gx1, gy1;	// what reaches the tile's lattice lines (TileLineRange), in the layer's coordinates
};

static void MakeTileFrame( int tile, TileFrame &f )
{
	const EyeVolume &ev = *s_pEyeVol;
	const TileKey &key = s_Tiles[tile];
	f.L = key.layer;
	f.scale = f.L ? ev.skyScales[f.L - 1] : 1.0;
	f.origin = f.L ? ev.skyOrigins[f.L - 1] : dvec3( 0, 0, 0 );
	const double side = EYE_TILE / f.scale;
	f.sp = s_EyeSpacing / f.scale;
	TileRect r = { key.tx * side, key.ty * side, ( key.tx + 1 ) * side, ( key.ty + 1 ) * side };
	f.r = r;
	double i0, i1, j0, j1;
	TileLineRange( r, f.sp, s_bShell, i0, i1, j0, j1 );
	const double grow = EYE_TOUCH + TILE_PAD;
	f.gx0 = MIN( i0 * f.sp, r.x0 ) - grow;
	f.gx1 = MAX( i1 * f.sp, r.x1 ) + grow;
	f.gy0 = MIN( j0 * f.sp, r.y0 ) - grow;
	f.gy1 = MAX( j1 * f.sp, r.y1 ) + grow;
}

static inline bool FrameMeets( const TileFrame &f, const dvec3 &mins, const dvec3 &maxs )
{
	return !( maxs.x < f.gx0 || mins.x > f.gx1 || maxs.y < f.gy0 || mins.y > f.gy1 );
}

// A clipped path chord in the frame's layer, or false when its wanted stretch misses the frame.
static bool FrameSpare( const TileFrame &f, const TileSeg &s, SpareChord &c )
{
	c.a = f.L ? f.origin + s.a * ( 1.0 / f.scale ) : s.a;
	c.b = f.L ? f.origin + s.b * ( 1.0 / f.scale ) : s.b;
	c.u0 = s.u0;
	c.u1 = s.u1;
	c.source = s.source;
	dvec3 p = c.a + ( c.b - c.a ) * c.u0, q = c.a + ( c.b - c.a ) * c.u1;
	return !( MAX( p.x, q.x ) < f.gx0 || MIN( p.x, q.x ) > f.gx1 || MAX( p.y, q.y ) < f.gy0 || MIN( p.y, q.y ) > f.gy1 );
}

// One tile's samples (#39), sorted by (x, y, z), each with the leaf cluster of its float
// position, into t.tileOut. The pieces are every piece of the piece pass that reaches the
// tile's lattice lines (TileLineRange) or the tile, in its layer: the reached nodes' pieces
// rebuilt by NodePieces, the listed polys and the segments and points, each mapped into the
// skybox for a sky layer. It reads only the tile and what the piece pass leaves (s_NodeHash,
// s_NodeBox, s_NodeUnder, s_TilePolys, s_TileSegs, s_TileHash, the graph, s_EyeRise, the
// obstacle sets), so a tile made again gives the same samples. Returns the samples made
// before the dedupe.
static size_t TileSamples( EyeThread &t, int tile, std::vector<int> *spareClusters = NULL )
{
	TileFrame f;
	MakeTileFrame( tile, f );
	const int L = f.L;
	// the frame in real coordinates, for the XY indexes
	const double rx0 = ( f.gx0 - f.origin.x ) * f.scale, rx1 = ( f.gx1 - f.origin.x ) * f.scale;
	const double ry0 = ( f.gy0 - f.origin.y ) * f.scale, ry1 = ( f.gy1 - f.origin.y ) * f.scale;
	std::vector<EyePiece> &pieces = t.tilePieces;
	pieces.clear();
	t.tileUnder.clear();
	t.spare.clear();
	auto add = [&]( const EyePiece &p, int under ) {
		if ( L )
		{
			pieces.push_back( EyePiece() );
			SkyMapPiece( p, f.origin, f.scale, pieces.back() );
		}
		else
			pieces.push_back( p );
		const EyePiece &m = pieces.back();
		if ( !FrameMeets( f, m.mins, m.maxs ) )
			pieces.pop_back();
		else
			t.tileUnder.push_back( under );
	};

	t.chordCand.clear();
	s_NodeHash.Query( rx0, ry0, rx1, ry1, t.chordCand );
	for ( size_t k = 0; k < t.chordCand.size(); k++ )
	{
		const int item = t.chordCand[k];
		const XYBox &b = s_NodeBox[item];
		if ( b.x1 < rx0 || b.x0 > rx1 || b.y1 < ry0 || b.y0 > ry1 )
			continue;
		NodePieces( t, s_EyeNodes[item] );
		const std::vector<int> &under = s_NodeUnder[item];
		if ( under.size() != t.eye.size() * s_nLayers )
			Error( "bspwalk: eye tile %d: node %d rebuilt %d pieces, the piece pass made %d\n", tile, s_EyeNodes[item],
				   (int)t.eye.size(), (int)( under.size() / s_nLayers ) );
		for ( size_t p = 0; p < t.eye.size(); p++ )
			add( t.eye[p], under[p * s_nLayers + L] );
	}
	t.chordCand.clear();
	s_TileHash.Query( rx0, ry0, rx1, ry1, t.chordCand );
	const int nPolys = (int)s_TilePolys.size();
	for ( size_t k = 0; k < t.chordCand.size(); k++ )
	{
		int id = t.chordCand[k];
		if ( id < nPolys )
		{
			add( s_TilePolys[id], s_TilePolyUnder[(size_t)id * s_nLayers + L] );
			continue;
		}
		const TileSeg &s = s_TileSegs[id - nPolys];
		if ( s.kind != TILESEG_SPARE )
		{
			add( s.kind == TILESEG_POINT ? PointPiece( s.a, s.source ) : SegmentPiece( s.a, s.b, s.source ), 0 );
			continue;
		}
		SpareChord c;
		if ( FrameSpare( f, s, c ) )
			t.spare.push_back( c );
	}

	t.tilePtrs.resize( pieces.size() );
	for ( size_t i = 0; i < pieces.size(); i++ )
		t.tilePtrs[i] = &pieces[i];
	t.tileOut.clear();
	return SampleTilePieces( t.tilePtrs.data(), t.tilePtrs.size(), t.spare.data(), t.spare.size(), f.sp, s_bShell, true, L - 1,
							 f.r, t.tile, t.tileOut, t.tileUnder.data(), spareClusters );
}

// The cluster a sample of tile 'tile' is kept under (#39). Its float position's leaf
// cluster, when that is -1 (solid: no eye cluster) or an eye cluster (EyeVolume::clusters,
// the clusters the pieces touch, which tier A used); when no piece of this tile touches
// that cluster (rounding moved the sample into a neighbouring leaf) it is counted
// ('offTile': the store finds it all the same, the cluster facts read every tile).
// Otherwise (rounding put it in a cluster no piece touches at all) the cluster of its
// unrounded position, a point of the piece that made it, when that is an eye cluster:
// counted ('unrecorded'). Neither: cluster -1, counted ('unplaced'); it is then no eye
// with vis data. Never a cluster outside EyeVolume::clusters: tier A has run, and an eye
// in a cluster it did not use could see faces it already hid.
static int TileSampleCluster( int tile, const EyeSample &s, EyeTileData &out )
{
	const int c = s.cluster;
	if ( c < 0 )
		return c;
	if ( c < s_pStore->NumClusters() && s_pStore->PieceCluster( c ) )
	{
		out.offTile += !s_pStore->TileInCluster( tile, c );
		return c;
	}
	const int u = dleafs[PointLeaf( dvec3( s.ux, s.uy, s.uz ) )].cluster;
	if ( u >= 0 && u < s_pStore->NumClusters() && s_pStore->PieceCluster( u ) )
	{
		out.unrecorded++;
		return u;
	}
	out.unplaced++;
	return -1;
}

// t.tileOut (sorted by (x, y, z)) as a stored tile: grouped by cluster (ascending, -1
// first), each group in the tile's order.
static void PackTile( EyeThread &t, int tile, EyeTileData &out )
{
	std::vector<EyeSample> &smp = t.tileOut;
	const int n = (int)smp.size();
	for ( int i = 0; i < n; i++ )
		smp[i].cluster = TileSampleCluster( tile, smp[i], out );
	t.order.resize( n );
	for ( int i = 0; i < n; i++ )
		t.order[i] = i;
	std::stable_sort( t.order.begin(), t.order.end(), [&smp]( int a, int b ) { return smp[a].cluster < smp[b].cluster; } );
	out.xyz.resize( (size_t)n * 3 );
	out.source.resize( n );
	out.groupCl.clear();
	out.groupOfs.clear();
	for ( int k = 0; k < n; k++ )
	{
		const EyeSample &s = smp[t.order[k]];
		if ( k == 0 || s.cluster != smp[t.order[k - 1]].cluster )
		{
			out.groupCl.push_back( s.cluster );
			out.groupOfs.push_back( k );
		}
		out.xyz[(size_t)k * 3] = s.x;
		out.xyz[(size_t)k * 3 + 1] = s.y;
		out.xyz[(size_t)k * 3 + 2] = s.z;
		out.source[k] = (unsigned char)s.source;
	}
	out.groupOfs.push_back( n );
	out.groupCl.shrink_to_fit();
	out.groupOfs.shrink_to_fit();
}

// The generator's scratch: a pool, one taken per tile being made (so at most one per
// thread making tiles at once), all freed by TrimEyeScratch while no tile is being made.
// Thread-local scratch would hold every worker's largest tile for the rest of the run.
static std::mutex s_GenPoolMutex;
static std::vector< std::unique_ptr<EyeThread> > s_GenAll;
static std::vector<EyeThread *> s_GenFree;

struct GenScratch
{
	EyeThread *t;
	GenScratch()
	{
		std::lock_guard<std::mutex> lock( s_GenPoolMutex );
		if ( s_GenFree.empty() )
		{
			s_GenAll.push_back( std::unique_ptr<EyeThread>( new EyeThread ) );
			t = s_GenAll.back().get();
		}
		else
		{
			t = s_GenFree.back();
			s_GenFree.pop_back();
		}
	}
	~GenScratch()
	{
		std::lock_guard<std::mutex> lock( s_GenPoolMutex );
		s_GenFree.push_back( t );
	}
};

void TrimEyeScratch()
{
	std::lock_guard<std::mutex> lock( s_GenPoolMutex );
	if ( s_GenFree.size() != s_GenAll.size() )
		Error( "bspwalk: TrimEyeScratch while an eye tile is being made\n" );
	s_GenFree.clear();
	s_GenAll.clear();
}

// The store's generator (EyeTileGenFn).
static void GenerateTile( void *, int tile, EyeTileData &out )
{
	GenScratch scratch;
	EyeThread &t = *scratch.t;
	out.made = (long long)TileSamples( t, tile );
	PackTile( t, tile, out );
	if ( out.unrecorded && !s_UnrecordedWarned.exchange( 1 ) )
		Warning( "bspwalk: eye tile %d has %lld sample(s) whose float position lies in a cluster no eye piece touches; they "
				 "take the cluster of their unrounded position (the eyes: line after the visibility stage counts them all)\n",
				 tile, out.unrecorded );
	if ( out.unplaced && !s_UnplacedWarned.exchange( 1 ) )
		Warning( "bspwalk: eye tile %d has %lld sample(s) in no cluster an eye piece touches, rounded or not; they are kept "
				 "as cluster -1, not as eyes (the eyes: line after the visibility stage counts them all)\n", tile, out.unplaced );
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
// The drift radius ignores walls, so a launch is kept off nodes it cannot reach
// through the air at all (#38): the flight's hull centre moves through the
// non-solid leaves of the world BSP, from leaf to adjacent leaf, so it stays in
// one connected set of them (an air component) and only passes over a node
// whose own air is in that component. The column it would raise is the node's
// FeetColumn, cut at the first obstacle over the floor: every eye in it sits on
// a free vertical path up from the node, so in the node's component too. A
// portal between two leaves lies in each one's bounds (vbsp writes a leaf's
// bounds from its portals, truncated to shorts: grown by LEAF_PAD here) and on
// the other's cell, so joining each leaf to every leaf whose cell meets its grown
// bounds (BoxLeaves, a superset) merges at least every real link: the
// components are never split wrongly, only possibly merged. Solid leaves include
// vbsp's filled outside, which a walked player never enters (it is cut off from
// every entity by structural brushes). A point in solid, or in a leaf with no
// bounds, has no component (-1) and prunes nothing.
static const double LEAF_PAD = 2.0;

struct FallLaunch
{
	XYBox box;
	double top;			// feet height
	int comp;			// air component, -1 when unknown
	int node;			// a rim launch: its node, whose component is looked up when first used; else -1
};

static std::vector<FallLaunch> s_FallLaunch;
static std::vector<int> s_LeafComp;		// per leaf: air component, -1 solid or unbounded
// per node: NodeComp, worked out by the first envelope thread that needs it (every
// thread gets the same value, so the race is benign); FALL_COMP_UNSET until then
static std::unique_ptr< std::atomic<int>[] > s_FallNodeComp;
static const int FALL_COMP_UNSET = INT_MIN;
static int NodeComp( const WalkNode &node );

static int FallNodeComp( int ni )
{
	int c = s_FallNodeComp[ni].load( std::memory_order_relaxed );
	if ( c == FALL_COMP_UNSET )
	{
		c = NodeComp( s_pEyeGraph->nodes[ni] );
		s_FallNodeComp[ni].store( c, std::memory_order_relaxed );
	}
	return c;
}
static int s_FallComps;
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
	int bComp = FallNodeComp( ni );
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
			if ( bComp >= 0 )
			{
				int lComp = L.node >= 0 ? FallNodeComp( L.node ) : L.comp;
				if ( lComp >= 0 && lComp != bComp )
					continue;	// no air path from the launch to B (#38)
			}
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

static void AddFallLaunch( double x0, double y0, double x1, double y1, double top, int comp, int node = -1 )
{
	FallLaunch L;
	L.box.x0 = x0;
	L.box.y0 = y0;
	L.box.x1 = x1;
	L.box.y1 = y1;
	L.top = top;
	L.comp = comp;
	L.node = node;
	s_FallLaunch.push_back( L );
}

static int FindComp( std::vector<int> &parent, int a )
{
	while ( parent[a] != a )
	{
		parent[a] = parent[parent[a]];
		a = parent[a];
	}
	return a;
}

// Appends the leaves whose cells meet the box (touching counts) under BSP node 'node'.
static void BoxLeaves( int node, const dvec3 &mins, const dvec3 &maxs, std::vector<int> &out )
{
	while ( node >= 0 )
	{
		const dnode_t &dn = dnodes[node];
		const dplane_t &pl = dplanes[dn.planenum];
		dvec3 n( pl.normal.x, pl.normal.y, pl.normal.z );
		// the box's extent along the normal
		double lo = 0, hi = 0;
		lo += n.x >= 0 ? n.x * mins.x : n.x * maxs.x;
		hi += n.x >= 0 ? n.x * maxs.x : n.x * mins.x;
		lo += n.y >= 0 ? n.y * mins.y : n.y * maxs.y;
		hi += n.y >= 0 ? n.y * maxs.y : n.y * mins.y;
		lo += n.z >= 0 ? n.z * mins.z : n.z * maxs.z;
		hi += n.z >= 0 ? n.z * maxs.z : n.z * mins.z;
		bool front = hi >= pl.dist - 1e-3, back = lo <= pl.dist + 1e-3;
		if ( front && back )
		{
			BoxLeaves( dn.children[0], mins, maxs, out );
			node = dn.children[1];
		}
		else
			node = dn.children[front ? 0 : 1];
	}
	out.push_back( -1 - node );
}

// s_LeafComp: the non-solid leaves of the world, each joined to every non-solid
// leaf whose cell meets its grown bounds (see LEAF_PAD): every portal of a leaf lies
// in its bounds and on its neighbour's cell, so every real link is among them.
static void BuildLeafComponents()
{
	s_LeafComp.assign( numleafs, -1 );
	std::vector<unsigned char> use( numleafs, 0 );
	for ( int l = 0; l < numleafs; l++ )
	{
		const dleaf_t &lf = dleafs[l];
		use[l] = !( lf.contents & CONTENTS_SOLID ) && lf.mins[0] <= lf.maxs[0] && lf.mins[1] <= lf.maxs[1] &&
				 lf.mins[2] <= lf.maxs[2];
	}
	std::vector<int> parent( numleafs );
	for ( int l = 0; l < numleafs; l++ )
		parent[l] = l;
	std::vector<int> cand;
	for ( int l = 0; l < numleafs; l++ )
	{
		if ( !use[l] )
			continue;
		const dleaf_t &lf = dleafs[l];
		dvec3 mins( lf.mins[0] - LEAF_PAD, lf.mins[1] - LEAF_PAD, lf.mins[2] - LEAF_PAD );
		dvec3 maxs( lf.maxs[0] + LEAF_PAD, lf.maxs[1] + LEAF_PAD, lf.maxs[2] + LEAF_PAD );
		cand.clear();
		BoxLeaves( dmodels[0].headnode, mins, maxs, cand );
		for ( size_t k = 0; k < cand.size(); k++ )
		{
			int m = cand[k];
			if ( m < 0 || m >= numleafs || m == l || !use[m] )
				continue;
			int ra = FindComp( parent, l ), rb = FindComp( parent, m );
			if ( ra != rb )
				parent[MAX( ra, rb )] = MIN( ra, rb );
		}
	}
	s_FallComps = 0;
	std::vector<int> label( numleafs, -1 );
	for ( int l = 0; l < numleafs; l++ )
	{
		if ( !use[l] )
			continue;
		int r = FindComp( parent, l );
		if ( label[r] < 0 )
			label[r] = s_FallComps++;
		s_LeafComp[l] = label[r];
	}
}

static int PointComp( const dvec3 &p )
{
	return s_LeafComp.empty() ? -1 : s_LeafComp[PointLeaf( p )];
}

// A node's air: its crouch hull centre over the middle of its (convex) polygon,
// where the crouch hull fits.
static int NodeComp( const WalkNode &node )
{
	int n = (int)node.poly.size();
	if ( n < 3 || !( node.plane.n.z > 0 ) )
		return -1;
	dvec3 c( 0, 0, 0 );
	for ( int i = 0; i < n; i++ )
		c += node.poly[i];
	c = c * ( 1.0 / n );
	c.z = node.Z( c.x, c.y ) + 0.5 * s_pEyeProfile->crouchHeight;
	return PointComp( c );
}

// Raises s_EyeRise over every reached node a fall can pass over (s_EyeNodes set).
static void BuildFallColumns( const PlayerProfile &profile, const WalkGraph &graph, double baseRise )
{
	s_FallLaunch.clear();
	int n = (int)graph.nodes.size();
	BuildLeafComponents();
	s_FallNodeComp.reset( new std::atomic<int>[n] );
	for ( int i = 0; i < n; i++ )
		s_FallNodeComp[i].store( FALL_COMP_UNSET, std::memory_order_relaxed );
	// a point launch's air: its crouch hull centre
	dvec3 up( 0, 0, 0.5 * profile.crouchHeight );
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
			AddFallLaunch( A.mins.x, A.mins.y, A.maxs.x, A.maxs.y, A.maxs.z, -1, i );
		}
	}
	for ( size_t i = 0; i < graph.seeds.size(); i++ )
	{
		const dvec3 &p = graph.seeds[i].point;
		AddFallLaunch( p.x, p.y, p.x, p.y, p.z, PointComp( p + up ) );
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
			AddFallLaunch( p.x, p.y, p.x, p.y, p.z, PointComp( p + up ) );
		}
	}
	for ( size_t v = 0; v < sg.verts.size(); v++ )
	{
		const SpecialVertex &sv = sg.verts[v];
		if ( sv.kind != SPECIAL_LADDER || v >= sg.reached.size() || !sg.reached[v] )
			continue;
		// a climber may let go anywhere on the ladder
		AddFallLaunch( MIN( sv.a.x, sv.b.x ), MIN( sv.a.y, sv.b.y ), MAX( sv.a.x, sv.b.x ), MAX( sv.a.y, sv.b.y ),
					   MAX( sv.a.z, sv.b.z ), PointComp( sv.a + up ) );
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
				AddFallLaunch( gp.mins.x, gp.mins.y, gp.maxs.x, gp.maxs.y, gp.maxs.z, -1 );	// a volume: unknown
		}
	}
	if ( s_FallLaunch.empty() || s_EyeNodes.empty() )
	{
		std::vector<int>().swap( s_LeafComp );
		s_FallNodeComp.reset();
		return;
	}

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
	std::vector<int>().swap( s_LeafComp );
	s_FallNodeComp.reset();
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
		double extraZ[2];
		int curves = 1 + ( isFree ? ArcStandEye( FreeArcPosture( g, i ), prof, extraZ ) : 0 );
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
		t.keep.clear();
		memset( t.pieces_, 0, sizeof( t.pieces_ ) );
		t.mapped = 0;
		t.solidViews.clear();
		t.cappedColumns = 0;
		t.all = EyeBound();
		t.chordHits.clear();
		t.listed.clear();
		t.tileKeys.clear();
		t.tileCompactAt = (size_t)1 << 16;
		t.fanCands.clear();
		t.fanCompactAt = (size_t)1 << 16;
		t.pieceCl.clear();
		t.clTiles.clear();
		t.clTileCompactAt = (size_t)1 << 16;
		t.solidEyes.clear();
	}
	s_nLayers = 1 + (int)eyes.skyOrigins.size();

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
	Msg( "eyes: fall columns raise %d of %d reached nodes (%d air components; %.2f s)\n", raised,
		 (int)s_EyeNodes.size(), s_FallComps,
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
	// env_cubemaps: a camera inside a brush, or looking at a toolsnodraw face, is not a viewpoint (bw_cubemap.h)
	CheckCubemaps( opts.cubemaps, eyes.cubemaps );
	for ( size_t i = 0; i < eyes.cubemaps.list.size(); i++ )
	{
		const CubemapInfo &c = eyes.cubemaps.list[i];
		if ( c.status == CUBEMAP_OK )
			pushView( PointPiece( c.origin, EYESRC_CUBEMAP ), 0 );
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

	s_bShell = s_EyeSpacing <= EYE_SHELL_MAX_SPACING;
	eyes.shell = s_bShell;

	// path chords (#39): every chord of every eye curve, in path order, indexed by
	// XY so that the real poly pieces record the stretches they hold
	{
		const int nFree = (int)graph.freeArcs.size(), nPath = nFree + (int)graph.surfEyes.size();
		s_PathFirst.assign( (size_t)nPath + 1, 0 );
		std::vector<float> tt;
		for ( int i = 0; i < nPath; i++ )
		{
			int c = PathItemChords( tt, i, NULL );
			s_PathFirst[i + 1] = s_PathFirst[i] + c;
			eyes.pieces[i < nFree ? EYESRC_ARC : EYESRC_SURF] += c;
		}
		if ( s_PathFirst[nPath] > (size_t)INT_MAX )
			Error( "bspwalk: %.0f path chords, more than an int indexes\n", (double)s_PathFirst[nPath] );
		s_PathChords.resize( s_PathFirst[nPath] );
		RunEyePass( nPath, PathChordsOne );
		std::vector<XYBox> boxes( s_PathChords.size() );
		for ( size_t i = 0; i < s_PathChords.size(); i++ )
		{
			const PathChord &c = s_PathChords[i];
			XYBox b = { MIN( c.a.x, c.b.x ), MIN( c.a.y, c.b.y ), MAX( c.a.x, c.b.x ), MAX( c.a.y, c.b.y ) };
			boxes[i] = b;
		}
		s_PathHash.Build( boxes, std::vector<unsigned char>( boxes.size(), 1 ) );
	}

	s_NodeBox.assign( s_EyeNodes.size(), XYBox() );
	s_NodeUsed.assign( s_EyeNodes.size(), 0 );
	s_NodeUnder.clear();
	s_NodeUnder.resize( s_EyeNodes.size() );
	RunEyePass( (int)s_EyeNodes.size(), NodeEyesOne );
	RunEyePass( (int)s_EyeSpecial.size(), SpecialEyesOne );

	// each chord's parts outside every real poly piece (#39): the stretches the
	// pieces recorded, sorted by chord (no thread order shows), merged, and the rest
	{
		std::vector<ChordHit> hits;
		size_t nHits = 0;
		for ( int i = 0; i <= MAX_TOOL_THREADS; i++ )
			nHits += s_EyeThreads[i].chordHits.size();
		hits.reserve( nHits );
		for ( int i = 0; i <= MAX_TOOL_THREADS; i++ )
		{
			std::vector<ChordHit> &h = s_EyeThreads[i].chordHits;
			hits.insert( hits.end(), h.begin(), h.end() );
			std::vector<ChordHit>().swap( h );
		}
		std::sort( hits.begin(), hits.end(), []( const ChordHit &a, const ChordHit &b ) {
			if ( a.chord != b.chord )
				return a.chord < b.chord;
			if ( a.t0 != b.t0 )
				return a.t0 < b.t0;
			return a.t1 < b.t1;
		} );
		s_PathParts.clear();
		std::vector< std::pair<double, double> > iv;
		std::vector<dvec3> ends;
		size_t h = 0;
		for ( size_t c = 0; c < s_PathChords.size(); c++ )
		{
			iv.clear();
			for ( ; h < hits.size() && hits[h].chord == (int)c; h++ )
				iv.push_back( std::make_pair( hits[h].t0, hits[h].t1 ) );
			const PathChord &pc = s_PathChords[c];
			ends.clear();
			eyes.pathLength += DLength( pc.b - pc.a );
			eyes.pathLengthKept += ChordKeptParts( pc.a, pc.b, iv.data(), iv.size(), ends );
			for ( size_t k = 0; k + 1 < ends.size(); k += 2 )
			{
				TileSeg s = { ends[k], ends[k + 1], 0.0, 1.0, pc.source, TILESEG_SEGMENT };
				s_PathParts.push_back( s );
				eyes.pathParts++;
			}
			// a clipped chord's own samples stay candidates for the tiles' cluster rule
			if ( !iv.empty() )
			{
				TileSeg s = { pc.a, pc.b, 0.0, 1.0, pc.source, TILESEG_SPARE };
				s_PathParts.push_back( s );
			}
		}
		eyes.pathChords = (long long)s_PathChords.size();
		std::vector<PathChord>().swap( s_PathChords );
		std::vector<size_t>().swap( s_PathFirst );
		s_PathHash = CXYHash();
	}
	RunEyePass( (int)s_PathParts.size(), PathPartsOne );
	RunEyePass( (int)s_EyeLoose.size(), LooseEyesOne );

	// gather
	std::vector<SolidViewHit> solidHits;
	std::vector<ListedPiece> listed;
	std::vector<TileKey> tileKeys;
	int capped = 0;
	eyes.clusters.assign( nClusters, 0 );
	for ( int i = 0; i <= MAX_TOOL_THREADS; i++ )
	{
		EyeThread &t = s_EyeThreads[i];
		for ( int c = 0; c < nClusters; c++ )
			eyes.clusters[c] |= t.clusters[c];
		if ( opts.keepPieces )
			eyes.keep.insert( eyes.keep.end(), t.keep.begin(), t.keep.end() );
		for ( int s = 0; s < EYESRC_COUNT; s++ )
			eyes.pieces[s] += t.pieces_[s];
		eyes.mappedPieces += t.mapped;
		solidHits.insert( solidHits.end(), t.solidViews.begin(), t.solidViews.end() );
		capped += t.cappedColumns;
		eyes.all.Merge( t.all );
		for ( size_t k = 0; k < t.listed.size(); k++ )
		{
			listed.push_back( ListedPiece() );
			std::swap( listed.back(), t.listed[k] );
		}
		tileKeys.insert( tileKeys.end(), t.tileKeys.begin(), t.tileKeys.end() );
		std::vector<EyePiece>().swap( t.keep );
		std::vector<EyePiece>().swap( t.pieces );
		std::vector<EyePiece>().swap( t.tmp );
		std::vector<EyePiece>().swap( t.eye );
		std::vector<ListedPiece>().swap( t.listed );
		std::vector<TileKey>().swap( t.tileKeys );
	}
	if ( capped )
		Msg( "eyes: %d node columns were cut into more than %d pieces; the rest of their obstacles were skipped "
			 "(those columns over-reach)\n", capped, (int)FEET_MAX_PIECES );
	// the fan eyes: of each cell's candidates over every thread the first in sample order, so
	// no thread order shows
	{
		std::vector<FanCand> fan;
		size_t nFan = 0;
		for ( int i = 0; i <= MAX_TOOL_THREADS; i++ )
			nFan += s_EyeThreads[i].fanCands.size();
		fan.reserve( nFan );
		for ( int i = 0; i <= MAX_TOOL_THREADS; i++ )
		{
			std::vector<FanCand> &f = s_EyeThreads[i].fanCands;
			fan.insert( fan.end(), f.begin(), f.end() );
			std::vector<FanCand>().swap( f );
			std::vector<EyeSample>().swap( s_EyeThreads[i].fanSamples );
		}
		CompactFanCands( fan );
		eyes.fanEyes.resize( fan.size() );
		for ( size_t i = 0; i < fan.size(); i++ )
			eyes.fanEyes[i] = fan[i].eye;
	}
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

	// The tier B eye store (#39). The tiles read the reached nodes by XY, the listed
	// pieces (in (pass, item) order: one thread made each item's, in order) and the
	// segments and points. Every tile any piece reaches is listed in canonical order
	// (layer: real, then sky camera 0, 1, ...; tx; ty), and each cluster lists the
	// tiles of the pieces that touch it. No tile depends on another, so the store
	// makes any tile on demand, as often as its cache needs, and these inputs stay
	// until FreeEyeVolume.
	double tTiles = Plat_FloatTime();
	std::stable_sort( listed.begin(), listed.end(), []( const ListedPiece &a, const ListedPiece &b ) {
		return a.pass != b.pass ? a.pass < b.pass : a.item < b.item;
	} );
	s_TilePolys.clear();
	s_TilePolyUnder.clear();
	s_TileSegs.clear();
	for ( size_t k = 0; k < listed.size(); k++ )
	{
		EyePiece &p = listed[k].piece;
		if ( p.kind == EYE_POLY )
		{
			s_TilePolys.push_back( EyePiece() );
			std::swap( s_TilePolys.back(), p );
			s_TilePolyUnder.insert( s_TilePolyUnder.end(), listed[k].under.begin(), listed[k].under.end() );
		}
		else if ( p.kind == EYE_SEGMENT )
		{
			TileSeg s = { p.verts[0], p.verts[1], 0.0, 1.0, p.source, TILESEG_SEGMENT };
			s_TileSegs.push_back( s );
		}
	}
	std::vector<ListedPiece>().swap( listed );
	s_TileSegs.insert( s_TileSegs.end(), s_PathParts.begin(), s_PathParts.end() );
	std::vector<TileSeg>().swap( s_PathParts );
	for ( size_t i = 0; i < s_EyeLoose.size(); i++ )
	{
		const EyePiece &p = s_EyeLoose[i];
		bool point = p.kind == EYE_POINT;
		TileSeg s = { p.verts[0], point ? p.verts[0] : p.verts[1], 0.0, 1.0, p.source, point ? TILESEG_POINT : TILESEG_SEGMENT };
		s_TileSegs.push_back( s );
	}
	{
		std::vector<XYBox> boxes;
		boxes.reserve( s_TilePolys.size() + s_TileSegs.size() );
		for ( size_t i = 0; i < s_TilePolys.size(); i++ )
		{
			XYBox b = { s_TilePolys[i].mins.x, s_TilePolys[i].mins.y, s_TilePolys[i].maxs.x, s_TilePolys[i].maxs.y };
			boxes.push_back( b );
		}
		for ( size_t i = 0; i < s_TileSegs.size(); i++ )
		{
			const TileSeg &s = s_TileSegs[i];
			dvec3 a = s.a + ( s.b - s.a ) * s.u0, b = s.a + ( s.b - s.a ) * s.u1;
			XYBox box = { MIN( a.x, b.x ), MIN( a.y, b.y ), MAX( a.x, b.x ), MAX( a.y, b.y ) };
			boxes.push_back( box );
		}
		s_TileHash.Build( boxes, std::vector<unsigned char>( boxes.size(), 1 ) );
		s_NodeHash.Build( s_NodeBox, s_NodeUsed );
	}
	std::sort( tileKeys.begin(), tileKeys.end() );
	tileKeys.erase( std::unique( tileKeys.begin(), tileKeys.end() ), tileKeys.end() );
	s_Tiles.swap( tileKeys );
	std::vector<TileKey>().swap( tileKeys );
	if ( s_Tiles.size() > (size_t)EYE_STORE_MAX_TILES )
		Error( "bspwalk: %.0f eye tiles, more than the %d an eye id holds; raise -eyespacing\n", (double)s_Tiles.size(),
			   EYE_STORE_MAX_TILES );
	eyes.tiles = (int)s_Tiles.size();

	EyeStoreSetup setup;
	setup.numTiles = (int)s_Tiles.size();
	setup.numClusters = nClusters;
	setup.tileSky.resize( s_Tiles.size() );
	for ( size_t i = 0; i < s_Tiles.size(); i++ )
		setup.tileSky[i] = s_Tiles[i].layer - 1;
	// each cluster's tiles: every thread's (cluster, tile) pairs, merged and mapped to tile ids
	{
		std::vector<ClusterTile> all;
		size_t nAll = 0;
		for ( int i = 0; i <= MAX_TOOL_THREADS; i++ )
			nAll += s_EyeThreads[i].clTiles.size();
		all.reserve( nAll );
		for ( int i = 0; i <= MAX_TOOL_THREADS; i++ )
		{
			std::vector<ClusterTile> &c = s_EyeThreads[i].clTiles;
			all.insert( all.end(), c.begin(), c.end() );
			std::vector<ClusterTile>().swap( c );
		}
		std::sort( all.begin(), all.end() );
		all.erase( std::unique( all.begin(), all.end() ), all.end() );
		setup.clTileOfs.assign( (size_t)nClusters + 1, 0 );
		setup.clTiles.resize( all.size() );
		for ( size_t i = 0; i < all.size(); i++ )
		{
			std::vector<TileKey>::const_iterator it = std::lower_bound( s_Tiles.begin(), s_Tiles.end(), all[i].key );
			if ( it == s_Tiles.end() || !( *it == all[i].key ) )
				Error( "bspwalk: an eye piece's tile (%d %d %d) is missing from the tile list\n", all[i].key.layer, all[i].key.tx,
					   all[i].key.ty );
			setup.clTiles[i] = (int)( it - s_Tiles.begin() );
			setup.clTileOfs[all[i].cluster + 1]++;
		}
		for ( int c = 0; c < nClusters; c++ )
			setup.clTileOfs[c + 1] += setup.clTileOfs[c];
	}
	setup.pieceClusters = eyes.pieceClusters;
	// The piece parts' bound per cluster (MarkRegion). The store merges each cluster's samples
	// into it: the descent does not cut a flat piece (one with no thickness; it fails every
	// clip), so only its corners reach MarkRegion and the samples along it in the clusters
	// between lie outside the bound of the parts (on d1_trainstation_02, 120 samples outside
	// the box of the parts, 271 outside their k-DOP). A sample is an eye position, so the
	// region holds it, and with it the stretch between collinear samples.
	setup.partBound.assign( nClusters, EyeBound() );
	for ( int i = 0; i <= MAX_TOOL_THREADS; i++ )
	{
		const EyeThread &t = s_EyeThreads[i];
		for ( int c = 0; c < nClusters && c < (int)t.region.size(); c++ )
			setup.partBound[c].Merge( t.region[c] );
	}
	for ( int i = 0; i <= MAX_TOOL_THREADS; i++ )
		std::vector<EyeBound>().swap( s_EyeThreads[i].region );
	setup.budgetBytes = opts.cacheMB * 1024.0 * 1024.0;
	setup.gen = GenerateTile;
	setup.ctx = NULL;
	eyes.store.reset( new EyeStore( setup ) );
	s_pStore = eyes.store.get();
	s_UnrecordedWarned = 0;
	s_UnplacedWarned = 0;

	// the viewpoint samples in solid, in the eye set's order (the real layer's tiles by (tx,
	// ty), each by (x, y, z)), one per point: the one the tile's dedupe keeps
	{
		std::vector<EyeSample> &se = eyes.solidEyes;
		se.clear();
		for ( int i = 0; i <= MAX_TOOL_THREADS; i++ )
		{
			std::vector<EyeSample> &v = s_EyeThreads[i].solidEyes;
			se.insert( se.end(), v.begin(), v.end() );
			std::vector<EyeSample>().swap( v );
		}
		std::sort( se.begin(), se.end(), []( const EyeSample &a, const EyeSample &b ) {
			double ta = floor( a.x / EYE_TILE ), tb = floor( b.x / EYE_TILE );
			if ( ta != tb )
				return ta < tb;
			ta = floor( a.y / EYE_TILE );
			tb = floor( b.y / EYE_TILE );
			if ( ta != tb )
				return ta < tb;
			return SampleOrder( a, b );
		} );
		se.erase( std::unique( se.begin(), se.end(), []( const EyeSample &a, const EyeSample &b ) {
			return a.x == b.x && a.y == b.y && a.z == b.z;
		} ), se.end() );
	}
	tTiles = Plat_FloatTime() - tTiles;

	// scratch the piece pass leaves (the generator has its own, per thread)
	for ( int i = 0; i <= MAX_TOOL_THREADS; i++ )
	{
		EyeThread &t = s_EyeThreads[i];
		std::vector<EyePiece>().swap( t.tilePieces );
		std::vector<const EyePiece *>().swap( t.tilePtrs );
		std::vector<SpareChord>().swap( t.spare );
		t.tile = TileScratch();
		std::vector<EyeSample>().swap( t.viewSamples );
		std::vector<EyePiece>().swap( t.eye );
		std::vector<EyePiece>().swap( t.pieces );
		std::vector<EyePiece>().swap( t.tmp );
		std::vector<int>().swap( t.pieceCl );
	}
	std::vector<EyePiece>().swap( s_EyeLoose );
	eyes.tBuild = Plat_FloatTime() - t0;

	Msg( "eyes:" );
	for ( int s = 0; s < EYESRC_COUNT; s++ )
		Msg( " %s %d", EyeSourceName( s ), eyes.pieces[s] );
	Msg( " pieces%s; %d of %d clusters; %d viewpoints; samples at %gu (%.0f estimated), made per tile on demand; %.2f s\n",
		 eyes.sky ? " (and each mapped into the 3D skybox)" : "", eyes.eyeClusters, nClusters, eyes.viewpoints, s_EyeSpacing,
		 est, eyes.tBuild );
	Msg( "eyes: path chords %lld (%.0fu), %lld parts kept outside the poly pieces (%.0fu); shell %s; %d tiles, a %.0f MB cache "
		 "(%.2f s)\n",
		 eyes.pathChords, eyes.pathLength, eyes.pathParts, eyes.pathLengthKept, eyes.shell ? "on" : "off (spacing over 16u)",
		 eyes.tiles, opts.cacheMB, tTiles );
	for ( size_t k = 0; k < eyes.skyOrigins.size(); k++ )
		Msg( "eyes: sky_camera at (%.1f %.1f %.1f), scale %g\n", eyes.skyOrigins[k].x, eyes.skyOrigins[k].y,
			 eyes.skyOrigins[k].z, eyes.skyScales[k] );
}

void PrintEyeStoreStats( const EyeVolume &eyes )
{
	EyeStore *store = eyes.store.get();
	if ( !store )
		return;
	store->ComputeAll();	// done by the visibility stage; here for a run that skipped it
	EyeStoreStats st = store->Stats();
	Msg( "eyes: %lld samples in %d tiles (%.0f made); %d eye clusters the lattice missed; a %.0f MB cache: %d tiles "
		 "generated, %lld again after eviction, peak %.1f MB resident; %.2f s generating (summed over threads)\n",
		 st.samples, st.tilesGenerated, (double)st.made, (int)store->BareClusters().size(), store->BudgetBytes() / ( 1024.0 * 1024.0 ),
		 st.tilesGenerated, st.regenerations, st.peakBytes / ( 1024.0 * 1024.0 ), st.genSeconds );
	if ( st.offTile || st.unrecorded || st.unplaced )
		Msg( "eyes: %lld sample(s) in an eye cluster no piece of their tile touches (kept), %lld took the cluster of their "
			 "unrounded position, %lld were left in cluster -1 (no eye piece touches their cluster, rounded or not)\n",
			 st.offTile, st.unrecorded, st.unplaced );
}

// -eyeverify: every piece of each layer, mapped into it, and every clipped path chord.
static std::vector< std::vector<EyePiece> > s_VerifyPieces;
static std::vector< std::vector<SpareChord> > s_VerifySpare;
static std::atomic<int> s_VerifyDiffer, s_VerifyRule, s_VerifyReported;
static std::atomic<long long> s_VerifySamples;

static bool SameTile( const EyeTileData &a, const EyeTileData &b )
{
	return a.xyz == b.xyz && a.source == b.source && a.groupCl == b.groupCl && a.groupOfs == b.groupOfs && a.made == b.made &&
		   a.offTile == b.offTile && a.unrecorded == b.unrecorded && a.unplaced == b.unplaced;
}

static void VerifyTileOne( int, int tile )
{
	GenScratch scratch;
	EyeThread &t = *scratch.t;
	EyeTileData a, b;
	a.made = (long long)TileSamples( t, tile );
	PackTile( t, tile, a );
	TileFrame f;
	MakeTileFrame( tile, f );
	const std::vector<EyePiece> &all = s_VerifyPieces[f.L];
	const std::vector<SpareChord> &spare = s_VerifySpare[f.L];
	t.tilePtrs.resize( all.size() );
	for ( size_t i = 0; i < all.size(); i++ )
		t.tilePtrs[i] = &all[i];
	t.tileOut.clear();
	std::vector<int> spareCl, kept;
	b.made = (long long)SampleTilePieces( t.tilePtrs.data(), t.tilePtrs.size(), spare.data(), spare.size(), f.sp, s_bShell, true,
										  f.L - 1, f.r, t.tile, t.tileOut, NULL, &spareCl );
	// the cluster rule's postcondition: every cluster one of the candidates it looked at lies in
	// has a kept sample of the tile (leaf clusters of the float positions, as the rule reads them)
	for ( size_t i = 0; i < t.tileOut.size(); i++ )
		kept.push_back( t.tileOut[i].cluster );
	std::sort( kept.begin(), kept.end() );
	int missing = 0;
	for ( size_t i = 0; i < spareCl.size(); i++ )
		missing += !std::binary_search( kept.begin(), kept.end(), spareCl[i] );
	PackTile( t, tile, b );
	// and the store's: no sample takes a cluster tier A did not use (EyeVolume::clusters)
	for ( size_t g = 0; g < a.groupCl.size(); g++ )
		missing += a.groupCl[g] >= 0 && ( a.groupCl[g] >= s_pStore->NumClusters() || !s_pStore->PieceCluster( a.groupCl[g] ) );
	s_VerifySamples += a.Count();
	const TileKey &key = s_Tiles[tile];
	bool differ = !SameTile( a, b );
	if ( differ )
		s_VerifyDiffer++;
	if ( missing )
		s_VerifyRule++;
	if ( ( differ || missing ) && s_VerifyReported++ < 20 )
		Warning( "bspwalk: -eyeverify: tile %d (layer %d, %d %d): %s%s%d samples (%lld made) from the tile's pieces, %d (%lld made) "
				 "from every piece; %d cluster(s) of the rule's candidates have no kept sample or are no eye cluster\n", tile,
				 key.layer, key.tx, key.ty,
				 differ ? "the samples differ; " : "", missing ? "the cluster rule fails; " : "", a.Count(), a.made, b.Count(),
				 b.made, missing );
}

bool VerifyEyeTiles( EyeVolume &eyes )
{
	if ( !eyes.store )
		return true;
	double t0 = Plat_FloatTime();
	std::vector<EyePiece> real;
	{
		GenScratch scratch;
		EyeThread &t = *scratch.t;
		for ( size_t k = 0; k < s_EyeNodes.size(); k++ )
		{
			if ( !s_NodeUsed[k] )
				continue;
			NodePieces( t, s_EyeNodes[k] );
			real.insert( real.end(), t.eye.begin(), t.eye.end() );
		}
	}
	real.insert( real.end(), s_TilePolys.begin(), s_TilePolys.end() );
	std::vector<TileSeg> spareSegs;
	for ( size_t i = 0; i < s_TileSegs.size(); i++ )
	{
		const TileSeg &s = s_TileSegs[i];
		if ( s.kind == TILESEG_SPARE )
			spareSegs.push_back( s );
		else
			real.push_back( s.kind == TILESEG_POINT ? PointPiece( s.a, s.source ) : SegmentPiece( s.a, s.b, s.source ) );
	}
	s_VerifyPieces.assign( s_nLayers, std::vector<EyePiece>() );
	s_VerifySpare.assign( s_nLayers, std::vector<SpareChord>() );
	for ( int L = 0; L < s_nLayers; L++ )
	{
		TileFrame f;
		f.L = L;
		f.scale = L ? eyes.skyScales[L - 1] : 1.0;
		f.origin = L ? eyes.skyOrigins[L - 1] : dvec3( 0, 0, 0 );
		f.gx0 = f.gy0 = -1e300;
		f.gx1 = f.gy1 = 1e300;
		std::vector<EyePiece> &out = s_VerifyPieces[L];
		out.resize( real.size() );
		for ( size_t i = 0; i < real.size(); i++ )
		{
			if ( L )
				SkyMapPiece( real[i], f.origin, f.scale, out[i] );
			else
				out[i] = real[i];
		}
		for ( size_t i = 0; i < spareSegs.size(); i++ )
		{
			SpareChord c;
			FrameSpare( f, spareSegs[i], c );
			s_VerifySpare[L].push_back( c );
		}
	}
	s_VerifyDiffer = 0;
	s_VerifyRule = 0;
	s_VerifyReported = 0;
	s_VerifySamples = 0;
	RunThreadsBatched( eyes.tiles, VerifyTileOne );
	std::vector< std::vector<EyePiece> >().swap( s_VerifyPieces );
	std::vector< std::vector<SpareChord> >().swap( s_VerifySpare );
	TrimEyeScratch();
	bool ok = !s_VerifyDiffer && !s_VerifyRule;
	Msg( "eyeverify: %d tiles, %lld samples, %d pieces per layer: %d tile(s) differ from the brute-force set, %d break the "
		 "cluster rule: %s (%.2f s)\n", eyes.tiles, s_VerifySamples.load(), (int)real.size(), s_VerifyDiffer.load(),
		 s_VerifyRule.load(), ok ? "PASS" : "FAIL", Plat_FloatTime() - t0 );
	return ok;
}

void FreeEyeVolume( EyeVolume &eyes )
{
	eyes.store.reset();
	s_pStore = NULL;
	s_NodeHash = CXYHash();
	std::vector<XYBox>().swap( s_NodeBox );
	std::vector<unsigned char>().swap( s_NodeUsed );
	std::vector< std::vector<int> >().swap( s_NodeUnder );
	s_TileHash = CXYHash();
	std::vector<EyePiece>().swap( s_TilePolys );
	std::vector<int>().swap( s_TilePolyUnder );
	std::vector<TileSeg>().swap( s_TileSegs );
	std::vector<TileKey>().swap( s_Tiles );
	s_EyeCrouchObs = ObstacleSet();
	s_EyeStandObs = ObstacleSet();
	s_bStandColumn = false;
	std::vector<int>().swap( s_EyeNodes );
	std::vector<double>().swap( s_EyeRise );
	TrimEyeScratch();
}
