//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: Stage 4 (#14), the eye volume. See bw_vis.h for the parts and
//          docs/bsp_walker_rebuild.md ("How #14 visibility works") for why each
//          part covers where a player's eye can be, and the accepted
//          limitations for the flights it does not hold.
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

// Winding helpers: the same base size and chop epsilon as bw_grow.cpp.
static const double EYE_BASE_HALFSIZE = 131072.0;
static const double EYE_CHOP_EPS = 1e-9;
// Vertices closer than this are one.
static const double EYE_WELD = 1e-7;
// A plane separates two convexes when every vertex of one is at least this
// close to its outside: touching is free.
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
												 "point_camera", "point_viewcontrol", "cli", "sky_camera" };
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
			// a later plane that coincides with this one (same side, w all on
			// it) is the same side: it does not chop this face away. The later
			// one's face is chopped by this one and dropped, so one copy stays;
			// chopping both would drop both and leave the piece unbounded there.
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

// Every vertex of 'verts' is at least -EYE_TOUCH outside (n, d): nothing of it
// is strictly inside the half-space n . x < d.
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
static thread_local int t_SampleMapped;	// set while a skybox copy is sampled (EmitPiece runs on worker threads)

// Per thread and cluster (EmitPiece points this at its EyeThread while it
// runs): the box of every part of an eye piece the descent put in the cluster
// (a side that fails to build adds that side's vertices), so the box holds
// every eye position the pieces give the cluster. An eye cluster with no
// lattice sample in it (a piece that only grazes a thin leaf) cannot be ray
// tested; its box tells which faces it may still see (bw_vis BareMaySee), and
// the same box catches a cluster whose samples all lie behind a face its
// pieces reach in front of (bw_vis CandidateEyes).
struct EyeRegion
{
	dvec3 lo, hi;
	bool any;
};
static thread_local std::vector<EyeRegion> *t_pRegion;

static inline void MarkRegion( int leaf, const dvec3 *v, size_t n )
{
	if ( !t_pRegion || !n )
		return;
	int c = dleafs[leaf].cluster;
	if ( c < 0 || c >= (int)t_pRegion->size() )
		return;
	EyeRegion &r = ( *t_pRegion )[c];
	for ( size_t i = 0; i < n; i++ )
	{
		r.lo = r.any ? DMin( r.lo, v[i] ) : v[i];
		r.hi = r.any ? DMax( r.hi, v[i] ) : v[i];
		r.any = true;
	}
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
	s.mapped = t_SampleMapped;
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
struct EyeThread
{
	std::vector<unsigned char> clusters;
	std::vector<EyeSample> samples;
	std::vector<EyeRegion> region;		// per cluster, see MarkRegion (sized by the first piece it emits)
	std::vector<EyePiece> keep;
	std::vector<int> cand;
	std::vector<const GrownPolytope *> obs;
	std::vector<EyePiece> pieces, stand, tmp;	// a node's feet pieces: crouch column, stand column, dead prism
	int pieces_[EYESRC_COUNT];
	int mapped;
	bool solidEye;
	std::vector< std::pair<int, dvec3> > solidViews;	// viewpoints in solid: source and position
	int cappedColumns;									// columns that hit FEET_MAX_PIECES
	dvec3 allLo, allHi;									// box of every piece (skybox copies too)
	bool allAny;
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
static std::vector<int> s_MoverIndex;		// (convex, pose) -> graph.movers index, by s_MoverKey
static std::vector<long long> s_MoverKey;

// One finished eye piece: its clusters, its skybox copy, its samples.
// A viewpoint wholly in solid is a camera the engine draws from with no cluster:
// every cluster is then drawn (the view cluster -1 has no PVS), so every cluster
// becomes an eye cluster. Skybox copies of the eye volume are not viewpoints in
// that sense: the 3D skybox pass takes its PVS from the sky_camera origin (added
// as a viewpoint), and copies that reach into the skybox room's solid are common
// (every CS:S map tried has them) without the real world showing in the skybox.
bool IsViewpointSource( int source )
{
	return source == EYESRC_CUBEMAP || source == EYESRC_CAMERA || source == EYESRC_VIEWCONTROL || source == EYESRC_CLI ||
		   source == EYESRC_SKYCAMERA;
}

static void EmitPiece( EyeThread &t, const EyePiece &p )
{
	const EyeVolume &ev = *s_pEyeVol;
	if ( t.region.empty() )
	{
		EyeRegion empty;
		empty.lo = empty.hi = dvec3( 0, 0, 0 );
		empty.any = false;
		t.region.assign( t.clusters.size(), empty );
	}
	t_pRegion = &t.region;
	int flags = PieceClusters( p, t.clusters );
	if ( IsViewpointSource( p.source ) && flags == 2 )
	{
		t.solidEye = true;
		t.solidViews.push_back( std::make_pair( p.source, p.verts.empty() ? dvec3( 0, 0, 0 ) : p.verts[0] ) );
	}
	t.pieces_[p.source]++;
	t.allLo = t.allAny ? DMin( t.allLo, p.mins ) : p.mins;
	t.allHi = t.allAny ? DMax( t.allHi, p.maxs ) : p.maxs;
	t.allAny = true;
	SamplePiece( p, s_EyeSpacing, t.samples );
	if ( s_pEyeOpts->keepPieces )
		t.keep.push_back( p );
	for ( size_t k = 0; k < ev.skyOrigins.size(); k++ )
	{
		EyePiece m;
		SkyMapPiece( p, ev.skyOrigins[k], ev.skyScales[k], m );
		m.sky = (int)k + 1;
		t_SampleMapped = 1;
		PieceClusters( m, t.clusters );
		t.mapped++;
		t.allLo = DMin( t.allLo, m.mins );
		t.allHi = DMax( t.allHi, m.maxs );
		// a mapped piece is 1/scale the size: sample it 1/scale as finely
		SamplePiece( m, s_EyeSpacing / ev.skyScales[k], t.samples );
		t_SampleMapped = 0;
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

// The rotation taking pose p's frame to pose q's (Rq Rp^T) as an axis and an
// angle in [0, pi].
static double RelRotation( const DTransform &p, const DTransform &q, dvec3 &axis )
{
	double R[3][3];
	for ( int i = 0; i < 3; i++ )
		for ( int j = 0; j < 3; j++ )
			R[i][j] = q.m[i][0] * p.m[j][0] + q.m[i][1] * p.m[j][1] + q.m[i][2] * p.m[j][2];
	double c = ( R[0][0] + R[1][1] + R[2][2] - 1.0 ) * 0.5;
	double th = acos( MAX( -1.0, MIN( 1.0, c ) ) );
	axis = dvec3( 0, 0, 1 );
	if ( th < 1e-9 )
		return 0.0;
	if ( th > EYE_PI - 1e-6 )
	{
		// R + I has rank one along the axis: its largest column
		double best = -1;
		for ( int j = 0; j < 3; j++ )
		{
			dvec3 v( R[0][j] + ( j == 0 ? 1.0 : 0.0 ), R[1][j] + ( j == 1 ? 1.0 : 0.0 ), R[2][j] + ( j == 2 ? 1.0 : 0.0 ) );
			if ( DDot( v, v ) > best )
			{
				best = DDot( v, v );
				axis = v;
			}
		}
		DNormalize( axis );
		return th;
	}
	axis = dvec3( R[2][1] - R[1][2], R[0][2] - R[2][0], R[1][0] - R[0][1] ) * ( 1.0 / ( 2.0 * sin( th ) ) );
	DNormalize( axis );
	return th;
}

// The pose a fraction t of the way from p to q: the rotation turned by t of
// the angle about its axis, origin and scale linear.
static DTransform PoseBetween( const DTransform &p, const DTransform &q, const dvec3 &axis, double theta, double t )
{
	double c = cos( theta * t ), s = sin( theta * t ), C = 1.0 - c;
	double x = axis.x, y = axis.y, z = axis.z;
	double r[3][3] = { { c + x * x * C, x * y * C - z * s, x * z * C + y * s },
					   { y * x * C + z * s, c + y * y * C, y * z * C - x * s },
					   { z * x * C - y * s, z * y * C + x * s, c + z * z * C } };
	DTransform out;
	for ( int i = 0; i < 3; i++ )
		for ( int j = 0; j < 3; j++ )
			out.m[i][j] = r[i][0] * p.m[0][j] + r[i][1] * p.m[1][j] + r[i][2] * p.m[2][j];
	out.origin = p.origin + ( q.origin - p.origin ) * t;
	out.scale = p.scale + ( q.scale - p.scale ) * t;
	return out;
}

// A ride sweep: chords between poses bow in from the true path by at most this.
static const double RIDE_BULGE = 1.0;
static const int RIDE_MAX_STEPS = 32;

// Rides: each feet piece carried from pose xp to pose xq, through enough poses
// between that a chord bows in at most RIDE_BULGE (a rotating door's sweep is
// an arc), each step the hull of the piece at both ends; the upright eye offset
// is added after the mapping (a tilted mover still has an upright player).
static void SweepPieces( EyeThread &t, const std::vector<EyePiece> &feet, double eyeZ, const DTransform &xp,
						 const DTransform &xq )
{
	if ( feet.empty() )
		return;
	dvec3 axis;
	double theta = RelRotation( xp, xq, axis );
	double rad = 0;
	for ( size_t p = 0; p < feet.size(); p++ )
		for ( size_t k = 0; k < feet[p].verts.size(); k++ )
			rad = MAX( rad, DLength( feet[p].verts[k] - xp.origin ) );
	int steps = 1;
	if ( theta > 0 && rad > RIDE_BULGE )
	{
		// bulge of a chord over dtheta: rad (1 - cos(dtheta / 2))
		double dth = 2.0 * acos( 1.0 - RIDE_BULGE / rad );
		steps = MIN( RIDE_MAX_STEPS, MAX( 1, (int)ceil( theta / dth ) ) );
	}
	DTransform prev = xp;
	dvec3 eye( 0, 0, eyeZ );
	for ( int s = 1; s <= steps; s++ )
	{
		DTransform cur = s == steps ? xq : PoseBetween( xp, xq, axis, theta, (double)s / steps );
		for ( size_t p = 0; p < feet.size(); p++ )
		{
			std::vector<dvec3> pts;
			for ( size_t k = 0; k < feet[p].verts.size(); k++ )
			{
				pts.push_back( PoseToPose( xp, prev, feet[p].verts[k] ) + eye );
				pts.push_back( PoseToPose( xp, cur, feet[p].verts[k] ) + eye );
			}
			EyePiece sw;
			if ( EyePolyFromPoints( pts, sw ) )
			{
				sw.source = EYESRC_RIDE;
				EmitPiece( t, sw );
			}
		}
		prev = cur;
	}
}

static void EmitFeet( EyeThread &t, const std::vector<EyePiece> &feet, double eyeZ )
{
	for ( size_t p = 0; p < feet.size(); p++ )
	{
		EyePiece e = Translated( feet[p], dvec3( 0, 0, eyeZ ) );
		e.source = EYESRC_NODE;
		EmitPiece( t, e );
	}
}

static void NodeEyesOne( int iThread, int item )
{
	EyeThread &t = s_EyeThreads[iThread];
	const WalkGraph &g = *s_pEyeGraph;
	const PlayerProfile &prof = *s_pEyeProfile;
	int ni = s_EyeNodes[item];
	const WalkNode &node = g.nodes[ni];
	double rise = s_EyeRise[ni];

	dvec3 mins = node.mins, maxs = node.maxs;
	maxs.z += rise;
	QueryObstacles( t, s_EyeCrouchObs, mins, maxs );
	t.pieces.clear();
	t.cappedColumns += FeetColumn( node.poly, node.plane, rise, t.obs, WALK_SHRINK, t.pieces );
	if ( t.pieces.empty() )
	{
		// a sliver whose prism did not build: its polygon raised as segments,
		// from the dead eye up to the top of the crouch (and stand) column
		double lo = MIN( prof.eyeDead, prof.eyeCrouch ), hi = prof.eyeCrouch + rise;
		if ( s_bStandColumn )
			hi = MAX( hi, prof.eyeStand + MAX( rise - ( prof.eyeStand - prof.eyeCrouch ), 1.0 ) );
		for ( size_t k = 0; k < node.poly.size(); k++ )
			EmitPiece( t, SegmentPiece( node.poly[k] + dvec3( 0, 0, lo ), node.poly[k] + dvec3( 0, 0, hi ), EYESRC_NODE ) );
		return;
	}
	EmitFeet( t, t.pieces, prof.eyeCrouch );

	// The stand column, when the profile needs one: the free stand-feet column
	// up to the same eye top, raised by eye_stand. Without it a standing eye
	// at f + eye_stand + h is the crouch eye of the crouch box at
	// f + eye_stand - eye_crouch + h, which lies inside the free standing box at
	// f + h only when eye_stand - eye_crouch <= stand_height - crouch_height.
	t.stand.clear();
	if ( s_bStandColumn )
	{
		double rs = MAX( rise - ( prof.eyeStand - prof.eyeCrouch ), 1.0 );
		maxs.z = node.maxs.z + rs;
		QueryObstacles( t, s_EyeStandObs, mins, maxs );
		t.cappedColumns += FeetColumn( node.poly, node.plane, rs, t.obs, WALK_SHRINK, t.stand );
		EmitFeet( t, t.stand, prof.eyeStand );
	}

	// a dead player's view, eye_dead above the floor: the prism up to the
	// crouch eye lies inside the crouch box on the floor, so it is free
	t.tmp.clear();
	double dead = MIN( prof.eyeDead, prof.eyeCrouch );
	if ( dead < prof.eyeCrouch )
	{
		std::vector<const GrownPolytope *> none;
		FeetColumn( node.poly, node.plane, prof.eyeCrouch - dead, none, WALK_SHRINK, t.tmp );
		EmitFeet( t, t.tmp, dead );
	}

	// rides: a top swept with its mover to every linked pose
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
			SweepPieces( t, t.pieces, prof.eyeCrouch, xp, xq );
			SweepPieces( t, t.stand, prof.eyeStand, xp, xq );
			// Without a stand column a standing rider's eye is world-up of its
			// feet, eye_stand - eye_crouch above the crouch eye; the pieces are
			// built at this pose, and a rotation about a non-vertical axis tilts
			// that offset away from up in the sweep: sweep the crouch feet at the
			// standing eye too.
			dvec3 axis;
			double th = RelRotation( xp, xq, axis );
			if ( !s_bStandColumn && th > 1e-9 && fabs( axis.z ) < 1.0 - 1e-9 )
				SweepPieces( t, t.pieces, prof.eyeStand, xp, xq );
			SweepPieces( t, t.tmp, dead, xp, xq );
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

static void ArcEyesOne( int iThread, int item )
{
	EyeThread &t = s_EyeThreads[iThread];
	const BwArc &a = s_pEyeGraph->freeArcs[item];
	float g = (float)s_pEyeProfile->gravity;
	bool kick = a.tKick > 0.0f && a.tKick < a.tEnd;
	float t0[2] = { 0.0f, kick ? a.tKick : a.tEnd }, t1[2] = { kick ? a.tKick : a.tEnd, a.tEnd };
	int ns[2] = { MAX( 1, (int)a.nSeg[0] ), kick ? MAX( 1, (int)a.nSeg[1] ) : 0 };
	const PlayerProfile &prof = *s_pEyeProfile;
	dvec3 eyes[2] = { dvec3( 0, 0, prof.eyeCrouch ), dvec3( 0, 0, prof.eyeStand ) };
	int nEyes = 1;
	double gap = prof.eyeStand - prof.eyeCrouch - prof.crouchLift;
	if ( a.vz == 0.0f && !kick && a.dg0 == 0.0f )
		nEyes = 2;	// a walk-off may start standing
	else if ( gap > 0 )
	{
		// a standing jump's eye is above the lifted duck-jump's by the gap (tf 3u)
		eyes[1] = dvec3( 0, 0, prof.eyeCrouch + gap );
		nEyes = 2;
	}
	dvec3 prev( a.ox, a.oy, ArcZ( a, g, 0.0f ) );
	for ( int piece = 0; piece < 2; piece++ )
	{
		for ( int k = 1; k <= ns[piece]; k++ )
		{
			float tt = k == ns[piece] ? t1[piece] : t0[piece] + ( t1[piece] - t0[piece] ) * (float)k / (float)ns[piece];
			dvec3 p( a.ox + a.vx * tt, a.oy + a.vy * tt, ArcZ( a, g, tt ) );
			for ( int e = 0; e < nEyes; e++ )
				EmitPiece( t, SegmentPiece( prev + eyes[e], p + eyes[e], EYESRC_ARC ) );
			prev = p;
		}
	}
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
		t.solidEye = false;
		t.solidViews.clear();
		t.cappedColumns = 0;
		t.allAny = false;
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

	// node columns: max_jump_rise (the eye rise of a standing jump when that is
	// higher), raised over every node a fall can pass over (BuildFallColumns)
	int n = (int)graph.nodes.size();
	double baseRise = MAX( profile.maxJumpRise,
						   profile.jumpApex * ( 1 + profile.airJumps ) + profile.eyeStand - profile.eyeCrouch );
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

	// sample spacing: the requested one, widened until the node columns'
	// estimated sample count fits the cap: area / s^2 columns per layer (at
	// least one, however small the node), each with height / s + 2 points (the
	// crouch column, the stand column when built, the dead prism), times one
	// plus the skybox copies (a copy is sampled 1/scale as finely, so it has as
	// many). Water, ladder, arc, ride, teleport and viewpoint samples are not
	// counted.
	s_EyeSpacing = opts.spacing;
	double deadH = MAX( 0.0, profile.eyeCrouch - MIN( profile.eyeDead, profile.eyeCrouch ) );
	int layers = 1 + s_bStandColumn + ( deadH > 0 );
	std::vector<double> nodeArea( s_EyeNodes.size() ), nodeH( s_EyeNodes.size() );
	for ( size_t k = 0; k < s_EyeNodes.size(); k++ )
	{
		double r = s_EyeRise[s_EyeNodes[k]];
		nodeArea[k] = graph.nodes[s_EyeNodes[k]].area;
		nodeH[k] = r + deadH + ( s_bStandColumn ? MAX( r - ( profile.eyeStand - profile.eyeCrouch ), 1.0 ) : 0.0 );
	}
	const double maxSpacing = 4096;
	for ( ;; )
	{
		double s = s_EyeSpacing;
		double est = 0;
		for ( size_t k = 0; k < nodeArea.size(); k++ )
			est += MAX( nodeArea[k] / ( s * s ), 1.0 ) * ( nodeH[k] / s + 2.0 * layers );
		est *= 1.0 + eyes.skyOrigins.size();
		if ( est <= opts.maxSamples || s >= maxSpacing )
			break;
		s_EyeSpacing = MIN( s * 1.25, maxSpacing );
	}
	if ( s_EyeSpacing > opts.spacing )
		Msg( "eyes: sample spacing widened from %gu to %.2fu to keep the node columns at about %.0f samples or fewer "
			 "(-eyemaxsamples); tier B is coarser\n", opts.spacing, s_EyeSpacing, opts.maxSamples );
	eyes.spacing = s_EyeSpacing;
	RunEyePass( (int)s_EyeNodes.size(), NodeEyesOne );

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
	RunEyePass( (int)s_EyeSpecial.size(), SpecialEyesOne );

	// free arcs
	RunEyePass( (int)graph.freeArcs.size(), ArcEyesOne );

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
	for ( int i = 0; i < g_nCubemapSamples; i++ )
	{
		const dcubemapsample_t &c = g_CubemapSamples[i];
		s_EyeLoose.push_back( PointPiece( dvec3( c.origin[0], c.origin[1], c.origin[2] ), EYESRC_CUBEMAP ) );
	}
	for ( int e = 0; e < num_entities; e++ )
	{
		const char *cls = EntValue( &entities[e], "classname" );
		if ( !Q_stricmp( cls, "point_camera" ) )
			s_EyeLoose.push_back( PointPiece( EntOrigin( &entities[e] ), EYESRC_CAMERA ) );
		else if ( !Q_stricmp( cls, "point_viewcontrol" ) )
		{
			// the camera, and its moveto rail: straight from its origin to the
			// moveto corner, then corner to corner along the target links
			dvec3 prev = EntOrigin( &entities[e] );
			s_EyeLoose.push_back( PointPiece( prev, EYESRC_VIEWCONTROL ) );
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
				s_EyeLoose.push_back( SegmentPiece( prev, p, EYESRC_VIEWCONTROL ) );
				prev = p;
			}
		}
	}
	for ( size_t i = 0; i < opts.viewpoints.size(); i++ )
		s_EyeLoose.push_back( PointPiece( opts.viewpoints[i], EYESRC_CLI ) );
	for ( size_t k = 0; k < eyes.skyOrigins.size(); k++ )
		s_EyeLoose.push_back( PointPiece( eyes.skyOrigins[k], EYESRC_SKYCAMERA ) );
	for ( size_t i = 0; i < s_EyeLoose.size(); i++ )
	{
		int s = s_EyeLoose[i].source;
		eyes.viewpoints += IsViewpointSource( s );
	}
	RunEyePass( (int)s_EyeLoose.size(), LooseEyesOne );

	// gather
	std::vector< std::pair<int, dvec3> > solidViews;
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
		eyes.solidEye = eyes.solidEye || t.solidEye;
		solidViews.insert( solidViews.end(), t.solidViews.begin(), t.solidViews.end() );
		capped += t.cappedColumns;
		if ( t.allAny )
		{
			eyes.allLo = eyes.allAny ? DMin( eyes.allLo, t.allLo ) : t.allLo;
			eyes.allHi = eyes.allAny ? DMax( eyes.allHi, t.allHi ) : t.allHi;
			eyes.allAny = true;
		}
		std::vector<EyeSample>().swap( t.samples );
		std::vector<EyePiece>().swap( t.keep );
		std::vector<EyePiece>().swap( t.pieces );
		std::vector<EyePiece>().swap( t.stand );
		std::vector<EyePiece>().swap( t.tmp );
	}
	if ( capped )
		Msg( "eyes: %d node columns were cut into more than %d pieces; the rest of their obstacles were skipped "
			 "(those columns over-reach)\n", capped, (int)FEET_MAX_PIECES );
	eyes.pieceClusters = eyes.clusters;
	if ( eyes.solidEye )
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
		Warning( "bspwalk: %d viewpoint(s) lie in a solid leaf; the engine draws every cluster from there, so every "
				 "cluster is an eye cluster (tier A proves nothing). Move them into open space:\n", (int)solidViews.size() );
		for ( size_t i = 0; i < solidViews.size(); i++ )
			Warning( "  %s at (%.1f %.1f %.1f)\n", EyeSourceName( solidViews[i].first ), solidViews[i].second.x,
					 solidViews[i].second.y, solidViews[i].second.z );
		std::fill( eyes.clusters.begin(), eyes.clusters.end(), 1 );
	}
	for ( int c = 0; c < nClusters; c++ )
		eyes.eyeClusters += eyes.clusters[c];

	// dedupe the samples (adjacent pieces share grid points), then their clusters.
	// A skybox copy and a real sample at one point both stay (they are different
	// eyes). Of the samples at one point with the same mapped flag the one kept
	// is one of a viewpoint (bw_vis keys the eyes of viewpoints in solid off it),
	// then the lowest source: the same whichever thread made which.
	std::sort( eyes.samples.begin(), eyes.samples.end(), []( const EyeSample &a, const EyeSample &b ) {
		if ( a.x != b.x )
			return a.x < b.x;
		if ( a.y != b.y )
			return a.y < b.y;
		if ( a.z != b.z )
			return a.z < b.z;
		if ( a.mapped != b.mapped )
			return a.mapped < b.mapped;
		bool va = IsViewpointSource( a.source ), vb = IsViewpointSource( b.source );
		if ( va != vb )
			return va;
		return a.source < b.source;
	} );
	eyes.samples.erase( std::unique( eyes.samples.begin(), eyes.samples.end(), []( const EyeSample &a, const EyeSample &b ) {
		return a.x == b.x && a.y == b.y && a.z == b.z && a.mapped == b.mapped;
	} ), eyes.samples.end() );
	std::vector<int> perCluster( nClusters, 0 );
	for ( size_t i = 0; i < eyes.samples.size(); i++ )
	{
		const EyeSample &s = eyes.samples[i];
		int leaf = PointLeaf( dvec3( s.x, s.y, s.z ) );
		int c = eyes.samples[i].cluster = dleafs[leaf].cluster;
		if ( c >= 0 && c < nClusters )
			perCluster[c]++;
	}
	// per eye cluster the pieces touch: the box of the piece parts in it (no box
	// found: unbounded, every face may be seen from it). Eye clusters with no
	// sample in them are listed apart (bare).
	eyes.regionLo.assign( nClusters, dvec3( 0, 0, 0 ) );
	eyes.regionHi.assign( nClusters, dvec3( 0, 0, 0 ) );
	for ( int c = 0; c < nClusters; c++ )
	{
		if ( !eyes.pieceClusters[c] )
			continue;
		bool any = false;
		dvec3 lo( -1e30, -1e30, -1e30 ), hi( 1e30, 1e30, 1e30 );
		for ( int i = 0; i <= MAX_TOOL_THREADS; i++ )
		{
			const EyeThread &t = s_EyeThreads[i];
			if ( c >= (int)t.region.size() || !t.region[c].any )
				continue;
			lo = any ? DMin( lo, t.region[c].lo ) : t.region[c].lo;
			hi = any ? DMax( hi, t.region[c].hi ) : t.region[c].hi;
			any = true;
		}
		eyes.regionLo[c] = lo;
		eyes.regionHi[c] = hi;
		if ( perCluster[c] )
			continue;
		eyes.bareClusters.push_back( c );
		eyes.bareLo.push_back( lo );
		eyes.bareHi.push_back( hi );
	}
	for ( int i = 0; i <= MAX_TOOL_THREADS; i++ )
		std::vector<EyeRegion>().swap( s_EyeThreads[i].region );

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
	Msg( " pieces%s; %d of %d clusters; %d viewpoints; %d samples at %gu (%d eye clusters the lattice missed); %.2f s\n",
		 eyes.sky ? " (and each mapped into the 3D skybox)" : "", eyes.eyeClusters, nClusters, eyes.viewpoints,
		 (int)eyes.samples.size(), s_EyeSpacing, (int)eyes.bareClusters.size(), eyes.tBuild );
	for ( size_t k = 0; k < eyes.skyOrigins.size(); k++ )
		Msg( "eyes: sky_camera at (%.1f %.1f %.1f), scale %g\n", eyes.skyOrigins[k].x, eyes.skyOrigins[k].y,
			 eyes.skyOrigins[k].z, eyes.skyScales[k] );
}
