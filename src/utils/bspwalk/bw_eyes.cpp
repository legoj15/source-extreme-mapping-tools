//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: Stage 4 (#14), the eye volume. See bw_vis.h for the parts and
//          docs/bsp_walker_rebuild.md ("How #14 visibility works") for why each
//          is a superset of where a player's eye can be.
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
// BSP descent: a vertex within this of a node plane is on it.
static const double EYE_SPLIT_EPS = 1e-4;

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
			if ( j != i )
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

void FeetColumn( const DWinding &poly, const DPlane &plane, double rise,
				 const std::vector<const GrownPolytope *> &obstacles, double shrink, std::vector<EyePiece> &out )
{
	size_t first = out.size();
	int n = (int)poly.size();
	if ( n < 3 || !( plane.n.z > 0 ) )
		return;
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
		return;

	std::vector<EyePiece> pieces( 1, prism ), next;
	std::vector<DPlane> both;
	std::vector<dvec3> ext;
	EyePiece meet, extr;
	double H = rise + 1.0;
	for ( size_t o = 0; o < obstacles.size() && !pieces.empty(); o++ )
	{
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
	(void)first;
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

static void SegmentClusters( int node, dvec3 a, dvec3 b, std::vector<unsigned char> &clusters )
{
	while ( node >= 0 )
	{
		const dnode_t &dn = dnodes[node];
		const dplane_t &pl = dplanes[dn.planenum];
		dvec3 n( pl.normal.x, pl.normal.y, pl.normal.z );
		double da = DDot( n, a ) - pl.dist, db = DDot( n, b ) - pl.dist;
		if ( da >= -EYE_SPLIT_EPS && db >= -EYE_SPLIT_EPS )
		{
			node = dn.children[0];
			continue;
		}
		if ( da <= EYE_SPLIT_EPS && db <= EYE_SPLIT_EPS )
		{
			node = dn.children[1];
			continue;
		}
		double t = da / ( da - db );
		dvec3 m = a + ( b - a ) * t;
		if ( da > 0 )
		{
			SegmentClusters( dn.children[0], a, m, clusters );
			a = m;
			node = dn.children[1];
		}
		else
		{
			SegmentClusters( dn.children[1], a, m, clusters );
			a = m;
			node = dn.children[0];
		}
	}
	MarkLeaf( -1 - node, clusters );
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
		if ( lo >= -EYE_SPLIT_EPS )
		{
			node = dn.children[0];
			continue;
		}
		if ( hi <= EYE_SPLIT_EPS )
		{
			node = dn.children[1];
			continue;
		}
		// split; a side that fails to build (a sliver) gets the whole piece
		EyePiece front, back;
		bool fOk = EyePolyClip( p, -n, -(double)pl.dist, front );
		bool bOk = EyePolyClip( p, n, (double)pl.dist, back );
		PolyClusters( dn.children[0], fOk ? front : p, clusters );
		PolyClusters( dn.children[1], bOk ? back : p, clusters );
		return;
	}
	MarkLeaf( -1 - node, clusters );
}

int PieceClusters( const EyePiece &piece, std::vector<unsigned char> &clusters )
{
	t_LeafFlags = 0;
	if ( clusters.empty() || piece.verts.empty() )
		return 0;
	int head = dmodels[0].headnode;
	if ( piece.kind == EYE_POINT )
		MarkLeaf( PointLeaf( piece.verts[0] ), clusters );
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
static int t_SampleMapped;	// set while a skybox copy is sampled

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
	double x0 = ceil( p.mins.x / spacing ) * spacing, y0 = ceil( p.mins.y / spacing ) * spacing;
	for ( double x = x0; x <= p.maxs.x; x += spacing )
	{
		for ( double y = y0; y <= p.maxs.y; y += spacing )
		{
			double zlo = -1e300, zhi = 1e300;
			bool ok = true;
			for ( size_t k = 0; k < p.planes.size() && ok; k++ )
			{
				const DPlane &pl = p.planes[k];
				double r = pl.d - pl.n.x * x - pl.n.y * y;
				if ( pl.n.z > 1e-9 )
					zhi = MIN( zhi, r / pl.n.z );
				else if ( pl.n.z < -1e-9 )
					zlo = MAX( zlo, r / pl.n.z );
				else if ( r < 0 )
					ok = false;
			}
			if ( !ok || zlo > zhi || zlo < -1e299 || zhi > 1e299 )
				continue;
			PushSample( out, dvec3( x, y, zlo ), p.source );
			for ( double z = ceil( zlo / spacing ) * spacing; z < zhi; z += spacing )
			{
				if ( z > zlo )
					PushSample( out, dvec3( x, y, z ), p.source );
			}
			if ( zhi > zlo )
				PushSample( out, dvec3( x, y, zhi ), p.source );
		}
	}
}

//-----------------------------------------------------------------------------
// Per-thread accumulation
//-----------------------------------------------------------------------------
struct EyeThread
{
	std::vector<unsigned char> clusters;
	std::vector<EyeSample> samples;
	std::vector<EyePiece> keep;
	std::vector<int> cand;
	std::vector<const GrownPolytope *> obs;
	std::vector<EyePiece> pieces, tmp;
	int pieces_[EYESRC_COUNT];
	int mapped;
	bool solidEye;
};

static EyeThread s_EyeThreads[MAX_TOOL_THREADS + 1];
static const CollisionWorld *s_pEyeWorld;
static const PlayerProfile *s_pEyeProfile;
static const WalkGraph *s_pEyeGraph;
static const EyeOptions *s_pEyeOpts;
static double s_EyeSpacing;					// the sample spacing in use (opts.spacing, or wider)
static EyeVolume *s_pEyeVol;
static ObstacleSet s_EyeCrouchObs, s_EyeStandObs;
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
static bool IsViewpoint( int source )
{
	return source == EYESRC_CUBEMAP || source == EYESRC_CAMERA || source == EYESRC_VIEWCONTROL || source == EYESRC_CLI ||
		   source == EYESRC_SKYCAMERA;
}

static void EmitPiece( EyeThread &t, const EyePiece &p )
{
	const EyeVolume &ev = *s_pEyeVol;
	int flags = PieceClusters( p, t.clusters );
	if ( IsViewpoint( p.source ) && flags == 2 )
		t.solidEye = true;
	t.pieces_[p.source]++;
	SamplePiece( p, s_EyeSpacing, t.samples );
	if ( s_pEyeOpts->keepPieces )
		t.keep.push_back( p );
	for ( size_t k = 0; k < ev.skyOrigins.size(); k++ )
	{
		EyePiece m;
		SkyMapPiece( p, ev.skyOrigins[k], ev.skyScales[k], m );
		PieceClusters( m, t.clusters );
		t.mapped++;
		// a mapped piece is 1/scale the size: sample it 1/scale as finely
		t_SampleMapped = 1;
		SamplePiece( m, s_EyeSpacing / ev.skyScales[k], t.samples );
		t_SampleMapped = 0;
		if ( s_pEyeOpts->keepPieces )
			t.keep.push_back( m );
	}
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
	FeetColumn( node.poly, node.plane, rise, t.obs, WALK_SHRINK, t.pieces );
	if ( t.pieces.empty() )
	{
		// a sliver whose prism did not build: its polygon raised as segments
		for ( size_t k = 0; k < node.poly.size(); k++ )
		{
			dvec3 a = node.poly[k] + dvec3( 0, 0, prof.eyeCrouch );
			EmitPiece( t, SegmentPiece( a, a + dvec3( 0, 0, rise ), EYESRC_NODE ) );
		}
		return;
	}
	dvec3 eye( 0, 0, prof.eyeCrouch );
	for ( size_t p = 0; p < t.pieces.size(); p++ )
	{
		EyePiece e = Translated( t.pieces[p], eye );
		e.source = EYESRC_NODE;
		EmitPiece( t, e );
	}
	// a dead player's view, EYE_DEAD_HEIGHT above the floor: the prism up to the
	// crouch eye lies inside the crouch box on the floor, so it is free
	double dead = MIN( EYE_DEAD_HEIGHT, prof.eyeCrouch );
	if ( dead < prof.eyeCrouch )
	{
		std::vector<const GrownPolytope *> none;
		t.tmp.clear();
		FeetColumn( node.poly, node.plane, prof.eyeCrouch - dead, none, WALK_SHRINK, t.tmp );
		for ( size_t p = 0; p < t.tmp.size(); p++ )
		{
			EyePiece e = Translated( t.tmp[p], dvec3( 0, 0, dead ) );
			e.source = EYESRC_NODE;
			EmitPiece( t, e );
		}
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
			for ( size_t p = 0; p < t.pieces.size(); p++ )
			{
				EyePiece e = Translated( t.pieces[p], eye );
				std::vector<dvec3> pts = e.verts;
				for ( size_t k = 0; k < e.verts.size(); k++ )
					pts.push_back( PoseToPose( xp, xq, e.verts[k] ) );
				EyePiece sw;
				if ( EyePolyFromPoints( pts, sw ) )
				{
					sw.source = EYESRC_RIDE;
					EmitPiece( t, sw );
				}
			}
		}
	}
}

// Water and ladder-brush volumes: the hull of the volume raised by eye_crouch
// and by eye_stand (a swimmer or climber may duck). Obstacles inside are not
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
	else if ( v.kind == SPECIAL_LADDER )
	{
		// the climber stands on the segment; it may duck
		EmitPiece( t, SegmentPiece( v.a + dvec3( 0, 0, prof.eyeStand ), v.b + dvec3( 0, 0, prof.eyeStand ), EYESRC_LADDER ) );
		EmitPiece( t, SegmentPiece( v.a + dvec3( 0, 0, prof.eyeCrouch ), v.b + dvec3( 0, 0, prof.eyeCrouch ), EYESRC_LADDER ) );
	}
}

// Traced free arcs: one segment per chord, at the crouch eye (every arc is
// traced crouched).
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
	int n = MAX( 1, (int)a.nSeg[0] + (int)a.nSeg[1] );
	dvec3 eye( 0, 0, s_pEyeProfile->eyeCrouch );
	dvec3 prev( a.ox, a.oy, ArcZ( a, g, 0.0f ) );
	for ( int k = 1; k <= n; k++ )
	{
		float tt = a.tEnd * (float)k / (float)n;
		dvec3 p( a.ox + a.vx * tt, a.oy + a.vy * tt, ArcZ( a, g, tt ) );
		EmitPiece( t, SegmentPiece( prev + eye, p + eye, EYESRC_ARC ) );
		prev = p;
	}
}

static std::vector<EyePiece> s_EyeLoose;	// points and segments built on the main thread

static void LooseEyesOne( int iThread, int item )
{
	EmitPiece( s_EyeThreads[iThread], s_EyeLoose[item] );
}

static void RunEyePass( int count, ThreadWorkerFn fn )
{
	RunThreadsBatched( count, fn );
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
	// counts (some maps switch cameras by area or at run time).
	for ( int e = 0; e < num_entities; e++ )
	{
		if ( Q_stricmp( EntValue( &entities[e], "classname" ), "sky_camera" ) )
			continue;
		const char *pszScale = EntValue( &entities[e], "scale" );
		double scale = pszScale[0] ? atof( pszScale ) : 16.0;
		if ( !( scale > 0 ) )
			scale = 16.0;
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
	s_EyeStandObs.Build( world, world.stand );
	for ( int i = 0; i <= MAX_TOOL_THREADS; i++ )
	{
		EyeThread &t = s_EyeThreads[i];
		t.clusters.assign( nClusters, 0 );
		t.samples.clear();
		t.keep.clear();
		memset( t.pieces_, 0, sizeof( t.pieces_ ) );
		t.mapped = 0;
		t.solidEye = false;
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
	// higher), raised under every reached drop edge to the drop's start
	int n = (int)graph.nodes.size();
	double baseRise = MAX( profile.maxJumpRise,
						   profile.jumpApex * ( 1 + profile.airJumps ) + profile.eyeStand - profile.eyeCrouch );
	s_EyeRise.assign( n, baseRise );
	for ( size_t i = 0; i < graph.edges.size(); i++ )
	{
		const WalkEdge &e = graph.edges[i];
		if ( e.kind != EDGE_DROP || !graph.reached[e.a] || !graph.reached[e.b] )
			continue;
		double r = graph.nodes[e.a].maxs.z + baseRise - graph.nodes[e.b].mins.z;
		s_EyeRise[e.b] = MAX( s_EyeRise[e.b], r );
	}
	s_EyeNodes.clear();
	for ( int i = 0; i < n; i++ )
	{
		if ( graph.reached[i] )
			s_EyeNodes.push_back( i );
	}

	// sample spacing: the requested one, widened until the node columns'
	// estimated sample count (area / s^2 grid lines, each with rise / s + 2
	// points; twice that with a skybox copy) fits the cap
	s_EyeSpacing = opts.spacing;
	double area = 0, lines = 0;
	for ( size_t k = 0; k < s_EyeNodes.size(); k++ )
	{
		area += graph.nodes[s_EyeNodes[k]].area;
		lines += graph.nodes[s_EyeNodes[k]].area * s_EyeRise[s_EyeNodes[k]];
	}
	for ( ;; )
	{
		double s = s_EyeSpacing;
		double est = ( lines / ( s * s * s ) + 2.0 * area / ( s * s ) ) * ( 1.0 + eyes.skyOrigins.size() );
		if ( est <= opts.maxSamples || s > 4096 )
			break;
		s_EyeSpacing *= 1.25;
	}
	if ( s_EyeSpacing > opts.spacing )
		Msg( "eyes: sample spacing widened from %gu to %.2fu to keep about %.0f samples or fewer (-eyemaxsamples)\n",
			 opts.spacing, s_EyeSpacing, opts.maxSamples );
	eyes.spacing = s_EyeSpacing;
	RunEyePass( (int)s_EyeNodes.size(), NodeEyesOne );

	// water and ladders
	s_EyeSpecial.clear();
	const SpecialGraph &sg = graph.special;
	for ( size_t v = 0; v < sg.verts.size(); v++ )
	{
		int k = sg.verts[v].kind;
		if ( v < sg.reached.size() && sg.reached[v] &&
			 ( k == SPECIAL_WATER || k == SPECIAL_LADDER || k == SPECIAL_LADDER_BRUSH ) )
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
			s_EyeLoose.push_back( SegmentPiece( p + dvec3( 0, 0, profile.eyeCrouch ), p + dvec3( 0, 0, profile.eyeStand ),
												EYESRC_TELEPORT ) );
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
			s_EyeLoose.push_back( PointPiece( EntOrigin( &entities[e] ), EYESRC_VIEWCONTROL ) );
	}
	for ( size_t i = 0; i < opts.viewpoints.size(); i++ )
		s_EyeLoose.push_back( PointPiece( opts.viewpoints[i], EYESRC_CLI ) );
	for ( size_t k = 0; k < eyes.skyOrigins.size(); k++ )
		s_EyeLoose.push_back( PointPiece( eyes.skyOrigins[k], EYESRC_SKYCAMERA ) );
	for ( size_t i = 0; i < s_EyeLoose.size(); i++ )
	{
		int s = s_EyeLoose[i].source;
		eyes.viewpoints += IsViewpoint( s );
	}
	RunEyePass( (int)s_EyeLoose.size(), LooseEyesOne );

	// gather
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
		std::vector<EyeSample>().swap( t.samples );
		std::vector<EyePiece>().swap( t.keep );
		std::vector<EyePiece>().swap( t.pieces );
		std::vector<EyePiece>().swap( t.tmp );
	}
	if ( eyes.solidEye )
	{
		Warning( "bspwalk: a viewpoint (env_cubemap, point_camera, point_viewcontrol, sky_camera or -viewpoint) lies in a "
				 "solid leaf; the engine draws every cluster from there, so every cluster is an eye cluster (tier A proves "
				 "nothing)\n" );
		std::fill( eyes.clusters.begin(), eyes.clusters.end(), 1 );
	}
	for ( int c = 0; c < nClusters; c++ )
		eyes.eyeClusters += eyes.clusters[c];

	// dedupe the samples (adjacent pieces share grid points), then their clusters
	std::sort( eyes.samples.begin(), eyes.samples.end(), []( const EyeSample &a, const EyeSample &b ) {
		if ( a.x != b.x )
			return a.x < b.x;
		if ( a.y != b.y )
			return a.y < b.y;
		return a.z < b.z;
	} );
	eyes.samples.erase( std::unique( eyes.samples.begin(), eyes.samples.end(), []( const EyeSample &a, const EyeSample &b ) {
		return a.x == b.x && a.y == b.y && a.z == b.z;
	} ), eyes.samples.end() );
	for ( size_t i = 0; i < eyes.samples.size(); i++ )
	{
		const EyeSample &s = eyes.samples[i];
		int leaf = PointLeaf( dvec3( s.x, s.y, s.z ) );
		eyes.samples[i].cluster = dleafs[leaf].cluster;
	}

	s_EyeCrouchObs = ObstacleSet();
	s_EyeStandObs = ObstacleSet();
	std::vector<int>().swap( s_EyeNodes );
	std::vector<double>().swap( s_EyeRise );
	std::vector<EyePiece>().swap( s_EyeLoose );
	eyes.tBuild = Plat_FloatTime() - t0;

	Msg( "eyes:" );
	for ( int s = 0; s < EYESRC_COUNT; s++ )
		Msg( " %s %d", EyeSourceName( s ), eyes.pieces[s] );
	Msg( " pieces%s; %d of %d clusters; %d viewpoints; %d samples at %gu; %.2f s\n",
		 eyes.sky ? " (and each mapped into the 3D skybox)" : "", eyes.eyeClusters, nClusters, eyes.viewpoints,
		 (int)eyes.samples.size(), s_EyeSpacing, eyes.tBuild );
	for ( size_t k = 0; k < eyes.skyOrigins.size(); k++ )
		Msg( "eyes: sky_camera at (%.1f %.1f %.1f), scale %g\n", eyes.skyOrigins[k].x, eyes.skyOrigins[k].y,
			 eyes.skyOrigins[k].z, eyes.skyScales[k] );
}
