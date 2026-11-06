//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: Stage 2 (exact walkable polygons) and the CPU part of Stage 3 (walk
//          / jump-up / drop edges, seeds, BFS flood). See bw_walk.h.
//
//          All geometry is double precision and works on half-spaces and face
//          polygons only. Plane triples are never intersected: the grown
//          planes include near-coplanar pairs whose intersection lines run
//          far from any real vertex (docs/bsp_walker_rebuild.md, Stage 1).
//
//=============================================================================//

#include <algorithm>
#include <set>
#include <unordered_map>
#include "cmdlib.h"
#include "bsplib.h"
#include "threads.h"
#include "tier0/threadtools.h"
#include "mathlib/mathlib.h"
#include "bw_walk.h"
#include "bw_util.h"
#include "bw_hash.h"

// On-plane band of the polygon splits. Coordinates reach ~2^15, where a
// double ulp is ~4e-12, so this is far above rounding and far below any real
// feature (the slivers kept are >= 1/16u wide once the shrink is added).
static const double PLANE_EPS = 1e-7;
// A piece is degenerate when its area is at most this times its extent
// squared (a 64u-long piece narrower than ~6e-8u).
static const double DEGEN_REL = 1e-9;
// A vertical segment is blocked only where it reaches at least this far into
// a (shrunk) obstacle; touching one counts as free.
static const double FREE_TOL = 1e-6;
// The boundary-inside-projection test of the edges reaches this far past the
// other polygon (over-reach direction).
static const double EDGE_REACH = 1e-4;
// Below this rise a vertical segment is not tested: both ends lie on free
// polygons, so the segment is within FREE_TOL plus the 1e-7 polygon cut slop
// of free space (granting the edge is the over-reach direction).
static const double DZ_TRIVIAL = 1e-6;
// Fixed vertex buffers of the clip and split loops (polygon + planes must fit;
// larger inputs take the allocating path).
enum { SPLIT_MAXV = 256 };

// Per-thread scalars on their own cache lines: adjacent ints written by
// different threads on every query cost more than the queries.
struct alignas( 64 ) PaddedInt
{
	int v;
};

const char *WalkEdgeKindName( int kind )
{
	static const char *s_Names[EDGE_KIND_COUNT] = { "walk", "jump", "drop", "arc", "slide" };
	return ( kind >= 0 && kind < EDGE_KIND_COUNT ) ? s_Names[kind] : "unknown";
}

//-----------------------------------------------------------------------------
// Polygon helpers
//-----------------------------------------------------------------------------
double WalkPolyArea( const DWinding &w )
{
	if ( w.size() < 3 )
		return 0.0;
	dvec3 mins( 1e300, 1e300, 1e300 ), maxs( -1e300, -1e300, -1e300 );
	for ( size_t i = 0; i < w.size(); i++ )
	{
		mins = DMin( mins, w[i] );
		maxs = DMax( maxs, w[i] );
	}
	dvec3 ext = maxs - mins;
	double area = DWindingArea( w );
	if ( area <= 1e-12 || area <= DEGEN_REL * DDot( ext, ext ) )
		return 0.0;
	return area;
}

// One pass of polylib's ClipWindingEpsilon, in double: the parts of w on the
// front (n . x >= d) and back. Points within eps go to both.
static void SplitWinding( const DWinding &w, const dvec3 &normal, double dist, double eps, DWinding &front, DWinding &back )
{
	front.clear();
	back.clear();
	int n = (int)w.size();
	if ( n == 0 )
		return;

	double dists[64];
	int sides[64];
	std::vector<double> distsHeap;
	std::vector<int> sidesHeap;
	double *pd = dists;
	int *ps = sides;
	if ( n + 1 > 64 )
	{
		distsHeap.resize( n + 1 );
		sidesHeap.resize( n + 1 );
		pd = distsHeap.data();
		ps = sidesHeap.data();
	}

	int counts[3] = { 0, 0, 0 };
	for ( int i = 0; i < n; i++ )
	{
		double dot = DDot( w[i], normal ) - dist;
		pd[i] = dot;
		ps[i] = ( dot > eps ) ? SIDE_FRONT : ( ( dot < -eps ) ? SIDE_BACK : SIDE_ON );
		counts[ps[i]]++;
	}
	pd[n] = pd[0];
	ps[n] = ps[0];

	if ( !counts[SIDE_FRONT] )
	{
		back = w;
		return;
	}
	if ( !counts[SIDE_BACK] )
	{
		front = w;
		return;
	}

	front.reserve( n + 2 );
	back.reserve( n + 2 );
	for ( int i = 0; i < n; i++ )
	{
		const dvec3 &p1 = w[i];
		if ( ps[i] == SIDE_ON )
		{
			front.push_back( p1 );
			back.push_back( p1 );
			continue;
		}
		if ( ps[i] == SIDE_FRONT )
			front.push_back( p1 );
		else
			back.push_back( p1 );

		if ( ps[i + 1] == SIDE_ON || ps[i + 1] == ps[i] )
			continue;

		const dvec3 &p2 = w[( i + 1 ) % n];
		double t = pd[i] / ( pd[i] - pd[i + 1] );
		dvec3 mid;
		for ( int j = 0; j < 3; j++ )
		{
			// axial planes split exactly (polylib's round-off guard)
			if ( normal[j] == 1.0 )
				mid[j] = dist;
			else if ( normal[j] == -1.0 )
				mid[j] = -dist;
			else
				mid[j] = p1[j] + t * ( p2[j] - p1[j] );
		}
		front.push_back( mid );
		back.push_back( mid );
	}
}

static void PolyBounds( const DWinding &w, dvec3 &mins, dvec3 &maxs )
{
	mins = dvec3( 1e300, 1e300, 1e300 );
	maxs = dvec3( -1e300, -1e300, -1e300 );
	for ( size_t i = 0; i < w.size(); i++ )
	{
		mins = DMin( mins, w[i] );
		maxs = DMax( maxs, w[i] );
	}
}

static bool BoxesOverlap( const dvec3 &amin, const dvec3 &amax, const dvec3 &bmin, const dvec3 &bmax )
{
	return amin.x <= bmax.x && bmin.x <= amax.x && amin.y <= bmax.y && bmin.y <= amax.y && amin.z <= bmax.z &&
		   bmin.z <= amax.z;
}

// WalkPolyArea of a raw vertex array.
static double PolyAreaRaw( const dvec3 *w, int n )
{
	if ( n < 3 )
		return 0.0;
	dvec3 mins = w[0], maxs = w[0];
	double total = 0;
	for ( int i = 1; i < n; i++ )
	{
		mins = DMin( mins, w[i] );
		maxs = DMax( maxs, w[i] );
		if ( i >= 2 )
			total += 0.5 * DLength( DCross( w[i - 1] - w[0], w[i] - w[0] ) );
	}
	dvec3 ext = maxs - mins;
	if ( total <= 1e-12 || total <= DEGEN_REL * DDot( ext, ext ) )
		return 0.0;
	return total;
}

// Does the polygon reach into the interior of { n_k . x <= d_k - shrink }
// with a non-degenerate area? The clip keeps a point only if it is inside
// every plane by more than PLANE_EPS or within it (DChopWindingInPlace's
// rule), on fixed buffers: this runs for every candidate of every piece.
static bool PolyMeetsPolytope( const DWinding &poly, const DPlane *planes, int numPlanes, double shrink )
{
	int n = (int)poly.size();
	// Cheap rejection first: one plane with the whole polygon on or in front of it.
	for ( int k = 0; k < numPlanes; k++ )
	{
		double d = planes[k].d - shrink;
		bool allOut = true;
		for ( int i = 0; i < n && allOut; i++ )
			allOut = DDot( planes[k].n, poly[i] ) - d >= -PLANE_EPS;
		if ( allOut )
			return false;
	}

	enum { MAXV = SPLIT_MAXV };
	if ( n + numPlanes >= MAXV )
	{
		DWinding inside = poly;
		for ( int k = 0; k < numPlanes && inside.size() >= 3; k++ )
			DChopWindingInPlace( inside, -planes[k].n, -( planes[k].d - shrink ), PLANE_EPS );
		return WalkPolyArea( inside ) > 0.0;
	}
	dvec3 bufA[MAXV], bufB[MAXV];
	double dist[MAXV];
	dvec3 *src = bufA, *dst = bufB;
	for ( int i = 0; i < n; i++ )
		src[i] = poly[i];
	for ( int k = 0; k < numPlanes && n >= 3; k++ )
	{
		// keep the back side: dist = d - n.x > 0 is inside
		const dvec3 &pn = planes[k].n;
		double d = planes[k].d - shrink;
		bool anyIn = false, anyOut = false;
		for ( int i = 0; i < n; i++ )
		{
			dist[i] = d - DDot( pn, src[i] );
			anyIn = anyIn || dist[i] > PLANE_EPS;
			anyOut = anyOut || dist[i] < -PLANE_EPS;
		}
		if ( !anyIn )
			return false;
		if ( !anyOut )
			continue;
		int m = 0;
		for ( int i = 0; i < n; i++ )
		{
			int j = ( i + 1 ) % n;
			if ( dist[i] >= -PLANE_EPS )
				dst[m++] = src[i];
			if ( ( dist[i] > PLANE_EPS && dist[j] < -PLANE_EPS ) || ( dist[i] < -PLANE_EPS && dist[j] > PLANE_EPS ) )
			{
				double t = dist[i] / ( dist[i] - dist[j] );
				dst[m++] = src[i] + ( src[j] - src[i] ) * t;
			}
		}
		std::swap( src, dst );
		n = m;
	}
	return PolyAreaRaw( src, n ) > 0.0;
}

// SplitWinding on fixed buffers (same rule, same axial snap). Sizes are
// counts; each output can hold n + 1 vertices.
static void SplitRaw( const dvec3 *w, int n, const dvec3 &normal, double dist, double eps, dvec3 *front, int &nf,
					  dvec3 *back, int &nb )
{
	enum { MAXD = SPLIT_MAXV + 1 };
	double pd[MAXD];
	int ps[MAXD];
	int counts[3] = { 0, 0, 0 };
	for ( int i = 0; i < n; i++ )
	{
		double dot = DDot( w[i], normal ) - dist;
		pd[i] = dot;
		ps[i] = ( dot > eps ) ? SIDE_FRONT : ( ( dot < -eps ) ? SIDE_BACK : SIDE_ON );
		counts[ps[i]]++;
	}
	pd[n] = pd[0];
	ps[n] = ps[0];
	nf = nb = 0;
	if ( !counts[SIDE_FRONT] )
	{
		for ( int i = 0; i < n; i++ )
			back[nb++] = w[i];
		return;
	}
	if ( !counts[SIDE_BACK] )
	{
		for ( int i = 0; i < n; i++ )
			front[nf++] = w[i];
		return;
	}
	for ( int i = 0; i < n; i++ )
	{
		const dvec3 &p1 = w[i];
		if ( ps[i] == SIDE_ON )
		{
			front[nf++] = p1;
			back[nb++] = p1;
			continue;
		}
		if ( ps[i] == SIDE_FRONT )
			front[nf++] = p1;
		else
			back[nb++] = p1;
		if ( ps[i + 1] == SIDE_ON || ps[i + 1] == ps[i] )
			continue;
		const dvec3 &p2 = w[( i + 1 ) % n];
		double t = pd[i] / ( pd[i] - pd[i + 1] );
		dvec3 mid;
		for ( int j = 0; j < 3; j++ )
		{
			if ( normal[j] == 1.0 )
				mid[j] = dist;
			else if ( normal[j] == -1.0 )
				mid[j] = -dist;
			else
				mid[j] = p1[j] + t * ( p2[j] - p1[j] );
		}
		front[nf++] = mid;
		back[nb++] = mid;
	}
}

// The pieces of poly in front of the polytope's planes (the peel of
// SubtractPolytope), appended to out. Only the output pieces allocate: the
// tier0 allocator serializes the 32 threads, which made Stage 2 five times
// slower on 32 threads than on 4.
static void CutPolytope( const DWinding &poly, const DPlane *planes, int numPlanes, double shrink,
						 std::vector<DWinding> &out )
{
	int n = (int)poly.size();
	if ( n + numPlanes >= SPLIT_MAXV )
	{
		DWinding rest = poly, front, back;
		for ( int k = 0; k < numPlanes && rest.size() >= 3; k++ )
		{
			SplitWinding( rest, planes[k].n, planes[k].d - shrink, PLANE_EPS, front, back );
			if ( WalkPolyArea( front ) > 0.0 )
				out.push_back( front );
			rest.swap( back );
		}
		return;
	}
	dvec3 bufA[SPLIT_MAXV], bufB[SPLIT_MAXV], front[SPLIT_MAXV];
	dvec3 *rest = bufA, *back = bufB;
	for ( int i = 0; i < n; i++ )
		rest[i] = poly[i];
	for ( int k = 0; k < numPlanes && n >= 3; k++ )
	{
		int nf, nb;
		SplitRaw( rest, n, planes[k].n, planes[k].d - shrink, PLANE_EPS, front, nf, back, nb );
		if ( PolyAreaRaw( front, nf ) > 0.0 )
			out.push_back( DWinding( front, front + nf ) );
		std::swap( rest, back );
		n = nb;
	}
}

bool SubtractPolytope( const DWinding &poly, const DPlane *planes, int numPlanes, double shrink,
					   std::vector<DWinding> &out )
{
	// Does the polygon reach into the polytope's interior at all? Splitting
	// first would fragment polygons the polytope only touches.
	if ( !PolyMeetsPolytope( poly, planes, numPlanes, shrink ) )
	{
		out.push_back( poly );
		return false;
	}
	// Peel off the part in front of each plane; what is left after the last
	// plane is inside the polytope and is dropped.
	CutPolytope( poly, planes, numPlanes, shrink, out );
	return true;
}

// Splits at the multiples of WALK_CHOP in X, then Y.
static void ChopToGrid( const DWinding &w, std::vector<DWinding> &out )
{
	dvec3 mins, maxs;
	PolyBounds( w, mins, maxs );
	if ( (int)w.size() + 4 >= SPLIT_MAXV )
	{
		// no fixed buffers for a polygon this large (never seen in practice)
		std::vector<DWinding> strips, next;
		strips.push_back( w );
		DWinding front, back;
		for ( int axis = 0; axis < 2; axis++ )
		{
			next.clear();
			dvec3 n( 0, 0, 0 );
			n[axis] = 1.0;
			for ( size_t s = 0; s < strips.size(); s++ )
			{
				dvec3 smin, smax;
				PolyBounds( strips[s], smin, smax );
				DWinding piece = strips[s];
				for ( int k = (int)floor( smin[axis] / WALK_CHOP ) + 1; k <= (int)ceil( smax[axis] / WALK_CHOP ) - 1 && piece.size() >= 3; k++ )
				{
					SplitWinding( piece, n, k * WALK_CHOP, PLANE_EPS, front, back );
					if ( WalkPolyArea( back ) > 0.0 )
						next.push_back( back );
					piece.swap( front );
				}
				if ( WalkPolyArea( piece ) > 0.0 )
					next.push_back( piece );
			}
			strips.swap( next );
		}
		out.insert( out.end(), strips.begin(), strips.end() );
		return;
	}

	// X strips, each cut into Y cells, all on fixed buffers; a convex
	// polygon gains at most one vertex per cut, two per strip.
	const dvec3 nx( 1, 0, 0 ), ny( 0, 1, 0 );
	dvec3 xa[SPLIT_MAXV], xb[SPLIT_MAXV], strip[SPLIT_MAXV], ya[SPLIT_MAXV], yb[SPLIT_MAXV], cell[SPLIT_MAXV];
	dvec3 *xrest = xa, *xfront = xb;
	int nxr = (int)w.size();
	for ( int i = 0; i < nxr; i++ )
		xrest[i] = w[i];
	int kx0 = (int)floor( mins.x / WALK_CHOP ) + 1, kx1 = (int)ceil( maxs.x / WALK_CHOP ) - 1;
	for ( int kx = kx0; kx <= kx1 + 1 && nxr >= 3; kx++ )
	{
		int ns;
		if ( kx <= kx1 )
		{
			int nf;
			SplitRaw( xrest, nxr, nx, kx * WALK_CHOP, PLANE_EPS, xfront, nf, strip, ns );
			std::swap( xrest, xfront );
			nxr = nf;
		}
		else
		{
			// the last strip is what is left
			ns = nxr;
			for ( int i = 0; i < ns; i++ )
				strip[i] = xrest[i];
			nxr = 0;
		}
		if ( PolyAreaRaw( strip, ns ) <= 0.0 )
			continue;

		double ylo = 1e300, yhi = -1e300;
		for ( int i = 0; i < ns; i++ )
		{
			ylo = MIN( ylo, strip[i].y );
			yhi = MAX( yhi, strip[i].y );
		}
		dvec3 *yrest = ya, *yfront = yb;
		int nyr = ns;
		for ( int i = 0; i < ns; i++ )
			yrest[i] = strip[i];
		int ky0 = (int)floor( ylo / WALK_CHOP ) + 1, ky1 = (int)ceil( yhi / WALK_CHOP ) - 1;
		for ( int ky = ky0; ky <= ky1 + 1 && nyr >= 3; ky++ )
		{
			int nc;
			if ( ky <= ky1 )
			{
				int nf;
				SplitRaw( yrest, nyr, ny, ky * WALK_CHOP, PLANE_EPS, yfront, nf, cell, nc );
				std::swap( yrest, yfront );
				nyr = nf;
			}
			else
			{
				nc = nyr;
				for ( int i = 0; i < nc; i++ )
					cell[i] = yrest[i];
				nyr = 0;
			}
			if ( PolyAreaRaw( cell, nc ) > 0.0 )
				out.push_back( DWinding( cell, cell + nc ) );
		}
	}
}

// Removes consecutive duplicate vertices, in place.
static void CleanWinding( DWinding &w )
{
	size_t m = 0;
	for ( size_t i = 0; i < w.size(); i++ )
	{
		if ( m )
		{
			dvec3 d = w[i] - w[m - 1];
			if ( DDot( d, d ) < 1e-20 )
				continue;
		}
		w[m++] = w[i];
	}
	while ( m > 1 )
	{
		dvec3 d = w[0] - w[m - 1];
		if ( DDot( d, d ) >= 1e-20 )
			break;
		m--;
	}
	w.resize( m );
}

//-----------------------------------------------------------------------------
// RunThreadsOnIndividual in blocks. It takes one critical section per work
// item, and with ~10^6 items of a few microseconds each (fightspace3) the 32
// threads spent most of their time queued on that lock (Stage 2+3 took 22 s).
//-----------------------------------------------------------------------------
static ThreadWorkerFn s_pBatchFn;
static int s_nBatchItems, s_nBatchSize;

static void BatchWorker( int iThread, int block )
{
	int end = MIN( s_nBatchItems, ( block + 1 ) * s_nBatchSize );
	for ( int i = block * s_nBatchSize; i < end; i++ )
		s_pBatchFn( iThread, i );
}

void RunThreadsBatched( int count, ThreadWorkerFn fn )
{
	if ( count <= 0 )
		return;
	// ~64 blocks per thread keeps the load balanced
	int threads = MAX( numthreads, 1 );
	s_nBatchSize = MAX( 1, MIN( 1024, count / ( threads * 64 ) ) );
	s_nBatchItems = count;
	s_pBatchFn = fn;
	RunThreadsOnIndividual( ( count + s_nBatchSize - 1 ) / s_nBatchSize, false, BatchWorker );
	s_pBatchFn = NULL;
}

// The prism over face f of polytope q: the face's in-plane edge planes plus a
// slab of +-1u about its plane. Subtracting it removes exactly the face.
static void FacePrism( const GrownPolytope &q, int f, std::vector<DPlane> &out )
{
	out.clear();
	const std::vector<int> &face = q.faces[f];
	const dvec3 &n = q.planes[f].n;
	int nv = (int)face.size();
	for ( int k = 0; k < nv; k++ )
	{
		const dvec3 &a = q.verts[face[k]], &b = q.verts[face[( k + 1 ) % nv]];
		// faces are CCW seen from outside (along n), so edge x n points out of the face
		dvec3 m = DCross( b - a, n );
		if ( DNormalize( m ) < 1e-9 )
			continue;
		DPlane p;
		p.n = m;
		p.d = DDot( m, a );
		out.push_back( p );
	}
	DPlane top, bottom;
	top.n = n;
	top.d = q.planes[f].d + 1.0;
	bottom.n = -n;
	bottom.d = -( q.planes[f].d - 1.0 );
	out.push_back( top );
	out.push_back( bottom );
}

// Is 'p' the plane of an up face of q? Returns the face, or -1.
static int CoplanarUpFace( const GrownPolytope &q, const DPlane &p )
{
	for ( size_t f = 0; f < q.planes.size(); f++ )
	{
		const DPlane &o = q.planes[f];
		if ( o.n.z >= SLIDE_MIN_NZ && DDot( o.n, p.n ) > 1.0 - 1e-12 && fabs( o.d - p.d ) < 1e-6 )
			return (int)f;
	}
	return -1;
}

// Subtracts every obstacle of the set except 'skip' from each polygon of
// 'pieces' (in place). With pOwnPlane (the up face of obstacle 'skip' the
// pieces come from), also removes the parts covered by a coplanar up face of
// a lower-numbered obstacle: adjacent floors overlap by up to 32u, since each
// is grown 16u past its brush, and neither clips the other (its top plane is
// shrunk below them). The overlap is the same free space twice; the lower
// index keeps it. Exact: the region is clipped by the same obstacles either way.
static void SubtractObstacles( int iThread, const ObstacleSet &obs, int skip, const DPlane *pOwnPlane,
							   std::vector<DWinding> &pieces, std::vector<int> &cand )
{
	std::vector<DPlane> prism;
	dvec3 mins( 1e300, 1e300, 1e300 ), maxs( -1e300, -1e300, -1e300 );
	for ( size_t i = 0; i < pieces.size(); i++ )
	{
		dvec3 a, b;
		PolyBounds( pieces[i], a, b );
		mins = DMin( mins, a );
		maxs = DMax( maxs, b );
	}
	cand.clear();
	obs.hash.Query( mins.x, mins.y, maxs.x, maxs.y, cand );	// ascending ids: deterministic order

	std::vector<DWinding> added;
	for ( size_t c = 0; c < cand.size() && !pieces.empty(); c++ )
	{
		int o = cand[c];
		if ( o == skip )
			continue;
		const GrownPolytope &q = ( *obs.polys )[o];
		if ( !BoxesOverlap( mins, maxs, q.mins, q.maxs ) )
			continue;

		// Cut pieces are replaced by their outside parts; uncut ones stay put.
		added.clear();
		bool anyCut = false;
		for ( size_t i = 0; i < pieces.size(); i++ )
		{
			dvec3 a, b;
			PolyBounds( pieces[i], a, b );
			if ( !BoxesOverlap( a, b, q.mins, q.maxs ) ||
				 !PolyMeetsPolytope( pieces[i], q.planes.data(), (int)q.planes.size(), WALK_SHRINK ) )
				continue;
			CutPolytope( pieces[i], q.planes.data(), (int)q.planes.size(), WALK_SHRINK, added );
			pieces[i].clear();
			anyCut = true;
		}

		int f = ( pOwnPlane && o < skip ) ? CoplanarUpFace( q, *pOwnPlane ) : -1;
		if ( f >= 0 )
		{
			FacePrism( q, f, prism );
			for ( int pass = 0; pass < 2; pass++ )
			{
				std::vector<DWinding> &list = pass ? added : pieces;
				size_t count = list.size();
				for ( size_t i = 0; i < count; i++ )
				{
					if ( list[i].size() < 3 || !PolyMeetsPolytope( list[i], prism.data(), (int)prism.size(), 0.0 ) )
						continue;
					CutPolytope( list[i], prism.data(), (int)prism.size(), 0.0, added );
					list[i].clear();
					anyCut = true;
				}
			}
		}
		if ( !anyCut )
			continue;

		size_t m = 0;
		for ( size_t i = 0; i < pieces.size(); i++ )
		{
			if ( pieces[i].size() >= 3 )
			{
				if ( m != i )
					pieces[m].swap( pieces[i] );
				m++;
			}
		}
		pieces.resize( m );
		for ( size_t i = 0; i < added.size(); i++ )
		{
			if ( added[i].size() >= 3 )
			{
				pieces.push_back( DWinding() );
				pieces.back().swap( added[i] );
			}
		}
		mins = dvec3( 1e300, 1e300, 1e300 );
		maxs = dvec3( -1e300, -1e300, -1e300 );
		for ( size_t i = 0; i < pieces.size(); i++ )
		{
			dvec3 a, b;
			PolyBounds( pieces[i], a, b );
			mins = DMin( mins, a );
			maxs = DMax( maxs, b );
		}
	}
}

//-----------------------------------------------------------------------------
// Mover poses (tops_only). Each pose is a rigid transform of the entity's
// spawn-pose convexes. Values follow the SDK (doors.cpp, func_movelinear.cpp).
//-----------------------------------------------------------------------------
static dvec3 MoveDirForKey( entity_t *ent )
{
	// "movedir" holds angles (pitch yaw roll); the game turns them into a
	// vector with AngleVectors.
	double a[3] = { 0, 0, 0 };
	sscanf( EntValue( ent, "movedir" ), "%lf %lf %lf", &a[0], &a[1], &a[2] );
	const double d2r = M_PI / 180.0;
	double sp = sin( a[0] * d2r ), cp = cos( a[0] * d2r );
	double sy = sin( a[1] * d2r ), cy = cos( a[1] * d2r );
	return dvec3( cp * cy, cp * sy, -sp );
}

// The SDK's DotProductAbs( movedir, OBBSize - 2 ): the engine spreads a brush
// model's bounds by 1u per side on load, so OBBSize - 2 is the BSP model size.
static double MoveDistanceAlong( entity_t *ent, const dvec3 &dir )
{
	const char *model = EntValue( ent, "model" );
	int m = ( model[0] == '*' ) ? atoi( model + 1 ) : -1;
	if ( m <= 0 || m >= nummodels )
		return 0.0;
	dvec3 size( dmodels[m].maxs.x - dmodels[m].mins.x, dmodels[m].maxs.y - dmodels[m].mins.y,
				dmodels[m].maxs.z - dmodels[m].mins.z );
	return fabs( dir.x ) * size.x + fabs( dir.y ) * size.y + fabs( dir.z ) * size.z;
}

static DTransform TranslatePose( const dvec3 &t )
{
	DTransform xf;
	xf.origin = t;
	return xf;
}

// Rotation of the spawn-pose shape (already at 'angles') to 'angles2', about the entity origin.
static DTransform RotatePose( const dvec3 &org, const QAngle &angles, const QAngle &angles2 )
{
	DTransform r1, r2, xf;
	r1.SetOrgAngles( dvec3( 0, 0, 0 ), angles.x, angles.y, angles.z );
	r2.SetOrgAngles( dvec3( 0, 0, 0 ), angles2.x, angles2.y, angles2.z );
	// R = R2 * R1^T; p' = R (p - org) + org
	for ( int i = 0; i < 3; i++ )
		for ( int j = 0; j < 3; j++ )
		{
			double s = 0;
			for ( int k = 0; k < 3; k++ )
				s += r2.m[i][k] * r1.m[j][k];
			xf.m[i][j] = s;
		}
	dvec3 ro( xf.m[0][0] * org.x + xf.m[0][1] * org.y + xf.m[0][2] * org.z,
			  xf.m[1][0] * org.x + xf.m[1][1] * org.y + xf.m[1][2] * org.z,
			  xf.m[2][0] * org.x + xf.m[2][1] * org.y + xf.m[2][2] * org.z );
	xf.origin = org - ro;
	return xf;
}

// Train poses: the path nodes reachable from the train's "target" through
// "target" (and a path_track's "altpath") links, each a translation (rotation,
// orientationtype, is ignored). A path node whose translation is (0 0 0) is the
// spawn pose. links: consecutive path nodes, plus spawn -> the first one (the
// train is moved there at spawn).
static const int MAX_TRAIN_POSES = 256;

static void TrainPoses( entity_t *ent, bool track, std::vector<DTransform> &poses,
						std::vector< std::pair<int, int> > &links )
{
	const char *pszPathClass = track ? "path_track" : "path_corner";
	dvec3 org = EntOrigin( ent );
	double height = track ? atof( EntValue( ent, "height" ) ) : 0.0;
	dvec3 centerLocal( 0, 0, 0 );
	const char *model = EntValue( ent, "model" );
	int m = ( model[0] == '*' ) ? atoi( model + 1 ) : -1;
	if ( m > 0 && m < nummodels )
		centerLocal = dvec3( 0.5 * ( dmodels[m].mins.x + dmodels[m].maxs.x ), 0.5 * ( dmodels[m].mins.y + dmodels[m].maxs.y ),
							 0.5 * ( dmodels[m].mins.z + dmodels[m].maxs.z ) );

	// FindEntityByName( target ) is the first entity of that name; it must be a path node
	int first = FindEntityByTargetname( EntValue( ent, "target" ), -1 );
	if ( first < 0 || Q_stricmp( EntValue( &entities[first], "classname" ), pszPathClass ) )
		return;
	std::vector<int> paths( 1, first ), poseOf;
	std::unordered_map<int, int> index;
	index[first] = 0;
	std::vector< std::pair<int, int> > pathLinks;	// indices into paths
	for ( size_t k = 0; k < paths.size(); k++ )
	{
		entity_t *pe = &entities[paths[k]];
		dvec3 t = track ? TrackTrainTranslation( org, EntOrigin( pe ), height ) : FuncTrainTranslation( org, centerLocal, EntOrigin( pe ) );
		int pose = 0;
		if ( DLength( t ) > 1e-6 )
		{
			for ( size_t i = 1; i < poses.size() && !pose; i++ )
			{
				if ( DLength( poses[i].origin - t ) <= 1e-6 )
					pose = (int)i;
			}
			if ( !pose )
			{
				poses.push_back( TranslatePose( t ) );
				pose = (int)poses.size() - 1;
			}
		}
		poseOf.push_back( pose );
		const char *pszNext[2] = { EntValue( pe, "target" ), track ? EntValue( pe, "altpath" ) : "" };
		for ( int n = 0; n < 2; n++ )
		{
			int q = pszNext[n][0] ? FindEntityByTargetname( pszNext[n], -1 ) : -1;
			if ( q < 0 || Q_stricmp( EntValue( &entities[q], "classname" ), pszPathClass ) )
				continue;
			std::unordered_map<int, int>::iterator it = index.find( q );
			if ( it == index.end() )
			{
				if ( (int)paths.size() >= MAX_TRAIN_POSES )
					continue;
				int qi = (int)paths.size();
				index[q] = qi;
				paths.push_back( q );
				pathLinks.push_back( std::make_pair( (int)k, qi ) );
			}
			else
				pathLinks.push_back( std::make_pair( (int)k, it->second ) );
		}
	}
	links.push_back( std::make_pair( 0, poseOf[0] ) );
	for ( size_t i = 0; i < pathLinks.size(); i++ )
		links.push_back( std::make_pair( poseOf[pathLinks[i].first], poseOf[pathLinks[i].second] ) );
}

DTransform MoverSweepPose( const MoverEntity &me, int a, int b, double f )
{
	if ( !me.rotating || a < 0 || b < 0 || a >= (int)me.poseAngles.size() || b >= (int)me.poseAngles.size() )
		return DTransform();
	dvec3 d = me.poseAngles[a] + ( me.poseAngles[b] - me.poseAngles[a] ) * f;
	QAngle ang( (float)me.hingeAngles.x, (float)me.hingeAngles.y, (float)me.hingeAngles.z );
	QAngle ang2( (float)( me.hingeAngles.x + d.x ), (float)( me.hingeAngles.y + d.y ), (float)( me.hingeAngles.z + d.z ) );
	return RotatePose( me.hingeOrigin, ang, ang2 );
}

// Pose 0 is always the spawn pose (identity). links: the pose pairs a rider
// moves between (either way), deduplicated. 'hinge' gets the rotating fields
// of MoverEntity (rotating false for every other class).
static void EntityPoses( int entityIndex, std::vector<DTransform> &poses, std::vector< std::pair<int, int> > &links,
						 MoverEntity &hinge )
{
	poses.clear();
	links.clear();
	poses.push_back( DTransform() );
	hinge.rotating = false;
	hinge.poseAngles.assign( 1, dvec3( 0, 0, 0 ) );
	if ( entityIndex < 0 || entityIndex >= num_entities )
		return;
	entity_t *ent = &entities[entityIndex];
	const char *cls = EntValue( ent, "classname" );
	bool allPairs = false;

	if ( !Q_stricmp( cls, "func_door" ) )
	{
		// doors.cpp CBaseDoor::Spawn: position2 = position1 + movedir * (DotProductAbs(movedir, size) - lip)
		dvec3 dir = MoveDirForKey( ent );
		double travel = MoveDistanceAlong( ent, dir ) - atof( EntValue( ent, "lip" ) );
		if ( fabs( travel ) > 1e-6 )	// a zero-travel door has only its spawn pose
			poses.push_back( TranslatePose( dir * travel ) );
	}
	else if ( !Q_stricmp( cls, "func_door_rotating" ) )
	{
		// doors.cpp CRotDoor::Spawn: angle2 = angles + axis * distance (negated
		// by "Reverse Dir"); a yaw door that is not one-way opens away from the
		// player, so both signs are poses.
		int flags = atoi( EntValue( ent, "spawnflags" ) );
		const char *pszDist = EntValue( ent, "distance" );
		double dist = pszDist[0] ? atof( pszDist ) : 90.0;
		if ( flags & 2 )
			dist = -dist;
		QAngle axis( 0, 1, 0 );			// SF_DOOR_ROTATE_YAW
		if ( flags & 64 )
			axis = QAngle( 0, 0, 1 );		// SF_DOOR_ROTATE_ROLL
		else if ( flags & 128 )
			axis = QAngle( 1, 0, 0 );		// SF_DOOR_ROTATE_PITCH
		Vector org( 0, 0, 0 );
		QAngle ang( 0, 0, 0 );
		GetVectorForKey( ent, (char *)"origin", org );
		GetAnglesForKey( ent, (char *)"angles", ang );
		poses.push_back( RotatePose( dvec3( org ), ang, ang + axis * (float)dist ) );
		hinge.poseAngles.push_back( dvec3( axis.x, axis.y, axis.z ) * (double)(float)dist );
		if ( !( flags & 16 ) && axis.y != 0 )	// SF_DOOR_ONEWAY
		{
			poses.push_back( RotatePose( dvec3( org ), ang, ang - axis * (float)dist ) );
			hinge.poseAngles.push_back( dvec3( axis.x, axis.y, axis.z ) * -(double)(float)dist );
		}
		hinge.rotating = true;
		hinge.hingeOrigin = dvec3( org );
		hinge.hingeAngles = dvec3( ang.x, ang.y, ang.z );
	}
	else if ( !Q_stricmp( cls, "func_movelinear" ) )
	{
		// func_movelinear.cpp: position1 = origin - movedir * dist * startpos, position2 = position1 + movedir * dist
		dvec3 dir = MoveDirForKey( ent );
		double dist = atof( EntValue( ent, "MoveDistance" ) );
		if ( dist <= 0 )
			dist = MoveDistanceAlong( ent, dir ) - atof( EntValue( ent, "lip" ) );
		double start = atof( EntValue( ent, "StartPosition" ) );
		dvec3 p1 = dir * ( -dist * start );
		dvec3 p2 = p1 + dir * dist;
		if ( DLength( p1 ) > 1e-6 )
			poses.push_back( TranslatePose( p1 ) );
		if ( DLength( p2 ) > 1e-6 && DLength( p2 - p1 ) > 1e-6 )
			poses.push_back( TranslatePose( p2 ) );
		allPairs = true;	// spawn lies on the line between them
	}
	else if ( !Q_stricmp( cls, "func_tracktrain" ) || !Q_stricmp( cls, "func_tanktrain" ) )
		TrainPoses( ent, true, poses, links );
	else if ( !Q_stricmp( cls, "func_train" ) )
		TrainPoses( ent, false, poses, links );
	// The swept volume between poses (for the eye volume) is #14's: the hull of
	// the two poses for a translation, and of MoverSweepPose steps for a rotation.
	if ( !hinge.rotating )
		hinge.poseAngles.assign( poses.size(), dvec3( 0, 0, 0 ) );

	// doors: spawn <-> each open pose; func_movelinear: every pair
	if ( links.empty() )
	{
		for ( int i = 1; i < (int)poses.size(); i++ )
		{
			links.push_back( std::make_pair( 0, i ) );
			for ( int j = 1; allPairs && j < i; j++ )
				links.push_back( std::make_pair( j, i ) );
		}
	}
	std::set< std::pair<int, int> > uniq;
	for ( size_t i = 0; i < links.size(); i++ )
	{
		int a = MIN( links[i].first, links[i].second ), b = MAX( links[i].first, links[i].second );
		if ( a != b )
			uniq.insert( std::make_pair( a, b ) );
	}
	links.assign( uniq.begin(), uniq.end() );
}

// The entity whose poses a parented entity rides: follow parentname (the part
// before a ',' attachment) to the top, at most 16 levels.
static int RootMover( int e )
{
	for ( int depth = 0; depth < 16; depth++ )
	{
		char parent[128];
		Q_strncpy( parent, EntValue( &entities[e], "parentname" ), sizeof( parent ) );
		char *comma = strchr( parent, ',' );
		if ( comma )
			*comma = 0;
		int p = parent[0] ? FindEntityByTargetname( parent, -1 ) : -1;
		if ( p < 0 || p == e )
			break;
		e = p;
	}
	return e;
}

//-----------------------------------------------------------------------------
// Stage 2
//-----------------------------------------------------------------------------
struct UpFace
{
	DWinding poly;
	DPlane plane;
	int category, convex, pose;
	int skip;			// obstacle never clipping this face (its own convex), or -1
};

static const CollisionWorld *s_pWorld;
static WalkGraph *s_pGraph;
static const PlayerProfile *s_pProfile;
static ObstacleSet *s_pCrouchObs, *s_pStandObs;
static std::vector<UpFace> *s_pUpFaces;
static std::vector< std::vector<WalkNode> > *s_pFaceNodes;
static std::vector<int> s_Cand[MAX_TOOL_THREADS + 1];

static void MoverGrowOne( int iThread, int i )
{
	MoverPolytope &m = s_pGraph->movers[i];
	ConvexShape shape = s_pWorld->convexes[m.convex].shape;
	if ( m.pose > 0 )
		TransformShape( shape, m.xf );
	if ( !GrowConvex( shape, s_pProfile->Crouch(), m.crouch ) )
		m.crouch = GrownPolytope();
}

// Pass 1: each up face chopped at the 64u grid.
static std::vector< std::vector<DWinding> > *s_pFaceChops;
static void ChopOne( int iThread, int f )
{
	ChopToGrid( ( *s_pUpFaces )[f].poly, ( *s_pFaceChops )[f] );
}

// Pass 2: one chop piece minus the obstacles near it (work items are pieces,
// so one huge floor does not run on a single thread against every obstacle
// under it).
struct ChopItem
{
	int face, piece;
};
static std::vector<ChopItem> *s_pChopItems;

static void PolygonsOne( int iThread, int item )
{
	const ChopItem &ci = ( *s_pChopItems )[item];
	const UpFace &uf = ( *s_pUpFaces )[ci.face];
	std::vector<DWinding> pieces;
	pieces.push_back( DWinding() );
	pieces.back().swap( ( *s_pFaceChops )[ci.face][ci.piece] );
	SubtractObstacles( iThread, *s_pCrouchObs, uf.skip, uf.skip >= 0 ? &uf.plane : NULL, pieces, s_Cand[iThread] );

	std::vector<WalkNode> &out = ( *s_pFaceNodes )[item];
	for ( size_t i = 0; i < pieces.size(); i++ )
	{
		CleanWinding( pieces[i] );
		double area = WalkPolyArea( pieces[i] );
		if ( area <= 0.0 )
			continue;
		out.push_back( WalkNode() );
		WalkNode &n = out.back();
		n.poly.swap( pieces[i] );
		// CCW seen from above (the up faces already are; splits keep it)
		double a2 = 0;
		for ( size_t k = 0; k < n.poly.size(); k++ )
		{
			const dvec3 &p = n.poly[k], &q = n.poly[( k + 1 ) % n.poly.size()];
			a2 += p.x * q.y - q.x * p.y;
		}
		if ( a2 < 0 )
			std::reverse( n.poly.begin(), n.poly.end() );
		n.plane = uf.plane;
		PolyBounds( n.poly, n.mins, n.maxs );
		n.area = area;
		n.category = uf.category;
		n.convex = uf.convex;
		n.pose = uf.pose;
		n.standArea = 0;
		n.slide = uf.plane.n.z < WALK_MIN_NZ;
	}
}

static void StandOne( int iThread, int i )
{
	WalkNode &n = s_pGraph->nodes[i];
	n.stand.assign( 1, n.poly );
	int skip = CategoryIsGrown( n.category ) ? n.convex : -1;
	// No coplanar dedupe here: n.poly already had it, and the stand parts are
	// subsets of n.poly (passing the plane changed no stand_area on the six
	// harness fixtures or fightspace3).
	SubtractObstacles( iThread, *s_pStandObs, skip, NULL, n.stand, s_Cand[iThread] );
	n.standArea = 0;
	for ( size_t k = 0; k < n.stand.size(); k++ )
		n.standArea += WalkPolyArea( n.stand[k] );
}

// Up faces, and with slides the steep faces (SLIDE_MIN_NZ <= n.z < WALK_MIN_NZ).
static void AddUpFaces( const GrownPolytope &p, int category, int convex, int pose, int skip, bool slides,
						std::vector<UpFace> &out )
{
	if ( !p.IsValid() )
		return;
	for ( size_t f = 0; f < p.faces.size(); f++ )
	{
		double nz = p.planes[f].n.z;
		if ( nz < ( slides ? SLIDE_MIN_NZ : WALK_MIN_NZ ) )
			continue;
		out.push_back( UpFace() );
		UpFace &uf = out.back();
		for ( size_t k = 0; k < p.faces[f].size(); k++ )
			uf.poly.push_back( p.verts[p.faces[f][k]] );
		uf.plane = p.planes[f];
		uf.category = category;
		uf.convex = convex;
		uf.pose = pose;
		uf.skip = skip;
	}
}

static ObstacleSet s_CrouchObs, s_StandObs;
static bool s_bCullOffMap = true;
static bool s_bSurf = true;

void SetSurf( bool surf )
{
	s_bSurf = surf;
}

bool SurfEnabled()
{
	return s_bSurf;
}

double SurfRise( const PlayerProfile &p )
{
	double v = p.maxAirSpeed * p.reachMargin;
	return v * v / ( 2.0 * ( p.gravity > 0 ? p.gravity : 1.0 ) );
}

void SetCullOffMap( bool cull )
{
	s_bCullOffMap = cull;
}

// The leaf of a point (world BSP).
static int WalkPointLeaf( const dvec3 &p )
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

static bool OffMap( const DWinding &poly )
{
	if ( poly.empty() )
		return false;
	dvec3 c( 0, 0, 0 );
	for ( size_t i = 0; i < poly.size(); i++ )
		c += poly[i];
	c = c * ( 1.0 / poly.size() );
	const dvec3 up( 0, 0, 1 );
	if ( dleafs[WalkPointLeaf( c + up )].cluster >= 0 )
		return false;
	for ( size_t i = 0; i < poly.size(); i++ )
	{
		dvec3 d = c - poly[i];
		double len = DLength( d );
		dvec3 p = len > 1.0 ? poly[i] + d * ( 1.0 / len ) : c;
		if ( dleafs[WalkPointLeaf( p + up )].cluster >= 0 )
			return false;
	}
	return true;
}

void BuildWalkPolygons( const CollisionWorld &world, const PlayerProfile &profile, WalkGraph &graph )
{
	double t0 = Plat_FloatTime();
	s_pWorld = &world;
	s_pGraph = &graph;
	s_pProfile = &profile;

	s_CrouchObs.Build( world, world.crouch );
	s_StandObs.Build( world, world.stand );
	s_pCrouchObs = &s_CrouchObs;
	s_pStandObs = &s_StandObs;

	// Movers: every tops_only convex at every pose of its entity; a parented
	// entity at its root mover's poses when that root moves.
	graph.movers.clear();
	graph.moverEnts.clear();
	std::vector<DTransform> poses, rootPoses;
	std::vector< std::pair<int, int> > links, rootLinks;
	int lastEntity = -2;
	for ( size_t i = 0; i < world.convexes.size(); i++ )
	{
		const SourceConvex &sc = world.convexes[i];
		if ( sc.category != CAT_TOPS_ONLY )
			continue;
		// an entity's convexes are consecutive: its poses are computed once
		if ( sc.entityIndex != lastEntity )
		{
			MoverEntity me;
			EntityPoses( sc.entityIndex, poses, links, me );
			me.entity = me.root = sc.entityIndex;
			int root = ( sc.entityIndex >= 0 && sc.entityIndex < num_entities ) ? RootMover( sc.entityIndex ) : sc.entityIndex;
			if ( root != sc.entityIndex )
			{
				MoverEntity rootHinge;
				EntityPoses( root, rootPoses, rootLinks, rootHinge );
				if ( rootPoses.size() > 1 )
				{
					poses = rootPoses;
					links = rootLinks;
					me.root = root;
					me.rotating = rootHinge.rotating;
					me.hingeOrigin = rootHinge.hingeOrigin;
					me.hingeAngles = rootHinge.hingeAngles;
					me.poseAngles = rootHinge.poseAngles;
				}
			}
			me.poses = (int)poses.size();
			me.links = links;
			graph.moverEnts.push_back( me );
			graph.stats.moverEntities++;
			graph.stats.moverPoses += (int)poses.size();
			lastEntity = sc.entityIndex;
		}
		for ( size_t p = 0; p < poses.size(); p++ )
		{
			graph.movers.push_back( MoverPolytope() );
			graph.movers.back().convex = (int)i;
			graph.movers.back().pose = (int)p;
			graph.movers.back().xf = poses[p];
		}
	}
	RunThreadsBatched( (int)graph.movers.size(), MoverGrowOne );

	// Up faces: crouch-grown polytopes of every grown category (stand up faces
	// are identical: the support of -B in a direction with n.z > 0 is its
	// z = 0 face for both heights), then the movers.
	std::vector<UpFace> upFaces;
	for ( size_t i = 0; i < world.convexes.size(); i++ )
	{
		int cat = world.convexes[i].category;
		if ( !CategoryIsGrown( cat ) )
			continue;
		// a polytope never clips its own face
		AddUpFaces( world.crouch[i], cat, (int)i, 0, CategoryIsObstacle( cat ) ? (int)i : -1, cat != CAT_WATER, upFaces );
	}
	for ( size_t m = 0; m < graph.movers.size(); m++ )
	{
		const MoverPolytope &mp = graph.movers[m];
		if ( !mp.crouch.IsValid() )
		{
			graph.stats.moverGrowFailed++;
			continue;
		}
		AddUpFaces( mp.crouch, CAT_TOPS_ONLY, mp.convex, mp.pose, -1, true, upFaces );
	}
	graph.stats.upFaces = (int)upFaces.size();

	s_pUpFaces = &upFaces;
	std::vector< std::vector<DWinding> > faceChops( upFaces.size() );
	s_pFaceChops = &faceChops;
	RunThreadsBatched( (int)upFaces.size(), ChopOne );

	std::vector<ChopItem> items;
	for ( size_t f = 0; f < faceChops.size(); f++ )
	{
		for ( size_t k = 0; k < faceChops[f].size(); k++ )
		{
			ChopItem ci = { (int)f, (int)k };
			items.push_back( ci );
		}
	}
	std::vector< std::vector<WalkNode> > faceNodes( items.size() );
	s_pChopItems = &items;
	s_pFaceNodes = &faceNodes;
	RunThreadsBatched( (int)items.size(), PolygonsOne );
	s_pChopItems = NULL;
	s_pFaceChops = NULL;

	graph.nodes.clear();
	size_t total = 0;
	for ( size_t f = 0; f < faceNodes.size(); f++ )
		total += faceNodes[f].size();
	graph.nodes.reserve( total );
	// off-map polygons (#21): every probe of the polygon (its centroid and its
	// vertices pulled 1u toward it, each 1u up) in a leaf of cluster -1. A feet
	// point just above a reachable floor is inside the player box, so it is never
	// in a solid leaf; such a polygon lies outside the sealed map (the top of its
	// outer shell) and no player is ever on it. Skipped when no leaf has a
	// cluster (nothing to tell inside from outside) or with -keepoffmap.
	bool cull = s_bCullOffMap;
	if ( cull )
	{
		cull = false;
		for ( int l = 0; l < numleafs && !cull; l++ )
			cull = dleafs[l].cluster >= 0;
	}
	graph.stats.offMapNodes = 0;
	graph.stats.offMapArea = 0;
	for ( size_t f = 0; f < faceNodes.size(); f++ )
	{
		for ( size_t k = 0; k < faceNodes[f].size(); k++ )
		{
			WalkNode &wn = faceNodes[f][k];
			if ( cull && OffMap( wn.poly ) )
			{
				graph.stats.offMapNodes++;
				graph.stats.offMapArea += wn.area;
				continue;
			}
			graph.nodes.push_back( std::move( wn ) );
		}
		std::vector<WalkNode>().swap( faceNodes[f] );
	}
	if ( graph.stats.offMapNodes )
		Msg( "walk: dropped %d walkable polygons outside the map (%.0f u^2; -keepoffmap keeps them)\n",
			 graph.stats.offMapNodes, graph.stats.offMapArea );
	graph.stats.totalArea = 0;
	graph.stats.slideNodes = 0;
	for ( size_t i = 0; i < graph.nodes.size(); i++ )
	{
		graph.stats.totalArea += graph.nodes[i].area;
		graph.stats.slideNodes += graph.nodes[i].slide;
	}
	graph.stats.tPolys = Plat_FloatTime() - t0;

	t0 = Plat_FloatTime();
	RunThreadsBatched( (int)graph.nodes.size(), StandOne );
	graph.stats.tStand = Plat_FloatTime() - t0;

	s_pUpFaces = NULL;
	s_pFaceNodes = NULL;
}

//-----------------------------------------------------------------------------
// Stage 3: edges
//-----------------------------------------------------------------------------

// Does the vertical segment at (x, y) over [z0, z1] reach FREE_TOL into the
// shrunk polytope q?
static bool SegmentHits( const GrownPolytope &q, double x, double y, double z0, double z1 )
{
	if ( x < q.mins.x || x > q.maxs.x || y < q.mins.y || y > q.maxs.y || z1 < q.mins.z || z0 > q.maxs.z )
		return false;
	// clip [z0, z1] to { n . p <= d - shrink - tol }
	double lo = z0, hi = z1;
	for ( size_t k = 0; k < q.planes.size(); k++ )
	{
		const DPlane &pl = q.planes[k];
		double rhs = pl.d - WALK_SHRINK - FREE_TOL - pl.n.x * x - pl.n.y * y;
		if ( fabs( pl.n.z ) < 1e-12 )
		{
			if ( rhs < 0 )
				return false;
			continue;
		}
		double z = rhs / pl.n.z;
		if ( pl.n.z > 0 )
			hi = MIN( hi, z );
		else
			lo = MAX( lo, z );
		if ( lo > hi )
			return false;
	}
	return true;
}

// The obstacle that last blocked a segment, per thread: neighbouring samples
// of one polygon pair are usually blocked by the same slab, so it is tried first.
static PaddedInt s_LastBlocker[MAX_TOOL_THREADS + 1];

// Is the vertical segment at (x, y) from z0 to z1 (z0 <= z1) clear of every
// shrunk crouch obstacle? Touching one counts as free.
static bool SegmentFree( int iThread, double x, double y, double z0, double z1, std::vector<int> &cand )
{
	const ObstacleSet &obs = *s_pCrouchObs;
	int last = s_LastBlocker[iThread].v;
	if ( last >= 0 && last < (int)obs.polys->size() && obs.use[last] && SegmentHits( ( *obs.polys )[last], x, y, z0, z1 ) )
		return false;
	cand.clear();
	obs.hash.Query( x, y, x, y, cand );
	for ( size_t c = 0; c < cand.size(); c++ )
	{
		if ( SegmentHits( ( *obs.polys )[cand[c]], x, y, z0, z1 ) )
		{
			s_LastBlocker[iThread].v = cand[c];
			return false;
		}
	}
	return true;
}

struct Poly2D
{
	std::vector<double> x, y;
};

static std::vector<Poly2D> s_Poly2D;
static CXYHash s_NodeHash;
static std::vector< std::vector<WalkEdge> > *s_pNodeEdges;
static std::vector<int> s_NodeCand[MAX_TOOL_THREADS + 1];
static std::vector<double> s_Segs[MAX_TOOL_THREADS + 1];		// x0 y0 x1 y1 per boundary sub-segment

struct SInterval
{
	double lo, hi;
};
static std::vector<SInterval> s_Blocked[MAX_TOOL_THREADS + 1];
static std::vector<double> s_ClipA[MAX_TOOL_THREADS + 1];

// The part [t0, t1] of A's edge i (from vertex i to i + 1) inside B's XY
// projection expanded by EDGE_REACH. False when there is none.
static bool EdgeInsidePoly( const Poly2D &A, int i, const Poly2D &B, double &t0, double &t1 )
{
	int na = (int)A.x.size(), nb = (int)B.x.size();
	double px = A.x[i], py = A.y[i];
	double qx = A.x[( i + 1 ) % na] - px, qy = A.y[( i + 1 ) % na] - py;
	t0 = 0, t1 = 1;
	for ( int j = 0; j < nb && t0 <= t1; j++ )
	{
		double ex = B.x[( j + 1 ) % nb] - B.x[j], ey = B.y[( j + 1 ) % nb] - B.y[j];
		double len = sqrt( ex * ex + ey * ey );
		if ( len < 1e-12 )
			continue;
		// outward normal of a CCW polygon's edge
		double mx = ey / len, my = -ex / len;
		double num = mx * ( px - B.x[j] ) + my * ( py - B.y[j] ) - EDGE_REACH;
		double den = mx * qx + my * qy;
		if ( fabs( den ) < 1e-15 )
		{
			if ( num > 0 )
				t0 = 2, t1 = 1;
			continue;
		}
		double t = -num / den;
		if ( den > 0 )
			t1 = MIN( t1, t );
		else
			t0 = MAX( t0, t );
	}
	return t0 <= t1;
}

// Appends the parts of A's boundary inside B's XY projection expanded by
// EDGE_REACH, as x0 y0 x1 y1 quadruples.
static void BoundarySegments( const Poly2D &A, const Poly2D &B, std::vector<double> &segs )
{
	int na = (int)A.x.size();
	for ( int i = 0; i < na; i++ )
	{
		double t0, t1;
		if ( !EdgeInsidePoly( A, i, B, t0, t1 ) )
			continue;
		double px = A.x[i], py = A.y[i];
		double qx = A.x[( i + 1 ) % na] - px, qy = A.y[( i + 1 ) % na] - py;
		segs.push_back( px + t0 * qx );
		segs.push_back( py + t0 * qy );
		segs.push_back( px + t1 * qx );
		segs.push_back( py + t1 * qy );
	}
}

enum
{
	F_WALK = 1, F_JUMP_AB = 2, F_JUMP_BA = 4, F_DROP_AB = 8, F_DROP_BA = 16, F_ALL = 31
};

// The dz band (dz = zB - zA) that gives each flag. Ends are closed, which
// only ever adds edges; a walk edge wins over the others for a pair anyway.
static void FlagBand( int flag, double step, double jump, double &lo, double &hi )
{
	switch ( flag )
	{
	case F_WALK:	lo = -step; hi = step; break;
	case F_JUMP_AB:	lo = step; hi = jump; break;
	case F_DROP_BA:	lo = step; hi = 1e300; break;
	case F_JUMP_BA:	lo = -jump; hi = -step; break;
	default:		lo = -1e300; hi = -step; break;		// F_DROP_AB
	}
}

// The flags whose band a sample at height difference dz falls in.
static int FlagsForDz( double dz, double step, double jump )
{
	if ( fabs( dz ) <= step )
		return F_WALK;
	if ( dz > 0 )
		return F_DROP_BA | ( dz <= jump ? F_JUMP_AB : 0 );
	return F_DROP_AB | ( -dz <= jump ? F_JUMP_BA : 0 );
}

// The s in [0, 1] where dz0 + s (dz1 - dz0) lies in [lo, hi]. False if none.
static bool BandRange( double dz0, double dz1, double lo, double hi, double &s0, double &s1 )
{
	double k = dz1 - dz0;
	if ( fabs( k ) < 1e-12 )
	{
		if ( dz0 < lo || dz0 > hi )
			return false;
		s0 = 0;
		s1 = 1;
		return true;
	}
	double a = ( lo - dz0 ) / k, b = ( hi - dz0 ) / k;
	if ( a > b )
		std::swap( a, b );
	s0 = MAX( a, 0.0 );
	s1 = MIN( b, 1.0 );
	return s0 <= s1;
}

// The s-range (into blocked) of the part of the (s, z) quad that lies in q's
// shrunk planes, in the vertical plane over P(s) = P + s D. The tolerance is
// in the planes (FREE_TOL), as in SegmentHits. Planes that leave the whole
// quad inside are skipped and one that leaves it all outside ends the test;
// only the planes that cut it are clipped (Sutherland-Hodgman in 2-D).
static void ClipQuadByPolytope( int iThread, const GrownPolytope &q, double px, double py, double dx, double dy,
								const double *quad, std::vector<SInterval> &blocked )
{
	enum { MAXC = 64, MAXP = 2 * ( MAXC + 4 ) };
	double cs[MAXC], cz[MAXC], cc[MAXC];		// a_s s + b_z z <= c of the cutting planes
	int nCut = 0;
	std::vector<double> &extra = s_ClipA[iThread];	// cutting planes past MAXC (rare)
	extra.clear();
	for ( size_t k = 0; k < q.planes.size(); k++ )
	{
		const DPlane &pl = q.planes[k];
		double as = pl.n.x * dx + pl.n.y * dy, bz = pl.n.z;
		double c = pl.d - WALK_SHRINK - FREE_TOL - pl.n.x * px - pl.n.y * py;
		int nOut = 0;
		for ( int v = 0; v < 4; v++ )
			nOut += ( as * quad[2 * v] + bz * quad[2 * v + 1] - c > 0 );
		if ( nOut == 4 )
			return;
		if ( nOut == 0 )
			continue;
		if ( nCut < MAXC )
		{
			cs[nCut] = as;
			cz[nCut] = bz;
			cc[nCut] = c;
			nCut++;
		}
		else
		{
			extra.push_back( as );
			extra.push_back( bz );
			extra.push_back( c );
		}
	}

	int nTotal = nCut + (int)extra.size() / 3;
	double bufA[MAXP], bufB[MAXP];
	std::vector<double> heapA, heapB;
	double *a = bufA, *b = bufB;
	if ( nTotal + 4 > MAXC + 4 )
	{
		heapA.resize( 2 * ( nTotal + 4 ) );
		heapB.resize( 2 * ( nTotal + 4 ) );
		a = heapA.data();
		b = heapB.data();
	}
	int n = 4;
	for ( int i = 0; i < 8; i++ )
		a[i] = quad[i];
	for ( int k = 0; k < nTotal && n > 0; k++ )
	{
		double as, bz, c;
		if ( k < nCut )
			as = cs[k], bz = cz[k], c = cc[k];
		else
			as = extra[3 * ( k - nCut )], bz = extra[3 * ( k - nCut ) + 1], c = extra[3 * ( k - nCut ) + 2];
		int m = 0;
		for ( int i = 0; i < n; i++ )
		{
			int j = ( i + 1 ) % n;
			double si = a[2 * i], zi = a[2 * i + 1], sj = a[2 * j], zj = a[2 * j + 1];
			double vi = as * si + bz * zi - c, vj = as * sj + bz * zj - c;
			if ( vi <= 0 )
			{
				b[2 * m] = si;
				b[2 * m + 1] = zi;
				m++;
			}
			if ( ( vi < 0 && vj > 0 ) || ( vi > 0 && vj < 0 ) )
			{
				double t = vi / ( vi - vj );
				b[2 * m] = si + t * ( sj - si );
				b[2 * m + 1] = zi + t * ( zj - zi );
				m++;
			}
		}
		std::swap( a, b );
		n = m;
	}
	if ( n < 1 )
		return;
	SInterval iv = { 1e300, -1e300 };
	for ( int i = 0; i < n; i++ )
	{
		iv.lo = MIN( iv.lo, a[2 * i] );
		iv.hi = MAX( iv.hi, a[2 * i] );
	}
	blocked.push_back( iv );
}

// The parameters s in [S0, S1] of the sub-segment P0 -> P1 where the
// vertical segment between zA(s) and zB(s) reaches into a shrunk crouch
// obstacle, as merged, sorted, closed intervals. For one convex obstacle that
// set is the s-projection of (the vertical region swept between A and B)
// intersected with the polytope: both are convex once the segment is split
// where zA and zB cross, so it is one clip and one interval per piece.
// Returns true, with blocked holding one interval, as soon as a single
// obstacle covers all of [S0, S1] (usually the upper node's own floor slab,
// tried first through the last-blocker cache).
static void ClipObstacle( int iThread, const GrownPolytope &q, double x0, double y0, double dx, double dy, const double *cuts,
						  int nPieces, double zA0, double zA1, double zB0, double zB1, std::vector<SInterval> &blocked )
{
	for ( int p = 0; p < nPieces; p++ )
	{
		double sa = cuts[p], sb = cuts[p + 1];
		double za = zA0 + sa * ( zA1 - zA0 ), zb = zB0 + sa * ( zB1 - zB0 );
		double wa = zA0 + sb * ( zA1 - zA0 ), wb = zB0 + sb * ( zB1 - zB0 );
		double quad[8] = { sa, MIN( za, zb ), sb, MIN( wa, wb ), sb, MAX( wa, wb ), sa, MAX( za, zb ) };
		ClipQuadByPolytope( iThread, q, x0, y0, dx, dy, quad, blocked );
	}
}

static bool BlockedIntervals( int iThread, double x0, double y0, double x1, double y1, const WalkNode &A, const WalkNode &B,
							  double S0, double S1, std::vector<SInterval> &blocked )
{
	blocked.clear();
	double dx = x1 - x0, dy = y1 - y0;
	double zA0 = A.Z( x0, y0 ), zA1 = A.Z( x1, y1 ), zB0 = B.Z( x0, y0 ), zB1 = B.Z( x1, y1 );
	double dz0 = zB0 - zA0, dz1 = zB1 - zA1;

	// pieces of [S0, S1] on which one surface stays below the other
	double cuts[3] = { S0, S1, S1 };
	int nPieces = 1;
	if ( ( dz0 < 0 && dz1 > 0 ) || ( dz0 > 0 && dz1 < 0 ) )
	{
		double sc = dz0 / ( dz0 - dz1 );
		if ( sc > S0 && sc < S1 )
		{
			cuts[1] = sc;
			nPieces = 2;
		}
	}

	// XYZ bounds of the swept region over [S0, S1]
	double ax = x0 + S0 * dx, ay = y0 + S0 * dy, bx = x0 + S1 * dx, by = y0 + S1 * dy;
	double xlo = MIN( ax, bx ), xhi = MAX( ax, bx ), ylo = MIN( ay, by ), yhi = MAX( ay, by );
	double zs[4] = { zA0 + S0 * ( zA1 - zA0 ), zA0 + S1 * ( zA1 - zA0 ), zB0 + S0 * ( zB1 - zB0 ), zB0 + S1 * ( zB1 - zB0 ) };
	double zlo = MIN( MIN( zs[0], zs[1] ), MIN( zs[2], zs[3] ) ), zhi = MAX( MAX( zs[0], zs[1] ), MAX( zs[2], zs[3] ) );

	const ObstacleSet &obs = *s_pCrouchObs;
	int last = s_LastBlocker[iThread].v;
	if ( last >= 0 && last < (int)obs.polys->size() && obs.use[last] )
	{
		const GrownPolytope &q = ( *obs.polys )[last];
		if ( !( xhi < q.mins.x || xlo > q.maxs.x || yhi < q.mins.y || ylo > q.maxs.y || zhi < q.mins.z || zlo > q.maxs.z ) )
		{
			ClipObstacle( iThread, q, x0, y0, dx, dy, cuts, nPieces, zA0, zA1, zB0, zB1, blocked );
			for ( size_t i = 0; i < blocked.size(); i++ )
			{
				if ( blocked[i].lo <= S0 && blocked[i].hi >= S1 )
				{
					blocked[0] = blocked[i];
					blocked.resize( 1 );
					return true;
				}
			}
		}
	}

	std::vector<int> &cand = s_Cand[iThread];
	cand.clear();
	obs.hash.Query( xlo, ylo, xhi, yhi, cand );
	for ( size_t c = 0; c < cand.size(); c++ )
	{
		if ( cand[c] == last )
			continue;	// already clipped above
		const GrownPolytope &q = ( *obs.polys )[cand[c]];
		if ( xhi < q.mins.x || xlo > q.maxs.x || yhi < q.mins.y || ylo > q.maxs.y || zhi < q.mins.z || zlo > q.maxs.z )
			continue;
		size_t before = blocked.size();
		ClipObstacle( iThread, q, x0, y0, dx, dy, cuts, nPieces, zA0, zA1, zB0, zB1, blocked );
		for ( size_t i = before; i < blocked.size(); i++ )
		{
			if ( blocked[i].lo <= S0 && blocked[i].hi >= S1 )
			{
				s_LastBlocker[iThread].v = cand[c];
				blocked[0] = blocked[i];
				blocked.resize( 1 );
				return true;
			}
		}
	}

	std::sort( blocked.begin(), blocked.end(), []( const SInterval &l, const SInterval &r ) { return l.lo < r.lo; } );
	size_t m = 0;
	for ( size_t i = 0; i < blocked.size(); i++ )
	{
		if ( m && blocked[i].lo <= blocked[m - 1].hi )
			blocked[m - 1].hi = MAX( blocked[m - 1].hi, blocked[i].hi );
		else
			blocked[m++] = blocked[i];
	}
	blocked.resize( m );
	return m == 1 && blocked[0].lo <= S0 && blocked[0].hi >= S1;
}

// Is some s in [s0, s1] outside every (merged, closed) blocked interval?
static bool RangeHasFree( const std::vector<SInterval> &blocked, double s0, double s1 )
{
	for ( size_t i = 0; i < blocked.size(); i++ )
	{
		if ( blocked[i].lo <= s0 && blocked[i].hi >= s1 )
			return false;
	}
	return true;
}

// The flags (among want) that the sub-segment x0 y0 -> x1 y1 gives. Quick
// (exact = false): its ends and midpoint against the point test, a subset of
// the answer. Exact: a flag holds where its dz band meets the free
// parameters. Where |dz| <= DZ_TRIVIAL the two surfaces meet and the column
// counts as free in both.
static int SegmentFlags( int iThread, double x0, double y0, double x1, double y1, const WalkNode &A, const WalkNode &B,
						 int want, bool exact, double step, double jump )
{
	double dz0 = B.Z( x0, y0 ) - A.Z( x0, y0 ), dz1 = B.Z( x1, y1 ) - A.Z( x1, y1 );
	double band[5][2];
	int bandFlags = 0;
	for ( int f = 0; f < 5; f++ )
	{
		int flag = 1 << f;
		if ( !( want & flag ) )
			continue;
		double lo, hi;
		FlagBand( flag, step, jump, lo, hi );
		if ( BandRange( dz0, dz1, lo, hi, band[f][0], band[f][1] ) )
			bandFlags |= flag;
	}
	if ( !bandFlags )
		return 0;

	int found = 0;
	for ( int k = 0; k < 3 && !exact; k++ )
	{
		double s = 0.5 * k;
		double x = x0 + s * ( x1 - x0 ), y = y0 + s * ( y1 - y0 );
		double zA = A.Z( x, y ), zB = B.Z( x, y );
		int flags = FlagsForDz( zB - zA, step, jump ) & bandFlags & ~found;
		if ( !flags )
			continue;
		if ( fabs( zB - zA ) > DZ_TRIVIAL && !SegmentFree( iThread, x, y, MIN( zA, zB ), MAX( zA, zB ), s_Cand[iThread] ) )
			continue;
		found |= flags;
		if ( found & F_WALK )
			return found;
	}
	if ( !exact )
		return found;

	// Where the surfaces meet (|dz| <= DZ_TRIVIAL) the column is free.
	double t0 = 0, t1 = -1;
	bool hasTrivial = BandRange( dz0, dz1, -DZ_TRIVIAL, DZ_TRIVIAL, t0, t1 );
	double S0 = 1e300, S1 = -1e300;
	for ( int f = 0; f < 5; f++ )
	{
		int flag = 1 << f;
		if ( !( bandFlags & flag ) || ( found & flag ) )
			continue;
		if ( hasTrivial && MAX( band[f][0], t0 ) <= MIN( band[f][1], t1 ) )
		{
			found |= flag;
			continue;
		}
		S0 = MIN( S0, band[f][0] );
		S1 = MAX( S1, band[f][1] );
	}
	if ( S0 > S1 )
		return found;

	// the free set of [S0, S1], the span of the bands still open
	std::vector<SInterval> &blocked = s_Blocked[iThread];
	if ( BlockedIntervals( iThread, x0, y0, x1, y1, A, B, S0, S1, blocked ) )
		return found;
	for ( int f = 0; f < 5; f++ )
	{
		int flag = 1 << f;
		if ( !( bandFlags & flag ) || ( found & flag ) )
			continue;
		if ( RangeHasFree( blocked, band[f][0], band[f][1] ) )
			found |= flag;
	}
	return found;
}

// Is the XY direction (dx, dy) more than 30 degrees uphill of the slide's
// contour line? The plane's downhill direction is its normal's XY part (z falls
// along +(n.x, n.y)).
static bool SlideUphill( const DPlane &pl, double dx, double dy )
{
	double nxy = sqrt( pl.n.x * pl.n.x + pl.n.y * pl.n.y ), d = sqrt( dx * dx + dy * dy );
	return pl.n.x * dx + pl.n.y * dy < -0.5 * nxy * d;
}

// Can a player sliding on S (a slide, plane pl) cross into O, their footprints
// meeting? He crosses where S's boundary lies in O (leaving across S's edge,
// along its outward normal) or O's lies in S (entering across O's edge, against
// its outward normal). Allowed when one such crossing, longer than a corner
// contact, is not more than 30 degrees uphill of the contour line (sideways
// strafing is kept). With only corner contacts, the direction from S's centroid
// to the contact decides.
static const double SLIDE_CONTACT_MIN = 1e-2;

static bool SlideExitAllowed( const Poly2D &S, const Poly2D &O, const DPlane &pl )
{
	double nxy = sqrt( pl.n.x * pl.n.x + pl.n.y * pl.n.y );
	if ( nxy < 1e-9 )
		return true;
	bool any = false, contact = false;
	double cx = 0, cy = 0;
	for ( int pass = 0; pass < 2; pass++ )
	{
		const Poly2D &P = pass ? O : S, &Q = pass ? S : O;
		int n = (int)P.x.size();
		for ( int i = 0; i < n; i++ )
		{
			double t0, t1;
			if ( !EdgeInsidePoly( P, i, Q, t0, t1 ) )
				continue;
			double ex = P.x[( i + 1 ) % n] - P.x[i], ey = P.y[( i + 1 ) % n] - P.y[i];
			double len = sqrt( ex * ex + ey * ey );
			if ( len < 1e-12 )
				continue;
			if ( !contact )
			{
				contact = true;
				cx = P.x[i] + 0.5 * ( t0 + t1 ) * ex;
				cy = P.y[i] + 0.5 * ( t0 + t1 ) * ey;
			}
			if ( ( t1 - t0 ) * len <= SLIDE_CONTACT_MIN )
				continue;
			any = true;
			// outward normal of a CCW polygon's edge; entering O is against O's
			double mx = ey / len, my = -ex / len;
			if ( pass )
				mx = -mx, my = -my;
			if ( !SlideUphill( pl, mx, my ) )
				return true;
		}
	}
	if ( any || !contact )
		return !any;
	double sx = 0, sy = 0;
	for ( size_t i = 0; i < S.x.size(); i++ )
	{
		sx += S.x[i];
		sy += S.y[i];
	}
	sx /= S.x.size();
	sy /= S.y.size();
	return !SlideUphill( pl, cx - sx, cy - sy );
}

static void EdgesOne( int iThread, int a )
{
	const std::vector<WalkNode> &nodes = s_pGraph->nodes;
	const WalkNode &A = nodes[a];
	const Poly2D &A2 = s_Poly2D[a];
	const double step = s_pProfile->stepHeight;
	const double jump = s_pProfile->maxJumpRise;

	std::vector<int> &cand = s_NodeCand[iThread];
	cand.clear();
	s_NodeHash.Query( A.mins.x - EDGE_REACH, A.mins.y - EDGE_REACH, A.maxs.x + EDGE_REACH, A.maxs.y + EDGE_REACH,
					  cand );

	std::vector<double> &segs = s_Segs[iThread];
	std::vector<WalkEdge> &out = ( *s_pNodeEdges )[a];
	for ( size_t c = 0; c < cand.size(); c++ )
	{
		int b = cand[c];
		if ( b <= a )
			continue;
		const WalkNode &B = nodes[b];
		if ( A.mins.x > B.maxs.x + EDGE_REACH || B.mins.x > A.maxs.x + EDGE_REACH ||
			 A.mins.y > B.maxs.y + EDGE_REACH || B.mins.y > A.maxs.y + EDGE_REACH )
			continue;

		segs.clear();
		BoundarySegments( A2, s_Poly2D[b], segs );
		BoundarySegments( s_Poly2D[b], A2, segs );

		// quick samples on every sub-segment first: most pairs get their walk
		// edge there, and the exact pass then only runs for what is missing. A
		// pair with a slide needs every flag (its walk edge is one-way and may be
		// refused, its drops are kept), so it does not stop at the walk.
		const bool slidePair = A.slide || B.slide;
		int found = 0;
		for ( int pass = 0; pass < 2 && ( found & F_ALL ) != F_ALL && ( slidePair || !( found & F_WALK ) ); pass++ )
		{
			for ( size_t k = 0; k + 3 < segs.size() && ( slidePair || !( found & F_WALK ) ); k += 4 )
				found |= SegmentFlags( iThread, segs[k], segs[k + 1], segs[k + 2], segs[k + 3], A, B, F_ALL & ~found, pass == 1,
									   step, jump );
		}

		// Slides: never jumped from, never left far uphill. A walk-height
		// boundary becomes a one-way slide edge onto the slide, and off it where
		// the crossing is not more than 30 degrees uphill of the slide's contour
		// line (SlideExitAllowed): sideways strafing is kept, the uphill climb a
		// surfer's momentum allows is not modelled. Drops off a slide stay.
		if ( slidePair )
		{
			if ( found & F_WALK )
			{
				if ( !A.slide || SlideExitAllowed( A2, s_Poly2D[b], A.plane ) )
				{
					WalkEdge e = { a, b, EDGE_SLIDE };
					out.push_back( e );
				}
				if ( !B.slide || SlideExitAllowed( s_Poly2D[b], A2, B.plane ) )
				{
					WalkEdge e = { b, a, EDGE_SLIDE };
					out.push_back( e );
				}
			}
			if ( A.slide )
				found &= ~F_JUMP_AB;
			if ( B.slide )
				found &= ~F_JUMP_BA;
		}
		// a walk edge already joins the pair both ways; the others add nothing
		else if ( found & F_WALK )
		{
			WalkEdge e = { a, b, EDGE_WALK };
			out.push_back( e );
			continue;
		}
		if ( found & F_JUMP_AB )
		{
			WalkEdge e = { a, b, EDGE_JUMP };
			out.push_back( e );
		}
		if ( found & F_JUMP_BA )
		{
			WalkEdge e = { b, a, EDGE_JUMP };
			out.push_back( e );
		}
		if ( found & F_DROP_AB )
		{
			WalkEdge e = { a, b, EDGE_DROP };
			out.push_back( e );
		}
		if ( found & F_DROP_BA )
		{
			WalkEdge e = { b, a, EDGE_DROP };
			out.push_back( e );
		}
	}
}

void BuildWalkEdges( const CollisionWorld &world, const PlayerProfile &profile, WalkGraph &graph )
{
	double t0 = Plat_FloatTime();
	s_pWorld = &world;
	s_pGraph = &graph;
	s_pProfile = &profile;
	s_pCrouchObs = &s_CrouchObs;

	int n = (int)graph.nodes.size();
	s_Poly2D.assign( n, Poly2D() );
	std::vector<XYBox> boxes( n );
	std::vector<unsigned char> use( n, 1 );
	for ( int i = 0; i < n; i++ )
	{
		const WalkNode &wn = graph.nodes[i];
		for ( size_t k = 0; k < wn.poly.size(); k++ )
		{
			s_Poly2D[i].x.push_back( wn.poly[k].x );
			s_Poly2D[i].y.push_back( wn.poly[k].y );
		}
		boxes[i].x0 = wn.mins.x;
		boxes[i].y0 = wn.mins.y;
		boxes[i].x1 = wn.maxs.x;
		boxes[i].y1 = wn.maxs.y;
	}
	s_NodeHash.Build( boxes, use );

	std::vector< std::vector<WalkEdge> > nodeEdges( n );
	s_pNodeEdges = &nodeEdges;
	RunThreadsBatched( n, EdgesOne );

	graph.edges.clear();
	memset( graph.stats.edgeCounts, 0, sizeof( graph.stats.edgeCounts ) );
	for ( int i = 0; i < n; i++ )
	{
		for ( size_t k = 0; k < nodeEdges[i].size(); k++ )
		{
			graph.edges.push_back( nodeEdges[i][k] );
			graph.stats.edgeCounts[nodeEdges[i][k].kind]++;
		}
	}
	s_pNodeEdges = NULL;
	graph.stats.tEdges = Plat_FloatTime() - t0;
}

//-----------------------------------------------------------------------------
// Rim nodes (arc launch nodes)
//-----------------------------------------------------------------------------
static std::vector<int> s_RimStart, s_RimAdj;
static std::vector<unsigned char> *s_pRim;
static const WalkGraph *s_pRimGraph;
static std::vector< std::pair<double, double> > s_RimIv[MAX_TOOL_THREADS + 1];
static const double RIM_GAP = 1e-3;	// an uncovered stretch of the boundary at least this long (units)
static const double RIM_DROP = 0.1;	// a neighbour this much lower at the boundary is a step down, not cover

static void RimOne( int iThread, int a )
{
	const Poly2D &A = s_Poly2D[a];
	const WalkNode &NA = s_pRimGraph->nodes[a];
	int na = (int)A.x.size();
	std::vector< std::pair<double, double> > &iv = s_RimIv[iThread];
	unsigned char rim = 0;
	for ( int i = 0; i < na && !rim; i++ )
	{
		double ex = A.x[( i + 1 ) % na] - A.x[i], ey = A.y[( i + 1 ) % na] - A.y[i];
		double len = sqrt( ex * ex + ey * ey );
		if ( len < RIM_GAP )
			continue;
		iv.clear();
		for ( int k = s_RimStart[a]; k < s_RimStart[a + 1]; k++ )
		{
			double t0, t1;
			if ( !EdgeInsidePoly( A, i, s_Poly2D[s_RimAdj[k]], t0, t1 ) )
				continue;
			// a walk neighbour below A here (a step down, or a sliver at the foot
			// of one) leaves A's edge a ledge: launching from it adds height
			const WalkNode &NB = s_pRimGraph->nodes[s_RimAdj[k]];
			bool lower = false;
			for ( int e = 0; e < 2 && !lower; e++ )
			{
				double t = e ? t1 : t0;
				double x = A.x[i] + t * ex, y = A.y[i] + t * ey;
				lower = NB.Z( x, y ) < NA.Z( x, y ) - RIM_DROP;
			}
			if ( !lower )
				iv.push_back( std::make_pair( t0, t1 ) );
		}
		std::sort( iv.begin(), iv.end() );
		double covered = 0, gap = RIM_GAP / len;
		for ( size_t k = 0; k < iv.size() && !rim; k++ )
		{
			if ( iv[k].first > covered + gap )
				rim = 1;
			covered = MAX( covered, iv[k].second );
		}
		if ( covered < 1.0 - gap )
			rim = 1;
	}
	( *s_pRim )[a] = rim;
}

void ComputeRimNodes( const WalkGraph &graph, std::vector<unsigned char> &rim )
{
	int n = (int)graph.nodes.size();
	rim.assign( n, 1 );
	if ( (int)s_Poly2D.size() != n )
		return;	// no edge pass: every node launches
	s_RimStart.assign( n + 1, 0 );
	for ( size_t i = 0; i < graph.edges.size(); i++ )
	{
		const WalkEdge &e = graph.edges[i];
		if ( e.kind != EDGE_WALK )
			continue;
		s_RimStart[e.a + 1]++;
		s_RimStart[e.b + 1]++;
	}
	for ( int i = 0; i < n; i++ )
		s_RimStart[i + 1] += s_RimStart[i];
	s_RimAdj.resize( s_RimStart[n] );
	std::vector<int> fill( s_RimStart.begin(), s_RimStart.end() - 1 );
	for ( size_t i = 0; i < graph.edges.size(); i++ )
	{
		const WalkEdge &e = graph.edges[i];
		if ( e.kind != EDGE_WALK )
			continue;
		s_RimAdj[fill[e.a]++] = e.b;
		s_RimAdj[fill[e.b]++] = e.a;
	}
	s_pRim = &rim;
	s_pRimGraph = &graph;
	RunThreadsBatched( n, RimOne );
	s_pRim = NULL;
	s_pRimGraph = NULL;
	std::vector<int>().swap( s_RimStart );
	std::vector<int>().swap( s_RimAdj );
}

//-----------------------------------------------------------------------------
// Seeds and flood
//-----------------------------------------------------------------------------
static bool PointInPoly2D( const Poly2D &P, double x, double y, double tol )
{
	int n = (int)P.x.size();
	for ( int j = 0; j < n; j++ )
	{
		double ex = P.x[( j + 1 ) % n] - P.x[j], ey = P.y[( j + 1 ) % n] - P.y[j];
		double len = sqrt( ex * ex + ey * ey );
		if ( len < 1e-12 )
			continue;
		if ( ( ey * ( x - P.x[j] ) - ex * ( y - P.y[j] ) ) / len > tol )
			return false;
	}
	return true;
}

// Highest node under the seed whose XY closure contains it, with its plane
// within [seed.z - 256, seed.z + step] and a free vertical segment to it. A
// plane at most SEED_EMBED above the seed needs no segment test: an origin
// placed slightly into its floor starts inside the shrunk floor solid. A node
// that passed the segment test wins over an untested embedded one, so a thin
// slab just above an embedded origin cannot steal the seed from its floor.
static int MapSeed( const dvec3 &p, double step, double maxDown )
{
	const double tol = 1e-3;
	const double SEED_EMBED = 2.0;
	std::vector<int> cand;
	s_NodeHash.Query( p.x, p.y, p.x, p.y, cand );
	int best = -1, bestEmbedded = -1;
	double bestZ = -1e300, bestEmbeddedZ = -1e300;
	for ( size_t c = 0; c < cand.size(); c++ )
	{
		const WalkNode &n = s_pGraph->nodes[cand[c]];
		if ( p.x < n.mins.x - tol || p.x > n.maxs.x + tol || p.y < n.mins.y - tol || p.y > n.maxs.y + tol )
			continue;
		if ( !PointInPoly2D( s_Poly2D[cand[c]], p.x, p.y, tol ) )
			continue;
		double z = n.Z( p.x, p.y );
		if ( z < p.z - maxDown || z > p.z + step )
			continue;
		if ( fabs( z - p.z ) <= DZ_TRIVIAL ||
			 SegmentFree( THREADINDEX_MAIN, p.x, p.y, MIN( z, p.z ), MAX( z, p.z ), s_Cand[THREADINDEX_MAIN] ) )
		{
			if ( z > bestZ )
			{
				best = cand[c];
				bestZ = z;
			}
		}
		else if ( z >= p.z && z - p.z <= SEED_EMBED && z > bestEmbeddedZ )
		{
			bestEmbedded = cand[c];
			bestEmbeddedZ = z;
		}
	}
	return best >= 0 ? best : bestEmbedded;
}

int MapSeedPoint( const dvec3 &p, double step, double maxDown )
{
	if ( (int)s_Poly2D.size() != (int)s_pGraph->nodes.size() )
		return -1;
	return MapSeed( p, step, maxDown );
}

// BFS over the CPU edges (walk both ways, the other kinds a -> b) and the
// special edges, CSR adjacency built once. Flood ids: nodes, then special
// vertices (node count + index). Reached nodes and reached special vertices are
// kept in the order reached, which is how the arc rounds find their frontier.
class CWalkFlood : public IArcFloodSink
{
public:
	CWalkFlood( WalkGraph &graph, const PlayerProfile &profile ) : m_Graph( graph ), m_nHead( 0 ), m_nStamp( 0 )
	{
		m_nNodes = (int)graph.nodes.size();
		m_SurfRise = SurfRise( profile );
		BuildSurf();
		int n = m_nNodes + (int)graph.special.verts.size();
		const std::vector< std::pair<int, int> > &sp = graph.special.edges;
		m_Start.assign( n + 1, 0 );
		for ( size_t i = 0; i < graph.edges.size(); i++ )
		{
			const WalkEdge &e = graph.edges[i];
			m_Start[e.a + 1]++;
			if ( e.kind == EDGE_WALK )
				m_Start[e.b + 1]++;
		}
		for ( size_t i = 0; i < sp.size(); i++ )
			m_Start[sp[i].first + 1]++;
		for ( int i = 0; i < n; i++ )
			m_Start[i + 1] += m_Start[i];
		m_Adj.resize( m_Start[n] );
		std::vector<int> fill( m_Start.begin(), m_Start.end() - 1 );
		for ( size_t i = 0; i < graph.edges.size(); i++ )
		{
			const WalkEdge &e = graph.edges[i];
			m_Adj[fill[e.a]++] = e.b;
			if ( e.kind == EDGE_WALK )
				m_Adj[fill[e.b]++] = e.a;
		}
		for ( size_t i = 0; i < sp.size(); i++ )
			m_Adj[fill[sp[i].first]++] = sp[i].second;
		m_Order.reserve( m_nNodes );
	}
	// seeds, arc landings, special vertices: fresh (the player may carry speed)
	virtual void Reach( int v ) { ReachState( v, true ); }
	virtual void Propagate()
	{
		for ( ; m_nHead < m_Queue.size(); m_nHead++ )
		{
			int u = m_Queue[m_nHead] >> 1;
			bool fresh = ( m_Queue[m_nHead] & 1 ) != 0;
			bool uSlide = u < m_nNodes && m_Graph.nodes[u].slide;
			for ( int k = m_Start[u]; k < m_Start[u + 1]; k++ )
			{
				int w = m_Adj[k];
				// a surfer stays a surfer while moving from slide to slide
				bool wSlide = w < m_nNodes && m_Graph.nodes[w].slide;
				ReachState( w, fresh || !( uSlide && wSlide ) );
			}
			if ( fresh && uSlide && s_bSurf && m_Graph.nodes[u].surf )
				Surf( u );
		}
	}
	int SurfNodes() const { return m_nSurfNodes; }
	virtual const std::vector<int> &Order() const { return m_Order; }
	virtual const std::vector<int> &SpecialOrder() const { return m_SpecialOrder; }

private:
	void ReachState( int v, bool fresh )
	{
		if ( v < m_nNodes )
		{
			if ( !m_Graph.reached[v] )
			{
				m_Graph.reached[v] = 1;
				m_Fresh[v] = fresh;
				m_nSurfNodes += !fresh;
				m_Order.push_back( v );
				m_Queue.push_back( v * 2 + ( fresh ? 1 : 0 ) );
			}
			else if ( fresh && !m_Fresh[v] )
			{
				// reached again with speed to spare: its edges and surf moves run again
				m_Fresh[v] = 1;
				m_Queue.push_back( v * 2 + 1 );
			}
		}
		else if ( !m_Graph.special.reached[v - m_nNodes] )
		{
			m_Graph.special.reached[v - m_nNodes] = 1;
			m_SpecialOrder.push_back( v - m_nNodes );
			m_Queue.push_back( v * 2 + 1 );
		}
	}

	// slide-slide adjacency (either direction of any edge) and each slide's
	// non-slide neighbours (the surface's edges)
	void BuildSurf()
	{
		m_Fresh.assign( m_nNodes, 0 );
		m_nSurfNodes = 0;
		m_SlideStart.assign( m_nNodes + 1, 0 );
		m_ExitStart.assign( m_nNodes + 1, 0 );
		const std::vector<WalkNode> &nodes = m_Graph.nodes;
		const std::vector<WalkEdge> &edges = m_Graph.edges;
		for ( int pass = 0; pass < 2; pass++ )
		{
			std::vector<int> fillS, fillE;
			if ( pass )
			{
				for ( int i = 0; i < m_nNodes; i++ )
				{
					m_SlideStart[i + 1] += m_SlideStart[i];
					m_ExitStart[i + 1] += m_ExitStart[i];
				}
				m_SlideAdj.resize( m_SlideStart[m_nNodes] );
				m_ExitAdj.resize( m_ExitStart[m_nNodes] );
				fillS.assign( m_SlideStart.begin(), m_SlideStart.end() - 1 );
				fillE.assign( m_ExitStart.begin(), m_ExitStart.end() - 1 );
			}
			for ( size_t i = 0; i < edges.size(); i++ )
			{
				int a = edges[i].a, b = edges[i].b;
				bool sa = nodes[a].slide != 0, sb = nodes[b].slide != 0;
				if ( sa && sb )
				{
					if ( pass )
					{
						m_SlideAdj[fillS[a]++] = b;
						m_SlideAdj[fillS[b]++] = a;
					}
					else
					{
						m_SlideStart[a + 1]++;
						m_SlideStart[b + 1]++;
					}
				}
				else if ( sa || sb )
				{
					int s = sa ? a : b, o = sa ? b : a;
					if ( pass )
						m_ExitAdj[fillE[s]++] = o;
					else
						m_ExitStart[s + 1]++;
				}
			}
		}
		m_Stamp.assign( m_nNodes, 0 );
		m_SurfCap.assign( m_nNodes, -1e300 );

		// connected slide surfaces: only those of SURF_MIN_AREA or more surf
		std::vector<WalkNode> &wn = m_Graph.nodes;
		std::vector<int> comp( m_nNodes, -1 ), stack, members;
		for ( int s = 0; s < m_nNodes; s++ )
		{
			wn[s].surf = 0;
			if ( !wn[s].slide || comp[s] >= 0 )
				continue;
			members.clear();
			stack.assign( 1, s );
			comp[s] = s;
			double area = 0;
			while ( !stack.empty() )
			{
				int u = stack.back();
				stack.pop_back();
				members.push_back( u );
				area += wn[u].area;
				for ( int k = m_SlideStart[u]; k < m_SlideStart[u + 1]; k++ )
				{
					int w = m_SlideAdj[k];
					if ( comp[w] < 0 )
					{
						comp[w] = s;
						stack.push_back( w );
					}
				}
			}
			if ( area >= SURF_MIN_AREA )
			{
				for ( size_t k = 0; k < members.size(); k++ )
					wn[members[k]].surf = 1;
			}
		}
	}

	// every slide of S's surface within the energy and distance bounds, and the
	// non-slide nodes at their edges
	// The set is the part of S's slide surface connected to S through slides no
	// higher than cap: a BFS that reached S with a cap at least as high has
	// already covered it, so it is skipped (near-linear on big surfaces). No XY
	// bound: a surfer can follow a ramp's contour without losing height.
	void Surf( int S )
	{
		const std::vector<WalkNode> &nodes = m_Graph.nodes;
		const WalkNode &A = nodes[S];
		const double cap = A.maxs.z + m_SurfRise;
		if ( m_SurfCap[S] >= cap )
			return;
		if ( ++m_nStamp == 0 )
		{
			std::fill( m_Stamp.begin(), m_Stamp.end(), 0 );
			m_nStamp = 1;
		}
		m_Bfs.clear();
		m_Bfs.push_back( S );
		m_Stamp[S] = m_nStamp;
		for ( size_t h = 0; h < m_Bfs.size(); h++ )
		{
			int u = m_Bfs[h];
			m_SurfCap[u] = MAX( m_SurfCap[u], cap );
			if ( u != S )
				ReachState( u, false );
			for ( int k = m_ExitStart[u]; k < m_ExitStart[u + 1]; k++ )
			{
				int e = m_ExitAdj[k];
				if ( nodes[e].mins.z <= cap )
					ReachState( e, true );
			}
			for ( int k = m_SlideStart[u]; k < m_SlideStart[u + 1]; k++ )
			{
				int w = m_SlideAdj[k];
				if ( m_Stamp[w] == m_nStamp )
					continue;
				if ( nodes[w].mins.z > cap )
					continue;
				m_Stamp[w] = m_nStamp;
				m_Bfs.push_back( w );
			}
		}
	}

	WalkGraph &m_Graph;
	int m_nNodes;
	std::vector<int> m_Start, m_Adj, m_Order, m_SpecialOrder, m_Queue;
	size_t m_nHead;
	// surfing (#24)
	double m_SurfRise;
	std::vector<unsigned char> m_Fresh;
	std::vector<double> m_SurfCap;
	std::vector<int> m_SlideStart, m_SlideAdj, m_ExitStart, m_ExitAdj, m_Stamp, m_Bfs;
	int m_nStamp, m_nSurfNodes;
};

// -vehicle-free: from the vehicle seeds, every node joined by walk, jump, drop
// or slide edges in either direction (a car drives up and down what is
// ground-connected). Returns how many nodes it adds.
static int VehicleFlood( WalkGraph &graph, IArcFloodSink &flood )
{
	int n = (int)graph.nodes.size();
	std::vector<int> start( n + 1, 0 ), adj;
	for ( size_t i = 0; i < graph.edges.size(); i++ )
	{
		const WalkEdge &e = graph.edges[i];
		if ( e.kind == EDGE_ARC )
			continue;
		start[e.a + 1]++;
		start[e.b + 1]++;
	}
	for ( int i = 0; i < n; i++ )
		start[i + 1] += start[i];
	adj.resize( start[n] );
	std::vector<int> fill( start.begin(), start.end() - 1 );
	for ( size_t i = 0; i < graph.edges.size(); i++ )
	{
		const WalkEdge &e = graph.edges[i];
		if ( e.kind == EDGE_ARC )
			continue;
		adj[fill[e.a]++] = e.b;
		adj[fill[e.b]++] = e.a;
	}
	std::vector<unsigned char> seen( n, 0 );
	std::vector<int> queue;
	for ( size_t i = 0; i < graph.seeds.size(); i++ )
	{
		int s = graph.seeds[i].node;
		if ( graph.seeds[i].vehicle && s >= 0 && !seen[s] )
		{
			seen[s] = 1;
			queue.push_back( s );
		}
	}
	int added = 0;
	for ( size_t h = 0; h < queue.size(); h++ )
	{
		int u = queue[h];
		added += !graph.reached[u];
		flood.Reach( u );
		for ( int k = start[u]; k < start[u + 1]; k++ )
		{
			if ( !seen[adj[k]] )
			{
				seen[adj[k]] = 1;
				queue.push_back( adj[k] );
			}
		}
	}
	return added;
}

// A point_teleport moves the entity its "target" keyvalue names to its own
// origin on the Teleport input (pointteleport.cpp CPointTeleport::InputTeleport:
// gEntList.FindEntityByName( NULL, m_target, this, activator, caller ); m_target
// is CBaseEntity's "target"). It can move the player when the target is a "!"
// procedural name (!player, !activator, !caller, !picker, !pvsplayer, ...;
// only !self is the teleport itself), or a name no entity in the BSP has (the
// player is not in the BSP, and may be named at run time). A target naming a
// BSP entity (an NPC, a prop) moves that entity, not the player. Nothing checks
// that the input is ever fired: seeding is the safe direction (over-reach).
// With spawnflag 1 (SF_TELEPORT_TO_SPAWN_POS) the destination is the target's
// spawn position, not the origin. For the player that is where it spawned, an
// info_player_* origin that is already a seed, so the origin seed is then only
// over-reach.
static bool PointTeleportMovesPlayer( entity_t *ent )
{
	const char *pszTarget = EntValue( ent, "target" );
	if ( !pszTarget[0] )
		return false;
	if ( pszTarget[0] == '!' )
		return Q_stricmp( pszTarget, "!self" ) != 0;
	return FindEntityByTargetname( pszTarget, -1 ) < 0;
}

bool FloodWalkGraph( const CollisionWorld &world, const PlayerProfile &profile, const std::vector<dvec3> &cliSeeds,
					 const ArcOptions &arcOpts, WalkGraph &graph )
{
	double t0 = Plat_FloatTime();
	s_pWorld = &world;
	s_pGraph = &graph;
	s_pProfile = &profile;
	s_pCrouchObs = &s_CrouchObs;

	graph.seeds.clear();
	for ( int e = 0; e < num_entities; e++ )
	{
		entity_t *ent = &entities[e];
		const char *cls = EntValue( ent, "classname" );
		bool vehicle = !Q_strnicmp( cls, "prop_vehicle_", 13 );
		bool teleport = !Q_stricmp( cls, "point_teleport" ) && PointTeleportMovesPlayer( ent );
		if ( Q_strnicmp( cls, "info_player_", 12 ) && Q_stricmp( cls, "info_ladder_dismount" ) && !vehicle && !teleport )
			continue;
		WalkSeed s;
		s.point = EntOrigin( ent );
		Q_strncpy( s.source, teleport ? "point_teleport" : cls, sizeof( s.source ) );
		s.node = -1;
		s.vehicle = vehicle;
		graph.seeds.push_back( s );
	}
	for ( size_t i = 0; i < cliSeeds.size(); i++ )
	{
		WalkSeed s;
		s.point = cliSeeds[i];
		Q_strncpy( s.source, "cli", sizeof( s.source ) );
		s.node = -1;
		s.vehicle = false;
		graph.seeds.push_back( s );
	}

	int n = (int)graph.nodes.size();
	graph.reached.assign( n, 0 );
	graph.stats.reachedCount = 0;
	graph.stats.reachedArea = 0;
	if ( graph.seeds.empty() )
	{
		Warning( "bspwalk: no seeds: the map has no info_player_*, info_ladder_dismount, prop_vehicle_* or point_teleport "
				 "that can move the player, and no -seed was given\n" );
		graph.stats.tFlood = Plat_FloatTime() - t0;
		return false;
	}

	// special movement: water, ladders, triggers, rides (bw_special.cpp)
	double ts = Plat_FloatTime();
	BuildSpecialGraph( world, profile, graph );
	graph.special.reached.assign( graph.special.verts.size(), 0 );
	graph.stats.tSpecial = Plat_FloatTime() - ts;

	CWalkFlood flood( graph, profile );
	int mapped = 0;
	for ( size_t i = 0; i < graph.seeds.size(); i++ )
	{
		WalkSeed &s = graph.seeds[i];
		s.node = MapSeed( s.point, profile.stepHeight, 256.0 );
		graph.special.stats.vehicleSeeds += s.vehicle;
		if ( s.node < 0 )
		{
			Warning( "bspwalk: seed %s (%.2f %.2f %.2f) is on no walkable polygon\n", s.source, s.point.x, s.point.y,
					 s.point.z );
			continue;
		}
		mapped++;
		flood.Reach( s.node );
	}
	if ( !mapped )
	{
		Warning( "bspwalk: none of the %d seed(s) is on a walkable polygon\n", (int)graph.seeds.size() );
		graph.stats.tFlood = Plat_FloatTime() - t0;
		return false;
	}

	// -vehicle-free first (from the vehicle seeds only); then round 0: the CPU
	// and special edges; then the arc rounds, each followed by the BFS.
	if ( arcOpts.vehicleFree )
		graph.special.stats.vehicleNodes = VehicleFlood( graph, flood );
	flood.Propagate();
	graph.stats.tFlood = Plat_FloatTime() - t0;
	graph.arcs = ArcStats();
	std::vector<unsigned char> gpuReached;
	bool gpuFlood = false;
	if ( arcOpts.enabled && arcOpts.check )
	{
		// -arccheck: a GPU-only flood first, for the node sets; then the checked
		// flood (both paths per arc, following the CPU) from the same round-0 state
		std::vector<unsigned char> reached0 = graph.reached, special0 = graph.special.reached;
		SpecialStats sstats0 = graph.special.stats;
		size_t edges0 = graph.edges.size(), freeArcs0 = graph.freeArcs.size();
		CWalkFlood gpu( flood );
		ArcOptions gpuOpts = arcOpts;
		gpuOpts.check = false;
		Msg( "arccheck: GPU-only flood first\n" );
		RunArcRounds( world, profile, gpuOpts, graph, gpu );
		gpuFlood = !strcmp( graph.arcs.path, "gpu" );
		gpuReached.swap( graph.reached );
		graph.reached = reached0;
		graph.special.reached = special0;
		graph.special.stats = sstats0;
		graph.edges.resize( edges0 );
		graph.freeArcs.resize( freeArcs0 );	// or the arc eyes would get both floods' arcs
		graph.arcs = ArcStats();
		Msg( "arccheck: checked flood (the flood follows the CPU)\n" );
	}
	if ( arcOpts.enabled )
	{
		RunArcRounds( world, profile, arcOpts, graph, flood );
		if ( gpuFlood && graph.arcs.checked )
		{
			ArcStats &as = graph.arcs;
			as.cpuOnlyNodes = as.gpuOnlyNodes = 0;
			int shown = 0;
			for ( int i = 0; i < n; i++ )
			{
				if ( graph.reached[i] && !gpuReached[i] )
				{
					as.cpuOnlyNodes++;
					if ( shown++ < 5 )
					{
						const WalkNode &nd = graph.nodes[i];
						dvec3 c( 0.5 * ( nd.mins.x + nd.maxs.x ), 0.5 * ( nd.mins.y + nd.maxs.y ), 0 );
						c.z = nd.Z( c.x, c.y );
						Warning( "bspwalk: arccheck: node %d reached by the CPU flood only, box centre (%.1f %.1f %.1f)\n", i,
								 c.x, c.y, c.z );
					}
				}
				else if ( !graph.reached[i] && gpuReached[i] )
					as.gpuOnlyNodes++;
			}
			Msg( "arccheck: nodes reached by the CPU flood only %d, by the GPU flood only %d\n", as.cpuOnlyNodes,
				 as.gpuOnlyNodes );
			if ( as.cpuOnlyNodes )
				Warning( "bspwalk: arccheck: the GPU flood misses %d node(s) the CPU reference reaches\n",
						 as.cpuOnlyNodes );
		}
		graph.stats.edgeCounts[EDGE_ARC] = 0;
		for ( size_t i = 0; i < graph.edges.size(); i++ )
			graph.stats.edgeCounts[EDGE_ARC] += graph.edges[i].kind == EDGE_ARC;
	}
	t0 = Plat_FloatTime();

	graph.stats.surfNodes = flood.SurfNodes();
	for ( int i = 0; i < n; i++ )
	{
		if ( graph.reached[i] )
		{
			graph.stats.reachedCount++;
			graph.stats.reachedArea += graph.nodes[i].area;
		}
	}
	SpecialStats &ss = graph.special.stats;
	memset( ss.reached, 0, sizeof( ss.reached ) );
	for ( size_t i = 0; i < graph.special.verts.size(); i++ )
		ss.reached[graph.special.verts[i].kind] += graph.special.reached[i];
	graph.stats.tFlood += Plat_FloatTime() - t0;
	return true;
}
