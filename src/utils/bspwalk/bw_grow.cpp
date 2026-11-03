//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: double-precision windings, convex builders, and the exact
//          Minkowski growth of a convex by the reflected player box.
//
//=============================================================================//

#include "cmdlib.h"
#include "bw_grow.h"

// Two vertices closer than this are the same vertex (units).
static const double WELD_EPSILON = 1e-5;
// Points within this of a clip plane count as on it. Small, because a near-parallel
// plane pair turns it into a vertex error of epsilon / angle.
static const double CHOP_EPSILON = 1e-9;
bool g_bGrowDebug = false;

// Half size of the starting winding for a plane: covers +-16384 with margin.
static const double BASE_WINDING_HALFSIZE = 131072.0;

//-----------------------------------------------------------------------------
// PlayerHull
//-----------------------------------------------------------------------------
void PlayerHull::ReflectedCorners( dvec3 corners[8] ) const
{
	for ( int i = 0; i < 8; i++ )
	{
		corners[i].x = ( i & 1 ) ? halfWidth : -halfWidth;
		corners[i].y = ( i & 2 ) ? halfWidth : -halfWidth;
		corners[i].z = ( i & 4 ) ? 0.0 : -height;
	}
}

//-----------------------------------------------------------------------------
// ConvexShape
//-----------------------------------------------------------------------------
int ConvexShape::AddVert( const dvec3 &p )
{
	for ( size_t i = 0; i < verts.size(); i++ )
	{
		dvec3 d = verts[i] - p;
		if ( DDot( d, d ) < WELD_EPSILON * WELD_EPSILON )
			return (int)i;
	}
	verts.push_back( p );
	return (int)verts.size() - 1;
}

void ConvexShape::Finish()
{
	edges.clear();
	for ( size_t f = 0; f < faces.size(); f++ )
	{
		const std::vector<int> &face = faces[f];
		for ( size_t i = 0; i < face.size(); i++ )
		{
			int a = face[i], b = face[( i + 1 ) % face.size()];
			if ( a == b )
				continue;
			std::pair<int, int> e( a < b ? a : b, a < b ? b : a );
			bool found = false;
			for ( size_t k = 0; k < edges.size() && !found; k++ )
				found = ( edges[k] == e );
			if ( !found )
				edges.push_back( e );
		}
	}

	mins = dvec3( 1e300, 1e300, 1e300 );
	maxs = dvec3( -1e300, -1e300, -1e300 );
	for ( size_t i = 0; i < verts.size(); i++ )
	{
		mins = DMin( mins, verts[i] );
		maxs = DMax( maxs, verts[i] );
	}
}

//-----------------------------------------------------------------------------
// Windings (polylib.cpp ported to double)
//-----------------------------------------------------------------------------
void DBaseWindingForPlane( const dvec3 &normal, double dist, double halfSize, DWinding &out )
{
	// find the major axis
	int x = 0;
	double maxv = -1;
	for ( int i = 0; i < 3; i++ )
	{
		double v = fabs( normal[i] );
		if ( v > maxv )
		{
			x = i;
			maxv = v;
		}
	}

	dvec3 vup( 0, 0, 0 );
	if ( x == 2 )
		vup.x = 1;
	else
		vup.z = 1;

	double v = DDot( vup, normal );
	vup = vup - normal * v;
	DNormalize( vup );

	dvec3 org = normal * dist;
	dvec3 vright = DCross( vup, normal );

	vup = vup * halfSize;
	vright = vright * halfSize;

	// same corner order as polylib: clockwise seen from the front
	out.resize( 4 );
	out[0] = org - vright + vup;
	out[1] = org + vright + vup;
	out[2] = org + vright - vup;
	out[3] = org - vright - vup;
}

void DChopWindingInPlace( DWinding &w, const dvec3 &normal, double dist, double epsilon )
{
	// SIDE_FRONT / SIDE_BACK / SIDE_ON come from mathlib.h (0 / 1 / 2)

	int n = (int)w.size();
	if ( n == 0 )
		return;

	std::vector<double> dists( n + 1 );
	std::vector<int> sides( n + 1 );
	int counts[3] = { 0, 0, 0 };

	for ( int i = 0; i < n; i++ )
	{
		double dot = DDot( w[i], normal ) - dist;
		dists[i] = dot;
		if ( dot > epsilon )
			sides[i] = SIDE_FRONT;
		else if ( dot < -epsilon )
			sides[i] = SIDE_BACK;
		else
			sides[i] = SIDE_ON;
		counts[sides[i]]++;
	}
	sides[n] = sides[0];
	dists[n] = dists[0];

	if ( !counts[SIDE_FRONT] )
	{
		w.clear();
		return;
	}
	if ( !counts[SIDE_BACK] )
		return;

	DWinding f;
	f.reserve( n + 4 );
	for ( int i = 0; i < n; i++ )
	{
		const dvec3 &p1 = w[i];

		if ( sides[i] == SIDE_ON )
		{
			f.push_back( p1 );
			continue;
		}
		if ( sides[i] == SIDE_FRONT )
			f.push_back( p1 );

		if ( sides[i + 1] == SIDE_ON || sides[i + 1] == sides[i] )
			continue;

		// generate a split point
		const dvec3 &p2 = w[( i + 1 ) % n];
		double dot = dists[i] / ( dists[i] - dists[i + 1] );
		dvec3 mid;
		for ( int j = 0; j < 3; j++ )
		{
			// avoid round off error when possible
			if ( normal[j] == 1.0 )
				mid[j] = dist;
			else if ( normal[j] == -1.0 )
				mid[j] = -dist;
			else
				mid[j] = p1[j] + dot * ( p2[j] - p1[j] );
		}
		f.push_back( mid );
	}
	w.swap( f );
}

double DWindingArea( const DWinding &w )
{
	double total = 0;
	for ( size_t i = 2; i < w.size(); i++ )
	{
		dvec3 c = DCross( w[i - 1] - w[0], w[i] - w[0] );
		total += 0.5 * DLength( c );
	}
	return total;
}

//-----------------------------------------------------------------------------
// Builders
//-----------------------------------------------------------------------------

// Adds a polygon (by vertex positions), welding its vertices, oriented so its
// normal points away from 'inside'. Returns false if it is degenerate.
static bool AddFaceOutward( ConvexShape &s, const dvec3 *pts, int n, const dvec3 &inside, const dvec3 *pKnownNormal )
{
	std::vector<int> idx;
	for ( int i = 0; i < n; i++ )
	{
		int k = s.AddVert( pts[i] );
		if ( !idx.empty() && ( idx.back() == k || idx.front() == k ) )
			continue;
		idx.push_back( k );
	}
	if ( idx.size() < 3 )
		return false;

	// Newell normal of the welded polygon
	dvec3 nrm( 0, 0, 0 );
	for ( size_t i = 0; i < idx.size(); i++ )
	{
		const dvec3 &a = s.verts[idx[i]];
		const dvec3 &b = s.verts[idx[( i + 1 ) % idx.size()]];
		nrm.x += ( a.y - b.y ) * ( a.z + b.z );
		nrm.y += ( a.z - b.z ) * ( a.x + b.x );
		nrm.z += ( a.x - b.x ) * ( a.y + b.y );
	}
	if ( DNormalize( nrm ) < 1e-9 )
		return false;

	dvec3 outward = pKnownNormal ? *pKnownNormal : nrm;
	if ( !pKnownNormal && DDot( outward, s.verts[idx[0]] - inside ) < 0 )
		outward = -outward;

	// make the vertex order CCW seen from outside
	if ( DDot( nrm, outward ) < 0 )
	{
		for ( size_t i = 0, j = idx.size() - 1; i < j; i++, j-- )
		{
			int t = idx[i];
			idx[i] = idx[j];
			idx[j] = t;
		}
	}
	s.faces.push_back( idx );
	s.faceNormals.push_back( outward );
	return true;
}

// Clips a base winding of every plane by all the others. pFacePlane, if given,
// receives the plane index of each output face (redundant planes give none).
static bool BuildFromPlanes( const DPlane *planes, int numPlanes, ConvexShape &out, std::vector<int> *pFacePlane )
{
	out = ConvexShape();
	if ( pFacePlane )
		pFacePlane->clear();
	DWinding w;
	std::vector<DWinding> windings;
	std::vector<int> windingPlane;
	for ( int i = 0; i < numPlanes; i++ )
	{
		DBaseWindingForPlane( planes[i].n, planes[i].d, BASE_WINDING_HALFSIZE, w );
		for ( int j = 0; j < numPlanes && !w.empty(); j++ )
		{
			if ( j == i )
				continue;
			// keep the back side of plane j (inside the convex)
			DChopWindingInPlace( w, -planes[j].n, -planes[j].d, CHOP_EPSILON );
		}
		if ( w.size() < 3 || DWindingArea( w ) < 1e-9 )
			continue;
		windings.push_back( w );
		windingPlane.push_back( i );
	}
	if ( windings.size() < 3 )
		return false;

	// The windings come out clockwise seen from the front (polylib order);
	// AddFaceOutward fixes the order against the known plane normal.
	dvec3 centre( 0, 0, 0 );
	int nc = 0;
	for ( size_t i = 0; i < windings.size(); i++ )
		for ( size_t k = 0; k < windings[i].size(); k++, nc++ )
			centre += windings[i][k];
	centre = centre * ( 1.0 / nc );

	for ( size_t i = 0; i < windings.size(); i++ )
	{
		if ( AddFaceOutward( out, windings[i].data(), (int)windings[i].size(), centre, &planes[windingPlane[i]].n ) && pFacePlane )
			pFacePlane->push_back( windingPlane[i] );
	}

	out.Finish();
	return out.faces.size() >= 4;
}

bool ConvexFromPlanes( const DPlane *planes, int numPlanes, ConvexShape &out )
{
	return BuildFromPlanes( planes, numPlanes, out, NULL );
}

//-----------------------------------------------------------------------------
// Convex hull of a point set (quickhull, double precision). A point within
// HULL_EPSILON of the current hull counts as inside, so the result can only
// shrink by that much: the safe direction for collision.
//-----------------------------------------------------------------------------
static const double HULL_EPSILON = 1e-6;

struct HullFace
{
	int v[3];
	dvec3 n;
	double d;
	bool alive;
};

static void MakeHullFace( const std::vector<dvec3> &P, int a, int b, int c, const dvec3 &interior, HullFace &f )
{
	dvec3 n = DCross( P[b] - P[a], P[c] - P[a] );
	DNormalize( n );
	if ( DDot( n, P[a] - interior ) < 0 )
	{
		int t = b;
		b = c;
		c = t;
		n = -n;
	}
	f.v[0] = a;
	f.v[1] = b;
	f.v[2] = c;
	f.n = n;
	f.d = DDot( n, P[a] );
	f.alive = true;
}

bool ConvexFromPoints( const dvec3 *pts, int numPts, ConvexShape &out )
{
	out = ConvexShape();

	// weld duplicates (vphysics lists each vertex once per triangle)
	ConvexShape welded;
	for ( int i = 0; i < numPts; i++ )
		welded.AddVert( pts[i] );
	const std::vector<dvec3> &P = welded.verts;
	int n = (int)P.size();
	if ( n < 4 )
		return false;

	// initial tetrahedron from extreme points
	int i0 = 0;
	for ( int i = 1; i < n; i++ )
		if ( P[i].x < P[i0].x )
			i0 = i;
	int i1 = -1;
	double best = 1e-6;
	for ( int i = 0; i < n; i++ )
	{
		double l = DLength( P[i] - P[i0] );
		if ( l > best )
		{
			best = l;
			i1 = i;
		}
	}
	if ( i1 < 0 )
		return false;
	dvec3 u = P[i1] - P[i0];
	DNormalize( u );
	int i2 = -1;
	best = 1e-6;
	for ( int i = 0; i < n; i++ )
	{
		double l = DLength( DCross( u, P[i] - P[i0] ) );
		if ( l > best )
		{
			best = l;
			i2 = i;
		}
	}
	if ( i2 < 0 )
		return false;
	dvec3 pn = DCross( P[i1] - P[i0], P[i2] - P[i0] );
	DNormalize( pn );
	int i3 = -1;
	best = 1e-6;
	for ( int i = 0; i < n; i++ )
	{
		double l = fabs( DDot( pn, P[i] - P[i0] ) );
		if ( l > best )
		{
			best = l;
			i3 = i;
		}
	}
	if ( i3 < 0 )
		return false;	// flat point set

	dvec3 interior = ( P[i0] + P[i1] + P[i2] + P[i3] ) * 0.25;
	std::vector<HullFace> faces( 4 );
	MakeHullFace( P, i0, i1, i2, interior, faces[0] );
	MakeHullFace( P, i0, i1, i3, interior, faces[1] );
	MakeHullFace( P, i0, i2, i3, interior, faces[2] );
	MakeHullFace( P, i1, i2, i3, interior, faces[3] );

	// Quickhull: always add the point furthest outside the current hull (a
	// far point is never nearly coplanar with what it sees), and grow its
	// visible region by adjacency so it is connected and its horizon is one
	// loop. A point once inside stays inside, since the hull only grows.
	std::vector<unsigned char> done( n, 0 );
	done[i0] = done[i1] = done[i2] = done[i3] = 1;
	std::vector<int> visible;
	std::vector<unsigned char> isVisible;
	std::vector< std::pair<int, int> > horizon;
	for ( ;; )
	{
		int bestPt = -1, bestFace = -1;
		double bestDist = HULL_EPSILON;
		for ( int p = 0; p < n; p++ )
		{
			if ( done[p] )
				continue;
			double maxd = -1e300;
			int argf = -1;
			for ( size_t f = 0; f < faces.size(); f++ )
			{
				double d = DDot( faces[f].n, P[p] ) - faces[f].d;
				if ( d > maxd )
				{
					maxd = d;
					argf = (int)f;
				}
			}
			if ( maxd <= HULL_EPSILON )
			{
				done[p] = 1;
				continue;
			}
			if ( maxd > bestDist )
			{
				bestDist = maxd;
				bestPt = p;
				bestFace = argf;
			}
		}
		if ( bestPt < 0 )
			break;
		done[bestPt] = 1;
		const dvec3 &pt = P[bestPt];

		// visible faces: flood from the face the point is furthest above
		isVisible.assign( faces.size(), 0 );
		visible.clear();
		horizon.clear();
		visible.push_back( bestFace );
		isVisible[bestFace] = 1;
		for ( size_t q = 0; q < visible.size(); q++ )
		{
			const HullFace &f = faces[visible[q]];
			for ( int k = 0; k < 3; k++ )
			{
				int a = f.v[k], b = f.v[( k + 1 ) % 3];
				// the neighbour across edge a->b holds b->a
				int g = -1;
				for ( size_t j = 0; j < faces.size() && g < 0; j++ )
				{
					const HullFace &h = faces[j];
					for ( int m = 0; m < 3; m++ )
					{
						if ( h.v[m] == b && h.v[( m + 1 ) % 3] == a )
						{
							g = (int)j;
							break;
						}
					}
				}
				if ( g >= 0 && isVisible[g] )
					continue;
				if ( g >= 0 && DDot( faces[g].n, pt ) - faces[g].d > HULL_EPSILON )
				{
					isVisible[g] = 1;
					visible.push_back( g );
					continue;
				}
				horizon.push_back( std::pair<int, int>( a, b ) );
			}
		}

		// replace the visible faces with a fan from the horizon to the point
		std::vector<HullFace> next;
		next.reserve( faces.size() + horizon.size() );
		for ( size_t f = 0; f < faces.size(); f++ )
			if ( !isVisible[f] )
				next.push_back( faces[f] );
		for ( size_t h = 0; h < horizon.size(); h++ )
		{
			HullFace f;
			MakeHullFace( P, horizon[h].first, horizon[h].second, bestPt, interior, f );
			next.push_back( f );
		}
		faces.swap( next );
	}

	for ( size_t f = 0; f < faces.size(); f++ )
	{
		std::vector<int> idx( 3 );
		for ( int k = 0; k < 3; k++ )
			idx[k] = out.AddVert( P[faces[f].v[k]] );
		out.faces.push_back( idx );
		out.faceNormals.push_back( faces[f].n );
	}
	out.Finish();
	return out.faces.size() >= 4;
}

bool ConvexFromTriangles( const dvec3 *triVerts, int numTris, ConvexShape &out )
{
	// The triangulation is not trusted: vphysics stores float vertices, so
	// "planar" quads are slightly bent and its diagonals need not be edges of
	// the true hull. Missing a hull edge would drop an edge x axis facet and
	// overgrow the sum, so rebuild the hull from the points.
	return ConvexFromPoints( triVerts, numTris * 3, out );
}

void ConvexFromBox( const dvec3 &mins, const dvec3 &maxs, const DTransform &xf, ConvexShape &out )
{
	out = ConvexShape();
	dvec3 c[8];
	for ( int i = 0; i < 8; i++ )
	{
		dvec3 p( ( i & 1 ) ? maxs.x : mins.x, ( i & 2 ) ? maxs.y : mins.y, ( i & 4 ) ? maxs.z : mins.z );
		c[i] = xf.Apply( p );
	}
	dvec3 centre = xf.Apply( ( mins + maxs ) * 0.5 );

	static const int quads[6][4] =
	{
		{ 0, 2, 6, 4 }, { 1, 3, 7, 5 },		// -X, +X
		{ 0, 1, 5, 4 }, { 2, 3, 7, 6 },		// -Y, +Y
		{ 0, 1, 3, 2 }, { 4, 5, 7, 6 },		// -Z, +Z
	};
	for ( int f = 0; f < 6; f++ )
	{
		dvec3 pts[4] = { c[quads[f][0]], c[quads[f][1]], c[quads[f][2]], c[quads[f][3]] };
		AddFaceOutward( out, pts, 4, centre, NULL );
	}
	out.Finish();
}

bool ConvexFromTriangle( const dvec3 &a, const dvec3 &b, const dvec3 &c, ConvexShape &out )
{
	out = ConvexShape();
	out.flat = true;
	dvec3 n = DCross( b - a, c - a );
	if ( DLength( n ) < 1e-6 )
		return false;
	DNormalize( n );

	std::vector<int> face;
	face.push_back( out.AddVert( a ) );
	face.push_back( out.AddVert( b ) );
	face.push_back( out.AddVert( c ) );
	if ( out.verts.size() != 3 )
		return false;
	out.faces.push_back( face );
	out.faceNormals.push_back( n );
	out.Finish();
	return true;
}

double PolytopeVolume( const std::vector<dvec3> &verts, const std::vector< std::vector<int> > &faces )
{
	if ( verts.empty() )
		return 0;
	dvec3 ref( 0, 0, 0 );
	for ( size_t i = 0; i < verts.size(); i++ )
		ref += verts[i];
	ref = ref * ( 1.0 / verts.size() );

	double vol = 0;
	for ( size_t f = 0; f < faces.size(); f++ )
	{
		const std::vector<int> &face = faces[f];
		for ( size_t i = 2; i < face.size(); i++ )
		{
			dvec3 a = verts[face[0]] - ref, b = verts[face[i - 1]] - ref, c = verts[face[i]] - ref;
			vol += DDot( a, DCross( b, c ) ) / 6.0;
		}
	}
	return vol;
}

//-----------------------------------------------------------------------------
// Minkowski growth
//-----------------------------------------------------------------------------
enum CandidateKind
{
	CAND_SOURCE_FACE = 0,	// a face plane of the source convex
	CAND_AXIAL,				// a face of the box
	CAND_BEVEL,				// source edge x box edge
	CAND_SUM_HULL,			// face of the hull of the sum points (safety net, see GrowConvex)
};

struct Candidate
{
	dvec3 n;
	int kind;
};

static void AddCandidate( std::vector<Candidate> &cands, dvec3 n, int kind )
{
	// No near-duplicate merging here: two different edges can differ in
	// direction by 1e-7 rad and still give two distinct facets. Facets are
	// deduplicated exactly, by their support sets, after the facet test.
	if ( DNormalize( n ) < 1e-9 )
		return;
	Candidate c;
	c.n = n;
	c.kind = kind;
	cands.push_back( c );
}

// The support set of a direction: the source vertices and box corners that
// attain the max of n . x. Two candidates with equal support sets are the
// same face of the sum.
struct SupportSet
{
	std::vector<int> srcVerts;	// ascending
	int cornerMask;				// bit c = corner c of -B
	double maxP;

	bool operator==( const SupportSet &o ) const { return cornerMask == o.cornerMask && srcVerts == o.srcVerts; }
};

// Is n the normal of a 2-D face of src (+) (-B)? That face is
// face_src(n) (+) face_-B(n); it is 2-D iff the directions spanned by the two
// support sets have rank 2. Too generous a tolerance only keeps an extra
// supporting plane (harmless); too strict would drop a facet and overgrow.
static bool IsFacet( const ConvexShape &src, const dvec3 corners[8], const dvec3 &n, double tolP, SupportSet &ss )
{
	ss.maxP = -1e300;
	for ( size_t i = 0; i < src.verts.size(); i++ )
	{
		double d = DDot( n, src.verts[i] );
		if ( d > ss.maxP )
			ss.maxP = d;
	}

	double maxB = -1e300;
	for ( int c = 0; c < 8; c++ )
	{
		double d = DDot( n, corners[c] );
		if ( d > maxB )
			maxB = d;
	}

	ss.srcVerts.clear();
	ss.cornerMask = 0;
	for ( size_t i = 0; i < src.verts.size(); i++ )
	{
		if ( DDot( n, src.verts[i] ) >= ss.maxP - tolP )
			ss.srcVerts.push_back( (int)i );
	}
	for ( int c = 0; c < 8; c++ )
	{
		if ( DDot( n, corners[c] ) >= maxB - 1e-9 )
			ss.cornerMask |= 1 << c;
	}

	// spanning directions of both support sets
	dvec3 dirs[64];
	int numDirs = 0;
	for ( size_t i = 1; i < ss.srcVerts.size() && numDirs < 56; i++ )
		dirs[numDirs++] = src.verts[ss.srcVerts[i]] - src.verts[ss.srcVerts[0]];
	int firstB = -1;
	for ( int c = 0; c < 8; c++ )
	{
		if ( !( ss.cornerMask & ( 1 << c ) ) )
			continue;
		if ( firstB < 0 )
			firstB = c;
		else
			dirs[numDirs++] = corners[c] - corners[firstB];
	}

	// rank >= 2 with a width above 1e-6u
	int longest = -1;
	double longestLen = 1e-6;
	for ( int i = 0; i < numDirs; i++ )
	{
		double l = DLength( dirs[i] );
		if ( l > longestLen )
		{
			longestLen = l;
			longest = i;
		}
	}
	if ( longest < 0 )
		return false;
	dvec3 u = dirs[longest] * ( 1.0 / longestLen );
	for ( int i = 0; i < numDirs; i++ )
	{
		if ( DLength( DCross( u, dirs[i] ) ) > 1e-6 )
			return true;
	}
	return false;
}

struct P2
{
	double u, v;
	dvec3 p;
};

// z of (b - a) x (c - a) in the plane basis; > 0 is a left turn
static double Cross2( const P2 &a, const P2 &b, const P2 &c )
{
	return ( b.u - a.u ) * ( c.v - a.v ) - ( b.v - a.v ) * ( c.u - a.u );
}

// The face polygon of a facet: the 2-D convex hull (monotone chain) of the
// sum points of its support set, CCW seen from outside (+n). Every vertex is
// an exact sum point v + c, so no plane intersections are computed.
static void FacetPolygon( const ConvexShape &src, const dvec3 corners[8], const dvec3 &n, const SupportSet &ss,
						  std::vector<dvec3> &poly )
{
	// in-plane basis with u x v = n
	dvec3 u = ( fabs( n.x ) < 0.6 ) ? DCross( dvec3( 1, 0, 0 ), n ) : DCross( dvec3( 0, 1, 0 ), n );
	DNormalize( u );
	dvec3 v = DCross( n, u );

	std::vector<P2> pts;
	for ( size_t i = 0; i < ss.srcVerts.size(); i++ )
	{
		for ( int c = 0; c < 8; c++ )
		{
			if ( !( ss.cornerMask & ( 1 << c ) ) )
				continue;
			P2 q;
			q.p = src.verts[ss.srcVerts[i]] + corners[c];
			q.u = DDot( q.p, u );
			q.v = DDot( q.p, v );
			pts.push_back( q );
		}
	}

	// sort by (u, v)
	for ( size_t i = 1; i < pts.size(); i++ )
	{
		P2 t = pts[i];
		size_t j = i;
		while ( j > 0 && ( pts[j - 1].u > t.u || ( pts[j - 1].u == t.u && pts[j - 1].v > t.v ) ) )
		{
			pts[j] = pts[j - 1];
			j--;
		}
		pts[j] = t;
	}

	// Andrew's monotone chain; collinear points are dropped
	int np = (int)pts.size();
	std::vector<int> h( 2 * np + 1 );
	int k = 0;
	for ( int i = 0; i < np; i++ )
	{
		while ( k >= 2 && Cross2( pts[h[k - 2]], pts[h[k - 1]], pts[i] ) <= 0 )
			k--;
		h[k++] = i;
	}
	for ( int i = np - 2, t = k + 1; i >= 0; i-- )
	{
		while ( k >= t && Cross2( pts[h[k - 2]], pts[h[k - 1]], pts[i] ) <= 0 )
			k--;
		h[k++] = i;
	}
	poly.clear();
	for ( int i = 0; i < k - 1; i++ )
		poly.push_back( pts[h[i]].p );
}

// Tolerance of the support sets: a source vertex within this of the max of
// n . v counts as on the face. It is >= HULL_EPSILON (a hull keeps points that
// close to a face) and grows with the coordinates; facets merged by it are at
// most this far apart (VerifyGrown bounds the resulting overgrowth).
static double SupportTolerance( const ConvexShape &src )
{
	double scale = 1.0;
	for ( size_t i = 0; i < src.verts.size(); i++ )
		for ( int k = 0; k < 3; k++ )
			scale = MAX( scale, fabs( src.verts[i][k] ) );
	return 1e-9 * scale + 2e-6;
}

bool GrowConvex( const ConvexShape &srcIn, const PlayerHull &hull, GrownPolytope &out )
{
	out = GrownPolytope();
	if ( srcIn.verts.size() < 3 )
		return false;

	// Work on the hull of the source points, whatever built them. Windings
	// clipped from float brush planes with near-parallel micro sides, and
	// vphysics' bent quads, are convex only to ~1e-4u, and their edges need
	// not be the hull's; a missing hull edge drops an edge x axis facet and
	// overgrows the sum. A flat shape (displacement triangle) is used as is.
	ConvexShape hullShape;
	const ConvexShape *pSrc = &srcIn;
	if ( !srcIn.flat && ConvexFromPoints( srcIn.verts.data(), (int)srcIn.verts.size(), hullShape ) )
		pSrc = &hullShape;
	const ConvexShape &src = *pSrc;		// candidate normals only

	dvec3 corners[8];
	hull.ReflectedCorners( corners );

	//
	// Candidate normals: source faces (both sides; a flat shape has two),
	// the 6 box faces, and +-(edge x axis) for every source edge. This set
	// holds every facet normal of a sum of two convex polytopes.
	//
	std::vector<Candidate> cands;
	cands.reserve( src.faceNormals.size() * 2 + 6 + src.edges.size() * 6 );
	for ( size_t f = 0; f < src.faceNormals.size(); f++ )
	{
		AddCandidate( cands, src.faceNormals[f], CAND_SOURCE_FACE );
		AddCandidate( cands, -src.faceNormals[f], src.flat ? CAND_SOURCE_FACE : CAND_BEVEL );
	}
	for ( int a = 0; a < 3; a++ )
	{
		dvec3 axis( 0, 0, 0 );
		axis[a] = 1.0;
		AddCandidate( cands, axis, CAND_AXIAL );
		AddCandidate( cands, -axis, CAND_AXIAL );
	}
	for ( size_t e = 0; e < src.edges.size(); e++ )
	{
		dvec3 d = src.verts[src.edges[e].second] - src.verts[src.edges[e].first];
		if ( DNormalize( d ) < 1e-9 )
			continue;
		for ( int a = 0; a < 3; a++ )
		{
			dvec3 axis( 0, 0, 0 );
			axis[a] = 1.0;
			dvec3 n = DCross( d, axis );
			AddCandidate( cands, n, CAND_BEVEL );
			AddCandidate( cands, -n, CAND_BEVEL );
		}
	}

	// Safety net: the face normals of the hull of the sum points themselves.
	// The set above is complete for exact input, but float brush planes leave
	// micro-chamfers (~1e-4u wide) whose sliver faces and edges the source
	// hull can miss, and fightspace3 showed up to 4e-4u of overgrowth from
	// them. Any extra candidate is safe: its offset is the exact support, so it
	// can only cut overgrowth, never true volume.
	if ( !srcIn.flat )
	{
		std::vector<dvec3> sum;
		sum.reserve( srcIn.verts.size() * 8 );
		for ( size_t i = 0; i < srcIn.verts.size(); i++ )
			for ( int c = 0; c < 8; c++ )
				sum.push_back( srcIn.verts[i] + corners[c] );
		ConvexShape sumHull;
		if ( ConvexFromPoints( sum.data(), (int)sum.size(), sumHull ) )
		{
			for ( size_t f = 0; f < sumHull.faceNormals.size(); f++ )
				AddCandidate( cands, sumHull.faceNormals[f], CAND_SUM_HULL );
		}
	}

	//
	// Keep the facets, one per distinct support set; offset = support of the
	// source + support of -B.
	//
	double tolP = SupportTolerance( srcIn );

	std::vector<SupportSet> sets;
	std::vector<dvec3> setNormals;
	SupportSet ss;
	for ( size_t c = 0; c < cands.size(); c++ )
	{
		bool facet = IsFacet( srcIn, corners, cands[c].n, tolP, ss );
		int dup = -1;
		if ( facet )
		{
			for ( size_t k = 0; k < sets.size() && dup < 0; k++ )
				if ( sets[k] == ss )
					dup = (int)k;
		}
		if ( g_bGrowDebug )
			Msg( "  cand %3d kind %d n (%.9f %.9f %.9f) maxP %.9f -> %s\n", (int)c, cands[c].kind, cands[c].n.x,
				 cands[c].n.y, cands[c].n.z, ss.maxP, !facet ? "-" : ( dup >= 0 ? "same face" : "FACET" ) );
		if ( !facet )
			continue;
		if ( dup >= 0 )
			continue;
		sets.push_back( ss );
		setNormals.push_back( cands[c].n );
	}
	if ( sets.size() < 4 )
		return false;

	//
	// Faces: the hull of each facet's sum points. Vertices are welded.
	//
	ConvexShape welded;
	std::vector<dvec3> poly;
	for ( size_t k = 0; k < sets.size(); k++ )
	{
		FacetPolygon( srcIn, corners, setNormals[k], sets[k], poly );
		std::vector<int> face;
		for ( size_t i = 0; i < poly.size(); i++ )
		{
			int vi = welded.AddVert( poly[i] );
			if ( face.empty() || ( face.back() != vi && face.front() != vi ) )
				face.push_back( vi );
		}
		if ( face.size() < 3 )
		{
			if ( g_bGrowDebug )
				Msg( "  facet %d (%.9f %.9f %.9f) welded to fewer than 3 vertices\n", (int)k, setNormals[k].x,
					 setNormals[k].y, setNormals[k].z );
			continue;
		}
		DPlane p;
		p.n = setNormals[k];
		p.d = sets[k].maxP + hull.ReflectedSupport( setNormals[k] );
		out.planes.push_back( p );
		out.faces.push_back( face );
	}
	if ( g_bGrowDebug )
		Msg( "  %d candidates, %d facets, %d faces\n", (int)cands.size(), (int)sets.size(), (int)out.faces.size() );
	if ( out.faces.size() < 4 )
	{
		out = GrownPolytope();	// a failed grow is empty: no planes, no faces
		return false;
	}

	out.verts.swap( welded.verts );
	out.mins = dvec3( 1e300, 1e300, 1e300 );
	out.maxs = dvec3( -1e300, -1e300, -1e300 );
	for ( size_t i = 0; i < out.verts.size(); i++ )
	{
		out.mins = DMin( out.mins, out.verts[i] );
		out.maxs = DMax( out.maxs, out.verts[i] );
	}
	return true;
}

//-----------------------------------------------------------------------------
// Verification against the exact sum point set
//-----------------------------------------------------------------------------

// Min-norm point of the affine hull of q[idx[0..k-1]] (k <= 4), as barycentric
// weights. False if the subset is degenerate or the point lies outside it.
static bool SubsetMinNorm( const dvec3 *q, const int *idx, int k, double *lambda )
{
	if ( k == 1 )
	{
		lambda[0] = 1.0;
		return true;
	}
	dvec3 e[3];
	for ( int i = 1; i < k; i++ )
		e[i - 1] = q[idx[i]] - q[idx[0]];
	int m = k - 1;
	double A[3][4];
	for ( int i = 0; i < m; i++ )
	{
		for ( int j = 0; j < m; j++ )
			A[i][j] = DDot( e[i], e[j] );
		A[i][m] = -DDot( q[idx[0]], e[i] );
	}
	double scale = 0;
	for ( int i = 0; i < m; i++ )
		scale = MAX( scale, A[i][i] );
	// Gaussian elimination with partial pivoting
	for ( int c = 0; c < m; c++ )
	{
		int piv = c;
		for ( int r = c + 1; r < m; r++ )
			if ( fabs( A[r][c] ) > fabs( A[piv][c] ) )
				piv = r;
		if ( fabs( A[piv][c] ) <= 1e-14 * scale )
			return false;
		for ( int j = 0; j <= m; j++ )
		{
			double t = A[c][j];
			A[c][j] = A[piv][j];
			A[piv][j] = t;
		}
		for ( int r = 0; r < m; r++ )
		{
			if ( r == c )
				continue;
			double f = A[r][c] / A[c][c];
			for ( int j = c; j <= m; j++ )
				A[r][j] -= f * A[c][j];
		}
	}
	double sumMu = 0;
	for ( int i = 0; i < m; i++ )
	{
		lambda[i + 1] = A[i][m] / A[i][i];
		sumMu += lambda[i + 1];
	}
	lambda[0] = 1.0 - sumMu;
	for ( int i = 0; i < k; i++ )
		if ( lambda[i] < 0 )
			return false;
	return true;
}

// Distance from p to the hull of pts (GJK with a brute-force subsimplex
// solver; no hull is built, so slivers cannot fool it). Stops early once the
// answer is known to be <= tol or > tol.
static double DistanceToHull( const std::vector<dvec3> &pts, const dvec3 &p, double tol )
{
	std::vector<dvec3> q( pts.size() );
	for ( size_t i = 0; i < pts.size(); i++ )
		q[i] = pts[i] - p;

	int simplex[4];
	int k = 0;
	size_t first = 0;
	for ( size_t i = 1; i < q.size(); i++ )
		if ( DDot( q[i], q[i] ) < DDot( q[first], q[first] ) )
			first = i;
	simplex[k++] = (int)first;
	dvec3 x = q[first];

	for ( int iter = 0; iter < 64; iter++ )
	{
		double xx = DDot( x, x );
		double upper = sqrt( xx );
		if ( upper <= tol )
			return upper;
		// support in -x: the lower bound on the distance along x / |x|
		size_t best = 0;
		double bestDot = 1e300;
		for ( size_t i = 0; i < q.size(); i++ )
		{
			double d = DDot( x, q[i] );
			if ( d < bestDot )
			{
				bestDot = d;
				best = i;
			}
		}
		// past here an exit returns the upper bound: doubt fails, never passes
		double lower = bestDot / upper;
		if ( lower > tol )
			return lower;
		if ( upper - lower <= 1e-12 * ( upper + 1.0 ) )
			return upper;
		bool inSimplex = false;
		for ( int i = 0; i < k; i++ )
			inSimplex = inSimplex || simplex[i] == (int)best;
		if ( inSimplex )
			return upper;
		simplex[k++] = (int)best;

		// min-norm point over every non-empty subset of the simplex
		double bestNorm = 1e300;
		int bestSub[4], bestK = 0;
		double bestLambda[4];
		for ( int mask = 1; mask < ( 1 << k ); mask++ )
		{
			int sub[4], sk = 0;
			for ( int i = 0; i < k; i++ )
				if ( mask & ( 1 << i ) )
					sub[sk++] = simplex[i];
			double lambda[4];
			if ( !SubsetMinNorm( q.data(), sub, sk, lambda ) )
				continue;
			dvec3 y( 0, 0, 0 );
			for ( int i = 0; i < sk; i++ )
				y += q[sub[i]] * lambda[i];
			double n = DDot( y, y );
			if ( n < bestNorm )
			{
				bestNorm = n;
				bestK = sk;
				for ( int i = 0; i < sk; i++ )
				{
					bestSub[i] = sub[i];
					bestLambda[i] = lambda[i];
				}
			}
		}
		if ( bestK == 4 )
			return 0.0;		// p inside the hull
		if ( bestNorm >= xx )
			return upper;	// no progress
		k = 0;
		x = dvec3( 0, 0, 0 );
		for ( int i = 0; i < bestK; i++ )
		{
			if ( bestLambda[i] <= 0 )
				continue;
			simplex[k++] = bestSub[i];
			x += q[bestSub[i]] * bestLambda[i];
		}
	}
	return sqrt( DDot( x, x ) );	// upper bound
}

bool VerifyGrown( const ConvexShape &src, const PlayerHull &hull, const GrownPolytope &poly,
				  char *pszWhy, int whyLen )
{
	if ( poly.planes.size() < 4 || poly.verts.size() < 4 || poly.faces.size() < 4 )
	{
		Q_snprintf( pszWhy, whyLen, "empty or open polytope (%d planes, %d verts)",
					(int)poly.planes.size(), (int)poly.verts.size() );
		return false;
	}

	dvec3 corners[8];
	hull.ReflectedCorners( corners );
	std::vector<dvec3> sum;
	sum.reserve( src.verts.size() * 8 );
	for ( size_t i = 0; i < src.verts.size(); i++ )
		for ( int c = 0; c < 8; c++ )
			sum.push_back( src.verts[i] + corners[c] );

	// every plane supports the sum: nothing outside, something on it
	for ( size_t p = 0; p < poly.planes.size(); p++ )
	{
		double m = -1e300;
		for ( size_t i = 0; i < sum.size(); i++ )
		{
			double dd = DDot( poly.planes[p].n, sum[i] );
			if ( dd > m )
				m = dd;
		}
		if ( fabs( m - poly.planes[p].d ) > 1e-6 )
		{
			Q_snprintf( pszWhy, whyLen, "plane %d (%.4f %.4f %.4f, %.4f) misses the support by %g",
						(int)p, poly.planes[p].n.x, poly.planes[p].n.y, poly.planes[p].n.z, poly.planes[p].d,
						m - poly.planes[p].d );
			return false;
		}
	}

	// every polytope vertex is a sum point (nothing bulges beyond the hull)
	for ( size_t v = 0; v < poly.verts.size(); v++ )
	{
		double best = 1e300;
		for ( size_t i = 0; i < sum.size(); i++ )
		{
			dvec3 d = sum[i] - poly.verts[v];
			double l = DDot( d, d );
			if ( l < best )
				best = l;
		}
		if ( sqrt( best ) > 1e-4 )
		{
			Q_snprintf( pszWhy, whyLen, "vertex (%.4f %.4f %.4f) is %g from every sum point",
						poly.verts[v].x, poly.verts[v].y, poly.verts[v].z, sqrt( best ) );
			return false;
		}
	}

	// The walker uses the planes, so bound what the planes enclose: clip every
	// plane by all the others and require each corner of that intersection to
	// lie within overTol of the hull of the sum points. The intersection is the
	// hull of its corners, so that bounds its overgrowth, and a missing facet
	// leaves a corner (or an open side out to the base winding) well outside.
	// Corners need not be sum points: near-coplanar support planes from
	// float-tilted source faces cross mid-face, and a facet normal taken from
	// a candidate within the support tolerance tilts its plane slightly. The
	// tolerance is the rounding of the BSP's own float coordinates (2^-24
	// relative, half an ulp); fightspace3's worst is 2.3e-4u at |coord| 13316
	// against 8e-4u allowed there, and a missing facet is units off.
	double scale = 1.0;
	for ( size_t i = 0; i < src.verts.size(); i++ )
		for ( int k = 0; k < 3; k++ )
			scale = MAX( scale, fabs( src.verts[i][k] ) );
	double overTol = 6e-8 * scale + 1e-5;
	DWinding w;
	for ( size_t p = 0; p < poly.planes.size(); p++ )
	{
		DBaseWindingForPlane( poly.planes[p].n, poly.planes[p].d, BASE_WINDING_HALFSIZE, w );
		for ( size_t j = 0; j < poly.planes.size() && !w.empty(); j++ )
		{
			if ( j != p )
				DChopWindingInPlace( w, -poly.planes[j].n, -poly.planes[j].d, CHOP_EPSILON );
		}
		for ( size_t k = 0; k < w.size(); k++ )
		{
			double dist = DistanceToHull( sum, w[k], overTol );
			if ( dist > overTol )
			{
				Q_snprintf( pszWhy, whyLen, "plane %d (%.4f %.4f %.4f) has corner (%.4f %.4f %.4f) %g outside the sum hull (tol %g)",
							(int)p, poly.planes[p].n.x, poly.planes[p].n.y, poly.planes[p].n.z, w[k].x, w[k].y, w[k].z,
							dist, overTol );
				return false;
			}
		}
	}

	double vol = PolytopeVolume( poly.verts, poly.faces );
	if ( !( vol > 0 ) )
	{
		Q_snprintf( pszWhy, whyLen, "non-positive volume %g", vol );
		return false;
	}
	return true;
}
