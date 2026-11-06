//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: special movement (#13 phase B): water volumes, ladders, trigger_push
//          / trigger_catapult / trigger_teleport, and mover rides, as flood
//          vertices beyond the walk nodes joined to them by directed edges. The
//          launchers they carry (velocity-driven arcs) run in the arc rounds
//          (bw_arcs.cpp). See "How #13 special movement works" in the doc.
//
//          Every test errs toward joining (touch epsilons are added to the
//          planes, obstacles that hide an intersection are shrunk), so a
//          rounding error can only over-reach.
//
//=============================================================================//

#include <algorithm>
#include <map>
#include <set>
#include "cmdlib.h"
#include "bsplib.h"
#include "tier0/platform.h"
#include "bw_walk.h"
#include "bw_arcs.h"
#include "bw_util.h"
#include "bw_hash.h"

// Touch tests: polygon against polytope planes moved out by this much.
static const double TOUCH_EPS = 1e-3;
// func_useableladder: a feet point within this of the ladder segment mounts it
// (gamemovement.cpp FindLadder, 64u); info_ladder_dismount points within this
// of it are its dismounts (func_ladder.cpp, 100u around the ends, rechecked
// along the climb).
static const double LADDER_MOUNT = 64.0;
static const double LADDER_DISMOUNT = 100.0;
// Launch samples along a ladder, and the cap of samples on a ladder brush.
static const double LADDER_SAMPLE = 16.0;
static const int LADDER_BRUSH_MAX_SAMPLES = 2048;
// A brush ladder is climbed from within 2u of its brush (gamemovement.cpp
// LadderMove traces 2u ahead).
static const double LADDER_BRUSH_RANGE = 2.0;
// useable ladder jump: 200 u/s along the view plus 50 up (MAX_CLIMB_SPEED);
// a brush ladder pushes off at 270 u/s along its normal
static const double LADDER_JUMP_VZ = 250.0;
static const double LADDER_JUMP_SPEED = 270.0;
// Points sampled inside a trigger volume as arc targets (a trigger crossed in
// flight): grid spacing and cap per volume. A push's points are its point
// entries (one flood vertex and launcher each), so they are sparser.
static const double TRIGGER_TARGET_SPACING = 32.0;
static const int TRIGGER_TARGET_CAP = 256;
static const double PUSH_TARGET_SPACING = 64.0;
static const int PUSH_TARGET_CAP = 64;
// Launch points of an entry polygon, and of a push's water / ladder overlap.
static const int PUSH_ENTRY_MAX_POINTS = 16;
// Node entries of a push: one per cluster of touching nodes (XY cell, height
// band), launching from one point per grid cell of their parts, at most a cap.
static const double PUSH_CLUSTER_XY = 256.0;
static const double PUSH_CLUSTER_Z = 64.0;
static const double PUSH_POINT_GRID = 32.0;
static const int PUSH_CLUSTER_MAX_POINTS = 48;

const char *SpecialKindName( int kind )
{
	static const char *s_Names[SPECIAL_KIND_COUNT] = { "water", "ladder", "ladder_brush", "push", "push_entry",
														 "catapult", "teleport", "arrival", "ride" };
	return ( kind >= 0 && kind < SPECIAL_KIND_COUNT ) ? s_Names[kind] : "?";
}

//-----------------------------------------------------------------------------
// Known answers (-selftest)
//-----------------------------------------------------------------------------
static dvec3 AngleForward( double pitchDeg, double yawDeg )
{
	const double d2r = 3.14159265358979323846 / 180.0;
	double sp = sin( pitchDeg * d2r ), cp = cos( pitchDeg * d2r );
	double sy = sin( yawDeg * d2r ), cy = cos( yawDeg * d2r );
	return dvec3( cp * cy, cp * sy, -sp );
}

dvec3 PushVelocity( const char *pszPushDir, double speed )
{
	double p = 0, y = 0, r = 0;
	if ( pszPushDir )
		sscanf( pszPushDir, "%lf %lf %lf", &p, &y, &r );
	if ( speed == 0.0 )
		speed = 100.0;
	return AngleForward( p, y ) * speed;
}

dvec3 TrackTrainTranslation( const dvec3 &trainOrigin, const dvec3 &pathOrigin, double height )
{
	return pathOrigin + dvec3( 0, 0, height ) - trainOrigin;
}

dvec3 FuncTrainTranslation( const dvec3 &trainOrigin, const dvec3 &modelCenterLocal, const dvec3 &corner )
{
	return corner - modelCenterLocal - trainOrigin;
}

//-----------------------------------------------------------------------------
// Geometry
//-----------------------------------------------------------------------------
static dvec3 ClosestOnSegment( const dvec3 &a, const dvec3 &b, const dvec3 &p )
{
	dvec3 e = b - a;
	double ll = DDot( e, e );
	double t = ll > 0 ? DDot( p - a, e ) / ll : 0.0;
	t = MAX( 0.0, MIN( 1.0, t ) );
	return a + e * t;
}

// Distance between segments p1-q1 and p2-q2 (Ericson, Real-Time Collision Detection 5.1.9).
static double SegSegDistance( const dvec3 &p1, const dvec3 &q1, const dvec3 &p2, const dvec3 &q2 )
{
	dvec3 d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
	double a = DDot( d1, d1 ), e = DDot( d2, d2 ), f = DDot( d2, r );
	double s, t;
	if ( a <= 1e-24 && e <= 1e-24 )
		return DLength( r );
	if ( a <= 1e-24 )
	{
		s = 0;
		t = MAX( 0.0, MIN( 1.0, f / e ) );
	}
	else
	{
		double c = DDot( d1, r );
		if ( e <= 1e-24 )
		{
			t = 0;
			s = MAX( 0.0, MIN( 1.0, -c / a ) );
		}
		else
		{
			double b = DDot( d1, d2 ), denom = a * e - b * b;
			s = denom > 1e-24 ? MAX( 0.0, MIN( 1.0, ( b * f - c * e ) / denom ) ) : 0.0;
			t = ( b * s + f ) / e;
			if ( t < 0 )
			{
				t = 0;
				s = MAX( 0.0, MIN( 1.0, -c / a ) );
			}
			else if ( t > 1 )
			{
				t = 1;
				s = MAX( 0.0, MIN( 1.0, ( b - c ) / a ) );
			}
		}
	}
	return DLength( ( p1 + d1 * s ) - ( p2 + d2 * t ) );
}

// Unit normal of a planar polygon (Newell); false for a degenerate one.
static bool PolyNormal( const DWinding &w, dvec3 &n )
{
	n = dvec3( 0, 0, 0 );
	for ( size_t i = 0; i < w.size(); i++ )
	{
		const dvec3 &a = w[i], &b = w[( i + 1 ) % w.size()];
		n.x += ( a.y - b.y ) * ( a.z + b.z );
		n.y += ( a.z - b.z ) * ( a.x + b.x );
		n.z += ( a.x - b.x ) * ( a.y + b.y );
	}
	return DNormalize( n ) > 1e-9;
}

// q (on the polygon's plane) inside the convex polygon, edges included.
static bool InsidePolygon( const DWinding &w, const dvec3 &n, const dvec3 &q )
{
	for ( size_t i = 0; i < w.size(); i++ )
	{
		const dvec3 &a = w[i], &b = w[( i + 1 ) % w.size()];
		dvec3 e = b - a;
		double len = DLength( e );
		if ( DDot( DCross( e, q - a ), n ) < -1e-9 * MAX( 1.0, len ) )
			return false;
	}
	return true;
}

dvec3 ClosestPointOnPolygon( const DWinding &w, const dvec3 &p )
{
	dvec3 n;
	if ( w.size() >= 3 && PolyNormal( w, n ) )
	{
		dvec3 q = p - n * DDot( n, p - w[0] );
		if ( InsidePolygon( w, n, q ) )
			return q;
	}
	dvec3 best = w.empty() ? p : w[0];
	double bd = 1e300;
	for ( size_t i = 0; i < w.size(); i++ )
	{
		dvec3 c = ClosestOnSegment( w[i], w[( i + 1 ) % w.size()], p );
		double d = DDot( c - p, c - p );
		if ( d < bd )
		{
			bd = d;
			best = c;
		}
	}
	return best;
}

double SegmentPolygonDistance( const dvec3 &a, const dvec3 &b, const DWinding &poly )
{
	if ( poly.empty() )
		return 1e300;
	dvec3 n;
	if ( poly.size() >= 3 && PolyNormal( poly, n ) )
	{
		double da = DDot( n, a - poly[0] ), db = DDot( n, b - poly[0] );
		if ( ( da <= 0 && db >= 0 ) || ( da >= 0 && db <= 0 ) )
		{
			if ( da != db )
			{
				dvec3 x = a + ( b - a ) * ( da / ( da - db ) );
				if ( InsidePolygon( poly, n, x ) )
					return 0.0;
			}
		}
	}
	double best = MIN( DLength( a - ClosestPointOnPolygon( poly, a ) ), DLength( b - ClosestPointOnPolygon( poly, b ) ) );
	for ( size_t i = 0; i < poly.size(); i++ )
		best = MIN( best, SegSegDistance( a, b, poly[i], poly[( i + 1 ) % poly.size()] ) );
	return best;
}

// Distance between two convex planar polygons (either may be degenerate).
static double PolyPolyDistance( const DWinding &A, const DWinding &B )
{
	double best = 1e300;
	for ( size_t i = 0; i < A.size() && best > 0; i++ )
		best = MIN( best, SegmentPolygonDistance( A[i], A[( i + 1 ) % A.size()], B ) );
	for ( size_t i = 0; i < B.size() && best > 0; i++ )
		best = MIN( best, SegmentPolygonDistance( B[i], B[( i + 1 ) % B.size()], A ) );
	return best;
}

// Keeps the part of w with n . x <= d.
static void ClipKeep( const DWinding &in, const dvec3 &n, double d, DWinding &out )
{
	out.clear();
	size_t k = in.size();
	if ( k == 1 )
	{
		if ( DDot( n, in[0] ) - d <= 0 )
			out.push_back( in[0] );
		return;
	}
	for ( size_t i = 0; i < k; i++ )
	{
		const dvec3 &p = in[i], &q = in[( i + 1 ) % k];
		double sp = DDot( n, p ) - d, sq = DDot( n, q ) - d;
		if ( sp <= 0 )
			out.push_back( p );
		if ( ( sp <= 0 ) != ( sq <= 0 ) )
			out.push_back( p + ( q - p ) * ( sp / ( sp - sq ) ) );
	}
}

static bool BoxesMeet( const dvec3 &amin, const dvec3 &amax, const dvec3 &bmin, const dvec3 &bmax, double eps )
{
	return amin.x <= bmax.x + eps && bmin.x <= amax.x + eps && amin.y <= bmax.y + eps && bmin.y <= amax.y + eps &&
		   amin.z <= bmax.z + eps && bmin.z <= amax.z + eps;
}

// The part of the polygon within the polytope (planes moved out by eps).
static void ClipToPolytope( const DWinding &poly, const GrownPolytope &q, double eps, DWinding &out )
{
	DWinding tmp;
	out = poly;
	for ( size_t k = 0; k < q.planes.size() && !out.empty(); k++ )
	{
		ClipKeep( out, q.planes[k].n, q.planes[k].d + eps, tmp );
		out.swap( tmp );
	}
}

bool PolygonTouchesPolytope( const DWinding &poly, const GrownPolytope &q, double eps )
{
	if ( poly.empty() || !q.IsValid() )
		return false;
	dvec3 mins = poly[0], maxs = poly[0];
	for ( size_t i = 1; i < poly.size(); i++ )
	{
		mins = DMin( mins, poly[i] );
		maxs = DMax( maxs, poly[i] );
	}
	if ( !BoxesMeet( mins, maxs, q.mins, q.maxs, eps ) )
		return false;
	DWinding w;
	ClipToPolytope( poly, q, eps, w );
	return !w.empty();
}

static bool PointInPolytope( const GrownPolytope &q, const dvec3 &p, double eps )
{
	for ( size_t k = 0; k < q.planes.size(); k++ )
	{
		if ( DDot( q.planes[k].n, p ) > q.planes[k].d + eps )
			return false;
	}
	return q.IsValid();
}

static void FacePolygon( const GrownPolytope &q, size_t f, DWinding &w )
{
	w.clear();
	for ( size_t k = 0; k < q.faces[f].size(); k++ )
		w.push_back( q.verts[q.faces[f][k]] );
}

// Do the polytopes meet (planes moved out by eps)? With pts, the vertices of
// their intersection: every face of each clipped by the other, plus the
// vertices of each inside the other.
static bool PolytopesIntersect( const GrownPolytope &A, const GrownPolytope &B, double eps, std::vector<dvec3> *pts )
{
	if ( !A.IsValid() || !B.IsValid() || !BoxesMeet( A.mins, A.maxs, B.mins, B.maxs, eps ) )
		return false;
	bool any = false;
	DWinding face, clipped;
	for ( int pass = 0; pass < 2; pass++ )
	{
		const GrownPolytope &P = pass ? B : A, &Q = pass ? A : B;
		for ( size_t f = 0; f < P.faces.size(); f++ )
		{
			FacePolygon( P, f, face );
			ClipToPolytope( face, Q, eps, clipped );
			if ( clipped.empty() )
				continue;
			any = true;
			if ( !pts )
				return true;
			pts->insert( pts->end(), clipped.begin(), clipped.end() );
		}
	}
	if ( !any )
	{
		// no face meets the other: one inside the other, or apart
		if ( PointInPolytope( B, A.verts[0], eps ) )
		{
			if ( pts )
				pts->insert( pts->end(), A.verts.begin(), A.verts.end() );
			return true;
		}
		if ( PointInPolytope( A, B.verts[0], eps ) )
		{
			if ( pts )
				pts->insert( pts->end(), B.verts.begin(), B.verts.end() );
			return true;
		}
	}
	return any;
}

// Is every point strictly inside one obstacle, shrunk by WALK_SHRINK?
static bool InsideOneObstacle( const ObstacleSet &obs, const std::vector<dvec3> &pts )
{
	if ( pts.empty() )
		return false;
	dvec3 mins = pts[0], maxs = pts[0];
	for ( size_t i = 1; i < pts.size(); i++ )
	{
		mins = DMin( mins, pts[i] );
		maxs = DMax( maxs, pts[i] );
	}
	std::vector<int> cand;
	obs.hash.Query( mins.x, mins.y, maxs.x, maxs.y, cand );
	for ( size_t c = 0; c < cand.size(); c++ )
	{
		const GrownPolytope &q = ( *obs.polys )[cand[c]];
		if ( !BoxesMeet( mins, maxs, q.mins, q.maxs, 0 ) )
			continue;
		bool all = true;
		for ( size_t i = 0; i < pts.size() && all; i++ )
		{
			for ( size_t k = 0; k < q.planes.size() && all; k++ )
				all = DDot( q.planes[k].n, pts[i] ) < q.planes[k].d - WALK_SHRINK;
		}
		if ( all )
			return true;
	}
	return false;
}

// A convex shape as a polytope (planes through its own vertices).
static bool ShapeToPolytope( const ConvexShape &s, GrownPolytope &out )
{
	out = GrownPolytope();
	if ( !s.IsValid() || s.flat )
		return false;
	out.verts = s.verts;
	out.faces = s.faces;
	for ( size_t f = 0; f < s.faces.size(); f++ )
	{
		DPlane pl;
		pl.n = s.faceNormals[f];
		pl.d = -1e300;
		for ( size_t k = 0; k < s.faces[f].size(); k++ )
			pl.d = MAX( pl.d, DDot( pl.n, s.verts[s.faces[f][k]] ) );	// the outermost vertex: never shrinks
		out.planes.push_back( pl );
	}
	out.mins = s.mins;
	out.maxs = s.maxs;
	return out.IsValid();
}

// The hull of a point set as a polytope.
static bool HullPolytope( const std::vector<dvec3> &pts, GrownPolytope &out )
{
	ConvexShape s;
	if ( !ConvexFromPoints( pts.data(), (int)pts.size(), s ) )
		return false;
	return ShapeToPolytope( s, out );
}

// v swept down by J: the feet points from which a jump of J reaches v.
static bool ExtendDown( const GrownPolytope &v, double J, GrownPolytope &out )
{
	std::vector<dvec3> pts( v.verts );
	for ( size_t i = 0; i < v.verts.size(); i++ )
		pts.push_back( v.verts[i] - dvec3( 0, 0, J ) );
	return HullPolytope( pts, out );
}

static void TranslatePolytope( GrownPolytope &q, const dvec3 &t )
{
	for ( size_t i = 0; i < q.verts.size(); i++ )
		q.verts[i] += t;
	for ( size_t k = 0; k < q.planes.size(); k++ )
		q.planes[k].d += DDot( q.planes[k].n, t );
	q.mins += t;
	q.maxs += t;
}

static dvec3 PointsCentroid( const std::vector<dvec3> &pts )
{
	dvec3 c( 0, 0, 0 );
	for ( size_t i = 0; i < pts.size(); i++ )
		c += pts[i];
	return pts.empty() ? c : c * ( 1.0 / pts.size() );
}

// Grid points inside the polytope, about 'spacing' apart (coarser until at most
// cap), plus its vertex centroid.
static void SamplePolytope( const GrownPolytope &q, double spacing, int cap, std::vector<dvec3> &out )
{
	out.clear();
	if ( !q.IsValid() )
		return;
	dvec3 size = q.maxs - q.mins;
	std::vector<dvec3> pts;
	for ( int iter = 0; iter < 32; iter++, spacing *= 1.26 )
	{
		int n[3];
		for ( int a = 0; a < 3; a++ )
			n[a] = MAX( 1, (int)floor( size[a] / spacing ) );
		pts.clear();
		if ( (double)n[0] * n[1] * n[2] <= 64.0 * cap )
		{
			for ( int i = 0; i < n[0]; i++ )
				for ( int j = 0; j < n[1]; j++ )
					for ( int k = 0; k < n[2]; k++ )
					{
						dvec3 p( q.mins.x + size.x * ( i + 0.5 ) / n[0], q.mins.y + size.y * ( j + 0.5 ) / n[1],
								 q.mins.z + size.z * ( k + 0.5 ) / n[2] );
						if ( PointInPolytope( q, p, 0.0 ) )
							pts.push_back( p );
					}
		}
		else
			continue;
		if ( (int)pts.size() <= cap )
			break;
	}
	if ( (int)pts.size() > cap )
	{
		// spread subset, not the low-x slab of the grid
		std::vector<dvec3> thin;
		thin.reserve( cap );
		for ( int i = 0; i < cap; i++ )
			thin.push_back( pts[(size_t)( (double)i * pts.size() / cap )] );
		pts.swap( thin );
	}
	out.swap( pts );
	out.push_back( PointsCentroid( q.verts ) );
}

// At most maxPts of the points (spread over the list) plus their centroid.
static void ThinPoints( const std::vector<dvec3> &in, int maxPts, std::vector<dvec3> &out )
{
	out.clear();
	if ( in.empty() )
		return;
	int n = (int)in.size();
	int step = MAX( 1, ( n + maxPts - 1 ) / maxPts );
	for ( int i = 0; i < n; i += step )
		out.push_back( in[i] );
	out.push_back( PointsCentroid( in ) );
}

// The part of segment a-b inside the polytope (planes moved out by eps). False if none.
static bool ClipSegment( const dvec3 &a, const dvec3 &b, const GrownPolytope &q, double eps, dvec3 &c0, dvec3 &c1 )
{
	if ( !q.IsValid() )
		return false;
	double t0 = 0, t1 = 1;
	dvec3 d = b - a;
	for ( size_t k = 0; k < q.planes.size(); k++ )
	{
		double s = DDot( q.planes[k].n, a ) - ( q.planes[k].d + eps ), v = DDot( q.planes[k].n, d );
		if ( fabs( v ) < 1e-15 )
		{
			if ( s > 0 )
				return false;
			continue;
		}
		double t = -s / v;
		if ( v > 0 )
			t1 = MIN( t1, t );
		else
			t0 = MAX( t0, t );
		if ( t0 > t1 )
			return false;
	}
	c0 = a + d * t0;
	c1 = a + d * t1;
	return true;
}

//-----------------------------------------------------------------------------
// Builder
//-----------------------------------------------------------------------------
struct SpecialBuilder
{
	const CollisionWorld &world;
	const PlayerProfile &profile;
	WalkGraph &graph;
	SpecialGraph &sg;
	int nNodes;
	CXYHash nodeHash;
	ObstacleSet crouchObs;
	std::vector< std::vector<GrownPolytope> > touch;	// per vertex: the trigger touch shapes / ladder reach
	std::vector<int> waterVerts, ladderVerts, ladderBrushVerts, triggerVerts;
	std::vector<int> cand;
	// per push vertex: its launch families and the top of its volumes
	struct PushInfo
	{
		std::vector<LaunchVel> vel;
		double top;
	};
	std::map<int, PushInfo> pushInfo;

	SpecialBuilder( const CollisionWorld &w, const PlayerProfile &p, WalkGraph &g )
		: world( w ), profile( p ), graph( g ), sg( g.special )
	{
		nNodes = (int)graph.nodes.size();
		std::vector<XYBox> boxes( nNodes );
		std::vector<unsigned char> use( nNodes, 1 );
		for ( int i = 0; i < nNodes; i++ )
		{
			const WalkNode &n = graph.nodes[i];
			boxes[i].x0 = n.mins.x;
			boxes[i].y0 = n.mins.y;
			boxes[i].x1 = n.maxs.x;
			boxes[i].y1 = n.maxs.y;
			use[i] = n.poly.size() >= 3;
		}
		nodeHash.Build( boxes, use );
		crouchObs.Build( world, world.crouch );
	}

	int AddVertex( int kind, int entity, const char *pszLabel )
	{
		SpecialVertex v;
		v.kind = kind;
		v.entity = entity;
		v.pose = 0;
		Q_strncpy( v.label, pszLabel ? pszLabel : "", sizeof( v.label ) );
		v.launcher = -1;
		sg.verts.push_back( v );
		touch.push_back( std::vector<GrownPolytope>() );
		sg.stats.count[kind]++;
		return (int)sg.verts.size() - 1;
	}
	int Id( int vertex ) const { return nNodes + vertex; }
	void Edge( int fa, int fb )
	{
		sg.edges.push_back( std::make_pair( fa, fb ) );
		sg.stats.edges++;
	}
	void Both( int fa, int fb )
	{
		Edge( fa, fb );
		Edge( fb, fa );
	}

	// Nodes whose polygon touches q.
	void NodesTouching( const GrownPolytope &q, std::vector<int> &out )
	{
		out.clear();
		if ( !q.IsValid() )
			return;
		cand.clear();
		nodeHash.Query( q.mins.x - TOUCH_EPS, q.mins.y - TOUCH_EPS, q.maxs.x + TOUCH_EPS, q.maxs.y + TOUCH_EPS, cand );
		for ( size_t c = 0; c < cand.size(); c++ )
		{
			const WalkNode &n = graph.nodes[cand[c]];
			if ( n.mins.z > q.maxs.z + TOUCH_EPS || n.maxs.z < q.mins.z - TOUCH_EPS )
				continue;
			if ( PolygonTouchesPolytope( n.poly, q, TOUCH_EPS ) )
				out.push_back( cand[c] );
		}
	}

	// The node a point falls to (teleport destinations, dismounts, catapult targets).
	int MapPoint( const dvec3 &p ) { return MapSeedPoint( p, profile.stepHeight, 1e9 ); }

	LaunchVel Vel( double vz, double lift, double bx, double by, double hmax, bool apex )
	{
		LaunchVel v;
		v.vz = vz;
		v.lift = lift;
		v.bx = bx;
		v.by = by;
		v.hmax = hmax;
		v.apexVz = apex;
		v.push = false;
		v.gIn = profile.gravity;
		return v;
	}

	LaunchVel PushVel( double vz, double lift, double bx, double by, double hmax, double gIn )
	{
		LaunchVel v = Vel( vz, lift, bx, by, hmax, false );
		v.push = true;
		v.gIn = gIn;
		return v;
	}

	// The longest flight time and the highest point of a push family leaving at
	// height sz, landing no lower than floorZ: all the way under gIn, or up to the
	// top (or the apex under gIn) and then ballistic (SolvePushArc). Exiting
	// lower than that never flies longer: a lifting push gives less speed, a
	// weaker gravity inside is left sooner.
	void PushFlightBound( const LaunchVel &f, double sz, double top, double floorZ, double &T, double &apex )
	{
		const double g = profile.gravity, gIn = f.gIn, vz = f.vz, vzUp = MAX( 0.0, vz );
		T = 0.0;
		apex = sz;
		if ( gIn > 1e-9 )
		{
			T = ( vz + sqrt( vz * vz + 2.0 * gIn * MAX( 0.0, sz - floorZ ) ) ) / gIn;
			apex = sz + vzUp * vzUp / ( 2.0 * gIn );
		}
		double zx = top;
		if ( gIn > 1e-9 )
			zx = MIN( zx, sz + vzUp * vzUp / ( 2.0 * gIn ) );
		if ( zx > sz + 1e-6 && ( fabs( gIn ) > 1e-9 || vz > 1e-9 ) )	// else he never rises (SolvePushArc)
		{
			double vzX = sqrt( MAX( 0.0, vz * vz - 2.0 * gIn * ( zx - sz ) ) );
			double t0 = fabs( gIn ) > 1e-9 ? ( vz - vzX ) / gIn : ( zx - sz ) / MAX( vz, 1e-9 );
			double T1 = ( vzX + sqrt( vzX * vzX + 2.0 * g * MAX( 0.0, zx - floorZ ) ) ) / g;
			T = MAX( T, t0 + T1 );
			apex = MAX( apex, zx + vzX * vzX / ( 2.0 * g ) );
		}
	}

	// The launch box, the XY radius of the farthest arc a family can fly (from
	// the top of the box, plus lift, at its vz, down arc_max_drop below the box)
	// and the highest point any flight reaches.
	void FinishLauncher( SpecialLauncher &L )
	{
		L.mins = dvec3( 1e300, 1e300, 1e300 );
		L.maxs = dvec3( -1e300, -1e300, -1e300 );
		for ( size_t i = 0; i < L.points.size(); i++ )
		{
			L.mins = DMin( L.mins, L.points[i] );
			L.maxs = DMax( L.maxs, L.points[i] );
		}
		for ( size_t i = 0; i < L.volumes.size(); i++ )
		{
			L.mins = DMin( L.mins, L.volumes[i].mins );
			L.maxs = DMax( L.maxs, L.volumes[i].maxs );
		}
		const double g = profile.gravity, floorZ = L.mins.z - profile.arcMaxDrop;
		L.reach = 0;
		L.zTop = L.maxs.z;
		L.sweepBx = L.vel.empty() ? 0.0 : L.vel[0].bx;
		L.sweepBy = L.vel.empty() ? 0.0 : L.vel[0].by;
		L.sweepT = 0.0;
		L.sweepOwn = 0.0;
		bool sweep = L.sweepBx != 0.0 || L.sweepBy != 0.0;
		for ( size_t f = 0; f < L.vel.size(); f++ )
		{
			const LaunchVel &v = L.vel[f];
			double T = 0.0, apex = 0.0;
			if ( v.push )
			{
				for ( int e = 0; e < 2; e++ )
				{
					double t, a;
					PushFlightBound( v, ( e ? L.maxs.z : L.mins.z ) + v.lift, L.pushTop, floorZ, t, a );
					T = MAX( T, t );
					apex = MAX( apex, a );
				}
			}
			else
			{
				double fall = v.lift + ( L.maxs.z - L.mins.z ) + profile.arcMaxDrop;
				T = ( v.vz + sqrt( v.vz * v.vz + 2.0 * g * MAX( 0.0, fall ) ) ) / g;
				apex = L.maxs.z + v.lift + ( v.vz > 0 ? v.vz * v.vz / ( 2.0 * g ) : 0.0 );
			}
			double speed = sqrt( v.bx * v.bx + v.by * v.by ) + v.hmax;
			L.reach = MAX( L.reach, speed * T + 2.0 * profile.halfWidth );
			L.zTop = MAX( L.zTop, apex );
			sweep = sweep && v.bx == L.sweepBx && v.by == L.sweepBy;
			L.sweepT = MAX( L.sweepT, T );
			L.sweepOwn = MAX( L.sweepOwn, v.hmax * T + 2.0 * profile.halfWidth );
		}
		// a barely lifting push flies for minutes: no farther than across any map
		L.reach = MIN( L.reach, 262144.0 );
		L.zTop = MIN( L.zTop, 131072.0 );
		if ( !sweep || L.sweepT * ( fabs( L.sweepBx ) + fabs( L.sweepBy ) ) > 262144.0 )
			L.sweepT = 0.0;
	}

	int AddLauncher( int vertex, SpecialLauncher &L )
	{
		L.vertex = vertex;
		if ( L.vel.empty() || !L.vel[0].push )
			L.pushTop = 0.0;
		FinishLauncher( L );
		sg.launchers.push_back( L );
		sg.verts[vertex].launcher = (int)sg.launchers.size() - 1;
		return sg.verts[vertex].launcher;
	}

	double Vmax() const { return profile.maxAirSpeed * profile.reachMargin; }

	//-------------------------------------------------------------------------
	// Water: each CAT_WATER world brush, crouch-grown, is a volume of free 3-D
	// movement. Its top face is also a walk node (a surface node, built with the
	// other up faces), so swimming out, jumping out up to max_jump_rise and
	// falling in are ordinary walk / jump / drop / arc edges.
	//-------------------------------------------------------------------------
	void Water()
	{
		std::vector<int> touching;
		for ( size_t i = 0; i < world.convexes.size(); i++ )
		{
			if ( world.convexes[i].category != CAT_WATER || !world.crouch[i].IsValid() )
				continue;
			int v = AddVertex( SPECIAL_WATER, -1, "water" );
			sg.verts[v].volumes.push_back( world.crouch[i] );
			waterVerts.push_back( v );
			NodesTouching( world.crouch[i], touching );
			for ( size_t k = 0; k < touching.size(); k++ )
				Both( touching[k], Id( v ) );
		}
		// overlapping volumes connect, unless the overlap lies inside an obstacle
		// (two pools either side of a wall overlap inside the wall's grown solid)
		std::vector<dvec3> pts;
		for ( size_t i = 0; i < waterVerts.size(); i++ )
		{
			const GrownPolytope &A = sg.verts[waterVerts[i]].volumes[0];
			for ( size_t j = i + 1; j < waterVerts.size(); j++ )
			{
				const GrownPolytope &B = sg.verts[waterVerts[j]].volumes[0];
				pts.clear();
				if ( PolytopesIntersect( A, B, TOUCH_EPS, &pts ) && !InsideOneObstacle( crouchObs, pts ) )
					Both( Id( waterVerts[i] ), Id( waterVerts[j] ) );
			}
		}
	}

	// Launch families of a climber (ladders): the ladder's own jump-off, at most
	// 270 u/s across (a brush ladder pushes off at 270 along its normal; a
	// useable ladder jumps 200 along the view plus 50 up, so at most 250 up),
	// and the normal arc kinds (walk-off, jump, duck-jump) at full air speed.
	void ClimbFamilies( SpecialLauncher &L )
	{
		double vmax = Vmax(), vj = profile.jumpVelocity;
		L.vel.push_back( Vel( 0.0, 0.0, 0, 0, vmax, false ) );
		L.vel.push_back( Vel( LADDER_JUMP_VZ, 0.0, 0, 0, LADDER_JUMP_SPEED, true ) );
		L.vel.push_back( Vel( vj, profile.crouchLift, 0, 0, vmax, true ) );
		L.vel.push_back( Vel( vj, 0.0, 0, 0, vmax, true ) );
	}

	//-------------------------------------------------------------------------
	// func_useableladder: a segment between its two mount points (point0 /
	// point1, absolute; the lower one is the bottom). A node mounts it when a
	// feet point of it, raised by at most max_jump_rise, comes within 64u of the
	// segment; the climber leaves at any node it could mount from, at its
	// dismounts, or by an arc from a sample every 16u.
	//-------------------------------------------------------------------------
	void Ladders()
	{
		const double J = profile.maxJumpRise;
		for ( int e = 0; e < num_entities; e++ )
		{
			entity_t *ent = &entities[e];
			if ( Q_stricmp( EntValue( ent, "classname" ), "func_useableladder" ) )
				continue;
			if ( atoi( EntValue( ent, "spawnflags" ) ) & 1 )
			{
				sg.stats.skippedFakeLadders++;	// SF_LADDER_DONTGETON: a prop ladder
				continue;
			}
			Vector p0( 0, 0, 0 ), p1( 0, 0, 0 );
			GetVectorForKey( ent, (char *)"point0", p0 );
			GetVectorForKey( ent, (char *)"point1", p1 );
			dvec3 top( p0 ), bottom( p1 );
			if ( bottom.z > top.z )
				std::swap( top, bottom );	// CFuncLadder::SetEndPoints
			const char *name = EntValue( ent, "targetname" );
			int v = AddVertex( SPECIAL_LADDER, e, name[0] ? name : "func_useableladder" );
			sg.verts[v].a = bottom;
			sg.verts[v].b = top;
			ladderVerts.push_back( v );

			// mounts: the segment swept down by J, within 64u of the node
			DWinding para;
			para.push_back( bottom );
			para.push_back( top );
			para.push_back( top - dvec3( 0, 0, J ) );
			para.push_back( bottom - dvec3( 0, 0, J ) );
			dvec3 lo = DMin( bottom, top ) - dvec3( LADDER_MOUNT, LADDER_MOUNT, LADDER_MOUNT + J );
			dvec3 hi = DMax( bottom, top ) + dvec3( LADDER_MOUNT, LADDER_MOUNT, LADDER_MOUNT );
			cand.clear();
			nodeHash.Query( lo.x, lo.y, hi.x, hi.y, cand );
			for ( size_t c = 0; c < cand.size(); c++ )
			{
				const WalkNode &n = graph.nodes[cand[c]];
				if ( !BoxesMeet( n.mins, n.maxs, lo, hi, 0 ) )
					continue;
				if ( PolyPolyDistance( n.poly, para ) <= LADDER_MOUNT )
					Both( cand[c], Id( v ) );
			}
			// dismounts: info_ladder_dismount near the climb, untargeted or targeting this ladder
			for ( int d = 0; d < num_entities; d++ )
			{
				entity_t *de = &entities[d];
				if ( Q_stricmp( EntValue( de, "classname" ), "info_ladder_dismount" ) )
					continue;
				const char *tgt = EntValue( de, "target" );
				if ( tgt[0] && !EntNameMatches( tgt, name ) )
					continue;
				dvec3 org = EntOrigin( de );
				if ( DLength( org - ClosestOnSegment( bottom, top, org ) ) > LADDER_DISMOUNT )
					continue;
				int node = MapPoint( org );
				if ( node >= 0 )
					Edge( Id( v ), node );
			}
			// jump-off arcs from samples along the climb
			SpecialLauncher L;
			L.hasExitDir = false;
			double len = DLength( top - bottom );
			int steps = MAX( 1, (int)ceil( len / LADDER_SAMPLE ) );
			for ( int k = 0; k <= steps; k++ )
				L.points.push_back( bottom + ( top - bottom ) * ( (double)k / steps ) );
			ClimbFamilies( L );
			AddLauncher( v, L );
		}
	}

	//-------------------------------------------------------------------------
	// CONTENTS_LADDER brushes (func_ladder, moved to the world by vbsp): the
	// brush grown by 2u and then by the stand hull is the climb volume.
	//-------------------------------------------------------------------------
	void LadderBrushes()
	{
		const double J = profile.maxJumpRise, R = LADDER_BRUSH_RANGE;
		std::vector<int> brushes, touching;
		CollectModelBrushes( 0, brushes );
		for ( size_t i = 0; i < brushes.size(); i++ )
		{
			if ( !( dbrushes[brushes[i]].contents & CONTENTS_LADDER ) )
				continue;
			ConvexShape shape, grown;
			if ( !BuildBrushShape( brushes[i], shape ) )
				continue;
			std::vector<dvec3> pts;
			for ( size_t k = 0; k < shape.verts.size(); k++ )
			{
				for ( int c = 0; c < 8; c++ )
					pts.push_back( shape.verts[k] + dvec3( ( c & 1 ) ? R : -R, ( c & 2 ) ? R : -R, ( c & 4 ) ? R : -R ) );
			}
			GrownPolytope vol, reach;
			if ( !ConvexFromPoints( pts.data(), (int)pts.size(), grown ) || !GrowConvex( grown, profile.Stand(), vol ) ||
				 !ExtendDown( vol, J, reach ) )
				continue;
			int v = AddVertex( SPECIAL_LADDER_BRUSH, -1, "ladder_brush" );
			sg.verts[v].volumes.push_back( vol );
			touch[v].push_back( reach );
			ladderBrushVerts.push_back( v );
			NodesTouching( reach, touching );
			for ( size_t k = 0; k < touching.size(); k++ )
				Both( touching[k], Id( v ) );

			// jump-off samples on the volume's side faces, 16u apart
			SpecialLauncher L;
			L.hasExitDir = false;
			DWinding face;
			for ( size_t f = 0; f < vol.faces.size() && (int)L.points.size() < LADDER_BRUSH_MAX_SAMPLES; f++ )
			{
				const dvec3 &n = vol.planes[f].n;
				if ( fabs( n.z ) >= WALK_MIN_NZ )
					continue;
				FacePolygon( vol, f, face );
				dvec3 u = DCross( n, dvec3( 0, 0, 1 ) );
				DNormalize( u );
				dvec3 w = DCross( n, u );
				double u0 = 1e300, u1 = -1e300, w0 = 1e300, w1 = -1e300;
				for ( size_t k = 0; k < face.size(); k++ )
				{
					double su = DDot( face[k] - face[0], u ), sw = DDot( face[k] - face[0], w );
					u0 = MIN( u0, su );
					u1 = MAX( u1, su );
					w0 = MIN( w0, sw );
					w1 = MAX( w1, sw );
				}
				int nu = MAX( 1, (int)ceil( ( u1 - u0 ) / LADDER_SAMPLE ) ), nw = MAX( 1, (int)ceil( ( w1 - w0 ) / LADDER_SAMPLE ) );
				for ( int a = 0; a <= nu && (int)L.points.size() < LADDER_BRUSH_MAX_SAMPLES; a++ )
				{
					for ( int b = 0; b <= nw && (int)L.points.size() < LADDER_BRUSH_MAX_SAMPLES; b++ )
					{
						dvec3 q = face[0] + u * ( u0 + ( u1 - u0 ) * a / nu ) + w * ( w0 + ( w1 - w0 ) * b / nw );
						if ( InsidePolygon( face, n, q ) )
							L.points.push_back( q );
					}
				}
			}
			if ( L.points.empty() )
				continue;
			ClimbFamilies( L );
			AddLauncher( v, L );
		}
	}

	// The model brushes of a brush entity, transformed, grown by the stand hull.
	void EntityVolumes( entity_t *ent, std::vector<GrownPolytope> &vols )
	{
		vols.clear();
		const char *model = EntValue( ent, "model" );
		int m = ( model[0] == '*' ) ? atoi( model + 1 ) : -1;
		if ( m <= 0 || m >= nummodels )
			return;
		DTransform xf;
		BrushEntityTransform( ent, xf );
		std::vector<int> brushes;
		CollectModelBrushes( m, brushes );
		for ( size_t i = 0; i < brushes.size(); i++ )
		{
			ConvexShape shape;
			if ( !BuildBrushShape( brushes[i], shape ) )
				continue;
			TransformShape( shape, xf );
			GrownPolytope g;
			if ( GrowConvex( shape, profile.Stand(), g ) )
				vols.push_back( g );
		}
	}

	// Trigger vertex edges: a node whose feet point, raised by at most a jump,
	// puts the standing box into the trigger fires it. A trigger delivers the
	// player nowhere by itself: a teleport through its arrivals, a catapult and a
	// push through their launch arcs.
	void TriggerNodeEdges( int v )
	{
		std::vector<int> touching;
		for ( size_t k = 0; k < touch[v].size(); k++ )
		{
			NodesTouching( touch[v][k], touching );
			for ( size_t i = 0; i < touching.size(); i++ )
				Edge( touching[i], Id( v ) );
		}
	}

	// Arc targets inside the trigger's volumes: a free arc to one fires it.
	void TriggerTargets( int v )
	{
		std::vector<dvec3> pts;
		const std::vector<GrownPolytope> &vols = sg.verts[v].volumes;
		for ( size_t k = 0; k < vols.size(); k++ )
		{
			SamplePolytope( vols[k], TRIGGER_TARGET_SPACING, TRIGGER_TARGET_CAP, pts );
			for ( size_t i = 0; i < pts.size(); i++ )
			{
				SpecialTarget t = { pts[i], v };
				sg.targets.push_back( t );
			}
		}
	}

	//-------------------------------------------------------------------------
	// trigger_push entries. The push acts only on a player inside it, from the
	// state he enters with, so it is reached through entries, each a flood
	// vertex launching from where the player can be when the push takes him:
	//   node entries  a node touching the push (feet within a jump below it):
	//                 launch points on the node's part under the push;
	//   point entries points inside the push, arc targets (entered in flight);
	//   overlaps      the part of a water volume, ladder or ladder brush inside it.
	// Each entry launches the push's families (SolvePushArc; a push-once is an
	// impulse, plain arcs). The push vertex itself only counts as reached.
	//-------------------------------------------------------------------------
	int PushEntry( int pv, const std::vector<dvec3> &pts )
	{
		const PushInfo &pi = pushInfo[pv];
		int e = AddVertex( SPECIAL_PUSH_ENTRY, sg.verts[pv].entity, sg.verts[pv].label );
		Edge( Id( e ), Id( pv ) );
		SpecialLauncher L;
		L.hasExitDir = false;
		L.points = pts;
		L.vel = pi.vel;
		L.pushTop = pi.top;
		AddLauncher( e, L );
		return e;
	}

	void PushEntries( int pv )
	{
		std::map< int, std::vector<dvec3> > nodePts;
		std::vector<int> touching;
		DWinding piece;
		for ( size_t k = 0; k < touch[pv].size(); k++ )
		{
			NodesTouching( touch[pv][k], touching );
			for ( size_t i = 0; i < touching.size(); i++ )
			{
				const WalkNode &n = graph.nodes[touching[i]];
				ClipToPolytope( n.poly, touch[pv][k], TOUCH_EPS, piece );
				if ( piece.empty() )
					continue;
				// the piece's vertices moved ARC_INSET toward its centroid, and the centroid
				dvec3 c = PointsCentroid( piece );
				std::vector<dvec3> &out = nodePts[touching[i]];
				for ( size_t v = 0; v < piece.size(); v++ )
				{
					dvec3 d = c - piece[v];
					double len = DLength( d );
					dvec3 q = len > 0 ? piece[v] + d * ( MIN( ARC_INSET, 0.5 * len ) / len ) : piece[v];
					out.push_back( dvec3( q.x, q.y, n.Z( q.x, q.y ) ) );
				}
				out.push_back( dvec3( c.x, c.y, n.Z( c.x, c.y ) ) );
			}
		}
		// one entry per cluster of touching nodes (a PUSH_CLUSTER_XY cell, a
		// PUSH_CLUSTER_Z band of the node's lowest point): every node of it joins
		// the entry, which launches once any of them is reached, from all their
		// points (over-reach only), kept one per PUSH_POINT_GRID cell
		struct Cluster
		{
			std::vector<int> nodes;
			std::vector<dvec3> pts;
			std::set<long long> cells;
		};
		std::map<long long, Cluster> clusters;
		for ( std::map< int, std::vector<dvec3> >::const_iterator it = nodePts.begin(); it != nodePts.end(); ++it )
		{
			const WalkNode &n = graph.nodes[it->first];
			long long ix = (long long)floor( n.mins.x / PUSH_CLUSTER_XY ), iy = (long long)floor( n.mins.y / PUSH_CLUSTER_XY );
			long long iz = (long long)floor( n.mins.z / PUSH_CLUSTER_Z );
			Cluster &c = clusters[( ( ix + ( 1 << 20 ) ) << 42 ) | ( ( iy + ( 1 << 20 ) ) << 21 ) | ( iz + ( 1 << 20 ) )];
			c.nodes.push_back( it->first );
			for ( size_t i = 0; i < it->second.size(); i++ )
			{
				const dvec3 &p = it->second[i];
				long long gx = (long long)floor( p.x / PUSH_POINT_GRID ), gy = (long long)floor( p.y / PUSH_POINT_GRID );
				long long gz = (long long)floor( p.z / PUSH_POINT_GRID );
				if ( c.cells.insert( ( ( gx + ( 1 << 20 ) ) << 42 ) | ( ( gy + ( 1 << 20 ) ) << 21 ) | ( gz + ( 1 << 20 ) ) ).second )
					c.pts.push_back( p );
			}
		}
		std::vector<dvec3> pts;
		for ( std::map<long long, Cluster>::const_iterator it = clusters.begin(); it != clusters.end(); ++it )
		{
			ThinPoints( it->second.pts, PUSH_CLUSTER_MAX_POINTS, pts );
			int e = PushEntry( pv, pts );
			for ( size_t i = 0; i < it->second.nodes.size(); i++ )
				Edge( it->second.nodes[i], Id( e ) );
		}
		// indexed, never a reference into sg.verts: PushEntry adds a vertex,
		// and the reallocation would leave a reference to the old element
		// dangling (an intermittent access violation on f14)
		for ( size_t k = 0; k < sg.verts[pv].volumes.size(); k++ )
		{
			SamplePolytope( sg.verts[pv].volumes[k], PUSH_TARGET_SPACING, PUSH_TARGET_CAP, pts );
			for ( size_t i = 0; i < pts.size(); i++ )
			{
				std::vector<dvec3> one( 1, pts[i] );
				SpecialTarget t = { pts[i], PushEntry( pv, one ) };
				sg.targets.push_back( t );
			}
		}
	}

	//-------------------------------------------------------------------------
	// trigger_push / trigger_catapult / trigger_teleport
	//-------------------------------------------------------------------------
	void Triggers()
	{
		const double J = profile.maxJumpRise, g = profile.gravity, vj = profile.jumpVelocity, vmax = Vmax();
		std::vector<GrownPolytope> vols;
		std::vector<int> touching;
		for ( int e = 0; e < num_entities; e++ )
		{
			entity_t *ent = &entities[e];
			const char *cls = EntValue( ent, "classname" );
			int kind = -1;
			if ( !Q_stricmp( cls, "trigger_push" ) )
				kind = SPECIAL_PUSH;
			else if ( !Q_stricmp( cls, "trigger_catapult" ) )
				kind = SPECIAL_CATAPULT;
			else if ( !Q_stricmp( cls, "trigger_teleport" ) )
				kind = SPECIAL_TELEPORT;
			if ( kind < 0 )
				continue;
			int sf = atoi( EntValue( ent, "spawnflags" ) );
			bool players = ( sf & ( 1 | 64 ) ) != 0;	// SF_TRIGGER_ALLOW_CLIENTS, SF_TRIGGER_ALLOW_ALL
			const char *name = EntValue( ent, "targetname" );

			TeleportReport rep;
			if ( kind == SPECIAL_TELEPORT )
			{
				rep.entity = e;
				rep.vertex = -1;
				Q_strncpy( rep.target, EntValue( ent, "target" ), sizeof( rep.target ) );
				Q_strncpy( rep.landmark, EntValue( ent, "landmark" ), sizeof( rep.landmark ) );
				rep.startDisabled = atoi( EntValue( ent, "StartDisabled" ) ) != 0;
				rep.players = players;
			}
			if ( !players )
			{
				sg.stats.skippedNoPlayers++;
				if ( kind == SPECIAL_TELEPORT )
					sg.teleports.push_back( rep );
				continue;
			}
			// StartDisabled triggers count: some output may enable them
			EntityVolumes( ent, vols );
			if ( vols.empty() )
			{
				if ( kind == SPECIAL_TELEPORT )
					sg.teleports.push_back( rep );
				continue;
			}
			int v = AddVertex( kind, e, name[0] ? name : cls );
			sg.verts[v].volumes = vols;
			for ( size_t k = 0; k < vols.size(); k++ )
			{
				GrownPolytope t;
				if ( ExtendDown( vols[k], J, t ) )
					touch[v].push_back( t );
				else
					touch[v].push_back( vols[k] );
			}
			triggerVerts.push_back( v );

			dvec3 vmins( 1e300, 1e300, 1e300 ), vmaxs( -1e300, -1e300, -1e300 );
			for ( size_t k = 0; k < vols.size(); k++ )
			{
				vmins = DMin( vmins, vols[k].mins );
				vmaxs = DMax( vmaxs, vols[k].maxs );
			}

			if ( kind == SPECIAL_PUSH )
			{
				// A continuous push is base velocity, set on every touch
				// (triggers.cpp CTriggerPush::Touch): its z part is an acceleration
				// (gamemovement.cpp StartGravity adds basevel.z dt each tick), so
				// inside the push gravity is g - s dz; its xy part carries the
				// player and is kept as momentum on leaving (player_command.cpp).
				// The player enters with his own state: standing (vz 0; a lifting
				// push takes him off the ground), jumping (vz vj), or duck-jumping
				// (lifted crouch_lift). A push pulling down is modelled with plain
				// gravity (more reach). A push-once (spawnflag 0x80) is one impulse
				// s * dir added to that state, then plain flight.
				double s = atof( EntValue( ent, "speed" ) );
				dvec3 v3 = PushVelocity( EntValue( ent, "pushdir" ), s );
				PushInfo &pi = pushInfo[v];
				pi.top = vmaxs.z;
				if ( sf & 0x80 )
				{
					pi.vel.push_back( Vel( v3.z, 0.0, v3.x, v3.y, vmax, true ) );
					pi.vel.push_back( Vel( vj + v3.z, 0.0, v3.x, v3.y, vmax, true ) );
					pi.vel.push_back( Vel( vj + v3.z, profile.crouchLift, v3.x, v3.y, vmax, true ) );
				}
				else
				{
					double gIn = MIN( g, g - v3.z );
					pi.vel.push_back( PushVel( 0.0, 0.0, v3.x, v3.y, vmax, gIn ) );
					pi.vel.push_back( PushVel( vj, 0.0, v3.x, v3.y, vmax, gIn ) );
					pi.vel.push_back( PushVel( vj, profile.crouchLift, v3.x, v3.y, vmax, gIn ) );
				}
				PushEntries( v );
			}
			else if ( kind == SPECIAL_CATAPULT )
			{
				// best effort (later engines): the launch velocity, or the launch target's node
				double speed = atof( EntValue( ent, "playerSpeed" ) );
				if ( speed <= 0 )
					speed = 450.0;
				double p = 0, y = 0, r = 0;
				sscanf( EntValue( ent, "launchDirection" ), "%lf %lf %lf", &p, &y, &r );
				dvec3 v3 = AngleForward( p, y ) * speed;
				const char *tgt = EntValue( ent, "launchTarget" );
				if ( tgt[0] )
				{
					for ( int t = FindEntityByTargetname( tgt, -1 ); t >= 0; t = FindEntityByTargetname( tgt, t ) )
					{
						int node = MapPoint( EntOrigin( &entities[t] ) );
						if ( node >= 0 )
							Edge( Id( v ), node );
					}
				}
				SpecialLauncher L;
				L.volumes = vols;
				L.exitDir = v3;
				DNormalize( L.exitDir );
				L.hasExitDir = true;
				L.vel.push_back( Vel( v3.z, 0.0, v3.x, v3.y, vmax, false ) );
				AddLauncher( v, L );
				TriggerNodeEdges( v );
				TriggerTargets( v );
			}
			else
			{
				TriggerNodeEdges( v );
				TriggerTargets( v );
				Teleport( v, ent, vols, rep );
			}
		}
	}

	//-------------------------------------------------------------------------
	// trigger_teleport: every entity named by its target is a destination (the
	// engine takes the first; which one that is depends on spawn order). The
	// player arrives at the destination's origin and falls from it; velocity is
	// kept (HL2), modelled as the arrival launcher's families. With a landmark
	// the arrival is offset by (player - landmark): the trigger's volumes moved
	// by (destination - landmark) are arrival regions as well.
	//-------------------------------------------------------------------------
	void Teleport( int v, entity_t *ent, const std::vector<GrownPolytope> &vols, TeleportReport &rep )
	{
		const double vj = profile.jumpVelocity, vmax = Vmax();
		rep.vertex = v;
		dvec3 lmOrg;
		bool hasLm = false;
		if ( rep.landmark[0] )
		{
			int lm = FindEntityByTargetname( rep.landmark, -1 );
			if ( lm >= 0 )
			{
				lmOrg = EntOrigin( &entities[lm] );
				hasLm = true;
			}
			else
				Warning( "bspwalk: trigger_teleport %s: landmark \"%s\" names no entity\n", sg.verts[v].label, rep.landmark );
		}
		std::vector<int> touching;
		for ( int d = rep.target[0] ? FindEntityByTargetname( rep.target, -1 ) : -1; d >= 0;
			  d = FindEntityByTargetname( rep.target, d ) )
		{
			TeleportDest td;
			td.entity = d;
			td.point = EntOrigin( &entities[d] );
			td.node = MapPoint( td.point );
			if ( td.node < 0 )
				Warning( "bspwalk: trigger_teleport %s: destination \"%s\" at (%.1f %.1f %.1f) maps to no node\n",
						 sg.verts[v].label, rep.target, td.point.x, td.point.y, td.point.z );
			rep.dests.push_back( td );

			const char *dname = EntValue( &entities[d], "targetname" );
			int a = AddVertex( SPECIAL_ARRIVAL, d, dname );
			Edge( Id( v ), Id( a ) );
			if ( td.node >= 0 )
				Edge( Id( a ), td.node );
			for ( size_t w = 0; w < waterVerts.size(); w++ )
			{
				if ( PointInPolytope( sg.verts[waterVerts[w]].volumes[0], td.point, TOUCH_EPS ) )
					Edge( Id( a ), Id( waterVerts[w] ) );
			}
			SpecialLauncher L;
			L.hasExitDir = false;
			L.points.push_back( td.point );
			if ( hasLm )
			{
				dvec3 off = td.point - lmOrg;
				for ( size_t k = 0; k < vols.size(); k++ )
				{
					GrownPolytope q = vols[k];
					TranslatePolytope( q, off );
					sg.verts[a].volumes.push_back( q );
					L.volumes.push_back( q );
					NodesTouching( q, touching );
					for ( size_t i = 0; i < touching.size(); i++ )
						Edge( Id( a ), touching[i] );
				}
			}
			L.vel.push_back( Vel( 0.0, 0.0, 0, 0, vmax, false ) );
			L.vel.push_back( Vel( vj, profile.crouchLift, 0, 0, vmax, true ) );
			L.vel.push_back( Vel( vj, 0.0, 0, 0, vmax, true ) );
			AddLauncher( a, L );
		}
		if ( !rep.target[0] || rep.dests.empty() )
			Warning( "bspwalk: trigger_teleport %s: target \"%s\" names no entity\n", sg.verts[v].label, rep.target );
		sg.teleports.push_back( rep );
	}

	//-------------------------------------------------------------------------
	// Cross links: water, ladders and triggers that meet.
	//-------------------------------------------------------------------------
	// PushEntry adds vertices (sg.verts and touch reallocate), so nothing here
	// holds a reference into either across a call to it: what is used after one
	// is copied.
	void CrossLinks()
	{
		const double J = profile.maxJumpRise;
		std::vector<dvec3> pts;
		DWinding face;
		for ( size_t w = 0; w < waterVerts.size(); w++ )
		{
			int wv = waterVerts[w];
			const GrownPolytope W = sg.verts[wv].volumes[0];
			// a swimmer within mount range of a useable ladder
			for ( size_t l = 0; l < ladderVerts.size(); l++ )
			{
				const SpecialVertex &L = sg.verts[ladderVerts[l]];
				DWinding para;
				para.push_back( L.a );
				para.push_back( L.b );
				para.push_back( L.b - dvec3( 0, 0, J ) );
				para.push_back( L.a - dvec3( 0, 0, J ) );
				dvec3 lo = DMin( L.a, L.b ) - dvec3( LADDER_MOUNT, LADDER_MOUNT, LADDER_MOUNT + J );
				dvec3 hi = DMax( L.a, L.b ) + dvec3( LADDER_MOUNT, LADDER_MOUNT, LADDER_MOUNT );
				if ( !BoxesMeet( W.mins, W.maxs, lo, hi, 0 ) )
					continue;
				bool near = PointInPolytope( W, L.a, TOUCH_EPS ) || PointInPolytope( W, L.b, TOUCH_EPS );
				for ( size_t f = 0; f < W.faces.size() && !near; f++ )
				{
					FacePolygon( W, f, face );
					near = PolyPolyDistance( face, para ) <= LADDER_MOUNT;
				}
				if ( near )
					Both( Id( wv ), Id( ladderVerts[l] ) );
			}
			for ( size_t l = 0; l < ladderBrushVerts.size(); l++ )
			{
				if ( PolytopesIntersect( W, touch[ladderBrushVerts[l]][0], TOUCH_EPS, NULL ) )
					Both( Id( wv ), Id( ladderBrushVerts[l] ) );
			}
			for ( size_t t = 0; t < triggerVerts.size(); t++ )
			{
				int tv = triggerVerts[t];
				if ( sg.verts[tv].kind == SPECIAL_PUSH )
				{
					// a swimmer enters the push where the water meets it
					for ( size_t k = 0; k < sg.verts[tv].volumes.size(); k++ )
					{
						pts.clear();
						if ( PolytopesIntersect( W, sg.verts[tv].volumes[k], TOUCH_EPS, &pts ) &&
							 !InsideOneObstacle( crouchObs, pts ) )
						{
							std::vector<dvec3> lp;
							ThinPoints( pts, PUSH_ENTRY_MAX_POINTS, lp );
							Edge( Id( wv ), Id( PushEntry( tv, lp ) ) );
						}
					}
					continue;
				}
				for ( size_t k = 0; k < touch[tv].size(); k++ )
				{
					if ( PolytopesIntersect( W, touch[tv][k], TOUCH_EPS, NULL ) )
					{
						Edge( Id( wv ), Id( tv ) );
						break;
					}
				}
			}
		}
		// climbers that reach into a trigger (into a push: an entry where they meet it)
		for ( size_t t = 0; t < triggerVerts.size(); t++ )
		{
			int tv = triggerVerts[t];
			const bool push = sg.verts[tv].kind == SPECIAL_PUSH;
			for ( size_t l = 0; l < ladderVerts.size(); l++ )
			{
				const dvec3 La = sg.verts[ladderVerts[l]].a, Lb = sg.verts[ladderVerts[l]].b;
				for ( size_t k = 0; k < touch[tv].size(); k++ )
				{
					dvec3 c0, c1;
					if ( !ClipSegment( La, Lb, touch[tv][k], TOUCH_EPS, c0, c1 ) )
						continue;
					if ( push )
					{
						std::vector<dvec3> lp;
						lp.push_back( c0 );
						lp.push_back( c1 );
						lp.push_back( ( c0 + c1 ) * 0.5 );
						Edge( Id( ladderVerts[l] ), Id( PushEntry( tv, lp ) ) );
						continue;
					}
					Edge( Id( ladderVerts[l] ), Id( tv ) );
					break;
				}
			}
			for ( size_t l = 0; l < ladderBrushVerts.size(); l++ )
			{
				const GrownPolytope C = sg.verts[ladderBrushVerts[l]].volumes[0];
				for ( size_t k = 0; k < touch[tv].size(); k++ )
				{
					pts.clear();
					if ( !PolytopesIntersect( C, touch[tv][k], TOUCH_EPS, push ? &pts : NULL ) )
						continue;
					if ( push )
					{
						std::vector<dvec3> lp;
						ThinPoints( pts, PUSH_ENTRY_MAX_POINTS, lp );
						Edge( Id( ladderBrushVerts[l] ), Id( PushEntry( tv, lp ) ) );
						continue;
					}
					Edge( Id( ladderBrushVerts[l] ), Id( tv ) );
					break;
				}
			}
		}
	}

	//-------------------------------------------------------------------------
	// Rides: a vertex per (root mover, pose); the nodes of every tops_only
	// convex riding that root at that pose join it both ways, and the root's
	// pose links join its vertices both ways.
	//-------------------------------------------------------------------------
	void Rides()
	{
		std::map<int, int> meOf;	// entity -> moverEnts index
		for ( size_t i = 0; i < graph.moverEnts.size(); i++ )
			meOf[graph.moverEnts[i].entity] = (int)i;
		std::map< std::pair<int, int>, int > rideOf;
		std::set<int> rootsLinked;
		for ( size_t i = 0; i < graph.moverEnts.size(); i++ )
		{
			const MoverEntity &me = graph.moverEnts[i];
			if ( me.poses < 2 )
				continue;
			for ( int p = 0; p < me.poses; p++ )
			{
				std::pair<int, int> key( me.root, p );
				if ( rideOf.count( key ) )
					continue;
				const char *name = ( me.root >= 0 && me.root < num_entities ) ? EntValue( &entities[me.root], "targetname" ) : "";
				const char *cls = ( me.root >= 0 && me.root < num_entities ) ? EntValue( &entities[me.root], "classname" ) : "mover";
				int v = AddVertex( SPECIAL_RIDE, me.root, name[0] ? name : cls );
				sg.verts[v].pose = p;
				rideOf[key] = v;
			}
			if ( rootsLinked.insert( me.root ).second )
			{
				for ( size_t k = 0; k < me.links.size(); k++ )
				{
					// every pose of the root got a vertex above
					std::map< std::pair<int, int>, int >::const_iterator a = rideOf.find( std::make_pair( me.root, me.links[k].first ) );
					std::map< std::pair<int, int>, int >::const_iterator b = rideOf.find( std::make_pair( me.root, me.links[k].second ) );
					Assert( a != rideOf.end() && b != rideOf.end() );
					if ( a == rideOf.end() || b == rideOf.end() )
						continue;
					Both( Id( a->second ), Id( b->second ) );
					sg.stats.rideLinks++;
				}
			}
		}
		for ( int n = 0; n < nNodes; n++ )
		{
			const WalkNode &wn = graph.nodes[n];
			if ( wn.category != CAT_TOPS_ONLY )
				continue;
			std::map<int, int>::const_iterator it = meOf.find( world.convexes[wn.convex].entityIndex );
			if ( it == meOf.end() )
				continue;
			const MoverEntity &me = graph.moverEnts[it->second];
			std::map< std::pair<int, int>, int >::const_iterator r = rideOf.find( std::make_pair( me.root, wn.pose ) );
			if ( r != rideOf.end() )
				Both( n, Id( r->second ) );
		}
	}

	//-------------------------------------------------------------------------
	// func_conveyor: its speed (default 100) is added to the arcs launched from
	// its top nodes. Spawnflag 1 is visual only (no push); 2 (not solid) never
	// makes nodes.
	//-------------------------------------------------------------------------
	void Conveyors()
	{
		for ( int n = 0; n < nNodes; n++ )
		{
			const WalkNode &wn = graph.nodes[n];
			if ( wn.category != CAT_TOPS_ONLY )
				continue;
			int e = world.convexes[wn.convex].entityIndex;
			if ( e < 0 || e >= num_entities || Q_stricmp( EntValue( &entities[e], "classname" ), "func_conveyor" ) )
				continue;
			if ( atoi( EntValue( &entities[e], "spawnflags" ) ) & 1 )
				continue;
			double speed = fabs( atof( EntValue( &entities[e], "speed" ) ) );
			if ( speed == 0 )
				speed = 100.0;
			if ( sg.speedBonus.empty() )
				sg.speedBonus.assign( nNodes, 0.0f );
			sg.speedBonus[n] = (float)speed;
			sg.stats.conveyorNodes++;
		}
	}
};

void BuildSpecialGraph( const CollisionWorld &world, const PlayerProfile &profile, WalkGraph &graph )
{
	double t0 = Plat_FloatTime();
	graph.special = SpecialGraph();
	SpecialBuilder b( world, profile, graph );
	b.Water();
	b.Ladders();
	b.LadderBrushes();
	b.Triggers();
	b.CrossLinks();
	b.Rides();
	b.Conveyors();
	SpecialStats &st = graph.special.stats;
	st.tBuild = Plat_FloatTime() - t0;
	Msg( "special: %d water, %d ladders, %d ladder brushes, %d pushes (%d entries), %d catapults, %d teleports (%d "
		 "arrivals), %d ride vertices (%d links); %d edges, %d trigger targets, %d conveyor nodes; skipped %d non-player "
		 "triggers, %d fake ladders (%.2f s)\n",
		 st.count[SPECIAL_WATER], st.count[SPECIAL_LADDER], st.count[SPECIAL_LADDER_BRUSH], st.count[SPECIAL_PUSH],
		 st.count[SPECIAL_PUSH_ENTRY], st.count[SPECIAL_CATAPULT], st.count[SPECIAL_TELEPORT], st.count[SPECIAL_ARRIVAL],
		 st.count[SPECIAL_RIDE], st.rideLinks, st.edges, (int)graph.special.targets.size(), st.conveyorNodes,
		 st.skippedNoPlayers, st.skippedFakeLadders, st.tBuild );
}
