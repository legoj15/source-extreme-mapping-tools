//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: Stage 3 ballistic arcs (#13). See bw_arcs.h and the "How #13 arcs
//          work" section of the design doc.
//
//          Target-driven, from the flood frontier: each round pairs every node
//          reached since the last round (A) with every unreached node (B)
//          within the profile's reach R in XY and at most maxJumpRise above A,
//          tries a small family of arcs per pair, and floods on from every B
//          that a free arc lands on. A pair is tried once: A is in exactly one
//          round's frontier. Rounds stop when one reaches nothing new.
//
//          Inside a round the pairs go in chunks of launch nodes, and each
//          chunk tries its launch/landing combinations in tiers: tier c traces
//          combination c of every pair whose B is still unreached, then the
//          BFS runs. A pair whose B was reached by an earlier tier (from any A)
//          is dropped, which changes nothing reached, only the arc count.
//
//=============================================================================//

#include <algorithm>
#include "cmdlib.h"
#include "threads.h"
#include "tier0/threadtools.h"
#include "bw_arcs.h"
#include "bw_hash.h"
#include "bw_optix.h"
#include "pacifier.h"

// Launch nodes per chunk, and arcs per GPU launch (memory bounds only).
static const int ARC_CHUNK_A = 16384;
static const int ARC_GPU_BATCH = 8 * 1024 * 1024;
// Pairs per batch: their geometry is cached across the 9 tiers (216 bytes each).
static const int ARC_PAIR_BATCH = 1024 * 1024;

//-----------------------------------------------------------------------------
// Arc solver
//-----------------------------------------------------------------------------
dvec3 ArcPoint( const ArcD &a, double g, double t )
{
	double z;
	const double g0 = g + a.dg0;
	if ( a.tKick > 0.0 && t > a.tKick )
	{
		double zk = a.start.z + a.vz * a.tKick - 0.5 * g0 * a.tKick * a.tKick;
		double u = t - a.tKick;
		z = zk + a.vzKick * u - 0.5 * g * u * u;
	}
	else
		z = a.start.z + a.vz * t - 0.5 * g0 * t * t;
	return dvec3( a.start.x + a.vx * t, a.start.y + a.vy * t, z );
}

// Chords over a piece of duration t under acceleration gp, each within sagitta.
static int PieceSegments( double t, double gp, double sagitta )
{
	if ( !( t > 0 ) )
		return 0;
	if ( fabs( gp ) < 1e-12 )
		return 1;	// a straight piece
	double dtMax = sqrt( 8.0 * sagitta / fabs( gp ) );
	return MIN( 65535, MAX( 1, (int)ceil( t / dtMax ) ) );
}

// A chord over dt of a parabola with acceleration g deviates from it by at
// most g dt^2 / 8 (vertically, at its middle; the perpendicular distance is
// smaller), so dt <= sqrt(8 sagitta / g) bounds every chord's error. A piece
// curving upward (g0 < 0) lies below its chords, the others above theirs: with
// one in the arc both pieces take half the bound, so one raise serves both
// (ArcTraceRaise).
void ArcSetSegments( ArcD &a, double g )
{
	const double g0 = g + a.dg0;
	const double sagitta = g0 < 0 ? 0.5 * ARC_SAGITTA : ARC_SAGITTA;
	double t0 = a.tKick > 0.0 ? a.tKick : a.tEnd;
	double t1 = a.tKick > 0.0 ? a.tEnd - a.tKick : 0.0;
	a.nSeg[0] = PieceSegments( t0, g0, sagitta );
	a.nSeg[1] = PieceSegments( t1, g, sagitta );
}

double ArcTraceRaise( const ArcD &a, double g )
{
	// signed sagitta of each piece's chords: > 0 where the arc bulges above them
	double down = 0.0, up = 0.0;
	if ( a.nSeg[0] > 0 )
	{
		double dt = ( a.tKick > 0.0 ? a.tKick : a.tEnd ) / a.nSeg[0];
		double s = ( g + a.dg0 ) * dt * dt / 8.0;
		if ( s > 0 )
			down = MAX( down, s );
		else
			up = MAX( up, -s );
	}
	if ( a.nSeg[1] > 0 )
	{
		double dt = ( a.tEnd - a.tKick ) / a.nSeg[1];
		down = MAX( down, g * dt * dt / 8.0 );
	}
	// never below 0: a lifting push (dg0 < 0) would otherwise sink the traced ends
	// below their floors (ARC_LIFT is 1/32) and block a free landing; a raise of 0
	// keeps the chord error within ARC_SAGITTA / 2 either way
	return MAX( 0.5 * ( down - up ), 0.0 );
}

bool SolveArcToPoint( const dvec3 &start, double vz, bool airJump, double g, const dvec3 &land, double maxSpeed,
					  int mesh, ArcD &out )
{
	double dz = land.z - start.z;
	double T;
	double tKick = 0.0, vzKick = 0.0;
	if ( airJump && vz > 0.0 )
	{
		// second jump of vz at the apex of the first
		tKick = vz / g;
		vzKick = vz;
		double hk = vz * vz / ( 2.0 * g );
		double disc = vzKick * vzKick - 2.0 * g * ( dz - hk );
		if ( disc < 0.0 )
			return false;
		T = tKick + ( vzKick + sqrt( disc ) ) / g;
	}
	else if ( vz > 0.0 )
	{
		double disc = vz * vz - 2.0 * g * dz;
		if ( disc < 0.0 )
			return false;
		T = ( vz + sqrt( disc ) ) / g;	// the later root: the descending branch
	}
	else
	{
		if ( dz >= 0.0 )
			return false;
		T = sqrt( -2.0 * dz / g );
	}
	if ( !( T > 0.0 ) )
		return false;
	double hx = land.x - start.x, hy = land.y - start.y;
	double dist = sqrt( hx * hx + hy * hy );
	if ( dist > maxSpeed * T )
		return false;
	out.start = start;
	out.vx = hx / T;
	out.vy = hy / T;
	out.vz = vz;
	out.tKick = tKick;
	out.vzKick = vzKick;
	out.dg0 = 0.0;
	out.tEnd = T;
	out.mesh = mesh;
	ArcSetSegments( out, g );
	return true;
}

bool SolveArcBase( const dvec3 &start, double vz, double bx, double by, double g, const dvec3 &land, double hmax,
				   int mesh, ArcD &out )
{
	// descending branch: the later root of z(T) = dz (vz may be <= 0)
	double dz = land.z - start.z;
	double disc = vz * vz - 2.0 * g * dz;
	if ( disc < 0.0 )
		return false;
	double T = ( vz + sqrt( disc ) ) / g;
	if ( !( T > 1e-9 ) )
		return false;
	double vx = ( land.x - start.x ) / T, vy = ( land.y - start.y ) / T;
	double ex = vx - bx, ey = vy - by;
	if ( ex * ex + ey * ey > hmax * hmax )
		return false;
	out.start = start;
	out.vx = vx;
	out.vy = vy;
	out.vz = vz;
	out.tKick = 0.0;
	out.vzKick = 0.0;
	out.dg0 = 0.0;
	out.tEnd = T;
	out.mesh = mesh;
	ArcSetSegments( out, g );
	return true;
}

// Squared distance from (x, y) to the segment from 0 to (bx, by).
static double DistSqToBaseSegment( double x, double y, double bx, double by )
{
	double bb = bx * bx + by * by;
	double t = bb > 0 ? ( x * bx + y * by ) / bb : 0.0;
	t = MAX( 0.0, MIN( 1.0, t ) );
	double ex = x - t * bx, ey = y - t * by;
	return ex * ex + ey * ey;
}

bool SolvePushArc( const dvec3 &start, double vz, double gIn, double zExit, double bx, double by, double g,
				   const dvec3 &land, double hmax, int mesh, ArcD &out )
{
	double T, t0 = 0.0, vzX = 0.0;
	const bool exits = zExit > start.z + 1e-6;
	if ( exits )
	{
		// up to zExit on the ascending branch under gIn
		double h = zExit - start.z;
		double disc = vz * vz - 2.0 * gIn * h;
		if ( disc < 0.0 )
			return false;
		vzX = sqrt( disc );
		if ( fabs( gIn ) > 1e-9 )
			t0 = ( vz - vzX ) / gIn;
		else if ( vz > 0.0 )
			t0 = h / vz;
		else
			return false;
		if ( !( t0 > 1e-9 ) )
			return false;
		double d1 = vzX * vzX - 2.0 * g * ( land.z - zExit );
		if ( d1 < 0.0 )
			return false;
		double T1 = ( vzX + sqrt( d1 ) ) / g;
		if ( !( T1 > 1e-9 ) )
			return false;
		T = t0 + T1;
	}
	else
	{
		if ( !( gIn > 1e-9 ) )
			return false;	// a lifting push never comes down inside
		double d0 = vz * vz - 2.0 * gIn * ( land.z - start.z );
		if ( d0 < 0.0 )
			return false;
		T = ( vz + sqrt( d0 ) ) / gIn;
		if ( !( T > 1e-9 ) )
			return false;
	}
	double vx = ( land.x - start.x ) / T, vy = ( land.y - start.y ) / T;
	if ( DistSqToBaseSegment( vx, vy, bx, by ) > hmax * hmax )
		return false;
	out.start = start;
	out.vx = vx;
	out.vy = vy;
	out.vz = vz;
	out.tKick = exits ? t0 : 0.0;
	out.vzKick = exits ? vzX : 0.0;
	out.dg0 = gIn - g;
	out.tEnd = T;
	out.mesh = mesh;
	ArcSetSegments( out, g );
	// a flight longer than the tracers' chord buffers (a very tall lift) gets
	// coarser chords rather than none: the same counts on both paths
	int total = out.nSeg[0] + out.nSeg[1];
	if ( total > ARC_MAX_CHORDS )
	{
		double k = (double)ARC_MAX_CHORDS / total;
		out.nSeg[0] = out.nSeg[0] ? MAX( 1, (int)( out.nSeg[0] * k ) ) : 0;
		out.nSeg[1] = out.nSeg[1] ? MAX( 1, (int)( out.nSeg[1] * k ) ) : 0;
		while ( out.nSeg[0] + out.nSeg[1] > ARC_MAX_CHORDS )
			out.nSeg[out.nSeg[0] > out.nSeg[1] ? 0 : 1]--;
	}
	return true;
}

//-----------------------------------------------------------------------------
// The arc family of a pair. Launch points on A (XY, z on A's plane):
//   L0  the point of A nearest to B (closest pair of the two polygons),
//   L1  the point of A's boundary nearest to B's centroid,
//   L2  the point of A's boundary nearest to B's far point.
// Landing points on B:
//   T0  the point of B nearest to A, T1  B's centroid, T2  the vertex of B
//   farthest from L0.
// Landing points, and launch points of the jumps, are moved ARC_INSET toward
// their polygon's centroid (a polygon edge cut by an obstacle lies 1/32u
// inside it). A walk-off starts on the exact boundary point: before the edge
// the player is on the ground, so a fall from an inset point would start
// inside the floor's own solid.
// Combinations, in tier order: (0,0) (0,1) (1,1) (0,2) (2,2) (1,0) (1,2)
// (2,0) (2,1); one that repeats an earlier one's points is skipped. Per
// combination: duck-jump (lifted by crouch_lift), crouched jump, walk-off
// (only down), and with air_jumps the two jumps with a second jump at the apex.
//-----------------------------------------------------------------------------
static const int s_Combo[ARC_COMBOS][2] = { { 0, 0 }, { 0, 1 }, { 1, 1 }, { 0, 2 }, { 2, 2 }, { 1, 0 }, { 1, 2 }, { 2, 0 }, { 2, 1 } };

static void ClosestOnSeg( double ax, double ay, double bx, double by, double px, double py, double &qx, double &qy )
{
	double ex = bx - ax, ey = by - ay;
	double ll = ex * ex + ey * ey;
	double t = ll > 0 ? ( ( px - ax ) * ex + ( py - ay ) * ey ) / ll : 0.0;
	t = MAX( 0.0, MIN( 1.0, t ) );
	qx = ax + t * ex;
	qy = ay + t * ey;
}

// Nearest point of the polygon's boundary to (px, py).
static void BoundaryNearest( const DWinding &w, double px, double py, double &qx, double &qy )
{
	double best = 1e300;
	int n = (int)w.size();
	for ( int i = 0; i < n; i++ )
	{
		const dvec3 &a = w[i], &b = w[( i + 1 ) % n];
		double cx, cy;
		ClosestOnSeg( a.x, a.y, b.x, b.y, px, py, cx, cy );
		double d = ( cx - px ) * ( cx - px ) + ( cy - py ) * ( cy - py );
		if ( d < best )
		{
			best = d;
			qx = cx;
			qy = cy;
		}
	}
}

// Closest pair of two convex polygons in XY (vertex against edge both ways;
// for overlapping footprints a pair near the overlap).
static void ClosestPair( const DWinding &A, const DWinding &B, double &ax, double &ay, double &bx, double &by )
{
	double best = 1e300;
	for ( int pass = 0; pass < 2; pass++ )
	{
		const DWinding &P = pass ? B : A, &Q = pass ? A : B;
		for ( size_t i = 0; i < P.size(); i++ )
		{
			double qx, qy;
			BoundaryNearest( Q, P[i].x, P[i].y, qx, qy );
			double d = ( qx - P[i].x ) * ( qx - P[i].x ) + ( qy - P[i].y ) * ( qy - P[i].y );
			if ( d < best )
			{
				best = d;
				if ( pass )
					ax = qx, ay = qy, bx = P[i].x, by = P[i].y;
				else
					ax = P[i].x, ay = P[i].y, bx = qx, by = qy;
			}
		}
	}
}

static void Centroid( const DWinding &w, double &cx, double &cy )
{
	cx = cy = 0;
	for ( size_t i = 0; i < w.size(); i++ )
	{
		cx += w[i].x;
		cy += w[i].y;
	}
	cx /= w.size();
	cy /= w.size();
}

static dvec3 OnNode( const WalkNode &n, double x, double y )
{
	return dvec3( x, y, n.Z( x, y ) );
}

// Moves (x, y) ARC_INSET toward the centroid (at most half way).
static dvec3 Inset( const WalkNode &n, double x, double y, double cx, double cy )
{
	double dx = cx - x, dy = cy - y;
	double len = sqrt( dx * dx + dy * dy );
	if ( len > 0 )
	{
		double s = MIN( ARC_INSET, 0.5 * len ) / len;
		x += dx * s;
		y += dy * s;
	}
	return OnNode( n, x, y );
}

struct PairPoints
{
	dvec3 edge[3];		// exact launch points (walk-off)
	dvec3 launch[3];	// inset launch points (jumps)
	dvec3 land[3];		// inset landing points
	bool walkOffOnly;	// A is a slide: no jumps
	double slideNz;		// A is a slide: its normal z (surf arcs, #24); -1 otherwise
	double bonus;		// A's conveyor speed, added to the air speed
};

// Surf arcs (#24): a slide carries the player along its plane at up to vmax, so
// it launches with vertical speed up to vmax * sin(slope) = vmax * sqrt(1 - nz^2)
// and horizontal speed up to vmax (a superset of the in-plane velocities).
static double SurfVz( double nz, double vmax )
{
	return ( nz >= 0 && SurfEnabled() ) ? vmax * sqrt( MAX( 0.0, 1.0 - nz * nz ) ) : 0.0;
}

// XY reach of a surf arc landing 'drop' below its launch height (clamped to
// [0, arc_max_drop]): vmax times the time to come down that far, plus the hull.
static double SurfReachAtDrop( double vz, double drop, const PlayerProfile &p )
{
	double g = p.gravity > 0 ? p.gravity : 1.0;
	drop = MAX( 0.0, MIN( drop, p.arcMaxDrop ) );
	double t = ( vz + sqrt( vz * vz + 2.0 * g * drop ) ) / g;
	return p.maxAirSpeed * p.reachMargin * t + 2.0 * p.halfWidth;
}

static void PairGeometry( const WalkNode &A, const WalkNode &B, PairPoints &pp )
{
	double cax, cay, cbx, cby;
	Centroid( A.poly, cax, cay );
	Centroid( B.poly, cbx, cby );
	double l0x, l0y, t0x, t0y;
	ClosestPair( A.poly, B.poly, l0x, l0y, t0x, t0y );
	// far point of B from L0
	double far = -1, t2x = cbx, t2y = cby;
	for ( size_t i = 0; i < B.poly.size(); i++ )
	{
		double d = ( B.poly[i].x - l0x ) * ( B.poly[i].x - l0x ) + ( B.poly[i].y - l0y ) * ( B.poly[i].y - l0y );
		if ( d > far )
		{
			far = d;
			t2x = B.poly[i].x;
			t2y = B.poly[i].y;
		}
	}
	double l1x, l1y, l2x, l2y;
	BoundaryNearest( A.poly, cbx, cby, l1x, l1y );
	BoundaryNearest( A.poly, t2x, t2y, l2x, l2y );

	double lx[3] = { l0x, l1x, l2x }, ly[3] = { l0y, l1y, l2y };
	double tx[3] = { t0x, cbx, t2x }, ty[3] = { t0y, cby, t2y };
	for ( int i = 0; i < 3; i++ )
	{
		pp.edge[i] = OnNode( A, lx[i], ly[i] );
		pp.launch[i] = Inset( A, lx[i], ly[i], cax, cay );
		pp.land[i] = ( i == 1 ) ? OnNode( B, cbx, cby ) : Inset( B, tx[i], ty[i], cbx, cby );
	}
	pp.walkOffOnly = A.slide != 0;
	pp.slideNz = ( A.slide && A.surf ) ? A.plane.n.z : -1.0;
	pp.bonus = 0.0;
}

static bool SameXY( const dvec3 &a, const dvec3 &b )
{
	return fabs( a.x - b.x ) < 1e-3 && fabs( a.y - b.y ) < 1e-3;
}

static int ComboArcs( const PairPoints &pp, const PlayerProfile &p, int combo, std::vector<ArcD> &out )
{
	int li = s_Combo[combo][0], ti = s_Combo[combo][1];
	for ( int c = 0; c < combo; c++ )
	{
		if ( SameXY( pp.edge[s_Combo[c][0]], pp.edge[li] ) && SameXY( pp.land[s_Combo[c][1]], pp.land[ti] ) )
			return 0;	// a repeat of an earlier combination
	}
	const double g = p.gravity, vmax = p.maxAirSpeed * p.reachMargin + pp.bonus, vj = p.jumpVelocity;
	const dvec3 &L = pp.launch[li], &E = pp.edge[li], &T = pp.land[ti];
	size_t before = out.size();
	ArcD a;
	const int maxKick = pp.walkOffOnly ? -1 : ( p.airJumps > 0 ? 1 : 0 );	// a slide never jumps
	for ( int kick = 0; kick <= maxKick; kick++ )
	{
		if ( SolveArcToPoint( L + dvec3( 0, 0, p.crouchLift ), vj, kick != 0, g, T, vmax, BW_MESH_CROUCH, a ) )
			out.push_back( a );
		if ( SolveArcToPoint( L, vj, kick != 0, g, T, vmax, BW_MESH_CROUCH, a ) )
			out.push_back( a );
	}
	if ( T.z < E.z - 1e-6 && SolveArcToPoint( E, 0.0, false, g, T, vmax, BW_MESH_CROUCH, a ) )
		out.push_back( a );
	// surf arcs from a slide: its full vertical speed, and the one that peaks
	// just above the landing when that is lower
	const double svz = SurfVz( pp.slideNz, vmax );
	if ( svz > 0 )
	{
		if ( SolveArcToPoint( L, svz, false, g, T, vmax, BW_MESH_CROUCH, a ) )
			out.push_back( a );
		double rise = T.z + 1.0 - L.z;
		double apexVz = rise > 0 ? sqrt( 2.0 * g * rise ) : 0.0;
		if ( apexVz > 0 && apexVz < svz - 1e-6 && SolveArcToPoint( L, apexVz, false, g, T, vmax, BW_MESH_CROUCH, a ) )
			out.push_back( a );
	}
	// lift start and landing alike: the same arc, translated (see ARC_LIFT)
	for ( size_t i = before; i < out.size(); i++ )
		out[i].start.z += ARC_LIFT;
	return (int)( out.size() - before );
}

int PairArcs( const WalkNode &A, const WalkNode &B, const PlayerProfile &p, int combo, std::vector<ArcD> &out )
{
	if ( combo < 0 || combo >= ARC_COMBOS || A.poly.size() < 3 || B.poly.size() < 3 )
		return 0;
	PairPoints pp;
	PairGeometry( A, B, pp );
	return ComboArcs( pp, p, combo, out );
}

//-----------------------------------------------------------------------------
// Tracer
//-----------------------------------------------------------------------------
static ObstacleSet s_ArcObs[BW_MESH_COUNT];
static std::vector<int> s_ArcCand[MAX_TOOL_THREADS + 1];
struct alignas( 64 ) ArcLastBlocker
{
	int v[BW_MESH_COUNT];
};
static ArcLastBlocker s_ArcLast[MAX_TOOL_THREADS + 1];

// The chords of an arc, as the kernel walks them: (p, q, tmin, tmax).
struct ArcChord
{
	dvec3 p, q;
	double t0, t1;
};

// The chords are raised by ArcTraceRaise (half the largest chord sagitta), so the
// parabola lies within +-sagitta/2 of them. maxOut is ARC_MAX_CHORDS, which every
// profile's longest arc fits (ValidateArcChords at profile load).
static int ArcChords( const ArcD &a, double g, ArcChord *out, int maxOut )
{
	int total = a.nSeg[0] + a.nSeg[1], k = 0, m = 0;
	const dvec3 raise( 0, 0, ArcTraceRaise( a, g ) );
	for ( int piece = 0; piece < 2; piece++ )
	{
		int n = a.nSeg[piece];
		if ( !n )
			continue;
		double ts = piece ? a.tKick : 0.0;
		double te = ( piece || a.tKick <= 0.0 ) ? a.tEnd : a.tKick;
		dvec3 p = ArcPoint( a, g, ts ) + raise;
		for ( int s = 1; s <= n; s++, k++ )
		{
			double t = ts + ( te - ts ) * ( (double)s / (double)n );
			dvec3 q = ArcPoint( a, g, t ) + raise;
			double len = DLength( q - p );
			if ( len > 0 && m < maxOut )
			{
				// interior joints overlap (as on the GPU); the arc's own ends are exact
				double tmin = ( k == 0 ) ? 0.0 : -BW_ARC_JOINT_OVERLAP / len;
				double tmax = ( k == total - 1 ) ? 1.0 : 1.0 + BW_ARC_JOINT_OVERLAP / len;
				if ( tmax > tmin )
				{
					ArcChord &c = out[m++];
					c.p = p;
					c.q = q;
					c.t0 = tmin;
					c.t1 = tmax;
				}
			}
			p = q;
		}
	}
	return m;
}

// Does p + t d, t in [t0, t1], reach ARC_TOUCH into the polytope?
static bool ChordHits( const GrownPolytope &q, const dvec3 &p, const dvec3 &d, double t0, double t1 )
{
	for ( size_t k = 0; k < q.planes.size(); k++ )
	{
		const DPlane &pl = q.planes[k];
		double a = DDot( pl.n, p ) - ( pl.d - ARC_TOUCH );
		double b = DDot( pl.n, d );
		if ( fabs( b ) < 1e-15 )
		{
			if ( a >= 0 )
				return false;
			continue;
		}
		double t = -a / b;
		if ( b > 0 )
			t1 = MIN( t1, t );
		else
			t0 = MAX( t0, t );
		if ( t0 >= t1 )
			return false;
	}
	return true;
}

static bool BoxesOverlap3( const dvec3 &amin, const dvec3 &amax, const dvec3 &bmin, const dvec3 &bmax )
{
	return amin.x <= bmax.x && bmin.x <= amax.x && amin.y <= bmax.y && bmin.y <= amax.y && amin.z <= bmax.z &&
		   bmin.z <= amax.z;
}

static void ChordBounds( const ArcChord &c, dvec3 &mins, dvec3 &maxs )
{
	dvec3 a = c.p + ( c.q - c.p ) * c.t0, b = c.p + ( c.q - c.p ) * c.t1;
	mins = DMin( a, b );
	maxs = DMax( a, b );
}

static bool TraceArcCpu( int iThread, const ArcD &a, double g )
{
	// the CPU arc tracer only has the crouch obstacle set (BW_MESH_VIS is a GPU
	// sight mesh; its rays go through bw_vis's own CPU path)
	Assert( a.mesh == BW_MESH_CROUCH );
	ArcChord chords[ARC_MAX_CHORDS];
	int n = ArcChords( a, g, chords, ARC_MAX_CHORDS );
	const ObstacleSet &obs = s_ArcObs[a.mesh];
	std::vector<int> &cand = s_ArcCand[iThread];
	int &last = s_ArcLast[iThread].v[a.mesh];
	for ( int i = 0; i < n; i++ )
	{
		const ArcChord &c = chords[i];
		dvec3 d = c.q - c.p, mins, maxs;
		ChordBounds( c, mins, maxs );
		if ( last >= 0 && last < (int)obs.polys->size() && obs.use[last] )
		{
			const GrownPolytope &q = ( *obs.polys )[last];
			if ( BoxesOverlap3( mins, maxs, q.mins, q.maxs ) && ChordHits( q, c.p, d, c.t0, c.t1 ) )
				return true;
		}
		cand.clear();
		obs.hash.Query( mins.x, mins.y, maxs.x, maxs.y, cand );
		for ( size_t k = 0; k < cand.size(); k++ )
		{
			const GrownPolytope &q = ( *obs.polys )[cand[k]];
			if ( !BoxesOverlap3( mins, maxs, q.mins, q.maxs ) )
				continue;
			if ( ChordHits( q, c.p, d, c.t0, c.t1 ) )
			{
				last = cand[k];
				return true;
			}
		}
	}
	return false;
}

// Is p deeper than depth inside an obstacle of the mesh (a negative depth: closer
// than -depth to one, or inside)?
// cand: obstacle ids of the mesh, a superset of those whose box meets p's.
static bool PointInsideOf( const std::vector<int> &cand, const dvec3 &p, int mesh, double depth )
{
	const ObstacleSet &obs = s_ArcObs[mesh];
	const double m = MAX( 0.0, -depth );
	const dvec3 lo = p - dvec3( m, m, m ), hi = p + dvec3( m, m, m );
	for ( size_t k = 0; k < cand.size(); k++ )
	{
		const GrownPolytope &q = ( *obs.polys )[cand[k]];
		if ( !BoxesOverlap3( lo, hi, q.mins, q.maxs ) )
			continue;
		bool inside = true;
		for ( size_t j = 0; j < q.planes.size() && inside; j++ )
			inside = DDot( q.planes[j].n, p ) < q.planes[j].d - depth;
		if ( inside )
			return true;
	}
	return false;
}

static const std::vector<ArcD> *s_pTraceArcs;
static std::vector<unsigned char> *s_pTraceOut;
static double s_TraceG;

static void TraceCpuOne( int iThread, int i )
{
	( *s_pTraceOut )[i] = TraceArcCpu( iThread, ( *s_pTraceArcs )[i], s_TraceG ) ? 1 : 0;
}

CArcTracer::CArcTracer() : m_tBuild( 0 ), m_bGpu( false ), m_g( 600.0 )
{
	for ( int m = 0; m < BW_MESH_COUNT; m++ )
	{
		m_nTris[m] = 0;
		m_gasBytes[m] = 0;
	}
}

CArcTracer::~CArcTracer()
{
	Shutdown();
}

// Triangles of the grown obstacles (the same set as the CPU obstacle sets):
// every face fan-triangulated; its vertices are exact sum points.
static void BuildMesh( const CollisionWorld &world, const std::vector<GrownPolytope> &polys, std::vector<float> &verts,
					   std::vector<unsigned int> &tris )
{
	verts.clear();
	tris.clear();
	for ( size_t i = 0; i < polys.size(); i++ )
	{
		const GrownPolytope &q = polys[i];
		if ( !CategoryIsObstacle( world.convexes[i].category ) || !q.IsValid() )
			continue;
		unsigned int base = (unsigned int)( verts.size() / 3 );
		for ( size_t v = 0; v < q.verts.size(); v++ )
		{
			verts.push_back( (float)q.verts[v].x );
			verts.push_back( (float)q.verts[v].y );
			verts.push_back( (float)q.verts[v].z );
		}
		for ( size_t f = 0; f < q.faces.size(); f++ )
		{
			const std::vector<int> &face = q.faces[f];
			for ( size_t k = 2; k < face.size(); k++ )
			{
				tris.push_back( base + face[0] );
				tris.push_back( base + face[k - 1] );
				tris.push_back( base + face[k] );
			}
		}
	}
}

void CArcTracer::Init( const CollisionWorld &world, const PlayerProfile &profile, bool wantGpu )
{
	double t0 = Plat_FloatTime();
	m_g = profile.gravity;
	// every arc is traced crouched (a standing jump is dominated, and phase B's
	// launches are crouched too), so the stand obstacle set and mesh stay empty
	s_ArcObs[BW_MESH_CROUCH].Build( world, world.crouch );
	for ( int i = 0; i <= MAX_TOOL_THREADS; i++ )
		for ( int m = 0; m < BW_MESH_COUNT; m++ )
			s_ArcLast[i].v[m] = -1;

	m_bGpu = false;
	if ( wantGpu && BwOptix_Init() )
	{
		m_bGpu = true;
		std::vector<float> verts;
		std::vector<unsigned int> tris;
		for ( int m = BW_MESH_CROUCH; m <= BW_MESH_CROUCH && m_bGpu; m++ )
		{
			BuildMesh( world, world.crouch, verts, tris );
			m_nTris[m] = (int)( tris.size() / 3 );
			if ( !BwOptix_BuildMesh( m, verts, tris ) )
			{
				Warning( "bspwalk: cannot build the OptiX mesh; arcs fall back to the CPU\n" );
				BwOptix_Shutdown();
				m_bGpu = false;
			}
			m_gasBytes[m] = BwOptix_MeshBytes( m );
		}
	}
	if ( !m_bGpu )
	{
		for ( int m = 0; m < BW_MESH_COUNT; m++ )
		{
			m_nTris[m] = 0;
			m_gasBytes[m] = 0;
		}
	}
	m_tBuild = Plat_FloatTime() - t0;
}

void CArcTracer::Shutdown()
{
	if ( m_bGpu )
		BwOptix_Shutdown();
	m_bGpu = false;
	for ( int m = 0; m < BW_MESH_COUNT; m++ )
		s_ArcObs[m] = ObstacleSet();
}

void CArcTracer::TraceCpu( const std::vector<ArcD> &arcs, std::vector<unsigned char> &blocked )
{
	blocked.assign( arcs.size(), 0 );
	s_pTraceArcs = &arcs;
	s_pTraceOut = &blocked;
	s_TraceG = m_g;
	RunThreadsBatched( (int)arcs.size(), TraceCpuOne );
	s_pTraceArcs = NULL;
	s_pTraceOut = NULL;
}

static const ArcD *s_pConvIn;
static BwArc *s_pConvOut;
static double s_ConvG;

// The GPU form carries the CPU chords' raise in its origin (ArcTraceRaise).
static inline void ToBwArc( const ArcD &a, double g, BwArc &b )
{
	b.ox = (float)a.start.x;
	b.oy = (float)a.start.y;
	b.oz = (float)( a.start.z + ArcTraceRaise( a, g ) );
	b.vx = (float)a.vx;
	b.vy = (float)a.vy;
	b.vz = (float)a.vz;
	b.tKick = (float)a.tKick;
	b.vzKick = (float)a.vzKick;
	b.dg0 = (float)a.dg0;
	b.tEnd = (float)a.tEnd;
	b.nSeg[0] = (unsigned short)a.nSeg[0];
	b.nSeg[1] = (unsigned short)a.nSeg[1];
	b.mesh = (unsigned int)a.mesh;
}

static void ConvertArcOne( int iThread, int i )
{
	ToBwArc( s_pConvIn[i], s_ConvG, s_pConvOut[i] );
}

bool CArcTracer::TraceGpuRaw( const BwArc *arcs, size_t n, std::vector<unsigned char> &blocked )
{
	if ( !m_bGpu )
		return false;
	blocked.assign( n, 0 );
	for ( size_t first = 0; first < n; first += ARC_GPU_BATCH )
	{
		size_t k = MIN( (size_t)ARC_GPU_BATCH, n - first );
		if ( !BwOptix_TraceArcs( arcs + first, (unsigned int)k, (float)m_g, &blocked[first] ) )
			return false;
	}
	return true;
}

bool CArcTracer::TraceGpu( const std::vector<ArcD> &arcs, std::vector<unsigned char> &blocked )
{
	if ( !m_bGpu )
		return false;
	blocked.assign( arcs.size(), 0 );
	std::vector<BwArc> batch;
	for ( size_t first = 0; first < arcs.size(); first += ARC_GPU_BATCH )
	{
		size_t n = MIN( (size_t)ARC_GPU_BATCH, arcs.size() - first );
		batch.resize( n );
		s_pConvIn = &arcs[first];
		s_pConvOut = batch.data();
		s_ConvG = m_g;
		RunThreadsBatched( (int)n, ConvertArcOne );
		s_pConvIn = NULL;
		s_pConvOut = NULL;
		if ( !BwOptix_TraceArcs( batch.data(), (unsigned int)n, (float)m_g, &blocked[first] ) )
			return false;
	}
	return true;
}

// max over t in [t0, t1] of min_k (d_k - n_k . (p + t d)): concave, so a
// ternary search finds it.
static double ChordDepth( const GrownPolytope &q, const ArcChord &c )
{
	dvec3 d = c.q - c.p;
	double lo = c.t0, hi = c.t1;
	struct F
	{
		static double At( const GrownPolytope &q, const dvec3 &p, const dvec3 &d, double t )
		{
			dvec3 x = p + d * t;
			double m = 1e300;
			for ( size_t k = 0; k < q.planes.size(); k++ )
				m = MIN( m, q.planes[k].d - DDot( q.planes[k].n, x ) );
			return m;
		}
	};
	for ( int it = 0; it < 100; it++ )
	{
		double m1 = lo + ( hi - lo ) / 3, m2 = hi - ( hi - lo ) / 3;
		if ( F::At( q, c.p, d, m1 ) < F::At( q, c.p, d, m2 ) )
			lo = m1;
		else
			hi = m2;
	}
	return F::At( q, c.p, d, 0.5 * ( lo + hi ) );
}

double CArcTracer::Depth( const ArcD &a ) const
{
	Assert( a.mesh == BW_MESH_CROUCH );
	ArcChord chords[ARC_MAX_CHORDS];
	int n = ArcChords( a, m_g, chords, ARC_MAX_CHORDS );
	const ObstacleSet &obs = s_ArcObs[a.mesh];
	std::vector<int> cand;
	double best = -1e300;
	for ( int i = 0; i < n; i++ )
	{
		dvec3 mins, maxs;
		ChordBounds( chords[i], mins, maxs );
		mins = mins - dvec3( 1, 1, 1 );
		maxs = maxs + dvec3( 1, 1, 1 );
		cand.clear();
		obs.hash.Query( mins.x, mins.y, maxs.x, maxs.y, cand );
		for ( size_t k = 0; k < cand.size(); k++ )
		{
			const GrownPolytope &q = ( *obs.polys )[cand[k]];
			if ( BoxesOverlap3( mins, maxs, q.mins, q.maxs ) )
				best = MAX( best, ChordDepth( q, chords[i] ) );
		}
	}
	return best;
}

//-----------------------------------------------------------------------------
// Rounds
//-----------------------------------------------------------------------------
struct ArcPair
{
	int a, b;
	int ia;		// a's index in the chunk's launch list
};

static const WalkGraph *s_pArcGraph;
static const PlayerProfile *s_pArcProfile;
static const std::vector<int> *s_pArcLaunch;
enum { LAUNCH_RIM = 1, LAUNCH_SLOPE = 2 };	// 0: not a launch node
static const std::vector<unsigned char> *s_pArcRim;	// LAUNCH_* per node
static const std::vector<double> *s_pArcSlopeDrop;	// per node: the deepest drop a slope launch serves
static int s_nArcChunkFirst;
static CXYHash s_UnreachedHash;
static std::vector< std::vector<int> > *s_pArcTargets;	// per launch node of the chunk

// Targets of one launch node: unreached nodes at most maxJumpRise above it
// (a surf launcher, a surf slide with svz > 0, reaches svz^2 / 2g above it
// instead of maxJumpRise) and within R(drop) in XY, where drop is the largest
// drop between the two boxes (the widest reach any arc of the pair can have; a
// surf launcher's pairs use its surf reach, and the hash query the larger of the
// jump reach and that). A slope
// launcher (interior, LAUNCH_SLOPE) only takes targets in its downhill
// half-plane that drop at most its slope drop: elsewhere its downhill
// neighbour, closer and not much lower, reaches at least as far (see
// SlopeLaunchDrop).
static void PairsOne( int iThread, int i )
{
	const WalkGraph &g = *s_pArcGraph;
	const PlayerProfile &p = *s_pArcProfile;
	int a = ( *s_pArcLaunch )[s_nArcChunkFirst + i];
	const WalkNode &A = g.nodes[a];
	const bool slope = ( *s_pArcRim )[a] == LAUNCH_SLOPE;
	const double slopeDrop = slope ? ( *s_pArcSlopeDrop )[a] : 0.0;
	// the query bounds the per-pair R: B's z extent is at most 64 * sqrt(2) * tan(acos(0.7))
	// = 92.4u (the 64u chop, n.z >= 0.7)
	const double bonus = g.special.speedBonus.empty() ? 0.0 : g.special.speedBonus[a];
	// a conveyor's speed widens every R by bonus / vmax of its flight distance
	const double bonusScale = 1.0 + bonus / ( p.maxAirSpeed * p.reachMargin );
	double Rmax = ( slope ? p.ArcReachAtDrop( slopeDrop + ( A.maxs.z - A.mins.z ) + 93.0 ) : p.arcReachMax ) * bonusScale;
	// surf arcs (#24): a slide reaches higher and farther
	const double svz = ( A.slide && A.surf ) ? SurfVz( A.plane.n.z, p.maxAirSpeed * p.reachMargin ) : 0.0;
	if ( svz > 0 )
		Rmax = MAX( Rmax, SurfReachAtDrop( svz, p.arcMaxDrop, p ) * bonusScale );
	const double cx = 0.5 * ( A.mins.x + A.maxs.x ), cy = 0.5 * ( A.mins.y + A.maxs.y );
	std::vector<int> &out = ( *s_pArcTargets )[i];
	out.clear();
	if ( A.poly.size() < 3 )
		return;
	std::vector<int> &cand = s_ArcCand[iThread];
	cand.clear();
	s_UnreachedHash.Query( A.mins.x - Rmax, A.mins.y - Rmax, A.maxs.x + Rmax, A.maxs.y + Rmax, cand );
	double zTop = A.maxs.z + ( svz > 0 ? svz * svz / ( 2.0 * p.gravity ) : p.maxJumpRise ) + 1e-6;
	for ( size_t k = 0; k < cand.size(); k++ )
	{
		int b = cand[k];
		if ( b == a || g.reached[b] )
			continue;
		const WalkNode &B = g.nodes[b];
		if ( B.mins.z > zTop || B.poly.size() < 3 )
			continue;
		if ( A.slide && svz <= 0 && B.mins.z >= A.maxs.z )
			continue;	// without surfing a slide launches walk-offs only: targets below it
		if ( slope )
		{
			if ( A.mins.z - B.maxs.z > slopeDrop )
				continue;
			// the corner of B's box farthest downhill must lie downhill of A
			double nx = A.plane.n.x, ny = A.plane.n.y;
			double bx = nx > 0 ? B.maxs.x : B.mins.x, by = ny > 0 ? B.maxs.y : B.mins.y;
			if ( nx * ( bx - cx ) + ny * ( by - cy ) <= 0 )
				continue;
		}
		double dx = MAX( 0.0, MAX( B.mins.x - A.maxs.x, A.mins.x - B.maxs.x ) );
		double dy = MAX( 0.0, MAX( B.mins.y - A.maxs.y, A.mins.y - B.maxs.y ) );
		double R = ( svz > 0 ? SurfReachAtDrop( svz, A.maxs.z - B.mins.z, p ) : p.ArcReachAtDrop( A.maxs.z - B.mins.z ) ) *
				   bonusScale;
		if ( dx * dx + dy * dy > R * R )
			continue;
		out.push_back( b );
	}
}

// The crouch obstacles near each launch node of the chunk: every traced origin an
// arc of the node can have (on its polygon, or crouch_lift above it, then ARC_LIFT
// and at most ARC_SAGITTA / 2 up) lies in this box, so the list is a superset of a
// per-point query's.
static std::vector< std::vector<int> > *s_pLaunchObs;

static void LaunchObsOne( int iThread, int i )
{
	const WalkNode &A = s_pArcGraph->nodes[( *s_pArcLaunch )[s_nArcChunkFirst + i]];
	const ObstacleSet &obs = s_ArcObs[BW_MESH_CROUCH];
	const double m = ARC_START_CLEAR + 1e-3;	// plus slack for plane-z rounding
	const dvec3 lo = A.mins - dvec3( m, m, m );
	const dvec3 hi = A.maxs + dvec3( m, m, m + MAX( 0.0, s_pArcProfile->crouchLift ) + ARC_LIFT + 0.5 * ARC_SAGITTA );
	std::vector<int> &cand = s_ArcCand[iThread];
	cand.clear();
	obs.hash.Query( lo.x, lo.y, hi.x, hi.y, cand );
	std::vector<int> &out = ( *s_pLaunchObs )[i];
	out.clear();
	for ( size_t k = 0; k < cand.size(); k++ )
	{
		const GrownPolytope &q = ( *obs.polys )[cand[k]];
		if ( BoxesOverlap3( lo, hi, q.mins, q.maxs ) )
			out.push_back( cand[k] );
	}
}

// Pair geometry, computed once per pair and shared by its 9 tiers.
static const std::vector<ArcPair> *s_pGeomPairs;
static std::vector<PairPoints> *s_pGeomOut;

static void PairGeomOne( int iThread, int i )
{
	const ArcPair &pr = ( *s_pGeomPairs )[i];
	const WalkGraph &g = *s_pArcGraph;
	PairGeometry( g.nodes[pr.a], g.nodes[pr.b], ( *s_pGeomOut )[i] );
	if ( !g.special.speedBonus.empty() )
		( *s_pGeomOut )[i].bonus = g.special.speedBonus[pr.a];
}

// Arc generation of one tier: pair i fills slots [i * K, i * K + K).
enum { ARC_PER_COMBO = 5 };
static const std::vector<int> *s_pTierAlive;	// indices into the batch
static std::vector<ArcD> *s_pTierSlots;
static std::vector<unsigned char> *s_pTierCount;
static int s_nTierCombo;
static std::vector<ArcD> s_TierGen[MAX_TOOL_THREADS + 1];

static void TierArcsOne( int iThread, int i )
{
	int pi = ( *s_pTierAlive )[i];
	std::vector<ArcD> &gen = s_TierGen[iThread];
	gen.clear();
	ComboArcs( ( *s_pGeomOut )[pi], *s_pArcProfile, s_nTierCombo, gen );
	// an arc whose traced origin (raised, ArcTraceRaise) is inside or against an
	// obstacle is not tried on either path (see ARC_START_CLEAR)
	const std::vector<int> &near = ( *s_pLaunchObs )[( *s_pGeomPairs )[pi].ia];
	const double g = s_pArcProfile->gravity;
	int n = 0;
	for ( size_t k = 0; k < gen.size() && n < ARC_PER_COMBO; k++ )
	{
		const ArcD &a = gen[k];
		dvec3 o = a.start + dvec3( 0, 0, ArcTraceRaise( a, g ) );
		if ( !PointInsideOf( near, o, a.mesh, -ARC_START_CLEAR ) )
			( *s_pTierSlots )[(size_t)i * ARC_PER_COMBO + n++] = a;
	}
	( *s_pTierCount )[i] = (unsigned char)n;
}

static const std::vector<size_t> *s_pTierOffset;
static std::vector<ArcD> *s_pTierArcs;		// one of these two is set
static std::vector<BwArc> *s_pTierRaw;
static std::vector<int> *s_pTierArcPair;

static void TierCompactOne( int iThread, int i )
{
	size_t o = ( *s_pTierOffset )[i];
	int n = ( *s_pTierCount )[i], pi = ( *s_pTierAlive )[i];
	const ArcD *src = &( *s_pTierSlots )[(size_t)i * ARC_PER_COMBO];
	for ( int k = 0; k < n; k++ )
	{
		if ( s_pTierRaw )
			ToBwArc( src[k], s_pArcProfile->gravity, ( *s_pTierRaw )[o + k] );
		else
			( *s_pTierArcs )[o + k] = src[k];
		( *s_pTierArcPair )[o + k] = pi;
	}
}

// Compact the tier's slots (in parallel: hundreds of millions of arcs per round)
// into arcs, or into raw (GPU form) when raw is not NULL.
static void CompactTier( const std::vector<int> &alive, const std::vector<unsigned char> &count,
						 std::vector<size_t> &offset, std::vector<ArcD> *arcs, std::vector<BwArc> *raw,
						 std::vector<int> &arcPair )
{
	offset.resize( alive.size() + 1 );
	offset[0] = 0;
	for ( size_t i = 0; i < alive.size(); i++ )
		offset[i + 1] = offset[i] + count[i];
	size_t total = offset[alive.size()];
	if ( raw )
		raw->resize( total );
	else
		arcs->resize( total );
	arcPair.resize( total );
	s_pTierOffset = &offset;
	s_pTierArcs = arcs;
	s_pTierRaw = raw;
	s_pTierArcPair = &arcPair;
	RunThreadsBatched( (int)alive.size(), TierCompactOne );
	s_pTierRaw = NULL;
}

// An interior node on a slope launches only where moving down the slope, toward
// the target, loses more reach than it gains. To first order a launch dh lower
// reaches dh * vmax / v_fall less far, v_fall being the landing fall speed, and
// a step dx downhill is dx * tan(slope) lower. So the uphill node is needed
// only when tan(slope) * vmax > v_fall, that is for drops up to
// (tan(slope) * vmax)^2 / 2g - maxJumpRise. SLOPE_LAUNCH_SLACK widens that
// (first order, discrete launch points). Flat interior nodes never launch.
static const double SLOPE_LAUNCH_SLACK = 0.75;

static bool SlopeLaunchDrop( const WalkNode &A, const PlayerProfile &p, double &drop )
{
	double nz = A.plane.n.z;
	if ( nz >= 1.0 || A.poly.size() < 3 )
		return false;
	double tanSlope = sqrt( MAX( 0.0, 1.0 - nz * nz ) ) / nz;
	double vFall = tanSlope * p.maxAirSpeed * p.reachMargin / SLOPE_LAUNCH_SLACK;
	drop = vFall * vFall / ( 2.0 * p.gravity ) - p.maxJumpRise;
	if ( drop < 1.0 - p.maxJumpRise )
		return false;	// serves less than a 1u band of target heights below the apex
	drop = MIN( drop, p.arcMaxDrop );
	return true;
}

// -arccheck: counts and explains the arcs the two paths disagree on.
// ends( i, a, b ) names arc i's launch and target for the report.
template <class F>
static void CompareTraces( const CArcTracer &tracer, const std::vector<ArcD> &arcs, const std::vector<unsigned char> &gpu,
						   const std::vector<unsigned char> &cpu, ArcStats &st, int &reported, F ends )
{
	const int maxReport = 8;
	st.checkArcs += (long long)arcs.size();
	for ( size_t i = 0; i < arcs.size(); i++ )
	{
		if ( gpu[i] == cpu[i] )
			continue;
		if ( gpu[i] )
			st.gpuOnlyBlocked++;
		else
			st.cpuOnlyBlocked++;
		double depth = tracer.Depth( arcs[i] );
		double dist = fabs( depth );
		st.checkMaxDist = MAX( st.checkMaxDist, dist );
		if ( dist > ARC_FLOAT_BOUND )
		{
			st.unexplained++;
			if ( reported++ < maxReport )
			{
				char a[32], b[32];
				ends( i, a, b );
				Warning( "bspwalk: arccheck: arc %s -> %s %s on the GPU, %s on the CPU, grazing distance %.6f "
						 "(start %.3f %.3f %.3f, v %.3f %.3f %.3f, T %.4f, mesh %d)\n",
						 a, b, gpu[i] ? "blocked" : "free", cpu[i] ? "blocked" : "free", depth, arcs[i].start.x,
						 arcs[i].start.y, arcs[i].start.z, arcs[i].vx, arcs[i].vy, arcs[i].vz, arcs[i].tEnd, arcs[i].mesh );
			}
		}
	}
}

//-----------------------------------------------------------------------------
// Special launches (#13B): each special vertex reached since the last round
// that carries a launcher (bw_special.cpp: ladders, ladder brushes, push
// entries, catapults, teleport arrivals) pairs with every unreached node within
// its reach and below its highest flight point. Per pair, like a node pair:
// landing points T0 (B's boundary point nearest the launcher, inset), T1 (B's
// centroid) and T2 (B's vertex farthest from the launcher, inset); launch
// points the launcher's point nearest each landing point and its highest point
// (point launchers), or the volume boundary point nearest each landing point
// and the nearest on a face toward exitDir (volume launchers); every launch
// point with every landing point, and per combination each velocity family:
//   plain families  SolveArcBase (an exact landing, horizontal velocity within
//                   hmax of the family's base). With apexVz the player may leave
//                   at any vz up to the family's: the vz peaking at the landing
//                   height, 0, and the vz whose flight time carries the base
//                   velocity to the landing are tried too;
//   push families   SolvePushArc: under the push's gravity up to its top (or
//                   the apex there) and then ballistic, the same exiting half
//                   way up, and the whole flight inside the push.
// The first free arc of a pair reaches B. Arcs are generated per thread and
// compacted, so memory follows the arcs a pair really has.
//-----------------------------------------------------------------------------
static const int SP_PAIR_BATCH = 64 * 1024;
enum { SP_MAX_STARTS = 12, SP_MAX_LANDS = 3 };
struct SpPair
{
	int launcher, b;
};
static const std::vector<SpPair> *s_pSpPairs;
static std::vector<ArcD> s_SpGen[MAX_TOOL_THREADS + 1];
static std::vector<int> s_SpGenPair[MAX_TOOL_THREADS + 1];

static void AddPoint( dvec3 *pts, int &n, int maxPts, const dvec3 &p )
{
	for ( int i = 0; i < n; i++ )
	{
		if ( DDot( pts[i] - p, pts[i] - p ) < 1e-6 )
			return;
	}
	if ( n < maxPts )
		pts[n++] = p;
}

static void SpecialStarts( const SpecialLauncher &L, const dvec3 *land, int nLand, dvec3 *starts, int &ns )
{
	ns = 0;
	if ( !L.points.empty() )
	{
		int iHigh = 0;
		for ( size_t i = 1; i < L.points.size(); i++ )
		{
			if ( L.points[i].z > L.points[iHigh].z )
				iHigh = (int)i;
		}
		for ( int t = 0; t < nLand; t++ )
		{
			int iNear = 0;
			double bd = 1e300;
			for ( size_t i = 0; i < L.points.size(); i++ )
			{
				double d = DDot( L.points[i] - land[t], L.points[i] - land[t] );
				if ( d < bd )
				{
					bd = d;
					iNear = (int)i;
				}
			}
			AddPoint( starts, ns, SP_MAX_STARTS, L.points[iNear] );
		}
		AddPoint( starts, ns, SP_MAX_STARTS, L.points[iHigh] );
	}
	if ( L.volumes.empty() )
		return;
	DWinding face;
	for ( int t = 0; t < nLand; t++ )
	{
		const dvec3 &T = land[t];
		dvec3 best, bestExit;
		double bd = 1e300, be = 1e300;
		for ( size_t v = 0; v < L.volumes.size(); v++ )
		{
			const GrownPolytope &q = L.volumes[v];
			for ( size_t f = 0; f < q.faces.size(); f++ )
			{
				face.clear();
				for ( size_t k = 0; k < q.faces[f].size(); k++ )
					face.push_back( q.verts[q.faces[f][k]] );
				dvec3 c = ClosestPointOnPolygon( face, T );
				double d = DDot( c - T, c - T );
				if ( d < bd )
				{
					bd = d;
					best = c;
				}
				if ( L.hasExitDir && DDot( q.planes[f].n, L.exitDir ) >= 0.5 && d < be )
				{
					be = d;
					bestExit = c;
				}
			}
		}
		if ( bd < 1e300 )
			AddPoint( starts, ns, SP_MAX_STARTS, best );
		if ( be < 1e300 )
			AddPoint( starts, ns, SP_MAX_STARTS, bestExit );
	}
}

// The traced origin of an arc is clear of every obstacle (ARC_START_CLEAR).
static bool ArcStartClear( int iThread, const ArcD &a, double g )
{
	Assert( a.mesh == BW_MESH_CROUCH );
	const ObstacleSet &obs = s_ArcObs[a.mesh];
	std::vector<int> &cand = s_ArcCand[iThread];
	dvec3 o = a.start + dvec3( 0, 0, ArcTraceRaise( a, g ) );
	const double m = ARC_START_CLEAR + 1e-3;
	cand.clear();
	obs.hash.Query( o.x - m, o.y - m, o.x + m, o.y + m, cand );
	return !PointInsideOf( cand, o, a.mesh, -ARC_START_CLEAR );
}

// Appends the arcs of the pair (launcher L, node B) to out.
static void SpecialPairArcs( int iThread, const SpecialLauncher &L, const WalkNode &B, const PlayerProfile &p,
							 std::vector<ArcD> &out )
{
	const double g = p.gravity;
	double cbx, cby;
	Centroid( B.poly, cbx, cby );
	const double lx = 0.5 * ( L.mins.x + L.maxs.x ), ly = 0.5 * ( L.mins.y + L.maxs.y );
	double t0x, t0y;
	BoundaryNearest( B.poly, lx, ly, t0x, t0y );
	double far = -1, t2x = cbx, t2y = cby;
	for ( size_t i = 0; i < B.poly.size(); i++ )
	{
		double d = ( B.poly[i].x - lx ) * ( B.poly[i].x - lx ) + ( B.poly[i].y - ly ) * ( B.poly[i].y - ly );
		if ( d > far )
		{
			far = d;
			t2x = B.poly[i].x;
			t2y = B.poly[i].y;
		}
	}
	dvec3 land[SP_MAX_LANDS];
	int nLand = 0;
	AddPoint( land, nLand, SP_MAX_LANDS, Inset( B, t0x, t0y, cbx, cby ) );
	AddPoint( land, nLand, SP_MAX_LANDS, OnNode( B, cbx, cby ) );
	AddPoint( land, nLand, SP_MAX_LANDS, Inset( B, t2x, t2y, cbx, cby ) );
	dvec3 starts[SP_MAX_STARTS];
	int ns;
	SpecialStarts( L, land, nLand, starts, ns );
	for ( int ti = 0; ti < nLand; ti++ )
	{
		const dvec3 &T = land[ti];
		for ( int si = 0; si < ns; si++ )
		{
			for ( size_t fi = 0; fi < L.vel.size(); fi++ )
			{
				const LaunchVel &f = L.vel[fi];
				dvec3 S = starts[si] + dvec3( 0, 0, f.lift );
				ArcD cands[4];
				int nc = 0;
				if ( f.push )
				{
					double zx = L.pushTop;
					double vzUp = MAX( 0.0, f.vz );
					if ( f.gIn > 1e-9 )
						zx = MIN( zx, S.z + vzUp * vzUp / ( 2.0 * f.gIn ) );
					double exits[3];
					int ne = 0;
					if ( zx > S.z + 1e-3 )
					{
						exits[ne++] = zx;
						if ( zx - S.z > 2.0 )
							exits[ne++] = S.z + 0.5 * ( zx - S.z );
					}
					if ( f.gIn > 1e-9 )
						exits[ne++] = -1e300;	// the whole flight inside the push
					for ( int k = 0; k < ne; k++ )
					{
						if ( SolvePushArc( S, f.vz, f.gIn, exits[k], f.bx, f.by, g, T, f.hmax, BW_MESH_CROUCH, cands[nc] ) )
							nc++;
					}
				}
				else
				{
					double dz = T.z - S.z;
					double vzs[4];
					int nv = 0;
					vzs[nv++] = f.vz;
					if ( f.apexVz )
					{
						if ( dz > 0 )
						{
							double va = sqrt( 2.0 * g * dz );
							if ( va < f.vz )
								vzs[nv++] = va;
						}
						else if ( f.vz > 0 )
							vzs[nv++] = 0.0;
						double bb = f.bx * f.bx + f.by * f.by;
						if ( bb > 1e-12 )
						{
							double Ts = ( ( T.x - S.x ) * f.bx + ( T.y - S.y ) * f.by ) / bb;
							if ( Ts > 1e-3 )
							{
								double vi = ( dz + 0.5 * g * Ts * Ts ) / Ts;
								if ( vi >= 0 && vi < f.vz )
									vzs[nv++] = vi;
							}
						}
					}
					for ( int k = 0; k < nv; k++ )
					{
						if ( SolveArcBase( S, vzs[k], f.bx, f.by, g, T, f.hmax, BW_MESH_CROUCH, cands[nc] ) )
							nc++;
					}
				}
				for ( int k = 0; k < nc; k++ )
				{
					ArcD &a = cands[k];
					a.start.z += ARC_LIFT;	// start and landing alike (see ARC_LIFT)
					if ( ArcStartClear( iThread, a, g ) )
						out.push_back( a );
				}
			}
		}
	}
}

static void SpecialArcsOne( int iThread, int i )
{
	const SpPair &pr = ( *s_pSpPairs )[i];
	const WalkGraph &g = *s_pArcGraph;
	std::vector<ArcD> &gen = s_SpGen[iThread];
	SpecialPairArcs( iThread, g.special.launchers[pr.launcher], g.nodes[pr.b], *s_pArcProfile, gen );
	s_SpGenPair[iThread].resize( gen.size(), i );	// the new arcs are pair i's
}

// Gathers the per-thread arcs of a generation pass into arcs / arcPair.
static void GatherGen( std::vector<ArcD> *gen, std::vector<int> *genPair, std::vector<ArcD> &arcs,
					   std::vector<int> &arcPair )
{
	arcs.clear();
	arcPair.clear();
	size_t total = 0;
	for ( int t = 0; t <= MAX_TOOL_THREADS; t++ )
		total += gen[t].size();
	arcs.reserve( total );
	arcPair.reserve( total );
	for ( int t = 0; t <= MAX_TOOL_THREADS; t++ )
	{
		arcs.insert( arcs.end(), gen[t].begin(), gen[t].end() );
		arcPair.insert( arcPair.end(), genPair[t].begin(), genPair[t].end() );
		gen[t].clear();
		genPair[t].clear();
	}
}

// Traces arcs on the GPU, the CPU, or both (-arccheck: compared, the flood
// follows the CPU). blocked[i] = 1 when arc i is blocked.
template <class F>
static void TraceArcs( CArcTracer &tracer, const std::vector<ArcD> &arcs, bool &useGpu, bool &check, ArcStats &st,
					   int &reported, std::vector<unsigned char> &blocked, std::vector<unsigned char> &blockedCpu, F ends )
{
	bool gpuOk = false;
	if ( useGpu )
	{
		gpuOk = tracer.TraceGpu( arcs, blocked );
		if ( !gpuOk )
		{
			Warning( "bspwalk: GPU arc trace failed; continuing on the CPU\n" );
			useGpu = check = false;
			st.path = "cpu";
		}
	}
	if ( !gpuOk || check )
		tracer.TraceCpu( arcs, gpuOk ? blockedCpu : blocked );
	if ( check && gpuOk )
	{
		CompareTraces( tracer, arcs, blocked, blockedCpu, st, reported, ends );
		blocked.swap( blockedCpu );	// the flood follows the CPU reference
	}
}

static void SpecialLaunchRound( const std::vector<int> &launchers, WalkGraph &graph, const PlayerProfile &profile,
								CArcTracer &tracer, bool &useGpu, bool &check, ArcStats &st, int &reported,
								IArcFloodSink &sink )
{
	double t0 = Plat_FloatTime();
	SpecialStats &ss = graph.special.stats;
	std::vector<SpPair> pairs;
	std::vector<int> cand;
	for ( size_t li = 0; li < launchers.size(); li++ )
	{
		const SpecialLauncher &L = graph.special.launchers[launchers[li]];
		double zTop = L.zTop + 1e-6;
		cand.clear();
		s_UnreachedHash.Query( L.mins.x - L.reach, L.mins.y - L.reach, L.maxs.x + L.reach, L.maxs.y + L.reach, cand );
		for ( size_t k = 0; k < cand.size(); k++ )
		{
			int b = cand[k];
			const WalkNode &B = graph.nodes[b];
			if ( graph.reached[b] || B.poly.size() < 3 || B.mins.z > zTop )
				continue;
			double dx = MAX( 0.0, MAX( B.mins.x - L.maxs.x, L.mins.x - B.maxs.x ) );
			double dy = MAX( 0.0, MAX( B.mins.y - L.maxs.y, L.mins.y - B.maxs.y ) );
			if ( dx * dx + dy * dy > L.reach * L.reach )
				continue;
			if ( L.sweepT > 0 )
			{
				// within sweepOwn of the launch box swept along b t: bounded by the
				// centre B to the swept centre segment, less both boxes' half diagonals
				double hx = 0.5 * ( L.maxs.x - L.mins.x ), hy = 0.5 * ( L.maxs.y - L.mins.y );
				double bx = 0.5 * ( B.maxs.x - B.mins.x ), by = 0.5 * ( B.maxs.y - B.mins.y );
				double px = 0.5 * ( B.mins.x + B.maxs.x ) - ( L.mins.x + hx ), py = 0.5 * ( B.mins.y + B.maxs.y ) - ( L.mins.y + hy );
				double sx = L.sweepBx * L.sweepT, sy = L.sweepBy * L.sweepT;
				double u = MAX( 0.0, MIN( 1.0, ( px * sx + py * sy ) / ( sx * sx + sy * sy ) ) );
				double ex = px - u * sx, ey = py - u * sy;
				double lim = L.sweepOwn + sqrt( hx * hx + hy * hy ) + sqrt( bx * bx + by * by );
				if ( ex * ex + ey * ey > lim * lim )
					continue;
			}
			SpPair pr = { launchers[li], b };
			pairs.push_back( pr );
		}
	}
	ss.launchPairs += (long long)pairs.size();

	std::vector<SpPair> batch;
	std::vector<ArcD> arcs;
	std::vector<unsigned char> blocked, blockedCpu, found;
	std::vector<int> arcPair;
	for ( size_t p0 = 0; p0 < pairs.size(); p0 += SP_PAIR_BATCH )
	{
		size_t p1 = MIN( pairs.size(), p0 + (size_t)SP_PAIR_BATCH );
		batch.clear();
		for ( size_t i = p0; i < p1; i++ )
		{
			if ( !graph.reached[pairs[i].b] )
				batch.push_back( pairs[i] );
		}
		if ( batch.empty() )
			continue;
		s_pSpPairs = &batch;
		RunThreadsBatched( (int)batch.size(), SpecialArcsOne );
		s_pSpPairs = NULL;
		GatherGen( s_SpGen, s_SpGenPair, arcs, arcPair );
		if ( arcs.empty() )
			continue;

		TraceArcs( tracer, arcs, useGpu, check, st, reported, blocked, blockedCpu, [&]( size_t i, char *a, char *b ) {
			const SpPair &pr = batch[arcPair[i]];
			Q_snprintf( a, 32, "launcher %d", pr.launcher );
			Q_snprintf( b, 32, "%d", pr.b );
		} );

		ss.launchArcs += (long long)arcs.size();
		found.assign( batch.size(), 0 );
		for ( size_t i = 0; i < arcs.size(); i++ )
		{
			if ( blocked[i] )
				continue;
			ss.launchFree++;
			int pi = arcPair[i];
			if ( found[pi] )
				continue;
			found[pi] = 1;
			ss.launchJoined++;
			graph.freeArcs.push_back( BwArc() );
			ToBwArc( arcs[i], profile.gravity, graph.freeArcs.back() );
			sink.Reach( batch[pi].b );
		}
		sink.Propagate();
	}
	ss.tLaunch += Plat_FloatTime() - t0;
}

//-----------------------------------------------------------------------------
// Trigger targets (#13B): a trigger is also fired by an arc that only passes
// through it. The points sampled inside each trigger volume (bw_special.cpp)
// are arc targets of the round's launch nodes, under the node pairs' rules:
// at most max_jump_rise above the node and within R(drop) of it; launch points
// the node's boundary point nearest the target (exact for a walk-off, inset for
// the jumps) and, when the target is above the node, the node's point below it;
// the duck-jump, the jump (with air_jumps also with a second jump at the apex)
// and the walk-off (only down; a slide only walks off). The arc ends at the
// target, so it is traced only that far. A free arc reaches the target's vertex.
//-----------------------------------------------------------------------------
struct TgPair
{
	int a, t;
};
static const std::vector<int> *s_pTgLaunch;
static const std::vector<int> *s_pTgLive;	// target indices whose vertex is unreached
static CXYHash s_TargetHash;
static std::vector<ArcD> s_TgGen[MAX_TOOL_THREADS + 1];
static std::vector<int> s_TgGenPair[MAX_TOOL_THREADS + 1];
static std::vector<TgPair> s_TgPairs[MAX_TOOL_THREADS + 1];

static bool InsidePolyXY( const DWinding &w, double x, double y )
{
	int n = (int)w.size(), sign = 0;
	for ( int i = 0; i < n; i++ )
	{
		const dvec3 &a = w[i], &b = w[( i + 1 ) % n];
		double c = ( b.x - a.x ) * ( y - a.y ) - ( b.y - a.y ) * ( x - a.x );
		int sc = c > 1e-9 ? 1 : ( c < -1e-9 ? -1 : 0 );
		if ( !sc )
			continue;
		if ( sign && sc != sign )
			return false;
		sign = sc;
	}
	return n >= 3;
}

static void TargetArcsOne( int iThread, int i )
{
	const WalkGraph &g = *s_pArcGraph;
	const PlayerProfile &p = *s_pArcProfile;
	const std::vector<SpecialTarget> &targets = g.special.targets;
	int a = ( *s_pTgLaunch )[i];
	const WalkNode &A = g.nodes[a];
	if ( A.poly.size() < 3 )
		return;
	const double bonus = g.special.speedBonus.empty() ? 0.0 : g.special.speedBonus[a];
	const double vmax = p.maxAirSpeed * p.reachMargin + bonus;
	const double bonusScale = 1.0 + bonus / ( p.maxAirSpeed * p.reachMargin );
	const double Rmax = p.arcReachMax * bonusScale, gr = p.gravity, vj = p.jumpVelocity;
	const int maxKick = A.slide ? -1 : ( p.airJumps > 0 ? 1 : 0 );
	std::vector<int> cand;
	s_TargetHash.Query( A.mins.x - Rmax, A.mins.y - Rmax, A.maxs.x + Rmax, A.maxs.y + Rmax, cand );
	std::vector<ArcD> &gen = s_TgGen[iThread];
	double cax, cay;
	Centroid( A.poly, cax, cay );
	for ( size_t k = 0; k < cand.size(); k++ )
	{
		int ti = ( *s_pTgLive )[cand[k]];
		const SpecialTarget &t = targets[ti];
		const dvec3 &P = t.p;
		if ( g.special.reached[t.vertex] || P.z > A.maxs.z + p.maxJumpRise + 1e-6 )
			continue;
		if ( A.slide && P.z >= A.maxs.z )
			continue;
		double dx = MAX( 0.0, MAX( P.x - A.maxs.x, A.mins.x - P.x ) );
		double dy = MAX( 0.0, MAX( P.y - A.maxs.y, A.mins.y - P.y ) );
		double R = p.ArcReachAtDrop( A.maxs.z - P.z ) * bonusScale;
		if ( dx * dx + dy * dy > R * R )
			continue;
		TgPair pr = { a, ti };
		s_TgPairs[iThread].push_back( pr );
		int pairIndex = (int)s_TgPairs[iThread].size() - 1;

		double lx, ly;
		BoundaryNearest( A.poly, P.x, P.y, lx, ly );
		dvec3 launch[2];
		int nl = 0;
		launch[nl++] = Inset( A, lx, ly, cax, cay );
		if ( InsidePolyXY( A.poly, P.x, P.y ) )
			launch[nl++] = OnNode( A, P.x, P.y );
		const dvec3 E = OnNode( A, lx, ly );
		ArcD c[2 * 2 * 2 + 1];	// launch points x kicks x (duck-jump, jump), and the walk-off
		int nc = 0;
		for ( int l = 0; l < nl; l++ )
		{
			for ( int kick = 0; kick <= maxKick; kick++ )
			{
				if ( SolveArcToPoint( launch[l] + dvec3( 0, 0, p.crouchLift ), vj, kick != 0, gr, P, vmax, BW_MESH_CROUCH, c[nc] ) )
					nc++;
				if ( SolveArcToPoint( launch[l], vj, kick != 0, gr, P, vmax, BW_MESH_CROUCH, c[nc] ) )
					nc++;
			}
		}
		if ( P.z < E.z - 1e-6 && SolveArcToPoint( E, 0.0, false, gr, P, vmax, BW_MESH_CROUCH, c[nc] ) )
			nc++;
		for ( int j = 0; j < nc; j++ )
		{
			c[j].start.z += ARC_LIFT;
			if ( ArcStartClear( iThread, c[j], gr ) )
			{
				gen.push_back( c[j] );
				s_TgGenPair[iThread].push_back( pairIndex );
			}
		}
	}
}

static void TriggerTargetRound( const std::vector<int> &launch, WalkGraph &graph, const PlayerProfile &profile,
								CArcTracer &tracer, bool &useGpu, bool &check, ArcStats &st, int &reported,
								IArcFloodSink &sink )
{
	const std::vector<SpecialTarget> &targets = graph.special.targets;
	if ( targets.empty() || launch.empty() )
		return;
	double t0 = Plat_FloatTime();
	SpecialStats &ss = graph.special.stats;
	std::vector<int> live;
	std::vector<XYBox> boxes;
	for ( size_t i = 0; i < targets.size(); i++ )
	{
		if ( graph.special.reached[targets[i].vertex] )
			continue;
		live.push_back( (int)i );
		XYBox b = { targets[i].p.x, targets[i].p.y, targets[i].p.x, targets[i].p.y };
		boxes.push_back( b );
	}
	if ( live.empty() )
		return;
	s_TargetHash.Build( boxes, std::vector<unsigned char>( boxes.size(), 1 ) );
	s_pTgLive = &live;
	const int nNodes = (int)graph.nodes.size();
	std::vector<ArcD> arcs;
	std::vector<int> arcPair;
	std::vector<TgPair> pairs;
	std::vector<unsigned char> blocked, blockedCpu;
	// in chunks of launch nodes (bounds the per-thread arc lists)
	for ( size_t first = 0; first < launch.size(); first += ARC_CHUNK_A )
	{
		std::vector<int> chunk( launch.begin() + first, launch.begin() + MIN( launch.size(), first + (size_t)ARC_CHUNK_A ) );
		s_pTgLaunch = &chunk;
		RunThreadsBatched( (int)chunk.size(), TargetArcsOne );
		s_pTgLaunch = NULL;
		// per-thread pair indices become global ones
		pairs.clear();
		arcs.clear();
		arcPair.clear();
		for ( int t = 0; t <= MAX_TOOL_THREADS; t++ )
		{
			int base = (int)pairs.size();
			pairs.insert( pairs.end(), s_TgPairs[t].begin(), s_TgPairs[t].end() );
			arcs.insert( arcs.end(), s_TgGen[t].begin(), s_TgGen[t].end() );
			for ( size_t k = 0; k < s_TgGenPair[t].size(); k++ )
				arcPair.push_back( base + s_TgGenPair[t][k] );
			s_TgPairs[t].clear();
			s_TgGen[t].clear();
			s_TgGenPair[t].clear();
		}
		ss.targetPairs += (long long)pairs.size();
		if ( arcs.empty() )
			continue;
		TraceArcs( tracer, arcs, useGpu, check, st, reported, blocked, blockedCpu, [&]( size_t i, char *a, char *b ) {
			const TgPair &pr = pairs[arcPair[i]];
			Q_snprintf( a, 32, "%d", pr.a );
			Q_snprintf( b, 32, "target %d", pr.t );
		} );
		ss.targetArcs += (long long)arcs.size();
		for ( size_t i = 0; i < arcs.size(); i++ )
		{
			if ( blocked[i] )
				continue;
			ss.targetFree++;
			int v = targets[pairs[arcPair[i]].t].vertex;
			if ( graph.special.reached[v] )
				continue;
			ss.targetJoined++;
			graph.freeArcs.push_back( BwArc() );
			ToBwArc( arcs[i], profile.gravity, graph.freeArcs.back() );
			sink.Reach( nNodes + v );
		}
		sink.Propagate();
	}
	s_pTgLive = NULL;
	s_TargetHash = CXYHash();
	ss.tTargets += Plat_FloatTime() - t0;
}

void RunArcRounds( const CollisionWorld &world, const PlayerProfile &profile, const ArcOptions &opts, WalkGraph &graph,
				   IArcFloodSink &sink )
{
	double tStart = Plat_FloatTime();
	// the shared thread pool draws a pacifier per call; arcs make dozens of calls
	SuppressPacifier( true );
	ArcStats &st = graph.arcs;
	st.reach = profile.arcReach;
	st.reachMax = profile.arcReachMax;
	st.allNodes = opts.allNodes;

	CArcTracer tracer;
	bool wantGpu = !opts.cpuOnly;
	tracer.Init( world, profile, wantGpu );
	if ( wantGpu && !tracer.HasGpu() )
		Warning( "bspwalk: no GPU arc path (CUDA device, OptiX or bspwalk_arcs.ptx missing); tracing arcs on the CPU\n" );
	bool useGpu = tracer.HasGpu();
	bool check = opts.check && useGpu;
	if ( opts.check && !useGpu )
		Warning( "bspwalk: -arccheck needs the GPU path; arcs are traced on the CPU only\n" );
	st.path = useGpu ? "gpu" : "cpu";
	st.checked = check;
	st.tBuild = tracer.m_tBuild;
	for ( int m = 0; m < 2; m++ )	// the arc meshes (stand, crouch)
	{
		st.meshTris[m] = tracer.m_nTris[m];
		st.gasMB[m] = tracer.m_gasBytes[m] / ( 1024.0 * 1024.0 );
	}
	if ( useGpu )
		Msg( "arcs: OptiX on %s, meshes %d / %d triangles (stand / crouch), GAS %.1f / %.1f MB, built in %.2f s\n",
			 BwOptix_DeviceName(), st.meshTris[0], st.meshTris[1], st.gasMB[0], st.gasMB[1], st.tBuild );

	s_pArcGraph = &graph;
	s_pArcProfile = &profile;
	const int n = (int)graph.nodes.size();
	std::vector<XYBox> boxes( n );
	for ( int i = 0; i < n; i++ )
	{
		boxes[i].x0 = graph.nodes[i].mins.x;
		boxes[i].y0 = graph.nodes[i].mins.y;
		boxes[i].x1 = graph.nodes[i].maxs.x;
		boxes[i].y1 = graph.nodes[i].maxs.y;
	}

	// launch nodes: rim nodes and slope launchers, unless -arcallnodes
	std::vector<unsigned char> rim;
	std::vector<double> slopeDrop( n, 0.0 );
	double tRim = Plat_FloatTime();
	ComputeRimNodes( graph, rim );
	for ( int i = 0; i < n; i++ )
	{
		if ( graph.nodes[i].slide )
			rim[i] = LAUNCH_RIM;	// a slide's walk-offs reach targets below it only
		if ( rim[i] )
		{
			st.rimNodes++;
			continue;
		}
		if ( SlopeLaunchDrop( graph.nodes[i], profile, slopeDrop[i] ) )
		{
			rim[i] = LAUNCH_SLOPE;
			st.slopeNodes++;
		}
	}
	tRim = Plat_FloatTime() - tRim;
	st.tGen += tRim;
	Msg( "arcs: %d rim nodes and %d slope launchers of %d nodes, found in %.2f s; launching from %s\n", st.rimNodes,
		 st.slopeNodes, n, tRim, opts.allNodes ? "all nodes (-arcallnodes)" : "those" );
	if ( opts.allNodes )
		rim.assign( n, LAUNCH_RIM );
	s_pArcRim = &rim;
	s_pArcSlopeDrop = &slopeDrop;

	std::vector<ArcPair> pairs, batch;
	std::vector<PairPoints> geom;
	std::vector<unsigned char> found, blocked, blockedCpu, count;
	std::vector<ArcD> slots, arcs;
	std::vector<BwArc> rawArcs;
	std::vector<int> arcPair, alive;
	std::vector<size_t> offset;
	std::vector< std::vector<int> > targets, launchObs;
	size_t done = 0, sDone = 0;
	int reported = 0;
	// Each round: the special launchers reached since the last one, then the
	// node launches of the nodes reached since the last one.
	for ( ;; )
	{
		const std::vector<int> &order = sink.Order();
		const std::vector<int> &sorder = sink.SpecialOrder();
		size_t end = order.size(), sEnd = sorder.size();
		if ( done == end && sDone == sEnd )
			break;
		std::vector<int> spLaunch;
		for ( size_t i = sDone; i < sEnd; i++ )
		{
			int l = graph.special.verts[sorder[i]].launcher;
			if ( l >= 0 )
				spLaunch.push_back( l );
		}
		sDone = sEnd;
		std::vector<int> launch;
		for ( size_t i = done; i < end; i++ )
		{
			if ( rim[order[i]] )
				launch.push_back( order[i] );
		}
		ArcRoundStats rs;
		memset( &rs, 0, sizeof( rs ) );
		rs.frontier = (int)( end - done );
		rs.launch = (int)launch.size();
		done = end;

		std::vector<unsigned char> use( n );
		int unreached = 0;
		for ( int i = 0; i < n; i++ )
		{
			use[i] = !graph.reached[i];
			unreached += use[i];
		}
		if ( !unreached )
			break;
		double tr = Plat_FloatTime();
		s_UnreachedHash.Build( boxes, use );
		rs.tGen += Plat_FloatTime() - tr;
		if ( !spLaunch.empty() )
			SpecialLaunchRound( spLaunch, graph, profile, tracer, useGpu, check, st, reported, sink );

		const int nChunks = ( (int)launch.size() + ARC_CHUNK_A - 1 ) / ARC_CHUNK_A;
		for ( int first = 0; first < (int)launch.size(); first += ARC_CHUNK_A )
		{
			// pairs of this chunk of launch nodes
			double tg = Plat_FloatTime();
			const ArcRoundStats before = rs;
			int count_a = MIN( ARC_CHUNK_A, (int)launch.size() - first );
			targets.resize( count_a );
			s_pArcLaunch = &launch;
			s_nArcChunkFirst = first;
			s_pArcTargets = &targets;
			RunThreadsBatched( count_a, PairsOne );
			launchObs.resize( count_a );
			s_pLaunchObs = &launchObs;
			RunThreadsBatched( count_a, LaunchObsOne );
			pairs.clear();
			for ( int i = 0; i < count_a; i++ )
			{
				for ( size_t k = 0; k < targets[i].size(); k++ )
				{
					ArcPair pr = { launch[first + i], targets[i][k], i };
					pairs.push_back( pr );
				}
			}
			rs.pairs += (long long)pairs.size();
			rs.tGen += Plat_FloatTime() - tg;

			// in batches of pairs (bounds the geometry cache); a pair whose B an
			// earlier batch or chunk of this round reached is dropped
			for ( size_t p0 = 0; p0 < pairs.size(); p0 += ARC_PAIR_BATCH )
			{
				tg = Plat_FloatTime();
				size_t p1 = MIN( pairs.size(), p0 + (size_t)ARC_PAIR_BATCH );
				batch.clear();
				for ( size_t i = p0; i < p1; i++ )
				{
					if ( !graph.reached[pairs[i].b] )
						batch.push_back( pairs[i] );
				}
				if ( batch.empty() )
					continue;
				geom.resize( batch.size() );
				s_pGeomPairs = &batch;
				s_pGeomOut = &geom;
				RunThreadsBatched( (int)batch.size(), PairGeomOne );
				found.assign( batch.size(), 0 );
				rs.tGen += Plat_FloatTime() - tg;

				for ( int combo = 0; combo < ARC_COMBOS; combo++ )
				{
					tg = Plat_FloatTime();
					alive.clear();
					for ( size_t i = 0; i < batch.size(); i++ )
					{
						if ( !found[i] && !graph.reached[batch[i].b] )
							alive.push_back( (int)i );
					}
					if ( alive.empty() )
						break;
					if ( slots.size() < alive.size() * ARC_PER_COMBO )
						slots.resize( alive.size() * ARC_PER_COMBO );
					count.assign( alive.size(), 0 );
					s_pTierAlive = &alive;
					s_pTierSlots = &slots;
					s_pTierCount = &count;
					s_nTierCombo = combo;
					RunThreadsBatched( (int)alive.size(), TierArcsOne );
					// the GPU-only path never needs the double arcs
					bool raw = useGpu && !check;
					CompactTier( alive, count, offset, &arcs, raw ? &rawArcs : NULL, arcPair );
					size_t nArcs = arcPair.size();
					rs.tGen += Plat_FloatTime() - tg;
					if ( !nArcs )
						continue;

					double tt = Plat_FloatTime();
					bool gpuOk = false;
					if ( useGpu )
					{
						gpuOk = raw ? tracer.TraceGpuRaw( rawArcs.data(), nArcs, blocked ) : tracer.TraceGpu( arcs, blocked );
						if ( !gpuOk )
						{
							Warning( "bspwalk: GPU arc trace failed; continuing on the CPU\n" );
							useGpu = check = false;
							st.path = "cpu";
							if ( raw )
								CompactTier( alive, count, offset, &arcs, NULL, arcPair );
						}
					}
					if ( !gpuOk || check )
						tracer.TraceCpu( arcs, gpuOk ? blockedCpu : blocked );
					rs.tTrace += Plat_FloatTime() - tt;

					if ( check && gpuOk )
					{
						CompareTraces( tracer, arcs, blocked, blockedCpu, st, reported, [&]( size_t i, char *a, char *b ) {
							Q_snprintf( a, 32, "%d", batch[arcPair[i]].a );
							Q_snprintf( b, 32, "%d", batch[arcPair[i]].b );
						} );
						blocked.swap( blockedCpu );	// the flood follows the CPU reference
					}

					rs.arcs += (long long)nArcs;
					for ( size_t i = 0; i < nArcs; i++ )
					{
						if ( blocked[i] )
							continue;
						rs.arcsFree++;
						int pi = arcPair[i];
						if ( found[pi] )
							continue;
						found[pi] = 1;
						rs.pairsFree++;
						WalkEdge e = { batch[pi].a, batch[pi].b, EDGE_ARC };
						graph.edges.push_back( e );
						if ( raw && i < rawArcs.size() )
							graph.freeArcs.push_back( rawArcs[i] );
						else if ( i < arcs.size() )
						{
							graph.freeArcs.push_back( BwArc() );
							ToBwArc( arcs[i], profile.gravity, graph.freeArcs.back() );
						}
						sink.Reach( batch[pi].b );
					}
					sink.Propagate();
				}
			}
			if ( nChunks > 1 )	// progress on large maps
				Msg( "arcs: round %d, chunk %d of %d: %lld pairs, %lld arcs (gen %.2f s, trace %.2f s)\n", st.rounds + 1,
					 first / ARC_CHUNK_A + 1, nChunks, rs.pairs - before.pairs, rs.arcs - before.arcs, rs.tGen - before.tGen,
					 rs.tTrace - before.tTrace );
		}
		// triggers crossed in flight by arcs from this round's launch nodes
		TriggerTargetRound( launch, graph, profile, tracer, useGpu, check, st, reported, sink );

		rs.reached = (int)( sink.Order().size() - end );
		st.perRound.push_back( rs );
		st.rounds++;
		st.pairs += rs.pairs;
		st.arcs += rs.arcs;
		st.arcsFree += rs.arcsFree;
		st.pairsFree += rs.pairsFree;
		st.tGen += rs.tGen;
		st.tTrace += rs.tTrace;
		Msg( "arcs: round %d: %d new nodes, %d launch, %lld pairs, %lld arcs (%lld free), %lld pairs joined, %d nodes "
			 "reached (gen %.2f s, trace %.2f s)\n",
			 st.rounds, rs.frontier, rs.launch, rs.pairs, rs.arcs, rs.arcsFree, rs.pairsFree, rs.reached, rs.tGen,
			 rs.tTrace );
		if ( !rs.reached && sink.SpecialOrder().size() == sEnd )
			break;	// nothing new: no node, and no special vertex whose launcher would run
	}
	if ( check )
		Msg( "arccheck: %lld arcs on both paths: %lld blocked only on the GPU, %lld only on the CPU, max grazing distance "
			 "%.6f (float bound %.6f), %lld unexplained\n",
			 st.checkArcs, st.gpuOnlyBlocked, st.cpuOnlyBlocked, st.checkMaxDist, ARC_FLOAT_BOUND, st.unexplained );
	tracer.Shutdown();
	SuppressPacifier( false );
	s_pArcGraph = NULL;
	s_pArcProfile = NULL;
	s_pArcRim = NULL;
	s_pArcSlopeDrop = NULL;
	s_pArcLaunch = NULL;
	s_pArcTargets = NULL;
	s_pLaunchObs = NULL;
	s_pGeomPairs = NULL;
	s_pGeomOut = NULL;
	s_pTierAlive = NULL;
	s_pTierSlots = NULL;
	s_pTierCount = NULL;
	s_pTierOffset = NULL;
	s_pTierArcs = NULL;
	s_pTierArcPair = NULL;
	s_UnreachedHash = CXYHash();
	st.tTotal = Plat_FloatTime() - tStart;
}
