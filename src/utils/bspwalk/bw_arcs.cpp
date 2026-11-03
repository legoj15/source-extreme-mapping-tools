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
	if ( a.tKick > 0.0 && t > a.tKick )
	{
		double zk = a.start.z + a.vz * a.tKick - 0.5 * g * a.tKick * a.tKick;
		double u = t - a.tKick;
		z = zk + a.vzKick * u - 0.5 * g * u * u;
	}
	else
		z = a.start.z + a.vz * t - 0.5 * g * t * t;
	return dvec3( a.start.x + a.vx * t, a.start.y + a.vy * t, z );
}

// A chord over dt of a parabola with acceleration g deviates from it by at
// most g dt^2 / 8 (vertically, at its middle; the perpendicular distance is
// smaller), so dt <= sqrt(8 sagitta / g) bounds every chord's error.
void ArcSetSegments( ArcD &a, double g )
{
	double dtMax = sqrt( 8.0 * ARC_SAGITTA / g );
	double t0 = a.tKick > 0.0 ? a.tKick : a.tEnd;
	double t1 = a.tKick > 0.0 ? a.tEnd - a.tKick : 0.0;
	a.nSeg[0] = t0 > 0 ? MAX( 1, (int)ceil( t0 / dtMax ) ) : 0;
	a.nSeg[1] = t1 > 0 ? MAX( 1, (int)ceil( t1 / dtMax ) ) : 0;
	a.nSeg[0] = MIN( a.nSeg[0], 65535 );
	a.nSeg[1] = MIN( a.nSeg[1], 65535 );
}

double ArcTraceRaise( const ArcD &a, double g )
{
	double dt = 0.0;
	if ( a.nSeg[0] > 0 )
		dt = ( a.tKick > 0.0 ? a.tKick : a.tEnd ) / a.nSeg[0];
	if ( a.nSeg[1] > 0 )
		dt = MAX( dt, ( a.tEnd - a.tKick ) / a.nSeg[1] );
	return g * dt * dt / 16.0;
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
	out.tEnd = T;
	out.mesh = mesh;
	ArcSetSegments( out, g );
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
};

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
	const double g = p.gravity, vmax = p.maxAirSpeed * p.reachMargin, vj = p.jumpVelocity;
	const dvec3 &L = pp.launch[li], &E = pp.edge[li], &T = pp.land[ti];
	size_t before = out.size();
	ArcD a;
	for ( int kick = 0; kick <= ( p.airJumps > 0 ? 1 : 0 ); kick++ )
	{
		if ( SolveArcToPoint( L + dvec3( 0, 0, p.crouchLift ), vj, kick != 0, g, T, vmax, BW_MESH_CROUCH, a ) )
			out.push_back( a );
		if ( SolveArcToPoint( L, vj, kick != 0, g, T, vmax, BW_MESH_CROUCH, a ) )
			out.push_back( a );
	}
	if ( T.z < E.z - 1e-6 && SolveArcToPoint( E, 0.0, false, g, T, vmax, BW_MESH_CROUCH, a ) )
		out.push_back( a );
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
	// TODO(#13B): phase A traces every arc crouched (a standing jump is dominated),
	// so the stand obstacle set and mesh stay empty until a launch source needs them
	s_ArcObs[BW_MESH_CROUCH].Build( world, world.crouch );
	for ( int i = 0; i <= MAX_TOOL_THREADS; i++ )
		s_ArcLast[i].v[0] = s_ArcLast[i].v[1] = -1;

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

// Targets of one launch node: unreached nodes at most maxJumpRise above it and
// within R(drop) in XY, where drop is the largest drop between the two boxes
// (the widest reach any arc of the pair can have). A slope launcher (interior,
// LAUNCH_SLOPE) only takes targets in its downhill half-plane that drop at most
// its slope drop: elsewhere its downhill neighbour, closer and not much lower,
// reaches at least as far (see SlopeLaunchDrop).
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
	const double Rmax = slope ? p.ArcReachAtDrop( slopeDrop + ( A.maxs.z - A.mins.z ) + 93.0 ) : p.arcReachMax;
	const double cx = 0.5 * ( A.mins.x + A.maxs.x ), cy = 0.5 * ( A.mins.y + A.maxs.y );
	std::vector<int> &out = ( *s_pArcTargets )[i];
	out.clear();
	if ( A.poly.size() < 3 )
		return;
	std::vector<int> &cand = s_ArcCand[iThread];
	cand.clear();
	s_UnreachedHash.Query( A.mins.x - Rmax, A.mins.y - Rmax, A.maxs.x + Rmax, A.maxs.y + Rmax, cand );
	double zTop = A.maxs.z + p.maxJumpRise + 1e-6;
	for ( size_t k = 0; k < cand.size(); k++ )
	{
		int b = cand[k];
		if ( b == a || g.reached[b] )
			continue;
		const WalkNode &B = g.nodes[b];
		if ( B.mins.z > zTop || B.poly.size() < 3 )
			continue;
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
		double R = p.ArcReachAtDrop( A.maxs.z - B.mins.z );
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
	for ( int m = 0; m < BW_MESH_COUNT; m++ )
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
	size_t done = 0;
	const int maxReport = 8;
	int reported = 0;
	// TODO(#13B): phase B launch sources (pushes, ladders, water, teleports,
	// ride edges; bw_arcs.h) join each round here, from the same frontier.
	for ( ;; )
	{
		const std::vector<int> &order = sink.Order();
		size_t end = order.size();
		if ( done == end )
			break;
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
						// the flood follows the CPU reference
						st.checkArcs += (long long)arcs.size();
						for ( size_t i = 0; i < arcs.size(); i++ )
						{
							if ( blocked[i] == blockedCpu[i] )
								continue;
							if ( blocked[i] )
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
									Warning( "bspwalk: arccheck: arc %d -> %d %s on the GPU, %s on the CPU, grazing distance %.6f "
											 "(start %.3f %.3f %.3f, v %.3f %.3f %.3f, T %.4f, mesh %d)\n",
											 batch[arcPair[i]].a, batch[arcPair[i]].b, blocked[i] ? "blocked" : "free",
											 blockedCpu[i] ? "blocked" : "free", depth, arcs[i].start.x, arcs[i].start.y,
											 arcs[i].start.z, arcs[i].vx, arcs[i].vy, arcs[i].vz, arcs[i].tEnd, arcs[i].mesh );
							}
						}
						blocked.swap( blockedCpu );
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
		if ( !rs.reached )
			break;
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
