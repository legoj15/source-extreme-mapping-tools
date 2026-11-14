//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: -selftest. Known-answer checks of the Minkowski grow routine and of
//          the Stage 2 polygon subtraction. Every grow answer here is derived
//          by hand from P (+) (-B), with
//          support_{-B}(n) = 16|nx| + 16|ny| + h*max(0,-nz).
//
//=============================================================================//

#include "cmdlib.h"
#include "bw_grow.h"
#include "bw_walk.h"
#include "bw_world.h"
#include "bw_arcs.h"
#include "bw_optix.h"
#include "bw_vis.h"
#include "bw_eyestore.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <set>
#include <thread>

static int s_nFailures;
static int s_nChecks;
static int s_nGpuChecks;	// of s_nChecks, the ones that exist only with a GPU
static int s_nGpuSkipped;	// GPU-only arc checks left out because no GPU came up
static bool s_bVisGpuSkipped;	// TestVisGpu's device checks left out (no GPU); their number is not known then
static void TestVisGpuDevice( const float g );

static void Check( bool bOk, const char *pszWhat, ... )
{
	char buf[512];
	va_list marker;
	va_start( marker, pszWhat );
	Q_vsnprintf( buf, sizeof( buf ), pszWhat, marker );
	va_end( marker );

	s_nChecks++;
	if ( !bOk )
		s_nFailures++;
	Msg( "  [%s] %s\n", bOk ? "pass" : "FAIL", buf );
}

static bool Near( double a, double b, double tol = 1e-9 )
{
	return fabs( a - b ) <= tol;
}

// Is there a plane with this (unit) normal and offset?
static bool HasPlane( const GrownPolytope &p, const dvec3 &n, double d )
{
	for ( size_t i = 0; i < p.planes.size(); i++ )
	{
		if ( DDot( p.planes[i].n, n ) > 1.0 - 1e-9 && Near( p.planes[i].d, d, 1e-7 ) )
			return true;
	}
	return false;
}

static void AxialPlane( DPlane &p, int axis, double sign, double d )
{
	p.n = dvec3( 0, 0, 0 );
	p.n[axis] = sign;
	p.d = d;
}

// Builds a polytope straight from a plane list, for the verifier's negative test.
static void PolytopeFromPlanes( const std::vector<DPlane> &planes, GrownPolytope &out )
{
	ConvexShape s;
	ConvexFromPlanes( planes.data(), (int)planes.size(), s );
	out.planes = planes;
	out.verts = s.verts;
	out.faces = s.faces;
	out.mins = s.mins;
	out.maxs = s.maxs;
}

static void VerifyCheck( const ConvexShape &src, const PlayerHull &hull, const GrownPolytope &p, const char *pszName )
{
	char why[256] = "";
	bool ok = VerifyGrown( src, hull, p, why, sizeof( why ) );
	Check( ok, "%s: polytope equals hull of {v + c} (%s)", pszName, ok ? "ok" : why );
}

//-----------------------------------------------------------------------------
// (a) Axis-aligned 64^3 cube centred on the origin.
//-----------------------------------------------------------------------------
static void TestCube()
{
	Msg( "(a) cube [-32,32]^3\n" );
	DPlane planes[6];
	for ( int a = 0; a < 3; a++ )
	{
		AxialPlane( planes[a * 2 + 0], a, 1.0, 32.0 );
		AxialPlane( planes[a * 2 + 1], a, -1.0, 32.0 );
	}
	ConvexShape cube;
	Check( ConvexFromPlanes( planes, 6, cube ), "source cube builds from its 6 planes" );

	PlayerProfile prof;
	GrownPolytope stand;
	bool ok = GrowConvex( cube, prof.Stand(), stand );
	Check( ok, "stand grow succeeds" );
	Check( Near( stand.mins.x, -48 ) && Near( stand.mins.y, -48 ) && Near( stand.mins.z, -104 ) &&
		   Near( stand.maxs.x, 48 ) && Near( stand.maxs.y, 48 ) && Near( stand.maxs.z, 32 ),
		   "stand: bounds [-48,48]x[-48,48]x[-104,32] (got [%g,%g]x[%g,%g]x[%g,%g])",
		   stand.mins.x, stand.maxs.x, stand.mins.y, stand.maxs.y, stand.mins.z, stand.maxs.z );
	Check( HasPlane( stand, dvec3( 1, 0, 0 ), 48 ) && HasPlane( stand, dvec3( -1, 0, 0 ), 48 ) &&
		   HasPlane( stand, dvec3( 0, 1, 0 ), 48 ) && HasPlane( stand, dvec3( 0, -1, 0 ), 48 ) &&
		   HasPlane( stand, dvec3( 0, 0, 1 ), 32 ) && HasPlane( stand, dvec3( 0, 0, -1 ), 104 ),
		   "stand: planes +-X 48, +-Y 48, +Z 32, -Z 104" );
	Check( Near( PolytopeVolume( stand.verts, stand.faces ), 96.0 * 96.0 * 136.0, 1e-3 ),
		   "stand: volume 96*96*136 (got %.3f)", PolytopeVolume( stand.verts, stand.faces ) );
	VerifyCheck( cube, prof.Stand(), stand, "stand" );

	GrownPolytope crouch;
	ok = GrowConvex( cube, prof.Crouch(), crouch );
	Check( ok && Near( crouch.mins.z, -68 ) && Near( crouch.maxs.z, 32 ),
		   "crouch: z in [-68,32] (got [%g,%g])", crouch.mins.z, crouch.maxs.z );
	VerifyCheck( cube, prof.Crouch(), crouch, "crouch" );

	// Negative test for the verifier itself: drop the +Z facet and rebuild.
	// The result is unbounded upward (clipped only by the huge base winding),
	// so its vertices are not sum points and verification must fail.
	std::vector<DPlane> cut;
	for ( size_t i = 0; i < stand.planes.size(); i++ )
	{
		if ( stand.planes[i].n.z < 0.5 )
			cut.push_back( stand.planes[i] );
	}
	GrownPolytope broken;
	PolytopeFromPlanes( cut, broken );
	char why[256] = "";
	Check( !VerifyGrown( cube, prof.Stand(), broken, why, sizeof( why ) ),
		   "verifier rejects the cube sum with its +Z facet removed (%s)", why );
}

//-----------------------------------------------------------------------------
// The 45 degree ramp wedge used by (b): base [0,64]x[0,64] on z=0, rising
// along +X to z=64 at x=64. Rotated by yawDeg about Z.
//-----------------------------------------------------------------------------
static void BuildRamp( double yawDeg, ConvexShape &out, dvec3 &slopeN )
{
	const double r2 = sqrt( 0.5 );
	dvec3 localN[5] = { dvec3( 0, 0, -1 ), dvec3( 1, 0, 0 ), dvec3( 0, -1, 0 ), dvec3( 0, 1, 0 ), dvec3( -r2, 0, r2 ) };
	double localD[5] = { 0, 64, 0, 64, 0 };

	double c = cos( yawDeg * M_PI / 180.0 ), s = sin( yawDeg * M_PI / 180.0 );
	DPlane planes[5];
	for ( int i = 0; i < 5; i++ )
	{
		// a rotation about the origin preserves every plane offset d
		planes[i].n = dvec3( c * localN[i].x - s * localN[i].y, s * localN[i].x + c * localN[i].y, localN[i].z );
		planes[i].d = localD[i];
	}
	slopeN = planes[4].n;
	ConvexFromPlanes( planes, 5, out );
}

static void TestRamp()
{
	PlayerProfile prof;
	PlayerHull hull = prof.Stand();

	Msg( "(b1) 45-degree ramp, axis aligned\n" );
	{
		ConvexShape ramp;
		dvec3 sn;
		BuildRamp( 0.0, ramp, sn );

		GrownPolytope g;
		bool ok = GrowConvex( ramp, hull, g );
		Check( ok, "grow succeeds" );
		double expect = 16.0 * ( fabs( sn.x ) + fabs( sn.y ) ) + 72.0 * ( sn.z < 0 ? -sn.z : 0 );
		Check( HasPlane( g, sn, expect ), "slope plane offset = max n.v (0) + 16|nx| + 16|ny| + 72 max(0,-nz) = %.9f", expect );
		Check( HasPlane( g, dvec3( -1, 0, 0 ), 16 ) && HasPlane( g, dvec3( 0, 0, 1 ), 64 ),
			   "bevel -X at 16 (wedge's x=0 edge) and +Z at 64 (ridge)" );
		Check( HasPlane( g, dvec3( 1, 0, 0 ), 80 ) && HasPlane( g, dvec3( 0, 1, 0 ), 80 ) &&
			   HasPlane( g, dvec3( 0, -1, 0 ), 16 ) && HasPlane( g, dvec3( 0, 0, -1 ), 72 ),
			   "faces +X 80, +Y 80, -Y 16, -Z 72" );
		VerifyCheck( ramp, hull, g, "axis ramp" );
	}

	Msg( "(b2) 45-degree ramp, yawed 30 degrees\n" );
	{
		ConvexShape ramp;
		dvec3 sn;
		BuildRamp( 30.0, ramp, sn );
		GrownPolytope g;
		bool ok = GrowConvex( ramp, hull, g );
		Check( ok, "grow succeeds" );
		double expect = 16.0 * ( fabs( sn.x ) + fabs( sn.y ) ) + 72.0 * ( sn.z < 0 ? -sn.z : 0 );
		Check( HasPlane( g, sn, expect ), "slope plane offset = %.9f", expect );

		// The slope's side edge at y'=0 runs along D = (cos30, sin30, 1). D x (-X)
		// = (0, -1, 0.5): P attains its max (0) along that edge, -B along its
		// X edge (y=-16, z=0), so the face is edge (+) edge = 2-D: a true bevel.
		dvec3 bevel( 0, -1, 0.5 );
		DNormalize( bevel );
		double bevelD = 16.0 * fabs( bevel.y );
		Check( HasPlane( g, bevel, bevelD ), "edge x axis bevel n=(0,-0.894,0.447) at d=%.9f", bevelD );
		VerifyCheck( ramp, hull, g, "yawed ramp" );

		// Negative test for the verifier: drop that bevel (plane and face).
		// Every remaining vertex is still a sum point and every plane still
		// supports, so only the plane-corner check can notice.
		GrownPolytope cut = g;
		for ( size_t i = 0; i < cut.planes.size(); i++ )
		{
			if ( DDot( cut.planes[i].n, bevel ) > 1.0 - 1e-9 )
			{
				cut.planes.erase( cut.planes.begin() + i );
				cut.faces.erase( cut.faces.begin() + i );
				break;
			}
		}
		char why[256] = "";
		Check( cut.planes.size() + 1 == g.planes.size() && !VerifyGrown( ramp, hull, cut, why, sizeof( why ) ),
			   "verifier rejects the yawed ramp sum with that bevel removed (%s)", why );
	}
}

//-----------------------------------------------------------------------------
// (c) A single flat triangle grows into a closed polytope.
//-----------------------------------------------------------------------------
static void TestTriangle()
{
	PlayerProfile prof;
	Msg( "(c) flat triangles\n" );

	struct
	{
		const char *name;
		dvec3 a, b, c;
	} tris[2] =
	{
		{ "sloped", dvec3( 0, 0, 0 ), dvec3( 100, 0, 0 ), dvec3( 30, 50, 10 ) },
		{ "vertical sliver in x=0", dvec3( 0, 0, 0 ), dvec3( 0, 0, 50 ), dvec3( 0, 40, 0 ) },
	};

	for ( int t = 0; t < 2; t++ )
	{
		ConvexShape tri;
		Check( ConvexFromTriangle( tris[t].a, tris[t].b, tris[t].c, tri ) && tri.flat, "%s: flat source", tris[t].name );
		for ( int posture = 0; posture < 2; posture++ )
		{
			PlayerHull hull = posture ? prof.Crouch() : prof.Stand();
			const char *pn = posture ? "crouch" : "stand";
			GrownPolytope g;
			bool ok = GrowConvex( tri, hull, g );
			double vol = ok ? PolytopeVolume( g.verts, g.faces ) : 0;
			Check( ok && vol > 1.0, "%s %s: closed, volume %.1f > 0", tris[t].name, pn, vol );
			Check( ok && g.maxs.x - g.mins.x >= 32.0 - 1e-9 && g.maxs.y - g.mins.y >= 32.0 - 1e-9,
				   "%s %s: >= 32u wide in X and Y (%.3f x %.3f)", tris[t].name, pn, g.maxs.x - g.mins.x, g.maxs.y - g.mins.y );
			VerifyCheck( tri, hull, g, pn );
		}
	}
}

//-----------------------------------------------------------------------------
// (d) Stage 2: convex polygon minus convex polytope.
//-----------------------------------------------------------------------------
static void BoxPlanes( const dvec3 &mins, const dvec3 &maxs, std::vector<DPlane> &planes )
{
	planes.resize( 6 );
	for ( int a = 0; a < 3; a++ )
	{
		AxialPlane( planes[a * 2 + 0], a, 1.0, maxs[a] );
		AxialPlane( planes[a * 2 + 1], a, -1.0, -mins[a] );
	}
}

static void SquareZ0( double x0, double y0, double x1, double y1, DWinding &w )
{
	// CCW seen from above, as an up face is
	w.resize( 4 );
	w[0] = dvec3( x0, y0, 0 );
	w[1] = dvec3( x1, y0, 0 );
	w[2] = dvec3( x1, y1, 0 );
	w[3] = dvec3( x0, y1, 0 );
}

static double TotalArea( const std::vector<DWinding> &pieces )
{
	double a = 0;
	for ( size_t i = 0; i < pieces.size(); i++ )
		a += WalkPolyArea( pieces[i] );
	return a;
}

// Subtracts several polytopes in turn, as BuildWalkPolygons does.
static void SubtractAll( const DWinding &poly, const std::vector<const GrownPolytope *> &obstacles, double shrink,
						 std::vector<DWinding> &out )
{
	out.assign( 1, poly );
	for ( size_t k = 0; k < obstacles.size(); k++ )
	{
		std::vector<DWinding> next;
		for ( size_t i = 0; i < out.size(); i++ )
			SubtractPolytope( out[i], obstacles[k]->planes.data(), (int)obstacles[k]->planes.size(), shrink, next );
		out.swap( next );
	}
}

static void TestSubtract()
{
	Msg( "(d) walkable polygon minus polytope\n" );

	DWinding square;
	SquareZ0( 0, 0, 100, 100, square );
	std::vector<DPlane> box;
	BoxPlanes( dvec3( 50, 50, -10 ), dvec3( 150, 150, 10 ), box );

	// (d1) square [0,100]^2 minus the box [50,150]^2: the L-shape, 10000 - 50^2.
	std::vector<DWinding> pieces;
	bool cut = SubtractPolytope( square, box.data(), (int)box.size(), 0.0, pieces );
	bool outside = true;
	for ( size_t i = 0; i < pieces.size(); i++ )
	{
		dvec3 c( 0, 0, 0 );
		for ( size_t k = 0; k < pieces[i].size(); k++ )
			c += pieces[i][k];
		c = c * ( 1.0 / pieces[i].size() );
		outside = outside && ( c.x < 50 || c.y < 50 );
	}
	Check( cut && pieces.size() >= 2 && Near( TotalArea( pieces ), 7500.0, 1e-9 ) && outside,
		   "square minus overlapping box: L-shape area 7500 in >= 2 convex pieces, none inside (got %d pieces, %.9f)",
		   (int)pieces.size(), TotalArea( pieces ) );

	// The 1/32 shrink only ever gives back free space.
	pieces.clear();
	SubtractPolytope( square, box.data(), (int)box.size(), WALK_SHRINK, pieces );
	double expect = 10000.0 - ( 50.0 - WALK_SHRINK ) * ( 50.0 - WALK_SHRINK );
	Check( Near( TotalArea( pieces ), expect, 1e-9 ), "with the 1/32 shrink: area 10000 - (50 - 1/32)^2 = %.9f (got %.9f)",
		   expect, TotalArea( pieces ) );

	// A box that only touches the square's edge leaves it whole.
	BoxPlanes( dvec3( 100, 0, -10 ), dvec3( 200, 100, 10 ), box );
	pieces.clear();
	cut = SubtractPolytope( square, box.data(), (int)box.size(), 0.0, pieces );
	Check( !cut && pieces.size() == 1 && Near( TotalArea( pieces ), 10000.0, 1e-9 ),
		   "a box touching only the square's edge leaves it whole (cut %d, %d pieces)", (int)cut, (int)pieces.size() );

	// (d2) Two walls 32.01u apart, grown by the crouch hull: the floor between
	// them keeps a 0.01u sliver (0.01 + 2/32 with the shrink). Placed near
	// 13000u, where a float ulp is ~0.001u, so this needs double precision.
	PlayerProfile prof;
	ConvexShape wallA, wallB;
	std::vector<DPlane> pa, pb;
	BoxPlanes( dvec3( 11900, 12900, 0 ), dvec3( 12200, 13000, 96 ), pa );
	BoxPlanes( dvec3( 11900, 13032.01, 0 ), dvec3( 12200, 13100, 96 ), pb );
	ConvexFromPlanes( pa.data(), 6, wallA );
	ConvexFromPlanes( pb.data(), 6, wallB );
	GrownPolytope ga, gb;
	bool grown = GrowConvex( wallA, prof.Crouch(), ga ) && GrowConvex( wallB, prof.Crouch(), gb );
	std::vector<const GrownPolytope *> walls;
	walls.push_back( &ga );
	walls.push_back( &gb );

	DWinding floor;
	SquareZ0( 12000, 12950, 12128, 13080, floor );
	for ( int s = 0; s < 2; s++ )
	{
		double shrink = s ? WALK_SHRINK : 0.0;
		SubtractAll( floor, walls, shrink, pieces );
		double ylo = 1e300, yhi = -1e300;
		for ( size_t i = 0; i < pieces.size(); i++ )
		{
			for ( size_t k = 0; k < pieces[i].size(); k++ )
			{
				ylo = MIN( ylo, pieces[i][k].y );
				yhi = MAX( yhi, pieces[i][k].y );
			}
		}
		double width = 0.01 + 2.0 * shrink;
		Check( grown && !pieces.empty() && Near( ylo, 13016.0 - shrink, 1e-6 ) && Near( yhi, 13016.01 + shrink, 1e-6 ) &&
			   Near( TotalArea( pieces ), 128.0 * width, 1e-6 ),
			   "32.01u corridor, shrink %g: sliver y %.6f..%.6f, %.6fu wide, area %.6f (expect %.6f wide, area %.6f)",
			   shrink, ylo, yhi, yhi - ylo, TotalArea( pieces ), width, 128.0 * width );
	}
}

//-----------------------------------------------------------------------------
// (e) Stage 3: edges between two nodes are decided exactly along their shared
// boundary, not at its ends and midpoint.
//-----------------------------------------------------------------------------
static WalkNode MakeNode( const DWinding &poly, const dvec3 &n, double d )
{
	WalkNode w;
	w.poly = poly;	// CCW seen from above
	w.plane.n = n;
	w.plane.d = d;
	w.mins = dvec3( 1e300, 1e300, 1e300 );
	w.maxs = dvec3( -1e300, -1e300, -1e300 );
	for ( size_t k = 0; k < poly.size(); k++ )
	{
		w.mins = DMin( w.mins, poly[k] );
		w.maxs = DMax( w.maxs, poly[k] );
	}
	w.area = WalkPolyArea( poly );
	w.category = CAT_WORLD;
	w.convex = -1;
	w.pose = 0;
	w.standArea = 0;
	return w;
}

// The edges between nodes 0 and 1 of a two-node graph over the given
// crouch obstacles (category world), counted per kind and direction.
static void EdgesBetween( const WalkNode &A, const WalkNode &B, const std::vector<GrownPolytope> &obstacles, int counts[2][EDGE_KIND_COUNT] )
{
	CollisionWorld world;
	for ( size_t i = 0; i < obstacles.size(); i++ )
	{
		world.convexes.push_back( SourceConvex() );
		world.convexes.back().category = CAT_WORLD;
		world.convexes.back().entityIndex = -1;
		world.crouch.push_back( obstacles[i] );
		world.stand.push_back( obstacles[i] );
	}
	PlayerProfile prof;
	WalkGraph graph;
	BuildWalkPolygons( world, prof, graph );	// builds the obstacle sets
	graph.nodes.clear();
	graph.nodes.push_back( A );
	graph.nodes.push_back( B );
	BuildWalkEdges( world, prof, graph );
	memset( counts, 0, sizeof( int ) * 2 * EDGE_KIND_COUNT );
	for ( size_t e = 0; e < graph.edges.size(); e++ )
		counts[graph.edges[e].a == 0 ? 0 : 1][graph.edges[e].kind]++;
}

static void TestEdges()
{
	Msg( "(e) walk graph edges along a shared boundary\n" );
	int c[2][EDGE_KIND_COUNT];

	// (e1) Flat A (z = 0) meets slope B (z = 60 - 0.4x) along y = 100, x 0..400:
	// dz = 60 - 0.4x is within the 18u step only for x in [105, 195], strictly
	// between the samples x = 0, 200, 400 (dz 60, -20, -100).
	DWinding a, b;
	SquareZ0( 0, 0, 400, 100, a );
	b.resize( 4 );
	b[0] = dvec3( 0, 100, 60 );
	b[1] = dvec3( 400, 100, -100 );
	b[2] = dvec3( 400, 200, -100 );
	b[3] = dvec3( 0, 200, 60 );
	dvec3 slopeN( 0.4, 0, 1 );
	double len = DLength( slopeN );
	WalkNode A = MakeNode( a, dvec3( 0, 0, 1 ), 0.0 );
	WalkNode B = MakeNode( b, slopeN * ( 1.0 / len ), 60.0 / len );
	EdgesBetween( A, B, std::vector<GrownPolytope>(), c );
	Check( c[0][EDGE_WALK] + c[1][EDGE_WALK] == 1, "flat meets slope, step band only between samples: one walk edge (got %d walk, %d jump, %d drop)",
		   c[0][EDGE_WALK] + c[1][EDGE_WALK], c[0][EDGE_JUMP] + c[1][EDGE_JUMP], c[0][EDGE_DROP] + c[1][EDGE_DROP] );

	// (e2) Flat A (z = 0) under a ledge B (z = 100) along y = 100, x 0..400,
	// with obstacle slabs at z 40..60 over x [-50, 250] and [300, 450]: every
	// sample is blocked, the drop through the gap x (250, 300) is free.
	DWinding b2;
	b2.resize( 4 );
	b2[0] = dvec3( 0, 100, 100 );
	b2[1] = dvec3( 400, 100, 100 );
	b2[2] = dvec3( 400, 200, 100 );
	b2[3] = dvec3( 0, 200, 100 );
	WalkNode B2 = MakeNode( b2, dvec3( 0, 0, 1 ), 100.0 );
	std::vector<DPlane> planes;
	std::vector<GrownPolytope> slabs( 2 );
	BoxPlanes( dvec3( -50, 50, 40 ), dvec3( 250, 150, 60 ), planes );
	PolytopeFromPlanes( planes, slabs[0] );
	BoxPlanes( dvec3( 300, 50, 40 ), dvec3( 450, 150, 60 ), planes );
	PolytopeFromPlanes( planes, slabs[1] );
	EdgesBetween( A, B2, slabs, c );
	Check( c[1][EDGE_DROP] == 1 && c[0][EDGE_DROP] + c[0][EDGE_WALK] + c[1][EDGE_WALK] + c[0][EDGE_JUMP] + c[1][EDGE_JUMP] == 0,
		   "ledge 100u up, samples blocked, gap between slabs: one drop edge ledge -> floor and nothing else (got drop %d/%d, walk %d, jump %d)",
		   c[1][EDGE_DROP], c[0][EDGE_DROP], c[0][EDGE_WALK] + c[1][EDGE_WALK], c[0][EDGE_JUMP] + c[1][EDGE_JUMP] );

	// (e3) One slab over the whole boundary: no edge at all.
	BoxPlanes( dvec3( -50, 50, 40 ), dvec3( 450, 150, 60 ), planes );
	slabs.resize( 1 );
	PolytopeFromPlanes( planes, slabs[0] );
	EdgesBetween( A, B2, slabs, c );
	int total = 0;
	for ( int k = 0; k < EDGE_KIND_COUNT; k++ )
		total += c[0][k] + c[1][k];
	Check( total == 0, "ledge 100u up behind one slab over the whole boundary: no edge (got %d)", total );
}

//-----------------------------------------------------------------------------
// (f) Stage 3 arcs: the solver against hand-derived parabolas, then gaps
// between two synthetic platforms (HL2 profile: vz 160, g 600, crouch lift
// 36, the default air speed in force: intended_air_speed 400 x 1.15 = 460 u/s).
//-----------------------------------------------------------------------------
// A world of axis-aligned boxes, category world, grown for both postures.
static void BoxWorld( const std::vector< std::pair<dvec3, dvec3> > &boxes, const PlayerProfile &prof, CollisionWorld &world )
{
	world = CollisionWorld();
	for ( size_t i = 0; i < boxes.size(); i++ )
	{
		std::vector<DPlane> planes;
		BoxPlanes( boxes[i].first, boxes[i].second, planes );
		world.convexes.push_back( SourceConvex() );
		SourceConvex &sc = world.convexes.back();
		sc.category = CAT_WORLD;
		sc.entityIndex = -1;
		sc.sourceIndex = (int)i;
		sc.label[0] = sc.targetname[0] = 0;
		ConvexFromPlanes( planes.data(), (int)planes.size(), sc.shape );
		world.stand.push_back( GrownPolytope() );
		world.crouch.push_back( GrownPolytope() );
		GrowConvex( sc.shape, prof.Stand(), world.stand.back() );
		GrowConvex( sc.shape, prof.Crouch(), world.crouch.back() );
	}
}

// The crouch-grown top of box i as a node (what Stage 2 would make of it alone).
static WalkNode TopNode( const CollisionWorld &world, int i )
{
	const GrownPolytope &g = world.crouch[i];
	for ( size_t f = 0; f < g.faces.size(); f++ )
	{
		if ( g.planes[f].n.z < 0.99 )
			continue;
		DWinding w;
		for ( size_t k = 0; k < g.faces[f].size(); k++ )
			w.push_back( g.verts[g.faces[f][k]] );
		WalkNode n = MakeNode( w, g.planes[f].n, g.planes[f].d );
		n.convex = i;
		return n;
	}
	return WalkNode();
}

struct ArcCounts
{
	int arcs, free;
	bool gpu;		// the GPU path came up (CArcTracer::HasGpu)
	int gpuDiff;	// arcs the GPU decides differently from the CPU; -1: the GPU trace did not run or failed
};

static ArcCounts TraceGap( const std::vector< std::pair<dvec3, dvec3> > &boxes, const PlayerProfile &prof )
{
	CollisionWorld world;
	BoxWorld( boxes, prof, world );
	WalkNode A = TopNode( world, 0 ), B = TopNode( world, 1 );
	std::vector<ArcD> arcs;
	for ( int c = 0; c < ARC_COMBOS; c++ )
		PairArcs( A, B, prof, c, arcs );
	ArcCounts r = { (int)arcs.size(), 0, false, -1 };
	CArcTracer tracer;
	tracer.Init( world, prof, true );
	r.gpu = tracer.HasGpu();
	std::vector<unsigned char> cpu, gpu;
	tracer.TraceCpu( arcs, cpu );
	for ( size_t i = 0; i < cpu.size(); i++ )
		r.free += !cpu[i];
	if ( r.gpu && tracer.TraceGpu( arcs, gpu ) )
	{
		r.gpuDiff = 0;
		for ( size_t i = 0; i < gpu.size(); i++ )
			r.gpuDiff += gpu[i] != cpu[i];
	}
	tracer.Shutdown();
	return r;
}

// The GPU half of a gap check. It exists only when a GPU came up; then a trace
// that did not run (gpuDiff stays -1) is a failure, not a skipped check.
static void CheckGpuAgrees( const ArcCounts &c, const char *pszName )
{
	if ( !c.gpu )
	{
		s_nGpuSkipped++;
		return;
	}
	s_nGpuChecks++;
	if ( c.gpuDiff < 0 )
		Check( false, "%s: GPU agrees with the CPU on every arc (the GPU trace failed)", pszName );
	else
		Check( c.gpuDiff == 0, "%s: GPU agrees with the CPU on every arc (%d differ)", pszName, c.gpuDiff );
}

static void TestArcs()
{
	Msg( "(f) ballistic arcs\n" );
	PlayerProfile prof;
	const double vmax = prof.airSpeed * prof.reachMargin;

	// (f1) the solver: a flat jump over 64u lands at T = 2 vz / g = 0.5333 s, vx = 120;
	// a 100u walk-off drop over 100u takes sqrt(2 * 100 / 600) = 0.57735 s; a duck-jump
	// (feet lifted 36) back to the launch floor takes (160 + sqrt(160^2 + 2 * 600 * 36)) / 600.
	ArcD a;
	bool ok = SolveArcToPoint( dvec3( 0, 0, 0 ), 160.0, false, 600.0, dvec3( 64, 0, 0 ), vmax, BW_MESH_CROUCH, a );
	Check( ok && Near( a.tEnd, 160.0 / 300.0, 1e-12 ) && Near( a.vx, 120.0, 1e-9 ) && Near( a.vy, 0.0, 1e-12 ),
		   "flat jump over 64u: T 0.533333, vx 120 (got %s, T %.6f, vx %.6f)", ok ? "ok" : "none", a.tEnd, a.vx );
	ok = SolveArcToPoint( dvec3( 0, 0, 100 ), 0.0, false, 600.0, dvec3( 100, 0, 0 ), vmax, BW_MESH_CROUCH, a );
	Check( ok && Near( a.tEnd, sqrt( 1.0 / 3.0 ), 1e-12 ) && Near( a.vx, 100.0 / sqrt( 1.0 / 3.0 ), 1e-9 ),
		   "walk-off 100u down over 100u: T 0.577350 (got %s, T %.6f)", ok ? "ok" : "none", a.tEnd );
	double tDuck = ( 160.0 + sqrt( 160.0 * 160.0 + 2.0 * 600.0 * 36.0 ) ) / 600.0;
	ok = SolveArcToPoint( dvec3( 0, 0, 36 ), 160.0, false, 600.0, dvec3( 300, 0, 0 ), vmax, BW_MESH_CROUCH, a );
	Check( ok && Near( a.tEnd, tDuck, 1e-12 ) && Near( prof.arcAirtime, tDuck, 1e-12 ),
		   "duck-jump back to the launch floor: T %.6f, the profile's arc airtime (got T %.6f, airtime %.6f)", tDuck,
		   ok ? a.tEnd : -1.0, prof.arcAirtime );
	ok = SolveArcToPoint( dvec3( 0, 0, 0 ), 160.0, false, 600.0, dvec3( 64, 0, 22 ), vmax, BW_MESH_CROUCH, a );
	Check( !ok, "a landing 22u up is above the 21.33u apex of a plain jump: no arc" );
	ok = SolveArcToPoint( dvec3( 0, 0, 0 ), 160.0, false, 600.0, dvec3( 400, 0, 0 ), vmax, BW_MESH_CROUCH, a );
	Check( !ok, "a flat jump over 400u needs 750 u/s > %.0f: no arc", vmax );
	// every chord within the sagitta bound: sample the parabola against its chords
	ok = SolveArcToPoint( dvec3( 0, 0, 36 ), 160.0, false, 600.0, dvec3( 300, 0, 0 ), vmax, BW_MESH_CROUCH, a );
	double worst = 0;
	int segs = ok ? a.nSeg[0] + a.nSeg[1] : 0;
	for ( int s = 0; s < segs; s++ )
	{
		double t0 = a.tEnd * s / segs, t1 = a.tEnd * ( s + 1 ) / segs;
		dvec3 p = ArcPoint( a, 600.0, t0 ), q = ArcPoint( a, 600.0, t1 );
		for ( int k = 1; k < 64; k++ )
		{
			double u = k / 64.0;
			dvec3 m = ArcPoint( a, 600.0, t0 + u * ( t1 - t0 ) );
			worst = MAX( worst, fabs( m.z - ( p.z + u * ( q.z - p.z ) ) ) );
		}
	}
	Check( ok && segs >= 2 && worst <= ARC_SAGITTA + 1e-9, "duck-jump chords: %d chords, worst sagitta %.4f <= %g", segs,
		   worst, ARC_SAGITTA );

	// Two platforms with tops at z = 0, 512u deep in y; the gap is between the brushes.
	std::vector< std::pair<dvec3, dvec3> > boxes( 2 );
	boxes[0] = std::make_pair( dvec3( -512, -256, -64 ), dvec3( 0, 256, 0 ) );

	// (f2) the 96u gap (f04): free arcs exist
	boxes[1] = std::make_pair( dvec3( 96, -256, -64 ), dvec3( 608, 256, 0 ) );
	ArcCounts c = TraceGap( boxes, prof );
	Check( c.arcs > 0 && c.free > 0, "96u gap: free arcs (%d arcs, %d free)", c.arcs, c.free );
	CheckGpuAgrees( c, "96u gap" );

	// (f3) a gap just inside R (brush gap R - 8) is crossed; just beyond (R + 8)
	// every arc is rejected for speed. R = %.1f for hl2 (so 400u is crossable).
	double R = prof.arcReach;
	boxes[1] = std::make_pair( dvec3( R - 8, -256, -64 ), dvec3( R + 504, 256, 0 ) );
	c = TraceGap( boxes, prof );
	Check( c.arcs > 0 && c.free > 0, "gap R - 8 = %.1fu: free arcs (%d arcs, %d free)", R - 8, c.arcs, c.free );
	boxes[1] = std::make_pair( dvec3( R + 8, -256, -64 ), dvec3( R + 520, 256, 0 ) );
	c = TraceGap( boxes, prof );
	Check( c.arcs == 0, "gap R + 8 = %.1fu: every arc rejected for speed (%d arcs)", R + 8, c.arcs );

	// (f4) the 96u gap under a slab over the gap (brush x 32..64): bottom at z 30
	// (grown down to -6) blocks every arc; at z 120 (grown to 84, above the 57.3u
	// duck-jump apex) arcs pass.
	boxes[1] = std::make_pair( dvec3( 96, -256, -64 ), dvec3( 608, 256, 0 ) );
	boxes.push_back( std::make_pair( dvec3( 32, -256, 30 ), dvec3( 64, 256, 200 ) ) );
	c = TraceGap( boxes, prof );
	Check( c.arcs > 0 && c.free == 0, "96u gap, slab at z 30 over it: every arc blocked (%d arcs, %d free)", c.arcs, c.free );
	CheckGpuAgrees( c, "low slab" );
	boxes[2] = std::make_pair( dvec3( 32, -256, 120 ), dvec3( 64, 256, 200 ) );
	c = TraceGap( boxes, prof );
	Check( c.free > 0, "96u gap, slab at z 120: free arcs (%d arcs, %d free)", c.arcs, c.free );
	CheckGpuAgrees( c, "high slab" );
	if ( s_nGpuSkipped )
		Msg( "  (%d GPU checks skipped: no CUDA device / OptiX)\n", s_nGpuSkipped );
}

//-----------------------------------------------------------------------------
// (g) #13 phase B special movement: push velocity, push launch, train poses,
// ladder mount distance, water jump-out (HL2 profile).
//-----------------------------------------------------------------------------
static bool NearV( const dvec3 &a, const dvec3 &b, double tol )
{
	return fabs( a.x - b.x ) <= tol && fabs( a.y - b.y ) <= tol && fabs( a.z - b.z ) <= tol;
}

static void TestSpecial()
{
	Msg( "(g) special movement\n" );
	PlayerProfile prof;

	// (g1) trigger_push: pushdir angles through AngleVectors; speed 0 is 100
	dvec3 v = PushVelocity( "-90 0 0", 600 );
	Check( NearV( v, dvec3( 0, 0, 600 ), 1e-9 ), "pushdir -90 0 0 at 600: velocity (0 0 600) (got %.6f %.6f %.6f)", v.x, v.y, v.z );
	v = PushVelocity( "0 90 0", 0 );
	Check( NearV( v, dvec3( 0, 100, 0 ), 1e-9 ), "pushdir 0 90 0, speed 0: the default 100 along +y (got %.6f %.6f %.6f)", v.x,
		   v.y, v.z );

	// (g2) a plain launch (a push-once's impulse): leaving z 224 with vz 600, landing
	// 64u lower and 40u away takes T = (600 + sqrt(600^2 + 2 * 600 * 64)) / 600
	ArcD a;
	double T = ( 600.0 + sqrt( 600.0 * 600.0 + 2.0 * 600.0 * 64.0 ) ) / 600.0;
	bool ok = SolveArcBase( dvec3( 0, 0, 224 ), 600.0, 0.0, 0.0, 600.0, dvec3( 40, 0, 160 ), 736.0, BW_MESH_CROUCH, a );
	Check( ok && Near( a.tEnd, T, 1e-12 ) && Near( a.vx, 40.0 / T, 1e-9 ) && Near( a.vz, 600.0, 1e-12 ),
		   "push launch vz 600 from z 224 to 40u away at z 160: T %.6f, vx %.6f (got %s, T %.6f, vx %.6f)", T, 40.0 / T,
		   ok ? "ok" : "none", a.tEnd, a.vx );
	// a horizontal push of 300 u/s with no own speed: a 100u walk-off lands 300 T away, not 301 T
	double Tf = sqrt( 200.0 / 600.0 );
	ok = SolveArcBase( dvec3( 0, 0, 100 ), 0.0, 300.0, 0.0, 600.0, dvec3( 300.0 * Tf, 0, 0 ), 1e-9, BW_MESH_CROUCH, a );
	bool ok2 = SolveArcBase( dvec3( 0, 0, 100 ), 0.0, 300.0, 0.0, 600.0, dvec3( 301.0 * Tf, 0, 0 ), 1e-9, BW_MESH_CROUCH, a );
	Check( ok && !ok2, "300 u/s push, no own speed: a 100u drop lands at 300 T (%s), not 301 T (%s)", ok ? "yes" : "no",
		   ok2 ? "yes" : "no" );

	// (g2b) a continuous push of 900 straight up at gravity 600 (f14): inside, gravity
	// is -300, so from rest the player leaves the column top (z 224) after
	// sqrt(2 * 224 / 300) s at 300 times that, then lands 64u lower and 40u away
	double tUp = sqrt( 2.0 * 224.0 / 300.0 ), vUp = 300.0 * tUp;
	double tAll = tUp + ( vUp + sqrt( vUp * vUp + 2.0 * 600.0 * 64.0 ) ) / 600.0;
	ok = SolvePushArc( dvec3( 0, 0, 0 ), 0.0, -300.0, 224.0, 0.0, 0.0, 600.0, dvec3( 40, 0, 160 ), 736.0, BW_MESH_CROUCH, a );
	Check( ok && Near( a.tKick, tUp, 1e-9 ) && Near( a.vzKick, vUp, 1e-9 ) && Near( a.tEnd, tAll, 1e-9 ) &&
			   Near( ArcPoint( a, 600.0, tUp ).z, 224.0, 1e-9 ) && Near( ArcPoint( a, 600.0, tAll ).z, 160.0, 1e-9 ),
		   "lifting push 900 at gravity 600, from rest: exits z 224 at %.4f s, %.3f u/s, lands z 160 at %.4f s (got %s, "
		   "%.4f s, %.3f u/s, %.4f s)", tUp, vUp, tAll, ok ? "ok" : "none", a.tKick, a.vzKick, a.tEnd );
	// its chords: every point of the flight within ARC_SAGITTA / 2 of the raised chords
	double raise = ok ? ArcTraceRaise( a, 600.0 ) : 0.0, worst = 0.0;
	for ( int piece = 0; ok && piece < 2; piece++ )
	{
		int n = a.nSeg[piece];
		double ts = piece ? a.tKick : 0.0, te = piece ? a.tEnd : a.tKick;
		for ( int k = 0; k < n; k++ )
		{
			double t0 = ts + ( te - ts ) * k / n, t1 = ts + ( te - ts ) * ( k + 1 ) / n;
			dvec3 p = ArcPoint( a, 600.0, t0 ), q = ArcPoint( a, 600.0, t1 );
			for ( int j = 1; j < 64; j++ )
			{
				double u = j / 64.0;
				double z = ArcPoint( a, 600.0, t0 + u * ( t1 - t0 ) ).z;
				worst = MAX( worst, fabs( z - ( p.z + u * ( q.z - p.z ) + raise ) ) );
			}
		}
	}
	Check( ok && worst <= 0.5 * ARC_SAGITTA + 1e-9, "lifting push flight: worst distance to the raised chords %.4f <= %g "
		   "(raise %.4f)", worst, 0.5 * ARC_SAGITTA, raise );
	// the f14 negative control: a push of 300 (gravity 300 inside) never lifts a
	// duck-jump (feet at 36, vz 160) to the 160u ledge: not over the top, not from
	// the apex 78.7, not inside
	double apexW = 36.0 + 160.0 * 160.0 / 600.0;
	bool w0 = SolvePushArc( dvec3( 0, 0, 36 ), 160.0, 300.0, 224.0, 0.0, 0.0, 600.0, dvec3( 40, 0, 160 ), 736.0, BW_MESH_CROUCH, a );
	bool w1 = SolvePushArc( dvec3( 0, 0, 36 ), 160.0, 300.0, apexW, 0.0, 0.0, 600.0, dvec3( 40, 0, 160 ), 736.0, BW_MESH_CROUCH, a );
	bool w2 = SolvePushArc( dvec3( 0, 0, 36 ), 160.0, 300.0, -1e300, 0.0, 0.0, 600.0, dvec3( 40, 0, 160 ), 736.0, BW_MESH_CROUCH, a );
	Check( !w0 && !w1 && !w2, "weak push 300: no flight from a duck-jump to z 160 (top exit %s, apex exit %s, inside %s)",
		   w0 ? "yes" : "no", w1 ? "yes" : "no", w2 ? "yes" : "no" );

	// (g3) train poses: a tracktrain's origin goes to path + (0, 0, height); a
	// func_train's model centre goes to the corner
	v = TrackTrainTranslation( dvec3( 256, 256, 6 ), dvec3( 824, 256, 154 ), 10.0 );
	Check( NearV( v, dvec3( 568, 0, 158 ), 1e-9 ), "tracktrain at (824 256 154), height 10, from (256 256 6): moves (568 0 158) "
		   "(got %.3f %.3f %.3f)", v.x, v.y, v.z );
	v = FuncTrainTranslation( dvec3( 100, 0, 0 ), dvec3( 0, 0, 8 ), dvec3( 500, 0, 64 ) );
	Check( NearV( v, dvec3( 400, 0, 56 ), 1e-9 ), "func_train centre (0 0 8) to corner (500 0 64) from (100 0 0): moves (400 0 56) "
		   "(got %.3f %.3f %.3f)", v.x, v.y, v.z );

	// (g4) ladder mount: segment to polygon distance
	DWinding sq;
	SquareZ0( 0, 0, 100, 100, sq );
	double d = SegmentPolygonDistance( dvec3( 164, 50, 0 ), dvec3( 164, 50, 100 ), sq );
	Check( Near( d, 64.0, 1e-9 ), "vertical segment 64u beside a square: distance 64 (got %.6f)", d );
	d = SegmentPolygonDistance( dvec3( 50, 50, -10 ), dvec3( 50, 50, 10 ), sq );
	Check( Near( d, 0.0, 1e-12 ), "segment through the square: distance 0 (got %.6f)", d );
	d = SegmentPolygonDistance( dvec3( 50, 50, 30 ), dvec3( 60, 50, 40 ), sq );
	Check( Near( d, 30.0, 1e-9 ), "segment 30u above the square: distance 30 (got %.6f)", d );

	// (g5) water jump-out: a pool (water to z 0) between a ledge 57u above its surface
	// (within the 57.33u duck-jump rise) and one 58u above it: a jump edge from a
	// water surface node to the first ledge only
	std::vector< std::pair<dvec3, dvec3> > boxes( 3 );
	boxes[0] = std::make_pair( dvec3( 0, 0, -128 ), dvec3( 128, 128, 0 ) );		// water
	boxes[1] = std::make_pair( dvec3( 128, 0, -128 ), dvec3( 256, 128, 57 ) );		// ledge A
	boxes[2] = std::make_pair( dvec3( -128, 0, -128 ), dvec3( 0, 128, 58 ) );		// ledge B
	CollisionWorld world;
	BoxWorld( boxes, prof, world );
	world.convexes[0].category = CAT_WATER;
	WalkGraph graph;
	BuildWalkPolygons( world, prof, graph );
	BuildWalkEdges( world, prof, graph );
	int waterNodes = 0, toA = 0, toB = 0;
	for ( size_t i = 0; i < graph.nodes.size(); i++ )
		waterNodes += graph.nodes[i].category == CAT_WATER;
	for ( size_t e = 0; e < graph.edges.size(); e++ )
	{
		const WalkEdge &we = graph.edges[e];
		if ( graph.nodes[we.a].category != CAT_WATER || we.kind != EDGE_JUMP )
			continue;
		toA += graph.nodes[we.b].convex == 1;
		toB += graph.nodes[we.b].convex == 2;
	}
	Check( waterNodes > 0 && toA > 0 && toB == 0, "water jump-out: surface nodes %d, jump edges to the 57u ledge %d (want > 0), "
		   "to the 58u ledge %d (want 0)", waterNodes, toA, toB );
}

//-----------------------------------------------------------------------------
// (h) Visibility (#14): the feet column, polyhedron subtraction, the ray and
//     mirror helpers. Answers by hand.
//-----------------------------------------------------------------------------
static void VisBoxPlanes( const dvec3 &mins, const dvec3 &maxs, std::vector<DPlane> &out )
{
	out.clear();
	for ( int a = 0; a < 3; a++ )
	{
		DPlane p;
		AxialPlane( p, a, 1.0, maxs[a] );
		out.push_back( p );
		AxialPlane( p, a, -1.0, -mins[a] );
		out.push_back( p );
	}
}

// Seam fillers (#33, #40): boxes that meet the ways brushes do, as BuildOccluders sees them
// (each shrunk by OCC_SHRINK, then the fillers, used as they are).
//   A 0..10^3 and B on its +y side (y 10..20): a wall of two brushes, seam plane y = 10
//   C y 21..31 beside B: a real 1u slot (y 20..21), which no filler may close
//   D y -10..0, z 0..4 on A's -y side: a partial contact, A's face is exposed above z 4
//   F a floor (x -20..20, y 40..60, z -10..0) under W1 (x -20..0) and W2 (x 0..20), both y 48..52,
//     z 0..10: a T of three brushes, whose pairwise fillers alone leave a tunnel along y at x 0, z 0
//   Q1..Q4 x 100..110, y 0..10, z 0..10 cut at x 105 and z 5: four brushes meeting along a line
struct SeamScene
{
	std::vector<ConvexShape> shapes;
	std::vector< std::vector<DPlane> > occ;		// shrunk boxes, then the fillers
	std::vector< std::vector<DPlane> > fill;
	std::vector<int> pairs;		// the two boxes each filler joins
	int contacts, extended;
};

static void BuildSeamScene( SeamScene &s, bool withFill )
{
	static const double boxes[][6] = {
		{ 0, 0, 0, 10, 10, 10 },			// A
		{ 0, 10, 0, 10, 20, 10 },			// B
		{ 0, 21, 0, 10, 31, 10 },			// C
		{ 0, -10, 0, 10, 0, 4 },			// D
		{ -20, 40, -10, 20, 60, 0 },		// F
		{ -20, 48, 0, 0, 52, 10 },			// W1
		{ 0, 48, 0, 20, 52, 10 },			// W2
		{ 100, 0, 0, 105, 10, 5 },			// Q1
		{ 105, 0, 0, 110, 10, 5 },			// Q2
		{ 100, 0, 5, 105, 10, 10 },			// Q3
		{ 105, 0, 5, 110, 10, 10 },			// Q4
	};
	s.shapes.clear();
	s.occ.clear();
	s.fill.clear();
	s.pairs.clear();
	std::vector<DPlane> p;
	for ( size_t i = 0; i < ARRAYSIZE( boxes ); i++ )
	{
		VisBoxPlanes( dvec3( boxes[i][0], boxes[i][1], boxes[i][2] ), dvec3( boxes[i][3], boxes[i][4], boxes[i][5] ), p );
		ConvexShape cs;
		if ( !ConvexFromPlanes( p.data(), (int)p.size(), cs ) )
			continue;
		s.shapes.push_back( cs );
		for ( size_t k = 0; k < p.size(); k++ )
			p[k].d -= OCC_SHRINK;
		s.occ.push_back( p );
	}
	s.contacts = BwVis_SeamFillers( s.shapes, s.fill, &s.extended, &s.pairs );
	if ( withFill )
		s.occ.insert( s.occ.end(), s.fill.begin(), s.fill.end() );
}

// Segments with known answers in the scene.
struct SeamCase
{
	const char *what;
	dvec3 a, b;
	int want;		// 1 blocked, 0 clear
};
static const double SEAM_LIFT = 1.0 / 256.0;	// FACE_LIFT: the closest the passes get to a surface
static const SeamCase s_SeamCases[] = {
	{ "in the seam plane between A and B (y 10)",            dvec3( -5, 10, 5 ),            dvec3( 15, 10, 5 ),            1 },
	{ "in that plane, 1/64u off it",                           dvec3( -5, 10 + 1.0 / 64, 5 ), dvec3( 15, 10 + 1.0 / 64, 5 ), 1 },
	{ "grazing A's exposed top 1/256u above it",               dvec3( -5, 5, 10 + SEAM_LIFT ), dvec3( 15, 5, 10 + SEAM_LIFT ), 0 },
	{ "grazing the top 1/256u above, along the seam line",     dvec3( -5, 10, 10 + SEAM_LIFT ), dvec3( 15, 10, 10 + SEAM_LIFT ), 0 },
	{ "through the real 1u slot between B and C",              dvec3( -5, 20.5, 5 ),          dvec3( 15, 20.5, 5 ),          0 },
	{ "along B's face in the slot, 1/256u off it",             dvec3( -5, 20 + SEAM_LIFT, 5 ), dvec3( 15, 20 + SEAM_LIFT, 5 ), 0 },
	{ "in the A-D contact (y 0, z 2)",                         dvec3( -5, 0, 2 ),             dvec3( 15, 0, 2 ),             1 },
	{ "on A's face above D (y 0, z 7), exposed",               dvec3( -5, 0, 7 ),             dvec3( 15, 0, 7 ),             0 },
	{ "1/256u off A's face just above D's top (z 4 + 1/256)",  dvec3( -5, -SEAM_LIFT, 4 + SEAM_LIFT ), dvec3( 15, -SEAM_LIFT, 4 + SEAM_LIFT ), 0 },
	{ "in the W1-W2 seam plane (x 0)",                         dvec3( 0, 44, 5 ),             dvec3( 0, 56, 5 ),             1 },
	{ "along the T's edge under the W1-W2 seam (x 0, z 0)",    dvec3( 0, 44, 0 ),             dvec3( 0, 56, 0 ),             1 },
	{ "grazing the floor 1/256u above, under W1",              dvec3( -10, 44, SEAM_LIFT ),   dvec3( -10, 56, SEAM_LIFT ),   1 },
	{ "grazing the floor 1/256u above, beside the walls",      dvec3( -10, 41, SEAM_LIFT ),   dvec3( -10, 47, SEAM_LIFT ),   0 },
	{ "along the line where Q1..Q4 meet (x 105, z 5)",         dvec3( 105, -5, 5 ),           dvec3( 105, 15, 5 ),           1 },
};

static bool SeamBlocked( const std::vector< std::vector<DPlane> > &occ, const dvec3 &a, const dvec3 &b )
{
	for ( size_t o = 0; o < occ.size(); o++ )
	{
		if ( SegmentCrossesConvex( a, b, occ[o].data(), (int)occ[o].size() ) )
			return true;
	}
	return false;
}

// #115: a sloped side can leave the thick filler's column no region while the thinner one
// still has one. A x -12 * OCC_SHRINK..10 under B, whose +x side leans in (x + 3z <= 0): at the
// thick column's height (5 * OCC_SHRINK) B's line sits at x -15 * OCC_SHRINK, past A's edge, at
// the thin one's (3 * OCC_SHRINK) at x -9 * OCC_SHRINK. One filler must come of it, and grown by
// OCC_SHRINK it must still lie in A or B (the filler rule).
static void TestSeamSloped()
{
	std::vector<ConvexShape> shapes( 2 );
	std::vector<DPlane> pa, pb;
	VisBoxPlanes( dvec3( -12 * OCC_SHRINK, 0, -4 ), dvec3( 10, 10, 0 ), pa );
	VisBoxPlanes( dvec3( -10, 0, 0 ), dvec3( 10, 10, 2 ), pb );
	pb[0].n = dvec3( 1, 0, 3 ) * ( 1.0 / sqrt( 10.0 ) );	// VisBoxPlanes' first plane is +x
	pb[0].d = 0;
	bool built = ConvexFromPlanes( pa.data(), (int)pa.size(), shapes[0] ) && ConvexFromPlanes( pb.data(), (int)pb.size(), shapes[1] );
	std::vector< std::vector<DPlane> > fill;
	std::vector<int> pairs;
	int extended = 0;
	int contacts = built ? BwVis_SeamFillers( shapes, fill, &extended, &pairs ) : 0;
	int outside = 0;
	for ( size_t f = 0; f < fill.size(); f++ )
	{
		std::vector<DPlane> grown( fill[f] );
		for ( size_t k = 0; k < grown.size(); k++ )
			grown[k].d += OCC_SHRINK - 1e-5;
		ConvexShape g;
		if ( !ConvexFromPlanes( grown.data(), (int)grown.size(), g ) )
		{
			outside++;
			continue;
		}
		for ( size_t v = 0; v < g.verts.size(); v++ )
		{
			bool inA = true, inB = true;
			for ( size_t k = 0; k < pa.size(); k++ )
				inA = inA && DDot( pa[k].n, g.verts[v] ) <= pa[k].d + 1e-9;
			for ( size_t k = 0; k < pb.size(); k++ )
				inB = inB && DDot( pb[k].n, g.verts[v] ) <= pb[k].d + 1e-9;
			outside += !inA && !inB;
		}
	}
	Check( built && contacts == 1 && fill.size() == 1 && outside == 0,
		   "seam fillers: a sloped side (x + 3z <= 0) over a sliver of A: %d contact(s), %d filler(s) (want 1), %d grown corner(s) "
		   "outside A and B", contacts, (int)fill.size(), outside );
}

static void TestSeamFillers()
{
	SeamScene bare, s;
	BuildSeamScene( bare, false );
	BuildSeamScene( s, true );
	Check( !s.fill.empty() && s.extended >= 1,
		   "seam fillers: %d contacts, %d fillers, %d reaching past an edge", s.contacts, (int)s.fill.size(), s.extended );
	// without fillers the scene leaks (so the checks below measure something)
	Check( !SeamBlocked( bare.occ, s_SeamCases[0].a, s_SeamCases[0].b ) && !SeamBlocked( bare.occ, s_SeamCases[10].a, s_SeamCases[10].b ),
		   "seam fillers: without them a segment in the A-B seam plane and one along the T's edge pass the shrunk boxes" );
	for ( size_t i = 0; i < ARRAYSIZE( s_SeamCases ); i++ )
	{
		const SeamCase &c = s_SeamCases[i];
		bool blocked = SeamBlocked( s.occ, c.a, c.b );
		Check( blocked == ( c.want != 0 ), "seam fillers: segment %s is %s (want %s)", c.what, blocked ? "blocked" : "clear",
			   c.want ? "blocked" : "clear" );
	}
	// the GPU fan shrinks every occluder FAN_GPU_SHRINK more, fillers too: the seam stays closed
	{
		std::vector< std::vector<DPlane> > fan( s.occ );
		for ( size_t o = 0; o < fan.size(); o++ )
			for ( size_t k = 0; k < fan[o].size(); k++ )
				fan[o][k].d -= FAN_GPU_SHRINK;
		bool seam = SeamBlocked( fan, s_SeamCases[0].a, s_SeamCases[0].b );
		bool edge = SeamBlocked( fan, s_SeamCases[10].a, s_SeamCases[10].b );
		bool cross = SeamBlocked( fan, s_SeamCases[13].a, s_SeamCases[13].b );
		Check( seam && edge && cross, "seam fillers: after the GPU fan's further shrink the A-B seam (%s), the T's edge (%s) and the "
			   "four-brush line (%s) stay closed", seam ? "closed" : "open", edge ? "closed" : "open", cross ? "closed" : "open" );
	}
	// the rule: a filler grown by OCC_SHRINK (less rounding) still lies inside the boxes. Points
	// spread through each grown filler, each must be in some box.
	{
		int outside = 0, tested = 0;
		unsigned int rng = 12345u;
		for ( size_t f = 0; f < s.fill.size(); f++ )
		{
			std::vector<DPlane> grown( s.fill[f] );
			for ( size_t k = 0; k < grown.size(); k++ )
				grown[k].d += OCC_SHRINK - 1e-5;
			ConvexShape g;
			if ( !ConvexFromPlanes( grown.data(), (int)grown.size(), g ) )
			{
				outside++;
				continue;
			}
			for ( int k = 0; k < 400; k++ )
			{
				// a random convex combination of the grown filler's vertices, and the vertices themselves
				dvec3 p( 0, 0, 0 );
				if ( k < (int)g.verts.size() )
					p = g.verts[k];
				else
				{
					double sum = 0;
					for ( size_t v = 0; v < g.verts.size(); v++ )
					{
						rng = rng * 1664525u + 1013904223u;
						double w = ( rng >> 8 ) / 16777216.0;
						w = w * w * w * w;	// favour points near one vertex (the corners are where it would fail)
						p += g.verts[v] * w;
						sum += w;
					}
					p = p * ( 1.0 / sum );
				}
				tested++;
				bool in = false;
				for ( size_t b = 0; b < s.shapes.size() && !in; b++ )
				{
					in = p.x >= s.shapes[b].mins.x - 1e-9 && p.y >= s.shapes[b].mins.y - 1e-9 && p.z >= s.shapes[b].mins.z - 1e-9 &&
						 p.x <= s.shapes[b].maxs.x + 1e-9 && p.y <= s.shapes[b].maxs.y + 1e-9 && p.z <= s.shapes[b].maxs.z + 1e-9;
				}
				outside += !in;
			}
		}
		Check( outside == 0 && tested > 0, "seam fillers: every filler grown by OCC_SHRINK lies inside the boxes (%d of %d points outside)",
			   outside, tested );
	}
	// a viewpoint in solid (#76) inside A skips A; the A-B filler's half in A must go with it.
	// The segment runs inside A along the seam, 1/128u from it, and out past A's +x face into air.
	{
		Check( s.pairs.size() == s.fill.size() * 2 && s.occ.size() == s.shapes.size() + s.fill.size(),
			   "seam fillers: one pair of boxes per filler (%d ints for %d fillers)", (int)s.pairs.size(), (int)s.fill.size() );
		const dvec3 a( 5, 10 - 1.0 / 128, 5 ), b( 20, 10 - 1.0 / 128, 5 );
		std::vector<int> held( 1, 0 ), rule( held );
		BwVis_SeamSkips( s.pairs, (int)s.shapes.size(), rule );
		bool blockedHeld = false, blockedRule = false;
		for ( size_t o = 0; o < s.occ.size(); o++ )
		{
			if ( !SegmentCrossesConvex( a, b, s.occ[o].data(), (int)s.occ[o].size() ) )
				continue;
			blockedHeld |= std::find( held.begin(), held.end(), (int)o ) == held.end();
			blockedRule |= std::find( rule.begin(), rule.end(), (int)o ) == rule.end();
		}
		Check( blockedHeld && !blockedRule && rule.size() > held.size(),
			   "seam fillers: a camera in A skipping only A is %s along the A-B seam (want blocked); with the fillers that join A "
			   "skipped too (%d skipped) it is %s (want clear)", blockedHeld ? "blocked" : "clear", (int)rule.size(),
			   blockedRule ? "blocked" : "clear" );
	}
}

static bool InPieces( const std::vector<EyePiece> &pieces, const dvec3 &p )
{
	for ( size_t i = 0; i < pieces.size(); i++ )
	{
		bool in = true;
		for ( size_t k = 0; k < pieces[i].planes.size() && in; k++ )
			in = DDot( pieces[i].planes[k].n, p ) <= pieces[i].planes[k].d + 1e-9;
		if ( in )
			return true;
	}
	return false;
}

static double PiecesVolume( const std::vector<EyePiece> &pieces )
{
	double v = 0;
	for ( size_t i = 0; i < pieces.size(); i++ )
	{
		ConvexShape s;
		if ( !ConvexFromPoints( pieces[i].verts.data(), (int)pieces[i].verts.size(), s ) )
			continue;
		dvec3 c = s.verts[0];
		for ( size_t f = 0; f < s.faces.size(); f++ )
		{
			const std::vector<int> &fv = s.faces[f];
			for ( size_t k = 1; k + 1 < fv.size(); k++ )
				v += DDot( s.verts[fv[0]] - c, DCross( s.verts[fv[k]] - c, s.verts[fv[k + 1]] - c ) ) / 6.0;
		}
	}
	return v;
}

// Every vertex of every piece lies inside every one of its own planes. The
// volume checks rebuild a hull from the vertices, so they cannot see a piece
// whose plane list disagrees with its vertices.
static bool PiecesPlaneConsistent( const std::vector<EyePiece> &pieces, double eps = 1e-9 )
{
	for ( size_t i = 0; i < pieces.size(); i++ )
		for ( size_t v = 0; v < pieces[i].verts.size(); v++ )
			for ( size_t k = 0; k < pieces[i].planes.size(); k++ )
				if ( DDot( pieces[i].planes[k].n, pieces[i].verts[v] ) > pieces[i].planes[k].d + eps )
					return false;
	return true;
}

// h7b/h7c: the leaves for the viewpoint-in-solid checks (BwVis_SolidEntryBlocks). The
// occluder is the box 0..10 on every axis; ctx picks what else is solid:
// 0: x < 0 (a solid slab touching the box's -x side)
// 1: x < -2 (the slab, then a 2u open gap before the box)
// 2: x < -3, and the sliver -0.5 < x < 0 (a thin nodraw brush against the box)
// 3: nothing (an open leaf even inside the box)
// 4: the slab x < 0 below z = 20 only, plus the strip 11 < x < 13 between z 0 and 20
//    (the mirror check: a non-occluder solid in front of the leg-B blocker)
static bool SolidLeafTest( const dvec3 &p, void *ctx )
{
	int mode = (int)(intptr_t)ctx;
	bool inBox = p.x > 0 && p.x < 10 && p.y > 0 && p.y < 10 && p.z > 0 && p.z < 10;
	switch ( mode )
	{
	case 0: return p.x < 0 || inBox;
	case 1: return p.x < -2 || inBox;
	case 2: return p.x < -3 || ( p.x > -0.5 && p.x < 0 ) || inBox;
	case 4: return ( p.x < 0 && p.z < 20 ) || ( p.x > 11 && p.x < 13 && p.z > 0 && p.z < 20 );
	default: return false;
	}
}

struct SolidTestWorld
{
	std::vector<DPlane> planes;
	std::vector<int> ofs, all, skip;
	intptr_t mode;
};

static bool SolidTestBlocked( const dvec3 &v, const dvec3 &p, void *ctx )
{
	const SolidTestWorld &w = *(const SolidTestWorld *)ctx;
	return BwVis_SolidSegmentBlocked( v, p, w.planes.data(), w.ofs.data(), w.all.data(), (int)w.all.size(), w.skip,
									  SolidLeafTest, (void *)w.mode );
}

static void TestVis()
{
	Msg( "(h) visibility: feet column, subtraction, rays, mirrors\n" );
	std::vector<DPlane> planes;

	// h1: a 64x64 floor at z 0, column rise 57.33; a slab z 40..56 over x 0..32
	// (shrunk 1/32 by FeetColumn) cuts the left half down to z 40 - 1/32 only
	DWinding floor;
	floor.push_back( dvec3( 0, 0, 0 ) );
	floor.push_back( dvec3( 64, 0, 0 ) );
	floor.push_back( dvec3( 64, 64, 0 ) );
	floor.push_back( dvec3( 0, 64, 0 ) );
	DPlane fp;
	fp.n = dvec3( 0, 0, 1 );
	fp.d = 0;
	const double rise = 57.33, sh = 1.0 / 32.0;
	GrownPolytope slab, below, beside;
	VisBoxPlanes( dvec3( -16, -16, 40 ), dvec3( 32, 80, 56 ), planes );
	PolytopeFromPlanes( planes, slab );
	VisBoxPlanes( dvec3( -16, -16, -16 ), dvec3( 80, 80, 0 ), planes );	// the floor's own solid
	PolytopeFromPlanes( planes, below );
	VisBoxPlanes( dvec3( 64, -16, -16 ), dvec3( 96, 80, 200 ), planes );	// a wall touching the edge
	PolytopeFromPlanes( planes, beside );
	std::vector<const GrownPolytope *> obs;
	obs.push_back( &below );
	obs.push_back( &beside );
	std::vector<EyePiece> col;
	FeetColumn( floor, fp, rise, obs, sh, col );
	double v = PiecesVolume( col );
	Check( Near( v, 64.0 * 64.0 * rise, 1e-6 ), "column with only the floor's solid and a touching wall: the full prism, volume %.4f (want %.4f)",
		   v, 64.0 * 64.0 * rise );
	obs.push_back( &slab );
	col.clear();
	FeetColumn( floor, fp, rise, obs, sh, col );
	v = PiecesVolume( col );
	// shrunk, the slab spans x up to 32 - 1/32 from z 40 + 1/32: that strip of
	// the floor keeps the column below it, the rest the full rise
	double want = ( 32.0 - sh ) * 64.0 * ( 40.0 + sh ) + ( 32.0 + sh ) * 64.0 * rise;
	Check( Near( v, want, 1e-6 ), "slab z 40..56 over half the floor: volume %.4f (want %.4f)", v, want );
	Check( InPieces( col, dvec3( 16, 32, 30 ) ) && !InPieces( col, dvec3( 16, 32, 45 ) ) && !InPieces( col, dvec3( 16, 32, 56.5 ) ) &&
			   InPieces( col, dvec3( 48, 32, 50 ) ),
		   "under the slab the column stops at its bottom; nothing above it; the open half keeps its rise" );
	Check( PiecesPlaneConsistent( col ), "slab column: every piece's vertices satisfy its planes" );

	// h1b: two overlapping obstacles, in both list orders: slab A (x -16..32,
	// z 40..56) and a lower slab B (x 16..48, z 30..50) that runs into it. Each
	// floor strip keeps the column up to the lowest obstacle above it.
	{
		GrownPolytope A, B;
		VisBoxPlanes( dvec3( -16, -16, 40 ), dvec3( 32, 80, 56 ), planes );
		PolytopeFromPlanes( planes, A );
		VisBoxPlanes( dvec3( 16, -16, 30 ), dvec3( 48, 80, 50 ), planes );
		PolytopeFromPlanes( planes, B );
		double owant = 64.0 * ( ( 16.0 + sh ) * ( 40.0 + sh ) + ( 32.0 - 2.0 * sh ) * ( 30.0 + sh ) + ( 16.0 + sh ) * rise );
		for ( int order = 0; order < 2; order++ )
		{
			std::vector<const GrownPolytope *> two;
			two.push_back( order ? &B : &A );
			two.push_back( order ? &A : &B );
			std::vector<EyePiece> oc;
			FeetColumn( floor, fp, rise, two, sh, oc );
			double ov = PiecesVolume( oc );
			Check( Near( ov, owant, 1e-6 ) && PiecesPlaneConsistent( oc ) && InPieces( oc, dvec3( 8, 32, 35 ) ) &&
					   !InPieces( oc, dvec3( 24, 32, 35 ) ) && InPieces( oc, dvec3( 24, 32, 25 ) ) && InPieces( oc, dvec3( 56, 32, 55 ) ),
				   "overlapping slabs, %s first: volume %.4f (want %.4f), %d pieces, planes consistent", order ? "B" : "A", ov, owant,
				   (int)oc.size() );
		}
	}

	// h2: a ceiling above the rise changes nothing; one at z 20 over all of it caps it
	GrownPolytope high, low;
	VisBoxPlanes( dvec3( -16, -16, 60 ), dvec3( 80, 80, 80 ), planes );
	PolytopeFromPlanes( planes, high );
	VisBoxPlanes( dvec3( -16, -16, 20 ), dvec3( 80, 80, 36 ), planes );
	PolytopeFromPlanes( planes, low );
	obs.assign( 1, &high );
	col.clear();
	FeetColumn( floor, fp, rise, obs, sh, col );
	Check( Near( PiecesVolume( col ), 64.0 * 64.0 * rise, 1e-6 ), "ceiling at z 60 (above the rise): the full prism" );
	obs.assign( 1, &low );
	col.clear();
	FeetColumn( floor, fp, rise, obs, sh, col );
	Check( Near( PiecesVolume( col ), 64.0 * 64.0 * ( 20.0 + sh ), 1e-6 ) && !InPieces( col, dvec3( 32, 32, 40 ) ),
		   "ceiling at z 20 over the floor: the column ends there, nothing past the ceiling (volume %.4f)", PiecesVolume( col ) );

	// h3: a sloped floor (n.z 0.8): the prism is the plane raised by 'rise' along z
	DWinding ramp;
	ramp.push_back( dvec3( 0, 0, 0 ) );
	ramp.push_back( dvec3( 64, 0, 48 ) );
	ramp.push_back( dvec3( 64, 64, 48 ) );
	ramp.push_back( dvec3( 0, 64, 0 ) );
	DPlane rp;
	rp.n = dvec3( -0.6, 0, 0.8 );
	rp.d = 0;
	obs.clear();
	col.clear();
	FeetColumn( ramp, rp, rise, obs, sh, col );
	Check( Near( PiecesVolume( col ), 64.0 * 64.0 * rise, 1e-6 ) && InPieces( col, dvec3( 32, 32, 24 + rise - 0.01 ) ) &&
			   !InPieces( col, dvec3( 32, 32, 24 + rise + 0.01 ) ),
		   "ramp n.z 0.8: the column's top is the ramp raised %.2fu along z", rise );

	// h4: polyhedron subtraction: a 10^3 cube minus a 5^3 corner cube
	EyePiece cube;
	VisBoxPlanes( dvec3( 0, 0, 0 ), dvec3( 10, 10, 10 ), planes );
	bool ok = EyePolyFromPlanes( planes, cube );
	std::vector<DPlane> corner;
	VisBoxPlanes( dvec3( 5, 5, 5 ), dvec3( 20, 20, 20 ), corner );
	std::vector<EyePiece> rest;
	if ( ok )
		EyePolySubtract( cube, corner, rest );
	Check( ok && Near( PiecesVolume( rest ), 1000.0 - 125.0, 1e-6 ) && PiecesPlaneConsistent( rest ) && !InPieces( rest, dvec3( 7, 7, 7 ) ) && InPieces( rest, dvec3( 2, 7, 7 ) ),
		   "cube minus a corner cube: %d pieces, volume %.4f (want 875)", (int)rest.size(), PiecesVolume( rest ) );

	// h5: an open plane set is not a piece (a slab missing one side)
	VisBoxPlanes( dvec3( 0, 0, 0 ), dvec3( 10, 10, 10 ), planes );
	planes.erase( planes.begin() );	// drop x <= 10
	EyePiece open;
	Check( !EyePolyFromPlanes( planes, open ), "a box missing its +x plane is open: no piece" );

	// h5b: a plane given twice (the front clip of a second obstacle whose face is
	// flush with the first's) is one side, not none: the piece keeps a bounded
	// plane set, so a later clip of it still builds
	VisBoxPlanes( dvec3( 0, 0, 0 ), dvec3( 10, 10, 10 ), planes );
	planes.push_back( planes[4] );	// z <= 10 again
	EyePiece twice, half;
	ok = EyePolyFromPlanes( planes, twice );
	bool clipped = ok && EyePolyClip( twice, dvec3( 1, 0, 0 ), 5.0, half );
	Check( ok && clipped && Near( PiecesVolume( std::vector<EyePiece>( 1, half ) ), 500.0, 1e-6 ),
		   "a box with its +z plane twice keeps one of them; clipped at x 5: %s (want a 500 piece)",
		   clipped ? "builds" : "open" );

	// h5c: the same through FeetColumn: two slabs with flush undersides at z 40
	// over the floor (x 16..48, then x 0..32), then a pillar through the column
	// under them at x 20..24 (all shrunk 1/32). The pieces under the slabs carry
	// the first slab's bottom plane when the second's is clipped, and the pillar
	// clips them again: nothing under the slabs may be lost (with both copies of
	// the plane dropped, 3 pieces and 140884.6 u^3 were left).
	{
		GrownPolytope A, B, C;
		VisBoxPlanes( dvec3( -16, -16, 40 ), dvec3( 32, 80, 56 ), planes );
		PolytopeFromPlanes( planes, A );
		VisBoxPlanes( dvec3( 16, -16, 40 ), dvec3( 48, 80, 70 ), planes );
		PolytopeFromPlanes( planes, B );
		VisBoxPlanes( dvec3( 20, -16, -16 ), dvec3( 24, 80, 200 ), planes );
		PolytopeFromPlanes( planes, C );
		std::vector<const GrownPolytope *> flush;
		flush.push_back( &B );
		flush.push_back( &A );
		flush.push_back( &C );
		std::vector<EyePiece> fc;
		FeetColumn( floor, fp, rise, flush, sh, fc );
		double fv = PiecesVolume( fc );
		double fwant = 64.0 * ( ( 48.0 - sh - ( 4.0 - 2.0 * sh ) ) * ( 40.0 + sh ) + ( 16.0 + sh ) * rise );
		Check( Near( fv, fwant, 1e-6 ) && PiecesPlaneConsistent( fc ) && InPieces( fc, dvec3( 8, 32, 20 ) ) && InPieces( fc, dvec3( 40, 32, 20 ) ) && !InPieces( fc, dvec3( 22, 32, 20 ) ),
			   "flush slab undersides then a pillar: volume %.4f (want %.4f), %d pieces", fv, fwant, (int)fc.size() );
	}

	// h9: tier B sampling of a piece (SampleEyePiece). Lattice columns 8u apart
	// with points at both ends of each column's span.
	{
		std::vector<EyeSample> s;
		std::set< std::pair<double, double> > cols;
		// h9a: a 32^2 box under a top sloped up along x (z = 16 + x / 4) whose +x
		// side sits 1e-9 inside x 32 (clip rounding): every column of the 5x5
		// lattice, the x 32 ones included, from z 0 to the top (24 at x 32)
		VisBoxPlanes( dvec3( 0, 0, 0 ), dvec3( 32, 32, 16 ), planes );
		planes[0].d = 32.0 - 1e-9;	// x <= 32
		planes[4].n = dvec3( -0.25, 0, 1 );
		double len = DNormalize( planes[4].n );
		planes[4].d = 16.0 / len;
		EyePiece slope;
		ok = EyePolyFromPlanes( planes, slope );
		double zmax = -1e300;
		if ( ok )
			SampleEyePiece( slope, 8.0, s );
		for ( size_t i = 0; i < s.size(); i++ )
		{
			zmax = MAX( zmax, (double)s[i].z );
			if ( fmod( s[i].x, 8.0 ) == 0 && fmod( s[i].y, 8.0 ) == 0 && fabs( s[i].z ) < 1e-6 )
				cols.insert( std::make_pair( (double)s[i].x, (double)s[i].y ) );
		}
		Check( ok && cols.size() == 25 && Near( zmax, 24.0, 1e-4 ),
			   "sloped box, +x side 1e-9 inside the lattice line: %d of 25 columns, top %.4f (want 24)", (int)cols.size(), zmax );

		// h9b: a strip 2u wide between lattice lines (x 1..3, y 0..64): columns
		// along its length, none more than 8u apart, not just its 8 corners
		VisBoxPlanes( dvec3( 1, 0, 0 ), dvec3( 3, 64, 16 ), planes );
		EyePiece strip;
		ok = EyePolyFromPlanes( planes, strip );
		s.clear();
		if ( ok )
			SampleEyePiece( strip, 8.0, s );
		std::vector<double> ys;
		for ( size_t i = 0; i < s.size(); i++ )
			ys.push_back( s[i].y );
		std::sort( ys.begin(), ys.end() );
		double gap = ys.empty() ? 1e300 : ys[0];
		for ( size_t i = 1; i < ys.size(); i++ )
			gap = MAX( gap, ys[i] - ys[i - 1] );
		if ( !ys.empty() )
			gap = MAX( gap, 64.0 - ys.back() );
		Check( ok && gap <= 8.0 + 1e-6, "strip 2u wide between lattice lines: largest gap along it %.2f (want <= 8)", gap );

		// h9c: a wedge whose top meets the floor 1e-9 short of the lattice line x 16
		// (top z = 16 - x - 1e-9): the x 16 columns are a point each, not dropped
		VisBoxPlanes( dvec3( 0, 0, 0 ), dvec3( 16, 16, 16 ), planes );
		planes[4].n = dvec3( 1, 0, 1 );
		len = DNormalize( planes[4].n );
		planes[4].d = ( 16.0 - 1e-9 ) / len;
		EyePiece wedge;
		ok = EyePolyFromPlanes( planes, wedge );
		s.clear();
		if ( ok )
			SampleEyePiece( wedge, 8.0, s );
		bool edge = false;
		for ( size_t i = 0; i < s.size(); i++ )
			edge = edge || ( s[i].x == 16.0f && s[i].y == 8.0f && fabs( s[i].z ) < 1e-6 );
		Check( ok && edge, "wedge closing 1e-9 short of x 16: a sample at (16 8 0) (%s)", edge ? "found" : "missing" );
	}

	// h6: segments against a convex: through, touching, ending on the face
	VisBoxPlanes( dvec3( 0, 0, 0 ), dvec3( 10, 10, 10 ), planes );
	Check( SegmentCrossesConvex( dvec3( -5, 5, 5 ), dvec3( 15, 5, 5 ), planes.data(), (int)planes.size() ), "segment through a box crosses it" );
	Check( !SegmentCrossesConvex( dvec3( -5, 10, 5 ), dvec3( 15, 10, 5 ), planes.data(), (int)planes.size() ),
		   "segment along a face (y = 10) only touches" );
	Check( !SegmentCrossesConvex( dvec3( -5, 5, 5 ), dvec3( 0, 5, 5 ), planes.data(), (int)planes.size() ),
		   "segment ending on the face (x = 0) does not enter" );
	Check( !SegmentCrossesConvex( dvec3( -5, 12, 5 ), dvec3( 15, 12, 5 ), planes.data(), (int)planes.size() ), "segment beside the box misses" );

	// h6b: seam fillers (#33, #40): see SeamScene
	TestSeamFillers();
	TestSeamSloped();

	// h7: f16's mirror: eye (-32, 256, 64) in the water plane z = -192
	dvec3 m = MirrorPoint( dvec3( -32, 256, 64 ), dvec3( 0, 0, 1 ), -192 );
	Check( Near( m.x, -32 ) && Near( m.y, 256 ) && Near( m.z, -448 ), "mirror of (-32 256 64) in z = -192 is (-32 256 -448) (got %g %g %g)",
		   m.x, m.y, m.z );

	// h7a: a displacement mirror's window (#78) about the plane z = 0, eye (-50 0 10)
	// (mirrored: (-50 0 -10)), sample (50 0 190), so q = (-45 0 0). A flat displacement
	// raised to z 100 over x 0..100: the real eye's ray through q heads down and misses it,
	// the sample's segment to the mirrored eye enters it at (5.005 0 100.01); the one pair
	// runs sample -> there and eye -> where its line enters the bounds (z 99.99)
	{
		const double tol = 1e-2;
		dvec3 eye( -50, 0, 10 ), mir( -50, 0, -10 ), sp( 50, 0, 190 ), q( -45, 0, 0 );
		VisSegment legs[4];
		int n = BwVis_DispMirrorPairs( dvec3( 0, -50, 100 ), dvec3( 100, 50, 100 ), tol, sp, q, mir, eye, legs );
		Check( n == 1 && NearV( legs[0].a, sp, 1e-9 ) && NearV( legs[0].b, dvec3( 5.005, 0, 100.01 ), 1e-9 ) &&
				   NearV( legs[1].b, eye, 1e-9 ) && Near( legs[1].a.z, 99.99, 1e-9 ),
			   "raised displacement mirror: one window, by the mirrored eye's line (%d pairs, sample leg to %g %g %g, eye leg from "
			   "z %g)", n, n > 0 ? legs[0].b.x : 0.0, n > 0 ? legs[0].b.y : 0.0, n > 0 ? legs[0].b.z : 0.0, n > 0 ? legs[1].a.z : 0.0 );
		// the same displacement lowered into the plane (z 0, x -100..100): both lines cross it at
		// q, the real eye's pair first, sample -> q, then q's side of the bounds -> eye
		n = BwVis_DispMirrorPairs( dvec3( -100, -50, 0 ), dvec3( 100, 50, 0 ), tol, sp, q, mir, eye, legs );
		Check( n == 2 && NearV( legs[0].b, q, 1e-9 ) && NearV( legs[1].b, eye, 1e-9 ) && legs[1].a.z < tol + 1e-9,
			   "displacement in the plane: both windows, the real eye's through q first (%d pairs)", n );
		// raised over x 200..300 instead: neither line meets it, no window
		n = BwVis_DispMirrorPairs( dvec3( 200, -50, 100 ), dvec3( 300, 50, 100 ), tol, sp, q, mir, eye, legs );
		Check( n == 0, "displacement off both lines: no window (%d pairs)", n );
	}

	// h7b: a camera in a solid leaf (#76) against the box occluder 0..10, shrunk by
	// OCC_SHRINK as BuildOccluders shrinks it. Entered from a solid leaf the box's
	// side is not drawn (vbsp): clear; entered from an open leaf: blocked.
	{
		std::vector<DPlane> box;
		VisBoxPlanes( dvec3( 0, 0, 0 ), dvec3( 10, 10, 10 ), box );
		for ( size_t k = 0; k < box.size(); k++ )
			box[k].d -= OCC_SHRINK;
		const DPlane *bp = box.data();
		const int nb = (int)box.size();
		const dvec3 cam( -5, 5, 5 ), far( 15, 5, 5 ), slant( 15, 7, 6 );
		Check( !BwVis_SolidEntryBlocks( cam, far, bp, nb, SolidLeafTest, (void *)0 ),
			   "solid view: entering the box from the solid slab touching it is clear" );
		Check( BwVis_SolidEntryBlocks( cam, far, bp, nb, SolidLeafTest, (void *)1 ),
			   "solid view: entering the box across an open gap is blocked" );
		Check( !BwVis_SolidEntryBlocks( cam, slant, bp, nb, SolidLeafTest, (void *)0 ),
			   "solid view: a slanted segment entering from the touching slab is clear" );
		Check( !BwVis_SolidEntryBlocks( cam, far, bp, nb, SolidLeafTest, (void *)2 ),
			   "solid view: entering the box from a 0.5u solid sliver against it is clear" );
		Check( !BwVis_SolidEntryBlocks( cam, dvec3( -1, 5, 5 ), bp, nb, SolidLeafTest, (void *)1 ),
			   "solid view: a segment that stops short of the box is clear" );
		// the camera inside the box: skipped (it holds the camera) it is clear; not
		// skipped, with no solid leaf around it, it would block
		SolidTestWorld w;
		w.planes = box;
		w.ofs.push_back( 0 );
		w.ofs.push_back( nb );
		w.all.push_back( 0 );
		w.mode = 3;
		const dvec3 inside( 5, 5, 5 );
		Check( SolidTestBlocked( inside, far, &w ), "solid view: from inside an open-leaf box, not skipped, the box blocks" );
		w.skip.push_back( 0 );
		Check( !SolidTestBlocked( inside, far, &w ),
			   "solid view: from inside the box that holds the camera (skipped) it is clear" );
		w.skip.clear();

		// h7c: BwVis_SolidCamSees. Direct: the face x = 20 facing -x behind the box.
		FaceSample fs;
		fs.p = dvec3( 19.75, 5, 5 );
		fs.n = dvec3( -1, 0, 0 );
		long long pairs = 0;
		w.mode = 0;
		Check( BwVis_SolidCamSees( cam, dvec3( -1, 0, 0 ), -20, false, &fs, 1, NULL, SolidTestBlocked, &w, pairs, 1000 ),
			   "solid view: the camera in the slab sees the face behind the box (box entered from solid)" );
		w.mode = 1;
		pairs = 0;
		Check( !BwVis_SolidCamSees( cam, dvec3( -1, 0, 0 ), -20, false, &fs, 1, NULL, SolidTestBlocked, &w, pairs, 1000 ),
			   "solid view: across an open gap the box hides that face" );
		pairs = 0;
		Check( !BwVis_SolidCamSees( dvec3( 25, 5, 5 ), dvec3( -1, 0, 0 ), -20, false, &fs, 1, NULL, SolidTestBlocked, &w, pairs,
									1000 ),
			   "solid view: a camera behind the face does not see it" );

		// Mirrored, as f16: the mirror z = 0 (polygon x, y -50..50), the face the underside
		// z = 20 of a ledge (faces -z), its sample (20, 5, 19.75). The camera (-20, 5, 30)
		// is above the face's plane: not in front of it directly. Its image (-20, 5, -30)
		// is: the path sample -> Q -> camera crosses z = 0 at Q (4.12, 5, 0).
		DWinding mw;
		mw.push_back( dvec3( -50, -50, 0 ) );
		mw.push_back( dvec3( 50, -50, 0 ) );
		mw.push_back( dvec3( 50, 50, 0 ) );
		mw.push_back( dvec3( -50, 50, 0 ) );
		BwSolidMirror mir;
		mir.n = dvec3( 0, 0, 1 );
		mir.d = 0;
		mir.w = &mw;
		mir.tol = 1e-2;
		FaceSample us;
		us.p = dvec3( 20, 5, 19.75 );
		us.n = dvec3( 0, 0, -1 );
		const dvec3 mcam( -20, 5, 30 );
		SolidTestWorld open;
		open.ofs.push_back( 0 );
		open.mode = 3;
		pairs = 0;
		Check( !BwVis_SolidCamSees( mcam, dvec3( 0, 0, -1 ), -20, false, &us, 1, NULL, SolidTestBlocked, &open, pairs, 1000 ),
			   "solid view: the underside is not seen directly from above its plane" );
		pairs = 0;
		Check( BwVis_SolidCamSees( mcam, dvec3( 0, 0, -1 ), -20, false, &us, 1, &mir, SolidTestBlocked, &open, pairs, 1000 ),
			   "solid view: the underside is seen in the mirror" );
		// A blocker on leg B (Q -> sample): the box 13..15 x, 0..10 y, 0..20 z. From
		// an open leaf it hides the underside; entered from the solid strip 11..13 in
		// front of it (mode 4) it does not, on leg B as on leg A.
		SolidTestWorld legB;
		VisBoxPlanes( dvec3( 13, 0, 0 ), dvec3( 15, 10, 20 ), legB.planes );
		for ( size_t k = 0; k < legB.planes.size(); k++ )
			legB.planes[k].d -= OCC_SHRINK;
		legB.ofs.push_back( 0 );
		legB.ofs.push_back( (int)legB.planes.size() );
		legB.all.push_back( 0 );
		legB.mode = 3;
		pairs = 0;
		Check( !BwVis_SolidCamSees( mcam, dvec3( 0, 0, -1 ), -20, false, &us, 1, &mir, SolidTestBlocked, &legB, pairs, 1000 ),
			   "solid view: a drawn blocker between Q and the underside hides it in the mirror" );
		legB.mode = 4;
		pairs = 0;
		Check( BwVis_SolidCamSees( mcam, dvec3( 0, 0, -1 ), -20, false, &us, 1, &mir, SolidTestBlocked, &legB, pairs, 1000 ),
			   "solid view: that blocker entered from a solid leaf (no drawn side) does not hide it" );
	}

	// h8: the fan directions are unit, cover the sphere evenly (mean ~ 0), and
	// the octahedral bins tell the axes apart
	dvec3 mean( 0, 0, 0 );
	bool unit = true;
	for ( int i = 0; i < 1024; i++ )
	{
		dvec3 d = FibonacciDir( i, 1024 );
		unit = unit && Near( DLength( d ), 1.0, 1e-12 );
		mean += d;
	}
	mean = mean * ( 1.0 / 1024 );
	Check( unit && DLength( mean ) < 1e-2, "1024 Fibonacci directions: unit, mean length %.5f", DLength( mean ) );
	int bx = OctBin( dvec3( 1, 0, 0 ), 32 ), bnx = OctBin( dvec3( -1, 0, 0 ), 32 ), bz = OctBin( dvec3( 0, 0, 1 ), 32 ),
		bnz = OctBin( dvec3( 0, 0, -1 ), 32 ), by = OctBin( dvec3( 0, 1, 0 ), 32 );
	Check( bx != bnx && bz != bnz && bx != bz && by != bz && bnz >= 0 && bnz < 1024 && bx < 1024,
		   "octahedral bins of +x -x +y +z -z differ (%d %d %d %d %d)", bx, bnx, by, bz, bnz );

	// h9: the candidate cap. Runs of 10, 1000, 5000 and 40 eyes cut to 100: the 10 stay whole and the
	// rest share what is left evenly (30 each); a lone eye next to a huge run is not dropped
	{
		const int sizes[4] = { 10, 1000, 5000, 40 };
		std::vector<int> eyes, starts;
		for ( int r = 0; r < 4; r++ )
		{
			starts.push_back( (int)eyes.size() );
			for ( int k = 0; k < sizes[r]; k++ )
				eyes.push_back( (int)eyes.size() );
		}
		CapCandidateEyes( eyes, starts, 100 );
		int got[4] = { 0, 0, 0, 0 };
		bool ascending = true;
		for ( size_t i = 0; i < eyes.size(); i++ )
		{
			got[eyes[i] < 10 ? 0 : eyes[i] < 1010 ? 1 : eyes[i] < 6010 ? 2 : 3]++;
			ascending = ascending && ( i == 0 || eyes[i] > eyes[i - 1] );
		}
		Check( eyes.size() == 100 && got[0] == 10 && got[1] == 30 && got[2] == 30 && got[3] == 30 && ascending,
			   "cap 100 over runs of 10 / 1000 / 5000 / 40: kept %d / %d / %d / %d (want 10 / 30 / 30 / 30), in order",
			   got[0], got[1], got[2], got[3] );

		eyes.clear();
		starts.clear();
		const int sizes2[3] = { 1, 3, 100000 };
		for ( int r = 0; r < 3; r++ )
		{
			starts.push_back( (int)eyes.size() );
			for ( int k = 0; k < sizes2[r]; k++ )
				eyes.push_back( (int)eyes.size() );
		}
		CapCandidateEyes( eyes, starts, 10 );
		int got2[3] = { 0, 0, 0 };
		for ( size_t i = 0; i < eyes.size(); i++ )
			got2[eyes[i] < 1 ? 0 : eyes[i] < 4 ? 1 : 2]++;
		Check( eyes.size() == 10 && got2[0] == 1 && got2[1] == 3 && got2[2] == 6,
			   "cap 10 over runs of 1 / 3 / 100000: kept %d / %d / %d (want 1 / 3 / 6)", got2[0], got2[1], got2[2] );

		// more clusters than the cap: one eye from each of an even subset; and a list that fits is left alone
		eyes.clear();
		starts.clear();
		for ( int r = 0; r < 6; r++ )
		{
			starts.push_back( (int)eyes.size() );
			for ( int k = 0; k < 20; k++ )
				eyes.push_back( (int)eyes.size() );
		}
		std::vector<int> fits( eyes.begin(), eyes.begin() + 20 ), fitStart( 1, 0 );
		CapCandidateEyes( eyes, starts, 3 );
		CapCandidateEyes( fits, fitStart, 20 );
		bool distinct = eyes.size() == 3 && eyes[0] / 20 != eyes[1] / 20 && eyes[1] / 20 != eyes[2] / 20;
		Check( distinct && fits.size() == 20, "6 clusters, cap 3: %d eyes from distinct clusters; 20 eyes under a cap of 20 are all kept (%d)",
			   (int)eyes.size(), (int)fits.size() );
	}
}

//-----------------------------------------------------------------------------
// (i) Tier B's GPU segments against the CPU: the face and mirror passes launch a
//     straight segment as a zero-gravity arc of the arc kernel over BW_MESH_VIS,
//     the shrunk occluders as triangles. Two boxes stand in for the occluders.
//-----------------------------------------------------------------------------
static bool CpuSegmentBlocked( const std::vector< std::vector<DPlane> > &occ, const dvec3 &a, const dvec3 &b )
{
	for ( size_t o = 0; o < occ.size(); o++ )
	{
		if ( SegmentCrossesConvex( a, b, occ[o].data(), (int)occ[o].size() ) )
			return true;
	}
	return false;
}

static void TestVisGpu()
{
	Msg( "(i) visibility: GPU segments against the CPU\n" );
	const float g = 600.0f;
	// the encoding, which needs no GPU: the arc's height at t is oz + vz t - g0 t^2 / 2 with g0 =
	// g + dg0 = 0, so t = 0 and t = 1 are the two ends exactly and nothing curves in between
	{
		BwArc arc;
		dvec3 a( 12.5, -3.25, 40 ), b( -60, 77.125, 5.5 );
		BwVis_EncodeSegment( arc, a, b, g );
		const float g0 = g + arc.dg0;
		const float half = 0.5f;
		float zmid = arc.oz + arc.vz * half - 0.5f * g0 * half * half;
		Check( arc.ox == (float)a.x && arc.oy == (float)a.y && arc.oz == (float)a.z && arc.vx == (float)( b.x - a.x ) &&
				   arc.vy == (float)( b.y - a.y ) && arc.vz == (float)( b.z - a.z ) && g0 == 0.0f && arc.tEnd == 1.0f &&
				   arc.nSeg[0] == 1 && arc.nSeg[1] == 0 && arc.tKick == 0.0f && arc.mesh == BW_MESH_VIS &&
				   zmid == (float)( ( a.z + b.z ) * 0.5 ),
			   "a straight segment encodes as a zero-gravity arc of one chord on the sight mesh (mid height %g)", zmid );
	}

	if ( !BwOptix_Init() )
	{
		Msg( "  (GPU segment checks skipped: no CUDA device / OptiX)\n" );
		s_bVisGpuSkipped = true;
		return;
	}
	// every check from here on exists only with a GPU (#44)
	int before = s_nChecks;
	TestVisGpuDevice( g );
	s_nGpuChecks += s_nChecks - before;
}

// The device half of TestVisGpu: known-answer segments, the sweep kernel and the seam scene on the GPU.
static void TestVisGpuDevice( const float g )
{
	std::vector< std::vector<DPlane> > occ( 2 );
	VisBoxPlanes( dvec3( 0, 0, 0 ), dvec3( 10, 10, 10 ), occ[0] );
	VisBoxPlanes( dvec3( 40, -20, 0 ), dvec3( 60, 20, 30 ), occ[1] );
	std::vector<float> mv;
	std::vector<unsigned int> mt;
	for ( size_t o = 0; o < occ.size(); o++ )
	{
		ConvexShape cs;
		if ( !ConvexFromPlanes( occ[o].data(), (int)occ[o].size(), cs ) )
			continue;
		unsigned int base = (unsigned int)( mv.size() / 3 );
		for ( size_t k = 0; k < cs.verts.size(); k++ )
		{
			mv.push_back( (float)cs.verts[k].x );
			mv.push_back( (float)cs.verts[k].y );
			mv.push_back( (float)cs.verts[k].z );
		}
		for ( size_t f = 0; f < cs.faces.size(); f++ )
		{
			for ( size_t k = 1; k + 1 < cs.faces[f].size(); k++ )
			{
				mt.push_back( base + cs.faces[f][0] );
				mt.push_back( base + cs.faces[f][k] );
				mt.push_back( base + cs.faces[f][k + 1] );
			}
		}
	}
	if ( !BwOptix_BuildMesh( BW_MESH_VIS, mv, mt ) )
	{
		BwOptix_Shutdown();
		Check( false, "the sight mesh of two boxes builds on the GPU" );
		return;
	}

	// i1: known answers, each also compared with the CPU (SegmentCrossesConvex, what bw_vis traces on
	// -cpuvis). A segment that touches a surface exactly is free on the CPU, but the GPU decides it in
	// float along the triangle edges it meets, so those are only reported (below); the touching cases
	// asserted are the closest the passes get: FACE_LIFT (1/256u) off the surface, outside and inside.
	// Every coordinate is exact in float, so a disagreement is the encoding or the mesh, not rounding.
	const double lift = 1.0 / 256.0;
	struct Case
	{
		const char *what;
		dvec3 a, b;
		int want;		// 1 blocked, 0 clear
	};
	const Case cases[] = {
		{ "above both boxes",                   dvec3( -20, 5, 50 ),         dvec3( 80, 5, 50 ),          0 },
		{ "beside the first box (y 12)",        dvec3( -5, 12, 5 ),          dvec3( 15, 12, 5 ),          0 },
		{ "through the first box",              dvec3( -5, 5, 5 ),           dvec3( 15, 5, 5 ),           1 },
		{ "diagonally through the second",      dvec3( 30, -30, 15 ),        dvec3( 70, 30, 15 ),         1 },
		{ "through both boxes",                 dvec3( -5, 5, 5 ),           dvec3( 70, 5, 5 ),           1 },
		{ "from inside a box to outside",       dvec3( 5, 5, 5 ),            dvec3( 5, 5, 20 ),           1 },
		{ "along the top, 1/256u above",        dvec3( -5, 5, 10 + lift ),   dvec3( 15, 5, 10 + lift ),   0 },
		{ "along a side, 1/256u off",           dvec3( -5, 10 + lift, 5 ),   dvec3( 15, 10 + lift, 5 ),   0 },
		{ "along an edge, 1/256u off both",     dvec3( -5, 10 + lift, 10 + lift ), dvec3( 15, 10 + lift, 10 + lift ), 0 },
		{ "ending 1/256u short of a face",      dvec3( -5, 5, 5 ),           dvec3( -lift, 5, 5 ),        0 },
		{ "along the top, 1/256u inside it",    dvec3( -5, 5, 10 - lift ),   dvec3( 15, 5, 10 - lift ),   1 },
		{ "ending 1/256u inside a face",        dvec3( -5, 5, 5 ),           dvec3( lift, 5, 5 ),         1 },
	};
	// exact touches: along a face, along the top, along an edge, ending on a face; clear on the CPU
	const Case touches[] = {
		{ "along a side (y = 10)",              dvec3( -5, 10, 5 ),          dvec3( 15, 10, 5 ),          0 },
		{ "along the top (z = 10)",             dvec3( -5, 5, 10 ),          dvec3( 15, 5, 10 ),          0 },
		{ "along an edge (y = 10, z = 10)",     dvec3( -5, 10, 10 ),         dvec3( 15, 10, 10 ),         0 },
		{ "ending on a face (x = 0)",           dvec3( -5, 5, 5 ),           dvec3( 0, 5, 5 ),            0 },
	};
	std::vector<VisSegment> segs;
	for ( size_t i = 0; i < ARRAYSIZE( cases ); i++ )
	{
		VisSegment s = { cases[i].a, cases[i].b };
		segs.push_back( s );
	}
	for ( size_t i = 0; i < ARRAYSIZE( touches ); i++ )
	{
		VisSegment s = { touches[i].a, touches[i].b };
		segs.push_back( s );
	}
	std::vector<unsigned char> gpu;
	bool launched = BwVis_TraceSegmentsGpu( segs, g, gpu ) && gpu.size() == segs.size();
	Check( launched, "the known-answer segments launch on the GPU" );
	for ( size_t i = 0; i < ARRAYSIZE( cases ) && launched; i++ )
	{
		bool cpu = CpuSegmentBlocked( occ, cases[i].a, cases[i].b );
		Check( cpu == ( cases[i].want != 0 ) && ( gpu[i] != 0 ) == cpu, "segment %s: %s on the CPU, %s on the GPU (want %s)",
			   cases[i].what, cpu ? "blocked" : "clear", gpu[i] ? "blocked" : "clear", cases[i].want ? "blocked" : "clear" );
	}
	for ( size_t i = 0; i < ARRAYSIZE( touches ) && launched; i++ )
	{
		bool cpu = CpuSegmentBlocked( occ, touches[i].a, touches[i].b );
		bool gpuBlocked = gpu[ARRAYSIZE( cases ) + i] != 0;
		Check( !cpu, "segment %s touches a box: free on the CPU", touches[i].what );
		if ( gpuBlocked != cpu )
			Msg( "  (note: segment %s: %s on the GPU; an exact touch is decided in float, within the %g float bound)\n",
				 touches[i].what, gpuBlocked ? "blocked" : "clear", VIS_FLOAT_BOUND );
	}

	// i2: 4096 segments between random points of a box around both occluders (a fixed sequence;
	// every point outside both: the passes' samples and eyes are in free space, and a ray that never
	// meets a surface, both ends inside one box, is clear on the GPU but crosses the box on the CPU).
	// None grazes an occluder closely enough for float rounding to matter, so the two paths must
	// agree on every one.
	if ( launched )
	{
		unsigned int seed = 12345u;
		auto frand = [&]() {
			seed = seed * 1664525u + 1013904223u;
			return ( seed >> 8 ) * ( 1.0 / 16777216.0 );
		};
		auto point = [&]() {
			for ( ;; )
			{
				dvec3 p( -30 + 120 * frand(), -40 + 80 * frand(), -10 + 60 * frand() );
				bool inside = false;
				for ( size_t o = 0; o < occ.size() && !inside; o++ )
				{
					inside = true;
					for ( size_t k = 0; k < occ[o].size() && inside; k++ )
						inside = DDot( occ[o][k].n, p ) <= occ[o][k].d;
				}
				if ( !inside )
					return p;
			}
		};
		std::vector<VisSegment> rnd;
		for ( int i = 0; i < 4096; i++ )
		{
			VisSegment s;
			s.a = point();
			s.b = point();
			rnd.push_back( s );
		}
		std::vector<unsigned char> rg;
		bool ok = BwVis_TraceSegmentsGpu( rnd, g, rg ) && rg.size() == rnd.size();
		int differ = 0, blockedCount = 0;
		for ( size_t i = 0; ok && i < rnd.size(); i++ )
		{
			bool cpu = CpuSegmentBlocked( occ, rnd[i].a, rnd[i].b );
			differ += ( rg[i] != 0 ) != cpu;
			blockedCount += cpu;
		}
		// some segments of each answer, or the comparison proves little
		Check( ok && differ == 0 && blockedCount > 200 && blockedCount < 3900,
			   "4096 random segments: the GPU and the CPU agree on all (%d differ; %d blocked)", differ, blockedCount );
	}

	// i3: the #35 sweep kernel against the CPU. 300 eyes (float, as eye samples are) and 37 face
	// samples with random normals (some eyes behind some samples); per eye, the 8 pairs ConfirmPairs
	// makes (BwVis_ConfirmSample) traced on the CPU, and the kernel's answer for that eye alone must
	// match: the same sample choice, the same back-face test, the same segments. Then all 300 in one
	// launch: clear exactly when some eye is.
	if ( launched )
	{
		unsigned int seed = 777u;
		auto frand = [&]() {
			seed = seed * 1664525u + 1013904223u;
			return ( seed >> 8 ) * ( 1.0 / 16777216.0 );
		};
		auto outside = [&]( const dvec3 &p ) {
			for ( size_t o = 0; o < occ.size(); o++ )
			{
				bool inside = true;
				for ( size_t k = 0; k < occ[o].size() && inside; k++ )
					inside = DDot( occ[o][k].n, p ) <= occ[o][k].d;
				if ( inside )
					return false;
			}
			return true;
		};
		std::vector<dvec3> eyes;
		while ( eyes.size() < 300 )
		{
			dvec3 p( (float)( -30 + 120 * frand() ), (float)( -40 + 80 * frand() ), (float)( -10 + 60 * frand() ) );
			if ( outside( p ) )
				eyes.push_back( p );
		}
		std::vector<double> samp;
		std::vector<dvec3> sp, sn;
		while ( sp.size() < 37 )
		{
			dvec3 p( 20 + 15 * frand(), -15 + 30 * frand(), 5 * frand() );
			if ( !outside( p ) )
				continue;
			dvec3 n( frand() - 0.5, frand() - 0.5, frand() - 0.5 );
			DNormalize( n );
			sp.push_back( p );
			sn.push_back( n );
			double v[6] = { p.x, p.y, p.z, n.x, n.y, n.z };
			samp.insert( samp.end(), v, v + 6 );
		}
		struct Fill
		{
			static void Eyes( void *ctx, size_t first, size_t count, float *out )
			{
				const std::vector<dvec3> &e = *(const std::vector<dvec3> *)ctx;
				for ( size_t k = 0; k < count; k++ )
				{
					out[3 * k] = (float)e[first + k].x;
					out[3 * k + 1] = (float)e[first + k].y;
					out[3 * k + 2] = (float)e[first + k].z;
				}
			}
		};
		const unsigned long long key = 41ULL << 32;
		const int rays = 8;
		bool ok = BwOptix_SweepEyes( eyes.size(), Fill::Eyes, (void *)&eyes ) &&
				  BwOptix_SweepSamples( 0, samp.data(), (unsigned int)sp.size() );
		int differ = 0, clearEyes = 0, backPairs = 0;
		std::vector<unsigned int> all;
		std::vector<unsigned long long> cpuPairs;	// i * rays + r of every clear pair
		for ( unsigned int i = 0; ok && i < eyes.size(); i++ )
		{
			bool cpuClear = false;
			for ( int r = 0; r < rays; r++ )
			{
				size_t k = BwVis_ConfirmSample( key, i, r, rays, sp.size() );
				if ( DDot( eyes[i] - sp[k], sn[k] ) <= 0 )
				{
					backPairs++;
					continue;
				}
				bool pairClear = !CpuSegmentBlocked( occ, sp[k], eyes[i] );
				cpuClear = cpuClear || pairClear;
				if ( pairClear )
					cpuPairs.push_back( (unsigned long long)i * rays + r );
			}
			bool gpuClear = false;
			ok = BwOptix_SweepTrace( 0, &i, 1, rays, false, key, i, gpuClear );
			differ += ok && gpuClear != cpuClear;
			clearEyes += cpuClear;
			all.push_back( i );
		}
		bool allClear = false;
		ok = ok && BwOptix_SweepTrace( 0, all.data(), (unsigned int)all.size(), rays, false, key, 0, allClear );
		// The agreement is the check; the scene only has to be non-degenerate (some eye clear, some blocked, some
		// pair back-facing) so that agreeing means something. Its exact counts are not pinned.
		Check( ok && differ == 0 && clearEyes >= 1 && clearEyes < (int)eyes.size() && backPairs >= 1 &&
				   allClear == ( clearEyes > 0 ),
			   "sweep kernel: %d eyes x %d confirmation pairs agree with the CPU eye by eye (%d differ; %d eyes clear, "
			   "%d pairs back-facing) and in one launch (%s)", (int)eyes.size(), rays, differ, clearEyes, backPairs,
			   allClear ? "clear" : "blocked" );
		// #114: the same launch listing its clear pairs (the one-way occluder path) lists exactly the
		// pairs clear on the CPU
		std::vector<unsigned long long> listed;
		bool listClear = false, overflow = true;
		bool okList = ok && BwOptix_SweepTrace( 0, all.data(), (unsigned int)all.size(), rays, false, key, 0, listClear,
												&listed, &overflow );
		std::sort( listed.begin(), listed.end() );
		Check( okList && !overflow && listClear == allClear && listed == cpuPairs,
			   "sweep kernel: the listed clear pairs are exactly the CPU's (%d listed, %d on the CPU)", (int)listed.size(),
			   (int)cpuPairs.size() );
		BwOptix_SweepFree();
	}

	// i4: the seam fillers on the sight mesh: the face pass's GPU mesh holds the shrunk occluders
	// and the fillers as they are (OccluderMesh( 0 )), so the seam scene's known answers hold there too
	{
		SeamScene s;
		BuildSeamScene( s, true );
		std::vector<float> sv;
		std::vector<unsigned int> st;
		for ( size_t o = 0; o < s.occ.size(); o++ )
		{
			ConvexShape cs;
			if ( !ConvexFromPlanes( s.occ[o].data(), (int)s.occ[o].size(), cs ) )
				continue;
			unsigned int base = (unsigned int)( sv.size() / 3 );
			for ( size_t k = 0; k < cs.verts.size(); k++ )
			{
				sv.push_back( (float)cs.verts[k].x );
				sv.push_back( (float)cs.verts[k].y );
				sv.push_back( (float)cs.verts[k].z );
			}
			for ( size_t f = 0; f < cs.faces.size(); f++ )
			{
				for ( size_t k = 1; k + 1 < cs.faces[f].size(); k++ )
				{
					st.push_back( base + cs.faces[f][0] );
					st.push_back( base + cs.faces[f][k] );
					st.push_back( base + cs.faces[f][k + 1] );
				}
			}
		}
		std::vector<VisSegment> segs;
		for ( size_t i = 0; i < ARRAYSIZE( s_SeamCases ); i++ )
		{
			VisSegment sg = { s_SeamCases[i].a, s_SeamCases[i].b };
			segs.push_back( sg );
		}
		std::vector<unsigned char> gpu;
		bool ok = BwOptix_BuildMesh( BW_MESH_VIS, sv, st ) && BwVis_TraceSegmentsGpu( segs, g, gpu ) && gpu.size() == segs.size();
		Check( ok, "the seam scene's sight mesh builds and its segments launch on the GPU" );
		for ( size_t i = 0; i < ARRAYSIZE( s_SeamCases ) && ok; i++ )
		{
			bool cpu = SeamBlocked( s.occ, s_SeamCases[i].a, s_SeamCases[i].b );
			Check( ( gpu[i] != 0 ) == cpu && cpu == ( s_SeamCases[i].want != 0 ), "seam segment %s: %s on the GPU, %s on the CPU (want %s)",
				   s_SeamCases[i].what, gpu[i] ? "blocked" : "clear", cpu ? "blocked" : "clear", s_SeamCases[i].want ? "blocked" : "clear" );
		}
	}
	BwOptix_Shutdown();
}

//-----------------------------------------------------------------------------
// The eye region bound (EyeBound, bw_vis.h): the k-DOP of the exact piece vertices,
// which the face pass asks how far it reaches in front of a plane.
//-----------------------------------------------------------------------------
static double RegionRand( unsigned &s )
{
	s = s * 1664525u + 1013904223u;
	return ( ( s >> 8 ) & 0xFFFFFF ) / 16777216.0;
}

// The least sum(w_i c_i) over n = sum(w_i d_i), w >= 0, by trying every basis of three
// directions: the linear program EyeBound::Support solves with a simplex, done the slow way.
static double RegionBruteSupport( const EyeBound &b, const dvec3 &n )
{
	double c[REGION_DIRS] = { b.hi.x, -b.lo.x, b.hi.y, -b.lo.y, b.hi.z, -b.lo.z };
	for ( int k = 0; k < REGION_EXTRA; k++ )
		c[6 + k] = b.h[k];
	double best = 1e300;
	for ( int i = 0; i < REGION_DIRS; i++ )
	{
		for ( int j = i + 1; j < REGION_DIRS; j++ )
		{
			for ( int k = j + 1; k < REGION_DIRS; k++ )
			{
				const dvec3 &a = RegionDir( i ), &bb = RegionDir( j ), &e = RegionDir( k );
				double det = DDot( a, DCross( bb, e ) );
				if ( fabs( det ) < 1e-6 )
					continue;
				double w0 = DDot( n, DCross( bb, e ) ) / det, w1 = DDot( n, DCross( e, a ) ) / det,
					   w2 = DDot( n, DCross( a, bb ) ) / det;
				if ( w0 < -1e-12 || w1 < -1e-12 || w2 < -1e-12 )
					continue;
				best = MIN( best, w0 * c[i] + w1 * c[j] + w2 * c[k] );
			}
		}
	}
	return best;
}

static void TestRegionBound()
{
	Msg( "eye region bound\n" );
	unsigned seed = 12345;
	int below = 0, aboveBox = 0, offLp = 0, tested = 0;
	for ( int shape = 0; shape < 6; shape++ )
	{
		for ( int rep = 0; rep < 20; rep++ )
		{
			// vertices of a region of this shape somewhere in +-4000
			dvec3 org( ( RegionRand( seed ) - 0.5 ) * 8000, ( RegionRand( seed ) - 0.5 ) * 8000, ( RegionRand( seed ) - 0.5 ) * 8000 );
			dvec3 ext( 8 + RegionRand( seed ) * 400, 8 + RegionRand( seed ) * 400, 8 + RegionRand( seed ) * 200 );
			double yaw = RegionRand( seed ) * 6.283185307179586;
			dvec3 ax( cos( yaw ), sin( yaw ), 0 ), ay( -sin( yaw ), cos( yaw ), 0 );
			int nv = shape == 4 ? 2 : shape == 5 ? 1 : 1 + (int)( RegionRand( seed ) * 12 );
			EyeBound b;
			std::vector<dvec3> pts;
			for ( int i = 0; i < nv; i++ )
			{
				double u = RegionRand( seed ), v = RegionRand( seed ), w = RegionRand( seed );
				dvec3 p = org;
				if ( shape == 0 )
					p += dvec3( u * ext.x, v * ext.y, w * ext.z );
				else if ( shape == 1 )
					p += dvec3( u * ext.x, v * ext.y, 0 );
				else if ( shape == 2 )		// a sliver along a yawed diagonal
					p += ax * ( u * ext.x ) + ay * ( v * 6 ) + dvec3( 0, 0, w * 8 );
				else if ( shape == 3 )
					p += ax * ( u * ext.x );
				else if ( shape == 4 )
					p += ax * ( i * ext.x );
				pts.push_back( p );
				b.Add( p );
			}
			for ( int t = 0; t < 40; t++ )
			{
				dvec3 n( RegionRand( seed ) * 2 - 1, RegionRand( seed ) * 2 - 1, RegionRand( seed ) * 2 - 1 );
				if ( t % 4 == 1 )
					n.z = 0;						// a vertical wall
				if ( t % 8 == 2 )
					n = dvec3( n.x > 0 ? 1 : -1, 0, 0 );		// on an axis
				if ( DNormalize( n ) < 1e-6 )
					continue;
				double trueMax = -1e300;
				for ( size_t i = 0; i < pts.size(); i++ )
					trueMax = MAX( trueMax, DDot( n, pts[i] ) );
				double box = b.BoxSupport( n ), kdop = b.Support( n ), brute = RegionBruteSupport( b, n );
				tested++;
				below += kdop < trueMax;
				aboveBox += kdop > box + 1e-9;
				offLp += fabs( kdop - brute ) > 1e-5;
			}
		}
	}
	Check( below == 0, "region bound: the bound is never below the true max of n . v over the vertices (%d random region/normal pairs: %d below)", tested, below );
	Check( aboveBox == 0, "region bound: never above the box's bound (%d above)", aboveBox );
	Check( offLp == 0, "region bound: the simplex equals the least over every basis of three directions (%d of %d differ by more than 1e-5)", offLp, tested );

	// A known answer: the vertices (0, 128, 0) and (400, 0, 0), against a wall facing 19 degrees off the
	// x axis. The hull reaches 0.9454 * 400 = 378.2 along it; the box adds the corner's y share, 41.7
	// more. The 22.5 degree direction brackets the wall normal with +x and gets within 0.5 of the truth.
	{
		EyeBound b;
		b.Add( dvec3( 0, 128, 0 ) );
		b.Add( dvec3( 400, 0, 0 ) );
		dvec3 n( 0.946, 0.326, 0 );
		DNormalize( n );
		double trueMax = MAX( DDot( n, dvec3( 0, 128, 0 ) ), DDot( n, dvec3( 400, 0, 0 ) ) );
		double box = b.BoxSupport( n ), kdop = b.Support( n );
		Check( box - trueMax > 40 && kdop >= trueMax && kdop - trueMax < 1.0,
			   "region bound: slanted wall over a diagonal region: true reach %.2f, box %.2f, k-DOP %.2f", trueMax, box, kdop );
		// stopping early still gives a bound that holds
		double early = b.Support( n, box - 1 );
		Check( early >= trueMax && early <= box && early <= box - 1, "region bound: stopAt returns a valid bound (%.2f)", early );
	}

	// Merging two bounds is the bound of the union, and the unbounded and origin regions answer as they should.
	{
		EyeBound a, b, whole;
		for ( int i = 0; i < 20; i++ )
		{
			dvec3 p( RegionRand( seed ) * 500 - 250, RegionRand( seed ) * 500 - 250, RegionRand( seed ) * 100 );
			( i < 9 ? a : b ).Add( p );
			whole.Add( p );
		}
		EyeBound m;
		m.Merge( a );
		m.Merge( EyeBound() );		// an empty one adds nothing
		m.Merge( b );
		bool same = m.any && m.lo.x == whole.lo.x && m.lo.y == whole.lo.y && m.lo.z == whole.lo.z && m.hi.x == whole.hi.x &&
					m.hi.y == whole.hi.y && m.hi.z == whole.hi.z;
		for ( int k = 0; k < REGION_EXTRA; k++ )
			same = same && m.h[k] == whole.h[k];
		Check( same, "region bound: merging two halves gives the bound of the whole" );
		EyeBound u, o;
		u.Unbounded();
		o.AtOrigin();
		dvec3 n( 0.6, -0.8, 0 );
		Check( u.Support( n ) > 1e29 && fabs( o.Support( n ) ) < 1e-6, "region bound: unbounded %g, origin %g", u.Support( n ), o.Support( n ) );
	}
}

// (e2) pixel walks (#87): MinWidthXY is a polygon's width, its least extent over its edge
// normals, not its box: a 1/32u strip at 45 degrees (a d1_trainstation_02 wall-top sliver) is
// 1/32 wide though its box is 16u; a 3-4-5 triangle's width is its altitude to the hypotenuse.
static void TestLedgeWidth()
{
	Msg( "(e2) walk node width\n" );
	const double w = 1.0 / 32.0, s = w / sqrt( 2.0 );
	double dx[4] = { 0.0, 16.0, 16.0 - s, -s }, dy[4] = { 0.0, 16.0, 16.0 + s, s };
	double diag = MinWidthXY( dx, dy, 4 );
	Check( Near( diag, w, 1e-12 ), "45-degree strip 1/32u across, box 16u: width %.9f (want %.9f)", diag, w );
	double tx[3] = { 0.0, 4.0, 0.0 }, ty[3] = { 0.0, 0.0, 3.0 };
	double tri = MinWidthXY( tx, ty, 3 );
	Check( Near( tri, 2.4, 1e-12 ), "3-4-5 triangle: width %.9f (want 2.4, the altitude to the hypotenuse)", tri );
}

// (h10) the smaller tier B eye set (#39): path chords clipped to the part outside
// the poly pieces, the lattice kept only on the shell of the volume, and tiles
// that give the same samples however the area is cut. Answers by hand.
static bool EyeBox( const dvec3 &mins, const dvec3 &maxs, EyePiece &out )
{
	std::vector<DPlane> planes;
	VisBoxPlanes( mins, maxs, planes );
	out.source = EYESRC_NODE;
	return EyePolyFromPlanes( planes, out );
}

static bool SameEnds( const std::vector<dvec3> &parts, const double *want, int nParts, double tol )
{
	if ( (int)parts.size() != 2 * nParts )
		return false;
	for ( int i = 0; i < 2 * nParts; i++ )
	{
		if ( !Near( parts[i].x, want[3 * i], tol ) || !Near( parts[i].y, want[3 * i + 1], tol ) ||
			 !Near( parts[i].z, want[3 * i + 2], tol ) )
			return false;
	}
	return true;
}

static int CountAt( const std::vector<EyeSample> &s, double x, double y, double z )
{
	int n = 0;
	for ( size_t i = 0; i < s.size(); i++ )
		n += s[i].x == (float)x && s[i].y == (float)y && s[i].z == (float)z;
	return n;
}

static bool SampleLess( const EyeSample &a, const EyeSample &b )
{
	if ( a.x != b.x )
		return a.x < b.x;
	if ( a.y != b.y )
		return a.y < b.y;
	return a.z < b.z;
}

// The region generated tile by tile (side 'side') against the same region as one
// rectangle: equal sets, and no point twice in the tile-by-tile list.
static bool TilesMatch( const std::vector<EyePiece> &pieces, double sp, double realSp, int sky, double side, int tx0,
						int ty0, int tx1, int ty1, int &nTiles, int &nWhole, int &nDup )
{
	std::vector<EyeSample> tiles, one, whole;
	for ( int tx = tx0; tx <= tx1; tx++ )
	{
		for ( int ty = ty0; ty <= ty1; ty++ )
		{
			one.clear();
			SampleEyeTile( pieces, sp, realSp, sky, tx * side, ty * side, ( tx + 1 ) * side, ( ty + 1 ) * side, one );
			tiles.insert( tiles.end(), one.begin(), one.end() );
		}
	}
	SampleEyeTile( pieces, sp, realSp, sky, tx0 * side, ty0 * side, ( tx1 + 1 ) * side, ( ty1 + 1 ) * side, whole );
	std::sort( tiles.begin(), tiles.end(), SampleLess );
	std::sort( whole.begin(), whole.end(), SampleLess );
	nDup = 0;
	for ( size_t i = 1; i < tiles.size(); i++ )
		nDup += !SampleLess( tiles[i - 1], tiles[i] );
	nTiles = (int)tiles.size();
	nWhole = (int)whole.size();
	bool same = tiles.size() == whole.size();
	for ( size_t i = 0; i < tiles.size() && same; i++ )
		same = !SampleLess( tiles[i], whole[i] ) && !SampleLess( whole[i], tiles[i] ) && tiles[i].source == whole[i].source &&
			   tiles[i].sky == whole[i].sky;
	return same && nDup == 0 && nTiles > 0;
}

static void TestEyeSet()
{
	Msg( "(h10) tier B eye set: chord clipping, shell, tiles (#39)\n" );
	const double tol = 1e-5;
	EyePiece A, B, C;
	std::vector<EyePiece> polys;
	std::vector<dvec3> parts;

	// c1: through a box [0,10]^3 along x: the parts before and after it
	EyeBox( dvec3( 0, 0, 0 ), dvec3( 10, 10, 10 ), A );
	polys.assign( 1, A );
	ClipEyeChord( dvec3( -5, 5, 5 ), dvec3( 15, 5, 5 ), polys, parts );
	const double c1[] = { -5, 5, 5, 0, 5, 5, 10, 5, 5, 15, 5, 5 };
	Check( SameEnds( parts, c1, 2, tol ), "chord through a box: %d parts (want -5..0 and 10..15)", (int)parts.size() / 2 );

	// c2: through two boxes that abut at x 10: nothing kept between them
	EyeBox( dvec3( 10, 0, 0 ), dvec3( 20, 10, 10 ), B );
	polys.push_back( B );
	ClipEyeChord( dvec3( -5, 5, 5 ), dvec3( 25, 5, 5 ), polys, parts );
	const double c2[] = { -5, 5, 5, 0, 5, 5, 20, 5, 5, 25, 5, 5 };
	Check( SameEnds( parts, c2, 2, tol ), "chord through two abutting boxes: %d parts (want -5..0 and 20..25)",
		   (int)parts.size() / 2 );

	// c3: wholly inside, c4: wholly outside (kept exactly)
	ClipEyeChord( dvec3( 2, 3, 4 ), dvec3( 18, 7, 6 ), polys, parts );
	Check( parts.empty(), "chord inside two boxes: %d parts (want 0)", (int)parts.size() / 2 );
	ClipEyeChord( dvec3( -5, 20, 5 ), dvec3( 15, 21, 5 ), polys, parts );
	const double c4[] = { -5, 20, 5, 15, 21, 5 };
	Check( SameEnds( parts, c4, 1, 0.0 ), "chord clear of the boxes: kept whole (%d parts)", (int)parts.size() / 2 );

	// c5: lying on the face y 10 is inside the closed piece; 1e-5 off it (past
	// EYE_TOUCH) is outside and kept whole
	polys.assign( 1, A );
	ClipEyeChord( dvec3( -5, 10, 5 ), dvec3( 15, 10, 5 ), polys, parts );
	const double c5[] = { -5, 10, 5, 0, 10, 5, 10, 10, 5, 15, 10, 5 };
	Check( SameEnds( parts, c5, 2, tol ), "chord along a face: %d parts (want -5..0 and 10..15)", (int)parts.size() / 2 );
	ClipEyeChord( dvec3( -5, 10 + 1e-5, 5 ), dvec3( 15, 10 + 1e-5, 5 ), polys, parts );
	Check( parts.size() == 2, "chord 1e-5 off a face: kept whole (%d parts)", (int)parts.size() / 2 );

	// c6: a gap of 5e-5 between two boxes (under EYE_SPLIT_EPS): that part is dropped
	EyeBox( dvec3( 10.00005, 0, 0 ), dvec3( 20, 10, 10 ), C );
	polys.push_back( C );
	ClipEyeChord( dvec3( -5, 5, 5 ), dvec3( 25, 5, 5 ), polys, parts );
	Check( SameEnds( parts, c2, 2, tol ), "chord across a 5e-5 gap between boxes: %d parts (want 2, the gap dropped)",
		   (int)parts.size() / 2 );

	// s1: a box [0,64]^3 at 8u: 81 lattice lines, both ends of each (162); the
	// heights 8..56 only on the 32 rim lines (224); 386 in all. The full lattice
	// (above EYE_SHELL_MAX_SPACING) would be 81 x 9 = 729.
	std::vector<EyeSample> s;
	EyeBox( dvec3( 0, 0, 0 ), dvec3( 64, 64, 64 ), A );
	polys.assign( 1, A );
	SampleEyeTile( polys, 8.0, 8.0, -1, -1024, -1024, 1024, 1024, s );
	int inner = CountAt( s, 32, 32, 32 ), rim = CountAt( s, 0, 32, 32 );
	Check( s.size() == 386 && !inner && rim == 1, "shell of a 64u box at 8u: %d samples (want 386), (32 32 32) %d (want 0), "
		   "(0 32 32) %d (want 1)", (int)s.size(), inner, rim );

	// s2: an L of two overlapping boxes, [0,64]x[0,32] and [0,32]x[0,64], 64 high:
	// 65 lines (130 ends); 33 lines have all 8 neighbours in the L, the other 32
	// keep their 7 heights (224): 354. The inner corner's line (32 32) keeps them.
	EyeBox( dvec3( 0, 0, 0 ), dvec3( 64, 32, 64 ), A );
	EyeBox( dvec3( 0, 0, 0 ), dvec3( 32, 64, 64 ), B );
	polys.assign( 1, A );
	polys.push_back( B );
	s.clear();
	SampleEyeTile( polys, 8.0, 8.0, -1, -1024, -1024, 1024, 1024, s );
	int corner = 0, deep = 0;
	for ( int k = 1; k <= 7; k++ )
	{
		corner += CountAt( s, 32, 32, 8.0 * k );
		deep += CountAt( s, 16, 16, 8.0 * k );
	}
	Check( s.size() == 354 && corner == 7 && deep == 0, "shell of an L at 8u: %d samples (want 354), inner corner line %d "
		   "heights (want 7), (16 16) %d (want 0)", (int)s.size(), corner, deep );

	// s3: two boxes stacked on a shared face z 32: one interval per line, so no
	// sample at z 32 off the rim, and the same 386 as one box
	EyeBox( dvec3( 0, 0, 0 ), dvec3( 64, 64, 32 ), A );
	EyeBox( dvec3( 0, 0, 32 ), dvec3( 64, 64, 64 ), B );
	polys.assign( 1, A );
	polys.push_back( B );
	s.clear();
	SampleEyeTile( polys, 8.0, 8.0, -1, -1024, -1024, 1024, 1024, s );
	int face = 0;
	for ( size_t i = 0; i < s.size(); i++ )
		face += s[i].z == 32.0f && s[i].x > 0.0f && s[i].x < 64.0f && s[i].y > 0.0f && s[i].y < 64.0f;
	Check( s.size() == 386 && face == 0, "two stacked boxes at 8u: %d samples (want 386), %d on the shared face off the rim "
		   "(want 0)", (int)s.size(), face );

	// s4: above EYE_SHELL_MAX_SPACING nothing is dropped: a 128u box at 32u is the
	// full 5 x 5 x 5 lattice
	EyeBox( dvec3( 0, 0, 0 ), dvec3( 128, 128, 128 ), A );
	polys.assign( 1, A );
	s.clear();
	SampleEyeTile( polys, 32.0, 32.0, -1, -1024, -1024, 1024, 1024, s );
	Check( s.size() == 125, "128u box at 32u (shell off): %d samples (want 125)", (int)s.size() );

	// t1: a region over 5 x 3 tiles (two boxes, a sloped piece, a segment) gives the
	// same samples tile by tile as in one piece, real and mapped into a skybox
	std::vector<EyePiece> region;
	EyeBox( dvec3( -300, -100, 0 ), dvec3( 520, 300, 40 ), A );
	region.push_back( A );
	EyeBox( dvec3( 250, 250, 30 ), dvec3( 262.5, 263, 90 ), B );
	region.push_back( B );
	{
		std::vector<DPlane> planes;
		VisBoxPlanes( dvec3( -100, -60, 50 ), dvec3( 400, 200, 200 ), planes );
		planes[4].n = dvec3( -0.3, 0.2, 1 );
		double len = DNormalize( planes[4].n );
		planes[4].d = 120.0 / len;
		if ( EyePolyFromPlanes( planes, C ) )
		{
			C.source = EYESRC_RIDE;
			region.push_back( C );
		}
	}
	EyePiece seg;
	seg.kind = EYE_SEGMENT;
	seg.source = EYESRC_ARC;
	seg.verts.push_back( dvec3( -290, -90, 250 ) );
	seg.verts.push_back( dvec3( 510, 290, 270 ) );
	seg.mins = dvec3( -290, -90, 250 );
	seg.maxs = dvec3( 510, 290, 270 );
	region.push_back( seg );
	int nT, nW, nD;
	bool ok = TilesMatch( region, 8.0, 8.0, -1, EYE_TILE, -2, -1, 2, 1, nT, nW, nD );
	Check( ok, "real layer, 15 tiles: %d samples tile by tile, %d as one, %d repeated", nT, nW, nD );
	std::vector<EyePiece> mapped( region.size() );
	const dvec3 origin( 1000.25, -2000, 300 );
	for ( size_t i = 0; i < region.size(); i++ )
		SkyMapPiece( region[i], origin, 16.0, mapped[i] );
	ok = TilesMatch( mapped, 0.5, 8.0, 0, EYE_TILE / 16.0, 61, -126, 64, -124, nT, nW, nD );
	Check( ok, "sky layer (scale 16), 12 tiles: %d samples tile by tile, %d as one, %d repeated", nT, nW, nD );
}

// The eye store's test generator: tile t of a grid of EYE_TILE squares (tx0..,
// ty0.., 'cols' tiles a row) sampled from 'pieces' at 8u. A sample's cluster is
// 0 west of x 100 and 1 east of it, -1 above z 200; the tile is packed as the
// real generator packs one (groups ascending, -1 first, each in the tile's order).
struct StoreTestGen
{
	const std::vector<EyePiece> *pieces;
	int tx0, ty0, cols;
	std::atomic<int> gens[64];
	int slowTile;		// the generator waits 100 ms in this tile (-1: none)
};

static void StoreTestTile( void *ctx, int tile, EyeTileData &out )
{
	StoreTestGen &g = *(StoreTestGen *)ctx;
	g.gens[tile].fetch_add( 1 );
	if ( tile == g.slowTile )
		std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
	int tx = g.tx0 + tile % g.cols, ty = g.ty0 + tile / g.cols;
	std::vector<EyeSample> s;
	SampleEyeTile( *g.pieces, 8.0, 8.0, -1, tx * EYE_TILE, ty * EYE_TILE, ( tx + 1 ) * EYE_TILE, ( ty + 1 ) * EYE_TILE, s );
	std::vector< std::pair<int, int> > key( s.size() );
	for ( size_t i = 0; i < s.size(); i++ )
		key[i] = std::make_pair( s[i].z > 200.0f ? -1 : s[i].x < 100.0f ? 0 : 1, (int)i );
	std::sort( key.begin(), key.end() );
	out.made = (long long)s.size();
	for ( size_t i = 0; i < key.size(); i++ )
	{
		const EyeSample &e = s[key[i].second];
		if ( out.groupCl.empty() || out.groupCl.back() != key[i].first )
		{
			out.groupCl.push_back( key[i].first );
			out.groupOfs.push_back( (int)i );
		}
		out.xyz.push_back( e.x );
		out.xyz.push_back( e.y );
		out.xyz.push_back( e.z );
		out.source.push_back( (unsigned char)e.source );
	}
	out.groupOfs.push_back( (int)s.size() );
}

static EyeStore *MakeTestStore( StoreTestGen &g, int numTiles, double budgetBytes )
{
	EyeStoreSetup setup;
	setup.numTiles = numTiles;
	setup.numClusters = 2;
	setup.clTileOfs.push_back( 0 );
	for ( int c = 0; c < 2; c++ )
	{
		for ( int t = 0; t < numTiles; t++ )
			setup.clTiles.push_back( t );
		setup.clTileOfs.push_back( (int)setup.clTiles.size() );
	}
	setup.pieceClusters.assign( 2, 1 );
	setup.partBound.assign( 2, EyeBound() );
	setup.budgetBytes = budgetBytes;
	setup.gen = StoreTestTile;
	setup.ctx = &g;
	return new EyeStore( setup );
}

static bool SameTileData( const EyeTileData &a, const EyeTileData &b )
{
	return a.xyz == b.xyz && a.source == b.source && a.groupCl == b.groupCl && a.groupOfs == b.groupOfs;
}

// (h11) The eye store (#39): what a face pass reads must not depend on the cache.
static void TestEyeStore()
{
	Msg( "(h11) tier B eye store: cache size, regeneration, pins, one generation per tile (#39)\n" );
	std::vector<EyePiece> region;
	EyePiece A, B;
	EyeBox( dvec3( -300, -100, 0 ), dvec3( 520, 300, 40 ), A );
	region.push_back( A );
	EyeBox( dvec3( 250, 250, 30 ), dvec3( 262.5, 263, 90 ), B );
	region.push_back( B );
	EyePiece seg;
	seg.kind = EYE_SEGMENT;
	seg.source = EYESRC_ARC;
	seg.verts.push_back( dvec3( -290, -90, 250 ) );
	seg.verts.push_back( dvec3( 510, 290, 270 ) );
	seg.mins = dvec3( -290, -90, 250 );
	seg.maxs = dvec3( 510, 290, 270 );
	region.push_back( seg );
	const int numTiles = 15;
	StoreTestGen gBig, gSmall;
	StoreTestGen *gens[2] = { &gBig, &gSmall };
	for ( int k = 0; k < 2; k++ )
	{
		gens[k]->pieces = &region;
		gens[k]->tx0 = -2;
		gens[k]->ty0 = -1;
		gens[k]->cols = 5;
		gens[k]->slowTile = -1;
		for ( int t = 0; t < 64; t++ )
			gens[k]->gens[t].store( 0 );
	}

	// st1: a cache under one tile's size gives every cluster the same facts and every tile
	// the same samples as a cache that holds them all
	std::unique_ptr<EyeStore> big( MakeTestStore( gBig, numTiles, 1e12 ) ), small( MakeTestStore( gSmall, numTiles, 1.0 ) );
	big->ComputeAll();
	small->ComputeAll();
	bool same = true;
	for ( int c = 0; c < 2 && same; c++ )
	{
		const EyeClusterFacts &a = big->Facts( c ), &b = small->Facts( c );
		same = a.count == b.count && !memcmp( a.box, b.box, sizeof( a.box ) ) && a.runs.size() == b.runs.size() && a.count > 0;
		for ( size_t r = 0; r < a.runs.size() && same; r++ )
			same = a.runs[r].tile == b.runs[r].tile && a.runs[r].start == b.runs[r].start && a.runs[r].count == b.runs[r].count;
	}
	for ( int t = 0; t < numTiles && same; t++ )
	{
		EyePinSet pa, pb;
		same = SameTileData( pa.Get( *big, t ), pb.Get( *small, t ) );
	}
	EyeStoreStats sb = big->Stats(), ss = small->Stats();
	Check( same && sb.samples == ss.samples && sb.samples > 0 && sb.tilesGenerated == numTiles && ss.tilesGenerated == numTiles &&
			   sb.regenerations == 0 && ss.regenerations > 0,
		   "1-byte cache vs one holding every tile: same facts and tiles (%lld vs %lld samples; %lld vs %lld regenerations)",
		   sb.samples, ss.samples, sb.regenerations, ss.regenerations );

	// st2: a tile evicted and made again is bit-identical to its first generation
	EyeTileData first;
	{
		EyePinSet p;
		first = p.Get( *small, 3 );
	}
	int before = gSmall.gens[3].load();
	for ( int t = 0; t < numTiles; t++ )
	{
		small->Pin( t );
		small->Unpin( t );
	}
	bool remade;
	{
		EyePinSet p;
		const EyeTileData &again = p.Get( *small, 3 );
		remade = gSmall.gens[3].load() > before;
		same = SameTileData( first, again );
	}
	Check( remade && same && first.Count() > 0, "a regenerated tile is bit-identical (%d samples, made again: %s)", first.Count(),
		   remade ? "yes" : "no" );

	// st3: a pinned tile outlives an over-budget cache (same data, not made again)
	const EyeTileData &held = small->Pin( 5 );
	int heldGens = gSmall.gens[5].load();
	for ( int round = 0; round < 2; round++ )
	{
		for ( int t = 0; t < numTiles; t++ )
		{
			if ( t == 5 )
				continue;
			small->Pin( t );
			small->Unpin( t );
		}
	}
	bool kept = &small->Pinned( 5 ) == &held && gSmall.gens[5].load() == heldGens;
	small->Unpin( 5 );
	Check( kept, "a pinned tile survives an over-budget cache (generations of it %d -> %d)", heldGens, gSmall.gens[5].load() );

	// st4: two threads asking for an absent tile at once: one generation, both get it
	StoreTestGen gSlow;
	gSlow.pieces = &region;
	gSlow.tx0 = -2;
	gSlow.ty0 = -1;
	gSlow.cols = 5;
	gSlow.slowTile = 7;
	for ( int t = 0; t < 64; t++ )
		gSlow.gens[t].store( 0 );
	std::unique_ptr<EyeStore> slow( MakeTestStore( gSlow, numTiles, 1e12 ) );
	const EyeTileData *got[2] = { NULL, NULL };
	std::atomic<int> ready( 0 );
	auto ask = [&]( int k ) {
		ready.fetch_add( 1 );
		while ( ready.load() < 2 )
			;
		got[k] = &slow->Pin( 7 );
	};
	std::thread t0( ask, 0 ), t1( ask, 1 );
	t0.join();
	t1.join();
	Check( gSlow.gens[7].load() == 1 && got[0] == got[1] && got[0] && got[0]->Count() > 0,
		   "two threads asking for one tile: %d generation(s) (want 1), same data: %s", gSlow.gens[7].load(),
		   got[0] == got[1] ? "yes" : "no" );
	slow->Unpin( 7 );
	slow->Unpin( 7 );
}

int RunGrowSelfTest()
{
	s_nFailures = 0;
	s_nChecks = 0;
	s_nGpuChecks = 0;
	s_nGpuSkipped = 0;
	s_bVisGpuSkipped = false;
	Msg( "bspwalk -selftest: Minkowski grow, walkable polygon, edge, arc and visibility known answers\n" );
	TestCube();
	TestRamp();
	TestTriangle();
	TestSubtract();
	TestEdges();
	TestLedgeWidth();
	TestArcs();
	TestSpecial();
	TestVis();
	TestRegionBound();
	TestEyeSet();
	TestEyeStore();
	TestVisGpu();
	// The total depends on the machine: the GPU-only checks run only where a CUDA device and OptiX come up.
	if ( s_nGpuSkipped || s_bVisGpuSkipped )
		Msg( "selftest: %d checks (%d GPU-only; %d arc checks%s more run with a GPU), %d failed -> %s\n", s_nChecks,
			 s_nGpuChecks, s_nGpuSkipped, s_bVisGpuSkipped ? " and the GPU visibility checks" : "", s_nFailures,
			 s_nFailures ? "FAIL" : "PASS" );
	else
		Msg( "selftest: %d checks (%d GPU-only), %d failed -> %s\n", s_nChecks, s_nGpuChecks, s_nFailures,
			 s_nFailures ? "FAIL" : "PASS" );
	return s_nFailures;
}
