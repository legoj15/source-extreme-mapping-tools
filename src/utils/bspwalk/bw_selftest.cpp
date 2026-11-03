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

static int s_nFailures;
static int s_nChecks;

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

int RunGrowSelfTest()
{
	s_nFailures = 0;
	s_nChecks = 0;
	Msg( "bspwalk -selftest: Minkowski grow and walkable polygon known answers\n" );
	TestCube();
	TestRamp();
	TestTriangle();
	TestSubtract();
	TestEdges();
	Msg( "selftest: %d checks, %d failed -> %s\n", s_nChecks, s_nFailures, s_nFailures ? "FAIL" : "PASS" );
	return s_nFailures;
}
