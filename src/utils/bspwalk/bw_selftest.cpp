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

//-----------------------------------------------------------------------------
// (f) Stage 3 arcs: the solver against hand-derived parabolas, then gaps
// between two synthetic platforms (HL2 profile: vz 160, g 600, crouch lift
// 36, max air speed 640 x 1.15 = 736 u/s).
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
	int gpuDiff;	// -1: no GPU
};

static ArcCounts TraceGap( const std::vector< std::pair<dvec3, dvec3> > &boxes, const PlayerProfile &prof )
{
	CollisionWorld world;
	BoxWorld( boxes, prof, world );
	WalkNode A = TopNode( world, 0 ), B = TopNode( world, 1 );
	std::vector<ArcD> arcs;
	for ( int c = 0; c < ARC_COMBOS; c++ )
		PairArcs( A, B, prof, c, arcs );
	ArcCounts r = { (int)arcs.size(), 0, -1 };
	CArcTracer tracer;
	tracer.Init( world, prof, true );
	std::vector<unsigned char> cpu, gpu;
	tracer.TraceCpu( arcs, cpu );
	for ( size_t i = 0; i < cpu.size(); i++ )
		r.free += !cpu[i];
	if ( tracer.HasGpu() && tracer.TraceGpu( arcs, gpu ) )
	{
		r.gpuDiff = 0;
		for ( size_t i = 0; i < gpu.size(); i++ )
			r.gpuDiff += gpu[i] != cpu[i];
	}
	tracer.Shutdown();
	return r;
}

static void TestArcs()
{
	Msg( "(f) ballistic arcs\n" );
	PlayerProfile prof;
	const double vmax = prof.maxAirSpeed * prof.reachMargin;

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
	if ( c.gpuDiff >= 0 )
		Check( c.gpuDiff == 0, "96u gap: GPU agrees with the CPU on every arc (%d differ)", c.gpuDiff );

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
	if ( c.gpuDiff >= 0 )
		Check( c.gpuDiff == 0, "low slab: GPU agrees with the CPU on every arc (%d differ)", c.gpuDiff );
	boxes[2] = std::make_pair( dvec3( 32, -256, 120 ), dvec3( 64, 256, 200 ) );
	c = TraceGap( boxes, prof );
	Check( c.free > 0, "96u gap, slab at z 120: free arcs (%d arcs, %d free)", c.arcs, c.free );
	if ( c.gpuDiff >= 0 )
		Check( c.gpuDiff == 0, "high slab: GPU agrees with the CPU on every arc (%d differ)", c.gpuDiff );
	else
		Msg( "  (GPU checks skipped: no CUDA device / OptiX)\n" );
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

	// h6: segments against a convex: through, touching, ending on the face
	VisBoxPlanes( dvec3( 0, 0, 0 ), dvec3( 10, 10, 10 ), planes );
	Check( SegmentCrossesConvex( dvec3( -5, 5, 5 ), dvec3( 15, 5, 5 ), planes.data(), (int)planes.size() ), "segment through a box crosses it" );
	Check( !SegmentCrossesConvex( dvec3( -5, 10, 5 ), dvec3( 15, 10, 5 ), planes.data(), (int)planes.size() ),
		   "segment along a face (y = 10) only touches" );
	Check( !SegmentCrossesConvex( dvec3( -5, 5, 5 ), dvec3( 0, 5, 5 ), planes.data(), (int)planes.size() ),
		   "segment ending on the face (x = 0) does not enter" );
	Check( !SegmentCrossesConvex( dvec3( -5, 12, 5 ), dvec3( 15, 12, 5 ), planes.data(), (int)planes.size() ), "segment beside the box misses" );

	// h7: f16's mirror: eye (-32, 256, 64) in the water plane z = -192
	dvec3 m = MirrorPoint( dvec3( -32, 256, 64 ), dvec3( 0, 0, 1 ), -192 );
	Check( Near( m.x, -32 ) && Near( m.y, 256 ) && Near( m.z, -448 ), "mirror of (-32 256 64) in z = -192 is (-32 256 -448) (got %g %g %g)",
		   m.x, m.y, m.z );

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
		return;
	}
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
	BwOptix_Shutdown();
}

int RunGrowSelfTest()
{
	s_nFailures = 0;
	s_nChecks = 0;
	Msg( "bspwalk -selftest: Minkowski grow, walkable polygon, edge, arc and visibility known answers\n" );
	TestCube();
	TestRamp();
	TestTriangle();
	TestSubtract();
	TestEdges();
	TestArcs();
	TestSpecial();
	TestVis();
	TestVisGpu();
	Msg( "selftest: %d checks, %d failed -> %s\n", s_nChecks, s_nFailures, s_nFailures ? "FAIL" : "PASS" );
	return s_nFailures;
}
