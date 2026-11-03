//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: -selftest. Known-answer checks of the Minkowski grow routine. Every
//          answer here is derived by hand from P (+) (-B), with
//          support_{-B}(n) = 16|nx| + 16|ny| + h*max(0,-nz).
//
//=============================================================================//

#include "cmdlib.h"
#include "bw_grow.h"

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

int RunGrowSelfTest()
{
	s_nFailures = 0;
	s_nChecks = 0;
	Msg( "bspwalk -selftest: Minkowski grow known answers\n" );
	TestCube();
	TestRamp();
	TestTriangle();
	Msg( "selftest: %d checks, %d failed -> %s\n", s_nChecks, s_nFailures, s_nFailures ? "FAIL" : "PASS" );
	return s_nFailures;
}
