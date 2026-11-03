//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: entry point and CLI.
//
//   bspwalk <map.bsp> -game <dir> [-dumpobj <base>] [-checkprops] [-verify]
//           [-selftest] [-threads N] [-out <json>]
//
// Milestone 2 (#11): loads the BSP, builds the player collision world in
// configuration space (Stage 1), and optionally dumps it as OBJ. -out writes
// run stats and an empty "faces" list (the harness contract); visibility
// comes in later milestones.
//
//=============================================================================//

#include "cmdlib.h"
#include "bsplib.h"
#include "threads.h"
#include "tier0/icommandline.h"
#include "tier0/platform.h"
#include "mathlib/mathlib.h"
#include "bw_grow.h"
#include "bw_world.h"
#include "bw_props.h"
#include "bw_disp.h"
#include "bw_obj.h"

#define BSPWALK_MILESTONE 2

struct StageTimes
{
	double init, load, brushes, props, disps, grow, verify, dumpobj, total;
	StageTimes() { memset( this, 0, sizeof( *this ) ); }
};

static void PrintUsage()
{
	Msg( "usage: bspwalk <map.bsp> -game <dir> [options]\n"
		 "  -dumpobj <base>  write <base>_raw.obj, <base>_stand.obj, <base>_crouch.obj\n"
		 "  -checkprops      compare each prop model's collision AABB with its .mdl hull\n"
		 "  -verify          check every grown polytope against its exact Minkowski point set\n"
		 "  -verifydump <f>  with -verify: write every failing convex to <f> (for -growdebug / offline checks)\n"
		 "  -selftest        known-answer checks of the grow routine (no map needed)\n"
		 "  -threads <n>     worker threads (default: all cores)\n"
		 "  -out <json>      write run stats as JSON\n" );
}

// The first argument that is neither an option nor an option's value.
static const char *FindBspArg( int argc, char **argv )
{
	static const char *s_ValueOptions[] = { "-game", "-vproject", "-dumpobj", "-out", "-threads", "-growdebug", "-verifydump" };
	for ( int i = 1; i < argc; i++ )
	{
		if ( argv[i][0] == '-' )
		{
			for ( int k = 0; k < ARRAYSIZE( s_ValueOptions ); k++ )
			{
				if ( !Q_stricmp( argv[i], s_ValueOptions[k] ) )
				{
					i++;
					break;
				}
			}
			continue;
		}
		return argv[i];
	}
	return NULL;
}

static void JsonString( FILE *fp, const char *s )
{
	fputc( '"', fp );
	for ( ; *s; s++ )
	{
		if ( *s == '"' || *s == '\\' )
			fprintf( fp, "\\%c", *s );
		else if ( (unsigned char)*s < 0x20 )
			fprintf( fp, "\\u%04x", *s );
		else
			fputc( *s, fp );
	}
	fputc( '"', fp );
}

// False when the category has no convexes (the bounds are then meaningless).
static bool WorldAABB( const CollisionWorld &world, int category, dvec3 &mins, dvec3 &maxs )
{
	mins = dvec3( 1e300, 1e300, 1e300 );
	maxs = dvec3( -1e300, -1e300, -1e300 );
	bool any = false;
	for ( size_t i = 0; i < world.convexes.size(); i++ )
	{
		if ( world.convexes[i].category != category )
			continue;
		mins = DMin( mins, world.convexes[i].shape.mins );
		maxs = DMax( maxs, world.convexes[i].shape.maxs );
		any = true;
	}
	return any;
}

static int CountGrown( const std::vector<GrownPolytope> &v )
{
	int n = 0;
	for ( size_t i = 0; i < v.size(); i++ )
		n += v[i].IsValid();
	return n;
}

static bool WriteJson( const char *pszPath, const char *pszBsp, const CollisionWorld &world, const PlayerProfile &prof,
					   const StageTimes &t, int verifyFailures, int propMismatches )
{
	FILE *fp = fopen( pszPath, "w" );
	if ( !fp )
		return false;

	const CollectStats &s = world.stats;
	dvec3 wmin, wmax;
	bool bWorld = WorldAABB( world, CAT_WORLD, wmin, wmax );

	fprintf( fp, "{\n  \"walker\": \"bspwalk\",\n  \"milestone\": %d,\n  \"bsp\": ", BSPWALK_MILESTONE );
	JsonString( fp, pszBsp );
	fprintf( fp, ",\n  \"profile\": {\"name\": \"%s\", \"half_width\": %g, \"stand_height\": %g, \"crouch_height\": %g},\n",
			 prof.name, prof.halfWidth, prof.standHeight, prof.crouchHeight );

	fprintf( fp, "  \"counts\": {" );
	for ( int c = 0; c < CAT_COUNT; c++ )
		fprintf( fp, "\"%s\": %d, ", CategoryName( c ), world.CountCategory( c ) );
	fprintf( fp, "\"convexes\": %d, \"grown_stand\": %d, \"grown_crouch\": %d, \"prop_models\": %d},\n",
			 (int)world.convexes.size(), CountGrown( world.stand ), CountGrown( world.crouch ), Props_ModelCount() );

	fprintf( fp, "  \"skipped\": {\"brushes_wrong_contents\": %d, \"brushes_build_failed\": %d, "
			 "\"brush_entities_nonsolid_class\": %d, \"brush_entities_passable\": %d, \"brush_entities_never_solid\": %d, "
			 "\"brush_entities_no_solid_brush\": %d, \"static_props_nonsolid\": %d, \"static_props_no_collision\": %d, "
			 "\"entity_props_nonsolid\": %d, \"entity_props_no_collision\": %d, \"disps_no_hull_collision\": %d, "
			 "\"disp_tris_degenerate\": %d, \"prop_convexes_build_failed\": %d, \"grow_failed\": %d},\n",
			 s.brushesWrongContents, s.brushesBuildFailed, s.brushEntitiesNonSolidClass, s.brushEntitiesPassable,
			 s.brushEntitiesNeverSolid, s.brushEntitiesNoSolidBrush, s.staticPropsNonSolid, s.staticPropsNoCollision,
			 s.entityPropsNonSolid, s.entityPropsNoCollision, s.dispsNoHullCollision, s.dispTrisDegenerate,
			 s.convexesBuildFailed, s.growFailed );
	fprintf( fp, "  \"prop_instances_hull_fallback\": %d,\n", s.propsHullFallback );

	if ( bWorld )
		fprintf( fp, "  \"world_aabb\": [[%.3f, %.3f, %.3f], [%.3f, %.3f, %.3f]],\n", wmin.x, wmin.y, wmin.z, wmax.x, wmax.y, wmax.z );
	else
		fprintf( fp, "  \"world_aabb\": null,\n" );
	fprintf( fp, "  \"model0_aabb\": [[%.3f, %.3f, %.3f], [%.3f, %.3f, %.3f]],\n",
			 dmodels[0].mins.x, dmodels[0].mins.y, dmodels[0].mins.z, dmodels[0].maxs.x, dmodels[0].maxs.y, dmodels[0].maxs.z );

	if ( verifyFailures >= 0 )
		fprintf( fp, "  \"verify_failures\": %d,\n", verifyFailures );
	if ( propMismatches >= 0 )
		fprintf( fp, "  \"checkprops_mismatches\": %d,\n", propMismatches );

	fprintf( fp, "  \"timings_s\": {\"init\": %.3f, \"load\": %.3f, \"brushes\": %.3f, \"props\": %.3f, \"disps\": %.3f, "
			 "\"grow\": %.3f, \"verify\": %.3f, \"dumpobj\": %.3f, \"total\": %.3f},\n",
			 t.init, t.load, t.brushes, t.props, t.disps, t.grow, t.verify, t.dumpobj, t.total );
	fprintf( fp, "  \"faces\": []\n}\n" );

	bool ok = !ferror( fp );
	fclose( fp );
	return ok;
}

static int GrowDebugFile( const char *pszPath )
{
	FILE *fp = fopen( pszPath, "r" );
	if ( !fp )
		Error( "bspwalk: cannot read %s\n", pszPath );
	std::vector<dvec3> pts;
	char line[512];
	int blocks = 0;
	while ( fgets( line, sizeof( line ), fp ) )
	{
		// a -verifydump file holds many convexes; use the first
		if ( line[0] == '#' && ++blocks > 1 )
			break;
		dvec3 v;
		if ( sscanf( line, " v %lf %lf %lf", &v.x, &v.y, &v.z ) == 3 )
			pts.push_back( v );
	}
	fclose( fp );

	ConvexShape shape;
	if ( !ConvexFromPoints( pts.data(), (int)pts.size(), shape ) )
		Error( "bspwalk: %d points do not form a solid\n", (int)pts.size() );
	Msg( "hull: %d of %d points, %d faces, %d edges\n", (int)shape.verts.size(), (int)pts.size(),
		 (int)shape.faces.size(), (int)shape.edges.size() );

	PlayerProfile prof;
	int failures = 0;
	for ( int posture = 0; posture < 2; posture++ )
	{
		PlayerHull hull = posture ? prof.Crouch() : prof.Stand();
		Msg( "%s:\n", posture ? "crouch" : "stand" );
		GrownPolytope g;
		g_bGrowDebug = true;
		bool ok = GrowConvex( shape, hull, g );
		g_bGrowDebug = false;
		char why[256] = "";
		ok = ok && VerifyGrown( shape, hull, g, why, sizeof( why ) );
		Msg( "%s: %s %s\n", posture ? "crouch" : "stand", ok ? "verified" : "FAILED", why );
		failures += !ok;
	}
	return failures ? 1 : 0;
}

int main( int argc, char **argv )
{
	double tStart = Plat_FloatTime();

	CommandLine()->CreateCmdLine( argc, argv );
	MathLib_Init( 2.2f, 2.2f, 0.0f, 1.0f, false, false, false, false );
	InstallSpewFunction();

	int exitCode = 0;
	bool selftest = CommandLine()->FindParm( "-selftest" ) != 0;
	if ( selftest && RunGrowSelfTest() )
		exitCode = 1;

	// -growdebug <file>: grow the hull of the "v x y z" lines of a file (as
	// printed by -verify for a failing convex) with every decision printed.
	const char *pszGrowDebug = CommandLine()->ParmValue( "-growdebug", (const char *)NULL );
	if ( pszGrowDebug )
		return GrowDebugFile( pszGrowDebug );

	const char *pszBspArg = FindBspArg( argc, argv );
	if ( !pszBspArg )
	{
		if ( !selftest )
		{
			PrintUsage();
			return 1;
		}
		return exitCode;
	}

	// Without -game the filesystem falls back to the VPROJECT environment
	// variable, as every SDK tool does. Say which game that is, since a stale
	// VPROJECT shows up later as missing models or a vphysics.dll load failure.
	if ( !CommandLine()->FindParm( "-game" ) && !CommandLine()->FindParm( "-vproject" ) )
	{
		const char *pszEnv = getenv( "VPROJECT" );
		if ( !pszEnv || !pszEnv[0] )
		{
			Warning( "bspwalk: no game directory: pass -game <dir> (the directory holding gameinfo.txt) or set VPROJECT\n" );
			PrintUsage();
			return 1;
		}
		Msg( "bspwalk: no -game given, using VPROJECT=%s\n", pszEnv );
	}

	// a relative path is taken against the process cwd; made absolute here so
	// the filesystem (which searches the game's paths) cannot resolve it elsewhere
	char szBsp[MAX_PATH];
	V_MakeAbsolutePath( szBsp, sizeof( szBsp ), pszBspArg );
	const char *pszBsp = szBsp;

	// -1 = all cores. A zero, negative or non-numeric count would run no
	// workers at all (nothing grown, nothing verified), so it falls back.
	numthreads = -1;
	if ( CommandLine()->FindParm( "-threads" ) )
	{
		const char *pszThreads = CommandLine()->ParmValue( "-threads", "" );
		int n = atoi( pszThreads );
		if ( n > 0 )
			numthreads = n;
		else
			Warning( "bspwalk: -threads \"%s\" is not a positive count; using all cores\n", pszThreads );
	}
	ThreadSetDefault();

	StageTimes t;
	double t0 = Plat_FloatTime();
	CmdLib_InitFileSystem( pszBsp );
	Props_Init();
	t.init = Plat_FloatTime() - t0;

	t0 = Plat_FloatTime();
	LoadBSPFile( pszBsp );
	ParseEntities();
	t.load = Plat_FloatTime() - t0;
	Msg( "bspwalk: %s (%d brushes, %d models, %d entities, %d displacements), %d threads\n",
		 pszBsp, numbrushes, nummodels, num_entities, g_dispinfo.Count(), numthreads );

	CollisionWorld world;
	PlayerProfile profile;

	t0 = Plat_FloatTime();
	CollectWorldBrushes( world );
	CollectBrushEntities( world );
	t.brushes = Plat_FloatTime() - t0;

	t0 = Plat_FloatTime();
	CollectStaticProps( world );
	CollectEntityProps( world );
	t.props = Plat_FloatTime() - t0;

	t0 = Plat_FloatTime();
	CollectDisplacements( world );
	t.disps = Plat_FloatTime() - t0;

	t0 = Plat_FloatTime();
	GrowWorld( world, profile );
	t.grow = Plat_FloatTime() - t0;
	if ( world.stats.growFailed )
		Warning( "bspwalk: %d convexes failed to grow (degenerate)\n", world.stats.growFailed );

	int verifyFailures = -1;
	if ( CommandLine()->FindParm( "-verify" ) )
	{
		t0 = Plat_FloatTime();
		verifyFailures = VerifyWorld( world, profile, CommandLine()->ParmValue( "-verifydump", (const char *)NULL ) );
		t.verify = Plat_FloatTime() - t0;
		Msg( "verify: %d polytope(s) differ from their exact Minkowski sum\n", verifyFailures );
		if ( verifyFailures )
			exitCode = 1;
	}

	const char *pszObj = CommandLine()->ParmValue( "-dumpobj", (const char *)NULL );
	if ( pszObj )
	{
		t0 = Plat_FloatTime();
		if ( !DumpWorldObj( world, pszObj ) )
			exitCode = 1;
		t.dumpobj = Plat_FloatTime() - t0;
	}

	int propMismatches = -1;
	if ( CommandLine()->FindParm( "-checkprops" ) )
		propMismatches = CheckPropHulls();

	t.total = Plat_FloatTime() - tStart;

	//
	// Summary
	//
	const CollectStats &s = world.stats;
	Msg( "\nconvexes by category:\n" );
	for ( int c = 0; c < CAT_COUNT; c++ )
		Msg( "  %-18s %7d%s\n", CategoryName( c ), world.CountCategory( c ),
			 CategoryIsGrown( c ) ? ( CategoryIsObstacle( c ) ? "" : "  (grown, standable only)" ) : "  (not grown)" );
	Msg( "  grown: %d stand, %d crouch; %d prop models\n", CountGrown( world.stand ), CountGrown( world.crouch ), Props_ModelCount() );
	Msg( "skipped: brushes wrong contents %d, brush build failed %d; brush entities: non-solid class %d, passable %d, "
		 "never solid %d, no solid brush %d\n",
		 s.brushesWrongContents, s.brushesBuildFailed, s.brushEntitiesNonSolidClass, s.brushEntitiesPassable,
		 s.brushEntitiesNeverSolid, s.brushEntitiesNoSolidBrush );
	Msg( "skipped: static props non-solid %d, no collision %d; entity props non-solid %d, no collision %d; "
		 "disps no hull collision %d, degenerate disp tris %d, bad prop convexes %d\n",
		 s.staticPropsNonSolid, s.staticPropsNoCollision, s.entityPropsNonSolid, s.entityPropsNoCollision,
		 s.dispsNoHullCollision, s.dispTrisDegenerate, s.convexesBuildFailed );
	if ( s.propsHullFallback )
		Msg( "props without a usable .phy, kept as their .mdl hull box: %d instance(s)\n", s.propsHullFallback );

	dvec3 wmin, wmax;
	if ( WorldAABB( world, CAT_WORLD, wmin, wmax ) )
		Msg( "world brush AABB [%.2f %.2f %.2f]-[%.2f %.2f %.2f], model 0 [%.2f %.2f %.2f]-[%.2f %.2f %.2f]\n",
			 wmin.x, wmin.y, wmin.z, wmax.x, wmax.y, wmax.z,
			 dmodels[0].mins.x, dmodels[0].mins.y, dmodels[0].mins.z, dmodels[0].maxs.x, dmodels[0].maxs.y, dmodels[0].maxs.z );
	else
		Warning( "bspwalk: no player-solid world brushes (model 0 is empty or entirely non-solid)\n" );
	Msg( "timings (s): init %.3f, load %.3f, brushes %.3f, props %.3f, disps %.3f, grow %.3f, verify %.3f, dumpobj %.3f, total %.3f\n",
		 t.init, t.load, t.brushes, t.props, t.disps, t.grow, t.verify, t.dumpobj, t.total );

	const char *pszOut = CommandLine()->ParmValue( "-out", (const char *)NULL );
	if ( pszOut )
	{
		if ( WriteJson( pszOut, pszBsp, world, profile, t, verifyFailures, propMismatches ) )
			Msg( "wrote %s\n", pszOut );
		else
		{
			Warning( "bspwalk: cannot write %s\n", pszOut );
			exitCode = 1;
		}
	}
	return exitCode;
}
