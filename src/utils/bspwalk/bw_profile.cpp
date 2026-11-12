//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: game profiles (bspwalk_profiles.txt next to the exe, KeyValues).
//
//=============================================================================//

#include "cmdlib.h"
#include "tier1/KeyValues.h"
#include "tier1/strtools.h"
#include "bw_grow.h"
#include "bw_arcs.h"
#include <windows.h>

// intended_air_speed, when a profile omits it, is this times max_ground_speed.
// SDK 2013 gamemovement.cpp: a sprinting HL2 jump adds at most 10% (CheckJumpButton,
// flSpeedBoostPerc 0.1, capped at m_flMaxSpeed * 1.1 = 352), and in the air
// AirAccelerate adds at most GetAirSpeedCap() = 30 u/s per tick across the
// velocity, so |v|^2 grows by at most 900 a tick: about 400 after a 0.6 s jump
// at 66.67 ticks/s (sqrt(352^2 + 900 * 40)). 1.25 x 320 = 400.
static const double INTENDED_AIR_FACTOR = 1.25;

PlayerProfile::PlayerProfile()
{
	// Built-in hl2 (SDK 2013, HL2_DLL).
	Q_strncpy( name, "hl2", sizeof( name ) );
	Q_strncpy( source, "default", sizeof( source ) );
	fallback = false;
	halfWidth = 16.0;
	standHeight = 72.0;
	crouchHeight = 36.0;
	eyeStand = 64.0;
	eyeCrouch = 28.0;
	eyeDead = 14.0;
	stepHeight = 18.0;
	gravity = 600.0;
	jumpVelocity = 160.0;
	crouchLift = 36.0;
	runSpeed = 190.0;
	maxGroundSpeed = 320.0;
	maxAirSpeed = 640.0;
	intendedAirSpeed = INTENDED_AIR_FACTOR * maxGroundSpeed;
	reachMargin = 1.15;
	airJumps = 0;
	// PLAYER_FATAL_FALL_SPEED 922.5 (shareddefs.h, HL2_DLL) at gravity 600, from a jump's apex
	arcMaxDrop = 922.5 * 922.5 / ( 2.0 * 600.0 ) + 160.0 * 160.0 / ( 2.0 * 600.0 );
	minLedgeWidth = 1.0;
	exploits = false;
	Derive();
}

double PlayerProfile::ArcReachAtDrop( double drop ) const
{
	double g = gravity > 0 ? gravity : 1.0;
	double v = jumpVelocity > 0 ? jumpVelocity : 0.0;
	int k = airJumps > 0 ? airJumps : 0;
	drop = MAX( 0.0, MIN( drop, arcMaxDrop ) );
	double t = ( 1 + k ) * v / g + sqrt( 2.0 * ( maxJumpRise + drop ) / g );
	return airSpeed * reachMargin * t + 2.0 * halfWidth;
}

void PlayerProfile::Derive()
{
	double g = gravity > 0 ? gravity : 1.0;
	double v = jumpVelocity > 0 ? jumpVelocity : 0.0;
	int k = airJumps > 0 ? airJumps : 0;
	// -exploits never reaches less than the default: a profile whose max_air_speed is below its
	// intended_air_speed keeps the intended one
	airSpeed = exploits ? MAX( maxAirSpeed, intendedAirSpeed ) : intendedAirSpeed;
	jumpApex = v * v / ( 2.0 * g );
	maxJumpRise = jumpApex * ( 1 + k ) + crouchLift;
	// rise through every jump, then fall from the top (lifted by the duck) back to the launch height
	arcAirtime = ( 1 + k ) * v / g + sqrt( 2.0 * maxJumpRise / g );
	arcReach = airSpeed * reachMargin * arcAirtime + 2.0 * halfWidth;
	arcReachMax = ArcReachAtDrop( arcMaxDrop );
}

// Directory of the running exe, with a trailing separator.
static void ExeDir( char *out, int len )
{
	char path[MAX_PATH];
	DWORD n = GetModuleFileNameA( NULL, path, sizeof( path ) );
	if ( n == 0 || n >= sizeof( path ) )
	{
		out[0] = 0;
		return;
	}
	char *slash = strrchr( path, '\\' );
	if ( slash )
		slash[1] = 0;
	Q_strncpy( out, path, len );
}

// The last component of a directory path, lowercased.
static void LastComponent( const char *pszDir, char *out, int len )
{
	char tmp[MAX_PATH];
	Q_strncpy( tmp, pszDir ? pszDir : "", sizeof( tmp ) );
	int n = Q_strlen( tmp );
	while ( n > 0 && ( tmp[n - 1] == '\\' || tmp[n - 1] == '/' ) )
		tmp[--n] = 0;
	const char *p = tmp;
	for ( const char *s = tmp; *s; s++ )
	{
		if ( *s == '\\' || *s == '/' )
			p = s + 1;
	}
	Q_strncpy( out, p, len );
	Q_strlower( out );
}

// Does the space-separated list contain 'word' (case-insensitive)?
static bool ListHas( const char *pszList, const char *word )
{
	char tok[64];
	const char *s = pszList;
	while ( *s )
	{
		while ( *s == ' ' || *s == '\t' || *s == ',' )
			s++;
		int n = 0;
		while ( *s && *s != ' ' && *s != '\t' && *s != ',' )
		{
			if ( n < (int)sizeof( tok ) - 1 )
				tok[n++] = *s;
			s++;
		}
		tok[n] = 0;
		if ( n && !Q_stricmp( tok, word ) )
			return true;
	}
	return false;
}

// The longest arc any map can ask for falls from the top of the world to its
// bottom (coordinates within +-16384). Its chords must fit ARC_MAX_CHORDS, or the
// tracers would have to drop some.
static bool ValidateArcChords( const PlayerProfile &p )
{
	const double H = 2.0 * 16384.0 + p.crouchLift;
	int worst = 0;
	for ( int kick = 0; kick <= ( p.airJumps > 0 ? 1 : 0 ); kick++ )
	{
		for ( int jump = 0; jump < 2; jump++ )
		{
			double vz = jump ? p.jumpVelocity : 0.0;
			ArcD a;
			if ( SolveArcToPoint( dvec3( 0, 0, 0 ), vz, kick && vz > 0.0, p.gravity, dvec3( 0, 0, -H ), 1e30,
								  BW_MESH_CROUCH, a ) )
				worst = MAX( worst, a.nSeg[0] + a.nSeg[1] );
		}
	}
	if ( worst > ARC_MAX_CHORDS )
	{
		Warning( "bspwalk: profile \"%s\": a %.0fu fall needs %d arc chords, more than the %d the tracers hold "
				 "(gravity or jump_velocity out of range)\n", p.name, H, worst, ARC_MAX_CHORDS );
		return false;
	}
	return true;
}

static bool ReadProfile( KeyValues *kv, PlayerProfile &p )
{
	PlayerProfile def;
	Q_strncpy( p.name, kv->GetName(), sizeof( p.name ) );
	Q_strncpy( p.source, "file", sizeof( p.source ) );
	struct
	{
		const char *key;
		double *dst;
		double fallback;
	} fields[] =
	{
		{ "half_width", &p.halfWidth, def.halfWidth },
		{ "stand_height", &p.standHeight, def.standHeight },
		{ "crouch_height", &p.crouchHeight, def.crouchHeight },
		{ "eye_stand", &p.eyeStand, def.eyeStand },
		{ "eye_crouch", &p.eyeCrouch, def.eyeCrouch },
		{ "eye_dead", &p.eyeDead, def.eyeDead },
		{ "step_height", &p.stepHeight, def.stepHeight },
		{ "gravity", &p.gravity, def.gravity },
		{ "jump_velocity", &p.jumpVelocity, def.jumpVelocity },
		{ "crouch_lift", &p.crouchLift, -1.0 },
		{ "run_speed", &p.runSpeed, def.runSpeed },
		{ "max_ground_speed", &p.maxGroundSpeed, def.maxGroundSpeed },
		{ "max_air_speed", &p.maxAirSpeed, -1.0 },
		{ "intended_air_speed", &p.intendedAirSpeed, -1.0 },
		{ "reach_margin", &p.reachMargin, def.reachMargin },
		{ "arc_max_drop", &p.arcMaxDrop, def.arcMaxDrop },
		{ "min_ledge_width", &p.minLedgeWidth, def.minLedgeWidth },
	};
	// read as strings (KeyValues keeps them so) to get double precision
	for ( int i = 0; i < ARRAYSIZE( fields ); i++ )
	{
		const char *s = kv->GetString( fields[i].key, "" );
		*fields[i].dst = s[0] ? atof( s ) : fields[i].fallback;
	}
	if ( p.crouchLift < 0 )
		p.crouchLift = p.standHeight - p.crouchHeight;
	if ( p.maxAirSpeed < 0 )
		p.maxAirSpeed = 2.0 * p.maxGroundSpeed;
	if ( p.intendedAirSpeed < 0 )
		p.intendedAirSpeed = INTENDED_AIR_FACTOR * p.maxGroundSpeed;
	p.airJumps = kv->GetInt( "air_jumps", 0 );

	if ( p.halfWidth <= 0 || p.crouchHeight <= 0 || p.standHeight < p.crouchHeight || p.gravity <= 0 ||
		 p.jumpVelocity < 0 || p.maxAirSpeed <= 0 || p.intendedAirSpeed <= 0 || p.reachMargin < 1.0 ||
		 p.airJumps < 0 || p.airJumps > 1 || p.arcMaxDrop < 0 || p.eyeDead < 0 || p.minLedgeWidth < 0 )
	{
		Warning( "bspwalk: profile \"%s\" is malformed (sizes, gravity and air speeds must be positive, reach_margin "
				 ">= 1, air_jumps 0 or 1, arc_max_drop, eye_dead and min_ledge_width >= 0)\n", p.name );
		return false;
	}
	p.Derive();
	return ValidateArcChords( p );
}

bool LoadPlayerProfile( const char *pszName, const char *pszGameDir, bool exploits, PlayerProfile &out )
{
	char dir[MAX_PATH], path[MAX_PATH];
	ExeDir( dir, sizeof( dir ) );
	V_snprintf( path, sizeof( path ), "%sbspwalk_profiles.txt", dir );

	char gameName[MAX_PATH];
	LastComponent( pszGameDir, gameName, sizeof( gameName ) );

	FILE *fp = fopen( path, "rb" );
	if ( !fp )
	{
		out = PlayerProfile();
		if ( pszName && Q_stricmp( pszName, "hl2" ) )
		{
			Warning( "bspwalk: -profile %s: %s is missing, only the built-in hl2 profile exists\n", pszName, path );
			return false;
		}
		Warning( "bspwalk: %s is missing; using the built-in hl2 profile\n", path );
		out.fallback = !pszName;
		// the built-in profile takes the mode too: surf reads -exploits from the command line, and a
		// run with surf on but the intended air speed and no pixel walks would be neither mode
		out.exploits = exploits;
		out.Derive();
		return true;
	}
	fseek( fp, 0, SEEK_END );
	long size = ftell( fp );
	if ( size < 0 )
	{
		fclose( fp );
		Warning( "bspwalk: cannot read %s\n", path );
		return false;
	}
	fseek( fp, 0, SEEK_SET );
	std::vector<char> buf( size + 1 );
	size_t got = fread( buf.data(), 1, size, fp );
	fclose( fp );
	buf[got] = 0;

	KeyValues *root = new KeyValues( "bspwalk_profiles" );
	if ( !root->LoadFromBuffer( path, buf.data() ) )
	{
		root->deleteThis();
		Warning( "bspwalk: cannot parse %s\n", path );
		return false;
	}

	// -profile <name>; else the profile named like the game directory or listing it in
	// "gamedirs"; else hl2.
	KeyValues *pick = NULL;
	const char *why = "";
	if ( pszName )
	{
		pick = root->FindKey( pszName );
		why = "-profile";
		if ( !pick )
		{
			Warning( "bspwalk: -profile %s: no such profile in %s\n", pszName, path );
			root->deleteThis();
			return false;
		}
	}
	else
	{
		for ( KeyValues *k = root->GetFirstTrueSubKey(); k && !pick; k = k->GetNextTrueSubKey() )
		{
			if ( gameName[0] && ( !Q_stricmp( k->GetName(), gameName ) || ListHas( k->GetString( "gamedirs", "" ), gameName ) ) )
			{
				pick = k;
				why = "game directory";
			}
		}
		if ( !pick )
		{
			pick = root->FindKey( "hl2" );
			why = "fallback";
		}
	}

	bool ok;
	if ( pick )
		ok = ReadProfile( pick, out );
	else
	{
		Warning( "bspwalk: %s has no hl2 profile; using the built-in one\n", path );
		out = PlayerProfile();
		ok = true;
	}
	root->deleteThis();
	if ( ok )
	{
		// An unknown game directory gets hl2's movement, which under-reaches a game whose
		// player moves farther (#49: GMod maps fell back silently): said loudly, and in the JSON.
		out.fallback = !pszName && Q_strcmp( why, "game directory" ) != 0;
		if ( out.fallback )
			Warning( "bspwalk: game directory \"%s\" matches no profile in %s; using the %s profile as a fallback "
					 "(reach, and so visibility, may be under-estimated: add a profile, or pick one with -profile)\n",
					 gameName, path, out.name );
		out.exploits = exploits;
		out.Derive();
		char ledge[64];
		if ( exploits )
			Q_strncpy( ledge, "pixel walks kept", sizeof( ledge ) );
		else
			V_snprintf( ledge, sizeof( ledge ), "min ledge width %.2fu", out.minLedgeWidth );
		Msg( "bspwalk: profile %s (%s, %s; game directory \"%s\"): %s, air speed %.0fu/s (max_air_speed %.0f, "
			 "intended_air_speed %.0f), reach %.1fu (%.1fu at the %.1fu max drop), jump rise %.2fu, %s\n",
			 out.name, out.source, why, gameName, exploits ? "exploits (-exploits)" : "intended movement", out.airSpeed,
			 out.maxAirSpeed, out.intendedAirSpeed, out.arcReach, out.arcReachMax, out.arcMaxDrop, out.maxJumpRise, ledge );
	}
	return ok;
}
