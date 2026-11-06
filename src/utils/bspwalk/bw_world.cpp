//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: brush collection (world + brush entities) with the brush-entity
//          policy of the design doc (Stage 3), and the threaded grow pass.
//
//=============================================================================//

#include "cmdlib.h"
#include "bsplib.h"
#include "bspflags.h"
#include "threads.h"
#include "tier0/threadtools.h"
#include "bw_world.h"
#include "bw_util.h"

//-----------------------------------------------------------------------------
// Categories
//-----------------------------------------------------------------------------
const char *CategoryName( int category )
{
	static const char *s_Names[CAT_COUNT] =
	{
		"world", "func_brush_solid", "tops_only", "static_prop", "entity_prop", "disp", "water",
	};
	return ( category >= 0 && category < CAT_COUNT ) ? s_Names[category] : "unknown";
}

bool CategoryIsGrown( int category )
{
	return category != CAT_TOPS_ONLY;
}

bool CategoryIsObstacle( int category )
{
	return category == CAT_WORLD || category == CAT_FUNC_BRUSH_SOLID || category == CAT_STATIC_PROP || category == CAT_DISP;
}

int CollisionWorld::CountCategory( int category ) const
{
	int n = 0;
	for ( size_t i = 0; i < convexes.size(); i++ )
		n += ( convexes[i].category == category );
	return n;
}

//-----------------------------------------------------------------------------
// Brush-entity policy, one table so CLI overrides can be added later.
//
// nonSolid:       the class never blocks the player, whatever its contents.
// passableFlags:  spawnflags that make an otherwise solid class passable
//                 (values from base.fgd).
// anglesAreMoveDir: the game turns "angles" into a move direction and spawns
//                 the model unrotated (CBaseToggle movers).
//-----------------------------------------------------------------------------
struct BrushClassRule
{
	const char *classname;
	bool prefix;
	bool nonSolid;
	int passableFlags;
	bool anglesAreMoveDir;
};

static const BrushClassRule s_BrushClassRules[] =
{
	// never solid to the player
	{ "trigger_",				true,	true,	0,	false },
	{ "func_illusionary",		false,	true,	0,	false },
	{ "func_clip_vphysics",		false,	true,	0,	false },
	{ "func_areaportal",		true,	true,	0,	false },	// func_areaportal, func_areaportalwindow
	{ "func_occluder",			false,	true,	0,	false },
	{ "func_viscluster",		false,	true,	0,	false },
	{ "func_dustmotes",			false,	true,	0,	false },
	{ "func_dustcloud",			false,	true,	0,	false },
	{ "func_smokevolume",		false,	true,	0,	false },
	{ "func_precipitation",		false,	true,	0,	false },
	{ "func_buyzone",			false,	true,	0,	false },
	{ "func_bomb_target",		false,	true,	0,	false },
	{ "func_hostage_rescue",	false,	true,	0,	false },
	{ "func_nobuild",			false,	true,	0,	false },
	{ "func_respawnroom",		true,	true,	0,	false },	// func_respawnroom, func_respawnroomvisualizer
	{ "func_ladder",			false,	true,	0,	false },	// vbsp moves its brushes to the world (CONTENTS_LADDER, bw_special.cpp)
	{ "func_water_analog",		false,	true,	0,	true },

	// solid unless a spawnflag says otherwise
	{ "func_door",				false,	false,	4 | 8,	true },	// "Non-solid to Player", "Passable" (base.fgd Door)
	{ "func_door_rotating",		false,	false,	4 | 8,	false },	// "Non-solid to Player", "Passable" (base.fgd Door)
	{ "func_movelinear",		false,	false,	8,	true },		// "Not Solid"
	{ "func_rotating",			false,	false,	64,	false },	// "Not Solid"
	{ "func_rot_button",		false,	false,	1,	false },	// "Not solid"
	{ "momentary_rot_button",	false,	false,	1,	false },	// "Not Solid"
	{ "func_conveyor",			false,	false,	2,	true },		// "Not Solid"
	{ "func_tracktrain",		false,	false,	8,	false },	// "Passable"
	{ "func_tanktrain",			false,	false,	8,	false },	// "Passable"
	{ "func_button",			false,	false,	0,	true },
};

static const BrushClassRule *FindBrushClassRule( const char *pszClass )
{
	for ( int i = 0; i < ARRAYSIZE( s_BrushClassRules ); i++ )
	{
		const BrushClassRule &r = s_BrushClassRules[i];
		if ( r.prefix ? !Q_strnicmp( pszClass, r.classname, Q_strlen( r.classname ) ) : !Q_stricmp( pszClass, r.classname ) )
			return &r;
	}
	return NULL;
}

//-----------------------------------------------------------------------------
// Brushes
//-----------------------------------------------------------------------------
static void CollectModelBrushes_r( int node, std::vector<int> &brushes, std::vector<unsigned char> &seen )
{
	if ( node < 0 )
	{
		const dleaf_t &leaf = dleafs[-1 - node];
		for ( int i = 0; i < leaf.numleafbrushes; i++ )
		{
			int b = dleafbrushes[leaf.firstleafbrush + i];
			if ( !seen[b] )
			{
				seen[b] = 1;
				brushes.push_back( b );
			}
		}
		return;
	}
	CollectModelBrushes_r( dnodes[node].children[0], brushes, seen );
	CollectModelBrushes_r( dnodes[node].children[1], brushes, seen );
}

// All brushes of a model, deduped (a brush appears in every leaf it touches).
void CollectModelBrushes( int model, std::vector<int> &brushes )
{
	std::vector<unsigned char> seen( numbrushes, 0 );
	brushes.clear();
	CollectModelBrushes_r( dmodels[model].headnode, brushes, seen );
}

// The brush as a convex from its non-bevel sides (bevels only support the
// brush; the grow adds its own edge x axis bevels).
bool BuildBrushShape( int brushIndex, ConvexShape &out )
{
	const dbrush_t &brush = dbrushes[brushIndex];
	DPlane planes[256];
	int n = 0;
	for ( int i = 0; i < brush.numsides && n < ARRAYSIZE( planes ); i++ )
	{
		const dbrushside_t &side = dbrushsides[brush.firstside + i];
		// low byte is the bevel flag; the high byte is "thin" on BSP v20
		if ( side.bevel & 0xFF )
			continue;
		const dplane_t &pl = dplanes[side.planenum];
		planes[n].n = dvec3( pl.normal );
		planes[n].d = pl.dist;
		n++;
	}
	return ConvexFromPlanes( planes, n, out );
}

static void AddConvex( CollisionWorld &world, const ConvexShape &shape, int category, int sourceIndex, int entityIndex,
					   const char *pszLabel, const char *pszTargetname )
{
	world.convexes.push_back( SourceConvex() );
	SourceConvex &sc = world.convexes.back();
	sc.shape = shape;
	sc.category = category;
	sc.sourceIndex = sourceIndex;
	sc.entityIndex = entityIndex;
	Q_strncpy( sc.label, pszLabel ? pszLabel : "", sizeof( sc.label ) );
	Q_strncpy( sc.targetname, pszTargetname ? pszTargetname : "", sizeof( sc.targetname ) );
}

void CollectWorldBrushes( CollisionWorld &world )
{
	std::vector<int> brushes;
	CollectModelBrushes( 0, brushes );
	ConvexShape shape;
	for ( size_t i = 0; i < brushes.size(); i++ )
	{
		int b = brushes[i];
		int category = CAT_WORLD;
		if ( !( dbrushes[b].contents & MASK_PLAYERSOLID_BRUSHONLY ) )
		{
			// water and slime: swimmable volumes, their tops a surface (#13B)
			if ( !( dbrushes[b].contents & ( CONTENTS_WATER | CONTENTS_SLIME ) ) )
			{
				world.stats.brushesWrongContents++;
				continue;
			}
			category = CAT_WATER;
		}
		if ( !BuildBrushShape( b, shape ) )
		{
			world.stats.brushesBuildFailed++;
			continue;
		}
		AddConvex( world, shape, category, b, -1, NULL, NULL );
	}
}

void BrushEntityTransform( entity_t *ent, DTransform &xf )
{
	const BrushClassRule *rule = FindBrushClassRule( EntValue( ent, "classname" ) );
	Vector org( 0, 0, 0 );
	QAngle ang( 0, 0, 0 );
	GetVectorForKey( ent, (char *)"origin", org );
	if ( !( rule && rule->anglesAreMoveDir ) )
		GetAnglesForKey( ent, (char *)"angles", ang );
	xf = DTransform();
	xf.SetOrgAngles( dvec3( org ), ang.x, ang.y, ang.z );
}

//-----------------------------------------------------------------------------
// Every targetname that some output sends one of 'inputs' to. Outputs are "target,Input,param,delay,refire"
// with ',' or 0x1B. With 'selfTargeted', an entity whose own output sends one
// of them to !self is flagged there; with 'relative', an output sending one to
// !activator, !caller or !picker (an entity known only at run time) sets it.
//-----------------------------------------------------------------------------
static void CollectInputTargets( const char *const *inputs, int numInputs, std::vector<CUtlString> &targets,
								 std::vector<unsigned char> *selfTargeted = NULL, bool *relative = NULL )
{
	if ( selfTargeted )
		selfTargeted->assign( num_entities, 0 );
	if ( relative )
		*relative = false;
	for ( int e = 0; e < num_entities; e++ )
	{
		for ( epair_t *ep = entities[e].epairs; ep; ep = ep->next )
		{
			const char *v = ep->value;
			char sep = strchr( v, 0x1B ) ? 0x1B : ',';
			int seps = 0;
			for ( const char *p = v; *p; p++ )
				seps += ( *p == sep );
			if ( seps < 4 )
				continue;

			const char *c1 = strchr( v, sep );
			const char *c2 = strchr( c1 + 1, sep );
			char input[64];
			Q_strncpy( input, c1 + 1, MIN( (int)sizeof( input ), (int)( c2 - c1 ) ) );
			bool match = false;
			for ( int k = 0; k < numInputs && !match; k++ )
				match = !Q_stricmp( input, inputs[k] );
			if ( !match )
				continue;

			char target[128];
			Q_strncpy( target, v, MIN( (int)sizeof( target ), (int)( c1 - v ) + 1 ) );
			if ( selfTargeted && !Q_stricmp( target, "!self" ) )
				( *selfTargeted )[e] = 1;
			if ( relative &&
				 ( !Q_stricmp( target, "!activator" ) || !Q_stricmp( target, "!caller" ) || !Q_stricmp( target, "!picker" ) ) )
				*relative = true;
			targets.push_back( CUtlString( target ) );
		}
	}
}

static bool IsDisableTarget( const std::vector<CUtlString> &targets, const char *pszName )
{
	if ( !pszName[0] )
		return false;
	for ( size_t i = 0; i < targets.size(); i++ )
	{
		const char *t = targets[i].Get();
		int len = Q_strlen( t );
		// a trailing '*' is an I/O wildcard
		if ( len > 0 && t[len - 1] == '*' )
		{
			if ( !Q_strnicmp( t, pszName, len - 1 ) )
				return true;
		}
		else if ( !Q_stricmp( t, pszName ) )
			return true;
	}
	return false;
}

// Inputs that can make a solid func_brush stop blocking where it spawned:
// turn it off, remove it, move it (parent) or rekey it (AddOutput). Render-only
// inputs (Alpha, DisableDraw, SetRender*) do not change collision.
static void CollectDisableTargets( std::vector<CUtlString> &targets, std::vector<unsigned char> &self, bool &relative )
{
	static const char *const s_Inputs[] = { "Disable", "Toggle", "Kill", "KillHierarchy", "AddOutput",
											"SetParent", "SetParentAttachment", "SetParentAttachmentMaintainOffset" };
	CollectInputTargets( s_Inputs, ARRAYSIZE( s_Inputs ), targets, &self, &relative );
}

void BrushEntitiesDrawnOpaque( std::vector<unsigned char> &out )
{
	// inputs that can hide the entity, make it translucent, remove it, move it
	// or change any key (AddOutput): its sight occlusion cannot be relied on.
	// An entity can send one to itself (!self). Which entity !activator, !caller
	// or !picker is depends on the run, so any such output makes every brush
	// entity unreliable.
	static const char *const s_Inputs[] = { "Disable", "Toggle", "Kill", "KillHierarchy", "Alpha", "AddOutput",
											"DisableDraw", "SetParent", "SetParentAttachment",
											"SetParentAttachmentMaintainOffset", "SetRenderMode", "SetRenderFX" };
	std::vector<CUtlString> targets;
	std::vector<unsigned char> self;
	bool relative;
	CollectInputTargets( s_Inputs, ARRAYSIZE( s_Inputs ), targets, &self, &relative );
	out.assign( num_entities, 0 );
	if ( relative )
		return;
	for ( int e = 0; e < num_entities; e++ )
	{
		entity_t *ent = &entities[e];
		const char *amt = EntValue( ent, "renderamt" );
		bool opaque = atoi( EntValue( ent, "rendermode" ) ) == 0		// kRenderNormal (10 is kRenderNone)
					  && ( !amt[0] || atoi( amt ) >= 255 )			// unset is 255
					  && atoi( EntValue( ent, "renderfx" ) ) == 0
					  && !( atoi( EntValue( ent, "effects" ) ) & 0x20 )	// EF_NODRAW
					  && !atoi( EntValue( ent, "StartDisabled" ) )
					  && atof( EntValue( ent, "fademaxdist" ) ) <= 0	// fades out with distance
					  && !atoi( EntValue( ent, "mindxlevel" ) ) && !atoi( EntValue( ent, "maxdxlevel" ) )	// not drawn on
					  && !atoi( EntValue( ent, "mincpulevel" ) ) && !atoi( EntValue( ent, "maxcpulevel" ) )	// some
					  && !atoi( EntValue( ent, "mingpulevel" ) ) && !atoi( EntValue( ent, "maxgpulevel" ) )	// settings
					  && !EntValue( ent, "parentname" )[0]
					  && !self[e] && !IsDisableTarget( targets, EntValue( ent, "targetname" ) );
		out[e] = opaque;
	}
}

void CollectBrushEntities( CollisionWorld &world )
{
	// A func_brush any of these may reach is not a permanent obstacle: an output
	// to !activator, !caller or !picker may reach any of them (over-reach, safe).
	std::vector<CUtlString> disableTargets;
	std::vector<unsigned char> disableSelf;
	bool disableRelative;
	CollectDisableTargets( disableTargets, disableSelf, disableRelative );

	std::vector<int> brushes;
	ConvexShape shape;
	for ( int e = 0; e < num_entities; e++ )
	{
		entity_t *ent = &entities[e];
		const char *model = EntValue( ent, "model" );
		if ( model[0] != '*' )
			continue;
		int modelIndex = atoi( model + 1 );
		if ( modelIndex <= 0 || modelIndex >= nummodels )
			continue;

		const char *cls = EntValue( ent, "classname" );
		const char *targetname = EntValue( ent, "targetname" );
		int spawnflags = atoi( EntValue( ent, "spawnflags" ) );
		const BrushClassRule *rule = FindBrushClassRule( cls );

		if ( rule && rule->nonSolid )
		{
			world.stats.brushEntitiesNonSolidClass++;
			continue;
		}
		if ( rule && ( spawnflags & rule->passableFlags ) )
		{
			world.stats.brushEntitiesPassable++;
			continue;
		}

		int category = CAT_TOPS_ONLY;
		if ( !Q_stricmp( cls, "func_brush" ) )
		{
			int solidity = atoi( EntValue( ent, "Solidity" ) );		// 0 toggle, 1 never, 2 always
			if ( solidity == 1 )
			{
				world.stats.brushEntitiesNeverSolid++;
				continue;
			}
			bool parented = EntValue( ent, "parentname" )[0] != 0;
			bool startDisabled = atoi( EntValue( ent, "StartDisabled" ) ) != 0;
			if ( solidity == 2 && !parented && !startDisabled && !disableRelative && !disableSelf[e]
				 && !IsDisableTarget( disableTargets, targetname ) )
				category = CAT_FUNC_BRUSH_SOLID;
		}

		// Brush-entity geometry is stored relative to the entity origin.
		DTransform xf;
		BrushEntityTransform( ent, xf );

		CollectModelBrushes( modelIndex, brushes );
		int added = 0;
		for ( size_t i = 0; i < brushes.size(); i++ )
		{
			int b = brushes[i];
			if ( !( dbrushes[b].contents & MASK_PLAYERSOLID_BRUSHONLY ) )
			{
				world.stats.brushesWrongContents++;
				continue;
			}
			if ( !BuildBrushShape( b, shape ) )
			{
				world.stats.brushesBuildFailed++;
				continue;
			}
			TransformShape( shape, xf );
			AddConvex( world, shape, category, b, e, cls, targetname );
			added++;
		}
		if ( !added )
			world.stats.brushEntitiesNoSolidBrush++;
	}
}

//-----------------------------------------------------------------------------
// Grow pass
//-----------------------------------------------------------------------------
static CollisionWorld *s_pGrowWorld;
static PlayerHull s_StandHull, s_CrouchHull;
static int32 volatile s_nGrowFailed;

static void GrowOne( int iThread, int i )
{
	const SourceConvex &sc = s_pGrowWorld->convexes[i];
	if ( !CategoryIsGrown( sc.category ) )
		return;
	bool ok = GrowConvex( sc.shape, s_StandHull, s_pGrowWorld->stand[i] );
	ok = GrowConvex( sc.shape, s_CrouchHull, s_pGrowWorld->crouch[i] ) && ok;
	if ( !ok )
		ThreadInterlockedIncrement( &s_nGrowFailed );
}

void GrowWorld( CollisionWorld &world, const PlayerProfile &profile )
{
	world.stand.clear();
	world.crouch.clear();
	world.stand.resize( world.convexes.size() );
	world.crouch.resize( world.convexes.size() );

	s_pGrowWorld = &world;
	s_StandHull = profile.Stand();
	s_CrouchHull = profile.Crouch();
	s_nGrowFailed = 0;
	RunThreadsOnIndividual( (int)world.convexes.size(), false, GrowOne );
	world.stats.growFailed = s_nGrowFailed;
	s_pGrowWorld = NULL;
}

static const PlayerProfile *s_pVerifyProfile;
static const CollisionWorld *s_pVerifyWorld;
static int32 volatile s_nVerifyFailed;
static int32 volatile s_nVerifyChecked;
static FILE *s_pVerifyDump;

static void VerifyOne( int iThread, int i )
{
	const SourceConvex &sc = s_pVerifyWorld->convexes[i];
	if ( !CategoryIsGrown( sc.category ) )
		return;
	char why[256];
	for ( int posture = 0; posture < 2; posture++ )
	{
		const GrownPolytope &p = posture ? s_pVerifyWorld->crouch[i] : s_pVerifyWorld->stand[i];
		PlayerHull hull = posture ? s_pVerifyProfile->Crouch() : s_pVerifyProfile->Stand();
		ThreadInterlockedIncrement( &s_nVerifyChecked );
		if ( VerifyGrown( sc.shape, hull, p, why, sizeof( why ) ) )
			continue;

		int nFailed = ThreadInterlockedIncrement( &s_nVerifyFailed );
		if ( nFailed <= 10 )
			Warning( "verify: %s convex %d (source %d) %s: %s\n", CategoryName( sc.category ), i, sc.sourceIndex,
					 posture ? "crouch" : "stand", why );
		if ( s_pVerifyDump )
		{
			// source points and planes: enough to reproduce offline (-growdebug, qhull)
			ThreadLock();
			fprintf( s_pVerifyDump, "# convex %d %s source %d %s height %g: %s\n", i, CategoryName( sc.category ),
					 sc.sourceIndex, posture ? "crouch" : "stand", hull.height, why );
			for ( size_t v = 0; v < sc.shape.verts.size(); v++ )
				fprintf( s_pVerifyDump, "v %.17g %.17g %.17g\n", sc.shape.verts[v].x, sc.shape.verts[v].y, sc.shape.verts[v].z );
			for ( size_t k = 0; k < p.planes.size(); k++ )
				fprintf( s_pVerifyDump, "p %.17g %.17g %.17g %.17g\n", p.planes[k].n.x, p.planes[k].n.y, p.planes[k].n.z,
						 p.planes[k].d );
			ThreadUnlock();
		}
	}
}

int VerifyWorld( const CollisionWorld &world, const PlayerProfile &profile, const char *pszDumpPath )
{
	s_pVerifyWorld = &world;
	s_pVerifyProfile = &profile;
	s_nVerifyFailed = 0;
	s_nVerifyChecked = 0;
	s_pVerifyDump =pszDumpPath ? fopen( pszDumpPath, "w" ) : NULL;
	if ( pszDumpPath && !s_pVerifyDump )
		Warning( "bspwalk: cannot write %s\n", pszDumpPath );
	RunThreadsOnIndividual( (int)world.convexes.size(), false, VerifyOne );
	if ( s_pVerifyDump )
	{
		fclose( s_pVerifyDump );
		s_pVerifyDump = NULL;
		Msg( "verify: failing convexes written to %s\n", pszDumpPath );
	}

	// Every grown convex has two polytopes; one that was never checked counts
	// as failed, so a run that skipped work cannot report a clean verify.
	int expected = 0;
	for ( size_t i = 0; i < world.convexes.size(); i++ )
		expected += CategoryIsGrown( world.convexes[i].category ) ? 2 : 0;
	int unchecked = expected - (int)s_nVerifyChecked;
	if ( unchecked > 0 )
		Warning( "verify: only %d of %d polytopes were checked\n", (int)s_nVerifyChecked, expected );
	return s_nVerifyFailed + MAX( unchecked, 0 );
}
