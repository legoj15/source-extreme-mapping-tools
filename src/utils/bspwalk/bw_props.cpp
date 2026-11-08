//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: prop collision through vphysics. Static props are read the way
//          CVradStaticPropMgr::UnserializeStaticProps() reads them
//          (vradstaticprops.cpp); collision is the .phy's solids[0], loaded
//          with VCollideLoad and read back through ICollisionQuery.
//
//=============================================================================//

#include "cmdlib.h"
#include "bsplib.h"
#include "filesystem_tools.h"
#include "physdll.h"
#include "phyfile.h"
#include "vcollide.h"
#include "vphysics_interface.h"
#include "studio.h"
#include "gamebspfile.h"
#include "utlbuffer.h"
#include <functional>
#include "bw_props.h"
#include "bw_util.h"

// 'IDST'
#define BW_IDSTUDIOHEADER ( ( 'T' << 24 ) + ( 'S' << 16 ) + ( 'D' << 8 ) + 'I' )

// SolidType_t values (const.h; game code, not in the tools tree)
enum
{
	BW_SOLID_NONE = 0,
	BW_SOLID_BSP = 1,
	BW_SOLID_BBOX = 2,
	BW_SOLID_VPHYSICS = 6,
};

static IPhysicsCollision *s_pPhysCollision;

struct PropModel
{
	char name[MAX_PATH];
	bool mdlOk;
	dvec3 hullMin, hullMax;				// studiohdr_t hull_min / hull_max
	bool phyOk;
	std::vector<ConvexShape> convexes;	// solids[0] convex pieces, model space
	int convexesFailed;
	char phyProblem[64];				// why phyOk is false ("" when it is true)
	bool warnedFallback;				// the hull-box fallback was announced
	bool failuresCounted;				// convexesFailed went into the stats
	int uses;
};

static std::vector<PropModel *> s_Models;

int Props_ModelCount()
{
	return (int)s_Models.size();
}

//-----------------------------------------------------------------------------
// vphysics
//-----------------------------------------------------------------------------
void Props_Init()
{
	CreateInterfaceFn physicsFactory = GetPhysicsFactory();
	char path[MAX_PATH];
	Q_snprintf( path, sizeof( path ), "%s..\\bin\\x64\\vphysics.dll", gamedir );
	if ( !physicsFactory )
	{
		// The search path may only hold the 32-bit bin; try the game's bin\x64.
		PhysicsDLLPath( path );
		physicsFactory = GetPhysicsFactory();
	}
	if ( !physicsFactory )
		Error( "bspwalk: unable to load vphysics.dll (needed for prop collision) for game \"%s\"; tried the search "
			   "path and %s\n", gamedir, path );

	s_pPhysCollision = (IPhysicsCollision *)physicsFactory( VPHYSICS_COLLISION_INTERFACE_VERSION, NULL );
	if ( !s_pPhysCollision )
		Error( "bspwalk: unable to get '%s' from vphysics.\n", VPHYSICS_COLLISION_INTERFACE_VERSION );
}

//-----------------------------------------------------------------------------
// Model loading (.mdl header hull + .phy convexes), cached by name
//-----------------------------------------------------------------------------
static void LoadPropModel( PropModel &m )
{
	CUtlBuffer buf;
	if ( g_pFullFileSystem->ReadFile( m.name, NULL, buf ) && buf.TellPut() >= (int)sizeof( studiohdr_t ) )
	{
		const studiohdr_t *hdr = (const studiohdr_t *)buf.Base();
		if ( hdr->id == BW_IDSTUDIOHEADER )
		{
			m.mdlOk = true;
			m.hullMin = dvec3( hdr->hull_min );
			m.hullMax = dvec3( hdr->hull_max );
			// a model compiled without a hull falls back to its view box
			if ( m.hullMin.x >= m.hullMax.x && m.hullMin.y >= m.hullMax.y && m.hullMin.z >= m.hullMax.z )
			{
				m.hullMin = dvec3( hdr->view_bbmin );
				m.hullMax = dvec3( hdr->view_bbmax );
			}
		}
	}
	if ( !m.mdlOk )
		Warning( "bspwalk: cannot load model \"%s\"\n", m.name );

	// .phy: when it is missing or unreadable, a vphysics-solid prop falls back
	// to the .mdl hull box (AddPropInstance), with one warning per model.
	char phyName[MAX_PATH];
	Q_strncpy( phyName, m.name, sizeof( phyName ) );
	Q_SetExtension( phyName, ".phy", sizeof( phyName ) );
	CUtlBuffer phy;
	if ( !g_pFullFileSystem->ReadFile( phyName, NULL, phy ) )
	{
		Q_strncpy( m.phyProblem, "no .phy", sizeof( m.phyProblem ) );
		return;
	}
	if ( phy.TellPut() < (int)sizeof( phyheader_t ) )
	{
		Q_strncpy( m.phyProblem, ".phy shorter than its header", sizeof( m.phyProblem ) );
		return;
	}

	phyheader_t header;
	phy.Get( &header, sizeof( header ) );
	if ( header.size != sizeof( header ) || header.solidCount <= 0 )
	{
		Q_snprintf( m.phyProblem, sizeof( m.phyProblem ), ".phy header malformed (size %d, %d solids)", header.size,
					header.solidCount );
		return;
	}

	vcollide_t collide;
	memset( &collide, 0, sizeof( collide ) );
	s_pPhysCollision->VCollideLoad( &collide, header.solidCount, (const char *)phy.PeekGet(), phy.TellPut() - phy.TellGet() );
	if ( !collide.solidCount || !collide.solids[0] )
	{
		Q_strncpy( m.phyProblem, "vphysics could not load the .phy", sizeof( m.phyProblem ) );
		return;
	}

	// The engine's static prop and prop_physics collision is solids[0].
	ICollisionQuery *query = s_pPhysCollision->CreateQueryModel( collide.solids[0] );
	std::vector<dvec3> tris;
	for ( int c = 0; c < query->ConvexCount(); c++ )
	{
		int nTris = query->TriangleCount( c );
		tris.resize( nTris * 3 );
		for ( int t = 0; t < nTris; t++ )
		{
			Vector v[3];
			query->GetTriangleVerts( c, t, v );
			for ( int k = 0; k < 3; k++ )
				tris[t * 3 + k] = dvec3( v[k] );
		}
		ConvexShape shape;
		if ( ConvexFromTriangles( tris.data(), nTris, shape ) )
			m.convexes.push_back( shape );
		else
			m.convexesFailed++;
	}
	s_pPhysCollision->DestroyQueryModel( query );
	s_pPhysCollision->VCollideUnload( &collide );
	m.phyOk = !m.convexes.empty();
	if ( !m.phyOk )
		Q_strncpy( m.phyProblem, "no .phy convex could be built", sizeof( m.phyProblem ) );
}

static PropModel &FindPropModel( const char *pszName )
{
	for ( size_t i = 0; i < s_Models.size(); i++ )
	{
		if ( !Q_stricmp( s_Models[i]->name, pszName ) )
		{
			s_Models[i]->uses++;
			return *s_Models[i];
		}
	}
	PropModel *m = new PropModel;
	Q_strncpy( m->name, pszName, sizeof( m->name ) );
	Q_FixSlashes( m->name, '/' );
	m->mdlOk = m->phyOk = false;
	m->convexesFailed = 0;
	m->phyProblem[0] = 0;
	m->warnedFallback = m->failuresCounted = false;
	m->uses = 1;
	LoadPropModel( *m );
	s_Models.push_back( m );
	return *m;
}

// Adds a prop instance; returns false if it has no usable collision.
static bool AddPropInstance( CollisionWorld &world, PropModel &m, int solid, const DTransform &xf, int category,
							 int sourceIndex, int entityIndex, const char *pszTargetname )
{
	if ( !m.failuresCounted )
	{
		world.stats.convexesBuildFailed += m.convexesFailed;	// per model, not per instance
		m.failuresCounted = true;
	}
	ConvexShape shape;
	if ( solid != BW_SOLID_BBOX && !m.phyOk && m.mdlOk )
	{
		// VRAD keeps such props solid too (vradstaticprops.cpp builds hulls of
		// the render meshes); the .mdl hull box is the coarser stand-in here.
		if ( !m.warnedFallback )
		{
			Warning( "bspwalk: %s: %s; using the .mdl hull box for its collision\n", m.name, m.phyProblem );
			m.warnedFallback = true;
		}
		world.stats.propsHullFallback++;
		solid = BW_SOLID_BBOX;
	}
	if ( solid == BW_SOLID_BBOX )
	{
		if ( !m.mdlOk )
			return false;
		ConvexFromBox( m.hullMin, m.hullMax, xf, shape );
		world.convexes.push_back( SourceConvex() );
		SourceConvex &sc = world.convexes.back();
		sc.shape = shape;
		sc.category = category;
		sc.sourceIndex = sourceIndex;
		sc.entityIndex = entityIndex;
		Q_strncpy( sc.label, m.name, sizeof( sc.label ) );
		Q_strncpy( sc.targetname, pszTargetname ? pszTargetname : "", sizeof( sc.targetname ) );
		return true;
	}

	if ( !m.phyOk )
		return false;
	for ( size_t c = 0; c < m.convexes.size(); c++ )
	{
		shape = m.convexes[c];
		TransformShape( shape, xf );
		world.convexes.push_back( SourceConvex() );
		SourceConvex &sc = world.convexes.back();
		sc.shape = shape;
		sc.category = category;
		sc.sourceIndex = sourceIndex;
		sc.entityIndex = entityIndex;
		Q_strncpy( sc.label, m.name, sizeof( sc.label ) );
		Q_strncpy( sc.targetname, pszTargetname ? pszTargetname : "", sizeof( sc.targetname ) );
	}
	return true;
}

//-----------------------------------------------------------------------------
// Static props (game lump 'sprp')
//-----------------------------------------------------------------------------
// Calls fn for every prop of the lump: its index, placement, solid byte and
// model name (NULL when its dictionary index is out of range). 'warn' is off for
// a second pass over the same lump, whose problems the first pass reported.
typedef std::function<void( int index, const Vector &origin, const QAngle &angles, unsigned char solid, const char *model )>
	StaticPropFn;

static void ReadStaticPropLump( bool warn, const StaticPropFn &fn )
{
	GameLumpHandle_t handle = g_GameLumps.GetGameLumpHandle( GAMELUMP_STATIC_PROPS );
	if ( handle == g_GameLumps.InvalidGameLump() )
		return;
	int size = g_GameLumps.GameLumpSize( handle );
	void *pLump = g_GameLumps.GetGameLump( handle );
	if ( !size || !pLump )
		return;
	int version = g_GameLumps.GetGameLumpVersion( handle );
	if ( version < 4 )
	{
		if ( warn )
			Warning( "bspwalk: static prop lump version %d is too old; static props skipped\n", version );
		return;
	}

	CUtlBuffer buf( pLump, size, CUtlBuffer::READ_ONLY );
	// counts are bounded by the bytes left, so a corrupt lump is skipped
	// loudly instead of driving a huge allocation
	int dictCount = buf.GetInt();
	if ( dictCount < 0 || (int64)dictCount * (int64)sizeof( StaticPropDictLump_t ) > size - buf.TellGet() )
	{
		if ( warn )
			Warning( "bspwalk: static prop dictionary count %d does not fit the lump; ALL STATIC PROPS SKIPPED\n", dictCount );
		return;
	}
	std::vector<StaticPropDictLump_t> dict( dictCount );
	for ( int i = 0; i < dictCount; i++ )
		buf.Get( &dict[i], sizeof( StaticPropDictLump_t ) );

	// skip the leaf list
	int leafCount = buf.GetInt();
	if ( leafCount < 0 || (int64)leafCount * (int64)sizeof( StaticPropLeafLump_t ) > size - buf.TellGet() )
	{
		if ( warn )
			Warning( "bspwalk: static prop leaf count %d does not fit the lump; ALL STATIC PROPS SKIPPED\n", leafCount );
		return;
	}
	buf.SeekGet( CUtlBuffer::SEEK_CURRENT, leafCount * sizeof( StaticPropLeafLump_t ) );

	int propCount = buf.GetInt();
	if ( propCount <= 0 )
		return;

	// Entry size from the version, as the engine's struct definitions give it
	// (gamebspfile.h has v4, v5, v6 and v10); the lump must hold exactly
	// propCount of them. Other versions (7-9 from other branches, 11+) are not
	// defined in this SDK: their stride is inferred from the lump and said so.
	int remaining = size - buf.TellGet();
	int stride = 0;
	switch ( version )
	{
	case 4:		stride = sizeof( StaticPropLumpV4_t ); break;
	case 5:		stride = sizeof( StaticPropLumpV5_t ); break;
	case 6:		stride = sizeof( StaticPropLumpV6_t ); break;
	case GAMELUMP_STATIC_PROPS_VERSION:	stride = sizeof( StaticPropLump_t ); break;
	}
	if ( stride )
	{
		if ( remaining != propCount * stride )
		{
			if ( warn )
				Warning( "bspwalk: static prop lump v%d holds %d bytes for %d props, expected %d x %d = %d; "
						 "the lump is corrupt or mislabelled, ALL STATIC PROPS SKIPPED\n",
						 version, remaining, propCount, propCount, stride, propCount * stride );
			return;
		}
	}
	else
	{
		stride = remaining / propCount;
		if ( warn )
			Warning( "bspwalk: static prop lump v%d is not defined in this SDK; reading %d-byte entries inferred from "
					 "the lump size (%d bytes for %d props)%s\n", version, stride, remaining, propCount,
					 remaining % propCount ? ", WHICH DOES NOT DIVIDE EVENLY" : "" );
		if ( stride < (int)sizeof( StaticPropLumpV4_t ) || remaining % propCount )
		{
			if ( warn )
				Warning( "bspwalk: ALL STATIC PROPS SKIPPED\n" );
			return;
		}
		if ( warn && version > GAMELUMP_STATIC_PROPS_VERSION )
			Warning( "bspwalk: static prop lump v%d: prop scale is not read (unit scale assumed)\n", version );
	}
	// Every version shares the prefix origin (0), angles (12), proptype (24),
	// first/count leaf (26, 28), solid (30).

	const unsigned char *base = (const unsigned char *)buf.PeekGet();
	for ( int i = 0; i < propCount; i++ )
	{
		const unsigned char *p = base + i * stride;
		Vector origin;
		QAngle angles;
		unsigned short propType;
		unsigned char solid;
		memcpy( &origin, p + 0, sizeof( origin ) );
		memcpy( &angles, p + 12, sizeof( angles ) );
		memcpy( &propType, p + 24, sizeof( propType ) );
		solid = p[30];

		char name[STATIC_PROP_NAME_LENGTH + 1];
		if ( propType < dict.size() )
			Q_strncpy( name, dict[propType].m_Name, sizeof( name ) );
		fn( i, origin, angles, solid, propType < dict.size() ? name : NULL );
	}
}

void CollectStaticProps( CollisionWorld &world )
{
	ReadStaticPropLump( true, [&world]( int i, const Vector &origin, const QAngle &angles, unsigned char solid,
										const char *name ) {
		if ( solid == BW_SOLID_NONE )
		{
			world.stats.staticPropsNonSolid++;
			return;
		}
		if ( !name )
		{
			world.stats.staticPropsNoCollision++;
			return;
		}

		PropModel &m = FindPropModel( name );

		DTransform xf;
		xf.SetOrgAngles( dvec3( origin ), angles.x, angles.y, angles.z );
		if ( !AddPropInstance( world, m, solid, xf, CAT_STATIC_PROP, i, -1, NULL ) )
			world.stats.staticPropsNoCollision++;
	} );
}

//-----------------------------------------------------------------------------
// Every static prop's collision, for tests that must not change the world
//-----------------------------------------------------------------------------
void CollectStaticPropInstances( std::vector<StaticPropInstance> &out, int &unloadable )
{
	out.clear();
	unloadable = 0;
	ReadStaticPropLump( false, [&out, &unloadable]( int i, const Vector &origin, const QAngle &angles, unsigned char solid,
													const char *name ) {
		if ( !name )
			return;
		PropModel &m = FindPropModel( name );
		// A prop drawn without collision (SOLID_NONE) still has its model's. The shape is the .phy's
		// convexes, or the .mdl hull box when there is no usable .phy (as AddPropInstance does) or
		// the prop is solid BBOX.
		bool hullBox = solid == BW_SOLID_BBOX || !m.phyOk;
		if ( hullBox && !m.mdlOk )
		{
			unloadable++;
			return;
		}
		StaticPropInstance inst;
		inst.index = i;
		inst.model = m.name;
		inst.hullBox = hullBox;
		inst.xf.SetOrgAngles( dvec3( origin ), angles.x, angles.y, angles.z );
		inst.handle = &m;
		dvec3 lo( 1e300, 1e300, 1e300 ), hi( -1e300, -1e300, -1e300 );
		std::vector<dvec3> corners;
		if ( hullBox )
		{
			for ( int k = 0; k < 8; k++ )
				corners.push_back( dvec3( k & 1 ? m.hullMax.x : m.hullMin.x, k & 2 ? m.hullMax.y : m.hullMin.y,
										  k & 4 ? m.hullMax.z : m.hullMin.z ) );
		}
		else
		{
			for ( size_t c = 0; c < m.convexes.size(); c++ )
			{
				const dvec3 &a = m.convexes[c].mins, &b = m.convexes[c].maxs;
				for ( int k = 0; k < 8; k++ )
					corners.push_back( dvec3( k & 1 ? b.x : a.x, k & 2 ? b.y : a.y, k & 4 ? b.z : a.z ) );
			}
		}
		for ( size_t k = 0; k < corners.size(); k++ )
		{
			dvec3 w = inst.xf.Apply( corners[k] );
			lo = DMin( lo, w );
			hi = DMax( hi, w );
		}
		inst.mins = lo;
		inst.maxs = hi;
		out.push_back( inst );
	} );
}

void StaticPropShapes( const StaticPropInstance &inst, std::vector<ConvexShape> &out )
{
	out.clear();
	const PropModel &m = *(const PropModel *)inst.handle;
	if ( inst.hullBox )
	{
		out.resize( 1 );
		ConvexFromBox( m.hullMin, m.hullMax, inst.xf, out[0] );
		return;
	}
	out = m.convexes;
	for ( size_t c = 0; c < out.size(); c++ )
		TransformShape( out[c], inst.xf );
}

//-----------------------------------------------------------------------------
// Entity props: grown, standable only (they move or break)
//-----------------------------------------------------------------------------
void CollectEntityProps( CollisionWorld &world )
{
	for ( int e = 0; e < num_entities; e++ )
	{
		entity_t *ent = &entities[e];
		const char *cls = EntValue( ent, "classname" );
		bool physics = !Q_strnicmp( cls, "prop_physics", 12 );
		bool dynamic = !Q_strnicmp( cls, "prop_dynamic", 12 );
		if ( !physics && !dynamic )
			continue;

		const char *model = EntValue( ent, "model" );
		if ( !model[0] )
		{
			world.stats.entityPropsNoCollision++;
			continue;
		}

		// prop_physics is always vphysics; prop_dynamic honours its "solid" key
		int solid = BW_SOLID_VPHYSICS;
		if ( dynamic )
		{
			const char *s = EntValue( ent, "solid" );
			if ( s[0] )
				solid = atoi( s );
			if ( solid == BW_SOLID_NONE )
			{
				world.stats.entityPropsNonSolid++;
				continue;
			}
			if ( solid != BW_SOLID_BBOX )
				solid = BW_SOLID_VPHYSICS;
		}

		Vector origin( 0, 0, 0 );
		QAngle angles( 0, 0, 0 );
		GetVectorForKey( ent, (char *)"origin", origin );
		GetAnglesForKey( ent, (char *)"angles", angles );
		double scale = 1.0;
		const char *ms = EntValue( ent, "modelscale" );
		if ( ms[0] && atof( ms ) > 0 )
			scale = atof( ms );

		DTransform xf;
		xf.SetOrgAngles( dvec3( origin ), angles.x, angles.y, angles.z, scale );
		PropModel &m = FindPropModel( model );
		if ( !AddPropInstance( world, m, solid, xf, CAT_ENTITY_PROP, e, e, EntValue( ent, "targetname" ) ) )
			world.stats.entityPropsNoCollision++;
	}
}

//-----------------------------------------------------------------------------
// -checkprops
//-----------------------------------------------------------------------------
int CheckPropHulls()
{
	int mismatches = 0;
	Msg( "checkprops: %d models (collision AABB in model space vs .mdl hull_min/hull_max)\n", (int)s_Models.size() );
	for ( size_t i = 0; i < s_Models.size(); i++ )
	{
		const PropModel &m = *s_Models[i];
		if ( !m.phyOk )
		{
			Msg( "  %-56s %s%s (uses %d)\n", m.name, m.phyProblem, m.mdlOk ? "" : ", no .mdl", m.uses );
			continue;
		}
		dvec3 mins( 1e300, 1e300, 1e300 ), maxs( -1e300, -1e300, -1e300 );
		for ( size_t c = 0; c < m.convexes.size(); c++ )
		{
			mins = DMin( mins, m.convexes[c].mins );
			maxs = DMax( maxs, m.convexes[c].maxs );
		}
		double diff = 0;
		if ( m.mdlOk )
		{
			for ( int k = 0; k < 3; k++ )
			{
				diff = MAX( diff, fabs( mins[k] - m.hullMin[k] ) );
				diff = MAX( diff, fabs( maxs[k] - m.hullMax[k] ) );
			}
		}
		bool bad = !m.mdlOk || diff > 1.0;
		mismatches += bad;
		Msg( "  %-56s phy [%.2f %.2f %.2f]-[%.2f %.2f %.2f] hull [%.2f %.2f %.2f]-[%.2f %.2f %.2f] maxdiff %.3f%s\n",
			 m.name, mins.x, mins.y, mins.z, maxs.x, maxs.y, maxs.z,
			 m.hullMin.x, m.hullMin.y, m.hullMin.z, m.hullMax.x, m.hullMax.y, m.hullMax.z, diff,
			 bad ? "  MISMATCH" : "" );
	}
	Msg( "checkprops: %d mismatch(es) > 1u\n", mismatches );
	return mismatches;
}
