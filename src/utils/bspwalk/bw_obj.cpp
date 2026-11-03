//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: -dumpobj (see bw_obj.h for the file layout and Blender settings).
//
//=============================================================================//

#include "cmdlib.h"
#include "bw_obj.h"
#include "bw_walk.h"

// OBJ object names: keep [A-Za-z0-9_-], fold everything else to '_', drop
// the "models/" prefix and ".mdl" suffix of model labels.
static void ObjectName( const SourceConvex &sc, int n, char *out, int outLen )
{
	char label[96];
	const char *src = sc.label;
	if ( !Q_strnicmp( src, "models/", 7 ) )
		src += 7;
	Q_strncpy( label, src, sizeof( label ) );
	int len = Q_strlen( label );
	if ( len > 4 && !Q_stricmp( label + len - 4, ".mdl" ) )
		label[len - 4] = 0;
	for ( char *p = label; *p; p++ )
	{
		char c = *p;
		if ( !( ( c >= 'a' && c <= 'z' ) || ( c >= 'A' && c <= 'Z' ) || ( c >= '0' && c <= '9' ) || c == '_' || c == '-' ) )
			*p = '_';
	}
	if ( label[0] && sc.category != CAT_WORLD )
		Q_snprintf( out, outLen, "%s_%d_%s", CategoryName( sc.category ), n, label );
	else
		Q_snprintf( out, outLen, "%s_%d", CategoryName( sc.category ), n );
}

// mode 0 = raw source, 1 = stand, 2 = crouch
static bool WriteObj( const CollisionWorld &world, const char *pszPath, int mode )
{
	FILE *fp = fopen( pszPath, "w" );
	if ( !fp )
	{
		Warning( "bspwalk: cannot write %s\n", pszPath );
		return false;
	}
	static char s_Buffer[1 << 20];
	setvbuf( fp, s_Buffer, _IOFBF, sizeof( s_Buffer ) );

	static const char *s_Modes[3] = { "source convexes (ungrown, all categories)", "grown, standing hull", "grown, crouching hull" };
	fprintf( fp, "# bspwalk collision dump: %s\n", s_Modes[mode] );
	fprintf( fp, "# Source units, Z up. Blender import: Up Axis = Z, Forward Axis = Y.\n" );
	fprintf( fp, "# Objects are named <category>_<n>[_<label>]; categories:" );
	for ( int c = 0; c < CAT_COUNT; c++ )
		fprintf( fp, " %s", CategoryName( c ) );
	fprintf( fp, "\n" );

	int counters[CAT_COUNT] = { 0 };
	int vertBase = 1;
	char name[160];
	for ( size_t i = 0; i < world.convexes.size(); i++ )
	{
		const SourceConvex &sc = world.convexes[i];
		int n = counters[sc.category]++;

		const std::vector<dvec3> *verts;
		const std::vector< std::vector<int> > *faces;
		if ( mode == 0 )
		{
			verts = &sc.shape.verts;
			faces = &sc.shape.faces;
		}
		else
		{
			if ( !CategoryIsGrown( sc.category ) )
				continue;
			const GrownPolytope &p = ( mode == 1 ) ? world.stand[i] : world.crouch[i];
			if ( !p.IsValid() )
				continue;
			verts = &p.verts;
			faces = &p.faces;
		}

		ObjectName( sc, n, name, sizeof( name ) );
		fprintf( fp, "o %s\ng %s\n", name, name );
		for ( size_t v = 0; v < verts->size(); v++ )
			fprintf( fp, "v %.6f %.6f %.6f\n", ( *verts )[v].x, ( *verts )[v].y, ( *verts )[v].z );
		for ( size_t f = 0; f < faces->size(); f++ )
		{
			const std::vector<int> &face = ( *faces )[f];
			fprintf( fp, "f" );
			for ( size_t k = 0; k < face.size(); k++ )
				fprintf( fp, " %d", vertBase + face[k] );
			fprintf( fp, "\n" );
		}
		vertBase += (int)verts->size();
	}

	bool ok = !ferror( fp );
	fclose( fp );
	return ok;
}

bool DumpWorldObj( const CollisionWorld &world, const char *pszBase )
{
	static const char *s_Suffix[3] = { "_raw.obj", "_stand.obj", "_crouch.obj" };
	bool ok = true;
	for ( int mode = 0; mode < 3; mode++ )
	{
		char path[MAX_PATH];
		Q_snprintf( path, sizeof( path ), "%s%s", pszBase, s_Suffix[mode] );
		if ( WriteObj( world, path, mode ) )
			Msg( "wrote %s\n", path );
		else
			ok = false;
	}
	return ok;
}

//-----------------------------------------------------------------------------
// Walkable polygons, split by the flood result.
//-----------------------------------------------------------------------------
static bool WriteReachObj( const CollisionWorld &world, const WalkGraph &graph, const char *pszPath, bool bReached )
{
	FILE *fp = fopen( pszPath, "w" );
	if ( !fp )
	{
		Warning( "bspwalk: cannot write %s\n", pszPath );
		return false;
	}
	static char s_Buffer[1 << 20];
	setvbuf( fp, s_Buffer, _IOFBF, sizeof( s_Buffer ) );

	fprintf( fp, "# bspwalk walkable polygons (crouch hull feet points): %s\n", bReached ? "reached" : "not reached" );
	fprintf( fp, "# Source units, Z up. Blender import: Up Axis = Z, Forward Axis = Y.\n" );
	fprintf( fp, "# Objects are named <source convex>_p<pose>_n<node>; the convex name matches the collision dumps.\n" );

	// per-category counter of each convex, as WriteObj numbers them
	std::vector<int> nth( world.convexes.size() );
	int counters[CAT_COUNT] = { 0 };
	for ( size_t i = 0; i < world.convexes.size(); i++ )
		nth[i] = counters[world.convexes[i].category]++;

	int vertBase = 1;
	char name[160], full[200];
	for ( size_t i = 0; i < graph.nodes.size(); i++ )
	{
		if ( ( graph.reached[i] != 0 ) != bReached )
			continue;
		const WalkNode &n = graph.nodes[i];
		ObjectName( world.convexes[n.convex], nth[n.convex], name, sizeof( name ) );
		Q_snprintf( full, sizeof( full ), "%s_p%d_n%d", name, n.pose, (int)i );
		fprintf( fp, "o %s\ng %s\n", full, full );
		for ( size_t v = 0; v < n.poly.size(); v++ )
			fprintf( fp, "v %.6f %.6f %.6f\n", n.poly[v].x, n.poly[v].y, n.poly[v].z );
		fprintf( fp, "f" );
		for ( size_t v = 0; v < n.poly.size(); v++ )
			fprintf( fp, " %d", vertBase + (int)v );
		fprintf( fp, "\n" );
		vertBase += (int)n.poly.size();
	}

	bool ok = !ferror( fp );
	fclose( fp );
	return ok;
}

bool DumpReachObj( const CollisionWorld &world, const WalkGraph &graph, const char *pszBase )
{
	bool ok = true;
	for ( int k = 0; k < 2; k++ )
	{
		char path[MAX_PATH];
		Q_snprintf( path, sizeof( path ), "%s%s", pszBase, k ? "_unreached.obj" : "_reached.obj" );
		if ( WriteReachObj( world, graph, path, k == 0 ) )
			Msg( "wrote %s\n", path );
		else
			ok = false;
	}
	return ok;
}
