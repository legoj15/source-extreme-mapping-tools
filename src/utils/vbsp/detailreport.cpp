//========= -reportdetail: structural brushes that could be func_detail ========
//
// Issue #101. See detailreport.h and docs/vbsp_detail_report.md.
//
// Two facts are taken from the world tree and nothing else is touched:
//
//  - Does the brush face the void? Asked before FillOutside, which turns every
//    leaf the entity flood did not reach into solid and so erases the answer.
//    A brush faces the void when a solid leaf holding a piece of it has a
//    portal to an empty leaf nobody reached, to the edge of the world, or to
//    the solid placeholder the block pass makes for a grid cell with no brush
//    (see IsVoidLeaf). Such a brush may be part of what seals the map, so it is
//    never listed.
//  - What does the brush cost? Every node of the tree records which brush
//    supplied its split plane (node_t::sideBrush, set in SelectSplitSide while
//    each block's tree is built, so it survives the merge of the block trees).
//    A brush is charged the nodes in the final tree that carry its planes and
//    the portals in the .prt that lie on those nodes' planes (portal_t::onnode
//    is the node a portal was cut on, and the pieces it is split into keep it).
//
// Every portal in the .prt is counted at the point WritePortalFile has built
// the list, so the numbers are the ones the vis step is given, not an estimate.
//=============================================================================
#include "vbsp.h"
#include "detailreport.h"

#include "bspflags.h"
#include "bsplib.h"
#include "tier1/strtools.h"
#include <algorithm>
#include <stdio.h>
#include <string.h>
#include <vector>

bool g_bReportDetail = false;
int g_nReportDetailRows = DETAILREPORT_DEFAULT_ROWS;

// One flag per map brush: a leaf holding a piece of it borders unreached
// space. Filled by DetailReport_FindVoidFacing on the tree that is kept.
static CUtlVector<unsigned char> s_FacesVoid;
static bool s_bHaveVoidInfo = false;
static double s_flVoidSeconds = 0; // time of the void test, for the report's cost line

//-----------------------------------------------------------------------------
// Which brushes can be listed
//-----------------------------------------------------------------------------
// A listed brush is an opaque solid wall-like brush of the world entity. These
// contents make a brush something else, a volume or a tool whose job is not to
// be seen, so the brush is never listed:
//   water, slime               fog volumes; they must stay in the tree
//   window, grate, translucent glass and grates; the flood passes through them
//   blocklos                   AI sight volume
//   areaportal                 shapes the areas on purpose
//   playerclip, monsterclip    clip volumes (already detail)
//   origin, ladder             not geometry
//   detail                     already func_detail, not in the tree
static const int kNotAWall = CONTENTS_WATER | CONTENTS_SLIME | CONTENTS_WINDOW |
	CONTENTS_GRATE | CONTENTS_TRANSLUCENT | CONTENTS_BLOCKLOS |
	CONTENTS_AREAPORTAL | CONTENTS_PLAYERCLIP | CONTENTS_MONSTERCLIP |
	CONTENTS_ORIGIN | CONTENTS_LADDER | CONTENTS_DETAIL;

// A side with any of these makes the brush a tool brush that shapes vis or the
// seal on purpose: hint and skip (the mapper put the split there), sky (the
// seal itself), trigger (a xbox-style tool texture).
static const int kToolSurf = SURF_HINT | SURF_SKIP | SURF_SKY | SURF_TRIGGER;

static int SideSurf( const side_t *pSide )
{
	int surf = pSide->surf;
	if ( pSide->texinfo >= 0 && pSide->texinfo < texinfo.Count() )
		surf |= texinfo[pSide->texinfo].flags;
	return surf;
}

// Brushes of other entities are not candidates: func_detail and the rest are
// already their own thing, and func_areaportal brushes (moved into the world
// tree) shape the areas. A func_instance's world brushes are those of another
// entity too, and their ids are shifted, so they are left out as well.
static bool IsCandidate( const mapbrush_t *mb )
{
	if ( mb->entitynum != 0 || mb->numsides == 0 )
		return false;
	if ( !( mb->contents & CONTENTS_SOLID ) || ( mb->contents & kNotAWall ) )
		return false;

	// a brush that is nodraw on every side is a seal or vis shaper, not a wall
	bool bAllNoDraw = true;
	for ( int i = 0; i < mb->numsides; i++ )
	{
		const side_t *pSide = &mb->original_sides[i];
		if ( pSide->bevel )
			continue;
		int surf = SideSurf( pSide );
		if ( surf & kToolSurf )
			return false;
		if ( !( surf & SURF_NODRAW ) )
			bAllNoDraw = false;
	}
	return !bAllNoDraw;
}

// The material of the brush's first side that is drawn, to tell the mapper what
// the brush is: the first side in the file is often nodraw. A brush with no
// drawn side gets its first side's material. An env_cubemap patch ("maps/<map>/
// <material>_<x>_<y>_<z>", what the texdata holds by now) is shown as the
// material it patches.
static void FirstSideMaterial( const mapbrush_t *mb, char *pOut, int nOut )
{
	const side_t *pFirst = NULL, *pDrawn = NULL;
	for ( int i = 0; i < mb->numsides && !pDrawn; i++ )
	{
		const side_t *pSide = &mb->original_sides[i];
		if ( pSide->bevel || pSide->texinfo < 0 || pSide->texinfo >= texinfo.Count() )
			continue;
		if ( !pFirst )
			pFirst = pSide;
		if ( !( SideSurf( pSide ) & SURF_NODRAW ) )
			pDrawn = pSide;
	}
	const side_t *pSide = pDrawn ? pDrawn : pFirst;
	if ( !pSide )
	{
		V_strncpy( pOut, "?", nOut );
		return;
	}

	const char *pName = TexDataStringTable_GetString( GetTexData( texinfo[pSide->texinfo].texdata )->nameStringTableID );
	char szPrefix[128];
	V_snprintf( szPrefix, sizeof( szPrefix ), "maps/%s/", mapbase );
	V_strncpy( pOut, pName, nOut );
	if ( !V_strnicmp( pOut, szPrefix, V_strlen( szPrefix ) ) )
	{
		int nTail = V_strlen( pOut );
		// strip "_<int>_<int>_<int>" from the end
		int nUnderscores = 0, nCut = nTail;
		for ( int c = nTail - 1; c >= 0 && nUnderscores < 3; c-- )
		{
			if ( pOut[c] == '_' )
			{
				nUnderscores++;
				nCut = c;
			}
			else if ( !( ( pOut[c] >= '0' && pOut[c] <= '9' ) || pOut[c] == '-' ) )
			{
				nUnderscores = -1; // not an origin suffix
				break;
			}
		}
		if ( nUnderscores == 3 )
		{
			pOut[nCut] = 0;
			V_memmove( pOut, pOut + V_strlen( szPrefix ), nCut - V_strlen( szPrefix ) + 1 );
		}
	}
}

//-----------------------------------------------------------------------------
// Void-facing, asked on the tree before FillOutside
//-----------------------------------------------------------------------------
// Two kinds of leaf count as void:
//  - an empty leaf no entity can walk to: FillOutside is about to make it solid.
//    The edge of the world is one too (tree->outside_node has no contents and is
//    never occupied on a sealed map).
//  - a solid leaf with no brush list. The block pass makes one for a grid cell
//    that holds no structural brush (ProcessBlock_Thread: CONTENTS_SOLID, nothing
//    in brushlist), so a wall that ends on that cell's edge has its portal to
//    this placeholder and to no empty leaf at all, although nothing but empty
//    space lies beyond it. Nothing sealed the cell, so it is void. It is the
//    only way a leaf is solid without a brush list: LeafNode sets CONTENTS_SOLID
//    only from a brush in the list, and FillOutside and PruneNodes run after
//    this test.
static bool IsVoidLeaf( const node_t *pLeaf )
{
	if ( pLeaf->contents & CONTENTS_SOLID )
		return !pLeaf->brushlist;
	return !pLeaf->occupied;
}

static void FindVoidFacing_r( const node_t *node )
{
	if ( node->planenum != PLANENUM_LEAF )
	{
		FindVoidFacing_r( node->children[0] );
		FindVoidFacing_r( node->children[1] );
		return;
	}

	// only a solid leaf holds the brushes in question
	if ( !node->brushlist || !( node->contents & CONTENTS_SOLID ) )
		return;

	bool bFacesVoid = false;
	for ( const portal_t *p = node->portals; p && !bFacesVoid; )
	{
		int s = ( p->nodes[1] == node );
		bFacesVoid = IsVoidLeaf( p->nodes[!s] );
		p = p->next[s];
	}
	if ( !bFacesVoid )
		return;

	for ( const bspbrush_t *b = node->brushlist; b; b = b->next )
	{
		int index = (int)( b->original - g_MainMap->mapbrushes );
		if ( index >= 0 && index < s_FacesVoid.Count() )
			s_FacesVoid[index] = 1;
	}
}

void DetailReport_FindVoidFacing( tree_t *tree )
{
	double flStart = Plat_FloatTime();
	s_FacesVoid.SetCount( g_MainMap->nummapbrushes );
	for ( int i = 0; i < s_FacesVoid.Count(); i++ )
		s_FacesVoid[i] = 0;

	FindVoidFacing_r( tree->headnode );
	s_bHaveVoidInfo = true;
	s_flVoidSeconds = Plat_FloatTime() - flStart;
}

//-----------------------------------------------------------------------------
// Cost, counted on the final tree and the final portal list
//-----------------------------------------------------------------------------
static void CountNodes_r( const node_t *node, CUtlVector<int> &nodes )
{
	// PruneNodes turns a node into a leaf without detaching its children
	if ( node->planenum == PLANENUM_LEAF )
		return;
	if ( node->sideBrush >= 0 && node->sideBrush < nodes.Count() )
		nodes[node->sideBrush]++;
	CountNodes_r( node->children[0], nodes );
	CountNodes_r( node->children[1], nodes );
}

struct DetailRow_t
{
	int brush;   // index into g_MainMap->mapbrushes
	int hammerId;
	int portals;
	int nodes;
};

// portals first, then nodes; the Hammer id breaks a tie so the order never
// depends on anything but the map
static bool RowBefore( const DetailRow_t &a, const DetailRow_t &b )
{
	if ( a.portals != b.portals )
		return a.portals > b.portals;
	if ( a.nodes != b.nodes )
		return a.nodes > b.nodes;
	if ( a.hammerId != b.hammerId )
		return a.hammerId < b.hammerId;
	return a.brush < b.brush;
}

void DetailReport_Emit( node_t *headnode, const CUtlVector<portal_t *> &visPortals )
{
	double flStart = Plat_FloatTime();

	if ( !s_bHaveVoidInfo )
	{
		Warning( "-reportdetail: the void test did not run, no report written\n" );
		return;
	}

	const int nBrushes = g_MainMap->nummapbrushes;
	CUtlVector<int> nodes, portals;
	nodes.SetCount( nBrushes );
	portals.SetCount( nBrushes );
	for ( int i = 0; i < nBrushes; i++ )
		nodes[i] = portals[i] = 0;

	CountNodes_r( headnode, nodes );
	for ( int i = 0; i < visPortals.Count(); i++ )
	{
		const node_t *pOn = visPortals[i]->onnode;
		if ( pOn && pOn->sideBrush >= 0 && pOn->sideBrush < nBrushes )
			portals[pOn->sideBrush]++;
	}

	int nStructural = 0, nVoidFacing = 0;
	std::vector<DetailRow_t> rows;
	for ( int i = 0; i < nBrushes; i++ )
	{
		const mapbrush_t *mb = &g_MainMap->mapbrushes[i];
		if ( !IsCandidate( mb ) )
			continue;
		nStructural++;
		if ( s_FacesVoid[i] )
		{
			nVoidFacing++;
			continue;
		}
		if ( nodes[i] == 0 )
			continue; // splits nothing: the tree has no cost to charge
		DetailRow_t row = { i, mb->id, portals[i], nodes[i] };
		rows.push_back( row );
	}
	std::sort( rows.begin(), rows.end(), RowBefore );

	int nCandidatePortals = 0;
	for ( size_t r = 0; r < rows.size(); r++ )
		nCandidatePortals += rows[r].portals;

	char filename[1024];
	sprintf( filename, "%s.detail.txt", source );
	FILE *pf = fopen( filename, "w" );
	if ( !pf )
		Warning( "-reportdetail: cannot open %s for writing\n", filename );
	else
		fprintf( pf, "rank\tbrush_id\tportals\tnodes\tsize_x\tsize_y\tsize_z\tcenter_x\tcenter_y\tcenter_z\tmaterial\n" );

	Msg( "\nDetail report: structural brushes that do not face the void, by visibility\n"
		 "portals charged to their planes. Advice, not proof: docs/vbsp_detail_report.md.\n" );
	int nShown = ( (int)rows.size() < g_nReportDetailRows ) ? (int)rows.size() : g_nReportDetailRows;
	if ( nShown > 0 )
		Msg( "%5s %9s %8s %6s  %-21s %-24s %s\n", "rank", "brush id", "portals", "nodes", "size x y z",
			 "centre x y z", "material" );

	for ( size_t r = 0; r < rows.size(); r++ )
	{
		const mapbrush_t *mb = &g_MainMap->mapbrushes[rows[r].brush];
		Vector size = mb->maxs - mb->mins;
		Vector centre = ( mb->maxs + mb->mins ) * 0.5f;
		char pMaterial[TEXTURE_NAME_LENGTH];
		FirstSideMaterial( mb, pMaterial, sizeof( pMaterial ) );

		if ( (int)r < nShown )
		{
			char szSize[64], szCentre[64];
			sprintf( szSize, "%.0f x %.0f x %.0f", size.x, size.y, size.z );
			sprintf( szCentre, "%.0f %.0f %.0f", centre.x, centre.y, centre.z );
			Msg( "%5d %9d %8d %6d  %-21s %-24s %s\n", (int)r + 1, rows[r].hammerId, rows[r].portals, rows[r].nodes,
				 szSize, szCentre, pMaterial );
		}
		if ( pf )
			fprintf( pf, "%d\t%d\t%d\t%d\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%s\n", (int)r + 1, rows[r].hammerId,
					 rows[r].portals, rows[r].nodes, size.x, size.y, size.z, centre.x, centre.y, centre.z, pMaterial );
	}
	if ( pf )
		fclose( pf );

	double flPercent = visPortals.Count() ? 100.0 * nCandidatePortals / visPortals.Count() : 0.0;
	Msg( "%d candidate brushes of %d structural world brushes (%d face the void); they carry %d of the %d "
		 "visportals (%.1f%%).\n",
		 (int)rows.size(), nStructural, nVoidFacing, nCandidatePortals, visPortals.Count(), flPercent );
	if ( (int)rows.size() > nShown )
		Msg( "Showing %d of %d; ", nShown, (int)rows.size() );
	double flSeconds = s_flVoidSeconds + Plat_FloatTime() - flStart;
	if ( pf )
		Msg( "the full list is in %s (report pass %.2f s)\n", filename, flSeconds );
	else
		Msg( "(report pass %.2f s)\n", flSeconds );
}
