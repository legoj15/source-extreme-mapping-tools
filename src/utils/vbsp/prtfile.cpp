//========= Copyright Valve Corporation, All rights reserved. ============//
//
// Purpose: 
//
// $NoKeywords: $
//
//=============================================================================//

#include "vbsp.h"
#include "collisionutils.h"
/*
==============================================================================

PORTAL FILE GENERATION

Save out name.prt for qvis to read
==============================================================================
*/


#define	PORTALFILE	"PRT1"

struct cluster_portals_t
{
	CUtlVector<portal_t *>	portals;
};

int		num_visclusters;				// clusters the player can be in
int		num_visportals;

int g_SkyCluster = -1;

void WriteFloat (FILE *f, vec_t v)
{
	if ( fabs(v - RoundInt(v)) < 0.001 )
		fprintf (f,"%i ",(int)RoundInt(v));
	else
		fprintf (f,"%f ",v);
}


/*
=================
WritePortalFile_r
=================
*/
void WritePortalFile(FILE *pFile, const CUtlVector<cluster_portals_t> &list)
{
	portal_t	*p;
	winding_t	*w;
	Vector		normal;
	vec_t		dist;

	for ( int clusterIndex = 0; clusterIndex < list.Count(); clusterIndex++ )
	{
		for ( int j = 0; j < list[clusterIndex].portals.Count(); j++ )
		{
			p = list[clusterIndex].portals[j];
			w = p->winding;
			// write out to the file
			
			// sometimes planes get turned around when they are very near
			// the changeover point between different axis.  interpret the
			// plane the same way vis will, and flip the side orders if needed
			// FIXME: is this still relevent?
			WindingPlane (w, normal, &dist);
			if ( DotProduct (p->plane.normal, normal) < 0.99 )
			{	// backwards...
				fprintf (pFile,"%i %i %i ",w->numpoints, p->nodes[1]->cluster, p->nodes[0]->cluster);
			}
			else
			{
				fprintf (pFile,"%i %i %i ",w->numpoints, p->nodes[0]->cluster, p->nodes[1]->cluster);
			}
			
			for (int i=0 ; i<w->numpoints ; i++)
			{
				fprintf (pFile,"(");
				WriteFloat (pFile, w->p[i][0]);
				WriteFloat (pFile, w->p[i][1]);
				WriteFloat (pFile, w->p[i][2]);
				fprintf (pFile,") ");
			}
			fprintf (pFile,"\n");
		}
	}
}

struct viscluster_t
{
	bspbrush_t *pBrushes;
	int clusterIndex;
	int leafCount;
	int leafStart;
};

static CUtlVector<viscluster_t> g_VisClusters;

// add to the list of brushes the merge leaves into single vis clusters
void AddVisCluster( entity_t *pFuncVisCluster )
{
	viscluster_t tmp;
	Vector clipMins, clipMaxs;
	clipMins[0] = clipMins[1] = clipMins[2] = MIN_COORD_INTEGER;
	clipMaxs[0] = clipMaxs[1] = clipMaxs[2] = MAX_COORD_INTEGER;

	// build the map brushes out into the minimum non-overlapping set of brushes
	bspbrush_t *pBSPBrush = MakeBspBrushList( pFuncVisCluster->firstbrush, pFuncVisCluster->firstbrush + pFuncVisCluster->numbrushes, 
		clipMins, clipMaxs, NO_DETAIL);
	tmp.pBrushes = ChopBrushes( pBSPBrush );

	// store the entry in the list
	tmp.clusterIndex = -1;
	tmp.leafCount = 0;
	tmp.leafStart = 0;

#if DEBUG_VISUALIZE_CLUSTERS
	int debug = atoi(ValueForKey(pFuncVisCluster,"debug"));
	if ( debug )
	{
		Msg("Debug vis cluster %d\n", g_VisClusters.Count() );
	}
#endif

	g_VisClusters.AddToTail( tmp );
	
	// clear out this entity so it won't get written to the bsp
	pFuncVisCluster->epairs = NULL;
	pFuncVisCluster->numbrushes = 0;
}

// returns the total overlapping volume of intersection between the node and the brush list
float VolumeOfIntersection( bspbrush_t *pBrushList, node_t *pNode )
{
	float volume = 0.0f;
	for ( bspbrush_t *pBrush = pBrushList; pBrush; pBrush = pBrush->next )
	{
		if ( IsBoxIntersectingBox( pNode->mins, pNode->maxs, pBrush->mins, pBrush->maxs ) )
		{
			bspbrush_t *pIntersect = IntersectBrush( pNode->volume, pBrush );
			if ( pIntersect )
			{
				volume += BrushVolume( pIntersect );
				FreeBrush( pIntersect );
			}
		}
	}

	return volume;
}

// Search for the forced cluster that overlaps this node the most (at least 10%
// of its volume). On a tie the higher-indexed one wins. Clusters never merge.
int GetVisCluster( node_t *pNode )
{
	float maxVolume = BrushVolume(pNode->volume) * 0.10f;		// needs to cover at least 10% of the volume to overlap
	int maxIndex = -1;
	// UNDONE: This could get slow
	for ( int i = g_VisClusters.Count(); --i >= 0; )
	{
		float volume = VolumeOfIntersection( g_VisClusters[i].pBrushes, pNode );
		if ( volume > maxVolume )
		{
			maxVolume = volume;
			maxIndex = i;
		}
	}
	return maxIndex;
}
/*
================
NumberLeafs_r
================
*/
void BuildVisLeafList_r (node_t *node, CUtlVector<node_t *> &leaves)
{
	if (node->planenum != PLANENUM_LEAF)
	{	// decision node
		node->cluster = -99;
		BuildVisLeafList_r (node->children[0], leaves);
		BuildVisLeafList_r (node->children[1], leaves);
		return;
	}
	
	if ( node->contents & CONTENTS_SOLID )
	{	// solid block, viewpoint never inside
		node->cluster = -1;
		return;
	}
	leaves.AddToTail(node);
}

// -autoviscluster (#100). The leaves under one tree node fill that node's convex
// volume. A subtree whose empty leaves share one area and contents value, with
// no hint split below it, no leaf in the 3D skybox or claimed by a func_viscluster
// and little solid in it, is a convex region of open space and can be one
// cluster. Some solid has to be allowed: every split plane is a brush side, so
// a subtree of only empty leaves practically never exists.
struct autostat_t
{
	double	emptyVolume, solidVolume;	// volume of the empty and of the solid leaves under the node
	int		area, contents;				// of the empty leaves, which all share them
	int		first, count;				// the empty leaves are [first, first+count) of the leaf list
	int		nodes;						// entries of this subtree in the list, its own included
	bool	uniform;					// the empty leaves may merge, share area and contents, no hint split
};

struct autoregion_t
{
	const CUtlVector<int>	*pHandCluster;	// func_viscluster of each empty leaf, -1 for none
	CUtlVector<int>			*pLeafRegion;	// out: region of each empty leaf, -1 for none
	CUtlVector<autostat_t>	stats;			// every node, in pre-order
	CUtlVector<bool>		hintPlane;		// by plane number: a hint side lies on it
	int		nextLeaf;		// leaf list index of the next empty leaf the walk meets
	int		numRegions;
	int		numLeaves;		// leaves in regions
};

static double AutoVisLeafVolume( node_t *node )
{
	if ( node->volume )
		return BrushVolume( node->volume );

	// leaves made for a grid block with no brush have none
	return (double)(node->maxs[0] - node->mins[0]) * (node->maxs[1] - node->mins[1]) * (node->maxs[2] - node->mins[2]);
}

// Fills ctx.stats for the subtree under node, in pre-order, the leaves in the order BuildVisLeafList_r lists them.
static void AutoVisStats_r( node_t *node, autoregion_t &ctx )
{
	int self = ctx.stats.AddToTail();
	autostat_t s;
	s.emptyVolume = s.solidVolume = 0.0;
	s.area = s.contents = 0;
	s.first = ctx.nextLeaf;
	s.count = 0;
	s.uniform = true;

	if ( node->planenum == PLANENUM_LEAF )
	{
		if ( node->contents & CONTENTS_SOLID )
		{
			s.solidVolume = AutoVisLeafVolume( node );
		}
		else
		{
			int index = ctx.nextLeaf++;
			s.emptyVolume = AutoVisLeafVolume( node );
			s.area = node->area;
			s.contents = node->contents;
			s.count = 1;
			s.uniform = (*ctx.pHandCluster)[index] < 0 && !Is3DSkyboxArea( node->area );
		}
	}
	else
	{
		AutoVisStats_r( node->children[0], ctx );
		AutoVisStats_r( node->children[1], ctx );
		const autostat_t &c0 = ctx.stats[self + 1];
		const autostat_t &c1 = ctx.stats[self + 1 + c0.nodes];

		s.emptyVolume = c0.emptyVolume + c1.emptyVolume;
		s.solidVolume = c0.solidVolume + c1.solidVolume;
		s.count = c0.count + c1.count;
		const autostat_t &withLeaves = c0.count ? c0 : c1;
		s.area = withLeaves.area;
		s.contents = withLeaves.contents;
		s.uniform = c0.uniform && c1.uniform;
		if ( c0.count && c1.count && ( c0.area != c1.area || c0.contents != c1.contents ) )
			s.uniform = false;

		// a hint split must stay a cluster boundary
		if ( node->planenum < ctx.hintPlane.Count() && ctx.hintPlane[node->planenum] )
			s.uniform = false;
	}

	s.nodes = ctx.stats.Count() - self;
	ctx.stats[self] = s;
}

// Top down: the first node on each path whose subtree can be one region becomes
// one, so the regions are the maximal ones. A region is the leaves under a node
// whose empty leaves are uniform and number at least two, whose bounding box
// is within the size limit and whose solid volume is at most the allowed
// fraction of its volume.
static void AutoVisPick_r( node_t *node, autoregion_t &ctx, int self )
{
	if ( node->planenum == PLANENUM_LEAF )
		return;

	const autostat_t &s = ctx.stats[self];
	bool region = s.uniform && s.count >= 2;
	if ( region && g_nAutoVisClusterMax > 0 )
	{
		for ( int i = 0; i < 3; i++ )
		{
			if ( node->maxs[i] - node->mins[i] > g_nAutoVisClusterMax )
				region = false;
		}
	}
	if ( region && s.solidVolume > g_flAutoVisClusterSolid * ( s.emptyVolume + s.solidVolume ) )
		region = false;

	if ( region )
	{
		for ( int i = s.first; i < s.first + s.count; i++ )
			(*ctx.pLeafRegion)[i] = ctx.numRegions;
		ctx.numRegions++;
		ctx.numLeaves += s.count;
		return;
	}

	AutoVisPick_r( node->children[0], ctx, self + 1 );
	AutoVisPick_r( node->children[1], ctx, self + 1 + ctx.stats[self + 1].nodes );
}

// Finds the regions of the tree: leafRegion gets, for each leaf of the leaf list,
// the number of its region or -1. handCluster is each leaf's func_viscluster.
static void AutoVisRegions( node_t *headnode, const CUtlVector<int> &handCluster, CUtlVector<int> &leafRegion, int &numRegions )
{
	autoregion_t ctx;
	ctx.pHandCluster = &handCluster;
	ctx.pLeafRegion = &leafRegion;
	ctx.nextLeaf = 0;
	ctx.numRegions = 0;
	ctx.numLeaves = 0;

	leafRegion.SetCount( handCluster.Count() );
	leafRegion.FillWithValue( -1 );

	// node_t::side cannot say which nodes a hint side made: BuildTree_r frees the
	// brush it points into. So a hint split is a node on the plane of a hint side.
	ctx.hintPlane.SetCount( g_MainMap->nummapplanes );
	ctx.hintPlane.FillWithValue( false );
	for ( int i = 0; i < g_MainMap->nummapbrushes; i++ )
	{
		const mapbrush_t &brush = g_MainMap->mapbrushes[i];
		for ( int j = 0; j < brush.numsides; j++ )
		{
			const side_t &side = brush.original_sides[j];
			if ( ( side.surf & SURF_HINT ) && ( side.planenum & ~1 ) < ctx.hintPlane.Count() )
				ctx.hintPlane[side.planenum & ~1] = true;
		}
	}

	AutoVisStats_r( headnode, ctx );
	if ( ctx.nextLeaf != handCluster.Count() )
		Error( "autoviscluster: walked %d empty leaves, the leaf list has %d\n", ctx.nextLeaf, handCluster.Count() );
	AutoVisPick_r( headnode, ctx, 0 );

	numRegions = ctx.numRegions;
	if ( g_nAutoVisClusterMax > 0 )
		Msg( "autoviscluster: %d regions merged %d leaves (cap %d, solid %g)\n", ctx.numRegions, ctx.numLeaves, g_nAutoVisClusterMax, g_flAutoVisClusterSolid );
	else
		Msg( "autoviscluster: %d regions merged %d leaves (no cap, solid %g)\n", ctx.numRegions, ctx.numLeaves, g_flAutoVisClusterSolid );
}

// Give each leaf in the list of empty leaves a vis cluster number
// some are within func_viscluster volumes and will be merged together
// every other leaf gets its own unique number, except those in an
// -autoviscluster region (leafRegion, NULL when off), which share one
// handCluster is the func_viscluster of each leaf (GetVisCluster is slow)
void NumberLeafs( const CUtlVector<node_t *> &leaves, const CUtlVector<int> &handCluster, const CUtlVector<int> *leafRegion, int numRegions )
{
	CUtlVector<int> regionCluster;
	regionCluster.SetCount( numRegions );
	regionCluster.FillWithValue( -1 );

	for ( int i = 0; i < leaves.Count(); i++ )
	{
		node_t *node = leaves[i];
		int visCluster = handCluster[i];
		if ( visCluster < 0 && leafRegion && (*leafRegion)[i] >= 0 )
		{
			int &cluster = regionCluster[(*leafRegion)[i]];
			if ( cluster < 0 )
			{
				cluster = num_visclusters;
				num_visclusters++;
			}
			node->cluster = cluster;
		}
		else if ( visCluster >= 0 )
		{
			if ( g_VisClusters[visCluster].clusterIndex < 0 )
			{
				g_VisClusters[visCluster].clusterIndex = num_visclusters;
				num_visclusters++;
			}
			g_VisClusters[visCluster].leafCount++;
			node->cluster = g_VisClusters[visCluster].clusterIndex;
		}
		else
		{
			if ( !g_bSkyVis && Is3DSkyboxArea( node->area ) )
			{
				if ( g_SkyCluster < 0 )
				{
					// allocate a cluster for the sky
					g_SkyCluster = num_visclusters;
					num_visclusters++;
				}
				node->cluster = g_SkyCluster;
			}
			else
			{
				node->cluster = num_visclusters;
				num_visclusters++;
			}
		}
	}

#if DEBUG_VISUALIZE_CLUSTERS
	for ( int i = 0; i < g_VisClusters.Count(); i++ )
	{
		char name[256];
		sprintf(name, "u:\\main\\game\\ep2\\maps\\vis_%02d.gl", i );
		FileHandle_t fp = g_pFileSystem->Open( name, "w" );
		Msg("Writing %s\n", name );
		for ( bspbrush_t *pBrush = g_VisClusters[i].pBrushes; pBrush; pBrush = pBrush->next )
		{
			for (int i =  0; i < pBrush->numsides; i++ )
				OutputWindingColor( pBrush->sides[i].winding, fp, 0, 255, 0 );
		}
		for ( int j = 0; j < leaves.Count(); j++ )
		{
			if ( leaves[j]->cluster == g_VisClusters[i].clusterIndex )
			{
				bspbrush_t *pBrush = leaves[j]->volume;
				for (int k = 0; k < pBrush->numsides; k++ )
					OutputWindingColor( pBrush->sides[k].winding, fp, 64 + (j&31), 64, 64 - (j&31) );
			}
		}
		g_pFileSystem->Close(fp);
	}
#endif
}

// build a list of all vis portals that connect clusters
int BuildPortalList( CUtlVector<cluster_portals_t> &portalList, const CUtlVector<node_t *> &leaves )
{
	int portalCount = 0;
	for ( int i = 0; i < leaves.Count(); i++ )
	{
		node_t *node = leaves[i];
		// count the portals
		for (portal_t *p = node->portals ; p ; )
		{
			if (p->nodes[0] == node)		// only write out from first leaf
			{
				if ( p->nodes[0]->cluster != p->nodes[1]->cluster )
				{
					if (Portal_VisFlood (p))
					{
						portalCount++;
						portalList[node->cluster].portals.AddToTail(p);
					}
				}
				p = p->next[0];
			}
			else
				p = p->next[1];		
		}
	}
	return portalCount;
}


/*
================
CreateVisPortals_r
================
*/
void CreateVisPortals_r (node_t *node)
{
	// stop as soon as we get to a leaf
	if (node->planenum == PLANENUM_LEAF )
		return;

	MakeNodePortal (node);
	SplitNodePortals (node);

	CreateVisPortals_r (node->children[0]);
	CreateVisPortals_r (node->children[1]);
}

int		clusterleaf;
void SaveClusters_r (node_t *node)
{
	if (node->planenum == PLANENUM_LEAF)
	{
		dleafs[clusterleaf++].cluster = node->cluster;
		return;
	}
	SaveClusters_r (node->children[0]);
	SaveClusters_r (node->children[1]);
}

/*
================
WritePortalFile
================
*/
void WritePortalFile (tree_t *tree)
{
	char	filename[1024];
	node_t *headnode;
	int start = Plat_FloatTime();

	qprintf ("--- WritePortalFile ---\n");

	sprintf (filename, "%s.prt", source);
	Msg ("writing %s...", filename);

	headnode = tree->headnode;

	FreeTreePortals_r (headnode);
	MakeHeadnodePortals (tree);

	CreateVisPortals_r (headnode);

// set the cluster field in every leaf and count the total number of portals
	num_visclusters = 0;
	Msg("Building visibility clusters...\n");
	CUtlVector<node_t *> leaves;
	BuildVisLeafList_r( headnode, leaves );

	CUtlVector<int> handCluster;
	for ( int i = 0; i < leaves.Count(); i++ )
		handCluster.AddToTail( GetVisCluster( leaves[i] ) );

	CUtlVector<int> leafRegion;
	int numRegions = 0;
	if ( g_bAutoVisCluster )
		AutoVisRegions( headnode, handCluster, leafRegion, numRegions );

	NumberLeafs (leaves, handCluster, g_bAutoVisCluster ? &leafRegion : NULL, numRegions);
	CUtlVector<cluster_portals_t> portalList;
	portalList.SetCount( num_visclusters );
	num_visportals = BuildPortalList( portalList, leaves );
// write the file
	FILE *pf = fopen (filename, "w");
	if (!pf)
		Error ("Error opening %s", filename);
		
	fprintf (pf, "%s\n", PORTALFILE);
	fprintf (pf, "%i\n", num_visclusters);
	fprintf (pf, "%i\n", num_visportals);

	qprintf ("%5i visclusters\n", num_visclusters);
	qprintf ("%5i visportals\n", num_visportals);

	WritePortalFile(pf, portalList);

	fclose (pf);

	// we need to store the clusters out now because ordering
	// issues made us do this after writebsp...
	clusterleaf = 1;
	SaveClusters_r (headnode);

	Msg("done (%d)\n", (int)(Plat_FloatTime() - start) );
}

