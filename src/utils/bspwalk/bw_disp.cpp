//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: displacement collision triangles, built with CCoreDispInfo exactly
//          as CVRadDispMgr::DispBuilderInit / UnserializeDisps do
//          (vraddisps.cpp), then read back as the full-power triangle grid the
//          engine's CDispCollTree uses (dispcoll_common.cpp).
//
//=============================================================================//

#include "cmdlib.h"
#include "bsplib.h"
#include "bspflags.h"
#include "builddisp.h"
#include "bw_disp.h"

// Mirrors CVRadDispMgr::DispBuilderInit (that one is a private member).
static void DispBuilderInit( CCoreDispInfo *pBuilderDisp, dface_t *pFace, int ndxFace )
{
	ddispinfo_t *pDisp = &g_dispinfo[pFace->dispinfo];

	CCoreDispSurface *pSurf = pBuilderDisp->GetSurface();
	pSurf->SetPointCount( 4 );
	pSurf->SetHandle( ndxFace );
	pSurf->SetContents( pDisp->contents );

	for ( int ndxPt = 0; ndxPt < 4; ndxPt++ )
	{
		int eIndex = dsurfedges[pFace->firstedge + ndxPt];
		if ( eIndex < 0 )
			pSurf->SetPoint( ndxPt, dvertexes[dedges[-eIndex].v[1]].point );
		else
			pSurf->SetPoint( ndxPt, dvertexes[dedges[eIndex].v[0]].point );
	}

	Vector vFaceNormal;
	pSurf->GetNormal( vFaceNormal );
	for ( int ndxPt = 0; ndxPt < 4; ndxPt++ )
		pSurf->SetPointNormal( ndxPt, vFaceNormal );

	pSurf->SetPointStart( pDisp->startPosition );
	pSurf->FindSurfPointStartIndex();
	pSurf->AdjustSurfPointData();

	const texinfo_t &tex = texinfo[pFace->texinfo];
	Vector vecU( tex.lightmapVecsLuxelsPerWorldUnits[0][0], tex.lightmapVecsLuxelsPerWorldUnits[0][1],
				 tex.lightmapVecsLuxelsPerWorldUnits[0][2] );
	Vector vecV( tex.lightmapVecsLuxelsPerWorldUnits[1][0], tex.lightmapVecsLuxelsPerWorldUnits[1][1],
				 tex.lightmapVecsLuxelsPerWorldUnits[1][2] );
	float lenU = VectorLength( vecU );
	if ( lenU > 1e-6f )
		pSurf->CalcLuxelCoords( static_cast<int>( 1.0f / lenU ), false, vecU, vecV );

	pBuilderDisp->SetNeighborData( pDisp->m_EdgeNeighbors, pDisp->m_CornerNeighbors );

	CDispVert *pVerts = &g_DispVerts[pDisp->m_iDispVertStart];
	CDispTri *pTris = &g_DispTris[pDisp->m_iDispTriStart];
	pBuilderDisp->InitDispInfo( pDisp->power, pDisp->minTess, pDisp->smoothingAngle, pVerts, pTris );
}

// Creates every displacement's builder (CCoreDispInfo, full power) and the
// displacement index -> face map. The caller deletes the builders.
static void CreateDispBuilders( CUtlVector<CCoreDispInfo *> &builders, CUtlVector<int> &dispFace )
{
	int numDisps = g_dispinfo.Count();
	builders.SetCount( numDisps );
	for ( int i = 0; i < numDisps; i++ )
	{
		builders[i] = new CCoreDispInfo;
		builders[i]->SetListIndex( i );
	}
	for ( int i = 0; i < numDisps; i++ )
		builders[i]->SetDispUtilsHelperInfo( builders.Base(), numDisps );

	// displacement index -> face
	dispFace.SetCount( numDisps );
	for ( int i = 0; i < numDisps; i++ )
		dispFace[i] = -1;
	for ( int f = 0; f < numfaces; f++ )
	{
		if ( dfaces[f].dispinfo >= 0 && dfaces[f].dispinfo < numDisps )
		{
			dispFace[dfaces[f].dispinfo] = f;
			DispBuilderInit( builders[dfaces[f].dispinfo], &dfaces[f], f );
		}
	}
	for ( int d = 0; d < numDisps; d++ )
	{
		if ( dispFace[d] >= 0 )
			builders[d]->Create();
	}
}

void DispRenderTriangles( std::vector< std::vector<dvec3> > &tris, std::vector<int> &dispFace )
{
	int numDisps = g_dispinfo.Count();
	tris.assign( numDisps, std::vector<dvec3>() );
	dispFace.assign( numDisps, -1 );
	if ( !numDisps )
		return;
	CUtlVector<CCoreDispInfo *> builders;
	CUtlVector<int> faceOf;
	CreateDispBuilders( builders, faceOf );
	for ( int d = 0; d < numDisps; d++ )
	{
		dispFace[d] = faceOf[d];
		if ( faceOf[d] < 0 )
			continue;
		CCoreDispInfo *pDisp = builders[d];
		int nTris = pDisp->GetTriCount();
		for ( int t = 0; t < nTris; t++ )
		{
			unsigned short i0, i1, i2;
			pDisp->GetTriIndices( t, i0, i1, i2 );
			Vector a, b, c;
			pDisp->GetVert( i0, a );
			pDisp->GetVert( i1, b );
			pDisp->GetVert( i2, c );
			tris[d].push_back( dvec3( a ) );
			tris[d].push_back( dvec3( b ) );
			tris[d].push_back( dvec3( c ) );
		}
	}
	for ( int i = 0; i < numDisps; i++ )
		delete builders[i];
}

void CollectDisplacements( CollisionWorld &world )
{
	int numDisps = g_dispinfo.Count();
	if ( !numDisps )
		return;

	CUtlVector<CCoreDispInfo *> builders;
	CUtlVector<int> dispFace;
	CreateDispBuilders( builders, dispFace );

	ConvexShape tri;
	for ( int d = 0; d < numDisps; d++ )
	{
		if ( dispFace[d] < 0 )
			continue;
		CCoreDispInfo *pDisp = builders[d];

		// "No Hull Collision" (stored in minTess' high bit) or non-solid contents:
		// the player box passes through it.
		if ( pDisp->GetSurface()->GetFlags() & CCoreDispInfo::SURF_NOHULL_COLL ||
			 !( g_dispinfo[d].contents & MASK_PLAYERSOLID_BRUSHONLY ) )
		{
			world.stats.dispsNoHullCollision++;
			continue;
		}

		int nTris = pDisp->GetTriCount();
		for ( int t = 0; t < nTris; t++ )
		{
			unsigned short i0, i1, i2;
			pDisp->GetTriIndices( t, i0, i1, i2 );
			Vector a, b, c;
			pDisp->GetVert( i0, a );
			pDisp->GetVert( i1, b );
			pDisp->GetVert( i2, c );
			if ( !ConvexFromTriangle( dvec3( a ), dvec3( b ), dvec3( c ), tri ) )
			{
				world.stats.dispTrisDegenerate++;
				continue;
			}
			world.convexes.push_back( SourceConvex() );
			SourceConvex &sc = world.convexes.back();
			sc.shape = tri;
			sc.category = CAT_DISP;
			sc.sourceIndex = d;
			sc.entityIndex = -1;
			Q_snprintf( sc.label, sizeof( sc.label ), "disp%d_tri%d", d, t );
			sc.targetname[0] = 0;
		}
	}

	for ( int i = 0; i < numDisps; i++ )
		delete builders[i];
}
