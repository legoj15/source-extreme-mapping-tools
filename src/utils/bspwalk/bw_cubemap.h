//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: which env_cubemap samples (lump 42) may be viewpoint eyes (#82).
//
//          A cubemap is rendered from its origin, so the eye volume (bw_eyes.cpp)
//          adds each one as a camera. One whose camera sits inside a brush has no
//          cluster: the engine draws every leaf from there, which turns the ray
//          tier off for the whole map. The map's author cannot see such a
//          cubemap's picture either (it is a capture from inside a wall), so it is
//          dropped instead of being obeyed:
//
//          1. In a brush. The origin's leaf has no cluster, or the near cube of the
//             six 90 degree frustums (origin +- near_z: their near planes form
//             exactly that cube) overlaps the interior of a world brush with
//             CONTENTS_SOLID (func_detail included). An exact separating axis test
//             against the brush; touching a face is not inside.
//          2. Sees a toolsnodraw face. Rays through the pixel centres of the six cube
//             faces; the nearest opaque solid world brush entry is through a side whose
//             material is TOOLS/TOOLSNODRAW and that is not buried (open space just
//             outside it). The engine draws nothing there, so the capture shows the
//             void behind it.
//
//          A cubemap whose near cube overlaps a static prop's collision (the .phy
//          convexes, or the .mdl hull box without one) is only reported: the capture
//          is of the inside of the model, but the camera is still a legal eye.
//
//=============================================================================//

#ifndef BW_CUBEMAP_H
#define BW_CUBEMAP_H
#pragma once

#include <string>
#include <vector>
#include "bw_math.h"

struct CubemapOptions
{
	double nearZ;		// -cubemapnearz: the client's near plane (VIEW_NEARZ, 7u); 0 tests the origin only
	int raysPerFace;	// -cubemaprays: rays per cube face edge (R x R per face, 6 faces)
	int nodrawMin;		// -cubemapnodraw: nodraw rays that disqualify a cubemap
	bool keepAll;		// -keepcubemaps: no disqualifying (rules 1 and 2 off; props are still checked)

	CubemapOptions() : nearZ( 7.0 ), raysPerFace( 64 ), nodrawMin( 1 ), keepAll( false ) {}
};

// Bounds of -cubemaprays (bspwalk.cpp clamps to them).
#define CUBEMAP_MIN_RAYS	4
#define CUBEMAP_MAX_RAYS	512

enum CubemapStatus
{
	CUBEMAP_OK = 0,		// an eye
	CUBEMAP_SOLID,		// camera inside a brush
	CUBEMAP_NODRAW		// sees a toolsnodraw face
};
const char *CubemapStatusName( int status );

struct CubemapProp
{
	std::string model;
	int staticIndex;	// in the static prop lump
	bool hullFallback;	// the shape is the .mdl hull box standing in for a missing .phy (crude: the warning may be false)
};

struct CubemapInfo
{
	int index;			// in lump 42
	dvec3 origin;
	int status;			// CubemapStatus
	const char *reason;	// "solid_leaf", "brush", "nodraw" or ""
	int brush;			// the dbrush that overlaps the near cube, or took the first nodraw hit; else -1
	int rays;			// rays cast (0: rule 2 did not run)
	int nodrawRays;		// of them, nodraw hits
	bool hasNodrawPoint;
	dvec3 nodrawPoint;	// the first nodraw hit
	std::vector<CubemapProp> props;	// static props whose collision the near cube overlaps (any status)

	CubemapInfo() : index( 0 ), status( CUBEMAP_OK ), reason( "" ), brush( -1 ), rays( 0 ), nodrawRays( 0 ),
					hasNodrawPoint( false ) {}
};

struct CubemapReport
{
	CubemapOptions opts;
	std::vector<CubemapInfo> list;
	int disqualifiedSolid, disqualifiedNodraw;
	int propWarnings;			// kept cubemaps (status ok) whose near cube overlaps a static prop; the ones the console warns about
	int propsUnloadable;		// static props with neither a usable .phy nor a readable .mdl: not checked
	int brushesUnbuilt;			// world solid brushes that did not close into a convex: not tested
	double tCheck;

	CubemapReport() : disqualifiedSolid( 0 ), disqualifiedNodraw( 0 ), propWarnings( 0 ), propsUnloadable( 0 ),
					  brushesUnbuilt( 0 ), tCheck( 0 ) {}
};

// Judges every cubemap of the loaded BSP (g_CubemapSamples) and prints a Warning for each one
// dropped, one for each static prop a kept one sits in, and a summary line. Needs the BSP and
// Props_Init. Main thread; the work runs on the worker threads.
void CheckCubemaps( const CubemapOptions &opts, CubemapReport &out );

#endif // BW_CUBEMAP_H
