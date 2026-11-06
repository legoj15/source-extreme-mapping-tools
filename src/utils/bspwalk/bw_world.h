//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: the player collision world. Collects every player-solid convex of
//          a loaded BSP, tags it with a category, and grows the ones that are
//          C-space obstacles by the stand and crouch boxes (Stage 1).
//
//=============================================================================//

#ifndef BW_WORLD_H
#define BW_WORLD_H
#pragma once

#include <string.h>
#include <vector>
#include "bw_grow.h"

enum ConvexCategory
{
	CAT_WORLD = 0,			// brushes of model 0 that block the player
	CAT_FUNC_BRUSH_SOLID,	// always-solid, never-toggled, unparented func_brush: treated as world
	CAT_TOPS_ONLY,			// other solid brush entities (movers): not grown, not an obstacle (#12 uses tops)
	CAT_STATIC_PROP,		// prop_static from the game lump: grown obstacle
	CAT_ENTITY_PROP,		// prop_physics* / prop_dynamic*: grown, standable only (never a side obstacle)
	CAT_DISP,				// displacement collision triangle: grown obstacle
	CAT_WATER,				// world brush with CONTENTS_WATER / SLIME: grown, its top is a swim surface (#13B)
	CAT_COUNT
};

const char *CategoryName( int category );
// Grown into C-space polytopes (everything except tops_only).
bool CategoryIsGrown( int category );
// A full side obstacle (world-like); entity props and tops_only are not.
bool CategoryIsObstacle( int category );

struct SourceConvex
{
	ConvexShape shape;			// world space
	int category;
	int sourceIndex;			// brush index, static prop index, or displacement index
	int entityIndex;			// index into entities[], -1 for world / static props / displacements
	char label[96];				// model name or classname
	char targetname[64];
};

// Skip and failure counters, all reported in the console summary and JSON.
struct CollectStats
{
	int brushesWrongContents;		// brush contents miss MASK_PLAYERSOLID_BRUSHONLY
	int brushesBuildFailed;			// planes did not close into a solid
	int brushEntitiesNonSolidClass;	// trigger_*, func_illusionary, ... (policy table)
	int brushEntitiesPassable;		// a passable / not-solid spawnflag
	int brushEntitiesNeverSolid;	// func_brush Solidity = Never
	int brushEntitiesNoSolidBrush;	// none of the model's brushes block the player
	int staticPropsNonSolid;		// SOLID_NONE
	int staticPropsNoCollision;		// the model failed to load (no .mdl and no usable .phy)
	int entityPropsNonSolid;		// prop_dynamic with solid 0
	int entityPropsNoCollision;
	int propsHullFallback;			// vphysics props without a usable .phy, kept as their .mdl hull box
	int dispsNoHullCollision;		// "no hull collision" displacement flag or contents miss the mask
	int dispTrisDegenerate;
	int convexesBuildFailed;		// prop convexes that were not closed solids, counted once per model
	int growFailed;					// sums that came out degenerate (should be 0)

	CollectStats() { memset( this, 0, sizeof( *this ) ); }
};

struct CollisionWorld
{
	std::vector<SourceConvex> convexes;
	// Index-aligned with convexes; an empty polytope (no planes) where the
	// category is not grown.
	std::vector<GrownPolytope> stand;
	std::vector<GrownPolytope> crouch;
	CollectStats stats;

	int CountCategory( int category ) const;
};

// Collection (bw_world.cpp)
// World brushes: player-solid ones as CAT_WORLD, water / slime ones as CAT_WATER.
void CollectWorldBrushes( CollisionWorld &world );
void CollectBrushEntities( CollisionWorld &world );
// Per entity: 1 when a brush entity of it is drawn opaque for the whole map
// (rendermode normal, renderamt 255, no renderfx, no EF_NODRAW, not start
// disabled or parented, no fade distance or dx/cpu/gpu level limits, and no
// output sends it an input that could hide, fade,
// kill, move or rekey it). Only such brush entities occlude sight (tier B).
void BrushEntitiesDrawnOpaque( std::vector<unsigned char> &out );
// Every brush of a BSP model, deduped.
void CollectModelBrushes( int model, std::vector<int> &brushes );
// A brush as a convex from its non-bevel sides. False if it does not close.
bool BuildBrushShape( int brushIndex, ConvexShape &out );
// A brush entity's model-to-world transform (origin, and angles unless the
// class policy turns them into a move direction).
struct entity_t;
void BrushEntityTransform( entity_t *ent, DTransform &xf );

// Grows every grown-category convex, both postures. Multithreaded.
void GrowWorld( CollisionWorld &world, const PlayerProfile &profile );

// Runs VerifyGrown on every polytope. Returns the number that fail, plus any
// that were never checked. If
// pszDumpPath is set, every failing convex (source points, planes) is written
// there as "# convex ..." blocks of "v x y z" / "p nx ny nz d" lines.
int VerifyWorld( const CollisionWorld &world, const PlayerProfile &profile, const char *pszDumpPath );

#endif // BW_WORLD_H
