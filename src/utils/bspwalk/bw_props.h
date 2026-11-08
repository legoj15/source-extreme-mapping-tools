//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: prop collision through vphysics (the same path VRAD uses; no
//          hand-parsed PHY): static props from the 'sprp' game lump and
//          entity props (prop_physics*, prop_dynamic*) from the entity lump.
//
//=============================================================================//

#ifndef BW_PROPS_H
#define BW_PROPS_H
#pragma once

#include "bw_world.h"

// Loads vphysics. Error()s out if it cannot: missing props would under-reach.
void Props_Init();

void CollectStaticProps( CollisionWorld &world );
void CollectEntityProps( CollisionWorld &world );

// A static prop of the 'sprp' lump with a collision shape, whatever its solid flag:
// CollectStaticProps leaves the SOLID_NONE ones out of the world (the player walks
// through them), but they are still drawn, and a camera inside one still sees its
// inside. For tests that must not change the world (bw_cubemap.cpp).
struct StaticPropInstance
{
	int index;				// in the lump
	const char *model;		// model name, valid for the run (the prop model cache's)
	bool hullBox;			// the shape is the .mdl hull box: no usable .phy, or solid BBOX
	bool hullFallback;		// hullBox because there is no usable .phy (not solid BBOX, whose box is the engine's collision)
	DTransform xf;			// model to world
	dvec3 mins, maxs;		// box of the shape in world space
	const void *handle;		// the cached model
};
// Every such prop, loading their models through the cache. 'unloadable' counts
// the ones with neither a usable .phy nor a readable .mdl, which are left out.
// Main thread only (it loads models).
void CollectStaticPropInstances( std::vector<StaticPropInstance> &out, int &unloadable );
// The prop's shape in world space: its .phy convexes, or the one hull box. Any
// thread once the instances are collected.
void StaticPropShapes( const StaticPropInstance &inst, std::vector<ConvexShape> &out );

// -checkprops: for every prop model used, compare the AABB of its collision
// vertices (model space) with the .mdl header hull. Prints one line per
// model; returns the number of models whose boxes differ by more than 1u.
int CheckPropHulls();

// Models the world's collision uses (not those only the cubemap check loaded).
int Props_ModelCount();

#endif // BW_PROPS_H
