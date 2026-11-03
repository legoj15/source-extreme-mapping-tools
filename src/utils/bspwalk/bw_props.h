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

// -checkprops: for every prop model used, compare the AABB of its collision
// vertices (model space) with the .mdl header hull. Prints one line per
// model; returns the number of models whose boxes differ by more than 1u.
int CheckPropHulls();

int Props_ModelCount();

#endif // BW_PROPS_H
