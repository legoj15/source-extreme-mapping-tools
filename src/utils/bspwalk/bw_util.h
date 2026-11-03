//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: small helpers shared by the bspwalk modules.
//
//=============================================================================//

#ifndef BW_UTIL_H
#define BW_UTIL_H
#pragma once

#include "bw_grow.h"

struct entity_t;

// ValueForKey with const-correct arguments; "" when absent.
const char *EntValue( entity_t *ent, const char *pszKey );

// Moves a shape into world space: vertices transformed, normals rotated,
// edges and bounds rebuilt.
void TransformShape( ConvexShape &shape, const DTransform &xf );

#endif // BW_UTIL_H
