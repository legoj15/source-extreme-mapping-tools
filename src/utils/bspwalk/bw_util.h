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

// Entity names as CGlobalEntityList::FindEntityByName matches them: case-blind,
// a trailing '*' a wildcard. The next entity after 'after' (-1: from the start)
// whose targetname matches, or -1.
bool EntNameMatches( const char *pszPattern, const char *pszName );
int FindEntityByTargetname( const char *pszName, int after );
dvec3 EntOrigin( entity_t *ent );

// Moves a shape into world space: vertices transformed, normals rotated,
// edges and bounds rebuilt.
void TransformShape( ConvexShape &shape, const DTransform &xf );

// Bytes this process can still allocate without paging: the smaller of free
// RAM and the remaining commit limit (0 when the query fails: callers must
// treat 0 as unknown, not as plenty; the eye sample guard uses a fixed cap).
double FreeMemoryBytes();
// The process's peak working set so far, in bytes (0 when the query fails).
double PeakWorkingSetBytes();

#endif // BW_UTIL_H
