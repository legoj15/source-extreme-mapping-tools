//========= proplm: second-UV prop lightmaps (VBSP side) =====================
//
// Converts opted-in prop_static / static_prop entities (keyvalue proplm "1")
// into prop_dynamic entities and records a per-model UV2 atlas in the 'plm2'
// game lump. Format: src/public/proplm.h. Design: docs/proplm.md.
//=============================================================================
#ifndef PROPLM_UV2_H
#define PROPLM_UV2_H
#pragma once

struct entity_t;

// Called from EmitStaticProps for an entity with proplm "1", before the normal
// static prop path. Builds (or reuses) the model's UV2 and, on success,
// rewrites the entity in place into a prop_dynamic carrying proplm, proplm_id,
// proplm_lm and proplm_res; the caller must then keep the entity and must NOT
// add it to the static prop lump. Returns false after printing a Warning when
// the model cannot be processed; the entity is then untouched and the caller
// emits it as an ordinary static prop.
bool Proplm_ConvertStaticProp(entity_t *pEnt);

// Writes the 'plm2' game lump for every model processed so far. Always replaces
// a 'plm2' lump left by an earlier compile (-onlyents loads the old BSP); removes
// it when no model was processed. The old compile's pak lightmaps under
// materials/maps/<map>/proplm/ that no converted prop's id names any more are
// removed with it (#116). Call once, from EmitStaticProps, after the entity
// loop.
void Proplm_WriteGameLump();

// -onlyprops guard: Error() when a static prop in the loaded VMF entities has
// proplm "1", or the loaded BSP still holds a 'plm2' lump. -onlyprops does not
// rewrite the entity lump, so neither case can come out right. Call after the
// BSP and the VMF are loaded, before EmitStaticProps.
void Proplm_RejectOnlyProps();

#endif // PROPLM_UV2_H
