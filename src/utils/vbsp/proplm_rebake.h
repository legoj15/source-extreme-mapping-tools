//========= proplm: "rebake" route (VBSP side) ================================
//
// A static prop opted in with keyvalue proplm "rebake" (or every static prop
// with generatelightmaps 1 under -proplmrebake) whose model's LOD0 UV islands
// overlap each other gets a copy of its model in the BSP pak whose one UV set
// does not overlap: the artist's islands moved rigidly into a new layout, the
// textures re-rasterized into that layout, the static prop pointed at the copy.
// Stock VRAD then writes a correct texelslighting_N.ppl and a stock engine
// renders it with no addon. Design and limits: docs/proplm.md, "rebake route".
//
// Failures never stop the compile: the prop keeps its original model and VBSP
// prints one Warning per prop naming the model, the prop's origin and why.
//=============================================================================
#ifndef PROPLM_REBAKE_H
#define PROPLM_REBAKE_H
#pragma once

struct entity_t;

// -proplmrebake: rebake every static prop with generatelightmaps 1 whose
// model's islands overlap each other.
extern bool g_bProplmRebakeAll;

// Finds the static props that ask for a rebake and the smallest lightmap
// size of each model. Call once from EmitStaticProps before the props are
// emitted.
void ProplmRebake_Prepare();

// The model name the static prop lump's dictionary should use for this prop
// entity: the rebaked copy, or NULL to keep the entity's own model. The
// model is rebaked (files written to the pak) on the first call for it; a
// failure prints one Warning per prop. Call only for props VBSP keeps.
const char *ProplmRebake_ModelForProp(entity_t *pEnt);

// Prints the map total. Call once after the props are emitted.
void ProplmRebake_Summary();

#endif // PROPLM_REBAKE_H
