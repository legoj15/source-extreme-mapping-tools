//========= proplm: "rebake" route (VBSP side) ================================
//
// A static prop opted in with keyvalue proplm "rebake" (or, under
// -proplightmap auto or vertex, every static prop with generatelightmaps 1)
// whose model's LOD0 UV islands overlap each other gets a copy of its model in
// the BSP pak whose one UV set does not overlap: the artist's islands moved
// rigidly into a new layout, the textures re-rasterized into that layout, the
// static prop pointed at the copy. Stock VRAD then writes a correct
// texelslighting_N.ppl and a stock engine renders it with no addon. Design and
// limits: docs/proplm.md, "rebake route".
//
// Props the rebake cannot serve get a route instead of a failure: one whose
// model could not be read or measured keeps per-texel lighting (VRAD reports
// its overlap); one whose overlap is known but whose rebake was refused
// (-proplightmap vertex, or over the automatic limits) falls back to vertex
// lighting, because a per-texel lightmap over overlapping UVs is corrupt.
// Failures never stop the compile: VBSP prints one Warning per prop naming
// the model, the prop's origin and why.
//=============================================================================
#ifndef PROPLM_REBAKE_H
#define PROPLM_REBAKE_H
#pragma once

struct entity_t;

// -proplightmap <auto|vertex|explicit>: how a static prop with
// generatelightmaps 1 whose model's UV islands overlap is lit.
enum ProplmLightmapMode_t {
  PROPLM_LIGHTMAP_AUTO,     // default: rebake it within the automatic limits
  PROPLM_LIGHTMAP_VERTEX,   // implicit props are not rebaked: vertex lighting
  PROPLM_LIGHTMAP_EXPLICIT, // only props marked proplm "rebake" are candidates
};
extern ProplmLightmapMode_t g_ProplmLightmapMode;

// What AddStaticPropToLump should do with this prop entity.
enum ProplmRoute_t {
  PROPLM_ROUTE_NONE,    // not a candidate: stock per-texel lighting
  PROPLM_ROUTE_KEEP,    // measured, no overlap: keep the entity's own model
  PROPLM_ROUTE_REBAKED, // the pak copy; its dictionary name is in *ppDictName
  PROPLM_ROUTE_VERTEX,  // overlap known, rebake refused: vertex lighting
  PROPLM_ROUTE_KEEP_UNMEASURED, // the model could not be read or measured:
                                // per-texel lighting as today
};

// Finds the static props that ask for a rebake and the smallest lightmap
// size of each model. Call once from EmitStaticProps before the props are
// emitted.
void ProplmRebake_Prepare();

// The route this prop entity's lighting takes. The model is rebaked (files
// written to the pak) on the first call for it; *ppDictName is set only for
// PROPLM_ROUTE_REBAKED. A refused rebake prints one Warning per prop. Call
// only for props VBSP keeps.
ProplmRoute_t ProplmRebake_RouteForProp(entity_t *pEnt, const char **ppDictName);

// Prints the map total. Call once after the props are emitted.
void ProplmRebake_Summary();

#endif // PROPLM_REBAKE_H
