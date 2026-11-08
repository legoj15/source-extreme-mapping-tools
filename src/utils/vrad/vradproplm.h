//========= proplm: second-UV prop lightmaps (VRAD half) =====================
//
// Bakes the lightmap textures for props that VBSP converted to prop_dynamic
// entities carrying the proplm keyvalues, and adds them to the pakfile.
// Format contract: src/public/proplm.h. Design notes: docs/proplm.md.
//=============================================================================
#ifndef VRADPROPLM_H
#define VRADPROPLM_H
#pragma once

// For every entity with classname prop_dynamic and proplm "1": rasterize the
// model's triangles from the 'plm2' game lump over UV2, light each covered
// texel with the same sampler the static prop .ppl bake uses, and add
// materials/<proplm_lm>.vtf (RGB888, values halved) to the pakfile.
// Call after the world lighting is solved, next to the static prop lighting but
// independent of -StaticPropLighting. Does nothing when no entity asks for it.
// Under -fast, or when the 'plm2' lump is unusable, it bakes nothing and prints
// one Warning with the number of proplm props whose VTFs were not written.
void ComputePropLmLighting();

#endif // VRADPROPLM_H
