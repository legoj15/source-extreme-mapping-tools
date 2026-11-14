//========= proplm: second-UV prop lightmaps (VBSP writes, VRAD reads) =======
//
// Custom game lump 'plm2'. Little-endian, packed, NEVER compressed.
// Format contract between VBSP (src/utils/vbsp/proplm_uv2.cpp), VRAD
// (src/utils/vrad/vradproplm.cpp) and the GMod Lua addon (game/addons/proplm).
// Documentation: docs/proplm.md.
//
// Triangles are stored with full per-corner geometry so that no consumer
// depends on VVD/VTX index order: the Lua addon matches them by
// (pos, uv0) per corner, VRAD rasterizes them directly.
//=============================================================================
#ifndef PROPLM_H
#define PROPLM_H
#pragma once

#include <stdint.h>

#define GAMELUMP_PROPLM_ID 'plm2'
#define GAMELUMP_PROPLM_VERSION 1

#define PROPLM_MODEL_NAME_LEN 128
#define PROPLM_MATERIAL_NAME_LEN 96

// The switch point the VBSP rebake writes to a lower LOD it must stop drawing
// (one of its triangles spans two islands, so it cannot follow the new
// layout): a distance no view reaches. VRAD skips a LOD at or above it when it
// lights the prop's texels (#126), so both tools read this one value.
#define PROPLM_LOD_OFF_SWITCH_POINT 1.0e7f

#pragma pack(push, 1)

// Lump layout:
//   ProplmHeader_t
//   for each model:
//     ProplmModel_t
//     ProplmMaterial_t   [materialCount]
//     ProplmTri_t        [triCount]
struct ProplmHeader_t {
  int32_t m_Version;    // GAMELUMP_PROPLM_VERSION
  int32_t m_ModelCount;
};

struct ProplmModel_t {
  char m_Name[PROPLM_MODEL_NAME_LEN]; // lowercase, "models/foo/bar.mdl"
  int32_t m_MaterialCount;
  int32_t m_TriCount;
  uint32_t m_PosChecksum; // see docs/proplm.md; lets consumers detect stale geometry
};

struct ProplmMaterial_t {
  char m_Name[PROPLM_MATERIAL_NAME_LEN]; // as in the MDL texture list, no extension
};

struct ProplmCorner_t {
  float m_Pos[3];    // model space, source units
  float m_Normal[3]; // model space, unit length (from VVD)
  float m_UV0[2];    // original VVD texcoord, v NOT flipped
  float m_UV2[2];    // lightmap UV in [0,1]; row = v * resolution, v NOT flipped
};

struct ProplmTri_t {
  int32_t m_MaterialIndex; // index into this model's ProplmMaterial_t array
  ProplmCorner_t m_Corner[3];
};

#pragma pack(pop)

// Entity keyvalues VBSP writes on every converted prop (classname prop_dynamic):
//   proplm      "1"
//   proplm_id   "<N>"                       unique per map, 0-based
//   proplm_lm   "maps/<mapname>/proplm/<N>" texture path (no materials/, no .vtf)
//   proplm_res  "<R>"                       square lightmap resolution in texels
// VRAD writes materials/maps/<mapname>/proplm/<N>.vtf into the pakfile.
// The VTF is RGB888, no mipmaps, holds the same encoded value the
// texelslighting .ppl would hold, divided by two (consumers multiply by two).

#endif // PROPLM_H
