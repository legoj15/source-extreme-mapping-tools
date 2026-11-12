//========= proplm: second-UV prop lightmaps (VRAD half) =====================
//
// VBSP replaces opted-in static props with prop_dynamic entities (keyvalues
// proplm, proplm_id, proplm_lm, proplm_res) and writes every distinct model's
// triangles, with a second UV set (UV2), into the 'plm2' game lump. Here we
// rasterize those triangles over UV2 for each instance, light every covered
// texel with the same sampler the static prop .ppl bake uses, and add the
// result to the pakfile as materials/<proplm_lm>.vtf.
//
// Format contract: src/public/proplm.h. Design notes: docs/proplm.md.
//=============================================================================

#include "vradproplm.h"

#include "pacifier.h"
#include "proplm.h"
#include "tier1/strtools.h"
#include "tier1/utlbuffer.h"
#include "vrad.h"
#include "vtf/vtf.h"

#include <math.h>
#include <string.h>

namespace {

// Largest lightmap side we accept from an entity: VBSP's own cap
// (PROPLM_RES_MAX), so a hand-edited proplm_res cannot ask for more memory than
// anything VBSP writes. It bounds the per-instance texel array.
const int kMaxLightmapRes = 2048;

// Uncovered texels within this many texels of a covered one are filled from
// their covered neighbors so bilinear filtering never blends in black.
const int kDilatePasses = 4;

// Barycentric slack so texels exactly on a shared edge are not lost to float
// error. The first triangle to claim a texel keeps it.
const double kInsideEpsilon = 1.0e-4;

// A model from the 'plm2' lump. The triangles stay in the lump memory.
struct PropLmModel_t {
  char m_Name[PROPLM_MODEL_NAME_LEN + 1]; // lowercase, forward slashes
  int m_TriCount;
  const byte *m_pTris; // m_TriCount packed ProplmTri_t, possibly unaligned
};

// One entity to bake.
struct PropLmJob_t {
  const PropLmModel_t *m_pModel;
  Vector m_Origin;
  QAngle m_Angles;
  float m_Scale; // modelscale (1 when missing or not positive)
  int m_Res;
  int m_EntityIndex;
  int m_TraceId;    // low bits of TRACE_ID_STATICPROP | id in the tracer
  int m_SkipId;     // the id the sampler skips: m_TraceId or -1
  int m_LightFlags; // GATHERLFLAGS_*
  bool m_bCastShadows;
  char m_PakName[MAX_PATH]; // materials/<proplm_lm>.vtf
};

// Tracer ids of proplm instances: TRACE_ID_STATICPROP | (kTraceIdBase + job),
// above any static prop index (the static prop lump holds at most 65535), so
// the two never collide and a skip id names exactly one of them.
const int kTraceIdBase = 0x00800000;

// The prepared jobs, built once (PrepareJobs) and used by the tracer setup
// and by the bake. The models point into the game lump, which lives as long
// as the BSP.
struct PropLmState_t {
  bool m_bPrepared;
  int m_Wanted;       // prop_dynamic entities with proplm "1"
  bool m_bLumpOk;     // the 'plm2' lump parsed
  char m_Why[256];    // why it did not
  CUtlVector<PropLmModel_t> m_Models;
  CUtlVector<PropLmJob_t> m_Jobs;
};
PropLmState_t s_State;

// Row-parallel lighting state for the instance being baked. RunThreadsOn
// takes no user data through the macro that prints the worker name, so the
// workers read this.
struct LightingContext_t {
  CUtlVector<colorTexel_t> *m_pTexels;
  int m_Res;
  int m_SkipId;
  int m_LightFlags;
};
LightingContext_t s_Lighting;

//-----------------------------------------------------------------------------
// Lowercase, forward-slash copy of a model or texture path.
//-----------------------------------------------------------------------------
void NormalizePath(const char *pSrc, char *pDst, int dstSize) {
  V_strncpy(pDst, pSrc, dstSize);
  for (char *p = pDst; *p; ++p) {
    if (*p == '\\')
      *p = '/';
  }
  V_strlower(pDst);
}

//-----------------------------------------------------------------------------
// Parses the 'plm2' lump with bounds checking. Returns false with the reason in
// pWhy when the lump is missing, stale or corrupt; the caller reports it.
//-----------------------------------------------------------------------------
bool ParsePropLmLump(CUtlVector<PropLmModel_t> &models, char *pWhy, int whyLen) {
  GameLumpHandle_t handle = g_GameLumps.GetGameLumpHandle(GAMELUMP_PROPLM_ID);
  if (handle == g_GameLumps.InvalidGameLump()) {
    V_snprintf(pWhy, whyLen,
               "the map has no 'plm2' game lump (was it compiled with the "
               "proplm VBSP?)");
    return false;
  }

  // ParseGameLump inflates an LZMA sub-lump; the flag survives only when that
  // failed, and the bytes are then not a 'plm2' lump at all (#63).
  if (g_GameLumps.GetGameLumpFlags(handle) & GAMELUMPFLAG_COMPRESSED) {
    V_snprintf(pWhy, whyLen,
               "the 'plm2' game lump is compressed and could not be inflated; "
               "recompile the map, or repack it with this repo's bspzip, "
               "which leaves 'plm2' uncompressed");
    return false;
  }

  const byte *pBase = (const byte *)g_GameLumps.GetGameLump(handle);
  const uint64 lumpSize = (uint64)Max(0, g_GameLumps.GameLumpSize(handle));
  if (!pBase || lumpSize < sizeof(ProplmHeader_t)) {
    V_snprintf(pWhy, whyLen, "the 'plm2' game lump is too small (%llu bytes)",
               (unsigned long long)lumpSize);
    return false;
  }

  ProplmHeader_t header;
  memcpy(&header, pBase, sizeof(header));
  uint64 offset = sizeof(header);

  if (header.m_Version != GAMELUMP_PROPLM_VERSION) {
    V_snprintf(pWhy, whyLen, "the 'plm2' game lump is version %d, expected %d",
               header.m_Version, GAMELUMP_PROPLM_VERSION);
    return false;
  }
  if (header.m_ModelCount < 0 ||
      (uint64)header.m_ModelCount >
          (lumpSize - offset) / sizeof(ProplmModel_t)) {
    V_snprintf(pWhy, whyLen,
               "the 'plm2' game lump is corrupt (model count %d in %llu bytes)",
               header.m_ModelCount, (unsigned long long)lumpSize);
    return false;
  }

  for (int i = 0; i < header.m_ModelCount; ++i) {
    if (lumpSize - offset < sizeof(ProplmModel_t)) {
      V_snprintf(pWhy, whyLen, "the 'plm2' game lump is truncated at model %d",
                 i);
      models.Purge();
      return false;
    }

    ProplmModel_t model;
    memcpy(&model, pBase + offset, sizeof(model));
    offset += sizeof(model);

    if (model.m_MaterialCount < 0 || model.m_TriCount < 0) {
      V_snprintf(pWhy, whyLen,
                 "the 'plm2' game lump is corrupt (model %d has %d materials, "
                 "%d triangles)",
                 i, model.m_MaterialCount, model.m_TriCount);
      models.Purge();
      return false;
    }

    const uint64 materialBytes =
        (uint64)model.m_MaterialCount * sizeof(ProplmMaterial_t);
    const uint64 triBytes = (uint64)model.m_TriCount * sizeof(ProplmTri_t);
    if (materialBytes + triBytes > lumpSize - offset) {
      V_snprintf(pWhy, whyLen,
                 "the 'plm2' game lump is truncated inside model %d (%d "
                 "materials, %d triangles)",
                 i, model.m_MaterialCount, model.m_TriCount);
      models.Purge();
      return false;
    }

    // Every corner must be finite: a NaN or inf would reach the rasterizer's
    // float-to-int casts and the lighting math.
    const ProplmTri_t *pTris = (const ProplmTri_t *)(pBase + offset + materialBytes);
    for (int t = 0; t < model.m_TriCount; ++t) {
      ProplmTri_t tri;
      memcpy(&tri, pTris + t, sizeof(tri));
      for (int c = 0; c < 3; ++c) {
        const ProplmCorner_t &k = tri.m_Corner[c];
        const float vals[8] = {k.m_Pos[0],    k.m_Pos[1],    k.m_Pos[2],
                               k.m_Normal[0], k.m_Normal[1], k.m_Normal[2],
                               k.m_UV2[0],    k.m_UV2[1]};
        for (int v = 0; v < 8; ++v) {
          if (!IsFinite(vals[v])) {
            V_snprintf(pWhy, whyLen,
                       "the 'plm2' game lump is corrupt (model %d triangle %d "
                       "has a non-finite position, normal or UV2)",
                       i, t);
            models.Purge();
            return false;
          }
        }
      }
    }

    PropLmModel_t entry;
    memset(&entry, 0, sizeof(entry));
    // The name field is fixed width; do not trust it to be terminated.
    char name[PROPLM_MODEL_NAME_LEN + 1];
    memcpy(name, model.m_Name, PROPLM_MODEL_NAME_LEN);
    name[PROPLM_MODEL_NAME_LEN] = 0;
    NormalizePath(name, entry.m_Name, sizeof(entry.m_Name));
    entry.m_TriCount = model.m_TriCount;
    entry.m_pTris = pBase + offset + materialBytes;
    models.AddToTail(entry);

    offset += materialBytes + triBytes;
  }

  return true;
}

const PropLmModel_t *FindModel(const CUtlVector<PropLmModel_t> &models,
                               const char *pName) {
  for (int i = 0; i < models.Count(); ++i) {
    if (!Q_stricmp(models[i].m_Name, pName))
      return &models[i];
  }
  return NULL;
}

//-----------------------------------------------------------------------------
// Rasterizes one world-space triangle over UV2. Texel (x, y) has its center at
// ((x + 0.5) / res, (y + 0.5) / res) and row = v * res with no v flip, the
// same convention as the static prop Rasterizer. Fills position and normal of
// every texel the triangle covers and has not been claimed yet.
//-----------------------------------------------------------------------------
void RasterizeTriangle(const Vector2D uv[3], const Vector worldPos[3],
                       const Vector worldNormal[3], int res,
                       CUtlVector<colorTexel_t> &texels) {
  const double a0x = uv[0].x, a0y = uv[0].y;
  const double edgeAx = uv[1].x - a0x, edgeAy = uv[1].y - a0y;
  const double edgeBx = uv[2].x - a0x, edgeBy = uv[2].y - a0y;

  const double dAA = edgeAx * edgeAx + edgeAy * edgeAy;
  const double dAB = edgeAx * edgeBx + edgeAy * edgeBy;
  const double dBB = edgeBx * edgeBx + edgeBy * edgeBy;
  const double denom = dAA * dBB - dAB * dAB;
  if (!(denom > 1.0e-24)) {
    // Zero area (or NaN): covers nothing.
    return;
  }
  const double invDenom = 1.0 / denom;

  // Clamped to [0,1] so the casts below stay in range whatever the lump holds.
  const float minU = clamp(Min(Min(uv[0].x, uv[1].x), uv[2].x), 0.0f, 1.0f);
  const float maxU = clamp(Max(Max(uv[0].x, uv[1].x), uv[2].x), 0.0f, 1.0f);
  const float minV = clamp(Min(Min(uv[0].y, uv[1].y), uv[2].y), 0.0f, 1.0f);
  const float maxV = clamp(Max(Max(uv[0].y, uv[1].y), uv[2].y), 0.0f, 1.0f);

  // Texel centers that can fall inside the bounding box.
  const int x0 = Max(0, (int)floorf(minU * res - 0.5f));
  const int x1 = Min(res - 1, (int)ceilf(maxU * res - 0.5f));
  const int y0 = Max(0, (int)floorf(minV * res - 0.5f));
  const int y1 = Min(res - 1, (int)ceilf(maxV * res - 0.5f));

  const double invRes = 1.0 / res;
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      colorTexel_t &texel = texels[y * res + x];
      if (texel.m_bValid)
        continue;

      const double cx = (x + 0.5) * invRes - a0x;
      const double cy = (y + 0.5) * invRes - a0y;
      const double dCA = cx * edgeAx + cy * edgeAy;
      const double dCB = cx * edgeBx + cy * edgeBy;

      const double b1 = (dBB * dCA - dAB * dCB) * invDenom;
      const double b2 = (dAA * dCB - dAB * dCA) * invDenom;
      const double b0 = 1.0 - b1 - b2;
      if (b0 < -kInsideEpsilon || b1 < -kInsideEpsilon || b2 < -kInsideEpsilon)
        continue;

      // Clamp so the slack cannot extrapolate noticeably off the surface.
      const float f0 = (float)Max(0.0, b0);
      const float f1 = (float)Max(0.0, b1);
      const float f2 = (float)Max(0.0, b2);
      const float invSum = 1.0f / (f0 + f1 + f2);

      texel.m_WorldPosition =
          (worldPos[0] * f0 + worldPos[1] * f1 + worldPos[2] * f2) * invSum;

      Vector normal =
          worldNormal[0] * f0 + worldNormal[1] * f1 + worldNormal[2] * f2;
      normal.NormalizeInPlace();
      texel.m_WorldNormal = normal;

      texel.m_bValid = true;
      texel.m_bPossiblyInteresting = true;
      texel.m_fDistanceToTri = 0.0f;
    }
  }
}

//-----------------------------------------------------------------------------
// The instance transform the static props use (rotate, then translate), with
// modelscale applied first to positions. Normals rotate only.
//-----------------------------------------------------------------------------
void InstanceMatrices(const PropLmJob_t &job, matrix3x4_t &matPos,
                      matrix3x4_t &matNormal) {
  AngleMatrix(job.m_Angles, job.m_Origin, matPos);
  AngleMatrix(job.m_Angles, matNormal);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c)
      matPos[r][c] *= job.m_Scale;
  }
}

//-----------------------------------------------------------------------------
// Rasterizes all of the instance's triangles into texels (world position and
// normal per covered texel). Returns the number of covered texels.
//-----------------------------------------------------------------------------
int RasterizeInstance(const PropLmJob_t &job,
                      CUtlVector<colorTexel_t> &texels) {
  const int res = job.m_Res;
  texels.SetCount(res * res);
  memset(texels.Base(), 0, texels.Count() * sizeof(colorTexel_t));

  // Same instance transform the static props use: rotate then translate for
  // positions, rotate only for normals.
  matrix3x4_t matPos, matNormal;
  InstanceMatrices(job, matPos, matNormal);

  for (int i = 0; i < job.m_pModel->m_TriCount; ++i) {
    ProplmTri_t tri;
    memcpy(&tri, job.m_pModel->m_pTris + (size_t)i * sizeof(ProplmTri_t),
           sizeof(tri));

    Vector2D uv[3];
    Vector worldPos[3];
    Vector worldNormal[3];
    for (int c = 0; c < 3; ++c) {
      uv[c].Init(tri.m_Corner[c].m_UV2[0], tri.m_Corner[c].m_UV2[1]);

      Vector modelPos(tri.m_Corner[c].m_Pos[0], tri.m_Corner[c].m_Pos[1],
                      tri.m_Corner[c].m_Pos[2]);
      Vector modelNormal(tri.m_Corner[c].m_Normal[0],
                         tri.m_Corner[c].m_Normal[1],
                         tri.m_Corner[c].m_Normal[2]);
      VectorTransform(modelPos, matPos, worldPos[c]);
      VectorTransform(modelNormal, matNormal, worldNormal[c]);
    }

    RasterizeTriangle(uv, worldPos, worldNormal, res, texels);
  }

  int covered = 0;
  for (int i = 0; i < texels.Count(); ++i) {
    if (texels[i].m_bValid)
      ++covered;
  }
  return covered;
}

//-----------------------------------------------------------------------------
// Worker: lights the covered texels of whole rows. The sampler is the one the
// static prop .ppl texels use, with the instance's skip id and flags (see
// PrepareJobs: the prop skips itself only under disableselfshadowing or
// -nossprops, as the static props do).
//-----------------------------------------------------------------------------
void ThreadLightRows(int iThread, void *) {
  CUtlVector<colorTexel_t> &texels = *s_Lighting.m_pTexels;
  const int res = s_Lighting.m_Res;

  for (;;) {
    int row = GetThreadWork();
    if (row == -1)
      break;

    for (int x = 0; x < res; ++x) {
      colorTexel_t &texel = texels[row * res + x];
      if (!texel.m_bValid)
        continue;

      ComputeTexelLightingAtPoint(texel.m_WorldPosition, texel.m_WorldNormal,
                                  iThread, s_Lighting.m_SkipId,
                                  s_Lighting.m_LightFlags, texel.m_Color);
    }
  }
}

//-----------------------------------------------------------------------------
// Fills uncovered texels next to filled ones with the average of their filled
// neighbors, kDilatePasses times. filled (one byte per texel) is updated.
//-----------------------------------------------------------------------------
void DilateTexels(int res, unsigned char *pRGB,
                  CUtlVector<unsigned char> &filled) {
  CUtlVector<unsigned char> nextFilled;
  CUtlVector<unsigned char> nextRGB;

  for (int pass = 0; pass < kDilatePasses; ++pass) {
    nextFilled.CopyArray(filled.Base(), filled.Count());
    nextRGB.CopyArray(pRGB, res * res * 3);
    bool changed = false;

    for (int y = 0; y < res; ++y) {
      for (int x = 0; x < res; ++x) {
        if (filled[y * res + x])
          continue;

        int sum[3] = {0, 0, 0};
        int count = 0;
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            int nx = x + dx, ny = y + dy;
            if ((dx == 0 && dy == 0) || nx < 0 || ny < 0 || nx >= res ||
                ny >= res || !filled[ny * res + nx])
              continue;
            const unsigned char *pN = pRGB + (ny * res + nx) * 3;
            sum[0] += pN[0];
            sum[1] += pN[1];
            sum[2] += pN[2];
            ++count;
          }
        }

        if (count) {
          unsigned char *pOut = nextRGB.Base() + (y * res + x) * 3;
          pOut[0] = (unsigned char)((sum[0] + count / 2) / count);
          pOut[1] = (unsigned char)((sum[1] + count / 2) / count);
          pOut[2] = (unsigned char)((sum[2] + count / 2) / count);
          nextFilled[y * res + x] = 1;
          changed = true;
        }
      }
    }

    memcpy(pRGB, nextRGB.Base(), res * res * 3);
    filled.CopyArray(nextFilled.Base(), nextFilled.Count());
    if (!changed)
      break;
  }
}

//-----------------------------------------------------------------------------
// Serializes res x res RGB888 bytes as a VTF and adds it to the pakfile.
//-----------------------------------------------------------------------------
bool AddLightmapToPak(const char *pPakName, int res,
                      const unsigned char *pRGB) {
  // NOMIP: one image, no mip chain (the addon samples it as a plain 2D
  // lookup). NOLOD: the engine's texture-quality setting never drops it to a
  // lower resolution. CLAMPS/CLAMPT: UV2 charts must never wrap around to the
  // opposite edge. Not sRGB: the bytes are already in the lightmap space the
  // .ppl uses. There is no low-res thumbnail either.
  const int flags = TEXTUREFLAGS_NOMIP | TEXTUREFLAGS_NOLOD |
                    TEXTUREFLAGS_CLAMPS | TEXTUREFLAGS_CLAMPT;

  IVTFTexture *pVTF = CreateVTFTexture();
  // The mip count must be forced: Init() builds the full chain even when
  // NOMIP is set, and the addon wants the file to hold exactly one image.
  if (!pVTF->Init(res, res, 1, IMAGE_FORMAT_RGB888, flags, 1, 1)) {
    Warning("proplm: could not create a %dx%d VTF for %s\n", res, res,
            pPakName);
    DestroyVTFTexture(pVTF);
    return false;
  }

  memcpy(pVTF->ImageData(0, 0, 0), pRGB, (size_t)res * res * 3);

  CUtlBuffer buf;
  if (!pVTF->Serialize(buf)) {
    Warning("proplm: could not serialize %s\n", pPakName);
    DestroyVTFTexture(pVTF);
    return false;
  }
  DestroyVTFTexture(pVTF);

  AddBufferToPak(GetPakFile(), pPakName, buf.Base(), buf.TellPut(), false);
  return true;
}

//-----------------------------------------------------------------------------
// Bakes one instance and adds its VTF to the pakfile.
//-----------------------------------------------------------------------------
bool BakeInstance(const PropLmJob_t &job) {
  const int res = job.m_Res;

  CUtlVector<colorTexel_t> texels;
  const int covered = RasterizeInstance(job, texels);
  if (covered == 0) {
    Warning("proplm: entity %d (%s) covers no lightmap texels at %dx%d; "
            "no %s written\n",
            job.m_EntityIndex, job.m_pModel->m_Name, res, res, job.m_PakName);
    return false;
  }

  s_Lighting.m_pTexels = &texels;
  s_Lighting.m_Res = res;
  s_Lighting.m_SkipId = job.m_SkipId;
  s_Lighting.m_LightFlags = job.m_LightFlags;
  // RunThreadsOn draws a pacifier per call; one line per instance is enough.
  SuppressPacifier(true);
  RunThreadsOn(res, false, ThreadLightRows);
  SuppressPacifier(false);
  s_Lighting.m_pTexels = NULL;

  // Same filter and encoding as the .ppl texels.
  CUtlVector<unsigned char> rgb;
  rgb.SetCount(res * res * 3);
  EncodeTexelsRGB888(res, res, texels, rgb.Base());

  CUtlVector<unsigned char> filled;
  filled.SetCount(res * res);
  for (int i = 0; i < texels.Count(); ++i)
    filled[i] = texels[i].m_bValid ? 1 : 0;
  DilateTexels(res, rgb.Base(), filled);

  // Consumers multiply by two, so store half, rounded to nearest.
  for (int i = 0; i < rgb.Count(); ++i)
    rgb[i] = (unsigned char)((rgb[i] + 1) >> 1);

  qprintf("proplm: entity %d %s -> %s (%dx%d, %d/%d texels covered)\n",
          job.m_EntityIndex, job.m_pModel->m_Name, job.m_PakName, res, res,
          covered, res * res);

  return AddLightmapToPak(job.m_PakName, res, rgb.Base());
}

//-----------------------------------------------------------------------------
// Builds s_State once: counts the proplm entities, parses the lump and makes
// one job per usable entity (per-entity problems are Warnings here, once).
//-----------------------------------------------------------------------------
void PrepareJobs() {
  if (s_State.m_bPrepared)
    return;
  s_State.m_bPrepared = true;

  // Cheap pre-pass: most maps have no proplm entities, and then the lump is
  // never touched.
  for (int i = 0; i < num_entities; ++i) {
    entity_t *pEnt = &entities[i];
    if (!Q_stricmp(ValueForKey(pEnt, "classname"), "prop_dynamic") &&
        !Q_stricmp(ValueForKey(pEnt, "proplm"), "1"))
      ++s_State.m_Wanted;
  }
  if (!s_State.m_Wanted)
    return;

  s_State.m_bLumpOk =
      ParsePropLmLump(s_State.m_Models, s_State.m_Why, sizeof(s_State.m_Why));
  if (!s_State.m_bLumpOk)
    return;

  CUtlVector<PropLmJob_t> &jobs = s_State.m_Jobs;
  for (int i = 0; i < num_entities; ++i) {
    entity_t *pEnt = &entities[i];
    if (Q_stricmp(ValueForKey(pEnt, "classname"), "prop_dynamic") ||
        Q_stricmp(ValueForKey(pEnt, "proplm"), "1"))
      continue;

    char modelName[MAX_PATH];
    NormalizePath(ValueForKey(pEnt, "model"), modelName, sizeof(modelName));
    const PropLmModel_t *pModel = FindModel(s_State.m_Models, modelName);
    if (!pModel) {
      Warning("proplm: entity %d uses model \"%s\", which is not in the "
              "'plm2' lump; skipping\n",
              i, modelName);
      continue;
    }

    const char *pLightmap = ValueForKey(pEnt, "proplm_lm");
    if (!pLightmap[0]) {
      Warning("proplm: entity %d (%s) has no proplm_lm; skipping\n", i,
              modelName);
      continue;
    }

    const int res = IntForKey(pEnt, "proplm_res");
    if (res <= 0 || res > kMaxLightmapRes) {
      Warning("proplm: entity %d (%s) has proplm_res %d, expected 1..%d; "
              "skipping\n",
              i, modelName, res, kMaxLightmapRes);
      continue;
    }

    PropLmJob_t job;
    memset(&job, 0, sizeof(job));
    job.m_pModel = pModel;
    job.m_Res = res;
    job.m_EntityIndex = i;
    GetVectorForKey(pEnt, "origin", job.m_Origin);
    GetAnglesForKey(pEnt, "angles", job.m_Angles);
    // modelscale: the engine draws a prop_dynamic scaled by it, so the bake
    // and the shadows use it too. A missing, zero, negative or non-finite
    // value is 1 (the engine's default).
    job.m_Scale = 1.0f;
    if (ValueForKey(pEnt, "modelscale")[0]) {
      const float scale = FloatForKey(pEnt, "modelscale");
      if (IsFinite(scale) && scale > 0.0f)
        job.m_Scale = scale;
      else
        Warning("proplm: entity %d (%s) has modelscale \"%s\"; using 1\n", i,
                modelName, ValueForKey(pEnt, "modelscale"));
    }
    // The static prop keys VBSP turns into flags (EmitStaticProps); VBSP
    // leaves them on the converted entity.
    job.m_TraceId = kTraceIdBase + jobs.Count();
    job.m_bCastShadows = IntForKey(pEnt, "disableshadows") != 1;
    job.m_SkipId = (g_bDisablePropSelfShadowing ||
                    IntForKey(pEnt, "disableselfshadowing") == 1)
                       ? job.m_TraceId
                       : -1;
    job.m_LightFlags = IntForKey(pEnt, "ignorenormals") == 1
                           ? GATHERLFLAGS_IGNORE_NORMALS
                           : 0;

    char lightmapPath[MAX_PATH];
    NormalizePath(pLightmap, lightmapPath, sizeof(lightmapPath));
    V_snprintf(job.m_PakName, sizeof(job.m_PakName), "materials/%s.vtf",
               lightmapPath);

    bool duplicate = false;
    for (int j = 0; j < jobs.Count(); ++j) {
      if (!Q_stricmp(jobs[j].m_PakName, job.m_PakName)) {
        duplicate = true;
        break;
      }
    }
    if (duplicate) {
      Warning("proplm: entity %d (%s) reuses lightmap %s; skipping\n", i,
              modelName, job.m_PakName);
      continue;
    }

    jobs.AddToTail(job);
  }
}

} // namespace

//-----------------------------------------------------------------------------
void PropLm_AddPolysForRayTrace() {
  PrepareJobs();
  const Vector fullCoverage(1.0f, 0.0f, 0.0f);
  int added = 0, props = 0;
  for (int j = 0; j < s_State.m_Jobs.Count(); ++j) {
    const PropLmJob_t &job = s_State.m_Jobs[j];
    if (!job.m_bCastShadows)
      continue;
    matrix3x4_t matPos, matNormal;
    InstanceMatrices(job, matPos, matNormal);
    for (int i = 0; i < job.m_pModel->m_TriCount; ++i) {
      ProplmTri_t tri;
      memcpy(&tri, job.m_pModel->m_pTris + (size_t)i * sizeof(ProplmTri_t),
             sizeof(tri));
      Vector world[3];
      for (int c = 0; c < 3; ++c) {
        const Vector modelPos(tri.m_Corner[c].m_Pos[0],
                              tri.m_Corner[c].m_Pos[1],
                              tri.m_Corner[c].m_Pos[2]);
        VectorTransform(modelPos, matPos, world[c]);
      }
      g_RtEnv.AddTriangle(TRACE_ID_STATICPROP | job.m_TraceId, world[0],
                          world[1], world[2], fullCoverage);
      ++added;
    }
    ++props;
  }
  if (props)
    Msg("proplm: %d prop(s) add %d triangles to the ray tracer\n", props,
        added);
}

//-----------------------------------------------------------------------------
void ComputePropLmLighting() {
  PrepareJobs();
  const int wanted = s_State.m_Wanted;
  if (!wanted)
    return;

  // VBSP already converted these props, so a skipped bake leaves them without
  // a lightmap: say so once, with the count, whatever the reason.
  if (do_fast) {
    Warning("proplm: %d prop(s) have proplm \"1\", but -fast skips the prop "
            "lightmap bake; their lightmap VTFs were not written.\n",
            wanted);
    return;
  }
  if (!s_State.m_bLumpOk) {
    Warning("proplm: %d prop(s) have proplm \"1\", but %s; their lightmap VTFs "
            "were not written.\n",
            wanted, s_State.m_Why);
    return;
  }

  const double startTime = Plat_FloatTime();
  const CUtlVector<PropLmJob_t> &jobs = s_State.m_Jobs;
  Msg("Computing prop lightmaps : %d instance(s), %d model(s) in lump\n",
      jobs.Count(), s_State.m_Models.Count());

  int written = 0;
  for (int i = 0; i < jobs.Count(); ++i) {
    if (BakeInstance(jobs[i]))
      ++written;
  }

  Msg("proplm: %d of %d prop lightmaps written to the pakfile\n", written,
      jobs.Count());
  Msg("PropLmLighting: %.2f seconds\n", Plat_FloatTime() - startTime);
}
