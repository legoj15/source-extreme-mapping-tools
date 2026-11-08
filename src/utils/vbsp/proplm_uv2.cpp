//========= proplm: second-UV prop lightmaps (VBSP side) =====================
//
// For every opted-in prop (keyvalue proplm "1") this reads the model's LOD0
// triangles (MDL + VVD + VTX), unwraps them with xatlas into a second UV set
// (UV2), stores the result per model in the 'plm2' game lump and rewrites the
// entity into a prop_dynamic. Lump format: src/public/proplm.h. Design and
// consumer notes: docs/proplm.md.
//=============================================================================
#include "proplm_uv2.h"
#include "proplm_shared.h"

#include "bsplib.h"
#include "optimize.h"
#include "proplm.h"
#include "studio.h"
#include "tier1/UtlBuffer.h"
#include "tier1/checksum_crc.h"
#include "tier1/strtools.h"
#include "tier1/utlstring.h"
#include "tier1/utlvector.h"
#include "vbsp.h"
#include "xatlas.h"
#include <math.h>
#include <string.h>

using namespace OptimizedModel;

// Resolution bounds for proplm_res, and the value used when the prop has no
// lightmapresolutionx/y.
#define PROPLM_RES_DEFAULT 256
#define PROPLM_RES_MIN 16
#define PROPLM_RES_MAX 2048

// xatlas settings (the brief fixes these; see docs/proplm.md).
#define PROPLM_XATLAS_PADDING 2
#define PROPLM_XATLAS_ITERATIONS 4
// Re-pack attempts (each 3% smaller) before giving up on fitting one atlas.
#define PROPLM_FIT_ATTEMPTS 40

namespace {

// One VVD vertex after fixups (proplm_shared.h).
typedef ProplmVvdVert_t Vert_t;

// One LOD0 trilist triangle as read from the VTX, before xatlas.
struct SrcTri_t {
  int mat;       // index into the model's material table
  int vert[3];   // fixed-up VVD vertex index per corner, in VTX index order
};

// A fully processed model (or a remembered failure).
struct Model_t {
  char key[MAX_PATH]; // lowercase, forward slashes; the cache key and lump name
  bool failed;
  char reason[256]; // why it failed, for the per-prop Warning
  int atlasRes; // resolution the charts were packed for
  CUtlVector<ProplmMaterial_t> materials;
  CUtlVector<ProplmTri_t> tris;
  uint32_t posChecksum;
};

} // namespace

static CUtlVector<Model_t *> s_Models;
static int s_NextPropId = 0;
// Lowercase material names already reported as too long for the lump.
static CUtlVector<CUtlString> s_TruncatedMaterials;

//-----------------------------------------------------------------------------
// Small helpers
//-----------------------------------------------------------------------------
static Model_t *FindModel(const char *pLowerName) {
  for (int i = 0; i < s_Models.Count(); ++i) {
    if (!strcmp(s_Models[i]->key, pLowerName))
      return s_Models[i];
  }
  return NULL;
}

// Index of pMaterial in the model's table (case-insensitive), appended if new.
// A name that does not fit ProplmMaterial_t::m_Name is cut off before the
// comparison, so a Warning is printed once per such name: the lump then holds a
// name the engine cannot resolve, and two long names sharing a prefix merge.
static int FindOrAddMaterial(Model_t *pModel, const char *pMaterial) {
  ProplmMaterial_t mat;
  memset(&mat, 0, sizeof(mat));
  if (V_strlen(pMaterial) >= (int)sizeof(mat.m_Name)) {
    char lowerFull[MAX_PATH];
    V_strncpy(lowerFull, pMaterial, sizeof(lowerFull));
    V_strlower(lowerFull);
    bool seen = false;
    for (int i = 0; i < s_TruncatedMaterials.Count(); ++i) {
      if (!strcmp(s_TruncatedMaterials[i].Get(), lowerFull)) {
        seen = true;
        break;
      }
    }
    if (!seen) {
      s_TruncatedMaterials.AddToTail(CUtlString(lowerFull));
      Warning("Warning: proplm: model \"%s\": material name \"%s\" is longer "
              "than %d characters and is truncated in the 'plm2' lump.\n",
              pModel->key, lowerFull, (int)sizeof(mat.m_Name) - 1);
    }
  }
  V_strncpy(mat.m_Name, pMaterial, sizeof(mat.m_Name));
  V_strlower(mat.m_Name);
  for (int i = 0; i < pModel->materials.Count(); ++i) {
    if (!memcmp(&pModel->materials[i], &mat, sizeof(mat)))
      return i;
  }
  return pModel->materials.AddToTail(mat);
}

// Rounded 1/16-unit position, the unit of m_PosChecksum.
static int32_t QuantizePos(float v) { return (int32_t)floor((double)v * 16.0 + 0.5); }

//-----------------------------------------------------------------------------
// File loading (shared with the rebake route through proplm_shared.h)
//-----------------------------------------------------------------------------
bool Proplm_ReadWholeFile(const char *pPath, CUtlBuffer &buf) {
  return g_pFullFileSystem->ReadFile(pPath, NULL, buf) && buf.TellPut() > 0;
}

static bool ReadWholeFile(const char *pPath, CUtlBuffer &buf) {
  return Proplm_ReadWholeFile(pPath, buf);
}

static bool VvdVertexIsFinite(const mstudiovertex_t &v) {
  return IsFinite(v.m_vecPosition.x) && IsFinite(v.m_vecPosition.y) &&
         IsFinite(v.m_vecPosition.z) && IsFinite(v.m_vecNormal.x) &&
         IsFinite(v.m_vecNormal.y) && IsFinite(v.m_vecNormal.z) &&
         IsFinite(v.m_vecTexCoord.x) && IsFinite(v.m_vecTexCoord.y);
}

static void CopyVvdVertex(const mstudiovertex_t &src, int raw, Vert_t &dst) {
  dst.pos[0] = src.m_vecPosition.x;
  dst.pos[1] = src.m_vecPosition.y;
  dst.pos[2] = src.m_vecPosition.z;
  dst.nrm[0] = src.m_vecNormal.x;
  dst.nrm[1] = src.m_vecNormal.y;
  dst.nrm[2] = src.m_vecNormal.z;
  dst.uv[0] = src.m_vecTexCoord.x;
  dst.uv[1] = src.m_vecTexCoord.y;
  dst.raw = raw;
}

// Fills pVerts with the LOD0 vertices of the VVD, fixups applied (the same
// order the VTX's origMeshVertID + mesh vertexoffset index into).
bool Proplm_LoadVvdVertices(const CUtlBuffer &vvdBuf, int mdlChecksum,
                            float maxTexcoord, CUtlVector<Vert_t> &verts,
                            char *pErr, int errLen) {
  const int fileSize = vvdBuf.TellPut();
  const byte *pBase = (const byte *)vvdBuf.Base();
  if (fileSize < (int)sizeof(vertexFileHeader_t)) {
    V_snprintf(pErr, errLen, "VVD is truncated");
    return false;
  }
  const vertexFileHeader_t *pHdr = (const vertexFileHeader_t *)pBase;
  if (pHdr->id != MODEL_VERTEX_FILE_ID ||
      pHdr->version != MODEL_VERTEX_FILE_VERSION) {
    V_snprintf(pErr, errLen, "VVD has the wrong id or version");
    return false;
  }
  if (pHdr->checksum != mdlChecksum) {
    V_snprintf(pErr, errLen, "VVD checksum does not match the MDL");
    return false;
  }

  const int lod0Count = pHdr->numLODVertexes[0];
  if (lod0Count <= 0 || pHdr->vertexDataStart <= 0 ||
      (int64_t)pHdr->vertexDataStart +
              (int64_t)lod0Count * (int64_t)sizeof(mstudiovertex_t) >
          fileSize) {
    V_snprintf(pErr, errLen, "VVD vertex block is missing or out of range");
    return false;
  }
  const mstudiovertex_t *pRaw =
      (const mstudiovertex_t *)(pBase + pHdr->vertexDataStart);
  for (int v = 0; v < lod0Count; ++v) {
    if (!VvdVertexIsFinite(pRaw[v])) {
      V_snprintf(pErr, errLen,
                 "VVD vertex %d has a non-finite position, normal or texcoord",
                 v);
      return false;
    }
    if (maxTexcoord > 0.0f &&
        (fabsf(pRaw[v].m_vecTexCoord.x) > maxTexcoord ||
         fabsf(pRaw[v].m_vecTexCoord.y) > maxTexcoord)) {
      V_snprintf(pErr, errLen,
                 "VVD vertex %d has a texcoord (%g %g) beyond +-%g", v,
                 pRaw[v].m_vecTexCoord.x, pRaw[v].m_vecTexCoord.y,
                 maxTexcoord);
      return false;
    }
  }

  verts.SetCount(lod0Count);
  int target = 0;
  if (pHdr->numFixups > 0) {
    if (pHdr->fixupTableStart <= 0 ||
        (int64_t)pHdr->fixupTableStart +
                (int64_t)pHdr->numFixups * (int64_t)sizeof(vertexFileFixup_t) >
            fileSize) {
      V_snprintf(pErr, errLen, "VVD fixup table is out of range");
      return false;
    }
    const vertexFileFixup_t *pFix =
        (const vertexFileFixup_t *)(pBase + pHdr->fixupTableStart);
    // LOD0 takes every fixup (the lod field is the highest LOD it applies to).
    for (int f = 0; f < pHdr->numFixups; ++f) {
      if (pFix[f].lod < 0)
        continue;
      if (pFix[f].sourceVertexID < 0 || pFix[f].numVertexes < 0 ||
          (int64_t)pFix[f].sourceVertexID + pFix[f].numVertexes > lod0Count ||
          target + pFix[f].numVertexes > lod0Count) {
        V_snprintf(pErr, errLen, "VVD fixup %d is out of range", f);
        return false;
      }
      for (int v = 0; v < pFix[f].numVertexes; ++v) {
        const int raw = pFix[f].sourceVertexID + v;
        CopyVvdVertex(pRaw[raw], raw, verts[target++]);
      }
    }
    if (target != lod0Count) {
      V_snprintf(pErr, errLen, "VVD fixups cover %d of %d vertices", target,
                 lod0Count);
      return false;
    }
  } else {
    for (int v = 0; v < lod0Count; ++v)
      CopyVvdVertex(pRaw[v], v, verts[v]);
  }
  return true;
}

//-----------------------------------------------------------------------------
// MDL table checks (the file layout the walkers below rely on).
//-----------------------------------------------------------------------------
static_assert(sizeof(mstudiotexture_t) == 64, "mstudiotexture_t file size");
static_assert(sizeof(mstudiobodyparts_t) == 16, "mstudiobodyparts_t file size");
static_assert(sizeof(mstudiomodel_t) == 148, "mstudiomodel_t file size");
static_assert(sizeof(mstudiomesh_t) == 116, "mstudiomesh_t file size");
static_assert(sizeof(studiohdr2_t) == 256, "studiohdr2_t file size");

// A NUL-terminated string starts at byte ofs and ends inside the file.
static bool StringInFile(const byte *pBase, int64 ofs, int64 size) {
  if (ofs < 0 || ofs >= size)
    return false;
  return memchr(pBase + ofs, 0, (size_t)(size - ofs)) != NULL;
}

bool Proplm_ConvertMdl(studiohdr_t *pHdr, int fileSize, char *pErr,
                       int errLen) {
  if (fileSize < (int)sizeof(studiohdr_t)) {
    V_snprintf(pErr, errLen, "the MDL is truncated");
    return false;
  }
  if (pHdr->version < 44 || pHdr->version > STUDIO_VERSION) {
    V_snprintf(pErr, errLen, "studio version %d is not 44..%d", pHdr->version,
               STUDIO_VERSION);
    return false;
  }
  // Versions 44..47 walk and write the local anim descriptors.
  if (pHdr->version != STUDIO_VERSION &&
      !Proplm_InFile(pHdr->localanimindex, pHdr->numlocalanim,
                     sizeof(mstudioanimdesc_t), fileSize)) {
    V_snprintf(pErr, errLen, "the MDL's local animation table is out of range");
    return false;
  }
  Studio_ConvertStudioHdrToNewVersion(pHdr);
  return true;
}

bool Proplm_ValidateMdl(const studiohdr_t *pHdr, int fileSize, char *pErr,
                        int errLen) {
  const int64 size = fileSize;
  const byte *pBase = (const byte *)pHdr;
  if (size < (int64)sizeof(studiohdr_t)) {
    V_snprintf(pErr, errLen, "the MDL is truncated");
    return false;
  }
#define PROPLM_MDL_FAIL(...)                                                   \
  do {                                                                         \
    V_snprintf(pErr, errLen, __VA_ARGS__);                                     \
    return false;                                                              \
  } while (0)

  if (!Proplm_InFile(pHdr->textureindex, pHdr->numtextures,
                     sizeof(mstudiotexture_t), size))
    PROPLM_MDL_FAIL("the MDL's texture table is out of range");
  for (int i = 0; i < pHdr->numtextures; ++i) {
    const int64 ofs = (int64)pHdr->textureindex + (int64)i * sizeof(mstudiotexture_t);
    const mstudiotexture_t *pTex = (const mstudiotexture_t *)(pBase + ofs);
    if (!StringInFile(pBase, ofs + pTex->sznameindex, size))
      PROPLM_MDL_FAIL("the MDL's texture %d name is out of range", i);
  }
  if (!Proplm_InFile(pHdr->cdtextureindex, pHdr->numcdtextures, sizeof(int), size))
    PROPLM_MDL_FAIL("the MDL's cdtexture table is out of range");
  for (int i = 0; i < pHdr->numcdtextures; ++i) {
    const int strOfs =
        *(const int *)(pBase + (int64)pHdr->cdtextureindex + (int64)i * sizeof(int));
    if (!StringInFile(pBase, strOfs, size))
      PROPLM_MDL_FAIL("the MDL's cdtexture %d is out of range", i);
  }
  if (pHdr->numskinref < 0 || pHdr->numskinfamilies < 0 ||
      !Proplm_InFile(pHdr->skinindex,
                     (int64)pHdr->numskinref * (int64)pHdr->numskinfamilies,
                     sizeof(short), size))
    PROPLM_MDL_FAIL("the MDL's skin table is out of range");
  const short *pSkins = (const short *)(pBase + pHdr->skinindex);
  for (int64 i = 0; i < (int64)pHdr->numskinref * pHdr->numskinfamilies; ++i) {
    if (pSkins[i] < 0 || pSkins[i] >= pHdr->numtextures)
      PROPLM_MDL_FAIL("the MDL's skin table names texture %d of %d", pSkins[i],
                      pHdr->numtextures);
  }
  if (!Proplm_InFile(pHdr->bodypartindex, pHdr->numbodyparts,
                     sizeof(mstudiobodyparts_t), size))
    PROPLM_MDL_FAIL("the MDL's bodypart table is out of range");
  for (int bp = 0; bp < pHdr->numbodyparts; ++bp) {
    const int64 bpOfs =
        (int64)pHdr->bodypartindex + (int64)bp * sizeof(mstudiobodyparts_t);
    const mstudiobodyparts_t *pBp = (const mstudiobodyparts_t *)(pBase + bpOfs);
    const int64 modelBase = bpOfs + pBp->modelindex;
    if (!Proplm_InFile(modelBase, pBp->nummodels, sizeof(mstudiomodel_t), size))
      PROPLM_MDL_FAIL("the MDL's bodypart %d model table is out of range", bp);
    for (int m = 0; m < pBp->nummodels; ++m) {
      const int64 mOfs = modelBase + (int64)m * sizeof(mstudiomodel_t);
      const mstudiomodel_t *pModel = (const mstudiomodel_t *)(pBase + mOfs);
      if (pModel->vertexindex < 0 || pModel->numvertices < 0)
        PROPLM_MDL_FAIL("the MDL's bodypart %d model %d has a negative vertex "
                        "range",
                        bp, m);
      const int64 meshBase = mOfs + pModel->meshindex;
      if (!Proplm_InFile(meshBase, pModel->nummeshes, sizeof(mstudiomesh_t), size))
        PROPLM_MDL_FAIL("the MDL's bodypart %d model %d mesh table is out of "
                        "range",
                        bp, m);
      for (int k = 0; k < pModel->nummeshes; ++k) {
        const mstudiomesh_t *pMesh = (const mstudiomesh_t *)(
            pBase + meshBase + (int64)k * sizeof(mstudiomesh_t));
        const int limit = pHdr->numskinref > 0 ? pHdr->numskinref : pHdr->numtextures;
        if (pMesh->material < 0 || pMesh->material >= limit ||
            pMesh->vertexoffset < 0)
          PROPLM_MDL_FAIL("the MDL's bodypart %d model %d mesh %d is out of "
                          "range",
                          bp, m, k);
      }
    }
  }
  if (pHdr->studiohdr2index) {
    const int64 h2 = pHdr->studiohdr2index;
    if (h2 <= 0 || (h2 & 3) || !Proplm_InFile(h2, 1, sizeof(studiohdr2_t), size))
      PROPLM_MDL_FAIL("the MDL's studiohdr2 is out of range");
    const studiohdr2_t *pHdr2 = (const studiohdr2_t *)(pBase + h2);
    if (pHdr2->sznameindex && !StringInFile(pBase, h2 + pHdr2->sznameindex, size))
      PROPLM_MDL_FAIL("the MDL's studiohdr2 name is out of range");
  }
#undef PROPLM_MDL_FAIL
  return true;
}

//-----------------------------------------------------------------------------
// Walks VTX LOD lod of every bodypart (model 0 only, or every model) and
// records each trilist triangle in VTX order. Strips that are not trilists are
// skipped, exactly like the other VBSP model walkers, unless bRequireTrilists.
// Every VTX offset and count is checked against the buffer before use.
//-----------------------------------------------------------------------------
bool Proplm_GatherVtxTriangles(studiohdr_t *pStudioHdr, const CUtlBuffer &vtxBuf,
                               int vertCount, int lod, bool bAllModels,
                               bool bRequireTrilists,
                               CUtlVector<ProplmVtxMesh_t> *pMeshes,
                               CUtlVector<ProplmVtxTri_t> &tris, char *pErr,
                               int errLen) {
  const int64 size = vtxBuf.TellPut();
  const byte *pBase = (const byte *)vtxBuf.Base();
  if (size < (int64)sizeof(FileHeader_t)) {
    V_snprintf(pErr, errLen, "VTX is truncated");
    return false;
  }
  const FileHeader_t *pVtxHdr = (const FileHeader_t *)pBase;
  if (pVtxHdr->version != OPTIMIZED_MODEL_FILE_VERSION) {
    V_snprintf(pErr, errLen, "VTX has version %d, expected %d",
               pVtxHdr->version, OPTIMIZED_MODEL_FILE_VERSION);
    return false;
  }
  if (pVtxHdr->checkSum != pStudioHdr->checksum) {
    V_snprintf(pErr, errLen, "VTX checksum does not match the MDL");
    return false;
  }
#define PROPLM_VTX_RANGE(ofs, count, type, what)                               \
  if (!Proplm_InFile((ofs), (count), sizeof(type), size)) {                    \
    V_snprintf(pErr, errLen, "VTX %s is out of range", what);                  \
    return false;                                                              \
  }

  const int64 bpBase = pVtxHdr->bodyPartOffset;
  PROPLM_VTX_RANGE(bpBase, pVtxHdr->numBodyParts, BodyPartHeader_t, "bodypart table");
  int meshCounter = 0;
  for (int bp = 0; bp < pStudioHdr->numbodyparts && bp < pVtxHdr->numBodyParts;
       ++bp) {
    mstudiobodyparts_t *pBodyPart = pStudioHdr->pBodypart(bp);
    const int64 bpOfs = bpBase + (int64)bp * sizeof(BodyPartHeader_t);
    const BodyPartHeader_t *pVtxBodyPart = (const BodyPartHeader_t *)(pBase + bpOfs);
    const int64 modelBase = bpOfs + pVtxBodyPart->modelOffset;
    PROPLM_VTX_RANGE(modelBase, pVtxBodyPart->numModels, ModelHeader_t, "model table");
    int modelCount = MIN(pBodyPart->nummodels, pVtxBodyPart->numModels);
    // Model 0 only: that is the default bodygroup, util.GetModelMeshes(m, 0, 0).
    if (!bAllModels && modelCount > 1)
      modelCount = 1;
    for (int mi = 0; mi < modelCount; ++mi) {
      mstudiomodel_t *pStudioModel = pBodyPart->pModel(mi);
      const int64 mOfs = modelBase + (int64)mi * sizeof(ModelHeader_t);
      const ModelHeader_t *pVtxModel = (const ModelHeader_t *)(pBase + mOfs);
      if (lod < 0 || pVtxModel->numLODs <= lod)
        continue;
      const int64 lodBase = mOfs + pVtxModel->lodOffset;
      PROPLM_VTX_RANGE(lodBase, pVtxModel->numLODs, ModelLODHeader_t, "LOD table");
      const int64 lodOfs = lodBase + (int64)lod * sizeof(ModelLODHeader_t);
      const ModelLODHeader_t *pVtxLod = (const ModelLODHeader_t *)(pBase + lodOfs);
      const int64 meshBase = lodOfs + pVtxLod->meshOffset;
      PROPLM_VTX_RANGE(meshBase, pVtxLod->numMeshes, MeshHeader_t, "mesh table");

      // vertexindex is a byte offset into the model's vertex block.
      const int modelVertexBase =
          pStudioModel->vertexindex / (int)sizeof(mstudiovertex_t);

      for (int meshIdx = 0;
           meshIdx < pStudioModel->nummeshes && meshIdx < pVtxLod->numMeshes;
           ++meshIdx) {
        mstudiomesh_t *pMesh = pStudioModel->pMesh(meshIdx);
        const int64 meshOfs = meshBase + (int64)meshIdx * sizeof(MeshHeader_t);
        const MeshHeader_t *pVtxMesh = (const MeshHeader_t *)(pBase + meshOfs);
        const int64 sgBase = meshOfs + pVtxMesh->stripGroupHeaderOffset;
        PROPLM_VTX_RANGE(sgBase, pVtxMesh->numStripGroups, StripGroupHeader_t,
                         "strip group table");
        const int meshId = meshCounter++;
        if (pMeshes) {
          ProplmVtxMesh_t visited;
          visited.bodyPart = bp;
          visited.model = mi;
          visited.mesh = meshIdx;
          visited.material = pMesh->material;
          pMeshes->AddToTail(visited);
        }

        for (int sg = 0; sg < pVtxMesh->numStripGroups; ++sg) {
          const int64 sgOfs = sgBase + (int64)sg * sizeof(StripGroupHeader_t);
          const StripGroupHeader_t *pStripGroup =
              (const StripGroupHeader_t *)(pBase + sgOfs);
          PROPLM_VTX_RANGE(sgOfs + pStripGroup->vertOffset, pStripGroup->numVerts,
                           Vertex_t, "strip group vertex table");
          PROPLM_VTX_RANGE(sgOfs + pStripGroup->indexOffset, pStripGroup->numIndices,
                           unsigned short, "strip group index table");
          PROPLM_VTX_RANGE(sgOfs + pStripGroup->stripOffset, pStripGroup->numStrips,
                           StripHeader_t, "strip table");
          const Vertex_t *pVerts =
              (const Vertex_t *)(pBase + sgOfs + pStripGroup->vertOffset);
          const unsigned short *pIndices =
              (const unsigned short *)(pBase + sgOfs + pStripGroup->indexOffset);
          const StripHeader_t *pStrips =
              (const StripHeader_t *)(pBase + sgOfs + pStripGroup->stripOffset);
          for (int s = 0; s < pStripGroup->numStrips; ++s) {
            const StripHeader_t *pStrip = &pStrips[s];
            if (!(pStrip->flags & STRIP_IS_TRILIST)) {
              if (bRequireTrilists) {
                V_snprintf(pErr, errLen,
                           "LOD %d has a strip that is not a triangle list", lod);
                return false;
              }
              continue;
            }
            if (pStrip->indexOffset < 0 || pStrip->numIndices < 0 ||
                (int64)pStrip->indexOffset + pStrip->numIndices >
                    pStripGroup->numIndices) {
              V_snprintf(pErr, errLen, "VTX strip indices are out of range");
              return false;
            }

            for (int idx = 0; idx + 2 < pStrip->numIndices; idx += 3) {
              ProplmVtxTri_t tri;
              tri.mesh = meshId;
              for (int vi = 0; vi < 3; ++vi) {
                const int globalIdx = pStrip->indexOffset + idx + vi;
                const unsigned short vtxVertIdx = pIndices[globalIdx];
                if (vtxVertIdx >= pStripGroup->numVerts) {
                  V_snprintf(pErr, errLen, "VTX vertex %d out of range",
                             (int)vtxVertIdx);
                  return false;
                }
                const Vertex_t *pVtxVert = &pVerts[vtxVertIdx];
                const int64 vvdIdx = (int64)modelVertexBase + pMesh->vertexoffset +
                                     pVtxVert->origMeshVertID;
                if (vvdIdx < 0 || vvdIdx >= vertCount) {
                  V_snprintf(pErr, errLen,
                             "VVD vertex %lld out of range (0..%d)",
                             (long long)vvdIdx, vertCount);
                  return false;
                }
                tri.vert[vi] = (int)vvdIdx;
              }
              tris.AddToTail(tri);
            }
          }
        }
      }
    }
  }
#undef PROPLM_VTX_RANGE
  return true;
}

//-----------------------------------------------------------------------------
// Walks LOD0 of model 0 of every bodypart (the default bodygroup) and records
// each trilist triangle in VTX order with its skin family 0 material.
//-----------------------------------------------------------------------------
static bool GatherLod0Triangles(Model_t *pModel, studiohdr_t *pStudioHdr,
                                const CUtlBuffer &vtxBuf, int vertCount,
                                CUtlVector<SrcTri_t> &tris, char *pErr,
                                int errLen) {
  CUtlVector<ProplmVtxMesh_t> meshes;
  CUtlVector<ProplmVtxTri_t> walked;
  if (!Proplm_GatherVtxTriangles(pStudioHdr, vtxBuf, vertCount, 0, false,
                                 false, &meshes, walked, pErr, errLen))
    return false;

  // Materials are registered per mesh in walk order, meshes without trilist
  // triangles included, as the walk did before it was shared.
  CUtlVector<int> meshMat;
  meshMat.SetCount(meshes.Count());
  for (int m = 0; m < meshes.Count(); ++m) {
    // mesh->material indexes the skin table; use skin family 0.
    int texIdx = meshes[m].material;
    if (pStudioHdr->numskinref > 0 && texIdx >= 0 &&
        texIdx < pStudioHdr->numskinref && pStudioHdr->numskinfamilies > 0) {
      texIdx = *pStudioHdr->pSkinref(texIdx);
    }
    const char *pMaterialName = "default";
    if (texIdx >= 0 && texIdx < pStudioHdr->numtextures) {
      pMaterialName = pStudioHdr->pTexture(texIdx)->pszName();
    }
    meshMat[m] = FindOrAddMaterial(pModel, pMaterialName);
  }

  tris.EnsureCapacity(walked.Count());
  for (int t = 0; t < walked.Count(); ++t) {
    SrcTri_t tri;
    tri.mat = meshMat[walked[t].mesh];
    for (int c = 0; c < 3; ++c)
      tri.vert[c] = walked[t].vert[c];
    tris.AddToTail(tri);
  }

  if (tris.Count() == 0) {
    V_snprintf(pErr, errLen, "no LOD0 triangles");
    return false;
  }
  return true;
}

//-----------------------------------------------------------------------------
// xatlas: charts once, then packs into exactly resolution x resolution.
//
// A first pack with texelsPerUnit 0 only supplies xatlas's own estimate and
// the size it actually needed. xatlas does not keep that result to a requested
// resolution (the atlas comes out non-square and may exceed it), so the final
// pack is done with an explicit texelsPerUnit, shrunk until it fits one
// resolution x resolution atlas: uniform texel density, padding intact.
// Returns false when the charts cannot fit this resolution however small they
// get (every chart costs a few texels of padding, so a model with very many
// charts needs a bigger atlas); the caller then tries a larger one.
//-----------------------------------------------------------------------------
static bool FitAtlas(xatlas::Atlas *atlas, int resolution) {
  xatlas::PackOptions packOptions;
  packOptions.padding = PROPLM_XATLAS_PADDING;
  packOptions.bilinear = true;
  packOptions.createImage = false;
  packOptions.resolution = (uint32_t)resolution;
  packOptions.texelsPerUnit = 0.0f;

  // Pass 1: estimate.
  xatlas::PackCharts(atlas, packOptions);
  if (atlas->meshCount != 1 || atlas->chartCount == 0 || atlas->width == 0 ||
      atlas->height == 0 || atlas->texelsPerUnit <= 0.0f)
    return false;
  const float largest = (float)(atlas->width > atlas->height ? atlas->width
                                                              : atlas->height);
  float texelsPerUnit = atlas->texelsPerUnit * (float)resolution / largest;

  // Pass 2: exact fit.
  for (int attempt = 0; attempt < PROPLM_FIT_ATTEMPTS; ++attempt) {
    packOptions.texelsPerUnit = texelsPerUnit;
    xatlas::PackCharts(atlas, packOptions);
    if (atlas->meshCount == 1 && atlas->atlasCount == 1 &&
        atlas->width == (uint32_t)resolution &&
        atlas->height == (uint32_t)resolution)
      return true;
    texelsPerUnit *= 0.97f;
  }
  return false;
}

// Unwraps the mesh and packs it at resolution, or at the next power of two up
// to PROPLM_RES_MAX that fits. On success *pFinalRes is the resolution used.
static bool UnwrapWithXatlas(const CUtlVector<float> &pos,
                             const CUtlVector<float> &nrm,
                             const CUtlVector<uint32_t> &idx, int resolution,
                             int *pFinalRes, xatlas::Atlas **ppAtlas,
                             char *pErr, int errLen) {
  xatlas::Atlas *atlas = xatlas::Create();

  xatlas::MeshDecl decl;
  decl.vertexCount = pos.Count() / 3;
  decl.vertexPositionData = pos.Base();
  decl.vertexPositionStride = 3 * sizeof(float);
  decl.vertexNormalData = nrm.Base();
  decl.vertexNormalStride = 3 * sizeof(float);
  decl.indexData = idx.Base();
  decl.indexCount = idx.Count();
  decl.indexFormat = xatlas::IndexFormat::UInt32;
  const xatlas::AddMeshError addErr = xatlas::AddMesh(atlas, decl);
  if (addErr != xatlas::AddMeshError::Success) {
    V_snprintf(pErr, errLen, "xatlas AddMesh failed: %s",
               xatlas::StringForEnum(addErr));
    xatlas::Destroy(atlas);
    return false;
  }
  xatlas::AddMeshJoin(atlas);

  xatlas::ChartOptions chartOptions;
  chartOptions.maxIterations = PROPLM_XATLAS_ITERATIONS;
  xatlas::ComputeCharts(atlas, chartOptions);

  for (int res = resolution; res <= PROPLM_RES_MAX; res *= 2) {
    if (FitAtlas(atlas, res)) {
      *pFinalRes = res;
      *ppAtlas = atlas;
      return true;
    }
  }

  V_snprintf(pErr, errLen, "xatlas could not fit the charts into %dx%d..%dx%d",
             resolution, resolution, PROPLM_RES_MAX, PROPLM_RES_MAX);
  xatlas::Destroy(atlas);
  return false;
}

//-----------------------------------------------------------------------------
// Builds pModel from the files. Returns false with a reason in pErr.
//-----------------------------------------------------------------------------
static bool BuildModel(Model_t *pModel, const char *pModelPath, int requestedRes,
                       char *pErr, int errLen) {
  // MDL. Not LoadStudioModel(): that insists on $staticprop, which a model
  // that ends up on a prop_dynamic does not need.
  CUtlBuffer mdlBuf;
  if (!ReadWholeFile(pModelPath, mdlBuf)) {
    V_snprintf(pErr, errLen, "cannot read the MDL");
    return false;
  }
  if (mdlBuf.TellPut() < (int)sizeof(studiohdr_t) ||
      strncmp((const char *)mdlBuf.Base(), "IDST", 4)) {
    V_snprintf(pErr, errLen, "not a studio model");
    return false;
  }
  studiohdr_t *pStudioHdr = (studiohdr_t *)mdlBuf.Base();
  if (!Proplm_ConvertMdl(pStudioHdr, mdlBuf.TellPut(), pErr, errLen))
    return false;
  if (!Proplm_ValidateMdl(pStudioHdr, mdlBuf.TellPut(), pErr, errLen))
    return false;

  char path[MAX_PATH];

  // VVD.
  V_strncpy(path, pModelPath, sizeof(path));
  V_SetExtension(path, ".vvd", sizeof(path));
  CUtlBuffer vvdBuf;
  if (!ReadWholeFile(path, vvdBuf)) {
    V_snprintf(pErr, errLen, "cannot read %s", path);
    return false;
  }

  // VTX: DX9 first, DX8 as the fallback.
  CUtlBuffer vtxBuf;
  V_strncpy(path, pModelPath, sizeof(path));
  V_SetExtension(path, ".dx90.vtx", sizeof(path));
  if (!ReadWholeFile(path, vtxBuf)) {
    V_strncpy(path, pModelPath, sizeof(path));
    V_SetExtension(path, ".dx80.vtx", sizeof(path));
    if (!ReadWholeFile(path, vtxBuf)) {
      V_snprintf(pErr, errLen, "no .dx90.vtx or .dx80.vtx");
      return false;
    }
  }

  CUtlVector<Vert_t> verts;
  if (!Proplm_LoadVvdVertices(vvdBuf, pStudioHdr->checksum, 0.0f, verts, pErr,
                              errLen))
    return false;

  CUtlVector<SrcTri_t> srcTris;
  if (!GatherLod0Triangles(pModel, pStudioHdr, vtxBuf, verts.Count(), srcTris,
                           pErr, errLen))
    return false;

  // Compact the vertices the triangles use into the arrays xatlas wants.
  CUtlVector<int> remap;
  remap.SetCount(verts.Count());
  for (int i = 0; i < remap.Count(); ++i)
    remap[i] = -1;
  CUtlVector<float> xaPos, xaNrm;
  CUtlVector<uint32_t> xaIdx;
  xaIdx.EnsureCapacity(srcTris.Count() * 3);
  int usedVerts = 0;
  for (int t = 0; t < srcTris.Count(); ++t) {
    for (int c = 0; c < 3; ++c) {
      const int v = srcTris[t].vert[c];
      if (remap[v] < 0) {
        remap[v] = usedVerts++;
        const Vert_t &src = verts[v];
        for (int k = 0; k < 3; ++k) {
          xaPos.AddToTail(src.pos[k]);
          xaNrm.AddToTail(src.nrm[k]);
        }
      }
      xaIdx.AddToTail((uint32_t)remap[v]);
    }
  }

  xatlas::Atlas *atlas = NULL;
  int resolution = requestedRes;
  if (!UnwrapWithXatlas(xaPos, xaNrm, xaIdx, requestedRes, &resolution, &atlas,
                        pErr, errLen))
    return false;
  pModel->atlasRes = resolution;

  // xatlas keeps the input triangle order: output triangle t is input
  // triangle t. A triangle xatlas could not chart (zero area, collapsed
  // vertices) has chartIndex -1 on its output vertices and is dropped here.
  const xatlas::Mesh &out = atlas->meshes[0];
  if (out.indexCount != (uint32_t)srcTris.Count() * 3) {
    V_snprintf(pErr, errLen, "xatlas returned %u indices for %d triangles",
               out.indexCount, srcTris.Count());
    xatlas::Destroy(atlas);
    return false;
  }

  const float invW = 1.0f / (float)atlas->width;
  const float invH = 1.0f / (float)atlas->height;
  pModel->tris.EnsureCapacity(srcTris.Count());
  int dropped = 0;
  int clamped = 0;
  for (int t = 0; t < srcTris.Count(); ++t) {
    bool valid = true;
    float uv2[3][2];
    for (int c = 0; c < 3; ++c) {
      const uint32_t oi = out.indexArray[t * 3 + c];
      if (oi >= out.vertexCount) {
        valid = false;
        break;
      }
      const xatlas::Vertex &ov = out.vertexArray[oi];
      if (ov.chartIndex < 0 || ov.atlasIndex != 0) {
        valid = false;
        break;
      }
      for (int k = 0; k < 2; ++k) {
        float u = ov.uv[k] * (k == 0 ? invW : invH);
        if (!(u >= 0.0f && u <= 1.0f)) {
          // NaN or out of range; xatlas keeps charts inside the atlas, so
          // this is float noise at the edge. Clamp and count it.
          u = u != u ? 0.0f : (u < 0.0f ? 0.0f : 1.0f);
          ++clamped;
        }
        uv2[c][k] = u;
      }
    }
    if (!valid) {
      ++dropped;
      continue;
    }

    ProplmTri_t tri;
    memset(&tri, 0, sizeof(tri));
    tri.m_MaterialIndex = srcTris[t].mat;
    for (int c = 0; c < 3; ++c) {
      const Vert_t &src = verts[srcTris[t].vert[c]];
      ProplmCorner_t &dst = tri.m_Corner[c];
      memcpy(dst.m_Pos, src.pos, sizeof(dst.m_Pos));
      const float len = sqrtf(src.nrm[0] * src.nrm[0] + src.nrm[1] * src.nrm[1] +
                              src.nrm[2] * src.nrm[2]);
      const float inv = len > 1e-6f ? 1.0f / len : 0.0f;
      dst.m_Normal[0] = src.nrm[0] * inv;
      dst.m_Normal[1] = src.nrm[1] * inv;
      dst.m_Normal[2] = len > 1e-6f ? src.nrm[2] * inv : 1.0f;
      memcpy(dst.m_UV0, src.uv, sizeof(dst.m_UV0));
      dst.m_UV2[0] = uv2[c][0];
      dst.m_UV2[1] = uv2[c][1];
    }
    pModel->tris.AddToTail(tri);
  }
  const float utilization =
      atlas->utilization ? atlas->utilization[0] * 100.0f : 0.0f;
  const int chartCount = (int)atlas->chartCount;
  xatlas::Destroy(atlas);

  if (pModel->tris.Count() == 0) {
    V_snprintf(pErr, errLen, "xatlas charted no triangle");
    return false;
  }

  // Stale-geometry checksum: CRC-32 over the quantized corner positions of the
  // kept triangles, in lump order (docs/proplm.md).
  CUtlVector<int32_t> quant;
  quant.EnsureCapacity(pModel->tris.Count() * 9);
  for (int t = 0; t < pModel->tris.Count(); ++t) {
    for (int c = 0; c < 3; ++c) {
      for (int k = 0; k < 3; ++k)
        quant.AddToTail(QuantizePos(pModel->tris[t].m_Corner[c].m_Pos[k]));
    }
  }
  CRC32_t crc;
  CRC32_Init(&crc);
  CRC32_ProcessBuffer(&crc, quant.Base(), quant.Count() * (int)sizeof(int32_t));
  CRC32_Final(&crc);
  pModel->posChecksum = crc;

  Msg("proplm: %s: %d triangles (%d of %d LOD0 triangles dropped as "
      "degenerate), %d materials, %d charts, %dx%d atlas%s, %.1f%% used%s\n",
      pModelPath, pModel->tris.Count(), dropped, srcTris.Count(),
      pModel->materials.Count(), chartCount, resolution, resolution,
      resolution != requestedRes ? " (charts did not fit the requested size)"
                                 : "",
      utilization, clamped ? " (UV2 clamped)" : "");
  return true;
}

//-----------------------------------------------------------------------------
// Returns the model, built on first use. A model that cannot be processed is
// remembered with failed set and the reason in Model_t::reason; the caller
// reports it, once per prop. The first requesting instance's resolution is the
// atlas resolution.
//-----------------------------------------------------------------------------
static Model_t *GetModel(const char *pModelName, int resolution) {
  char lower[MAX_PATH];
  V_strncpy(lower, pModelName, sizeof(lower));
  V_FixSlashes(lower, '/');
  V_strlower(lower);

  if (Model_t *pFound = FindModel(lower))
    return pFound;

  Model_t *pModel = new Model_t;
  V_strncpy(pModel->key, lower, sizeof(pModel->key));
  pModel->failed = true;
  pModel->reason[0] = '\0';
  pModel->atlasRes = 0;
  pModel->posChecksum = 0;
  s_Models.AddToTail(pModel);

  if (V_strlen(lower) >= PROPLM_MODEL_NAME_LEN) {
    V_snprintf(pModel->reason, sizeof(pModel->reason),
               "the model name is longer than %d characters",
               PROPLM_MODEL_NAME_LEN - 1);
    return pModel;
  }

  if (!BuildModel(pModel, lower, resolution, pModel->reason,
                  sizeof(pModel->reason))) {
    // Drop partial results; keep the entry so the failure is remembered.
    pModel->materials.Purge();
    pModel->tris.Purge();
    return pModel;
  }
  pModel->failed = false;
  return pModel;
}

//-----------------------------------------------------------------------------
// Public: entity conversion
//-----------------------------------------------------------------------------
bool Proplm_ConvertStaticProp(entity_t *pEnt) {
  char modelName[MAX_PATH];
  V_strncpy(modelName, ValueForKey(pEnt, "model"), sizeof(modelName));
  if (!modelName[0]) {
    Warning("Warning: proplm: prop at (%g %g %g) has no model; it stays a "
            "static prop.\n",
            pEnt->origin.x, pEnt->origin.y, pEnt->origin.z);
    return false;
  }

  // The prop's lightmapresolutionx/y give the lightmap size: the larger one,
  // square. Missing or zero in both means the default. It is rounded up to a
  // power of two because VRAD writes it out as a VTF.
  int resolution = PROPLM_RES_DEFAULT;
  const int resX = IntForKey(pEnt, "lightmapresolutionx");
  const int resY = IntForKey(pEnt, "lightmapresolutiony");
  const int larger = resX > resY ? resX : resY;
  if (larger > 0)
    resolution = larger;
  if (resolution < PROPLM_RES_MIN)
    resolution = PROPLM_RES_MIN;
  if (resolution > PROPLM_RES_MAX)
    resolution = PROPLM_RES_MAX;
  int pow2 = PROPLM_RES_MIN;
  while (pow2 < resolution)
    pow2 *= 2;
  resolution = pow2;

  const Model_t *pBuilt = GetModel(modelName, resolution);
  if (pBuilt->failed) {
    Warning("Warning: proplm: prop \"%s\" at (%g %g %g) stays a static prop: "
            "%s.\n",
            modelName, pEnt->origin.x, pEnt->origin.y, pEnt->origin.z,
            pBuilt->reason);
    return false;
  }

  // All instances share one layout, packed with its chart padding counted in
  // texels of atlasRes (the first instance's size, or larger when the charts
  // did not fit). A smaller lightmap would put neighbouring charts less than a
  // texel apart, so no instance goes below it.
  if (pBuilt->atlasRes > resolution)
    resolution = pBuilt->atlasRes;

  // VRAD lights a converted prop at its own triangles, never at an info target.
  if (ValueForKey(pEnt, "lightingorigin")[0]) {
    Warning("Warning: proplm: prop \"%s\" at (%g %g %g): lightingorigin \"%s\" "
            "is ignored on a converted prop.\n",
            modelName, pEnt->origin.x, pEnt->origin.y, pEnt->origin.z,
            ValueForKey(pEnt, "lightingorigin"));
  }

  const int propId = s_NextPropId++;
  char value[MAX_PATH];

  // Collision: read the way the static prop path reads it, IntForKey(solid),
  // where a missing key is 0 (Not solid). Not solid stays "0"; any other value
  // becomes VPhysics.
  const bool bNotSolid = IntForKey(pEnt, "solid") == 0;

  SetKeyValue(pEnt, "classname", "prop_dynamic");
  SetKeyValue(pEnt, "solid", bNotSolid ? "0" : "6");
  SetKeyValue(pEnt, "proplm", "1");
  V_snprintf(value, sizeof(value), "%d", propId);
  SetKeyValue(pEnt, "proplm_id", value);
  V_snprintf(value, sizeof(value), "maps/%s/proplm/%d", mapbase, propId);
  SetKeyValue(pEnt, "proplm_lm", value);
  V_snprintf(value, sizeof(value), "%d", resolution);
  SetKeyValue(pEnt, "proplm_res", value);
  return true;
}

//-----------------------------------------------------------------------------
// Public: -onlyprops guard
//-----------------------------------------------------------------------------
void Proplm_RejectOnlyProps() {
  int marked = 0;
  for (int i = 0; i < num_entities; ++i) {
    const char *pClass = ValueForKey(&entities[i], "classname");
    if ((!strcmp(pClass, "static_prop") || !strcmp(pClass, "prop_static")) &&
        IntForKey(&entities[i], "proplm") == 1)
      ++marked;
  }
  const bool oldLump = g_GameLumps.GetGameLumpHandle(GAMELUMP_PROPLM_ID) !=
                       g_GameLumps.InvalidGameLump();
  if (!marked && !oldLump)
    return;

  // -onlyprops writes the static prop lump but never the entity lump, and a
  // proplm prop only exists as an entity-lump prop_dynamic plus a 'plm2' lump
  // that the two must agree on. Writing either half alone loses the props (new
  // proplm "1": skipped from the static props and never added as an entity) or
  // duplicates them (proplm removed from the VMF: a static prop again beside the
  // old prop_dynamic).
  if (marked) {
    Error("-onlyprops cannot convert proplm props: %d prop(s) in the VMF have "
          "proplm \"1\", and the converted prop_dynamic entities live in the "
          "entity lump, which -onlyprops does not rewrite. Run a full compile or "
          "-onlyents instead.\n",
          marked);
  } else {
    Error("-onlyprops cannot be used on a map that was compiled with proplm: "
          "the BSP holds a 'plm2' game lump and prop_dynamic entities from the "
          "earlier compile, and -onlyprops would turn the VMF's props back into "
          "static props beside them. Run a full compile or -onlyents instead.\n");
  }
}

//-----------------------------------------------------------------------------
// Public: lump
//-----------------------------------------------------------------------------
void Proplm_WriteGameLump() {
  // Replace whatever a previous compile left (-onlyents / -onlyprops).
  GameLumpHandle_t handle = g_GameLumps.GetGameLumpHandle(GAMELUMP_PROPLM_ID);
  if (handle != g_GameLumps.InvalidGameLump())
    g_GameLumps.DestroyGameLump(handle);

  int modelCount = 0;
  size_t size = sizeof(ProplmHeader_t);
  for (int m = 0; m < s_Models.Count(); ++m) {
    const Model_t *pModel = s_Models[m];
    if (pModel->failed)
      continue;
    ++modelCount;
    size += sizeof(ProplmModel_t) +
            pModel->materials.Count() * sizeof(ProplmMaterial_t) +
            pModel->tris.Count() * sizeof(ProplmTri_t);
  }
  if (modelCount == 0)
    return;

  // Flags 0: this lump is never compressed (see CompressGameLump).
  handle = g_GameLumps.CreateGameLump(GAMELUMP_PROPLM_ID, (int)size, 0,
                                      GAMELUMP_PROPLM_VERSION);
  byte *pOut = (byte *)g_GameLumps.GetGameLump(handle);
  memset(pOut, 0, size);

  ProplmHeader_t header;
  header.m_Version = GAMELUMP_PROPLM_VERSION;
  header.m_ModelCount = modelCount;
  memcpy(pOut, &header, sizeof(header));
  pOut += sizeof(header);

  int totalTris = 0;
  for (int m = 0; m < s_Models.Count(); ++m) {
    const Model_t *pModel = s_Models[m];
    if (pModel->failed)
      continue;

    ProplmModel_t entry;
    memset(&entry, 0, sizeof(entry));
    V_strncpy(entry.m_Name, pModel->key, sizeof(entry.m_Name));
    entry.m_MaterialCount = pModel->materials.Count();
    entry.m_TriCount = pModel->tris.Count();
    entry.m_PosChecksum = pModel->posChecksum;
    memcpy(pOut, &entry, sizeof(entry));
    pOut += sizeof(entry);

    const int matBytes = pModel->materials.Count() * (int)sizeof(ProplmMaterial_t);
    memcpy(pOut, pModel->materials.Base(), matBytes);
    pOut += matBytes;

    const int triBytes = pModel->tris.Count() * (int)sizeof(ProplmTri_t);
    memcpy(pOut, pModel->tris.Base(), triBytes);
    pOut += triBytes;
    totalTris += pModel->tris.Count();
  }

  Msg("proplm: wrote 'plm2' lump: %d model(s), %d triangles, %d bytes; %d "
      "prop(s) converted to prop_dynamic.\n",
      modelCount, totalTris, (int)size, s_NextPropId);
}
