//========= Copyright Valve Corporation, All rights reserved. ============//
//
// Purpose: Places "detail" objects which are client-only renderable things
//
// $Revision: $
// $NoKeywords: $
//=============================================================================//

#include "CModel.h"
#include "CollisionUtils.h"
#include "KeyValues.h"
#include "PhysDll.h"
#include "Studio.h"
#include "UtlBuffer.h"
#include "VPhysics_Interface.h"
#include "bspfile.h"
#include "bsplib.h"
#include "byteswap.h"
#include "gamebspfile.h"
#include "phyfile.h"
#include "tier1/strtools.h"
#include "utldict.h"
#include "utlsymbol.h"
#include "utlvector.h"
#include "vbsp.h"
#include <float.h>

#ifdef _WIN32
#include <windows.h>
#endif

static void SetCurrentModel(studiohdr_t *pStudioHdr);
static void FreeCurrentModelVertexes();

IPhysicsCollision *s_pPhysCollision = NULL;

//-----------------------------------------------------------------------------
// These puppies are used to construct the game lumps
//-----------------------------------------------------------------------------
static CUtlVector<StaticPropDictLump_t> s_StaticPropDictLump;
static CUtlVector<StaticPropLump_t> s_StaticPropLump;
static CUtlVector<StaticPropLeafLump_t> s_StaticPropLeafLump;

//-----------------------------------------------------------------------------
// Used to build the static prop
//-----------------------------------------------------------------------------
struct StaticPropBuild_t {
  char const *m_pModelName;
  char const *m_pLightingOrigin;
  Vector m_Origin;
  QAngle m_Angles;
  int m_Solid;
  int m_Skin;
  int m_Flags;
  float m_FadeMinDist;
  float m_FadeMaxDist;
  bool m_FadesOut;
  float m_flForcedFadeScale;
  unsigned short m_nMinDXLevel;
  unsigned short m_nMaxDXLevel;
  int m_LightmapResolutionX;
  int m_LightmapResolutionY;
};

//-----------------------------------------------------------------------------
// Used to cache collision model generation
//-----------------------------------------------------------------------------
struct ModelCollisionLookup_t {
  CUtlSymbol m_Name;
  CPhysCollide *m_pCollide;
};

static bool ModelLess(ModelCollisionLookup_t const &src1,
                      ModelCollisionLookup_t const &src2) {
  return src1.m_Name < src2.m_Name;
}

static CUtlRBTree<ModelCollisionLookup_t, unsigned short>
    s_ModelCollisionCache(0, 32, ModelLess);
static CUtlVector<int> s_LightingInfo;

// Maps original model name -> patched model name for
// -forcedynamicpropsasstatic. The patched model lives under maps/<mapname>/
// with a _static suffix, mirroring how WVT/cubemap patches work.
static CUtlDict<const char *, int> s_PatchedModelNames;

//-----------------------------------------------------------------------------
// Gets the keyvalues from a studiohdr
//-----------------------------------------------------------------------------
bool StudioKeyValues(studiohdr_t *pStudioHdr, KeyValues *pValue) {
  if (!pStudioHdr)
    return false;

  return pValue->LoadFromBuffer(pStudioHdr->pszName(),
                                pStudioHdr->KeyValueText());
}

//-----------------------------------------------------------------------------
// Makes sure the studio model is a static prop
//-----------------------------------------------------------------------------
enum isstaticprop_ret {
  RET_VALID,
  RET_FAIL_NOT_MARKED_STATIC_PROP,
  RET_FAIL_DYNAMIC,
};

isstaticprop_ret IsStaticProp(studiohdr_t *pHdr) {
  if (!(pHdr->flags & STUDIOHDR_FLAGS_STATIC_PROP)) {
    if (g_bForceDynamicPropsAsStatic) {
      Warning("Warning: Forcing non-$staticprop model \"%s\" as static prop, "
              "collision and rendering may be incorrect.\n",
              pHdr->pszName());
    } else {
      return RET_FAIL_NOT_MARKED_STATIC_PROP;
    }
  }

  if (g_bAllowDynamicPropsAsStatic)
    return RET_VALID;

  // If it's got a propdata section in the model's keyvalues, it's not allowed
  // to be a prop_static
  KeyValues *modelKeyValues = new KeyValues(pHdr->pszName());
  if (StudioKeyValues(pHdr, modelKeyValues)) {
    KeyValues *sub = modelKeyValues->FindKey("prop_data");
    if (sub) {
      if (!(sub->GetInt("allowstatic", 0))) {
        modelKeyValues->deleteThis();
        return RET_FAIL_DYNAMIC;
      }
    }
  }
  modelKeyValues->deleteThis();

  return RET_VALID;
}

//-----------------------------------------------------------------------------
// Add static prop model to the list of models
//-----------------------------------------------------------------------------

static int AddStaticPropDictLump(char const *pModelName) {
  StaticPropDictLump_t dictLump;
  strncpy(dictLump.m_Name, pModelName, DETAIL_NAME_LENGTH);

  for (int i = s_StaticPropDictLump.Size(); --i >= 0;) {
    if (!memcmp(&s_StaticPropDictLump[i], &dictLump, sizeof(dictLump)))
      return i;
  }

  return s_StaticPropDictLump.AddToTail(dictLump);
}

//-----------------------------------------------------------------------------
// Load studio model vertex data from a file...
//-----------------------------------------------------------------------------
bool LoadStudioModel(char const *pModelName, char const *pEntityType,
                     CUtlBuffer &buf) {
  if (!g_pFullFileSystem->ReadFile(pModelName, NULL, buf))
    return false;

  // Check that it's valid
  if (strncmp((const char *)buf.PeekGet(), "IDST", 4) &&
      strncmp((const char *)buf.PeekGet(), "IDAG", 4)) {
    return false;
  }

  studiohdr_t *pHdr = (studiohdr_t *)buf.PeekGet();

  Studio_ConvertStudioHdrToNewVersion(pHdr);

  if (pHdr->version != STUDIO_VERSION) {
    return false;
  }

  isstaticprop_ret isStaticProp = IsStaticProp(pHdr);
  if (isStaticProp != RET_VALID) {
    if (isStaticProp == RET_FAIL_NOT_MARKED_STATIC_PROP) {
      Warning("Error! To use model \"%s\"\n"
              "      with %s, it must be compiled with $staticprop!\n",
              pModelName, pEntityType);
    } else if (isStaticProp == RET_FAIL_DYNAMIC) {
      Warning("Error! %s using model \"%s\", which must be used on a dynamic "
              "entity (i.e. prop_physics). Deleted.\n",
              pEntityType, pModelName);
    }
    return false;
  }

  // ensure reset
  pHdr->SetVertexBase(NULL);
  pHdr->SetIndexBase(NULL);

  return true;
}

//-----------------------------------------------------------------------------
// Computes a convex hull from a studio mesh
//-----------------------------------------------------------------------------
static CPhysConvex *ComputeConvexHull(mstudiomesh_t *pMesh) {
  // Generate a list of all verts in the mesh
  Vector **ppVerts =
      (Vector **)stackalloc(pMesh->numvertices * sizeof(Vector *));
  const mstudio_meshvertexdata_t *vertData = pMesh->GetVertexData();
  Assert(vertData); // This can only return NULL on X360 for now
  for (int i = 0; i < pMesh->numvertices; ++i) {
    ppVerts[i] = vertData->Position(i);
  }

  // Generate a convex hull from the verts
  return s_pPhysCollision->ConvexFromVerts(ppVerts, pMesh->numvertices);
}

//-----------------------------------------------------------------------------
// Computes a convex hull from the studio model
//-----------------------------------------------------------------------------
CPhysCollide *ComputeConvexHull(studiohdr_t *pStudioHdr) {
  CUtlVector<CPhysConvex *> convexHulls;

  for (int body = 0; body < pStudioHdr->numbodyparts; ++body) {
    mstudiobodyparts_t *pBodyPart = pStudioHdr->pBodypart(body);
    for (int model = 0; model < pBodyPart->nummodels; ++model) {
      mstudiomodel_t *pStudioModel = pBodyPart->pModel(model);
      for (int mesh = 0; mesh < pStudioModel->nummeshes; ++mesh) {
        // Make a convex hull for each mesh
        // NOTE: This won't work unless the model has been compiled
        // with $staticprop
        mstudiomesh_t *pStudioMesh = pStudioModel->pMesh(mesh);
        convexHulls.AddToTail(ComputeConvexHull(pStudioMesh));
      }
    }
  }

  // Convert an array of convex elements to a compiled collision model
  // (this deletes the convex elements)
  return s_pPhysCollision->ConvertConvexToCollide(convexHulls.Base(),
                                                  convexHulls.Size());
}

//-----------------------------------------------------------------------------
// Add, find collision model in cache
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Decompile-recompile pipeline for forced non-$staticprop models.
// MDL-to-SMD decompilation logic is derived from Crowbar by ZeqMacaw.
// Licensed under Creative Commons Attribution-ShareAlike 3.0 Unported.
// https://github.com/ZeqMacaw/Crowbar
//-----------------------------------------------------------------------------
#include "filesystem_tools.h"
#include "optimize.h"
using namespace OptimizedModel;

//-----------------------------------------------------------------------------
// Writes an SMD reference mesh from MDL/VVD/VTX data.
// Returns true on success.
//-----------------------------------------------------------------------------
static bool WriteSMDFromModel(const char *pSmdPath, studiohdr_t *pStudioHdr,
                              CUtlBuffer &vvdBuf, CUtlBuffer &vtxBuf) {
  FILE *fp = fopen(pSmdPath, "w");
  if (!fp)
    return false;

  vertexFileHeader_t *pVvdHdr = (vertexFileHeader_t *)vvdBuf.Base();
  FileHeader_t *pVtxHdr = (FileHeader_t *)vtxBuf.Base();

  if (!pVvdHdr || !pVtxHdr || pVvdHdr->id != MODEL_VERTEX_FILE_ID) {
    fclose(fp);
    return false;
  }

  mstudiovertex_t *pRawVerts =
      (mstudiovertex_t *)((byte *)pVvdHdr + pVvdHdr->vertexDataStart);

  // Apply VVD LOD fixup table if present.
  // The fixup table remaps raw vertices into the correct order for each LOD.
  // Without this, vertex indices from vertexindex/vertexoffset point to wrong
  // positions in the raw data, causing triangle corruption.
  mstudiovertex_t *pVerts = pRawVerts;
  mstudiovertex_t *pFixedVerts = NULL;
  int numLod0Verts = pVvdHdr->numLODVertexes[0];

  if (pVvdHdr->numFixups > 0) {
    pFixedVerts = new mstudiovertex_t[numLod0Verts];
    vertexFileFixup_t *pFixups =
        (vertexFileFixup_t *)((byte *)pVvdHdr + pVvdHdr->fixupTableStart);

    int target = 0;
    for (int f = 0; f < pVvdHdr->numFixups; ++f) {
      if (pFixups[f].lod < 0)
        continue; // LOD 0: all entries with lod >= 0 apply (always true)

      if (target + pFixups[f].numVertexes > numLod0Verts)
        break; // safety

      memcpy(&pFixedVerts[target], &pRawVerts[pFixups[f].sourceVertexID],
             pFixups[f].numVertexes * sizeof(mstudiovertex_t));
      target += pFixups[f].numVertexes;
    }
    pVerts = pFixedVerts;
    Msg("  Applied VVD fixup: %d fixup entries, %d vertices\n",
        pVvdHdr->numFixups, target);
  }

  // -- Header --
  fprintf(fp, "// SMD generated by VBSP -forcedynamicpropsasstatic\n");
  fprintf(fp, "// Decompilation logic derived from Crowbar by ZeqMacaw\n");
  fprintf(fp, "version 1\n");

  // -- Nodes section (bone hierarchy) --
  fprintf(fp, "nodes\n");
  for (int b = 0; b < pStudioHdr->numbones; ++b) {
    mstudiobone_t *pBone = pStudioHdr->pBone(b);
    fprintf(fp, "  %d \"%s\" %d\n", b, pBone->pszName(), pBone->parent);
  }
  fprintf(fp, "end\n");

  // -- Skeleton section (bind pose) --
  fprintf(fp, "skeleton\n");
  fprintf(fp, "time 0\n");
  for (int b = 0; b < pStudioHdr->numbones; ++b) {
    mstudiobone_t *pBone = pStudioHdr->pBone(b);
    fprintf(fp, "  %d  %f %f %f  %f %f %f\n", b, pBone->pos.x, pBone->pos.y,
            pBone->pos.z, pBone->rot.x, pBone->rot.y, pBone->rot.z);
  }
  fprintf(fp, "end\n");

  // -- Triangles section --
  fprintf(fp, "triangles\n");

  for (int bp = 0; bp < pStudioHdr->numbodyparts && bp < pVtxHdr->numBodyParts;
       ++bp) {
    mstudiobodyparts_t *pBodyPart = pStudioHdr->pBodypart(bp);
    BodyPartHeader_t *pVtxBodyPart = pVtxHdr->pBodyPart(bp);

    for (int m = 0; m < pBodyPart->nummodels && m < pVtxBodyPart->numModels;
         ++m) {
      mstudiomodel_t *pModel = pBodyPart->pModel(m);
      ModelHeader_t *pVtxModel = pVtxBodyPart->pModel(m);

      // Use LOD 0 only
      if (pVtxModel->numLODs < 1)
        continue;
      ModelLODHeader_t *pVtxLod = pVtxModel->pLOD(0);

      // Model's global vertex base in the VVD (per Source SDK)
      int modelVertexBase = pModel->vertexindex / sizeof(mstudiovertex_t);

      for (int meshIdx = 0;
           meshIdx < pModel->nummeshes && meshIdx < pVtxLod->numMeshes;
           ++meshIdx) {
        mstudiomesh_t *pMesh = pModel->pMesh(meshIdx);
        MeshHeader_t *pVtxMesh = pVtxLod->pMesh(meshIdx);

        // Get material name for this mesh
        const char *pMaterialName = "default";
        if (pMesh->material >= 0 && pMesh->material < pStudioHdr->numtextures) {
          pMaterialName = pStudioHdr->pTexture(pMesh->material)->pszName();
        }

        for (int sg = 0; sg < pVtxMesh->numStripGroups; ++sg) {
          StripGroupHeader_t *pStripGroup = pVtxMesh->pStripGroup(sg);

          for (int s = 0; s < pStripGroup->numStrips; ++s) {
            StripHeader_t *pStrip = pStripGroup->pStrip(s);

            if (!(pStrip->flags & STRIP_IS_TRILIST))
              continue;

            for (int idx = 0; idx < pStrip->numIndices - 2; idx += 3) {
              fprintf(fp, "%s\n", pMaterialName);

              // Write 3 vertices per triangle
              for (int vi = 0; vi < 3; ++vi) {
                // VTX winding order: 0, 2, 1 (per Crowbar)
                int windIdx = (vi == 1) ? 2 : (vi == 2) ? 1 : 0;
                int globalIdx = pStrip->indexOffset + idx + windIdx;
                unsigned short vtxVertIdx = *pStripGroup->pIndex(globalIdx);
                Vertex_t *pVtxVert = pStripGroup->pVertex(vtxVertIdx);

                // Global VVD index: model base + mesh offset + vertex ID
                int vvdIdx = modelVertexBase + pMesh->vertexoffset +
                             pVtxVert->origMeshVertID;

                if (vvdIdx < 0 || vvdIdx >= pVvdHdr->numLODVertexes[0]) {
                  // Safety: skip bad indices
                  fprintf(fp,
                          "  0  0.000000 0.000000 0.000000  "
                          "0.000000 0.000000 1.000000  0.000000 0.000000\n");
                  continue;
                }

                mstudiovertex_t &vert = pVerts[vvdIdx];
                int boneIdx = vert.m_BoneWeights.bone[0];

                // Position and normal: keep original space — studiomdl
                // with $staticprop will handle the coordinate conversion
                fprintf(fp, "  %d  %f %f %f  %f %f %f  %f %f", boneIdx,
                        vert.m_vecPosition.x, vert.m_vecPosition.y,
                        vert.m_vecPosition.z, vert.m_vecNormal.x,
                        vert.m_vecNormal.y, vert.m_vecNormal.z,
                        vert.m_vecTexCoord.x, 1.0f - vert.m_vecTexCoord.y);

                // Bone weights
                int numWeights = vert.m_BoneWeights.numbones;
                if (numWeights > 1) {
                  fprintf(fp, " %d", numWeights);
                  for (int bw = 0; bw < numWeights; ++bw) {
                    fprintf(fp, " %d %f", vert.m_BoneWeights.bone[bw],
                            vert.m_BoneWeights.weight[bw]);
                  }
                }
                fprintf(fp, "\n");
              }
            }
          }
        }
      }
    }
  }

  fprintf(fp, "end\n");
  fclose(fp);

  if (pFixedVerts)
    delete[] pFixedVerts;

  return true;
}

//-----------------------------------------------------------------------------
// Generates a minimal QC file for studiomdl with $staticprop.
//-----------------------------------------------------------------------------
static bool GenerateStaticPropQC(const char *pQcPath, const char *pModelName,
                                 const char *pSmdPath, const char *pPhySmdPath,
                                 bool bIsCollisionJoints,
                                 studiohdr_t *pStudioHdr) {
  FILE *fp = fopen(pQcPath, "w");
  if (!fp)
    return false;

  fprintf(fp, "// Auto-generated by VBSP -forcedynamicpropsasstatic\n");
  fprintf(fp, "$staticprop\n");
  fprintf(fp, "$casttextureshadows\n");
  fprintf(fp, "$modelname \"%s\"\n", pModelName);

  // $cdmaterials
  for (int i = 0; i < pStudioHdr->numcdtextures; ++i) {
    fprintf(fp, "$cdmaterials \"%s\"\n", pStudioHdr->pCdtexture(i));
  }

  // $body
  fprintf(fp, "$body \"body\" \"%s\"\n", pSmdPath);

  // $sequence
  fprintf(fp, "$sequence \"idle\" \"%s\" fps 30\n", pSmdPath);

  // Always use $collisionmodel with $concave for $staticprop recompilation
  if (pPhySmdPath) {
    fprintf(fp, "$collisionmodel \"%s\" {\n", pPhySmdPath);
    fprintf(fp, "  $concave\n");
    fprintf(fp, "  $maxconvexpieces 2048\n");
    fprintf(fp, "}\n");
  }

  fclose(fp);
  return true;
}

//-----------------------------------------------------------------------------
// Decompile a PHY file into SMD format using IPhysicsCollision::CreateDebugMesh
// to extract triangle vertices from the collision model.
//-----------------------------------------------------------------------------
static bool DecompilePhyToSmd(const char *pModelName, const char *pSmdPath,
                              studiohdr_t *pStudioHdr,
                              bool *pOutIsCollisionJoints) {
  if (pOutIsCollisionJoints)
    *pOutIsCollisionJoints = false;
  if (!s_pPhysCollision)
    return false;

  // Read the original PHY file
  char phyPath[512];
  V_strncpy(phyPath, pModelName, sizeof(phyPath));
  V_SetExtension(phyPath, ".phy", sizeof(phyPath));

  CUtlBuffer phyBuf;
  if (!g_pFullFileSystem->ReadFile(phyPath, NULL, phyBuf))
    return false;

  int phyLen = phyBuf.TellMaxPut();
  if (phyLen < (int)sizeof(phyheader_t))
    return false;

  phyheader_t *pPhyHdr = (phyheader_t *)phyBuf.Base();
  if (pPhyHdr->solidCount <= 0)
    return false;

  // Get the collision data (after the header)
  const char *pSolidData = (const char *)phyBuf.Base() + pPhyHdr->size;
  int dataSize = phyLen - pPhyHdr->size;

  if (dataSize <= 0)
    return false;

  // Load collision data via VCollideLoad
  vcollide_t collide;
  memset(&collide, 0, sizeof(collide));
  s_pPhysCollision->VCollideLoad(&collide, pPhyHdr->solidCount, pSolidData,
                                 dataSize, false);

  if (collide.solidCount <= 0)
    return false;

  FILE *fp = fopen(pSmdPath, "w");
  if (!fp) {
    s_pPhysCollision->VCollideUnload(&collide);
    return false;
  }

  // Pre-count total convex pieces to know how many bones we need
  int totalConvexPieces = 0;
  for (int s = 0; s < collide.solidCount; ++s) {
    if (!collide.solids[s])
      continue;
    ICollisionQuery *pQuery =
        s_pPhysCollision->CreateQueryModel(collide.solids[s]);
    if (pQuery) {
      totalConvexPieces += pQuery->ConvexCount();
      s_pPhysCollision->DestroyQueryModel(pQuery);
    }
  }

  if (totalConvexPieces <= 0) {
    fclose(fp);
    s_pPhysCollision->VCollideUnload(&collide);
    return false;
  }

  // Apply poseToBone^-1 (= boneToPose) to ALL physics vertices.
  //
  // $collisionjoints: ICollisionQuery returns bone-local HL vertices.
  //   poseToBone^-1 converts them to model space.  Studiomdl then applies
  //   the new $staticprop bone's poseToBone, placing them correctly.
  //
  // $collisionmodel + $staticprop: ICollisionQuery returns model-space HL
  //   vertices.  poseToBone^-1 pre-rotates them to match the $staticprop
  //   visual rotation.  Studiomdl's boneToWorld * poseToBone = identity
  //   preserves this pre-rotation, so collision matches the rotated mesh.

  bool bIsCollisionJoints = (collide.solidCount > 1); // multi-solid = always
  if (bIsCollisionJoints && pOutIsCollisionJoints)
    *pOutIsCollisionJoints = true;
  if (!bIsCollisionJoints && collide.solidCount == 1 && collide.pKeyValues &&
      pStudioHdr) {
    // Parse first solid name from PHY keyvalues.
    // Format: solid { "index" "0" "name" "solid_name" ... }
    const char *pName = strstr(collide.pKeyValues, "\"name\"");
    if (pName) {
      // Skip past "name" and whitespace to the value
      pName += 6; // skip "name"
      while (*pName && (*pName == ' ' || *pName == '\t' || *pName == '"'))
        pName++;
      // Extract the value (until closing quote)
      char solidName[256];
      int i = 0;
      while (*pName && *pName != '"' && i < 255)
        solidName[i++] = *pName++;
      solidName[i] = '\0';

      // Check if this name matches any bone in the model
      for (int b = 0; b < pStudioHdr->numbones; ++b) {
        if (V_stricmp(solidName, pStudioHdr->pBone(b)->pszName()) == 0) {
          bIsCollisionJoints = true;
          if (pOutIsCollisionJoints)
            *pOutIsCollisionJoints = true;
          Msg("  PHY solid name \"%s\" matches bone %d — using "
              "$collisionjoints path\n",
              solidName, b);
          break;
        }
      }
    }
  }

  // Build per-solid poseToBone matrices.
  //
  // For $collisionjoints each solid maps to a DIFFERENT bone — we must
  // use THAT bone's poseToBone, not just bone 0.  The PHY keyvalues
  // contain entries like:  solid { "index" "0" "name" "bone_name" ... }
  //
  // For $collisionmodel (single solid), we still use bone 0.
  const int MAX_SOLIDS = 128;
  matrix3x4_t solidPoseToBone[MAX_SOLIDS];
  int solidBoneIndex[MAX_SOLIDS];
  for (int s = 0; s < MAX_SOLIDS; ++s) {
    SetIdentityMatrix(solidPoseToBone[s]);
    solidBoneIndex[s] = 0;
  }

  if (bIsCollisionJoints && collide.pKeyValues && pStudioHdr) {
    // Parse each "solid" block to find solid index → bone name mapping.
    const char *pCur = collide.pKeyValues;
    while ((pCur = strstr(pCur, "\"index\"")) != NULL) {
      // Extract index value
      pCur += 7; // skip "index"
      while (*pCur && (*pCur == ' ' || *pCur == '\t' || *pCur == '"'))
        pCur++;
      int solidIdx = atoi(pCur);

      // Find the "name" field after this index
      const char *pName = strstr(pCur, "\"name\"");
      if (!pName)
        break;
      pName += 6; // skip "name"
      while (*pName && (*pName == ' ' || *pName == '\t' || *pName == '"'))
        pName++;
      char boneName[256];
      int i = 0;
      while (*pName && *pName != '"' && i < 255)
        boneName[i++] = *pName++;
      boneName[i] = '\0';

      // Find matching bone in the studiohdr
      if (solidIdx >= 0 && solidIdx < MAX_SOLIDS &&
          solidIdx < collide.solidCount) {
        for (int b = 0; b < pStudioHdr->numbones; ++b) {
          if (V_stricmp(boneName, pStudioHdr->pBone(b)->pszName()) == 0) {
            MatrixCopy(pStudioHdr->pBone(b)->poseToBone,
                       solidPoseToBone[solidIdx]);
            solidBoneIndex[solidIdx] = b;
            Msg("    Solid %d → bone %d (\"%s\")\n", solidIdx, b, boneName);
            break;
          }
        }
      }
      pCur = pName;
    }
  } else if (pStudioHdr && pStudioHdr->numbones > 0) {
    // Single-solid $collisionmodel: ICollisionQuery returns model-space
    // vertices directly (studiomdl's ConvertToWorldSpace applies
    // poseToBone then boneToWorld, which cancel out for a root bone).
    // No transform needed — leave solidPoseToBone as identity.
    // solidBoneIndex already defaults to 0.
  }

  // Write SMD header matching reference bone hierarchy
  fprintf(fp, "// Physics SMD decompiled by VBSP\n");
  fprintf(fp, "version 1\n");
  fprintf(fp, "nodes\n");
  if (pStudioHdr && pStudioHdr->numbones > 0) {
    for (int b = 0; b < pStudioHdr->numbones; ++b) {
      mstudiobone_t *pBone = pStudioHdr->pBone(b);
      fprintf(fp, "  %d \"%s\" %d\n", b, pBone->pszName(), pBone->parent);
    }
  } else {
    fprintf(fp, "  0 \"phys_root\" -1\n");
  }
  fprintf(fp, "end\n");
  fprintf(fp, "skeleton\n");
  fprintf(fp, "time 0\n");
  if (pStudioHdr && pStudioHdr->numbones > 0) {
    for (int b = 0; b < pStudioHdr->numbones; ++b) {
      mstudiobone_t *pBone = pStudioHdr->pBone(b);
      fprintf(fp, "  %d  %f %f %f  %f %f %f\n", b, pBone->pos.x, pBone->pos.y,
              pBone->pos.z, pBone->rot.x, pBone->rot.y, pBone->rot.z);
    }
  } else {
    fprintf(fp,
            "  0  0.000000 0.000000 0.000000  0.000000 0.000000 0.000000\n");
  }
  fprintf(fp, "end\n");
  fprintf(fp, "triangles\n");

  int totalTris = 0;
  int totalConvex = 0;
  for (int s = 0; s < collide.solidCount; ++s) {
    if (!collide.solids[s])
      continue;

    // Use ICollisionQuery for exact original hull triangles
    ICollisionQuery *pQuery =
        s_pPhysCollision->CreateQueryModel(collide.solids[s]);
    if (!pQuery)
      continue;

    // Use this solid's specific poseToBone matrix
    matrix3x4_t &curPoseToBone = solidPoseToBone[s < MAX_SOLIDS ? s : 0];

    int numConvex = pQuery->ConvexCount();
    for (int c = 0; c < numConvex; ++c) {
      int boneIdx = solidBoneIndex[s < MAX_SOLIDS ? s : 0];
      int numTris = pQuery->TriangleCount(c);

      if (numTris > 0) {
        Vector *triVerts = new Vector[numTris * 3];
        Vector *triNormals = new Vector[numTris * 3];

        for (int t = 0; t < numTris; ++t) {
          pQuery->GetTriangleVerts(c, t, &triVerts[t * 3]);

          // Apply this solid's poseToBone^-1 to transform to model space.
          // VectorITransform(v, poseToBone) = poseToBone^-1 * v = boneToPose *
          // v
          for (int v = 0; v < 3; ++v) {
            Vector tmp;
            VectorITransform(triVerts[t * 3 + v], curPoseToBone, tmp);

            // For $staticprop $collisionmodel, apply the same (Y, -X, Z)
            // swap that WriteSMDFromModel applies to reference mesh vertices.
            // Without this, physics vertices are rotated 90° around Z
            // relative to the reference mesh.
            if (!bIsCollisionJoints) {
              float oldX = tmp.x;
              tmp.x = tmp.y;
              tmp.y = -oldX;
            }

            triVerts[t * 3 + v] = tmp;
            triNormals[t * 3 + v].Init(); // zero out for accumulation
          }
        }

        // Accumulate smoothed face normals
        for (int t = 0; t < numTris; ++t) {
          Vector &v0 = triVerts[t * 3 + 0];
          Vector &v1 = triVerts[t * 3 + 1];
          Vector &v2 = triVerts[t * 3 + 2];

          // Compute face normal (in model space)
          Vector e1 = v1 - v0;
          Vector e2 = v2 - v0;
          Vector faceNormal = CrossProduct(e1, e2);
          VectorNormalize(faceNormal);

          // Find all vertices that match position to accumulate smooth normals
          for (int i = 0; i < numTris * 3; ++i) {
            if (triVerts[i].DistToSqr(v0) < 1e-4f)
              triNormals[i] += faceNormal;
            if (triVerts[i].DistToSqr(v1) < 1e-4f)
              triNormals[i] += faceNormal;
            if (triVerts[i].DistToSqr(v2) < 1e-4f)
              triNormals[i] += faceNormal;
          }
        }

        for (int t = 0; t < numTris; ++t) {
          fprintf(fp, "phy\n");
          for (int v = 0; v < 3; ++v) {
            Vector pos = triVerts[t * 3 + v];
            Vector norm = triNormals[t * 3 + v];
            VectorNormalize(norm);
            fprintf(fp, "  %d %f %f %f %f %f %f 0 0\n", boneIdx, pos.x, pos.y,
                    pos.z, norm.x, norm.y, norm.z);
          }
          totalTris++;
        }

        delete[] triVerts;
        delete[] triNormals;
      }
      totalConvex++;
    }
    s_pPhysCollision->DestroyQueryModel(pQuery);
  }

  fprintf(fp, "end\n");
  fclose(fp);

  s_pPhysCollision->VCollideUnload(&collide);

  Msg("  Decompiled PHY to physics SMD: %d triangles, %d convex pieces\n",
      totalTris, totalConvex);
  return totalTris > 0;
}

//-----------------------------------------------------------------------------
// Locates studiomdl.exe relative to gamedir and compiles a QC file.
// Searches gamedir/../bin/x64/ then gamedir/../bin/ for studiomdl.exe.
//-----------------------------------------------------------------------------
static bool CompileWithStudiomdl(const char *pQcAbsPath) {
  char studiomdlPath[MAX_PATH];
  bool bFound = false;

  // gamedir is like "E:\Steam\...\GarrysMod\garrysmod\"
  // studiomdl.exe is in the parent's bin/ directory
  char gameBase[MAX_PATH];
  V_strncpy(gameBase, gamedir, sizeof(gameBase));
  V_StripTrailingSlash(gameBase);
  V_StripFilename(gameBase); // go up one level from game dir

  const char *binSubdirs[] = {"bin" CORRECT_PATH_SEPARATOR_S
                              "x64" CORRECT_PATH_SEPARATOR_S "studiomdl.exe",
                              "bin" CORRECT_PATH_SEPARATOR_S "studiomdl.exe"};

  for (int s = 0; s < 2 && !bFound; ++s) {
    V_snprintf(studiomdlPath, sizeof(studiomdlPath), "%s%c%s", gameBase,
               CORRECT_PATH_SEPARATOR, binSubdirs[s]);
    if (g_pFullFileSystem->FileExists(studiomdlPath, NULL)) {
      bFound = true;
    }
  }

  if (!bFound) {
    Warning("Could not find studiomdl.exe for model recompilation\n");
    Warning("  Searched from: %s\n", gamedir);
    return false;
  }

  // Build gamedir without trailing separator to avoid \" escape issues
  char cleanGameDir[MAX_PATH];
  V_strncpy(cleanGameDir, gamedir, sizeof(cleanGameDir));
  V_StripTrailingSlash(cleanGameDir);

  // system() calls cmd.exe /c — we cd to studiomdl's directory first so it
  // can find sibling DLLs (mdllib.dll etc.), then use outer quotes for cmd.exe.
  char studiomdlDir[MAX_PATH];
  V_strncpy(studiomdlDir, studiomdlPath, sizeof(studiomdlDir));
  V_StripFilename(studiomdlDir);

  char studiomdlExe[MAX_PATH];
  V_FileBase(studiomdlPath, studiomdlExe, sizeof(studiomdlExe));
  V_strncat(studiomdlExe, ".exe", sizeof(studiomdlExe));

#ifdef _WIN32
  char cmdline[4 * MAX_PATH];
  V_snprintf(cmdline, sizeof(cmdline),
             "\"%s\" -nop4 -fullcollide -game \"%s\" \"%s\"", studiomdlPath,
             cleanGameDir, pQcAbsPath);

  Msg("  Running: %s\n", cmdline);

  STARTUPINFOA si = {sizeof(si)};
  PROCESS_INFORMATION pi;
  if (!CreateProcessA(NULL, cmdline, NULL, NULL, TRUE, 0, NULL, studiomdlDir,
                      &si, &pi)) {
    Warning("CreateProcess failed for studiomdl.exe (error %d)\n",
            (int)GetLastError());
    return false;
  }

  WaitForSingleObject(pi.hProcess, INFINITE);

  DWORD exitCode = 0;
  GetExitCodeProcess(pi.hProcess, &exitCode);

  CloseHandle(pi.hProcess);
  CloseHandle(pi.hThread);

  if (exitCode != 0) {
    Warning("studiomdl.exe exited with code %d\n", (int)exitCode);
    return false;
  }
#else
  char cmdline[4 * MAX_PATH];
  V_snprintf(
      cmdline, sizeof(cmdline),
      "\"cd /d \"%s\" && \"%s\" -nop4 -fullcollide -game \"%s\" \"%s\"\"",
      studiomdlDir, studiomdlExe, cleanGameDir, pQcAbsPath);

  Msg("  Running: %s\n", cmdline);

  int result = system(cmdline);
  if (result != 0) {
    Warning("studiomdl.exe exited with code %d\n", result);
    return false;
  }
#endif
  return true;
}

//-----------------------------------------------------------------------------
// Embeds a recompiled or patched model into the BSP pakfile.
// Tries full decompile-recompile first; falls back to flag-patching + VVD
// rotation for simple cases.
//
// On full-recompile success, writes the new model path (with maps/<mapname>/
// prefix and _static suffix) into pPatchedNameOut and returns true.
// On fallback or failure, returns false.
//-----------------------------------------------------------------------------
static bool EmbedPatchedModelInPak(char const *pModelName, CUtlBuffer &mdlBuf,
                                   studiohdr_t *pStudioHdr,
                                   char *pPatchedNameOut, int nPatchedNameMax) {
  pPatchedNameOut[0] = '\0';

  // Try the full decompile-recompile pipeline first
  char vtxPath[1024], vvdPath[1024];
  V_strncpy(vtxPath, pModelName, sizeof(vtxPath));
  V_SetExtension(vtxPath, ".dx90.vtx", sizeof(vtxPath));
  V_strncpy(vvdPath, pModelName, sizeof(vvdPath));
  V_SetExtension(vvdPath, ".vvd", sizeof(vvdPath));

  CUtlBuffer vvdBuf, vtxBuf;
  bool bHasVVD = g_pFullFileSystem->ReadFile(vvdPath, NULL, vvdBuf);
  bool bHasVTX = g_pFullFileSystem->ReadFile(vtxPath, NULL, vtxBuf);

  if (bHasVVD && bHasVTX) {
    // Generate temp file paths
    char tempDir[MAX_PATH], tempSmd[MAX_PATH], tempQc[MAX_PATH];
    char autoModelName[512];

    // Use a unique name based on the original model
    char modelBaseName[256];
    V_FileBase(pModelName, modelBaseName, sizeof(modelBaseName));

    V_snprintf(tempDir, sizeof(tempDir), "%s_autostatic", gamedir);
    g_pFullFileSystem->CreateDirHierarchy(tempDir);

    V_snprintf(tempSmd, sizeof(tempSmd), "%s%c%s_ref.smd", tempDir,
               CORRECT_PATH_SEPARATOR, modelBaseName);
    V_snprintf(tempQc, sizeof(tempQc), "%s%c%s.qc", tempDir,
               CORRECT_PATH_SEPARATOR, modelBaseName);

    // Strip "models/" prefix from pModelName to get the relative path
    // e.g. pModelName = "models/props_combine/Cell_01_pod_cheap.mdl"
    //      pRelative  = "props_combine/Cell_01_pod_cheap.mdl"
    const char *pRelative = pModelName;
    if (V_strnicmp(pRelative, "models/", 7) == 0 ||
        V_strnicmp(pRelative, "models\\", 7) == 0)
      pRelative += 7;

    // Build the patched $modelname for the QC:
    //   maps/<mapbase>/<reldir>/<basename>_static.mdl
    // This mirrors the WVT/cubemap patch naming convention.
    char modelRelDir[256];
    V_strncpy(modelRelDir, pRelative, sizeof(modelRelDir));
    V_StripFilename(modelRelDir);

    // Try the full maps/<mapbase>/ path first
    if (modelRelDir[0]) {
      V_snprintf(autoModelName, sizeof(autoModelName),
                 "maps/%s/%s/%s_static.mdl", mapbase, modelRelDir,
                 modelBaseName);
    } else {
      V_snprintf(autoModelName, sizeof(autoModelName), "maps/%s/%s_static.mdl",
                 mapbase, modelBaseName);
    }

    // Check if the full pakfile path fits within DETAIL_NAME_LENGTH.
    // The dict lump stores "models/<autoModelName>".
    char testPakPath[512];
    V_snprintf(testPakPath, sizeof(testPakPath), "models/%s", autoModelName);

    if (V_strlen(testPakPath) >= DETAIL_NAME_LENGTH) {
      // Fallback: keep original directory but append _static to basename
      Warning("  Patched model path too long (%d >= %d), using short path\n",
              V_strlen(testPakPath), DETAIL_NAME_LENGTH);
      if (modelRelDir[0]) {
        V_snprintf(autoModelName, sizeof(autoModelName), "%s/%s_static.mdl",
                   modelRelDir, modelBaseName);
      } else {
        V_snprintf(autoModelName, sizeof(autoModelName), "%s_static.mdl",
                   modelBaseName);
      }
    }

    Msg("  Decompiling model to SMD: %s\n", tempSmd);
    Msg("  Patched model name: models/%s\n", autoModelName);

    if (WriteSMDFromModel(tempSmd, pStudioHdr, vvdBuf, vtxBuf)) {
      // Try to decompile original PHY into a physics SMD
      char tempPhySmd[MAX_PATH];
      V_snprintf(tempPhySmd, sizeof(tempPhySmd), "%s%c%s_physics.smd", tempDir,
                 CORRECT_PATH_SEPARATOR, modelBaseName);
      const char *pPhySmdPath = NULL;
      bool bIsCollisionJoints = false;
      if (DecompilePhyToSmd(pModelName, tempPhySmd, pStudioHdr,
                            &bIsCollisionJoints)) {
        pPhySmdPath = tempPhySmd;
      }

      // Generate QC with the patched $modelname
      if (GenerateStaticPropQC(tempQc, autoModelName, tempSmd, pPhySmdPath,
                               bIsCollisionJoints, pStudioHdr)) {
        Msg("  Generated QC: %s\n", tempQc);

        // Compile with studiomdl
        if (CompileWithStudiomdl(tempQc)) {
          // studiomdl outputs to gamedir/models/<autoModelName dirs>/
          // We need to find the compiled files there.
          char compiledModelDir[MAX_PATH];
          V_snprintf(compiledModelDir, sizeof(compiledModelDir), "%smodels%c",
                     gamedir, CORRECT_PATH_SEPARATOR);

          // Get directory portion of autoModelName for the compiled path
          char autoRelDir[256];
          V_strncpy(autoRelDir, autoModelName, sizeof(autoRelDir));
          V_StripFilename(autoRelDir);
          if (autoRelDir[0]) {
            V_strncat(compiledModelDir, autoRelDir, sizeof(compiledModelDir));
            V_AppendSlash(compiledModelDir, sizeof(compiledModelDir));
          }

          // The compiled basename has _static appended
          char compiledBaseName[256];
          V_snprintf(compiledBaseName, sizeof(compiledBaseName), "%s_static",
                     modelBaseName);

          // Build the patched pakfile path: models/<autoModelName>
          char patchedPakBase[512];
          V_snprintf(patchedPakBase, sizeof(patchedPakBase), "models/%s",
                     autoModelName);

          // Embed recompiled files under the PATCHED model name so the
          // original dynamic model on disk is NOT overwritten.
          const char *exts[] = {".mdl", ".vvd", ".dx90.vtx", ".phy"};
          bool bSuccess = true;

          for (int e = 0; e < 4; ++e) {
            char compiledPath[MAX_PATH];
            V_snprintf(compiledPath, sizeof(compiledPath), "%s%s%s",
                       compiledModelDir, compiledBaseName, exts[e]);

            // Embed under patched model path
            char pakPath[512];
            V_strncpy(pakPath, patchedPakBase, sizeof(pakPath));
            V_SetExtension(pakPath, exts[e], sizeof(pakPath));

            CUtlBuffer fileBuf;
            if (g_pFullFileSystem->ReadFile(compiledPath, NULL, fileBuf)) {
              AddBufferToPak(GetPakFile(), pakPath, fileBuf.Base(),
                             fileBuf.TellMaxPut(), false);
              Msg("  Embedded recompiled: %s\n", pakPath);
            } else if (e == 0) {
              // MDL is required
              Warning("Failed to find recompiled MDL: %s\n", compiledPath);
              bSuccess = false;
              break;
            }
          }

          if (bSuccess) {
            Msg("  Successfully recompiled model as $staticprop\n");
            Msg("  Patched model path: %s\n", patchedPakBase);

            // Write the patched model path back to caller
            V_strncpy(pPatchedNameOut, patchedPakBase, nPatchedNameMax);

            // Clean up compiled model files from gamedir
            // DISABLED FOR DEBUGGING — keep all temp files for inspection
            /*
            const char *cleanExts[] = {".mdl", ".vvd", ".dx90.vtx", ".phy",
                                       ".dx80.vtx"};
            for (int e = 0; e < 5; ++e) {
              char compiledPath[MAX_PATH];
              V_snprintf(compiledPath, sizeof(compiledPath), "%s%s%s",
                         compiledModelDir, compiledBaseName, cleanExts[e]);
              remove(compiledPath);
            }

            // Clean up temp SMD/QC/physics files
            remove(tempSmd);
            remove(tempQc);
            remove(tempPhySmd);
            */

            return true;
          }
        }
      }
    }

    // Clean up temp files on failure too
    remove(tempSmd);
    remove(tempQc);

    Warning("  Decompile-recompile failed, falling back to flag-patching\n");
    return false;
  }

  // Fallback: flag-patching + VVD rotation (works for simple 1-bone models)
  pStudioHdr->flags |= STUDIOHDR_FLAGS_STATIC_PROP;

  AddBufferToPak(GetPakFile(), pModelName, mdlBuf.Base(), mdlBuf.TellMaxPut(),
                 false);
  Msg("  [Fallback] Embedded patched model in BSP: %s\n", pModelName);

  // Embed VTX and PHY unmodified
  char companionPath[1024];
  const char *unmodifiedExts[] = {".dx90.vtx", ".phy"};
  for (int e = 0; e < 2; ++e) {
    V_strncpy(companionPath, pModelName, sizeof(companionPath));
    V_SetExtension(companionPath, unmodifiedExts[e], sizeof(companionPath));
    CUtlBuffer companionBuf;
    if (g_pFullFileSystem->ReadFile(companionPath, NULL, companionBuf)) {
      AddBufferToPak(GetPakFile(), companionPath, companionBuf.Base(),
                     companionBuf.TellMaxPut(), false);
    }
  }

  // Embed modified VVD with +90 Z rotation
  if (bHasVVD) {
    vertexFileHeader_t *pVvdHdr = (vertexFileHeader_t *)vvdBuf.Base();
    if (pVvdHdr->id == MODEL_VERTEX_FILE_ID && pVvdHdr->vertexDataStart != 0) {
      int numVerts = pVvdHdr->numLODVertexes[0];
      mstudiovertex_t *pVerts =
          (mstudiovertex_t *)((byte *)pVvdHdr + pVvdHdr->vertexDataStart);
      for (int v = 0; v < numVerts; ++v) {
        float oldX = pVerts[v].m_vecPosition.x;
        float oldY = pVerts[v].m_vecPosition.y;
        pVerts[v].m_vecPosition.x = -oldY;
        pVerts[v].m_vecPosition.y = oldX;
        oldX = pVerts[v].m_vecNormal.x;
        oldY = pVerts[v].m_vecNormal.y;
        pVerts[v].m_vecNormal.x = -oldY;
        pVerts[v].m_vecNormal.y = oldX;
      }
      if (pVvdHdr->tangentDataStart != 0) {
        Vector4D *pTangents =
            (Vector4D *)((byte *)pVvdHdr + pVvdHdr->tangentDataStart);
        for (int t = 0; t < numVerts; ++t) {
          float oldX = pTangents[t].x;
          float oldY = pTangents[t].y;
          pTangents[t].x = -oldY;
          pTangents[t].y = oldX;
        }
      }
    }
    V_strncpy(companionPath, pModelName, sizeof(companionPath));
    V_SetExtension(companionPath, ".vvd", sizeof(companionPath));
    AddBufferToPak(GetPakFile(), companionPath, vvdBuf.Base(),
                   vvdBuf.TellMaxPut(), false);
    Msg("  [Fallback] Embedded modified VVD\n");
  }

  return false;
}

static CPhysCollide *GetCollisionModel(char const *pModelName) {
  // Convert to a common string
  char *pTemp = (char *)_alloca(strlen(pModelName) + 1);
  strcpy(pTemp, pModelName);
  _strlwr(pTemp);

  char *pSlash = strchr(pTemp, '\\');
  while (pSlash) {
    *pSlash = '/';
    pSlash = strchr(pTemp, '\\');
  }

  // Find it in the cache
  ModelCollisionLookup_t lookup;
  lookup.m_Name = pTemp;
  int i = s_ModelCollisionCache.Find(lookup);
  if (i != s_ModelCollisionCache.InvalidIndex())
    return s_ModelCollisionCache[i].m_pCollide;

  // Load the studio model file
  CUtlBuffer buf;
  if (!LoadStudioModel(pModelName, "prop_static", buf)) {
    Warning("Error loading studio model \"%s\"!\n", pModelName);

    // This way we don't try to load it multiple times
    lookup.m_pCollide = 0;
    s_ModelCollisionCache.Insert(lookup);

    return 0;
  }

  // Compute the convex hull of the model...
  studiohdr_t *pStudioHdr = (studiohdr_t *)buf.PeekGet();

  // If this model was forced as a static prop and doesn't have the flag,
  // patch it and embed the modified model in the BSP pakfile.
  // On success, register the patched model name so the static prop
  // dictionary lump references the new path instead of the original.
  if (g_bForceDynamicPropsAsStatic &&
      !(pStudioHdr->flags & STUDIOHDR_FLAGS_STATIC_PROP)) {
    char patchedName[512];
    if (EmbedPatchedModelInPak(pModelName, buf, pStudioHdr, patchedName,
                               sizeof(patchedName)) &&
        patchedName[0]) {
      // Store mapping: original -> patched, for AddStaticPropToLump
      char *pStored = new char[V_strlen(patchedName) + 1];
      V_strcpy(pStored, patchedName);
      s_PatchedModelNames.Insert(pModelName, pStored);
      Msg("  Registered patched model: %s -> %s\n", pModelName, pStored);
    }
  }

  // necessary for vertex access
  SetCurrentModel(pStudioHdr);

  lookup.m_pCollide = ComputeConvexHull(pStudioHdr);
  s_ModelCollisionCache.Insert(lookup);

  if (!lookup.m_pCollide) {
    Warning("Bad geometry on \"%s\"!\n", pModelName);
  }

  // Debugging
  if (g_DumpStaticProps) {
    static int propNum = 0;
    char tmp[128];
    sprintf(tmp, "staticprop%03d.txt", propNum);
    DumpCollideToGlView(lookup.m_pCollide, tmp);
    ++propNum;
  }

  FreeCurrentModelVertexes();

  // Insert into cache...
  return lookup.m_pCollide;
}

//-----------------------------------------------------------------------------
// Tests a single leaf against the static prop
//-----------------------------------------------------------------------------

static bool TestLeafAgainstCollide(int depth, int *pNodeList,
                                   Vector const &origin, QAngle const &angles,
                                   CPhysCollide *pCollide) {
  // Copy the planes in the node list into a list of planes
  float *pPlanes = (float *)_alloca(depth * 4 * sizeof(float));
  int idx = 0;
  for (int i = depth; --i >= 0; ++idx) {
    int sign = (pNodeList[i] < 0) ? -1 : 1;
    int node = (sign < 0) ? -pNodeList[i] - 1 : pNodeList[i];
    dnode_t *pNode = &dnodes[node];
    dplane_t *pPlane = &dplanes[pNode->planenum];

    pPlanes[idx * 4] = sign * pPlane->normal[0];
    pPlanes[idx * 4 + 1] = sign * pPlane->normal[1];
    pPlanes[idx * 4 + 2] = sign * pPlane->normal[2];
    pPlanes[idx * 4 + 3] = sign * pPlane->dist;
  }

  // Make a convex solid out of the planes
  CPhysConvex *pPhysConvex =
      s_pPhysCollision->ConvexFromPlanes(pPlanes, depth, 0.0f);

  // This should never happen, but if it does, return no collision
  Assert(pPhysConvex);
  if (!pPhysConvex)
    return false;

  CPhysCollide *pLeafCollide =
      s_pPhysCollision->ConvertConvexToCollide(&pPhysConvex, 1);

  // Collide the leaf solid with the static prop solid
  trace_t tr;
  s_pPhysCollision->TraceCollide(vec3_origin, vec3_origin, pLeafCollide,
                                 vec3_angle, pCollide, origin, angles, &tr);

  s_pPhysCollision->DestroyCollide(pLeafCollide);

  return (tr.startsolid != 0);
}

//-----------------------------------------------------------------------------
// Find all leaves that intersect with this bbox + test against the static
// prop..
//-----------------------------------------------------------------------------

static void ComputeConvexHullLeaves_R(int node, int depth, int *pNodeList,
                                      Vector const &mins, Vector const &maxs,
                                      Vector const &origin,
                                      QAngle const &angles,
                                      CPhysCollide *pCollide,
                                      CUtlVector<unsigned short> &leafList) {
  Assert(pNodeList && pCollide);
  Vector cornermin, cornermax;

  while (node >= 0) {
    dnode_t *pNode = &dnodes[node];
    dplane_t *pPlane = &dplanes[pNode->planenum];

    // Arbitrary split plane here
    for (int i = 0; i < 3; ++i) {
      if (pPlane->normal[i] >= 0) {
        cornermin[i] = mins[i];
        cornermax[i] = maxs[i];
      } else {
        cornermin[i] = maxs[i];
        cornermax[i] = mins[i];
      }
    }

    if (DotProduct(pPlane->normal, cornermax) <= pPlane->dist) {
      // Add the node to the list of nodes
      pNodeList[depth] = node;
      ++depth;

      node = pNode->children[1];
    } else if (DotProduct(pPlane->normal, cornermin) >= pPlane->dist) {
      // In this case, we are going in front of the plane. That means that
      // this plane must have an outward normal facing in the oppisite direction
      // We indicate this be storing a negative node index in the node list
      pNodeList[depth] = -node - 1;
      ++depth;

      node = pNode->children[0];
    } else {
      // Here the box is split by the node. First, we'll add the plane as if its
      // outward facing normal is in the direction of the node plane, then
      // we'll have to reverse it for the other child...
      pNodeList[depth] = node;
      ++depth;

      ComputeConvexHullLeaves_R(pNode->children[1], depth, pNodeList, mins,
                                maxs, origin, angles, pCollide, leafList);

      pNodeList[depth - 1] = -node - 1;
      ComputeConvexHullLeaves_R(pNode->children[0], depth, pNodeList, mins,
                                maxs, origin, angles, pCollide, leafList);
      return;
    }
  }

  Assert(pNodeList && pCollide);

  // Never add static props to solid leaves
  if ((dleafs[-node - 1].contents & CONTENTS_SOLID) == 0) {
    if (TestLeafAgainstCollide(depth, pNodeList, origin, angles, pCollide)) {
      leafList.AddToTail(-node - 1);
    }
  }
}

//-----------------------------------------------------------------------------
// Places Static Props in the level
//-----------------------------------------------------------------------------

static void ComputeStaticPropLeaves(CPhysCollide *pCollide,
                                    Vector const &origin, QAngle const &angles,
                                    CUtlVector<unsigned short> &leafList) {
  // Compute an axis-aligned bounding box for the collide
  Vector mins, maxs;
  s_pPhysCollision->CollideGetAABB(&mins, &maxs, pCollide, origin, angles);

  // Find all leaves that intersect with the bounds
  int tempNodeList[1024];
  ComputeConvexHullLeaves_R(0, 0, tempNodeList, mins, maxs, origin, angles,
                            pCollide, leafList);
}

//-----------------------------------------------------------------------------
// Computes the lighting origin
//-----------------------------------------------------------------------------
static bool ComputeLightingOrigin(StaticPropBuild_t const &build,
                                  Vector &lightingOrigin) {
  for (int i = s_LightingInfo.Count(); --i >= 0;) {
    int entIndex = s_LightingInfo[i];

    // Check against all lighting info entities
    char const *pTargetName = ValueForKey(&entities[entIndex], "targetname");
    if (!Q_strcmp(pTargetName, build.m_pLightingOrigin)) {
      GetVectorForKey(&entities[entIndex], "origin", lightingOrigin);
      return true;
    }
  }

  return false;
}

//-----------------------------------------------------------------------------
// Places Static Props in the level
//-----------------------------------------------------------------------------
static void AddStaticPropToLump(StaticPropBuild_t const &build) {
  // Get the collision model
  CPhysCollide *pConvexHull = GetCollisionModel(build.m_pModelName);
  if (!pConvexHull)
    return;

  // Compute the leaves the static prop's convex hull hits
  CUtlVector<unsigned short> leafList;
  ComputeStaticPropLeaves(pConvexHull, build.m_Origin, build.m_Angles,
                          leafList);

  if (!leafList.Count()) {
    Warning("Static prop %s outside the map (%.2f, %.2f, %.2f)\n",
            build.m_pModelName, build.m_Origin.x, build.m_Origin.y,
            build.m_Origin.z);
    return;
  }
  // Insert an element into the lump data...
  int i = s_StaticPropLump.AddToTail();
  StaticPropLump_t &propLump = s_StaticPropLump[i];

  // If this model was recompiled with -forcedynamicpropsasstatic, the patched
  // version lives under maps/<mapname>/ with a _static suffix.  We must
  // reference the patched path in the dict lump so the engine loads the
  // correct (static) version from the pakfile.
  const char *pDictModelName = build.m_pModelName;
  int iPatch = s_PatchedModelNames.Find(build.m_pModelName);
  if (iPatch != s_PatchedModelNames.InvalidIndex()) {
    pDictModelName = s_PatchedModelNames[iPatch];
    Msg("  Using patched model in dict: %s\n", pDictModelName);
  }
  propLump.m_PropType = AddStaticPropDictLump(pDictModelName);
  VectorCopy(build.m_Origin, propLump.m_Origin);
  VectorCopy(build.m_Angles, propLump.m_Angles);
  propLump.m_FirstLeaf = s_StaticPropLeafLump.Count();
  propLump.m_LeafCount = leafList.Count();
  propLump.m_Solid = build.m_Solid;
  propLump.m_Skin = build.m_Skin;
  propLump.m_Flags = build.m_Flags;
  if (build.m_FadesOut) {
    propLump.m_Flags |= STATIC_PROP_FLAG_FADES;
  }
  propLump.m_FadeMinDist = build.m_FadeMinDist;
  propLump.m_FadeMaxDist = build.m_FadeMaxDist;
  propLump.m_flForcedFadeScale = build.m_flForcedFadeScale;
  propLump.m_nMinDXLevel = build.m_nMinDXLevel;
  propLump.m_nMaxDXLevel = build.m_nMaxDXLevel;

  if (build.m_pLightingOrigin && *build.m_pLightingOrigin) {
    if (ComputeLightingOrigin(build, propLump.m_LightingOrigin)) {
      propLump.m_Flags |= STATIC_PROP_USE_LIGHTING_ORIGIN;
    }
  }

  propLump.m_nLightmapResolutionX = build.m_LightmapResolutionX;
  propLump.m_nLightmapResolutionY = build.m_LightmapResolutionY;

  // Add the leaves to the leaf lump
  for (int j = 0; j < leafList.Size(); ++j) {
    StaticPropLeafLump_t insert;
    insert.m_Leaf = leafList[j];
    s_StaticPropLeafLump.AddToTail(insert);
  }
}

//-----------------------------------------------------------------------------
// Places static props in the lump
//-----------------------------------------------------------------------------

static void SetLumpData() {
  GameLumpHandle_t handle =
      g_GameLumps.GetGameLumpHandle(GAMELUMP_STATIC_PROPS);
  if (handle != g_GameLumps.InvalidGameLump())
    g_GameLumps.DestroyGameLump(handle);

  int dictsize = s_StaticPropDictLump.Size() * sizeof(StaticPropDictLump_t);
  int objsize = s_StaticPropLump.Size() * sizeof(StaticPropLump_t);
  int leafsize = s_StaticPropLeafLump.Size() * sizeof(StaticPropLeafLump_t);
  int size = dictsize + objsize + leafsize + 3 * sizeof(int);

  handle = g_GameLumps.CreateGameLump(GAMELUMP_STATIC_PROPS, size, 0,
                                      GAMELUMP_STATIC_PROPS_VERSION);

  // Serialize the data
  CUtlBuffer buf(g_GameLumps.GetGameLump(handle), size);
  buf.PutInt(s_StaticPropDictLump.Size());
  if (dictsize)
    buf.Put(s_StaticPropDictLump.Base(), dictsize);
  buf.PutInt(s_StaticPropLeafLump.Size());
  if (leafsize)
    buf.Put(s_StaticPropLeafLump.Base(), leafsize);
  buf.PutInt(s_StaticPropLump.Size());
  if (objsize)
    buf.Put(s_StaticPropLump.Base(), objsize);
}

//-----------------------------------------------------------------------------
// Places Static Props in the level
//-----------------------------------------------------------------------------

void EmitStaticProps() {

  CreateInterfaceFn physicsFactory = GetPhysicsFactory();
  if (physicsFactory) {
    s_pPhysCollision = (IPhysicsCollision *)physicsFactory(
        VPHYSICS_COLLISION_INTERFACE_VERSION, NULL);
    if (!s_pPhysCollision)
      return;
  }

  // Generate a list of lighting origins, and strip them out
  int i;
  for (i = 0; i < num_entities; ++i) {
    char *pEntity = ValueForKey(&entities[i], "classname");
    if (!Q_strcmp(pEntity, "info_lighting")) {
      s_LightingInfo.AddToTail(i);
    }
  }

  // Emit specifically specified static props
  for (i = 0; i < num_entities; ++i) {
    char *pEntity = ValueForKey(&entities[i], "classname");
    if (!strcmp(pEntity, "static_prop") || !strcmp(pEntity, "prop_static")) {
      StaticPropBuild_t build;

      GetVectorForKey(&entities[i], "origin", build.m_Origin);
      GetAnglesForKey(&entities[i], "angles", build.m_Angles);
      build.m_pModelName = ValueForKey(&entities[i], "model");
      build.m_Solid = IntForKey(&entities[i], "solid");
      build.m_Skin = IntForKey(&entities[i], "skin");
      build.m_FadeMaxDist = FloatForKey(&entities[i], "fademaxdist");
      build.m_Flags =
          0; // IntForKey( &entities[i], "spawnflags" ) & STATIC_PROP_WC_MASK;
      if (IntForKey(&entities[i], "ignorenormals") == 1) {
        build.m_Flags |= STATIC_PROP_IGNORE_NORMALS;
      }
      if (IntForKey(&entities[i], "disableshadows") == 1) {
        build.m_Flags |= STATIC_PROP_NO_SHADOW;
      }
      if (IntForKey(&entities[i], "disablevertexlighting") == 1) {
        build.m_Flags |= STATIC_PROP_NO_PER_VERTEX_LIGHTING;
      }
      if (IntForKey(&entities[i], "disableselfshadowing") == 1) {
        build.m_Flags |= STATIC_PROP_NO_SELF_SHADOWING;
      }

      if (IntForKey(&entities[i], "screenspacefade") == 1) {
        build.m_Flags |= STATIC_PROP_SCREEN_SPACE_FADE;
      }

      if (IntForKey(&entities[i], "generatelightmaps") == 0) {
        build.m_Flags |= STATIC_PROP_NO_PER_TEXEL_LIGHTING;
        build.m_LightmapResolutionX = 0;
        build.m_LightmapResolutionY = 0;
      } else {
        build.m_LightmapResolutionX =
            IntForKey(&entities[i], "lightmapresolutionx");
        build.m_LightmapResolutionY =
            IntForKey(&entities[i], "lightmapresolutiony");
      }

      const char *pKey = ValueForKey(&entities[i], "fadescale");
      if (pKey && pKey[0]) {
        build.m_flForcedFadeScale = FloatForKey(&entities[i], "fadescale");
      } else {
        build.m_flForcedFadeScale = 1;
      }
      build.m_FadesOut = (build.m_FadeMaxDist > 0);
      build.m_pLightingOrigin = ValueForKey(&entities[i], "lightingorigin");
      if (build.m_FadesOut) {
        build.m_FadeMinDist = FloatForKey(&entities[i], "fademindist");
        if (build.m_FadeMinDist < 0) {
          build.m_FadeMinDist = build.m_FadeMaxDist;
        }
      } else {
        build.m_FadeMinDist = 0;
      }
      build.m_nMinDXLevel =
          (unsigned short)IntForKey(&entities[i], "mindxlevel");
      build.m_nMaxDXLevel =
          (unsigned short)IntForKey(&entities[i], "maxdxlevel");
      AddStaticPropToLump(build);

      // strip this ent from the .bsp file
      entities[i].epairs = 0;
    }
  }

  // Strip out lighting origins; has to be done here because they are used when
  // static props are made
  for (i = s_LightingInfo.Count(); --i >= 0;) {
    // strip this ent from the .bsp file
    entities[s_LightingInfo[i]].epairs = 0;
  }

  SetLumpData();
}

static studiohdr_t *g_pActiveStudioHdr;
static void SetCurrentModel(studiohdr_t *pStudioHdr) {
  // track the correct model
  g_pActiveStudioHdr = pStudioHdr;
}

static void FreeCurrentModelVertexes() {
  Assert(g_pActiveStudioHdr);

  if (g_pActiveStudioHdr->VertexBase()) {
    free(g_pActiveStudioHdr->VertexBase());
    g_pActiveStudioHdr->SetVertexBase(NULL);
  }
}

const vertexFileHeader_t *mstudiomodel_t::CacheVertexData(void *pModelData) {
  char fileName[260];
  FileHandle_t fileHandle;
  vertexFileHeader_t *pVvdHdr;

  Assert(pModelData == NULL);
  Assert(g_pActiveStudioHdr);

  if (g_pActiveStudioHdr->VertexBase()) {
    return (vertexFileHeader_t *)g_pActiveStudioHdr->VertexBase();
  }

  // mandatory callback to make requested data resident
  // load and persist the vertex file
  strcpy(fileName, "models/");
  strcat(fileName, g_pActiveStudioHdr->pszName());
  Q_StripExtension(fileName, fileName, sizeof(fileName));
  strcat(fileName, ".vvd");

  // load the model
  fileHandle = g_pFileSystem->Open(fileName, "rb");
  if (!fileHandle) {
    Error("Unable to load vertex data \"%s\"\n", fileName);
  }

  // Get the file size
  int size = g_pFileSystem->Size(fileHandle);
  if (size == 0) {
    g_pFileSystem->Close(fileHandle);
    Error("Bad size for vertex data \"%s\"\n", fileName);
  }

  pVvdHdr = (vertexFileHeader_t *)malloc(size);
  g_pFileSystem->Read(pVvdHdr, size, fileHandle);
  g_pFileSystem->Close(fileHandle);

  // check header
  if (pVvdHdr->id != MODEL_VERTEX_FILE_ID) {
    Error("Error Vertex File %s id %d should be %d\n", fileName, pVvdHdr->id,
          MODEL_VERTEX_FILE_ID);
  }
  if (pVvdHdr->version != MODEL_VERTEX_FILE_VERSION) {
    Error("Error Vertex File %s version %d should be %d\n", fileName,
          pVvdHdr->version, MODEL_VERTEX_FILE_VERSION);
  }
  if (pVvdHdr->checksum != g_pActiveStudioHdr->checksum) {
    Error("Error Vertex File %s checksum %d should be %d\n", fileName,
          pVvdHdr->checksum, g_pActiveStudioHdr->checksum);
  }

  g_pActiveStudioHdr->SetVertexBase((void *)pVvdHdr);
  return pVvdHdr;
}
