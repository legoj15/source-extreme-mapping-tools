//========= proplm: model readers shared by the uv2 and rebake routes =========
//
// The MDL/VVD/VTX reading that both proplm routes need (proplm_uv2.cpp,
// proplm_rebake.cpp). Implemented in proplm_uv2.cpp. Issue #62 is the eventual
// single loader for every VBSP/VRAD model walker; this header only stops a
// third copy.
//=============================================================================
#ifndef PROPLM_SHARED_H
#define PROPLM_SHARED_H
#pragma once

#include "tier0/platform.h"
#include "tier1/UtlBuffer.h"
#include "tier1/utlvector.h"

struct studiohdr_t;

// One VVD vertex of the LOD0 vertex list (fixups applied), in plain floats.
// raw is the vertex's index in the VVD's own vertex block, before fixups: the
// slot to patch when the file is rewritten.
struct ProplmVvdVert_t {
  float pos[3];
  float nrm[3];
  float uv[2];
  int raw;
};

// One mesh visited by Proplm_GatherVtxTriangles, in walk order.
struct ProplmVtxMesh_t {
  int bodyPart;
  int model;
  int mesh;
  int material; // mstudiomesh_t::material: an index into the skinref table
};

// One trilist triangle: the visited mesh it came from and its three corners as
// indices into the LOD0 vertex list, in VTX index order.
struct ProplmVtxTri_t {
  int mesh; // index into the ProplmVtxMesh_t list
  int vert[3];
};

// True when count elements of elemSize bytes starting at byte ofs lie inside a
// file of size bytes. Overflow-safe for any int inputs.
inline bool Proplm_InFile(int64 ofs, int64 count, int64 elemSize, int64 size) {
  if (ofs < 0 || count < 0 || elemSize < 0 || ofs > size)
    return false;
  return elemSize == 0 || count <= (size - ofs) / elemSize;
}

// g_pFullFileSystem->ReadFile; false for a missing or empty file.
bool Proplm_ReadWholeFile(const char *pPath, CUtlBuffer &buf);

// Fills verts with the LOD0 vertex list of the VVD (fixups applied; the order
// the VTX's origMeshVertID + mesh vertexoffset index into, for every LOD while
// the root LOD is 0). Checks id, version, the checksum against the MDL's,
// every offset, and that positions, normals and texcoords are finite.
bool Proplm_LoadVvdVertices(const CUtlBuffer &vvdBuf, int mdlChecksum,
                            CUtlVector<ProplmVvdVert_t> &verts, char *pErr,
                            int errLen);

// Largest |texcoord| a VVD may hold (Proplm_LoadVvdVertices refuses more).
#define PROPLM_MAX_TEXCOORD 1024.0f

// Checks the header size and the studio version (44..STUDIO_VERSION), and for
// 44..47 the local animation table Studio_ConvertStudioHdrToNewVersion walks
// and writes, then runs that conversion. Call it before anything else reads
// the MDL, then Proplm_ValidateMdl.
bool Proplm_ConvertMdl(studiohdr_t *pHdr, int fileSize, char *pErr, int errLen);

// Checks every MDL table the proplm routes walk against the file size before
// anything dereferences it: textures and their names, cdtextures, the skin
// table (each entry a valid texture), bodyparts, models and meshes, and
// studiohdr2 (positive, 4-byte aligned, inside the file) with its name.
// pHdr is the file's first byte (after Proplm_ConvertMdl).
bool Proplm_ValidateMdl(const studiohdr_t *pHdr, int fileSize, char *pErr,
                        int errLen);

// Walks VTX LOD lod of every bodypart, either model 0 only (the default
// bodygroup) or every model, and appends each trilist triangle in VTX order.
// Strips that are not trilists are skipped, or with bRequireTrilists fail the
// walk. A model without that LOD is skipped. Checks the VTX version and
// checksum, every VTX offset and count against the buffer size, and every
// index. The MDL must have passed Proplm_ValidateMdl. pMeshes may be NULL;
// tris[i].mesh is then still an index into the walk order.
bool Proplm_GatherVtxTriangles(studiohdr_t *pStudioHdr, const CUtlBuffer &vtxBuf,
                               int vertCount, int lod, bool bAllModels,
                               bool bRequireTrilists,
                               CUtlVector<ProplmVtxMesh_t> *pMeshes,
                               CUtlVector<ProplmVtxTri_t> &tris, char *pErr,
                               int errLen);

#endif // PROPLM_SHARED_H
