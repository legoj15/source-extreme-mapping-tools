//========= Copyright Valve Corporation, All rights reserved. ============//
//
// Purpose:
//
// $NoKeywords: $
//
//=============================================================================//

#ifdef MPI
#include "vmpi.h"
#endif
#include "vrad.h"
#include <psapi.h>
#pragma comment(lib, "psapi.lib")

static void PrintCommitCharge(const char *label) {
  PROCESS_MEMORY_COUNTERS_EX pmc = {};
  pmc.cb = sizeof(pmc);
  if (GetProcessMemoryInfo(GetCurrentProcess(), (PROCESS_MEMORY_COUNTERS *)&pmc,
                           sizeof(pmc))) {
    double commitMB = (double)pmc.PrivateUsage / (1024.0 * 1024.0);
    double wsMB = (double)pmc.WorkingSetSize / (1024.0 * 1024.0);
    Msg("  [MEM] %s: Commit=%.0f MB, WorkingSet=%.0f MB\n", label, commitMB,
        wsMB);
  }
}

#ifdef MPI
#include "messbuf.h"
static MessageBuffer mb;
#endif

#ifdef VRAD_RTX_CUDA_SUPPORT
#include "tier0/fasttimer.h"
// Per-thread profiling accumulators — lock-free, summed at report time
static double g_tVisCPUPrepTime[MAX_TOOL_THREADS];
static double g_tVisWaitTime[MAX_TOOL_THREADS];
static double g_tPVSDecompTime[MAX_TOOL_THREADS];
static double g_tClusterIterTime[MAX_TOOL_THREADS];
static double g_tTransferMakeTime[MAX_TOOL_THREADS];
static long long g_tRaysTraced[MAX_TOOL_THREADS];
#endif

/*
===================================================================

VISIBILITY MATRIX

Determine which patches can see each other
Use the PVS to accelerate if available
===================================================================
*/

#define TEST_EPSILON 0.1
#define PLANE_TEST_EPSILON                                                     \
  0.01 // patch must be this much in front of the plane to be considered "in
       // front"
#define PATCH_FACE_OFFSET                                                      \
  0.1 // push patch origins off from the face by this amount to avoid self
      // collisions

#define STREAM_SIZE 512

class CTransferMaker {
public:
  int m_iThread; // Thread ID for per-thread profiling accumulators

  CTransferMaker(transfer_t *all_transfers);
  ~CTransferMaker();

  FORCEINLINE void TestMakeTransfer(Vector start, Vector stop, int ndxShooter,
                                    int ndxReceiver) {
#ifdef VRAD_RTX_CUDA_SUPPORT
    if (g_bUseGPU) {
      int idx = m_gpuRays.AddToTail();
      RayBatch &ray = m_gpuRays[idx];
      ray.origin.x = start.x;
      ray.origin.y = start.y;
      ray.origin.z = start.z;

      // OPTIMIZATION: Use normalized direction vector
      ray.direction.x = stop.x - start.x;
      ray.direction.y = stop.y - start.y;
      ray.direction.z = stop.z - start.z;

      float len = VectorLength(*(Vector *)&ray.direction);
      if (len > 0) {
        float invLen = 1.0f / len;
        ray.direction.x *= invLen;
        ray.direction.y *= invLen;
        ray.direction.z *= invLen;
      }

      ray.tmax = len;
      ray.tmin = 0.0f;

      ray.skip_id = -1;

      PatchPair_t pair;
      pair.shooter = ndxShooter;
      pair.receiver = ndxReceiver;
      m_gpuPairs.AddToTail(pair);
      return;
    }
#endif
    g_RtEnv.AddToRayStream(m_RayStream, start, stop, &m_pResults[m_nTests]);
    m_pShooterPatches[m_nTests] = ndxShooter;
    m_pRecieverPatches[m_nTests] = ndxReceiver;
    ++m_nTests;
  }

  void Finish();

#ifdef VRAD_RTX_CUDA_SUPPORT
  void TraceAll();
  void ProcessResult(int i);

  struct PatchPair_t {
    int shooter;
    int receiver;
  };
  CUtlVector<RayBatch> m_gpuRays;
  CUtlVector<PatchPair_t> m_gpuPairs;
  CUtlVector<RayResult> m_gpuResults;
#endif

  transfer_t *m_AllTransfers;

  // FAST VISITATION: Use patch IDs to avoid memset(0) every row
  int *m_faceTestedPatch;
  int *m_dispTestedPatch;

private:
  int m_nTests;
  RayTracingSingleResult *m_pResults;
  int *m_pShooterPatches;
  int *m_pRecieverPatches;
  RayStream m_RayStream;
};

CTransferMaker::CTransferMaker(transfer_t *all_transfers)
    : m_AllTransfers(all_transfers), m_nTests(0), m_iThread(0) {
  m_pResults = (RayTracingSingleResult *)calloc(
      1, MAX_PATCHES * sizeof(RayTracingSingleResult));
  m_pShooterPatches = (int *)calloc(1, MAX_PATCHES * sizeof(int));
  m_pRecieverPatches = (int *)calloc(1, MAX_PATCHES * sizeof(int));

  m_faceTestedPatch = (int *)malloc(numfaces * sizeof(int));
  m_dispTestedPatch = (int *)malloc(numfaces * sizeof(int));
  for (int i = 0; i < numfaces; i++) {
    m_faceTestedPatch[i] = -1;
    m_dispTestedPatch[i] = -1;
  }

#ifdef VRAD_RTX_CUDA_SUPPORT
  if (g_bUseGPU) {
    m_gpuRays.EnsureCapacity(MAX_PATCHES);
    m_gpuPairs.EnsureCapacity(MAX_PATCHES);
    m_gpuResults.EnsureCapacity(MAX_PATCHES);
  }
#endif
}

CTransferMaker::~CTransferMaker() {
  free(m_pResults);
  free(m_pShooterPatches);
  free(m_pRecieverPatches);
  free(m_faceTestedPatch);
  free(m_dispTestedPatch);
}

void CTransferMaker::Finish() {
#ifdef VRAD_RTX_CUDA_SUPPORT
  if (g_bUseGPU) {
    TraceAll();
    int count = m_gpuRays.Count();
    for (int i = 0; i < count; ++i) {
      ProcessResult(i);
    }
    m_gpuRays.RemoveAll();
    m_gpuPairs.RemoveAll();
    m_gpuResults.RemoveAll();
    return;
  }
#endif
  g_RtEnv.FinishRayStream(m_RayStream);

  if (m_nTests > 0) {
    g_tRaysTraced[m_iThread] += m_nTests;
  }

  for (int i = 0; i < m_nTests; i++) {
    // Visible if: no hit found, or hit is at/beyond endpoint (matching
    // Reference SDK)
    if (m_pResults[i].HitID == -1 ||
        m_pResults[i].HitDistance >= m_pResults[i].ray_length) {
      MakeTransfer(m_pShooterPatches[i], m_pRecieverPatches[i], m_AllTransfers);
    }
  }
  m_nTests = 0;
}

#ifdef VRAD_RTX_CUDA_SUPPORT
void CTransferMaker::TraceAll() {
  int rayCount = m_gpuRays.Count();
  if (rayCount > 0) {
    g_tRaysTraced[m_iThread] += rayCount;

    m_gpuResults.SetCount(rayCount);

    double start = Plat_FloatTime();
    RayTraceOptiX::TraceBatch(m_gpuRays.Base(), m_gpuResults.Base(), rayCount);
    double end = Plat_FloatTime();

    g_tVisWaitTime[m_iThread] += (end - start);
  }
}

void CTransferMaker::ProcessResult(int i) {
  // Visible if: no hit found, or hit is at/beyond endpoint
  if (m_gpuResults[i].hit_id == -1 ||
      m_gpuResults[i].hit_t >= m_gpuRays[i].tmax) {
    MakeTransfer(m_gpuPairs[i].shooter, m_gpuPairs[i].receiver, m_AllTransfers);
  }
}
#endif

dleaf_t *PointInLeaf(int iNode, Vector const &point) {
  if (iNode < 0)
    return &dleafs[(-1 - iNode)];

  dnode_t *node = &dnodes[iNode];
  dplane_t *plane = &dplanes[node->planenum];

  float dist = DotProduct(point, plane->normal) - plane->dist;
  if (dist > TEST_EPSILON) {
    return PointInLeaf(node->children[0], point);
  } else if (dist < -TEST_EPSILON) {
    return PointInLeaf(node->children[1], point);
  } else {
    dleaf_t *pTest = PointInLeaf(node->children[0], point);
    if (pTest->cluster != -1)
      return pTest;

    return PointInLeaf(node->children[1], point);
  }
}

int ClusterFromPoint(Vector const &point) {
  dleaf_t *leaf = PointInLeaf(0, point);

  return leaf->cluster;
}

void PvsForOrigin(Vector &org, byte *pvs) {
  int visofs;
  int cluster;

  if (!visdatasize) {
    memset(pvs, 255, (dvis->numclusters + 7) / 8);
    return;
  }

  cluster = ClusterFromPoint(org);
  if (cluster < 0) {
    visofs = -1;
  } else {
    visofs = dvis->bitofs[cluster][DVIS_PVS];
  }

  if (visofs == -1)
    Error("visofs == -1");

  DecompressVis(&dvisdata[visofs], pvs);
}

void TestPatchToPatch(int ndxPatch1, int ndxPatch2, Vector const &p1, int head,
                      transfer_t *transfers, CTransferMaker &transferMaker,
                      int iThread) {
  Vector tmp;

  //
  // get patches
  //
  if (ndxPatch1 == g_Patches.InvalidIndex() ||
      ndxPatch2 == g_Patches.InvalidIndex())
    return;

  CPatch *patch = &g_Patches.Element(ndxPatch1);
  CPatch *patch2 = &g_Patches.Element(ndxPatch2);

  if (patch2->child1 != g_Patches.InvalidIndex()) {
    // check to see if we should use a child node instead

    VectorSubtract(patch->origin, patch2->origin, tmp);
    // SQRT( 1/4 )
    // FIXME: should be based on form-factor (ie. include visible angle, etc)
    if (DotProduct(tmp, tmp) * 0.0625 < patch2->area) {
      TestPatchToPatch(ndxPatch1, patch2->child1, p1, head, transfers,
                       transferMaker, iThread);
      TestPatchToPatch(ndxPatch1, patch2->child2, p1, head, transfers,
                       transferMaker, iThread);
      return;
    }
  }

  // check vis between patch and patch2
  // if bit has not already been set
  //  && v2 is not behind light plane
  //  && v2 is visible from v1
  if (DotProduct(patch2->origin, patch->normal) >
      patch->planeDist + PLANE_TEST_EPSILON) {
    // SDK 2013 behavior: offset BOTH shooter and receiver by 1.0 * normal
    Vector p2;
    VectorAdd(patch2->origin, patch2->normal, p2);

#ifdef VRAD_RTX_CUDA_SUPPORT
    if (g_bUseGPU) {
      int idx = transferMaker.m_gpuRays.AddToTail();
      RayBatch &ray = transferMaker.m_gpuRays[idx];
      ray.origin.x = p1.x;
      ray.origin.y = p1.y;
      ray.origin.z = p1.z;

      // Set direction FIRST (p2 - p1), then normalize
      ray.direction.x = p2.x - p1.x;
      ray.direction.y = p2.y - p1.y;
      ray.direction.z = p2.z - p1.z;

      float len = VectorLength(*(Vector *)&ray.direction);
      if (len > 1e-4f) {
        float invLen = 1.0f / len;
        ray.direction.x *= invLen;
        ray.direction.y *= invLen;
        ray.direction.z *= invLen;
        ray.tmax = len;
      } else {
        ray.tmax = 0.0f; // Skip backward/zero rays
      }
      ray.tmin = 0.0f;

      ray.skip_id = -1;

      CTransferMaker::PatchPair_t pair;
      pair.shooter = ndxPatch1;
      pair.receiver = ndxPatch2;
      transferMaker.m_gpuPairs.AddToTail(pair);
      return;
    }
#endif

    transferMaker.TestMakeTransfer(p1, p2, ndxPatch1, ndxPatch2);
  }
}

/*
==============
TestPatchToFace

Sets vis bits for all patches in the face
==============
*/
void TestPatchToFace(unsigned patchnum, int facenum, Vector const &p1, int head,
                     transfer_t *transfers, CTransferMaker &transferMaker,
                     int iThread) {
  if (faceParents.Element(facenum) == g_Patches.InvalidIndex() ||
      patchnum == g_Patches.InvalidIndex())
    return;

  CPatch *patch = &g_Patches.Element(patchnum);
  CPatch *patch2 = &g_Patches.Element(faceParents.Element(facenum));

  // if emitter is behind that face plane, skip all patches

  CPatch *pNextPatch;

  if (patch2 && DotProduct(patch->origin, patch2->normal) >
                    patch2->planeDist + PLANE_TEST_EPSILON) {
    // we need to do a real test
    for (; patch2; patch2 = pNextPatch) {
      // next patch
      pNextPatch = NULL;
      if (patch2->ndxNextParent != g_Patches.InvalidIndex()) {
        pNextPatch = &g_Patches.Element(patch2->ndxNextParent);
      }

      /*
      // skip patches too far away
      VectorSubtract( patch->origin, patch2->origin, tmp );
      if (DotProduct( tmp, tmp ) > 512 * 512)
              continue;
      */

      int ndxPatch2 = patch2 - g_Patches.Base();
      TestPatchToPatch(patchnum, ndxPatch2, p1, head, transfers, transferMaker,
                       iThread);
    }
  }
}

struct ClusterDispList_t {
  CUtlVector<int> dispFaces;
};

static CUtlVector<ClusterDispList_t> g_ClusterDispFaces;

//-----------------------------------------------------------------------------
// Helps us find all displacements associated with a particular cluster
//-----------------------------------------------------------------------------
void AddDispsToClusterTable(void) {
  g_ClusterDispFaces.SetCount(g_ClusterLeaves.Count());

  //
  // add displacement faces to the cluster table
  //
  for (int ndxFace = 0; ndxFace < numfaces; ndxFace++) {
    // search for displacement faces
    if (g_pFaces[ndxFace].dispinfo == -1)
      continue;

    //
    // get the clusters associated with the face
    //
    if (g_FacePatches.Element(ndxFace) != g_FacePatches.InvalidIndex()) {
      CPatch *pNextPatch = NULL;
      for (CPatch *pPatch = &g_Patches.Element(g_FacePatches.Element(ndxFace));
           pPatch; pPatch = pNextPatch) {
        // next patch
        pNextPatch = NULL;
        if (pPatch->ndxNext != g_Patches.InvalidIndex()) {
          pNextPatch = &g_Patches.Element(pPatch->ndxNext);
        }

        if (pPatch->clusterNumber != g_Patches.InvalidIndex()) {
          int ndxDisp =
              g_ClusterDispFaces[pPatch->clusterNumber].dispFaces.Find(ndxFace);
          if (ndxDisp == -1) {
            ndxDisp =
                g_ClusterDispFaces[pPatch->clusterNumber].dispFaces.AddToTail();
            g_ClusterDispFaces[pPatch->clusterNumber].dispFaces[ndxDisp] =
                ndxFace;
          }
        }
      }
    }
  }
}

//-----------------------------------------------------------------------------
// Static prop patch cluster table (ported from CSGO)
//-----------------------------------------------------------------------------
struct ClusterPatchList_t {
  CUtlVector<int> patches;
};

static CUtlVector<ClusterPatchList_t> g_ClusterStaticPropPatches;

void AddStaticPropPatchesToClusterTable(void) {
  g_ClusterStaticPropPatches.SetCount(g_ClusterLeaves.Count());

  for (int i = 0; i < g_Patches.Count(); i++) {
    const CPatch &patch = g_Patches[i];
    if (patch.faceNumber >= 0 || patch.clusterNumber < 0) {
      continue;
    }

    g_ClusterStaticPropPatches[patch.clusterNumber].patches.AddToTail(i);
  }
}

/*
==============
BuildVisRow

Calc vis bits from a single patch
==============
*/
void BuildVisRow(int patchnum, byte *pvs, int head, transfer_t *transfers,
                 CTransferMaker &transferMaker, int iThread) {
  int j, k, l, leafIndex;
  CPatch *patch;
  dleaf_t *leaf;

  patch = &g_Patches.Element(patchnum);

  Vector p1;
  VectorAdd(patch->origin, patch->normal, p1);

  for (j = 0; j < dvis->numclusters; j++) {
    if (!(pvs[(j) >> 3] & (1 << ((j) & 7)))) {
      continue; // not in pvs
    }

    for (leafIndex = 0; leafIndex < g_ClusterLeaves[j].leafCount; leafIndex++) {
      leaf = dleafs + g_ClusterLeaves[j].leafs[leafIndex];

      for (k = 0; k < leaf->numleaffaces; k++) {
        l = dleaffaces[leaf->firstleafface + k];
        // faces can be marksurfed by multiple leaves, but
        // don't bother testing again
        if (transferMaker.m_faceTestedPatch[l] == patchnum) {
          continue;
        }
        transferMaker.m_faceTestedPatch[l] = patchnum;

        // don't check patches on the same face
        if (patch->faceNumber == l)
          continue;
        TestPatchToFace(patchnum, l, p1, head, transfers, transferMaker,
                        iThread);
      }
    }

    int dispCount = g_ClusterDispFaces[j].dispFaces.Size();
    for (int ndxDisp = 0; ndxDisp < dispCount; ndxDisp++) {
      int ndxFace = g_ClusterDispFaces[j].dispFaces[ndxDisp];
      if (transferMaker.m_dispTestedPatch[ndxFace] == patchnum)
        continue;

      transferMaker.m_dispTestedPatch[ndxFace] = patchnum;

      // don't check patches on the same face
      if (patch->faceNumber == ndxFace)
        continue;

      TestPatchToFace(patchnum, ndxFace, p1, head, transfers, transferMaker,
                      iThread);
    }

    // Test static prop patches in this cluster
    if (g_bStaticPropBounce && j < g_ClusterStaticPropPatches.Count()) {
      int staticPropPatchCount = g_ClusterStaticPropPatches[j].patches.Count();
      for (int i = 0; i < staticPropPatchCount; i++) {
        int nPatchIdx = g_ClusterStaticPropPatches[j].patches[i];
        if ((unsigned)nPatchIdx != patchnum) {
          TestPatchToPatch(patchnum, nPatchIdx, p1, head, transfers,
                           transferMaker, iThread);
        }
      }
    }
  }
}
// Msg("%d) Transfers: %5d\n", patchnum, patch->numtransfers);

/*
===========
BuildVisLeafs

  This is run by multiple threads
===========
*/

transfer_t *BuildVisLeafs_Start() {
  return (transfer_t *)calloc(1, MAX_PATCHES * sizeof(transfer_t));
}

// Auto-scaled GPU ray batch size based on available physical RAM.
// Each ray in the batch costs ~72 bytes (RayBatch + PatchPair + RayResult).
// With 12+ threads, large batches can create significant commit pressure.
// Scale from 256K (tight RAM) to 2M (plenty of RAM).
static int GetAutoScaledGPURayBatch() {
  static int s_cachedBatch = 0;
  if (s_cachedBatch > 0)
    return s_cachedBatch;

  // Default: 2M rays per batch
  s_cachedBatch = 2 * 1024 * 1024;

#ifdef _WIN32
  MEMORYSTATUSEX memInfo;
  memInfo.dwLength = sizeof(memInfo);
  if (GlobalMemoryStatusEx(&memInfo)) {
    unsigned long long availGB = memInfo.ullAvailPhys / (1024ULL * 1024 * 1024);
    if (availGB < 8) {
      s_cachedBatch = 256 * 1024; // 256K rays - ~18 MB per thread
      Msg("GPU ray batch: 256K (%.1f GB RAM available)\n",
          (float)memInfo.ullAvailPhys / (1024.0f * 1024 * 1024));
    } else if (availGB < 16) {
      s_cachedBatch = 512 * 1024; // 512K rays - ~36 MB per thread
      Msg("GPU ray batch: 512K (%.1f GB RAM available)\n",
          (float)memInfo.ullAvailPhys / (1024.0f * 1024 * 1024));
    } else if (availGB < 32) {
      s_cachedBatch = 1024 * 1024; // 1M rays - ~72 MB per thread
      Msg("GPU ray batch: 1M (%.1f GB RAM available)\n",
          (float)memInfo.ullAvailPhys / (1024.0f * 1024 * 1024));
    } else {
      Msg("GPU ray batch: 2M (%.1f GB RAM available)\n",
          (float)memInfo.ullAvailPhys / (1024.0f * 1024 * 1024));
    }
  }
#endif

  return s_cachedBatch;
}
#define MAX_GPU_RAY_BATCH GetAutoScaledGPURayBatch()

// If PatchCB is non-null, it is called after each row is generated (used by
// MPI).
void BuildVisLeafs_Cluster(int threadnum, transfer_t *transfers, int iCluster,
                           CTransferMaker &transferMaker,
                           void (*PatchCB)(int iThread, int patchnum,
                                           CPatch *patch)) {
  byte pvs[(MAX_MAP_CLUSTERS + 7) / 8];
  CPatch *patch;
  int head;

  double pvsStart = Plat_FloatTime();
  DecompressVis(&dvisdata[dvis->bitofs[iCluster][DVIS_PVS]], pvs);
  double pvsEnd = Plat_FloatTime();

  g_tPVSDecompTime[threadnum] += (pvsEnd - pvsStart);
  head = 0;

#ifdef VRAD_RTX_CUDA_SUPPORT
  // GPU path: batch rays across patches, flush when buffer exceeds threshold.
  // This keeps GPU kernel launches large (good utilization) while bounding
  // per-thread memory to ~72 MB instead of the previous unbounded growth
  // that could reach 3-5 GB per thread on dense maps.
  CUtlVector<int> patchRayOffsets;
  CUtlVector<int> clusterPatches;
  int lastFlushedPatch =
      0; // Index into clusterPatches of first unprocessed patch
#endif

  if (clusterChildren.Element(iCluster) != clusterChildren.InvalidIndex()) {
    CPatch *pNextPatch;
    for (patch = &g_Patches.Element(clusterChildren.Element(iCluster)); patch;
         patch = pNextPatch) {
      pNextPatch = NULL;
      if (patch->ndxNextClusterChild != g_Patches.InvalidIndex()) {
        pNextPatch = &g_Patches.Element(patch->ndxNextClusterChild);
      }

      int patchnum = patch - g_Patches.Base();

#ifdef VRAD_RTX_CUDA_SUPPORT
      if (g_bUseGPU) {
        clusterPatches.AddToTail(patchnum);
        patchRayOffsets.AddToTail(transferMaker.m_gpuRays.Count());
      }
#endif

      // build to all other world clusters
      double start = Plat_FloatTime();
      BuildVisRow(patchnum, pvs, head, transfers, transferMaker, threadnum);
      double end = Plat_FloatTime();

      g_tVisCPUPrepTime[threadnum] += (end - start);
      g_tClusterIterTime[threadnum] += (end - start);

#ifdef VRAD_RTX_CUDA_SUPPORT
      if (g_bUseGPU) {
        // Flush if ray buffer exceeds threshold
        if (transferMaker.m_gpuRays.Count() >= MAX_GPU_RAY_BATCH) {
          patchRayOffsets.AddToTail(transferMaker.m_gpuRays.Count());
          transferMaker.TraceAll();

          // Process results for all patches accumulated since last flush
          for (int i = lastFlushedPatch; i < clusterPatches.Count(); i++) {
            int pn = clusterPatches[i];
            int rstart = patchRayOffsets[i - lastFlushedPatch];
            int rend = patchRayOffsets[i - lastFlushedPatch + 1];
            for (int j = rstart; j < rend; j++) {
              transferMaker.ProcessResult(j);
            }
            MakeScales(pn, transfers);
            if (PatchCB)
              PatchCB(threadnum, pn, &g_Patches.Element(pn));
          }
          lastFlushedPatch = clusterPatches.Count();

          // Clear buffers and offsets for next batch
          transferMaker.m_gpuRays.RemoveAll();
          transferMaker.m_gpuPairs.RemoveAll();
          transferMaker.m_gpuResults.RemoveAll();
          patchRayOffsets.RemoveAll();
        }
        continue; // Skip the CPU Finish/MakeScales below
      }
#endif

      // CPU path: process per-patch immediately
      transferMaker.Finish();
      MakeScales(patchnum, transfers);
      if (PatchCB)
        PatchCB(threadnum, patchnum, patch);
    }
  }

#ifdef VRAD_RTX_CUDA_SUPPORT
  // Flush any remaining rays from the last partial batch
  if (g_bUseGPU && clusterPatches.Count() > lastFlushedPatch) {
    patchRayOffsets.AddToTail(transferMaker.m_gpuRays.Count());
    transferMaker.TraceAll();

    for (int i = lastFlushedPatch; i < clusterPatches.Count(); i++) {
      int pn = clusterPatches[i];
      int rstart = patchRayOffsets[i - lastFlushedPatch];
      int rend = patchRayOffsets[i - lastFlushedPatch + 1];
      for (int j = rstart; j < rend; j++) {
        transferMaker.ProcessResult(j);
      }
      MakeScales(pn, transfers);
      if (PatchCB)
        PatchCB(threadnum, pn, &g_Patches.Element(pn));
    }

    transferMaker.m_gpuRays.RemoveAll();
    transferMaker.m_gpuPairs.RemoveAll();
    transferMaker.m_gpuResults.RemoveAll();
  }
#endif
}

void BuildVisLeafs_End(transfer_t *transfers) { free(transfers); }

// Separate function for GPU path to avoid macro mess in BuildVisLeafs
#ifdef VRAD_RTX_CUDA_SUPPORT
void BuildVisLeafs_GPU(int threadnum, void *pUserData) {
  transfer_t *transfers = BuildVisLeafs_Start();
  // We don't need CTransferMaker for the visibility tracing part,
  // but we use it via MakeScales which takes transfers.
  // Actually MakeScales creates the transfer lists in g_Patches.

  // We need buffers for the thread.
  CUtlVector<int> visibleClusters;
  CUtlVector<VisiblePair> visiblePairs;
  CUtlVector<int> shooterPatches;

  // Pre-allocate
  visibleClusters.EnsureCapacity(dvis->numclusters);

  while (1) {
    int iCluster = GetThreadWork();
    if (iCluster == -1)
      break;

    // 1. Decompress PVS
    byte pvs[(MAX_MAP_CLUSTERS + 7) / 8];
    double pvsStart = Plat_FloatTime();
    DecompressVis(&dvisdata[dvis->bitofs[iCluster][DVIS_PVS]], pvs);
    double pvsEnd = Plat_FloatTime();

    g_tPVSDecompTime[threadnum] += (pvsEnd - pvsStart);

    // 2. Build Visible Cluster List
    visibleClusters.RemoveAll();
    for (int j = 0; j < dvis->numclusters; j++) {
      if (pvs[j >> 3] & (1 << (j & 7))) {
        visibleClusters.AddToTail(j);
      }
    }

    // 3. Gather Shooter Patches
    shooterPatches.RemoveAll();
    if (clusterChildren.Element(iCluster) != clusterChildren.InvalidIndex()) {
      CPatch *pNextPatch;
      for (CPatch *patch =
               &g_Patches.Element(clusterChildren.Element(iCluster));
           patch; patch = pNextPatch) {
        pNextPatch = NULL;
        if (patch->ndxNextClusterChild != g_Patches.InvalidIndex()) {
          pNextPatch = &g_Patches.Element(patch->ndxNextClusterChild);
        }
        shooterPatches.AddToTail(patch - g_Patches.Base());
      }
    }

    if (shooterPatches.Count() == 0 || visibleClusters.Count() == 0)
      continue;

    // 4. Trace on GPU
    double start = Plat_FloatTime();
    RayTraceOptiX::TraceClusterVisibility(shooterPatches, visibleClusters,
                                          visiblePairs);
    double end = Plat_FloatTime();

    g_tVisWaitTime[threadnum] += (end - start);
    g_tClusterIterTime[threadnum] += (end - start);

    // 5. Process Results
    // visiblePairs contains {shooter, receiver} indices
    for (int i = 0; i < visiblePairs.Count(); i++) {
      MakeTransfer(visiblePairs[i].shooter, visiblePairs[i].receiver,
                   transfers);
    }

    // 6. Finalize Transfers (MakeScales)
    for (int i = 0; i < shooterPatches.Count(); i++) {
      MakeScales(shooterPatches[i], transfers);
    }
  }

  BuildVisLeafs_End(transfers);
}
#endif

void BuildVisLeafs(int threadnum, void *pUserData) {
#if 0 // DISABLED: Brute-force GPU visibility creates too many transfers (~1.7
      // billion). Instead, use the hybrid approach in BuildVisLeafs_Cluster
      // which uses CPU hierarchical subdivision + GPU ray tracing via
      // CTransferMaker.
#ifdef VRAD_RTX_CUDA_SUPPORT
  if (g_bUseGPU) {
    BuildVisLeafs_GPU(threadnum, pUserData);
    return;
  }
#endif
#endif

  transfer_t *transfers = BuildVisLeafs_Start();
  CTransferMaker transferMaker(transfers);
  transferMaker.m_iThread = threadnum;

  while (1) {
    //
    // build a minimal BSP tree that only
    // covers areas relevent to the PVS
    //
    // JAY: Now this returns a cluster index
    int iCluster = GetThreadWork();
    if (iCluster == -1)
      break;

    BuildVisLeafs_Cluster(threadnum, transfers, iCluster, transferMaker, NULL);
  }

  BuildVisLeafs_End(transfers);
}

//-----------------------------------------------------------------------------
// GPU Visibility Initialization
//-----------------------------------------------------------------------------
#ifdef VRAD_RTX_CUDA_SUPPORT
static bool s_bVisDataUploaded = false;

struct FlatFaceList_t {
  CUtlVector<int> faces;
};

void InitGPUVisibility() {
  if (!g_bUseGPU || s_bVisDataUploaded)
    return;

  Msg("Initializing GPU Visibility Data...\n");
  double start = Plat_FloatTime();

  // 1. Flatten Cluster -> Faces (Leaves + Displacements) -> THEN TO PATCHES
  // We already have g_ClusterDispFaces. We need to walk leaves too.

  CUtlVector<int> clusterLeafOffsets;
  CUtlVector<int> clusterLeafIndices;

  clusterLeafOffsets.AddToTail(0);

  for (int iCluster = 0; iCluster < dvis->numclusters; iCluster++) {
    // Collect all faces for this cluster, then expand to leaf patches
    CUtlVector<int> facesInCluster;

    // Leaf Faces
    for (int leafIndex = 0; leafIndex < g_ClusterLeaves[iCluster].leafCount;
         leafIndex++) {
      int leafID = g_ClusterLeaves[iCluster].leafs[leafIndex];
      dleaf_t *leaf = &dleafs[leafID];
      for (int k = 0; k < leaf->numleaffaces; k++) {
        int faceIdx = dleaffaces[leaf->firstleafface + k];
        facesInCluster.AddToTail(faceIdx);
      }
    }

    // Displacement Faces
    if (iCluster < g_ClusterDispFaces.Count()) {
      const CUtlVector<int> &dispFaces = g_ClusterDispFaces[iCluster].dispFaces;
      for (int i = 0; i < dispFaces.Count(); i++) {
        facesInCluster.AddToTail(dispFaces[i]);
      }
    }

    // Now convert faces to leaf patches
    for (int i = 0; i < facesInCluster.Count(); ++i) {
      int faceIdx = facesInCluster[i];
      if (g_FacePatches.Element(faceIdx) != g_FacePatches.InvalidIndex()) {
        // Iterate linked list of patches for this face
        CPatch *pNextPatch = NULL;
        for (CPatch *pPatch =
                 &g_Patches.Element(g_FacePatches.Element(faceIdx));
             pPatch; pPatch = pNextPatch) {

          pNextPatch = NULL;
          if (pPatch->ndxNext != g_Patches.InvalidIndex()) {
            pNextPatch = &g_Patches.Element(pPatch->ndxNext);
          }

          // Only add LEAF patches (patches with no children or valid
          // cluster?) Actually in VRAD, patches with children are "parents"
          // and we generally don't test against them? "if (patch->child1 !=
          // g_Patches.InvalidIndex()) continue;"
          if (pPatch->child1 == g_Patches.InvalidIndex()) {
            clusterLeafIndices.AddToTail(pPatch - g_Patches.Base());
          }
        }
      }
    }

    clusterLeafOffsets.AddToTail(clusterLeafIndices.Count());
  }

  // 2. Face -> Root Patch Mapping (Unchanged, kept for reference if needed,
  // though we might not need it for Leafs anymore) Actually the kernel
  // probably won't use this if we feed it leaves directly.
  CUtlVector<int> faceParentPatch;
  faceParentPatch.SetCount(numfaces);
  for (int i = 0; i < numfaces; i++) {
    // faceParents contains the index of the first patch for the face
    // We can just use that. If invalid, store -1.
    if (i < faceParents.Count()) {
      faceParentPatch[i] = faceParents[i];
    } else {
      faceParentPatch[i] = -1;
    }
  }

  // 3. Convert Patches
  CUtlVector<GPUPatch> gpuPatches;
  gpuPatches.SetCount(g_Patches.Count());

  for (int i = 0; i < g_Patches.Count(); i++) {
    const CPatch &src = g_Patches[i];
    GPUPatch &dst = gpuPatches[i];

    dst.origin.x = src.origin.x;
    dst.origin.y = src.origin.y;
    dst.origin.z = src.origin.z;

    dst.normal.x = src.normal.x;
    dst.normal.y = src.normal.y;
    dst.normal.z = src.normal.z;

    dst.planeDist = src.planeDist;
    dst.faceNumber = src.faceNumber;
    dst.clusterNumber = src.clusterNumber;
    dst.area = src.area;
    dst.child1 = src.child1;
    dst.child2 = src.child2;
    dst.ndxNextParent = src.ndxNextParent;
  }

  // 4. Upload
  RayTraceOptiX::UploadVisibilityData(clusterLeafOffsets, clusterLeafIndices,
                                      gpuPatches);

  s_bVisDataUploaded = true;

  double end = Plat_FloatTime();
  Msg("GPU Visibility Init took %.2f seconds.\n", end - start);
}
#endif

/*
==============
BuildVisMatrix
==============
*/
void BuildVisMatrix(void) {
  double flStart = Plat_FloatTime();

  PrintCommitCharge("BuildVisMatrix START");

#ifdef VRAD_RTX_CUDA_SUPPORT
  if (g_bUseGPU) {
    InitGPUVisibility();
    PrintCommitCharge("After InitGPUVisibility");
  }
#endif

#ifdef MPI
  if (g_bUseMPI) {
    RunMPIBuildVisLeafs();
  } else
#endif
  {
    Msg("  Launching BuildVisLeafs threads (%d clusters)...\n",
        dvis->numclusters);
    PrintCommitCharge("Before RunThreadsOn");
    RunThreadsOn(dvis->numclusters, true, BuildVisLeafs);
    PrintCommitCharge("After RunThreadsOn");
  }
  double flEnd = Plat_FloatTime();
  double flWall = flEnd - flStart;
  g_flVisMatrixTime = flWall;

  uint64 total_transfers = 0;
  for (int i = 0; i < g_Patches.Count(); i++) {
    total_transfers += g_Patches[i].numtransfers;
  }
  Msg("BuildVisMatrix complete. Total transfers: %llu\n", total_transfers);

  Msg("\nVisibility Matrix Profile:\n");

  // Sum per-thread profiling accumulators
  double g_totalVisCPUPrepTime = 0, g_totalVisWaitTime = 0;
  double g_totalPVSDecompTime = 0, g_totalClusterIterTime = 0;
#ifdef VRAD_RTX_CUDA_SUPPORT
  for (int t = 0; t < MAX_TOOL_THREADS; t++) {
    g_totalVisCPUPrepTime += g_tVisCPUPrepTime[t];
    g_totalVisWaitTime += g_tVisWaitTime[t];
    g_totalPVSDecompTime += g_tPVSDecompTime[t];
    g_totalClusterIterTime += g_tClusterIterTime[t];
    g_TotalRaysTraced += g_tRaysTraced[t];
  }
#endif

  Msg("  Total Rays Traced:      %lld\n", g_TotalRaysTraced);
  Msg("  Phase Wall Clock:       %.2f s\n", flWall);
  Msg("  Cumulative CPU Time:    %.2f s (Sum of all threads)\n",
      g_totalVisCPUPrepTime);
  Msg("    - PVS Decompression:  %.2f s\n", g_totalPVSDecompTime);
  Msg("    - Cluster Iteration:  %.2f s\n", g_totalClusterIterTime);

#ifdef VRAD_RTX_CUDA_SUPPORT
  if (g_bUseGPU) {
    Msg("  Cumulative GPU Wait:    %.2f s\n", g_totalVisWaitTime);

    double totalCpuActive = g_totalVisCPUPrepTime + g_totalVisWaitTime;
    if (totalCpuActive > 0 && flWall > 0) {
      double cpuRatio = g_totalVisCPUPrepTime / totalCpuActive;
      double gpuRatio = g_totalVisWaitTime / totalCpuActive;

      Msg("  Wall Clock Estimations:\n");
      Msg("    - CPU Prep (Est):     %.2f s (%.1f%% Utilization)\n",
          flWall * cpuRatio, cpuRatio * 100.0);
      Msg("    - GPU Wait (Est):     %.2f s (%.1f%% Utilization)\n",
          flWall * gpuRatio, gpuRatio * 100.0);
      Msg("    - Thread Efficiency:  %.1fx\n", totalCpuActive / flWall);
    }
  }
#endif
}

void FreeVisMatrix(void) {}
