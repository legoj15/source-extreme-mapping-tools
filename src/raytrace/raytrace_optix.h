//========================================================================
// raytrace_optix.h - OptiX 9.1 Hardware Ray Tracing for VRAD RTX
//========================================================================

#ifndef RAYTRACE_OPTIX_H
#define RAYTRACE_OPTIX_H

#include "gpu_scene_data.h"
#include "mathlib/vector.h"
#include "raytrace_shared.h"
#include "tier1/utlblockmemory.h"
#include "tier1/utlvector.h"

#include "visibility_gpu.h"

// Forward declarations - matching public/raytrace.h struct declarations
struct CacheOptimizedTriangle;
struct CacheOptimizedKDNode;

//-----------------------------------------------------------------------------
// RayTraceOptiX - Hardware-accelerated ray tracing using RTX cores
//-----------------------------------------------------------------------------
class RayTraceOptiX {
public:
  // Lifecycle
  static bool Initialize();
  static void Shutdown();
  static bool IsInitialized() { return s_bInitialized; }

  // Scene management
  static void
  BuildScene(const CUtlBlockVector<CacheOptimizedTriangle> &triangles,
             const CUtlVector<CacheOptimizedKDNode> &nodes,
             const CUtlVector<int> &triangle_indices,
             const CUtlVector<Vector> &vertices, // 3 vertices per triangle
             const Vector &scene_min, const Vector &scene_max);

  // Visibility Data Management
  static void UploadVisibilityData(const CUtlVector<int> &clusterLeafOffsets,
                                   const CUtlVector<int> &clusterLeafIndices,
                                   const CUtlVector<GPUPatch> &patches);

  // Visibility Tracing
  static void TraceClusterVisibility(const CUtlVector<int> &shooterPatches,
                                     const CUtlVector<int> &visibleClusters,
                                     CUtlVector<VisiblePair> &visiblePairs);

  // Ray tracing
  static void TraceBatch(const RayBatch *rays, RayResult *results,
                         int num_rays);

  // Direct Lighting (Phase 2)
  static void TraceDirectLighting(int numSamples);

  // Sky Light Support (Phase 2b)
  static void UploadSkyDirections(float sunAngularExtent);
  static void FreeSkyDirections();

  // Texture Shadow Support
  static void
  UploadTextureShadowData(const int *triangleMaterials, int numTriangles,
                          const GPUTextureShadowTri *materialEntries,
                          int numMaterialEntries,
                          const unsigned char *alphaAtlas, int atlasSize);
  static void FreeTextureShadowData();
  static bool HasTextureShadowData() { return s_d_texShadowTris != nullptr; }
  static void SetFaceCulling(bool backface, bool frontface) {
    s_backfaceWTShadowCull = backface;
    s_frontfaceWTShadowCull = frontface;
  }

  // Device info
  static const char *GetDeviceName() { return s_szDeviceName; }
  static int GetDeviceMemoryMB() { return s_nDeviceMemoryMB; }
  static bool GetVRAMUsage(size_t &freeMB, size_t &totalMB);

private:
  static bool s_bInitialized;
  static char s_szDeviceName[256];
  static int s_nDeviceMemoryMB;

  // OptiX objects stored as opaque pointers to avoid including optix.h
  static void *s_context;
  static void *s_module;
  static void *s_pipeline;
  static void *s_raygenPG;
  static void *s_missPG;
  static void *s_hitgroupPG;
  static void *s_visRaygenPG;            // Visibility kernel raygen
  static void *s_directLightingRaygenPG; // Direct lighting kernel raygen

  // Acceleration structure
  static void *s_d_gas_output_buffer;
  static unsigned long long s_gas_handle;

  // Shader Binding Table
  static void *s_d_sbt_buffer;

  // Triangle data on GPU
  static CUDATriangle *s_d_triangles;
  static int s_triangleCount;

  // Double-buffered ray/result/param buffers for ping-pong pipelining
  static const int NUM_BUFFERS = 2;
  static RayBatch *s_d_rays[NUM_BUFFERS];
  static RayResult *s_d_results[NUM_BUFFERS];
  static void *s_d_launchParams[NUM_BUFFERS];
  static int s_maxBatchSize;

  // Visibility Buffers
  static GPUVisSceneData s_visData; // Holds device pointers
  static void *s_d_shooterPatches;  // Temp buffer for current cluster
  static void *s_d_visibleClusters; // Temp buffer for current cluster
  static void *s_d_visiblePairs;    // Output buffer
  static void *s_d_pairCount;       // Atomic counter
  static int s_maxVisibilityPairs;  // Size of output buffer

  // Double-buffered async streaming (CUDA streams + pinned host memory)
  static void *s_streams[NUM_BUFFERS]; // cudaStream_t
  static RayBatch
      *s_h_rays_pinned[NUM_BUFFERS]; // Page-locked host buffers for uploads
  static RayResult *
      s_h_results_pinned[NUM_BUFFERS]; // Page-locked host buffers for downloads

  // Thread safety
  static class CThreadMutex s_Mutex;

  // ----- GPU Bounce buffers -----
  static long long *s_d_csrOffsets;
  static int *s_d_csrPatch;
  static float *s_d_csrWeight;
  static float3_t *s_d_reflectivity;
  static float3_t *s_d_patchOrigin;
  static float3_t *s_d_patchNormal;
  static int *s_d_needsBumpmap;
  static int *s_d_faceNumber;
  static float3_t *s_d_emitlight;
  static float3_t *s_d_addlight;
  static float3_t *s_d_addlightBump;
  static float3_t *s_d_bumpNormals;
  static int s_bounceNumPatches;
  static int s_bounceTotalTransfers;
  static bool s_bounceInitialized;

  // Sky direction sample buffer
  static float3_t *s_d_skyDirs; // Device pointer
  static int s_numSkyDirs;
  static float s_sunAngularExtent;

  // Texture shadow data on GPU
  static int *s_d_triMaterials; // Per-triangle material index (-1 = opaque)
  static GPUTextureShadowTri
      *s_d_texShadowTris;               // Per-material-entry UV + atlas info
  static unsigned char *s_d_alphaAtlas; // Flattened alpha texture data
  static bool s_textureShadowsEnabled;
  static bool s_backfaceWTShadowCull;
  static bool s_frontfaceWTShadowCull;

public:
  // GPU Bounce light gathering
  static bool InitBounceBuffers(int numPatches, int totalTransfers,
                                const long long *csrOffsets,
                                const int *csrPatch, const float *csrWeight,
                                const float *reflectivity,
                                const float *patchOrigin,
                                const float *patchNormal,
                                const int *needsBumpmap, const int *faceNumber,
                                const float *bumpNormals, int numBumpPatches);
  static void GatherLightGPU(const float *emitlight, float *addlight,
                             float *addlightBump);
  static void FreeBounceBuffers();
  static void PrintBounceProfile();
  static void ResetBounceProfile();
};

#endif // RAYTRACE_OPTIX_H
