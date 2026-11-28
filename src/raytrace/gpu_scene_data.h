//========= GPU Scene Data Structures ============//
//
// Purpose: POD structures for uploading lightmap sample/light/cluster data
//          to VRAM for kernel-side direct lighting (Phase 2+).
//          Phase 1 only uploads the data; it does not launch kernels.
//
//=============================================================================//

#ifndef GPU_SCENE_DATA_H
#define GPU_SCENE_DATA_H

#include "raytrace_shared.h"

// One entry per lightmap sample across all faces.
// Stored contiguously: face 0 samples, face 1 samples, ...
struct GPUSampleData {
  float3_t position; // World-space sample position
  float3_t normal;   // World-space sample normal (flat or phong-interpolated)
  int faceIndex;     // Which dface this sample belongs to
  int sampleIndex;   // Index within the face's sample array
  int clusterIndex;  // PVS cluster for this sample (-1 if none)
  int lightFilter;   // GPU_LIGHTS_*: which lights this sample gathers, plus
                     // flag bits above GPU_LIGHTS_MASK (see below)
};

// GPUSampleData::lightFilter. The CPU supersampler (SupersampleLightAtPoint)
// gathers every light but the sky ambient on its 4x4 sub-grid and the sky
// ambient alone on its 2x2 sub-grid, and averages each over its own count; the
// GPU sub-positions carry the same split so the averages match (#161).
// Primary samples gather everything.
#define GPU_LIGHTS_ALL 0
#define GPU_LIGHTS_NON_AMBIENT 1
#define GPU_LIGHTS_AMBIENT_ONLY 2

// The bits above GPU_LIGHTS_MASK are per-sample flags, so a reader of the
// light selection must mask them off. GPU_SAMPLE_NO_SKY_RECURSE marks a sample
// whose area holds a sky camera: the CPU's TestLine_DoesHitSky
// (vrad/trace.cpp) traces sun and sky-ambient rays into the 3D skybox only
// for a ray start outside every sky camera's area, and the kernel reads this
// bit to make the same decision per sample (#175).
#define GPU_LIGHTS_MASK 0xFF
#define GPU_SAMPLE_NO_SKY_RECURSE 0x100

// One of the map's 3D skybox cameras (sky_camera_t in vrad.h). A sun or
// sky-ambient ray that reaches the sky is traced again from the sample's
// position scaled into this camera's skybox, as the CPU's TestLine_DoesHitSky
// does, so skybox geometry casts its scaled-up shadow on the map.
struct GPUSkyCamera {
  float3_t origin;  // sky_camera's origin, in world space
  float worldToSky; // 1 / scale: world position -> position in the skybox
};

// Per-cluster light visibility list (CSR-style).
// clusterOffset indexes into a flat int array of light indices.
struct GPUClusterLightList {
  int lightOffset; // Offset into flat light-index array
  int lightCount;  // Number of lights visible from this cluster
};

// Per-face metadata: maps face index → range in the GPUSampleData buffer.
struct GPUFaceInfo {
  int sampleOffset; // First sample index in global GPUSampleData array
  int sampleCount;  // Number of samples for this face
  int needsBumpmap; // 1 if face has SURF_BUMPLIGHT, 0 otherwise
  int normalCount;  // 1 (flat only) or 4 (flat + 3 bump basis)
  // World-space bump basis normals (only valid when needsBumpmap=1 and
  // perSampleBump=0; the kernel builds them per sample otherwise)
  float bumpNormal0_x, bumpNormal0_y, bumpNormal0_z;
  float bumpNormal1_x, bumpNormal1_y, bumpNormal1_z;
  float bumpNormal2_x, bumpNormal2_y, bumpNormal2_z;
  // 1 when the CPU computes the bump directions per sample for this face
  // (needsBumpmap=1 AND it is a displacement face or not flat —
  // ComputeIlluminationPointAndNormalsSSE in vrad/lightmap.cpp); the kernel
  // then builds the basis from each sample's own normal like GetBumpNormals.
  int perSampleBump;
  // GetBumpNormals' handedness (DotProduct(flatNormal, CrossProduct(sVect,
  // tVect)) < 0), computed on the host with the same flatNormal the CPU
  // passes: l.facenormal.
  int bumpLeftHanded;
  // The texture S vector GetBumpNormals receives:
  // textureVecsTexelsPerWorldUnits[0][0..2].
  float sVect_x, sVect_y, sVect_z;
};

// Per-sample direct lighting output from the GPU kernel.
// First dimension [4] = lightstyle slots (matching MAXLIGHTMAPS).
// Second dimension [4] = bump vectors: [0]=flat normal, [1..3]=bump basis.
// Non-bumpmapped faces only use bump channel [0].
// styleMap[s] holds the lightstyle value for slot s (-1 = unused).
#define GPU_MAXLIGHTMAPS 4
struct GPULightOutput {
  float r[GPU_MAXLIGHTMAPS][4], g[GPU_MAXLIGHTMAPS][4],
      b[GPU_MAXLIGHTMAPS][4];     // [style_slot][bump_vector]
  int styleMap[GPU_MAXLIGHTMAPS]; // lightstyle value for each slot (-1 =
                                  // unused)
};

//-----------------------------------------------------------------------------
// GPU Scene Data upload/download interface (implemented in
// direct_lighting_gpu.cpp)
//-----------------------------------------------------------------------------
#ifdef VRAD_RTX_CUDA_SUPPORT

// Upload complete scene data to GPU.
// Called once after CalcPoints + BuildPerClusterLightLists.
// All arrays are host memory; this function copies them to VRAM.
void UploadGPUSceneData(const GPUSampleData *samples, int numSamples,
                        const GPUFaceInfo *faceInfos, int numFaces,
                        const GPUClusterLightList *clusterLists,
                        int numClusters, const int *clusterLightIndices,
                        int numClusterLightEntries);

// Allocate and zero the per-sample output buffer in VRAM.
void AllocateDirectLightingOutput(int numSamples);

// Download the output buffer from GPU → host.
void DownloadDirectLightingOutput(GPULightOutput *hostBuffer, int numSamples);

// Free all GPU scene data buffers.
void ShutdownGPUSceneData();

// SS sub-position upload: replace device samples with SS sub-positions.
// Saves the original samples for restoration after the SS pass.
void UploadSSSubPositions(const GPUSampleData *subPositions, int count);
void RestoreOriginalSamples();

// Query uploaded counts (for diagnostics).
int GetGPUSampleCount();
int GetGPUFaceInfoCount();
int GetGPUClusterCount();

// Device pointers — for kernel launches.
GPUSampleData *GetDeviceSamples();
GPUFaceInfo *GetDeviceFaceInfos();
GPUClusterLightList *GetDeviceClusterLightLists();
int *GetDeviceClusterLightIndices();
GPULightOutput *GetDeviceDirectLightingOutput();

#endif // VRAD_RTX_CUDA_SUPPORT

#endif // GPU_SCENE_DATA_H
