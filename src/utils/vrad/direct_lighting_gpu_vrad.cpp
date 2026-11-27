//========= GPU Direct Lighting - VRAD Integration ============//
//
// Purpose: Integrate GPU direct lighting with VRAD lightmap.cpp
//          Converts directlight_t to GPULight and provides batched shadow
//          tracing
//
//=============================================================================//

#include "vrad.h"

#ifdef VRAD_RTX_CUDA_SUPPORT

#include "direct_lighting_gpu.h"
#include "hardware_profiling.h"
#include "lightmap.h"
#include "mathlib/anorms.h"
#include "mathlib/bumpvects.h"
#include "raytrace_optix.h"

// Overload in lightmap.cpp that accepts float* texture vectors
extern void GetBumpNormals(const float *sVect, const float *tVect,
                           const Vector &flatNormal, const Vector &phongNormal,
                           Vector bumpNormals[NUM_BUMP_VECTS]);

//-----------------------------------------------------------------------------
// The number of sky-ambient directions GatherSampleAmbientSkySSE traces for a
// lightmap sample, computed the same way, for the GPU's direction buffer
// (#161).
//-----------------------------------------------------------------------------
int GPUSkyDirectionCount() {
  int nsky_samples = NUMVERTEXNORMALS;
  if (do_fast)
    nsky_samples /= 4;
  else
    nsky_samples *= g_flSkySampleScale;
  return nsky_samples;
}

//-----------------------------------------------------------------------------
// Convert directlight_t to GPULight
//-----------------------------------------------------------------------------
static GPULight ConvertDirectLight(const directlight_t *dl) {
  GPULight gpu;

  gpu.origin_x = dl->light.origin.x;
  gpu.origin_y = dl->light.origin.y;
  gpu.origin_z = dl->light.origin.z;

  gpu.intensity_x = dl->light.intensity.x;
  gpu.intensity_y = dl->light.intensity.y;
  gpu.intensity_z = dl->light.intensity.z;

  gpu.normal_x = dl->light.normal.x;
  gpu.normal_y = dl->light.normal.y;
  gpu.normal_z = dl->light.normal.z;

  gpu.type = dl->light.type;
  gpu.facenum = dl->facenum;

  gpu.constant_attn = dl->light.constant_attn;
  gpu.linear_attn = dl->light.linear_attn;
  gpu.quadratic_attn = dl->light.quadratic_attn;

  gpu.stopdot = dl->light.stopdot;
  gpu.stopdot2 = dl->light.stopdot2;
  gpu.exponent = dl->light.exponent;

  gpu.startFadeDistance = dl->m_flStartFadeDistance;
  gpu.endFadeDistance = dl->m_flEndFadeDistance;
  gpu.capDist = dl->m_flCapDist;

  gpu.style = dl->light.style;

  return gpu;
}

//-----------------------------------------------------------------------------
// Initialize GPU direct lighting from VRAD's activelights
//-----------------------------------------------------------------------------
void InitGPUDirectLighting() {
  // Convert all active lights
  CUtlVector<GPULight> gpuLights;

  for (directlight_t *dl = activelights; dl != nullptr; dl = dl->next) {
    gpuLights.AddToTail(ConvertDirectLight(dl));
  }

  if (gpuLights.Count() > 0) {
    InitDirectLightingGPU(gpuLights.Base(), gpuLights.Count());
    Msg("GPU Direct Lighting: Initialized with %d lights\n", gpuLights.Count());
  }
}

//-----------------------------------------------------------------------------
// Shutdown GPU direct lighting
//-----------------------------------------------------------------------------
void ShutdownGPUDirectLighting() { ShutdownDirectLightingGPU(); }

//-----------------------------------------------------------------------------
// Thread-local shadow ray batch for GPU tracing
// Collects rays during GatherSampleLight, traces them all at once
//-----------------------------------------------------------------------------
struct GPUShadowRayBatch {
  static const int MAX_RAYS = 65536; // Max rays per batch

  GPUShadowRay rays[MAX_RAYS];
  int numRays;

  // For result application - store metadata for each ray
  struct RayMetadata {
    int sampleIndex;  // Which sample this ray is for
    int lightIndex;   // Which light (index into activelights)
    int bumpIndex;    // Which bump normal
    float falloffDot; // Pre-computed falloff * dot product
    float sunAmount;  // Sun amount for sky lights
  };
  RayMetadata metadata[MAX_RAYS];

  GPUShadowRayBatch() : numRays(0) {}

  void Clear() { numRays = 0; }

  bool IsFull() const { return numRays >= MAX_RAYS; }

  // Add a shadow ray to the batch
  // Returns true if added, false if batch is full
  bool AddRay(const Vector &samplePos, const Vector &lightPos, int sampleIdx,
              int lightIdx, int bumpIdx, float falloffDot, float sunAmount) {
    if (numRays >= MAX_RAYS)
      return false;

    Vector dir = lightPos - samplePos;
    float dist = dir.Length();
    if (dist < 0.001f)
      return false;

    dir /= dist; // Normalize

    GPUShadowRay &ray = rays[numRays];
    ray.origin.x = samplePos.x + dir.x * 0.1f; // Offset by epsilon
    ray.origin.y = samplePos.y + dir.y * 0.1f;
    ray.origin.z = samplePos.z + dir.z * 0.1f;
    ray.direction.x = dir.x;
    ray.direction.y = dir.y;
    ray.direction.z = dir.z;
    ray.tmax = dist - 0.2f; // Stop slightly before light
    ray.sampleIndex = sampleIdx;
    ray.lightIndex = lightIdx;
    ray.faceIndex = 0; // Will be set by caller

    RayMetadata &meta = metadata[numRays];
    meta.sampleIndex = sampleIdx;
    meta.lightIndex = lightIdx;
    meta.bumpIndex = bumpIdx;
    meta.falloffDot = falloffDot;
    meta.sunAmount = sunAmount;

    numRays++;
    return true;
  }
};

// Thread-local batches (one per thread)
static GPUShadowRayBatch *g_pThreadBatches = nullptr;
static int g_nThreadBatchCount = 0;

//-----------------------------------------------------------------------------
// Initialize per-thread batches
//-----------------------------------------------------------------------------
void InitGPUShadowBatches(int numThreads) {
  if (g_pThreadBatches) {
    delete[] g_pThreadBatches;
  }
  g_pThreadBatches = new GPUShadowRayBatch[numThreads];
  g_nThreadBatchCount = numThreads;
  Msg("GPU Direct Lighting: Allocated %d thread-local shadow ray batches\n",
      numThreads);
}

//-----------------------------------------------------------------------------
// Cleanup per-thread batches
//-----------------------------------------------------------------------------
void ShutdownGPUShadowBatches() {
  if (g_pThreadBatches) {
    delete[] g_pThreadBatches;
    g_pThreadBatches = nullptr;
  }
  g_nThreadBatchCount = 0;
}

//-----------------------------------------------------------------------------
// Get thread-local batch
//-----------------------------------------------------------------------------
GPUShadowRayBatch *GetThreadShadowBatch(int iThread) {
  if (!g_pThreadBatches || iThread < 0 || iThread >= g_nThreadBatchCount) {
    return nullptr;
  }
  return &g_pThreadBatches[iThread];
}

//-----------------------------------------------------------------------------
// Flush and trace a thread's shadow ray batch
// Returns results in the provided array
//-----------------------------------------------------------------------------
void FlushShadowBatch(int iThread, GPUShadowResult *results) {
  GPUShadowRayBatch *batch = GetThreadShadowBatch(iThread);
  if (!batch || batch->numRays == 0)
    return;

  TraceShadowBatch(batch->rays, results, batch->numRays);
  batch->Clear();
}

//=============================================================================
// BuildGPUSceneData — Phase 1 bridge
//
// After BuildFacelights completes (samples exist for all faces), this function
// packs sample positions/normals + face metadata + cluster-light lists into
// contiguous GPU-friendly arrays and uploads them to VRAM.
//=============================================================================

// Forward decl for ClusterFromPoint (declared in vrad.h, included above)
// int ClusterFromPoint(Vector const &point);

void BuildGPUSceneData() {
  // --- Pass 1: count total samples across all faces ---
  int totalSamples = 0;
  for (int f = 0; f < numfaces; f++) {
    totalSamples += facelight[f].numsamples;
  }

  if (totalSamples == 0) {
    Msg("BuildGPUSceneData: No samples to upload.\n");
    return;
  }

  Msg("BuildGPUSceneData: Packing %d samples across %d faces...\n",
      totalSamples, numfaces);

  // --- Allocate host buffers ---
  GPUSampleData *hostSamples = new GPUSampleData[totalSamples];
  GPUFaceInfo *hostFaceInfos = new GPUFaceInfo[numfaces];
  GPUHostMem_Track("GPUSampleData",
                   (long long)totalSamples * sizeof(GPUSampleData));
  GPUHostMem_Track("GPUFaceInfo", (long long)numfaces * sizeof(GPUFaceInfo));

  // --- Pass 2: fill sample + face info arrays ---
  int sampleCursor = 0;
  for (int f = 0; f < numfaces; f++) {
    facelight_t *fl = &facelight[f];
    hostFaceInfos[f].sampleOffset = sampleCursor;
    hostFaceInfos[f].sampleCount = fl->numsamples;
    bool hasBump = (texinfo[g_pFaces[f].texinfo].flags & SURF_BUMPLIGHT) != 0;
    hostFaceInfos[f].needsBumpmap = hasBump ? 1 : 0;

    // Compute the flat face normal for the hacky offset that the CPU applies
    // in ComputeIlluminationPointAndNormalsSSE: pInfo->m_Points += faceNormal;
    Vector flatNormal;
    VectorCopy(dplanes[g_pFaces[f].planenum].normal, flatNormal);

    if (hasBump) {
      hostFaceInfos[f].normalCount = NUM_BUMP_VECTS + 1; // 4: flat + 3 bump

      // Compute world-space bump basis from face texture vectors
      // NOTE: CPU (lightmap.cpp:4039) uses the raw plane normal without
      // side flipping for GetBumpNormals — must match exactly.
      texinfo_t *pTexInfo = &texinfo[g_pFaces[f].texinfo];

      Vector bumpVects[NUM_BUMP_VECTS];
      GetBumpNormals(pTexInfo->textureVecsTexelsPerWorldUnits[0],
                     pTexInfo->textureVecsTexelsPerWorldUnits[1], flatNormal,
                     flatNormal, bumpVects);

      hostFaceInfos[f].bumpNormal0_x = bumpVects[0].x;
      hostFaceInfos[f].bumpNormal0_y = bumpVects[0].y;
      hostFaceInfos[f].bumpNormal0_z = bumpVects[0].z;
      hostFaceInfos[f].bumpNormal1_x = bumpVects[1].x;
      hostFaceInfos[f].bumpNormal1_y = bumpVects[1].y;
      hostFaceInfos[f].bumpNormal1_z = bumpVects[1].z;
      hostFaceInfos[f].bumpNormal2_x = bumpVects[2].x;
      hostFaceInfos[f].bumpNormal2_y = bumpVects[2].y;
      hostFaceInfos[f].bumpNormal2_z = bumpVects[2].z;
    } else {
      hostFaceInfos[f].normalCount = 1;
      hostFaceInfos[f].bumpNormal0_x = 0;
      hostFaceInfos[f].bumpNormal0_y = 0;
      hostFaceInfos[f].bumpNormal0_z = 0;
      hostFaceInfos[f].bumpNormal1_x = 0;
      hostFaceInfos[f].bumpNormal1_y = 0;
      hostFaceInfos[f].bumpNormal1_z = 0;
      hostFaceInfos[f].bumpNormal2_x = 0;
      hostFaceInfos[f].bumpNormal2_y = 0;
      hostFaceInfos[f].bumpNormal2_z = 0;
    }

    for (int s = 0; s < fl->numsamples; s++) {
      sample_t &sample = fl->sample[s];
      GPUSampleData &gpu = hostSamples[sampleCursor];

      // CPU automatically shifts the initial sample ray positions +1 unit
      // OUTWARDS along the normal. Doing this removes ALL the difference
      // between GPU and CPU.
      gpu.position.x = sample.pos.x + flatNormal.x;
      gpu.position.y = sample.pos.y + flatNormal.y;
      gpu.position.z = sample.pos.z + flatNormal.z;
      gpu.normal.x = sample.normal.x;
      gpu.normal.y = sample.normal.y;
      gpu.normal.z = sample.normal.z;
      gpu.faceIndex = f;
      gpu.sampleIndex = s;
      gpu.clusterIndex = ClusterFromPoint(sample.pos);
      gpu.lightFilter = GPU_LIGHTS_ALL;

      sampleCursor++;
    }
  }

  // --- Build cluster-light index arrays ---
  // Convert from directlight_t* pointers to integer indices matching
  // the GPULight array order (sequential iteration of activelights).
  int numClusters = dvis->numclusters;

  // Build a pointer → index lookup for active lights.
  // The GPULight array is built by iterating activelights in order,
  // so lightPtrs[i] corresponds to GPULight index i.
  CUtlVector<directlight_t *> lightPtrs;
  for (directlight_t *dl = activelights; dl != nullptr; dl = dl->next) {
    lightPtrs.AddToTail(dl);
  }
  int numLightsTotal = lightPtrs.Count();

  GPUClusterLightList *hostClusterLists = nullptr;
  int *hostClusterLightIndices = nullptr;
  int numClusterLightEntries = 0;

  if (numClusters > 0 && g_nClusterLights && g_ClusterLightOffsets) {
    hostClusterLists = new GPUClusterLightList[numClusters];
    GPUHostMem_Track("GPUClusterLightList",
                     (long long)numClusters * sizeof(GPUClusterLightList));

    // Count total entries
    numClusterLightEntries = g_nTotalClusterLightEntries;
    hostClusterLightIndices = new int[numClusterLightEntries];
    GPUHostMem_Track("ClusterLightIndices",
                     (long long)numClusterLightEntries * sizeof(int));

    // Walk the existing CSR-style per-cluster light arrays, converting
    // directlight_t* pointers to integer indices.
    extern directlight_t **g_ClusterLightFlat;
    extern int *g_ClusterLightOffsets;
    int cursor = 0;
    for (int c = 0; c < numClusters; c++) {
      hostClusterLists[c].lightOffset = cursor;
      hostClusterLists[c].lightCount = g_nClusterLights[c];

      int offset = g_ClusterLightOffsets[c];
      for (int i = 0; i < g_nClusterLights[c]; i++) {
        directlight_t *dl = g_ClusterLightFlat[offset + i];
        // Linear scan to find the index — typically <1000 lights
        int lightIdx = -1;
        for (int k = 0; k < numLightsTotal; k++) {
          if (lightPtrs[k] == dl) {
            lightIdx = k;
            break;
          }
        }
        hostClusterLightIndices[cursor] = lightIdx;
        cursor++;
      }
    }
  }

  // --- Upload to VRAM ---
  UploadGPUSceneData(hostSamples, totalSamples, hostFaceInfos, numfaces,
                     hostClusterLists, numClusters, hostClusterLightIndices,
                     numClusterLightEntries);

  Msg("BuildGPUSceneData: %d lights indexed, %d clusters, %d cluster-light "
      "entries\n",
      numLightsTotal, numClusters, numClusterLightEntries);

  // --- Cleanup host buffers ---
  GPUHostMem_Track("GPUSampleData",
                   -(long long)totalSamples * sizeof(GPUSampleData));
  GPUHostMem_Track("GPUFaceInfo", -(long long)numfaces * sizeof(GPUFaceInfo));
  delete[] hostSamples;
  delete[] hostFaceInfos;
  if (hostClusterLists) {
    GPUHostMem_Track("GPUClusterLightList",
                     -(long long)numClusters * sizeof(GPUClusterLightList));
  }
  if (hostClusterLightIndices) {
    GPUHostMem_Track("ClusterLightIndices",
                     -(long long)numClusterLightEntries * sizeof(int));
  }
  delete[] hostClusterLists;
  delete[] hostClusterLightIndices;
}

void ShutdownGPUSceneDataBridge() {
  ShutdownGPUSceneData();
  ShutdownDirectLightingGPU();
}

//=============================================================================
// Phase 2: Launch GPU direct lighting kernel and apply results
//=============================================================================

void LaunchGPUDirectLighting() {
  int numSamples = GetGPUSampleCount();
  if (numSamples <= 0) {
    Warning("LaunchGPUDirectLighting: No samples uploaded!\n");
    return;
  }

  // Allocate and zero the output buffer
  AllocateDirectLightingOutput(numSamples);

  // Upload sky sample directions for emit_skylight/emit_skyambient handling
  RayTraceOptiX::UploadSkyDirections(g_SunAngularExtent,
                                     GPUSkyDirectionCount());

  Msg("LaunchGPUDirectLighting: Launching kernel for %d samples...\n",
      numSamples);

  double startTime = Plat_FloatTime();

  // Launch the OptiX __raygen__direct_lighting kernel
  RayTraceOptiX::TraceDirectLighting(numSamples);

  double elapsed = Plat_FloatTime() - startTime;
  Msg("LaunchGPUDirectLighting: Kernel completed in %.3f seconds\n", elapsed);
}

void DownloadAndApplyGPUResults() {
  int numSamples = GetGPUSampleCount();
  int numFaces = GetGPUFaceInfoCount();
  if (numSamples <= 0 || numFaces <= 0) {
    Warning("DownloadAndApplyGPUResults: No data to download!\n");
    return;
  }

  // Download the GPU output buffer to host
  GPULightOutput *hostOutput = new GPULightOutput[numSamples];
  GPUHostMem_Track("GPULightOutput",
                   (long long)numSamples * sizeof(GPULightOutput));
  DownloadDirectLightingOutput(hostOutput, numSamples);

  // We also need the face info array to know which samples belong to which face
  // Re-iterate the same facelight structure used during upload
  int applied = 0;
  int sampleCursor = 0;

  // Diagnostic accumulators
  double totalR = 0, totalG = 0, totalB = 0;
  int zeroSamples = 0, nonzeroSamples = 0;
  float maxR = 0, maxG = 0, maxB = 0;
  int namedStyleSamples = 0; // track non-zero style applications

  for (int facenum = 0; facenum < numfaces; facenum++) {
    facelight_t &fl = facelight[facenum];
    if (fl.numsamples <= 0) {
      continue;
    }

    dface_t *f = &g_pFaces[facenum];
    int normalCount =
        (texinfo[f->texinfo].flags & SURF_BUMPLIGHT) ? NUM_BUMP_VECTS + 1 : 1;

    // Allocate gpu_point[] to store GPU point light contributions separately.
    // These are subtracted before gradient detection and restored after SS.
    for (int style = 0; style < MAXLIGHTMAPS; style++) {
      for (int n = 0; n < NUM_BUMP_VECTS + 1; n++) {
        if (fl.light[style][n]) {
          fl.gpu_point[style][n] = new LightingValue_t[fl.numsamples];
          memset(fl.gpu_point[style][n], 0,
                 fl.numsamples * sizeof(LightingValue_t));
          GPUHostMem_Track("gpu_point",
                           (long long)fl.numsamples * sizeof(LightingValue_t));
        } else {
          fl.gpu_point[style][n] = nullptr;
        }
      }
    }

    for (int s = 0; s < fl.numsamples; s++) {
      if (sampleCursor >= numSamples)
        break;

      const GPULightOutput &out = hostOutput[sampleCursor];

      // Diagnostic tracking (style slot 0, bump channel 0 = flat normal)
      totalR += out.r[0][0];
      totalG += out.g[0][0];
      totalB += out.b[0][0];
      float totalSampleR = out.r[0][0];
      float totalSampleG = out.g[0][0];
      float totalSampleB = out.b[0][0];
      if (totalSampleR > maxR)
        maxR = totalSampleR;
      if (totalSampleG > maxG)
        maxG = totalSampleG;
      if (totalSampleB > maxB)
        maxB = totalSampleB;
      if (totalSampleR == 0.0f && totalSampleG == 0.0f && totalSampleB == 0.0f)
        zeroSamples++;
      else
        nonzeroSamples++;

      // Iterate over GPU style slots and route each into the correct CPU
      // lightstyle slot via FindOrAllocateLightstyleSamples.
      for (int gs = 0; gs < GPU_MAXLIGHTMAPS; gs++) {
        int gpuStyle = out.styleMap[gs];
        if (gpuStyle < 0)
          break; // no more styles for this sample

        // Check if this style slot has any energy at all
        bool hasEnergy = false;
        for (int n = 0; n < normalCount; n++) {
          if (out.r[gs][n] > 0.0f || out.g[gs][n] > 0.0f ||
              out.b[gs][n] > 0.0f) {
            hasEnergy = true;
            break;
          }
        }
        if (!hasEnergy)
          continue;

        // Find or allocate the CPU-side lightstyle slot
        int cpuSlot =
            FindOrAllocateLightstyleSamples(f, &fl, gpuStyle, normalCount);
        if (cpuSlot < 0)
          continue; // overflow — too many styles on this face

        if (gpuStyle != 0)
          namedStyleSamples++;

        // Apply per-bump-vector GPU results to this style slot.
        // Also store in gpu_point[] for subtract/restore during SS.
        // If FindOrAllocateLightstyleSamples created a new slot,
        // gpu_point[cpuSlot] won't have been pre-allocated — allocate now.
        for (int n = 0; n < normalCount; n++) {
          if (fl.light[cpuSlot][n]) {
            if (!fl.gpu_point[cpuSlot][n]) {
              fl.gpu_point[cpuSlot][n] = new LightingValue_t[fl.numsamples];
              memset(fl.gpu_point[cpuSlot][n], 0,
                     fl.numsamples * sizeof(LightingValue_t));
              GPUHostMem_Track("gpu_point", (long long)fl.numsamples *
                                                sizeof(LightingValue_t));
            }

            fl.light[cpuSlot][n][s].m_vecLighting.x += out.r[gs][n];
            fl.light[cpuSlot][n][s].m_vecLighting.y += out.g[gs][n];
            fl.light[cpuSlot][n][s].m_vecLighting.z += out.b[gs][n];

            fl.gpu_point[cpuSlot][n][s].m_vecLighting.x = out.r[gs][n];
            fl.gpu_point[cpuSlot][n][s].m_vecLighting.y = out.g[gs][n];
            fl.gpu_point[cpuSlot][n][s].m_vecLighting.z = out.b[gs][n];
          }
        }
      }
      applied++;

      sampleCursor++;
    }
  }

  Msg("DownloadAndApplyGPUResults: Applied %d sample results across %d faces\n",
      applied, numFaces);
  if (namedStyleSamples > 0) {
    Msg("  Named light styles: %d sample-style applications (non-zero style)\n",
        namedStyleSamples);
  }
  Msg("  GPU Output Diagnostics:\n");
  Msg("    Total energy: R=%.1f G=%.1f B=%.1f\n", totalR, totalG, totalB);
  Msg("    Nonzero samples: %d (%.1f%%)\n", nonzeroSamples,
      numSamples > 0 ? 100.0 * nonzeroSamples / numSamples : 0.0);
  Msg("    Zero samples: %d (%.1f%%)\n", zeroSamples,
      numSamples > 0 ? 100.0 * zeroSamples / numSamples : 0.0);
  Msg("    Max sample: R=%.3f G=%.3f B=%.3f\n", maxR, maxG, maxB);

  GPUHostMem_Track("GPULightOutput",
                   -(long long)numSamples * sizeof(GPULightOutput));
  delete[] hostOutput;
}

#endif // VRAD_RTX_CUDA_SUPPORT
