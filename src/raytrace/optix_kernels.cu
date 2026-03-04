//========================================================================
// optix_kernels.cu - OptiX 9.1 Ray Tracing Kernels for VRAD RTX
// Compiled to PTX and loaded at runtime
//========================================================================

#include "gpu_scene_data.h"
#include "visibility_gpu.h"
#include <optix.h>
#include <optix_device.h>

// Include shared structures (same as CUDA version)
// Note: This is compiled separately, so we define the structs inline
// UPDATE: Now we use visibility_gpu.h which includes raytrace_shared.h
// So we must NOT redefine these structs.

//-----------------------------------------------------------------------------
// GPU Light Structure - must match GPULight in direct_lighting_gpu.h
// NOTE: Uses explicit float fields (not float3_t) to guarantee identical
// struct layout between MSVC (host) and NVCC (device PTX) compilation.
//-----------------------------------------------------------------------------
struct GPULight {
  float origin_x, origin_y, origin_z;
  float intensity_x, intensity_y, intensity_z;
  float normal_x, normal_y, normal_z; // For emit_surface and emit_spotlight

  int type; // emit_point=0, emit_surface=1, emit_spotlight=2, emit_skylight=3
  int facenum; // -1 for point lights, face index for surface lights

  // Attenuation
  float constant_attn;
  float linear_attn;
  float quadratic_attn;

  // Spotlight parameters
  float stopdot;  // cos(inner cone angle)
  float stopdot2; // cos(outer cone angle)
  float exponent;

  // Fade distances
  float startFadeDistance;
  float endFadeDistance;
  float capDist;

  // Lightstyle index (0 = always-on, 1-63 = named/switchable)
  int style;
};

// Light type enums (must match emittype_t in bspfile.h)
#define EMIT_SURFACE 0
#define EMIT_POINT 1
#define EMIT_SPOTLIGHT 2
#define EMIT_SKYLIGHT 3
#define EMIT_QUAKELIGHT 4
#define EMIT_SKYAMBIENT 5

//-----------------------------------------------------------------------------
// Launch Parameters - must match host struct exactly
//-----------------------------------------------------------------------------
struct OptixLaunchParams {
  const RayBatch *rays;
  RayResult *results;
  int num_rays;
  OptixTraversableHandle traversable;
  const CUDATriangle *triangles;

  // Visibility Extension
  const int *shooter_patches;
  int num_shooters;
  const int *visible_clusters;
  int num_visible_clusters;
  GPUVisSceneData vis_scene_data;
  VisiblePair *visible_pairs;
  int *pair_count_atomic;
  int max_pairs;

  // Direct Lighting Extension (Phase 2)
  const GPUSampleData *d_samples;
  const GPULight *d_lights;
  const GPUClusterLightList *d_clusterLists;
  const int *d_clusterLightIndices;
  const GPUFaceInfo *d_faceInfos;
  GPULightOutput *d_lightOutput;
  int num_samples;
  int num_lights;
  int num_clusters;

  // Sky Light Extension (Phase 2b)
  const float3 *d_skyDirs; // Precomputed hemisphere sample directions
  int numSkyDirs;          // Number of sky sample directions (162 default)
  float sunAngularExtent;  // Area sun jitter (0 = point sun)
  int numSunSamples;       // Samples for area sun (30 default, 0 = point sun)

  // Sun Shadow Anti-aliasing
  int sunShadowSamples;  // Sub-luxel position samples (default 16)
  float sunShadowRadius; // World-space jitter radius in units (default 4.0)

  // Texture Shadow Support
  const int *d_triMaterials; // Per-triangle material index (-1 = opaque)
  const GPUTextureShadowTri
      *d_texShadowTris;              // Per-material-entry UV + atlas info
  const unsigned char *d_alphaAtlas; // Flattened alpha texture data
  int textureShadowsEnabled;         // 1 if texture shadows active, 0 otherwise
  int backfaceWTShadowCull;  // 1 if backface culling for texture shadows
  int frontfaceWTShadowCull; // 1 if frontface culling for texture shadows
};

extern "C" __constant__ OptixLaunchParams params;

//-----------------------------------------------------------------------------
// Ray payload - data passed between programs
// Now includes skip_id for self-intersection filtering
//-----------------------------------------------------------------------------
struct RayPayload {
  float hit_t;
  int hit_id;
  float3 normal;
  int skip_id; // Triangle ID to skip (passed through payload)
};

//-----------------------------------------------------------------------------
// Ray Generation Program - launches one ray per thread
//-----------------------------------------------------------------------------
extern "C" __global__ void __raygen__visibility() {
  const uint3 idx = optixGetLaunchIndex();
  const int rayIdx = idx.x;

  if (rayIdx >= params.num_rays)
    return;

  // Get ray parameters
  const RayBatch &ray = params.rays[rayIdx];

  float3 origin = make_float3(ray.origin.x, ray.origin.y, ray.origin.z);
  float3 direction =
      make_float3(ray.direction.x, ray.direction.y, ray.direction.z);

  // Initialize payload - include skip_id for filtering
  unsigned int p0 = __float_as_uint(1e30f);    // hit_t
  unsigned int p1 = (unsigned int)-1;          // hit_id
  unsigned int p2 = __float_as_uint(0.0f);     // normal.x
  unsigned int p3 = __float_as_uint(0.0f);     // normal.y
  unsigned int p4 = __float_as_uint(0.0f);     // normal.z
  unsigned int p5 = (unsigned int)ray.skip_id; // skip_id for filtering
  unsigned int p6 =
      __float_as_uint(0.0f); // coverage accumulator (0.0 = no occlusion)

  // Use small epsilon for tmin to avoid self-intersection at ray origin
  // 1e-4 is optimal for Source engine units - smaller values cause self-hits
  float tmin = ray.tmin;
  if (tmin < 1e-4f)
    tmin = 1e-4f;

  // Trace ray - OPTIX_RAY_FLAG_NONE already means no face culling
  optixTrace(params.traversable, origin, direction,
             tmin,     // tmin (with epsilon to avoid self-hits)
             ray.tmax, // tmax
             0.0f,     // rayTime
             OptixVisibilityMask(255), OPTIX_RAY_FLAG_NONE,
             0, // SBT offset
             1, // SBT stride
             0, // missSBTIndex
             p0, p1, p2, p3, p4, p5, p6);

  // Write results
  RayResult &result = params.results[rayIdx];
  result.hit_t = __uint_as_float(p0);
  result.hit_id = (int)p1;
  result.normal.x = __uint_as_float(p2);
  result.normal.y = __uint_as_float(p3);
  result.normal.z = __uint_as_float(p4);
}

//-----------------------------------------------------------------------------
// Any Hit Program - called for every potential intersection
// Used to filter out self-intersections (skip_id)
//-----------------------------------------------------------------------------
extern "C" __global__ void __anyhit__visibility() {
  // Get the primitive (triangle) index
  const int primIdx = optixGetPrimitiveIndex();

  // Get skip_id from payload
  const int skip_id = (int)optixGetPayload_5();

  // Get triangle's actual ID
  const CUDATriangle &tri = params.triangles[primIdx];

  // If this triangle matches skip_id, reject the intersection
  if (tri.triangle_id == skip_id) {
    optixIgnoreIntersection();
    return;
  }

  // Texture shadow check: if enabled and triangle has transparency flag
  if (params.textureShadowsEnabled && (tri.flags & 0x01)) {
    int matIdx = params.d_triMaterials[primIdx];
    if (matIdx >= 0) {
      const GPUTextureShadowTri &mat = params.d_texShadowTris[matIdx];

      // Get OptiX barycentric coordinates
      // OptiX returns (b1, b2) where b0 = 1 - b1 - b2
      float2 bary = optixGetTriangleBarycentrics();
      float b0 = 1.0f - bary.x - bary.y;
      float b1 = bary.x;
      float b2 = bary.y;

      // Face culling check: compute dot product of ray direction and triangle
      // normal
      if (params.backfaceWTShadowCull || params.frontfaceWTShadowCull) {
        float3 rayDir = optixGetWorldRayDirection();
        float dot = rayDir.x * tri.nx + rayDir.y * tri.ny + rayDir.z * tri.nz;
        // Backface: ray hits from behind (dot > 0)
        if (params.backfaceWTShadowCull && dot > 0.0f) {
          optixIgnoreIntersection();
          return;
        }
        // Frontface: ray hits from the front (dot < 0)
        if (params.frontfaceWTShadowCull && dot < 0.0f) {
          optixIgnoreIntersection();
          return;
        }
      }

      // Interpolate UV coordinates
      float u = b0 * mat.u0 + b1 * mat.u1 + b2 * mat.u2;
      float v = b0 * mat.v0 + b1 * mat.v1 + b2 * mat.v2;

      // Wrap (power-of-2 assumption matches CPU SampleMaterial)
      int iu = __float2int_rn(u * mat.texWidth) & (mat.texWidth - 1);
      int iv = __float2int_rn(v * mat.texHeight) & (mat.texHeight - 1);

      unsigned char alpha =
          params.d_alphaAtlas[mat.atlasOffset + iv * mat.texWidth + iu];

      // True transparency: additive coverage model (matches CPU)
      // CPU does: coverage += alpha/255; coverage = min(coverage, 1.0);
      float addedCoverage = alpha * (1.0f / 255.0f);
      float coverage = __uint_as_float(optixGetPayload_6());
      coverage = fminf(coverage + addedCoverage, 1.0f);
      optixSetPayload_6(__float_as_uint(coverage));

      // If coverage hasn't reached 1.0, let the ray continue
      if (coverage < 1.0f) {
        optixIgnoreIntersection();
        return;
      }
    }
  }

  // Otherwise, accept the intersection (continue to closest-hit)
}

//-----------------------------------------------------------------------------
// Closest Hit Program - called when ray hits a triangle
//-----------------------------------------------------------------------------
extern "C" __global__ void __closesthit__visibility() {
  // Get the primitive (triangle) index
  const int primIdx = optixGetPrimitiveIndex();

  // Get hit distance
  const float t = optixGetRayTmax();

  // Get triangle normal from our data
  const CUDATriangle &tri = params.triangles[primIdx];

  // Update payload with hit information
  optixSetPayload_0(__float_as_uint(t));
  optixSetPayload_1((unsigned int)primIdx);
  optixSetPayload_2(__float_as_uint(tri.nx));
  optixSetPayload_3(__float_as_uint(tri.ny));
  optixSetPayload_4(__float_as_uint(tri.nz));
}

//-----------------------------------------------------------------------------
// Miss Program - called when ray doesn't hit anything
//-----------------------------------------------------------------------------
extern "C" __global__ void __miss__visibility() {
  // No hit - set hit_id to -1
  optixSetPayload_0(__float_as_uint(1e30f));
  optixSetPayload_1((unsigned int)-1);
  optixSetPayload_2(__float_as_uint(0.0f));
  optixSetPayload_3(__float_as_uint(0.0f));
  optixSetPayload_4(__float_as_uint(0.0f));
}

//-----------------------------------------------------------------------------
// Cluster Visibility Ray Generation
//-----------------------------------------------------------------------------
extern "C" __global__ void __raygen__cluster_visibility() {
  const uint3 idx = optixGetLaunchIndex();
  const int shooterIdx = idx.x;
  const int clusterIdx = idx.y;

  if (shooterIdx >= params.num_shooters ||
      clusterIdx >= params.num_visible_clusters)
    return;

  // Get shooter patch
  int shooterPatchID = params.shooter_patches[shooterIdx];
  const GPUPatch &shooter = params.vis_scene_data.patches[shooterPatchID];

  // Get target cluster
  int targetClusterID = params.visible_clusters[clusterIdx];

  // Iterate leaves in target cluster
  // Note: This is an inner loop inside the thread.
  // Ideally we'd parallelize this more if leaves are many.
  // But for a first pass, one thread per Cluster-Shooter pair is decent
  // granularity.

  int leafStart = params.vis_scene_data.clusterLeafOffsets[targetClusterID];
  int leafEnd = params.vis_scene_data.clusterLeafOffsets[targetClusterID + 1];

  for (int li = leafStart; li < leafEnd; li++) {
    int receiverPatchID = params.vis_scene_data.clusterLeafIndices[li];
    const GPUPatch &receiver = params.vis_scene_data.patches[receiverPatchID];

    // Skip self-face (don't shadow other patches on same face)
    if (receiver.faceNumber == shooter.faceNumber)
      continue;

    // Plane & Visibility Check Logic from vismat.cpp:
    // Test 1: Receiver origin must be in front of shooter's plane
    // if (DotProduct(patch2->origin, patch->normal) > patch->planeDist +
    // PLANE_TEST_EPSILON)
    float3 rxOrigin = receiver.origin;
    float3 sxNormal = shooter.normal;
    float dot1 = rxOrigin.x * sxNormal.x + rxOrigin.y * sxNormal.y +
                 rxOrigin.z * sxNormal.z;

    // PLANE_TEST_EPSILON 0.01
    if (dot1 <= shooter.planeDist + 0.01f)
      continue;

    // Test 2: Shooter origin must be in front of receiver's plane
    // (from TestPatchToFace pre-filter logic)
    float3 sxOrigin = shooter.origin;
    float3 rxNormal = receiver.normal;
    float dot2 = sxOrigin.x * rxNormal.x + sxOrigin.y * rxNormal.y +
                 sxOrigin.z * rxNormal.z;

    if (dot2 <= receiver.planeDist + 0.01f)
      continue;

    {
      // Setup Ray
      // p1 = shooter.origin + shooter.normal
      // p2 = receiver.origin + receiver.normal (SDK 2013 behavior)

      float3 startPos = make_float3(shooter.origin.x + shooter.normal.x,
                                    shooter.origin.y + shooter.normal.y,
                                    shooter.origin.z + shooter.normal.z);

      float3 endPos = make_float3(receiver.origin.x + receiver.normal.x,
                                  receiver.origin.y + receiver.normal.y,
                                  receiver.origin.z + receiver.normal.z);

      float3 dir = make_float3(endPos.x - startPos.x, endPos.y - startPos.y,
                               endPos.z - startPos.z);
      float len = sqrtf(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);

      if (len < 1e-4f)
        continue;

      float3 dirNorm = make_float3(dir.x / len, dir.y / len, dir.z / len);

      // Trace Visibility Ray
      unsigned int p0, p1, p2, p3, p4, p5, p6;
      p5 = (unsigned int)-1;      // Skip ID - don't self collide, but we offset
                                  // origin anyway
      p6 = __float_as_uint(0.0f); // Coverage accumulator

      // optixTrace — use OPTIX_RAY_FLAG_NONE so any-hit runs for
      // both skip_id filtering and texture shadow transparency
      optixTrace(
          params.traversable, startPos, dirNorm,
          0.0f,        // tmin
          len - 1e-4f, // tmax (fixed epsilon subtraction instead of scaling)
          0.0f, OptixVisibilityMask(255),
          OPTIX_RAY_FLAG_NONE, // Enable any-hit for texture shadows
          0, 1, 0, p0, p1, p2, p3, p4, p5, p6); // Payload

      // Check result
      // Our __miss__ sets hit_id to -1. __closesthit__ sets it to primIdx.
      int hitID = (int)p1;

      // If hitID is -1 (Miss), then the path is clear -> VISIBLE
      if (hitID == -1) {
        // Record Visibility
        int idx = atomicAdd(params.pair_count_atomic, 1);
        // Check bounds? (host handles overflow check on readback, but writing
        // OOB is bad) We really should pass buffer size. For now assuming 2M
        // is enough or we risk crash if extremely visible scene. Ideally we
        // add a bounds check here.

        if (idx < params.max_pairs) {
          params.visible_pairs[idx].shooter = shooterPatchID;
          params.visible_pairs[idx].receiver = receiverPatchID;
        }
      }
    }
  }
}

//=============================================================================
// Phase 2: Direct Lighting Kernel
//
// Each thread processes ONE lightmap sample.
// For each sample, iterates over all lights visible from the sample's PVS
// cluster, computes falloff + dot product (replicating CPU
// GatherSampleStandardLightSSE math), traces an inline shadow ray for
// occlusion, and atomicAdd's the contribution to the output buffer.
//
// Handles all light types: emit_point (0), emit_surface (1), emit_spotlight
// (2), emit_skylight (3), and emit_skyambient (5). Sky/ambient use TraceSkyRay
// for visibility; CPU only stores sunAmount.
//=============================================================================

// DIST_EPSILON from Source Engine (bspfile.h)
#define GPU_DIST_EPSILON 0.03125f
// MAX_TRACE_LENGTH = sqrt(3) * 2 * MAX_COORD_INTEGER (Source Engine
// worldsize.h)
#define GPU_MAX_TRACE_LENGTH 56755.84f

//-----------------------------------------------------------------------------
// Inline shadow trace: returns 1.0 if visible, 0.0 if occluded
//-----------------------------------------------------------------------------
__forceinline__ __device__ float TraceShadowRay(float3 origin, float3 direction,
                                                float tmax) {
  // Shadow ray with true transparency support
  // Use OPTIX_RAY_FLAG_NONE to allow any-hit for skip_id and alpha accumulation
  unsigned int p0 = __float_as_uint(1e30f);
  unsigned int p1 = (unsigned int)-1;
  unsigned int p2 = 0, p3 = 0, p4 = 0;
  unsigned int p5 = (unsigned int)-1;      // No skip ID for shadow rays
  unsigned int p6 = __float_as_uint(0.0f); // Coverage accumulator

  optixTrace(params.traversable, origin, direction,
             1e-3f, // tmin: avoid self-shadow artifacts
             tmax,  // tmax: distance to light
             0.0f,  // rayTime
             OptixVisibilityMask(255), OPTIX_RAY_FLAG_NONE, 0, 1,
             0, // SBT offset, stride, miss index
             p0, p1, p2, p3, p4, p5, p6);

  // Return visibility from additive coverage model (matches CPU)
  float coverage = __uint_as_float(p6);
  int hitPrim = (int)p1;

  // Miss = ray reached end without hitting anything solid
  // Return fractional visibility: 1.0 - accumulated coverage
  if (hitPrim == -1)
    return 1.0f - coverage;

  // Closest hit was accepted (opaque surface, or coverage reached 1.0)
  // Either way, the path to the light is fully blocked.
  return 0.0f;
}

//-----------------------------------------------------------------------------
// Inline sky visibility trace: returns fractional visibility [0.0–1.0]
// "Does hit sky" semantics with true transparency support.
//   - Miss = ray escapes to sky → visible (transparency from any-hit)
//   - Hit sky triangle (TRACE_ID_SKY) → visible (transparency from any-hit)
//   - Hit solid geometry → blocked (0.0, transparency already 0 from any-hit)
//-----------------------------------------------------------------------------
__forceinline__ __device__ float TraceSkyRay(float3 origin, float3 direction,
                                             float tmax) {
  unsigned int p0 = __float_as_uint(1e30f);
  unsigned int p1 = (unsigned int)-1;
  unsigned int p2 = 0, p3 = 0, p4 = 0;
  unsigned int p5 = (unsigned int)-1;
  unsigned int p6 = __float_as_uint(0.0f); // Coverage accumulator

  optixTrace(params.traversable, origin, direction,
             1e-3f, // tmin
             tmax,  // tmax
             0.0f,  // rayTime
             OptixVisibilityMask(255), OPTIX_RAY_FLAG_NONE, 0, 1,
             0, // SBT offset, stride, miss index
             p0, p1, p2, p3, p4, p5, p6);

  float coverage = __uint_as_float(p6);
  int hitPrim = (int)p1;

  // Miss → sky visible, return fractional visibility
  if (hitPrim == -1)
    return 1.0f - coverage;

  // Hit sky triangle → sky visible through any transparent surfaces
  if (params.triangles[hitPrim].triangle_id & TRACE_ID_SKY_GPU)
    return 1.0f - coverage;

  // Hit solid (non-sky) geometry → fully blocked
  return 0.0f;
}

//-----------------------------------------------------------------------------
// Direct Lighting Ray Generation
// 1D launch: one thread per lightmap sample
//-----------------------------------------------------------------------------
extern "C" __global__ void __raygen__direct_lighting() {
  const int sampleIdx = optixGetLaunchIndex().x;

  if (sampleIdx >= params.num_samples)
    return;

  // Load sample data
  const GPUSampleData &sample = params.d_samples[sampleIdx];
  float3 samplePos =
      make_float3(sample.position.x, sample.position.y, sample.position.z);
  float3 sampleNormal =
      make_float3(sample.normal.x, sample.normal.y, sample.normal.z);
  int clusterIdx = sample.clusterIndex;

  // If sample has no valid cluster, skip (can't look up lights)
  if (clusterIdx < 0 || clusterIdx >= params.num_clusters)
    return;

  // Load per-face bump info
  const GPUFaceInfo &faceInfo = params.d_faceInfos[sample.faceIndex];
  int normalCount = faceInfo.normalCount; // 1 or 4

  // Build bump normal array for this face
  // bumpNormal[0] = flat normal (from sample), bumpNormal[1..3] = bump basis
  float3 bumpNormals[4];
  bumpNormals[0] = sampleNormal;
  if (normalCount > 1) {
    bumpNormals[1] = make_float3(faceInfo.bumpNormal0_x, faceInfo.bumpNormal0_y,
                                 faceInfo.bumpNormal0_z);
    bumpNormals[2] = make_float3(faceInfo.bumpNormal1_x, faceInfo.bumpNormal1_y,
                                 faceInfo.bumpNormal1_z);
    bumpNormals[3] = make_float3(faceInfo.bumpNormal2_x, faceInfo.bumpNormal2_y,
                                 faceInfo.bumpNormal2_z);
  }

  // Look up which lights are visible from this cluster
  const GPUClusterLightList &clusterList = params.d_clusterLists[clusterIdx];
  int lightOffset = clusterList.lightOffset;
  int lightCount = clusterList.lightCount;

  // Per-style accumulators: [style_slot][bump_vector]
  // Matches CPU's MAXLIGHTMAPS (4) style slots per face.
  float accumR[4][4] = {};
  float accumG[4][4] = {};
  float accumB[4][4] = {};

  // Style→slot mapping: styleSlots[s] = lightstyle value for slot s
  int styleSlots[4] = {-1, -1, -1, -1};
  int numStyles = 0;

  // Separate sun/sky accumulators removed — CPU evaluates sky now.
  // Only point/surface/spot lights are accumulated on GPU.

  // Iterate over all lights visible from this cluster
  for (int li = 0; li < lightCount; li++) {
    int lightIdx = params.d_clusterLightIndices[lightOffset + li];
    if (lightIdx < 0 || lightIdx >= params.num_lights)
      continue;

    const GPULight &light = params.d_lights[lightIdx];

    // Resolve this light's style to a slot index (0–3)
    int slot = -1;
    for (int s = 0; s < numStyles; s++) {
      if (styleSlots[s] == light.style) {
        slot = s;
        break;
      }
    }
    if (slot < 0) {
      if (numStyles < 4) {
        slot = numStyles++;
        styleSlots[slot] = light.style;
      } else {
        continue; // overflow — matches CPU's FindOrAllocateLightstyleSamples
      }
    }

    // ---------------------------------------------------------------
    // Sky light types — directional lights with no position.
    // Evaluated on GPU using TraceSkyRay for shadow testing.
    // ---------------------------------------------------------------
    if (light.type == EMIT_SKYLIGHT) {
      // Sun direction = -lightNormal (sun shines in negative normal direction)
      float3 sunDir =
          make_float3(-light.normal_x, -light.normal_y, -light.normal_z);

      // Dot product: how much the surface faces the sun
      float dot = sampleNormal.x * sunDir.x + sampleNormal.y * sunDir.y +
                  sampleNormal.z * sunDir.z;
      dot = fmaxf(dot, 0.0f);
      if (dot <= 0.0f)
        continue;

      // Offset origin along surface normal to avoid self-intersection
      float3 offsetOrigin =
          make_float3(samplePos.x + sampleNormal.x * GPU_DIST_EPSILON,
                      samplePos.y + sampleNormal.y * GPU_DIST_EPSILON,
                      samplePos.z + sampleNormal.z * GPU_DIST_EPSILON);

      // Trace sky visibility — point sun or area sun
      float totalVis = 0.0f;
      int nSunSamples =
          (params.sunAngularExtent > 0.0f && params.numSkyDirs > 0)
              ? params.numSunSamples
              : 1;
      if (nSunSamples <= 0)
        nSunSamples = 1;

      for (int s = 0; s < nSunSamples; s++) {
        float3 dir = sunDir;
        if (s > 0 && params.d_skyDirs != nullptr) {
          // Jitter sun direction using pre-computed hemisphere samples
          // Scale by angular extent, offset from base sun direction
          int dirIdx = (s - 1) % params.numSkyDirs;
          float3 jitter = params.d_skyDirs[dirIdx];
          dir.x = sunDir.x + jitter.x * params.sunAngularExtent;
          dir.y = sunDir.y + jitter.y * params.sunAngularExtent;
          dir.z = sunDir.z + jitter.z * params.sunAngularExtent;
          // Normalize the jittered direction
          float len = sqrtf(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
          if (len > 1e-6f) {
            float invLen = 1.0f / len;
            dir.x *= invLen;
            dir.y *= invLen;
            dir.z *= invLen;
          }
        }
        totalVis += TraceSkyRay(offsetOrigin, dir, GPU_MAX_TRACE_LENGTH);
      }

      float seeAmount = totalVis / (float)nSunSamples;
      if (seeAmount <= 0.0f)
        continue;

      // Accumulate: seeAmount * dot * intensity (matches CPU AddLight path)
      float scale0 = seeAmount * dot;
      accumR[slot][0] += scale0 * light.intensity_x;
      accumG[slot][0] += scale0 * light.intensity_y;
      accumB[slot][0] += scale0 * light.intensity_z;

      for (int n = 1; n < normalCount; n++) {
        float bDot = bumpNormals[n].x * sunDir.x + bumpNormals[n].y * sunDir.y +
                     bumpNormals[n].z * sunDir.z;
        bDot = fmaxf(bDot, 0.0f);
        float bScale = seeAmount * bDot;
        accumR[slot][n] += bScale * light.intensity_x;
        accumG[slot][n] += bScale * light.intensity_y;
        accumB[slot][n] += bScale * light.intensity_z;
      }
      continue;
    }

    if (light.type == EMIT_SKYAMBIENT) {
      // Ambient sky: sample hemisphere directions and trace for sky visibility
      if (params.numSkyDirs <= 0 || params.d_skyDirs == nullptr)
        continue;

      float3 offsetOrigin =
          make_float3(samplePos.x + sampleNormal.x * GPU_DIST_EPSILON,
                      samplePos.y + sampleNormal.y * GPU_DIST_EPSILON,
                      samplePos.z + sampleNormal.z * GPU_DIST_EPSILON);

      // Accumulate weighted sky visibility across hemisphere
      float ambientAccumR[4] = {0, 0, 0, 0};
      float ambientAccumG[4] = {0, 0, 0, 0};
      float ambientAccumB[4] = {0, 0, 0, 0};
      float totalDot0 = 0.0f;
      float possibleHitCount[4] = {0, 0, 0, 0};

      for (int j = 0; j < params.numSkyDirs; j++) {
        float3 skyDir = params.d_skyDirs[j];
        // Negate direction (d_skyDirs points outward from sphere center,
        // we need trace direction matching CPU's -anorm convention)
        float3 traceDir = make_float3(-skyDir.x, -skyDir.y, -skyDir.z);

        // Dot product with flat normal
        float dot0 = sampleNormal.x * traceDir.x + sampleNormal.y * traceDir.y +
                     sampleNormal.z * traceDir.z;
        if (dot0 <= 1e-6f)
          continue;

        totalDot0 += dot0;
        possibleHitCount[0] += 1.0f;

        // Track per-bump possibleHitCount (CPU: only counts if BOTH flat
        // AND bump normals have positive dot for this direction)
        for (int n = 1; n < normalCount; n++) {
          float bDot = bumpNormals[n].x * traceDir.x +
                       bumpNormals[n].y * traceDir.y +
                       bumpNormals[n].z * traceDir.z;
          if (bDot > 1e-6f)
            possibleHitCount[n] += 1.0f;
        }

        // Trace sky visibility
        float vis = TraceSkyRay(offsetOrigin, traceDir, GPU_MAX_TRACE_LENGTH);
        if (vis <= 0.0f)
          continue;

        // Accumulate: vis * dot * intensity for flat normal
        float w0 = vis * dot0;
        ambientAccumR[0] += w0 * light.intensity_x;
        ambientAccumG[0] += w0 * light.intensity_y;
        ambientAccumB[0] += w0 * light.intensity_z;

        // Bump normals
        for (int bn = 1; bn < normalCount; bn++) {
          float bDot = bumpNormals[bn].x * traceDir.x +
                       bumpNormals[bn].y * traceDir.y +
                       bumpNormals[bn].z * traceDir.z;
          if (bDot <= 1e-6f)
            continue;
          float w = vis * bDot;
          ambientAccumR[bn] += w * light.intensity_x;
          ambientAccumG[bn] += w * light.intensity_y;
          ambientAccumB[bn] += w * light.intensity_z;
        }
      }

      // Normalize (matches CPU GatherSampleAmbientSkySSE exactly)
      // CPU formula: result[n] = ambient_intensity[n] * possibleHitCount[0]
      //                          / (sumdot * possibleHitCount[n]) * intensity
      // GPU accumulators already include intensity, so:
      //   result[0] = ambientAccum[0] / sumdot
      //   result[n] = ambientAccum[n] * possibleHitCount[0] / (sumdot *
      //   possibleHitCount[n])
      if (totalDot0 > 0.0f) {
        float invSumDot = 1.0f / totalDot0;
        accumR[slot][0] += ambientAccumR[0] * invSumDot;
        accumG[slot][0] += ambientAccumG[0] * invSumDot;
        accumB[slot][0] += ambientAccumB[0] * invSumDot;

        for (int n = 1; n < normalCount; n++) {
          // Scale by possibleHitCount[0]/possibleHitCount[n] to compensate
          // for reduced hemisphere coverage of offset bump normals
          float bumpFactor = (possibleHitCount[n] > 0.0f)
                                 ? possibleHitCount[0] / possibleHitCount[n]
                                 : 1.0f;
          float factor = invSumDot * bumpFactor;
          accumR[slot][n] += ambientAccumR[n] * factor;
          accumG[slot][n] += ambientAccumG[n] * factor;
          accumB[slot][n] += ambientAccumB[n] * factor;
        }
      }
      continue;
    }

    // ---------------------------------------------------------------
    // Compute delta = lightOrigin - samplePos
    // ---------------------------------------------------------------
    float3 lightOrigin =
        make_float3(light.origin_x, light.origin_y, light.origin_z);
    float3 src = lightOrigin;

    float3 delta = make_float3(src.x - samplePos.x, src.y - samplePos.y,
                               src.z - samplePos.z);
    float dist2 = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;

    if (dist2 < 1e-10f)
      continue;

    float dist = sqrtf(dist2);
    float invDist = 1.0f / dist;
    delta.x *= invDist;
    delta.y *= invDist;
    delta.z *= invDist;

    // ---------------------------------------------------------------
    // Compute dot product for flat normal: N · delta
    // ---------------------------------------------------------------
    float dot = sampleNormal.x * delta.x + sampleNormal.y * delta.y +
                sampleNormal.z * delta.z;
    dot = fmaxf(dot, 0.0f);

    // ---------------------------------------------------------------
    // Hard falloff: zero contribution if past endFadeDistance
    // ---------------------------------------------------------------
    bool hasHardFalloff = (light.endFadeDistance > light.startFadeDistance);
    if (hasHardFalloff) {
      if (dist > light.endFadeDistance)
        continue;
    }

    // Clamp distance for falloff evaluation (CPU: max(1, min(dist, capDist)))
    float falloffEvalDist = fmaxf(dist, 1.0f);
    falloffEvalDist = fminf(falloffEvalDist, light.capDist);

    // ---------------------------------------------------------------
    // Compute falloff based on light type (matches CPU SSE exactly)
    // ---------------------------------------------------------------
    float falloff = 0.0f;
    float3 shadowOrigin = src;

    switch (light.type) {
    case EMIT_POINT: {
      // falloff = 1 / (constant + linear*d + quadratic*d²)
      float denom = light.constant_attn + light.linear_attn * falloffEvalDist +
                    light.quadratic_attn * falloffEvalDist * falloffEvalDist;
      falloff = (denom > 0.0f) ? (1.0f / denom) : 0.0f;
      break;
    }

    case EMIT_SURFACE: {
      // dot2 = -delta · lightNormal (how much light faces sample)
      float3 lightNormal =
          make_float3(light.normal_x, light.normal_y, light.normal_z);
      float dot2 = -(delta.x * lightNormal.x + delta.y * lightNormal.y +
                     delta.z * lightNormal.z);
      dot2 = fmaxf(dot2, 0.0f);

      if (dot <= 0.0f || dot2 <= 0.0f) {
        falloff = 0.0f;
        break;
      }

      // falloff = dot2 / dist²
      falloff = (dist2 > 0.0f) ? (dot2 / dist2) : 0.0f;

      // CPU offsets shadow origin along light normal by DIST_EPSILON
      shadowOrigin.x += lightNormal.x * GPU_DIST_EPSILON;
      shadowOrigin.y += lightNormal.y * GPU_DIST_EPSILON;
      shadowOrigin.z += lightNormal.z * GPU_DIST_EPSILON;
      break;
    }

    case EMIT_SPOTLIGHT: {
      float3 lightNormal =
          make_float3(light.normal_x, light.normal_y, light.normal_z);
      float dot2 = -(delta.x * lightNormal.x + delta.y * lightNormal.y +
                     delta.z * lightNormal.z);

      // Outside outer cone entirely? Skip
      if (dot2 <= light.stopdot2) {
        falloff = 0.0f;
        break;
      }

      // Zero dot if outside cone (CPU: dot = AndSIMD(inCone, dot))
      if (dot2 <= light.stopdot2)
        dot = 0.0f;

      // Point-light-style attenuation
      float denom = light.constant_attn + light.linear_attn * falloffEvalDist +
                    light.quadratic_attn * falloffEvalDist * falloffEvalDist;
      falloff = (denom > 0.0f) ? (1.0f / denom) : 0.0f;
      falloff *= dot2;

      // Fringe interpolation: between stopdot (inner) and stopdot2 (outer)
      if (dot2 <= light.stopdot) {
        float range = light.stopdot - light.stopdot2;
        float mult = (range > 0.0f) ? ((dot2 - light.stopdot2) / range) : 0.0f;
        mult = fminf(fmaxf(mult, 0.0f), 1.0f);

        // Apply exponent (CPU uses PowSIMD which is fixed-point)
        if (light.exponent != 0.0f && light.exponent != 1.0f) {
          mult = powf(mult, light.exponent);
        }
        falloff *= mult;
      }
      break;
    }

    default:
      continue;
    } // switch

    // ---------------------------------------------------------------
    // Hard falloff fade: quintic smoothstep
    // ---------------------------------------------------------------
    if (hasHardFalloff) {
      float range = light.endFadeDistance - light.startFadeDistance;
      float t =
          (range > 0.0f) ? ((dist - light.startFadeDistance) / range) : 0.0f;
      t = fminf(fmaxf(t, 0.0f), 1.0f);
      t = 1.0f - t;
      float fade = t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
      falloff *= fade;
    }

    // Quick check: if falloff * flat-normal dot is zero, skip entirely
    float contribution = falloff * dot;
    if (contribution <= 0.0f)
      continue;

    // ---------------------------------------------------------------
    // Shadow ray: trace from sample toward light
    // ---------------------------------------------------------------
    float3 offsetOrigin = make_float3(samplePos.x, samplePos.y, samplePos.z);

    float3 shadowDir = make_float3(shadowOrigin.x - offsetOrigin.x,
                                   shadowOrigin.y - offsetOrigin.y,
                                   shadowOrigin.z - offsetOrigin.z);
    float shadowDist =
        sqrtf(shadowDir.x * shadowDir.x + shadowDir.y * shadowDir.y +
              shadowDir.z * shadowDir.z);

    if (shadowDist < 1e-6f)
      continue;

    float invShadowDist = 1.0f / shadowDist;
    shadowDir.x *= invShadowDist;
    shadowDir.y *= invShadowDist;
    shadowDir.z *= invShadowDist;

    float visibility = TraceShadowRay(offsetOrigin, shadowDir, shadowDist);

    if (visibility <= 0.0f)
      continue;

    // ---------------------------------------------------------------
    // Accumulate per bump vector: falloff * dot[n] * visibility * intensity
    // dot[0] is the flat normal (already computed); dot[1..3] are bump basis
    // ---------------------------------------------------------------
    float scale0 = falloff * dot * visibility;
    accumR[slot][0] += scale0 * light.intensity_x;
    accumG[slot][0] += scale0 * light.intensity_y;
    accumB[slot][0] += scale0 * light.intensity_z;

    // Compute per-bump-vector contributions (only for bumpmapped faces)
    for (int n = 1; n < normalCount; n++) {
      float bDot = bumpNormals[n].x * delta.x + bumpNormals[n].y * delta.y +
                   bumpNormals[n].z * delta.z;
      bDot = fmaxf(bDot, 0.0f);
      float bScale = falloff * bDot * visibility;
      accumR[slot][n] += bScale * light.intensity_x;
      accumG[slot][n] += bScale * light.intensity_y;
      accumB[slot][n] += bScale * light.intensity_z;
    }

  } // for each light

  // ---------------------------------------------------------------
  // Write results — no atomics needed (1D launch, one thread per sample)
  // ---------------------------------------------------------------
  for (int s = 0; s < numStyles; s++) {
    params.d_lightOutput[sampleIdx].styleMap[s] = styleSlots[s];

    for (int n = 0; n < normalCount; n++) {
      params.d_lightOutput[sampleIdx].r[s][n] = accumR[s][n];
      params.d_lightOutput[sampleIdx].g[s][n] = accumG[s][n];
      params.d_lightOutput[sampleIdx].b[s][n] = accumB[s][n];
    }
  }
}
