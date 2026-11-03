// optix_kernels.cu - OptiX 9.1 device programs for vrad_nextgen
// Compiled to PTX by compile_ptx.bat

#include <optix.h>

struct PointLight {
  float posX, posY, posZ;
  float colR, colG, colB;
  float constAttn, linearAttn, quadAttn;
  // Falloff control (matching vrad_rtx SetLightFalloffParams):
  //   capDist            - max distance for attenuation polynomial eval (default 1e22)
  //   startFadeDistance   - distance at which quintic fade begins (default 0)
  //   endFadeDistance     - distance at which light is fully faded out (default -1 = disabled)
  float capDist;
  float startFadeDistance;
  float endFadeDistance;
};

struct GPUFaceInfo {
  float lmS[4];
  float lmT[4];
  int lmMinsS, lmMinsT;
  int lmW, lmH;
  int luxelOffset;
  float reflR, reflG, reflB;
  float normalX, normalY, normalZ;
};

struct GPUPatch {
  float originX, originY, originZ;
  float normalX, normalY, normalZ;
  float unshot_energyR, unshot_energyG, unshot_energyB;
  float total_energyR, total_energyG, total_energyB;
  float reflectivityR, reflectivityG, reflectivityB;
};

struct LaunchParams {
  OptixTraversableHandle traversable;
  float *outputBuffer;
  int numSamples;
  float sunDirX, sunDirY, sunDirZ;
  float sunColorR, sunColorG, sunColorB;
  int hasSun;
  PointLight *lights;
  int numLights;
  float *samplePositions;
  float *sampleNormals;
  int *triToFace;
  GPUFaceInfo *faceInfos;
  GPUPatch *patches;
  int numPatches;
  int *luxelToPatch;
  int bouncePassIndex; // -1 = direct, 0 = patch solve, -2 = final gather
  int numBounceRays;
  int totalLuxels;
};


extern "C" {
__constant__ LaunchParams params;
}

__device__ unsigned int pcg_hash(unsigned int input) {
  unsigned int state = input * 747796405u + 2891336453u;
  unsigned int word = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
  return (word >> 22u) ^ word;
}

__device__ unsigned int next_pcg(unsigned int &state) {
  unsigned int oldstate = state;
  state = oldstate * 747796405u + 2891336453u;
  unsigned int word =
      ((oldstate >> ((oldstate >> 28u) + 4u)) ^ oldstate) * 277803737u;
  return (word >> 22u) ^ word;
}

__device__ float pcg_float(unsigned int &state) {
  unsigned int r = next_pcg(state);
  return (float)(r & 0x00FFFFFF) / (float)0x01000000;
}

__device__ void cosine_hemisphere(float u1, float u2, float nx, float ny,
                                  float nz, float &dx, float &dy, float &dz) {
  float r = sqrtf(u1);
  float theta = 6.283185307f * u2;
  float lx = r * cosf(theta);
  float ly = r * sinf(theta);
  float lz = sqrtf(1.0f - u1);

  float upX, upY, upZ;
  if (fabsf(nx) < 0.9f) {
    upX = 1;
    upY = 0;
    upZ = 0;
  } else {
    upX = 0;
    upY = 1;
    upZ = 0;
  }

  float tx = upY * nz - upZ * ny;
  float ty = upZ * nx - upX * nz;
  float tz = upX * ny - upY * nx;
  float tlen = sqrtf(tx * tx + ty * ty + tz * tz);
  if (tlen < 1e-8f) {
    dx = nx;
    dy = ny;
    dz = nz;
    return;
  }
  tx /= tlen;
  ty /= tlen;
  tz /= tlen;

  float bx = ny * tz - nz * ty;
  float by = nz * tx - nx * tz;
  float bz = nx * ty - ny * tx;

  dx = lx * tx + ly * bx + lz * nx;
  dy = lx * ty + ly * by + lz * ny;
  dz = lx * tz + ly * bz + lz * nz;
}

extern "C" __global__ void __raygen__lightmap() {
  const uint3 idx = optixGetLaunchIndex();
  int sampleIdx = idx.x;
  if (sampleIdx >= params.numSamples)
    return;

  float px = params.samplePositions[sampleIdx * 3 + 0];
  float py = params.samplePositions[sampleIdx * 3 + 1];
  float pz = params.samplePositions[sampleIdx * 3 + 2];

  float nx = params.sampleNormals[sampleIdx * 3 + 0];
  float ny = params.sampleNormals[sampleIdx * 3 + 1];
  float nz = params.sampleNormals[sampleIdx * 3 + 2];

  float totalR = 0, totalG = 0, totalB = 0;

  // ---------------------------------------------------------------------------
  // Surface offset bias (1.0 unit along face normal)
  // This matches vrad_rtx's ComputeIlluminationPointAndNormalsSSE which does:
  //   pInfo->m_Points += faceNormal;
  // before any light gathering. The biased position is used for BOTH the shadow
  // ray origin AND the light distance/NdotL computation.
  // NOTE: This offset may need to be revisited or removed in the future if it
  //       introduces artifacts. It exists to prevent self-intersection with the
  //       emitting surface.  0.5 was too small — caused self-intersection noise
  //       at high luxel density.
  // ---------------------------------------------------------------------------
  float bias = 1.0f;
  float bpx = px + nx * bias;
  float bpy = py + ny * bias;
  float bpz = pz + nz * bias;

  if (params.bouncePassIndex < 0) {
    if (params.hasSun) {
      float NdotL =
          nx * params.sunDirX + ny * params.sunDirY + nz * params.sunDirZ;
      if (NdotL > 0.0f) {
        unsigned int p0 = 0, p1 = 0;
        optixTrace(params.traversable,
                   make_float3(bpx, bpy, bpz),
                   make_float3(params.sunDirX, params.sunDirY, params.sunDirZ),
                   0.0f, 1e16f, 0.0f, OptixVisibilityMask(255),
                   OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT |
                       OPTIX_RAY_FLAG_DISABLE_CLOSESTHIT,
                   0, 1, 0, p0, p1);
        if (p0 == 0xFFFFFFFF) {
          totalR += NdotL * params.sunColorR;
          totalG += NdotL * params.sunColorG;
          totalB += NdotL * params.sunColorB;
        }
      }
    }
    for (int li = 0; li < params.numLights; li++) {
      PointLight light = params.lights[li];

      // Compute direction and distance from the BIASED position.
      // This matches vrad_rtx where ComputeIlluminationPointAndNormalsSSE
      // offsets sample positions by 1 unit along the face normal BEFORE
      // GatherSampleStandardLightSSE computes delta/dist.
      float dx = light.posX - bpx;
      float dy = light.posY - bpy;
      float dz = light.posZ - bpz;
      float distSq = dx * dx + dy * dy + dz * dz;
      if (distSq < 1e-6f)
        continue;
      float dist = sqrtf(distSq);
      float invDist = 1.0f / dist;
      float ldx = dx * invDist, ldy = dy * invDist, ldz = dz * invDist;
      float NdotL = nx * ldx + ny * ldy + nz * ldz;
      if (NdotL <= 0.0f)
        continue;

      // Early-out if past the end fade distance
      bool hasHardFalloff = (light.endFadeDistance > light.startFadeDistance);
      if (hasHardFalloff && dist > light.endFadeDistance)
        continue;

      // Attenuation: clamp dist >= 1, then cap at capDist for polynomial eval
      // (matching vrad_rtx GatherSampleStandardLightSSE lines 2052-2071)
      float attenDist = fmaxf(dist, 1.0f);
      float falloffEvalDist = fminf(attenDist, light.capDist);
      float atten = light.constAttn +
                    light.linearAttn * falloffEvalDist +
                    light.quadAttn * falloffEvalDist * falloffEvalDist;
      if (atten < 1e-6f)
        atten = 1e-6f;
      float falloff = 1.0f / atten;

      // Fade region: quintic interpolating polynomial t³(t(t*6-15)+10)
      // (matching vrad_rtx GatherSampleStandardLightSSE lines 2140-2157)
      if (hasHardFalloff) {
        float fadeRange = light.endFadeDistance - light.startFadeDistance;
        float t = (dist - light.startFadeDistance) / fadeRange;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        t = 1.0f - t;
        float fade = t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
        falloff *= fade;
      }

      // Shadow ray from biased position
      unsigned int p0 = 0, p1 = 0;
      optixTrace(params.traversable,
                 make_float3(bpx, bpy, bpz),
                 make_float3(ldx, ldy, ldz), 0.0f, dist, 0.0f,
                 OptixVisibilityMask(255),
                 OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT |
                     OPTIX_RAY_FLAG_DISABLE_CLOSESTHIT,
                 0, 1, 0, p0, p1);
      if (p0 == 0xFFFFFFFF) {
        totalR += NdotL * falloff * light.colR;
        totalG += NdotL * falloff * light.colG;
        totalB += NdotL * falloff * light.colB;
      }
    }
  } else if (params.bouncePassIndex == -2) {
    // Final gather: look up patch total_energy for each luxel
    int patchIndex = params.luxelToPatch[sampleIdx];
    GPUPatch p = params.patches[patchIndex];
    // Output direct (totalR... already accumulated from direct pass logic above) + patch radiosity
    totalR += p.total_energyR;
    totalG += p.total_energyG;
    totalB += p.total_energyB;
  }

  params.outputBuffer[sampleIdx * 3 + 0] = totalR;
  params.outputBuffer[sampleIdx * 3 + 1] = totalG;
  params.outputBuffer[sampleIdx * 3 + 2] = totalB;
}

extern "C" __global__ void __raygen__apply_bounce() {
  int patchIdx = optixGetLaunchIndex().x;
  if (patchIdx >= params.numPatches) return;
  
  float recR = params.outputBuffer[patchIdx * 3 + 0];
  float recG = params.outputBuffer[patchIdx * 3 + 1];
  float recB = params.outputBuffer[patchIdx * 3 + 2];
  
  GPUPatch p = params.patches[patchIdx];
  
  p.total_energyR += recR;
  p.total_energyG += recG;
  p.total_energyB += recB;
  
  p.unshot_energyR = recR * p.reflectivityR;
  p.unshot_energyG = recG * p.reflectivityG;
  p.unshot_energyB = recB * p.reflectivityB;
  
  params.patches[patchIdx] = p;
}

extern "C" __global__ void __raygen__radiosity_bounce() {
  int launchIdx = optixGetLaunchIndex().x;
  if (params.bouncePassIndex >= 0) {
    if (launchIdx >= params.numPatches) return;
    int patchIdx = launchIdx;

    GPUPatch patch = params.patches[patchIdx];
    unsigned int seed = pcg_hash(patchIdx * 1973 + params.bouncePassIndex * 9277);

    float receivedR = 0, receivedG = 0, receivedB = 0;

    for (int i = 0; i < params.numBounceRays; i++) {
      float r1 = pcg_float(seed);
      float r2 = pcg_float(seed);

      float dx, dy, dz;
      cosine_hemisphere(r1, r2, patch.normalX, patch.normalY, patch.normalZ, dx, dy, dz);

      unsigned int p0 = 0xFFFFFFFF, p1 = 0;
      float bias = 1.0f;
      optixTrace(params.traversable,
                 make_float3(patch.originX + patch.normalX * bias, 
                             patch.originY + patch.normalY * bias, 
                             patch.originZ + patch.normalZ * bias),
                 make_float3(dx, dy, dz), 0.0f, 99999.0f, 0.0f,
                 OptixVisibilityMask(255), OPTIX_RAY_FLAG_NONE, 0, 1, 0, p0,
                 p1);

      if (p0 != 0xFFFFFFFF) {
        int faceIndex = params.triToFace[p0];
        GPUFaceInfo fi = params.faceInfos[faceIndex];

        float t = __int_as_float(p1);
        float hx = patch.originX + patch.normalX * bias + dx * t;
        float hy = patch.originY + patch.normalY * bias + dy * t;
        float hz = patch.originZ + patch.normalZ * bias + dz * t;

        float u = fi.lmS[0] * hx + fi.lmS[1] * hy + fi.lmS[2] * hz + fi.lmS[3];
        float v = fi.lmT[0] * hx + fi.lmT[1] * hy + fi.lmT[2] * hz + fi.lmT[3];
        int luxU = (int)floorf(u) - fi.lmMinsS;
        int luxV = (int)floorf(v) - fi.lmMinsT;

        if (luxU < 0) luxU = 0;
        if (luxU >= fi.lmW) luxU = fi.lmW - 1;
        if (luxV < 0) luxV = 0;
        if (luxV >= fi.lmH) luxV = fi.lmH - 1;

        int offset = fi.luxelOffset + (luxV * fi.lmW + luxU);
        int hitPatchIndex = params.luxelToPatch[offset];

        GPUPatch hitPatch = params.patches[hitPatchIndex];

        receivedR += hitPatch.unshot_energyR;
        receivedG += hitPatch.unshot_energyG;
        receivedB += hitPatch.unshot_energyB;
      }
    }

    float invN = 1.0f / (float)params.numBounceRays;
    params.outputBuffer[patchIdx * 3 + 0] = receivedR * invN;
    params.outputBuffer[patchIdx * 3 + 1] = receivedG * invN;
    params.outputBuffer[patchIdx * 3 + 2] = receivedB * invN;
  } else if (params.bouncePassIndex == -2) {
    if (launchIdx >= params.numSamples) return;
    int sampleIdx = launchIdx;
    
    int patchIndex = params.luxelToPatch[sampleIdx];
    GPUPatch p = params.patches[patchIndex];
    
    // totalR... are not tracked in this pass automatically across kernels,
    // actually wait: final gather should WRITE to outputBuffer.
    // The previous pass calculated direct lighting. We need to add to it.
    float totalR = params.outputBuffer[sampleIdx * 3 + 0];
    float totalG = params.outputBuffer[sampleIdx * 3 + 1];
    float totalB = params.outputBuffer[sampleIdx * 3 + 2];
    
    totalR += p.total_energyR;
    totalG += p.total_energyG;
    totalB += p.total_energyB;
    
    params.outputBuffer[sampleIdx * 3 + 0] = totalR;
    params.outputBuffer[sampleIdx * 3 + 1] = totalG;
    params.outputBuffer[sampleIdx * 3 + 2] = totalB;
  }
}

extern "C" __global__ void __closesthit__radiance() {
  optixSetPayload_0(optixGetPrimitiveIndex());
  float t = optixGetRayTmax();
  optixSetPayload_1(*(unsigned int *)&t);
}

extern "C" __global__ void __miss__radiance() { optixSetPayload_0(0xFFFFFFFF); }
