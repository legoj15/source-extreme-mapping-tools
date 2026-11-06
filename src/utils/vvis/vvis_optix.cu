#include "vvis_optix.h"
#include <cuda_runtime.h>
#include <optix.h>
#include <optix_device.h>

extern "C" __constant__ VVIS_OptixLaunchParams params;

// PRNG from OptiX SDK
static __forceinline__ __device__ unsigned int tea(unsigned int val0,
                                                   unsigned int val1) {
  unsigned int v0 = val0;
  unsigned int v1 = val1;
  unsigned int s0 = 0;
  for (unsigned int n = 0; n < 16; n++) {
    s0 += 0x9e3779b9;
    v0 += ((v1 << 4) + 0xa341316c) ^ (v1 + s0) ^ ((v1 >> 5) + 0xc8013ea4);
    v1 += ((v0 << 4) + 0xad90777d) ^ (v0 + s0) ^ ((v0 >> 5) + 0x7e95761e);
  }
  return v0;
}

static __forceinline__ __device__ float rnd(unsigned int &seed) {
  seed = (1664525 * seed + 1013904223);
  return (float)(seed & 0x00FFFFFF) / (float)0x01000000;
}

__device__ inline float3 get_stochastic_point(const VVIS_GPUPortal &portal,
                                              unsigned int &seed) {
  if (portal.numPoints < 3) {
    // Fallback to origin
    return make_float3(portal.origin_x, portal.origin_y, portal.origin_z);
  }

  // Pick random triangle in fan (0, i+1, i+2)
  int triIdx = (int)(rnd(seed) * (portal.numPoints - 2));
  if (triIdx >= portal.numPoints - 2)
    triIdx = portal.numPoints - 3;

  Vector v0 = params.winding_points[portal.windingOffset];
  Vector v1 = params.winding_points[portal.windingOffset + triIdx + 1];
  Vector v2 = params.winding_points[portal.windingOffset + triIdx + 2];

  float r1 = rnd(seed);
  float r2 = rnd(seed);
  if (r1 + r2 > 1.0f) {
    r1 = 1.0f - r1;
    r2 = 1.0f - r2;
  }
  float r0 = 1.0f - r1 - r2;

  return make_float3(v0.x * r0 + v1.x * r1 + v2.x * r2,
                     v0.y * r0 + v1.y * r1 + v2.y * r2,
                     v0.z * r0 + v1.z * r1 + v2.z * r2);
}

extern "C" __global__ void __raygen__portal_visibility() {
  const uint3 idx = optixGetLaunchIndex();
  const int shooterIdx = idx.x;
  const int receiverIdx = idx.y;

  if (shooterIdx >= params.num_portals || receiverIdx >= params.num_portals)
    return;
  if (shooterIdx == receiverIdx)
    return;

  // Fast rejection: Base PortalFlood check
  int byteIdx = receiverIdx >> 3;
  int bitMask = 1 << (receiverIdx & 7);
  if (!(params.portal_flood[shooterIdx * params.portal_bytes + byteIdx] &
        bitMask))
    return;

  const VVIS_GPUPortal &shooter = params.portals[shooterIdx];
  const VVIS_GPUPortal &receiver = params.portals[receiverIdx];

  unsigned int seed = tea(shooterIdx, receiverIdx);

  // Up to 4096 random rays per pair. Sampling can miss a narrow opening, and a
  // ray can slip through a gap the portal flow treats as closed, so the result
  // is never used as PVS; vvis.cpp only counts it against the exact CPU flow.
  const int NUM_RAYS = 4096;
  bool visible = false;

  // Stochastic Raycasting
  // For VVIS, any unoccluded ray proves the portals can see each other.
  for (int r = 0; r < NUM_RAYS; r++) {
    float3 p1;
    float3 p2;

    // Ray 0: centroid to centroid
    if (r == 0) {
      p1 = make_float3(shooter.origin_x, shooter.origin_y, shooter.origin_z);
      p2 = make_float3(receiver.origin_x, receiver.origin_y, receiver.origin_z);
    } else {
      p1 = get_stochastic_point(shooter, seed);
      p2 = get_stochastic_point(receiver, seed);
    }

    float3 dir = make_float3(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
    float len = sqrtf(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);

    if (len < 1e-4f)
      continue;

    dir.x /= len;
    dir.y /= len;
    dir.z /= len;

    // The portal flow only moves forward through a portal (along its normal,
    // into the leaf it leads to). A ray that leaves the shooter backwards or
    // reaches the receiver from its far side is not a line the flow can
    // follow, so it says nothing about this pair.
    if (dir.x * shooter.normal_x + dir.y * shooter.normal_y +
            dir.z * shooter.normal_z <= 0.0f ||
        dir.x * receiver.normal_x + dir.y * receiver.normal_y +
            dir.z * receiver.normal_z <= 0.0f)
      continue;

    // Use physical portal points, no manual coordinate munging!
    // We let Optix natively ignore t < 0.05f to avoid epsilon micro-collisions
    // at the ray source or destination boundaries.
    float traceLen = len - 0.05f;
    if (traceLen <= 0.05f)
      continue;

    unsigned int p0_out, p1_out, p2_out;
    p1_out = 0; // miss flag

    optixTrace(params.traversable, p1, dir, 0.05f, traceLen, 0.0f,
               OptixVisibilityMask(255),
               OPTIX_RAY_FLAG_DISABLE_ANYHIT |
                   OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT,
               0, 1, 0, p0_out, p1_out, p2_out);

    // hit_id in p1_out is set to 1 by __miss__, so 1 means miss, meaning path
    // is clear!
    if (p1_out == 1) {
      visible = true;
      break;
    }
  }

  if (visible) {
    // Correctly atomic OR into the unaligned portal_vis byte array
    unsigned char *address =
        &params.portal_vis[shooterIdx * params.portal_bytes + byteIdx];
    unsigned int *base_address = (unsigned int *)((size_t)address & ~3);
    unsigned int selectors = ((size_t)address & 3) * 8;
    unsigned int mask = bitMask << selectors;
    atomicOr(base_address, mask);
  }
}

extern "C" __global__ void __miss__visibility() {
  // We set payload to 1 on miss
  optixSetPayload_1(1);
}

extern "C" __global__ void __closesthit__visibility() {
  // We set payload to 0 on hit
  optixSetPayload_1(0);
}

extern "C" __global__ void __anyhit__visibility() {
  // Should be disabled by flags, but just in case
}
