#include "cuda_runtime.h"
#include "tier0/dbg.h"
#include "vis.h"
#include "vvis_gpu.h"
#include "vvis_optix.h"
#include <algorithm>
#include <vector>

// Scale factor for float precision
#define VIS_EPSILON 0.01f

#define CHECK_CUDA(call)                                                       \
  {                                                                            \
    cudaError_t err = call;                                                    \
    if (err != cudaSuccess) {                                                  \
      Msg("CUDA Error at %s:%d: %s\n", __FILE__, __LINE__,                     \
          cudaGetErrorString(err));                                            \
      exit(1);                                                                 \
    }                                                                          \
  }

// ----------------------------------------------------------------------------------
// Data Structures (Device & Host)
// ----------------------------------------------------------------------------------

#define MAX_POINTS_ON_WINDING_GPU 16
#define MAX_DEPTH_GPU 10

struct CUDAPortal {
  Vector origin;
  float radius;
  Vector planeNormal;
  float planeDist;
  int numPoints;
  int windingOffset; // Index into global g_dAllWindingPoints
  int leaf;          // The leaf this portal leads TO
};

struct CUDALeaf {
  int portalOffset; // Index into g_dLeafPortals
  int numPortals;
};

struct CUDAWinding {
  int numpoints;
  Vector points[MAX_POINTS_ON_WINDING_GPU];
};

// ----------------------------------------------------------------------------------
// Global Device Pointers
// ----------------------------------------------------------------------------------

__constant__ int c_numPortals;
__constant__ int c_portalClusters;
__constant__ int c_portalBytes;
__constant__ float c_visRadiusClient;
__constant__ bool c_bUseRadiusClient;

static CUDAPortal *g_dPortals = nullptr;
static Vector *g_dWindingPoints = nullptr;

static unsigned char *g_dPortalVis = nullptr;

// ----------------------------------------------------------------------------------
// Helper Math (Device)
// ----------------------------------------------------------------------------------

__device__ inline float DotProductGPU(const Vector &a, const Vector &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

__device__ inline void CrossProductGPU(const Vector &a, const Vector &b,
                                       Vector &result) {
  result.x = a.y * b.z - a.z * b.y;
  result.y = a.z * b.x - a.x * b.z;
  result.z = a.x * b.y - a.y * b.x;
}

__device__ inline void VectorSubtractGPU(const Vector &a, const Vector &b,
                                         Vector &c) {
  c.x = a.x - b.x;
  c.y = a.y - b.y;
  c.z = a.z - b.z;
}

__device__ inline void VectorCopyGPU(const Vector &src, Vector &dst) {
  dst.x = src.x;
  dst.y = src.y;
  dst.z = src.z;
}

__device__ inline void VectorScaleGPU(Vector &v, float s) {
  v.x *= s;
  v.y *= s;
  v.z *= s;
}

__device__ inline void VectorNegateGPU(Vector &v) {
  v.x = -v.x;
  v.y = -v.y;
  v.z = -v.z;
}

__device__ inline void VectorNormalizeGPU(Vector &v) {
  float len = sqrtf(DotProductGPU(v, v));
  if (len > 1e-6f)
    VectorScaleGPU(v, 1.0f / len);
}

__device__ inline void VectorAddScaledGPU(const Vector &a, const Vector &dir,
                                          float t, Vector &out) {
  out.x = a.x + dir.x * t;
  out.y = a.y + dir.y * t;
  out.z = a.z + dir.z * t;
}

// Old CPU-emulation ray probes and chop functions deleted.

// ----------------------------------------------------------------------------------
// Kernels
// ----------------------------------------------------------------------------------

__global__ void BasePortalVisKernel(const CUDAPortal *portals,
                                    const Vector *windingPoints,
                                    unsigned char *outVis, int numPortals,
                                    int portalBytes) {
  int p1Idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (p1Idx >= numPortals)
    return;

  const CUDAPortal *p1 = &portals[p1Idx];
  unsigned char *myVisRow = outVis + (size_t)p1Idx * portalBytes;

  for (int p2Idx = 0; p2Idx < numPortals; p2Idx++) {
    if (p1Idx == p2Idx)
      continue;
    const CUDAPortal *p2 = &portals[p2Idx];

    bool p2_hasFront = false;
    for (int k = 0; k < p2->numPoints; k++) {
      Vector pt;
      VectorCopyGPU(windingPoints[p2->windingOffset + k], pt);
      float d = DotProductGPU(pt, p1->planeNormal) - p1->planeDist;
      if (d > VIS_EPSILON) {
        p2_hasFront = true;
        break;
      }
    }
    if (!p2_hasFront)
      continue;

    bool p1_hasFront = false;
    for (int k = 0; k < p1->numPoints; k++) {
      Vector pt;
      VectorCopyGPU(windingPoints[p1->windingOffset + k], pt);
      float d = DotProductGPU(pt, p2->planeNormal) - p2->planeDist;
      if (d < -VIS_EPSILON) {
        p1_hasFront = true;
        break;
      }
    }
    if (!p1_hasFront)
      continue;

    // If using radius visibility, cull the target portal when its nearest
    // winding point lies outside the vis radius from p1's origin. Mirrors the
    // CPU BasePortalVis g_bUseRadius test (flow.cpp) so -cuda output matches
    // the CPU PVS on fog-culled maps (env_fog_controller farz > 0).
    if (c_bUseRadiusClient) {
      float minDist2 = 1024000000.0f; // 32000^2 sentinel, matches CPU
      for (int k = 0; k < p2->numPoints; k++) {
        Vector seg;
        VectorSubtractGPU(windingPoints[p2->windingOffset + k], p1->origin, seg);
        float dist2 = DotProductGPU(seg, seg);
        if (dist2 < minDist2)
          minDist2 = dist2;
      }
      if (minDist2 > c_visRadiusClient)
        continue;
    }

    myVisRow[p2Idx >> 3] |= (1 << (p2Idx & 7));
  }
}

// ----------------------------------------------------------------------------------
// Host Interface
// ----------------------------------------------------------------------------------

void RunBasePortalVisCUDA() {
  Msg("Starting CUDA BasePortalVis...\n");
  int numPortals = g_numportals * 2;
  std::vector<CUDAPortal> hPortals(numPortals);
  std::vector<Vector> hWindingPoints;
  for (int i = 0; i < numPortals; i++) {
    hPortals[i].origin = portals[i].origin;
    hPortals[i].radius = portals[i].radius;
    hPortals[i].planeNormal = portals[i].plane.normal;
    hPortals[i].planeDist = portals[i].plane.dist;
    hPortals[i].numPoints = portals[i].winding->numpoints;
    hPortals[i].windingOffset = (int)hWindingPoints.size();
    hPortals[i].leaf = portals[i].leaf;
    for (int k = 0; k < portals[i].winding->numpoints; k++)
      hWindingPoints.push_back(portals[i].winding->points[k]);
  }
  CHECK_CUDA(cudaMalloc(&g_dPortals, numPortals * sizeof(CUDAPortal)));
  CHECK_CUDA(cudaMemcpy(g_dPortals, hPortals.data(),
                        numPortals * sizeof(CUDAPortal),
                        cudaMemcpyHostToDevice));
  CHECK_CUDA(
      cudaMalloc(&g_dWindingPoints, hWindingPoints.size() * sizeof(Vector)));
  CHECK_CUDA(cudaMemcpy(g_dWindingPoints, hWindingPoints.data(),
                        hWindingPoints.size() * sizeof(Vector),
                        cudaMemcpyHostToDevice));
  int visSize = numPortals * portalbytes;
  CHECK_CUDA(cudaMalloc(&g_dPortalVis, visSize));
  CHECK_CUDA(cudaMemset(g_dPortalVis, 0, visSize));
  float r = (float)g_VisRadius;
  cudaMemcpyToSymbol(c_visRadiusClient, &r, sizeof(float));
  cudaMemcpyToSymbol(c_bUseRadiusClient, &g_bUseRadius, sizeof(bool));
  cudaMemcpyToSymbol(c_numPortals, &numPortals, sizeof(int));
  cudaMemcpyToSymbol(c_portalBytes, &portalbytes, sizeof(int));
  cudaDeviceSetLimit(cudaLimitStackSize, 16384);
  int blockSize = 128;
  int numBlocks = (numPortals + blockSize - 1) / blockSize;
  BasePortalVisKernel<<<numBlocks, blockSize>>>(
      g_dPortals, g_dWindingPoints, g_dPortalVis, numPortals, portalbytes);
  cudaDeviceSynchronize();
  std::vector<unsigned char> hVis(visSize);
  cudaMemcpy(hVis.data(), g_dPortalVis, visSize, cudaMemcpyDeviceToHost);

  int totalBaseVis = 0;
  for (int i = 0; i < numPortals; i++) {
    // Allocate portalfront, portalflood, portalvis (same as CPU BasePortalVis)
    if (!portals[i].portalfront) {
      portals[i].portalfront = (byte *)malloc(portalbytes);
      memset(portals[i].portalfront, 0, portalbytes);
    }
    memcpy(portals[i].portalfront, &hVis[i * portalbytes], portalbytes);

    if (!portals[i].portalflood) {
      portals[i].portalflood = (byte *)malloc(portalbytes);
    }
    memset(portals[i].portalflood, 0, portalbytes);

    if (!portals[i].portalvis) {
      portals[i].portalvis = (byte *)malloc(portalbytes);
    }
    memset(portals[i].portalvis, 0, portalbytes);

    // Count bits for debug
    for (int j = 0; j < portalbytes; j++) {
      unsigned char b = hVis[i * portalbytes + j];
      while (b) {
        if (b & 1)
          totalBaseVis++;
        b >>= 1;
      }
    }
  }
  Msg("CUDA BasePortalVis Total Bits Set: %d\n", totalBaseVis);

  // Run SimpleFlood on CPU for each portal to populate portalflood
  // (required for CPU PortalFlow to function correctly)
  for (int i = 0; i < numPortals; i++) {
    SimpleFlood(&portals[i], portals[i].leaf);
    portals[i].nummightsee = CountBits(portals[i].portalflood, numPortals);
  }

  cudaFree(g_dPortals);
  cudaFree(g_dWindingPoints);
  cudaFree(g_dPortalVis);
  g_dPortals = nullptr;
  g_dWindingPoints = nullptr;
  g_dPortalVis = nullptr;
  Msg("CUDA BasePortalVis Finished.\n");
}

void RunPortalFlowCUDA() {
  Msg("Starting CUDA PortalFlow (OptiX Hardware Raytracing)...\n");
  int numPortals = g_numportals * 2;

  std::vector<VVIS_GPUPortal> hPortals(numPortals);
  std::vector<Vector> hWindingPoints;

  for (int i = 0; i < numPortals; i++) {
    hPortals[i].origin_x = portals[i].origin.x;
    hPortals[i].origin_y = portals[i].origin.y;
    hPortals[i].origin_z = portals[i].origin.z;
    hPortals[i].normal_x = portals[i].plane.normal.x;
    hPortals[i].normal_y = portals[i].plane.normal.y;
    hPortals[i].normal_z = portals[i].plane.normal.z;
    hPortals[i].dist = portals[i].plane.dist;
    hPortals[i].numPoints = portals[i].winding->numpoints;
    hPortals[i].windingOffset = (int)hWindingPoints.size();
    for (int k = 0; k < portals[i].winding->numpoints; k++)
      hWindingPoints.push_back(portals[i].winding->points[k]);
  }

  Msg("Pre-calculating SimpleFlood on CPU...\n");
  std::vector<unsigned char> hFlood(numPortals * portalbytes);
  for (int i = 0; i < numPortals; i++) {
    if (!portals[i].portalflood) {
      portals[i].portalflood = (byte *)malloc(portalbytes);
      memset(portals[i].portalflood, 0, portalbytes);
    }
    SimpleFlood(&portals[i], portals[i].leaf);
    memcpy(&hFlood[i * portalbytes], portals[i].portalflood, portalbytes);
  }

  if (!CVVisOptiX::Initialize()) {
    Error("Failed to initialize OptiX for VVIS.\n");
  }
  CVVisOptiX::BuildScene();

  VVIS_GPUPortal *d_portals = nullptr;
  Vector *d_windingPoints = nullptr;
  unsigned char *d_portalFlood = nullptr;
  unsigned char *d_portalVis = nullptr;

  CHECK_CUDA(cudaMalloc(&d_portals, numPortals * sizeof(VVIS_GPUPortal)));
  CHECK_CUDA(cudaMemcpy(d_portals, hPortals.data(),
                        numPortals * sizeof(VVIS_GPUPortal),
                        cudaMemcpyHostToDevice));

  CHECK_CUDA(
      cudaMalloc(&d_windingPoints, hWindingPoints.size() * sizeof(Vector)));
  CHECK_CUDA(cudaMemcpy(d_windingPoints, hWindingPoints.data(),
                        hWindingPoints.size() * sizeof(Vector),
                        cudaMemcpyHostToDevice));

  CHECK_CUDA(cudaMalloc(&d_portalFlood, numPortals * portalbytes));
  CHECK_CUDA(cudaMemcpy(d_portalFlood, hFlood.data(), numPortals * portalbytes,
                        cudaMemcpyHostToDevice));

  CHECK_CUDA(cudaMalloc(&d_portalVis, numPortals * portalbytes));
  CHECK_CUDA(cudaMemset(d_portalVis, 0, numPortals * portalbytes));

  CVVisOptiX::TraceVisibility(numPortals, portalbytes, d_portals,
                              d_windingPoints, d_portalFlood, d_portalVis);

  std::vector<unsigned char> hOutVis(numPortals * portalbytes);
  CHECK_CUDA(cudaMemcpy(hOutVis.data(), d_portalVis, numPortals * portalbytes,
                        cudaMemcpyDeviceToHost));

  for (int i = 0; i < numPortals; i++) {
    if (!portals[i].portalvis)
      portals[i].portalvis = (byte *)malloc(portalbytes);
    memcpy(portals[i].portalvis, &hOutVis[i * portalbytes], portalbytes);
    portals[i].status = stat_done;
  }

  cudaFree(d_portals);
  cudaFree(d_windingPoints);
  cudaFree(d_portalFlood);
  cudaFree(d_portalVis);

  CVVisOptiX::Shutdown();
  Msg("CUDA OptiX PortalFlow Finished.\n");
}
