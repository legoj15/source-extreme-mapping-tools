#include "cuda_runtime.h"
#include "tier0/dbg.h"
#include "tier1/strtools.h"
#include "vis.h"
#include "vvis_gpu.h"
#include "vvis_optix.h"
#include <limits.h>
#include <vector>

// __FILE__ is the full build path under /FC; the reason only needs the name.
static const char *BaseName(const char *path) {
  const char *name = path;
  for (const char *c = path; *c; c++) {
    if (*c == '\\' || *c == '/')
      name = c + 1;
  }
  return name;
}

// The ray pass below is optional (the PVS never depends on it), so a failed
// CUDA call writes its reason to `reason` and makes the function return false
// instead of exiting. Uses the `reason` and `reasonSize` of the enclosing
// function.
#define HINT_CUDA(call)                                                       \
  {                                                                            \
    cudaError_t err = call;                                                    \
    if (err != cudaSuccess) {                                                  \
      V_snprintf(reason, reasonSize, "CUDA error '%s' at %s:%d",               \
                 cudaGetErrorString(err), BaseName(__FILE__), __LINE__);       \
      return false;                                                            \
    }                                                                          \
  }

namespace {

// Frees a device buffer when it goes out of scope, so every early return
// releases what was allocated.
struct DeviceBuffer {
  void *p = nullptr;
  ~DeviceBuffer() {
    if (p)
      cudaFree(p);
  }
};

// Shuts OptiX down on every way out, including a failed Initialize.
struct OptiXScope {
  ~OptiXScope() { CVVisOptiX::Shutdown(); }
};

} // namespace

// The CUDA BasePortalVis kernel that used to live here was removed: -cuda now
// runs the CPU BasePortalVis, because the flow must start from exactly the CPU
// portalflood (the kernel also ignored radius vis, g_bUseRadius).

static bool RunPortalFlowCUDAImpl(char *reason, int reasonSize) {
  Msg("Starting CUDA PortalFlow (OptiX Hardware Raytracing)...\n");
  int numPortals = g_numportals * 2;
  size_t floodBytes = (size_t)numPortals * portalbytes;

  // The ray kernel indexes the portal bit matrix with 32-bit math.
  if (floodBytes > UINT_MAX) {
    V_snprintf(reason, reasonSize,
               "the %llu byte portal bit matrix is past the 32-bit indexing of "
               "the ray kernel",
               (unsigned long long)floodBytes);
    return false;
  }

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

  // Declared before Initialize so a failed one is cleaned up too.
  OptiXScope optix;
  if (!CVVisOptiX::Initialize() || !CVVisOptiX::BuildScene()) {
    V_snprintf(reason, reasonSize, "%s", CVVisOptiX::LastError());
    return false;
  }

  DeviceBuffer portalsBuf, windingBuf, floodBuf, visBuf;

  HINT_CUDA(cudaMalloc(&portalsBuf.p, numPortals * sizeof(VVIS_GPUPortal)));
  HINT_CUDA(cudaMemcpy(portalsBuf.p, hPortals.data(),
                       numPortals * sizeof(VVIS_GPUPortal),
                       cudaMemcpyHostToDevice));

  HINT_CUDA(
      cudaMalloc(&windingBuf.p, hWindingPoints.size() * sizeof(Vector)));
  HINT_CUDA(cudaMemcpy(windingBuf.p, hWindingPoints.data(),
                       hWindingPoints.size() * sizeof(Vector),
                       cudaMemcpyHostToDevice));

  // portalflood comes from the CPU BasePortalVis that already ran. Rows go to
  // the device one by one so the whole matrix is never duplicated on the host.
  HINT_CUDA(cudaMalloc(&floodBuf.p, floodBytes));
  for (int i = 0; i < numPortals; i++)
    HINT_CUDA(cudaMemcpy((unsigned char *)floodBuf.p + (size_t)i * portalbytes,
                         portals[i].portalflood, portalbytes,
                         cudaMemcpyHostToDevice));

  HINT_CUDA(cudaMalloc(&visBuf.p, floodBytes));
  HINT_CUDA(cudaMemset(visBuf.p, 0, floodBytes));

  if (!CVVisOptiX::TraceVisibility(
          numPortals, portalbytes, (VVIS_GPUPortal *)portalsBuf.p,
          (Vector *)windingBuf.p, (unsigned char *)floodBuf.p,
          (unsigned char *)visBuf.p)) {
    V_snprintf(reason, reasonSize, "%s", CVVisOptiX::LastError());
    return false;
  }

  // Not PVS: a ray can slip through a gap the portal flow treats as closed,
  // and sampling misses narrow openings. vvis.cpp only reports it.
  byte *hints = (byte *)malloc(floodBytes);
  if (!hints) {
    V_snprintf(reason, reasonSize,
               "out of memory for the %llu byte hint matrix",
               (unsigned long long)floodBytes);
    return false;
  }
  cudaError_t copyErr =
      cudaMemcpy(hints, visBuf.p, floodBytes, cudaMemcpyDeviceToHost);
  if (copyErr != cudaSuccess) {
    free(hints);
    V_snprintf(reason, reasonSize, "CUDA error '%s' at %s:%d",
               cudaGetErrorString(copyErr), BaseName(__FILE__), __LINE__);
    return false;
  }

  free(g_pPortalFlowHints);
  g_pPortalFlowHints = hints;
  Msg("CUDA OptiX PortalFlow hints finished.\n");
  return true;
}

// GPU ray pass for -cuda: for every portalflood pair, OptiX shoots up to 4096
// rays from one portal to the other against the world brushes. A pair with a
// clear ray goes into g_pPortalFlowHints, which vvis.cpp only counts against
// the exact CPU flow's result; it never sets a PVS bit.
//
// Returns false, with the reason in `reason` and g_pPortalFlowHints left NULL,
// when the pass cannot run (no CUDA device, OptiX failed to start, a missing
// vvis_optix.ptx, out of memory, a failed launch). Nothing here exits. Only the
// small per-portal tables are std::vectors; the project builds without C++
// exceptions, so the big buffers are malloc'd and checked.
bool RunPortalFlowCUDA(char *reason, int reasonSize) {
  reason[0] = '\0';
  bool ok = RunPortalFlowCUDAImpl(reason, reasonSize);
  if (!ok) {
    free(g_pPortalFlowHints);
    g_pPortalFlowHints = NULL;
  }
  return ok;
}
