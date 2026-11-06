#ifndef VVIS_OPTIX_H
#define VVIS_OPTIX_H

#include "mathlib/vector.h"
#include "tier1/utlvector.h"

// Structs shared between Host (C++) and Device (PTX)
struct VVIS_GPUPortal {
  float origin_x, origin_y, origin_z;
  float normal_x, normal_y, normal_z;
  float dist;
  int numPoints;
  int windingOffset;
};

// Launch parameters
struct VVIS_OptixLaunchParams {
  unsigned int num_portals;
  unsigned int portal_bytes;
  VVIS_GPUPortal *portals;
  Vector *winding_points;
  unsigned char *portal_flood;
  unsigned char *portal_vis;
  unsigned long long traversable;
};

// Interface class. Initialize, BuildScene and TraceVisibility return false on
// failure instead of exiting, with the reason in LastError(); the -cuda hint
// pass built on them is optional and must not stop the compile. Shutdown frees
// whatever exists, so it is also the cleanup after a failed Initialize.
class CVVisOptiX {
public:
  static bool Initialize();
  static bool BuildScene();
  static bool TraceVisibility(int numPortals, int portalBytes,
                              VVIS_GPUPortal *d_portals, Vector *d_windings,
                              unsigned char *d_portalFlood,
                              unsigned char *d_portalVis);
  static void Shutdown();
  static bool IsInitialized();
  static const char *LastError();
};

#endif // VVIS_OPTIX_H
