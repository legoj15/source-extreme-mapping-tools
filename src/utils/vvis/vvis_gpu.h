#ifndef VVIS_GPU_H
#define VVIS_GPU_H

#ifdef _WIN32
#pragma once
#endif

// Fills g_pPortalFlowHints from OptiX portal-to-portal rays (see vvis_gpu.cu).
// Returns false, with the reason in `reason`, if the pass cannot run; it never
// exits, since the PVS does not depend on it.
bool RunPortalFlowCUDA(char *reason, int reasonSize);

#endif // VVIS_GPU_H
