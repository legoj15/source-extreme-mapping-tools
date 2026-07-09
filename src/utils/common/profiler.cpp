//========= Copyright Valve Corporation, All rights reserved. ============//
//
// Purpose:
//
//=============================================================================//

#include "profiler.h"
#include "tier0/dbg.h"
#include <pdh.h>
#include <pdhmsg.h>
#include <windows.h>

// ---------------------------------------------------------------------------
// NVML (NVIDIA Management Library) is loaded at runtime via LoadLibrary /
// GetProcAddress instead of being linked against nvml.lib. nvml.dll ships only
// with the NVIDIA driver, so a hard import would make vvis_optix_dll.dll fail
// to load on machines without an NVIDIA GPU -- taking the advertised CPU
// fallback down with it. Dynamic loading degrades gracefully to "GPU profiling
// unavailable" instead, and also removes the build-time dependency on the CUDA
// toolkit's nvml.h being on the include path. This mirrors the approach already
// used in src/utils/vrad/hardware_profiling.cpp.
//
// The minimal type/enum/prototype definitions below are ABI-compatible subsets
// of <nvml.h> -- just enough for the handful of calls this profiler makes.
// ---------------------------------------------------------------------------

// NVML return codes (nvmlReturn_t is an int-sized enum in nvml.h)
#define NVML_SUCCESS 0
#define NVML_ERROR_INSUFFICIENT_SIZE 7

// Opaque device handle (nvmlDevice_t)
typedef void *nvmlDevice_t;

// GPU-wide utilization rates (matches nvmlUtilization_t)
struct nvmlUtilization_t {
  unsigned int gpu;    // percent of time one or more kernels were executing
  unsigned int memory; // percent of time device memory was being accessed
};

// Per-process utilization sample. This layout MUST match nvml.h's
// nvmlProcessUtilizationSample_t exactly, because the driver writes an array of
// these into our buffer:
//   unsigned int       pid;         // offset 0
//   <4 bytes padding to 8-byte-align timeStamp>
//   unsigned long long timeStamp;   // offset 8
//   unsigned int       smUtil;      // offset 16
//   unsigned int       memUtil;     // offset 20
//   unsigned int       encUtil;     // offset 24
//   unsigned int       decUtil;     // offset 28
// => 32 bytes total on x64 with natural alignment.
struct nvmlProcessUtilizationSample_t {
  unsigned int pid;
  unsigned long long timeStamp;
  unsigned int smUtil;
  unsigned int memUtil;
  unsigned int encUtil;
  unsigned int decUtil;
};

// NVML entry points, as function pointers.
typedef int (*PFN_nvmlInit)(void);
typedef int (*PFN_nvmlShutdown)(void);
typedef const char *(*PFN_nvmlErrorString)(int);
typedef int (*PFN_nvmlDeviceGetHandleByIndex)(unsigned int, nvmlDevice_t *);
typedef int (*PFN_nvmlDeviceGetProcessUtilization)(
    nvmlDevice_t, nvmlProcessUtilizationSample_t *, unsigned int *,
    unsigned long long);
typedef int (*PFN_nvmlDeviceGetUtilizationRates)(nvmlDevice_t,
                                                 nvmlUtilization_t *);

static HMODULE s_hNVML = NULL;
static PFN_nvmlShutdown s_pfnShutdown = NULL;
static PFN_nvmlErrorString s_pfnErrorString = NULL;
static PFN_nvmlDeviceGetProcessUtilization s_pfnGetProcessUtil = NULL;
static PFN_nvmlDeviceGetUtilizationRates s_pfnGetUtilRates = NULL;

// Unload nvml.dll and null every resolved entry point, so no stale pointer can
// outlive the module. Safe to call whether or not the library is loaded.
static void FreeNVMLLibrary() {
  if (s_hNVML) {
    FreeLibrary(s_hNVML);
    s_hNVML = NULL;
  }
  s_pfnShutdown = NULL;
  s_pfnErrorString = NULL;
  s_pfnGetProcessUtil = NULL;
  s_pfnGetUtilRates = NULL;
}

// Load nvml.dll, initialize NVML, and resolve a handle to GPU 0. On success,
// returns true and stores the device handle in *pDevice; on any failure returns
// false, unloads the library, and leaves the s_pfn* pointers safe to ignore.
static bool LoadNVML(nvmlDevice_t *pDevice) {
  *pDevice = NULL;

  // nvml.dll is on PATH once the driver is installed; fall back to the classic
  // NVSMI folder and System32, matching vrad's hardware_profiling.cpp.
  s_hNVML = LoadLibraryA("nvml.dll");
  if (!s_hNVML) {
    char path[MAX_PATH];
    if (GetEnvironmentVariableA("ProgramFiles", path, MAX_PATH)) {
      strcat_s(path, "\\NVIDIA Corporation\\NVSMI\\nvml.dll");
      s_hNVML = LoadLibraryA(path);
    }
  }
  if (!s_hNVML) {
    char path[MAX_PATH];
    if (GetSystemDirectoryA(path, MAX_PATH)) {
      strcat_s(path, "\\nvml.dll");
      s_hNVML = LoadLibraryA(path);
    }
  }
  if (!s_hNVML)
    return false; // No NVIDIA driver present -- GPU profiling unavailable.

  // nvml.h #defines the base names onto _v2 variants; prefer those, fall back.
  PFN_nvmlInit pfnInit = (PFN_nvmlInit)GetProcAddress(s_hNVML, "nvmlInit_v2");
  if (!pfnInit)
    pfnInit = (PFN_nvmlInit)GetProcAddress(s_hNVML, "nvmlInit");

  PFN_nvmlDeviceGetHandleByIndex pfnGetHandle =
      (PFN_nvmlDeviceGetHandleByIndex)GetProcAddress(
          s_hNVML, "nvmlDeviceGetHandleByIndex_v2");
  if (!pfnGetHandle)
    pfnGetHandle = (PFN_nvmlDeviceGetHandleByIndex)GetProcAddress(
        s_hNVML, "nvmlDeviceGetHandleByIndex");

  s_pfnShutdown = (PFN_nvmlShutdown)GetProcAddress(s_hNVML, "nvmlShutdown");
  s_pfnErrorString =
      (PFN_nvmlErrorString)GetProcAddress(s_hNVML, "nvmlErrorString");
  s_pfnGetProcessUtil = (PFN_nvmlDeviceGetProcessUtilization)GetProcAddress(
      s_hNVML, "nvmlDeviceGetProcessUtilization");
  s_pfnGetUtilRates = (PFN_nvmlDeviceGetUtilizationRates)GetProcAddress(
      s_hNVML, "nvmlDeviceGetUtilizationRates");

  if (!pfnInit || !pfnGetHandle) {
    FreeNVMLLibrary();
    return false;
  }

  // Don't warn on init failure -- the user may simply not have an NVIDIA GPU.
  if (pfnInit() != NVML_SUCCESS) {
    FreeNVMLLibrary();
    return false;
  }

  nvmlDevice_t device = NULL;
  int result = pfnGetHandle(0, &device);
  if (result != NVML_SUCCESS) {
    if (s_pfnErrorString)
      Warning("Failed to get NVML device handle for GPU 0: %s\n",
              s_pfnErrorString(result));
    if (s_pfnShutdown)
      s_pfnShutdown();
    FreeNVMLLibrary();
    return false;
  }

  *pDevice = device;
  return true;
}

CProfiler g_Profiler;

CProfiler::CProfiler() {
  m_bInitialized = false;
  m_hPdhQuery = NULL;
  m_hPdhCounter = NULL;
  m_bNvmlInitialized = false;
  m_NvmlDevice = NULL;
}

CProfiler::~CProfiler() { Shutdown(); }

void CProfiler::Initialize() {
  if (m_bInitialized)
    return;

  // --- CPU Profiling Setup (PDH) ---
  PDH_STATUS status = PdhOpenQuery(NULL, 0, (HQUERY *)&m_hPdhQuery);
  if (status == ERROR_SUCCESS) {
    // Add counter for total processor time
    // We use the "Processor(_Total)\% Processor Time" counter (English)
    // Alternatively, PdhAddEnglishCounter is safer for non-English Windows
    status = PdhAddEnglishCounter((HQUERY)m_hPdhQuery,
                                  "\\Processor(_Total)\\% Processor Time", 0,
                                  (HCOUNTER *)&m_hPdhCounter);
    if (status != ERROR_SUCCESS) {
      // Fallback to localized name if english fails (though English API should
      // work)
      status = PdhAddCounter((HQUERY)m_hPdhQuery,
                             "\\Processor(_Total)\\% Processor Time", 0,
                             (HCOUNTER *)&m_hPdhCounter);
    }

    if (status == ERROR_SUCCESS) {
      // Collect initial data
      PdhCollectQueryData((HQUERY)m_hPdhQuery);
    } else {
      m_hPdhQuery = NULL;
      m_hPdhCounter = NULL;
      Warning("Failed to initialize CPU profiling counter. Error: 0x%x\n",
              status);
    }
  } else {
    Warning("Failed to open PDH query for CPU profiling. Error: 0x%x\n",
            status);
  }

  // --- GPU Profiling Setup (NVML, dynamically loaded) ---
  nvmlDevice_t device = NULL;
  if (LoadNVML(&device)) {
    m_NvmlDevice = device;
    m_bNvmlInitialized = true;
  }

  m_bInitialized = true;
}

void CProfiler::Shutdown() {
  if (m_hPdhQuery) {
    PdhCloseQuery((HQUERY)m_hPdhQuery);
    m_hPdhQuery = NULL;
    m_hPdhCounter = NULL;
  }

  if (m_bNvmlInitialized) {
    if (s_pfnShutdown)
      s_pfnShutdown();
    FreeNVMLLibrary();
    m_bNvmlInitialized = false;
    m_NvmlDevice = NULL;
  }

  m_bInitialized = false;
}

double CProfiler::GetCpuUsage() {
  if (!m_hPdhCounter)
    return 0.0;

  PDH_FMT_COUNTERVALUE counterVal;
  PDH_STATUS status = PdhCollectQueryData((HQUERY)m_hPdhQuery);

  if (status == ERROR_SUCCESS) {
    status = PdhGetFormattedCounterValue((HCOUNTER)m_hPdhCounter,
                                         PDH_FMT_DOUBLE, NULL, &counterVal);
    if (status == ERROR_SUCCESS) {
      return counterVal.doubleValue;
    }
  }

  return 0.0;
}

double CProfiler::GetGpuUsage() {
  if (!m_bNvmlInitialized || !m_NvmlDevice)
    return -1.0;

  // Try to get specific process utilization first (Compute/CUDA isolation)
  unsigned int pid = GetCurrentProcessId();
  nvmlProcessUtilizationSample_t *pSamples = NULL;
  unsigned int sampleCount = 0;
  unsigned long long lastSeenTimestamp = 0;

  // Default to "query unavailable" so we fall through to global utilization if
  // the driver doesn't export the process-utilization entry point.
  int result = NVML_ERROR_INSUFFICIENT_SIZE;

  if (s_pfnGetProcessUtil) {
    // First call to get count
    result = s_pfnGetProcessUtil((nvmlDevice_t)m_NvmlDevice, NULL, &sampleCount,
                                 lastSeenTimestamp);

    if (result == NVML_ERROR_INSUFFICIENT_SIZE && sampleCount > 0) {
      pSamples = new nvmlProcessUtilizationSample_t[sampleCount];
      if (pSamples) {
        // Just to be safe, checking allocation
        result = s_pfnGetProcessUtil((nvmlDevice_t)m_NvmlDevice, pSamples,
                                     &sampleCount, lastSeenTimestamp);
      }
    }
  }

  double usage = -1.0;

  if (result == NVML_SUCCESS && pSamples) {
    // We successfully got process samples. Defaults to 0 if we are not found
    // (idle).
    usage = 0.0;
    for (unsigned int i = 0; i < sampleCount; i++) {
      if (pSamples[i].pid == pid) {
        // smUtil returns percentage of time SM was active
        usage += (double)pSamples[i].smUtil;
      }
    }
    // Note: If we didn't find our PID, it means we had 0 utilization in the
    // last window, so usage=0.0 is correct.
  }

  if (pSamples) {
    delete[] pSamples;
  }

  // Fallback to global utilization if process-specific query failed (e.g. not
  // supported)
  if (usage < 0.0) {
    if (s_pfnGetUtilRates) {
      nvmlUtilization_t utilization;
      result = s_pfnGetUtilRates((nvmlDevice_t)m_NvmlDevice, &utilization);

      if (result == NVML_SUCCESS) {
        return (double)utilization.gpu;
      }
    }
  } else {
    return usage;
  }

  return -1.0;
}
