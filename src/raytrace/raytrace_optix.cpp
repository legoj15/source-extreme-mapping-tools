//========================================================================
// raytrace_optix.cpp - OptiX 9.1 Hardware Ray Tracing for VRAD RTX
//========================================================================

#include "raytrace_optix.h"
#include "direct_lighting_gpu.h"
#include "raytrace.h"
#include "tier0/dbg.h"
#include "tier1/strtools.h"
#include "tier1/utlvector.h"

#include <cuda_runtime.h>
#include <optix.h>
#include <optix_function_table_definition.h>
#include <optix_stubs.h>

#include <fstream>
#include <vector>

//-----------------------------------------------------------------------------
// OptiX Error Handling
//-----------------------------------------------------------------------------
#define OPTIX_CHECK(call)                                                      \
  do {                                                                         \
    OptixResult res = call;                                                    \
    if (res != OPTIX_SUCCESS) {                                                \
      Warning("OptiX Error at %s:%d - %s\n", __FILE__, __LINE__,               \
              optixGetErrorName(res));                                         \
      return false;                                                            \
    }                                                                          \
  } while (0)

#define OPTIX_CHECK_VOID(call)                                                 \
  do {                                                                         \
    OptixResult res = call;                                                    \
    if (res != OPTIX_SUCCESS) {                                                \
      Warning("OptiX Error at %s:%d - %s\n", __FILE__, __LINE__,               \
              optixGetErrorName(res));                                         \
      return;                                                                  \
    }                                                                          \
  } while (0)

#define CUDA_CHECK(call)                                                       \
  do {                                                                         \
    cudaError_t err = call;                                                    \
    if (err != cudaSuccess) {                                                  \
      Warning("CUDA Error at %s:%d - %s\n", __FILE__, __LINE__,                \
              cudaGetErrorString(err));                                        \
      return false;                                                            \
    }                                                                          \
  } while (0)

#define CUDA_CHECK_VOID(call)                                                  \
  do {                                                                         \
    cudaError_t err = call;                                                    \
    if (err != cudaSuccess) {                                                  \
      Warning("CUDA Error at %s:%d - %s\n", __FILE__, __LINE__,                \
              cudaGetErrorString(err));                                        \
      return;                                                                  \
    }                                                                          \
  } while (0)

//-----------------------------------------------------------------------------
// Static member definitions
//-----------------------------------------------------------------------------
bool RayTraceOptiX::s_bInitialized = false;
char RayTraceOptiX::s_szDeviceName[256] = {0};
int RayTraceOptiX::s_nDeviceMemoryMB = 0;

void *RayTraceOptiX::s_context = nullptr;
void *RayTraceOptiX::s_module = nullptr;
void *RayTraceOptiX::s_pipeline = nullptr;
void *RayTraceOptiX::s_raygenPG = nullptr;
void *RayTraceOptiX::s_missPG = nullptr;
void *RayTraceOptiX::s_hitgroupPG = nullptr;

void *RayTraceOptiX::s_d_gas_output_buffer = nullptr;
unsigned long long RayTraceOptiX::s_gas_handle = 0;

void *RayTraceOptiX::s_d_sbt_buffer = nullptr;

CUDATriangle *RayTraceOptiX::s_d_triangles = nullptr;
int RayTraceOptiX::s_triangleCount = 0;

RayBatch *RayTraceOptiX::s_d_rays[RayTraceOptiX::NUM_BUFFERS] = {nullptr,
                                                                 nullptr};
RayResult *RayTraceOptiX::s_d_results[RayTraceOptiX::NUM_BUFFERS] = {nullptr,
                                                                     nullptr};
int RayTraceOptiX::s_maxBatchSize = 0;

void *RayTraceOptiX::s_d_launchParams[RayTraceOptiX::NUM_BUFFERS] = {nullptr,
                                                                     nullptr};

void *RayTraceOptiX::s_streams[RayTraceOptiX::NUM_BUFFERS] = {nullptr, nullptr};
RayBatch *RayTraceOptiX::s_h_rays_pinned[RayTraceOptiX::NUM_BUFFERS] = {
    nullptr, nullptr};
RayResult *RayTraceOptiX::s_h_results_pinned[RayTraceOptiX::NUM_BUFFERS] = {
    nullptr, nullptr};

// Visibility Buffers
GPUVisSceneData RayTraceOptiX::s_visData = {};
void *RayTraceOptiX::s_d_shooterPatches = nullptr;
void *RayTraceOptiX::s_d_visibleClusters = nullptr;
void *RayTraceOptiX::s_d_visiblePairs = nullptr;
void *RayTraceOptiX::s_d_pairCount = nullptr;
int RayTraceOptiX::s_maxVisibilityPairs = 0;
void *RayTraceOptiX::s_visRaygenPG = nullptr;
void *RayTraceOptiX::s_directLightingRaygenPG = nullptr;

CThreadMutex RayTraceOptiX::s_Mutex;

// Bounce GPU buffers
long long *RayTraceOptiX::s_d_csrOffsets = nullptr;
int *RayTraceOptiX::s_d_csrPatch = nullptr;
float *RayTraceOptiX::s_d_csrWeight = nullptr;
float3_t *RayTraceOptiX::s_d_reflectivity = nullptr;
float3_t *RayTraceOptiX::s_d_patchOrigin = nullptr;
float3_t *RayTraceOptiX::s_d_patchNormal = nullptr;
int *RayTraceOptiX::s_d_needsBumpmap = nullptr;
int *RayTraceOptiX::s_d_faceNumber = nullptr;
float3_t *RayTraceOptiX::s_d_emitlight = nullptr;
float3_t *RayTraceOptiX::s_d_addlight = nullptr;
float3_t *RayTraceOptiX::s_d_addlightBump = nullptr;
float3_t *RayTraceOptiX::s_d_bumpNormals = nullptr;
int RayTraceOptiX::s_bounceNumPatches = 0;
int RayTraceOptiX::s_bounceTotalTransfers = 0;
bool RayTraceOptiX::s_bounceInitialized = false;

// Sky direction sample buffer (device)
float3_t *RayTraceOptiX::s_d_skyDirs = nullptr;
int RayTraceOptiX::s_numSkyDirs = 0;
float RayTraceOptiX::s_sunAngularExtent = 0.0f;

// Texture Shadow Buffers
int *RayTraceOptiX::s_d_triMaterials = nullptr;
GPUTextureShadowTri *RayTraceOptiX::s_d_texShadowTris = nullptr;
unsigned char *RayTraceOptiX::s_d_alphaAtlas = nullptr;
bool RayTraceOptiX::s_textureShadowsEnabled = false;
bool RayTraceOptiX::s_backfaceWTShadowCull = false;
bool RayTraceOptiX::s_frontfaceWTShadowCull = false;

// Bounce GPU profiling accumulators (milliseconds from CUDA events)
static float s_bounceUploadMs = 0.0f;
static float s_bounceKernelMs = 0.0f;
static float s_bounceDownloadMs = 0.0f;
static int s_bounceProfileCount = 0;

//-----------------------------------------------------------------------------
// OptiX Launch Parameters - passed to GPU kernels (must match optix_kernels.cu)
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

  // Direct Lighting Extension (Phase 2) — must match optix_kernels.cu
  const GPUSampleData *d_samples;
  const GPULight *d_lights;
  const GPUClusterLightList *d_clusterLists;
  const int *d_clusterLightIndices;
  const GPUFaceInfo *d_faceInfos;
  GPULightOutput *d_lightOutput;
  int num_samples;
  int num_lights;
  int num_clusters;

  // Sky Light Extension (Phase 2b) — must match optix_kernels.cu
  const float3_t *d_skyDirs; // Precomputed hemisphere sample directions
  int numSkyDirs;            // Number of sky sample directions (162 default)
  float sunAngularExtent;    // Area sun jitter (0 = point sun)
  int numSunSamples;         // Samples for area sun (30 default, 0 = point sun)

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

// Embedded PTX (will be set during build or loaded from file)
static std::vector<char> s_ptxCode;

//-----------------------------------------------------------------------------
// OptiX logging callback
//-----------------------------------------------------------------------------
static void optixLogCallback(unsigned int level, const char *tag,
                             const char *message, void *cbdata) {
  if (level <= 2) {
    Warning("[OptiX %s] %s\n", tag, message);
  }
}

//-----------------------------------------------------------------------------
// Load PTX from file
//-----------------------------------------------------------------------------
static bool LoadPTX(const char *filename) {
  // Try loading from same directory as executable
  char fullPath[MAX_PATH];

  // First try current directory
  V_snprintf(fullPath, sizeof(fullPath), "%s", filename);

  std::ifstream file(fullPath, std::ios::binary | std::ios::ate);
  if (!file.is_open()) {
    // Try bin directory
    V_snprintf(fullPath, sizeof(fullPath), "bin\\x64\\%s", filename);
    file.open(fullPath, std::ios::binary | std::ios::ate);
  }

  if (!file.is_open()) {
    Warning("Failed to open PTX file: %s\n", filename);
    return false;
  }

  size_t size = file.tellg();
  file.seekg(0, std::ios::beg);

  s_ptxCode.resize(size + 1);
  if (!file.read(s_ptxCode.data(), size)) {
    Warning("Failed to read PTX file: %s\n", filename);
    return false;
  }
  s_ptxCode[size] = '\0';

  Msg("Loaded PTX: %s (%zu bytes)\n", filename, size);
  return true;
}

//-----------------------------------------------------------------------------
// Initialize OptiX
//-----------------------------------------------------------------------------
bool RayTraceOptiX::Initialize() {
  if (s_bInitialized)
    return true;

  Msg("Initializing OptiX 9.1 Hardware Ray Tracing...\n");

  // Initialize CUDA
  CUDA_CHECK(cudaFree(0)); // Initialize CUDA context

  int deviceCount = 0;
  CUDA_CHECK(cudaGetDeviceCount(&deviceCount));
  if (deviceCount == 0) {
    Warning("No CUDA-capable GPU found!\n");
    return false;
  }

  // Get device properties
  cudaDeviceProp prop;
  CUDA_CHECK(cudaGetDeviceProperties(&prop, 0));

  // Check for RTX capability (compute capability >= 7.5 for Turing)
  if (prop.major < 7 || (prop.major == 7 && prop.minor < 5)) {
    Warning(
        "GPU does not support RTX ray tracing (requires Turing or newer)\n");
    return false;
  }

  V_strncpy(s_szDeviceName, prop.name, sizeof(s_szDeviceName));
  s_nDeviceMemoryMB = (int)(prop.totalGlobalMem / (1024 * 1024));

  Msg("  Device: %s (RTX Capable)\n", s_szDeviceName);
  Msg("  Memory: %d MB\n", s_nDeviceMemoryMB);

  // Initialize OptiX
  OPTIX_CHECK(optixInit());

  // Create OptiX device context
  OptixDeviceContextOptions options = {};
  options.logCallbackFunction = optixLogCallback;
  options.logCallbackLevel = 3;

  CUcontext cuCtx = 0; // Use current CUDA context
  OPTIX_CHECK(optixDeviceContextCreate(cuCtx, &options,
                                       (OptixDeviceContext *)&s_context));

  // Load PTX
  if (!LoadPTX("optix_kernels.ptx")) {
    Warning("Failed to load OptiX PTX module\n");
    return false;
  }

  // Create module from PTX
  OptixModuleCompileOptions moduleOptions = {};
  moduleOptions.maxRegisterCount = OPTIX_COMPILE_DEFAULT_MAX_REGISTER_COUNT;
  moduleOptions.optLevel = OPTIX_COMPILE_OPTIMIZATION_DEFAULT;
  moduleOptions.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_NONE;

  OptixPipelineCompileOptions pipelineOptions = {};
  pipelineOptions.usesMotionBlur = false;
  pipelineOptions.traversableGraphFlags =
      OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_SINGLE_GAS;
  pipelineOptions.numPayloadValues =
      7; // hit_t, hit_id, normal.xyz, skip_id, transparency
  pipelineOptions.numAttributeValues = 2;
  pipelineOptions.exceptionFlags = OPTIX_EXCEPTION_FLAG_NONE;
  pipelineOptions.pipelineLaunchParamsVariableName = "params";

  char log[2048];
  size_t logSize = sizeof(log);

  OPTIX_CHECK(optixModuleCreate((OptixDeviceContext)s_context, &moduleOptions,
                                &pipelineOptions, s_ptxCode.data(),
                                s_ptxCode.size(), log, &logSize,
                                (OptixModule *)&s_module));

  if (logSize > 1)
    Msg("OptiX Module Log: %s\n", log);

  // Create program groups
  OptixProgramGroupOptions pgOptions = {};

  // Ray generation
  OptixProgramGroupDesc raygenDesc = {};
  raygenDesc.kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
  raygenDesc.raygen.module = (OptixModule)s_module;
  raygenDesc.raygen.entryFunctionName = "__raygen__visibility";

  logSize = sizeof(log);
  OPTIX_CHECK(optixProgramGroupCreate((OptixDeviceContext)s_context,
                                      &raygenDesc, 1, &pgOptions, log, &logSize,
                                      (OptixProgramGroup *)&s_raygenPG));

  // Visibility Ray Generation
  OptixProgramGroupDesc visRaygenDesc = {};
  visRaygenDesc.kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
  visRaygenDesc.raygen.module = (OptixModule)s_module;
  visRaygenDesc.raygen.entryFunctionName = "__raygen__cluster_visibility";

  logSize = sizeof(log);
  OPTIX_CHECK(optixProgramGroupCreate(
      (OptixDeviceContext)s_context, &visRaygenDesc, 1, &pgOptions, log,
      &logSize, (OptixProgramGroup *)&s_visRaygenPG));

  // Direct Lighting Ray Generation (Phase 2)
  OptixProgramGroupDesc dlRaygenDesc = {};
  dlRaygenDesc.kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
  dlRaygenDesc.raygen.module = (OptixModule)s_module;
  dlRaygenDesc.raygen.entryFunctionName = "__raygen__direct_lighting";

  logSize = sizeof(log);
  OPTIX_CHECK(optixProgramGroupCreate(
      (OptixDeviceContext)s_context, &dlRaygenDesc, 1, &pgOptions, log,
      &logSize, (OptixProgramGroup *)&s_directLightingRaygenPG));

  // Miss program
  OptixProgramGroupDesc missDesc = {};
  missDesc.kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
  missDesc.miss.module = (OptixModule)s_module;
  missDesc.miss.entryFunctionName = "__miss__visibility";

  logSize = sizeof(log);
  OPTIX_CHECK(optixProgramGroupCreate((OptixDeviceContext)s_context, &missDesc,
                                      1, &pgOptions, log, &logSize,
                                      (OptixProgramGroup *)&s_missPG));

  // Hit group
  OptixProgramGroupDesc hitgroupDesc = {};
  hitgroupDesc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
  hitgroupDesc.hitgroup.moduleCH = (OptixModule)s_module;
  hitgroupDesc.hitgroup.entryFunctionNameCH = "__closesthit__visibility";
  hitgroupDesc.hitgroup.moduleAH = (OptixModule)s_module;
  hitgroupDesc.hitgroup.entryFunctionNameAH = "__anyhit__visibility";

  logSize = sizeof(log);
  OPTIX_CHECK(optixProgramGroupCreate(
      (OptixDeviceContext)s_context, &hitgroupDesc, 1, &pgOptions, log,
      &logSize, (OptixProgramGroup *)&s_hitgroupPG));

  // Create pipeline — 5 program groups now (added directLightingRaygen)
  OptixProgramGroup programGroups[] = {
      (OptixProgramGroup)s_raygenPG, (OptixProgramGroup)s_missPG,
      (OptixProgramGroup)s_hitgroupPG, (OptixProgramGroup)s_visRaygenPG,
      (OptixProgramGroup)s_directLightingRaygenPG};

  OptixPipelineLinkOptions linkOptions = {};
  linkOptions.maxTraceDepth = 1;

  logSize = sizeof(log);
  OPTIX_CHECK(optixPipelineCreate(
      (OptixDeviceContext)s_context, &pipelineOptions, &linkOptions,
      programGroups, 5, log, &logSize, (OptixPipeline *)&s_pipeline));

  if (logSize > 1)
    Msg("OptiX Pipeline Log: %s\n", log);

  // Allocate double-buffered ray/result/param buffers for ping-pong pipelining
  s_maxBatchSize = 1000000; // 1M rays per batch
  for (int i = 0; i < NUM_BUFFERS; i++) {
    CUDA_CHECK(cudaMalloc(&s_d_rays[i], s_maxBatchSize * sizeof(RayBatch)));
    CUDA_CHECK(cudaMalloc(&s_d_results[i], s_maxBatchSize * sizeof(RayResult)));
    CUDA_CHECK(cudaMalloc(&s_d_launchParams[i], sizeof(OptixLaunchParams)));

    // Create CUDA stream for async operations
    CUDA_CHECK(cudaStreamCreate((cudaStream_t *)&s_streams[i]));

    // Allocate pinned (page-locked) host memory for async transfers
    CUDA_CHECK(cudaHostAlloc(&s_h_rays_pinned[i],
                             s_maxBatchSize * sizeof(RayBatch),
                             cudaHostAllocDefault));
    CUDA_CHECK(cudaHostAlloc(&s_h_results_pinned[i],
                             s_maxBatchSize * sizeof(RayResult),
                             cudaHostAllocDefault));
  }

  // Allocate visibility output buffers (start with 100M pairs)
  s_maxVisibilityPairs = 100000000;
  CUDA_CHECK(cudaMalloc(&s_d_visiblePairs,
                        s_maxVisibilityPairs * sizeof(VisiblePair)));
  CUDA_CHECK(cudaMalloc(&s_d_pairCount, sizeof(int)));

  // Temp buffers for cluster input
  CUDA_CHECK(cudaMalloc(&s_d_shooterPatches,
                        65536 * sizeof(int))); // Max patches in a cluster?
  CUDA_CHECK(cudaMalloc(&s_d_visibleClusters, 65536 * sizeof(int)));

  s_bInitialized = true;
  Msg("OptiX initialization complete.\n");
  return true;
}

//-----------------------------------------------------------------------------
// Query VRAM usage via cudaMemGetInfo
//-----------------------------------------------------------------------------
bool RayTraceOptiX::GetVRAMUsage(size_t &freeMB, size_t &totalMB) {
  if (!s_bInitialized) {
    freeMB = totalMB = 0;
    return false;
  }

  size_t freeBytes = 0, totalBytes = 0;
  cudaError_t err = cudaMemGetInfo(&freeBytes, &totalBytes);
  if (err != cudaSuccess) {
    freeMB = totalMB = 0;
    return false;
  }

  freeMB = freeBytes / (1024 * 1024);
  totalMB = totalBytes / (1024 * 1024);
  return true;
}

//-----------------------------------------------------------------------------
// Shutdown OptiX
//-----------------------------------------------------------------------------
void RayTraceOptiX::Shutdown() {
  if (!s_bInitialized)
    return;

  Msg("OptiX shutdown...\n");

  // Free double-buffered async streaming resources
  for (int i = 0; i < NUM_BUFFERS; i++) {
    if (s_streams[i]) {
      cudaStreamDestroy((cudaStream_t)s_streams[i]);
      s_streams[i] = nullptr;
    }
    if (s_h_rays_pinned[i]) {
      cudaFreeHost(s_h_rays_pinned[i]);
      s_h_rays_pinned[i] = nullptr;
    }
    if (s_h_results_pinned[i]) {
      cudaFreeHost(s_h_results_pinned[i]);
      s_h_results_pinned[i] = nullptr;
    }
    if (s_d_rays[i])
      cudaFree(s_d_rays[i]);
    if (s_d_results[i])
      cudaFree(s_d_results[i]);
    if (s_d_launchParams[i])
      cudaFree(s_d_launchParams[i]);
  }
  if (s_d_triangles)
    cudaFree(s_d_triangles);
  if (s_d_gas_output_buffer)
    cudaFree(s_d_gas_output_buffer);
  if (s_d_sbt_buffer)
    cudaFree(s_d_sbt_buffer);

  if (s_d_visiblePairs)
    cudaFree(s_d_visiblePairs);
  if (s_d_pairCount)
    cudaFree(s_d_pairCount);
  if (s_d_shooterPatches)
    cudaFree(s_d_shooterPatches);
  if (s_d_visibleClusters)
    cudaFree(s_d_visibleClusters);

  // Free scene data
  if (s_visData.clusterLeafOffsets)
    cudaFree(s_visData.clusterLeafOffsets);
  if (s_visData.clusterLeafIndices)
    cudaFree(s_visData.clusterLeafIndices);
  if (s_visData.patches)
    cudaFree((void *)s_visData.patches); // Cast required due to struct type
  if (s_visData.pvsData)
    cudaFree(s_visData.pvsData);

  // Free sky direction buffer
  FreeSkyDirections();

  // Free texture shadow buffers
  FreeTextureShadowData();

  if (s_pipeline)
    optixPipelineDestroy((OptixPipeline)s_pipeline);
  if (s_raygenPG)
    optixProgramGroupDestroy((OptixProgramGroup)s_raygenPG);
  if (s_visRaygenPG)
    optixProgramGroupDestroy((OptixProgramGroup)s_visRaygenPG);
  if (s_directLightingRaygenPG)
    optixProgramGroupDestroy((OptixProgramGroup)s_directLightingRaygenPG);
  if (s_missPG)
    optixProgramGroupDestroy((OptixProgramGroup)s_missPG);
  if (s_hitgroupPG)
    optixProgramGroupDestroy((OptixProgramGroup)s_hitgroupPG);
  if (s_module)
    optixModuleDestroy((OptixModule)s_module);
  if (s_context)
    optixDeviceContextDestroy((OptixDeviceContext)s_context);

  s_bInitialized = false;
  Msg("OptiX shutdown complete.\n");
}

//-----------------------------------------------------------------------------
// Build scene (acceleration structure) from triangles
//-----------------------------------------------------------------------------
void RayTraceOptiX::BuildScene(
    const CUtlBlockVector<CacheOptimizedTriangle> &triangles,
    const CUtlVector<CacheOptimizedKDNode> &nodes,
    const CUtlVector<int> &triangle_indices, const CUtlVector<Vector> &vertices,
    const Vector &scene_min, const Vector &scene_max) {

  if (!s_bInitialized) {
    Warning("RayTraceOptiX::BuildScene called before Initialize!\n");
    return;
  }

  Msg("Building OptiX acceleration structure...\n");

  // Validate vertex data
  int numTriangles = triangles.Count();
  s_triangleCount = numTriangles;

  if (vertices.Count() != numTriangles * 3) {
    Warning("RayTraceOptiX::BuildScene: vertex count mismatch! Expected %d, "
            "got %d\n",
            numTriangles * 3, vertices.Count());
    return;
  }

  // Convert vertices to float3 for OptiX
  std::vector<float3> optixVertices(numTriangles * 3);
  for (int i = 0; i < numTriangles * 3; i++) {
    const Vector &v = vertices[i];
    optixVertices[i] = make_float3(v.x, v.y, v.z);
  }

  // Upload vertices to GPU
  CUdeviceptr d_vertices;
  CUDA_CHECK_VOID(
      cudaMalloc((void **)&d_vertices, optixVertices.size() * sizeof(float3)));
  CUDA_CHECK_VOID(cudaMemcpy((void *)d_vertices, optixVertices.data(),
                             optixVertices.size() * sizeof(float3),
                             cudaMemcpyHostToDevice));

  // Build triangle indices (0, 1, 2, 3, 4, 5, ...)
  std::vector<uint32_t> indices(numTriangles * 3);
  for (int i = 0; i < numTriangles * 3; i++) {
    indices[i] = i;
  }

  CUdeviceptr d_indices;
  CUDA_CHECK_VOID(
      cudaMalloc((void **)&d_indices, indices.size() * sizeof(uint32_t)));
  CUDA_CHECK_VOID(cudaMemcpy((void *)d_indices, indices.data(),
                             indices.size() * sizeof(uint32_t),
                             cudaMemcpyHostToDevice));

  // Create build input
  OptixBuildInput buildInput = {};
  buildInput.type = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;

  buildInput.triangleArray.vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3;
  buildInput.triangleArray.vertexStrideInBytes = sizeof(float3);
  buildInput.triangleArray.numVertices = (uint32_t)optixVertices.size();
  buildInput.triangleArray.vertexBuffers = &d_vertices;

  buildInput.triangleArray.indexFormat = OPTIX_INDICES_FORMAT_UNSIGNED_INT3;
  buildInput.triangleArray.indexStrideInBytes = sizeof(uint32_t) * 3;
  buildInput.triangleArray.numIndexTriplets = numTriangles;
  buildInput.triangleArray.indexBuffer = d_indices;

  // Disable face culling to match CUDA's double-sided plane-equation
  // intersection
  uint32_t inputFlags[] = {OPTIX_GEOMETRY_FLAG_DISABLE_TRIANGLE_FACE_CULLING};
  buildInput.triangleArray.flags = inputFlags;
  buildInput.triangleArray.numSbtRecords = 1;

  // Get memory requirements
  OptixAccelBuildOptions accelOptions = {};
  accelOptions.buildFlags =
      OPTIX_BUILD_FLAG_ALLOW_COMPACTION | OPTIX_BUILD_FLAG_PREFER_FAST_TRACE;
  accelOptions.operation = OPTIX_BUILD_OPERATION_BUILD;

  OptixAccelBufferSizes bufferSizes;
  OPTIX_CHECK_VOID(optixAccelComputeMemoryUsage((OptixDeviceContext)s_context,
                                                &accelOptions, &buildInput, 1,
                                                &bufferSizes));

  // Allocate temp and output buffers
  CUdeviceptr d_temp;
  CUDA_CHECK_VOID(cudaMalloc((void **)&d_temp, bufferSizes.tempSizeInBytes));
  CUDA_CHECK_VOID(
      cudaMalloc(&s_d_gas_output_buffer, bufferSizes.outputSizeInBytes));

  // Build acceleration structure
  OPTIX_CHECK_VOID(optixAccelBuild(
      (OptixDeviceContext)s_context, 0, &accelOptions, &buildInput, 1, d_temp,
      bufferSizes.tempSizeInBytes, (CUdeviceptr)s_d_gas_output_buffer,
      bufferSizes.outputSizeInBytes, (OptixTraversableHandle *)&s_gas_handle,
      nullptr, 0));

  CUDA_CHECK_VOID(cudaDeviceSynchronize());

  // Free temp buffers
  cudaFree((void *)d_temp);
  cudaFree((void *)d_vertices);
  cudaFree((void *)d_indices);

  // Upload triangle data for hit shader
  CUDA_CHECK_VOID(
      cudaMalloc(&s_d_triangles, numTriangles * sizeof(CUDATriangle)));

  std::vector<CUDATriangle> cudaTriangles(numTriangles);
  for (int i = 0; i < numTriangles; i++) {
    const TriIntersectData_t &tri = triangles[i].m_Data.m_IntersectData;
    cudaTriangles[i].nx = tri.m_flNx;
    cudaTriangles[i].ny = tri.m_flNy;
    cudaTriangles[i].nz = tri.m_flNz;
    cudaTriangles[i].d = tri.m_flD;
    cudaTriangles[i].triangle_id = tri.m_nTriangleID;
    for (int j = 0; j < 6; j++) {
      cudaTriangles[i].edge_eqs[j] = tri.m_ProjectedEdgeEquations[j];
    }
    cudaTriangles[i].coord_select0 = tri.m_nCoordSelect0;
    cudaTriangles[i].coord_select1 = tri.m_nCoordSelect1;
    cudaTriangles[i].flags = tri.m_nFlags;
    cudaTriangles[i].unused = 0;
  }

  CUDA_CHECK_VOID(cudaMemcpy(s_d_triangles, cudaTriangles.data(),
                             numTriangles * sizeof(CUDATriangle),
                             cudaMemcpyHostToDevice));

  // Build Shader Binding Table
  struct SbtRecord {
    char header[OPTIX_SBT_RECORD_HEADER_SIZE];
  };

  SbtRecord raygenRecord, missRecord, hitgroupRecord;

  OPTIX_CHECK_VOID(
      optixSbtRecordPackHeader((OptixProgramGroup)s_raygenPG, &raygenRecord));
  OPTIX_CHECK_VOID(
      optixSbtRecordPackHeader((OptixProgramGroup)s_missPG, &missRecord));
  OPTIX_CHECK_VOID(optixSbtRecordPackHeader((OptixProgramGroup)s_hitgroupPG,
                                            &hitgroupRecord));

  // Allocate SBT on device
  size_t sbtSize =
      sizeof(SbtRecord) * 5; // raygen, miss, hitgroup, visRaygen, dlRaygen
  CUDA_CHECK_VOID(cudaMalloc(&s_d_sbt_buffer, sbtSize));

  SbtRecord *h_sbt = new SbtRecord[5];
  h_sbt[0] = raygenRecord;
  h_sbt[1] = missRecord;
  h_sbt[2] = hitgroupRecord;

  // Pack visibility raygen header
  SbtRecord visRaygenRecord;
  OPTIX_CHECK_VOID(optixSbtRecordPackHeader((OptixProgramGroup)s_visRaygenPG,
                                            &visRaygenRecord));
  h_sbt[3] = visRaygenRecord;

  // Pack direct lighting raygen header (Phase 2)
  SbtRecord dlRaygenRecord;
  OPTIX_CHECK_VOID(optixSbtRecordPackHeader(
      (OptixProgramGroup)s_directLightingRaygenPG, &dlRaygenRecord));
  h_sbt[4] = dlRaygenRecord;

  CUDA_CHECK_VOID(
      cudaMemcpy(s_d_sbt_buffer, h_sbt, sbtSize, cudaMemcpyHostToDevice));
  delete[] h_sbt;

  Msg("  Triangles: %d\n", numTriangles);
  Msg("  GAS Size: %.2f MB\n",
      bufferSizes.outputSizeInBytes / (1024.0f * 1024.0f));
  Msg("OptiX acceleration structure built.\n");
}

//-----------------------------------------------------------------------------
// Trace a batch of rays (double-buffered ping-pong via 2 CUDA streams)
//
// Overlaps PCIe transfers with GPU compute by alternating between two
// independent buffer sets.  While stream A traces batch N, stream B can
// upload batch N+1 and the CPU can copy out batch N-1's results.
//-----------------------------------------------------------------------------
void RayTraceOptiX::TraceBatch(const RayBatch *rays, RayResult *results,
                               int num_rays) {
  if (!s_bInitialized || num_rays <= 0)
    return;

  AUTO_LOCK(s_Mutex);

  // SBT is invariant across batches — build once
  OptixShaderBindingTable sbt = {};
  sbt.raygenRecord = (CUdeviceptr)s_d_sbt_buffer;
  sbt.missRecordBase =
      (CUdeviceptr)s_d_sbt_buffer + OPTIX_SBT_RECORD_HEADER_SIZE;
  sbt.missRecordStrideInBytes = OPTIX_SBT_RECORD_HEADER_SIZE;
  sbt.missRecordCount = 1;
  sbt.hitgroupRecordBase =
      (CUdeviceptr)s_d_sbt_buffer + 2 * OPTIX_SBT_RECORD_HEADER_SIZE;
  sbt.hitgroupRecordStrideInBytes = OPTIX_SBT_RECORD_HEADER_SIZE;
  sbt.hitgroupRecordCount = 1;

  int raysProcessed = 0;
  int prevBufIdx = -1;       // Which buffer slot has in-flight GPU work
  int prevBatchSize = 0;     // Size of the previous in-flight batch
  int prevRaysProcessed = 0; // Output offset for the previous batch

  while (raysProcessed < num_rays) {
    int bufIdx = (prevBufIdx < 0) ? 0 : (1 - prevBufIdx); // Ping-pong
    int batchSize = min(num_rays - raysProcessed, s_maxBatchSize);
    cudaStream_t stream = (cudaStream_t)s_streams[bufIdx];

    // --- Copy rays into this slot's pinned host buffer ---
    memcpy(s_h_rays_pinned[bufIdx], rays + raysProcessed,
           batchSize * sizeof(RayBatch));

    // --- Async upload rays to this slot's device buffer ---
    CUDA_CHECK_VOID(cudaMemcpyAsync(s_d_rays[bufIdx], s_h_rays_pinned[bufIdx],
                                    batchSize * sizeof(RayBatch),
                                    cudaMemcpyHostToDevice, stream));

    // --- Set up and upload launch params for this slot ---
    OptixLaunchParams launchParams;
    launchParams.rays = s_d_rays[bufIdx];
    launchParams.results = s_d_results[bufIdx];
    launchParams.num_rays = batchSize;
    launchParams.traversable = s_gas_handle;
    launchParams.triangles = s_d_triangles;

    // Texture shadow data
    launchParams.d_triMaterials = s_d_triMaterials;
    launchParams.d_texShadowTris = s_d_texShadowTris;
    launchParams.d_alphaAtlas = s_d_alphaAtlas;
    launchParams.textureShadowsEnabled = s_textureShadowsEnabled ? 1 : 0;
    launchParams.backfaceWTShadowCull = s_backfaceWTShadowCull ? 1 : 0;
    launchParams.frontfaceWTShadowCull = s_frontfaceWTShadowCull ? 1 : 0;

    CUDA_CHECK_VOID(cudaMemcpyAsync(s_d_launchParams[bufIdx], &launchParams,
                                    sizeof(OptixLaunchParams),
                                    cudaMemcpyHostToDevice, stream));

    // --- Launch trace on this stream (waits for its own uploads) ---
    OPTIX_CHECK_VOID(optixLaunch((OptixPipeline)s_pipeline, stream,
                                 (CUdeviceptr)s_d_launchParams[bufIdx],
                                 sizeof(OptixLaunchParams), &sbt, batchSize, 1,
                                 1));

    // --- Async download results into this slot's pinned host buffer ---
    CUDA_CHECK_VOID(cudaMemcpyAsync(
        s_h_results_pinned[bufIdx], s_d_results[bufIdx],
        batchSize * sizeof(RayResult), cudaMemcpyDeviceToHost, stream));

    // --- While this batch traces, collect the *previous* batch's results ---
    if (prevBufIdx >= 0) {
      CUDA_CHECK_VOID(
          cudaStreamSynchronize((cudaStream_t)s_streams[prevBufIdx]));
      memcpy(results + prevRaysProcessed, s_h_results_pinned[prevBufIdx],
             prevBatchSize * sizeof(RayResult));
    }

    prevBufIdx = bufIdx;
    prevBatchSize = batchSize;
    prevRaysProcessed = raysProcessed;
    raysProcessed += batchSize;
  }

  // --- Sync and copy the final batch ---
  if (prevBufIdx >= 0) {
    CUDA_CHECK_VOID(cudaStreamSynchronize((cudaStream_t)s_streams[prevBufIdx]));
    memcpy(results + prevRaysProcessed, s_h_results_pinned[prevBufIdx],
           prevBatchSize * sizeof(RayResult));
  }
}

//-----------------------------------------------------------------------------
// Upload visibility data structures to GPU
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// Upload visibility data structures to GPU
//-----------------------------------------------------------------------------
void RayTraceOptiX::UploadVisibilityData(
    const CUtlVector<int> &clusterLeafOffsets,
    const CUtlVector<int> &clusterLeafIndices,
    const CUtlVector<GPUPatch> &patches) {
  if (!s_bInitialized)
    return;

  // Helper lambda for alloc and copy
  auto UploadBuffer = [](const void *src, int count, int elemSize) -> void * {
    void *d_ptr;
    size_t bytes = count * elemSize;
    if (cudaMalloc(&d_ptr, bytes) != cudaSuccess)
      return nullptr;
    cudaMemcpy(d_ptr, src, bytes, cudaMemcpyHostToDevice);
    return d_ptr;
  };

  s_visData.clusterLeafOffsets = (int *)UploadBuffer(
      clusterLeafOffsets.Base(), clusterLeafOffsets.Count(), sizeof(int));
  s_visData.clusterLeafIndices = (int *)UploadBuffer(
      clusterLeafIndices.Base(), clusterLeafIndices.Count(), sizeof(int));
  s_visData.patches = (GPUPatch *)UploadBuffer(patches.Base(), patches.Count(),
                                               sizeof(GPUPatch));
  s_visData.numPatches = patches.Count();

  // Check if any failed
  if (!s_visData.clusterLeafOffsets || !s_visData.patches) {
    Warning("Failed to upload visibility data to GPU!\n");
  } else {
    Msg("Uploaded Visibility Data: %d patches, %d cluster offsets\n",
        patches.Count(), clusterLeafOffsets.Count());
  }
}

//-----------------------------------------------------------------------------
// Trace visibility for a specific cluster
//-----------------------------------------------------------------------------
void RayTraceOptiX::TraceClusterVisibility(
    const CUtlVector<int> &shooterPatches,
    const CUtlVector<int> &visibleClusters,
    CUtlVector<VisiblePair> &visiblePairs) {
  if (!s_bInitialized)
    return;

  AUTO_LOCK(s_Mutex);

  // 1. Reset Atomic Counter
  int zero = 0;
  CUDA_CHECK_VOID(
      cudaMemcpy(s_d_pairCount, &zero, sizeof(int), cudaMemcpyHostToDevice));

  // 2. Upload Input Lists
  int numShooters = shooterPatches.Count();
  int numClusters = visibleClusters.Count();

  CUDA_CHECK_VOID(cudaMemcpy(s_d_shooterPatches, shooterPatches.Base(),
                             numShooters * sizeof(int),
                             cudaMemcpyHostToDevice));

  CUDA_CHECK_VOID(cudaMemcpy(s_d_visibleClusters, visibleClusters.Base(),
                             numClusters * sizeof(int),
                             cudaMemcpyHostToDevice));

  // 3. Setup Launch Params
  OptixLaunchParams params = {};
  params.traversable = s_gas_handle;
  params.triangles = s_d_triangles;

  // Visibility params
  params.shooter_patches = (int *)s_d_shooterPatches;
  params.num_shooters = numShooters;
  params.visible_clusters = (int *)s_d_visibleClusters;
  params.num_visible_clusters = numClusters;
  params.vis_scene_data = s_visData;
  params.visible_pairs = (VisiblePair *)s_d_visiblePairs;
  params.pair_count_atomic = (int *)s_d_pairCount;
  params.max_pairs = s_maxVisibilityPairs;

  // Texture shadow data
  params.d_triMaterials = s_d_triMaterials;
  params.d_texShadowTris = s_d_texShadowTris;
  params.d_alphaAtlas = s_d_alphaAtlas;
  params.textureShadowsEnabled = s_textureShadowsEnabled ? 1 : 0;
  params.backfaceWTShadowCull = s_backfaceWTShadowCull ? 1 : 0;
  params.frontfaceWTShadowCull = s_frontfaceWTShadowCull ? 1 : 0;

  // Upload to first launch params buffer slot
  CUDA_CHECK_VOID(cudaMemcpy(s_d_launchParams[0], &params,
                             sizeof(OptixLaunchParams),
                             cudaMemcpyHostToDevice));

  // 4. Build SBT for Visibility Kernel
  // We put visRaygenRecord at index 3 of the SBT buffer
  OptixShaderBindingTable sbt = {};
  sbt.raygenRecord =
      (CUdeviceptr)s_d_sbt_buffer + 3 * OPTIX_SBT_RECORD_HEADER_SIZE; // Index 3
  sbt.missRecordBase =
      (CUdeviceptr)s_d_sbt_buffer + OPTIX_SBT_RECORD_HEADER_SIZE;
  sbt.missRecordStrideInBytes = OPTIX_SBT_RECORD_HEADER_SIZE;
  sbt.missRecordCount = 1;
  sbt.hitgroupRecordBase =
      (CUdeviceptr)s_d_sbt_buffer + 2 * OPTIX_SBT_RECORD_HEADER_SIZE;
  sbt.hitgroupRecordStrideInBytes = OPTIX_SBT_RECORD_HEADER_SIZE;
  sbt.hitgroupRecordCount = 1;

  // 5. Launch on stream[0] (avoids default stream implicit sync)
  cudaStream_t visStream = (cudaStream_t)s_streams[0];
  OPTIX_CHECK_VOID(optixLaunch(
      (OptixPipeline)s_pipeline, visStream, (CUdeviceptr)s_d_launchParams[0],
      sizeof(OptixLaunchParams), &sbt, numShooters, numClusters, 1));

  // 6. Download Results — stream sync instead of full device sync
  CUDA_CHECK_VOID(cudaStreamSynchronize(visStream));

  int resultCount = 0;
  CUDA_CHECK_VOID(cudaMemcpy(&resultCount, s_d_pairCount, sizeof(int),
                             cudaMemcpyDeviceToHost));

  if (resultCount > 0) {
    if (resultCount > s_maxVisibilityPairs) {
      Warning("Visibility buffer overflow! %d > %d\n", resultCount,
              s_maxVisibilityPairs);
      resultCount = s_maxVisibilityPairs;
    }
    visiblePairs.SetCount(resultCount);
    CUDA_CHECK_VOID(cudaMemcpy(visiblePairs.Base(), s_d_visiblePairs,
                               resultCount * sizeof(VisiblePair),
                               cudaMemcpyDeviceToHost));
  } else {
    visiblePairs.RemoveAll();
  }
}

//-----------------------------------------------------------------------------
// TraceDirectLighting — Phase 2 kernel launch
// 1D launch: one thread per lightmap sample
//-----------------------------------------------------------------------------
void RayTraceOptiX::TraceDirectLighting(int numSamples) {
  if (!s_bInitialized || numSamples <= 0)
    return;

  AUTO_LOCK(s_Mutex);

  // Build launch params with all Phase 1 device pointers
  OptixLaunchParams params = {};
  params.traversable = s_gas_handle;
  params.triangles = s_d_triangles;

  // Direct lighting data
  params.d_samples = GetDeviceSamples();
  params.d_lights = GetGPULights();
  params.d_clusterLists = GetDeviceClusterLightLists();
  params.d_clusterLightIndices = GetDeviceClusterLightIndices();
  params.d_faceInfos = GetDeviceFaceInfos();
  params.d_lightOutput = GetDeviceDirectLightingOutput();
  params.num_samples = numSamples;
  params.num_lights = GetGPULightCount();

  // num_clusters: we need to pass the cluster count so the kernel can
  // bounds-check. We stored it during upload — query from the data.
  // For now, pass a safe upper bound. The kernel only accesses
  // clusterLists[sample.clusterIndex], so bounds checking uses this.
  // We'll use the count from the uploaded scene data.
  extern int GetGPUClusterCount(); // forward decl
  params.num_clusters = GetGPUClusterCount();

  // Sky light parameters
  params.d_skyDirs = s_d_skyDirs;
  params.numSkyDirs = s_numSkyDirs;
  params.sunAngularExtent = s_sunAngularExtent;
  params.numSunSamples = (s_sunAngularExtent > 0.0f) ? 30 : 0;
  params.sunShadowSamples = 16;  // Sub-luxel anti-aliasing
  params.sunShadowRadius = 2.0f; // World-space jitter radius

  // Texture shadow data
  params.d_triMaterials = s_d_triMaterials;
  params.d_texShadowTris = s_d_texShadowTris;
  params.d_alphaAtlas = s_d_alphaAtlas;
  params.textureShadowsEnabled = s_textureShadowsEnabled ? 1 : 0;
  params.backfaceWTShadowCull = s_backfaceWTShadowCull ? 1 : 0;
  params.frontfaceWTShadowCull = s_frontfaceWTShadowCull ? 1 : 0;

  // Upload params to device
  CUDA_CHECK_VOID(cudaMemcpy(s_d_launchParams[0], &params,
                             sizeof(OptixLaunchParams),
                             cudaMemcpyHostToDevice));

  // Build SBT pointing to direct lighting raygen (index 4)
  OptixShaderBindingTable sbt = {};
  sbt.raygenRecord =
      (CUdeviceptr)s_d_sbt_buffer + 4 * OPTIX_SBT_RECORD_HEADER_SIZE;
  sbt.missRecordBase =
      (CUdeviceptr)s_d_sbt_buffer + OPTIX_SBT_RECORD_HEADER_SIZE;
  sbt.missRecordStrideInBytes = OPTIX_SBT_RECORD_HEADER_SIZE;
  sbt.missRecordCount = 1;
  sbt.hitgroupRecordBase =
      (CUdeviceptr)s_d_sbt_buffer + 2 * OPTIX_SBT_RECORD_HEADER_SIZE;
  sbt.hitgroupRecordStrideInBytes = OPTIX_SBT_RECORD_HEADER_SIZE;
  sbt.hitgroupRecordCount = 1;

  // Launch: 1D, one thread per sample
  cudaStream_t stream = (cudaStream_t)s_streams[0];
  OPTIX_CHECK_VOID(optixLaunch(
      (OptixPipeline)s_pipeline, stream, (CUdeviceptr)s_d_launchParams[0],
      sizeof(OptixLaunchParams), &sbt, numSamples, 1, 1));

  // Synchronize — results needed immediately
  CUDA_CHECK_VOID(cudaStreamSynchronize(stream));
}

//-----------------------------------------------------------------------------
// UploadSkyDirections — Precompute hemisphere sample directions and upload
// Uses Halton sequence (bases 2, 3) matching CPU's DirectionalSampler_t
//-----------------------------------------------------------------------------
void RayTraceOptiX::UploadSkyDirections(float sunAngularExtent) {
  if (!s_bInitialized)
    return;

  s_sunAngularExtent = sunAngularExtent;

  // Precompute sky sample directions using Halton sequence, matching CPU
  const int NUM_SKY_DIRS = 162; // NUMVERTEXNORMALS
  float3_t skyDirs[162];

  // Generate directions using Halton(2, 3) - same as DirectionalSampler_t
  for (int j = 0; j < NUM_SKY_DIRS; j++) {
    // Halton base-2 for z value
    float zvalue = 0;
    {
      float f = 0.5f;
      int i = j + 1;
      while (i > 0) {
        zvalue += (i & 1) * f;
        i >>= 1;
        f *= 0.5f;
      }
    }
    zvalue = 2.0f * zvalue - 1.0f; // map [0,1] to [-1,1]
    float phi = acosf(zvalue);

    // Halton base-3 for rotation angle
    float vrot = 0;
    {
      float f = 1.0f / 3.0f;
      int i = j + 1;
      while (i > 0) {
        vrot += (i % 3) * f;
        i /= 3;
        f /= 3.0f;
      }
    }
    float theta = 2.0f * 3.14159265358979323846f * vrot;
    float sin_p = sinf(phi);

    skyDirs[j].x = cosf(theta) * sin_p;
    skyDirs[j].y = sinf(theta) * sin_p;
    skyDirs[j].z = zvalue;
  }

  // Upload to GPU
  FreeSkyDirections(); // Free any existing buffer
  CUDA_CHECK_VOID(cudaMalloc(&s_d_skyDirs, NUM_SKY_DIRS * sizeof(float3_t)));
  CUDA_CHECK_VOID(cudaMemcpy(s_d_skyDirs, skyDirs,
                             NUM_SKY_DIRS * sizeof(float3_t),
                             cudaMemcpyHostToDevice));
  s_numSkyDirs = NUM_SKY_DIRS;

  Msg("Uploaded %d sky sample directions (sunExtent=%.4f)\n", NUM_SKY_DIRS,
      sunAngularExtent);
}

//-----------------------------------------------------------------------------
// FreeSkyDirections — Free device memory for sky direction buffer
//-----------------------------------------------------------------------------
void RayTraceOptiX::FreeSkyDirections() {
  if (s_d_skyDirs) {
    cudaFree(s_d_skyDirs);
    s_d_skyDirs = nullptr;
  }
  s_numSkyDirs = 0;
}

// CUDA kernel launcher declarations (defined in raytrace_cuda.cu)
extern "C" void LaunchGatherLightKernel(const BounceGatherParams &params,
                                        int block_size);
extern "C" void LaunchGatherLightBumpKernel(const BounceGatherParams &params,
                                            const float3_t *patchBumpNormals,
                                            int block_size);

// -----------------------------------------------------------------------------
// InitBounceBuffers - Upload CSR transfer data and static patch data to GPU
// Called once after BuildVisMatrix completes
// -----------------------------------------------------------------------------
bool RayTraceOptiX::InitBounceBuffers(
    int numPatches, int totalTransfers, const long long *csrOffsets,
    const int *csrPatch, const float *csrWeight, const float *reflectivity,
    const float *patchOrigin, const float *patchNormal, const int *needsBumpmap,
    const int *faceNumber, const float *bumpNormals, int numBumpPatches) {
  if (!s_bInitialized)
    return false;

  FreeBounceBuffers();

  s_bounceNumPatches = numPatches;
  s_bounceTotalTransfers = totalTransfers;

  Msg("  GPU Bounce: Uploading %d patches, %d transfers (%.1f MB)...\n",
      numPatches, totalTransfers,
      (float)(totalTransfers * (sizeof(int) + sizeof(float)) +
              numPatches * (sizeof(int) + 7 * sizeof(float3_t) + sizeof(int))) /
          (1024.0f * 1024.0f));

  // CSR transfer data
  CUDA_CHECK(cudaMalloc(&s_d_csrOffsets, (numPatches + 1) * sizeof(long long)));
  CUDA_CHECK(cudaMemcpy(s_d_csrOffsets, csrOffsets,
                        (numPatches + 1) * sizeof(long long),
                        cudaMemcpyHostToDevice));

  CUDA_CHECK(cudaMalloc(&s_d_csrPatch, totalTransfers * sizeof(int)));
  CUDA_CHECK(cudaMemcpy(s_d_csrPatch, csrPatch, totalTransfers * sizeof(int),
                        cudaMemcpyHostToDevice));

  CUDA_CHECK(cudaMalloc(&s_d_csrWeight, totalTransfers * sizeof(float)));
  CUDA_CHECK(cudaMemcpy(s_d_csrWeight, csrWeight,
                        totalTransfers * sizeof(float),
                        cudaMemcpyHostToDevice));

  // Per-patch static data (as float3_t = 3 floats each)
  CUDA_CHECK(cudaMalloc(&s_d_reflectivity, numPatches * sizeof(float3_t)));
  CUDA_CHECK(cudaMemcpy(s_d_reflectivity, reflectivity,
                        numPatches * sizeof(float3_t), cudaMemcpyHostToDevice));

  CUDA_CHECK(cudaMalloc(&s_d_patchOrigin, numPatches * sizeof(float3_t)));
  CUDA_CHECK(cudaMemcpy(s_d_patchOrigin, patchOrigin,
                        numPatches * sizeof(float3_t), cudaMemcpyHostToDevice));

  CUDA_CHECK(cudaMalloc(&s_d_patchNormal, numPatches * sizeof(float3_t)));
  CUDA_CHECK(cudaMemcpy(s_d_patchNormal, patchNormal,
                        numPatches * sizeof(float3_t), cudaMemcpyHostToDevice));

  CUDA_CHECK(cudaMalloc(&s_d_needsBumpmap, numPatches * sizeof(int)));
  CUDA_CHECK(cudaMemcpy(s_d_needsBumpmap, needsBumpmap,
                        numPatches * sizeof(int), cudaMemcpyHostToDevice));

  CUDA_CHECK(cudaMalloc(&s_d_faceNumber, numPatches * sizeof(int)));
  CUDA_CHECK(cudaMemcpy(s_d_faceNumber, faceNumber, numPatches * sizeof(int),
                        cudaMemcpyHostToDevice));

  // Per-bounce dynamic buffers (emitlight, addlight)
  CUDA_CHECK(cudaMalloc(&s_d_emitlight, numPatches * sizeof(float3_t)));
  CUDA_CHECK(cudaMalloc(&s_d_addlight, numPatches * sizeof(float3_t)));
  CUDA_CHECK(cudaMalloc(&s_d_addlightBump, numPatches * 3 * sizeof(float3_t)));

  // Bump normals (4 normals per bump-mapped patch, but stored for ALL patches)
  if (bumpNormals && numBumpPatches > 0) {
    CUDA_CHECK(cudaMalloc(&s_d_bumpNormals, numPatches * 4 * sizeof(float3_t)));
    CUDA_CHECK(cudaMemcpy(s_d_bumpNormals, bumpNormals,
                          numPatches * 4 * sizeof(float3_t),
                          cudaMemcpyHostToDevice));
  }

  s_bounceInitialized = true;
  Msg("  GPU Bounce: Buffers initialized successfully.\n");
  return true;
}

// -----------------------------------------------------------------------------
// GatherLightGPU - Run one bounce iteration on GPU
// Uploads emitlight, launches kernels, downloads addlight
// Instrumented with CUDA events for sub-phase profiling
// -----------------------------------------------------------------------------
void RayTraceOptiX::GatherLightGPU(const float *emitlight, float *addlight,
                                   float *addlightBump) {
  if (!s_bounceInitialized)
    return;

  int numPatches = s_bounceNumPatches;

  // Create CUDA events for timing
  cudaEvent_t evStart, evAfterUpload, evAfterKernel, evEnd;
  cudaEventCreate(&evStart);
  cudaEventCreate(&evAfterUpload);
  cudaEventCreate(&evAfterKernel);
  cudaEventCreate(&evEnd);

  // --- Upload phase ---
  cudaEventRecord(evStart);

  cudaMemcpy(s_d_emitlight, emitlight, numPatches * sizeof(float3_t),
             cudaMemcpyHostToDevice);
  cudaMemset(s_d_addlight, 0, numPatches * sizeof(float3_t));
  cudaMemset(s_d_addlightBump, 0, numPatches * 3 * sizeof(float3_t));

  cudaEventRecord(evAfterUpload);

  // --- Kernel phase ---
  BounceGatherParams params;
  params.csrOffsets = s_d_csrOffsets;
  params.csrPatch = s_d_csrPatch;
  params.csrWeight = s_d_csrWeight;
  params.emitlight = s_d_emitlight;
  params.reflectivity = s_d_reflectivity;
  params.patchOrigin = s_d_patchOrigin;
  params.patchNormal = s_d_patchNormal;
  params.addlight = s_d_addlight;
  params.needsBumpmap = s_d_needsBumpmap;
  params.faceNumber = s_d_faceNumber;
  params.addlightBump = s_d_addlightBump;
  params.numPatches = numPatches;
  params.patchOffset = 0; // All patches in one batch, no offset needed

  const int blockSize = 256;

  LaunchGatherLightKernel(params, blockSize);

  if (s_d_bumpNormals) {
    LaunchGatherLightBumpKernel(params, s_d_bumpNormals, blockSize);
  }

  cudaEventRecord(evAfterKernel);

  // --- Download phase ---
  cudaDeviceSynchronize();

  cudaMemcpy(addlight, s_d_addlight, numPatches * sizeof(float3_t),
             cudaMemcpyDeviceToHost);

  if (addlightBump) {
    cudaMemcpy(addlightBump, s_d_addlightBump,
               numPatches * 3 * sizeof(float3_t), cudaMemcpyDeviceToHost);
  }

  cudaEventRecord(evEnd);
  cudaEventSynchronize(evEnd);

  // Accumulate timing
  float uploadMs = 0, kernelMs = 0, downloadMs = 0;
  cudaEventElapsedTime(&uploadMs, evStart, evAfterUpload);
  cudaEventElapsedTime(&kernelMs, evAfterUpload, evAfterKernel);
  cudaEventElapsedTime(&downloadMs, evAfterKernel, evEnd);

  s_bounceUploadMs += uploadMs;
  s_bounceKernelMs += kernelMs;
  s_bounceDownloadMs += downloadMs;
  s_bounceProfileCount++;

  cudaEventDestroy(evStart);
  cudaEventDestroy(evAfterUpload);
  cudaEventDestroy(evAfterKernel);
  cudaEventDestroy(evEnd);
}

// -----------------------------------------------------------------------------
// FreeBounceBuffers - Release all GPU bounce memory
// -----------------------------------------------------------------------------
void RayTraceOptiX::FreeBounceBuffers() {
  if (s_d_csrOffsets) {
    cudaFree(s_d_csrOffsets);
    s_d_csrOffsets = nullptr;
  }
  if (s_d_csrPatch) {
    cudaFree(s_d_csrPatch);
    s_d_csrPatch = nullptr;
  }
  if (s_d_csrWeight) {
    cudaFree(s_d_csrWeight);
    s_d_csrWeight = nullptr;
  }
  if (s_d_reflectivity) {
    cudaFree(s_d_reflectivity);
    s_d_reflectivity = nullptr;
  }
  if (s_d_patchOrigin) {
    cudaFree(s_d_patchOrigin);
    s_d_patchOrigin = nullptr;
  }
  if (s_d_patchNormal) {
    cudaFree(s_d_patchNormal);
    s_d_patchNormal = nullptr;
  }
  if (s_d_needsBumpmap) {
    cudaFree(s_d_needsBumpmap);
    s_d_needsBumpmap = nullptr;
  }
  if (s_d_faceNumber) {
    cudaFree(s_d_faceNumber);
    s_d_faceNumber = nullptr;
  }
  if (s_d_emitlight) {
    cudaFree(s_d_emitlight);
    s_d_emitlight = nullptr;
  }
  if (s_d_addlight) {
    cudaFree(s_d_addlight);
    s_d_addlight = nullptr;
  }
  if (s_d_addlightBump) {
    cudaFree(s_d_addlightBump);
    s_d_addlightBump = nullptr;
  }
  if (s_d_bumpNormals) {
    cudaFree(s_d_bumpNormals);
    s_d_bumpNormals = nullptr;
  }
  s_bounceNumPatches = 0;
  s_bounceTotalTransfers = 0;
  s_bounceInitialized = false;
}

// -----------------------------------------------------------------------------
// PrintBounceProfile - Print accumulated GPU bounce sub-phase timing
// -----------------------------------------------------------------------------
void RayTraceOptiX::PrintBounceProfile() {
  if (s_bounceProfileCount == 0)
    return;

  float totalMs = s_bounceUploadMs + s_bounceKernelMs + s_bounceDownloadMs;
  Msg("\nGPU Bounce Profile (%d bounces, %.1f ms total):\n",
      s_bounceProfileCount, totalMs);
  Msg("  Upload (H->D):  %6.1f ms  (%4.1f%%)\n", s_bounceUploadMs,
      totalMs > 0 ? 100.0f * s_bounceUploadMs / totalMs : 0);
  Msg("  Kernels (GPU):  %6.1f ms  (%4.1f%%)\n", s_bounceKernelMs,
      totalMs > 0 ? 100.0f * s_bounceKernelMs / totalMs : 0);
  Msg("  Download (D->H):%6.1f ms  (%4.1f%%)\n", s_bounceDownloadMs,
      totalMs > 0 ? 100.0f * s_bounceDownloadMs / totalMs : 0);
  Msg("  Avg per bounce: %6.2f ms\n", totalMs / s_bounceProfileCount);
}

// -----------------------------------------------------------------------------
// ResetBounceProfile - Reset accumulated GPU bounce profiling data
// -----------------------------------------------------------------------------
void RayTraceOptiX::ResetBounceProfile() {
  s_bounceUploadMs = 0.0f;
  s_bounceKernelMs = 0.0f;
  s_bounceDownloadMs = 0.0f;
  s_bounceProfileCount = 0;
}

// -----------------------------------------------------------------------------
// UploadTextureShadowData - Upload per-triangle material data and alpha
// texture atlas to GPU for texture shadow support in any-hit shader
// -----------------------------------------------------------------------------
void RayTraceOptiX::UploadTextureShadowData(
    const int *triangleMaterials, int numTriangles,
    const GPUTextureShadowTri *materialEntries, int numMaterialEntries,
    const unsigned char *alphaAtlas, int atlasSize) {
  if (!s_bInitialized)
    return;

  // Free any previous data
  FreeTextureShadowData();

  if (numMaterialEntries == 0 || atlasSize == 0) {
    Msg("No texture shadow data to upload (0 material entries).\n");
    s_textureShadowsEnabled = false;
    return;
  }

  // Upload per-triangle material indices
  size_t triMatSize = (size_t)numTriangles * sizeof(int);
  CUDA_CHECK_VOID(cudaMalloc((void **)&s_d_triMaterials, triMatSize));
  CUDA_CHECK_VOID(cudaMemcpy(s_d_triMaterials, triangleMaterials, triMatSize,
                             cudaMemcpyHostToDevice));

  // Upload per-material-entry UV + atlas data
  size_t matEntrySize =
      (size_t)numMaterialEntries * sizeof(GPUTextureShadowTri);
  CUDA_CHECK_VOID(cudaMalloc((void **)&s_d_texShadowTris, matEntrySize));
  CUDA_CHECK_VOID(cudaMemcpy(s_d_texShadowTris, materialEntries, matEntrySize,
                             cudaMemcpyHostToDevice));

  // Upload flattened alpha texture atlas
  CUDA_CHECK_VOID(cudaMalloc((void **)&s_d_alphaAtlas, atlasSize));
  CUDA_CHECK_VOID(cudaMemcpy(s_d_alphaAtlas, alphaAtlas, atlasSize,
                             cudaMemcpyHostToDevice));

  s_textureShadowsEnabled = true;

  Msg("Uploaded Texture Shadow Data: %d material entries, %d triangles, "
      "%.2f MB alpha atlas\n",
      numMaterialEntries, numTriangles, atlasSize / (1024.0f * 1024.0f));
}

// -----------------------------------------------------------------------------
// FreeTextureShadowData - Release GPU texture shadow buffers
// -----------------------------------------------------------------------------
void RayTraceOptiX::FreeTextureShadowData() {
  if (s_d_triMaterials) {
    cudaFree(s_d_triMaterials);
    s_d_triMaterials = nullptr;
  }
  if (s_d_texShadowTris) {
    cudaFree(s_d_texShadowTris);
    s_d_texShadowTris = nullptr;
  }
  if (s_d_alphaAtlas) {
    cudaFree(s_d_alphaAtlas);
    s_d_alphaAtlas = nullptr;
  }
  s_textureShadowsEnabled = false;
}
