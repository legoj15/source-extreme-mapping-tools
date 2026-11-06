#include "vvis_optix.h"
#include "bsplib.h"
#include "polylib.h"
#include "tier0/dbg.h"
#include "tier1/strtools.h"
#include <cuda_runtime.h>
#include <fstream>
#include <optix.h>
#include <optix_function_table_definition.h>
#include <optix_stubs.h>
#include <stdarg.h>
#include <vector>

// A failed call records its reason for CVVisOptiX::LastError() and returns
// false. Nothing here prints or exits: the caller decides, and the -cuda hint
// pass reports one warning and carries on.
static char s_lastError[512] = "";

// __FILE__ is the full build path under /FC; the reason only needs the name.
static const char *BaseName(const char *path) {
  const char *name = path;
  for (const char *c = path; *c; c++) {
    if (*c == '\\' || *c == '/')
      name = c + 1;
  }
  return name;
}

static void SetError(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  V_vsnprintf(s_lastError, sizeof(s_lastError), fmt, args);
  va_end(args);
}

#define OPTIX_CHECK(call)                                                      \
  do {                                                                         \
    OptixResult res = call;                                                    \
    if (res != OPTIX_SUCCESS) {                                                \
      SetError("OptiX error %s at %s:%d", optixGetErrorName(res),              \
               BaseName(__FILE__), __LINE__);                                  \
      return false;                                                            \
    }                                                                          \
  } while (0)

#define CUDA_CHECK(call)                                                       \
  do {                                                                         \
    cudaError_t err = call;                                                    \
    if (err != cudaSuccess) {                                                  \
      SetError("CUDA error '%s' at %s:%d", cudaGetErrorString(err),            \
               BaseName(__FILE__), __LINE__);                                  \
      return false;                                                            \
    }                                                                          \
  } while (0)

// Frees a device buffer when it goes out of scope, so an early return from a
// failed call does not leak it.
namespace {
struct DeviceBuffer {
  void *p = nullptr;
  ~DeviceBuffer() {
    if (p)
      cudaFree(p);
  }
};
} // namespace

static bool s_bInitialized = false;
static void *s_context = nullptr;
static void *s_module = nullptr;
static void *s_pipeline = nullptr;
static void *s_raygenPG = nullptr;
static void *s_missPG = nullptr;
static void *s_hitgroupPG = nullptr;
static void *s_d_gas_output_buffer = nullptr;
static unsigned long long s_gas_handle = 0;
static void *s_d_sbt_buffer = nullptr;

static std::vector<char> s_ptxCode;

static void optixLogCallback(unsigned int level, const char *tag,
                             const char *message, void *cbdata) {
  if (level <= 2) {
    Warning("[OptiX %s] %s\n", tag, message);
  }
}

static bool LoadPTX(const char *filename) {
  char fullPath[MAX_PATH];

#ifdef _WIN32
  GetModuleFileNameA(NULL, fullPath, MAX_PATH);
  char *lastSlash = strrchr(fullPath, '\\');
  if (lastSlash) {
    *(lastSlash + 1) = '\0';
    strcat(fullPath, filename);
  } else {
    V_snprintf(fullPath, sizeof(fullPath), "%s", filename);
  }
#else
  V_snprintf(fullPath, sizeof(fullPath), "%s", filename);
#endif

  std::ifstream file(fullPath, std::ios::binary | std::ios::ate);
  if (!file.is_open()) {
    // Fallback to bin\x64 logic just in case
    V_snprintf(fullPath, sizeof(fullPath), "bin\\x64\\%s", filename);
    file.open(fullPath, std::ios::binary | std::ios::ate);
  }
  if (!file.is_open()) {
    SetError("cannot open %s next to the executable or in bin\\x64", filename);
    return false;
  }

  size_t size = file.tellg();
  file.seekg(0, std::ios::beg);
  s_ptxCode.resize(size + 1);
  if (!file.read(s_ptxCode.data(), size)) {
    SetError("cannot read %s", filename);
    return false;
  }
  s_ptxCode[size] = '\0';
  Msg("Loaded PTX: %s (%zu bytes)\n", filename, size);
  return true;
}

static Vector VertCoord(dface_t const &f, int vnum) {
  int eIndex = dsurfedges[f.firstedge + vnum];
  int point = (eIndex < 0) ? dedges[-eIndex].v[1] : dedges[eIndex].v[0];
  dvertex_t *v = dvertexes + point;
  return Vector(v->point[0], v->point[1], v->point[2]);
}

// The init steps. A failure leaves whatever was created so far in the statics;
// Initialize frees it through Shutdown.
static bool InitializeImpl() {
  Msg("Initializing CVVisOptiX...\n");
  CUDA_CHECK(cudaFree(0));

  OPTIX_CHECK(optixInit());

  OptixDeviceContextOptions options = {};
  options.logCallbackFunction = optixLogCallback;
  options.logCallbackLevel = 3;

  CUcontext cuCtx = 0;
  OPTIX_CHECK(optixDeviceContextCreate(cuCtx, &options,
                                       (OptixDeviceContext *)&s_context));

  if (!LoadPTX("vvis_optix.ptx")) {
    return false;
  }

  OptixModuleCompileOptions moduleOptions = {};
  moduleOptions.maxRegisterCount = OPTIX_COMPILE_DEFAULT_MAX_REGISTER_COUNT;
  moduleOptions.optLevel = OPTIX_COMPILE_OPTIMIZATION_DEFAULT;
  moduleOptions.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_NONE;

  OptixPipelineCompileOptions pipelineOptions = {};
  // hit_t, hit_id, coverage
  pipelineOptions.usesMotionBlur = false;
  pipelineOptions.traversableGraphFlags =
      OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_SINGLE_GAS;
  pipelineOptions.numPayloadValues = 3;
  pipelineOptions.numAttributeValues = 2;
  pipelineOptions.exceptionFlags = OPTIX_EXCEPTION_FLAG_NONE;
  pipelineOptions.pipelineLaunchParamsVariableName = "params";

  char log[2048];
  size_t logSize = sizeof(log);

  OPTIX_CHECK(optixModuleCreate((OptixDeviceContext)s_context, &moduleOptions,
                                &pipelineOptions, s_ptxCode.data(),
                                s_ptxCode.size(), log, &logSize,
                                (OptixModule *)&s_module));

  OptixProgramGroupOptions pgOptions = {};
  OptixProgramGroupDesc raygenDesc = {};
  raygenDesc.kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
  raygenDesc.raygen.module = (OptixModule)s_module;
  raygenDesc.raygen.entryFunctionName = "__raygen__portal_visibility";
  logSize = sizeof(log);
  OPTIX_CHECK(optixProgramGroupCreate((OptixDeviceContext)s_context,
                                      &raygenDesc, 1, &pgOptions, log, &logSize,
                                      (OptixProgramGroup *)&s_raygenPG));

  OptixProgramGroupDesc missDesc = {};
  missDesc.kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
  missDesc.miss.module = (OptixModule)s_module;
  missDesc.miss.entryFunctionName = "__miss__visibility";
  logSize = sizeof(log);
  OPTIX_CHECK(optixProgramGroupCreate((OptixDeviceContext)s_context, &missDesc,
                                      1, &pgOptions, log, &logSize,
                                      (OptixProgramGroup *)&s_missPG));

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

  OptixProgramGroup programGroups[] = {(OptixProgramGroup)s_raygenPG,
                                       (OptixProgramGroup)s_missPG,
                                       (OptixProgramGroup)s_hitgroupPG};

  OptixPipelineLinkOptions linkOptions = {};
  linkOptions.maxTraceDepth = 1;

  logSize = sizeof(log);
  OPTIX_CHECK(optixPipelineCreate(
      (OptixDeviceContext)s_context, &pipelineOptions, &linkOptions,
      programGroups, 3, log, &logSize, (OptixPipeline *)&s_pipeline));

  struct SbtRecord {
    char header[OPTIX_SBT_RECORD_HEADER_SIZE];
  };
  size_t sbtSize = sizeof(SbtRecord) * 3;
  CUDA_CHECK(cudaMalloc(&s_d_sbt_buffer, sbtSize));

  SbtRecord h_sbt[3];
  OPTIX_CHECK(
      optixSbtRecordPackHeader((OptixProgramGroup)s_raygenPG, &h_sbt[0]));
  OPTIX_CHECK(optixSbtRecordPackHeader((OptixProgramGroup)s_missPG, &h_sbt[1]));
  OPTIX_CHECK(
      optixSbtRecordPackHeader((OptixProgramGroup)s_hitgroupPG, &h_sbt[2]));
  CUDA_CHECK(
      cudaMemcpy(s_d_sbt_buffer, h_sbt, sbtSize, cudaMemcpyHostToDevice));

  return true;
}

bool CVVisOptiX::Initialize() {
  if (s_bInitialized)
    return true;

  if (!InitializeImpl()) {
    Shutdown();
    return false;
  }
  s_bInitialized = true;
  Msg("OptiX initialized successfully.\n");
  return true;
}

// Recursive walk of the BSP tree to gather strictly world geometry brushes
static void GetWorldBrushes_r(int node, std::vector<int> &list) {
  if (node < 0) {
    int leafIndex = -1 - node;
    // Add the solids in the leaf
    for (int i = 0; i < dleafs[leafIndex].numleafbrushes; i++) {
      int brushIndex = dleafbrushes[dleafs[leafIndex].firstleafbrush + i];

      // Ensure unique brushes
      bool found = false;
      for (size_t j = 0; j < list.size(); j++) {
        if (list[j] == brushIndex) {
          found = true;
          break;
        }
      }
      if (!found) {
        list.push_back(brushIndex);
      }
    }
  } else {
    // recurse
    dnode_t *pnode = dnodes + node;
    GetWorldBrushes_r(pnode->children[0], list);
    GetWorldBrushes_r(pnode->children[1], list);
  }
}

bool CVVisOptiX::BuildScene() {
  if (!s_bInitialized) {
    SetError("OptiX is not initialized");
    return false;
  }
  Msg("Building OptiX geometry from watertight BSP brushes...\n");

  std::vector<float3> optixVertices;

  // 1. Extract structural world brushes (watertight CSG)
  // We explicitly walk dmodels[0].headnode to avoid dynamic brush entities like
  // func_door, func_breakable, and func_wall which do NOT block static VVIS.
  std::vector<int> worldBrushes;
  if (nummodels > 0) {
    GetWorldBrushes_r(dmodels[0].headnode, worldBrushes);
  }

  for (size_t i = 0; i < worldBrushes.size(); i++) {
    dbrush_t *pBrush = &dbrushes[worldBrushes[i]];

    // Only opaque brushes block visibility
    if (!(pBrush->contents & MASK_OPAQUE))
      continue;

    // func_detail brushes do NOT block visibility in VVIS
    if (pBrush->contents & CONTENTS_DETAIL)
      continue;

    // Parse planes into an infinite polyhedron, and chop it down to bounds
    for (int k = 0; k < pBrush->numsides; k++) {
      dbrushside_t *side = &dbrushsides[pBrush->firstside + k];
      dplane_t *plane = &dplanes[side->planenum];

      // Skip yielding triangles for faces that do not block visibility
      texinfo_t *tx = &texinfo[side->texinfo];
      if (tx->flags & (SURF_HINT | SURF_SKIP | SURF_TRANS | SURF_TRIGGER)) {
        continue;
      }

      winding_t *w = BaseWindingForPlane(plane->normal, plane->dist);

      for (int j = 0; j < pBrush->numsides && w; j++) {
        if (k == j)
          continue;
        dbrushside_t *pOtherSide = &dbrushsides[pBrush->firstside + j];
        if (pOtherSide->bevel)
          continue;
        dplane_t *pOtherPlane = &dplanes[pOtherSide->planenum ^ 1];
        ChopWindingInPlace(&w, pOtherPlane->normal, pOtherPlane->dist, 0);
      }

      if (w) {
        for (int j = 2; j < w->numpoints; j++) {
          optixVertices.push_back(make_float3(w->p[0].x, w->p[0].y, w->p[0].z));
          optixVertices.push_back(
              make_float3(w->p[j - 1].x, w->p[j - 1].y, w->p[j - 1].z));
          optixVertices.push_back(make_float3(w->p[j].x, w->p[j].y, w->p[j].z));
        }
        FreeWinding(w);
      }
    }
  }

  // 2. Extract SURF_SKY map boundary sealers natively from faces
  for (int i = 0; i < numfaces; i++) {
    dface_t const &f = dfaces[i];
    texinfo_t *tx = (f.texinfo >= 0) ? &(texinfo[f.texinfo]) : nullptr;
    if (tx && (tx->flags & SURF_SKY)) {
      int ntris = f.numedges - 2;
      for (int tri = 0; tri < ntris; tri++) {
        Vector v0 = VertCoord(f, 0);
        Vector v1 = VertCoord(f, tri + 1);
        Vector v2 = VertCoord(f, tri + 2);
        optixVertices.push_back(make_float3(v0.x, v0.y, v0.z));
        optixVertices.push_back(make_float3(v1.x, v1.y, v1.z));
        optixVertices.push_back(make_float3(v2.x, v2.y, v2.z));
      }
    }
  }

  Msg("Extracted %zu triangles from BSP.\n", optixVertices.size() / 3);

  if (optixVertices.empty()) {
    SetError("the BSP has no opaque world geometry to build a scene from");
    return false;
  }

  DeviceBuffer verticesBuf, indicesBuf, tempBuf;
  CUDA_CHECK(cudaMalloc(&verticesBuf.p, optixVertices.size() * sizeof(float3)));
  CUDA_CHECK(cudaMemcpy(verticesBuf.p, optixVertices.data(),
                        optixVertices.size() * sizeof(float3),
                        cudaMemcpyHostToDevice));
  CUdeviceptr d_vertices = (CUdeviceptr)verticesBuf.p;

  std::vector<uint32_t> indices(optixVertices.size());
  for (size_t i = 0; i < indices.size(); i++)
    indices[i] = (uint32_t)i;

  CUDA_CHECK(cudaMalloc(&indicesBuf.p, indices.size() * sizeof(uint32_t)));
  CUDA_CHECK(cudaMemcpy(indicesBuf.p, indices.data(),
                        indices.size() * sizeof(uint32_t),
                        cudaMemcpyHostToDevice));
  CUdeviceptr d_indices = (CUdeviceptr)indicesBuf.p;

  OptixBuildInput buildInput = {};
  buildInput.type = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;
  buildInput.triangleArray.vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3;
  buildInput.triangleArray.vertexStrideInBytes = sizeof(float3);
  buildInput.triangleArray.numVertices = (uint32_t)optixVertices.size();
  buildInput.triangleArray.vertexBuffers = &d_vertices;
  buildInput.triangleArray.indexFormat = OPTIX_INDICES_FORMAT_UNSIGNED_INT3;
  buildInput.triangleArray.indexStrideInBytes = sizeof(uint32_t) * 3;
  buildInput.triangleArray.numIndexTriplets =
      (uint32_t)(optixVertices.size() / 3);
  buildInput.triangleArray.indexBuffer = d_indices;

  uint32_t inputFlags[] = {OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT |
                           OPTIX_GEOMETRY_FLAG_DISABLE_TRIANGLE_FACE_CULLING};
  buildInput.triangleArray.flags = inputFlags;
  buildInput.triangleArray.numSbtRecords = 1;

  OptixAccelBuildOptions accelOptions = {};
  accelOptions.buildFlags =
      OPTIX_BUILD_FLAG_ALLOW_COMPACTION | OPTIX_BUILD_FLAG_PREFER_FAST_TRACE;
  accelOptions.operation = OPTIX_BUILD_OPERATION_BUILD;

  OptixAccelBufferSizes bufferSizes;
  OPTIX_CHECK(optixAccelComputeMemoryUsage((OptixDeviceContext)s_context,
                                           &accelOptions, &buildInput, 1,
                                           &bufferSizes));

  CUDA_CHECK(cudaMalloc(&tempBuf.p, bufferSizes.tempSizeInBytes));
  // The GAS output stays allocated (and is freed by Shutdown) after this
  // function returns, including when the build below fails.
  CUDA_CHECK(cudaMalloc(&s_d_gas_output_buffer, bufferSizes.outputSizeInBytes));

  OPTIX_CHECK(optixAccelBuild(
      (OptixDeviceContext)s_context, 0, &accelOptions, &buildInput, 1,
      (CUdeviceptr)tempBuf.p, bufferSizes.tempSizeInBytes,
      (CUdeviceptr)s_d_gas_output_buffer, bufferSizes.outputSizeInBytes,
      (OptixTraversableHandle *)&s_gas_handle, nullptr, 0));
  CUDA_CHECK(cudaDeviceSynchronize());

  Msg("GAS Built. Size: %.2f MB\n",
      bufferSizes.outputSizeInBytes / (1024.0f * 1024.0f));
  return true;
}

bool CVVisOptiX::TraceVisibility(int numPortals, int portalBytes,
                                 VVIS_GPUPortal *d_portals, Vector *d_windings,
                                 unsigned char *d_portalFlood,
                                 unsigned char *d_portalVis) {
  if (!s_bInitialized || !s_gas_handle) {
    SetError("OptiX has no scene to trace against");
    return false;
  }

  Msg("Launching OptiX Visibility Rays (%d x %d)...\n", numPortals, numPortals);

  VVIS_OptixLaunchParams launchParams = {};
  launchParams.num_portals = numPortals;
  launchParams.portal_bytes = portalBytes;
  launchParams.portals = d_portals;
  launchParams.winding_points = d_windings;
  launchParams.portal_flood = d_portalFlood;
  launchParams.portal_vis = d_portalVis;
  launchParams.traversable = s_gas_handle;

  DeviceBuffer paramsBuf;
  CUDA_CHECK(cudaMalloc(&paramsBuf.p, sizeof(VVIS_OptixLaunchParams)));
  CUDA_CHECK(cudaMemcpy(paramsBuf.p, &launchParams,
                        sizeof(VVIS_OptixLaunchParams),
                        cudaMemcpyHostToDevice));
  CUdeviceptr d_params = (CUdeviceptr)paramsBuf.p;

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

  OPTIX_CHECK(optixLaunch((OptixPipeline)s_pipeline, 0, d_params,
                          sizeof(VVIS_OptixLaunchParams), &sbt, numPortals,
                          numPortals, 1));
  CUDA_CHECK(cudaDeviceSynchronize());

  Msg("OptiX Visibility Rays Finished.\n");
  return true;
}

// Frees whatever Initialize and BuildScene created, however far they got, so
// it also cleans up after a failed Initialize.
void CVVisOptiX::Shutdown() {
  if (s_d_sbt_buffer)
    cudaFree(s_d_sbt_buffer);
  if (s_d_gas_output_buffer)
    cudaFree(s_d_gas_output_buffer);

  if (s_pipeline)
    optixPipelineDestroy((OptixPipeline)s_pipeline);
  if (s_raygenPG)
    optixProgramGroupDestroy((OptixProgramGroup)s_raygenPG);
  if (s_missPG)
    optixProgramGroupDestroy((OptixProgramGroup)s_missPG);
  if (s_hitgroupPG)
    optixProgramGroupDestroy((OptixProgramGroup)s_hitgroupPG);
  if (s_module)
    optixModuleDestroy((OptixModule)s_module);
  if (s_context)
    optixDeviceContextDestroy((OptixDeviceContext)s_context);

  s_d_sbt_buffer = nullptr;
  s_d_gas_output_buffer = nullptr;
  s_gas_handle = 0;
  s_pipeline = nullptr;
  s_raygenPG = nullptr;
  s_missPG = nullptr;
  s_hitgroupPG = nullptr;
  s_module = nullptr;
  s_context = nullptr;
  std::vector<char>().swap(s_ptxCode);
  s_bInitialized = false;
}

bool CVVisOptiX::IsInitialized() { return s_bInitialized; }

const char *CVVisOptiX::LastError() { return s_lastError; }
