//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: the OptiX host side: the arc tracer (also the tier B face and
//          mirror segments) and the tier B fan pipeline (bw_fan.cu). See
//          bw_optix.h.
//
//          One raygen thread per arc walks its chords; each chord is a ray
//          against the triangle mesh the arc names (BW_MESH_*). Three meshes
//          can be built: the grown obstacles of the stand and crouch postures
//          (a point ray is then an exact box trace; bw_arcs.cpp builds only the
//          crouch one) and the shrunk opaque occluders for tier B sight
//          segments (BW_MESH_VIS, built by bw_vis.cpp). Rays use
//          TERMINATE_ON_FIRST_HIT, DISABLE_ANYHIT and DISABLE_CLOSESTHIT: the
//          payload starts at "hit" and only the miss program clears it, so the
//          hit group carries no programs.
//
//=============================================================================//

#include "cmdlib.h"
#include "tier1/strtools.h"
#include <windows.h>
#include <cuda_runtime.h>
#include <optix.h>
#include <optix_function_table_definition.h>
#include <optix_stubs.h>
#include "bw_optix.h"

#define BW_CUDA( call )                                                                             \
	do                                                                                              \
	{                                                                                               \
		cudaError_t e_ = ( call );                                                                  \
		if ( e_ != cudaSuccess )                                                                    \
		{                                                                                           \
			Warning( "bspwalk: CUDA error %s at %s:%d\n", cudaGetErrorString( e_ ), __FILE__, __LINE__ ); \
			return false;                                                                           \
		}                                                                                           \
	} while ( 0 )

#define BW_OPTIX( call )                                                                            \
	do                                                                                              \
	{                                                                                               \
		OptixResult r_ = ( call );                                                                  \
		if ( r_ != OPTIX_SUCCESS )                                                                  \
		{                                                                                           \
			Warning( "bspwalk: OptiX error %s at %s:%d\n", optixGetErrorName( r_ ), __FILE__, __LINE__ ); \
			return false;                                                                           \
		}                                                                                           \
	} while ( 0 )

struct BwOptixState
{
	bool ready;
	bool cudaUp;
	char device[256];
	OptixDeviceContext context;
	OptixModule module;
	OptixProgramGroup raygen, miss, hitgroup, sweepRaygen;
	OptixPipeline pipeline;
	CUdeviceptr sbt;
	CUdeviceptr gas[BW_MESH_COUNT];
	size_t gasBytes[BW_MESH_COUNT];
	OptixTraversableHandle handle[BW_MESH_COUNT];
	CUdeviceptr dArcs, dBlocked, dParams;
	size_t arcCap;

	// the #35 sweep (__raygen__sweep): every eye sample, a candidate index buffer, the
	// clear flag, and per slot (a bw_vis thread) the confirmation samples of its face
	CUdeviceptr dSweepEyes, dSweepIdx, dSweepClear;
	size_t sweepEyeCount, sweepIdxCap;
	CUdeviceptr dSweepSamples[BW_SWEEP_SLOTS];
	size_t sweepSampleCap[BW_SWEEP_SLOTS];
	unsigned int sweepNumSamples[BW_SWEEP_SLOTS];

	// the #14 eye fan: its own module (bspwalk_fan.ptx), pipeline and two meshes
	OptixModule fanModule;
	OptixProgramGroup fanRaygen, fanMiss[2], fanHit[2];
	OptixPipeline fanPipeline;
	CUdeviceptr fanSbt, fanParams;
	bool fanReady;								// every object above made (FanPipeline finished)
	CUdeviceptr fanGas[2];						// occluders, face slabs
	OptixTraversableHandle fanHandle[2];
	CUdeviceptr dFanEyes, dFanDirs, dFanPrim, dFanN, dFanVisible, dFanT, dFanPlates;
	size_t fanTCap;
	unsigned int fanRays, fanFaces, fanEyes, fanPlates;
};
static BwOptixState s_Ox;

static void OptixLog( unsigned int level, const char *tag, const char *message, void * )
{
	if ( level <= 2 )
		Warning( "bspwalk: [OptiX %s] %s\n", tag, message );
}

// Reads <exe dir>\<name>. Never a cwd-relative fallback.
static bool LoadPtx( const char *name, std::vector<char> &ptx, char *path, int pathLen )
{
	path[0] = 0;	// the caller prints it on failure
	DWORD n = GetModuleFileNameA( NULL, path, pathLen );
	if ( n == 0 || (int)n >= pathLen )
		return false;
	char *slash = strrchr( path, '\\' );
	if ( slash )
		slash[1] = 0;
	V_strncat( path, name, pathLen );
	FILE *fp = fopen( path, "rb" );
	if ( !fp )
		return false;
	fseek( fp, 0, SEEK_END );
	long size = ftell( fp );
	if ( size < 0 )
	{
		fclose( fp );
		return false;
	}
	fseek( fp, 0, SEEK_SET );
	ptx.resize( size + 1 );
	size_t got = fread( ptx.data(), 1, size, fp );
	fclose( fp );
	ptx[got] = 0;
	ptx.resize( got );
	return got > 0;
}

static bool InitInner()
{
	int count = 0;
	if ( cudaGetDeviceCount( &count ) != cudaSuccess || count <= 0 )
	{
		cudaGetLastError();
		Warning( "bspwalk: no CUDA device\n" );
		return false;
	}
	BW_CUDA( cudaSetDevice( 0 ) );
	BW_CUDA( cudaFree( 0 ) );
	s_Ox.cudaUp = true;
	cudaDeviceProp prop;
	BW_CUDA( cudaGetDeviceProperties( &prop, 0 ) );
	Q_strncpy( s_Ox.device, prop.name, sizeof( s_Ox.device ) );

	BW_OPTIX( optixInit() );
	OptixDeviceContextOptions ctxOpt = {};
	ctxOpt.logCallbackFunction = OptixLog;
	ctxOpt.logCallbackLevel = 2;
	BW_OPTIX( optixDeviceContextCreate( 0, &ctxOpt, &s_Ox.context ) );

	std::vector<char> ptx;
	char path[MAX_PATH];
	if ( !LoadPtx( "bspwalk_arcs.ptx", ptx, path, sizeof( path ) ) )
	{
		Warning( "bspwalk: cannot read %s (built by src/utils/bspwalk/optix_compile.bat)\n", path );
		return false;
	}

	OptixModuleCompileOptions modOpt = {};
	modOpt.maxRegisterCount = OPTIX_COMPILE_DEFAULT_MAX_REGISTER_COUNT;
	modOpt.optLevel = OPTIX_COMPILE_OPTIMIZATION_DEFAULT;
	modOpt.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_NONE;

	OptixPipelineCompileOptions pipeOpt = {};
	pipeOpt.usesMotionBlur = 0;
	pipeOpt.traversableGraphFlags = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_SINGLE_GAS;
	pipeOpt.numPayloadValues = 1;
	pipeOpt.numAttributeValues = 2;
	pipeOpt.exceptionFlags = OPTIX_EXCEPTION_FLAG_NONE;
	pipeOpt.pipelineLaunchParamsVariableName = "params";
	pipeOpt.pipelineLaunchParamsSizeInBytes = sizeof( BwArcLaunchParams );
	pipeOpt.usesPrimitiveTypeFlags = (unsigned int)OPTIX_PRIMITIVE_TYPE_FLAGS_TRIANGLE;

	char log[2048];
	size_t logSize = sizeof( log );
	BW_OPTIX( optixModuleCreate( s_Ox.context, &modOpt, &pipeOpt, ptx.data(), ptx.size(), log, &logSize, &s_Ox.module ) );

	OptixProgramGroupOptions pgOpt = {};
	OptixProgramGroupDesc desc[4] = {};
	desc[0].kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
	desc[0].raygen.module = s_Ox.module;
	desc[0].raygen.entryFunctionName = "__raygen__arcs";
	desc[1].kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
	desc[1].miss.module = s_Ox.module;
	desc[1].miss.entryFunctionName = "__miss__arcs";
	desc[2].kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;	// no programs: built-in triangles, closest hit disabled per ray
	desc[3].kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
	desc[3].raygen.module = s_Ox.module;
	desc[3].raygen.entryFunctionName = "__raygen__sweep";
	OptixProgramGroup *pgs[4] = { &s_Ox.raygen, &s_Ox.miss, &s_Ox.hitgroup, &s_Ox.sweepRaygen };
	for ( int i = 0; i < 4; i++ )
	{
		logSize = sizeof( log );
		BW_OPTIX( optixProgramGroupCreate( s_Ox.context, &desc[i], 1, &pgOpt, log, &logSize, pgs[i] ) );
	}

	OptixPipelineLinkOptions linkOpt = {};
	linkOpt.maxTraceDepth = 1;
	// the SBT holds the arc raygen, miss and hitgroup records, then the sweep's raygen record
	OptixProgramGroup groups[4] = { s_Ox.raygen, s_Ox.miss, s_Ox.hitgroup, s_Ox.sweepRaygen };
	logSize = sizeof( log );
	BW_OPTIX( optixPipelineCreate( s_Ox.context, &pipeOpt, &linkOpt, groups, 4, log, &logSize, &s_Ox.pipeline ) );

	struct SbtRecord
	{
		__declspec( align( OPTIX_SBT_RECORD_ALIGNMENT ) ) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
	};
	SbtRecord rec[4];
	for ( int i = 0; i < 4; i++ )
		BW_OPTIX( optixSbtRecordPackHeader( groups[i], &rec[i] ) );
	BW_CUDA( cudaMalloc( (void **)&s_Ox.sbt, sizeof( rec ) ) );
	BW_CUDA( cudaMemcpy( (void *)s_Ox.sbt, rec, sizeof( rec ), cudaMemcpyHostToDevice ) );
	BW_CUDA( cudaMalloc( (void **)&s_Ox.dParams, sizeof( BwArcLaunchParams ) ) );
	return true;
}

bool BwOptix_Init()
{
	if ( s_Ox.ready )
		return true;
	memset( &s_Ox, 0, sizeof( s_Ox ) );
	if ( !InitInner() )
	{
		BwOptix_Shutdown();
		return false;
	}
	s_Ox.ready = true;
	return true;
}

const char *BwOptix_DeviceName()
{
	return s_Ox.device;
}

size_t BwOptix_MeshBytes( int mesh )
{
	return ( mesh >= 0 && mesh < BW_MESH_COUNT ) ? s_Ox.gasBytes[mesh] : 0;
}

// Builds a compacted triangle GAS into gas / handle / bytes. allowAnyHit: the fan's
// face slabs run an any-hit program; every other mesh disables it.
static bool BuildGasInner( const std::vector<float> &verts, const std::vector<unsigned int> &tris, bool allowAnyHit,
						   CUdeviceptr &gas, OptixTraversableHandle &gasHandle, size_t &gasBytes, CUdeviceptr &dVerts,
						   CUdeviceptr &dTris, CUdeviceptr &dTemp, CUdeviceptr &dOut, CUdeviceptr &dSize )
{
	BW_CUDA( cudaMalloc( (void **)&dVerts, verts.size() * sizeof( float ) ) );
	BW_CUDA( cudaMemcpy( (void *)dVerts, verts.data(), verts.size() * sizeof( float ), cudaMemcpyHostToDevice ) );
	BW_CUDA( cudaMalloc( (void **)&dTris, tris.size() * sizeof( unsigned int ) ) );
	BW_CUDA( cudaMemcpy( (void *)dTris, tris.data(), tris.size() * sizeof( unsigned int ), cudaMemcpyHostToDevice ) );

	OptixBuildInput input = {};
	input.type = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;
	input.triangleArray.vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3;
	input.triangleArray.vertexStrideInBytes = 3 * sizeof( float );
	input.triangleArray.numVertices = (unsigned int)( verts.size() / 3 );
	input.triangleArray.vertexBuffers = &dVerts;
	input.triangleArray.indexFormat = OPTIX_INDICES_FORMAT_UNSIGNED_INT3;
	input.triangleArray.indexStrideInBytes = 3 * sizeof( unsigned int );
	input.triangleArray.numIndexTriplets = (unsigned int)( tris.size() / 3 );
	input.triangleArray.indexBuffer = dTris;
	unsigned int flags[1] = { ( allowAnyHit ? 0u : (unsigned int)OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT ) |
							  OPTIX_GEOMETRY_FLAG_DISABLE_TRIANGLE_FACE_CULLING };
	input.triangleArray.flags = flags;
	input.triangleArray.numSbtRecords = 1;

	OptixAccelBuildOptions accel = {};
	accel.buildFlags = OPTIX_BUILD_FLAG_ALLOW_COMPACTION | OPTIX_BUILD_FLAG_PREFER_FAST_TRACE;
	accel.operation = OPTIX_BUILD_OPERATION_BUILD;
	OptixAccelBufferSizes sizes;
	BW_OPTIX( optixAccelComputeMemoryUsage( s_Ox.context, &accel, &input, 1, &sizes ) );
	BW_CUDA( cudaMalloc( (void **)&dTemp, sizes.tempSizeInBytes ) );
	BW_CUDA( cudaMalloc( (void **)&dOut, sizes.outputSizeInBytes ) );
	BW_CUDA( cudaMalloc( (void **)&dSize, sizeof( unsigned long long ) ) );
	OptixAccelEmitDesc emit = {};
	emit.result = dSize;
	emit.type = OPTIX_PROPERTY_TYPE_COMPACTED_SIZE;
	OptixTraversableHandle handle = 0;
	BW_OPTIX( optixAccelBuild( s_Ox.context, 0, &accel, &input, 1, dTemp, sizes.tempSizeInBytes, dOut, sizes.outputSizeInBytes,
							   &handle, &emit, 1 ) );
	BW_CUDA( cudaDeviceSynchronize() );

	unsigned long long compacted = 0;
	BW_CUDA( cudaMemcpy( &compacted, (void *)dSize, sizeof( compacted ), cudaMemcpyDeviceToHost ) );
	if ( compacted > 0 && compacted < sizes.outputSizeInBytes )
	{
		CUdeviceptr dCompact = 0;
		BW_CUDA( cudaMalloc( (void **)&dCompact, compacted ) );
		OptixTraversableHandle h2 = 0;
		OptixResult r = optixAccelCompact( s_Ox.context, 0, handle, dCompact, compacted, &h2 );
		cudaError_t e = cudaDeviceSynchronize();
		if ( r != OPTIX_SUCCESS || e != cudaSuccess )
		{
			cudaFree( (void *)dCompact );
			Warning( "bspwalk: GAS compaction failed (%s / %s)\n", optixGetErrorName( r ), cudaGetErrorString( e ) );
			return false;
		}
		cudaFree( (void *)dOut );
		dOut = dCompact;
		handle = h2;
		gasBytes = (size_t)compacted;
	}
	else
		gasBytes = sizes.outputSizeInBytes;
	gas = dOut;
	gasHandle = handle;
	dOut = 0;
	return true;
}

static bool BuildGas( const std::vector<float> &verts, const std::vector<unsigned int> &tris, bool allowAnyHit,
					  CUdeviceptr &gas, OptixTraversableHandle &handle, size_t &bytes )
{
	CUdeviceptr dVerts = 0, dTris = 0, dTemp = 0, dOut = 0, dSize = 0;
	bool ok = BuildGasInner( verts, tris, allowAnyHit, gas, handle, bytes, dVerts, dTris, dTemp, dOut, dSize );
	cudaFree( (void *)dVerts );
	cudaFree( (void *)dTris );
	cudaFree( (void *)dTemp );
	cudaFree( (void *)dSize );
	if ( dOut )
		cudaFree( (void *)dOut );
	return ok;
}

bool BwOptix_BuildMesh( int mesh, const std::vector<float> &verts, const std::vector<unsigned int> &tris )
{
	if ( !s_Ox.ready || mesh < 0 || mesh >= BW_MESH_COUNT )
		return false;
	if ( s_Ox.gas[mesh] )
	{
		cudaFree( (void *)s_Ox.gas[mesh] );
		s_Ox.gas[mesh] = 0;
		s_Ox.handle[mesh] = 0;
		s_Ox.gasBytes[mesh] = 0;
	}
	if ( tris.empty() )
		return true;	// handle 0: optixTrace on a null handle misses everything
	return BuildGas( verts, tris, false, s_Ox.gas[mesh], s_Ox.handle[mesh], s_Ox.gasBytes[mesh] );
}

// Device buffers for 'cap' arcs, replacing the old ones. False (nothing left allocated) when
// either allocation fails.
static bool AllocArcBuffers( size_t cap )
{
	if ( s_Ox.dArcs )
		cudaFree( (void *)s_Ox.dArcs );
	if ( s_Ox.dBlocked )
		cudaFree( (void *)s_Ox.dBlocked );
	s_Ox.dArcs = s_Ox.dBlocked = 0;
	s_Ox.arcCap = 0;
	if ( cudaMalloc( (void **)&s_Ox.dArcs, cap * sizeof( BwArc ) ) != cudaSuccess ||
		 cudaMalloc( (void **)&s_Ox.dBlocked, cap ) != cudaSuccess )
	{
		cudaGetLastError();
		if ( s_Ox.dArcs )
			cudaFree( (void *)s_Ox.dArcs );
		s_Ox.dArcs = s_Ox.dBlocked = 0;
		return false;
	}
	s_Ox.arcCap = cap;
	return true;
}

bool BwOptix_TraceArcs( const BwArc *arcs, unsigned int n, float gravity, unsigned char *blocked )
{
	if ( !s_Ox.ready )
		return false;
	if ( !n )
		return true;
	if ( n > s_Ox.arcCap )
	{
		// Grow geometrically: batches that creep up in size (the face pass's tile levels) would
		// otherwise free and allocate again, each a device-wide sync, on nearly every launch. If
		// the doubled size does not fit, take exactly what this launch needs.
		const size_t MIN_CAP = 1 << 16;
		size_t cap = MAX( MAX( (size_t)n, s_Ox.arcCap * 2 ), MIN_CAP );
		if ( !AllocArcBuffers( cap ) && !AllocArcBuffers( n ) )
		{
			Warning( "bspwalk: cannot allocate device buffers for %u arcs\n", n );
			return false;
		}
	}
	BW_CUDA( cudaMemcpy( (void *)s_Ox.dArcs, arcs, (size_t)n * sizeof( BwArc ), cudaMemcpyHostToDevice ) );

	BwArcLaunchParams p = {};
	p.arcs = (const BwArc *)s_Ox.dArcs;
	p.blocked = (unsigned char *)s_Ox.dBlocked;
	p.numArcs = n;
	p.gravity = gravity;
	for ( int m = 0; m < BW_MESH_COUNT; m++ )
		p.gas[m] = s_Ox.handle[m];
	BW_CUDA( cudaMemcpy( (void *)s_Ox.dParams, &p, sizeof( p ), cudaMemcpyHostToDevice ) );

	const size_t rec = OPTIX_SBT_RECORD_HEADER_SIZE;
	OptixShaderBindingTable sbt = {};
	sbt.raygenRecord = s_Ox.sbt;
	sbt.missRecordBase = s_Ox.sbt + rec;
	sbt.missRecordStrideInBytes = (unsigned int)rec;
	sbt.missRecordCount = 1;
	sbt.hitgroupRecordBase = s_Ox.sbt + 2 * rec;
	sbt.hitgroupRecordStrideInBytes = (unsigned int)rec;
	sbt.hitgroupRecordCount = 1;
	BW_OPTIX( optixLaunch( s_Ox.pipeline, 0, s_Ox.dParams, sizeof( p ), &sbt, n, 1, 1 ) );
	BW_CUDA( cudaDeviceSynchronize() );
	BW_CUDA( cudaMemcpy( blocked, (void *)s_Ox.dBlocked, n, cudaMemcpyDeviceToHost ) );
	return true;
}

//-----------------------------------------------------------------------------
// #35 sweep
//-----------------------------------------------------------------------------
void BwOptix_SweepFree()
{
	if ( s_Ox.cudaUp )
	{
		if ( s_Ox.dSweepEyes )
			cudaFree( (void *)s_Ox.dSweepEyes );
		if ( s_Ox.dSweepIdx )
			cudaFree( (void *)s_Ox.dSweepIdx );
		if ( s_Ox.dSweepClear )
			cudaFree( (void *)s_Ox.dSweepClear );
		for ( int k = 0; k < BW_SWEEP_SLOTS; k++ )
		{
			if ( s_Ox.dSweepSamples[k] )
				cudaFree( (void *)s_Ox.dSweepSamples[k] );
		}
	}
	s_Ox.dSweepEyes = s_Ox.dSweepIdx = s_Ox.dSweepClear = 0;
	s_Ox.sweepEyeCount = s_Ox.sweepIdxCap = 0;
	memset( s_Ox.dSweepSamples, 0, sizeof( s_Ox.dSweepSamples ) );
	memset( s_Ox.sweepSampleCap, 0, sizeof( s_Ox.sweepSampleCap ) );
	memset( s_Ox.sweepNumSamples, 0, sizeof( s_Ox.sweepNumSamples ) );
}

// Every eye sample's xyz, uploaded in slices: fill(ctx, first, count, out)
// writes the xyz of samples [first, first + count) to out.
static bool SweepEyesInner( size_t n, void ( *fill )( void *, size_t, size_t, float * ), void *ctx )
{
	// The launch names eyes by 32-bit index (BwArcLaunchParams::sweepIdx), so
	// more samples than that cannot be addressed; the caller falls back.
	if ( n > 0xFFFFFFFFull )
		return false;
	BW_CUDA( cudaMalloc( (void **)&s_Ox.dSweepEyes, MAX( n, (size_t)1 ) * 3 * sizeof( float ) ) );
	BW_CUDA( cudaMalloc( (void **)&s_Ox.dSweepClear, sizeof( unsigned int ) ) );
	const size_t SLICE = (size_t)1 << 22;
	std::vector<float> host( MIN( n, SLICE ) * 3 );
	for ( size_t first = 0; first < n; first += SLICE )
	{
		size_t count = MIN( SLICE, n - first );
		fill( ctx, first, count, host.data() );
		BW_CUDA( cudaMemcpy( (void *)( s_Ox.dSweepEyes + first * 3 * sizeof( float ) ), host.data(),
							 count * 3 * sizeof( float ), cudaMemcpyHostToDevice ) );
	}
	s_Ox.sweepEyeCount = n;
	return true;
}

bool BwOptix_SweepEyes( size_t n, void ( *fill )( void *, size_t, size_t, float * ), void *ctx )
{
	BwOptix_SweepFree();
	if ( !s_Ox.ready || !SweepEyesInner( n, fill, ctx ) )
	{
		cudaGetLastError();
		BwOptix_SweepFree();
		return false;
	}
	return true;
}

bool BwOptix_SweepSamples( int slot, const double *samples, unsigned int n )
{
	if ( !s_Ox.dSweepEyes || slot < 0 || slot >= BW_SWEEP_SLOTS )
		return false;
	if ( n > s_Ox.sweepSampleCap[slot] )
	{
		if ( s_Ox.dSweepSamples[slot] )
			cudaFree( (void *)s_Ox.dSweepSamples[slot] );
		s_Ox.dSweepSamples[slot] = 0;
		s_Ox.sweepSampleCap[slot] = 0;
		size_t cap = MAX( (size_t)n, (size_t)4096 );
		BW_CUDA( cudaMalloc( (void **)&s_Ox.dSweepSamples[slot], cap * 6 * sizeof( double ) ) );
		s_Ox.sweepSampleCap[slot] = cap;
	}
	if ( n )
		BW_CUDA( cudaMemcpy( (void *)s_Ox.dSweepSamples[slot], samples, (size_t)n * 6 * sizeof( double ),
							 cudaMemcpyHostToDevice ) );
	s_Ox.sweepNumSamples[slot] = n;
	return true;
}

bool BwOptix_SweepTrace( int slot, const unsigned int *idx, unsigned int n, unsigned int rays, bool twoSided,
						 unsigned long long key, unsigned long long base, bool &clear )
{
	clear = false;
	if ( !s_Ox.dSweepEyes || slot < 0 || slot >= BW_SWEEP_SLOTS || !s_Ox.sweepNumSamples[slot] )
		return false;
	if ( !n || !rays )
		return true;
	if ( n > s_Ox.sweepIdxCap )
	{
		if ( s_Ox.dSweepIdx )
			cudaFree( (void *)s_Ox.dSweepIdx );
		s_Ox.dSweepIdx = 0;
		s_Ox.sweepIdxCap = 0;
		BW_CUDA( cudaMalloc( (void **)&s_Ox.dSweepIdx, (size_t)n * sizeof( unsigned int ) ) );
		s_Ox.sweepIdxCap = n;
	}
	BW_CUDA( cudaMemcpy( (void *)s_Ox.dSweepIdx, idx, (size_t)n * sizeof( unsigned int ), cudaMemcpyHostToDevice ) );
	BW_CUDA( cudaMemset( (void *)s_Ox.dSweepClear, 0, sizeof( unsigned int ) ) );

	BwArcLaunchParams p = {};
	for ( int m = 0; m < BW_MESH_COUNT; m++ )
		p.gas[m] = s_Ox.handle[m];
	p.eyes = (const float *)s_Ox.dSweepEyes;
	p.sweepIdx = (const unsigned int *)s_Ox.dSweepIdx;
	p.sweepSamples = (const double *)s_Ox.dSweepSamples[slot];
	p.sweepClear = (unsigned int *)s_Ox.dSweepClear;
	p.sweepEyes = n;
	p.sweepRays = rays;
	p.sweepNumSamples = s_Ox.sweepNumSamples[slot];
	p.sweepTwoSided = twoSided ? 1 : 0;
	p.sweepKey = key;
	p.sweepBase = base;
	BW_CUDA( cudaMemcpy( (void *)s_Ox.dParams, &p, sizeof( p ), cudaMemcpyHostToDevice ) );

	const size_t rec = OPTIX_SBT_RECORD_HEADER_SIZE;
	OptixShaderBindingTable sbt = {};
	sbt.raygenRecord = s_Ox.sbt + 3 * rec;
	sbt.missRecordBase = s_Ox.sbt + rec;
	sbt.missRecordStrideInBytes = (unsigned int)rec;
	sbt.missRecordCount = 1;
	sbt.hitgroupRecordBase = s_Ox.sbt + 2 * rec;
	sbt.hitgroupRecordStrideInBytes = (unsigned int)rec;
	sbt.hitgroupRecordCount = 1;
	BW_OPTIX( optixLaunch( s_Ox.pipeline, 0, s_Ox.dParams, sizeof( p ), &sbt, rays, n, 1 ) );
	BW_CUDA( cudaDeviceSynchronize() );
	unsigned int c = 0;
	BW_CUDA( cudaMemcpy( &c, (void *)s_Ox.dSweepClear, sizeof( c ), cudaMemcpyDeviceToHost ) );
	clear = c != 0;
	return true;
}

//-----------------------------------------------------------------------------
// #14 eye fan
//-----------------------------------------------------------------------------
// Destroys whatever FanPipeline made, also after it failed part way.
static void FanPipelineFree()
{
	s_Ox.fanReady = false;
	if ( s_Ox.cudaUp )
	{
		if ( s_Ox.fanSbt )
			cudaFree( (void *)s_Ox.fanSbt );
		if ( s_Ox.fanParams )
			cudaFree( (void *)s_Ox.fanParams );
	}
	s_Ox.fanSbt = s_Ox.fanParams = 0;
	if ( s_Ox.fanPipeline )
		optixPipelineDestroy( s_Ox.fanPipeline );
	s_Ox.fanPipeline = NULL;
	if ( s_Ox.fanRaygen )
		optixProgramGroupDestroy( s_Ox.fanRaygen );
	s_Ox.fanRaygen = NULL;
	for ( int i = 0; i < 2; i++ )
	{
		if ( s_Ox.fanMiss[i] )
			optixProgramGroupDestroy( s_Ox.fanMiss[i] );
		if ( s_Ox.fanHit[i] )
			optixProgramGroupDestroy( s_Ox.fanHit[i] );
		s_Ox.fanMiss[i] = s_Ox.fanHit[i] = NULL;
	}
	if ( s_Ox.fanModule )
		optixModuleDestroy( s_Ox.fanModule );
	s_Ox.fanModule = NULL;
}

static bool FanPipeline()
{
	if ( s_Ox.fanReady )
		return true;
	// a setup that failed part way left some objects: start over
	FanPipelineFree();
	std::vector<char> ptx;
	char path[MAX_PATH];
	if ( !LoadPtx( "bspwalk_fan.ptx", ptx, path, sizeof( path ) ) )
	{
		Warning( "bspwalk: cannot read %s (built by src/utils/bspwalk/optix_compile.bat)\n", path );
		return false;
	}
	OptixModuleCompileOptions modOpt = {};
	modOpt.maxRegisterCount = OPTIX_COMPILE_DEFAULT_MAX_REGISTER_COUNT;
	modOpt.optLevel = OPTIX_COMPILE_OPTIMIZATION_DEFAULT;
	modOpt.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_NONE;

	OptixPipelineCompileOptions pipeOpt = {};
	pipeOpt.usesMotionBlur = 0;
	pipeOpt.traversableGraphFlags = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_SINGLE_GAS;
	pipeOpt.numPayloadValues = 1;
	pipeOpt.numAttributeValues = 2;
	pipeOpt.exceptionFlags = OPTIX_EXCEPTION_FLAG_NONE;
	pipeOpt.pipelineLaunchParamsVariableName = "fanParams";
	pipeOpt.pipelineLaunchParamsSizeInBytes = sizeof( BwFanLaunchParams );
	pipeOpt.usesPrimitiveTypeFlags = (unsigned int)OPTIX_PRIMITIVE_TYPE_FLAGS_TRIANGLE;

	char log[2048];
	size_t logSize = sizeof( log );
	BW_OPTIX( optixModuleCreate( s_Ox.context, &modOpt, &pipeOpt, ptx.data(), ptx.size(), log, &logSize, &s_Ox.fanModule ) );

	// SBT order: raygen, miss occ, miss face, hit occ (closest hit), hit face (any hit)
	OptixProgramGroupOptions pgOpt = {};
	OptixProgramGroupDesc desc[5] = {};
	desc[0].kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
	desc[0].raygen.module = s_Ox.fanModule;
	desc[0].raygen.entryFunctionName = "__raygen__fan";
	desc[1].kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
	desc[1].miss.module = s_Ox.fanModule;
	desc[1].miss.entryFunctionName = "__miss__occ";
	desc[2].kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
	desc[2].miss.module = s_Ox.fanModule;
	desc[2].miss.entryFunctionName = "__miss__face";
	desc[3].kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
	desc[3].hitgroup.moduleCH = s_Ox.fanModule;
	desc[3].hitgroup.entryFunctionNameCH = "__closesthit__occ";
	desc[4].kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
	desc[4].hitgroup.moduleAH = s_Ox.fanModule;
	desc[4].hitgroup.entryFunctionNameAH = "__anyhit__face";
	OptixProgramGroup *pgs[5] = { &s_Ox.fanRaygen, &s_Ox.fanMiss[0], &s_Ox.fanMiss[1], &s_Ox.fanHit[0], &s_Ox.fanHit[1] };
	for ( int i = 0; i < 5; i++ )
	{
		logSize = sizeof( log );
		BW_OPTIX( optixProgramGroupCreate( s_Ox.context, &desc[i], 1, &pgOpt, log, &logSize, pgs[i] ) );
	}
	OptixPipelineLinkOptions linkOpt = {};
	linkOpt.maxTraceDepth = 1;
	OptixProgramGroup groups[5] = { s_Ox.fanRaygen, s_Ox.fanMiss[0], s_Ox.fanMiss[1], s_Ox.fanHit[0], s_Ox.fanHit[1] };
	logSize = sizeof( log );
	BW_OPTIX( optixPipelineCreate( s_Ox.context, &pipeOpt, &linkOpt, groups, 5, log, &logSize, &s_Ox.fanPipeline ) );

	struct SbtRecord
	{
		__declspec( align( OPTIX_SBT_RECORD_ALIGNMENT ) ) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
	};
	SbtRecord rec[5];
	for ( int i = 0; i < 5; i++ )
		BW_OPTIX( optixSbtRecordPackHeader( groups[i], &rec[i] ) );
	BW_CUDA( cudaMalloc( (void **)&s_Ox.fanSbt, sizeof( rec ) ) );
	BW_CUDA( cudaMemcpy( (void *)s_Ox.fanSbt, rec, sizeof( rec ), cudaMemcpyHostToDevice ) );
	BW_CUDA( cudaMalloc( (void **)&s_Ox.fanParams, sizeof( BwFanLaunchParams ) ) );
	s_Ox.fanReady = true;
	return true;
}

template <typename T> static bool Upload( CUdeviceptr &d, const std::vector<T> &v )
{
	BW_CUDA( cudaMalloc( (void **)&d, MAX( (size_t)1, v.size() * sizeof( T ) ) ) );
	if ( !v.empty() )
		BW_CUDA( cudaMemcpy( (void *)d, v.data(), v.size() * sizeof( T ), cudaMemcpyHostToDevice ) );
	return true;
}

static bool FanSetupInner( const std::vector<float> &occVerts, const std::vector<unsigned int> &occTris,
						   const std::vector<float> &faceVerts, const std::vector<unsigned int> &faceTris,
						   const std::vector<unsigned int> &primFace, const std::vector<float> &faceN,
						   const std::vector<float> &plates, unsigned int numPlates,
						   const std::vector<float> &dirs, const std::vector<float> &eyes,
						   const std::vector<unsigned char> &visible )
{
	if ( !FanPipeline() )
		return false;
	size_t bytes = 0;
	if ( !occTris.empty() && !BuildGas( occVerts, occTris, false, s_Ox.fanGas[0], s_Ox.fanHandle[0], bytes ) )
		return false;
	if ( !faceTris.empty() && !BuildGas( faceVerts, faceTris, true, s_Ox.fanGas[1], s_Ox.fanHandle[1], bytes ) )
		return false;
	if ( !Upload( s_Ox.dFanEyes, eyes ) || !Upload( s_Ox.dFanDirs, dirs ) || !Upload( s_Ox.dFanPrim, primFace ) ||
		 !Upload( s_Ox.dFanN, faceN ) || !Upload( s_Ox.dFanVisible, visible ) || !Upload( s_Ox.dFanPlates, plates ) )
		return false;
	s_Ox.fanPlates = numPlates;
	s_Ox.fanRays = (unsigned int)( dirs.size() / 3 );
	s_Ox.fanEyes = (unsigned int)( eyes.size() / 3 );
	s_Ox.fanFaces = (unsigned int)visible.size();
	return true;
}

bool BwOptix_FanSetup( const std::vector<float> &occVerts, const std::vector<unsigned int> &occTris,
					   const std::vector<float> &faceVerts, const std::vector<unsigned int> &faceTris,
					   const std::vector<unsigned int> &primFace, const std::vector<float> &faceN,
					   const std::vector<float> &plates, unsigned int numPlates,
					   const std::vector<float> &dirs, const std::vector<float> &eyes, const std::vector<unsigned char> &visible )
{
	if ( !s_Ox.ready )
		return false;
	BwOptix_FanFree();
	if ( FanSetupInner( occVerts, occTris, faceVerts, faceTris, primFace, faceN, plates, numPlates, dirs, eyes, visible ) )
		return true;
	BwOptix_FanFree();
	return false;
}

bool BwOptix_FanTrace( unsigned int first, unsigned int count, float maxT, float past, float facingTol, float pad,
					   float *tOut )
{
	if ( !s_Ox.ready || !s_Ox.fanReady || first + count > s_Ox.fanEyes )
		return false;
	if ( !count || !s_Ox.fanRays )
		return true;
	size_t nT = (size_t)count * s_Ox.fanRays;
	if ( tOut && nT > s_Ox.fanTCap )
	{
		if ( s_Ox.dFanT )
			cudaFree( (void *)s_Ox.dFanT );
		s_Ox.dFanT = 0;
		s_Ox.fanTCap = 0;
		BW_CUDA( cudaMalloc( (void **)&s_Ox.dFanT, nT * sizeof( float ) ) );
		s_Ox.fanTCap = nT;
	}
	BwFanLaunchParams p = {};
	p.eyes = (const float *)s_Ox.dFanEyes + (size_t)first * 3;
	p.dirs = (const float *)s_Ox.dFanDirs;
	p.primFace = (const unsigned int *)s_Ox.dFanPrim;
	p.faceN = (const float *)s_Ox.dFanN;
	p.visible = (unsigned char *)s_Ox.dFanVisible;
	p.tOut = tOut ? (float *)s_Ox.dFanT : NULL;
	p.numRays = s_Ox.fanRays;
	p.maxT = maxT;
	p.past = past;
	p.facingTol = facingTol;
	p.plates = (const float *)s_Ox.dFanPlates;
	p.numPlates = s_Ox.fanPlates;
	p.pad = pad;
	p.occGas = s_Ox.fanHandle[0];
	p.faceGas = s_Ox.fanHandle[1];
	BW_CUDA( cudaMemcpy( (void *)s_Ox.fanParams, &p, sizeof( p ), cudaMemcpyHostToDevice ) );

	const size_t rec = OPTIX_SBT_RECORD_HEADER_SIZE;
	OptixShaderBindingTable sbt = {};
	sbt.raygenRecord = s_Ox.fanSbt;
	sbt.missRecordBase = s_Ox.fanSbt + rec;
	sbt.missRecordStrideInBytes = (unsigned int)rec;
	sbt.missRecordCount = 2;
	sbt.hitgroupRecordBase = s_Ox.fanSbt + 3 * rec;
	sbt.hitgroupRecordStrideInBytes = (unsigned int)rec;
	sbt.hitgroupRecordCount = 2;
	BW_OPTIX( optixLaunch( s_Ox.fanPipeline, 0, s_Ox.fanParams, sizeof( p ), &sbt, s_Ox.fanRays, count, 1 ) );
	BW_CUDA( cudaDeviceSynchronize() );
	if ( tOut )
		BW_CUDA( cudaMemcpy( tOut, (void *)s_Ox.dFanT, nT * sizeof( float ), cudaMemcpyDeviceToHost ) );
	return true;
}

bool BwOptix_FanVisible( unsigned char *visible )
{
	if ( !s_Ox.ready || !s_Ox.dFanVisible )
		return false;
	BW_CUDA( cudaMemcpy( visible, (void *)s_Ox.dFanVisible, s_Ox.fanFaces, cudaMemcpyDeviceToHost ) );
	return true;
}

void BwOptix_FanFree()
{
	if ( !s_Ox.cudaUp )
		return;
	CUdeviceptr *bufs[] = { &s_Ox.fanGas[0], &s_Ox.fanGas[1], &s_Ox.dFanEyes, &s_Ox.dFanDirs, &s_Ox.dFanPrim,
							&s_Ox.dFanN, &s_Ox.dFanVisible, &s_Ox.dFanT, &s_Ox.dFanPlates };
	for ( size_t i = 0; i < ARRAYSIZE( bufs ); i++ )
	{
		if ( *bufs[i] )
			cudaFree( (void *)*bufs[i] );
		*bufs[i] = 0;
	}
	s_Ox.fanHandle[0] = s_Ox.fanHandle[1] = 0;
	s_Ox.fanTCap = 0;
	s_Ox.fanRays = s_Ox.fanFaces = s_Ox.fanEyes = s_Ox.fanPlates = 0;
}

void BwOptix_Shutdown()
{
	BwOptix_FanFree();
	FanPipelineFree();
	if ( s_Ox.cudaUp )
	{
		for ( int m = 0; m < BW_MESH_COUNT; m++ )
		{
			if ( s_Ox.gas[m] )
				cudaFree( (void *)s_Ox.gas[m] );
		}
		if ( s_Ox.dArcs )
			cudaFree( (void *)s_Ox.dArcs );
		if ( s_Ox.dBlocked )
			cudaFree( (void *)s_Ox.dBlocked );
		if ( s_Ox.dParams )
			cudaFree( (void *)s_Ox.dParams );
		if ( s_Ox.sbt )
			cudaFree( (void *)s_Ox.sbt );
	}
	BwOptix_SweepFree();
	if ( s_Ox.pipeline )
		optixPipelineDestroy( s_Ox.pipeline );
	if ( s_Ox.raygen )
		optixProgramGroupDestroy( s_Ox.raygen );
	if ( s_Ox.miss )
		optixProgramGroupDestroy( s_Ox.miss );
	if ( s_Ox.hitgroup )
		optixProgramGroupDestroy( s_Ox.hitgroup );
	if ( s_Ox.sweepRaygen )
		optixProgramGroupDestroy( s_Ox.sweepRaygen );
	if ( s_Ox.module )
		optixModuleDestroy( s_Ox.module );
	if ( s_Ox.context )
		optixDeviceContextDestroy( s_Ox.context );
	memset( &s_Ox, 0, sizeof( s_Ox ) );
}
