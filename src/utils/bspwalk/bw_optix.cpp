//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: the OptiX arc tracer (host side). See bw_optix.h.
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
	OptixProgramGroup raygen, miss, hitgroup;
	OptixPipeline pipeline;
	CUdeviceptr sbt;
	CUdeviceptr gas[BW_MESH_COUNT];
	size_t gasBytes[BW_MESH_COUNT];
	OptixTraversableHandle handle[BW_MESH_COUNT];
	CUdeviceptr dArcs, dBlocked, dParams;
	size_t arcCap;
};
static BwOptixState s_Ox;

static void OptixLog( unsigned int level, const char *tag, const char *message, void * )
{
	if ( level <= 2 )
		Warning( "bspwalk: [OptiX %s] %s\n", tag, message );
}

// Reads <exe dir>\bspwalk_arcs.ptx. Never a cwd-relative fallback.
static bool LoadPtx( std::vector<char> &ptx, char *path, int pathLen )
{
	path[0] = 0;	// the caller prints it on failure
	DWORD n = GetModuleFileNameA( NULL, path, pathLen );
	if ( n == 0 || (int)n >= pathLen )
		return false;
	char *slash = strrchr( path, '\\' );
	if ( slash )
		slash[1] = 0;
	V_strncat( path, "bspwalk_arcs.ptx", pathLen );
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
	if ( !LoadPtx( ptx, path, sizeof( path ) ) )
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
	OptixProgramGroupDesc desc[3] = {};
	desc[0].kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
	desc[0].raygen.module = s_Ox.module;
	desc[0].raygen.entryFunctionName = "__raygen__arcs";
	desc[1].kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
	desc[1].miss.module = s_Ox.module;
	desc[1].miss.entryFunctionName = "__miss__arcs";
	desc[2].kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;	// no programs: built-in triangles, closest hit disabled per ray
	OptixProgramGroup *pgs[3] = { &s_Ox.raygen, &s_Ox.miss, &s_Ox.hitgroup };
	for ( int i = 0; i < 3; i++ )
	{
		logSize = sizeof( log );
		BW_OPTIX( optixProgramGroupCreate( s_Ox.context, &desc[i], 1, &pgOpt, log, &logSize, pgs[i] ) );
	}

	OptixPipelineLinkOptions linkOpt = {};
	linkOpt.maxTraceDepth = 1;
	OptixProgramGroup groups[3] = { s_Ox.raygen, s_Ox.miss, s_Ox.hitgroup };
	logSize = sizeof( log );
	BW_OPTIX( optixPipelineCreate( s_Ox.context, &pipeOpt, &linkOpt, groups, 3, log, &logSize, &s_Ox.pipeline ) );

	struct SbtRecord
	{
		__declspec( align( OPTIX_SBT_RECORD_ALIGNMENT ) ) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
	};
	SbtRecord rec[3];
	for ( int i = 0; i < 3; i++ )
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

static bool BuildMeshInner( int mesh, const std::vector<float> &verts, const std::vector<unsigned int> &tris,
							CUdeviceptr &dVerts, CUdeviceptr &dTris, CUdeviceptr &dTemp, CUdeviceptr &dOut, CUdeviceptr &dSize )
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
	unsigned int flags[1] = { OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT | OPTIX_GEOMETRY_FLAG_DISABLE_TRIANGLE_FACE_CULLING };
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
		s_Ox.gasBytes[mesh] = (size_t)compacted;
	}
	else
		s_Ox.gasBytes[mesh] = sizes.outputSizeInBytes;
	s_Ox.gas[mesh] = dOut;
	s_Ox.handle[mesh] = handle;
	dOut = 0;
	return true;
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
	CUdeviceptr dVerts = 0, dTris = 0, dTemp = 0, dOut = 0, dSize = 0;
	bool ok = BuildMeshInner( mesh, verts, tris, dVerts, dTris, dTemp, dOut, dSize );
	cudaFree( (void *)dVerts );
	cudaFree( (void *)dTris );
	cudaFree( (void *)dTemp );
	cudaFree( (void *)dSize );
	if ( dOut )
		cudaFree( (void *)dOut );
	return ok;
}

bool BwOptix_TraceArcs( const BwArc *arcs, unsigned int n, float gravity, unsigned char *blocked )
{
	if ( !s_Ox.ready )
		return false;
	if ( !n )
		return true;
	if ( n > s_Ox.arcCap )
	{
		if ( s_Ox.dArcs )
			cudaFree( (void *)s_Ox.dArcs );
		if ( s_Ox.dBlocked )
			cudaFree( (void *)s_Ox.dBlocked );
		s_Ox.dArcs = s_Ox.dBlocked = 0;
		s_Ox.arcCap = 0;
		BW_CUDA( cudaMalloc( (void **)&s_Ox.dArcs, (size_t)n * sizeof( BwArc ) ) );
		BW_CUDA( cudaMalloc( (void **)&s_Ox.dBlocked, n ) );
		s_Ox.arcCap = n;
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

void BwOptix_Shutdown()
{
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
	if ( s_Ox.pipeline )
		optixPipelineDestroy( s_Ox.pipeline );
	if ( s_Ox.raygen )
		optixProgramGroupDestroy( s_Ox.raygen );
	if ( s_Ox.miss )
		optixProgramGroupDestroy( s_Ox.miss );
	if ( s_Ox.hitgroup )
		optixProgramGroupDestroy( s_Ox.hitgroup );
	if ( s_Ox.module )
		optixModuleDestroy( s_Ox.module );
	if ( s_Ox.context )
		optixDeviceContextDestroy( s_Ox.context );
	memset( &s_Ox, 0, sizeof( s_Ox ) );
}
