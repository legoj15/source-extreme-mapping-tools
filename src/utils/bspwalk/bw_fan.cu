//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: OptiX eye fan kernel for #14 tier B (compiled to
//          game/bin/x64/bspwalk_fan.ptx by optix_compile.bat). One thread per
//          (fan direction, eye): a closest-hit trace against the occluder mesh
//          gives the first occluder, then an any-hit trace against the padded
//          face slabs, up to that distance plus 'past', marks every face it
//          meets; then the zero-area faces, tested directly (MarkPlates).
//          The host builds both meshes and the plates with margins meant to
//          make this mark every face the CPU reference (bw_vis.cpp, FanOne)
//          marks. One case the margins cannot cover: a ray that starts inside
//          a face's slab meets only the slab's far side; the host marks those
//          faces itself (FanEyeInSlab). -vischeck's fan_cpu_only_faces is the
//          test that the rest holds.
//
//=============================================================================//

#include <optix.h>
#include "bw_arc_types.h"

extern "C" {
__constant__ BwFanLaunchParams fanParams;
}

// Narrows [lo, hi] to the t where a + b t <= w; false once it is empty.
static __forceinline__ __device__ bool Clip( float a, float b, float w, float &lo, float &hi )
{
	if ( b == 0.0f )
		return a <= w;
	const float t = ( w - a ) / b;
	if ( b > 0.0f )
		hi = fminf( hi, t );
	else
		lo = fmaxf( lo, t );
	return lo <= hi;
}

// The zero-area faces (BwFanLaunchParams::plates). A slab mesh would have to
// span the whole map for them, and its huge triangles' boxes overlap every
// ray, so every ray paid for them on every launch. Here a marked one costs
// each ray its record's two header words and one byte read. The interval
// test is the slab test: float error in a and
// b moves a true crossing by far less than 'pad', so it stays inside.
static __forceinline__ __device__ void MarkPlates( const float3 &o, const float3 &d, float tLim )
{
	const float *p = fanParams.plates;
	const float pad = fanParams.pad;
	for ( unsigned int i = 0; i < fanParams.numPlates; i++ )
	{
		const unsigned int f = __float_as_uint( p[0] ), h = __float_as_uint( p[1] );
		const float *pl = p + 2;
		const float *hp = p + 6;
		p = hp + 4 * h;
		if ( fanParams.visible[f] )
			continue;
		const float *n = &fanParams.faceN[4 * f];
		if ( n[3] != 0.0f && n[0] * d.x + n[1] * d.y + n[2] * d.z >= fanParams.facingTol )
			continue;
		float lo = 0.0f, hi = tLim;
		const float a = pl[0] * o.x + pl[1] * o.y + pl[2] * o.z - pl[3];
		const float b = pl[0] * d.x + pl[1] * d.y + pl[2] * d.z;
		if ( !Clip( a, b, pad, lo, hi ) || !Clip( -a, -b, pad, lo, hi ) )
			continue;
		bool in = true;
		for ( unsigned int k = 0; k < h && in; k++ )
		{
			const float *m = hp + 4 * k;
			in = Clip( m[0] * o.x + m[1] * o.y + m[2] * o.z - m[3], m[0] * d.x + m[1] * d.y + m[2] * d.z, pad, lo, hi );
		}
		if ( in )
			fanParams.visible[f] = 1;
	}
}

extern "C" __global__ void __raygen__fan()
{
	const uint3 idx = optixGetLaunchIndex();
	const unsigned int r = idx.x, e = idx.y;
	const float3 o = make_float3( fanParams.eyes[3 * e], fanParams.eyes[3 * e + 1], fanParams.eyes[3 * e + 2] );
	const float3 d = make_float3( fanParams.dirs[3 * r], fanParams.dirs[3 * r + 1], fanParams.dirs[3 * r + 2] );

	// The miss program leaves the payload at maxT. A mesh with no triangles has
	// no traversable (handle 0); the OptiX headers do not define a trace on
	// that, so it is skipped: no occluder, or no face slab.
	unsigned int tBits = __float_as_uint( fanParams.maxT );
	if ( fanParams.occGas )
		optixTrace( (OptixTraversableHandle)fanParams.occGas, o, d, 0.0f, fanParams.maxT, 0.0f, OptixVisibilityMask( 255 ),
					OPTIX_RAY_FLAG_DISABLE_ANYHIT, 0, 1, 0, tBits );
	const float t = __uint_as_float( tBits );
	if ( fanParams.tOut )
		fanParams.tOut[(size_t)e * fanParams.numRays + r] = t;

	// every face slab up to the occluder plus 'past': the any-hit program marks
	// and ignores each, so the traversal runs on to tMax
	unsigned int unused = 0;
	if ( fanParams.faceGas )
		optixTrace( (OptixTraversableHandle)fanParams.faceGas, o, d, 0.0f, t + fanParams.past, 0.0f, OptixVisibilityMask( 255 ),
					OPTIX_RAY_FLAG_DISABLE_CLOSESTHIT, 1, 1, 1, unused );
	MarkPlates( o, d, t + fanParams.past );
}

extern "C" __global__ void __closesthit__occ()
{
	optixSetPayload_0( __float_as_uint( optixGetRayTmax() ) );
}

extern "C" __global__ void __miss__occ()
{
}

extern "C" __global__ void __miss__face()
{
}

extern "C" __global__ void __anyhit__face()
{
	const unsigned int f = fanParams.primFace[optixGetPrimitiveIndex()];
	// a stale read only repeats a store of the same value
	if ( !fanParams.visible[f] )
	{
		const float *n = &fanParams.faceN[4 * f];
		const float3 d = optixGetWorldRayDirection();
		if ( n[3] == 0.0f || n[0] * d.x + n[1] * d.y + n[2] * d.z < fanParams.facingTol )
			fanParams.visible[f] = 1;
	}
	optixIgnoreIntersection();
}
