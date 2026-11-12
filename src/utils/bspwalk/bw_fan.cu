//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: OptiX eye fan kernel for #14 tier B (compiled to
//          game/bin/x64/bspwalk_fan.ptx by optix_compile.bat). One thread per
//          (fan direction, eye): a closest-hit trace against the occluder mesh
//          gives the first occluder, an any-hit trace against the one-way
//          occluders (#77, __anyhit__dir) may shorten it, then an any-hit trace
//          against the padded face slabs, up to that distance plus 'past', marks
//          every face it meets that the eye cluster's PVS holds; then the
//          zero-area faces, tested directly (MarkPlates).
//          The host builds the three meshes and the plates with margins meant to
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

// The PVS gate (bw_vis.cpp FanEyeMaySee): may eye e of this launch see face f?
static __forceinline__ __device__ bool EyeMaySee( unsigned int e, unsigned int f )
{
	const int c = fanParams.eyeCl[e];
	if ( c < 0 )
		return true;
	const int a = fanParams.faceClOfs[f], b = fanParams.faceClOfs[f + 1];
	if ( a == b )
		return true;
	const unsigned char *row = fanParams.pvs + (size_t)c * fanParams.pvsRowBytes;
	for ( int k = a; k < b; k++ )
	{
		const int q = fanParams.faceCl[k];
		if ( q == c || ( ( row[q >> 3] >> ( q & 7 ) ) & 1 ) )
			return true;
	}
	return false;
}

// The zero-area faces (BwFanLaunchParams::plates). A slab mesh would have to
// span the whole map for them, and its huge triangles' boxes overlap every
// ray, so every ray paid for them on every launch. Here a marked one costs
// each ray its record's two header words and one byte read. The interval
// test is the slab test: float error in a and
// b moves a true crossing by far less than 'pad', so it stays inside.
static __forceinline__ __device__ void MarkPlates( unsigned int e, const float3 &o, const float3 &d, float tLim )
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
		if ( in && EyeMaySee( e, f ) )
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
	// then the one-way occluders up to that distance (#77): __anyhit__dir lets through what
	// bw_vis.cpp NearestOccluder might not stop
	if ( fanParams.dirGas )
		optixTrace( (OptixTraversableHandle)fanParams.dirGas, o, d, 0.0f, __uint_as_float( tBits ), 0.0f,
					OptixVisibilityMask( 255 ), OPTIX_RAY_FLAG_NONE, 2, 1, 0, tBits );
	const float t = __uint_as_float( tBits );
	if ( fanParams.tOut )
		fanParams.tOut[(size_t)e * fanParams.numRays + r] = t;

	// every face slab up to the occluder plus 'past': the any-hit program marks
	// and ignores each, so the traversal runs on to tMax
	unsigned int unused = 0;
	if ( fanParams.faceGas )
		optixTrace( (OptixTraversableHandle)fanParams.faceGas, o, d, 0.0f, t + fanParams.past, 0.0f, OptixVisibilityMask( 255 ),
					OPTIX_RAY_FLAG_DISABLE_CLOSESTHIT, 1, 1, 1, unused );
	MarkPlates( e, o, d, t + fanParams.past );
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

// A ray that meets a one-way occluder's hull stops there only where bw_vis.cpp
// NearestOccluder surely stops it (DirEntry): the last real plane it crosses inward (the entry)
// is drawn, the eye lies more than dirSlop outside it, and every nodraw plane it crosses inward
// is crossed earlier by more than dirSlop / c over both planes (c = -n . dir), past the float
// rounding of a crossing (under 0.01u / c at |coord| <= 16384). A ray running within 1e-4 of
// parallel to a plane it lies within dirSlop of may be outside it on the CPU: it runs on. Every
// doubt lets the ray run on, which only marks more. The hull is shrunk past the CPU's, so a ray
// stopped here stops no earlier than on the CPU.
extern "C" __global__ void __anyhit__dir()
{
	const unsigned int ob = fanParams.dirPrim[optixGetPrimitiveIndex()];
	const float3 d = optixGetWorldRayDirection();
	const float3 o = optixGetWorldRayOrigin();
	const float slop = fanParams.dirSlop;
	const unsigned int a = fanParams.dirOfs[ob], b = fanParams.dirOfs[ob + 1];
	float sE = -3.0e38f, cE = 1.0f;
	unsigned int kE = b;
	for ( unsigned int k = a; k < b; k++ )
	{
		const float *p = &fanParams.dirPlanes[4 * k];
		const float c = -( p[0] * d.x + p[1] * d.y + p[2] * d.z );
		const float out = p[0] * o.x + p[1] * o.y + p[2] * o.z - p[3];	// > 0: the eye is outside
		if ( c <= 1e-4f )
		{
			if ( c > -1e-4f && out > -slop )
			{
				optixIgnoreIntersection();
				return;
			}
			continue;	// crossed outward, or parallel well inside
		}
		const float s = out / c;	// where the ray crosses the plane
		if ( s > sE )
		{
			sE = s;
			cE = c;
			kE = k;
		}
	}
	if ( kE == b || !fanParams.dirDrawn[kE] || sE * cE <= slop )
	{
		optixIgnoreIntersection();
		return;
	}
	for ( unsigned int k = a; k < b; k++ )
	{
		if ( k == kE || fanParams.dirDrawn[k] )
			continue;
		const float *p = &fanParams.dirPlanes[4 * k];
		const float c = -( p[0] * d.x + p[1] * d.y + p[2] * d.z );
		if ( c <= 1e-4f )
			continue;
		const float s = ( p[0] * o.x + p[1] * o.y + p[2] * o.z - p[3] ) / c;
		if ( sE - s <= slop / cE + slop / c )
		{
			optixIgnoreIntersection();
			return;
		}
	}
}

extern "C" __global__ void __anyhit__face()
{
	const unsigned int f = fanParams.primFace[optixGetPrimitiveIndex()];
	// a stale read only repeats a store of the same value
	if ( !fanParams.visible[f] )
	{
		const float *n = &fanParams.faceN[4 * f];
		const float3 d = optixGetWorldRayDirection();
		if ( ( n[3] == 0.0f || n[0] * d.x + n[1] * d.y + n[2] * d.z < fanParams.facingTol ) &&
			 EyeMaySee( optixGetLaunchIndex().y, f ) )
			fanParams.visible[f] = 1;
	}
	optixIgnoreIntersection();
}
