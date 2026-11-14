//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: OptiX arc kernel (compiled to game/bin/x64/bspwalk_arcs.ptx by
//          optix_compile.bat). One thread per arc: its chords are traced in
//          order as rays against the triangle mesh the arc names (BwArc::mesh,
//          BW_MESH_*): the grown obstacles of a posture for the ballistic arcs,
//          or the shrunk opaque occluders (BW_MESH_VIS) for the tier B sight
//          segments of bw_vis.cpp, which are zero-gravity one-chord arcs.
//          Tracing stops at the first hit. The chord times and the joint
//          overlap are the same as the CPU reference in bw_arcs.cpp
//          (TraceArcCpu), so the two differ only by float rounding.
//
//=============================================================================//

#include <optix.h>
#include "bw_arc_types.h"

extern "C" {
__constant__ BwArcLaunchParams params;
}

static __forceinline__ __device__ float3 ArcPos( const BwArc &a, float g, float t )
{
	float z;
	const float g0 = g + a.dg0;
	if ( a.tKick > 0.0f && t > a.tKick )
	{
		float zk = a.oz + a.vz * a.tKick - 0.5f * g0 * a.tKick * a.tKick;
		float u = t - a.tKick;
		z = zk + a.vzKick * u - 0.5f * g * u * u;
	}
	else
		z = a.oz + a.vz * t - 0.5f * g0 * t * t;
	return make_float3( a.ox + a.vx * t, a.oy + a.vy * t, z );
}

extern "C" __global__ void __raygen__arcs()
{
	const unsigned int i = optixGetLaunchIndex().x;
	if ( i >= params.numArcs )
		return;
	const BwArc a = params.arcs[i];
	const float g = params.gravity;
	const OptixTraversableHandle gas = (OptixTraversableHandle)params.gas[a.mesh];
	const int total = (int)a.nSeg[0] + (int)a.nSeg[1];

	unsigned char blocked = 0;
	int k = 0;
	for ( int piece = 0; piece < 2 && !blocked; piece++ )
	{
		const int n = (int)a.nSeg[piece];
		if ( !n )
			continue;
		const float ts = piece ? a.tKick : 0.0f;
		const float te = ( piece || a.tKick <= 0.0f ) ? a.tEnd : a.tKick;
		float3 p = ArcPos( a, g, ts );
		for ( int s = 1; s <= n && !blocked; s++, k++ )
		{
			const float t = ts + ( te - ts ) * ( (float)s / (float)n );
			const float3 q = ArcPos( a, g, t );
			const float3 d = make_float3( q.x - p.x, q.y - p.y, q.z - p.z );
			const float len = sqrtf( d.x * d.x + d.y * d.y + d.z * d.z );
			if ( len > 0.0f )
			{
				// interior joints overlap; the arc's own ends are exact (the host lifts
				// every arc off its floors, so both ends lie in free space)
				const float lo = ( k == 0 ) ? 0.0f : -BW_ARC_JOINT_OVERLAP / len;
				const float hi = ( k == total - 1 ) ? 1.0f : 1.0f + BW_ARC_JOINT_OVERLAP / len;
				if ( hi > lo )
				{
					// OptiX needs tmin >= 0: start the ray at p + lo d instead
					const float3 o = make_float3( p.x + lo * d.x, p.y + lo * d.y, p.z + lo * d.z );
					unsigned int hit = 1;	// stays 1 unless the miss program runs
					optixTrace( gas, o, d, 0.0f, hi - lo, 0.0f, OptixVisibilityMask( 255 ),
								OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT | OPTIX_RAY_FLAG_DISABLE_ANYHIT |
									OPTIX_RAY_FLAG_DISABLE_CLOSESTHIT,
								0, 1, 0, hit );
					blocked = hit ? 1 : 0;
				}
			}
			p = q;
		}
	}
	params.blocked[i] = blocked;
}

extern "C" __global__ void __miss__arcs()
{
	optixSetPayload_0( 0u );
}

// bw_vis.cpp Mix64 (splitmix64's finalizer).
static __forceinline__ __device__ unsigned long long Mix64( unsigned long long x )
{
	x += 0x9e3779b97f4a7c15ULL;
	x = ( x ^ ( x >> 30 ) ) * 0xbf58476d1ce4e5b9ULL;
	x = ( x ^ ( x >> 27 ) ) * 0x94d049bb133111ebULL;
	return x ^ ( x >> 31 );
}

// The #35 sweep: launch (rays, eyes). Thread (r, i) makes the pair bw_vis.cpp's
// ConfirmPairs makes for ray r of eye i of rank sweepRank[i] (the same hashed stratum, the same
// back-face test in double, without FMA contraction so it rounds as the host
// does) and traces it as __raygen__arcs traces the zero-gravity one-chord arc
// BwVis_EncodeSegment makes of it: the same float origin and direction, so
// both paths answer alike. A clear pair sets *sweepClear; threads that start
// after that skip their trace.
extern "C" __global__ void __raygen__sweep()
{
	const uint3 li = optixGetLaunchIndex();
	const unsigned int r = li.x, i = li.y;
	if ( r >= params.sweepRays || i >= params.sweepEyes )
		return;
	// listing (#114): stop once the list is full (the caller falls back); else at the first clear pair
	if ( params.sweepOut ? *(volatile unsigned int *)params.sweepOutCount >= params.sweepOutCap
						 : *(volatile unsigned int *)params.sweepClear != 0 )
		return;
	const unsigned long long h = Mix64( params.sweepKey ^ params.sweepRank[i] );
	const double u = (double)( Mix64( h + (unsigned long long)r ) >> 11 ) * ( 1.0 / 9007199254740992.0 );
	const unsigned int nS = params.sweepNumSamples;
	unsigned long long k = (unsigned long long)__ddiv_rn( __dmul_rn( __dadd_rn( (double)r, u ), (double)nS ),
														   (double)params.sweepRays );
	if ( k > nS - 1 )
		k = nS - 1;
	const double *s = params.sweepSamples + 6 * k;
	// The float index is 64-bit: 3 * e in 32 bits wraps from 1.43e9 eyes up.
	const unsigned long long e3 = 3ull * params.sweepIdx[i];
	const double ex = (double)params.eyes[e3], ey = (double)params.eyes[e3 + 1], ez = (double)params.eyes[e3 + 2];
	const double dx = __dsub_rn( ex, s[0] ), dy = __dsub_rn( ey, s[1] ), dz = __dsub_rn( ez, s[2] );
	if ( !params.sweepTwoSided &&
		 __dadd_rn( __dadd_rn( __dmul_rn( dx, s[3] ), __dmul_rn( dy, s[4] ) ), __dmul_rn( dz, s[5] ) ) <= 0.0 )
		return;	// the sample's back side
	const float3 o = make_float3( __double2float_rn( s[0] ), __double2float_rn( s[1] ), __double2float_rn( s[2] ) );
	const float3 v = make_float3( __double2float_rn( dx ), __double2float_rn( dy ), __double2float_rn( dz ) );
	const float3 q = make_float3( __fadd_rn( o.x, v.x ), __fadd_rn( o.y, v.y ), __fadd_rn( o.z, v.z ) );
	const float3 d = make_float3( __fsub_rn( q.x, o.x ), __fsub_rn( q.y, o.y ), __fsub_rn( q.z, o.z ) );
	unsigned int hit = 0;
	if ( sqrtf( d.x * d.x + d.y * d.y + d.z * d.z ) > 0.0f )
	{
		hit = 1;	// stays 1 unless the miss program runs
		optixTrace( (OptixTraversableHandle)params.gas[BW_MESH_VIS], o, d, 0.0f, 1.0f, 0.0f, OptixVisibilityMask( 255 ),
					OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT | OPTIX_RAY_FLAG_DISABLE_ANYHIT | OPTIX_RAY_FLAG_DISABLE_CLOSESTHIT,
					0, 1, 0, hit );
	}
	if ( !hit )
	{
		*params.sweepClear = 1;
		if ( params.sweepOut )
		{
			const unsigned int slot = atomicAdd( params.sweepOutCount, 1u );
			if ( slot < params.sweepOutCap )
				params.sweepOut[slot] = (unsigned long long)i * params.sweepRays + r;
		}
	}
}
