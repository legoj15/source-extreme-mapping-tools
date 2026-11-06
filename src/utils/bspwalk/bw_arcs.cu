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
