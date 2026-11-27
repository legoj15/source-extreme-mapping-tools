//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: the candidate sweep's one-way occluder filter (#141), shared by the
//          host (bw_vis.cpp, bw_selftest.cpp; MSVC) and the sweep kernel
//          (bw_arcs.cu; nvcc). No includes beyond math, no C++ beyond inline
//          functions on plain arrays.
//
//          The sweep kernel tests only the two-sided occluders, so it lists
//          its clear pairs and the CPU re-tests each (bw_vis.cpp
//          ClearRecheckBlocked). Behind a one-way occluder nearly every listed
//          pair is one the CPU then blocks. BwSweepDirSure answers, for one
//          pair and one one-way occluder, whether bw_vis.cpp DirSegmentBlocks
//          surely blocks it; the kernel drops such a pair instead of listing
//          it. It never decides a pair free: every pair it does not drop is
//          listed and re-tested exactly as before, so no verdict can change as
//          long as "sure" implies "DirSegmentBlocks blocks". Why it does:
//
//          - The test runs in double on the pair's own double ends (the face
//            sample, the float eye widened), the numbers the CPU uses, and
//            every comparison keeps BW_SWEEPDIR_EPS (1e-6u) on the safe side.
//            The ends and planes are below 2^18u in magnitude (the tool's
//            coordinate box), so a dot product or a crossing rounds by under
//            about 1e-10u: four orders below the margin, FMA or not.
//          - The crossing (DirSegmentBlocks' SegmentCrossesConvexGrown on the
//            shrunk planes): two points 1e-3u apart on the segment lie at
//            least EPS inside every shrunk plane, so every plane's computed
//            crossing on the CPU lies outside them and its interval holds
//            both: deeper than OCC_DEPTH (1e-6u).
//          - The entry (DirEntry on the real planes): the entry point lies at
//            least BW_SWEEPDIR_INSET inside every other plane, so every other
//            plane crossed inward is crossed that much earlier: no tie, the
//            entry is that plane, at least BW_SWEEPDIR_MIN_DIST from the eye.
//          - The rest (DirSegmentBlocks' run walk back from the entry, the
//            drawn sides where the run starts, and the probe OCC_SHRINK
//            outside them, in a leaf with a cluster and in no brush) depends
//            only on where on the side the segment enters and on its slope.
//            The host proves it once per side for every entry point that far
//            inside the side, every slope of at least BW_SWEEPDIR_MIN_COS and
//            every eye at least BW_SWEEPDIR_MIN_DIST away (bw_vis.cpp
//            SweepDirSideSure), and flags the side sure only then.
//
//=============================================================================//

#ifndef BW_SWEEPDIR_H
#define BW_SWEEPDIR_H

#ifdef __CUDACC__
#define BW_SWEEPDIR_FN static __forceinline__ __device__ __host__
#else
#include <math.h>
#define BW_SWEEPDIR_FN static inline
#endif

// bw_vis.h OCC_SHRINK: the one-way occluders' planes (s_DirPlanes) are their real planes moved in by this.
#define BW_SWEEPDIR_SHRINK		( 1.0 / 32.0 )
// The entry point lies at least this far inside every other plane of the occluder. It covers a
// probe up to 0.5u back along the segment (OCC_SHRINK / BW_SWEEPDIR_MIN_COS) and the crossing
// points (OCC_SHRINK / c + 2e-3 past the entry) staying inside the other shrunk planes.
#define BW_SWEEPDIR_INSET		0.625
// The segment meets the entry side at a cosine of at least this (c = -n . u): the probe then lies
// at most OCC_SHRINK / c = 0.5u back along the segment, below DirSegmentBlocks' 1u cap.
#define BW_SWEEPDIR_MIN_COS		( 1.0 / 16.0 )
// The eye is at least this far from the entry point along the segment: DirSegmentBlocks' first
// run window (1u back) then neither reaches the eye nor needs widening, and the probe lies on the
// segment.
#define BW_SWEEPDIR_MIN_DIST	2.0
#define BW_SWEEPDIR_EPS			1e-6

// Does one-way occluder 'planes' (n planes: n xyz and d of s_DirPlanes, 4 doubles each; sure[k]
// nonzero when the host proved side k, see above) surely block the view from eye v to face
// sample p, as bw_vis.cpp DirSegmentBlocks( v, p, o ) would? False whenever in doubt.
BW_SWEEPDIR_FN bool BwSweepDirSure( double vx, double vy, double vz, double px, double py, double pz, const double *planes,
									const unsigned char *sure, unsigned int n )
{
	const double dx = px - vx, dy = py - vy, dz = pz - vz;
	const double len = sqrt( dx * dx + dy * dy + dz * dz );
	if ( !( len > BW_SWEEPDIR_MIN_DIST ) )
		return false;
	// the entry: as DirEntry, the plane crossed inward last, on the real planes
	unsigned int kE = n;
	double tE = -1e300;
	for ( unsigned int k = 0; k < n; k++ )
	{
		const double *pl = planes + 4 * k;
		const double denom = pl[0] * dx + pl[1] * dy + pl[2] * dz;
		if ( !( denom < 0.0 ) )
			continue;
		const double t = ( pl[3] + BW_SWEEPDIR_SHRINK - ( pl[0] * vx + pl[1] * vy + pl[2] * vz ) ) / denom;
		if ( t > tE )
		{
			tE = t;
			kE = k;
		}
	}
	if ( kE == n || !sure[kE] )
		return false;
	const double *pe = planes + 4 * kE;
	const double c = -( pe[0] * dx + pe[1] * dy + pe[2] * dz ) / len;
	const double sE = tE * len;
	if ( !( c >= BW_SWEEPDIR_MIN_COS ) || !( sE >= BW_SWEEPDIR_MIN_DIST ) )
		return false;
	const double yx = vx + dx * tE, yy = vy + dy * tE, yz = vz + dz * tE;
	for ( unsigned int k = 0; k < n; k++ )
	{
		const double *pl = planes + 4 * k;
		if ( k != kE && pl[0] * yx + pl[1] * yy + pl[2] * yz - ( pl[3] + BW_SWEEPDIR_SHRINK ) > -( BW_SWEEPDIR_INSET + BW_SWEEPDIR_EPS ) )
			return false;
	}
	// two points past the entry, inside the shrunk volume: the first 1e-3u past the shrunk entry plane
	const double sa = sE + BW_SWEEPDIR_SHRINK / c + 1e-3;
	if ( !( sa + 1e-3 <= len ) )
		return false;
	for ( int i = 0; i < 2; i++ )
	{
		const double t = ( sa + 1e-3 * i ) / len;
		const double xx = vx + dx * t, xy = vy + dy * t, xz = vz + dz * t;
		for ( unsigned int k = 0; k < n; k++ )
		{
			const double *pl = planes + 4 * k;
			if ( pl[0] * xx + pl[1] * xy + pl[2] * xz - pl[3] > -BW_SWEEPDIR_EPS )
				return false;
		}
	}
	return true;
}

#endif // BW_SWEEPDIR_H
