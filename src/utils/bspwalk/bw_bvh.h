//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: a bounding volume hierarchy over boxes and the slab test for a segment
//          against a box, shared by the visibility stage (bw_vis.cpp: occluders,
//          faces) and the env_cubemap check (bw_cubemap.cpp: brushes).
//
//=============================================================================//

#ifndef BW_BVH_H
#define BW_BVH_H
#pragma once

#include <algorithm>
#include <vector>
#include "tier0/basetypes.h"
#include "bw_math.h"

struct BvhNode
{
	dvec3 mins, maxs;
	int left, right;	// children, or -1
	int first, count;	// leaf items in 'items'
};

struct Bvh
{
	std::vector<BvhNode> nodes;
	std::vector<int> items;

	void Build( const std::vector<dvec3> &mins, const std::vector<dvec3> &maxs )
	{
		nodes.clear();
		items.resize( mins.size() );
		for ( size_t i = 0; i < items.size(); i++ )
			items[i] = (int)i;
		if ( !items.empty() )
			BuildNode( mins, maxs, 0, (int)items.size() );
	}

	int BuildNode( const std::vector<dvec3> &mins, const std::vector<dvec3> &maxs, int first, int count )
	{
		int idx = (int)nodes.size();
		nodes.push_back( BvhNode() );
		BvhNode nd;
		nd.mins = dvec3( 1e300, 1e300, 1e300 );
		nd.maxs = dvec3( -1e300, -1e300, -1e300 );
		for ( int i = first; i < first + count; i++ )
		{
			nd.mins = DMin( nd.mins, mins[items[i]] );
			nd.maxs = DMax( nd.maxs, maxs[items[i]] );
		}
		nd.left = nd.right = -1;
		nd.first = first;
		nd.count = count;
		if ( count > 4 )
		{
			dvec3 ext = nd.maxs - nd.mins;
			int axis = ( ext.x >= ext.y && ext.x >= ext.z ) ? 0 : ( ext.y >= ext.z ? 1 : 2 );
			int mid = first + count / 2;
			std::nth_element( items.begin() + first, items.begin() + mid, items.begin() + first + count, [&]( int a, int b ) {
				return mins[a][axis] + maxs[a][axis] < mins[b][axis] + maxs[b][axis];
			} );
			nd.left = BuildNode( mins, maxs, first, mid - first );
			nd.right = BuildNode( mins, maxs, mid, first + count - mid );
			nd.count = 0;
		}
		nodes[idx] = nd;
		return idx;
	}
};

// Does the segment o + t dir, t in [t0, t1], meet the box?
static inline bool SegBox( const dvec3 &o, const dvec3 &inv, double t0, double t1, const dvec3 &mins, const dvec3 &maxs )
{
	for ( int a = 0; a < 3; a++ )
	{
		double ta = ( mins[a] - o[a] ) * inv[a], tb = ( maxs[a] - o[a] ) * inv[a];
		if ( ta > tb )
		{
			double t = ta;
			ta = tb;
			tb = t;
		}
		// SafeInv clamps a zero direction to 1e300, so 0 * inv is 0; a NaN bound
		// (1/d overflowing to inf for a denormal d) must not narrow [t0, t1]
		if ( ta == ta )
			t0 = MAX( t0, ta );
		if ( tb == tb )
			t1 = MIN( t1, tb );
		if ( t0 > t1 )
			return false;
	}
	return true;
}

static inline dvec3 SafeInv( const dvec3 &d )
{
	return dvec3( d.x != 0 ? 1.0 / d.x : 1e300, d.y != 0 ? 1.0 / d.y : 1e300, d.z != 0 ? 1.0 / d.z : 1e300 );
}

#endif // BW_BVH_H
