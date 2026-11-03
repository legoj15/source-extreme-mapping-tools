//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: the hierarchical XY hash and the per-posture obstacle sets shared
//          by the walk graph (bw_walk.cpp) and the arc tracer (bw_arcs.cpp).
//
//=============================================================================//

#ifndef BW_HASH_H
#define BW_HASH_H
#pragma once

#include <algorithm>
#include <unordered_map>
#include <vector>
#include "threads.h"
#include "bw_world.h"

// RunThreadsOnIndividual in blocks (bw_walk.cpp).
void RunThreadsBatched( int count, ThreadWorkerFn fn );

//-----------------------------------------------------------------------------
// Uniform XY hash, in levels (cell 64u * 4^L) so a 30000u brush does not fill
// millions of cells: each box goes to the finest level where it spans at most
// three cells per axis. Queries visit every level.
//-----------------------------------------------------------------------------
struct XYBox
{
	double x0, y0, x1, y1;
};

class CXYHash
{
public:
	CXYHash() : m_nObjects( 0 ) {}

	// boxes[i] is object i; objects with use[i] == 0 are left out.
	void Build( const std::vector<XYBox> &boxes, const std::vector<unsigned char> &use )
	{
		m_nObjects = (int)boxes.size();
		m_Levels.clear();
		m_Levels.resize( NUM_LEVELS );
		std::vector< std::vector< std::pair<unsigned long long, int> > > pairs( NUM_LEVELS );
		for ( int L = 0; L < NUM_LEVELS; L++ )
		{
			m_Levels[L].cell = 64.0 * pow( 4.0, L );
			m_Levels[L].inv = 1.0 / m_Levels[L].cell;
		}
		for ( int i = 0; i < m_nObjects; i++ )
		{
			if ( !use[i] )
				continue;
			const XYBox &b = boxes[i];
			double w = MAX( b.x1 - b.x0, b.y1 - b.y0 );
			int L = 0;
			while ( L < NUM_LEVELS - 1 && m_Levels[L].cell * 2.0 < w )
				L++;
			const Level &lv = m_Levels[L];
			int ix0 = Cell( b.x0, lv.inv ), ix1 = Cell( b.x1, lv.inv );
			int iy0 = Cell( b.y0, lv.inv ), iy1 = Cell( b.y1, lv.inv );
			for ( int ix = ix0; ix <= ix1; ix++ )
				for ( int iy = iy0; iy <= iy1; iy++ )
					pairs[L].push_back( std::make_pair( Key( ix, iy ), i ) );
		}
		for ( int L = 0; L < NUM_LEVELS; L++ )
		{
			std::vector< std::pair<unsigned long long, int> > &p = pairs[L];
			std::sort( p.begin(), p.end() );
			Level &lv = m_Levels[L];
			lv.ids.resize( p.size() );
			lv.cells.reserve( p.size() / 2 + 1 );
			for ( size_t k = 0; k < p.size(); )
			{
				size_t e = k;
				while ( e < p.size() && p[e].first == p[k].first )
				{
					lv.ids[e] = p[e].second;
					e++;
				}
				lv.cells[p[k].first] = std::make_pair( (int)k, (int)e );
				k = e;
			}
		}
	}

	// Appends each object whose cells meet the query box, once per call, in
	// ascending id order (deterministic whatever the levels). Duplicates (an
	// object in several of the visited cells) are removed by sorting the
	// appended range: O(candidates), where a per-thread stamp array would
	// cost objects x threads (1.8M nodes x 33 threads = 240 MB on fightspace3).
	void Query( double x0, double y0, double x1, double y1, std::vector<int> &out ) const
	{
		size_t first = out.size();
		for ( int L = 0; L < NUM_LEVELS; L++ )
		{
			const Level &lv = m_Levels[L];
			if ( lv.ids.empty() )
				continue;
			int ix0 = Cell( x0, lv.inv ), ix1 = Cell( x1, lv.inv );
			int iy0 = Cell( y0, lv.inv ), iy1 = Cell( y1, lv.inv );
			for ( int ix = ix0; ix <= ix1; ix++ )
			{
				for ( int iy = iy0; iy <= iy1; iy++ )
				{
					std::unordered_map< unsigned long long, std::pair<int, int> >::const_iterator it = lv.cells.find( Key( ix, iy ) );
					if ( it == lv.cells.end() )
						continue;
					out.insert( out.end(), lv.ids.begin() + it->second.first, lv.ids.begin() + it->second.second );
				}
			}
		}
		std::sort( out.begin() + first, out.end() );
		out.erase( std::unique( out.begin() + first, out.end() ), out.end() );
	}

private:
	enum { NUM_LEVELS = 9 };	// 64u .. 4.2M u cells
	struct Level
	{
		double cell, inv;
		std::unordered_map< unsigned long long, std::pair<int, int> > cells;	// key -> [begin, end) in ids
		std::vector<int> ids;
	};
	static int Cell( double v, double inv ) { return (int)floor( v * inv ); }
	static unsigned long long Key( int ix, int iy ) { return ( (unsigned long long)(unsigned int)ix << 32 ) | (unsigned int)iy; }

	std::vector<Level> m_Levels;
	int m_nObjects;
};

//-----------------------------------------------------------------------------
// Obstacle sets: per posture, the CategoryIsObstacle polytopes of the world,
// hashed by XY. Object id = convex index.
//-----------------------------------------------------------------------------
struct ObstacleSet
{
	const std::vector<GrownPolytope> *polys;
	std::vector<unsigned char> use;
	CXYHash hash;

	void Build( const CollisionWorld &world, const std::vector<GrownPolytope> &posture )
	{
		polys = &posture;
		std::vector<XYBox> boxes( world.convexes.size() );
		use.assign( world.convexes.size(), 0 );
		for ( size_t i = 0; i < world.convexes.size(); i++ )
		{
			const GrownPolytope &p = posture[i];
			if ( !CategoryIsObstacle( world.convexes[i].category ) || !p.IsValid() )
				continue;
			use[i] = 1;
			boxes[i].x0 = p.mins.x;
			boxes[i].y0 = p.mins.y;
			boxes[i].x1 = p.maxs.x;
			boxes[i].y1 = p.maxs.y;
		}
		hash.Build( boxes, use );
	}
};

#endif // BW_HASH_H
