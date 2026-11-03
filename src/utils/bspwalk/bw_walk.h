//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: Stage 2 (exact walkable polygons) and the CPU part of Stage 3 (walk
//          graph, seeds, BFS flood).
//
//          A walkable polygon is an up face (normal z >= 0.7) of a grown
//          polytope minus every other grown obstacle of the same posture. The
//          feet point can rest anywhere on it. Graph nodes are the crouch
//          polygons (a player can always crouch-walk); each node also carries
//          its stand sub-polygons (clipped further by stand-grown obstacles).
//
//          Every clipper is shrunk by 1/32u (plane d -= 1/32), so any epsilon
//          errs toward more free space, never less.
//
//=============================================================================//

#ifndef BW_WALK_H
#define BW_WALK_H
#pragma once

#include <vector>
#include "bw_world.h"

// Every clipper and blocker is shrunk by this much: over-reach only costs
// savings, under-reach blacks out faces.
#define WALK_SHRINK		( 1.0 / 32.0 )
// Polygons are chopped at multiples of this in X and Y so edges stay local.
#define WALK_CHOP		64.0
// Up faces: normal z at least this (Source's walkable limit).
#define WALK_MIN_NZ		0.7

//-----------------------------------------------------------------------------
// Polygon minus convex polytope. 'poly' is a convex planar polygon; the
// polytope is { x : planes[k].n . x <= planes[k].d - shrink for all k }.
// Appends the convex pieces of poly outside the polytope to 'out'. A polygon
// that does not reach into the polytope's interior is appended unchanged.
// Returns false in that case, true if it was cut.
//-----------------------------------------------------------------------------
bool SubtractPolytope( const DWinding &poly, const DPlane *planes, int numPlanes, double shrink,
					   std::vector<DWinding> &out );
// Area of a polygon that is not a degenerate sliver (see bw_walk.cpp), 0 otherwise.
double WalkPolyArea( const DWinding &w );

enum WalkEdgeKind
{
	EDGE_WALK = 0,	// |dz| <= step height, both directions (stored once, a < b)
	EDGE_JUMP,		// a -> b, vertical duck-jump up: step < dz <= max jump rise
	EDGE_DROP,		// a -> b, falling straight down: dz < -step, any height
	EDGE_KIND_COUNT
};
const char *WalkEdgeKindName( int kind );

struct WalkNode
{
	DWinding poly;					// crouch polygon, world space, CCW seen from above
	DPlane plane;					// n . x = d on the polygon, n.z >= WALK_MIN_NZ
	dvec3 mins, maxs;
	double area;
	int category;					// ConvexCategory of the source
	int convex;						// index into CollisionWorld::convexes
	int pose;						// mover pose index (0 = spawn) for tops_only; 0 otherwise
	std::vector<DWinding> stand;	// stand sub-polygons (clipped by stand-grown obstacles)
	double standArea;

	double Z( double x, double y ) const { return ( plane.d - plane.n.x * x - plane.n.y * y ) / plane.n.z; }
};

struct WalkEdge
{
	int a, b;
	int kind;
};

struct WalkSeed
{
	dvec3 point;
	char source[64];	// classname, or "cli"
	int node;			// -1 when it maps to no node
};

struct WalkStats
{
	int upFaces;				// up faces considered (all postures' sources, all poses)
	int moverEntities;			// tops_only entities with at least one pose
	int moverPoses;				// poses over all of them
	int moverGrowFailed;		// mover convexes that failed to grow at some pose
	int edgeCounts[EDGE_KIND_COUNT];
	int reachedCount;
	double reachedArea;
	double totalArea;
	double tPolys, tStand, tEdges, tFlood;		// Stage 2 polygons, stand sub-polygons, Stage 3 edges, seeds + BFS

	WalkStats() { memset( this, 0, sizeof( *this ) ); }
};

// A tops_only convex grown (crouch hull) at one pose of its entity. Only its
// up faces are used: movers are never obstacles. No stand grow: up faces are
// the same for both hulls (#13 adds it if ride edges need the stand hull).
struct MoverPolytope
{
	int convex;
	int pose;
	DTransform xf;		// the pose (identity at spawn)
	GrownPolytope crouch;
};

struct WalkGraph
{
	std::vector<MoverPolytope> movers;
	std::vector<WalkNode> nodes;
	std::vector<WalkEdge> edges;
	std::vector<WalkSeed> seeds;
	std::vector<unsigned char> reached;	// per node
	WalkStats stats;
};

// Stage 2: walkable polygons (nodes, with stand sub-polygons). Multithreaded.
void BuildWalkPolygons( const CollisionWorld &world, const PlayerProfile &profile, WalkGraph &graph );
// Stage 3 (CPU): walk / jump-up / drop edges. Multithreaded.
void BuildWalkEdges( const CollisionWorld &world, const PlayerProfile &profile, WalkGraph &graph );
// Seeds from the entity lump (info_player_*, info_ladder_dismount) plus the
// given CLI points. Maps each to a node and floods. Returns false (with a
// message printed) when there are no seeds or none maps: a hard error.
bool FloodWalkGraph( const CollisionWorld &world, const PlayerProfile &profile, const std::vector<dvec3> &cliSeeds,
					 WalkGraph &graph );

#endif // BW_WALK_H
