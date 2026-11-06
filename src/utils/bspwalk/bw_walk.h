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
#include "bw_arc_types.h"

// Every clipper and blocker is shrunk by this much: over-reach only costs
// savings, under-reach blacks out faces.
#define WALK_SHRINK		( 1.0 / 32.0 )
// Polygons are chopped at multiples of this in X and Y so edges stay local.
#define WALK_CHOP		64.0
// Up faces: normal z at least this (Source's walkable limit).
#define WALK_MIN_NZ		0.7
// Steep faces with SLIDE_MIN_NZ <= n.z < WALK_MIN_NZ become slide nodes (#13B):
// landable, never standable, left only downhill. Steeper faces are walls (a fall
// along one moves the player sideways by at most 0.1 of the drop).
#define SLIDE_MIN_NZ	0.1
// Surfing (#24) needs a connected slide surface at least this big (u^2): a 64u
// square. The chamfers of a prop's collision hull are slides too, but too small
// to ride; giving them full-speed surf launches let f05's 40u stove launch the
// player onto a 176u ledge.
#define SURF_MIN_AREA	4096.0

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
	EDGE_ARC,		// a -> b, a free ballistic arc (jump, duck-jump or walk-off; bw_arcs.h)
	EDGE_SLIDE,		// a -> b, a walk-height boundary with a slide node: onto it, or off it not uphill
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
	unsigned char slide = 0;		// a steep face (SLIDE_MIN_NZ <= n.z < WALK_MIN_NZ): a slide node
	unsigned char surf = 0;			// a slide on a surface big enough to surf (SURF_MIN_AREA, #24)

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
	bool vehicle;		// prop_vehicle_* (-vehicle-free floods from it)
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
	int slideNodes;				// nodes that are slides (steep faces)
	int offMapNodes;			// polygons dropped as outside the map (#21)
	int surfNodes;				// nodes first reached by a surf move (#24)
	double offMapArea;
	double tPolys, tStand, tEdges, tFlood;		// Stage 2 polygons, stand sub-polygons, Stage 3 edges, seeds + BFS
	double tSpecial;			// special movement graph (water, ladders, triggers, rides)

	WalkStats() { memset( this, 0, sizeof( *this ) ); }
};

// Arc pass options (CLI: -noarcs, -cpuarcs, -arccheck, -arcallnodes).
struct ArcOptions
{
	bool enabled;
	bool cpuOnly;		// never try the GPU
	bool check;			// trace every arc on both paths; the flood follows the CPU result
	bool allNodes;		// every reached node launches arcs, not only rim and slope nodes (to measure that rule)
	bool vehicleFree;	// flood option (-vehicle-free): from vehicle seeds, walk/jump/drop edges both ways

	ArcOptions() : enabled( true ), cpuOnly( false ), check( false ), allNodes( false ), vehicleFree( false ) {}
};

struct ArcRoundStats
{
	int frontier;			// nodes reached since the previous round
	int launch;				// of those, the launch nodes (rim and slope, or all with -arcallnodes)
	long long pairs;		// (launch, target) node pairs in range
	long long arcs;			// arcs traced
	long long arcsFree;
	long long pairsFree;	// pairs given an arc edge
	int reached;			// nodes newly reached in the round (arc landings and their BFS)
	double tGen, tTrace;
};

struct ArcStats
{
	const char *path;		// "gpu", "cpu" or "off"
	double reach;			// the profile's R(0)
	double reachMax;		// R(arc_max_drop), the widest pair search
	bool allNodes;			// -arcallnodes
	int rimNodes;			// nodes with a boundary part no walk edge covers
	int slopeNodes;			// interior nodes steep enough to launch (bw_arcs.cpp SlopeLaunchDrop)
	int cpuOnlyNodes;		// -arccheck: nodes the CPU flood reaches and a GPU-only flood does not (-1: not run)
	int gpuOnlyNodes;		// -arccheck: the reverse
	int rounds;
	long long pairs, arcs, arcsFree, pairsFree;
	double tBuild, tGen, tTrace, tTotal;
	int meshTris[2];
	double gasMB[2];
	// -arccheck
	bool checked;
	long long checkArcs, gpuOnlyBlocked, cpuOnlyBlocked, unexplained;
	double checkMaxDist;
	std::vector<ArcRoundStats> perRound;
	// surf flights (#26): eyes only, traced on the CPU reference on every path
	int surfLips;
	double surfLipLength, surfLipSpacing;
	long long surfLaunches;		// lip sample points with launch speed
	long long surfFlights;		// flights traced: crouched, and lifted crouch_lift (the stand eye)
	long long surfFlightsFree;	// crouched flights free to their end (a fall of arc_max_drop)
	long long surfEyePaths;		// eye paths kept (crouch and stand)
	double tSurf;

	ArcStats() : path( "off" ), reach( 0 ), reachMax( 0 ), allNodes( false ), rimNodes( 0 ), slopeNodes( 0 ), cpuOnlyNodes( -1 ), gpuOnlyNodes( -1 ), rounds( 0 ), pairs( 0 ), arcs( 0 ), arcsFree( 0 ), pairsFree( 0 ), tBuild( 0 ),
		tGen( 0 ), tTrace( 0 ), tTotal( 0 ), checked( false ), checkArcs( 0 ), gpuOnlyBlocked( 0 ), cpuOnlyBlocked( 0 ),
		unexplained( 0 ), checkMaxDist( 0 ), surfLips( 0 ), surfLipLength( 0 ), surfLipSpacing( 0 ), surfLaunches( 0 ),
		surfFlights( 0 ), surfFlightsFree( 0 ), surfEyePaths( 0 ), tSurf( 0 )
	{
		meshTris[0] = meshTris[1] = 0;
		gasMB[0] = gasMB[1] = 0;
	}
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

// A tops_only entity's poses and the pairs of them a rider moves between
// (doors: spawn <-> open; func_movelinear: all pairs; trains: consecutive path
// nodes). 'root' is the entity whose poses these are: a parented entity rides
// its root mover's poses, and its nodes join the root's ride vertices.
struct MoverEntity
{
	int entity;
	int root;
	int poses;
	std::vector< std::pair<int, int> > links;
};

//-----------------------------------------------------------------------------
// Special movement (#13 phase B, bw_special.cpp): water volumes, ladders,
// triggers (teleport, push, catapult), mover rides. Each is a flood vertex
// beyond the nodes (flood id = node count + index) joined to nodes and to each
// other by directed edges; launchers add velocity-driven arcs (bw_arcs.cpp).
//-----------------------------------------------------------------------------
enum SpecialKind
{
	SPECIAL_WATER = 0,		// a water brush grown by the crouch hull: free 3-D movement inside
	SPECIAL_LADDER,			// func_useableladder segment
	SPECIAL_LADDER_BRUSH,	// CONTENTS_LADDER brush (func_ladder) grown by the stand hull + 2u
	SPECIAL_PUSH,			// trigger_push volume (reached through its entries)
	SPECIAL_PUSH_ENTRY,		// a way into a push: a node's part under it, a point inside it, a water / ladder overlap
	SPECIAL_CATAPULT,		// trigger_catapult volume (later engines; best effort)
	SPECIAL_TELEPORT,		// trigger_teleport (touch shapes)
	SPECIAL_ARRIVAL,		// a teleport arrival: destination point, or landmark-shifted touch region
	SPECIAL_RIDE,			// a mover root at one pose
	SPECIAL_KIND_COUNT
};
const char *SpecialKindName( int kind );

// One launch velocity family: vertical speed vz (the start lifted by 'lift', a
// duck), any horizontal velocity within hmax of (bx, by). With apexVz the vz
// that peaks exactly at the landing height is tried too, when it is below vz.
// A push family (inside a continuous trigger_push) flies under gravity gIn up
// to the launcher's pushTop (or half way) and then ballistic, or all the way
// under gIn, with the horizontal velocity within hmax of the segment from 0 to
// (bx, by) (SolvePushArc).
struct LaunchVel
{
	double vz, lift, bx, by, hmax;
	bool apexVz;
	bool push;
	double gIn;
};

// Velocity-driven arcs of a special vertex: from its launch points (the ones
// nearest each landing point, plus the highest) or from the boundary of its
// launch volumes (the point nearest each landing point, plus the nearest on the
// faces facing exitDir), with each velocity family.
struct SpecialLauncher
{
	int vertex;
	std::vector<dvec3> points;
	std::vector<GrownPolytope> volumes;
	dvec3 exitDir;
	bool hasExitDir;
	std::vector<LaunchVel> vel;
	double pushTop;			// push families: the top of the push volume (feet z)
	dvec3 mins, maxs;		// box of every launch point
	double reach;			// XY reach beyond the box (query radius)
	double zTop;			// highest feet z any family's flight reaches
	// when every family carries the same horizontal push velocity b: each flight
	// stays within sweepOwn (own speed times flight time, plus the hull) of the
	// box swept along b * t, t up to sweepT; a tighter pair query than reach
	// (sweepT 0: not used)
	double sweepBx, sweepBy, sweepT, sweepOwn;
};

// A point inside a trigger volume (feet C-space, stand-grown): a free arc to it
// from a reached node reaches 'vertex' (the trigger, or a push's point entry).
struct SpecialTarget
{
	dvec3 p;
	int vertex;
};

struct SpecialVertex
{
	int kind;
	int entity;				// entity index, -1 for world water / ladder brushes
	int pose;				// SPECIAL_RIDE
	char label[64];			// targetname or classname
	std::vector<GrownPolytope> volumes;	// C-space: water volume, push / trigger touch shapes
	dvec3 a, b;				// SPECIAL_LADDER: bottom and top feet points
	int launcher;			// index into SpecialGraph::launchers, -1 for none
};

struct TeleportDest
{
	int entity;
	dvec3 point;
	int node;				// MapSeedPoint of the point (falls straight down), -1 when unmapped
};

struct TeleportReport
{
	int entity;
	int vertex;				// its SPECIAL_TELEPORT vertex
	char target[64], landmark[64];
	bool startDisabled;
	bool players;			// spawnflags let players fire it (clients or everything)
	std::vector<TeleportDest> dests;
};

struct SpecialStats
{
	int count[SPECIAL_KIND_COUNT];
	int reached[SPECIAL_KIND_COUNT];
	int edges;
	int rideLinks;
	int conveyorNodes;
	int vehicleSeeds, vehicleNodes;
	int skippedNoPlayers;		// triggers whose spawnflags exclude players
	int skippedFakeLadders;		// func_useableladder "fake" (spawnflag 1)
	long long launchPairs, launchArcs, launchFree, launchJoined;
	long long targetPairs, targetArcs, targetFree, targetJoined;	// arcs into trigger volumes
	double tBuild, tLaunch, tTargets;

	SpecialStats() { memset( this, 0, sizeof( *this ) ); }
};

struct SpecialGraph
{
	std::vector<SpecialVertex> verts;
	std::vector<SpecialLauncher> launchers;
	std::vector<SpecialTarget> targets;
	std::vector< std::pair<int, int> > edges;	// directed, flood ids
	std::vector<unsigned char> reached;			// per vertex
	std::vector<TeleportReport> teleports;
	std::vector<float> speedBonus;				// per node: conveyor speed added to its arcs (empty: none)
	SpecialStats stats;
};

struct WalkGraph
{
	std::vector<MoverPolytope> movers;
	std::vector<MoverEntity> moverEnts;
	std::vector<WalkNode> nodes;
	std::vector<WalkEdge> edges;
	std::vector<WalkSeed> seeds;
	std::vector<unsigned char> reached;	// per node
	WalkStats stats;
	ArcStats arcs;
	SpecialGraph special;
	bool vehicleFree = false;			// -vehicle-free was given (reported only)
	// The first free arc of every arc edge, special launch join and trigger
	// target join, in the GPU form (its origin carries the chord raise): the
	// eye volume's flight samples (#14, bw_eyes.cpp).
	std::vector<BwArc> freeArcs;
	// Surf flights (#26, bw_arcs.cpp SurfFlights): the free part of every flight
	// off a surf lip, as an eye path (the feet arc raised to the crouch or the
	// stand eye, origin carrying the chord raise), for the eye volume.
	std::vector<BwArc> surfEyes;
};

// Surfing (#24, default on; -nosurf turns it off). A player can carry up to
// max_air_speed * reach_margin onto a slide and ride it along its plane: a
// slide reached other than by surfing has surf moves to every node of its
// connected slide surface, and to the nodes at that surface's edges, at most
// SurfRise above its top; nodes reached by a surf move do not surf again (on a
// slide the player cannot stop to gain speed), and that carries along
// slide-to-slide edges. Slides also launch surf arcs (bw_arcs.cpp), at full
// speed when fresh, else with what their surf cap's energy leaves.
void SetSurf( bool surf );
bool SurfEnabled();
double SurfRise( const PlayerProfile &p );		// vmax^2 / 2g
// Stage 2: drop walkable polygons that lie outside the map (every probe 1u
// above them in a cluster -1 leaf; default on, -keepoffmap turns it off).
void SetCullOffMap( bool cull );
// Stage 2: walkable polygons (nodes, with stand sub-polygons). Multithreaded.
void BuildWalkPolygons( const CollisionWorld &world, const PlayerProfile &profile, WalkGraph &graph );
// Stage 3 (CPU): walk / jump-up / drop edges. Multithreaded.
void BuildWalkEdges( const CollisionWorld &world, const PlayerProfile &profile, WalkGraph &graph );
// The node a feet point maps to: the highest whose XY closure contains it, with
// its plane in [p.z - maxDown, p.z + step] and a free vertical segment to it (an
// origin embedded up to 2u in its floor needs none). -1 when none. Needs
// BuildWalkEdges first. Main thread only.
int MapSeedPoint( const dvec3 &p, double step, double maxDown );

// Special movement (bw_special.cpp). Builds the special vertices and their
// edges for the nodes of 'graph'; needs BuildWalkEdges first.
void BuildSpecialGraph( const CollisionWorld &world, const PlayerProfile &profile, WalkGraph &graph );
// AngleVectors( pushdir ) * speed, speed 0 meaning 100 (triggers.cpp CTriggerPush::Spawn).
dvec3 PushVelocity( const char *pszPushDir, double speed );
// func_tracktrain at a path_track: its origin moves to the path origin + (0, 0, height)
// (trains.cpp CFuncTrackTrain::Find). Returns the translation from the spawn origin.
dvec3 TrackTrainTranslation( const dvec3 &trainOrigin, const dvec3 &pathOrigin, double height );
// func_train at a path_corner: its model centre moves to the corner (trains.cpp
// CFuncTrain::Activate: origin = corner - OBBCenter). The translation from spawn.
dvec3 FuncTrainTranslation( const dvec3 &trainOrigin, const dvec3 &modelCenterLocal, const dvec3 &corner );
// Distance between segment a-b and a planar convex polygon (0 when they meet).
double SegmentPolygonDistance( const dvec3 &a, const dvec3 &b, const DWinding &poly );
// The point of a convex planar polygon (or of a degenerate one's edges) nearest p.
dvec3 ClosestPointOnPolygon( const DWinding &poly, const dvec3 &p );
// Does the convex polygon reach within eps of the polytope (planes n . x <= d + eps)?
bool PolygonTouchesPolytope( const DWinding &poly, const GrownPolytope &q, double eps );

// Seeds from the entity lump (info_player_*, info_ladder_dismount) plus the
// given CLI points. Maps each to a node and floods: a BFS over the walk / jump
// / drop edges, then rounds of target-driven arcs from the newly reached nodes
// (bw_arcs.cpp), each followed by the BFS, until a round reaches nothing.
// Returns false (with a message printed) when there are no seeds or none
// maps: a hard error.
bool FloodWalkGraph( const CollisionWorld &world, const PlayerProfile &profile, const std::vector<dvec3> &cliSeeds,
					 const ArcOptions &arcOpts, WalkGraph &graph );

// The flood as the arc rounds see it (bw_walk.cpp implements it).
class IArcFloodSink
{
public:
	virtual void Reach( int node ) = 0;		// mark reached (queued for the BFS)
	virtual void Propagate() = 0;			// BFS over the CPU edges from everything queued
	virtual const std::vector<int> &Order() const = 0;	// every reached node, in the order reached
	virtual const std::vector<int> &SpecialOrder() const = 0;	// every reached special vertex, in order
	// Surfing (#24). A node reached other than by a surf move is fresh: a surf
	// slide launches at full speed. One reached only by surf moves keeps the
	// energy of the highest surf cap that covered it or that it inherited from a
	// surfed slide it was reached from (SurfCap, -1e300 for none).
	virtual bool Fresh( int node ) const = 0;
	virtual double SurfCap( int node ) const = 0;
	// reached surf slides whose launch got better later (made fresh, or covered
	// by a higher cap), in order; a node can repeat
	virtual const std::vector<int> &Upgraded() const = 0;
};
// The speed a surf slide launches with from height z: vmax when fresh, else
// what is left of the surf cap's energy (no new speed: the no-re-surf rule).
double SurfLaunchSpeed( bool fresh, double cap, double z, double vmax, double gravity );
// Rim nodes: rim[i] = 1 when some part of node i's boundary (in XY) lies in no
// walk-edge neighbour's polygon (expanded by the edge pass's 1e-4u), or only in
// neighbours more than 0.1u lower there (a step down). Flat interior nodes
// launch no arcs: any arc from one crosses into a neighbour at the same height,
// which launches the same family. Sloped interior nodes are handled by the arc
// rounds (bw_arcs.cpp SlopeLaunchDrop). A heuristic, measured against
// -arcallnodes. Needs BuildWalkEdges first.
void ComputeRimNodes( const WalkGraph &graph, std::vector<unsigned char> &rim );
// Surf lips (#26): the stretches of a slide's boundary (in XY) where a surfer
// leaves it into the air. A stretch is covered, not a lip, where a neighbour
// over any edge kind meets the slide (within 0.5u in z) and does not fall
// away below the slide's plane continued 1u past it (more than 0.1u): a
// coplanar or concave crease carries the surfer on. A convex crease (a ramp's
// top edge onto a flat top or over a ridge), a step, a drop, open air or a
// wall is a lip. Only nodes with want[i] set. Needs BuildWalkEdges first.
struct SurfLip
{
	int node;
	dvec3 a, b;			// the stretch, on the node's plane
	double ox, oy;		// outward unit normal of the boundary edge, in XY
};
void ComputeSurfLips( const WalkGraph &graph, const std::vector<unsigned char> &want, std::vector<SurfLip> &lips );
// Arc rounds (bw_arcs.cpp). Appends EDGE_ARC edges to graph.edges.
void RunArcRounds( const CollisionWorld &world, const PlayerProfile &profile, const ArcOptions &opts, WalkGraph &graph,
				   IArcFloodSink &sink );

#endif // BW_WALK_H
