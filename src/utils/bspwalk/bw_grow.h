//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: convex shapes, double-precision windings, and the exact Minkowski
//          growth of a convex by the (reflected) player box.
//
//          The player is an axis-aligned box B that never rotates. For a
//          feet-origin point p the box is p + B with
//              B = [-hw,hw] x [-hw,hw] x [0,h]
//          so p collides with convex P iff p is in P (+) (-B). GrowConvex()
//          builds that sum exactly as a plane list plus vertex/face lists.
//
//=============================================================================//

#ifndef BW_GROW_H
#define BW_GROW_H
#pragma once

#include <vector>
#include "bw_math.h"

// A plane with the inside on the back: n . x <= d.
struct DPlane
{
	dvec3 n;
	double d;
};

typedef std::vector<dvec3> DWinding;

//-----------------------------------------------------------------------------
// Player hull (feet origin). Defaults are HL2's; profile files arrive in #13.
//-----------------------------------------------------------------------------
struct PlayerHull
{
	double halfWidth;	// box spans [-halfWidth, halfWidth] in X and Y
	double height;		// box spans [0, height] in Z

	// Support function of -B: max over c in -B of n . c.
	double ReflectedSupport( const dvec3 &n ) const
	{
		return halfWidth * ( fabs( n.x ) + fabs( n.y ) ) + height * ( n.z < 0.0 ? -n.z : 0.0 );
	}
	// The 8 corners of -B.
	void ReflectedCorners( dvec3 corners[8] ) const;
};

// Game profile (bspwalk_profiles.txt, bw_profile.cpp). The defaults are the
// built-in hl2 profile (SDK 2013 HL2_DLL: gamerules.cpp view vectors,
// gamemovement.cpp:2437 jump 160 at gravity 600, hl2_player.cpp sprint 320).
struct PlayerProfile
{
	char name[32];
	char source[16];		// "file" or "default" (built in)
	double halfWidth;
	double standHeight;
	double crouchHeight;
	double eyeStand, eyeCrouch;
	double eyeDead;			// a dead player's view height above the floor (VEC_DEAD_VIEWHEIGHT)
	double stepHeight;		// sv_stepsize: walk edges up to this rise
	double gravity;			// sv_gravity
	double jumpVelocity;	// vz of a ground jump
	double crouchLift;		// feet rise of an in-air duck (stand - crouch height)
	double runSpeed;		// normal ground speed
	double maxGroundSpeed;	// fastest ground speed (sprint / fastest class)
	double maxAirSpeed;		// bhop / ABH / air-strafe ceiling (2x max ground speed where uncapped): -exploits only
	double intendedAirSpeed;	// fastest horizontal air speed of intended movement (a sprint jump plus one jump's air strafe)
	double reachMargin;		// over-reach multiplier on speeds (hedges constant error only)
	int airJumps;			// extra jumps in the air (tf scout: 1)
	double arcMaxDrop;		// arcs reach out as far as a fall this deep allows (the fatal fall height)
	double minLedgeWidth;	// a walk node narrower than this is a pixel walk (bw_walk.cpp MarkNarrowNodes); -exploits keeps it
	bool exploits;			// -exploits (#87): bhop / ABH speed, surfing and pixel walks

	// Derived (Derive()).
	double airSpeed;		// the horizontal air speed in force: max_air_speed with exploits, else intended_air_speed
	double jumpApex;		// vz^2 / 2g
	double maxJumpRise;		// highest feet rise of a vertical jump: apex * (1 + airJumps) + crouch lift
	double arcAirtime;		// longest flight back to the launch height (duck-jump, air jumps)
	double arcReach;		// R(0): airSpeed * reach_margin * arcAirtime + 2 * halfWidth
	double arcReachMax;		// R(arcMaxDrop)

	PlayerProfile();
	void Derive();
	// R(drop): the XY reach of an arc landing 'drop' below its launch height
	// (clamped to [0, arcMaxDrop]): the air speed in force times the flight time of the
	// highest jump (duck lift, air jumps) down to that height, plus the hull width.
	double ArcReachAtDrop( double drop ) const;

	PlayerHull Stand() const { PlayerHull h; h.halfWidth = halfWidth; h.height = standHeight; return h; }
	PlayerHull Crouch() const { PlayerHull h; h.halfWidth = halfWidth; h.height = crouchHeight; return h; }
};

// Loads the profile: 'pszName' (NULL = pick from the game directory name, with
// fallback hl2) from bspwalk_profiles.txt next to the exe. A missing file gives
// the built-in hl2 profile with a warning. False (message printed) for an
// unknown -profile name or a malformed entry. 'exploits' (-exploits) puts
// max_air_speed in force instead of intended_air_speed.
bool LoadPlayerProfile( const char *pszName, const char *pszGameDir, bool exploits, PlayerProfile &out );

//-----------------------------------------------------------------------------
// A source convex: vertices, polygon faces (CCW seen from outside for closed
// solids; a flat shape has one face), outward face normals, and unique edges.
//-----------------------------------------------------------------------------
struct ConvexShape
{
	std::vector<dvec3> verts;
	std::vector< std::vector<int> > faces;
	std::vector<dvec3> faceNormals;			// one per face
	std::vector< std::pair<int, int> > edges;
	dvec3 mins, maxs;
	bool flat;								// single polygon (displacement triangle)

	ConvexShape() : flat( false ) {}

	// Weld-insert a vertex, returns its index.
	int AddVert( const dvec3 &p );
	// Rebuilds edges and bounds from verts/faces.
	void Finish();
	bool IsValid() const { return verts.size() >= 3 && !faces.empty(); }
};

// The grown polytope: planes[i] is the plane of faces[i].
struct GrownPolytope
{
	std::vector<DPlane> planes;
	std::vector<dvec3> verts;
	std::vector< std::vector<int> > faces;	// CCW seen from outside
	dvec3 mins, maxs;

	// GrowConvex leaves a failed grow empty; this is what dump and counts test.
	bool IsValid() const { return planes.size() >= 4 && verts.size() >= 4 && faces.size() == planes.size(); }
};

//-----------------------------------------------------------------------------
// Double-precision winding helpers (a port of the few polylib.cpp functions
// bspwalk needs; polylib itself is float).
//-----------------------------------------------------------------------------
void DBaseWindingForPlane( const dvec3 &normal, double dist, double halfSize, DWinding &out );
// Keeps the part of the winding on the front of (normal, dist). Empties it if nothing is left.
void DChopWindingInPlace( DWinding &w, const dvec3 &normal, double dist, double epsilon );
double DWindingArea( const DWinding &w );

//-----------------------------------------------------------------------------
// Builders
//-----------------------------------------------------------------------------
// Intersection of half-spaces n . x <= d (a brush). Returns false if empty.
bool ConvexFromPlanes( const DPlane *planes, int numPlanes, ConvexShape &out );
// Convex hull of a point set (may shrink by <= 1e-6u). False if the points are flat.
bool ConvexFromPoints( const dvec3 *pts, int numPts, ConvexShape &out );
// Closed convex given as a triangle soup (vphysics convex): the hull of its points.
bool ConvexFromTriangles( const dvec3 *triVerts, int numTris, ConvexShape &out );
// Closed convex given by its 8 box corners in model space, transformed.
void ConvexFromBox( const dvec3 &mins, const dvec3 &maxs, const DTransform &xf, ConvexShape &out );
// A single flat triangle (displacement collision triangle).
bool ConvexFromTriangle( const dvec3 &a, const dvec3 &b, const dvec3 &c, ConvexShape &out );

//-----------------------------------------------------------------------------
// Exact Minkowski sum src (+) (-B). Returns false if the result is degenerate.
//-----------------------------------------------------------------------------
bool GrowConvex( const ConvexShape &src, const PlayerHull &hull, GrownPolytope &out );
// Prints every candidate decision of GrowConvex (-growdebug, single-threaded use only).
extern bool g_bGrowDebug;

// Independent check of a grown polytope against the exact sum point set
// {v + c : v in src, c in corners(-B)}: every plane must support that set,
// every face vertex must be one of those points, and every corner of the plane
// intersection must lie within 6e-8 * |coord| + 1e-5 (the input's float
// rounding) of their hull, which bounds the overgrowth. Returns false and fills pszWhy on failure.
bool VerifyGrown( const ConvexShape &src, const PlayerHull &hull, const GrownPolytope &poly,
				  char *pszWhy, int whyLen );

// Signed volume of a closed polytope from its CCW faces.
double PolytopeVolume( const std::vector<dvec3> &verts, const std::vector< std::vector<int> > &faces );

// Known-answer checks of the grow routine and the Stage 2 polygon subtraction
// (-selftest, bw_selftest.cpp). Returns number of failures.
int RunGrowSelfTest();

#endif // BW_GROW_H
