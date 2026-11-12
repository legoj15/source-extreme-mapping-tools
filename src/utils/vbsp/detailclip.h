//========= detailclip: drop detail props that clip into the map ==============
//
// Issue #92. VBSP places detail props (grass, rocks, ...) from the
// %detailtype of a face's material and knows nothing about what else is
// there: they come out inside func_detail structures, in walls, and in static
// props. With -detailclip, PlaceDetail asks this module about each candidate
// after all of its random draws, so a run with the flag drops props from the
// run without it and never moves another one. Off by default.
//
// A candidate is dropped when
//   - a point 1 unit above the spawn point (along the surface normal) lies
//     inside a visible solid world or func_detail brush ("buried"), or
//   - its bounding box, shrunk by -detailclip_shrink (horizontally about its
//     centre line, vertically toward its base, never below the spawn plane)
//     and cut off below the tangent plane of the spawn surface, intersects such a
//     brush ("brush"), or intersects a triangle of the LOD0 mesh of a static
//     prop ("static prop"). A box wholly below that plane (a world-upright box
//     on a ceiling, or a prop sunk under its spawn point) is tested uncut.
//   - or it intersects a triangle of a displacement other than the one it was
//     placed on and those sewn to it along an edge ("displacement", see
//     docs/detailclip.md): the full-resolution displaced mesh, tested
//     like a static prop mesh. Displacement brushes are left out of the brush
//     set (their flat base is not the visible surface), and there is no buried
//     test for displacements.
// Hand-placed prop_detail / prop_detail_sprite entities are never tested.
//=============================================================================
#ifndef DETAILCLIP_H
#define DETAILCLIP_H
#pragma once

#include "mathlib/vector.h"

extern bool g_bDetailClip;        // -detailclip
extern float g_flDetailClipShrink; // -detailclip_shrink, default 0.5

// The extent of one detail prop. mins/maxs are in the prop's own space: z up,
// the placement point at the origin, framed by the angles written to the lump.
// yawAgnostic props (screen-aligned orientations, whose angles the engine
// recomputes per view) are tested as a world-upright box with a square
// footprint wide enough for any yaw.
struct DetailClipShape_t {
  bool boundsKnown;
  bool yawAgnostic;
  Vector mins, maxs;
};

enum DetailClipVerdict_t {
  DETAILCLIP_KEEP = 0,
  DETAILCLIP_BURIED,
  DETAILCLIP_BRUSH,
  DETAILCLIP_STATICPROP,
  DETAILCLIP_DISPLACEMENT,
};

// Builds the brush, static prop and displacement lookups. Call once
// EmitStaticProps has run, before the first DetailClip_Test.
void DetailClip_Begin();

// Prints the summary line and frees the lookups.
void DetailClip_End();

// The bounds of a detail model: its default-bodygroup LOD0 mesh when the VVD
// and VTX load, else the studio header's hull, else its view bounding box.
// False when the model can be read but none of them has an extent (the prop is
// then only given the buried test). nOrientation is the detail prop's
// orientation (DetailPropOrientation_t); anything but "normal" turns it into a
// yaw-agnostic shape.
bool DetailClip_ModelShape(const char *pModelName, int nOrientation,
                           DetailClipShape_t &shape);

// Tests one candidate placed at pt on a surface with the given unit normal,
// with the angles that will be written to the lump. nDispInfo is the index into
// mapdispinfo[] of the displacement the candidate was placed on (that one is not
// tested against), or -1 for a candidate on a brush face.
DetailClipVerdict_t DetailClip_Test(const Vector &pt, const Vector &normal,
                                    const QAngle &angles,
                                    const DetailClipShape_t &shape,
                                    int nDispInfo);

#endif // DETAILCLIP_H
