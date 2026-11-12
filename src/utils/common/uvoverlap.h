//========= UV overlap detector shared by VBSP and VRAD (issue #56) ===========
//
// One exact measure of how much a model's UV0 layout overlaps itself, used by
// the VBSP rebake route (proplm_rebake.cpp: does the model need a rebake, can
// it be rebaked, did it work) and by VRAD's static prop lighting
// (vradstaticprops.cpp: the per-model overlap warning and report).
//
// It lives in utils/common because that is the only source tree both tools
// compile. It reads no files: each tool keeps its own model loader (VBSP the
// bounds-checked proplm_shared.h readers, VRAD the SDK's studio accessors) and
// hands over plain vertex and triangle arrays.
//
// Rules:
// * Islands: triangles that share a vertex index are one island. Across a hard
//   edge (split vertices) two triangles join only when they share a whole edge
//   whose two endpoints match bit for bit in position, UV and tangent
//   handedness (sign). Mirrored halves differ in handedness, so they stay apart.
// * Raster: a grid of resX x resY cells per UV unit; a cell belongs to a
//   triangle when its center is inside it. A center exactly on an edge goes to
//   exactly one side (a top-left style tie rule over edge functions that are
//   evaluated with the edge's endpoints in a fixed order, so both triangles of
//   a shared edge compute the same value). Two triangles that meet along an
//   edge or at a corner therefore never share a cell; a triangle folded back
//   over its neighbour does. No neighbour is exempted beyond that.
//   A triangle with zero UV area covers nothing.
// * Folding: cells are folded into one 0..1 tile (the lightmap wraps). A cell
//   is overlapped when two claims on it come from different triangles, or from
//   one triangle at two places (a triangle wider or taller than one tile
//   overlaps itself).
// * Categories, per island: fold (two of its triangles on the same unfolded
//   cell: a fold or a mirror inside the island; moving islands cannot fix it),
//   repeat (the island on one cell at two unfolded places: it is wider than a
//   tile, or one of its triangles is), other (another island on the cell).
//=============================================================================
#ifndef UVOVERLAP_H
#define UVOVERLAP_H
#pragma once

#include "tier0/platform.h"
#include "tier1/utlvector.h"

struct UvOverlapVert_t {
  float pos[3];
  float uv[2];
  int sign; // tangent handedness, +1 or -1 (+1 when the model has none)
};

struct UvOverlapTri_t {
  int vert[3]; // indices into the vertex array
};

struct UvOverlapIsland_t {
  int tris;         // triangles in the island
  int cells;        // distinct folded cells its triangles cover
  int foldCells;    // cells two of its own triangles cover at one unfolded place
  int repeatCells;  // cells it covers at two unfolded places
  int otherCells;   // cells another island covers too
  int spanningTris; // triangles wider or taller than one UV tile
  // Unfolded: the island's own area, where repeats are separate places.
  int unfoldedCells; // distinct (cell, tile) places its triangles cover
  int foldClaims;    // claims past the first at one place (k claims count k-1)
  double mn[2], mx[2]; // UV bounds of its triangles' corners
};

struct UvOverlapResult_t {
  int resX, resY;     // cells per UV unit
  int cells;          // distinct folded cells covered by anything
  int overlapCells;   // of those, cells claimed twice (any category)
  int foldCells;      // cells with a fold claim pair (any island)
  int repeatCells;    // cells with a repeat claim pair
  int otherCells;     // cells claimed by two or more islands
  int movableCells;   // cells with a repeat or other pair (what moving islands fixes)
  int spanningTris;
  int degenerateTris; // zero UV area, not rasterized
  double fraction;    // overlapCells / cells (0 when nothing is covered)
  CUtlVector<int> islandOfTri;
  CUtlVector<UvOverlapIsland_t> islands;
};

// Fills islandOfTri (one entry per triangle) and returns the island count.
// Islands are numbered in the order of their first triangle.
int UvOverlap_BuildIslands(const UvOverlapVert_t *pVerts, int nVerts,
                           const UvOverlapTri_t *pTris, int nTris,
                           CUtlVector<int> &islandOfTri);

// Builds the islands (UvOverlap_BuildIslands) and measures. maxRepeats bounds
// the UV extent of one triangle, in tiles per axis, so a corrupt or absurdly
// tiled triangle cannot make the raster unbounded; such a triangle fails the
// measure (false, reason in pErr). resX/resY are cells per UV unit (1..8192).
bool UvOverlap_Measure(const UvOverlapVert_t *pVerts, int nVerts,
                       const UvOverlapTri_t *pTris, int nTris, int resX,
                       int resY, int maxRepeats, UvOverlapResult_t &result,
                       char *pErr, int errLen);

// The same measure over islands the caller already has (pIslandOfTri, one
// entry per triangle, each in 0..nIslands-1), so a layout that moved islands
// is measured with the islands it was built from.
bool UvOverlap_MeasureIslands(const UvOverlapVert_t *pVerts, int nVerts,
                              const UvOverlapTri_t *pTris, int nTris,
                              const int *pIslandOfTri, int nIslands, int resX,
                              int resY, int maxRepeats,
                              UvOverlapResult_t &result, char *pErr,
                              int errLen);

// Prints the result: one summary line, one category line, and one line per
// island that overlaps (up to maxIslands of them, worst first). Every line
// starts with "uvoverlap: <name>: ".
void UvOverlap_Print(const char *pName, const UvOverlapResult_t &result,
                     int maxIslands);

// Reads a plain-text mesh: "v x y z u v [sign]" and "f a b c" lines (0-based
// vertex indices), '#' comments. Used by the VRAD -uvoverlap diagnostic.
bool UvOverlap_ReadTextMesh(const char *pPath,
                            CUtlVector<UvOverlapVert_t> &verts,
                            CUtlVector<UvOverlapTri_t> &tris, char *pErr,
                            int errLen);

#endif // UVOVERLAP_H
