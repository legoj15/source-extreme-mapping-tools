//========= Copyright Valve Corporation, All rights reserved. ============//
//
// Purpose:
//
// $NoKeywords: $
//
//=============================================================================//
// faces.c

#include "materialpatch.h"
#include "mstristrip.h"
#include "tier1/strtools.h"
#include "utilmatlib.h"
#include "utlvector.h"
#include "vbsp.h"
#include <float.h>

/*

  some faces will be removed before saving, but still form nodes:

  the insides of sky volumes
  meeting planes of different water current volumes

*/

// undefine for dumb linear searches
#define USE_HASHING

#define INTEGRAL_EPSILON 0.01
#define POINT_EPSILON 0.1
#define OFF_EPSILON 0.25

int c_merge;
int c_subdivide;

int c_totalverts;
int c_uniqueverts;
int c_degenerate;
int c_tjunctions;
int c_faceoverflows;
int c_facecollapse;
int c_badstartverts;

// How the t-junction sewing of the faces came out (see "T-junction sewing"
// below).  Reset per FixTjuncs() call; the world model's totals are copied to
// the g_tjunc* globals (declared in vbsp.h, read by -countverts in vbsp.cpp).
static int c_tjuncRootClean;    // bad stock start, but a fan from it sews anyway
static int c_tjuncRerooted;     // rotated to another vertex whose fan sews
static int c_tjuncPrimSewn;     // sewn by a primitive
static int c_tjuncUnsewnPrim;   // primitive emitted but it still leaves a seam
static int c_tjuncUnsewnBudget; // primitive dropped: out of primitive space
static int c_tjuncUnsewnOther;  // no fan and no usable primitive sews it
static int c_tjuncStockKept;    // water/fog face kept as stock drew it, seam left
static int c_tjuncZeroArea;     // zero-area winding: nothing is drawn, nothing to sew
static int c_tjuncBrushUnsewn;  // brush entity faces no fan sews (no prims there)
int g_tjuncRootClean;
int g_tjuncRerooted;
int g_tjuncPrimSewn;
int g_tjuncUnsewn;
int g_tjuncBrushUnsewn;

#define MAX_SUPERVERTS 512
int superverts[MAX_SUPERVERTS];
int numsuperverts;

face_t *edgefaces[MAX_MAP_EDGES][2];
int firstmodeledge = 1;
int firstmodelface;

int c_tryedges;

Vector edge_dir;
Vector edge_start;
vec_t edge_len;

int num_edge_verts;
int edge_verts[MAX_MAP_VERTS];

float g_maxLightmapDimension = 32;

face_t *NewFaceFromFace(face_t *f);

// Used to speed up GetEdge2(). Holds a list of edges connected to each vert.
CUtlVector<int> g_VertEdgeList[MAX_MAP_VERTS];

//===========================================================================

typedef struct hashvert_s {
  struct hashvert_s *next;
  int num;
} hashvert_t;

#define HASH_BITS 7
#define HASH_SIZE (COORD_EXTENT >> HASH_BITS)

int vertexchain[MAX_MAP_VERTS];       // the next vertex in a hash chain
int hashverts[HASH_SIZE * HASH_SIZE]; // a vertex number, or 0 for no verts

// face_t		*edgefaces[MAX_MAP_EDGES][2];

//============================================================================

unsigned HashVec(Vector &vec) {
  int x, y;

  x = (MAX_COORD_INTEGER + (int)(vec[0] + 0.5)) >> HASH_BITS;
  y = (MAX_COORD_INTEGER + (int)(vec[1] + 0.5)) >> HASH_BITS;

  if (x < 0 || x >= HASH_SIZE || y < 0 || y >= HASH_SIZE)
    Error("HashVec: point outside valid range");

  return y * HASH_SIZE + x;
}

#ifdef USE_HASHING
/*
=============
GetVertex

Uses hashing
=============
*/
int GetVertexnum(Vector &in) {
  int h;
  int i;
  Vector vert;
  int vnum;

  c_totalverts++;

  for (i = 0; i < 3; i++) {
    if (fabs(in[i] - (int)(in[i] + 0.5)) < INTEGRAL_EPSILON)
      vert[i] = (int)(in[i] + 0.5);
    else
      vert[i] = in[i];
  }

  h = HashVec(vert);

  for (vnum = hashverts[h]; vnum; vnum = vertexchain[vnum]) {
    Vector &p = dvertexes[vnum].point;
    if (fabs(p[0] - vert[0]) < POINT_EPSILON &&
        fabs(p[1] - vert[1]) < POINT_EPSILON &&
        fabs(p[2] - vert[2]) < POINT_EPSILON)
      return vnum;
  }

  // emit a vertex
  if (!g_bCountVertsOnly && numvertexes >= 65536)
    Error("Too many unique verts for BSP format, max = 65536\n");
  if (numvertexes == MAX_MAP_VERTS)
    Error("Too many unique verts, max = %d (map has too much brush geometry)\n",
          MAX_MAP_VERTS);

  dvertexes[numvertexes].point[0] = vert[0];
  dvertexes[numvertexes].point[1] = vert[1];
  dvertexes[numvertexes].point[2] = vert[2];

  vertexchain[numvertexes] = hashverts[h];
  hashverts[h] = numvertexes;

  c_uniqueverts++;

  numvertexes++;

  return numvertexes - 1;
}
#else
/*
==================
GetVertexnum

Dumb linear search
==================
*/
int GetVertexnum(Vector &v) {
  int i, j;
  dvertex_t *dv;
  vec_t d;

  c_totalverts++;

  // make really close values exactly integral
  for (i = 0; i < 3; i++) {
    if (fabs(v[i] - (int)(v[i] + 0.5)) < INTEGRAL_EPSILON)
      v[i] = (int)(v[i] + 0.5);
    if (v[i] < MIN_COORD_INTEGER || v[i] > MAX_COORD_INTEGER)
      Error("GetVertexnum: outside world, vertex %.1f %.1f %.1f", v.x, v.y,
            v.z);
  }

  // search for an existing vertex match
  for (i = 0, dv = dvertexes; i < numvertexes; i++, dv++) {
    for (j = 0; j < 3; j++) {
      d = v[j] - dv->point[j];
      if (d > POINT_EPSILON || d < -POINT_EPSILON)
        break;
    }
    if (j == 3)
      return i; // a match
  }

  // new point
  if (!g_bCountVertsOnly && numvertexes >= 65536)
    Error("Too many unique verts for BSP format, max = 65536\n");
  if (numvertexes == MAX_MAP_VERTS)
    Error("Too many unique verts, max = %d (map has too much brush geometry)\n",
          MAX_MAP_VERTS);
  VectorCopy(v, dv->point);
  numvertexes++;
  c_uniqueverts++;

  return numvertexes - 1;
}
#endif

/*
==================
FaceFromSuperverts

The faces vertexes have beeb added to the superverts[] array,
and there may be more there than can be held in a face (MAXEDGES).

If less, the faces vertexnums[] will be filled in, otherwise
face will reference a tree of split[] faces until all of the
vertexnums can be added.

superverts[base] will become face->vertexnums[0], and the others
will be circularly filled in.
==================
*/

// Most fragments MAX_SUPERVERTS superverts can be cut into.
#define MAX_RING_FRAGMENTS (MAX_SUPERVERTS / (MAXEDGES - 2) + 2)

// Partition the ring of numsuperverts vertices, starting at ring position
// `base`, into polygons of at most MAXEDGES vertices.  Each fragment is a list
// of ring positions in winding order; a fragment's last vertex closes back to
// its first along a chord, except where it follows the ring.  Cutting off the
// first MAXEDGES vertices leaves the rest of the ring closed by the chord
// from the cut's last vertex back to its first, so every later fragment keeps
// that first vertex as its own last one.  (Stock VBSP indexed the leftover as
// a contiguous run of the ring, which is only right for the first cut: from
// the third fragment on it picked up a stray vertex.)
// Returns the fragment count.
static int SplitRing(int base, int fragPos[][MAXEDGES], int fragLen[]) {
  int ring[MAX_SUPERVERTS];
  int remaining = numsuperverts;
  int nf = 0;

  for (int i = 0; i < remaining; i++)
    ring[i] = (i + base) % numsuperverts;

  while (remaining > MAXEDGES) {
    memcpy(fragPos[nf], ring, MAXEDGES * sizeof(int));
    fragLen[nf++] = MAXEDGES;

    int first = ring[0];
    memmove(ring, ring + (MAXEDGES - 1),
            (remaining - (MAXEDGES - 1)) * sizeof(int));
    ring[remaining - (MAXEDGES - 1)] = first;
    remaining -= (MAXEDGES - 2);
  }

  memcpy(fragPos[nf], ring, remaining * sizeof(int));
  fragLen[nf++] = remaining;
  return nf;
}

void FaceFromSuperverts(face_t **pListHead, face_t *f, int base) {
  int fragPos[MAX_RING_FRAGMENTS][MAXEDGES];
  int fragLen[MAX_RING_FRAGMENTS];
  face_t *newf;
  int i;

  int numFragments = SplitRing(base, fragPos, fragLen);

  // all but the last fragment get a face of their own, because of vertex
  // overload
  for (int frag = 0; frag < numFragments - 1; frag++) {
    c_faceoverflows++;

    newf = NewFaceFromFace(f);
    f->split[0] = newf;

    newf->next = *pListHead;
    *pListHead = newf;

    newf->numpoints = MAXEDGES;
    for (i = 0; i < MAXEDGES; i++)
      newf->vertexnums[i] = superverts[fragPos[frag][i]];

    f->split[1] = NewFaceFromFace(f);
    f = f->split[1];

    f->next = *pListHead;
    *pListHead = f;
  }

  // copy the vertexes back to the face
  int last = numFragments - 1;
  f->numpoints = fragLen[last];
  for (i = 0; i < fragLen[last]; i++)
    f->vertexnums[i] = superverts[fragPos[last][i]];
}

/*
==================
EmitFaceVertexes
==================
*/
void EmitFaceVertexes(face_t **pListHead, face_t *f) {
  winding_t *w;
  int i;

  if (f->merged || f->split[0] || f->split[1])
    return;

  w = f->w;
  for (i = 0; i < w->numpoints; i++) {
    if (noweld) { // make every point unique
      if (!g_bCountVertsOnly && numvertexes >= 65536)
        Error("Too many unique verts for BSP format, max = 65536\n");
      if (numvertexes == MAX_MAP_VERTS)
        Error("Too many unique verts, max = %d (map has too much brush "
              "geometry)\n",
              MAX_MAP_VERTS);
      superverts[i] = numvertexes;
      VectorCopy(w->p[i], dvertexes[numvertexes].point);
      numvertexes++;
      c_uniqueverts++;
      c_totalverts++;
    } else
      superverts[i] = GetVertexnum(w->p[i]);
  }
  numsuperverts = w->numpoints;

  // this may fragment the face if > MAXEDGES
  FaceFromSuperverts(pListHead, f, 0);
}

/*
==================
EmitNodeFaceVertexes_r
==================
*/
void EmitNodeFaceVertexes_r(node_t *node) {
  int i;
  face_t *f;

  if (node->planenum == PLANENUM_LEAF) {
    // leaf faces are emitted in second pass
    return;
  }

  for (f = node->faces; f; f = f->next) {
    EmitFaceVertexes(&node->faces, f);
  }

  for (i = 0; i < 2; i++) {
    EmitNodeFaceVertexes_r(node->children[i]);
  }
}

void EmitLeafFaceVertexes(face_t **ppLeafFaceList) {
  face_t *f = *ppLeafFaceList;

  while (f) {
    EmitFaceVertexes(ppLeafFaceList, f);
    f = f->next;
  }
}

#ifdef USE_HASHING
/*
==========
FindEdgeVerts

Uses the hash tables to cut down to a small number
==========
*/
void FindEdgeVerts(Vector &v1, Vector &v2) {
  int x1, x2, y1, y2, t;
  int x, y;
  int vnum;

#if 0
{
	int		i;
	num_edge_verts = numvertexes-1;
	for (i=0 ; i<numvertexes-1 ; i++)
		edge_verts[i] = i+1;
}
#endif

  x1 = (MAX_COORD_INTEGER + (int)(v1[0] + 0.5)) >> HASH_BITS;
  y1 = (MAX_COORD_INTEGER + (int)(v1[1] + 0.5)) >> HASH_BITS;
  x2 = (MAX_COORD_INTEGER + (int)(v2[0] + 0.5)) >> HASH_BITS;
  y2 = (MAX_COORD_INTEGER + (int)(v2[1] + 0.5)) >> HASH_BITS;

  if (x1 > x2) {
    t = x1;
    x1 = x2;
    x2 = t;
  }
  if (y1 > y2) {
    t = y1;
    y1 = y2;
    y2 = t;
  }
#if 0
	x1--;
	x2++;
	y1--;
	y2++;
	if (x1 < 0)
		x1 = 0;
	if (x2 >= HASH_SIZE)
		x2 = HASH_SIZE;
	if (y1 < 0)
		y1 = 0;
	if (y2 >= HASH_SIZE)
		y2 = HASH_SIZE;
#endif
  num_edge_verts = 0;
  for (x = x1; x <= x2; x++) {
    for (y = y1; y <= y2; y++) {
      for (vnum = hashverts[y * HASH_SIZE + x]; vnum;
           vnum = vertexchain[vnum]) {
        edge_verts[num_edge_verts++] = vnum;
      }
    }
  }
}

#else
/*
==========
FindEdgeVerts

Forced a dumb check of everything
==========
*/
void FindEdgeVerts(Vector &v1, Vector &v2) {
  int i;

  num_edge_verts = numvertexes - 1;
  for (i = 0; i < num_edge_verts; i++)
    edge_verts[i] = i + 1;
}
#endif

/*
==========
TestEdge

Can be recursively reentered
==========
*/
void TestEdge(vec_t start, vec_t end, int p1, int p2, int startvert) {
  int j, k;
  vec_t dist;
  Vector delta;
  Vector exact;
  Vector off;
  vec_t error;
  Vector p;

  if (p1 == p2) {
    c_degenerate++;
    return; // degenerate edge
  }

  for (k = startvert; k < num_edge_verts; k++) {
    j = edge_verts[k];
    if (j == p1 || j == p2)
      continue;

    VectorCopy(dvertexes[j].point, p);

    VectorSubtract(p, edge_start, delta);
    dist = DotProduct(delta, edge_dir);
    if (dist <= start || dist >= end)
      continue; // off an end
    VectorMA(edge_start, dist, edge_dir, exact);
    VectorSubtract(p, exact, off);
    error = off.Length();

    if (error > OFF_EPSILON)
      continue; // not on the edge

    // break the edge
    c_tjunctions++;
    TestEdge(start, dist, p1, j, k + 1);
    TestEdge(dist, end, j, p2, k + 1);
    return;
  }

  // the edge p1 to p2 is now free of tjunctions
  if (numsuperverts >= MAX_SUPERVERTS)
    Error("Edge with too many vertices due to t-junctions.  Max %d verts along "
          "an edge!\n",
          MAX_SUPERVERTS);
  superverts[numsuperverts] = p1;
  numsuperverts++;
}

// stores the edges that each vert is part of
struct face_vert_table_t {
  face_vert_table_t() {
    edge0 = -1;
    edge1 = -1;
  }

  void AddEdge(int edge) {
    if (edge0 == -1) {
      edge0 = edge;
    } else {
      // can only have two edges
      Assert(edge1 == -1);
      edge1 = edge;
    }
  }

  bool HasEdge(int edge) const {
    if (edge >= 0) {
      if (edge0 == edge || edge1 == edge)
        return true;
    }
    return false;
  }

  int edge0;
  int edge1;
};

// if these two verts share an edge, they must be collinear
bool IsDiagonal(const face_vert_table_t &v0, const face_vert_table_t &v1) {
  if (v1.HasEdge(v0.edge0) || v1.HasEdge(v0.edge1))
    return false;

  return true;
}

void Triangulate_r(CUtlVector<int> &out, const CUtlVector<int> &inIndices,
                   const CUtlVector<face_vert_table_t> &poly) {
  Assert(inIndices.Count() > 2);

  // one triangle left, return
  if (inIndices.Count() == 3) {
    for (int i = 0; i < inIndices.Count(); i++) {
      out.AddToTail(inIndices[i]);
    }
    return;
  }

  // check each pair of verts and see if they are diagonal (not on a shared
  // edge) if so, split & recurse
  for (int i = 0; i < inIndices.Count(); i++) {
    int count = inIndices.Count();

    // i + count is myself, i + count-1 is previous, so we need to stop at
    // i+count-2
    for (int j = 2; j < count - 1; j++) {
      // if these two form a diagonal, split the poly along
      // the diagonal and triangulate the two sub-polys
      int index = inIndices[i];
      int nextArray = (i + j) % count;
      int nextIndex = inIndices[nextArray];
      if (IsDiagonal(poly[index], poly[nextIndex])) {
        // add the poly up to the diagonal
        CUtlVector<int> in1;
        for (int k = i; k != nextArray; k = (k + 1) % count) {
          in1.AddToTail(inIndices[k]);
        }
        in1.AddToTail(nextIndex);

        // add the rest of the poly starting with the diagonal
        CUtlVector<int> in2;
        in2.AddToTail(index);
        for (int l = nextArray; l != i; l = (l + 1) % count) {
          in2.AddToTail(inIndices[l]);
        }

        // triangulate the sub-polys
        Triangulate_r(out, in1, poly);
        Triangulate_r(out, in2, poly);
        return;
      }
    }
  }

  // didn't find a diagonal
  Assert(0);
}

//-----------------------------------------------------------------------------
// T-junction sewing
//
// The engine draws a face with no primitive as a triangle fan from its first
// vertex.  Every t-junction vertex inserted into a face by TestEdge() has to
// be the corner of a real (non-degenerate) triangle, otherwise the face's edge
// skips over it and the neighbouring face that does own that vertex cracks
// against it.  A fan from vertex r puts a t-junction vertex on a zero-area
// triangle exactly when r lies on the line of the edge that vertex was cut
// from and the vertex is not adjacent to r.
//
// So each face is sewn, in order of cost, by:
//  1. the stock start: an original vertex with no t-junctions on either side;
//  2. rotating the winding to start at any other vertex whose fan has no
//     such triangle (usually a t-junction vertex that is the only one on its
//     edge) - no primitive needed;
//  3. a re-triangulated primitive (PRIM_TRILIST) for faces no fan can sew.
// Only 3. costs primitive space, which is a hard format limit (dprimitive_t's
// firstIndex/indexCount are 16-bit: 65536 indices, 32768 primitives).  If the
// primitives that are wanted do not fit, the ones that recover the most seams
// per index are kept, the rest of the faces are fan-drawn from their least bad
// root, and the loss is reported.
//
// A face with more than MAXEDGES vertices is split into several faces, and
// each of those is sewn on its own (its chords included: a chord is a side of
// the fragment like any other).
//
// Faces that touch water keep the stock behaviour (see FixFaceEdges).
//
// A fan or primitive is also judged by how far its triangles fold back over the
// polygon (a triangle with the wrong winding draws outside the face).  Folds up
// to SEW_FOLD_FREE are noise; deeper ones are avoided when a rotation, which
// costs nothing, or a triangulation that sews something anyway does better;
// SEW_FLIP_HEIGHT is the hard limit.
//-----------------------------------------------------------------------------

// A polygon side is unsewn when the third corner of the triangle that carries
// it lies this close to the side's line: the triangle has no area, so the side
// is never drawn.  (Real slivers - a vertex a few hundredths off a long edge -
// still sew.)  Kept at twice bsp_prim_check.py's threshold so the tool never
// flags a seam VBSP believed it had sewn.
#define SEW_DEGENERATE_HEIGHT 0.02f

// A fan root, or a primitive triangle, is rejected when it folds back over a
// polygon side by more than this (the polygon is concave there and the
// triangle would draw outside it).  The inward nubs a t-junction vertex can
// make stay far below.  Same value as bsp_prim_check.py.
#define SEW_FLIP_HEIGHT 1.0f

// Folds shallower than this count as none.
#define SEW_FOLD_FREE 0.02f

// How well a fan or triangulation sews a face: the sides left without a
// triangle that has area, then how far its worst triangle folds back (all folds
// up to SEW_FOLD_FREE rate alike).
struct tjScore_t {
  int unsewn;
  float fold;
};

static inline float FoldKey(float fold) {
  return fold <= SEW_FOLD_FREE ? 0.0f : fold;
}

// Is a strictly better than b?
static inline bool ScoreBetter(const tjScore_t &a, const tjScore_t &b) {
  if (a.unsewn != b.unsewn)
    return a.unsewn < b.unsewn;
  return FoldKey(a.fold) < FoldKey(b.fold);
}

// A face that will be written to the BSP, and the ring positions (indices into
// superverts[]) of its vertices in winding order.
struct tjFace_t {
  face_t *face;
  int m;
  int R[MAXEDGES];
};

// The ring of superverts[] of the current face.
struct tjRing_t {
  int N;                      // numsuperverts
  int owner[MAX_SUPERVERTS];  // original edge sub-edge k (ring position k to
                              // k+1) was cut from, or -1
  bool spike[MAX_SUPERVERTS]; // sub-edge k is also walked backwards by another
                              // sub-edge: a zero-width spike, nothing borders it
  bool covered[MAXEDGES];
  Vector normal;
  bool normalValid;

  void Init(int numOrigEdges, const int *edgeStart, const int *edgeCount) {
    N = numsuperverts;
    for (int k = 0; k < N; k++)
      owner[k] = -1;
    for (int e = 0; e < numOrigEdges; e++) {
      for (int k = 0; k < edgeCount[e]; k++)
        owner[(edgeStart[e] + k) % N] = e;
    }

    // Welding verts closer than POINT_EPSILON can leave a winding that visits a
    // vertex twice with a sliver between, e.g. A B C A' D with A' == A (two
    // edges of a sliver tip pick up the same nearby vertex).  The sub-edges
    // A->B and B->A' enclose nothing and no triangle can have area on them, so
    // those two sides are left out of the sewing.  The rest of such a winding
    // is sewn like any other.
    for (int k = 0; k < N; k++) {
      spike[k] = false;
      int a = superverts[k], b = superverts[(k + 1) % N];
      for (int j = 0; j < N && !spike[k]; j++) {
        if (j != k && superverts[j] == b && superverts[(j + 1) % N] == a)
          spike[k] = true;
      }
    }

    // winding normal by Newell's method; robust to collinear runs
    normal.Init(0, 0, 0);
    for (int k = 0; k < N; k++) {
      const Vector &p1 = dvertexes[superverts[k]].point;
      const Vector &p2 = dvertexes[superverts[(k + 1) % N]].point;
      normal[0] += (p1[1] - p2[1]) * (p1[2] + p2[2]);
      normal[1] += (p1[2] - p2[2]) * (p1[0] + p2[0]);
      normal[2] += (p1[0] - p2[0]) * (p1[1] + p2[1]);
    }
    normalValid = normal.LengthSqr() >= 1e-6f;
    if (normalValid)
      normal.NormalizeInPlace();
  }

  const Vector &Point(int pos) const { return dvertexes[superverts[pos]].point; }

  // Is side s of the face R[0..m-1] a spike (see above)?  Chords between
  // fragments are never spikes.
  bool IsSpike(const int *R, int m, int s) const {
    int k = R[s];
    return R[(s + 1) % m] == (k + 1) % N && spike[k];
  }

  // Does the triangle that carries the side from ring position a to b, with
  // its third corner at ring position `apex`, have no area?  Sliver triangles
  // (the apex a few hundredths or more off the line) still draw the side, so
  // they sew.
  bool SideDegenerate(int a, int b, int apex) const {
    const Vector &pa = Point(a);
    Vector ab, ap, cross;
    VectorSubtract(Point(b), pa, ab);
    vec_t len = ab.Length();
    if (len < 1e-4f)
      return false; // coincident verts: nothing to crack against
    VectorSubtract(Point(apex), pa, ap);
    CrossProduct(ab, ap, cross);
    return cross.Length() <= SEW_DEGENERATE_HEIGHT * len;
  }

  // Sides of the face R[0..m-1] not carried by a triangle with area, for a
  // triangulation given as triples of face-local indices.
  int CountUnsewn(const int *R, int m, const int *tris, int numTris) {
    for (int s = 0; s < m; s++)
      covered[s] = IsSpike(R, m, s);
    for (int t = 0; t < numTris; t++) {
      const int *tri = &tris[t * 3];
      for (int c = 0; c < 3; c++) {
        int p = tri[c], q = tri[(c + 1) % 3], apex = tri[(c + 2) % 3];
        int s = -1;
        if (q == (p + 1) % m)
          s = p;
        else if (p == (q + 1) % m)
          s = q;
        if (s >= 0 && !covered[s] &&
            !SideDegenerate(R[s], R[(s + 1) % m], R[apex]))
          covered[s] = true;
      }
    }
    int unsewn = 0;
    for (int s = 0; s < m; s++) {
      if (!covered[s])
        unsewn++;
    }
    return unsewn;
  }

  // Is the triangulation (triples of face-local indices) a sound tiling of the
  // face R[0..m-1]?  m-2 triangles; none folded back over its longest side by
  // more than SEW_FLIP_HEIGHT (Triangulate_r only asks whether two vertices
  // share an edge, not whether the diagonal between them lies inside a concave
  // face); the signed areas add up to the polygon's, and the unsigned ones do
  // not exceed it by more than the folds just allowed.  *pFold gets the worst
  // fold.
  bool TrianglesValid(const int *R, int m, const int *tris, int numTris,
                      float *pFold) const {
    if (!normalValid || numTris != m - 2)
      return false;

    const Vector &p0 = Point(R[0]);
    double polyArea = 0;
    for (int s = 1; s < m - 1; s++) {
      Vector u, v, cross;
      VectorSubtract(Point(R[s]), p0, u);
      VectorSubtract(Point(R[s + 1]), p0, v);
      CrossProduct(u, v, cross);
      polyArea += 0.5 * DotProduct(cross, normal);
    }

    double signedSum = 0, absSum = 0, foldAllowance = 0;
    float worstFold = 0;
    for (int t = 0; t < numTris; t++) {
      float fold, longest;
      double area = TriangleArea(R[tris[t * 3 + 0]], R[tris[t * 3 + 1]],
                                 R[tris[t * 3 + 2]], &fold, &longest);
      if (fold > SEW_FLIP_HEIGHT)
        return false; // folded back: a diagonal outside the face
      if (fold > worstFold)
        worstFold = fold;
      signedSum += area;
      absSum += fabs(area);
      foldAllowance += 0.5 * SEW_FLIP_HEIGHT * longest;
    }
    double slop = 1e-3 * fabs(polyArea) + 0.05;
    if (fabs(signedSum - polyArea) > slop ||
        absSum > polyArea + foldAllowance + slop)
      return false;
    if (pFold)
      *pFold = worstFold;
    return true;
  }

  // Signed area of the triangle at ring positions a, b, c (positive when it
  // winds with the face), how far it folds back (the height of its corner above
  // its longest side when it winds the wrong way, else 0) and that side's
  // length.
  double TriangleArea(int a, int b, int c, float *pFold,
                      float *pLongest) const {
    Vector ab, ac, bc, cross;
    VectorSubtract(Point(b), Point(a), ab);
    VectorSubtract(Point(c), Point(a), ac);
    VectorSubtract(Point(c), Point(b), bc);
    CrossProduct(ab, ac, cross);
    double area = 0.5 * DotProduct(cross, normal);
    float longest = max(ab.Length(), max(ac.Length(), bc.Length()));
    *pLongest = longest;
    *pFold =
        (area < 0 && longest > 1e-6f) ? (float)(-2 * area / longest) : 0.0f;
    return area;
  }

  // Sewing of the fan the engine draws for the face R[0..m-1] (R[0] is the
  // fan root).  False if some triangle folds back over a polygon side by more
  // than SEW_FLIP_HEIGHT (its apex lies that far beyond the side's line), which
  // means the fan would draw outside the face; otherwise the score has the
  // number of sides left without a triangle that has area and the worst fold.
  bool EvaluateFan(const int *R, int m, tjScore_t *pScore) const {
    if (!normalValid)
      return false;
    float worstFold = 0;
    for (int j = 1; j < m - 1; j++) {
      const Vector &a = Point(R[j]);
      Vector ab, ap, cross;
      VectorSubtract(Point(R[j + 1]), a, ab);
      VectorSubtract(Point(R[0]), a, ap);
      CrossProduct(ab, ap, cross);
      vec_t len = ab.Length();
      if (len < 1e-6f)
        continue;
      // apex height above the side, positive on the inside of the winding
      float height = DotProduct(cross, normal) / len;
      if (height < -SEW_FLIP_HEIGHT)
        return false;
      if (-height > worstFold)
        worstFold = -height;
    }
    int unsewn = 0;
    for (int s = 0; s < m; s++) {
      if (IsSpike(R, m, s))
        continue;
      // the apex of the fan triangle that carries side s
      int apex = (s == 0) ? R[2] : (s == m - 1) ? R[m - 2] : R[0];
      if (SideDegenerate(R[s], R[(s + 1) % m], apex))
        unsewn++;
    }
    pScore->unsewn = unsewn;
    pScore->fold = worstFold;
    return true;
  }

  // The best triangulation of the face R[0..m-1] (triples of face-local
  // indices, m-2 of them in outTris) by dynamic programming over its
  // sub-polygons: fewest sides left without a triangle that has area, then the
  // shallowest fold; no triangle folds back by more than SEW_FLIP_HEIGHT.
  // This is exact over proper triangulations (both parts of the score combine
  // monotonically) and costs O(m^3), nothing for a face of at most MAXEDGES
  // vertices.  A pinched ring can have a sound tiling outside that family
  // (one side carried by two triangles) that sews more; it is not searched.
  // False if the face has no such triangulation.
  bool BestTriangulation(const int *R, int m, int *outTris,
                         tjScore_t *pScore) const {
    if (!normalValid || m < 3)
      return false;

    tjScore_t best[MAXEDGES][MAXEDGES];
    signed char split[MAXEDGES][MAXEDGES];
    bool ok[MAXEDGES][MAXEDGES];
    for (int i = 0; i < m; i++) {
      for (int k = 0; k < m; k++)
        ok[i][k] = false;
    }
    for (int i = 0; i + 1 < m; i++) {
      ok[i][i + 1] = true;
      best[i][i + 1].unsewn = 0;
      best[i][i + 1].fold = 0;
    }
    for (int span = 2; span < m; span++) {
      for (int i = 0; i + span < m; i++) {
        int k = i + span;
        for (int j = i + 1; j < k; j++) {
          if (!ok[i][j] || !ok[j][k])
            continue;
          // the triangle (i, j, k): its fold, and the sides of it that are
          // sides of the face
          float fold, longest;
          TriangleArea(R[i], R[j], R[k], &fold, &longest);
          if (fold > SEW_FLIP_HEIGHT)
            continue;
          int unsewn = 0;
          if (j == i + 1 && !IsSpike(R, m, i) &&
              SideDegenerate(R[i], R[j], R[k]))
            unsewn++;
          if (k == j + 1 && !IsSpike(R, m, j) &&
              SideDegenerate(R[j], R[k], R[i]))
            unsewn++;
          if (i == 0 && k == m - 1 && !IsSpike(R, m, m - 1) &&
              SideDegenerate(R[k], R[i], R[j]))
            unsewn++;
          tjScore_t cand;
          cand.unsewn = best[i][j].unsewn + best[j][k].unsewn + unsewn;
          cand.fold = max(max(best[i][j].fold, best[j][k].fold),
                          FoldKey(fold));
          if (!ok[i][k] || ScoreBetter(cand, best[i][k])) {
            best[i][k] = cand;
            split[i][k] = (signed char)j;
            ok[i][k] = true;
          }
        }
      }
    }
    if (!ok[0][m - 1])
      return false;

    // read the triangles back off the splits
    int todo[2 * MAXEDGES][2];
    int numTodo = 0, numTris = 0;
    todo[numTodo][0] = 0;
    todo[numTodo][1] = m - 1;
    numTodo++;
    while (numTodo > 0) {
      numTodo--;
      int i = todo[numTodo][0], k = todo[numTodo][1];
      if (k - i < 2)
        continue;
      int j = split[i][k];
      outTris[numTris * 3 + 0] = i;
      outTris[numTris * 3 + 1] = j;
      outTris[numTris * 3 + 2] = k;
      numTris++;
      todo[numTodo][0] = i;
      todo[numTodo][1] = j;
      numTodo++;
      todo[numTodo][0] = j;
      todo[numTodo][1] = k;
      numTodo++;
    }
    *pScore = best[0][m - 1];
    return numTris == m - 2;
  }

  // The faces FaceFromSuperverts(base) writes, with ring positions.  A face
  // that has to be split gets each fragment started at its best vertex
  // (fragment faces are separate faces, free to start anywhere).  False if a
  // fan folds back too far; otherwise *pScore is the unsewn total over the
  // faces and the worst fold.
  bool PlanFaces(int base, tjFace_t *faces, int *pNumFaces,
                 tjScore_t *pScore) const {
    int fragPos[MAX_RING_FRAGMENTS][MAXEDGES];
    int fragLen[MAX_RING_FRAGMENTS];
    int numFragments = SplitRing(base, fragPos, fragLen);

    tjScore_t total;
    total.unsewn = 0;
    total.fold = 0;
    for (int frag = 0; frag < numFragments; frag++) {
      int m = fragLen[frag];
      int bestRot = -1;
      tjScore_t bestScore = {0, 0};
      for (int rot = 0; rot < (numFragments == 1 ? 1 : m); rot++) {
        int R[MAXEDGES];
        for (int i = 0; i < m; i++)
          R[i] = fragPos[frag][(i + rot) % m];
        tjScore_t score;
        if (!EvaluateFan(R, m, &score))
          continue;
        if (bestRot < 0 || ScoreBetter(score, bestScore)) {
          bestRot = rot;
          bestScore = score;
          if (score.unsewn == 0 && FoldKey(score.fold) == 0)
            break;
        }
      }
      if (bestRot < 0)
        return false;
      faces[frag].m = m;
      for (int i = 0; i < m; i++)
        faces[frag].R[i] = fragPos[frag][(i + bestRot) % m];
      total.unsewn += bestScore.unsewn;
      if (bestScore.fold > total.fold)
        total.fold = bestScore.fold;
    }
    *pNumFaces = numFragments;
    *pScore = total;
    return true;
  }

  // Score of the faces FaceFromSuperverts(base) writes, each started where
  // FaceFromSuperverts() starts it; false if a fan folds back too far.
  bool PlainScore(int base, tjScore_t *pScore) const {
    int fragPos[MAX_RING_FRAGMENTS][MAXEDGES];
    int fragLen[MAX_RING_FRAGMENTS];
    int numFragments = SplitRing(base, fragPos, fragLen);
    pScore->unsewn = 0;
    pScore->fold = 0;
    for (int frag = 0; frag < numFragments; frag++) {
      tjScore_t score;
      if (!EvaluateFan(fragPos[frag], fragLen[frag], &score))
        return false;
      pScore->unsewn += score.unsewn;
      if (score.fold > pScore->fold)
        pScore->fold = score.fold;
    }
    return true;
  }

  // The start vertex whose fan scores best (fewest unsewn sides, then the
  // shallowest fold; the lowest ring position among equals), or false if every
  // root folds back too far.  Stops at the first root that sews with no fold.
  bool FindBestRoot(int *pBestBase, tjScore_t *pBestScore) const {
    bool found = false;
    for (int base = 0; base < N; base++) {
      tjFace_t faces[MAX_RING_FRAGMENTS];
      int numFaces;
      tjScore_t score;
      if (!PlanFaces(base, faces, &numFaces, &score))
        continue;
      if (!found || ScoreBetter(score, *pBestScore)) {
        found = true;
        *pBestBase = base;
        *pBestScore = score;
        if (score.unsewn == 0 && FoldKey(score.fold) == 0)
          break;
      }
    }
    return found;
  }
};

// FaceFromSuperverts() and a list of the faces it wrote.  With a ring, a face
// that has to be split also gets the best start vertex for every fragment.
static int FaceFromSupervertsSewn(face_t **pListHead, face_t *f, int base,
                                  const tjRing_t *ring, tjFace_t *faces) {
  int numFaces = 0;
  tjScore_t score;
  if (ring && numsuperverts > MAXEDGES &&
      ring->PlanFaces(base, faces, &numFaces, &score)) {
    FaceFromSuperverts(pListHead, f, base);
    // the fragment faces hang off f as split[0] of f, f->split[1], ...
    face_t *cur = f;
    for (int frag = 0; frag < numFaces; frag++) {
      face_t *fragFace = cur->split[0] ? cur->split[0] : cur;
      faces[frag].face = fragFace;
      fragFace->numpoints = faces[frag].m;
      for (int i = 0; i < faces[frag].m; i++)
        fragFace->vertexnums[i] = superverts[faces[frag].R[i]];
      cur = cur->split[1];
    }
    return numFaces;
  }

  FaceFromSuperverts(pListHead, f, base);
  if (numsuperverts <= MAXEDGES) {
    faces[0].face = f;
    faces[0].m = numsuperverts;
    for (int i = 0; i < numsuperverts; i++)
      faces[0].R[i] = (i + base) % numsuperverts;
    return 1;
  }

  // split without a plan: plain fragments
  int fragPos[MAX_RING_FRAGMENTS][MAXEDGES];
  int fragLen[MAX_RING_FRAGMENTS];
  numFaces = SplitRing(base, fragPos, fragLen);
  face_t *cur = f;
  for (int frag = 0; frag < numFaces; frag++) {
    faces[frag].face = cur->split[0] ? cur->split[0] : cur;
    faces[frag].m = fragLen[frag];
    for (int i = 0; i < fragLen[frag]; i++)
      faces[frag].R[i] = fragPos[frag][i];
    cur = cur->split[1];
  }
  return numFaces;
}

// A re-triangulated primitive wanted by a face.  Decided in
// ResolveTjuncPrims() once the whole model is known, so that running out of
// primitive space costs the cheapest seams instead of whatever came last in
// tree order.
struct tjPrimCandidate_t {
  face_t *face;
  int benefit;      // polygon sides the primitive sews that the fan does not
  int fanUnsewn;    // sides the fan leaves unsewn without the primitive
  int residual;     // sides the primitive still leaves unsewn
  bool stockStyle;  // a water face: stock gave it the primitive regardless
  CUtlVector<int> indices; // face-local
};

static CUtlVector<tjPrimCandidate_t *> s_tjPrimCandidates;
static int c_tjuncPrimSearched;  // primitives that came from the search
static int c_tjuncPrimRejected;  // faces with no sound triangulation at all

// Primitive format limits (see the note above).
#define TJUNC_MAX_PRIMS 32768
#define TJUNC_MAX_PRIMINDICES 65536

// Re-triangulate one written face for a primitive.  The stock triangulation
// (Triangulate_r) is used if it is a sound tiling that sews the face; if it is
// not, or leaves seams or folds, the best triangulation of the face is searched
// for (BestTriangulation), which finds a sewing one whenever there is one, so
// concave and pinched faces get a primitive too.  The result is kept only if it
// sews more of the face than the fan the engine would draw does (a primitive is
// never spent on a fold alone) - or, for stockStyle, whatever the stock
// triangulation gives.  Returns false if there is none.
static bool AddPrimCandidate(const tjFace_t &tf, tjRing_t &ring,
                             const tjScore_t &fan, bool stockStyle) {
  const int m = tf.m;

  // Mark the sides each vertex sits between with the original edge they were
  // cut from (chords between fragments get an edge of their own), the same
  // "is collinear" test as ever: two vertices that share an edge cannot be
  // joined by a diagonal.
  CUtlVector<face_vert_table_t> poly;
  poly.AddMultipleToTail(m);
  for (int j = 0; j < m; j++) {
    int prev = (j + m - 1) % m;
    int before = (tf.R[j] == (tf.R[prev] + 1) % ring.N) ? ring.owner[tf.R[prev]]
                                                         : -1;
    int after = (tf.R[(j + 1) % m] == (tf.R[j] + 1) % ring.N)
                    ? ring.owner[tf.R[j]]
                    : -1;
    if (before < 0)
      before = MAXEDGES + prev; // the chord into vertex j
    if (after < 0)
      after = MAXEDGES + j; // the chord out of vertex j
    poly[j].AddEdge(before);
    if (after != before)
      poly[j].AddEdge(after);
  }

  CUtlVector<int> inIndices;
  CUtlVector<int> outIndices;
  for (int i = 0; i < m; i++)
    inIndices.AddToTail(i);
  Triangulate_r(outIndices, inIndices, poly);

  const int *bestTris = NULL;
  tjScore_t bestScore = {0, 0};
  float fold;
  if (outIndices.Count() == 3 * (m - 2) &&
      ring.TrianglesValid(tf.R, m, outIndices.Base(), m - 2, &fold)) {
    bestTris = outIndices.Base();
    bestScore.unsewn = ring.CountUnsewn(tf.R, m, outIndices.Base(), m - 2);
    bestScore.fold = fold;
  }

  int searched[3 * (MAXEDGES - 2)];
  bool fromSearch = false;
  if (!stockStyle &&
      (!bestTris || bestScore.unsewn > 0 || FoldKey(bestScore.fold) > 0)) {
    tjScore_t score;
    if (ring.BestTriangulation(tf.R, m, searched, &score) &&
        ring.TrianglesValid(tf.R, m, searched, m - 2, NULL) &&
        (!bestTris || ScoreBetter(score, bestScore))) {
      bestTris = searched;
      bestScore = score;
      fromSearch = true;
    }
  }

  if (!bestTris) {
    c_tjuncPrimRejected++;
    return false;
  }
  if (!stockStyle && bestScore.unsewn >= fan.unsewn)
    return false;
  if (fromSearch)
    c_tjuncPrimSearched++;

  tjPrimCandidate_t *cand = new tjPrimCandidate_t;
  cand->face = tf.face;
  cand->benefit = fan.unsewn > bestScore.unsewn ? fan.unsewn - bestScore.unsewn
                                                : 0;
  cand->fanUnsewn = fan.unsewn;
  cand->residual = bestScore.unsewn;
  cand->stockStyle = stockStyle;
  cand->indices.CopyArray(bestTris, 3 * (m - 2));
  s_tjPrimCandidates.AddToTail(cand);
  return true;
}

/*
==================
FixFaceEdges

Cuts the edges of one face around the vertices that lie on them and picks the
start vertex / primitive that sews them (see "T-junction sewing" above).
==================
*/
void FixFaceEdges(face_t **pList, face_t *f) {
  int p1, p2;
  int i;
  Vector e2;
  vec_t len;
  int count[MAX_SUPERVERTS], start[MAX_SUPERVERTS];

  if (f->merged || f->split[0] || f->split[1])
    return;

  numsuperverts = 0;

  int originalPoints = f->numpoints;
  for (i = 0; i < f->numpoints; i++) {
    p1 = f->vertexnums[i];
    p2 = f->vertexnums[(i + 1) % f->numpoints];

    VectorCopy(dvertexes[p1].point, edge_start);
    VectorCopy(dvertexes[p2].point, e2);

    FindEdgeVerts(edge_start, e2);

    VectorSubtract(e2, edge_start, edge_dir);
    len = VectorNormalize(edge_dir);

    start[i] = numsuperverts;
    TestEdge(0, len, p1, p2, 0);

    count[i] = numsuperverts - start[i];
  }

  if (numsuperverts < 3) { // entire face collapsed
    f->numpoints = 0;
    c_facecollapse++;
    return;
  }

  // The stock start: a vertex that doesn't have tjunctions on either side,
  // which can cause artifacts on trifans, especially underwater.
  int stockBase = 0;
  bool badStart = true;
  for (i = 0; i < f->numpoints; i++) {
    if (count[i] == 1 && count[(i + f->numpoints - 1) % f->numpoints] == 1) {
      stockBase = start[i]; // rotate the vertex order
      badStart = false;
      break;
    }
  }
  if (badStart)
    c_badstartverts++;

  // Faces EmitFace() drops are never drawn (the same test it makes; faces from
  // the tree always have dispinfo -1, displacement faces are made elsewhere),
  // and triangles need no sewing.
  if (((texinfo[f->texinfo].flags & SURF_NODRAW) && f->dispinfo == -1) ||
      numsuperverts < 4) {
    FaceFromSuperverts(pList, f, stockBase); // may fragment if > MAXEDGES
    return;
  }

  tjRing_t ring;
  ring.Init(originalPoints, start, count);
  tjFace_t written[MAX_RING_FRAGMENTS];

  // A zero-area winding draws nothing; there is nothing to sew.
  if (!ring.normalValid) {
    FaceFromSuperverts(pList, f, stockBase);
    if (entity_num == 0)
      c_tjuncZeroArea++;
    return;
  }

  // Faces that touch water keep the stock behaviour (the original authors saw
  // artifacts there that nothing here explains): the stock start, no rotation
  // of split fragments, and - as stock VBSP did for every world face with no
  // clean start - the re-triangulated primitive, whether or not the fan from
  // the stock start sews.  The two deliberate differences from stock: no
  // primitive for a face split because of more than MAXEDGES vertices (stock's
  // was orphaned: EmitFace never writes the split parent), and Triangulate_r's
  // result is validated (TrianglesValid) before it is used.
  if (f->fogVolumeLeaf != NULL || (texinfo[f->texinfo].flags & SURF_WARP)) {
    FaceFromSuperverts(pList, f, stockBase);
    if (entity_num != 0)
      return;
    tjScore_t fan;
    if (!ring.PlainScore(stockBase, &fan)) {
      fan.unsewn = numsuperverts;
      fan.fold = 0;
    }
    if (badStart && numsuperverts <= MAXEDGES) {
      tjFace_t tf;
      tf.face = f;
      tf.m = numsuperverts;
      for (i = 0; i < numsuperverts; i++)
        tf.R[i] = (i + stockBase) % numsuperverts;
      if (AddPrimCandidate(tf, ring, fan, true))
        return;
    }
    if (fan.unsewn > 0)
      c_tjuncStockKept++;
    return;
  }

  // Pick the start vertex.  A fan from the stock start that sews every side
  // without folding is kept.  Otherwise any other vertex is tried - rotating
  // the winding changes nothing but the fan, so it is free, and a lossless
  // improvement wherever the face is, brush entities included (which just never
  // get a primitive, as stock) - and the best fan wins: fewest unsewn sides,
  // then the shallowest fold, the stock start among equals.  It also applies
  // where the count above saw no t-junction at all: a fan is unsewn wherever the
  // start sits on a straight run (original corners that happen to be collinear,
  // or a nub of an inserted vertex next to a corner), and a fan folds back over
  // a concave face.
  int numWritten;
  tjScore_t stock = {numsuperverts, 0};
  bool stockValid = ring.PlanFaces(stockBase, written, &numWritten, &stock);
  int base = stockBase;
  tjScore_t cur = stock;
  bool curValid = stockValid;
  if (!(stockValid && stock.unsewn == 0 && FoldKey(stock.fold) == 0)) {
    int bestBase;
    tjScore_t best;
    if (ring.FindBestRoot(&bestBase, &best) &&
        (!stockValid || ScoreBetter(best, stock))) {
      base = bestBase;
      cur = best;
      curValid = true;
    }
  }

  if (curValid && cur.unsewn == 0) {
    if (base != stockBase)
      c_tjuncRerooted++;
    else if (badStart)
      c_tjuncRootClean++;
    FaceFromSupervertsSewn(pList, f, base, &ring, written);
    return;
  }

  // No fan sews this face: write it with the best one there is, then see
  // whether a primitive does better on any of the faces written.
  int numFaces = FaceFromSupervertsSewn(pList, f, base, &ring, written);
  for (int w = 0; w < numFaces; w++) {
    tjScore_t fan;
    if (!ring.EvaluateFan(written[w].R, written[w].m, &fan)) {
      fan.unsewn = written[w].m; // folds back too far: as bad as it gets
      fan.fold = 0;
    }
    if (fan.unsewn == 0)
      continue;
    if (entity_num != 0) { // no primitives for brush entities
      c_tjuncBrushUnsewn++;
      continue;
    }
    if (!AddPrimCandidate(written[w], ring, fan, false))
      c_tjuncUnsewnOther++;
  }
}

static const tjPrimCandidate_t *const *s_pRankCandidates;

// Best value first: most seams recovered per primitive index, then tree order.
static int CompareCandidateValue(const void *pa, const void *pb) {
  int a = *(const int *)pa, b = *(const int *)pb;
  const tjPrimCandidate_t *ca = s_pRankCandidates[a];
  const tjPrimCandidate_t *cb = s_pRankCandidates[b];
  long long lhs = (long long)ca->benefit * cb->indices.Count();
  long long rhs = (long long)cb->benefit * ca->indices.Count();
  if (lhs != rhs)
    return lhs > rhs ? -1 : 1;
  return a - b;
}

/*
==================
ResolveTjuncPrims

Writes the primitives the faces asked for, in tree order, if they fit the
format limits.  If they do not, keeps the best-value ones that do; the other
faces stay as the fan they were written as.
==================
*/
static void ResolveTjuncPrims() {
  int numCands = s_tjPrimCandidates.Count();
  if (numCands == 0)
    return;

  int availPrims = TJUNC_MAX_PRIMS - g_numprimitives;
  int availIndices = TJUNC_MAX_PRIMINDICES - g_numprimindices;

  int wantIndices = 0;
  for (int c = 0; c < numCands; c++)
    wantIndices += s_tjPrimCandidates[c]->indices.Count();

  CUtlVector<char> keep;
  keep.SetCount(numCands);
  if (numCands <= availPrims && wantIndices <= availIndices) {
    for (int c = 0; c < numCands; c++)
      keep[c] = 1;
  } else {
    CUtlVector<int> order;
    for (int c = 0; c < numCands; c++) {
      keep[c] = 0;
      order.AddToTail(c);
    }
    s_pRankCandidates = s_tjPrimCandidates.Base();
    qsort(order.Base(), numCands, sizeof(int), CompareCandidateValue);

    int usedPrims = 0, usedIndices = 0;
    for (int o = 0; o < numCands; o++) {
      const tjPrimCandidate_t *cand = s_tjPrimCandidates[order[o]];
      if (usedPrims + 1 > availPrims ||
          usedIndices + cand->indices.Count() > availIndices)
        continue; // a smaller one further down may still fit
      keep[order[o]] = 1;
      usedPrims++;
      usedIndices += cand->indices.Count();
    }
    int numDropped = 0, droppedSeams = 0;
    for (int c = 0; c < numCands; c++) {
      if (!keep[c]) {
        numDropped++;
        droppedSeams += s_tjPrimCandidates[c]->benefit;
      }
    }
    Warning("\nWARNING: t-junction primitives do not fit the BSP format "
            "(%d prims / %d indices wanted, room for %d / %d).\n"
            "         %d of %d faces are drawn without their sewing primitive "
            "(%d seams left open); the rest are sewn.\n",
            numCands, wantIndices, availPrims, availIndices, numDropped,
            numCands, droppedSeams);
  }

  for (int c = 0; c < numCands; c++) {
    tjPrimCandidate_t *cand = s_tjPrimCandidates[c];
    face_t *f = cand->face;
    if (!keep[c]) {
      // dropped: the face stays a fan, which only matters if the fan has seams
      if (cand->fanUnsewn > 0) {
        if (cand->stockStyle)
          c_tjuncStockKept++;
        else
          c_tjuncUnsewnBudget++;
      }
    } else {
      dprimitive_t &newPrim = g_primitives[g_numprimitives];
      f->firstPrimID = g_numprimitives;
      g_numprimitives++;
      f->numPrims = 1;
      newPrim.firstIndex = g_numprimindices;
      newPrim.firstVert = g_numprimverts;
      newPrim.indexCount = cand->indices.Count();
      newPrim.vertCount = 0;
      newPrim.type = PRIM_TRILIST;
      for (int i = 0; i < cand->indices.Count(); i++)
        g_primindices[g_numprimindices + i] = cand->indices[i];
      g_numprimindices += cand->indices.Count();
      if (cand->residual == 0)
        c_tjuncPrimSewn++;
      else if (cand->stockStyle)
        c_tjuncStockKept++;
      else
        c_tjuncUnsewnPrim++;
    }
    delete cand;
  }
  s_tjPrimCandidates.RemoveAll();
}

/*
==================
FixEdges_r
==================
*/
void FixEdges_r(node_t *node) {
  int i;
  face_t *f;

  if (node->planenum == PLANENUM_LEAF) {
    return;
  }

  for (f = node->faces; f; f = f->next)
    FixFaceEdges(&node->faces, f);

  for (i = 0; i < 2; i++)
    FixEdges_r(node->children[i]);
}

//-----------------------------------------------------------------------------
// Purpose: Fix the t-junctions on detail faces
//-----------------------------------------------------------------------------
void FixLeafFaceEdges(face_t **ppLeafFaceList) {
  face_t *f;

  for (f = *ppLeafFaceList; f; f = f->next) {
    FixFaceEdges(ppLeafFaceList, f);
  }
}

/*
===========
FixTjuncs

===========
*/

face_t *FixTjuncs(node_t *headnode, face_t *pLeafFaceList) {
  // snap and merge all vertexes
  qprintf("---- snap verts ----\n");
  memset(hashverts, 0, sizeof(hashverts));
  memset(vertexchain, 0, sizeof(vertexchain));
  c_totalverts = 0;
  c_uniqueverts = 0;
  c_faceoverflows = 0;
  EmitNodeFaceVertexes_r(headnode);

  // UNDONE: This count is wrong with tjuncs off on details - since

  // break edges on tjunctions
  qprintf("---- tjunc ----\n");
  c_tryedges = 0;
  c_degenerate = 0;
  c_facecollapse = 0;
  c_tjunctions = 0;
  c_tjuncRootClean = 0;
  c_tjuncRerooted = 0;
  c_tjuncPrimSewn = 0;
  c_tjuncUnsewnPrim = 0;
  c_tjuncUnsewnBudget = 0;
  c_tjuncUnsewnOther = 0;
  c_tjuncStockKept = 0;
  c_tjuncZeroArea = 0;
  c_tjuncBrushUnsewn = 0;
  c_tjuncPrimSearched = 0;
  c_tjuncPrimRejected = 0;
  int numPrimsBefore = g_numprimitives;
  int numPrimIndicesBefore = g_numprimindices;

  if (g_bAllowDetailCracks) {
    FixEdges_r(headnode);
    EmitLeafFaceVertexes(&pLeafFaceList);
    FixLeafFaceEdges(&pLeafFaceList);
  } else {
    EmitLeafFaceVertexes(&pLeafFaceList);
    if (!notjunc) {
      FixEdges_r(headnode);
      FixLeafFaceEdges(&pLeafFaceList);
    }
  }

  // write the primitives the faces asked for (or cut them back to fit)
  ResolveTjuncPrims();

  if (entity_num != 0) {
    g_tjuncBrushUnsewn += c_tjuncBrushUnsewn;
  } else {
    g_tjuncRootClean = c_tjuncRootClean;
    g_tjuncRerooted = c_tjuncRerooted;
    g_tjuncPrimSewn = c_tjuncPrimSewn;
    g_tjuncUnsewn = c_tjuncUnsewnPrim + c_tjuncUnsewnBudget +
                    c_tjuncUnsewnOther + c_tjuncStockKept;
    // Printed with Msg(), not qprintf(): this is the only report of how the
    // faces with no clean start vertex were sewn, and of the primitive space
    // used (a hard 64K format limit).
    Msg("T-junction sewing: %d faces already clean at the stock start, "
        "%d rerooted by rotation, %d sewn by primitive "
        "(%d prims, %d indices); %d faces left unsewn "
        "(%d over the primitive budget, %d where the primitive still leaves "
        "a seam, %d that no fan or primitive sews, %d water/fog faces kept "
        "at the stock start); %d zero-area windings (nothing to sew)\n",
        c_tjuncRootClean, c_tjuncRerooted, c_tjuncPrimSewn,
        g_numprimitives - numPrimsBefore,
        g_numprimindices - numPrimIndicesBefore, g_tjuncUnsewn,
        c_tjuncUnsewnBudget, c_tjuncUnsewnPrim, c_tjuncUnsewnOther,
        c_tjuncStockKept, c_tjuncZeroArea);
    if (c_tjuncPrimSearched || c_tjuncPrimRejected)
      Msg("  (%d primitives came from the triangulation search because the "
          "stock one was unsewn, folded or incomplete; %d faces had no sound "
          "triangulation)\n",
          c_tjuncPrimSearched, c_tjuncPrimRejected);
  }

  qprintf("%i unique from %i\n", c_uniqueverts, c_totalverts);
  qprintf("%5i edges degenerated\n", c_degenerate);
  qprintf("%5i faces degenerated\n", c_facecollapse);
  qprintf("%5i edges added by tjunctions\n", c_tjunctions);
  qprintf("%5i faces added by tjunctions\n", c_faceoverflows);
  qprintf("%5i bad start verts\n", c_badstartverts);

  return pLeafFaceList;
}

//========================================================

int c_faces;

face_t *AllocFace(void) {
  static int s_FaceId = 0;

  face_t *f;

  f = (face_t *)malloc(sizeof(*f));
  memset(f, 0, sizeof(*f));
  f->id = s_FaceId;
  ++s_FaceId;

  // Placement-new the CUtlVector so its internal state is valid
  new (&f->mergedSideIds) CUtlVector<int>();

  c_faces++;

  return f;
}

face_t *NewFaceFromFace(face_t *f) {
  face_t *newf;

  newf = AllocFace();
  *newf = *f;
  newf->merged = NULL;
  newf->split[0] = newf->split[1] = NULL;
  newf->w = NULL;
  return newf;
}

void FreeFace(face_t *f) {
  if (f->w)
    FreeWinding(f->w);
  // Explicitly destroy the CUtlVector before free()
  f->mergedSideIds.~CUtlVector<int>();
  free(f);
  c_faces--;
}

void FreeFaceList(face_t *pFaces) {
  while (pFaces) {
    face_t *next = pFaces->next;

    FreeFace(pFaces);
    pFaces = next;
  }
}

//========================================================

void GetEdge2_InitOptimizedList() {
  for (int i = 0; i < MAX_MAP_VERTS; i++)
    g_VertEdgeList[i].RemoveAll();
}

void IntSort(CUtlVector<int> &theList) {
  for (int i = 0; i < theList.Size() - 1; i++) {
    if (theList[i] > theList[i + 1]) {
      int temp = theList[i];
      theList[i] = theList[i + 1];
      theList[i + 1] = temp;
      if (i > 0)
        i -= 2;
      else
        i = -1;
    }
  }
}

int AddEdge(int v1, int v2, face_t *f) {
  if (numedges >= MAX_MAP_EDGES)
    Error("Too many edges in map, max == %d", MAX_MAP_EDGES);

  g_VertEdgeList[v1].AddToTail(numedges);
  g_VertEdgeList[v2].AddToTail(numedges);
  IntSort(g_VertEdgeList[v1]);
  IntSort(g_VertEdgeList[v2]);

  dedge_t *edge = &dedges[numedges];
  numedges++;

  edge->v[0] = v1;
  edge->v[1] = v2;
  edgefaces[numedges - 1][0] = f;
  return numedges - 1;
}

/*
==================
GetEdge

Called by writebsp.
Don't allow four way edges
==================
*/
int GetEdge2(int v1, int v2, face_t *f) {
  dedge_t *edge;

  c_tryedges++;

  if (!noshare) {
    // Check all edges connected to v1.
    CUtlVector<int> &theList = g_VertEdgeList[v1];
    for (int i = 0; i < theList.Size(); i++) {
      int iEdge = theList[i];
      edge = &dedges[iEdge];
      if (v1 == edge->v[1] && v2 == edge->v[0] &&
          edgefaces[iEdge][0]->contents == f->contents) {
        if (edgefaces[iEdge][1])
          continue;

        edgefaces[iEdge][1] = f;
        return -iEdge;
      }
    }
  }

  return AddEdge(v1, v2, f);
}

/*
===========================================================================

FACE MERGING

===========================================================================
*/

#define CONTINUOUS_EPSILON 0.001

/*
=============
TryMergeWinding

If two polygons share a common edge and the edges that meet at the
common points are both inside the other polygons, merge them

Returns NULL if the faces couldn't be merged, or the new face.
The originals will NOT be freed.
=============
*/
winding_t *TryMergeWinding(winding_t *f1, winding_t *f2, Vector &planenormal) {
  Vector *p1, *p2, *p3, *p4, *back;
  winding_t *newf;
  int i, j, k, l;
  Vector normal, delta;
  vec_t dot;
  qboolean keep1, keep2;

  //
  // find a common edge
  //
  p1 = p2 = NULL; // stop compiler warning
  j = 0;          //

  for (i = 0; i < f1->numpoints; i++) {
    p1 = &f1->p[i];
    p2 = &f1->p[(i + 1) % f1->numpoints];
    for (j = 0; j < f2->numpoints; j++) {
      p3 = &f2->p[j];
      p4 = &f2->p[(j + 1) % f2->numpoints];
      for (k = 0; k < 3; k++) {
        if (fabs((*p1)[k] - (*p4)[k]) > EQUAL_EPSILON)
          break;
        if (fabs((*p2)[k] - (*p3)[k]) > EQUAL_EPSILON)
          break;
      }
      if (k == 3)
        break;
    }
    if (j < f2->numpoints)
      break;
  }

  if (i == f1->numpoints)
    return NULL; // no matching edges

  //
  // check slope of connected lines
  // if the slopes are colinear, the point can be removed
  //
  back = &f1->p[(i + f1->numpoints - 1) % f1->numpoints];
  VectorSubtract(*p1, *back, delta);
  CrossProduct(planenormal, delta, normal);
  VectorNormalize(normal);

  back = &f2->p[(j + 2) % f2->numpoints];
  VectorSubtract(*back, *p1, delta);
  dot = DotProduct(delta, normal);
  if (dot > CONTINUOUS_EPSILON)
    return NULL; // not a convex polygon
  keep1 = (qboolean)(dot < -CONTINUOUS_EPSILON);

  back = &f1->p[(i + 2) % f1->numpoints];
  VectorSubtract(*back, *p2, delta);
  CrossProduct(planenormal, delta, normal);
  VectorNormalize(normal);

  back = &f2->p[(j + f2->numpoints - 1) % f2->numpoints];
  VectorSubtract(*back, *p2, delta);
  dot = DotProduct(delta, normal);
  if (dot > CONTINUOUS_EPSILON)
    return NULL; // not a convex polygon
  keep2 = (qboolean)(dot < -CONTINUOUS_EPSILON);

  //
  // build the new polygon
  //
  newf = AllocWinding(f1->numpoints + f2->numpoints);

  // copy first polygon
  for (k = (i + 1) % f1->numpoints; k != i; k = (k + 1) % f1->numpoints) {
    if (k == (i + 1) % f1->numpoints && !keep2)
      continue;

    VectorCopy(f1->p[k], newf->p[newf->numpoints]);
    newf->numpoints++;
  }

  // copy second polygon
  for (l = (j + 1) % f2->numpoints; l != j; l = (l + 1) % f2->numpoints) {
    if (l == (j + 1) % f2->numpoints && !keep1)
      continue;
    VectorCopy(f2->p[l], newf->p[newf->numpoints]);
    newf->numpoints++;
  }

  return newf;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
bool OverlaysAreEqual(face_t *f1, face_t *f2) {
  // Check the overlay ids - see if they are the same.
  if (f1->originalface->aOverlayIds.Count() !=
      f2->originalface->aOverlayIds.Count())
    return false;

  int nOverlayCount = f1->originalface->aOverlayIds.Count();
  for (int iOverlay = 0; iOverlay < nOverlayCount; ++iOverlay) {
    int nOverlayId = f1->originalface->aOverlayIds[iOverlay];
    if (f2->originalface->aOverlayIds.Find(nOverlayId) == -1)
      return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
bool FaceOnWaterBrush(face_t *face) {
  side_t *pSide = face->originalface;
  if (!pSide)
    return false;

  if (pSide->contents & (CONTENTS_WATER | CONTENTS_SLIME))
    return true;

  return false;
}

/*
=============
TryMerge

If two polygons share a common edge and the edges that meet at the
common points are both inside the other polygons, merge them

Returns NULL if the faces couldn't be merged, or the new face.
The originals will NOT be freed.
=============
*/
face_t *TryMerge(face_t *f1, face_t *f2, Vector &planenormal) {
  face_t *newf;
  winding_t *nw;

  if (!f1->w || !f2->w)
    return NULL;
  if (f1->texinfo != f2->texinfo)
    return NULL;
  if (f1->planenum != f2->planenum) // on front and back sides
    return NULL;
  if (f1->contents != f2->contents)
    return NULL;
  if (f1->originalface->smoothingGroups != f2->originalface->smoothingGroups)
    return NULL;
  if (!OverlaysAreEqual(f1, f2))
    return NULL;
  if (nomergewater && (FaceOnWaterBrush(f1) || FaceOnWaterBrush(f2)))
    return NULL;

  nw = TryMergeWinding(f1->w, f2->w, planenormal);
  if (!nw)
    return NULL;

  c_merge++;
  newf = NewFaceFromFace(f1);
  newf->w = nw;

  // Accumulate all original VMF side IDs from both parents
  // once each: fragments of one side that merge back would repeat its ID. A
  // parent with an empty list stands for its originalface alone, the same
  // fallback writebsp uses, so no merged side is dropped.
  newf->mergedSideIds.RemoveAll();
  const face_t *parents[2] = {f1, f2};
  for (int p = 0; p < 2; p++) {
    const face_t *pf = parents[p];
    if (pf->mergedSideIds.Count() == 0) {
      if (newf->mergedSideIds.Find(pf->originalface->id) < 0)
        newf->mergedSideIds.AddToTail(pf->originalface->id);
      continue;
    }
    for (int i = 0; i < pf->mergedSideIds.Count(); i++) {
      if (newf->mergedSideIds.Find(pf->mergedSideIds[i]) < 0)
        newf->mergedSideIds.AddToTail(pf->mergedSideIds[i]);
    }
  }

  f1->merged = newf;
  f2->merged = newf;

  return newf;
}

/*
===============
MergeFaceList
===============
*/
void MergeFaceList(face_t **pList) {
  face_t *f1, *f2, *end;
  face_t *merged;
  plane_t *plane;

  merged = NULL;

  for (f1 = *pList; f1; f1 = f1->next) {
    if (f1->merged || f1->split[0] || f1->split[1])
      continue;
    for (f2 = *pList; f2 != f1; f2 = f2->next) {
      if (f2->merged || f2->split[0] || f2->split[1])
        continue;

      plane = &g_MainMap->mapplanes[f1->planenum];
      merged = TryMerge(f1, f2, plane->normal);
      if (!merged)
        continue;

      // add merged to the end of the face list
      // so it will be checked against all the faces again
      for (end = *pList; end->next; end = end->next)
        ;
      merged->next = NULL;
      end->next = merged;
      break;
    }
  }
}

//=====================================================================

/*
===============
SubdivideFace

Chop up faces that are larger than we want in the surface cache
===============
*/
void SubdivideFace(face_t **pFaceList, face_t *f) {
  float mins, maxs;
  vec_t v;
  vec_t luxelsPerWorldUnit;
  int axis, i;
  texinfo_t *tex;
  Vector temp;
  vec_t dist;
  winding_t *w, *frontw, *backw;

  if (f->merged || f->split[0] || f->split[1])
    return;

  // special (non-surface cached) faces don't need subdivision
  tex = &texinfo[f->texinfo];

  if (tex->flags & SURF_NOLIGHT) {
    return;
  }

  for (axis = 0; axis < 2; axis++) {
    while (1) {
      mins = 999999;
      maxs = -999999;

      VECTOR_COPY(tex->lightmapVecsLuxelsPerWorldUnits[axis], temp);
      w = f->w;
      for (i = 0; i < w->numpoints; i++) {
        v = DotProduct(w->p[i], temp);
        if (v < mins)
          mins = v;
        if (v > maxs)
          maxs = v;
      }
#if 0
			if (maxs - mins <= 0)
				Error ("zero extents");
#endif
      if (maxs - mins <= g_maxLightmapDimension)
        break;

      // split it
      c_subdivide++;

      luxelsPerWorldUnit = VectorNormalize(temp);

      dist = (mins + g_maxLightmapDimension - 1) / luxelsPerWorldUnit;

      ClipWindingEpsilon(w, temp, dist, ON_EPSILON, &frontw, &backw);
      if (!frontw || !backw)
        Error("SubdivideFace: didn't split the polygon");

      f->split[0] = NewFaceFromFace(f);
      f->split[0]->w = frontw;
      f->split[0]->next = *pFaceList;
      *pFaceList = f->split[0];

      f->split[1] = NewFaceFromFace(f);
      f->split[1]->w = backw;
      f->split[1]->next = *pFaceList;
      *pFaceList = f->split[1];

      SubdivideFace(pFaceList, f->split[0]);
      SubdivideFace(pFaceList, f->split[1]);
      return;
    }
  }
}

void SubdivideFaceList(face_t **pFaceList) {
  face_t *f;

  for (f = *pFaceList; f; f = f->next) {
    SubdivideFace(pFaceList, f);
  }
}

//-----------------------------------------------------------------------------
// Assigns the bottom material to the bottom face
//-----------------------------------------------------------------------------
static bool AssignBottomWaterMaterialToFace(face_t *f) {
  // NOTE: This happens *after* cubemap fixup occurs, so we need to get the
  // fixed-up bottom material for this
  texinfo_t *pTexInfo = &texinfo[f->texinfo];
  dtexdata_t *pTexData = GetTexData(pTexInfo->texdata);
  const char *pMaterialName =
      TexDataStringTable_GetString(pTexData->nameStringTableID);

  char pBottomMatName[512];
  if (!GetValueFromPatchedMaterial(pMaterialName, "$bottommaterial",
                                   pBottomMatName, 512)) {
    if (!Q_stristr(pMaterialName, "nodraw") &&
        !Q_stristr(pMaterialName, "toolsskip")) {
      Warning("error: material %s doesn't have a $bottommaterial\n",
              pMaterialName);
    }
    return false;
  }

  // Assert( mapplanes[f->planenum].normal.z < 0 );
  texinfo_t newTexInfo;
  newTexInfo.flags = pTexInfo->flags;
  int j, k;
  for (j = 0; j < 2; j++) {
    for (k = 0; k < 4; k++) {
      newTexInfo.textureVecsTexelsPerWorldUnits[j][k] =
          pTexInfo->textureVecsTexelsPerWorldUnits[j][k];
      newTexInfo.lightmapVecsLuxelsPerWorldUnits[j][k] =
          pTexInfo->lightmapVecsLuxelsPerWorldUnits[j][k];
    }
  }
  newTexInfo.texdata = FindOrCreateTexData(pBottomMatName);
  f->texinfo = FindOrCreateTexInfo(newTexInfo);

  return true;
}

//===========================================================================

int c_nodefaces;

/*
============
FaceFromPortal

============
*/
extern int FindOrCreateTexInfo(const texinfo_t &searchTexInfo);

face_t *FaceFromPortal(portal_t *p, int pside) {
  face_t *f;
  side_t *side;
  int deltaContents;

  // portal does not bridge different visible contents
  side = p->side;
  if (!side)
    return NULL;

  // allocate a new face
  f = AllocFace();

  // save the original "side" from the map brush -- portal->side
  // see FindPortalSide(...)
  f->originalface = side;

  // Seed the merged side ID list with this face's original VMF side ID
  f->mergedSideIds.AddToTail(side->id);

  //
  // save material info
  //
  f->texinfo = side->texinfo;
  f->dispinfo = -1; // all faces with displacement info are created elsewhere
  f->smoothingGroups = side->smoothingGroups;

  // save plane info
  f->planenum = (side->planenum & ~1) | pside;
  if (entity_num != 0) {
    // the brush model renderer doesn't use PLANEBACK, so write the real plane
    // inside water faces can be flipped because they are generated on the
    // inside of the brush
    if (p->nodes[pside]->contents & (CONTENTS_WATER | CONTENTS_SLIME)) {
      f->planenum = (side->planenum & ~1) | pside;
    } else {
      f->planenum = side->planenum;
    }
  }

  // save portal info
  f->portal = p;
  f->fogVolumeLeaf = NULL;

  deltaContents =
      VisibleContents(p->nodes[!pside]->contents ^ p->nodes[pside]->contents);

  // don't show insides of windows or grates
  if (((p->nodes[pside]->contents & CONTENTS_WINDOW) &&
       deltaContents == CONTENTS_WINDOW) ||
      ((p->nodes[pside]->contents & CONTENTS_GRATE) &&
       deltaContents == CONTENTS_GRATE)) {
    FreeFace(f);
    return NULL;
  }

  if (p->nodes[pside]->contents & MASK_WATER) {
    f->fogVolumeLeaf = p->nodes[pside];
  } else if (p->nodes[!pside]->contents & MASK_WATER) {
    f->fogVolumeLeaf = p->nodes[!pside];
  }

  // If it's the underside of water, we need to figure out what material to use,
  // etc.
  if ((p->nodes[pside]->contents & CONTENTS_WATER) &&
      deltaContents == CONTENTS_WATER) {
    if (!AssignBottomWaterMaterialToFace(f)) {
      FreeFace(f);
      return NULL;
    }
  }

  //
  // generate the winding for the face and save face contents
  //
  if (pside) {
    f->w = ReverseWinding(p->winding);
    f->contents = p->nodes[1]->contents;
  } else {
    f->w = CopyWinding(p->winding);
    f->contents = p->nodes[0]->contents;
  }

  f->numPrims = 0;
  f->firstPrimID = 0;

  // return the created face
  return f;
}

/*
===============
MakeFaces_r

If a portal will make a visible face,
mark the side that originally created it

  solid / empty : solid
  solid / water : solid
  water / empty : water
  water / water : none
===============
*/
void MakeFaces_r(node_t *node) {
  portal_t *p;
  int s;

  // recurse down to leafs
  if (node->planenum != PLANENUM_LEAF) {
    MakeFaces_r(node->children[0]);
    MakeFaces_r(node->children[1]);

    // merge together all visible faces on the node
    if (!nomerge)
      MergeFaceList(&node->faces);
    if (!nosubdiv)
      SubdivideFaceList(&node->faces);

    return;
  }

  // solid leafs never have visible faces
  if (node->contents & CONTENTS_SOLID)
    return;

  // see which portals are valid
  for (p = node->portals; p; p = p->next[s]) {
    s = (p->nodes[1] == node);

    p->face[s] = FaceFromPortal(p, s);
    if (p->face[s]) {
      c_nodefaces++;
      p->face[s]->next = p->onnode->faces;
      p->onnode->faces = p->face[s];
    }
  }
}

/*
============
MakeFaces
============
*/
void MakeFaces(node_t *node) {
  qprintf("--- MakeFaces ---\n");
  c_merge = 0;
  c_subdivide = 0;
  c_nodefaces = 0;

  MakeFaces_r(node);

  qprintf("%5i makefaces\n", c_nodefaces);
  qprintf("%5i merged\n", c_merge);
  qprintf("%5i subdivided\n", c_subdivide);
}