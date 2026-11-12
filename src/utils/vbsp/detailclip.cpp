//========= detailclip: drop detail props that clip into the map ==============
//
// See detailclip.h for what is dropped and why. Everything here runs only with
// -detailclip.
//
// The geometry tests are exact convex-convex separating-axis tests in double
// precision:
//   - a candidate is its bounding box (shrunk, see BuildCandidate) cut by the
//     tangent plane of the surface it sits on, a convex polytope with at most
//     a dozen vertices;
//   - a world or func_detail brush is the convex hull of its side windings;
//   - a static prop is a bag of LOD0 triangles, held in one BVH per model and
//     tested in the model's own space, so the instances (which differ only by
//     origin and angles) share it;
//   - a displacement is a bag of world-space triangles (its full-resolution
//     displaced mesh), held in one BVH per displacement and tested directly.
// Separating axes are the face normals of both shapes and the cross products
// of their edge directions, which makes the test exact for convex shapes: a
// candidate is only reported as overlapping when it really is. Overlaps
// shallower than kPenetration count as touching, which keeps props that rest
// on a surface or a neighbouring wall.
//=============================================================================
#include "detailclip.h"

#include "bsplib.h"
#include "bspflags.h"
#include "builddisp.h"
#include "disp_vbsp.h"
#include "gamebspfile.h"
#include "mathlib/mathlib.h"
#include "proplm_shared.h"
#include "studio.h"
#include "tier1/UtlBuffer.h"
#include "tier1/strtools.h"
#include "tier1/utldict.h"
#include "tier1/utlvector.h"
#include "vbsp.h"
#include <algorithm>
#include <float.h>
#include <math.h>
#include <string.h>

bool g_bDetailClip = false;
float g_flDetailClipShrink = 0.5f;

namespace {

//-----------------------------------------------------------------------------
// Tolerances (world units)
//-----------------------------------------------------------------------------
// The candidate box starts this far above the tangent plane of the spawn
// surface, so the surface the prop stands on is never an overlap.
const double kLift = 0.1;
// Two shapes must overlap by more than this along every axis to count.
const double kPenetration = 0.01;
// The buried probe: this far above the spawn point along the surface normal,
// and how far inside a brush it must be (a probe on a face is not inside).
const double kProbeHeight = 1.0;
const double kProbeMargin = 0.01;
// Floor on a box half-extent, for models that are paper thin on an axis.
const double kMinHalfExtent = 0.25;

//-----------------------------------------------------------------------------
// Small double-precision vector
//-----------------------------------------------------------------------------
struct V3 {
  double x, y, z;
};

inline V3 MakeV3(double x, double y, double z) {
  V3 r = {x, y, z};
  return r;
}
inline V3 FromVector(const Vector &v) { return MakeV3(v.x, v.y, v.z); }
inline V3 operator+(const V3 &a, const V3 &b) {
  return MakeV3(a.x + b.x, a.y + b.y, a.z + b.z);
}
inline V3 operator-(const V3 &a, const V3 &b) {
  return MakeV3(a.x - b.x, a.y - b.y, a.z - b.z);
}
inline V3 operator*(const V3 &a, double s) {
  return MakeV3(a.x * s, a.y * s, a.z * s);
}
inline double Dot3(const V3 &a, const V3 &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline V3 Cross3(const V3 &a, const V3 &b) {
  return MakeV3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x);
}
inline double Length3(const V3 &a) { return sqrt(Dot3(a, a)); }

// The windows.h min/max macros are on in this project, so no std::min/max.
template <class T> inline T Min2(T a, T b) { return a < b ? a : b; }
template <class T> inline T Max2(T a, T b) { return a < b ? b : a; }

struct Box_t {
  V3 mn, mx;
};

// Inclusive: touching boxes overlap. These are only broad-phase rejects.
inline bool BoxesOverlap(const Box_t &a, const Box_t &b) {
  return a.mn.x <= b.mx.x && a.mx.x >= b.mn.x && a.mn.y <= b.mx.y &&
         a.mx.y >= b.mn.y && a.mn.z <= b.mx.z && a.mx.z >= b.mn.z;
}

inline void BoxAdd(Box_t &b, const V3 &p) {
  b.mn.x = Min2(b.mn.x, p.x);
  b.mn.y = Min2(b.mn.y, p.y);
  b.mn.z = Min2(b.mn.z, p.z);
  b.mx.x = Max2(b.mx.x, p.x);
  b.mx.y = Max2(b.mx.y, p.y);
  b.mx.z = Max2(b.mx.z, p.z);
}

inline Box_t EmptyBox() {
  Box_t b;
  b.mn = MakeV3(DBL_MAX, DBL_MAX, DBL_MAX);
  b.mx = MakeV3(-DBL_MAX, -DBL_MAX, -DBL_MAX);
  return b;
}

// Appends axis unless a (nearly) parallel one, of either sign, is in the list.
void AddAxis(CUtlVector<V3> &list, const V3 &axis) {
  for (int i = 0; i < list.Count(); ++i) {
    if (fabs(Dot3(list[i], axis)) >= 0.999999)
      return;
  }
  list.AddToTail(axis);
}

//-----------------------------------------------------------------------------
// Convex-convex separating axis test
//-----------------------------------------------------------------------------
struct PolyView_t {
  const V3 *v;
  int nv;
  const V3 *faceAxis; // unit face normals (either sign)
  int nfa;
  const V3 *edge; // unit edge directions (either sign)
  int ne;
};

void Project(const V3 *v, int n, const V3 &axis, double &lo, double &hi) {
  lo = hi = Dot3(v[0], axis);
  for (int i = 1; i < n; ++i) {
    const double d = Dot3(v[i], axis);
    lo = Min2(lo, d);
    hi = Max2(hi, d);
  }
}

bool SeparatedOn(const PolyView_t &a, const PolyView_t &b, const V3 &axis) {
  double alo, ahi, blo, bhi;
  Project(a.v, a.nv, axis, alo, ahi);
  Project(b.v, b.nv, axis, blo, bhi);
  return ahi <= blo + kPenetration || bhi <= alo + kPenetration;
}

// True when the two convex shapes overlap by more than kPenetration on every
// candidate axis. With the true face normals and edge directions of both
// shapes among the axes (extra axes are harmless) this is exact.
bool ConvexOverlap(const PolyView_t &a, const PolyView_t &b) {
  for (int i = 0; i < a.nfa; ++i) {
    if (SeparatedOn(a, b, a.faceAxis[i]))
      return false;
  }
  for (int i = 0; i < b.nfa; ++i) {
    if (SeparatedOn(a, b, b.faceAxis[i]))
      return false;
  }
  for (int i = 0; i < a.ne; ++i) {
    for (int j = 0; j < b.ne; ++j) {
      V3 c = Cross3(a.edge[i], b.edge[j]);
      const double len = Length3(c);
      if (len < 1e-6)
        continue; // parallel edges: already covered by a face axis
      if (SeparatedOn(a, b, c * (1.0 / len)))
        return false;
    }
  }
  return true;
}

//-----------------------------------------------------------------------------
// Uniform 2D grid over boxes (xy; z is left to the exact tests)
//-----------------------------------------------------------------------------
class CGrid2D {
public:
  void Build(const CUtlVector<Box_t> &boxes, double cellSize) {
    m_Start.Purge();
    m_Items.Purge();
    m_Big.Purge();
    m_Stamp.Purge();
    m_Query = 0;
    m_NX = m_NY = 0;
    if (boxes.Count() == 0)
      return;

    Box_t all = EmptyBox();
    for (int i = 0; i < boxes.Count(); ++i) {
      BoxAdd(all, boxes[i].mn);
      BoxAdd(all, boxes[i].mx);
    }
    m_OriginX = all.mn.x;
    m_OriginY = all.mn.y;
    const double extent = Max2(all.mx.x - all.mn.x, all.mx.y - all.mn.y);
    // At most 1024 cells along an axis, however large the map.
    const double cell = Max2(cellSize, extent / 1024.0);
    m_InvCell = 1.0 / cell;
    m_NX = (int)((all.mx.x - m_OriginX) * m_InvCell) + 1;
    m_NY = (int)((all.mx.y - m_OriginY) * m_InvCell) + 1;

    // A box covering this many cells is kept on a list that is always tested:
    // filing a map-sized brush in every cell would cost more than it saves.
    const int bigCells = 256;

    m_Start.SetCount(m_NX * m_NY + 1);
    memset(m_Start.Base(), 0, m_Start.Count() * sizeof(int));
    for (int pass = 0; pass < 2; ++pass) {
      if (pass == 1) {
        int sum = 0;
        for (int c = 0; c < m_NX * m_NY; ++c) {
          const int n = m_Start[c];
          m_Start[c] = sum;
          sum += n;
        }
        m_Start[m_NX * m_NY] = sum;
        m_Items.SetCount(sum);
        m_Fill.SetCount(m_NX * m_NY);
        memset(m_Fill.Base(), 0, m_Fill.Count() * sizeof(int));
      }
      for (int i = 0; i < boxes.Count(); ++i) {
        int x0, y0, x1, y1;
        CellRange(boxes[i], x0, y0, x1, y1);
        if ((x1 - x0 + 1) * (y1 - y0 + 1) > bigCells) {
          if (pass == 0)
            m_Big.AddToTail(i);
          continue;
        }
        for (int y = y0; y <= y1; ++y) {
          for (int x = x0; x <= x1; ++x) {
            const int c = y * m_NX + x;
            if (pass == 0)
              ++m_Start[c];
            else
              m_Items[m_Start[c] + m_Fill[c]++] = i;
          }
        }
      }
    }
    m_Fill.Purge();
    m_Stamp.SetCount(boxes.Count());
    memset(m_Stamp.Base(), 0, m_Stamp.Count() * sizeof(int));
  }

  // Calls fn(id) once for every box id filed in a cell the query touches, and
  // for every big box; stops and returns true as soon as fn returns true.
  template <class Fn> bool Query(const Box_t &q, Fn fn) {
    if (m_NX == 0)
      return false;
    ++m_Query;
    for (int i = 0; i < m_Big.Count(); ++i) {
      if (fn(m_Big[i]))
        return true;
    }
    int x0, y0, x1, y1;
    CellRange(q, x0, y0, x1, y1);
    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        const int c = y * m_NX + x;
        for (int k = m_Start[c]; k < m_Start[c + 1]; ++k) {
          const int id = m_Items[k];
          if (m_Stamp[id] == m_Query)
            continue;
          m_Stamp[id] = m_Query;
          if (fn(id))
            return true;
        }
      }
    }
    return false;
  }

  void Purge() {
    m_Start.Purge();
    m_Items.Purge();
    m_Big.Purge();
    m_Stamp.Purge();
    m_NX = m_NY = 0;
  }

private:
  void CellRange(const Box_t &b, int &x0, int &y0, int &x1, int &y1) const {
    x0 = Max2(0, Min2(m_NX - 1, (int)floor((b.mn.x - m_OriginX) * m_InvCell)));
    x1 = Max2(0, Min2(m_NX - 1, (int)floor((b.mx.x - m_OriginX) * m_InvCell)));
    y0 = Max2(0, Min2(m_NY - 1, (int)floor((b.mn.y - m_OriginY) * m_InvCell)));
    y1 = Max2(0, Min2(m_NY - 1, (int)floor((b.mx.y - m_OriginY) * m_InvCell)));
  }

  double m_OriginX = 0, m_OriginY = 0, m_InvCell = 1;
  int m_NX = 0, m_NY = 0;
  CUtlVector<int> m_Start, m_Items, m_Fill, m_Big, m_Stamp;
  int m_Query = 0;
};

//-----------------------------------------------------------------------------
// Counters for the summary line
//-----------------------------------------------------------------------------
struct Counters_t {
  int candidates;
  int buried;
  int brush;
  int staticProp;
  int displacement;
  int noBounds; // candidates that could only be given the buried test
  double buildSeconds; // DetailClip_Begin
  double testSeconds;  // all DetailClip_Test calls
};
Counters_t s_Count;

//-----------------------------------------------------------------------------
// World and func_detail brushes
//-----------------------------------------------------------------------------
struct ClipBrush_t {
  Box_t box;
  int planeStart, planeCount; // s_PlaneN / s_PlaneD, for the point test
  int vertStart, vertCount;   // s_BrushVert
  int axisStart, axisCount;   // s_BrushAxis: face normals, one per direction
  int edgeStart, edgeCount;   // s_BrushEdge: unit edge directions, one each
};

CUtlVector<ClipBrush_t> s_Brush;
CUtlVector<V3> s_PlaneN;
CUtlVector<double> s_PlaneD;
CUtlVector<V3> s_BrushVert;
CUtlVector<V3> s_BrushAxis;
CUtlVector<V3> s_BrushEdge;
CGrid2D s_BrushGrid;
int s_BadBrushes = 0;

// Contents the player can run into: solid, and not clip/water/glass. Sky brushes
// count: the player runs into them and a prop poking into one shows through the
// sky.
bool IsSolidContents(int contents) {
  const int notSolid = CONTENTS_PLAYERCLIP | CONTENTS_MONSTERCLIP |
                       CONTENTS_WINDOW | CONTENTS_GRATE | CONTENTS_SLIME |
                       CONTENTS_WATER | CONTENTS_TRANSLUCENT |
                       CONTENTS_AREAPORTAL | CONTENTS_ORIGIN | CONTENTS_LADDER;
  return (contents & CONTENTS_SOLID) && !(contents & notSolid);
}

const int kInvisibleSurf = SURF_NODRAW | SURF_SKIP | SURF_HINT;

// A solid brush the player can see: it has at least one side that is drawn (a
// brush made only of nodraw / skip / hint sides is invisible scaffolding).
bool IsVisibleSolidBrush(const mapbrush_t *b) {
  if (b->numsides < 4 || !IsSolidContents(b->contents))
    return false;
  // A displacement's parent brush is only its flat base; the visible surface
  // is the displaced mesh, which can dip below it. Testing the base hull would
  // bury every prop placed in a dip, so displacement brushes are skipped. (The
  // map loader already drops them from the list; this is a guard.)
  for (int i = 0; i < b->numsides; ++i) {
    if (b->original_sides[i].pMapDisp)
      return false;
  }
  for (int i = 0; i < b->numsides; ++i) {
    const side_t *s = &b->original_sides[i];
    if (s->bevel || !s->winding)
      continue;
    if (!(s->surf & kInvisibleSurf))
      return true;
  }
  return false;
}

bool AddClipBrush(const mapbrush_t *b) {
  CUtlVector<V3> verts, edges, axes;
  CUtlVector<V3> planeN;
  CUtlVector<double> planeD;

  for (int i = 0; i < b->numsides; ++i) {
    const side_t *s = &b->original_sides[i];
    if (s->bevel || !s->winding || s->winding->numpoints < 3)
      continue;
    const plane_t &pl = g_MainMap->mapplanes[s->planenum];
    const V3 n = FromVector(pl.normal);
    bool known = false;
    for (int k = 0; k < planeN.Count(); ++k) {
      if (Dot3(planeN[k], n) > 0.999999 && fabs(planeD[k] - pl.dist) < 0.01) {
        known = true;
        break;
      }
    }
    if (!known) {
      planeN.AddToTail(n);
      planeD.AddToTail((double)pl.dist);
      AddAxis(axes, n);
    }

    const winding_t *w = s->winding;
    for (int j = 0; j < w->numpoints; ++j) {
      const V3 p = FromVector(w->p[j]);
      bool have = false;
      for (int k = 0; k < verts.Count(); ++k) {
        const V3 d = verts[k] - p;
        if (fabs(d.x) < 0.02 && fabs(d.y) < 0.02 && fabs(d.z) < 0.02) {
          have = true;
          break;
        }
      }
      if (!have)
        verts.AddToTail(p);

      const V3 q = FromVector(w->p[(j + 1) % w->numpoints]);
      V3 e = q - p;
      // Every real edge direction goes in, however short: a sliver's edge
      // left out of the axes can hide the separating axis and report an
      // overlap that is not there. Extra axes are always safe. (AddAxis still
      // merges directions within about 0.08 degrees of one already listed.)
      const double len = Length3(e);
      if (len < 1e-6)
        continue;
      AddAxis(edges, e * (1.0 / len));
    }
  }

  if (verts.Count() < 4 || planeN.Count() < 4)
    return false;

  // The hull of the windings must sit behind every side plane. If it does not
  // the plane faces the wrong way (or the winding is junk); leave the brush
  // out rather than test against nonsense.
  V3 center = MakeV3(0, 0, 0);
  Box_t box = EmptyBox();
  for (int i = 0; i < verts.Count(); ++i) {
    center = center + verts[i];
    BoxAdd(box, verts[i]);
  }
  center = center * (1.0 / verts.Count());
  for (int i = 0; i < planeN.Count(); ++i) {
    if (Dot3(planeN[i], center) - planeD[i] > 0.05) {
      ++s_BadBrushes;
      return false;
    }
  }

  ClipBrush_t cb;
  cb.box = box;
  cb.planeStart = s_PlaneN.Count();
  cb.planeCount = planeN.Count();
  for (int i = 0; i < planeN.Count(); ++i) {
    s_PlaneN.AddToTail(planeN[i]);
    s_PlaneD.AddToTail(planeD[i]);
  }
  cb.vertStart = s_BrushVert.Count();
  cb.vertCount = verts.Count();
  for (int i = 0; i < verts.Count(); ++i)
    s_BrushVert.AddToTail(verts[i]);
  cb.axisStart = s_BrushAxis.Count();
  cb.axisCount = axes.Count();
  for (int i = 0; i < axes.Count(); ++i)
    s_BrushAxis.AddToTail(axes[i]);
  cb.edgeStart = s_BrushEdge.Count();
  cb.edgeCount = edges.Count();
  for (int i = 0; i < edges.Count(); ++i)
    s_BrushEdge.AddToTail(edges[i]);
  s_Brush.AddToTail(cb);
  return true;
}

void BuildBrushes() {
  CUtlVector<Box_t> boxes;
  // After the func_detail merge the world entity owns every func_detail brush
  // too; the brushes of the other brush entities are not part of the map.
  const entity_t *world = &g_MainMap->entities[0];
  for (int i = 0; i < world->numbrushes; ++i) {
    const mapbrush_t *b = &g_MainMap->mapbrushes[world->firstbrush + i];
    if (!IsVisibleSolidBrush(b))
      continue;
    AddClipBrush(b);
  }
  boxes.SetCount(s_Brush.Count());
  for (int i = 0; i < s_Brush.Count(); ++i)
    boxes[i] = s_Brush[i].box;
  s_BrushGrid.Build(boxes, 256.0);
}

bool PointInsideBrush(const ClipBrush_t &cb, const V3 &p) {
  for (int i = 0; i < cb.planeCount; ++i) {
    if (Dot3(s_PlaneN[cb.planeStart + i], p) - s_PlaneD[cb.planeStart + i] >
        -kProbeMargin)
      return false;
  }
  return true;
}

PolyView_t BrushView(const ClipBrush_t &cb) {
  PolyView_t v;
  v.v = s_BrushVert.Base() + cb.vertStart;
  v.nv = cb.vertCount;
  v.faceAxis = s_BrushAxis.Base() + cb.axisStart;
  v.nfa = cb.axisCount;
  v.edge = s_BrushEdge.Base() + cb.edgeStart;
  v.ne = cb.edgeCount;
  return v;
}

//-----------------------------------------------------------------------------
// Model files: header bounds and the LOD0 mesh
//-----------------------------------------------------------------------------
struct Tri_t {
  float v[3][3];
};

struct BvhNode_t {
  float mn[3], mx[3];
  int left, right; // children, or -1 for a leaf
  int first, count; // a leaf's triangles
};

struct MeshInfo_t {
  bool unreadable; // the MDL did not load at all
  char reason[200];
  bool hasHull, hasView, hasMesh;
  V3 hullMin, hullMax, viewMin, viewMax;
  Box_t meshBox; // over the vertices the default-bodygroup LOD0 triangles use
  CUtlVector<Tri_t> tris;       // only when asked for
  CUtlVector<BvhNode_t> nodes;  // root is node 0
};

// A flat card has no thickness but is a real extent; only a point (or a bounds
// the model never filled in) is not.
bool BoxHasExtent(const V3 &mn, const V3 &mx) {
  return Length3(mx - mn) > 0.5;
}

int BuildBvh(MeshInfo_t &m, CUtlVector<int> &order, int first, int count) {
  const int nodeIndex = m.nodes.AddToTail();
  BvhNode_t node;
  node.mn[0] = node.mn[1] = node.mn[2] = FLT_MAX;
  node.mx[0] = node.mx[1] = node.mx[2] = -FLT_MAX;
  Box_t centroids = EmptyBox();
  for (int i = first; i < first + count; ++i) {
    const Tri_t &t = m.tris[order[i]];
    V3 c = MakeV3(0, 0, 0);
    for (int k = 0; k < 3; ++k) {
      for (int a = 0; a < 3; ++a) {
        node.mn[a] = Min2(node.mn[a], t.v[k][a]);
        node.mx[a] = Max2(node.mx[a], t.v[k][a]);
      }
      c = c + MakeV3(t.v[k][0], t.v[k][1], t.v[k][2]);
    }
    BoxAdd(centroids, c * (1.0 / 3.0));
  }
  node.left = node.right = -1;
  node.first = first;
  node.count = count;
  if (count > 4) {
    const V3 ext = centroids.mx - centroids.mn;
    const int axis = (ext.x >= ext.y && ext.x >= ext.z) ? 0 : (ext.y >= ext.z ? 1 : 2);
    const int mid = first + count / 2;
    std::nth_element(order.Base() + first, order.Base() + mid,
                     order.Base() + first + count, [&](int a, int b) {
                       const Tri_t &ta = m.tris[a];
                       const Tri_t &tb = m.tris[b];
                       return ta.v[0][axis] + ta.v[1][axis] + ta.v[2][axis] <
                              tb.v[0][axis] + tb.v[1][axis] + tb.v[2][axis];
                     });
    node.count = 0;
    m.nodes[nodeIndex] = node;
    const int l = BuildBvh(m, order, first, mid - first);
    const int r = BuildBvh(m, order, mid, first + count - mid);
    m.nodes[nodeIndex].left = l;
    m.nodes[nodeIndex].right = r;
    return nodeIndex;
  }
  m.nodes[nodeIndex] = node;
  return nodeIndex;
}

// Builds the BVH over m.tris and reorders m.tris to match its leaves.
void FinishBvh(MeshInfo_t &m) {
  CUtlVector<int> order;
  order.SetCount(m.tris.Count());
  for (int i = 0; i < order.Count(); ++i)
    order[i] = i;
  // The leaves index the reordered triangle array, so build over the order
  // first, then permute the triangles to match.
  BuildBvh(m, order, 0, order.Count());
  CUtlVector<Tri_t> sorted;
  sorted.SetCount(m.tris.Count());
  for (int i = 0; i < order.Count(); ++i)
    sorted[i] = m.tris[order[i]];
  m.tris.Swap(sorted);
}

void LoadMeshInfo(const char *pModelName, bool wantTris, MeshInfo_t &m) {
  m.unreadable = true;
  m.hasHull = m.hasView = m.hasMesh = false;
  m.meshBox = EmptyBox();
  m.reason[0] = 0;

  CUtlBuffer mdlBuf;
  if (!Proplm_ReadWholeFile(pModelName, mdlBuf)) {
    V_snprintf(m.reason, sizeof(m.reason), "cannot read the MDL");
    return;
  }
  if (mdlBuf.TellPut() < (int)sizeof(studiohdr_t) ||
      strncmp((const char *)mdlBuf.Base(), "IDST", 4)) {
    V_snprintf(m.reason, sizeof(m.reason), "not a studio model");
    return;
  }
  studiohdr_t *pHdr = (studiohdr_t *)mdlBuf.Base();
  if (!Proplm_ConvertMdl(pHdr, mdlBuf.TellPut(), m.reason, sizeof(m.reason)))
    return;
  if (!Proplm_ValidateMdl(pHdr, mdlBuf.TellPut(), m.reason, sizeof(m.reason)))
    return;
  m.unreadable = false;

  m.hullMin = FromVector(pHdr->hull_min);
  m.hullMax = FromVector(pHdr->hull_max);
  m.hasHull = BoxHasExtent(m.hullMin, m.hullMax);
  m.viewMin = FromVector(pHdr->view_bbmin);
  m.viewMax = FromVector(pHdr->view_bbmax);
  m.hasView = BoxHasExtent(m.viewMin, m.viewMax);

  char path[MAX_PATH];
  V_strncpy(path, pModelName, sizeof(path));
  V_SetExtension(path, ".vvd", sizeof(path));
  CUtlBuffer vvdBuf;
  if (!Proplm_ReadWholeFile(path, vvdBuf)) {
    V_snprintf(m.reason, sizeof(m.reason), "cannot read %s", path);
    return;
  }
  CUtlBuffer vtxBuf;
  V_strncpy(path, pModelName, sizeof(path));
  V_SetExtension(path, ".dx90.vtx", sizeof(path));
  if (!Proplm_ReadWholeFile(path, vtxBuf)) {
    V_strncpy(path, pModelName, sizeof(path));
    V_SetExtension(path, ".dx80.vtx", sizeof(path));
    if (!Proplm_ReadWholeFile(path, vtxBuf)) {
      V_snprintf(m.reason, sizeof(m.reason), "no .dx90.vtx or .dx80.vtx");
      return;
    }
  }

  CUtlVector<ProplmVvdVert_t> verts;
  if (!Proplm_LoadVvdVertices(vvdBuf, pHdr->checksum, 0.0f, verts, m.reason,
                              sizeof(m.reason)))
    return;
  CUtlVector<ProplmVtxTri_t> tris;
  if (!Proplm_GatherVtxTriangles(pHdr, vtxBuf, verts.Count(), 0, false, false,
                                 NULL, tris, m.reason, sizeof(m.reason)))
    return;
  if (tris.Count() == 0) {
    V_snprintf(m.reason, sizeof(m.reason), "no LOD0 triangles");
    return;
  }

  m.hasMesh = true;
  if (wantTris)
    m.tris.EnsureCapacity(tris.Count());
  for (int t = 0; t < tris.Count(); ++t) {
    Tri_t tri;
    for (int c = 0; c < 3; ++c) {
      const ProplmVvdVert_t &vert = verts[tris[t].vert[c]];
      for (int a = 0; a < 3; ++a)
        tri.v[c][a] = vert.pos[a];
      BoxAdd(m.meshBox, MakeV3(vert.pos[0], vert.pos[1], vert.pos[2]));
    }
    if (wantTris)
      m.tris.AddToTail(tri);
  }

  if (wantTris)
    FinishBvh(m);
}

void NormalizeModelKey(const char *pName, char *pOut, int outLen) {
  V_strncpy(pOut, pName, outLen);
  V_FixSlashes(pOut, '/');
  V_strlower(pOut);
}

// Detail model bounds (no triangles kept), one load per model.
CUtlDict<MeshInfo_t *, int> s_DetailModels;
// Static prop meshes with their BVH, one load per model.
CUtlDict<MeshInfo_t *, int> s_PropModels;

void FreeModelDict(CUtlDict<MeshInfo_t *, int> &dict) {
  for (int i = dict.First(); i != dict.InvalidIndex(); i = dict.Next(i))
    delete dict[i];
  dict.Purge();
}

//-----------------------------------------------------------------------------
// Static prop instances
//-----------------------------------------------------------------------------
struct PropInst_t {
  const MeshInfo_t *mesh;
  matrix3x4_t xf; // model -> world
  Box_t box;      // world bounds of the mesh
};

CUtlVector<PropInst_t> s_Inst;
CGrid2D s_InstGrid;
int s_PropModelsLoaded = 0;

void BuildStaticProps() {
  CUtlVector<Box_t> boxes;
  const int count = StaticProp_InstanceCount();
  for (int i = 0; i < count; ++i) {
    Vector origin;
    QAngle angles;
    const char *pModelName;
    StaticProp_GetInstance(i, origin, angles, pModelName);

    char key[MAX_PATH];
    NormalizeModelKey(pModelName, key, sizeof(key));
    int d = s_PropModels.Find(key);
    if (d == s_PropModels.InvalidIndex()) {
      MeshInfo_t *m = new MeshInfo_t;
      LoadMeshInfo(pModelName, true, *m);
      d = s_PropModels.Insert(key, m);
      if (m->hasMesh) {
        ++s_PropModelsLoaded;
      } else {
        Warning("Warning: detailclip: no mesh for static prop model \"%s\" "
                "(%s); detail props inside it are kept.\n",
                pModelName, m->reason);
      }
    }
    const MeshInfo_t *mesh = s_PropModels[d];
    if (!mesh->hasMesh)
      continue;

    PropInst_t inst;
    inst.mesh = mesh;
    AngleMatrix(angles, origin, inst.xf);
    inst.box = EmptyBox();
    for (int c = 0; c < 8; ++c) {
      const Vector local((c & 1) ? mesh->meshBox.mx.x : mesh->meshBox.mn.x,
                         (c & 2) ? mesh->meshBox.mx.y : mesh->meshBox.mn.y,
                         (c & 4) ? mesh->meshBox.mx.z : mesh->meshBox.mn.z);
      Vector world;
      VectorTransform(local, inst.xf, world);
      BoxAdd(inst.box, FromVector(world));
    }
    s_Inst.AddToTail(inst);
    boxes.AddToTail(inst.box);
  }
  s_InstGrid.Build(boxes, 256.0);
}

//-----------------------------------------------------------------------------
// Displacements
//-----------------------------------------------------------------------------
// The visible surface of a displacement is its displaced mesh, not the flat
// base of its parent brush, so each one is a bag of world-space triangles in a
// BVH, the same shape as a static prop mesh (with an identity transform). The
// mesh is the full-resolution tessellation, the one the engine draws up close
// and the one VBSP samples to place the props.
struct DispInst_t {
  int dispIndex;    // index into mapdispinfo[], as dface_t::dispinfo
  MeshInfo_t *mesh; // owned
  Box_t box;        // world bounds of the mesh
  // The mesh's four edges in s_DispEdgeVert: 4 runs of edgeLen vertices each,
  // in order along the edge.
  int firstEdgeVert, edgeLen;
};

CUtlVector<DispInst_t> s_Disp;
CGrid2D s_DispGrid;
CUtlVector<V3> s_DispEdgeVert;

// Pairs of displacements sewn along an edge (sewn neighbours), as
// (low mapdispinfo index << 32 | high), sorted. A candidate skips these as it
// skips its own displacement (#112): together they are one continuous surface,
// and the relief of that surface is what the tangent plane stands for. Testing
// the neighbour but not the own displacement removed props near seams for the
// same relief that is accepted everywhere else.
CUtlVector<uint64> s_DispSewn;
const double kSewTolerance = 0.1; // units between two meshes' ring vertices

inline uint64 SewnKey(int a, int b) {
  if (a > b)
    std::swap(a, b);
  return ((uint64)(uint32)a << 32) | (uint32)b;
}

bool DispSewn(int a, int b) {
  if (a < 0 || b < 0)
    return false;
  const uint64 key = SewnKey(a, b);
  return std::binary_search(s_DispSewn.Base(),
                            s_DispSewn.Base() + s_DispSewn.Count(), key);
}

// Built from mapdispinfo[] rather than g_CoreDispInfos, which -onlyprops never
// fills.
void AddDisplacement(int dispIndex) {
  mapdispinfo_t *pMapDisp = &mapdispinfo[dispIndex];
  const winding_t *pBase =
      pMapDisp->face.originalface ? pMapDisp->face.originalface->winding : NULL;
  if (!pBase || pBase->numpoints != 4)
    return;

  CCoreDispInfo core;
  DispMapToCoreDispInfo(pMapDisp, &core, NULL, NULL);
  const CPowerInfo *pPower = core.GetPowerInfo();

  MeshInfo_t *m = new MeshInfo_t;
  m->unreadable = false;
  m->hasHull = m->hasView = m->hasMesh = false;
  m->reason[0] = 0;
  m->meshBox = EmptyBox();
  m->tris.EnsureCapacity(pPower->GetNumTriInfos());
  for (int t = 0; t < pPower->GetNumTriInfos(); ++t) {
    const CTriInfo *pTri = pPower->GetTriInfo(t);
    Tri_t tri;
    for (int c = 0; c < 3; ++c) {
      const Vector &p = core.GetVert(pTri->m_Indices[c]);
      for (int a = 0; a < 3; ++a)
        tri.v[c][a] = p[a];
      BoxAdd(m->meshBox, FromVector(p));
    }
    m->tris.AddToTail(tri);
  }
  if (m->tris.Count() == 0) {
    delete m;
    return;
  }
  FinishBvh(*m);
  m->hasMesh = true;

  DispInst_t d;
  d.dispIndex = dispIndex;
  d.mesh = m;
  d.box = m->meshBox;
  d.firstEdgeVert = s_DispEdgeVert.Count();
  // Vertex index is y * width + x; displacements are square (width == height).
  const int w = core.GetWidth();
  d.edgeLen = w;
  for (int k = 0; k < w; ++k) // y = 0
    s_DispEdgeVert.AddToTail(FromVector(core.GetVert(k)));
  for (int k = 0; k < w; ++k) // x = w - 1
    s_DispEdgeVert.AddToTail(FromVector(core.GetVert(k * w + w - 1)));
  for (int k = 0; k < w; ++k) // y = w - 1
    s_DispEdgeVert.AddToTail(FromVector(core.GetVert((w - 1) * w + k)));
  for (int k = 0; k < w; ++k) // x = 0
    s_DispEdgeVert.AddToTail(FromVector(core.GetVert(k * w)));
  s_Disp.AddToTail(d);
}

// Squared distance from p to segment ab.
double PointSegDist2(const V3 &p, const V3 &a, const V3 &b) {
  const V3 ab = b - a, ap = p - a;
  const double len2 = Dot3(ab, ab);
  double t = len2 > 0 ? Dot3(ap, ab) / len2 : 0;
  t = t < 0 ? 0 : (t > 1 ? 1 : t);
  const V3 d = ap - ab * t;
  return Dot3(d, d);
}

// Does a whole edge of a lie on b's outer ring (every vertex of that edge
// within kSewTolerance of one of b's edge polylines)?
bool EdgeLiesOnRing(const DispInst_t &a, const DispInst_t &b) {
  const double tol2 = kSewTolerance * kSewTolerance;
  for (int e = 0; e < 4; ++e) {
    bool all = true;
    for (int i = 0; i < a.edgeLen && all; ++i) {
      const V3 &p = s_DispEdgeVert[a.firstEdgeVert + e * a.edgeLen + i];
      bool near = false;
      for (int f = 0; f < 4 && !near; ++f) {
        const int base = b.firstEdgeVert + f * b.edgeLen;
        for (int j = 0; j + 1 < b.edgeLen && !near; ++j)
          near = PointSegDist2(p, s_DispEdgeVert[base + j],
                               s_DispEdgeVert[base + j + 1]) <= tol2;
      }
      all = near;
    }
    if (all)
      return true;
  }
  return false;
}

// Are two displacements sewn along an edge? A whole edge of one lies on the
// other's ring: true for neighbours sharing a full edge at any power (a
// coarse edge's vertices land on the fine edge, a sewn fine edge's on the
// coarse segments) and for a smaller neighbour along part of a bigger one's
// edge. Not true for a neighbour that touches only at a corner, or whose
// shared edge was left unsewn anywhere along it: those stay obstacles.
bool DispsSewn(const DispInst_t &a, const DispInst_t &b) {
  return EdgeLiesOnRing(a, b) || EdgeLiesOnRing(b, a);
}

// The displacements the player can run into and see. The map loader drops a
// displacement's parent brush from the brush list (map.cpp), so the mesh is the
// only obstacle there is.
void BuildDisplacements() {
  CUtlVector<Box_t> boxes;
  for (int i = 0; i < nummapdispinfo; ++i) {
    const mapdispinfo_t &disp = mapdispinfo[i];
    // face.contents is the contents of the parent brush.
    if (!IsSolidContents(disp.face.contents))
      continue;
    const side_t *side = disp.face.originalface;
    if (!side || (side->surf & kInvisibleSurf))
      continue;
    AddDisplacement(i);
  }
  boxes.SetCount(s_Disp.Count());
  for (int i = 0; i < s_Disp.Count(); ++i)
    boxes[i] = s_Disp[i].box;
  s_DispGrid.Build(boxes, 256.0);

  // Sewn pairs, for the own-surface skip in DetailClip_Test.
  for (int i = 0; i < s_Disp.Count(); ++i) {
    Box_t grown = s_Disp[i].box;
    const V3 pad = MakeV3(kSewTolerance, kSewTolerance, kSewTolerance);
    grown.mn = grown.mn - pad;
    grown.mx = grown.mx + pad;
    s_DispGrid.Query(grown, [&](int j) {
      if (j > i && BoxesOverlap(grown, s_Disp[j].box) &&
          DispsSewn(s_Disp[i], s_Disp[j]))
        s_DispSewn.AddToTail(
            SewnKey(s_Disp[i].dispIndex, s_Disp[j].dispIndex));
      return false;
    });
  }
  std::sort(s_DispSewn.Base(), s_DispSewn.Base() + s_DispSewn.Count());
}

//-----------------------------------------------------------------------------
// The candidate: a shrunk bounding box cut by the spawn surface's tangent plane
//-----------------------------------------------------------------------------
struct Candidate_t {
  V3 verts[24];
  int nv;
  V3 faceAxis[4];
  int nfa;
  V3 edge[6];
  int ne;
  Box_t box;
};

PolyView_t CandidateView(const Candidate_t &c) {
  PolyView_t v;
  v.v = c.verts;
  v.nv = c.nv;
  v.faceAxis = c.faceAxis;
  v.nfa = c.nfa;
  v.edge = c.edge;
  v.ne = c.ne;
  return v;
}

// When nothing of the box is above the tangent plane (a world-upright box on a
// downward-facing surface, which stands up into the brush it hangs from, or a
// prop sunk below its spawn point), the whole uncut box is the candidate.
void BuildCandidate(const Vector &pt, const Vector &normal, const QAngle &angles,
                    const DetailClipShape_t &shape, Candidate_t &out) {
  const double keep = 1.0 - (double)g_flDetailClipShrink;

  V3 axis[3];
  double cx, cy, hx, hy;
  if (shape.yawAgnostic) {
    // Any yaw: a square wide enough for the farthest corner of the bounds.
    double r = 0;
    for (int c = 0; c < 4; ++c) {
      const double x = (c & 1) ? shape.maxs.x : shape.mins.x;
      const double y = (c & 2) ? shape.maxs.y : shape.mins.y;
      r = Max2(r, sqrt(x * x + y * y));
    }
    axis[0] = MakeV3(1, 0, 0);
    axis[1] = MakeV3(0, 1, 0);
    axis[2] = MakeV3(0, 0, 1);
    cx = cy = 0;
    hx = hy = r * keep;
  } else {
    matrix3x4_t m;
    AngleMatrix(angles, m);
    for (int a = 0; a < 3; ++a)
      axis[a] = MakeV3(m[0][a], m[1][a], m[2][a]);
    cx = 0.5 * ((double)shape.mins.x + shape.maxs.x);
    cy = 0.5 * ((double)shape.mins.y + shape.maxs.y);
    hx = 0.5 * ((double)shape.maxs.x - shape.mins.x) * keep;
    hy = 0.5 * ((double)shape.maxs.y - shape.mins.y) * keep;
  }
  // Horizontally shrunk about its centre line; vertically shrunk toward the
  // base, the end at the spawn point. Only the part above the spawn plane
  // counts (the tangent-plane cut below discards the rest), so the base is
  // clamped to local z = 0: a model whose bounds start below its origin would
  // otherwise shrink into a slab that lies wholly underground. A shape that
  // hangs wholly below its origin shrinks toward its top, the attached end.
  double z0, z1;
  if (shape.maxs.z > 0) {
    z0 = Max2((double)shape.mins.z, 0.0);
    z1 = z0 + ((double)shape.maxs.z - z0) * keep;
  } else {
    z1 = (double)shape.maxs.z;
    z0 = z1 - (z1 - (double)shape.mins.z) * keep;
  }
  const double cz = 0.5 * (z0 + z1);
  hx = Max2(hx, kMinHalfExtent);
  hy = Max2(hy, kMinHalfExtent);
  const double hz = Max2(0.5 * (z1 - z0), kMinHalfExtent);

  const V3 origin = FromVector(pt);
  const V3 center = origin + axis[0] * cx + axis[1] * cy + axis[2] * cz;

  V3 corner[8];
  for (int c = 0; c < 8; ++c) {
    corner[c] = center + axis[0] * ((c & 1) ? hx : -hx) +
                axis[1] * ((c & 2) ? hy : -hy) + axis[2] * ((c & 4) ? hz : -hz);
  }

  // Tangent plane of the spawn surface, lifted by kLift.
  V3 n = FromVector(normal);
  const double nlen = Length3(n);
  n = nlen > 1e-6 ? n * (1.0 / nlen) : MakeV3(0, 0, 1);
  const double planeD = Dot3(n, origin) + kLift;
  double s[8];
  for (int c = 0; c < 8; ++c)
    s[c] = Dot3(n, corner[c]) - planeD;

  out.nv = 0;
  out.box = EmptyBox();
  for (int c = 0; c < 8; ++c) {
    if (s[c] >= 0) {
      out.verts[out.nv++] = corner[c];
      BoxAdd(out.box, corner[c]);
    }
  }
  for (int c = 0; c < 8; ++c) {
    for (int bit = 1; bit <= 4; bit <<= 1) {
      if (c & bit)
        continue;
      const int o = c | bit;
      if ((s[c] < 0 && s[o] > 0) || (s[c] > 0 && s[o] < 0)) {
        const double t = s[c] / (s[c] - s[o]);
        const V3 p = corner[c] + (corner[o] - corner[c]) * t;
        out.verts[out.nv++] = p;
        BoxAdd(out.box, p);
      }
    }
  }
  out.nfa = 0;
  for (int a = 0; a < 3; ++a)
    out.faceAxis[out.nfa++] = axis[a];
  out.ne = 0;
  for (int a = 0; a < 3; ++a)
    out.edge[out.ne++] = axis[a];
  if (out.nv == 0) {
    for (int c = 0; c < 8; ++c) {
      out.verts[out.nv++] = corner[c];
      BoxAdd(out.box, corner[c]);
    }
    return;
  }

  out.faceAxis[out.nfa++] = n;
  // The cap polygon's edges run along (plane normal x box axis).
  for (int a = 0; a < 3; ++a) {
    V3 e = Cross3(n, axis[a]);
    const double len = Length3(e);
    if (len > 1e-6)
      out.edge[out.ne++] = e * (1.0 / len);
  }
}

//-----------------------------------------------------------------------------
// Candidate against one static prop instance
//-----------------------------------------------------------------------------
// Triangle against the candidate (both in the instance's model space).
bool TriangleOverlaps(const PolyView_t &cand, const Tri_t &t) {
  V3 v[3];
  for (int k = 0; k < 3; ++k)
    v[k] = MakeV3(t.v[k][0], t.v[k][1], t.v[k][2]);
  V3 e[3] = {v[1] - v[0], v[2] - v[1], v[0] - v[2]};
  V3 nrm = Cross3(e[0], e[1]);
  const double nl = Length3(nrm);
  if (nl < 1e-9)
    return false; // a sliver has no area to clip into
  nrm = nrm * (1.0 / nl);
  V3 ue[3];
  for (int k = 0; k < 3; ++k) {
    const double l = Length3(e[k]);
    ue[k] = l > 1e-9 ? e[k] * (1.0 / l) : MakeV3(0, 0, 0);
  }
  PolyView_t tri;
  tri.v = v;
  tri.nv = 3;
  tri.faceAxis = &nrm;
  tri.nfa = 1;
  tri.edge = ue;
  tri.ne = 3;
  return ConvexOverlap(cand, tri);
}

bool BvhOverlaps(const MeshInfo_t &m, const PolyView_t &cand, const Box_t &candBox) {
  int stack[64];
  int sp = 0;
  stack[sp++] = 0;
  while (sp > 0) {
    const BvhNode_t &node = m.nodes[stack[--sp]];
    if (node.mn[0] > candBox.mx.x || node.mx[0] < candBox.mn.x ||
        node.mn[1] > candBox.mx.y || node.mx[1] < candBox.mn.y ||
        node.mn[2] > candBox.mx.z || node.mx[2] < candBox.mn.z)
      continue;
    if (node.left < 0) {
      for (int i = node.first; i < node.first + node.count; ++i) {
        if (TriangleOverlaps(cand, m.tris[i]))
          return true;
      }
    } else if (sp + 2 <= 64) {
      stack[sp++] = node.left;
      stack[sp++] = node.right;
    }
  }
  return false;
}

// Re-expresses the candidate in the model space of inst (rotation only).
void CandidateToLocal(const Candidate_t &c, const matrix3x4_t &xf, Candidate_t &out) {
  auto toLocal = [&](const V3 &w) {
    const double dx = w.x - xf[0][3], dy = w.y - xf[1][3], dz = w.z - xf[2][3];
    return MakeV3(dx * xf[0][0] + dy * xf[1][0] + dz * xf[2][0],
                  dx * xf[0][1] + dy * xf[1][1] + dz * xf[2][1],
                  dx * xf[0][2] + dy * xf[1][2] + dz * xf[2][2]);
  };
  auto dirToLocal = [&](const V3 &w) {
    return MakeV3(w.x * xf[0][0] + w.y * xf[1][0] + w.z * xf[2][0],
                  w.x * xf[0][1] + w.y * xf[1][1] + w.z * xf[2][1],
                  w.x * xf[0][2] + w.y * xf[1][2] + w.z * xf[2][2]);
  };
  out.nv = c.nv;
  out.box = EmptyBox();
  for (int i = 0; i < c.nv; ++i) {
    out.verts[i] = toLocal(c.verts[i]);
    BoxAdd(out.box, out.verts[i]);
  }
  out.nfa = c.nfa;
  for (int i = 0; i < c.nfa; ++i)
    out.faceAxis[i] = dirToLocal(c.faceAxis[i]);
  out.ne = c.ne;
  for (int i = 0; i < c.ne; ++i)
    out.edge[i] = dirToLocal(c.edge[i]);
}

void FreeAll() {
  s_Brush.Purge();
  s_PlaneN.Purge();
  s_PlaneD.Purge();
  s_BrushVert.Purge();
  s_BrushAxis.Purge();
  s_BrushEdge.Purge();
  s_BrushGrid.Purge();
  s_Inst.Purge();
  s_InstGrid.Purge();
  for (int i = 0; i < s_Disp.Count(); ++i)
    delete s_Disp[i].mesh;
  s_Disp.Purge();
  s_DispGrid.Purge();
  s_DispEdgeVert.Purge();
  s_DispSewn.Purge();
  FreeModelDict(s_DetailModels);
  FreeModelDict(s_PropModels);
}

} // namespace

//-----------------------------------------------------------------------------
// Public interface
//-----------------------------------------------------------------------------
void DetailClip_Begin() {
  FreeAll();
  memset(&s_Count, 0, sizeof(s_Count));
  s_BadBrushes = 0;
  s_PropModelsLoaded = 0;
  const double start = Plat_FloatTime();
  BuildBrushes();
  BuildStaticProps();
  BuildDisplacements();
  s_Count.buildSeconds = Plat_FloatTime() - start;
  Msg("detailclip: testing against %d solid brushes, %d displacements (%d sewn pairs) and %d "
      "static props (%d models), shrink %.2f\n",
      s_Brush.Count(), s_Disp.Count(), s_DispSewn.Count(), s_Inst.Count(), s_PropModelsLoaded,
      g_flDetailClipShrink);
  if (s_BadBrushes) {
    Warning("Warning: detailclip: %d brushes left out (a side plane faces the "
            "wrong way for its winding).\n",
            s_BadBrushes);
  }
}

void DetailClip_End() {
  const int removed = s_Count.buried + s_Count.brush + s_Count.staticProp +
                      s_Count.displacement;
  Msg("detailclip: %d candidates, %d kept, %d removed: %d buried in solid, %d "
      "overlapping brushes, %d overlapping static props, %d overlapping "
      "displacements (%d had no known bounds and only got the buried test); "
      "%.0f ms to build the lookups, %.0f ms testing\n",
      s_Count.candidates, s_Count.candidates - removed, removed, s_Count.buried,
      s_Count.brush, s_Count.staticProp, s_Count.displacement, s_Count.noBounds,
      s_Count.buildSeconds * 1000.0, s_Count.testSeconds * 1000.0);
  FreeAll();
}

bool DetailClip_ModelShape(const char *pModelName, int nOrientation,
                           DetailClipShape_t &shape) {
  memset(&shape, 0, sizeof(shape));
  char key[MAX_PATH];
  NormalizeModelKey(pModelName, key, sizeof(key));
  int d = s_DetailModels.Find(key);
  if (d == s_DetailModels.InvalidIndex()) {
    MeshInfo_t *m = new MeshInfo_t;
    LoadMeshInfo(pModelName, false, *m);
    d = s_DetailModels.Insert(key, m);
  }
  const MeshInfo_t &m = *s_DetailModels[d];

  V3 mn, mx;
  if (m.hasMesh && BoxHasExtent(m.meshBox.mn, m.meshBox.mx)) {
    mn = m.meshBox.mn;
    mx = m.meshBox.mx;
  } else if (m.hasHull) {
    mn = m.hullMin;
    mx = m.hullMax;
  } else if (m.hasView) {
    mn = m.viewMin;
    mx = m.viewMax;
  } else {
    return false;
  }
  shape.boundsKnown = true;
  shape.yawAgnostic = (nOrientation != DETAIL_PROP_ORIENT_NORMAL);
  shape.mins.Init((float)mn.x, (float)mn.y, (float)mn.z);
  shape.maxs.Init((float)mx.x, (float)mx.y, (float)mx.z);
  return true;
}

namespace {
// Adds the time its scope took to the test total.
struct TestTimer_t {
  double start;
  TestTimer_t() : start(Plat_FloatTime()) {}
  ~TestTimer_t() { s_Count.testSeconds += Plat_FloatTime() - start; }
};
} // namespace

DetailClipVerdict_t DetailClip_Test(const Vector &pt, const Vector &normal,
                                    const QAngle &angles,
                                    const DetailClipShape_t &shape,
                                    int nDispInfo) {
  TestTimer_t timer;
  ++s_Count.candidates;

  V3 n = FromVector(normal);
  const double nlen = Length3(n);
  n = nlen > 1e-6 ? n * (1.0 / nlen) : MakeV3(0, 0, 1);

  // Buried: the probe point inside a brush.
  const V3 probe = FromVector(pt) + n * kProbeHeight;
  Box_t probeBox;
  probeBox.mn = probeBox.mx = probe;
  const bool buried = s_BrushGrid.Query(probeBox, [&](int id) {
    const ClipBrush_t &cb = s_Brush[id];
    return BoxesOverlap(cb.box, probeBox) && PointInsideBrush(cb, probe);
  });
  if (buried) {
    ++s_Count.buried;
    return DETAILCLIP_BURIED;
  }

  if (!shape.boundsKnown) {
    ++s_Count.noBounds;
    return DETAILCLIP_KEEP;
  }

  Candidate_t cand;
  BuildCandidate(pt, normal, angles, shape, cand);
  const PolyView_t candView = CandidateView(cand);

  const bool hitBrush = s_BrushGrid.Query(cand.box, [&](int id) {
    const ClipBrush_t &cb = s_Brush[id];
    return BoxesOverlap(cb.box, cand.box) &&
           ConvexOverlap(candView, BrushView(cb));
  });
  if (hitBrush) {
    ++s_Count.brush;
    return DETAILCLIP_BRUSH;
  }

  const bool hitProp = s_InstGrid.Query(cand.box, [&](int id) {
    const PropInst_t &inst = s_Inst[id];
    if (!BoxesOverlap(inst.box, cand.box))
      return false;
    Candidate_t local;
    CandidateToLocal(cand, inst.xf, local);
    return BvhOverlaps(*inst.mesh, CandidateView(local), local.box);
  });
  if (hitProp) {
    ++s_Count.staticProp;
    return DETAILCLIP_STATICPROP;
  }

  // Last, so a candidate that a brush or static prop also catches keeps its
  // earlier verdict. The displacement it sits on is skipped, and so are those
  // sewn to it: that continuous surface is what the tangent plane stands for.
  const bool hitDisp = s_DispGrid.Query(cand.box, [&](int id) {
    const DispInst_t &d = s_Disp[id];
    return d.dispIndex != nDispInfo && BoxesOverlap(d.box, cand.box) &&
           !DispSewn(d.dispIndex, nDispInfo) &&
           BvhOverlaps(*d.mesh, candView, cand.box);
  });
  if (hitDisp) {
    ++s_Count.displacement;
    return DETAILCLIP_DISPLACEMENT;
  }
  return DETAILCLIP_KEEP;
}
