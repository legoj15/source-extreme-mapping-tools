//========= UV overlap detector shared by VBSP and VRAD (issue #56) ===========
// See uvoverlap.h for the rules.
//=============================================================================
#include "uvoverlap.h"
#include "tier0/dbg.h"
#include "tier1/strtools.h"
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace {

struct IslandKey_t {
  float pos[3];
  float uv[2];
  int sign;
  int vert; // not part of the key
};

struct IslandEdge_t {
  int a, b; // key ids, a < b
  int tri;
};

// One claim on a folded cell: the triangle and the tile it was rasterized in.
struct Claim_t {
  int tri;
  int tx, ty;
};

struct ExtraClaim_t {
  int cell;
  Claim_t c;
};

int FindSet(CUtlVector<int> &parent, int x) {
  while (parent[x] != x) {
    parent[x] = parent[parent[x]];
    x = parent[x];
  }
  return x;
}

void UnionSets(CUtlVector<int> &parent, int a, int b) {
  a = FindSet(parent, a);
  b = FindSet(parent, b);
  if (a == b)
    return;
  if (a < b)
    parent[b] = a;
  else
    parent[a] = b;
}

int CompareKeyNoVert(const IslandKey_t *a, const IslandKey_t *b) {
  return memcmp(a, b, offsetof(IslandKey_t, vert));
}

int __cdecl CompareKey(const void *pa, const void *pb) {
  const IslandKey_t *a = (const IslandKey_t *)pa;
  const IslandKey_t *b = (const IslandKey_t *)pb;
  const int c = CompareKeyNoVert(a, b);
  return c ? c : a->vert - b->vert;
}

int __cdecl CompareEdge(const void *pa, const void *pb) {
  const IslandEdge_t *a = (const IslandEdge_t *)pa;
  const IslandEdge_t *b = (const IslandEdge_t *)pb;
  if (a->a != b->a)
    return a->a < b->a ? -1 : 1;
  if (a->b != b->b)
    return a->b < b->b ? -1 : 1;
  return a->tri - b->tri;
}

int __cdecl CompareExtra(const void *pa, const void *pb) {
  const ExtraClaim_t *a = (const ExtraClaim_t *)pa;
  const ExtraClaim_t *b = (const ExtraClaim_t *)pb;
  if (a->cell != b->cell)
    return a->cell < b->cell ? -1 : 1;
  return a->c.tri - b->c.tri;
}

// One claim on a cell, in the order the classification reads them.
struct SortedClaim_t {
  int isl, ty, tx, tri;
};

int __cdecl CompareSorted(const void *pa, const void *pb) {
  const SortedClaim_t *a = (const SortedClaim_t *)pa;
  const SortedClaim_t *b = (const SortedClaim_t *)pb;
  if (a->isl != b->isl)
    return a->isl < b->isl ? -1 : 1;
  if (a->ty != b->ty)
    return a->ty < b->ty ? -1 : 1;
  if (a->tx != b->tx)
    return a->tx < b->tx ? -1 : 1;
  return a->tri < b->tri ? -1 : (a->tri > b->tri ? 1 : 0);
}

// Edge function of the directed edge a -> b at c (positive on the left),
// evaluated with the endpoints in a fixed order so that the two triangles of
// a shared edge get exactly opposite values.
inline double EdgeFn(const double *a, const double *b, const double *c) {
  const bool fwd = a[0] < b[0] || (a[0] == b[0] && a[1] < b[1]);
  const double *A = fwd ? a : b, *B = fwd ? b : a;
  const double f = (B[0] - A[0]) * (c[1] - A[1]) - (B[1] - A[1]) * (c[0] - A[0]);
  return fwd ? f : -f;
}

// Tie rule for a center exactly on a counter-clockwise edge with direction d:
// the center counts as nudged by (-1, -eta), i.e. inside when d points up, or
// exactly sideways towards -x. Exactly one of d and -d passes, and a fan of
// triangles around a shared corner gives that corner to exactly one of them.
inline bool OwnsEdge(double dx, double dy) {
  return dy > 0.0 || (dy == 0.0 && dx < 0.0);
}

inline int FloorDiv(int a, int b) {
  return a >= 0 ? a / b : -((-a + b - 1) / b);
}

} // namespace

int UvOverlap_BuildIslands(const UvOverlapVert_t *pVerts, int nVerts,
                           const UvOverlapTri_t *pTris, int nTris,
                           CUtlVector<int> &islandOfTri) {
  islandOfTri.SetCount(nTris);
  if (!nTris)
    return 0;
  CUtlVector<int> parent;
  parent.SetCount(nVerts);
  for (int i = 0; i < nVerts; ++i)
    parent[i] = i;
  CUtlVector<unsigned char> used;
  used.SetCount(nVerts);
  memset(used.Base(), 0, nVerts);
  for (int t = 0; t < nTris; ++t) {
    UnionSets(parent, pTris[t].vert[0], pTris[t].vert[1]);
    UnionSets(parent, pTris[t].vert[0], pTris[t].vert[2]);
    for (int c = 0; c < 3; ++c)
      used[pTris[t].vert[c]] = 1;
  }
  // Key ids: vertices equal bit for bit in position, UV and handedness.
  CUtlVector<IslandKey_t> keys;
  for (int v = 0; v < nVerts; ++v) {
    if (!used[v])
      continue;
    IslandKey_t &k = keys[keys.AddToTail()];
    memset(&k, 0, sizeof(k));
    memcpy(k.pos, pVerts[v].pos, sizeof(k.pos));
    memcpy(k.uv, pVerts[v].uv, sizeof(k.uv));
    k.sign = pVerts[v].sign < 0 ? -1 : 1;
    k.vert = v;
  }
  qsort(keys.Base(), keys.Count(), sizeof(IslandKey_t), CompareKey);
  CUtlVector<int> keyId;
  keyId.SetCount(nVerts);
  for (int v = 0; v < nVerts; ++v)
    keyId[v] = -1;
  for (int i = 0, id = -1; i < keys.Count(); ++i) {
    if (!i || CompareKeyNoVert(&keys[i - 1], &keys[i]))
      ++id;
    keyId[keys[i].vert] = id;
  }
  // Across a hard edge, two triangles join only when they share a whole edge.
  CUtlVector<IslandEdge_t> edges;
  edges.EnsureCapacity(nTris * 3);
  for (int t = 0; t < nTris; ++t) {
    for (int e = 0; e < 3; ++e) {
      const int ka = keyId[pTris[t].vert[e]];
      const int kb = keyId[pTris[t].vert[(e + 1) % 3]];
      if (ka == kb)
        continue;
      IslandEdge_t &ref = edges[edges.AddToTail()];
      ref.a = MIN(ka, kb);
      ref.b = MAX(ka, kb);
      ref.tri = t;
    }
  }
  qsort(edges.Base(), edges.Count(), sizeof(IslandEdge_t), CompareEdge);
  for (int i = 1; i < edges.Count(); ++i) {
    if (edges[i].a == edges[i - 1].a && edges[i].b == edges[i - 1].b)
      UnionSets(parent, pTris[edges[i - 1].tri].vert[0],
                pTris[edges[i].tri].vert[0]);
  }
  CUtlVector<int> islandOfRoot;
  islandOfRoot.SetCount(nVerts);
  for (int v = 0; v < nVerts; ++v)
    islandOfRoot[v] = -1;
  int count = 0;
  for (int t = 0; t < nTris; ++t) {
    const int root = FindSet(parent, pTris[t].vert[0]);
    if (islandOfRoot[root] < 0)
      islandOfRoot[root] = count++;
    islandOfTri[t] = islandOfRoot[root];
  }
  return count;
}

// Total cells over every triangle's bounding box that one measure may visit.
// Time: cells tested, summed over the triangles' bounding boxes.
#define UVOVERLAP_MAX_RASTER_CELLS ((int64)1 << 28)
// Memory: every claim past a cell's first is stored (16 bytes), so at most
// 1 GB. The rebake's worst case (16 x 16 repeats at 512 cells per unit) stores
// 2^26 - 2^18.
#define UVOVERLAP_MAX_EXTRA_CLAIMS (1 << 26)
#define UVOVERLAP_MAX_CELL_INDEX ((double)(1 << 30))

bool UvOverlap_Measure(const UvOverlapVert_t *pVerts, int nVerts,
                       const UvOverlapTri_t *pTris, int nTris, int resX,
                       int resY, int maxRepeats, UvOverlapResult_t &r,
                       char *pErr, int errLen) {
  for (int t = 0; t < nTris; ++t) {
    for (int c = 0; c < 3; ++c) {
      const int v = pTris[t].vert[c];
      if (v < 0 || v >= nVerts) {
        V_snprintf(pErr, errLen, "triangle %d names vertex %d of %d", t, v,
                   nVerts);
        return false;
      }
    }
  }
  CUtlVector<int> islandOfTri;
  const int nIsl =
      UvOverlap_BuildIslands(pVerts, nVerts, pTris, nTris, islandOfTri);
  return UvOverlap_MeasureIslands(pVerts, nVerts, pTris, nTris,
                                  islandOfTri.Base(), nIsl, resX, resY,
                                  maxRepeats, r, pErr, errLen);
}

bool UvOverlap_MeasureIslands(const UvOverlapVert_t *pVerts, int nVerts,
                              const UvOverlapTri_t *pTris, int nTris,
                              const int *pIslandOfTri, int nIsl, int resX,
                              int resY, int maxRepeats, UvOverlapResult_t &r,
                              char *pErr, int errLen) {
  memset(&r, 0, offsetof(UvOverlapResult_t, islandOfTri));
  r.resX = resX;
  r.resY = resY;
  r.islandOfTri.RemoveAll();
  r.islands.RemoveAll();
  if (resX < 1 || resY < 1 || resX > 8192 || resY > 8192) {
    V_snprintf(pErr, errLen, "grid %dx%d is outside 1..8192", resX, resY);
    return false;
  }
  for (int t = 0; t < nTris; ++t) {
    for (int c = 0; c < 3; ++c) {
      const int v = pTris[t].vert[c];
      if (v < 0 || v >= nVerts) {
        V_snprintf(pErr, errLen, "triangle %d names vertex %d of %d", t, v,
                   nVerts);
        return false;
      }
      if (!isfinite(pVerts[v].uv[0]) || !isfinite(pVerts[v].uv[1])) {
        V_snprintf(pErr, errLen, "vertex %d has a non-finite UV", v);
        return false;
      }
    }
    if (pIslandOfTri[t] < 0 || pIslandOfTri[t] >= nIsl) {
      V_snprintf(pErr, errLen, "triangle %d is in island %d of %d", t,
                 pIslandOfTri[t], nIsl);
      return false;
    }
  }
  r.islandOfTri.SetCount(nTris);
  if (nTris)
    memcpy(r.islandOfTri.Base(), pIslandOfTri, nTris * sizeof(int));
  r.islands.SetCount(nIsl);
  for (int i = 0; i < nIsl; ++i) {
    UvOverlapIsland_t &isl = r.islands[i];
    memset(&isl, 0, sizeof(isl));
    isl.mn[0] = isl.mn[1] = 1e30;
    isl.mx[0] = isl.mx[1] = -1e30;
  }

  const int nCells = resX * resY;
  CUtlVector<Claim_t> first;
  first.SetCount(nCells);
  for (int i = 0; i < nCells; ++i)
    first[i].tri = -1;
  CUtlVector<ExtraClaim_t> extras;
  int64 work = 0;

  for (int t = 0; t < nTris; ++t) {
    UvOverlapIsland_t &isl = r.islands[r.islandOfTri[t]];
    ++isl.tris;
    double p[3][2];
    double uvMn[2] = {1e30, 1e30}, uvMx[2] = {-1e30, -1e30};
    for (int c = 0; c < 3; ++c) {
      const float *uv = pVerts[pTris[t].vert[c]].uv;
      p[c][0] = (double)uv[0] * resX; // exact: float times a power below 2^14
      p[c][1] = (double)uv[1] * resY;
      for (int k = 0; k < 2; ++k) {
        uvMn[k] = MIN(uvMn[k], (double)uv[k]);
        uvMx[k] = MAX(uvMx[k], (double)uv[k]);
        isl.mn[k] = MIN(isl.mn[k], (double)uv[k]);
        isl.mx[k] = MAX(isl.mx[k], (double)uv[k]);
      }
    }
    if (uvMx[0] - uvMn[0] > maxRepeats || uvMx[1] - uvMn[1] > maxRepeats) {
      V_snprintf(pErr, errLen,
                 "triangle %d spans more than %d UV tiles (%g x %g)", t,
                 maxRepeats, uvMx[0] - uvMn[0], uvMx[1] - uvMn[1]);
      return false;
    }
    const double area = EdgeFn(p[0], p[1], p[2]);
    if (area == 0.0) {
      ++r.degenerateTris;
      continue;
    }
    if (uvMx[0] - uvMn[0] > 1.0 || uvMx[1] - uvMn[1] > 1.0) {
      ++isl.spanningTris;
      ++r.spanningTris;
    }
    if (area < 0.0) {
      for (int k = 0; k < 2; ++k) {
        const double tmp = p[1][k];
        p[1][k] = p[2][k];
        p[2][k] = tmp;
      }
    }
    const double minX = MIN(MIN(p[0][0], p[1][0]), p[2][0]);
    const double maxX = MAX(MAX(p[0][0], p[1][0]), p[2][0]);
    const double minY = MIN(MIN(p[0][1], p[1][1]), p[2][1]);
    const double maxY = MAX(MAX(p[0][1], p[1][1]), p[2][1]);
    // Cell indices are ints: a UV this far out would overflow the casts below.
    if (minX < -UVOVERLAP_MAX_CELL_INDEX || maxX > UVOVERLAP_MAX_CELL_INDEX ||
        minY < -UVOVERLAP_MAX_CELL_INDEX || maxY > UVOVERLAP_MAX_CELL_INDEX) {
      V_snprintf(pErr, errLen,
                 "triangle %d has UVs too far from the origin to measure at "
                 "%dx%d cells per unit (%g %g .. %g %g)",
                 t, resX, resY, uvMn[0], uvMn[1], uvMx[0], uvMx[1]);
      return false;
    }
    const int x0 = (int)floor(minX - 0.5), x1 = (int)ceil(maxX - 0.5);
    const int y0 = (int)floor(minY - 0.5), y1 = (int)ceil(maxY - 0.5);
    work += (int64)(x1 - x0 + 1) * (int64)(y1 - y0 + 1);
    if (work > UVOVERLAP_MAX_RASTER_CELLS) {
      V_snprintf(pErr, errLen,
                 "the UV layout is too large to measure at %dx%d cells per "
                 "unit",
                 resX, resY);
      return false;
    }
    bool own[3];
    for (int e = 0; e < 3; ++e) {
      const double *a = p[e], *b = p[(e + 1) % 3];
      own[e] = OwnsEdge(b[0] - a[0], b[1] - a[1]);
    }
    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        const double c[2] = {x + 0.5, y + 0.5};
        bool inside = true;
        for (int e = 0; e < 3 && inside; ++e) {
          const double ev = EdgeFn(p[e], p[(e + 1) % 3], c);
          inside = ev > 0.0 || (ev == 0.0 && own[e]);
        }
        if (!inside)
          continue;
        const int tx = FloorDiv(x, resX), ty = FloorDiv(y, resY);
        const int cell = (y - ty * resY) * resX + (x - tx * resX);
        Claim_t claim;
        claim.tri = t;
        claim.tx = tx;
        claim.ty = ty;
        if (first[cell].tri < 0) {
          first[cell] = claim;
        } else {
          if (extras.Count() >= UVOVERLAP_MAX_EXTRA_CLAIMS) {
            V_snprintf(pErr, errLen,
                       "the UV layout overlaps too much to measure at %dx%d "
                       "cells per unit",
                       resX, resY);
            return false;
          }
          ExtraClaim_t &ex = extras[extras.AddToTail()];
          ex.cell = cell;
          ex.c = claim;
        }
      }
    }
  }

  // Cells with one claim.
  CUtlVector<unsigned char> multi;
  multi.SetCount(nCells);
  memset(multi.Base(), 0, nCells);
  for (int i = 0; i < extras.Count(); ++i)
    multi[extras[i].cell] = 1;
  for (int cell = 0; cell < nCells; ++cell) {
    if (first[cell].tri < 0)
      continue;
    ++r.cells;
    if (!multi[cell])
      ++r.islands[r.islandOfTri[first[cell].tri]].cells;
  }

  // Cells claimed more than once.
  // Each cell's claims are sorted by (island, tile, triangle), so one pass
  // over the runs classifies them: an island run with two tiles is a repeat,
  // a tile run with two triangles is a fold.
  qsort(extras.Base(), extras.Count(), sizeof(ExtraClaim_t), CompareExtra);
  CUtlVector<SortedClaim_t> list;
  for (int i = 0; i < extras.Count();) {
    const int cell = extras[i].cell;
    list.RemoveAll();
    const Claim_t &c0 = first[cell];
    SortedClaim_t s0 = {r.islandOfTri[c0.tri], c0.ty, c0.tx, c0.tri};
    list.AddToTail(s0);
    for (; i < extras.Count() && extras[i].cell == cell; ++i) {
      const Claim_t &c = extras[i].c;
      SortedClaim_t s = {r.islandOfTri[c.tri], c.ty, c.tx, c.tri};
      list.AddToTail(s);
    }
    qsort(list.Base(), list.Count(), sizeof(SortedClaim_t), CompareSorted);
    bool anyFold = false, anyRepeat = false;
    const bool other = list[0].isl != list[list.Count() - 1].isl;
    for (int a = 0; a < list.Count();) {
      int b = a + 1;
      bool fold = false, repeat = false;
      for (; b < list.Count() && list[b].isl == list[a].isl; ++b) {
        const SortedClaim_t &p = list[b - 1], &q = list[b];
        if (q.tx != list[a].tx || q.ty != list[a].ty)
          repeat = true;
        if (q.tx == p.tx && q.ty == p.ty && q.tri != p.tri)
          fold = true;
      }
      UvOverlapIsland_t &isl = r.islands[list[a].isl];
      ++isl.cells;
      if (fold) {
        ++isl.foldCells;
        anyFold = true;
      }
      if (repeat) {
        ++isl.repeatCells;
        anyRepeat = true;
      }
      if (other)
        ++isl.otherCells;
      a = b;
    }
    r.foldCells += anyFold;
    r.repeatCells += anyRepeat;
    r.otherCells += other;
    r.movableCells += (anyRepeat || other);
    r.overlapCells += (anyFold || anyRepeat || other);
  }
  r.fraction = r.cells ? (double)r.overlapCells / (double)r.cells : 0.0;
  return true;
}

void UvOverlap_Print(const char *pName, const UvOverlapResult_t &r,
                     int maxIslands) {
  Msg("uvoverlap: %s: %d triangles, %d islands, grid %dx%d per UV unit, %d "
      "cells covered, %d overlapped (%.4f%%)\n",
      pName, r.islandOfTri.Count(), r.islands.Count(), r.resX, r.resY, r.cells,
      r.overlapCells, 100.0 * r.fraction);
  Msg("uvoverlap: %s: fold %d, repeat %d, other %d cells; %d triangles span "
      "more than one tile; %d zero-area triangles\n",
      pName, r.foldCells, r.repeatCells, r.otherCells, r.spanningTris,
      r.degenerateTris);
  // Worst islands first: by overlapped cells, then index.
  CUtlVector<int> order;
  for (int i = 0; i < r.islands.Count(); ++i) {
    const UvOverlapIsland_t &isl = r.islands[i];
    if (isl.foldCells || isl.repeatCells || isl.otherCells)
      order.AddToTail(i);
  }
  for (int a = 1; a < order.Count(); ++a) {
    const int key = order[a];
    const UvOverlapIsland_t &k = r.islands[key];
    const int kv = k.foldCells + k.repeatCells + k.otherCells;
    int b = a - 1;
    for (; b >= 0; --b) {
      const UvOverlapIsland_t &o = r.islands[order[b]];
      if (o.foldCells + o.repeatCells + o.otherCells >= kv)
        break;
      order[b + 1] = order[b];
    }
    order[b + 1] = key;
  }
  for (int n = 0; n < order.Count() && n < maxIslands; ++n) {
    const UvOverlapIsland_t &isl = r.islands[order[n]];
    Msg("uvoverlap: %s: island %d: %d triangles, %d cells, fold %d, repeat %d, "
        "other %d, spanning %d, uv %g %g .. %g %g\n",
        pName, order[n], isl.tris, isl.cells, isl.foldCells, isl.repeatCells,
        isl.otherCells, isl.spanningTris, isl.mn[0], isl.mn[1], isl.mx[0],
        isl.mx[1]);
  }
  if (order.Count() > maxIslands)
    Msg("uvoverlap: %s: %d more overlapping islands not listed\n", pName,
        order.Count() - maxIslands);
}

bool UvOverlap_ReadTextMesh(const char *pPath,
                            CUtlVector<UvOverlapVert_t> &verts,
                            CUtlVector<UvOverlapTri_t> &tris, char *pErr,
                            int errLen) {
  FILE *fp = fopen(pPath, "rt");
  if (!fp) {
    V_snprintf(pErr, errLen, "cannot open %s", pPath);
    return false;
  }
  char line[512];
  int lineNo = 0;
  bool ok = true;
  while (ok && fgets(line, sizeof(line), fp)) {
    ++lineNo;
    char *p = line;
    while (*p == ' ' || *p == '\t')
      ++p;
    if (!*p || *p == '#' || *p == '\n' || *p == '\r')
      continue;
    if (p[0] == 'v' && (p[1] == ' ' || p[1] == '\t')) {
      UvOverlapVert_t v;
      v.sign = 1;
      const int n = sscanf(p + 2, "%f %f %f %f %f %d", &v.pos[0], &v.pos[1],
                           &v.pos[2], &v.uv[0], &v.uv[1], &v.sign);
      if (n < 5 || (v.sign != 1 && v.sign != -1)) {
        V_snprintf(pErr, errLen, "%s:%d: bad vertex line", pPath, lineNo);
        ok = false;
      } else
        verts.AddToTail(v);
    } else if (p[0] == 'f' && (p[1] == ' ' || p[1] == '\t')) {
      UvOverlapTri_t t;
      if (sscanf(p + 2, "%d %d %d", &t.vert[0], &t.vert[1], &t.vert[2]) != 3) {
        V_snprintf(pErr, errLen, "%s:%d: bad face line", pPath, lineNo);
        ok = false;
      } else
        tris.AddToTail(t);
    } else {
      V_snprintf(pErr, errLen, "%s:%d: unknown line", pPath, lineNo);
      ok = false;
    }
  }
  fclose(fp);
  for (int t = 0; ok && t < tris.Count(); ++t) {
    for (int c = 0; c < 3; ++c) {
      if (tris[t].vert[c] < 0 || tris[t].vert[c] >= verts.Count()) {
        V_snprintf(pErr, errLen, "%s: face %d names vertex %d of %d", pPath, t,
                   tris[t].vert[c], verts.Count());
        ok = false;
        break;
      }
    }
  }
  return ok;
}
