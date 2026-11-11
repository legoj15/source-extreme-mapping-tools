//========= Copyright Valve Corporation, All rights reserved. ============//
//
// Purpose: the merged-window portal flow behind "vvis -mergeflow".
//
// The exact flow (flow.cpp) enumerates portal chains, which takes minutes to
// never on large open maps. This flow keeps at most N (source window, pass
// window) states per portal and merges every further arrival into the cheapest
// one, so it is polynomial. Each merged state is a convex superset of what was
// folded into it, and every plane ClipToSeperators accepts separates the whole
// source window from the whole pass window, so in exact arithmetic no straight
// sight line through a chain of portals is lost: the result is a superset of
// TRUE visibility. It is not bit-identical to the exact flow and is not a
// superset of its result either: ClipToSeperators is not monotone (a sliver
// window yields no separating plane and leaves the target unclipped), so the
// per-chain flow can mark a portal that a fatter merged window clips away.
//
// All of the geometry here is double, deliberately. With float (vec_t) the
// flow loses real sight lines on sliver portals and grazing angles at
// 16k-unit coordinates (issue #89); the double build lost none in 1.9e9 rays.
// The portals therefore come from LoadPortals as the parsed doubles
// (g_pMergePortals), not from the float windings.
//
// With a vis radius (-radius_override, or an env_fog_controller farz) the
// flood also applies stock's radius test, which narrows the result exactly as
// it narrows the default flow's. The prototype has no such test, so the
// bit-identity below and the superset statement above are for maps without one.
//
// experiments/prtvis is the measuring tool for this algorithm (bounds, ray
// check, comparison with a compiled BSP). Changes here should be measured
// there first; its double build (prtvis_d) must stay bit-identical to this.
// That holds at the same state count (prtvis --k defaults to 1, -mergestates
// to 4), without prtvis's --might / --reuse, on a map with no vis radius.
//
// $NoKeywords: $
//=============================================================================//
#include "vis.h"
#include "threads.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <utility>
#include <vector>
#if defined(_MSC_VER)
#include <intrin.h>
#endif

// The project builds with /fp:fast. Reassociation or FMA contraction would
// change the rounding this code is validated against, so pin it for the file.
#pragma float_control(precise, on, push)
#pragma fp_contract(off)

// The vis.h bit macros (SetBit, CheckBit, ClearBit) work on bytes; the helpers
// below work on 64-bit words, so they carry other names.

namespace {

typedef uint64_t u64;

const double VIS_EPS = 0.01;    // ON_VIS_EPSILON
const double CONTAIN_EPS = 1e-6; // slack of the dominance test in Contains
enum { kFront = 0, kBack = 1, kOn = 2 };

// Winding capacity: the most points LoadPortals accepts in one portal, so the
// mode takes every portal file stock does (stock's own stack windings hold 12).
// The hull scratch arrays below are all sized from it.
const int MAXW = MAX_POINTS_ON_WINDING;
const int HULL_MAX = 24;         // larger hulls become a clipped rectangle
const int GUARD_UPDATES = 256;   // merges into one portal's states, per base
const int GUARD_ARRIVALS = 65536; // backstop for creation-only creep

// ---------------------------------------------------------------------------
// bit helpers

inline bool BitGet(const u64 *r, int i) { return (r[i >> 6] >> (i & 63)) & 1; }
inline void BitSet(u64 *r, int i) { r[i >> 6] |= 1ull << (i & 63); }
// No __popcnt64: it emits the POPCNT instruction unconditionally, which is not
// baseline x64, and this only feeds the reported totals.
inline int Pop(u64 x) {
  x = x - ((x >> 1) & 0x5555555555555555ull);
  x = (x & 0x3333333333333333ull) + ((x >> 2) & 0x3333333333333333ull);
  x = (x + (x >> 4)) & 0x0f0f0f0f0f0f0f0full;
  return (int)((x * 0x0101010101010101ull) >> 56);
}

struct BitMat {
  size_t rows = 0, W = 0;
  std::vector<u64> d;
  void init(size_t r, size_t bits) {
    rows = r;
    W = (bits + 63) / 64;
    d.assign(r * W, 0);
  }
  u64 *row(size_t i) { return d.data() + i * W; }
  const u64 *row(size_t i) const { return d.data() + i * W; }
};

long long CountRow(const u64 *r, size_t W) {
  long long c = 0;
  for (size_t i = 0; i < W; i++)
    c += Pop(r[i]);
  return c;
}

// ---------------------------------------------------------------------------
// geometry, all double

struct V3 {
  double x, y, z;
  double &operator[](int i) { return (&x)[i]; }
  double operator[](int i) const { return (&x)[i]; }
};
inline V3 operator-(const V3 &a, const V3 &b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
inline V3 operator+(const V3 &a, const V3 &b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}
inline V3 operator*(const V3 &a, double s) {
  return {a.x * s, a.y * s, a.z * s};
}
inline double Dot(const V3 &a, const V3 &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline V3 Cross(const V3 &a, const V3 &b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

struct Plane {
  V3 n;
  double d;
};

struct Winding {
  int n;
  V3 p[MAXW];
};

inline void Wcopy(Winding &dst, const Winding &src) {
  dst.n = src.n;
  memcpy(dst.p, src.p, (size_t)src.n * sizeof(V3));
}

// Read-only view over a polygon held either as a Winding or as a vector.
struct WRef {
  const V3 *p;
  int n;
  WRef(const Winding &w) : p(w.p), n(w.n) {}
  WRef(const std::vector<V3> &v) : p(v.data()), n((int)v.size()) {}
};

void ToWinding(const std::vector<V3> &v, Winding &w) {
  w.n = (int)v.size();
  memcpy(w.p, v.data(), v.size() * sizeof(V3));
}

// ChopWinding, in place: keeps the part in front of split (+/- VIS_EPS).
// False when nothing is in front. On overflow w is left unchanged, which is the
// stock conservative fallback.
bool Chop(Winding &w, const Plane &split) {
  double dists[MAXW + 1];
  int sides[MAXW + 1];
  int counts[3] = {0, 0, 0};

  for (int i = 0; i < w.n; i++) {
    double dot = Dot(w.p[i], split.n);
    dot -= split.d;
    dists[i] = dot;
    if (dot > VIS_EPS)
      sides[i] = kFront;
    else if (dot < -VIS_EPS)
      sides[i] = kBack;
    else
      sides[i] = kOn;
    counts[sides[i]]++;
  }
  if (!counts[1])
    return true;
  if (!counts[0])
    return false;

  sides[w.n] = sides[0];
  dists[w.n] = dists[0];

  Winding neww;
  neww.n = 0;
  for (int i = 0; i < w.n; i++) {
    const V3 &p1 = w.p[i];
    if (neww.n == MAXW)
      return true;
    if (sides[i] == kOn) {
      neww.p[neww.n++] = p1;
      continue;
    }
    if (sides[i] == kFront)
      neww.p[neww.n++] = p1;
    if (sides[i + 1] == kOn || sides[i + 1] == sides[i])
      continue;
    if (neww.n == MAXW)
      return true;

    const V3 &p2 = w.p[(i + 1) % w.n];
    double dot = dists[i] / (dists[i] - dists[i + 1]);
    V3 mid;
    for (int j = 0; j < 3; j++) { // avoid round off error when possible
      if (split.n[j] == 1)
        mid[j] = split.d;
      else if (split.n[j] == -1)
        mid[j] = -split.d;
      else
        mid[j] = p1[j] + dot * (p2[j] - p1[j]);
    }
    neww.p[neww.n++] = mid;
  }
  Wcopy(w, neww);
  return true;
}

// ClipToSeperators, clipping target in place. False when target is clipped
// away.
bool ClipToSeperators(const Winding &source, const Winding &pass,
                      Winding &target, bool flipclip) {
  for (int i = 0; i < source.n; i++) {
    int l = (i + 1) % source.n;
    V3 v1 = source.p[l] - source.p[i];

    for (int j = 0; j < pass.n; j++) {
      V3 v2 = pass.p[j] - source.p[i];
      Plane plane;
      plane.n.x = v1.y * v2.z - v1.z * v2.y;
      plane.n.y = v1.z * v2.x - v1.x * v2.z;
      plane.n.z = v1.x * v2.y - v1.y * v2.x;

      double length =
          plane.n.x * plane.n.x + plane.n.y * plane.n.y + plane.n.z * plane.n.z;
      if (length < VIS_EPS)
        continue;

      length = 1.0 / std::sqrt(length);
      plane.n.x *= length;
      plane.n.y *= length;
      plane.n.z *= length;
      plane.d = Dot(pass.p[j], plane.n);

      bool fliptest = false;
      int k;
      for (k = 0; k < source.n; k++) {
        if (k == i || k == l)
          continue;
        double d = Dot(source.p[k], plane.n) - plane.d;
        if (d < -VIS_EPS) {
          fliptest = false;
          break;
        } else if (d > VIS_EPS) {
          fliptest = true;
          break;
        }
      }
      if (k == source.n)
        continue; // planar with source portal

      if (fliptest) {
        plane.n = V3{-plane.n.x, -plane.n.y, -plane.n.z};
        plane.d = -plane.d;
      }

      int counts[3] = {0, 0, 0};
      for (k = 0; k < pass.n; k++) {
        if (k == j)
          continue;
        double d = Dot(pass.p[k], plane.n) - plane.d;
        if (d < -VIS_EPS)
          break;
        else if (d > VIS_EPS)
          counts[0]++;
        else
          counts[2]++;
      }
      if (k != pass.n)
        continue; // points on negative side
      if (!counts[0])
        continue; // planar with separating plane

      if (flipclip) {
        plane.n = V3{-plane.n.x, -plane.n.y, -plane.n.z};
        plane.d = -plane.d;
      }
      if (!Chop(target, plane))
        return false;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// polygon helpers for the state merging (not VVIS math)

struct HP {
  double x, y;
  V3 p;
};

void MakeBasis(const V3 &n, V3 &u, V3 &v) {
  double ax = std::fabs(n.x), ay = std::fabs(n.y), az = std::fabs(n.z);
  V3 a = (ax < ay) ? (ax < az ? V3{1, 0, 0} : V3{0, 0, 1})
                   : (ay < az ? V3{0, 1, 0} : V3{0, 0, 1});
  u = Cross(n, a);
  double len = std::sqrt(Dot(u, u));
  u = u * (1.0 / len);
  v = Cross(n, u);
}

// Monotone-chain hull of a and b in the plane basis of n. H must hold
// 4 * MAXW + 2 entries. Counter-clockwise around n; strictly convex.
int Hull2D(WRef a, WRef b, const V3 &n, HP *H, V3 &o, V3 &u, V3 &v) {
  HP pts[2 * MAXW];
  int m = 0;
  MakeBasis(n, u, v);
  o = a.n ? a.p[0] : b.p[0];
  auto add = [&](const V3 &p) {
    double dx = (double)p.x - o.x, dy = (double)p.y - o.y,
           dz = (double)p.z - o.z;
    pts[m].x = dx * u.x + dy * u.y + dz * u.z;
    pts[m].y = dx * v.x + dy * v.y + dz * v.z;
    pts[m].p = p;
    m++;
  };
  for (int i = 0; i < a.n; i++)
    add(a.p[i]);
  for (int i = 0; i < b.n; i++)
    add(b.p[i]);
  if (m <= 2) {
    for (int i = 0; i < m; i++)
      H[i] = pts[i];
    return m;
  }
  std::sort(pts, pts + m, [](const HP &p, const HP &q) {
    return p.x < q.x || (p.x == q.x && p.y < q.y);
  });
  auto cross = [](const HP &a0, const HP &b0, const HP &c0) {
    return (b0.x - a0.x) * (c0.y - a0.y) - (b0.y - a0.y) * (c0.x - a0.x);
  };
  int k = 0;
  for (int i = 0; i < m; i++) {
    while (k >= 2 && cross(H[k - 2], H[k - 1], pts[i]) <= 0)
      k--;
    H[k++] = pts[i];
  }
  for (int i = m - 2, t = k + 1; i >= 0; i--) {
    while (k >= t && cross(H[k - 2], H[k - 1], pts[i]) <= 0)
      k--;
    H[k++] = pts[i];
  }
  return k - 1;
}

double HullArea(const HP *H, int h) {
  double s = 0;
  for (int i = 0; i < h; i++) {
    const HP &p = H[i], &q = H[(i + 1) % h];
    s += p.x * q.y - q.x * p.y;
  }
  return fabs(s) * 0.5;
}

double PolyArea(WRef a, const V3 &n) {
  double sx = 0, sy = 0, sz = 0;
  for (int i = 1; i + 1 < a.n; i++) {
    double e1x = (double)a.p[i].x - a.p[0].x, e1y = (double)a.p[i].y - a.p[0].y,
           e1z = (double)a.p[i].z - a.p[0].z;
    double e2x = (double)a.p[i + 1].x - a.p[0].x,
           e2y = (double)a.p[i + 1].y - a.p[0].y,
           e2z = (double)a.p[i + 1].z - a.p[0].z;
    sx += e1y * e2z - e1z * e2y;
    sy += e1z * e2x - e1x * e2z;
    sz += e1x * e2y - e1y * e2x;
  }
  return fabs(sx * n.x + sy * n.y + sz * n.z) * 0.5;
}

// Every vertex of B is inside convex A within CONTAIN_EPS (A is on plane normal
// nrm). The box test keeps a collinear A from accepting its whole line.
bool Contains(WRef A, WRef B, const V3 &nrm) {
  const double eps = CONTAIN_EPS;
  if (B.n == 0)
    return true;
  if (A.n == 0)
    return false;
  V3 lo = A.p[0], hi = A.p[0];
  for (int i = 1; i < A.n; i++) {
    lo = {(std::min)(lo.x, A.p[i].x), (std::min)(lo.y, A.p[i].y),
          (std::min)(lo.z, A.p[i].z)};
    hi = {(std::max)(hi.x, A.p[i].x), (std::max)(hi.y, A.p[i].y),
          (std::max)(hi.z, A.p[i].z)};
  }
  for (int i = 0; i < B.n; i++) {
    const V3 &q = B.p[i];
    if (q.x < lo.x - eps || q.y < lo.y - eps || q.z < lo.z - eps ||
        q.x > hi.x + eps || q.y > hi.y + eps || q.z > hi.z + eps)
      return false;
  }
  if (A.n < 3) {
    for (int i = 0; i < B.n; i++) {
      const V3 &q = B.p[i];
      double best = 1e30;
      for (int e = 0; e < A.n; e++) {
        const V3 &a = A.p[e], &b = A.p[(e + 1) % A.n];
        double ex = (double)b.x - a.x, ey = (double)b.y - a.y,
               ez = (double)b.z - a.z;
        double wx = (double)q.x - a.x, wy = (double)q.y - a.y,
               wz = (double)q.z - a.z;
        double l2 = ex * ex + ey * ey + ez * ez;
        double t = l2 > 0 ? (wx * ex + wy * ey + wz * ez) / l2 : 0;
        t = (std::max)(0.0, (std::min)(1.0, t));
        double dx = wx - t * ex, dy = wy - t * ey, dz = wz - t * ez;
        best = (std::min)(best, sqrt(dx * dx + dy * dy + dz * dz));
      }
      if (best > eps)
        return false;
    }
    return true;
  }
  // inward in-plane edge normals; sign follows the polygon's orientation
  double sx = 0, sy = 0, sz = 0;
  for (int i = 1; i + 1 < A.n; i++) {
    double e1x = (double)A.p[i].x - A.p[0].x, e1y = (double)A.p[i].y - A.p[0].y,
           e1z = (double)A.p[i].z - A.p[0].z;
    double e2x = (double)A.p[i + 1].x - A.p[0].x,
           e2y = (double)A.p[i + 1].y - A.p[0].y,
           e2z = (double)A.p[i + 1].z - A.p[0].z;
    sx += e1y * e2z - e1z * e2y;
    sy += e1z * e2x - e1x * e2z;
    sz += e1x * e2y - e1y * e2x;
  }
  double sgn = (sx * nrm.x + sy * nrm.y + sz * nrm.z) < 0 ? -1.0 : 1.0;
  double mx[MAXW], my[MAXW], mz[MAXW], md[MAXW];
  int ne = 0;
  for (int i = 0; i < A.n; i++) {
    const V3 &a = A.p[i], &b = A.p[(i + 1) % A.n];
    double ex = (double)b.x - a.x, ey = (double)b.y - a.y,
           ez = (double)b.z - a.z;
    double l2 = ex * ex + ey * ey + ez * ez;
    if (l2 < 1e-12)
      continue;
    double inv = sgn / sqrt(l2);
    double cx = ((double)nrm.y * ez - (double)nrm.z * ey) * inv;
    double cy = ((double)nrm.z * ex - (double)nrm.x * ez) * inv;
    double cz = ((double)nrm.x * ey - (double)nrm.y * ex) * inv;
    mx[ne] = cx;
    my[ne] = cy;
    mz[ne] = cz;
    md[ne] = cx * a.x + cy * a.y + cz * a.z;
    ne++;
  }
  for (int i = 0; i < B.n; i++) {
    const V3 &q = B.p[i];
    for (int e = 0; e < ne; e++) {
      if (mx[e] * q.x + my[e] * q.y + mz[e] * q.z - md[e] < -eps)
        return false;
    }
  }
  return true;
}

// What the run reports: how often the safety nets fired.
struct Counters {
  u64 guard = 0; // states frozen to the whole portal (Freeze)
  u64 obb = 0;   // hulls over HULL_MAX replaced by a clipped rectangle
};

// A convex superset of hull(a U b) on the plane with normal nrm. A hull over
// HULL_MAX points becomes its minimum-area bounding rectangle clipped to the
// owning portal's winding, which is a superset by construction.
std::vector<V3> MakeConvex(WRef a, WRef b, const V3 &nrm, const Winding &owner,
                           Counters &cnt) {
  HP H[4 * MAXW + 2];
  V3 o, u, v;
  int h = Hull2D(a, b, nrm, H, o, u, v);
  std::vector<V3> out;
  if (h <= HULL_MAX) {
    out.reserve(h);
    for (int i = 0; i < h; i++)
      out.push_back(H[i].p);
    return out;
  }
  cnt.obb++;
  double bestArea = 1e300, bdx = 0, bdy = 0, bminU = 0, bmaxU = 0, bminV = 0,
         bmaxV = 0;
  for (int i = 0; i < h; i++) {
    const HP &p0 = H[i], &p1 = H[(i + 1) % h];
    double dx = p1.x - p0.x, dy = p1.y - p0.y;
    double len = sqrt(dx * dx + dy * dy);
    if (len < 1e-9)
      continue;
    dx /= len;
    dy /= len;
    double px = -dy, py = dx;
    double minU = 1e300, maxU = -1e300, minV = 1e300, maxV = -1e300;
    for (int k = 0; k < h; k++) {
      double U = H[k].x * dx + H[k].y * dy, V = H[k].x * px + H[k].y * py;
      minU = (std::min)(minU, U);
      maxU = (std::max)(maxU, U);
      minV = (std::min)(minV, V);
      maxV = (std::max)(maxV, V);
    }
    double area = (maxU - minU) * (maxV - minV);
    if (area < bestArea) {
      bestArea = area;
      bdx = dx;
      bdy = dy;
      bminU = minU;
      bmaxU = maxU;
      bminV = minV;
      bmaxV = maxV;
    }
  }
  Winding w;
  Wcopy(w, owner);
  if (bestArea < 1e299) {
    const double pad = 0.05;
    bminU -= pad;
    bmaxU += pad;
    bminV -= pad;
    bmaxV += pad;
    double px = -bdy, py = bdx;
    double cu[4] = {bminU, bmaxU, bmaxU, bminU},
           cv[4] = {bminV, bminV, bmaxV, bmaxV};
    V3 corner[4];
    for (int i = 0; i < 4; i++) {
      double x = cu[i] * bdx + cv[i] * px, y = cu[i] * bdy + cv[i] * py;
      corner[i] = {o.x + u.x * x + v.x * y, o.y + u.y * x + v.y * y,
                   o.z + u.z * x + v.z * y};
    }
    for (int i = 0; i < 4; i++) {
      V3 e = corner[(i + 1) % 4] - corner[i];
      V3 m = Cross(nrm, e);
      double len = std::sqrt(Dot(m, m));
      if (len < 1e-9)
        continue;
      Plane pl;
      pl.n = m * (1.0 / len);
      pl.d = Dot(pl.n, corner[i]);
      if (!Chop(w, pl)) {
        Wcopy(w, owner);
        break;
      }
    }
  }
  out.assign(w.p, w.p + w.n);
  return out;
}

// ---------------------------------------------------------------------------
// portals, rebuilt from the parsed doubles in file order (forward portal 2i,
// backward 2i+1), the way LoadPortals builds the float ones

struct Portal {
  Plane plane;
  int leaf;  // cluster the portal leads into
  int owner; // cluster that lists it
  Winding w;
  V3 origin;
  double radius;
};

struct Map {
  int nc = 0, N = 0;
  std::vector<Portal> portals;
  std::vector<std::vector<int>> cp; // portals owned by each cluster, file order
};

Plane PlaneFromWinding(const Winding &w) {
  V3 v1 = w.p[2] - w.p[1], v2 = w.p[0] - w.p[1];
  Plane pl;
  pl.n = Cross(v2, v1);
  double radius = std::sqrt(Dot(pl.n, pl.n));
  // Stock's guard against a zero-area winding; it does not bias a double normal.
  double iradius = 1.0 / (radius + std::numeric_limits<double>::epsilon());
  pl.n = pl.n * iradius;
  pl.d = Dot(w.p[0], pl.n);
  return pl;
}

void SetPortalSphere(Portal &p) {
  const Winding &w = p.w;
  V3 total = {0, 0, 0};
  for (int i = 0; i < w.n; i++)
    total = total + w.p[i];
  for (int i = 0; i < 3; i++)
    total[i] /= w.n;
  double bestr = 0;
  for (int i = 0; i < w.n; i++) {
    V3 dist = w.p[i] - total;
    double r = std::sqrt(Dot(dist, dist));
    if (r > bestr)
      bestr = r;
  }
  p.origin = total;
  p.radius = bestr;
}

void BuildMap(Map &m) {
  if (g_numportals > 0 && !g_pMergePortals)
    Error("MergeFlow: the portal file was not kept for the merge flow\n");
  m.nc = portalclusters;
  m.N = g_numportals * 2;
  m.portals.assign(m.N, Portal());
  m.cp.assign(m.nc, std::vector<int>());

  for (int i = 0; i < g_numportals; i++) {
    const mergeportal_t &mp = g_pMergePortals[i];
    int np = mp.numpoints;
    int l0 = mp.leafnums[0], l1 = mp.leafnums[1];
    if (np < 3)
      Error("MergeFlow: portal %d has %d points; a portal needs at least 3\n",
            i, np);
    if (np > MAXW) // LoadPortals refuses these first; this guards a MAXW change
      Error("MergeFlow: portal %d has %d points, more than the %d a winding "
            "holds\n",
            i, np, MAXW);
    if (l0 < 0 || l1 < 0 || l0 >= m.nc || l1 >= m.nc)
      Error("MergeFlow: portal %d: cluster out of range\n", i);
    Winding w;
    w.n = np;
    for (int j = 0; j < np; j++)
      for (int k = 0; k < 3; k++)
        w.p[j][k] = mp.points[j * 3 + k];
    Plane plane = PlaneFromWinding(w);

    Portal &fw = m.portals[2 * i];
    Wcopy(fw.w, w);
    fw.plane.n = V3{-plane.n.x, -plane.n.y, -plane.n.z};
    fw.plane.d = -plane.d;
    fw.leaf = l1;
    fw.owner = l0;
    SetPortalSphere(fw);
    m.cp[l0].push_back(2 * i);

    Portal &bw = m.portals[2 * i + 1];
    bw.w.n = np;
    for (int j = 0; j < np; j++)
      bw.w.p[j] = w.p[np - 1 - j];
    bw.plane = plane;
    bw.leaf = l0;
    bw.owner = l1;
    SetPortalSphere(bw);
    m.cp[l1].push_back(2 * i + 1);
  }
}

// ---------------------------------------------------------------------------
// U0: BasePortalVis + SimpleFlood, in double

struct U0Scratch {
  std::vector<u64> front;
  std::vector<char> seen;
  std::vector<int> stack;
};

void ComputeU0(const Map &m, int base, u64 *flood, U0Scratch &s) {
  const Portal &p = m.portals[base];
  size_t Wd = (m.N + 63) / 64;
  s.front.assign(Wd, 0);
  for (int j = 0; j < m.N; j++) {
    if (j == base)
      continue;
    const Portal &tp = m.portals[j];
    int k;
    for (k = 0; k < tp.w.n; k++) {
      double d = Dot(tp.w.p[k], p.plane.n) - p.plane.d;
      if (d > VIS_EPS)
        break;
    }
    if (k == tp.w.n)
      continue; // no points on front
    for (k = 0; k < p.w.n; k++) {
      double d = Dot(p.w.p[k], tp.plane.n) - tp.plane.d;
      if (d < -VIS_EPS)
        break;
    }
    if (k == p.w.n)
      continue; // stock: no point behind tp
    // stock's radius vis: skip a portal with no point inside the radius
    if (g_bUseRadius) {
      double minDist2 = 1024000000.0; // 32000^2
      for (k = 0; k < tp.w.n; k++) {
        V3 seg = tp.w.p[k] - p.origin;
        double dist2 = (seg.x * seg.x) + (seg.y * seg.y) + (seg.z * seg.z);
        if (dist2 < minDist2)
          minDist2 = dist2;
      }
      if (minDist2 > g_VisRadius)
        continue;
    }
    BitSet(s.front.data(), j);
  }
  // SimpleFlood, iterative: the set is the closure over clusters, so each
  // cluster needs scanning only once.
  s.seen.assign(m.nc, 0);
  s.stack.clear();
  s.stack.push_back(p.leaf);
  s.seen[p.leaf] = 1;
  while (!s.stack.empty()) {
    int l = s.stack.back();
    s.stack.pop_back();
    for (int r : m.cp[l]) {
      if (!BitGet(s.front.data(), r))
        continue;
      if (BitGet(flood, r))
        continue;
      BitSet(flood, r);
      int nl = m.portals[r].leaf;
      if (!s.seen[nl]) {
        s.seen[nl] = 1;
        s.stack.push_back(nl);
      }
    }
  }
}

// ---------------------------------------------------------------------------
// U2: bounded-state flow

struct SState {
  std::vector<V3> S, W;
  bool dirty;
};

struct U2Worker {
  const Map &m;
  const BitMat &U0;
  int K;
  Counters cnt;

  std::vector<std::vector<SState>> st;
  std::vector<int> upd, tot;
  std::vector<char> frozen, inq, touched;
  std::vector<int> touchedList, queue;
  std::vector<std::pair<Winding, Winding>> snap;
  size_t qhead = 0;

  int base = 0;
  const Portal *P = nullptr;
  const u64 *U0P = nullptr;
  u64 *U2P = nullptr;

  U2Worker(const Map &m_, const BitMat &u0, int k) : m(m_), U0(u0), K(k) {
    st.resize(m.N);
    upd.assign(m.N, 0);
    tot.assign(m.N, 0);
    frozen.assign(m.N, 0);
    inq.assign(m.N, 0);
    touched.assign(m.N, 0);
  }

  // MTFlowEnter lines 849-872: the pass and source windings for entering r
  // from a state whose source is prevSource.
  bool Step(const Portal &r, const Winding &prevSource, Winding &pass,
            Winding &source) const {
    double d = Dot(r.origin, P->plane.n);
    d -= P->plane.d;
    if (d < -r.radius)
      return false;
    Wcopy(pass, r.w);
    if (!(d > r.radius)) {
      if (!Chop(pass, P->plane))
        return false;
    }
    d = Dot(P->origin, r.plane.n);
    d -= r.plane.d;
    if (d > P->radius)
      return false;
    Wcopy(source, prevSource);
    if (!(d < -P->radius)) {
      Plane back;
      back.n = V3{-r.plane.n.x, -r.plane.n.y, -r.plane.n.z};
      back.d = -r.plane.d;
      if (!Chop(source, back))
        return false;
    }
    return true;
  }

  void Touch(int q) {
    if (!touched[q]) {
      touched[q] = 1;
      touchedList.push_back(q);
    }
  }
  void Enqueue(int q) {
    if (!inq[q]) {
      inq[q] = 1;
      queue.push_back(q);
    }
  }

  void Freeze(int q) {
    cnt.guard++;
    Winding W;
    Wcopy(W, m.portals[q].w);
    if (!Chop(W, P->plane))
      Wcopy(W, m.portals[q].w);
    auto &v = st[q];
    v.clear();
    SState s;
    s.S.assign(P->w.p, P->w.p + P->w.n);
    s.W.assign(W.p, W.p + W.n);
    s.dirty = true;
    v.push_back(std::move(s));
    frozen[q] = 1;
    Enqueue(q);
  }

  void Arrive(int q, const Winding &S, const Winding &W) {
    Touch(q);
    if (frozen[q])
      return;
    const V3 &nP = P->plane.n;
    const V3 &nq = m.portals[q].plane.n;
    auto &v = st[q];
    for (const SState &s : v) {
      if (Contains(s.S, S, nP) && Contains(s.W, W, nq))
        return;
    }
    size_t keep = 0;
    for (size_t i = 0; i < v.size(); i++) {
      if (Contains(S, v[i].S, nP) && Contains(W, v[i].W, nq))
        continue;
      if (keep != i)
        v[keep] = std::move(v[i]);
      keep++;
    }
    v.erase(v.begin() + keep, v.end());

    tot[q]++;
    bool atCap = v.size() >= (size_t)K;
    if (!atCap) {
      SState s;
      s.S.assign(S.p, S.p + S.n);
      s.W.assign(W.p, W.p + W.n);
      s.dirty = true;
      v.push_back(std::move(s));
    } else {
      size_t best = 0;
      if (v.size() > 1) {
        double bestCost = 1e300;
        for (size_t i = 0; i < v.size(); i++) {
          HP H[4 * MAXW + 2];
          V3 o, u, w;
          int hw = Hull2D(v[i].W, W, nq, H, o, u, w);
          double cost = HullArea(H, hw) - PolyArea(v[i].W, nq);
          int hs = Hull2D(v[i].S, S, nP, H, o, u, w);
          cost += HullArea(H, hs) - PolyArea(v[i].S, nP);
          if (cost < bestCost) {
            bestCost = cost;
            best = i;
          }
        }
      }
      SState &s = v[best];
      s.S = MakeConvex(s.S, S, nP, P->w, cnt);
      s.W = MakeConvex(s.W, W, nq, m.portals[q].w, cnt);
      s.dirty = true;
      upd[q]++;
    }
    if (upd[q] > GUARD_UPDATES || tot[q] > GUARD_ARRIVALS) {
      Freeze(q);
      return;
    }
    Enqueue(q);
  }

  void Flow(int q, const Winding &S, const Winding &Wd) {
    const Portal &pq = m.portals[q];
    const u64 *tq = U0.row(q);
    for (int r : m.cp[pq.leaf]) {
      if (!BitGet(U0P, r))
        continue;
      if (!BitGet(tq, r))
        continue;
      Winding pass, source;
      if (!Step(m.portals[r], S, pass, source))
        continue;
      if (!ClipToSeperators(source, Wd, pass, false))
        continue;
      if (!ClipToSeperators(Wd, source, pass, true))
        continue;
      BitSet(U2P, r);
      Arrive(r, source, pass);
    }
  }

  // The bits of base portal b's row are set in out (portalbytes long, zeroed).
  void Run(int b, u64 *out) {
    base = b;
    P = &m.portals[b];
    U0P = U0.row(b);
    U2P = out;
    queue.clear();
    qhead = 0;

    // first hop: no separator clipping, as in stock when the previous pass is
    // NULL
    for (int r : m.cp[P->leaf]) {
      if (!BitGet(U0P, r))
        continue;
      Winding pass, source;
      if (!Step(m.portals[r], P->w, pass, source))
        continue;
      BitSet(U2P, r);
      Arrive(r, source, pass);
    }
    while (qhead < queue.size()) {
      int q = queue[qhead++];
      inq[q] = 0;
      snap.clear();
      for (SState &s : st[q]) {
        if (!s.dirty)
          continue;
        s.dirty = false;
        snap.emplace_back();
        ToWinding(s.S, snap.back().first);
        ToWinding(s.W, snap.back().second);
      }
      for (size_t i = 0; i < snap.size(); i++)
        Flow(q, snap[i].first, snap[i].second);
    }
    for (int q : touchedList) {
      st[q].clear();
      upd[q] = tot[q] = 0;
      frozen[q] = inq[q] = touched[q] = 0;
    }
    touchedList.clear();
  }
};

// ---------------------------------------------------------------------------
// threading: RunThreadsOnIndividual takes a plain function, so the run's state
// is reached through this pointer

struct MergeRun {
  Map m;
  BitMat U0;
  std::vector<U0Scratch> scratch; // per thread
  std::vector<int> nummightsee;
  std::vector<int> order;         // base portals, fewest might-see first
  std::vector<U2Worker> workers;  // per thread
};

MergeRun *s_run = nullptr;

void MergeBaseItem(int iThread, int portalnum) {
  MergeRun &r = *s_run;
  ComputeU0(r.m, portalnum, r.U0.row(portalnum), r.scratch[iThread]);
  r.nummightsee[portalnum] = (int)CountRow(r.U0.row(portalnum), r.U0.W);
}

void MergeFlowItem(int iThread, int itemnum) {
  MergeRun &r = *s_run;
  int b = r.order[itemnum];
  byte *vis = portals[b].portalvis;
  memset(vis, 0, portalbytes);
  r.workers[iThread].Run(b, (u64 *)vis);
  portals[b].status = stat_done;
}

} // namespace

void RunMergeFlow(int nStates, long long *pGuardFirings,
                  long long *pRectFallbacks) {
  *pGuardFirings = 0;
  *pRectFallbacks = 0;
  if (nStates < 1)
    Error("MergeFlow: -mergestates must be at least 1 (got %d)\n", nStates);

  MergeRun run;
  BuildMap(run.m);
  // The doubles were only kept for the rebuild above.
  FreeMergePortals();

  const int N = run.m.N;
  if (N == 0)
    return;
  // The workers write 64-bit words straight into portals[].portalvis, so the
  // row must be a whole number of them.
  if (portalbytes != (int)(((N + 63) / 64) * 8))
    Error("MergeFlow: portal row is %d bytes, expected %d\n", portalbytes,
          (int)(((N + 63) / 64) * 8));

  // RunThreadsOnIndividual resolves an unset thread count itself; resolve it
  // here first, so the per-thread arrays match the count it will run.
  if (numthreads < 1)
    ThreadSetDefault();
  int nt = numthreads;
  if (nt < 1)
    nt = 1;
  if (nt > MAX_TOOL_THREADS)
    nt = MAX_TOOL_THREADS;

  run.U0.init(N, N);
  run.scratch.resize(nt);
  run.nummightsee.assign(N, 0);
  s_run = &run;
  RunThreadsOnIndividual(N, true, MergeBaseItem);

  run.order.resize(N);
  for (int i = 0; i < N; i++)
    run.order[i] = i;
  std::stable_sort(run.order.begin(), run.order.end(),
                   [&](int a, int b) {
                     return run.nummightsee[a] < run.nummightsee[b];
                   });

  run.workers.reserve(nt);
  for (int t = 0; t < nt; t++)
    run.workers.emplace_back(run.m, run.U0, nStates);
  RunThreadsOnIndividual(N, true, MergeFlowItem);
  s_run = nullptr;

  for (const U2Worker &w : run.workers) {
    *pGuardFirings += (long long)w.cnt.guard;
    *pRectFallbacks += (long long)w.cnt.obb;
  }
}

#pragma float_control(pop)
