// prtvis: research tool, not part of the compilers.
//
// Cheap bounds on the visibility VVIS would compute from a .prt, compared at
// cluster level with the exact PVS stored in a compiled .bsp:
//   U0  VVIS's own BasePortalVis + SimpleFlood (upper bound)
//   U2  bounded-state flow: the exact flow, but the arrivals at a portal are
//       merged into at most K convex (source, pass) states
//   L   straight rays walked through the portal graph (lower bound)
// The geometry is a port of src/utils/vvis/{vvis,flow}.cpp; clip math is float
// by default so the results track VVIS, and -DPRTVIS_DOUBLE runs the same math
// in double (prtvis_d.exe) to measure what float rounding costs. One
// translation unit, standard C++17 only.
//
// What U2 is and is not. A merged state contains every state folded into it,
// and every plane ClipToSeperators accepts separates the whole source from the
// whole pass, so in exact arithmetic no straight sight line through a portal
// chain is lost: U2 is meant as a superset of TRUE visibility. It is NOT a
// superset of the stock result. ClipToSeperators is not monotone: a degenerate
// sliver window yields no separating plane and leaves the target unclipped, so
// the stock per-chain flow marks portals that a fatter merged state clips away
// (fightspace, visclusters on: 2 cluster pairs at K=1). The guarantee is also
// only as good as the arithmetic. The float build loses real sight lines on
// sliver portals and grazing angles at 16k-unit coordinates (8 to 47 portal
// bits per 68 million, issue #89); the double build lost none in 1.9e9 rays.
// Even in double, a line whose source and window points both lie within the vis
// epsilon of an accepted plane can be clipped for a far target, as its distance
// from the plane grows linearly along the ray: the sliver-width case (--eps).
// A smaller --eps is not the cure. The portal file's coordinates come out of
// VBSP's float math, good to about 0.001 at 16k units, and below that the side
// tests decide on noise: 1e-4 and 1e-6 move the result by 20 cluster pairs on
// d1_trainstation_03 and fightspace, and 1e-4 exposed a grazing line off a
// 2.5-unit portal that 0.01 never samples. vvis -mergeflow keeps stock's 0.01.
// "portal level L not in U2" is the check: run rays with every U2 measurement.
//
// --might (experimental, off by default) restores two prunes the plain flow
// lacks, as stock has them: each state carries a mightsee bitset (the portals a
// chain reaching it can still mark: U0(P), ANDed with the test row of every
// portal on the way), a hop into r needs r in it, and a hop that can mark
// nothing new is skipped. Merging ORs the bitsets, and a state only dominates an
// arrival whose bitset it contains. --reuse then makes the test row of a portal
// finished in an earlier wave its U2 row, as stock's portalvis does.
//
// --patch-bsp <in.bsp> --out-bsp <out.bsp> (with --upper, every base portal)
// writes the post-cross-check U2 cluster rows, and the PAS built from them, as
// the visibility lump of a copy of in.bsp, to load the merged-window result in
// the game. It is a measurement aid: the float build can drop real sight lines
// (above), so a patched map is not one to ship. The copy is written to
// <out>.tmp, checked, and only then renamed over <out>.

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#if defined(_MSC_VER)
#include <intrin.h>
#endif
#if !defined(_WIN32)
#include <sys/types.h> // off_t, for fseeko / ftello
#endif

typedef uint64_t u64;
typedef std::chrono::steady_clock Clock;

static double Secs(Clock::time_point a, Clock::time_point b) {
  return std::chrono::duration<double>(b - a).count();
}

// Scalar of the VVIS-math path (V3, windings, Chop, ClipToSeperators, Step, U0).
// float is stock's vec_t; the double helpers below (Hull2D, Contains, ray walk)
// are double in both builds.
#ifdef PRTVIS_DOUBLE
typedef double real;
static const char *const REAL_NAME = "double";
#else
typedef float real;
static const char *const REAL_NAME = "float";
#endif

// VVIS's ON_VIS_EPSILON (0.01). A runtime value (--eps) so one build can sweep
// it; independent of g_containEps.
static real g_visEps = real(0.01);
// Slack of the dominance test (--contain-eps): an arrival sticking out of a
// kept state by less than this is dropped, and with it any sight line through
// that strip. The float build keeps stock's 0.01, since its window vertices are
// only good to about 0.001 at 16k units; the double build can afford 1e-6.
// Exactly 0 stops identical windows from dominating each other (rounding).
#ifdef PRTVIS_DOUBLE
static double g_containEps = 1e-6;
#else
static double g_containEps = 0.01;
#endif
enum { SIDE_FRONT = 0, SIDE_BACK = 1, SIDE_ON = 2 };

static const int MAXW = 64;         // winding capacity = the most points stock accepts in a portal (its stack windings clip at 12)
static const int HULL_MAX = 24;     // larger hulls become a clipped rectangle
static const int GUARD_UPDATES = 256;   // merges into one portal's states, per base
static const int GUARD_ARRIVALS = 65536; // backstop for creation-only creep
static const int K0_CAP = 4096;     // K == 0: states per portal before merging
static const int WAVE = 512;        // bases per reuse wave

// ---------------------------------------------------------------------------
// bit helpers

static inline bool GetBit(const u64 *r, int i) { return (r[i >> 6] >> (i & 63)) & 1; }
static inline void SetBit(u64 *r, int i) { r[i >> 6] |= 1ull << (i & 63); }
static inline void ClrBit(u64 *r, int i) { r[i >> 6] &= ~(1ull << (i & 63)); }
static inline int Pop(u64 x) {
#if defined(_MSC_VER) && defined(_M_X64)
  return (int)__popcnt64(x);
#else
  return __builtin_popcountll(x);
#endif
}
static inline int Ctz(u64 x) {
#if defined(_MSC_VER) && defined(_M_X64)
  unsigned long i;
  _BitScanForward64(&i, x);
  return (int)i;
#else
  return __builtin_ctzll(x);
#endif
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

static long long CountRow(const u64 *r, size_t W) {
  long long c = 0;
  for (size_t i = 0; i < W; i++) c += Pop(r[i]);
  return c;
}
static long long CountMat(const BitMat &m) { return CountRow(m.d.data(), m.d.size()); }

template <class F> static void ForEachBit(const u64 *r, size_t W, F f) {
  for (size_t w = 0; w < W; w++) {
    u64 x = r[w];
    while (x) {
      f((int)(w * 64 + Ctz(x)));
      x &= x - 1;
    }
  }
}

static u64 Fnv(const void *data, size_t n, u64 h = 0xcbf29ce484222325ull) {
  const unsigned char *p = (const unsigned char *)data;
  for (size_t i = 0; i < n; i++) {
    h ^= p[i];
    h *= 0x100000001b3ull;
  }
  return h;
}

// ---------------------------------------------------------------------------
// geometry (real: float like vec_t, or double with -DPRTVIS_DOUBLE)

struct V3 {
  real x, y, z;
  real &operator[](int i) { return (&x)[i]; }
  real operator[](int i) const { return (&x)[i]; }
};
static inline V3 operator-(const V3 &a, const V3 &b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
static inline V3 operator+(const V3 &a, const V3 &b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
static inline V3 operator*(const V3 &a, real s) { return {a.x * s, a.y * s, a.z * s}; }
static inline real Dot(const V3 &a, const V3 &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline V3 Cross(const V3 &a, const V3 &b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

struct Plane {
  V3 n;
  real d;
};

struct Winding {
  int n;
  V3 p[MAXW];
};

static inline void Wcopy(Winding &dst, const Winding &src) {
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

static void ToWinding(const std::vector<V3> &v, Winding &w) {
  w.n = (int)v.size();
  memcpy(w.p, v.data(), v.size() * sizeof(V3));
}

// ChopWinding, in place: keeps the part in front of split (+/- g_visEps).
// False when nothing is in front. On overflow w is left unchanged, which is the
// stock conservative fallback.
static bool Chop(Winding &w, const Plane &split) {
  real dists[MAXW + 1];
  int sides[MAXW + 1];
  int counts[3] = {0, 0, 0};

  for (int i = 0; i < w.n; i++) {
    real dot = Dot(w.p[i], split.n);
    dot -= split.d;
    dists[i] = dot;
    if (dot > g_visEps)
      sides[i] = SIDE_FRONT;
    else if (dot < -g_visEps)
      sides[i] = SIDE_BACK;
    else
      sides[i] = SIDE_ON;
    counts[sides[i]]++;
  }
  if (!counts[1]) return true;
  if (!counts[0]) return false;

  sides[w.n] = sides[0];
  dists[w.n] = dists[0];

  Winding neww;
  neww.n = 0;
  for (int i = 0; i < w.n; i++) {
    const V3 &p1 = w.p[i];
    if (neww.n == MAXW) return true;
    if (sides[i] == SIDE_ON) {
      neww.p[neww.n++] = p1;
      continue;
    }
    if (sides[i] == SIDE_FRONT) neww.p[neww.n++] = p1;
    if (sides[i + 1] == SIDE_ON || sides[i + 1] == sides[i]) continue;
    if (neww.n == MAXW) return true;

    const V3 &p2 = w.p[(i + 1) % w.n];
    real dot = dists[i] / (dists[i] - dists[i + 1]);
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

// ClipToSeperators, clipping target in place. False when target is clipped away.
static bool ClipToSeperators(const Winding &source, const Winding &pass,
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

      real length = plane.n.x * plane.n.x + plane.n.y * plane.n.y + plane.n.z * plane.n.z;
      if (length < g_visEps) continue;

      length = real(1) / std::sqrt(length);
      plane.n.x *= length;
      plane.n.y *= length;
      plane.n.z *= length;
      plane.d = Dot(pass.p[j], plane.n);

      bool fliptest = false;
      int k;
      for (k = 0; k < source.n; k++) {
        if (k == i || k == l) continue;
        real d = Dot(source.p[k], plane.n) - plane.d;
        if (d < -g_visEps) {
          fliptest = false;
          break;
        } else if (d > g_visEps) {
          fliptest = true;
          break;
        }
      }
      if (k == source.n) continue; // planar with source portal

      if (fliptest) {
        plane.n = V3{-plane.n.x, -plane.n.y, -plane.n.z};
        plane.d = -plane.d;
      }

      int counts[3] = {0, 0, 0};
      for (k = 0; k < pass.n; k++) {
        if (k == j) continue;
        real d = Dot(pass.p[k], plane.n) - plane.d;
        if (d < -g_visEps)
          break;
        else if (d > g_visEps)
          counts[0]++;
        else
          counts[2]++;
      }
      if (k != pass.n) continue; // points on negative side
      if (!counts[0]) continue;  // planar with separating plane

      if (flipclip) {
        plane.n = V3{-plane.n.x, -plane.n.y, -plane.n.z};
        plane.d = -plane.d;
      }
      if (!Chop(target, plane)) return false;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// double-precision polygon helpers for the state merging (not VVIS math)

struct HP {
  double x, y;
  V3 p;
};

static void MakeBasis(const V3 &n, V3 &u, V3 &v) {
  real ax = std::fabs(n.x), ay = std::fabs(n.y), az = std::fabs(n.z);
  V3 a = (ax < ay) ? (ax < az ? V3{1, 0, 0} : V3{0, 0, 1}) : (ay < az ? V3{0, 1, 0} : V3{0, 0, 1});
  u = Cross(n, a);
  real len = std::sqrt(Dot(u, u));
  u = u * (real(1) / len);
  v = Cross(n, u);
}

// Monotone-chain hull of a and b in the plane basis of n. H must hold
// 4 * MAXW + 2 entries. Counter-clockwise around n; strictly convex.
static int Hull2D(WRef a, WRef b, const V3 &n, HP *H, V3 &o, V3 &u, V3 &v) {
  HP pts[2 * MAXW];
  int m = 0;
  MakeBasis(n, u, v);
  o = a.n ? a.p[0] : b.p[0];
  auto add = [&](const V3 &p) {
    double dx = (double)p.x - o.x, dy = (double)p.y - o.y, dz = (double)p.z - o.z;
    pts[m].x = dx * u.x + dy * u.y + dz * u.z;
    pts[m].y = dx * v.x + dy * v.y + dz * v.z;
    pts[m].p = p;
    m++;
  };
  for (int i = 0; i < a.n; i++) add(a.p[i]);
  for (int i = 0; i < b.n; i++) add(b.p[i]);
  if (m <= 2) {
    for (int i = 0; i < m; i++) H[i] = pts[i];
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
    while (k >= 2 && cross(H[k - 2], H[k - 1], pts[i]) <= 0) k--;
    H[k++] = pts[i];
  }
  for (int i = m - 2, t = k + 1; i >= 0; i--) {
    while (k >= t && cross(H[k - 2], H[k - 1], pts[i]) <= 0) k--;
    H[k++] = pts[i];
  }
  return k - 1;
}

static double HullArea(const HP *H, int h) {
  double s = 0;
  for (int i = 0; i < h; i++) {
    const HP &p = H[i], &q = H[(i + 1) % h];
    s += p.x * q.y - q.x * p.y;
  }
  return fabs(s) * 0.5;
}

static double PolyArea(WRef a, const V3 &n) {
  double sx = 0, sy = 0, sz = 0;
  for (int i = 1; i + 1 < a.n; i++) {
    double e1x = (double)a.p[i].x - a.p[0].x, e1y = (double)a.p[i].y - a.p[0].y,
           e1z = (double)a.p[i].z - a.p[0].z;
    double e2x = (double)a.p[i + 1].x - a.p[0].x, e2y = (double)a.p[i + 1].y - a.p[0].y,
           e2z = (double)a.p[i + 1].z - a.p[0].z;
    sx += e1y * e2z - e1z * e2y;
    sy += e1z * e2x - e1x * e2z;
    sz += e1x * e2y - e1y * e2x;
  }
  return fabs(sx * n.x + sy * n.y + sz * n.z) * 0.5;
}

// Every vertex of B is inside convex A within g_containEps (A is on plane
// normal nrm). The box test keeps a collinear A from accepting its whole line.
static bool Contains(WRef A, WRef B, const V3 &nrm) {
  const double eps = g_containEps;
  if (B.n == 0) return true;
  if (A.n == 0) return false;
  V3 lo = A.p[0], hi = A.p[0];
  for (int i = 1; i < A.n; i++) {
    lo = {std::min(lo.x, A.p[i].x), std::min(lo.y, A.p[i].y), std::min(lo.z, A.p[i].z)};
    hi = {std::max(hi.x, A.p[i].x), std::max(hi.y, A.p[i].y), std::max(hi.z, A.p[i].z)};
  }
  for (int i = 0; i < B.n; i++) {
    const V3 &q = B.p[i];
    if (q.x < lo.x - eps || q.y < lo.y - eps || q.z < lo.z - eps || q.x > hi.x + eps ||
        q.y > hi.y + eps || q.z > hi.z + eps)
      return false;
  }
  if (A.n < 3) {
    for (int i = 0; i < B.n; i++) {
      const V3 &q = B.p[i];
      double best = 1e30;
      for (int e = 0; e < A.n; e++) {
        const V3 &a = A.p[e], &b = A.p[(e + 1) % A.n];
        double ex = (double)b.x - a.x, ey = (double)b.y - a.y, ez = (double)b.z - a.z;
        double wx = (double)q.x - a.x, wy = (double)q.y - a.y, wz = (double)q.z - a.z;
        double l2 = ex * ex + ey * ey + ez * ez;
        double t = l2 > 0 ? (wx * ex + wy * ey + wz * ez) / l2 : 0;
        t = std::max(0.0, std::min(1.0, t));
        double dx = wx - t * ex, dy = wy - t * ey, dz = wz - t * ez;
        best = std::min(best, sqrt(dx * dx + dy * dy + dz * dz));
      }
      if (best > eps) return false;
    }
    return true;
  }
  // inward in-plane edge normals; sign follows the polygon's orientation
  double sx = 0, sy = 0, sz = 0;
  for (int i = 1; i + 1 < A.n; i++) {
    double e1x = (double)A.p[i].x - A.p[0].x, e1y = (double)A.p[i].y - A.p[0].y,
           e1z = (double)A.p[i].z - A.p[0].z;
    double e2x = (double)A.p[i + 1].x - A.p[0].x, e2y = (double)A.p[i + 1].y - A.p[0].y,
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
    double ex = (double)b.x - a.x, ey = (double)b.y - a.y, ez = (double)b.z - a.z;
    double l2 = ex * ex + ey * ey + ez * ez;
    if (l2 < 1e-12) continue;
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
      if (mx[e] * q.x + my[e] * q.y + mz[e] * q.z - md[e] < -eps) return false;
    }
  }
  return true;
}

struct Counters {
  u64 created = 0, merges = 0, dominated = 0, arrivals = 0, guard = 0, capHits = 0, obb = 0,
      rays = 0, flowSteps = 0, reusePrunes = 0, mightSkips = 0, nothingNew = 0, rayCapHits = 0;
  void add(const Counters &o) {
    rayCapHits += o.rayCapHits;
    mightSkips += o.mightSkips;
    nothingNew += o.nothingNew;
    created += o.created;
    merges += o.merges;
    dominated += o.dominated;
    arrivals += o.arrivals;
    guard += o.guard;
    capHits += o.capHits;
    obb += o.obb;
    rays += o.rays;
    flowSteps += o.flowSteps;
    reusePrunes += o.reusePrunes;
  }
};

// A convex superset of hull(a U b) on the plane with normal nrm. A hull over
// HULL_MAX points becomes its minimum-area bounding rectangle clipped to the
// owning portal's winding, which is a superset by construction.
static std::vector<V3> MakeConvex(WRef a, WRef b, const V3 &nrm, const Winding &owner,
                                  Counters &cnt) {
  HP H[4 * MAXW + 2];
  V3 o, u, v;
  int h = Hull2D(a, b, nrm, H, o, u, v);
  std::vector<V3> out;
  if (h <= HULL_MAX) {
    out.reserve(h);
    for (int i = 0; i < h; i++) out.push_back(H[i].p);
    return out;
  }
  cnt.obb++;
  double bestArea = 1e300, bdx = 0, bdy = 0, bminU = 0, bmaxU = 0, bminV = 0, bmaxV = 0;
  for (int i = 0; i < h; i++) {
    const HP &p0 = H[i], &p1 = H[(i + 1) % h];
    double dx = p1.x - p0.x, dy = p1.y - p0.y;
    double len = sqrt(dx * dx + dy * dy);
    if (len < 1e-9) continue;
    dx /= len;
    dy /= len;
    double px = -dy, py = dx;
    double minU = 1e300, maxU = -1e300, minV = 1e300, maxV = -1e300;
    for (int k = 0; k < h; k++) {
      double U = H[k].x * dx + H[k].y * dy, V = H[k].x * px + H[k].y * py;
      minU = std::min(minU, U);
      maxU = std::max(maxU, U);
      minV = std::min(minV, V);
      maxV = std::max(maxV, V);
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
    double cu[4] = {bminU, bmaxU, bmaxU, bminU}, cv[4] = {bminV, bminV, bmaxV, bmaxV};
    V3 corner[4];
    for (int i = 0; i < 4; i++) {
      double x = cu[i] * bdx + cv[i] * px, y = cu[i] * bdy + cv[i] * py;
      corner[i] = {(real)(o.x + u.x * x + v.x * y), (real)(o.y + u.y * x + v.y * y),
                   (real)(o.z + u.z * x + v.z * y)};
    }
    for (int i = 0; i < 4; i++) {
      V3 e = corner[(i + 1) % 4] - corner[i];
      V3 m = Cross(nrm, e);
      real len = std::sqrt(Dot(m, m));
      if (len < real(1e-9)) continue;
      Plane pl;
      pl.n = m * (real(1) / len);
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
// portal file

struct Portal {
  Plane plane;
  int leaf;  // cluster the portal leads into
  int owner; // cluster that lists it
  Winding w;
  V3 origin;
  real radius;
  signed char orient; // +1 when the winding runs counter-clockwise around plane.n
};

struct Map {
  int nc = 0, N = 0;
  std::vector<Portal> portals;
  std::vector<std::vector<int>> cp; // portals owned by each cluster
};

// A half-written --patch-bsp temporary, removed by Die so no failure path can
// leave it behind (set while the temporary exists, cleared once it is renamed).
static std::string g_tempFile;

[[noreturn]] static void Die(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  fprintf(stderr, "prtvis: ");
  vfprintf(stderr, fmt, ap);
  fprintf(stderr, "\n");
  va_end(ap);
  if (!g_tempFile.empty()) remove(g_tempFile.c_str());
  exit(1);
}

static Plane PlaneFromWinding(const Winding &w) {
  V3 v1 = w.p[2] - w.p[1], v2 = w.p[0] - w.p[1];
  Plane pl;
  pl.n = Cross(v2, v1);
  real radius = std::sqrt(Dot(pl.n, pl.n));
  // Stock's FLT_EPSILON guard against a zero-area winding; epsilon() of real is
  // the same value in the float build and does not bias the double normal.
  real iradius = real(1) / (radius + std::numeric_limits<real>::epsilon());
  pl.n = pl.n * iradius;
  pl.d = Dot(w.p[0], pl.n);
  return pl;
}

static void SetPortalSphere(Portal &p) {
  const Winding &w = p.w;
  V3 total = {0, 0, 0};
  for (int i = 0; i < w.n; i++) total = total + w.p[i];
  for (int i = 0; i < 3; i++) total[i] /= w.n;
  real bestr = 0;
  for (int i = 0; i < w.n; i++) {
    V3 dist = w.p[i] - total;
    real r = std::sqrt(Dot(dist, dist));
    if (r > bestr) bestr = r;
  }
  p.origin = total;
  p.radius = bestr;
}

static signed char Orientation(const Winding &w, const V3 &n) {
  double sx = 0, sy = 0, sz = 0;
  for (int i = 1; i + 1 < w.n; i++) {
    double e1x = (double)w.p[i].x - w.p[0].x, e1y = (double)w.p[i].y - w.p[0].y,
           e1z = (double)w.p[i].z - w.p[0].z;
    double e2x = (double)w.p[i + 1].x - w.p[0].x, e2y = (double)w.p[i + 1].y - w.p[0].y,
           e2z = (double)w.p[i + 1].z - w.p[0].z;
    sx += e1y * e2z - e1z * e2y;
    sy += e1z * e2x - e1x * e2z;
    sz += e1x * e2y - e1y * e2x;
  }
  return (sx * n.x + sy * n.y + sz * n.z) < 0 ? -1 : 1;
}

// 64-bit file offsets: long is 32 bits on Windows, so ftell would wrap on a file
// over 2 GB and report a bogus size.
#if defined(_WIN32)
static int Seek64(FILE *f, long long off, int whence) { return _fseeki64(f, off, whence); }
static long long Tell64(FILE *f) { return _ftelli64(f); }
#else
static int Seek64(FILE *f, long long off, int whence) { return fseeko(f, (off_t)off, whence); }
static long long Tell64(FILE *f) { return (long long)ftello(f); }
#endif

// Size of an open file, or -1 when it cannot be queried (a directory, a pipe).
static long long Size64(FILE *f) {
  if (Seek64(f, 0, SEEK_END) != 0) return -1;
  long long sz = Tell64(f);
  if (sz < 0 || Seek64(f, 0, SEEK_SET) != 0) return -1;
  return sz;
}

static const long long MAX_INPUT_BYTES = 0x7fffffffll; // the lump offsets are 32-bit signed ints

// Dies unless the file opens and fits MAX_INPUT_BYTES. Returns its size.
static long long CheckInputSize(const char *path) {
  FILE *f = fopen(path, "rb");
  if (!f) Die("cannot open %s", path);
  long long sz = Size64(f);
  fclose(f);
  if (sz < 0) Die("cannot read the size of %s", path);
  if (sz > MAX_INPUT_BYTES) Die("%s is %lld bytes; files over 2 GB are not supported", path, sz);
  return sz;
}

static std::vector<char> ReadFile(const char *path) {
  long long sz = CheckInputSize(path);
  FILE *f = fopen(path, "rb");
  if (!f) Die("cannot open %s", path);
  std::vector<char> buf((size_t)sz + 1, 0);
  if (sz > 0 && fread(buf.data(), 1, (size_t)sz, f) != (size_t)sz) Die("short read on %s", path);
  fclose(f);
  buf.resize((size_t)sz);
  return buf;
}

static void LoadPortals(const char *path, Map &m) {
  std::vector<char> buf = ReadFile(path);
  buf.push_back(0);
  const char *c = buf.data();
  auto skipws = [&]() {
    while (*c == ' ' || *c == '\t' || *c == '\r' || *c == '\n') c++;
  };
  auto readInt = [&](const char *what) -> long {
    skipws();
    char *e;
    long v = strtol(c, &e, 10);
    if (e == c) Die("portal file: expected an integer (%s)", what);
    c = e;
    return v;
  };
  skipws();
  const char *s = c;
  while (*c && *c != ' ' && *c != '\n' && *c != '\r' && *c != '\t') c++;
  if (c - s != 4 || strncmp(s, "PRT1", 4) != 0) Die("%s is not a PRT1 portal file", path);
  m.nc = (int)readInt("clusters");
  int nfile = (int)readInt("portals");
  m.N = nfile * 2;
  m.portals.assign(m.N, Portal());
  m.cp.assign(m.nc, std::vector<int>());

  for (int i = 0; i < nfile; i++) {
    int np = (int)readInt("points");
    int l0 = (int)readInt("leaf"), l1 = (int)readInt("leaf");
    if (np < 3 || np > MAXW) Die("portal %d has %d points; the tool supports 3..64", i, np);
    if (l0 < 0 || l1 < 0 || l0 >= m.nc || l1 >= m.nc) Die("portal %d: cluster out of range", i);
    Winding w;
    w.n = np;
    for (int j = 0; j < np; j++) {
      skipws();
      if (*c != '(') Die("portal %d: expected '('", i);
      c++;
      double v[3];
      for (int k = 0; k < 3; k++) {
        skipws();
        char *e;
        v[k] = strtod(c, &e);
        if (e == c) Die("portal %d: bad coordinate", i);
        c = e;
      }
      skipws();
      if (*c != ')') Die("portal %d: expected ')'", i);
      c++;
      for (int k = 0; k < 3; k++) w.p[j][k] = (real)v[k]; // double build keeps the parsed double
    }
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
    for (int j = 0; j < np; j++) bw.w.p[j] = w.p[np - 1 - j];
    bw.plane = plane;
    bw.leaf = l0;
    bw.owner = l1;
    SetPortalSphere(bw);
    m.cp[l1].push_back(2 * i + 1);
  }
  for (Portal &p : m.portals) p.orient = Orientation(p.w, p.plane.n);
}

// ---------------------------------------------------------------------------
// threading

template <class F> static void ParallelFor(int count, int nthreads, F fn) {
  std::atomic<int> next(0);
  auto work = [&](int tid) {
    for (;;) {
      int i = next.fetch_add(1);
      if (i >= count) break;
      fn(i, tid);
    }
  };
  int nt = std::max(1, std::min(nthreads, count));
  if (nt == 1) {
    work(0);
    return;
  }
  std::vector<std::thread> th;
  for (int t = 1; t < nt; t++) th.emplace_back(work, t);
  work(0);
  for (auto &t : th) t.join();
}

// ---------------------------------------------------------------------------
// U0: BasePortalVis + SimpleFlood

struct U0Scratch {
  std::vector<u64> front;
  std::vector<char> seen;
  std::vector<int> stack;
};

static void ComputeU0(const Map &m, int base, u64 *flood, U0Scratch &s) {
  const Portal &p = m.portals[base];
  size_t Wd = (m.N + 63) / 64;
  s.front.assign(Wd, 0);
  for (int j = 0; j < m.N; j++) {
    if (j == base) continue;
    const Portal &tp = m.portals[j];
    int k;
    for (k = 0; k < tp.w.n; k++) {
      real d = Dot(tp.w.p[k], p.plane.n) - p.plane.d;
      if (d > g_visEps) break;
    }
    if (k == tp.w.n) continue; // no points on front
    for (k = 0; k < p.w.n; k++) {
      real d = Dot(p.w.p[k], tp.plane.n) - tp.plane.d;
      if (d < -g_visEps) break;
    }
    if (k == p.w.n) continue; // stock: no point behind tp
    SetBit(s.front.data(), j);
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
      if (!GetBit(s.front.data(), r)) continue;
      if (GetBit(flood, r)) continue;
      SetBit(flood, r);
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
  // --might only (empty otherwise, so no allocation): stock's mightsee for this
  // state, the directed portals a chain reaching it can still mark. Over all N.
  std::vector<u64> m;
  bool dirty;
};

// a is a subset of b (both W words).
static inline bool Subset(const u64 *a, const u64 *b, size_t W) {
  for (size_t i = 0; i < W; i++)
    if (a[i] & ~b[i]) return false;
  return true;
}

struct U2Worker {
  const Map &m;
  const BitMat &U0;
  BitMat &U2;
  const std::vector<int> &waveOf;
  bool reuse;
  int K;
  bool might; // --might: carry a mightsee bitset per state
  int curWave = 0;
  Counters cnt;

  std::vector<std::vector<SState>> st;
  std::vector<int> upd, tot;
  std::vector<char> frozen, inq, touched;
  std::vector<int> touchedList, queue;
  std::vector<std::pair<Winding, Winding>> snap;
  size_t qhead = 0;

  // --might scratch. mW is 0 when off, so the default path allocates nothing.
  size_t mW = 0;
  std::vector<u64> mbuf;  // mnew of the hop in progress
  std::vector<u64> snapM; // the snapshotted states' bitsets, mW words each, parallel to snap
  int live = 0;           // states alive in st[] for the current base
  int maxLive = 0;        // largest 'live' over every base this worker ran

  int base = 0;
  const Portal *P = nullptr;
  const u64 *U0P = nullptr;
  u64 *U2P = nullptr;

  U2Worker(const Map &m_, const BitMat &u0, BitMat &u2, const std::vector<int> &wo, bool reuse_, int k,
           bool might_ = false)
      : m(m_), U0(u0), U2(u2), waveOf(wo), reuse(reuse_), K(k), might(might_) {
    st.resize(m.N);
    upd.assign(m.N, 0);
    tot.assign(m.N, 0);
    frozen.assign(m.N, 0);
    inq.assign(m.N, 0);
    touched.assign(m.N, 0);
    if (might) {
      mW = U0.W;
      mbuf.assign(mW, 0);
    }
  }

  const u64 *TestRow(int q) const {
    if (reuse && waveOf[q] < curWave) return U2.row(q);
    return U0.row(q);
  }

  // MTFlowEnter lines 849-872: the pass and source windings for entering r
  // from a state whose source is prevSource.
  bool Step(const Portal &r, const Winding &prevSource, Winding &pass, Winding &source) const {
    real d = Dot(r.origin, P->plane.n);
    d -= P->plane.d;
    if (d < -r.radius) return false;
    Wcopy(pass, r.w);
    if (!(d > r.radius)) {
      if (!Chop(pass, P->plane)) return false;
    }
    d = Dot(P->origin, r.plane.n);
    d -= r.plane.d;
    if (d > P->radius) return false;
    Wcopy(source, prevSource);
    if (!(d < -P->radius)) {
      Plane back;
      back.n = V3{-r.plane.n.x, -r.plane.n.y, -r.plane.n.z};
      back.d = -r.plane.d;
      if (!Chop(source, back)) return false;
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
    if (!Chop(W, P->plane)) Wcopy(W, m.portals[q].w);
    auto &v = st[q];
    if (might) live -= (int)v.size();
    v.clear();
    SState s;
    s.S.assign(P->w.p, P->w.p + P->w.n);
    s.W.assign(W.p, W.p + W.n);
    // the frozen state stands for anything: it may still mark whatever U0(P) allows
    if (might) {
      s.m.assign(U0P, U0P + mW);
      if (++live > maxLive) maxLive = live;
    }
    s.dirty = true;
    v.push_back(std::move(s));
    frozen[q] = 1;
    Enqueue(q);
  }

  // mn: the arrival's mightsee bitset (mW words), or nullptr without --might.
  void Arrive(int q, const Winding &S, const Winding &W, const u64 *mn) {
    cnt.arrivals++;
    Touch(q);
    if (frozen[q]) {
      cnt.dominated++;
      return;
    }
    const V3 &nP = P->plane.n;
    const V3 &nq = m.portals[q].plane.n;
    auto &v = st[q];
    // With --might a state only dominates an arrival that can mark no more than
    // it can, and is only dropped by an arrival that can mark at least as much.
    for (const SState &s : v) {
      if ((!mn || Subset(mn, s.m.data(), mW)) && Contains(s.S, S, nP) && Contains(s.W, W, nq)) {
        cnt.dominated++;
        return;
      }
    }
    size_t keep = 0;
    for (size_t i = 0; i < v.size(); i++) {
      if ((!mn || Subset(v[i].m.data(), mn, mW)) && Contains(S, v[i].S, nP) && Contains(W, v[i].W, nq)) continue;
      if (keep != i) v[keep] = std::move(v[i]);
      keep++;
    }
    if (might) live -= (int)(v.size() - keep);
    v.erase(v.begin() + keep, v.end());

    tot[q]++;
    bool atCap = (K == 0) ? v.size() >= (size_t)K0_CAP : v.size() >= (size_t)K;
    if (!atCap) {
      SState s;
      s.S.assign(S.p, S.p + S.n);
      s.W.assign(W.p, W.p + W.n);
      if (might) {
        s.m.assign(mn, mn + mW);
        if (++live > maxLive) maxLive = live;
      }
      s.dirty = true;
      v.push_back(std::move(s));
      cnt.created++;
    } else {
      if (K == 0) cnt.capHits++;
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
      // the merged state may mark whatever either side could; dirty (always set on
      // a merge) is what makes a grown bitset flow again
      if (might)
        for (size_t i = 0; i < mW; i++) s.m[i] |= mn[i];
      s.dirty = true;
      cnt.merges++;
      upd[q]++;
    }
    if (upd[q] > GUARD_UPDATES || tot[q] > GUARD_ARRIVALS) {
      Freeze(q);
      return;
    }
    Enqueue(q);
  }

  // mq: the state's mightsee bitset with --might, else nullptr.
  void Flow(int q, const Winding &S, const Winding &Wd, const u64 *mq) {
    const Portal &pq = m.portals[q];
    const u64 *tq = TestRow(q);
    for (int r : m.cp[pq.leaf]) {
      const u64 *mn = nullptr;
      if (mq) {
        // r in mq implies both plain gates below (mq is inside U0(P) and inside the row of q)
        if (!GetBit(mq, r)) {
          // count only what the plain gates would have admitted, i.e. the effect of the bitset
          if (GetBit(U0P, r)) {
            if (GetBit(tq, r)) cnt.mightSkips++;
            else if (tq != U0.row(q) && GetBit(U0.row(q), r)) cnt.reusePrunes++;
          }
          continue;
        }
        // stock: might = prevstack->mightsee & test(r); a branch that can mark
        // nothing new is skipped
        const u64 *tr = TestRow(r);
        bool fresh = !GetBit(U2P, r);
        for (size_t i = 0; i < mW; i++) {
          mbuf[i] = mq[i] & tr[i];
          if (mbuf[i] & ~U2P[i]) fresh = true;
        }
        if (!fresh) {
          cnt.nothingNew++;
          continue;
        }
        mn = mbuf.data();
      } else {
        if (!GetBit(U0P, r)) continue;
        if (!GetBit(tq, r)) {
          // counts only r that U0(q) would have let through, i.e. the effect of reuse
          if (tq != U0.row(q) && GetBit(U0.row(q), r)) cnt.reusePrunes++;
          continue;
        }
      }
      cnt.flowSteps++;
      Winding pass, source;
      if (!Step(m.portals[r], S, pass, source)) continue;
      if (!ClipToSeperators(source, Wd, pass, false)) continue;
      if (!ClipToSeperators(Wd, source, pass, true)) continue;
      SetBit(U2P, r);
      Arrive(r, source, pass, mn);
    }
  }

  void Run(int b) {
    base = b;
    P = &m.portals[b];
    U0P = U0.row(b);
    U2P = U2.row(b);
    queue.clear();
    qhead = 0;

    // first hop: no separator clipping, as in stock when the previous pass is NULL
    for (int r : m.cp[P->leaf]) {
      if (!GetBit(U0P, r)) continue;
      Winding pass, source;
      if (!Step(m.portals[r], P->w, pass, source)) continue;
      const u64 *mn = nullptr;
      if (might) {
        const u64 *tr = TestRow(r);
        for (size_t i = 0; i < mW; i++) mbuf[i] = U0P[i] & tr[i];
        mn = mbuf.data();
      }
      SetBit(U2P, r);
      Arrive(r, source, pass, mn);
    }
    while (qhead < queue.size()) {
      int q = queue[qhead++];
      inq[q] = 0;
      snap.clear();
      if (might) snapM.clear();
      for (SState &s : st[q]) {
        if (!s.dirty) continue;
        s.dirty = false;
        snap.emplace_back();
        ToWinding(s.S, snap.back().first);
        ToWinding(s.W, snap.back().second);
        if (might) snapM.insert(snapM.end(), s.m.begin(), s.m.end());
      }
      for (size_t i = 0; i < snap.size(); i++)
        Flow(q, snap[i].first, snap[i].second, might ? snapM.data() + i * mW : nullptr);
    }
    for (int q : touchedList) {
      st[q].clear();
      upd[q] = tot[q] = 0;
      frozen[q] = inq[q] = touched[q] = 0;
    }
    touchedList.clear();
    live = 0;
  }
};

// ---------------------------------------------------------------------------
// L: ray walk

static inline u64 SplitMix(u64 &s) {
  u64 z = (s += 0x9E3779B97F4A7C15ull);
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
  return z ^ (z >> 31);
}
static inline double Rand01(u64 &s) { return (double)(SplitMix(s) >> 11) * (1.0 / 9007199254740992.0); }

struct D3 {
  double x, y, z;
};

static D3 Centroid(const Winding &w) {
  double ax = 0, ay = 0, az = 0, total = 0;
  for (int i = 1; i + 1 < w.n; i++) {
    double e1x = (double)w.p[i].x - w.p[0].x, e1y = (double)w.p[i].y - w.p[0].y,
           e1z = (double)w.p[i].z - w.p[0].z;
    double e2x = (double)w.p[i + 1].x - w.p[0].x, e2y = (double)w.p[i + 1].y - w.p[0].y,
           e2z = (double)w.p[i + 1].z - w.p[0].z;
    double cx = e1y * e2z - e1z * e2y, cy = e1z * e2x - e1x * e2z, cz = e1x * e2y - e1y * e2x;
    double a = 0.5 * sqrt(cx * cx + cy * cy + cz * cz);
    ax += a * ((double)w.p[0].x + w.p[i].x + w.p[i + 1].x) / 3.0;
    ay += a * ((double)w.p[0].y + w.p[i].y + w.p[i + 1].y) / 3.0;
    az += a * ((double)w.p[0].z + w.p[i].z + w.p[i + 1].z) / 3.0;
    total += a;
  }
  if (total > 1e-9) return {ax / total, ay / total, az / total};
  double sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < w.n; i++) {
    sx += w.p[i].x;
    sy += w.p[i].y;
    sz += w.p[i].z;
  }
  return {sx / w.n, sy / w.n, sz / w.n};
}

// Area-weighted point over the triangle fan, pulled 2% toward the centroid.
static D3 RandomPoint(const Winding &w, const D3 &cen, u64 &rng) {
  double areas[MAXW];
  double total = 0;
  for (int i = 1; i + 1 < w.n; i++) {
    double e1x = (double)w.p[i].x - w.p[0].x, e1y = (double)w.p[i].y - w.p[0].y,
           e1z = (double)w.p[i].z - w.p[0].z;
    double e2x = (double)w.p[i + 1].x - w.p[0].x, e2y = (double)w.p[i + 1].y - w.p[0].y,
           e2z = (double)w.p[i + 1].z - w.p[0].z;
    double cx = e1y * e2z - e1z * e2y, cy = e1z * e2x - e1x * e2z, cz = e1x * e2y - e1y * e2x;
    areas[i] = 0.5 * sqrt(cx * cx + cy * cy + cz * cz);
    total += areas[i];
  }
  double u1 = Rand01(rng), u2 = Rand01(rng), pick = Rand01(rng) * total;
  if (total <= 1e-12) return cen;
  int tri = w.n - 2;
  double acc = 0;
  for (int i = 1; i + 1 < w.n; i++) {
    acc += areas[i];
    if (pick <= acc) {
      tri = i;
      break;
    }
  }
  double r1 = sqrt(u1);
  double wa = 1 - r1, wb = r1 * (1 - u2), wc = r1 * u2;
  const V3 &A = w.p[0], &B = w.p[tri], &C = w.p[tri + 1];
  D3 p = {wa * A.x + wb * B.x + wc * C.x, wa * A.y + wb * B.y + wc * C.y,
          wa * A.z + wb * B.z + wc * C.z};
  return {p.x + 0.02 * (cen.x - p.x), p.y + 0.02 * (cen.y - p.y), p.z + 0.02 * (cen.z - p.z)};
}

// Hit point at least margin inside every edge of the portal's winding.
static bool InsideMargin(const Portal &r, const D3 &h, double margin) {
  const Winding &w = r.w;
  const double sgn = r.orient;
  for (int i = 0; i < w.n; i++) {
    const V3 &a = w.p[i], &b = w.p[(i + 1) % w.n];
    double ex = (double)b.x - a.x, ey = (double)b.y - a.y, ez = (double)b.z - a.z;
    double l2 = ex * ex + ey * ey + ez * ez;
    if (l2 < 1e-12) continue;
    double inv = sgn / sqrt(l2);
    double mx = ((double)r.plane.n.y * ez - (double)r.plane.n.z * ey) * inv;
    double my = ((double)r.plane.n.z * ex - (double)r.plane.n.x * ez) * inv;
    double mz = ((double)r.plane.n.x * ey - (double)r.plane.n.y * ex) * inv;
    double dist = mx * (h.x - a.x) + my * (h.y - a.y) + mz * (h.z - a.z);
    if (dist < margin) return false;
  }
  return true;
}

struct RayWorker {
  const Map &m;
  const BitMat &U0;
  BitMat &L;
  int nrays;
  double margin;
  Counters cnt;
  std::vector<int> stamp;
  int cur = 0;
  std::vector<std::pair<int, double>> stack;
  std::vector<D3> cen;

  RayWorker(const Map &m_, const BitMat &u0, BitMat &l, int n, double mg, const std::vector<D3> &c)
      : m(m_), U0(u0), L(l), nrays(n), margin(mg), cen(c) {
    stamp.assign(m.N, 0);
  }

  void Walk(int P, const D3 &a, const D3 &d, u64 *LP) {
    cnt.rays++;
    cur++;
    stack.clear();
    stack.push_back({m.portals[P].leaf, 0.0});
    int pushes = 1;
    bool capped = false;
    while (!stack.empty()) {
      std::pair<int, double> top = stack.back();
      stack.pop_back();
      for (int ri : m.cp[top.first]) {
        const Portal &r = m.portals[ri];
        double dn = d.x * r.plane.n.x + d.y * r.plane.n.y + d.z * r.plane.n.z;
        if (!(dn > 1e-6)) continue;
        double t = ((double)r.plane.d - (a.x * r.plane.n.x + a.y * r.plane.n.y + a.z * r.plane.n.z)) / dn;
        if (!(t > top.second + 0.01)) continue;
        if (stamp[ri] == cur) continue;
        D3 h = {a.x + d.x * t, a.y + d.y * t, a.z + d.z * t};
        if (!InsideMargin(r, h, margin)) continue;
        stamp[ri] = cur;
        SetBit(LP, ri);
        if (pushes < 4096) {
          stack.push_back({r.leaf, t});
          pushes++;
        } else {
          capped = true; // the crossing is still marked, but nothing beyond it is walked
        }
      }
    }
    if (capped) cnt.rayCapHits++;
  }

  void Run(int P) {
    const Portal &pp = m.portals[P];
    const u64 *U0P = U0.row(P);
    u64 *LP = L.row(P);
    ForEachBit(U0P, U0.W, [&](int Q) {
      if (GetBit(LP, Q)) return;
      const Portal &qq = m.portals[Q];
      for (int s = 0; s < nrays; s++) {
        D3 a, b;
        if (s == 0) {
          a = cen[P];
          b = cen[Q];
        } else {
          u64 h = 0xcbf29ce484222325ull;
          for (u64 v : {(u64)P + 1, (u64)Q + 1, (u64)s + 1}) {
            h ^= v;
            h *= 0x100000001b3ull;
            h ^= h >> 29;
          }
          u64 rng = h;
          SplitMix(rng);
          a = RandomPoint(pp.w, cen[P], rng);
          b = RandomPoint(qq.w, cen[Q], rng);
        }
        D3 d = {b.x - a.x, b.y - a.y, b.z - a.z};
        double len = sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
        if (len < 1e-9) continue;
        d = {d.x / len, d.y / len, d.z / len};
        if (!(d.x * pp.plane.n.x + d.y * pp.plane.n.y + d.z * pp.plane.n.z > 1e-6)) continue;
        if (!(d.x * qq.plane.n.x + d.y * qq.plane.n.y + d.z * qq.plane.n.z > 1e-6)) continue;
        Walk(P, a, d, LP);
        if (GetBit(LP, Q)) break;
      }
    });
  }
};

// ---------------------------------------------------------------------------
// cluster level

// ClusterMerge: OR a family's portal rows per cluster, add the portals
// themselves, map portal -> the cluster it leads into, add the cluster.
// preTotal is VVIS's "Total clusters visible".
static void BuildClusterMatrix(const Map &m, const BitMat &fam, BitMat &cm, long long &preTotal) {
  cm.init(m.nc, m.nc);
  std::vector<u64> pv(fam.W);
  preTotal = 0;
  for (int c = 0; c < m.nc; c++) {
    std::fill(pv.begin(), pv.end(), 0);
    for (int p : m.cp[c]) {
      const u64 *r = fam.row(p);
      for (size_t i = 0; i < fam.W; i++) pv[i] |= r[i];
      SetBit(pv.data(), p);
    }
    u64 *row = cm.row(c);
    ForEachBit(pv.data(), fam.W, [&](int p) {
      if (p < m.N) SetBit(row, m.portals[p].leaf);
    });
    long long numvis = CountRow(row, cm.W);
    SetBit(row, c);
    numvis++;
    preTotal += numvis;
  }
}

// CompressAndCrosscheckClusterVis: clear i from row c when c is not in row i.
// The stock loop works in place, but its result is the symmetric AND.
static void CrossCheck(const BitMat &in, BitMat &out, int nc) {
  out = in;
  for (int c = 0; c < nc; c++) {
    ForEachBit(in.row(c), in.W, [&](int i) {
      if (i == c || i >= nc) return;
      if (!GetBit(in.row(i), c)) ClrBit(out.row(c), i);
    });
  }
}

// which: 0 reads the PVS rows (bitofs[c][DVIS_PVS]), 1 the PAS rows. The PAS is
// only read back by the --patch-bsp self-check.
static bool LoadExact(const char *path, int nc, BitMat &E, int which = 0) {
  const char *rowName = which ? "PAS" : "PVS";
  std::vector<char> f = ReadFile(path);
  if (f.size() < 8 + 64 * 16) Die("%s is too small to be a BSP", path);
  auto geti = [&](size_t off) {
    int v;
    memcpy(&v, f.data() + off, 4);
    return v;
  };
  if (geti(0) != 0x50534256) Die("%s is not a VBSP file", path);
  size_t lumpOff = 8 + 4 * 16; // LUMP_VISIBILITY = 4
  int fileofs = geti(lumpOff), filelen = geti(lumpOff + 4), uncompressed = geti(lumpOff + 12);
  if (fileofs < 0 || filelen < 4 || (size_t)fileofs + (size_t)filelen > f.size())
    Die("%s: visibility lump is missing or out of range", path);
  const unsigned char *lump = (const unsigned char *)f.data() + fileofs;
  if (uncompressed != 0 || memcmp(lump, "LZMA", 4) == 0)
    Die("%s: the visibility lump is LZMA-compressed; decompress the BSP first", path);
  int n;
  memcpy(&n, lump, 4);
  if (n != nc) Die("%s has %d clusters, the portal file has %d", path, n, nc);
  int row = (nc + 7) >> 3;
  if (filelen < 4 + 8 * nc) Die("%s: visibility lump is truncated", path);
  E.init(nc, nc);
  std::vector<unsigned char> dec(E.W * 8, 0);
  for (int c = 0; c < nc; c++) {
    int ofs;
    memcpy(&ofs, lump + 4 + 8 * c + 4 * which, 4); // bitofs[c][DVIS_PVS or DVIS_PAS]
    if (ofs < 0 || ofs >= filelen) Die("%s: bad %s offset for a cluster", path, rowName);
    const unsigned char *in = lump + ofs;
    const unsigned char *inEnd = lump + filelen;
    std::fill(dec.begin(), dec.end(), 0);
    int out = 0;
    while (out < row) { // DecompressVis
      if (in >= inEnd) Die("%s: %s row runs past the lump", path, rowName);
      if (*in) {
        dec[out++] = *in++;
        continue;
      }
      if (in + 1 >= inEnd) Die("%s: %s row runs past the lump", path, rowName);
      int cnt = in[1];
      if (!cnt) Die("%s: DecompressVis: 0 repeat", path);
      in += 2;
      if (out + cnt > row) cnt = row - out;
      out += cnt; // zeros already in place
    }
    memcpy(E.row(c), dec.data(), E.W * 8);
  }
  return true;
}

struct Cmp {
  long long x = 0, e = 0, inter = 0, xOnly = 0, eOnly = 0;
};
static Cmp Compare(const BitMat &X, const BitMat &E) {
  Cmp r;
  for (size_t i = 0; i < X.d.size(); i++) {
    r.x += Pop(X.d[i]);
    r.e += Pop(E.d[i]);
    r.inter += Pop(X.d[i] & E.d[i]);
    r.xOnly += Pop(X.d[i] & ~E.d[i]);
    r.eOnly += Pop(E.d[i] & ~X.d[i]);
  }
  return r;
}

static void WriteMat(const std::string &path, const BitMat &m, int nc) {
  FILE *f = fopen(path.c_str(), "wb");
  if (!f) Die("cannot write %s", path.c_str());
  int row = (nc + 7) >> 3;
  for (int c = 0; c < nc; c++) fwrite(m.row(c), 1, (size_t)row, f);
  fclose(f);
}

// ---------------------------------------------------------------------------
// --patch-bsp: the post-cross-check U2 cluster rows as the visibility lump of a
// copy of a compiled .bsp, so the map can be loaded in the game with them.
//
// The copy is the input with the new lump added and lump 4 of the header pointed
// at it; the old vis lump stays behind as dead bytes. Only the pakfile may move:
// the game lump (static props) stores absolute file offsets, so every other lump
// must stay where it is. bsplib.cpp keeps LUMP_PAKFILE physically last, so when
// the input has it last the new lump goes in front of it:
//   [input up to the pakfile] [pad to 4] [new vis lump] [pad to 4] [pakfile]
// and lump 40's offset is the only other header field that changes. Otherwise
// (no pakfile, or something after it) the lump is appended after the input,
// zero-padded to 4 bytes, and nothing moves.

static const int LUMP_VISIBILITY = 4;
static const int LUMP_LEAFS = 10;
static const int LUMP_PAKFILE = 40;
static const int MAX_CLUSTERS = 65536;
// sizeof(dleaf_version_0_t) and sizeof(dleaf_t) in bspfile.h: the version-1 leaf
// drops the 24-byte ambient cube of version 0 and is padded to 32. The cluster
// (a signed short) is at byte 4 in both.
static const size_t LEAF_SIZE_V0 = 56, LEAF_SIZE_V1 = 32;
static const size_t BSP_HEADER = 8 + 64 * 16 + 4; // dheader_t: ident, version, 64 lumps, mapRevision
static const size_t MAX_MAP_VISIBILITY = 0x1000000; // bspfile.h; VVIS and VRAD refuse a bigger lump

static size_t LumpEntry(int i) { return 8 + 16 * (size_t)i; } // lump_t: fileofs, filelen, version, uncompressedSize
static uint32_t Rd32(const unsigned char *p) {
  return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}
static void Wr32(unsigned char *p, uint32_t v) {
  for (int i = 0; i < 4; i++) p[i] = (unsigned char)(v >> (8 * i));
}
static const unsigned char *Bytes(const std::vector<char> &v) { return (const unsigned char *)v.data(); }

// Lexical only. std::filesystem needs -lstdc++fs on g++ 8, which would break the
// documented g++ -std=c++17 -pthread build; what this cannot see (relative vs
// absolute, links, 8.3 names) is caught by the byte compare in ReadPatchInput.
static std::string NormPath(std::string s) {
#ifdef _WIN32
  for (char &ch : s) {
    ch = (char)tolower((unsigned char)ch);
    if (ch == '\\') ch = '/';
  }
#endif
  bool rooted = !s.empty() && s[0] == '/';
  std::vector<std::string> seg;
  for (size_t i = 0; i <= s.size();) {
    size_t j = s.find('/', i);
    if (j == std::string::npos) j = s.size();
    std::string t = s.substr(i, j - i);
    i = j + 1;
    if (t.empty() || t == ".") continue;
    if (t == ".." && !seg.empty() && seg.back() != "..") seg.pop_back();
    else if (t == ".." && rooted) continue; // above the root stays at the root
    else seg.push_back(t);
  }
  std::string r = rooted ? "/" : "";
  for (size_t i = 0; i < seg.size(); i++) r += (i ? "/" : "") + seg[i];
  return r;
}

static bool SameBytes(const std::string &path, const std::vector<char> &ref) {
  FILE *f = fopen(path.c_str(), "rb");
  if (!f) return false;
  bool same = false;
  if (Size64(f) == (long long)ref.size()) {
    same = true;
    std::vector<char> buf(1 << 20);
    for (size_t off = 0; same && off < ref.size();) {
      size_t n = std::min(buf.size(), ref.size() - off);
      same = fread(buf.data(), 1, n, f) == n && memcmp(buf.data(), ref.data() + off, n) == 0;
      off += n;
    }
  }
  fclose(f);
  return same;
}

// Two files with the same bytes. A file that is missing or unreadable is the
// same as nothing, so a not-yet-existing output never clashes.
static bool SameContent(const std::string &a, const std::string &b) {
  FILE *fa = fopen(a.c_str(), "rb");
  if (!fa) return false;
  FILE *fb = fopen(b.c_str(), "rb");
  if (!fb) {
    fclose(fa);
    return false;
  }
  long long sa = Size64(fa), sb = Size64(fb);
  bool same = sa >= 0 && sa == sb;
  std::vector<char> x(1 << 20), y(1 << 20);
  for (long long off = 0; same && off < sa;) {
    size_t n = (size_t)std::min<long long>((long long)x.size(), sa - off);
    same = fread(x.data(), 1, n, fa) == n && fread(y.data(), 1, n, fb) == n && memcmp(x.data(), y.data(), n) == 0;
    off += (long long)n;
  }
  fclose(fa);
  fclose(fb);
  return same;
}

// Refuses an --out-bsp that would overwrite another of the run's files: the same
// name (lexical), or the same bytes under another name (a copy, a relative vs an
// absolute path, a link). The temporary the write goes through is held to the
// same standard, since it is created next to the output.
static void RefuseSameFile(const char *outFlag, const std::string &outPath, const char *otherFlag,
                           const std::string &otherPath) {
  if (otherPath.empty()) return;
  std::string o = NormPath(outPath), t = NormPath(outPath + ".tmp"), x = NormPath(otherPath);
  if (o == x)
    Die("%s and %s name the same file (%s); the %s is never overwritten", outFlag, otherFlag, otherPath.c_str(),
        otherFlag);
  if (t == x)
    Die("the temporary %s.tmp of %s is the %s %s; pick another output name", outPath.c_str(), outFlag, otherFlag,
        otherPath.c_str());
  if (SameContent(outPath, otherPath))
    Die("%s %s already holds exactly the bytes of the %s %s (the same file under another name, or a copy); "
        "refusing to write over it",
        outFlag, outPath.c_str(), otherFlag, otherPath.c_str());
}

// Reads and checks the BSP to patch before any compute, so a wrong file fails
// in a second and not after the flow has run.
static std::vector<char> ReadPatchInput(const std::string &inPath, const std::string &outPath, int nc) {
  if (nc > MAX_CLUSTERS)
    Die("the portal file has %d clusters; --patch-bsp handles at most %d (the BSP stores cluster numbers in 16 bits)",
        nc, MAX_CLUSTERS);
  std::vector<char> in = ReadFile(inPath.c_str());
  const unsigned char *b = Bytes(in);
  if (in.size() < BSP_HEADER) Die("%s is too small to be a BSP", inPath.c_str());
  if (Rd32(b) != 0x50534256) Die("%s is not a VBSP file", inPath.c_str());
  uint32_t ver = Rd32(b + 4);
  if (ver < 19 || ver > 21) Die("%s is BSP version %u; --patch-bsp handles 19, 20 and 21", inPath.c_str(), ver);
  // The self-check compares every lump byte for byte, so each has to be readable.
  for (int i = 0; i < 64; i++) {
    uint32_t ofs = Rd32(b + LumpEntry(i)), len = Rd32(b + LumpEntry(i) + 4);
    if (len != 0 && (uint64_t)ofs + len > in.size())
      Die("%s: lump %d (offset %u, length %u) runs past the end of the file", inPath.c_str(), i, ofs, len);
  }
  size_t e = LumpEntry(LUMP_VISIBILITY);
  uint32_t vofs = Rd32(b + e), vlen = Rd32(b + e + 4), vunc = Rd32(b + e + 12);
  if (vlen == 0) {
    fprintf(stderr,
            "prtvis: warning: %s has an empty visibility lump. VVIS also writes the fog-volume flags and the\n"
            "        leaf-to-water distances, which depend on visibility, so the input should have been through\n"
            "        'vvis -fast' first; continuing without them.\n",
            inPath.c_str());
  } else {
    if (vlen < 4) Die("%s: the visibility lump is truncated", inPath.c_str());
    if (vunc != 0 || memcmp(b + vofs, "LZMA", 4) == 0)
      Die("%s: the visibility lump is LZMA-compressed; decompress the BSP first", inPath.c_str());
    uint32_t n = Rd32(b + vofs);
    if (n != (uint32_t)nc)
      Die("%s has %d clusters in its visibility lump, the portal file has %d: wrong BSP for this PRT",
          inPath.c_str(), (int)n, nc);
  }

  // The leaves say how many clusters the BSP uses, with or without a vis lump
  // to ask: a PRT with fewer clusters than the leaves reference is the wrong pair.
  size_t le = LumpEntry(LUMP_LEAFS);
  uint32_t lofs = Rd32(b + le), llen = Rd32(b + le + 4), lver = Rd32(b + le + 8);
  if (lver > 1)
    Die("%s: LUMP_LEAFS (lump %d) has version %u; --patch-bsp reads 0 and 1", inPath.c_str(), LUMP_LEAFS, lver);
  size_t lsz = lver == 0 ? LEAF_SIZE_V0 : LEAF_SIZE_V1;
  if (llen % lsz != 0)
    Die("%s: LUMP_LEAFS (lump %d, version %u) is %u bytes, not a multiple of the %zu-byte leaf", inPath.c_str(),
        LUMP_LEAFS, lver, llen, lsz);
  int maxCluster = -1; // -1 is "no cluster" (solid leaves)
  for (size_t k = 0; k < llen / lsz; k++) {
    const unsigned char *lf = b + lofs + k * lsz;
    maxCluster = std::max(maxCluster, (int)(int16_t)(uint16_t)(lf[4] | lf[5] << 8));
  }
  if (maxCluster + 1 > nc)
    Die("%s has leaves in cluster %d, so at least %d clusters, and the portal file has %d: wrong BSP for this PRT",
        inPath.c_str(), maxCluster, maxCluster + 1, nc);
  if (maxCluster + 1 < nc)
    fprintf(stderr,
            "prtvis: warning: the leaves of %s use %d clusters (highest cluster %d) and the portal file has %d; "
            "continuing\n",
            inPath.c_str(), maxCluster + 1, maxCluster, nc);

  if (SameBytes(outPath, in))
    Die("--out-bsp %s already holds exactly the bytes of %s (the same file under another name, or a copy); "
        "refusing to write over it",
        outPath.c_str(), inPath.c_str());
  return in;
}

// CalcPAS: row c = the OR of the PVS rows of every cluster set in PVS row c.
static void BuildPas(const BitMat &pvs, BitMat &pas, int nc) {
  pas.init(nc, nc);
  for (int c = 0; c < nc; c++) {
    u64 *dst = pas.row(c);
    memcpy(dst, pvs.row(c), pvs.W * 8);
    ForEachBit(pvs.row(c), pvs.W, [&](int i) {
      const u64 *src = pvs.row(i);
      for (size_t w = 0; w < pvs.W; w++) dst[w] |= src[w];
    });
  }
}

// The row as the byte string VVIS compresses: bit i is bit (i & 7) of byte i >> 3.
static void RowBytes(const u64 *r, int rowBytes, unsigned char *out) {
  for (int j = 0; j < rowBytes; j++) out[j] = (unsigned char)(r[j >> 3] >> ((j & 7) * 8));
}

// CompressVis (bsplib.cpp): a zero byte is followed by its run length (at most 255).
static size_t CompressVisRow(const unsigned char *vis, int visrow, unsigned char *dest) {
  unsigned char *p = dest;
  for (int j = 0; j < visrow; j++) {
    *p++ = vis[j];
    if (vis[j]) continue;
    int rep = 1;
    for (j++; j < visrow; j++) {
      if (vis[j] || rep == 255) break;
      rep++;
    }
    *p++ = (unsigned char)rep;
    j--;
  }
  return (size_t)(p - dest);
}

// dvis_t: numclusters, bitofs[nc][2] (byte offsets from the start of the lump),
// then every PVS row in cluster order, then every PAS row, as VVIS lays it out.
static std::vector<unsigned char> BuildVisLump(const BitMat &pvs, const BitMat &pas, int nc) {
  int row = (nc + 7) >> 3;
  std::vector<unsigned char> lump(4 + 8 * (size_t)nc, 0);
  Wr32(lump.data(), (uint32_t)nc);
  std::vector<unsigned char> raw((size_t)row), comp(2 * (size_t)row + 2);
  for (int which = 0; which < 2; which++) {
    const BitMat &mat = which ? pas : pvs;
    for (int c = 0; c < nc; c++) {
      RowBytes(mat.row(c), row, raw.data());
      size_t n = CompressVisRow(raw.data(), row, comp.data());
      Wr32(lump.data() + 4 + 8 * (size_t)c + 4 * which, (uint32_t)lump.size());
      lump.insert(lump.end(), comp.begin(), comp.begin() + (std::ptrdiff_t)n);
    }
  }
  return lump;
}

static void Problem(int &bad, const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  fprintf(stderr, "prtvis: self-check FAILED: ");
  vfprintf(stderr, fmt, ap);
  fprintf(stderr, "\n");
  va_end(ap);
  bad++;
}

static size_t FirstDiff(const unsigned char *x, const unsigned char *y, size_t n) {
  for (size_t i = 0; i < n; i++)
    if (x[i] != y[i]) return i;
  return n;
}

// Compares the written matrix rows with the rows read back from the file.
static void CheckRows(int &bad, const char *name, const BitMat &want, const BitMat &got, int nc) {
  int rowsBad = 0;
  for (int c = 0; c < nc; c++) {
    if (memcmp(want.row(c), got.row(c), want.W * 8) == 0) continue;
    if (rowsBad++ < 5) {
      long long wantBits = CountRow(want.row(c), want.W), gotBits = CountRow(got.row(c), got.W), diff = 0;
      for (size_t w = 0; w < want.W; w++) diff += Pop(want.row(c)[w] ^ got.row(c)[w]);
      Problem(bad, "%s row of cluster %d: written %lld bits, read back %lld bits, %lld bits differ", name, c,
              wantBits, gotBits, diff);
    }
  }
  if (rowsBad) Problem(bad, "%d of %d %s rows differ after reading the file back", rowsBad, nc, name);
}

// Where everything goes in the output. pakLast: the new lump is put in front of
// the pakfile, which moves up behind it. Otherwise the lump is appended after
// the whole input and nothing moves.
struct Layout {
  bool pakLast = false;
  size_t pakOfs = 0, pakLen = 0; // lump 40 in the input (pakLast only)
  size_t prefixEnd = 0;          // input bytes [0, prefixEnd) are copied unchanged
  size_t lumpOfs = 0;            // the new visibility lump
  size_t newPakOfs = 0;          // the pakfile in the output (pakLast only)
  size_t outSize = 0;
};

static size_t Align4(size_t n) { return (n + 3) & ~(size_t)3; }

// True when the input keeps its pakfile physically last, the way bsplib.cpp
// writes it, so the output can too. Otherwise says in why what is different.
static bool PakIsLast(const std::vector<char> &in, std::string &why) {
  const unsigned char *b = Bytes(in);
  size_t pe = LumpEntry(LUMP_PAKFILE);
  size_t ofs = Rd32(b + pe), len = Rd32(b + pe + 4);
  if (len == 0) {
    why = "the input has no pakfile (lump 40 is empty)";
    return false;
  }
  if (ofs < BSP_HEADER) {
    why = "lump 40 starts inside the header";
    return false;
  }
  size_t end = ofs + len; // ReadPatchInput checked that this is within the file
  bool zeros = true;
  for (size_t i = end; i < in.size(); i++) zeros = zeros && b[i] == 0;
  if (in.size() - end > 3 || !zeros) {
    why = "the pakfile (lump 40) is not the last thing in the file";
    return false;
  }
  for (int i = 0; i < 64; i++) {
    if (i == LUMP_VISIBILITY || i == LUMP_PAKFILE) continue;
    size_t o = Rd32(b + LumpEntry(i)), l = Rd32(b + LumpEntry(i) + 4);
    if (l != 0 && o + l > ofs) {
      char buf[96];
      snprintf(buf, sizeof buf, "lump %d reaches into or past the pakfile", i);
      why = buf;
      return false;
    }
  }
  return true;
}

static Layout PlanLayout(const std::vector<char> &in, size_t lumpLen, bool pakLast) {
  Layout L;
  L.pakLast = pakLast;
  if (pakLast) {
    size_t pe = LumpEntry(LUMP_PAKFILE);
    L.pakOfs = Rd32(Bytes(in) + pe);
    L.pakLen = Rd32(Bytes(in) + pe + 4);
    L.prefixEnd = L.pakOfs;
  } else {
    L.prefixEnd = in.size();
  }
  L.lumpOfs = Align4(L.prefixEnd);
  if (pakLast) {
    L.newPakOfs = Align4(L.lumpOfs + lumpLen);
    L.outSize = L.newPakOfs + L.pakLen;
  } else {
    L.outSize = L.lumpOfs + lumpLen;
  }
  return L;
}

// Reads the written file back. Returns the number of problems, each printed.
static int VerifyPatchedBsp(const std::string &outPath, const std::vector<char> &in,
                            const std::vector<unsigned char> &lump, const Layout &L, const BitMat &pvs,
                            const BitMat &pas, int nc) {
  int bad = 0;
  std::vector<char> outv = ReadFile(outPath.c_str());
  const unsigned char *a = Bytes(in), *b = Bytes(outv);
  if (outv.size() != L.outSize) {
    Problem(bad, "file is %zu bytes, expected %zu", outv.size(), L.outSize);
    return bad;
  }
  // The ident, version and map revision around the lump table.
  size_t tableEnd = LumpEntry(64);
  size_t d = FirstDiff(a, b, 8);
  if (d < 8) Problem(bad, "header byte %zu changed", d);
  d = FirstDiff(a + tableEnd, b + tableEnd, BSP_HEADER - tableEnd);
  if (d < BSP_HEADER - tableEnd) Problem(bad, "header byte %zu changed", tableEnd + d);
  // Lump 4 points at the new data and is stored uncompressed.
  size_t e = LumpEntry(LUMP_VISIBILITY);
  if (Rd32(b + e) != L.lumpOfs || Rd32(b + e + 4) != lump.size())
    Problem(bad, "lump 4 entry is offset %u length %u, expected %zu and %zu", Rd32(b + e), Rd32(b + e + 4),
            L.lumpOfs, lump.size());
  if (Rd32(b + e + 8) != Rd32(a + e + 8)) Problem(bad, "lump 4 version changed");
  if (Rd32(b + e + 12) != 0) Problem(bad, "lump 4 uncompressedSize is not 0");
  // Every other lump: same length, version and size field, same bytes at its
  // offset. The offset is the same too, except the pakfile's when it moved.
  for (int i = 0; i < 64; i++) {
    if (i == LUMP_VISIBILITY) continue;
    size_t en = LumpEntry(i);
    uint32_t ofs = Rd32(a + en), len = Rd32(a + en + 4);
    uint32_t want = (i == LUMP_PAKFILE && L.pakLast) ? (uint32_t)L.newPakOfs : ofs;
    if (Rd32(b + en) != want) Problem(bad, "lump %d offset is %u, expected %u", i, Rd32(b + en), want);
    if (memcmp(a + en + 4, b + en + 4, 12) != 0) Problem(bad, "lump %d length, version or size field changed", i);
    if (len && (uint64_t)Rd32(b + en) + len <= outv.size() && memcmp(a + ofs, b + Rd32(b + en), len) != 0)
      Problem(bad, "lump %d bytes changed", i);
  }
  // Everything before the new lump is the input, bar the entries rewritten above.
  d = FirstDiff(a + BSP_HEADER, b + BSP_HEADER, L.prefixEnd - BSP_HEADER);
  if (d < L.prefixEnd - BSP_HEADER) Problem(bad, "input byte %zu changed", BSP_HEADER + d);
  for (size_t i = L.prefixEnd; i < L.lumpOfs; i++)
    if (b[i]) Problem(bad, "padding byte %zu is not zero", i);
  d = FirstDiff(lump.data(), b + L.lumpOfs, lump.size());
  if (d < lump.size()) Problem(bad, "new lump differs from the one built, from byte %zu", d);
  if (L.pakLast) {
    for (size_t i = L.lumpOfs + lump.size(); i < L.newPakOfs; i++)
      if (b[i]) Problem(bad, "padding byte %zu is not zero", i);
    if (Rd32(b + LumpEntry(LUMP_PAKFILE)) + (uint64_t)L.pakLen != outv.size())
      Problem(bad, "the pakfile does not end the file");
  }
  BitMat pvs2, pas2;
  LoadExact(outPath.c_str(), nc, pvs2, 0);
  LoadExact(outPath.c_str(), nc, pas2, 1);
  CheckRows(bad, "PVS", pvs, pvs2, nc);
  CheckRows(bad, "PAS", pas, pas2, nc);
  return bad;
}

// Builds the lump from the post-cross-check U2 rows, writes the copy through a
// temporary, checks it, and only then puts it at the output path. Returns the
// process exit code.
static int PatchBsp(const std::string &outPath, const std::vector<char> &in, const BitMat &pvs, long long preTotal,
                    int nc) {
  BitMat pas;
  BuildPas(pvs, pas, nc);
  std::vector<unsigned char> lump = BuildVisLump(pvs, pas, nc);
  if (lump.size() > MAX_MAP_VISIBILITY)
    Die("the visibility lump would be %zu bytes, over MAX_MAP_VISIBILITY (%zu bytes); nothing written",
        lump.size(), MAX_MAP_VISIBILITY);
  std::string why;
  bool pakLast = PakIsLast(in, why);
  if (!pakLast)
    fprintf(stderr, "prtvis: warning: %s; the new visibility lump is appended at the end of the file\n",
            why.c_str());
  Layout L = PlanLayout(in, lump.size(), pakLast);
  if (L.outSize > (size_t)MAX_INPUT_BYTES) Die("the patched BSP would be over 2 GB; nothing written");

  unsigned char hdr[BSP_HEADER];
  memcpy(hdr, in.data(), BSP_HEADER);
  size_t e = LumpEntry(LUMP_VISIBILITY);
  Wr32(hdr + e, (uint32_t)L.lumpOfs);
  Wr32(hdr + e + 4, (uint32_t)lump.size());
  Wr32(hdr + e + 12, 0); // the old fourCC / uncompressedSize field: 0 = stored as is; version (e + 8) is kept
  if (L.pakLast) Wr32(hdr + LumpEntry(LUMP_PAKFILE), (uint32_t)L.newPakOfs); // its length and the rest stay

  // The write goes to <out>.tmp and only a copy that passes the self-check is
  // renamed onto the output, so a failure never leaves a half-written output,
  // and never touches one that was already there.
  std::string tmp = outPath + ".tmp";
  FILE *f = fopen(tmp.c_str(), "wb");
  if (!f) Die("cannot write %s", tmp.c_str());
  g_tempFile = tmp; // Die removes it from here on
  static const unsigned char zeros[4] = {0, 0, 0, 0};
  auto put = [&](const void *p, size_t n) { return n == 0 || fwrite(p, 1, n, f) == n; };
  bool ok = put(hdr, BSP_HEADER);
  ok = ok && put(in.data() + BSP_HEADER, L.prefixEnd - BSP_HEADER);
  ok = ok && put(zeros, L.lumpOfs - L.prefixEnd);
  ok = ok && put(lump.data(), lump.size());
  if (L.pakLast) {
    ok = ok && put(zeros, L.newPakOfs - (L.lumpOfs + lump.size()));
    ok = ok && put(in.data() + L.pakOfs, L.pakLen);
  }
  ok = (fclose(f) == 0) && ok;
  if (!ok) Die("short write on %s; nothing written to %s", tmp.c_str(), outPath.c_str());

  int bad = VerifyPatchedBsp(tmp, in, lump, L, pvs, pas, nc);
  if (bad) {
    fprintf(stderr, "prtvis: %d self-check problem(s); nothing written to %s\n", bad, outPath.c_str());
    remove(tmp.c_str());
    g_tempFile.clear();
    return 1;
  }
  // rename() does not replace an existing file on Windows, so clear the way if needed.
  if (rename(tmp.c_str(), outPath.c_str()) != 0) {
    if (remove(outPath.c_str()) != 0)
      Die("cannot rename %s over %s; the existing output (if any) is untouched", tmp.c_str(), outPath.c_str());
    if (rename(tmp.c_str(), outPath.c_str()) != 0) {
      g_tempFile.clear(); // the old output is gone, so the checked copy is all there is: keep it
      Die("removed the old %s but cannot rename %s to it; the checked copy is left at %s", outPath.c_str(),
          tmp.c_str(), tmp.c_str());
    }
  }
  g_tempFile.clear();
  printf("patched bsp: %s  vis lump %zu bytes  layout %s  avg clusters visible %lld (pre-cross-check %lld)  "
         "avg clusters audible %lld  self-check ok\n",
         outPath.c_str(), lump.size(), L.pakLast ? "pakfile-last" : "appended", CountMat(pvs) / nc, preTotal / nc,
         CountMat(pas) / nc);
  return 0;
}

// ---------------------------------------------------------------------------
// --trace P,T: finds a ray from base portal P that crosses portal T, then
// replays that one portal chain through the stock clip steps and prints where
// the chain dies and how far inside each window the ray is. This is the tool
// for a "ray-proven but not in the flow" portal.

// Least signed distance from h to the edges of convex w (positive = inside).
static double InsideDist(const Winding &w, const V3 &nrm, const D3 &h) {
  if (w.n < 3) return -1e30;
  double sx = 0, sy = 0, sz = 0;
  for (int i = 1; i + 1 < w.n; i++) {
    double e1x = (double)w.p[i].x - w.p[0].x, e1y = (double)w.p[i].y - w.p[0].y,
           e1z = (double)w.p[i].z - w.p[0].z;
    double e2x = (double)w.p[i + 1].x - w.p[0].x, e2y = (double)w.p[i + 1].y - w.p[0].y,
           e2z = (double)w.p[i + 1].z - w.p[0].z;
    sx += e1y * e2z - e1z * e2y;
    sy += e1z * e2x - e1x * e2z;
    sz += e1x * e2y - e1y * e2x;
  }
  double sgn = (sx * nrm.x + sy * nrm.y + sz * nrm.z) < 0 ? -1.0 : 1.0;
  double best = 1e30;
  for (int i = 0; i < w.n; i++) {
    const V3 &a = w.p[i], &b = w.p[(i + 1) % w.n];
    double ex = (double)b.x - a.x, ey = (double)b.y - a.y, ez = (double)b.z - a.z;
    double l2 = ex * ex + ey * ey + ez * ez;
    if (l2 < 1e-12) continue;
    double inv = sgn / sqrt(l2);
    double mx = ((double)nrm.y * ez - (double)nrm.z * ey) * inv;
    double my = ((double)nrm.z * ex - (double)nrm.x * ez) * inv;
    double mz = ((double)nrm.x * ey - (double)nrm.y * ex) * inv;
    best = std::min(best, mx * (h.x - a.x) + my * (h.y - a.y) + mz * (h.z - a.z));
  }
  return best;
}

static void PrintW(const char *tag, const Winding &w) {
  printf("      %s (%d):", tag, w.n);
  for (int i = 0; i < w.n; i++) printf(" (%.3f %.3f %.3f)", w.p[i].x, w.p[i].y, w.p[i].z);
  printf("\n");
}

static void Trace(const Map &m, const BitMat &U0, int P, int T, int nrays, double margin, bool verbose) {
  struct Node {
    int portal;
    double t;
    int parent;
  };
  const Portal &pp = m.portals[P];
  std::vector<D3> cen(m.N);
  for (int i = 0; i < m.N; i++) cen[i] = Centroid(m.portals[i].w);
  std::vector<Node> nodes;
  std::vector<int> chain;
  D3 ra{}, rd{};
  bool found = false;
  ForEachBit(U0.row(P), U0.W, [&](int Q) {
    if (found) return;
    const Portal &qq = m.portals[Q];
    for (int s = 0; s < nrays && !found; s++) {
      D3 a, b;
      if (s == 0) {
        a = cen[P];
        b = cen[Q];
      } else {
        u64 h = 0xcbf29ce484222325ull;
        for (u64 v : {(u64)P + 1, (u64)Q + 1, (u64)s + 1}) {
          h ^= v;
          h *= 0x100000001b3ull;
          h ^= h >> 29;
        }
        u64 rng = h;
        SplitMix(rng);
        a = RandomPoint(pp.w, cen[P], rng);
        b = RandomPoint(qq.w, cen[Q], rng);
      }
      D3 d = {b.x - a.x, b.y - a.y, b.z - a.z};
      double len = sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
      if (len < 1e-9) continue;
      d = {d.x / len, d.y / len, d.z / len};
      if (!(d.x * pp.plane.n.x + d.y * pp.plane.n.y + d.z * pp.plane.n.z > 1e-6)) continue;
      if (!(d.x * qq.plane.n.x + d.y * qq.plane.n.y + d.z * qq.plane.n.z > 1e-6)) continue;
      nodes.clear();
      std::vector<std::pair<int, double>> stack; // (node index or -1, t)
      std::vector<int> seen;
      stack.push_back({-1, 0.0});
      while (!stack.empty() && !found) {
        int parent = stack.back().first;
        double t0 = stack.back().second;
        stack.pop_back();
        int cluster = parent < 0 ? pp.leaf : m.portals[nodes[parent].portal].leaf;
        for (int ri : m.cp[cluster]) {
          const Portal &r = m.portals[ri];
          double dn = d.x * r.plane.n.x + d.y * r.plane.n.y + d.z * r.plane.n.z;
          if (!(dn > 1e-6)) continue;
          double t = ((double)r.plane.d - (a.x * r.plane.n.x + a.y * r.plane.n.y + a.z * r.plane.n.z)) / dn;
          if (!(t > t0 + 0.01)) continue;
          if (std::find(seen.begin(), seen.end(), ri) != seen.end()) continue;
          D3 h = {a.x + d.x * t, a.y + d.y * t, a.z + d.z * t};
          if (!InsideMargin(r, h, margin)) continue;
          seen.push_back(ri);
          nodes.push_back({ri, t, parent});
          if (ri == T) {
            for (int n = (int)nodes.size() - 1; n >= 0; n = nodes[n].parent) chain.push_back(n);
            std::reverse(chain.begin(), chain.end());
            ra = a;
            rd = d;
            found = true;
            break;
          }
          stack.push_back({(int)nodes.size() - 1, t});
        }
      }
    }
  });
  if (!found) {
    printf("trace: no ray from portal %d crosses portal %d (rays %d, margin %g)\n", P, T, nrays, margin);
    return;
  }
  printf("trace: base %d (cluster %d -> %d) target %d; ray from (%.3f %.3f %.3f) dir (%.6f %.6f %.6f); %zu portals\n",
         P, pp.owner, pp.leaf, T, ra.x, ra.y, ra.z, rd.x, rd.y, rd.z, chain.size());
  PrintW("base portal", pp.w);
  BitMat dummy;
  dummy.init(1, 64);
  std::vector<int> waveOf(m.N, 0);
  U2Worker wk(m, U0, dummy, waveOf, false, 1);
  wk.P = &pp;
  Winding src, prevPass;
  Wcopy(src, pp.w);
  prevPass.n = 0;
  bool havePass = false;
  int prev = P;
  for (size_t i = 0; i < chain.size(); i++) {
    const Node &nd = nodes[chain[i]];
    const Portal &r = m.portals[nd.portal];
    D3 h = {ra.x + rd.x * nd.t, ra.y + rd.y * nd.t, ra.z + rd.z * nd.t};
    bool inP = GetBit(U0.row(P), nd.portal), inPrev = GetBit(U0.row(prev), nd.portal);
    Winding pass, source, afterStep;
    pass.n = source.n = afterStep.n = 0;
    const char *why = "ok";
    if (!wk.Step(r, src, pass, source))
      why = "DIED in Step (sphere test or plane chop)";
    else {
      Wcopy(afterStep, pass);
      if (havePass) {
        if (!ClipToSeperators(source, prevPass, pass, false))
          why = "DIED in separators(source, prevpass)";
        else if (!ClipToSeperators(prevPass, source, pass, true))
          why = "DIED in separators(prevpass, source)";
      }
    }
    bool ok = why[0] == 'o';
    printf("  %2zu portal %5d (cluster %4d -> %4d) t %10.3f  U0(base) %d U0(prev) %d  %s", i, nd.portal, r.owner,
           r.leaf, nd.t, (int)inP, (int)inPrev, why);
    if (ok)
      printf("  ray inside pass by %.4f, inside source by %.4f, inside portal by %.3f\n",
             InsideDist(pass, r.plane.n, h), InsideDist(source, pp.plane.n, ra), InsideDist(r.w, r.plane.n, h));
    else
      printf("  (ray inside portal by %.3f)\n", InsideDist(r.w, r.plane.n, h));
    if (verbose || !ok) {
      PrintW("portal", r.w);
      if (afterStep.n) PrintW("pass after plane chop", afterStep);
      if (ok) PrintW("pass after separators", pass);
      if (source.n) PrintW("source", source);
      if (havePass) PrintW("prev pass", prevPass);
      printf("      ray hit (%.3f %.3f %.3f)\n", h.x, h.y, h.z);
    }
    if (!ok) return;
    Wcopy(src, source);
    Wcopy(prevPass, pass);
    havePass = true;
    prev = nd.portal;
  }
  printf("trace: the whole chain survives the stock clip steps\n");
}

// ---------------------------------------------------------------------------

struct Opt {
  std::string prt, bsp, dump, patchBsp, outBsp;
  bool flood = false, upper = false, rays = false;
  int K = 1, nrays = 16, threads = 0, sample = 0;
  // Off by default: it pruned nothing on any map measured (the gate only sees
  // portals of the cluster q leads into, where U0(q) and U2(q) agree) and it
  // makes every row depend on earlier rows being sound.
  bool reuse = false;
  // Experimental, off by default: stock's mightsee bitset carried per state (see U2Worker).
  bool might = false;
  double margin = 0.5;
  std::vector<int> only; // --only-bases: just these base portals (debugging)
  int traceP = -1, traceT = -1;
  bool traceVerbose = false;
};

int main(int argc, char **argv) {
  Opt o;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto need = [&]() -> const char * {
      if (i + 1 >= argc) Die("%s needs a value", a.c_str());
      return argv[++i];
    };
    if (a == "--bsp") o.bsp = need();
    else if (a == "--patch-bsp") o.patchBsp = need();
    else if (a == "--out-bsp") o.outBsp = need();
    else if (a == "--flood") o.flood = true;
    else if (a == "--upper") o.upper = true;
    else if (a == "--k") o.K = atoi(need());
    else if (a == "--no-reuse") o.reuse = false;
    else if (a == "--reuse") o.reuse = true;
    else if (a == "--might") o.might = true;
    else if (a == "--rays") {
      o.rays = true;
      o.nrays = atoi(need());
    } else if (a == "--margin") o.margin = atof(need());
    else if (a == "--threads") o.threads = atoi(need());
    else if (a == "--sample-bases") o.sample = atoi(need());
    else if (a == "--eps") g_visEps = (real)atof(need());
    else if (a == "--contain-eps") g_containEps = atof(need());
    else if (a == "--trace") {
      const char *c = need();
      o.traceP = atoi(c);
      while (*c && *c != ',') c++;
      o.traceT = *c ? atoi(c + 1) : -1;
    } else if (a == "--trace-verbose") o.traceVerbose = true;
    else if (a == "--only-bases") {
      for (const char *c = need(); *c;) {
        o.only.push_back(atoi(c));
        while (*c && *c != ',') c++;
        if (*c) c++;
      }
    }
    else if (a == "--dump") o.dump = need();
    else if (a.size() > 2 && a[0] == '-' && a[1] == '-') Die("unknown option %s", a.c_str());
    else if (o.prt.empty()) o.prt = a;
    else Die("unexpected argument %s", a.c_str());
  }
  if (o.prt.empty()) {
    fprintf(stderr,
            "usage: prtvis <map.prt> [--bsp <map.bsp>] [--flood] [--upper] [--k N] [--reuse] [--might]\n"
            "              [--rays N] [--margin F] [--threads N] [--sample-bases N] [--dump <prefix>]\n"
            "              [--eps F] [--contain-eps F] [--only-bases P,..] [--trace P,T] [--trace-verbose]\n"
            "              [--patch-bsp <in.bsp> --out-bsp <out.bsp>]\n"
            "  --patch-bsp/--out-bsp: needs --upper over all base portals; writes the U2 cluster rows (and the\n"
            "  PAS built from them) as the visibility lump of a copy of in.bsp, kept in front of the pakfile when\n"
            "  the input has it last. The input is never modified; out.bsp appears only after its self-check passes.\n");
    return 2;
  }
  // Checked before the no-mode default below, which would switch --upper on.
  bool patching = !o.patchBsp.empty() || !o.outBsp.empty();
  if (patching) {
    if (o.patchBsp.empty() || o.outBsp.empty()) Die("--patch-bsp and --out-bsp must be given together");
    if (!o.upper) Die("--patch-bsp writes U2: it needs --upper");
    if (o.sample > 0 || !o.only.empty())
      Die("--patch-bsp needs U2 for every base portal: it cannot be combined with --sample-bases or --only-bases");
    if (o.traceP >= 0) Die("--patch-bsp cannot be combined with --trace");
    if (NormPath(o.patchBsp) == NormPath(o.outBsp))
      Die("--patch-bsp and --out-bsp name the same file (%s); the input is never modified", o.patchBsp.c_str());
    if (NormPath(o.outBsp + ".tmp") == NormPath(o.patchBsp))
      Die("the temporary %s.tmp of --out-bsp is the --patch-bsp input; pick another output name", o.outBsp.c_str());
    // Same file as the portal file or the exact-PVS BSP: the write would destroy
    // an input the run still has to read. (The --patch-bsp input is compared by
    // content in ReadPatchInput.)
    RefuseSameFile("--out-bsp", o.outBsp, "portal file", o.prt);
    RefuseSameFile("--out-bsp", o.outBsp, "--bsp", o.bsp);
  }
  if (!o.bsp.empty()) CheckInputSize(o.bsp.c_str()); // a missing or over-2-GB BSP fails now, not after the flow
  if (!o.flood && !o.upper && !o.rays) o.flood = o.upper = o.rays = true;
  if (o.K < 0) Die("--k must be >= 0");
  if (o.threads <= 0) o.threads = (int)std::max(1u, std::thread::hardware_concurrency());
  bool sampling = o.sample > 0 || !o.only.empty();
  if (sampling) o.reuse = false;

  Clock::time_point t0 = Clock::now();
  Map m;
  LoadPortals(o.prt.c_str(), m);
  Clock::time_point t1 = Clock::now();
  std::vector<char> bspIn;
  if (patching) {
    if (m.nc <= 0) Die("the portal file has no clusters; nothing to patch");
    bspIn = ReadPatchInput(o.patchBsp, o.outBsp, m.nc);
  }
  printf("clusters %d  directed portals %d  threads %d  real=%s\n", m.nc, m.N, o.threads, REAL_NAME);
  printf("params: k=%d reuse=%d%s rays=%d margin=%g eps=%g sample-bases=%d  bounds:%s%s%s\n", o.K,
         (int)o.reuse, o.might ? " might=1" : "", o.nrays, o.margin, (double)g_visEps, o.sample, "U0",
         o.upper ? " U2" : "", o.rays ? " L" : "");
  printf("phase load: %.3fs\n", Secs(t0, t1));

  // U0
  BitMat U0;
  U0.init(m.N, m.N);
  std::vector<int> nummightsee(m.N, 0);
  {
    std::vector<U0Scratch> sc(o.threads);
    ParallelFor(m.N, o.threads, [&](int b, int tid) {
      ComputeU0(m, b, U0.row(b), sc[tid]);
      nummightsee[b] = (int)CountRow(U0.row(b), U0.W);
    });
  }
  Clock::time_point t2 = Clock::now();
  printf("phase U0 flood: %.3fs\n", Secs(t1, t2));
  if (o.traceP >= 0) {
    if (o.traceP >= m.N || o.traceT < 0 || o.traceT >= m.N) Die("--trace wants P,T portal numbers");
    Trace(m, U0, o.traceP, o.traceT, o.nrays, o.margin, o.traceVerbose);
    return 0;
  }

  std::vector<int> order(m.N);
  for (int i = 0; i < m.N; i++) order[i] = i;
  std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return nummightsee[a] < nummightsee[b]; });
  std::vector<int> bases = order;
  if (!o.only.empty()) {
    for (int b : o.only)
      if (b < 0 || b >= m.N) Die("--only-bases: no portal %d", b);
    bases = o.only;
  } else if (sampling) {
    if (o.sample < m.N) {
      bases.clear();
      for (int k = 0; k < o.sample; k++) bases.push_back(order[(size_t)k * m.N / o.sample]);
    }
  }

  Counters total;
  BitMat U2, L;
  double tU2 = 0, tL = 0;
  if (o.upper) {
    Clock::time_point a = Clock::now();
    U2.init(m.N, m.N);
    std::vector<int> waveOf(m.N, 0);
    for (int k = 0; k < m.N; k++) waveOf[order[k]] = k / WAVE;
    std::vector<U2Worker> workers;
    workers.reserve(o.threads);
    for (int t = 0; t < o.threads; t++) workers.emplace_back(m, U0, U2, waveOf, o.reuse, o.K, o.might);
    if (o.reuse) {
      int nwaves = (m.N + WAVE - 1) / WAVE;
      for (int w = 0; w < nwaves; w++) {
        int lo = w * WAVE, hi = std::min(m.N, lo + WAVE);
        for (auto &wk : workers) wk.curWave = w;
        ParallelFor(hi - lo, o.threads, [&](int i, int tid) { workers[tid].Run(order[lo + i]); });
      }
    } else {
      ParallelFor((int)bases.size(), o.threads, [&](int i, int tid) { workers[tid].Run(bases[i]); });
    }
    for (auto &wk : workers) total.add(wk.cnt);
    tU2 = Secs(a, Clock::now());
    printf("phase U2 bounded flow: %.3fs\n", tU2);
    if (o.might) {
      int maxLive = 0;
      for (auto &wk : workers) maxLive = std::max(maxLive, wk.maxLive);
      size_t words = U0.W;
      printf("might: most live states in one base %d  = %zu bitset bytes per worker thread (%zu words each)\n",
             maxLive, (size_t)maxLive * words * 8, words);
    }
  }
  if (o.rays) {
    Clock::time_point a = Clock::now();
    L.init(m.N, m.N);
    std::vector<D3> cen(m.N);
    for (int i = 0; i < m.N; i++) cen[i] = Centroid(m.portals[i].w);
    std::vector<RayWorker> workers;
    workers.reserve(o.threads);
    for (int t = 0; t < o.threads; t++) workers.emplace_back(m, U0, L, o.nrays, o.margin, cen);
    ParallelFor((int)bases.size(), o.threads, [&](int i, int tid) { workers[tid].Run(bases[i]); });
    for (auto &wk : workers) total.add(wk.cnt);
    tL = Secs(a, Clock::now());
    printf("phase L rays: %.3fs\n", tL);
  }

  // portal level
  long long bitsU0 = 0, bitsU2 = 0, bitsL = 0, u2NotU0 = 0, lNotU0 = 0;
  for (int b : bases) {
    bitsU0 += CountRow(U0.row(b), U0.W);
    if (o.upper) {
      bitsU2 += CountRow(U2.row(b), U2.W);
      for (size_t i = 0; i < U2.W; i++) u2NotU0 += Pop(U2.row(b)[i] & ~U0.row(b)[i]);
    }
    if (o.rays) {
      bitsL += CountRow(L.row(b), L.W);
      for (size_t i = 0; i < L.W; i++) lNotU0 += Pop(L.row(b)[i] & ~U0.row(b)[i]);
    }
  }
  printf("portal level set bits%s: U0 %lld", sampling ? " (sampled bases only)" : "", bitsU0);
  if (o.upper) printf("  U2 %lld  (U2 not in U0: %lld)", bitsU2, u2NotU0);
  if (o.rays) printf("  L %lld  (L not in U0: %lld)", bitsL, lNotU0);
  printf("\n");
  // The soundness check of the upper bound: a ray-proven portal it lacks.
  if (o.upper && o.rays) {
    long long lNotU2 = 0;
    int shown = 0;
    for (int b : bases) {
      for (size_t i = 0; i < L.W; i++) {
        u64 x = L.row(b)[i] & ~U2.row(b)[i];
        lNotU2 += Pop(x);
        while (x && shown < 12) {
          int t = (int)(i * 64 + Ctz(x));
          x &= x - 1;
          printf("  L not in U2: base portal %d (cluster %d -> %d) sees portal %d (cluster %d -> %d)\n", b,
                 m.portals[b].owner, m.portals[b].leaf, t, m.portals[t].owner, m.portals[t].leaf);
          shown++;
        }
      }
    }
    printf("portal level L not in U2: %lld\n", lNotU2);
  }
  printf("counters: states created %llu  merges %llu  dominated arrivals %llu  arrivals %llu  "
         "guard firings %llu  k0 cap merges %llu  rect fallbacks %llu  flow steps %llu  reuse prunes %llu  rays walked %llu  "
         "might-gate skips %llu  nothing-new skips %llu  ray cap hits %llu\n",
         (unsigned long long)total.created, (unsigned long long)total.merges,
         (unsigned long long)total.dominated, (unsigned long long)total.arrivals,
         (unsigned long long)total.guard, (unsigned long long)total.capHits,
         (unsigned long long)total.obb, (unsigned long long)total.flowSteps,
         (unsigned long long)total.reusePrunes, (unsigned long long)total.rays,
         (unsigned long long)total.mightSkips, (unsigned long long)total.nothingNew,
         (unsigned long long)total.rayCapHits);

  if (sampling) {
    printf("sampled %zu of %d bases\n", bases.size(), m.N);
    if (o.upper) printf("U2 seconds per base: %.6f  (x %d bases = %.1fs wall-equivalent at %d threads)\n",
                        tU2 / bases.size(), m.N, tU2 / bases.size() * m.N, o.threads);
    if (o.rays) printf("L  seconds per base: %.6f  (x %d bases = %.1fs wall-equivalent at %d threads)\n",
                       tL / bases.size(), m.N, tL / bases.size() * m.N, o.threads);
    return 0;
  }

  // cluster level
  Clock::time_point c0 = Clock::now();
  BitMat cmU0, cmU2, cmL, xU0, xU2, xL, E;
  long long preU0 = 0, preU2 = 0, preL = 0;
  BuildClusterMatrix(m, U0, cmU0, preU0);
  CrossCheck(cmU0, xU0, m.nc);
  if (o.upper) {
    BuildClusterMatrix(m, U2, cmU2, preU2);
    CrossCheck(cmU2, xU2, m.nc);
  }
  if (o.rays) {
    BuildClusterMatrix(m, L, cmL, preL);
    CrossCheck(cmL, xL, m.nc);
  }
  bool haveE = !o.bsp.empty();
  if (haveE) LoadExact(o.bsp.c_str(), m.nc, E);
  printf("phase cluster level: %.3fs\n", Secs(c0, Clock::now()));

  printf("cluster level, after cross-check (pre-cross-check 'Total clusters visible' in brackets):\n");
  auto report = [&](const char *name, const BitMat &X, long long pre) {
    long long pairs = CountMat(X);
    printf("  %-3s pairs %lld  avg/cluster %.2f  [pre %lld]", name, pairs, (double)pairs / m.nc, pre);
    if (haveE) {
      Cmp c = Compare(X, E);
      printf("  | E %lld  X&E %lld  X-E %lld  E-X %lld", c.e, c.inter, c.xOnly, c.eOnly);
    }
    printf("\n");
  };
  if (o.rays) report("L", xL, preL);
  if (o.upper) report("U2", xU2, preU2);
  report("U0", xU0, preU0);
  if (haveE) {
    printf("  E   pairs %lld  avg/cluster %.2f\n", CountMat(E), (double)CountMat(E) / m.nc);
  }

  // order-independent fingerprint of the results, for thread-count comparisons
  auto fp = [](const BitMat &x) { return (unsigned long long)Fnv(x.d.data(), x.d.size() * 8); };
  printf("checksums (portal level / cluster level): U0 %016llx/%016llx", fp(U0), fp(xU0));
  if (o.upper) printf("  U2 %016llx/%016llx", fp(U2), fp(xU2));
  if (o.rays) printf("  L %016llx/%016llx", fp(L), fp(xL));
  printf("\n");

  if (!o.dump.empty()) {
    if (o.rays) WriteMat(o.dump + "_L.bin", xL, m.nc);
    if (o.upper) WriteMat(o.dump + "_U2.bin", xU2, m.nc);
    WriteMat(o.dump + "_U0.bin", xU0, m.nc);
  }
  if (patching) return PatchBsp(o.outBsp, bspIn, xU2, preU2, m.nc);
  return 0;
}
