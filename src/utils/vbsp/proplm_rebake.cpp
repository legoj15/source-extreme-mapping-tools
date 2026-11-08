//========= proplm: "rebake" route (VBSP side) ================================
//
// For every static prop that asks for it (proplm "rebake", or -proplmrebake)
// this rewrites the prop's model in memory so its one UV set no longer
// overlaps, and embeds the copy in the BSP pak:
//
//   1. Islands are the artist's own UV islands over LOD0 of every bodypart and
//      every model: triangles that share a VVD vertex, plus VVD vertices with
//      bit-identical position, UV and tangent handedness.
//   2. Each island is only translated (never rotated or scaled), so every VVD
//      vertex lands in exactly one island and vertex count, fixups, tangents,
//      VTX, PHY and all MDL mesh data stay byte-identical; only the VVD
//      texcoords change, to oldUV / S + offset. (Exception: a lower LOD with
//      a triangle across two islands is switched off through its VTX switch
//      point and the MDL's numAllowedRootLODs.)
//   3. Island offsets are whole multiples of PROPLM_REBAKE_TEX_SNAP texels of
//      every texture, so each texture is re-rasterized by an exact texel copy
//      into a new VTF at the source's own texel density.
//   4. The MDL gets the new name and one cdtexture pointing at per-model copies
//      of its materials, whose UV0 textures point at the rebaked VTFs.
//
// Design and limits: docs/proplm.md ("rebake route"). The uv2 sibling route is
// proplm_uv2.cpp; the readers both use are in proplm_shared.h.
//=============================================================================
#include "proplm_rebake.h"

#include "KeyValues.h"
#include "bitmap/imageformat.h"
#include "bsplib.h"
#include "gamebspfile.h"
#include "materialpatch.h"
#include "optimize.h"
#include "proplm_shared.h"
#include "studio.h"
#include "tier1/UtlBuffer.h"
#include "tier1/strtools.h"
#include "tier1/utlvector.h"
#include "vbsp.h"
#include "vtf/vtf.h"
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

using namespace OptimizedModel;

bool g_bProplmRebakeAll = false;

// Overlap measurement grid, in cells per UV unit. Triangles are rasterized in
// unwrapped UV (a tiled island is not cut at the texture edge) and the cells
// are folded into one 0..1 tile for the inter-island count, because the
// lightmap wraps.
#define PROPLM_REBAKE_OVERLAP_RES 512
// An island whose own triangles cover more than this share of its cells twice
// (a folded or mirrored-in-place island) cannot be fixed by moving islands:
// the model is left as it is (it needs the "vertex" strategy, issue #54).
#define PROPLM_REBAKE_SELF_OVERLAP 0.01
// An island may span at most this many texture repeats per axis (bounds the
// rasterizer's memory; real props tile a few times at most).
#define PROPLM_REBAKE_MAX_REPEATS 16
// Padding on every side of every island: at least this many texels of every
// output texture (filled with the source's own neighbouring texels) ...
#define PROPLM_REBAKE_TEX_PAD 4
// ... and at least this many lightmap texels at the prop's lightmap size, so
// VRAD's .ppl never blends two islands.
#define PROPLM_REBAKE_LM_PAD 2
// lightmapresolutionx/y used for the padding when a prop has none (the value
// Hammer's FGD fills in).
#define PROPLM_REBAKE_LM_DEFAULT 32
// Island offsets are whole multiples of this many texels of every texture:
// one DXT block, so the copy is exact and block aligned.
#define PROPLM_REBAKE_TEX_SNAP 4
// Largest side of a rebaked VTF; bigger layouts are scaled down by powers of
// two (up to PROPLM_REBAKE_MAX_DOWNSCALE) and the log says so.
#define PROPLM_REBAKE_MAX_TEX 8192
#define PROPLM_REBAKE_MAX_DOWNSCALE 8
// Layout sizes tried, in texture repeats per side: powers of two from
// 2^MIN_POW2 to 2^MAX_POW2.
#define PROPLM_REBAKE_MIN_POW2 (-4)
#define PROPLM_REBAKE_MAX_POW2 5
// Switch point given to lower LODs that cannot follow the new layout (one of
// their triangles spans two islands), so the engine never selects them.
#define PROPLM_REBAKE_LOD_OFF 1.0e7f
// Texels outside every island and its padding take the average of their
// filled neighbours for this many rings (so mips do not pull in a flat
// colour), then the texture's mean colour.
#define PROPLM_REBAKE_DILATE 8

namespace {

// uv' = a * uv + b; the matrix Source builds from "center .. scale .. rotate
// .. translate ..".
struct Affine_t {
  double a[2][2];
  double b[2];
  bool identity;
};

// One source VTF, decoded to BGRA8888.
struct SrcTex_t {
  char name[MAX_PATH]; // material-relative, lowercase, no extension
  IVTFTexture *pTex;
  int width, height;
  ImageFormat format; // the source's format, kept for the output
  int flags;
  float bumpScale;
  Vector reflectivity;
};

// One rebaked VTF: a source VTF sampled through one texture transform.
struct OutTex_t {
  int src;
  Affine_t xf;
  char name[MAX_PATH]; // material-relative, lowercase, no extension
  CUtlVector<int> users; // texture-table indices whose material samples it
};

// One UV0 texture key of a material and the rebaked VTF it now names.
struct TexKey_t {
  KeyValues *pBlock; // the material, or one of its shader fallback blocks
  char key[64];
  int out;
};

// One texture-table entry of the MDL.
struct Material_t {
  char texName[MAX_PATH];  // the MDL's texture name, lowercase
  char material[MAX_PATH]; // the material it resolves to, "" if none exists
  KeyValues *pKV;          // patch-expanded VMT
  CUtlVector<TexKey_t> keys;
  CUtlVector<TexKey_t> dropKeys; // texture transforms baked into the VTFs
  CUtlVector<KeyValues *> blocks; // the material, then its fallback blocks
  bool hasDetail;                 // any block names a $detail texture
};

struct Island_t {
  CUtlVector<int> tris;     // LOD0 triangle indices
  double mn[2], mx[2];      // UV bounds of every vertex of the island, all LODs
  int64 snap[2];            // offset in grid steps
  double t[2];              // offset in UV units (snap * grid)
  CUtlVector<int> textures; // texture-table indices its triangles use
};

enum RebakeState_t { REBAKE_PENDING, REBAKE_DONE, REBAKE_UNCHANGED, REBAKE_FAILED };

// One distinct model asked for by at least one prop.
struct Rebake_t {
  char key[MAX_PATH]; // lowercase, forward slashes
  int lmResX, lmResY; // the smallest lightmap size among its props
  int props;
  RebakeState_t state;
  char reason[320];
  char dictName[MAX_PATH];
  int pakBytes;
  int emitted; // props that went into the static prop lump with the copy
  bool keepsPatched; // refused because the source is not a $staticprop while
                     // -forcedynamicpropsasstatic swapped in a patched copy
};

struct PropReq_t {
  int ent;
  int rebake; // index into s_Rebakes
};

struct VertKey_t {
  float pos[3];
  float uv[2];
  int sign; // tangent handedness: mirrored halves are separate islands
  int raw;
};

// One LOD0 triangle edge between two distinct vertex keys (key ids, a < b).
struct EdgeRef_t {
  int a, b;
  int tri;
};

} // namespace

static CUtlVector<Rebake_t *> s_Rebakes;
static CUtlVector<PropReq_t> s_Props;
static bool s_bWarnedDxt = false;

//-----------------------------------------------------------------------------
// Small helpers
//-----------------------------------------------------------------------------
static int FindSet(CUtlVector<int> &parent, int x) {
  while (parent[x] != x) {
    parent[x] = parent[parent[x]];
    x = parent[x];
  }
  return x;
}

static void UnionSets(CUtlVector<int> &parent, int a, int b) {
  a = FindSet(parent, a);
  b = FindSet(parent, b);
  if (a == b)
    return;
  if (a < b)
    parent[b] = a;
  else
    parent[a] = b;
}

static int CompareVertKeyNoRaw(const VertKey_t *a, const VertKey_t *b) {
  return memcmp(a, b, offsetof(VertKey_t, raw));
}

static int __cdecl CompareVertKey(const void *pa, const void *pb) {
  const VertKey_t *a = (const VertKey_t *)pa;
  const VertKey_t *b = (const VertKey_t *)pb;
  const int c = CompareVertKeyNoRaw(a, b);
  if (c)
    return c;
  return a->raw - b->raw;
}

// Lower-LOD matching keys: 0 = position, UV and handedness; 1 = position
// and UV; 2 = UV only. qsort/bsearch take no context, hence the static.
static int s_KeyLevel = 0;

static int __cdecl CompareVertKeyLevelNoRaw(const void *pa, const void *pb) {
  const VertKey_t *a = (const VertKey_t *)pa;
  const VertKey_t *b = (const VertKey_t *)pb;
  if (s_KeyLevel == 0)
    return CompareVertKeyNoRaw(a, b);
  if (s_KeyLevel == 1)
    return memcmp(a, b, offsetof(VertKey_t, sign));
  return memcmp(a->uv, b->uv, sizeof(a->uv));
}

static int __cdecl CompareVertKeyLevel(const void *pa, const void *pb) {
  const int c = CompareVertKeyLevelNoRaw(pa, pb);
  if (c)
    return c;
  return ((const VertKey_t *)pa)->raw - ((const VertKey_t *)pb)->raw;
}

static int __cdecl CompareEdgeRef(const void *pa, const void *pb) {
  const EdgeRef_t *a = (const EdgeRef_t *)pa;
  const EdgeRef_t *b = (const EdgeRef_t *)pb;
  if (a->a != b->a)
    return a->a < b->a ? -1 : 1;
  if (a->b != b->b)
    return a->b < b->b ? -1 : 1;
  return a->tri - b->tri;
}

static int64 GcdInt(int64 a, int64 b) {
  while (b) {
    const int64 t = a % b;
    a = b;
    b = t;
  }
  return a;
}

// Material-relative texture name: forward slashes, lowercase, no leading
// "materials/" and no ".vtf".
static void NormalizeTexName(const char *pIn, char *pOut, int outLen) {
  char tmp[MAX_PATH];
  V_strncpy(tmp, pIn, sizeof(tmp));
  V_strncpy(pOut, tmp, outLen);
  V_FixSlashes(pOut, '/');
  V_strlower(pOut);
  while (pOut[0] == '/')
    memmove(pOut, pOut + 1, strlen(pOut));
  if (!V_strncmp(pOut, "materials/", 10))
    memmove(pOut, pOut + 10, strlen(pOut + 10) + 1);
  const int len = V_strlen(pOut);
  if (len > 4 && !V_stricmp(pOut + len - 4, ".vtf"))
    pOut[len - 4] = '\0';
}

static bool ReadGameOrPakFile(const char *pPath, CUtlBuffer &buf) {
  if (ReadFileFromPak(GetPakFile(), pPath, false, buf) && buf.TellPut() > 0)
    return true;
  buf.Purge();
  return Proplm_ReadWholeFile(pPath, buf);
}

static bool GameOrPakFileExists(const char *pPath) {
  return FileExistsInPak(GetPakFile(), pPath) ||
         g_pFullFileSystem->FileExists(pPath);
}

static int AddToPak(const char *pName, const void *pData, int len, bool bText) {
  AddBufferToPak(GetPakFile(), pName, (void *)pData, len, bText);
  return len;
}

//-----------------------------------------------------------------------------
// Texture transforms ("center cx cy scale sx sy rotate r translate tx ty")
//-----------------------------------------------------------------------------
static void SetIdentity(Affine_t &xf) {
  xf.a[0][0] = 1.0;
  xf.a[0][1] = 0.0;
  xf.a[1][0] = 0.0;
  xf.a[1][1] = 1.0;
  xf.b[0] = 0.0;
  xf.b[1] = 0.0;
  xf.identity = true;
}

static bool ParseTransform(const char *pValue, Affine_t &xf) {
  SetIdentity(xf);
  if (!pValue || !pValue[0])
    return true;
  double center[2] = {0.5, 0.5}, scale[2] = {1.0, 1.0}, translate[2] = {0.0, 0.0};
  double rotate = 0.0;
  char buf[256];
  V_strncpy(buf, pValue, sizeof(buf));
  char *pCtx = NULL;
  char *pTok = strtok_s(buf, " \t", &pCtx);
  while (pTok) {
    double *pDst = NULL;
    int count = 2;
    if (!V_stricmp(pTok, "center"))
      pDst = center;
    else if (!V_stricmp(pTok, "scale"))
      pDst = scale;
    else if (!V_stricmp(pTok, "translate"))
      pDst = translate;
    else if (!V_stricmp(pTok, "rotate")) {
      pDst = &rotate;
      count = 1;
    } else
      return false;
    for (int i = 0; i < count; ++i) {
      pTok = strtok_s(NULL, " \t", &pCtx);
      if (!pTok)
        return false;
      pDst[i] = atof(pTok);
    }
    pTok = strtok_s(NULL, " \t", &pCtx);
  }
  // Source: translate(-center), scale, rotate, translate(center + translate).
  const double rad = rotate * M_PI / 180.0;
  const double c = cos(rad), s = sin(rad);
  xf.a[0][0] = c * scale[0];
  xf.a[0][1] = -s * scale[1];
  xf.a[1][0] = s * scale[0];
  xf.a[1][1] = c * scale[1];
  xf.b[0] = -(xf.a[0][0] * center[0] + xf.a[0][1] * center[1]) + center[0] +
            translate[0];
  xf.b[1] = -(xf.a[1][0] * center[0] + xf.a[1][1] * center[1]) + center[1] +
            translate[1];
  xf.identity = fabs(xf.a[0][0] - 1.0) < 1e-6 && fabs(xf.a[0][1]) < 1e-6 &&
                fabs(xf.a[1][0]) < 1e-6 && fabs(xf.a[1][1] - 1.0) < 1e-6 &&
                fabs(xf.b[0]) < 1e-6 && fabs(xf.b[1]) < 1e-6;
  return true;
}

static bool SameTransform(const Affine_t &x, const Affine_t &y) {
  if (x.identity || y.identity)
    return x.identity == y.identity;
  for (int i = 0; i < 2; ++i) {
    if (fabs(x.b[i] - y.b[i]) > 1e-9)
      return false;
    for (int j = 0; j < 2; ++j) {
      if (fabs(x.a[i][j] - y.a[i][j]) > 1e-9)
        return false;
    }
  }
  return true;
}

//-----------------------------------------------------------------------------
// UV rasterization: calls fn(x, y) for each cell of a res-per-unit grid whose
// center lies strictly inside the triangle (so two triangles that only share
// an edge never both claim a cell on it).
//-----------------------------------------------------------------------------
template <typename F>
static void RasterTriangle(const float *uv0, const float *uv1, const float *uv2,
                           double res, F &&fn) {
  double p[3][2] = {{uv0[0] * res, uv0[1] * res},
                    {uv1[0] * res, uv1[1] * res},
                    {uv2[0] * res, uv2[1] * res}};
  double area = (p[1][0] - p[0][0]) * (p[2][1] - p[0][1]) -
                (p[1][1] - p[0][1]) * (p[2][0] - p[0][0]);
  if (fabs(area) < 1e-12)
    return;
  if (area < 0.0) {
    for (int k = 0; k < 2; ++k) {
      const double t = p[1][k];
      p[1][k] = p[2][k];
      p[2][k] = t;
    }
  }
  const double minX = MIN(MIN(p[0][0], p[1][0]), p[2][0]);
  const double maxX = MAX(MAX(p[0][0], p[1][0]), p[2][0]);
  const double minY = MIN(MIN(p[0][1], p[1][1]), p[2][1]);
  const double maxY = MAX(MAX(p[0][1], p[1][1]), p[2][1]);
  const int x0 = (int)floor(minX - 0.5), x1 = (int)ceil(maxX - 0.5);
  const int y0 = (int)floor(minY - 0.5), y1 = (int)ceil(maxY - 0.5);
  for (int y = y0; y <= y1; ++y) {
    const double cy = y + 0.5;
    for (int x = x0; x <= x1; ++x) {
      const double cx = x + 0.5;
      bool inside = true;
      for (int e = 0; e < 3 && inside; ++e) {
        const double *a = p[e], *b = p[(e + 1) % 3];
        inside = (b[0] - a[0]) * (cy - a[1]) - (b[1] - a[1]) * (cx - a[0]) > 0.0;
      }
      if (inside)
        fn(x, y);
    }
  }
}

// Rasterizes the LOD0 triangles of one island in unwrapped UV. Appends each
// distinct cell to cells (x, y pairs) and returns the number of cells claimed
// a second time by the island's own triangles. False when the island spans
// more than PROPLM_REBAKE_MAX_REPEATS repeats.
static bool RasterIsland(const Island_t &isl,
                         const CUtlVector<ProplmVtxTri_t> &tris,
                         const CUtlVector<Vector2D> &uv, double res,
                         CUtlVector<int> &cells, int *pDup) {
  *pDup = 0;
  double mn[2] = {1e30, 1e30}, mx[2] = {-1e30, -1e30};
  for (int i = 0; i < isl.tris.Count(); ++i) {
    for (int c = 0; c < 3; ++c) {
      const Vector2D &t = uv[tris[isl.tris[i]].vert[c]];
      for (int k = 0; k < 2; ++k) {
        mn[k] = MIN(mn[k], (double)t[k]);
        mx[k] = MAX(mx[k], (double)t[k]);
      }
    }
  }
  if (mx[0] - mn[0] > PROPLM_REBAKE_MAX_REPEATS ||
      mx[1] - mn[1] > PROPLM_REBAKE_MAX_REPEATS)
    return false;
  const int x0 = (int)floor(mn[0] * res) - 1, y0 = (int)floor(mn[1] * res) - 1;
  const int w = (int)ceil(mx[0] * res) + 2 - x0, h = (int)ceil(mx[1] * res) + 2 - y0;
  CUtlVector<unsigned char> seen;
  seen.SetCount(w * h);
  memset(seen.Base(), 0, w * h);
  int dup = 0;
  for (int i = 0; i < isl.tris.Count(); ++i) {
    const ProplmVtxTri_t &tri = tris[isl.tris[i]];
    RasterTriangle(uv[tri.vert[0]].Base(), uv[tri.vert[1]].Base(),
                   uv[tri.vert[2]].Base(), res, [&](int x, int y) {
                     const int lx = x - x0, ly = y - y0;
                     if (lx < 0 || ly < 0 || lx >= w || ly >= h)
                       return;
                     unsigned char &s = seen[ly * w + lx];
                     if (s) {
                       ++dup;
                     } else {
                       s = 1;
                       cells.AddToTail(x);
                       cells.AddToTail(y);
                     }
                   });
  }
  *pDup = dup;
  return true;
}

// Share of the islands' cells, folded into one 0..1 tile, that another island
// (or another repeat of the same island) already covers. pSelfWorst gets the
// largest self-overlap share of any island and its index.
static bool MeasureOverlap(const CUtlVector<Island_t> &islands,
                           const CUtlVector<ProplmVtxTri_t> &tris,
                           const CUtlVector<Vector2D> &uv, int *pOverlap,
                           int *pTotal, double *pSelfWorst, int *pSelfIsland,
                           char *pErr, int errLen) {
  const int res = PROPLM_REBAKE_OVERLAP_RES;
  CUtlVector<int> owner;
  owner.SetCount(res * res);
  for (int i = 0; i < owner.Count(); ++i)
    owner[i] = -1;
  *pOverlap = 0;
  *pTotal = 0;
  *pSelfWorst = 0.0;
  *pSelfIsland = -1;
  CUtlVector<int> cells;
  for (int i = 0; i < islands.Count(); ++i) {
    cells.RemoveAll();
    int dup = 0;
    if (!RasterIsland(islands[i], tris, uv, res, cells, &dup)) {
      V_snprintf(pErr, errLen,
                 "UV island %d spans more than %d texture repeats", i,
                 PROPLM_REBAKE_MAX_REPEATS);
      return false;
    }
    const int count = cells.Count() / 2;
    if (count > 0 && dup > 2) {
      const double self = (double)dup / (double)count;
      if (self > *pSelfWorst) {
        *pSelfWorst = self;
        *pSelfIsland = i;
      }
    }
    for (int c = 0; c < count; ++c) {
      const int fx = ((cells[c * 2] % res) + res) % res;
      const int fy = ((cells[c * 2 + 1] % res) + res) % res;
      int &o = owner[fy * res + fx];
      if (o >= 0)
        ++*pOverlap;
      else
        o = i;
      ++*pTotal;
    }
  }
  return true;
}

//-----------------------------------------------------------------------------
// Packing: a deterministic shelf packer over the islands' padded bounds.
// Offsets are snapped to whole grid steps (grid[k] UV units). Returns false
// when the islands do not fit size[0] x size[1].
//-----------------------------------------------------------------------------
static bool ShelfPack(CUtlVector<Island_t> &islands, const CUtlVector<int> &order,
                      const double size[2], const double pad[2],
                      const double grid[2]) {
  const double eps = 1e-9;
  double cursorX = 0.0, shelfY = 0.0, shelfTop = 0.0;
  bool shelfEmpty = true;
  for (int n = 0; n < order.Count(); ++n) {
    Island_t &isl = islands[order[n]];
    for (int attempt = 0;; ++attempt) {
      const int64 su = (int64)ceil((cursorX + pad[0] - isl.mn[0]) / grid[0] - eps);
      const int64 sv = (int64)ceil((shelfY + pad[1] - isl.mn[1]) / grid[1] - eps);
      const double tu = (double)su * grid[0], tv = (double)sv * grid[1];
      const double right = isl.mx[0] + tu + pad[0];
      const double top = isl.mx[1] + tv + pad[1];
      if (right <= size[0] + eps) {
        if (top > size[1] + eps)
          return false;
        isl.snap[0] = su;
        isl.snap[1] = sv;
        isl.t[0] = tu;
        isl.t[1] = tv;
        cursorX = right;
        shelfTop = MAX(shelfTop, top);
        shelfEmpty = false;
        break;
      }
      if (shelfEmpty || attempt > 0)
        return false; // wider than the whole layout
      shelfY = shelfTop;
      cursorX = 0.0;
      shelfEmpty = true;
    }
  }
  return true;
}

//-----------------------------------------------------------------------------
// Texture re-rasterization
//-----------------------------------------------------------------------------
static inline const unsigned char *Texel(const SrcTex_t &s, int x, int y) {
  x %= s.width;
  if (x < 0)
    x += s.width;
  y %= s.height;
  if (y < 0)
    y += s.height;
  return s.pTex->ImageData(0, 0, 0) + ((size_t)y * s.width + x) * 4;
}

// Bilinear sample with wrap addressing at uv (0..1 per repeat).
static void SampleBilinear(const SrcTex_t &s, double u, double v,
                           unsigned char out[4]) {
  const double fx = u * s.width - 0.5, fy = v * s.height - 0.5;
  const int x0 = (int)floor(fx), y0 = (int)floor(fy);
  const double ax = fx - x0, ay = fy - y0;
  const unsigned char *p00 = Texel(s, x0, y0), *p10 = Texel(s, x0 + 1, y0);
  const unsigned char *p01 = Texel(s, x0, y0 + 1), *p11 = Texel(s, x0 + 1, y0 + 1);
  for (int c = 0; c < 4; ++c) {
    const double top = p00[c] + (p10[c] - p00[c]) * ax;
    const double bot = p01[c] + (p11[c] - p01[c]) * ax;
    out[c] = (unsigned char)(top + (bot - top) * ay + 0.5);
  }
}

// Fills the output texels (BGRA8888, outW x outH, mask set where written) of
// every island in usedIslands from the source. Identity transforms copy
// texels exactly (box filtered when downscaled); others resample bilinearly.
static void FillIslands(const OutTex_t &out, const SrcTex_t &src,
                        const CUtlVector<Island_t> &islands,
                        const CUtlVector<int> &usedIslands, const double size[2],
                        const double pad[2], int down, const int64 snapTexels[2],
                        int outW, int outH, unsigned char *pDst,
                        unsigned char *pMask) {
  for (int n = 0; n < usedIslands.Count(); ++n) {
    const Island_t &isl = islands[usedIslands[n]];
    const double lo[2] = {isl.mn[0] + isl.t[0] - pad[0], isl.mn[1] + isl.t[1] - pad[1]};
    const double hi[2] = {isl.mx[0] + isl.t[0] + pad[0], isl.mx[1] + isl.t[1] + pad[1]};
    // Texels whose centers lie in [lo, hi).
    const int ix0 = MAX(0, (int)ceil(lo[0] * outW / size[0] - 0.5));
    const int ix1 = MIN(outW, (int)ceil(hi[0] * outW / size[0] - 0.5));
    const int iy0 = MAX(0, (int)ceil(lo[1] * outH / size[1] - 0.5));
    const int iy1 = MIN(outH, (int)ceil(hi[1] * outH / size[1] - 0.5));
    // Output texel offset of this island: t * texels-per-unit / down, whole by
    // construction of the grid.
    const int64 offX = isl.snap[0] * snapTexels[0];
    const int64 offY = isl.snap[1] * snapTexels[1];
    for (int y = iy0; y < iy1; ++y) {
      for (int x = ix0; x < ix1; ++x) {
        unsigned char *pOut = pDst + ((size_t)y * outW + x) * 4;
        if (out.xf.identity) {
          const int sx = (int)((x - offX) * down), sy = (int)((y - offY) * down);
          if (down == 1) {
            memcpy(pOut, Texel(src, sx, sy), 4);
          } else {
            int sum[4] = {0, 0, 0, 0};
            for (int dy = 0; dy < down; ++dy) {
              for (int dx = 0; dx < down; ++dx) {
                const unsigned char *p = Texel(src, sx + dx, sy + dy);
                for (int c = 0; c < 4; ++c)
                  sum[c] += p[c];
              }
            }
            for (int c = 0; c < 4; ++c)
              pOut[c] = (unsigned char)((sum[c] + down * down / 2) / (down * down));
          }
        } else {
          // Texel center in layout units, back to the island's source UV,
          // then through the material's texture transform.
          const double cu = (x + 0.5) * size[0] / outW - isl.t[0];
          const double cv = (y + 0.5) * size[1] / outH - isl.t[1];
          const double su = out.xf.a[0][0] * cu + out.xf.a[0][1] * cv + out.xf.b[0];
          const double sv = out.xf.a[1][0] * cu + out.xf.a[1][1] * cv + out.xf.b[1];
          SampleBilinear(src, su, sv, pOut);
        }
        pMask[(size_t)y * outW + x] = 1;
      }
    }
  }
}

// Grows the filled area by PROPLM_REBAKE_DILATE rings (each new texel is the
// average of its filled 8-neighbours), then fills the rest with the mean.
static void Dilate(int w, int h, unsigned char *pData, unsigned char *pMask) {
  CUtlVector<int> ring;
  for (int pass = 0; pass < PROPLM_REBAKE_DILATE; ++pass) {
    ring.RemoveAll();
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        if (pMask[(size_t)y * w + x])
          continue;
        int sum[4] = {0, 0, 0, 0}, n = 0;
        for (int dy = -1; dy <= 1; ++dy) {
          const int yy = y + dy;
          if (yy < 0 || yy >= h)
            continue;
          for (int dx = -1; dx <= 1; ++dx) {
            const int xx = x + dx;
            if (xx < 0 || xx >= w || pMask[(size_t)yy * w + xx] != 1)
              continue;
            const unsigned char *p = pData + ((size_t)yy * w + xx) * 4;
            for (int c = 0; c < 4; ++c)
              sum[c] += p[c];
            ++n;
          }
        }
        if (!n)
          continue;
        unsigned char *p = pData + ((size_t)y * w + x) * 4;
        for (int c = 0; c < 4; ++c)
          p[c] = (unsigned char)((sum[c] + n / 2) / n);
        pMask[(size_t)y * w + x] = 2;
        ring.AddToTail(y * w + x);
      }
    }
    if (!ring.Count())
      break;
    for (int i = 0; i < ring.Count(); ++i)
      pMask[ring[i]] = 1;
  }
  double sum[4] = {0, 0, 0, 0};
  int64 n = 0;
  for (size_t i = 0; i < (size_t)w * h; ++i) {
    if (pMask[i]) {
      for (int c = 0; c < 4; ++c)
        sum[c] += pData[i * 4 + c];
      ++n;
    }
  }
  unsigned char mean[4] = {128, 128, 128, 255};
  if (n) {
    for (int c = 0; c < 4; ++c)
      mean[c] = (unsigned char)(sum[c] / n + 0.5);
  }
  for (size_t i = 0; i < (size_t)w * h; ++i) {
    if (!pMask[i])
      memcpy(pData + i * 4, mean, 4);
  }
}

static bool IsSupportedFormat(ImageFormat fmt) {
  switch (fmt) {
  case IMAGE_FORMAT_DXT1:
  case IMAGE_FORMAT_DXT1_ONEBITALPHA:
  case IMAGE_FORMAT_DXT3:
  case IMAGE_FORMAT_DXT5:
  case IMAGE_FORMAT_BGR888:
  case IMAGE_FORMAT_RGB888:
  case IMAGE_FORMAT_BGRA8888:
  case IMAGE_FORMAT_RGBA8888:
  case IMAGE_FORMAT_ABGR8888:
  case IMAGE_FORMAT_ARGB8888:
  case IMAGE_FORMAT_BGRX8888:
  case IMAGE_FORMAT_I8:
  case IMAGE_FORMAT_IA88:
  case IMAGE_FORMAT_A8:
  case IMAGE_FORMAT_UV88:
  case IMAGE_FORMAT_BGR565:
  case IMAGE_FORMAT_BGRA4444:
  case IMAGE_FORMAT_BGRA5551:
  case IMAGE_FORMAT_BGRX5551:
    return true;
  default:
    return false;
  }
}

// Builds the rebaked VTF from the BGRA8888 image: full mip chain unless the
// source had none, source flags, bump scale and reflectivity, encoded back to
// the source's format (DXT stays DXT). Serialized into buf.
static bool EncodeVtf(const SrcTex_t &src, int w, int h, const unsigned char *pData,
                      CUtlBuffer &buf, ImageFormat *pWritten, char *pErr,
                      int errLen) {
  IVTFTexture *pOut = CreateVTFTexture();
  const int mips = (src.flags & TEXTUREFLAGS_NOMIP) ? 1 : -1;
  if (!pOut->Init(w, h, 1, IMAGE_FORMAT_BGRA8888, src.flags, 1, mips)) {
    DestroyVTFTexture(pOut);
    V_snprintf(pErr, errLen, "cannot create a %dx%d VTF", w, h);
    return false;
  }
  memcpy(pOut->ImageData(0, 0, 0), pData, (size_t)w * h * 4);
  if (pOut->MipCount() > 1)
    pOut->GenerateMipmaps();
  pOut->SetBumpScale(src.bumpScale);
  pOut->SetReflectivity(src.reflectivity);

  ImageFormat target = src.format;
  pOut->ConvertImageFormat(target, false);
  if (pOut->Format() != target) {
    // This build's image library could not encode the source format (DXT
    // needs the S3TC encoder): write uncompressed instead and say so once.
    const bool alpha = ImageLoader::ImageFormatInfo(target).m_NumAlphaBits > 0;
    const ImageFormat fallback = alpha ? IMAGE_FORMAT_BGRA8888 : IMAGE_FORMAT_BGR888;
    if (!s_bWarnedDxt) {
      s_bWarnedDxt = true;
      Warning("Warning: proplm rebake: cannot encode %s; rebaked textures in "
              "that format are written as %s.\n",
              ImageLoader::GetName(target), ImageLoader::GetName(fallback));
    }
    if (pOut->Format() != fallback)
      pOut->ConvertImageFormat(fallback, false);
    target = fallback;
  }
  *pWritten = pOut->Format();
  const bool ok = pOut->Serialize(buf);
  DestroyVTFTexture(pOut);
  if (!ok) {
    V_snprintf(pErr, errLen, "cannot serialize a %dx%d VTF", w, h);
    return false;
  }
  return true;
}

//-----------------------------------------------------------------------------
// MDL rewrite: new internal name (name[64] and the studiohdr2 full name), one
// cdtexture. Appends the new strings at the end of a copy of the file and
// updates length; nothing else moves, so every other offset stays valid.
//-----------------------------------------------------------------------------
static void PutAligned(CUtlBuffer &buf, const void *pData, int len) {
  buf.Put(pData, len);
  while (buf.TellPut() & 3)
    buf.PutChar(0);
}

// Sets the switch point of LOD firstLod and every later LOD of every model in
// a VTX to PROPLM_REBAKE_LOD_OFF (the shadow LOD, switch point < 0, is left
// alone). Only floats change; the layout and checksum stay. Models are bounded
// by the MDL's bodyparts and models, LODs by the VTX header's numLODs, and
// every offset is checked against the buffer.
static bool DisableLods(const studiohdr_t *pMdl, CUtlBuffer &vtx, int firstLod,
                        char *pErr, int errLen) {
  const int64 size = vtx.TellPut();
  if (size < (int64)sizeof(FileHeader_t)) {
    V_snprintf(pErr, errLen, "VTX is truncated");
    return false;
  }
  byte *pBase = (byte *)vtx.Base();
  const FileHeader_t *pHdr = (const FileHeader_t *)pBase;
  const int64 bpBase = pHdr->bodyPartOffset;
  if (!Proplm_InFile(bpBase, pHdr->numBodyParts, sizeof(BodyPartHeader_t), size)) {
    V_snprintf(pErr, errLen, "VTX bodypart table is out of range");
    return false;
  }
  for (int bp = 0; bp < pHdr->numBodyParts && bp < pMdl->numbodyparts; ++bp) {
    const int64 bpOfs = bpBase + (int64)bp * sizeof(BodyPartHeader_t);
    const BodyPartHeader_t *pBody = (const BodyPartHeader_t *)(pBase + bpOfs);
    const int64 modelBase = bpOfs + pBody->modelOffset;
    if (!Proplm_InFile(modelBase, pBody->numModels, sizeof(ModelHeader_t), size)) {
      V_snprintf(pErr, errLen, "VTX model table is out of range");
      return false;
    }
    const int models = MIN(pBody->numModels, pMdl->pBodypart(bp)->nummodels);
    for (int m = 0; m < models; ++m) {
      const int64 mOfs = modelBase + (int64)m * sizeof(ModelHeader_t);
      const ModelHeader_t *pModel = (const ModelHeader_t *)(pBase + mOfs);
      const int64 lodBase = mOfs + pModel->lodOffset;
      if (!Proplm_InFile(lodBase, pModel->numLODs, sizeof(ModelLODHeader_t), size)) {
        V_snprintf(pErr, errLen, "VTX LOD table is out of range");
        return false;
      }
      const int lods = MIN(pModel->numLODs, pHdr->numLODs);
      for (int lod = MAX(firstLod, 0); lod < lods; ++lod) {
        ModelLODHeader_t *pLod =
            (ModelLODHeader_t *)(pBase + lodBase + (int64)lod * sizeof(ModelLODHeader_t));
        if (pLod->switchPoint >= 0.0f)
          pLod->switchPoint = PROPLM_REBAKE_LOD_OFF;
      }
    }
  }
  return true;
}

// Number of material replacements the VTX declares for LOD lod (a LOD drawn
// with another material than the MDL's), or -1 when the list is out of range.
static int VtxLodReplacements(const CUtlBuffer &vtx, int lod) {
  const int64 size = vtx.TellPut();
  if (size < (int64)sizeof(FileHeader_t))
    return -1;
  const FileHeader_t *pHdr = (const FileHeader_t *)vtx.Base();
  if (lod < 0 || lod >= pHdr->numLODs || !pHdr->materialReplacementListOffset)
    return 0;
  if (!Proplm_InFile(pHdr->materialReplacementListOffset, pHdr->numLODs,
                     sizeof(MaterialReplacementListHeader_t), size))
    return -1;
  const MaterialReplacementListHeader_t *pList =
      (const MaterialReplacementListHeader_t *)((const byte *)vtx.Base() +
                                                pHdr->materialReplacementListOffset) +
      lod;
  return pList->numReplacements < 0 ? -1 : pList->numReplacements;
}

static bool RewriteMdl(const CUtlBuffer &src, const char *pInternalName,
                       const char *pCdTexture, int allowedRootLods,
                       CUtlBuffer &out, char *pErr, int errLen) {
  out.Purge();
  out.Put(src.Base(), src.TellPut());
  while (out.TellPut() & 3)
    out.PutChar(0);

  const int cdStr = out.TellPut();
  PutAligned(out, pCdTexture, V_strlen(pCdTexture) + 1);
  const int cdArr = out.TellPut();
  PutAligned(out, &cdStr, sizeof(cdStr));

  int hdr2 = ((const studiohdr_t *)src.Base())->studiohdr2index;
  if (hdr2 && (hdr2 < 0 || (hdr2 & 3) ||
               !Proplm_InFile(hdr2, 1, sizeof(studiohdr2_t), src.TellPut()))) {
    V_snprintf(pErr, errLen, "the MDL's studiohdr2 is out of range");
    return false;
  }
  const int nameLen = V_strlen(pInternalName);
  if (!hdr2 && nameLen >= (int)sizeof(((studiohdr_t *)0)->name)) {
    // No studiohdr2 to hold a long name: add an empty one.
    hdr2 = out.TellPut();
    unsigned char zeros[sizeof(studiohdr2_t)];
    memset(zeros, 0, sizeof(zeros));
    PutAligned(out, zeros, sizeof(zeros));
  }
  int nameStr = 0;
  if (hdr2) {
    nameStr = out.TellPut();
    PutAligned(out, pInternalName, nameLen + 1);
  }

  if (!out.IsValid()) {
    V_snprintf(pErr, errLen, "out of memory rewriting the MDL");
    return false;
  }
  byte *pBase = (byte *)out.Base();
  studiohdr_t *pHdr = (studiohdr_t *)pBase;
  memset(pHdr->name, 0, sizeof(pHdr->name));
  V_strncpy(pHdr->name, pInternalName, sizeof(pHdr->name));
  pHdr->numcdtextures = 1;
  pHdr->cdtextureindex = cdArr;
  if (allowedRootLods > 0 &&
      (!pHdr->numAllowedRootLODs || pHdr->numAllowedRootLODs > allowedRootLods))
    pHdr->numAllowedRootLODs = (byte)allowedRootLods;
  if (hdr2) {
    pHdr->studiohdr2index = hdr2;
    *(int *)(pBase + hdr2 + offsetof(studiohdr2_t, sznameindex)) = nameStr - hdr2;
  }
  pHdr->length = out.TellPut();
  return true;
}

//-----------------------------------------------------------------------------
// The rebake of one model. Returns REBAKE_DONE (files written to the pak),
// REBAKE_UNCHANGED (islands do not overlap) or REBAKE_FAILED, with the reason.
//-----------------------------------------------------------------------------
namespace {
// Everything one rebake allocates outside CUtl containers, freed on return.
struct RebakeScratch_t {
  CUtlVector<SrcTex_t> srcs;
  CUtlVector<Material_t> mats;
  ~RebakeScratch_t() {
    for (int i = 0; i < srcs.Count(); ++i) {
      if (srcs[i].pTex)
        DestroyVTFTexture(srcs[i].pTex);
    }
    for (int i = 0; i < mats.Count(); ++i) {
      if (mats[i].pKV)
        mats[i].pKV->deleteThis();
    }
  }
};
} // namespace

// $detailscale (one number or "[x y]") times the layout size.
static void ScaleDetail(const char *pScale, const double size[2], char *pOut,
                        int outLen) {
  float sx = 4.0f, sy = 4.0f;
  if (pScale[0] == '[' && sscanf(pScale, "[ %f %f ]", &sx, &sy) == 2) {
    V_snprintf(pOut, outLen, "[%g %g]", sx * size[0], sy * size[1]);
  } else {
    sx = (float)atof(pScale);
    V_snprintf(pOut, outLen, "%g", sx * size[0]);
  }
}

static const char *s_BaseUvKeys[] = {"$basetexture", "$envmapmask",
                                     "$phongexponenttexture", "$selfillummask",
                                     "$ambientoccltexture"};
static const char *s_BumpUvKeys[] = {"$bumpmap", "$normalmap"};
static const char *s_UvProxies[] = {"TextureScroll", "TextureTransform",
                                    "AnimatedTexture", "AnimatedOffsetTexture",
                                    "TextureToggle"};

static RebakeState_t BuildRebake(Rebake_t *pR, char *pErr, int errLen) {
  RebakeScratch_t scratch;
  const char *pKey = pR->key;

  // -- names -----------------------------------------------------------------
  const int keyLen = V_strlen(pKey);
  if (V_strncmp(pKey, "models/", 7) || keyLen < 12 ||
      V_stricmp(pKey + keyLen - 4, ".mdl")) {
    V_snprintf(pErr, errLen, "the model path is not models/....mdl");
    return REBAKE_FAILED;
  }
  char rel[MAX_PATH]; // e.g. props_c17/furniturefridge001a
  V_strncpy(rel, pKey + 7, sizeof(rel));
  rel[V_strlen(rel) - 4] = '\0';
  char mapLower[64];
  V_strncpy(mapLower, mapbase, sizeof(mapLower));
  V_strlower(mapLower);
  char internalName[MAX_PATH], rebakeDir[MAX_PATH], cdTexture[MAX_PATH];
  V_snprintf(internalName, sizeof(internalName), "maps/%s/rebake/%s.mdl", mapLower, rel);
  V_snprintf(pR->dictName, sizeof(pR->dictName), "models/%s", internalName);
  V_snprintf(rebakeDir, sizeof(rebakeDir), "maps/%s/rebake/%s", mapLower, rel);
  V_snprintf(cdTexture, sizeof(cdTexture), "%s/", rebakeDir);
  if (V_strlen(pR->dictName) >= STATIC_PROP_NAME_LENGTH) {
    V_snprintf(pErr, errLen,
               "the rebaked model name \"%s\" is longer than the static prop "
               "lump's %d characters",
               pR->dictName, STATIC_PROP_NAME_LENGTH - 1);
    return REBAKE_FAILED;
  }

  // -- files -----------------------------------------------------------------
  char path[MAX_PATH];
  CUtlBuffer mdlFile, vvdFile, vtx90, vtx80, phyFile;
  if (!ReadGameOrPakFile(pKey, mdlFile)) {
    V_snprintf(pErr, errLen, "cannot read the MDL");
    return REBAKE_FAILED;
  }
  if (mdlFile.TellPut() < (int)sizeof(studiohdr_t) ||
      strncmp((const char *)mdlFile.Base(), "IDST", 4)) {
    V_snprintf(pErr, errLen, "not a studio model");
    return REBAKE_FAILED;
  }
  V_strncpy(path, pKey, sizeof(path));
  V_SetExtension(path, ".vvd", sizeof(path));
  if (!ReadGameOrPakFile(path, vvdFile)) {
    V_snprintf(pErr, errLen, "cannot read %s", path);
    return REBAKE_FAILED;
  }
  V_strncpy(path, pKey, sizeof(path));
  V_SetExtension(path, ".dx90.vtx", sizeof(path));
  const bool has90 = ReadGameOrPakFile(path, vtx90);
  V_strncpy(path, pKey, sizeof(path));
  V_SetExtension(path, ".dx80.vtx", sizeof(path));
  const bool has80 = ReadGameOrPakFile(path, vtx80);
  if (!has90 && !has80) {
    V_snprintf(pErr, errLen, "no .dx90.vtx or .dx80.vtx");
    return REBAKE_FAILED;
  }
  V_strncpy(path, pKey, sizeof(path));
  V_SetExtension(path, ".phy", sizeof(path));
  const bool hasPhy = ReadGameOrPakFile(path, phyFile);
  const CUtlBuffer &vtxWalk = has90 ? vtx90 : vtx80;

  // A working copy of the MDL for the walk; mdlFile stays byte-exact.
  CUtlBuffer mdlWork;
  mdlWork.Put(mdlFile.Base(), mdlFile.TellPut());
  studiohdr_t *pHdr = (studiohdr_t *)mdlWork.Base();
  if (!Proplm_ConvertMdl(pHdr, mdlWork.TellPut(), pErr, errLen))
    return REBAKE_FAILED;
  if (pHdr->length != mdlFile.TellPut()) {
    V_snprintf(pErr, errLen, "the MDL's length field (%d) is not its size (%d)",
               pHdr->length, mdlFile.TellPut());
    return REBAKE_FAILED;
  }
  if (!Proplm_ValidateMdl(pHdr, mdlWork.TellPut(), pErr, errLen))
    return REBAKE_FAILED;
  // Not a $staticprop model: VRAD would refuse a copy of it. Under
  // -forcedynamicpropsasstatic VBSP has already put a patched copy in the pak:
  // in fallback mode the flag-patched MDL under the original name, which this
  // read found (so it is rebaked); in decompile mode a recompiled *_static
  // model under another name, which the prop then keeps.
  if (!(pHdr->flags & STUDIOHDR_FLAGS_STATIC_PROP)) {
    pR->keepsPatched = g_bForceDynamicPropsAsStatic != 0;
    V_snprintf(pErr, errLen, "the model is not compiled as a $staticprop");
    return REBAKE_FAILED;
  }
  if (pHdr->numincludemodels > 0) {
    V_snprintf(pErr, errLen, "the model has $includemodel models");
    return REBAKE_FAILED;
  }
  if (pHdr->numanimblocks > 0) {
    V_snprintf(pErr, errLen, "the model has external animation blocks");
    return REBAKE_FAILED;
  }

  CUtlVector<ProplmVvdVert_t> verts;
  if (!Proplm_LoadVvdVertices(vvdFile, pHdr->checksum, verts, pErr, errLen))
    return REBAKE_FAILED;
  const vertexFileHeader_t *pVvd = (const vertexFileHeader_t *)vvdFile.Base();
  const int rawCount = pVvd->numLODVertexes[0];

  // Per raw VVD slot: position, UV, tangent handedness.
  CUtlVector<Vector2D> uv;
  CUtlVector<VertKey_t> keyOf;
  uv.SetCount(rawCount);
  keyOf.SetCount(rawCount);
  memset(keyOf.Base(), 0, rawCount * sizeof(VertKey_t));
  const bool hasTangents =
      pVvd->tangentDataStart > 0 &&
      (int64)pVvd->tangentDataStart + (int64)rawCount * (int64)sizeof(Vector4D) <=
          vvdFile.TellPut();
  for (int i = 0; i < verts.Count(); ++i) {
    const ProplmVvdVert_t &v = verts[i];
    VertKey_t &k = keyOf[v.raw];
    memcpy(k.pos, v.pos, sizeof(k.pos));
    memcpy(k.uv, v.uv, sizeof(k.uv));
    k.sign = 1;
    if (hasTangents) {
      const Vector4D *pTan =
          (const Vector4D *)((const byte *)vvdFile.Base() + pVvd->tangentDataStart);
      k.sign = pTan[v.raw].w < 0.0f ? -1 : 1;
    }
    k.raw = v.raw;
    uv[v.raw].Init(v.uv[0], v.uv[1]);
  }

  // -- LOD0 triangles of every bodypart and model, in raw slots --------------
  CUtlVector<ProplmVtxMesh_t> meshes;
  CUtlVector<ProplmVtxTri_t> tris;
  // Every strip must be a triangle list: a strip the walk skipped would keep
  // its old UVs in the new layout.
  if (!Proplm_GatherVtxTriangles(pHdr, vtxWalk, verts.Count(), 0, true, true,
                                 &meshes, tris, pErr, errLen))
    return REBAKE_FAILED;
  if (!tris.Count()) {
    V_snprintf(pErr, errLen, "no LOD0 triangles");
    return REBAKE_FAILED;
  }
  for (int t = 0; t < tris.Count(); ++t) {
    for (int c = 0; c < 3; ++c)
      tris[t].vert[c] = verts[tris[t].vert[c]].raw;
  }

  // -- islands ---------------------------------------------------------------
  CUtlVector<int> parent;
  parent.SetCount(rawCount);
  for (int i = 0; i < rawCount; ++i)
    parent[i] = i;
  CUtlVector<unsigned char> usedLod0;
  usedLod0.SetCount(rawCount);
  memset(usedLod0.Base(), 0, rawCount);
  for (int t = 0; t < tris.Count(); ++t) {
    UnionSets(parent, tris[t].vert[0], tris[t].vert[1]);
    UnionSets(parent, tris[t].vert[0], tris[t].vert[2]);
    for (int c = 0; c < 3; ++c)
      usedLod0[tris[t].vert[c]] = 1;
  }
  CUtlVector<VertKey_t> lod0Keys;
  for (int r = 0; r < rawCount; ++r) {
    if (usedLod0[r])
      lod0Keys.AddToTail(keyOf[r]);
  }
  qsort(lod0Keys.Base(), lod0Keys.Count(), sizeof(VertKey_t), CompareVertKey);
  // Across a hard edge (split vertices), two triangles are one island only when
  // they share a whole edge: both endpoints equal in position, UV and
  // handedness. Triangles that meet only at a corner stay apart. Faces whose
  // shared edge matches end to end do merge even when they lie on top of each
  // other in UV (box faces each mapped to the full square whose common edges
  // land on the same UV edge); that island then fails the self-overlap check.
  CUtlVector<int> keyId;
  keyId.SetCount(rawCount);
  for (int r = 0; r < rawCount; ++r)
    keyId[r] = -1;
  for (int i = 0, id = -1; i < lod0Keys.Count(); ++i) {
    if (!i || CompareVertKeyNoRaw(&lod0Keys[i - 1], &lod0Keys[i]))
      ++id;
    keyId[lod0Keys[i].raw] = id;
  }
  CUtlVector<EdgeRef_t> edges;
  edges.EnsureCapacity(tris.Count() * 3);
  for (int t = 0; t < tris.Count(); ++t) {
    for (int e = 0; e < 3; ++e) {
      const int ka = keyId[tris[t].vert[e]], kb = keyId[tris[t].vert[(e + 1) % 3]];
      if (ka == kb)
        continue;
      EdgeRef_t ref;
      ref.a = MIN(ka, kb);
      ref.b = MAX(ka, kb);
      ref.tri = t;
      edges.AddToTail(ref);
    }
  }
  qsort(edges.Base(), edges.Count(), sizeof(EdgeRef_t), CompareEdgeRef);
  for (int i = 1; i < edges.Count(); ++i) {
    if (edges[i].a == edges[i - 1].a && edges[i].b == edges[i - 1].b)
      UnionSets(parent, tris[edges[i - 1].tri].vert[0], tris[edges[i].tri].vert[0]);
  }

  CUtlVector<int> rootIsland, rawIsland;
  rootIsland.SetCount(rawCount);
  rawIsland.SetCount(rawCount);
  for (int r = 0; r < rawCount; ++r) {
    rootIsland[r] = -1;
    rawIsland[r] = -1;
  }
  CUtlVector<Island_t> islands;
  for (int t = 0; t < tris.Count(); ++t) {
    const int root = FindSet(parent, tris[t].vert[0]);
    if (rootIsland[root] < 0)
      rootIsland[root] = islands.AddToTail();
    islands[rootIsland[root]].tris.AddToTail(t);
  }
  for (int r = 0; r < rawCount; ++r) {
    if (usedLod0[r])
      rawIsland[r] = rootIsland[FindSet(parent, r)];
  }

  // -- vertices only lower LODs use --------------------------------------------
  // They move with the LOD0 island they share position and UV with, else with
  // the island their LOD triangles connect them to, else with the island of a
  // LOD0 vertex with the same UV.
  const FileHeader_t *pVtxHdr = (const FileHeader_t *)vtxWalk.Base();
  if (pVtxHdr->numLODs < 1 || pVtxHdr->numLODs > MAX_NUM_LODS) {
    V_snprintf(pErr, errLen, "VTX has %d LODs", pVtxHdr->numLODs);
    return REBAKE_FAILED;
  }
  CUtlVector<ProplmVtxTri_t> lodTris;
  CUtlVector<int> lodOf, lodMat;
  for (int lod = 1; lod < pVtxHdr->numLODs; ++lod) {
    CUtlVector<ProplmVtxMesh_t> lodMeshes;
    CUtlVector<ProplmVtxTri_t> one;
    if (!Proplm_GatherVtxTriangles(pHdr, vtxWalk, verts.Count(), lod, true, true,
                                   &lodMeshes, one, pErr, errLen))
      return REBAKE_FAILED;
    for (int t = 0; t < one.Count(); ++t) {
      for (int c = 0; c < 3; ++c)
        one[t].vert[c] = verts[one[t].vert[c]].raw;
      lodTris.AddToTail(one[t]);
      lodOf.AddToTail(lod);
      lodMat.AddToTail(lodMeshes[one[t].mesh].material);
    }
  }
  // A LOD the VTX draws with replacement materials would sample textures the
  // rebake does not lay out: LOD0 with one fails, a lower one is switched off.
  int firstReplacedLod = 0;
  for (int v = 0; v < 2; ++v) {
    if (!(v ? has80 : has90))
      continue;
    for (int lod = 0; lod < pVtxHdr->numLODs; ++lod) {
      const int n = VtxLodReplacements(v ? vtx80 : vtx90, lod);
      if (n < 0) {
        V_snprintf(pErr, errLen, "VTX material replacement list is out of range");
        return REBAKE_FAILED;
      }
      if (!n)
        continue;
      if (!lod) {
        V_snprintf(pErr, errLen, "the VTX replaces LOD0 materials");
        return REBAKE_FAILED;
      }
      if (!firstReplacedLod || lod < firstReplacedLod)
        firstReplacedLod = lod;
    }
  }
  // Matching passes, strictest first; after each (but the first), LOD
  // triangles hand their island on to their still unassigned corners. The
  // UV-only pass is for decimated LODs that keep a LOD0 position but take the
  // UV of a neighbouring LOD0 vertex.
  for (int level = 0; level < 3; ++level) {
    s_KeyLevel = level;
    CUtlVector<VertKey_t> sorted;
    sorted.AddVectorToTail(lod0Keys);
    qsort(sorted.Base(), sorted.Count(), sizeof(VertKey_t), CompareVertKeyLevel);
    for (int t = 0; t < lodTris.Count(); ++t) {
      for (int c = 0; c < 3; ++c) {
        const int r = lodTris[t].vert[c];
        if (rawIsland[r] >= 0)
          continue;
        const VertKey_t *pFound = (const VertKey_t *)bsearch(
            &keyOf[r], sorted.Base(), sorted.Count(), sizeof(VertKey_t),
            CompareVertKeyLevelNoRaw);
        if (pFound)
          rawIsland[r] = rawIsland[pFound->raw];
      }
    }
    if (level == 0)
      continue; // position + UV with and without handedness first
    for (bool changed = true; changed;) {
      changed = false;
      for (int t = 0; t < lodTris.Count(); ++t) {
        int island = -1;
        for (int c = 0; c < 3 && island < 0; ++c)
          island = rawIsland[lodTris[t].vert[c]];
        if (island < 0)
          continue;
        for (int c = 0; c < 3; ++c) {
          if (rawIsland[lodTris[t].vert[c]] < 0) {
            rawIsland[lodTris[t].vert[c]] = island;
            changed = true;
          }
        }
      }
    }
  }
  // A lower-LOD triangle whose corners moved with different islands would
  // stretch across the layout; the first LOD with one and every LOD after it
  // are switched off in the copy (LOD0 is then drawn at every distance).
  int lodSpanning = 0, firstBadLod = 0;
  for (int t = 0; t < lodTris.Count(); ++t) {
    const int a = rawIsland[lodTris[t].vert[0]];
    const int b = rawIsland[lodTris[t].vert[1]];
    const int c = rawIsland[lodTris[t].vert[2]];
    if (a < 0 || b < 0 || c < 0) {
      V_snprintf(pErr, errLen,
                 "a lower-LOD vertex shares no position or UV with LOD0 and "
                 "no lower-LOD triangle connects it to one");
      return REBAKE_FAILED;
    }
    if (a != b || a != c) {
      ++lodSpanning;
      if (!firstBadLod || lodOf[t] < firstBadLod)
        firstBadLod = lodOf[t];
    }
  }
  if (firstReplacedLod && (!firstBadLod || firstReplacedLod < firstBadLod))
    firstBadLod = firstReplacedLod;

  // Island bounds over every vertex that moves with it, all LODs.
  for (int i = 0; i < islands.Count(); ++i) {
    islands[i].mn[0] = islands[i].mn[1] = 1e30;
    islands[i].mx[0] = islands[i].mx[1] = -1e30;
  }
  for (int r = 0; r < rawCount; ++r) {
    const int i = rawIsland[r];
    if (i < 0)
      continue;
    for (int k = 0; k < 2; ++k) {
      islands[i].mn[k] = MIN(islands[i].mn[k], (double)uv[r][k]);
      islands[i].mx[k] = MAX(islands[i].mx[k], (double)uv[r][k]);
    }
  }

  // -- overlap -----------------------------------------------------------------
  int overlapBefore = 0, cellsBefore = 0, selfIsland = -1;
  double selfWorst = 0.0;
  if (!MeasureOverlap(islands, tris, uv, &overlapBefore, &cellsBefore,
                      &selfWorst, &selfIsland, pErr, errLen))
    return REBAKE_FAILED;
  if (selfWorst > PROPLM_REBAKE_SELF_OVERLAP) {
    V_snprintf(pErr, errLen,
               "UV island %d of %d overlaps itself (%.1f%% of its area); "
               "moving islands cannot fix that (needs the \"vertex\" strategy)",
               selfIsland, islands.Count(), selfWorst * 100.0);
    return REBAKE_FAILED;
  }
  if (overlapBefore == 0) {
    V_snprintf(pErr, errLen,
               "its %d UV islands do not overlap when folded into one tile",
               islands.Count());
    return REBAKE_UNCHANGED;
  }

  // -- materials ---------------------------------------------------------------
  // Texture-table entry a mesh material slot names in skin family f.
  auto skinTexture = [&](int skinref, int f) {
    if (pHdr->numskinref > 0 && skinref >= 0 && skinref < pHdr->numskinref &&
        pHdr->numskinfamilies > 0)
      return (int)pHdr->pSkinref(f * pHdr->numskinref)[skinref];
    return skinref;
  };
  // Entries no drawn triangle uses, in any skin family, get no copy: no VTF,
  // no VMT, and no checks.
  CUtlVector<bool> texUsed;
  texUsed.SetCount(pHdr->numtextures);
  for (int i = 0; i < texUsed.Count(); ++i)
    texUsed[i] = false;
  auto markUsed = [&](int skinref) {
    for (int f = 0; f < MAX(1, pHdr->numskinfamilies); ++f) {
      const int tex = skinTexture(skinref, f);
      if (tex >= 0 && tex < texUsed.Count())
        texUsed[tex] = true;
    }
  };
  for (int t = 0; t < tris.Count(); ++t)
    markUsed(meshes[tris[t].mesh].material);
  for (int t = 0; t < lodTris.Count(); ++t) {
    if (!firstBadLod || lodOf[t] < firstBadLod)
      markUsed(lodMat[t]);
  }
  CUtlVector<OutTex_t> outs;
  CUtlVector<Material_t> &mats = scratch.mats;
  CUtlVector<SrcTex_t> &srcs = scratch.srcs;
  bool anyDetail = false;
  mats.SetCount(pHdr->numtextures);
  for (int i = 0; i < pHdr->numtextures; ++i) {
    Material_t &m = mats[i];
    m.pKV = NULL;
    m.hasDetail = false;
    m.material[0] = '\0';
    V_strncpy(m.texName, pHdr->pTexture(i)->pszName(), sizeof(m.texName));
    V_FixSlashes(m.texName, '/');
    V_strlower(m.texName);
    // The engine takes the first cdtexture under which the material exists.
    for (int j = 0; j < pHdr->numcdtextures && !m.material[0]; ++j) {
      char cand[MAX_PATH], vmt[MAX_PATH];
      V_snprintf(cand, sizeof(cand), "%s%s", pHdr->pCdtexture(j), m.texName);
      NormalizeTexName(cand, cand, sizeof(cand));
      V_FixDoubleSlashes(cand);
      V_snprintf(vmt, sizeof(vmt), "materials/%s.vmt", cand);
      if (GameOrPakFileExists(vmt))
        V_strncpy(m.material, cand, sizeof(m.material));
    }
    if (!texUsed[i])
      continue; // drawn by nothing that stays enabled
    if (!m.material[0])
      continue; // missing before, missing after: no copy to write
    m.pKV = LoadMaterialKeyValues(m.material, LOAD_MATERIAL_KEY_VALUES_FLAGS_EXPAND_PATCH);
    if (!m.pKV) {
      V_snprintf(pErr, errLen, "cannot parse material %s", m.material);
      return REBAKE_FAILED;
    }
    // The material's own keys, then each shader fallback block ("<dx90" and
    // the like). A block's own keys win over the material's when the engine
    // picks it, so every check runs on every block, and a block without its
    // own transform uses the material's.
    CUtlVector<KeyValues *> &blocks = m.blocks;
    blocks.AddToTail(m.pKV);
    for (KeyValues *p = m.pKV->GetFirstTrueSubKey(); p; p = p->GetNextTrueSubKey()) {
      if (V_stricmp(p->GetName(), "Proxies"))
        blocks.AddToTail(p);
    }
    for (int bl = 0; bl < blocks.Count(); ++bl) {
      KeyValues *pBlock = blocks[bl];
      if (KeyValues *pProxies = pBlock->FindKey("Proxies")) {
        for (KeyValues *p = pProxies->GetFirstSubKey(); p; p = p->GetNextKey()) {
          for (int k = 0; k < (int)ARRAYSIZE(s_UvProxies); ++k) {
            if (!V_stricmp(p->GetName(), s_UvProxies[k])) {
              V_snprintf(pErr, errLen,
                         "material %s animates its texture (%s proxy), which a "
                         "fixed layout cannot follow",
                         m.material, s_UvProxies[k]);
              return REBAKE_FAILED;
            }
          }
        }
      }
      if (pBlock->GetString("$detail", "")[0])
        m.hasDetail = true;
      // $envmapmask is laid out through $basetexturetransform (checked equal
      // below), so its own transform goes too.
      if (pBlock->GetString("$envmapmasktransform", NULL)) {
        TexKey_t drop;
        drop.pBlock = pBlock;
        V_strncpy(drop.key, "$envmapmasktransform", sizeof(drop.key));
        drop.out = -1;
        m.dropKeys.AddToTail(drop);
      }
    }
    anyDetail |= m.hasDetail;

    // The material's own transforms: a texture key a fallback block inherits
    // is laid out through these.
    Affine_t rootXf[2];
    ParseTransform(m.pKV->GetString("$basetexturetransform", NULL), rootXf[0]);
    ParseTransform(m.pKV->GetString("$bumptransform", NULL), rootXf[1]);
    auto isUv0Image = [&](const char *pValue) {
      if (!pValue || !pValue[0])
        return false;
      char name[MAX_PATH];
      NormalizeTexName(pValue, name, sizeof(name));
      return name[0] != '_' && V_stricmp(name, "env_cubemap") != 0;
    };

    for (int bl = 0; bl < blocks.Count(); ++bl)
    for (int group = 0; group < 2; ++group) {
      KeyValues *pBlock = blocks[bl];
      const char **ppKeys = group ? s_BumpUvKeys : s_BaseUvKeys;
      const int keyCount = group ? ARRAYSIZE(s_BumpUvKeys) : ARRAYSIZE(s_BaseUvKeys);
      const char *pXfKey = group ? "$bumptransform" : "$basetexturetransform";
      Affine_t xf;
      const char *pXf = pBlock->GetString(pXfKey, NULL);
      if (pXf) {
        TexKey_t drop;
        drop.pBlock = pBlock;
        V_strncpy(drop.key, pXfKey, sizeof(drop.key));
        drop.out = -1;
        m.dropKeys.AddToTail(drop);
      } else if (bl) {
        pXf = m.pKV->GetString(pXfKey, NULL);
      }
      if (!ParseTransform(pXf, xf)) {
        V_snprintf(pErr, errLen, "material %s: cannot parse %s \"%s\"",
                   m.material, pXfKey, pXf);
        return REBAKE_FAILED;
      }
      // A fallback block with its own transform that inherits a texture of
      // this group from the material: the texture is laid out through the
      // material's transform and the block's is dropped, so they must agree.
      // ($envmapmask follows $envmapmasktransform, checked next.)
      if (bl && pBlock->GetString(pXfKey, NULL) &&
          !SameTransform(xf, rootXf[group])) {
        for (int k = 0; k < keyCount; ++k) {
          if (!group && !V_stricmp(ppKeys[k], "$envmapmask"))
            continue;
          if (pBlock->GetString(ppKeys[k], NULL) ||
              !isUv0Image(m.pKV->GetString(ppKeys[k], NULL)))
            continue;
          V_snprintf(pErr, errLen,
                     "material %s: fallback block \"%s\" has its own %s but "
                     "inherits %s, which is laid out through the material's",
                     m.material, pBlock->GetName(), pXfKey, ppKeys[k]);
          return REBAKE_FAILED;
        }
      }
      // The same for an inherited $envmapmask under the block's own
      // $envmapmasktransform: the mask was laid out through the material's
      // $basetexturetransform.
      if (bl && !group && !pBlock->GetString("$envmapmask", NULL) &&
          isUv0Image(m.pKV->GetString("$envmapmask", NULL))) {
        const char *pMaskXf = pBlock->GetString("$envmapmasktransform", NULL);
        Affine_t maskXf;
        if (pMaskXf && (!ParseTransform(pMaskXf, maskXf) ||
                        !SameTransform(maskXf, rootXf[0]))) {
          V_snprintf(pErr, errLen,
                     "material %s: fallback block \"%s\" has its own "
                     "$envmapmasktransform but inherits $envmapmask, which is "
                     "laid out through the material's $basetexturetransform",
                     m.material, pBlock->GetName());
          return REBAKE_FAILED;
        }
      }
      for (int k = 0; k < keyCount; ++k) {
        const char *pValue = pBlock->GetString(ppKeys[k], "");
        if (!pValue[0])
          continue;
        char texName[MAX_PATH];
        NormalizeTexName(pValue, texName, sizeof(texName));
        if (texName[0] == '_' || !V_stricmp(texName, "env_cubemap"))
          continue; // render targets and the like are not UV0 images
        if (!group && !V_stricmp(ppKeys[k], "$envmapmask")) {
          // The mask is re-laid with the base texture's transform; a mask
          // mapped through a different one cannot share the layout.
          const char *pMaskXf = pBlock->GetString("$envmapmasktransform", NULL);
          if (!pMaskXf && bl)
            pMaskXf = m.pKV->GetString("$envmapmasktransform", NULL);
          Affine_t maskXf;
          if (!ParseTransform(pMaskXf, maskXf)) {
            V_snprintf(pErr, errLen,
                       "material %s: cannot parse $envmapmasktransform \"%s\"",
                       m.material, pMaskXf);
            return REBAKE_FAILED;
          }
          if (!SameTransform(maskXf, xf)) {
            V_snprintf(pErr, errLen,
                       "material %s: $envmapmask uses a different transform "
                       "from $basetexture",
                       m.material);
            return REBAKE_FAILED;
          }
        }
        int src = -1;
        for (int s = 0; s < srcs.Count() && src < 0; ++s) {
          if (!V_strcmp(srcs[s].name, texName))
            src = s;
        }
        if (src < 0) {
          SrcTex_t st;
          memset(&st, 0, sizeof(st));
          V_strncpy(st.name, texName, sizeof(st.name));
          char vtfPath[MAX_PATH];
          V_snprintf(vtfPath, sizeof(vtfPath), "materials/%s.vtf", texName);
          CUtlBuffer vtfBuf;
          if (!ReadGameOrPakFile(vtfPath, vtfBuf)) {
            V_snprintf(pErr, errLen, "material %s: cannot read %s", m.material,
                       vtfPath);
            return REBAKE_FAILED;
          }
          st.pTex = CreateVTFTexture();
          src = srcs.AddToTail(st); // owned by scratch from here on
          SrcTex_t &s = srcs[src];
          if (!s.pTex->Unserialize(vtfBuf)) {
            V_snprintf(pErr, errLen, "cannot parse %s", vtfPath);
            return REBAKE_FAILED;
          }
          if (s.pTex->FrameCount() != 1 || s.pTex->IsCubeMap() ||
              s.pTex->Depth() != 1) {
            V_snprintf(pErr, errLen,
                       "%s is animated, a cubemap or a volume texture", vtfPath);
            return REBAKE_FAILED;
          }
          if (!IsSupportedFormat(s.pTex->Format())) {
            V_snprintf(pErr, errLen, "%s has format %s, which is not handled",
                       vtfPath, ImageLoader::GetName(s.pTex->Format()));
            return REBAKE_FAILED;
          }
          s.width = s.pTex->Width();
          s.height = s.pTex->Height();
          s.format = s.pTex->Format();
          s.flags = s.pTex->Flags();
          s.bumpScale = s.pTex->BumpScale();
          s.reflectivity = s.pTex->Reflectivity();
          s.pTex->ConvertImageFormat(IMAGE_FORMAT_BGRA8888, false);
          if (s.pTex->Format() != IMAGE_FORMAT_BGRA8888) {
            V_snprintf(pErr, errLen, "cannot decode %s", vtfPath);
            return REBAKE_FAILED;
          }
        }
        int out = -1;
        for (int o = 0; o < outs.Count() && out < 0; ++o) {
          if (outs[o].src == src && SameTransform(outs[o].xf, xf))
            out = o;
        }
        if (out < 0) {
          out = outs.AddToTail();
          outs[out].src = src;
          outs[out].xf = xf;
          // Name: the source file's base name, made unique within the model.
          char base[MAX_PATH], cand[MAX_PATH];
          V_FileBase(texName, base, sizeof(base));
          V_snprintf(cand, sizeof(cand), "%s/%s", rebakeDir, base);
          for (int n = 2;; ++n) {
            bool taken = false;
            for (int o = 0; o < out && !taken; ++o)
              taken = !V_strcmp(outs[o].name, cand);
            if (!taken)
              break;
            V_snprintf(cand, sizeof(cand), "%s/%s_%d", rebakeDir, base, n);
          }
          V_strncpy(outs[out].name, cand, sizeof(outs[out].name));
        }
        if (outs[out].users.Find(i) < 0)
          outs[out].users.AddToTail(i);
        TexKey_t tk;
        tk.pBlock = pBlock;
        V_strncpy(tk.key, ppKeys[k], sizeof(tk.key));
        tk.out = out;
        m.keys.AddToTail(tk);
      }
    }
  }

  // Which texture-table entries each island's triangles use, in any skin:
  // LOD0's, and those of every lower LOD that stays enabled, so a VTF is
  // filled wherever any drawn triangle samples it.
  auto addTextures = [&](Island_t &isl, int skinref) {
    for (int f = 0; f < MAX(1, pHdr->numskinfamilies); ++f) {
      const int tex = skinTexture(skinref, f);
      if (tex >= 0 && tex < pHdr->numtextures && isl.textures.Find(tex) < 0)
        isl.textures.AddToTail(tex);
    }
  };
  for (int i = 0; i < islands.Count(); ++i) {
    Island_t &isl = islands[i];
    for (int n = 0; n < isl.tris.Count(); ++n)
      addTextures(isl, meshes[tris[isl.tris[n]].mesh].material);
  }
  for (int t = 0; t < lodTris.Count(); ++t) {
    if (firstBadLod && lodOf[t] >= firstBadLod)
      continue;
    for (int c = 0; c < 3; ++c)
      addTextures(islands[rawIsland[lodTris[t].vert[c]]], lodMat[t]);
  }

  // -- layout ------------------------------------------------------------------
  // Grid: offsets are whole multiples of PROPLM_REBAKE_TEX_SNAP texels of
  // every texture (gcd of the sizes), times the downscale.
  int64 gcdW = 0, gcdH = 0;
  int minW = 0, minH = 0;
  for (int o = 0; o < outs.Count(); ++o) {
    const SrcTex_t &s = srcs[outs[o].src];
    gcdW = GcdInt(gcdW, s.width);
    gcdH = GcdInt(gcdH, s.height);
    minW = minW ? MIN(minW, s.width) : s.width;
    minH = minH ? MIN(minH, s.height) : s.height;
  }
  if (!gcdW) {
    // No texture to copy (every material missing): lay out for the lightmap
    // only, on a 1/64 grid.
    gcdW = gcdH = 256;
    minW = minH = 256;
  }
  CUtlVector<int> order;
  for (int i = 0; i < islands.Count(); ++i)
    order.AddToTail(i);
  {
    // Tallest first, then widest, then island order: deterministic.
    struct Cmp {
      static const CUtlVector<Island_t> *&Islands() {
        static const CUtlVector<Island_t> *p;
        return p;
      }
      static int __cdecl Fn(const void *pa, const void *pb) {
        const Island_t &a = (*Islands())[*(const int *)pa];
        const Island_t &b = (*Islands())[*(const int *)pb];
        const double ha = a.mx[1] - a.mn[1], hb = b.mx[1] - b.mn[1];
        if (ha != hb)
          return ha > hb ? -1 : 1;
        const double wa = a.mx[0] - a.mn[0], wb = b.mx[0] - b.mn[0];
        if (wa != wb)
          return wa > wb ? -1 : 1;
        return *(const int *)pa - *(const int *)pb;
      }
    };
    Cmp::Islands() = &islands;
    qsort(order.Base(), order.Count(), sizeof(int), Cmp::Fn);
  }

  double size[2] = {0, 0}, pad[2] = {0, 0}, grid[2] = {0, 0};
  int down = 1;
  bool packed = false;
  for (; down <= PROPLM_REBAKE_MAX_DOWNSCALE && !packed; down *= 2) {
    grid[0] = (double)(PROPLM_REBAKE_TEX_SNAP * down) / (double)gcdW;
    grid[1] = (double)(PROPLM_REBAKE_TEX_SNAP * down) / (double)gcdH;
    // Candidate layouts: powers of two per side, at most 2:1, by area, square
    // first. A model with a $detail texture stays square ($detailscale is one
    // number).
    for (int areaExp = 2 * PROPLM_REBAKE_MIN_POW2;
         areaExp <= 2 * PROPLM_REBAKE_MAX_POW2 && !packed; ++areaExp) {
      for (int skew = 0; skew <= 1 && !packed; ++skew) {
        int eu, ev;
        if (areaExp % 2 == 0) {
          if (skew)
            continue;
          eu = ev = areaExp / 2;
        } else {
          if (anyDetail)
            continue;
          // 2:1 either way; wider first.
          eu = skew ? (int)floor(areaExp / 2.0) : (int)ceil(areaExp / 2.0);
          ev = areaExp - eu;
        }
        if (eu < PROPLM_REBAKE_MIN_POW2 || eu > PROPLM_REBAKE_MAX_POW2 ||
            ev < PROPLM_REBAKE_MIN_POW2 || ev > PROPLM_REBAKE_MAX_POW2)
          continue;
        const double cand[2] = {ldexp(1.0, eu), ldexp(1.0, ev)};
        // Whole grid steps, whole output texels, and under the size cap.
        bool ok = true;
        for (int k = 0; k < 2 && ok; ++k) {
          const double steps = cand[k] / grid[k];
          ok = fabs(steps - floor(steps + 0.5)) < 1e-9 && steps >= 1.0;
        }
        for (int o = 0; o < outs.Count() && ok; ++o) {
          const SrcTex_t &s = srcs[outs[o].src];
          ok = cand[0] * s.width / down <= PROPLM_REBAKE_MAX_TEX &&
               cand[1] * s.height / down <= PROPLM_REBAKE_MAX_TEX;
        }
        if (!ok)
          continue;
        // Padding: PROPLM_REBAKE_TEX_PAD texels of the coarsest output
        // texture and PROPLM_REBAKE_LM_PAD lightmap texels.
        const double p[2] = {
            MAX((double)PROPLM_REBAKE_TEX_PAD * down / minW,
                (double)PROPLM_REBAKE_LM_PAD * cand[0] / pR->lmResX),
            MAX((double)PROPLM_REBAKE_TEX_PAD * down / minH,
                (double)PROPLM_REBAKE_LM_PAD * cand[1] / pR->lmResY)};
        if (ShelfPack(islands, order, cand, p, grid)) {
          size[0] = cand[0];
          size[1] = cand[1];
          pad[0] = p[0];
          pad[1] = p[1];
          packed = true;
        }
      }
    }
    if (packed)
      break;
  }
  if (!packed) {
    V_snprintf(pErr, errLen,
               "%d UV islands do not fit a %dx%d-repeat layout within %d texels "
               "per side",
               islands.Count(), 1 << PROPLM_REBAKE_MAX_POW2,
               1 << PROPLM_REBAKE_MAX_POW2, PROPLM_REBAKE_MAX_TEX);
    return REBAKE_FAILED;
  }

  // -- new UVs -------------------------------------------------------------------
  CUtlVector<Vector2D> newUv;
  newUv.SetCount(rawCount);
  CUtlBuffer vvdOut;
  vvdOut.Put(vvdFile.Base(), vvdFile.TellPut());
  mstudiovertex_t *pOutVerts =
      (mstudiovertex_t *)((byte *)vvdOut.Base() + pVvd->vertexDataStart);
  for (int r = 0; r < rawCount; ++r) {
    newUv[r] = uv[r];
    const int i = rawIsland[r];
    if (i < 0)
      continue; // no triangle of any LOD uses it
    for (int k = 0; k < 2; ++k) {
      const double n = ((double)uv[r][k] + islands[i].t[k]) / size[k];
      if (!(n >= -1e-5 && n <= 1.0 + 1e-5)) {
        V_snprintf(pErr, errLen, "internal: vertex %d lands at %g outside 0..1",
                   r, n);
        return REBAKE_FAILED;
      }
      newUv[r][k] = (float)n;
    }
    pOutVerts[r].m_vecTexCoord = newUv[r];
  }
  int overlapAfter = 0, cellsAfter = 0, dummyIsland = -1;
  double dummySelf = 0.0;
  if (!MeasureOverlap(islands, tris, newUv, &overlapAfter, &cellsAfter,
                      &dummySelf, &dummyIsland, pErr, errLen))
    return REBAKE_FAILED;

  // -- textures --------------------------------------------------------------------
  int pakBytes = 0;
  char texLog[1024] = "";
  for (int o = 0; o < outs.Count(); ++o) {
    const OutTex_t &out = outs[o];
    const SrcTex_t &s = srcs[out.src];
    const int outW = (int)(size[0] * s.width / down + 0.5);
    const int outH = (int)(size[1] * s.height / down + 0.5);
    // Output texels per grid step: grid * size per unit / down.
    const int64 snapTexels[2] = {(int64)PROPLM_REBAKE_TEX_SNAP * s.width / gcdW,
                                 (int64)PROPLM_REBAKE_TEX_SNAP * s.height / gcdH};
    CUtlVector<int> used;
    for (int i = 0; i < islands.Count(); ++i) {
      for (int n = 0; n < islands[i].textures.Count(); ++n) {
        if (out.users.Find(islands[i].textures[n]) >= 0) {
          used.AddToTail(i);
          break;
        }
      }
    }
    if (!used.Count()) {
      // Only a material no LOD0 triangle uses samples it (a lower-LOD
      // replacement, say): every island, since they share one layout.
      for (int i = 0; i < islands.Count(); ++i)
        used.AddToTail(i);
    }
    CUtlVector<unsigned char> image, mask;
    image.SetCount(outW * outH * 4);
    mask.SetCount(outW * outH);
    memset(mask.Base(), 0, mask.Count());
    FillIslands(out, s, islands, used, size, pad, down, snapTexels, outW, outH,
                image.Base(), mask.Base());
    Dilate(outW, outH, image.Base(), mask.Base());
    CUtlBuffer vtfOut;
    ImageFormat written;
    if (!EncodeVtf(s, outW, outH, image.Base(), vtfOut, &written, pErr, errLen))
      return REBAKE_FAILED;
    char vtfPath[MAX_PATH];
    V_snprintf(vtfPath, sizeof(vtfPath), "materials/%s.vtf", out.name);
    pakBytes += AddToPak(vtfPath, vtfOut.Base(), vtfOut.TellPut(), false);
    char one[256];
    V_snprintf(one, sizeof(one), "%s%s %dx%d %s", o ? ", " : "", V_UnqualifiedFileName(out.name),
               outW, outH, ImageLoader::GetName(written));
    V_strncat(texLog, one, sizeof(texLog));
  }

  // -- materials -------------------------------------------------------------------
  for (int i = 0; i < mats.Count(); ++i) {
    Material_t &m = mats[i];
    if (!m.pKV)
      continue;
    for (int k = 0; k < m.keys.Count(); ++k)
      m.keys[k].pBlock->SetString(m.keys[k].key, outs[m.keys[k].out].name);
    for (int k = 0; k < m.dropKeys.Count(); ++k) {
      KeyValues *pBlock = m.dropKeys[k].pBlock;
      KeyValues *pDrop = pBlock->FindKey(m.dropKeys[k].key);
      if (pDrop) {
        pBlock->RemoveSubKey(pDrop);
        pDrop->deleteThis();
      }
    }
    if (m.hasDetail) {
      // Detail coordinates are UV0 * $detailscale: scale every block's own
      // $detailscale by the layout size so the detail density on the surface
      // does not change, and give the material one (scaled default 4) when it
      // has none, which blocks without their own inherit.
      bool rootScaled = false;
      for (int bl = 0; bl < m.blocks.Count(); ++bl) {
        const char *pScale = m.blocks[bl]->GetString("$detailscale", NULL);
        if (!pScale)
          continue;
        char value[128];
        ScaleDetail(pScale, size, value, sizeof(value));
        m.blocks[bl]->SetString("$detailscale", value);
        rootScaled |= bl == 0;
      }
      if (!rootScaled) {
        char value[128];
        ScaleDetail("4", size, value, sizeof(value));
        m.pKV->SetString("$detailscale", value);
      }
    }
    CUtlBuffer vmt(0, 0, CUtlBuffer::TEXT_BUFFER);
    m.pKV->RecursiveSaveToFile(vmt, 0);
    char vmtPath[MAX_PATH];
    V_snprintf(vmtPath, sizeof(vmtPath), "materials/%s/%s.vmt", rebakeDir,
               m.texName);
    V_FixDoubleSlashes(vmtPath);
    pakBytes += AddToPak(vmtPath, vmt.Base(), vmt.TellPut(), true);
  }

  // -- model files -----------------------------------------------------------------
  // VRAD reads .dx80.vtx and the engine .dx90.vtx: write both, the missing one
  // as a copy of the other (the layout is the same).
  CUtlBuffer v90, v80;
  v90.Put((has90 ? vtx90 : vtx80).Base(), (has90 ? vtx90 : vtx80).TellPut());
  v80.Put((has80 ? vtx80 : vtx90).Base(), (has80 ? vtx80 : vtx90).TellPut());
  if (firstBadLod && (!DisableLods(pHdr, v90, firstBadLod, pErr, errLen) ||
                      !DisableLods(pHdr, v80, firstBadLod, pErr, errLen)))
    return REBAKE_FAILED;
  CUtlBuffer mdlOut;
  if (!RewriteMdl(mdlFile, internalName, cdTexture, firstBadLod, mdlOut, pErr,
                  errLen))
    return REBAKE_FAILED;
  char stem[MAX_PATH], outPath[MAX_PATH];
  V_strncpy(stem, pR->dictName, sizeof(stem));
  stem[V_strlen(stem) - 4] = '\0'; // ".mdl"
  V_snprintf(outPath, sizeof(outPath), "%s.mdl", stem);
  pakBytes += AddToPak(outPath, mdlOut.Base(), mdlOut.TellPut(), false);
  V_snprintf(outPath, sizeof(outPath), "%s.vvd", stem);
  pakBytes += AddToPak(outPath, vvdOut.Base(), vvdOut.TellPut(), false);
  V_snprintf(outPath, sizeof(outPath), "%s.dx90.vtx", stem);
  pakBytes += AddToPak(outPath, v90.Base(), v90.TellPut(), false);
  V_snprintf(outPath, sizeof(outPath), "%s.dx80.vtx", stem);
  pakBytes += AddToPak(outPath, v80.Base(), v80.TellPut(), false);
  if (hasPhy) {
    V_snprintf(outPath, sizeof(outPath), "%s.phy", stem);
    pakBytes += AddToPak(outPath, phyFile.Base(), phyFile.TellPut(), false);
  }
  pR->pakBytes = pakBytes;

  char lodLog[200];
  if (firstBadLod) {
    char why[96];
    if (firstReplacedLod)
      V_snprintf(why, sizeof(why), "; LOD %d replaces materials", firstReplacedLod);
    else
      why[0] = '\0';
    V_snprintf(lodLog, sizeof(lodLog),
               "LOD %d and later switched off (%d lower-LOD triangles span two "
               "islands%s)",
               firstBadLod, lodSpanning, why);
  } else
    V_snprintf(lodLog, sizeof(lodLog), "%d lower LOD(s) kept",
               pVtxHdr->numLODs - 1);
  Msg("proplm rebake: %s: %d islands, UV overlap folded into one tile %.1f%% -> "
      "%.1f%%, "
      "layout %gx%g repeats%s, %s; textures: %s; %d bytes added to the pak\n",
      pKey, islands.Count(), 100.0 * overlapBefore / MAX(1, cellsBefore),
      100.0 * overlapAfter / MAX(1, cellsAfter), size[0], size[1],
      down > 1 ? " (textures scaled down to fit the size cap)" : "",
      lodLog, texLog[0] ? texLog : "none", pakBytes);
  if (down > 1) {
    Warning("Warning: proplm rebake: %s: textures scaled down %dx to stay within "
            "%d texels per side.\n",
            pKey, down, PROPLM_REBAKE_MAX_TEX);
  }
  return REBAKE_DONE;
}

//-----------------------------------------------------------------------------
// Public
//-----------------------------------------------------------------------------
static bool IsStaticPropEntity(entity_t *pEnt) {
  const char *pClass = ValueForKey(pEnt, "classname");
  return !strcmp(pClass, "static_prop") || !strcmp(pClass, "prop_static");
}

void ProplmRebake_Prepare() {
  s_Rebakes.PurgeAndDeleteElements();
  s_Props.Purge();

  for (int i = 0; i < num_entities; ++i) {
    entity_t *pEnt = &entities[i];
    if (!IsStaticPropEntity(pEnt))
      continue;
    const char *pProplm = ValueForKey(pEnt, "proplm");
    const bool bExplicit = !V_stricmp(pProplm, "rebake");
    // proplm "1" is the uv2 route (the prop becomes a prop_dynamic).
    const bool bImplicit = g_bProplmRebakeAll && IntForKey(pEnt, "proplm") != 1 &&
                           IntForKey(pEnt, "generatelightmaps") != 0;
    if (!bExplicit && !bImplicit)
      continue;
    const char *pModel = ValueForKey(pEnt, "model");
    if (!pModel[0])
      continue;
    if (IntForKey(pEnt, "generatelightmaps") == 0) {
      Warning("Warning: proplm rebake: prop \"%s\" at (%g %g %g) keeps its "
              "model: %s, so it has no lightmap to rebake for.\n",
              pModel, pEnt->origin.x, pEnt->origin.y, pEnt->origin.z,
              ValueForKey(pEnt, "generatelightmaps")[0]
                  ? "it has generatelightmaps 0"
                  : "it has no generatelightmaps key (read as 0)");
      continue;
    }

    char key[MAX_PATH];
    V_strncpy(key, pModel, sizeof(key));
    V_FixSlashes(key, '/');
    V_strlower(key);
    int lmX = IntForKey(pEnt, "lightmapresolutionx");
    int lmY = IntForKey(pEnt, "lightmapresolutiony");
    if (lmX <= 0)
      lmX = PROPLM_REBAKE_LM_DEFAULT;
    if (lmY <= 0)
      lmY = PROPLM_REBAKE_LM_DEFAULT;

    int r = -1;
    for (int k = 0; k < s_Rebakes.Count() && r < 0; ++k) {
      if (!strcmp(s_Rebakes[k]->key, key))
        r = k;
    }
    if (r < 0) {
      Rebake_t *pR = new Rebake_t;
      memset(pR, 0, sizeof(*pR));
      V_strncpy(pR->key, key, sizeof(pR->key));
      pR->lmResX = lmX;
      pR->lmResY = lmY;
      pR->state = REBAKE_PENDING;
      r = s_Rebakes.AddToTail(pR);
    }
    // The coarsest lightmap among the instances sets the padding.
    Rebake_t *pR = s_Rebakes[r];
    pR->lmResX = MIN(pR->lmResX, lmX);
    pR->lmResY = MIN(pR->lmResY, lmY);
    ++pR->props;
    PropReq_t req;
    req.ent = i;
    req.rebake = r;
    s_Props.AddToTail(req);
  }
}

const char *ProplmRebake_ModelForProp(entity_t *pEnt) {
  if (!pEnt)
    return NULL;
  const int ent = (int)(pEnt - entities);
  for (int p = 0; p < s_Props.Count(); ++p) {
    if (s_Props[p].ent != ent)
      continue;
    // Built on the first prop that VBSP actually emits, so a prop it refuses
    // (not a valid static prop, outside the map) costs no pak space.
    Rebake_t *pR = s_Rebakes[s_Props[p].rebake];
    if (pR->state == REBAKE_PENDING) {
      pR->state = BuildRebake(pR, pR->reason, sizeof(pR->reason));
      if (pR->state == REBAKE_UNCHANGED)
        Msg("proplm rebake: %s: %s; model left unchanged.\n", pR->key,
            pR->reason);
    }
    if (pR->state == REBAKE_FAILED) {
      Warning("Warning: proplm rebake: prop \"%s\" at (%g %g %g) keeps %s: "
              "%s.\n",
              ValueForKey(pEnt, "model"), pEnt->origin.x, pEnt->origin.y,
              pEnt->origin.z,
              pR->keepsPatched
                  ? "the patched model from -forcedynamicpropsasstatic"
                  : "its model",
              pR->reason);
    }
    if (pR->state != REBAKE_DONE)
      return NULL;
    ++pR->emitted;
    return pR->dictName;
  }
  return NULL;
}

void ProplmRebake_Summary() {
  if (!s_Props.Count())
    return;
  int done = 0, unchanged = 0, failed = 0, props = 0, bytes = 0;
  for (int r = 0; r < s_Rebakes.Count(); ++r) {
    const Rebake_t *pR = s_Rebakes[r];
    if (pR->state == REBAKE_DONE) {
      ++done;
      props += pR->emitted;
      bytes += pR->pakBytes;
    } else if (pR->state == REBAKE_UNCHANGED) {
      ++unchanged;
    } else if (pR->state == REBAKE_FAILED) {
      ++failed;
    }
  }
  Msg("proplm rebake: %d model(s) rebaked for %d prop(s), %d model(s) needed "
      "no rebake, %d model(s) failed; %d bytes added to the pak.\n",
      done, props, unchanged, failed, bytes);
}
