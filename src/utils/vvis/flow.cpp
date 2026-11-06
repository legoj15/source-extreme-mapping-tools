//========= Copyright Valve Corporation, All rights reserved. ============//
//
// Purpose:
//
// $NoKeywords: $
//
//=============================================================================//
#include "vis.h"
#include "vmpi.h"
#include <emmintrin.h> // SSE2
#include "threads.h"
#include <intrin.h>
#include <limits.h>

int g_TraceClusterStart = -1;
int g_TraceClusterStop = -1;
/*

  each portal will have a list of all possible to see from first portal

  if (!thread->portalmightsee[portalnum])

  portal mightsee

  for p2 = all other portals in leaf
        get sperating planes
        for all portals that might be seen by p2
                mark as unseen if not present in seperating plane
        flood fill a new mightsee
        save as passagemightsee


  void CalcMightSee (leaf_t *leaf,
*/

int CountBits(byte *bits, int numbits) {
  int i;
  int c;

  c = 0;
  for (i = 0; i < numbits; i++)
    if (CheckBit(bits, i))
      c++;

  return c;
}

int c_fullskip;
int c_portalskip, c_leafskip;
int c_vistest, c_mighttest;

int c_chop, c_nochop;

int active;

#ifdef MPI
extern bool g_bVMPIEarlyExit;
#endif

void CheckStack(leaf_t *leaf, threaddata_t *thread) {
  pstack_t *p, *p2;

  for (p = thread->pstack_head.next; p; p = p->next) {
    //		Msg ("=");
    if (p->leaf == leaf)
      Error("CheckStack: leaf recursion");
    for (p2 = thread->pstack_head.next; p2 != p; p2 = p2->next)
      if (p2->leaf == p->leaf)
        Error("CheckStack: late leaf recursion");
  }
  //	Msg ("\n");
}

winding_t *AllocStackWinding(pstack_t *stack) {
  int i;

  for (i = 0; i < 3; i++) {
    if (stack->freewindings[i]) {
      stack->freewindings[i] = 0;
      return &stack->windings[i];
    }
  }

  Error("Out of memory. AllocStackWinding: failed");

  return NULL;
}

void FreeStackWinding(winding_t *w, pstack_t *stack) {
  int i;

  i = w - stack->windings;

  if (i < 0 || i > 2)
    return; // not from local

  if (stack->freewindings[i])
    Error("FreeStackWinding: allready free");
  stack->freewindings[i] = 1;
}

/*
==============
ChopWinding

==============
*/

#ifdef _WIN32
#pragma warning(disable : 4701)
#endif

winding_t *ChopWinding(winding_t *in, pstack_t *stack, plane_t *split) {
  vec_t dists[128];
  int sides[128];
  int counts[3];
  vec_t dot;
  int i, j;
  Vector mid;
  winding_t *neww;

  counts[0] = counts[1] = counts[2] = 0;

  // determine sides for each point
  for (i = 0; i < in->numpoints; i++) {
    dot = DotProduct(in->points[i], split->normal);
    dot -= split->dist;
    dists[i] = dot;
    if (dot > ON_VIS_EPSILON)
      sides[i] = SIDE_FRONT;
    else if (dot < -ON_VIS_EPSILON)
      sides[i] = SIDE_BACK;
    else {
      sides[i] = SIDE_ON;
    }
    counts[sides[i]]++;
  }

  if (!counts[1])
    return in; // completely on front side

  if (!counts[0]) {
    FreeStackWinding(in, stack);
    return NULL;
  }

  sides[i] = sides[0];
  dists[i] = dists[0];

  neww = AllocStackWinding(stack);

  neww->numpoints = 0;

  for (i = 0; i < in->numpoints; i++) {
    Vector &p1 = in->points[i];

    if (neww->numpoints == MAX_POINTS_ON_FIXED_WINDING) {
      FreeStackWinding(neww, stack);
      return in; // can't chop -- fall back to original
    }

    if (sides[i] == SIDE_ON) {
      VectorCopy(p1, neww->points[neww->numpoints]);
      neww->numpoints++;
      continue;
    }

    if (sides[i] == SIDE_FRONT) {
      VectorCopy(p1, neww->points[neww->numpoints]);
      neww->numpoints++;
    }

    if (sides[i + 1] == SIDE_ON || sides[i + 1] == sides[i])
      continue;

    if (neww->numpoints == MAX_POINTS_ON_FIXED_WINDING) {
      FreeStackWinding(neww, stack);
      return in; // can't chop -- fall back to original
    }

    // generate a split point
    Vector &p2 = in->points[(i + 1) % in->numpoints];

    dot = dists[i] / (dists[i] - dists[i + 1]);
    for (j = 0; j < 3; j++) { // avoid round off error when possible
      if (split->normal[j] == 1)
        mid[j] = split->dist;
      else if (split->normal[j] == -1)
        mid[j] = -split->dist;
      else
        mid[j] = p1[j] + dot * (p2[j] - p1[j]);
    }

    VectorCopy(mid, neww->points[neww->numpoints]);
    neww->numpoints++;
  }

  // free the original winding
  FreeStackWinding(in, stack);

  return neww;
}

#ifdef _WIN32
#pragma warning(default : 4701)
#endif

/*
==============
ClipToSeperators

Source, pass, and target are an ordering of portals.

Generates seperating planes canidates by taking two points from source and one
point from pass, and clips target by them.

If target is totally clipped away, that portal can not be seen through.

Normal clip keeps target on the same side as pass, which is correct if the
order goes source, pass, target.  If the order goes pass, source, target then
flipclip should be set.
==============
*/
winding_t *ClipToSeperators(winding_t *source, winding_t *pass,
                            winding_t *target, bool flipclip, pstack_t *stack) {
  int i, j, k, l;
  plane_t plane;
  Vector v1, v2;
  float d;
  vec_t length;
  int counts[3];
  bool fliptest;

  // check all combinations
  for (i = 0; i < source->numpoints; i++) {
    l = (i + 1) % source->numpoints;
    VectorSubtract(source->points[l], source->points[i], v1);

    // fing a vertex of pass that makes a plane that puts all of the
    // vertexes of pass on the front side and all of the vertexes of
    // source on the back side
    for (j = 0; j < pass->numpoints; j++) {
      VectorSubtract(pass->points[j], source->points[i], v2);

      plane.normal[0] = v1[1] * v2[2] - v1[2] * v2[1];
      plane.normal[1] = v1[2] * v2[0] - v1[0] * v2[2];
      plane.normal[2] = v1[0] * v2[1] - v1[1] * v2[0];

      // if points don't make a valid plane, skip it

      length = plane.normal[0] * plane.normal[0] +
               plane.normal[1] * plane.normal[1] +
               plane.normal[2] * plane.normal[2];

      if (length < ON_VIS_EPSILON)
        continue;

      length = 1 / sqrt(length);

      plane.normal[0] *= length;
      plane.normal[1] *= length;
      plane.normal[2] *= length;

      plane.dist = DotProduct(pass->points[j], plane.normal);

      //
      // find out which side of the generated seperating plane has the
      // source portal
      //
#if 1
      fliptest = false;
      for (k = 0; k < source->numpoints; k++) {
        if (k == i || k == l)
          continue;
        d = DotProduct(source->points[k], plane.normal) - plane.dist;
        if (d <
            -ON_VIS_EPSILON) { // source is on the negative side, so we want all
          // pass and target on the positive side
          fliptest = false;
          break;
        } else if (d > ON_VIS_EPSILON) { // source is on the positive side, so
                                         // we want all
          // pass and target on the negative side
          fliptest = true;
          break;
        }
      }
      if (k == source->numpoints)
        continue; // planar with source portal
#else
      fliptest = flipclip;
#endif
      //
      // flip the normal if the source portal is backwards
      //
      if (fliptest) {
        VectorSubtract(vec3_origin, plane.normal, plane.normal);
        plane.dist = -plane.dist;
      }
#if 1
      //
      // if all of the pass portal points are now on the positive side,
      // this is the seperating plane
      //
      counts[0] = counts[1] = counts[2] = 0;
      for (k = 0; k < pass->numpoints; k++) {
        if (k == j)
          continue;
        d = DotProduct(pass->points[k], plane.normal) - plane.dist;
        if (d < -ON_VIS_EPSILON)
          break;
        else if (d > ON_VIS_EPSILON)
          counts[0]++;
        else
          counts[2]++;
      }
      if (k != pass->numpoints)
        continue; // points on negative side, not a seperating plane

      if (!counts[0])
        continue; // planar with seperating plane
#else
      k = (j + 1) % pass->numpoints;
      d = DotProduct(pass->points[k], plane.normal) - plane.dist;
      if (d < -ON_VIS_EPSILON)
        continue;
      k = (j + pass->numpoints - 1) % pass->numpoints;
      d = DotProduct(pass->points[k], plane.normal) - plane.dist;
      if (d < -ON_VIS_EPSILON)
        continue;
#endif
      //
      // flip the normal if we want the back side
      //
      if (flipclip) {
        VectorSubtract(vec3_origin, plane.normal, plane.normal);
        plane.dist = -plane.dist;
      }

      //
      // clip target by the seperating plane
      //
      target = ChopWinding(target, stack, &plane);
      if (!target)
        return NULL; // target is not visible

      // JAY: End the loop, no need to find additional separators on this edge ?
      //			j = pass->numpoints;
    }
  }

  return target;
}

class CPortalTrace {
public:
  CUtlVector<Vector> m_list;
  CThreadFastMutex m_mutex;
} g_PortalTrace;

void vvis_WindingCenter(winding_t *w, Vector &center) {
  int i;
  float scale;

  VectorCopy(vec3_origin, center);
  for (i = 0; i < w->numpoints; i++)
    VectorAdd(w->points[i], center, center);

  scale = 1.0 / w->numpoints;
  VectorScale(center, scale, center);
}

Vector ClusterCenter(int cluster) {
  Vector mins, maxs;
  ClearBounds(mins, maxs);
  int count = leafs[cluster].portals.Count();
  for (int i = 0; i < count; i++) {
    winding_t *w = leafs[cluster].portals[i]->winding;
    for (int j = 0; j < w->numpoints; j++) {
      AddPointToBounds(w->points[j], mins, maxs);
    }
  }
  return (mins + maxs) * 0.5f;
}

void DumpPortalTrace(pstack_t *pStack) {
  AUTO_LOCK(g_PortalTrace.m_mutex);
  if (g_PortalTrace.m_list.Count())
    return;

  Warning("Dumped cluster trace!!!\n");
  Vector mid;
  mid = ClusterCenter(g_TraceClusterStart);
  g_PortalTrace.m_list.AddToTail(mid);
  for (; pStack != NULL; pStack = pStack->next) {
    winding_t *w = pStack->pass ? pStack->pass : pStack->portal->winding;
    vvis_WindingCenter(w, mid);
    g_PortalTrace.m_list.AddToTail(mid);
    for (int i = 0; i < w->numpoints; i++) {
      g_PortalTrace.m_list.AddToTail(w->points[i]);
      g_PortalTrace.m_list.AddToTail(mid);
    }
    for (int i = 0; i < w->numpoints; i++) {
      g_PortalTrace.m_list.AddToTail(w->points[i]);
    }
    g_PortalTrace.m_list.AddToTail(w->points[0]);
    g_PortalTrace.m_list.AddToTail(mid);
  }
  mid = ClusterCenter(g_TraceClusterStop);
  g_PortalTrace.m_list.AddToTail(mid);
}

void WritePortalTrace(const char *source) {
  FILE *linefile;
  char filename[1024];

  if (!g_PortalTrace.m_list.Count()) {
    Warning("No trace generated from %d to %d\n", g_TraceClusterStart,
            g_TraceClusterStop);
    return;
  }

  sprintf(filename, "%s.lin", source);
  linefile = fopen(filename, "w");
  if (!linefile)
    Error("Couldn't open %s\n", filename);

  for (int i = 0; i < g_PortalTrace.m_list.Count(); i++) {
    Vector p = g_PortalTrace.m_list[i];
    fprintf(linefile, "%f %f %f\n", p[0], p[1], p[2]);
  }
  fclose(linefile);
  Warning("Wrote %s!!!\n", filename);
}

/*
==================
RecursiveLeafFlow

Flood fill through the leafs
If src_portal is NULL, this is the originating leaf
==================
*/
void RecursiveLeafFlow(int leafnum, threaddata_t *thread, pstack_t *prevstack) {
  pstack_t stack;
  portal_t *p;
  plane_t backplane;
  leaf_t *leaf;
  int i;
  long *test, *might, *vis, more;
  int pnum;

#ifdef MPI
  // Early-out if we're a VMPI worker that's told to exit. If we don't do this
  // here, then the worker might spin its wheels for a while on an expensive
  // work unit and not be available to the pool. This is pretty common in vis.
  if (g_bVMPIEarlyExit)
    return;
#endif

  if (leafnum == g_TraceClusterStop) {
    DumpPortalTrace(&thread->pstack_head);
    return;
  }
  thread->c_chains++;

  leaf = &leafs[leafnum];

  prevstack->next = &stack;

  stack.next = NULL;
  stack.leaf = leaf;
  stack.portal = NULL;

  might = (long *)stack.mightsee;
  vis = (long *)thread->base->portalvis;

  // check all portals for flowing into other leafs
  for (i = 0; i < leaf->portals.Count(); i++) {

    p = leaf->portals[i];
    pnum = p - portals;

    if (!(prevstack->mightsee[pnum >> 3] & (1 << (pnum & 7)))) {
      continue; // can't possibly see it
    }

    // if the portal can't see anything we haven't allready seen, skip it
    if (p->status == stat_done) {
      test = (long *)p->portalvis;
    } else {
      test = (long *)p->portalflood;
    }

    // SSE2-accelerated mightsee pruning: process 4 longs (128 bits) per
    // iteration. On MSVC x64, long is 32-bit, so __m128i = 4 longs.
    {
      long *prev_ms = (long *)prevstack->mightsee;
      __m128i acc = _mm_setzero_si128();
      int j_simd = 0;
      for (; j_simd + 3 < portallongs; j_simd += 4) {
        __m128i ms = _mm_loadu_si128((__m128i *)&prev_ms[j_simd]);
        __m128i ts = _mm_loadu_si128((__m128i *)&test[j_simd]);
        __m128i vs = _mm_loadu_si128((__m128i *)&vis[j_simd]);
        __m128i m = _mm_and_si128(ms, ts);
        _mm_storeu_si128((__m128i *)&might[j_simd], m);
        acc = _mm_or_si128(acc, _mm_andnot_si128(vs, m));
      }
      // Scalar tail
      more = 0;
      for (; j_simd < portallongs; j_simd++) {
        might[j_simd] = prev_ms[j_simd] & test[j_simd];
        more |= (might[j_simd] & ~vis[j_simd]);
      }
      // Reduce the SSE accumulator (128 bits -> 32 bits)
      if (!more) {
        __m128i hi64 = _mm_srli_si128(acc, 8);
        __m128i or64 = _mm_or_si128(acc, hi64);
        __m128i hi32 = _mm_srli_si128(or64, 4);
        __m128i or32 = _mm_or_si128(or64, hi32);
        more = (long)_mm_cvtsi128_si32(or32);
      }
    }

    if (!more &&
        CheckBit(thread->base->portalvis, pnum)) { // can't see anything new
      continue;
    }

    // get plane of portal, point normal into the neighbor leaf
    stack.portalplane = p->plane;
    VectorSubtract(vec3_origin, p->plane.normal, backplane.normal);
    backplane.dist = -p->plane.dist;

    stack.portal = p;
    stack.next = NULL;
    stack.freewindings[0] = 1;
    stack.freewindings[1] = 1;
    stack.freewindings[2] = 1;

    float d = DotProduct(p->origin, thread->pstack_head.portalplane.normal);
    d -= thread->pstack_head.portalplane.dist;
    if (d < -p->radius) {
      continue;
    } else if (d > p->radius) {
      stack.pass = p->winding;
    } else {
      stack.pass =
          ChopWinding(p->winding, &stack, &thread->pstack_head.portalplane);
      if (!stack.pass)
        continue;
    }

    d = DotProduct(thread->base->origin, p->plane.normal);
    d -= p->plane.dist;
    if (d > thread->base->radius) {
      continue;
    } else if (d < -thread->base->radius) {
      stack.source = prevstack->source;
    } else {
      stack.source = ChopWinding(prevstack->source, &stack, &backplane);
      if (!stack.source)
        continue;
    }

    if (!prevstack->pass) { // the second leaf can only be blocked if coplanar

      // mark the portal as visible
      SetBit(thread->base->portalvis, pnum);

      RecursiveLeafFlow(p->leaf, thread, &stack);
      continue;
    }

    stack.pass = ClipToSeperators(stack.source, prevstack->pass, stack.pass,
                                  false, &stack);
    if (!stack.pass)
      continue;

    stack.pass = ClipToSeperators(prevstack->pass, stack.source, stack.pass,
                                  true, &stack);
    if (!stack.pass)
      continue;

    // mark the portal as visible
    SetBit(thread->base->portalvis, pnum);

    // flow through it for real
    RecursiveLeafFlow(p->leaf, thread, &stack);
  }
}

/*
===============
PortalFlow

generates the portalvis bit vector
===============
*/
void PortalFlow(int iThread, int portalnum) {
  threaddata_t data;
  int i;
  portal_t *p;
  int c_might, c_can;

  p = sorted_portals[portalnum];
  p->status = stat_working;

  c_might = CountBits(p->portalflood, g_numportals * 2);

  memset(&data, 0, sizeof(data));
  data.base = p;

  data.pstack_head.portal = p;
  data.pstack_head.source = p->winding;
  data.pstack_head.portalplane = p->plane;
  for (i = 0; i < portallongs; i++)
    ((long *)data.pstack_head.mightsee)[i] = ((long *)p->portalflood)[i];

  RecursiveLeafFlow(p->leaf, &data, &data.pstack_head);

  p->status = stat_done;

  c_can = CountBits(p->portalvis, g_numportals * 2);

  qprintf("portal:%4i  mightsee:%4i  cansee:%4i (%i chains)\n",
          (int)(p - portals), c_might, c_can, data.c_chains);
}

/*
===============================================================================

RunPortalFlowMT: the stock PortalFlow, with each portal's recursion split
across all threads. The default flow (-stockflow selects the stock one).

The stock flow runs one portal per thread, so a map with a few huge portals
(fightspace3) spends hours with one thread busy on each of them. Here the
recursion of one portal is cut into work items at its second level: one item
per (first portal, second portal) pair, each replaying the two stock steps down
to its branch and then recursing exactly as RecursiveLeafFlow does.

The result is the stock result. Each item follows the stock clip chains with
the stock mightsee, so every bit it sets is one the stock flow sets. The only
thing that depends on timing is the stock skip ("the portal is marked and its
mightsee holds nothing unmarked"), and that skip is sound whoever made the
marks: anything the skipped branch could reach is in its mightsee, so it is
already marked. Marks go into the shared portalvis with an atomic OR; a stale
read of it only costs a branch the skip would have saved, never a bit.

Portals are queued in the stock order (fewest mightsee first), and a portal
becomes stat_done, so its portalvis prunes later portals as in the stock flow,
once all of its items have finished. Which neighbours are done when a portal
runs depends on thread timing here exactly as it does in the stock flow; the
cluster PVS has matched the CPU reference bit for bit on d1_trainstation.

===============================================================================
*/

// The items of one (portal, first portal) pair: one item per portal out of
// the first portal's leaf. Items are numbered, not stored (fightspace3 has
// about 10^8), and found from s_MTGroupStart by binary search.
struct MTGroup_t {
  int portal;           // index into portals[]
  unsigned short first; // index into the source leaf's portals
  bool noSecond;        // the first portal's leaf has no portals out
};

static CUtlVector<MTGroup_t> s_MTGroups;
static CUtlVector<int> s_MTGroupStart; // first item number of each group
static int s_nMTItems;
static volatile long *s_pMTItemsLeft = NULL; // per portal, items not finished

static inline void SetBitAtomic(byte *bits, int n) {
  _InterlockedOr8((volatile char *)&bits[n >> 3], (char)(1 << (n & 7)));
}

// A portal's status is the one field the threads publish to each other: it
// becomes stat_done only after the last mark in its portalvis, and a reader
// that sees stat_done then reads that portalvis. The store is a full barrier
// (interlocked exchange). The load is an acquire: x86/x64 loads already are
// one in hardware, and the compiler barrier keeps the portalvis reads that
// follow from being moved above it.
#if !defined(_M_X64) && !defined(_M_IX86)
#error RunPortalFlowMT needs an acquire load of portal_t::status for this CPU
#endif
static_assert(sizeof(vstatus_t) == sizeof(long),
              "portal_t::status is exchanged as a long");

static inline void PublishPortalDone(portal_t *p) {
  _InterlockedExchange((volatile long *)&p->status, (long)stat_done);
}

static inline bool PortalIsDone(const portal_t *p) {
  int status = __iso_volatile_load32((const volatile int *)&p->status);
  _ReadWriteBarrier();
  return status == stat_done;
}

// The body of the stock RecursiveLeafFlow loop for one portal p of leaf: fills
// *stack and returns true when the flow continues through p (p is marked), or
// false when the branch is skipped or clipped away. The only change from the
// stock body is the atomic mark.
static bool MTFlowEnter(portal_t *p, leaf_t *leaf, threaddata_t *thread,
                        pstack_t *prevstack, pstack_t *stack) {
  plane_t backplane;
  long *test, *might, *vis, more;
  int pnum = p - portals;

  if (!CheckBit(prevstack->mightsee, pnum))
    return false; // can't possibly see it

  prevstack->next = stack;
  stack->next = NULL;
  stack->leaf = leaf;
  stack->portal = NULL;

  might = (long *)stack->mightsee;
  vis = (long *)thread->base->portalvis;

  // if the portal can't see anything we haven't allready seen, skip it
  if (PortalIsDone(p)) {
    test = (long *)p->portalvis;
  } else {
    test = (long *)p->portalflood;
  }

  {
    long *prev_ms = (long *)prevstack->mightsee;
    __m128i acc = _mm_setzero_si128();
    int j_simd = 0;
    for (; j_simd + 3 < portallongs; j_simd += 4) {
      __m128i ms = _mm_loadu_si128((__m128i *)&prev_ms[j_simd]);
      __m128i ts = _mm_loadu_si128((__m128i *)&test[j_simd]);
      __m128i vs = _mm_loadu_si128((__m128i *)&vis[j_simd]);
      __m128i m = _mm_and_si128(ms, ts);
      _mm_storeu_si128((__m128i *)&might[j_simd], m);
      acc = _mm_or_si128(acc, _mm_andnot_si128(vs, m));
    }
    more = 0;
    for (; j_simd < portallongs; j_simd++) {
      might[j_simd] = prev_ms[j_simd] & test[j_simd];
      more |= (might[j_simd] & ~vis[j_simd]);
    }
    if (!more) {
      __m128i hi64 = _mm_srli_si128(acc, 8);
      __m128i or64 = _mm_or_si128(acc, hi64);
      __m128i hi32 = _mm_srli_si128(or64, 4);
      __m128i or32 = _mm_or_si128(or64, hi32);
      more = (long)_mm_cvtsi128_si32(or32);
    }
  }

  if (!more &&
      CheckBit(thread->base->portalvis, pnum)) // can't see anything new
    return false;

  // get plane of portal, point normal into the neighbor leaf
  stack->portalplane = p->plane;
  VectorSubtract(vec3_origin, p->plane.normal, backplane.normal);
  backplane.dist = -p->plane.dist;

  stack->portal = p;
  stack->next = NULL;
  stack->freewindings[0] = 1;
  stack->freewindings[1] = 1;
  stack->freewindings[2] = 1;

  float d = DotProduct(p->origin, thread->pstack_head.portalplane.normal);
  d -= thread->pstack_head.portalplane.dist;
  if (d < -p->radius) {
    return false;
  } else if (d > p->radius) {
    stack->pass = p->winding;
  } else {
    stack->pass =
        ChopWinding(p->winding, stack, &thread->pstack_head.portalplane);
    if (!stack->pass)
      return false;
  }

  d = DotProduct(thread->base->origin, p->plane.normal);
  d -= p->plane.dist;
  if (d > thread->base->radius) {
    return false;
  } else if (d < -thread->base->radius) {
    stack->source = prevstack->source;
  } else {
    stack->source = ChopWinding(prevstack->source, stack, &backplane);
    if (!stack->source)
      return false;
  }

  if (!prevstack->pass) { // the second leaf can only be blocked if coplanar
    SetBitAtomic(thread->base->portalvis, pnum);
    return true;
  }

  stack->pass = ClipToSeperators(stack->source, prevstack->pass, stack->pass,
                                 false, stack);
  if (!stack->pass)
    return false;

  stack->pass = ClipToSeperators(prevstack->pass, stack->source, stack->pass,
                                 true, stack);
  if (!stack->pass)
    return false;

  SetBitAtomic(thread->base->portalvis, pnum);
  return true;
}

// RecursiveLeafFlow, with the loop body in MTFlowEnter. It leaves out the
// stock g_bVMPIEarlyExit return and the -trace cluster dump on purpose: both
// are unreachable here, because CalcPortalVis sends an MPI run to
// RunMPIPortalFlow and a -trace run takes CalcVisTrace, so neither gets to the
// default path that calls RunPortalFlowMT.
static void MTLeafFlow(int leafnum, threaddata_t *thread, pstack_t *prevstack) {
  pstack_t stack;
  leaf_t *leaf = &leafs[leafnum];

  thread->c_chains++;
  for (int i = 0; i < leaf->portals.Count(); i++) {
    portal_t *p = leaf->portals[i];
    if (MTFlowEnter(p, leaf, thread, prevstack, &stack))
      MTLeafFlow(p->leaf, thread, &stack);
  }
}

static void MTFlowItem(int iThread, int itemnum) {
  // Last group starting at or before itemnum.
  int lo = 0, hi = s_MTGroupStart.Count() - 1;
  while (lo < hi) {
    int mid = lo + (hi - lo + 1) / 2;
    if (s_MTGroupStart[mid] <= itemnum)
      lo = mid;
    else
      hi = mid - 1;
  }
  const MTGroup_t &group = s_MTGroups[lo];
  int secondIndex = itemnum - s_MTGroupStart[lo];
  portal_t *p = &portals[group.portal];
  threaddata_t data;

  // The stock PortalFlow setup, without clearing the 8 KB mightsee tail that
  // nothing reads (only portallongs words are used).
  data.base = p;
  data.c_chains = 0;
  data.pstack_head.next = NULL;
  data.pstack_head.leaf = NULL;
  data.pstack_head.portal = p;
  data.pstack_head.source = p->winding;
  data.pstack_head.pass = NULL;
  data.pstack_head.portalplane = p->plane;
  for (int i = 0; i < portallongs; i++)
    ((long *)data.pstack_head.mightsee)[i] = ((long *)p->portalflood)[i];

  // Replay the first two levels of the stock recursion down to this item's
  // branch, then recurse from there as the stock flow would.
  pstack_t first, second;
  leaf_t *leaf0 = &leafs[p->leaf];
  portal_t *p1 = leaf0->portals[group.first];
  if (MTFlowEnter(p1, leaf0, &data, &data.pstack_head, &first) &&
      !group.noSecond) {
    leaf_t *leaf1 = &leafs[p1->leaf];
    portal_t *p2 = leaf1->portals[secondIndex];
    if (MTFlowEnter(p2, leaf1, &data, &first, &second))
      MTLeafFlow(p2->leaf, &data, &second);
  }

  if (_InterlockedDecrement(&s_pMTItemsLeft[group.portal]) == 0)
    PublishPortalDone(p);
}

// Queues portal p's items, one per (first, second) portal pair.
static void AddMTItems(portal_t *p) {
  int pnum = p - portals;
  leaf_t *leaf0 = &leafs[p->leaf];
  for (int a = 0; a < leaf0->portals.Count(); a++) {
    portal_t *p1 = leaf0->portals[a];
    if (!CheckBit(p->portalflood, p1 - portals))
      continue; // MTFlowEnter would skip it at once
    // A leaf with no portals out still needs its entry portal marked.
    int n1 = leafs[p1->leaf].portals.Count();
    int count = MAX(n1, 1);
    if ((long long)s_nMTItems + count > INT_MAX)
      Error("RunPortalFlowMT: more than %d work items\n", INT_MAX);
    // MTGroup_t::first is 16 bits; stop instead of truncating the index.
    if (a > USHRT_MAX)
      Error("RunPortalFlowMT: leaf %d has %d portals, more than the %d the "
            "work items can index\n",
            p->leaf, leaf0->portals.Count(), USHRT_MAX + 1);
    MTGroup_t group;
    group.portal = pnum;
    group.first = (unsigned short)a;
    group.noSecond = (n1 == 0);
    s_MTGroups.AddToTail(group);
    s_MTGroupStart.AddToTail(s_nMTItems);
    s_nMTItems += count;
    s_pMTItemsLeft[pnum] += count;
  }
}

void RunPortalFlowMT(void) {
  int numPortals = g_numportals * 2;

  s_MTGroups.RemoveAll();
  s_MTGroupStart.RemoveAll();
  s_nMTItems = 0;
  s_pMTItemsLeft = (volatile long *)calloc(numPortals, sizeof(long));

  for (int si = 0; si < numPortals; si++)
    AddMTItems(sorted_portals[si]);
  for (int i = 0; i < numPortals; i++) {
    if (!s_pMTItemsLeft[i])
      portals[i].status = stat_done; // nothing in front of it to flow into
  }

  RunThreadsOnIndividual(s_nMTItems, true, MTFlowItem);

  s_MTGroups.Purge();
  s_MTGroupStart.Purge();
  free((void *)s_pMTItemsLeft);
  s_pMTItemsLeft = NULL;
}

/*
===============================================================================

This is a rough first-order aproximation that is used to trivially reject some
of the final calculations.


Calculates portalfront and portalflood bit vectors

===============================================================================
*/

int c_flood, c_vis;

/*
==================
SimpleFlood

==================
*/
void SimpleFlood(portal_t *srcportal, int leafnum) {
  int i;
  leaf_t *leaf;
  portal_t *p;
  int pnum;

  leaf = &leafs[leafnum];

  for (i = 0; i < leaf->portals.Count(); i++) {
    p = leaf->portals[i];
    pnum = p - portals;
    if (!CheckBit(srcportal->portalfront, pnum))
      continue;

    if (CheckBit(srcportal->portalflood, pnum))
      continue;

    SetBit(srcportal->portalflood, pnum);

    SimpleFlood(srcportal, p->leaf);
  }
}

/*
==============
BasePortalVis
==============
*/
void BasePortalVis(int iThread, int portalnum) {
  int j, k;
  portal_t *tp, *p;
  float d;
  winding_t *w;
  Vector segment;
  double dist2, minDist2;

  // get the portal
  p = portals + portalnum;

  //
  // allocate memory for bitwise vis solutions for this portal
  //
  p->portalfront = (byte *)malloc(portalbytes);
  memset(p->portalfront, 0, portalbytes);

  p->portalflood = (byte *)malloc(portalbytes);
  memset(p->portalflood, 0, portalbytes);

  p->portalvis = (byte *)malloc(portalbytes);
  memset(p->portalvis, 0, portalbytes);

  //
  // test the given portal against all of the portals in the map
  //
  for (j = 0, tp = portals; j < g_numportals * 2; j++, tp++) {
    // don't test against itself
    if (j == portalnum)
      continue;

    //
    //
    //
    w = tp->winding;
    for (k = 0; k < w->numpoints; k++) {
      d = DotProduct(w->points[k], p->plane.normal) - p->plane.dist;
      if (d > ON_VIS_EPSILON)
        break;
    }
    if (k == w->numpoints)
      continue; // no points on front

    //
    //
    //
    w = p->winding;
    for (k = 0; k < w->numpoints; k++) {
      d = DotProduct(w->points[k], tp->plane.normal) - tp->plane.dist;
      if (d < -ON_VIS_EPSILON)
        break;
    }
    if (k == w->numpoints)
      continue; // no points on front

    //
    // if using radius visibility -- check to see if any portal points lie
    // inside of the radius given
    //
    if (g_bUseRadius) {
      w = tp->winding;
      minDist2 = 1024000000.0; // 32000^2
      for (k = 0; k < w->numpoints; k++) {
        VectorSubtract(w->points[k], p->origin, segment);
        dist2 = (segment[0] * segment[0]) + (segment[1] * segment[1]) +
                (segment[2] * segment[2]);
        if (dist2 < minDist2) {
          minDist2 = dist2;
        }
      }

      if (minDist2 > g_VisRadius)
        continue;
    }

    // add current portal to given portal's list of visible portals
    SetBit(p->portalfront, j);
  }

  SimpleFlood(p, p->leaf);

  p->nummightsee = CountBits(p->portalflood, g_numportals * 2);
  //	Msg ("portal %i: %i mightsee\n", portalnum, p->nummightsee);
  c_flood += p->nummightsee;
}

/*
===============================================================================

This is a second order aproximation

Calculates portalvis bit vector

WAAAAAAY too slow.

===============================================================================
*/

/*
==================
RecursiveLeafBitFlow

==================
*/
void RecursiveLeafBitFlow(int leafnum, byte *mightsee, byte *cansee) {
  portal_t *p;
  leaf_t *leaf;
  int i, j;
  long more;
  int pnum;
  byte newmight[MAX_PORTALS / 8];

  leaf = &leafs[leafnum];

  // check all portals for flowing into other leafs
  for (i = 0; i < leaf->portals.Count(); i++) {
    p = leaf->portals[i];
    pnum = p - portals;

    // if some previous portal can't see it, skip
    if (!CheckBit(mightsee, pnum))
      continue;

    // if this portal can see some portals we mightsee, recurse
    more = 0;
    for (j = 0; j < portallongs; j++) {
      ((long *)newmight)[j] =
          ((long *)mightsee)[j] & ((long *)p->portalflood)[j];
      more |= ((long *)newmight)[j] & ~((long *)cansee)[j];
    }

    if (!more)
      continue; // can't see anything new

    SetBit(cansee, pnum);

    RecursiveLeafBitFlow(p->leaf, newmight, cansee);
  }
}

/*
==============
BetterPortalVis
==============
*/
void BetterPortalVis(int portalnum) {
  portal_t *p;

  p = portals + portalnum;

  RecursiveLeafBitFlow(p->leaf, p->portalflood, p->portalvis);

  // build leaf vis information
  p->nummightsee = CountBits(p->portalvis, g_numportals * 2);
  c_vis += p->nummightsee;
}
