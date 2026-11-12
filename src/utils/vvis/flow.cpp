//========= Copyright Valve Corporation, All rights reserved. ============//
//
// Purpose:
//
// $NoKeywords: $
//
//=============================================================================//
#include "vis.h"
#include <emmintrin.h> // SSE2
#include "threads.h"
#include "tier0/threadtools.h"
#include <intrin.h>
#include <limits.h>
#include <windows.h> // GetCurrentThreadStackLimits

// The project builds with /fp:fast. The flow's geometry is double (vis.h) and
// its side tests must not depend on reassociation or FMA contraction, so the
// floating-point model is pinned for the whole file (it only changes the
// floating-point code; the bit loops are integer).
#pragma float_control(precise, on, push)
#pragma fp_contract(off)

static inline double VisDot(const visvec_t &a, const visvec_t &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline void VisSubtract(const visvec_t &a, const visvec_t &b,
                               visvec_t &out) {
  out.x = a.x - b.x;
  out.y = a.y - b.y;
  out.z = a.z - b.z;
}

static inline void VisNegate(plane_t &plane) {
  plane.normal.x = -plane.normal.x;
  plane.normal.y = -plane.normal.y;
  plane.normal.z = -plane.normal.z;
  plane.dist = -plane.dist;
}

static inline Vector VisToVector(const visvec_t &v) {
  return Vector((float)v.x, (float)v.y, (float)v.z);
}

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
  double dists[128];
  int sides[128];
  int counts[3];
  double dot;
  int i, j;
  visvec_t mid;
  winding_t *neww;

  counts[0] = counts[1] = counts[2] = 0;

  // determine sides for each point
  for (i = 0; i < in->numpoints; i++) {
    dot = VisDot(in->points[i], split->normal);
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
    const visvec_t &p1 = in->points[i];

    if (neww->numpoints == MAX_POINTS_ON_FIXED_WINDING) {
      FreeStackWinding(neww, stack);
      return in; // can't chop -- fall back to original
    }

    if (sides[i] == SIDE_ON) {
      neww->points[neww->numpoints] = p1;
      neww->numpoints++;
      continue;
    }

    if (sides[i] == SIDE_FRONT) {
      neww->points[neww->numpoints] = p1;
      neww->numpoints++;
    }

    if (sides[i + 1] == SIDE_ON || sides[i + 1] == sides[i])
      continue;

    if (neww->numpoints == MAX_POINTS_ON_FIXED_WINDING) {
      FreeStackWinding(neww, stack);
      return in; // can't chop -- fall back to original
    }

    // generate a split point
    const visvec_t &p2 = in->points[(i + 1) % in->numpoints];

    dot = dists[i] / (dists[i] - dists[i + 1]);
    for (j = 0; j < 3; j++) { // avoid round off error when possible
      if (split->normal[j] == 1)
        mid[j] = split->dist;
      else if (split->normal[j] == -1)
        mid[j] = -split->dist;
      else
        mid[j] = p1[j] + dot * (p2[j] - p1[j]);
    }

    neww->points[neww->numpoints] = mid;
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
  visvec_t v1, v2;
  double d;
  double length;
  int counts[3];
  bool fliptest;

  // check all combinations
  for (i = 0; i < source->numpoints; i++) {
    l = (i + 1) % source->numpoints;
    VisSubtract(source->points[l], source->points[i], v1);

    // fing a vertex of pass that makes a plane that puts all of the
    // vertexes of pass on the front side and all of the vertexes of
    // source on the back side
    for (j = 0; j < pass->numpoints; j++) {
      VisSubtract(pass->points[j], source->points[i], v2);

      plane.normal[0] = v1[1] * v2[2] - v1[2] * v2[1];
      plane.normal[1] = v1[2] * v2[0] - v1[0] * v2[2];
      plane.normal[2] = v1[0] * v2[1] - v1[1] * v2[0];

      // if points don't make a valid plane, skip it

      length = plane.normal[0] * plane.normal[0] +
               plane.normal[1] * plane.normal[1] +
               plane.normal[2] * plane.normal[2];

      if (length < ON_VIS_EPSILON)
        continue;

      length = 1.0 / sqrt(length);

      plane.normal[0] *= length;
      plane.normal[1] *= length;
      plane.normal[2] *= length;

      plane.dist = VisDot(pass->points[j], plane.normal);

      //
      // find out which side of the generated seperating plane has the
      // source portal
      //
#if 1
      fliptest = false;
      for (k = 0; k < source->numpoints; k++) {
        if (k == i || k == l)
          continue;
        d = VisDot(source->points[k], plane.normal) - plane.dist;
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
      if (fliptest)
        VisNegate(plane);
#if 1
      //
      // if all of the pass portal points are now on the positive side,
      // this is the seperating plane
      //
      counts[0] = counts[1] = counts[2] = 0;
      for (k = 0; k < pass->numpoints; k++) {
        if (k == j)
          continue;
        d = VisDot(pass->points[k], plane.normal) - plane.dist;
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
      d = VisDot(pass->points[k], plane.normal) - plane.dist;
      if (d < -ON_VIS_EPSILON)
        continue;
      k = (j + pass->numpoints - 1) % pass->numpoints;
      d = VisDot(pass->points[k], plane.normal) - plane.dist;
      if (d < -ON_VIS_EPSILON)
        continue;
#endif
      //
      // flip the normal if we want the back side
      //
      if (flipclip)
        VisNegate(plane);

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

// The -trace output (.lin) is float text, as it always was: the centres and
// points are worked out in double and rounded to Vector for the list.
void vvis_WindingCenter(winding_t *w, Vector &center) {
  visvec_t sum = {0, 0, 0};
  for (int i = 0; i < w->numpoints; i++) {
    sum.x += w->points[i].x;
    sum.y += w->points[i].y;
    sum.z += w->points[i].z;
  }
  double scale = 1.0 / w->numpoints;
  sum.x *= scale;
  sum.y *= scale;
  sum.z *= scale;
  center = VisToVector(sum);
}

Vector ClusterCenter(int cluster) {
  Vector mins, maxs;
  ClearBounds(mins, maxs);
  int count = leafs[cluster].portals.Count();
  for (int i = 0; i < count; i++) {
    winding_t *w = leafs[cluster].portals[i]->winding;
    for (int j = 0; j < w->numpoints; j++) {
      AddPointToBounds(VisToVector(w->points[j]), mins, maxs);
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
      g_PortalTrace.m_list.AddToTail(VisToVector(w->points[i]));
      g_PortalTrace.m_list.AddToTail(mid);
    }
    for (int i = 0; i < w->numpoints; i++) {
      g_PortalTrace.m_list.AddToTail(VisToVector(w->points[i]));
    }
    g_PortalTrace.m_list.AddToTail(VisToVector(w->points[0]));
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
===============================================================================

RunPortalFlowMT: the stock PortalFlow, with each portal's recursion split
across all threads. It is the only copy of the portal-flow recursion: the
default flow and -trace both run it.

The stock flow ran one portal per thread, so a map with a few huge portals
(fightspace3) spent hours with one thread busy on each of them. Here the
recursion of one portal is cut into work items at its second level: one item
per (first portal, second portal) pair, each replaying the two stock steps down
to its branch and then recursing exactly as the stock RecursiveLeafFlow did.

The result is this flow's own one-thread result, bit for bit, at any thread
count. It is no longer Valve's float VVIS bit for bit: the rules and the order
are stock, but the geometry is double (vis.h, issue #89), so a side test at the
end of a long chain no longer decides on float rounding the windows carried
down it. Two reads in the stock flow see what other portals write:

- A neighbour's portalvis prunes the flow once the neighbour is finished; until
  then its portalflood is used. On one thread that means portals earlier in
  sorted_portals prune with their portalvis and later ones with their
  portalflood. The threaded stock flow takes whatever happens to be finished,
  so its PVS changes from run to run, gaining bits and losing others
  (ddd_market). Here the order decides, never the timing: an earlier neighbour
  that is still running is waited for (WaitPortalDone).
- The stock skip ("the portal is marked and its mightsee holds nothing
  unmarked") reads the base portal's own portalvis while its other items are
  marking it. That skip is sound whoever made the marks: anything the skipped
  branch could reach is in its mightsee, so it is already marked. Marks go in
  with an atomic OR; a stale read only costs a branch the skip would have
  saved, never a bit.

Portals are queued in the order of sorted_portals (the stock order: fewest
mightsee first unless -nosort; for -trace, the start cluster's portals in leaf
order), and a portal becomes stat_done once all of its items have finished.

Waiting would make this slower than the stock flow if threads only spun: one
item can hold most of a portal's work, and every later portal that flows
through that portal waits on it. So a waiting thread is handed branches of the
portal it waits on: the running items give away the portals they have not yet
flowed into, shallowest level first (MTHandOff), and the waiter replays and
runs them (WaitPortalDone). Which thread runs a branch never changes what the
branch marks.

===============================================================================
*/

// The items of one (portal, first portal) pair: one item per portal out of
// the first portal's leaf. Items are numbered, not stored (fightspace3 has
// about 10^8), and found from s_MTGroupStart by binary search.
struct MTGroup_t {
  int portal;           // index into portals[]
  unsigned short first; // index into the source leaf's portals
  bool noSecond;        // the first portal's leaf has no portals out
  bool stopOnly;        // -trace: no first portal either; see AddMTItems
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

// Each portal's position in sorted_portals, the stock processing order.
// INT_MAX for a portal that is not flowed (-trace flows one cluster's portals
// only): it never counts as earlier, so its portalflood is used, as the stock
// flow did for a portal it had not run.
static int *s_pMTRank = NULL;

// -trace: set once a chain to the stop cluster has been dumped, so the rest of
// the flow can stop. Never set in a normal run.
static volatile long s_bMTTraceDumped = 0;

// The stock -trace test, made where RecursiveLeafFlow made it: on entering
// leafnum with the chain down to it on thread's stack. True means stop here:
// this chain has just been dumped, or another one already was. In a normal run
// g_TraceClusterStop is -1 and this is never true.
static inline bool MTTraceStop(int leafnum, threaddata_t *thread) {
  if (leafnum == g_TraceClusterStop) {
    DumpPortalTrace(&thread->pstack_head);
    s_bMTTraceDumped = 1;
    return true;
  }
  return s_bMTTraceDumped != 0;
}

// Branches handed off to waiting threads (d1_trainstation without them: one
// item of portal 2966 ran 48 s while dozens of threads waited on it). A branch
// is the chain of portals from its base portal, replayed with MTFlowEnter,
// which gives the same windings and mightsee every time.
static const int MT_MAX_PATH = 512;     // deeper chains are not handed off
static const int MT_MAX_HELP_DEPTH = 4; // nested helps per thread

// The flow's stack. Every level of a DFS (MTLeafFlow) keeps a pstack_t on the
// stack, and a thread that helps (WaitPortalDone) starts another DFS on top of
// the one it is in. The sizes follow from pstack_t, so they keep up if it
// grows:
// - MT_LEVEL_BYTES: one MTLeafFlow level. Its frame measured 9,341 bytes
//   (pstack_t is 9,168 of them); the rest is slack.
// - MT_STACK_MARGIN: kept free below the deepest level, for the calls a level
//   makes (MTFlowEnter, the clips, the start of a help) and for Error.
// - MT_STACK_HELP_MIN: a thread only starts helping with this much stack left,
//   so every DFS, an item's own or a helped one, has room for MT_DFS_LEVELS
//   levels however the helps nest. A thread with less left waits without
//   helping, which is always allowed (see WaitPortalDone).
// - MT_STACK_RESERVE: twice that, so a thread up to MT_DFS_LEVELS deep can
//   still help. Address space only; pages are committed as the recursion
//   reaches them.
// A chain deeper than its thread has room for stops the run with an Error
// (MTLeafFlow) where it used to overflow the stack. Real maps are nowhere
// near: the deepest chains measured for issue #110 were 7 to 48 portals
// (validation, cm_laststand, d1_trainstation_02_hrcs, ddd_market,
// validation_harder), under 1 MB of stack. "vvis -v" prints both numbers.
static const size_t MT_LEVEL_BYTES = sizeof(pstack_t) + 1024;
static const size_t MT_STACK_MARGIN = 256 * 1024;
static const int MT_DFS_LEVELS = 3200;
static const size_t MT_STACK_HELP_MIN =
    MT_DFS_LEVELS * MT_LEVEL_BYTES + MT_STACK_MARGIN;
static const size_t MT_STACK_RESERVE = 2 * MT_STACK_HELP_MIN;

// The calling thread's stack: the lowest address of its reserve and the
// address it started from.
static thread_local const char *s_pMTStackLow = NULL;
static thread_local const char *s_pMTStackHigh = NULL;

// Bytes of stack left below pHere, an address in the caller's frame.
static inline size_t MTStackLeft(const void *pHere) {
  if (!s_pMTStackLow) {
    ULONG_PTR low, high;
    GetCurrentThreadStackLimits(&low, &high);
    s_pMTStackLow = (const char *)low;
    s_pMTStackHigh = (const char *)high;
  }
  return (size_t)((const char *)pHere - s_pMTStackLow);
}

// What the run used, for the -v report: the longest chain of portals below a
// base portal, and the most stack one thread had in use. A thread writes the
// shared values only when it beats its own record, which is rare.
static volatile long s_nMTDeepestChain = 0;
static volatile long long s_nMTPeakStack = 0;
static thread_local int s_nMTThreadDeepest = 0;
static thread_local long long s_nMTThreadPeak = 0;

// depth: portals between the base portal and the level at pHere. Call after
// MTStackLeft on the same thread.
static inline void MTNoteDepth(int depth, const void *pHere) {
  if (depth > s_nMTThreadDeepest) {
    s_nMTThreadDeepest = depth;
    long seen = s_nMTDeepestChain;
    while (depth > seen) {
      long was = _InterlockedCompareExchange(&s_nMTDeepestChain, depth, seen);
      if (was == seen)
        break;
      seen = was;
    }
  }
  long long used = s_pMTStackHigh - (const char *)pHere;
  if (used > s_nMTThreadPeak) {
    s_nMTThreadPeak = used;
    long long seen = s_nMTPeakStack;
    while (used > seen) {
      long long was =
          _InterlockedCompareExchange64(&s_nMTPeakStack, used, seen);
      if (was == seen)
        break;
      seen = was;
    }
  }
}

static volatile long *s_pMTWaiters = NULL; // per portal, threads waiting on it
static volatile long *s_pMTQueued = NULL;  // per portal, branches queued
static CUtlVector<int> *s_pMTQueue = NULL; // per portal: [portals..., length]...
static CThreadFastMutex s_MTQueueLock;
static thread_local int s_nMTHelpDepth = 0;

static bool MTPopBranch(int pnum, int *path, int *len);
static void MTRunBranch(int pnum, const int *path, int len);
static void MTFinishBranch(int pnum);

// Waits for an earlier portal's items to finish, running its handed-off
// branches meanwhile. This cannot deadlock: items are handed out in item order
// (GetThreadWork) and numbered in stock order, so every item of an earlier
// portal is already running or done; it and its branches only wait on portals
// earlier still, and each item runs whatever of its portal's queue is left
// before it finishes, so a branch no helper takes is still run.
static void WaitPortalDone(const portal_t *p) {
  if (PortalIsDone(p))
    return;
  int pnum = p - portals;
  // A helped branch is a whole DFS on top of this one; it is only started
  // with room for one (MT_STACK_HELP_MIN).
  bool help = s_nMTHelpDepth < MT_MAX_HELP_DEPTH &&
              MTStackLeft(&pnum) >= MT_STACK_HELP_MIN;
  if (help)
    _InterlockedIncrement(&s_pMTWaiters[pnum]);
  int path[MT_MAX_PATH], len;
  for (int spins = 0; !PortalIsDone(p);) {
    if (help && MTPopBranch(pnum, path, &len)) {
      // Still counted as waiting while it runs the branch, so the branch keeps
      // handing its untouched portals off and the queue stays stocked for
      // threads that start waiting later. Counting only idle waiters measured
      // slower (d3_c17_02 15.2 -> 17.3 s, ddd_market 14.5 -> 15.5 s).
      s_nMTHelpDepth++;
      MTRunBranch(pnum, path, len);
      s_nMTHelpDepth--;
      MTFinishBranch(pnum);
      spins = 0;
    } else if (spins++ < 256) {
      ThreadPause();
    } else {
      ThreadSleep(0);
    }
  }
  if (help)
    _InterlockedDecrement(&s_pMTWaiters[pnum]);
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

  // if the portal can't see anything we haven't allready seen, skip it.
  // p's portalvis narrows the flow exactly when the one-thread stock flow has
  // it finished: when p comes before the base portal in the stock order. If p
  // is still running, its portalflood (a superset) stands in until the flow
  // goes on past p. The skip test and the clips below come out the same with
  // either: a branch skipped with the superset is skipped with the subset, and
  // the clips never read the mightsee. So the wait for p comes last, and a
  // branch that is skipped or clipped away never waits.
  bool waitForVis = false;
  if (s_pMTRank[pnum] < s_pMTRank[thread->base - portals] &&
      PortalIsDone(p)) {
    test = (long *)p->portalvis;
  } else {
    test = (long *)p->portalflood;
    waitForVis = s_pMTRank[pnum] < s_pMTRank[thread->base - portals];
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
  backplane = p->plane;
  VisNegate(backplane);

  stack->portal = p;
  stack->next = NULL;
  stack->freewindings[0] = 1;
  stack->freewindings[1] = 1;
  stack->freewindings[2] = 1;

  double d = VisDot(p->origin, thread->pstack_head.portalplane.normal);
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

  d = VisDot(thread->base->origin, p->plane.normal);
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

  // the second leaf can only be blocked if coplanar
  if (prevstack->pass) {
    stack->pass = ClipToSeperators(stack->source, prevstack->pass, stack->pass,
                                   false, stack);
    if (!stack->pass)
      return false;

    stack->pass = ClipToSeperators(prevstack->pass, stack->source, stack->pass,
                                   true, stack);
    if (!stack->pass)
      return false;
  }

  SetBitAtomic(thread->base->portalvis, pnum);

  // The flow goes on past p: give it the mightsee the stock flow would have.
  if (waitForVis) {
    WaitPortalDone(p);
    const long *prev_ms = (const long *)prevstack->mightsee;
    const long *pvis = (const long *)p->portalvis;
    for (int j = 0; j < portallongs; j++)
      might[j] = prev_ms[j] & pvis[j];
  }
  return true;
}

// One DFS from a base portal: the stock thread data, plus where each
// MTLeafFlow level is in its loop, so a hand-off can give away the untouched
// portals of any level (MTHandOff) and not only those of the deepest one.
struct MTFrame_t {
  leaf_t *leaf;
  pstack_t *prevstack;
  int next;  // index of the leaf's next portal to flow into
  int count; // the loop stops here; a hand-off lowers it
};

struct MTRun_t {
  threaddata_t data;  // first, so MTFlowEnter's threaddata_t * is this run
  int firstFrame;     // depth of the first MTLeafFlow level (the branch length)
  MTFrame_t frames[MT_MAX_PATH]; // by depth: portals between the base and it
};

// Queues the untouched portals of the shallowest level that has any, each as a
// branch through the chain down to that level, for the threads waiting on the
// base portal. depth is the deepest active level.
static void MTHandOff(MTRun_t *run, int depth) {
  int base = run->data.base - portals;
  for (int d = run->firstFrame; d <= depth; d++) {
    MTFrame_t &f = run->frames[d];
    int n = 0;
    for (int j = f.next; j < f.count; j++)
      n += CheckBit(f.prevstack->mightsee, f.leaf->portals[j] - portals) ? 1 : 0;
    if (!n) {
      f.count = f.next; // the rest would be skipped at once; stop rescanning
      continue;
    }

    // The chain down to this level: the portal entered at each depth 1..d.
    int chain[MT_MAX_PATH];
    int c = 0;
    for (pstack_t *s = run->data.pstack_head.next; c < d; s = s->next)
      chain[c++] = s->portal - portals;

    // Counted before they can be taken, so the base cannot finish early.
    _InterlockedExchangeAdd(&s_pMTItemsLeft[base], n);
    s_MTQueueLock.Lock();
    CUtlVector<int> &q = s_pMTQueue[base];
    for (int j = f.next; j < f.count; j++) {
      int pnum = f.leaf->portals[j] - portals;
      if (!CheckBit(f.prevstack->mightsee, pnum))
        continue; // MTFlowEnter would skip it at once
      q.AddMultipleToTail(d, chain);
      q.AddToTail(pnum);
      q.AddToTail(d + 1);
    }
    s_MTQueueLock.Unlock();
    _InterlockedExchangeAdd(&s_pMTQueued[base], n);
    f.count = f.next;
    return;
  }
}

// The stock RecursiveLeafFlow, with the loop body in MTFlowEnter. depth counts
// the portals between the base and prevstack.
static void MTLeafFlow(int leafnum, MTRun_t *run, pstack_t *prevstack,
                       int depth) {
  pstack_t stack;
  leaf_t *leaf = &leafs[leafnum];
  threaddata_t *thread = &run->data;
  int base = thread->base - portals;

  if (MTTraceStop(leafnum, thread))
    return;

  // Stop before the next level could run off the stack. The margin is larger
  // than a level, so this level's frame was safe to allocate.
  size_t stackLeft = MTStackLeft(&stack);
  if (stackLeft < MT_STACK_MARGIN)
    Error("VVIS: a chain of %d portals from portal %d is deeper than the "
          "flow's stack holds (%d MB a thread).\n",
          depth, base, (int)(MT_STACK_RESERVE >> 20));
  MTNoteDepth(depth, &stack);

  if (depth >= MT_MAX_PATH - 1) { // too deep to record: no hand-offs below
    for (int i = 0; i < leaf->portals.Count(); i++) {
      portal_t *p = leaf->portals[i];
      if (MTFlowEnter(p, leaf, thread, prevstack, &stack))
        MTLeafFlow(p->leaf, run, &stack, depth + 1);
    }
    return;
  }

  MTFrame_t &f = run->frames[depth];
  f.leaf = leaf;
  f.prevstack = prevstack;
  f.count = leaf->portals.Count();
  for (int i = 0; i < f.count; i++) {
    f.next = i + 1;
    // A thread waits on the base portal (this one too, if it is helping) and
    // the queue is empty. Each hand-off moves untouched portals of this run to
    // the queue, so the work is split, never repeated, and it ends with the
    // run's DFS.
    if (s_pMTWaiters[base] && !s_pMTQueued[base])
      MTHandOff(run, depth);
    portal_t *p = leaf->portals[i];
    if (MTFlowEnter(p, leaf, thread, prevstack, &stack))
      MTLeafFlow(p->leaf, run, &stack, depth + 1);
  }
}

// Enters path[k..len-1] below prevstack, then flows on from the last portal.
// A step that is skipped now loses nothing: the skip is sound whenever it
// holds, and the clips come out as they did for the thread that queued it.
static void MTReplay(MTRun_t *run, pstack_t *prevstack, leaf_t *leaf,
                     const int *path, int k, int len) {
  pstack_t stack;
  portal_t *p = &portals[path[k]];
  if (!MTFlowEnter(p, leaf, &run->data, prevstack, &stack))
    return;
  if (k + 1 < len) {
    // A work item's first portal can lead into the -trace stop cluster.
    if (MTTraceStop(p->leaf, &run->data))
      return;
    MTReplay(run, &stack, &leafs[p->leaf], path, k + 1, len);
  } else {
    MTLeafFlow(p->leaf, run, &stack, len);
  }
}

// Runs the branch path (portal numbers from the base portal pnum's leaf on).
static void MTRunBranch(int pnum, const int *path, int len) {
  portal_t *p = &portals[pnum];
  MTRun_t run;
  threaddata_t &data = run.data;

  // The stock PortalFlow setup, without clearing the 8 KB mightsee tail that
  // nothing reads (only portallongs words are used).
  data.base = p;
  data.pstack_head.next = NULL;
  data.pstack_head.leaf = NULL;
  data.pstack_head.portal = p;
  data.pstack_head.source = p->winding;
  data.pstack_head.pass = NULL;
  data.pstack_head.portalplane = p->plane;
  for (int i = 0; i < portallongs; i++)
    ((long *)data.pstack_head.mightsee)[i] = ((long *)p->portalflood)[i];

  // -trace: the base portal itself can lead into the stop cluster.
  if (MTTraceStop(p->leaf, &data))
    return;

  run.firstFrame = len;
  MTReplay(&run, &data.pstack_head, &leafs[p->leaf], path, 0, len);
}

static bool MTPopBranch(int pnum, int *path, int *len) {
  if (!s_pMTQueued[pnum])
    return false;
  bool got = false;
  s_MTQueueLock.Lock();
  CUtlVector<int> &q = s_pMTQueue[pnum];
  if (q.Count()) {
    int n = q.Tail();
    int start = q.Count() - 1 - n;
    for (int i = 0; i < n; i++)
      path[i] = q[start + i];
    q.RemoveMultipleFromTail(n + 1);
    *len = n;
    got = true;
  }
  s_MTQueueLock.Unlock();
  if (got)
    _InterlockedDecrement(&s_pMTQueued[pnum]);
  return got;
}

static void MTFinishBranch(int pnum) {
  if (_InterlockedDecrement(&s_pMTItemsLeft[pnum]) == 0)
    PublishPortalDone(&portals[pnum]);
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

  if (group.stopOnly) {
    MTRunBranch(group.portal, NULL, 0); // dumps the trace and returns
    MTFinishBranch(group.portal);
    return;
  }

  // The item's branch: the stock recursion's first two steps.
  portal_t *p1 = leafs[p->leaf].portals[group.first];
  int path[2] = {(int)(p1 - portals), 0};
  if (!group.noSecond)
    path[1] = leafs[p1->leaf].portals[secondIndex] - portals;
  MTRunBranch(group.portal, path, group.noSecond ? 1 : 2);

  // Run what is left of the branches this portal handed off; whatever no
  // helper took is run here, so none is lost.
  int len;
  int queued[MT_MAX_PATH];
  while (MTPopBranch(group.portal, queued, &len)) {
    MTRunBranch(group.portal, queued, len);
    MTFinishBranch(group.portal);
  }
  MTFinishBranch(group.portal);
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
    group.stopOnly = false;
    s_MTGroups.AddToTail(group);
    s_MTGroupStart.AddToTail(s_nMTItems);
    s_nMTItems += count;
    s_pMTItemsLeft[pnum] += count;
  }

  // -trace: a portal straight into a stop cluster that has no way on is a
  // whole chain with nothing to flow into. It gets one item, so the dump comes
  // at the portal's turn, as it did in the stock flow.
  if (!s_pMTItemsLeft[pnum] && p->leaf == g_TraceClusterStop) {
    MTGroup_t group;
    group.portal = pnum;
    group.first = 0;
    group.noSecond = true;
    group.stopOnly = true;
    s_MTGroups.AddToTail(group);
    s_MTGroupStart.AddToTail(s_nMTItems);
    if (s_nMTItems == INT_MAX)
      Error("RunPortalFlowMT: more than %d work items\n", INT_MAX);
    s_nMTItems++;
    s_pMTItemsLeft[pnum] = 1;
  }
}

void GetPortalFlowDepth(int *pDeepestChain, long long *pPeakStackBytes,
                        long long *pStackReserveBytes) {
  *pDeepestChain = s_nMTDeepestChain;
  *pPeakStackBytes = s_nMTPeakStack;
  *pStackReserveBytes = (long long)MT_STACK_RESERVE;
}

// Flows the first numSorted portals of sorted_portals, in that order: all of
// them for the PVS, the portals out of the start cluster for -trace.
void RunPortalFlowMT(int numSorted) {
  int numPortals = g_numportals * 2;

  s_MTGroups.RemoveAll();
  s_MTGroupStart.RemoveAll();
  s_nMTItems = 0;
  s_bMTTraceDumped = 0;
  s_nMTDeepestChain = 0;
  s_nMTPeakStack = 0;
  s_pMTItemsLeft = (volatile long *)calloc(numPortals, sizeof(long));
  s_pMTRank = (int *)malloc(numPortals * sizeof(int));
  for (int i = 0; i < numPortals; i++)
    s_pMTRank[i] = INT_MAX;
  for (int si = 0; si < numSorted; si++)
    s_pMTRank[sorted_portals[si] - portals] = si;

  for (int si = 0; si < numSorted; si++)
    AddMTItems(sorted_portals[si]);
  for (int si = 0; si < numSorted; si++) {
    portal_t *p = sorted_portals[si];
    if (!s_pMTItemsLeft[p - portals])
      p->status = stat_done; // nothing in front of it to flow into
  }

  s_pMTWaiters = (volatile long *)calloc(numPortals, sizeof(long));
  s_pMTQueued = (volatile long *)calloc(numPortals, sizeof(long));
  s_pMTQueue = new CUtlVector<int>[numPortals];

  // A thread that helps runs a second DFS on top of its own (WaitPortalDone).
  g_nRunThreadsStackReserve = MT_STACK_RESERVE;
  RunThreadsOnIndividual(s_nMTItems, true, MTFlowItem);
  g_nRunThreadsStackReserve = 0;

  s_MTGroups.Purge();
  s_MTGroupStart.Purge();
  free((void *)s_pMTItemsLeft);
  s_pMTItemsLeft = NULL;
  free(s_pMTRank);
  s_pMTRank = NULL;
  free((void *)s_pMTWaiters);
  s_pMTWaiters = NULL;
  free((void *)s_pMTQueued);
  s_pMTQueued = NULL;
  delete[] s_pMTQueue;
  s_pMTQueue = NULL;
}

/*
===============================================================================

This is a rough first-order aproximation that is used to trivially reject some
of the final calculations.


Calculates portalfront and portalflood bit vectors

===============================================================================
*/

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
  double d;
  winding_t *w;
  visvec_t segment;
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
      d = VisDot(w->points[k], p->plane.normal) - p->plane.dist;
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
      d = VisDot(w->points[k], tp->plane.normal) - tp->plane.dist;
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
        VisSubtract(w->points[k], p->origin, segment);
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
}

#pragma float_control(pop)
