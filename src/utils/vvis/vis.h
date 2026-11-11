//========= Copyright Valve Corporation, All rights reserved. ============//
//
// Purpose:
//
// $NoKeywords: $
//
//=============================================================================//
// vis.h

#include "bsplib.h"
#include "cmdlib.h"
#include "mathlib/mathlib.h"

#define MAX_PORTALS 65536

#define PORTALFILE "PRT1"

extern bool g_bUseRadius;  // prototyping TF2, "radius vis" solution
extern double g_VisRadius; // the radius for the TF2 "radius vis"

// The portal geometry is double from the parsed .prt on: portal windings,
// planes and spheres, the flow's stack windings and its separator planes
// (issue #89). In float, a window carried rounding along a long chain, and a
// side test at the end of the chain could decide on that rounding and drop a
// cluster pair a straight line sees through. The arithmetic on it lives in
// flow.cpp and vvis.cpp, under a pinned floating-point model.
struct visvec_t {
  double x, y, z;
  double &operator[](int i) { return (&x)[i]; }
  const double &operator[](int i) const { return (&x)[i]; }
};

struct plane_t {
  visvec_t normal;
  double dist;
};

#define MAX_POINTS_ON_WINDING 64
#define MAX_POINTS_ON_FIXED_WINDING 12

struct winding_t {
  qboolean original; // don't free, it's part of the portal
  int numpoints;
  visvec_t points[MAX_POINTS_ON_FIXED_WINDING]; // variable sized
};

winding_t *NewWinding(int points);
void FreeWinding(winding_t *w);
winding_t *CopyWinding(winding_t *w);

typedef enum { stat_none, stat_working, stat_done } vstatus_t;
struct portal_t {
  plane_t plane; // normal pointing into neighbor
  int leaf;      // neighbor

  visvec_t origin; // for fast clip testing
  double radius;

  winding_t *winding;
  vstatus_t status;
  byte *portalfront; // [portals], preliminary
  byte *portalflood; // [portals], intermediate
  byte *portalvis;   // [portals], final

  int nummightsee; // bit count on portalflood for sort
};

struct leaf_t {
  CUtlVector<portal_t *> portals;
};

struct pstack_t {
  byte mightsee[MAX_PORTALS / 8]; // bit string
  pstack_t *next;
  leaf_t *leaf;
  portal_t *portal; // portal exiting
  winding_t *source;
  winding_t *pass;

  winding_t windings[3]; // source, pass, temp in any order
  int freewindings[3];

  plane_t portalplane;
};

struct threaddata_t {
  portal_t *base;
  int c_chains;
  pstack_t pstack_head;
};

extern int g_numportals;
extern int portalclusters;

extern portal_t *portals;
extern leaf_t *leafs;

extern int c_portaltest, c_portalpass, c_portalcheck;
extern int c_portalskip, c_leafskip;
extern int c_vistest, c_mighttest;
extern int c_chains;

extern byte *vismap, *vismap_p, *vismap_end; // past visfile

extern int testlevel;

extern byte *uncompressed;

extern int leafbytes, leaflongs;
extern int portalbytes, portallongs;

void LeafFlow(int leafnum);

void BasePortalVis(int iThread, int portalnum);
void BetterPortalVis(int portalnum);
void RunPortalFlowMT(int numSorted);

void WritePortalTrace(const char *source);
void SimpleFlood(portal_t *srcportal, int leafnum);

// One portal as parsed from the .prt, in double, for the merge flow
// (mergeflow.cpp), which builds its own portals from it.
struct mergeportal_t {
  int numpoints;
  int leafnums[2];
  double *points; // numpoints * 3, malloc'd
};
// [g_numportals] in file order. Only filled when -mergeflow is set. Freed (and
// reset to NULL) by FreeMergePortals, which RunMergeFlow calls as soon as it has
// rebuilt its portals and RunVVis calls again once the run's work is done, for
// the -fast and -trace runs that never reach RunMergeFlow.
extern mergeportal_t *g_pMergePortals;
void FreeMergePortals(void);

// Writes every portal's visibility row to portals[n].portalvis and marks it
// stat_done, in place of the exact portal flow. The rows must be allocated
// (portalbytes each) but need not be cleared; CalcVis allocates them in place
// of BasePortalVis, which this flow does not need. The counts are how often the
// merge's safety nets fired: states frozen to the whole portal, and hulls
// replaced by a rectangle.
void RunMergeFlow(int nStates, long long *pGuardFirings,
                  long long *pRectFallbacks);

extern portal_t *sorted_portals[MAX_MAP_PORTALS * 2];
extern int g_TraceClusterStart, g_TraceClusterStop;

int CountBits(byte *bits, int numbits);

#define CheckBit(bitstring, bitNumber)                                         \
  ((bitstring)[((bitNumber) >> 3)] & (1 << ((bitNumber) & 7)))
#define SetBit(bitstring, bitNumber)                                           \
  ((bitstring)[((bitNumber) >> 3)] |= (1 << ((bitNumber) & 7)))
#define ClearBit(bitstring, bitNumber)                                         \
  ((bitstring)[((bitNumber) >> 3)] &= ~(1 << ((bitNumber) & 7)))
