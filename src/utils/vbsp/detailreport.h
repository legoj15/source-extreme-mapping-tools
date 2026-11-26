//========= -reportdetail: structural brushes that could be func_detail ========
//
// Issue #101. Report only: the pass reads the tree and changes nothing in the
// compile. With the option off none of this runs and the BSP and the .prt are
// byte for byte what they were. See docs/vbsp_detail_report.md for what is
// listed, how a cost is attributed, and why the list is advice and not proof.
//=============================================================================

#ifndef DETAILREPORT_H
#define DETAILREPORT_H

#ifdef _WIN32
#pragma once
#endif

#include "tier1/utlvector.h"

enum { DETAILREPORT_DEFAULT_ROWS = 40 };

extern bool g_bReportDetail;      // -reportdetail
extern int g_nReportDetailRows;   // table rows on the console (the file has all)

// Call on the final world tree after its portals are built and the entity flood
// has run, before FillOutside turns the leaves the flood did not reach into
// solid. Finds the brushes with a solid leaf next to such a leaf (the void).
void DetailReport_FindVoidFacing( tree_t *tree );

// Call from WritePortalFile once the visibility portals are listed: counts, per
// brush, the nodes of the final tree it supplied the plane for and the listed
// portals that lie on those nodes, then prints the table and writes
// <map>.detail.txt.
void DetailReport_Emit( node_t *headnode, const CUtlVector<portal_t *> &visPortals );

#endif // DETAILREPORT_H
