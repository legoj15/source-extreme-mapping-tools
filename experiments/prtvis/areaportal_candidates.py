#!/usr/bin/env python3
"""List the places in a compiled map where an areaportal could go (issue #97).

VBSP places no areaportals of its own. This script reads the portal file VBSP
hands to VVIS and looks for small openings (doorways, corridor mouths, window
openings) whose removal splits the map into two large regions. Nothing is
compiled or changed; the output is a ranked list for a person to judge by eye,
and optionally a VMF of thin tool brushes to open in Hammer beside the map.

Usage:
  python -I experiments/prtvis/areaportal_candidates.py <map.prt> [--vmf <map.vmf>]
        [--min-side 8] [--max-area 16384] [--top 40] [--no-pairs]
        [--bsp <map.bsp>] [--min-volume 0] [--no-collapse]
        [--json out.json] [--vmf-out candidates.vmf]

Options:
  --vmf       the map source. Used only to count the mapper's own func_areaportal
              and func_areaportalwindow entities. When there is at least one, the
              script says a generator would stand down on this map (the mapper's
              own areaportals win); the candidates are still listed.
  --min-side  each of the two regions an opening separates must hold at least this
              many clusters (default 8)
  --max-area  a pair of openings is only considered when each opening is under this
              many square units (default 16384, a 128 x 128 double door)
  --top       how many chokepoints (or, with --no-collapse, candidates) to print,
              write to the JSON and draw in the VMF (default 40)
  --no-pairs  skip the second list (the pair search)
  --bsp       the compiled map, for volume weights. Every cluster is weighted with
              the summed bounding-box volume of its open leaves in the BSP's world
              tree (sliver_stats' BSP reader is reused by import, the way the
              scripts in this folder import each other). With it the sides are
              reported in clusters and as shares of the map's total open volume,
              the score uses the smaller side's volume share instead of its cluster
              count, and --min-volume applies. The BSP's cluster count must match
              the portal file's.
  --min-volume  drop candidates (and pairs) whose smaller side holds less than this
              share of the map's volume (default 0: off; needs --bsp)
  --no-collapse  list every candidate separately, as before. By default the nested
              cuts of one chokepoint are folded together first (step 5 below).
  --json      the same data as the tables, machine readable: clusters, portals,
              openings, the budget figures, "candidates" (the printed top N, with
              the portal indices of each opening and, folded away, the "nested"
              members with their portal indices and sides) and "pairs"
  --vmf-out   a VMF with one thin brush (1 unit thick, centred on the portal's
              plane, every side tools/toolsareaportal) per portal of each listed
              candidate and pair, one Hammer group and one visgroup per listed
              chokepoint (its best-ranked member's portals). Look at it, do not
              compile it: these are world brushes, not func_areaportal entities.
Exit code 0 on success (also when there are no candidates), 2 on a bad or unreadable
input or an unwritable output, with a one-line message.

What it does
  1. Reads the PRT1 file: clusters are the nodes of a graph, every portal is an edge
     (parallel portals stay separate edges). Portals joining a cluster to itself are
     ignored.
  2. Merges portals into openings. The BSP splits one doorway into several portals,
     so portals that lie in one plane (same normal and distance within the tolerances
     below, either facing) and share a stretch of edge are one opening. An opening's
     area is the sum of its portals' areas; its centre, normal and box come from all
     of its portals.
  3. Single openings. For each opening all of its portals are removed from the graph.
     It is a candidate when the clusters it joined then fall into pieces of which the
     smallest holds at least --min-side clusters and all the others together hold at
     least --min-side. side_a is the smallest piece, side_b is the rest. The exact
     definition is used: all of the opening's portals leave the graph together, so an
     opening that is a doorway plus a touching extra portal is judged as the whole.
     An opening that joins clusters of several disconnected parts of the map is
     judged on the largest part it touches.
     Method, two stages. First a cheap exact test for "separates nothing": every
     portal gets a 128-bit label (a cycle-space hash: a spanning forest is grown, the
     portals outside it get random labels, a forest portal gets the XOR of the labels
     of the portals whose loop runs through it). Some subset of an opening's portals
     is a whole cut exactly when some subset of its labels XORs to zero, which is a
     linear-dependence test over GF(2) done by Gaussian elimination, O(portals x 128)
     in all (a wrong verdict needs a 2^-128 coincidence per subset). An opening whose
     labels are independent cannot separate anything and is dropped without a search.
     Second, every other opening gets one bidirectional breadth-first search on the
     graph without its portals: one search per cluster it touches, advanced one node
     at a time in turn, two searches that meet merged into one. The search ends as
     soon as every search has met (nothing is separated), or as soon as a search runs
     dry with fewer than --min-side clusters (a side too small to count), and
     otherwise when all but one search have run dry, the last piece's size being the
     component size minus the pieces found. The work of one search follows the smaller
     side of the cut. Each finished piece also records which clusters it holds (the
     nodes its search visited; the last piece's set is the complement, worked out
     only when that piece is the smallest) — the collapse of step 5 needs the cluster
     set of the smaller side, not only its size, and getting it adds no work worth
     counting to the search. Worst case O(openings x (clusters + portals)); a
     4,000-cluster chain, where every opening is a cut, takes about 20 s, and graphs
     with loops or many small side rooms are much cheaper.
  4. Pairs ("a room with two doors"). The labels of step 3 also find these: an opening
     that does not separate alone, whose own labels do not XOR to zero (zero means the
     opening is a whole boundary by itself) and that is under --max-area is eligible, and two
     eligible openings whose labels XOR to the same value are together the whole
     boundary of a region (the XOR of the two is zero), so they are the pair
     candidates. Each is then judged by the same search as a single opening, with both
     openings removed. Bounded so the run time stays within a couple of minutes: at
     most 64 openings per label class (a ring of doors makes every pair a cut), at
     most 20,000 pairs searched, and a budget of 10,000,000 search steps for the
     whole pair phase (the pairs with the smallest total area are searched first; when
     the budget runs out the list says so). Only exact two-opening boundaries are
     found: two openings that are a cut only together with a third are not looked for.
  5. Chokepoints. One doorway often comes out several times: the compiler cuts it by
     parallel planes a few units apart, and a long tunnel crossed by the block grid
     every 1,024 units gives a candidate per plane — a chain of nested cuts whose
     smaller sides differ by a cluster or two. Candidates are therefore folded, in
     rank order: a candidate whose smaller side is nested in (or contains) the
     smaller side of a chokepoint already kept, and differs from it by at most
     max(NEST_MIN_CLUSTERS, NEST_FRACTION x the smaller of the two sides) clusters —
     and, when --bsp gives volume weights, by at most NEST_FRACTION of the smaller
     side's volume as well — is the same chokepoint and is folded into it; everything
     else starts a chokepoint of its own. Sides that are not nested never fold, so
     two doorways that cut off different rooms stay two rows. The table, the JSON and
     the VMF show one row per chokepoint, its best-ranked member, with the number of
     candidates folded into it; --no-collapse lists every candidate as before. Pairs
     are folded by the same rule on the pair's smaller side, which gathers the
     combinations of the parallel cuts of the same two doors into one row. Two cuts
     either side of a passage's middle have disjoint smaller sides; they fold too when
     little lies between them. Folding compares each candidate with the members of
     the groups before it, counts first and sets only when the counts allow it: small
     on a real map, quadratic in the candidates on a map that is one long chain.
  6. Ranking: score = smaller side x 8192 / max(area, 256), where the smaller side
     is its cluster count, or with --bsp its share of the map's total open volume.
     A 64 x 128 doorway scores its smaller side's cluster count (with --bsp: a
     doorway that cuts off the whole map, share 1, scores 1); every doubling of the
     opening's area halves the score. Pairs use the sum of the two areas.
  7. Budget facts: the BSP format allows 256 areas and 1,024 areaportals. The script
     prints how many candidates there are in all, how many chokepoints they fold
     into, and how many areas the top N would create (components of the cluster
     graph with those N openings removed; area numbering and the solid area 0 are
     not modelled).

What it does not decide: whether the engine accepts an areaportal placed at a
candidate (it needs a brush that fills the opening and touches the world on all
sides, and the opening must really split two areas in the engine's own flood), what
an areaportal there would cost or save in game (draw calls, PVS, the extra area
overhead), or whether the place suits the mapper's design (an opening found here
may be open air or a window, where a func_areaportal tied to a door would not apply).
A candidate is a place to look at in Hammer, not a decision. Distances and areas are
in map units; coordinates are the map's own (Hammer's).

The brushes in --vmf-out are the portal polygons themselves, extruded half a unit
each way along their plane, so they follow the BSP's splits: one doorway shows as
several brushes in one group.
"""
import argparse
import colorsys
import json
import math
import os
import random
import re
import struct
import sys
import time
from bisect import bisect_right
from collections import deque

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)  # python -I drops it
from sliver_stats import (CONTENTS_SOLID, LUMP_LEAFS, LUMP_MODELS, LUMP_NODES,  # noqa: E402
                          LUMP_PLANES, box_cell, read_bsp, split_cell)

POINT = re.compile(r"\(([^)]*)\)")
VMF_CLASSNAME = re.compile(r'"classname"\s+"([^"]*)"')

# ---------------------------------------------------------------------------
# tolerances and limits

NORMAL_DOT_MIN = 0.9999     # two windings are parallel when |n1 . n2| is at least this (under 1 degree)
PLANE_TOL = 0.25            # units: every vertex of one winding lies this close to the other's plane
EDGE_TOL = 0.25             # units: two edges are collinear when both ends of one are this close to the other's line
EDGE_OVERLAP_MIN = 1.0      # units: shared edge length that counts as touching (a shared corner does not)
AABB_TOL = 0.5              # units: slack on the box test that picks the pairs worth the exact tests (>= the two above)
MIN_AREA = 1e-3             # square units: a smaller winding has no usable plane and stays a one-portal opening
SCORE_AREA_REF = 8192.0     # square units: a 64 x 128 doorway; its score is its smaller side's cluster count
SCORE_AREA_FLOOR = 256.0    # square units: a 16 x 16 hole; smaller openings do not score higher
MICRO_EDGE = 0.01           # units: consecutive winding points closer than this are one point in the VMF brushes
BRUSH_THICKNESS = 1.0       # units: the VMF brushes are this thick, centred on the portal's plane
MAX_AREAS = 256             # BSP format: dareas_t count limit
MAX_AREAPORTALS = 1024      # BSP format: dareaportals_t count limit
HASH_SEED = 97              # the portal labels come from a seeded generator, so a run is repeatable
LABEL_BITS = 128            # label width: a wrong "dependent" verdict needs about 2^(portals in an opening - 128) luck
PAIR_CLASS_MAX = 64         # openings per hash class that are paired up (a ring of doors makes every pair a cut)
PAIR_MAX_CHECKS = 20000     # pairs searched at most
PAIR_STEP_BUDGET = 10000000 # search steps (node expansions) for the whole pair phase: under a minute in Python
NEST_MIN_CLUSTERS = 4       # clusters: nested smaller sides this far apart in count are still one chokepoint
NEST_FRACTION = 0.25        # of the smaller side: sides may differ by this fraction (clusters, and volume with --bsp) and still fold


class InputError(Exception):
    """A bad or unreadable input or output; the message is the one line printed."""


class StepBudget(Exception):
    """The pair phase used its search-step budget."""


# ---------------------------------------------------------------------------
# vector helpers (plain tuples)


def vsub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vdot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vcross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def vlen(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


# ---------------------------------------------------------------------------
# input


def load_prt(path):
    """(nclusters, portals) with portals a list of (c0, c1, pts); the loader shape of prt_stats.py."""
    try:
        with open(path, "r") as f:
            lines = f.read().splitlines()
    except OSError as e:
        raise InputError(f"{path}: cannot read ({e.strerror or e})")
    except UnicodeDecodeError:
        raise InputError(f"{path}: not a text portal file")
    if len(lines) < 3 or lines[0].strip() != "PRT1":
        raise InputError(f"{path}: not a PRT1 portal file")
    try:
        nclusters = int(lines[1])
        nportals = int(lines[2])
    except ValueError:
        raise InputError(f"{path}: the cluster and portal counts on lines 2 and 3 are not numbers")
    portals = []
    for lineno, line in enumerate(lines[3:], 4):
        line = line.strip()
        if not line:
            continue
        head, _, rest = line.partition("(")
        try:
            n, c0, c1 = (int(x) for x in head.split())
            pts = [tuple(float(v) for v in m.split()) for m in POINT.findall("(" + rest)]
        except ValueError:
            raise InputError(f"{path}:{lineno}: malformed portal line")
        if any(len(p) != 3 for p in pts):
            raise InputError(f"{path}:{lineno}: a point does not have three coordinates")
        if len(pts) != n or n < 3:
            raise InputError(f"{path}:{lineno}: portal has {len(pts)} points, header says {n} "
                             f"(at least 3 expected)")
        if not (0 <= c0 < nclusters and 0 <= c1 < nclusters):
            raise InputError(f"{path}:{lineno}: cluster number outside 0..{nclusters - 1}")
        portals.append((c0, c1, pts))
    if len(portals) != nportals:
        raise InputError(f"{path}: header says {nportals} portals, the file holds {len(portals)}")
    return nclusters, portals


def count_vmf_areaportals(path):
    """(func_areaportal count, func_areaportalwindow count) in a VMF.

    Entities Hammer keeps inside a `hidden { }` block are left out: VBSP skips them, so
    they are not areaportals of the compiled map.
    """
    try:
        with open(path, "r", errors="replace") as f:
            text = f.read()
    except OSError as e:
        raise InputError(f"{path}: cannot read ({e.strerror or e})")
    counts = {"func_areaportal": 0, "func_areaportalwindow": 0}
    stack, pending = [], None      # open block names; the name waiting for its "{"
    for raw in text.splitlines():
        s = raw.strip()
        if s == "{":
            stack.append(pending)
            pending = None
        elif s == "}":
            if stack:
                stack.pop()
        elif s.startswith('"'):
            m = VMF_CLASSNAME.match(s)
            if m and stack and stack[-1] == "entity" and "hidden" not in stack:
                c = m.group(1).lower()
                if c in counts:
                    counts[c] += 1
        elif s:
            pending = s
    return counts["func_areaportal"], counts["func_areaportalwindow"]


def bsp_cluster_volumes(path, nclusters):
    """A volume weight per cluster: the summed bounding-box volume of the cluster's open leaves.

    The leaves are the convex cells of the BSP's world tree, rebuilt in double by cutting
    the world box with the node planes (sliver_stats' reader and splitter, imported the way
    the scripts in this folder import each other). The bounding-box volume rather than the
    exact cell volume: it is a weight for "how much space is behind this opening", and it
    stays finite for the sliver leaves a real map is full of."""
    try:
        _, lump = read_bsp(path)
        raw_planes, _ = lump(LUMP_PLANES)
        raw_nodes, _ = lump(LUMP_NODES)
        raw_leafs, ver = lump(LUMP_LEAFS)
        raw, _ = lump(LUMP_MODELS)
    except OSError as e:
        raise InputError(f"{path}: cannot read ({e.strerror or e})")
    except SystemExit as e:      # the reader exits by itself on a bad file or a compressed lump
        msg = str(e)
        raise InputError(msg if msg.startswith(f"{path}: ") else f"{path}: {msg}")
    except struct.error as e:    # a file too short to hold its own header or lump table
        raise InputError(f"{path}: not a complete BSP ({e})")
    planes = [struct.unpack_from("<4f", raw_planes, i * 20) for i in range(len(raw_planes) // 20)]
    nodes = [struct.unpack_from("<iii", raw_nodes, i * 32) for i in range(len(raw_nodes) // 32)]
    size = 56 if ver == 0 else 32
    leafs = [struct.unpack_from("<ih", raw_leafs, i * size) for i in range(len(raw_leafs) // size)]
    if len(raw) < 40:
        raise InputError(f"{path}: the model lump is too short for the world model's head node")
    headnode = struct.unpack_from("<i", raw, 36)[0]
    weight = [0.0] * nclusters
    seen = set()
    walked = set()     # nodes visited: a corrupt file can loop
    stack = [(headnode, box_cell())]
    while stack:
        num, cell = stack.pop()
        if num < 0:
            i = -1 - num
            if i >= len(leafs):
                raise InputError(f"{path}: the world tree names leaf {i} of {len(leafs)}")
            contents, cluster = leafs[i]
            if cluster >= nclusters:
                raise InputError(f"{path}: leaf {i} is in cluster {cluster}, but the portal file "
                                 f"has {nclusters}")
            if cluster >= 0:
                seen.add(cluster)
                if not contents & CONTENTS_SOLID:
                    pts = [p for poly in cell for p in poly]
                    if pts:
                        ext = [max(p[k] for p in pts) - min(p[k] for p in pts) for k in range(3)]
                        weight[cluster] += ext[0] * ext[1] * ext[2]
            continue
        if num >= len(nodes):
            raise InputError(f"{path}: the world tree names node {num} of {len(nodes)}")
        if num in walked:
            raise InputError(f"{path}: the world tree reaches node {num} twice (not a tree)")
        walked.add(num)
        planenum, front_child, back_child = nodes[num]
        if not 0 <= planenum < len(planes):
            raise InputError(f"{path}: node {num} names plane {planenum} of {len(planes)}")
        nx, ny, nz, dist = planes[planenum]
        if not all(math.isfinite(v) for v in (nx, ny, nz, dist)):
            raise InputError(f"{path}: plane {planenum} is not finite")
        front, back = split_cell(cell, (nx, ny, nz), dist)
        stack.append((front_child, front))
        stack.append((back_child, back))
    if len(seen) != nclusters:
        raise InputError(f"{path}: the BSP's world tree holds {len(seen)} clusters, "
                         f"but the portal file has {nclusters}")
    return weight


# ---------------------------------------------------------------------------
# portal geometry


def winding_geometry(pts):
    """(area, unit normal, centroid) of a convex winding; the normal follows the point order.

    A winding with no usable plane gets area 0 and the zero normal."""
    p0 = pts[0]
    ax = ay = az = 0.0
    tris = []
    for i in range(1, len(pts) - 1):
        c = vcross(vsub(pts[i], p0), vsub(pts[i + 1], p0))
        ax += c[0]
        ay += c[1]
        az += c[2]
        tris.append((c, pts[i], pts[i + 1]))
    twice = math.sqrt(ax * ax + ay * ay + az * az)
    if 0.5 * twice < MIN_AREA:
        n = len(pts)
        return 0.0, (0.0, 0.0, 0.0), tuple(sum(p[k] for p in pts) / n for k in range(3))
    nrm = (ax / twice, ay / twice, az / twice)
    wsum = cx = cy = cz = 0.0
    for c, p1, p2 in tris:
        w = vdot(c, nrm)
        wsum += w
        cx += w * (p0[0] + p1[0] + p2[0]) / 3.0
        cy += w * (p0[1] + p1[1] + p2[1]) / 3.0
        cz += w * (p0[2] + p1[2] + p2[2]) / 3.0
    return 0.5 * twice, nrm, (cx / wsum, cy / wsum, cz / wsum)


def coplanar(na, ca, pa, nb, cb, pb):
    """Two windings (normal, centroid, points each) lie in one plane within the tolerances, either facing."""
    if abs(vdot(na, nb)) < NORMAL_DOT_MIN:
        return False
    da = vdot(na, ca)
    for p in pb:
        if abs(vdot(na, p) - da) > PLANE_TOL:
            return False
    db = vdot(nb, cb)
    for p in pa:
        if abs(vdot(nb, p) - db) > PLANE_TOL:
            return False
    return True


def share_edge(pa, pb):
    """Some edge of winding A and some edge of B are collinear and overlap by EDGE_OVERLAP_MIN or more."""
    na, nb = len(pa), len(pb)
    for i in range(na):
        a0 = pa[i]
        e = vsub(pa[(i + 1) % na], a0)
        el = vlen(e)
        if el < EDGE_OVERLAP_MIN:
            continue
        d = (e[0] / el, e[1] / el, e[2] / el)
        for j in range(nb):
            b0 = pb[j]
            b1 = pb[(j + 1) % nb]
            r0 = vsub(b0, a0)
            r1 = vsub(b1, a0)
            if vlen(vcross(r0, d)) > EDGE_TOL or vlen(vcross(r1, d)) > EDGE_TOL:
                continue
            t0 = vdot(r0, d)
            t1 = vdot(r1, d)
            if t0 > t1:
                t0, t1 = t1, t0
            if min(t1, el) - max(t0, 0.0) >= EDGE_OVERLAP_MIN:
                return True
    return False


def build_openings(portals, usable):
    """Group the usable portals into openings. Returns (openings, geometry arrays).

    openings: list of portal-index lists. The pairs worth the exact tests are found
    by a sweep along the axis where the boxes overlap least: sort by the box's low
    end, and for each portal look at the following ones until their low end is
    past its high end, keeping those whose boxes also overlap on the other two axes."""
    n = len(portals)
    area = [0.0] * n
    nrm = [(0.0, 0.0, 0.0)] * n
    ctr = [(0.0, 0.0, 0.0)] * n
    lo = [(0.0, 0.0, 0.0)] * n
    hi = [(0.0, 0.0, 0.0)] * n
    for i, (_, _, pts) in enumerate(portals):
        area[i], nrm[i], ctr[i] = winding_geometry(pts)
        lo[i] = tuple(min(p[k] for p in pts) for k in range(3))
        hi[i] = tuple(max(p[k] for p in pts) for k in range(3))
    parent = list(range(n))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    ids = [i for i in range(n) if usable[i] and area[i] > 0.0]
    if ids:
        # sweep along the axis with the fewest overlapping box extents, so the fewest pairs are looked at
        best = None
        for axis in range(3):
            starts = sorted(lo[i][axis] for i in ids)
            work = sum(bisect_right(starts, hi[i][axis] + AABB_TOL) for i in ids)
            if best is None or work < best[0]:
                best = (work, axis)
        ax = best[1]
        ay, az = [k for k in range(3) if k != ax]
        order = sorted(ids, key=lambda i: lo[i][ax])
        lox = [lo[i][ax] for i in order]
        hix = [hi[i][ax] + AABB_TOL for i in order]
        loy = [lo[i][ay] for i in order]
        hiy = [hi[i][ay] + AABB_TOL for i in order]
        loz = [lo[i][az] for i in order]
        hiz = [hi[i][az] + AABB_TOL for i in order]
        count = len(order)
        for a in range(count):
            limit = hix[a]
            ya0, ya1, za0, za1 = loy[a], hiy[a], loz[a], hiz[a]
            i = order[a]
            for b in range(a + 1, count):
                if lox[b] > limit:
                    break
                if loy[b] > ya1 or hiy[b] < ya0 or loz[b] > za1 or hiz[b] < za0:
                    continue
                j = order[b]
                ri, rj = find(i), find(j)
                if ri == rj:
                    continue
                pi, pj = portals[i][2], portals[j][2]
                if coplanar(nrm[i], ctr[i], pi, nrm[j], ctr[j], pj) and share_edge(pi, pj):
                    parent[rj] = ri
    groups = {}
    for i in range(n):
        if usable[i]:
            groups.setdefault(find(i), []).append(i)
    openings = sorted(groups.values(), key=lambda g: g[0])
    return openings, area, nrm, ctr, lo, hi


def describe_opening(plist, portals, area, nrm, ctr, lo, hi):
    """Area, plane normal, centre and box of one opening."""
    total = sum(area[i] for i in plist)
    ref = max(plist, key=lambda i: area[i])
    nr = nrm[ref]
    sx = sy = sz = 0.0
    cx = cy = cz = 0.0
    for i in plist:
        n = nrm[i]
        sign = -1.0 if vdot(n, nr) < 0.0 else 1.0
        sx += sign * n[0] * area[i]
        sy += sign * n[1] * area[i]
        sz += sign * n[2] * area[i]
        cx += ctr[i][0] * area[i]
        cy += ctr[i][1] * area[i]
        cz += ctr[i][2] * area[i]
    ln = math.sqrt(sx * sx + sy * sy + sz * sz)
    if ln > 0.0 and total > 0.0:
        normal = (sx / ln, sy / ln, sz / ln)
        centre = (cx / total, cy / total, cz / total)
    else:
        normal = (0.0, 0.0, 0.0)
        pts = [p for i in plist for p in portals[i][2]]
        centre = tuple(sum(p[k] for p in pts) / len(pts) for k in range(3))
    big = max(range(3), key=lambda a: abs(normal[a]))
    if normal[big] < 0.0:
        normal = (-normal[0], -normal[1], -normal[2])
    mins = tuple(min(lo[i][k] for i in plist) for k in range(3))
    maxs = tuple(max(hi[i][k] for i in plist) for k in range(3))
    return {"area": total, "normal": normal, "centre": centre, "mins": mins, "maxs": maxs,
            "portals": sorted(plist)}


# ---------------------------------------------------------------------------
# the cluster graph and the search


def components(nclusters, portals, usable):
    """(component id per cluster, size per component id) of the cluster graph."""
    parent = list(range(nclusters))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for i, (c0, c1, _) in enumerate(portals):
        if usable[i]:
            a, b = find(c0), find(c1)
            if a != b:
                parent[b] = a
    comp = [find(c) for c in range(nclusters)]
    size = {}
    for c in comp:
        size[c] = size.get(c, 0) + 1
    return comp, size


def endpoints_of(plist, portals, comp, size):
    """Clusters an opening joins, restricted to the largest component it touches; and that component's size."""
    by_comp = {}
    for i in plist:
        for c in portals[i][:2]:
            by_comp.setdefault(comp[c], set()).add(c)
    best = max(by_comp, key=lambda k: (size[k], -k))
    return sorted(by_comp[best]), size[best]


def separate(adj, op_of, ex0, ex1, endpoints, total, min_side, comp_clusters, used=None, limit=None):
    """The pieces the opening(s) ex0 and ex1 cut the endpoints' component into, as a list of
    (size, cluster set) pairs, smallest first (the caller's side_a is the first piece).

    Returns None when the endpoints stay in one piece, or when the smallest piece or
    all the others together are under min_side. One search per endpoint, advanced a
    node each in turn, merged when they meet (see the module docstring). A finished
    piece's set is the clusters its search visited; the last piece's set is the
    complement in comp_clusters (the component's clusters), worked out only when that
    piece is the smallest, so the search's cost bound stands. `used` is a one-element
    list the steps are added to; with `limit` the call raises StepBudget once used[0]
    passes it."""
    n = len(endpoints)
    parent = list(range(n))
    queue = [deque((e,)) for e in endpoints]
    count = [1] * n
    seen = [[e] for e in endpoints]    # the clusters each piece holds so far
    owner = {e: i for i, e in enumerate(endpoints)}
    ring = deque(range(n))
    alive = n
    finished = []
    steps = 0
    try:
        while alive > 1:
            cur = ring.popleft()
            if parent[cur] != cur:
                continue
            r = cur
            u = queue[r].popleft()
            steps += 1
            if limit is not None and used[0] + steps > limit:
                raise StepBudget()
            for v, pi in adj[u]:
                k = op_of[pi]
                if k == ex0 or k == ex1:
                    continue
                o = owner.get(v)
                if o is None:
                    owner[v] = r
                    queue[r].append(v)
                    count[r] += 1
                    seen[r].append(v)
                    continue
                while parent[o] != o:
                    parent[o] = parent[parent[o]]
                    o = parent[o]
                if o == r:
                    continue
                # two searches met: the one with the longer queue absorbs the other
                big, small = (r, o) if len(queue[r]) >= len(queue[o]) else (o, r)
                parent[small] = big
                queue[big].extend(queue[small])
                queue[small] = None
                count[big] += count[small]
                seen[big].extend(seen[small])
                alive -= 1
                r = big
            if parent[cur] == cur:
                if queue[cur]:
                    ring.append(cur)
                else:
                    if count[cur] < min_side:
                        return None
                    finished.append(cur)
                    alive -= 1
    finally:
        if used is not None:
            used[0] += steps
    if not finished:
        return None
    rest = total - sum(count[r] for r in finished)
    pieces = [(count[r], frozenset(seen[r])) for r in finished]
    if rest > 0:
        if rest < min(p[0] for p in pieces):
            left = set(comp_clusters)
            for _, s in pieces:
                left -= s
            pieces.append((rest, frozenset(left)))
        else:
            pieces.append((rest, None))   # never the smallest, and its set is not looked at
    pieces.sort(key=lambda p: p[0])
    sizes = [p[0] for p in pieces]
    if len(sizes) < 2 or sizes[0] < min_side or sum(sizes[1:]) < min_side:
        return None
    return pieces


def cycle_labels(nclusters, portals, usable, seed):
    """A 128-bit label per portal such that a set of portals is a whole cut (the boundary of a
    set of clusters) exactly when its labels XOR to zero, apart from a 2^-128 chance per set.

    A spanning forest is grown by breadth-first search; every other portal gets a random
    label; a tree portal's label is the XOR of the labels of the other portals whose
    cycle through the tree uses it, which a bottom-up sweep sums."""
    rng = random.Random(seed)
    adj = [[] for _ in range(nclusters)]
    for i, (c0, c1, _) in enumerate(portals):
        if usable[i]:
            adj[c0].append((c1, i))
            adj[c1].append((c0, i))
    seen = [False] * nclusters
    parent_edge = [-1] * nclusters
    parent_node = [-1] * nclusters
    tree = [False] * len(portals)
    order = []
    for root in range(nclusters):
        if seen[root]:
            continue
        seen[root] = True
        dq = deque((root,))
        while dq:
            u = dq.popleft()
            order.append(u)
            for v, pi in adj[u]:
                if not seen[v]:
                    seen[v] = True
                    parent_edge[v] = pi
                    parent_node[v] = u
                    tree[pi] = True
                    dq.append(v)
    labels = [0] * len(portals)
    acc = [0] * nclusters
    for i, (c0, c1, _) in enumerate(portals):
        if usable[i] and not tree[i]:
            r = rng.getrandbits(LABEL_BITS)
            labels[i] = r
            acc[c0] ^= r
            acc[c1] ^= r
    for u in reversed(order):
        pe = parent_edge[u]
        if pe >= 0:
            labels[pe] = acc[u]
            acc[parent_node[u]] ^= acc[u]
    return labels


def has_dependency(vectors):
    """True when some non-empty subset of the integers XORs to zero (linearly dependent over GF(2))."""
    basis = {}
    for v in vectors:
        while v:
            top = v.bit_length()
            b = basis.get(top)
            if b is None:
                basis[top] = v
                break
            v ^= b
        else:
            return True
    return False


def score_of(side, area):
    return side * SCORE_AREA_REF / max(area, SCORE_AREA_FLOOR)


def same_chokepoint(a, b, weight):
    """True when two candidates are one chokepoint: little lies between the two cuts.

    Two ways. Nested: one smaller side contains the other, and what lies between is their
    difference (two doorways that cut off different rooms never nest). Or opposite: the
    two smaller sides are disjoint ends of one passage, as for two cuts either side of a
    tunnel's middle, and what lies between is the larger side of one minus the smaller
    side of the other (two doorways into different rooms off one hall have the whole hall
    between them, so they stay apart). Little: at most max(NEST_MIN_CLUSTERS,
    NEST_FRACTION x the smaller count) clusters and, when volume weights are given, at
    most NEST_FRACTION x the smaller of the two volumes.

    The counts are compared before the sets: on a map that is one long chain every pair of
    candidates gets here."""
    ca, cb = a["set"], b["set"]
    la, lb = len(ca), len(cb)
    tol = max(NEST_MIN_CLUSTERS, NEST_FRACTION * min(la, lb))
    if abs(la - lb) <= tol and (ca <= cb or cb <= ca):
        between_vol = abs(a["vol"] - b["vol"]) if weight is not None else 0.0
    elif a["cid"] == b["cid"] and a["side_b"] - lb <= tol and ca.isdisjoint(cb):
        between_vol = a["volb"] - b["vol"] if weight is not None else 0.0
    else:
        return False
    if weight is not None and between_vol > NEST_FRACTION * min(a["vol"], b["vol"]):
        return False
    return True


def collapse_groups(ranked, weight):
    """Fold the nested cuts of one chokepoint together, in rank order. Returns a list of
    groups whose first member is the best-ranked candidate, the one the table shows.

    A candidate joins a group when it is the same chokepoint as the group's HEAD, its
    best-ranked member, so a group reaches no further than the tolerance from the row the
    table shows. Matching any member instead would chain: every cross-section of a long
    hall is a cut one cluster from the next, and a hall with a room at each end became one
    row that listed neither doorway. The price is that a long uniform passage comes out as
    a few rows, one about every quarter change in what it separates, not as one."""
    groups = []
    for cand in ranked:
        for g in groups:
            if same_chokepoint(cand, g[0], weight):
                g.append(cand)
                break
        else:
            groups.append([cand])
    return groups


# ---------------------------------------------------------------------------
# output


def unneg(x, nd):
    """x rounded to nd places, with a negative zero turned into zero."""
    return round(x, nd) + 0.0


def fmt3(v, nd=1):
    return "(" + ", ".join(f"{unneg(x, nd):.{nd}f}" for x in v) + ")"


def fmtn(v):
    return "(" + " ".join(f"{unneg(x, 2):+.2f}" for x in v) + ")"


def box_size(o):
    return tuple(o["maxs"][k] - o["mins"][k] for k in range(3))


def fmt_score(v, total_vol):
    # with volume weights the score is a share times 8192 over an area in the millions:
    # two decimals would round every row to 0.00
    return f"{v:>8.3g}" if total_vol is not None else f"{v:>8.2f}"


def print_candidates(cands, total, chokepoints, collapsed, total_vol):
    both = f"{total} in all, {chokepoints} chokepoints" if collapsed else f"{total} in all"
    print(f"\nSingle openings that separate two regions: {both}, top {len(cands)} listed")
    if not cands:
        return
    head = (f"{'rank':>4} {'score':>8} {'area':>10} {'prts':>4}  {'normal':<20} {'centre':<24} "
            f"{'size (x, y, z)':<26} ")
    if total_vol is None and not collapsed:
        head += "clusters a / b"       # the last column: unpadded, as before
    else:
        head += f"{'clusters a / b':<15}"
        if total_vol is not None:
            head += f" {'vol a / b':<15}"
            if not collapsed:
                head = head.rstrip()
        if collapsed:
            head += f" {'folded':>6}"
    print(head)
    for c in cands:
        o = c["opening"]
        sides = f"{c['side_a']} / {c['side_b']}"
        line = (f"{c['rank']:>4} {fmt_score(c['score'], total_vol)} {o['area']:>10.1f} {len(o['portals']):>4}  "
                f"{fmtn(o['normal']):<20} {fmt3(o['centre']):<24} {fmt3(box_size(o)):<26} ")
        if total_vol is None and not collapsed:
            print(line + sides)          # the last column: no padding, as before
            continue
        line += f"{sides:<15}"
        if total_vol is not None:
            line += f" {100.0 * c['vol'] / total_vol:.1f}% / {100.0 * c['volb'] / total_vol:.1f}%".ljust(15)
            if not collapsed:
                print(line.rstrip())
                continue
        line += f" {len(c['group']) - 1:>6}"
        print(line)


def print_pairs(pairs, total, chokepoints, max_area, note, collapsed, total_vol=None):
    # A bounded search found some of them, not all: never print its count as the total.
    count = f"at least {total} (search bounded, see below)" if note else f"{total} in all"
    if collapsed:
        count += f", {'at least ' if note else ''}{chokepoints} chokepoints"
    print(f"\nPairs of openings that separate two regions only together (each under "
          f"{max_area:g} sq units): {count}, top {len(pairs)} listed")
    if note:
        print(f"  not a complete list: {note}")
    if not pairs:
        return
    head = f"{'rank':>4} {'score':>8} {'area':>10}  {'clusters a / b':<16}"
    if total_vol is not None:
        head += f" {'vol a / b':<15}"
    if collapsed:
        head += f" {'folded':>6}  "
    head += "opening 1 (area, portals, centre); opening 2"
    print(head)
    for p in pairs:
        parts = [f"{o['area']:.1f}, {len(o['portals'])}, {fmt3(o['centre'])}" for o in p["openings"]]
        gap = f" {len(p['group']) - 1:>6}  " if collapsed else " "
        vol = (f" {100.0 * p['vol'] / total_vol:.1f}% / {100.0 * p['volb'] / total_vol:.1f}%".ljust(15)
               if total_vol is not None else "")
        print(f"{p['rank']:>4} {fmt_score(p['score'], total_vol)} {p['area']:>10.1f}  "
              f"{str(p['side_a']) + ' / ' + str(p['side_b']):<16}{vol}{gap}{parts[0]};  {parts[1]}")


def opening_json(o):
    return {"area": round(o["area"], 3), "portals": o["portals"],
            "normal": [round(x, 4) for x in o["normal"]],
            "centre": [round(x, 3) for x in o["centre"]],
            "mins": [round(x, 3) for x in o["mins"]], "maxs": [round(x, 3) for x in o["maxs"]]}


# ---------------------------------------------------------------------------
# the VMF of candidate brushes


def num(v):
    s = f"{v:.4f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def pstr(p):
    return f"({num(p[0])} {num(p[1])} {num(p[2])})"


def largest_triangle(poly):
    """Indices i < j < k (so the winding sense is kept) of three points that span a large triangle.

    Not the largest one, which costs a cubic search on a long winding: the point farthest
    from the first, then the point farthest from the line through those two. That is one
    pass each, and it is never three points on a line unless the whole winding is.
    """
    n = len(poly)
    j = max(range(1, n), key=lambda q: vlen(vsub(poly[q], poly[0])))
    edge = vsub(poly[j], poly[0])
    k = max((q for q in range(1, n) if q != j),
            key=lambda q: vlen(vcross(edge, vsub(poly[q], poly[0]))))
    return tuple(sorted((0, j, k)))


def side_lines(poly, sid):
    """One VMF side for a polygon given counter-clockwise seen from outside.

    Hammer takes the outward normal as (p0 - p1) x (p2 - p1) of the three plane
    points, so the points go in clockwise order seen from outside."""
    i, j, k = largest_triangle(poly)
    p0, p1, p2 = poly[k], poly[j], poly[i]
    return [
        "\t\tside", "\t\t{",
        f'\t\t\t"id" "{sid}"',
        f'\t\t\t"plane" "{pstr(p0)} {pstr(p1)} {pstr(p2)}"',
        '\t\t\t"material" "tools/toolsareaportal"',
        '\t\t\t"uaxis" "[1 0 0 0] 0.25"',
        '\t\t\t"vaxis" "[0 -1 0 0] 0.25"',
        '\t\t\t"rotation" "0"',
        '\t\t\t"lightmapscale" "16"',
        '\t\t\t"smoothing_groups" "0"',
        "\t\t}",
    ]


def portal_brush(pts, normal, solid_id, next_id, editor):
    """Lines of one solid: the winding extruded half a thickness each way along its normal.

    The winding is counter-clockwise seen from the side its normal points to, so the
    top face keeps the order, the bottom face reverses it, and the side face over edge
    i is (B_i, B_i+1, T_i+1, T_i). Returns (lines, next free id) or None when the
    winding has fewer than three distinct points."""
    ded = []
    for p in pts:
        if not ded or vlen(vsub(p, ded[-1])) >= MICRO_EDGE:
            ded.append(p)
    while len(ded) > 1 and vlen(vsub(ded[0], ded[-1])) < MICRO_EDGE:
        ded.pop()
    if len(ded) < 3:
        return None
    h = 0.5 * BRUSH_THICKNESS
    top = [(p[0] + h * normal[0], p[1] + h * normal[1], p[2] + h * normal[2]) for p in ded]
    bot = [(p[0] - h * normal[0], p[1] - h * normal[1], p[2] - h * normal[2]) for p in ded]
    faces = [top, bot[::-1]]
    n = len(ded)
    for i in range(n):
        j = (i + 1) % n
        faces.append([bot[i], bot[j], top[j], top[i]])
    lines = ["\tsolid", "\t{", f'\t\t"id" "{solid_id}"']
    for poly in faces:
        lines += side_lines(poly, next_id)
        next_id += 1
    lines += ["\t\teditor", "\t\t{"] + [f'\t\t\t{e}' for e in editor] + ["\t\t}", "\t}"]
    return lines, next_id


def write_vmf(path, labelled, portals, nrm, area):
    """labelled: list of (label, [portal indices]); one group and one visgroup each."""
    nid = 2
    group_ids = []
    for _ in labelled:
        group_ids.append(nid)
        nid += 1
    out = ["versioninfo", "{", '\t"editorversion" "400"', '\t"editorbuild" "8870"',
           '\t"mapversion" "1"', '\t"formatversion" "100"', '\t"prefab" "0"', "}",
           "visgroups", "{"]
    colors = []
    for v, (label, _) in enumerate(labelled):
        r, g, b = colorsys.hsv_to_rgb((v * 0.61803398875) % 1.0, 0.75, 0.95)
        colors.append(f"{int(r * 255)} {int(g * 255)} {int(b * 255)}")
        out += ["\tvisgroup", "\t{", f'\t\t"name" "areaportal {label}"', f'\t\t"visgroupid" "{v + 1}"',
                f'\t\t"color" "{colors[v]}"', "\t}"]
    out += ["}", "world", "{", '\t"id" "1"', '\t"mapversion" "1"', '\t"classname" "worldspawn"']
    group_blocks = []
    for v, (label, plist) in enumerate(labelled):
        editor = [f'"color" "{colors[v]}"', f'"groupid" "{group_ids[v]}"', f'"visgroupid" "{v + 1}"',
                  '"visgroupshown" "1"', '"visgroupautoshown" "1"']
        for i in plist:
            if area[i] <= 0.0:
                continue
            made = portal_brush(portals[i][2], nrm[i], nid, nid + 1, editor)
            if made is None:
                continue
            lines, nid = made
            out += lines
        group_blocks.append(["\tgroup", "\t{", f'\t\t"id" "{group_ids[v]}"', "\t\teditor", "\t\t{",
                             f'\t\t\t"color" "{colors[v]}"', f'\t\t\t"visgroupid" "{v + 1}"',
                             '\t\t\t"visgroupshown" "1"', '\t\t\t"visgroupautoshown" "1"', "\t\t}", "\t}"])
    for gb in group_blocks:
        out += gb
    out += ["}"]
    try:
        with open(path, "w") as f:
            f.write("\n".join(out) + "\n")
    except OSError as e:
        raise InputError(f"{path}: cannot write ({e.strerror or e})")


# ---------------------------------------------------------------------------


def positive_int(text):
    v = int(text)
    if v < 1:
        raise argparse.ArgumentTypeError("must be at least 1")
    return v


def non_negative_int(text):
    v = int(text)
    if v < 0:
        raise argparse.ArgumentTypeError("must be 0 or more")
    return v


def positive_float(text):
    v = float(text)
    if not v > 0.0:
        raise argparse.ArgumentTypeError("must be above 0")
    return v


def unit_float(text):
    v = float(text)
    if not 0.0 <= v <= 1.0:
        raise argparse.ArgumentTypeError("must be a share between 0 and 1")
    return v


def run(args):
    t_start = time.time()
    nclusters, portals = load_prt(args.prt)
    vmf_counts = count_vmf_areaportals(args.vmf) if args.vmf else None
    if args.min_volume and not args.bsp:
        raise InputError("--min-volume needs --bsp: without a BSP the sides are counted in clusters")

    weight = None
    total_vol = None
    if args.bsp:
        weight = bsp_cluster_volumes(args.bsp, nclusters)
        total_vol = sum(weight)
        if total_vol <= 0.0:
            raise InputError(f"{args.bsp}: the open leaves of the world tree have no volume")

    usable = [c0 != c1 for c0, c1, _ in portals]
    openings, area, nrm, ctr, lo, hi = build_openings(portals, usable)
    op_of = [-1] * len(portals)
    for k, plist in enumerate(openings):
        for i in plist:
            op_of[i] = k
    adj = [[] for _ in range(nclusters)]
    for i, (c0, c1, _) in enumerate(portals):
        if usable[i]:
            adj[c0].append((c1, i))
            adj[c1].append((c0, i))
    comp, comp_size = components(nclusters, portals, usable)
    comp_clusters = {}
    for c, cid in enumerate(comp):
        comp_clusters.setdefault(cid, []).append(c)
    comp_clusters = {cid: frozenset(cs) for cid, cs in comp_clusters.items()}
    comp_vol = {cid: sum(weight[c] for c in cs) for cid, cs in comp_clusters.items()} if weight else None
    in_graph = [bool(adj[c]) for c in range(nclusters)]
    areas_now = len({comp[c] for c in range(nclusters) if in_graph[c]})

    print(f"{args.prt}")
    print(f"clusters {nclusters}  portals {len(portals)}  openings {len(openings)}  "
          f"portal-graph components {areas_now}")
    if weight is not None:
        print(f"BSP {args.bsp}: the clusters hold {total_vol:,.0f} cubic units of open leaf boxes")
        if len(comp_vol) > 1:      # other components, clusters with no portal among them
            print("  volume shares are of the whole map: a row's two shares fall short of 100% by "
                  "what the clusters outside its portal-graph component hold")

    def measure(c):
        # what the score calls "the smaller side": the cluster count of the side with fewer
        # clusters, or with --bsp the share of the map's open volume of the side with less
        # volume (not always the same side)
        return min(c["vol"], c["volb"]) / total_vol if weight is not None else c["side_a"]

    # single openings
    t0 = time.time()
    op_area = [sum(area[i] for i in plist) for plist in openings]
    labels = cycle_labels(nclusters, portals, usable, HASH_SEED)
    singles = []        # one dict per separating opening
    separating = set()  # every opening that separates alone, also the ones --min-volume drops:
                        # the pair phase must not pick them up (a pair is "only together")
    under_min_volume = 0
    pairs_under_min_volume = 0
    for k, plist in enumerate(openings):
        ends, total = endpoints_of(plist, portals, comp, comp_size)
        if total < 2 * args.min_side:
            continue
        # no subset of the opening's portals is a whole cut: removing them separates nothing
        if not has_dependency([labels[i] for i in plist]):
            continue
        cid = comp[ends[0]]
        pieces = separate(adj, op_of, k, -2, ends, total, args.min_side, comp_clusters[cid])
        if pieces is None:
            continue
        separating.add(k)
        sa, sa_set = pieces[0]
        cand = {"k": k, "side_a": sa, "side_b": total - sa, "set": sa_set, "cid": cid,
                "vol": None, "volb": None}
        if weight is not None:
            cand["vol"] = sum(weight[c] for c in sa_set)
            cand["volb"] = comp_vol[cid] - cand["vol"]
            if args.min_volume and min(cand["vol"], cand["volb"]) / total_vol < args.min_volume:
                under_min_volume += 1
                continue
        singles.append(cand)
    ranked = sorted(singles, key=lambda s: (-score_of(measure(s), op_area[s["k"]]),
                                            op_area[s["k"]], s["k"]))
    print(f"single-opening search: {time.time() - t0:.1f} s")
    if under_min_volume:
        print(f"  {under_min_volume} more openings separate two regions but are under "
              f"--min-volume {args.min_volume:g} and are left out of every count below")

    groups = collapse_groups(ranked, weight) if not args.no_collapse else [[c] for c in ranked]
    cands = []
    for r, g in enumerate(groups[:args.top], 1):
        best = g[0]
        o = describe_opening(openings[best["k"]], portals, area, nrm, ctr, lo, hi)
        cands.append({"rank": r, "score": score_of(measure(best), o["area"]), "opening": o,
                      "k": best["k"], "side_a": best["side_a"], "side_b": best["side_b"],
                      "vol": best["vol"], "volb": best["volb"], "group": g})
    print_candidates(cands, len(ranked), len(groups), not args.no_collapse, total_vol)

    # pairs
    pairs = []
    pairs_total = 0
    pair_groups = []
    pair_note = ""
    if not args.no_pairs:
        t0 = time.time()
        single_ks = set(separating)
        classes = {}
        for k, plist in enumerate(openings):
            if k in single_ks or op_area[k] >= args.max_area:
                continue
            h = 0
            for i in plist:
                h ^= labels[i]
            if h:
                classes.setdefault(h, []).append(k)
        # Every bound that drops pairs says so: the totals below are then "at least".
        notes = []
        todo = []
        dropped = 0
        for members in classes.values():
            if len(members) < 2:
                continue
            members = sorted(members, key=lambda k: op_area[k])
            if len(members) > PAIR_CLASS_MAX:
                n = len(members)
                dropped += n * (n - 1) // 2 - PAIR_CLASS_MAX * (PAIR_CLASS_MAX - 1) // 2
                members = members[:PAIR_CLASS_MAX]
            for a in range(len(members)):
                for b in range(a + 1, len(members)):
                    todo.append((op_area[members[a]] + op_area[members[b]], members[a], members[b]))
        if dropped:
            notes.append(f"{dropped:,} pairs were not searched: a group of openings with one cut label is "
                         f"paired up only among its {PAIR_CLASS_MAX} smallest")
        todo.sort()
        if len(todo) > PAIR_MAX_CHECKS:
            notes.append(f"{len(todo) - PAIR_MAX_CHECKS:,} of the remaining {len(todo):,} pairs were not "
                         f"searched: only the {PAIR_MAX_CHECKS:,} with the smallest total area were")
            todo = todo[:PAIR_MAX_CHECKS]
        used = [0]
        found = []
        try:
            for tot_area, k1, k2 in todo:
                plist = openings[k1] + openings[k2]
                ends, total = endpoints_of(plist, portals, comp, comp_size)
                if total < 2 * args.min_side:
                    continue
                cid = comp[ends[0]]
                pieces = separate(adj, op_of, k1, k2, ends, total, args.min_side,
                                  comp_clusters[cid], used, PAIR_STEP_BUDGET)
                if pieces is None:
                    continue
                sa, sa_set = pieces[0]
                cand = {"ks": (k1, k2), "side_a": sa, "side_b": total - sa, "set": sa_set,
                        "cid": cid, "area": tot_area, "vol": None, "volb": None}
                if weight is not None:
                    cand["vol"] = sum(weight[c] for c in sa_set)
                    cand["volb"] = comp_vol[cid] - cand["vol"]
                    if args.min_volume and min(cand["vol"], cand["volb"]) / total_vol < args.min_volume:
                        pairs_under_min_volume += 1
                        continue
                found.append(cand)
        except StepBudget:
            notes.append(f"the search stopped at its budget of {PAIR_STEP_BUDGET:,} steps "
                         f"(cheapest pairs first)")
        pair_note = "; ".join(notes)
        found.sort(key=lambda p: (-score_of(measure(p), p["area"]), p["area"], p["ks"][0], p["ks"][1]))
        pairs_total = len(found)
        pair_groups = collapse_groups(found, weight) if not args.no_collapse else [[p] for p in found]
        for r, g in enumerate(pair_groups[:args.top], 1):
            k1, k2 = g[0]["ks"]
            os_ = [describe_opening(openings[k], portals, area, nrm, ctr, lo, hi) for k in (k1, k2)]
            pairs.append({"rank": r, "score": score_of(measure(g[0]), g[0]["area"]),
                          "area": g[0]["area"], "openings": os_, "ks": (k1, k2),
                          "side_a": g[0]["side_a"], "side_b": g[0]["side_b"],
                          "vol": g[0]["vol"], "volb": g[0]["volb"], "group": g})
        print(f"pair search: {time.time() - t0:.1f} s, {used[0]:,} steps")
        print_pairs(pairs, pairs_total, len(pair_groups), args.max_area, pair_note,
                    not args.no_collapse, total_vol)
        if pairs_under_min_volume:
            print(f"  {pairs_under_min_volume} more pairs separate two regions but are under "
                  f"--min-volume {args.min_volume:g} and are left out of the pair counts")

    # budget facts
    # Everything listed above counts: a pair is two areaportals, and a folded candidate is
    # another place for the same areaportal, not an extra one.
    listed = {c["k"] for c in cands} | {k for p in pairs for k in p["ks"]}
    after = components(nclusters, portals, [usable[i] and op_of[i] not in listed for i in range(len(portals))])[0]
    areas_after = len({after[c] for c in range(nclusters) if in_graph[c]})
    collapsed = not args.no_collapse
    print("\nBudget: the BSP format allows "
          f"{MAX_AREAS} areas and {MAX_AREAPORTALS:,} areaportals.")
    print(f"  candidates in all: {len(ranked)} single openings"
          + (f" in {len(groups)} chokepoints" if collapsed else "")
          + ((f", {'at least ' if pair_note else ''}{pairs_total} pairs"
              + (f" in {len(pair_groups)} chokepoints" if collapsed else ""))
             if not args.no_pairs else ""))
    rows = "chokepoints" if collapsed else "candidates"
    print(f"  listed above ({rows}): single openings {len(cands)}, pairs {len(pairs)}; together they would use "
          f"{len(listed)} of {MAX_AREAPORTALS:,} areaportals and cut the "
          f"map into {areas_after} areas (now {areas_now}) of {MAX_AREAS}"
          + ("  -- OVER THE AREA LIMIT" if areas_after > MAX_AREAS else ""))

    # the mapper's own areaportals
    stands_down = False
    if vmf_counts is not None:
        ap, apw = vmf_counts
        stands_down = ap + apw > 0
        print(f"\nThe map source has {ap} func_areaportal and {apw} func_areaportalwindow entities.")
        if stands_down:
            print("A generator would stand down on this map: the mapper's own areaportals win. "
                  "The candidates above are still listed.")

    if args.json:
        data = {"clusters": nclusters, "portals": len(portals), "openings": len(openings),
                "candidates_total": len(ranked), "chokepoints_total": len(groups),
                "collapsed": collapsed, "pairs_total": pairs_total,
                "pairs_chokepoints_total": len(pair_groups),
                "budget": {"area_limit": MAX_AREAS, "areaportal_limit": MAX_AREAPORTALS,
                           "areas_now": areas_now, "areas_after_top": areas_after,
                           "top_count": len(cands)},
                "min_side": args.min_side, "max_area": args.max_area,
                "candidates": [], "pairs": []}
        if weight is not None:
            data["bsp"] = {"path": args.bsp, "total_volume": round(total_vol, 1),
                           "min_volume": args.min_volume}
        if vmf_counts is not None:
            data["vmf"] = {"func_areaportal": vmf_counts[0], "func_areaportalwindow": vmf_counts[1],
                           "generator_stands_down": stands_down}
        if pair_note:
            data["pairs_note"] = pair_note
        for c in cands:
            e = opening_json(c["opening"])
            entry = {"rank": c["rank"], "score": float(f"{c['score']:.6g}"), "area": e["area"],
                     "portals": e["portals"], "normal": e["normal"], "centre": e["centre"],
                     "mins": e["mins"], "maxs": e["maxs"],
                     "side_a": c["side_a"], "side_b": c["side_b"],
                     "nested": [{"portals": sorted(openings[m["k"]]), "side_a": m["side_a"],
                                 "side_b": m["side_b"]} for m in c["group"][1:]]}
            if weight is not None:
                entry["side_a_volume"] = round(c["vol"], 1)
                entry["side_b_volume"] = round(c["volb"], 1)
                entry["side_a_share"] = round(c["vol"] / total_vol, 6)
                entry["side_b_share"] = round(c["volb"] / total_vol, 6)
            data["candidates"].append(entry)
        for p in pairs:
            entry = {"rank": p["rank"], "score": float(f"{p['score']:.6g}"), "area": round(p["area"], 3),
                     "openings": [opening_json(o) for o in p["openings"]],
                     "side_a": p["side_a"], "side_b": p["side_b"],
                     "nested": [{"portals": sorted(openings[m["ks"][0]] + openings[m["ks"][1]]),
                                 "side_a": m["side_a"], "side_b": m["side_b"]} for m in p["group"][1:]]}
            if weight is not None:
                entry["side_a_volume"] = round(p["vol"], 1)
                entry["side_b_volume"] = round(p["volb"], 1)
                entry["side_a_share"] = round(p["vol"] / total_vol, 6)
                entry["side_b_share"] = round(p["volb"] / total_vol, 6)
            data["pairs"].append(entry)
        try:
            with open(args.json, "w") as f:
                json.dump(data, f, indent=1)
        except OSError as e:
            raise InputError(f"{args.json}: cannot write ({e.strerror or e})")

    if args.vmf_out:
        labelled = [(f"candidate {c['rank']}", c["opening"]["portals"]) for c in cands]
        labelled += [(f"pair {p['rank']}", sorted(p["openings"][0]["portals"] + p["openings"][1]["portals"]))
                     for p in pairs]
        write_vmf(args.vmf_out, labelled, portals, nrm, area)
        print(f"\nwrote {len(labelled)} candidate groups to {args.vmf_out}")
    print(f"total {time.time() - t_start:.1f} s")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("prt", help="the map's PRT1 portal file")
    ap.add_argument("--vmf", help="the map source, to count the mapper's own areaportal entities")
    ap.add_argument("--min-side", type=positive_int, default=8,
                    help="clusters each of the two separated regions must hold (default 8)")
    ap.add_argument("--max-area", type=positive_float, default=16384.0,
                    help="largest opening, in square units, that a pair may use (default 16384)")
    ap.add_argument("--top", type=non_negative_int, default=40,
                    help="chokepoints to list (candidates, with --no-collapse; default 40)")
    ap.add_argument("--no-pairs", action="store_true", help="skip the pair search")
    ap.add_argument("--bsp", help="the compiled map, to weight the clusters by open-leaf volume "
                                  "(sides as volume shares, score on the smaller side's share)")
    ap.add_argument("--min-volume", type=unit_float, default=0.0,
                    help="drop candidates whose smaller side holds less than this share of the "
                         "map's volume (default 0: off; needs --bsp)")
    ap.add_argument("--no-collapse", action="store_true",
                    help="list every candidate separately instead of one row per chokepoint")
    ap.add_argument("--json", help="write the same data as JSON")
    ap.add_argument("--vmf-out", help="write the listed candidates as thin tool brushes in a VMF")
    args = ap.parse_args(argv)
    try:
        run(args)
    except InputError as e:
        print(f"areaportal_candidates.py: {e}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
