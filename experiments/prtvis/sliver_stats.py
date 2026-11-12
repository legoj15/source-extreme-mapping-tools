#!/usr/bin/env python3
"""Count sliver portals and sliver leaves in a compiled map (issue #104).

Reads only: <map>.prt (what VBSP hands to VVIS) and, when it is there,
<map>.bsp (the leaves). Nothing is compiled or changed.

Usage:
  python -I experiments/prtvis/sliver_stats.py <map.prt> [--bsp <map.bsp>] [--json out.json]
                                               [--list N]

Portals (from the .prt text, in double):
  dup vertex      two consecutive points are exactly the same point
  micro edge      an edge shorter than 0.01 (VVIS's ON_VIS_EPSILON), 0.1, 0.2
                  (0.2 is the edge length VBSP's WindingIsTiny counts from)
  first3 on line  the first three points give no plane at all (their cross
                  product is exactly zero, which is also the test VVIS makes).
                  VVIS derives the portal's plane from them, and before issue
                  #110 such a portal ended up closed. Points that are only
                  nearly on a line are not counted here; "micro edge" is the
                  measure for those.
  tiny            fewer than three edges longer than 0.2 (WindingIsTiny's own
                  rule; VBSP is meant to have dropped these already)
  thin            width under 0.5 / 1 / 2 units, where width is the smallest
                  extent of the polygon across any of its edges
  small           area under 1 / 16 / 64 square units
  bent            a point further than 0.01 from the polygon's own plane
  reflex          a corner that turns the wrong way by more than 0.01 units
                  (VVIS's clip rules assume a convex window)
Leaves (from the .bsp, each leaf's convex cell rebuilt in double by cutting the
world box with the node planes on the way down the tree):
  empty cell      a non-solid leaf whose cell has no volume in double: the
                  planes that bound it leave nothing between them
  small           open volume under 1 / 64 / 512 cubic units
  thin            3 * volume / surface under 0.5 / 1 / 2 units (the inradius of
                  the cell is between a third of that and that)
A cluster's volume is the sum over its non-solid leaves.
--list N prints the N worst portals and clusters of each kind.
"""
import argparse
import json
import math
import re
import struct
import sys

sys.setrecursionlimit(100000)
POINT = re.compile(r"\(([^)]*)\)")

# ---------------------------------------------------------------------------
# vector helpers (plain tuples; the files are small enough)


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def length(a):
    return math.sqrt(dot(a, a))


def area_vector(pts):
    ax = ay = az = 0.0
    p0 = pts[0]
    for i in range(1, len(pts) - 1):
        c = cross(sub(pts[i], p0), sub(pts[i + 1], p0))
        ax += c[0]
        ay += c[1]
        az += c[2]
    return (0.5 * ax, 0.5 * ay, 0.5 * az)


# ---------------------------------------------------------------------------
# portals


def load_prt(path):
    with open(path) as f:
        magic = f.readline().strip()
        nclusters = int(f.readline())
        nportals = int(f.readline())
        portals = []
        for line in f:
            line = line.strip()
            if not line:
                continue
            head = line.split("(", 1)[0].split()
            pts = [tuple(float(v) for v in m.split()) for m in POINT.findall(line)]
            portals.append((int(head[1]), int(head[2]), pts))
    if magic != "PRT1" or len(portals) != nportals:
        sys.exit(f"{path}: not a PRT1 file or portal count mismatch")
    return nclusters, portals


def portal_metrics(pts):
    n = len(pts)
    edges = [length(sub(pts[(i + 1) % n], pts[i])) for i in range(n)]
    av = area_vector(pts)
    area = length(av)
    m = {
        "points": n,
        "area": area,
        "min_edge": min(edges),
        "max_edge": max(edges),
        "dup": sum(1 for e in edges if e == 0.0),
        "long_edges": sum(1 for e in edges if e > 0.2),
        "max_coord": max(abs(c) for p in pts for c in p),
    }
    first3 = cross(sub(pts[0], pts[1]), sub(pts[2], pts[1])) if n >= 3 else (0, 0, 0)
    m["first3_degenerate"] = length(first3) == 0.0
    if area > 0.0:
        nrm = (av[0] / area, av[1] / area, av[2] / area)
        cx = sum(p[0] for p in pts) / n
        cy = sum(p[1] for p in pts) / n
        cz = sum(p[2] for p in pts) / n
        m["bend"] = max(abs(dot(sub(p, (cx, cy, cz)), nrm)) for p in pts)
        # width: for each edge, the furthest any point lies from the edge's line,
        # measured in the polygon's plane; the smallest of those
        width = float("inf")
        reflex = 0.0
        for i in range(n):
            a, b = pts[i], pts[(i + 1) % n]
            e = sub(b, a)
            el = length(e)
            if el == 0.0:
                continue
            inward = cross(nrm, (e[0] / el, e[1] / el, e[2] / el))
            ds = [dot(sub(p, a), inward) for p in pts]
            width = min(width, max(ds) - min(ds))
            # nrm follows the winding's own sense, so every point of a convex
            # polygon is on the inward side of every edge
            reflex = max(reflex, -min(ds))
        m["width"] = width
        m["reflex"] = reflex
    else:
        m["bend"] = 0.0
        m["width"] = 0.0
        m["reflex"] = 0.0
    return m


# ---------------------------------------------------------------------------
# leaves: convex cells from the BSP tree

LUMP_PLANES, LUMP_NODES, LUMP_LEAFS, LUMP_MODELS = 1, 5, 10, 14
CONTENTS_SOLID = 1
WORLD = 32768.0
CLIP_EPS = 1e-7


def load_bsp(path):
    with open(path, "rb") as f:
        data = f.read()
    if data[:4] != b"VBSP":
        sys.exit(f"{path}: not a BSP")

    def lump(i):
        ofs, ln, ver, uncompressed = struct.unpack_from("<iiii", data, 8 + i * 16)
        if uncompressed:
            sys.exit(f"{path}: lump {i} is compressed (bspzip -repack -compress); "
                     f"this script reads plain lumps only")
        return data[ofs:ofs + ln], ver

    raw, _ = lump(LUMP_PLANES)
    planes = [struct.unpack_from("<4f", raw, i * 20) for i in range(len(raw) // 20)]
    raw, _ = lump(LUMP_NODES)
    nodes = [struct.unpack_from("<iii", raw, i * 32) for i in range(len(raw) // 32)]
    raw, ver = lump(LUMP_LEAFS)
    size = 56 if ver == 0 else 32
    leafs = [struct.unpack_from("<ih", raw, i * size) for i in range(len(raw) // size)]
    raw, _ = lump(LUMP_MODELS)
    headnode = struct.unpack_from("<i", raw, 36)[0]
    return planes, nodes, leafs, headnode


def box_cell():
    w = WORLD
    c = [(x, y, z) for x in (-w, w) for y in (-w, w) for z in (-w, w)]
    quads = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    return [[c[i] for i in q] for q in quads]


def split_cell(faces, nrm, dist):
    """Cut a convex cell (list of polygons) by the plane; returns (front, back)."""
    front, back = [], []
    cut_f = []
    for poly in faces:
        ds = [dot(p, nrm) - dist for p in poly]
        if all(d >= -CLIP_EPS for d in ds):
            front.append(poly)
            if all(d <= CLIP_EPS for d in ds):
                back.append(poly)
            else:
                cut_f.extend(p for p, d in zip(poly, ds) if abs(d) <= CLIP_EPS)
            continue
        if all(d <= CLIP_EPS for d in ds):
            back.append(poly)
            cut_f.extend(p for p, d in zip(poly, ds) if abs(d) <= CLIP_EPS)
            continue
        fp, bp = [], []
        n = len(poly)
        for i in range(n):
            p, d = poly[i], ds[i]
            q, e = poly[(i + 1) % n], ds[(i + 1) % n]
            if d >= -CLIP_EPS:
                fp.append(p)
            if d <= CLIP_EPS:
                bp.append(p)
            if (d > CLIP_EPS and e < -CLIP_EPS) or (d < -CLIP_EPS and e > CLIP_EPS):
                t = d / (d - e)
                m = (p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1]), p[2] + t * (q[2] - p[2]))
                fp.append(m)
                bp.append(m)
                cut_f.append(m)
            elif abs(d) <= CLIP_EPS:
                cut_f.append(p)
        if len(fp) >= 3:
            front.append(fp)
        if len(bp) >= 3:
            back.append(bp)
    cap = cap_polygon(cut_f, nrm)
    if cap:
        front.append(cap)
        back.append(cap)
    return front, back


def cap_polygon(points, nrm):
    """The cut's own face: the distinct cut points in order round the normal."""
    uniq = []
    for p in points:
        if not any(abs(p[0] - q[0]) + abs(p[1] - q[1]) + abs(p[2] - q[2]) < 1e-6 for q in uniq):
            uniq.append(p)
    if len(uniq) < 3:
        return None
    cx = sum(p[0] for p in uniq) / len(uniq)
    cy = sum(p[1] for p in uniq) / len(uniq)
    cz = sum(p[2] for p in uniq) / len(uniq)
    c = (cx, cy, cz)
    k = min(range(3), key=lambda i: abs(nrm[i]))
    ref = [0.0, 0.0, 0.0]
    ref[k] = 1.0
    u = cross(nrm, ref)
    ul = length(u)
    u = (u[0] / ul, u[1] / ul, u[2] / ul)
    v = cross(nrm, u)
    uniq.sort(key=lambda p: math.atan2(dot(sub(p, c), v), dot(sub(p, c), u)))
    return uniq


def cell_volume_surface(faces):
    pts = [p for poly in faces for p in poly]
    if len(faces) < 4 or not pts:
        return 0.0, 0.0
    c = (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts),
         sum(p[2] for p in pts) / len(pts))
    vol = surf = 0.0
    for poly in faces:
        av = area_vector(poly)
        surf += length(av)
        vol += abs(dot(av, sub(poly[0], c))) / 3.0
    return vol, surf


def leaf_cells(planes, nodes, leafs, headnode):
    """(leaf index, volume, surface) for every leaf reached from the world model."""
    out = []
    stack = [(headnode, box_cell())]
    while stack:
        num, cell = stack.pop()
        if num < 0:
            vol, surf = cell_volume_surface(cell)
            out.append((-1 - num, vol, surf))
            continue
        planenum, front_child, back_child = nodes[num]
        nx, ny, nz, dist = planes[planenum]
        front, back = split_cell(cell, (nx, ny, nz), dist)
        stack.append((front_child, front))
        stack.append((back_child, back))
    return out


# ---------------------------------------------------------------------------


def count(values, limit):
    return sum(1 for v in values if v < limit)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("prt")
    ap.add_argument("--bsp")
    ap.add_argument("--json")
    ap.add_argument("--list", type=int, default=0)
    args = ap.parse_args()

    nclusters, portals = load_prt(args.prt)
    ms = [portal_metrics(p[2]) for p in portals]
    res = {"prt": args.prt, "clusters": nclusters, "portals": len(portals)}
    P = res["portal_counts"] = {
        "dup_vertex": sum(1 for m in ms if m["dup"]),
        "edge_lt_0.01": count((m["min_edge"] for m in ms), 0.01),
        "edge_lt_0.1": count((m["min_edge"] for m in ms), 0.1),
        "edge_lt_0.2": count((m["min_edge"] for m in ms), 0.2),
        "first3_on_line": sum(1 for m in ms if m["first3_degenerate"]),
        "tiny_by_vbsp_rule": sum(1 for m in ms if m["long_edges"] < 3),
        "width_lt_0.5": count((m["width"] for m in ms), 0.5),
        "width_lt_1": count((m["width"] for m in ms), 1.0),
        "width_lt_2": count((m["width"] for m in ms), 2.0),
        "area_lt_1": count((m["area"] for m in ms), 1.0),
        "area_lt_16": count((m["area"] for m in ms), 16.0),
        "area_lt_64": count((m["area"] for m in ms), 64.0),
        "bent_gt_0.01": sum(1 for m in ms if m["bend"] > 0.01),
        "reflex_gt_0.01": sum(1 for m in ms if m["reflex"] > 0.01),
        "max_coord": max(m["max_coord"] for m in ms) if ms else 0.0,
    }
    print(f"{args.prt}\n  clusters {nclusters}  portals {len(portals)}  "
          f"largest coordinate {P['max_coord']:.0f}")
    print("  portals: " + "  ".join(f"{k} {v}" for k, v in P.items() if k != "max_coord"))

    # clusters whose every portal is a sliver (width < 1): nothing wider leads in
    per_cluster = {}
    for (c0, c1, _), m in zip(portals, ms):
        for c in (c0, c1):
            per_cluster.setdefault(c, []).append(m["width"])
    only_thin = sorted(c for c, ws in per_cluster.items() if max(ws) < 1.0)
    res["clusters_with_only_thin_portals"] = len(only_thin)
    res["clusters_without_portals"] = nclusters - len(per_cluster)
    print(f"  clusters whose every portal is under 1 unit wide: {len(only_thin)}; "
          f"clusters with no portal: {res['clusters_without_portals']}")

    if args.list:
        for key, label in (("width", "thinnest"), ("area", "smallest"), ("min_edge", "shortest edge")):
            order = sorted(range(len(ms)), key=lambda i: ms[i][key])[:args.list]
            print(f"  {label} portals (file index: clusters, {key}, area, width, points):")
            for i in order:
                print(f"    {i}: {portals[i][0]}<->{portals[i][1]}  {key} {ms[i][key]:.4g}  "
                      f"area {ms[i]['area']:.4g}  width {ms[i]['width']:.4g}  "
                      f"points {ms[i]['points']}  at {ms[i]['max_coord']:.0f}")

    if args.bsp:
        planes, nodes, leafs, headnode = load_bsp(args.bsp)
        cells = leaf_cells(planes, nodes, leafs, headnode)
        open_cells = [(i, v, s) for i, v, s in cells
                      if leafs[i][1] >= 0 and not leafs[i][0] & CONTENTS_SOLID]
        vols = [v for _, v, _ in open_cells]
        thick = [3.0 * v / s if s > 0 else 0.0 for _, v, s in open_cells]
        cluster_vol = {}
        cluster_leaves = {}
        for i, v, _ in open_cells:
            c = leafs[i][1]
            cluster_vol[c] = cluster_vol.get(c, 0.0) + v
            cluster_leaves[c] = cluster_leaves.get(c, 0) + 1
        cv = list(cluster_vol.values())
        L = res["leaf_counts"] = {
            "open_leaves": len(open_cells),
            "empty_cell": sum(1 for v in vols if v <= 1e-6),
            "vol_lt_1": count(vols, 1.0), "vol_lt_64": count(vols, 64.0),
            "vol_lt_512": count(vols, 512.0),
            "thick_lt_0.5": count(thick, 0.5), "thick_lt_1": count(thick, 1.0),
            "thick_lt_2": count(thick, 2.0),
        }
        C = res["cluster_counts"] = {
            "clusters_with_leaves": len(cv),
            "vol_lt_1": count(cv, 1.0), "vol_lt_64": count(cv, 64.0),
            "vol_lt_512": count(cv, 512.0),
        }
        res["cluster_volume"] = {str(c): v for c, v in cluster_vol.items()}
        print("  leaves:   " + "  ".join(f"{k} {v}" for k, v in L.items()))
        print("  clusters: " + "  ".join(f"{k} {v}" for k, v in C.items()))
        if args.list:
            order = sorted(cluster_vol, key=lambda c: cluster_vol[c])[:args.list]
            print("  smallest clusters (cluster: volume, leaves, portals):")
            for c in order:
                print(f"    {c}: {cluster_vol[c]:.4g}  leaves {cluster_leaves[c]}  "
                      f"portals {len(per_cluster.get(c, []))}")

    if args.json:
        res["portal_metrics"] = [
            {"clusters": [p[0], p[1]], **{k: (round(v, 6) if isinstance(v, float) else v)
                                          for k, v in m.items()}}
            for p, m in zip(portals, ms)]
        with open(args.json, "w") as f:
            json.dump(res, f)
    return 0


if __name__ == "__main__":
    sys.exit(main())
