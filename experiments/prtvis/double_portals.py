#!/usr/bin/env python3
"""Rebuild a map's portals in double from its BSP tree and compare with the .prt (issue #104).

VBSP builds its portals with single-precision windings. This script does the
same construction in double, from the compiled BSP alone: every node's plane is
cut down to the node's convex cell, the piece is pushed down both subtrees to
the leaves, and what a front leaf and a back leaf share is a portal. The node
planes are the BSP's own (single precision, as VBSP stored them), so the result
is "double winding arithmetic on VBSP's planes": what a double-precision
portal path in VBSP would write, without building one.

Usage:
  python -I experiments/prtvis/double_portals.py <map.bsp> <map.prt> [--write out.prt]
                                                 [--list N] [--min-area A]

Compares, per unordered cluster pair:
  both      the pair has portals in the .prt and in the rebuild
  prt only  VBSP wrote a portal the rebuild has none for (or only a speck)
  dbl only  the rebuild finds a shared face VBSP did not write. Most of these
            are pairs VBSP leaves closed on purpose (the two sides' contents);
            the ones under --min-area (default 0.05 square units) are counted
            apart as specks
and the sliver counts (sliver_stats.py's portal measures) of the .prt next to
those of the rebuild restricted to the .prt's pairs.

--write saves the rebuild restricted to the .prt's cluster pairs as a portal
file VVIS can read (each coordinate as the shortest text that reads back as the
same double), so the two can be run through vvis
and their PVS compared pair by pair.
"""
import argparse
import os
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # python -I drops it

import sliver_stats as ss  # noqa: E402  (its CLIP_EPS is set from --eps)
from sliver_stats import (CONTENTS_SOLID, area_vector, box_cell, cross, dot,  # noqa: E402
                          length, load_bsp, load_prt, portal_metrics, split_cell, sub)

sys.setrecursionlimit(100000)


def clip_poly(poly, nrm, dist, keep_front):
    """The part of a convex polygon on one side of a plane (None when nothing is left)."""
    sign = 1.0 if keep_front else -1.0
    CLIP_EPS = ss.CLIP_EPS
    ds = [sign * (dot(p, nrm) - dist) for p in poly]
    if all(d >= -CLIP_EPS for d in ds):
        return poly
    if all(d <= CLIP_EPS for d in ds):
        return None
    out = []
    n = len(poly)
    for i in range(n):
        p, d = poly[i], ds[i]
        q, e = poly[(i + 1) % n], ds[(i + 1) % n]
        if d >= -CLIP_EPS:
            out.append(p)
        if (d > CLIP_EPS and e < -CLIP_EPS) or (d < -CLIP_EPS and e > CLIP_EPS):
            t = d / (d - e)
            out.append((p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1]),
                        p[2] + t * (q[2] - p[2])))
    return out if len(out) >= 3 else None


def push_down(poly, num, planes, nodes, out):
    """Split poly through the subtree at num; appends (leaf, fragment)."""
    stack = [(num, poly)]
    while stack:
        num, poly = stack.pop()
        if num < 0:
            out.append((-1 - num, poly))
            continue
        planenum, front_child, back_child = nodes[num]
        nx, ny, nz, dist = planes[planenum]
        f = clip_poly(poly, (nx, ny, nz), dist, True)
        b = clip_poly(poly, (nx, ny, nz), dist, False)
        if f:
            stack.append((front_child, f))
        if b:
            stack.append((back_child, b))


def intersect(a, b, nrm):
    """Intersection of two convex polygons lying in the plane with normal nrm."""
    out = a
    n = len(b)
    # b's winding sense round nrm decides which side of each edge is inside
    sense = 1.0 if dot(area_vector(b), nrm) >= 0 else -1.0
    for i in range(n):
        p, q = b[i], b[(i + 1) % n]
        e = sub(q, p)
        el = length(e)
        if el == 0.0:
            continue
        inward = cross(nrm, (e[0] / el, e[1] / el, e[2] / el))
        inward = (sense * inward[0], sense * inward[1], sense * inward[2])
        out = clip_poly(out, inward, dot(p, inward), True)
        if not out:
            return None
    return out


def dedupe(poly):
    """Drops points that repeat the one before (the clips can hand the same corner twice)."""
    out = []
    for p in poly:
        if not out or abs(p[0] - out[-1][0]) + abs(p[1] - out[-1][1]) + abs(p[2] - out[-1][2]) > 1e-9:
            out.append(p)
    while len(out) > 1 and abs(out[0][0] - out[-1][0]) + abs(out[0][1] - out[-1][1]) + \
            abs(out[0][2] - out[-1][2]) <= 1e-9:
        out.pop()
    return out


def rebuild(planes, nodes, leafs, headnode):
    """[(front leaf, back leaf, polygon)] for every face two leaves share."""
    portals = []
    stack = [(headnode, box_cell())]
    while stack:
        num, cell = stack.pop()
        if num < 0 or not cell:
            continue
        planenum, front_child, back_child = nodes[num]
        nx, ny, nz, dist = planes[planenum]
        nrm = (nx, ny, nz)
        front, back = split_cell(cell, nrm, dist)
        # split_cell appends the cut's face last on both sides when there is one
        if front and back and front[-1] is back[-1]:
            cap = front[-1]
            fl, bl = [], []
            push_down(cap, front_child, planes, nodes, fl)
            push_down(cap, back_child, planes, nodes, bl)
            for la, fa in fl:
                for lb, fb in bl:
                    shared = intersect(fa, fb, nrm)
                    if shared:
                        shared = dedupe(shared)
                        if len(shared) >= 3:
                            portals.append((la, lb, shared, nrm))
        stack.append((front_child, front))
        stack.append((back_child, back))
    return portals


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bsp")
    ap.add_argument("prt")
    ap.add_argument("--write")
    ap.add_argument("--list", type=int, default=0)
    ap.add_argument("--min-area", type=float, default=0.05)
    ap.add_argument("--eps", type=float, default=1e-7,
                    help="on-plane distance of the rebuild's clips (VBSP splits its "
                         "portals with 0.001)")
    args = ap.parse_args()

    ss.CLIP_EPS = args.eps
    planes, nodes, leafs, headnode = load_bsp(args.bsp)
    nclusters, prt = load_prt(args.prt)
    raw = rebuild(planes, nodes, leafs, headnode)

    def cluster(leaf):
        contents, c = leafs[leaf]
        return -1 if contents & CONTENTS_SOLID else c

    dbl = {}  # unordered cluster pair -> [(c of the plane's front side, c of its back side, polygon)]
    for la, lb, poly, nrm in raw:
        ca, cb = cluster(la), cluster(lb)
        if ca < 0 or cb < 0 or ca == cb:
            continue
        dbl.setdefault((min(ca, cb), max(ca, cb)), []).append((ca, cb, poly, nrm))
    act = {}
    for c0, c1, pts in prt:
        act.setdefault((min(c0, c1), max(c0, c1)), []).append(pts)

    def area(poly):
        return length(area_vector(poly))

    both = sorted(set(act) & set(dbl))
    prt_only = sorted(set(act) - set(dbl))
    dbl_only = sorted(set(dbl) - set(act))
    specks = [k for k in dbl_only if sum(area(p[2]) for p in dbl[k]) < args.min_area]
    print(f"{args.prt}  (rebuild eps {args.eps:g})")
    print(f"  cluster pairs: .prt {len(act)}  rebuild {len(dbl)}  both {len(both)}  "
          f"prt only {len(prt_only)}  dbl only {len(dbl_only)} "
          f"(of them under {args.min_area} sq units: {len(specks)})")
    print(f"  portals: .prt {len(prt)}  rebuild on the .prt's pairs "
          f"{sum(len(dbl[k]) for k in both)}")

    # area agreement on the shared pairs
    worst = []
    for k in both:
        a = sum(area(p) for p in act[k])
        d = sum(area(p[2]) for p in dbl[k])
        worst.append((abs(a - d), k, a, d, len(act[k]), len(dbl[k])))
    worst.sort(reverse=True)
    differing_count = sum(1 for w in worst if w[4] != w[5])
    print(f"  shared pairs whose portal count differs: {differing_count}; "
          f"whose area differs by more than 1 sq unit: {sum(1 for w in worst if w[0] > 1.0)}, "
          f"by more than 0.01: {sum(1 for w in worst if w[0] > 0.01)}")

    def tally(polys):
        ms = [portal_metrics(p) for p in polys]
        return {
            "portals": len(ms),
            "dup_vertex": sum(1 for m in ms if m["dup"]),
            "edge_lt_0.01": sum(1 for m in ms if m["min_edge"] < 0.01),
            "edge_lt_0.1": sum(1 for m in ms if m["min_edge"] < 0.1),
            "edge_lt_0.2": sum(1 for m in ms if m["min_edge"] < 0.2),
            "first3_on_line": sum(1 for m in ms if m["first3_degenerate"]),
            "tiny_by_vbsp_rule": sum(1 for m in ms if m["long_edges"] < 3),
            "width_lt_0.5": sum(1 for m in ms if m["width"] < 0.5),
            "width_lt_1": sum(1 for m in ms if m["width"] < 1.0),
            "area_lt_1": sum(1 for m in ms if m["area"] < 1.0),
        }

    ta = tally([p for k in act for p in act[k]])
    td = tally([p[2] for k in both for p in dbl[k]])
    print("  .prt    : " + "  ".join(f"{k} {v}" for k, v in ta.items()))
    print("  rebuild : " + "  ".join(f"{k} {v}" for k, v in td.items()))

    if prt_only:
        print("  prt only pairs (clusters: portals, area): " + "; ".join(
            f"{k[0]}<->{k[1]}: {len(act[k])}, {sum(area(p) for p in act[k]):.4g}"
            for k in prt_only[:max(args.list, 10)]))
    if args.list:
        print("  largest area differences on shared pairs (clusters: .prt area/count, rebuild area/count):")
        for diff, k, a, d, na, nd in worst[:args.list]:
            print(f"    {k[0]}<->{k[1]}: {a:.4f}/{na}  {d:.4f}/{nd}")
        big = sorted(((sum(area(p[2]) for p in dbl[k]), k) for k in dbl_only), reverse=True)
        print("  largest dbl only pairs (clusters: area): " + "; ".join(
            f"{k[0]}<->{k[1]}: {a:.4g}" for a, k in big[:args.list]))

    if args.write:
        # One rebuilt portal per .prt portal, in the .prt's order, with its
        # cluster order and starting at the corner nearest the .prt's first
        # point, so the two files differ in the geometry and in nothing else
        # (VVIS's result depends a little on portal order and starting corner).
        def centroid(poly):
            n = len(poly)
            return (sum(p[0] for p in poly) / n, sum(p[1] for p in poly) / n,
                    sum(p[2] for p in poly) / n)

        left = {k: list(v) for k, v in dbl.items()}
        lines = []
        unmatched = 0
        for c0, c1, pts in prt:
            k = (min(c0, c1), max(c0, c1))
            cands = left.get(k)
            if not cands:
                unmatched += 1
                continue
            c = centroid(pts)
            best = min(range(len(cands)), key=lambda i: length(sub(centroid(cands[i][2]), c)))
            ca, cb, poly, nrm = cands.pop(best)
            # VVIS takes the plane from the points, (p0-p1)x(p2-p1), which is
            # minus the fan's area vector, and wants it to face the first
            # cluster listed. The node plane's normal faces ca's side.
            want = nrm if c0 == ca else (-nrm[0], -nrm[1], -nrm[2])
            out = poly if dot(area_vector(poly), want) < 0 else poly[::-1]
            start = min(range(len(out)), key=lambda i: length(sub(out[i], pts[0])))
            out = out[start:] + out[:start]
            text = " ".join(f"({p[0]!r} {p[1]!r} {p[2]!r} )" for p in out)
            lines.append(f"{len(out)} {c0} {c1} {text} ")
        if unmatched:
            print(f"  {unmatched} .prt portals have no rebuilt counterpart and were left out")
        # A pair the rebuild cuts into more portals than the .prt has: the
        # extra ones are written too, after the matched ones, so the file is
        # the whole rebuild on the .prt's pairs and not a part of it.
        extra = 0
        for k in both:
            for ca, cb, poly, nrm in left.get(k, []):
                out = poly if dot(area_vector(poly), nrm) < 0 else poly[::-1]
                text = " ".join(f"({p[0]!r} {p[1]!r} {p[2]!r} )" for p in out)
                lines.append(f"{len(out)} {ca} {cb} {text} ")
                extra += 1
        if extra:
            print(f"  {extra} rebuilt portals had no .prt portal to pair with and were appended")
        with open(args.write, "w", newline="\n") as f:
            f.write("PRT1\n%d\n%d\n" % (nclusters, len(lines)))
            f.write("\n".join(lines) + "\n")
        print(f"  wrote {args.write}: {len(lines)} portals")
    return 0


if __name__ == "__main__":
    sys.exit(main())
