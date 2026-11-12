#!/usr/bin/env python3
"""Write an edited copy of a portal file, to measure what its slivers cost in the PVS (issue #104).

Usage:
  python -I experiments/prtvis/prt_variants.py <in.prt> <out.prt> <edit> [value]

Edits (one per call; point text is copied unchanged, so nothing is re-rounded):
  rotate [K]       start every winding K points later (default 1). Same portals,
                   same order, same geometry: what the PVS moves by is how much
                   it depends on which corner a winding happens to start at.
  clean [EPS]      drop a point closer than EPS (default 0.01, VVIS's on-plane
                   distance) to the point kept before it. Portals left with
                   fewer than 3 points are dropped.
  drop-thin [W]    drop portals narrower than W units (default 1).
  drop-small [A]   drop portals under A square units (default 1).
  fix-first3       start a winding whose first three points are on one line at
                   a corner where they are not (VVIS derives the plane from the
                   first three points).
Prints how many portals the edit touched and how many it dropped.
"""
import argparse
import math
import os
import re
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # python -I drops it

from sliver_stats import cross, length, portal_metrics, sub  # noqa: E402

POINT = re.compile(r"\([^)]*\)")


def parse(line):
    head = line.split("(", 1)[0].split()
    texts = POINT.findall(line)
    pts = [tuple(float(v) for v in t.strip("()").split()) for t in texts]
    return int(head[1]), int(head[2]), texts, pts


def first3_ok(pts, k):
    n = len(pts)
    a, b, c = pts[k % n], pts[(k + 1) % n], pts[(k + 2) % n]
    return length(cross(sub(a, b), sub(c, b))) > 0.0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("src")
    ap.add_argument("dst")
    ap.add_argument("edit", choices=["rotate", "clean", "drop-thin", "drop-small", "fix-first3"])
    ap.add_argument("value", nargs="?", type=float)
    args = ap.parse_args()
    default = {"rotate": 1, "clean": 0.01, "drop-thin": 1.0, "drop-small": 1.0, "fix-first3": 0}
    value = args.value if args.value is not None else default[args.edit]

    with open(args.src) as f:
        lines = [ln.rstrip("\n") for ln in f]
    nclusters = int(lines[1])
    out = []
    touched = dropped = 0
    for line in lines[3:]:
        if not line.strip():
            continue
        c0, c1, texts, pts = parse(line)
        n = len(pts)
        keep = list(range(n))
        if args.edit == "rotate":
            k = int(value) % n
            keep = keep[k:] + keep[:k]
            if k:
                touched += 1
        elif args.edit == "clean":
            keep = [0]
            for i in range(1, n):
                if math.dist(pts[i], pts[keep[-1]]) >= value:
                    keep.append(i)
            while len(keep) > 1 and math.dist(pts[keep[-1]], pts[keep[0]]) < value:
                keep.pop()
            if len(keep) != n:
                touched += 1
        elif args.edit == "drop-thin":
            if portal_metrics(pts)["width"] < value:
                keep = []
        elif args.edit == "drop-small":
            if portal_metrics(pts)["area"] < value:
                keep = []
        elif args.edit == "fix-first3":
            if not first3_ok(pts, 0):
                for k in range(1, n):
                    if first3_ok(pts, k):
                        keep = keep[k:] + keep[:k]
                        touched += 1
                        break
        if len(keep) < 3:
            dropped += 1
            continue
        out.append(f"{len(keep)} {c0} {c1} " + " ".join(texts[i] for i in keep) + " ")

    with open(args.dst, "w", newline="\n") as f:
        f.write("PRT1\n%d\n%d\n" % (nclusters, len(out)))
        f.write("\n".join(out) + "\n")
    print(f"{args.edit} {value:g}: {len(out)} portals written, {touched} changed, {dropped} dropped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
