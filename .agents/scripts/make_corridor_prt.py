#!/usr/bin/env python3
"""Write a portal file for a straight corridor of N clusters, plus a BSP to run it with.

For checks of vvis's portal flow that need a chain of a known depth: cluster i
sees cluster j through every portal between them, so the flow follows one
chain N - 2 portals deep, which no real map comes near (the stack comment in
src/utils/vvis/flow.cpp has the measured depths). A full run costs O(N^2)
(2,000 clusters took 4 minutes); "-trace 0 <N-1>" follows the one chain and
takes seconds, which is enough to reach the flow's depth check:
5,000 clusters complete, 8,000 stop with the "deeper than the flow's stack
holds" error.

Usage:
  python -I .agents/scripts/make_corridor_prt.py <out_dir> <clusters> [--name corridor]

Writes <out_dir>/<name>.prt and copies the vvis_mergeflow fixture BSP beside it
as <name>.bsp (vvis needs a BSP next to the portal file; only the portal file
drives the flow, and it must have at least as many clusters as that BSP: 21).
Run it with:  vvis_optix.exe -game <dir> [-v] <out_dir>/<name>
"""
import argparse
import os
import shutil
import subprocess

SIZE = 64  # portal edge and cluster length


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out_dir")
    ap.add_argument("clusters", type=int)
    ap.add_argument("--name", default="corridor")
    args = ap.parse_args()
    if args.clusters < 21:
        ap.error("the fixture BSP has 21 clusters; the portal file needs at least as many")

    os.makedirs(args.out_dir, exist_ok=True)
    lines = ["PRT1", str(args.clusters), str(args.clusters - 1)]
    for i in range(args.clusters - 1):
        x = SIZE * (i + 1)
        # Wound so the plane vvis derives faces cluster i; vvis turns it round
        # for the portal leading from i into i + 1.
        pts = [(x, 0, 0), (x, SIZE, 0), (x, SIZE, SIZE), (x, 0, SIZE)]
        text = " ".join(f"({px} {py} {pz} )" for px, py, pz in pts)
        lines.append(f"4 {i} {i + 1} {text} ")
    with open(os.path.join(args.out_dir, args.name + ".prt"), "w", newline="\n") as f:
        f.write("\n".join(lines) + "\n")

    root = subprocess.run(["git", "rev-parse", "--show-toplevel"], capture_output=True,
                          text=True, cwd=os.path.dirname(os.path.abspath(__file__))).stdout.strip()
    bsp = os.path.join(root, "game", "bin", "x64", "unit_test", "vvis_mergeflow",
                       "fixtures", "validation.bsp")
    shutil.copyfile(bsp, os.path.join(args.out_dir, args.name + ".bsp"))
    print(f"{args.clusters} clusters, {args.clusters - 1} portals -> "
          f"{os.path.join(args.out_dir, args.name)}")


if __name__ == "__main__":
    main()
