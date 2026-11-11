#!/usr/bin/env python3
"""
run_sightline_tests -- vvis_optix must keep cluster pairs that a straight sight line connects.

    python run_sightline_tests.py                             # the fixtures next to this script
    python run_sightline_tests.py --game <dir> --bin <dir containing vvis_optix.exe>
    python run_sightline_tests.py --keep                      # keep the temporary folder

Issue #89: the default (exact) portal flow used to clip with 32-bit float windings and separator
planes (src/utils/vvis/flow.cpp, ClipToSeperators / ChopWinding).  On long, grazing portal chains
the rounding decided a side test wrongly and the flow dropped a cluster pair that is plainly
visible.  The flow's geometry is double now; this harness keeps it from going back.

Each fixture is a portal file cut out of a real map (gm_fightspace_redux without func_viscluster):
only the clusters and portals of one portal chain, renumbered, so the chain is the only route
from its first cluster to its last.  Coordinates are the source .prt text, unchanged.  The .json
beside it lists the expected pairs, and for each one a straight segment and a margin.

Per fixture:
    proof   checked in exact arithmetic (fractions.Fraction on the decimal text of the .prt)
            before VVIS runs: the segment crosses the pair's portals in order, each in the
            direction of the chain, each strictly inside the polygon by at least the margin.
            So the pair is truly visible, whatever VVIS says.  A broken proof is an ERROR.
    A->B    cluster A's PVS row contains cluster B, after one default-flow VVIS run
    B->A    cluster B's PVS row contains cluster A
    threads the PVS rows are the same after a second run with -threads 1 (these chains are where
            rounding decides, so they are the ones to show the result does not depend on threads)
VVIS runs on a copy of the BSP of ../vvis_mergeflow/fixtures (it needs a BSP beside the portal
file; only the portal file drives the flow), with the visibility lump emptied first.  The portal
file must have at least as many clusters as that BSP: VVIS sizes the cluster table from the
portal file, and the BSP's leaves index it after the flow (PAS, water distance, fog volumes).

Exit code: 0 every check passed; 1 at least one check failed and nothing errored; 2 a VVIS run,
the PVS read or a proof failed, or a usage error.  A temporary folder with a non-passing case is
kept (its path is printed).
"""
from __future__ import annotations

import argparse
import json
import re
import shutil
import struct
import subprocess
import sys
import tempfile
from fractions import Fraction
from pathlib import Path
from typing import List, Sequence, Tuple

THIS_DIR = Path(__file__).resolve().parent
X64_DIR = THIS_DIR.parents[1]                                  # game/bin/x64
sys.dont_write_bytecode = True                                 # a test run must not leave a __pycache__ in x64
if str(X64_DIR) not in sys.path:                               # python -I drops the script dir and the cwd
    sys.path.insert(0, str(X64_DIR))

from bsp_diff_visibility import BSPVisibility                  # noqa: E402

FIXTURE_DIR = THIS_DIR / "fixtures"
BSP_SOURCE = THIS_DIR.parent / "vvis_mergeflow" / "fixtures" / "validation.bsp"
DEFAULT_GAME = r"E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\sourcetest"
VVIS_NAME = "vvis_optix.exe"
LUMP_ENTITIES, LUMP_VISIBILITY = 0, 4


class RunError(Exception):
    """A VVIS run, a PVS read or a proof failed, so the case's checks mean nothing (ERROR)."""


def usage_error(msg: str):
    print(f"usage error: {msg}", file=sys.stderr)
    raise SystemExit(2)


def describe_exit(rc: int) -> str:
    return f"0x{rc & 0xFFFFFFFF:08X}" if rc < 0 or rc >= 0x80000000 else str(rc)


# ---------------------------------------------------------------------------
# the portal file and the proof (exact arithmetic)
# ---------------------------------------------------------------------------
def read_prt(path: Path) -> Tuple[int, List[Tuple[int, int, List[List[Fraction]]]]]:
    """(cluster count, [(cluster 0, cluster 1, points)]) with every coordinate an exact Fraction."""
    lines = path.read_text().splitlines()
    if not lines or lines[0].strip() != "PRT1":
        raise RunError(f"{path.name} is not a PRT1 portal file")
    nclusters, nportals = int(lines[1]), int(lines[2])
    body = [l for l in lines[3:] if l.strip()]
    if len(body) != nportals:
        raise RunError(f"{path.name}: header says {nportals} portals, file has {len(body)}")
    portals = []
    for i, line in enumerate(body):
        m = re.match(r"\s*(\d+)\s+(\d+)\s+(\d+)\s+(.*)$", line)
        pts = [[Fraction(c) for c in t.split()] for t in re.findall(r"\(([^)]*)\)", m.group(4))] if m else []
        if not m or len(pts) != int(m.group(1)) or len(pts) < 3 or any(len(p) != 3 for p in pts):
            raise RunError(f"{path.name}: cannot parse portal {i}")
        portals.append((int(m.group(2)), int(m.group(3)), pts))
    return nclusters, portals


def _sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def check_proof(portals, pair) -> float:
    """Raise RunError unless the pair's segment crosses its portals in order, in the chain's
    direction, strictly inside each by the margin.  Returns the least margin found (approximate)."""
    a, b = [[Fraction(c) for c in p] for p in pair["segment"]]
    margin = Fraction(pair["margin"])
    if margin <= 0:
        raise RunError("proof margin must be positive")
    d = _sub(b, a)
    cluster, last_t, least = pair["from"], Fraction(0), None
    for k, pi in enumerate(pair["portals"]):
        l0, l1, pts = portals[pi]
        if cluster not in (l0, l1):
            raise RunError(f"proof: portal {pi} does not touch cluster {cluster}")
        nxt = l1 if cluster == l0 else l0
        n = _cross(_sub(pts[1], pts[0]), _sub(pts[2], pts[0]))   # the winding's own orientation
        if _dot(n, n) == 0 or any(_dot(n, _sub(p, pts[0])) != 0 for p in pts):
            raise RunError(f"proof: portal {pi} is degenerate or not planar")
        for i in range(len(pts)):                                 # strictly convex, same turn everywhere
            turn = _cross(_sub(pts[(i + 1) % len(pts)], pts[i]), _sub(pts[(i + 2) % len(pts)], pts[i]))
            if _dot(turn, n) <= 0:
                raise RunError(f"proof: portal {pi} is not strictly convex")
        # VVIS (LoadPortals): the portal from the line's first cluster to its second faces
        # -(p0-p1)x(p2-p1) = n.  The segment must cross from `cluster` into `nxt`.
        dn = _dot(d, n)
        if not ((dn > 0) if cluster == l0 else (dn < 0)):
            raise RunError(f"proof: the segment crosses portal {pi} against the chain direction")
        sa, sb = _dot(n, _sub(a, pts[0])), _dot(n, _sub(b, pts[0]))
        t = sa / (sa - sb)
        if not (last_t < t < 1) or (k == 0 and t <= 0):
            raise RunError(f"proof: portal {pi} is not crossed after the previous one, inside the segment")
        x = [a[j] + t * d[j] for j in range(3)]
        n2 = _dot(n, n)
        for i in range(len(pts)):
            e = _sub(pts[(i + 1) % len(pts)], pts[i])
            s = _dot(_cross(e, _sub(x, pts[i])), n)               # = (distance to edge line) * |e| * |n|
            if s <= 0 or s * s < margin * margin * _dot(e, e) * n2:
                raise RunError(f"proof: the crossing of portal {pi} is not inside it by {pair['margin']}")
            dist = float(s) / (float(_dot(e, e)) ** 0.5 * float(n2) ** 0.5)
            least = dist if least is None else min(least, dist)
        cluster, last_t = nxt, t
    if cluster != pair["to"]:
        raise RunError(f"proof: the portals lead to cluster {cluster}, not {pair['to']}")
    return least


# ---------------------------------------------------------------------------
# running VVIS
# ---------------------------------------------------------------------------
def lump(data: bytes, index: int) -> bytes:
    ofs, length = struct.unpack_from("<ii", data, 8 + 16 * index)
    return data[ofs:ofs + length]


def prepare_bsp(dst: Path, prt_clusters: int) -> None:
    """Copy the shared BSP, check it is safe and meaningful to pair, empty its visibility lump."""
    data = BSP_SOURCE.read_bytes()
    vis = lump(data, LUMP_VISIBILITY)
    bsp_clusters = struct.unpack_from("<i", vis, 0)[0] if len(vis) >= 4 else 0
    if bsp_clusters <= 0 or prt_clusters < bsp_clusters:
        raise RunError(f"the portal file has {prt_clusters} clusters, {BSP_SOURCE.name} has {bsp_clusters}: "
                       f"pad the portal file's cluster count to at least that")
    if b"env_fog_controller" in lump(data, LUMP_ENTITIES):
        raise RunError(f"{BSP_SOURCE.name} has an env_fog_controller, so VVIS would cull by radius")
    dst.write_bytes(data)
    with open(dst, "r+b") as f:                                # header entry 4: fileofs, then filelen
        f.seek(8 + 16 * LUMP_VISIBILITY + 4)
        f.write((0).to_bytes(4, "little"))


def run_vvis(bin_dir: Path, game: str, bsp: Path, timeout: float, extra: Sequence[str] = ()) -> None:
    cmd = [str(bin_dir / VVIS_NAME), "-game", game, *extra, str(bsp)]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace",
                              timeout=timeout, cwd=str(bin_dir))
    except subprocess.TimeoutExpired:
        raise RunError(f"vvis timed out after {timeout:.0f}s") from None
    except OSError as exc:
        raise RunError(f"cannot execute {cmd[0]}: {exc}") from None
    bsp.with_suffix(".vvis.out").write_text(f"$ {' '.join(cmd)}\nexit {proc.returncode}\n--- stdout ---\n"
                                            f"{proc.stdout}\n--- stderr ---\n{proc.stderr}\n", encoding="utf-8")
    if proc.returncode != 0:
        tail = " | ".join((proc.stdout + proc.stderr).strip().splitlines()[-2:])
        raise RunError(f"vvis exited {describe_exit(proc.returncode)}: {tail}")


def read_pvs_rows(bsp: Path) -> List[int]:
    try:
        vis = BSPVisibility(str(bsp))
        if vis.num_clusters <= 0:
            raise RunError(f"{bsp.name} has no visibility lump after vvis")
        return [int.from_bytes(vis.decompress_pvs(i), "little") for i in range(vis.num_clusters)]
    except (OSError, ValueError, IndexError) as exc:
        raise RunError(f"cannot read the PVS of {bsp.name}: {type(exc).__name__}: {exc}") from None


# ---------------------------------------------------------------------------
# one fixture
# ---------------------------------------------------------------------------
def run_case(prt_src: Path, spec: dict, work: Path, args) -> List[Tuple[str, bool, str]]:
    nclusters, portals = read_prt(prt_src)
    checks = []
    for pair in spec["pairs"]:
        least = check_proof(portals, pair)
        checks.append(("proof", True, f"segment {pair['from']}->{pair['to']} crosses {len(pair['portals'])} "
                                      f"portals, each inside by >= {pair['margin']} (least {least:.3f})"))
    work.mkdir(parents=True)
    bsp = work / f"{prt_src.stem}.bsp"
    prepare_bsp(bsp, nclusters)
    shutil.copyfile(prt_src, bsp.with_suffix(".prt"))         # VVIS finds the portal file by the BSP's name
    run_vvis(args.bin, args.game, bsp, args.timeout)
    rows = read_pvs_rows(bsp)
    if len(rows) != nclusters:
        raise RunError(f"the PVS has {len(rows)} clusters, the portal file {nclusters}")
    for pair in spec["pairs"]:
        a, b = pair["from"], pair["to"]
        for x, y in ((a, b), (b, a)):
            seen = bool((rows[x] >> y) & 1)
            checks.append((f"{x}->{y}", seen, f"cluster {x}'s PVS row {'contains' if seen else 'is MISSING'} "
                                              f"cluster {y}"))
    one = work / "threads1" / bsp.name
    one.parent.mkdir()
    prepare_bsp(one, nclusters)
    shutil.copyfile(prt_src, one.with_suffix(".prt"))
    run_vvis(args.bin, args.game, one, args.timeout, ["-threads", "1"])
    differing = sum(a != b for a, b in zip(rows, read_pvs_rows(one)))
    checks.append(("threads", differing == 0, f"{differing} of {len(rows)} PVS rows differ between the default "
                                              f"thread count and -threads 1"))
    return checks


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Check that vvis_optix keeps sight-line-proven cluster pairs.",
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__.split("Per fixture")[0])
    ap.add_argument("--game", default=DEFAULT_GAME, help="game directory containing gameinfo.txt")
    ap.add_argument("--bin", default=str(X64_DIR), help=f"folder holding {VVIS_NAME} (default: game/bin/x64)")
    ap.add_argument("--keep", action="store_true", help="keep the temporary folder (always kept when a case does not pass)")
    ap.add_argument("--timeout", type=float, default=300, help="seconds per vvis run (default 300)")
    args = ap.parse_args(argv)

    args.game = str(Path(args.game).expanduser().resolve())
    args.bin = Path(args.bin).expanduser().resolve()
    if not (args.bin / VVIS_NAME).is_file():
        usage_error(f"{VVIS_NAME} not found in {args.bin}")
    if not (Path(args.game) / "gameinfo.txt").is_file():
        usage_error(f"no gameinfo.txt in {args.game}")
    if not BSP_SOURCE.is_file():
        usage_error(f"missing {BSP_SOURCE}")
    cases = []
    for prt in sorted(FIXTURE_DIR.glob("*.prt")):
        spec = prt.with_suffix(".json")
        if not spec.is_file():
            usage_error(f"fixture {prt.name} has no {spec.name} next to it")
        cases.append((prt.stem, prt, json.loads(spec.read_text())))
    if not cases:
        usage_error(f"no fixtures in {FIXTURE_DIR}")

    print(f"vvis sight-line tests: {len(cases)} fixture(s), bin={args.bin}, game={args.game}")
    tmp = Path(tempfile.mkdtemp(prefix="vvis_sightline_"))
    results = []
    try:
        for i, (name, prt, spec) in enumerate(cases):
            print(f"  running {name}", flush=True)
            try:
                checks = run_case(prt, spec, tmp / f"{i:02d}_{name}", args)
                results.append((name, "PASS" if all(ok for _, ok, _ in checks) else "FAIL", checks))
            except RunError as exc:
                results.append((name, "ERROR", [("run", False, str(exc))]))
            except Exception as exc:                           # noqa: BLE001 -- one case must never sink the run
                results.append((name, "ERROR", [("run", False, f"unexpected {type(exc).__name__}: {exc}")]))
    finally:
        keep = args.keep or any(status != "PASS" for _, status, _ in results)
        if not keep:
            shutil.rmtree(tmp, ignore_errors=True)

    print()
    width = max(len(n) for n, _, _ in results) + 2
    for name, _, checks in results:
        for check, ok, detail in checks:
            state = "ERROR" if check == "run" else ("PASS" if ok else "FAIL")
            print(f"{name:<{width}}{check:<8}{state:<6}{detail}")
    n_err = sum(s == "ERROR" for _, s, _ in results)
    n_fail = sum(s == "FAIL" for _, s, _ in results)
    print(f"\nTOTALS  fixtures {len(results)}: {sum(s == 'PASS' for _, s, _ in results)} pass, {n_fail} fail, "
          f"{n_err} error")
    if keep:
        print(f"kept: {tmp}")
    return 2 if n_err else (1 if n_fail else 0)


if __name__ == "__main__":
    sys.exit(main())
