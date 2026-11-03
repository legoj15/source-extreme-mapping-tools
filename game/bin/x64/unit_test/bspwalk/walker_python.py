#!/usr/bin/env python3
"""
walker_python -- run the current Python walker on a compiled BSP and write the
bspwalk JSON contract.  This is the BASELINE walker the native bspwalk.exe is
scored against; it reuses the same in-process pipeline as vis_simulator.py:

    BSP -> CollisionWorld -> ReachabilityMap(16u grid) -> eye positions
        -> VisibilityOracle.classify_faces -> never-visible faces

Usage:
    python walker_python.py --bsp x.bsp --game <game dir> --out x.walk.json [--vmf x.vmf] [--workers N]
                            [--no-side-flip]

--no-side-flip is a diagnostic, off by default (the baseline is the walker AS SHIPPED).
visibility.py negates the plane normal of every face with dface_t.side == 1, but in BSPs
from this repo's vbsp_lmo.exe the stored plane is already the face's own orientation, so
the flip points the sample offsets and the backface cull the wrong way for every face whose
outward normal is -x/-y/-z.  This flag forces side=0 on the oracle's private copy of the
faces so the remaining defects can be seen without that one.

Output (the contract every walker must write):
    {"walker": "python",
     "faces": [{"face": int, "sideids": [int], "tier": "A"|"B"|"visible", "reason": str}],
     "stats": {...timings and counts...}}
Faces absent from "faces" are treated as visible.  The Python walker has no
proof tier: every face it calls never-visible is tier "B" (ray-sampled) with
reason "python-ray".

Exit code: 0 on success.  Anything that would make the answer meaningless
(no VPK could be loaded, no spawn point, nothing reachable, no side-id lumps)
exits non-zero with a message instead of writing an empty result.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

# vmt_checker prints an arrow character; a piped Windows stdout is cp1252 and would crash.
for _stream in (sys.stdout, sys.stderr):
    if hasattr(_stream, "reconfigure"):
        _stream.reconfigure(encoding="utf-8", errors="replace")

_X64_DIR = Path(__file__).resolve().parents[2]       # game/bin/x64
if str(_X64_DIR) not in sys.path:
    sys.path.insert(0, str(_X64_DIR))

from bsp_reader import BSPReader                                   # noqa: E402
from collision import CollisionWorld                               # noqa: E402
from reachability import ReachabilityMap                           # noqa: E402
from visibility import VisibilityOracle                            # noqa: E402
from vmt_checker import _resolve_search_paths, _find_dir_vpk       # noqa: E402
from vpk_reader import VPKReader                                   # noqa: E402


class WalkerError(RuntimeError):
    pass


def load_vpk_readers(game_dir: Path):
    """Open every VPK that gameinfo.txt lists.

    A listed VPK that is absent from disk is reported (gameinfo routinely lists
    optional ones, e.g. the low-violence pack) but a VPK that exists and fails
    to open is fatal: silently dropping it makes prop collision disappear.
    """
    search_paths = _resolve_search_paths(game_dir, verbose=True)
    readers, missing, loaded = [], [], []
    for sp in search_paths:
        if sp.suffix.lower() != ".vpk":
            continue
        dir_vpk = _find_dir_vpk(sp)
        if dir_vpk is None or not dir_vpk.exists():
            missing.append(str(dir_vpk or sp))
            continue
        try:
            readers.append(VPKReader(dir_vpk))
        except Exception as exc:                        # noqa: BLE001
            raise WalkerError(f"VPK {dir_vpk} exists but failed to load: {exc}") from exc
        loaded.append(str(dir_vpk))
    for path in missing:
        print(f"  WARNING: gameinfo lists a VPK that is not on disk: {path}", flush=True)
    if not readers:
        raise WalkerError(f"no VPK could be loaded for {game_dir}; "
                          f"static-prop collision would silently vanish")
    print(f"  Loaded {len(readers)} VPK(s), {len(missing)} listed but absent", flush=True)
    return readers, loaded, missing


def run(bsp_path: Path, game_dir: Path, workers: int, grid_res: float,
        no_side_flip: bool = False) -> dict:
    t_start = time.perf_counter()
    stats: dict = {"grid_res": grid_res, "workers": workers, "no_side_flip": no_side_flip}

    t0 = time.perf_counter()
    bsp = BSPReader(bsp_path)
    bsp.read()
    face_sideids = bsp.read_face_side_ids()
    if not face_sideids or not any(face_sideids):
        raise WalkerError(f"{bsp_path.name} has no side-id lumps (22/23); compile with vbsp -emitsideids")
    readers, loaded, missing = load_vpk_readers(game_dir)
    stats["load_s"] = round(time.perf_counter() - t0, 3)
    stats["vpks_loaded"] = len(loaded)
    stats["vpks_missing"] = missing

    t0 = time.perf_counter()
    world = CollisionWorld(bsp, vpk_readers=readers, verbose=True)
    stats["collision_s"] = round(time.perf_counter() - t0, 3)

    t0 = time.perf_counter()
    rmap = ReachabilityMap(world, grid_res=grid_res, verbose=True)
    reach_count = rmap.run()
    stats["reach_s"] = round(time.perf_counter() - t0, 3)
    stats["reachable_cells"] = reach_count
    if reach_count == 0:
        raise WalkerError("no reachable cells (no spawn point, or the spawn is inside solid)")
    eyes = rmap.get_eye_positions()
    stats["eye_positions"] = len(eyes)

    t0 = time.perf_counter()
    oracle = VisibilityOracle(bsp, world, eyes, verbose=True)
    if no_side_flip:
        for face in oracle.faces:          # the oracle's own copies; the BSP file is untouched
            face.side = 0
    results = oracle.classify_faces(num_workers=workers)
    stats["oracle_s"] = round(time.perf_counter() - t0, 3)

    num_faces = len(face_sideids)
    never_visible = sorted(fi for fi, res in results.items() if not res.get("visible", True))
    faces = [{"face": fi, "sideids": list(face_sideids[fi]), "tier": "B", "reason": "python-ray"}
             for fi in never_visible if fi < num_faces]
    stats.update({
        "faces_total": num_faces,
        "faces_classified": len(results),
        "faces_skipped_by_oracle": num_faces - len(results),
        "faces_never_visible": len(never_visible),
        "total_s": round(time.perf_counter() - t_start, 3),
    })
    return {"walker": "python-noflip" if no_side_flip else "python", "faces": faces, "stats": stats}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Python walker baseline for bspwalk")
    ap.add_argument("--bsp", required=True, type=Path, help="compiled BSP (vbsp -emitsideids, then vvis)")
    ap.add_argument("--vmf", type=Path, help="source VMF (informational only)")
    ap.add_argument("--game", required=True, type=Path, help="game directory containing gameinfo.txt")
    ap.add_argument("--out", required=True, type=Path, help="JSON output path")
    ap.add_argument("--workers", type=int, default=1,
                    help="oracle worker processes (default 1: fixtures are tiny and Windows spawn is slow; 0 = auto)")
    ap.add_argument("--grid-res", type=float, default=16.0, help="reachability grid size in units")
    ap.add_argument("--no-side-flip", action="store_true",
                    help="diagnostic: do not negate face normals by dface_t.side (see module docstring)")
    args = ap.parse_args(argv)

    if not args.bsp.is_file():
        print(f"ERROR: BSP not found: {args.bsp}", file=sys.stderr)
        return 2
    if not (args.game / "gameinfo.txt").is_file():
        print(f"ERROR: no gameinfo.txt in {args.game}", file=sys.stderr)
        return 2
    try:
        result = run(args.bsp, args.game, args.workers, args.grid_res, args.no_side_flip)
    except WalkerError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 3
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with open(args.out, "w", encoding="utf-8", newline="\n") as f:
        json.dump(result, f, indent=1)
        f.write("\n")
    s = result["stats"]
    print(f"walker_python: {s['faces_never_visible']} never-visible of {s['faces_total']} faces "
          f"({s['faces_classified']} classified) in {s['total_s']}s -> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
