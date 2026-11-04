"""
bspwalk_runner -- run bspwalk.exe on a compiled BSP and read its face tiers (#15).

    nv_faces, result = run_bspwalk(bsp_path, game_dir)

bspwalk (docs/bsp_walker_rebuild.md) classifies every BSP face as tier A (PVS-proven hidden),
tier B (ray-sampled hidden) or visible.  This returns the set of BSP face indices whose tier is
in `tiers` (default both: the #16 proposal gives both tiers max lightmap scale) and the parsed
JSON.  The BSP needs VVIS data for tier A and `vbsp -emitsideids` for side ids (not needed here:
callers map BSP faces to VMF sides themselves).

Raises BspwalkError when the exe is missing, exits non-zero, or writes no usable JSON, so a caller
never mistakes a failed run for "every face visible".
"""
import json
import os
import subprocess
import tempfile
from pathlib import Path
from typing import Iterable, Optional, Sequence, Set, Tuple

_THIS_DIR = Path(__file__).resolve().parent
DEFAULT_EXE = _THIS_DIR / "bspwalk.exe"


class BspwalkError(RuntimeError):
    pass


def find_bspwalk(explicit: Optional[str] = None) -> Optional[Path]:
    """The bspwalk.exe to use: an explicit path, else the one next to this script."""
    if explicit:
        p = Path(explicit).resolve()
        return p if p.is_file() else None
    return DEFAULT_EXE if DEFAULT_EXE.is_file() else None


def run_bspwalk(bsp_path, game_dir, exe: Optional[Path] = None, tiers: Iterable[str] = ("A", "B"),
                extra_args: Sequence[str] = (), timeout: Optional[float] = None,
                verbose: bool = False) -> Tuple[Set[int], dict]:
    exe = Path(exe) if exe else find_bspwalk()
    if not exe or not exe.is_file():
        raise BspwalkError(f"bspwalk.exe not found ({exe or DEFAULT_EXE})")
    bsp_path = Path(bsp_path).resolve()
    game_dir = Path(game_dir).resolve()
    want = {t.strip().upper() for t in tiers}
    bad = want - {"A", "B"}
    if bad:
        raise BspwalkError(f"unknown tier(s) {sorted(bad)}; use A and/or B")
    fd, out = tempfile.mkstemp(prefix="bspwalk_", suffix=".json")
    os.close(fd)
    try:
        # -game always (AGENTS.md); -reachslim: the node vertices are not needed here
        cmd = [str(exe), str(bsp_path), "-game", str(game_dir), "-reachslim", "-out", out] + list(extra_args)
        if verbose:
            print("  bspwalk:", " ".join(cmd), flush=True)
        proc = subprocess.run(cmd, cwd=str(exe.parent), capture_output=True, text=True, timeout=timeout)
        if verbose and proc.stdout:
            print(proc.stdout, flush=True)
        if proc.returncode != 0:
            tail = "\n".join((proc.stdout + proc.stderr).strip().splitlines()[-15:])
            raise BspwalkError(f"bspwalk exited {proc.returncode}:\n{tail}")
        try:
            with open(out, encoding="utf-8") as fp:
                result = json.load(fp)
        except (OSError, ValueError) as e:
            raise BspwalkError(f"bspwalk wrote no readable JSON: {e}") from e
    finally:
        try:
            os.remove(out)
        except OSError:
            pass
    faces = result.get("faces")
    if not isinstance(faces, list) or not faces:
        raise BspwalkError("bspwalk JSON has no face tiers (was it run with -novis?)")
    hidden = {int(f["face"]) for f in faces if f.get("tier") in want}
    return hidden, result


def summary(result: dict) -> str:
    vis = result.get("vis", {})
    reach = result.get("reach", {})
    return (f"bspwalk: {vis.get('tier_a', '?')} tier A, {vis.get('tier_b_hidden', '?')} tier B, "
            f"{vis.get('visible', '?')} visible of {vis.get('faces', '?')} faces "
            f"(PVS {'yes' if vis.get('pvs') else 'no'}, rays on the {vis.get('path', '?')}); "
            f"{reach.get('reached_count', '?')} of {reach.get('node_count', '?')} walkable polygons reached; "
            f"{result.get('timings_s', {}).get('total', 0):.1f} s")
