#!/usr/bin/env python3
"""
run_bspwalk_tests -- compile the bspwalk fixture maps and score a walker on them.

    python run_bspwalk_tests.py                          # all fixtures, game/bin/x64/bspwalk.exe
    python run_bspwalk_tests.py --fixtures f01,f03       # a subset (id prefix or full name)
    python run_bspwalk_tests.py --walker path/to/bspwalk.exe --tierb-threshold 0
    python run_bspwalk_tests.py --regen                  # rerun make_fixtures first
    python run_bspwalk_tests.py --skip-compile           # reuse BSPs from the last run (ERROR for a
                                                         # fixture whose VMF changed since its compile)
    python run_bspwalk_tests.py --repeat 10              # compile once, run the walker 10x per fixture to
                                                         # catch an intermittent crash (flaky = the runs ended
                                                         # in different statuses)

Pipeline per fixture (all under unit_test/unit_test_output/bspwalk/<NAME>/):
    VMF copied in, stale outputs deleted ->
    vbsp_lmo.exe -emitsideids -> vvis_optix.exe (CPU path) ->
    check side-id lumps 22/23 and that every expected side exists ->
    walker -> JSON contract -> score.

Walker contract (see README.md):
    {"walker": str, "faces": [{"face": int, "sideids": [int], "tier": "A"|"B"|"visible",
                               "reason": str}], "stats": {...}}
    Faces absent from the list are visible. bspwalk.exe also writes
    "reach": {"nodes": [{"id", "reached", "plane": [nx,ny,nz,d], "verts": [[x,y,z]..]}..], ...};
    a JSON without it is an ERROR (bspwalk writes it on every walking run; -nowalk drops it).
    "cubemaps": {"list": [{"origin": [x,y,z], "status": "ok"|"solid"|"nodraw", "props": [...], ...}], ...}
    is read only for a fixture with expect.json "cubemaps" (Map.expect_cubemap).
    --walker <path to exe>   runs <exe> <bsp> -game <game> -out <json> (default: game/bin/x64/bspwalk.exe)

Scoring (per expected side; a side's tier is the weakest of the BSP faces that carry it,
visible beating B beating A):
    FAIL   any must_visible side whose tier is A (a proof was claimed, and it is wrong);
           more than --tierb-threshold must_visible sides whose tier is B.
    report must_hidden sides classed visible (missed savings, never fails).
Reach scoring (expect.json must_reach / must_not_reach feet points): a point is on a node when
it lies within 0.5u of the node's polygon in XY and within 2u of its plane in z.
    FAIL   a must_reach point on no reached node; a must_not_reach point on a reached node.
Cubemap scoring (expect.json cubemaps): matched by origin within 1u in the walker's cubemaps.list.
    FAIL   no matching entry; a different status; static prop warnings present or absent against the expectation.
    ERROR  a compile/leak/lump/walker problem (a crash, a missing or malformed "reach"), an
           expected side with no BSP face (fixture defect), or a must_visible and a
           must_hidden side merged into one BSP face (fixture defect).
--repeat N: the fixture is compiled once and the walker run N times; the fixture's status is the
worst of its runs, and a fixture whose runs ended in different statuses is reported as FLAKY
(runs that all fail the same way are "every run bad": a deterministic failure, not a flake).
Exit code: 0 every fixture passed; 1 at least one FAIL and no ERROR; 2 any ERROR, or a usage
error (unknown fixture name, bad arguments; nothing was run).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import time
import traceback
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

THIS_DIR = Path(__file__).resolve().parent
X64_DIR = THIS_DIR.parents[1]                                  # game/bin/x64
for _p in (THIS_DIR, X64_DIR):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from bsp_reader import (BSPReader, LUMP_FACE_SIDEIDS_DATA,        # noqa: E402
                        LUMP_FACE_SIDEIDS_INDEX)
from bspwalk_runner import BspwalkError, describe_exit, split_args  # noqa: E402
from vmfgen import CUBEMAP_STATUSES  # noqa: E402

MAPS_DIR = X64_DIR / "unit_test_maps" / "bspwalk"
OUT_DIR = X64_DIR / "unit_test" / "unit_test_output" / "bspwalk"
LOG_ROOT = X64_DIR / "unit_test" / "unit_test_logs"
VBSP_EXE = X64_DIR / "vbsp_lmo.exe"
VVIS_EXE = X64_DIR / "vvis_optix.exe"
DEFAULT_GAME = r"E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\sourcetest"
TIERS = ("A", "B", "visible")
CUBEMAP_MATCH_TOL = 1.0                                            # lump 42 origins are whole units
STATUS_RANK = {"PASS": 0, "FAIL": 1, "ERROR": 2}                   # a fixture takes its worst run's status
# VBSP drops a prop_static it cannot use yet still exits 0 with a BSP, so the fixture silently
# lacks its prop.  Both messages come from src/utils/vbsp/staticprop.cpp.
PROP_DELETED_RE = re.compile(r"must be used on a dynamic entity|must be compiled with \$staticprop"
                             r"|Error!.*\bDeleted\b")


class FixtureError(Exception):
    """A problem that makes a fixture's score meaningless (reported as ERROR, never a crash)."""


def usage_error(msg: str):
    """Bad arguments: say why and exit 2 (1 is reserved for FAIL)."""
    print(f"usage error: {msg}", file=sys.stderr)
    raise SystemExit(2)


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
def _run(cmd, log_path: Path, timeout: float, cwd: Path) -> subprocess.CompletedProcess:
    """Run a tool, tee its combined output to log_path, never raise on a non-zero exit."""
    env = dict(os.environ, PYTHONIOENCODING="utf-8")
    try:
        proc = subprocess.run([str(c) for c in cmd], capture_output=True, text=True,
                              encoding="utf-8", errors="replace", timeout=timeout,
                              cwd=str(cwd), env=env)
    except subprocess.TimeoutExpired as exc:
        out = (exc.stdout or b"")
        out = out.decode("utf-8", "replace") if isinstance(out, bytes) else out
        log_path.write_text(f"$ {' '.join(map(str, cmd))}\nTIMEOUT after {timeout}s\n{out}", encoding="utf-8")
        raise FixtureError(f"{Path(str(cmd[0])).name} timed out after {timeout:.0f}s") from None
    except FileNotFoundError:
        raise FixtureError(f"cannot execute {cmd[0]}") from None
    log_path.write_text(f"$ {' '.join(map(str, cmd))}\nexit {proc.returncode}\n--- stdout ---\n"
                        f"{proc.stdout}\n--- stderr ---\n{proc.stderr}\n", encoding="utf-8")
    return proc


def load_expect(name: str) -> dict:
    path = MAPS_DIR / f"{name}.expect.json"
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise FixtureError(f"cannot read {path.name}: {exc}") from None
    if not isinstance(data, dict):
        raise FixtureError(f"{path.name}: root is not an object")
    for key in ("must_visible", "must_hidden"):
        if not isinstance(data.get(key), list):
            raise FixtureError(f"{path.name}: '{key}' missing or not a list")
        for i, entry in enumerate(data[key]):
            side = entry.get("side") if isinstance(entry, dict) else None
            if isinstance(side, bool) or not isinstance(side, int):
                raise FixtureError(f"{path.name}: {key}[{i}] needs an integer 'side': {entry!r}")
    for key in ("must_reach", "must_not_reach"):          # optional (older fixtures have none)
        data.setdefault(key, [])
        if not isinstance(data[key], list):
            raise FixtureError(f"{path.name}: '{key}' is not a list")
        for i, entry in enumerate(data[key]):
            pt = entry.get("point") if isinstance(entry, dict) else None
            if (not isinstance(pt, list) or len(pt) != 3
                    or any(isinstance(c, bool) or not isinstance(c, (int, float)) for c in pt)):
                raise FixtureError(f"{path.name}: {key}[{i}] needs a 'point' of three numbers: {entry!r}")
    data.setdefault("cubemaps", [])                       # optional: expect_cubemap verdicts
    if not isinstance(data["cubemaps"], list):
        raise FixtureError(f"{path.name}: 'cubemaps' is not a list")
    for i, entry in enumerate(data["cubemaps"]):
        pt = entry.get("origin") if isinstance(entry, dict) else None
        if (not isinstance(pt, list) or len(pt) != 3
                or any(isinstance(c, bool) or not isinstance(c, (int, float)) for c in pt)
                or entry.get("status") not in CUBEMAP_STATUSES):
            raise FixtureError(f"{path.name}: cubemaps[{i}] needs an 'origin' of three numbers and a 'status' "
                               f"of {CUBEMAP_STATUSES}: {entry!r}")
    return data


def list_fixtures(selection: Optional[str]) -> List[str]:
    names = sorted(p.name[:-len(".expect.json")] for p in MAPS_DIR.glob("*.expect.json"))
    if not selection:
        return names
    wanted = [w.strip() for w in selection.split(",") if w.strip()]
    if not wanted:
        usage_error(f"--fixtures {selection!r} names no fixture")
    picked: List[str] = []
    for want in wanted:
        hits = [n for n in names if n == want or n.startswith(want)]
        if not hits:
            usage_error(f"no fixture matches {want!r}; have: {', '.join(names) or '(none)'}")
        picked.extend(h for h in hits if h not in picked)
    return picked


# ---------------------------------------------------------------------------
# compile
# ---------------------------------------------------------------------------
def file_sha1(path: Path) -> str:
    return hashlib.sha1(path.read_bytes()).hexdigest()


def hash_record(work: Path, name: str) -> Path:
    """Where compile_fixture records the sha1 of the VMF a BSP was built from."""
    return work / f"{name}.vmf.sha1"


def check_bsp_fresh(name: str, work: Path) -> None:
    """--skip-compile guard: the BSP in the work dir must come from the fixture VMF as it is now."""
    vmf_src = MAPS_DIR / f"{name}.vmf"
    if not vmf_src.is_file():
        raise FixtureError(f"{vmf_src.name} not found (run make_fixtures.py or pass --regen)")
    record = hash_record(work, name)
    if not record.is_file():
        raise FixtureError("stale BSP, recompile: no VMF hash record next to the BSP "
                           "(not compiled by this harness, or the compile failed)")
    if record.read_text(encoding="utf-8").strip() != file_sha1(vmf_src):
        raise FixtureError(f"stale BSP, recompile: {vmf_src.name} changed since the BSP was built")


def compile_fixture(name: str, game: str, work: Path, timeout: float) -> dict:
    """VMF -> BSP (vbsp -emitsideids) -> vvis.  Returns timings; raises FixtureError."""
    vmf_src = MAPS_DIR / f"{name}.vmf"
    if not vmf_src.is_file():
        raise FixtureError(f"{vmf_src.name} not found (run make_fixtures.py or pass --regen)")
    for tool in (VBSP_EXE, VVIS_EXE):
        if not tool.is_file():
            raise FixtureError(f"{tool.name} not found in {X64_DIR}")
    work.mkdir(parents=True, exist_ok=True)
    # Delete every stale product first: vbsp_runner judges BSP freshness by mtime, and a
    # stale BSP would silently be scored in place of a failed compile.
    for stale in work.glob(f"{name}.*"):
        if stale.is_file() and stale.suffix.lower() != ".vmf":
            stale.unlink()
    for stale in (work / f"{name}.vbsp.out", work / f"{name}.vvis.out", work / f"{name}.walk.out"):
        stale.unlink(missing_ok=True)
    vmf = work / f"{name}.vmf"
    shutil.copy2(vmf_src, vmf)
    vmf_hash = file_sha1(vmf)
    bsp = work / f"{name}.bsp"

    t0 = time.perf_counter()
    proc = _run([VBSP_EXE, "-emitsideids", "-game", game, vmf], work / f"{name}.vbsp.out", timeout, X64_DIR)
    vbsp_s = time.perf_counter() - t0
    text = proc.stdout + proc.stderr
    if "**** leaked ****" in text or "MAP LEAKED" in text:
        raise FixtureError("MAP LEAKED (vbsp reports a leak; see <name>.vbsp.out and the .lin file)")
    if proc.returncode != 0 or not bsp.is_file():
        tail = " | ".join(text.strip().splitlines()[-3:])
        what = "vbsp produced no BSP" if not bsp.is_file() else "vbsp failed"
        raise FixtureError(f"{what} (exit {proc.returncode}): {tail}")
    deleted = [ln.strip() for ln in text.splitlines() if PROP_DELETED_RE.search(ln)]
    if deleted:
        raise FixtureError(f"vbsp deleted {len(deleted)} static prop(s), so the fixture lacks "
                           f"them: {' | '.join(deleted[:2])}")

    t0 = time.perf_counter()
    proc = _run([VVIS_EXE, "-game", game, bsp], work / f"{name}.vvis.out", timeout, X64_DIR)
    vvis_s = time.perf_counter() - t0
    if proc.returncode != 0:
        tail = " | ".join((proc.stdout + proc.stderr).strip().splitlines()[-3:])
        raise FixtureError(f"vvis failed (exit {proc.returncode}): {tail}")
    hash_record(work, name).write_text(vmf_hash + "\n", encoding="utf-8")   # only after a full compile
    return {"vbsp_s": round(vbsp_s, 2), "vvis_s": round(vvis_s, 2)}


def read_bsp_sideids(bsp_path: Path) -> List[List[int]]:
    """Per-face side-id lists from lumps 22/23; FixtureError if the lumps are missing/empty."""
    if not bsp_path.is_file():
        raise FixtureError(f"{bsp_path.name} does not exist (use --skip-compile only after a compile)")
    bsp = BSPReader(bsp_path)
    bsp.read()
    idx = bsp._get_lump_data(LUMP_FACE_SIDEIDS_INDEX)
    dat = bsp._get_lump_data(LUMP_FACE_SIDEIDS_DATA)
    if not idx or not dat:
        raise FixtureError(f"side-id lumps 22/23 are empty (index {len(idx)} B, data {len(dat)} B); "
                           f"vbsp must run with -emitsideids")
    sideids = bsp.read_face_side_ids()
    if not sideids:
        raise FixtureError("side-id lumps could not be parsed")
    if bsp.read_visibility() is None:
        raise FixtureError("BSP has no visibility lump; vvis did not run")
    return sideids


# ---------------------------------------------------------------------------
# walker
# ---------------------------------------------------------------------------
def run_walker(walker: str, name: str, game: str, work: Path, timeout: float,
               extra: Optional[List[str]] = None, tag: str = "") -> dict:
    """One walker run; `tag` (".r2", ...) keeps the files of a --repeat run apart."""
    extra = extra or []
    bsp, out = work / f"{name}.bsp", work / f"{name}{tag}.walk.json"
    out.unlink(missing_ok=True)
    log = work / f"{name}{tag}.walk.out"
    exe = Path(walker)
    if not exe.is_file():
        raise FixtureError(f"walker executable not found: {walker}")
    cmd = [exe, bsp, "-game", game, "-out", out] + extra
    t0 = time.perf_counter()
    proc = _run(cmd, log, timeout, X64_DIR)
    walker_s = time.perf_counter() - t0
    if proc.returncode != 0:
        tail = " | ".join((proc.stderr or proc.stdout).strip().splitlines()[-2:])
        raise FixtureError(f"walker exited {describe_exit(proc.returncode)}: {tail}")
    if not out.is_file():
        raise FixtureError("walker exited 0 but wrote no JSON")
    try:
        data = json.loads(out.read_text(encoding="utf-8"))
    except ValueError as exc:
        raise FixtureError(f"walker JSON is invalid: {exc}") from None
    if not isinstance(data, dict):
        raise FixtureError(f"walker JSON root must be an object, got {type(data).__name__}")
    data["_wall_s"] = round(walker_s, 2)
    return data


def validate_contract(walk: dict, face_sideids: List[List[int]]) -> Dict[int, str]:
    """Check the walker JSON against the contract; return {face: tier}."""
    if not isinstance(walk.get("walker"), str) or not isinstance(walk.get("faces"), list):
        raise FixtureError("walker JSON needs a string 'walker' and a list 'faces'")
    tiers: Dict[int, str] = {}
    for entry in walk["faces"]:
        try:
            face, tier = entry["face"], entry["tier"]
            if isinstance(face, bool) or not isinstance(face, int) or not isinstance(entry["sideids"], list):
                raise TypeError("face must be an integer and sideids a list")
            sids = entry["sideids"]
            if any(isinstance(s, bool) or not isinstance(s, int) for s in sids):
                raise TypeError("sideids must be integers")
        except (KeyError, TypeError, ValueError):
            raise FixtureError(f"malformed face entry: {entry!r}") from None
        if face in tiers:
            raise FixtureError(f"face {face} is listed more than once")
        if tier not in TIERS:
            raise FixtureError(f"face {face}: tier {tier!r} is not one of {TIERS}")
        if not 0 <= face < len(face_sideids):
            raise FixtureError(f"face {face} is outside the BSP's {len(face_sideids)} faces")
        if sorted(sids) != sorted(face_sideids[face]):
            raise FixtureError(f"face {face}: walker sideids {sids} disagree with the BSP lump "
                               f"{face_sideids[face]}")
        tiers[face] = tier
    return tiers


# ---------------------------------------------------------------------------
# scoring
# ---------------------------------------------------------------------------
def side_tier(faces: List[int], face_tier: Dict[int, str]) -> str:
    """Weakest tier over the faces carrying a side: visible > B > A.  'absent' if no face."""
    if not faces:
        return "absent"
    tiers = {face_tier.get(f, "visible") for f in faces}
    for t in ("visible", "B", "A"):
        if t in tiers:
            return t
    return "visible"


def score(expect: dict, face_sideids: List[List[int]], face_tier: Dict[int, str],
          threshold: int) -> dict:
    faces_of: Dict[int, List[int]] = {}
    for fi, ids in enumerate(face_sideids):
        for sid in ids:
            faces_of.setdefault(sid, []).append(fi)

    vis = {int(e["side"]): e.get("why", "") for e in expect["must_visible"]}
    hid = {int(e["side"]): e.get("why", "") for e in expect["must_hidden"]}
    errors: List[str] = []
    for sid in sorted(set(vis) & set(hid)):
        errors.append(f"side {sid} is both must_visible and must_hidden")
    for kind, group in (("must_visible", vis), ("must_hidden", hid)):
        for sid in sorted(group):
            if sid not in faces_of:
                errors.append(f"{kind} side {sid} has no BSP face (fixture defect: filled solid, "
                              f"culled, or nodraw?): {group[sid]}")
    # A face shared by a must-visible and a must-hidden side cannot be scored.
    for fi, ids in enumerate(face_sideids):
        v = [s for s in ids if s in vis]
        h = [s for s in ids if s in hid]
        if v and h:
            errors.append(f"fixture defect: BSP face {fi} merges must_visible side(s) {v} "
                          f"with must_hidden side(s) {h}")

    counts = {"vis_total": len(vis), "vis_ok": 0, "vis_B": 0, "vis_A": 0,
              "hid_total": len(hid), "hid_A": 0, "hid_B": 0, "hid_missed": 0}
    problems = {"vis_A": [], "vis_B": [], "hid_missed": []}
    for sid in sorted(vis):
        t = side_tier(faces_of.get(sid, []), face_tier)
        if t == "visible":
            counts["vis_ok"] += 1
        elif t == "B":
            counts["vis_B"] += 1
            problems["vis_B"].append({"side": sid, "why": vis[sid]})
        elif t == "A":
            counts["vis_A"] += 1
            problems["vis_A"].append({"side": sid, "why": vis[sid]})
    for sid in sorted(hid):
        t = side_tier(faces_of.get(sid, []), face_tier)
        if t == "A":
            counts["hid_A"] += 1
        elif t == "B":
            counts["hid_B"] += 1
        elif t == "visible":
            counts["hid_missed"] += 1
            problems["hid_missed"].append({"side": sid, "why": hid[sid]})

    if errors:
        status = "ERROR"
    elif counts["vis_A"] > 0 or counts["vis_B"] > threshold:
        status = "FAIL"
    else:
        status = "PASS"
    return {"status": status, "counts": counts, "problems": problems, "errors": errors}


# Reach tolerances: a feet point is on a reached polygon when it lies within REACH_XY_TOL of the
# polygon in XY and within REACH_Z_TOL of its plane there.  The fixtures keep every reach
# expectation tens of units from the nearest wrong answer, so these never decide a result.
REACH_XY_TOL = 0.5
REACH_Z_TOL = 2.0


def _xy_distance_to_polygon(x: float, y: float, verts: List[List[float]]) -> float:
    """Distance in XY from (x, y) to a convex polygon (0 inside), any winding."""
    n = len(verts)
    area2 = sum(verts[i][0] * verts[(i + 1) % n][1] - verts[(i + 1) % n][0] * verts[i][1] for i in range(n))
    sign = 1.0 if area2 >= 0 else -1.0
    inside = True
    best = float("inf")
    for i in range(n):
        ax, ay = verts[i][0], verts[i][1]
        bx, by = verts[(i + 1) % n][0], verts[(i + 1) % n][1]
        ex, ey = bx - ax, by - ay
        if sign * (ex * (y - ay) - ey * (x - ax)) < 0:
            inside = False
        ll = ex * ex + ey * ey
        t = 0.0 if ll == 0 else max(0.0, min(1.0, ((x - ax) * ex + (y - ay) * ey) / ll))
        dx, dy = x - (ax + t * ex), y - (ay + t * ey)
        best = min(best, (dx * dx + dy * dy) ** 0.5)
    return 0.0 if inside else best


def reached_node_at(point: List[float], nodes: List[dict]) -> Optional[int]:
    """Id of a reached node whose polygon carries the feet point (README tolerances), else None."""
    x, y, z = point
    for node in nodes:
        if not node.get("reached"):
            continue
        nx, ny, nz, d = node["plane"]
        if nz <= 0 or abs((d - nx * x - ny * y) / nz - z) > REACH_Z_TOL:
            continue
        if _xy_distance_to_polygon(x, y, node["verts"]) <= REACH_XY_TOL:
            return node["id"]
    return None


def score_reach(expect: dict, walk: dict) -> dict:
    """Score must_reach / must_not_reach against the walker's 'reach' object.

    The walker JSON must carry a 'reach' object, even for a fixture with no reach expectations:
    bspwalk writes one on every walking run, so its absence (-nowalk, an old exe, a regression)
    must not let the reach half of the suite pass unscored.  Returns status 'PASS' or 'FAIL'.
    """
    want, avoid = expect.get("must_reach", []), expect.get("must_not_reach", [])
    res = {"status": "PASS", "reach_total": len(want), "reach_ok": 0, "no_reach_total": len(avoid),
           "no_reach_ok": 0, "problems": []}
    reach = walk.get("reach")
    if reach is None:
        raise FixtureError("walker JSON has no 'reach' object (bspwalk writes one on every walking run; "
                           "was it run with -nowalk, or is this an old exe?)")
    nodes = reach.get("nodes") if isinstance(reach, dict) else None
    if not isinstance(nodes, list):
        raise FixtureError("walker 'reach' needs a list 'nodes'")
    def number(c) -> bool:
        return isinstance(c, (int, float)) and not isinstance(c, bool)

    for i, node in enumerate(nodes):
        try:
            ok = (isinstance(node, dict) and isinstance(node["id"], int) and not isinstance(node["id"], bool)
                  and isinstance(node["reached"], bool)
                  and isinstance(node["plane"], list) and len(node["plane"]) == 4
                  and all(number(c) for c in node["plane"])
                  and isinstance(node["verts"], list) and len(node["verts"]) >= 3
                  and all(isinstance(v, list) and len(v) == 3 and all(number(c) for c in v)
                          for v in node["verts"]))
        except (KeyError, TypeError):
            ok = False
        if not ok:
            raise FixtureError(f"malformed reach node {i}: needs id, reached, plane [nx, ny, nz, d], verts")
    for e in want:
        if reached_node_at(e["point"], nodes) is None:
            res["problems"].append({"kind": "missed must_reach", "point": e["point"], "why": e.get("why", "")})
        else:
            res["reach_ok"] += 1
    for e in avoid:
        hit = reached_node_at(e["point"], nodes)
        if hit is None:
            res["no_reach_ok"] += 1
        else:
            res["problems"].append({"kind": f"reached must_not_reach (node {hit})", "point": e["point"],
                                    "why": e.get("why", "")})
    res["status"] = "FAIL" if res["problems"] else "PASS"
    return res


def score_cubemaps(expect: dict, walk: dict) -> dict:
    """Score expect.json 'cubemaps' against the walker's cubemaps.list.

    An expectation FAILs when the walker has no cubemap within CUBEMAP_MATCH_TOL of its origin
    (an old exe, a run without eyes, or a cubemap the walker lost), when the entry's status
    differs, or when the walker's static prop warnings (a non-empty 'props') differ from the
    expected presence.  A fixture with no cubemap expectation needs nothing from the walker.
    """
    want = expect.get("cubemaps", [])
    res = {"status": "PASS", "total": len(want), "ok": 0, "problems": []}
    block = walk.get("cubemaps")
    entries = block.get("list") if isinstance(block, dict) else None
    if not isinstance(entries, list):
        entries = []

    def origin_of(entry):
        o = entry.get("origin") if isinstance(entry, dict) else None
        ok = isinstance(o, list) and len(o) == 3 and all(
            isinstance(c, (int, float)) and not isinstance(c, bool) for c in o)
        return o if ok else None

    for e in want:
        pt = e["origin"]
        near = [(max(abs(a - b) for a, b in zip(origin_of(c), pt)), c) for c in entries if origin_of(c)]
        near = [c for dist, c in sorted(near, key=lambda t: t[0]) if dist <= CUBEMAP_MATCH_TOL]
        problem = None
        if not near:
            problem = "the walker JSON has no cubemap entry there"
        else:
            got = near[0]
            got_props = bool(got.get("props"))
            if got.get("status") != e["status"]:
                problem = f"expected status {e['status']!r}, walker says {got.get('status')!r} ({got.get('reason', '')})"
            elif got_props != bool(e.get("props", False)):
                problem = f"expected {'a' if e.get('props') else 'no'} static prop warning, walker has {len(got.get('props') or [])}"
        if problem:
            res["problems"].append({"origin": pt, "problem": problem, "why": e.get("why", "")})
        else:
            res["ok"] += 1
    res["status"] = "FAIL" if res["problems"] else "PASS"
    return res


# ---------------------------------------------------------------------------
# one fixture end to end
# ---------------------------------------------------------------------------
def score_walk(name: str, args, work: Path, expect: dict, face_sideids: List[List[int]], run_no: int) -> dict:
    """One walker run on the compiled BSP, scored.  Never raises: a problem is an ERROR result."""
    res = {"status": "ERROR", "errors": [], "counts": {}, "problems": {}, "timings": {}, "walker_stats": {}}
    try:
        walk = run_walker(args.walker, name, args.game, work, args.timeout, args.walker_extra,
                          tag="" if run_no == 1 else f".r{run_no}")
        res["timings"]["walker_s"] = walk.pop("_wall_s")
        stats = walk.get("stats")
        res["walker_stats"] = stats if isinstance(stats, dict) else {}
        face_tier = validate_contract(walk, face_sideids)
        res["walker_faces_reported"] = len(face_tier)
        res.update(score(expect, face_sideids, face_tier, args.tierb_threshold))
        res["reach"] = score_reach(expect, walk)
        if res["reach"]["status"] == "FAIL" and res["status"] == "PASS":
            res["status"] = "FAIL"
        res["cubemaps"] = score_cubemaps(expect, walk)
        if res["cubemaps"]["status"] == "FAIL" and res["status"] == "PASS":
            res["status"] = "FAIL"
    except FixtureError as exc:
        res["status"] = "ERROR"
        res["errors"].append(str(exc))
    except Exception as exc:                       # noqa: BLE001 -- one run must never sink the suite
        res["status"] = "ERROR"
        res["errors"].append(f"unexpected {type(exc).__name__}: {exc}")
        res["traceback"] = traceback.format_exc()
    return res


def run_fixture(name: str, args) -> dict:
    res = {"fixture": name, "status": "ERROR", "errors": [], "counts": {}, "problems": {},
           "timings": {}, "walker_stats": {}}
    work = OUT_DIR / name
    t_all = time.perf_counter()
    try:
        expect = load_expect(name)
        if args.skip_compile:
            check_bsp_fresh(name, work)
        else:
            res["timings"].update(compile_fixture(name, args.game, work, args.timeout))
        face_sideids = read_bsp_sideids(work / f"{name}.bsp")
        res["faces_in_bsp"] = len(face_sideids)
        for stale in work.glob(f"{name}.r[0-9]*.walk.*"):      # files of an earlier, longer --repeat
            stale.unlink()
        runs = [score_walk(name, args, work, expect, face_sideids, i) for i in range(1, args.repeat + 1)]
        worst = max(runs, key=lambda r: STATUS_RANK[r["status"]])          # the first run of the worst status
        for key, val in worst.items():
            if key == "timings":
                res["timings"].update(val)
            else:
                res[key] = val
        if args.repeat > 1:
            res["runs"] = [{"run": i, "status": r["status"], "error": "; ".join(r["errors"])[:300]}
                           for i, r in enumerate(runs, 1)]
            res["flaky"] = len({r["status"] for r in runs}) > 1
    except FixtureError as exc:
        res["status"] = "ERROR"
        res["errors"].append(str(exc))
    except Exception as exc:                       # noqa: BLE001 -- one fixture must never sink the run
        res["status"] = "ERROR"
        res["errors"].append(f"unexpected {type(exc).__name__}: {exc}")
        res["traceback"] = traceback.format_exc()
    res["timings"]["total_s"] = round(time.perf_counter() - t_all, 2)
    return res


# ---------------------------------------------------------------------------
# reporting
# ---------------------------------------------------------------------------
def print_report(results: List[dict], threshold: int) -> None:
    hdr = (f"{'fixture':<26}{'status':<7}| {'vis ok':>6} {'B':>3} {'A':>3} | "
           f"{'hid A':>5} {'B':>3} {'miss':>4} | {'reach':>5} {'noreach':>7} | {'walker s':>8} {'total s':>8}")
    print()
    print("must_visible: ok = seen, B = tier B (counts against --tierb-threshold), A = FAIL")
    print("must_hidden : A / B = proven or sampled hidden (good), miss = classed visible (savings missed)")
    print("reach       : must_reach on a reached polygon / must_not_reach on none (ok/total; any miss = FAIL)")
    print(hdr)
    print("-" * len(hdr))
    for r in results:
        c = r.get("counts") or {}
        if c:
            row = (f"{c['vis_ok']:>6} {c['vis_B']:>3} {c['vis_A']:>3} | "
                   f"{c['hid_A']:>5} {c['hid_B']:>3} {c['hid_missed']:>4}")
        else:
            row = f"{'-':>6} {'-':>3} {'-':>3} | {'-':>5} {'-':>3} {'-':>4}"
        rc = r.get("reach") or {}
        if rc.get("status") in ("PASS", "FAIL"):
            reach_col = (f"{str(rc['reach_ok']) + '/' + str(rc['reach_total']):>5} "
                         f"{str(rc['no_reach_ok']) + '/' + str(rc['no_reach_total']):>7}")
        else:
            reach_col = f"{'-':>5} {'-':>7}"
        t = r["timings"]
        print(f"{r['fixture']:<26}{r['status']:<7}| {row} | {reach_col} | "
              f"{t.get('walker_s', float('nan')):>8.1f} {t.get('total_s', float('nan')):>8.1f}")
    print()
    for r in results:
        notes = []
        for e in r.get("errors", []):
            notes.append(f"  ERROR: {e}")
        p = r.get("problems") or {}
        for key, label in (("vis_A", "FAIL must_visible tier A"), ("vis_B", "must_visible tier B"),
                           ("hid_missed", "missed (must_hidden seen as visible)")):
            for item in p.get(key, [])[:6]:
                notes.append(f"  {label}: side {item['side']}: {item['why']}")
            if len(p.get(key, [])) > 6:
                notes.append(f"  {label}: ... and {len(p[key]) - 6} more")
        for item in (r.get("reach") or {}).get("problems", []):
            pt = ", ".join(f"{c:g}" for c in item["point"])
            notes.append(f"  FAIL {item['kind']}: ({pt}): {item['why']}")
        for item in (r.get("cubemaps") or {}).get("problems", []):
            pt = ", ".join(f"{c:g}" for c in item["origin"])
            notes.append(f"  FAIL cubemap ({pt}): {item['problem']}: {item['why']}")
        runs = r.get("runs") or []
        bad_runs = [x for x in runs if x["status"] != "PASS"]
        if bad_runs:
            tally = ", ".join(f"{n} {st}" for st in STATUS_RANK
                              for n in [sum(x["status"] == st for x in runs)] if n)
            notes.append(f"  {'FLAKY' if r.get('flaky') else 'every run bad'}: "
                         f"{len(runs)} runs: {tally}")
            for x in bad_runs[:6]:
                notes.append(f"  run {x['run']}: {x['status']} {x['error']}".rstrip())
            if len(bad_runs) > 6:
                notes.append(f"  ... and {len(bad_runs) - 6} more bad run(s)")
        if notes:
            print(f"{r['fixture']}")
            print("\n".join(notes))
    tot = {k: sum((r.get("counts") or {}).get(k, 0) for r in results)
           for k in ("vis_total", "vis_ok", "vis_B", "vis_A", "hid_total", "hid_A", "hid_B", "hid_missed")}
    scored = [r["reach"] for r in results if (r.get("reach") or {}).get("status") in ("PASS", "FAIL")]
    n_runs = max((len(r.get("runs") or []) for r in results), default=0)
    rt = {k: sum(x[k] for x in scored) for k in ("reach_ok", "reach_total", "no_reach_ok", "no_reach_total")}
    n_err = sum(r["status"] == "ERROR" for r in results)
    n_fail = sum(r["status"] == "FAIL" for r in results)
    n_pass = sum(r["status"] == "PASS" for r in results)
    reach_txt = (f"must_reach {rt['reach_ok']}/{rt['reach_total']} reached, must_not_reach "
                 f"{rt['no_reach_ok']}/{rt['no_reach_total']} unreached ({len(scored)} fixture(s) scored)")
    print(f"\nREACH   {reach_txt}")
    cm = [r["cubemaps"] for r in results if (r.get("cubemaps") or {}).get("total")]
    if cm:
        print(f"CUBEMAP {sum(x['ok'] for x in cm)}/{sum(x['total'] for x in cm)} cubemap verdicts as expected "
              f"({len(cm)} fixture(s) scored)")
    if n_runs:
        bad = [f"{r['fixture']} ({sum(x['status'] != 'PASS' for x in r['runs'])}/{len(r['runs'])} bad"
               f"{', FLAKY' if r.get('flaky') else ''})"
               for r in results if any(x["status"] != "PASS" for x in r.get("runs") or [])]
        print(f"REPEAT  {n_runs} walker run(s) per fixture; "
              f"{'every run of every fixture passed' if not bad else 'bad runs in: ' + ', '.join(bad)}")
    print(f"\nTOTALS  fixtures {len(results)}: {n_pass} pass, {n_fail} fail, {n_err} error | "
          f"must_visible {tot['vis_total']}: {tot['vis_ok']} seen, {tot['vis_B']} tier B "
          f"(threshold {threshold}), {tot['vis_A']} tier A | "
          f"must_hidden {tot['hid_total']}: {tot['hid_A'] + tot['hid_B']} hidden "
          f"({tot['hid_A']} A, {tot['hid_B']} B), {tot['hid_missed']} missed")


def archive_logs(results: List[dict], summary: dict) -> Path:
    stamp = datetime.now().strftime("%Y-%m-%dT%H-%M-%S")
    dest = LOG_ROOT / f"{stamp}_bspwalk"
    n = 1
    while dest.exists():  # two runs in the same second must not share an archive
        n += 1
        dest = LOG_ROOT / f"{stamp}_bspwalk_{n}"
    dest.mkdir(parents=True)
    for r in results:
        work = OUT_DIR / r["fixture"]
        for pat in ("*.out", "*.log", "*.walk.json"):
            for f in work.glob(pat):
                shutil.copy2(f, dest / f.name)
    (dest / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return dest


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Score a walker on the bspwalk fixture maps.",
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__.split("Pipeline")[0])
    ap.add_argument("--fixtures", help="comma separated fixture ids/names (default: every expect.json)")
    ap.add_argument("--walker", default=str(X64_DIR / "bspwalk.exe"), help="path to the walker executable (default: game/bin/x64/bspwalk.exe)")
    ap.add_argument("--game", default=DEFAULT_GAME, help="game directory containing gameinfo.txt")
    ap.add_argument("--tierb-threshold", type=int, default=0,
                    help="max must_visible sides allowed in tier B before a fixture FAILs (default 0)")
    ap.add_argument("--timeout", type=float, default=900, help="seconds per tool run (default 900)")
    ap.add_argument("--regen", action="store_true", help="rerun make_fixtures.py first")
    ap.add_argument("--skip-compile", action="store_true", help="reuse the BSPs from the previous run")
    ap.add_argument("--jobs", type=int, default=4, help="fixtures processed in parallel (default 4)")
    ap.add_argument("--repeat", type=int, default=1, metavar="N",
                    help="run the walker N times per fixture (compiled once) to catch an intermittent "
                         "crash; the fixture takes its worst run's status and is FLAKY when its runs "
                         "ended in different statuses (default 1)")
    ap.add_argument("--walker-args", default="",
                    help="extra arguments appended to the walker command line, e.g. \"--no-side-flip\"")
    args = ap.parse_args(argv)                     # bad options exit 2 via argparse
    if args.repeat < 1:
        usage_error(f"--repeat must be at least 1, got {args.repeat}")

    # Tools run with cwd=game/bin/x64, so a relative path from the caller's cwd would break.
    args.game = str(Path(args.game).expanduser().resolve())
    args.walker = str(Path(args.walker).expanduser().resolve())
    if not Path(args.walker).is_file():
        usage_error(f"walker executable not found: {args.walker}")
    try:
        args.walker_extra = split_args(args.walker_args)
    except BspwalkError as exc:
        usage_error(f"--walker-args: {exc}")

    if args.regen:
        import make_fixtures
        rc = make_fixtures.main(["--only", args.fixtures] if args.fixtures else [])
        if rc:
            return rc
    names = list_fixtures(args.fixtures)
    if not names:
        print(f"No expect.json files in {MAPS_DIR}; run make_fixtures.py or pass --regen.", file=sys.stderr)
        return 2
    if not (Path(args.game) / "gameinfo.txt").is_file():
        print(f"ERROR: no gameinfo.txt in {args.game}", file=sys.stderr)
        return 2

    print(f"bspwalk tests: {len(names)} fixture(s), walker={args.walker}, "
          f"tierb-threshold={args.tierb_threshold}, jobs={args.jobs}, repeat={args.repeat}")
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    t0 = time.perf_counter()

    def job(name):
        r = run_fixture(name, args)
        bad = sum(x["status"] != "PASS" for x in r.get("runs") or [])
        print(f"  finished {name}: {r['status']} in {r['timings']['total_s']}s"
              + (f" ({bad}/{args.repeat} runs not PASS)" if bad else ""), flush=True)
        return r

    with ThreadPoolExecutor(max_workers=max(1, min(args.jobs, len(names)))) as pool:
        results = list(pool.map(job, names))
    wall = time.perf_counter() - t0

    n_err = sum(r["status"] == "ERROR" for r in results)
    n_fail = sum(r["status"] == "FAIL" for r in results)
    exit_code = 2 if n_err else (1 if n_fail else 0)
    summary = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "walker": args.walker, "game": args.game, "tierb_threshold": args.tierb_threshold,
        "repeat": args.repeat,
        "wall_s": round(wall, 1), "exit_code": exit_code, "fixtures": results,
    }
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    (OUT_DIR / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print_report(results, args.tierb_threshold)       # after the summary, so a report bug cannot lose it
    dest = archive_logs(results, summary)
    print(f"wall time {wall:.1f}s | summary: {OUT_DIR / 'summary.json'} | logs: {dest}")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
