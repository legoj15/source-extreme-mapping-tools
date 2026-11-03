#!/usr/bin/env python3
"""
run_bspwalk_tests -- compile the bspwalk fixture maps and score a walker on them.

    python run_bspwalk_tests.py                          # all fixtures, Python walker
    python run_bspwalk_tests.py --fixtures f01,f03       # a subset (id prefix or full name)
    python run_bspwalk_tests.py --walker path/to/bspwalk.exe --tierb-threshold 0
    python run_bspwalk_tests.py --regen                  # rerun make_fixtures first
    python run_bspwalk_tests.py --skip-compile           # reuse BSPs from the last run (ERROR for a
                                                         # fixture whose VMF changed since its compile)

Pipeline per fixture (all under unit_test/unit_test_output/bspwalk/<NAME>/):
    VMF copied in, stale outputs deleted ->
    vbsp_lmo.exe -emitsideids -> vvis_optix.exe (CPU path) ->
    check side-id lumps 22/23 and that every expected side exists ->
    walker -> JSON contract -> score.

Walker contract (see README.md):
    {"walker": str, "faces": [{"face": int, "sideids": [int], "tier": "A"|"B"|"visible",
                               "reason": str}], "stats": {...}}
    Faces absent from the list are visible.
    --walker python          runs walker_python.py --bsp B --vmf V --game G --out J
    --walker <path to exe>   runs <exe> <bsp> -game <game> -out <json>   (placeholder contract)

Scoring (per expected side; a side's tier is the weakest of the BSP faces that carry it,
visible beating B beating A):
    FAIL   any must_visible side whose tier is A (a proof was claimed, and it is wrong);
           more than --tierb-threshold must_visible sides whose tier is B.
    report must_hidden sides classed visible (missed savings, never fails).
    ERROR  a compile/leak/lump/walker problem, an expected side with no BSP face
           (fixture defect), or a must_visible and a must_hidden side merged into one
           BSP face (fixture defect).
Exit code: 0 every fixture passed; 1 at least one FAIL and no ERROR; 2 any ERROR, or a usage
error (unknown fixture name, bad arguments; nothing was run).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shlex
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

MAPS_DIR = X64_DIR / "unit_test_maps" / "bspwalk"
OUT_DIR = X64_DIR / "unit_test" / "unit_test_output" / "bspwalk"
LOG_ROOT = X64_DIR / "unit_test" / "unit_test_logs"
VBSP_EXE = X64_DIR / "vbsp_lmo.exe"
VVIS_EXE = X64_DIR / "vvis_optix.exe"
DEFAULT_GAME = r"E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\sourcetest"
TIERS = ("A", "B", "visible")
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
               extra: Optional[List[str]] = None) -> dict:
    extra = extra or []
    bsp, vmf, out = work / f"{name}.bsp", work / f"{name}.vmf", work / f"{name}.walk.json"
    out.unlink(missing_ok=True)
    log = work / f"{name}.walk.out"
    if walker.lower() == "python":
        cmd = [sys.executable, THIS_DIR / "walker_python.py", "--bsp", bsp, "--vmf", vmf,
               "--game", game, "--out", out, "--workers", "1"] + extra
    else:
        exe = Path(walker)
        if not exe.is_file():
            raise FixtureError(f"walker executable not found: {walker}")
        cmd = [exe, bsp, "-game", game, "-out", out] + extra
    t0 = time.perf_counter()
    proc = _run(cmd, log, timeout, X64_DIR)
    walker_s = time.perf_counter() - t0
    if proc.returncode != 0:
        tail = " | ".join((proc.stderr or proc.stdout).strip().splitlines()[-2:])
        raise FixtureError(f"walker exited {proc.returncode}: {tail}")
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
            sids = [int(s) for s in entry["sideids"]]
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


# ---------------------------------------------------------------------------
# one fixture end to end
# ---------------------------------------------------------------------------
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
        walk = run_walker(args.walker, name, args.game, work, args.timeout, args.walker_extra)
        res["timings"]["walker_s"] = walk.pop("_wall_s")
        stats = walk.get("stats")
        res["walker_stats"] = stats if isinstance(stats, dict) else {}
        face_tier = validate_contract(walk, face_sideids)
        res["walker_faces_reported"] = len(face_tier)
        res.update(score(expect, face_sideids, face_tier, args.tierb_threshold))
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
           f"{'hid A':>5} {'B':>3} {'miss':>4} | {'walker s':>8} {'total s':>8}")
    print()
    print("must_visible: ok = seen, B = tier B (counts against --tierb-threshold), A = FAIL")
    print("must_hidden : A / B = proven or sampled hidden (good), miss = classed visible (savings missed)")
    print(hdr)
    print("-" * len(hdr))
    for r in results:
        c = r.get("counts") or {}
        if c:
            row = (f"{c['vis_ok']:>6} {c['vis_B']:>3} {c['vis_A']:>3} | "
                   f"{c['hid_A']:>5} {c['hid_B']:>3} {c['hid_missed']:>4}")
        else:
            row = f"{'-':>6} {'-':>3} {'-':>3} | {'-':>5} {'-':>3} {'-':>4}"
        t = r["timings"]
        print(f"{r['fixture']:<26}{r['status']:<7}| {row} | "
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
        if notes:
            print(f"{r['fixture']}")
            print("\n".join(notes))
    tot = {k: sum((r.get("counts") or {}).get(k, 0) for r in results)
           for k in ("vis_total", "vis_ok", "vis_B", "vis_A", "hid_total", "hid_A", "hid_B", "hid_missed")}
    n_err = sum(r["status"] == "ERROR" for r in results)
    n_fail = sum(r["status"] == "FAIL" for r in results)
    n_pass = sum(r["status"] == "PASS" for r in results)
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
    ap.add_argument("--walker", default="python", help="'python' or the path to a walker executable")
    ap.add_argument("--game", default=DEFAULT_GAME, help="game directory containing gameinfo.txt")
    ap.add_argument("--tierb-threshold", type=int, default=0,
                    help="max must_visible sides allowed in tier B before a fixture FAILs (default 0)")
    ap.add_argument("--timeout", type=float, default=900, help="seconds per tool run (default 900)")
    ap.add_argument("--regen", action="store_true", help="rerun make_fixtures.py first")
    ap.add_argument("--skip-compile", action="store_true", help="reuse the BSPs from the previous run")
    ap.add_argument("--jobs", type=int, default=4, help="fixtures processed in parallel (default 4)")
    ap.add_argument("--walker-args", default="",
                    help="extra arguments appended to the walker command line, e.g. \"--no-side-flip\"")
    args = ap.parse_args(argv)                     # bad options exit 2 via argparse

    # Tools run with cwd=game/bin/x64, so a relative path from the caller's cwd would break.
    args.game = str(Path(args.game).expanduser().resolve())
    if args.walker.lower() != "python":
        args.walker = str(Path(args.walker).expanduser().resolve())
        if not Path(args.walker).is_file():
            usage_error(f"walker executable not found: {args.walker}")
    try:
        args.walker_extra = shlex.split(args.walker_args, posix=(os.name != "nt"))
    except ValueError as exc:
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
          f"tierb-threshold={args.tierb_threshold}, jobs={args.jobs}")
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    t0 = time.perf_counter()

    def job(name):
        r = run_fixture(name, args)
        print(f"  finished {name}: {r['status']} in {r['timings']['total_s']}s", flush=True)
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
