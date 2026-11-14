"""bspwalk eye store checks (#39), on the fixture BSPs the fixture harness compiled.

(a) -eyeverify on every fixture BSP (unit_test_output/bspwalk/<NAME>/<NAME>.bsp, with the
    fixture's expect.json walker_args): every eye tile, made as the store makes it, must match
    the same tile made from every piece with no spatial index, every tile's cluster rule must
    hold, and every sample's tile must be in its cluster's tile list, at the base spacing and at
    every -eyerefine level (#39 stage D). -eyeverify makes every tile, so this is the complete
    check of the tile lists and of the tiles. bspwalk exits 1 otherwise (a missing list entry is
    an Error). The verdict is the exit code plus the JSON's eyes.verify (tiles > 0, differ 0, and
    one entry of eyes.verify.levels per level 0 to eyes.refine, each with differ 0, rule 0 and no
    list_miss), so a run where -eyeverify silently did nothing, or skipped a level, fails.
(b) bspwalk_tier_diff.py --same on SAME_FIXTURES, runs default, -eyecache 1, -threads 4 and
    BSPWALK_FACE_BATCH=1 (the face pass one face per batch): the face tiers must not depend on the
    cache budget, the thread count or the face pass's batch size. Each of them leaves faces hidden
    at the base spacing, so the default -eyerefine runs its refinement levels (#39 stage C) on it
    and the check covers the levels too. Levels run until -eyerefine or until no face is left
    hidden (stage D: a level that sees no face does not stop the next), so a fixture's default run
    stops before the last level asked (vis.refine against eyes.refine) only when its last level
    left no face hidden, and at least one fixture must run them all.
(c) The same --same check on SWEEP_FIXTURES at -eyespacing 2, where faces pass the candidate cap
    (FACE_MAX_CANDIDATES): their tiles bin a thinned set and the candidate sweep pairs the rest.
    It protects "the thinning and the sweep do not depend on the cache, the threads or the batch";
    a candidate rank that depends on the batch breaks it. The run must reach the cap
    (vis.capped_faces > 0), or it covers nothing. These two run with -eyerefine 0: their levels
    (1u, 0.5u) would only add time, and (b) covers the levels.

SAME_FIXTURES are the fixture with a sky camera, two with reflective faces (a plain mirror, and
displacement water mirrors, which go through the mirror setup's displacement branch) and the
one with the most eye samples. At the default options faces pass the cap only at the refinement
levels (vis.capped_faces is the union over the levels: f13 19, f16 1, f27 3 on 2026-11-14), so
(c) is what puts level 0's thinning and sweep under the check.

Run run_bspwalk_tests.py first (it compiles the BSPs). Exit code 0 when every check passes.
"""
import argparse
import json
import os
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
X64_DIR = HERE.parents[1]
REPO = HERE.parents[4]
MAPS_DIR = X64_DIR / "unit_test_maps" / "bspwalk"
OUT_DIR = X64_DIR / "unit_test" / "unit_test_output" / "bspwalk"
TIER_DIFF = REPO / ".agents" / "scripts" / "bspwalk_tier_diff.py"
STEAM_COMMON = os.environ.get("SEMT_STEAM_COMMON", r"E:\Steam\steamapps\common")
DEFAULT_GAME = os.path.join(STEAM_COMMON, "Source SDK Base 2013 Multiplayer", "sourcetest")
SAME_FIXTURES = ("f13_skybox_facade", "f16_reflection_cubemap", "f27_disp_water_mirror", "f06_disp_hill")
# (c): at 2u faces of these pass the candidate cap (f11 2 and f17 1 seen only by the sweep)
SWEEP_FIXTURES = ("f11_slit_view", "f17_reflection_pvs")
SWEEP_ARGS = ["-eyespacing", "2", "-eyerefine", "0"]


def walker_args(name):
    with open(MAPS_DIR / f"{name}.expect.json", encoding="utf-8") as fp:
        return list(json.load(fp).get("walker_args") or [])


def verify_one(exe, game, name, timeout):
    bsp = OUT_DIR / name / f"{name}.bsp"
    if not bsp.is_file():
        return name, False, f"no {bsp} (run run_bspwalk_tests.py first)"
    with tempfile.TemporaryDirectory(prefix="bspwalk_verify_") as tmp:
        out = Path(tmp) / f"{name}.json"
        cmd = [str(exe), str(bsp), "-game", game, "-reachslim", "-eyeverify", "-out", str(out)] + walker_args(name)
        try:
            r = subprocess.run(cmd, cwd=str(Path(exe).parent), capture_output=True, text=True, errors="replace",
                               timeout=timeout)
        except subprocess.TimeoutExpired:
            return name, False, f"timed out after {timeout:.0f} s"
        verify, refine, tier_b = None, 0, False
        try:
            with open(out, encoding="utf-8") as fp:
                j = json.load(fp)
            verify = j.get("eyes", {}).get("verify")
            refine = int(j.get("eyes", {}).get("refine") or 0)
            tier_b = bool(j.get("vis", {}).get("tier_b"))
        except (OSError, ValueError):
            pass
    # The exit code is -eyeverify's verdict (1 when a tile differs or the cluster rule fails; the
    # generator stops with an Error when a sample's tile is not in its cluster's list); eyes.verify
    # says it ran. The console line is only shown: it can follow a progress counter on the same line
    # ("0...1...9...eyeverify: ...").
    line = next((ln[ln.index("eyeverify: "):].strip() for ln in r.stdout.splitlines() if "eyeverify: " in ln), "")
    if r.returncode != 0:
        return name, False, line or f"exit {r.returncode}"
    if not isinstance(verify, dict) or not verify.get("tiles", 0) > 0 or verify.get("differ", 1) != 0:
        return name, False, f"eyes.verify is {verify!r} (want tiles > 0, differ 0)"
    # every level 0 to eyes.refine checked, each clean (#39 stage D); tier A only checks level 0
    want = refine if tier_b else 0
    levels = verify.get("levels") or []
    got = [lv.get("level") for lv in levels]
    if got != list(range(want + 1)):
        return name, False, f"eyes.verify.levels covers levels {got} (want 0 to {want})"
    bad = [lv for lv in levels if lv.get("differ") or lv.get("rule") or lv.get("list_miss")]
    if bad:
        return name, False, f"eyes.verify.levels has a failing level: {bad!r}"
    return name, True, (line or f"{verify['tiles']} tiles, {verify.get('samples')} samples") + f" [levels 0 to {want}]"


def same_one(exe, game, name, out, timeout, more=()):
    """(ok, what, refinement levels run, levels asked, vis.capped_faces) of the default run."""
    bsp = OUT_DIR / name / f"{name}.bsp"
    if not bsp.is_file():
        return name, False, f"no {bsp}", 0, 0, 0
    bindir = str(Path(exe).parent)
    extra = "".join("," + a for a in walker_args(name) + list(more))
    cmd = [sys.executable, "-I", str(TIER_DIFF), "--bsp", str(bsp), "--game", game, "--out", str(out / name), "--same",
           "--run", f"default={bindir}{extra}", "--run", f"cache1={bindir},-eyecache,1{extra}",
           "--run", f"threads4={bindir},-threads,4{extra}", "--run", f"batch1={bindir},ENV:BSPWALK_FACE_BATCH=1{extra}"]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, errors="replace", timeout=timeout * 3)
    except subprocess.TimeoutExpired:
        return name, False, "timed out", 0, 0, 0
    first = next((ln.strip() for ln in r.stdout.splitlines() if ln.startswith("default:")), "")
    # the refinement levels (#39 stage C) the default run went through (vis.refine), the levels
    # asked (eyes.refine) and the faces past the candidate cap
    levels, asked, capped = 0, 0, 0
    try:
        with open(out / name / "default.json", encoding="utf-8") as fp:
            j = json.load(fp)
        refine = j.get("vis", {}).get("refine") or []
        levels = len(refine)
        asked = int(j.get("eyes", {}).get("refine") or 0)
        # a run that stopped before the last level asked with faces still hidden: -1
        if 0 < levels < asked and refine[-1].get("faces_in", 0) > refine[-1].get("faces_seen", 0):
            levels = -levels
        capped = int(j.get("vis", {}).get("capped_faces") or 0)
    except (OSError, ValueError):
        pass
    what = first if r.returncode == 0 else r.stdout.strip()[-600:] or r.stderr.strip()[-600:]
    return name, r.returncode == 0, f"{what} [refinement levels: {levels} of {asked}; capped faces {capped}]", levels, asked, capped


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--walker", default=str(X64_DIR / "bspwalk.exe"))
    ap.add_argument("--game", default=DEFAULT_GAME)
    ap.add_argument("--out", default=None, help="directory for the tier-diff runs (default: a temporary one)")
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--timeout", type=float, default=600)
    args = ap.parse_args()
    names = sorted(p.stem.replace(".expect", "") for p in MAPS_DIR.glob("*.expect.json"))
    failed = 0
    with ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
        for name, ok, what in pool.map(lambda n: verify_one(args.walker, args.game, n, args.timeout), names):
            print(f"eyeverify  {'PASS' if ok else 'FAIL'}  {name}: {what}")
            failed += not ok
    with tempfile.TemporaryDirectory(prefix="bspwalk_cache_") as tmp:
        out = Path(args.out) if args.out else Path(tmp)
        refined, partial = 0, []
        for name in SAME_FIXTURES:
            name, ok, what, levels, asked, _ = same_one(args.walker, args.game, name, out, args.timeout)
            print(f"same tiers {'PASS' if ok else 'FAIL'}  {name} (default, -eyecache 1, -threads 4, batch 1): {what}")
            failed += not ok
            refined += asked > 0 and levels == asked
            if levels < 0:
                partial.append(f"{name} {-levels} of {asked} with faces still hidden")
        for name in SWEEP_FIXTURES:
            name, ok, what, _, _, capped = same_one(args.walker, args.game, name, out, args.timeout, SWEEP_ARGS)
            if ok and capped <= 0:
                ok, what = False, f"{what}: no face passed the candidate cap, so the thinning and the sweep went untested"
            print(f"same tiers {'PASS' if ok else 'FAIL'}  {name} at {' '.join(SWEEP_ARGS)} (default, -eyecache 1, -threads 4, "
                  f"batch 1): {what}")
            failed += not ok
    # (b) covers the refinement levels only if some fixture leaves a face hidden at the base spacing;
    # a level that sees no face no longer stops the rest, so a run has no level or all of them
    ok = refined > 0 and not partial
    print(f"refinement {'PASS' if ok else 'FAIL'}  {refined} of {len(SAME_FIXTURES)} same-tiers fixtures ran every "
          f"refinement level asked (want at least one, and none stopped part way{': ' + ', '.join(partial) if partial else ''})")
    failed += not ok
    total = len(names) + len(SAME_FIXTURES) + len(SWEEP_FIXTURES) + 1
    print(f"bspwalk_cache: {total - failed}/{total} checks passed -> {'PASS' if not failed else 'FAIL'}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
