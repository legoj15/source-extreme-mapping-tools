"""bspwalk eye store checks (#39), on the fixture BSPs the fixture harness compiled.

(a) -eyeverify on every fixture BSP (unit_test_output/bspwalk/<NAME>/<NAME>.bsp, with the
    fixture's expect.json walker_args): every eye tile, made as the store makes it, must match
    the same tile made from every piece with no spatial index, and every tile's cluster rule
    must hold. bspwalk exits 1 otherwise.
(b) bspwalk_tier_diff.py --same on SAME_FIXTURES, runs default, -eyecache 1 and -threads 4:
    the face tiers must not depend on the cache budget or the thread count.

No fixture has a face with more candidate eyes than the cap (vis.capped_faces is 0 on every
one), so (b) cannot cover the candidate thinning; it takes the fixture with a sky camera, two
with reflective faces (a plain mirror, and displacement water mirrors, which go through the
mirror setup's displacement branch) and the one with the most eye samples.

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


def walker_args(name):
    with open(MAPS_DIR / f"{name}.expect.json", encoding="utf-8") as fp:
        return list(json.load(fp).get("walker_args") or [])


def verify_one(exe, game, name, timeout):
    bsp = OUT_DIR / name / f"{name}.bsp"
    if not bsp.is_file():
        return name, False, f"no {bsp} (run run_bspwalk_tests.py first)"
    cmd = [str(exe), str(bsp), "-game", game, "-reachslim", "-eyeverify"] + walker_args(name)
    try:
        r = subprocess.run(cmd, cwd=str(Path(exe).parent), capture_output=True, text=True, errors="replace",
                           timeout=timeout)
    except subprocess.TimeoutExpired:
        return name, False, f"timed out after {timeout:.0f} s"
    # The exit code is the verdict (-eyeverify exits 1 when a tile differs). The line is
    # only shown; it can follow a progress counter on the same line ("0...1...9...eyeverify: ...").
    line = next((ln[ln.index("eyeverify: "):].strip() for ln in r.stdout.splitlines() if "eyeverify: " in ln), "")
    return name, r.returncode == 0, line or f"exit {r.returncode}"


def same_one(exe, game, name, out, timeout):
    bsp = OUT_DIR / name / f"{name}.bsp"
    if not bsp.is_file():
        return name, False, f"no {bsp}"
    bindir = str(Path(exe).parent)
    extra = "".join("," + a for a in walker_args(name))
    cmd = [sys.executable, "-I", str(TIER_DIFF), "--bsp", str(bsp), "--game", game, "--out", str(out / name), "--same",
           "--run", f"default={bindir}{extra}", "--run", f"cache1={bindir},-eyecache,1{extra}",
           "--run", f"threads4={bindir},-threads,4{extra}"]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, errors="replace", timeout=timeout * 3)
    except subprocess.TimeoutExpired:
        return name, False, "timed out"
    first = next((ln.strip() for ln in r.stdout.splitlines() if ln.startswith("default:")), "")
    return name, r.returncode == 0, (first if r.returncode == 0 else r.stdout.strip()[-600:] or r.stderr.strip()[-600:])


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
        for name in SAME_FIXTURES:
            name, ok, what = same_one(args.walker, args.game, name, out, args.timeout)
            print(f"same tiers {'PASS' if ok else 'FAIL'}  {name} (default, -eyecache 1, -threads 4): {what}")
            failed += not ok
    total = len(names) + len(SAME_FIXTURES)
    print(f"bspwalk_cache: {total - failed}/{total} checks passed -> {'PASS' if not failed else 'FAIL'}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
