#!/usr/bin/env python3
"""
run_mergeflow_tests -- check vvis_optix -mergeflow against the exact portal flow.

    python run_mergeflow_tests.py                         # the fixtures next to this script
    python run_mergeflow_tests.py --game <dir> --bin <dir containing vvis_optix.exe>
    python run_mergeflow_tests.py --pair my.bsp my.prt    # also run a local map (repeatable)
    python run_mergeflow_tests.py --keep                  # keep the temporary folder

-mergeflow (src/utils/vvis/mergeflow.cpp) replaces the exact flow with a polynomial merged-window
flow.  Its PVS is close to the exact one but neither equal to it nor a superset of it: the exact
flow marks a few pairs through sliver windows that a merged window clips away (ddd_market: 6 of
543,760 pairs), and the merge flow marks pairs the exact flow rules out.  What it must do is stay
close on both sides, actually run, and not depend on the thread count.  This harness protects
that; it asserts no exact pair counts or timings, because those change legitimately.

Per fixture (a .bsp with its .prt in fixtures/), on COPIES in a temporary folder -- VVIS overwrites
the BSP it is given -- it runs vvis_optix in two groups, once as is and once with a vis radius
(-radius_override, which the merge flow handles in code of its own):
    default flow | -mergeflow | -fast                     (each group)
    -mergeflow -mergestates 1 | -mergeflow -threads 1     (first group only)
and reads the PVS rows (lump 4, index 0) of the results.  Checks, per group:
    ran        the -mergeflow run's log has VVIS's MergeFlow summary line and the default run's has
               not (otherwise every other check would compare the default flow with itself)
    shortfall  at most MAX_MISSING_PERCENT of the default flow's cluster pairs are missing from the
               -mergeflow PVS (none allowed on a map too small for the percentage to reach one pair)
    tighter    the pairs -mergeflow adds over the default flow are at most MAX_FLOOD_SHARE of what
               -fast (the flood bound, no window clipping at all) adds; none when -fast adds none
and in the first group also:
    states     shortfall and tighter again for -mergestates 1, the loosest setting
    threads    the -mergeflow rows are identical with the default thread count and with 1 thread
A pair is one set bit of a PVS row (cluster A sees cluster B), the diagonal included.

Not covered: the fixtures have only 4-point windings and small coordinates, so the hull limit, the
rectangle fallback and the double-versus-float difference are not exercised here.

Exit code: 0 every check passed; 1 at least one check failed and nothing errored; 2 a VVIS run or
the PVS read failed, or a usage error (nothing was run).  A temporary folder with a non-passing
case is kept (its path is printed) so the logs and BSPs can be inspected.
"""
import os
from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import List, Sequence, Tuple

THIS_DIR = Path(__file__).resolve().parent
X64_DIR = THIS_DIR.parents[1]                                  # game/bin/x64
sys.dont_write_bytecode = True                                 # a test run must not leave a __pycache__ in x64
if str(X64_DIR) not in sys.path:                               # python -I drops the script dir and the cwd
    sys.path.insert(0, str(X64_DIR))

from bsp_diff_visibility import BSPVisibility, popcount        # noqa: E402

FIXTURE_DIR = THIS_DIR / "fixtures"
# Same default as unit_test/bspwalk/run_bspwalk_tests.py.  A game directory is always passed to the
# tools: without it they fall back to VPROJECT, which can point at an unrelated game.
# The Steam library differs per machine (the dev box has it on E:, Server1 on C:).
STEAM_COMMON = os.environ.get("SEMT_STEAM_COMMON", r"E:\Steam\steamapps\common")
DEFAULT_GAME = os.path.join(STEAM_COMMON, "Source SDK Base 2013 Multiplayer", "sourcetest")
VVIS_NAME = "vvis_optix.exe"
MAX_MISSING_PERCENT = 0.01                                     # of the default flow's pairs
MAX_FLOOD_SHARE = 0.25                                         # of the pairs -fast adds
MERGE_LOG_MARK = "MergeFlow ("                                 # RunMergeFlowPath's summary line
# (group name, arguments every run of the group gets).  The first group also gets the
# -mergestates 1 and -threads 1 runs.  1500 units cuts the fixtures' PVS to a fraction.
GROUPS = (("", []),
          ("radius", ["-radius_override", "1500"]))
RUNS_PER_FIXTURE = 3 * len(GROUPS) + 2

Check = Tuple[str, bool, str]                                  # (check, passed, detail)


class RunError(Exception):
    """A VVIS run or a PVS read failed, so the case's checks mean nothing (reported as ERROR)."""


def usage_error(msg: str):
    """Bad arguments: say why and exit 2 (1 is reserved for a failed check)."""
    print(f"usage error: {msg}", file=sys.stderr)
    raise SystemExit(2)


def describe_exit(rc: int) -> str:
    """'3' for a plain code, '0xC0000005' for a Windows crash (NTSTATUS in the high bits)."""
    return f"0x{rc & 0xFFFFFFFF:08X}" if rc < 0 or rc >= 0x80000000 else str(rc)


# ---------------------------------------------------------------------------
# running VVIS
# ---------------------------------------------------------------------------
def run_vvis(bin_dir: Path, game: str, bsp: Path, extra: Sequence[str], timeout: float) -> str:
    """Run vvis_optix on `bsp` (rewritten in place); returns its stdout, also kept in <bsp stem>.vvis.out."""
    cmd = [str(bin_dir / VVIS_NAME), "-game", game, *extra, str(bsp)]
    log = bsp.with_suffix(".vvis.out")
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace",
                              timeout=timeout, cwd=str(bin_dir))
    except subprocess.TimeoutExpired:
        raise RunError(f"{' '.join(extra) or 'default flow'}: vvis timed out after {timeout:.0f}s") from None
    except OSError as exc:
        raise RunError(f"cannot execute {cmd[0]}: {exc}") from None
    log.write_text(f"$ {' '.join(cmd)}\nexit {proc.returncode}\n--- stdout ---\n{proc.stdout}\n"
                   f"--- stderr ---\n{proc.stderr}\n", encoding="utf-8")
    if proc.returncode != 0:
        tail = " | ".join((proc.stdout + proc.stderr).strip().splitlines()[-2:])
        raise RunError(f"{' '.join(extra) or 'default flow'}: vvis exited {describe_exit(proc.returncode)}: {tail}")
    return proc.stdout


def strip_vis(bsp: Path) -> None:
    """Empty the visibility lump of a COPY (header entry 4: fileofs, then filelen).

    The fixtures carry a PVS already. Without this, a VVIS run that exits 0 without writing one
    would leave every copy holding the fixture's rows and the checks would pass on them.
    """
    with open(bsp, "r+b") as f:
        f.seek(8 + 4 * 16 + 4)
        f.write((0).to_bytes(4, "little"))


def read_pvs_rows(bsp: Path) -> List[int]:
    """The PVS row of every cluster as an int (bit n = this cluster sees cluster n)."""
    try:
        vis = BSPVisibility(str(bsp))
        if vis.num_clusters <= 0:
            raise RunError(f"{bsp.name} has no visibility lump after vvis")
        return [int.from_bytes(vis.decompress_pvs(i), "little") for i in range(vis.num_clusters)]
    except (OSError, ValueError, IndexError) as exc:           # truncated lump, run past its end, bad ident
        raise RunError(f"cannot read the PVS of {bsp.name}: {type(exc).__name__}: {exc}") from None


# ---------------------------------------------------------------------------
# the checks
# ---------------------------------------------------------------------------
def compare_pairs(reference: Sequence[int], candidate: Sequence[int]) -> Tuple[int, int, int]:
    """(pairs of `reference` missing from `candidate`, pairs only in `candidate`, pairs in `reference`)."""
    missing = extra = total = 0
    for ref, cand in zip(reference, candidate):
        total += popcount(ref)
        missing += popcount(ref & ~cand)
        extra += popcount(cand & ~ref)
    return missing, extra, total


def closeness(default: Sequence[int], merge: Sequence[int], fast: Sequence[int]) -> List[Tuple[bool, str]]:
    """One -mergeflow result against the default flow and -fast: the shortfall verdict, then the
    tighter verdict."""
    missing, extra, total = compare_pairs(default, merge)
    # Every cluster sees itself, so a real result is never empty; zero would pass every check.
    if total == 0:
        raise RunError("the default flow marked no cluster pairs at all")
    allowed = int(total * MAX_MISSING_PERCENT / 100)
    verdicts = [(missing <= allowed,
                 f"{missing} of {total} default-flow pairs missing, allowed {MAX_MISSING_PERCENT:g}% = {allowed}")]
    _, flood_extra, _ = compare_pairs(default, fast)
    ceiling = int(flood_extra * MAX_FLOOD_SHARE)
    verdicts.append((extra <= ceiling,
                     f"+{extra} pairs over the default flow; -fast adds {flood_extra}, ceiling "
                     f"{MAX_FLOOD_SHARE:g} of that = {ceiling}"))
    return verdicts


def run_case(bsp_src: Path, prt_src: Path, work: Path, args) -> List[Check]:
    """The VVIS runs on copies, then the checks.  Raises RunError."""
    def run(folder_name: str, extra: Sequence[str]) -> Tuple[List[int], str]:
        folder = work / folder_name
        folder.mkdir(parents=True)
        # VVIS finds the portal file from the BSP's name, so the copy takes the BSP's stem.
        bsp = folder / bsp_src.name
        shutil.copyfile(bsp_src, bsp)
        shutil.copyfile(prt_src, bsp.with_suffix(".prt"))
        strip_vis(bsp)
        log = run_vvis(args.bin, args.game, bsp, extra, args.timeout)
        return read_pvs_rows(bsp), log

    checks: List[Check] = []
    for index, (group, base) in enumerate(GROUPS):
        tag = f"{group}_" if group else ""
        prefix = f"{group}:" if group else ""
        default, default_log = run(tag + "default", base)
        merge, merge_log = run(tag + "mergeflow", [*base, "-mergeflow"])
        fast, _ = run(tag + "fast", [*base, "-fast"])
        if not len(default) == len(merge) == len(fast):
            raise RunError(f"the runs disagree on the cluster count: default {len(default)}, "
                           f"-mergeflow {len(merge)}, -fast {len(fast)}")
        in_merge, in_default = MERGE_LOG_MARK in merge_log, MERGE_LOG_MARK in default_log
        checks.append((prefix + "ran", in_merge and not in_default,
                       f"merge flow summary line in the -mergeflow log: {in_merge}, in the default log: {in_default}"))
        for name, (ok, detail) in zip(("shortfall", "tighter"), closeness(default, merge, fast)):
            checks.append((prefix + name, ok, detail))
        if index:
            continue
        one_state, _ = run("mergestates1", ["-mergeflow", "-mergestates", "1"])
        one_thread, _ = run("mergeflow1", ["-mergeflow", "-threads", "1"])
        if not len(one_state) == len(one_thread) == len(merge):
            raise RunError("the -mergeflow variants disagree on the cluster count")
        for ok, detail in closeness(default, one_state, fast):
            checks.append(("states", ok, "-mergestates 1: " + detail))
        differing = sum(a != b for a, b in zip(merge, one_thread))
        checks.append(("threads", differing == 0,
                       f"{differing} of {len(merge)} -mergeflow PVS rows differ between the default thread "
                       f"count and -threads 1"))
    return checks


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def collect_cases(pairs: Sequence[Sequence[str]]) -> List[Tuple[str, Path, Path]]:
    cases = []
    for bsp in sorted(FIXTURE_DIR.glob("*.bsp")):
        prt = bsp.with_suffix(".prt")
        if not prt.is_file():
            usage_error(f"fixture {bsp.name} has no {prt.name} next to it")
        cases.append((bsp.stem, bsp, prt))
    for bsp_arg, prt_arg in pairs:
        bsp, prt = Path(bsp_arg).expanduser().resolve(), Path(prt_arg).expanduser().resolve()
        for f in (bsp, prt):
            if not f.is_file():
                usage_error(f"--pair file not found: {f}")
        cases.append((bsp.stem, bsp, prt))
    return cases


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Check vvis_optix -mergeflow against the default flow.",
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__.split("Per fixture")[0])
    ap.add_argument("--game", default=DEFAULT_GAME, help="game directory containing gameinfo.txt")
    ap.add_argument("--bin", default=str(X64_DIR), help=f"folder holding {VVIS_NAME} (default: game/bin/x64)")
    ap.add_argument("--keep", action="store_true", help="keep the temporary folder (always kept when a case does not pass)")
    ap.add_argument("--pair", nargs=2, action="append", default=[], metavar=("BSP", "PRT"),
                    help="also check this map (a BSP and its portal file; repeatable). VVIS runs on copies.")
    ap.add_argument("--timeout", type=float, default=900, help="seconds per vvis run (default 900)")
    args = ap.parse_args(argv)                     # bad options exit 2 via argparse

    # The tools run with the bin folder as their working directory, so paths must not be cwd-relative.
    args.game = str(Path(args.game).expanduser().resolve())
    args.bin = Path(args.bin).expanduser().resolve()
    if not (args.bin / VVIS_NAME).is_file():
        usage_error(f"{VVIS_NAME} not found in {args.bin}")
    if not (Path(args.game) / "gameinfo.txt").is_file():
        usage_error(f"no gameinfo.txt in {args.game}")
    cases = collect_cases(args.pair)
    if not cases:
        usage_error(f"no fixtures in {FIXTURE_DIR} and no --pair given")

    print(f"vvis mergeflow tests: {len(cases)} fixture(s), bin={args.bin}, game={args.game}")
    tmp = Path(tempfile.mkdtemp(prefix="vvis_mergeflow_"))
    results = []                                   # (name, status, [(check, passed, detail)] or [error])
    t0 = time.perf_counter()
    try:
        for i, (name, bsp, prt) in enumerate(cases):
            print(f"  running {name} ({RUNS_PER_FIXTURE} vvis runs)", flush=True)
            try:
                checks = run_case(bsp, prt, tmp / f"{i:02d}_{name}", args)
                status = "PASS" if all(ok for _, ok, _ in checks) else "FAIL"
                results.append((name, status, checks))
            except RunError as exc:
                results.append((name, "ERROR", [("run", False, str(exc))]))
            except Exception as exc:               # noqa: BLE001 -- one case must never sink the run
                results.append((name, "ERROR", [("run", False, f"unexpected {type(exc).__name__}: {exc}")]))
    finally:
        wall = time.perf_counter() - t0
        keep = args.keep or any(status != "PASS" for _, status, _ in results)
        if not keep:
            shutil.rmtree(tmp, ignore_errors=True)

    print()
    width = max(len(n) for n, _, _ in results) + 2 if results else 10
    for name, _, checks in results:
        for check, ok, detail in checks:
            state = "ERROR" if check == "run" else ("PASS" if ok else "FAIL")
            print(f"{name:<{width}}{check:<18}{state:<6}{detail}")
    n_err = sum(s == "ERROR" for _, s, _ in results)
    n_fail = sum(s == "FAIL" for _, s, _ in results)
    n_pass = sum(s == "PASS" for _, s, _ in results)
    exit_code = 2 if n_err else (1 if n_fail else 0)
    print(f"\nTOTALS  fixtures {len(results)}: {n_pass} pass, {n_fail} fail, {n_err} error | wall time {wall:.1f}s")
    if keep:
        print(f"kept: {tmp}")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
