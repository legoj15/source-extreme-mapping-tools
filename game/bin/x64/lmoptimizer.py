#!/usr/bin/env python3
"""
lmoptimizer — VMF Lightmap Scale Optimizer

Preprocesses a VMF file to intelligently adjust per-face lightmapscale values,
reducing VBSP vertex counts on maps with high-resolution lightmap settings.

Two modes:
  1. Auto-compile (default, no --bsp): find the lowest global lightmapscale that
     fits the vertex budget, compile a lit baseline BSP (VBSP -> VVIS -> VRAD) on
     private scratch files next to the input VMF, cache it under .lmopt_cache,
     then run the BSP pipeline on it
  2. BSP mode (--bsp): Read real lightmap data from a compiled, VRAD-lit BSP

The vertex budget solver maximizes faces at lightmapscale=1, degrading the
most uniform faces first to fit within VBSP's vertex limit.

Exit status: 0 = finished; 1 = any failure (nothing is written, and a previous
output file and the user's own compile products are never touched). The output
VMF is written once, atomically, only after every stage has succeeded.

Usage:
    python lmoptimizer.py input.vmf --bsp map.bsp [options]
    python lmoptimizer.py input.vmf --bsp map.bsp --vbsp vbsp_lmo.exe --game C:\\hl2\\hl2 [options]
    python lmoptimizer.py input.vmf [-o output.vmf] [options]
"""
from __future__ import annotations

import argparse
import dataclasses
import hashlib
import json
import math
import os
import re
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Set, Tuple

# Allow running as a script from any directory
_this_dir = Path(__file__).resolve().parent
_parent_dir = _this_dir.parent
if str(_parent_dir) not in sys.path:
    sys.path.insert(0, str(_parent_dir))
if str(_this_dir) not in sys.path:
    sys.path.insert(0, str(_this_dir))

import gameinfo_search
from vmf_parser import (VMFParser, VMFParseError, VMFWriter, VMFBrush, KVNode,
                        KVPair, extract_brushes, extract_lights)
from vertex_estimator import DEFAULT_DARK_THRESHOLD


class PipelineAbort(Exception):
    """A stage cannot continue. main() prints the message and exits 1.

    Library functions raise this instead of calling sys.exit(), so they are
    testable and callers can clean up with ordinary try/finally.
    """


def _configure_stdio() -> None:
    """Force UTF-8, line-buffered console output on Windows (cp1252 can't
    encode the box-drawing characters).

    Done from main(), not at import: replacing sys.stdout/sys.stderr as an
    import side effect breaks any host that captures them (pytest, the GUI
    importing this module) and the new stderr wrapper was not line-buffered,
    so ERROR text reached the GUI only at exit.
    """
    if sys.platform != 'win32':
        return
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding='utf-8', errors='replace',
                               line_buffering=True)
        except (AttributeError, ValueError, OSError):
            pass  # not a text stream we can reconfigure; leave it alone


#: Printed by main() on every return: '<marker> errors=N', N = ERROR reports
#: this process made itself. The GUI reads it with LMOptimizerGUI._STATUS_RE
#: (lmoptimizer_gui.py, which spells the marker out: keep the two in step)
#: instead of guessing from 'ERROR' text, which child tools print too.
STATUS_MARKER = '##lmoptimizer-status'
# --coplanar: seam groups (default), strict post-solve unification, or none.
COPLANAR_MODES = ('seam', 'strict', 'off')
_errors_logged = 0


def _log_error(text: str) -> None:
    """Print one of this tool's own ERROR reports to stderr and count it."""
    global _errors_logged
    _errors_logged += 1
    print(text, file=sys.stderr, flush=True)


def _nonneg_int(text: str) -> int:
    """argparse type: an integer >= 0 (a negative count would silently mean off)."""
    v = int(text)
    if v < 0:
        raise argparse.ArgumentTypeError(f"must be 0 or more, got {v}")
    return v


def _build_parser() -> argparse.ArgumentParser:
    from vrad_runner import DEFAULT_TEXTURE_SHADOWS, TEXTURE_SHADOW_LEVELS
    parser = argparse.ArgumentParser(
        description="VMF Lightmap Scale Optimizer — reduce VBSP vertex counts "
                    "by adjusting lightmapscale on uniformly-lit faces.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python lmoptimizer.py mymap.vmf --bsp mymap.bsp --dry-run
  python lmoptimizer.py mymap.vmf --bsp mymap.bsp --vbsp vbsp_lmo.exe --game C:\\hl2\\hl2
  python lmoptimizer.py mymap.vmf --bsp mymap.bsp -o mymap_optimized.vmf
  python lmoptimizer.py mymap.vmf --bsp mymap.bsp --vertex-budget 60000
        """,
    )

    parser.add_argument('input', type=str, help='Input VMF file path')
    parser.add_argument('-o', '--output', type=str, default=None,
                        help='Output VMF file path (default: input_optimized.vmf)')
    parser.add_argument('--bsp', type=str, default=None,
                        help='Read lightmap data from compiled BSP (fastest, most accurate)')

    # Vertex budget solver (BSP mode)
    parser.add_argument('--vertex-budget', type=int, default=65536,
                        help='Target vertex limit for budget solver (default: 65536)')
    parser.add_argument('--headroom', type=int, default=3277,
                        help='Safety margin subtracted from vertex budget. '
                             'Prevents near-limit crashes with vvis -fast '
                             '(default: 3277 = 5%% of 65536)')
    parser.add_argument('--max-lightmap-dim', type=int, default=32,
                        help="VBSP's g_maxLightmapDimension (default: 32)")

    # VBSP integration (ground-truth vertex counting)
    parser.add_argument('--vbsp', type=str, default=None,
                        help='Path to vbsp_lmo.exe with -countverts support '
                             '(auto-detects in same directory if omitted)')
    parser.add_argument('--game', type=str, default=None,
                        help='Game directory for VBSP -game flag '
                             '(required when --vbsp is used)')

    # VRAD integration (surface light counting)
    parser.add_argument('--vrad', type=str, default=None,
                        help='Path to vrad_rtx.exe with -countlights support '
                             '(auto-detects if omitted)')
    parser.add_argument('--vvis', type=str, default=None,
                        help='Path to vvis.exe (auto-detects alongside VBSP '
                             'if omitted). Required for accurate light '
                             'counting — without VIS data, VRAD '
                             'under-counts surface lights.')
    parser.add_argument('--binroot', type=str, default=None,
                        help='Bin root directory for the tools (e.g. C:\\hl2\\bin). '
                             'Passed as -binroot to VBSP, VVIS, and VRAD.')
    parser.add_argument('--lights', type=str, default=None,
                        help='Path to a custom .rad lights file to forward '
                             'to VRAD during light counting '
                             '(e.g. E:\\lights_custom.rad).')
    parser.add_argument('--light-budget', type=int, default=32767,
                        help='Maximum surface lights in compiled BSP '
                             '(default: 32767 — GMod limit). '
                             'Set to 0 to disable light budget enforcement.')

    # Classification thresholds
    parser.add_argument('--dark-threshold', type=float,
                        default=DEFAULT_DARK_THRESHOLD,
                        help='Non-texlight sides lit perfectly flat in the '
                             'baseline are '
                             'raised straight to the scale where they stop '
                             'being split (coarser saves no vertices) only '
                             'if their mean brightness is at or below this. '
                             'Also the minimum of the --chop darkness cut '
                             '(a sub-face is dark below the larger of 30%% '
                             'of its face mean and this). Linear lightmap '
                             'luminance, not gamma-encoded: 5 shows as about '
                             '43 of 255 on screen, a dim grey (default: 5.0)')
    parser.add_argument('--detail-min-scale', type=int, default=5,
                        help='Minimum lightmapscale for %%detailtype materials (default: 5)')
    parser.add_argument('--smooth-tolerance', type=float, default=None,
                        nargs='?', const=1.0,
                        help='Whole sides whose baseline lighting is reproduced '
                             'within this display-space tolerance (out of 255) '
                             'at a coarser scale get that scale. 1.0 is strict, '
                             '4.0 loose (default when the flag is present: 1.0). '
                             'Omit this flag entirely to disable it.')
    parser.add_argument('--skip-ahead-calls', type=_nonneg_int, default=40,
                        help='Extra VBSP calls the promotion skip-ahead may '
                             'spend after the prefix search: a unit that does '
                             'not fit is stepped over and later units that do '
                             'fit are promoted (default: 40; 0 = off, '
                             'promotion stops at the first unit that '
                             'overflows).')
    parser.add_argument('--promote-order', choices=('value', 'priority'),
                        default='value',
                        help='Order the promotion units (coplanar groups or '
                             'single sides) are taken in. value (default): '
                             'summed lighting priority x area x luxel gain '
                             '(0 for a side a minimum scale holds in place) '
                             'per estimated chop cell the promotion adds, so '
                             'a group that '
                             'is mostly flat or costs thousands of leaffaces '
                             'ranks behind cheap detailed units. priority: '
                             "each unit's single best member, cost ignored "
                             '(the older order).')

    # Coplanar unification mode
    parser.add_argument('--coplanar', choices=COPLANAR_MODES, default='seam',
                        help='How coplanar sides share a lightmap scale. seam '
                             '(default): sides become one promotion unit only '
                             'where a seam would show (a shared edge, same '
                             'material and texture axes, shadow detail across '
                             'that edge). strict: every side is promoted on its '
                             'own, then uniform coplanar sides with matching '
                             'material and texture axes get one scale. off: no '
                             'grouping or unification; every side is solved on '
                             'its own.')
    parser.add_argument('--strict-coplanar', action='store_const',
                        dest='coplanar', const='strict',
                        help='Same as --coplanar strict.')

    parser.add_argument('--dry-run', action='store_true',
                        help='Solve and report without writing the output VMF. '
                             'It still RUNS VBSP -countverts (and bspwalk with '
                             '--visibility-check) on scratch copies placed '
                             'beside the input VMF and removed afterwards; it '
                             'never writes the output VMF, the baseline cache '
                             'or any compile product, and skips the light '
                             'budget check. With --bsp nothing is compiled. '
                             'Without --bsp (auto-compile) a cached baseline '
                             'from an earlier run is required and the run is '
                             'rejected without one.')
    parser.add_argument('--allow-unlit', action='store_true',
                        help='Accept a --bsp that has no usable lightmap '
                             'samples (never lit by VRAD, or LZMA-compressed). '
                             'Faces then classify as unknown and only the '
                             'vertex budget is solved. Without this flag such '
                             'a BSP is an error.')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Print per-face classification details')

    # Brush carving
    parser.add_argument('--chop', '--multichop', dest='chop', action='store_true',
                        help='Carve axis-aligned brushes around flat dark shadow '
                             'regions (darkness cut floored by --dark-threshold) so the dark piece alone can take max '
                             'lightmapscale. A carve is made only when a piece is '
                             'wholly dark. (--multichop is the old name.)')

    # Visibility (player position simulator)
    parser.add_argument('--visibility-check', action='store_true',
                        help='Run bspwalk.exe to find faces no player can see. '
                             'Tier A faces are a PVS proof, tier B faces are '
                             'ray-sampled; the tiers in --bspwalk-tiers get the '
                             'maximum lightmap scale')
    parser.add_argument('--vis-debug', action='store_true',
                        help='Paint never-visible faces with dev_measuregeneric01b')
    parser.add_argument('--rtx', action='store_true',
                        help='Pass -rtx to VRAD for GPU-accelerated lighting (if supported)')
    parser.add_argument('--texture-shadows', default=DEFAULT_TEXTURE_SHADOWS,
                        choices=list(TEXTURE_SHADOW_LEVELS),
                        help="Texture-shadow level for the VRAD analysis pass; match the "
                             "texture-shadow flag of your final compile. none = no flag, "
                             "props = -textureshadows (default), world = "
                             "-worldtextureshadows, translucent = -translucentshadows, "
                             "all = -alltextureshadows. world: $alphatest grate brushes "
                             "(fences) cast detailed shadows; translucent adds "
                             "$translucent grates; only all covers alpha/translucent "
                             "faces on ordinary brushes (e.g. glass)")
    parser.add_argument('--no-cache', action='store_true',
                        help='Force a fresh VBSP/VVIS/VRAD compile even if a cached BSP exists')
    parser.add_argument('--vis-workers', type=int, default=0,
                        help='bspwalk worker threads (0=all cores)')
    parser.add_argument('--bspwalk', type=str, default=None,
                        help='Path to bspwalk.exe (default: next to this script)')
    parser.add_argument('--bspwalk-tiers', type=str, default='A,B',
                        help='bspwalk tiers treated as never-visible (max lightmap scale): A, B or A,B (default)')
    parser.add_argument('--bspwalk-args', type=str, default='',
                        help='Extra bspwalk.exe arguments, e.g. "-profile episodic"')
    parser.add_argument('--bspwalk-timeout', type=float, default=None,
                        help='Kill bspwalk and fail after this many seconds '
                             '(default: no limit)')

    return parser

@dataclass
class _Tools:
    """Everything the pipeline shells out to, resolved and validated up front."""
    vbsp: Optional[Path] = None
    vrad: Optional[Path] = None
    vvis: Optional[Path] = None
    game_dir: Optional[Path] = None
    bin_root: Optional[Path] = None
    lights_rad: Optional[Path] = None
    bspwalk: Optional[Path] = None


def _preflight(args, auto_mode: bool,
               output_path: Optional[Path] = None) -> Optional[_Tools]:
    """Validate every prerequisite BEFORE any parse or compile work.

    Returns the resolved tools, or None after printing every problem found
    (one report, not one failure per run). Explicit paths the user typed must
    exist (a typo must not silently fall back to auto-detection or to the
    lower-quality calibration solver); auto-compile mode additionally needs
    VBSP, VVIS, VRAD and a game directory, and --visibility-check needs
    bspwalk.exe and a game directory.
    """
    errors: List[str] = []

    for flag, value in (('--vbsp', args.vbsp), ('--vrad', args.vrad),
                        ('--vvis', args.vvis), ('--lights', args.lights),
                        ('--bsp', args.bsp)):
        if value and not Path(value).is_file():
            errors.append(f"{flag} file not found: {Path(value).resolve()}")
    if args.binroot and not Path(args.binroot).is_dir():
        errors.append(f"--binroot directory not found: {Path(args.binroot).resolve()}")
    if args.game and not Path(args.game).is_dir():
        errors.append(f"--game directory not found: {Path(args.game).resolve()}")
    timeout = args.bspwalk_timeout
    if timeout is not None and (not math.isfinite(timeout) or timeout < 0):
        errors.append(f"--bspwalk-timeout must be a number of seconds >= 0 "
                      f"(0 = no limit), got {timeout!r}")
    if output_path is not None and not args.dry_run:
        problem = _output_folder_problem(output_path)
        if problem:
            errors.append(problem)
    if errors:
        _print_preflight_errors(errors)
        return None

    # _resolve_game_dir prints its own detailed message on a bad layout and
    # raises PipelineAbort, which main() turns into exit 1
    game_dir = _resolve_game_dir(args) if args.game else None
    vbsp = _resolve_vbsp(args)
    vrad = _resolve_vrad(args)
    vvis = _resolve_vvis(args, vbsp, warn=False)
    tools = _Tools(
        vbsp=vbsp, vrad=vrad, vvis=vvis, game_dir=game_dir,
        bin_root=Path(args.binroot).resolve() if args.binroot else None,
        lights_rad=Path(args.lights).resolve() if args.lights else None,
    )

    if auto_mode:
        if not game_dir:
            errors.append("--game <mod directory> is required in auto-compile "
                          "mode (no --bsp)")
        if not vbsp:
            errors.append("vbsp_lmo.exe not found — pass --vbsp")
        if not vrad:
            errors.append("vrad_rtx.exe not found — pass --vrad (auto-compile "
                          "lights the baseline with VRAD)")
        if not vvis:
            errors.append("VVIS not found (looked for vvis.exe / vvis_optix.exe "
                          "next to VBSP, under --binroot and next to the game "
                          "folder) — pass --vvis. Without VIS data the "
                          "baseline has no bounce lighting")

    if args.visibility_check:
        # bspwalk_runner owns what bspwalk needs (exe, game dir with
        # gameinfo.txt, a valid tier list, parseable extra args); a mistyped
        # --bspwalk path raises from find_bspwalk instead of falling back to
        # the default exe.
        try:
            import bspwalk_runner
            exe = bspwalk_runner.find_bspwalk(args.bspwalk)
            tools.bspwalk, _game, _tiers = bspwalk_runner.validate_setup(
                game_dir, exe=exe, tiers=args.bspwalk_tiers)
            bspwalk_runner.split_args(args.bspwalk_args)
        except ImportError as e:
            errors.append(f"bspwalk_runner could not be imported: {e}")
        except bspwalk_runner.BspwalkError as e:
            errors.append(f"--visibility-check: {e}")

    if errors:
        _print_preflight_errors(errors)
        return None
    return tools


def _output_folder_problem(output_path: Path) -> Optional[str]:
    """Why the output file cannot be written, or None. Checked before any
    compile so the failure does not arrive after VRAD. The folder may not
    exist yet (it is created when publishing); then its nearest existing
    ancestor must be writable. Probes with a real temp file: os.access lies
    on Windows."""
    folder = output_path.parent
    probe = folder
    while not probe.exists() and probe != probe.parent:
        probe = probe.parent
    if not probe.is_dir():
        return f"output folder {folder} cannot be created ({probe} is not a folder)"
    try:
        import tempfile
        with tempfile.NamedTemporaryFile(dir=probe, prefix='.lmopt_probe_'):
            pass
    except OSError as e:
        return f"output folder {probe} is not writable: {e}"
    return None


def _print_preflight_errors(errors: List[str]) -> None:
    _log_error("ERROR: cannot start — fix the following first:")
    for e in errors:
        print(f"  • {e}", file=sys.stderr)


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point. Returns the process exit code.

    0 = finished; every failure returns nonzero and writes
    no output VMF — a previous good output file is never touched on failure.
    Every return ends with the STATUS_MARKER line (an uncaught exception
    prints it before its traceback, and exits nonzero anyway).
    """
    global _errors_logged
    _errors_logged = 0
    try:
        return _main(argv)
    finally:
        print(f"{STATUS_MARKER} errors={_errors_logged}", flush=True)


def _main(argv: Optional[Sequence[str]]) -> int:
    _configure_stdio()
    args = _build_parser().parse_args(argv)
    if args.smooth_tolerance is not None and args.smooth_tolerance <= 0:
        # The GUI's 0 = off; nothing can be reproduced within a tolerance <= 0.
        print(f"  WARNING: --smooth-tolerance {args.smooth_tolerance:g} is not "
              f"positive; smooth-side evidence is off", file=sys.stderr, flush=True)
        args.smooth_tolerance = None

    # ─── Resolve paths ────────────────────────────────────────────────────────
    input_path = Path(args.input).resolve()
    if not input_path.is_file():
        _log_error(f"ERROR: Input file not found: {input_path}")
        return 1

    if args.output:
        output_path = Path(args.output).resolve()
    else:
        output_path = input_path.with_name(
            input_path.stem + '_optimized' + input_path.suffix)

    if output_path == input_path or (
            output_path.exists() and os.path.samefile(input_path, output_path)):
        _log_error("ERROR: Output path must differ from the input path (the "
              "input VMF is never modified).")
        return 1
    if output_path.is_dir():
        _log_error(f"ERROR: Output path is a directory: {output_path}")
        return 1

    auto_mode = not args.bsp
    from vbsp_runner import VBSPError
    from vrad_runner import VRADError
    try:
        tools = _preflight(args, auto_mode, output_path)
        if tools is None:
            return 1
        _print_banner(args, input_path, output_path, tools)
        return _run_pipeline(args, input_path, output_path, tools, auto_mode)
    except (PipelineAbort, VBSPError, VRADError, TimeoutError,
            VMFParseError) as e:
        _log_error(f"\n  ERROR: {e}")
        return 1
    except OSError as e:
        _log_error(f"\n  ERROR: file operation failed: {e}")
        return 1


def _print_banner(args, input_path: Path, output_path: Path, tools: _Tools) -> None:
    print(f"╔══════════════════════════════════════════════════╗")
    print(f"║          VMF Lightmap Scale Optimizer            ║")
    print(f"╚══════════════════════════════════════════════════╝")
    print(f"  Input:  {input_path}")
    print(f"  Output: {output_path}")
    if args.bsp:
        print(f"  BSP:    {Path(args.bsp).resolve()}")
    if tools.vbsp:
        print(f"  VBSP:   {tools.vbsp}")
    if tools.vrad:
        print(f"  VRAD:   {tools.vrad}")
    if tools.vvis:
        print(f"  VVIS:   {tools.vvis}")
    if args.light_budget > 0:
        print(f"  Light budget: {args.light_budget:,}")
    print()


def _run_pipeline(args, input_path: Path, output_path: Path,
                  tools: _Tools, auto_mode: bool) -> int:
    # ─── Sweep the leftovers of a hard-killed earlier run ─────────────────────
    # A cancelled GUI run kills the process without running finally blocks, so
    # its scratch VMFs / compiler products / atomic-write temps stay behind.
    # They carry their owner's host and pid in the name; this machine's dead
    # owners' files (and only those, once old enough) are removed.
    # (Skipped under --dry-run, which must not write or delete anything.)
    if not args.dry_run:
        _sweep_leftovers(input_path, output_path)

    # ─── Phase 1: Parse VMF ───────────────────────────────────────────────────
    t0 = time.perf_counter()
    print("[1/5] Parsing VMF...", flush=True)
    vmf_parser = VMFParser()
    root = vmf_parser.parse_file(input_path)
    t1 = time.perf_counter()
    print(f"  Parsed in {t1 - t0:.2f}s", flush=True)

    # The map's own scales, for the scale report's input_scale: auto-compile
    # applies the baseline scale to the tree before the solver, so this is the
    # last point at which they can still be read.
    input_scales = None if args.dry_run else _snapshot_scales(root)

    # ─── Route: direct BSP mode, or auto-compile of a baseline BSP ────────────
    if auto_mode:
        result = _run_auto_compile_mode(args, root, input_path, tools, t0)
    else:
        result = _run_bsp_mode(args, root, t0, tools)

    if result is None:
        return 1  # the failing stage already said why

    # ─── Report ───────────────────────────────────────────────────────────────
    if 'vbsp_solver_result' in result:
        _print_vbsp_solver_results(result, args)
    else:
        _print_solver_results(result, args)

    if not _vertex_budget_met(result):
        _log_error("\n  ERROR: the solved scales do not fit the vertex budget — "
              "no output written.")
        return 1

    # ─── Light budget enforcement (post-solver) ───────────────────────────────
    # (the report's changed_by tells the raises it makes from this snapshot)
    pre_light_scales = None if args.dry_run else _snapshot_scales(root)
    if not args.dry_run and args.light_budget > 0:
        if tools.vbsp and tools.vrad and tools.game_dir:
            ok = _run_light_budget_enforcement(
                root=root,
                side_map=_build_vmf_side_map(root),
                anchor_vmf=input_path,
                vbsp_exe=tools.vbsp,
                vrad_exe=tools.vrad,
                vvis_exe=tools.vvis,
                game_dir=tools.game_dir,
                bin_root=tools.bin_root,
                lights_rad=tools.lights_rad,
                level_rad=_level_rad_path(input_path),
                args=args,
            )
            if not ok:
                _log_error("\n  ERROR: the light budget could not be met — no "
                      "output written (a previous output is untouched).")
                return 1
        else:
            missing = []
            if not tools.vrad:
                missing.append("vrad_rtx.exe")
            if not tools.vbsp:
                missing.append("vbsp_lmo.exe")
            if not tools.game_dir:
                missing.append("--game")
            print(f"\n  WARNING: light budget NOT enforced (missing "
                  f"{', '.join(missing)}). Pass --light-budget 0 to silence "
                  f"this.", file=sys.stderr, flush=True)

    # ─── Final modification: vis-debug proxy textures ─────────────────────────
    if getattr(args, 'vis_debug', False) and not args.dry_run:
        _apply_vis_debug(root, result)

    # ─── Write ────────────────────────────────────────────────────────────────
    if args.dry_run:
        print("\n  [DRY RUN] No output file written.")
        return 0
    print(f"\n  Writing optimized VMF to: {output_path}", flush=True)
    _publish_vmf(root, output_path)
    # Publishing leaves the tree as it was, so the report reads the same final
    # scales; it is written only now that the VMF is on disk.
    _write_scale_report(root, result, args, input_path, output_path, auto_mode,
                        input_scales, pre_light_scales)
    print(f"  Total time: {time.perf_counter() - t0:.2f}s")
    print(f"\n  Done! Run VBSP on the optimized VMF and compare vertex counts.")
    return 0


def _sweep_leftovers(input_path: Path, output_path: Path) -> None:
    """Best-effort cleanup of an earlier hard-killed run's stamped files. A
    problem here (an unreadable folder, a pid that cannot be checked) is a
    warning, never a reason to fail the run."""
    try:
        import tempfile
        from vbsp_runner import sweep_stale_scratch
        cache_dir = input_path.parent / '.lmopt_cache'
        swept = sweep_stale_scratch(input_path.parent, 'scratch')
        swept += sweep_stale_scratch(output_path.parent, 'publish')
        if cache_dir.is_dir():
            swept += sweep_stale_scratch(cache_dir, 'cache')
        # the fallbacks that land in the system temp folder
        temp_dir = Path(tempfile.gettempdir())
        swept += sweep_stale_scratch(temp_dir, ('scratch', 'cache'))
        if swept:
            print(f"  Removed {swept} scratch file(s) left by an earlier "
                  f"interrupted run.", flush=True)
    except Exception as e:  # noqa: BLE001 - housekeeping must never fail a run
        print(f"  WARNING: could not clean up leftovers of an earlier run: {e}",
              file=sys.stderr, flush=True)


def _publish_vmf(root, output_path: Path) -> None:
    """Write the final VMF atomically: temp file next to it, then os.replace.

    The only code that creates the user's output file. A failure (or a kill)
    mid-write leaves any previous output untouched instead of truncated.
    """
    from vbsp_runner import atomic_tmp_name
    tmp = output_path.with_name(atomic_tmp_name(output_path.name, hidden=True))
    try:
        VMFWriter().write_file(root, tmp)
        os.replace(tmp, output_path)
    finally:
        try:
            if tmp.exists():
                tmp.unlink()
        except OSError:
            pass


#: The per-side decision report written beside the output VMF as
#: '<output stem>.lmo_report.json'; docs/lmoptimizer_report.md has the format
#: and lmo_report.py is the query script.
REPORT_SUFFIX = '.lmo_report.json'
REPORT_VERSION = 1

#: Arguments the report header records: the ones that change which scale a side gets.
_REPORT_ARGS = (
    'vertex_budget', 'headroom', 'max_lightmap_dim', 'light_budget',
    'dark_threshold', 'detail_min_scale', 'smooth_tolerance', 'coplanar',
    'chop', 'visibility_check', 'bspwalk_tiers', 'bspwalk_args', 'promote_order',
    'skip_ahead_calls', 'texture_shadows', 'rtx', 'allow_unlit',
)


def _snapshot_scales(root) -> Dict[int, int]:
    """side_id -> lightmapscale in the tree right now."""
    return {sid: _get_lightmapscale(node)
            for sid, node in _build_vmf_side_map(root).items()}


def _scale_report_path(output_path: Path) -> Path:
    return output_path.with_name(output_path.stem + REPORT_SUFFIX)


def _scalar_fields(obj) -> dict:
    """The plain numbers, strings and flags of a solver result (not its scale
    and decision tables), for the report header."""
    items = (((f.name, getattr(obj, f.name)) for f in dataclasses.fields(obj))
             if dataclasses.is_dataclass(obj) else vars(obj).items())
    return {k: v for k, v in items if isinstance(v, (bool, int, float, str))}


def _build_scale_report(root, result: dict, args, input_path: Path,
                        output_path: Path, auto_mode: bool,
                        input_scales: Dict[int, int],
                        pre_light_scales: Optional[Dict[int, int]]) -> dict:
    """Why every side ended at its lightmapscale: the run header plus one
    record per side of the final tree that has a face extent, and one for each
    side without an extent that a floor or a stage still set the scale of (a
    displacement whose polygon vanished; flagged no_geometry, no decision).

    Reads the tree as it is (so final_scale includes the floors, strict
    unification and the light-budget raises) and the solver's decision records;
    input_scales is the tree as parsed, before the auto-compile baseline was
    applied. Missing pieces degrade to absent fields, never to an exception:
    the result dict may come from a stage that records less.
    """
    from vertex_estimator import should_skip_face
    data = result.get('report_data') or {}
    vbsp_result = result.get('vbsp_solver_result')
    solver_obj = vbsp_result if vbsp_result is not None else result.get('solver_result')
    decisions = getattr(vbsp_result, 'decisions', None) or {}
    solver_scales = data.get('solver_scales') or {}
    stage_changes = data.get('stage_changes') or {}
    side_floors = data.get('side_floors') or {}
    floor_kinds = data.get('floor_kinds') or {}
    promoted_ids = data.get('promoted_side_ids') or set()
    carved_from = {new: old for old, news in (data.get('side_remap') or {}).items()
                   for new in news}
    flag_sets = (('never_visible', result.get('never_visible_sides')),
                 ('raw_never_visible', result.get('raw_nv_sides')),
                 ('culled', data.get('culled_sides')),
                 ('texlight', result.get('texlight_sides')))

    side_map = _build_vmf_side_map(root)
    entity_of = _build_brush_entity_map(root)
    brush_of: Dict[int, int] = {}
    for solid in root.get_all_recursive('solid'):
        brush_id = solid.get_property('id')
        if not brush_id:
            continue
        for side in solid.get_children_by_name('side'):
            side_id = side.get_property('id')
            if side_id:
                try:
                    brush_of[int(side_id)] = int(brush_id)
                except ValueError:
                    pass

    def flags_of(sid: int) -> dict:
        flags = {name: True for name, members in flag_sets
                 if members and sid in members}
        if sid in carved_from:
            flags['carved_from'] = carved_from[sid]
        if sid in promoted_ids:
            flags['carve_promoted'] = True
        return flags

    def input_of(sid: int) -> Optional[int]:
        # A carved piece inherits the scale of the side it was cut from.
        return input_scales.get(carved_from.get(sid, sid))

    def stages_of(sid: int, final: int) -> List[str]:
        """The stages outside the solver that moved this side, in the order they ran."""
        changed = [stage for stage in ('strict_unify', 'seam_unify', 'floors')
                   if sid in stage_changes.get(stage, ())]
        if pre_light_scales is not None and pre_light_scales.get(sid, final) != final:
            changed.append('light_budget')
        input_scale = input_of(sid)
        if (not changed and sid not in solver_scales
                and input_scale is not None and final != input_scale):
            # Moved, but by no tracked stage and not by the solver: auto-compile
            # raises these (vanished or skipped displacements) on the tree before
            # the solver runs.
            changed = ['before_solver']
        return changed

    sides: Dict[int, dict] = {}
    for fe in data.get('face_extents') or ():
        sid = fe.side_id
        node = side_map.get(sid)
        if node is None:
            continue
        final = _get_lightmapscale(node)
        brush_id = brush_of.get(sid)
        rec = {
            'brush_id': brush_id,
            'entity': entity_of.get(brush_id, ''),
            'material': fe.material,
            'extent_s': round(fe.extent_s, 2),
            'extent_t': round(fe.extent_t, 2),
            'has_displacement': fe.has_displacement,
            'input_scale': input_of(sid),
            'solver_scale': solver_scales.get(sid),
            'final_scale': final,
            'flags': flags_of(sid),
            'floor': side_floors.get(sid),
            'floor_kinds': floor_kinds.get(sid, []),
            'changed_by': stages_of(sid, final),
        }
        decision = decisions.get(sid)
        if decision is not None:
            rec.update(decision)
        elif should_skip_face(fe.material, fe.brush_entity):
            rec['decision'] = 'skipped'
            rec['skip_reason'] = ('tool material' if fe.material.upper().startswith('TOOLS/')
                                  else f'brush entity {fe.brush_entity}')
        sides[sid] = rec

    # Sides with no face extent never reach the solver, but a floor (a
    # displacement whose polygon vanished is floored at _DISP_NO_GEOMETRY_FLOOR)
    # or another step can still have set their scale, so the report must be able
    # to answer for them too. Nothing is recorded for the many that nothing touched.
    for sid, node in side_map.items():
        if sid in sides:
            continue
        final = _get_lightmapscale(node)
        input_scale = input_of(sid)
        changed_by = stages_of(sid, final)
        moved = input_scale is not None and final != input_scale
        if not (sid in side_floors or changed_by or moved):
            continue
        brush_id = brush_of.get(sid)
        sides[sid] = {
            'brush_id': brush_id,
            'entity': entity_of.get(brush_id, ''),
            'material': node.get_property('material') or '',
            'has_displacement': bool(node.get_children_by_name('dispinfo')),
            'input_scale': input_scale,
            'solver_scale': solver_scales.get(sid),
            'final_scale': final,
            'flags': {**flags_of(sid), 'no_geometry': True},
            'floor': side_floors.get(sid),
            'floor_kinds': floor_kinds.get(sid, []),
            'changed_by': changed_by,
        }

    return {
        'run': {
            'report_version': REPORT_VERSION,
            'created': datetime.now(timezone.utc).isoformat(timespec='seconds'),
            'input': str(input_path),
            'output': str(output_path),
            'mode': 'auto-compile' if auto_mode else 'bsp',
            'bsp': None if auto_mode else str(Path(args.bsp).resolve()),
            'solver': 'vbsp' if vbsp_result is not None else 'calibration',
            'args': {name: getattr(args, name, None) for name in _REPORT_ARGS},
            'result': _scalar_fields(solver_obj) if solver_obj is not None else {},
        },
        'sides': sides,
    }


def _format_scale_report(report: dict) -> str:
    """The report as JSON text with the header indented and one side per
    line, so grep and a line-oriented diff work on a multi-megabyte file."""
    lines = [f'{json.dumps(str(sid))}:{json.dumps(rec, separators=(",", ":"))}'
             for sid, rec in sorted(report['sides'].items())]
    return ('{\n"run": ' + json.dumps(report['run'], indent=1)
            + ',\n"sides": {\n' + ',\n'.join(lines) + '\n}\n}\n')


def _write_scale_report(root, result: dict, args, input_path: Path,
                        output_path: Path, auto_mode: bool,
                        input_scales: Optional[Dict[int, int]],
                        pre_light_scales: Optional[Dict[int, int]]) -> None:
    """Write the scale report beside the published VMF (atomically, like
    _publish_vmf). Called only after the VMF was published. It is diagnostics:
    any failure is a warning, never a failed run. A failure also removes an
    older report beside the output, which would describe a different run (and
    warns if that cannot be done).
    """
    from vbsp_runner import atomic_tmp_name
    path = _scale_report_path(output_path)
    tmp = path.with_name(atomic_tmp_name(path.name, hidden=True))
    try:
        report = _build_scale_report(root, result, args, input_path, output_path,
                                     auto_mode, input_scales or {}, pre_light_scales)
        with open(tmp, 'w', encoding='utf-8', newline='\n') as f:
            f.write(_format_scale_report(report))
        os.replace(tmp, path)
    except Exception as e:  # noqa: BLE001 - a diagnostic file must never fail the run
        print(f"  WARNING: could not write the scale report {path.name}: {e}",
              file=sys.stderr, flush=True)
        try:
            if path.exists():
                path.unlink()
                print(f"  WARNING: removed the older {path.name}, which no longer "
                      f"matches the output.", file=sys.stderr, flush=True)
        except OSError as unlink_error:
            print(f"  WARNING: an older {path.name} from an earlier run is still "
                  f"beside the output and does NOT match it (could not remove "
                  f"it: {unlink_error}).", file=sys.stderr, flush=True)
        return
    finally:
        try:
            if tmp.exists():
                tmp.unlink()
        except OSError:
            pass
    print(f"  Scale report: {path} ({len(report['sides']):,} sides; query it "
          f"with lmo_report.py)", flush=True)


def _vertex_budget_met(result: dict) -> bool:
    """True if the solved result fits the vertex budget it was solved for."""
    if 'vbsp_solver_result' in result:
        sr = result['vbsp_solver_result']
        return (sr.vbsp_vertex_count <= sr.vertex_budget and
                sr.vbsp_leafface_count <= sr.vertex_budget and
                sr.vbsp_face_count <= sr.vertex_budget)
    sr = result['solver_result']
    return sr.optimized_verts <= sr.vertex_budget


def _apply_vis_debug(root, result: dict) -> None:
    """Paint never-visible faces with proxy textures (--vis-debug)."""
    nv_sides = result.get('raw_nv_sides', result.get('never_visible_sides', set()))
    texlight_skip = result.get('texlight_sides', set())
    texlight_mats = result.get('texlight_mats', set())
    _VIS_MAT   = 'dev/dev_measuregeneric01'
    _INVIS_MAT = 'dev/dev_measuregeneric01b'

    painted = 0
    for world_node in root.get_children_by_name('world'):
        for solid in world_node.get_children_by_name('solid'):
            for side in solid.get_children_by_name('side'):
                cur_mat = (side.get_property('material') or '').upper().replace('\\', '/').strip('/')
                if cur_mat.startswith('TOOLS/'):
                    continue
                if cur_mat in texlight_mats:
                    continue

                sid = side.get_property('id')
                if sid:
                    sid_int = int(sid)
                    if sid_int in texlight_skip:
                        continue
                    mat = _INVIS_MAT if sid_int in nv_sides else _VIS_MAT
                    side.set_property('material', mat)
                    painted += 1
    print(f"\n  [Vis Debug] Painted {painted} world faces with proxy textures.", flush=True)


def _print_solver_results(result, args):
    """Print vertex budget solver results."""
    sr = result['solver_result']
    under = sr.optimized_verts <= sr.vertex_budget

    print()
    print("┌──────────────────────────────────────────────────┐")
    print("│              Vertex Budget Results               │")
    print("├──────────────────────────────────────────────────┤")
    print(f"│  Vertex budget:          {sr.vertex_budget:>8,}               │")
    print(f"│  VMF current estimate:   {sr.initial_verts:>8,}               │")
    print(f"│  All-at-scale-1:         {sr.all_at_one_verts:>8,}               │")
    print(f"│  After optimization:     {sr.optimized_verts:>8,}               │")
    print("├──────────────────────────────────────────────────┤")
    print(f"│  Faces at scale=1:       {sr.faces_at_scale_1:>8}               │")
    print(f"│  Faces degraded:         {sr.faces_degraded:>8}               │")
    print(f"│  Max scale assigned:     {sr.max_scale_assigned:>8}               │")
    print(f"│  Faces skipped (tools):  {sr.faces_skipped:>8}               │")
    if hasattr(sr, 'faces_pre_promoted') and sr.faces_pre_promoted > 0:
        print(f"│  Faces pre-promoted:     {sr.faces_pre_promoted:>8}               │")
    print(f"│  Solver iterations:      {sr.iterations:>8}               │")
    print("├──────────────────────────────────────────────────┤")

    if under:
        margin = sr.vertex_budget - sr.optimized_verts
        saved_from_ideal = sr.all_at_one_verts - sr.optimized_verts
        print(f"│  ✓ UNDER BUDGET by {margin:>6,} verts             │")
        print(f"│  Quality cost: {saved_from_ideal:>6,} verts from ideal      │")
    else:
        over = sr.optimized_verts - sr.vertex_budget
        print(f"│  ✗ OVER BUDGET by {over:>7,} verts              │")
        print(f"│  All faces maxed out — map needs fewer          │")
        print(f"│  brushes or a higher vertex limit.              │")

    if sr.degraded_detail_faces > 0:
        print(f"│  ⚠ {sr.degraded_detail_faces} lighting-detail faces degraded     │")

    print("└──────────────────────────────────────────────────┘")


def _print_vbsp_solver_results(result, args):
    """Print VBSP-in-the-loop solver results (ground truth)."""
    sr = result['vbsp_solver_result']
    budget = sr.vertex_budget
    under = (sr.vbsp_vertex_count <= budget and
             sr.vbsp_leafface_count <= budget and
             sr.vbsp_face_count <= budget)

    print()
    print("┌──────────────────────────────────────────────────┐")
    print("│        VBSP Ground-Truth Solver Results          │")
    print("├──────────────────────────────────────────────────┤")
    print(f"│  Budget (per limit):     {budget:>8,}               │")
    print(f"│  VBSP vertices:          {sr.vbsp_vertex_count:>8,}               │")
    print(f"│  VBSP leaffaces:         {sr.vbsp_leafface_count:>8,}               │")
    print(f"│  VBSP faces:             {sr.vbsp_face_count:>8,}               │")
    print(f"│  Binding limit:        {sr.binding_limit:>10}               │")
    print("├──────────────────────────────────────────────────┤")
    print(f"│  Baseline scale:         {sr.baseline_scale:>8}               │")
    print(f"│  Faces promoted to 1:    {sr.faces_promoted:>8}               │")
    print(f"│  Faces at baseline:      {sr.faces_at_baseline:>8}               │")
    print(f"│  Faces skipped (tools):  {sr.faces_skipped:>8}               │")
    if hasattr(sr, 'faces_pre_promoted') and sr.faces_pre_promoted > 0:
        print(f"│  Faces pre-promoted:     {sr.faces_pre_promoted:>8}               │")
    if (getattr(sr, 'faces_evidence_raised', 0) > 0
            or getattr(args, 'smooth_tolerance', None) is not None):
        print(f"│  Evidence raised:        {sr.faces_evidence_raised:>8}               │")
    print(f"│  VBSP invocations:       {sr.vbsp_calls:>8}               │")
    print(f"│  Solve time:           {sr.solve_time:>7.1f}s               │")
    print("├──────────────────────────────────────────────────┤")

    if under:
        worst = max(sr.vbsp_vertex_count, sr.vbsp_leafface_count, sr.vbsp_face_count)
        margin = budget - worst
        print(f"│  ✓ ALL LIMITS OK, margin: {margin:>5,} ({sr.binding_limit})  │")
    else:
        worst = max(sr.vbsp_vertex_count, sr.vbsp_leafface_count, sr.vbsp_face_count)
        over = worst - budget
        print(f"│  ✗ OVER BUDGET by {over:>7,} ({sr.binding_limit})       │")
        print(f"│  Map has too many brushes for this budget.      │")

    print("└──────────────────────────────────────────────────┘")


# ═══════════════════════════════════════════════════════════════════════════════
#  BSP MODE — vertex budget solver with real lightmap data
# ═══════════════════════════════════════════════════════════════════════════════

def _discard_log(_line: str) -> None:
    """bspwalk console sink for non-verbose runs."""


def _level_rad_path(input_vmf: Path) -> Path:
    """The map's implicit level light file (<map>.rad) VRAD would load."""
    return Path(input_vmf).with_suffix('.rad')


def _run_bsp_mode(args, root, t0: float, tools: _Tools,
                  emissive_floor: int = 1) -> dict | None:
    """Run the BSP-based optimization pipeline with vertex budget solver.

    Returns the result dict, or None after printing why it failed (the caller
    turns None into a nonzero exit). emissive_floor is the lowest lightmap
    scale the baseline light-budget ladder found texlight faces can take
    (1 = no constraint); it becomes a raise-only floor on those faces.
    """
    from bsp_reader import BSPReader, match_bsp_to_vmf, bsp_lighting_problem
    from vertex_estimator import compute_face_extent, FaceExtent, estimate_map_vertices
    from budget_solver import solve_vertex_budget
    from geometry import build_all_faces, build_brush_faces

    bsp_path = Path(args.bsp).resolve()
    if not bsp_path.exists():
        _log_error(f"ERROR: BSP file not found: {bsp_path}")
        return None

    max_lm_dim = args.max_lightmap_dim

    # ─── Phase 2: Build geometry + compute face extents ───────────────────────
    t1 = time.perf_counter()
    print("[2/5] Building geometry and computing face extents...", flush=True)

    # Extract brushes with entity classification
    brush_entity_map = _build_brush_entity_map(root)
    brushes = extract_brushes(root)
    print(f"  Found {len(brushes)} brushes", flush=True)

    # Build face polygons via CSG
    all_geometry_faces = build_all_faces(brushes)
    print(f"  Built {len(all_geometry_faces)} face polygons", flush=True)
    vanished_disps = _report_geometry_gaps(
        brushes, all_geometry_faces, _build_vmf_side_map(root))

    # Compute face extents from polygon vertices + VMF axis data
    face_extents: List[FaceExtent] = []
    side_map = _build_vmf_side_map(root)

    for gf in all_geometry_faces:
        side_node = side_map.get(gf.side_id)
        if side_node is None:
            continue

        uaxis_str = side_node.get_property('uaxis') or ''
        vaxis_str = side_node.get_property('vaxis') or ''

        entity_class = brush_entity_map.get(gf.brush_id, '')

        fe = compute_face_extent(
            gf.vertices, uaxis_str, vaxis_str,
            gf.lightmapscale, gf.side_id, gf.material,
            brush_entity=entity_class,
            has_displacement=bool(side_node.get_children_by_name('dispinfo')),
        )
        face_extents.append(fe)

    t2 = time.perf_counter()
    print(f"  Computed extents for {len(face_extents)} faces in {t2 - t1:.2f}s",
          flush=True)

    # ─── Phase 2.5: Scan materials for %detailtype ────────────────────────────
    detail_type_materials = set()
    detail_min_scale = args.detail_min_scale
    game_dir = tools.game_dir

    if game_dir:
        print("[2.5/6] Scanning materials for %detailtype...", flush=True)
        from vmt_checker import scan_materials
        
        # Collect unique material names from map faces
        unique_materials = {fe.material.lower().replace('\\', '/') for fe in face_extents}
        print(f"  {len(unique_materials)} unique materials in map", flush=True)
        
        detail_type_materials = scan_materials(
            unique_materials, game_dir, verbose=args.verbose)
        
        if detail_type_materials:
            print(f"  Found {len(detail_type_materials)} materials with %detailtype "
                  f"(min scale: {detail_min_scale}):", flush=True)
            for mat in sorted(detail_type_materials):
                print(f"    • {mat}", flush=True)
        else:
            print(f"  No %detailtype materials found", flush=True)
    else:
        print("  (No --game dir — skipping %detailtype scan)", flush=True)

    # ─── Raw initial vertex estimate (uncalibrated, at VMF's current scales) ────
    raw_initial_verts, _ = estimate_map_vertices(
        face_extents, scales=None, max_lm_dim=max_lm_dim)
    budget = args.vertex_budget - args.headroom
    if args.headroom <= 0:
        print("  ╔══════════════════════════════════════════════════╗")
        print("  ║  ⚠ WARNING: --headroom is 0!                    ║")
        print("  ║  The solver will fill to 100% of BSP limits.    ║")
        print("  ║  Maps compiled this way may crash the engine    ║")
        print("  ║  when run with vvis -fast or without vvis.      ║")
        print("  ║  Use full vvis.exe for stable results.          ║")
        print("  ╚══════════════════════════════════════════════════╝")
        print()

    # ─── Phase 3: Read BSP lightmap data ──────────────────────────────────────
    print("[3/6] Reading BSP lightmap data...", flush=True)
    lighting_problem = bsp_lighting_problem(bsp_path)
    if lighting_problem:
        # Every face would classify as UNKNOWN: nothing can be detected as
        # uniform and promotions are ordered by priority-0 ties, so the
        # output carries no lighting information at all. Refuse unless the
        # caller asked for a vertex-budget-only run.
        if not getattr(args, 'allow_unlit', False):
            _log_error(f"  ERROR: {bsp_path.name} has no usable lighting: "
                  f"{lighting_problem}. Pass --allow-unlit to solve the "
                  f"vertex budget without lighting classification.")
            return None
        print(f"  ⚠ WARNING (--allow-unlit): {bsp_path.name} has no usable "
              f"lighting ({lighting_problem}); faces classify as UNKNOWN and "
              f"nothing is promoted by lighting detail.",
              file=sys.stderr, flush=True)
    bsp = BSPReader(bsp_path)
    try:
        bsp.read()
    except (OSError, ValueError) as e:
        _log_error(f"  ERROR: cannot read {bsp_path.name}: {e}")
        return None

    # Read actual VBSP vertex count for calibration
    bsp_actual_verts = bsp.read_vertex_count()
    print(f"  BSP actual vertex count: {bsp_actual_verts:,}", flush=True)

    # Detect what lightmapscale the BSP was compiled at
    bsp_compile_scale = bsp.get_bsp_lightmapscale()
    print(f"  BSP compilation lightmapscale: {bsp_compile_scale}", flush=True)

    # Compute raw estimate at the BSP's compilation scale (not VMF's current)
    bsp_scale_overrides = {fe.side_id: bsp_compile_scale for fe in face_extents}
    raw_at_bsp_scale, _ = estimate_map_vertices(
        face_extents, scales=bsp_scale_overrides, max_lm_dim=max_lm_dim)

    # Calibration factor: BSP truth / our raw estimate at the SAME scale
    if raw_at_bsp_scale > 0:
        calibration_factor = bsp_actual_verts / raw_at_bsp_scale
    else:
        calibration_factor = 1.0
    print(f"  Calibration factor: {calibration_factor:.4f} "
          f"(raw@scale{bsp_compile_scale}: {raw_at_bsp_scale:,})", flush=True)

    # Show calibrated initial estimate (at VMF's current scales)
    cal_initial_verts = int(raw_initial_verts * calibration_factor)
    if cal_initial_verts > budget:
        over = cal_initial_verts - budget
        pct = over / budget * 100
        print(f"\n  ⚠ Current VMF: ~{cal_initial_verts:,} calibrated vertices", flush=True)
        print(f"    OVER budget by {over:,} ({pct:.1f}%)", flush=True)
    else:
        under = budget - cal_initial_verts
        print(f"\n  Current VMF: ~{cal_initial_verts:,} calibrated vertices", flush=True)
        print(f"    Under budget by {under:,}", flush=True)
    print()

    try:
        bsp_faces = bsp.extract_all_face_data(verbose=args.verbose)
    except (OSError, ValueError) as e:
        _log_error(f"  ERROR: cannot extract lightmap data from {bsp_path.name}: {e}")
        return None
    t3 = time.perf_counter()
    print(f"  Extracted {len(bsp_faces)} renderable faces in {t3 - t2:.2f}s",
          flush=True)


    # ─── Phase 4: Match BSP faces to VMF sides ───────────────────────────────
    print("[4/6] Matching BSP faces to VMF sides...", flush=True)

    # Compute plane normals for VMF sides from their plane points
    vmf_side_info = _build_vmf_side_plane_info(root)
    print(f"  {len(vmf_side_info)} VMF sides with plane data", flush=True)

    # Run two-phase matching
    face_data = match_bsp_to_vmf(bsp_faces, vmf_side_info, verbose=args.verbose)
    t4 = time.perf_counter()
    print(f"  Matched {len(face_data)} VMF sides in {t4 - t3:.2f}s", flush=True)

    if not face_data:
        _log_error("\n  ERROR: No BSP faces could be matched to VMF sides — is "
              "this BSP compiled from this VMF?")
        return None

    culled_sides = _find_culled_sides(
        bsp, face_extents, side_map,
        {gf.side_id: gf.vertices for gf in all_geometry_faces})

    # ─── Build texlight (emissive) side set unconditionally ────────────────────
    # This is needed by both the visibility oracle AND the budget solver.
    texlight_sides: set = set()
    texlight_mats: set = _collect_texlight_materials(
        game_dir, tools.lights_rad,
        extra_rads=[_level_rad_path(Path(args.input))], vrad_exe=tools.vrad,
        bin_root=tools.bin_root)
    for side_id in face_data.keys():
        node = side_map.get(side_id)
        mat = (node.get_property('material') or '').upper().replace('\\', '/').strip('/') if node else ''
        if mat in texlight_mats:
            texlight_sides.add(side_id)
    if texlight_sides:
        print(f"  {len(texlight_sides)} texlight (emissive) sides identified",
              flush=True)

    # ─── Phase 4.1: Visibility oracle (never-visible → uniform dark) ──────────
    never_visible_sides: set = set()
    raw_nv_sides: set = set()   # filled by the bspwalk cross-reference below
    nv_bsp_faces: set = set()
    do_vis = getattr(args, 'visibility_check', False)

    if do_vis:
        # bspwalk: hidden faces in the tiers --bspwalk-tiers selects (default A,B)
        from bspwalk_runner import (run_bspwalk, summary, spacing_warnings, cubemap_report,
                                    void_reason, split_args, BspwalkError)
        import subprocess
        print("  Running bspwalk...", flush=True)
        # _preflight validated exe, game dir, tiers and extra args via
        # bspwalk_runner.validate_setup; the broad except is the backstop so
        # no bspwalk failure mode (a start-up OSError, a timeout, a bad
        # argument) can end as a traceback or, worse, as a silent exit 0 with
        # no output. run_bspwalk always streams bspwalk's console; keep this
        # tool's old quiet default unless --verbose (failures still carry the
        # last lines of bspwalk's output in the error text). A profile fallback
        # warning goes to stderr whatever `log` is, and the spacing warnings
        # below (tier B sampled coarser than asked for: its never-visible faces
        # are then less certain) go to stderr too, so both show without
        # --verbose. The cubemap findings and a void result (a viewpoint inside
        # solid: tier A proves nothing, tier B is skipped, 0 faces hidden)
        # are reported from the JSON for the same reason: bspwalk's console
        # warned about them, but the quiet log used to discard that and the
        # run carried on as if the check had worked (d1_trainstation_02, #82).
        try:
            nv_bsp_faces, bw = run_bspwalk(
                bsp_path, game_dir, exe=tools.bspwalk,
                tiers=args.bspwalk_tiers, extra_args=args.bspwalk_args,
                threads=args.vis_workers, timeout=args.bspwalk_timeout,
                verbose=args.verbose,
                bin_root=str(tools.bin_root) if tools.bin_root else None,
                log=None if args.verbose else _discard_log)
        except (BspwalkError, OSError, ValueError, TypeError,
                subprocess.SubprocessError) as e:
            _log_error(f"  ERROR: bspwalk failed: {e}")
            return None
        # The per-cubemap lines are warnings (stderr); the summary line (last) is
        # information. Printed before the void check because a disqualified cubemap
        # is often why the result is the way it is. With --verbose bspwalk's own
        # console, which prints the same per-cubemap warnings, was streamed above, so
        # only the summary is added then.
        report = cubemap_report(bw)
        if not args.verbose:
            # a map can drop a hundred cubemaps: name the first ten, the JSON keeps them all
            shown = report[:-1][:10]
            for line in shown:
                print(f"  WARNING: bspwalk {line}", file=sys.stderr, flush=True)
            if len(report) - 1 > len(shown):
                print(f"  (and {len(report) - 1 - len(shown)} more cubemap warnings; "
                      f"--verbose lists them all)", flush=True)
        if report:
            print(f"  bspwalk {report[-1]}", flush=True)
        void = void_reason(bw, where_hint=(
            "bspwalk's console output above lists them" if args.verbose else
            "--verbose prints bspwalk's console output, which lists them"))
        if void is not None:
            # Same exit as a bspwalk failure: the caller turns None into a nonzero
            # exit, no output VMF is written and the scratch files are cleaned up.
            keep = any(a.lower() == '-keepcubemaps' for a in split_args(args.bspwalk_args))
            _log_error(f"  ERROR: the bspwalk visibility check is void: {void}.\n"
                       f"  Fix: move those viewpoints into open space and recompile, "
                       f"or run without --visibility-check"
                       + (", or drop -keepcubemaps from --bspwalk-args so bspwalk can "
                          "disqualify those cubemaps itself." if keep else "."))
            return None
        print(f"  {summary(bw, warnings=False)}; {len(nv_bsp_faces)} faces never visible "
              f"(tiers {args.bspwalk_tiers})", flush=True)
        for warning in spacing_warnings(bw):
            print(f"  WARNING: bspwalk {warning}", file=sys.stderr, flush=True)

        # Cross-reference: override luminances for never-visible VMF sides
        raw_nv_sides, never_visible_sides = _mark_never_visible(
            face_data, nv_bsp_faces, texlight_sides)
        if nv_bsp_faces:
            print(f"  {len(never_visible_sides)} VMF sides → max scale "
                  f"(never-visible), {len(texlight_sides)} texlights skipped in map", flush=True)

    # ─── Phase 4.5: Brush carving (--chop) ────────────────────────────────────
    carve_results = []
    side_remap: Dict[int, List[int]] = {}   # stays empty unless --chop carves (the scale report reads it)
    promoted_side_ids: Set[int] = set()

    if getattr(args, 'chop', False):
        print("[4.5/6] Aggressive brush carving...", flush=True)
        from brush_carver import find_carve_candidates, apply_carves
        
        candidates = find_carve_candidates(
            lighting_data=face_data,
            brushes=brushes,
            dark_floor=args.dark_threshold,
            verbose=args.verbose,
        )
        
        if candidates:
            if args.dry_run:
                print(f"  [DRY RUN] {len(candidates)} candidate faces; a brush "
                      f"is only carved if a piece is wholly dark, decided "
                      f"on a real run", flush=True)
                for c in candidates:
                    print(f"    • brush {c.brush_id}, side {c.side_id}: "
                          f"{len(c.uniform_regions)} dark + "
                          f"{len(c.varied_regions)} other sub-faces", flush=True)
            else:
                carve_results, promoted_side_ids, side_remap = apply_carves(
                    vmf_root=root,
                    candidates=candidates,
                    face_extents=face_extents,
                    verbose=True,
                )
                
                if carve_results:
                    total_savings = sum(r.estimated_savings for r in carve_results)
                    total_new = sum(r.new_brush_count for r in carve_results)
                    print(f"  Carved {len(carve_results)} brushes into "
                          f"{total_new} pieces (est. savings: ~{total_savings} verts)",
                          flush=True)
                    
                    # Rebuild geometry and face extents for the modified VMF
                    print("  Rebuilding geometry after carving...", flush=True)
                    brushes = extract_brushes(root)
                    brush_entity_map = _build_brush_entity_map(root)
                    all_geometry_faces = build_all_faces(brushes)
                    
                    face_extents = []
                    side_map = _build_vmf_side_map(root)
                    
                    for gf in all_geometry_faces:
                        side_node = side_map.get(gf.side_id)
                        if side_node is None:
                            continue
                        uaxis_str = side_node.get_property('uaxis') or ''
                        vaxis_str = side_node.get_property('vaxis') or ''
                        entity_class = brush_entity_map.get(gf.brush_id, '')
                        fe = compute_face_extent(
                            gf.vertices, uaxis_str, vaxis_str,
                            gf.lightmapscale, gf.side_id, gf.material,
                            brush_entity=entity_class,
                            has_displacement=bool(side_node.get_children_by_name('dispinfo')),
                        )
                        face_extents.append(fe)
                    
                    print(f"  Rebuilt: {len(brushes)} brushes, "
                          f"{len(face_extents)} face extents", flush=True)
                    
                    # Carved pieces have new side IDs. Every side except the
                    # promoted dark faces inherits its original side's
                    # lighting data; with none, the solver would read them as
                    # the most uniform faces in the map and degrade them first.
                    inherited = 0
                    for old_sid, new_sids in side_remap.items():
                        old_ld = face_data.get(old_sid)
                        if old_ld is None:
                            continue
                        for sid in new_sids:
                            if sid not in promoted_side_ids:
                                # new entry per side; luminances and
                                # sub_faces lists stay shared (read-only
                                # after this point)
                                face_data[sid] = dataclasses.replace(
                                    old_ld, vmf_side_id=sid,
                                    bsp_face_indices=list(old_ld.bsp_face_indices))
                                inherited += 1
                    # Emissive and never-visible sides keep their status
                    # on their carved pieces
                    for old_sid, new_sids in side_remap.items():
                        if old_sid in texlight_sides:
                            texlight_sides.update(new_sids)
                        if old_sid in raw_nv_sides:
                            raw_nv_sides.update(new_sids)
                        if old_sid in never_visible_sides:
                            never_visible_sides.update(new_sids)
                        if old_sid in culled_sides:
                            culled_sides.update(new_sids)
                    if inherited:
                        print(f"  {inherited} carved sides inherit their original "
                              f"side's lighting data", flush=True)

                    # Inject synthetic lighting data for promoted carved faces
                    # so the solver's pre-promotion step treats them as uniform
                    if promoted_side_ids:
                        from bsp_reader import FaceLightmapData
                        for sid in promoted_side_ids:
                            if sid not in face_data:
                                face_data[sid] = FaceLightmapData(
                                    vmf_side_id=sid,
                                    material='',
                                    bsp_face_indices=[],
                                    luminances=[0.0],
                                )
                        print(f"  Injected {len(promoted_side_ids)} synthetic "
                              f"lighting entries for carved dark-uniform faces",
                              flush=True)
                else:
                    print("  No viable carves found", flush=True)
        else:
            print("  No carve candidates found (all faces are "
                  "fully uniform or fully varied)", flush=True)

    # Culled sides read as uniform dark from here on (after carving, which
    # must not pick them as candidates): with no lighting data the solver
    # would class them as detailed and promote them, often dragged to scale 1
    # by a visible coplanar neighbour.
    culled_sides &= {fe.side_id for fe in face_extents}
    _inject_culled_lighting(face_data, culled_sides)

    # ─── Phase 5: Solve vertex budget ─────────────────────────────────────────
    vbsp_path = tools.vbsp
    game_dir = tools.game_dir
    coplanar_mode = getattr(args, 'coplanar', 'seam')
    if coplanar_mode == 'off':
        print("  Coplanar grouping off (--coplanar off): every side is "
              "solved on its own", flush=True)

    # Per-side scale floors that no later stage may lower: %detailtype
    # minimums, the displacement extent clamp (VBSP's fatal 'Bad surface
    # extents') and the texlight floor the baseline light ladder found.
    # Coplanar unification, in particular, would otherwise drag a face down
    # to a finer neighbour's scale.
    floor_kinds: Dict[int, List[str]] = {}   # side_id -> why it has a floor (the scale report)
    side_floors = _compute_side_floors(
        face_extents, detail_type_materials, detail_min_scale,
        texlight_mats=texlight_mats, emissive_floor=emissive_floor,
        kinds=floor_kinds)
    for sid in vanished_disps:
        side_floors[sid] = max(side_floors.get(sid, 1), _DISP_NO_GEOMETRY_FLOOR)
        floor_kinds.setdefault(sid, []).append('disp_no_geometry')
    # VBSP drops culled sides, so their scale costs no BSP vertices (only the
    # calibrated estimate still counts them); Hammer's default keeps a tiny
    # one (max useful scale 1) from reading as a promoted face in the editor.
    for sid in culled_sides:
        side_floors[sid] = max(side_floors.get(sid, 1), _CULLED_SIDE_FLOOR)
        floor_kinds.setdefault(sid, []).append('culled')

    # Everything the scale report needs from this stage that is not already in
    # the result; stage_changes are filled as the post-solver stages run.
    report_data = {
        'face_extents': face_extents,
        'culled_sides': culled_sides,
        'side_floors': side_floors,
        'floor_kinds': floor_kinds,
        'side_remap': side_remap,
        'promoted_side_ids': promoted_side_ids,
        'stage_changes': {},
    }

    if vbsp_path and game_dir:
        # ─── VBSP-in-the-loop solver (ground truth) ───────────────────────────────
        print("[5/6] Solving vertex budget with VBSP ground truth...", flush=True)
        from vbsp_solver import solve_with_vbsp
        
        # Seam groups (_compute_coplanar_groups): sides a lightmap seam would
        # show between. Strict mode does not promote them atomically, but the
        # opt-in evidence floors still share one density across each group.
        # Off computes none: every side is promoted and floored on its own.
        strict = coplanar_mode == 'strict'
        smooth_tol = getattr(args, 'smooth_tolerance', None)
        plane_grps = None
        if coplanar_mode != 'off' and (not strict or smooth_tol is not None):
            # Culled sides stay out: VBSP dropped them, and through AABB
            # adjacency they would bridge visible regions into one group.
            face_verts = {gf.side_id: gf.vertices for gf in all_geometry_faces
                          if gf.side_id not in culled_sides}
            plane_grps = _compute_coplanar_groups(
                side_map=side_map,
                face_vertices=face_verts,
                face_extents=face_extents,
                detail_type_materials=detail_type_materials,
                lighting_data=face_data,
                never_visible=never_visible_sides,
            )
        coplanar_grps = None if strict else plane_grps
        if coplanar_grps:
            total_faces = sum(len(g) for g in coplanar_grps)
            print(f"  {len(coplanar_grps)} coplanar seam groups "
                  f"({total_faces} faces: shared edge, same material, "
                  f"detail across it) will be promoted atomically",
                  flush=True)

        # Opt-in floors from the baseline lightmap: sides it reproduces at a
        # coarser scale (the uniform test only takes dark ones).
        smooth_scales = None
        if smooth_tol is not None:
            smooth_scales = _smooth_evidence(face_data, smooth_tol,
                                             never_visible_sides | culled_sides)

        input_path = Path(args.input).resolve()
        vbsp_result = solve_with_vbsp(
            face_extents=face_extents,
            lighting_data=face_data,
            vmf_root=root,
            side_map=side_map,
            vbsp_exe=vbsp_path,
            game_dir=game_dir,
            input_vmf=input_path,
            bin_root=getattr(args, 'binroot', None),
            vertex_budget=budget,
            max_lm_dim=max_lm_dim,
            detail_type_materials=detail_type_materials,
            detail_min_scale=detail_min_scale,
            coplanar_groups=coplanar_grps,
            emissive_sides=texlight_sides if texlight_sides else None,
            uniform_max_luminance=args.dark_threshold,
            side_min_scales=side_floors,
            evidence_scales=smooth_scales,
            evidence_groups=plane_grps,
            skip_ahead_calls=args.skip_ahead_calls,
            promote_order=args.promote_order,
            verbose=True,
        )
        
        # What the solver itself chose, before the stages below move it: the
        # scale report's solver_scale, with changed_by naming those stages.
        report_data['solver_scales'] = dict(vbsp_result.scales)

        # Post-solver unification only for strict mode. It sets a group to
        # ceil(extent/32), which can sit BELOW a member's floor (a %detailtype
        # minimum or texlight floor), so floors are re-applied last.
        unified = 0
        if strict:
            before_unify = dict(vbsp_result.scales)
            unified = _unify_coplanar_strict(
                scales=vbsp_result.scales,
                side_map=side_map,
                face_data=face_data,
                face_extents=face_extents,
                max_lm_dim=max_lm_dim,
                verbose=True,
                exclude=culled_sides,
            )
            report_data['stage_changes']['strict_unify'] = {
                sid for sid, scale in vbsp_result.scales.items()
                if before_unify.get(sid) != scale}
        raised_by_floors: Set[int] = set()
        _apply_scale_floors(vbsp_result.scales, side_map, side_floors,
                            raised_ids=raised_by_floors)
        report_data['stage_changes']['floors'] = raised_by_floors

        # Re-apply unified scales to VMF
        if unified > 0:
            for side_id, new_scale in vbsp_result.scales.items():
                node = side_map.get(side_id)
                if node:
                    _set_lightmapscale(node, new_scale)
        
        modified = sum(1 for sid, scale in vbsp_result.scales.items()
                       if scale != _get_lightmapscale_default(side_map.get(sid)))
        
        return {
            'vbsp_solver_result': vbsp_result,
            'modified': modified,
            'never_visible_sides': never_visible_sides,
            'raw_nv_sides': raw_nv_sides,
            'texlight_sides': texlight_sides,
            'texlight_mats': texlight_mats,
            'report_data': report_data,
        }
    else:
        # ─── Calibration-factor solver (fallback) ─────────────────────────────
        print("[5/6] Solving vertex budget (calibration fallback)...", flush=True)
        if not vbsp_path:
            print("  (No vbsp_lmo.exe found — using calibration factor)", flush=True)
        if not game_dir:
            print("  (No --game dir specified — using calibration factor)", flush=True)
        if getattr(args, 'smooth_tolerance', None) is not None:
            print("  WARNING: --smooth-tolerance needs the VBSP solver and is "
                  "ignored by the calibration fallback", file=sys.stderr, flush=True)
        
        solver_result = solve_vertex_budget(
            face_extents=face_extents,
            lighting_data=face_data,
            vertex_budget=budget,
            max_lm_dim=max_lm_dim,
            calibration_factor=calibration_factor,
            detail_type_materials=detail_type_materials,
            detail_min_scale=detail_min_scale,
            emissive_sides=texlight_sides if texlight_sides else None,
            uniform_max_luminance=args.dark_threshold,
            verbose=args.verbose or True,  # Always show solver progress
        )
        
        report_data['solver_scales'] = dict(solver_result.scales)
        before_unify = dict(solver_result.scales)

        # Unify coplanar groups (nothing when --coplanar off)
        if coplanar_mode == 'strict':
            _unify_coplanar_strict(
                scales=solver_result.scales,
                side_map=side_map,
                face_data=face_data,
                face_extents=face_extents,
                max_lm_dim=max_lm_dim,
                verbose=True,
                exclude=culled_sides,
            )
        elif coplanar_mode == 'seam':
            # Build vertex lookup from geometry faces for adjacency testing
            face_verts = {gf.side_id: gf.vertices for gf in all_geometry_faces
                          if gf.side_id not in culled_sides}
            _unify_coplanar_groups(
                scales=solver_result.scales,
                side_map=side_map,
                face_data=face_data,
                face_extents=face_extents,
                face_vertices=face_verts,
                detail_type_materials=detail_type_materials,
                min_scales=side_floors,
                verbose=True,
            )
        if coplanar_mode in ('strict', 'seam'):
            report_data['stage_changes'][
                'strict_unify' if coplanar_mode == 'strict' else 'seam_unify'] = {
                sid for sid, scale in solver_result.scales.items()
                if before_unify.get(sid) != scale}
        # Floors last, after whichever unifier ran (none with --coplanar off):
        # nothing may end below its floor
        # (the fallback has no VBSP check, so a lowered displacement would
        # reach the output VMF and fatal VBSP's extent check at compile time).
        raised_by_floors = set()
        _apply_scale_floors(solver_result.scales, side_map, side_floors,
                            write_nodes=False, raised_ids=raised_by_floors)
        report_data['stage_changes']['floors'] = raised_by_floors
        
        # Apply scales to VMF
        modified = 0
        if not args.dry_run:
            for side_id, new_scale in solver_result.scales.items():
                node = side_map.get(side_id)
                if node is None:
                    continue
                current = _get_lightmapscale(node)
                if new_scale != current:
                    _set_lightmapscale(node, new_scale)
                    modified += 1
        else:
            for side_id, new_scale in solver_result.scales.items():
                node = side_map.get(side_id)
                if node is None:
                    continue
                current = _get_lightmapscale(node)
                if new_scale != current:
                    modified += 1
        
        return {
            'solver_result': solver_result,
            'modified': modified,
            'never_visible_sides': never_visible_sides,
            'raw_nv_sides': raw_nv_sides,
            'texlight_sides': texlight_sides,
            'texlight_mats': texlight_mats,
            'report_data': report_data,
        }


def _build_brush_entity_map(root) -> Dict[int, str]:
    """Map brush IDs to their parent entity classnames.
    
    Returns a dict: brush_id → entity_classname.
    Worldspawn brushes map to "" (empty string).
    func_detail brushes map to "func_detail", etc.
    """
    brush_entity: Dict[int, str] = {}

    # Worldspawn brushes
    for world_node in root.get_children_by_name('world'):
        for solid in world_node.get_children_by_name('solid'):
            bid = solid.get_property('id')
            if bid:
                brush_entity[int(bid)] = ''

    # Entity brushes
    for entity_node in root.get_all_recursive('entity'):
        classname = entity_node.get_property('classname') or ''
        for solid in entity_node.get_children_by_name('solid'):
            bid = solid.get_property('id')
            if bid:
                brush_entity[int(bid)] = classname

    return brush_entity


def _build_vmf_side_plane_info(root) -> dict:
    """Build VMF side plane info for BSP matching: side_id → {normal, dist, material}."""
    from bsp_reader import _canonicalize_plane
    vmf_side_info = {}
    for side_node in root.get_all_recursive('side'):
        sid_str = side_node.get_property('id')
        if not sid_str:
            continue
        sid = int(sid_str)
        plane_str = side_node.get_property('plane') or ''
        material = side_node.get_property('material') or ''

        groups = re.findall(r'\(([^)]+)\)', plane_str)
        if len(groups) < 3:
            continue

        pts = []
        for g in groups[:3]:
            parts = g.strip().split()
            pts.append((float(parts[0]), float(parts[1]), float(parts[2])))

        p1, p2, p3 = pts
        e1 = (p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2])
        e2 = (p3[0]-p1[0], p3[1]-p1[1], p3[2]-p1[2])
        nx = e1[1]*e2[2] - e1[2]*e2[1]
        ny = e1[2]*e2[0] - e1[0]*e2[2]
        nz = e1[0]*e2[1] - e1[1]*e2[0]
        length = math.sqrt(nx*nx + ny*ny + nz*nz)
        if length < 1e-10:
            continue
        nx /= length
        ny /= length
        nz /= length
        dist = nx*p1[0] + ny*p1[1] + nz*p1[2]

        # Canonicalize so VMF normals match BSP convention
        nx, ny, nz, dist = _canonicalize_plane(nx, ny, nz, dist)

        vmf_side_info[sid] = {
            'normal': (nx, ny, nz),
            'dist': dist,
            'material': material,
        }

    return vmf_side_info


def _build_vmf_side_map(root) -> dict:
    """Build a mapping from VMF side ID → side KVNode for in-place modification."""
    side_map = {}
    for side_node in root.get_all_recursive('side'):
        side_id_str = side_node.get_property('id')
        if side_id_str:
            try:
                side_map[int(side_id_str)] = side_node
            except ValueError:
                pass
    return side_map


def _unify_coplanar_strict(
    scales: Dict[int, int],
    side_map: dict,
    face_data: dict,
    face_extents: list,
    max_lm_dim: int = 32,
    verbose: bool = False,
    exclude: Optional[Set[int]] = None,
) -> int:
    """Unify lightmapscale for coplanar face groups with uniform lighting.

    Sides in `exclude` (culled by VBSP) never join a group.
    
    When multiple brushes share a coplanar face with the same material and
    texture alignment, and ALL faces have perfectly uniform BSP lighting,
    assign them all the same max_useful_scale based on the combined extent.
    
    This targets void-box scenarios where manually carved brush lids produce
    multiple faces with different scales that should match.
    
    Modifies `scales` dict in-place. Returns the number of faces unified.
    """
    from vertex_estimator import min_scale_no_subdivision
    import re
    
    # Build extent lookup
    extent_by_id = {fe.side_id: fe for fe in face_extents}
    
    # Group faces by coplanar key: (plane_key, material, uaxis, vaxis)
    groups: Dict[tuple, list] = {}
    
    skip = exclude or set()
    for side_id in side_map:
        node = side_map.get(side_id)
        if node is None or side_id in skip:
            continue
        
        plane_str = node.get_property('plane')
        material = (node.get_property('material') or '').upper()
        uaxis = node.get_property('uaxis')
        vaxis = node.get_property('vaxis')
        
        if not plane_str or not uaxis or not vaxis:
            continue
        
        # Parse plane to get normal and distance for grouping
        # Round to avoid floating-point mismatch
        matches = re.findall(r'\(([^)]+)\)', plane_str)
        if len(matches) < 3:
            continue
        pts = [tuple(float(x) for x in m.split()) for m in matches[:3]]
        
        # Compute plane normal from 3 points
        v1 = (pts[1][0] - pts[0][0], pts[1][1] - pts[0][1], pts[1][2] - pts[0][2])
        v2 = (pts[2][0] - pts[0][0], pts[2][1] - pts[0][1], pts[2][2] - pts[0][2])
        nx = v1[1]*v2[2] - v1[2]*v2[1]
        ny = v1[2]*v2[0] - v1[0]*v2[2]
        nz = v1[0]*v2[1] - v1[1]*v2[0]
        length = (nx*nx + ny*ny + nz*nz) ** 0.5
        if length < 1e-8:
            continue
        nx, ny, nz = nx/length, ny/length, nz/length
        dist = nx*pts[0][0] + ny*pts[0][1] + nz*pts[0][2]
        
        # Round for grouping (avoid float noise)
        plane_key = (round(nx, 4), round(ny, 4), round(nz, 4), round(dist, 1))
        
        group_key = (plane_key, material, uaxis, vaxis)
        groups.setdefault(group_key, []).append(side_id)
    
    # Process groups with 2+ faces
    unified_count = 0
    for group_key, side_ids in groups.items():
        if len(side_ids) < 2:
            continue
        
        # Check: group qualifies if all BSP-matched faces have near-uniform
        # lighting. Faces WITHOUT BSP data are OK (BSP merged them away).
        # At least one face must have REAL BSP data (not synthetic/injected).
        has_any_bsp = False
        all_eligible = True
        for sid in side_ids:
            ld = face_data.get(sid)
            if ld is None:
                continue  # No BSP data = face was merged away, eligible
            # Only count real BSP data (synthetic injected data has empty indices)
            is_real_bsp = bool(ld.bsp_face_indices)
            if is_real_bsp:
                has_any_bsp = True
            # is_perfectly_uniform is the whole test: it is False for a face
            # with NO samples (unknown), which a 'luminance_range <= 0' clause
            # would re-admit (an empty list has range 0.0).
            if not ld.is_perfectly_uniform:
                all_eligible = False
                break
        
        if not all_eligible or not has_any_bsp:
            continue
        
        # Compute max_useful_scale from the largest face extent in the group.
        # This is the scale that eliminates subdivision for the biggest piece,
        # ensuring all pieces in the group share that same scale.
        max_extent_s = 0.0
        max_extent_t = 0.0
        for sid in side_ids:
            fe = extent_by_id.get(sid)
            if fe:
                max_extent_s = max(max_extent_s, fe.extent_s)
                max_extent_t = max(max_extent_t, fe.extent_t)
        
        if max_extent_s < 1e-6 and max_extent_t < 1e-6:
            continue
        
        unified_scale = min_scale_no_subdivision(max_extent_s, max_extent_t, max_lm_dim)
        
        # Sanity cap: don't produce unreasonably large scales
        if unified_scale > 16:
            continue
        
        # Apply to all members
        for sid in side_ids:
            scales[sid] = unified_scale
        
        unified_count += len(side_ids)
        if verbose:
            material = group_key[1]
            print(f"  Unified {len(side_ids)} coplanar uniform faces "
                  f"(mat={material}, scale={unified_scale})", flush=True)
    
    return unified_count


def _mark_never_visible(face_data: dict, nv_bsp_faces: set,
                        texlight_sides: set):
    """The visibility override: a side whose BSP faces are all never-visible
    gets luminances [0.0] (so it reads as uniform dark), texlights excepted.
    Its sub-faces keep their real lighting. Returns (raw, overridden) sides."""
    raw_nv_sides: set = set()
    never_visible_sides: set = set()
    if nv_bsp_faces:
        for side_id, fld in face_data.items():
            bsp_indices = set(fld.bsp_face_indices)
            if bsp_indices and bsp_indices.issubset(nv_bsp_faces):
                raw_nv_sides.add(side_id)

        if raw_nv_sides:
            never_visible_sides = raw_nv_sides - texlight_sides

            for side_id in never_visible_sides:
                fld = face_data[side_id]
                fld.luminances = [0.0]
    return raw_nv_sides, never_visible_sides


def _plane_of(verts) -> Optional[Tuple[Tuple[float, float, float], float]]:
    """(unit normal, dist) of a polygon by Newell's method, sign canonical
    (first nonzero component positive): facing is ignored."""
    nx = ny = nz = 0.0
    n = len(verts)
    for i in range(n):
        x1, y1, z1 = verts[i]
        x2, y2, z2 = verts[(i + 1) % n]
        nx += (y1 - y2) * (z1 + z2)
        ny += (z1 - z2) * (x1 + x2)
        nz += (x1 - x2) * (y1 + y2)
    length = (nx * nx + ny * ny + nz * nz) ** 0.5
    if length < 1e-9:
        return None
    nx, ny, nz = nx / length, ny / length, nz / length
    for c in (nx, ny, nz):
        if abs(c) > 1e-4:
            if c < 0:
                nx, ny, nz = -nx, -ny, -nz
            break
    x, y, z = verts[0]
    return (nx, ny, nz), nx * x + ny * y + nz * z


def _aabb(verts):
    return (tuple(min(v[i] for v in verts) for i in range(3)),
            tuple(max(v[i] for v in verts) for i in range(3)))


def _find_culled_sides(bsp, face_extents: list, side_map: dict,
                       face_vertices: dict) -> set:
    """Sides VBSP dropped from the BSP: in the VMF geometry, on no BSP face.

    Needs the -emitsideids lumps; without them a side missing from face_data
    may only have failed the geometric match, so nothing is called culled.
    Displacements are left out (their own floor handling covers a missing one).

    Lump version 1 (vbsp_lmo since #73) lists every merged side, so a side
    missing from the lumps is culled. Version 0 lumps miss merged detail
    faces: the old MakeBrushFace never seeded a detail face's merged-ID list,
    so a merge wrote only its first side. VBSP routes a side onto that path
    by CONTENTS_DETAIL, which func_detail and %compileDetail materials (on
    world brushes too) both set, so for version 0 every side missing from the
    lumps is tested: it is culled only when no BSP face (lit or not) lies on
    its plane and overlaps it. A merged face covers every side merged into it.
    Geometric, so VBSP's material renames (cubemap, WVT, water depth) do not
    matter. Conservative: a hidden side
    coplanar with an overlapping face keeps its old handling, and so does
    every candidate when the BSP yields no usable face to test against.
    """
    from vertex_estimator import should_skip_face
    lump = bsp.read_face_side_ids()
    if lump is None:
        print("  (BSP has no side-ID lumps: culled-side detection skipped)",
              flush=True)
        return set()
    emitted = set()
    for ids in lump:
        if ids:
            emitted.update(ids)

    candidates = []
    culled = set()
    for fe in face_extents:
        sid = fe.side_id
        if (sid in emitted or fe.has_displacement or side_map.get(sid) is None
                or should_skip_face(fe.material, fe.brush_entity)):
            continue
        candidates.append(sid)

    if bsp.face_side_ids_version() >= 1:
        print(f"  {len(candidates)} VMF sides culled by VBSP (no BSP face)",
              flush=True)
        return set(candidates)

    held = 0
    if candidates:
        # Each BSP face's plane comes from the planes lump (what VBSP built
        # from the VMF plane points), not refitted from float32 vertices,
        # whose tilt drifts the dist far from the origin. Bucketed by integer
        # dist; normals compared with a dot tolerance.
        faces = bsp.read_faces()
        planes = bsp.read_planes()
        vx, ed, se = bsp.read_vertexes(), bsp.read_edges(), bsp.read_surfedges()
        buckets: Dict[int, list] = {}
        for f in faces:
            verts = bsp.get_face_vertices(f, vx, ed, se)
            if len(verts) < 3 or not (0 <= f.planenum < len(planes)):
                continue
            plane = planes[f.planenum]
            n, d = plane.normal, plane.dist
            for c in n:          # same sign convention as _plane_of
                if abs(c) > 1e-4:
                    if c < 0:
                        n, d = (-n[0], -n[1], -n[2]), -d
                    break
            buckets.setdefault(int(round(d)), []).append((n, d, _aabb(verts)))
        if not buckets:
            # nothing to test against: fail closed, keep the old handling
            print("  (BSP faces unreadable: culled-side detection skipped)",
                  flush=True)
            return set()

        def _covered(sid: int) -> bool:
            verts = face_vertices.get(sid)
            if not verts or len(verts) < 3:
                return True          # unknown shape: keep the old handling
            pl = _plane_of(verts)
            if pl is None:
                return True
            (nx, ny, nz), d = pl
            mins, maxs = _aabb(verts)
            # both signs: a near-zero component may canonicalize differently
            for sign in (1.0, -1.0):
                sd = sign * d
                for di in (int(round(sd)) - 1, int(round(sd)), int(round(sd)) + 1):
                    for (bx, by, bz), bd, (bmins, bmaxs) in buckets.get(di, ()):
                        if sign * (nx * bx + ny * by + nz * bz) < 1.0 - 1e-4:
                            continue
                        if abs(bd - sd) > 0.1:
                            continue
                        # a real overlap in the plane, however thin; a face
                        # that only shares an edge overlaps by ~0 there
                        overlaps = [min(maxs[i], bmaxs[i]) - max(mins[i], bmins[i])
                                    for i in range(3)]
                        # -0.1 along the normal matches the dist tolerance
                        if (min(overlaps) >= -0.1
                                and sum(1 for o in overlaps if o > 0.01) >= 2):
                            return True
            return False

        for sid in candidates:
            if _covered(sid):
                held += 1
            else:
                culled.add(sid)
    print(f"  {len(culled)} VMF sides culled by VBSP (no BSP face); {held} "
          f"sides without one kept (a merged face covers them)",
          flush=True)
    return culled


def _inject_culled_lighting(face_data: dict, culled_sides: set) -> int:
    """Give each culled side without lighting data a uniform-dark entry, the
    same synthetic shape carving uses (no BSP face indices), so the solver
    sets it to its max useful scale instead of promoting it."""
    from bsp_reader import FaceLightmapData
    added = 0
    for sid in culled_sides:
        if sid not in face_data:
            face_data[sid] = FaceLightmapData(
                vmf_side_id=sid, material='', bsp_face_indices=[],
                luminances=[0.0])
            added += 1
    return added


def _smooth_evidence(face_data: dict, tolerance: float,
                     never_visible_sides: set) -> Dict[int, int]:
    """--smooth-tolerance: {side_id: coarsest reproducible scale}, printed as
    one summary line. Never-visible sides are left out: the override only
    flattened their luminances, and their sub-faces' real lighting must not
    set a coplanar group's floor."""
    from collections import Counter
    from lightmap_evidence import evidence_scales
    visible = {sid: ld for sid, ld in face_data.items()
               if sid not in never_visible_sides}
    scales = evidence_scales(visible, tolerance)
    spread = ', '.join(f"{s}: {n}" for s, n in
                       sorted(Counter(scales.values()).items()))
    # Candidates only: the solver caps, groups and compares them with the
    # baseline, and reports the sides it actually raised.
    print(f"  Smooth-side evidence (tolerance {tolerance:g}): "
          f"{len(scales)} of {len(face_data)} sides are candidates (reproduced "
          f"at a coarser scale, before capping and grouping)"
          + (f" (scale: sides  {spread})" if spread else ""), flush=True)
    return scales


# Coplanar promotion groups (issue #74). Two sides are tied only where a
# lightmap seam between them would show: they share an edge, have the same
# material and texture axes (one continuous surface), and the baseline
# lightmap has detail along that edge on both sides (a shadow crossing it).
# Plane + bounding-box adjacency alone chained every z=0 floor of
# d1_trainstation_02 into one 167-face unit no budget could promote.

# Display-space (out of 255) jump between neighbouring luxels along a shared
# edge that counts as detail crossing it. A smooth gradient steps far less
# per luxel; a shadow edge steps far more.
EDGE_DETAIL_STEP = 4.0
# Sample spacing along a shared edge, in luxels of the side's own lattice
# (half a luxel visits every luxel the edge crosses), and a cap per edge.
_EDGE_SAMPLE_STEP = 0.5
_EDGE_MAX_SAMPLES = 4096


def _plane_key(node) -> Optional[tuple]:
    """(nx, ny, nz, dist) rounded for grouping, from a side's plane points."""
    plane_str = node.get_property('plane')
    if not plane_str:
        return None
    matches = re.findall(r'\(([^)]+)\)', plane_str)
    if len(matches) < 3:
        return None
    pts = [tuple(float(x) for x in m.split()) for m in matches[:3]]
    v1 = (pts[1][0] - pts[0][0], pts[1][1] - pts[0][1], pts[1][2] - pts[0][2])
    v2 = (pts[2][0] - pts[0][0], pts[2][1] - pts[0][1], pts[2][2] - pts[0][2])
    nx = v1[1]*v2[2] - v1[2]*v2[1]
    ny = v1[2]*v2[0] - v1[0]*v2[2]
    nz = v1[0]*v2[1] - v1[1]*v2[0]
    length = (nx*nx + ny*ny + nz*nz) ** 0.5
    if length < 1e-8:
        return None
    nx, ny, nz = nx/length, ny/length, nz/length
    dist = nx*pts[0][0] + ny*pts[0][1] + nz*pts[0][2]
    return (round(nx, 4), round(ny, 4), round(nz, 4), round(dist, 1))


def _axis_direction(axis_str: Optional[str]) -> Optional[tuple]:
    """A texture axis's unit direction ('[x y z shift] scale'), rounded. The
    lightmap grid follows the direction only, not the shift or scale."""
    if not axis_str:
        return None
    m = re.search(r'\[([^\]]+)\]', axis_str)
    if not m:
        return None
    parts = m.group(1).split()
    if len(parts) < 3:
        return None
    x, y, z = (float(v) for v in parts[:3])
    length = (x*x + y*y + z*z) ** 0.5
    if length < 1e-8:
        return None
    return (round(x/length, 3), round(y/length, 3), round(z/length, 3))


def _shared_edges(verts_a, verts_b, tol: float = 0.1, min_len: float = 0.5):
    """Every stretch of edge two coplanar polygons share, as world segments
    (p, q). Covers T-junctions (a short edge along a longer one shares its own
    length) and edges split by collinear vertices (one piece per sub-edge)."""
    pieces = []
    na, nb = len(verts_a), len(verts_b)
    for i in range(na):
        p = verts_a[i]
        q = verts_a[(i + 1) % na]
        d = (q[0]-p[0], q[1]-p[1], q[2]-p[2])
        L = (d[0]*d[0] + d[1]*d[1] + d[2]*d[2]) ** 0.5
        if L < 1e-6:
            continue
        u = (d[0]/L, d[1]/L, d[2]/L)
        for j in range(nb):
            ts = []
            for pt in (verts_b[j], verts_b[(j + 1) % nb]):
                w = (pt[0]-p[0], pt[1]-p[1], pt[2]-p[2])
                t = w[0]*u[0] + w[1]*u[1] + w[2]*u[2]
                off = (w[0]-t*u[0], w[1]-t*u[1], w[2]-t*u[2])
                if (off[0]*off[0] + off[1]*off[1] + off[2]*off[2]) > tol*tol:
                    break
                ts.append(t)
            else:
                lo = max(0.0, min(ts))
                hi = min(L, max(ts))
                if hi - lo > min_len:
                    pieces.append(((p[0]+u[0]*lo, p[1]+u[1]*lo, p[2]+u[2]*lo),
                                   (p[0]+u[0]*hi, p[1]+u[1]*hi, p[2]+u[2]*hi)))
    return pieces


def _display(lum: float) -> float:
    """lightmap_evidence.display for one value, without numpy (the default
    VBSP path must not need it)."""
    return 255.0 * (max(lum / 255.0, 0.0) ** (1.0 / 2.2))


def _luxel_at(ld, pt):
    """(sub-face index, i, j, luminance) of the baseline luxel at world point
    pt, or None. Of the sub-faces whose luxel lattice holds the point, the one
    holding it deepest wins: a point near a lattice border may also fall on a
    neighbouring sub-face's extrapolated edge luxels."""
    best = None
    best_depth = -1.0
    for si, sf in enumerate(ld.sub_faces):
        vecs = sf.lightmap_vecs
        if vecs is None or not sf.luminances:
            continue
        s = (pt[0]*vecs[0][0] + pt[1]*vecs[0][1] + pt[2]*vecs[0][2]
             + vecs[0][3] - sf.lightmap_mins[0])
        t = (pt[0]*vecs[1][0] + pt[1]*vecs[1][1] + pt[2]*vecs[1][2]
             + vecs[1][3] - sf.lightmap_mins[1])
        w = sf.lightmap_size[0] + 1
        h = sf.lightmap_size[1] + 1
        # Luxel k covers [k - 0.5, k + 0.5]; the lattice is [-0.5, w - 0.5].
        depth = min(s + 0.5, w - 0.5 - s, t + 0.5, h - 0.5 - t)
        if depth < 0 or depth <= best_depth:
            continue
        # floor(x + 0.5), not round(): round-half-to-even sends border
        # samples to alternating neighbours and skips luxels.
        i = min(max(math.floor(s + 0.5), 0), w - 1)
        j = min(max(math.floor(t + 0.5), 0), h - 1)
        if j * w + i >= len(sf.luminances):
            continue
        best = (si, i, j, sf.luminances[j * w + i])
        best_depth = depth
    return best


def _edge_runs(ld, p, q) -> List[List[float]]:
    """Display-space luminances of the side's baseline luxels along the
    segment p-q: one value per distinct luxel, split into runs wherever no
    luxel covers the segment (a gap is not a neighbour). Empty with no
    lighting. Never-visible sides keep their real sub-face lighting
    (_mark_never_visible only flattens .luminances), so callers keep them
    out by side id, as _compute_coplanar_groups does with never_visible."""
    if ld is None or not getattr(ld, 'sub_faces', None):
        return []
    d = (q[0]-p[0], q[1]-p[1], q[2]-p[2])
    L = (d[0]*d[0] + d[1]*d[1] + d[2]*d[2]) ** 0.5
    # Luxels per world unit: the densest lattice axis over the sub-faces.
    density = max((math.sqrt(sum(c * c for c in v[:3]))
                   for sf in ld.sub_faces if sf.lightmap_vecs is not None
                   for v in sf.lightmap_vecs), default=1.0)
    n = max(2, min(_EDGE_MAX_SAMPLES, int(L * density / _EDGE_SAMPLE_STEP) + 1))
    runs: List[List[float]] = []
    run: List[float] = []
    last = None
    for k in range(n):
        f = k / (n - 1)
        hit = _luxel_at(ld, (p[0]+d[0]*f, p[1]+d[1]*f, p[2]+d[2]*f))
        if hit is None:
            if run:
                runs.append(run)
            run, last = [], None
            continue
        if hit[:3] != last:
            run.append(_display(hit[3]))
            last = hit[:3]
    if run:
        runs.append(run)
    return runs


def _edge_has_detail(runs: List[List[float]], step: float) -> bool:
    """A jump of more than `step` between neighbouring luxels along the edge."""
    return any(abs(b - a) > step
               for run in runs for a, b in zip(run, run[1:]))


def _compute_coplanar_groups(
    side_map: dict,
    face_vertices: dict,
    face_extents: list,
    detail_type_materials: Optional[Set[str]] = None,
    lighting_data: Optional[dict] = None,
    edge_detail_step: float = EDGE_DETAIL_STEP,
    never_visible: Optional[Set[int]] = None,
) -> List[List[int]]:
    """Connected components of sides a lightmap seam would show between.

    Two sides are linked when they lie on one plane, have the same material
    and texture axis directions, share an edge, and the baseline lightmap
    (lighting_data, side_id -> FaceLightmapData) has detail along a piece of
    that edge on both of them. A side with no lighting data is unknown, not
    flat: it links to a neighbour showing detail on their edge, so a smooth
    side cannot take a coarse evidence floor across what may be a seam.
    Without any lighting data nothing links. Never-visible sides (their
    lighting is flattened, not measured) link to nothing. Returns only groups
    with 2+ sides.

    Excludes tool-textured faces, skip-entity faces, and %detailtype faces.
    Does NOT filter by uniform lighting: the solver handles that.
    """
    from vertex_estimator import should_skip_face

    detail_set = detail_type_materials or set()
    lighting = lighting_data or {}
    hidden = never_visible or set()
    if not lighting:
        return []
    extent_by_id = {fe.side_id: fe for fe in face_extents}

    groups: Dict[tuple, list] = {}
    for side_id in side_map:
        node = side_map.get(side_id)
        if node is None or side_id not in face_vertices or side_id in hidden:
            continue
        material = (node.get_property('material') or '').upper().replace('\\', '/')
        fe = extent_by_id.get(side_id)
        if should_skip_face(material, fe.brush_entity if fe else ''):
            continue
        if material.lower() in detail_set:
            continue
        plane_key = _plane_key(node)
        u_dir = _axis_direction(node.get_property('uaxis'))
        v_dir = _axis_direction(node.get_property('vaxis'))
        if plane_key is None or u_dir is None or v_dir is None:
            continue
        groups.setdefault((plane_key, material, u_dir, v_dir), []).append(side_id)

    def _detail(sid, seg) -> Optional[bool]:
        """Detail along seg on side sid; None when its lighting is unknown."""
        ld = lighting.get(sid)
        # No entry, or one with no samples (a truncated sample range is
        # UNKNOWN, bsp_reader._add_to_result): unknown, not flat. A stand-in
        # entry (carved dark piece, culled side: luminances [0.0], no
        # sub-faces) is known flat, so it cannot bridge two detailed sides.
        if ld is None or not ld.luminances:
            return None
        return _edge_has_detail(_edge_runs(ld, *seg), edge_detail_step)

    def _seam(sid_a, sid_b, segs) -> bool:
        for seg in segs:
            a = _detail(sid_a, seg)
            b = _detail(sid_b, seg)
            if (a or a is None) and (b or b is None) and (a or b):
                return True
        return False

    result = []
    for side_ids in groups.values():
        if len(side_ids) < 2:
            continue
        aabbs = {}
        for sid in side_ids:
            verts = face_vertices.get(sid)
            if verts and len(verts) >= 3:
                aabbs[sid] = (tuple(min(v[k] for v in verts) for k in range(3)),
                              tuple(max(v[k] for v in verts) for k in range(3)))
        adjacency: Dict[int, set] = {sid: set() for sid in aabbs}
        items = list(aabbs.items())
        for i in range(len(items)):
            sid_a, (mins_a, maxs_a) = items[i]
            for j in range(i + 1, len(items)):
                sid_b, (mins_b, maxs_b) = items[j]
                if not _aabbs_touch(mins_a, maxs_a, mins_b, maxs_b):
                    continue
                segs = _shared_edges(face_vertices[sid_a], face_vertices[sid_b])
                if segs and _seam(sid_a, sid_b, segs):
                    adjacency[sid_a].add(sid_b)
                    adjacency[sid_b].add(sid_a)
        for component in _find_connected_components(list(aabbs), adjacency):
            if len(component) >= 2:
                result.append(component)
    return result


def _aabbs_touch(mins_a, maxs_a, mins_b, maxs_b, epsilon: float = 1.0) -> bool:
    """Check if two AABBs touch or overlap (within epsilon tolerance)."""
    for i in range(3):
        if maxs_a[i] + epsilon < mins_b[i] or maxs_b[i] + epsilon < mins_a[i]:
            return False
    return True


def _find_connected_components(side_ids: list, adjacency: dict) -> list:
    """Find connected components via BFS on an adjacency graph.
    
    Args:
        side_ids: list of side IDs
        adjacency: dict mapping side_id → set of adjacent side_ids
    
    Returns:
        List of lists, each being a connected component.
    """
    visited = set()
    components = []
    for sid in side_ids:
        if sid in visited:
            continue
        # BFS from this node
        component = []
        queue = [sid]
        visited.add(sid)
        while queue:
            current = queue.pop(0)
            component.append(current)
            for neighbor in adjacency.get(current, set()):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
        components.append(component)
    return components


def _unify_coplanar_groups(
    scales: Dict[int, int],
    side_map: dict,
    face_data: dict,
    face_extents: list,
    face_vertices: dict,
    detail_type_materials: Optional[Set[str]] = None,
    verbose: bool = False,
    min_scales: Optional[Dict[int, int]] = None,
) -> int:
    """Unify lightmapscale for touching coplanar face groups (broad grouping).
    
    Groups faces by plane only (ignoring material and texture axes), then
    clusters each plane group into connected components based on spatial
    adjacency (AABB overlap). Only faces within the same connected component
    are unified.
    
    Within each component, perfectly uniform faces are excluded: they keep
    their solver-assigned max_useful_scale.
    The remaining "complex" faces are unified to the minimum (best-quality)
    scale already assigned to any member, preventing visual incoherence.
    
    Excludes tool-textured faces, skip-entity faces, and %detailtype faces.

    min_scales (side_id -> floor) are never violated: the unified scale is
    the group's finest, but a member with a higher floor (a displacement's
    VBSP extent minimum, a texlight floor) keeps at least its own — dragging
    a 1088-unit displacement to a flat neighbour's scale 1 is a fatal 'Bad
    surface extents' at the user's next compile.
    
    Modifies `scales` dict in-place. Returns the number of faces unified.
    """
    from vertex_estimator import should_skip_face

    detail_set = detail_type_materials or set()
    floors = min_scales or {}
    
    # Build a lookup for face extent data by side_id
    extent_by_id = {fe.side_id: fe for fe in face_extents}
    
    # Group faces by coplanar key: plane only
    groups: Dict[tuple, list] = {}
    
    for side_id in side_map:
        node = side_map.get(side_id)
        if node is None:
            continue
        
        # Get material and check for exclusions
        material = (node.get_property('material') or '').upper()
        fe = extent_by_id.get(side_id)
        entity_class = fe.brush_entity if fe else ''
        
        # Skip tool textures and non-geometry entities
        if should_skip_face(material, entity_class):
            continue
        
        # Skip %detailtype materials
        mat_lower = material.lower().replace('\\', '/')
        if mat_lower in detail_set:
            continue
        
        # Must have vertex data for adjacency testing
        if side_id not in face_vertices:
            continue
        
        plane_key = _plane_key(node)
        if plane_key is None:
            continue
        
        groups.setdefault(plane_key, []).append(side_id)
    
    # Process groups — cluster into connected components by adjacency
    unified_count = 0
    for plane_key, side_ids in groups.items():
        if len(side_ids) < 2:
            continue
        
        # Build AABBs for each face in this plane group
        aabbs = {}
        for sid in side_ids:
            verts = face_vertices.get(sid)
            if verts and len(verts) >= 3:
                mins = (min(v[0] for v in verts),
                        min(v[1] for v in verts),
                        min(v[2] for v in verts))
                maxs = (max(v[0] for v in verts),
                        max(v[1] for v in verts),
                        max(v[2] for v in verts))
                aabbs[sid] = (mins, maxs)
        
        # Build adjacency graph via AABB overlap
        adjacency: Dict[int, set] = {sid: set() for sid in side_ids if sid in aabbs}
        aabb_items = list(aabbs.items())
        for i in range(len(aabb_items)):
            sid_a, (mins_a, maxs_a) = aabb_items[i]
            for j in range(i + 1, len(aabb_items)):
                sid_b, (mins_b, maxs_b) = aabb_items[j]
                if _aabbs_touch(mins_a, maxs_a, mins_b, maxs_b):
                    adjacency[sid_a].add(sid_b)
                    adjacency[sid_b].add(sid_a)
        
        # Find connected components
        components = _find_connected_components(
            [sid for sid in side_ids if sid in aabbs], adjacency)
        
        # Process each connected component independently
        for component in components:
            if len(component) < 2:
                continue
            
            # Partition into uniform (excluded) and complex (unified)
            complex_ids = []
            excluded_ids = []
            
            for sid in component:
                ld = face_data.get(sid)
                if ld and ld.is_perfectly_uniform:
                    excluded_ids.append(sid)
                else:
                    complex_ids.append(sid)
            
            if len(complex_ids) < 2:
                continue
            
            # Find the minimum (best-quality) scale among the complex faces
            min_scale = None
            for sid in complex_ids:
                s = scales.get(sid)
                if s is not None:
                    if min_scale is None or s < min_scale:
                        min_scale = s
            
            if min_scale is None:
                continue
            
            # Apply the unified scale to all complex faces in the component,
            # but never below a member's own floor
            changed = 0
            for sid in complex_ids:
                old = scales.get(sid)
                target = max(min_scale, floors.get(sid, 1))
                if old is not None and old != target:
                    scales[sid] = target
                    changed += 1
            
            if changed > 0:
                unified_count += len(complex_ids)
                if verbose:
                    print(f"  Unified {len(complex_ids)} touching coplanar faces "
                          f"(plane={plane_key}, scale={min_scale}, "
                          f"{len(excluded_ids)} uniform excluded)",
                          flush=True)
    
    if verbose and unified_count > 0:
        print(f"  Total coplanar-unified: {unified_count} faces", flush=True)
    
    return unified_count


def _get_lightmapscale(side_node) -> int:
    """Get lightmapscale from a VMF side KVNode, defaulting to 16."""
    val = side_node.get_property('lightmapscale')
    if val:
        try:
            return int(val)
        except ValueError:
            pass
    return 16


def _get_lightmapscale_default(side_node) -> int:
    """Get lightmapscale from a side node, returning 0 if node is None."""
    if side_node is None:
        return 0
    return _get_lightmapscale(side_node)


def _set_lightmapscale(side_node, value) -> None:
    """Set a side's lightmapscale, appending the key if the side lacks it.

    KVNode.set_property only updates EXISTING pairs and silently returns
    False otherwise. The one implementation lives in vbsp_solver
    (set_lightmapscale) so the solver's own writes use the same primitive;
    every scale-writing site in this tool goes through it.
    """
    from vbsp_solver import set_lightmapscale
    set_lightmapscale(side_node, value)


#: Scale floor for a displacement whose geometry could not be built. Its
#: extent is unknown, so the minimum VBSP's fatal 'Bad surface extents' check
#: needs (ceil(span/124), 9 for a 1000-unit span) cannot be computed; 16 covers
#: spans up to ~1980 units, far beyond any ordinary displacement.
_DISP_NO_GEOMETRY_FLOOR = 16
_CULLED_SIDE_FLOOR = 16


def _report_geometry_gaps(brushes: list, geometry_faces: list,
                          side_map: dict) -> List[int]:
    """Say which brush sides produced no polygon; return the displacement ones.

    A side whose CSG winding collapses (the 2026-11 bowtie bug was exactly
    this) used to vanish silently: a vanished displacement has no extent, so
    the extent clamp never saw it and VBSP only fatalled at the user's own
    compile. Returns the ids of displacement sides that did not come out as a
    4-vertex base quad; callers floor them at _DISP_NO_GEOMETRY_FLOOR.
    """
    by_side = {gf.side_id: gf for gf in geometry_faces}
    missing = sum(len(b.sides) for b in brushes) - len(by_side)
    if missing > 0:
        print(f"  {missing} brush side(s) produced no geometry (degenerate "
              f"after CSG) and are ignored.", flush=True)
    bad = [sid for sid, node in side_map.items()
           if node.get_children_by_name('dispinfo')
           and (sid not in by_side or len(by_side[sid].vertices) != 4)]
    if bad:
        print(f"  WARNING: {len(bad)} displacement side(s) have no usable "
              f"base quad ({', '.join(map(str, sorted(bad)[:8]))}"
              f"{'...' if len(bad) > 8 else ''}); their extent is unknown, so "
              f"they are held at lightmapscale >= {_DISP_NO_GEOMETRY_FLOOR}.",
              file=sys.stderr, flush=True)
    return bad


def _compute_side_floors(face_extents: list, detail_type_materials: Optional[Set[str]],
                         detail_min_scale: int,
                         texlight_mats: Optional[Set[str]] = None,
                         emissive_floor: int = 1,
                         kinds: Optional[Dict[int, List[str]]] = None) -> Dict[int, int]:
    """side_id -> the lowest lightmapscale that side may end at.

    Combines the minimums the pipeline already knows per face:
      * %detailtype materials: detail_min_scale (optimizable faces only)
      * displacements: VBSP's fatal-extent clamp, for every displacement
        (even entity/tool-skipped ones: VBSP checks them all)
      * texlight faces (optimizable only): emissive_floor, the scale the
        baseline light-budget ladder needed to keep VRAD under its hard limit
    Sides whose floor is 1 are omitted.

    kinds, if given, is filled with side_id -> the names of the rules that put
    each side's floor above 1 ('displacement', 'detailtype', 'texlight'), for
    the scale report; the returned floors are the same either way.
    """
    from vertex_estimator import should_skip_face, min_scale_for_valid_extents
    detail_set = detail_type_materials or set()
    texlights = texlight_mats or set()
    floors: Dict[int, int] = {}
    for fe in face_extents:
        floor = 1
        why: List[str] = []
        if fe.has_displacement:
            disp_floor = min_scale_for_valid_extents(fe.extent_s, fe.extent_t)
            if disp_floor > 1:
                why.append('displacement')
            floor = max(floor, disp_floor)
        if not should_skip_face(fe.material, fe.brush_entity):
            if fe.material.lower().replace('\\', '/') in detail_set:
                if detail_min_scale > 1:
                    why.append('detailtype')
                floor = max(floor, detail_min_scale)
            if (emissive_floor > 1 and
                    fe.material.upper().replace('\\', '/').strip('/') in texlights):
                why.append('texlight')
                floor = max(floor, emissive_floor)
        if floor > 1:
            floors[fe.side_id] = floor
            if kinds is not None:
                kinds[fe.side_id] = why
    return floors


def _apply_scale_floors(scales: Dict[int, int], side_map: dict,
                        floors: Dict[int, int], write_nodes: bool = True,
                        raised_ids: Optional[Set[int]] = None) -> int:
    """Raise (never lower) every side to its floor. Returns how many were raised.

    Sides the solver assigned (present in `scales`) are raised in the dict and,
    if write_nodes, in the VMF tree. Sides the solver never saw (skipped
    entity/tool displacements) are raised in the tree directly. raised_ids, if
    given, collects the ids of the sides raised (the scale report's changed_by).
    """
    raised = 0
    for sid, floor in floors.items():
        node = side_map.get(sid)
        if sid in scales:
            if scales[sid] < floor:
                scales[sid] = floor
                raised += 1
                if raised_ids is not None:
                    raised_ids.add(sid)
                if write_nodes and node is not None:
                    _set_lightmapscale(node, floor)
        elif node is not None and _get_lightmapscale(node) < floor:
            _set_lightmapscale(node, floor)
            raised += 1
            if raised_ids is not None:
                raised_ids.add(sid)
    return raised


def _try_find_vbsp() -> Optional[Path]:
    """Try to find vbsp_lmo.exe in the same directory as this script."""
    from vbsp_runner import find_vbsp
    return find_vbsp()


def _try_find_vrad() -> Optional[Path]:
    """Try to find vrad_rtx.exe via auto-detection."""
    from vrad_runner import find_vrad
    return find_vrad()


def _resolve_vbsp(args) -> Optional[Path]:
    """Resolve the VBSP executable path from args or auto-detection."""
    if args.vbsp:
        path = Path(args.vbsp).resolve()
        if path.is_file():
            return path
        print(f"  WARNING: --vbsp path not found: {path}", file=sys.stderr)
        return None
    # Auto-detect in same directory
    return _try_find_vbsp()


_VVIS_EXE_NAMES = ('vvis.exe', 'vvis_optix.exe')


def _resolve_vvis(args, vbsp_path: Optional[Path] = None,
                  warn: bool = True) -> Optional[Path]:
    """Resolve VVIS path from args, or auto-detect alongside VBSP.
    
    VVIS is needed to generate VIS data before VRAD -countlights.
    Without VIS data, VRAD sets numbounce=0 which causes SubdividePatches
    to skip, resulting in drastically under-counted surface lights.

    Looks for vvis.exe, then vvis_optix.exe (the build this repo ships — a
    layout with only the optix tool used to find no VVIS at all and silently
    produce a no-bounce baseline). warn=False suppresses the not-found
    message for callers that report it themselves.
    """
    if hasattr(args, 'vvis') and args.vvis:
        path = Path(args.vvis).resolve()
        if path.is_file():
            return path
        print(f"  WARNING: --vvis path not found: {path}", file=sys.stderr)
        return None

    def _first_in(directory: Path) -> Optional[Path]:
        for name in _VVIS_EXE_NAMES:
            candidate = directory / name
            if candidate.is_file():
                return candidate
        return None

    # Auto-detect: look for VVIS alongside VBSP
    if vbsp_path:
        found = _first_in(vbsp_path.parent)
        if found:
            return found
    # Try the binroot or game dir's bin/x64
    if hasattr(args, 'binroot') and args.binroot:
        found = _first_in(Path(args.binroot).resolve() / 'bin' / 'x64')
        if found:
            return found
    if hasattr(args, 'game') and args.game:
        found = _first_in(Path(args.game).resolve().parent / 'bin' / 'x64')
        if found:
            return found
    if warn:
        print("  WARNING: Could not find vvis.exe — light counting may "
              "be inaccurate without VIS data.", file=sys.stderr)
    return None


def _resolve_vrad(args) -> Optional[Path]:
    """Resolve the VRAD executable path from args or auto-detection."""
    if hasattr(args, 'vrad') and args.vrad:
        path = Path(args.vrad).resolve()
        if path.is_file():
            return path
        print(f"  WARNING: --vrad path not found: {path}", file=sys.stderr)
        return None
    # Auto-detect
    return _try_find_vrad()


def _resolve_game_dir(args) -> Optional[Path]:
    """Resolve the game directory from args."""
    if args.game:
        path = Path(args.game).resolve()
        if not path.is_dir():
            print(f"  WARNING: --game path not found: {path}", file=sys.stderr)
            return None
            
        if not (path / 'gameinfo.txt').is_file():
            print(f"\n  ╔══════════════════════════════════════════════════╗",
                  file=sys.stderr)
            print(f"  ║  ERROR: Missing gameinfo.txt in --game dir!      ║",
                  file=sys.stderr)
            print(f"  ╠══════════════════════════════════════════════════╣",
                  file=sys.stderr)
            print(f"  ║  {str(path)[:48]:<48}║",
                  file=sys.stderr)
            print(f"  ║                                                  ║",
                  file=sys.stderr)
            print(f"  ║  The --game directory must contain gameinfo.txt. ║",
                  file=sys.stderr)
            print(f"  ║  You might have pointed it to the root folder    ║",
                  file=sys.stderr)
            print(f"  ║  instead of the specific mod folder.             ║",
                  file=sys.stderr)
            print(f"  ║                                                  ║",
                  file=sys.stderr)
            print(f"  ║  Examples of correct --game folders:             ║",
                  file=sys.stderr)
            print(f"  ║   • GarrysMod/garrysmod                          ║",
                  file=sys.stderr)
            print(f"  ║   • Half-Life 2/hl2                              ║",
                  file=sys.stderr)
            print(f"  ║   • Team Fortress 2/tf                           ║",
                  file=sys.stderr)
            print(f"  ╚══════════════════════════════════════════════════╝",
                  file=sys.stderr)
            raise PipelineAbort(f"--game {path} has no gameinfo.txt")
        
        # Validate engine binaries
        engine_root = Path(args.binroot).resolve() if getattr(args, 'binroot', None) else path.parent
        bin_x64 = engine_root / 'bin' / 'x64'
        if not bin_x64.is_dir():
            print(f"\n  ╔══════════════════════════════════════════════════╗",
                  file=sys.stderr)
            print(f"  ║  ERROR: Incompatible engine directory!           ║",
                  file=sys.stderr)
            print(f"  ╠══════════════════════════════════════════════════╣",
                  file=sys.stderr)
            print(f"  ║  {str(engine_root)[:48]:<48}║",
                  file=sys.stderr)
            print(f"  ║                                                  ║",
                  file=sys.stderr)
            print(f"  ║  Missing: bin/x64 folder in directory.          ║",
                  file=sys.stderr)
            print(f"  ║  This engine does not have 64-bit binaries.     ║",
                  file=sys.stderr)
            print(f"  ║                                                  ║",
                  file=sys.stderr)
            print(f"  ║  Compatible examples:                            ║",
                  file=sys.stderr)
            print(f"  ║   • Source SDK Base 2013 Multiplayer             ║",
                  file=sys.stderr)
            print(f"  ║   • Half-Life 2 Deathmatch                      ║",
                  file=sys.stderr)
            print(f"  ║  Incompatible:                                   ║",
                  file=sys.stderr)
            print(f"  ║   • Source SDK Base 2013 Singleplayer            ║",
                  file=sys.stderr)
            print(f"  ║   • Half-Life 2                                 ║",
                  file=sys.stderr)
            print(f"  ╚══════════════════════════════════════════════════╝",
                  file=sys.stderr)
            raise PipelineAbort(f"{engine_root} has no bin/x64 folder")
        return path
    return None


def _parse_texlight_materials(rad_path) -> set:
    """Parse a lights.rad file and return the set of texlight material names.

    rad_path is a Path or a gameinfo_search.FoundFile (a copy inside a VPK);
    anything with read_bytes().

    Returns uppercase-normalized material names (with / separators).
    Skips 'forcetextureshadow' lines and comments.

    Format per line:
        [ldr:|hdr:] material_name R G B intensity
        forcetextureshadow model.mdl
        // comment
    """
    materials = set()
    try:
        text = rad_path.read_bytes().decode('utf-8', errors='replace')
    except OSError:
        return materials
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith('//'):
            continue
        # Skip forcetextureshadow lines
        if line.lower().startswith('forcetextureshadow'):
            continue
        # Strip optional ldr:/hdr: prefix
        if line.lower().startswith(('ldr:', 'hdr:')):
            line = line[4:].strip()
        # First token is the material name
        parts = line.split()
        if len(parts) >= 4:  # material R G B [intensity]
            mat = parts[0].upper().replace('\\', '/').strip('/')
            materials.add(mat)
    return materials


def _find_game_lights_rad(game_dir: Optional[Path],
                          vrad_exe: Optional[Path] = None,
                          bin_root: Optional[Path] = None,
                          strict: bool = True):
    """Find the global lights.rad VRAD loads, as a gameinfo_search.FoundFile.

    VRAD asks its filesystem for 'lights.rad' with no path ID
    (src/utils/vrad/vrad.cpp, FileExists before ReadLightFile): the base
    folder, then every gameinfo SearchPaths entry in order -- base-game
    folders and VPKs included -- then platform (see gameinfo_search). Only if
    that fails does it fall back to the folder the VRAD executable lives in.
    Mirror both, in that order. With strict, raises OSError when a VPK on the
    search path cannot be parsed (whether it holds lights.rad is then
    unknown); without it such a VPK is skipped, as the filesystem skips a pack
    it cannot mount.
    """
    if game_dir:
        found = gameinfo_search.find_file('lights.rad', Path(game_dir), bin_root,
                                          strict=strict)
        if found:
            return found
    if vrad_exe:
        candidate = Path(vrad_exe).parent / 'lights.rad'
        if candidate.is_file():
            return gameinfo_search.FoundFile(str(candidate), loose=candidate)
    return None


def _collect_texlight_materials(game_dir: Optional[Path], lights_rad: Optional[Path],
                                extra_rads: Sequence[Path] = (),
                                vrad_exe: Optional[Path] = None,
                                bin_root: Optional[Path] = None) -> set:
    """Union of texlight material names from every .rad file VRAD reads:
    the global lights.rad, a custom --lights file and any extra_rads (the
    map's own <map>.rad)."""
    mats: set = set()
    # An unparseable VPK is skipped (the filesystem would not mount it) and
    # the search goes on through the remaining paths.
    game_rad = _find_game_lights_rad(game_dir, vrad_exe, bin_root, strict=False)
    if game_rad:
        mats |= _parse_texlight_materials(game_rad)
    for rad in (lights_rad, *extra_rads):
        if rad and Path(rad).is_file():
            mats |= _parse_texlight_materials(Path(rad))
    return mats


def _find_emissive_side_nodes(side_map: dict, texlight_mats: set) -> dict:
    """Map side_id → side node for faces whose material is a texlight."""
    emissive = {}
    for side_id, node in side_map.items():
        material = (node.get_property('material') or '').upper()
        mat_upper = material.replace('\\', '/').strip('/')
        if mat_upper in texlight_mats:
            emissive[side_id] = node
    return emissive


def _raise_emissive_scales(emissive: dict, target: int) -> int:
    """Raise (never lower) each emissive side's lightmapscale to target.

    Returns how many faces actually changed. Raise-only matters: the
    baseline light-budget pass must not make any face's lightmap finer
    than the map (or a previous stage) set it.
    """
    bumped = 0
    for node in emissive.values():
        current = _get_lightmapscale(node)
        if current < target:
            _set_lightmapscale(node, target)
            bumped += 1
    return bumped


#: Rungs probed (in order) while looking for the smallest texlight scale that
#: satisfies a light limit. Roughly geometric so any start reaches 128 in a
#: handful of probes; the answer is then refined by bisection, so the rungs
#: only bound the search and never decide the result.
_SCALE_STEPS = (1, 2, 3, 4, 6, 8, 12, 16, 24, 32, 48, 64, 96, 128)


class _StageFailed(Exception):
    """A probe inside _search_min_scale failed (the cause was already reported)."""


def _search_min_scale(fits, over: int, ceiling: int = 128) -> Optional[int]:
    """Smallest integer scale T in (over, ceiling] for which fits(T) is True.

    `over` is a scale already known NOT to fit; fits(T) must be monotonic (a
    coarser scale never makes the count worse). Probes the _SCALE_STEPS rungs
    above `over` until one fits, then bisects between the last failing and the
    first fitting rung, so the result is the exact minimum rather than the
    next rung up. Returns None if even `ceiling` does not fit. Exceptions from
    fits() propagate.
    """
    low = over
    high = None
    for step in _SCALE_STEPS:
        if step <= low or step > ceiling:
            continue
        if fits(step):
            high = step
            break
        low = step
    if high is None:
        return None
    while high - low > 1:
        mid = (low + high) // 2
        if fits(mid):
            high = mid
        else:
            low = mid
    return high


def _emissive_originals(emissive: dict) -> Dict[int, int]:
    return {sid: _get_lightmapscale(node) for sid, node in emissive.items()}


def _apply_emissive_floor(emissive: dict, originals: Dict[int, int],
                          target: int) -> int:
    """Put every emissive face at max(its original scale, target).

    Restores the originals first, so a lower probe after a higher one really
    lowers the faces again (a plain raise-only bump would leave them at the
    higher scale). Returns how many faces ended above their original.
    """
    for sid, node in emissive.items():
        _set_lightmapscale(node, originals[sid])
    return _raise_emissive_scales(emissive, target)


def _fit_baseline_light_budget(root, work_vmf: Path, vbsp_exe: Path,
                               vvis_exe: Optional[Path], vrad_exe: Optional[Path],
                               game_dir: Path, bin_root: Optional[Path],
                               lights_rad: Optional[Path], eligible_sids: list,
                               extra_rads: Sequence[Path] = (),
                               max_lights: Optional[int] = None,
                               verbose: bool = False) -> int:
    """Keep VRAD's direct-light count under ITS hard limit during the baseline.

    At fine lightmap scales every texlight patch is its own direct light, so
    a scale-2 baseline can blow past VRAD's hard 32,767-light limit and kill
    the lighting compile outright ('Too many lights' — VRAD exits without
    writing any lighting data). Raising only the emissive faces keeps the
    fine baseline sampling for everything else, which is what the solver
    classifies from.

    The target is the compiler's limit (max_lights, default
    vrad_runner.VRAD_HARD_LIGHT_LIMIT) and nothing else: the baseline only
    has to SURVIVE VRAD. The user's --light-budget is a final-map target
    owned by the post-solver enforcement, which may be tighter or looser
    than the hard limit, and must not make the baseline abort or climb.

    The scale is the SMALLEST that fits (rung probe, then bisection — see
    _search_min_scale): it becomes a floor for every eligible texlight face
    in the final map, so overshooting it would coarsen the whole map's
    emissive surfaces for nothing.

    Only ELIGIBLE sides are raised: the solver later overwrites every
    eligible side's scale. So the scale this pass finds is RETURNED and the
    caller carries it into the solver as a floor for those faces; without
    that the solver re-promotes the texlights to scale 1 and enforcement has
    to rediscover the fix one compile at a time. Ineligible texlight faces
    are left to the enforcement stage.

    Light counting is best-effort: a VRAD without -countlights support, a
    timeout, or a counting failure skips this pass with a warning, and the
    post-VRAD lighting-data gate remains the backstop — if the compile then
    dies ('Too many lights') or writes no lightmap samples, the run aborts
    with VRAD's real error instead of a counting-infrastructure failure.

    The BSP next to work_vmf must already be compiled (VBSP + VVIS).
    Mutates root's emissive sides in place and rewrites/recompiles work_vmf
    between steps; on return the tree and the BSP on disk agree. Raises
    PipelineAbort if the hard limit cannot be met even at scale 128 — the
    map cannot be lit at all and continuing would poison the classification.
    Returns the emissive floor found (1 = none needed).
    """
    from vbsp_runner import compile_bsp
    from vrad_runner import count_lights, VRADError, VRAD_HARD_LIGHT_LIMIT
    from vmf_parser import VMFWriter

    if max_lights is None:
        max_lights = VRAD_HARD_LIGHT_LIMIT

    texlight_mats = _collect_texlight_materials(
        game_dir, lights_rad, extra_rads=extra_rads, vrad_exe=vrad_exe,
        bin_root=bin_root)
    if not texlight_mats:
        return 1  # No texlights configured: light count is entity-driven
    eligible_ids = {sid for sid, _min_s in eligible_sids}
    emissive = {sid: node for sid, node in
                _find_emissive_side_nodes(_build_vmf_side_map(root),
                                          texlight_mats).items()
                if sid in eligible_ids}
    if not emissive:
        return 1

    if vrad_exe is None:
        print("  ⚠ No VRAD executable — skipping the baseline light-budget "
              "pass.", file=sys.stderr, flush=True)
        return 1

    work_vmf = Path(work_vmf)
    bsp_path = work_vmf.with_suffix('.bsp')
    writer = VMFWriter()

    def _count() -> Optional[int]:
        """Direct-light count, or None when counting itself failed."""
        try:
            return count_lights(vrad_exe, bsp_path, game_dir,
                                bin_root=bin_root, timeout=600,
                                verbose=verbose, lights_rad=lights_rad).count
        except (VRADError, TimeoutError, OSError) as e:
            print(f"  ⚠ Light counting unavailable ({e}) — skipping the "
                  f"baseline light-budget pass; VRAD's own compile remains "
                  f"the backstop.", flush=True)
            return None

    count = _count()
    if count is None or count <= max_lights:
        if count is not None:
            print(f"  ✓ Baseline light count {count:,} ≤ VRAD limit {max_lights:,}",
                  flush=True)
        return 1

    print(f"  ⚠ Baseline light count {count:,} > VRAD limit {max_lights:,} — "
          f"raising {len(emissive)} texlight faces so the lighting compile "
          f"survives", flush=True)

    originals = _emissive_originals(emissive)
    compiled = {'target': None}   # the scale the BSP on disk was compiled at

    def _compile_at(target: int) -> None:
        # -emitsideids matches the main baseline compile: this BSP becomes
        # the cached classification BSP, and without the side-ID lumps
        # face matching silently degrades to truncated IDs + plane
        # heuristics.
        _apply_emissive_floor(emissive, originals, target)
        writer.write_file(root, work_vmf)
        compile_bsp(vbsp_exe, work_vmf, game_dir, bin_root=bin_root,
                    verbose=False, timeout=300, extra_args=['-emitsideids'])
        compiled['target'] = target
        if vvis_exe:
            if not _run_vvis_fast(vvis_exe, bsp_path, game_dir,
                                  bin_root=bin_root, verbose=False):
                raise PipelineAbort(
                    "VVIS -fast failed on a light-ladder recompile (a "
                    "baseline without VIS data is not a valid classification "
                    "input)")

    def _fits(target: int) -> bool:
        _compile_at(target)
        probe = _count()
        if probe is None:
            raise _StageFailed()   # counting broke mid-search
        print(f"    texlight scale → {target}: {probe:,} lights", flush=True)
        return probe <= max_lights

    try:
        found = _search_min_scale(_fits, over=min(originals.values()))
    except _StageFailed:
        # Counting broke mid-search: leave the tree where it is and let the
        # VRAD gate decide
        return compiled['target'] or 1

    if found is None:
        raise PipelineAbort(
            "surface light count is still over VRAD's limit with all "
            "eligible texlight faces at scale 128 — the remaining lights come "
            "from entities or from texlight faces the optimizer skips "
            "(tools/entity rules); reduce light entities")

    if compiled['target'] != found:
        _compile_at(found)   # the last probe was a failing one: the BSP VRAD
                             # lights next must be the fitting scale's
    return found


def _run_vvis_fast(vvis_exe: Path, bsp_path: Path, game_dir: Path,
                   bin_root: Path = None,
                   verbose: bool = False, timeout: int = 600) -> bool:
    """Run VVIS -fast on a BSP to generate minimal VIS data.
    
    VRAD requires VIS data to perform SubdividePatches(); without it,
    numbounce is set to 0 and subdivision is skipped, causing surface
    lights from texlights to be drastically under-counted.
    
    Returns True on success, False on failure.
    """
    # Full .bsp path: the tools cut an extensionless dotted name (our scratch
    # '<stem>.lmopt_...') at its last dot and would open '<stem>.bsp'.
    bsp_arg = str(bsp_path)
    import subprocess
    import os
    cmd = [
        str(vvis_exe),
        '-fast',
        '-game', str(game_dir),
    ]
    if bin_root:
        cmd.extend(['-binroot', str(Path(bin_root).resolve())])
    cmd.append(bsp_arg)
    
    env = os.environ.copy()
    if bin_root:
        bin_path_x64 = str((Path(bin_root) / 'bin' / 'x64').resolve())
        bin_path_32 = str((Path(bin_root) / 'bin').resolve())
        env['PATH'] = f"{bin_path_x64}{os.pathsep}{bin_path_32}{os.pathsep}{env.get('PATH', '')}"

    if verbose:
        print(f"  VVIS: {' '.join(cmd)}", flush=True)
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout,
            cwd=str(vvis_exe.parent),
            env=env,
        )
        if verbose:
            for line in result.stdout.splitlines():
                print(f"    [vvis] {line}", flush=True)
        if result.returncode != 0:
            tail = (result.stdout + result.stderr).strip().splitlines()[-5:]
            print(f"  VVIS exited with code {result.returncode}:\n    "
                  + "\n    ".join(tail), file=sys.stderr, flush=True)
        return result.returncode == 0
    except (subprocess.TimeoutExpired, FileNotFoundError, OSError) as e:
        print(f"  VVIS error: {e}", file=sys.stderr, flush=True)
        return False


def _run_light_budget_enforcement(
    root,
    side_map: dict,
    anchor_vmf: Path,
    vbsp_exe: Path,
    vrad_exe: Path,
    vvis_exe: Path | None = None,
    game_dir: Path = None,
    bin_root: Path | None = None,
    lights_rad: Path | None = None,
    level_rad: Path | None = None,
    args=None,
) -> bool:
    """Compile the optimized VMF, count surface lights, and degrade if over budget.

    This is a post-solver step that raises the lightmapscale of texlight
    (emissive) faces until the surface light count is within the budget, to
    the smallest scale that fits (rung probe, then bisection — see
    _search_min_scale). It mutates `root` in memory only; the caller
    publishes the result.

    Compiles a scratch copy that lives next to the INPUT VMF (so relative
    func_instance paths resolve exactly as in the user's own compile) under a
    unique name — never at the output path, so the user's previous output and
    their own compile products are not touched.

    Returns True when the map is verified within budget, False on any failure
    (compile/count failure, no way to reduce, still over budget). The
    failure has been printed.
    """
    from vbsp_runner import compile_bsp, VBSPError, ScratchVMF
    from vrad_runner import count_lights, VRADError

    light_budget = getattr(args, 'light_budget', 32767) if args else 32767
    verbose = getattr(args, 'verbose', False) if args else False

    print()
    print("┌──────────────────────────────────────────────────┐")
    print("│          Surface Light Budget Check              │")
    print("├──────────────────────────────────────────────────┤")

    def _fail(box_lines: List[str]) -> bool:
        import textwrap
        message = " ".join(l.strip() for l in box_lines)
        for line in textwrap.wrap(message, 46):
            print(f"│  {line:<46}│")
        print("└──────────────────────────────────────────────────┘")
        _log_error("  ERROR: " + message)
        return False

    if not vvis_exe:
        print(f"│  ⚠ No VVIS — surface light counts may be low     │")

    with ScratchVMF(anchor_vmf, 'budget') as work:
        work.mirror_level_rad(level_rad)

        def _compile_and_count(what: str) -> Optional[int]:
            """VBSP + VVIS + VRAD -countlights on the current tree; None on failure."""
            work.write(root)
            work.clear_artifacts()
            try:
                compile_bsp(vbsp_exe, work.path, game_dir, bin_root=bin_root,
                            verbose=verbose, timeout=300)
            except (VBSPError, TimeoutError) as e:
                _fail([f"VBSP compilation failed ({what}):"]
                      + str(e).splitlines()[:2])
                return None
            if vvis_exe and not _run_vvis_fast(
                    vvis_exe, work.bsp, game_dir, bin_root=bin_root,
                    verbose=verbose):
                _fail([f"VVIS -fast failed ({what}) — light counts would "
                       f"be unreliable"])
                return None
            try:
                return count_lights(vrad_exe, work.bsp, game_dir,
                                    bin_root=bin_root, verbose=verbose,
                                    timeout=600, lights_rad=lights_rad).count
            except (VRADError, TimeoutError, OSError) as e:
                _fail([f"VRAD light counting failed ({what}):"]
                      + str(e).splitlines()[:2])
                return None

        print(f"│  Compiling with VBSP, VVIS, counting with VRAD...│", flush=True)
        count = _compile_and_count("initial")
        if count is None:
            return False

        print(f"│  Surface lights: {count:>8,}                      │", flush=True)
        print(f"│  Light budget:   {light_budget:>8,}                      │", flush=True)

        if count <= light_budget:
            margin = light_budget - count
            pct = count / light_budget * 100
            print(f"│  ✓ UNDER BUDGET by {margin:>6,} ({pct:.0f}%)             │")
            print("└──────────────────────────────────────────────────┘")
            return True

        # Targeted degradation — only bump texlight-emitting faces
        print("├──────────────────────────────────────────────────┤")
        print(f"│  ⚠ OVER BUDGET — degrading emissive surfaces... │")
        print("├──────────────────────────────────────────────────┤", flush=True)

        # Collect texlight material names from every .rad file VRAD reads
        texlight_mats = _collect_texlight_materials(
            game_dir, lights_rad,
            extra_rads=[level_rad] if level_rad else [], vrad_exe=vrad_exe,
            bin_root=bin_root)
        if not texlight_mats:
            return _fail([f"{count:,} lights > budget {light_budget:,} and no "
                          f"texlight materials were found in lights.rad —",
                          "pass --lights if your texlights live elsewhere"])

        print(f"│  Texlight materials found: {len(texlight_mats):>4}                 │")

        # Identify emissive faces in the VMF
        emissive_sides = _find_emissive_side_nodes(side_map, texlight_mats)
        print(f"│  Emissive faces in VMF:    {len(emissive_sides):>4}                 │")
        if not emissive_sides:
            return _fail([f"{count:,} lights > budget {light_budget:,} but no "
                          f"emissive faces exist — the lights come from "
                          f"entities and cannot be reduced here"])

        originals = _emissive_originals(emissive_sides)
        probes = {'count': count}

        def _fits(target: int) -> bool:
            _apply_emissive_floor(emissive_sides, originals, target)
            probed = _compile_and_count(f"scale {target}")
            if probed is None:
                raise _StageFailed()   # already reported
            probes['count'] = probed
            status = '✓' if probed <= light_budget else '…'
            print(f"│  [{status}] texlight scale {target:>3}: {probed:>7,} lights"
                  f"{'':<8}│", flush=True)
            return probed <= light_budget

        try:
            found = _search_min_scale(_fits, over=min(originals.values()))
        except _StageFailed:
            return False

        if found is None:
            over = probes['count'] - light_budget
            return _fail([f"STILL OVER BUDGET by {over:,} with every emissive "
                          f"face at its coarsest —",
                          "reduce light entities or surface-light materials, "
                          "or raise --light-budget"])

        # The last probe may have been a failing one: leave the tree at the
        # smallest scale that fit (its count was measured above)
        _apply_emissive_floor(emissive_sides, originals, found)
        print("├──────────────────────────────────────────────────┤")
        print(f"│  ✓ UNDER BUDGET at texlight scale {found:>3}              │")
        print("└──────────────────────────────────────────────────┘")
        return True


# ═══════════════════════════════════════════════════════════════════════════════
#  AUTO-COMPILE MODE
# ═══════════════════════════════════════════════════════════════════════════════

def _compute_eligible_min_scales(root, args, game_dir: Path):
    """Compute the per-side minimum lightmapscale for every optimizable face.

    Combines %detailtype minimums (detail_min_scale) with the VBSP fatal
    extent clamp for displacement faces — brush faces are subdivided by
    VBSP automatically, so only displacements need the clamp.

    Returns (eligible, skipped, side_map, disp_floor): eligible is
    [(side_id, min_scale)]; disp_floor is the same shape for faces skipped
    by the entity/material rules that are still renderable displacements —
    they never join the optimization, but VBSP's fatal extent check applies
    to them too, so their scales must be raised (never lowered) to the
    clamp minimum.
    """
    from vertex_estimator import (should_skip_face, compute_face_extent,
                                  min_scale_for_valid_extents)
    from geometry import build_all_faces
    from vmt_checker import scan_materials

    # Build geometry + face extents
    brush_entity_map = _build_brush_entity_map(root)
    brushes = extract_brushes(root)
    all_geometry_faces = build_all_faces(brushes)
    side_map = _build_vmf_side_map(root)
    vanished_disps = _report_geometry_gaps(brushes, all_geometry_faces, side_map)

    face_extents: list = []
    for gf in all_geometry_faces:
        side_node = side_map.get(gf.side_id)
        if side_node is None:
            continue
        uaxis_str = side_node.get_property('uaxis') or ''
        vaxis_str = side_node.get_property('vaxis') or ''
        entity_class = brush_entity_map.get(gf.brush_id, '')
        fe = compute_face_extent(
            gf.vertices, uaxis_str, vaxis_str,
            gf.lightmapscale, gf.side_id, gf.material,
            brush_entity=entity_class,
            has_displacement=bool(side_node.get_children_by_name('dispinfo')),
        )
        face_extents.append(fe)

    # Classify faces — skip tool textures, func_monitor, etc.
    unique_materials = {fe.material.lower().replace('\\', '/') for fe in face_extents}
    detail_set = scan_materials(unique_materials, game_dir, verbose=False)

    detail_min_scale = args.detail_min_scale
    eligible: list = []
    disp_floor: list = []
    for fe in face_extents:
        extent_min = 0
        if fe.has_displacement:
            extent_min = min_scale_for_valid_extents(fe.extent_s, fe.extent_t)
        if should_skip_face(fe.material, fe.brush_entity):
            if extent_min > 0:
                disp_floor.append((fe.side_id, extent_min))
            continue
        mat_lower = fe.material.lower().replace('\\', '/')
        min_s = detail_min_scale if mat_lower in detail_set else 1
        min_s = max(min_s, extent_min)
        eligible.append((fe.side_id, min_s))

    # Displacements with no usable geometry are in no face extent at all
    disp_floor.extend((sid, _DISP_NO_GEOMETRY_FLOOR) for sid in vanished_disps)

    return eligible, len(face_extents) - len(eligible), side_map, disp_floor


def _apply_min_floor_scales(side_map: dict, floors: list) -> None:
    """Raise (never lower) each listed side's lightmapscale to its floor.

    Used for skipped-entity displacements: they stay out of the optimizer,
    but VBSP's fatal extent check still applies to them at compile time.
    """
    for sid, min_s in floors:
        node = side_map.get(sid)
        if not node:
            continue
        current = _get_lightmapscale(node)
        if current < min_s:
            _set_lightmapscale(node, min_s)


def _apply_baseline_scales(side_map: dict, eligible: list, baseline: int) -> None:
    """Apply max(baseline, per-side min) lightmapscale to all eligible sides."""
    for sid, min_s in eligible:
        node = side_map.get(sid)
        if node:
            _set_lightmapscale(node, max(baseline, min_s))


def _find_safe_baseline(
    root, args, vbsp_path: Path, game_dir: Path,
    input_path: Path, vertex_budget: int,
    eligible_sids: list, side_map: dict,
    verbose: bool = False,
) -> int:
    """Find the minimum global lightmapscale that fits within the vertex budget.

    Uses VBSP -countverts in a linear scan (scale=1, 2, 3...) to find the
    lowest scale that doesn't crash VBSP or exceed the budget. Respects the
    caller-computed per-side minimums (entity-skip rules, %detailtype and
    displacement extent clamps). Applies the winning scale to the root VMF
    tree in memory. Trial VMFs are siblings of the input map (instances
    resolve), never at the output path.

    Only a VBSPRejectedError — VBSP's own capacity errors ('Too many edges',
    'Edge with too many vertices due to t-junctions', ...), see vbsp_runner —
    counts as 'too fine, try the next scale'. Any other failure (missing exe,
    crash, timeout, unparsable output, a VMF VBSP cannot read) propagates
    immediately instead of being retried 64 times and reported as 'too many
    brushes'. If VBSP rejected every scale, the last rejection propagates too.

    Returns:
        The baseline scale (1–64), or 0 if nothing fits.
    """
    from vbsp_runner import count_vertices, ScratchVMF, VBSPRejectedError

    last_error: Optional[Exception] = None
    counted_any = False
    with ScratchVMF(input_path, 'baseline_scan') as trial:
        for scale in range(1, 65):
            # Apply this scale to all eligible faces
            _apply_baseline_scales(side_map, eligible_sids, scale)
            trial.write(root)

            if verbose:
                print(f"  [baseline] Testing global scale={scale}...",
                      flush=True)

            try:
                result = count_vertices(vbsp_path, trial.path, game_dir,
                                        bin_root=getattr(args, 'binroot', None),
                                        verbose=False)
            except VBSPRejectedError as e:
                last_error = e
                if verbose:
                    print(f"    VBSP rejected scale={scale}: {e}", flush=True)
                continue
            counted_any = True
            fits = (result.count <= vertex_budget and
                    result.leafface_count <= vertex_budget and
                    result.face_count <= vertex_budget)

            if verbose:
                print(f"    Verts: {result.count:,}  "
                      f"Leaffaces: {result.leafface_count:,}  "
                      f"Faces: {result.face_count:,}  "
                      f"({'under' if fits else 'OVER'} budget)",
                      flush=True)

            if fits:
                return scale

    if not counted_any and last_error is not None:
        raise last_error
    return 0  # Nothing fits


# ─── Baseline cache ───────────────────────────────────────────────────────────
#
# One entry per input map, in <input dir>/.lmopt_cache/:
#   <stem>.<12 hex>.bsp   the lit baseline BSP (unique name per write)
#   <stem>.manifest.json  {version, key, baseline, emissive_floor, bsp_file, bsp_size}
# The manifest is written LAST (BSP via temp + os.replace first) and deleted
# FIRST on an update, so a crash anywhere leaves a cache miss -- never a key
# paired with the wrong BSP or baseline. Because every entry's BSP has its own
# name, concurrent runs on one map cannot swap each other's BSP either.

_CACHE_VERSION = 4
_LEGACY_CACHE_EXTS = ('.vmf_hash', '.baseline_scale')


def _md5_file(path) -> str:
    h = hashlib.md5()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def _tool_impl_files(exe: Path, extras: Sequence[str] = ()) -> List[Path]:
    """The files whose bytes decide what a compiler tool does.

    vrad_rtx.exe and vvis_optix.exe are ~180 KB launcher stubs that load the
    real implementation (vrad_rtx_dll.dll / vvis_optix_dll.dll) and, for VRAD,
    its GPU kernels (.ptx); hashing the stub alone means a rebuilt compiler never
    invalidates the cache. Collects the exe, its `<stem>_dll.dll`, a
    `<stem>.ptx` if one exists, a VRAD-style `<stem>.redirect` file plus the DLL
    it names, and any `extras` (file names next to the exe; VRAD's
    optix_kernels.ptx comes in this way).
    """
    exe = Path(exe)
    files = [exe]
    candidates = [exe.with_name(exe.stem + '_dll.dll'),
                  exe.with_name(exe.stem + '.ptx')]
    redirect = exe.with_name(exe.stem + '.redirect')
    if redirect.is_file():
        candidates.append(redirect)
        try:
            first = redirect.read_text(encoding='utf-8', errors='replace').splitlines()
            if first and first[0].strip():
                candidates.append(exe.parent / first[0].strip())
        except OSError:
            pass
    candidates.extend(exe.parent / name for name in extras)
    files.extend(c for c in candidates if c.is_file())
    return files


def _hash_files(files: Sequence[Path]) -> str:
    h = hashlib.md5()
    for f in files:
        h.update(f"{Path(f).name}={_md5_file(f)}\n".encode())
    return h.hexdigest()


def _rad_input_files(game_dir: Optional[Path], vrad_exe: Optional[Path],
                     lights_rad: Optional[Path], level_rad: Optional[Path],
                     bin_root: Optional[Path] = None) -> List[Tuple[str, object]]:
    """(label, readable source or None) for every .rad file VRAD really reads.

    The global lights.rad (VRAD's search path -- base-game folders and VPKs
    included -- else the VRAD exe's folder; see _find_game_lights_rad), the
    --lights file and the map's own <map>.rad. Each source has read_bytes().
    Absent ones are still listed so that ADDING a file changes the key.
    """
    game_rad = _find_game_lights_rad(game_dir, vrad_exe, bin_root)
    return [('lights.rad', game_rad),
            ('--lights', Path(lights_rad) if lights_rad and Path(lights_rad).is_file() else None),
            ('level.rad', Path(level_rad) if level_rad and Path(level_rad).is_file() else None)]


def _build_cache_key(*, input_vmf: Path, vbsp: Path, vrad: Path, vvis: Path,
                     game_dir: Path, budget: int, detail_min_scale: int,
                     rtx: bool, texture_shadows: str, bin_root: Optional[Path],
                     lights_rad: Optional[Path], level_rad: Optional[Path],
                     eligible_hash: str,
                     instance_files: Sequence[Path] = ()) -> str:
    """Fingerprint of the inputs the cached lit baseline BSP depends on.

    COVERS: the input VMF bytes; every func_instance VMF it pulls in, nested
    ones included (instance_files, see _instance_closure); the settings
    feeding the baseline scan and per-side minimums; the eligibility output;
    the IMPLEMENTATION of VBSP, VRAD and VVIS (exe plus the DLL/kernels behind
    a launcher stub, see _tool_impl_files); the VRAD fast-compile arguments
    and the texture-shadow level (it changes which brushes cast shadows in the
    analysis lighting); and every .rad file VRAD reads (global lights.rad
    wherever VRAD's search path finds it, a VPK included; --lights; <map>.rad).

    DOES NOT COVER: models, materials/VMTs/textures, the physics/prop data,
    the rest of the game folder or the Steam install -- anything VBSP, VVIS and
    VRAD read beyond the files above. Change those and the baseline can be stale:
    use --no-cache. The user's --light-budget is deliberately not part of the
    key either: the baseline only has to survive VRAD's hard limit, so changing
    the budget reuses the BSP and only re-runs the post-solver enforcement.

    Raises OSError if a required file cannot be read (the caller then runs
    without the cache: an unhashable input cannot be proven unchanged).
    """
    from vrad_runner import FAST_VRAD_ARGS, TEXTURE_SHADOW_LEVELS
    rad_sig = []
    for label, src in _rad_input_files(game_dir, vrad, lights_rad, level_rad, bin_root):
        digest = hashlib.md5(src.read_bytes()).hexdigest() if src else 'absent'
        rad_sig.append(f"{label}={digest}")
    settings = "|".join([
        str(budget), str(detail_min_scale), str(game_dir), str(bin_root or ''),
        str(bool(rtx)), repr(list(FAST_VRAD_ARGS)),
        repr(list(TEXTURE_SHADOW_LEVELS[texture_shadows])),
    ])
    return "|".join([
        _md5_file(input_vmf),
        hashlib.md5(settings.encode()).hexdigest(),
        eligible_hash,
        _hash_files(_tool_impl_files(vbsp)),
        _hash_files(_tool_impl_files(vrad, extras=('optix_kernels.ptx',))),
        _hash_files(_tool_impl_files(vvis)),
        hashlib.md5(";".join(rad_sig).encode()).hexdigest(),
        _hash_files(sorted(Path(f) for f in instance_files)),
    ])


#: VBSP's key/value buffer (MAX_KEYVALUE_LEN, src/public/chunkfile.h).
_MAX_KEYVALUE_LEN = 1024

_Epairs = Tuple[Tuple[str, str], ...]


def _entity_epairs(entity) -> _Epairs:
    """An entity's keys as VBSP's epair list holds them: SetKeyValue
    (src/utils/common/bsplib.cpp) PREPENDS a new key and overwrites a repeated
    one in place, so the list runs in reverse first-appearance order with each
    key's last value."""
    from vmf_parser import KVPair
    order: List[str] = []
    values = {}
    for child in entity.children:
        if isinstance(child, KVPair):
            k = child.key.lower()
            if k not in values:
                order.append(k)
            values[k] = (child.key, child.value)
    return tuple(values[k] for k in reversed(order))


def _replace_instance_value(value: str, parent: _Epairs) -> str:
    """CMapFile::ReplaceInstancePair (src/utils/vbsp/map.cpp ~2292): each
    'replace*' key of the parent func_instance, in epair order, is '<var>
    <text>'; every ASCII-case-insensitive <var> in the value becomes <text>.
    A key with no space, or an empty <var> (V_stristr never matches one), is
    skipped; a result that would overflow VBSP's buffer leaves the value as it
    was."""
    new = value
    for key, pair in parent:
        if not key.lower().startswith('replace'):
            continue
        var, sep, text = pair.partition(' ')
        if not sep or not var:
            continue
        new = re.sub(re.escape(var), lambda _m: text, new,
                     flags=re.IGNORECASE | re.ASCII)
        if len(new.encode('utf-8', 'replace')) >= _MAX_KEYVALUE_LEN:
            return value
    return new


def _instance_entity_epairs(root) -> List[_Epairs]:
    """The epairs that can make a func_instance, per entity of a parsed VMF:
    only 'classname', 'file' and 'replace*' keys (in VBSP epair order), and
    only for entities that have a 'file' key. Compact, so the closure can keep
    one list per instance file instead of whole parse trees."""
    out = []
    for entity in root.get_all_recursive('entity'):
        pairs = tuple((k, v) for k, v in _entity_epairs(entity)
                      if k.lower() in ('classname', 'file')
                      or k.lower().startswith('replace'))
        if any(k.lower() == 'file' for k, _ in pairs):
            out.append(pairs)
    return out


def _instance_refs(entities: List[_Epairs],
                   parent: _Epairs = ()) -> List[Tuple[str, _Epairs]]:
    """(file, the func_instance's substituted replace keys) for every
    func_instance among `entities`, each key first given the parent
    func_instance's replace values as MergeEntities does when it copies an
    instance's entities into the main map (an empty parent = the main map
    itself, which is never substituted). Only replace keys reach the next
    level, so placements that differ in origin or id are one expansion."""
    refs = []
    for epairs in entities:
        pairs = tuple((k, _replace_instance_value(v, parent)) for k, v in epairs)
        keys = {k.lower(): v for k, v in pairs}
        if keys.get('classname', '').lower() == 'func_instance':
            name = keys.get('file', '').strip()
            if name:
                refs.append((name, tuple(p for p in pairs
                                         if p[0].lower().startswith('replace'))))
    return refs


def _fix_slashes(text: str) -> str:
    """V_FixSlashes: one separator style (here '/', which Windows accepts too)."""
    return text.replace('\\', '/')


def _set_vmf_extension(name: str) -> str:
    """V_SetExtension(name, ".vmf"): drop the last '.ext' of the FINAL path
    component, if any, then append '.vmf'. Plain string work: pathlib's
    with_suffix raises ValueError for names like '.' or 'C:'."""
    slash = name.rfind('/')
    dot = name.rfind('.')
    if dot > slash:
        name = name[:dot]
    return name + '.vmf'


def _resolve_instance(base_vmf: Path, name: str,
                      game_dir: Optional[Path]) -> Optional[Path]:
    """Find an instance VMF exactly the way VBSP does, or None.

    Mirrors CMapFile::DeterminePath (src/utils/vbsp/map.cpp ~1914-1970) with
    the same plain string joins, so a name with a leading separator stays
    UNDER the map folder (as VBSP joins it) instead of becoming a drive-root
    path, and any name VBSP could not use is simply not found:
      1. <folder of the VMF being compiled> + '\\' + name, '.vmf' forced
      2. the same name under the FIRST '\\maps\\' ancestor of that folder
         (the folder is dot-collapsed first; matched case-insensitively)
      3. gameinfo.txt's InstancePath (lower-cased) + the name AS WRITTEN (no
         '.vmf' forcing there) under the game folder (the real lookup is the
         whole GAME search path; VPK-resident instances are not found here)
    Separators of either kind are accepted (V_FixSlashes), on every platform.
    Total: any failure (an embedded NUL, a bad drive spec...) means None, and
    the caller then does not cache.
    """
    try:
        fixed = _set_vmf_extension(_fix_slashes(name))
        base = _fix_slashes(str(base_vmf))
        folder = base[:base.rfind('/')] if '/' in base else ''

        first = folder + '/' + fixed
        if os.path.isfile(first):
            return Path(first)

        collapsed = _fix_slashes(os.path.normpath(folder)) + '/'
        m = re.search('/maps/', collapsed, re.IGNORECASE)
        if m:
            second = collapsed[:m.end()] + fixed
            if os.path.isfile(second):
                return Path(second)

        if game_dir:
            try:
                text = (Path(game_dir) / 'gameinfo.txt').read_text(
                    encoding='utf-8', errors='replace')
            except OSError:
                text = ''
            m = re.search(r'"instancepath"\s+"([^"]*)"', text, re.IGNORECASE)
            if m and m.group(1):
                relative = _fix_slashes(m.group(1).lower()) + _fix_slashes(name)
                # a GAME-relative path: an absolute-looking one is not
                if not relative.startswith('/') and ':' not in relative:
                    third = _fix_slashes(str(game_dir)) + '/' + relative
                    if os.path.isfile(third):
                        return Path(third)
    except (ValueError, OSError):
        pass
    return None


def _instance_closure(input_vmf: Path, root, game_dir: Optional[Path],
                      max_expansions: int = 4096
                      ) -> Tuple[List[Path], List[str]]:
    """(resolved instance files, unresolvable instance names) for a map.

    Nested instances resolve against the MAIN map file, not against the file
    that references them. Evidence, src/utils/vbsp/map.cpp:
      * LoadMapFile() ends with `g_LoadingMap->CheckForInstances(pszFileName)`
        (~2642) for the main map AND for every instance it loads, but
        CheckForInstances returns at once when `this != g_MainMap` (~1982-1990:
        "all sub-instances will be appended to the main map master list as they
        are read in so the main loop below will naturally get to the appended
        ones"), so an instance never resolves anything itself;
      * the main map's loop over its growing entity list calls
        `DeterminePath(pszFileName, pInstanceFile, ...)` (~2040) with the MAIN
        map's file name for every func_instance, including the ones
        MergeEntities() copied in from an instance verbatim.
    So every level is resolved from input_vmf. A nested instance's keys (its
    'file' and its own replace keys) first get its parent func_instance's
    replace values (_instance_refs), so the same file can be reached with
    different parameters and is expanded once per distinct set of replace
    keys. Cycles are cut by a visited set; a parameter chain that keeps
    growing (VBSP would recurse forever) stops at max_expansions distinct
    (name, replace keys) states and is unresolved.
    Total: any failure to read or parse an instance marks it unresolved.
    """
    files: List[Path] = []
    unresolved: List[str] = []
    seen = set()
    seen_files = set()
    expanded = set()
    parsed = {}
    queue = list(_instance_refs(_instance_entity_epairs(root)))
    while queue:
        name, pairs = queue.pop()
        key = (name.lower().replace('\\', '/'), pairs)
        if key in seen:
            continue
        if len(seen) >= max_expansions:
            unresolved.append(name)
            break
        seen.add(key)
        path = _resolve_instance(input_vmf, name, game_dir)
        if path is None:
            unresolved.append(name)
            continue
        resolved_key = os.path.normcase(os.path.abspath(path))
        if resolved_key not in seen_files:   # 'inst/a' and 'inst/a.vmf' are one file
            seen_files.add(resolved_key)
            files.append(path)
        expansion = (resolved_key, pairs)
        if expansion in expanded:
            continue
        expanded.add(expansion)
        if resolved_key not in parsed:
            try:
                parsed[resolved_key] = _instance_entity_epairs(
                    VMFParser().parse_file(path))
            except (OSError, ValueError, VMFParseError, RecursionError):
                parsed[resolved_key] = None
        if parsed[resolved_key] is None:
            unresolved.append(name)
            continue
        queue.extend(_instance_refs(parsed[resolved_key], pairs))
    return sorted(files), sorted(set(unresolved))


def _cache_manifest_path(cache_dir: Path, stem: str) -> Path:
    return cache_dir / (stem + '.manifest.json')


def _read_cache_entry(cache_dir: Path, stem: str,
                      key: str) -> Tuple[Optional[dict], str]:
    """(entry, reason). entry = {'baseline', 'emissive_floor', 'bsp'} when the
    cache holds a trustworthy baseline for `key`; otherwise None, with a
    non-empty reason only when an entry for this key exists but is unusable (a
    plain miss or a stale key is silent)."""
    from bsp_reader import bsp_lighting_problem
    manifest = _cache_manifest_path(cache_dir, stem)
    if not manifest.is_file():
        return None, ''
    try:
        data = json.loads(manifest.read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return None, 'unreadable manifest'
    if (not isinstance(data, dict) or data.get('version') != _CACHE_VERSION
            or data.get('key') != key):
        return None, ''
    bsp_name = data.get('bsp_file')
    if (not isinstance(bsp_name, str) or not bsp_name
            or Path(bsp_name).name != bsp_name):
        return None, 'manifest names no BSP file'
    bsp = cache_dir / bsp_name
    baseline = data.get('baseline')
    floor = data.get('emissive_floor', 1)
    if not (isinstance(baseline, int) and 1 <= baseline <= 64):
        return None, 'invalid baseline scale'
    if not (isinstance(floor, int) and 1 <= floor <= 128):
        return None, 'invalid emissive floor'
    try:
        if bsp.stat().st_size != data.get('bsp_size'):
            return None, 'cached BSP size does not match the manifest'
    except OSError:
        return None, 'cached BSP missing or unreadable'
    # A matching key with an unlit BSP is the poisoned-cache case: it was
    # cached by a run whose VRAD never lit the map, and classifying from it
    # marks every face uniformly black.
    problem = bsp_lighting_problem(bsp)
    if problem:
        return None, f'cached BSP unusable: {problem}'
    return {'baseline': baseline, 'emissive_floor': floor, 'bsp': bsp}, ''


#: A superseded cache BSP is deleted only once this old (seconds). It guards
#: the one window the per-run handoff copy cannot: another run that has just
#: read the manifest and is about to copy that BSP. (On Windows a BSP that is
#: being copied cannot be deleted at all.)
CACHE_GC_MIN_AGE = 24 * 3600


def _write_cache_entry(cache_dir: Path, stem: str, key: str, baseline: int,
                       emissive_floor: int, src_bsp: Path,
                       gc_min_age: float = CACHE_GC_MIN_AGE) -> Path:
    """Store a lit baseline atomically and return the path of ITS BSP.

    Each entry's BSP has a unique name that the manifest points at, so two
    runs on the same map can never pair one run's key with the other run's BSP
    (a shared '<stem>.bsp' let the second writer replace the first's BSP under
    its feet). The manifest is written LAST (BSP via temp + os.replace first)
    and deleted FIRST on an update, so a crash anywhere leaves a cache miss.
    The BSP's mtime is set to NOW (copy2 would keep the source's old one and
    make a fresh entry instantly collectable). Superseded BSPs are removed
    once they are gc_min_age seconds old; no run depends on one, because
    every run (a --dry-run included) works on its own handoff copy (see
    _acquire_cache_hit).
    Raises OSError on any I/O failure (callers treat the cache as best-effort).
    """
    import shutil
    import uuid
    from vbsp_runner import atomic_tmp_name
    cache_dir.mkdir(parents=True, exist_ok=True)
    manifest = _cache_manifest_path(cache_dir, stem)
    # Invalidate first: any failure below leaves a miss, never a mismatch.
    for stale in [manifest] + [cache_dir / (stem + ext) for ext in _LEGACY_CACHE_EXTS]:
        try:
            if stale.is_file():
                stale.unlink()
        except OSError:
            pass
    bsp = cache_dir / f"{stem}.{uuid.uuid4().hex[:12]}.bsp"
    tmp_bsp = cache_dir / atomic_tmp_name(bsp.name, hidden=False)
    tmp_manifest = cache_dir / atomic_tmp_name(stem + '.manifest.json', hidden=False)
    try:
        shutil.copyfile(src_bsp, tmp_bsp)
        os.utime(tmp_bsp, None)   # mtime = now, not the source's
        os.replace(tmp_bsp, bsp)
        payload = {'version': _CACHE_VERSION, 'key': key, 'baseline': baseline,
                   'emissive_floor': emissive_floor, 'bsp_file': bsp.name,
                   'bsp_size': bsp.stat().st_size}
        tmp_manifest.write_text(json.dumps(payload), encoding='utf-8')
        os.replace(tmp_manifest, manifest)
    except OSError:
        try:
            if bsp.exists():
                bsp.unlink()
        except OSError:
            pass
        raise
    finally:
        for tmp in (tmp_bsp, tmp_manifest):
            try:
                if tmp.exists():
                    tmp.unlink()
            except OSError:
                pass
    _collect_old_cache_bsps(cache_dir, stem, keep=bsp.name, min_age=gc_min_age)
    return bsp


def _collect_old_cache_bsps(cache_dir: Path, stem: str, keep: str,
                            min_age: float) -> None:
    """Delete this map's superseded cache BSPs (unique-named and the legacy
    '<stem>.bsp') that are at least min_age seconds old. Never raises."""
    pattern = re.compile(r'^' + re.escape(stem) + r'(?:\.[0-9a-f]{12})?\.bsp$')
    now = time.time()
    try:
        entries = list(cache_dir.iterdir())
    except OSError:
        return
    for path in entries:
        if path.name == keep or not pattern.match(path.name):
            continue
        try:
            if now - path.stat().st_mtime >= min_age:
                path.unlink()
        except OSError:
            pass


def _handoff_bsp(cache_dir: Path, stem: str, src_bsp: Path,
                 temp_only: bool = False) -> Path:
    """Copy a lit baseline to a private file the pipeline can use for its whole
    run, whatever happens to the cache meanwhile (another run's GC, a rewrite),
    including the long classification and the bspwalk run that reads it by path.

    The copy sits in the cache folder under an owner-stamped temp name, or in the
    system temp folder under the same stamped name if the cache folder cannot
    be written; the sweep collects either after a hard kill. temp_only (a
    --dry-run, which must leave the map folder untouched) goes straight to the
    system temp folder. The caller deletes it. Raises FileNotFoundError if the SOURCE is gone (the caller re-reads the
    cache), other OSErrors if no folder can take the copy.
    """
    import shutil
    import tempfile
    from vbsp_runner import atomic_tmp_name
    if not Path(src_bsp).is_file():
        raise FileNotFoundError(str(src_bsp))
    name = atomic_tmp_name(stem + '.bsp', hidden=False)
    if not temp_only:
        try:
            cache_dir.mkdir(parents=True, exist_ok=True)
            dest = cache_dir / name
            shutil.copyfile(src_bsp, dest)
            return dest
        except FileNotFoundError:
            raise
        except OSError:
            pass
    dest = Path(tempfile.gettempdir()) / name
    shutil.copyfile(src_bsp, dest)
    return dest


def _acquire_cache_hit(cache_dir: Path, stem: str, key: str, attempts: int = 2,
                       temp_only: bool = False
                       ) -> Tuple[Optional[dict], Optional[Path], str]:
    """(entry, private BSP copy, reason) for a usable cache hit, or (None, None,
    reason). Reads the entry and immediately copies its BSP; if the BSP
    vanished in between (a concurrent writer superseded the entry and its GC
    got there) the manifest is re-read and the copy retried, and a vanishing
    that persists is a miss."""
    reason = ''
    for _ in range(attempts):
        entry, reason = _read_cache_entry(cache_dir, stem, key)
        if entry is None:
            return None, None, reason
        try:
            return entry, _handoff_bsp(cache_dir, stem, entry['bsp'],
                                       temp_only=temp_only), ''
        except FileNotFoundError:
            reason = 'cached BSP vanished while it was being copied'
    return None, None, reason


def _run_auto_compile_mode(args, root, input_path: Path, tools: _Tools,
                           t0: float) -> dict | None:
    """Compile a baseline BSP (or reuse the cached one), then run the BSP pipeline.

    Never writes the user's output file: baseline and trial VMFs are scratch
    siblings of the input map (ScratchVMF), the lit BSP goes to the cache
    directory, and main() publishes the final VMF once everything succeeded.
    Any failure raises (main exits 1) with the scratch files removed and
    nothing else touched. With --dry-run nothing is compiled and no cache or
    output is written: an existing cache entry is used read-only (its handoff
    copy goes to the system temp folder), otherwise the run is rejected.

    The pipeline always reads a private handoff copy of the lit BSP, never the
    cache file itself, and deletes it when done (a hard kill leaves a stamped
    file that the next run's sweep collects).

    The cache is best-effort. It is skipped (a note says why) when an
    instance the map pulls in cannot be resolved the way VBSP resolves it, when
    an input or tool cannot be fingerprinted, when the scratch compile had to
    fall back to the system temp folder (the baseline would lack the map's
    relative instances), and a failed cache write only warns.
    """
    from vbsp_runner import compile_bsp, ScratchVMF
    from vrad_runner import DEFAULT_TEXTURE_SHADOWS

    print("  Mode:   Auto-compile", flush=True)

    vbsp_path, game_dir, vrad_path, vvis_path = (
        tools.vbsp, tools.game_dir, tools.vrad, tools.vvis)  # _preflight guarantees all four
    bin_root = tools.bin_root
    lights_rad = tools.lights_rad
    level_rad = _level_rad_path(input_path)
    dry_run = bool(getattr(args, 'dry_run', False))
    texture_shadows = getattr(args, 'texture_shadows', DEFAULT_TEXTURE_SHADOWS)

    budget = args.vertex_budget - args.headroom

    # ─── Caching Setup ───────────────────────────────────────────────────────
    cache_dir = input_path.parent / '.lmopt_cache'

    # Eligibility is needed on every path (scan, compile, or cache hit) —
    # compute it once up front so the cache key can cover its output too.
    eligible, _eligible_skipped, side_map, disp_floor = \
        _compute_eligible_min_scales(root, args, game_dir)
    _apply_min_floor_scales(side_map, disp_floor)
    print(f"  {len(eligible)} eligible faces, "
          f"{_eligible_skipped} skipped (tools/entities)"
          + (f", {len(disp_floor)} skipped-disp clamped" if disp_floor else ""),
          flush=True)
    eligible_hash = hashlib.md5(repr(sorted(eligible)).encode()).hexdigest()

    cache_key: Optional[str] = None
    instance_files, unresolved = _instance_closure(input_path, root, game_dir)
    if unresolved:
        print(f"  ⚠ Baseline cache disabled for this run: instance(s) "
              f"{', '.join(unresolved[:5])}"
              f"{'...' if len(unresolved) > 5 else ''} could not be resolved "
              f"the way VBSP resolves them (beside the map, under its maps "
              f"folder, or gameinfo InstancePath), so their contents cannot "
              f"be fingerprinted.", file=sys.stderr, flush=True)
    else:
        try:
            cache_key = _build_cache_key(
                input_vmf=input_path, vbsp=vbsp_path, vrad=vrad_path,
                vvis=vvis_path, game_dir=game_dir, budget=budget,
                detail_min_scale=getattr(args, 'detail_min_scale', 5),
                rtx=getattr(args, 'rtx', False),
                texture_shadows=texture_shadows, bin_root=bin_root,
                lights_rad=lights_rad, level_rad=level_rad,
                eligible_hash=eligible_hash, instance_files=instance_files)
        except OSError as e:
            print(f"  ⚠ Baseline cache disabled for this run: cannot "
                  f"fingerprint the inputs/tools ({e}).", file=sys.stderr,
                  flush=True)

    # Every run, cached or not, works on its OWN copy of the lit BSP (the
    # handoff): the long classification and the bspwalk run read it by path for
    # minutes, and a concurrent run's cache write/GC must never be able to pull
    # it away. The cache entry is only a store that the handoff is copied from.
    entry = None
    handoff: Optional[Path] = None
    try:
        if cache_key is not None and not getattr(args, 'no_cache', False):
            # A dry run writes nothing beside the map: its handoff copy goes
            # to the system temp folder.
            entry, handoff, reason = _acquire_cache_hit(
                cache_dir, input_path.stem, cache_key, temp_only=dry_run)
            if reason:
                print(f"  ⚠ Cached baseline unusable ({reason}) — recompiling.",
                      flush=True)

        if entry is None and dry_run and (cache_key is None or getattr(args, 'no_cache', False)):
            raise PipelineAbort(
                "--dry-run in auto-compile mode needs a cached baseline BSP, "
                "but the cache is off for this run (--no-cache, or the "
                "warning above). Pass --bsp <lit BSP> for a dry run")
        if entry is None and dry_run:
            raise PipelineAbort(
                "--dry-run in auto-compile mode needs a cached baseline BSP "
                "for this exact map and tool set, and none exists. A dry run "
                "never compiles or writes a cache. Run once without "
                "--dry-run, or pass --bsp <lit BSP>")

        if entry is not None:
            baseline = entry['baseline']
            emissive_floor = entry['emissive_floor']
            bsp_for_pipeline = handoff
            print(f"  ✓ Cached baseline BSP found and inputs are unchanged "
                  f"(key matched).", flush=True)
            print(f"  Skipping baseline compile pipeline.", flush=True)
            # Re-apply the cached baseline to this run's in-memory tree (the
            # input VMF is never modified, so nothing persists between runs).
            _apply_baseline_scales(side_map, eligible, baseline)
            print(f"  ✓ Baseline scale={baseline} re-applied from cache", flush=True)
        else:
            # ─── Step 1: Find safe baseline scale ────────────────────────────
            print(f"\n[2/8] Finding safe baseline lightmapscale...", flush=True)
            baseline = _find_safe_baseline(
                root, args, vbsp_path, game_dir, input_path,
                vertex_budget=budget,
                eligible_sids=eligible,
                side_map=side_map,
                verbose=True,
            )

            if baseline == 0:
                raise PipelineAbort(
                    "the map exceeds the vertex budget even at lightmapscale "
                    "64 (too many brushes)")

            print(f"  ✓ Baseline scale={baseline} fits budget", flush=True)

            # ─── Step 2: Full compile (VBSP → VVIS → VRAD) on a scratch VMF ──
            # _find_safe_baseline applied the baseline to all eligible side
            # nodes on root. Everything below compiles a private sibling of the
            # input (so relative instances resolve) and leaves the user's
            # output path and any compile products next to it alone.
            print(f"\n[3/8] Compiling baseline BSP with lighting...", flush=True)
            with ScratchVMF(input_path, 'baseline') as work:
                work.mirror_level_rad(level_rad)
                work.write(root)

                print("  [3a/8] Running VBSP...", flush=True)
                compile_bsp(
                    vbsp_path, work.path, game_dir,
                    bin_root=bin_root,
                    verbose=args.verbose, timeout=300,
                    extra_args=['-emitsideids']
                )

                # VVIS -fast is required for accurate VRAD lighting: without
                # VIS data VRAD sets numbounce=0 (direct light only), so
                # shadowed faces read as uniformly dark. A failed run must not
                # produce a baseline that gets cached as valid.
                print("  [3b/8] Running VVIS -fast...", flush=True)
                if not _run_vvis_fast(vvis_path, work.bsp, game_dir,
                                      bin_root=bin_root, verbose=args.verbose):
                    raise PipelineAbort(
                        "VVIS -fast failed. A baseline lit without VIS data "
                        "has no bounce lighting and would poison the "
                        "classification (and the cache)")

                # Step 2b.5: fit the baseline under VRAD's hard light limit.
                # Fine lightmap scales turn every texlight patch into a direct
                # light, and VRAD hard-errors past 32,767 ('Too many lights')
                # without writing any lighting data. Raise texlight faces
                # (only) to the smallest scale that fits so the lighting
                # compile below survives; the scale found is carried into the
                # solver as a floor.
                print("  [3b.5/8] Checking baseline surface light count...",
                      flush=True)
                emissive_floor = _fit_baseline_light_budget(
                    root, work.path, vbsp_path, vvis_path, vrad_path,
                    game_dir, bin_root, lights_rad, eligible_sids=eligible,
                    extra_rads=[level_rad], verbose=args.verbose,
                )

                # Step 2c: VRAD fast compile (produces actual lighting data).
                # Required: without it every face classifies as unknown, and an
                # unlit baseline once produced wholesale 'uniform black'
                # classifications and garbage scales. Fail loudly instead.
                print("  [3c/8] Running VRAD fast compile...", flush=True)
                from vrad_runner import compile_rad
                from bsp_reader import bsp_lighting_problem
                compile_rad(
                    vrad_path, work.bsp, game_dir,
                    bin_root=bin_root,
                    verbose=args.verbose, timeout=600,
                    lights_rad=lights_rad, rtx=getattr(args, 'rtx', False),
                    texture_shadows=texture_shadows,
                    stream_output=True,
                )
                problem = bsp_lighting_problem(work.bsp)
                if problem:
                    # VRAD exited 0 but the BSP has no usable lightmap samples
                    # — treat exactly like a failed compile rather than
                    # classifying from empty lighting data.
                    raise PipelineAbort(
                        f"VRAD exited 0 but the BSP has no usable lighting "
                        f"({problem}). If your texlights live in a .rad file "
                        f"this tool could not find, pass --lights")
                print("  ✓ VRAD fast compile complete — BSP has lighting data",
                      flush=True)

                # Private copy first (taken while the scratch BSP still exists),
                # then the best-effort cache store.
                handoff = _handoff_bsp(cache_dir, input_path.stem, work.bsp)
                bsp_for_pipeline = handoff
                if cache_key is None:
                    pass   # fingerprinting was impossible: never cache
                elif work.relocated:
                    print("  ⚠ Baseline not cached: the input folder is not "
                          "writable, so the baseline was compiled away from the "
                          "map's relative instances.", file=sys.stderr, flush=True)
                else:
                    try:
                        _write_cache_entry(
                            cache_dir, input_path.stem, cache_key, baseline,
                            emissive_floor, work.bsp)
                    except OSError as e:
                        print(f"  ⚠ Baseline not cached ({e}); continuing "
                              f"without the cache.", file=sys.stderr, flush=True)

        # Point args.bsp at the lit baseline and run the standard BSP pipeline
        args.bsp = str(bsp_for_pipeline)
        return _run_bsp_mode(args, root, t0, tools, emissive_floor=emissive_floor)
    finally:
        if handoff is not None:
            try:
                handoff.unlink()
            except OSError:
                pass


if __name__ == '__main__':
    sys.exit(main())
