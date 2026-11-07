"""Compile a VMF to a lit BSP the same way lmoptimizer compiles its baseline.

vbsp_lmo -emitsideids, vvis -fast, then VRAD with the runner's fast flags
(-noextra -bounce 2 ...) and optionally -rtx. The output is directly
comparable with lmoptimizer's cached baseline in lightmap_verify.py.

--scale N rewrites every non-displacement "lightmapscale" to N in a copy first: a uniformly
finer compile is the reference lightmap_verify.py --reference measures the
baseline and an optimized map against.

Usage:
  python lit_compile.py MAP.vmf --game GAMEDIR --out DIR [--name STEM]
         [--scale N] [--maxlightmapdim D] [--rtx] [--vbsp EXE] [--vvis EXE] [--vrad EXE]
"""
from __future__ import annotations

import argparse
import re
import shutil
import sys
from pathlib import Path

from vbsp_runner import compile_bsp, find_vbsp
from vrad_runner import compile_rad, find_vrad

_SCALE_RE = re.compile(r'("lightmapscale"\s+")(\d+)(")')


def force_scale(text: str, scale: int):
    """Set every non-displacement side's lightmapscale; returns (text, count).

    Displacements are never chopped to the lightmap limit, so a fine scale
    overflows VRAD's 125 luxels on large ones; lightmap_verify skips them
    anyway, so they keep their own scale.
    """
    out, buf, depth, side_depth, n = [], None, 0, None, 0
    lines = text.splitlines(keepends=True)
    for i, line in enumerate(lines):
        tok = line.strip()
        if buf is None and tok == "side" and i + 1 < len(lines) and lines[i + 1].strip() == "{":
            buf, side_depth = [], depth
        (buf if buf is not None else out).append(line)
        depth += tok.count("{") - tok.count("}")
        if buf is not None and depth == side_depth and tok == "}":
            block = "".join(buf)
            if "dispinfo" not in block:
                block, k = _SCALE_RE.subn(lambda m: f"{m[1]}{scale}{m[3]}", block)
                n += k
            out.append(block)
            buf = None
    return "".join(out), n


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("vmf", type=Path)
    ap.add_argument("--game", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--name", help="output stem (default: the VMF's); keep it dotless")
    ap.add_argument("--scale", type=int, help="force every side to this lightmapscale")
    ap.add_argument("--maxlightmapdim", type=int,
                    help="VBSP -maxlightmapdim; with --scale S use 32*12/S so a finer "
                         "reference keeps the baseline's face splits (and vertex count)")
    ap.add_argument("--rtx", action="store_true")
    ap.add_argument("--vbsp", type=Path)
    ap.add_argument("--vvis", type=Path)
    ap.add_argument("--vrad", type=Path)
    ap.add_argument("--timeout", type=int, default=3600)
    a = ap.parse_args(argv)

    here = Path(__file__).resolve().parent
    vbsp = a.vbsp or find_vbsp()
    vrad = a.vrad or find_vrad()
    vvis = a.vvis or next((here / n for n in ("vvis.exe", "vvis_optix.exe")
                           if (here / n).is_file()), None)
    for label, exe in (("VBSP", vbsp), ("VVIS", vvis), ("VRAD", vrad)):
        if not exe or not Path(exe).is_file():
            print(f"ERROR: {label} not found (pass --{label.lower()})", file=sys.stderr)
            return 1

    a.out.mkdir(parents=True, exist_ok=True)
    stem = a.name or a.vmf.stem
    work = a.out / f"{stem}.vmf"
    if a.scale:
        text = a.vmf.read_text(encoding="utf-8", errors="surrogateescape")
        text, n = force_scale(text, a.scale)
        work.write_text(text, encoding="utf-8", errors="surrogateescape")
        print(f"  set {n} sides to lightmapscale {a.scale}")
    elif a.vmf.resolve() != work.resolve():
        shutil.copyfile(a.vmf, work)

    vbsp_args = ["-emitsideids"]
    if a.maxlightmapdim:
        vbsp_args += ["-maxlightmapdim", str(a.maxlightmapdim)]
    bsp = compile_bsp(vbsp, work, a.game, timeout=a.timeout, extra_args=vbsp_args)
    # lmoptimizer's own VVIS step: VRAD skips patch subdivision without VIS.
    from lmoptimizer import _run_vvis_fast
    if not _run_vvis_fast(vvis, bsp, a.game, timeout=a.timeout):
        print("ERROR: VVIS -fast failed", file=sys.stderr)
        return 1
    compile_rad(vrad, bsp, a.game, timeout=a.timeout, rtx=a.rtx)
    print(bsp)
    return 0


if __name__ == "__main__":
    sys.exit(main())
