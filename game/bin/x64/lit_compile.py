"""Compile a VMF to a lit BSP the same way lmoptimizer compiles its baseline.

vbsp_lmo -emitsideids, vvis -fast, then VRAD with the runner's fast flags
(-noextra -bounce 2 ...), the --texture-shadows level's flag (default props:
-textureshadows) and optionally -rtx. The output is directly comparable with
lmoptimizer's cached baseline in lightmap_verify.py when both use the same
--texture-shadows level.

The map's own <map>.rad is copied next to the output under the output stem
(VRAD reads '<bsp stem>.rad'), and --lights passes a custom lights file, as
lmoptimizer does. Not reproduced: lmoptimizer's texlight light-budget fit; a
map whose baseline needed it will hit VRAD's light limit here, loudly.

--scale N rewrites every non-displacement side's lightmapscale to N in a
copy first (adding the key where a side has none): a uniformly finer
compile is the reference lightmap_verify.py --reference measures the
baseline and an optimized map against.

Use this repo's vrad_rtx.exe (pass --vrad if this folder has none): the
older E:/GitHub/vrad-rtx build overflows on faces over 32 luxels, which a
--maxlightmapdim reference produces.

Usage:
  python lit_compile.py MAP.vmf --game GAMEDIR --out DIR [--name STEM]
         [--scale N] [--maxlightmapdim D] [--lights FILE] [--binroot DIR] [--rtx]
         [--texture-shadows {none,props,world,translucent,all}]
         [--vbsp EXE] [--vvis EXE] [--vrad EXE]
"""
from __future__ import annotations

import argparse
import os
import re
import shutil
import sys
from pathlib import Path

from vbsp_runner import compile_bsp, find_vbsp
from vrad_runner import (DEFAULT_TEXTURE_SHADOWS, TEXTURE_SHADOW_LEVELS,
                         compile_rad, find_vrad)

# Any quoted value, so a "16.0" is replaced rather than duplicated. Key and
# value on one line, as Hammer writes them and vmf_parser requires: bridging
# to the next line would take the next pair's key as the value.
_SCALE_RE = re.compile(r'(^[ \t]*"lightmapscale"[ \t]+")([^"]*)(")', re.M)
_QUOTED_RE = re.compile(r'"[^"]*"')


def _braces(line: str) -> int:
    """Net brace depth change of a VMF line, ignoring braces in quoted values
    (a material such as "{fence01")."""
    bare = _QUOTED_RE.sub("", line)
    return bare.count("{") - bare.count("}")


def force_scale(text: str, scale: int):
    """Set every non-displacement side's lightmapscale; returns (text, count).

    A side without the key (VBSP's implicit 16) gets one. Displacements are
    never chopped to the lightmap limit, so a fine scale overflows VRAD's
    125 luxels on large ones; lightmap_verify skips them anyway, so they
    keep their own scale.
    """
    out, buf, depth, side_depth, n = [], None, 0, None, 0
    lines = text.splitlines(keepends=True)
    for i, line in enumerate(lines):
        tok = line.strip()
        if (buf is None and tok == "side" and i + 1 < len(lines)
                and lines[i + 1].strip() == "{"):
            buf, side_depth = [], depth
        (buf if buf is not None else out).append(line)
        depth += _braces(tok)
        if buf is not None and depth == side_depth and tok == "}":
            block = "".join(buf)
            if "dispinfo" not in block:
                block, k = _SCALE_RE.subn(lambda m: f"{m[1]}{scale}{m[3]}", block)
                if k == 0:
                    keys = [ln for ln in buf if ln.lstrip().startswith('"')]
                    indent = keys[0][:len(keys[0]) - len(keys[0].lstrip())] if keys else "\t"
                    eol = "\r\n" if buf[-1].endswith("\r\n") else "\n"
                    block = "".join(buf[:-1]) + f'{indent}"lightmapscale" "{scale}"{eol}' + buf[-1]
                    k = 1
                n += k
            out.append(block)
            buf = None
    if buf is not None:
        raise ValueError("unclosed side block at end of VMF")
    return "".join(out), n


def same(x: Path, y: Path) -> bool:
    """Same file, by resolved path, case-insensitively where the OS is."""
    return os.path.normcase(x.resolve()) == os.path.normcase(y.resolve())


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("vmf", type=Path)
    ap.add_argument("--game", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--name", help="output stem (default: the VMF's); keep it dotless")
    ap.add_argument("--scale", type=int, help="force every side to this lightmapscale")
    ap.add_argument("--maxlightmapdim", type=int,
                    help="VBSP -maxlightmapdim; with --scale S use 32*B/S for a baseline "
                         "at global scale B, so a finer reference keeps the baseline's "
                         "face splits (and vertex count)")
    ap.add_argument("--lights", type=Path, help="custom lights .rad passed to VRAD")
    ap.add_argument("--binroot", type=Path, help="engine bin root passed to every tool")
    ap.add_argument("--rtx", action="store_true")
    ap.add_argument("--texture-shadows", default=DEFAULT_TEXTURE_SHADOWS,
                    choices=list(TEXTURE_SHADOW_LEVELS),
                    help="VRAD texture-shadow level; match lmoptimizer's "
                         "--texture-shadows (and your final compile) so the "
                         "reference lighting shows the same shadows")
    ap.add_argument("--vbsp", type=Path)
    ap.add_argument("--vvis", type=Path)
    ap.add_argument("--vrad", type=Path)
    ap.add_argument("--timeout", type=int, default=3600)
    a = ap.parse_args(argv)
    if a.scale is not None and a.scale < 1:
        ap.error("--scale must be 1 or more")

    here = Path(__file__).resolve().parent
    vbsp = a.vbsp or find_vbsp()
    vrad = a.vrad or find_vrad()
    vvis = a.vvis or next((here / n for n in ("vvis.exe", "vvis_optix.exe")
                           if (here / n).is_file()), None)
    for label, exe in (("VBSP", vbsp), ("VVIS", vvis), ("VRAD", vrad)):
        if not exe or not Path(exe).is_file():
            print(f"ERROR: {label} not found (pass --{label.lower()})", file=sys.stderr)
            return 1
        print(f"  {label}: {exe}")
    if not same(Path(vrad).parent, here):
        print("  WARNING: VRAD is not this folder's build; an older build may crash "
              "on faces over 32 luxels", file=sys.stderr)

    a.out.mkdir(parents=True, exist_ok=True)
    stem = a.name or a.vmf.stem
    work = a.out / f"{stem}.vmf"
    if a.scale is not None and same(work, a.vmf):
        print("ERROR: --scale would overwrite the input VMF; pass --name or "
              "another --out", file=sys.stderr)
        return 1
    if a.scale is not None:
        text = a.vmf.read_text(encoding="utf-8", errors="surrogateescape")
        try:
            text, n = force_scale(text, a.scale)
        except ValueError as e:
            print(f"ERROR: {e} (malformed input VMF?)", file=sys.stderr)
            return 1
        if n == 0:
            print("ERROR: --scale found no side blocks to rescale (every side is a "
                  "displacement, or 'side' and '{' are not on separate lines)",
                  file=sys.stderr)
            return 1
        work.write_text(text, encoding="utf-8", errors="surrogateescape")
        print(f"  set lightmapscale {a.scale} on {n} non-displacement sides")
    elif not same(a.vmf, work):
        shutil.copyfile(a.vmf, work)

    level_rad = a.vmf.with_suffix(".rad")
    work_rad = work.with_suffix(".rad")
    if level_rad.is_file():
        if not same(level_rad, work_rad):
            shutil.copyfile(level_rad, work_rad)
    elif work_rad.is_file():
        # VRAD reads <out stem>.rad: one left by an earlier run of another
        # map in this folder would light this one with its texlights.
        print(f"  removing {work_rad}: the input map has no .rad")
        try:
            work_rad.unlink()
        except OSError as e:
            print(f"ERROR: cannot remove stale {work_rad}: {e}", file=sys.stderr)
            return 1

    vbsp_args = ["-emitsideids"]
    if a.maxlightmapdim:
        vbsp_args += ["-maxlightmapdim", str(a.maxlightmapdim)]
    bsp = compile_bsp(vbsp, work, a.game, bin_root=a.binroot, timeout=a.timeout,
                      extra_args=vbsp_args)
    # lmoptimizer's own VVIS step: VRAD skips patch subdivision without VIS.
    from lmoptimizer import _run_vvis_fast
    if not _run_vvis_fast(vvis, bsp, a.game, bin_root=a.binroot, timeout=a.timeout):
        print("ERROR: VVIS -fast failed", file=sys.stderr)
        return 1
    compile_rad(vrad, bsp, a.game, bin_root=a.binroot, timeout=a.timeout, rtx=a.rtx,
                lights_rad=a.lights, texture_shadows=a.texture_shadows)
    print(bsp)
    return 0


if __name__ == "__main__":
    sys.exit(main())
