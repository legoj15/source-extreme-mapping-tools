#!/usr/bin/env python3
"""Does `vrad_rtx -cuda` light a map the same however its supersampling passes are batched?

    python -I .agents/scripts/vrad_ss_batch_check.py --game <dir> --out <dir> [--bin <bin dir>]
        [--capacity 1000000] [--capacity 100000] [--vrad-args "-bounce 0"] map.bsp [map.bsp ...]

The GPU supersampler holds a pass's sub-positions in one buffer (120M entries, halved when the
allocation fails) and cuts a pass that does not fit into batches. Which samples a pass takes must
not depend on that cut: it would make the lighting depend on free memory. For each unlit BSP this
lights a copy with the default buffer and one copy per --capacity (the buffer size in entries,
through the VRAD_SS_SUBPOS_CAPACITY environment variable; default 1000000 and 100000, which cut the
comparison maps' first pass into several batches), and compares every lump of each with the
default run's. Exit 0: every lump of every run byte-identical; 1: a difference (the lumps are
listed) or a run that took no more batches than the default one (the check then proved nothing:
use a smaller --capacity); 2: usage or a failed compile.

Run it after any change to the GPU supersampling passes (`SSPass_*` in src/utils/vrad/lightmap.cpp,
the pass loop in vrad.cpp). Needs the GPU. The inputs are unlit BSPs, for example the ones
`tests/compare/vrad_noise_floor.ps1` leaves in `tests/out/vrad_noise_floor/<test>/ref/` when run
with -SkipRef, or any `vbsp_lmo` output.
"""
import argparse
import os
import shutil
import struct
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
LIGHT_LUMPS = {7: "faces", 8: "lighting", 15: "world lights", 51: "leaf ambient index (HDR)", 52: "leaf ambient index",
               53: "lighting (HDR)", 54: "world lights (HDR)", 55: "leaf ambient lighting (HDR)",
               56: "leaf ambient lighting", 58: "faces (HDR)"}


def lumps(path):
    data = path.read_bytes()
    out = {}
    for i in range(64):
        ofs, length = struct.unpack_from("<2i", data, 8 + i * 16)
        out[i] = data[ofs:ofs + length] if length > 0 else b""
    return out


def light(vrad, game, src, folder, args, capacity):
    folder.mkdir(parents=True, exist_ok=True)
    bsp = folder / src.name
    shutil.copyfile(src, bsp)
    env = dict(os.environ)
    env.pop("VRAD_SS_SUBPOS_CAPACITY", None)
    if capacity:
        env["VRAD_SS_SUBPOS_CAPACITY"] = str(capacity)
    cmd = [str(vrad)] + args + ["-cuda", "-game", str(game), str(bsp.with_suffix(""))]
    log = folder / "console.log"
    with open(log, "wb") as fp:
        rc = subprocess.run(cmd, cwd=str(vrad.parent), env=env, stdout=fp, stderr=subprocess.STDOUT).returncode
    text = log.read_text(encoding="utf-8", errors="replace")
    uploads = text.count("SSPass GPU: Uploading")
    passes = sum(1 for ln in text.splitlines() if ln.strip().startswith("SS Pass "))
    return rc, bsp, uploads, passes


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--game", required=True, help="game directory (the one with gameinfo.txt)")
    ap.add_argument("--out", required=True, type=Path, help="folder for the lit copies and logs")
    ap.add_argument("--bin", type=Path, default=REPO / "game" / "bin" / "x64", help="folder with vrad_rtx.exe")
    ap.add_argument("--capacity", type=int, action="append", help="buffer size in entries (repeatable)")
    ap.add_argument("--vrad-args", default="", help="extra vrad arguments, one string")
    ap.add_argument("maps", nargs="+", type=Path, help="unlit BSPs")
    a = ap.parse_args()
    vrad = a.bin.resolve() / "vrad_rtx.exe"
    if not vrad.is_file():
        print(f"error: no {vrad}", file=sys.stderr)
        return 2
    a.game = str(Path(a.game).resolve())   # vrad runs from its own folder
    capacities = a.capacity or [1000000, 100000]
    extra = a.vrad_args.split()
    bad = 0
    for src in a.maps:
        src = src.resolve()
        if not src.is_file():
            print(f"error: no {src}", file=sys.stderr)
            return 2
        base = a.out.resolve() / src.stem
        rc, ref_bsp, uploads, passes = light(vrad, a.game, src, base / "default", extra, 0)
        if rc != 0:
            print(f"error: {src.name}: vrad_rtx failed with the default buffer (exit {rc}, {base / 'default' / 'console.log'})",
                  file=sys.stderr)
            return 2
        ref = lumps(ref_bsp)
        print(f"{src.name}: default buffer: {passes} pass(es) in {uploads} batch(es)")
        for cap in capacities:
            rc, bsp, up, ps = light(vrad, a.game, src, base / f"cap{cap}", extra, cap)
            if rc != 0:
                print(f"error: {src.name}: vrad_rtx failed at {cap:,} entries (exit {rc})", file=sys.stderr)
                return 2
            got = lumps(bsp)
            differ = [i for i in sorted(ref) if ref[i] != got[i]]
            if up <= uploads:
                print(f"  {cap:>11,} entries: {ps} pass(es) in {up} batch(es): no more than the default run, "
                      f"nothing was checked  FAIL")
                bad += 1
            elif differ:
                names = ", ".join(f"{i} ({LIGHT_LUMPS.get(i, 'other')})" for i in differ)
                print(f"  {cap:>11,} entries: {ps} pass(es) in {up} batch(es): lumps differ: {names}  FAIL")
                bad += 1
            else:
                print(f"  {cap:>11,} entries: {ps} pass(es) in {up} batch(es): every lump identical  PASS")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
