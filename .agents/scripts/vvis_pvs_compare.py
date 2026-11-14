#!/usr/bin/env python3
r"""Run vvis variants on copies of the same BSP+PRT pairs and compare the PVS.

For a change to src/utils/vvis that must not move the PVS (or must move it in
a way that can be listed pair by pair). Every run works on its own copy, so the
input pairs stay pristine.

Usage:
  python -I .agents/scripts/vvis_pvs_compare.py --maps <dir> --out <workdir> \
      --run base=<bin dir> --run new=<bin dir> [--run t1=<bin dir>,-threads,1] \
      [--only map,map] [--game <dir>] [--reuse]

  --maps   directory holding <map>/<map>.bsp and <map>/<map>.prt
  --run    label=<directory with vvis_optix.exe>[,arg,arg...]; the first run
           is the reference every other run is compared with. A run can take
           its portal files from another directory: label=<bin dir>@<prt dir>
           reads <prt dir>/<map>.prt in place of the map's own (the BSP stays
           the one from --maps), to compare edited portal files
  --reuse  keep the result of a run whose output BSP is already in --out

Prints one line per (map, run): the sha256 of the visibility lump (PVS and
PAS, so "same" means byte-identical), the flow's own line from the log, and
for a run that differs from the reference the unordered cluster pairs it
gains and loses. Exit code 0 when every run equals the reference, 1 otherwise.
"""
import argparse
import hashlib
import os
import shutil
import struct
import subprocess
import sys

LUMP_VISIBILITY = 4
# The Steam library differs per machine (the dev box has it on E:, Server1 on C:).
STEAM_COMMON = os.environ.get("SEMT_STEAM_COMMON", r"E:\Steam\steamapps\common")
DEFAULT_GAME = os.path.join(STEAM_COMMON, "Source SDK Base 2013 Multiplayer", "sourcetest")


def vis_lump(path):
    with open(path, "rb") as f:
        data = f.read()
    if data[:4] != b"VBSP":
        raise ValueError(f"{path}: not a BSP")
    ofs, length = struct.unpack_from("<II", data, 8 + LUMP_VISIBILITY * 16)
    return data[ofs:ofs + length]


def pvs_rows(lump):
    """One int per cluster, bit j set when cluster j is in its PVS."""
    if not lump:
        return []
    n = struct.unpack_from("<I", lump, 0)[0]
    row_bytes = (n + 7) // 8
    rows = []
    for i in range(n):
        ofs = struct.unpack_from("<i", lump, 4 + i * 8)[0]
        out = bytearray(row_bytes)
        o = 0
        while o < row_bytes:
            v = lump[ofs]
            if v == 0:
                o += lump[ofs + 1]
                ofs += 2
            else:
                out[o] = v
                o += 1
                ofs += 1
        rows.append(int.from_bytes(out, "little") & ((1 << n) - 1))
    return rows


def pairs_only_in(a, b):
    """Unordered pairs (i, j), i < j, visible in rows a and not in rows b."""
    out = set()
    for i, (ra, rb) in enumerate(zip(a, b)):
        d = ra & ~rb
        while d:
            j = (d & -d).bit_length() - 1
            d &= d - 1
            out.add((min(i, j), max(i, j)))
    return sorted(out)


def flow_line(log_text):
    for line in log_text.splitlines():
        if line.startswith("PortalFlow:") or line.startswith("MergeFlow ("):
            return line.strip()
    return ""


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--maps", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--run", action="append", required=True)
    ap.add_argument("--only", default="")
    ap.add_argument("--game", default=DEFAULT_GAME)
    ap.add_argument("--reuse", action="store_true")
    ap.add_argument("--max-pairs", type=int, default=20,
                    help="pairs listed per direction for a differing run")
    args = ap.parse_args()

    runs = []
    for spec in args.run:
        label, _, rest = spec.partition("=")
        parts = rest.split(",")
        bindir, _, prtdir = parts[0].partition("@")
        exe = os.path.join(bindir, "vvis_optix.exe")
        if not os.path.isfile(exe):
            sys.exit(f"no vvis_optix.exe in {bindir}")
        runs.append((label, exe, parts[1:], prtdir))

    maps = sorted(d for d in os.listdir(args.maps)
                  if os.path.isfile(os.path.join(args.maps, d, d + ".bsp")))
    if args.only:
        wanted = args.only.split(",")
        missing = [m for m in wanted if m not in maps]
        if missing:
            sys.exit(f"not in {args.maps}: {missing}")
        maps = wanted

    all_same = True
    for m in maps:
        ref_rows = ref_hash = None
        for run_index, (label, exe, extra, prtdir) in enumerate(runs):
            work = os.path.join(args.out, label, m)
            bsp = os.path.join(work, m + ".bsp")
            log = os.path.join(work, "vvis_stdout.log")
            if not (args.reuse and os.path.isfile(log)):
                os.makedirs(work, exist_ok=True)
                shutil.copyfile(os.path.join(args.maps, m, m + ".bsp"), bsp)
                shutil.copyfile(os.path.join(prtdir or os.path.join(args.maps, m), m + ".prt"),
                                os.path.join(work, m + ".prt"))
                cmd = [exe, "-game", args.game] + extra + [os.path.join(work, m)]
                res = subprocess.run(cmd, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True,
                                     errors="replace")
                if res.returncode != 0:
                    print(f"{m:28s} {label:8s} FAILED rc={res.returncode}")
                    print(res.stdout[-2000:])
                    all_same = False
                    if run_index == 0:
                        print(f"{m:28s} the reference run failed; nothing to compare with")
                        break
                    continue
                with open(log, "w") as f:
                    f.write(res.stdout)
            with open(log) as f:
                text = f.read()
            lump = vis_lump(bsp)
            digest = hashlib.sha256(lump).hexdigest()[:16]
            rows = pvs_rows(lump)
            if ref_rows is None:
                ref_rows, ref_hash = rows, digest
                verdict = "reference"
            elif digest == ref_hash:
                verdict = "same"
            else:
                all_same = False
                gained = pairs_only_in(rows, ref_rows)
                lost = pairs_only_in(ref_rows, rows)
                verdict = f"DIFFERS gains {len(gained)} pairs, loses {len(lost)}"
                if rows == ref_rows:
                    verdict = "DIFFERS in the lump bytes only (PVS rows equal)"
                for name, pairs in (("gains", gained), ("loses", lost)):
                    if pairs:
                        shown = " ".join(f"{i}<->{j}" for i, j in pairs[:args.max_pairs])
                        more = "" if len(pairs) <= args.max_pairs else " ..."
                        verdict += f"\n{'':38s}{name}: {shown}{more}"
            print(f"{m:28s} {label:8s} {digest}  {verdict}  | {flow_line(text)}")
            sys.stdout.flush()
    return 0 if all_same else 1


if __name__ == "__main__":
    sys.exit(main())
