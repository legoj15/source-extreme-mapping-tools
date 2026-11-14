#!/usr/bin/env python3
r"""Compile maps under several VBSP block-grid settings and compare what the PVS costs.

For any question about the block grid (-blocksize, a grid rule, a default):
each (map, variant) is compiled with vbsp_lmo, run through vvis_optix and
measured with game/bin/x64/pvs_cost.py. One table per map, the first variant
being the reference the percentages are against.

Usage:
  python -I .agents/scripts/blockgrid_sweep.py --game <dir> --out <workdir> \
      --variant 1024= --variant "2048=-blocksize 2048" [--variant label=args ...] \
      [--vis=-mergeflow] [--bin <dir>] [--strip-visclusters] [--reuse | --remeasure] \
      [--json <file>] map.vmf [map.vmf ...]

  --variant  label=<vbsp arguments>; an empty argument list is the stock grid
  --vis      extra vvis arguments (none: the exact flow). Write it with "=",
             as in --vis=-mergeflow or --vis="-mergeflow -threads 8"; a value
             that starts with a dash is otherwise taken for an option
  --bin      directory with vbsp_lmo.exe and vvis_optix.exe (default: this
             checkout's game/bin/x64)
  --strip-visclusters  compile a copy of the VMF without its func_viscluster
             entities, so the grid alone decides the clusters
  --reuse    keep a (map, variant) whose result is already in --out
  --json     also write every row to this file

Columns: clusters and portals from the .prt, vvis wall seconds (on a shared
box only a 2x difference means anything), the map's world triangle count, and
from pvs_cost.py, for a viewpoint uniform over the floor, the share of the
floor in the PVS and the expected world triangles and static props in it.
"volume tris" is the older volume-weighted triangle number of the #96 and #105
tables (pvs_cost.py says why it misleads). Read triangles against "map tris":
a coarser grid cuts fewer faces, so it can show fewer triangles while the PVS
itself got looser; floor seen and props do not move with the face count.
--remeasure re-runs pvs_cost.py on the BSPs a previous run left in --out, and
compiles any variant that has no result there yet. A stored result is only
used when its vbsp arguments, vis arguments and --strip-visclusters match this
run's; otherwise that variant is compiled again. A row ending in
[areaportal leak] compiled, but vbsp reported that leak: a grid line can cut
an areaportal brush so it no longer seals. The PVS numbers do not depend on
areas; the BSP is still not one to ship.
Needs numpy. Exit code 1 if any compile failed.
"""
import argparse
import json
import re
import shlex
import shutil
import struct
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
BIN = REPO / "game" / "bin" / "x64"
sys.path.insert(0, str(BIN))  # python -I drops this script's folder
import pvs_cost  # noqa: E402


VMF_TOKEN = re.compile(r'"[^"]*"|[{}]|[^\s{}"]+')  # a quoted string, a brace, a bare word
# vbsp finishes with exit code 0 on these; the BSP is still not what the map meant
VBSP_WARNINGS = (("Areaportal leak", "areaportal leak"),)


def strip_visclusters(text):
    """The VMF without its top-level func_viscluster entity blocks.

    A top-level block is a bare word and its braces: `entity { ... }`. Braces
    inside quoted values are text, not structure, and the classname counts only
    as the entity's own key, not inside a nested block. A VMF with an unclosed
    quote is refused: every later brace would be read as text.
    """
    if text.count('"') % 2:
        raise ValueError("the VMF has an odd number of quote characters (one is not closed)")
    out, last, depth, start, name, key, viscluster = [], 0, 0, None, None, None, False
    for m in VMF_TOKEN.finditer(text):
        token = m.group()
        if token == "{":
            depth += 1
            key = None
        elif token == "}":
            depth -= 1
            key = None
            if depth == 0:
                if start is not None and name == "entity" and viscluster:
                    out.append(text[last:start])
                    last = m.end()
                start, viscluster = None, False
        elif depth == 0 and start is None:
            start, name = m.start(), token
        elif depth == 1:
            if key is None and token.startswith('"'):
                key = token.lower()  # the first of a "key" "value" pair
            else:
                if key == '"classname"' and token.lower() == '"func_viscluster"':
                    viscluster = True
                key = None  # a value, or the bare name of a nested block
    out.append(text[last:])
    return "".join(out)


def settings(vbsp_args, opts):
    """What decides a row's result besides the map; stored with it and checked before a reuse."""
    return {"vbsp_args": list(vbsp_args), "vis_args": list(opts.vis),
            "strip_visclusters": bool(opts.strip_visclusters)}


def run(cmd, log):
    t0 = time.perf_counter()
    with open(log, "w", encoding="utf-8", errors="replace") as f:
        rc = subprocess.call([str(c) for c in cmd], cwd=str(Path(cmd[0]).parent),
                             stdout=f, stderr=subprocess.STDOUT)
    return rc, time.perf_counter() - t0


def compile_variant(vmf, work, vbsp_args, opts):
    work.mkdir(parents=True, exist_ok=True)
    name = vmf.stem
    target = work / vmf.name
    row = {"map": name, **settings(vbsp_args, opts)}
    if opts.strip_visclusters:
        try:
            # latin-1 maps every byte to one character and back, so nothing else changes
            target.write_bytes(strip_visclusters(vmf.read_bytes().decode("latin-1")).encode("latin-1"))
        except ValueError as e:
            row["error"] = f"--strip-visclusters: {e}"
            return row
    else:
        shutil.copyfile(vmf, target)
    for ext in (".bsp", ".prt", ".lin"):
        (work / (name + ext)).unlink(missing_ok=True)

    rc, row["vbsp_s"] = run([opts.bin / "vbsp_lmo.exe", "-game", opts.game] + vbsp_args + [target],
                            work / "vbsp.log")
    prt = work / (name + ".prt")
    if rc != 0 or not prt.is_file():
        row["error"] = f"vbsp rc={rc}" + ("" if prt.is_file() else ", no .prt (leak?)")
        return row
    log = (work / "vbsp.log").read_text(encoding="utf-8", errors="replace")
    row["warnings"] = [label for needle, label in VBSP_WARNINGS if needle in log]
    header = prt.read_text(errors="replace").split("\n", 3)
    row["clusters"], row["portals"] = int(header[1]), int(header[2])

    rc, row["vvis_s"] = run([opts.bin / "vvis_optix.exe", "-game", opts.game] + opts.vis
                            + [work / name], work / "vvis.log")
    if rc != 0:
        row["error"] = f"vvis rc={rc}"
        return row
    return measure_into(row, work / (name + ".bsp"))


def measure_into(row, bsp):
    """Adds the pvs_cost.py numbers to the row; one unreadable BSP is that row's error."""
    try:
        row.update(pvs_cost.measure(str(bsp)))
    except (OSError, ValueError, IndexError, struct.error) as e:
        row["error"] = f"pvs_cost: {e}"
    return row


COLUMNS = (("clusters", "clusters", "d"), ("portals", "portals", "d"), ("vvis_s", "vvis s", ".0f"),
           ("triangles", "map tris", "d"), ("floor_seen_pct", "floor seen %", ".2f"),
           ("floor_expected_triangles", "floor tris", ".1f"),
           ("floor_expected_props", "floor props", ".1f"),
           ("expected_triangles", "volume tris", ".1f"))
PERCENT = ("floor_seen_pct", "floor_expected_triangles", "floor_expected_props", "expected_triangles")


def print_table(name, rows):
    print(f"\n{name}")
    width = max(len(label) for label, _ in rows)
    print("  " + "variant".ljust(width) + "".join(title.rjust(19 if key in PERCENT else 12)
                                                  for key, title, _ in COLUMNS))
    ref = rows[0][1]
    for label, row in rows:
        if "error" in row:
            print("  " + label.ljust(width) + "  FAILED: " + row["error"])
            continue
        cells = []
        for key, _title, fmt in COLUMNS:
            if row.get(key) is None:  # a result an older version of the tools wrote
                cells.append("-".rjust(19 if key in PERCENT else 12))
                continue
            cell = format(row[key], fmt)
            if key in PERCENT:
                base = ref.get(key)
                cell += f" ({100.0 * (row[key] / base - 1):+5.1f}%)" if base else " " * 9
                cells.append(cell.rjust(19))
            else:
                cells.append(cell.rjust(12))
        notes = "".join(f"  [{w}]" for w in row.get("warnings", ()))
        print("  " + label.ljust(width) + "".join(cells) + notes)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("maps", nargs="+", type=Path)
    ap.add_argument("--game", required=True)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--variant", action="append", required=True)
    ap.add_argument("--vis", default="")
    ap.add_argument("--bin", type=Path, default=BIN)
    ap.add_argument("--strip-visclusters", action="store_true")
    ap.add_argument("--reuse", action="store_true")
    ap.add_argument("--remeasure", action="store_true")
    ap.add_argument("--json", type=Path)
    opts = ap.parse_args()
    opts.vis = shlex.split(opts.vis)
    # the tools run from their own folder, so every path handed to them is absolute
    opts.out, opts.bin = opts.out.resolve(), opts.bin.resolve()
    opts.game = str(Path(opts.game).resolve())
    for exe in ("vbsp_lmo.exe", "vvis_optix.exe"):
        if not (opts.bin / exe).is_file():
            sys.exit(f"no {exe} in {opts.bin}")

    variants = []
    for spec in opts.variant:
        label, sep, args = spec.partition("=")
        if not sep or not label:
            sys.exit(f"--variant wants label=<vbsp arguments>, got {spec!r}")
        variants.append((label, shlex.split(args)))

    failed, everything = False, []
    for vmf in opts.maps:
        if not vmf.is_file():
            sys.exit(f"no such map: {vmf}")
        rows = []
        for label, vbsp_args in variants:
            work = opts.out / vmf.stem / label
            result = work / "result.json"
            row = None
            if (opts.reuse or opts.remeasure) and result.is_file():
                row = json.loads(result.read_text())
                wanted = settings(vbsp_args, opts)
                if "vbsp_args" not in row:
                    # written before the settings were stored: cannot be checked
                    row["warnings"] = sorted(set(row.get("warnings", [])) | {"settings not recorded"})
                elif any(row.get(k) != v for k, v in wanted.items()):
                    print(f"[{vmf.stem} / {label}] stored result used other settings; compiling again",
                          flush=True)
                    row = None
            if row is None:
                row = compile_variant(vmf.resolve(), work, vbsp_args, opts)
                result.write_text(json.dumps(row, indent=1))
            elif opts.remeasure and "error" not in row:
                measure_into(row, work / (vmf.stem + ".bsp"))
                result.write_text(json.dumps(row, indent=1))
            row["variant"] = label
            failed |= "error" in row
            rows.append((label, row))
            everything.append(row)
            print(f"[{vmf.stem} / {label}] " + (row.get("error") or
                  f"{row['clusters']} clusters, vvis {row['vvis_s']:.0f} s, "
                  f"{row.get('floor_seen_pct', '?')} % of the floor seen"), flush=True)
        print_table(vmf.stem, rows)
    if opts.json:
        opts.json.write_text(json.dumps(everything, indent=1))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
