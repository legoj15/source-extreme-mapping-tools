#!/usr/bin/env python3
"""Compare bspwalk face tiers between runs.

Two ways to use it:

  python -I bspwalk_tier_diff.py base.json other.json [more.json ...]
      Diff existing bspwalk -out files against the first one.

  python -I bspwalk_tier_diff.py --bsp map.bsp --game <dir> --out <dir> \
      --run base=<bin dir> --run new=<bin dir>,-eyespacing,4 [--run ...]
      Run each build (bspwalk.exe in <bin dir>, extra arguments after commas)
      on the same BSP with -reachslim, then diff every run against the first.
      A label may carry environment settings: --run "x=<bin dir>,ENV:NAME=1".

For each run: eye spacing and sample count, tier counts, the faces hidden here
but visible in the base run (and the reverse, with their reason here), stage
times and the peak working set. Exit code 1 when a run fails, 2 when any run's
tiers differ from the base (use --same to require identical tiers).
"""
import argparse
import json
import os
import subprocess
import sys


def load(path):
    with open(path, "r", encoding="utf-8") as f:
        j = json.load(f)
    return j, {f["face"]: (f["tier"], f["reason"]) for f in j.get("faces", [])}


def describe(label, j, tiers, base, show):
    e, v, t = j.get("eyes", {}), j.get("vis", {}), j.get("timings_s", {})
    hid_b = {f for f, x in base.items() if x[0] != "visible"}
    hid = {f for f, x in tiers.items() if x[0] != "visible"}
    more, fewer = sorted(hid - hid_b), sorted(hid_b - hid)
    samples = e.get("samples", e.get("samples_generated"))
    print(f"{label}: spacing {e.get('spacing')} (asked {e.get('spacing_asked')}), samples {samples}; "
          f"tier A {v.get('tier_a')}, tier B {v.get('tier_b_hidden')}, visible {v.get('visible')}; "
          f"hidden here only {len(more)}, hidden in base only {len(fewer)}")
    print(f"    eyes {t.get('eyes')} s, vis {t.get('vis')} s (fan {v.get('fan_s')}, face {v.get('face_s')}), "
          f"total {t.get('total')} s; face rays {v.get('face_rays')}; peak {j.get('peak_working_set_mb')} MB")
    if more:
        print("    hidden here only:", " ".join(str(f) for f in more[:show]), "..." if len(more) > show else "")
    if fewer:
        print("    hidden in base only:", " ".join(f"{f}({tiers[f][1]})" for f in fewer[:show]),
              "..." if len(fewer) > show else "")
    return bool(more or fewer)


def run_one(spec, bsp, game, out):
    label, _, rest = spec.partition("=")
    parts = rest.split(",")
    bindir, extra, env = parts[0], [], dict(os.environ)
    for p in parts[1:]:
        if p.startswith("ENV:"):
            k, _, val = p[4:].partition("=")
            env[k] = val
        elif p:
            extra.append(p)
    js = os.path.join(out, label + ".json")
    cmd = [os.path.join(bindir, "bspwalk.exe"), bsp, "-game", game, "-reachslim", "-out", js] + extra
    with open(os.path.join(out, label + ".log"), "w", encoding="utf-8", errors="replace") as log:
        rc = subprocess.run(cmd, cwd=bindir, env=env, stdout=log, stderr=subprocess.STDOUT).returncode
    if rc != 0 or not os.path.isfile(js):
        print(f"{label}: bspwalk exit {rc}, see {os.path.join(out, label + '.log')}")
        return None
    return js


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("json", nargs="*", help="bspwalk -out files; the first is the base")
    ap.add_argument("--bsp")
    ap.add_argument("--game")
    ap.add_argument("--out", help="directory for the runs' JSON and logs")
    ap.add_argument("--run", action="append", default=[], metavar="label=<bin dir>[,arg...]")
    ap.add_argument("--show", type=int, default=24, help="faces listed per line (default 24)")
    ap.add_argument("--same", action="store_true", help="exit 2 unless every run's tiers match the base")
    a = ap.parse_args()

    files = [(os.path.basename(p), p) for p in a.json]
    if a.run:
        if not (a.bsp and a.game and a.out):
            ap.error("--run needs --bsp, --game and --out")
        os.makedirs(a.out, exist_ok=True)
        for spec in a.run:
            js = run_one(spec, a.bsp, a.game, a.out)
            if js is None:
                return 1
            files.append((spec.partition("=")[0], js))
    if len(files) < 2:
        ap.error("need at least two runs to compare")

    _, base = load(files[0][1])
    differ = False
    for label, path in files:
        j, tiers = load(path)
        differ |= describe(label, j, tiers, base, a.show)
    return 2 if differ and a.same else 0


if __name__ == "__main__":
    sys.exit(main())
