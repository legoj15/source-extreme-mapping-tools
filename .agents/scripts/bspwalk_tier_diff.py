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

A face's tier is "A", "B" (both hidden), "visible", or "cubemap_only" (#149: visible, but only
env_cubemap viewpoints were found to see it); a run from before #149 has no cubemap_only. A
side's class is the strongest of its faces' (visible, then cubemap_only, then hidden).

For each run: eye spacing and sample count, tier counts, the cubemap_only faces and sides, the
faces hidden here but not hidden in the base run (and the reverse, with their reason here), the
faces whose hidden tier moved between A and B, the faces cubemap_only here but not in the base and
the reverse, stage times, face rays and the peak working set.

Exit code 1 when a run fails. 2 when a check fails:
  --identity   every run's hidden faces, tier A and tier B face for face, are the base's, and its
               cubemap_only faces are a subset of the faces the base did not hide (#149: the class
               only splits the visible set);
  --same       every run's tiers are the base's face for face, cubemap_only included (the cache,
               thread and batch checks of run_bspwalk_cache_tests.py).
"""
import argparse
import json
import os
import subprocess
import sys

HIDDEN = ("A", "B")


def load(path):
    with open(path, "r", encoding="utf-8") as f:
        j = json.load(f)
    return j, {f["face"]: (f["tier"], f["reason"]) for f in j.get("faces", [])}


def side_classes(j):
    """{side id: "visible" | "cubemap_only" | "hidden"} from the faces' side ids."""
    out = {}
    rank = {"visible": 2, "cubemap_only": 1}
    for f in j.get("faces", []):
        cls = f["tier"] if f["tier"] in rank else "hidden"
        for s in f.get("sideids", []):
            old = out.get(s)
            if old is None or rank.get(cls, 0) > rank.get(old, 0):
                out[s] = cls
    return out


def describe(label, j, tiers, base, show):
    """Prints one run against the base; returns (hidden sets differ, identity broken, tiers differ)."""
    e, v, t = j.get("eyes", {}), j.get("vis", {}), j.get("timings_s", {})
    hid_b = {f for f, x in base.items() if x[0] in HIDDEN}
    hid = {f for f, x in tiers.items() if x[0] in HIDDEN}
    more, fewer = sorted(hid - hid_b), sorted(hid_b - hid)
    moved = sorted(f for f in hid & hid_b if tiers[f][0] != base[f][0])
    cube = {f for f, x in tiers.items() if x[0] == "cubemap_only"}
    cube_b = {f for f, x in base.items() if x[0] == "cubemap_only"}
    cube_more, cube_fewer = sorted(cube - cube_b), sorted(cube_b - cube)
    cube_hidden_b = sorted(f for f in cube if f in hid_b or f not in base)
    sides = side_classes(j)
    samples = e.get("samples", e.get("samples_generated"))
    print(f"{label}: spacing {e.get('spacing')}, samples {samples}; "
          f"tier A {v.get('tier_a')}, tier B {v.get('tier_b_hidden')}, visible {v.get('visible')} "
          f"(cubemap_only {len(cube)} faces, {sum(c == 'cubemap_only' for c in sides.values())} sides); "
          f"hidden here only {len(more)}, hidden in base only {len(fewer)}, A<->B {len(moved)}")
    print(f"    eyes {t.get('eyes')} s, vis {t.get('vis')} s (fan {v.get('fan_s')}, face {v.get('face_s')}), "
          f"total {t.get('total')} s; fan rays {v.get('fan_rays')}, face rays {v.get('face_rays')}, "
          f"mirror rays {v.get('mirror_rays')} ({v.get('ray_unit')}); peak {j.get('peak_working_set_mb')} MB")
    if more:
        print("    hidden here only:", " ".join(str(f) for f in more[:show]), "..." if len(more) > show else "")
    if fewer:
        print("    hidden in base only:", " ".join(f"{f}({tiers[f][1]})" for f in fewer[:show]),
              "..." if len(fewer) > show else "")
    if moved:
        print("    tier A<->B:", " ".join(f"{f}({base[f][0]}->{tiers[f][0]})" for f in moved[:show]),
              "..." if len(moved) > show else "")
    if cube_more:
        print("    cubemap_only here only:", " ".join(str(f) for f in cube_more[:show]),
              "..." if len(cube_more) > show else "")
    if cube_fewer:
        print("    cubemap_only in base only:", " ".join(f"{f}({tiers.get(f, ('?',))[0]})" for f in cube_fewer[:show]),
              "..." if len(cube_fewer) > show else "")
    if cube_hidden_b:
        print("    cubemap_only here, hidden (or absent) in base:", " ".join(str(f) for f in cube_hidden_b[:show]),
              "..." if len(cube_hidden_b) > show else "")
    hid_differ = bool(more or fewer)
    identity_broken = bool(more or fewer or moved or cube_hidden_b or set(tiers) != set(base))
    tiers_differ = any(tiers.get(f, (None,))[0] != x[0] for f, x in base.items()) or len(tiers) != len(base)
    return hid_differ, identity_broken, tiers_differ


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
    ap.add_argument("--same", action="store_true", help="exit 2 unless every run's tiers match the base face for face")
    ap.add_argument("--identity", action="store_true",
                    help="exit 2 unless every run's hidden faces (A and B) are the base's and its cubemap_only faces "
                         "were not hidden in the base (#149)")
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
    identity_broken = tiers_differ = False
    for label, path in files:
        j, tiers = load(path)
        _, i, t = describe(label, j, tiers, base, a.show)
        identity_broken |= i
        tiers_differ |= t
    if a.identity:
        print(f"identity (#149): {'BROKEN' if identity_broken else 'holds'}: hidden faces and their tiers "
              f"{'differ' if identity_broken else 'match'}; cubemap_only within the base's unhidden faces")
    if a.same:
        print(f"same: tiers {'differ' if tiers_differ else 'match face for face'}")
    return 2 if (a.identity and identity_broken) or (a.same and tiers_differ) else 0


if __name__ == "__main__":
    sys.exit(main())
