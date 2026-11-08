"""How much a scale change on some sides moves the BOUNCED light on others.

VRAD sizes bounce patches in luxels (maxchop/minchop, default 4), so a side
made coarser also emits its bounce light from coarser patches. lmoptimizer
judges each side by its own baseline lightmap only and never re-lights, so
that effect on other sides is invisible to it. This measures it.

Inputs: the same two maps (BASE = original scales, NEW = lmoptimizer output),
each compiled twice with lit_compile.py, at --bounce 2 and --bounce 0:

  python bounce_attribution.py BASE_B2 BASE_B0 NEW_B2 [NEW_B0]
         [--noise BASE_B2_AGAIN] [--radius 512] [--json out]

Sides whose scale did not change ("control") have the same luxel lattice in
both maps, so their direct light should match exactly (the 0-bounce pair
checks that) and any difference at 2 bounces comes from other sides' bounce.
Each control side's change is the p99 over its own-face luxels of
|NEW - BASE| in display space (out of 255, as lightmap_verify).

Exposure of a control side: the sum, over sides whose scale changed (--emitters:
coarser, finer or changed) within --radius,
of emitted power (mean linear baseline luminance x area x texture
reflectivity) / d^2, d being the gap between the two sides' bounding boxes,
clamped to at least 16 units; only pairs that face each other count. Its
rank correlation with the change says whether those sides explain it.

--noise: a second BASE 2-bounce compile; its difference from BASE_B2 on the
same sides is the run-to-run floor (RTX ray order, thread timing).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

from lightmap_verify import _load, _side_errors, _face_grid


def _lin(disp):
    return 255.0 * np.power(np.maximum(disp, 0.0) / 255.0, 2.2)


def _poly_area_centroid(poly):
    c = poly.mean(axis=0)
    area = 0.0
    for k in range(len(poly)):
        area += 0.5 * np.linalg.norm(np.cross(poly[k] - c, poly[(k + 1) % len(poly)] - c))
    return area, c


def _side_geometry(m, reflect):
    """{side: (area, centroid, facing normal, mins, maxs, mean linear, reflectivity)}."""
    from lightmap_verify import _facing
    out = {}
    for sid, fis in m["by_side"].items():
        areas, cents, pts, lum, lw = [], [], [], 0.0, 0.0
        for fi in fis:
            a, c = _poly_area_centroid(m["polys"][fi])
            areas.append(a)
            cents.append(c)
            pts.append(m["polys"][fi])
            g = _lin(_face_grid(m, fi))
            lum += float(g.mean()) * a
            lw += a
        A = float(sum(areas))
        if A <= 0:
            continue
        cen = np.average(np.array(cents), axis=0, weights=np.array(areas))
        allp = np.concatenate(pts)
        f0 = fis[0]
        out[sid] = (A, cen, _facing(m, f0), allp.min(axis=0), allp.max(axis=0),
                    lum / lw, reflect(m["faces"][f0].texinfo))
    return out


def _reflectivity_fn(path: Path):
    from bsp_reader import BSPReader
    r = BSPReader(path)
    r.read()
    tis = r.read_texinfos()
    tds = r.read_texdata()

    def f(ti):
        try:
            td = tis[ti].texdata
            if not 0 <= td < len(tds):  # a negative index would wrap
                return 0.5
            rv = tds[td].reflectivity
            return 0.2126 * rv[0] + 0.7152 * rv[1] + 0.0722 * rv[2]
        except (IndexError, AttributeError):
            return 0.5
    return f


def _p99s(a, b):
    """{side: (scale a, scale b, p99 |b - a|, mean signed b - a)}."""
    skipped = {"missing_in_new": 0, "no_luxels": 0}
    return {sid: (sa, sb, float(np.percentile(np.abs(e), 99)), float(np.mean(e)))
            for sid, (sa, sb, e) in _side_errors(a, b, skipped, signed=True).items()}


def _avg_ranks(v):
    """Ranks with ties sharing their mean rank (many changes are exactly 0)."""
    v = np.asarray(v, dtype=float)
    order = np.argsort(v, kind="stable")
    ranks = np.empty(len(v))
    sv = v[order]
    i = 0
    while i < len(v):
        j = i
        while j + 1 < len(v) and sv[j + 1] == sv[i]:
            j += 1
        ranks[order[i:j + 1]] = (i + j) / 2.0
        i = j + 1
    return ranks


def _spearman(x, y):
    if len(x) < 3:
        return None
    rx, ry = _avg_ranks(x), _avg_ranks(y)
    with np.errstate(invalid="ignore", divide="ignore"):
        r = float(np.corrcoef(rx, ry)[0, 1])
    return r if np.isfinite(r) else None  # constant input: undefined


def _round(x):
    return None if x is None else round(x, 3)


def _dist(v):
    v = np.asarray(v, dtype=float)
    if v.size == 0:
        return None
    return {"sides": int(v.size), "median": round(float(np.median(v)), 3),
            "p90": round(float(np.percentile(v, 90)), 3),
            "p99": round(float(np.percentile(v, 99)), 3), "max": round(float(v.max()), 2),
            "over_2": int((v > 2).sum()), "over_4": int((v > 4).sum()),
            "over_8": int((v > 8).sum())}


def measure(base2, base0, new2, new0=None, noise=None, radius=512.0,
            emitter_set="changed") -> dict:
    B2, B0, N2 = (_load(p) for p in (base2, base0, new2))
    total = _p99s(B2, N2)    # all light, NEW vs BASE
    # Direct only; without NEW_B0 the direct check is skipped (reported 0).
    direct = _p99s(B0, _load(new0)) if new0 else {s: (0, 0, 0.0, 0.0) for s in total}
    noise_p = _p99s(B2, _load(noise)) if noise else {}

    control = [s for s, v in total.items() if v[0] == v[1] and s in direct]
    coarser = [s for s, v in total.items() if v[1] > v[0]]
    finer = [s for s, v in total.items() if v[1] < v[0]]

    geo = _side_geometry(B2, _reflectivity_fn(base2))
    # Bounce share of each side in BASE: mean (b2 - b0) / b2, linear.
    geo0 = _side_geometry(B0, lambda ti: 0.0)
    bshare = {s: max(0.0, 1.0 - geo0[s][5] / g[5]) if g[5] > 0 and s in geo0 else 0.0
              for s, g in geo.items()}

    emitters = []
    pool = {"coarser": coarser, "finer": finer, "changed": coarser + finer}[emitter_set]
    for s in pool:
        if s not in geo:
            continue
        A, c, n, mn, mx, lum, refl = geo[s]
        emitters.append((s, A * lum * refl, c, n, mn, mx, total[s][0], total[s][1]))

    if emitters:
        E_P = np.array([e[1] for e in emitters])
        E_c = np.array([e[2] for e in emitters])
        E_n = np.array([e[3] for e in emitters])
        E_mn = np.array([e[4] for e in emitters])
        E_mx = np.array([e[5] for e in emitters])
    rows = []
    for s in control:
        if s not in geo:
            continue
        A, c, n, mn, mx, lum, refl = geo[s]
        exp_sum, top = 0.0, None
        if emitters:
            gap = np.maximum(0.0, np.maximum(mn - E_mx, E_mn - mx))
            d = np.linalg.norm(gap, axis=1)
            v = E_c - c
            # Each must be in front of the other to exchange light.
            ok = (d <= radius) & (v @ n > 0) & (np.einsum("ij,ij->i", -v, E_n) > 0)
            e = np.where(ok, E_P / np.maximum(d, 16.0) ** 2, 0.0)
            exp_sum = float(e.sum())
            k = int(np.argmax(e))
            if e[k] > 0:
                top = (emitters[k][0], float(e[k]), emitters[k][6], emitters[k][7],
                       round(float(d[k]), 1))
        rows.append(dict(side=s, scale=total[s][0], total_p99=total[s][2],
                         mean_signed=total[s][3],
                         direct_p99=direct[s][2] if new0 else None,
                         noise_p99=noise_p[s][2] if s in noise_p else None,
                         bounce_share=round(bshare.get(s, 0.0), 3), exposure=exp_sum,
                         top_emitter=None if top is None else
                         {"side": top[0], "scale": f"{top[2]}->{top[3]}", "dist": top[4]}))

    tot = np.array([r["total_p99"] for r in rows])
    dirp = np.array([r["direct_p99"] or 0.0 for r in rows])
    exp_ = np.array([r["exposure"] for r in rows])
    exposed = exp_ > 0
    out = {
        "sides": {"control": len(control), "coarser": len(coarser), "finer": len(finer)},
        "control_total_change": _dist(tot),
        "control_direct_change": _dist(dirp) if new0 else None,
        # Of the control sides moved past 4: how many got brighter vs darker.
        "control_over_4_brighter_darker": [
            int(sum(1 for r in rows if r["total_p99"] > 4 and r["mean_signed"] > 0)),
            int(sum(1 for r in rows if r["total_p99"] > 4 and r["mean_signed"] < 0))],
        "control_noise": _dist([r["noise_p99"] for r in rows
                                if r["noise_p99"] is not None]) if noise else None,
        "control_exposed": _dist(tot[exposed]),
        "control_unexposed": _dist(tot[~exposed]),
        "spearman_exposure_vs_change": _round(_spearman(exp_[exposed], tot[exposed])),
    }
    if exposed.sum() >= 8:
        q = np.quantile(exp_[exposed], [0.25, 0.5, 0.75])
        bins = np.digitize(exp_[exposed], q)
        out["change_by_exposure_quartile"] = [
            round(float(np.median(tot[exposed][bins == i])), 3) for i in range(4)]
    worst = sorted(rows, key=lambda r: -(r["total_p99"] - (r["direct_p99"] or 0.0)))[:15]
    out["worst_control"] = [{k: (round(v, 3) if isinstance(v, float) else v)
                             for k, v in r.items()} for r in worst]
    out["per_side"] = rows
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    for n in ("base_b2", "base_b0", "new_b2"):
        ap.add_argument(n, type=Path)
    ap.add_argument("new_b0", type=Path, nargs="?",
                    help="omit to skip the direct-light check")
    ap.add_argument("--noise", type=Path, help="a second BASE 2-bounce compile")
    ap.add_argument("--radius", type=float, default=512.0)
    ap.add_argument("--emitters", choices=("coarser", "finer", "changed"), default="changed",
                    help="which scale-changed sides count toward exposure")
    ap.add_argument("--json", type=Path)
    a = ap.parse_args(argv)
    rep = measure(a.base_b2, a.base_b0, a.new_b2, a.new_b0, a.noise, a.radius,
                  a.emitters)
    print(json.dumps({k: v for k, v in rep.items() if k != "per_side"}, indent=2,
                     default=float))
    if a.json:
        a.json.write_text(json.dumps(rep, indent=2, default=float))
    return 0 if rep["sides"]["control"] else 1


if __name__ == "__main__":
    sys.exit(main())
