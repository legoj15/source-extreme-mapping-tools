"""Upper bound on what any lightmap carving could save (issue #52, step 0).

Carving splits a VMF side so that the part whose lighting a coarser
lightmapscale reproduces can take that scale while the rest stays at the
scale the solver gave the side. It pays only on a *mixed* side: part of it
reproducible at a scale coarser than its optimized one, part not. A side
wholly reproducible at a coarser scale is the plain solver's business (it
can raise the whole side), so it is reported apart and not counted.

Evidence: a lit baseline BSP (the one lmoptimizer compiles and caches;
nearly all sides at one baseline scale, b is taken per side group). A luxel is reproducible at scale b*k when
the face's lightmap, sampled every k-th luxel and bilinearly reconstructed,
is within a tolerance of it in display (gamma 2.2) space, from the aligned
and the half-step-shifted grid, and faces under MIN_SAMPLED_LUXELS across
carry no evidence: the smooth-side predictor's rule (--aligned-only gives
the step 0 rule). That accepts flat
regions and smooth gradients alike, far more than the carver's "exactly
flat" test -- what a ceiling wants.

Cost: the optimized VMF compiled by vbsp_lmo -emitsideids. Per side group
(sides merged into one BSP face are one group) with optimized scale s,
n BSP faces and V face vertices, for every k with b*k > s and coarse
fraction 0 < f_k < 1:

    saving_k = V * f_k * (1 - max(1/(n*f_k), (s / (b*k))**2))

i.e. the coarse part's n*f_k sub-faces shrink by the square of the scale
ratio but never below one face. The group's ceiling is the best k.

A side the solver promoted finer than the baseline (s < b) is measured from
b: the evidence says nothing below b, and undoing a promotion is the solver
taking back spare budget, not a carving saving. Its V and n are scaled to
what they would be at b, and the removed share is reported as
promotion_face_verts. The new
cuts' edge and T-junction vertices, and the faces they add, are ignored:
this is a ceiling, not a forecast. Faces are reported the same way since
leaffaces, not vertices, can be the binding limit.

Usage:
  python carve_ceiling.py EVIDENCE.bsp --cost OPTIMIZED.bsp [--json out.json]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

from bsp_reader import BSPReader, bsp_lighting_problem, LUMP_LIGHTING_HDR
from lightmap_evidence import FACTORS, MIN_SAMPLED_LUXELS, display, recon_error

LUMP_LEAFFACES = 16  # bsp_reader does not export it
TOLERANCES = (1.0, 1.5, 2.0, 4.0)  # display-space units of 255
# Region granularity: a block of luxels counts as coarse only when every luxel
# in it passes. 1 = per luxel (loosest), 8 = 8x8 luxel tiles, 0 = the whole
# BSP face (VBSP's lightmap-size chop: the current carver's granularity).
TILES = (1, 8, 0)


def _passing_luxels(mask: np.ndarray, tile: int) -> int:
    if tile == 1:
        return int(np.count_nonzero(mask))
    if tile == 0:
        return mask.size if mask.all() else 0
    h, w = mask.shape
    n = 0
    for y in range(0, h, tile):
        for x in range(0, w, tile):
            blk = mask[y:y + tile, x:x + tile]
            if blk.all():
                n += blk.size
    return n


def _side_lists(r: BSPReader, n_faces: int):
    sides = r.read_face_side_ids()
    if sides is not None:
        return [s if s else [] for s in sides]
    print(f"WARNING: {r.filepath.name} has no side-ID lumps (compile with "
          f"vbsp_lmo -emitsideids); falling back to 16-bit hammerfaceids, "
          f"merged faces map to one side only", file=sys.stderr)
    ids = r.read_face_ids()
    return [[ids[i]] if i < len(ids) and ids[i] > 0 else [] for i in range(n_faces)]


def _face_scales(r: BSPReader, faces):
    tis = r.read_texinfos()
    out = []
    for f in faces:
        v = tis[f.texinfo].lightmap_vecs[0] if 0 <= f.texinfo < len(tis) else (0, 0, 0)
        n = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
        out.append(round(1.0 / n) if n > 0 else 0)
    return out


def _face_vertex_ids(r: BSPReader, faces):
    edges = r.read_edges()
    surf = r.read_surfedges()
    return [[edges[se][0] if se >= 0 else edges[-se][1]
             for se in surf[f.firstedge:f.firstedge + f.numedges]] for f in faces]


def measure(evidence: Path, cost: Path, aligned_only: bool = False) -> dict:
    problem = bsp_lighting_problem(evidence)
    if problem:
        raise SystemExit(f"{evidence}: {problem}")

    ev = BSPReader(evidence)
    ev.read()
    use_hdr = ev._lumps[LUMP_LIGHTING_HDR].filelen > 0
    ev_faces = ev.read_faces(use_hdr=use_hdr)
    ev_scale = _face_scales(ev, ev_faces)
    light = ev.read_lighting(use_hdr=use_hdr)
    words = np.frombuffer(light[:len(light) // 4 * 4], dtype=np.uint8).reshape(-1, 4)
    exp = words[:, 3].astype(np.int8).astype(np.float64)
    lin = (0.2126 * words[:, 0] + 0.7152 * words[:, 1]
           + 0.0722 * words[:, 2]) * np.power(2.0, exp)
    disp_all = display(lin)
    ev_sides = _side_lists(ev, len(ev_faces))

    parent: dict = {}

    def find(x):
        while parent.setdefault(x, x) != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    # Only lit faces join sides: an unlit merged face carries no evidence and
    # would pool unrelated lightmaps into one group.
    for f, ids in zip(ev_faces, ev_sides):
        if f.dispinfo < 0 and f.lightofs >= 0:
            for a in ids:
                parent[find(a)] = find(ids[0])

    keys = [(tile, t, k) for tile in TILES for t in TOLERANCES for k in FACTORS]
    coarse = defaultdict(lambda: np.zeros(len(keys)))
    total = defaultdict(int)
    base_scale = {}
    for fi, f in enumerate(ev_faces):
        if f.dispinfo >= 0 or f.lightofs < 0 or not ev_sides[fi]:
            continue
        w, h = f.lightmap_size[0] + 1, f.lightmap_size[1] + 1
        first = f.lightofs // 4
        if first + w * h > len(disp_all):
            continue
        grid = disp_all[first:first + w * h].reshape(h, w)
        g = find(ev_sides[fi][0])
        total[g] += w * h
        base_scale[g] = max(base_scale.get(g, 0), ev_scale[fi])
        if not aligned_only and min(w, h) < MIN_SAMPLED_LUXELS:
            # The predictor's rule: no luxel centre on the face, no evidence.
            errs = {k: np.full(grid.shape, np.inf) for k in FACTORS}
        elif aligned_only:
            errs = {k: recon_error(grid, k) for k in FACTORS}
        else:
            # The predictor's rule: a luxel must hold on the aligned and the
            # half-step-shifted grid (lightmap_evidence.worst_recon_error).
            errs = {k: np.maximum(recon_error(grid, k), recon_error(grid, k, k // 2))
                    for k in FACTORS}
        coarse[g] += np.array([_passing_luxels(errs[k] <= t, tile)
                               for tile, t, k in keys])

    cb = BSPReader(cost)
    cb.read()
    cfaces = cb.read_faces(use_hdr=False)
    c_scale = _face_scales(cb, cfaces)
    csides = _side_lists(cb, len(cfaces))
    cverts = _face_vertex_ids(cb, cfaces)
    group_faces = defaultdict(list)
    unmapped = spanning = 0
    for fi, f in enumerate(cfaces):
        if f.dispinfo >= 0:
            continue
        gs = {find(s) for s in csides[fi] if s in parent}
        if not gs:
            unmapped += 1
            continue
        if len(gs) > 1:
            spanning += 1
        group_faces[min(gs)].append(fi)

    # The model gives a group one optimized scale. lmoptimizer unifies
    # coplanar groups so this holds on every map measured; warn if it breaks,
    # since pooling at the minimum would overstate both columns.
    mixed_scale = sum(1 for fis in group_faces.values()
                      if len({c_scale[i] for i in fis}) > 1)
    if mixed_scale:
        print(f"WARNING: {mixed_scale} side groups have cost faces at different "
              f"scales; they are measured at their finest scale, which "
              f"overstates the ceiling", file=sys.stderr)

    total_verts = cb.read_vertex_count()
    total_face_verts = sum(f.numedges for f in cfaces) or 1
    leaffaces = len(cb._get_lump_data(LUMP_LEAFFACES)) // 2

    results = []
    for tile, t in [(tile, t) for tile in TILES for t in TOLERANCES]:
        n_mixed = n_solver = 0
        fv = est_v = est_f = sol_v = sol_f = promo_v = 0.0
        uv: set = set()
        for g, n_lux in total.items():
            fis = group_faces.get(g)
            if not fis or not base_scale.get(g):
                continue
            s = min(c_scale[i] for i in fis)
            if s <= 0:
                continue
            b = base_scale[g]
            V = sum(cfaces[i].numedges for i in fis)
            n = len(fis)
            if s < b:
                keep = max(1.0 / n, (s / b) ** 2)
                promo_v += V * (1.0 - keep)
                V, n, s = V * keep, max(1.0, n * keep), b
            fracs = [coarse[g][keys.index((tile, t, k))] / n_lux for k in FACTORS]
            # The plain solver could raise the whole side to the coarsest
            # scale that reproduces all of it; carving only gets what is
            # left beyond that.
            s_eff = max([s] + [b * k for k, f in zip(FACTORS, fracs) if f >= 1.0])
            if s_eff > s:
                n_solver += 1
                keep = max(1.0 / n, (s / s_eff) ** 2)
                sol_v += V * (1.0 - keep)
                sol_f += n * (1.0 - keep)
                V_eff = V * keep
                n_eff = max(1.0, n * keep)
            else:
                V_eff, n_eff = V, n
            best_v = best_f = 0.0
            for k, frac in zip(FACTORS, fracs):
                target = b * k
                if target <= s_eff or not 0.0 < frac < 1.0:
                    continue
                keep = max(1.0 / (n_eff * frac), (s_eff / target) ** 2)
                best_v = max(best_v, V_eff * frac * (1.0 - keep))
                best_f = max(best_f, n_eff * frac * (1.0 - keep))
            if best_v > 0:
                n_mixed += 1
                fv += sum(cfaces[i].numedges for i in fis)
                est_v += best_v
                est_f += best_f
                for i in fis:
                    uv.update(cverts[i])
        results.append({
            "tile": tile, "tolerance": t, "mixed_groups": n_mixed,
            "solver_room_groups": n_solver,
            "promotion_face_verts": round(promo_v),
            "solver_vert_saving": round(sol_v), "solver_face_saving": round(sol_f),
            "mixed_face_verts": round(fv), "mixed_unique_verts": len(uv),
            "est_vert_saving": round(est_v), "est_face_saving": round(est_f),
            "est_vert_pct_of_map": round(100.0 * est_v / total_face_verts, 2),
            "solver_vert_pct_of_map": round(100.0 * sol_v / total_face_verts, 2),
            "unique_pct_of_map": round(100.0 * len(uv) / max(total_verts, 1), 2),
            "est_face_pct_of_map": round(100.0 * est_f / max(len(cfaces), 1), 2),
        })
    return {
        "evidence": str(evidence), "cost": str(cost),
        "lit_groups": len(total), "cost_vertexes": total_verts,
        "cost_face_verts": total_face_verts, "cost_faces": len(cfaces),
        "cost_leaffaces": leaffaces, "cost_faces_unmapped": unmapped,
        "cost_faces_spanning_groups": spanning,
        "mixed_scale_groups": mixed_scale,
        "results": results,
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("evidence", type=Path, help="lit baseline BSP")
    ap.add_argument("--cost", type=Path, default=None,
                    help="optimized BSP whose vertices are counted (default: evidence)")
    ap.add_argument("--json", type=Path, default=None)
    ap.add_argument("--aligned-only", action="store_true",
                    help="the step 0 rule: aligned grid only, narrow faces included")
    a = ap.parse_args(argv)
    rep = measure(a.evidence, a.cost or a.evidence, aligned_only=a.aligned_only)
    print(f"evidence {rep['evidence']}\ncost     {rep['cost']}")
    print(f"lit side groups {rep['lit_groups']}; cost BSP: {rep['cost_vertexes']} "
          f"dvertexes, {rep['cost_face_verts']} face verts, {rep['cost_faces']} faces, "
          f"{rep['cost_leaffaces']} leaffaces, {rep['cost_faces_unmapped']} faces unmapped, "
          f"{rep['cost_faces_spanning_groups']} spanning groups; "
          f"promotion share {rep['results'][0]['promotion_face_verts']} face verts "
          f"(measured from baseline, not credited)")
    print(f"{'tile':>4} {'tol':>4} {'solverN':>7} {'solvV%':>6} {'mixedN':>6} {'mixUniqV':>8} "
          f"{'%map':>6} {'carveV':>7} {'carveV%':>7} {'carveF':>6} {'carveF%':>7}")
    for r in rep["results"]:
        print(f"{r['tile'] or 'face':>4} {r['tolerance']:>4} {r['solver_room_groups']:>7} "
              f"{r['solver_vert_pct_of_map']:>6} {r['mixed_groups']:>6} "
              f"{r['mixed_unique_verts']:>8} {r['unique_pct_of_map']:>6} "
              f"{r['est_vert_saving']:>7} {r['est_vert_pct_of_map']:>7} "
              f"{r['est_face_saving']:>6} {r['est_face_pct_of_map']:>7}")
    if a.json:
        a.json.write_text(json.dumps(rep, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
