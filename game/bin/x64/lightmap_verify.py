"""Compare a recompiled map's lightmaps against the baseline, side by side
(issue #52, step 4 check for evidence-raised scales).

For every luxel of a side's baseline lightmap: recover its world position
from the baseline face's lightmap vectors, find the side's face in the new
BSP whose lightmap covers that point, sample it bilinearly, and compare the
two in display space (lightmap_evidence.display). Scales may differ between
the two BSPs; geometry and side IDs must match (both compiled with
vbsp_lmo -emitsideids from the same map).

Reports the error distribution per side class: sides whose scale went up by
more than --raised-ratio (the evidence-raised sides), and the rest (control:
run-to-run VRAD noise plus whatever the global scale change did).

Usage:
  python lightmap_verify.py BASELINE.bsp NEW.bsp [--raised-ratio 1.5] [--json out]
"""
from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

from bsp_reader import BSPReader, LUMP_LIGHTING_HDR
from carve_ceiling import _side_lists, _face_scales
from lightmap_evidence import display


def _load(path: Path):
    r = BSPReader(path)
    r.read()
    hdr = r._lumps[LUMP_LIGHTING_HDR].filelen > 0
    faces = r.read_faces(use_hdr=hdr)
    light = r.read_lighting(use_hdr=hdr)
    w = np.frombuffer(light[:len(light) // 4 * 4], dtype=np.uint8).reshape(-1, 4)
    lin = (0.2126 * w[:, 0] + 0.7152 * w[:, 1] + 0.0722 * w[:, 2]) \
        * np.power(2.0, w[:, 3].astype(np.int8).astype(np.float64))
    tis = r.read_texinfos()
    planes = r.read_planes()
    sides = _side_lists(r, len(faces))
    scales = _face_scales(r, faces)
    by_side = defaultdict(list)
    for i, f in enumerate(faces):
        if f.dispinfo >= 0 or f.lightofs < 0 or len(sides[i]) != 1:
            continue  # merged faces carry several sides; skip them
        by_side[sides[i][0]].append(i)
    verts = r.read_vertexes()
    edges = r.read_edges()
    surf = r.read_surfedges()
    polys = {i: np.array(r.get_face_vertices(faces[i], verts, edges, surf))
             for fis in by_side.values() for i in fis}
    return dict(faces=faces, disp=display(lin), tis=tis, planes=planes,
                by_side=by_side, scales=scales, polys=polys)


def _face_grid(m, fi):
    f = m["faces"][fi]
    w, h = f.lightmap_size[0] + 1, f.lightmap_size[1] + 1
    first = f.lightofs // 4
    if first + w * h > len(m["disp"]):
        raise ValueError(f"face {fi}: lightmap runs past the lighting lump")
    return m["disp"][first:first + w * h].reshape(h, w)


def _luxel_world(m, fi):
    """World positions of every luxel of face fi, shape (h, w, 3)."""
    f = m["faces"][fi]
    v = np.array(m["tis"][f.texinfo].lightmap_vecs, dtype=np.float64)
    pl = m["planes"][f.planenum]
    A = np.array([v[0][:3], v[1][:3], pl.normal], dtype=np.float64)
    Ainv = np.linalg.inv(A)
    w, h = f.lightmap_size[0] + 1, f.lightmap_size[1] + 1
    s, t = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
    rhs = np.stack([s + f.lightmap_mins[0] - v[0][3],
                    t + f.lightmap_mins[1] - v[1][3],
                    np.full_like(s, pl.dist)], axis=-1)
    return rhs @ Ainv.T


def _sample(m, fi, P):
    """Bilinear sample of face fi at world points P (N, 3).

    Returns (value, score): score is the signed distance (world units) of the
    point inside the face polygon (negative outside), -inf where the point
    is off the face's lightmap rectangle. The caller keeps the best score, so
    a luxel on a sub-face boundary reads the face it belongs to.
    """
    f = m["faces"][fi]
    v = np.array(m["tis"][f.texinfo].lightmap_vecs, dtype=np.float64)
    u = P @ v[0][:3] + v[0][3] - f.lightmap_mins[0]
    t = P @ v[1][:3] + v[1][3] - f.lightmap_mins[1]
    grid = _face_grid(m, fi)
    h, w = grid.shape
    eps = 1e-3
    inside = (u >= -eps) & (u <= w - 1 + eps) & (t >= -eps) & (t <= h - 1 + eps)
    poly = m["polys"][fi]
    n = np.array(m["planes"][f.planenum].normal, dtype=np.float64)
    score = np.full(len(P), np.inf)
    for k in range(len(poly)):
        e = poly[(k + 1) % len(poly)] - poly[k]
        inward = np.cross(n, e)
        inward /= max(np.linalg.norm(inward), 1e-12)
        d = (P - poly[k]) @ inward
        score = np.minimum(score, d)
    # Winding may be either way round; a point is inside one orientation.
    score2 = np.full(len(P), np.inf)
    for k in range(len(poly)):
        e = poly[(k + 1) % len(poly)] - poly[k]
        inward = -np.cross(n, e)
        inward /= max(np.linalg.norm(inward), 1e-12)
        score2 = np.minimum(score2, (P - poly[k]) @ inward)
    score = np.maximum(score, score2)
    u = np.clip(u, 0, w - 1)
    t = np.clip(t, 0, h - 1)
    x0 = np.minimum(np.floor(u).astype(int), max(w - 2, 0))
    y0 = np.minimum(np.floor(t).astype(int), max(h - 2, 0))
    x1 = np.minimum(x0 + 1, w - 1)
    y1 = np.minimum(y0 + 1, h - 1)
    fx, fy = u - x0, t - y0
    val = (grid[y0, x0] * (1 - fx) * (1 - fy) + grid[y0, x1] * fx * (1 - fy)
           + grid[y1, x0] * (1 - fx) * fy + grid[y1, x1] * fx * fy)
    return val, np.where(inside, score, -np.inf)


def _facing(m, fi):
    f = m["faces"][fi]
    n = np.array(m["planes"][f.planenum].normal, dtype=np.float64)
    return -n if f.side else n


def compare(base: Path, new: Path, raised_ratio: float = 1.5) -> dict:
    a, b = _load(base), _load(new)
    classes = {"raised": [], "control": []}
    per_side = {}
    skipped = {"missing_in_new": 0, "no_luxels": 0}
    for sid, afis in a["by_side"].items():
        bfis = b["by_side"].get(sid)
        if not bfis:
            skipped["missing_in_new"] += 1
            continue
        sa = max(a["scales"][i] for i in afis)
        sb = max(b["scales"][i] for i in bfis)
        errs = []
        for fi in afis:
            P = _luxel_world(a, fi).reshape(-1, 3)
            ref = _face_grid(a, fi).reshape(-1)
            got = np.full(ref.shape, np.nan)
            best = np.full(ref.shape, -np.inf)
            na = _facing(a, fi)
            for bj in bfis:
                # A two-sided surface (water) has a front and a back face
                # under one side ID; only the same facing is comparable.
                if _facing(b, bj) @ na < 0.99:
                    continue
                val, sc = _sample(b, bj, P)
                take = sc > best
                got[take] = val[take]
                best[take] = sc[take]
            # Only luxels inside their own face: border luxels are VRAD's
            # extrapolation and a neighbour face legitimately disagrees.
            _, own = _sample(a, fi, P)
            ok = np.isfinite(best) & (own >= 0.0)
            errs.append(np.abs(got[ok] - ref[ok]))
        e = np.concatenate(errs) if errs else np.zeros(0)
        if e.size == 0:
            skipped["no_luxels"] += 1
            continue
        cls = "raised" if sb >= sa * raised_ratio else "control"
        classes[cls].append(e)
        per_side[sid] = (cls, sa, sb, float(np.percentile(e, 99)), float(e.max()))
    out = {"compared_sides": len(per_side), "skipped_sides": skipped}
    for cls, lst in classes.items():
        if not lst:
            out[cls] = None
            continue
        e = np.concatenate(lst)
        side_p99 = np.array([v[3] for v in per_side.values() if v[0] == cls])
        out[cls] = {
            "sides": len(lst), "luxels": int(e.size),
            "mean": round(float(e.mean()), 3),
            "p50": round(float(np.percentile(e, 50)), 3),
            "p99": round(float(np.percentile(e, 99)), 3),
            "max": round(float(e.max()), 2),
            "sides_p99_over_4": int((side_p99 > 4).sum()),
            "sides_p99_over_8": int((side_p99 > 8).sum()),
        }
    worst = sorted(((v[3], sid, v) for sid, v in per_side.items() if v[0] == "raised"),
                   reverse=True)[:10]
    out["worst_raised"] = [{"side": sid, "scale": f"{v[1]}->{v[2]}", "p99": round(v[3], 2),
                            "max": round(v[4], 2)} for _, sid, v in worst]
    out["per_side"] = {str(sid): {"class": v[0], "scale": [v[1], v[2]], "p99": round(v[3], 3),
                                  "max": round(v[4], 2)} for sid, v in per_side.items()}
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("baseline", type=Path)
    ap.add_argument("new", type=Path)
    ap.add_argument("--raised-ratio", type=float, default=1.5,
                    help="a side whose scale grew by at least this factor is 'raised'")
    ap.add_argument("--json", type=Path)
    a = ap.parse_args(argv)
    rep = compare(a.baseline, a.new, a.raised_ratio)
    if rep["compared_sides"] == 0:
        print("ERROR: no sides compared (unlit BSP, or not compiled with "
              "-emitsideids?)", file=sys.stderr)
    print(json.dumps({k: v for k, v in rep.items() if k != "per_side"}, indent=2))
    if a.json:
        a.json.write_text(json.dumps(rep, indent=2))
    return 0 if rep["compared_sides"] else 1


if __name__ == "__main__":
    sys.exit(main())
