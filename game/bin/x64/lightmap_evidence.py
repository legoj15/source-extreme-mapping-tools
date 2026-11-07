"""Where a baseline lightmap is reproduced at a coarser lightmapscale.

Shared by carve_ceiling.py (the step 0 measurement) and the solvers (the
opt-in per-side evidence scale, issue #52 step 2).

A luxel is reproduced at factor k when the face's lightmap, sampled every
k-th luxel and bilinearly reconstructed, is within a tolerance of it in display
(gamma 2.2) space. That accepts flat regions and smooth gradients alike, far
more than the "exactly flat" test the uniform promotion uses.
"""
from __future__ import annotations

import math
from typing import Dict, Iterable

import numpy as np

FACTORS = (2, 4, 8, 16, 32)


def display(lum: np.ndarray) -> np.ndarray:
    # No upper clamp: overbright luxels (linear > 255, ~17% on trainstation)
    # still differ on screen, and clamping them made bright regions look flat.
    return 255.0 * np.power(np.maximum(lum / 255.0, 0.0), 1.0 / 2.2)


def recon_error(grid: np.ndarray, k: int) -> np.ndarray:
    """Per-luxel |grid - bilinear(grid sampled every k-th luxel)|."""
    h, w = grid.shape
    ys = sorted(set(range(0, h, k)) | {h - 1})
    xs = sorted(set(range(0, w, k)) | {w - 1})
    coarse = grid[np.ix_(ys, xs)]
    rows = np.stack([np.interp(np.arange(h), ys, coarse[:, j])
                     for j in range(len(xs))], axis=1)
    recon = np.stack([np.interp(np.arange(w), xs, rows[i, :])
                      for i in range(h)], axis=0)
    return np.abs(recon - grid)


def coarsest_reproducible_scale(ld, tolerance: float,
                                factors: Iterable[int] = FACTORS) -> int:
    """The coarsest scale at which the WHOLE side's baseline lightmap holds.

    ld is a bsp_reader.FaceLightmapData. The baseline scale b is the finest
    of its sub-faces' scales (each sub-face is tested at k on its own luxel
    grid, so only b*k is validated for all of them); the result is b*k for
    the largest k in factors
    for which every luxel of every sub-face reconstructs within tolerance
    (display units of 255). The largest passing k, not the first run of
    passing ones: the fractions are not monotone in k, and the question is
    only whether k works. 0 means no evidence: nothing passes, the side is
    never-visible, has no sub-faces, or any sub-face lacks its luxels or
    scale (an unlit baseline is unknown, not smooth).
    """
    if ld.is_never_visible or not ld.sub_faces:
        return 0
    base = None
    grids = []
    for sf in ld.sub_faces:
        vecs = sf.lightmap_vecs
        if not vecs or not vecs[0] or len(vecs[0]) < 3:
            return 0
        v = vecs[0]
        norm = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
        w, h = sf.lightmap_size[0] + 1, sf.lightmap_size[1] + 1
        if norm <= 0 or not sf.luminances or len(sf.luminances) != w * h:
            return 0
        sf_scale = round(1.0 / norm)
        base = sf_scale if base is None else min(base, sf_scale)
        grids.append(display(np.asarray(sf.luminances, dtype=np.float64)
                              .reshape(h, w)))
    if not base or base <= 0:
        return 0
    best = 0
    for k in factors:
        if k > best and all(float(recon_error(g, k).max()) <= tolerance
                            for g in grids):
            best = k
    return base * best


def evidence_scales(face_data: Dict[int, object], tolerance: float) -> Dict[int, int]:
    """{side_id: coarsest reproducible scale} for the sides that have one."""
    out = {}
    for sid, ld in face_data.items():
        scale = coarsest_reproducible_scale(ld, tolerance)
        if scale > 0:
            out[sid] = scale
    return out
