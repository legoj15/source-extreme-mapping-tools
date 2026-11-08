"""Tests for the opt-in smooth-side evidence scale (issue #52, step 2).

Run:  python game/bin/x64/unit_test/lmoptimizer/test_smooth_evidence.py
  or: pytest game/bin/x64/unit_test/lmoptimizer
Exit code 0 = all pass.

  A. coarsest_reproducible_scale: the coarsest scale at which a side's whole
     baseline lightmap is reproduced within a display-space tolerance.
  B. solve_with_vbsp(evidence_scales=...): the evidence is a floor for a
     side that is neither uniform nor emissive: the minimum of the
     raw evidence over its coplanar group, then capped at its max useful
     scale (1 = no floor). Only a floor above the baseline blocks promotion;
     a lower one only stops a promotion short of scale 1.
  C. lmoptimizer leaves never-visible sides out of the evidence.
  D. solve_with_vbsp(skip_ahead_calls=...): after the prefix search, units
     that do not fit are skipped and cheaper lower-priority ones still join.
  E. solve_with_vbsp(promote_order=...): "value" ranks promotion units by
     priority x area x luxel gain (0 for a side a minimum scale holds in
     place) per estimated cell of added cost, "priority" by the best
     member's priority alone.
"""
import contextlib
import io
import math
import tempfile
from pathlib import Path
from unittest import mock

import os as _os
import sys as _sys
# python -I drops the script's folder from sys.path; _testkit lives beside this file.
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))

from _testkit import check, run_all

import numpy as np

import lightmap_evidence
import vbsp_runner
import vbsp_solver
from bsp_reader import BSPSubFaceInfo, FaceLightmapData
from lightmap_evidence import (display, recon_error, worst_recon_error,
                                coarsest_reproducible_scale)
from vertex_estimator import FaceExtent
from vmf_parser import KVNode, KVPair

SIZE = 17  # luxels per side: lightmap_size (16, 16)


def to_linear(display):
    """Inverse of lightmap_evidence.display: tolerance is measured in display
    space, so fixtures are written there and converted back to the linear
    luminance the BSP reader hands over."""
    return 255.0 * np.power(np.asarray(display, dtype=float) / 255.0, 2.2)


def sub_face(grid, scale=2, index=0):
    """A BSPSubFaceInfo over a (rows x cols) grid of LINEAR luminance."""
    grid = np.asarray(grid, dtype=float)
    h, w = grid.shape
    vecs = ((1.0 / scale, 0.0, 0.0, 0.0), (0.0, 1.0 / scale, 0.0, 0.0))
    return BSPSubFaceInfo(face_index=index, lightmap_mins=(0, 0),
                          lightmap_size=(w - 1, h - 1), lightmap_vecs=vecs,
                          luminances=[float(x) for x in grid.ravel()],
                          is_uniform=bool(np.ptp(grid) == 0.0))


def side(*sub_faces):
    luminances = [x for sf in sub_faces for x in sf.luminances]
    return FaceLightmapData(vmf_side_id=1, material='M',
                            bsp_face_indices=[sf.face_index for sf in sub_faces],
                            luminances=luminances, sub_faces=list(sub_faces))


def display_grid(fn):
    """A SIZE x SIZE grid of linear luminance from fn(x, y) in display space."""
    ys, xs = np.mgrid[0:SIZE, 0:SIZE]
    return to_linear(fn(xs.astype(float), ys.astype(float)))


# ─── A. coarsest_reproducible_scale ──────────────────────────────────────────

def test_constant_side_takes_the_coarsest_factor():
    ld = side(sub_face(np.full((SIZE, SIZE), 120.0)))
    got = coarsest_reproducible_scale(ld, 1.0)
    check("a constant lit side at scale 2 -> 2 * 32", got == 64, f"got {got}")


def test_ramp_is_reproduced_but_a_luminance_ramp_is_not():
    # Bilinear reconstruction from the aligned grid is exact on a ramp in the
    # space the tolerance is measured in. A ramp in LINEAR luminance bends in
    # display space (gamma), so the same reconstruction misses it.
    ys, xs = np.mgrid[0:SIZE, 0:SIZE]
    steep = display(display_grid(lambda x, y: 20.0 + (200.0 - 20.0) * x / (SIZE - 1)))
    check("a display-space ramp: the aligned grid is exact at every factor",
          all(float(recon_error(steep, k).max()) < 1e-6 for k in (2, 4, 8, 16, 32)))
    lum_ramp = display(20.0 + (200.0 - 20.0) * xs / (SIZE - 1))
    check("a linear-luminance ramp: the aligned grid misses it at k=32",
          float(recon_error(lum_ramp, 32).max()) > 1.0)
    # The half-step grid puts a coarse luxel off the face that reads the edge
    # luxel, flattening the slope there by slope * k / 4: a 3.2-unit ramp
    # over 16 luxels passes up to k=16 (0.8) and fails k=32 (1.6).
    gentle = display_grid(lambda x, y: 100.0 + 3.2 * x / (SIZE - 1))
    got = coarsest_reproducible_scale(side(sub_face(gentle)), 1.0)
    check("a gentle display-space ramp at scale 2 -> 2 * 16", got == 32, f"got {got}")
    got = coarsest_reproducible_scale(side(sub_face(to_linear(steep))), 1.0)
    check("a steep ramp is flattened at the edge at every factor", got == 0, f"got {got}")


def test_hard_step_is_not_reproducible():
    grid = np.full((SIZE, SIZE), 20.0)
    grid[:, 8:] = 200.0
    got = coarsest_reproducible_scale(side(sub_face(grid)), 1.0)
    check("a hard step at column 8 -> 0", got == 0, f"got {got}")
    got = coarsest_reproducible_scale(side(sub_face(grid)), 1000.0)
    check("the same step passes a tolerance larger than the step (control)",
          got == 64, f"got {got}")
    # Overbright luxels (linear > 255) still differ on screen: no upper clamp.
    over = np.full((SIZE, SIZE), 300.0)
    over[:, 8:] = 900.0
    got = coarsest_reproducible_scale(side(sub_face(over)), 1.0)
    check("a step between two overbright values is still a step", got == 0, f"got {got}")


def test_smooth_bump_stops_at_the_factor_that_still_reproduces_it():
    # Curvature errs by about a*k^2/4 per axis between samples, and the slope
    # at the edges is flattened by the half-step grid; a = 0.05 gives a worst
    # error of 0.75 at k=2, 1.4 at k=4 and 2.4 at k=8.
    bump = display_grid(lambda x, y: 100.0 + 0.05 * ((x - 8.0) ** 2 + (y - 8.0) ** 2))
    disp = display(bump)
    e = {k: worst_recon_error(disp, k) for k in (2, 4, 8)}
    check("fixture: k=2 passes tolerance 1.0, k=4 passes 1.5, k=8 fails 1.5",
          e[2] <= 1.0 < e[4] <= 1.5 < e[8], f"errors {e}")
    got = coarsest_reproducible_scale(side(sub_face(bump)), 1.0)
    check("a smooth bump at scale 2 -> 2 * 2", got == 4, f"got {got}")
    got = coarsest_reproducible_scale(side(sub_face(bump)), 1.5)
    check("the same bump passes k=4 at a looser tolerance",
          got == 8, f"got {got}")


def test_a_crease_on_the_aligned_grid_is_caught_by_the_half_step_grid():
    # A roof ridge |x - 8| * 3 has its crease on a sample of every aligned
    # grid (8 is a multiple of 2, 4 and 8), so the aligned reconstruction is
    # exact. VRAD's coarser luxels need not land there: a grid shifted by half
    # a step straddles the crease and flattens it by up to 3 * k / 2. This is
    # the shadow-edge and light-falloff peak that recompiles showed the
    # aligned-only test letting through.
    ridge = display_grid(lambda x, y: 100.0 + 3.0 * np.abs(x - 8.0))
    disp = display(ridge)
    check("fixture: the aligned grid reproduces the ridge at k=2",
          float(recon_error(disp, 2).max()) <= 1.0)
    got = coarsest_reproducible_scale(side(sub_face(ridge)), 1.0)
    check("the ridge is not reproducible at any factor", got == 0, f"got {got}")


def test_strips_too_narrow_to_sample_give_no_evidence():
    # A strip 2 luxels wide has no luxel centre on the face: its lightmap is
    # VRAD's extrapolation and reconstructs perfectly at any factor. On
    # fightspace3 such a strip (side 46443, 8 units wide) was raised 12 -> 24
    # and lost 28/255 against a finer reference.
    strip = np.full((SIZE, 2), 120.0)
    got = coarsest_reproducible_scale(side(sub_face(strip)), 1.0)
    check("a side made only of 2-luxel strips -> no evidence", got == 0, f"got {got}")
    wall = np.full((SIZE, SIZE), 120.0)
    got = coarsest_reproducible_scale(side(sub_face(wall, 2, 0), sub_face(strip, 2, 1)), 1.0)
    check("a strip beside a full sub-face is skipped, not fatal", got == 64, f"got {got}")
    # The strip is left out of the test, not out of the base: at scale 2 it
    # still sets b, so a scale-4 wall's factor 32 gives 2 * 32, not 4 * 32.
    got = coarsest_reproducible_scale(side(sub_face(wall, 4, 0), sub_face(strip, 2, 1)), 1.0)
    check("a narrow strip's finer scale still sets the base", got == 64, f"got {got}")


def test_the_largest_passing_factor_wins_even_if_a_smaller_one_fails():
    # Period-4 ripple [0, -0.6, 0.9, -0.6]: nodes every 4th luxel see it within
    # 0.9, but nodes every 2nd luxel interpolate across it and miss by 1.05.
    # The fractions are not monotone in k; the answer is the largest k that works.
    ripple = np.array([0.0, -0.6, 0.9, -0.6] * 5)[:SIZE]
    disp = np.tile(100.0 + ripple, (SIZE, 1))
    errors = {k: float(recon_error(disp, k).max()) for k in (2, 4, 8, 16, 32)}
    check("fixture: k=2 fails and every coarser k passes tolerance 1.0",
          errors[2] > 1.0 and all(errors[k] <= 1.0 for k in (4, 8, 16, 32)),
          f"errors {errors}")
    got = coarsest_reproducible_scale(side(sub_face(to_linear(disp))), 1.0)
    check("k=2 failing does not hide k=32 passing", got == 64, f"got {got}")


def test_every_sub_face_must_reproduce_and_the_finest_scale_is_the_base():
    flat = np.full((SIZE, SIZE), 120.0)
    step = np.full((SIZE, SIZE), 20.0)
    step[:, 8:] = 200.0
    got = coarsest_reproducible_scale(side(sub_face(flat, 2, 0), sub_face(step, 2, 1)), 1.0)
    check("one failing sub-face spoils the side", got == 0, f"got {got}")
    # Each sub-face is tested on its own luxel grid, so k is validated
    # against the finest one; a coarser base would claim untested factors.
    got = coarsest_reproducible_scale(side(sub_face(flat, 2, 0), sub_face(flat, 4, 1)), 1.0)
    check("sub-faces at scales 2 and 4: base is the finest, 2 -> 2 * 32", got == 64, f"got {got}")


def test_missing_or_malformed_data_is_no_evidence():
    flat = np.full((SIZE, SIZE), 120.0)
    check("empty luminances -> 0",
          coarsest_reproducible_scale(
              side(BSPSubFaceInfo(0, (0, 0), (16, 16),
                                  sub_face(flat).lightmap_vecs, [], False)), 1.0) == 0)
    short = sub_face(flat)
    short.luminances = short.luminances[:-1]
    check("a luminance list not matching lightmap_size -> 0",
          coarsest_reproducible_scale(side(short), 1.0) == 0)
    no_vecs = sub_face(flat)
    no_vecs.lightmap_vecs = None
    check("no lightmap_vecs -> 0", coarsest_reproducible_scale(side(no_vecs), 1.0) == 0)
    zero_vecs = sub_face(flat)
    zero_vecs.lightmap_vecs = ((0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0))
    check("zero-length lightmap_vecs -> 0",
          coarsest_reproducible_scale(side(zero_vecs), 1.0) == 0)
    check("no sub-faces -> 0",
          coarsest_reproducible_scale(side(), 1.0) == 0)
    never = side(sub_face(flat))
    never.is_never_visible = True
    check("a never-visible side -> 0", coarsest_reproducible_scale(never, 1.0) == 0)
    check("control: the same data is 64 when it is visible",
          coarsest_reproducible_scale(side(sub_face(flat)), 1.0) == 64)
    check("factors restrict the answer",
          coarsest_reproducible_scale(side(sub_face(flat)), 1.0, factors=(2, 4)) == 8)


def test_evidence_scales_keeps_only_sides_with_evidence():
    flat = side(sub_face(np.full((SIZE, SIZE), 120.0)))
    empty = FaceLightmapData(vmf_side_id=2, material='M', bsp_face_indices=[],
                             luminances=[])
    got = lightmap_evidence.evidence_scales({1: flat, 2: empty}, 1.0)
    check("only the side with data appears", got == {1: 64}, f"got {got}")


# ─── B. solver: the evidence is a floor ──────────────────────────────────────

def _keyless(sid):
    return KVNode(name='side', children=[KVPair('id', str(sid)),
                                         KVPair('material', 'CONCRETE/FLOOR')])


def _extent(sid, span):
    # max useful scale = ceil(span / 32): 512 -> 16, 128 -> 4
    return FaceExtent(side_id=sid, extent_s=span, extent_t=span, s_min=0, s_max=span,
                      t_min=0, t_max=span, s_axis=(1, 0, 0), t_axis=(0, 1, 0),
                      lightmapscale=16, material='CONCRETE/FLOOR',
                      num_vertices=4, brush_entity='')


def _solve(td, budget, coplanar_groups=([4, 5], [6, 7]), **kw):
    """Eight sides; the mocked VBSP charges 1000 // scale per side, so the
    verts, leaffaces and faces counts all equal that sum.

      1  evidence 8                          -> the floor, even with top priority
      2  no evidence                         -> ordinary
      3  emissive, evidence 8                -> evidence ignored
      4,5  coplanar, evidence 8 and 4        -> both take the group minimum, 4
      6,7  coplanar, evidence 8 and none     -> a member without evidence: none
      8  evidence 128, max useful scale 4    -> capped at 4
    """
    nodes = {sid: _keyless(sid) for sid in range(1, 9)}
    root = KVNode(name='__root__', children=list(nodes.values()))
    extents = [_extent(sid, 128 if sid == 8 else 512) for sid in nodes]
    # Side 1 has the highest promotion priority, so a missing exclusion
    # would promote it to scale 1 first.
    lighting = {1: FaceLightmapData(vmf_side_id=1, material='M', bsp_face_indices=[],
                                    luminances=[10.0, 200.0, 10.0, 200.0])}

    def fake_count(vbsp_exe, vmf_path, game_dir, **kw):
        n = sum(1000 // int(node.get_property('lightmapscale')) for node in nodes.values())
        return vbsp_runner.VertexCount(n, False, n, False, n, False)

    with mock.patch.object(vbsp_runner, 'count_vertices', autospec=True,
                           side_effect=fake_count):
        return vbsp_solver.solve_with_vbsp(
            face_extents=extents, lighting_data=lighting, vmf_root=root,
            side_map=nodes, vbsp_exe=Path(td) / 'vbsp.exe', game_dir=Path(td),
            input_vmf=Path(td) / 'm.vmf', vertex_budget=budget,
            detail_type_materials=set(), detail_min_scale=5,
            coplanar_groups=[list(g) for g in coplanar_groups] if coplanar_groups else None,
            emissive_sides={3},
            evidence_scales={1: 8, 3: 8, 4: 8, 5: 4, 6: 8, 8: 128}, **kw), nodes


def test_solver_floors_sides_at_their_evidence_and_does_not_promote_them():
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as td:
        # Budget 4000: all at 1 costs 4875, so the baseline is 2 (2875) and a
        # promotion round runs. Promoting side 1 would cost 875 more.
        result, nodes = _solve(td, 4000)
        s = result.scales
        check("fixture: the baseline is 2, so Phase 2 ran",
              result.baseline_scale == 2, f"baseline {result.baseline_scale}")
        check("side 1 stays at its evidence scale, not promoted to 1",
              s[1] == 8, f"got {s[1]}")
        check("coplanar members 4 and 5 both take the group minimum",
              s[4] == 4 and s[5] == 4, f"got {s[4]}, {s[5]}")
        check("a member without evidence: 6 and 7 get no floor (at most the baseline)",
              s[6] <= 2 and s[7] <= 2, f"got {s[6]}, {s[7]}")
        check("an emissive side ignores its evidence",
              s[3] <= 2, f"got {s[3]}")
        check("evidence is capped at the side's max useful scale (4)",
              s[8] == 4, f"got {s[8]}")
        check("the scales reach the VMF nodes",
              all(int(nodes[sid].get_property('lightmapscale')) == s[sid] for sid in s))
        check("sides with a floor are not counted as promoted or at baseline",
              result.faces_promoted + result.faces_at_baseline == 4,
              f"promoted {result.faces_promoted} at baseline {result.faces_at_baseline}")
        check("faces_evidence_raised counts sides 1, 4, 5 and 8",
              result.faces_evidence_raised == 4, f"got {result.faces_evidence_raised}")
        check("the result fits the budget",
              max(result.vbsp_vertex_count, result.vbsp_leafface_count,
                  result.vbsp_face_count) <= 4000)

    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as td:
        # Budget 3000: scale 2 fits only if the baseline scan counts the floors
        # (2875); counted without them the baseline would have to be 3.
        result, _nodes = _solve(td, 3000)
        check("the baseline scan applies the floors: scale 2 fits at 2875",
              result.baseline_scale == 2 and result.vbsp_vertex_count == 2875,
              f"baseline {result.baseline_scale} verts {result.vbsp_vertex_count}")

    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as td:
        # Room for everything at scale 1: the early-exit path.
        result, nodes = _solve(td, 100000)
        s = result.scales
        check("early exit: sides without a floor go to 1",
              result.baseline_scale == 1 and s[2] == s[3] == s[6] == s[7] == 1,
              f"baseline {result.baseline_scale} scales {s}")
        check("early exit: sides with a floor keep it",
              (s[1], s[4], s[5], s[8]) == (8, 4, 4, 4), f"got {s}")
        check("early exit: promoted counts only the sides without a floor",
              result.faces_promoted == 4, f"got {result.faces_promoted}")
        check("early exit: faces_evidence_raised counts sides 1, 4, 5 and 8",
              result.faces_evidence_raised == 4, f"got {result.faces_evidence_raised}")


def test_solver_without_evidence_is_unchanged():
    """Control: no evidence_scales -> side 1 is the top promotion candidate and
    nothing is counted as evidence-raised."""
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as td:
        nodes = {sid: _keyless(sid) for sid in (1, 2)}
        root = KVNode(name='__root__', children=list(nodes.values()))

        def fake_count(vbsp_exe, vmf_path, game_dir, **kw):
            n = sum(1000 // int(node.get_property('lightmapscale')) for node in nodes.values())
            return vbsp_runner.VertexCount(n, False, n, False, n, False)

        with mock.patch.object(vbsp_runner, 'count_vertices', autospec=True,
                               side_effect=fake_count):
            result = vbsp_solver.solve_with_vbsp(
                face_extents=[_extent(1, 512), _extent(2, 512)], lighting_data={},
                vmf_root=root, side_map=nodes, vbsp_exe=Path(td) / 'vbsp.exe',
                game_dir=Path(td), input_vmf=Path(td) / 'm.vmf',
                vertex_budget=100000, detail_type_materials=set(),
                detail_min_scale=5)
        check("no evidence: both sides at 1, nothing evidence-raised",
              result.scales == {1: 1, 2: 1} and result.faces_evidence_raised == 0,
              f"got {result.scales} raised {result.faces_evidence_raised}")


def test_strict_mode_still_takes_the_group_minimum():
    """--strict-coplanar passes no promotion groups; the evidence minimum must
    still run over the seam groups (evidence_groups) so a group keeps one density."""
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as td:
        result, _nodes = _solve(td, 4000, coplanar_groups=None,
                                evidence_groups=[[4, 5], [6, 7]])
        s = result.scales
        check("strict: coplanar 4 and 5 both take the group minimum, 4",
              s[4] == 4 and s[5] == 4, f"got {s[4]}, {s[5]}")
        check("strict: 6 has no floor because its coplanar 7 has no evidence",
              s[6] <= 2, f"got {s[6]}")


# Four sides with their own VBSP cost (cost // scale each), for the cases
# where a floor must NOT block promotion:
#   1  span 512, evidence 16, coplanar with 2   -> floored at 8 (the group min)
#   2  span 32 (max useful 1), evidence 8       -> capped to 1 = no floor
#   3  span 512, evidence 2                     -> below the baseline (3):
#                                                  promotable, promoted to 2
#   4  span 512, no evidence, expensive         -> ordinary, stays at baseline
_COST = {1: 1000, 2: 60, 3: 600, 4: 6000}


def _solve_vacuous(td, budget, evidence=None):
    nodes = {sid: _keyless(sid) for sid in _COST}
    root = KVNode(name='__root__', children=list(nodes.values()))
    extents = [_extent(sid, 32 if sid == 2 else 512) for sid in nodes]

    def fake_count(vbsp_exe, vmf_path, game_dir, **kw):
        n = sum(_COST[sid] // int(node.get_property('lightmapscale'))
                for sid, node in nodes.items())
        return vbsp_runner.VertexCount(n, False, n, False, n, False)

    with mock.patch.object(vbsp_runner, 'count_vertices', autospec=True,
                           side_effect=fake_count):
        return vbsp_solver.solve_with_vbsp(
            face_extents=extents, lighting_data={}, vmf_root=root,
            side_map=nodes, vbsp_exe=Path(td) / 'vbsp.exe', game_dir=Path(td),
            input_vmf=Path(td) / 'm.vmf', vertex_budget=budget,
            detail_type_materials=set(), detail_min_scale=5,
            coplanar_groups=[[1, 2]],
            evidence_scales=evidence or {1: 16, 2: 8, 3: 2})


def _cost(scales):
    return sum(_COST[sid] // sc for sid, sc in scales.items())


def test_floors_that_change_nothing_do_not_block_promotion():
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as td:
        # Phase 1: scale 2 costs 3455, 3 costs 2345 -> baseline 3 at 2500.
        # Phase 2 units in order [2], [3], [4]: promoting 2 (+40) and 3 to its
        # evidence scale 2 (+100) fits at 2485; 3 tried at 1 (+400) would not.
        result = _solve_vacuous(td, 2500)
        s = result.scales
        check("fixture: the baseline is 3",
              result.baseline_scale == 3, f"baseline {result.baseline_scale}")
        check("a small coplanar sibling (max useful 1) does not drag side 1's "
              "floor down: side 1 sits at the group's raw minimum, 8",
              s[1] == 8, f"got {s[1]}")
        check("side 2, capped to 1, has no floor and is promoted to 1",
              s[2] == 1, f"got {s[2]}")
        check("side 3, evidence 2 below the baseline, is promoted to 2, not 1",
              s[3] == 2, f"got {s[3]}")
        check("side 4 stays at the baseline", s[4] == 3, f"got {s[4]}")
        check("promoted counts sides 2 and 3; at baseline counts side 4",
              (result.faces_promoted, result.faces_at_baseline) == (2, 1),
              f"got {result.faces_promoted}, {result.faces_at_baseline}")
        check("faces_evidence_raised counts only side 1",
              result.faces_evidence_raised == 1, f"got {result.faces_evidence_raised}")
        # The reported counts come from the Phase 2 trial; they must be the
        # cost of the floored final assignment (side 1 at 8, side 3 at 2).
        check("the Phase 2 trial applies the floors: counts equal the cost "
              "of the final scales {1: 8, 2: 1, 3: 2, 4: 3} (2485)",
              s == {1: 8, 2: 1, 3: 2, 4: 3}
              and result.vbsp_vertex_count == _cost(s) == 2485,
              f"scales {s} verts {result.vbsp_vertex_count} cost {_cost(s)}")

    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as td:
        # Early exit (everything fits at 1): sides 1 and 3 keep their floors.
        result = _solve_vacuous(td, 100000)
        s = result.scales
        check("early exit: floors kept, the rest at 1",
              result.baseline_scale == 1 and s == {1: 8, 2: 1, 3: 2, 4: 1},
              f"baseline {result.baseline_scale} scales {s}")
        check("early exit: promoted counts sides 2 and 4, raised counts 1 and 3",
              (result.faces_promoted, result.faces_evidence_raised) == (2, 2),
              f"got {result.faces_promoted}, {result.faces_evidence_raised}")

    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as td:
        # Side 3's floor equals the baseline (3): the baseline already
        # reproduces it, so it must not take a promotion slot ("promoted" to
        # the scale it already has). Baseline 3 costs 2345; promoting 2 fits.
        # It still counts as at baseline (with side 4).
        result = _solve_vacuous(td, 2500, evidence={1: 16, 2: 8, 3: 3})
        s = result.scales
        check("a floor equal to the baseline: side 3 stays at 3, not promoted",
              result.baseline_scale == 3 and s == {1: 8, 2: 1, 3: 3, 4: 3}
              and (result.faces_promoted, result.faces_at_baseline) == (1, 2),
              f"baseline {result.baseline_scale} scales {s} promoted/at baseline "
              f"{result.faces_promoted}/{result.faces_at_baseline}")
        check("...and it is not counted as raised",
              result.faces_evidence_raised == 1, f"got {result.faces_evidence_raised}")


# ─── D. solver: the skip-ahead pass after the prefix search ──────────────────

_SKIP_BASE = 100  # every side costs this at any scale above 1


def _solve_skip(td, extras, budget, **kw):
    """One side per promotion unit, in priority order (side 1 first). The mocked
    VBSP charges a side _SKIP_BASE at any scale above 1 and _SKIP_BASE + its
    extra at scale 1, so verts, leaffaces and faces all equal the sum and a
    unit's cost of promotion is its extra. With the budget below the all-at-1
    total the baseline is 2 and Phase 2 runs."""
    sids = range(1, len(extras) + 1)
    nodes = {sid: _keyless(sid) for sid in sids}
    root = KVNode(name='__root__', children=list(nodes.values()))
    # priority = std_dev * mean, strictly decreasing with the side id
    lighting = {sid: FaceLightmapData(
        vmf_side_id=sid, material='M', bsp_face_indices=[],
        luminances=[90.0 * (len(extras) + 1 - sid), 110.0 * (len(extras) + 1 - sid)])
        for sid in sids}

    def cost(scales):
        return sum(_SKIP_BASE + (extras[sid - 1] if scales[sid] == 1 else 0)
                   for sid in sids)

    def fake_count(vbsp_exe, vmf_path, game_dir, **kw):
        n = cost({sid: int(node.get_property('lightmapscale'))
                  for sid, node in nodes.items()})
        return vbsp_runner.VertexCount(n, False, n, False, n, False)

    with mock.patch.object(vbsp_runner, 'count_vertices', autospec=True,
                           side_effect=fake_count):
        result = vbsp_solver.solve_with_vbsp(
            face_extents=[_extent(sid, 512) for sid in sids],
            lighting_data=lighting, vmf_root=root, side_map=nodes,
            vbsp_exe=Path(td) / 'vbsp.exe', game_dir=Path(td),
            input_vmf=Path(td) / 'm.vmf', vertex_budget=budget,
            detail_type_materials=set(), detail_min_scale=5,
            **kw)
    return result, nodes, cost


def test_a_unit_that_does_not_fit_no_longer_ends_promotion():
    """The prefix search stops at the first unit that overflows. Unit 2 alone
    costs 1000, units 1 and 3-5 cost 50 each, and 200 of margin is enough for
    the four cheap ones: they must all be promoted, in priority order, with
    only unit 2 left at the baseline."""
    extras = [50, 1000, 50, 50, 50]
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as td:
        # Baseline 2 costs 500; the budget leaves 200 for promotions.
        result, nodes, cost = _solve_skip(td, extras, 700)
        s = result.scales
        check("fixture: the baseline is 2, so Phase 2 ran",
              result.baseline_scale == 2, f"baseline {result.baseline_scale}")
        check("units 1, 3, 4 and 5 are promoted to scale 1 and unit 2 stays at "
              "the baseline",
              s == {1: 1, 2: 2, 3: 1, 4: 1, 5: 1}, f"scales {s}")
        check("the scales reach the VMF nodes",
              all(int(nodes[sid].get_property('lightmapscale')) == s[sid] for sid in s))
        check("the reported counts are the VBSP count of the final scales, "
              "and fit the budget",
              result.vbsp_vertex_count == result.vbsp_leafface_count
              == result.vbsp_face_count == cost(s) == 700,
              f"verts {result.vbsp_vertex_count} cost {cost(s)}")
        check("faces_promoted and faces_at_baseline describe the final set",
              (result.faces_promoted, result.faces_at_baseline) == (4, 1),
              f"got {result.faces_promoted}, {result.faces_at_baseline}")
        # The decision records (the .lmo_report.json source): each side says
        # why it ended where it did.
        rec = _decisions(result)
        check("the prefix unit (side 1) reports promoted_prefix",
              rec(1).get('decision') == 'promoted_prefix', f"got {rec(1)}")
        check("the unit that does not fit (side 2) reports stepped_over, as the "
              "stranding unit, with the 150 of budget the prefix left it",
              rec(2).get('decision') == 'stepped_over'
              and rec(2).get('stranding') is True
              and rec(2).get('headroom_at_skip') == 150
              and rec(2).get('binding_limit') in ('verts', 'leaffaces', 'faces')
              and rec(2).get('trial_worst_count') == 1550,
              f"got {rec(2)}")
        check("its record carries the rank, size and the chop cells promotion adds "
              "(81 at baseline 2, 289 at scale 1)",
              (rec(2).get('rank'), rec(2).get('unit_size'),
               rec(2).get('chop_cells_at_baseline'),
               rec(2).get('chop_cells_promoted')) == (2, 1, 81, 289),
              f"got {rec(2)}")
        check("the cheaper later units 3-5 report promoted_skip_ahead",
              all(rec(sid).get('decision') == 'promoted_skip_ahead'
                  for sid in (3, 4, 5)),
              f"got {[rec(sid).get('decision') for sid in (3, 4, 5)]}")
        run = tuple(getattr(result, name, None) for name in
                    ('skip_ahead_cap', 'prefix_units', 'promotion_units'))
        check("the run records the skip-ahead settings it used",
              run == (40, 1, 5) and (getattr(result, 'skip_ahead_used', 0) or 0) > 0,
              f"got {run}, used {getattr(result, 'skip_ahead_used', None)}")


def _decisions(result):
    """result.decisions as a lookup that answers {} for a side it has no record
    of (or for a result without the field), so a missing record fails a check
    on its value instead of crashing the test."""
    table = getattr(result, 'decisions', {})
    return lambda sid: table.get(sid, {})


def test_skip_ahead_calls_zero_is_the_prefix_search_only():
    extras = [50, 1000, 50, 50, 50]
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as td:
        result, _nodes, cost = _solve_skip(td, extras, 700, skip_ahead_calls=0)
        s = result.scales
        check("0 disables the pass: only the prefix (unit 1) is promoted",
              s == {1: 1, 2: 2, 3: 2, 4: 2, 5: 2}, f"scales {s}")
        check("the counts are the prefix trial's",
              result.vbsp_vertex_count == cost(s) == 550,
              f"verts {result.vbsp_vertex_count} cost {cost(s)}")
        check("faces_promoted counts the prefix only",
              (result.faces_promoted, result.faces_at_baseline) == (1, 4),
              f"got {result.faces_promoted}, {result.faces_at_baseline}")
        rec = _decisions(result)
        check("with the pass off the stranding unit is still stepped_over and "
              "the units after it were never tried (not_reached)",
              [rec(sid).get('decision') for sid in (1, 2, 3, 4, 5)]
              == ['promoted_prefix', 'stepped_over', 'not_reached',
                  'not_reached', 'not_reached']
              and getattr(result, 'skip_ahead_used', None) == 0,
              f"got {[rec(sid).get('decision') for sid in (1, 2, 3, 4, 5)]} "
              f"used {getattr(result, 'skip_ahead_used', None)}")


def test_skip_ahead_matches_first_fit_and_honors_its_call_cap():
    """Costs are additive, so skipping ahead with an uncapped budget of calls
    must give exactly what a one-by-one first-fit pass over the units after
    the prefix gives: here units 7, 8, 9, 11 and 14 join the prefix 1-5 while
    6, 10, 12 and 13 do not. The runs it tries (1, 2, 4, then 3 between) cross
    the 900 unit, so a wrong bisection would overflow or miss one."""
    extras = [30, 40, 500, 20, 20, 700, 30, 10, 25, 900, 15, 15, 15, 5]
    budget = _SKIP_BASE * len(extras) + 700
    fits = []
    used = 0
    for i, e in enumerate(extras):
        if used + e <= 700:
            used += e
            fits.append(i + 1)
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as td:
        prefix, _n, _c = _solve_skip(td, extras, budget, skip_ahead_calls=0)
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as td:
        full, _n, cost = _solve_skip(td, extras, budget, skip_ahead_calls=1000)
        got = sorted(sid for sid, sc in full.scales.items() if sc == 1)
        check("fixture: the prefix is units 1-5, so the pass has work to do",
              sorted(sid for sid, sc in prefix.scales.items() if sc == 1) == [1, 2, 3, 4, 5],
              f"prefix scales {prefix.scales}")
        check("the promoted set is the first-fit set", got == fits,
              f"got {got} want {fits}")
        check("the counts are the VBSP count of the final scales, and fit",
              full.vbsp_vertex_count == cost(full.scales) <= budget,
              f"verts {full.vbsp_vertex_count} cost {cost(full.scales)}")
        check("faces_promoted counts the final set",
              full.faces_promoted == len(fits), f"got {full.faces_promoted}")
        # On this fixture the pass needs 8 calls (galloping can take more
        # than one call per later unit in general). A pass that never ends
        # (e.g. forgetting the end of the list) would burn the whole cap.
        check("the pass takes at most the 8 VBSP calls this fixture needs",
              full.vbsp_calls - prefix.vbsp_calls <= 8,
              f"extra {full.vbsp_calls - prefix.vbsp_calls}")
        rec = _decisions(full)
        verdicts = {sid: rec(sid).get('decision') for sid in range(1, 15)}
        check("uncapped, every unit has a verdict: 1-5 prefix, the first-fit "
              "units after it skip_ahead, the rest stepped_over, none not_reached",
              verdicts == {sid: ('promoted_prefix' if sid <= 5
                                 else 'promoted_skip_ahead' if sid in fits
                                 else 'stepped_over') for sid in range(1, 15)},
              f"got {verdicts}")
        check("a unit stepped over mid-list reports the budget left at that "
              "moment (700 minus what the units accepted before it spent) and "
              "the trial that overflowed (a worst count above the budget)",
              all(rec(sid).get('headroom_at_skip') == 700 - sum(
                      extras[i - 1] for i in fits if i < sid)
                  and rec(sid).get('binding_limit') in ('verts', 'leaffaces', 'faces')
                  and rec(sid).get('trial_worst_count', 0) > budget
                  for sid in range(6, 15) if sid not in fits),
              f"got {[(sid, rec(sid).get('headroom_at_skip'), rec(sid).get('binding_limit'), rec(sid).get('trial_worst_count')) for sid in range(6, 15) if sid not in fits]}")
    # The cap can land in the gallop (2), at the first failure (3), in the
    # bisection (4) or between rounds (7); the pass needs 8 calls in all.
    for cap in (1, 2, 3, 4, 7):
        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as td:
            capped, _n, cost = _solve_skip(td, extras, budget, skip_ahead_calls=cap)
            got = sorted(sid for sid, sc in capped.scales.items() if sc == 1)
            check(f"cap {cap}: exactly {cap} extra VBSP calls",
                  capped.vbsp_calls - prefix.vbsp_calls == cap,
                  f"extra {capped.vbsp_calls - prefix.vbsp_calls}")
            check(f"cap {cap}: a counted fitting set that keeps the whole prefix "
                  "and is a subset of first-fit",
                  set(range(1, 6)) <= set(got) <= set(fits)
                  and capped.vbsp_vertex_count == cost(capped.scales) <= budget,
                  f"got {got} verts {capped.vbsp_vertex_count} cost {cost(capped.scales)}")
            rec = _decisions(capped)
            table = getattr(capped, 'decisions', {})
            check(f"cap {cap}: the promoted_* decisions are exactly the scale-1 "
                  "sides and the pass reports the calls it spent",
                  {sid for sid, r in table.items()
                   if r.get('decision', '').startswith('promoted')} == set(got)
                  and getattr(capped, 'skip_ahead_used', None) == cap,
                  f"got {sorted(table)} used {getattr(capped, 'skip_ahead_used', None)}")
            if cap == 1:
                check("cap 1: one call tries unit 7, so 6 is stepped over (the "
                      "stranding unit) and 8-14 are not_reached",
                      {sid: rec(sid).get('decision') for sid in range(6, 15)}
                      == {6: 'stepped_over', 7: 'promoted_skip_ahead',
                          **{sid: 'not_reached' for sid in range(8, 15)}},
                      f"got {[(sid, rec(sid).get('decision')) for sid in range(6, 15)]}")


# ─── E. solver: promotion order by value per cost ────────────────────────────

# Sides 1-4 are one coplanar group of big faces (span 150); sides 5-10 are
# cheap singletons (span 60). The mocked VBSP charges 10 per chop cell, a face
# being chopped every ~32 luxels per axis. The spans keep that count and the
# solver's own estimate in agreement at the scales visited:
#   big face   scale 1: 5x5 = 25 cells   scale 2: 3x3 = 9    (+16 to promote)
#   small face scale 1: 2x2 = 4 cells    scale 2: 1          (+3 to promote)
_ORDER_SPAN = {**{sid: 150 for sid in (1, 2, 3, 4)},
               **{sid: 60 for sid in range(5, 11)}}
_ORDER_CELL_COST = 10


def _chop_cells(span, scale):
    n = max(1, math.ceil(span / scale / 32))
    return n * n


def _solve_order(td, budget, detail, reject_above=None, **kw):
    """Ten sides under the cost model above. detail maps side id -> half the
    spread of its two lightmap samples, so its promotion priority (std dev x
    mean) is proportional to it. The all-at-1 total is 1240 and all-at-2 is
    420, so a budget between them gives baseline 2: the group costs +640 to
    promote, the six small faces +180 together, +30 each."""
    nodes = {sid: _keyless(sid) for sid in _ORDER_SPAN}
    root = KVNode(name='__root__', children=list(nodes.values()))
    lighting = {sid: FaceLightmapData(
        vmf_side_id=sid, material='M', bsp_face_indices=[],
        luminances=[100.0 - d, 100.0 + d]) for sid, d in detail.items()}

    def cost(scales):
        return sum(_ORDER_CELL_COST * _chop_cells(_ORDER_SPAN[sid], sc)
                   for sid, sc in scales.items())

    def fake_count(vbsp_exe, vmf_path, game_dir, **kw):
        scales = {sid: int(node.get_property('lightmapscale'))
                  for sid, node in nodes.items()}
        # reject_above: VBSP refuses any trial whose sides 2-10 are all
        # coarser than it, so later trials are rejected after a counted one.
        if reject_above and min(scales[s] for s in range(2, 11)) > reject_above:
            raise vbsp_runner.VBSPRejectedError("mock capacity error")
        n = cost(scales)
        return vbsp_runner.VertexCount(n, False, n, False, n, False)

    with mock.patch.object(vbsp_runner, 'count_vertices', autospec=True,
                           side_effect=fake_count):
        result = vbsp_solver.solve_with_vbsp(
            face_extents=[_extent(sid, span) for sid, span in _ORDER_SPAN.items()],
            lighting_data=lighting, vmf_root=root, side_map=nodes,
            vbsp_exe=Path(td) / 'vbsp.exe', game_dir=Path(td),
            input_vmf=Path(td) / 'm.vmf', vertex_budget=budget,
            detail_type_materials=set(), detail_min_scale=5,
            coplanar_groups=[[1, 2, 3, 4]], **kw)
    cost.nodes = nodes  # the VMF side nodes, for tests that read the tree
    return result, cost


def _promoted(result):
    return sorted(sid for sid, sc in result.scales.items() if sc == 1)


# One detailed member (40) and three nearly flat ones; the small faces are
# moderately detailed (20), half the group's best.
_ONE_DETAILED = {1: 40.0, 2: 1.0, 3: 1.0, 4: 1.0, **{sid: 20.0 for sid in range(5, 11)}}
# Control: every group member is as detailed as that one.
_ALL_DETAILED = {**_ONE_DETAILED, 2: 40.0, 3: 40.0, 4: 40.0}


def test_value_order_promotes_cheap_units_ahead_of_an_expensive_group():
    """The group ranks 40 on its single detailed member, the small faces 20,
    but the group costs 640 for four faces of which three are flat. A budget
    of 1100 holds either the six small faces (+180) or the group (+640) and
    one small face, not the group and a second one: max-priority order spends
    the margin on the group; value per cost must spend it on the cheap ones."""
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as td:
        # No keyword: the default is the value order.
        result, cost = _solve_order(td, 1100, _ONE_DETAILED)
        check("fixture: the baseline is 2, so Phase 2 ran",
              result.baseline_scale == 2, f"baseline {result.baseline_scale}")
        got = _promoted(result)
        check("default order: the six small faces are promoted, the group stays "
              "at the baseline", got == [5, 6, 7, 8, 9, 10], f"promoted {got}")
        check("the counts are the VBSP count of the final scales, and fit",
              result.vbsp_vertex_count == cost(result.scales) == 600,
              f"verts {result.vbsp_vertex_count} cost {cost(result.scales)}")

    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as td:
        result, _cost_fn = _solve_order(td, 1100, _ONE_DETAILED, promote_order="value")
        got = _promoted(result)
        check("promote_order='value' gives the same set", got == [5, 6, 7, 8, 9, 10],
              f"promoted {got}")

    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as td:
        result, cost = _solve_order(td, 1100, _ONE_DETAILED, promote_order="priority")
        got = _promoted(result)
        check("promote_order='priority' is the old order: the group and one "
              "small face", got == [1, 2, 3, 4, 5], f"promoted {got}")
        check("...and its counts are the VBSP count of its final scales",
              result.vbsp_vertex_count == cost(result.scales) == 1090,
              f"verts {result.vbsp_vertex_count} cost {cost(result.scales)}")

    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as td:
        # Control: when every group member is detailed the group is worth its
        # cost (4 x 40 over +640 beats 20 per +30), so the value order is not
        # simply 'cheap first' or 'groups last'.
        result, _cost_fn = _solve_order(td, 1100, _ALL_DETAILED, promote_order="value")
        got = _promoted(result)
        check("control: an all-detailed group is promoted first, then one small face",
              got == [1, 2, 3, 4, 5], f"promoted {got}")

    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as td:
        try:
            _solve_order(td, 1100, _ONE_DETAILED, promote_order="cheapest")
        except ValueError:
            pass
        else:
            check("an unknown promote_order is rejected", False, "no ValueError")


def test_value_order_gives_a_side_held_at_its_minimum_scale_no_value():
    """Side 1 is the group's detailed member but a minimum scale holds it at
    5, so promoting the group only refines its three flat members (+480 for
    them, side 1 costs nothing either way). On area alone side 1 would carry
    the group ahead of the small faces; its luxel gain is 0, so the value
    order must take the six small faces (+180) and leave the group, which no
    longer fits a budget of 830. The priority order takes the group, and its
    promoted-face count must not include side 1, whose scale never moved."""
    detail = {1: 60.0, 2: 1.0, 3: 1.0, 4: 1.0, **{sid: 20.0 for sid in range(5, 11)}}
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as td:
        # verbose=True as lmoptimizer always calls it: its value-order report
        # must run (output captured).
        with contextlib.redirect_stdout(io.StringIO()):
            result, cost = _solve_order(td, 830, detail, side_min_scales={1: 5},
                                        verbose=True)
        check("fixture: baseline 2 with side 1 held at 5",
              result.baseline_scale == 2 and result.scales.get(1) == 5,
              f"baseline {result.baseline_scale} side 1 at {result.scales.get(1)}")
        got = _promoted(result)
        check("value order: the six small faces, not the group whose detailed "
              "member cannot move", got == [5, 6, 7, 8, 9, 10], f"promoted {got}")
        check("...counted by VBSP and within the budget",
              result.vbsp_vertex_count == cost(result.scales) == 520,
              f"verts {result.vbsp_vertex_count} cost {cost(result.scales)}")

    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as td:
        with contextlib.redirect_stdout(io.StringIO()):
            result, _cost_fn = _solve_order(td, 830, detail, side_min_scales={1: 5},
                                            promote_order="priority", verbose=True)
        got = _promoted(result)
        check("priority order takes the group", got == [2, 3, 4], f"promoted {got}")
        check("faces_promoted counts the three sides whose scale changed, not "
              "side 1", result.faces_promoted == 3,
              f"faces_promoted {result.faces_promoted}")

    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as td:
        # Everything fits at scale 1 (the early return): side 1 still stays
        # at 5, so it is not one of the faces promoted to 1 there either.
        result, _cost_fn = _solve_order(td, 100000, detail, side_min_scales={1: 5})
        check("baseline 1: faces_promoted is the nine sides at scale 1",
              result.baseline_scale == 1 and result.faces_promoted == 9
              and result.faces_at_baseline == 1,
              f"baseline {result.baseline_scale} promoted {result.faces_promoted} "
              f"at baseline {result.faces_at_baseline}")


def test_no_baseline_fits_returns_the_assignment_vbsp_counted():
    """Nothing fits even at 64: the result must still be an assignment VBSP
    counted, with the VMF tree matching it. Side 1's minimum scale of 128 is
    above 64, so the old synthesized all-64 dict was never counted. VBSP
    rejects every global scale above 39, so the last trial written to the
    tree (64) is not the last one counted (39)."""
    detail = {sid: 20.0 for sid in range(1, 11)}
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as td:
        result, cost = _solve_order(td, 1, detail, side_min_scales={1: 128},
                                    reject_above=39)
        check("baseline_scale is the last counted trial's scale",
              result.baseline_scale == 39 and result.scales.get(2) == 39,
              f"baseline {result.baseline_scale} side 2 at {result.scales.get(2)}")
        check("fixture: no baseline fits", result.vbsp_vertex_count > 1,
              f"verts {result.vbsp_vertex_count}")
        check("the returned scales are the counted assignment",
              result.scales.get(1) == 128
              and cost(result.scales) == result.vbsp_vertex_count,
              f"side 1 at {result.scales.get(1)}, cost {cost(result.scales)} "
              f"vs counted {result.vbsp_vertex_count}")
        tree = {sid: int(node.get_property('lightmapscale'))
                for sid, node in cost.nodes.items()}
        check("the VMF tree holds the same scales", tree == result.scales,
              f"tree {tree} result {result.scales}")


def test_never_visible_sides_give_no_evidence():
    """lmoptimizer's visibility override flattens a never-visible side's
    luminances but leaves its sub-faces (and is_never_visible) alone; those
    sides must still be left out of the evidence."""
    import lmoptimizer
    flat = np.full((SIZE, SIZE), 120.0)
    hidden = side(sub_face(flat, 2, 0))
    seen = side(sub_face(flat, 2, 1))
    seen.vmf_side_id = 2
    face_data = {1: hidden, 2: seen}
    raw_nv, nv = lmoptimizer._mark_never_visible(face_data, {0}, texlight_sides=set())
    check("fixture: the override marks side 1 never-visible and flattens it",
          nv == {1} and hidden.luminances == [0.0], f"nv {nv}")
    got = lmoptimizer._smooth_evidence(face_data, 1.0, nv)
    check("only the visible side has evidence", got == {2: 64}, f"got {got}")


TESTS = [
    test_constant_side_takes_the_coarsest_factor,
    test_ramp_is_reproduced_but_a_luminance_ramp_is_not,
    test_hard_step_is_not_reproducible,
    test_smooth_bump_stops_at_the_factor_that_still_reproduces_it,
    test_a_crease_on_the_aligned_grid_is_caught_by_the_half_step_grid,
    test_strips_too_narrow_to_sample_give_no_evidence,
    test_the_largest_passing_factor_wins_even_if_a_smaller_one_fails,
    test_every_sub_face_must_reproduce_and_the_finest_scale_is_the_base,
    test_missing_or_malformed_data_is_no_evidence,
    test_evidence_scales_keeps_only_sides_with_evidence,
    test_solver_floors_sides_at_their_evidence_and_does_not_promote_them,
    test_solver_without_evidence_is_unchanged,
    test_strict_mode_still_takes_the_group_minimum,
    test_floors_that_change_nothing_do_not_block_promotion,
    test_a_unit_that_does_not_fit_no_longer_ends_promotion,
    test_skip_ahead_calls_zero_is_the_prefix_search_only,
    test_skip_ahead_matches_first_fit_and_honors_its_call_cap,
    test_value_order_promotes_cheap_units_ahead_of_an_expensive_group,
    test_value_order_gives_a_side_held_at_its_minimum_scale_no_value,
    test_no_baseline_fits_returns_the_assignment_vbsp_counted,
    test_never_visible_sides_give_no_evidence,
]


if __name__ == '__main__':
    _sys.exit(run_all("test_smooth_evidence", TESTS))
