"""Tests for the opt-in smooth-side evidence scale (issue #52, step 2).

Run:  python game/bin/x64/unit_test/lmoptimizer/test_smooth_evidence.py
  or: pytest game/bin/x64/unit_test/lmoptimizer
Exit code 0 = all pass.

  A. coarsest_reproducible_scale: the coarsest scale at which a side's whole
     baseline lightmap is reproduced within a display-space tolerance.
  B. solve_with_vbsp(evidence_scales=...): the evidence is a floor for a
     side that is neither uniform, gradient nor emissive: the minimum of the
     raw evidence over its coplanar group, then capped at its max useful
     scale (1 = no floor). Only a floor above the baseline blocks promotion;
     a lower one only stops a promotion short of scale 1.
  C. lmoptimizer leaves never-visible sides out of the evidence.
"""
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
from lightmap_evidence import display, recon_error, coarsest_reproducible_scale
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
    # Bilinear reconstruction is exact on a ramp in the space the tolerance is
    # measured in. A ramp in LINEAR luminance bends in display space (gamma),
    # so the same endpoints do not reach the coarsest factor.
    ramp = display_grid(lambda x, y: 20.0 + (200.0 - 20.0) * x / (SIZE - 1))
    got = coarsest_reproducible_scale(side(sub_face(ramp)), 1.0)
    check("a display-space ramp 20..200 at scale 2 -> 2 * 32", got == 64, f"got {got}")

    ys, xs = np.mgrid[0:SIZE, 0:SIZE]
    lum_ramp = 20.0 + (200.0 - 20.0) * xs / (SIZE - 1)
    got = coarsest_reproducible_scale(side(sub_face(lum_ramp)), 1.0)
    check("a linear-luminance ramp 20..200 does not reach the coarsest factor",
          got < 64, f"got {got}")


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
    # Linear interpolation of a*x^2 at node spacing k errs by a*k^2/4 at the
    # midpoint; a = 0.25 per axis gives 0.5 at k=2 and 2.0 at k=4 (x and y add).
    bump = display_grid(lambda x, y: 100.0 + 0.25 * ((x - 8.0) ** 2 + (y - 8.0) ** 2))
    disp = display(bump)
    e2, e4 = float(recon_error(disp, 2).max()), float(recon_error(disp, 4).max())
    check("fixture: k=2 error passes and k=4 error fails tolerance 1.0",
          e2 <= 1.0 < e4, f"k=2 {e2:.3f}  k=4 {e4:.3f}")
    got = coarsest_reproducible_scale(side(sub_face(bump)), 1.0)
    check("a smooth bump at scale 2 -> 2 * 2", got == 4, f"got {got}")
    got = coarsest_reproducible_scale(side(sub_face(bump)), 4.0)
    check("the same bump passes k=4 at a looser tolerance",
          got == 8, f"got {got}")


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
            gradient_tolerance=None,
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
                detail_min_scale=5, gradient_tolerance=None)
        check("no evidence: both sides at 1, nothing evidence-raised",
              result.scales == {1: 1, 2: 1} and result.faces_evidence_raised == 0,
              f"got {result.scales} raised {result.faces_evidence_raised}")


def test_strict_mode_still_takes_the_group_minimum():
    """--strict-coplanar passes no promotion groups; the evidence minimum must
    still run over the planes (evidence_groups) so one plane keeps one density."""
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
            gradient_tolerance=None, coplanar_groups=[[1, 2]],
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
        result = _solve_vacuous(td, 2500, evidence={1: 16, 2: 8, 3: 3})
        s = result.scales
        check("a floor equal to the baseline: side 3 stays at 3, not promoted",
              result.baseline_scale == 3 and s == {1: 8, 2: 1, 3: 3, 4: 3}
              and (result.faces_promoted, result.faces_at_baseline) == (1, 1),
              f"baseline {result.baseline_scale} scales {s} promoted/at baseline "
              f"{result.faces_promoted}/{result.faces_at_baseline}")
        check("...and it is not counted as raised",
              result.faces_evidence_raised == 1, f"got {result.faces_evidence_raised}")


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
    test_the_largest_passing_factor_wins_even_if_a_smaller_one_fails,
    test_every_sub_face_must_reproduce_and_the_finest_scale_is_the_base,
    test_missing_or_malformed_data_is_no_evidence,
    test_evidence_scales_keeps_only_sides_with_evidence,
    test_solver_floors_sides_at_their_evidence_and_does_not_promote_them,
    test_solver_without_evidence_is_unchanged,
    test_strict_mode_still_takes_the_group_minimum,
    test_floors_that_change_nothing_do_not_block_promotion,
    test_never_visible_sides_give_no_evidence,
]


if __name__ == '__main__':
    _sys.exit(run_all("test_smooth_evidence", TESTS))
