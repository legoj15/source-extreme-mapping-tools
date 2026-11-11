"""Regression tests for the geometry winding builder and extent clamps.

Run:  python game/bin/x64/unit_test/lmoptimizer/test_geometry_extents.py
  or: pytest game/bin/x64/unit_test/lmoptimizer
Exit code 0 = all pass.

Covers:
  1. _make_base_winding must emit a simple (non-self-intersecting) quad ring.
     The old order (+,-) (+,+) (-,-) (-,+) was a bowtie; clipping it by the
     brush's other planes collapsed z-normal faces (floors/ceilings) into
     degenerate slivers, which zeroed their lightmap extents and let huge
     displacements reach VBSP at scales that trip its fatal
     "Bad surface extents" check (bsplib.cpp CalcFaceExtents).
  2. min_scale_for_valid_extents must include VBSP's floor/ceil rounding
     slack: VBSP errors when ceil(maxs) - floor(mins) > 126, and the
     rounding can add +1 over the real span, so the span itself must be
     kept <= 125 luxels.
"""
import sys
import types
from unittest import mock

import os as _os
import sys as _sys
# python -I drops the script's folder from sys.path; _testkit lives beside this file.
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))

from _testkit import MINI_VMF, check, run_all

from vmf_parser import VMFBrush, VMFSide, KVNode, KVPair
from geometry import (
    _make_base_winding, build_brush_faces, Plane,
)
from vertex_estimator import (
    FaceExtent, compute_face_extent, min_scale_for_valid_extents,
)
import geometry
import lmoptimizer


def make_box_side(side_id, p0, p1, p2, material="NATURE/GRASS",
                  lightmapscale=16, uaxis="[1 0 0 0] 0.25",
                  vaxis="[0 1 0 0] 0.25"):
    return VMFSide(
        id=side_id,
        plane_points=[p0, p1, p2],
        material=material,
        lightmapscale=lightmapscale,
        uaxis=uaxis,
        vaxis=vaxis,
    )


def axis_box_brush(brush_id=1, x0=-5312.0, x1=-4224.0, y0=-3712.0,
                   y1=-3248.0, z0=-16.0, z1=0.0):
    """An axis-aligned box with Hammer-style side windings, including the
    'reversed' top/bottom winding order seen in real maps (inward normals),
    which is what triggered the bowtie collapse."""
    sides = [
        # top (z=z1), winding that yields a -z plane normal
        make_box_side(1, (x1, y1, z1), (x1, y0, z1), (x0, y0, z1)),
        # bottom (z=z0), winding that yields a +z plane normal
        make_box_side(2, (x0, y0, z0), (x1, y0, z0), (x1, y1, z0)),
        # west (x=x0)
        make_box_side(3, (x0, y1, z1), (x0, y0, z1), (x0, y0, z0)),
        # east (x=x1)
        make_box_side(4, (x1, y1, z0), (x1, y0, z0), (x1, y0, z1)),
        # north (y=y1)
        make_box_side(5, (x0, y1, z0), (x1, y1, z0), (x1, y1, z1)),
        # south (y=y0)
        make_box_side(6, (x0, y0, z1), (x1, y0, z1), (x1, y0, z0)),
    ]
    return VMFBrush(id=brush_id, sides=sides)


def winding_is_simple_ring(verts, eps=1e-4):
    """True if verts form a non-self-intersecting convex quad."""
    if len(verts) != 4:
        return False
    uniq = {tuple(round(c, 4) for c in v) for v in verts}
    if len(uniq) != 4:
        return False
    # For a proper ring, the two diagonals intersect; for a bowtie the
    # edges intersect. Easier convex check: all cross products of
    # consecutive edges share a sign.
    crosses = []
    for i in range(4):
        a = verts[i]
        b = verts[(i + 1) % 4]
        c = verts[(i + 2) % 4]
        e1 = tuple(b[k] - a[k] for k in range(3))
        e2 = tuple(c[k] - b[k] for k in range(3))
        cr = (e1[1] * e2[2] - e1[2] * e2[1],
              e1[2] * e2[0] - e1[0] * e2[2],
              e1[0] * e2[1] - e1[1] * e2[0])
        crosses.append(cr)
    # Dominant axis of the plane normal
    n = crosses[0]
    ax = max(range(3), key=lambda k: abs(n[k]))
    vals = [c[ax] for c in crosses]
    return all(v > eps for v in vals) or all(v < -eps for v in vals)


def test_base_winding_is_simple_ring():
    print("[1] _make_base_winding emits a simple quad ring")
    for normal, dist in [((0, 0, -1), 0.0), ((0, 0, 1), -16.0),
                         ((0, 1, 0), -3712.0), ((1, 0, 0), -5312.0),
                         ((0.6, 0.8, 0.0), 5.0)]:
        plane = Plane(normal=normal, dist=dist)
        w = _make_base_winding(plane, size=65536.0)
        check(f"simple ring for normal {normal}",
              winding_is_simple_ring(w),
              f"got {[tuple(round(c, 1) for c in v) for v in w]}")


def test_box_faces_not_degenerate():
    print("[2] axis box faces survive CSG clipping")
    brush = axis_box_brush()
    faces = build_brush_faces(brush)
    check("all 6 faces built", len(faces) == 6, f"got {len(faces)}")
    for f in faces:
        uniq = {tuple(round(c, 3) for c in v) for v in f.vertices}
        check(f"side {f.side_id} has 4 unique verts",
              len(uniq) == 4,
              f"got {len(uniq)}: {[tuple(round(c,1) for c in v) for v in f.vertices]}")
        check(f"side {f.side_id} has real area",
              f.area > 1.0, f"area={f.area}")
    # The top face (side 1) must span the full 1088 x 464 footprint
    top = next(f for f in faces if f.side_id == 1)
    xs = [v[0] for v in top.vertices]
    ys = [v[1] for v in top.vertices]
    check("top face spans x (-5312..-4224)",
          abs(min(xs) - (-5312.0)) < 0.01 and abs(max(xs) - (-4224.0)) < 0.01,
          f"x range {min(xs)}..{max(xs)}")
    check("top face spans y (-3712..-3248)",
          abs(min(ys) - (-3712.0)) < 0.01 and abs(max(ys) - (-3248.0)) < 0.01,
          f"y range {min(ys)}..{max(ys)}")


def test_displacement_extent_clamp():
    print("[3] displacement extent clamp reproduces VBSP luxel math")
    # 1088-unit span along uaxis [1 0 0]: VBSP size at scale S is
    # ceil(maxs)-floor(mins) which reaches 219 at S=5 (the observed crash).
    verts = [(-4224.0, -3248.0, 0.0), (-4224.0, -3712.0, 0.0),
             (-5312.0, -3712.0, 0.0), (-5312.0, -3248.0, 0.0)]
    fe = compute_face_extent(
        verts, "[1 0 0 0] 0.25", "[0 1 0 -64] 0.25",
        lightmapscale=5, side_id=99454, material="NATURE/RED_GRASS",
        has_displacement=True)
    check("extent_s is the full 1088 units",
          abs(fe.extent_s - 1088.0) < 0.01, f"extent_s={fe.extent_s}")
    check("extent_t is the full 464 units",
          abs(fe.extent_t - 464.0) < 0.01, f"extent_t={fe.extent_t}")
    min_s = min_scale_for_valid_extents(fe.extent_s, fe.extent_t)
    check("min scale for 1088-unit disp is 9", min_s == 9, f"got {min_s}")
    # Discriminating case: 1000 units needs 9 under the 124 budget but only
    # 8 under a naive 126 divisor (regression teeth for the float32 margin).
    check("min scale for 1000-unit span is 9 (not 8)",
          min_scale_for_valid_extents(1000.0, 1000.0) == 9,
          f"got {min_scale_for_valid_extents(1000.0, 1000.0)}")
    # Raw (unnormalized) axes scale the span exactly like VBSP's projection
    fe2 = compute_face_extent(
        verts, "[2 0 0 0] 0.25", "[0 1 0 -64] 0.25",
        lightmapscale=5, side_id=99455, material="NATURE/RED_GRASS",
        has_displacement=True)
    check("non-unit uaxis doubles the span",
          abs(fe2.extent_s - 2176.0) < 0.02, f"extent_s={fe2.extent_s}")


def test_rounding_edge_cases():
    print("[4] the safe span budget is exactly 124 luxels")
    # 124 = VBSP's fatal 126 minus ~2 luxels of endpoint floor/ceil
    # inflation, with float32 projection wobble absorbed by the same margin.
    # The span sitting right at the budget must keep scale 1; a hair over
    # must step to 2. (A budget of 125/126 or 123 moves these boundaries.)
    for span, expected in [(124.0, 1), (124.001, 2), (248.0, 2), (248.001, 3),
                           (1.0, 1), (0.0, 1)]:
        got = min_scale_for_valid_extents(span, span)
        check(f"span {span} -> scale {expected}", got == expected, f"got {got}")
    check("the larger of the two axes decides",
          min_scale_for_valid_extents(10.0, 500.0) == 5)


def test_ring_orientation_matches_normal():
    print("[5] base winding is CCW around the plane normal")
    for normal in [(0, 0, -1), (0, 0, 1), (0, 1, 0), (1, 0, 0),
                   (0.6, 0.8, 0.0)]:
        plane = Plane(normal=normal, dist=5.0)
        w = _make_base_winding(plane, size=1024.0)
        # Newell normal of the ring
        n = [0.0, 0.0, 0.0]
        for i in range(len(w)):
            a, b = w[i], w[(i + 1) % len(w)]
            n[0] += (a[1] - b[1]) * (a[2] + b[2])
            n[1] += (a[2] - b[2]) * (a[0] + b[0])
            n[2] += (a[0] - b[0]) * (a[1] + b[1])
        dot = sum(n[k] * normal[k] for k in range(3))
        check(f"ring normal aligns with plane normal {normal}",
              dot > 0, f"dot={dot}")


class _FakeArgs:
    detail_min_scale = 5


def test_eligible_min_scales_and_baseline_application():
    print("[6] eligible min-scales and baseline application round-trip")
    from vmf_parser import VMFParser
    from pathlib import Path
    root = VMFParser().parse_string(MINI_VMF)
    args = _FakeArgs()
    game_dir = Path("C:/nonexistent_game_dir")  # no %detailtype VMTs found

    eligible, skipped, side_map, disp_floor = \
        lmoptimizer._compute_eligible_min_scales(root, args, game_dir)

    # Only the RED_GRASS displacement is eligible; the 5 nodraw sides skip.
    check("1 eligible face", len(eligible) == 1, f"got {eligible}")
    check("5 skipped faces", skipped == 5, f"got {skipped}")
    check("no skipped displacements in this map", disp_floor == [],
          f"got {disp_floor}")
    sid, min_s = eligible[0]
    check("displacement side id is 1", sid == 1, f"got {sid}")
    check("displacement min scale is 9 (extent clamp, no detail vmts)",
          min_s == 9, f"got {min_s}")

    lmoptimizer._apply_baseline_scales(side_map, eligible, baseline=2)
    node = side_map[1]
    check("applied scale is max(2, 9) = 9",
          node.get_property('lightmapscale') == '9',
          f"got {node.get_property('lightmapscale')}")

    # Nodraw sides must be untouched
    check("nodraw side keeps its original scale",
          side_map[2].get_property('lightmapscale') == '16',
          f"got {side_map[2].get_property('lightmapscale')}")


def test_skipped_displacement_still_gets_extent_floor():
    print("[7] skipped-entity/tool displacements still get the fatal-limit floor")
    from vmf_parser import VMFParser
    from pathlib import Path
    # Same map, but the displacement side uses a tool material so it is
    # skipped by should_skip_face — VBSP's extent check still applies to it.
    root = VMFParser().parse_string(
        MINI_VMF.replace("NATURE/RED_GRASS", "TOOLS/TOOLSNODRAW"))
    args = _FakeArgs()
    game_dir = Path("C:/nonexistent_game_dir")

    eligible, skipped, side_map, disp_floor = \
        lmoptimizer._compute_eligible_min_scales(root, args, game_dir)
    check("no eligible faces (all tools)", eligible == [], f"got {eligible}")
    check("skipped displacement captured in disp_floor",
          disp_floor == [(1, 9)], f"got {disp_floor}")

    # Drop side 1's scale below the floor, then raise-only must fix it
    side_map[1].set_property('lightmapscale', '4')
    lmoptimizer._apply_min_floor_scales(side_map, disp_floor)
    check("side 1 raised from 4 to 9",
          side_map[1].get_property('lightmapscale') == '9',
          f"got {side_map[1].get_property('lightmapscale')}")

    # Above the floor it must stay untouched (raise-only)
    side_map[1].set_property('lightmapscale', '16')
    lmoptimizer._apply_min_floor_scales(side_map, disp_floor)
    check("side 1 at 16 stays 16",
          side_map[1].get_property('lightmapscale') == '16',
          f"got {side_map[1].get_property('lightmapscale')}")


def _extent(side_id, material='CONCRETE/FLOOR', ext_s=64.0, ext_t=64.0,
            has_disp=False, brush_entity=''):
    return FaceExtent(side_id=side_id, extent_s=ext_s, extent_t=ext_t,
                      s_min=0.0, s_max=ext_s, t_min=0.0, t_max=ext_t,
                      s_axis=(1, 0, 0), t_axis=(0, 1, 0), lightmapscale=1,
                      material=material, num_vertices=4,
                      brush_entity=brush_entity, has_displacement=has_disp)


def _flat_side(side_id, material='CONCRETE/FLOOR', scale='1'):
    return KVNode(name='side', children=[
        KVPair('id', str(side_id)),
        KVPair('plane', '(0 0 0) (64 0 0) (64 64 0)'),
        KVPair('material', material),
        KVPair('lightmapscale', scale),
    ])


def test_unify_never_drops_displacement_below_floor():
    """Protects: the calibration-fallback path (no --game, e.g. the documented
    `--bsp map.bsp --dry-run`) must not output a displacement below its VBSP
    extent floor. Unification used to drag it to a flat coplanar neighbour's
    scale 1, which fatals 'Bad surface extents' at the user's next compile."""
    print("[8] coplanar unification respects per-side floors")
    side_map = {1: _flat_side(1, 'NATURE/RED_GRASS', '9'), 2: _flat_side(2, scale='1')}
    extents = [_extent(1, 'NATURE/RED_GRASS', 1088.0, 464.0, has_disp=True),
               _extent(2)]
    face_vertices = {  # touching at x = 1088
        1: [(0, 0, 0), (1088, 0, 0), (1088, 464, 0), (0, 464, 0)],
        2: [(1088, 0, 0), (1200, 0, 0), (1200, 464, 0), (1088, 464, 0)],
    }
    floors = lmoptimizer._compute_side_floors(extents, set(), 5)
    check("displacement floor computed (1088 units -> 9)", floors == {1: 9},
          f"got {floors}")

    scales = {1: 9, 2: 1}
    lmoptimizer._unify_coplanar_groups(
        scales, side_map, face_data={}, face_extents=extents,
        face_vertices=face_vertices, min_scales=floors)
    check("displacement keeps its floor through unification", scales[1] == 9,
          f"got {scales}")
    check("the flat neighbour keeps the fine scale", scales[2] == 1,
          f"got {scales}")


def test_coplanar_groups_only_link_visible_seams():
    """Protects: promotion groups tie two sides only where a seam would show
    (shared edge, same material and texture axes, detail along the edge on
    both). Plane + bounding-box grouping chained every z=0 floor of
    d1_trainstation_02 into one 167-face unit that could never be promoted
    (issue #74); reverting to it links all five sides below."""
    print("[8b] coplanar groups link only seam-relevant neighbours")
    from bsp_reader import FaceLightmapData, BSPSubFaceInfo

    def side(sid, material='CONCRETE/FLOOR'):
        return KVNode(name='side', children=[
            KVPair('id', str(sid)),
            KVPair('plane', '(0 0 0) (0 64 0) (64 64 0)'),
            KVPair('material', material),
            KVPair('uaxis', '[1 0 0 0] 0.25'),
            KVPair('vaxis', '[0 -1 0 0] 0.25'),
        ])

    def lighting(sid, x0, y0, lum, sw=64, sh=64):
        # scale 1: one luxel per unit, s = x - x0, t = y - y0, (sw+1) x (sh+1)
        lums = [lum(x0 + i, y0 + j) for j in range(sh + 1) for i in range(sw + 1)]
        sf = BSPSubFaceInfo(face_index=sid, lightmap_mins=(0, 0),
                            lightmap_size=(sw, sh),
                            lightmap_vecs=((1, 0, 0, -x0), (0, 1, 0, -y0)),
                            luminances=lums, is_uniform=False)
        return FaceLightmapData(vmf_side_id=sid, material='', bsp_face_indices=[sid],
                                luminances=list(lums), sub_faces=[sf])

    def stripes(x, y):  # shadow bars running across x: vary along y
        return 200.0 if (y // 8) % 2 else 20.0

    def stripes_left(x, y):  # bars stop short of x = 128
        return stripes(x, y) if x < 100 else 120.0

    def square(x0, y0):
        return [(x0, y0, 0), (x0 + 64, y0, 0), (x0 + 64, y0 + 64, 0), (x0, y0 + 64, 0)]

    # A [0,64] and B [64,128] share x=64 with bars crossing it -> linked.
    # B and C [128,192] share x=128, flat on B's side -> not linked.
    # E [-64,0] shares x=0 with A, bars on both, other material -> not linked.
    # F [-64,0]x[64,128] only touches A's corner -> not linked.
    layout = {1: ((0, 0), stripes, 'CONCRETE/FLOOR'),
              2: ((64, 0), stripes_left, 'CONCRETE/FLOOR'),
              3: ((128, 0), lambda x, y: 120.0, 'CONCRETE/FLOOR'),
              5: ((-64, 0), stripes, 'CONCRETE/OTHER'),
              6: ((-64, 64), stripes, 'CONCRETE/FLOOR')}
    side_map = {sid: side(sid, mat) for sid, (_o, _l, mat) in layout.items()}
    face_vertices = {sid: square(*o) for sid, (o, _l, _m) in layout.items()}
    light = {sid: lighting(sid, o[0], o[1], lum) for sid, (o, lum, _m) in layout.items()}
    extents = [_extent(sid, mat) for sid, (_o, _l, mat) in layout.items()]

    groups = lmoptimizer._compute_coplanar_groups(
        side_map, face_vertices, extents, lighting_data=light)
    check("only A and B form a group", sorted(sorted(g) for g in groups) == [[1, 2]],
          f"got {groups}")
    check("no lighting data links nothing",
          lmoptimizer._compute_coplanar_groups(side_map, face_vertices, extents) == [])

    # B's lighting unknown (no BSP data): unknown is not flat, so A's bars
    # crossing their edge still tie them; a never-visible A ties nothing.
    unknown_b = {sid: ld for sid, ld in light.items() if sid != 2}
    groups = lmoptimizer._compute_coplanar_groups(
        side_map, face_vertices, extents, lighting_data=unknown_b)
    check("an unknown side next to detail is linked",
          sorted(sorted(g) for g in groups) == [[1, 2]], f"got {groups}")
    empty_b = dict(light)
    empty_b[2] = FaceLightmapData(vmf_side_id=2, material='', bsp_face_indices=[2],
                                  luminances=[], sub_faces=[])
    groups = lmoptimizer._compute_coplanar_groups(
        side_map, face_vertices, extents, lighting_data=empty_b)
    check("a side with no samples is unknown too",
          sorted(sorted(g) for g in groups) == [[1, 2]], f"got {groups}")
    # A stand-in entry (carved dark piece or culled side: luminances [0.0],
    # no sub-faces) is known flat: B can no longer tie A to anything.
    stand_in_b = dict(light)
    stand_in_b[2] = FaceLightmapData(vmf_side_id=2, material='', bsp_face_indices=[],
                                     luminances=[0.0])
    groups = lmoptimizer._compute_coplanar_groups(
        side_map, face_vertices, extents, lighting_data=stand_in_b)
    check("a stand-in uniform entry is flat, not unknown", groups == [],
          f"got {groups}")
    groups = lmoptimizer._compute_coplanar_groups(
        side_map, face_vertices, extents, lighting_data=light, never_visible={1})
    check("a never-visible side links to nothing", groups == [], f"got {groups}")

    # P [0,100]x[0,64] over Q, whose y=0 edge is split by a collinear vertex at
    # x=50; the bars (varying along x) stop at x=44, on the piece Q lists last.
    def bars_left(x, y):
        return 120.0 if x >= 44 else (200.0 if (x // 8) % 2 else 20.0)
    side_map2 = {7: side(7), 8: side(8)}
    verts2 = {7: [(0, 0, 0), (100, 0, 0), (100, 64, 0), (0, 64, 0)],
              8: [(0, 0, 0), (0, -64, 0), (100, -64, 0), (100, 0, 0), (50, 0, 0)]}
    light2 = {7: lighting(7, 0, 0, bars_left, sw=100),
              8: lighting(8, 0, -64, bars_left, sw=100)}
    groups = lmoptimizer._compute_coplanar_groups(
        side_map2, verts2, [_extent(7), _extent(8)], lighting_data=light2)
    check("detail on any piece of a split shared edge links",
          sorted(sorted(g) for g in groups) == [[7, 8]], f"got {groups}")

    # A one-luxel bar on an odd luxel, with the lattice half a luxel off the
    # world grid: every 1-unit sample then lands on a luxel border, and a
    # fixed 1-unit stride with round-half-to-even visits only even luxels.
    def half_phase(sid, x0, y0):
        lums = [200.0 if j == 33 else 120.0 for j in range(65) for i in range(65)]
        sf = BSPSubFaceInfo(face_index=sid, lightmap_mins=(0, 0),
                            lightmap_size=(64, 64),
                            lightmap_vecs=((1, 0, 0, -x0 + 0.5), (0, 1, 0, -y0 + 0.5)),
                            luminances=lums, is_uniform=False)
        return FaceLightmapData(vmf_side_id=sid, material='', bsp_face_indices=[sid],
                                luminances=list(lums), sub_faces=[sf])
    light3 = {1: half_phase(1, 0, 0), 2: half_phase(2, 64, 0)}
    groups = lmoptimizer._compute_coplanar_groups(
        {1: side(1), 2: side(2)}, {1: square(0, 0), 2: square(64, 0)},
        [_extent(1), _extent(2)], lighting_data=light3)
    check("a one-luxel bar on an odd luxel is seen", sorted(sorted(g) for g in groups) == [[1, 2]],
          f"got {groups}")


def test_scale_floors_are_raise_only():
    print("[9] side floors: detail, displacement and texlight minimums")
    texlights = {'LIGHTS/WHITE001'}
    extents = [
        _extent(1, 'NATURE/RED_GRASS', 1088.0, 464.0, has_disp=True),  # disp -> 9
        _extent(2, 'TOOLS/TOOLSNODRAW', 1088.0, 464.0, has_disp=True),  # skipped disp -> 9
        _extent(3, 'CONCRETE/DETAIL01'),                                # %detailtype -> 5
        _extent(4, 'lights/white001'),                                  # texlight -> 8
        _extent(5, 'lights/white001', brush_entity='trigger_multiple'),  # skipped: no floor
        _extent(6),                                                      # plain
    ]
    floors = lmoptimizer._compute_side_floors(
        extents, {'concrete/detail01'}, 5, texlight_mats=texlights,
        emissive_floor=8)
    check("floors computed per rule", floors == {1: 9, 2: 9, 3: 5, 4: 8},
          f"got {floors}")
    check("no emissive floor requested -> texlights unconstrained",
          4 not in lmoptimizer._compute_side_floors(
              extents, set(), 5, texlight_mats=texlights, emissive_floor=1))

    side_map = {sid: _flat_side(sid, scale=str(v))
                for sid, v in {1: 1, 2: 4, 3: 1, 4: 16, 6: 1}.items()}
    scales = {1: 1, 3: 1, 4: 16}          # side 2 was skipped by the solver
    raised = lmoptimizer._apply_scale_floors(scales, side_map, floors)
    check("solver sides raised to their floor, never lowered",
          scales == {1: 9, 3: 5, 4: 16}, f"got {scales}")
    check("raised sides are written to the tree",
          side_map[1].get_property('lightmapscale') == '9'
          and side_map[3].get_property('lightmapscale') == '5')
    check("a face above its floor keeps its scale",
          side_map[4].get_property('lightmapscale') == '16')
    check("a skipped displacement is raised in the tree",
          side_map[2].get_property('lightmapscale') == '9')
    check("raise count", raised == 3, f"got {raised}")

    side_map2 = {1: _flat_side(1, scale='1')}
    scales2 = {1: 1}
    lmoptimizer._apply_scale_floors(scales2, side_map2, {1: 9}, write_nodes=False)
    check("write_nodes=False updates only the dict",
          scales2 == {1: 9} and side_map2[1].get_property('lightmapscale') == '1')


def test_vanished_displacement_is_reported_and_floored():
    """Protects: a displacement whose CSG winding collapses (the bowtie bug's
    failure mode) no longer vanishes silently with no extent clamp at all; it
    is reported and held at a conservative scale floor."""
    print("[10] displacements without usable geometry")
    from vmf_parser import VMFParser
    from pathlib import Path
    root = VMFParser().parse_string(MINI_VMF)
    args = _FakeArgs()
    with mock.patch.object(geometry, 'build_all_faces', return_value=[]):
        eligible, skipped, side_map, disp_floor =             lmoptimizer._compute_eligible_min_scales(
                root, args, Path("C:/nonexistent_game_dir"))
    check("the vanished displacement gets the conservative floor",
          disp_floor == [(1, lmoptimizer._DISP_NO_GEOMETRY_FLOOR)],
          f"got {disp_floor}")

    def side(sid, disp):
        children = [KVPair('id', str(sid))]
        if disp:
            children.append(KVNode(name='dispinfo'))
        return KVNode(name='side', children=children)

    quad = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)]
    tri = quad[:3]
    faces = [types.SimpleNamespace(side_id=1, vertices=quad),
             types.SimpleNamespace(side_id=3, vertices=tri)]
    brushes = [types.SimpleNamespace(sides=[1, 2, 3, 4])]
    side_map = {1: side(1, True), 2: side(2, True), 3: side(3, True), 4: side(4, False)}
    bad = lmoptimizer._report_geometry_gaps(brushes, faces, side_map)
    check("displacements with no polygon or a non-quad base are returned",
          sorted(bad) == [2, 3], f"got {sorted(bad)}")

    # The scale report must answer for such a side too: it has no face extent
    # and the solver never saw it, yet the floor raised it in the tree (an
    # explicit 8 became 16). A side with no extent that nothing touched stays out.
    vanished = KVNode(name='side', children=[
        KVPair('id', '2'), KVPair('material', 'NATURE/RED_GRASS'),
        KVPair('lightmapscale', '8'), KVNode(name='dispinfo')])
    untouched = KVNode(name='side', children=[
        KVPair('id', '5'), KVPair('material', 'CONCRETE/WALL'),
        KVPair('lightmapscale', '16')])
    tree = KVNode(name='__root__', children=[KVNode(name='world', children=[
        KVNode(name='solid', children=[KVPair('id', '100'), vanished, untouched])])])
    floors = {2: lmoptimizer._DISP_NO_GEOMETRY_FLOOR}
    raised = set()
    lmoptimizer._apply_scale_floors({}, lmoptimizer._build_vmf_side_map(tree), floors,
                                    raised_ids=raised)
    report = lmoptimizer._build_scale_report(
        tree, {'report_data': {'face_extents': [], 'side_floors': floors,
                               'floor_kinds': {2: ['disp_no_geometry']},
                               'stage_changes': {'floors': raised}}},
        _FakeArgs(), Path('in.vmf'), Path('out.vmf'), True,
        {2: 8, 5: 16},     # the tree as parsed
        {2: 16, 5: 16})    # the tree when the light-budget stage began: floors done
    rec = report['sides'].get(2, {})
    check("a displacement with no polygon is in the report: input 8, final 16, "
          "floor 16 of kind disp_no_geometry, flagged no_geometry, moved by floors, "
          "no solver decision",
          (rec.get('input_scale'), rec.get('final_scale'), rec.get('floor'),
           rec.get('floor_kinds'), rec.get('flags'), rec.get('changed_by'))
          == (8, 16, 16, ['disp_no_geometry'], {'no_geometry': True}, ['floors'])
          and rec.get('solver_scale') is None and 'decision' not in rec,
          f"got {rec}")
    check("...and a side with no extent that nothing touched is not listed",
          5 not in report['sides'], f"got {sorted(report['sides'])}")

    # --vis-debug repaints the tree before the report is built; the record must
    # still carry the map's own material, not the proxy texture.
    vis_result = {'raw_nv_sides': {2}, 'report_data': {
        'face_extents': [], 'side_floors': floors,
        'floor_kinds': {2: ['disp_no_geometry']}, 'stage_changes': {'floors': raised}}}
    lmoptimizer._apply_vis_debug(tree, vis_result)
    report = lmoptimizer._build_scale_report(
        tree, vis_result, _FakeArgs(), Path('in.vmf'), Path('out.vmf'), True,
        {2: 8, 5: 16}, {2: 16, 5: 16})
    check("under --vis-debug a no-extent record keeps the map's material",
          report['sides'].get(2, {}).get('material') == 'NATURE/RED_GRASS',
          f"got {report['sides'].get(2)}")

    # Moved with no tracked stage and not by the solver means auto-compile
    # raised it up front: a skipped (tool) displacement that has an extent, and
    # a carved piece with no extent, which inherits its original side's input.
    skipped_disp = KVNode(name='side', children=[
        KVPair('id', '6'), KVPair('material', 'TOOLS/TOOLSSKYBOX'),
        KVPair('lightmapscale', '9'), KVNode(name='dispinfo')])
    piece = KVNode(name='side', children=[
        KVPair('id', '31'), KVPair('material', 'CONCRETE/WALL'),
        KVPair('lightmapscale', '16')])
    tree = KVNode(name='__root__', children=[KVNode(name='world', children=[
        KVNode(name='solid', children=[KVPair('id', '101'), skipped_disp, piece])])])
    extent = types.SimpleNamespace(side_id=6, material='TOOLS/TOOLSSKYBOX',
                                   extent_s=64.0, extent_t=64.0, brush_entity='',
                                   has_displacement=True)
    report = lmoptimizer._build_scale_report(
        tree, {'report_data': {'face_extents': [extent], 'side_floors': {6: 9},
                               'floor_kinds': {6: ['displacement']},
                               'side_remap': {30: [31]}, 'stage_changes': {}}},
        _FakeArgs(), Path('in.vmf'), Path('out.vmf'), True,
        {6: 4, 30: 4}, {6: 9, 31: 16})
    disp, carved = report['sides'].get(6, {}), report['sides'].get(31, {})
    check("a skipped displacement raised before the solver is attributed to it",
          (disp.get('decision'), disp.get('changed_by')) == ('skipped', ['before_solver']),
          f"got {disp}")
    check("a carved piece with no extent inherits its original side's input scale",
          (carved.get('input_scale'), (carved.get('flags') or {}).get('carved_from'),
           carved.get('changed_by')) == (4, 30, ['before_solver']),
          f"got {carved}")


TESTS = [
    test_base_winding_is_simple_ring,
    test_box_faces_not_degenerate,
    test_displacement_extent_clamp,
    test_rounding_edge_cases,
    test_ring_orientation_matches_normal,
    test_eligible_min_scales_and_baseline_application,
    test_skipped_displacement_still_gets_extent_floor,
    test_unify_never_drops_displacement_below_floor,
    test_coplanar_groups_only_link_visible_seams,
    test_scale_floors_are_raise_only,
    test_vanished_displacement_is_reported_and_floored,
]


if __name__ == "__main__":
    sys.exit(run_all("test_geometry_extents", TESTS))
