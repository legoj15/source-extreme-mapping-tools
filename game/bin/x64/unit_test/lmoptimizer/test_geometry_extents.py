"""Regression tests for the geometry winding builder and extent clamps.

Run:  python game/bin/x64/unit_test/lmoptimizer/test_geometry_extents.py
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
import math
import sys
from pathlib import Path

# Make game/bin/x64 importable
BIN_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(BIN_DIR))

from vmf_parser import VMFBrush, VMFSide          # noqa: E402
from geometry import (                             # noqa: E402
    _make_base_winding, build_brush_faces, Plane,
)
from vertex_estimator import (                     # noqa: E402
    compute_face_extent, min_scale_for_valid_extents,
)
import lmoptimizer                                 # noqa: E402

FAILURES = []


def check(name, cond, detail=""):
    if cond:
        print(f"  ok    {name}")
    else:
        print(f"  FAIL  {name}  {detail}")
        FAILURES.append(name)


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
    # At the returned scale, worst-case VBSP ceil/floor size must be <= 126
    for span, name in [(fe.extent_s, "S"), (fe.extent_t, "T"),
                       (fe2.extent_s, "S-raw")]:
        s = min_scale_for_valid_extents(span, span)
        worst = math.ceil(span / s) + 1  # endpoint rounding can add ~2
        check(f"{name}: worst-case VBSP size {worst} <= 126 at scale {s}",
              worst <= 126, f"span={span}")


def test_rounding_edge_cases():
    print("[4] floor/ceil rounding edge cases stay under the fatal limit")
    for span in [251.0, 252.0, 126.0, 125.5, 124.0, 125.0,
                 1000.0, 1008.0, 1009.0]:
        s = min_scale_for_valid_extents(span, span)
        worst = math.ceil(span / s) + 1  # endpoint rounding can add ~2
        check(f"span {span} -> scale {s}, worst size {worst} <= 126",
              worst <= 126, "")


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


MINI_VMF = """versioninfo
{
"editorversion" "400"
"editorbuild" "8864"
"mapversion" "1"
"formatversion" "100"
"prefab" "0"
}
visgroups
{
}
world
{
"id" "1"
"mapversion" "1"
"classname" "worldspawn"
"skyname" "sky_day01_01"
solid
{
"id" "100"
side
{
"id" "1"
"plane" "(-4224 -3248 0) (-4224 -3712 0) (-5312 -3712 0)"
"material" "NATURE/RED_GRASS"
"uaxis" "[1 0 0 0] 0.25"
"vaxis" "[0 1 0 -64] 0.25"
"rotation" "0"
"lightmapscale" "16"
"smoothing_groups" "0"
dispinfo
{
"power" "3"
"startposition" "[-5312 -3712 0]"
"flags" "0"
"elevation" "0"
"subdiv" "0"
}
}
side
{
"id" "2"
"plane" "(-5312 -3712 -16) (-4224 -3712 -16) (-4224 -3248 -16)"
"material" "TOOLS/TOOLSNODRAW"
"uaxis" "[1 0 0 0] 0.25"
"vaxis" "[0 1 0 0] 0.25"
"rotation" "0"
"lightmapscale" "16"
"smoothing_groups" "0"
}
side
{
"id" "3"
"plane" "(-5312 -3248 0) (-5312 -3712 0) (-5312 -3712 -16)"
"material" "TOOLS/TOOLSNODRAW"
"uaxis" "[0 1 0 0] 0.25"
"vaxis" "[0 0 -1 0] 0.25"
"rotation" "0"
"lightmapscale" "16"
"smoothing_groups" "0"
}
side
{
"id" "4"
"plane" "(-4224 -3248 -16) (-4224 -3712 -16) (-4224 -3712 0)"
"material" "TOOLS/TOOLSNODRAW"
"uaxis" "[0 1 0 0] 0.25"
"vaxis" "[0 0 -1 0] 0.25"
"rotation" "0"
"lightmapscale" "16"
"smoothing_groups" "0"
}
side
{
"id" "5"
"plane" "(-5312 -3248 -16) (-4224 -3248 -16) (-4224 -3248 0)"
"material" "TOOLS/TOOLSNODRAW"
"uaxis" "[1 0 0 0] 0.25"
"vaxis" "[0 0 -1 0] 0.25"
"rotation" "0"
"lightmapscale" "16"
"smoothing_groups" "0"
}
side
{
"id" "6"
"plane" "(-5312 -3712 0) (-4224 -3712 0) (-4224 -3712 -16)"
"material" "TOOLS/TOOLSNODRAW"
"uaxis" "[1 0 0 0] 0.25"
"vaxis" "[0 0 -1 0] 0.25"
"rotation" "0"
"lightmapscale" "16"
"smoothing_groups" "0"
}
}
}
entity
{
"id" "200"
"classname" "info_player_start"
"origin" "-4768 -3480 24"
"angles" "0 0 0"
}
cameras
{
"activecamera" "0"
}
cordon
{
"mins" "(-8192 -8192 -8192)"
"maxs" "(8192 8192 8192)"
"active" "0"
}
"""


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


def main():
    test_base_winding_is_simple_ring()
    test_box_faces_not_degenerate()
    test_displacement_extent_clamp()
    test_rounding_edge_cases()
    test_ring_orientation_matches_normal()
    test_eligible_min_scales_and_baseline_application()
    test_skipped_displacement_still_gets_extent_floor()
    if FAILURES:
        print(f"\n{len(FAILURES)} FAILURE(S): {FAILURES}")
        return 1
    print("\nAll tests passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
