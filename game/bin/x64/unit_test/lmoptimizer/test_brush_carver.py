"""Brush carver: what gets promoted after a carve, and what keeps its data.

Run:  python game/bin/x64/unit_test/lmoptimizer/test_brush_carver.py
  or: pytest game/bin/x64/unit_test/lmoptimizer
Exit code 0 = all pass.

A carved piece is raised to max lightmapscale (it is told it is "uniformly
dark"). Promoting the wrong area flattens real lighting, so these pin:
  A. A brush written by stock Hammer (no vertices_plus) is still carved.
  B. Only the carved copy of the dark face is promoted, never the piece's
     other five sides, and only when the piece covers no lit sub-face.
  C. A piece that straddles lit and dark area is never promoted, and a carve
     that would promote nothing is not made at all.
  D. Every side of a carved brush that is not promoted maps back to its
     original side, so it keeps that side's lighting data.
"""
import os as _os
import sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
from _testkit import check, run_all  # noqa: E402

from bsp_reader import BSPSubFaceInfo  # noqa: E402
from brush_carver import apply_carves, find_carve_candidates  # noqa: E402
from vmf_parser import VMFParser, extract_brushes  # noqa: E402

TOP_SIDE_ID = 1

# A 512 x 512 x 16 box, sides as stock Hammer writes them: no vertices_plus.
_VMF = """
world
{
	"id" "100"
	"classname" "worldspawn"
	solid
	{
		"id" "200"
		side { "id" "1" "plane" "(0 512 16) (512 512 16) (512 0 16)" "material" "DEV/FLOOR" "uaxis" "[1 0 0 0] 0.25" "vaxis" "[0 -1 0 0] 0.25" "lightmapscale" "16" }
		side { "id" "2" "plane" "(0 0 0) (512 0 0) (512 512 0)" "material" "DEV/FLOOR" "uaxis" "[1 0 0 0] 0.25" "vaxis" "[0 -1 0 0] 0.25" "lightmapscale" "16" }
		side { "id" "3" "plane" "(0 512 16) (0 0 16) (0 0 0)" "material" "DEV/FLOOR" "uaxis" "[0 1 0 0] 0.25" "vaxis" "[0 0 -1 0] 0.25" "lightmapscale" "16" }
		side { "id" "4" "plane" "(512 512 0) (512 0 0) (512 0 16)" "material" "DEV/FLOOR" "uaxis" "[0 1 0 0] 0.25" "vaxis" "[0 0 -1 0] 0.25" "lightmapscale" "16" }
		side { "id" "5" "plane" "(512 512 16) (0 512 16) (0 512 0)" "material" "DEV/FLOOR" "uaxis" "[1 0 0 0] 0.25" "vaxis" "[0 0 -1 0] 0.25" "lightmapscale" "16" }
		side { "id" "6" "plane" "(512 0 0) (0 0 0) (0 0 16)" "material" "DEV/FLOOR" "uaxis" "[1 0 0 0] 0.25" "vaxis" "[0 0 -1 0] 0.25" "lightmapscale" "16" }
	}
}
"""


def _expand_braces(text):
    # The test VMF keeps each side on one line; the parser wants one token per line.
    out = []
    for line in text.splitlines():
        line = line.replace('{', '\n{\n').replace('}', '\n}\n')
        for part in line.split('\n'):
            part = part.strip()
            if not part:
                continue
            if part.startswith('side'):
                out.append('side')
                continue
            # split '"k" "v" "k" "v"' runs into one pair per line
            toks = part.split('" "')
            if len(toks) > 2:
                toks[0] = toks[0].lstrip('"')
                toks[-1] = toks[-1].rstrip('"')
                for i in range(0, len(toks) - 1, 2):
                    out.append(f'"{toks[i]}" "{toks[i + 1]}"')
            else:
                out.append(part)
    return '\n'.join(out)


def _sub(x0, x1, y0, y1, lum_lo, lum_hi, face_index=0):
    """A BSP sub-face of the top face covering [x0,x1] x [y0,y1], 16 u/luxel."""
    lums = [float(lum_lo)] * 4 if lum_lo == lum_hi else [float(lum_lo), float(lum_hi)] * 2
    return BSPSubFaceInfo(
        face_index=face_index,
        lightmap_mins=(int(x0 / 16), int(y0 / 16)),
        lightmap_size=(int((x1 - x0) / 16), int((y1 - y0) / 16)),
        lightmap_vecs=((1 / 16, 0.0, 0.0, 0.0), (0.0, 1 / 16, 0.0, 0.0)),
        luminances=lums,
        is_uniform=(lum_lo == lum_hi),
    )


class _LD:
    def __init__(self, subs):
        self.sub_faces = subs


def _carve(subs):
    root = VMFParser().parse_string(_expand_braces(_VMF))
    brushes = extract_brushes(root)
    cands = find_carve_candidates({TOP_SIDE_ID: _LD(subs)}, brushes)
    if not cands:
        return root, cands, [], set(), {}
    results, promoted, remap = apply_carves(root, cands, [])
    return root, cands, results, promoted, remap


def _pieces(root):
    return [b for b in extract_brushes(root)]


def _top_rect(brush):
    """(xmin, xmax, ymin, ymax) of a box brush from its planes."""
    xs, ys = [], []
    for s in brush.sides:
        pts = s.plane_points
        if len({p[0] for p in pts}) == 1:
            xs.append(pts[0][0])
        if len({p[1] for p in pts}) == 1:
            ys.append(pts[0][1])
    return min(xs), max(xs), min(ys), max(ys)


def test_clean_dark_corner_promotes_only_its_top_face():
    # Dark 128 x 128 corner; the rest of the floor is lit and varied.
    subs = [
        _sub(0, 128, 0, 128, 0, 0),
        _sub(128, 512, 0, 512, 100, 160),
        _sub(0, 128, 128, 512, 100, 160),
    ]
    root, cands, results, promoted, remap = _carve(subs)
    check("stock-Hammer brush (no vertices_plus) is a carve candidate", len(cands) == 1,
          f"got {len(cands)} candidates")
    check("the brush is carved", len(results) == 1, f"got {len(results)} carves")
    pieces = _pieces(root)
    # Cuts sit one luxel (16u) inside the dark area: VBSP rounds luxel
    # windows outward, so the recovered edge can be off by up to a luxel.
    dark = [b for b in pieces if _top_rect(b) == (0, 112, 0, 112)]
    check("a piece covers the dark corner, one luxel in from its edge",
          len(dark) == 1, f"rects: {[_top_rect(b) for b in pieces]}")
    if not dark:
        return
    top_ids = set()
    for s in dark[0].sides:
        pts = s.plane_points
        if len({p[2] for p in pts}) == 1 and pts[0][2] == 16:
            top_ids.add(s.id)
    check("only the dark piece's top face is promoted", promoted == top_ids,
          f"promoted={sorted(promoted)} top={sorted(top_ids)}")
    # Sides are written in order 1..6, and every piece keeps that order.
    wrong = [(i + 1, s.id) for b in pieces for i, s in enumerate(b.sides)
             if s.id not in remap.get(i + 1, [])]
    check("each new side maps back to the original side it was copied from",
          not wrong, f"(original, new) not mapped: {wrong}")


def test_checkerboard_dark_is_not_promoted_or_carved():
    # Two dark cells on a diagonal: their bounding box also holds two lit
    # cells, so the 0..128 piece straddles lit area. Its centre (64,64) sits
    # on a dark cell's corner, which is what the old centre test looked at.
    subs = [
        _sub(0, 64, 0, 64, 0, 0),
        _sub(64, 128, 64, 128, 0, 0),
        _sub(64, 128, 0, 64, 100, 160),
        _sub(0, 64, 64, 128, 100, 160),
        _sub(128, 512, 0, 512, 100, 160),
        _sub(0, 128, 128, 512, 100, 160),
    ]
    root, cands, results, promoted, remap = _carve(subs)
    check("no side of a piece that covers lit area is promoted", not promoted,
          f"promoted={sorted(promoted)}")
    check("a carve that promotes nothing is not made", not results,
          f"got {len(results)} carves")


def _promoted_rects(root, promoted):
    return [_top_rect(b) for b in _pieces(root)
            if any(s.id in promoted for s in b.sides)]


def _overlaps(rect, x0, x1, y0, y1):
    rx0, rx1, ry0, ry1 = rect
    return rx0 < x1 and rx1 > x0 and ry0 < y1 and ry1 > y0


def test_bright_strip_inside_dark_cluster_blocks_promotion():
    # A flat but BRIGHT strip (uniform, lum 160) between two dark bands: no
    # promoted piece may cover it, though either band alone may be promoted.
    subs = [
        _sub(0, 128, 0, 112, 0, 0),
        _sub(0, 128, 112, 144, 160, 160),
        _sub(0, 128, 144, 208, 0, 0),
        _sub(0, 128, 208, 512, 100, 160),
        _sub(128, 512, 0, 512, 100, 160),
    ]
    root, cands, results, promoted, remap = _carve(subs)
    rects = _promoted_rects(root, promoted)
    check("no promoted piece covers the bright-uniform strip or lit area",
          not any(_overlaps(r, 0, 128, 112, 144) or _overlaps(r, 0, 128, 208, 512)
                  or _overlaps(r, 128, 512, 0, 512) for r in rects),
          f"promoted rects={rects}")


def test_two_separate_shadows_on_one_axis_still_carve():
    # Two full-height dark stripes along X. Boundary cuts 64/128/256/320 exceed
    # the per-axis cap; keeping one whole stripe's pair must still carve it.
    subs = [
        _sub(0, 64, 0, 512, 100, 160),
        _sub(64, 128, 0, 512, 0, 0),
        _sub(128, 256, 0, 512, 100, 160),
        _sub(256, 320, 0, 512, 0, 0),
        _sub(320, 512, 0, 512, 100, 160),
    ]
    root, cands, results, promoted, remap = _carve(subs)
    check("one of two separate shadow stripes is carved and promoted",
          len(results) == 1 and len(promoted) == 1,
          f"carves={len(results)} promoted={sorted(promoted)}")


def test_rotated_lightmap_axes_are_never_promoted():
    # Texture (so lightmap) axes rotated 30 degrees on the floor: a sub-face's
    # luxel rectangle is a rotated box in world space, which an axis-aligned
    # range cannot describe. One dark 8x8-luxel block, the rest lit.
    import math
    c, s_ = math.cos(math.radians(30)), math.sin(math.radians(30))
    vecs = ((c / 16, s_ / 16, 0.0, 0.0), (-s_ / 16, c / 16, 0.0, 16.0))
    subs = []
    for x, y, w, h, dark in [(20, 12, 8, 8, True), (0, 0, 20, 44, False),
                             (28, 0, 16, 44, False), (20, 0, 8, 12, False),
                             (20, 20, 8, 24, False)]:
        subs.append(BSPSubFaceInfo(
            face_index=0, lightmap_mins=(x, y), lightmap_size=(w, h),
            lightmap_vecs=vecs,
            luminances=[0.0] * 4 if dark else [100.0, 160.0] * 2,
            is_uniform=dark))
    root, cands, results, promoted, remap = _carve(subs)
    check("a face with rotated lightmap axes is not promoted", not promoted,
          f"promoted={sorted(promoted)}")


def test_half_luxel_offset_still_carves_and_stays_dark():
    # The S axis is offset half a luxel, so sub-face edges fall mid-luxel and
    # VBSP writes each luxel window rounded outward (floor mins, ceil maxs).
    # True layout: dark corner 0..128 x 0..128, the rest lit.
    import math

    def padded(x0, x1, y0, y1, dark):
        s0, s1 = x0 / 16 + 0.5, x1 / 16 + 0.5
        t0, t1 = y0 / 16, y1 / 16
        return BSPSubFaceInfo(
            face_index=0,
            lightmap_mins=(math.floor(s0), math.floor(t0)),
            lightmap_size=(math.ceil(s1) - math.floor(s0), math.ceil(t1) - math.floor(t0)),
            lightmap_vecs=((1 / 16, 0.0, 0.0, 0.5), (0.0, 1 / 16, 0.0, 0.0)),
            luminances=[0.0] * 4 if dark else [100.0, 160.0] * 2,
            is_uniform=dark)
    subs = [padded(0, 128, 0, 128, True), padded(128, 512, 0, 512, False),
            padded(0, 128, 128, 512, False)]
    root, cands, results, promoted, remap = _carve(subs)
    rects = _promoted_rects(root, promoted)
    check("a mid-luxel dark corner is still carved and promoted", len(rects) == 1,
          f"carves={len(results)} rects={rects}")
    check("the promoted piece lies inside the true dark corner",
          all(r[1] <= 128 and r[3] <= 128 for r in rects), f"rects={rects}")


def _chop_run(top_material, subs=None, hidden=None):
    """lmoptimizer --chop in --bsp mode on the fixture box, its top side made of
    top_material and lit with a dark corner, or with `subs` (the BSP reader is
    stubbed, the solver is the calibration fallback, LIGHTS/CARVETEST is a
    texlight). hidden: BSP face indices a stubbed bspwalk reports never
    visible (--visibility-check). Returns (exit code, output root or None)."""
    import contextlib
    import io
    import tempfile
    from pathlib import Path
    from unittest import mock
    import bsp_reader
    from _testkit import run_main, write_bsp
    from bsp_reader import FaceLightmapData

    import bspwalk_runner
    subs = subs or [_sub(0, 128, 0, 128, 0, 0), _sub(128, 512, 0, 512, 100, 160),
                    _sub(0, 128, 128, 512, 100, 160)]
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as td, \
            contextlib.ExitStack() as stack:
        td = Path(td)
        vmf = td / 'm.vmf'
        vmf.write_text(_expand_braces(_VMF.replace(
            '"id" "1" "plane" "(0 512 16) (512 512 16) (512 0 16)" "material" "DEV/FLOOR"',
            f'"id" "1" "plane" "(0 512 16) (512 512 16) (512 0 16)" "material" "{top_material}"')))
        rad = td / 'lights.rad'
        rad.write_text('LIGHTS/CARVETEST 255 255 255 200\n')
        bsp = td / 'm.bsp'
        write_bsp(bsp, ldr_len=4096)
        reader = stack.enter_context(mock.patch.object(bsp_reader, 'BSPReader')).return_value
        reader.read_vertex_count.return_value = 1000
        reader.get_bsp_lightmapscale.return_value = 16
        reader.extract_all_face_data.return_value = []
        reader.face_side_ids_version.return_value = 0
        lit = FaceLightmapData(vmf_side_id=1, material=top_material,
                               bsp_face_indices=sorted({sf.face_index for sf in subs}),
                               luminances=[x for sf in subs for x in sf.luminances])
        lit.sub_faces = subs
        stack.enter_context(mock.patch.object(bsp_reader, 'match_bsp_to_vmf',
                                              return_value={1: lit}))
        argv = [str(vmf), '--bsp', str(bsp), '--chop', '--lights', str(rad),
                '--light-budget', '0', '-o', str(td / 'out.vmf')]
        if hidden is not None:
            argv.append('--visibility-check')
            stack.enter_context(mock.patch.object(
                bspwalk_runner, 'validate_setup',
                return_value=(td / 'bspwalk.exe', td, ['A', 'B'])))
            stack.enter_context(mock.patch.object(
                bspwalk_runner, 'run_bspwalk', return_value=(set(hidden), {
                    'eyes': {'solid_eye': False}, 'vis': {'tier_b_ran': True}})))
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            code = run_main(argv)
        out = td / 'out.vmf'
        return code, (VMFParser().parse_file(out) if out.is_file() else None)


def _brush_count(root):
    return len(extract_brushes(root)) if root is not None else 0


def _top_scales(root):
    """{top rectangle: lightmapscale} of every piece of the carved box."""
    scales = {}
    for b in extract_brushes(root):
        for s in b.sides:
            pts = s.plane_points
            if len({p[2] for p in pts}) == 1 and pts[0][2] == 16:
                scales[_top_rect(b)] = int(s._node.get_property('lightmapscale'))
    return scales


def test_texlight_side_is_never_carved():
    """Protects (#79): a carve is kept only when its dark piece gets promoted,
    and texlight sides are never promoted, so a mixed dark/lit texlight side
    must not be carved (it used to be split for nothing). Breaks if the carve
    candidates stop excluding texlight sides."""
    code, root = _chop_run('DEV/FLOOR')
    check("control: an ordinary side with a dark corner is carved",
          code == 0 and _brush_count(root) > 1, f"code={code} brushes={_brush_count(root)}")
    code, root = _chop_run('LIGHTS/CARVETEST')
    check("the same side as a texlight leaves the brush whole",
          code == 0 and _brush_count(root) == 1, f"code={code} brushes={_brush_count(root)}")


def test_hidden_part_of_a_partly_visible_side_is_carved_off():
    """Protects (#86): a side with one visible sliver loses the whole-side
    never-visible override, so --chop carves the never-visible part off as its
    own piece, which gets the coarse scale, while every piece over a visible
    face keeps a finer one. Every sub-face is lit and varied here: only the
    visibility result makes the corner promotable. Breaks if the carver stops
    treating never-visible sub-faces as promotable, or promotes a piece that
    covers a visible face."""
    subs = [_sub(0, 128, 0, 128, 100, 160, face_index=7),
            _sub(128, 512, 0, 512, 100, 160, face_index=8),
            _sub(0, 128, 128, 512, 100, 160, face_index=9)]
    code, root = _chop_run('DEV/FLOOR', subs=subs, hidden={7})
    scales = _top_scales(root) if root is not None else {}
    corner = scales.get((0, 112, 0, 112))
    others = [v for r, v in scales.items() if r != (0, 112, 0, 112)]
    check("the hidden corner is carved off as its own piece",
          code == 0 and corner is not None and others, f"code={code} pieces={scales}")
    check("only the hidden piece is coarsened",
          corner is not None and others and corner > max(others), f"pieces={scales}")

    code, root = _chop_run('DEV/FLOOR', subs=subs, hidden=set())
    check("control: with nothing hidden the lit side is not carved",
          code == 0 and _brush_count(root) == 1, f"code={code} brushes={_brush_count(root)}")


if __name__ == '__main__':
    raise SystemExit(run_all("brush carver", [
        test_texlight_side_is_never_carved,
        test_hidden_part_of_a_partly_visible_side_is_carved_off,
        test_clean_dark_corner_promotes_only_its_top_face,
        test_checkerboard_dark_is_not_promoted_or_carved,
        test_bright_strip_inside_dark_cluster_blocks_promotion,
        test_two_separate_shadows_on_one_axis_still_carve,
        test_rotated_lightmap_axes_are_never_promoted,
        test_half_luxel_offset_still_carves_and_stays_dark,
    ]))
