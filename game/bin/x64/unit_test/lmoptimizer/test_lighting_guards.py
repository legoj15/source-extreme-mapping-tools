"""Regression tests for the unlit-baseline poisoning chain (2026-11-04).

Run:  python game/bin/x64/unit_test/lmoptimizer/test_lighting_guards.py
  or: pytest game/bin/x64/unit_test/lmoptimizer
Exit code 0 = all pass.

Incident: optimizing the original HL2-MapPack d1_trainstation_02.vmf at a
scale-2 baseline produced masses of 16/32/64/128 faces while the hrcs
remaster of the same geometry got clean 1s/2s. Root cause chain:

  1. At scale 2 the map's texlights subdivide into 2.5M patches ->
     39,027 direct lights > VRAD's hard 32,767 limit ("Too many lights"),
     so VRAD exited in ~5s having written ZERO lighting data.
  2. lmoptimizer caught the VRADError, printed a warning, and continued
     with an unlit baseline BSP (and cached it as a valid baseline).
  3. Faces extracted from an unlit BSP carry empty luminance lists, and
     FaceLightmapData.is_perfectly_uniform returned True for <2 samples
     with mean luminance 0 -- so every matched face classified as
     "perfectly uniform AND dark" and was pre-promoted to its
     no-subdivision scale (ceil(extent/32): the 16/32/64/128 garbage).

These tests pin the fixes:
  A. Empty luminances mean UNKNOWN, never "perfectly uniform"; genuinely
     constant data is uniform despite float rounding.
  B. bsp_has_lighting() agrees with what BSPReader can decode (empty,
     compressed and truncated lighting lumps are all unusable).
  C. The baseline light-budget ladder: texlight discovery, the raise-only
     bump, VRAD's hard limit (not the user's budget) as its target, the
     floor it hands back, and its best-effort / no-TypeError contract.
"""
import struct
import tempfile
import types
from pathlib import Path
from unittest import mock

import os as _os
import sys as _sys
# python -I drops the script's folder from sys.path; _testkit lives beside this file.
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))

from _testkit import check, run_all, write_bsp

import lmoptimizer
import vbsp_runner
import vrad_runner
import bsp_reader
from bsp_reader import BSPReader, FaceLightmapData, bsp_has_lighting
from vmf_parser import KVNode, KVPair

# ─── A. No-data faces are unknown, not uniform ───────────────────────────────

def test_empty_luminances_are_not_uniform():
    fld = FaceLightmapData(vmf_side_id=1, material='M', bsp_face_indices=[0],
                           luminances=[])
    check("empty luminances -> is_perfectly_uniform False",
          fld.is_perfectly_uniform is False,
          f"got {fld.is_perfectly_uniform}")
    # Stats must stay safe (zeros), the uniform gate is what protects the solver
    check("empty luminances -> variance 0", fld.variance == 0.0)
    check("empty luminances -> mean 0", fld.mean_luminance == 0.0)
    check("empty luminances -> range 0", fld.luminance_range == 0.0)


def test_sub_face_without_samples_is_not_uniform():
    # A truncated sample range leaves one BSP face with an empty list; it is
    # unknown, and must not count as a "uniform sub-face".
    bfd = bsp_reader.BSPFaceData(
        face_index=0, hammer_id=1, plane_normal=(0, 0, 1), plane_dist=0.0,
        face_side=0, material='M', luminances=[], luxel_count=4, area=1.0)
    result = {}
    bsp_reader._add_to_result(result, 1, bfd)
    check("empty sub-face -> is_uniform False",
          result[1].sub_faces[0].is_uniform is False)
    bfd.luminances = [3.0, 3.0]
    result = {}
    bsp_reader._add_to_result(result, 1, bfd)
    check("constant sub-face -> is_uniform True (control)",
          result[1].sub_faces[0].is_uniform is True)


def test_single_sample_is_still_uniform():
    # Never-visible faces (bspwalk) are injected as luminances=[0.0] and rely
    # on a single real sample counting as uniform AND dark.
    fld = FaceLightmapData(vmf_side_id=1, material='M', bsp_face_indices=[0],
                           luminances=[0.0])
    check("single sample -> is_perfectly_uniform True (never-visible faces rely on it)",
          fld.is_perfectly_uniform is True)


def test_constant_samples_are_uniform():
    # Protects: genuinely flat faces are pre-promoted to their coarse scale.
    # mean = sum/n is an ulp off for values like 0.1, so a variance == 0.0
    # clause demoted them to "varying" and spent vertex budget on flat faces.
    for value, count in [(7.0, 16), (0.1, 3), (0.7, 3), (12.345678, 7)]:
        fld = FaceLightmapData(vmf_side_id=1, material='M', bsp_face_indices=[0],
                               luminances=[value] * count)
        check(f"{count} x {value} -> is_perfectly_uniform True",
              fld.is_perfectly_uniform is True,
              f"variance={fld.variance!r} range={fld.luminance_range!r}")


def test_varying_samples_are_not_uniform():
    fld = FaceLightmapData(vmf_side_id=1, material='M', bsp_face_indices=[0],
                           luminances=[0.0, 200.0, 50.0, 3.0])
    check("varying samples -> is_perfectly_uniform False",
          fld.is_perfectly_uniform is False)


# ─── B. Lighting-lump usability ──────────────────────────────────────────────

def test_bsp_has_lighting():
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)

        unlit = td / 'unlit.bsp'
        write_bsp(unlit)
        check("all lighting lumps empty -> False", bsp_has_lighting(unlit) is False)

        ldr = td / 'ldr.bsp'
        write_bsp(ldr, ldr_len=4096)
        check("LDR lighting present -> True", bsp_has_lighting(ldr) is True)

        hdr = td / 'hdr.bsp'
        write_bsp(hdr, hdr_len=4096)
        check("HDR lighting present -> True", bsp_has_lighting(hdr) is True)

        # BSPReader prefers HDR when it is non-empty; the gate must judge the
        # lump the reader will actually decode.
        hdr_lzma = td / 'hdr_lzma.bsp'
        write_bsp(hdr_lzma, ldr_len=4096, hdr_len=4096, hdr_comp=123456)
        check("compressed HDR (reader's choice) -> False even with plain LDR",
              bsp_has_lighting(hdr_lzma) is False)

        ldr_lzma = td / 'ldr_lzma.bsp'
        write_bsp(ldr_lzma, ldr_len=4096, ldr_comp=123456)
        check("compressed LDR -> False", bsp_has_lighting(ldr_lzma) is False)

        # A lump that claims more bytes than the file holds
        truncated = td / 'truncated.bsp'
        write_bsp(truncated, ldr_len=4096)
        truncated.write_bytes(truncated.read_bytes()[:-100])
        check("lighting lump past end of file -> False",
              bsp_has_lighting(truncated) is False)

        # Long enough to pass the length check, with a NON-empty lump table, so
        # only the ident check can reject it
        garbage = td / 'garbage.bsp'
        garbage.write_bytes(b'NOTABSP!' + struct.pack('<4i', 1032, 4096, 0, 0) * 70 + bytes(8192))
        check("non-VBSP file with a plausible lump table -> False",
              bsp_has_lighting(garbage) is False)

        check("missing file -> False",
              bsp_has_lighting(td / 'nope.bsp') is False)


def test_reader_refuses_compressed_lighting():
    # The gate says False for a compressed lump; the reader must agree and
    # refuse instead of slicing compressed bytes as ColorRGBExp32 noise.
    with tempfile.TemporaryDirectory() as td:
        path = Path(td) / 'lzma.bsp'
        write_bsp(path, ldr_len=4096, ldr_comp=123456)
        reader = BSPReader(path)
        reader.read()
        raised = False
        try:
            reader.read_lighting(use_hdr=False)
        except ValueError:
            raised = True
        check("BSPReader.read_lighting refuses an LZMA-compressed lump", raised)

        plain = Path(td) / 'plain.bsp'
        write_bsp(plain, ldr_len=4096)
        reader = BSPReader(plain)
        reader.read()
        check("plain lighting lump is returned whole",
              len(reader.read_lighting(use_hdr=False)) == 4096)


# ─── C. Baseline light-budget helpers ────────────────────────────────────────

RAD_GAME = 'lights/white001 255 255 255 400\nlights/yellow001 255 255 0  400\n'
RAD_CUSTOM = 'lights/custom_glow 100 200 255 800\n'


def make_side(side_id, material, lightmapscale='2'):
    return KVNode(name='side', children=[
        KVPair('id', str(side_id)),
        KVPair('material', material),
        KVPair('lightmapscale', lightmapscale),
    ])


def test_collect_texlight_materials():
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        game = td / 'game'
        game.mkdir()
        (game / 'lights.rad').write_text(RAD_GAME)
        custom = td / 'custom.rad'
        custom.write_text(RAD_CUSTOM)

        mats = lmoptimizer._collect_texlight_materials(game, custom)
        check("union of game + custom rad parsed",
              mats == {'LIGHTS/WHITE001', 'LIGHTS/YELLOW001', 'LIGHTS/CUSTOM_GLOW'},
              f"got {sorted(mats)}")

        mats_no_custom = lmoptimizer._collect_texlight_materials(game, None)
        check("custom rad optional",
              mats_no_custom == {'LIGHTS/WHITE001', 'LIGHTS/YELLOW001'})

        mats_no_game = lmoptimizer._collect_texlight_materials(td / 'missing', custom)
        check("missing game dir tolerated",
              mats_no_game == {'LIGHTS/CUSTOM_GLOW'})

        # VRAD also reads the map's own <map>.rad ...
        level = td / 'level.rad'
        level.write_text('lights/level_glow 1 2 3 4\n')
        mats_level = lmoptimizer._collect_texlight_materials(
            game, None, extra_rads=[level])
        check("map's own .rad is read too",
              'LIGHTS/LEVEL_GLOW' in mats_level, f"got {sorted(mats_level)}")

        # ... and falls back to a lights.rad next to the VRAD exe when the
        # game folder has none
        bindir = td / 'bin'
        bindir.mkdir()
        (bindir / 'lights.rad').write_text(RAD_GAME)
        mats_exe = lmoptimizer._collect_texlight_materials(
            td / 'missing', None, vrad_exe=bindir / 'vrad_rtx.exe')
        check("lights.rad next to the VRAD exe is the fallback",
              mats_exe == {'LIGHTS/WHITE001', 'LIGHTS/YELLOW001'},
              f"got {sorted(mats_exe)}")


def test_find_emissive_side_nodes():
    texlights = {'LIGHTS/WHITE001', 'LIGHTS/CUSTOM_GLOW'}
    side_map = {
        1: make_side(1, 'lights/white001'),          # emissive, lowercase
        2: make_side(2, 'CONCRETE/CONCRETEWALL001A'),  # not emissive
        3: make_side(3, '\\lights\\CUSTOM_GLOW\\'),    # backslashes + junk slashes
        4: make_side(4, 'TOOLS/TOOLSNODRAW'),
    }
    emissive = lmoptimizer._find_emissive_side_nodes(side_map, texlights)
    check("emissive sides detected with case/slash normalization",
          set(emissive.keys()) == {1, 3},
          f"got {sorted(emissive.keys())}")
    check("no emissive materials -> empty result",
          lmoptimizer._find_emissive_side_nodes(side_map, set()) == {})


def test_raise_emissive_scales():
    side_map = {
        1: make_side(1, 'lights/white001', '2'),
        2: make_side(2, 'lights/white001', '2'),
        3: make_side(3, 'lights/white001', '16'),   # already above target
    }
    emissive = lmoptimizer._find_emissive_side_nodes(
        side_map, {'LIGHTS/WHITE001'})

    bumped = lmoptimizer._raise_emissive_scales(emissive, target=8)
    check("raise-only bump reports changed faces", bumped == 2, f"got {bumped}")
    check("below-target raised to target",
          side_map[1].get_property('lightmapscale') == '8')
    check("already-above face untouched",
          side_map[3].get_property('lightmapscale') == '16')

    bumped_again = lmoptimizer._raise_emissive_scales(emissive, target=8)
    check("idempotent at target", bumped_again == 0, f"got {bumped_again}")


def test_raise_emissive_scales_missing_key():
    # A side with no explicit lightmapscale key reads as the engine default
    # (16): targets at or below 16 legitimately no-op, but a target above 16
    # must APPEND the pair — KVNode.set_property only updates existing
    # pairs, and silently counting the no-op as a bump once drove the
    # budget ladder to a false 'lights exceed the limit' exit.
    no_key = KVNode(name='side', children=[
        KVPair('id', '7'),
        KVPair('material', 'lights/white001'),
    ])
    emissive = {7: no_key}
    check("missing key reads as default 16, target 8 no-ops",
          lmoptimizer._raise_emissive_scales(emissive, target=8) == 0)
    bumped = lmoptimizer._raise_emissive_scales(emissive, target=32)
    check("missing key appended and counted", bumped == 1, f"got {bumped}")
    check("missing key side now at target",
          no_key.get_property('lightmapscale') == '32',
          f"got {no_key.get_property('lightmapscale')}")


# ─── The ladder, driven with the REAL runner signatures stubbed out ──────────

def drive_ladder(td, count_for, *, sides=None, eligible=((1, 1), (2, 1)),
                 lights_rad=None, vrad=True, vvis_exe=None, vvis_ok=True,
                 game_rad=RAD_GAME):
    """Run _fit_baseline_light_budget against autospecced compile_bsp /
    count_lights (so argument drift against the real signatures fails) and
    return what happened. count_for(root) -> the light count for the tree as
    it is when VRAD is asked (it may raise an exception to simulate a counting
    failure)."""
    game = Path(td) / 'game'
    game.mkdir(exist_ok=True)
    (game / 'lights.rad').write_text(game_rad)
    work = Path(td) / 'work.vmf'
    work.write_text('placeholder')

    root = KVNode(name='__root__', children=sides if sides is not None else [
        make_side(1, 'lights/white001', '2'),            # eligible texlight
        make_side(2, 'concrete/concretewall001a', '2'),  # eligible, not emissive
        make_side(3, 'lights/white001', '2'),            # texlight, NOT eligible
    ])
    out = types.SimpleNamespace(compiled_scales=[], count_calls=[], floor=None,
                                aborted=None, root=root)

    def fake_compile(vbsp_exe, vmf_path, game_dir, **kw):
        out.compiled_scales.append((scale_of(root, 0), kw.get('extra_args')))
        bsp = Path(vmf_path).with_suffix('.bsp')
        bsp.write_bytes(b'x')
        return bsp

    def fake_count(vrad_exe, bsp_path, game_dir, **kw):
        out.count_calls.append(kw)
        return vrad_runner.LightCount(count_for(root), False)

    with mock.patch.object(vbsp_runner, 'compile_bsp', autospec=True,
                           side_effect=fake_compile), \
            mock.patch.object(vrad_runner, 'count_lights', autospec=True,
                              side_effect=fake_count), \
            mock.patch.object(lmoptimizer, '_run_vvis_fast', autospec=True,
                              return_value=vvis_ok):
        try:
            out.floor = lmoptimizer._fit_baseline_light_budget(
                root, work, vbsp_exe=Path(td) / 'vbsp.exe', vvis_exe=vvis_exe,
                vrad_exe=(Path(td) / 'vrad.exe') if vrad else None,
                game_dir=game, bin_root=None, lights_rad=lights_rad,
                eligible_sids=list(eligible))
        except lmoptimizer.PipelineAbort as e:
            out.aborted = e
    return out


def scale_of(root, index):
    return root.children[index].get_property('lightmapscale')


def over_below(minimum, hard=50_000, ok=11_000):
    """count_for: too many lights until the eligible texlight is at `minimum`."""
    return lambda root: hard if int(scale_of(root, 0)) < minimum else ok


def test_ladder_finds_the_smallest_fitting_scale_and_returns_it():
    """Protects: the baseline texlight floor is the SMALLEST scale that keeps
    VRAD under its limit (it pins every eligible texlight face of the final
    map, so a coarser-than-needed rung would coarsen the whole map's emissive
    surfaces for nothing) and the BSP VRAD lights next was compiled at it."""
    for needed in (3, 4, 5, 9, 13, 100):
        with tempfile.TemporaryDirectory() as td:
            run = drive_ladder(td, over_below(needed))
            check(f"needs {needed}: floor is exactly {needed}",
                  run.floor == needed and run.aborted is None,
                  f"floor={run.floor} aborted={run.aborted}")
            check(f"needs {needed}: the eligible texlight ends there",
                  scale_of(run.root, 0) == str(needed),
                  f"got {scale_of(run.root, 0)}")
            check(f"needs {needed}: the BSP on disk was compiled at that scale",
                  run.compiled_scales[-1][0] == str(needed),
                  f"got {run.compiled_scales[-1]}")
    with tempfile.TemporaryDirectory() as td:
        run = drive_ladder(td, over_below(4))
        check("every ladder recompile passes -emitsideids (BSP feeds classification)",
              run.compiled_scales and all(
                  '-emitsideids' in (extra or []) for _scale, extra in run.compiled_scales),
              f"got {run.compiled_scales}")
        check("non-emissive eligible face untouched", scale_of(run.root, 1) == '2')
        check("ineligible texlight face untouched (enforcement owns it)",
              scale_of(run.root, 2) == '2')


def test_ladder_targets_vrad_limit_not_user_budget():
    # The baseline only has to survive VRAD. 20,000 lights is fine for VRAD
    # (32,767 limit) whatever the user's --light-budget is; enforcing a small
    # budget is the post-solver stage's job.
    with tempfile.TemporaryDirectory() as td:
        run = drive_ladder(td, lambda root: 20_000)
        check("baseline below VRAD's limit: no recompile",
              run.compiled_scales == [], f"got {run.compiled_scales}")
        check("baseline below VRAD's limit: scales untouched",
              scale_of(run.root, 0) == '2')
        check("baseline below VRAD's limit: floor 1, no abort",
              run.floor == 1 and run.aborted is None,
              f"floor={run.floor} aborted={run.aborted}")

    # Over the hard limit it must climb even though a lax user budget (e.g.
    # 40,000) would have said 'fine' -- VRAD would die on 'Too many lights'.
    with tempfile.TemporaryDirectory() as td:
        run = drive_ladder(td, lambda root: 35_000 if int(scale_of(root, 0)) < 4 else 30_000)
        check("baseline above VRAD's limit climbs", run.floor == 4,
              f"floor={run.floor}")


def test_ladder_unfittable_aborts():
    """Counts that never fit must exhaust the ladder (through 128) and abort."""
    with tempfile.TemporaryDirectory() as td:
        run = drive_ladder(td, lambda root: 50_000,
                           sides=[make_side(1, 'lights/white001', '2')],
                           eligible=[(1, 1)])
        check("unfittable baseline aborts", run.aborted is not None,
              f"floor={run.floor}")
        check("ladder reached the 128 cap", scale_of(run.root, 0) == '128',
              f"got {scale_of(run.root, 0)}")


def test_ladder_counting_is_best_effort():
    def broken(root):
        raise vrad_runner.VRADError('no -countlights')

    with tempfile.TemporaryDirectory() as td:
        run = drive_ladder(td, broken)
        check("count failure skips the pass: no abort, floor 1, no recompile",
              run.aborted is None and run.floor == 1 and run.compiled_scales == [],
              f"aborted={run.aborted} floor={run.floor}")

    def breaks_after_first(root):
        if int(scale_of(root, 0)) > 2:
            raise vrad_runner.VRADError('died')
        return 39_027

    with tempfile.TemporaryDirectory() as td:
        run = drive_ladder(td, breaks_after_first)
        check("count failure mid-ladder returns the scale the tree was left at",
              run.aborted is None and run.floor == int(scale_of(run.root, 0))
              and run.floor > 2,
              f"aborted={run.aborted} floor={run.floor} tree={scale_of(run.root, 0)}")

    # A missing VRAD (None) used to reach count_lights(None) and die with a
    # TypeError traceback after the whole baseline compile.
    with tempfile.TemporaryDirectory() as td:
        run = drive_ladder(td, lambda root: 0, vrad=False)
        check("no VRAD executable: skipped without calling count_lights",
              run.count_calls == [] and run.floor == 1 and run.aborted is None,
              f"calls={len(run.count_calls)} floor={run.floor}")


def test_ladder_vvis_failure_aborts():
    # A recompile whose VVIS failed has no PVS; counting/lighting from it
    # would be silently degraded.
    with tempfile.TemporaryDirectory() as td:
        run = drive_ladder(td, over_below(4), vvis_exe=Path(td) / 'vvis.exe',
                           vvis_ok=False)
        check("VVIS failure on a ladder recompile aborts", run.aborted is not None,
              f"floor={run.floor}")


def test_ladder_uses_custom_rad_for_discovery_and_counting():
    with tempfile.TemporaryDirectory() as td:
        custom = Path(td) / 'custom.rad'
        custom.write_text(RAD_CUSTOM)
        run = drive_ladder(
            td, over_below(4), lights_rad=custom,
            sides=[make_side(1, 'lights/custom_glow', '2')], eligible=[(1, 1)],
            game_rad='lights/unrelated 1 1 1 1\n')
        check("texlight defined only in the custom .rad is raised",
              scale_of(run.root, 0) == '4', f"got {scale_of(run.root, 0)}")
        check("custom .rad is forwarded to every light count",
              run.count_calls and all(c.get('lights_rad') == custom
                                      for c in run.count_calls),
              f"got {[c.get('lights_rad') for c in run.count_calls]}")


def test_scale_search_returns_the_exact_minimum():
    """Protects: the shared rung-then-bisect search behind both light stages
    lands on the exact minimum (a rung-only ladder overshot 5 -> 6, 9 -> 12)."""
    for needed in (2, 3, 5, 7, 9, 13, 17, 33, 100, 128):
        probes = []

        def fits(scale, needed=needed):
            probes.append(scale)
            return scale >= needed

        got = lmoptimizer._search_min_scale(fits, over=1)
        check(f"minimum {needed} is found exactly", got == needed, f"got {got}")
    check("a limit no scale can meet returns None",
          lmoptimizer._search_min_scale(lambda scale: False, over=1) is None)
    check("the search only looks above the known-failing scale",
          lmoptimizer._search_min_scale(lambda scale: scale >= 3, over=10) == 11)


# ─── --strict-coplanar must not treat unknown faces as uniform ───────────────

def test_strict_unify_skips_faces_without_samples():
    from vertex_estimator import FaceExtent

    def side(sid, uaxis='[1 0 0 0] 0.25'):
        return KVNode(name='side', children=[
            KVPair('id', str(sid)),
            KVPair('plane', '(0 0 0) (64 0 0) (64 64 0)'),
            KVPair('material', 'CONCRETE/FLOOR'),
            KVPair('uaxis', uaxis),
            KVPair('vaxis', '[0 -1 0 0] 0.25'),
            KVPair('lightmapscale', '1'),
        ])

    def square(x0):
        return [(x0, 0, 0), (x0 + 64, 0, 0), (x0 + 64, 64, 0), (x0, 64, 0)]

    def extent(sid):
        return FaceExtent(side_id=sid, extent_s=64.0, extent_t=64.0, s_min=0, s_max=64,
                          t_min=0, t_max=64, s_axis=(1, 0, 0), t_axis=(0, 1, 0),
                          lightmapscale=1, material='CONCRETE/FLOOR',
                          num_vertices=4, brush_entity='')

    def run(luminances):
        side_map = {1: side(1), 2: side(2)}
        face_data = {sid: FaceLightmapData(vmf_side_id=sid, material='M',
                                           bsp_face_indices=[sid],
                                           luminances=list(luminances))
                     for sid in (1, 2)}
        scales = {1: 1, 2: 1}
        lmoptimizer._unify_coplanar_strict(
            scales, side_map, face_data, [extent(1), extent(2)],
            face_vertices={1: square(0), 2: square(64)})
        return scales

    check("uniform group is unified to its no-subdivision scale (control)",
          run([5.0] * 4) == {1: 2, 2: 2}, f"got {run([5.0] * 4)}")
    check("a group whose faces have NO samples stays untouched",
          run([]) == {1: 1, 2: 1}, f"got {run([])}")


def test_strict_unify_shares_the_seam_grouping_definitions():
    """Protects (#75): --strict-coplanar groups sides by the same rules as the
    seam groups: one plane, material and texture-axis DIRECTION (the lightmap
    lattice ignores shift and scale), joined by a shared edge. It used to
    match the raw uaxis/vaxis strings and join any two sides on the plane
    however far apart. Breaks if strict grows its own definition again."""
    from vertex_estimator import FaceExtent

    def side(sid, uaxis):
        return KVNode(name='side', children=[
            KVPair('id', str(sid)), KVPair('plane', '(0 0 0) (64 0 0) (64 64 0)'),
            KVPair('material', 'CONCRETE/FLOOR'), KVPair('uaxis', uaxis),
            KVPair('vaxis', '[0 -1 0 0] 0.25'), KVPair('lightmapscale', '1')])

    def square(x0):
        return [(x0, 0, 0), (x0 + 64, 0, 0), (x0 + 64, 64, 0), (x0, 64, 0)]

    def extent(sid):
        return FaceExtent(side_id=sid, extent_s=64.0, extent_t=64.0, s_min=0, s_max=64,
                          t_min=0, t_max=64, s_axis=(1, 0, 0), t_axis=(0, 1, 0),
                          lightmapscale=1, material='CONCRETE/FLOOR',
                          num_vertices=4, brush_entity='')

    def run(uaxes, origins):
        sids = list(range(1, len(uaxes) + 1))
        side_map = {sid: side(sid, ua) for sid, ua in zip(sids, uaxes)}
        face_data = {sid: FaceLightmapData(vmf_side_id=sid, material='M',
                                           bsp_face_indices=[sid], luminances=[5.0] * 4)
                     for sid in sids}
        scales = {sid: 1 for sid in sids}
        lmoptimizer._unify_coplanar_strict(
            scales, side_map, face_data, [extent(sid) for sid in sids],
            face_vertices={sid: square(x) for sid, x in zip(sids, origins)})
        return scales

    got = run(['[1 0 0 0] 0.25', '[1 0 0 32] 0.5'], [0, 64])
    check("neighbours whose axes differ only in shift and scale are unified",
          got == {1: 2, 2: 2}, f"got {got}")
    got = run(['[1 0 0 0] 0.25', '[1 0 0 0] 0.25'], [0, 512])
    check("coplanar sides that share no edge are not unified",
          got == {1: 1, 2: 1}, f"got {got}")


# ─── Surface flags ───────────────────────────────────────────────────────────

def test_unlit_face_filter_skips_sky_and_keeps_water():
    """Protects: the unlit-BSP face filter skips 2D-sky faces and keeps water.
    SURF_SKY2D was 0x8 (the SDK header's SURF_WARP), so the filter kept 2D-sky
    faces and dropped water."""
    skip = bsp_reader._SURF_SKIP_VIS
    check("a SURF_SKY2D face is skipped", bool(skip & bsp_reader.SURF_SKY2D))
    check("a SURF_WARP (water) face is kept", not skip & bsp_reader.SURF_WARP)
    check("SURF_SKY, SURF_NODRAW and SURF_SKIP faces are skipped",
          all(skip & flag for flag in (bsp_reader.SURF_SKY, bsp_reader.SURF_NODRAW,
                                       bsp_reader.SURF_SKIP)))
    check("SURF_SKY2D and SURF_WARP are different flags",
          bsp_reader.SURF_SKY2D != bsp_reader.SURF_WARP)


# ─── VVIS auto-detection ─────────────────────────────────────────────────────

def test_vvis_found_for_the_optix_layout():
    # This repo ships vvis_optix.exe and no vvis.exe next to vbsp_lmo.exe; the
    # detector used to look for vvis.exe only, found nothing, and the run
    # carried on with a no-bounce baseline.
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        (td / 'vbsp_lmo.exe').write_bytes(b'x')
        (td / 'vvis_optix.exe').write_bytes(b'x')
        args = types.SimpleNamespace(vvis=None, binroot=None, game=None)
        found = lmoptimizer._resolve_vvis(args, td / 'vbsp_lmo.exe', warn=False)
        check("vvis_optix.exe next to VBSP is found",
              found == td / 'vvis_optix.exe', f"got {found}")


TESTS = [
    test_empty_luminances_are_not_uniform,
    test_sub_face_without_samples_is_not_uniform,
    test_single_sample_is_still_uniform,
    test_constant_samples_are_uniform,
    test_varying_samples_are_not_uniform,
    test_bsp_has_lighting,
    test_reader_refuses_compressed_lighting,
    test_collect_texlight_materials,
    test_find_emissive_side_nodes,
    test_raise_emissive_scales,
    test_raise_emissive_scales_missing_key,
    test_ladder_finds_the_smallest_fitting_scale_and_returns_it,
    test_ladder_targets_vrad_limit_not_user_budget,
    test_ladder_unfittable_aborts,
    test_ladder_counting_is_best_effort,
    test_ladder_vvis_failure_aborts,
    test_ladder_uses_custom_rad_for_discovery_and_counting,
    test_scale_search_returns_the_exact_minimum,
    test_strict_unify_skips_faces_without_samples,
    test_strict_unify_shares_the_seam_grouping_definitions,
    test_unlit_face_filter_skips_sky_and_keeps_water,
    test_vvis_found_for_the_optix_layout,
]


if __name__ == '__main__':
    import sys
    sys.exit(run_all("test_lighting_guards", TESTS))
