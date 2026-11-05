"""Regression tests for the unlit-baseline poisoning chain (2026-11-04).

Run:  python game/bin/x64/unit_test/lmoptimizer/test_lighting_guards.py
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

These tests pin the three fixes:
  A. Empty luminances mean UNKNOWN, never "perfectly uniform".
  B. bsp_has_lighting() detects unlit BSPs (used to abort and to reject
     poisoned cache entries).
  C. The baseline light-budget helpers: texlight material collection,
     emissive side detection, and the raise-only bump that keeps VRAD's
     direct-light count under the budget during the baseline compile.
"""
import struct
import sys
import tempfile
from pathlib import Path

# Make game/bin/x64 importable
BIN_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(BIN_DIR))

from bsp_reader import FaceLightmapData, bsp_has_lighting, LUMP_LIGHTING, LUMP_LIGHTING_HDR  # noqa: E402
from vmf_parser import KVNode, KVPair                                      # noqa: E402
import lmoptimizer                                                         # noqa: E402

FAILURES = []


def check(name, cond, detail=""):
    if cond:
        print(f"  ok    {name}")
    else:
        print(f"  FAIL  {name}  {detail}")
        FAILURES.append(name)


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


def test_single_sample_is_still_uniform():
    fld = FaceLightmapData(vmf_side_id=1, material='M', bsp_face_indices=[0],
                           luminances=[0.0])
    check("single sample -> is_perfectly_uniform True (real data, trivially constant)",
          fld.is_perfectly_uniform is True)


def test_constant_samples_are_uniform():
    fld = FaceLightmapData(vmf_side_id=1, material='M', bsp_face_indices=[0],
                           luminances=[7.0] * 16)
    check("constant samples -> is_perfectly_uniform True",
          fld.is_perfectly_uniform is True)
    check("constant samples -> mean 7", fld.mean_luminance == 7.0)


def test_varying_samples_are_not_uniform():
    fld = FaceLightmapData(vmf_side_id=1, material='M', bsp_face_indices=[0],
                           luminances=[0.0, 200.0, 50.0, 3.0])
    check("varying samples -> is_perfectly_uniform False",
          fld.is_perfectly_uniform is False)


def test_never_visible_flag_still_uniform():
    fld = FaceLightmapData(vmf_side_id=1, material='M', bsp_face_indices=[0],
                           luminances=[0.0, 12.0])
    fld.is_never_visible = True
    check("is_never_visible -> is_perfectly_uniform True (oracle decision preserved)",
          fld.is_perfectly_uniform is True)


# ─── B. Unlit BSP detection ──────────────────────────────────────────────────

def write_bsp_header(path, ldr_len=0, hdr_len=0):
    """Minimal BSP: VBSP ident + version + 64 lump directory entries."""
    lumps = [(0, 0, 0, 0)] * 64
    lumps[LUMP_LIGHTING] = (0, ldr_len, 0, 0)
    lumps[LUMP_LIGHTING_HDR] = (0, hdr_len, 0, 0)
    with open(path, 'wb') as f:
        f.write(b'VBSP')
        f.write(struct.pack('<i', 20))
        for ofs, length, ver, fourcc in lumps:
            f.write(struct.pack('<iiI4s', ofs, length, ver, fourcc.to_bytes(4, 'little') if isinstance(fourcc, int) else fourcc))


def test_bsp_has_lighting():
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)

        unlit = td / 'unlit.bsp'
        write_bsp_header(unlit, ldr_len=0, hdr_len=0)
        check("all lighting lumps empty -> False", bsp_has_lighting(unlit) is False)

        ldr = td / 'ldr.bsp'
        write_bsp_header(ldr, ldr_len=4096, hdr_len=0)
        check("LDR lighting present -> True", bsp_has_lighting(ldr) is True)

        hdr = td / 'hdr.bsp'
        write_bsp_header(hdr, ldr_len=0, hdr_len=1048576)
        check("HDR lighting present -> True", bsp_has_lighting(hdr) is True)

        garbage = td / 'garbage.bsp'
        garbage.write_bytes(b'NOTABSP' * 64)
        check("garbage file -> False", bsp_has_lighting(garbage) is False)

        check("missing file -> False",
              bsp_has_lighting(td / 'nope.bsp') is False)


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


def main():
    print("test_lighting_guards:")
    test_empty_luminances_are_not_uniform()
    test_single_sample_is_still_uniform()
    test_constant_samples_are_uniform()
    test_varying_samples_are_not_uniform()
    test_never_visible_flag_still_uniform()
    test_bsp_has_lighting()
    test_collect_texlight_materials()
    test_find_emissive_side_nodes()
    test_raise_emissive_scales()
    test_raise_emissive_scales_missing_key()

    if FAILURES:
        print(f"\n{len(FAILURES)} FAILED: {', '.join(FAILURES)}")
        return 1
    print("\nAll lighting-guard tests passed.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
