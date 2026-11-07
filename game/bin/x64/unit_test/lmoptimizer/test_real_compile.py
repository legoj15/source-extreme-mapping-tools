"""Real-binary guard: an uncached lit compile of a lmoptimizer scratch map.

Run:  python game/bin/x64/unit_test/lmoptimizer/test_real_compile.py [--game DIR]
  or: pytest game/bin/x64/unit_test/lmoptimizer
Exit code 0 = all pass. Needs vbsp_lmo.exe, vvis_optix.exe and vrad_rtx.exe in
game/bin/x64 and a game directory (default: HL2 Deathmatch's hl2mp, or the
LMOPT_TEST_GAME environment variable). Missing either is reported as SKIP.

Protects (issue #53): scratch maps are named '<stem>.lmopt_..._<rand>.vmf', a
name with a dot in it. VBSP must write '<scratch>.bsp' (not an extensionless
file), and VVIS and VRAD must process that BSP, not the user's '<stem>.bsp'
beside it. VVIS is given the path without '.bsp' and VRAD with it, the two
spellings that old VVIS and old VRAD each cut at the stem's dot. Breaks if
VBSP, VVIS or VRAD goes back to Default/Strip-Extension or FileBase on the
map name.
"""
import os
import sys
import tempfile
from pathlib import Path

from _testkit import BIN_DIR, check, run_all

sys.path.insert(0, str(BIN_DIR / 'unit_test' / 'bspwalk'))

DEFAULT_GAME = r"E:/Steam/steamapps/common/Half-Life 2 Deathmatch/hl2mp"
GAME = Path(os.environ.get('LMOPT_TEST_GAME', DEFAULT_GAME))

VBSP = BIN_DIR / 'vbsp_lmo.exe'
VVIS = BIN_DIR / 'vvis_optix.exe'
VRAD = BIN_DIR / 'vrad_rtx.exe'


def _skip_reason():
    missing = [p.name for p in (VBSP, VVIS, VRAD) if not p.is_file()]
    if missing:
        return f"not built: {', '.join(missing)}"
    if not (GAME / 'gameinfo.txt').is_file():
        return f"no gameinfo.txt in {GAME} (set LMOPT_TEST_GAME or --game)"
    return None


def _write_lit_room(vmf_path):
    import vmfgen
    m = vmfgen.Map(name=vmf_path.stem, description="dotted-name compile guard")
    room = m.room((0, 0, 0), (256, 256, 128))
    m.entity("info_player_start", origin=room.floor_center())
    m.entity("light", origin=(128, 128, 96), _light="255 255 255 200")
    # to_kv, not write(): write() demands bspwalk visibility expectations.
    from vmf_parser import VMFWriter
    VMFWriter().write_file(m.to_kv(), vmf_path)


def test_dotted_scratch_compiles_and_lights_its_own_bsp():
    reason = _skip_reason()
    if reason:
        if 'pytest' in sys.modules and 'PYTEST_CURRENT_TEST' in os.environ:
            import pytest
            pytest.skip(reason)
        print(f"  SKIP  {reason}")
        return
    import bsp_reader
    import lmoptimizer
    import vbsp_runner
    import vrad_runner

    with tempfile.TemporaryDirectory() as td:
        anchor = Path(td) / 'room.vmf'
        _write_lit_room(anchor)
        # The user's own compile of the map: no tool may read or overwrite it.
        decoy = anchor.with_suffix('.bsp')
        decoy.write_bytes(b'user bsp')

        with vbsp_runner.ScratchVMF(anchor, 'guard') as scratch:
            scratch.path.write_bytes(anchor.read_bytes())
            check("scratch name has a dot in its stem (the case under test)",
                  '.' in scratch.path.stem, scratch.path.name)

            bsp = vbsp_runner.compile_bsp(VBSP, scratch.path, GAME)
            check("VBSP writes <scratch>.bsp", bsp == scratch.bsp and bsp.is_file(),
                  str(bsp))
            check("VBSP writes no extensionless BSP",
                  not scratch.path.with_suffix('').exists())

            # Extensionless on purpose (the runner passes '.bsp'): VVIS
            # itself must keep the dotted stem and append '.bsp'.
            check("VVIS succeeds on the scratch BSP named without '.bsp'",
                  lmoptimizer._run_vvis_fast(VVIS, scratch.path.with_suffix(''), GAME))
            check("VVIS writes no extensionless file",
                  not scratch.path.with_suffix('').exists())

            vrad_runner.compile_rad(VRAD, scratch.bsp, GAME)
            check("VRAD lights the scratch BSP", bsp_reader.bsp_has_lighting(scratch.bsp))
            check("the user's <stem>.bsp is untouched", decoy.read_bytes() == b'user bsp')


TESTS = [test_dotted_scratch_compiles_and_lights_its_own_bsp]


if __name__ == '__main__':
    if '--game' in sys.argv:
        GAME = Path(sys.argv[sys.argv.index('--game') + 1])
    sys.exit(run_all("test_real_compile", TESTS))
