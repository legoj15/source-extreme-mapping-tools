"""Real-binary guard: a merged func_detail face lists every merged side (#73).

Run:  python game/bin/x64/unit_test/lmoptimizer/test_detail_merge_sideids.py [--game DIR]
  or: pytest game/bin/x64/unit_test/lmoptimizer
Exit code 0 = all pass. Needs vbsp_lmo.exe in game/bin/x64 (or LMOPT_TEST_VBSP)
and a game directory (default: HL2 Deathmatch's hl2mp, or LMOPT_TEST_GAME).
Missing either is reported as SKIP.

Protects: two touching func_detail boxes with coplanar, same-material tops
compile to one BSP face, and its -emitsideids lump entry lists both top sides.
Breaks if VBSP's detail path (MakeBrushFace) stops seeding a face's merged
side-ID list, or TryMerge stops carrying both parents' lists: the lump then
names only the first side and lmoptimizer's _find_culled_sides has to guess.
Also breaks if TryMerge lists a split-and-remerged side twice (second pair).
"""
import os
import sys
import tempfile
from pathlib import Path

# python -I drops the script's folder from sys.path; _testkit lives beside this file.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _testkit import BIN_DIR, check, run_all

sys.path.insert(0, str(BIN_DIR / 'unit_test' / 'bspwalk'))

DEFAULT_GAME = r"E:/Steam/steamapps/common/Half-Life 2 Deathmatch/hl2mp"
GAME = Path(os.environ.get('LMOPT_TEST_GAME', DEFAULT_GAME))
VBSP = Path(os.environ.get('LMOPT_TEST_VBSP', BIN_DIR / 'vbsp_lmo.exe'))


def _skip_reason():
    if not VBSP.is_file():
        return f"not built: {VBSP}"
    if not (GAME / 'gameinfo.txt').is_file():
        return f"no gameinfo.txt in {GAME} (set LMOPT_TEST_GAME or --game)"
    return None


def _write_two_detail_boxes(vmf_path):
    """Room with two touching func_detail boxes, plus a pair offset on both
    axes so ChopBrushes splits a top whose fragments merge back. Returns the
    touching pair's top side IDs."""
    import vmfgen
    from vmf_parser import VMFWriter
    m = vmfgen.Map(name=vmf_path.stem, description="#73 detail merge side IDs")
    room = m.room((0, 0, 0), (512, 512, 256))
    m.entity("info_player_start", origin=room.floor_center())
    a = m.box((128, 300, 0), (256, 428, 64))
    b = m.box((256, 300, 0), (384, 428, 64))
    c = m.box((128, 40, 0), (256, 168, 64))
    d = m.box((192, 72, 0), (320, 200, 64))
    m.brush_entity("func_detail", [a, b, c, d])
    VMFWriter().write_file(m.to_kv(), vmf_path)
    return a.side('top'), b.side('top')


def test_merged_detail_face_lists_both_sides():
    reason = _skip_reason()
    if reason:
        if 'pytest' in sys.modules and 'PYTEST_CURRENT_TEST' in os.environ:
            import pytest
            pytest.skip(reason)
        print(f"  SKIP  {reason}")
        return
    import bsp_reader
    import vbsp_runner

    with tempfile.TemporaryDirectory() as td:
        vmf = Path(td) / 'detail_merge.vmf'
        top_a, top_b = _write_two_detail_boxes(vmf)
        bsp = vbsp_runner.compile_bsp(VBSP, vmf, GAME, extra_args=['-emitsideids'])
        reader = bsp_reader.BSPReader(bsp)
        reader.read()
        lump = reader.read_face_side_ids()
        check("BSP has side-ID lumps", lump is not None)
        check("lumps are stamped version 1+ (every merged side listed)",
              reader.face_side_ids_version() >= 1,
              f"version {reader.face_side_ids_version()}")

        with_a = [ids for ids in lump if top_a in ids]
        with_b = [ids for ids in lump if top_b in ids]
        check("both top sides are in the lumps", with_a and with_b,
              f"side {top_a}: {with_a}, side {top_b}: {with_b}")
        check("the tops merged into one face that lists both sides",
              any(top_b in ids for ids in with_a), f"{with_a}")
        check("no face lists a side twice",
              all(len(ids) == len(set(ids)) for ids in lump),
              f"{[ids for ids in lump if len(ids) != len(set(ids))]}")


TESTS = [test_merged_detail_face_lists_both_sides]


if __name__ == '__main__':
    if '--game' in sys.argv:
        GAME = Path(sys.argv[sys.argv.index('--game') + 1])
    sys.exit(run_all("test_detail_merge_sideids", TESTS))
