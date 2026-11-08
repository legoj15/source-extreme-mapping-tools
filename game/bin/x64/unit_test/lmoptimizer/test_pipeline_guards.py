"""Regression tests for the lmoptimizer pipeline's failure handling and cache.

Run:  python game/bin/x64/unit_test/lmoptimizer/test_pipeline_guards.py
  or: pytest game/bin/x64/unit_test/lmoptimizer
Exit code 0 = all pass.

Each test states the behaviour it protects:

  * cache entries: a poisoned (unlit) or half-written entry is never trusted
  * cache key: rebuilding VRAD/VVIS/VBSP or editing a .rad invalidates it
  * ScratchVMF: compile scratch is private, beside the input, and cleaning it
    never touches a file the user made
  * baseline scan: a VBSP environment failure is reported as such, not as 'too
    many brushes' after 64 retries; trial VMFs sit next to the input so
    func_instance paths resolve
  * solver: scales reach sides that lack a lightmapscale key
  * compile_rad: a hung VRAD times out; a nonzero VRAD exit is a failure even
    though the BSP VBSP wrote is still on disk
  * main(): every failure exits nonzero, never touches a previous output or
    the user's own compile products, leaves no scratch behind, and bad setups
    are rejected before any work starts
"""
import contextlib
import io
import json
import os
import re
import struct
import sys
import tempfile
import time
import types
from pathlib import Path
from unittest import mock

import os as _os
import sys as _sys
# python -I drops the script's folder from sys.path; _testkit lives beside this file.
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))

from _testkit import MINI_VMF, check, run_all, run_main, write_bsp

import bspwalk_runner
import bsp_reader
import lmoptimizer
import vbsp_runner
import vbsp_solver
import vrad_runner
from bsp_reader import FaceLightmapData
from vertex_estimator import FaceExtent
from vmf_parser import KVNode, KVPair, VMFParser

PREVIOUS_OUTPUT = 'PREVIOUS GOOD OUTPUT'


def tempdir():
    # ignore_cleanup_errors: a killed stub's grandchild can briefly hold its cwd
    return tempfile.TemporaryDirectory(ignore_cleanup_errors=True)


def lmopt_leftovers(directory):
    """Scratch/temp files this tool may have left in a directory."""
    return sorted(p.name for p in Path(directory).iterdir()
                  if 'lmopt' in p.name and p.name != '.lmopt_cache'
                  or p.name.endswith('.tmp'))


# ─── Baseline cache ──────────────────────────────────────────────────────────

def test_cache_entry_rejects_poisoned_and_stale_entries():
    """Protects: a cached BSP VRAD never lit (the 2026-11-04 incident), a
    mismatched key, a corrupt manifest, a manifest pointing outside the cache
    folder or a half-finished update is never trusted; a failed update
    invalidates instead of pairing a new key with an old BSP."""
    with tempdir() as td:
        td = Path(td)
        cache = td / '.lmopt_cache'
        lit = td / 'lit.bsp'
        write_bsp(lit, ldr_len=4096)
        unlit = td / 'unlit.bsp'
        write_bsp(unlit, ldr_len=0)
        unlit.write_bytes(unlit.read_bytes() + bytes(4096))   # same size as lit

        written = lmoptimizer._write_cache_entry(cache, 'm', 'KEY', 3, 8, lit)
        entry, reason = lmoptimizer._read_cache_entry(cache, 'm', 'KEY')
        check("valid entry is returned with baseline, emissive floor and its BSP",
              entry is not None and entry['baseline'] == 3
              and entry['emissive_floor'] == 8 and entry['bsp'] == written
              and written.is_file() and not reason,
              f"got {entry} / {reason!r}")
        entry, reason = lmoptimizer._read_cache_entry(cache, 'm', 'OTHER-KEY')
        check("a different key is a silent miss", entry is None and not reason)

        # Poisoned: the key matches, the BSP is unlit (same size, so only the
        # lighting check can catch it)
        lmoptimizer._write_cache_entry(cache, 'm', 'KEY', 3, 1, unlit)
        entry, reason = lmoptimizer._read_cache_entry(cache, 'm', 'KEY')
        check("matching key but unlit BSP is rejected, with a reason",
              entry is None and bool(reason), f"got {entry} / {reason!r}")

        # Out-of-range baseline in an otherwise matching manifest
        lmoptimizer._write_cache_entry(cache, 'm', 'KEY', 3, 1, lit)
        manifest = cache / 'm.manifest.json'
        data = json.loads(manifest.read_text())
        data['baseline'] = 0
        manifest.write_text(json.dumps(data))
        entry, reason = lmoptimizer._read_cache_entry(cache, 'm', 'KEY')
        check("out-of-range baseline is rejected", entry is None and bool(reason))

        data['baseline'] = 3
        data['bsp_file'] = '../lit.bsp'
        manifest.write_text(json.dumps(data))
        entry, reason = lmoptimizer._read_cache_entry(cache, 'm', 'KEY')
        check("a manifest naming a BSP outside the cache folder is rejected",
              entry is None and bool(reason))

        manifest.write_text('{not json')
        entry, reason = lmoptimizer._read_cache_entry(cache, 'm', 'KEY')
        check("corrupt manifest is rejected", entry is None and bool(reason))

        # A failed update must leave a MISS, not the old key over a new BSP
        lmoptimizer._write_cache_entry(cache, 'm', 'KEY', 3, 1, lit)
        raised = False
        try:
            lmoptimizer._write_cache_entry(cache, 'm', 'KEY2', 5, 1, td / 'missing.bsp')
        except OSError:
            raised = True
        check("an update from a missing BSP raises", raised)
        check("...and invalidates the previous entry",
              lmoptimizer._read_cache_entry(cache, 'm', 'KEY')[0] is None
              and lmoptimizer._read_cache_entry(cache, 'm', 'KEY2')[0] is None)
        check("...and leaves no temp files behind",
              not [p for p in cache.iterdir() if p.name.endswith('.tmp')],
              f"got {sorted(p.name for p in cache.iterdir())}")

        # Legacy two-file entries are cleared on write
        (cache / 'm.vmf_hash').write_text('old')
        (cache / 'm.baseline_scale').write_text('2')
        lmoptimizer._write_cache_entry(cache, 'm', 'KEY', 3, 1, lit)
        check("legacy cache files are removed",
              not (cache / 'm.vmf_hash').exists()
              and not (cache / 'm.baseline_scale').exists())


def test_cache_entries_of_concurrent_runs_never_swap_bsps():
    """Protects: two runs on one map can never pair one run's key with the
    other run's BSP (a shared '<stem>.bsp' let the second writer replace the
    first writer's BSP under it); a fresh entry is never instantly collectable
    (copying used to keep the source's old mtime); superseded BSPs go away only
    once a day old."""
    with tempdir() as td:
        td = Path(td)
        cache = td / '.lmopt_cache'
        lit_a, lit_b = td / 'a.bsp', td / 'b.bsp'
        write_bsp(lit_a, ldr_len=4096)
        write_bsp(lit_b, ldr_len=8192)
        three_days_ago = time.time() - 3 * 86400
        for src in (lit_a, lit_b):
            os.utime(src, (three_days_ago, three_days_ago))   # an old source file

        path_a = lmoptimizer._write_cache_entry(cache, 'm', 'KEY-A', 2, 1, lit_a)
        path_b = lmoptimizer._write_cache_entry(cache, 'm', 'KEY-B', 3, 1, lit_b)
        check("each write gets its own BSP file", path_a != path_b)
        check("run A's BSP is untouched by run B's write",
              path_a.read_bytes() == lit_a.read_bytes())
        check("a fresh entry's mtime is now, not its source's (old) mtime",
              time.time() - path_b.stat().st_mtime < 300)
        entry_b, _ = lmoptimizer._read_cache_entry(cache, 'm', 'KEY-B')
        check("the manifest pairs B's key with B's BSP",
              entry_b is not None and entry_b['bsp'] == path_b
              and entry_b['bsp'].read_bytes() == lit_b.read_bytes())
        check("A's key is a miss now, never a hit on B's BSP",
              lmoptimizer._read_cache_entry(cache, 'm', 'KEY-A')[0] is None)

        # Garbage collection: unreferenced BSPs a day old (including the legacy
        # shared name) go; younger ones and other maps' files stay
        two_days_ago = time.time() - 2 * 86400
        two_hours_ago = time.time() - 2 * 3600
        os.utime(path_a, (two_days_ago, two_days_ago))
        legacy = cache / 'm.bsp'
        legacy.write_bytes(b'legacy')
        os.utime(legacy, (two_days_ago, two_days_ago))
        (cache / 'other.bsp').write_bytes(b'other map')
        os.utime(cache / 'other.bsp', (two_days_ago, two_days_ago))
        os.utime(path_b, (two_hours_ago, two_hours_ago))
        path_c = lmoptimizer._write_cache_entry(cache, 'm', 'KEY-C', 4, 1, lit_a)
        check("a superseded BSP over a day old is collected", not path_a.exists())
        check("the old legacy-named BSP is collected", not legacy.exists())
        check("a superseded BSP only hours old survives (a concurrent run may "
              "be about to copy it)", path_b.exists())
        check("the current BSP and another map's file survive",
              path_c.exists() and (cache / 'other.bsp').exists())


def test_cache_key_tracks_tool_implementation_and_rad_inputs():
    """Protects: rebuilding VRAD (its DLL, behind a launcher stub), VVIS or
    VBSP, or editing any .rad VRAD reads or any func_instance VMF the map
    pulls in, invalidates the cached baseline instead of silently reusing
    lighting from the old build. Likewise a different --texture-shadows level
    (the analysis lighting's shadows differ) forces a recompile, and the level
    is what VRAD is asked for. (Models and materials are not hashed: see
    _build_cache_key's docstring.)"""
    with tempdir() as td:
        td = Path(td)
        tools = td / 'tools'
        tools.mkdir()
        files = {name: tools / name for name in (
            'vbsp_lmo.exe', 'vrad_rtx.exe', 'vrad_rtx_dll.dll',
            'optix_kernels.ptx', 'vvis_optix.exe', 'vvis_optix_dll.dll')}
        for name, path in files.items():
            path.write_bytes(name.encode())
        game = td / 'game'
        game.mkdir()
        (game / 'lights.rad').write_text('lights/a 1 1 1 1\n')
        maps = td / 'maps'
        maps.mkdir()
        vmf = maps / 'm.vmf'
        vmf.write_text('map v1')
        level_rad = maps / 'm.rad'
        custom = td / 'custom.rad'
        custom.write_text('lights/b 1 1 1 1\n')
        inst_dir = maps / 'inst'
        inst_dir.mkdir()
        inst_a = inst_dir / 'a.vmf'
        inst_a.write_text('instance a v1')
        inst_b = inst_dir / 'b.vmf'
        inst_b.write_text('instance b v1')

        def key(**override):
            kw = dict(input_vmf=vmf, vbsp=files['vbsp_lmo.exe'],
                      vrad=files['vrad_rtx.exe'], vvis=files['vvis_optix.exe'],
                      game_dir=game, budget=62259, detail_min_scale=5,
                      rtx=False, texture_shadows='props', bin_root=None,
                      lights_rad=None, level_rad=level_rad, eligible_hash='e1',
                      instance_files=[inst_a])
            kw.update(override)
            return lmoptimizer._build_cache_key(**kw)

        base = key()
        check("key is stable for identical inputs", key() == base)

        def rewrite(path, text):
            def mutate():
                path.write_text(text)
            return mutate

        mutations = [
            ("VRAD launcher bytes", rewrite(files['vrad_rtx.exe'], 'changed')),
            ("VRAD implementation DLL", rewrite(files['vrad_rtx_dll.dll'], 'changed')),
            ("VRAD GPU kernels", rewrite(files['optix_kernels.ptx'], 'changed')),
            ("VBSP exe", rewrite(files['vbsp_lmo.exe'], 'changed')),
            ("VVIS launcher", rewrite(files['vvis_optix.exe'], 'changed')),
            ("VVIS implementation DLL", rewrite(files['vvis_optix_dll.dll'], 'changed')),
            ("game lights.rad content", rewrite(game / 'lights.rad', 'lights/a 9 9 9 9\n')),
            ("level .rad appears", rewrite(level_rad, 'lights/c 1 1 1 1\n')),
            ("input VMF", rewrite(vmf, 'map v2')),
            ("instance VMF", rewrite(inst_a, 'instance a v2')),
        ]
        for what, mutate in mutations:
            mutate()
            check(f"key changes when the {what} changes", key() != base)
            # restore for the next, independent probe
            files['vrad_rtx.exe'].write_bytes(b'vrad_rtx.exe')
            files['vrad_rtx_dll.dll'].write_bytes(b'vrad_rtx_dll.dll')
            files['optix_kernels.ptx'].write_bytes(b'optix_kernels.ptx')
            files['vbsp_lmo.exe'].write_bytes(b'vbsp_lmo.exe')
            files['vvis_optix.exe'].write_bytes(b'vvis_optix.exe')
            files['vvis_optix_dll.dll'].write_bytes(b'vvis_optix_dll.dll')
            (game / 'lights.rad').write_text('lights/a 1 1 1 1\n')
            if level_rad.exists():
                level_rad.unlink()
            vmf.write_text('map v1')
            inst_a.write_text('instance a v1')
            check(f"key is restored after undoing the {what} change", key() == base)

        check("key changes with --lights", key(lights_rad=custom) != base)
        check("key changes with --rtx", key(rtx=True) != base)
        # The analysis lighting's shadows depend on the level, so a cached
        # baseline lit at one level must not be reused for another.
        levels = sorted(vrad_runner.TEXTURE_SHADOW_LEVELS)
        level_keys = {lv: key(texture_shadows=lv) for lv in levels}
        check("key differs between every pair of texture-shadow levels",
              len(set(level_keys.values())) == len(levels),
              f"keys collide: {level_keys}")
        check("key changes with the vertex budget", key(budget=1000) != base)
        check("key changes with the eligibility output", key(eligible_hash='e2') != base)
        check("key changes when another instance joins the closure",
              key(instance_files=[inst_a, inst_b]) != base)

    # End to end through main(): the CLI level reaches compile_rad, a repeat
    # at the same level is a cache hit, and another level recompiles.
    world, world_again, every, default = run_cache_rounds([
        ('--texture-shadows', 'world'), ('--texture-shadows', 'world'),
        ('--texture-shadows', 'all'), ()])
    check("--texture-shadows world is what VRAD is asked for",
          world.compiled and world.rad_levels == ['world'], f"{world.rad_levels}")
    check("the same level again is a cache hit (VRAD not run)",
          not world_again.compiled and world_again.rad_levels == [],
          f"{world_again.rad_levels}")
    check("another level recompiles with that level",
          every.compiled and every.rad_levels == ['all'], f"{every.rad_levels}")
    check("no flag recompiles with the default level",
          default.compiled and default.rad_levels == [vrad_runner.DEFAULT_TEXTURE_SHADOWS],
          f"{default.rad_levels}")


def _write_vpk(path, files):
    """A minimal VPK v1 directory file holding every file inline (preload)."""
    tree = b''
    by_ext = {}
    for name, data in files.items():
        stem, ext = name.rsplit('.', 1)
        folder, _, base = stem.rpartition('/')
        by_ext.setdefault(ext, {}).setdefault(folder or ' ', []).append((base, data))
    for ext, folders in by_ext.items():
        tree += ext.encode() + b'\0'
        for folder, entries in folders.items():
            tree += folder.encode() + b'\0'
            for base, data in entries:
                tree += base.encode() + b'\0'
                tree += struct.pack('<IHHIIH', 0, len(data), 0x7FFF, 0, 0, 0xFFFF)
                tree += data
            tree += b'\0'
        tree += b'\0'
    tree += b'\0'
    path.write_bytes(struct.pack('<III', 0x55AA1234, 1, len(tree)) + tree)


def test_cache_key_follows_vrads_lights_rad_search_path():
    """Protects: VRAD finds lights.rad through gameinfo's SearchPaths (a
    base-game folder or VPK when the mod has none), so editing THAT file
    invalidates the cached baseline. Breaks if the resolver only looks in the
    mod folder again (issue #30)."""
    with tempdir() as td:
        td = Path(td)
        vrad = td / 'bin' / 'vrad_rtx.exe'
        vrad.parent.mkdir()
        vrad.write_bytes(b'vrad')
        mod = td / 'mymod'
        mod.mkdir()
        base_game = td / 'hl2'
        base_game.mkdir()
        (mod / 'gameinfo.txt').write_text(
            '"GameInfo"\n{\n  game "My Mod"\n  FileSystem\n  {\n'
            '    SteamAppId 243730\n    SearchPaths\n    {\n'
            '      game+mod  |gameinfo_path|.\n'
            '      game      |all_source_engine_paths|hl2/hl2_misc.vpk\n'
            '      game      |all_source_engine_paths|hl2  // base game\n'
            '      gamebin   |gameinfo_path|bin\n'
            '    }\n  }\n}\n')

        vbsp, vvis = td / 'bin' / 'vbsp_lmo.exe', td / 'bin' / 'vvis_optix.exe'
        vbsp.write_bytes(b'vbsp')
        vvis.write_bytes(b'vvis')
        vmf = td / 'm.vmf'
        vmf.write_text('map')

        def key():
            return lmoptimizer._build_cache_key(
                input_vmf=vmf, vbsp=vbsp, vrad=vrad, vvis=vvis, game_dir=mod,
                budget=1, detail_min_scale=1, rtx=False,
                texture_shadows='props', bin_root=None,
                lights_rad=None, level_rad=None, eligible_hash='e')

        none = key()
        (base_game / 'lights.rad').write_text('lights/a 1 1 1 1\n')
        loose = key()
        check("a base-game lights.rad is found through gameinfo", loose != none)
        (base_game / 'lights.rad').write_text('lights/a 9 9 9 9\n')
        edited = key()
        check("editing the base-game lights.rad changes the key", edited != loose)
        check("its texlights are collected",
              lmoptimizer._collect_texlight_materials(mod, None, vrad_exe=vrad)
              == {'LIGHTS/A'})

        # A VPK mounted ahead of the loose folder wins, as in VRAD
        vpk = base_game / 'hl2_misc_dir.vpk'
        _write_vpk(vpk, {'lights.rad': b'lights/vpk 1 1 1 1\n'})
        in_vpk = key()
        check("a lights.rad inside an earlier VPK takes precedence",
              in_vpk != edited)
        check("texlights come from the VPK copy",
              lmoptimizer._collect_texlight_materials(mod, None, vrad_exe=vrad)
              == {'LIGHTS/VPK'})
        # (a different length too, so a coarse mtime cannot hide the edit)
        _write_vpk(vpk, {'lights.rad': b'lights/vpk 22 2 2 2\n'})
        check("editing the lights.rad inside the VPK changes the key",
              key() != in_vpk)

        # The mod's own copy beats both
        (mod / 'lights.rad').write_text('lights/mod 1 1 1 1\n')
        check("the mod folder's lights.rad wins",
              lmoptimizer._collect_texlight_materials(mod, None, vrad_exe=vrad)
              == {'LIGHTS/MOD'})


# ─── Instances ───────────────────────────────────────────────────────────────

def _instance_vmf(*files):
    """A VMF with one func_instance entity per file name."""
    body = ''.join(
        f'entity\n{{\n"id" "{300 + i}"\n"classname" "func_instance"\n'
        f'"file" "{name}"\n}}\n' for i, name in enumerate(files))
    return body


def test_instance_closure_resolves_the_way_vbsp_does():
    """Protects: the cache key covers every instance VBSP would pull in --
    beside the map, under the map's 'maps' ancestor folder (with either
    separator), under gameinfo's InstancePath, nested ones resolved against the
    MAIN map, a leading separator joined UNDER the map folder as VBSP joins it
    -- and reports whatever it cannot find or read (then the run does not
    cache) without ever raising, however odd the name."""
    with tempdir() as td:
        td = Path(td)
        game = td / 'game'
        sub = game / 'maps' / 'sub'
        (sub / 'inst').mkdir(parents=True)
        (game / 'maps' / 'shared').mkdir()
        (game / 'instances').mkdir()
        (game / 'gameinfo.txt').write_text('"GameInfo"\n{\n"InstancePath" "instances/"\n}\n')
        input_vmf = sub / 'm.vmf'
        # near: beside the map (extension omitted on purpose); it nests one
        (sub / 'inst' / 'near.vmf').write_text(_instance_vmf('inst/nested.vmf'))
        # nested lives next to the MAIN map's inst folder, not near.vmf's own
        (sub / 'inst' / 'nested.vmf').write_text(_instance_vmf())
        # shared: only under the maps ancestor folder
        (game / 'maps' / 'shared' / 'common.vmf').write_text(_instance_vmf())
        # far: only under gameinfo InstancePath
        (game / 'instances' / 'far.vmf').write_text(_instance_vmf())
        input_text = _instance_vmf('inst/near', 'shared/common.vmf', 'far.vmf',
                                   'ghost.vmf', 'inst/near.vmf')
        root = VMFParser().parse_string(input_text)

        files, unresolved = lmoptimizer._instance_closure(input_vmf, root, game)
        names = sorted(p.name for p in files)
        check("beside-the-map, maps-ancestor, InstancePath and nested instances "
              "are all found", names == ['common.vmf', 'far.vmf', 'near.vmf', 'nested.vmf'],
              f"got {names}")
        check("an instance VBSP could not find is reported (so no cache)",
              unresolved == ['ghost.vmf'], f"got {unresolved}")

        # Separators: either style, and a LEADING one stays under the map folder
        for spelled in ('inst\\near', '\\inst\\near.vmf', '/inst/near',
                        'shared\\common', '\\shared/common.vmf'):
            files, unresolved = lmoptimizer._instance_closure(
                input_vmf, VMFParser().parse_string(_instance_vmf(spelled)), game)
            check(f"{spelled!r} resolves like VBSP's string join",
                  bool(files) and not unresolved,
                  f"files={files} unresolved={unresolved}")

        # Total: no name raises; each simply cannot be found
        odd = ['.', '\\', '/', 'C:', 'C:\\x', '..', '...']
        files, unresolved = lmoptimizer._instance_closure(
            input_vmf, VMFParser().parse_string(_instance_vmf(*odd)), game)

        def same(name):   # '\\' and '/' are the same name to VBSP
            return name.lower().replace('\\', '/')

        check("odd instance names are reported unresolved, never raised",
              files == [] and {same(n) for n in odd} == {same(u) for u in unresolved},
              f"files={files} unresolved={unresolved}")

        # The maps-ancestor rule matches case-insensitively and keeps the
        # folder's own spelling
        upper = td / 'game2' / 'MAPS' / 'sub'
        upper.mkdir(parents=True)
        (upper.parent / 'shared').mkdir()
        (upper.parent / 'shared' / 'common.vmf').write_text(_instance_vmf())
        files, unresolved = lmoptimizer._instance_closure(
            upper / 'm.vmf', VMFParser().parse_string(_instance_vmf('shared/common')),
            game)
        check("a 'MAPS' ancestor folder is matched case-insensitively",
              bool(files) and not unresolved, f"files={files} unresolved={unresolved}")

        # An instance that exists but cannot be parsed is unresolved too
        (sub / 'inst' / 'broken.vmf').write_text('{')
        files, unresolved = lmoptimizer._instance_closure(
            input_vmf, VMFParser().parse_string(_instance_vmf('inst/broken')), game)
        check("an unparsable instance is reported unresolved",
              unresolved == ['inst/broken'], f"got {unresolved}")

        # Parameterised chains (VBSP ReplaceInstancePair): a nested instance's
        # keys, its replace keys included, get the PARENT func_instance's
        # replace values (case-insensitive variable, keys applied in VBSP's
        # epair order), so the grandchild's file comes from both levels.
        def inst(name, *replaces):
            keys = ''.join(f'"replace{i:02d}" "{r}"\n' for i, r in enumerate(replaces, 1))
            return (f'entity\n{{\n"classname" "func_instance"\n"file" "{name}"\n'
                    f'{keys}}}\n')
        (sub / 'inst' / 'mid.vmf').write_text(inst('inst/$Kind', '$leaf $Kind_leaf'))
        (sub / 'inst' / 'door.vmf').write_text(inst('inst/$LEAF'))
        (sub / 'inst' / 'door_leaf.vmf').write_text(_instance_vmf())
        files, unresolved = lmoptimizer._instance_closure(
            input_vmf, VMFParser().parse_string(inst('inst/mid', '$kind door')), game)
        names = sorted(p.name for p in files)
        check("replace keys flow down a nested instance chain",
              names == ['door.vmf', 'door_leaf.vmf', 'mid.vmf'] and not unresolved,
              f"files={names} unresolved={unresolved}")
        # VBSP applies the parent's keys in reverse file order (SetKeyValue
        # prepends): replace02 runs before replace01 here.
        (sub / 'inst' / 'ab.vmf').write_text(_instance_vmf())
        files, unresolved = lmoptimizer._instance_closure(
            input_vmf, VMFParser().parse_string(
                inst('inst/mid', '$x a', '$kind $xb')), game)
        check("replace keys apply in VBSP's epair order",
              sorted(p.name for p in files) == ['ab.vmf', 'mid.vmf'],
              f"files={files} unresolved={unresolved}")
        many = ''.join(
            f'entity\n{{\n"id" "{i}"\n"classname" "func_instance"\n'
            f'"file" "inst/door_leaf"\n"origin" "{i} 0 0"\n}}\n' for i in range(50))
        files, unresolved = lmoptimizer._instance_closure(
            input_vmf, VMFParser().parse_string(many), game, max_expansions=4)
        check("placements differing only in id/origin are one expansion",
              len(files) == 1 and not unresolved,
              f"files={files} unresolved={unresolved}")


# ─── ScratchVMF ──────────────────────────────────────────────────────────────

def test_scratch_vmf_is_private_and_cleans_only_its_own_files():
    """Protects: scratch VMFs sit beside the input (VBSP resolves
    func_instance relative to the VMF) under unique names, and cleanup never
    deletes a file the user made, whatever its name resembles."""
    with tempdir() as td:
        maps = Path(td) / 'maps'
        maps.mkdir()
        anchor = maps / 'm.vmf'
        anchor.write_text('input')
        (maps / 'm.rad').write_text('lights/x 1 1 1 1\n')
        user_files = {name: name.encode() for name in (
            'm_optimized.vmf', 'm_optimized.bsp', 'm_optimized.log', 'm.bsp',
            'm.log', 'm.prt', 'm.lin')}
        for name, data in user_files.items():
            (maps / name).write_bytes(data)

        with vbsp_runner.ScratchVMF(anchor, 'unit') as first, \
                vbsp_runner.ScratchVMF(anchor, 'unit') as second:
            check("scratch VMF is a sibling of the input",
                  first.path.parent == maps and first.path.exists())
            check("scratch names are unique per instance and not the input",
                  first.path != second.path and first.path != anchor)
            first.write(KVNode(name='__root__', children=[KVPair('a', 'b')]))
            check("write() produces a VMF", 'a' in first.path.read_text())
            for ext in ('.bsp', '.prt', '.log', '.lin'):
                first.path.with_suffix(ext).write_bytes(b'compiler product')
            first.mirror_level_rad(maps / 'm.rad')
            rad = first.path.with_suffix('.rad')
            check("the map's level .rad is mirrored under the scratch stem",
                  rad.exists() and rad.read_text() == 'lights/x 1 1 1 1\n')
            first.clear_artifacts()
            check("clear_artifacts removes compiler products only",
                  not any(first.path.with_suffix(e).exists()
                          for e in ('.bsp', '.prt', '.log', '.lin'))
                  and first.path.exists() and rad.exists())
            first.bsp.write_bytes(b'again')
            kept_second = second.path

        check("close removes the VMF, products and mirrored .rad",
              not first.path.exists() and not first.bsp.exists() and not rad.exists())
        check("close leaves nothing of either scratch behind",
              lmopt_leftovers(maps) == [], f"got {lmopt_leftovers(maps)}")
        check("the user's input and level .rad survive",
              anchor.read_text() == 'input' and (maps / 'm.rad').exists())
        check("every user file that merely looks similar survives",
              all((maps / n).read_bytes() == d for n, d in user_files.items()))
        check("second scratch also gone", not kept_second.exists())


def dead_pid():
    """A pid that belonged to a process that has now exited."""
    import subprocess
    proc = subprocess.Popen([sys.executable, '-c', 'pass'])
    proc.wait()
    return proc.pid


def test_host_tag_distinguishes_long_hostnames():
    """Protects: the host stamp encodes the WHOLE hostname (a readable prefix
    plus a hash), so machines whose names only differ late in a long name do
    not share a stamp, and the stamp stays [a-z0-9]."""
    names = ['LAB-GPU-SERVER-PRIMARY-MAPPING-01', 'LAB-GPU-SERVER-PRIMARY-MAPPING-02',
             'lab-gpu-server-primary-mapping-01.example.com', 'ws', 'Ünïcode-Rechner']
    tags = []
    for name in names:
        with mock.patch.object(vbsp_runner.socket, 'gethostname', return_value=name):
            tag = vbsp_runner.host_tag()
            check(f"{name!r}: tag is [a-z0-9]+ and stable",
                  re.fullmatch(r'[a-z0-9]+', tag) and tag == vbsp_runner.host_tag(),
                  f"got {tag!r}")
            tags.append(tag)
    check("hostnames differing only after the 24th character get different stamps",
          len(set(tags)) == len(tags), f"got {tags}")
    clones = []
    for guid in ('0f1e2d3c-aaaa', '0f1e2d3c-bbbb'):
        with mock.patch.object(vbsp_runner.socket, 'gethostname', return_value='ws'), \
             mock.patch.object(vbsp_runner, '_MACHINE_ID', guid):
            clones.append(vbsp_runner.host_tag())
    check("cloned VMs with one hostname but their own machine id get different stamps",
          clones[0] != clones[1], f"got {clones}")


def test_sweep_removes_only_this_tools_dead_stale_files():
    """Protects: scratch left by a hard-killed run (a GUI Cancel skips finally
    blocks) is cleaned by the next run, but ONLY files with the exact names
    this tool creates, made on THIS machine by a process that is gone, and old
    enough that a live run on a shared folder cannot be mistaken for a dead
    one. Hidden temp-looking files of the user's, other machines' files, live
    runs' files and every user file survive; stamped compiler products are
    collected even when their scratch VMF is already gone; an unverifiable pid
    never fails the sweep."""
    with tempdir() as td:
        td = Path(td)
        maps, out, cache = td / 'maps', td / 'out', td / 'maps' / '.lmopt_cache'
        for d in (maps, out, cache):
            d.mkdir(parents=True, exist_ok=True)
        dead, live = dead_pid(), os.getpid()
        host = vbsp_runner.host_tag()
        now = time.time()
        hour_ago = now - 3600

        def make(directory, name, mtime=hour_ago):
            path = directory / name
            path.write_bytes(b'x')
            os.utime(path, (mtime, mtime))
            return name

        def scratch_set(pid, tag, host_name=host, mtime=hour_ago, exts=None):
            stem = f'm.lmopt_baseline_h{host_name}_p{pid}_{tag}'
            return [make(maps, stem + ext, mtime) for ext in
                    (exts or ('.vmf', '.bsp', '.prt', '.log', '.lin', '.rad'))]

        dead_set = scratch_set(dead, 'abcd1234')
        products_only = scratch_set(dead, 'novmf123', exts=('.bsp', '.log', '.prt'))
        live_set = scratch_set(live, 'wxyz9876')
        # The extensionless BSP that pre-#53 compilers wrote for a dotted stem.
        bare_dead = scratch_set(dead, 'bare1234', exts=('',))
        bare_live = scratch_set(live, 'bare5678', exts=('',))
        foreign_set = scratch_set(dead, 'foreign1', host_name='anothermachine')
        fresh_set = scratch_set(dead, 'fresh123', mtime=now)
        huge_pid_set = scratch_set('9' * 30, 'hugepid1')
        user_files = [make(maps, n) for n in (
            'm.vmf', 'm.bsp', 'm.log', 'm.rad', 'm_optimized.bsp',
            f'm.lmopt_notes_{dead}.vmf',
            f'm.lmopt_baseline_h{host}_p{dead}_short.vmf',
            f'm.lmopt_baseline_{dead}_abcd1234.vmf')]   # the pre-host name shape

        dead_tmp = make(out, f'.m_optimized.vmf.lmopt_h{host}_p{dead}.tmp')
        live_tmp = make(out, f'.m_optimized.vmf.lmopt_h{host}_p{live}.tmp')
        foreign_tmp = make(out, f'.m_optimized.vmf.lmopt_hanothermachine_p{dead}.tmp')
        fresh_tmp = make(out, f'.fresh.vmf.lmopt_h{host}_p{dead}.tmp', now)
        hidden_user_files = [make(out, n) for n in (
            '.notes.999999.tmp', f'.lightmap_backup.{dead}.tmp', '.hidden_notes.vmf',
            f'm_optimized.vmf.lmopt_h{host}_p{dead}.tmp')]   # not hidden: a cache-style name

        dead_cache_tmp = make(cache, f'm.bsp.lmopt_h{host}_p{dead}.tmp')
        cache_keep = [make(cache, n) for n in (
            'm.manifest.json', f'.m.bsp.lmopt_h{host}_p{dead}.tmp',
            f'm.abc.lmopt_hanothermachine_p{dead}.tmp')]

        vbsp_runner.sweep_stale_scratch(maps, 'scratch')
        vbsp_runner.sweep_stale_scratch(out, 'publish')
        vbsp_runner.sweep_stale_scratch(cache, 'cache')

        def survives(directory, names):
            return all((directory / n).exists() for n in names)

        check("a dead local run's scratch VMF, products and mirrored .rad are removed",
              not any((maps / n).exists() for n in dead_set))
        check("stamped compiler products whose scratch VMF is gone are removed too",
              not any((maps / n).exists() for n in products_only))
        check("a dead run's lone extensionless BSP is removed; a live run's survives",
              not any((maps / n).exists() for n in bare_dead)
              and survives(maps, bare_live))
        check("a dead local run's publish temp and cache temp are removed",
              not (out / dead_tmp).exists() and not (cache / dead_cache_tmp).exists())
        check("a live run's scratch and temps survive",
              survives(maps, live_set) and (out / live_tmp).exists())
        check("another machine's files survive (its pid means nothing here)",
              survives(maps, foreign_set) and (out / foreign_tmp).exists())
        check("a dead run's files too young to be sure about survive",
              survives(maps, fresh_set) and (out / fresh_tmp).exists())
        check("a name whose pid cannot be checked survives (and nothing raised)",
              survives(maps, huge_pid_set))
        check("hidden temp-looking user files and lookalike names survive",
              survives(out, hidden_user_files) and survives(cache, cache_keep))
        check("user files and near-miss scratch names survive",
              survives(maps, user_files))

        # The default age threshold is what protects the fresh files: with it
        # disabled they are collected (so the threshold is doing the work)
        vbsp_runner.sweep_stale_scratch(maps, 'scratch', min_age=0)
        check("with the age threshold off, the fresh dead set is collected",
              not any((maps / n).exists() for n in fresh_set))

        # The names the tool REALLY creates are the ones the sweep recognises
        # (simulate a hard kill: never close, owner pid gone, files old)
        with mock.patch.object(vbsp_runner, 'pid_alive', return_value=False):
            killed = vbsp_runner.ScratchVMF(maps / 'm.vmf', 'baseline')
            real = [killed.path] + [killed.path.with_suffix(e) for e in
                                    ('.bsp', '.prt', '.log', '.lin')]
            real += [out / vbsp_runner.atomic_tmp_name('m_optimized.vmf', hidden=True),
                     cache / vbsp_runner.atomic_tmp_name('m.bsp', hidden=False)]
            for path in real:
                path.write_bytes(b'x')
                os.utime(path, (hour_ago, hour_ago))
            vbsp_runner.sweep_stale_scratch(maps, 'scratch')
            vbsp_runner.sweep_stale_scratch(out, 'publish')
            vbsp_runner.sweep_stale_scratch(cache, 'cache')
        check("scratch, publish temp and cache temp as really named are swept",
              not any(path.exists() for path in real),
              f"left: {[p.name for p in real if p.exists()]}")

    check("pid_alive never raises for a pid outside the OS range",
          vbsp_runner.pid_alive(10 ** 30) is True and vbsp_runner.pid_alive(-5) is True)


# ─── Baseline scan ───────────────────────────────────────────────────────────

def drive_baseline_scan(td, behaviour):
    """Run _find_safe_baseline with the real count_vertices signature stubbed.
    behaviour(scale_index) returns a VertexCount or raises."""
    maps = Path(td) / 'maps'
    maps.mkdir(exist_ok=True)
    input_vmf = maps / 'm.vmf'
    input_vmf.write_text(MINI_VMF)
    root = VMFParser().parse_string(MINI_VMF)
    side_map = lmoptimizer._build_vmf_side_map(root)
    calls = []

    def fake_count(vbsp_exe, vmf_path, game_dir, **kw):
        calls.append(Path(vmf_path).parent)
        return behaviour(len(calls))

    args = types.SimpleNamespace(binroot=None)
    outcome = types.SimpleNamespace(calls=calls, result=None, error=None, maps=maps)
    with mock.patch.object(vbsp_runner, 'count_vertices', autospec=True,
                           side_effect=fake_count):
        try:
            outcome.result = lmoptimizer._find_safe_baseline(
                root, args, Path(td) / 'vbsp.exe', Path(td), input_vmf,
                vertex_budget=62000, eligible_sids=[(1, 9)], side_map=side_map)
        except Exception as e:  # noqa: BLE001 - the test inspects the type
            outcome.error = e
    return outcome


def counts(n):
    return vbsp_runner.VertexCount(n, False, n, False, 10, False)


def test_baseline_scan_reports_environment_failures_as_such():
    """Protects: only VBSP's own capacity rejections mean 'try a coarser
    scale'; a missing VBSP, a crash, a timeout or any other VBSP failure
    aborts at once instead of being retried 64 times and blamed on 'too many
    brushes'. Trial VMFs are siblings of the input so func_instance resolves;
    nothing is left behind."""
    with tempdir() as td:
        def reject_then_fit(call):
            if call <= 3:
                raise vbsp_runner.VBSPRejectedError('Too many edges in map')
            return counts(1000)
        run = drive_baseline_scan(td, reject_then_fit)
        check("capacity rejections at fine scales mean 'too fine': fit at 4",
              run.result == 4 and run.error is None,
              f"got {run.result} / {run.error!r}")
        check("every trial VMF is compiled from the input's directory",
              run.calls and all(p == run.maps for p in run.calls),
              f"got {run.calls}")
        check("no scratch left behind", lmopt_leftovers(run.maps) == [],
              f"got {lmopt_leftovers(run.maps)}")

    for what, error in (("missing VBSP", vbsp_runner.VBSPNotFoundError('gone')),
                        ("VBSP timeout", TimeoutError('slow')),
                        ("unparsable output", vbsp_runner.VBSPParseError('?')),
                        ("a VBSP crash or unreadable VMF",
                         vbsp_runner.VBSPCompileError('exited with code 3221225477'))):
        with tempdir() as td:
            def boom(call, error=error):
                raise error
            run = drive_baseline_scan(td, boom)
            check(f"{what} aborts on the first call",
                  run.error is error and len(run.calls) == 1,
                  f"calls={len(run.calls)} error={run.error!r}")

    with tempdir() as td:
        def always_reject(call):
            raise vbsp_runner.VBSPRejectedError('Too many edges in map')
        run = drive_baseline_scan(td, always_reject)
        check("VBSP rejecting every scale surfaces the rejection, not 'nothing fits'",
              isinstance(run.error, vbsp_runner.VBSPRejectedError),
              f"got {run.result} / {run.error!r}")

    with tempdir() as td:
        run = drive_baseline_scan(td, lambda call: counts(10 ** 9))
        check("a map that never fits returns 0",
              run.result == 0, f"got {run.result}")


def test_count_vertices_separates_scale_rejections_from_other_failures():
    """Protects: the discriminator the baseline scan relies on. VBSP's own
    capacity Error() messages become VBSPRejectedError; every other nonzero
    exit (an instance file it cannot open, a crash code) stays a plain
    VBSPCompileError, so a broken setup is never read as 'try a coarser
    scale'."""
    with tempdir() as td:
        td = Path(td)
        vmf = td / 'm.vmf'
        vmf.write_text(MINI_VMF)
        game = td / 'game'
        game.mkdir()

        cases = [
            ("Too many edges", 'Too many edges in map, max == 32768', 1, True),
            ("t-junction edge overflow",
             'Edge with too many vertices due to t-junctions.  Max 64', 1, True),
            ("Bad surface extents (only a full compile can raise it)",
             'Bad surface extents - surface is too big', 1, False),
            ("an unreadable instance file", 'Could not open instance file x.vmf', 1, False),
        ]
        if os.name == 'nt':
            # A Windows crash code with a rejection-looking message is still a crash
            cases.append(("a crash exit code", 'Too many edges in map', -1073741819, False))
        for label, text, code, rejected in cases:
            tool = write_stub_tool(td, 'vbsp_stub', exit_code=code, text=text)
            error = None
            try:
                vbsp_runner.count_vertices(tool, vmf, game)
            except vbsp_runner.VBSPError as e:
                error = e
            check(f"{label}: {'is' if rejected else 'is not'} a scale rejection",
                  isinstance(error, vbsp_runner.VBSPRejectedError) == rejected
                  and isinstance(error, vbsp_runner.VBSPCompileError),
                  f"got {error!r}")

        ok = write_stub_tool(td, 'vbsp_ok', exit_code=0,
                             text='VERTCOUNT: 1234\nLEAFFACECOUNT: 5\nFACECOUNT: 6')
        counted = vbsp_runner.count_vertices(ok, vmf, game)
        check("a normal -countverts run is parsed",
              (counted.count, counted.leafface_count, counted.face_count) == (1234, 5, 6),
              f"got {counted}")


# ─── Solver writes ───────────────────────────────────────────────────────────

def test_solver_writes_scales_to_sides_without_the_key():
    """Protects: VMFs whose sides omit lightmapscale (script-generated or
    third-party exports) still receive the solver's scales — KVNode.set_property
    silently no-ops on a missing key, which left the output unchanged while
    the log claimed success."""
    def keyless(sid):
        return KVNode(name='side', children=[KVPair('id', str(sid)),
                                             KVPair('material', 'CONCRETE/FLOOR')])

    def extent(sid):
        return FaceExtent(side_id=sid, extent_s=64.0, extent_t=64.0, s_min=0, s_max=64,
                          t_min=0, t_max=64, s_axis=(1, 0, 0), t_axis=(0, 1, 0),
                          lightmapscale=16, material='CONCRETE/FLOOR',
                          num_vertices=4, brush_entity='')

    with tempdir() as td:
        nodes = {1: keyless(1), 2: keyless(2)}
        root = KVNode(name='__root__', children=list(nodes.values()))
        with mock.patch.object(vbsp_runner, 'count_vertices', autospec=True,
                               return_value=counts(100)):
            result = vbsp_solver.solve_with_vbsp(
                face_extents=[extent(1), extent(2)], lighting_data={},
                vmf_root=root, side_map=nodes, vbsp_exe=Path(td) / 'vbsp.exe',
                game_dir=Path(td), input_vmf=Path(td) / 'm.vmf',
                vertex_budget=65536, detail_type_materials=set(),
                detail_min_scale=5,
                side_min_scales={2: 6})
        check("side 1 received the solver's scale",
              nodes[1].get_property('lightmapscale') == '1',
              f"got {nodes[1].get_property('lightmapscale')}")
        check("a per-side floor from the caller is honoured",
              nodes[2].get_property('lightmapscale') == '6'
              and result.scales[2] == 6,
              f"got {nodes[2].get_property('lightmapscale')} / {result.scales}")
        check("the solver leaves no scratch beside the input",
              lmopt_leftovers(td) == [], f"got {lmopt_leftovers(td)}")


def test_solver_tolerates_only_capacity_rejections():
    """Protects: inside the vertex-budget solver, only VBSP's own capacity
    rejections mean 'try a coarser scale'; a crash or other VBSP failure
    surfaces at once instead of becoming a bogus 'doesn't fit' result."""
    def keyless(sid):
        return KVNode(name='side', children=[KVPair('id', str(sid)),
                                             KVPair('material', 'CONCRETE/FLOOR')])

    def extent(sid):
        return FaceExtent(side_id=sid, extent_s=64.0, extent_t=64.0, s_min=0, s_max=64,
                          t_min=0, t_max=64, s_axis=(1, 0, 0), t_axis=(0, 1, 0),
                          lightmapscale=16, material='CONCRETE/FLOOR',
                          num_vertices=4, brush_entity='')

    def solve(td, behaviour):
        nodes = {1: keyless(1), 2: keyless(2)}
        root = KVNode(name='__root__', children=list(nodes.values()))
        calls = []

        def fake_count(vbsp_exe, vmf_path, game_dir, **kw):
            calls.append(vmf_path)
            return behaviour(len(calls))

        outcome = types.SimpleNamespace(result=None, error=None, calls=calls)
        with mock.patch.object(vbsp_runner, 'count_vertices', autospec=True,
                               side_effect=fake_count):
            try:
                outcome.result = vbsp_solver.solve_with_vbsp(
                    face_extents=[extent(1), extent(2)], lighting_data={},
                    vmf_root=root, side_map=nodes, vbsp_exe=Path(td) / 'vbsp.exe',
                    game_dir=Path(td), input_vmf=Path(td) / 'm.vmf',
                    vertex_budget=65536, detail_type_materials=set(),
                    detail_min_scale=5)
            except vbsp_runner.VBSPError as e:
                outcome.error = e
        return outcome

    def rejected_twice_then_fits(call):
        if call <= 2:
            raise vbsp_runner.VBSPRejectedError('Too many edges in map')
        return counts(1000)

    with tempdir() as td:
        run = solve(td, rejected_twice_then_fits)
        check("capacity rejections at fine scales are skipped: baseline 3",
              run.error is None and run.result.baseline_scale == 3,
              f"error={run.error!r}")

    with tempdir() as td:
        def crashes(call):
            raise vbsp_runner.VBSPCompileError('exited with code 3221225477')
        run = solve(td, crashes)
        check("a VBSP crash surfaces at once",
              isinstance(run.error, vbsp_runner.VBSPCompileError)
              and not isinstance(run.error, vbsp_runner.VBSPRejectedError)
              and len(run.calls) == 1, f"calls={len(run.calls)} error={run.error!r}")


# ─── compile_rad ─────────────────────────────────────────────────────────────

def write_stub_tool(directory, name, exit_code=0, sleep_seconds=0, text=''):
    """A stand-in compiler executable (a real process, so timeouts are real).
    `text` is printed line by line ('\\n' separates lines)."""
    printed = (text or 'stub').split('\n')
    if os.name == 'nt':
        path = Path(directory) / (name + '.bat')
        lines = ['@echo off'] + [f'echo {line}' for line in printed]
        if sleep_seconds:
            lines.append(f'ping -n {sleep_seconds + 1} 127.0.0.1 >nul')
        lines.append(f'exit /b {exit_code}')
        path.write_text('\r\n'.join(lines) + '\r\n')
    else:
        path = Path(directory) / name
        # printf with a quoted literal: parentheses and slashes in the text
        # must not reach the shell's parser
        lines = ['#!/bin/sh'] + [
            "printf '%s\\n' '" + line.replace("'", "'\\''") + "'" for line in printed]
        if sleep_seconds:
            lines.append(f'sleep {sleep_seconds}')
        lines.append(f'exit {exit_code}')
        path.write_text('\n'.join(lines) + '\n')
        path.chmod(0o755)
    return path


def test_compile_rad_enforces_timeout_and_exit_code():
    """Protects: a hung VRAD times out instead of blocking the run forever,
    and a nonzero VRAD exit is a failure even though VBSP's BSP is still on
    disk (it used to return success whenever the file existed). Also that the
    texture-shadow level reaches VRAD as exactly its own flag: the default
    command carries -textureshadows once, and no level adds another level's."""
    with tempdir() as td:
        td = Path(td)
        game = td / 'game'
        game.mkdir()
        bsp = td / 'm.bsp'
        bsp.write_bytes(b'VBSP written by VBSP, long before VRAD ran')

        failing = write_stub_tool(td, 'vrad_fail', exit_code=1,
                                  text='Too many lights (39027 / 32767)')
        raised = None
        try:
            vrad_runner.compile_rad(failing, bsp, game, timeout=60, max_retries=1)
        except vrad_runner.VRADError as e:
            raised = e
        check("nonzero exit raises VRADCompileError although the BSP exists",
              isinstance(raised, vrad_runner.VRADCompileError), f"got {raised!r}")
        check("the error carries VRAD's own output",
              raised is not None and 'Too many lights' in str(raised))

        ok = write_stub_tool(td, 'vrad_ok', exit_code=0)
        check("exit 0 returns the BSP path",
              vrad_runner.compile_rad(ok, bsp, game, timeout=60) == bsp.resolve())

        hung = write_stub_tool(td, 'vrad_hung', exit_code=0, sleep_seconds=30)
        started = time.monotonic()
        timed_out = False
        try:
            vrad_runner.compile_rad(hung, bsp, game, timeout=1, max_retries=1)
        except TimeoutError:
            timed_out = True
        elapsed = time.monotonic() - started
        check("a hung VRAD raises TimeoutError", timed_out)

        # The command line VRAD receives, per texture-shadow level
        def vrad_args(**kw):
            seen = []

            def capture(cmd, **_):
                seen.append(list(cmd))
                return 0, []
            with mock.patch.object(vrad_runner, '_run_streaming', capture):
                vrad_runner.compile_rad(ok, bsp, game, **kw)
            return seen[0]

        # VRAD's own spellings, hard-coded on purpose: a typo in the runner's
        # table must not be invisible to the test that reads the same table.
        expected_flags = {'none': [], 'props': ['-textureshadows'],
                          'world': ['-worldtextureshadows'],
                          'translucent': ['-translucentshadows'],
                          'all': ['-alltextureshadows']}
        shadow_flags = {f for flags in expected_flags.values() for f in flags}
        default_cmd = vrad_args()
        check("the default command carries -textureshadows and no other level's flag",
              [f for f in shadow_flags if f in default_cmd] == ['-textureshadows'],
              f"cmd={default_cmd}")
        for level, flags in expected_flags.items():
            cmd = vrad_args(texture_shadows=level)
            present = [f for f in sorted(shadow_flags) if f in cmd]
            check(f"level '{level}' passes exactly {flags or 'no texture-shadow flag'}",
                  present == sorted(flags), f"present={present}")
        check("-staticproppolys survives every level (it is not a shadow flag)",
              all('-staticproppolys' in vrad_args(texture_shadows=lv)
                  for lv in expected_flags))
        bad_level = None
        try:
            vrad_args(texture_shadows='everything')
        except ValueError as e:
            bad_level = e
        check("an unknown texture-shadow level is rejected",
              bad_level is not None, "no ValueError")
        # The stub's child (ping/sleep) holds the output pipe; only a real
        # process-TREE kill lets the reader finish at once. A root-only kill
        # leaves the reader blocked for its full 5 s join.
        check("...promptly: the whole process tree is killed, not just the root",
              elapsed < 5, f"took {elapsed:.1f}s")


# ─── main(): failure paths ───────────────────────────────────────────────────

class Workspace:
    """A temp tree shaped like a real setup, with stub tool paths."""

    def __init__(self, td):
        td = Path(td)
        self.root = td
        self.game = td / 'root' / 'game'
        self.game.mkdir(parents=True)
        (self.game / 'gameinfo.txt').write_text('"GameInfo"{}')
        (td / 'root' / 'bin' / 'x64').mkdir(parents=True)
        self.tools = td / 'tools'
        self.tools.mkdir()
        for name in ('vbsp.exe', 'vrad.exe', 'vvis.exe', 'bspwalk.exe'):
            (self.tools / name).write_bytes(b'stub')
        self.maps = td / 'maps'
        self.maps.mkdir()
        self.vmf = self.maps / 'm.vmf'
        self.vmf.write_text(MINI_VMF)
        # What a previous run / the user's own compile left behind
        self.output = self.maps / 'm_optimized.vmf'
        self.output.write_text(PREVIOUS_OUTPUT)
        self.user_bsp = self.maps / 'm_optimized.bsp'
        self.user_bsp.write_bytes(b'USER COMPILE')
        self.user_log = self.maps / 'm_optimized.log'
        self.user_log.write_bytes(b'USER LOG')

    def argv(self, *extra, tools=('vbsp', 'vrad', 'vvis'), game=True):
        argv = [str(self.vmf)]
        for t in tools:
            argv += [f'--{t}', str(self.tools / f'{t}.exe')]
        if game:
            argv += ['--game', str(self.game)]
        return argv + list(extra)

    def untouched(self):
        return (self.output.read_text() == PREVIOUS_OUTPUT
                and self.user_bsp.read_bytes() == b'USER COMPILE'
                and self.user_log.read_bytes() == b'USER LOG')


def stub_compilers(stack, *, vrad_compile=None, vvis_ok=True):
    """Patch the external compilers with autospecced stand-ins. Returns the
    mocks (count_vertices, compile_bsp, compile_rad)."""
    def fake_compile_bsp(vbsp_exe, vmf_path, game_dir, **kw):
        bsp = Path(vmf_path).with_suffix('.bsp')
        write_bsp(bsp)               # VBSP output: no lighting yet
        return bsp

    count = stack.enter_context(mock.patch.object(
        vbsp_runner, 'count_vertices', autospec=True, return_value=counts(1000)))
    compile_bsp = stack.enter_context(mock.patch.object(
        vbsp_runner, 'compile_bsp', autospec=True, side_effect=fake_compile_bsp))
    compile_rad = stack.enter_context(mock.patch.object(
        vrad_runner, 'compile_rad', autospec=True,
        side_effect=vrad_compile or (lambda vrad_exe, bsp_path, *a, **k: Path(bsp_path))))
    stack.enter_context(mock.patch.object(
        vrad_runner, 'count_lights', autospec=True,
        return_value=vrad_runner.LightCount(10, False)))
    stack.enter_context(mock.patch.object(
        lmoptimizer, '_run_vvis_fast', autospec=True, return_value=vvis_ok))
    return count, compile_bsp, compile_rad


def run_failing_auto_compile(vrad_compile=None, vvis_ok=True):
    """Auto-compile where something fails; returns (exit code, workspace-state)."""
    import contextlib
    with tempdir() as td, contextlib.ExitStack() as stack:
        ws = Workspace(td)
        stub_compilers(stack, vrad_compile=vrad_compile, vvis_ok=vvis_ok)
        code = run_main(ws.argv())
        cache_entry = (ws.maps / '.lmopt_cache' / 'm.manifest.json').exists()
        return types.SimpleNamespace(
            code=code, untouched=ws.untouched(), leftovers=lmopt_leftovers(ws.maps),
            cache_entry=cache_entry, input_intact=ws.vmf.read_text() == MINI_VMF)


def test_failed_auto_compile_leaves_previous_output_untouched():
    """Protects: a failing run exits nonzero, leaves a previous good
    <map>_optimized.vmf and the user's own compile products alone, leaves no
    scratch, and never caches an unlit baseline (the 2026-11-04 poisoning)."""
    def vrad_dies(vrad_exe, bsp_path, *a, **k):
        raise vrad_runner.VRADCompileError('VRAD exited with code 1: Too many lights')

    run = run_failing_auto_compile(vrad_compile=vrad_dies)
    check("VRAD failure exits nonzero", run.code == 1, f"got {run.code}")
    check("VRAD failure: previous output and user's compile untouched", run.untouched)
    check("VRAD failure: no scratch files left", run.leftovers == [],
          f"got {run.leftovers}")
    check("VRAD failure: nothing cached", not run.cache_entry)
    check("VRAD failure: the input VMF is intact", run.input_intact)

    # VRAD 'succeeds' but the BSP has no lightmap samples: must abort and must
    # not be cached as a valid baseline
    run = run_failing_auto_compile()
    check("unlit baseline after a clean VRAD exit exits nonzero", run.code == 1,
          f"got {run.code}")
    check("unlit baseline: previous output and user's compile untouched",
          run.untouched)
    check("unlit baseline: no scratch files left", run.leftovers == [],
          f"got {run.leftovers}")
    check("unlit baseline: NOT cached", not run.cache_entry)

    # VVIS fails but VRAD would happily light the BSP: only the VVIS failure
    # itself can stop this run from caching a no-bounce baseline as valid
    def vrad_lights_it(vrad_exe, bsp_path, *a, **k):
        write_bsp(Path(bsp_path), ldr_len=4096)
        return Path(bsp_path)

    run = run_failing_auto_compile(vrad_compile=vrad_lights_it, vvis_ok=False)
    check("a failed VVIS exits nonzero (no degraded baseline)", run.code == 1,
          f"got {run.code}")
    check("failed VVIS: previous output untouched, no scratch, nothing cached",
          run.untouched and run.leftovers == [] and not run.cache_entry)


def test_dry_run_in_auto_compile_mode_does_nothing():
    """Protects: --dry-run never compiles, caches or writes in auto-compile
    mode (it used to compile, cache and overwrite the output)."""
    import contextlib
    with tempdir() as td, contextlib.ExitStack() as stack:
        ws = Workspace(td)
        count, compile_bsp, compile_rad = stub_compilers(stack)
        code = run_main(ws.argv('--dry-run'))
        check("dry-run without a cached baseline is rejected", code == 1,
              f"got {code}")
        check("dry-run: no compiler was run",
              not count.called and not compile_bsp.called and not compile_rad.called)
        check("dry-run: no cache directory or scratch created",
              not (ws.maps / '.lmopt_cache').exists() and lmopt_leftovers(ws.maps) == [])
        check("dry-run: previous output and user's compile untouched", ws.untouched())


def test_unlit_bsp_is_an_error_unless_allowed():
    """Protects: --bsp with a BSP VRAD never lit (or one this tool cannot
    decode) is rejected before any lightmap extraction, not warned about and
    optimized into meaningless scales with exit 0."""
    import contextlib
    with tempdir() as td, contextlib.ExitStack() as stack:
        ws = Workspace(td)
        unlit = ws.maps / 'unlit.bsp'
        write_bsp(unlit)
        lzma = ws.maps / 'lzma.bsp'
        write_bsp(lzma, ldr_len=4096, ldr_comp=999)
        reader_cls = stack.enter_context(mock.patch.object(bsp_reader, 'BSPReader'))
        reader_cls.return_value.read.side_effect = ValueError('stop here')

        for name, path in (("unlit", unlit), ("LZMA-compressed", lzma)):
            reader_cls.reset_mock()
            code = run_main(ws.argv('--bsp', str(path), '-o', str(ws.output)))
            check(f"{name} --bsp exits nonzero", code == 1, f"got {code}")
            check(f"{name} --bsp never reaches lightmap extraction",
                  not reader_cls.called)
            check(f"{name} --bsp leaves the previous output untouched", ws.untouched())

        reader_cls.reset_mock()
        code = run_main(ws.argv('--bsp', str(unlit), '--allow-unlit'))
        check("--allow-unlit passes the gate", reader_cls.called)


def test_bspwalk_failure_in_bsp_mode_exits_nonzero():
    """Protects: a bspwalk failure (missing exe, nonzero exit, no spawn
    point...) in direct --bsp mode exits nonzero with no output; it used to
    print ERROR, write nothing and exit 0, and the GUI showed 'Done'."""
    import contextlib
    with tempdir() as td, contextlib.ExitStack() as stack:
        ws = Workspace(td)
        lit = ws.maps / 'lit.bsp'
        write_bsp(lit, ldr_len=4096)
        reader_cls = stack.enter_context(mock.patch.object(bsp_reader, 'BSPReader'))
        reader = reader_cls.return_value
        reader.read_vertex_count.return_value = 1000
        reader.get_bsp_lightmapscale.return_value = 16
        reader.extract_all_face_data.return_value = []
        reader.face_side_ids_version.return_value = 0
        stack.enter_context(mock.patch.object(
            bsp_reader, 'match_bsp_to_vmf',
            return_value={1: FaceLightmapData(vmf_side_id=1, material='M',
                                              bsp_face_indices=[0],
                                              luminances=[1.0, 2.0])}))
        walk = stack.enter_context(mock.patch.object(
            bspwalk_runner, 'run_bspwalk',
            side_effect=bspwalk_runner.BspwalkError('bspwalk exited 1: no usable seed')))
        code = run_main(ws.argv('--bsp', str(lit), '--visibility-check',
                                '--bspwalk', str(ws.tools / 'bspwalk.exe'),
                                '--bspwalk-timeout', '12.5',
                                '--bspwalk-tiers', 'a', '--bspwalk-args',
                                '-dumpobj "C:/My Maps/x"',
                                '--binroot', str(ws.root / 'root'),
                                '-o', str(ws.output)))
        check("bspwalk was actually invoked", walk.called)
        kwargs = walk.call_args.kwargs if walk.called else {}
        check("--bspwalk-timeout reaches run_bspwalk", kwargs.get('timeout') == 12.5,
              f"got {kwargs.get('timeout')!r}")
        check("--binroot reaches run_bspwalk",
              kwargs.get('bin_root') == str(ws.root / 'root'),
              f"got {kwargs.get('bin_root')!r}")
        check("tiers and extra args are handed over for run_bspwalk to parse",
              kwargs.get('tiers') == 'a'
              and kwargs.get('extra_args') == '-dumpobj "C:/My Maps/x"',
              f"got {kwargs.get('tiers')!r} / {kwargs.get('extra_args')!r}")
        check("bspwalk failure exits nonzero", code == 1, f"got {code}")
        check("bspwalk failure writes nothing", ws.untouched())


def test_bad_setups_are_rejected_before_any_work():
    """Protects: missing VBSP/VVIS/VRAD/bspwalk, a missing --game, bad tiers
    and mistyped tool paths are all reported up front (exit 1, before the
    pipeline starts) instead of after minutes of compiling or as a TypeError
    traceback."""
    with tempdir() as td:
        ws = Workspace(td)
        lit = ws.maps / 'lit.bsp'
        write_bsp(lit, ldr_len=4096)
        ghost = str(ws.tools / 'does_not_exist.exe')

        with mock.patch.object(lmoptimizer, '_run_pipeline', return_value=0) as pipeline, \
                mock.patch.object(lmoptimizer, '_try_find_vrad', return_value=None), \
                mock.patch.object(lmoptimizer, '_try_find_vbsp', return_value=None):
            check("control: a complete auto-compile setup reaches the pipeline",
                  run_main(ws.argv()) == 0 and pipeline.call_count == 1)
            pipeline.reset_mock()
            check("control: 0 (no limit) and a finite --bspwalk-timeout are fine",
                  run_main(ws.argv('--bspwalk-timeout', '0')) == 0
                  and run_main(ws.argv('--bspwalk-timeout', '30.5')) == 0)
            pipeline.reset_mock()
            check("control: an output folder that does not exist yet is fine",
                  run_main(ws.argv('-o', str(ws.root / 'new' / 'dir' / 'out.vmf'))) == 0
                  and pipeline.call_count == 1)

            # VBSP in a folder with no VVIS beside it (auto-detection looks there)
            lonely = ws.root / 'lonely'
            lonely.mkdir()
            (lonely / 'vbsp.exe').write_bytes(b'stub')
            cases = [
                ("auto-compile without VVIS",
                 [str(ws.vmf), '--vbsp', str(lonely / 'vbsp.exe'),
                  '--vrad', str(ws.tools / 'vrad.exe'), '--game', str(ws.game)]),
                ("auto-compile without VRAD", ws.argv(tools=('vbsp', 'vvis'))),
                ("auto-compile without VBSP", ws.argv(tools=('vrad', 'vvis'))),
                ("auto-compile without --game", ws.argv(game=False)),
                ("a mistyped --vbsp path", ws.argv('--vbsp', ghost)),
                ("a mistyped --vrad path with --bsp",
                 ws.argv('--bsp', str(lit), '--vrad', ghost)),
                ("a missing --bsp file",
                 ws.argv('--bsp', str(ws.maps / 'nope.bsp'))),
                ("--visibility-check without --game",
                 ws.argv('--bsp', str(lit), '--visibility-check',
                         '--bspwalk', str(ws.tools / 'bspwalk.exe'), game=False)),
                ("--visibility-check with a missing bspwalk.exe",
                 ws.argv('--bsp', str(lit), '--visibility-check', '--bspwalk', ghost)),
                ("a negative --bspwalk-timeout",
                 ws.argv('--bsp', str(lit), '--bspwalk-timeout', '-5')),
                ("a NaN --bspwalk-timeout",
                 ws.argv('--bsp', str(lit), '--bspwalk-timeout', 'nan')),
                ("an infinite --bspwalk-timeout",
                 ws.argv('--bsp', str(lit), '--bspwalk-timeout', 'inf')),
                ("an output folder that cannot be written (its parent is a file)",
                 ws.argv('-o', str(ws.tools / 'vbsp.exe' / 'out.vmf'))),
                ("a --game folder without gameinfo.txt",
                 ws.argv('--game', str(ws.root / 'tools'))),
                ("--bspwalk-args with an unterminated quote",
                 ws.argv('--bsp', str(lit), '--visibility-check',
                         '--bspwalk', str(ws.tools / 'bspwalk.exe'),
                         '--bspwalk-args', '-dumpobj "unterminated')),
                ("--bspwalk-tiers with an unknown tier",
                 ws.argv('--bsp', str(lit), '--visibility-check',
                         '--bspwalk', str(ws.tools / 'bspwalk.exe'),
                         '--bspwalk-tiers', 'C')),
            ]
            for name, argv in cases:
                pipeline.reset_mock()
                code = run_main(argv)
                check(f"{name}: exit 1 before the pipeline starts",
                      code == 1 and not pipeline.called,
                      f"code={code} pipeline_called={pipeline.called}")
            check("no case touched the previous output", ws.untouched())


def _side(side_id, material, scale='1'):
    return KVNode(name='side', children=[
        KVPair('id', str(side_id)), KVPair('material', material),
        KVPair('lightmapscale', scale)])


def drive_enforcement(td, count_for, *, texlights=True, vvis_ok=True,
                      compile_error=None):
    """Run the post-solver light enforcement against autospecced compile_bsp /
    count_lights. count_for(root) -> the light count for the current tree."""
    maps = Path(td) / 'maps'
    maps.mkdir(exist_ok=True)
    anchor = maps / 'm.vmf'
    anchor.write_text('input')
    game = Path(td) / 'game'
    game.mkdir(exist_ok=True)
    if texlights:
        (game / 'lights.rad').write_text('lights/white001 255 255 255 400\n')
    sides = [_side(1, 'lights/white001'), _side(2, 'lights/white001'),
             _side(3, 'concrete/wall')]
    root = KVNode(name='__root__', children=sides)
    out = types.SimpleNamespace(compiled=[], ok=None, root=root, maps=maps)

    def fake_compile(vbsp_exe, vmf_path, game_dir, **kw):
        if compile_error:
            raise compile_error
        out.compiled.append(Path(vmf_path))
        bsp = Path(vmf_path).with_suffix('.bsp')
        bsp.write_bytes(b'x')
        return bsp

    with mock.patch.object(vbsp_runner, 'compile_bsp', autospec=True,
                           side_effect=fake_compile), \
            mock.patch.object(vrad_runner, 'count_lights', autospec=True,
                              side_effect=lambda *a, **k: vrad_runner.LightCount(
                                  count_for(root), False)), \
            mock.patch.object(lmoptimizer, '_run_vvis_fast', autospec=True,
                              return_value=vvis_ok):
        out.ok = lmoptimizer._run_light_budget_enforcement(
            root=root, side_map=lmoptimizer._build_vmf_side_map(root),
            anchor_vmf=anchor, vbsp_exe=Path(td) / 'vbsp.exe',
            vrad_exe=Path(td) / 'vrad.exe', vvis_exe=Path(td) / 'vvis.exe',
            game_dir=game, bin_root=None, lights_rad=None, level_rad=None,
            args=types.SimpleNamespace(light_budget=32767, verbose=False))
    return out


def scale_at(root, index):
    return int(root.children[index].get_property('lightmapscale'))


def test_enforcement_finds_the_smallest_fitting_scale_or_reports_failure():
    """Protects: the post-solver light stage settles on the SMALLEST texlight
    scale that fits the budget (a rung-only ladder overshot 5 -> 6 and 13 ->
    16), reports every failure as False (so main exits nonzero), and compiles
    scratch beside the input, leaving nothing behind."""
    for needed in (2, 5, 9, 12, 13):
        with tempdir() as td:
            run = drive_enforcement(
                td, lambda r, needed=needed: 50_000
                if min(scale_at(r, 0), scale_at(r, 1)) < needed else 30_000)
            check(f"needs {needed}: a fitting scale is found",
                  run.ok is True, f"got {run.ok}")
            check(f"needs {needed}: emissive faces end exactly there",
                  scale_at(run.root, 0) == needed and scale_at(run.root, 1) == needed,
                  f"got {scale_at(run.root, 0)}, {scale_at(run.root, 1)}")
            check(f"needs {needed}: the non-emissive face is untouched",
                  scale_at(run.root, 2) == 1)
            check(f"needs {needed}: every compile was a scratch sibling of the input",
                  all(p.parent == run.maps for p in run.compiled))
            check(f"needs {needed}: no scratch left behind",
                  lmopt_leftovers(run.maps) == [], f"got {lmopt_leftovers(run.maps)}")

    with tempdir() as td:
        run = drive_enforcement(td, lambda r: 100)
        check("already within budget: True, nothing raised",
              run.ok is True and scale_at(run.root, 0) == 1)

    with tempdir() as td:
        run = drive_enforcement(td, lambda r: 50_000)
        check("a budget no scale can meet returns False",
              run.ok is False, f"got {run.ok}")
        check("...after climbing every emissive face to 128",
              scale_at(run.root, 0) == 128 and scale_at(run.root, 1) == 128)

    with tempdir() as td:
        run = drive_enforcement(td, lambda r: 50_000, texlights=False)
        check("over budget with no texlight definitions found returns False",
              run.ok is False and scale_at(run.root, 0) == 1)

    with tempdir() as td:
        run = drive_enforcement(td, lambda r: 50_000, vvis_ok=False)
        check("a VVIS failure returns False (counts would be unreliable)",
              run.ok is False)

    with tempdir() as td:
        run = drive_enforcement(td, lambda r: 50_000,
                                compile_error=vbsp_runner.VBSPCompileError('boom'))
        check("a VBSP failure returns False", run.ok is False)
        check("...and leaves no scratch", lmopt_leftovers(run.maps) == [])


def test_importing_lmoptimizer_leaves_stdio_alone():
    """Protects: hosts that capture stdout/stderr (pytest, the GUI importing
    the module) keep working — lmoptimizer used to replace sys.stdout and
    sys.stderr at import time."""
    import subprocess
    from _testkit import BIN_DIR
    code = ("import sys; o, e = sys.stdout, sys.stderr; import lmoptimizer; "
            "sys.exit(0 if sys.stdout is o and sys.stderr is e else 3)")
    proc = subprocess.run([sys.executable, '-c', code], cwd=str(BIN_DIR),
                          capture_output=True, text=True)
    check("import does not rewrap sys.stdout / sys.stderr", proc.returncode == 0,
          f"rc={proc.returncode} {proc.stderr[-300:]}")


def solver_result(verts, budget=62259):
    """A stand-in for the VBSP solver's result (only what the report prints)."""
    return {'vbsp_solver_result': types.SimpleNamespace(
        vertex_budget=budget, vbsp_vertex_count=verts, vbsp_leafface_count=verts,
        vbsp_face_count=100, binding_limit='verts', baseline_scale=2,
        faces_promoted=3, faces_at_baseline=4, faces_skipped=5, vbsp_calls=6,
        solve_time=0.1, faces_pre_promoted=0),
        'modified': 0}


def test_main_publishes_only_after_every_stage_succeeds():
    """Protects: the output VMF is written once, atomically, only when the
    vertex budget AND the light budget are met; every other outcome exits
    nonzero and leaves the previous output untouched (over-budget results and
    unmet light budgets used to be written anyway with exit 0)."""
    with tempdir() as td:
        ws = Workspace(td)
        lit = ws.maps / 'lit.bsp'
        write_bsp(lit, ldr_len=4096)

        def run(result, enforcement, *extra):
            with mock.patch.object(lmoptimizer, '_run_bsp_mode',
                                   return_value=result), \
                    mock.patch.object(lmoptimizer, '_run_light_budget_enforcement',
                                      return_value=enforcement) as enforce:
                code = run_main(ws.argv('--bsp', str(lit), '-o', str(ws.output),
                                        *extra))
            return code, enforce

        code, enforce = run(solver_result(70_000), True)
        check("a solution over the vertex budget exits 1", code == 1, f"got {code}")
        check("...and writes nothing", ws.untouched())

        code, enforce = run(solver_result(1_000), False)
        check("an unmet light budget exits 1", code == 1 and enforce.called,
              f"code={code} called={enforce.called}")
        check("...and writes nothing", ws.untouched())

        code, enforce = run(None, True)
        check("a failed stage (result None) exits 1 without enforcing",
              code == 1 and not enforce.called)
        check("...and writes nothing", ws.untouched())

        code, enforce = run(solver_result(1_000), True, '--dry-run')
        check("--dry-run succeeds without enforcing or writing",
              code == 0 and not enforce.called and ws.untouched())

        code, enforce = run(solver_result(1_000), True, '--light-budget', '0')
        check("--light-budget 0 skips enforcement and publishes",
              code == 0 and not enforce.called
              and ws.output.read_text() != PREVIOUS_OUTPUT)

        ws.output.write_text(PREVIOUS_OUTPUT)
        code, enforce = run(solver_result(1_000), True)
        published = VMFParser().parse_file(ws.output)
        original = VMFParser().parse_string(MINI_VMF)
        check("success: enforcement ran and the published output is the parsed map",
              code == 0 and enforce.called
              and len(published.get_all_recursive('side'))
              == len(original.get_all_recursive('side')),
              f"code={code}")
        check("...atomically: no temp file left, user's compile untouched",
              not [p for p in ws.maps.iterdir() if p.name.endswith('.tmp')]
              and ws.user_bsp.read_bytes() == b'USER COMPILE')


def lit_vrad(vrad_exe, bsp_path, *a, **k):
    """compile_rad stand-in that really 'lights' the BSP."""
    write_bsp(Path(bsp_path), ldr_len=4096)
    return Path(bsp_path)


INSTANCE_ENTITY = (
    'entity\n{\n"id" "300"\n"classname" "func_instance"\n'
    '"file" "ghost_instance.vmf"\n}\n')


class _RelocatedScratch(vbsp_runner.ScratchVMF):
    """A baseline scratch that had to fall back to the system temp folder."""

    def __init__(self, anchor_vmf, label='scratch'):
        super().__init__(anchor_vmf, label)
        if label == 'baseline':
            self.relocated = True


def _cache_dir_files(ws):
    cache = ws.maps / '.lmopt_cache'
    return sorted(p.name for p in cache.iterdir()) if cache.is_dir() else []


def run_cache_rounds(rounds, *, write_error=None, relocated=False, instance=False,
                     gc_during_pipeline=False):
    """Successive auto-compile runs on ONE workspace (the BSP pipeline itself
    stubbed), each with its own extra args. Returns one record per round: what
    the pipeline was handed, and what the cache and folders looked like."""
    records = []
    with tempdir() as td, contextlib.ExitStack() as stack:
        ws = Workspace(td)
        if instance:
            ws.vmf.write_text(MINI_VMF + INSTANCE_ENTITY)
        count, compile_bsp, compile_rad = stub_compilers(stack, vrad_compile=lit_vrad)
        seen = {}

        def fake_bsp_mode(args, root, t0, tools, emissive_floor=1):
            bsp = Path(args.bsp)
            seen['bsp'] = bsp
            seen['existed'] = bsp.is_file()
            if gc_during_pipeline:
                # another run's cache GC removes the entry's BSP mid-pipeline
                for p in (ws.maps / '.lmopt_cache').glob('*.bsp'):
                    p.unlink()
                seen['survived_gc'] = bsp.is_file()
            return solver_result(1_000)

        stack.enter_context(mock.patch.object(lmoptimizer, '_run_bsp_mode',
                                              side_effect=fake_bsp_mode))
        stack.enter_context(mock.patch.object(lmoptimizer,
                                              '_run_light_budget_enforcement',
                                              return_value=True))
        if write_error:
            stack.enter_context(mock.patch.object(
                lmoptimizer, '_write_cache_entry', side_effect=write_error))
        if relocated:
            stack.enter_context(mock.patch.object(
                vbsp_runner, 'ScratchVMF', _RelocatedScratch))
        cache_dir = ws.maps / '.lmopt_cache'
        for extra in rounds:
            seen.clear()
            compiles_before = compile_bsp.call_count
            rad_calls_before = len(compile_rad.call_args_list)
            files_before = _cache_dir_files(ws)
            output_before = ws.output.read_text()
            err = io.StringIO()
            with contextlib.redirect_stderr(err):
                code = run_main(ws.argv(*extra))
            manifest = cache_dir / 'm.manifest.json'
            entry_bsp = None
            if manifest.is_file():
                entry_bsp = cache_dir / json.loads(manifest.read_text())['bsp_file']
            records.append(types.SimpleNamespace(
                code=code, seen=dict(seen), stderr=err.getvalue(),
                manifest=manifest.is_file(), entry_bsp=entry_bsp,
                compiled=compile_bsp.call_count > compiles_before,
                rad_levels=[c.kwargs.get('texture_shadows') for c in
                            compile_rad.call_args_list[rad_calls_before:]],
                handed_cache_file=(seen.get('bsp') is not None
                                   and seen['bsp'] == entry_bsp),
                handoff_gone=(seen.get('bsp') is not None
                              and not seen['bsp'].is_file()),
                files_before=files_before, files_after=_cache_dir_files(ws),
                leftovers=lmopt_leftovers(ws.maps),
                output_changed=ws.output.read_text() != output_before,
                user_bsp_intact=ws.user_bsp.read_bytes() == b'USER COMPILE'))
    return records


def test_cache_is_best_effort_and_skipped_when_unsafe():
    """Protects: the baseline cache can never fail a run (a cache write error
    only warns) and is never written when the baseline cannot be trusted as the
    map's: an instance VBSP cannot resolve means its contents are unknown, and a
    scratch that fell back to the system temp folder compiled without the map's
    relative instances. Non-cached runs still classify from the lit BSP, on a
    private copy that is cleaned up afterwards."""
    run, = run_cache_rounds([()])
    check("control: a normal run succeeds and writes a cache entry",
          run.code == 0 and run.manifest and run.seen['existed']
          and run.output_changed, f"code={run.code} manifest={run.manifest}")
    check("the pipeline read its own private copy, not the cache file",
          not run.handed_cache_file and run.handoff_gone and run.leftovers == [],
          f"handed_cache_file={run.handed_cache_file} left={run.leftovers}")

    run, = run_cache_rounds([()], write_error=OSError('disk full'))
    check("a failed cache write does not fail the run",
          run.code == 0 and run.output_changed, f"code={run.code}")
    check("...it warns, and the pipeline still got the lit BSP",
          bool(run.stderr.strip()) and run.seen['existed'])
    check("...no manifest; the private copy is cleaned up",
          not run.manifest and run.handoff_gone and run.leftovers == [],
          f"manifest={run.manifest} gone={run.handoff_gone} left={run.leftovers}")

    run, = run_cache_rounds([()], relocated=True)
    check("a scratch that fell back to the system temp folder is never cached",
          run.code == 0 and not run.manifest and run.seen['existed']
          and run.handoff_gone, f"code={run.code} manifest={run.manifest}")

    run, = run_cache_rounds([()], instance=True)
    check("an unresolvable instance disables the cache for the run",
          run.code == 0 and not run.manifest and run.seen['existed']
          and run.handoff_gone, f"code={run.code} manifest={run.manifest}")


def test_cache_hit_runs_on_a_private_copy():
    """Protects: a cache hit hands the pipeline (match phase and bspwalk, which
    read the BSP by path for minutes) its OWN copy of the cached BSP, so
    another run's cache GC cannot pull the file away mid-run; the copy is
    cleaned up; a dry run gets a private copy too, outside the cache folder,
    and leaves that folder exactly as it was."""
    first, hit = run_cache_rounds([(), ()], gc_during_pipeline=False)
    check("the second identical run is a cache hit (no recompile)",
          first.compiled and not hit.compiled and hit.code == 0,
          f"first.compiled={first.compiled} hit.compiled={hit.compiled}")
    check("the hit's pipeline got a private copy, not the cache file",
          hit.seen['existed'] and not hit.handed_cache_file, f"{hit.seen}")
    check("the private copy is gone afterwards and the entry is intact",
          hit.handoff_gone and hit.leftovers == [] and hit.entry_bsp is not None
          and hit.entry_bsp.name in hit.files_after)

    _, gc_hit = run_cache_rounds([(), ()], gc_during_pipeline=True)
    check("the pipeline's BSP survives the cache entry being collected under it",
          gc_hit.code == 0 and gc_hit.seen.get('survived_gc') is True,
          f"{gc_hit.seen}")

    _, _, dry = run_cache_rounds([(), (), ('--dry-run',)])
    check("a dry-run cache hit succeeds without compiling or writing anything",
          dry.code == 0 and not dry.compiled and not dry.output_changed
          and dry.files_before == dry.files_after, f"code={dry.code}")
    check("...and reads a private copy outside the cache, removed at exit",
          dry.seen.get('existed') and not dry.handed_cache_file
          and dry.seen['bsp'].parent != dry.entry_bsp.parent
          and dry.handoff_gone, f"{dry.seen}")


def test_cache_hit_survives_a_concurrent_writer_and_gc():
    """Protects: the cache-hit reader re-reads the manifest when the BSP it was
    about to copy vanished (another writer superseded the entry and collected
    it), and treats a BSP that keeps vanishing as a miss -- never a crash or a
    stale pairing."""
    with tempdir() as td:
        td = Path(td)
        cache = td / '.lmopt_cache'
        first, second = td / 'first.bsp', td / 'second.bsp'
        write_bsp(first, ldr_len=4096)
        write_bsp(second, ldr_len=8192)
        old_path = lmoptimizer._write_cache_entry(cache, 'm', 'KEY', 2, 1, first)

        real_handoff = lmoptimizer._handoff_bsp
        state = {'calls': 0}

        def vanishing_once(cache_dir, stem, src, **kw):
            state['calls'] += 1
            if state['calls'] == 1:
                # a concurrent writer replaces the entry and the old BSP is collected
                lmoptimizer._write_cache_entry(cache, 'm', 'KEY', 3, 1, second)
                old_path.unlink()
            return real_handoff(cache_dir, stem, src, **kw)

        with mock.patch.object(lmoptimizer, '_handoff_bsp', side_effect=vanishing_once):
            entry, private, reason = lmoptimizer._acquire_cache_hit(cache, 'm', 'KEY')
        check("the reader retried against the replacement entry",
              entry is not None and entry['baseline'] == 3 and not reason,
              f"entry={entry} reason={reason!r}")
        check("...and its private copy is the replacement's content",
              private is not None and private.read_bytes() == second.read_bytes())

        def always_vanishing(cache_dir, stem, src, **kw):
            raise FileNotFoundError(str(src))

        with mock.patch.object(lmoptimizer, '_handoff_bsp', side_effect=always_vanishing):
            entry, private, reason = lmoptimizer._acquire_cache_hit(cache, 'm', 'KEY')
        check("a BSP that keeps vanishing is a miss, with a reason",
              entry is None and private is None and bool(reason), f"reason={reason!r}")


def test_light_budget_not_enforced_is_loudly_reported():
    """Protects: --bsp with no VBSP/VRAD/game still finishes, but says on
    stderr that the light budget was not checked (instead of exiting 0 as if
    it had been). Asked for a budget, it speaks; with the budget disabled
    (--light-budget 0) there is nothing to report and it stays quiet."""
    with tempdir() as td:
        ws = Workspace(td)
        lit = ws.maps / 'lit.bsp'
        write_bsp(lit, ldr_len=4096)

        def run(*extra):
            err = io.StringIO()
            with mock.patch.object(lmoptimizer, '_run_bsp_mode',
                                   return_value=solver_result(1_000)), \
                    mock.patch.object(lmoptimizer, '_run_light_budget_enforcement',
                                      return_value=True) as enforce, \
                    mock.patch.object(lmoptimizer, '_try_find_vbsp', return_value=None), \
                    mock.patch.object(lmoptimizer, '_try_find_vrad', return_value=None), \
                    contextlib.redirect_stderr(err):
                code = run_main([str(ws.vmf), '--bsp', str(lit), '-o', str(ws.output),
                                 *extra])
            return code, enforce.called, err.getvalue().strip()

        code, enforced, stderr = run()
        check("the run finishes", code == 0 and ws.output.read_text() != PREVIOUS_OUTPUT,
              f"code={code}")
        check("enforcement was not attempted", not enforced)
        check("a message about the unchecked light budget reaches stderr",
              bool(stderr), "stderr was empty")
        code, enforced, stderr = run('--light-budget', '0')
        check("with the budget disabled there is nothing to warn about",
              code == 0 and not enforced and not stderr, f"stderr={stderr!r}")


def test_handoff_falls_back_to_a_stamped_temp_file_the_sweep_collects():
    """Protects: when the cache folder cannot take the private BSP copy, the
    fallback in the system temp folder carries the owner stamp, so a hard kill
    does not leak an unrecognisable file."""
    import tempfile
    with tempdir() as td:
        td = Path(td)
        src = td / 'lit.bsp'
        write_bsp(src, ldr_len=4096)
        blocker = td / 'not_a_folder'
        blocker.write_bytes(b'x')
        unusable_cache = blocker / '.lmopt_cache'     # mkdir under a file fails
        copy = lmoptimizer._handoff_bsp(unusable_cache, 'm', src)
        try:
            check("the copy landed in the system temp folder",
                  copy.parent == Path(tempfile.gettempdir()) and copy.is_file()
                  and copy.read_bytes() == src.read_bytes(), f"got {copy}")
            os.utime(copy, (time.time() - 3600, time.time() - 3600))
            with mock.patch.object(vbsp_runner, 'pid_alive', return_value=False):
                vbsp_runner.sweep_stale_scratch(Path(tempfile.gettempdir()),
                                                ('scratch', 'cache'))
            check("a hard-killed run's fallback copy is collected by the sweep",
                  not copy.exists(), f"left: {copy.name}")
        finally:
            if copy.exists():
                copy.unlink()

        missing_source = td / 'gone.bsp'
        raised = False
        try:
            lmoptimizer._handoff_bsp(td / '.lmopt_cache', 'm', missing_source)
        except FileNotFoundError:
            raised = True
        check("a missing SOURCE is reported as such (the caller re-reads the cache)",
              raised)


def test_sweep_failure_never_fails_a_run():
    """Protects: housekeeping of an earlier run's leftovers is best-effort; an
    error in the sweep is a warning, never the reason a run exits 1."""
    with tempdir() as td:
        ws = Workspace(td)
        lit = ws.maps / 'lit.bsp'
        write_bsp(lit, ldr_len=4096)
        err = io.StringIO()
        with mock.patch.object(vbsp_runner, 'sweep_stale_scratch',
                               side_effect=OSError('folder vanished')), \
                mock.patch.object(lmoptimizer, '_run_bsp_mode',
                                  return_value=solver_result(1_000)), \
                mock.patch.object(lmoptimizer, '_run_light_budget_enforcement',
                                  return_value=True), \
                contextlib.redirect_stderr(err):
            code = run_main(ws.argv('--bsp', str(lit), '-o', str(ws.output)))
        check("the run still succeeds and publishes",
              code == 0 and ws.output.read_text() != PREVIOUS_OUTPUT, f"code={code}")
        check("...and the sweep problem was reported", bool(err.getvalue().strip()))


def test_the_test_runner_counts_raw_assertions_as_failures():
    """Protects: run_all cannot swallow a bare `assert` in a test (it used to
    treat every AssertionError as one check() had already recorded)."""
    import _testkit
    saved = list(_testkit.FAILURES)

    def raw_assert():
        assert False, 'a bare assert'

    def passes():
        check("a passing check", True)

    try:
        with contextlib.redirect_stdout(io.StringIO()),                 contextlib.redirect_stderr(io.StringIO()):   # keep the demo failure out of the log
            _testkit.FAILURES.clear()
            code_fail = _testkit.run_all("runner self-test", [raw_assert])
            _testkit.FAILURES.clear()
            code_ok = _testkit.run_all("runner self-test", [passes])
    finally:
        _testkit.FAILURES[:] = saved
    check("a bare assert makes the run fail", code_fail == 1, f"got {code_fail}")
    check("passing tests make it succeed", code_ok == 0, f"got {code_ok}")


TESTS = [
    test_cache_entry_rejects_poisoned_and_stale_entries,
    test_cache_entries_of_concurrent_runs_never_swap_bsps,
    test_cache_key_tracks_tool_implementation_and_rad_inputs,
    test_cache_key_follows_vrads_lights_rad_search_path,
    test_instance_closure_resolves_the_way_vbsp_does,
    test_scratch_vmf_is_private_and_cleans_only_its_own_files,
    test_host_tag_distinguishes_long_hostnames,
    test_sweep_removes_only_this_tools_dead_stale_files,
    test_baseline_scan_reports_environment_failures_as_such,
    test_count_vertices_separates_scale_rejections_from_other_failures,
    test_solver_writes_scales_to_sides_without_the_key,
    test_solver_tolerates_only_capacity_rejections,
    test_compile_rad_enforces_timeout_and_exit_code,
    test_failed_auto_compile_leaves_previous_output_untouched,
    test_dry_run_in_auto_compile_mode_does_nothing,
    test_unlit_bsp_is_an_error_unless_allowed,
    test_bspwalk_failure_in_bsp_mode_exits_nonzero,
    test_bad_setups_are_rejected_before_any_work,
    test_enforcement_finds_the_smallest_fitting_scale_or_reports_failure,
    test_importing_lmoptimizer_leaves_stdio_alone,
    test_main_publishes_only_after_every_stage_succeeds,
    test_cache_is_best_effort_and_skipped_when_unsafe,
    test_cache_hit_runs_on_a_private_copy,
    test_cache_hit_survives_a_concurrent_writer_and_gc,
    test_handoff_falls_back_to_a_stamped_temp_file_the_sweep_collects,
    test_light_budget_not_enforced_is_loudly_reported,
    test_sweep_failure_never_fails_a_run,
    test_the_test_runner_counts_raw_assertions_as_failures,
]


if __name__ == '__main__':
    sys.exit(run_all("test_pipeline_guards", TESTS))
