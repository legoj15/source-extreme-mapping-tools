"""Real-binary guard: VBSP's proplm UV2 profile (issue #60, VBSP half).

Run:  python -I game/bin/x64/unit_test/proplm/test_proplm_uv2.py [--game DIR]
  or: pytest game/bin/x64/unit_test/proplm
Exit code 0 = pass. Needs vbsp_lmo.exe in game/bin/x64 and a game directory
whose content mounts models/props_canal/boat001a (default: HL2 Deathmatch's
hl2mp, or the PROPLM_TEST_GAME environment variable). Missing either is SKIP.
(Garry's Mod is not usable here: its engine DLLs are in bin/win64, which the
x64 compilers do not look in.)

What it protects: compiling the staticpropbounce fixture with its prop_static
marked proplm "1" must give a 'plm2' game lump whose UV2 is a real lightmap
layout of the model's own LOD0 triangles: every UV2 inside [0,1], the unwrap
spread over the atlas, no texel shared by two triangles, each UV2 triangle
paired with its own 3D corners, (all but a fraction of a percent of) the
triangles the VTX has in LOD0 of model 0 of each bodypart, a position checksum
a consumer can recompute, the lump stored uncompressed, and the prop moved from
the static prop lump to a prop_dynamic carrying the proplm keys. Also: -onlyprops
on a map with a proplm prop must stop with an error and leave the BSP alone (it
never rewrites the entity lump, so the prop would be lost). Breaks if the unwrap
stops being a layout (UV2 left in texel units, copied from UV0, charts packed on
top of each other, triangles or corners paired with the wrong geometry), if the
walk takes the wrong LOD, model or strips, if the entity/lump hand-off to VRAD
and the Lua addon changes, or if -onlyprops silently accepts a proplm prop.
"""
import math
import os
import re
import struct
import subprocess
import sys
import tempfile
import zlib
from pathlib import Path

# python -I drops the script's folder from sys.path. _testkit (shared with the
# lmoptimizer tests) puts game/bin/x64 on sys.path, which gives vbsp_runner and
# vpk_reader.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'lmoptimizer'))
from _testkit import BIN_DIR, check, run_all

VBSP = BIN_DIR / 'vbsp_lmo.exe'
FIXTURE = BIN_DIR / 'bsp_unit_tests' / 'validation_vrad_staticpropbounce.vmf'
# The Steam library differs per machine (the dev box has it on E:, Server1 on C:).
STEAM_COMMON = os.environ.get("SEMT_STEAM_COMMON", r"E:\Steam\steamapps\common")
DEFAULT_GAME = os.path.join(STEAM_COMMON, "Half-Life 2 Deathmatch", "hl2mp")
GAME = Path(os.environ.get('PROPLM_TEST_GAME', DEFAULT_GAME))
MODEL = 'models/props_canal/boat001a.mdl'

# The fixture's prop_static asks for lightmapresolutionx/y 512.
PROP_RES = 512

LUMP_ENTITIES = 0
LUMP_GAME = 35
# Source writes 'plm2' as a multi-character constant, so the id int is
# big-endian-ordered characters ('sprp' likewise).
ID_PLM2 = (ord('p') << 24) | (ord('l') << 16) | (ord('m') << 8) | ord('2')
ID_SPRP = (ord('s') << 24) | (ord('p') << 16) | (ord('r') << 8) | ord('p')


# --------------------------------------------------------------------------
# Inputs
# --------------------------------------------------------------------------
def _skip_reason():
    if not VBSP.is_file():
        return f"not built: {VBSP.name}"
    if not FIXTURE.is_file():
        return f"missing fixture {FIXTURE}"
    if not (GAME / 'gameinfo.txt').is_file():
        return f"no gameinfo.txt in {GAME} (set PROPLM_TEST_GAME or --game)"
    if _read_game_file(MODEL) is None:
        return f"{MODEL} is not in {GAME} or the VPKs beside it"
    if _read_vtx() is None:
        return f"{MODEL} has neither a .dx90.vtx nor a .dx80.vtx in {GAME} or the VPKs beside it"
    return None


def _read_game_file(rel):
    """A game file from the loose tree or, failing that, the VPKs beside the
    game dir (hl2mp mounts hl2's)."""
    loose = GAME / rel
    if loose.is_file():
        return loose.read_bytes()
    import vpk_reader
    for vpk in sorted(list(GAME.glob('*_dir.vpk')) + list(GAME.parent.glob('*/*_dir.vpk'))):
        data = vpk_reader.VPKReader(vpk).read_file(rel)
        if data is not None:
            return data
    return None


def _read_vtx():
    """The model's .dx90.vtx, else its .dx80.vtx (VBSP falls back the same way);
    None when neither exists."""
    for ext in ('.dx90.vtx', '.dx80.vtx'):
        data = _read_game_file(MODEL.replace('.mdl', ext))
        if data is not None:
            return data
    return None


def _fixture_prop_keys():
    """Keyvalues of the fixture VMF's prop_static, from the tracked file (before
    proplm is added): the plain keys up to its nested editor block."""
    text = FIXTURE.read_bytes().decode('utf-8')
    at = text.index('"classname" "prop_static"')
    start = text.rindex('entity', 0, at)
    end = text.index('editor', at)
    return dict(re.findall(r'"([^"]+)"\s+"([^"]*)"', text[start:end]))


def _floats(text):
    return [float(v) for v in text.split()]


def _marked_fixture(dst):
    """The tracked fixture VMF with proplm "1" added to its single prop_static."""
    text = FIXTURE.read_bytes().decode('utf-8')
    nl = '\r\n' if '\r\n' in text else '\n'
    anchor = '"classname" "prop_static"' + nl
    assert text.count(anchor) == 1, "fixture must hold exactly one prop_static"
    dst.write_bytes(text.replace(anchor, anchor + '\t"proplm" "1"' + nl).encode('utf-8'))


# --------------------------------------------------------------------------
# BSP / lump readers (just enough for this test)
# --------------------------------------------------------------------------
def _lump(bsp, lump_id):
    ofs, length = struct.unpack_from('<ii', bsp, 8 + lump_id * 16)
    return bsp[ofs:ofs + length]


def _game_lumps(bsp):
    """{id: (flags, version, bytes)} from the game lump directory."""
    ofs, _ = struct.unpack_from('<ii', bsp, 8 + LUMP_GAME * 16)
    count, = struct.unpack_from('<i', bsp, ofs)
    out = {}
    for i in range(count):
        gid, flags, version, fofs, flen = struct.unpack_from('<IHHii', bsp, ofs + 4 + i * 16)
        out[gid] = (flags, version, bsp[fofs:fofs + flen])
    return out


def _entities(bsp):
    text = _lump(bsp, LUMP_ENTITIES).split(b'\0')[0].decode('utf-8', 'replace')
    ents = []
    for block in re.findall(r'\{([^{}]*)\}', text):
        ents.append(dict(re.findall(r'"([^"]*)"\s+"([^"]*)"', block)))
    return ents


def _static_prop_models(sprp):
    """(dict model names, prop count) of a 'sprp' game lump."""
    dict_count, = struct.unpack_from('<i', sprp, 0)
    names = [sprp[4 + i * 128:4 + (i + 1) * 128].split(b'\0')[0].decode() for i in range(dict_count)]
    pos = 4 + dict_count * 128
    leaf_count, = struct.unpack_from('<i', sprp, pos)
    pos += 4 + leaf_count * 2
    prop_count, = struct.unpack_from('<i', sprp, pos)
    return names, prop_count


def _parse_plm2(data):
    """Parse the 'plm2' lump per src/public/proplm.h, requiring that it uses
    every byte. Returns (version, [model dicts])."""
    version, model_count = struct.unpack_from('<ii', data, 0)
    pos = 8
    models = []
    for _ in range(model_count):
        name, mat_count, tri_count, checksum = struct.unpack_from('<128siiI', data, pos)
        pos += 140
        mats = []
        for _ in range(mat_count):
            mats.append(data[pos:pos + 96].split(b'\0')[0].decode())
            pos += 96
        tris = []
        for _ in range(tri_count):
            mat, = struct.unpack_from('<i', data, pos)
            pos += 4
            corners = []
            for _ in range(3):
                c = struct.unpack_from('<3f3f2f2f', data, pos)
                pos += 40
                corners.append({'pos': c[0:3], 'normal': c[3:6], 'uv0': c[6:8], 'uv2': c[8:10]})
            tris.append({'mat': mat, 'corners': corners})
        models.append({'name': name.split(b'\0')[0].decode(), 'materials': mats,
                       'tris': tris, 'checksum': checksum})
    assert pos == len(data), f"plm2 lump has {len(data) - pos} unread bytes"
    return version, models


def _vtx_lod0_trilist_triangles(vtx):
    """Triangles in LOD0 of model 0 of every bodypart, trilist strips only,
    read straight from the .vtx (all structs packed: bodypart 8, model 8, lod
    12, mesh 9, stripgroup 25, strip 27 bytes)."""
    num_body, body_ofs = struct.unpack_from('<ii', vtx, 28)
    total = 0
    for b in range(num_body):
        body = body_ofs + b * 8
        num_models, model_ofs = struct.unpack_from('<ii', vtx, body)
        if num_models < 1:
            continue
        model = body + model_ofs
        num_lods, lod_ofs = struct.unpack_from('<ii', vtx, model)
        if num_lods < 1:
            continue
        lod = model + lod_ofs
        num_meshes, mesh_ofs = struct.unpack_from('<ii', vtx, lod)
        for m in range(num_meshes):
            mesh = lod + mesh_ofs + m * 9
            num_groups, group_ofs = struct.unpack_from('<ii', vtx, mesh)
            for g in range(num_groups):
                group = mesh + group_ofs + g * 25
                num_strips, strip_ofs = struct.unpack_from('<ii', vtx, group + 16)
                for s in range(num_strips):
                    strip = group + strip_ofs + s * 27
                    num_indices, = struct.unpack_from('<i', vtx, strip)
                    flags, = struct.unpack_from('<B', vtx, strip + 18)
                    if flags & 1:  # STRIP_IS_TRILIST
                        total += num_indices // 3
    return total


# --------------------------------------------------------------------------
# UV2 layout measurements
# --------------------------------------------------------------------------
def _uv2_layout(tris, res=256):
    """(sum of UV2 triangle areas, share of claimed texels claimed by more than
    one triangle) with the triangles rasterized into res x res texels. A texel
    is claimed when its center is strictly inside a triangle, so two triangles
    that only share an edge never double-claim."""
    area_sum = 0.0
    owner = set()
    claimed = doubled = 0
    for tri in tris:
        pts = [(c['uv2'][0] * res, c['uv2'][1] * res) for c in tri['corners']]
        (ax, ay), (bx, by), (cx, cy) = pts
        area2 = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
        area_sum += abs(area2) / 2.0 / (res * res)
        if abs(area2) < 1e-9:
            continue
        x0 = max(0, int(math.floor(min(ax, bx, cx) - 0.5)))
        x1 = min(res - 1, int(math.ceil(max(ax, bx, cx) - 0.5)))
        y0 = max(0, int(math.floor(min(ay, by, cy) - 0.5)))
        y1 = min(res - 1, int(math.ceil(max(ay, by, cy) - 0.5)))
        for py in range(y0, y1 + 1):
            for px in range(x0, x1 + 1):
                x, y = px + 0.5, py + 0.5
                w0 = ((bx - x) * (cy - y) - (by - y) * (cx - x)) / area2
                w1 = ((cx - x) * (ay - y) - (cy - y) * (ax - x)) / area2
                w2 = 1.0 - w0 - w1
                if w0 > 1e-6 and w1 > 1e-6 and w2 > 1e-6:
                    claimed += 1
                    if (px, py) in owner:
                        doubled += 1
                    else:
                        owner.add((px, py))
    return area_sum, (doubled / claimed if claimed else 0.0)


def _edge_proportional_share(tris, max_spread=1.6):
    """Share of triangles whose UV2 edges are proportional to their 3D edges
    (longest UV2/3D edge ratio over the shortest, under max_spread). A chart is
    nearly an isometry of its surface, so a triangle whose UV2 corners are
    paired with the wrong positions (a different triangle's, or the corners
    rotated or swapped) is not. Degenerate triangles are left out."""
    good = total = 0
    for tri in tris:
        pos = [c['pos'] for c in tri['corners']]
        uv = [c['uv2'] for c in tri['corners']]
        ratios = []
        for k in range(3):
            d3 = math.dist(pos[k], pos[(k + 1) % 3])
            d2 = math.dist(uv[k], uv[(k + 1) % 3])
            if d3 < 1e-6 or d2 < 1e-9:
                break
            ratios.append(d2 / d3)
        else:
            total += 1
            good += max(ratios) / min(ratios) < max_spread
    return good / total if total else 0.0


# --------------------------------------------------------------------------
# The test
# --------------------------------------------------------------------------
def test_proplm_uv2_lump_is_a_clean_lightmap_layout():
    reason = _skip_reason()
    if reason:
        if 'pytest' in sys.modules and 'PYTEST_CURRENT_TEST' in os.environ:
            import pytest
            pytest.skip(reason)
        print(f"  SKIP  {reason}")
        return
    import vbsp_runner

    with tempfile.TemporaryDirectory() as td:
        vmf = Path(td) / 'proplm_fixture.vmf'
        _marked_fixture(vmf)
        bsp_path = vbsp_runner.compile_bsp(VBSP, vmf, GAME)
        bsp = bsp_path.read_bytes()

        # -onlyprops never rewrites the entity lump, so it cannot convert a
        # proplm prop (it would be dropped from the static props and never
        # become an entity). It must refuse, before writing anything.
        only = subprocess.run([str(VBSP), '-game', str(GAME), '-onlyprops', str(vmf)],
                              capture_output=True, text=True, errors='replace', timeout=300)
        out = only.stdout + only.stderr
        check("-onlyprops on a map with a proplm prop exits nonzero",
              only.returncode != 0, f"rc={only.returncode}")
        check("-onlyprops names the problem (proplm) and is not just the usage text",
              'proplm' in out and 'usage' not in out.lower(), out[-400:])
        check("the refused -onlyprops run left the BSP untouched",
              bsp_path.read_bytes() == bsp)

    # -- the lump ---------------------------------------------------------
    lumps = _game_lumps(bsp)
    check("the BSP has a 'plm2' game lump", ID_PLM2 in lumps, f"ids {sorted(lumps)}")
    flags, _, data = lumps[ID_PLM2]
    check("'plm2' is stored uncompressed (flags 0)", flags == 0, f"flags {flags}")
    version, models = _parse_plm2(data)
    check("'plm2' header version is 1", version == 1, str(version))
    check("exactly the fixture's model is in the lump",
          [m['name'] for m in models] == [MODEL], str([m['name'] for m in models]))
    model = models[0]
    tris = model['tris']

    # -- UV2 -------------------------------------------------------------
    uv2 = [v for t in tris for c in t['corners'] for v in c['uv2']]
    check("every UV2 coordinate is finite and inside [0,1]",
          all(0.0 <= v <= 1.0 for v in uv2),
          f"min {min(uv2)} max {max(uv2)}")
    area_sum, overlap = _uv2_layout(tris)
    check("UV2 triangle areas sum to at most the unit square",
          area_sum <= 1.0 + 1e-3, f"{area_sum:.3f}")
    check("the unwrap is spread over the atlas, not collapsed",
          area_sum >= 0.2, f"{area_sum:.3f}")
    check("under 1% of the claimed 256x256 texels belong to two triangles",
          overlap < 0.01, f"{overlap * 100:.2f}%")
    # boat001a's own UV0 does not overlap, so a UV2 copied from UV0 would pass
    # the checks above; this one is what catches it.
    same = sum(1 for t in tris for c in t['corners']
               if abs(c['uv2'][0] - c['uv0'][0]) < 1e-4 and abs(c['uv2'][1] - c['uv0'][1]) < 1e-4)
    check("UV2 is its own layout, not a copy of UV0",
          same < 0.1 * 3 * len(tris), f"{same} of {3 * len(tris)} corners equal UV0")

    # xatlas hands back triangles in input order and the lump pairs each UV2
    # with that triangle's own corner positions; shifted or rotated pairings
    # break the proportionality of a triangle's edges.
    share = _edge_proportional_share(tris)
    check("every UV2 triangle is the layout of its own 3D triangle",
          share >= 0.95, f"{share * 100:.1f}% of triangles have edges proportional to 3D")

    # -- triangle set ----------------------------------------------------
    # VBSP drops a triangle xatlas cannot chart (zero area), so the lump may hold
    # a few fewer than the VTX walk; never more, and nearly all of them.
    expected = _vtx_lod0_trilist_triangles(_read_vtx())
    check("the lump holds (nearly) every LOD0 trilist triangle of model 0 of each bodypart",
          0.99 * expected < len(tris) <= expected, f"lump {len(tris)} vtx {expected}")
    check("every triangle's material index is in the material table",
          all(0 <= t['mat'] < len(model['materials']) for t in tris))

    # -- checksum: CRC-32 of the corner positions in 1/16 units, lump order --
    quant = []
    for t in tris:
        for c in t['corners']:
            quant += [math.floor(p * 16.0 + 0.5) for p in c['pos']]
    crc = zlib.crc32(struct.pack(f'<{len(quant)}i', *quant)) & 0xFFFFFFFF
    check("m_PosChecksum is the CRC-32 of the quantized corner positions",
          crc == model['checksum'], f"computed {crc:08x} lump {model['checksum']:08x}")

    # -- hand-off: static prop lump and entity lump -----------------------
    if ID_SPRP in lumps:
        names, prop_count = _static_prop_models(lumps[ID_SPRP][2])
        check("the prop is no longer in the static prop lump",
              prop_count == 0 and MODEL not in names, f"{prop_count} props, dict {names}")
    ents = _entities(bsp)
    check("the prop is no longer an entity of class prop_static",
          not [e for e in ents if e.get('classname') in ('prop_static', 'static_prop')
               and e.get('model') == MODEL])
    dyn = [e for e in ents if e.get('classname') == 'prop_dynamic' and e.get('model') == MODEL]
    check("the entity lump has the model as one prop_dynamic", len(dyn) == 1, str(len(dyn)))
    ent = dyn[0]
    src = _fixture_prop_keys()
    check("prop_dynamic keeps origin, angles and skin of the VMF's prop_static",
          (_floats(ent.get('origin', '')), _floats(ent.get('angles', '')), int(ent.get('skin', -1)))
          == (_floats(src['origin']), _floats(src['angles']), int(src['skin'])),
          str((ent.get('origin'), ent.get('angles'), ent.get('skin'))))
    check("prop_dynamic is solid VPhysics (the VMF prop had a nonzero solid)",
          int(src['solid']) != 0 and ent.get('solid') == '6', str(ent.get('solid')))
    lm = ent.get('proplm_lm', '')
    # proplm_res is at least the requested size, a power of two, at most 2048 (the layout may need more)
    res = int(ent.get('proplm_res') or 0)
    check("prop_dynamic carries proplm, proplm_id and proplm_res",
          (ent.get('proplm'), ent.get('proplm_id')) == ('1', '0')
          and PROP_RES <= res <= 2048 and res & (res - 1) == 0,
          str({k: v for k, v in ent.items() if k.startswith('proplm')}))
    check("proplm_lm is a lowercase path whose last component is the proplm_id (VRAD names the VTF after it)",
          lm == lm.lower() and lm.rsplit('/', 1)[-1] == ent.get('proplm_id'), lm)


TESTS = [test_proplm_uv2_lump_is_a_clean_lightmap_layout]


if __name__ == '__main__':
    if '--game' in sys.argv:
        GAME = Path(sys.argv[sys.argv.index('--game') + 1])
    sys.exit(run_all("test_proplm_uv2", TESTS))
