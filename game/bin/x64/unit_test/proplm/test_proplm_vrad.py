"""VRAD half of proplm (issue #60): bake of a UV2 prop lightmap into the pakfile.

Run:  python game/bin/x64/unit_test/proplm/test_proplm_vrad.py [--game DIR] [--keep DIR]
  or: pytest game/bin/x64/unit_test/proplm
Exit code 0 = all pass.

Input is synthetic so the VBSP half is not needed: a stock lit BSP gets a hand-made
'plm2' game lump (one 16-unit cube whose six faces are laid out as separate UV2
rectangles) and one prop_dynamic entity with proplm "1", rotated 90 degrees in yaw,
parked next to the map's only light. VRAD runs with -bounce 0 (direct light only) and
WITHOUT -StaticPropLighting, then the VTF it put in the pakfile is read back.

Needs vrad_rtx.exe (+ vrad_rtx_dll.dll) in game/bin/x64, a game directory (default:
HL2 Deathmatch's hl2mp like the lmoptimizer tests, or PROPLM_TEST_GAME / --game; it must
have bin/x64/vphysics.dll beside it, Garry's Mod's bin/win64 layout does not load) and a compiled BSP with a `light` entity
(PROPLM_TEST_BSP, else unit_test_maps/validation_vrad_staticpropbounce.bsp here or in
the main checkout). Anything missing is reported as SKIP.

Protects (the numbers a consumer depends on):
  * the VTF lands at materials/<proplm_lm>.vtf, RGB888, proplm_res square, one mip, even
    though -StaticPropLighting is not given (VBSP already converted the prop, so the bake
    cannot depend on the static prop lighting switch);
  * row = v * res, no v flip, and the entity yaw is applied: only the three cube faces
    that face the light (after rotation) are lit, and they sit at the lump's rectangles;
  * values are the .ppl encoding halved (no byte above 128): lit texels are non-black,
    unlit stay 0;
  * texels just outside a lit rectangle are filled (dilation), not black, and nothing is
    lit beyond a few texels of it (the rasterizer covers what the triangles cover).
Breaks if VRAD stops applying the instance transform, flips v, drops the halving or the
dilation, the rasterizer's coverage rule changes, or the bake is tied to -StaticPropLighting
again.
"""
import io
import math
import os
import re
import struct
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path

# python -I drops the script's folder from sys.path; _testkit lives with the lmoptimizer tests.
BIN_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(BIN_DIR / 'unit_test' / 'lmoptimizer'))
from _testkit import check, run_all  # noqa: E402

DEFAULT_GAME = r"E:/Steam/steamapps/common/Half-Life 2 Deathmatch/hl2mp"
GAME = Path(os.environ.get('PROPLM_TEST_GAME', DEFAULT_GAME))
VRAD = BIN_DIR / 'vrad_rtx.exe'
VRAD_DLL = BIN_DIR / 'vrad_rtx_dll.dll'
BASE_BSP_NAME = 'validation_vrad_staticpropbounce.bsp'

# ---- the synthetic prop ------------------------------------------------------------------
RES = 64
HALF = 8.0                        # cube half size, source units
ORIGIN = (-357.0, 347.0, 12.0)    # 60 units from the base map's light, on the side away from its
                                  # boat static prop (which would shadow it), above the floor (top at z=2)
YAW = 90.0
LM_PATH = 'maps/proplm_synth/proplm/0'
MODEL = 'models/proplm_synth/cube.mdl'
# model-space outward normals of the six faces, in chart order (3 columns x 2 rows)
FACES = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
CELL_W, CELL_H, MARGIN = 0.25, 0.5, 0.04  # the right-hand column of the atlas stays empty


def chart_rect(face):
    """UV2 rectangle (u0, v0, u1, v1) of a face."""
    col, row = face % 3, face // 3
    return (col * CELL_W + MARGIN, row * CELL_H + MARGIN,
            (col + 1) * CELL_W - MARGIN, (row + 1) * CELL_H - MARGIN)


def face_axes(n):
    """Two model-space tangents that, with n, form a right-handed frame."""
    ref = (0, 0, 1) if abs(n[2]) < 0.5 else (1, 0, 0)
    t = (n[1] * ref[2] - n[2] * ref[1], n[2] * ref[0] - n[0] * ref[2], n[0] * ref[1] - n[1] * ref[0])
    b = (n[1] * t[2] - n[2] * t[1], n[2] * t[0] - n[0] * t[2], n[0] * t[1] - n[1] * t[0])
    return t, b


def cube_triangles():
    """(corner, ...) x3 per triangle; corner = (pos, normal, uv0, uv2)."""
    tris = []
    for face, n in enumerate(FACES):
        t, b = face_axes(n)
        u0, v0, u1, v1 = chart_rect(face)
        quad = []
        for s, tt in ((0, 0), (1, 0), (1, 1), (0, 1)):
            pos = tuple(HALF * (n[i] + (2 * s - 1) * t[i] + (2 * tt - 1) * b[i]) for i in range(3))
            quad.append((pos, n, (s, tt), (u0 + s * (u1 - u0), v0 + tt * (v1 - v0))))
        tris.append((quad[0], quad[1], quad[2]))
        tris.append((quad[0], quad[2], quad[3]))
    return tris


def build_plm2():
    tris = cube_triangles()
    out = struct.pack('<ii', 1, 1)
    out += struct.pack('<128siiI', MODEL.encode(), 1, len(tris), 0)
    out += struct.pack('<96s', b'proplm_synth/cube')
    for tri in tris:
        out += struct.pack('<i', 0)
        for pos, n, uv0, uv2 in tri:
            out += struct.pack('<10f', *pos, *n, *uv0, *uv2)
    return out


# ---- BSP surgery -------------------------------------------------------------------------
LUMP_ENTITIES, LUMP_GAME = 0, 35
HEADER_SIZE = 8 + 64 * 16 + 4


def prop_entity(origin, lm_path, prop_id, extra=''):
    return (f'{{\n"classname" "prop_dynamic"\n"model" "{MODEL}"\n'
            f'"origin" "{origin[0]:g} {origin[1]:g} {origin[2]:g}"\n"angles" "0 {YAW:g} 0"\n'
            f'"proplm" "1"\n"proplm_id" "{prop_id}"\n"proplm_lm" "{lm_path}"\n"proplm_res" "{RES}"\n'
            f'{extra}}}\n')


def inject(bsp_in, bsp_out, ents=None):
    """Copy bsp_in to bsp_out with a plm2 game lump and proplm prop_dynamic entities added
    (default: the one cube at ORIGIN)."""
    d = Path(bsp_in).read_bytes()
    ident, version = struct.unpack_from('<ii', d, 0)
    assert ident == 0x50534256, 'not a VBSP file'
    lumps = [struct.unpack_from('<iiii', d, 8 + i * 16) for i in range(64)]  # ofs, len, ver, fourcc
    data = [d[o:o + n] for o, n, _, _ in lumps]

    # entities: append before the terminating NUL
    ent = ''.join(ents or [prop_entity(ORIGIN, LM_PATH, 0)]).encode()
    e = data[LUMP_ENTITIES]
    data[LUMP_ENTITIES] = (e[:-1] if e.endswith(b'\0') else e) + ent + b'\0'

    # game lumps: keep the existing ones, add plm2
    game = data[LUMP_GAME]
    entries = []  # (id, flags, version, bytes)
    if game:
        count, = struct.unpack_from('<i', game, 0)
        for i in range(count):
            gid, flags, gver, fo, fl = struct.unpack_from('<IHHii', game, 4 + i * 16)
            entries.append((gid, flags, gver, d[fo:fo + fl]))
    assert not any(g[0] == struct.unpack('<I', b'2mlp')[0] for g in entries), 'base map already has plm2'
    entries.append((struct.unpack('<I', b'2mlp')[0], 0, 1, build_plm2()))

    out = bytearray(HEADER_SIZE)
    new_lumps = [None] * 64
    for i in range(64):
        while len(out) % 4:
            out.append(0)
        pos = len(out)
        if i == LUMP_GAME:
            dir_size = 4 + 16 * len(entries)
            blob = bytearray(struct.pack('<i', len(entries)))
            body = bytearray()
            for gid, flags, gver, payload in entries:
                blob += struct.pack('<IHHii', gid, flags, gver, pos + dir_size + len(body), len(payload))
                body += payload
            payload = bytes(blob + body)
        else:
            payload = data[i]
        _, _, ver, fourcc = lumps[i]
        new_lumps[i] = (pos if payload else 0, len(payload), ver, fourcc)
        out += payload

    struct.pack_into('<ii', out, 0, ident, version)
    for i, lump in enumerate(new_lumps):
        struct.pack_into('<iiii', out, 8 + i * 16, *lump)
    struct.pack_into('<i', out, 8 + 64 * 16, struct.unpack_from('<i', d, 8 + 64 * 16)[0])
    Path(bsp_out).write_bytes(out)


def read_pak_file(bsp, name):
    d = Path(bsp).read_bytes()
    ofs, length, _, _ = struct.unpack_from('<iiii', d, 8 + 40 * 16)
    with zipfile.ZipFile(io.BytesIO(d[ofs:ofs + length])) as z:
        names = [n.lower() for n in z.namelist()]
        return (z.read(name) if name in names else None), names


# ---- VTF ---------------------------------------------------------------------------------
IMAGE_FORMAT_RGB888 = 2  # Source's enum: RGBA8888 0, ABGR8888 1, RGB888 2, BGR888 3
FLAG_CLAMPS, FLAG_CLAMPT, FLAG_NOMIP, FLAG_NOLOD, FLAG_SRGB = 0x4, 0x8, 0x100, 0x200, 0x40


def parse_vtf(buf):
    assert buf[:4] == b'VTF\0', 'bad signature'
    major, minor, header_size = struct.unpack_from('<III', buf, 4)
    width, height, flags, frames = struct.unpack_from('<HHIH', buf, 16)
    fmt, = struct.unpack_from('<i', buf, 0x34)
    mips = buf[0x38]
    low_fmt, = struct.unpack_from('<i', buf, 0x39)
    low_w, low_h = buf[0x3D], buf[0x3E]
    if (major, minor) >= (7, 3):
        count, = struct.unpack_from('<I', buf, 0x44)
        offset = None
        for i in range(count):
            tag, rofs = buf[0x50 + 8 * i:0x50 + 8 * i + 3], struct.unpack_from('<I', buf, 0x50 + 8 * i + 4)[0]
            if tag == b'\x30\x00\x00':
                offset = rofs
        assert offset is not None, 'no high-res image resource'
    else:
        low_size = 0 if low_fmt == -1 or not low_w else max(1, low_w // 4) * max(1, low_h // 4) * 8
        offset = header_size + low_size
    pixels = buf[offset:offset + width * height * 3]
    return dict(version=(major, minor), width=width, height=height, flags=flags, frames=frames,
                fmt=fmt, mips=mips, pixels=pixels, data_end=offset + width * height * 3, size=len(buf))


# ---- expected coverage, computed independently of VRAD -----------------------------------
def covered_texels(face):
    """Texels whose center lies in the face's two triangles (= the chart rectangle)."""
    u0, v0, u1, v1 = chart_rect(face)
    return {(x, y) for y in range(RES) for x in range(RES)
            if u0 <= (x + .5) / RES <= u1 and v0 <= (y + .5) / RES <= v1}


def lit_faces(light):
    """Faces whose world normal faces the light, after the entity yaw."""
    c, s = math.cos(math.radians(YAW)), math.sin(math.radians(YAW))
    lit = set()
    for face, n in enumerate(FACES):
        wn = (c * n[0] - s * n[1], s * n[0] + c * n[1], n[2])
        centre = tuple(ORIGIN[i] + HALF * wn[i] for i in range(3))
        if sum(wn[i] * (light[i] - centre[i]) for i in range(3)) > 1e-3:
            lit.add(face)
    return lit


# ---- the test ----------------------------------------------------------------------------
def find_base_bsp():
    cands = []
    if os.environ.get('PROPLM_TEST_BSP'):
        cands.append(Path(os.environ['PROPLM_TEST_BSP']))
    cands.append(BIN_DIR / 'unit_test_maps' / BASE_BSP_NAME)
    try:  # a git worktree keeps the compiled maps in the main checkout
        common = subprocess.run(['git', '-C', str(BIN_DIR), 'rev-parse', '--git-common-dir'],
                                capture_output=True, text=True, timeout=20).stdout.strip()
        if common:
            cands.append(Path(common).resolve().parent / 'game' / 'bin' / 'x64' / 'unit_test_maps' / BASE_BSP_NAME)
    except (OSError, subprocess.SubprocessError):
        pass
    return next((c for c in cands if c.is_file()), None)


def skip_reason():
    for p in (VRAD, VRAD_DLL):
        if not p.is_file():
            return f'not built: {p.name} in {BIN_DIR}'
    if not (GAME / 'gameinfo.txt').is_file():
        return f'no gameinfo.txt in {GAME} (set PROPLM_TEST_GAME or --game)'
    if find_base_bsp() is None:
        return f'no base BSP ({BASE_BSP_NAME}); set PROPLM_TEST_BSP'
    return None


def light_origin(bsp):
    ents = Path(bsp).read_bytes()
    ofs, length, _, _ = struct.unpack_from('<iiii', ents, 8)
    text = ents[ofs:ofs + length].decode('latin1')
    for block in re.findall(r'\{[^}]*\}', text):
        kv = dict(re.findall(r'"([^"]*)" "([^"]*)"', block))
        if kv.get('classname') == 'light':
            return tuple(float(v) for v in kv['origin'].split())
    raise AssertionError('base BSP has no light entity')


def test_proplm_vtf_is_baked_into_the_pak():
    reason = skip_reason()
    if reason:
        if 'pytest' in sys.modules and 'PYTEST_CURRENT_TEST' in os.environ:
            import pytest
            pytest.skip(reason)
        print(f'  SKIP  {reason}')
        return

    keep = os.environ.get('PROPLM_TEST_KEEP')
    with tempfile.TemporaryDirectory() as td:
        work = Path(keep) if keep else Path(td)
        work.mkdir(parents=True, exist_ok=True)
        base = find_base_bsp()
        bsp = work / 'proplm_synth.bsp'
        inject(base, bsp)
        light = light_origin(bsp)

        proc = subprocess.run(
            [str(VRAD), '-game', str(GAME), '-bounce', '0', str(bsp)],  # no -StaticPropLighting on purpose
            cwd=str(BIN_DIR), capture_output=True, text=True, timeout=900)
        (work / 'vrad.log').write_text(proc.stdout + proc.stderr, encoding='utf-8', errors='replace')
        check('VRAD exits 0', proc.returncode == 0, f'rc={proc.returncode}, see {work / "vrad.log"}')

        name = f'materials/{LM_PATH}.vtf'
        raw, names = read_pak_file(bsp, name)
        check('the lightmap is in the pakfile at materials/<proplm_lm>.vtf', raw is not None,
              f'pak holds {names[:8]}')

        vtf = parse_vtf(raw)
        check('VTF is proplm_res square, RGB888, one frame, one mip',
              (vtf['width'], vtf['height'], vtf['fmt'], vtf['frames'], vtf['mips']) == (RES, RES, IMAGE_FORMAT_RGB888, 1, 1),
              str({k: vtf[k] for k in ('width', 'height', 'fmt', 'frames', 'mips')}))
        want = FLAG_NOMIP | FLAG_NOLOD | FLAG_CLAMPS | FLAG_CLAMPT
        check('VTF flags are NOMIP|NOLOD|CLAMPS|CLAMPT and not SRGB',
              vtf['flags'] & (want | FLAG_SRGB) == want, hex(vtf['flags']))
        check('VTF holds exactly the image data', vtf['data_end'] == vtf['size'], f"{vtf['data_end']} vs {vtf['size']}")

        px = vtf['pixels']
        at = lambda x, y: px[(y * RES + x) * 3:(y * RES + x) * 3 + 3]
        check('stored bytes never exceed 128 (the .ppl value halved; consumers multiply by two)',
              max(px) <= 128, f'max byte {max(px)}')
        lit = lit_faces(light)
        check('the cube has three faces toward the light', len(lit) == 3, str(sorted(lit)))
        for face in range(len(FACES)):
            texels = covered_texels(face)
            vals = [sum(at(x, y)) for x, y in texels]
            if face in lit:
                check(f'face {face} faces the light: its rectangle is lit',
                      min(vals) > 0 and sum(vals) / len(vals) >= 6, f'min {min(vals)}, mean {sum(vals) / len(vals):.1f}')
            else:
                check(f'face {face} faces away: its rectangle stays 0 (direct light only)',
                      max(vals) == 0, f'max {max(vals)}')

        # dilation: the ring of texels just outside a lit rectangle is not black
        for face in sorted(lit):
            inside = covered_texels(face)
            ring = {(x + dx, y + dy) for x, y in inside for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                    if 0 <= x + dx < RES and 0 <= y + dy < RES} - inside
            mine = {(x, y) for f in range(len(FACES)) for x, y in covered_texels(f)}
            ring -= mine
            check(f'face {face}: texels around the lit rectangle are dilated, not black',
                  ring and all(sum(at(x, y)) > 0 for x, y in ring),
                  f'{sum(1 for x, y in ring if sum(at(x, y)) == 0)} of {len(ring)} black')

        # nothing is lit away from the lit rectangles: a few texels of dilation, no more
        near = set()
        for face in lit:
            for x, y in covered_texels(face):
                for dx in range(-6, 7):
                    for dy in range(-6, 7):
                        near.add((x + dx, y + dy))
        stray = [(x, y) for y in range(RES) for x in range(RES) if sum(at(x, y)) and (x, y) not in near]
        check('no lit texel lies more than a few texels from a lit rectangle', not stray, f'{len(stray)} stray, e.g. {stray[:3]}')


def run_vrad(bsp, work, tag):
    proc = subprocess.run([str(VRAD), '-game', str(GAME), '-bounce', '0', str(bsp)],
                          cwd=str(BIN_DIR), capture_output=True, text=True, timeout=900)
    (work / f'vrad_{tag}.log').write_text(proc.stdout + proc.stderr, encoding='utf-8', errors='replace')
    check(f'VRAD exits 0 ({tag})', proc.returncode == 0, f'rc={proc.returncode}, see {work / f"vrad_{tag}.log"}')


def face_means(bsp, lm_path):
    raw, names = read_pak_file(bsp, f'materials/{lm_path}.vtf')
    check(f'{lm_path} is in the pakfile', raw is not None, f'pak holds {names[:8]}')
    px = parse_vtf(raw)['pixels']
    return [sum(sum(px[(y * RES + x) * 3:(y * RES + x) * 3 + 3]) for x, y in covered_texels(f))
            / len(covered_texels(f)) for f in range(len(FACES))]


def test_proplm_props_cast_and_take_shadows():
    """#61: proplm props are in VRAD's tracer, with the static prop keys.

    A second cube halfway between the light and the first one must darken the first cube's
    lit faces, and must not with disableshadows 1. A third cube with ignorenormals 1 and
    disableselfshadowing 1 must get light on the faces that face away from the light (the
    sampler then ignores the normal and the cube does not block itself). Breaks if proplm
    triangles stop reaching the tracer, if the skip id or the flags stop reaching the
    sampler, or if disableshadows is ignored.
    """
    reason = skip_reason()
    if reason:
        if 'pytest' in sys.modules and 'PYTEST_CURRENT_TEST' in os.environ:
            import pytest
            pytest.skip(reason)
        print(f'  SKIP  {reason}')
        return
    keep = os.environ.get('PROPLM_TEST_KEEP')
    with tempfile.TemporaryDirectory() as td:
        work = Path(keep) if keep else Path(td)
        work.mkdir(parents=True, exist_ok=True)
        base = find_base_bsp()
        light = light_origin(base)
        mid = tuple((ORIGIN[i] + light[i]) / 2 for i in range(3))
        # the third cube: on the far side of the first one, as far again from the light
        far = tuple(ORIGIN[i] + (ORIGIN[i] - light[i]) * 0.5 for i in range(2)) + (ORIGIN[2],)
        means = {}
        for tag, blocker_keys in (('shadow', ''), ('noshadow', '"disableshadows" "1"\n')):
            bsp = work / f'proplm_shadow_{tag}.bsp'
            inject(base, bsp, [
                prop_entity(ORIGIN, 'maps/proplm_synth/proplm/0', 0),
                prop_entity(mid, 'maps/proplm_synth/proplm/1', 1, blocker_keys),
                prop_entity(far, 'maps/proplm_synth/proplm/2', 2,
                            '"ignorenormals" "1"\n"disableselfshadowing" "1"\n'),
            ])
            run_vrad(bsp, work, tag)
            means[tag] = (face_means(bsp, 'maps/proplm_synth/proplm/0'),
                          face_means(bsp, 'maps/proplm_synth/proplm/2'))
        lit = sorted(lit_faces(light))
        shadowed = sum(means['shadow'][0][f] for f in lit)
        open_ = sum(means['noshadow'][0][f] for f in lit)
        check('a proplm cube between the light and another darkens it',
              open_ > 0 and shadowed < 0.7 * open_, f'lit faces {shadowed:.1f} with the blocker, {open_:.1f} without')
        # the side faces that face away (the bottom face rests on the floor and stays dark)
        away = [f for f in range(len(FACES)) if f not in lit and FACES[f][2] == 0]
        back = [means['noshadow'][1][f] for f in away]
        check('ignorenormals + disableselfshadowing lights the side faces that face away',
              min(back) > 0, f'face means {[round(v, 1) for v in back]}')


TESTS = [test_proplm_vtf_is_baked_into_the_pak, test_proplm_props_cast_and_take_shadows]


if __name__ == '__main__':
    if '--game' in sys.argv:
        GAME = Path(sys.argv[sys.argv.index('--game') + 1])
    if '--keep' in sys.argv:
        os.environ['PROPLM_TEST_KEEP'] = sys.argv[sys.argv.index('--keep') + 1]
    sys.exit(run_all('test_proplm_vrad', TESTS))
