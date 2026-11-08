"""Real-binary guard: VBSP's proplm "rebake" route (issue #57).

Run:  python -I game/bin/x64/unit_test/proplm/test_proplm_rebake.py [--game DIR]
  or: pytest game/bin/x64/unit_test/proplm
Exit code 0 = pass. Needs vbsp_lmo.exe in game/bin/x64 and a game directory
whose content mounts models/props_wasteland/kitchen_fridge001a (default: HL2
Deathmatch's hl2mp, or the PROPLM_TEST_GAME environment variable). Missing
either is SKIP.

What it protects: compiling the staticpropbounce fixture with its prop_static
swapped for kitchen_fridge001a (whose LOD0 UV islands overlap each other by
about a quarter) and marked proplm "rebake" must point the static prop at a
copy of the model in the BSP pak whose one UV set is a real lightmap layout:
(a) MDL, VVD and both VTX share one checksum and the copy has exactly the
source's vertices, positions untouched; (b) no texel of the new UV space is
claimed by two islands, all UVs in [0,1]; (c) the rebaked base texture at the
new UV shows what the source texture shows at the old UV (DXT decoded, within
a re-encode tolerance); (d) every mesh material resolves to a VMT in the pak
whose $basetexture is in the pak. Breaks if islands are packed on top of each
other or not moved with their texels, if the VVD is written with UVs for the
wrong slots, if the texture copy is offset, flipped or resampled from the wrong
place, if the MDL's cdtexture/name rewrite or the checksums go wrong, or if the
static prop lump keeps the original model.
"""
import math
import os
import re
import struct
import subprocess
import sys
import tempfile
import zipfile
import io
from pathlib import Path

# python -I drops the script's folder from sys.path. _testkit (shared with the
# lmoptimizer tests) puts game/bin/x64 on sys.path, which gives vpk_reader.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'lmoptimizer'))
from _testkit import BIN_DIR, check, run_all

VBSP = BIN_DIR / 'vbsp_lmo.exe'
FIXTURE = BIN_DIR / 'bsp_unit_tests' / 'validation_vrad_staticpropbounce.vmf'
DEFAULT_GAME = r"E:/Steam/steamapps/common/Half-Life 2 Deathmatch/hl2mp"
GAME = Path(os.environ.get('PROPLM_TEST_GAME', DEFAULT_GAME))
# boat001a (the fixture's own prop) has prop_data without allowstatic, so VBSP
# refuses it as a static prop; the fridge is allowed and its islands overlap.
MODEL = 'models/props_wasteland/kitchen_fridge001a.mdl'
MAP_NAME = 'proplm_rebake'

LUMP_GAME = 35
LUMP_PAKFILE = 40
ID_SPRP = (ord('s') << 24) | (ord('p') << 16) | (ord('r') << 8) | ord('p')

OVERLAP_RES = 512


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
    return None


_VPKS = None


def _read_game_file(rel):
    """A game file from the loose tree or, failing that, the VPKs beside the
    game dir (hl2mp mounts hl2's)."""
    global _VPKS
    loose = GAME / rel
    if loose.is_file():
        return loose.read_bytes()
    import vpk_reader
    if _VPKS is None:
        _VPKS = [vpk_reader.VPKReader(v) for v in
                 sorted(list(GAME.glob('*_dir.vpk')) + list(GAME.parent.glob('*/*_dir.vpk')))]
    for vpk in _VPKS:
        data = vpk.read_file(rel)
        if data is not None:
            return data
    return None


def _marked_fixture(dst):
    """The tracked fixture VMF with its single prop_static swapped for the
    fridge (stood on the floor) and marked proplm "rebake"."""
    text = FIXTURE.read_bytes().decode('utf-8')
    nl = '\r\n' if '\r\n' in text else '\n'
    anchor = '"classname" "prop_static"' + nl
    assert text.count(anchor) == 1, "fixture must hold exactly one prop_static"
    text = text.replace(anchor, anchor + '\t"proplm" "rebake"' + nl)
    text = re.sub(r'"model" "models/props_canal/boat001a\.mdl"', f'"model" "{MODEL}"', text)
    text = text.replace('"origin" "-404 431 25"', '"origin" "-404 431 0"')
    assert MODEL in text, "fixture prop model line not found"
    dst.write_bytes(text.encode('utf-8'))


# --------------------------------------------------------------------------
# BSP readers
# --------------------------------------------------------------------------
def _lump(bsp, lump_id):
    ofs, length = struct.unpack_from('<ii', bsp, 8 + lump_id * 16)
    return bsp[ofs:ofs + length]


def _static_prop_names(bsp):
    ofs, _ = struct.unpack_from('<ii', bsp, 8 + LUMP_GAME * 16)
    count, = struct.unpack_from('<i', bsp, ofs)
    for i in range(count):
        gid, _, _, fofs, flen = struct.unpack_from('<IHHii', bsp, ofs + 4 + i * 16)
        if gid == ID_SPRP:
            sprp = bsp[fofs:fofs + flen]
            n, = struct.unpack_from('<i', sprp, 0)
            return [sprp[4 + k * 128:4 + (k + 1) * 128].split(b'\0')[0].decode() for k in range(n)]
    return []


# --------------------------------------------------------------------------
# Model readers (studio.h / optimize.h layouts; x64 build keeps the file layout)
# --------------------------------------------------------------------------
def _cstr(b, ofs):
    return b[ofs:b.index(b'\0', ofs)].decode('latin-1')


def _parse_mdl(b):
    h = {'checksum': struct.unpack_from('<i', b, 8)[0],
         'name': b[12:76].split(b'\0')[0].decode('latin-1')}
    # pszName(): studiohdr2's sznameindex (offset 20) when set; name[64] holds
    # only 63 characters.
    hdr2, = struct.unpack_from('<i', b, 400)
    if hdr2 and struct.unpack_from('<i', b, hdr2 + 20)[0]:
        h['name'] = _cstr(b, hdr2 + struct.unpack_from('<i', b, hdr2 + 20)[0])
    (numtex, texidx, numcd, cdidx, numskinref, numfam, skinidx, numbody,
     bodyidx) = struct.unpack_from('<9i', b, 204)
    h['textures'] = [_cstr(b, texidx + i * 64 + struct.unpack_from('<i', b, texidx + i * 64)[0])
                     for i in range(numtex)]
    h['cdtextures'] = [_cstr(b, struct.unpack_from('<i', b, cdidx + i * 4)[0]) for i in range(numcd)]
    h['skins'] = [list(struct.unpack_from(f'<{numskinref}h', b, skinidx + f * 2 * numskinref))
                  for f in range(numfam)]
    h['bodyparts'] = []
    for bp in range(numbody):
        o = bodyidx + bp * 16
        _, nummodels, _, modelidx = struct.unpack_from('<4i', b, o)
        models = []
        for m in range(nummodels):
            mo = o + modelidx + m * 148
            nummeshes, meshidx, numverts, vertidx = struct.unpack_from('<4i', b, mo + 72)
            meshes = [dict(zip(('material', 'numvertices', 'vertexoffset'),
                               (struct.unpack_from('<i', b, mo + meshidx + k * 116)[0],)
                               + struct.unpack_from('<2i', b, mo + meshidx + k * 116 + 8)))
                      for k in range(nummeshes)]
            models.append({'vertexindex': vertidx, 'numvertices': numverts, 'meshes': meshes})
        h['bodyparts'].append(models)
    return h


def _parse_vvd(b):
    """checksum, LOD0 vertex count, raw slots [(pos bytes, uv, tangent w)] and
    the flat LOD0 index -> raw slot map (fixups)."""
    checksum, = struct.unpack_from('<i', b, 8)
    lod0 = struct.unpack_from('<i', b, 16)[0]
    numfix, fixstart, vertstart, tanstart = struct.unpack_from('<4i', b, 48)
    raw = []
    for i in range(lod0):
        o = vertstart + i * 48
        w = struct.unpack_from('<f', b, tanstart + i * 16 + 12)[0] if tanstart else 1.0
        raw.append((b[o + 16:o + 28], struct.unpack_from('<2f', b, o + 40), w))
    flat = []
    for f in range(numfix):
        lod, src, cnt = struct.unpack_from('<3i', b, fixstart + f * 12)
        flat.extend(range(src, src + cnt))
    return {'checksum': checksum, 'count': lod0, 'raw': raw, 'flat': flat or list(range(lod0))}


def _vtx_checksum(vtx):
    return struct.unpack_from('<i', vtx, 16)[0]


def _lod0_triangles(vtx, mdl):
    """[(texture slot, flat vertex indices)] of LOD0, every model of every
    bodypart, trilist strips (structs packed: bodypart 8, model 8, lod 12,
    mesh 9, stripgroup 25, strip 27, vertex 9 with origMeshVertID at +4)."""
    num_body, body_ofs = struct.unpack_from('<ii', vtx, 28)
    out = []
    for bp in range(min(num_body, len(mdl['bodyparts']))):
        body = body_ofs + bp * 8
        num_models, model_ofs = struct.unpack_from('<ii', vtx, body)
        for mi in range(min(num_models, len(mdl['bodyparts'][bp]))):
            smodel = mdl['bodyparts'][bp][mi]
            model = body + model_ofs + mi * 8
            num_lods, lod_ofs = struct.unpack_from('<ii', vtx, model)
            if num_lods < 1:
                continue
            lod = model + lod_ofs
            num_meshes, mesh_ofs = struct.unpack_from('<ii', vtx, lod)
            base = smodel['vertexindex'] // 48
            for k in range(min(num_meshes, len(smodel['meshes']))):
                smesh = smodel['meshes'][k]
                mesh = lod + mesh_ofs + k * 9
                num_groups, group_ofs = struct.unpack_from('<ii', vtx, mesh)
                for g in range(num_groups):
                    group = mesh + group_ofs + g * 25
                    _, vo, _, io_, ns, so = struct.unpack_from('<6i', vtx, group)
                    for s in range(ns):
                        strip = group + so + s * 27
                        sni, sio = struct.unpack_from('<ii', vtx, strip)
                        if not vtx[strip + 18] & 1:
                            continue
                        for t in range(0, sni - 2, 3):
                            tri = []
                            for c in range(3):
                                idx, = struct.unpack_from('<H', vtx, group + io_ + (sio + t + c) * 2)
                                orig, = struct.unpack_from('<H', vtx, group + vo + idx * 9 + 4)
                                tri.append(base + smesh['vertexoffset'] + orig)
                            out.append((smesh['material'], tuple(tri)))
    return out


def _islands(vvd, tris):
    """Island id per triangle: triangles sharing a raw slot, or a whole edge
    whose two endpoints have bit-identical position, UV and tangent
    handedness, are one island (a shared corner alone is not enough)."""
    raw, flat = vvd['raw'], vvd['flat']
    parent = list(range(len(raw)))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        a, b = find(a), find(b)
        if a != b:
            parent[b] = a
    ids = {}
    edges = {}
    for _, t in tris:
        r = [flat[i] for i in t]
        union(r[0], r[1])
        union(r[0], r[2])
        k = [ids.setdefault((raw[x][0], struct.pack('<2f', *raw[x][1]), raw[x][2] > 0), len(ids))
             for x in r]
        for e in range(3):
            a, b = k[e], k[(e + 1) % 3]
            if a == b:
                continue
            edge = (min(a, b), max(a, b))
            if edge in edges:
                union(edges[edge], r[0])
            else:
                edges[edge] = r[0]
    return [find(flat[t[0]]) for _, t in tris]


def _raster(uv, res):
    """Cells (unwrapped) whose centers are strictly inside the UV triangle."""
    (ax, ay), (bx, by), (cx, cy) = [(u * res, v * res) for u, v in uv]
    area2 = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
    if abs(area2) < 1e-12:
        return []
    out = []
    for py in range(math.floor(min(ay, by, cy)), math.ceil(max(ay, by, cy)) + 1):
        y = py + 0.5
        for px in range(math.floor(min(ax, bx, cx)), math.ceil(max(ax, bx, cx)) + 1):
            x = px + 0.5
            w0 = ((bx - x) * (cy - y) - (by - y) * (cx - x)) / area2
            w1 = ((cx - x) * (ay - y) - (cy - y) * (ax - x)) / area2
            if w0 > 1e-6 and w1 > 1e-6 and 1.0 - w0 - w1 > 1e-6:
                out.append((px, py))
    return out


def _overlap(vvd, tris, island_of, fold):
    """(share of claimed texels claimed by two islands, share claimed twice by
    one island). fold wraps cells into the unit square (texture repeat)."""
    raw, flat = vvd['raw'], vvd['flat']
    owner = {}
    own_cells = {}
    claimed = self_doubled = 0
    for (_, t), isl in zip(tris, island_of):
        mine = own_cells.setdefault(isl, set())
        for c in _raster([raw[flat[i]][1] for i in t], OVERLAP_RES):
            claimed += 1
            if c in mine:
                self_doubled += 1
            mine.add(c)
    inter = total = 0
    for isl, cells in own_cells.items():
        for c in cells:
            f = (c[0] % OVERLAP_RES, c[1] % OVERLAP_RES) if fold else c
            total += 1
            if owner.setdefault(f, isl) != isl:
                inter += 1
    return (inter / total if total else 0.0), (self_doubled / claimed if claimed else 0.0)


# --------------------------------------------------------------------------
# VMT / VTF
# --------------------------------------------------------------------------
def _vmt_basetexture(text):
    m = re.search(r'"?\$basetexture"?\s+"?([^"\s]+)"?', text, re.IGNORECASE)
    return m.group(1).replace('\\', '/').lower() if m else None


_FORMATS = {0: ('raw', 4, (0, 1, 2, 3)), 2: ('raw', 3, (0, 1, 2)), 3: ('raw', 3, (2, 1, 0)),
            12: ('raw', 4, (2, 1, 0, 3)), 16: ('raw', 4, (2, 1, 0)),
            13: ('dxt', 8, None), 15: ('dxt', 16, None)}


def _level_size(fmt, w, h):
    kind, size, _ = _FORMATS[fmt]
    if kind == 'dxt':
        return max(1, (w + 3) // 4) * max(1, (h + 3) // 4) * size
    return w * h * size


class Vtf:
    """Mip 0, frame 0 of a 2D VTF, texels decoded on demand (RGB)."""

    def __init__(self, data):
        self.data = data
        major, minor, hdr = struct.unpack_from('<3I', data, 4)
        self.w, self.h, flags, frames = struct.unpack_from('<HHIH', data, 16)
        self.fmt, mips, lowfmt, loww, lowh = struct.unpack_from('<iBiBB', data, 52)
        if self.fmt not in _FORMATS:
            raise ValueError(f"unsupported VTF format {self.fmt}")
        faces = 6 if flags & 0x4000 else 1
        start = None
        if (major, minor) >= (7, 3):
            nres, = struct.unpack_from('<I', data, 68)
            for r in range(nres):
                tag, _, ofs = struct.unpack_from('<3sBI', data, 80 + r * 8)
                if tag == b'\x30\x00\x00':
                    start = ofs
        if start is None:
            start = hdr + (_level_size(lowfmt, loww, lowh) if lowfmt in _FORMATS and loww else 0)
        for m in range(mips - 1, 0, -1):
            start += _level_size(self.fmt, max(1, self.w >> m), max(1, self.h >> m)) * frames * faces
        self.base = start
        self.blocks = {}

    def texel(self, x, y):
        x %= self.w
        y %= self.h
        kind, size, order = _FORMATS[self.fmt]
        if kind == 'raw':
            o = self.base + (y * self.w + x) * size
            return tuple(self.data[o + order[k]] for k in range(3))
        bx, by = x // 4, y // 4
        key = (bx, by)
        if key not in self.blocks:
            o = self.base + (by * max(1, (self.w + 3) // 4) + bx) * size
            if size == 16:
                o += 8  # DXT5: alpha block first
            c0, c1, bits = struct.unpack_from('<HHI', self.data, o)
            pal = [self._rgb565(c0), self._rgb565(c1)]
            if c0 > c1 or size == 16:
                pal += [tuple((2 * a + b) / 3 for a, b in zip(pal[0], pal[1])),
                        tuple((a + 2 * b) / 3 for a, b in zip(pal[0], pal[1]))]
            else:
                pal += [tuple((a + b) / 2 for a, b in zip(pal[0], pal[1])), (0, 0, 0)]
            self.blocks[key] = [pal[(bits >> (2 * i)) & 3] for i in range(16)]
        return self.blocks[key][(y % 4) * 4 + (x % 4)]

    @staticmethod
    def _rgb565(c):
        return (((c >> 11) & 31) * 255 / 31, ((c >> 5) & 63) * 255 / 63, (c & 31) * 255 / 31)


# --------------------------------------------------------------------------
# The test
# --------------------------------------------------------------------------
def test_proplm_rebake_writes_a_non_overlapping_copy():
    reason = _skip_reason()
    if reason:
        if 'pytest' in sys.modules and 'PYTEST_CURRENT_TEST' in os.environ:
            import pytest
            pytest.skip(reason)
        print(f"  SKIP  {reason}")
        return

    with tempfile.TemporaryDirectory() as td:
        vmf = Path(td) / f'{MAP_NAME}.vmf'
        _marked_fixture(vmf)
        run = subprocess.run([str(VBSP), '-game', str(GAME), str(vmf)], capture_output=True,
                             text=True, errors='replace', timeout=600)
        log = run.stdout + run.stderr
        check("VBSP exits 0", run.returncode == 0, log[-600:])
        bsp = vmf.with_suffix('.bsp').read_bytes()

    # The fixture's static prop: its dictionary entry is the copy.
    names = _static_prop_names(bsp)
    check("the static prop lump names a model, and not the original",
          len(names) > 0 and MODEL not in [n.lower() for n in names], str(names))
    rebaked = names[0]

    pak = zipfile.ZipFile(io.BytesIO(_lump(bsp, LUMP_PAKFILE)))
    paknames = {n.lower(): n for n in pak.namelist()}

    def pak_read(rel):
        n = paknames.get(rel.lower())
        return pak.read(n) if n else None

    stem = rebaked[:-4]
    files = {ext: pak_read(stem + ext) for ext in ('.mdl', '.vvd', '.dx90.vtx', '.dx80.vtx')}
    check("the pak holds the copy's .mdl, .vvd, .dx90.vtx and .dx80.vtx",
          all(files.values()), str({k: v is not None for k, v in files.items()}))

    # -- (a) checksums and vertices ---------------------------------------
    src_mdl = _parse_mdl(_read_game_file(MODEL))
    src_vvd = _parse_vvd(_read_game_file(MODEL[:-4] + '.vvd'))
    mdl = _parse_mdl(files['.mdl'])
    vvd = _parse_vvd(files['.vvd'])
    sums = (mdl['checksum'], vvd['checksum'], _vtx_checksum(files['.dx90.vtx']),
            _vtx_checksum(files['.dx80.vtx']))
    check("MDL, VVD, dx90 VTX and dx80 VTX of the copy share one checksum",
          len(set(sums)) == 1, str(sums))
    check("the copy's MDL calls itself by the copy's name (pszName)",
          'models/' + mdl['name'].replace('\\', '/').lower() == rebaked.lower(),
          f"{mdl['name']} vs {rebaked}")
    check("the copy has exactly the source's vertices",
          vvd['count'] == src_vvd['count'] and vvd['flat'] == src_vvd['flat'],
          f"copy {vvd['count']} source {src_vvd['count']}")
    check("vertex positions are untouched",
          [r[0] for r in vvd['raw']] == [r[0] for r in src_vvd['raw']])

    # -- (b) the new UV space ---------------------------------------------
    tris = _lod0_triangles(files['.dx80.vtx'], mdl)
    island_of = _islands(vvd, tris)
    used = {vvd['flat'][i] for _, t in tris for i in t}
    uvs = [c for r in used for c in vvd['raw'][r][1]]
    check("the copy has LOD0 triangles", len(uvs) > 0)
    check("every LOD0 UV of the copy is inside [0,1]",
          all(0.0 <= c <= 1.0 for c in uvs), f"min {min(uvs)} max {max(uvs)}")
    src_inter, _ = _overlap(src_vvd, tris, _islands(src_vvd, tris), fold=True)
    check("the source's islands do overlap (else this test measures nothing)",
          src_inter > 0.1, f"{src_inter * 100:.1f}%")
    inter, self_doubled = _overlap(vvd, tris, island_of, fold=False)
    check(f"no {OVERLAP_RES}x{OVERLAP_RES} texel of the copy is claimed by two islands",
          inter == 0.0, f"{inter * 100:.3f}%")
    check("under 1% of the copy's texels are claimed twice inside one island (the artist's own)",
          self_doubled < 0.01, f"{self_doubled * 100:.3f}%")

    # -- (d) materials ----------------------------------------------------
    # The engine takes the first cdtexture under which the material exists.
    used_slots = sorted({m for m, _ in tris})
    vmts = {}
    for slot in used_slots:
        for fam in mdl['skins']:
            name = mdl['textures'][fam[slot]].replace('\\', '/')
            vmts[name] = next((v for v in (pak_read(f"materials/{cd.replace(chr(92), '/')}{name}.vmt")
                                          for cd in mdl['cdtextures']) if v), None)
    check("every mesh material of every skin resolves to a VMT in the pak",
          all(v is not None for v in vmts.values()),
          str([k for k, v in vmts.items() if v is None]))
    bases = {k: _vmt_basetexture(v.decode('latin-1')) for k, v in vmts.items()}
    check("every pak VMT's $basetexture is a VTF in the pak",
          all(b and pak_read(f'materials/{b}.vtf') for b in bases.values()), str(bases))

    # -- (c) texture content ----------------------------------------------
    # Skin 0: per texture slot, the source VMT (first cdtexture that has it)
    # and the source VTF it names.
    errors = []
    raw, src_raw, flat = vvd['raw'], src_vvd['raw'], vvd['flat']
    for slot in used_slots:
        name = mdl['textures'][mdl['skins'][0][slot]].replace('\\', '/')
        src_vmt = None
        for scd in src_mdl['cdtextures']:
            src_vmt = _read_game_file(f"materials/{scd.replace(chr(92), '/')}{name}.vmt".lower())
            if src_vmt:
                break
        check(f"the source VMT of {name} is found", src_vmt is not None)
        src_text = src_vmt.decode('latin-1')
        if re.search(r'\$basetexturetransform', src_text, re.IGNORECASE):
            print(f"  note  {name} has a $basetexturetransform; texel compare skipped")
            continue
        src_tex = Vtf(_read_game_file(f'materials/{_vmt_basetexture(src_text)}.vtf'))
        new_tex = Vtf(pak_read(f'materials/{bases[name]}.vtf'))
        for m, t in tris:
            if m != slot:
                continue
            nuv = [raw[flat[i]][1] for i in t]
            ouv = [src_raw[flat[i]][1] for i in t]
            # Only triangles a few texels wide: their centroid is well away
            # from island borders, where filtering and DXT blocks mix colours.
            area = abs((nuv[1][0] - nuv[0][0]) * (nuv[2][1] - nuv[0][1])
                       - (nuv[1][1] - nuv[0][1]) * (nuv[2][0] - nuv[0][0])) / 2
            if area * new_tex.w * new_tex.h < 64:
                continue
            for wts in ((1 / 3, 1 / 3, 1 / 3), (0.5, 0.25, 0.25), (0.25, 0.5, 0.25), (0.25, 0.25, 0.5)):
                nu = sum(w * p[0] for w, p in zip(wts, nuv))
                nv = sum(w * p[1] for w, p in zip(wts, nuv))
                ou = sum(w * p[0] for w, p in zip(wts, ouv))
                ov = sum(w * p[1] for w, p in zip(wts, ouv))
                a = new_tex.texel(math.floor(nu * new_tex.w), math.floor(nv * new_tex.h))
                b = src_tex.texel(math.floor(ou * src_tex.w), math.floor(ov * src_tex.h))
                errors.append(max(abs(x - y) for x, y in zip(a, b)))
    check("the texture comparison has samples (it cannot pass vacuously)", len(errors) > 0)
    errors.sort()
    mean = sum(errors) / len(errors)
    p95 = errors[int(0.95 * (len(errors) - 1))]
    print(f"  info  {len(errors)} texel samples: mean channel error {mean:.2f}, p95 {p95:.1f}, max {errors[-1]:.1f}")
    check("rebaked texture at the new UV matches the source at the old UV (mean channel error < 2)",
          mean < 2.0, f"mean {mean:.2f}")
    check("... and 95% of samples within 8 of the source (DXT re-encode)",
          p95 <= 8, f"p95 {p95:.1f}")


TESTS = [test_proplm_rebake_writes_a_non_overlapping_copy]


if __name__ == '__main__':
    if '--game' in sys.argv:
        GAME = Path(sys.argv[sys.argv.index('--game') + 1])
    sys.exit(run_all("test_proplm_rebake", TESTS))
