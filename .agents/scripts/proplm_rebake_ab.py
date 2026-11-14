"""Compile the staticpropbounce fixture with one prop_static per model, each
marked proplm "rebake", and report what VBSP's rebake did; with two VBSP
builds, also diff their pak lumps.

  python -I .agents/scripts/proplm_rebake_ab.py [--game DIR] [--vbsp A.exe [--vbsp B.exe]]
         [--keep DIR] [--lmres N] model [model ...]

Models are paths under the game, e.g. models/props_wasteland/gear01.mdl.
Per build it prints the "proplm rebake:" log lines and warnings, the pak
files the rebake added (maps/<map>/rebake/...) with sizes and CRC-32, and
their total. With two builds it lists the pak files that differ, are missing
or are new, and says whether the pak lumps are byte-identical. Default VBSP
is game/bin/x64/vbsp_lmo.exe of this checkout; default game HL2DM's hl2mp.
"""
import os
import argparse
import io
import re
import struct
import subprocess
import sys
import tempfile
import zipfile
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BIN = ROOT / 'game' / 'bin' / 'x64'
FIXTURE = BIN / 'bsp_unit_tests' / 'validation_vrad_staticpropbounce.vmf'
# The Steam library differs per machine (the dev box has it on E:, Server1 on C:).
STEAM_COMMON = os.environ.get("SEMT_STEAM_COMMON", r"E:\Steam\steamapps\common")
DEFAULT_GAME = os.path.join(STEAM_COMMON, "Half-Life 2 Deathmatch", "hl2mp")
LUMP_PAKFILE = 40
MAP = 'proplm_ab'


def entity_block(text, classname):
    i = text.index(f'"classname" "{classname}"')
    start = text.rfind('entity', 0, i)
    depth, j = 0, text.index('{', start)
    while True:
        if text[j] == '{':
            depth += 1
        elif text[j] == '}':
            depth -= 1
            if depth == 0:
                return start, j + 1
        j += 1


def make_vmf(dst, models, lmres):
    text = FIXTURE.read_bytes().decode('utf-8')
    nl = '\r\n' if '\r\n' in text else '\n'
    a, b = entity_block(text, 'prop_static')
    block = text[a:b]
    out = []
    for k, model in enumerate(models):
        e = block.replace('"classname" "prop_static"', '"classname" "prop_static"' + nl + '\t"proplm" "rebake"')
        e = re.sub(r'"model" "[^"]*"', f'"model" "{model}"', e)
        e = re.sub(r'"origin" "[^"]*"', f'"origin" "{-404 + 96 * k} 431 0"', e)
        e = re.sub(r'"id" "\d+"', f'"id" "{900000 + k}"', e, count=1)
        if lmres:
            e = re.sub(r'"lightmapresolution([xy])" "[^"]*"', rf'"lightmapresolution\1" "{lmres}"', e)
        out.append(e)
    dst.write_bytes((text[:a] + nl.join(out) + text[b:]).encode('utf-8'))


def lump(bsp, lump_id):
    ofs, length = struct.unpack_from('<ii', bsp, 8 + lump_id * 16)
    return bsp[ofs:ofs + length]


def run(vbsp, game, models, lmres, keep):
    with tempfile.TemporaryDirectory() as td:
        vmf = Path(td) / f'{MAP}.vmf'
        make_vmf(vmf, models, lmres)
        p = subprocess.run([str(vbsp), '-game', str(game), str(vmf)], capture_output=True, text=True,
                           errors='replace', timeout=1200)
        log = p.stdout + p.stderr
        bsp = vmf.with_suffix('.bsp').read_bytes() if vmf.with_suffix('.bsp').exists() else b''
        if keep:
            Path(keep).mkdir(parents=True, exist_ok=True)
            (Path(keep) / f'{Path(vbsp).stem}_{MAP}.bsp').write_bytes(bsp)
            (Path(keep) / f'{Path(vbsp).stem}_{MAP}.log').write_text(log)
    pak = {}
    if bsp:
        z = zipfile.ZipFile(io.BytesIO(lump(bsp, LUMP_PAKFILE)))
        for n in z.namelist():
            data = z.read(n)
            pak[n.lower()] = (len(data), zlib.crc32(data))
    return p.returncode, log, pak, lump(bsp, LUMP_PAKFILE) if bsp else b''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--game', default=DEFAULT_GAME)
    ap.add_argument('--vbsp', action='append')
    ap.add_argument('--keep')
    ap.add_argument('--lmres', type=int, default=0)
    ap.add_argument('models', nargs='+')
    a = ap.parse_args()
    builds = [str(Path(v).resolve()) for v in a.vbsp] if a.vbsp else [str(BIN / 'vbsp_lmo.exe')]
    results = []
    for vbsp in builds:
        rc, log, pak, raw = run(vbsp, a.game, a.models, a.lmres, a.keep)
        results.append((pak, raw))
        print(f'== {vbsp}: exit {rc}')
        for line in log.splitlines():
            if 'proplm rebake' in line or ('Warning' in line and 'proplm' in line):
                print('  ' + line.strip())
        rebake = {n: v for n, v in pak.items() if '/rebake/' in n}
        for n in sorted(rebake):
            print(f'  {rebake[n][0]:>10}  {rebake[n][1]:08x}  {n}')
        print(f'  rebake pak files {len(rebake)}, bytes {sum(v[0] for v in rebake.values())}; '
              f'pak lump {len(raw)} bytes')
    if len(results) == 2:
        (pa, ra), (pb, rb) = results
        print('== diff')
        for n in sorted(set(pa) | set(pb)):
            if pa.get(n) != pb.get(n):
                print(f'  {"missing" if n not in pb else "new" if n not in pa else "differs"}: {n}  '
                      f'{pa.get(n)} -> {pb.get(n)}')
        print('  pak lumps byte-identical' if ra == rb else '  pak lumps differ')


if __name__ == '__main__':
    sys.exit(main())
