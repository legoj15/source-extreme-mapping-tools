"""Does VBSP -onlyents drop the prop lightmap VTFs no prop names any more (#116)?

  python -I .agents/scripts/proplm_stale_vtf_check.py --game DIR --out DIR
         --vbsp LABEL=EXE [--vbsp LABEL=EXE ...] [--vrad BINDIR] [--map NAME]
         [--model models/props_wasteland/kitchen_fridge001a.mdl] [--lmres 128]

The VTFs materials/maps/<map>/proplm/<id>.vtf are written by VRAD, not VBSP: VBSP
only numbers the props (proplm_id 0, 1, ...) and points each at its VTF. So the
scenario is the normal pipeline plus an edit:

  1. seed: compile the staticpropbounce fixture with two prop_static copies of MODEL, both
     proplm "1", with the first --vbsp, then run VRAD on it: ids 0 and 1 get a VTF each;
  2. drop the second prop from the VMF and run every --vbsp with -onlyents on a copy of the
     seed BSP: that VBSP numbers one prop (id 0), so materials/maps/<map>/proplm/1.vtf is
     stale;
  3. the same with every prop removed, and with a full compile that keeps the old pak
     (-keepstalezip) instead of -onlyents: the other ways a pak outlives its props;
  4. control: the same -onlyents with the two-prop VMF unchanged: nothing is stale.

Prints the pak's proplm entries (name, size, CRC-32) of the seed and after each run, and the
VBSP lines that say what it did. Standard library only. SEMT_REPO overrides the checkout root.
"""
import argparse
import io
import os
import re
import shutil
import struct
import subprocess
import sys
import zipfile
import zlib
from pathlib import Path

ROOT = Path(os.environ.get('SEMT_REPO') or Path(__file__).resolve().parents[2])
BIN = ROOT / 'game' / 'bin' / 'x64'
FIXTURE = BIN / 'bsp_unit_tests' / 'validation_vrad_staticpropbounce.vmf'
LUMP_PAKFILE = 40
STEP_X = 96


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


def make_vmf(dst, model, count, lmres):
    text = FIXTURE.read_bytes().decode('utf-8')
    nl = '\r\n' if '\r\n' in text else '\n'
    a, b = entity_block(text, 'prop_static')
    block = text[a:b]
    out = []
    for k in range(count):
        e = block.replace('"classname" "prop_static"', '"classname" "prop_static"' + nl + '\t"proplm" "1"')
        e = re.sub(r'"model" "[^"]*"', f'"model" "{model}"', e)
        e = re.sub(r'"origin" "[^"]*"', f'"origin" "{-404 + STEP_X * k} 431 0"', e)
        e = re.sub(r'"id" "\d+"', f'"id" "{900000 + k}"', e, count=1)
        e = re.sub(r'"lightmapresolution([xy])" "[^"]*"', rf'"lightmapresolution\1" "{lmres}"', e)
        out.append(e)
    dst.write_bytes((text[:a] + nl.join(out) + text[b:]).encode('utf-8'))


def run(cmd, log, cwd=None):
    p = subprocess.run([str(c) for c in cmd], cwd=cwd, capture_output=True, text=True, errors='replace', timeout=1800)
    text = p.stdout + p.stderr
    Path(log).write_text(text, encoding='utf-8')
    return p.returncode, text


def proplm_entries(bsp):
    data = Path(bsp).read_bytes()
    ofs, length = struct.unpack_from('<ii', data, 8 + LUMP_PAKFILE * 16)
    z = zipfile.ZipFile(io.BytesIO(data[ofs:ofs + length]))
    out = []
    for n in sorted(z.namelist(), key=str.lower):
        if '/proplm/' in n.lower():
            raw = z.read(n)
            out.append(f'{n}  {len(raw)} bytes  crc {zlib.crc32(raw):08x}')
    return out


def show(title, entries):
    print(f'  {title}: {len(entries)} entr{"y" if len(entries) == 1 else "ies"}')
    for e in entries:
        print('      ' + e)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--game', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--vbsp', action='append', required=True, help='LABEL=EXE; the first also builds the seed')
    ap.add_argument('--vrad', default=str(BIN), help='folder holding vrad_rtx.exe (default: this checkout)')
    ap.add_argument('--map', default='proplm_stale')
    ap.add_argument('--model', default='models/props_wasteland/kitchen_fridge001a.mdl')
    ap.add_argument('--lmres', type=int, default=128)
    a = ap.parse_args()

    out = Path(a.out)
    shutil.rmtree(out, ignore_errors=True)
    builds = [(v.partition('=')[0], v.partition('=')[2]) for v in a.vbsp]

    seed = out / 'seed'
    seed.mkdir(parents=True)
    make_vmf(seed / f'{a.map}.vmf', a.model, 2, a.lmres)
    rc, text = run([builds[0][1], '-game', a.game, seed / f'{a.map}.vmf'], seed / 'vbsp.log')
    print(f'seed: VBSP {builds[0][0]} exit {rc}')
    rc2, text = run([Path(a.vrad) / 'vrad_rtx.exe', '-game', a.game, seed / f'{a.map}.bsp'], seed / 'vrad.log', cwd=a.vrad)
    print(f'seed: VRAD exit {rc2}')
    for l in text.splitlines():
        if 'proplm' in l.lower():
            print('    ' + l.strip()[:200])
    show('seed pak', proplm_entries(seed / f'{a.map}.bsp'))

    scenarios = (('edit: second prop removed from the VMF, -onlyents', 1, '-onlyents'),
                 ('edit: every prop removed from the VMF, -onlyents', 0, '-onlyents'),
                 ('edit: second prop removed from the VMF, full compile with -keepstalezip', 1, '-keepstalezip'),
                 ('control: VMF unchanged, -onlyents', 2, '-onlyents'))
    for n, (scenario, count, flag) in enumerate(scenarios):
        print(f'\n=== {scenario}')
        for label, exe in builds:
            work = out / f'{n}_{count}prop_{label}'
            work.mkdir()
            shutil.copy2(seed / f'{a.map}.bsp', work / f'{a.map}.bsp')
            make_vmf(work / f'{a.map}.vmf', a.model, count, a.lmres)
            rc, text = run([exe, '-game', a.game, flag, work / f'{a.map}.vmf'], work / 'vbsp.log')
            print(f'{flag} with {label} ({exe}): exit {rc}')
            for l in text.splitlines():
                if re.search(r'proplm|stale|Error', l):
                    print('    ' + l.strip()[:200])
            show('pak after', proplm_entries(work / f'{a.map}.bsp'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
