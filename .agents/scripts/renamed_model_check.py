"""Does a prop_static whose MDL was renamed after compiling survive VBSP and VRAD?

  python -I .agents/scripts/renamed_model_check.py --sdk DIR --source-vpk VPK --out DIR
         --vbsp LABEL=EXE [--vbsp LABEL=EXE ...] --vrad LABEL=BINDIR [--vrad LABEL=BINDIR ...]
         [--source MODEL] [--layout renamed|original]

Builds a scratch copy of the SDK's hl2mp game dir under OUT (nothing is written to
a real install): junctions to the SDK's bin, hl2, hl2_complete and platform folders,
an hl2mp folder with the real gameinfo.txt and hardlinks of its VPKs, and in its
custom/ folder the files of one stock static prop copied byte for byte out of
--source-vpk (a game the scratch dir does not mount, so the model's own name
resolves to nothing there):

  renamed   models/renamed_test/<name>.mdl .vvd .dx90.vtx .dx80.vtx .sw.vtx .phy; the MDL's
            internal name still names the original path, which does not exist
  original  the same files at their own path (the control: nothing is renamed)

Then compiles the fixture with a prop_static of that model with every --vbsp, and
runs every --vrad (-StaticPropLighting -bounce 0) on every BSP, and prints per
step the exit code, the log lines that say why, and for a finished VRAD the
prop's per-texel lightmap (texels non-black). Standard library only. SEMT_REPO
overrides the checkout root.
"""
import argparse
import hashlib
import io
import os
import re
import shutil
import struct
import subprocess
import sys
import zipfile
from pathlib import Path

ROOT = Path(os.environ.get('SEMT_REPO') or Path(__file__).resolve().parents[2])
BIN = ROOT / 'game' / 'bin' / 'x64'
sys.path.insert(0, str(BIN))  # python -I drops this script's folder
import sprp_lightmap  # noqa: E402
import vpk_reader  # noqa: E402

FIXTURE = BIN / 'bsp_unit_tests' / 'validation_vrad_staticpropbounce.vmf'
LUMP_PAKFILE = 40
EXTS = ['.mdl', '.vvd', '.dx90.vtx', '.dx80.vtx', '.sw.vtx', '.phy']
NEW_DIR = 'models/renamed_test'


LINKED = ('bin', 'hl2', 'hl2_complete', 'platform')  # SDK folders the scratch root only points at
HL2MP_FILES = ('gameinfo.txt', 'hl2mp_pak_dir.vpk', 'hl2mp_pak_000.vpk')


def pick_source(vpk):
    """A stock static prop VBSP accepts as prop_static (the $staticprop flag set, no prop_data
    section), with every file of the set and a vertex file between 20 and 100 KB."""
    best = None
    for f in vpk.list_files('models/', 'mdl'):
        base = f[:-4]
        if not all(vpk.has_file(base + e) for e in EXTS if e != '.sw.vtx'):
            continue
        mdl = vpk.read_file(f)
        if not struct.unpack_from('<i', mdl, 152)[0] & 0x10 or b'prop_data' in mdl:
            continue
        size = len(vpk.read_file(base + '.vvd'))
        if 20000 <= size <= 100000 and (best is None or (size, f) < best):
            best = (size, f)
    if best is None:
        sys.exit('no usable static prop in the source VPK')
    return best[1]


def build_scratch(out, sdk, source_vpk, source, layout):
    """A scratch SDK root: junctions to the SDK's read-only folders and its own hl2mp folder (the
    game dir) holding the real gameinfo.txt, hardlinks of the hl2mp VPKs and a custom/ folder
    that gets the model files."""
    sdk = Path(sdk)
    root = out / 'sdk_scratch'
    for name in LINKED:
        if (root / name).exists():
            os.rmdir(root / name)  # a junction: its target is not touched
    if root.exists():
        shutil.rmtree(root)
    game = root / 'hl2mp'
    game.mkdir(parents=True)
    for name in LINKED:
        subprocess.run(['cmd', '/c', 'mklink', '/J', str(root / name), str(sdk / name)], check=True,
                       capture_output=True)
    for name in HL2MP_FILES:
        try:
            os.link(sdk / 'hl2mp' / name, game / name)
        except OSError:
            shutil.copy2(sdk / 'hl2mp' / name, game / name)
    content = game / 'custom' / 'renamed_models'
    vpk = vpk_reader.VPKReader(str(source_vpk))
    source = source or pick_source(vpk)
    base = source[:-4]
    stem = f'{NEW_DIR}/{Path(base).name}_copy' if layout == 'renamed' else base
    written = []
    for ext in EXTS:
        data = vpk.read_file(base + ext)
        if data is None:
            continue
        dst = content / (stem + ext)
        dst.parent.mkdir(parents=True, exist_ok=True)
        dst.write_bytes(data)
        written.append((f'hl2mp/custom/renamed_models/{stem}{ext}', len(data)))
    mdl = (content / (stem + '.mdl')).read_bytes()
    name = mdl[12:76].split(b'\0')[0].decode('latin-1')
    flags, = struct.unpack_from('<i', mdl, 152)
    return game, stem + '.mdl', written, name, flags, source


def make_vmf(dst, model):
    text = FIXTURE.read_bytes().decode('utf-8')
    text = re.sub(r'"model" "models/props_canal/boat001a\.mdl"', f'"model" "{model}"', text)
    text = text.replace('"origin" "-404 431 25"', '"origin" "-404 431 0"')
    dst.write_bytes(text.encode('utf-8'))


def run(cmd, log, cwd=None):
    p = subprocess.run([str(c) for c in cmd], cwd=cwd, capture_output=True, text=True, errors='replace', timeout=1800)
    text = p.stdout + p.stderr
    Path(log).write_text(text, encoding='utf-8')
    return p.returncode, text


def why(text, pattern=r'Error|Unable|checksum|Warning|deleted|Deleted|dropp|refus|vertex|\.vvd|proptexel'):
    lines = [l.strip() for l in text.splitlines() if re.search(pattern, l)]
    seen, out = set(), []
    for l in lines:
        if l not in seen:
            seen.add(l)
            out.append(l[:230])
    return out[:10]


def prop_summary(bsp):
    data = bsp.read_bytes()
    try:
        props = sprp_lightmap.parse(data)
    except SystemExit as e:
        return f'no static prop lump ({e})', (None, None)
    ofs, length = struct.unpack_from('<ii', data, 8 + LUMP_PAKFILE * 16)
    z = zipfile.ZipFile(io.BytesIO(data[ofs:ofs + length]))
    files = {n.lower(): n for n in z.namelist()}
    desc = f'{len(props)} static prop(s): ' + ', '.join(f'{p["model"]}' for p in props)
    lit = None
    digest = None
    for p in props:
        name = f'texelslighting_{p["index"]}.ppl'
        if name in files:
            buf = z.read(files[name])
            digest = hashlib.sha1(buf).hexdigest()[:12]
            _, _, fmt, nmesh = struct.unpack_from('<iIIi', buf, 0)
            total = 0
            for k in range(nmesh):
                lod, o, nbytes, w, h = struct.unpack_from('<5I', buf, 32 + k * 32)
                if lod == 0:
                    px = buf[o:o + w * h * 3]
                    total += sum(1 for i in range(0, len(px), 3) if px[i] or px[i + 1] or px[i + 2])
            lit = (lit or 0) + total
    return desc, (lit, digest)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sdk', required=True, help='SDK root holding hl2mp, hl2 and bin/x64/vphysics.dll')
    ap.add_argument('--source-vpk', required=True)
    ap.add_argument('--source', help='model path in --source-vpk (default: the smallest usable static prop)')
    ap.add_argument('--layout', choices=('renamed', 'original'), default='renamed')
    ap.add_argument('--out', required=True)
    ap.add_argument('--vbsp', action='append', required=True, help='LABEL=EXE')
    ap.add_argument('--vrad', action='append', required=True, help='LABEL=BINDIR holding vrad_rtx.exe')
    a = ap.parse_args()

    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    game, model, written, name, flags, source = build_scratch(out, a.sdk, a.source_vpk, a.source, a.layout)
    print(f'scratch game dir {game}')
    print(f'layout {a.layout}: {source} from {a.source_vpk}')
    for f, n in written:
        print(f'  wrote {f} ({n} bytes)')
    print(f'  MDL internal name "{name}", flags 0x{flags:x}, placed as {model}')
    for vb in a.vbsp:
        vlabel, _, exe = vb.partition('=')
        work = out / vlabel
        work.mkdir(exist_ok=True)
        vmf = work / 'renamed.vmf'
        make_vmf(vmf, model)
        rc, text = run([exe, '-game', game, vmf], work / 'vbsp.log')
        bsp = work / 'renamed.bsp'
        print(f'\nVBSP {vlabel} ({exe}): exit {rc}')
        for l in why(text, r'Error|Unable|Warning|Deleted|deleted|refus|vertex|static prop|\.vvd|checksum'):
            print('    ' + l)
        if rc or not bsp.exists():
            print('    no BSP')
            continue
        desc, _ = prop_summary(bsp)
        print(f'    BSP: {desc}')
        for vr in a.vrad:
            rlabel, _, bindir = vr.partition('=')
            copy = work / f'{rlabel}.bsp'
            copy.write_bytes(bsp.read_bytes())
            rc, text = run([Path(bindir) / 'vrad_rtx.exe', '-game', game, '-StaticPropLighting', '-bounce', '0', copy],
                           work / f'{rlabel}.log', cwd=bindir)
            print(f'  VRAD {rlabel} on the {vlabel} BSP: exit {rc}')
            for l in why(text, r'Error|Unable|checksum|\[proptexel\]|vertex data'):
                print('      ' + l)
            if rc == 0:
                _, (lit, digest) = prop_summary(copy)
                print(f'      per-texel lightmap: {lit} texels non-black at LOD 0, texelslighting_0.ppl sha1 {digest}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
