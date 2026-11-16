"""Compare BSPs lump by lump, and check that VBSP output is reproducible.

    python -I bsp_lumps.py diff a.bsp b.bsp
    python -I bsp_lumps.py repro MAP.vmf --game DIR --out DIR [--rounds N]
                            [--variant "-threads 1"] [--variant "-threads 32"]
                            [--vbsp EXE] [--keep]

diff   Compares two BSPs lump by lump, game lumps (sprp, dprp, ...) one by one.
       Exit 0 when the two files are byte-identical, 1 otherwise, 2 if a file
       cannot be read as a BSP. The per-lump lines say where a difference is:
       the game lump (35) holds absolute file offsets, so it is compared with
       those made relative to the lump, and entry by entry (data, flags,
       version); the header is compared without the offsets; the pak lump (40)
       raw. The difference count counts these keys, so one changed game lump
       can show as several. The whole file is compared last: two BSPs whose
       lumps all match but whose lump offsets, gaps between lumps or trailing
       bytes differ are different files, and are reported as such.

repro  Compiles one VMF repeatedly with vbsp_lmo and checks every BSP is
       identical. Each round compiles the map once per variant, interleaved
       (A B A B ...), so wall-clock numbers from a loaded box stay comparable.
       Every BSP is compared lump by lump (diff) with the first one, and so is
       the .prt; a differing BSP is kept, an identical one deleted unless
       --keep. Prints one line per compile and a summary per variant: wall
       time (median, min) and the block-pass time vbsp prints. Exit 1 if any
       BSP or .prt differs, 2 if a compile fails or a BSP cannot be read.

The acceptance tool for VBSP reproducibility (#106) and for the threaded block
pass (#103): repro for #106 (two runs of one exe identical) and #103 (-threads
1 and -threads N identical, and faster).
"""
import argparse
import os
import re
import shutil
import statistics
import struct
import subprocess
import sys
import time
from pathlib import Path

sys.path[:0] = [str(Path(__file__).resolve().parent / p) for p in ('lib', '.')]  # python -I drops the script's folder

import binpaths


# ---------------------------------------------------------------------------
# diff
# ---------------------------------------------------------------------------

def load(path):
    data = open(path, 'rb').read()
    if len(data) < 8 + 16 * 64 + 4:
        raise ValueError(f'{path}: too short for a BSP header')
    lumps = [struct.unpack_from('<iii4s', data, 8 + 16 * i) for i in range(64)]
    for i, (o, n, _v, _cc) in enumerate(lumps):
        if o < 0 or n < 0 or o + n > len(data):
            raise ValueError(f'{path}: lump {i} runs past the end of the file')
    out = {i: data[o:o + n] for i, (o, n, _v, _cc) in enumerate(lumps)}
    # Everything in the header but the offsets: ident, BSP version, each
    # lump's length, version and fourCC, the map revision.
    out['header'] = data[0:8] + b''.join(struct.pack('<ii4s', n, v, cc) for (_o, n, v, cc) in lumps) + data[8 + 16 * 64:8 + 16 * 64 + 4]
    o, n = lumps[35][0], lumps[35][1]
    if n:
        if n < 4:
            raise ValueError(f'{path}: game lump directory shorter than its count')
        count, = struct.unpack_from('<i', data, o)
        if count < 0 or 4 + 16 * count > n:
            raise ValueError(f'{path}: game lump directory holds {count} entries in {n} bytes')
        order = []
        for k in range(count):
            gid, fl, ver, fo, fn = struct.unpack_from('<4sHHii', data, o + 4 + 16 * k)
            if fo < 0 or fn < 0 or fo + fn > len(data):
                raise ValueError(f'{path}: game lump {gid!r} runs past the end of the file')
            # keyed by position too, so a duplicate id cannot hide the other
            key = f'game:{k}:' + gid[::-1].decode('latin1')
            out[key] = data[fo:fo + fn]
            out[key + ':flags,version'] = struct.pack('<HH', fl, ver)
            order.append(gid)
        out['game:directory order'] = b''.join(order)
        # The whole lump with each entry's file offset made relative to the
        # lump, so bytes outside the directory and the entries count too.
        rel = bytearray(data[o:o + n])
        for k in range(count):
            fo, = struct.unpack_from('<i', data, o + 4 + 16 * k + 8)
            struct.pack_into('<i', rel, 4 + 16 * k + 8, fo - o)
        out['game:lump 35, offsets relative'] = bytes(rel)
    # Last, the file as it is on disk: catches what the keys above leave out
    # (lump offsets, bytes between lumps, bytes after the last lump).
    out['whole file (offsets, gaps, trailing bytes)'] = data
    return out


def diff(path_a, path_b, quiet=False):
    """Return the number of differing lumps; print one line per difference."""
    a, b = load(path_a), load(path_b)
    bad = 0
    for k in sorted(set(a) | set(b), key=str):
        if k == 35:
            continue
        x, y = a.get(k, b''), b.get(k, b'')
        if x == y:
            continue
        bad += 1
        if not quiet:
            m = min(len(x), len(y))
            first = next((i for i in range(m) if x[i] != y[i]), m)
            ndiff = sum(1 for i in range(m) if x[i] != y[i])
            print(f'lump {k}: DIFF len {len(x)} vs {len(y)}, first byte {first}, {ndiff} bytes differ')
    if not quiet:
        n = sum(1 for k in a if k != 35 and a[k])
        print(f'{n} non-empty lumps compared, {bad} differ' + ('' if bad else ' -> IDENTICAL'))
    return bad


def cmd_diff(args):
    try:
        bad = diff(args.a, args.b)
    except (OSError, ValueError, struct.error) as e:
        print(f'unreadable BSP: {e}', file=sys.stderr)
        return 2
    return 1 if bad else 0


# ---------------------------------------------------------------------------
# repro
# ---------------------------------------------------------------------------

def compile_once(vbsp, game, vmf, run_dir, args):
    os.makedirs(run_dir, exist_ok=True)
    base = os.path.splitext(os.path.basename(vmf))[0]
    shutil.copy(vmf, run_dir)
    src_dir = os.path.dirname(os.path.abspath(vmf))
    for f in os.listdir(src_dir):
        if f.lower().endswith('.vbsp'):
            shutil.copy(os.path.join(src_dir, f), run_dir)
    target = os.path.join(run_dir, base + '.vmf')
    for ext in ('.bsp', '.prt', '.lin'):
        if os.path.exists(os.path.join(run_dir, base + ext)):
            os.remove(os.path.join(run_dir, base + ext))
    cmd = [vbsp, '-game', game] + args + [target]
    t0 = time.perf_counter()
    with open(os.path.join(run_dir, 'vbsp.log'), 'w', encoding='utf-8', errors='replace') as log:
        rc = subprocess.call(cmd, cwd=os.path.dirname(vbsp), stdout=log, stderr=subprocess.STDOUT)
    wall = time.perf_counter() - t0
    text = open(os.path.join(run_dir, 'vbsp.log'), encoding='utf-8', errors='replace').read()
    m = re.search(r'Block pass: ([\d.]+) s', text)
    return rc, wall, (float(m.group(1)) if m else None), os.path.join(run_dir, base + '.bsp')


def cmd_repro(a):
    if a.rounds < 1:
        a.parser.error('--rounds needs 1 or more')
    a.out, a.game, a.vmf = os.path.abspath(a.out), os.path.abspath(a.game), os.path.abspath(a.vmf)
    variants = a.variant or ['']
    ref = None
    bad = 0
    stats = {v: ([], []) for v in variants}
    for r in range(a.rounds):
        for vi, v in enumerate(variants):
            run_dir = os.path.join(a.out, f'r{r:02d}_v{vi}')
            rc, wall, block, bsp = compile_once(os.path.abspath(a.vbsp), a.game, a.vmf, run_dir, v.split())
            if rc != 0 or not os.path.exists(bsp):
                print(f'round {r} [{v}]: vbsp exit {rc}, see {run_dir}/vbsp.log')
                return 2
            stats[v][0].append(wall)
            if block is not None:
                stats[v][1].append(block)
            prt = bsp[:-4] + '.prt'
            prt_data = open(prt, 'rb').read() if os.path.exists(prt) else None
            if ref is None:
                try:
                    load(bsp)
                except (OSError, ValueError, struct.error) as e:
                    print(f'round {r} [{v}]: unreadable BSP: {e}')
                    return 2
                ref, ref_prt, verdict = bsp, prt_data, 'reference'
            else:
                try:
                    nd = diff(ref, bsp, quiet=True)
                except (OSError, ValueError, struct.error) as e:
                    print(f'round {r} [{v}]: unreadable BSP: {e}')
                    return 2
                prt_same = prt_data == ref_prt
                verdict = ('identical' if nd == 0 else f'{nd} LUMP KEYS DIFFER') + ('' if prt_same else ', PRT DIFFERS')
                bad += nd != 0 or not prt_same
                if nd == 0 and prt_same and not a.keep:
                    os.remove(bsp)
            blk = f', block pass {block:.2f} s' if block is not None else ''
            print(f'round {r} [{v or "default"}]: wall {wall:.2f} s{blk}: {verdict}', flush=True)
    for v, (walls, blocks) in stats.items():
        line = f'[{v or "default"}] wall median {statistics.median(walls):.2f} min {min(walls):.2f}'
        if blocks:
            line += f'; block pass median {statistics.median(blocks):.2f} min {min(blocks):.2f}'
        print(line)
    print(f'{a.rounds * len(variants)} compiles, {bad} differ from the first')
    return 1 if bad else 0


# ---------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='command', required=True)

    d = sub.add_parser('diff', help='compare two BSPs lump by lump')
    d.add_argument('a')
    d.add_argument('b')
    d.set_defaults(func=cmd_diff)

    r = sub.add_parser('repro', help='compile one VMF repeatedly and compare every BSP with the first')
    r.add_argument('vmf')
    r.add_argument('--game', required=True)
    r.add_argument('--out', required=True)
    r.add_argument('--rounds', type=int, default=2)
    r.add_argument('--variant', action='append', default=None,
                   help='extra vbsp arguments, one string per variant (default: one empty variant)')
    r.add_argument('--vbsp', default=str(binpaths.exe('vbsp_lmo.exe')))
    r.add_argument('--keep', action='store_true')
    r.set_defaults(func=cmd_repro, parser=r)

    a = p.parse_args()
    return a.func(a)


if __name__ == '__main__':
    sys.exit(main())
