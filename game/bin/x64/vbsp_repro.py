"""Compile one VMF repeatedly with vbsp_lmo and check every BSP is identical.

    python vbsp_repro.py MAP.vmf --game DIR --out DIR [--rounds N]
                         [--variant "-threads 1"] [--variant "-threads 32"]
                         [--vbsp EXE] [--keep]

Each round compiles the map once per variant, interleaved (A B A B ...), so
wall-clock numbers from a loaded box stay comparable. Every BSP is compared
lump by lump (lumpdiff.py) with the first one; a differing BSP is kept, an
identical one deleted unless --keep. Prints one line per compile and a
summary per variant: wall time (median, min) and the block-pass time vbsp
prints. Exit 1 if any BSP differs.

Used for #106 (two runs of one exe identical) and #103 (-threads 1 and
-threads N identical, and faster).
"""
import argparse
import os
import re
import shutil
import statistics
import subprocess
import sys
import time

import lumpdiff

HERE = os.path.dirname(os.path.abspath(__file__))


def compile_once(vbsp, game, vmf, run_dir, args):
    os.makedirs(run_dir, exist_ok=True)
    base = os.path.splitext(os.path.basename(vmf))[0]
    shutil.copy(vmf, run_dir)
    src_dir = os.path.dirname(os.path.abspath(vmf))
    for f in os.listdir(src_dir):
        if f.lower().endswith('.vbsp'):
            shutil.copy(os.path.join(src_dir, f), run_dir)
    target = os.path.join(run_dir, base + '.vmf')
    cmd = [vbsp, '-game', game] + args + [target]
    t0 = time.perf_counter()
    with open(os.path.join(run_dir, 'vbsp.log'), 'w', encoding='utf-8', errors='replace') as log:
        rc = subprocess.call(cmd, cwd=os.path.dirname(vbsp), stdout=log, stderr=subprocess.STDOUT)
    wall = time.perf_counter() - t0
    text = open(os.path.join(run_dir, 'vbsp.log'), encoding='utf-8', errors='replace').read()
    m = re.search(r'Block pass: ([\d.]+) s', text)
    return rc, wall, (float(m.group(1)) if m else None), os.path.join(run_dir, base + '.bsp')


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('vmf')
    p.add_argument('--game', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--rounds', type=int, default=2)
    p.add_argument('--variant', action='append', default=None,
                   help='extra vbsp arguments, one string per variant (default: one empty variant)')
    p.add_argument('--vbsp', default=os.path.join(HERE, 'vbsp_lmo.exe'))
    p.add_argument('--keep', action='store_true')
    a = p.parse_args()
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
            if ref is None:
                ref, verdict = bsp, 'reference'
            else:
                nd = lumpdiff.diff(ref, bsp, quiet=True)
                verdict = 'identical' if nd == 0 else f'{nd} LUMPS DIFFER'
                bad += nd != 0
                if nd == 0 and not a.keep:
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


if __name__ == '__main__':
    sys.exit(main())
