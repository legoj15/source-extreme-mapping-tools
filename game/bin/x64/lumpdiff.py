"""Compare two BSPs lump by lump, game lumps (sprp, dprp, ...) one by one.

    python lumpdiff.py a.bsp b.bsp

Exit 0 when every lump is byte-identical, 1 otherwise. The game lump
directory (lump 35) holds absolute file offsets, so it is compared through
its entries instead of as raw bytes. The pak lump (40) is compared raw.

The acceptance tool for VBSP reproducibility (#106) and for the threaded
block pass (#103); vbsp_repro.py drives it over repeated compiles.
"""
import struct
import sys


def load(path):
    data = open(path, 'rb').read()
    lumps = [struct.unpack_from('<iii4s', data, 8 + 16 * i) for i in range(64)]
    out = {i: data[o:o + n] for i, (o, n, _v, _cc) in enumerate(lumps)}
    o, n = lumps[35][0], lumps[35][1]
    if n:
        count, = struct.unpack_from('<i', data, o)
        for k in range(count):
            gid, _fl, _ver, fo, fn = struct.unpack_from('<4sHHii', data, o + 4 + 16 * k)
            out['game:' + gid[::-1].decode('latin1')] = data[fo:fo + fn]
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


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    sys.exit(1 if diff(sys.argv[1], sys.argv[2]) else 0)
