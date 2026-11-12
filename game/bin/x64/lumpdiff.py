"""Compare two BSPs lump by lump, game lumps (sprp, dprp, ...) one by one.

    python lumpdiff.py a.bsp b.bsp

Exit 0 when every lump is byte-identical, 1 otherwise, 2 if a file cannot
be read as a BSP. The game lump (35) holds absolute file offsets, so it is
compared with those made relative to the lump, and entry by entry (data,
flags, version) for a readable report. The difference count counts these
keys, so one changed game lump can show as several. The header is
compared without the offsets. The pak lump (40) is compared raw.

The acceptance tool for VBSP reproducibility (#106) and for the threaded
block pass (#103); vbsp_repro.py drives it over repeated compiles.
"""
import struct
import sys


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
    try:
        bad = diff(sys.argv[1], sys.argv[2])
    except (OSError, ValueError, struct.error) as e:
        print(f'unreadable BSP: {e}', file=sys.stderr)
        sys.exit(2)
    sys.exit(1 if bad else 0)
