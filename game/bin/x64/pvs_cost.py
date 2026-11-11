"""What the engine would draw, on average, given a BSP's PVS.

Cluster counts cannot be compared between two compiles of the same map that
cut space differently (other block size, hints, visclusters). This prints
numbers that can: for a viewpoint chosen uniformly by volume in the map's
empty leaves, the expected leaves, faces and triangles in the PVS.

    python pvs_cost.py a.bsp [b.bsp ...] [--json]

Weights are leaf bounding-box volumes, which overstate non-box leaves; the
same bias applies to every BSP of one map. Static props are not counted.
Needs numpy. Lumps must be uncompressed (no bspzip -repack).
"""
import json
import struct
import sys

import numpy as np

from bsp_reader import BSPReader

LUMP_VISIBILITY, LUMP_FACES, LUMP_LEAFS, LUMP_LEAFFACES, LUMP_DISPINFO = 4, 7, 10, 16, 26
DFACE_SIZE, DDISPINFO_SIZE = 56, 176


def _leaf_stride(reader):
    data = reader._get_lump_data(LUMP_LEAFS)
    # Lump version 0 carries 24 bytes of ambient lighting per leaf.
    return data, (56 if reader._lumps[LUMP_LEAFS].version == 0 else 32)


def _pvs_row(vis, ofs, nclusters):
    """Decompress one run-length PVS row into a bool array."""
    out = np.zeros(nclusters, dtype=bool)
    c = 0
    while c < nclusters:
        byte = vis[ofs]
        ofs += 1
        if byte == 0:
            c += 8 * vis[ofs]
            ofs += 1
            continue
        for bit in range(8):
            if byte & (1 << bit) and c + bit < nclusters:
                out[c + bit] = True
        c += 8
    return out


def measure(path):
    r = BSPReader(path)
    r.read()
    for lump in (LUMP_VISIBILITY, LUMP_FACES, LUMP_LEAFS, LUMP_LEAFFACES, LUMP_DISPINFO):
        if r._lumps[lump].uncompressed_size != 0:
            raise ValueError(f"{path}: lump {lump} is compressed; decompress the BSP first")

    faces = r._get_lump_data(LUMP_FACES)
    nfaces = len(faces) // DFACE_SIZE
    disp = r._get_lump_data(LUMP_DISPINFO)
    tris = np.zeros(nfaces, dtype=np.int64)
    for i in range(nfaces):
        numedges, _texinfo, dispinfo = struct.unpack_from('<hhh', faces, i * DFACE_SIZE + 8)
        if dispinfo >= 0:
            power = struct.unpack_from('<i', disp, dispinfo * DDISPINFO_SIZE + 20)[0]
            tris[i] = 2 * (1 << power) ** 2
        else:
            tris[i] = max(numedges - 2, 0)

    leaffaces = np.frombuffer(r._get_lump_data(LUMP_LEAFFACES), dtype='<u2')
    leafdata, stride = _leaf_stride(r)
    nleafs = len(leafdata) // stride

    visinfo = r.read_visibility()
    if visinfo is None:
        raise ValueError(f"{path}: no visibility lump")
    nclusters, offsets, vis = visinfo

    cl_vol = np.zeros(nclusters)
    cl_leafs = np.zeros(nclusters, dtype=np.int64)
    cl_faces = [[] for _ in range(nclusters)]
    for i in range(nleafs):
        _contents, cluster, _areaflags, x0, y0, z0, x1, y1, z1, firstface, numfaces = \
            struct.unpack_from('<ihh6hHH', leafdata, i * stride)
        if cluster < 0 or cluster >= nclusters:
            continue
        cl_vol[cluster] += float(x1 - x0) * (y1 - y0) * (z1 - z0)
        cl_leafs[cluster] += 1
        cl_faces[cluster].append(leaffaces[firstface:firstface + numfaces])
    cl_faces = [np.unique(np.concatenate(f)) if f else np.zeros(0, dtype='<u2') for f in cl_faces]

    total_vol = cl_vol.sum()
    w = cl_vol / total_vol
    seen_clusters = np.zeros(nclusters)
    seen_leafs = np.zeros(nclusters)
    seen_vol = np.zeros(nclusters)
    seen_faces = np.zeros(nclusters)
    seen_tris = np.zeros(nclusters)
    mark = np.zeros(nfaces, dtype=bool)
    for c in range(nclusters):
        row = _pvs_row(vis, offsets[c][0], nclusters)
        idx = np.flatnonzero(row)
        seen_clusters[c] = len(idx)
        seen_leafs[c] = cl_leafs[idx].sum()
        seen_vol[c] = cl_vol[idx].sum() / total_vol
        mark[:] = False
        for v in idx:
            mark[cl_faces[v]] = True
        seen_faces[c] = mark.sum()
        seen_tris[c] = tris[mark].sum()

    return {
        "bsp": str(path),
        "clusters": int(nclusters),
        "leafs_in_clusters": int(cl_leafs.sum()),
        "faces": int(nfaces),
        "triangles": int(tris.sum()),
        "vis_lump_bytes": len(vis),
        "mean_pvs_clusters_pct": round(100.0 * seen_clusters.mean() / nclusters, 2),
        "expected_leafs": round(float((w * seen_leafs).sum()), 1),
        "expected_volume_pct": round(100.0 * float((w * seen_vol).sum()), 2),
        "expected_faces": round(float((w * seen_faces).sum()), 1),
        "expected_triangles": round(float((w * seen_tris).sum()), 1),
        "worst_triangles": int(seen_tris.max()),
    }


def main(argv):
    paths = [a for a in argv if not a.startswith('--')]
    if not paths:
        print(__doc__)
        return 2
    rows = [measure(p) for p in paths]
    if '--json' in argv:
        print(json.dumps(rows, indent=1))
        return 0
    keys = [k for k in rows[0] if k != "bsp"]
    width = max(len(k) for k in keys)
    for k in keys:
        print(k.ljust(width), *(str(row[k]).rjust(11) for row in rows))
    for i, row in enumerate(rows):
        print(f"  column {i + 1}: {row['bsp']}")
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
