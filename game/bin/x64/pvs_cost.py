"""What the engine would draw, on average, given a BSP's PVS.

Cluster counts cannot be compared between two compiles of the same map that
cut space differently (other block size, hints, visclusters). This prints
numbers that can: the expected leaves, faces, triangles and static props in
the PVS of a random viewpoint, under two ways of choosing the viewpoint.

    python pvs_cost.py a.bsp [b.bsp ...] [--json]

floor_* numbers: the viewpoint is uniform over the map's floor area (drawn
world faces and displacements a player can stand on: normal z above 0.7, and
not sky, water, nodraw or a tool surface; a face several leaves share is
divided between them). This is the one to read for what a player sees. floor_seen_pct is the
share of the map's floor in the PVS: the chance that two random spots on the
floor may see each other.

expected_* numbers: the viewpoint is uniform by volume. Weights are leaf
bounding-box volumes, which overstate non-box leaves. One huge leaf nobody
stands in (sky, an enclosing void) can hold most of the weight, and then these
numbers follow that leaf's PVS and little else: d2_prison_05 reads 90 % of its
volume visible while the mean cluster sees 14 % of the clusters. Kept because
the #96 and #105 tables use them.

Static props are counted as props
(expected_props), not as triangles: a prop is in the PVS when any leaf it
touches is, and fade distances are ignored. The triangle numbers are world
faces and displacements only.
Needs numpy. Lumps must be uncompressed (no bspzip -repack).
"""
import json
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # python -I drops this folder
from bsp_reader import BSPReader  # noqa: E402

LUMP_PLANES, LUMP_VISIBILITY, LUMP_TEXINFO, LUMP_FACES = 1, 4, 6, 7
LUMP_LEAFS, LUMP_LEAFFACES, LUMP_DISPINFO = 10, 16, 26
FLOOR_NORMAL_Z = 0.7  # the engine's own limit for ground a player can stand on
TEXINFO_SIZE, TEXINFO_FLAGS_OFS = 72, 64
# Upward faces that are not ground: sky (2D and 3D), water surfaces, triggers,
# nodraw, hint and skip. They are 40 to 80 % of the upward face area of an HL2
# map (the sky box alone is 79 % on d1_trainstation_03).
SURF_NOT_FLOOR = 0x2 | 0x4 | 0x8 | 0x40 | 0x80 | 0x100 | 0x200
LUMP_GAME, SPRP = 35, 0x73707270
DFACE_SIZE, DDISPINFO_SIZE = 56, 176


def _static_props(path):
    """The static prop leaf list and each prop's (first leaf, leaf count) in it.

    None when the BSP has no static prop lump. The prop record size is taken
    from the lump length, so every lump version reads the same way.
    """
    with open(path, 'rb') as f:
        data = f.read()
    ofs, length = struct.unpack_from('<ii', data, 8 + LUMP_GAME * 16)
    if length < 4:
        return None
    (count,) = struct.unpack_from('<i', data, ofs)
    for i in range(count):
        gid, gflags, _gver, gofs, glen = struct.unpack_from('<iHHii', data, ofs + 4 + i * 16)
        if gid != SPRP:
            continue
        if gflags & 1:
            raise ValueError(f"{path}: the static prop lump is compressed; decompress the BSP first")
        (ndict,) = struct.unpack_from('<i', data, gofs)
        p = gofs + 4 + ndict * 128
        (nleaf,) = struct.unpack_from('<i', data, p)
        leaves = np.frombuffer(data, dtype='<u2', count=nleaf, offset=p + 4)
        p += 4 + nleaf * 2
        (nprops,) = struct.unpack_from('<i', data, p)
        p += 4
        if nprops == 0:
            return leaves, []
        size, rest = divmod(gofs + glen - p, nprops)
        if rest or size < 30:
            raise ValueError(f"{path}: static prop lump does not divide into {nprops} props")
        # origin 12, angles 12, prop type 2, then first leaf and leaf count
        return leaves, [struct.unpack_from('<HH', data, p + k * size + 26) for k in range(nprops)]
    return None


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
    for lump in (LUMP_PLANES, LUMP_VISIBILITY, LUMP_TEXINFO, LUMP_FACES, LUMP_LEAFS, LUMP_LEAFFACES,
                 LUMP_DISPINFO):
        if r._lumps[lump].uncompressed_size != 0:
            raise ValueError(f"{path}: lump {lump} is compressed; decompress the BSP first")

    faces = r._get_lump_data(LUMP_FACES)
    nfaces = len(faces) // DFACE_SIZE
    disp = r._get_lump_data(LUMP_DISPINFO)
    tris = np.zeros(nfaces, dtype=np.int64)
    # dplane_t is a normal, a distance and an int: five 4-byte fields
    plane_z = np.frombuffer(r._get_lump_data(LUMP_PLANES), dtype='<f4').reshape(-1, 5)[:, 2]
    texinfo = r._get_lump_data(LUMP_TEXINFO)
    floor_area = np.zeros(nfaces)
    for i in range(nfaces):
        planenum, side = struct.unpack_from('<HB', faces, i * DFACE_SIZE)
        numedges, ti, dispinfo = struct.unpack_from('<hhh', faces, i * DFACE_SIZE + 8)
        if (-plane_z[planenum] if side else plane_z[planenum]) > FLOOR_NORMAL_Z:
            surf = struct.unpack_from('<i', texinfo, ti * TEXINFO_SIZE + TEXINFO_FLAGS_OFS)[0]
            if not surf & SURF_NOT_FLOOR:
                floor_area[i] = struct.unpack_from('<f', faces, i * DFACE_SIZE + 24)[0]
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
    leaf_cluster = np.full(nleafs, -1, dtype=np.int64)
    for i in range(nleafs):
        _contents, cluster, _areaflags, x0, y0, z0, x1, y1, z1, firstface, numfaces = \
            struct.unpack_from('<ihh6hHH', leafdata, i * stride)
        if cluster < 0 or cluster >= nclusters:
            continue
        leaf_cluster[i] = cluster
        cl_vol[cluster] += float(x1 - x0) * (y1 - y0) * (z1 - z0)
        cl_leafs[cluster] += 1
        cl_faces[cluster].append(leaffaces[firstface:firstface + numfaces])
    # floor area per cluster; a face listed by several leaves is shared out
    face_refs = np.zeros(nfaces)
    for chunks in cl_faces:
        for chunk in chunks:
            np.add.at(face_refs, chunk, 1)
    cl_floor = np.array([sum(float((floor_area[c] / face_refs[c]).sum()) for c in chunks)
                         for chunks in cl_faces])
    cl_faces = [np.unique(np.concatenate(f)) if f else np.zeros(0, dtype='<u2') for f in cl_faces]

    cl_props = [[] for _ in range(nclusters)]
    nprops = 0
    static_props = _static_props(path)
    if static_props:
        prop_leaves, spans = static_props
        nprops = len(spans)
        for k, (first, count) in enumerate(spans):
            for cluster in set(leaf_cluster[prop_leaves[first:first + count]].tolist()):
                if cluster >= 0:
                    cl_props[cluster].append(k)
    cl_props = [np.array(p, dtype=np.int64) for p in cl_props]

    total_vol = cl_vol.sum()
    w = cl_vol / total_vol
    total_floor = cl_floor.sum()
    wf = cl_floor / total_floor if total_floor > 0 else np.zeros(nclusters)
    seen_floor = np.zeros(nclusters)
    seen_clusters = np.zeros(nclusters)
    seen_leafs = np.zeros(nclusters)
    seen_vol = np.zeros(nclusters)
    seen_faces = np.zeros(nclusters)
    seen_tris = np.zeros(nclusters)
    seen_props = np.zeros(nclusters)
    mark = np.zeros(nfaces, dtype=bool)
    prop_mark = np.zeros(nprops, dtype=bool)
    for c in range(nclusters):
        row = _pvs_row(vis, offsets[c][0], nclusters)
        idx = np.flatnonzero(row)
        seen_clusters[c] = len(idx)
        seen_leafs[c] = cl_leafs[idx].sum()
        seen_vol[c] = cl_vol[idx].sum() / total_vol
        seen_floor[c] = cl_floor[idx].sum() / total_floor if total_floor > 0 else 0.0
        mark[:] = False
        prop_mark[:] = False
        for v in idx:
            mark[cl_faces[v]] = True
            prop_mark[cl_props[v]] = True
        seen_faces[c] = mark.sum()
        seen_tris[c] = tris[mark].sum()
        seen_props[c] = prop_mark.sum()

    return {
        "bsp": str(path),
        "clusters": int(nclusters),
        "leafs_in_clusters": int(cl_leafs.sum()),
        "faces": int(nfaces),
        "triangles": int(tris.sum()),
        "static_props": nprops,
        "vis_lump_bytes": len(vis),
        "mean_pvs_clusters_pct": round(100.0 * seen_clusters.mean() / nclusters, 2),
        "expected_leafs": round(float((w * seen_leafs).sum()), 1),
        "expected_volume_pct": round(100.0 * float((w * seen_vol).sum()), 2),
        "expected_faces": round(float((w * seen_faces).sum()), 1),
        "expected_triangles": round(float((w * seen_tris).sum()), 1),
        "expected_props": round(float((w * seen_props).sum()), 1),
        "worst_triangles": int(seen_tris.max()),
        "floor_seen_pct": round(100.0 * float((wf * seen_floor).sum()), 2),
        "floor_expected_leafs": round(float((wf * seen_leafs).sum()), 1),
        "floor_expected_triangles": round(float((wf * seen_tris).sum()), 1),
        "floor_expected_props": round(float((wf * seen_props).sum()), 1),
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
