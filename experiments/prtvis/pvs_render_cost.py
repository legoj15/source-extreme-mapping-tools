#!/usr/bin/env python3
"""What the extra PVS of one vvis'd BSP costs the world renderer, against another's (issue #122).

Issue #122 asks whether VBSP should cull portals under about 1 unit wide. Dropping
them from the .prt loses up to 1.6% of the visible cluster pairs. This script puts
a render cost on those pairs: how much world geometry the engine draws for the
pairs the culled map no longer sees. Read-only; nothing is compiled or written
except the optional --json.

Usage:
  python -I experiments/prtvis/pvs_render_cost.py <full.bsp> <culled.bsp> [--json out.json]

Both files must be vvis output of the SAME map (same tree, same clusters), differing
only in the visibility lump (LUMP 4) and in the two things vvis itself derives from the
PVS (lump 46, each leaf's distance to water, and the CONTENTS_TESTFOGVOLUME bit of leaf
contents; a difference there is printed as a note). The script exits 2 with a message
when any other lump differs (the game lump is compared sub-lump by sub-lump, since its
directory holds file offsets that move with the visibility lump) or when the cluster
counts differ. Lumps are read plain; a compressed lump is refused.

Model (the Source world renderer, minus the view): a viewer in cluster A marks every
open leaf (cluster >= 0) whose cluster is in PVS(A). It then draws
  leaves          the marked leaves
  faces           the faces in the marked leaves' leaf-face lists (LUMP_LEAFFACES),
                  each once however many leaves list it. Not counted: faces with a
                  displacement (dispinfo != -1) and faces whose texinfo is SURF_NODRAW,
                  SURF_SKY, SURF_SKY2D, SURF_HINT or SURF_SKIP (tallied in the map
                  summary, excluded from faces and triangles)
  triangles       numedges - 2 per counted face
  displacements   each displacement is linked to the leaves its bounding box touches:
                  the box is the base face's box widened on each axis by the smallest
                  and the largest vec * dist of its vertices (conservative: it holds
                  every displaced vertex; 10 to 25% fewer leaves on the test maps than
                  growing by the largest |vec * dist| both ways), cut down the node tree
  disp triangles  2 * (2^power)^2 per displacement
  static props    a prop is drawn when any leaf in its leaf list is marked
There is no frustum or distance culling (there is no viewpoint): every item in the
PVS counts, so the numbers are an upper bound on what any real view draws. Brush
entities (func_brush and the like) and detail sprites are outside the model.

Per viewer cluster A the drawn set is taken under the full PVS and under the culled
PVS; extra = |drawn(full) minus drawn(culled)|, a set difference. When the culled PVS
is not a subset of the full one (pairs only the culled map sees) a warning counts them.
A cluster the culled PVS leaves seeing only itself (the full PVS sees more) is listed:
for it the "extra" is a hole in the view, not a saving.

The output is a table over all clusters (mean drawn under the full PVS, mean extra,
extra as a percentage of that mean, the largest extra of any one cluster, and the
largest extra as a percentage of that cluster's own full amount, each with its
cluster number) and the mean columns again over only the clusters whose PVS differs.
A brute-force recount with plain set unions on a sample of viewers guards the fast
path; the script exits 3 if the two disagree.
"""
import argparse
import json
import os
import struct
import sys
from itertools import compress

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)  # python -I drops it
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..", "..", ".agents", "scripts")))

from sliver_stats import LUMP_LEAFS, LUMP_MODELS, LUMP_NODES, LUMP_PLANES, read_bsp  # noqa: E402
from vvis_pvs_compare import pairs_only_in, pvs_rows, vis_lump  # noqa: E402

LUMP_VERTEXES, LUMP_VISIBILITY, LUMP_TEXINFO, LUMP_FACES = 3, 4, 6, 7
LUMP_EDGES, LUMP_SURFEDGES, LUMP_LEAFFACES = 12, 13, 16
LUMP_DISPINFO, LUMP_DISP_VERTS, LUMP_GAME_LUMP, LUMP_LEAFMINDISTTOWATER = 26, 33, 35, 46
CONTENTS_TESTFOGVOLUME = 0x100
LUMP_NAMES = {0: "entities", 1: "planes", 2: "texdata", 3: "vertexes", 4: "visibility",
              5: "nodes", 6: "texinfo", 7: "faces", 8: "lighting", 9: "occlusion",
              10: "leafs", 11: "faceids", 12: "edges", 13: "surfedges", 14: "models",
              15: "worldlights", 16: "leaffaces", 17: "leafbrushes", 18: "brushes",
              19: "brushsides", 20: "areas", 21: "areaportals", 26: "dispinfo",
              27: "originalfaces", 33: "disp_verts", 35: "game lump", 40: "pakfile",
              53: "lighting hdr", 58: "faces hdr"}
SURF_SKY2D, SURF_SKY, SURF_NODRAW, SURF_HINT, SURF_SKIP = 0x2, 0x4, 0x80, 0x100, 0x200
EXCLUDED_FLAGS = (("nodraw", SURF_NODRAW), ("sky", SURF_SKY), ("sky2d", SURF_SKY2D),
                  ("hint", SURF_HINT), ("skip", SURF_SKIP))
KNOWN_PROP_SIZES = (56, 60, 64, 68, 72, 76)  # static prop lump versions 4 to 11
FACE_SIZE, TEXINFO_SIZE, DISPINFO_SIZE, DISPVERT_SIZE = 56, 72, 176, 20

# (metric name, group, column in the group's weight tuple)
METRICS = (("leaves", "leaves", 0), ("faces", "faces", 0), ("triangles", "faces", 1),
           ("displacements", "disps", 0), ("disp triangles", "disps", 1),
           ("static props", "props", 0))
GROUP_COLUMNS = {"leaves": 1, "faces": 2, "disps": 2, "props": 1}

_TO01 = bytes.maketrans(b"01", b"\x00\x01")


def refuse(msg):
    print(f"pvs_render_cost: {msg}", file=sys.stderr)
    sys.exit(2)


# ---------------------------------------------------------------------------
# the two files must be the same map


def game_sublumps(data, path):
    """[(id, flags, version, bytes)] of the game lump's directory."""
    ofs, ln = struct.unpack_from("<ii", data, 8 + LUMP_GAME_LUMP * 16)
    if ln < 4:
        return []
    count = struct.unpack_from("<i", data, ofs)[0]
    out = []
    for i in range(count):
        gid, flags, ver, fofs, flen = struct.unpack_from("<4sHHii", data, ofs + 4 + i * 16)
        if fofs < 0 or flen < 0 or fofs + flen > len(data):
            refuse(f"{path}: game lump entry {gid!r} points outside the file")
        out.append((gid, flags, ver, data[fofs:fofs + flen]))
    return out


def differing_lumps(a, b, path_a, path_b):
    """(lumps whose content differs, notes on what vvis derives from the PVS and so may differ).

    Other than the visibility lump itself, vvis writes two things from the PVS: lump 46,
    each leaf's distance to water, and the CONTENTS_TESTFOGVOLUME bit of leaf contents.
    Those two are allowed to differ and are reported in the notes."""
    diff, notes = [], []
    if struct.unpack_from("<i", a, 4)[0] != struct.unpack_from("<i", b, 4)[0]:
        refuse(f"BSP versions differ: {path_a} vs {path_b}")
    for i in range(64):
        if i == LUMP_VISIBILITY:
            continue
        oa, la, va, ua = struct.unpack_from("<iiii", a, 8 + i * 16)
        ob, lb, vb, ub = struct.unpack_from("<iiii", b, 8 + i * 16)
        if (la, va, ua) != (lb, vb, ub):
            diff.append(i)
            continue
        ra, rb = a[oa:oa + la], b[ob:ob + lb]
        if i == LUMP_GAME_LUMP:
            if game_sublumps(a, path_a) != game_sublumps(b, path_b):
                diff.append(i)
        elif ra == rb:
            continue
        elif i == LUMP_LEAFMINDISTTOWATER:
            n = sum(1 for x, y in zip(struct.unpack(f"<{la // 2}H", ra),
                                      struct.unpack(f"<{lb // 2}H", rb)) if x != y)
            notes.append(f"lump 46 (leaf distance to water, derived from the PVS) differs in {n} of {la // 2} entries")
        elif i == LUMP_LEAFS:
            size = 56 if va == 0 else 32
            fog = 0
            for o in range(0, la - size + 1, size):
                ca, cb = struct.unpack_from("<i", ra, o)[0], struct.unpack_from("<i", rb, o)[0]
                if ra[o + 4:o + size] != rb[o + 4:o + size] or (ca ^ cb) & ~CONTENTS_TESTFOGVOLUME:
                    diff.append(i)
                    break
                fog += ca != cb
            else:
                notes.append(f"leaf lump: CONTENTS_TESTFOGVOLUME (derived from the PVS) differs in "
                             f"{fog} of {la // size} leaves")
        else:
            diff.append(i)
    return diff, notes


# ---------------------------------------------------------------------------
# the world as the renderer sees it


class World:
    pass


def read_world(path, data, lump):
    w = World()
    raw, _ = lump(LUMP_PLANES)
    w.planes = [struct.unpack_from("<4f", raw, i * 20) for i in range(len(raw) // 20)]
    raw, _ = lump(LUMP_NODES)
    w.nodes = [struct.unpack_from("<iii", raw, i * 32) for i in range(len(raw) // 32)]
    raw, _ = lump(LUMP_MODELS)
    w.headnode = struct.unpack_from("<i", raw, 36)[0]

    raw, ver = lump(LUMP_LEAFS)
    size = 56 if ver == 0 else 32  # version 0 carries a 24-byte ambient light block
    if len(raw) % size:
        refuse(f"{path}: leaf lump (version {ver}) is not a whole number of {size}-byte leaves")
    w.leaf_version = ver
    w.leafs = []  # (contents, cluster, first leaf face, number of leaf faces)
    for i in range(len(raw) // size):
        contents, cluster = struct.unpack_from("<ih", raw, i * size)
        first, num = struct.unpack_from("<HH", raw, i * size + 20)
        w.leafs.append((contents, cluster, first, num))

    raw, _ = lump(LUMP_LEAFFACES)
    w.leaffaces = struct.unpack(f"<{len(raw) // 2}H", raw)

    raw, _ = lump(LUMP_FACES)
    if len(raw) % FACE_SIZE:
        refuse(f"{path}: face lump is not a whole number of {FACE_SIZE}-byte faces")
    w.faces = [struct.unpack_from("<HBBihhh", raw, i * FACE_SIZE)  # plane, side, onNode, firstedge, numedges, texinfo, dispinfo
               for i in range(len(raw) // FACE_SIZE)]
    raw, _ = lump(LUMP_TEXINFO)
    w.texflags = [struct.unpack_from("<i", raw, i * TEXINFO_SIZE + 64)[0]
                  for i in range(len(raw) // TEXINFO_SIZE)]

    raw, _ = lump(LUMP_SURFEDGES)
    w.surfedges = struct.unpack(f"<{len(raw) // 4}i", raw)
    raw, _ = lump(LUMP_EDGES)
    w.edges = list(struct.iter_unpack("<HH", raw))
    raw, _ = lump(LUMP_VERTEXES)
    w.verts = list(struct.iter_unpack("<3f", raw))

    raw, _ = lump(LUMP_DISPINFO)
    if len(raw) % DISPINFO_SIZE:
        refuse(f"{path}: dispinfo lump is not a whole number of {DISPINFO_SIZE}-byte entries")
    w.dispinfo = [(struct.unpack_from("<i", raw, i * DISPINFO_SIZE + 12)[0],   # first disp vert
                   struct.unpack_from("<i", raw, i * DISPINFO_SIZE + 20)[0],   # power
                   struct.unpack_from("<H", raw, i * DISPINFO_SIZE + 36)[0])   # base face
                  for i in range(len(raw) // DISPINFO_SIZE)]
    raw, _ = lump(LUMP_DISP_VERTS)
    if len(raw) % DISPVERT_SIZE:
        refuse(f"{path}: disp vertex lump is not a whole number of {DISPVERT_SIZE}-byte entries")
    w.dispverts = [(x, y, z, d) for x, y, z, d, _alpha in struct.iter_unpack("<5f", raw)]

    read_props(w, path, data)
    return w


def read_props(w, path, data):
    """Static props: for each prop, its slice of the prop leaf list."""
    w.props = []        # (first leaf entry, leaf count)
    w.prop_leaves = ()
    w.prop_version = None
    w.prop_stride = None
    ofs, ln = struct.unpack_from("<ii", data, 8 + LUMP_GAME_LUMP * 16)
    if ln < 4:
        return
    count = struct.unpack_from("<i", data, ofs)[0]
    for i in range(count):
        gid, flags, gver, fofs, flen = struct.unpack_from("<4sHHii", data, ofs + 4 + i * 16)
        if gid not in (b"prps", b"sprp"):  # 'sprp' stored little endian
            continue
        if flags & 1:
            refuse(f"{path}: the static prop game lump is compressed; plain lumps only")
        s = data[fofs:fofs + flen]
        dicts = struct.unpack_from("<i", s, 0)[0]
        p = 4 + dicts * 128
        nleaf = struct.unpack_from("<i", s, p)[0]
        w.prop_leaves = struct.unpack_from(f"<{nleaf}H", s, p + 4)
        p += 4 + nleaf * 2
        nprops = struct.unpack_from("<i", s, p)[0]
        p += 4
        rest = len(s) - p
        if nprops:
            if rest % nprops or rest // nprops < 30:
                refuse(f"{path}: static prop lump version {gver}: {rest} bytes for {nprops} "
                       f"props is not a whole struct size")
            stride = rest // nprops
            w.prop_stride, w.prop_version = stride, gver
            if stride not in KNOWN_PROP_SIZES:
                print(f"warning: static prop struct of {stride} bytes (version {gver}) is not "
                      f"one of {KNOWN_PROP_SIZES}; reading FirstLeaf/LeafCount at 26/28 anyway",
                      file=sys.stderr)
            for k in range(nprops):
                w.props.append(struct.unpack_from("<HH", s, p + k * stride + 26))
        return


def face_polygon_box(w, f):
    firstedge, numedges = w.faces[f][3], w.faces[f][4]
    pts = []
    for se in w.surfedges[firstedge:firstedge + numedges]:
        pts.append(w.verts[w.edges[se][0] if se >= 0 else w.edges[-se][1]])
    return pts


def box_leaves(w, lo, hi):
    """Every leaf the box touches, by cutting it down the world model's node tree."""
    out = []
    stack = [w.headnode]
    while stack:
        n = stack.pop()
        if n < 0:
            out.append(-1 - n)
            continue
        plane, front, back = w.nodes[n]
        nx, ny, nz, dist = w.planes[plane]
        dmax = (nx * (hi[0] if nx >= 0 else lo[0]) + ny * (hi[1] if ny >= 0 else lo[1])
                + nz * (hi[2] if nz >= 0 else lo[2]))
        dmin = (nx * (lo[0] if nx >= 0 else hi[0]) + ny * (lo[1] if ny >= 0 else hi[1])
                + nz * (lo[2] if nz >= 0 else hi[2]))
        if dmax >= dist:  # the engine's BoxOnPlaneSide: front when the near corner reaches the plane
            stack.append(front)
        if dmin < dist:
            stack.append(back)
    return out


# ---------------------------------------------------------------------------
# per-cluster contents


class Group:
    """One kind of drawable. item -> clusters it sits in; drawn when any of them is in the PVS.

    An item in a single cluster is summed per cluster; one spanning several clusters
    is kept by id so a view counts it once. cl_items is the plain per-cluster set,
    which the brute-force check uses and the fast path does not."""

    def __init__(self, ncl, item_clusters, weights, ncols):
        self.ncols = ncols
        self.weights = weights
        self.cl_items = [set() for _ in range(ncl)]
        self.single = [[0] * ncl for _ in range(ncols)]
        self.by_cluster = [[] for _ in range(ncl)]
        self.multi_w = [{} for _ in range(ncols)]
        for it, cs in item_clusters.items():
            for c in cs:
                self.cl_items[c].add(it)
            if len(cs) == 1:
                (c,) = cs
                for k in range(ncols):
                    self.single[k][c] += weights[it][k]
            else:
                for k in range(ncols):
                    self.multi_w[k][it] = weights[it][k]
                for c in cs:
                    self.by_cluster[c].append(it)
        self.has_multi = bool(self.multi_w[0])
        self.n_items = len(item_clusters)
        self.n_multi = len(self.multi_w[0])

    def single_sums(self, idx):
        return [sum(map(col.__getitem__, idx)) for col in self.single]

    def multi_set(self, idx):
        return set().union(*map(self.by_cluster.__getitem__, idx)) if self.has_multi else set()

    def multi_sums(self, u):
        return [sum(map(mw.__getitem__, u)) for mw in self.multi_w]


def bits(row, n):
    """The set bits of a PVS row as cluster numbers."""
    return list(compress(range(n), f"{row:b}".encode()[::-1].translate(_TO01)))


def build_groups(w, ncl):
    notes = {"leaves_solid_with_cluster": 0, "leaves_no_cluster": 0,
             "faces_listed": 0, "faces_disp": 0, "faces_excluded": 0,
             "excluded_by": {k: 0 for k, _ in EXCLUDED_FLAGS}, "faces_bad_texinfo": 0,
             "disp_in_no_cluster": 0, "props_in_no_cluster": 0, "props_no_leaves": 0}
    open_leaf = []
    for i, (contents, cluster, first, num) in enumerate(w.leafs):
        if cluster >= ncl:
            refuse(f"leaf {i} is in cluster {cluster}, but the visibility lump has {ncl} clusters")
        if first + num > len(w.leaffaces):
            refuse(f"leaf {i}: leaf-face range {first}+{num} runs past the {len(w.leaffaces)} "
                   f"entries of the leaf-face lump")
        if cluster >= 0:
            open_leaf.append(i)
            if contents & 1:
                notes["leaves_solid_with_cluster"] += 1
        else:
            notes["leaves_no_cluster"] += 1

    # leaves
    leaf_items = {i: frozenset((w.leafs[i][1],)) for i in open_leaf}
    leaf_w = {i: (1,) for i in open_leaf}

    # faces
    face_clusters = {}
    unique_listed = set()
    seen_class = {}
    for i in open_leaf:
        _, cluster, first, num = w.leafs[i]
        for f in w.leaffaces[first:first + num]:
            if f >= len(w.faces):
                refuse(f"leaf {i} lists face {f}, but the face lump has {len(w.faces)}")
            if f not in seen_class:
                unique_listed.add(f)
                _, _, _, _, numedges, texinfo, dispinfo = w.faces[f]
                if dispinfo != -1:
                    seen_class[f] = "disp"
                    notes["faces_disp"] += 1
                else:
                    if 0 <= texinfo < len(w.texflags):
                        flags = w.texflags[texinfo]
                    else:
                        flags = 0
                        notes["faces_bad_texinfo"] += 1
                    hit = [k for k, bit in EXCLUDED_FLAGS if flags & bit]
                    if hit:
                        seen_class[f] = "excluded"
                        notes["faces_excluded"] += 1
                        for k in hit:
                            notes["excluded_by"][k] += 1
                    else:
                        seen_class[f] = "drawn"
            if seen_class[f] == "drawn":
                face_clusters.setdefault(f, set()).add(cluster)
    notes["faces_listed"] = len(unique_listed)
    face_w = {f: (1, max(0, w.faces[f][4] - 2)) for f in face_clusters}

    # displacements
    disp_clusters, disp_w = {}, {}
    for d, (vstart, power, mapface) in enumerate(w.dispinfo):
        if mapface >= len(w.faces) or w.faces[mapface][6] != d:
            refuse(f"displacement {d}: base face {mapface} does not point back at it")
        nv = ((1 << power) + 1) ** 2
        dv = w.dispverts[vstart:vstart + nv]
        if len(dv) != nv:
            refuse(f"displacement {d}: vertices {vstart}+{nv} run past the disp vertex lump")
        pts = face_polygon_box(w, mapface)
        # a vertex sits at its base grid point (somewhere in the base face's box) plus
        # vec*dist, so every vertex is in the base box shifted by its own offset: the
        # box widened by the smallest and largest offset on each axis holds them all,
        # without having to work out which grid point is which corner
        lo = [min(p[k] for p in pts) + min(v[k] * v[3] for v in dv) for k in range(3)]
        hi = [max(p[k] for p in pts) + max(v[k] * v[3] for v in dv) for k in range(3)]
        cs = {w.leafs[l][1] for l in box_leaves(w, lo, hi) if w.leafs[l][1] >= 0}
        if cs:
            disp_clusters[d] = cs
            disp_w[d] = (1, 2 * (1 << power) ** 2)
        else:
            notes["disp_in_no_cluster"] += 1

    # static props
    prop_clusters = {}
    for p, (first, count) in enumerate(w.props):
        if count == 0:
            notes["props_no_leaves"] += 1
        cs = {w.leafs[l][1] for l in w.prop_leaves[first:first + count]
              if l < len(w.leafs) and w.leafs[l][1] >= 0}
        if cs:
            prop_clusters[p] = cs
        else:
            notes["props_in_no_cluster"] += 1
    prop_w = {p: (1,) for p in prop_clusters}

    groups = {"leaves": Group(ncl, leaf_items, leaf_w, 1),
              "faces": Group(ncl, {f: frozenset(c) for f, c in face_clusters.items()}, face_w, 2),
              "disps": Group(ncl, disp_clusters, disp_w, 2),
              "props": Group(ncl, prop_clusters, prop_w, 1)}
    notes["open_leaves"] = len(open_leaf)
    return groups, notes, unique_listed, open_leaf


# ---------------------------------------------------------------------------
# the measurement


def measure(groups, ncl, rows_f, rows_c):
    names = [m[0] for m in METRICS]
    res = {k: {m: [0] * ncl for m in names} for k in ("full", "culled", "extra", "culled_only")}
    for a in range(ncl):
        rf, rc = rows_f[a], rows_c[a]
        idx_f = bits(rf, ncl)
        idx_c = bits(rc, ncl) if rc != rf else idx_f
        idx_i = bits(rf & rc, ncl) if rc != rf else idx_f
        for gname, g in groups.items():
            uf = g.multi_set(idx_f)
            full = [x + y for x, y in zip(g.single_sums(idx_f), g.multi_sums(uf))]
            if rc == rf:
                cul, inter = full, full
            else:
                uc = g.multi_set(idx_c)
                cul = [x + y for x, y in zip(g.single_sums(idx_c), g.multi_sums(uc))]
                inter = [x + y for x, y in zip(g.single_sums(idx_i), g.multi_sums(uf & uc))]
            for name, mg, col in METRICS:
                if mg == gname:
                    res["full"][name][a] = full[col]
                    res["culled"][name][a] = cul[col]
                    res["extra"][name][a] = full[col] - inter[col]
                    res["culled_only"][name][a] = cul[col] - inter[col]
    return res


def brute(groups, a, rf, rc, ncl):
    """The same numbers for one viewer by plain set unions (no single/multi split)."""
    idx_f, idx_c = bits(rf, ncl), bits(rc, ncl)
    out = {}
    for name, gname, col in METRICS:
        g = groups[gname]
        uf = set().union(*(g.cl_items[c] for c in idx_f))
        uc = set().union(*(g.cl_items[c] for c in idx_c))
        wsum = lambda s: sum(g.weights[it][col] for it in s)  # noqa: E731
        out[name] = (wsum(uf), wsum(uc), wsum(uf - uc))
    return out


def unordered_pairs(rows, n):
    """Cluster pairs i < j visible from i, from j, or both."""
    mat = bytearray(n * n)
    for i, r in enumerate(rows):
        mat[i * n:(i + 1) * n] = f"{r:0{n}b}".encode()[::-1]  # ASCII 0/1, bit j at column j
    total = 0
    for j in range(n):
        col = int(bytes(mat[j::n])[::-1], 2)  # the bit i is set when row i sees j
        total += ((rows[j] | col) >> (j + 1)).bit_count()
    return total


def stats(full, extra, idx):
    """Means over the clusters in idx, and the worst cluster of idx by extra and by extra share."""
    n = len(idx)
    if not n:
        return None
    mf = sum(full[a] for a in idx) / n
    me = sum(extra[a] for a in idx) / n
    worst = max(idx, key=lambda a: (extra[a], -a))
    share = [(extra[a] / full[a], a) for a in idx if full[a] > 0]
    top = max(share, key=lambda t: (t[0], -t[1])) if share else (0.0, idx[0])
    return {"mean_full": mf, "mean_extra": me,
            "extra_pct_of_mean": 100.0 * me / mf if mf else 0.0,
            "max_extra": {"value": extra[worst], "cluster": worst, "full_there": full[worst]},
            "max_extra_pct": {"value": 100.0 * top[0], "cluster": top[1],
                              "extra": extra[top[1]], "full_there": full[top[1]]}}


# ---------------------------------------------------------------------------


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("full", help="the BSP with the larger PVS")
    ap.add_argument("culled", help="the same map's BSP with the smaller PVS")
    ap.add_argument("--json", help="write the same numbers, and the per-cluster values, here")
    args = ap.parse_args()

    for path in (args.full, args.culled):
        if not os.path.isfile(path):
            refuse(f"{path}: no such file")
    data_f, lump_f = read_bsp(args.full)
    data_c, _ = read_bsp(args.culled)
    diff, derived = differing_lumps(data_f, data_c, args.full, args.culled)
    if diff:
        refuse("the two BSPs differ in lumps other than the visibility lump: "
               + ", ".join(f"{i} ({LUMP_NAMES.get(i, '?')})" for i in diff)
               + "; they must be vvis runs of the same map")
    vis_f, vis_c = vis_lump(args.full), vis_lump(args.culled)
    if len(vis_f) < 4 or len(vis_c) < 4:
        refuse("a BSP has no visibility lump (not vvis'd)")
    ncl = struct.unpack_from("<I", vis_f, 0)[0]
    if struct.unpack_from("<I", vis_c, 0)[0] != ncl:
        refuse(f"cluster counts differ: {ncl} vs {struct.unpack_from('<I', vis_c, 0)[0]}")
    rows_f, rows_c = pvs_rows(vis_f), pvs_rows(vis_c)

    w = read_world(args.full, data_f, lump_f)
    groups, notes, listed, open_leaf = build_groups(w, ncl)
    gl, gf, gd, gp = groups["leaves"], groups["faces"], groups["disps"], groups["props"]

    # the per-cluster face sets must cover every counted face the open leaves list
    covered = set().union(*gf.cl_items)
    counted = notes["faces_listed"] - notes["faces_disp"] - notes["faces_excluded"]
    if len(covered) != counted or not covered <= listed:
        print(f"pvs_render_cost: the per-cluster face sets hold {len(covered)} faces, the leaf-face "
              f"lists of open leaves name {counted} countable ones", file=sys.stderr)
        return 3

    # visible pairs
    lost = pairs_only_in(rows_f, rows_c)
    gained = pairs_only_in(rows_c, rows_f)
    differing = [a for a in range(ncl) if rows_f[a] != rows_c[a]]
    not_subset = [a for a in range(ncl) if rows_c[a] & ~rows_f[a]]
    pairs_full, pairs_culled = unordered_pairs(rows_f, ncl), unordered_pairs(rows_c, ncl)

    res = measure(groups, ncl, rows_f, rows_c)

    # brute-force recount on a sample: the ends, an even spread, and the worst clusters by extra
    sample = {0, ncl - 1} | {i * (ncl - 1) // 7 for i in range(8)}
    for name, _, _ in METRICS:
        sample |= set(sorted(differing, key=lambda a: -res["extra"][name][a])[:2])
    for a in sorted(sample):
        b = brute(groups, a, rows_f[a], rows_c[a], ncl)
        for name, _, _ in METRICS:
            fast = (res["full"][name][a], res["culled"][name][a], res["extra"][name][a])
            if fast != b[name]:
                print(f"pvs_render_cost: cluster {a}, {name}: fast path (full, culled, extra) "
                      f"{fast} but brute-force set unions give {b[name]}", file=sys.stderr)
                return 3

    # --- report
    all_idx = list(range(ncl))
    out = {"full": args.full, "culled": args.culled, "clusters": ncl}
    pr = print
    pr(f"{args.full}  vs  {args.culled}")
    for note in derived:
        pr(f"  note: {note}")
    pr(f"  map: {ncl} clusters, {len(w.leafs)} leaves ({notes['open_leaves']} open with a cluster, "
       f"{notes['leaves_no_cluster']} without; {notes['leaves_solid_with_cluster']} of the open "
       f"ones carry CONTENTS_SOLID), leaf lump version {w.leaf_version}")
    ex = notes["excluded_by"]
    pr(f"  faces named by open leaves' leaf-face lists: {notes['faces_listed']} unique; counted "
       f"{counted} ({sum(w.faces[f][4] - 2 for f in covered):,} triangles); not counted "
       f"{notes['faces_disp']} displacement faces and {notes['faces_excluded']} with texinfo flags "
       f"(nodraw {ex['nodraw']}, sky {ex['sky']}, sky2d {ex['sky2d']}, hint {ex['hint']}, "
       f"skip {ex['skip']})"
       + (f"; {notes['faces_bad_texinfo']} with texinfo out of range" if notes['faces_bad_texinfo'] else ""))
    pr(f"  per-cluster face sets cover those {counted} faces: ok; "
       f"{gf.n_multi} of them are in more than one cluster")
    pr(f"  displacements: {len(w.dispinfo)} ({notes['disp_in_no_cluster']} touch no open leaf), "
       f"{gd.n_multi} in more than one cluster")
    pr(f"  static props: {len(w.props)} (struct {w.prop_stride} bytes, version {w.prop_version}; "
       f"{notes['props_in_no_cluster']} in no open cluster, of which {notes['props_no_leaves']} "
       f"list no leaf), {gp.n_multi} in more than one cluster")
    own = []
    for label, g in (("leaves", gl), ("faces", gf), ("displacements", gd), ("static props", gp)):
        sizes = [len(s) for s in g.cl_items]
        own.append(f"{label} {sum(sizes) / ncl:.1f} ({max(sizes)})")
    pr("  one cluster holds, mean (max): " + ", ".join(own))
    pr("  no frustum or distance culling (there is no viewpoint): every item in a viewer's PVS counts,")
    pr("  so these are upper bounds on what a real view draws; brush entities are not included")
    pr(f"  PVS: {ncl} clusters, {len(differing)} differ; visible cluster pairs (either direction, "
       f"i<j): full {pairs_full:,}, culled {pairs_culled:,}; lost {len(lost):,}"
       + (f" ({100.0 * len(lost) / pairs_full:.2f}% of full)" if pairs_full else "")
       + f", gained {len(gained):,}")
    isolated = [a for a in differing if rows_c[a].bit_count() <= 1 < rows_f[a].bit_count()]
    halved = [a for a in differing if rows_c[a].bit_count() * 2 < rows_f[a].bit_count()]
    shown = ", ".join(map(str, isolated[:10])) + (" ..." if len(isolated) > 10 else "")
    pr(f"  clusters the culled PVS leaves seeing only themselves: {len(isolated)}"
       + (f" ({shown})" if isolated else "")
       + f"; clusters that lose more than half their PVS: {len(halved)}")
    out["map"] = {"leaves": len(w.leafs), "open_leaves": notes["open_leaves"],
                  "faces_listed": notes["faces_listed"], "faces_counted": counted,
                  "faces_displacement": notes["faces_disp"],
                  "faces_excluded": notes["faces_excluded"], "excluded_by": ex,
                  "displacements": len(w.dispinfo), "static_props": len(w.props),
                  "prop_struct_bytes": w.prop_stride, "prop_version": w.prop_version}
    out["pairs"] = {"full": pairs_full, "culled": pairs_culled, "lost": len(lost),
                    "gained": len(gained), "clusters_differing": len(differing),
                    "clusters_not_subset": len(not_subset), "clusters_isolated": isolated,
                    "clusters_losing_over_half": len(halved)}
    if gained or not_subset:
        only = {m: sum(res["culled_only"][m]) for m, _, _ in METRICS}
        pr(f"  WARNING: the culled PVS is not a subset of the full one: {len(gained)} pairs only the "
           f"culled map sees, in {len(not_subset)} clusters' rows; extras below are |full minus "
           f"culled|, and the items only the culled map draws, summed over clusters, are "
           + ", ".join(f"{m} {v:,}" for m, v in only.items()))
        out["culled_only_items_summed"] = only

    def table(idx, title, with_max):
        pr(f"  {title}")
        head = f"    {'metric':<15} {'mean full':>12} {'mean extra':>11} {'extra %':>8}"
        if with_max:
            head += f"   {'max extra':>10} {'cluster':>7}   {'max extra %':>11} {'cluster':>7}"
        pr(head)
        rec = {}
        for name, _, _ in METRICS:
            s = stats(res["full"][name], res["extra"][name], idx)
            if s is None:
                pr(f"    {name:<15} (no clusters)")
                continue
            rec[name] = s
            line = (f"    {name:<15} {s['mean_full']:>12,.2f} {s['mean_extra']:>11,.2f} "
                    f"{s['extra_pct_of_mean']:>7.3f}%")
            if with_max:  # a cluster is named only when its extra is not zero
                c1 = s['max_extra']['cluster'] if s['max_extra']['value'] else "-"
                c2 = s['max_extra_pct']['cluster'] if s['max_extra_pct']['value'] else "-"
                line += (f"   {s['max_extra']['value']:>10,} {c1:>7}"
                         f"   {s['max_extra_pct']['value']:>10.2f}% {c2:>7}")
            pr(line)
        return rec

    out["metrics"] = {"all": table(all_idx, f"over all {ncl} clusters:", True)}
    if differing:
        out["metrics"]["differing"] = table(
            differing, f"over only the {len(differing)} clusters whose PVS differs:", False)
    else:
        pr("  no cluster's PVS differs: every extra is 0")
        out["metrics"]["differing"] = {}
    pr(f"  self-check: fast path equals plain set unions on {len(sample)} viewers "
       f"(all six metrics, full, culled and extra)")

    if args.json:
        out["per_cluster"] = {k: v for k, v in res.items()}
        with open(args.json, "w") as f:
            json.dump(out, f)
    return 0


if __name__ == "__main__":
    sys.exit(main())
