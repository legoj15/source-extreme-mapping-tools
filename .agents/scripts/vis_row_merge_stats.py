#!/usr/bin/env python3
r"""What would merging BSP clusters with identical (or similar) PVS rows save, and cost?

A measurement for issue #100, before any VVIS code is written. It reads a
compiled, vis'd BSP, models a post-vis pass that merges clusters whose PVS rows
match, and prints the visibility lump size and the visible-pair counts before
and after, for several tolerances. It never writes anything.

Usage:
  python -I .agents/scripts/vis_row_merge_stats.py <bsp> [--prt <prt>] \
      [--tol 0,0.005,0.01,0.02,0.05,0.1] [--adjacent-only]

  --prt            the map's portal file (PRT1); read for the portal graph
  --tol            comma list; t = 0 is always run first. A cluster joins the
                   first existing group whose current union row differs from
                   the cluster's row in at most t * popcount(cluster row) bits
                   and whose leader (first member) the cluster can see; a
                   group's row is the OR of its members' rows
  --adjacent-only  (needs --prt) a cluster may only join a group that holds a
                   cluster it shares a portal with

Size model (checked against the real lump before any number is trusted): the
lump is dvis_t { int numclusters; int bitofs[n][2] } (4 + 8n bytes), then every
PVS row in cluster order, then every PAS row in cluster order, no padding. A
row is (n+7)>>3 bytes compressed with CompressVis (a zero byte is followed by a
repeat count, at most 255 per pair, trailing zeros treated like any others).
The PAS row of cluster c is the OR of the PVS rows of every cluster in c's PVS
(CalcPAS), taken from the PVS after vvis's symmetric crosscheck.

Merged matrix: M'[g,h] = any M[a,b] for a in g, b in h. Leaf-weighted pairs =
sum over clusters c of leaves(c) * (sum of leaves(d) for d visible from c),
the figure that compares maps with different cluster counts. The "after" size
keeps the groups in order of their first member and does not otherwise
renumber, so it is what a straightforward implementation would write.

Exit code 0 when the size self-check and the t = 0 lossless checks hold, 1
otherwise (a failed check is printed as a finding, the run continues).
"""
import argparse
import struct
import sys
import time

import numpy as np

LUMP_VISIBILITY = 4
LUMP_LEAFS = 10
# dleaf_t (version 1) is 32 bytes; version 0 adds a 24-byte CompressedLightCube
# and is padded to 4: 56 bytes. contents is at 0, the int16 cluster at 4.
LEAF_SIZES = {0: 56, 1: 32}

_POP8 = np.array([bin(i).count("1") for i in range(256)], dtype=np.uint8)


# ---------------------------------------------------------------------------
# file formats
# ---------------------------------------------------------------------------

def read_bsp(path):
    with open(path, "rb") as f:
        data = f.read()
    if data[:4] != b"VBSP":
        sys.exit(f"{path}: not a BSP")
    lumps = [struct.unpack_from("<iiii", data, 8 + i * 16) for i in range(64)]
    return data, lumps


def lump_bytes(data, lumps, index):
    ofs, length, version, fourcc = lumps[index]
    if fourcc:
        sys.exit(f"lump {index} is compressed (uncompressed size {fourcc}); "
                 "decompress the BSP first")
    if ofs < 0 or length < 0 or ofs + length > len(data):
        sys.exit(f"lump {index} lies outside the file")
    return data[ofs:ofs + length], version


def leaf_clusters(data, lumps):
    """The cluster number of every leaf (-1 = none)."""
    raw, version = lump_bytes(data, lumps, LUMP_LEAFS)
    if version not in LEAF_SIZES:
        sys.exit(f"unknown LUMP_LEAFS version {version}")
    size = LEAF_SIZES[version]
    if len(raw) % size:
        sys.exit(f"LUMP_LEAFS length {len(raw)} is not a multiple of the "
                 f"version {version} leaf size {size}")
    arr = np.frombuffer(raw, dtype=np.uint8).reshape(-1, size)
    return arr[:, 4:6].copy().view("<i2").ravel().astype(np.int64), version, size


def read_prt_graph(path, n):
    """Per-cluster sorted arrays of the clusters sharing a portal; edge count."""
    with open(path, "r") as f:
        lines = [ln for ln in f.read().splitlines() if ln.strip()]
    if len(lines) < 3 or lines[0].strip() != "PRT1":
        sys.exit(f"{path}: not a PRT1 portal file")
    ncl, nportals = int(lines[1]), int(lines[2])
    if ncl != n:
        sys.exit(f"{path}: {ncl} clusters but the BSP visibility lump has {n}")
    if len(lines) - 3 != nportals:
        sys.exit(f"{path}: header says {nportals} portals, {len(lines) - 3} lines")
    sets = [set() for _ in range(n)]
    for ln in lines[3:]:
        parts = ln.split(None, 3)
        a, b = int(parts[1]), int(parts[2])
        if not (0 <= a < n and 0 <= b < n):
            sys.exit(f"{path}: portal between clusters {a} and {b}, out of range")
        if a != b:
            sets[a].add(b)
            sets[b].add(a)
    nbrs = [np.array(sorted(s), dtype=np.int64) for s in sets]
    return nbrs, sum(len(s) for s in sets) // 2, nportals


# ---------------------------------------------------------------------------
# CompressVis / DecompressVis, mirrored from src/utils/common/bsplib.cpp
# ---------------------------------------------------------------------------

def compress_vis(vis):
    out = bytearray()
    j = 0
    n = len(vis)
    while j < n:
        out.append(vis[j])
        if vis[j]:
            j += 1
            continue
        rep = 1
        j += 1
        while j < n:
            if vis[j] or rep == 255:
                break
            rep += 1
            j += 1
        out.append(rep)
    return bytes(out)


def decompress_vis(lump, ofs, row_bytes):
    """Returns (row, offset after the last compressed byte read)."""
    out = bytearray(row_bytes)
    o = 0
    size = len(lump)
    try:
        while True:
            v = lump[ofs]
            if v:
                out[o] = v
                o += 1
                ofs += 1
            else:
                c = lump[ofs + 1]
                if not c:
                    raise ValueError("0 repeat")
                ofs += 2
                o += min(c, row_bytes - o)
            if o >= row_bytes:
                break
    except IndexError:
        raise ValueError(f"compressed row runs past the lump end ({size})")
    return bytes(out), ofs


def compressed_sizes(bits):
    """Vectorised len(compress_vis(row)) for every row of a bool matrix."""
    g = bits.shape[0]
    packed = np.packbits(bits, axis=1, bitorder="little")
    rb = packed.shape[1]
    zero = np.zeros((g, rb + 1), dtype=bool)  # a false column ends each row's runs
    zero[:, :rb] = packed == 0
    flat = zero.ravel()
    prev = np.concatenate(([False], flat[:-1]))
    nxt = np.concatenate((flat[1:], [False]))
    starts = np.flatnonzero(flat & ~prev)
    ends = np.flatnonzero(flat & ~nxt)
    cost = 2 * ((ends - starts + 1 + 254) // 255)
    run_cost = np.bincount(starts // (rb + 1), weights=cost, minlength=g)
    nonzero = rb - zero[:, :rb].sum(axis=1)
    return nonzero + run_cost.astype(np.int64)


def pas_matrix(m):
    f = m.astype(np.float32)
    return (f @ f) > 0


def lump_size(pvs):
    """Modelled visibility lump length for a PVS matrix (PAS derived from it)."""
    n = pvs.shape[0]
    pas = pas_matrix(pvs)
    return 4 + 8 * n + int(compressed_sizes(pvs).sum()) + int(compressed_sizes(pas).sum())


# ---------------------------------------------------------------------------
# merging
# ---------------------------------------------------------------------------

def popcount_rows(a):
    """Set bits per row of a 2-D uint64 array."""
    if hasattr(np, "bitwise_count"):
        return np.bitwise_count(a).sum(axis=1, dtype=np.int64)
    return _POP8[np.ascontiguousarray(a).view(np.uint8)].reshape(a.shape[0], -1).sum(
        axis=1, dtype=np.int64)


def exact_groups(m):
    """Group ids (by first member) for clusters with byte-identical rows."""
    packed = np.packbits(m, axis=1, bitorder="little")
    seen = {}
    gid = np.empty(m.shape[0], dtype=np.int64)
    for c in range(m.shape[0]):
        gid[c] = seen.setdefault(packed[c].tobytes(), len(seen))
    return gid


def leader_groups(m, tol, nbrs=None):
    """Leader clustering as described in the module docstring."""
    n = m.shape[0]
    packed = np.packbits(m, axis=1, bitorder="little")
    words = (packed.shape[1] + 7) // 8
    buf = np.zeros((n, words * 8), dtype=np.uint8)
    buf[:, :packed.shape[1]] = packed
    rows = buf.view(np.uint64)
    pc = m.sum(axis=1)
    union = np.zeros((n, words), dtype=np.uint64)
    leaders = np.zeros(n, dtype=np.int64)
    gid = np.full(n, -1, dtype=np.int64)
    ng = 0
    for c in range(n):
        cand = np.flatnonzero(m[c, leaders[:ng]])
        if nbrs is not None and cand.size:
            gs = gid[nbrs[c]]
            cand = np.intersect1d(cand, np.unique(gs[gs >= 0]), assume_unique=True)
        g = -1
        if cand.size:
            d = popcount_rows(union[cand] ^ rows[c])
            ok = np.flatnonzero(d <= tol * pc[c])
            if ok.size:
                g = int(cand[ok[0]])
        if g < 0:
            g = ng
            leaders[ng] = c
            union[ng] = rows[c]
            ng += 1
        else:
            union[g] |= rows[c]
        gid[c] = g
    return gid


def merge_matrix(m, gid):
    """M'[g,h] = any M[a,b], a in g, b in h (collapse rows, then columns)."""
    order = np.argsort(gid, kind="stable")
    starts = np.searchsorted(gid[order], np.arange(int(gid.max()) + 1))
    rows = np.logical_or.reduceat(m[order], starts, axis=0)
    return np.logical_or.reduceat(rows[:, order], starts, axis=1)


def weighted_pairs(m, leaves):
    """sum over c of leaves[c] * (sum of leaves[d] for d with m[c,d])."""
    lf = leaves.astype(np.float64)
    total = 0
    for i in range(0, m.shape[0], 512):
        part = m[i:i + 512].astype(np.float64) @ lf
        total += int(np.rint(part).astype(np.int64) @ leaves[i:i + 512])
    return total


# ---------------------------------------------------------------------------
# report
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bsp")
    ap.add_argument("--prt")
    ap.add_argument("--tol", default="0,0.005,0.01,0.02,0.05,0.1")
    ap.add_argument("--adjacent-only", action="store_true")
    args = ap.parse_args()
    if args.adjacent_only and not args.prt:
        ap.error("--adjacent-only needs --prt")
    tols = sorted({0.0} | {float(t) for t in args.tol.split(",") if t.strip()})

    t_start = time.time()
    failures = []
    data, lumps = read_bsp(args.bsp)
    lump, vis_version = lump_bytes(data, lumps, LUMP_VISIBILITY)
    if len(lump) < 4:
        sys.exit("no visibility data in this BSP (run vvis first)")
    n = struct.unpack_from("<i", lump, 0)[0]
    rb = (n + 7) >> 3
    if n <= 0 or len(lump) < 4 + 8 * n:
        sys.exit(f"visibility lump of {len(lump)} bytes cannot hold {n} clusters")
    bitofs = np.frombuffer(lump, dtype="<i4", count=2 * n, offset=4).reshape(n, 2)

    # decode the stored PVS and PAS rows
    spans = [[None] * n, [None] * n]
    decoded = [[None] * n, [None] * n]
    for kind in (0, 1):
        for c in range(n):
            ofs = int(bitofs[c, kind])
            row, end = decompress_vis(lump, ofs, rb)
            decoded[kind][c] = row
            spans[kind][c] = (ofs, end)
    pvs_bits = np.unpackbits(np.frombuffer(b"".join(decoded[0]), np.uint8).reshape(n, rb),
                             axis=1, bitorder="little")
    pas_bits = np.unpackbits(np.frombuffer(b"".join(decoded[1]), np.uint8).reshape(n, rb),
                             axis=1, bitorder="little")
    if pvs_bits[:, n:].any() or pas_bits[:, n:].any():
        failures.append("pad bits beyond the last cluster are set in a stored row")
    m = pvs_bits[:, :n].astype(bool)
    stored_pas = pas_bits[:, :n].astype(bool)

    clusters_of_leaves, leaf_version, leaf_size = leaf_clusters(data, lumps)
    if clusters_of_leaves.size and clusters_of_leaves.max() >= n:
        sys.exit(f"a leaf names cluster {clusters_of_leaves.max()}, but the visibility "
                 f"lump has {n} clusters")
    leaves = np.bincount(clusters_of_leaves[clusters_of_leaves >= 0],
                         minlength=n).astype(np.int64)

    nbrs = None
    prt_note = ""
    if args.prt:
        nbrs, edges, nportals = read_prt_graph(args.prt, n)
        prt_note = f"{nportals} portals, {edges} cluster pairs share a portal"

    print(f"bsp: {args.bsp}")
    print(f"  visibility lump: offset {lumps[LUMP_VISIBILITY][0]}, "
          f"{len(lump)} bytes, version {vis_version}")
    print(f"  leaf lump: version {leaf_version} ({leaf_size} bytes each), "
          f"{clusters_of_leaves.size} leaves, "
          f"{int((clusters_of_leaves < 0).sum())} with cluster -1")
    print(f"  clusters: {n}; leaves per cluster min {int(leaves.min())} "
          f"median {int(np.median(leaves))} max {int(leaves.max())}; "
          f"{int((leaves == 0).sum())} clusters hold no leaf")
    if prt_note:
        print(f"  portal file: {prt_note}")
    symmetric = bool((m == m.T).all())
    print(f"  PVS symmetric: {'yes' if symmetric else 'NO (merged matrices are not guaranteed symmetric)'}")
    if not symmetric:
        failures.append("the PVS matrix is not symmetric")
    print(f"  self bit set on every row: {'yes' if m.diagonal().all() else 'NO'}")

    # ---- size model self-check
    print()
    print("size model self-check")
    pas = pas_matrix(m)
    pvs_packed = np.packbits(m, axis=1, bitorder="little")
    pas_packed = np.packbits(pas, axis=1, bitorder="little")
    pvs_comp = [compress_vis(pvs_packed[c].tobytes()) for c in range(n)]
    pas_comp = [compress_vis(pas_packed[c].tobytes()) for c in range(n)]
    pvs_len = np.array([len(x) for x in pvs_comp], dtype=np.int64)
    pas_len = np.array([len(x) for x in pas_comp], dtype=np.int64)
    header = 4 + 8 * n
    modelled = header + int(pvs_len.sum()) + int(pas_len.sum())
    residual = len(lump) - modelled
    pvs_same = sum(pvs_comp[c] == lump[spans[0][c][0]:spans[0][c][1]] for c in range(n))
    pas_same = sum(pas_comp[c] == lump[spans[1][c][0]:spans[1][c][1]] for c in range(n))
    pas_equal = int((pas == stored_pas).all(axis=1).sum())
    pvs_ofs = header + np.concatenate(([0], np.cumsum(pvs_len)[:-1]))
    pas_ofs = header + int(pvs_len.sum()) + np.concatenate(([0], np.cumsum(pas_len)[:-1]))
    layout_ok = bool((bitofs[:, 0] == pvs_ofs).all() and (bitofs[:, 1] == pas_ofs).all())
    fast_ok = bool((compressed_sizes(m) == pvs_len).all()
                   and (compressed_sizes(pas) == pas_len).all())
    print(f"  modelled lump bytes {modelled} = header {header} + PVS {int(pvs_len.sum())} "
          f"+ PAS {int(pas_len.sum())}")
    print(f"  real lump bytes     {len(lump)}   residual {residual}")
    print(f"  PVS rows byte-identical to the recompression: {pvs_same}/{n}")
    print(f"  PAS rows byte-identical to the recompression of M@M: {pas_same}/{n} "
          f"(decoded PAS equals M@M on {pas_equal}/{n} rows)")
    print(f"  bitofs offsets equal the modelled layout: {'yes' if layout_ok else 'NO'}")
    print(f"  vectorised row sizes equal the literal CompressVis mirror: "
          f"{'yes' if fast_ok else 'NO'}")
    if residual != 0:
        failures.append(f"modelled lump size differs from the real one by {residual} bytes")
    if pvs_same != n or pas_same != n or pas_equal != n or not layout_ok or not fast_ok:
        failures.append("the size model does not reproduce every stored row")
    base_lump = modelled

    # ---- baseline
    base_pairs = int(m.sum())
    base_lw = weighted_pairs(m, leaves)
    print()
    print("baseline")
    print(f"  lump bytes {base_lump}; cluster pairs visible (ordered, self included) "
          f"{base_pairs} ({base_pairs / (n * n) * 100:.2f}% of {n}^2); "
          f"leaf-weighted pairs {base_lw}")

    # ---- tolerances
    mode = "adjacent-only (groups only grow across a shared portal)" if args.adjacent_only \
        else "any cluster that sees the group leader"
    print()
    print(f"merge by tolerance; join rule: {mode}")
    head = (f"{'tol':>7} {'clusters':>9} {'lump bytes':>11} {'lump %':>7} "
            f"{'cluster pairs':>14} {'leaf-wtd pairs':>16} {'leaf-wtd +%':>12} {'max grp':>8}")
    print(head)
    print("-" * len(head))
    print(f"{'base':>7} {n:>9} {base_lump:>11} {100.0:>7.2f} {base_pairs:>14} "
          f"{base_lw:>16} {0.0:>12.2f} {1:>8}")
    zero_gid = None
    for t in tols:
        if t == 0.0 and not args.adjacent_only:
            gid = exact_groups(m)
            if not (leader_groups(m, 0.0) == gid).all():
                failures.append("leader clustering at t = 0 disagrees with exact-row grouping")
                print("  FINDING: leader clustering at t = 0 disagrees with exact-row grouping")
        else:
            gid = leader_groups(m, t, nbrs if args.adjacent_only else None)
        g = int(gid.max()) + 1
        mp = merge_matrix(m, gid)
        gl = np.bincount(gid, weights=leaves, minlength=g).astype(np.int64)
        size = lump_size(mp)
        pairs = int(mp.sum())
        lw = weighted_pairs(mp, gl)
        grp_size = np.bincount(gid, minlength=g)
        print(f"{t:>7g} {g:>9} {size:>11} {size / base_lump * 100:>7.2f} {pairs:>14} "
              f"{lw:>16} {(lw - base_lw) / base_lw * 100:>12.2f} {int(grp_size.max()):>8}")
        sys.stdout.flush()
        if t == 0.0:
            zero_gid = gid
            expanded = mp[np.ix_(gid, gid)]
            if not (expanded == m).all():
                diff = expanded != m
                gained, lost = int((diff & expanded).sum()), int((diff & m).sum())
                failures.append("t = 0 merge is not lossless")
                print(f"  FINDING: t = 0 merge is NOT lossless: {gained} cells gained, "
                      f"{lost} lost after expanding back to the original clusters")
            if not (pas_matrix(mp)[np.ix_(gid, gid)] == pas).all():
                failures.append("t = 0 merged PAS does not expand back to the original PAS")
                print("  FINDING: the t = 0 merged PAS does not expand back to the original PAS")

    # ---- t = 0 groups
    sizes = np.bincount(zero_gid)
    gleaves = np.bincount(zero_gid, weights=leaves, minlength=len(sizes)).astype(np.int64)
    print()
    which = "adjacent-only " if args.adjacent_only else ""
    print(f"t = 0 {which}group sizes (lossless: expanding the merge reproduces the original PVS and PAS)")
    print(f"  {'members':>8} {'groups':>8} {'clusters':>9} {'leaves':>9}")
    for label, lo, hi in (("1", 1, 1), ("2", 2, 2), ("3-5", 3, 5), ("6-20", 6, 20),
                          ("21+", 21, 1 << 30)):
        sel = (sizes >= lo) & (sizes <= hi)
        print(f"  {label:>8} {int(sel.sum()):>8} {int(sizes[sel].sum()):>9} "
              f"{int(gleaves[sel].sum()):>9}")
    print("  largest five groups (members, total leaves, first cluster ids):")
    for gi in sorted(range(len(sizes)), key=lambda i: (-sizes[i], i))[:5]:
        members = np.flatnonzero(zero_gid == gi)
        ids = ",".join(str(x) for x in members[:8]) + (",..." if len(members) > 8 else "")
        print(f"    {int(sizes[gi]):>6} members, {int(gleaves[gi]):>6} leaves  [{ids}]")

    print()
    if failures:
        print("CHECKS FAILED:")
        for f in failures:
            print(f"  - {f}")
    else:
        print("all self-checks held")
    print(f"({time.time() - t_start:.1f} s)")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
