"""Validate the t-junction sewing of a Source BSP: its primitive lumps and the
triangulation the engine draws for every face.

A face with no primitive is drawn as a triangle fan from its first vertex; a
face with primitives is drawn from their triangles.  Every polygon side
(including each sub-edge VBSP cut an edge into around a neighbour's vertex)
must be a side of a triangle that has area.  Where a side belongs only to a
zero-area triangle, the vertex that cut the edge is "unsewn" and the
neighbouring face cracks against it.

Checks per map (each failure makes the map invalid):
  * the lumps are readable: indices into vertices, edges, surfedges and
    texinfos stay inside their lumps (a malformed BSP is reported, not a
    crash).  The faces are LUMP_FACES, or LUMP_FACES_HDR when that is empty;
  * primitives are well-formed (type TRILIST, face-local indices i.e.
    vertCount == 0, whole triangles), their index ranges lie inside the lump,
    and the totals stay under the hard BSP format limits (32768 prims / 65536
    indices - dprimitive_t's firstIndex/indexCount are 16-bit).  Faces may
    share index runs;
  * a face carries one primitive (VBSP emits one per face) and that primitive
    is a sound tiling of the face by itself: n-2 triangles, every index
    addresses a vertex of the face, no triangle folded back over the face
    (a diagonal outside a concave face) and the areas add up;
  * no primitive is an orphan (referenced by no face - VBSP emitted it for a
    face it then never wrote);
  * no fan-drawn face folds back over itself by more than SEW_FLIP_HEIGHT (the
    fan would draw outside the face);
  * no world face has unsewn t-junction vertices, whether fan-drawn or
    carrying a primitive that still leaves a seam.  Three kinds are listed
    apart: "FAN-SEWABLE" faces, where another start vertex would have sewn
    every side (a lossless fix VBSP did not apply - what the old "skip the
    primitive on convex faces" shortcut broke); "needs-prim" faces, which no
    start vertex sews (VBSP dropped their primitive because the format limits
    were full, or could not sew them); and "prim-residual" faces.  Pinched
    windings (a vertex visited twice, left by welding the two edges of a
    sliver tip to one vertex) are checked like any other face: only their two
    spike sides, which enclose nothing, are left out.

Reported, not failures:
  * seams on faces VBSP deliberately leaves as stock drew them: water faces
    (SURF_WARP) and the faces of brush entities (any model but the world).
    A face that only borders a fog volume cannot be told from the lumps, so one
    of those with a seam is reported as a failure like any other world face;
    VBSP keeps such faces at the stock start on purpose, so for a map with
    fog-volume faces against water use --warn-only, or read the FAN-SEWABLE /
    needs-prim lines for faces that sit against water;
  * the depth of the folds in fans and primitives (the worst, and how many are
    deeper than SEW_FOLD_FREE): VBSP prefers a rotation or triangulation
    without a fold when one costs nothing, but accepts a shallow one;
  * slivers: faces no wider than SEW_DEGENERATE_HEIGHT across, whose long sides
    can never have a triangle with area; and zero-area faces.  Both are only
    counted.

Faces that are never drawn (NODRAW) and displacements are skipped.

Usage:
    python bsp_prim_check.py [--all] [--warn-only] <map.bsp> [more.bsp ...]

    --all        list every offending face (default: the first 10 per map)
    --warn-only  print violations but exit 0 (for BSPs built by an older VBSP)

Exits non-zero if any check fails on any map.
"""
from __future__ import annotations

import struct
import sys

LUMP_VERTEXES = 3
LUMP_TEXINFO = 6
LUMP_FACES = 7
LUMP_FACES_HDR = 58
LUMP_EDGES = 12
LUMP_SURFEDGES = 13
LUMP_MODELS = 14
LUMP_PRIMS = 37
LUMP_PRIMINDICES = 39
DFACE_SIZE = 56
TEXINFO_SIZE = 72
MODEL_SIZE = 48
PRIM_SIZE = 10
SURF_WARP = 0x8
SURF_NODRAW = 0x80

MAX_PRIMS = 32768
MAX_PRIM_INDICES = 65536

# VBSP's TestEdge puts a vertex on an edge when it lies within this of it.
OFF_EPSILON = 0.25

# A polygon side is unsewn when the third corner of the triangle that carries
# it lies within this many units of the side's line.  Keep at most half of
# SEW_DEGENERATE_HEIGHT in src/utils/vbsp/faces.cpp so a seam VBSP believes
# it sewed is never reported here.
SEW_DEGENERATE_HEIGHT = 0.01

# Folds in a fan or primitive shallower than this are noise (faces.cpp:
# SEW_FOLD_FREE).
SEW_FOLD_FREE = 0.02

# A vertex is a t-junction vertex (a vertex some neighbour's edge passes
# through) when it lies this close to the line through its two ring
# neighbours.  An inserted vertex is within OFF_EPSILON of its original edge,
# and so are its two ring neighbours (inserted vertices or the edge's own
# corners), so it is within twice that of the line through them.  A vertex whose
# projection falls outside the segment between its neighbours is not on the
# edge between them.  The rule can still miss a vertex that sits within a
# couple of hundredths of a unit of a neighbour (the projection then falls
# just outside), and it flags the odd original corner that happens to be
# straight - which also needs sewing, since a neighbour may have a vertex there.
STRAIGHT_HEIGHT = 2 * OFF_EPSILON + 0.01

# A face no wider than a side counts as sewn across has sides that can never
# have a triangle with area: it is a sliver.
SLIVER_WIDTH = SEW_DEGENERATE_HEIGHT

# A triangle folds back over a polygon side by more than this (matches
# SEW_FLIP_HEIGHT in faces.cpp): it draws outside the face.
SEW_FLIP_HEIGHT = 1.0


class BspFormatError(Exception):
    """The file is not a BSP this tool can read."""


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _len(v):
    return (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) ** 0.5


def line_distance(p, a, b):
    """Distance of p from the line through a and b (0 if a == b)."""
    ab = _sub(b, a)
    n = _len(ab)
    if n < 1e-9:
        return 0.0
    return _len(_cross(ab, _sub(p, a))) / n


class Face:
    __slots__ = ('index', 'firstedge', 'numedges', 'texinfo', 'dispinfo',
                 'firstprim', 'numprims')

    def __init__(self, index, firstedge, numedges, texinfo, dispinfo,
                 firstprim, numprims):
        self.index = index
        self.firstedge = firstedge
        self.numedges = numedges
        self.texinfo = texinfo
        self.dispinfo = dispinfo
        self.firstprim = firstprim
        self.numprims = numprims


class Bsp:
    def __init__(self, path):
        with open(path, 'rb') as fh:
            self.d = fh.read()
        if len(self.d) < 8 + 64 * 16 or self.d[:4] != b'VBSP':
            raise BspFormatError('not a VBSP file')

    def lump(self, i, record=1):
        ofs, ln, _ver, uncompressed = struct.unpack_from(
            '<iiii', self.d, 8 + i * 16)
        if uncompressed:
            raise BspFormatError(f'lump {i} is LZMA-compressed')
        if ofs < 0 or ln < 0 or ofs + ln > len(self.d):
            raise BspFormatError(
                f'lump {i} (offset {ofs}, length {ln}) lies outside the file')
        if ln % record:
            raise BspFormatError(
                f'lump {i} length {ln} is not a multiple of {record}')
        return ofs, ln

    def verts(self):
        ofs, ln = self.lump(LUMP_VERTEXES, 12)
        return [struct.unpack_from('<3f', self.d, ofs + i * 12)
                for i in range(ln // 12)]

    def edges(self):
        ofs, ln = self.lump(LUMP_EDGES, 4)
        return [struct.unpack_from('<HH', self.d, ofs + i * 4)
                for i in range(ln // 4)]

    def surfedges(self):
        ofs, ln = self.lump(LUMP_SURFEDGES, 4)
        return [v[0] for v in struct.iter_unpack('<i', self.d[ofs:ofs + ln])]

    def texinfo_flags(self):
        ofs, ln = self.lump(LUMP_TEXINFO, TEXINFO_SIZE)
        return [struct.unpack_from('<i', self.d, ofs + i * TEXINFO_SIZE + 64)[0]
                for i in range(ln // TEXINFO_SIZE)]

    def faces(self):
        ofs, ln = self.lump(LUMP_FACES, DFACE_SIZE)
        if ln == 0:  # an HDR-only BSP keeps its faces in the HDR lump
            ofs, ln = self.lump(LUMP_FACES_HDR, DFACE_SIZE)
        out = []
        for i in range(ln // DFACE_SIZE):
            base = ofs + i * DFACE_SIZE
            firstedge, numedges, texinfo, dispinfo = struct.unpack_from(
                '<ihhh', self.d, base + 4)
            numprims_raw, firstprim = struct.unpack_from(
                '<HH', self.d, base + 48)
            out.append(Face(i, firstedge, numedges, texinfo, dispinfo,
                            firstprim, numprims_raw & 0x7FFF))
        return out

    def world_faces(self):
        """(first, count) of the faces of model 0 (the world); the faces of
        every other model belong to brush entities."""
        ofs, ln = self.lump(LUMP_MODELS, MODEL_SIZE)
        if ln < MODEL_SIZE:
            return None
        # dmodel_t: mins, maxs, origin (9 floats), headnode, firstface, numfaces
        firstface, numfaces = struct.unpack_from('<ii', self.d, ofs + 40)
        return firstface, numfaces

    def prims(self):
        ofs, ln = self.lump(LUMP_PRIMS, PRIM_SIZE)
        return [struct.unpack_from('<BxHHHH', self.d, ofs + p * PRIM_SIZE)
                for p in range(ln // PRIM_SIZE)]

    def prim_indices(self):
        ofs, ln = self.lump(LUMP_PRIMINDICES, 2)
        return list(struct.unpack_from('<%dH' % (ln // 2), self.d, ofs))


def face_winding_verts(face, edges, surfedges):
    """Vertex numbers of a face in winding order: vertex i is the START of
    surfedge i (VBSP's GetEdge2 stores a positive surfedge as v[0] -> v[1] and
    a reversed one as a negative index), so face-local index 0 - the root of
    the engine's fan and of primitive indices - is this list's first entry."""
    out = []
    for i in range(face.firstedge, face.firstedge + face.numedges):
        se = surfedges[i]
        e = edges[se] if se >= 0 else edges[-se]
        out.append(e[0] if se >= 0 else e[1])
    return out


def winding_problem(face, edges, surfedges, num_verts):
    """Why this face's edge indices cannot be followed, or None."""
    if face.firstedge < 0 or face.firstedge + face.numedges > len(surfedges):
        return (f'surfedge range {face.firstedge}+{face.numedges} outside the '
                f'surfedge lump ({len(surfedges)})')
    for i in range(face.firstedge, face.firstedge + face.numedges):
        se = surfedges[i]
        if abs(se) >= len(edges):
            return f'surfedge {se} outside the edge lump ({len(edges)})'
        for v in edges[abs(se)]:
            if v >= num_verts:
                return f'edge {abs(se)} uses vertex {v} outside the vertex ' \
                       f'lump ({num_verts})'
    return None


def fan_triangles(n):
    return [(0, k, k + 1) for k in range(1, n - 1)]


def spike_sides(winding):
    """Polygon sides that a later side walks backwards (A->B then B->A, left
    by welding two verts closer than 0.1 around a sliver): they enclose
    nothing, no triangle can have area on them, and no neighbour borders them."""
    n = len(winding)
    walked = {}
    for k in range(n):
        walked.setdefault((winding[k], winding[(k + 1) % n]), []).append(k)
    return {k for k in range(n)
            if any(j != k for j in walked.get(
                (winding[(k + 1) % n], winding[k]), ()))}


def unsewn_subedges(points, tris, winding):
    """Polygon sides (index k = ring position k to k+1) not carried by a
    triangle with area.  A side carried by no triangle at all counts too."""
    n = len(points)
    spikes = spike_sides(winding)
    covered = [k in spikes for k in range(n)]
    for tri in tris:
        for s in range(3):
            p, q, apex = tri[s], tri[(s + 1) % 3], tri[(s + 2) % 3]
            if q == (p + 1) % n:
                k = p
            elif p == (q + 1) % n:
                k = q
            else:
                continue  # a diagonal
            a, b = points[k], points[(k + 1) % n]
            if _len(_sub(b, a)) < 1e-4:
                covered[k] = True  # coincident verts: nothing to crack against
            elif line_distance(points[apex], a, b) > SEW_DEGENERATE_HEIGHT:
                covered[k] = True
    return [k for k in range(n) if not covered[k]]


def newell_normal(points):
    """Unit winding normal by Newell's method, None for a degenerate winding."""
    n = len(points)
    nx = ny = nz = 0.0
    for k in range(n):
        p1, p2 = points[k], points[(k + 1) % n]
        nx += (p1[1] - p2[1]) * (p1[2] + p2[2])
        ny += (p1[2] - p2[2]) * (p1[0] + p2[0])
        nz += (p1[0] - p2[0]) * (p1[1] + p2[1])
    ln = (nx * nx + ny * ny + nz * nz) ** 0.5
    if ln * ln < 1e-6:
        return None
    return (nx / ln, ny / ln, nz / ln)


def fan_fold_depth(points, tris, normal):
    """How far the fan's triangles fold back over the polygon: the largest
    height by which a triangle's apex lies beyond the line of the side that
    carries it (0 if none does).  The fan is (r, k, k+1): apex r, side k, k+1."""
    worst = 0.0
    for a, b, c in tris:
        bc = _sub(points[c], points[b])
        length = _len(bc)
        if length < 1e-6:
            continue
        height = _dot(_cross(bc, _sub(points[a], points[b])), normal) / length
        worst = max(worst, -height)
    return worst


def fan_folds(points, tris, normal):
    """Does some triangle fold back over a polygon side by more than
    SEW_FLIP_HEIGHT (the fan would draw outside the face)?"""
    return fan_fold_depth(points, tris, normal) > SEW_FLIP_HEIGHT


def tiling_check(points, tris, normal):
    """(problem, fold) for `tris` (face-local index triples) as a tiling of the
    polygon.  problem is why it is not sound, or None: n-2 triangles, none
    folded back over its longest side by more than SEW_FLIP_HEIGHT, signed
    areas adding up to the polygon's, and unsigned areas exceeding it by no
    more than the folds just allowed (the same test faces.cpp applies before it
    keeps a primitive).  fold is the worst triangle's: the height of its corner
    above its longest side when it winds the wrong way."""
    n = len(points)
    if len(tris) != n - 2:
        return (f'{len(tris)} triangles for {n} vertices '
                f'(expected {n - 2})'), 0.0
    p0 = points[0]
    poly = sum(0.5 * _dot(_cross(_sub(points[s], p0), _sub(points[s + 1], p0)),
                          normal) for s in range(1, n - 1))
    signed = absolute = allowance = worst = 0.0
    for t, (a, b, c) in enumerate(tris):
        pa, pb, pc = points[a], points[b], points[c]
        area = 0.5 * _dot(_cross(_sub(pb, pa), _sub(pc, pa)), normal)
        longest = max(_len(_sub(pb, pa)), _len(_sub(pc, pa)),
                      _len(_sub(pc, pb)))
        fold = -2 * area / longest if area < 0 and longest > 1e-6 else 0.0
        worst = max(worst, fold)
        if fold > SEW_FLIP_HEIGHT:
            return (f'triangle {t} {(a, b, c)} is folded back over the '
                    f'face'), worst
        signed += area
        absolute += abs(area)
        allowance += 0.5 * SEW_FLIP_HEIGHT * longest
    slop = 1e-3 * abs(poly) + 0.05
    if abs(signed - poly) > slop:
        return (f'triangle areas add up to {signed:.2f}, the face is '
                f'{poly:.2f}'), worst
    if absolute > poly + allowance + slop:
        return (f'triangles overlap (unsigned area {absolute:.2f} > '
                f'face {poly:.2f})'), worst
    return None, worst


def sewing_fan_root(points, winding):
    """A start vertex whose fan sews every polygon side without folding back
    over the face (the one with the shallowest fold), or None."""
    n = len(points)
    normal = newell_normal(points)
    if normal is None:
        return None
    best = None
    for r in range(n):
        tris = [(r, (r + k) % n, (r + k + 1) % n) for k in range(1, n - 1)]
        fold = fan_fold_depth(points, tris, normal)
        if fold > SEW_FLIP_HEIGHT or unsewn_subedges(points, tris, winding):
            continue
        if best is None or fold < best[0]:
            best = (fold, r)
            if fold <= SEW_FOLD_FREE:
                break
    return None if best is None else best[1]


def straight_verts(points):
    """Ring positions of the t-junction vertices: within STRAIGHT_HEIGHT of
    the line through their two ring neighbours and strictly between them."""
    n = len(points)
    out = set()
    for k in range(n):
        prev, nxt = points[k - 1], points[(k + 1) % n]
        seg = _sub(nxt, prev)
        seg2 = _dot(seg, seg)
        if seg2 < 1e-12:
            continue
        t = _dot(_sub(points[k], prev), seg) / seg2
        if 0.0 < t < 1.0 and \
                line_distance(points[k], prev, nxt) <= STRAIGHT_HEIGHT:
            out.add(k)
    return out


def is_sliver(points):
    """Is the whole polygon within SLIVER_WIDTH of one line (so that its long
    sides cannot have a triangle with area)?"""
    n = len(points)
    best = (0.0, 0, 0)
    for i in range(n):
        for j in range(i + 1, n):
            d = _len(_sub(points[i], points[j]))
            if d > best[0]:
                best = (d, i, j)
    _, i, j = best
    return max(line_distance(p, points[i], points[j])
               for p in points) <= SLIVER_WIDTH


def triangulation(face, prims, pidx):
    """The triangles the engine draws for `face`, as face-local index triples."""
    if not face.numprims:
        return fan_triangles(face.numedges)
    tris = []
    for p in range(face.firstprim, face.firstprim + face.numprims):
        _, fi, ic, _, _ = prims[p]
        run = pidx[fi:fi + ic]
        for q in range(0, len(run) - 2, 3):
            tris.append((run[q], run[q + 1], run[q + 2]))
    return tris


def folded_worst(folded):
    return max(depth for _, depth in folded)


def validate(path, list_all=False):
    print(f'{path}:')
    try:
        b = Bsp(path)
        verts = b.verts()
        edges = b.edges()
        surfedges = b.surfedges()
        flags = b.texinfo_flags()
        faces = b.faces()
        prims = b.prims()
        pidx = b.prim_indices()
        world = b.world_faces()
    except (BspFormatError, OSError, struct.error) as err:
        print(f'  ERROR: cannot read the BSP: {err}')
        print('  VALID=False')
        return False
    num_indices = len(pidx)
    limit = None if list_all else 10

    problems = []   # any entry fails the map

    # --- lump-level checks ------------------------------------------------
    for p, (t, fi, ic, fv, vc) in enumerate(prims):
        if t != 0 or vc != 0:
            problems.append(f'prim {p}: unexpected type={t} vertCount={vc} '
                            f'(expected TRILIST with face-local indices)')
        elif ic % 3:
            problems.append(f'prim {p}: indexCount {ic} is not a whole '
                            f'number of triangles')
        if fi + ic > num_indices:
            problems.append(f'prim {p}: index range {fi}+{ic} outside lump')
    if len(prims) > MAX_PRIMS:
        problems.append(f'{len(prims)} prims exceeds format max {MAX_PRIMS}')
    if num_indices > MAX_PRIM_INDICES:
        problems.append(f'{num_indices} indices exceeds format max '
                        f'{MAX_PRIM_INDICES}')

    # --- per-face checks --------------------------------------------------
    referenced = [False] * len(prims)
    prim_faces = 0
    tj_faces = 0          # drawn faces that have t-junction vertices
    face_faults = []      # malformed faces and primitives that are not tilings
    fixable = []          # fan-drawn, unsewn, but another start vertex sews it
    needs_prim = []       # fan-drawn, unsewn, no start vertex sews it
    prim_residual = []    # primitive still leaves a seam
    folded = []           # fan-drawn faces folding back by > SEW_FLIP_HEIGHT
    exempt = {'water': [], 'brush entity': []}   # seams stock VBSP leaves
    pinched = slivers = degenerate = 0
    max_fold = {'fan': 0.0, 'prim': 0.0}         # worst fold seen
    num_folded = {'fan': 0, 'prim': 0}           # folds deeper than the noise
    for face in faces:
        if face.numedges < 3:
            continue
        if face.numprims:
            prim_faces += 1
            if face.firstprim + face.numprims > len(prims):
                face_faults.append(
                    f'face {face.index}: prim range '
                    f'{face.firstprim}+{face.numprims} outside prim lump')
                continue
            for p in range(face.firstprim, face.firstprim + face.numprims):
                referenced[p] = True
        if face.texinfo >= len(flags):
            face_faults.append(f'face {face.index}: texinfo {face.texinfo} '
                               f'outside the texinfo lump ({len(flags)})')
            continue
        tex_flags = flags[face.texinfo] if face.texinfo >= 0 else 0
        if face.dispinfo >= 0 or tex_flags & SURF_NODRAW:
            continue  # not drawn from its winding
        why = winding_problem(face, edges, surfedges, len(verts))
        if why:
            face_faults.append(f'face {face.index}: {why}')
            continue

        winding = face_winding_verts(face, edges, surfedges)
        points = [verts[v] for v in winding]
        n = len(winding)
        normal = newell_normal(points)
        in_world = world is None or world[0] <= face.index < world[0] + world[1]
        exempt_kind = ('water' if tex_flags & SURF_WARP
                       else None if in_world else 'brush entity')

        # every primitive, each on its own, must tile the face
        if face.numprims > 1:
            face_faults.append(f'face {face.index}: {face.numprims} primitives '
                               f'(VBSP emits one per face)')
            continue
        tris = fan_triangles(n)
        faulty = False
        for p in range(face.firstprim, face.firstprim + face.numprims):
            _, fi, ic, _, _ = prims[p]
            run = pidx[fi:fi + ic]
            tris = [(run[q], run[q + 1], run[q + 2])
                    for q in range(0, len(run) - 2, 3)]
            if any(i >= n for tri in tris for i in tri):
                bad = max(i for tri in tris for i in tri)
                face_faults.append(f'face {face.index} ({n} verts): primitive '
                                   f'{p} index {bad} outside the face')
                faulty = True
            elif normal is not None:
                why, fold = tiling_check(points, tris, normal)
                max_fold['prim'] = max(max_fold['prim'], fold)
                if fold > SEW_FOLD_FREE:
                    num_folded['prim'] += 1
                if why:
                    face_faults.append(f'face {face.index} ({n} verts): '
                                       f'primitive {p} is not a sound tiling: '
                                       f'{why}')
                    faulty = True
        if faulty:
            continue
        if normal is None:
            degenerate += 1          # zero area: nothing to draw or sew
            continue
        if len(set(winding)) < n:
            pinched += 1             # checked like any other face below
        if not face.numprims:
            depth = fan_fold_depth(points, tris, normal)
            max_fold['fan'] = max(max_fold['fan'], depth)
            if depth > SEW_FOLD_FREE:
                num_folded['fan'] += 1
            if depth > SEW_FLIP_HEIGHT and exempt_kind is None:
                folded.append((face, depth))

        straight = straight_verts(points)
        if straight:
            tj_faces += 1
        bad_sides = unsewn_subedges(points, tris, winding)
        if not bad_sides:
            continue
        if is_sliver(points) and sewing_fan_root(points, winding) is None:
            slivers += 1
            continue
        bad_verts = sorted({v for k in bad_sides for v in (k, (k + 1) % n)
                            if v in straight})
        entry = (face, bad_sides, bad_verts)
        if exempt_kind:
            exempt[exempt_kind].append(entry)
        elif face.numprims:
            prim_residual.append(entry)
        else:
            root = sewing_fan_root(points, winding)
            if root is None:
                needs_prim.append(entry)
            else:
                fixable.append(entry + (root,))

    orphans = [p for p, r in enumerate(referenced) if not r]
    orphan_indices = sum(prims[p][2] for p in orphans)

    def tjverts(entries):
        return sum(len(e[2]) for e in entries)

    if face_faults:
        problems.extend(face_faults if limit is None else face_faults[:limit])
        if limit is not None and len(face_faults) > limit:
            problems.append(f'... {len(face_faults) - limit} more face/index '
                            f'faults (--all lists them)')
    if orphans:
        problems.append(f'{len(orphans)} orphan prims ({orphan_indices} '
                        f'indices) are referenced by no face')
    if folded:
        problems.append(f'{len(folded)} fan-drawn faces fold back over '
                        f'themselves by more than {SEW_FLIP_HEIGHT} (the fan '
                        f'draws outside the face; worst {folded_worst(folded):.3f})')
    if fixable:
        problems.append(f'{len(fixable)} faces are fan-drawn with unsewn '
                        f't-junction vertices ({tjverts(fixable)} verts) '
                        f'although another start vertex sews them')
    if needs_prim:
        problems.append(f'{len(needs_prim)} faces are fan-drawn with unsewn '
                        f't-junction vertices ({tjverts(needs_prim)} verts) '
                        f'and no start vertex sews them: they need a '
                        f'primitive that was dropped or could not sew them')
    if prim_residual:
        problems.append(f'{len(prim_residual)} faces still have unsewn '
                        f't-junction vertices ({tjverts(prim_residual)} '
                        f'verts) with their primitive')

    print(f'  prims={len(prims)} primIndices={num_indices} '
          f'primFaces={prim_faces} orphans={len(orphans)} '
          f'orphanIndices={orphan_indices}')
    print(f'  drawnFacesWithTjunctionVerts={tj_faces} '
          f'unsewnButFanSewable={len(fixable)} ({tjverts(fixable)} verts) '
          f'unsewnNeedingPrim={len(needs_prim)} ({tjverts(needs_prim)} verts) '
          f'unsewnWithPrim={len(prim_residual)} '
          f'({tjverts(prim_residual)} verts)')
    print(f'  maxFold fan={max_fold["fan"]:.3f} prim={max_fold["prim"]:.3f} '
          f'(deeper than {SEW_FOLD_FREE}: {num_folded["fan"]} fans, '
          f'{num_folded["prim"]} prims; fans deeper than {SEW_FLIP_HEIGHT}: '
          f'{len(folded)})')
    print(f'  exempt/informational: waterSeams={len(exempt["water"])} '
          f'brushEntitySeams={len(exempt["brush entity"])} '
          f'pinchedWindingsChecked={pinched} slivers={slivers} '
          f'zeroAreaFaces={degenerate}')

    def show(title, entries):
        shown = 0
        for entry in entries:
            if limit is not None and shown >= limit:
                print(f'  ... {len(entries) - shown} more {title} faces '
                      f'(--all lists them)')
                break
            face, bad_sides, bad_verts = entry[:3]
            how = f'{face.numprims} prim(s)' if face.numprims else 'fan'
            extra = (f', start vertex {entry[3]} would sew it'
                     if len(entry) > 3 else '')
            print(f'  {title} FACE {face.index}: {face.numedges} verts, {how}: '
                  f'{len(bad_verts)} unsewn t-junction verts '
                  f'(face-local {bad_verts}), {len(bad_sides)} sides{extra}')
            shown += 1

    shown = 0
    for face, depth in folded:
        if limit is not None and shown >= limit:
            print(f'  ... {len(folded) - shown} more folding fans')
            break
        print(f'  FOLDING-FAN FACE {face.index}: {face.numedges} verts, fan '
              f'folds back by {depth:.3f}')
        shown += 1
    show('FAN-SEWABLE', fixable)
    show('needs-prim', needs_prim)
    show('prim-residual', prim_residual)
    show('WARN water', exempt['water'])
    show('WARN brush-entity', exempt['brush entity'])
    shown = 0
    for p in orphans:
        if limit is not None and shown >= limit:
            print(f'  ... {len(orphans) - shown} more orphan prims')
            break
        print(f'  ORPHAN prim {p}: {prims[p][2]} indices at {prims[p][1]}')
        shown += 1

    if problems:
        for msg in problems:
            print(f'  FAIL: {msg}')
        print('  VALID=False')
        return False
    print('  VALID=True')
    return True


def main(argv):
    args = [a for a in argv[1:] if not a.startswith('--')]
    opts = {a for a in argv[1:] if a.startswith('--')}
    unknown = opts - {'--all', '--warn-only'}
    if unknown or not args:
        print(__doc__)
        return 1
    results = [validate(p, '--all' in opts) for p in args]
    if '--warn-only' in opts:
        return 0
    return 0 if all(results) else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv))
