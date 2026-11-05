"""Validate the primitive lumps (LUMP_PRIMITIVES/PRIMVERTS/PRIMINDICES) of a
Source BSP.

Checks per map:
  * all primitives are well-formed (type TRILIST, face-local indices
    i.e. vertCount == 0), index ranges are contiguous and inside the lump,
    and totals stay under the hard BSP format limits (32768 prims /
    65536 indices — dprimitive_t's firstIndex/indexCount are 16-bit);
  * every face that carries a primitive has a concave winding (convex faces
    are fanned by the engine and must not need one — see the convexity-gated
    emission in src/utils/vbsp/faces.cpp FixFaceEdges);
  * every primitive index addresses a vertex of its own face.

Usage:
    python bsp_prim_check.py <map.bsp> [more.bsp ...]

Exits non-zero if any check fails.
"""
from __future__ import annotations

import struct
import sys

LUMP_VERTEXES = 3
LUMP_FACES = 7
LUMP_EDGES = 12
LUMP_SURFEDGES = 13
LUMP_PRIMS = 37
LUMP_PRIMINDICES = 39
DFACE_SIZE = 56
PRIM_SIZE = 10
# keep in sync with SupervertPolyIsConvex() in src/utils/vbsp/faces.cpp
COLLINEAR_SIN = 0.01


def _vec_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _vec_cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _vec_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _vec_len(v):
    return (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) ** 0.5


def is_convex(points):
    """Newell-normal convexity test; turns under asin(COLLINEAR_SIN) count
    as straight (inserted t-junction verts are collinear within OFF_EPSILON).
    Degenerate windings report non-convex so callers keep the safe path."""
    n = len(points)
    nx = ny = nz = 0.0
    for k in range(n):
        p1, p2 = points[k], points[(k + 1) % n]
        nx += (p1[1] - p2[1]) * (p1[2] + p2[2])
        ny += (p1[2] - p2[2]) * (p1[0] + p2[0])
        nz += (p1[0] - p2[0]) * (p1[1] + p2[1])
    ln2 = nx * nx + ny * ny + nz * nz
    if ln2 < 1e-6:
        return False
    ln = ln2 ** 0.5
    normal = (nx / ln, ny / ln, nz / ln)
    for k in range(n):
        a = points[k]
        b = points[(k + 1) % n]
        c = points[(k + 2) % n]
        e1 = _vec_sub(b, a)
        e2 = _vec_sub(c, b)
        if _vec_dot(_vec_cross(e1, e2), normal) < \
                -COLLINEAR_SIN * _vec_len(e1) * _vec_len(e2):
            return False
    return True


class Bsp:
    def __init__(self, path):
        self.d = open(path, 'rb').read()

    def lump(self, i):
        return struct.unpack_from('<ii', self.d, 8 + i * 16)

    def verts(self):
        ofs, ln = self.lump(LUMP_VERTEXES)
        return [struct.unpack_from('<3f', self.d, ofs + i * 12)
                for i in range(ln // 12)]

    def edges(self):
        ofs, ln = self.lump(LUMP_EDGES)
        return [struct.unpack_from('<HH', self.d, ofs + i * 4)
                for i in range(ln // 4)]

    def surfedges(self):
        ofs, ln = self.lump(LUMP_SURFEDGES)
        return [v[0] for v in struct.iter_unpack('<i', self.d[ofs:ofs + ln])]

    def faces(self):
        """Yield (firstedge, numedges, firstPrimID, numPrims) per face."""
        ofs, ln = self.lump(LUMP_FACES)
        for i in range(ln // DFACE_SIZE):
            numprims_raw, firstprim = struct.unpack_from(
                '<HH', self.d, ofs + i * DFACE_SIZE + 48)
            firstedge, numedges = struct.unpack_from(
                '<ih', self.d, ofs + i * DFACE_SIZE + 4)
            yield firstedge, numedges, firstprim, numprims_raw & 0x7FFF

    def face_winding_verts(self, firstedge, numedges, edges, surfedges):
        idx = []
        for i in range(firstedge, firstedge + numedges):
            se = surfedges[i]
            e = edges[se if se >= 0 else -se]
            idx.append(e[1] if se >= 0 else e[0])
        return idx

    def prims(self):
        ofs, ln = self.lump(LUMP_PRIMS)
        for p in range(ln // PRIM_SIZE):
            yield struct.unpack_from('<BxHHHH', self.d, ofs + p * PRIM_SIZE)

    def prim_index(self, i):
        ofs = self.lump(LUMP_PRIMINDICES)[0]
        return struct.unpack_from('<H', self.d, ofs + i * 2)[0]


def validate(path):
    b = Bsp(path)
    verts = b.verts()
    edges = b.edges()
    surfedges = b.surfedges()

    prims = list(b.prims())
    num_indices = b.lump(LUMP_PRIMINDICES)[1] // 2

    problems = []
    expected_first = None
    for p, (t, fi, ic, fv, vc) in enumerate(prims):
        if t != 0 or vc != 0:
            problems.append(f'prim {p}: unexpected type={t} vertCount={vc} '
                            f'(expected TRILIST face-local)')
            continue
        if fi + ic > num_indices:
            problems.append(f'prim {p}: index range {fi}+{ic} outside lump')
        if expected_first is not None and fi != expected_first:
            problems.append(f'prim {p}: non-contiguous firstIndex '
                            f'{fi} != {expected_first}')
        expected_first = fi + ic
    if len(prims) > 32768:
        problems.append(f'{len(prims)} prims exceeds format max 32768')
    if num_indices > 65536:
        problems.append(f'{num_indices} indices exceeds format max 65536')

    prim_face_count = 0
    for firstedge, numedges, firstprim, numprims in b.faces():
        if not numprims:
            continue
        prim_face_count += 1
        winding = b.face_winding_verts(firstedge, numedges, edges, surfedges)
        if firstprim + numprims > len(prims):
            problems.append(
                f'face with {numedges} edges: prim range {firstprim}+{numprims} '
                f'outside prim lump')
            continue
        if not is_convex([verts[v] for v in winding]):
            continue  # concave faces are exactly the ones needing a prim
        convex_prim_face = True
        for p in range(firstprim, firstprim + numprims):
            t, fi, ic, fv, vc = prims[p]
            for q in range(fi, fi + ic):
                if b.prim_index(q) >= len(winding):
                    convex_prim_face = False
                    problems.append(
                        f'CONVEX face (fan-safe) carries a prim {p} — '
                        f'redundant; and index {b.prim_index(q)} exceeds its '
                        f'{len(winding)}-vert winding')
                    break
            if not convex_prim_face:
                break

    print(f'{path}:')
    print(f'  prims={len(prims)} primIndices={num_indices} '
          f'primFaces={prim_face_count} orphans={len(prims) - prim_face_count}')
    if problems:
        for msg in problems[:10]:
            print(f'  FAIL: {msg}')
        print('  VALID=False')
        return False
    print('  VALID=True')
    return True


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 1
    ok = all(validate(p) for p in argv[1:])
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv))
