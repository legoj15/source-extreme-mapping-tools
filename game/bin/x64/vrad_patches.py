"""VRAD bounce-patch count of a BSP, at its own lightmap scales or at new ones.

Replays src/utils/vrad/vrad.cpp's SubdividePatches with the default chop
(maxchop = minchop = 4 luxel widths): a face's patch is split at the middle of
its widest world axis while any world-axis extent, in luxels, is >= 4, and
VRAD's "N patches after subdivision" counts every node of those trees
(parents included). Every face of nonzero area has a patch; sky faces and
those VRAD never subdivides (SURF_NOCHOP, and SURF_NOLIGHT unless the face
emits light, which VRAD marks SURF_LIGHT at run time) keep one per VBSP
fragment.
Displacements split their lightmap at dispchop (8 luxels) instead; that part
is approximate.

A face recounted at another scale is first chopped the way VBSP's
SubdivideFace (src/utils/vbsp/faces.cpp) would chop it at that scale: along
each lightmap axis, at 31 luxels from its start while it spans more than the
max lightmap dimension (32). Chopping matters: a 66-luxel span is 32 patches
across whole but 18 as VBSP's 31 + 31 + 4.

Checked on trainstation (#83): the input and optimized BSPs count
2,492,934 and 4,635,719 against VRAD's 2,492,962 and 4,635,888 (the
displacement estimate is the gap). Bounce transfers grow with roughly the square of
the leaf patches (measured 3.25x for 1.86x leaves there), and a GPU bounce
needs 8 B per transfer plus 160 B per patch (#84).
"""
import math
from functools import lru_cache
from typing import Dict, List, Optional, Sequence, Set, Tuple

from vertex_estimator import DEFAULT_MAX_LIGHTMAP_DIM

CHOP = 4.0
DISP_CHOP = 8.0
SURF_LIGHT = 0x1
SURF_SKY = 0x4
SURF_NOLIGHT = 0x400
SURF_NOCHOP = 0x4000
_EPS = 0.1


def _split(w, normal, dist):
    """(back, front) of winding w cut by the plane normal . p = dist; points
    within _EPS of it go to both."""
    back, front = [], []
    n = len(w)
    for i in range(n):
        p, q = w[i], w[(i + 1) % n]
        dp = sum(p[k] * normal[k] for k in range(3)) - dist
        dq = sum(q[k] * normal[k] for k in range(3)) - dist
        if dp <= _EPS:
            back.append(p)
        if dp >= -_EPS:
            front.append(p)
        if (dp > _EPS and dq < -_EPS) or (dp < -_EPS and dq > _EPS):
            t = dp / (dp - dq)
            m = tuple(p[k] + t * (q[k] - p[k]) for k in range(3))
            back.append(m)
            front.append(m)
    return back, front


_AXES = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def _tree(w, luxscale: float, chop: float = CHOP) -> Tuple[int, int]:
    """(nodes, leaves) of one face's VRAD patch tree."""
    mins = [min(v[k] for v in w) for k in range(3)]
    maxs = [max(v[k] for v in w) for k in range(3)]
    total = [(maxs[k] - mins[k]) * luxscale for k in range(3)]
    if all(t < chop for t in total):
        return 1, 1
    flat = [k for k in range(3) if maxs[k] - mins[k] < 0.01]
    if len(flat) == 1:
        # On an axis plane, a piece that fills its bounding rectangle splits
        # like that rectangle: hand it to the cached box recursion.
        i, j = [k for k in range(3) if k != flat[0]]
        area = abs(sum(w[n][i] * w[(n + 1) % len(w)][j] - w[(n + 1) % len(w)][i] * w[n][j]
                       for n in range(len(w)))) * 0.5
        if area >= (maxs[i] - mins[i]) * (maxs[j] - mins[j]) - 0.01:
            return _box_tree(tuple(round(t, 4) for t in total), chop)
    axis = max(range(3), key=lambda k: total[k])
    a, b = _split(w, _AXES[axis], (mins[axis] + maxs[axis]) * 0.5)
    if len(a) < 3 or len(b) < 3:
        return 1, 1
    na, la = _tree(a, luxscale, chop)
    nb, lb = _tree(b, luxscale, chop)
    return 1 + na + nb, la + lb


@lru_cache(maxsize=None)
def _box_tree(ext: Tuple[float, float, float], chop: float) -> Tuple[int, int]:
    """(nodes, leaves) for an axis-aligned rectangle with these luxel extents:
    each half is the same rectangle with the widest extent halved."""
    if all(t < chop for t in ext):
        return 1, 1
    axis = max(range(3), key=lambda k: ext[k])
    half = list(ext)
    half[axis] = round(ext[axis] * 0.5, 4)
    n, l = _box_tree(tuple(half), chop)
    return 1 + 2 * n, 2 * l


def vbsp_chop(w, axes: Sequence[Sequence[float]], luxscale: float,
              max_dim: int = DEFAULT_MAX_LIGHTMAP_DIM) -> List[list]:
    """The pieces VBSP's SubdivideFace cuts winding w into at luxscale:
    along each lightmap axis (unit vectors), a piece spanning more than
    max_dim luxels is cut max_dim - 1 luxels past its start, and both halves
    are chopped again. A work stack, not recursion: a long face takes one
    cut per max_dim - 1 luxels, and a frame per cut ran out near 31k luxels."""
    pieces, stack = [], [w]
    while stack:
        cur = stack.pop()
        for axis in axes:
            proj = [sum(p[k] * axis[k] for k in range(3)) * luxscale for p in cur]
            lo, hi = min(proj), max(proj)
            if hi - lo <= max_dim:
                continue
            a, b = _split(cur, axis, (lo + max_dim - 1) / luxscale)
            if len(a) >= 3 and len(b) >= 3:
                stack += [b, a]     # a comes off first, as the recursion ran it
                cur = None
            break
        if cur is not None:
            pieces.append(cur)
    return pieces


def face_patches(w, luxscale: float, axes=None,
                 max_dim: int = DEFAULT_MAX_LIGHTMAP_DIM,
                 subdivide: bool = True) -> Tuple[int, int]:
    """(nodes, leaves) for a face winding at luxscale luxels per world unit;
    with its lightmap axes, after VBSP chops it at that scale. Without
    subdivide (VRAD's PreventSubdivision) each piece is one patch."""
    pieces = vbsp_chop(w, axes, luxscale, max_dim) if axes else [w]
    nodes = leaves = 0
    for piece in pieces:
        n, l = _tree(piece, luxscale) if subdivide else (1, 1)
        nodes += n
        leaves += l
    return nodes, leaves


def _unit(v) -> Tuple[float, float, float]:
    length = math.sqrt(sum(x * x for x in v[:3]))
    return tuple(x / length for x in v[:3]) if length > 0 else (0.0, 0.0, 0.0)


def _area(w) -> float:
    """Area of a planar winding (Newell)."""
    nx = ny = nz = 0.0
    n = len(w)
    for i in range(n):
        p, q = w[i], w[(i + 1) % n]
        nx += (p[1] - q[1]) * (p[2] + q[2])
        ny += (p[2] - q[2]) * (p[0] + q[0])
        nz += (p[0] - q[0]) * (p[1] + q[1])
    return 0.5 * math.sqrt(nx * nx + ny * ny + nz * nz)


def _material(bsp, texinfos) -> List[str]:
    """Upper-case material per texinfo ('' when unknown)."""
    texdata = bsp.read_texdata()
    names = bsp.read_material_names()
    out = []
    for tex in texinfos:
        name = ''
        if 0 <= tex.texdata < len(texdata):
            sid = texdata[tex.texdata].name_string_table_id
            if 0 <= sid < len(names):
                name = names[sid].upper().replace('\\', '/').strip('/')
        out.append(name)
    return out


def count_patches(bsp, face_sides: Optional[Dict[int, List[int]]] = None,
                  scales: Optional[Dict[int, float]] = None,
                  polygons: Optional[Dict[int, List[list]]] = None,
                  side_axes: Optional[Dict[int, tuple]] = None,
                  max_dim: int = DEFAULT_MAX_LIGHTMAP_DIM,
                  replaced: Optional[Dict[int, List[int]]] = None,
                  texlights: Optional[Set[str]] = None) -> Tuple[int, int]:
    """(patches, leaf patches) VRAD makes for a read BSPReader, at its own
    lightmap scales or, given face_sides (BSP face index -> VMF side ids) and
    scales (side id -> lightmapscale), at those: a face of several sides
    takes the finest of theirs, a face of no listed side keeps its own. A
    recounted face is chopped as VBSP would chop it at the new scale
    (SURF_NOLIGHT faces never are).

    A face recounted COARSER than it was compiled is a fragment VBSP chopped
    at the finer scale; compiled coarse, its side is fewer, larger faces. So
    when polygons (side id -> its windings) and side_axes (side id -> its
    lightmap axis unit vectors) have all of such a face's sides, those sides
    are counted once from their polygons instead (a little high where VBSP
    would cut away hidden parts or merge sides into one face).

    replaced (side id -> the side ids that now stand in its place, the
    pieces --chop carved it into): a face of such a side is dropped and the
    pieces are counted from their polygons, each at its own scale.

    texlights (upper-case material names): VRAD marks a face whose material
    emits SURF_LIGHT at run time, which lets a SURF_NOLIGHT face subdivide.
    Zero-area faces get no patch (MakePatchForFace)."""
    want = scales or {}
    sides_of = face_sides or {}
    polys = polygons or {}
    axes_of = side_axes or {}
    swaps = replaced or {}
    lights = texlights or set()
    texinfos = bsp.read_texinfos()
    materials = _material(bsp, texinfos) if lights else []
    verts = bsp.read_vertexes()
    edges = bsp.read_edges()
    surfedges = bsp.read_surfedges()
    nodes = leaves = 0
    whole: Dict[int, Tuple[bool, bool]] = {}   # side -> (chops, subdivides)

    def countable(sid):
        return sid in want and sid in polys and sid in axes_of

    for index, face in enumerate(bsp.read_faces()):
        if not 0 <= face.texinfo < len(texinfos):
            continue
        tex = texinfos[face.texinfo]
        flags = tex.flags
        disp = face.dispinfo >= 0
        w = None
        if not disp:
            w = []
            for i in range(face.firstedge, face.firstedge + face.numedges):
                e = surfedges[i]
                w.append(verts[edges[e][0]] if e >= 0 else verts[edges[-e][1]])
            if len(w) < 3 or _area(w) <= 0:
                continue   # MakePatchForFace: no patch for a zero-area face
        # VRAD sets SURF_LIGHT at run time on a brush face that emits
        # (MakePatchForFace; not on displacement patches).
        lit = bool(flags & SURF_LIGHT) or (
            not disp and bool(materials) and materials[face.texinfo] in lights)
        # VBSP's SubdivideFace skips SURF_NOLIGHT faces; VRAD's
        # PreventSubdivision skips SURF_NOCHOP and unlit SURF_NOLIGHT ones,
        # and SubdividePatch never splits a sky patch (displacement patches
        # are never flagged sky).
        rule = (not flags & SURF_NOLIGHT,
                not (flags & SURF_NOCHOP or (flags & SURF_NOLIGHT and not lit)
                     or (flags & SURF_SKY and not disp)))
        chops, splits = rule
        listed = sides_of.get(index, ())
        swapped = any(sid in swaps for sid in listed)
        if swapped:
            stand_ins = ([sid for sid in listed if sid not in swaps]
                         + [p for sid in listed for p in swaps.get(sid, ())])
            # polygons stand in only for faces VBSP chops: a never-chopped
            # (SURF_NOLIGHT) side keeps one patch per BSP face it is split into
            if not disp and chops and all(countable(sid) for sid in stand_ins):
                for sid in stand_ins:
                    whole[sid] = rule
                continue
            # a piece without a polygon: the face at the finest scale known
            sids = [sid for sid in stand_ins if sid in want]
        else:
            sids = [sid for sid in listed if sid in want]
        lv = tex.lightmap_vecs
        own = (math.sqrt(sum(x * x for x in lv[0][:3]))
               + math.sqrt(sum(x * x for x in lv[1][:3]))) / 2
        if own <= 0:
            nodes += 1
            leaves += 1
            continue
        luxscale = 1.0 / min(want[sid] for sid in sids) if sids else own
        rescaled = abs(luxscale - own) > own * 0.001
        if disp:
            if splits:
                ratio = luxscale / own
                n, l = _box_tree((round((face.lightmap_size[0] + 1) * ratio, 4),
                                  round((face.lightmap_size[1] + 1) * ratio, 4), 0.0),
                                 DISP_CHOP)
            else:
                n = l = 1
        elif (chops and not swapped and luxscale < own and rescaled and sids
              and all(countable(sid) for sid in sids)):
            for sid in sids:
                whole[sid] = rule
            continue
        else:
            axes = (_unit(lv[0]), _unit(lv[1])) if rescaled and chops else None
            n, l = face_patches(w, luxscale, axes, max_dim, splits)
        nodes += n
        leaves += l
    for sid, (chops, splits) in whole.items():
        for w in polys[sid]:
            if len(w) >= 3 and _area(w) > 0:
                n, l = face_patches(w, 1.0 / want[sid],
                                    axes_of[sid] if chops else None,
                                    max_dim, splits)
                nodes += n
                leaves += l
    return nodes, leaves
