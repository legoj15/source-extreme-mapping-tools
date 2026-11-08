"""_find_culled_sides: which VMF sides VBSP dropped (#73 merged-detail guard).

A func_detail side missing from the side-ID lumps may sit in a merged face
(VBSP never seeds a detail face's merged-ID list), so it is held whenever a
BSP face lies on its plane and overlaps it; it is culled otherwise. A wrong
"culled" floors a visible side at lightmapscale 16 and splits its merge.
"""
import os as _os
import sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
import _testkit  # noqa: F401  (puts game/bin/x64 on sys.path)

import types

import lmoptimizer


def _quad(x0, y0, x1, y1, z):
    return [(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)]


class _FakeBSP:
    """Only what _find_culled_sides reads: the side-ID lump and face polygons."""

    def __init__(self, faces, side_ids):
        self._faces = [types.SimpleNamespace(planenum=0, verts=v) for v in faces]
        self._side_ids = side_ids

    def read_face_side_ids(self):
        return self._side_ids

    def read_faces(self):
        return self._faces

    def read_planes(self):
        return [object()]

    def read_vertexes(self):
        return None

    def read_edges(self):
        return None

    def read_surfedges(self):
        return None

    def get_face_vertices(self, face, *_):
        return face.verts


def _extent(sid, entity='func_detail'):
    return types.SimpleNamespace(side_id=sid, has_displacement=False,
                                 material='CONCRETE/FLOOR', brush_entity=entity)


def _culled(sides, bsp_faces):
    """sides: {sid: (polygon, entity)}; side 1 is the only one in the lumps."""
    bsp = _FakeBSP(bsp_faces, [[1]] * len(bsp_faces))
    extents = [_extent(sid, ent) for sid, (_, ent) in sides.items()]
    side_map = {sid: object() for sid in sides}
    verts = {sid: poly for sid, (poly, _) in sides.items()}
    return lmoptimizer._find_culled_sides(bsp, extents, side_map, verts)


def test_merged_detail_sides_are_held_and_uncovered_ones_culled():
    # one merged BSP face over x 0..64; float32-style noise on its vertices
    merged = [(x + 3e-6, y - 2e-6, z + 1e-6) for x, y, z in _quad(0, 0, 64, 32, 64)]
    culled = _culled({
        1: (_quad(0, 0, 32, 32, 64), 'func_detail'),       # the face's root side
        2: (_quad(32, 0, 64, 32, 64), 'func_detail'),      # merged sibling
        3: (_quad(10, 4, 30, 4.5, 64), 'func_detail'),     # thin strip inside it
        4: (_quad(64, 0, 96, 32, 64), 'func_detail'),      # only shares an edge
        5: (_quad(0, 0, 64, 32, 128), 'func_detail'),      # other plane
        6: (_quad(0, 0, 64, 32, 64), ''),                  # world: lumps are exact
    }, [merged])
    assert 2 not in culled and 3 not in culled
    assert {4, 5, 6} <= culled
    assert 1 not in culled


def test_no_lumps_means_nothing_culled():
    bsp = _FakeBSP([], None)
    assert lmoptimizer._find_culled_sides(
        bsp, [_extent(2)], {2: object()}, {2: _quad(0, 0, 8, 8, 0)}) == set()
