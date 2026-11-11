"""_find_culled_sides: which VMF sides VBSP dropped (#73).

Version 1 side-ID lumps list every merged side, so a side missing from them
is culled. In version 0 lumps (vbsp_lmo before #73) a side missing from them
may sit in a merged detail face (%compileDetail puts world sides on that path
too), so it is held whenever a BSP face lies on its plane and overlaps it; it
is culled otherwise. A wrong "culled" floors a visible side at lightmapscale
16 and splits its merge.
"""
import os as _os
import sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
import _testkit  # puts game/bin/x64 on sys.path
from _testkit import check

import types

import lmoptimizer


def _quad(x0, y0, x1, y1, z):
    return [(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)]


class _FakeBSP:
    """Only what _find_culled_sides reads: the side-ID lump and face polygons."""

    def __init__(self, faces, side_ids, version=0):
        """faces: (polygon, (normal, dist)) pairs; the plane is what VBSP
        would store in the planes lump."""
        self._faces = [types.SimpleNamespace(planenum=i, verts=v)
                       for i, (v, _) in enumerate(faces)]
        self._planes = [types.SimpleNamespace(normal=n, dist=d)
                        for _, (n, d) in faces]
        self._side_ids = side_ids
        self._version = version

    def face_side_ids_version(self):
        return self._version

    def read_face_side_ids(self):
        return self._side_ids

    def read_faces(self):
        return self._faces

    def read_planes(self):
        return self._planes

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


def _culled(sides, bsp_faces, version=0):
    """sides: {sid: (polygon, entity)}; side 1 is the only one in the lumps."""
    bsp = _FakeBSP(bsp_faces, [[1]] * len(bsp_faces), version)
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
        6: (_quad(8, 8, 24, 24, 64), ''),                  # %compileDetail world side
        7: (_quad(0, 0, 64, 32, 128), ''),                 # world, other plane
    }, [(merged, ((0.0, 0.0, 1.0), 64.0))])
    check("merged siblings held (detail, thin strip, world)",
          not culled & {1, 2, 3, 6}, f"culled={sorted(culled)}")
    check("edge-only, other-plane and uncovered world sides culled",
          {4, 5, 7} <= culled, f"culled={sorted(culled)}")


def test_complete_lumps_cull_every_missing_side():
    # version 1: a coplanar, overlapping side missing from the lumps really
    # was culled (VBSP would have listed it), so geometry no longer holds it
    culled = _culled({
        1: (_quad(0, 0, 32, 32, 64), 'func_detail'),       # in the lumps
        2: (_quad(32, 0, 64, 32, 64), 'func_detail'),      # coplanar, overlapping
        6: (_quad(8, 8, 24, 24, 64), ''),                  # %compileDetail world side
        7: (_quad(0, 0, 64, 32, 128), ''),                 # world, other plane
    }, [(_quad(0, 0, 64, 32, 64), ((0.0, 0.0, 1.0), 64.0))], version=1)
    check("every side missing from version 1 lumps is culled",
          culled == {2, 6, 7}, f"culled={sorted(culled)}")


def test_no_lumps_or_no_faces_means_nothing_culled():
    side = ([_extent(2)], {2: object()}, {2: _quad(0, 0, 8, 8, 0)})
    check("no side-ID lumps",
          lmoptimizer._find_culled_sides(_FakeBSP([], None), *side) == set())
    check("lumps but no readable faces (fail closed)",
          lmoptimizer._find_culled_sides(_FakeBSP([], [[1]]), *side) == set())


if __name__ == '__main__':
    raise SystemExit(_testkit.run_all("test_culled_sides", [
        test_merged_detail_sides_are_held_and_uncovered_ones_culled,
        test_complete_lumps_cull_every_missing_side,
        test_no_lumps_or_no_faces_means_nothing_culled,
    ]))
