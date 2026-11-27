"""_cubemap_only_sides: which VMF sides only a cubemap sees (#149).

A side whose BSP faces are all hidden or cubemap-only, with at least one
cubemap-only, is cubemap-only: not hidden (it stays lit and drawn), just
lower priority than a side a player sees. A side with any visible face is
visible; a side whose faces are all hidden is never-visible, not
cubemap-only; a side with no BSP faces is neither.
"""
import os as _os
import sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
import _testkit  # puts scripts and scripts/lib on sys.path
from _testkit import check

import types

import lmoptimizer

# The BSP faces the check knows: NV in the hidden tiers (A, B), CUBE only
# cubemap-seen, 30 visible to a player.
NV = {10, 11}
CUBE = {20, 21}
TEXLIGHTS = {5}


def _sides(spec):
    """spec: {side id: BSP face indices}; the face_data the check built."""
    face_data = {sid: types.SimpleNamespace(bsp_face_indices=indices)
                 for sid, indices in spec.items()}
    return lmoptimizer._cubemap_only_sides(face_data, NV, CUBE, TEXLIGHTS)


def test_a_side_of_cubemap_only_faces_is_cubemap_only():
    got = _sides({1: [20, 21]})
    check("every face cubemap-only -> the side is in the set",
          got == {1}, f"got {got}")
    got = _sides({2: [10, 20]})
    check("cubemap-only plus hidden faces -> still in the set",
          got == {2}, f"got {got}")


def test_a_visible_or_all_hidden_side_is_not_cubemap_only():
    got = _sides({3: [20, 30]})
    check("cubemap-only plus one visible face -> not in the set",
          got == set(), f"got {got}")
    got = _sides({4: [10, 11]})
    check("a side whose faces are all hidden -> not in the set",
          got == set(), f"got {got}")


def test_texlight_and_faceless_sides_are_not_cubemap_only():
    got = _sides({5: [20], 6: [21]})
    check("a texlight side is left out (its cubemap-only sibling is not)",
          got == {6}, f"got {got}")
    got = _sides({7: []})
    check("a side with no BSP faces -> not in the set",
          got == set(), f"got {got}")


if __name__ == '__main__':
    raise SystemExit(_testkit.run_all("test_cubemap_only_sides", [
        test_a_side_of_cubemap_only_faces_is_cubemap_only,
        test_a_visible_or_all_hidden_side_is_not_cubemap_only,
        test_texlight_and_faceless_sides_are_not_cubemap_only,
    ]))
