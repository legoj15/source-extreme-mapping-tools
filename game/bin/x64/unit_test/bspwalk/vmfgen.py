#!/usr/bin/env python3
"""
vmfgen -- minimal, deterministic VMF builder for the bspwalk fixture maps.

A fixture is a small hand-built map whose answer (which brush sides a player
can ever see) is known by construction.  This module builds the VMF and the
`*.expect.json` sidecar from a few calls:

    import vmfgen
    m = vmfgen.Map(name="f99_demo", description="...")
    room = m.room((0, 0, 0), (256, 256, 128))          # INTERIOR bounds
    m.entity("info_player_start", origin=room.floor_center())
    m.expect_visible(room.inner("floor"), "player stands on it")
    m.write("f99_demo.vmf", "f99_demo.expect.json")

Everything is deterministic: ids come from one counter that starts at 2 (the
world is id 1) and increments in creation order, so re-running a fixture
rewrites byte-identical files.

------------------------------------------------------------------------------
Conventions
------------------------------------------------------------------------------
* Axes: +x east, -x west, +y north, -y south, +z up.  Box faces are named
  'top' (+z) 'bottom' (-z) 'north' (+y) 'south' (-y) 'east' (+x) 'west' (-x),
  always by the OUTWARD normal of the face.
* Coordinates must be multiples of 1/64 so they are exactly representable in
  float32 (VBSP reads plane points as floats).  Off-grid values such as 7.25
  are fine; 7.3 raises ValueError.  Plane points are three of the face's own
  vertices, so integer geometry yields integer plane points.
  (The VMF format itself does allow fractional plane points; Hammer writes
  them for off-grid brushes.)
* Plane points are wound clockwise when viewed from outside the brush, which
  is the VBSP/Hammer convention: normal = (p0 - p1) x (p2 - p1).
* Interior-bounds convention: `room(mins, maxs)` takes the PLAYABLE interior;
  the shell is built OUTSIDE it.  A 32.5-wide corridor is interior width 32.5.

------------------------------------------------------------------------------
Traps worth knowing (each cost someone a debugging session)
------------------------------------------------------------------------------
* VBSP fills every empty region that no entity can flood into with solid and
  throws its faces away.  A sealed room with no entity inside therefore
  vanishes from the BSP.  Give such rooms an entity: room(..., marker=True).
* VBSP ignores entities whose origin is exactly "0 0 0" when flooding.
  Map.entity() refuses that origin for point entities.
* VBSP merges coplanar, same-material, adjacent sides into one BSP face, so
  two sides can share a face.  Never give a must-be-visible side and a
  must-be-hidden side the same plane and material when they touch.  The test
  harness reports this as a fixture defect.
* A leak (an empty region touching the void) makes VBSP refuse to write the
  BSP.  Every fixture must be sealed.

------------------------------------------------------------------------------
Player constants (HL2, Source SDK 2013)
------------------------------------------------------------------------------
Fixtures derive their margins from these, so they live here once.
"""
from __future__ import annotations

import json
import math
import sys
from itertools import combinations
from math import gcd
from pathlib import Path
from typing import (Callable, Dict, Iterable, List, Optional, Sequence, Tuple,
                    Union)

_X64_DIR = Path(__file__).resolve().parents[2]   # game/bin/x64
if str(_X64_DIR) not in sys.path:
    sys.path.insert(0, str(_X64_DIR))

from vmf_parser import KVNode, KVPair, VMFWriter  # noqa: E402

Vec = Tuple[float, float, float]
Rect = Tuple[float, float, float, float]

# --- Materials -------------------------------------------------------------
MAT_WALL = "DEV/DEV_MEASUREGENERIC01"
MAT_NODRAW = "TOOLS/TOOLSNODRAW"
MAT_SKYBOX = "TOOLS/TOOLSSKYBOX"
# DEV/DEV_MEASUREGENERIC01B is deliberately NOT exported: vis_simulator uses it
# as its "never visible" paint, so a fixture using it would be ambiguous.

# --- Player constants (HL2) -----------------------------------------------
HULL_STAND = (32.0, 32.0, 72.0)
HULL_DUCK = (32.0, 32.0, 36.0)
EYE_STAND = 64.0
EYE_DUCK = 28.0
EYE_DEAD = 14.0                  # a dead player's view over the floor (HL2 VEC_DEAD_VIEWHEIGHT; the walker profile key eye_dead)
STEP_HEIGHT = 18.0
SV_GRAVITY = 600.0               # HL2.  (The engine default 800 gives a shorter airtime;
                                 # fixtures are sized to hold under both.)
JUMP_APEX = 21.0                 # GAMEMOVEMENT_JUMP_HEIGHT: jump speed = sqrt(2*g*21) ~ 158.7 u/s
CROUCH_LIFT = HULL_STAND[2] - HULL_DUCK[2]          # 36: feet rise when ducking in the air
STAND_JUMP_FEET = JUMP_APEX                          # highest feet reach, no duck
CROUCH_JUMP_FEET = JUMP_APEX + CROUCH_LIFT           # 57: highest feet reach with a duck-jump
MAX_EYE_ABOVE_FLOOR = max(STAND_JUMP_FEET + EYE_STAND,
                          CROUCH_JUMP_FEET + EYE_DUCK)   # ~85: highest eye over a floor
RUN_SPEED = 190.0                # hl2_normspeed
SPRINT_SPEED = 320.0             # hl2_sprintspeed
AIR_CONTROL_SPEED = 30.0         # air wishspeed cap: a standing jump gains at most ~30 u/s
# bspwalk's arc bound (#13, bspwalk_profiles.txt hl2): the walker tries arcs up to
# MAX_AIR_SPEED * REACH_MARGIN horizontally, with the SDK jump speed of 160 u/s.
JUMP_SPEED = 160.0               # gamemovement sv_jump speed (apex 21.33, a hair over JUMP_APEX)
MAX_AIR_SPEED = 2 * SPRINT_SPEED # 640: the design doc's air speed ceiling, 2x sprint
REACH_MARGIN = 1.15


def airtime(gravity: float = SV_GRAVITY, apex: float = JUMP_APEX) -> float:
    """Seconds a flat jump stays in the air (up and back to the same height)."""
    return 2.0 * math.sqrt(2.0 * apex / gravity)


def jump_gap_reach(speed: float, gravity: float = SV_GRAVITY) -> float:
    """Widest edge-to-edge gap a flat jump at `speed` can cross.

    The hull may overhang the edge by just under 16 on take-off and land with
    just under 16 overhanging, so the hull centre travels gap - 32.
    """
    return speed * airtime(gravity) + 32.0


def arc_reach_bound(max_air: float = MAX_AIR_SPEED, margin: float = REACH_MARGIN, gravity: float = SV_GRAVITY,
                    jump_speed: float = JUMP_SPEED, lift: float = CROUCH_LIFT, half_width: float = HULL_STAND[0] / 2,
                    air_jumps: int = 0) -> float:
    """Widest XY distance between walkable polygons that bspwalk tries an arc across (its R).

    Mirrors PlayerProfile::Derive: the airtime of a duck-jump (plus air jumps at
    the apex) landing on the launch height, at max_air * margin, plus the hull
    width.  A gap wider than this is never crossed by the walker under that profile.
    """
    k = air_jumps
    apex = jump_speed * jump_speed / (2.0 * gravity)
    rise = apex * (1 + k) + lift
    t = (1 + k) * jump_speed / gravity + math.sqrt(2.0 * rise / gravity)
    return max_air * margin * t + 2.0 * half_width


FACE_NAMES = ("floor", "ceiling", "north", "south", "east", "west")
# Name of the brush side that faces INTO a room, per room face.
_INNER_SIDE = {"floor": "top", "ceiling": "bottom", "north": "south",
               "south": "north", "east": "west", "west": "east"}
_OUTER_SIDE = {"floor": "bottom", "ceiling": "top", "north": "north",
               "south": "south", "east": "east", "west": "west"}

_GRID = 64


# ---------------------------------------------------------------------------
# Number handling
# ---------------------------------------------------------------------------
def _q(v: float) -> float:
    """Return v if it is an exact multiple of 1/64, else raise."""
    s = round(v * _GRID)
    if abs(s / _GRID - v) > 1e-9:
        raise ValueError(f"coordinate {v!r} is not a multiple of 1/{_GRID}; "
                         f"use values such as 7.25 or 39.75")
    return s / _GRID


def _vec(v: Sequence[float]) -> Vec:
    if len(v) != 3:
        raise ValueError(f"expected 3 components, got {v!r}")
    return (_q(float(v[0])), _q(float(v[1])), _q(float(v[2])))


def _fvec(v: Sequence[float]) -> Vec:
    """A point for the verifier declarations (not written to the VMF: any float)."""
    return (float(v[0]), float(v[1]), float(v[2]))


def _fmt(v: float) -> str:
    """Format an exact 1/64 multiple without exponent or trailing zeros."""
    s = f"{v:.6f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def _fmt_f(v: float) -> str:
    """Format an arbitrary float (normals, distances) compactly."""
    s = f"{v:.6g}"
    return "0" if s in ("-0",) else s


def _fmt_vec(v: Sequence[float]) -> str:
    return " ".join(_fmt(c) for c in v)


def _iv(v: Vec) -> Tuple[int, int, int]:
    return (round(v[0] * _GRID), round(v[1] * _GRID), round(v[2] * _GRID))


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------
class Side:
    """One brush side.  `id` is the VMF side id that -emitsideids reports."""

    def __init__(self, sid: int, name: str, verts: List[Vec],
                 normal: Tuple[int, int, int], material: str):
        self.id = sid
        self.name = name
        self.verts = verts            # clockwise when viewed from outside
        self.normal = normal          # integer outward normal (not unit length)
        self.material = material
        self.lightmapscale = 16
        self.disp: Optional[dict] = None

    @property
    def unit_normal(self) -> Vec:
        n = math.sqrt(sum(c * c for c in self.normal))
        return (self.normal[0] / n, self.normal[1] / n, self.normal[2] / n)

    def plane_points(self) -> List[Vec]:
        """Three of this side's own vertices, cyclic order preserved (CW from outside)."""
        v = self.verts
        if len(v) == 3:
            return list(v)
        best, best_area = None, -1.0
        for a, b, c in combinations(range(len(v)), 3):
            ab = _sub(v[b], v[a])
            ac = _sub(v[c], v[a])
            cx = _cross(ab, ac)
            area = cx[0] ** 2 + cx[1] ** 2 + cx[2] ** 2
            if area > best_area + 1e-12:
                best, best_area = (a, b, c), area
        return [v[best[0]], v[best[1]], v[best[2]]]


class Solid:
    """A convex brush.  Sides are addressable by name: solid.side('top') -> id."""

    def __init__(self, sid: int, sides: List[Side], tag: Optional[str] = None):
        self.id = sid
        self.sides = sides
        self.tag = tag
        self.owner: Optional["Entity"] = None   # None = worldspawn
        self._by_name: Dict[str, Side] = {s.name: s for s in sides}

    def side_obj(self, name: str) -> Side:
        try:
            return self._by_name[name]
        except KeyError:
            raise KeyError(f"solid {self.id} has no side {name!r}; "
                           f"it has {sorted(self._by_name)}") from None

    def rename_side(self, old: str, new: str) -> None:
        s = self._by_name.pop(old)
        s.name = new
        self._by_name[new] = s

    def side(self, name: str) -> int:
        """Side id of the face named `name` ('top','bottom','north','south','east','west')."""
        return self.side_obj(name).id

    @property
    def mins(self) -> Vec:
        return tuple(min(v[i] for s in self.sides for v in s.verts) for i in range(3))

    @property
    def maxs(self) -> Vec:
        return tuple(max(v[i] for s in self.sides for v in s.verts) for i in range(3))


class Entity:
    """A point or brush entity.  Created through Map.entity / Map.brush_entity."""

    def __init__(self, eid: int, classname: str, origin: Optional[Vec],
                 keys: Dict[str, str]):
        self.id = eid
        self.classname = classname
        self.origin = origin
        self.keys = keys
        self.connections: List[Tuple[str, str]] = []
        self.solids: List[Solid] = []


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _hull_faces(points: List[Vec]):
    """Exact convex hull of 1/64-grid points.

    Returns [(int_normal, [vertex indices ordered clockwise from outside])].
    Works in scaled integers, so coplanarity tests need no epsilon.
    """
    ip = [_iv(p) for p in points]
    planes: Dict[Tuple[int, int, int, int], None] = {}
    for i, j, k in combinations(range(len(ip)), 3):
        a, b, c = ip[i], ip[j], ip[k]
        n = _cross(_sub(b, a), _sub(c, a))
        if n == (0, 0, 0):
            continue
        d = _dot(n, a)
        pos = neg = False
        for p in ip:
            s = _dot(n, p) - d
            if s > 0:
                pos = True
            elif s < 0:
                neg = True
            if pos and neg:
                break
        if pos and neg:
            continue
        if not pos and not neg:
            raise ValueError("all vertices are coplanar; brush has no volume")
        if pos:                        # all points on the n.x >= d side: flip outward
            n = (-n[0], -n[1], -n[2])
            d = -d
        g = gcd(gcd(abs(n[0]), abs(n[1])), abs(n[2]))
        planes[(n[0] // g, n[1] // g, n[2] // g, d // g)] = None
    if len(planes) < 4:
        raise ValueError("fewer than 4 faces; brush has no volume")

    faces = []
    for (nx, ny, nz, d) in sorted(planes):
        n = (nx, ny, nz)
        idx = [m for m, p in enumerate(ip) if _dot(n, p) == d]
        pts = [points[m] for m in idx]
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        cz = sum(p[2] for p in pts) / len(pts)
        nl = math.sqrt(nx * nx + ny * ny + nz * nz)
        un = (nx / nl, ny / nl, nz / nl)
        r0 = (pts[0][0] - cx, pts[0][1] - cy, pts[0][2] - cz)
        rl = math.sqrt(_dot(r0, r0))
        e1 = (r0[0] / rl, r0[1] / rl, r0[2] / rl)
        e2 = _cross(un, e1)

        def ang(m, e1=e1, e2=e2):
            p = points[m]
            r = (p[0] - cx, p[1] - cy, p[2] - cz)
            return math.atan2(_dot(r, e2), _dot(r, e1))

        ccw = sorted(idx, key=ang)           # CCW viewed from outside (right-hand about +n)
        faces.append((n, list(reversed(ccw))))   # VBSP wants clockwise
    return faces


def _axis_face_name(n: Tuple[int, int, int]) -> Optional[str]:
    if n[0] == 0 and n[1] == 0:
        return "top" if n[2] > 0 else "bottom"
    if n[0] == 0 and n[2] == 0:
        return "north" if n[1] > 0 else "south"
    if n[1] == 0 and n[2] == 0:
        return "east" if n[0] > 0 else "west"
    return None


def _tex_axes(n: Tuple[int, int, int]) -> Tuple[str, str]:
    """World-aligned texture axes (Hammer's default projection) for a face normal."""
    ax, ay, az = abs(n[0]), abs(n[1]), abs(n[2])
    if az >= ax and az >= ay:
        return "[1 0 0 0] 0.25", "[0 -1 0 0] 0.25"
    if ax >= ay:
        return "[0 1 0 0] 0.25", "[0 0 -1 0] 0.25"
    return "[1 0 0 0] 0.25", "[0 0 -1 0] 0.25"


def _rect_overlap(a: Rect, b: Rect) -> bool:
    return min(a[1], b[1]) > max(a[0], b[0]) and min(a[3], b[3]) > max(a[2], b[2])


def _cut_rect(piece: Rect, hole: Rect) -> List[Rect]:
    """Remove `hole` from `piece`; return the up-to-4 remaining rectangles."""
    if not _rect_overlap(piece, hole):
        return [piece]
    u0, u1, v0, v1 = piece
    h0, h1 = max(hole[0], u0), min(hole[1], u1)
    g0, g1 = max(hole[2], v0), min(hole[3], v1)
    out: List[Rect] = []
    if h0 > u0:
        out.append((u0, h0, v0, v1))            # left
    if h1 < u1:
        out.append((h1, u1, v0, v1))            # right
    if g0 > v0:
        out.append((h0, h1, v0, g0))            # below the hole
    if g1 < v1:
        out.append((h0, h1, g1, v1))            # above the hole
    return out


class Room:
    """A sealed hollow room built from boxes.  Created by Map.room().

    `inner(face)` / `inner_all(face)` give the side id(s) that face INTO the
    room for 'floor','ceiling','north','south','east','west'.  A wall with an
    opening is several boxes, so use inner_all() for it; inner() raises if
    there is not exactly one.  `outer()/outer_all()` give the outside-facing
    sides.
    """

    def __init__(self, mins: Vec, maxs: Vec, wall: float):
        self.mins = mins
        self.maxs = maxs
        self.wall = wall
        self.solids: List[Solid] = []
        # face -> [(solid, rect)] ; rect is in the wall's own (u, v) frame
        self._pieces: Dict[str, List[Tuple[Solid, Rect]]] = {}
        self._interior: Dict[str, Rect] = {}

    def _select(self, face: str, inner: bool) -> List[int]:
        if face not in FACE_NAMES:
            raise ValueError(f"unknown room face {face!r}; use one of {FACE_NAMES}")
        if face not in self._pieces:
            raise ValueError(f"room has no {face} (omitted)")
        side_name = (_INNER_SIDE if inner else _OUTER_SIDE)[face]
        ids = []
        for solid, rect in self._pieces[face]:
            # Corner pieces outside the interior rectangle have an inner side that is
            # pressed against a neighbouring wall; VBSP culls it, so it is not a face.
            if inner and not _rect_overlap(rect, self._interior[face]):
                continue
            ids.append(solid.side(side_name))
        return ids

    def inner_all(self, face: str) -> List[int]:
        return self._select(face, True)

    def inner(self, face: str) -> int:
        ids = self._select(face, True)
        if len(ids) != 1:
            raise ValueError(f"room {face} has {len(ids)} inner sides (it has an opening?); "
                             f"use inner_all({face!r})")
        return ids[0]

    def outer_all(self, face: str) -> List[int]:
        return self._select(face, False)

    def outer(self, face: str) -> int:
        ids = self._select(face, False)
        if len(ids) != 1:
            raise ValueError(f"room {face} has {len(ids)} outer sides; use outer_all({face!r})")
        return ids[0]

    def all_inner(self) -> List[int]:
        """Every inner side that exists, in FACE_NAMES order."""
        out: List[int] = []
        for f in FACE_NAMES:
            if f in self._pieces:
                out.extend(self.inner_all(f))
        return out

    def center(self) -> Vec:
        return tuple((self.mins[i] + self.maxs[i]) / 2.0 for i in range(3))

    def floor_center(self, dz: float = 1.0) -> Vec:
        """Point in the middle of the floor, `dz` above it (good for info_player_start)."""
        c = self.center()
        return (c[0], c[1], self.mins[2] + dz)


# ---------------------------------------------------------------------------
# Map
# ---------------------------------------------------------------------------
class Map:
    """A VMF under construction plus its visibility expectations."""

    def __init__(self, name: str = "", description: str = "",
                 skyname: str = "sky_day01_01"):
        self.name = name
        self.description = description
        self.skyname = skyname
        self.notes: List[str] = []
        self._next_id = 2                      # world is id 1
        self.solids: List[Solid] = []
        self.entities: List[Entity] = []
        self._visible: List[Tuple[int, str]] = []
        self._hidden: List[Tuple[int, str]] = []
        self._reach: List[Tuple[Vec, str]] = []
        self._no_reach: List[Tuple[Vec, str]] = []
        self._cubemaps: List[dict] = []
        self._verify: dict = {}                # verify_expectations.py declarations

    # -- ids --------------------------------------------------------------
    def _alloc(self) -> int:
        i = self._next_id
        self._next_id += 1
        return i

    # -- brushes ----------------------------------------------------------
    def convex(self, vertices: Sequence[Sequence[float]], material: str = MAT_WALL,
               tag: Optional[str] = None,
               face_materials: Optional[Dict[str, str]] = None) -> Solid:
        """Convex brush from its vertex list (the convex hull of the points).

        Use this for ramps and wedges.  Axis-aligned faces get the standard
        names ('top', 'north', ...); slanted faces are named 'face0', 'face1'...
        in hull order (look them up with solid.sides, or rename with
        solid.rename_side()).  `face_materials` overrides material by face name.
        """
        pts: List[Vec] = []
        for v in vertices:
            q = _vec(v)
            if q not in pts:
                pts.append(q)
        if len(pts) < 4:
            raise ValueError("a brush needs at least 4 distinct vertices")
        solid_id = self._alloc()
        sides: List[Side] = []
        n_other = 0
        for n, idx in _hull_faces(pts):
            name = _axis_face_name(n)
            if name is None:
                name = f"face{n_other}"
                n_other += 1
            mat = (face_materials or {}).get(name, material)
            sides.append(Side(self._alloc(), name, [pts[i] for i in idx], n, mat))
        solid = Solid(solid_id, sides, tag)
        self.solids.append(solid)
        return solid

    def box(self, mins: Sequence[float], maxs: Sequence[float],
            material: str = MAT_WALL, tag: Optional[str] = None,
            face_materials: Optional[Dict[str, str]] = None) -> Solid:
        """Axis-aligned box.  solid.side('top'|'bottom'|'north'|'south'|'east'|'west') -> side id."""
        lo, hi = _vec(mins), _vec(maxs)
        if not all(hi[i] > lo[i] for i in range(3)):
            raise ValueError(f"box needs mins < maxs on every axis: {lo} {hi}")
        corners = [(x, y, z) for x in (lo[0], hi[0]) for y in (lo[1], hi[1])
                   for z in (lo[2], hi[2])]
        return self.convex(corners, material, tag, face_materials)

    def brush_from_planes(self, planes: Sequence[Tuple[float, float, float, float]],
                          material: str = MAT_WALL, tag: Optional[str] = None,
                          face_materials: Optional[Dict[str, str]] = None) -> Solid:
        """Convex brush from half-spaces (nx, ny, nz, d): inside is n.x <= d.

        Every vertex (intersection of three planes) must land on the 1/64 grid,
        otherwise ValueError: pass vertices to convex() instead.
        """
        from fractions import Fraction
        fp = [tuple(Fraction(c).limit_denominator(1 << 20) for c in p) for p in planes]
        verts: List[Vec] = []
        for (a, b, c) in combinations(fp, 3):
            m = [a[:3], b[:3], c[:3]]
            det = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                   - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                   + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
            if det == 0:
                continue
            rhs = [a[3], b[3], c[3]]

            def solve(col):
                mm = [list(r) for r in m]
                for r in range(3):
                    mm[r][col] = rhs[r]
                return (mm[0][0] * (mm[1][1] * mm[2][2] - mm[1][2] * mm[2][1])
                        - mm[0][1] * (mm[1][0] * mm[2][2] - mm[1][2] * mm[2][0])
                        + mm[0][2] * (mm[1][0] * mm[2][1] - mm[1][1] * mm[2][0])) / det
            p = (solve(0), solve(1), solve(2))
            if all(q[0] * p[0] + q[1] * p[1] + q[2] * p[2] <= q[3] for q in fp):
                v = tuple(float(x) for x in p)
                if v not in verts:
                    verts.append(v)
        if len(verts) < 4:
            raise ValueError("planes do not bound a volume")
        return self.convex(verts, material, tag, face_materials)

    def wedge(self, mins: Sequence[float], maxs: Sequence[float], high: str = "east",
              material: str = MAT_WALL, tag: Optional[str] = None) -> Solid:
        """Ramp: a box whose top slopes from full height at the `high` side
        ('east','west','north','south') down to zero thickness at the opposite
        side.  The slanted side is named 'slope'.
        """
        lo, hi = _vec(mins), _vec(maxs)
        lowside = {"east": "west", "west": "east", "north": "south", "south": "north"}
        if high not in lowside:
            raise ValueError("high must be east/west/north/south")
        pts = []
        for x in (lo[0], hi[0]):
            for y in (lo[1], hi[1]):
                pts.append((x, y, lo[2]))
        for x in (lo[0], hi[0]):
            for y in (lo[1], hi[1]):
                at_high = ((high == "east" and x == hi[0]) or (high == "west" and x == lo[0])
                           or (high == "north" and y == hi[1]) or (high == "south" and y == lo[1]))
                if at_high:
                    pts.append((x, y, hi[2]))
        s = self.convex(pts, material, tag)
        slanted = [x for x in s.sides if x.name.startswith("face")]
        if len(slanted) != 1:
            raise AssertionError("wedge should have exactly one slanted side")
        s.rename_side(slanted[0].name, "slope")
        return s

    # -- rooms ------------------------------------------------------------
    def room(self, mins: Sequence[float], maxs: Sequence[float], wall: float = 16,
             material: str = MAT_WALL, openings: Sequence[Tuple[str, Rect]] = (),
             omit: Sequence[str] = (), floor: bool = True, ceiling: bool = True,
             marker: bool = False, tag: Optional[str] = None) -> Room:
        """Hollow sealed room whose INTERIOR is mins..maxs, shell built outside it.

        openings: [(face, (a0, a1, b0, b1)), ...] cuts a hole through a wall by
          building that wall from boxes around the hole.  The rectangle is in
          WORLD coordinates of the two in-plane axes:
              'east'/'west'   -> (ymin, ymax, zmin, zmax)
              'north'/'south' -> (xmin, xmax, zmin, zmax)
              'floor'/'ceiling' -> (xmin, xmax, ymin, ymax)
          Two rooms whose openings line up are joined, and the pair stays sealed
          (each room's own shell closes the sides of the passage).
        omit: faces not built at all (for a room whose neighbour supplies that
          wall).  Slabs and walls shrink so nothing overlaps; the caller must
          ensure the omitted face is closed by something else.
        floor/ceiling=False is shorthand for omitting them.
        marker=True drops an info_target inside so VBSP does not fill a sealed
          room as 'outside' (see module docstring).
        """
        lo, hi = _vec(mins), _vec(maxs)
        wall = _q(float(wall))
        if not all(hi[i] > lo[i] for i in range(3)):
            raise ValueError(f"room needs mins < maxs on every axis: {lo} {hi}")
        omitted = set(omit)
        if not floor:
            omitted.add("floor")
        if not ceiling:
            omitted.add("ceiling")
        for f in omitted:
            if f not in FACE_NAMES:
                raise ValueError(f"unknown face {f!r} in omit")

        def th(face):
            return 0.0 if face in omitted else wall

        wx0, wx1 = lo[0] - th("west"), hi[0] + th("east")
        wy0, wy1 = lo[1] - th("south"), hi[1] + th("north")
        # wall extent (u0,u1,v0,v1) in its own frame, and the box builder for a sub-rect
        spec = {
            "floor": ((wx0, wx1, wy0, wy1),
                      lambda r: ((r[0], r[2], lo[2] - wall), (r[1], r[3], lo[2]))),
            "ceiling": ((wx0, wx1, wy0, wy1),
                        lambda r: ((r[0], r[2], hi[2]), (r[1], r[3], hi[2] + wall))),
            "north": ((wx0, wx1, lo[2], hi[2]),
                      lambda r: ((r[0], hi[1], r[2]), (r[1], hi[1] + wall, r[3]))),
            "south": ((wx0, wx1, lo[2], hi[2]),
                      lambda r: ((r[0], lo[1] - wall, r[2]), (r[1], lo[1], r[3]))),
            "east": ((lo[1], hi[1], lo[2], hi[2]),
                     lambda r: ((hi[0], r[0], r[2]), (hi[0] + wall, r[1], r[3]))),
            "west": ((lo[1], hi[1], lo[2], hi[2]),
                     lambda r: ((lo[0] - wall, r[0], r[2]), (lo[0], r[1], r[3]))),
        }
        interior = {
            "floor": (lo[0], hi[0], lo[1], hi[1]), "ceiling": (lo[0], hi[0], lo[1], hi[1]),
            "north": (lo[0], hi[0], lo[2], hi[2]), "south": (lo[0], hi[0], lo[2], hi[2]),
            "east": (lo[1], hi[1], lo[2], hi[2]), "west": (lo[1], hi[1], lo[2], hi[2]),
        }
        holes: Dict[str, List[Rect]] = {}
        for face, rect in openings:
            if face not in FACE_NAMES:
                raise ValueError(f"opening on unknown face {face!r}")
            if face in omitted:
                raise ValueError(f"opening on omitted face {face!r}")
            r = tuple(_q(float(c)) for c in rect)
            ext = spec[face][0]
            if not (r[0] < r[1] and r[2] < r[3]):
                raise ValueError(f"opening {face} {r} is empty or reversed")
            if r[0] < ext[0] or r[1] > ext[1] or r[2] < ext[2] or r[3] > ext[3]:
                raise ValueError(f"opening {face} {r} exceeds the wall extent {ext}")
            holes.setdefault(face, []).append(r)

        room = Room(lo, hi, wall)
        room._interior = interior
        for face in FACE_NAMES:
            if face in omitted:
                continue
            ext, builder = spec[face]
            pieces: List[Rect] = [ext]
            for hole in holes.get(face, []):
                nxt: List[Rect] = []
                for p in pieces:
                    nxt.extend(_cut_rect(p, hole))
                pieces = nxt
            room._pieces[face] = []
            for rect in pieces:
                bmin, bmax = builder(rect)
                s = self.box(bmin, bmax, material, tag)
                room.solids.append(s)
                room._pieces[face].append((s, rect))
        if marker:
            c = room.center()
            self.marker((c[0], c[1], lo[2] + 8.0))
        return room

    # -- displacements ----------------------------------------------------
    def displace(self, solid: Solid, side: str = "top", power: int = 3,
                 heights: Optional[Callable[[float, float], float]] = None,
                 alphas: Optional[Callable[[float, float], float]] = None,
                 material: Optional[str] = None,
                 other_material: Optional[str] = MAT_NODRAW) -> Side:
        """Turn one quad side of `solid` into a displacement surface.

        heights(u, v) -> distance to push the vertex along the side's outward
        normal.  u and v run 0..1 across the quad.  The grid frame is: start
        corner p0 = the vertex with the smallest (z, y, x); p1 = the next
        corner clockwise from outside; p3 = the previous.  v runs p0->p1, u
        runs p0->p3.  For a 'top' side this means u = fraction along +x and
        v = fraction along +y, so world x = xmin + u*(xmax-xmin).
        Keep heights at the quad border at 0 unless a neighbour closes the gap.

        power is 2..4 (5x5, 9x9, 17x17 vertices).  Remaining sides of the
        brush are re-textured `other_material` (nodraw) like Hammer users do;
        pass None to keep them.  alphas(u, v) -> 0..255 blend alpha.
        """
        if power not in (2, 3, 4):
            raise ValueError("power must be 2, 3 or 4")
        sd = solid.side_obj(side)
        if len(sd.verts) != 4:
            raise ValueError(f"side {side!r} has {len(sd.verts)} vertices; displacements need a quad")
        n = 1 << power
        verts = sd.verts
        start = min(range(4), key=lambda i: (verts[i][2], verts[i][1], verts[i][0]))
        p = [verts[(start + k) % 4] for k in range(4)]
        un = sd.unit_normal
        rows = n + 1
        zero3 = " ".join(["0 0 0"] * rows)
        data = {"power": power, "start": p[0], "normals": [], "distances": [],
                "alphas": [], "offsets": [zero3] * rows}
        for i in range(rows):
            v = i / n
            nrm_row, dist_row, alpha_row = [], [], []
            for j in range(rows):
                u = j / n
                nrm_row.append(" ".join(_fmt_f(c) for c in un))
                dist_row.append(_fmt_f(float(heights(u, v)) if heights else 0.0))
                alpha_row.append(_fmt_f(float(alphas(u, v)) if alphas else 0.0))
            data["normals"].append(" ".join(nrm_row))
            data["distances"].append(" ".join(dist_row))
            data["alphas"].append(" ".join(alpha_row))
        sd.disp = data
        if material:
            sd.material = material
        if other_material:
            for o in solid.sides:
                if o is not sd:
                    o.material = other_material
        return sd

    # -- entities ---------------------------------------------------------
    def entity(self, classname: str, origin: Optional[Sequence[float]] = None,
               keys: Optional[Dict[str, str]] = None, connections: Sequence[Tuple[str, str]] = (),
               **kw) -> Entity:
        """Point entity.  Extra keyvalues go in kwargs (or `keys` for odd names).

        connections: [("OnTrigger", "target,Input,param,delay,refire"), ...]
        """
        org = _vec(origin) if origin is not None else None
        if org is not None and org == (0.0, 0.0, 0.0):
            raise ValueError("origin 0 0 0 is ignored by VBSP's entity flood; move the entity")
        k: Dict[str, str] = {}
        for key, val in list(kw.items()) + list((keys or {}).items()):
            k[key] = _fmt(val) if isinstance(val, float) else str(val)
        ent = Entity(self._alloc(), classname, org, k)
        ent.connections = list(connections)
        self.entities.append(ent)
        return ent

    def brush_entity(self, classname: str, solids: Iterable[Union[Solid, Room]],
                     origin: Optional[Sequence[float]] = None,
                     keys: Optional[Dict[str, str]] = None,
                     connections: Sequence[Tuple[str, str]] = (), **kw) -> Entity:
        """Brush entity (func_door, trigger_teleport, func_water_analog, ...).

        The solids are removed from the world and attached to the entity.
        """
        flat: List[Solid] = []
        for s in solids:
            flat.extend(s.solids if isinstance(s, Room) else [s])
        if not flat:
            raise ValueError("brush entity needs at least one solid")
        k: Dict[str, str] = {}
        for key, val in list(kw.items()) + list((keys or {}).items()):
            k[key] = _fmt(val) if isinstance(val, float) else str(val)
        ent = Entity(self._alloc(), classname,
                     _vec(origin) if origin is not None else None, k)
        ent.connections = list(connections)
        for s in flat:
            if s.owner is not None:
                raise ValueError(f"solid {s.id} already belongs to entity {s.owner.id}")
            s.owner = ent
            ent.solids.append(s)
        self.entities.append(ent)
        return ent

    def marker(self, origin: Sequence[float], classname: str = "info_target") -> Entity:
        """A bare point entity that keeps the region around it from being filled solid."""
        return self.entity(classname, origin=origin, targetname=f"bspwalk_marker_{self._next_id}")

    # -- expectations -----------------------------------------------------
    @staticmethod
    def _side_ids(side) -> List[int]:
        if isinstance(side, Side):
            return [side.id]
        if isinstance(side, int):
            return [side]
        return [i for s in side for i in Map._side_ids(s)]

    def expect_visible(self, side, why: str) -> None:
        """Side(s) a reachable player can see.  Accepts an id, a Side, or any iterable of them."""
        for sid in self._side_ids(side):
            self._visible.append((sid, why))

    def expect_hidden(self, side, why: str) -> None:
        """Side(s) NO reachable player can see."""
        for sid in self._side_ids(side):
            self._hidden.append((sid, why))

    @staticmethod
    def _feet(point: Sequence[float]) -> Vec:
        if len(point) != 3:
            raise ValueError(f"reach point needs x, y, z: {point!r}")
        return (float(point[0]), float(point[1]), float(point[2]))

    def expect_reach(self, point: Sequence[float], why: str) -> None:
        """A feet point (on a floor, z = floor height) some player can stand on.

        Scored against the walker's reached walkable polygons (README, Scoring): the point
        must lie on a reached polygon within 0.5u in XY and 2u in z.  No 1/64 grid rule:
        the point is not written to the VMF.
        """
        self._reach.append((self._feet(point), why))

    def expect_no_reach(self, point: Sequence[float], why: str) -> None:
        """A feet point on a floor that NO player can reach (no reached polygon within tolerance)."""
        self._no_reach.append((self._feet(point), why))

    CUBEMAP_STATUSES = ("ok", "solid", "nodraw")

    def expect_cubemap(self, origin: Sequence[float], status: str, props: bool = False, why: str = "") -> None:
        """The walker's verdict on an env_cubemap: 'ok' (an eye), 'solid' (camera inside a brush) or
        'nodraw' (sees a toolsnodraw face), and whether it warns that the cubemap sits in a static prop.

        Scored against the walker JSON's cubemaps.list (README, Walker JSON contract).  VBSP stores
        a cubemap origin truncated to whole units, so use integer coordinates.
        """
        if status not in self.CUBEMAP_STATUSES:
            raise ValueError(f"cubemap status {status!r} is not one of {self.CUBEMAP_STATUSES}")
        o = self._feet(origin)
        if any(c != int(c) for c in o):
            raise ValueError(f"cubemap origin {o} is not whole units (VBSP truncates lump 42 origins)")
        self._cubemaps.append({"origin": list(o), "status": status, "props": bool(props), "why": why})

    def note(self, text: str) -> None:
        self.notes.append(text)

    # -- verifier declarations (verify_expectations.py; never written to the VMF) --
    # Where a player's eye can be, so the expectations can be checked by brute force
    # independently of any walker.  role "exact": every point is a real eye position
    # (used for must_visible and must_hidden); role "hidden": a superset, used only to
    # look for counterexamples to must_hidden (sweeps whose exact shape is awkward).
    def verify_floor(self, x0: float, y0: float, x1: float, y1: float, z: float,
                     why: str = "", role: str = "exact") -> None:
        """A reachable floor rectangle at height z: eyes from z + EYE_DUCK to z + max jump eye over it,
        and a dead player's view (z + EYE_DEAD up to the crouch eye) wherever a crouched hull fits."""
        self._verify.setdefault("floors", []).append(
            ((float(x0), float(y0), float(x1), float(y1), float(z)), role, why))

    def verify_eye_box(self, mins: Sequence[float], maxs: Sequence[float],
                       why: str = "", role: str = "exact") -> None:
        """An axis-aligned box of eye points (swimming, climbing, riding, falling)."""
        self._verify.setdefault("boxes", []).append((_fvec(mins), _fvec(maxs), role, why))

    def verify_viewpoint(self, point: Sequence[float], why: str = "") -> None:
        """A camera the map renders from (env_cubemap, point_camera): one exact eye."""
        self._verify.setdefault("viewpoints", []).append((_fvec(point), why))

    def verify_sky(self, origin: Sequence[float], scale: float) -> None:
        """3D skybox: every eye E is also an eye at origin + E / scale."""
        self._verify["sky"] = (_fvec(origin), float(scale))

    def verify_mirror(self, z: float, x0: float, y0: float, x1: float, y1: float, why: str = "") -> None:
        """A horizontal reflective surface (expensive water) facing up at height z over a rectangle."""
        self._verify.setdefault("mirrors", []).append(((float(z), float(x0), float(y0), float(x1), float(y1)), why))

    def verify_skip_solid(self, solid: "Solid") -> None:
        """A world solid that neither blocks sight nor the player (beyond the material rules)."""
        self._verify.setdefault("skip", set()).add(solid.id)

    @property
    def verify_spec(self) -> dict:
        return self._verify

    # -- output -----------------------------------------------------------
    def all_sides(self) -> Dict[int, Side]:
        return {s.id: s for solid in self.solids for s in solid.sides}

    def validate(self) -> None:
        ids = self.all_sides()
        seen: Dict[int, str] = {}
        for kind, lst in (("visible", self._visible), ("hidden", self._hidden)):
            for sid, why in lst:
                if sid not in ids:
                    raise ValueError(f"expectation names unknown side id {sid}: {why}")
                if sid in seen and seen[sid] != kind:
                    raise ValueError(f"side {sid} is expected both visible and hidden")
                seen[sid] = kind
        if not self._visible and not self._hidden:
            raise ValueError("fixture has no expectations")
        if not any(e.classname.startswith("info_player") for e in self.entities):
            raise ValueError("fixture has no info_player_* spawn entity")

    def _side_node(self, s: Side) -> KVNode:
        pts = s.plane_points()
        # The three points must reproduce the outward normal under the VBSP convention.
        chk = _cross(_sub(pts[0], pts[1]), _sub(pts[2], pts[1]))
        if _dot(chk, s.normal) <= 0:
            raise AssertionError(f"side {s.id}: plane points wound the wrong way")
        u, v = _tex_axes(s.normal)
        node = KVNode("side", [
            KVPair("id", str(s.id)),
            KVPair("plane", " ".join(f"({_fmt_vec(q)})" for q in pts)),
            KVPair("material", s.material),
            KVPair("uaxis", u),
            KVPair("vaxis", v),
            KVPair("rotation", "0"),
            KVPair("lightmapscale", str(s.lightmapscale)),
            KVPair("smoothing_groups", "0"),
        ])
        if s.disp is not None:
            d = s.disp
            rows = (1 << d["power"]) + 1

            def block(name, key):
                return KVNode(name, [KVPair(f"row{i}", d[key][i]) for i in range(rows)])
            nrm = block("normals", "normals")
            tags = KVNode("triangle_tags", [
                KVPair(f"row{i}", " ".join(["9"] * (2 * (rows - 1)))) for i in range(rows - 1)])
            node.children.append(KVNode("dispinfo", [
                KVPair("power", str(d["power"])),
                KVPair("startposition", f"[{_fmt_vec(d['start'])}]"),
                KVPair("flags", "0"),
                KVPair("elevation", "0"),
                KVPair("subdiv", "0"),
                nrm,
                block("distances", "distances"),
                block("offsets", "offsets"),
                KVNode("offset_normals", [KVPair(f"row{i}", d["normals"][i]) for i in range(rows)]),
                block("alphas", "alphas"),
                tags,
                KVNode("allowed_verts", [KVPair("10", " ".join(["-1"] * 10))]),
            ]))
        return node

    def _solid_node(self, solid: Solid) -> KVNode:
        node = KVNode("solid", [KVPair("id", str(solid.id))])
        for s in solid.sides:
            node.children.append(self._side_node(s))
        node.children.append(_editor("0 104 249"))
        return node

    def to_kv(self) -> KVNode:
        """The whole VMF as a KVNode tree (serialise with vmf_parser.VMFWriter)."""
        root = KVNode("__root__")
        root.children.append(KVNode("versioninfo", [
            KVPair("editorversion", "400"), KVPair("editorbuild", "8870"),
            KVPair("mapversion", "1"), KVPair("formatversion", "100"), KVPair("prefab", "0")]))
        root.children.append(KVNode("visgroups"))
        root.children.append(KVNode("viewsettings", [
            KVPair("bSnapToGrid", "1"), KVPair("bShowGrid", "1"),
            KVPair("bShowLogicalGrid", "0"), KVPair("nGridSpacing", "16")]))
        world = KVNode("world", [
            KVPair("id", "1"), KVPair("mapversion", "1"), KVPair("classname", "worldspawn"),
            KVPair("detailmaterial", "detail/detailsprites"), KVPair("detailvbsp", "detail.vbsp"),
            KVPair("maxpropscreenwidth", "-1"), KVPair("skyname", self.skyname)])
        for solid in self.solids:
            if solid.owner is None:
                world.children.append(self._solid_node(solid))
        root.children.append(world)
        for ent in self.entities:
            node = KVNode("entity", [KVPair("id", str(ent.id)), KVPair("classname", ent.classname)])
            for k, v in ent.keys.items():
                node.children.append(KVPair(k, v))
            if ent.origin is not None:
                node.children.append(KVPair("origin", _fmt_vec(ent.origin)))
            if ent.connections:
                node.children.append(KVNode("connections", [KVPair(k, v) for k, v in ent.connections]))
            for solid in ent.solids:
                node.children.append(self._solid_node(solid))
            node.children.append(_editor("0 255 0" if not ent.solids else "220 30 220"))
            root.children.append(node)
        root.children.append(KVNode("cameras", [KVPair("activecamera", "-1")]))
        root.children.append(KVNode("cordons", [KVPair("active", "0")]))
        return root

    def expect_dict(self) -> dict:
        def dedup(lst):
            seen, out = set(), []
            for sid, why in lst:
                if sid not in seen:
                    seen.add(sid)
                    out.append({"side": sid, "why": why})
            return out
        out = {
            "fixture": self.name,
            "description": self.description,
            "must_visible": dedup(self._visible),
            "must_hidden": dedup(self._hidden),
            "must_reach": [{"point": list(p), "why": why} for p, why in self._reach],
            "must_not_reach": [{"point": list(p), "why": why} for p, why in self._no_reach],
            "notes": "\n".join(self.notes),
        }
        if self._cubemaps:                       # only when declared: other fixtures' files stay as they are
            out["cubemaps"] = [dict(c) for c in self._cubemaps]
        return out

    def write(self, vmf_path, expect_path=None) -> None:
        """Validate, then write the VMF and (optionally) the expect.json sidecar."""
        self.validate()
        VMFWriter().write_file(self.to_kv(), vmf_path)
        if expect_path is not None:
            text = json.dumps(self.expect_dict(), indent=2) + "\n"
            with open(expect_path, "w", encoding="utf-8", newline="\n") as f:
                f.write(text)


def _editor(color: str) -> KVNode:
    return KVNode("editor", [KVPair("color", color), KVPair("visgroupshown", "1"),
                             KVPair("visgroupautoshown", "1")])
