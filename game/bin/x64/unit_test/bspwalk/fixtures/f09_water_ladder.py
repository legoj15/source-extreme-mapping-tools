"""
f09 -- two mechanics, each with a control: a room reachable only through a flooded passage, and a
ledge reachable only by a ladder.  Two sealed chambers, each with its own spawn (a walker seeds every
info_player_*).

Chamber 1 -- the flooded passage.  The only difference between the wet cell and the dry cell is the
water, so any difference in result is the water.

    x:   0 ........ 512 | 528 ........ 1040 | 1056 ....... 1568      (y 0..512, floor z=0)
        room W (wet)      hall H (spawn)       room D (dry)
        pit B_w --tunnel-- pit A_w   spawn    pit A_d --tunnel-- pit B_d
        hut_w                                                       hut_d

    Each pit is 128 x 128 and 128u deep (floor z=-128, open at the top to the room's floor z=0).
    A tunnel 64u long, 64u wide and 96u high (z -128..-32) joins pit A (in the hall, 16u from the
    dividing wall) to pit B (in the far room), passing under the 16u dividing wall; the lintel over it
    is z -32..0.  Wet cell: all three cavities are filled with water to z=0 (flush with the floor).
    Dry cell: the same cavities, no water.

    Wet route: step into pit A_w, swim down 128u, through the tunnel (fully submerged, 96u high), up
    pit B_w, and walk out at the surface, flush with room W's floor.  Dry route: fall 128u into pit
    A_d (no fall damage: 392 u/s, damage starts at 580), walk the tunnel, and be at the bottom of
    pit B_d, whose walls are 128u: a duck-jump climbs 57u (65.6u with the walker's margin).  So pit B_d
    is a dead end and room D is never entered.  Nothing else leads out of the hall: the dividing walls
    run floor to ceiling across the full width.

    hut_w / hut_d are mirror images (door facing the far wall of their room), closed boxes on the
    floor, interior 96 x 160 x 112, door 64 wide x 80 high, roof top z=128.

Chamber 2 -- the ladder (y 704..1216; x 0..1536; ceiling z=384).  Same plan as f03 with 160u ledges:

    ledge L1  x 0..448     top z=160   hut1 door faces WEST   func_useableladder on its EAST face
    ledge L2  x 1088..1536 top z=160   hut2 door faces EAST   no ladder
    spawn (info_player_deathmatch) at (768, y0+256, 1) on the floor between them.

    The ladder is a func_useableladder: point0 (top) = (468, y0+256, 160), point1 (bottom) =
    (468, y0+256, 0), i.e. the centre line of the climb 20u in front of L1's east face (hull is 16
    half-width), from the floor to the ledge top.  The entity carries origin "0 0 0" on purpose:
    Hammer writes point0/point1 as absolute world positions, and with a zero origin an
    absolute reading and an origin-relative reading give the same ladder.  There is no
    info_ladder_dismount: the ladder alone must connect the floor to the ledge.

Why each expectation is certain:
  visible  hut_w inner sides: reaching room W needs the swim; once there the player walks round the
           hut to its door.  (Eyes in W see the walls, so the hut interior is seen from the door.)
  visible  tunnel ceilings (both cells): a submerged swimmer (wet) / a walker (dry) is in the tunnel and
           sees the underside of the lintel.  The dry one needs a 128u fall to be modelled.
  visible  hut1 inner sides: climb the ladder, walk round to the west door.
  visible  controls: L1 east face and L2 west face, seen from the spawn; room 2 ceiling.
  hidden   hut_d inner sides: room D is not entered, and the only reachable eyes in it are at the
           bottom of pit B_d (x <= 1200, z <= -43).  The hut interior is x 1344..1440 with its door
           in the east wall (x 1440..1456).  Any segment from an eye with x <= 1200 to a point in
           the hut stays in x <= 1440, so it cannot pass the doorway (x >= 1440) and must cross
           the hut shell (west/north/south walls or roof; the floor below the hut is solid slab, no
           cavity is under it).  The eyes in the hall and the wet cell are separated from D by solid
           and by the same plane argument (x < 1344).
  hidden   hut2 inner sides: L2's west face is 620u from the ladder, and a jump off the ladder or the
           ledge reaches ~372u even at 2x sprint speed.  Every reachable eye has x <= 1088 - 16 or x <= 468
           while hut2's doorway is at x >= 1312, so the same plane argument applies.
  hidden   hut_w and hut_d roof tops (z=128), hut1/hut2 roof tops (z=288): above every eye in their
           room (85 over the floor; 160 + 85 = 245 on the ledge).
"""
from typing import List, Sequence, Tuple

import vmfgen

NAME = "f09_water_ladder"
DESCRIPTION = ("Room reachable only through a flooded passage (dry twin is a trap) and a ledge "
               "reachable only by a func_useableladder (ladderless twin is unreachable).")

WATER = "NATURE/WATER_DX70"      # %compilewater, no reflection/refraction (a plain translucent water material)

Box = Tuple[Tuple[float, float, float], Tuple[float, float, float]]


def _subtract(box: Box, hole: Box) -> List[Box]:
    """box minus hole as disjoint axis-aligned boxes (hole may stick out of box)."""
    (bx0, by0, bz0), (bx1, by1, bz1) = box
    (hx0, hy0, hz0), (hx1, hy1, hz1) = hole
    ix0, iy0, iz0 = max(bx0, hx0), max(by0, hy0), max(bz0, hz0)
    ix1, iy1, iz1 = min(bx1, hx1), min(by1, hy1), min(bz1, hz1)
    if ix0 >= ix1 or iy0 >= iy1 or iz0 >= iz1:
        return [box]
    out: List[Box] = []
    if bx0 < ix0:
        out.append(((bx0, by0, bz0), (ix0, by1, bz1)))
    if ix1 < bx1:
        out.append(((ix1, by0, bz0), (bx1, by1, bz1)))
    if by0 < iy0:
        out.append(((ix0, by0, bz0), (ix1, iy0, bz1)))
    if iy1 < by1:
        out.append(((ix0, iy1, bz0), (ix1, by1, bz1)))
    if bz0 < iz0:
        out.append(((ix0, iy0, bz0), (ix1, iy1, iz0)))
    if iz1 < bz1:
        out.append(((ix0, iy0, iz1), (ix1, iy1, bz1)))
    return out


def _carve(m: vmfgen.Map, block: Box, holes: Sequence[Box]) -> List[vmfgen.Solid]:
    """A solid slab with cavities (vmfgen has no boolean subtraction): returns the boxes that tile it."""
    pieces: List[Box] = [block]
    for hole in holes:
        nxt: List[Box] = []
        for p in pieces:
            nxt.extend(_subtract(p, hole))
        pieces = nxt
    return [m.box(lo, hi) for lo, hi in pieces]


def _water_box(m: vmfgen.Map, lo, hi) -> vmfgen.Solid:
    # Every side carries the water material: a water volume gets its contents from its textures, and
    # the sides against solid or other water are culled by VBSP anyway.
    return m.box(lo, hi, material=WATER)


def build(m: vmfgen.Map) -> None:
    # ---- reach margins (see docstring) -------------------------------------------------------
    pit_depth = 128.0
    assert pit_depth > vmfgen.CROUCH_JUMP_FEET * 1.15 + 40          # dry pit B_d cannot be climbed
    assert pit_depth < 580.0 ** 2 / (2 * vmfgen.SV_GRAVITY)           # fall speed under the damage threshold
    ledge_top = 160.0
    assert ledge_top > vmfgen.CROUCH_JUMP_FEET * 1.15 + 40           # nothing climbs the ledges without help
    assert 1088.0 - 468.0 > vmfgen.jump_gap_reach(2 * vmfgen.SPRINT_SPEED) * 1.15   # L2 out of jump reach
    assert 128.0 > vmfgen.MAX_EYE_ABOVE_FLOOR * 1.15 + 20             # hut roof top above every eye
    assert 288.0 > ledge_top + vmfgen.MAX_EYE_ABOVE_FLOOR * 1.15 + 20

    # ======================= chamber 1: the flooded passage ==============================
    m.room((0, 0, -128), (1568, 512, 256), floor=False)             # shell; the carved slab below is its floor
    m.entity("info_player_start", origin=(784, 256, 1), angles="0 0 0")

    pit_a_w: Box = ((544, 192, -128), (672, 320, 0))
    tun_w: Box = ((480, 224, -128), (544, 288, -32))
    pit_b_w: Box = ((352, 192, -128), (480, 320, 0))
    pit_a_d: Box = ((896, 192, -128), (1024, 320, 0))
    tun_d: Box = ((1024, 224, -128), (1088, 288, -32))
    pit_b_d: Box = ((1088, 192, -128), (1216, 320, 0))
    slab = _carve(m, ((0, 0, -144), (1568, 512, 0)),
                  [pit_a_w, tun_w, pit_b_w, pit_a_d, tun_d, pit_b_d])

    m.box((512, 0, 0), (528, 512, 256))         # dividing wall hall | W
    m.box((1040, 0, 0), (1056, 512, 256))       # dividing wall hall | D

    for lo, hi in (pit_a_w, tun_w, pit_b_w):    # the wet cell is flooded to z=0; the dry cell is not
        _water_box(m, lo, hi)

    hut_w = m.room((128, 176, 0), (224, 336, 112), floor=False,
                   openings=[("west", (224, 288, 0, 80))])
    hut_d = m.room((1344, 176, 0), (1440, 336, 112), floor=False,
                   openings=[("east", (224, 288, 0, 80))])

    # Lintel pieces over the tunnels: the side facing down into the tunnel is the tunnel ceiling.
    def tunnel_ceilings(tun: Box) -> List[int]:
        (tx0, ty0, _), (tx1, ty1, tz1) = tun
        ids = []
        for s in slab:
            lo, hi = s.mins, s.maxs
            if abs(lo[2] - tz1) < 1e-6 and lo[0] < tx1 and hi[0] > tx0 and lo[1] < ty1 and hi[1] > ty0:
                ids.append(s.side("bottom"))
        assert ids, "no lintel found over the tunnel"
        return ids

    m.expect_visible(hut_w.all_inner(),
                     "hut_w inner side: room W is reached by swimming the flooded passage")
    m.expect_visible(tunnel_ceilings(tun_w), "wet tunnel ceiling: a swimmer is inside the tunnel")
    m.expect_visible(tunnel_ceilings(tun_d), "dry tunnel ceiling: a player who fell into pit A_d walks the tunnel")
    m.expect_hidden(hut_d.all_inner(),
                    "hut_d inner side: room D is never entered (pit B_d is a 128u dead end) and the doorway faces away")
    m.expect_hidden(hut_w.outer("ceiling"), "hut_w roof top z=128: above every eye (max 85)")
    m.expect_hidden(hut_d.outer("ceiling"), "hut_d roof top z=128: above every eye (max 85)")

    # ======================= chamber 2: the ladder ======================================
    y0 = 704.0
    room2 = m.room((0, y0, 0), (1536, y0 + 512, 384))
    m.entity("info_player_deathmatch", origin=(768, y0 + 256, 1), angles="0 0 0")
    ledge1 = m.box((0, y0, 0), (448, y0 + 512, ledge_top))
    ledge2 = m.box((1088, y0, 0), (1536, y0 + 512, ledge_top))
    hut1 = m.room((144, y0 + 176, ledge_top), (240, y0 + 336, ledge_top + 112), floor=False,
                  openings=[("west", (y0 + 224, y0 + 288, ledge_top, ledge_top + 80))])
    hut2 = m.room((1216, y0 + 176, ledge_top), (1312, y0 + 336, ledge_top + 112), floor=False,
                  openings=[("east", (y0 + 224, y0 + 288, ledge_top, ledge_top + 80))])
    ladder_x, ladder_y = 448.0 + 20.0, y0 + 256.0
    m.entity("func_useableladder", origin=None,
             keys={"origin": "0 0 0",
                   "point0": f"{ladder_x:g} {ladder_y:g} {ledge_top:g}",
                   "point1": f"{ladder_x:g} {ladder_y:g} 0"})

    m.expect_visible(hut1.all_inner(), "hut1 inner side: the ledge is reached by the ladder, the doorway faces away")
    m.expect_visible(ledge1.side("east"), "control: ledge face the ladder climbs, seen from the spawn")
    m.expect_visible(ledge2.side("west"), "control: face of the ladderless ledge, seen from the spawn")
    m.expect_visible(room2.inner("ceiling"), "control: ceiling of the ladder chamber")
    m.expect_hidden(hut2.all_inner(), "hut2 inner side: ledge top at 160u has no ladder, and the doorway faces away")
    m.expect_hidden(hut1.outer("ceiling"), "hut1 roof top z=288: above every eye (max 160 + 85)")
    m.expect_hidden(hut2.outer("ceiling"), "hut2 roof top z=288: above every eye")

    m.note("Chamber 1 cells are identical except for the water. Ladder: func_useableladder point0 (top) / point1 "
           "(bottom) are absolute world positions, origin 0 0 0; no info_ladder_dismount.")
