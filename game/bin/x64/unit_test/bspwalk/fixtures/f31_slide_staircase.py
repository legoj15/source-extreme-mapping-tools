"""
f31 -- a platform reached only by climbing a staircase of steep wedges, each step of which is
an uphill slide climb (#77), and a control point on the hall floor the default walk still
reaches.

A steep face (0.1 <= n.z < 0.7) is a slide: a standing player slides down it and cannot jump
off it.  A slide node counts as reached at every height it covers, even when the player only
got onto its bottom edge; EDGE_SLIDE edges then let the flood step sideways into the next
slide whose bottom sits at the previous one's top, so a chain of them climbs with no speed at
all (d1_trainstation_02 climbs props 2253-2257 from z 170 to 381 this way).  The default run
now tracks each slide's slide top, the highest feet height the player can actually be at on it
(bw_walk.cpp, WalkGraph::slideTop): a move out of a slide reaches only what is within a step of
that height, and a slide's walk-off arcs and fall launches start no higher than it.  A slide
chain carries no rise, so the staircase is a wall to the default run.  With -exploits the old
behaviour stands (SetSlideClimb).

Geometry (x east, y north, z up; interior bounds, 16u walls):
    H  hall         x 0..640   y 0..768   z 0..512   info_player_start (96, 96, 1)
    wedges          four, each x 64..96 (32u run) and 128u wide in y, rising 64u per step
                   (slope 2, n.z 0.447, 63 degrees), bands y 192..320, 320..448, 448..576,
                   576..704, sloping east-up / west-up by turns:
                       w0  y 192..320  z 0..64    high east   (its low west edge lies on the floor)
                       w1  y 320..448  z 64..128  high west   (bottom east, at w0's top)
                       w2  y 448..576  z 128..192 high east   (bottom west, at w1's top)
                       w3  y 576..704  z 192..256 high west   (bottom east, at w2's top)
    P  platform     x 0..320   y 704..768 (against the north wall), top z 256 = w3's top.

Each pair touches along its shared y line, and there the two planes meet at one end (dz runs
from 2 * rise at the far end down to 0 at the common end), so a walk-height band (|dz| <= 18)
straddles the corner and the edge pass makes the EDGE_SLIDE.  The crossing at every contact is
along +-y, the contour direction of a wedge sloping in x, so SlideExitAllowed keeps every step
(bw_walk.cpp).  With -exploits the flood walks floor -> w0 -> w1 -> w2 -> w3 -> P.  In the
default run the floor puts the player at most a step (18u) onto w0, an arc from the floor lands
at most max_jump_rise (57u) up it, and slide to slide carries no rise: nothing puts the player
above 57u on any wedge, every next bottom (128u up and more) is out of reach, and P (256u up)
has no other route: it is past every jump from the floor and past every eye over it.

Expectations (main run, -exploits):
  reach    the hall floor; P's top.
  visible  P's top (256u above the floor, past every floor eye; an eye on P sees it) and P's
           south face over the hall floor (control).
Default run (no -exploits):
  reach    the hall floor.
  no reach P's top and w2's mid slope (the chain is broken at its first step).
"""
import vmfgen

NAME = "f31_slide_staircase"
DESCRIPTION = ("A platform reached only through an uphill staircase of steep wedges, each step a slide-climb "
               "exploit (#77, -exploits only); the default run keeps the hall floor but cannot gain height "
               "along the slides.")

WEDGE_X0, WEDGE_X1, RISE, WEDGES = 64.0, 96.0, 64.0, 4
WEDGE_Y0, WEDGE_W = 192.0, 128.0
P_X1, P_TOP = 320.0, RISE * WEDGES


def build(m: vmfgen.Map) -> None:
    run = WEDGE_X1 - WEDGE_X0
    nz = run / (run ** 2 + RISE ** 2) ** 0.5
    assert 0.1 <= nz < 0.5                                            # a slide well past walkable (63 degrees)
    assert run >= vmfgen.HULL_DUCK[0] and WEDGE_W >= vmfgen.HULL_DUCK[1]   # a player fits along and across it
    assert RISE > vmfgen.STEP_HEIGHT                                   # no step of the chain is a walk up
    # the default run's best hold on a wedge is a jump rise above the floor (an arc landing,
    # max_jump_rise) plus one step onto the next; the wedge past that must clear it with room
    # for the 16u the node polygons grow past each brush (slope 2)
    assert 2 * RISE > vmfgen.CROUCH_JUMP_FEET + vmfgen.STEP_HEIGHT + 2 * 16
    assert P_TOP > vmfgen.CROUCH_JUMP_FEET * 1.15 + 80                 # P: no jump from the floor
    assert P_TOP > vmfgen.MAX_EYE_ABOVE_FLOOR + 50                     # P's top above every floor eye

    m.room((0, 0, 0), (640, 768, 512))
    m.entity("info_player_start", origin=(96, 96, 1), angles="0 0 0")
    m.walker_args("-exploits")                                         # the slide climb is an exploit (#77)

    high = "east"
    for k in range(WEDGES):
        y0 = WEDGE_Y0 + WEDGE_W * k
        m.wedge((WEDGE_X0, y0, RISE * k), (WEDGE_X1, y0 + WEDGE_W, RISE * (k + 1)), high=high)
        high = "west" if high == "east" else "east"
    plat = m.box((0, WEDGE_Y0 + WEDGE_W * WEDGES, 0), (P_X1, 768, P_TOP))

    floor_pt = (128, 128, 0)
    top_pt = (160, 736, P_TOP)
    w2_pt = (WEDGE_X0 + run / 2, WEDGE_Y0 + WEDGE_W * 2.5, RISE * 2 + RISE / 2)
    m.expect_visible(plat.side("top"), "P's top: 256u above the floor, only an eye up there (reached with -exploits) sees it")
    m.expect_visible(plat.side("south"), "control: P's face over the hall floor")
    m.expect_reach(floor_pt, "hall floor at the spawn")
    m.expect_reach(top_pt, "P's top: up the slide staircase with -exploits")
    m.expect_reach_default(floor_pt, "hall floor: slides do not block the walk")
    m.expect_no_reach_default(top_pt, "P's top: a slide chain carries no rise (#77)")
    m.expect_no_reach_default(w2_pt, "wedge 2 mid slope: the chain is broken at its first step")
    m.note(f"Wedges {WEDGES} x rise {RISE:g}u over a {run:g}u run (n.z {nz:.3f}, 63 degrees), {WEDGE_W:g}u wide; "
           f"P top {P_TOP:g}u over the floor.")

    # verify_expectations.py (#19); the wedges are slides, their surfaces hold no floor claim
    m.verify_floor(0, 0, 640, 768, 0, "hall floor")
    m.verify_floor(0, WEDGE_Y0 + WEDGE_W * WEDGES, P_X1, 768, P_TOP, "platform P (with -exploits)")
