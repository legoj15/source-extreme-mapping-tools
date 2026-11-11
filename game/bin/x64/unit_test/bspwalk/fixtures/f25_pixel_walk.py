"""
f25 -- a platform reached only along a ledge too narrow to stand on (a pixel walk, #87), and a
crouch tunnel whose floor is crossed by a 0.5u trim strip that must still be walked.

A pixel walk is standing or walking where the player box overhangs a ledge by less than a
player can really stand on.  bspwalk models intended movement by default: a walk node narrower
than min_ledge_width (1u), whose hull with its walk-edge neighbours is that narrow too, and on
which the box nowhere overlaps the node's own surface by 1u, is never reached (bw_walk.cpp
MarkNarrowNodes).  f02's 32.5u corridor checks the last part: its 0.5u feet strip lies on a wide
floor and stays walkable.  With -exploits it is walkable.  The main run passes
-exploits (m.walker_args); the harness's second run without it checks the default.

Geometry (x east, y north, z up; interior bounds, 16u walls):
    H  hall             x 0..1024   y 0..640    z 0..384    info_player_start (128,128,1)
    ledge steps         seven 0.5u-deep plates on the hall's north wall (y 639.5..640), each 32u
                        long and 8u thick: step k at x 256+32k..288+32k, top z 48+16k (48..144)
    P  platform         x 480..672  y 448..640  top z 160, against the north wall, east of step 6
    T  tunnel           x 1056..1360 y 64..128  z 0..40, entered through a 64 x 40 hole in the
                        hall's east wall; floor of three brushes: A x 1040..1200, a 0.5u trim
                        x 1200..1200.5, B x 1200.5..1376, all top z 0

The ledge.  A step's feet polygon is its top grown 16u each way, minus the wall's grown hull:
y 623.5..624.03, 0.53u wide.  The player's box overhangs the step by at most 0.5u.  Step 0
(z 48) is a duck-jump from the floor (57u), each next step a 16u step up (18u), and P (z 160) a
16u step up from step 6.  Nothing else reaches P: from the floor it is 160u up, and every step
above step 0 is more than a duck-jump (57u) above the floor.  Step 6 touches P, so its hull
with P is wide and it stays a node (a fringe of P); steps 0 to 5 have only each other as walk
neighbours and are pixel walks.

The trim.  The three tunnel floor brushes are coplanar; their grown tops overlap and the lower
convex index keeps the overlap (bw_walk.cpp, dedupe), which leaves the trim a 0.5u-wide node
between A's and B's.  The tunnel is 40u tall: a crouched player (36u) fits, a jump (21u more)
does not, so no arc crosses the strip and B is reached only by walking over it.  The trim node
is thinner than min_ledge_width but its hull with A and B is wide: it must stay walkable.

Expectations (main run, -exploits):
  reach    the hall floor; a middle step (a pixel walk); P's top; B's far end.
  visible  P's top: it faces up and is 160u above the floor, where every eye is at most 85u up;
           an eye on P (or on the ledge) sees it.
Default run (no -exploits):
  reach    B's far end (across the trim).
  no reach the middle step and P's top.
"""
import vmfgen

NAME = "f25_pixel_walk"
DESCRIPTION = ("A platform reached only along a 0.5u ledge (a pixel walk, -exploits only), and a crouch tunnel "
               "whose floor is crossed by a 0.5u trim strip that the default walk must still cross.")

LEDGE_Y = 639.5                         # the plates' south face; the north wall's inner face is y 640
STEP_X0, STEP_LEN, STEP_Z0, STEP_RISE, STEPS = 256.0, 32.0, 48.0, 16.0, 7
P_X0, P_X1, P_Y0, P_TOP = 480.0, 672.0, 448.0, 160.0
T_X0, T_X1, T_Y0, T_Y1, T_TOP = 1056.0, 1360.0, 64.0, 128.0, 40.0
TRIM_X0, TRIM_X1 = 1200.0, 1200.5


def build(m: vmfgen.Map) -> None:
    depth = 640.0 - LEDGE_Y
    assert depth + 1 / 32 < vmfgen.MIN_LEDGE_WIDTH        # feet polygon: the depth plus the 1/32u obstacle shrink
    assert TRIM_X1 - TRIM_X0 < vmfgen.MIN_LEDGE_WIDTH                          # and so is the trim, alone
    assert STEP_Z0 < vmfgen.CROUCH_JUMP_FEET - 5                               # step 0: a duck-jump from the floor
    assert STEP_Z0 + STEP_RISE > vmfgen.CROUCH_JUMP_FEET + 5                   # step 1 already is not
    assert STEP_RISE < vmfgen.STEP_HEIGHT                                      # each step a walk up
    top6 = STEP_Z0 + STEP_RISE * (STEPS - 1)
    assert P_TOP - top6 == STEP_RISE and STEP_X0 + STEP_LEN * STEPS == P_X0    # step 6 meets P
    assert P_TOP > vmfgen.CROUCH_JUMP_FEET * 1.15 + 80                         # P: no jump from the floor
    assert P_TOP > vmfgen.MAX_EYE_ABOVE_FLOOR + 50                             # P's top above every floor eye
    assert vmfgen.HULL_DUCK[2] < T_TOP < vmfgen.HULL_DUCK[2] + vmfgen.STAND_JUMP_FEET   # crouch fits, no jump

    hall = m.room((0, 0, 0), (1024, 640, 384), openings=[("east", (T_Y0, T_Y1, 0, T_TOP))])
    m.entity("info_player_start", origin=(128, 128, 1), angles="0 0 0")
    m.walker_args("-exploits")

    for k in range(STEPS):
        x0, z = STEP_X0 + STEP_LEN * k, STEP_Z0 + STEP_RISE * k
        m.box((x0, LEDGE_Y, z - 8), (x0 + STEP_LEN, 640, z))
    plat = m.box((P_X0, P_Y0, 0), (P_X1, 640, P_TOP))

    # the tunnel: no floor of its own; A, the trim and B in that order (convex index order)
    m.room((T_X0, T_Y0, 0), (T_X1, T_Y1, T_TOP), floor=False, openings=[("west", (T_Y0, T_Y1, 0, T_TOP))])
    for x0, x1 in ((1040.0, TRIM_X0), (TRIM_X0, TRIM_X1), (TRIM_X1, T_X1 + 16)):
        m.box((x0, T_Y0 - 16, -16), (x1, T_Y1 + 16, 0))

    mid = 3
    mid_pt = (STEP_X0 + STEP_LEN * mid, 640.0 - 16 - depth / 2, STEP_Z0 + STEP_RISE * mid)
    b_far = (T_X1 - 40, (T_Y0 + T_Y1) / 2, 0)
    m.expect_visible(plat.side("top"), "P's top: only an eye on P or on the ledge (160u up) sees it")
    m.expect_reach((128, 300, 0), "hall floor at the spawn")
    m.expect_reach(mid_pt, "ledge step 3: a pixel walk, standable with -exploits")
    m.expect_reach((576, 520, P_TOP), "P's top: along the ledge with -exploits")
    m.expect_reach(b_far, "tunnel floor B: crouch-walk over the trim")
    m.expect_no_reach_default(mid_pt, "ledge step 3: 0.53u wide, a pixel walk")
    m.expect_no_reach_default((576, 520, P_TOP), "P's top: reached only along the ledge")
    m.expect_reach_default(b_far, "tunnel floor B: the 0.5u trim between A and B is not a pixel walk")
    m.note(f"Ledge {depth:g}u deep, steps z {STEP_Z0:g}..{top6:g}, P top {P_TOP:g}; trim {TRIM_X1 - TRIM_X0:g}u "
           f"in a {T_TOP:g}u tunnel.")

    # verify_expectations.py (#19); the tunnel is crouch-only and holds no claim
    m.verify_floor(0, 0, 1024, 640, 0, "hall floor")
    m.verify_floor(P_X0, P_Y0, P_X1, 640, P_TOP, "platform P (with -exploits)")
