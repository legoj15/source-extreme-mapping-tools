"""
f14 -- a ledge reachable only by riding a func_tracktrain, a ledge reachable only through a
trigger_push, and a ledge of the same height that has neither.

Three separate sealed chambers, each with its own spawn (a walker seeds every info_player_*).  They are
separate so that the train, the push arcs and the control cannot interact: a push launch or a long
jump from a rider could otherwise reach a control ledge in the same hall.  Every ledge top is 160u.
Every ledge carries a closed hut (interior 96 x 160 x 112, door 64 wide x 80 high, roof top 288) built
on it with `m.room(..., floor=False)`, exactly the f03/f04 trick.

Chamber T (train), y 0..512, x 0..1280, ceiling z=448, spawn (512, 64, 1) clear of the train's path:

    x: 0 .... 192-320 ...................... 760-888 | 896 ....... 1280
              train at A                    train at C | ledge E (top z=160)
              top z=12                      top z=160  | hut E, door faces EAST

    One func_tracktrain: a 128 x 128 x 12 slab, `origin` = its centre, height 0, orientationtype 0
    (never rotates), spawnflags 18 (no user control, fixed orientation), speed 100.  Two path_track
    nodes, both orientationtype 0, forming a loop A -> C -> A:
        A (256, 256,   6): slab x 192..320, y 192..320, z 0..12      rests on the floor; top 12u is a
                           12u step from the floor (<= 18), so it can be boarded here
        C (824, 256, 154): slab x 760..888, y 192..320, z 148..160   its top is level with ledge E's top
                           and 8u from its west face (x=896)
    The slab travels the straight line A -> C (about 587u, tilt-free).  It never touches ledge E: its
    east edge is at x = centre + 64 <= 888 < 896.  Walking from C's top to the ledge is an 8u gap.
    The ledge is 160u above the floor: a duck-jump reaches 57u (65.6u with margin), so E is reachable
    only by the train.  This is the "tops are floors" rule: the slab's upward face is walkable at
    every pose, and consecutive poses are joined by the ride.

Chamber P (push), y 704..1216, x 0..1024, ceiling z=448, spawn (256, y0+256, 1):

    ledge P x 640..1024 (top 160), hut P door faces EAST, roof top 288
    trigger_push column x 560..640, y0+192..y0+320, z 0..224, pushdir "-90 0 0" (straight up),
    speed 600, spawnflags 1 (clients).  The column touches ledge P's west face.

    Whichever way the push is applied it reaches the ledge: as a continuous upward velocity of 600 u/s
    while inside (the player rises 160u in 0.27 s and is above the ledge top while still in the 224u
    column, then steps east), or as a launch velocity (apex 600^2 / (2 * 600) = 300u at gravity
    600, 225u at 800, both above 160 + margin).  Without the push the ledge is 160u away.  The roof top
    is NOT asserted hidden here: a launched player's eye can be 300 + 64 above the floor.

Chamber C (control), y 1408..1920, x 0..1024, ceiling z=448, spawn (768, y0+256, 1):

    ledge C x 0..448 (top 160), hut C door faces WEST (x 128..144), no mechanism.
    Same plane argument as f03's high ledge: every reachable eye is at x >= 464 while the hut's only
    aperture is at x <= 144.

Why each expectation is certain:
  visible  hut E inner sides: the train carries the player to pose C, 8u from the ledge; the player
           walks onto the ledge and round the hut to its east door.
  visible  hut P inner sides: the push lifts the player over the 160u ledge; same walk.
  visible  controls: ledge E west face, ledge P west face and ledge C east face (seen from the floors,
           the trigger and the train are not solid world geometry), the three ceilings.
  hidden   hut C inner sides: nothing lifts the player to 160u (duck-jump 57u), and the shell argument:
           every segment from an eye at x >= 464 into the hut, whose only doorway is at x <= 144,
           crosses a wall.
  hidden   hut E roof top (z=288) and hut C roof top: the highest eye in chamber T or C is 160 + 85 = 245
           (ledge, or a rider at pose C); the roof is 288.
"""
import vmfgen

NAME = "f14_train_and_push"
DESCRIPTION = ("Ledge reachable only by riding a func_tracktrain, ledge reachable only via a trigger_push, "
               "and an equally high ledge with neither; each carries a hut.")

TRIGGER = "TOOLS/TOOLSTRIGGER"
LEDGE = 160.0
HUT_H = 112.0


def _ledge_hut(m: vmfgen.Map, x0: float, y0: float, door: str) -> vmfgen.Room:
    """Closed hut interior x x0..x0+96, y y0+176..y0+336, z LEDGE..LEDGE+112, door in the `door` wall."""
    ymid_lo, ymid_hi = y0 + 224, y0 + 288
    return m.room((x0, y0 + 176, LEDGE), (x0 + 96, y0 + 336, LEDGE + HUT_H), floor=False,
                  openings=[(door, (ymid_lo, ymid_hi, LEDGE, LEDGE + 80))])


def build(m: vmfgen.Map) -> None:
    cj = vmfgen.CROUCH_JUMP_FEET
    assert LEDGE > cj * 1.15 + 40                                    # unreachable without help
    assert LEDGE + HUT_H + 16 > LEDGE + vmfgen.MAX_EYE_ABOVE_FLOOR * 1.15 + 20     # roof tops above ledge eyes
    assert 600.0 ** 2 / (2 * 800.0) > LEDGE + 40                      # push apex clears the ledge even at gravity 800
    train_top_a, train_top_c = 12.0, LEDGE
    assert train_top_a <= vmfgen.STEP_HEIGHT - 4                      # boardable from the floor at A

    # ====================== chamber T: the train ===================================
    y0 = 0.0
    room_t = m.room((0, y0, 0), (1280, y0 + 512, 448))
    m.entity("info_player_start", origin=(512, y0 + 64, 1), angles="0 0 0")
    ledge_e = m.box((896, y0, 0), (1280, y0 + 512, LEDGE))
    hut_e = _ledge_hut(m, 1040, y0, "east")

    slab = m.box((192, y0 + 192, 0), (320, y0 + 320, train_top_a))
    m.brush_entity("func_tracktrain", [slab], origin=(256, y0 + 256, train_top_a / 2),
                   targetname="f14_train", target="f14_pt_a", speed=100, startspeed=100,
                   orientationtype=0, height=0, spawnflags="18")
    m.entity("path_track", origin=(256, y0 + 256, train_top_a / 2),
             targetname="f14_pt_a", target="f14_pt_c", orientationtype=0)
    m.entity("path_track", origin=(824, y0 + 256, train_top_c - 6.0),
             targetname="f14_pt_c", target="f14_pt_a", orientationtype=0)

    m.expect_visible(hut_e.all_inner(), "hut E inner side: the ledge is reached by riding the tracktrain to pose C")
    m.expect_visible(ledge_e.side("west"), "control: ledge E face, seen from the floor")
    m.expect_visible(room_t.inner("ceiling"), "control: ceiling of the train chamber")
    m.expect_hidden(hut_e.outer("ceiling"), "hut E roof top z=288: above every eye (ledge/rider max 160 + 85)")

    # ====================== chamber P: the push ====================================
    y0 = 704.0
    room_p = m.room((0, y0, 0), (1024, y0 + 512, 448))
    m.entity("info_player_deathmatch", origin=(256, y0 + 256, 1), angles="0 0 0")
    ledge_p = m.box((640, y0, 0), (1024, y0 + 512, LEDGE))
    hut_p = _ledge_hut(m, 784, y0, "east")
    column = m.box((560, y0 + 192, 0), (640, y0 + 320, 224), material=TRIGGER)
    m.brush_entity("trigger_push", [column], pushdir="-90 0 0", speed=600, spawnflags="1")

    m.expect_visible(hut_p.all_inner(), "hut P inner side: the ledge is reached through the upward trigger_push")
    m.expect_visible(ledge_p.side("west"), "control: ledge P face, seen from the floor")
    m.expect_visible(room_p.inner("ceiling"), "control: ceiling of the push chamber")

    # ====================== chamber C: the control =================================
    y0 = 1408.0
    room_c = m.room((0, y0, 0), (1024, y0 + 512, 448))
    m.entity("info_player_deathmatch", origin=(768, y0 + 256, 1), angles="0 0 0")
    ledge_c = m.box((0, y0, 0), (448, y0 + 512, LEDGE))
    hut_c = _ledge_hut(m, 144, y0, "west")

    m.expect_visible(ledge_c.side("east"), "control: ledge C face, seen from the floor")
    m.expect_visible(room_c.inner("ceiling"), "control: ceiling of the control chamber")
    m.expect_hidden(hut_c.all_inner(), "hut C inner side: 160u ledge with nothing to lift the player, doorway faces away")
    m.expect_hidden(hut_c.outer("ceiling"), "hut C roof top z=288: above every eye (max 85)")

    m.note("Train: A (256,256,6) <-> C (824,256,154), slab 128x128x12, tops 12u / 160u. Push: pushdir -90 0 0 speed 600, "
           "column z 0..224 against the ledge face. Control ledge has neither. All ledge tops 160u.")
