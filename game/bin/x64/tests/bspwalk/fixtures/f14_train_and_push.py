"""
f14 -- a ledge reachable only by riding a func_tracktrain, a ledge reachable only through a
trigger_push, a ledge of the same height that has neither, and one beside a push too weak to lift.

Four separate sealed chambers, each with its own spawn (a walker seeds every info_player_*).  They are
separate so that the train, the push arcs and the controls cannot interact: a push launch or a long
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
    speed 900, spawnflags 1 (clients).  The column touches ledge P's west face.

    A continuous push is base velocity: its z part is an acceleration (gamemovement StartGravity), so
    inside the column the net acceleration is 900 - 600 = 300 u/s^2 upward.  A player standing in it
    is lifted off the floor and leaves the column top (feet z 224) at sqrt(2 * 300 * 224) = 366.6 u/s,
    peaks at 224 + 366.6^2 / 1200 = 336, and comes down on the ledge (top 160).  Without the push, or
    with the push turned horizontal, the ledge is 160u away and a duck-jump rises 57u.  (At speed 600
    the push only cancels gravity: a standing player hovers, but a jumping one keeps his 160 u/s up to
    the top and still makes the ledge, so 600 would not separate the push model from a plain jump-in.)
    The roof top is NOT asserted hidden here: a lifted player's eye can be 336 + 64 above the floor.

Chamber W (weak push), y 2112..2624, x 0..1024, ceiling z=448, spawn (256, y0+256, 1):

    The same column and ledge as chamber P, with speed 300: inside it gravity is 600 - 300 = 300, so a
    duck-jump peaks at 36 + 160^2 / (2 * 300) = 78.7 < 160 and the push never lifts.  The ledge stays
    unreached (the negative control for the push model).

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
PUSH_SPEED = 900.0     # lifts: 300 u/s^2 net upward at gravity 600
WEAK_SPEED = 300.0     # does not lift: 300 u/s^2 net downward


def _ledge_hut(m: vmfgen.Map, x0: float, y0: float, door: str) -> vmfgen.Room:
    """Closed hut interior x x0..x0+96, y y0+176..y0+336, z LEDGE..LEDGE+112, door in the `door` wall."""
    ymid_lo, ymid_hi = y0 + 224, y0 + 288
    return m.room((x0, y0 + 176, LEDGE), (x0 + 96, y0 + 336, LEDGE + HUT_H), floor=False,
                  openings=[(door, (ymid_lo, ymid_hi, LEDGE, LEDGE + 80))])


def build(m: vmfgen.Map) -> None:
    cj = vmfgen.CROUCH_JUMP_FEET
    assert LEDGE > cj * 1.15 + 40                                    # unreachable without help
    assert LEDGE + HUT_H + 16 > LEDGE + vmfgen.MAX_EYE_ABOVE_FLOOR * 1.15 + 20     # roof tops above ledge eyes
    g, vj = vmfgen.SV_GRAVITY, vmfgen.JUMP_SPEED
    push_top = 224.0
    v_exit = (2 * (PUSH_SPEED - g) * push_top) ** 0.5                 # leaving the lifting column from rest
    assert push_top + v_exit ** 2 / (2 * g) > LEDGE + 40               # its apex clears the ledge
    assert vmfgen.CROUCH_LIFT + vj ** 2 / (2 * (g - WEAK_SPEED)) < LEDGE - 40     # the weak push never lifts
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
    m.expect_reach((1000, y0 + 64, LEDGE), "ledge E top: ride the tracktrain to pose C, walk the 8u gap")
    m.expect_reach((1088, y0 + 256, LEDGE), "hut E floor: walk round to its east doorway on ledge E")

    # ====================== chamber P: the push ====================================
    y0 = 704.0
    room_p = m.room((0, y0, 0), (1024, y0 + 512, 448))
    m.entity("info_player_deathmatch", origin=(256, y0 + 256, 1), angles="0 0 0")
    ledge_p = m.box((640, y0, 0), (1024, y0 + 512, LEDGE))
    hut_p = _ledge_hut(m, 784, y0, "east")
    column = m.box((560, y0 + 192, 0), (640, y0 + 320, push_top), material=TRIGGER)
    m.brush_entity("trigger_push", [column], pushdir="-90 0 0", speed=PUSH_SPEED, spawnflags="1")

    m.expect_visible(hut_p.all_inner(), "hut P inner side: the ledge is reached through the upward trigger_push")
    m.expect_visible(ledge_p.side("west"), "control: ledge P face, seen from the floor")
    m.expect_visible(room_p.inner("ceiling"), "control: ceiling of the push chamber")
    m.expect_reach((900, y0 + 64, LEDGE), "ledge P top: lifted by the upward trigger_push")
    m.expect_reach((832, y0 + 256, LEDGE), "hut P floor: walk round to its east doorway on ledge P")

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
    m.expect_reach((768, y0 + 400, 0), "control chamber floor at the spawn")
    m.expect_no_reach((300, y0 + 64, LEDGE), "ledge C top: 160u with nothing to lift the player")
    m.expect_no_reach((192, y0 + 256, LEDGE), "hut C floor on the unreachable ledge C")

    # ====================== chamber W: the weak push ===============================
    y0 = 2112.0
    m.room((0, y0, 0), (1024, y0 + 512, 448))
    m.entity("info_player_deathmatch", origin=(256, y0 + 256, 1), angles="0 0 0")
    m.box((640, y0, 0), (1024, y0 + 512, LEDGE))
    weak = m.box((560, y0 + 192, 0), (640, y0 + 320, push_top), material=TRIGGER)
    m.brush_entity("trigger_push", [weak], pushdir="-90 0 0", speed=WEAK_SPEED, spawnflags="1")
    m.expect_reach((256, y0 + 400, 0), "weak push chamber floor at the spawn")
    m.expect_no_reach((900, y0 + 64, LEDGE), "ledge W top: the push beside it (300 < gravity 600) never lifts")

    m.note("Train: A (256,256,6) <-> C (824,256,154), slab 128x128x12, tops 12u / 160u. Push: pushdir -90 0 0 speed 900, "
           "column z 0..224 against the ledge face. Control ledge has neither. Weak push: speed 300, never lifts. "
           "All ledge tops 160u.")

    # verify_expectations.py (#19)
    m.verify_floor(0, 0, 896, 512, 0, "train chamber floor")
    m.verify_floor(192, 192, 320, 320, train_top_a, "train top at pose A")
    m.verify_floor(824 - 64, 192, 824 + 64, 320, train_top_c, "train top at pose C")
    m.verify_eye_box((192, 192, train_top_a + vmfgen.EYE_DUCK), (888, 320, train_top_c + 86), "riding the train",
                     role="hidden")
    m.verify_floor(896, 0, 1280, 512, LEDGE, "ledge E top, hut E's inside included")
    m.verify_floor(0, 704, 640, 1216, 0, "push chamber floor")
    # a lifted eye peaks at 336 + 64 = 400 (docstring); 404 keeps a 4u margin.  x runs on to the ledge's far
    # end because the lifted player comes down on the ledge, not in the column.
    m.verify_eye_box((560, 704 + 192, vmfgen.EYE_DUCK), (1024, 704 + 320, 404), "lifted in the push column and over ledge P",
                     role="hidden")
    m.verify_floor(640, 704, 1024, 1216, LEDGE, "ledge P top, hut P's inside included")
    m.verify_floor(448, 1408, 1024, 1920, 0, "control chamber floor")
    m.verify_floor(0, 2112, 640, 2624, 0, "weak push chamber floor")
