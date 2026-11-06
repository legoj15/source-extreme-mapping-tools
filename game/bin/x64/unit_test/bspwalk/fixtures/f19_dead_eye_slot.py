"""
f19 -- a low hall's ceiling seen only through a floor-level slot, and only by a dead player's view (#27).

bspwalk adds a dead player's view (HL2 VEC_DEAD_VIEWHEIGHT, 14u over the floor) to the eyes of
every reached floor: eyes from floor + 14 up to the crouch eye at floor + 28.  Every other eye
is at or above floor + 28, so a fixture only tests that layer if what it must see is out of reach
of every eye at 28u and up.

The slot: the spawn room S and a low hall H are joined by a tube 56u wide, 32u long (two 16u
walls) and 16u tall, z 8..24 over the floor.  Its top, z 24, is under the crouch eye (28).  A
line from any eye at z >= 28 through the tube is therefore descending when it leaves the tube
(it is under 24 at the far mouth), and never reaches a point of H above z 24.  H's ceiling is
at z 32.  A line from an eye under 24 can rise through the tube, so the dead layer sees it:

    eye E = (128,176,14), 16u from S's north wall (the hull's reach), to the ceiling point
    P = (128,288,32): slope 18/112 = 0.161; z at the near mouth (y 192) 16.6, at the far mouth
    (y 224) 21.7: both inside 8..24.  Eyes at 21 see it from y >= 352 (slope <= 3/48).
    The crouch eye (128,176,28) is above the tube's top plane.  Nothing at 28u or more sees the
    ceiling, standing and jumping eyes included.

Geometry (x east, y north, z up; interior bounds, 16u walls):
    S  spawn room  x 0..256  y 0..192  z 0..96    info_player_start (128,96,1)
       north wall  y 192..208, tube hole x 100..156 z 8..24
    H  hall        x 0..256  y 224..480  z 0..32  DEV/GRAYGRID; its south wall y 208..224 has the
                   same hole.  H is unreachable (the tube is shorter than a crouched hull, and
                   so is the hall) and not marked: VBSP floods into it through the tube.

must_visible: S's interior (the player stands in it); H's ceiling, dead view only (verify_expectations
              --mutants proves the checker fails without its dead eyes); H's floor, a control any
              eye looking down the tube sees (for the crouch eye at y 176: slope 0.25..0.42, floor
              hit at y 243..288).
must_hidden:  H's south wall, H side (y=224, faces +y): every eye has y <= 176 < 224.
"""
import vmfgen

NAME = "f19_dead_eye_slot"
DESCRIPTION = ("Low hall behind a 16u-tall floor-level slot: its ceiling is visible only from a dead "
               "player's view (14..28u over the floor), its floor from any eye looking down the slot.")

HALL_MAT = "DEV/GRAYGRID"
WALL = 16.0
SLOT_X0, SLOT_X1 = 100.0, 156.0        # off the 8u and 16u lattices: no eye or face point lies in a seam plane
SLOT_Z0, SLOT_Z1 = 8.0, 24.0
HALL_H = 32.0                          # interior height of H: the ceiling's z
HALL_Y0, HALL_Y1 = 224.0, 480.0
SPAWN_Y1 = 192.0


def build(m: vmfgen.Map) -> None:
    # The margins the docstring's argument depends on.
    assert vmfgen.EYE_DEAD < SLOT_Z1 < vmfgen.EYE_DUCK             # a dead eye can rise through it, no 28u eye can
    assert HALL_H > SLOT_Z1                                        # the ceiling is above everything a 28u eye sees
    assert SLOT_Z1 - SLOT_Z0 < vmfgen.HULL_DUCK[2] and HALL_H < vmfgen.HULL_DUCK[2]   # nobody fits
    tube = HALL_Y0 - SPAWN_Y1                                       # 32: both walls
    assert tube == 2 * WALL
    near, far = vmfgen.HULL_STAND[0] / 2, vmfgen.HULL_STAND[0] / 2 + tube   # eye to the near / far mouth
    e, p = (128.0, SPAWN_Y1 - near, vmfgen.EYE_DEAD), (128.0, 288.0, HALL_H)
    def z_at(y):
        return e[2] + (p[2] - e[2]) * (y - e[1]) / (p[1] - e[1])
    assert SLOT_Z0 < z_at(SPAWN_Y1) < SLOT_Z1 and SLOT_Z0 < z_at(HALL_Y0) < SLOT_Z1

    opening = (SLOT_X0, SLOT_X1, SLOT_Z0, SLOT_Z1)
    spawn = m.room((0, 0, 0), (256, SPAWN_Y1, 96), openings=[("north", opening)])
    m.entity("info_player_start", origin=spawn.floor_center(1), angles="0 90 0")
    hall = m.room((0, HALL_Y0, 0), (256, HALL_Y1, HALL_H), material=HALL_MAT,
                  openings=[("south", opening)])

    m.expect_visible(spawn.all_inner(), "inner side of the room the player spawns in")
    m.expect_visible(hall.inner("ceiling"),
                     "hall ceiling, dead view only: (128,176,14)->(128,288,32) rises through the tube "
                     "(z 16.6 at y 192, 21.7 at y 224); a line from an eye at 28u or more is descending")
    m.expect_visible(hall.inner("floor"),
                     "control: a line from the crouch eye (128,176,28) down the tube hits the floor at y 243..288")
    m.expect_hidden(hall.inner_all("south"),
                    "slot wall, hall side: faces +y, every eye has y<=176")

    m.expect_reach(spawn.floor_center(0), "spawn room floor: the player spawns on it")
    m.expect_no_reach(hall.floor_center(0), "hall floor: neither the 16u tube nor the 32u hall fits a crouched hull")

    m.note("Tube z 8..24: under the crouch eye (28). Ceiling z 32 is seen only from eyes at z < 24.")

    # verify_expectations.py (#19, #27): the floor declaration gives the dead layer as well
    m.verify_floor(0, 0, 256, SPAWN_Y1, 0, "spawn room floor")
