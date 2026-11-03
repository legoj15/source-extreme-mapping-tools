"""
f07 -- a room behind a closed func_door.

A func_door in a doorway is opaque and solid while shut, but the player can open
it (spawnflags 256 "Use Opens" + 1024 "Touch Opens"), so everything behind it
is reachable and visible in play.  The design doc's policy for brush entities
("tops only"): a mover contributes no obstacle and never occludes sight, so the
flood goes through the door and the room beyond is reached.  A walker that
treats the door as permanently solid (the Python walker does, per the defect
list in the design doc) never leaves the first room and calls the whole second
room hidden.

Geometry (interior bounds, 16u walls; plan view, x east):

    spawn room S   x 0..384    y 0..512  z 0..192   info_player_start at (192,256)
    back room B    x 416..800  y 0..512  z 0..192
    void room V    x 832..1024 y 0..512  z 0..192   sealed, east of B
    S east wall (x 384..400) and B west wall (x 400..416) each have a doorway
    y 208..304, z 0..112, lined up: a 32u-long passage joins the rooms.
    The door brush: x 396..404 (8u thick, in the middle of the passage),
    y 208..304, z 0..112, exactly filling the doorway.  Keys: func_door,
    movedir "0 90 0" (slides north into the wall), speed 100, spawnflags 1280.

The door is not what keeps the map sealed.  VBSP does not flood through, or
seal with, a brush entity: the passage is open air in the world, closed round its
edges by the cut faces of S's and B's own wall boxes.  S, the passage and B form
one sealed region without the door, so removing the door leaves the map exactly as
sealed; that is why this compiles (no leak) and why VVIS sees through it.  Measured on the
compiled BSP: the S cluster is in the PVS of the B cluster and the reverse
(func_door is no vis blocker), while the V cluster is in neither's PVS.

Expectations:
  visible  S inner sides: the player spawns there.
  visible  B inner sides (ceiling, floor, north/south/east walls, the west wall's
           pieces around the doorway): the player opens the door, walks the
           32u passage and stands in B.  Eyes in B (28..85u over the floor) see
           all six walls of a 384 x 512 x 192 room.
  hidden   V inner sides: a sealed box (info_target keeps VBSP from filling it);
           no opening, so no sight line enters, and nothing reachable is inside.
  (No roof-top expectations: the rooms stand in the void, so VBSP drops their
  outside faces -- an "absent side" error in the harness.)
"""
import vmfgen

NAME = "f07_closed_door"
DESCRIPTION = ("Spawn room, a room behind a closed func_door (openable, so visible), "
               "and a sealed void room beside it.")

DOOR_Y = (208.0, 304.0)
DOOR_H = 112.0


def build(m: vmfgen.Map) -> None:
    assert DOOR_Y[1] - DOOR_Y[0] > vmfgen.HULL_STAND[0] + 32      # a hull walks through
    assert DOOR_H > vmfgen.HULL_STAND[2] + 16

    opening = (DOOR_Y[0], DOOR_Y[1], 0, DOOR_H)
    start = m.room((0, 0, 0), (384, 512, 192), openings=[("east", opening)])
    m.entity("info_player_start", origin=(192, 256, 1), angles="0 0 0")
    back = m.room((416, 0, 0), (800, 512, 192), openings=[("west", opening)])
    void = m.room((832, 0, 0), (1024, 512, 192), marker=True)

    door = m.box((396, DOOR_Y[0], 0), (404, DOOR_Y[1], DOOR_H))
    m.brush_entity("func_door", [door], targetname="bspwalk_door", movedir="0 90 0",
                   speed="100", lip="0", wait="4", spawnflags="1280")

    m.expect_visible(start.all_inner(), "inner side of the room the player spawns in")
    m.expect_visible(back.all_inner(),
                     "inner side of the room behind a closed func_door: doors are openable, "
                     "so the player reaches it and sees all of it")
    m.expect_hidden(void.all_inner(), "inner side of a sealed void room east of the back room")

    m.note("func_door fills the doorway but is not what seals the map; the rooms' own walls do. "
           "Use Opens + Touch Opens, so the player can open it.")
