"""
f01 -- sealed void rooms next to a spawn room.

The cheapest ground truth there is: a region with no way in.  Tier A (PVS
proof) should classify every void-room face hidden, because the void rooms
share no portal with the spawn room.

Geometry (interior bounds, 16u walls):
    spawn room   x 0..512    y 0..512  z 0..192   info_player_start in the middle
    void east    x 544..800  y 0..512  z 0..192   its west wall touches the
                                                  spawn room's east wall
    void above   x 0..512    y 0..512  z 224..416 its floor touches the spawn
                                                  room's ceiling
No opening anywhere, so no player can enter a void room and no sight line
crosses a wall.  Each void room holds an info_target: without one VBSP fills
the room solid and the faces would not exist at all.

Why each expectation is certain:
  visible  all six inner sides of the spawn room -- the player spawns inside it
           and eyes at 28..85 above the floor see every wall, the floor and the
           192u ceiling.
  hidden   all inner sides of both void rooms -- closed boxes: no player
           position is inside them, and every straight line from the spawn room
           to them crosses at least one solid wall.
"""
import vmfgen

NAME = "f01_void_room"
DESCRIPTION = "Spawn room with two sealed void rooms (east and above) that share no opening."


def build(m: vmfgen.Map) -> None:
    spawn = m.room((0, 0, 0), (512, 512, 192))
    m.entity("info_player_start", origin=spawn.floor_center(), angles="0 0 0")

    void_east = m.room((544, 0, 0), (800, 512, 192), marker=True)
    void_above = m.room((0, 0, 224), (512, 512, 416), marker=True)

    m.expect_visible(spawn.all_inner(), "inner side of the room the player spawns in")
    m.expect_hidden(void_east.all_inner(), "inner side of a sealed void room east of the spawn room")
    m.expect_hidden(void_above.all_inner(), "inner side of a sealed void room above the spawn room")

    m.expect_reach(spawn.floor_center(0), "spawn room floor: the player spawns on it")
    m.expect_no_reach(void_east.floor_center(0), "floor of the sealed void room east of the spawn room")
    m.expect_no_reach(void_above.floor_center(0), "floor of the sealed void room above the spawn room "
                      "(224u up, behind the 16u ceiling and floor slabs)")

    # verify_expectations.py (#19): the spawn room floor is the only place to stand
    m.verify_floor(0, 0, 512, 512, 0, "spawn room floor")
