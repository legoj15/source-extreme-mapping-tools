"""
f08 -- rooms reachable only through trigger_teleport, plus a room nobody can teleport into.

Four sealed rooms (no opening anywhere, so VVIS sees four disjoint PVS islands and the
spawn room's PVS contains none of the others):

    S  spawn room    x 0..512     y 0..512     z 0..192
                     info_player_start at (128, 256, 1)
                     pad 1: trigger_teleport brush x 352..448 y 192..320 z 0..96 (on the floor,
                     away from the spawn so it is reached by walking), target f08_dest_a, spawnflags 1 (clients)
    A  teleport room x 640..1152  y 0..512     z 0..192
                     info_teleport_destination "f08_dest_a" at (768, 256, 1)
                     pad 2: trigger_teleport brush x 992..1088 y 192..320 z 0..96, target f08_dest_c,
                     spawnflags 1.  Reached from the destination by walking about 220u.
    C  chain room    x 1280..1792 y 0..512     z 0..192
                     info_teleport_destination "f08_dest_c" at (1408, 256, 1).  Reachable only
                     by teleporting twice, so a walker has to keep seeding teleports from the
                     space it reached through earlier teleports.
    B  orphan room   x 0..512     y 640..1152  z 0..192
                     info_teleport_destination "f08_orphan" at (256, 896, 1) that no trigger
                     targets.  The entity inside keeps VBSP from filling the room solid.

The trigger brushes are brush entities, so they leave the world: they are neither obstacles nor
occluders, and they carry TOOLS/TOOLSTRIGGER (no drawn faces).

Why each expectation is certain:
  visible  every inner side of S: the player spawns in it.
  visible  every inner side of A: pad 1 is on S's floor, 220u from the spawn, and the destination
           puts the player inside A.  Eyes at 28..85 over A's floor see its walls, floor and
           192u ceiling.  A is sealed from S, so a walker that only flood-fills walkable space
           and PVS (VVIS proves S cannot see A) wrongly calls A hidden.  That is the point.
  visible  every inner side of C: pad 2 is on A's floor, reached by walking in A, and its
           destination is inside C.
  hidden   every inner side of B: no trigger_teleport (or anything else) targets "f08_orphan", and
           B has no opening.  Every segment from S, A or C into B crosses a wall.
"""
import vmfgen

NAME = "f08_teleport_room"
DESCRIPTION = ("Rooms reachable only through trigger_teleport (one directly, one by chaining "
               "two teleports) and a sealed room whose teleport destination nobody targets.")

TRIGGER = "TOOLS/TOOLSTRIGGER"


def _pad(m: vmfgen.Map, mins, maxs, target: str) -> vmfgen.Entity:
    brush = m.box(mins, maxs, material=TRIGGER)
    return m.brush_entity("trigger_teleport", [brush], target=target, spawnflags="1")


def build(m: vmfgen.Map) -> None:
    spawn = m.room((0, 0, 0), (512, 512, 192))
    m.entity("info_player_start", origin=(128, 256, 1), angles="0 0 0")
    _pad(m, (352, 192, 0), (448, 320, 96), "f08_dest_a")

    room_a = m.room((640, 0, 0), (1152, 512, 192))
    m.entity("info_teleport_destination", origin=(768, 256, 1), angles="0 0 0", targetname="f08_dest_a")
    _pad(m, (992, 192, 0), (1088, 320, 96), "f08_dest_c")

    room_c = m.room((1280, 0, 0), (1792, 512, 192))
    m.entity("info_teleport_destination", origin=(1408, 256, 1), angles="0 0 0", targetname="f08_dest_c")

    room_b = m.room((0, 640, 0), (512, 1152, 192))
    m.entity("info_teleport_destination", origin=(256, 896, 1), angles="0 0 0", targetname="f08_orphan")

    m.expect_visible(spawn.all_inner(), "inner side of the room the player spawns in")
    m.expect_visible(room_a.all_inner(), "inner side of the room trigger_teleport 1 sends the player to")
    m.expect_visible(room_c.all_inner(), "inner side of the room reached by chaining teleport 1 then teleport 2")
    m.expect_hidden(room_b.all_inner(), "inner side of a sealed room whose info_teleport_destination nothing targets")

    m.expect_reach(spawn.floor_center(0), "spawn room floor: the player spawns on it")
    m.expect_reach(room_a.floor_center(0), "room A floor: teleport 1 from the spawn room")
    m.expect_reach(room_c.floor_center(0), "room C floor: teleport 1, then teleport 2 from room A")
    m.expect_no_reach(room_b.floor_center(0), "room B floor: sealed, its destination is never targeted")

    m.note("Teleports: S pad1 -> f08_dest_a (room A), A pad2 -> f08_dest_c (room C); f08_orphan (room B) is never targeted.")

    # verify_expectations.py (#19)
    m.verify_floor(0, 0, 512, 512, 0, "spawn room floor")
    m.verify_floor(640, 0, 1152, 512, 0, "room A floor (teleport 1)")
    m.verify_floor(1280, 0, 1792, 512, 0, "room C floor (teleport 2)")
