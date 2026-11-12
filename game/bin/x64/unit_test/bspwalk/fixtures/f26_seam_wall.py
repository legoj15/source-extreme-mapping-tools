"""
f26 -- a wall built of abutting brushes hides what is behind it, seam planes included (#33, #40).

bspwalk shrinks every occluder by 1/32u on its own, so two opaque brushes that meet on a plane
leave a 1/16u slot along it, and a ray lying in that plane passed through the wall.  On
d1_trainstation_02 every face a finer eye spacing "found" was seen only from an eye exactly in
a brush seam plane and only along it.  The eye lattice is world aligned (multiples of the 8u
spacing in x, y and z), so the seams here sit on lattice coordinates and the targets have a
sample on the same plane or line.

W is func_detail, so VVIS sees through it and the boxes behind it are in the spawn room's PVS:
tier A cannot hide them and the face pass decides (a world wall would hide them by PVS alone and
the seams would never be tested).  An outer shell seals the map, since detail does not.

Geometry (x east, y north, z up; interior bounds, 16u walls):
    S  spawn room   x 0..512    y 0..512   z 0..192   east wall omitted (W closes it)
    W  wall         x 512..528  y -16..528 z -16..208, func_detail brushes:
         W1 y -16..128; W2 y 128..224              seam plane y = 128 between them
         W3a/W3b/W3c/W3d y 224..256..288 cut at z 64  four brushes meeting along y 256, z 64
         W4 y 288..SLOT_Y0; W5 y SLOT_Y1..528        a real slot y SLOT_Y0..SLOT_Y1 (1u), z 0..192,
                                                     closed below and above by two small brushes
    R  room         x 528..784  y 0..512   z 0..192   west wall omitted (W)
    T   box in R    x 640..656  y 96..160   z 0..96   west side centred on y 128 (the seam plane)
    T2  box in R    x 640..656  y 240..272  z 48..80  west side centred on (y 256, z 64) (the line)
    L   a low wall in R along y SLOT_Y0: Lw x 528..648, Le x 648..784, y 352..SLOT_Y0, z 0..96
    K   a block standing against L's north face, x 640..656, y SLOT_Y0..SLOT_Y1, z 0..192
    P   pillar in R x 700..716  y 386..424  z 0..192
    O   outer shell, interior x -64..848  y -64..592  z -64..272, around everything.
    info_player_start at (128, 256, 1).

Every sight line from S into R crosses W through the slot: it lies in y 383.98..384.98 for
all of x 512..528, so |dy/dx| <= 1/16 there, and by x = 656 it is within y 376..393.

must_hidden (strict), each truly hidden and, before the seam fillers, seen only through a seam:
    T's sides: y 96..160 is out of the slot's reach.  An eye at (x, 128, z) sees the west side's
        centroid (640, 128, 48) along the plane y = 128, through the W1/W2 seam.
    T2's sides: out of the slot's reach.  An eye at (x, 256, 64) sees the west side's centroid
        (640, 256, 64) along the line where W3a..W3d meet: pairwise fillers alone leave a tunnel
        there, so it checks that fillers reach past an edge where a third and fourth brush go on.
    Le's north side (y = SLOT_Y0, faces +y, x 656..784 where K does not cover it): a segment
        from the slot to it stays within y SLOT_Y0..SLOT_Y1 + 0.5 where it passes K (x 640..656)
        and so runs through K.  An eye at y 384 (1/64u above the plane) sees it grazing the
        plane under K: the K/Le contact's slot (#33's grazing class).
must_visible:
    S inner sides and W's west sides (the spawn room's east wall).
    P's west side (x = 700), seen through the real slot: from the eye (496, 384, 64) with
        dy/dx 0.0265 a segment crosses x 512..528 at y 384.4..384.8 and passes x 640..656 at
        y 387.8..388.2, above K.  An over-blocking fix (one that closes a real 1u slot) hides it
        and FAILs.  R's east wall is not claimed: verify_expectations samples it on a 64u grid,
        sight lines through the slot meet it within y 383.8..392.9, and the one sample row
        there (y 384) is reached only along lines that K blocks.
"""
import vmfgen

NAME = "f26_seam_wall"
DESCRIPTION = ("A wall of abutting world brushes (a two-brush seam, four brushes meeting along a line) and a "
               "block standing against a low wall hide what is behind them, rays along seam planes included; "
               "a real 1u slot beside them still shows a pillar.")

SLOT_Y0 = 384.0 - 1.0 / 64.0
SLOT_Y1 = SLOT_Y0 + 1.0
WX0, WX1 = 512.0, 528.0
WZ0, WZ1 = -16.0, 208.0


def build(m: vmfgen.Map) -> None:
    # the lattice hits the seam plane, the line and the grazing eye (8u spacing, world aligned)
    assert 128 % 8 == 0 and 256 % 8 == 0 and 64 % 8 == 0 and 384 % 8 == 0
    assert vmfgen.MAX_EYE_ABOVE_FLOOR > 64 > 28
    # the grazing eye at y 384 is inside the slot and above Le's north face
    assert SLOT_Y0 < 384.0 < SLOT_Y1
    # a sight line through the slot moves at most 1/16u in y per unit x: by x 656 it is
    # nowhere near T (y <= 160) or T2 (y <= 272)
    assert SLOT_Y0 - (656 - WX1) * (SLOT_Y1 - SLOT_Y0) / (WX1 - WX0) > 272

    spawn = m.room((0, 0, 0), (512, 512, 192), omit=("east",))
    m.entity("info_player_start", origin=(128, 256, 1), angles="0 0 0")
    room = m.room((528, 0, 0), (784, 512, 192), omit=("west",))

    m.room((-64, -64, -64), (848, 592, 272))               # O: seals the map around the detail wall
    wall = [
        m.box((WX0, -16, WZ0), (WX1, 128, WZ1)),            # W1
        m.box((WX0, 128, WZ0), (WX1, 224, WZ1)),            # W2
        m.box((WX0, 224, WZ0), (WX1, 256, 64)),             # W3a
        m.box((WX0, 256, WZ0), (WX1, 288, 64)),             # W3b
        m.box((WX0, 224, 64), (WX1, 256, WZ1)),             # W3c
        m.box((WX0, 256, 64), (WX1, 288, WZ1)),             # W3d
        m.box((WX0, 288, WZ0), (WX1, SLOT_Y0, WZ1)),        # W4
        m.box((WX0, SLOT_Y1, WZ0), (WX1, 528, WZ1)),        # W5
    ]
    closers = [m.box((WX0, SLOT_Y0, WZ0), (WX1, SLOT_Y1, 0)),     # under the slot
               m.box((WX0, SLOT_Y0, 192), (WX1, SLOT_Y1, WZ1))]   # over the slot
    m.brush_entity("func_detail", wall + closers)

    t = m.box((640, 96, 0), (656, 160, 96))
    t2 = m.box((640, 240, 48), (656, 272, 80))
    m.box((528, 352, 0), (648, SLOT_Y0, 96))                # Lw
    le = m.box((648, 352, 0), (784, SLOT_Y0, 96))           # Le
    m.box((640, SLOT_Y0, 0), (656, SLOT_Y1, 192))           # K
    pillar = m.box((700, 386, 0), (716, 424, 192))

    m.expect_visible(spawn.all_inner(), "inner side of the room the player spawns in")
    for w in wall:
        m.expect_visible(w.side("west"), "W's west side: the spawn room's east wall")
    m.expect_visible(pillar.side("west"), "P's west side, seen through the 1u slot past K")
    for s in ("west", "east", "north", "south", "top"):
        m.expect_hidden(t.side(s), "T is out of the slot's reach; its west side lies across the W1/W2 seam plane y 128",
                        strict=True)
    for s in ("west", "east", "north", "south", "top", "bottom"):
        m.expect_hidden(t2.side(s), "T2 is out of the slot's reach; its west side's centre is on the line where "
                        "W3a..W3d meet", strict=True)
    m.expect_hidden(le.side("north"), "Le's north side: every segment from the slot to it runs through K; an eye "
                    "1/64u above its plane sees it only grazing the K/Le contact", strict=True)

    m.expect_reach(spawn.floor_center(0), "spawn room floor: the player spawns on it")
    m.expect_no_reach(room.floor_center(0), "R's floor: the slot is 1u wide")

    m.note("W is ten func_detail brushes; the slot y SLOT_Y0..SLOT_Y1 is the only way into R.")

    # verify_expectations.py (#19)
    m.verify_floor(0, 0, 512, 512, 0, "spawn room floor")
