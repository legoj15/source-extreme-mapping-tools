"""
f24 -- a wall whose far side is nodraw still hides what is behind it from the near side (#77).

On d1_trainstation_02 a storefront wall is an opaque world brush whose store-side face is
nodraw and open to the store.  bspwalk dropped every such brush from the occluders (a ray
entering through the nodraw side sees through the brush), so fan rays from the street went
straight through the drawn street-side face and marked a door panel's inner face that no
player can see.  The engine draws the street side, so a view entering there is blocked; only
a view entering through the nodraw side sees through.

Geometry (x east, y north, z up; interior bounds, 16u walls):
    S  spawn room   x 0..512    y 0..512  z 0..192   east wall omitted (W closes it)
    W  wall         x 512..528  y -16..528 z -16..208, four world brushes around a window
                    y 224..288, z 96..160; each brush's east side (facing T) is nodraw, the
                    rest drawn.  The window sill (z 96) is above any jump (feet <= 57.33), so
                    no player enters T.
    T  store room   x 528..784  y 0..512  z 0..192   west wall omitted (W)
    block in T      x 560..576  y 432..496  z 0..128, a world box (the hidden corner)
    pillar in T     x 600..616  y 300..340  z 0..192, a world box in view of the window
    info_player_start at (128, 256, 1).

must_hidden (strict):
    block west side (x=560, faces -x, y 432..496).  Every player eye is in S (x <= 496).  A
    segment from S to that side either crosses W outside the window (W's drawn west side,
    x=512, faces the eye: blocked) or passes the window, where it lies within y 224..288 for
    all of x 512..528; reaching y >= 432 at x = 560 then needs a slope dy/dx >= (432-288)/32
    = 4.5, which over W's 16u depth moves y by 72u, more than the window's 64u.  The window is
    in S's PVS, so tier A cannot hide it; a walker that drops W as an occluder (its nodraw
    side faces T, inside the map) calls it visible.
must_visible:
    S inner sides, W's west sides (the spawn room's east wall), T's east inner side (x=784)
    and the pillar's west side (x=600), both seen through the window: (400, 160, 85) ->
    (600, 320, 140) crosses x 512..528 at y 250..262, z 116..120.  The pillar is the d1
    neighbour panel's role: a face beyond the one-way wall that players do see, which the
    fix must not hide.
"""
import vmfgen

NAME = "f24_one_way_wall"
DESCRIPTION = ("A wall of world brushes whose far side is nodraw (open to a room behind it) still hides "
               "that room's corner from the near side: only a view entering through the nodraw side sees "
               "through such a brush.")

WINDOW = (224.0, 288.0, 96.0, 160.0)     # y0, y1, z0, z1 in W
BLOCK_LO = (560.0, 432.0, 0.0)
BLOCK_HI = (576.0, 496.0, 128.0)


def build(m: vmfgen.Map) -> None:
    y0, y1, z0, z1 = WINDOW
    # nobody climbs into the window: the sill is above the highest feet a jump reaches
    assert z0 > vmfgen.CROUCH_JUMP_FEET + 16
    # the slope argument in the docstring: through 16u of wall a 64u window cannot carry a
    # sight line from S to y >= BLOCK_LO[1] at x = BLOCK_LO[0]
    slope = ( BLOCK_LO[1] - y1 ) / ( BLOCK_LO[0] - 528.0 )
    assert slope * 16.0 > y1 - y0

    spawn = m.room((0, 0, 0), (512, 512, 192), omit=("east",))
    m.entity("info_player_start", origin=(128, 256, 1), angles="0 0 0")
    nodraw_east = {"east": vmfgen.MAT_NODRAW}
    w_south = m.box((512, -16, -16), (528, y0, 208), face_materials=nodraw_east)
    w_north = m.box((512, y1, -16), (528, 528, 208), face_materials=nodraw_east)
    w_below = m.box((512, y0, -16), (528, y1, z0), face_materials=nodraw_east)
    w_above = m.box((512, y0, z1), (528, y1, 208), face_materials=nodraw_east)
    store = m.room((528, 0, 0), (784, 512, 192), omit=("west",))
    block = m.box(BLOCK_LO, BLOCK_HI)
    pillar = m.box((600, 300, 0), (616, 340, 192))

    m.expect_visible(spawn.all_inner(), "inner side of the room the player spawns in")
    for w in (w_south, w_north, w_below, w_above):
        m.expect_visible(w.side("west"), "W's drawn west side: the spawn room's east wall")
    m.expect_visible(store.inner("east"), "store room east inner side, seen through the window")
    m.expect_visible(pillar.side("west"), "pillar west side in the store room, seen through the window")
    m.expect_hidden(block.side("west"), "block west side: behind W's drawn west side from every eye, and "
                    "out of the window's reach; W's nodraw east side does not make W see-through from S",
                    strict=True)

    m.expect_reach(spawn.floor_center(0), "spawn room floor: the player spawns on it")
    m.expect_no_reach(store.floor_center(0), "store room floor: the window sill is out of jump reach")

    m.note("W's four brushes have a nodraw east side, open to the store room T.")

    # verify_expectations.py (#19)
    m.verify_floor(0, 0, 512, 512, 0, "spawn room floor")
