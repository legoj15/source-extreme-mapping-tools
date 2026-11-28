"""
f36 -- a face that only an env_cubemap inside a solid brush sees is cubemap_only (#177 item 2).

f23's map with an env_cubemap at the camera's origin O instead of the point_camera, run with
-keepcubemaps (m.walker_args): #82 drops a cubemap in a brush as a viewpoint, and -keepcubemaps
keeps every cubemap (the old behaviour), so this one is a viewpoint in solid like f23's camera.
The geometry, the eye regions and why each side is seen by the viewpoint in the wall and by no
player are f23's (see its docstring).  A cubemap's view is a reflection's capture, never shown
to the player as it is (#149), so every side f23 expects visible only because the camera in the
wall sees it -- the sealed room V's five inner sides, the screen's west side, the hut's west
inner side, the hut's east outer sides -- is cubemap_only here, not visible.  The sides the
player sees (the spawn room's inner sides, W's west side) stay visible, and f23's three strict
hidden sides stay hidden: the cubemap at O is behind V's west inner side, below the hut's roof
top, and the screen blocks the target's west side from it.

The walker reports the cubemap "ok" (-keepcubemaps turns the in-brush and nodraw checks off),
and the harness's second run without the option is scored on reach only, so it needs no
default-run expectation: without -keepcubemaps the cubemap is no viewpoint and V would hide,
which is f22's business, not this fixture's.
"""
import vmfgen

NAME = "f36_cubemap_in_solid"
DESCRIPTION = ("f23's map with an env_cubemap inside the wall brush instead of the point_camera, under "
               "-keepcubemaps (#177 item 2): every side only that cubemap sees (the sealed room beyond the "
               "wall, the screen in it, the hut wall through the doorway only it looks into) is cubemap_only, "
               "not visible; the sides the player sees stay visible and the strict hidden sides stay hidden.")

ORIGIN = (536.0, 256.0, 96.0)
ROOF_TOP = 144.0


def build(m: vmfgen.Map) -> None:
    # every player eye and the cubemap are below the roof top, so its plane hides it from all
    assert vmfgen.MAX_EYE_ABOVE_FLOOR < ROOF_TOP and ORIGIN[2] < ROOF_TOP
    assert ROOF_TOP > vmfgen.CROUCH_JUMP_FEET + 64      # nobody stands on the roof
    # the gap between the hut and W is narrower than a hull
    assert 512 - 504 < 2 * 16
    # VBSP truncates lump 42 origins to whole units (expect_cubemap checks it too)
    assert all(c == int(c) for c in ORIGIN)

    m.walker_args("-keepcubemaps")                      # keep the in-brush cubemap a viewpoint (#82)

    spawn = m.room((0, 0, 0), (512, 512, 192), omit=("east",))
    m.entity("info_player_start", origin=(128, 256, 1), angles="0 0 0")
    wall = m.box((512, -16, -16), (560, 528, 208))
    void = m.room((560, 0, 0), (816, 512, 192), omit=("west",), marker=True)
    m.entity("env_cubemap", origin=ORIGIN, cubemapsize=0)
    screen = m.box((600, 232, 0), (616, 280, 144))
    target = m.box((700, 224, 0), (716, 288, 128))
    hut = m.room((384, 192, 0), (488, 320, 128), floor=False,
                 openings=[("east", (232, 280, 64, 128))])

    m.expect_visible(spawn.all_inner(), "inner side of the room the player spawns in")
    m.expect_visible(wall.side("west"), "W's west side: the east wall of the spawn room")
    for face in ("east", "floor", "ceiling", "north", "south"):
        m.expect_cubemap_only(void.inner(face), f"sealed room V {face} inner side: the env_cubemap inside W "
                              "lies in front of it and W does not hide it from there; no player sees it")
    m.expect_cubemap_only(hut.inner("west"), "hut west inner side: the env_cubemap looks at it through the "
                          "east doorway; no player eye can reach that doorway's line of sight")
    m.expect_cubemap_only(hut.outer_all("east"), "hut east outer side across the 8u gap: only the env_cubemap "
                          "is in front of it (every player eye has x <= 496); the PVS holds the gap")
    m.expect_cubemap_only(screen.side("west"), "screen west side in V: the env_cubemap looks straight at it")
    m.expect_hidden(target.side("west"), "target west side in V: in front of the env_cubemap but behind the "
                    "screen from it; no player sees into V", strict=True)
    m.expect_hidden(wall.side("east"), "V west inner side (W's east side): the env_cubemap at x=536 is behind "
                    "it, no player sees into V", strict=True)
    m.expect_hidden(hut.outer("ceiling"), "hut roof top z=144: every eye and the env_cubemap are below it",
                    strict=True)

    # -keepcubemaps skips the in-brush and nodraw rules (bw_cubemap.cpp CheckOne), so the cubemap
    # inside W stays "ok"; the static prop check still runs, and the map has no props
    m.expect_cubemap(ORIGIN, "ok", why="inside W under -keepcubemaps: no disqualifying rule runs")

    m.expect_reach(spawn.floor_center(0), "spawn room floor: the player spawns on it")
    m.expect_no_reach(void.floor_center(0), "floor of the sealed room beyond W")

    m.note(f"env_cubemap at {ORIGIN}, inside the world brush x 512..560 between S and V (f23's point_camera "
           f"position); -keepcubemaps keeps it a viewpoint.")

    # verify_expectations.py (#19): f23's eye regions, with the cubemap as the viewpoint
    m.verify_floor(0, 0, 512, 176, 0, "spawn room floor south of the hut")
    m.verify_floor(0, 336, 512, 512, 0, "spawn room floor north of the hut")
    m.verify_floor(0, 176, 368, 336, 0, "spawn room floor west of the hut")
    m.verify_viewpoint(ORIGIN, "env_cubemap inside W", kind="cubemap")
