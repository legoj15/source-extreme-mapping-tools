"""
f22 -- an env_cubemap inside a solid brush keeps only what it faces visible (#76).

Misplaced cubemaps are common on Valve maps (d1_trainstation_02 has three).  bspwalk used to
treat one viewpoint in solid as "every cluster is an eye cluster, skip the ray tier", so the
whole map came out visible.  Now such a viewpoint keeps visible only the faces it lies in front
of; tier A and tier B still hide the rest.

Geometry (x east, y north, z up; interior bounds, 16u walls):
    S  spawn room   x 0..512    y 0..512  z 0..192   east wall omitted (W closes it)
    W  wall brush   x 512..560  y -16..528 z -16..208  one world brush between S and V
    V  void room    x 560..816  y 0..512  z 0..192   west wall omitted (W), sealed, info_target
    env_cubemap at O = (536, 256, 96), inside W (24u from either face)
    hut in S        interior x 384..488 y 192..320 z 0..128 (floor is S's), outer x 368..504
                    y 176..336, roof top z=144; ONE doorway in its east wall (y 232..280,
                    z 64..128), facing W across an 8u gap (x 504..512) no player hull fits in
    info_player_start at (128, 256, 1).

The engine draws a cubemap in solid from O with W's sides facing away, so W does not hide
anything from it; the verifier models that (a viewpoint skips the solids that hold it).

must_visible:
    S inner sides: the player stands in S.
    V inner east, floor, ceiling, north, south: O is in front of each, and with W skipped the
        segment O -> any of them crosses only V's empty interior.  No player sees V (sealed), so
        the PVS hides them: a tier A that ignores the cubemap in solid calls them hidden (FAIL).
    hut west inner side (x=384, faces +x): O sees it through the doorway (O -> (384,256,96) runs
        at y=256, z=96 through the doorway and the gap).  No player does: a player's eye has
        x <= 496 (hull against W at 512) and x <= 352 or y outside 160..352 near the hut; any
        segment from there into the hut crosses its north or south wall (x 368..504, z 0..128)
        or lies behind the side's plane.  VVIS (measured) leaves the hut interior out of S's PVS,
        so this is the tier A case, like V.
    hut east outer sides (x=504, faces +x, across the 8u gap from W): O is in front of them and
        sees them through the gap.  Every player eye has x <= 496, behind their plane.  The gap is
        in S's PVS (measured), so only the face pass's rule for a viewpoint in solid keeps them:
        the tier B case.
must_hidden (strict: classing them visible FAILs, since a correct walker always hides them):
    V inner west (W's east side, x=560, faces +x): O at x=536 is behind it and no player sees
        into V: tier A.  Before #76 the cubemap in solid made every face visible.
    hut roof top (z=144, faces +z): every player eye is at most 85.33u up and O is at z=96,
        both below its plane: tier B (the PVS holds it).  Before #76 tier B did not run.
"""
import vmfgen

NAME = "f22_cubemap_in_solid"
DESCRIPTION = ("An env_cubemap inside a solid wall brush: the faces it lies in front of stay visible "
               "(a sealed room beyond the wall, a hut wall seen through a doorway only it looks into), "
               "while a PVS-hidden face behind it and a roof no eye is above stay hidden.")

CUBEMAP_ORIGIN = (536.0, 256.0, 96.0)
ROOF_TOP = 144.0


def build(m: vmfgen.Map) -> None:
    # every player eye and the cubemap are below the roof top, so its plane hides it from all
    assert vmfgen.MAX_EYE_ABOVE_FLOOR < ROOF_TOP and CUBEMAP_ORIGIN[2] < ROOF_TOP
    assert ROOF_TOP > vmfgen.CROUCH_JUMP_FEET + 64      # nobody stands on the roof
    # the gap between the hut and W is narrower than a hull
    assert 512 - 504 < 2 * 16

    spawn = m.room((0, 0, 0), (512, 512, 192), omit=("east",))
    m.entity("info_player_start", origin=(128, 256, 1), angles="0 0 0")
    wall = m.box((512, -16, -16), (560, 528, 208))
    void = m.room((560, 0, 0), (816, 512, 192), omit=("west",), marker=True)
    m.entity("env_cubemap", origin=CUBEMAP_ORIGIN, cubemapsize=0)
    hut = m.room((384, 192, 0), (488, 320, 128), floor=False,
                 openings=[("east", (232, 280, 64, 128))])

    m.expect_visible(spawn.all_inner(), "inner side of the room the player spawns in")
    m.expect_visible(wall.side("west"), "W's west side: the east wall of the spawn room")
    for face in ("east", "floor", "ceiling", "north", "south"):
        m.expect_visible(void.inner(face), f"sealed room V {face} inner side: the cubemap inside W lies in "
                         "front of it and W does not hide it from there; no player sees it")
    m.expect_visible(hut.inner("west"), "hut west inner side: the cubemap looks at it through the east "
                     "doorway; no player eye can reach that doorway's line of sight")
    m.expect_visible(hut.outer_all("east"), "hut east outer side across the 8u gap: only the cubemap is in "
                     "front of it (every player eye has x <= 496); the PVS holds the gap")

    m.expect_hidden(wall.side("east"), "V west inner side (W's east side): the cubemap is behind it, "
                    "no player sees into V", strict=True)
    m.expect_hidden(hut.outer("ceiling"), "hut roof top z=144: every eye and the cubemap are below it",
                    strict=True)

    m.expect_reach(spawn.floor_center(0), "spawn room floor: the player spawns on it")
    m.expect_no_reach(void.floor_center(0), "floor of the sealed room beyond W")

    m.note(f"env_cubemap at {CUBEMAP_ORIGIN}, inside the world brush x 512..560 between S and V.")

    # verify_expectations.py (#19)
    # the spawn room floor around the hut (its interior is unreachable: the only doorway faces the gap)
    m.verify_floor(0, 0, 512, 176, 0, "spawn room floor south of the hut")
    m.verify_floor(0, 336, 512, 512, 0, "spawn room floor north of the hut")
    m.verify_floor(0, 176, 368, 336, 0, "spawn room floor west of the hut")
    m.verify_viewpoint(CUBEMAP_ORIGIN, "env_cubemap inside W")
