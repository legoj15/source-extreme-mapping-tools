"""
f22 -- which env_cubemaps bspwalk lets be viewpoints (#82).

Every env_cubemap sample (lump 42) used to be a viewpoint eye. One whose capture camera sits
inside a brush made the whole visibility pass give up on tier B, so bspwalk now drops a cubemap

  * inside a brush: its leaf has no cluster, or the near cube of its six 90 degree frustums
    (origin +-7u, the engine's near plane) overlaps the interior of a world brush with
    CONTENTS_SOLID, func_detail included; or
  * that sees a toolsnodraw face: rays through the pixel centres of the six cube faces whose
    nearest opaque world brush entry is through a TOOLS/TOOLSNODRAW side that is not buried
    (the point 1/8u outside it is in open space).

A cubemap whose near cube overlaps a static prop's collision only draws a warning; it stays a
viewpoint. VBSP stores cubemap origins truncated to whole units, so all origins here are whole.

Geometry (x east, y north, z up; interior bounds, 16u walls):
    P  player room   x 0..768  y 0..512  z 0..256   info_player_start (96,256,1)
       pillar        x 320..416  y 208..304  z 0..200
                     cubemap (a) (368,256,100) in its middle: its leaf is solid  -> "solid"
                     its top face (z 200) is above every eye (max ~85) and in the PVS: only tier B
                     hides it. A walker that keeps a viewpoint in the pillar turns tier B off for the
                     whole map (every face stays visible), so the top is the "tier B still ran" check
                     (must_hidden misses are reported, not failed)
       west wall     cubemap (b) (3,128,128): in open space, 3u from the wall slab (x -16..0), so
                     its near cube (x -4..10) cuts into it                         -> "solid"
                     cubemap (g) (7,384,128): 7u from it, the near cube (x 0..14) only touches the
                     slab, which is not inside                                     -> "ok"
       detail box    func_detail x 600..664  y 380..444  z 0..64, cubemap (f) (632,412,32) inside it.
                     Detail brushes split no leaf, so the leaf test passes; only the brush test
                     catches it                                                      -> "solid"
       stove prop    models/props_c17/furniturestove001a.mdl at (560,256.25,19.953125) (f05's block),
                     cubemap (e) (560,256,20) inside its collision: stays a viewpoint, with a
                     static prop warning                                             -> "ok" + props
    N  sealed room   x 896..1152  y 0..256  z 0..192   marker; cubemap (c) (960,128,96)
       box           x 1072..1120  y 100..156  z 0..64, TOOLS/TOOLSNODRAW top: the cubemap looks down
                     onto the face the engine does not draw                          -> "nodraw"
                     (with the cubemap dropped nothing in N is a viewpoint: all of N is PVS-hidden)
    M  sealed room   x 1312..1568  y 0..256  z 0..192  marker; cubemap (d) (1408,128,96)
       box           x 1504..1568  y 96..160  z 0..64, nodraw EAST and BOTTOM sides, flush against the
                     east wall and the floor: both are buried (solid right outside them) and no ray
                     reaches them anyway                                             -> "ok"
                     M's inner sides are must_visible: tier A hides them if the cubemap is not an eye
"""
import vmfgen

NAME = "f22_cubemap_rules"
DESCRIPTION = ("env_cubemaps in a pillar, 3u from a wall and in a func_detail are dropped as viewpoints, one "
               "looking at a toolsnodraw face too; one in a static prop and one beside flush nodraw sides stay.")

STOVE_MODEL = "models/props_c17/furniturestove001a.mdl"       # f05's block: allowstatic, solid 6
STOVE_ORIGIN = (560.0, 256.25, 19.953125)

CUBEMAP_PILLAR = (368.0, 256.0, 100.0)
CUBEMAP_WALL = (3.0, 128.0, 128.0)
CUBEMAP_TOUCH = (7.0, 384.0, 128.0)
CUBEMAP_DETAIL = (632.0, 412.0, 32.0)
CUBEMAP_PROP = (560.0, 256.0, 20.0)
CUBEMAP_NODRAW = (960.0, 128.0, 96.0)
CUBEMAP_FLUSH = (1408.0, 128.0, 96.0)


def build(m: vmfgen.Map) -> None:
    room = m.room((0, 0, 0), (768, 512, 256))
    m.entity("info_player_start", origin=(96, 256, 1), angles="0 0 0")

    # (a) the pillar
    pillar = m.box((320, 208, 0), (416, 304, 200))
    m.entity("env_cubemap", origin=CUBEMAP_PILLAR, cubemapsize=0)
    # (b) near the west wall
    m.entity("env_cubemap", origin=CUBEMAP_WALL, cubemapsize=0)
    m.entity("env_cubemap", origin=CUBEMAP_TOUCH, cubemapsize=0)
    # (f) inside a func_detail box
    detail = m.box((600, 380, 0), (664, 444, 64))
    m.brush_entity("func_detail", [detail])
    m.entity("env_cubemap", origin=CUBEMAP_DETAIL, cubemapsize=0)
    # (e) inside a static prop
    m.entity("prop_static", origin=STOVE_ORIGIN, model=STOVE_MODEL, angles="0 0 0", solid="6", skin="0")
    m.entity("env_cubemap", origin=CUBEMAP_PROP, cubemapsize=0)

    # (c) sees a nodraw face
    nodraw_room = m.room((896, 0, 0), (1152, 256, 192), marker=True)
    m.box((1072, 100, 0), (1120, 156, 64), face_materials={"top": vmfgen.MAT_NODRAW})
    m.entity("env_cubemap", origin=CUBEMAP_NODRAW, cubemapsize=0)

    # (d) nodraw sides that are only buried
    flush_room = m.room((1312, 0, 0), (1568, 256, 192), marker=True)
    m.box((1504, 96, 0), (1568, 160, 64),
          face_materials={"east": vmfgen.MAT_NODRAW, "bottom": vmfgen.MAT_NODRAW})
    m.entity("env_cubemap", origin=CUBEMAP_FLUSH, cubemapsize=0)

    m.expect_cubemap(CUBEMAP_PILLAR, "solid", why="origin in the pillar: a solid leaf")
    m.expect_cubemap(CUBEMAP_WALL, "solid", why="open space, but the near cube (7u) cuts the 3u-away wall slab")
    m.expect_cubemap(CUBEMAP_TOUCH, "ok", why="the near cube (7u) just touches the wall slab: touching is not inside")
    m.expect_cubemap(CUBEMAP_DETAIL, "solid", why="inside a func_detail brush: the leaf is open, the brush test decides")
    m.expect_cubemap(CUBEMAP_PROP, "ok", props=True, why="inside the stove prop: a warning, still a viewpoint")
    m.expect_cubemap(CUBEMAP_NODRAW, "nodraw", why="looks down onto the box's toolsnodraw top")
    m.expect_cubemap(CUBEMAP_FLUSH, "ok", why="the box's nodraw sides are flush against the wall and floor: buried")

    m.expect_visible(room.all_inner(), "player room, the player stands in it")
    for side in ("west", "east", "north", "south"):
        m.expect_visible(pillar.side(side), "pillar side, seen from the floor around it")
    m.expect_visible(flush_room.all_inner(),
                     "room M: its cubemap is a viewpoint, so M is in the eye clusters (tier A would hide it otherwise)")
    m.expect_hidden(pillar.side("top"),
                    "pillar top z=200: above every eye (max ~85). Hidden only if tier B ran, i.e. no viewpoint "
                    "is left in solid")
    m.expect_hidden(nodraw_room.all_inner(),
                    "room N: its only viewpoint (the cubemap) is dropped, no other eye can see in")

    m.note(f"Cubemaps: pillar {CUBEMAP_PILLAR}, wall {CUBEMAP_WALL}, detail {CUBEMAP_DETAIL}, prop {CUBEMAP_PROP}, "
           f"nodraw {CUBEMAP_NODRAW}, flush {CUBEMAP_FLUSH}, touching {CUBEMAP_TOUCH}. Near cube 7u, 64x64 rays per face, 1 nodraw ray disqualifies.")

    # verify_expectations.py (#19): the floor of P (the hull test drops lattice eyes in the pillar), the two
    # cubemaps that stay viewpoints
    m.verify_floor(0, 0, 768, 512, 0, "player room floor")
    m.verify_viewpoint(CUBEMAP_PROP, "env_cubemap in the stove prop (a viewpoint)")
    m.verify_viewpoint(CUBEMAP_TOUCH, "env_cubemap whose near cube touches the west wall (a viewpoint)")
    m.verify_viewpoint(CUBEMAP_FLUSH, "env_cubemap in room M (a viewpoint)")
