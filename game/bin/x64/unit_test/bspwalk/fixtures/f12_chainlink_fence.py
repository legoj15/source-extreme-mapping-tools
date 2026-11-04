"""
f12 -- a chain-link prop fence across a room, with an alcove and a hut behind it.

models/props_c17/fence03a.mdl (checked in hl2_misc_dir.vpk with vpk_reader.py):
its material models/props_c17/fence_alpha.vmt has "$AlphaTest" 1 and surfaceprop
"chainlink", so it is a real chain-link sheet you can see straight through.  Its
header hull is x -2.63..2.22, y -130.41..130.44, z -54.36..54.36: a 4.8u thick
sheet in the model's YZ plane, 260.9u wide and 108.7u tall.

A prop never occludes sight (design doc, Stage 5), so everything behind the fence is
visible through it.  The Python walker uses prop collision as a sight occluder (known
defect, design doc table), but this fixture cannot show that against it: that walker's
reachability ignores both the prop's collision and PLAYERCLIP brushes (a leaf holding only
a clip brush has contents 0, and collision.py hull_intersects quick-rejects it), so it
walks into the east half (786 reachable cells; the west half holds about 430) and sees the
faces from there.  In the baseline this fixture therefore shows up as must_hidden "missed"
(hut B), not as faces hidden behind the fence.

Geometry (interior bounds, 16u walls; x east, y north):
    R  room       x 0..512      y 0..512    z 0..192   info_player_start (128,256,1)
    fence         x ~253.4..258.2 (origin x=256), y -2.4..514.4 (two overlapping fence03a
                  at y=128 and y=384, ends buried 2.4u in the side walls), z -0.11..108.6
                  (origin z=54.25, bottom 0.11 below the floor)
    playerclip    x 240..272    y 0..512    z 0..192   around the fence
    alcove        x 528..656    y 288..416  z 0..128   carved into R's east wall (opening
                  y 288..416, z 0..128 in the wall slab x 512..528)
    hut B         interior x 320..400  y 64..192  z 0..96   walls 16 (outer x 304..416,
                  y 48..208, roof top z=112), no floor of its own, ONE doorway in its
                  EAST wall (y 112..144, z 0..80), 96u gap between the hut and R's east wall

Why nobody gets past the fence: it spans the room from wall to wall and from the floor
to 108.6u.  The best feet height any jump reaches is 57u (CROUCH_JUMP_FEET), so a
player can neither climb the fence (108.6 is far above 57 * 1.15 = 66) nor pass over it:
the feet would have to clear 108.6.  Its hull is 4.8u thick, so no 32u hull squeezes by.
A TOOLS/TOOLSPLAYERCLIP wall (x 240..272, floor to ceiling, wall to wall) also stands
around the fence, so the barrier does not depend on how a walker models prop collision
(the crate fixtures test that); this fixture isolates sight THROUGH the prop.  The clip
is 32u thick on purpose: with the 16u hull margin each side it spans four cells of a 16u
grid, so a grid walker with a 3-cell (48u) jump cannot hop it.
The clip does not block sight.  Every reachable eye therefore has x <= 240 - 16 = 224
(hull half-width 16), in the west half, with y 16..496 and z 28..85.

The alcove interior is visible only through the fence plus the opening, the east wall's
inner side is visible only through the fence, and hut B is a closed box: its doorway is
in the east wall, facing away from every reachable eye.

must_visible, each with an explicit line from a reachable eye:
    alcove back wall (x=656)          from (200,352,64) to (656,352,64): crosses the opening
                                      at x 512..528 with y=352, z=64, both inside the opening
    alcove side wall (y=288)          from (224,352,64) to (600,288,64): y is 303.0 at x=512 and
                                      300.3 at x=528, inside 288..416
    alcove floor                      from (224,352,64) to (600,352,0): z 15.0 .. 12.3 at the opening
    alcove ceiling (z=128)            from (224,352,28) to (600,352,128): z 104.6 .. 108.9
    R east wall pieces (x=512, faces -x)  from (200,256,64) to (512,256,64): y=256 clears hut B
                                      (y 48..208) and sits left of the alcove opening (y 288..416)
    hut B west/north/south outer faces    west: from (200,128,64); south (y=48, faces -y):
                                      from (200,20,64) to (360,48,50); north (y=208, faces +y):
                                      from (200,300,64) to (360,208,50)
    R west wall (control)             the player's own side
must_hidden, each a proof:
    hut B inner sides (7 pieces)      every segment from a reachable eye (x <= 224) to a point
                                      in the hut interior (x < 400) stays at x < 400, so it never
                                      reaches the east wall slab x 400..416 where the only
                                      aperture is.  It must cross the shell.
    hut B east outer sides, faces +x  front half-space is x > 416; every eye has x <= 224
    hut B roof top (z=112, faces +z)  front half-space is z > 112; highest eye is 85
Hut B's hiddenness depends on the barrier: with the gap of 96u (> 32) a player on the east
side could walk behind the hut and look in through its door.
VVIS facts measured on the compiled BSP: the prop does not affect the PVS.  The PVS of the
west half's clusters excludes hut B's interior (its 7 inner sides are PVS-hidden, tier A),
but contains hut B's east outer sides and roof top, so those 4 are provable only by rays
(tier B).  Everything under must_visible is PVS-visible.
"""
import vmfgen

NAME = "f12_chainlink_fence"
DESCRIPTION = ("Chain-link prop_static fence spanning a room; an alcove and a hut behind it are "
               "visible through the fence, the hut interior is not.")

FENCE_MODEL = "models/props_c17/fence03a.mdl"
FENCE_X = 256.0
FENCE_Z = 54.25                 # model origin is the hull centre; hull half-height is 54.36
FENCE_HALF_H = 54.362
FENCE_HALF_W = 130.4
FENCE_THICK_MAX = 2.63          # hull x -2.63..2.22 around the origin
CLIP_MAT = "TOOLS/TOOLSPLAYERCLIP"


def build(m: vmfgen.Map) -> None:
    fence_top = FENCE_Z + FENCE_HALF_H
    assert FENCE_Z - FENCE_HALF_H < 0 < 0.2                                     # bottom buried in the floor
    assert fence_top > vmfgen.CROUCH_JUMP_FEET * 1.15 + 30                      # cannot be climbed
    assert 128 - FENCE_HALF_W < 0 and 384 + FENCE_HALF_W > 512                  # ends reach both side walls
    assert 128 + FENCE_HALF_W > 384 - FENCE_HALF_W                              # the two sheets overlap

    room = m.room((0, 0, 0), (512, 512, 192),
                  openings=[("east", (288, 416, 0, 128))])
    m.entity("info_player_start", origin=(128, 256, 1), angles="0 0 0")

    for y in (128.0, 384.0):
        m.entity("prop_static", origin=(FENCE_X, y, FENCE_Z), model=FENCE_MODEL,
                 angles="0 0 0", solid=6, skin=0)

    # The barrier proper, so that no walker's prop-collision model decides who gets past.
    clip_x0, clip_x1 = 240.0, 272.0
    assert clip_x0 < FENCE_X - FENCE_THICK_MAX and clip_x1 > FENCE_X + 2.3
    m.box((clip_x0, 0, 0), (clip_x1, 512, 192), material=CLIP_MAT, tag="playerclip")

    alcove = m.room((528, 288, 0), (656, 416, 128), omit=("west",))
    hut_b = m.room((320, 64, 0), (400, 192, 96), floor=False,
                   openings=[("east", (112, 144, 0, 80))])

    m.expect_visible(alcove.all_inner(),
                     "alcove inner side: seen only through the fence and the 128u opening")
    m.expect_visible(room.inner_all("east"),
                     "east wall of the room: seen only through the fence")
    m.expect_visible(hut_b.outer("west"), "hut B west face: seen only through the fence")
    m.expect_visible(hut_b.outer("north"), "hut B north face (+y): seen from y>208 through the fence")
    m.expect_visible(hut_b.outer("south"), "hut B south face (-y): seen from y<48 through the fence")
    m.expect_visible(room.inner("west"), "control: the player's own side of the room")

    m.expect_hidden(hut_b.all_inner(),
                    "hut B interior: its only doorway is in the east wall, facing away from every eye")
    m.expect_hidden(hut_b.outer_all("east"),
                    "hut B east face (+x): front half-space x>416, every reachable eye has x<=224")
    m.expect_hidden(hut_b.outer("ceiling"),
                    "hut B roof top z=112: above the highest eye (85)")

    m.note("fence03a is alpha-tested chain-link (fence_alpha.vmt: $AlphaTest 1, surfaceprop chainlink). "
           "Prop solidity 6 (VPhysics); a playerclip wall in the fence makes the barrier independent of "
           "prop-collision modelling.")

    # verify_expectations.py (#19): the clip wall keeps every eye at x <= 224
    m.verify_floor(0, 0, clip_x0, 512, 0, "room floor west of the fence")
