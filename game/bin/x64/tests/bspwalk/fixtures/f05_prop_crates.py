"""
f05 -- a ledge reachable only by climbing a static prop, and a ledge that is
too high even with that prop under it.

The prop is a 40u-tall block: `models/props_c17/furniturestove001a.mdl` from
hl2_misc_dir.vpk, a prop_static with `solid 6` (VPhysics collision).

Why not the obvious wood crate (`models/props_junk/wood_crate001a.mdl`):
VBSP refuses it as a prop_static.  The model's keyvalues carry a `prop_data`
block without `allowstatic 1`, so the compile logs "Error! prop_static using
model ..., which must be used on a dynamic entity (i.e. prop_physics). Deleted."
and writes a BSP with NO prop in it (the compile still succeeds, so the fixture
would silently have no crate).  The only way round it is vbsp
`-allowdynamicpropsasstatic`, which the harness does not pass.  The stove has
`prop_data { allowstatic 1 }`, so it is a legal static prop.  (f15 uses the
wood crate as a prop_physics, where it is legal.)  Of the static-legal models
in the VPK this is the closest to a crate: about 40u tall with a flat top.

Collision (read from the .phy: IVP ledge vertices via c_point_offset, axes
(x, z, -y), 39.37 in/m; it fits inside the .mdl header hull
x -17.69..17.72, y -26.98..26.25, z -20.33..20.25):
    main hull  x -17.41..15.25  y -26.40..25.80  z -20.05..19.97 (12 vertices,
               chamfered at the bottom; the top face is planar, z 19.894 at the
               west edge rising to 19.968 at the east edge, slope 0.14 deg)
    side rail  x 16.21..17.44   y -26.70..25.97  z 15.75..16.98  (below the top)
With the origin 19.953u above the floor the prop is embedded 0.10u in it (no
sliver under it) and its top is 39.84..39.92u above the floor.  A walker that
cannot see prop collision (or reads it as a zero-thickness sheet) cannot stand on it.

Physics used (HL2, see f03): a standing jump lifts the feet 21u, a duck-jump
57u, a step is 18u.  The block top (39.84u) is out of reach of a standing jump
and in reach, with 17u to spare, of a duck-jump.  Highest eye over a floor ~85u.

    target ledge  top  80u : from the floor 80 > 57 * 1.15 = 65.6 (even the
                             design doc's reach_margin cannot get there), but
                             from the block top 80 - 39.84 = 40.2u < 57, so
                             block (duck-jump) then ledge (duck-jump).
    control ledge top 176u : from the block top 176 - 39.84 = 136.2u, more than
                             the CS:S duck-jump reach with margin applied,
                             93 * 1.15 = 107.  Unreachable in every profile.

The blocks sit against the ledge faces, embedded 0.16-0.19u, so the hop from
block top to ledge needs no horizontal travel beyond the lip.

How visibility is made certain: the same closed "hut" trick as f03/f04.  Each
ledge carries a box with ONE doorway, in the wall facing AWAY from the room.
For any eye whose x lies on the room side of the hut's door-side wall plane, the
straight segment to a point inside the hut keeps x strictly inside the hut's x
range, so it never enters the wall slab that holds the aperture and must cross
the solid shell instead.  The hut has no floor slab: the ledge top is its floor.

Layout (plan view, x east, y north; room interior 1152 x 512 x 384):

    x: 0 ........ 352 | 352 ..... 704 | 704 ............. 1152
       [CONTROL ledge]  floor strip    [TARGET ledge]
       top z=176        spawn (528,256)  top z=80
       hut, door WEST   block, x=369   block, x=687, hut, door EAST
                        (against ledge) (against ledge)

    control hut: interior x 96..192  y 176..336 z 176..272, door in its WEST wall
                 (y 224..288, z 176..256); roof top z=288.
    target hut:  interior x 848..944 y 176..336 z 80..176,  door in its EAST wall
                 (y 224..288, z 80..160); roof top z=192.
    There is a 160u run of ledge either side of the target hut and 192u east of it,
    so the doorway can be walked round to.  The ledges are 704u apart.

Expectations:
  visible  target-hut inner sides: climb the block (duck-jump), duck-jump onto
           the ledge, walk round the hut to its east doorway and step in.
  visible  controls: each ledge's face looking into the room (seen from the
           spawn), and the room ceiling.
  hidden   control-hut inner sides: ledge unreachable, and every reachable eye
           (floor strip, either block top, target ledge) has x >= 336 while the
           hut's only aperture is in its west wall at x 80..96, with every
           interior point at x > 96.
  hidden   both huts' roof tops: top faces are only visible from above their
           plane; the highest reachable eye is 80 + 85 = 165u, below both roofs
           (192 and 288).
"""
import vmfgen

NAME = "f05_prop_crates"
DESCRIPTION = ("Hut on an 80u ledge reachable only by climbing a static 40u block prop; control hut on a "
               "176u ledge too high even from its block.")

PROP_MODEL = "models/props_c17/furniturestove001a.mdl"
PROP_ORIGIN_Z = 19.953125                    # 1/64 multiple
PROP_TOP_LO = PROP_ORIGIN_Z + 19.8942      # lowest point of the planar top face, above the floor
PROP_TOP_HI = PROP_ORIGIN_Z + 19.9684      # highest point of the top face
PROP_X_MIN = -17.4076                      # west extent of the main hull
PROP_X_MAX = 17.4393                       # east extent (the side rail)
CS_CROUCH_JUMP_FEET = 93.0                 # CS:S duck-jump reach, as in f03

TARGET_TOP = 80.0
CONTROL_TOP = 176.0
X_CTRL, X_TARGET = 369.25, 686.75          # prop origins, y = 256.25


def build(m: vmfgen.Map) -> None:
    reach = vmfgen.CROUCH_JUMP_FEET
    hedge = 1.15                                                       # design doc reach_margin
    # The block is a duck-jump from the floor, with spare.
    assert PROP_TOP_HI > vmfgen.STAND_JUMP_FEET + 10
    assert PROP_TOP_HI < reach - 10
    # Target ledge: out of reach from the floor even with the hedge, in reach from the block top.
    assert TARGET_TOP > reach * hedge + 10
    assert TARGET_TOP - PROP_TOP_LO < reach - 10
    # Control ledge: out of reach from a block top in every profile, hedge included.
    assert CONTROL_TOP - PROP_TOP_LO > CS_CROUCH_JUMP_FEET * hedge + 20
    # Roof tops above every reachable eye (target ledge is the highest floor).
    assert TARGET_TOP + 112 > TARGET_TOP + vmfgen.MAX_EYE_ABOVE_FLOOR + 20
    # Blocks sit against the ledge faces, embedded by a hair (no sliver, no gap).
    assert 0.0 < 352.0 - (X_CTRL + PROP_X_MIN) < 0.5
    assert 0.0 < (X_TARGET + PROP_X_MAX) - 704.0 < 0.5

    shell = m.room((0, 0, 0), (1152, 512, 384))
    m.entity("info_player_start", origin=(528, 256, 1), angles="0 0 0")

    ledge_ctrl = m.box((0, 0, 0), (352, 512, CONTROL_TOP))
    ledge_target = m.box((704, 0, 0), (1152, 512, TARGET_TOP))

    hut_ctrl = m.room((96, 176, CONTROL_TOP), (192, 336, CONTROL_TOP + 96), floor=False,
                      openings=[("west", (224, 288, CONTROL_TOP, CONTROL_TOP + 80))])
    hut_target = m.room((848, 176, TARGET_TOP), (944, 336, TARGET_TOP + 96), floor=False,
                        openings=[("east", (224, 288, TARGET_TOP, TARGET_TOP + 80))])

    for x in (X_CTRL, X_TARGET):
        m.entity("prop_static", origin=(x, 256.25, PROP_ORIGIN_Z), model=PROP_MODEL,
                 angles="0 0 0", solid="6", skin="0")

    m.expect_visible(hut_target.all_inner(),
                     "target-hut inner side: ledge top 80u is reachable only by prop (duck-jump) then ledge, "
                     "doorway faces away from the room")
    m.expect_visible(ledge_target.side("west"), "control: ledge face seen from the spawn point")
    m.expect_visible(ledge_ctrl.side("east"), "control: ledge face seen from the spawn point")
    m.expect_visible(shell.inner("ceiling"), "control: room ceiling")

    m.expect_hidden(hut_ctrl.all_inner(),
                    "control-hut inner side: ledge top 176u is 136u above its prop, beyond any duck-jump, "
                    "and the doorway faces away")
    m.expect_hidden(hut_target.outer("ceiling"),
                    "target-hut roof top at z=192: above every reachable eye (max 165)")
    m.expect_hidden(hut_ctrl.outer("ceiling"),
                    "control-hut roof top at z=288: above every reachable eye")

    # Feet points.  The prop tops are 39.84..39.92u up (tilted 0.14 deg), inside the 2u z tolerance.
    for x, which in ((X_TARGET, "target"), (X_CTRL, "control")):
        m.expect_reach((x, 256.25, PROP_TOP_LO), f"top of the {which} block prop: a duck-jump from the floor")
    m.expect_reach((1050, 64, TARGET_TOP), "target ledge top: duck-jump from the block top (40.2u)")
    m.expect_reach((896, 256, TARGET_TOP), "target hut floor: walk round to its east doorway on the 80u ledge")
    m.expect_no_reach((300, 64, CONTROL_TOP), "control ledge top: 136u above its block, beyond any duck-jump")
    m.expect_no_reach((144, 256, CONTROL_TOP), "control hut floor on the unreachable 176u ledge")

    m.note(f"Prop top {PROP_TOP_LO:.2f}..{PROP_TOP_HI:.2f}u (collision read from the .phy). Target ledge "
           f"{TARGET_TOP}u (prop->ledge {TARGET_TOP - PROP_TOP_LO:.1f}u), control ledge {CONTROL_TOP}u "
           f"(prop->ledge {CONTROL_TOP - PROP_TOP_LO:.1f}u). Duck-jump feet reach {reach}u. "
           f"wood_crate001a is rejected by VBSP as a prop_static (no allowstatic).")

    # verify_expectations.py (#19): props are not solids here, so a prop top is a floor
    m.verify_floor(352, 0, 704, 512, 0, "room floor between the ledges")
    for x in (X_CTRL, X_TARGET):
        m.verify_floor(x + PROP_X_MIN, 256.25 - 16, x + PROP_X_MAX, 256.25 + 16, PROP_TOP_HI, "block prop top")
    m.verify_floor(704, 0, 1152, 512, TARGET_TOP, "target ledge top, the target hut's inside included")
