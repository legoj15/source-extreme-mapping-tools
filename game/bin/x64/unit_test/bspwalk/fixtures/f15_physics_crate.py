"""
f15 -- a balcony reachable only by climbing a prop_physics crate, and a balcony
that is too high even with that crate.

Prop: `models/props_junk/wood_crate001a.mdl` (hl2_misc_dir.vpk) as a
`prop_physics` in the ENTITY lump (no game-lump entry; contrast f05, where the
crate is not legal as a prop_static, see that fixture).  The design doc's
policy: an entity prop is solid for standing on at its spawn origin and never an
occluder.  Spawnflags 1 (Start Asleep) so it stays where it is placed.

Collision hull (read from the .phy: IVP ledge vertices via c_point_offset, axes
(x, z, -y), 39.37 in/m; the .mdl header hull is +-20.3 x +-20.2 x -20.2..20.0):
    a plain box  x -20.03..19.94   y -19.87..19.88   z -19.95..19.69
With the origin 20u above the floor the crate rests on it (0.05u clear, so
vphysics does not have to push it out of the floor) and its top is 39.69u
above the floor.  The crate's north face is 0.12u from the balcony face.

Physics used (HL2, see f03): a standing jump lifts the feet 21u, a duck-jump
57u, a step is 18u.  Crate top 39.69u: duck-jump from the floor with 17u to
spare.  Highest eye over a floor ~85u.

    target balcony top  80u : from the floor 80 > 57 * 1.15 = 65.6, but from the
                              crate top 80 - 39.69 = 40.3u < 57.
    control balcony top 192u: even from the crate top it is 152.3u up, past the
                              CS:S duck-jump reach with margin applied
                              (93 * 1.15 = 107).  192 also exceeds
                              80 + 39.69 + 57 = 176.7, the best a crate carried
                              onto the target balcony could do.

There is exactly ONE crate in the map, so crate stacking (a limitation the
design doc accepts) cannot arise, and shoving the crate across the room does not
open the control balcony: crate top + any duck-jump is at most 96.7u (146.7u at
the CS:S numbers with margin), below 192.

How visibility is made certain: the closed "hut" trick of f03/f04, here with the
doorways on the north/south walls.  Each hut has ONE doorway, in the wall
facing away from the room.  Every reachable eye on the room side of the hut has
a y coordinate that keeps the whole segment to any hut-interior point on the
inner side of the door-wall slab, so the segment cannot pass through the
aperture and must cross the solid shell.  No floor slab: the balcony top is the
hut floor.

Layout (plan view, x east, y north; room interior 640 x 1152 x 384):

    y 1152 +--------------------------------+
           | TARGET balcony, top z=80       |  hut interior x 224..384
           |   hut: door in its NORTH wall  |    y 848..944  z 80..176
           |         (x 272..336, z 80..160)|    roof top z=192
     704   +--------------------------------+
           |   crate (480, 684)             |
           |   floor strip, spawn (320,480) |
     256   +--------------------------------+
           | CONTROL balcony, top z=192     |  hut interior x 224..384
           |   hut: door in its SOUTH wall  |    y 64..160  z 192..288
       0   +--------------------------------+    door x 272..336, z 192..272
                                                 roof top z=304
    There is 192u of balcony north of the target hut for the player to walk to
    the doorway.  The balconies are 448u apart.

Expectations:
  visible  target-hut inner sides: climb the crate, duck-jump onto the balcony,
           walk round to the north doorway and step in.
  visible  controls: each balcony's face looking into the room, and the ceiling.
  hidden   control-hut inner sides: balcony unreachable; every reachable eye has
           y >= 272 while the aperture is in the south wall at y 48..64 and
           every interior point has y > 64.
  hidden   both huts' roof tops: top faces are only visible from above their
           plane; the highest reachable eye is 80 + 85 = 165u, below the roofs
           (192 and 304).
"""
import vmfgen

NAME = "f15_physics_crate"
DESCRIPTION = ("Hut on an 80u balcony reachable only by climbing a prop_physics crate at its spawn origin; "
               "control hut on a 192u balcony that is out of reach.")

CRATE_MODEL = "models/props_junk/wood_crate001a.mdl"
CRATE_MIN = (-20.03, -19.87, -19.95)        # collision box from the .phy, relative to the origin
CRATE_MAX = (19.94, 19.88, 19.69)
CRATE_ORIGIN = (480.0, 684.0, 20.0)
CRATE_TOP = CRATE_ORIGIN[2] + CRATE_MAX[2]  # 39.69 above the floor
CS_CROUCH_JUMP_FEET = 93.0                  # CS:S duck-jump reach, as in f03

TARGET_TOP = 80.0
CONTROL_TOP = 192.0
Y_TARGET_FACE, Y_CONTROL_FACE = 704.0, 256.0


def build(m: vmfgen.Map) -> None:
    reach = vmfgen.CROUCH_JUMP_FEET
    hedge = 1.15                                                       # design doc reach_margin
    assert CRATE_TOP > vmfgen.STAND_JUMP_FEET + 10 and CRATE_TOP < reach - 10
    # Target: out of reach from the floor even with the hedge, in reach from the crate top.
    assert TARGET_TOP > reach * hedge + 10
    assert TARGET_TOP - CRATE_TOP < reach - 10
    # Control: out of reach from the crate in every profile, and from a crate carried onto the target balcony.
    assert CONTROL_TOP - CRATE_TOP > CS_CROUCH_JUMP_FEET * hedge + 20
    assert CONTROL_TOP > TARGET_TOP + CRATE_TOP + reach
    assert TARGET_TOP + 112 > TARGET_TOP + vmfgen.MAX_EYE_ABOVE_FLOOR + 20      # roof above any reachable eye
    # The crate rests on the floor and nearly touches the balcony face.
    assert 0.0 < CRATE_ORIGIN[2] + CRATE_MIN[2] < 0.1
    assert 0.0 < Y_TARGET_FACE - (CRATE_ORIGIN[1] + CRATE_MAX[1]) < 0.5

    shell = m.room((0, 0, 0), (640, 1152, 384))
    m.entity("info_player_start", origin=(320, 480, 1), angles="0 0 0")

    bal_target = m.box((0, Y_TARGET_FACE, 0), (640, 1152, TARGET_TOP))
    bal_control = m.box((0, 0, 0), (640, Y_CONTROL_FACE, CONTROL_TOP))

    hut_target = m.room((224, 848, TARGET_TOP), (384, 944, TARGET_TOP + 96), floor=False,
                        openings=[("north", (272, 336, TARGET_TOP, TARGET_TOP + 80))])
    hut_control = m.room((224, 64, CONTROL_TOP), (384, 160, CONTROL_TOP + 96), floor=False,
                         openings=[("south", (272, 336, CONTROL_TOP, CONTROL_TOP + 80))])

    m.entity("prop_physics", origin=CRATE_ORIGIN, model=CRATE_MODEL, angles="0 0 0", spawnflags="1")

    m.expect_visible(hut_target.all_inner(),
                     "target-hut inner side: balcony top 80u is reachable only by crate (duck-jump) then balcony, "
                     "doorway faces away from the room")
    m.expect_visible(bal_target.side("south"), "control: balcony face seen from the spawn point")
    m.expect_visible(bal_control.side("north"), "control: balcony face seen from the spawn point")
    m.expect_visible(shell.inner("ceiling"), "control: room ceiling")

    m.expect_hidden(hut_control.all_inner(),
                    "control-hut inner side: balcony top 192u is 152u above the crate, beyond any duck-jump, "
                    "and the doorway faces away")
    m.expect_hidden(hut_target.outer("ceiling"),
                    "target-hut roof top at z=192: above every reachable eye (max 165)")
    m.expect_hidden(hut_control.outer("ceiling"),
                    "control-hut roof top at z=304: above every reachable eye")

    m.expect_reach((CRATE_ORIGIN[0], CRATE_ORIGIN[1], CRATE_TOP), "crate top: a duck-jump from the floor")
    m.expect_reach((100, 1000, TARGET_TOP), "target balcony top: duck-jump from the crate top (40.3u)")
    m.expect_reach((304, 896, TARGET_TOP), "target hut floor: walk round to its north doorway")
    m.expect_no_reach((100, 200, CONTROL_TOP), "control balcony top: 152u above the crate")
    m.expect_no_reach((304, 112, CONTROL_TOP), "control hut floor on the unreachable 192u balcony")

    m.note(f"One prop_physics crate (Start Asleep), top {CRATE_TOP:.2f}u. Target balcony {TARGET_TOP}u "
           f"(crate->balcony {TARGET_TOP - CRATE_TOP:.1f}u), control balcony {CONTROL_TOP}u "
           f"(crate->balcony {CONTROL_TOP - CRATE_TOP:.1f}u). Duck-jump feet reach {reach}u.")
