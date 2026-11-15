"""
f03 -- a ledge reachable only by a crouch-jump, and a ledge that is too high.

Physics used (HL2, Source SDK 2013 gamemovement): sv_gravity 600, jump speed
sqrt(2*600*21) ~ 158.7 u/s, so a standing jump lifts the feet at most 21u.
Ducking in mid-air lifts the feet a further 72-36 = 36u, so a duck-jump puts
the feet at most 57u above the take-off floor.  Highest eye above a floor:
max(21.3 + 64, 57 + 28) ~ 85u.  Walking up a step needs <= 18u.

Natural round heights of 60u (reachable) and 70u (not) do not work: by these numbers
60u is already out of reach (57u), so the heights are set to margins that stay
certain even if the constants are a few units off, or the walker adds its
deliberate 15% over-reach:
    low ledge   top at  48u : > 21.3 + 18 = 39.3 (needs a duck-jump, no stand-jump),
                              < 57 - 9 (reachable with room to spare)
    high ledge  top at 112u : > 57 * 1.15 = 65.6, and > 93 * 1.15 = 107 (the CS:S
                              duck-jump reach), so unreachable in every profile
                              that is in the design doc.

How visibility is made certain (no grazing sight lines, no height trickery):
each ledge carries a "hut", a closed box standing on it with one doorway in the
wall FACING AWAY from the room.  For any eye west of the hut's east wall plane
(low ledge hut; mirrored for the high one), every straight segment to a point
inside the hut crosses the hut shell, because the only aperture is on the far
side and a segment never reaches a plane that lies beyond both its endpoints.
The hut interior is therefore visible only to eyes that are inside the hut or
have walked round to the doorway side -- which needs the ledge.

Layout (plan view, x east, interior bounds, room 1536 x 512 x 320):

    x:  0 ......... 448 ........ 1088 ........ 1536
        [ HIGH ledge ]   floor     [ LOW ledge ]
        top z=112                  top z=48
        hut, door faces WEST       hut, door faces EAST

    spawn: (768, 256, 1) on the floor between them.
    low  hut: interior x 1216..1312 y 176..336 z 48..144, door in its EAST wall
              (y 224..288, z 48..128); roof top at z=160.
    high hut: interior x 144..240   y 176..336 z 112..208, door in its WEST wall
              (y 224..288, z 112..192); roof top at z=224.
    The ledges are 640u apart, so one cannot be used as a step to the other
    (a duck-jump crosses ~200u at most).

The huts have no floor slab of their own: the ledge top is their floor.

Expectations:
  visible  low-hut inner sides (ceiling, west/north/south walls, the three east
           wall pieces around the door): a player on the low ledge walks round
           the hut to the east doorway and stands inside.
  visible  controls: the face of each ledge that looks into the room, seen from
           the spawn point; the ceiling of the room.
  hidden   high-hut inner sides: the high ledge is unreachable (112u), and by
           the shell argument no reachable eye can see in: every reachable eye
           has x >= 464 (the floor starts at x=448 and a hull is 16 off the
           wall) while the hut's only aperture is in its west wall, at x 128..144.
  hidden   both huts' roof tops (z=160 / 224): top faces are only visible from
           above their plane; the highest reachable eye is 48 + 85 = 133u.
"""
import vmfgen

NAME = "f03_crouchjump_ledges"
DESCRIPTION = ("Ledge at 48u reachable only by a duck-jump, ledge at 112u unreachable; "
               "each carries a hut whose interior is visible only from the ledge.")

LOW_TOP = 48.0
HIGH_TOP = 112.0


def build(m: vmfgen.Map) -> None:
    # Margins, derived from the physics constants (see module docstring).
    assert LOW_TOP > vmfgen.STAND_JUMP_FEET + vmfgen.STEP_HEIGHT + 5     # no stand-jump
    assert LOW_TOP < vmfgen.CROUCH_JUMP_FEET - 5                          # duck-jump reaches
    assert HIGH_TOP > vmfgen.CROUCH_JUMP_FEET * 1.15 + 20                 # out of reach even with margin
    roof_low, roof_high = LOW_TOP + 112, HIGH_TOP + 112
    assert roof_low > LOW_TOP + vmfgen.MAX_EYE_ABOVE_FLOOR + 20           # roof above any reachable eye

    shell = m.room((0, 0, 0), (1536, 512, 320))
    m.entity("info_player_start", origin=(768, 256, 1), angles="0 0 0")

    ledge_low = m.box((1088, 0, 0), (1536, 512, LOW_TOP))
    ledge_high = m.box((0, 0, 0), (448, 512, HIGH_TOP))

    hut_low = m.room((1216, 176, LOW_TOP), (1312, 336, LOW_TOP + 96), floor=False,
                     openings=[("east", (224, 288, LOW_TOP, LOW_TOP + 80))])
    hut_high = m.room((144, 176, HIGH_TOP), (240, 336, HIGH_TOP + 96), floor=False,
                      openings=[("west", (224, 288, HIGH_TOP, HIGH_TOP + 80))])

    m.expect_visible(hut_low.all_inner(),
                     "low-hut inner side: ledge is duck-jump reachable, hut doorway faces away from the room")
    m.expect_visible(ledge_low.side("west"), "control: ledge face seen from the spawn point")
    m.expect_visible(ledge_high.side("east"), "control: ledge face seen from the spawn point")
    m.expect_visible(shell.inner("ceiling"), "control: room ceiling")

    m.expect_hidden(hut_high.all_inner(),
                    "high-hut inner side: ledge top at 112u is beyond any jump, and the doorway faces away")
    m.expect_hidden(hut_low.outer("ceiling"),
                    "low-hut roof top at z=160: above every reachable eye (max 133)")
    m.expect_hidden(hut_high.outer("ceiling"),
                    "high-hut roof top at z=224: above every reachable eye")

    # Reach (feet points; ledge tops away from the huts, and each hut's floor, which is the ledge top).
    m.expect_reach((1150, 64, LOW_TOP), "low ledge top: a duck-jump from the room floor (48u <= 57u)")
    m.expect_reach((1264, 256, LOW_TOP), "low-hut floor: walk round to its east doorway on the 48u ledge")
    m.expect_reach((768, 256, 0), "room floor at the spawn point")
    m.expect_no_reach((300, 64, HIGH_TOP), "high ledge top: 112u above the floor, beyond any duck-jump")
    m.expect_no_reach((192, 256, HIGH_TOP), "high-hut floor on the unreachable 112u ledge")

    m.note(f"Low ledge {LOW_TOP}u, high ledge {HIGH_TOP}u. Duck-jump feet reach is "
           f"{vmfgen.CROUCH_JUMP_FEET}u, standing jump {vmfgen.STAND_JUMP_FEET}u "
           f"(the original 60/70 pair is wrong for HL2: 60 is already out of reach).")

    # verify_expectations.py (#19)
    m.verify_floor(448, 0, 1088, 512, 0, "room floor between the ledges")
    m.verify_floor(1088, 0, 1536, 512, LOW_TOP, "low ledge top, the low hut's inside included")
