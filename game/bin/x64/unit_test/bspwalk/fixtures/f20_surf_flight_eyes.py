"""
f20 -- a room seen only by a surfer in mid-flight, through a slot at the top of a flight (#26).

f18's ramp again (n.z 0.625, a slide). A player carrying speed surfs up it and off its top edge
(vmax = 640 * 1.15 = 736 u/s; the flight's vertical part 0.78 vmax gives an apex 275u above the
launch, at about 440u east of the edge). Here the top edge meets a flat platform at the ramp's own
height, so the platform is reached by a surf move (a surf exit, no arc), and the flights off the
edge land back on it: before #26 no arc was traced for them and they added no eye.

Geometry (x east, y north, z up; interior bounds, 16u walls):
    S  chamber          x 0..1600   y 0..512    z 0..448   info_player_start (128,256,1)
       ramp (wedge)     x 512..768  rises from z 0 to z 320 eastward
       platform         x 768..1600 top z 320 (the ramp's top edge meets it)
       ceiling opening  x 784..1600 y 128..384 into the well
    W  well             x 784..1600 y 128..384  z 464..1024, open at the bottom only
    T  slot             x 1176..1240, z 568..600 (32u: no crouched player fits), through the well's
                        north wall, a 112u tube and R's south wall: 144u long
    R  room             x 1080..1336 y 528..784 z 568..700, floor level with the slot's bottom
    T2, R2 (control)    the same slot through the well's SOUTH wall at z 760..792 (128u long), into
                        room R2 x 1080..1336 y -256..0 z 760..900
    hint                over the opening at z 464: splits the leaf over the platform from the well,
                        as a mapper would (without it the node eyes' cluster runs up the well and R
                        is only tier B before #26)

Why R is seen only in flight: every node eye is at most 320 + 85.33 = 405.3, under the ceiling
(448). A sight line through the slot rises or falls at most 32/144 = 0.22; one from under the
ceiling must climb from z 464 (the opening, at y 128..384) to the slot's mouth (y 384, z >= 568),
at least 104 in 256, 0.41. R's floor (z 568) faces up, so it needs an eye above 568 anyway. A
flight straight east off the edge at full speed peaks at feet z 595 (crouch eye 623) right in
front of the slot (x 1176..1240): from y 144..256 its apex eyes see R's north wall and floor.
Why R2 is not: its slot starts at 760; a flight eye is at most 595 + 64 = 659 (standing), and
from there its mouth needs a slope of at least (760 - 659) / 240 = 0.42 > 32/128.
"""
import vmfgen

NAME = "f20_surf_flight_eyes"
DESCRIPTION = ("A room seen only from the apex of a surf flight off a ramp's top edge onto the platform it "
               "meets (a surf move, no arc), through a slot no node eye can see through; a control room "
               "behind a slot above every flight.")

RAMP_X0, RAMP_X1, RAMP_TOP = 512.0, 768.0, 320.0
CEIL = 448.0
WELL_X0, WELL_X1, WELL_Y0, WELL_Y1, WELL_TOP = 784.0, 1600.0, 128.0, 384.0, 1024.0
SLOT_X0, SLOT_X1 = 1176.0, 1240.0
SLOT_Z0, SLOT_Z1 = 568.0, 600.0
SLOT2_Z0, SLOT2_Z1 = 760.0, 792.0
HINT, SKIP = "TOOLS/TOOLSHINT", "TOOLS/TOOLSSKIP"


def build(m: vmfgen.Map) -> None:
    nz = (RAMP_X1 - RAMP_X0) / ((RAMP_X1 - RAMP_X0) ** 2 + RAMP_TOP ** 2) ** 0.5
    assert 0.1 <= nz < 0.7, nz
    sin = (1 - nz * nz) ** 0.5
    vmax = vmfgen.MAX_AIR_SPEED * vmfgen.REACH_MARGIN
    surf_apex = (vmax * sin) ** 2 / (2 * vmfgen.SV_GRAVITY)
    apex_dx = vmax * nz * vmax * sin / vmfgen.SV_GRAVITY            # horizontal run to the apex
    eye_top = RAMP_TOP + surf_apex + vmfgen.EYE_DUCK                  # highest crouched flight eye
    eye_top_stand = RAMP_TOP + surf_apex + vmfgen.EYE_STAND           # and standing
    node_eye_top = RAMP_TOP + vmfgen.MAX_EYE_ABOVE_FLOOR
    slot_len = 16 + 112 + 16
    slope = (SLOT_Z1 - SLOT_Z0) / slot_len
    slope2 = (SLOT2_Z1 - SLOT2_Z0) / (16 + 96 + 16)
    assert node_eye_top + 30 < CEIL                                    # no node eye reaches the well
    assert (SLOT_Z0 - (CEIL + 16)) / (WELL_Y1 - WELL_Y0) > slope + 0.1  # no line from under the ceiling
    assert SLOT_Z1 - SLOT_Z0 < 36                                       # no crouched player fits the slot
    assert SLOT_X0 < RAMP_X1 + apex_dx < SLOT_X1, RAMP_X1 + apex_dx     # the apex is in front of the slot
    assert SLOT_Z0 < eye_top < SLOT_Z1 + 40, eye_top                    # and level with it
    assert (SLOT2_Z0 - eye_top_stand) / (WELL_Y1 - WELL_Y0 - 16) > slope2 + 0.1  # R2 out of every flight's sight

    m.room((0, 0, 0), (1600, 512, CEIL), openings=[("ceiling", (WELL_X0, WELL_X1, WELL_Y0, WELL_Y1))])
    m.entity("info_player_start", origin=(128, 256, 1), angles="0 0 0")
    m.wedge((RAMP_X0, 0, 0), (RAMP_X1, 512, RAMP_TOP), high="east")
    m.box((RAMP_X1, 0, 0), (1600, 512, RAMP_TOP))
    # a hint over the opening: without it the leaf over the platform runs up the well, so the
    # node eyes' cluster would see R (tier B, not A, before the flight eyes)
    hint = m.box((WELL_X0, WELL_Y0, CEIL), (WELL_X1, WELL_Y1, CEIL + 16), material=SKIP,
                 face_materials={"top": HINT})
    m.verify_skip_solid(hint)
    m.room((WELL_X0, WELL_Y0, CEIL + 16), (WELL_X1, WELL_Y1, WELL_TOP), floor=False,
           openings=[("north", (SLOT_X0, SLOT_X1, SLOT_Z0, SLOT_Z1)),
                     ("south", (SLOT_X0, SLOT_X1, SLOT2_Z0, SLOT2_Z1))])
    m.room((SLOT_X0, WELL_Y1 + 16, SLOT_Z0), (SLOT_X1, 512, SLOT_Z1), omit=("south", "north"))
    room_r = m.room((1080, 528, SLOT_Z0), (1336, 784, 700), openings=[("south", (SLOT_X0, SLOT_X1, SLOT_Z0, SLOT_Z1))])
    m.room((SLOT_X0, 16, SLOT2_Z0), (SLOT_X1, WELL_Y0 - 16, SLOT2_Z1), omit=("south", "north"))
    room_r2 = m.room((1080, -256, SLOT2_Z0), (1336, 0, 900),
                     openings=[("north", (SLOT_X0, SLOT_X1, SLOT2_Z0, SLOT2_Z1))])

    m.expect_visible(room_r.inner("north"), "R's back wall: seen through the slot from a surf flight's apex only")
    m.expect_visible(room_r.inner("floor"), "R's floor (z 568, facing up): seen only by an eye above it, in flight")
    m.expect_hidden(room_r2.all_inner(), "R2 behind a slot above every surf flight (eyes at most 659)")
    m.expect_reach((1200, 64, RAMP_TOP), "platform: a surf exit off the ramp's top edge")
    m.expect_reach((256, 256, 0), "chamber floor at the spawn")
    m.expect_no_reach(room_r.floor_center(0), "R floor: behind a 32u slot no player fits")

    # verify_expectations.py (#19)
    m.verify_floor(0, 0, RAMP_X0, 512, 0, "chamber floor west of the ramp")
    m.verify_floor(RAMP_X1, 0, 1600, 512, RAMP_TOP, "platform (eyes up to 405, under the ceiling)")
    # flights straight east off the edge at y 144..256, any speed up to vmax: below the full-speed
    # envelope (feet 578 at x 1100, 595 at the apex, 582 at x 1300) every point is on one of them
    m.verify_eye_box((1100, WELL_Y0 + 16, 540), (1300, 256, 604),
                     "apex region of the flights straight east off the ramp's top edge")
    m.verify_eye_box((RAMP_X0, 0, vmfgen.EYE_DUCK), (1600, 512, eye_top_stand + 2),
                     "surfing the ramp and every flight off it, crouched or standing", role="hidden")

    m.note(f"Ramp n.z {nz:.3f}; surf apex {surf_apex:.0f}u at {apex_dx:.0f}u past the edge; flight eye top "
           f"{eye_top:.0f} crouched, {eye_top_stand:.0f} standing; node eye top {node_eye_top:.0f}; slot slope "
           f"{slope:.3f}, R2's {slope2:.3f}.")
