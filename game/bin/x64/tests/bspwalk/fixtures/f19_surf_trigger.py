"""
f19 -- a trigger_teleport that only a surfer flying off a ramp's top edge passes through (#26).

f18's ramp again: a steep face (n.z 0.625) is a slide, a standing player slides down it and cannot
jump off it, and a player carrying speed surfs up it and off its top edge (vmax = 640 * 1.15 =
736 u/s; the vertical part 0.78 vmax gives a flight apex 275u above the launch point).

Geometry (x east, y north, z up; interior bounds, 16u walls):
    S  surf chamber     x 0..1600  y 0..512    z 0..1024   info_player_start (128,256,1)
       ramp (wedge)     x 512..768 rises from z 0 to z 320 eastward
       trigger T        x 784..880 y 128..384 z 496..640 over the ramp's top edge, target f19_dest
       trigger U        x 784..880 y 128..384 z 800..896, target f19_never (the control)
    R  room             x 0..512   y 704..1216 z 0..192, sealed, info_teleport_destination f19_dest
    Q  room             x 640..1152 y 704..1216 z 0..192, sealed, info_teleport_destination f19_never

Why T is reached only by surfing: in feet space (the trigger grown by the standing hull, 72u tall)
T starts at z 424, and a node fires a trigger by touching it when its top is within a duck-jump
(57u) of that, z 367. The ramp's highest point is z 320 and the chamber floor z 0, so no node
touches it, no jump reaches it (the floor's jumps peak at 57), and a walk-off only goes down. A
surf flight from the top edge peaks at 320 + 275 = 595, through T.
Why U is not: in feet space it starts at z 728, above every surf flight (595), so Q stays sealed.
"""
import vmfgen

# Surfing is an exploit (#87): the main run passes -exploits (m.walker_args); the harness's second
# run without it must leave room R unreached.

NAME = "f19_surf_trigger"
DESCRIPTION = ("trigger_teleport above a surf ramp's top edge, reached only by a surf flight through it; a "
               "control teleport above every surf flight.")

TRIGGER = "TOOLS/TOOLSTRIGGER"
RAMP_X0, RAMP_X1, RAMP_TOP = 512.0, 768.0, 320.0
T_Z0, T_Z1 = 496.0, 640.0
U_Z0, U_Z1 = 800.0, 896.0


def build(m: vmfgen.Map) -> None:
    nz = (RAMP_X1 - RAMP_X0) / ((RAMP_X1 - RAMP_X0) ** 2 + RAMP_TOP ** 2) ** 0.5
    assert 0.1 <= nz < 0.7, nz
    vmax = vmfgen.MAX_AIR_SPEED * vmfgen.REACH_MARGIN
    surf_apex = (vmax * (1 - nz * nz) ** 0.5) ** 2 / (2 * vmfgen.SV_GRAVITY)
    stand = 72.0
    jump_rise = vmfgen.CROUCH_JUMP_FEET
    assert T_Z0 - stand - jump_rise > RAMP_TOP + 8         # no node touches T, even raised by a jump
    assert T_Z0 - stand < RAMP_TOP + surf_apex - 100       # well inside a surf flight from the top edge
    assert U_Z0 - stand > RAMP_TOP + surf_apex + 100       # U above every surf flight

    m.walker_args("-exploits")                             # surfing is an exploit (#87)
    m.room((0, 0, 0), (1600, 512, 1024))
    m.entity("info_player_start", origin=(128, 256, 1), angles="0 0 0")
    m.wedge((RAMP_X0, 0, 0), (RAMP_X1, 512, RAMP_TOP), high="east")
    t = m.box((784, 128, T_Z0), (880, 384, T_Z1), material=TRIGGER)
    m.brush_entity("trigger_teleport", [t], target="f19_dest", spawnflags="1")
    u = m.box((784, 128, U_Z0), (880, 384, U_Z1), material=TRIGGER)
    m.brush_entity("trigger_teleport", [u], target="f19_never", spawnflags="1")

    room_r = m.room((0, 704, 0), (512, 1216, 192))
    m.entity("info_teleport_destination", origin=(256, 960, 1), angles="0 0 0", targetname="f19_dest")
    room_q = m.room((640, 704, 0), (1152, 1216, 192))
    m.entity("info_teleport_destination", origin=(896, 960, 1), angles="0 0 0", targetname="f19_never")

    m.expect_visible(room_r.all_inner(), "inner side of room R: teleport T is crossed by a surf flight off the ramp")
    m.expect_hidden(room_q.all_inner(), "inner side of room Q: teleport U is above every surf flight, Q is sealed")
    m.expect_reach((256, 256, 0), "surf chamber floor at the spawn")
    m.expect_reach(room_r.floor_center(0), "room R floor: teleport T, crossed only by surfing off the ramp")
    m.expect_no_reach(room_q.floor_center(0), "room Q floor: teleport U is out of every flight's reach")
    m.expect_no_reach_default(room_r.floor_center(0), "room R floor: teleport T needs a surf flight, off without -exploits")

    # verify_expectations.py (#19)
    m.verify_floor(0, 0, RAMP_X0, 512, 0, "surf chamber floor west of the ramp")
    m.verify_floor(RAMP_X1, 0, 1600, 512, 0, "surf chamber floor east of the ramp")
    m.verify_floor(0, 704, 512, 1216, 0, "room R floor (teleport T)")
    m.verify_eye_box((RAMP_X0, 0, vmfgen.EYE_DUCK), (1600, 512, RAMP_TOP + surf_apex + 86),
                     "surfing the ramp and flying off it", role="hidden")

    m.note(f"Ramp n.z {nz:.3f}; surf apex from the top edge {surf_apex:.0f}u. T feet z from {T_Z0 - stand:g}, "
           f"U feet z from {U_Z0 - stand:g}.")
