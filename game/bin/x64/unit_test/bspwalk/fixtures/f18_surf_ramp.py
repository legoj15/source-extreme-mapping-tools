"""
f18 -- a platform reachable only by surfing up a steep ramp, and one too high to surf to (#24).

A steep face (0.1 <= n.z < 0.7) is a slide: a standing player slides down it and cannot jump
off it.  A player who reaches it carrying speed rides it along its plane instead (surfing): up
the ramp, and off its top edge into the air.  The speed comes from bunny hopping or a fall; the
walker bounds it by max_air_speed * reach_margin (hl2: 640 * 1.15 = 736 u/s).

Geometry (x east, y north, z up; interior bounds, 16u walls), two chambers:
    S  surf chamber     x 0..1600  y 0..512     z 0..1024   info_player_start (128,256,1)
       ramp (wedge)     x 512..768 rises from z 0 to z 320 eastward: slope 320/256, n.z 0.625
       gap              x 768..896, floor z 0 (reached only by going over the ramp)
       platform P       x 896..1280, top z 360; hut on it, doorway in its EAST wall
    C  control chamber  y 704..1216, the same ramp and gap, platform Q top z 800; hut, doorway east

Why P is reachable only by surfing: the ramp spans the chamber's whole width, so everything east
of it is reached over its top (z 320).  P's top is 360: 40u above the top edge and 128u across,
with nothing to stand on in between.  A player on a slide cannot jump; walking off the top edge
falls into the gap (z 0), from which P is 360u up.  Surfing: leaving the top edge along the
ramp's plane at v gives vz = v sin(51.3 deg) = 0.78 v; reaching z 361 at 128u+ needs
v >= about 200 u/s, well inside the bunny-hop ceiling (Source bunny hopping exceeds 400 u/s).
Why Q is not: from the top edge (z 320) the highest a surf flight climbs is
(736 * 0.78)^2 / (2 * 600) = 275u, to z 595 < 800; from anywhere lower on the ramp less.
Hut interiors: each doorway faces east, away from the ramp; P's is seen only from P.  Q's hut
sits on a platform above every eye (max 595 + 85 = 680 < 800), so its interior and roof are
hidden.
"""
import vmfgen

NAME = "f18_surf_ramp"
DESCRIPTION = ("Platform reachable only by surfing up a steep ramp and off its top edge; control platform too "
               "high for any surf flight. Each carries a hut whose interior faces away from the ramp.")

RAMP_X0, RAMP_X1, RAMP_TOP = 512.0, 768.0, 320.0
GAP_X1 = 896.0
PLAT_X1 = 1280.0
P_TOP, Q_TOP = 360.0, 800.0


def build(m: vmfgen.Map) -> None:
    nz = (RAMP_X1 - RAMP_X0) / ((RAMP_X1 - RAMP_X0) ** 2 + RAMP_TOP ** 2) ** 0.5
    assert 0.1 <= nz < 0.7, nz                                         # a slide, not a floor
    vmax = vmfgen.MAX_AIR_SPEED * vmfgen.REACH_MARGIN
    surf_apex = (vmax * (1 - nz * nz) ** 0.5) ** 2 / (2 * vmfgen.SV_GRAVITY)
    assert P_TOP - RAMP_TOP > vmfgen.CROUCH_JUMP_FEET * 0 + 20          # above the top edge: no walk-off lands on it
    assert P_TOP - 0 > vmfgen.CROUCH_JUMP_FEET * 1.15 + 40              # not from the gap floor
    assert Q_TOP > RAMP_TOP + surf_apex + 150                           # beyond every surf flight
    assert Q_TOP > RAMP_TOP + surf_apex + vmfgen.MAX_EYE_ABOVE_FLOOR + 100   # and above every eye

    for y0, top, spawn_cls in ((0.0, P_TOP, "info_player_start"), (704.0, Q_TOP, "info_player_deathmatch")):
        m.room((0, y0, 0), (1600, y0 + 512, 1024))
        m.entity(spawn_cls, origin=(128, y0 + 256, 1), angles="0 0 0")
        m.wedge((RAMP_X0, y0, 0), (RAMP_X1, y0 + 512, RAMP_TOP), high="east")
        plat = m.box((GAP_X1, y0, 0), (PLAT_X1, y0 + 512, top))
        hut = m.room((1008, y0 + 176, top), (1104, y0 + 336, top + 96), floor=False,
                     openings=[("east", (y0 + 224, y0 + 288, top, top + 80))])
        if top == P_TOP:
            m.expect_visible(hut.all_inner(), "hut P inner side: P is reached only by surfing off the ramp's top edge; "
                                              "the doorway faces away from the ramp")
            m.expect_visible(plat.side("west"), "control: platform P's face toward the ramp")
            # (no claim on hut P's roof: a surf flight peaks at z 595 near the gap and can look down on it)
            m.expect_reach((1200, y0 + 64, top), "platform P top: surf up the ramp and off its top edge")
            m.expect_reach((1056, y0 + 256, top), "hut P floor: walk round to its east doorway")
            m.expect_reach((832, y0 + 256, 0), "gap floor: walk off the ramp's top edge")
            # verify_expectations.py (#19)
            m.verify_floor(0, y0, RAMP_X0, y0 + 512, 0, "floor west of the ramp")
            m.verify_floor(RAMP_X1, y0, GAP_X1, y0 + 512, 0, "gap floor")
            m.verify_floor(GAP_X1, y0, PLAT_X1, y0 + 512, top, "platform P, hut P's inside included")
            m.verify_floor(PLAT_X1, y0, 1600, y0 + 512, 0, "floor east of P (drop off P)")
            m.verify_eye_box((RAMP_X0, y0, vmfgen.EYE_DUCK), (GAP_X1, y0 + 512, RAMP_TOP + surf_apex + 86),
                             "surfing the ramp and flying off it", role="hidden")
        else:
            m.expect_hidden(hut.all_inner(), "hut Q inner side: Q (z 800) is above every surf flight (max 595)")
            m.expect_hidden(hut.outer("ceiling"), "hut Q roof top z=912: above every eye")
            m.expect_visible(plat.side("west"), "control: platform Q's face toward the ramp")
            m.expect_reach((256, y0 + 256, 0), "control chamber floor at the spawn")
            m.expect_no_reach((1200, y0 + 64, top), "platform Q top: 480u above the ramp's top edge, beyond any surf")
            m.expect_no_reach((1056, y0 + 256, top), "hut Q floor on the unreachable platform Q")
            m.verify_floor(0, y0, RAMP_X0, y0 + 512, 0, "control chamber floor west of the ramp")
            m.verify_floor(RAMP_X1, y0, GAP_X1, y0 + 512, 0, "control gap floor")
            m.verify_eye_box((RAMP_X0, y0, vmfgen.EYE_DUCK), (GAP_X1, y0 + 512, RAMP_TOP + surf_apex + 86),
                             "surfing the control ramp and flying off it", role="hidden")

    m.note(f"Ramp n.z {nz:.3f}; surf apex from the top edge {surf_apex:.0f}u (vmax {vmax:.0f} u/s). "
           f"P top {P_TOP:g}, Q top {Q_TOP:g}.")
