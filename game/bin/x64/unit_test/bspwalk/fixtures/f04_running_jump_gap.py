"""
f04 -- a platform across a gap a running jump clears, and one across a gap
nothing clears.

Physics used (HL2, Source SDK 2013 gamemovement): sv_gravity 600, jump speed
sqrt(2*600*21) ~ 158.7 u/s, flat airtime 2*158.7/600 = 0.53 s (0.46 s at the
engine-default gravity 800; the fixture holds under both).  Ground speed 190
(hl2_normspeed) or 320 (sprint).  The hull may overhang an edge by just under
16u on take-off and on landing, so the widest gap crossed at speed v is about
v * airtime + 32.  A standing jump gains at most ~30 u/s of air control.

    gap     96u : standing jump  ~ 30*0.53 + 32 =  48u  -> cannot cross
                  walk speed 190 ~ 190*0.53 + 32 = 132u -> crosses (36u spare)
                  (at gravity 800: 190*0.46 + 32 = 119u, 23u spare)
                  The spawn platform gives a 500u run-up.
    gap   1280u : beyond every arc bspwalk tries.  Its profiles bound the air
                  speed at max_air_speed * reach_margin (hl2: 640 * 1.15 = 736
                  u/s) over a duck-jump's airtime (0.70 s), plus the hull width:
                  R = 550u (vmfgen.arc_reach_bound).  tf, with its larger hull,
                  800 u/s air speed and a double jump, is about 1148u.  (The
                  gap was 400u until #13: a 736 u/s duck-jump crosses that, so
                  the walker reached the west island.)

Both gaps are pits, 256u deep: falling in is a dead end (a duck-jump climbs 57u)
but the pit floor is reachable, so it is a visible control.  The 256u fall costs
a little health in HL2 and is not lethal.

The hidden/visible targets are the same closed "hut" trick as f03: a box on each
far platform with a single doorway in the wall that faces AWAY from the gap.
Every eye on the near side of the gap lies west (or east, for the west hut) of
the hut's far wall plane, so every straight segment into the hut crosses the
hut shell.  The interior is visible only to eyes on the far platform that have
walked round to the doorway, which is what crossing the gap buys.

Layout (plan view, x east; room interior 2992 x 512, z from -256 to 320):

    x: -880 .. -432 | 1280u pit | 848 ....... 1360 | 96u pit | 1456 ....... 2112
       [WEST island]             [SPAWN platform]             [EAST platform]
       top z=0                  top z=0                       top z=0
       hut, door faces WEST     info_player_start             hut, door faces EAST

    east hut:  interior x 1648..1744 y 176..336 z 0..96, door in its EAST wall
               (y 224..288, z 0..80); roof top z=112.
    west hut:  interior x -736..-640 y 176..336 z 0..96, door in its WEST wall
               (y 224..288, z 0..80); roof top z=112.
    Pit floors: top at z=-256, x -432..848 and x 1360..1456.

Expectations:
  visible  east-hut inner sides: running jump crosses the 96u gap; walk round
           the hut to the east doorway and step inside.
  visible  both pit floors (controls): a player standing at the spawn
           platform's edge sees them, and falls onto them.
  hidden   west-hut inner sides: 1280u is uncrossable, and every reachable eye
           (spawn platform, east platform, either pit) has x >= -416 while the
           hut's only aperture is in its west wall at x -752..-736.
  hidden   both huts' roof tops (z=112): the highest reachable eye is 85u over
           a z=0 floor.
"""
import vmfgen

NAME = "f04_running_jump_gap"
DESCRIPTION = ("Platform across a 96u gap (running jump) and across a 1280u gap (impossible); "
               "each carries a hut whose interior is visible only from the platform.")

GAP_NEAR = 96.0
GAP_FAR = 1280.0
PIT_TOP = -256.0


def build(m: vmfgen.Map) -> None:
    run = vmfgen.RUN_SPEED
    assert vmfgen.jump_gap_reach(0 + vmfgen.AIR_CONTROL_SPEED) < GAP_NEAR      # standing jump cannot
    assert vmfgen.jump_gap_reach(run) > GAP_NEAR + 20                           # running jump can, with spare
    assert vmfgen.jump_gap_reach(run, gravity=800.0) > GAP_NEAR + 10            # ... at gravity 800 too
    assert vmfgen.arc_reach_bound() < GAP_FAR - 600                              # bspwalk hl2 arcs (R 550) cannot
    assert vmfgen.arc_reach_bound(max_air=800, gravity=800, jump_speed=268.3281573, lift=20, half_width=24,
                                  air_jumps=1) < GAP_FAR - 100                   # nor tf's (R ~1148)
    assert -PIT_TOP > vmfgen.CROUCH_JUMP_FEET * 3                                # no climbing out

    x_pit_b_end = 848.0
    x_west_end = x_pit_b_end - GAP_FAR
    x_west = x_west_end - 448.0                     # the west island is 448u wide
    x_spawn_end = 1360.0
    x_east_start, x_end = x_spawn_end + GAP_NEAR, 2112.0
    assert x_pit_b_end - x_west_end == GAP_FAR and x_east_start - x_spawn_end == GAP_NEAR

    # Outer shell without a floor: the platforms and pit floors below are the floor.
    m.room((x_west, 0, PIT_TOP), (x_end, 512, 320), floor=False)
    bot = PIT_TOP - 16
    plat_west = m.box((x_west, 0, bot), (x_west_end, 512, 0))
    pit_b = m.box((x_west_end, 0, bot), (x_pit_b_end, 512, PIT_TOP))
    plat_spawn = m.box((x_pit_b_end, 0, bot), (x_spawn_end, 512, 0))
    pit_a = m.box((x_spawn_end, 0, bot), (x_east_start, 512, PIT_TOP))
    plat_east = m.box((x_east_start, 0, bot), (x_end, 512, 0))
    m.entity("info_player_start", origin=(880, 256, 1), angles="0 0 0")

    hut_east = m.room((1648, 176, 0), (1744, 336, 96), floor=False,
                      openings=[("east", (224, 288, 0, 80))])
    hut_west = m.room((x_west + 144, 176, 0), (x_west + 240, 336, 96), floor=False,
                      openings=[("west", (224, 288, 0, 80))])

    m.expect_visible(hut_east.all_inner(),
                     "east-hut inner side: platform is a 96u running jump away, doorway faces away from the gap")
    m.expect_visible(pit_a.side("top"), "control: floor of the 96u pit, seen from the spawn platform edge")
    m.expect_visible(pit_b.side("top"), "control: floor of the 400u pit, seen from the spawn platform edge")

    m.expect_hidden(hut_west.all_inner(),
                    "west-hut inner side: platform is a 400u gap away, doorway faces away from the gap")
    m.expect_hidden(hut_east.outer("ceiling"), "east-hut roof top at z=112: above every reachable eye (max 85)")
    m.expect_hidden(hut_west.outer("ceiling"), "west-hut roof top at z=112: above every reachable eye")

    # Reach (feet points on the platform tops, away from the huts, and on the hut floors).
    m.expect_reach((1100, 100, 0), "spawn platform top")
    m.expect_reach((1550, 100, 0), "east platform top: a running jump across the 96u gap")
    m.expect_reach((1696, 256, 0), "east-hut floor: walk round to its east doorway")
    m.expect_reach((x_spawn_end + GAP_NEAR / 2, 256, PIT_TOP), "floor of the 96u pit: a drop from either platform")
    m.expect_reach(((x_west_end + x_pit_b_end) / 2, 256, PIT_TOP), "floor of the wide pit: a drop from the spawn platform")
    m.expect_no_reach((x_west + 300, 100, 0), "west platform top: beyond the wide gap")
    m.expect_no_reach((x_west + 192, 256, 0), "west-hut floor on the unreachable west platform")

    m.note(f"Gaps {GAP_NEAR}u and {GAP_FAR}u, pits {-PIT_TOP}u deep. Run reach {vmfgen.jump_gap_reach(run):.0f}u at "
           f"{run:.0f} u/s (gravity 600), standing reach {vmfgen.jump_gap_reach(vmfgen.AIR_CONTROL_SPEED):.0f}u, "
           f"bspwalk arc bound {vmfgen.arc_reach_bound():.0f}u (hl2).")

    # verify_expectations.py (#19)
    m.verify_floor(x_pit_b_end, 0, x_spawn_end, 512, 0, "spawn platform")
    m.verify_floor(x_east_start, 0, x_end, 512, 0, "east platform, the east hut's inside included")
    m.verify_floor(x_spawn_end, 0, x_east_start, 512, PIT_TOP, "floor of the 96u pit")
    m.verify_floor(x_west_end, 0, x_pit_b_end, 512, PIT_TOP, "floor of the wide pit")
    m.verify_eye_box((x_west_end, 0, PIT_TOP + vmfgen.EYE_DUCK), (x_pit_b_end, 512, vmfgen.EYE_STAND + 22),
                     "falling into the wide pit", role="hidden")
    m.verify_eye_box((x_spawn_end, 0, PIT_TOP + vmfgen.EYE_DUCK), (x_east_start, 512, vmfgen.EYE_STAND + 22),
                     "falling into or jumping over the 96u pit", role="hidden")
