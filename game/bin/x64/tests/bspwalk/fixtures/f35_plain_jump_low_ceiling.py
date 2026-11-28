"""
f35 -- an east platform reached only by a plain (unlifted) jump, under a low ceiling (#115).

A pair's arcs are generated duck-jump first ("lifted": the launch raised by crouch_lift,
ARC_EYE_LIFTED) and plain second (ARC_EYE_CROUCHED, launched at the feet point), both traced
with the crouched hull, and the walker keeps the FIRST free one, so a plain arc survives only
where its lifted twin is blocked.

Jump speed is fixed (160 u/s, gravity 600: apex 21.33 over the launch), so under a ceiling 90u
over the floor every lifted arc's crouched hull rises to launch + 36 + 36 + 21.33 = 93.33 at
its apex (blocked) and every plain arc's to launch + 36 + 21.33 = 57.33 (free): across the pit
the pair's kept arc is always the plain jump.  The gap is 96u, f04's near pit: a standing jump
cannot cross it (~30 u/s of air control), and by vmfgen's overhang model (the hull centre
travels the gap less 32u) a real running jump cut short by the ceiling still can (190 u/s over
the 0.406 s a jump lasts when the hull is stopped at 90 - 72 = 18u: 13u spare), so the east
platform is reached through a plain jump.  Measured on the walker: the east platform is reached as it is, and
with the plain arc removed from ComboArcs (bw_arcs.cpp: the second SolveArcToPoint, the one
that sets ARC_EYE_CROUCHED) it is not ("missed must_reach: (704, 32, 0)").

What this fixture does NOT show: the plain arc's standing eye curve.  Issue #115 item 8 found
that the air over a pit has fall-column eyes to the same height (a version of this map with a
face seen only from over the pit passed on a walker without the curve), so the plain arc is
pinned down here by reach alone.

The map (x east, y north, z up; interior bounds; the shell's walls are 16 thick):

    corridor   x 0..1120   y 0..64    z -256..90    floor=False
    west plat  x 0..512    top z=0                  spawn at (96, 32, 1)
    pit        x 512..608  floor top z=-256         96u wide, like f04's near gap
    east plat  x 608..1120 top z=0
"""
import vmfgen

NAME = "f35_plain_jump_low_ceiling"
DESCRIPTION = ("An east platform across a 96u pit under a 90u ceiling, reached only by a plain (unlifted) jump "
               "arc: every lifted duck-jump twin's crouched hull tops out at 93.33u and is ceiling-blocked, so the "
               "pair's kept arc is the plain one (#115).")

PIT = 96.0                 # the pit's width in x: a running jump crosses it, a standing one cannot (f04)
CEILING_Z = 90.0           # the corridor's ceiling: over every lifted arc's hull top, under every plain one
PIT_TOP = -256.0           # the pit's floor: a drop, not a climb (a duck-jump climbs 57u)
X_WEST_END = 512.0         # the west platform's east edge
X_EAST_START = X_WEST_END + PIT            # 608: the east platform's west edge
X_END = 1120.0             # the corridor's east end
X_PIT_C = (X_WEST_END + X_EAST_START) / 2.0      # 560: the pit's middle


def build(m: vmfgen.Map) -> None:
    vj, g, lift = vmfgen.JUMP_SPEED, vmfgen.SV_GRAVITY, vmfgen.CROUCH_LIFT
    hull_d, half = vmfgen.HULL_DUCK[2], vmfgen.HULL_STAND[0] / 2     # 36 tall, 16 from the centre
    apex = vj * vj / (2 * g)                    # 21.33: a jump's rise over its launch
    lifted_top = lift + apex + hull_d           # 93.33: a lifted arc's hull top over its launch
    plain_top = apex + hull_d                   # 57.33: a plain arc's
    assert lifted_top > CEILING_Z + 2, lifted_top      # every lifted arc is blocked by the ceiling
    assert plain_top < CEILING_Z - 2, plain_top        # every plain arc stays under it
    assert vmfgen.jump_gap_reach(vmfgen.AIR_CONTROL_SPEED) < PIT       # a standing jump cannot cross
    feet_cut = CEILING_Z - vmfgen.HULL_STAND[2]         # 18: how high a real jump still goes
    t_air = (vj - (vj * vj - 2 * g * feet_cut) ** 0.5) / g + (2 * feet_cut / g) ** 0.5    # 0.406 s
    assert vmfgen.RUN_SPEED * t_air + 2 * half > PIT + 4    # a running jump crosses, cut short

    corridor = m.room((0, 0, PIT_TOP), (X_END, 64, CEILING_Z), floor=False)
    m.entity("info_player_start", origin=(96, 32, 1), angles="0 0 0")
    bot = PIT_TOP - 16
    m.box((0, 0, bot), (X_WEST_END, 64, 0))
    pit = m.box((X_WEST_END, 0, bot), (X_EAST_START, 64, PIT_TOP))
    m.box((X_EAST_START, 0, bot), (X_END, 64, 0))

    m.expect_visible(corridor.inner("east"),
                     "control: the corridor's east end wall, seen along the corridor from the spawn")
    m.expect_visible(pit.side("top"), "control: the pit floor, seen from a platform edge and landed on")
    m.expect_visible(corridor.inner("ceiling"), "control: the corridor's ceiling over the spawn")

    m.expect_reach((96, 32, 0), "west platform at the spawn")
    m.expect_reach((704, 32, 0),
                   f"east platform: across the {PIT:g}u pit the pair's kept arc is the plain (unlifted) jump, "
                   f"every lifted duck-jump arc being blocked by the {CEILING_Z:g}u ceiling (hull top "
                   f"{lifted_top:.2f})")
    m.expect_reach((X_PIT_C, 32, PIT_TOP), "pit floor: a drop from either platform")

    m.note(f"Ceiling z={CEILING_Z:g}: a lifted arc's hull top {lifted_top:.2f} (blocked), a plain one's "
           f"{plain_top:.2f} (free), so the pit is crossed only by a plain arc.  A running jump cut by the "
           f"ceiling crosses the pit with {vmfgen.RUN_SPEED * t_air + 2 * half - PIT:.0f}u to spare by vmfgen's overhang model; a standing "
           f"jump reaches {vmfgen.jump_gap_reach(vmfgen.AIR_CONTROL_SPEED):.0f}u.")

    # verify_expectations.py (#19)
    m.verify_floor(0, 0, X_WEST_END, 64, 0, "west platform")
    m.verify_floor(X_EAST_START, 0, X_END, 64, 0, "east platform")
    m.verify_floor(X_WEST_END, 0, X_EAST_START, 64, PIT_TOP, "pit floor")
