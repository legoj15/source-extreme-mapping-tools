"""
f29 -- a fall column raised around a wall end (#38 guard).

The fall columns (bw_eyes.cpp BuildFallColumns) raise every node a fall from a ledge can drift
over, whatever is in between: air strafing curves a flight, so a straight path test would lose
real flights.  #38 keeps a launch off nodes whose air is not connected to its own at all (BSP
leaf components); this fixture is the curved case such a bound must keep: the only way into the
pocket's high air is a flight that turns round a wall end, and no straight line from the ledge
reaches the pocket.

Layout (x east, y north, z up; interior bounds, 16u walls), room x 0..768, y 0..768, z 0..512:
    L    ledge      x 208..768  y 0..304    top z=384   the spawn; its hull centres x >= 192, y <= 288
    W1   wall       x 192..768  y 304..320  z 0..512    full height; its west end leaves a gap x 0..192
    B    platform   x 224..768  y 320..768  top z=8     a step up from the floor: its own nodes
    F    fin        x 240..256  y 400..768  z 8..512    full height: the pocket's opening is x 240..256, y 320..400
    S    shelf      x 256..288  y 704..768  z 8..160    in the pocket's far corner; its TOP is the claim
floor z=0 elsewhere (south of W1 west of L, and north of W1 west of B).

Why only a curved flight sees S's top (z=160):
  * Base columns: every eye over a floor is at most 85.3 above it (8 + 85.3 over B), below 160.
  * L's eyes (y <= 288 + hull) and the south floor's: a line to S (y >= 704, x >= 256) passes
    the band y 304..320 at x >= 192 (W1) unless it passes the gap at x <= 192, and from there it
    crosses x = 240..256 at y >= 504, inside F.
  * The floor north of W1 west of B (hull centres x <= 208): a line to S must leave F's x range at
    y <= 400 to clear it, which needs a slope >= (704 - 400) / (288 - 256) = 9.5, so it starts at
    x >= 256 - 80 / 9.5 = 247.6: east of every such centre.
  * Arcs: bspwalk traces each arc straight in XY.  L's hull centres have x >= 192 and B's x >= 208,
    and the gap needs x <= 176 while y is in 288..336, so no straight path from L reaches B; the
    straight arcs that pass the gap end at x < 192, where the previous point applies.
  So S's top is seen only from B's high air, which only the fall columns put eyes in: a bound that
  tests straight paths from the launch drops them and FAILs this fixture; the #38 air components
  keep them (W1's gap joins the air).

The real flight (the exact eye box, x 280..320, y 344..376, eye z 200..290): run west along L
(feet z=384), jump at the west edge (centre x 192), air-strafe round W1's end (the centre x <= 176
while y is in 288..336) and east through the opening into the pocket.  Air strafing adds up to 30
u/s a tick across the velocity (AirAccelerate, wish speed cap 30) at 66.67 ticks/s, so at speed v
the turn radius is v^2 / 2000 (62u at 352): the turns fit in the gap region (x 0..176).  Every box
point has such a path of length 300 (a loop west of the gap lengthens the shortest, under 220,
continuously), flown at any speed in 190..352 u/s, arriving at t = 300 / v in 0.85..1.58 s.  The
jump's feet are 384 + 160 t - 300 t^2 until they land on B (z=8) at t = 1.417 s: from 302 down to
8, so the eye (feet + 28 crouched .. + 64 standing) takes every height from 36 to 366 there.
"""
import vmfgen

NAME = "f29_fall_around_wall_end"
DESCRIPTION = ("A shelf top seen only from the high air of a pocket that a fall off a ledge reaches only by "
               "curving round a wall end: the fall columns must raise the pocket (#38 guard).")

LEDGE = 384.0
PLAT = 8.0
SHELF_TOP = 160.0
WALL_END_X = 192.0
FIN_X0, FIN_X1, FIN_Y0 = 240.0, 256.0, 400.0
EYE_BOX = ((280.0, 344.0, 200.0), (320.0, 376.0, 290.0))
PATH = 300.0


def build(m: vmfgen.Map) -> None:
    g, vj = vmfgen.SV_GRAVITY, vmfgen.JUMP_SPEED
    half = vmfgen.HULL_STAND[0] / 2
    # base columns stay under the shelf top (with the walker's margin)
    assert PLAT + vmfgen.MAX_EYE_ABOVE_FLOOR * vmfgen.REACH_MARGIN < SHELF_TOP - 8
    # no straight path: L's centres x >= WALL_END_X, B's x >= 224 - 16, the gap's x <= WALL_END_X - 16
    assert 208.0 - half >= WALL_END_X and 224.0 - half > WALL_END_X - half
    # the floor west of B cannot see S past F: centres x <= 224 - 16; the line needs x >= 247.6
    assert 224.0 - half < FIN_X1 - (FIN_Y0 - 320.0) / ((704.0 - FIN_Y0) / (288.0 - FIN_X1))
    # the flight: arrival times for a 300u path at 190..352 u/s, feet from the jump, landing on B
    t_land = (vj + (vj * vj + 2 * g * (LEDGE - PLAT)) ** 0.5) / g
    t0, t1 = PATH / 352.0, min(PATH / 190.0, t_land)
    feet_hi = LEDGE + vj * t0 - 0.5 * g * t0 * t0
    feet_lo = LEDGE + vj * t1 - 0.5 * g * t1 * t1
    assert feet_lo + vmfgen.EYE_DUCK <= EYE_BOX[0][2] and feet_hi + vmfgen.EYE_STAND >= EYE_BOX[1][2], (feet_lo, feet_hi)
    assert (352.0 ** 2) / 2000.0 < WALL_END_X - half - 16   # the tightest turn fits west of the gap

    room = m.room((0, 0, 0), (768, 768, 512))
    ledge = m.box((208, 0, 0), (768, 304, LEDGE))
    m.entity("info_player_start", origin=(488, 152, LEDGE + 1), angles="0 0 0")
    wall = m.box((WALL_END_X, 304, 0), (768, 320, 512))
    plat = m.box((224, 320, 0), (768, 768, PLAT))
    m.box((FIN_X0, FIN_Y0, PLAT), (FIN_X1, 768, 512))
    shelf = m.box((256, 704, PLAT), (288, 768, SHELF_TOP))

    m.expect_visible(shelf.side("top"),
                     f"shelf top z={SHELF_TOP:g}: seen only from the pocket's air above it, which a fall off the "
                     f"ledge (z={LEDGE:g}) reaches only by curving round the wall end")
    m.expect_visible(plat.side("top"), "control: the platform, walked on")
    m.expect_visible(wall.side("north"), "control: the wall's north face, seen from the floor north of it")
    m.expect_visible(ledge.side("west"), "control: the ledge's west face, seen from the floor below it")
    m.expect_visible(room.inner("ceiling"), "control: ceiling")

    m.expect_reach((488, 152, LEDGE), "ledge top: the spawn")
    m.expect_reach((100, 150, 0), "floor west of the ledge: walk off it")
    m.expect_reach((100, 600, 0), "floor north of the wall: round its west end")
    m.expect_reach((600, 600, PLAT), "the pocket's platform: an 8u step up")

    m.note(f"Flight to the eye box: feet {feet_lo:.0f}..{feet_hi:.0f} at t {t0:.2f}..{t1:.2f} s (path {PATH:g}u).")

    # verify_expectations.py (#19)
    m.verify_floor(208 - half, half, 768 - half, 304 - half, LEDGE, "ledge top")
    m.verify_floor(half, half, 208 - half, 304 - half, 0, "floor west of the ledge")
    m.verify_floor(half, 320 + half, 224 - half, 768 - half, 0, "floor north of the wall, west of the platform")
    m.verify_floor(FIN_X1 + half + 32, 320 + half, 768 - half, 768 - half, PLAT, "the pocket's platform")
    m.verify_eye_box(EYE_BOX[0], EYE_BOX[1], "a fall off the ledge curving round the wall end into the pocket")
