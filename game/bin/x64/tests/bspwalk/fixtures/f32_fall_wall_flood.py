"""
f32 -- a fall column stops at a wall the flight cannot get over, and passes a slot in it (#38).

The fall columns (bw_eyes.cpp BuildFallColumns) raise the nodes a fall from a ledge can drift
over.  Before #38's flood they raised every such node in the ledge's air component (BSP leaves
joined), so a wall that is too tall to fly over did nothing as long as the air met somewhere
else: over the wall's top, or round its far end.  The flood (FallFloodOne) follows the air from
the ledge only as high and as far as the fall allows, so the wall stops it and a slot in the wall
lets it through.

Two sealed chambers, the same but for the slot, each with its own spawn on its ledge.
Layout per chamber (x east, y north, z up; interior bounds, 16u shell), y0 = 0 (A) or 2048 (B):
    room       x 0..1024   y y0..y0+1536   z 0..768   one air space
    L  ledge   x 0..256    y y0..y0+256    top z=256  the spawn; walk or jump off it
    W  wall    x 384..400  y y0..y0+1280   z 0..512   its north end leaves a gap y0+1280..y0+1536,
                                                     1000u from the ledge; air also meets over it
    T  block   x 832..896  y y0+64..y0+192  z 0..160  east of the wall; its TOP is the claim
    B only: a slot through W at y y0+288..y0+352, full height (z 0..512), 113u from the ledge.

Chamber A: why T's top is hidden (strict: the pre-#38 fall columns put eyes over the east floor at
z ~340, and every correct column must leave them out):
  * Real eyes over the east floor (x >= 400) are floor eyes, at most 85.3 above z=0: under the top
    (z=160), so in no position to see it.
  * A fall off L: the feet are never above 256 + 21.3 and the eye never above 256 + 21.3 + 64 =
    341.3 (a duck in the air lifts the feet 36 and keeps the eye), under W's top (z=512).  Over the
    east floor it would have to get round W's north end, 1000u from L, while a sprint jump off L
    is down on the floor (z=0) within 1.23 s, 1.23 x 460 = 566u even at the walker's arc speed.
    So every fall eye stays west of W (the verifier's "hidden" box), and a line from there to T's
    top crosses x 384..400 below z=512, at y 0..1280: inside W.
  * Arcs (bspwalk traces each straight in XY) from L to the east floor cross W.
  So the only eyes that see T's top are fall columns raised over the east floor through the air
  over W or round its end: the pre-#38 component test kept them (one air space), the flood drops
  them (the air over W starts at z=512, above any fall eye; the gap is out of reach).

Chamber B: why T's top is visible (a real flight through the slot):
  Stand at L's north-east corner (centre (256, y0+256), feet 256), sprint-jump north-east into the
  slot and east through it: (256, y0+256) -> (384, y0+320) -> the eye box x 480..520,
  y0+304..y0+336, a path of 239..280u flown at 234..294 u/s (the hull is 32 wide, the slot 64;
  the turn is 27 degrees).  The feet are 256 + 160 t - 300 t^2: 136 at t = 0.95 s and 106 at
  t = 1.02 s, before the landing at t = 1.23 s; so the standing eye takes every height from 170
  to 200 over the box.  From there T's top (z=160, 300u east) is in plain view: nothing stands
  between them.  The flood passes the slot, so the columns over the east floor are raised there.
"""
import vmfgen

NAME = "f32_fall_wall_flood"
DESCRIPTION = ("A block top seen only from fall columns over the far side of a wall too tall to fly over "
               "(must be hidden: the air meets only over the wall and round its far end), and its twin "
               "with a slot in the wall that a real fall passes (must be visible) (#38).")

LEDGE = 256.0
WALL_X0, WALL_X1, WALL_TOP, WALL_Y1 = 384.0, 400.0, 512.0, 1280.0
ROOM_X, ROOM_Y, ROOM_Z = 1024.0, 1536.0, 768.0
T_BOX = ((832.0, 64.0, 0.0), (896.0, 192.0, 160.0))
SLOT = (288.0, 352.0)
EYE_BOX = ((480.0, 304.0, 170.0), (520.0, 336.0, 200.0))
PATH = 280.0
CHAMBER_Y = (0.0, 2048.0)
FALL_Y1 = 900.0     # the verifier's box of fall eyes ends here (out of reach beyond)


def build(m: vmfgen.Map) -> None:
    g, vj = vmfgen.SV_GRAVITY, vmfgen.JUMP_SPEED
    half = vmfgen.HULL_STAND[0] / 2
    apex = vj * vj / (2 * g)
    # A: the highest fall eye is under the wall top, and the floor eyes are under T's top
    assert LEDGE + apex + vmfgen.EYE_STAND < WALL_TOP - 100
    assert vmfgen.MAX_EYE_ABOVE_FLOOR * vmfgen.REACH_MARGIN < T_BOX[1][2] - 50
    # A: the wall's north end is out of reach of any fall off the ledge, even at the arc speed
    t_land = (vj + (vj * vj + 2 * g * LEDGE) ** 0.5) / g
    reach = vmfgen.INTENDED_AIR_SPEED * vmfgen.REACH_MARGIN * t_land + 2 * half
    assert WALL_Y1 - (256.0 + half) > reach + 300, (WALL_Y1, reach)
    # B: the flight -- feet over the eye box at arrival, speeds at most a sprint, before landing
    def feet(t):
        return LEDGE + vj * t - 0.5 * g * t * t

    def t_at(z):
        return (vj + (vj * vj + 2 * g * (LEDGE - z)) ** 0.5) / g

    t_hi, t_lo = t_at(EYE_BOX[1][2] - vmfgen.EYE_STAND), t_at(EYE_BOX[0][2] - vmfgen.EYE_STAND)
    assert t_lo < t_land and PATH / t_hi <= vmfgen.SPRINT_SPEED, (t_hi, t_lo, PATH / t_hi)
    assert EYE_BOX[0][2] > T_BOX[1][2] + 5   # the eyes look down onto T's top

    for ci, y0 in enumerate(CHAMBER_Y):
        slot = ci == 1
        tag = "B" if slot else "A"
        room = m.room((0, y0, 0), (ROOM_X, y0 + ROOM_Y, ROOM_Z))
        ledge = m.box((0, y0, 0), (256, y0 + 256, LEDGE))
        m.entity("info_player_start" if not slot else "info_player_deathmatch",
                 origin=(128, y0 + 128, LEDGE + 1), angles="0 0 0")
        if slot:
            m.box((WALL_X0, y0, 0), (WALL_X1, y0 + SLOT[0], WALL_TOP))
            wall = m.box((WALL_X0, y0 + SLOT[1], 0), (WALL_X1, y0 + WALL_Y1, WALL_TOP))
        else:
            wall = m.box((WALL_X0, y0, 0), (WALL_X1, y0 + WALL_Y1, WALL_TOP))
        block = m.box((T_BOX[0][0], y0 + T_BOX[0][1], 0), (T_BOX[1][0], y0 + T_BOX[1][1], T_BOX[1][2]))

        if slot:
            m.expect_visible(block.side("top"),
                             f"chamber B: T's top z={T_BOX[1][2]:g}, seen from a fall off the ledge (z={LEDGE:g}) "
                             f"that flies east through the slot in the wall")
        else:
            m.expect_hidden(block.side("top"),
                            f"chamber A: T's top z={T_BOX[1][2]:g}: no fall off the ledge gets over the "
                            f"{WALL_TOP:g}u wall or round its far end, and the floor eyes are under it",
                            strict=True)
        m.expect_visible(block.side("west"), f"control {tag}: T's west face, seen from the east floor")
        m.expect_visible(wall.side("east"), f"control {tag}: the wall's east face, seen from the east floor")
        m.expect_visible(ledge.side("east"), f"control {tag}: the ledge's east face, seen from the floor below")
        m.expect_visible(room.inner("ceiling"), f"control {tag}: ceiling")

        m.expect_reach((128, y0 + 128, LEDGE), f"ledge top {tag}: the spawn")
        m.expect_reach((320, y0 + 600, 0), f"floor west of the wall {tag}: walk off the ledge")
        m.expect_reach((700, y0 + 600, 0), f"floor east of the wall {tag}: round its north end")

        # verify_expectations.py (#19)
        m.verify_floor(half, y0 + half, 256 - half, y0 + 256 - half, LEDGE, f"ledge top {tag}")
        m.verify_floor(half, y0 + 256 + half, WALL_X0 - half, y0 + ROOM_Y - half, 0, f"floor west {tag}")
        m.verify_floor(256 + half, y0 + half, WALL_X0 - half, y0 + 256 + half, 0, f"floor below the ledge {tag}")
        m.verify_floor(WALL_X0, y0 + WALL_Y1 + half, WALL_X1, y0 + ROOM_Y - half, 0, f"the gap {tag}")
        m.verify_floor(WALL_X1 + half, y0 + half, ROOM_X - half, y0 + ROOM_Y - half, 0, f"floor east {tag}")
        # every fall eye off the ledge west of the wall: within 566u of it, so y <= 256 + 16 + 566
        # (chamber B's also go through the slot: those are not needed, B has no must_hidden)
        assert FALL_Y1 > 256.0 + half + reach
        m.verify_eye_box((0, y0, 0), (WALL_X0, y0 + FALL_Y1, LEDGE + 100), f"falls off the ledge {tag}",
                         role="hidden")
        if slot:
            m.verify_eye_box((EYE_BOX[0][0], y0 + EYE_BOX[0][1], EYE_BOX[0][2]),
                             (EYE_BOX[1][0], y0 + EYE_BOX[1][1], EYE_BOX[1][2]),
                             "a fall off the ledge flown east through the slot")

    m.note(f"Chamber B flight: feet 136..106 at t {t_hi:.2f}..{t_lo:.2f} s over a {PATH:g}u path "
           f"(landing at {t_land:.2f} s).")
