"""
f06 -- a displacement hill with a hut on its summit whose doorway faces away
from the whole valley.

The only way up is a displaced slope.  Displacement terrain is walkable (Source:
a surface is stood on when its normal has z >= 0.7, i.e. slopes up to 45.57
degrees).  The slope is one power-4 displacement (17 x 17 vertices, 16 x 16 cells
of 32 x 32u) that rises 128u over 352u with a smoothstep:

    height(x, y) = 128 * smoothstep((x - xs(y)) / 352),  xs(y) = 224 + 64 * sin(pi * y / 512)

so the foot of the hill is a wavy line (x 224..288), not a straight one, and the
surface has a gradient in both x and y.  The analytic gradient is at most
128 * 1.5 / 352 * sqrt(1 + (64 * pi / 512)^2) = 0.59, i.e. surface normal z >=
0.86.  build() checks the actual cell mesh numerically: all 1024 candidate
triangles (the 512 of each of the two possible diagonal splits of every cell; the
engine alternates them) have normal z >= 0.80, so the slope is walkable
everywhere whatever triangulation the compiler picks.  It also asserts that the
displaced surface meets the summit brush's top edge exactly (every vertex of the
x=704 border column is 128.0).

Layout (plan view, x east, y north; room interior 1024 x 512 x 384; the floor of
the whole shell is at z=-16):

    x: 0 .... 192 | 192 .......... 704 | 704 ................... 1024
       valley       displaced slope      summit pad (solid brush)
       top z=0      0 -> 128             top z=128
       spawn (96,256)                    hut interior x 768..864  y 176..336
                                         door in the hut's EAST wall
                                         (y 224..288, z 128..208); roof top z=240
    The hill spans the whole room depth and ends at the east wall, so the valley
    (z=0, x < 224..288) lies entirely WEST of the hut and there is no low ground
    anywhere east, north or south of it.  The summit east of the hut is 144u wide
    (x 880..1024) and 160u run either side of the hut, so the doorway can be
    walked round to.  The pad is 128u above the valley, far beyond any jump.

Two facts about how VBSP treats the displacement (both measured on this
fixture's BSP, not assumed):
  * A displacement brush is NOT solid.  The leaf inside the 16u slab under the
    slope is empty (contents 0), the valley slab beside it is solid (1), and a
    first version with the slab floating on nothing leaked through it.  The
    shell's floor slab is therefore at z=-16 and the displacement slab sits on
    it; the 16u pocket under the slope is sealed by the valley slab, the summit
    pad, the shell walls and the shell floor.  The BSP floor under the slope is
    z=-16, not z=0 -- only the displaced surface itself is the walking surface.
  * Because the pad is solid brush up to z=128 and the pocket is walled off by
    it, a walker that IGNORES displacement collision cannot get up (the pad's
    west face is 144u high), while one that models it climbs the slope.  A
    walker that ignores displacement collision but floods BSP air can enter the
    pocket (a 16u drop at x=192) -- its eyes are all at x < 704, west of the hut,
    so the hut stays hidden from them.

How visibility is made certain: the closed "hut" trick of f03/f04.  Every
reachable eye with x < 752 (valley, slope, pocket, west of the pad) is west of
the hut and segments from it to a hut-interior point (x > 768) have x < 864
along their whole length, so they never enter the east-wall slab (x 864..880)
that holds the aperture, and must cross the solid shell.  Eyes beside the hut on
the summit's north/south runs cross the north/south wall slabs.  Only eyes east
of the hut (x >= 880) see in -- reachable only by walking up the slope.
Confirmed against VVIS: the hut's cluster is in the PVS of the east-strip
cluster only; the spawn, slope and north-run clusters do not see it.

NOTE for walker authors: the Python walker (bsp_reader) skips displacement faces,
so it cannot score displacement *sides*; this fixture expects none.  Everything
scored here is a normal brush side.

Expectations:
  visible  hut inner sides: walk up the slope, round the hut, in through the
           east doorway.
  visible  controls: the hut's outer west wall (from the spawn eye at z=65 the
           sight lines to its upper part, z >= 180, clear the terrain by >= 29u),
           the valley's west wall, the room ceiling.
  hidden   hut roof top (z=240): top faces are only visible from above their
           plane; the highest reachable eye is 128 + 85 = 213u, and the roof is
           112u above the summit (> 57 * 1.15 and > the CS:S 93 * 1.15 duck-jump,
           so the roof is no floor either).
"""
import math

import vmfgen

NAME = "f06_disp_hill"
DESCRIPTION = ("Displacement hill (128u rise, slope normal z >= 0.8) with a hut on the summit whose "
               "doorway faces away from the valley; the hut is visible only from the hill.")

X_VALLEY = 192.0                     # valley slab ends, displaced slab begins
X_SUMMIT = 704.0                     # displaced slab ends, summit pad begins
DEPTH = 512.0
HILL_H = 128.0
RAMP_X0, RAMP_WAVE, RAMP_LEN = 224.0, 64.0, 352.0
POWER = 4
MIN_NORMAL_Z = 0.80                  # required; the walkable limit is 0.7
HUT_X0, HUT_X1 = 768.0, 864.0        # interior
HUT_Y0, HUT_Y1 = 176.0, 336.0
HUT_H = 96.0
CS_CROUCH_JUMP_FEET = 93.0           # CS:S duck-jump reach, as in f03


def hill_height(x: float, y: float) -> float:
    xs = RAMP_X0 + RAMP_WAVE * math.sin(math.pi * y / DEPTH)
    t = min(max((x - xs) / RAMP_LEN, 0.0), 1.0)
    return HILL_H * t * t * (3.0 - 2.0 * t)


def heights(u: float, v: float) -> float:
    """vmfgen frame for a top side: u along +x over the displaced slab, v along +y."""
    return hill_height(X_VALLEY + u * (X_SUMMIT - X_VALLEY), v * DEPTH)


def mesh_stats(power: int = POWER):
    """(min normal z over all triangles under both diagonals, number of cells, east border heights)."""
    n = 1 << power
    xs = [X_VALLEY + (X_SUMMIT - X_VALLEY) * j / n for j in range(n + 1)]
    ys = [DEPTH * i / n for i in range(n + 1)]
    z = [[heights(j / n, i / n) for j in range(n + 1)] for i in range(n + 1)]

    def nz(a, b, c):
        ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
        vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
        cx, cy, cz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
        return abs(cz) / math.sqrt(cx * cx + cy * cy + cz * cz)

    low = 1.0
    for i in range(n):
        for j in range(n):
            a = (xs[j], ys[i], z[i][j])
            b = (xs[j + 1], ys[i], z[i][j + 1])
            c = (xs[j + 1], ys[i + 1], z[i + 1][j + 1])
            d = (xs[j], ys[i + 1], z[i + 1][j])
            for tri in ((a, b, c), (a, c, d), (a, b, d), (b, c, d)):
                low = min(low, nz(*tri))
    return low, n * n, [z[i][n] for i in range(n + 1)]


def build(m: vmfgen.Map) -> None:
    low, cells, east_border = mesh_stats()
    assert low >= MIN_NORMAL_Z, f"hill too steep: min normal z {low:.3f}"
    assert all(h == HILL_H for h in east_border), "displaced surface does not meet the summit pad edge"
    assert RAMP_X0 + RAMP_WAVE + RAMP_LEN <= X_SUMMIT
    # Roof top above every reachable eye, and no floor either (even at CS:S duck-jump with margin).
    assert HUT_H + 16 > vmfgen.MAX_EYE_ABOVE_FLOOR + 20
    assert HUT_H + 16 > CS_CROUCH_JUMP_FEET * 1.15

    # VBSP does not treat a displacement brush as solid (see the module docstring): the
    # shell's own floor, at z=-16, is what seals the room under the displaced slab.
    shell = m.room((0, 0, -16), (1024, 512, 384))
    m.entity("info_player_start", origin=(96, 256, 1), angles="0 0 0")

    m.box((0, 0, -16), (X_VALLEY, 512, 0))                     # valley floor, top z=0
    hill = m.box((X_VALLEY, 0, -16), (X_SUMMIT, 512, 0))       # slab carrying the displacement
    m.displace(hill, "top", power=POWER, heights=heights)
    m.box((X_SUMMIT, 0, -16), (1024, 512, HILL_H))             # summit pad, top z=128

    hut = m.room((HUT_X0, HUT_Y0, HILL_H), (HUT_X1, HUT_Y1, HILL_H + HUT_H), floor=False,
                 openings=[("east", (224, 288, HILL_H, HILL_H + 80))])

    m.expect_visible(hut.all_inner(),
                     "hut inner side: the hut stands on the hill summit, doorway faces east away from the "
                     "valley; reachable only by walking up the displacement slope")
    m.expect_visible(hut.outer("west"), "control: hut outer west wall, seen from the spawn over the slope")
    m.expect_visible(shell.inner("west"), "control: valley west wall seen from the spawn")
    m.expect_visible(shell.inner("ceiling"), "control: room ceiling")

    m.expect_hidden(hut.outer("ceiling"),
                    f"hut roof top at z={HILL_H + HUT_H + 16:g}: above every reachable eye (max {HILL_H + 85:g})")

    m.expect_reach(((HUT_X0 + HUT_X1) / 2, (HUT_Y0 + HUT_Y1) / 2, HILL_H),
                   "hut floor on the summit: walk up the displacement slope and in through the east doorway")
    m.expect_reach((950, 100, HILL_H), "summit east of the hut: top of the displacement slope")
    m.expect_no_reach(((HUT_X0 + HUT_X1) / 2, (HUT_Y0 + HUT_Y1) / 2, HILL_H + HUT_H + 16),
                      "hut roof top: 112u above the summit, beyond any duck-jump")

    m.note(f"Hill: {HILL_H:g}u over a {RAMP_LEN:g}u smoothstep; min triangle normal z {low:.3f} "
           f"(both diagonals, {cells} cells). VBSP treats the displacement brush as non-solid. "
           f"Displacement sides are not expected.")

    # verify_expectations.py (#19)
    m.verify_floor(0, 0, X_VALLEY, 512, 0, "valley floor")
    m.verify_floor(X_SUMMIT, 0, 1024, 512, HILL_H, "summit pad, the hut's inside included")
    m.verify_eye_box((X_VALLEY, 0, vmfgen.EYE_DUCK), (X_SUMMIT, 512, HILL_H + 86),
                     "over the displacement slope", role="hidden")
