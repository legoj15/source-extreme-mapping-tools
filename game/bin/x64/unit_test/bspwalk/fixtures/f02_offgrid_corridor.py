"""
f02 -- a 32.5u-wide, off-grid, L-shaped corridor to a far room.

The player hull is 32 wide, so a 32.5u passage leaves 0.25u of slack each
side.  The corridor's walls sit at y = 7.25 / 39.75 and x = 367.5 / 400, none
on a multiple of 16, and the centre line (y 23.5, x 383.75) is more than 0.25u
from every cell centre of a 16u grid (centres at multiples of 16, or of 16
offset by 8).  A 16u-grid walker therefore cannot place a hull in it.  An exact
(Minkowski) walker sees a 0.5u-wide free strip and walks through it.

The coordinates are fractional (7.25, 39.75, 367.5, 55.75 ...).  The VMF
format allows fractional plane points (Hammer writes them for off-grid
brushes); every value here is a multiple of 1/4, exactly representable in
float32.  Integer plane points are not required, and an integer 33u corridor
would not be 32.5u.

Layout (plan view, y up; interior bounds):
    spawn room S    x 0..256     y -112..160   z 0..128
    corridor C1     x 272..400   y 7.25..39.75 z 0..96   leaves S's east wall
    corridor C2     x 367.5..400 y 55.75..300  z 0..96   turns north off C1's
                                                          last 32.5u
    far room F      x 256..560   y 316..560    z 0..128   C2 enters F's south wall
    hidden room H   x 592..800   y 316..560    z 0..128   sealed, east of F
The 90 degree turn means nothing in F is visible from S: a straight corridor
would show the far room through a thin slit.  Reaching F requires walking the
whole 32.5u passage.

Why each expectation is certain:
  visible  F's inner sides: the player walks C1 -> C2 -> F (a 32 hull fits a
           32.5 passage; the corner square is also 32.5 x 32.5), so eyes in F
           see all of them.  The corridor inner sides are visible to a player
           standing in the corridor, and C1's are also visible from S.
  hidden   H's inner sides: sealed box (info_target keeps VBSP from filling
           it); no opening, so no sight line enters.
"""
import vmfgen

NAME = "f02_offgrid_corridor"
DESCRIPTION = "32.5u off-grid L-shaped corridor from a spawn room to a far room, plus a sealed room off the far room."

Y0, Y1 = 7.25, 39.75          # corridor C1 width 32.5
X0, X1 = 367.5, 400.0         # corridor C2 width 32.5
H_CORR = 96


def build(m: vmfgen.Map) -> None:
    assert abs((Y1 - Y0) - 32.5) < 1e-9 and abs((X1 - X0) - 32.5) < 1e-9
    assert (Y1 - Y0) > vmfgen.HULL_STAND[0] and H_CORR > vmfgen.HULL_STAND[2]

    spawn = m.room((0, -112, 0), (256, 160, 128), openings=[("east", (Y0, Y1, 0, H_CORR))])
    m.entity("info_player_start", origin=(96, 24, 1), angles="0 0 0")

    # C1 has no west wall of its own: S's east wall (with the opening) closes it.
    c1 = m.room((272, Y0, 0), (400, Y1, H_CORR), omit=("west",),
                openings=[("north", (X0, X1, 0, H_CORR))])
    # C2 has neither south nor north wall: C1's north wall (opening) and F's south wall close it.
    c2 = m.room((X0, Y1 + 16, 0), (X1, 300, H_CORR), omit=("south", "north"))
    far = m.room((256, 316, 0), (560, 560, 128), openings=[("south", (X0, X1, 0, H_CORR))])
    hidden = m.room((592, 316, 0), (800, 560, 128), marker=True)

    m.expect_visible(far.all_inner(), "far room inner side: reachable only through the 32.5u corridor")
    m.expect_visible(c1.all_inner(), "C1 corridor inner side: visible from inside the corridor")
    m.expect_visible(c2.all_inner(), "C2 corridor inner side: visible from inside the corridor")
    m.expect_visible(spawn.all_inner(), "spawn room inner side")
    m.expect_hidden(hidden.all_inner(), "sealed room off the far room: no opening")

    m.note("Corridor is 32.5u wide at off-grid coordinates; hull is 32u so slack is 0.25u per side.")
