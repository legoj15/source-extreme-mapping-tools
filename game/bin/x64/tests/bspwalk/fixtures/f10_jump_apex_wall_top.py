"""
f10 -- faces that are visible only at the apex of a jump (eye above the standing eye height).

Physics used (HL2, vmfgen constants): standing eye 64u over the floor, jump apex +21u
(standing jump) so the eye peaks at 64 + 21 = 85u; a duck-jump lifts the feet 57u and the
crouched eye is 28u, also 85u.  MAX_EYE_ABOVE_FLOOR = 85 is the highest eye any reachable
position has over a floor.  Nothing in this map lets the player stand on anything but the
z=0 floor: every wall and block top is above duck-jump reach (57u, 65.6u with the walker's
15% margin).

A face whose normal points up is visible only from eyes ABOVE its plane (the eye must be on
the front side).  So an up-facing face at 64 < z < 85 is seen by a jumping player and by no
standing or crouching one.  Two heights are used:

    VIS_TOP  72u  : 8u above the standing eye, 13u under the apex eye.  Visible, apex only.
    HID_TOP 128u  : 43u above the apex eye, above 85 * 1.15 = 98 (the walker's margin) and
                    above the CS:S standing-jump eye (57 + 64 = 121).  Seen by nobody.

Layout (plan view, x east, one hall 1624 x 512, ceiling at z=320, nothing climbable):

    x: 0 ..... 320 | 336 ................ 1104 | 1112 ........... 1624
       [ zone T ]  W_T      main zone      W_V   [   zone V    ]
       floor z=0   z..128   spawn, niches  z..72   floor z=0
                   (16 thick)              (8 thick)

    W_T and W_V span the full hall width (y 0..512) and stop under the ceiling, so air is
    contiguous over them (VBSP floods all three zones) but neither can be crossed: 128 and
    72 are both above 57.

    Niche blocks N1 / N2 (x 640..704, open to the west): a lower block up to the niche floor,
    a 40u-high niche (48u deep, 128u wide, closed by two side pieces and a back piece) and an
    upper block over it.  N1 niche floor z=72 (VIS_TOP), N2 niche floor z=128 (HID_TOP).

Why each expectation is certain:
  visible  W_V top (z=72, an 8u strip): any apex eye next to it is 13u above its plane and nothing
           stands between.  Seen at no other eye height.
  visible  N1 niche floor (z=72): from an apex eye in front of the niche opening the sight line
           to a floor point 0..48u inside is 13u above the floor at the lip and stays inside the
           72..112 opening, so it is clear.  Standing/crouching eyes (<= 64) are under the floor
           plane and see its back.
  visible  zone V floor (z=0), x >= about 1250: the apex eye (85) 16u from W_V (hull against it)
           clears the 8u thick rim (72) and the sight line reaches the floor 85 * 24 / 13 = 157u past
           the eye, x >= 1245.  Zone V runs to x=1624, so about 370u of floor is seen.  A standing
           eye (64) is under the rim and sees only the wall.
  hidden   W_T top (z=128), N2 niche floor (z=128): above every eye that can exist (<= 85).
  hidden   zone T floor (z=0): a sight line from an eye <= 85 to a floor point behind W_T descends
           and crosses the plane of the wall below its 128u rim; nobody is high enough to look over it.
  hidden   W_T west face: the back of a face whose plane (x=320) has every reachable eye on the
           other side (x >= 352).
  visible  controls: the spawn-zone floor, W_T east face, W_V west face.
"""
import vmfgen

NAME = "f10_jump_apex_wall_top"
DESCRIPTION = ("Up-facing faces at 72u (visible only at the apex of a jump) and 128u (never): "
               "wall tops, niche floors and the floor behind a 72u wall.")

VIS_TOP = 72.0
HID_TOP = 128.0

SHELL_LO = -16.0
FLOOR_BOT = -32.0
CEIL = 320.0

X_T_END = 320.0            # zone T: 0..320
X_WT_END = 336.0           # wall T: 320..336, top HID_TOP
X_MAIN_END = 1104.0        # main zone: 336..1104
X_WV_END = 1112.0          # wall V: 1104..1112, top VIS_TOP
X_END = 1624.0             # zone V: 1112..1624
Y_END = 512.0


def _niche_block(m: vmfgen.Map, bx0: float, by0: float, by1: float, floor_z: float) -> vmfgen.Solid:
    """A block with a west-facing niche (floor at floor_z).  Returns the lower block (its top side is the niche floor)."""
    depth, niche_depth, side, h_niche, h_top = 64.0, 48.0, 32.0, 40.0, 60.0
    bx1 = bx0 + depth
    lower = m.box((bx0, by0, 0), (bx1, by1, floor_z))
    m.box((bx0, by0, floor_z), (bx1, by0 + side, floor_z + h_niche))
    m.box((bx0, by1 - side, floor_z), (bx1, by1, floor_z + h_niche))
    m.box((bx0 + niche_depth, by0 + side, floor_z), (bx1, by1 - side, floor_z + h_niche))
    m.box((bx0, by0, floor_z + h_niche), (bx1, by1, floor_z + h_niche + h_top))
    return lower


def build(m: vmfgen.Map) -> None:
    eye, top = vmfgen.EYE_STAND, vmfgen.MAX_EYE_ABOVE_FLOOR
    # VIS_TOP must be clearly above the standing eye and clearly under the apex eye.
    assert eye + 8 <= VIS_TOP <= top - 8
    # HID_TOP must be above every eye even with the walker's 15% margin on the jump rise.
    assert HID_TOP > top * 1.15 + 20
    assert HID_TOP > vmfgen.CROUCH_JUMP_FEET + vmfgen.EYE_STAND            # CS:S-like standing jump eye
    # Neither wall can be climbed, even with the margin.
    assert VIS_TOP > vmfgen.CROUCH_JUMP_FEET * 1.15 + 5
    # Over-the-wall geometry: the apex eye is 16u from W_V, rim 8u thick, so the floor is seen
    # beyond 85 * 24 / (85 - VIS_TOP) from the eye; make sure a good stretch of zone V lies beyond that.
    w_v_thick = X_WV_END - X_MAIN_END
    reach = top * (16.0 + w_v_thick) / (top - VIS_TOP)
    x_first = X_MAIN_END - 16.0 + reach
    assert X_END - x_first > 300, (x_first, X_END)

    # Shell without a floor: the floor boxes below are the floor (as in f04).
    shell = m.room((0, 0, SHELL_LO), (X_END, Y_END, CEIL), floor=False)
    m.entity("info_player_start", origin=(480, 256, 1), angles="0 0 0")

    floor_t = m.box((0, 0, FLOOR_BOT), (X_T_END, Y_END, 0))
    wall_t = m.box((X_T_END, 0, FLOOR_BOT), (X_WT_END, Y_END, HID_TOP))
    floor_main = m.box((X_WT_END, 0, FLOOR_BOT), (X_MAIN_END, Y_END, 0))
    wall_v = m.box((X_MAIN_END, 0, FLOOR_BOT), (X_WV_END, Y_END, VIS_TOP))
    floor_v = m.box((X_WV_END, 0, FLOOR_BOT), (X_END, Y_END, 0))

    n1 = _niche_block(m, 640, 32, 224, VIS_TOP)
    n2 = _niche_block(m, 640, 288, 480, HID_TOP)

    m.expect_visible(wall_v.side("top"), "72u wall top: only an apex eye (85) is above its plane")
    m.expect_visible(n1.side("top"), "72u niche floor: only an apex eye (85) is above it, and it sees in")
    m.expect_visible(floor_v.side("top"),
                     "floor behind the 72u wall: seen over the 8u rim from an apex eye, 157u past the eye and beyond")
    m.expect_visible(floor_main.side("top"), "control: floor of the spawn zone")
    m.expect_visible(wall_t.side("east"), "control: face of the 128u wall that looks at the spawn zone")
    m.expect_visible(wall_v.side("west"), "control: face of the 72u wall that looks at the spawn zone")

    m.expect_hidden(wall_t.side("top"), "128u wall top: above every eye (max 85)")
    m.expect_hidden(n2.side("top"), "128u niche floor: above every eye (max 85)")
    m.expect_hidden(floor_t.side("top"), "floor behind the 128u wall: no eye is high enough to look over it")
    m.expect_hidden(wall_t.side("west"), "back of the 128u wall's west face: every reachable eye is east of x=352")

    m.note(f"VIS_TOP {VIS_TOP}u (standing eye {eye}u, apex eye {top:.0f}u), HID_TOP {HID_TOP}u. "
           f"Zone V floor is seen from x>={x_first:.0f} (nominal apex eye {top:.0f}u).")

    # verify_expectations.py (#19)
    m.verify_floor(X_WT_END, 0, X_MAIN_END, Y_END, 0, "main zone floor")
