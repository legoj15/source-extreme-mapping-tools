"""
f11 -- a large floor in an unreachable room, seen only through a 4u slit.

The spawn room S and a big hall H share one wall.  The wall has a single slit,
4u wide and 16u deep (the wall's thickness), 160u tall, centred on x=256.  A
32u hull cannot pass it, but the player can stand and look through it.  Every
sight line into H runs down that tunnel, and that constrains it hard:

    a straight line that crosses both planes y=512 and y=528 inside x 254..258
    has |dx/dy| <= (258-254)/(528-512) = 0.25, whatever the eye's position.

Geometry (interior bounds, 16u walls; x east, y north):
    S  spawn room   x 0..512      y 0..512     z 0..192   info_player_start (256,256,1)
    H  hall         x -256..768   y 528..1552  z 0..192   (1024 x 1024 floor)
    wall slab       y 512..528    belongs to H as its south wall; S has no north wall.
    slit            x 254..258    z 0..160     through the slab
    crate           x 236..276    y 700..740   z 0..48    on H's floor, on the slit axis
H is not marked: VBSP floods into it through the slit (if it did not, H would be
filled solid and every H expectation would report "no BSP face").  H and the crate
use DEV/GRAYGRID so that H's floor and ceiling never merge with S's coplanar,
same-material floor and ceiling into one BSP face.

Reachable eyes (the only ones): x 16..496, y 16..496, z 28..85 over S's floor
(hull 16 off every wall; crouch eye 28, highest jump eye ~85).

must_visible, each with an explicit unobstructed line from the eye E=(256,480,64):
    H floor     to (256,600,0)       z at the slit (y 512..528): 47..38, inside 0..160
    H ceiling   to (256,600,192)     z at the slit: 98..115, inside 0..160
    H north wall (y=1552, faces -y)  to (256,1552,128): z at the slit 66..67; at y=700 it is
                                     64+64*220/1072 = 77 > 48, so it clears the crate
    crate south face (y=700)         to (256,700,24); crate top (z=48) to (256,720,48)
    wall slab, S side (y=512)        standing in S looking north
    all of S                         the player stands in it
must_hidden, each a proof:
    H south wall, H side (y=528, faces +y)
        A face is seen only from its front half-space.  Every eye has y <= 496 < 528.
    H west wall (x=-256) and east wall (x=768)
        The nearest point of either wall is >= 510u from the slit sideways and at most
        1040u beyond it, so the line would need |dx/dy| >= 510/1040 = 0.49, twice the
        0.25 the tunnel allows.
    crate north face (y=740, faces +y)    same half-space argument as the wall's back
    crate west face (x=236, faces -x) and east face (x=276, faces +x)
        West face: front half-space is x<236.  A line through the slit has x in 254..258
        at y=528; to reach x<236 at the crate it must move west, but then it approached
        the face from x>236, its back.  Moving east it only ever has x>=254.  The east
        face is the mirror image.
VVIS facts measured on the compiled BSP: the slit portal survives (H is not filled), and
the PVS of the spawn room's clusters contains every hall cluster, so the PVS (tier A)
proves none of the 8 must_hidden sides; only the ray tier can.  That is the point: a
walker whose sampling lets rays through a 4u slit at the wrong slope calls the hall's side
walls visible (missed savings), one that undersamples calls the floor hidden (a black face).
"""
import vmfgen

NAME = "f11_slit_view"
DESCRIPTION = ("1024x1024 hall in an unreachable room seen only through a 4u x 16u-deep slit; "
               "side walls and back faces provably hidden.")

SLIT_X0, SLIT_X1 = 254.0, 258.0
SLIT_Z1 = 160.0
WALL = 16.0
HALL_MAT = "DEV/GRAYGRID"


def build(m: vmfgen.Map) -> None:
    # Margins that the docstring's argument depends on.
    assert SLIT_X1 - SLIT_X0 < vmfgen.HULL_STAND[0] / 4          # nothing like passable
    slope_max = (SLIT_X1 - SLIT_X0) / WALL                        # 0.25
    hall_x0, hall_x1, hall_y0, hall_y1 = -256.0, 768.0, 528.0, 1552.0
    slit_c = (SLIT_X0 + SLIT_X1) / 2
    assert slit_c - hall_x0 == hall_x1 - slit_c == 512.0
    # West/east hall walls need a slope of at least dx/dy_max, where dy runs from the
    # slit's far plane to the hall's far wall.
    need = (SLIT_X0 - hall_x0) / (hall_y1 - hall_y0 + WALL)
    assert need > 1.8 * slope_max

    spawn = m.room((0, 0, 0), (512, 512, 192), omit=("north",))
    m.entity("info_player_start", origin=(256, 256, 1), angles="0 90 0")

    hall = m.room((hall_x0, hall_y0, 0), (hall_x1, hall_y1, 192), material=HALL_MAT,
                  openings=[("south", (SLIT_X0, SLIT_X1, 0, SLIT_Z1))])
    crate = m.box((236, 700, 0), (276, 740, 48), material=HALL_MAT)

    m.expect_visible(spawn.all_inner(), "inner side of the room the player spawns in")
    m.expect_visible(hall.outer_all("south"), "S side of the slit wall: faces the spawn room")
    m.expect_visible(hall.inner("floor"),
                     "hall floor: line (256,480,64)->(256,600,0) runs down the slit axis")
    m.expect_visible(hall.inner("ceiling"),
                     "hall ceiling: line (256,480,64)->(256,600,192) stays inside slit z 0..160")
    m.expect_visible(hall.inner("north"),
                     "hall far wall: line (256,480,64)->(256,1552,128) clears the 48u crate")
    m.expect_visible(crate.side("south"), "crate face toward the slit: line to (256,700,24)")
    m.expect_visible(crate.side("top"), "crate top: eye 64 is above 48, line to (256,720,48)")

    m.expect_hidden(hall.inner_all("south"),
                    "slit wall, hall side: faces +y, every eye has y<=496")
    m.expect_hidden(hall.inner("west"),
                    "hall west wall: needs |dx/dy|>=0.49, the 4u x 16u slit allows 0.25")
    m.expect_hidden(hall.inner("east"),
                    "hall east wall: needs |dx/dy|>=0.49, the 4u x 16u slit allows 0.25")
    m.expect_hidden(crate.side("north"), "crate back face: faces +y, every eye has y<=496")
    m.expect_hidden(crate.side("west"),
                    "crate west face (-x): a line through the slit approaches it from x>236, its back")
    m.expect_hidden(crate.side("east"),
                    "crate east face (+x): a line through the slit approaches it from x<276, its back")

    m.note("Slit 4u wide, 16u deep: tunnel slope limit 0.25. Hall walls need 0.49.")

    # verify_expectations.py (#19)
    m.verify_floor(0, 0, 512, 512, 0, "spawn room floor")
