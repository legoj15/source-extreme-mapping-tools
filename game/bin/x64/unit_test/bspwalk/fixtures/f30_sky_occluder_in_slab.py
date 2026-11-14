"""
f30 -- a 3D skybox face seen only from mapped eyes behind a skybox occluder that lies inside
the mapped eye slab (#39 stage D).

bspwalk's tier B eye samples are the shell of the eye lattice: a lattice height is dropped when
it lies strictly inside the eye volume on its own line and on the 8 lines around it.  That is
enough only when no occluder lies inside the eye volume: a sight line from a face to an interior
eye, extended past the eye, must reach the volume's boundary (where a boundary sample sees the
face too) before it meets an occluder.  The real eye volume is carved by the solids the walker
collides with, so no occluder lies in it; its skybox copy (origin + E / scale) is not carved
against the skybox's own brushes.  Here a skybox wall B stands inside the slab the play area
maps to, and the face T is seen only through a hole in a second wall A that straddles the
slab's far edge: every sight line from T through the hole enters the slab inside A's hole,
which is narrower than every lattice step there, and ends on B.  The eyes that see T all lie
between A and B, strictly inside the slab, and the plain shell drops every one of them.

Geometry (x east, y north, z up; interior bounds, 16u walls):
    M  main room  x 6400..6912  y 1024..1536  z 0..256  floor DEV/DEV_MEASUREGENERIC01, walls and
                  ceiling TOOLS/TOOLSSKYBOX, info_player_start (6656, 1280, 1).  Eyes: x 6416..6896,
                  y 1040..1520 (hull 16 off the walls), z 14..85 (dead eye to the highest jump eye).
    K  sky room   x 3500..4500  y -500..500  z -256..512 (DEV/GRAYGRID, sealed)
                  sky_camera at S = (3600, -64, 64), scale 16.  bspwalk maps every eye piece into
                  the skybox, the sky_camera viewpoint too: S + S / 16 = (3825, -68, 68).  Both lie
                  west of B, so neither sees T.
    Mapped eyes   E' = S + E / 16: x 4001..4031, y 1..31, z 64.875..69.3125 (the slab).  The sky
                  lattice step is the real one / 16: 0.5u at the default 8u, 0.25 and 0.125 at
                  the refinement levels.
    Every brush below is func_detail (one sky cluster: tier A cannot hide anything in K).
    B  wall       x 3995..4010, the whole of K's y and z: covers the slab's west part.
    A  wall       x 4030.5..4031.5, the whole of K's y and z except a hole
                  y 16.046875..16.078125 (1/32u), z 66.5..67.5; four boxes around it.  The slab's
                  east edge x = 4031 lies inside A.
    T  target     x 4400..4416  y 16.75..18.25  z 60..74, east of A.

Why the shell misses T: the only samples on the slab's east edge line (x = 4031) are inside A,
for no lattice line (y a multiple of 0.125) passes the hole, and the hole's walls stand 1/64u
clear of y 16.0 and 16.125 even after bspwalk's 1/32u occluder shrink.  The free part of the
slab (x 4010..4030.5) keeps only its top (z 69.3125) and bottom (64.875) samples and its y = 1 and
y = 31 edge lines.  A y edge line cannot reach a 1/32u hole 15u away through 1u of wall.  A top
sample at x >= 4010 on the line y = 16 must drop at least 1.8125u over at most 20.5u to enter the
hole, so it meets x = 4400 below z = 30 (T: z >= 60); a bottom sample must rise at least 1.625u
over at most 20.5u and meets x = 4400 above z = 95 (T: z <= 74).

must_visible:
    T west (x = 4400, faces -x).  Exact (unshrunk) segment: from the mapped eye position (4012, 16, 66.75), a point of the full lattice (with the keep rule the samples in wall B's skin see the face along nearly the same line)
    (real eye (6592, 1280, 44): a crouched player) to (4400, 17, 67) has dy/dx = 1/388: it crosses
    A's hole at y 16.0477..16.0503, z 66.762..66.763, and B does not lie on it.  The full lattice of
    the tool before #39 has samples there (x 4010.5..4030, y 16, z 65..69 in 0.5 steps).
must_hidden:
    T east (x = 4416, faces +x): every eye has x <= 4031, the real eyes and S lie west of it.
"""
import vmfgen

NAME = "f30_sky_occluder_in_slab"
DESCRIPTION = ("3D skybox (scale 16) whose mapped eye slab holds a skybox wall: a face seen only through a "
               "1/32u hole from mapped eyes strictly inside the slab, between the hole and that wall, stays "
               "visible; its back is hidden.")

SKY_ORIGIN = (3600.0, -64.0, 64.0)
SCALE = 16
GRAY = "DEV/GRAYGRID"
KY0, KY1, KZ0, KZ1 = -500.0, 500.0, -256.0, 512.0
HOLE_Y0, HOLE_Y1 = 16.0 + 3.0 / 64.0, 16.0 + 5.0 / 64.0
HOLE_Z0, HOLE_Z1 = 66.5, 67.5
AX0, AX1 = 4030.5, 4031.5
BX0, BX1 = 3995.0, 4010.0
MX = 5376.0          # the main room's x offset from 1024..1536


def _retexture_floor(room: vmfgen.Room, material: str) -> None:
    for solid, _rect in room._pieces["floor"]:
        solid.side_obj("top").material = material


def build(m: vmfgen.Map) -> None:
    lo, hi = 1040.0, 1520.0
    ex = (SKY_ORIGIN[0] + (lo + MX) / SCALE, SKY_ORIGIN[0] + (hi + MX) / SCALE)    # 4001 .. 4031
    ey = (SKY_ORIGIN[1] + lo / SCALE, SKY_ORIGIN[1] + hi / SCALE)      # 1 .. 31
    ez = (SKY_ORIGIN[2] + vmfgen.EYE_DEAD / SCALE, SKY_ORIGIN[2] + vmfgen.MAX_EYE_ABOVE_FLOOR / SCALE)
    # B covers the slab's west part, A its east edge; the hole is inside the slab's y and z
    assert BX0 < ex[0] < BX1 < AX0 < ex[1] < AX1
    # the sky_camera and its own skybox copy lie west of B
    assert SKY_ORIGIN[0] * (1 + 1.0 / SCALE) < BX0
    assert ey[0] < HOLE_Y0 < HOLE_Y1 < ey[1] and ez[0] < HOLE_Z0 < HOLE_Z1 < ez[1]
    # no line of the finest refinement lattice (0.125u in the sky) passes the hole, shrunk
    # walls included (1/32u off each side)
    step = 8.0 / 4 / SCALE
    shrink = 1.0 / 32.0
    k = int(HOLE_Y0 // step)
    assert k * step < HOLE_Y0 - shrink and HOLE_Y1 + shrink < (k + 1) * step
    # the docstring's top and bottom sample bounds: free part x BX1 .. AX0
    run = AX0 - BX1
    assert ez[1] - (ez[1] - HOLE_Z1) / run * (4400 - BX1) < 60
    assert ez[0] + (HOLE_Z0 - ez[0]) / run * (4400 - BX1) > 74

    main = m.room((MX + 1024, 1024, 0), (MX + 1536, 1536, 256), material=vmfgen.MAT_SKYBOX)
    _retexture_floor(main, vmfgen.MAT_WALL)
    m.entity("info_player_start", origin=(MX + 1280, 1280, 1), angles="0 0 0")

    m.room((3500, KY0, KZ0), (4500, KY1, KZ1), material=GRAY)
    m.entity("sky_camera", origin=SKY_ORIGIN, scale=SCALE, fogenable=0)

    wall_b = m.box((BX0, KY0, KZ0), (BX1, KY1, KZ1))
    wall_a = [
        m.box((AX0, KY0, KZ0), (AX1, KY1, HOLE_Z0)),                # below the hole
        m.box((AX0, KY0, HOLE_Z1), (AX1, KY1, KZ1)),                # above it
        m.box((AX0, KY0, HOLE_Z0), (AX1, HOLE_Y0, HOLE_Z1)),        # south of it
        m.box((AX0, HOLE_Y1, HOLE_Z0), (AX1, KY1, HOLE_Z1)),        # north of it
    ]
    target = m.box((4400, 16.75, 60), (4416, 18.25, 74))
    m.brush_entity("func_detail", [wall_b] + wall_a + [target])

    m.expect_visible(target.side("west"), "T west: seen through A's 1/32u hole from mapped eyes inside the slab "
                     "between A and B, e.g. along (4012,16,66.75)->(4400,17,67)")
    m.expect_visible(main.inner("floor"), "control: floor of the room the player spawns in")
    m.expect_hidden(target.side("east"), "T east: faces +x, every eye has x <= 4031")

    m.note(f"sky_camera origin {SKY_ORIGIN}, scale {SCALE}; mapped eyes x {ex[0]}..{ex[1]}, y {ey[0]}..{ey[1]}, "
           f"z {ez[0]:.3f}..{ez[1]:.2f}. B (x {BX0}..{BX1}) stands inside the slab; A's hole "
           f"y {HOLE_Y0}..{HOLE_Y1}, z {HOLE_Z0}..{HOLE_Z1} at x {AX0}..{AX1}.")

    # verify_expectations.py (#19)
    m.verify_floor(MX + 1024, 1024, MX + 1536, 1536, 0, "main room floor")
    m.verify_sky(SKY_ORIGIN, SCALE)
