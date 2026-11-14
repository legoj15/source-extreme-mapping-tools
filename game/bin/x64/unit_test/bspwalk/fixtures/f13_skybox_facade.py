"""
f13 -- a 3D skybox holding a building facade, seen from the playable room.

A 3D skybox is rendered by a camera at  sky_origin + eye / scale  for every player
eye (design doc, Stage 4).  Its faces are therefore judged from those mapped eyes,
never from reachability.  The facade's back and a hut whose door turns away from the
mapped eyes are hidden even though they share a sealed room with visible faces.

Geometry (x east, y north, z up; interior bounds, 16u walls):
    M  main room  x 0..512  y 0..512  z 0..256   floor DEV/DEV_MEASUREGENERIC01,
                  walls and ceiling TOOLS/TOOLSSKYBOX, info_player_start (256,256,1).
                  Reachable eyes: x,y 16..496 (hull 16 off the walls), z 28..85
                  (crouch eye 28, highest jump eye 85; vmfgen.MAX_EYE_ABOVE_FLOOR).
    K  sky room   x 4000..6000  y -1000..1000  z 0..1024  (DEV/GRAYGRID, sealed)
                  sky_camera at S=(4096,0,64), scale 16.
    Mapped eyes   E' = S + E/16  ->  x 4097..4127, y 1..31, z 65.75..69.33.
                  (This is what the walker must do; the Python walker has no such step.)
    Facade, all DEV/DEV_MEASUREGENERIC01 boxes standing on K's floor:
        F0  x 5000..5120  y -320..320  z 0..640     the tower; its west face is the front
        W1  x 4800..5000  y  100..320  z 0..300     wing in front of the tower, north
        W2  x 4800..5000  y -320..-100 z 0..300     wing in front of the tower, south
    Hut in K (a decoy that only mapped eyes could expose):
        interior x 4300..4460  y -700..-540  z 0..96, walls 16 (outer x 4284..4476,
        y -716..-524, roof top z=112), doorway in its SOUTH wall (x 4350..4410, z 0..80).

A face is seen from a point only if the point is in its front half-space
(face normal . (E' - P) > 0) and the segment is clear.  Each claim below is a
half-space test over the whole E' box; each visible one also has an explicit clear
segment from E'=(4110,15,69), checked numerically against every solid.

must_visible (E' is in front of the plane, segment clear):
    F0 west   x=5000, faces -x    to (5000,0,300): the segment stays at |y|<=15, between the wings
    W1 west   x=4800, faces -x    to (4800,200,200)
    W1 south  y=100,  faces -y    to (4900,100,200): y stays below 100 until the target
    W2 west   x=4800, faces -x    to (4800,-200,200)
    W2 north  y=-100, faces +y    to (4900,-100,200)
    hut west  x=4284, faces -x    to (4284,-620,60)
    hut north y=-524, faces +y    to (4380,-524,60): it faces the mapped eyes (y>=1)
    K floor   z=0,   faces +z     to (4500,0,0);  K ceiling z=1024, faces -z: to (4500,0,1024)
    M floor, M ceiling            the player stands on the floor (control); the ceiling is
                                  TOOLSSKYBOX but is still a BSP face the player sees (control)
Facade and wing targets are above the mapped eye.  The hut targets are 6-9u below it over
600u of run (slope 0.015) and the K floor line descends 0.18 per unit; from a standing eye
(64) in the middle of M the sky wall is 256u away, a drop of at most 46u, so the same
direction meets a sky wall before M's floor and the sky shows there in the real game too.
The walker evaluates all directions anyway.
must_hidden (E' is behind the plane for EVERY mapped eye):
    F0 east  x=5120 faces +x      E'.x <= 4127 < 5120
    F0 north y=320  faces +y      E'.y <= 31   < 320      (W1's north side shares this plane)
    F0 south y=-320 faces -y      E'.y >= 1    > -320     (W2's south side shares this plane)
    F0 top   z=640  faces +z      E'.z <= 69.4 < 640
    W1 top, W2 top  z=300         E'.z <= 69.4 < 300
    hut inside (all)              the doorway is in the south wall, plane y=-700; every mapped
                                  eye has y >= 1 and every interior point has y > -700, so a
                                  segment never gets into the slab y -716..-700 that holds
                                  the aperture; it must cross the shell
    hut south outer, faces -y     E'.y >= 1 > -716;  hut east outer faces +x: E'.x < 4476
    hut roof top z=112 faces +z   E'.z <= 69.4 < 112
VVIS facts measured on the compiled BSP: every leaf of the sky room is in ONE cluster
(25 leaves, cluster 0, area 1; the main room is cluster 1, area 2), so the PVS cannot
separate anything inside the sky room: all the hidden sides above are tier B at best.
"""
import vmfgen

NAME = "f13_skybox_facade"
DESCRIPTION = ("3D skybox (sky_camera scale 16) in a sealed room holding a building facade; "
               "faces toward the mapped eyes visible, backs and a decoy hut hidden.")

SKY_ORIGIN = (4096.0, 0.0, 64.0)
SCALE = 16
GRAY = "DEV/GRAYGRID"


def _retexture_floor(room: vmfgen.Room, material: str) -> None:
    """Give a room's floor (inner side) its own material.

    vmfgen.Map.room() has one material for the whole shell and no per-face override,
    so this reaches into the Room's pieces.  (vmfgen gap, reported.)
    """
    for solid, _rect in room._pieces["floor"]:
        solid.side_obj("top").material = material


def build(m: vmfgen.Map) -> None:
    lo_eye, hi_eye = 16.0, 496.0
    z_lo, z_hi = vmfgen.EYE_DUCK, vmfgen.MAX_EYE_ABOVE_FLOOR
    ex = (SKY_ORIGIN[0] + lo_eye / SCALE, SKY_ORIGIN[0] + hi_eye / SCALE)       # 4097 .. 4127
    ey = (SKY_ORIGIN[1] + lo_eye / SCALE, SKY_ORIGIN[1] + hi_eye / SCALE)       # 1 .. 31
    ez = (SKY_ORIGIN[2] + z_lo / SCALE, SKY_ORIGIN[2] + z_hi / SCALE)           # 65.75 .. 69.33
    # The half-space claims in the docstring.
    assert ex[1] < 4284 < 4476 and ex[1] < 5000 < 5120
    assert ey[0] > -100 and ey[1] < 100 and ey[1] < 320 and ey[0] > -320
    assert ey[0] > -524 and ey[0] > -716
    assert ez[1] < 112 < 300 < 640

    # Main room: sky on every wall and the ceiling, normal floor.
    main = m.room((0, 0, 0), (512, 512, 256), material=vmfgen.MAT_SKYBOX)
    _retexture_floor(main, vmfgen.MAT_WALL)
    m.entity("info_player_start", origin=(256, 256, 1), angles="0 0 0")

    # Sky room with the camera inside it.
    sky = m.room((4000, -1000, 0), (6000, 1000, 1024), material=GRAY)
    m.entity("sky_camera", origin=SKY_ORIGIN, scale=SCALE, fogenable=0)

    tower = m.box((5000, -320, 0), (5120, 320, 640))
    wing_n = m.box((4800, 100, 0), (5000, 320, 300))
    wing_s = m.box((4800, -320, 0), (5000, -100, 300))
    hut = m.room((4300, -700, 0), (4460, -540, 96), floor=False,
                 openings=[("south", (4350, 4410, 0, 80))])

    m.expect_visible(tower.side("west"), "facade front: faces the mapped eyes, line (4110,15,69)->(5000,0,300)")
    m.expect_visible(wing_n.side("west"), "north wing front: faces -x, mapped eyes at x<=4127")
    m.expect_visible(wing_n.side("south"), "north wing south side: faces -y, mapped eyes have y<=31<100")
    m.expect_visible(wing_s.side("west"), "south wing front: faces -x")
    m.expect_visible(wing_s.side("north"), "south wing north side: faces +y, mapped eyes have y>=1>-100")
    m.expect_visible(hut.outer("west"), "sky hut west face: faces -x toward the mapped eyes")
    m.expect_visible(hut.outer("north"), "sky hut north face: faces +y, the mapped eyes are at y>=1>-524")
    m.expect_visible(sky.inner("floor"), "sky room floor: below every mapped eye, faces up")
    m.expect_visible(sky.inner("ceiling"), "sky room ceiling: above every mapped eye, faces down")
    m.expect_visible(main.inner("floor"), "control: floor of the room the player spawns in")
    m.expect_visible(main.inner("ceiling"), "control: TOOLSSKYBOX ceiling of the room the player spawns in")

    m.expect_hidden(tower.side("east"), "facade back: faces +x, every mapped eye has x<=4127")
    m.expect_hidden(tower.side("north"), "tower north side: faces +y, every mapped eye has y<=31<320")
    m.expect_hidden(tower.side("south"), "tower south side: faces -y, every mapped eye has y>=1>-320")
    m.expect_hidden(tower.side("top"), "tower roof: faces up, every mapped eye is below z=69.4")
    m.expect_hidden(wing_n.side("top"), "north wing roof: faces up, every mapped eye is below 69.4")
    m.expect_hidden(wing_s.side("top"), "south wing roof: faces up, every mapped eye is below 69.4")
    m.expect_hidden(hut.all_inner(), "sky hut interior: doorway faces -y, away from every mapped eye")
    m.expect_hidden(hut.outer_all("south"), "sky hut south face: faces -y, mapped eyes have y>=1")
    m.expect_hidden(hut.outer_all("east"), "sky hut east face: faces +x, mapped eyes have x<=4127")
    m.expect_hidden(hut.outer("ceiling"), "sky hut roof top z=112: above every mapped eye")

    m.note(f"sky_camera origin {SKY_ORIGIN}, scale {SCALE}; mapped eyes x {ex[0]}..{ex[1]}, "
           f"y {ey[0]}..{ey[1]}, z {ez[0]:.2f}..{ez[1]:.2f}.")

    # verify_expectations.py (#19)
    m.verify_floor(0, 0, 512, 512, 0, "main room floor")
    m.verify_sky(SKY_ORIGIN, SCALE)
