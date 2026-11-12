"""
f27 -- a face seen only in the reflection a water displacement shows (#78).

f16's ledge underside, with the water under it a displacement instead of a brush.

What the engine does with a water material on a displacement (this repo's vbsp, the SDK
client; the engine's own renderer is not in the tree, so the last step is inferred):
  * vbsp removes a brush with a displaced side from the world (map.cpp, "remove them from the
    world") and emits the displaced face from mapdispinfo, whose fogVolumeLeaf is never set
    (disp_vbsp.cpp DispGetFaceInfo): the face has no water leaf and surfaceFogVolumeID -1
    (writebsp.cpp EmitFace, ivp.cpp WriteFogVolumeIDs).  It is a drawn surface with the
    water material, not a fog volume.
  * The client renders one water reflection per view, for the visible fog volume, about the
    horizontal plane at that volume's surface height (viewrender.cpp CAboveWaterView /
    CReflectionView, m_fogInfo.m_flWaterHeight), into the one reflection texture that every
    Water material with $reflecttexture samples in screen space.
  * So a water displacement shows the reflection of ANOTHER water's plane, through its own
    screen footprint.  bspwalk models it as a mirror about each plane the engine may reflect
    about (each fog volume's surface, facing up; the up-facing planes of the other reflective
    faces; a func_reflective_glass's planes), whose window is the displacement's bounds.  The
    underwater view renders refraction only (CUnderWaterView), so a water face's side (here
    the pool's east side, x=128, open to the displacement's removed brush) is no mirror plane:
    the strict control below FAILs if it were one.

Geometry (x east, y north, z up; interior bounds, 16u walls), f16's pit:
    P   pit room   x -256..512  y 0..512  z -256..320
    walkway block  x -256..0    y 0..512  z -256..0     the player's floor, z=0
    playerclip     x -16..0     y 0..512  z 0..320      keeps the player off the rim
    pool           x 0..128     y 0..512  z -256..-192  a water BRUSH: the fog volume whose
                                                        surface plane z=-192 is reflected
    D              x 128..512   y 0..512  top z=-192    a flat power-2 displacement with the
                                                        same water material (its brush is gone)
    ledge          x 384..512   y 0..512  z -112..-96   its underside z=-112 faces down
info_player_start at (-128,256,1).  Eyes: x -240..-32, y 16..496, z 28..85 over z=0.

Why the underside is seen by no direct eye: every eye is above it (z >= 28 > -112).
Why the pool alone shows it to no eye: E=(x,y,z) mirrors to (x, y, -384 - z); the line from
there to an underside point (x >= 384, z -112) crosses z=-192 at the fraction
(192 + z) / (272 + z) >= 0.73 of the way, so at x >= -240 + 0.73 * 624 = 217 > 128: past the pool.
Why D shows it: E=(-32,256,64) mirrors to (-32,256,-448); the line to (450,256,-112) crosses
z=-192 at Q=(335,256,-192), on D; E -> Q passes the rim at z=41.7, and Q -> P passes x=384 at
z=-158, below the ledge.  Without the #78 rule bspwalk never traces through D: tier B FAILs.
must_hidden (strict): the walkway block's east face (x=0, faces +x): every eye, real or
mirrored, has x <= -32.
"""
import vmfgen

NAME = "f27_disp_water_mirror"
DESCRIPTION = ("A ledge underside seen only in the reflection a water displacement shows (the plane of "
               "the map's water brush): displacement reflectives get the mirror rule (#78).")

WATER_MAT = "DEV/DEV_WATER3_EXP"
CLIP_MAT = "TOOLS/TOOLSPLAYERCLIP"
WATER_Z = -192.0
POOL_X1 = 128.0
LEDGE_BOTTOM, LEDGE_TOP = -112.0, -96.0


def build(m: vmfgen.Map) -> None:
    eye = (-32.0, 256.0, 64.0)
    probe = (450.0, 256.0, LEDGE_BOTTOM)
    mirror_z = 2 * WATER_Z - eye[2]
    t = (WATER_Z - mirror_z) / (probe[2] - mirror_z)
    qx = eye[0] + t * (probe[0] - eye[0])
    assert POOL_X1 < qx < 384                                           # Q is on D, left of the ledge
    z_at_rim = eye[2] + (WATER_Z - eye[2]) * (0 - eye[0]) / (qx - eye[0])
    assert z_at_rim > 20                                                # clear over the walkway edge
    z_at_ledge = WATER_Z + (probe[2] - WATER_Z) * (384 - qx) / (probe[0] - qx)
    assert z_at_ledge < LEDGE_BOTTOM                                    # under the ledge until P
    assert LEDGE_BOTTOM < vmfgen.EYE_DUCK - 1                           # no eye is below the underside
    # the pool reflects no underside point to any eye: its crossing is past x = POOL_X1
    for ez in (vmfgen.EYE_DUCK, vmfgen.MAX_EYE_ABOVE_FLOOR):
        frac = (WATER_Z - (2 * WATER_Z - ez)) / (LEDGE_BOTTOM - (2 * WATER_Z - ez))
        assert -240 + frac * (384 + 240) > POOL_X1

    m.room((-256, 0, -256), (512, 512, 320))
    m.entity("info_player_start", origin=(-128, 256, 1), angles="0 0 0")
    walkway = m.box((-256, 0, -256), (0, 512, 0))
    m.box((-16, 0, 0), (0, 512, 320), material=CLIP_MAT, tag="playerclip")
    pool = m.box((0, 0, -256), (POOL_X1, 512, WATER_Z), material=WATER_MAT, tag="water")
    dbrush = m.box((POOL_X1, 0, -256), (512, 512, WATER_Z), material=WATER_MAT)
    m.displace(dbrush, "top", power=2, material=WATER_MAT)
    ledge = m.box((384, 0, LEDGE_BOTTOM), (512, 512, LEDGE_TOP))

    m.expect_visible(ledge.side("bottom"),
                     "water displacement mirror: the mirrored eye (-32,256,-448) sees the underside through "
                     f"D at ({qx:.0f},256,{WATER_Z:.0f}), about the pool's plane")
    m.expect_visible(ledge.side("top"), "control: ledge top, seen directly from the walkway")
    m.expect_visible(pool.side("top"), "control: the pool surface, seen directly")
    m.expect_visible(walkway.side("top"), "control: the walkway the player stands on")
    m.expect_hidden(walkway.side("east"), "walkway east face (+x): every eye, real or mirrored, has x <= -32",
                    strict=True)

    m.expect_reach((-128, 256, 0), "walkway")
    m.expect_no_reach((300, 256, -256), "pit floor: the playerclip keeps the player on the walkway")

    m.note(f"Pool {WATER_MAT} brush x 0..{POOL_X1:g} and displacement D x {POOL_X1:g}..512, both at z={WATER_Z:g}; "
           f"reflection E={eye} -> Q=({qx:.1f},256,{WATER_Z:g}) on D -> P={probe}.")

    # verify_expectations.py (#19)
    m.verify_floor(-256, 0, 0, 512, 0, "walkway (the playerclip keeps the hull off the rim)")
    m.verify_mirror(WATER_Z, 0, 0, POOL_X1, 512, "the pool (expensive water)")
    m.verify_mirror(WATER_Z, POOL_X1, 0, 512, 512, "D: shows the reflection about the pool's plane")
