"""
f16 -- faces that only a second camera sees: a water reflection, and an env_cubemap.

Two independent mechanics in one map.

(a) Reflective water.  DEV/DEV_WATER3_EXP (checked in hl2_textures/misc VPK): shader
    "Water", "$reflecttexture" "_rt_WaterReflection", "$forceexpensive" 1,
    "%compilewater" 1.  The renderer draws a reflection from the eye mirrored about the
    water plane: E_r = (x, y, 2*Wz - z).

    Geometry (x east, y north, z up; interior bounds, 16u walls):
        P   pit room   x -256..512  y 0..512  z -256..320   floor top at z=-256
        walkway block  x -256..0    y 0..512  z -256..0     the player's floor is its top, z=0
        playerclip     x -16..0     y 0..512  z 0..320      keeps the player off the rim
        water          x 0..512     y 0..512  z -256..-192  surface at Wz=-192
        ledge          x 384..512   y 0..512  z -112..-96   fixed to the far wall; its
                                                            underside z=-112 faces down
    info_player_start at (-128,256,1).  Reachable eyes: x <= -32 (hull 16 off the clip),
    y 16..496, z 28..85 over z=0.  The pit is unreachable: the clip runs from the walkway
    to the ceiling, and nothing else leads down.

    Why the underside (z=-112, faces -z) is seen by no direct eye: it is visible only from
    points below z=-112; every eye is at z >= 28.
    Why it is seen in the reflection: take E=(-32,256,64).  Then E_r=(-32,256,-448).  The
    line E_r -> P=(450,256,-112) has direction (482,0,336) and crosses z=-192 at
    t=256/336=0.762, x=335.2: Q=(335.2,256,-192), inside the water (x 0..512, y 0..512).
    E -> Q has slope -0.697 and is at z=41.7 over the rim (x=0), clear of the walkway block
    (the playerclip does not block sight).  Q -> P climbs from (335.2,-192) to (450,-112)
    and is at z=-158 where it passes x=384, below the ledge underside, so it is clear.
    The harness name for this is "second camera".

    VVIS facts measured on the compiled BSP: the ledge underside's leaf is already in the
    PVS of the walkway's cluster (the pit is one open space), so tier A never hides it
    and this part exercises the tier B mirrored-eye set only.  It cannot show whether a
    walker adds the water cluster's PVS to tier A (that needs the reflected face's cluster
    to be PVS-invisible to the player, e.g. a niche off the pool behind a viewing window;
    not built).

    The ledge top (z=-96) is seen directly (control), as is the water surface itself, and
    the pit floor through the water (refraction; water does not occlude).

(b) env_cubemap.  A sealed room C that no player can enter; its env_cubemap at
    (768,128,96) is captured from there, independently of the player.
        C  cubemap room  x 704..960  y 0..256  z 0..192   env_cubemap at (768,128,96)
        hut in C         interior x 864..928 y 80..176 z 0..96, outer x 848..944 y 64..192,
                         roof top z=112, ONE doorway in its east wall (y 112..144, z 0..80)
        V  void room     x 1024..1280 y 0..256 z 0..192   sealed, info_target only
    VBSP does NOT count env_cubemap as an occupant: it turns the entity into a cubemap
    sample (lump 42) before the flood fill, so a sealed room whose only entity is the
    cubemap is filled solid (verified: all 20 C sides had no BSP face).  C therefore also
    holds an info_target marker, as any sealed room does.  The walker has to read cubemap
    origins from lump 42, not from the entity lump.

    must_visible from the cubemap origin O=(768,128,96): all six inner sides of C (the east
    wall above the hut: (768,128,96)->(960,128,170) is at z=127 where it passes the hut's
    west face x=848, above the roof top z=112), and the hut's west face (x=848, faces -x).
    must_hidden:
        hut interior: the aperture is in the east slab x 928..944; O and every interior point
                      have x < 928, so no segment reaches it; the rest of the shell is closed
        hut north face (y=192, faces +y), south face (y=64, faces -y): O has y=128, behind both
        hut east outer faces (x=944, faces +x): O has x=768
        hut roof top (z=112, faces +z): O has z=96
        V inner sides: a sealed room with no entity of the kind a walker adds as a viewpoint

    VVIS facts measured on the compiled BSP: from the walkway's cluster alone the PVS hides
    all of C and its hut, so a tier A that ignores env_cubemap calls the 7 must_visible C
    sides hidden (FAIL).  Adding the cubemap's cluster makes them PVS-visible again; the hut
    interior (7 sides) and V (6 sides) stay PVS-hidden, the hut's north, south, east
    (3 pieces) and roof top (6 sides) are PVS-visible and only the ray tier hides them.
"""
import vmfgen

NAME = "f16_reflection_cubemap"
DESCRIPTION = ("A ledge underside seen only in expensive-water reflection, and a sealed room seen "
               "only from its env_cubemap; a decoy hut and a plain void room stay hidden.")

WATER_MAT = "DEV/DEV_WATER3_EXP"
CLIP_MAT = "TOOLS/TOOLSPLAYERCLIP"
WATER_Z = -192.0
LEDGE_BOTTOM, LEDGE_TOP = -112.0, -96.0
CUBEMAP_ORIGIN = (768.0, 128.0, 96.0)


def build(m: vmfgen.Map) -> None:
    # (a) -- numbers behind the reflection proof.
    eye = (-32.0, 256.0, 64.0)
    probe = (450.0, 256.0, LEDGE_BOTTOM)
    mirror_z = 2 * WATER_Z - eye[2]
    t = (WATER_Z - mirror_z) / (probe[2] - mirror_z)
    qx = eye[0] + t * (probe[0] - eye[0])
    assert 0 < qx < 384 and 0 < probe[1] < 512                          # Q is on open water, left of the ledge
    z_at_rim = eye[2] + (WATER_Z - eye[2]) * (0 - eye[0]) / (qx - eye[0])
    assert z_at_rim > 20                                                # clear over the walkway edge
    assert LEDGE_BOTTOM < vmfgen.EYE_DUCK - 1                           # no reachable eye is below the underside
    assert LEDGE_BOTTOM < 0 - 50

    pit = m.room((-256, 0, -256), (512, 512, 320))
    m.entity("info_player_start", origin=(-128, 256, 1), angles="0 0 0")
    walkway = m.box((-256, 0, -256), (0, 512, 0))
    m.box((-16, 0, 0), (0, 512, 320), material=CLIP_MAT, tag="playerclip")
    water = m.box((0, 0, -256), (512, 512, WATER_Z), material=WATER_MAT, tag="water")
    ledge = m.box((384, 0, LEDGE_BOTTOM), (512, 512, LEDGE_TOP))

    # (b)
    cube = m.room((704, 0, 0), (960, 256, 192), marker=True)
    m.entity("env_cubemap", origin=CUBEMAP_ORIGIN, cubemapsize=0)
    hut = m.room((864, 80, 0), (928, 176, 96), floor=False,
                 openings=[("east", (112, 144, 0, 80))])
    void = m.room((1024, 0, 0), (1280, 256, 192), marker=True)

    m.expect_visible(ledge.side("bottom"),
                     "second camera: the underside faces down, every eye is above it; "
                     "the mirrored eye (-32,256,-448) sees it through the water at (335,256,-192)")
    m.expect_visible(ledge.side("top"), "control: ledge top, seen directly from the walkway")
    m.expect_visible(water.side("top"), "control: the reflective water surface itself, seen directly")
    m.expect_visible(pit.inner("floor"), "control: pit floor, seen through the water")
    m.expect_visible(walkway.side("top"), "control: the walkway the player stands on")

    m.expect_visible(cube.all_inner(), "cubemap viewpoint: inner side of a sealed room with an env_cubemap")
    m.expect_visible(hut.outer("west"), "cubemap viewpoint: hut west face toward the cubemap origin")

    m.expect_hidden(hut.all_inner(),
                    "hut interior: doorway faces east, away from the cubemap origin")
    m.expect_hidden(hut.outer("north"), "hut north face (+y): the cubemap origin is at y=128, behind it")
    m.expect_hidden(hut.outer("south"), "hut south face (-y): the cubemap origin is at y=128, behind it")
    m.expect_hidden(hut.outer_all("east"), "hut east face (+x): the cubemap origin is at x=768, behind it")
    m.expect_hidden(hut.outer("ceiling"), "hut roof top z=112: the cubemap origin is at z=96, below it")
    m.expect_hidden(void.all_inner(), "sealed void room: no opening, no viewpoint inside")
    m.expect_cubemap(CUBEMAP_ORIGIN, "ok", why="open space in a valid sealed room, no nodraw anywhere: stays a viewpoint")

    m.note(f"Water {WATER_MAT} at z={WATER_Z}; reflection proof E=({eye[0]:.0f},{eye[1]:.0f},{eye[2]:.0f}) "
           f"-> Q=({qx:.1f},256,{WATER_Z:.0f}) -> P={probe}. env_cubemap at {CUBEMAP_ORIGIN}.")

    # verify_expectations.py (#19)
    m.verify_floor(-256, 0, 0, 512, 0, "walkway (the playerclip keeps the hull off the rim)")
    m.verify_viewpoint(CUBEMAP_ORIGIN, "env_cubemap")
    m.verify_mirror(WATER_Z, 0, 0, 512, 512, "expensive water")
