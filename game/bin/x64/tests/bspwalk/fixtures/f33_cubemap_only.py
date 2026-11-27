"""
f33 -- faces only an env_cubemap sees get their own class, cubemap_only (#149).

A kept env_cubemap viewpoint is an eye like any other for hiding: a face it sees is not hidden. Its
view is never shown to the player, though (it is a reflection's capture), so a face only cubemaps
see is classed cubemap_only, not visible: lmoptimizer may treat it as a low-priority group. A
point_camera's or point_viewcontrol's view is shown to the player (a monitor, a cutscene): a face
only it sees stays visible.

Geometry (x east, y north, z up; interior bounds, 16u walls):
    P  player room    x 0..768     y 0..512  z 0..256   info_player_start (96,256,1)
                      env_cubemap (384,256,128) in the open: every inner side of P is seen by the
                      player and by that cubemap -> visible (a player eye decides, whatever else
                      sees it, whichever eye the fan's cell happened to take)
    C  cubemap room   x 896..1152  y 0..256  z 0..192   sealed, marker; env_cubemap (1024,128,96)
                      every inner side is seen from the cubemap and by no player -> cubemap_only
                      block in C  x 1072..1120  y 100..156  z 0..64: its west and top sides face the
                      cubemap -> cubemap_only; its east, north and south sides face away (the cubemap
                      is at x 1024 < 1120, y 100 < 128 < 156) -> hidden
    K  camera room    x 1312..1568 y 0..256  z 0..192   sealed, marker; point_camera (1408,128,96)
                      every inner side is seen only from the point_camera -> visible
    V  void room      x 1728..1984 y 0..256  z 0..192   sealed, marker, no viewpoint -> hidden (strict)

Every room is sealed from the others, so on the compiled BSP each one's PVS holds only itself: C is
in the eye clusters only through its cubemap, K only through its camera, and V not at all (tier A).
VBSP turns env_cubemap into a lump 42 sample before the flood fill, so C needs its marker as f16's
room does. Origins are whole numbers (VBSP truncates lump 42 origins).

Without the class every C side is "visible" and the fixture FAILs on must_cubemap_only.
"""
import vmfgen

NAME = "f33_cubemap_only"
DESCRIPTION = ("A sealed room seen only from its env_cubemap is cubemap_only; the player room, seen by the "
               "player and a cubemap, and a sealed room seen only from a point_camera stay visible.")

CUBEMAP_P = (384.0, 256.0, 128.0)
CUBEMAP_C = (1024.0, 128.0, 96.0)
CAMERA_K = (1408.0, 128.0, 96.0)


def build(m: vmfgen.Map) -> None:
    room = m.room((0, 0, 0), (768, 512, 256))
    m.entity("info_player_start", origin=(96, 256, 1), angles="0 0 0")
    m.entity("env_cubemap", origin=CUBEMAP_P, cubemapsize=0)

    cube = m.room((896, 0, 0), (1152, 256, 192), marker=True)
    block = m.box((1072, 100, 0), (1120, 156, 64))
    m.entity("env_cubemap", origin=CUBEMAP_C, cubemapsize=0)

    cam = m.room((1312, 0, 0), (1568, 256, 192), marker=True)
    m.entity("point_camera", origin=CAMERA_K, angles="0 0 0", FOV=90)

    void = m.room((1728, 0, 0), (1984, 256, 192), marker=True)

    m.expect_visible(room.all_inner(), "player room: the player stands in it (its cubemap sees it too)")
    m.expect_cubemap_only(cube.all_inner(), "room C: sealed, seen only from its env_cubemap")
    for side in ("west", "top"):
        m.expect_cubemap_only(block.side(side), f"block in C, {side} side: faces the cubemap, no player sees it")
    for side in ("east", "north", "south"):
        m.expect_hidden(block.side(side), f"block in C, {side} side: faces away from the cubemap at (1024,128,96)")
    m.expect_visible(cam.all_inner(), "room K: sealed, seen only from its point_camera (shown to the player)")
    m.expect_hidden(void.all_inner(), "room V: sealed, no viewpoint: the PVS hides it", strict=True)

    m.expect_cubemap(CUBEMAP_P, "ok", why="open space in the player room: a viewpoint")
    m.expect_cubemap(CUBEMAP_C, "ok", why="open space in sealed room C: a viewpoint")

    m.note(f"env_cubemap {CUBEMAP_P} in the player room, {CUBEMAP_C} in sealed room C; point_camera {CAMERA_K} "
           f"in sealed room K; void room V.")

    # verify_expectations.py (#19)
    m.verify_floor(0, 0, 768, 512, 0, "player room floor")
    m.verify_viewpoint(CUBEMAP_P, "env_cubemap in the player room", kind="cubemap")
    m.verify_viewpoint(CUBEMAP_C, "env_cubemap in room C", kind="cubemap")
    m.verify_viewpoint(CAMERA_K, "point_camera in room K", kind="camera")
