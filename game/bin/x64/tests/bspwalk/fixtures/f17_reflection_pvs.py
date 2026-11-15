"""
f17 -- a recess the PVS hides from every eye, seen only in a water reflection (tier-A-sharp).

f16's reflected ledge underside is already in the walkway cluster's PVS, so it tests only the
ray tier's mirrored eyes.  Here the reflected face is in the PVS of no eye cluster: only the
rule that a visible reflective face joins the eye clusters (design doc, Stage 4, second
cameras) keeps tier A from claiming it hidden.

Geometry (x east, y north, z up; interior bounds, 16u walls):
    U  upper room   x 0..512    y 0..512  z 176..368   info_player_start (128,128,177); its floor
                                                      is C's ceiling slab (z 160..176)
    C  chamber      x 0..1024   y 0..512  z -64..160   flooded to z 0 (DEV/DEV_WATER3_EXP)
       filler       x 0..512    z 112..160 inside C: under U the chamber ceiling is at z 112
    window          x 192..352  y 192..320 through C's ceiling slab and the filler, z 112..176,
                    filled with playerclip (sight passes, players do not)
    R  recess       x 608..736  y 192..320 z 176..304, open downward through C's ceiling slab
                    (x 608..736, y 192..320, z 160..176)

Why no eye sees into R directly, and why the PVS agrees: every eye is in U (z >= 204).  A
straight line from U into C passes the window tube top to bottom, so it is descending and
below z 112 once it leaves the tube; R's only opening is at z >= 160 > 112.  So no line from
U reaches R, and VVIS (exact portal visibility) leaves R out of U's PVS.

Why R's east wall (x 736, faces -x) is seen in the reflection: E = (200, 256, 256) mirrored in
the water plane z 0 is E_r = (200, 256, -256).  The line E_r -> P = (736, 256, 272) has slope
dx/dz = 536/528 = 1.015: its descending (real) half crosses z 176 at x 281 and z 112 at x 346
(inside the window, x 192..352), the water at Q = (460, 256, 0) (open water, x 0..1024), z 160
at x 622 and z 176 at x 638 (inside R's opening, x 608..736), and reaches R's east wall at
z 272 (R spans z 176..304).  E -> Q is
the descending half, Q -> P the rising one; both stay in free space.
must_hidden: R's west wall (x 608, faces +x): every real or mirrored eye has x <= 496 < 608.
"""
import vmfgen

NAME = "f17_reflection_pvs"
DESCRIPTION = ("A recess no eye cluster's PVS holds, seen only in a water reflection: tier A must add the "
               "visible water's clusters to the eye clusters.")

WATER_MAT = "DEV/DEV_WATER3_EXP"
CLIP_MAT = "TOOLS/TOOLSPLAYERCLIP"
WATER_Z = 0.0
WIN = (192.0, 352.0, 192.0, 320.0)     # window x0, x1, y0, y1
REC = (608.0, 736.0, 192.0, 320.0)     # recess x0, x1, y0, y1


def build(m: vmfgen.Map) -> None:
    # the reflection line of the docstring, checked against the openings
    E, P = (200.0, 256.0, 256.0), (736.0, 256.0, 272.0)
    Er = (E[0], E[1], 2 * WATER_Z - E[2])
    def x_at(z):
        return Er[0] + (P[0] - Er[0]) * (z - Er[2]) / (P[2] - Er[2])
    def x_mirror(z):                    # the real (descending) half
        return x_at(2 * WATER_Z - z)
    assert WIN[0] < x_mirror(176) < WIN[1] and WIN[0] < x_mirror(112) < WIN[1] - 4
    assert REC[0] < x_at(160) < REC[1] and REC[0] < x_at(176) < REC[1]
    assert 176 < P[2] < 304 and 0 < x_at(WATER_Z) < 1024
    assert E[2] <= 176 + vmfgen.EYE_STAND + 22 and 16 <= E[0] <= 496

    chamber = m.room((0, 0, -64), (1024, 512, 160),
                     openings=[("ceiling", (WIN[0], WIN[1], WIN[2], WIN[3])),
                               ("ceiling", (REC[0], REC[1], REC[2], REC[3]))])
    upper = m.room((0, 0, 176), (512, 512, 368), floor=False)
    m.entity("info_player_start", origin=(128, 128, 177), angles="0 0 0")
    # filler under U (the chamber ceiling is at z 112 there), with the window hole
    x0, x1, y0, y1 = WIN
    for lo, hi in (((0, 0, 112), (x0, 512, 160)), ((x1, 0, 112), (512, 512, 160)),
                   ((x0, 0, 112), (x1, y0, 160)), ((x0, y1, 112), (x1, 512, 160))):
        m.box(lo, hi)
    m.box((x0, y0, 112), (x1, y1, 176), material=CLIP_MAT, tag="playerclip")
    water = m.box((0, 0, -64), (1024, 512, WATER_Z), material=WATER_MAT, tag="water")
    recess = m.room((REC[0], REC[2], 176), (REC[1], REC[3], 304), floor=False)

    m.expect_visible(recess.inner("east"),
                     "second camera, PVS-sharp: no eye cluster's PVS holds the recess; the mirrored eye "
                     "(200,256,-256) sees its east wall through the window and the water at (460,256,0)")
    m.expect_visible(water.side("top"), "control: the water surface, seen through the window")
    m.expect_visible(upper.inner("ceiling"), "control: ceiling of the room the player is in")
    m.expect_hidden(recess.inner("west"), "recess west wall (+x): every real and mirrored eye has x <= 496 < 608")

    m.expect_reach((256, 400, 176), "upper room floor")
    m.expect_no_reach((800, 400, 0), "chamber: the window is playerclip")

    # verify_expectations.py (#19)
    m.verify_floor(0, 0, 512, 512, 176, "upper room floor (over the clip window too)")
    m.verify_mirror(WATER_Z, 0, 0, 1024, 512, "expensive water")
    m.note(f"Reflection E={E} -> Q=({x_at(WATER_Z):.0f},256,0) -> P={P}; window x {WIN[0]:g}..{WIN[1]:g} "
           f"(z 112..176), recess opening x {REC[0]:g}..{REC[1]:g} (z 160..176).")
