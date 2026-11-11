"""
f28 -- a face seen only from the standing eye of a push flight (#42), in the default mode.

Every free arc is traced crouched and its eye curve emitted at the crouch eye.  A duck-jump's
arc starts crouch_lift up, so its crouch eye is the standing jump's eye; but an arc that
starts on the player's own feet (a walk-off, a surf arc, a push, ladder or teleport launch)
may be flown standing, with the eye eye_stand - eye_crouch = 36u higher.  Before #42 only
walk-offs got that standing curve.

f14's push chamber (x east, y north, z up; interior bounds, 16u walls):
    room       x 0..1024   y 0..512  z 0..448     spawn (256,256,1)
    ledge      x 640..1024 y 0..512  top z=160    reached only through the push
    column     x 560..640  y 192..320 z 0..224    trigger_push, pushdir "-90 0 0" (up), speed 900
    shelf      x 192..320  y 192..320 z 376..392  a floating slab; its TOP (z=392, faces +z) is the face
    clip       x 192..320  y 192..320 z 393..448  playerclip over the shelf: nobody stands on it

In the column the net acceleration is 900 - 600 = 300 u/s^2 up.  The three ways in (bw_special
push families) and their peaks after leaving the column top (feet z 224):
    standing     (vz 0)            exits at sqrt(2*300*224) = 366.6 u/s, feet apex 336.0
    jumping      (vz 160)          exits at sqrt(160^2 + 2*300*224) = 400 u/s, feet apex 357.3
    duck-jumping (vz 160, lift 36) exits at sqrt(160^2 + 2*300*188) = 372.1 u/s, feet apex 339.4
The highest crouch eye of any of them is 357.3 + 28 = 385.3 (+0.53 chord raise): below the shelf
top, so before #42 no eye sees it (node eyes reach 160 + 85 = 245, the column's own eyes 224 + 64).
The standing flight's standing eye peaks at 336 + 64 = 400 > 392: it sees the shelf top over the
column (from (600,256,400) to (320,256,392) the line drops 8u in 280u, through open air).
The shelf is 240u west of the column; the push flights that join the ledge fly east.
"""
import vmfgen

NAME = "f28_push_stand_eye"
DESCRIPTION = ("A shelf top seen only from the standing eye of a trigger_push flight, above every crouch eye "
               "of the push's flights: unlifted push arcs get the standing-eye curve (#42).")

TRIGGER = "TOOLS/TOOLSTRIGGER"
CLIP_MAT = "TOOLS/TOOLSPLAYERCLIP"
LEDGE = 160.0
PUSH_SPEED = 900.0
PUSH_TOP = 224.0
SHELF_TOP = 392.0


def build(m: vmfgen.Map) -> None:
    g, vj, lift = vmfgen.SV_GRAVITY, vmfgen.JUMP_SPEED, vmfgen.CROUCH_LIFT
    up = PUSH_SPEED - g
    v_stand = (2 * up * PUSH_TOP) ** 0.5
    v_jump = (vj ** 2 + 2 * up * PUSH_TOP) ** 0.5
    v_duck = (vj ** 2 + 2 * up * (PUSH_TOP - lift)) ** 0.5
    apex = {k: PUSH_TOP + v * v / (2 * g) for k, v in (("stand", v_stand), ("jump", v_jump), ("duck", v_duck))}
    crouch_eye_max = max(apex.values()) + vmfgen.EYE_DUCK + 0.5 + 1.0 / 32
    stand_eye = apex["stand"] + vmfgen.EYE_STAND
    assert crouch_eye_max < SHELF_TOP - 4, crouch_eye_max      # no crouch eye of a push flight sees it
    assert stand_eye > SHELF_TOP + 4, stand_eye                 # the standing flight's eye does
    assert LEDGE + vmfgen.MAX_EYE_ABOVE_FLOOR * 1.15 + 40 < SHELF_TOP
    assert LEDGE > vmfgen.CROUCH_JUMP_FEET * 1.15 + 40          # the ledge needs the push

    room = m.room((0, 0, 0), (1024, 512, 448))
    m.entity("info_player_start", origin=(256, 256, 1), angles="0 0 0")
    ledge = m.box((640, 0, 0), (1024, 512, LEDGE))
    column = m.box((560, 192, 0), (640, 320, PUSH_TOP), material=TRIGGER)
    m.brush_entity("trigger_push", [column], pushdir="-90 0 0", speed=PUSH_SPEED, spawnflags="1")
    shelf = m.box((192, 192, SHELF_TOP - 16), (320, 320, SHELF_TOP))
    m.box((192, 192, SHELF_TOP + 1), (320, 320, 448), material=CLIP_MAT, tag="playerclip")

    m.expect_visible(shelf.side("top"),
                     f"standing push flight: its eye peaks at {stand_eye:.0f} over the column, above the shelf top "
                     f"z={SHELF_TOP:g}; every crouch eye of a push flight stays below {crouch_eye_max:.1f}")
    m.expect_visible(shelf.side("bottom"), "control: the shelf underside, seen from the floor")
    m.expect_visible(ledge.side("west"), "control: ledge face, seen from the floor")
    m.expect_visible(room.inner("ceiling"), "control: ceiling")

    m.expect_reach((256, 400, 0), "floor at the spawn")
    m.expect_reach((900, 64, LEDGE), "ledge top: lifted by the upward trigger_push")

    m.note(f"Push apexes (feet): {', '.join(f'{k} {v:.1f}' for k, v in apex.items())}; shelf top z={SHELF_TOP:g}.")

    # verify_expectations.py (#19)
    m.verify_floor(0, 0, 640, 512, 0, "chamber floor")
    m.verify_floor(640, 0, 1024, 512, LEDGE, "ledge top")
    # a standing player lifted straight up the column, rising and falling over it: exact
    m.verify_eye_box((560, 192, vmfgen.EYE_DUCK), (640, 320, stand_eye - 1), "standing in and over the push column")
    # every push flight (any family, landing anywhere east), for counterexamples only
    m.verify_eye_box((560, 0, vmfgen.EYE_DUCK), (1024, 512, apex["jump"] + vmfgen.EYE_STAND + 4),
                     "push flights over the ledge", role="hidden")
