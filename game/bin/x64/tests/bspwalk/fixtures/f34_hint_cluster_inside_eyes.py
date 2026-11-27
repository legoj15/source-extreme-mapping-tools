"""
f34 -- visleaves that lie wholly inside the eye volume (#142).

Tier B's eye samples are a shell: of the lattice points inside the eye volume
only those with a neighbour column that does not reach their height are kept
(beside each column's two ends and every piece's vertices); the interior ones
are dropped.  A cluster whose every lattice point is interior would then hold
no sample at all, so the tile generator's cluster rule gives it one back: the
first dropped point in it, in sample order.  No other fixture (and no
production test map) has such a cluster, so nothing exercised that rule or the
candidates it reads along a lattice line.

Geometry (interior bounds, 16u walls):
    spawn room   x 0..512    y 0..512    z 0..192   info_player_start in the middle
    mid box      x 196..316  y 196..316  z 44..76   all six faces TOOLS/TOOLSHINT
    low box      x 68..124   y 68..124   z 15..27   all six faces TOOLS/TOOLSHINT

The eye volume over the floor is two stacked prisms per walk node, the dead
view's (z 14..28) and the living player's (z 28..85, crouched eye to the top of
a jump), 16u off the walls, so its columns run z 14..85 and its vertices sit at
z 14, 28 and 85.  Both boxes float clear of the walls and their planes sit 4u
off the 8u lattice lines (and 1u off the heights 14 and 28), so the leaf inside
each is a cluster the shell leaves empty at every refinement level:
  mid box  a column's run of dropped heights enters the leaf on its way up
           (16, 24, 32, 40 below it at 8u): the rule's candidate there is the
           first height past a leaf change, found by a gallop and a bisection.
  low box  the run starts in the leaf (its first height, 16, is the first
           lattice height above the column's lower end at 14): the candidate
           is the run's own first height.
Every other leaf the hint planes cut the room into runs out to a wall, the
floor or the ceiling and so holds shell samples.

What it protects: `run_bspwalk_cache_tests.py` runs `-eyeverify` here, which
compares each tile's samples with the brute-force set (every dropped height a
candidate).  A cluster rule that picks another height of a run than the first
in a leaf differs on this map.

Why each expectation is certain:
  visible  all six inner sides of the spawn room -- the player spawns inside it;
           a hint brush neither blocks sight nor movement.
"""
import vmfgen

NAME = "f34_hint_cluster_inside_eyes"
DESCRIPTION = "Spawn room with two hint boxes floating inside the eye volume: clusters the sample shell leaves empty."

HINT = "TOOLS/TOOLSHINT"
EYE_DEAD = 14.0                                   # bw_profile.cpp eyeDead: the column's lower end
MID_LO, MID_HI = (196, 196, 44), (316, 316, 76)
LOW_LO, LOW_HI = (68, 68, 15), (124, 124, 27)


def build(m: vmfgen.Map) -> None:
    eye_top = vmfgen.EYE_STAND + 21                                  # the top of a jump
    assert MID_LO[2] - vmfgen.EYE_DUCK >= 16 and eye_top - MID_HI[2] >= 8, eye_top
    assert EYE_DEAD < LOW_LO[2] < 16 and 24 < LOW_HI[2] < vmfgen.EYE_DUCK   # holds 16 and 24, not 14 or 28
    assert all(c % 8 == 4 for lo_hi in (MID_LO, MID_HI, LOW_LO[:2], LOW_HI[:2]) for c in lo_hi)

    spawn = m.room((0, 0, 0), (512, 512, 192))
    m.entity("info_player_start", origin=spawn.floor_center(), angles="0 0 0")

    for lo, hi in ((MID_LO, MID_HI), (LOW_LO, LOW_HI)):
        m.verify_skip_solid(m.box(lo, hi, material=HINT))

    m.expect_visible(spawn.all_inner(), "inner side of the room the player spawns in")
    m.expect_reach(spawn.floor_center(0), "spawn room floor: the player spawns on it")

    # verify_expectations.py (#19): the spawn room floor is the only place to stand
    m.verify_floor(0, 0, 512, 512, 0, "spawn room floor")
