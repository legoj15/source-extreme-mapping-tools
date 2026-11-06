# bspwalk fixtures and harness

Ground truth for the native BSP walker rewrite (`docs/bsp_walker_rebuild.md`, issue #10).
Each fixture is a tiny map whose answer is known by construction: a list of brush side
ids that a reachable player **must see** and a list that **no reachable player can see**.
A walker is scored on how many of those it gets wrong.

## Run

```
python make_fixtures.py [--only f03] [--no-verify]   # writes unit_test_maps/bspwalk/<NAME>.vmf + .expect.json,
                                                    # then runs verify_expectations.py; that rebuilds each
                                                    # fixture module from its declarations and does not read the files
python verify_expectations.py [--only f11] [--mutants] [--quiet] [--jobs N]
python run_bspwalk_tests.py [--fixtures f01,f03] [--walker <exe>] [--tierb-threshold N]
                            [--timeout SEC] [--regen] [--skip-compile] [--jobs N] [--repeat N]
                            [--game <dir>] [--walker-args=--no-side-flip]
```

Everything is relative to this directory's grandparent (`game/bin/x64`). The harness, per fixture:
copies the VMF to `unit_test/unit_test_output/bspwalk/<NAME>/`, deletes stale products, runs
`vbsp_lmo.exe -emitsideids`, then `vvis_optix.exe` (CPU path), checks lumps 22/23 and that
every expected side exists in the BSP, runs the walker, scores it. Results:
`unit_test_output/bspwalk/summary.json` (always written, even when every fixture ERRORs); logs
archived to `unit_test_logs/<timestamp>_bspwalk/`. The default walker is `game/bin/x64/bspwalk.exe`; `--jobs` defaults to 4.

`--game` (and a walker executable path) is resolved to an absolute path first, because the tools
run with `game/bin/x64` as their working directory.

Walker options go through `--walker-args` with `=`, because a value that starts with `-` is
otherwise taken as a harness option: `--walker bspwalk.exe --walker-args=-cpuarcs` (CPU arc
path) or `--walker-args=-arccheck` (GPU and CPU arcs compared; a fixture with unexplained
disagreements ERRORs because the walker exits 1).

`-reachslim` (or any option that drops the reach nodes' `verts`) makes every fixture ERROR, even
one with no reach expectation, because the harness checks the nodes on every run. `-vischeck` does
not gate the walker's exit code: its disagreements are only reported, unlike `-arccheck`.

`--repeat N` compiles each fixture once and runs the walker N times on it, scoring every run, to
catch an intermittent crash. A fixture's status is the worst of its runs; the report lists each
bad run, marks a fixture whose runs ended in different statuses (say 14 PASS and 1 ERROR) as
`FLAKY`, labels one whose runs all fail the same way `every run bad` (a deterministic failure, not
a flake), and ends with a `REPEAT` line. Run 1 writes `<NAME>.walk.json` / `<NAME>.walk.out`, run K writes
`<NAME>.rK.walk.json` / `<NAME>.rK.walk.out` (all archived with the logs). Repeats of one fixture run
one after another; `--jobs` still spreads different fixtures over threads.

`--skip-compile` reuses the BSP in the work dir only if the fixture VMF still hashes to the sha1
recorded in `<NAME>.vmf.sha1` at compile time (written after vbsp and vvis both succeed). A changed
VMF, or no record, is an ERROR for that fixture ("stale BSP, recompile").

### Exit codes

| Code | Meaning |
|---|---|
| 0 | every fixture passed |
| 1 | at least one FAIL and no ERROR |
| 2 | any ERROR; or a usage error (unknown fixture name, bad arguments, missing walker or game dir), in which case nothing ran |

`make_fixtures.py` exits 0 when every claim verified (or with `--no-verify`), 1 when a claim failed
verification, 2 on a usage error (an unknown `--only` id, nothing written) or when numpy is missing
(the files are already written, unverified). `verify_expectations.py` uses the same three codes. An
unexpected exception inside one fixture's run is reported as that fixture's ERROR (text and traceback
in `summary.json`) and never stops the others.

## Scoring

A side's tier is the weakest of the BSP faces that carry its id (`visible` beats `B` beats `A`).

| Outcome | Meaning |
|---|---|
| FAIL | a `must_visible` side is tier A (a proof was claimed and it is wrong), or more than `--tierb-threshold` `must_visible` sides are tier B |
| reported | a `must_hidden` side classed visible: savings missed, never fails |
| ERROR | compile/leak/lump/walker problem (including vbsp exiting non-zero, vbsp deleting a static prop, or the walker crashing; a Windows crash code such as `0xC0000005` is printed in hex); malformed walker JSON (root not an object, `faces` not a list, a face listed twice, a missing or malformed `reach`); an `expect.json` entry without an integer `side` or a reach entry without a 3-number `point`; an expected side with no BSP face; a `must_visible` and a `must_hidden` side merged into one BSP face (the last two are fixture defects) |

Reach is scored separately from the face tiers, on `must_reach` / `must_not_reach` feet points.
A point is on a node when it is within 0.5u of the node's polygon in XY and within 2u of the
node's plane in z.

| Reach outcome | Meaning |
|---|---|
| FAIL | a `must_reach` point is on no reached node, or a `must_not_reach` point is on a reached node |
| ERROR | the walker's JSON has no `reach` object at all (bspwalk writes one on every walking run; `-nowalk`, an old exe or a regression drops it), even for a fixture with no reach expectations |

## Walker JSON contract

```
{"walker": "bspwalk", "milestone": N, "bsp": "...", "profile": {...},
 "counts": {...}, "skipped": {...}, "reach": {...}, "eyes": {...}, "vis": {...},
 "timings_s": {...},
 "faces": [{"face": 12, "sideids": [31], "tier": "A" | "B" | "visible", "reason": "pvs"}]}
```

The harness requires `walker` (a string), `faces` and `reach`; the other keys are run statistics
(`counts` and `skipped` are the collision counts, `timings_s` the stage timings; `reach`, `eyes` and
`vis` are absent on a `-nowalk` / `-novis` run). Optional extras appear only when asked for:
`verify_failures`, `checkprops_mismatches`.

`face` is a BSP face index; `sideids` must equal the BSP's lump 22/23 entry for that face
(the harness checks). Faces not listed are visible. Tier A = proven hidden (PVS); tier B =
ray-sampled hidden. `reason` is one of `pvs` (tier A), `rays` (tier B), or for a visible face
`fan` (hit by an eye fan ray), `ray` (a clear face-to-eye ray), `mirror` (a clear ray through a
reflective face), `brush_entity` (a brush entity face, not evaluated), `no_tierb` (`-notierb`
run) or `no_eye` (no ray test possible). The harness does not read `reason`.

The harness runs the walker from `game/bin/x64` with absolute paths (change it here and in
`run_walker()` together); shortened to relative paths here:

```
bspwalk.exe unit_test/unit_test_output/bspwalk/f03_crouchjump_ledges/f03_crouchjump_ledges.bsp
            -game "<game dir>" -out unit_test/unit_test_output/bspwalk/f03_crouchjump_ledges/f03_crouchjump_ledges.walk.json
            [extra --walker-args words]
```

`bspwalk.exe` also writes a `reach` object (its full shape is in `docs/bsp_walker_rebuild.md`).
The harness reads only `reach.nodes`:

```
"reach": {"nodes": [{"id": 0, "reached": true, "plane": [nx, ny, nz, d],
                     "verts": [[x, y, z], ...]}, ...], ...}
```

`plane` is `n . p = d` with `nz > 0`; `verts` is the walkable polygon (feet points).

## Fixtures

| id | file | mechanic |
|---|---|---|
| f01 | `f01_void_room` | Spawn room with two sealed void rooms (east and above) that share no opening. |
| f02 | `f02_offgrid_corridor` | 32.5u off-grid L-shaped corridor from a spawn room to a far room, plus a sealed room off the far room. |
| f03 | `f03_crouchjump_ledges` | Ledge at 48u reachable only by a duck-jump, ledge at 112u unreachable; each carries a hut whose interior is visible only from the ledge. |
| f04 | `f04_running_jump_gap` | Platform across a 96u gap (running jump) and across a 1280u gap (impossible); each carries a hut whose interior is visible only from the platform. |
| f05 | `f05_prop_crates` | Hut on an 80u ledge reachable only by climbing a static 40u block prop; control hut on a 176u ledge too high even from its block. |
| f06 | `f06_disp_hill` | Displacement hill (128u rise, slope normal z >= 0.8) with a hut on the summit whose doorway faces away from the valley; the hut is visible only from the hill. |
| f07 | `f07_closed_door` | Spawn room, a room behind a closed func_door (openable, so visible), and a sealed void room beside it. |
| f08 | `f08_teleport_room` | Rooms reachable only through trigger_teleport (one directly, one by chaining two teleports) and a sealed room whose teleport destination nobody targets. |
| f09 | `f09_water_ladder` | Room reachable only through a flooded passage (dry twin is a trap) and a ledge reachable only by a func_useableladder (ladderless twin is unreachable). |
| f10 | `f10_jump_apex_wall_top` | Up-facing faces at 72u (visible only at the apex of a jump) and 128u (never): wall tops, niche floors and the floor behind a 72u wall. |
| f11 | `f11_slit_view` | 1024x1024 hall in an unreachable room seen only through a 4u x 16u-deep slit; side walls and back faces provably hidden. |
| f12 | `f12_chainlink_fence` | Chain-link prop_static fence spanning a room; an alcove and a hut behind it are visible through the fence, the hut interior is not. |
| f13 | `f13_skybox_facade` | 3D skybox (sky_camera scale 16) in a sealed room holding a building facade; faces toward the mapped eyes visible, backs and a decoy hut hidden. |
| f14 | `f14_train_and_push` | Ledge reachable only by riding a func_tracktrain, ledge reachable only via a trigger_push, and an equally high ledge with neither; each carries a hut. |
| f15 | `f15_physics_crate` | Hut on an 80u balcony reachable only by climbing a prop_physics crate at its spawn origin; control hut on a 192u balcony that is out of reach. |
| f16 | `f16_reflection_cubemap` | A ledge underside seen only in expensive-water reflection, and a sealed room seen only from its env_cubemap; a decoy hut and a plain void room stay hidden. |
| f17 | `f17_reflection_pvs` | A recess no eye cluster's PVS holds, seen only in a water reflection through a window tube: tier A must add the visible water's clusters to the eye clusters (without that rule bspwalk claims the recess wall tier A and FAILs). |
| f18 | `f18_surf_ramp` | A platform reached only by surfing up a steep ramp and off its top edge (#24), and a control platform above every surf flight; each carries a hut whose interior faces away from the ramp. |

## Verifying the expectations (#19)

`verify_expectations.py` checks every claim by brute force, independently of any walker: it
builds the fixture's `vmfgen.Map`, takes the eye regions the fixture declares, and tests
segments from a lattice of eyes to a grid of points on each expected side against the map's
own solids (playerclip, water, trigger brushes, displaced solids and props do not occlude). A
must_visible claim needs one clear front-facing segment from an exact eye (directly, through a
declared mirror, or from a viewpoint); a must_hidden claim needs none from any eye. Lattice eyes
whose crouched hull would overlap a player-solid solid are dropped, so a declaration can be a
whole floor. `--mutants` runs wrong variants (an eye inside f01's void room or f11's hall, f16
without its cubemap or mirror, f13 without its sky mapping) that must each fail. A fixture that
declares no eyes is reported and skipped. It needs numpy; fixtures run in parallel (`--jobs`).

Declarations (never written to the VMF):

| Call | Meaning |
|---|---|
| `m.verify_floor(x0, y0, x1, y1, z, why, role="exact")` | a reachable floor rectangle: eyes 28 to 85.33 above it |
| `m.verify_eye_box(mins, maxs, why, role="exact")` | a box of eye points (swimming, climbing, falling, riding) |
| `m.verify_viewpoint(p, why)` | a camera the map renders from (env_cubemap) |
| `m.verify_sky(origin, scale)` | 3D skybox: each eye E also at origin + E / scale |
| `m.verify_mirror(z, x0, y0, x1, y1, why)` | a horizontal reflective surface facing up |
| `m.verify_skip_solid(solid)` | a world solid that neither occludes nor blocks |

`role="exact"` regions hold only real eye positions and are used for both kinds of claim;
`role="hidden"` regions may be supersets (a train ride's sweep, a push column) and are used only
to look for counterexamples to must_hidden.

## Add a fixture

Create `fixtures/fNN_slug.py` (the stem must match `NAME`). Nothing else registers it.

```python
import vmfgen
NAME = "f05_crate_stairs"
DESCRIPTION = "one line for expect.json"

def build(m: vmfgen.Map) -> None:
    room = m.room((0, 0, 0), (512, 512, 192))              # INTERIOR bounds; shell built outside
    m.entity("info_player_start", origin=room.floor_center())   # one spawn is required
    m.expect_visible(room.all_inner(), "player stands in it")
    m.expect_hidden(other_room.all_inner(), "sealed")
```

`vmfgen` quick reference (full docstrings in `vmfgen.py`):

| Call | Returns |
|---|---|
| `Map(name, description, skyname)` | built for you by `make_fixtures` |
| `m.box(mins, maxs, material, tag, face_materials)` | `Solid`; `.side('top'/'bottom'/'north'/'south'/'east'/'west')` -> side id |
| `m.room(mins, maxs, wall=16, openings=[(face, rect)], omit=(), floor=True, ceiling=True, marker=False)` | `Room`; `.inner(face)`, `.inner_all(face)`, `.all_inner()`, `.outer(face)`, `.floor_center()` |
| `m.convex(vertices)`, `m.brush_from_planes(planes)`, `m.wedge(mins, maxs, high='east')` | `Solid` (ramps/wedges; slanted side named `slope` on a wedge) |
| `m.displace(solid, 'top', power=3, heights=f(u, v))` | the displacement `Side` (u along +x, v along +y on a top side) |
| `m.entity(classname, origin, **keys)`, `m.brush_entity(classname, solids, **keys)` | `Entity`; brush-entity solids leave the world |
| `m.marker(origin)` | an `info_target` that keeps a sealed region from being filled |
| `m.expect_visible(side_or_ids, why)`, `m.expect_hidden(...)` | accept an id, a `Side`, or any iterable |
| `m.expect_reach((x, y, z), why)`, `m.expect_no_reach(...)` | a feet point the walker must (must not) reach; written as `must_reach` / `must_not_reach` |
| `m.verify_floor(...)`, `m.verify_eye_box(...)`, ... | eye regions for `verify_expectations.py` (above); declare them for every new fixture |

Room opening rectangles are in world coordinates of the two in-plane axes: `east/west` is
`(ymin, ymax, zmin, zmax)`, `north/south` is `(xmin, xmax, zmin, zmax)`, `floor/ceiling` is
`(xmin, xmax, ymin, ymax)`. Coordinates must be multiples of 1/64 (7.25 is fine).

### Traps (each is real; the harness reports them as ERROR)

- **A sealed region with no entity is filled solid by VBSP and its faces vanish.** Use
  `marker=True` (verified: a bare sealed room produced 0 faces). Entities at origin `0 0 0` are
  ignored by the flood, so `m.entity` refuses that origin. `env_cubemap` does NOT count as an
  entity here: VBSP converts it to cubemap data before the flood, so a room whose only entity is
  an `env_cubemap` is still filled. Add a marker.
- **A model without `allowstatic` cannot be a `prop_static`.** `models/props_junk/wood_crate001a.mdl`
  and others whose `prop_data` block lacks `allowstatic 1` are deleted by VBSP with "must be used on
  a dynamic entity ... Deleted" while the compile still succeeds. The harness scans the VBSP output
  for that and ERRORs the fixture. Use a static-legal model (f05 uses the stove) or a `prop_physics`
  (f15 uses the crate).
- **Coplanar same-material neighbours merge into one BSP face.** Never let a must-visible and a
  must-hidden side touch on one plane with one material.
- **Leaks:** every fixture must be sealed. A leaked map writes no BSP.
- Reach/margin numbers: use `vmfgen.CROUCH_JUMP_FEET` (57), `STAND_JUMP_FEET` (21), `MAX_EYE_ABOVE_FLOOR`
  (~85), `jump_gap_reach(speed)`, and assert your margins in `build()` so they are checked on every run.
- Prefer the **hut trick** for certainty (f03, f04): a closed box with one doorway in the wall that
  faces away from where the forbidden eyes are. Every segment from those eyes into the box crosses
  its shell, so the interior is provably hidden, with no grazing-angle sight lines to argue about.
  A face that only faces upward (a roof top) is provably hidden from any eye below its plane.
