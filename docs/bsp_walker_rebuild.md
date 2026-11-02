# BSP walker rebuild (`bspwalk`)

Status: design, not started. Replaces the Python walker (`game/bin/x64/{reachability,visibility,collision,phy_reader}.py`).
Goal: from a compiled BSP, find every face no player can ever see, and do it in minutes on a worst-case map.

## Why a rebuild, not a patch

Measured 2026-11-02 on `unit_test_maps/fightspace3_unittest.bsp` (full ±16384 grid, multi-floor buildings, teleports): the Python reachability flood at 16u did not finish in 15 minutes; the visibility pass never started.

Verified defects in the Python walker (the ones that cause false never-visible, the must-not-happen error):

| Defect | Where | Effect |
|---|---|---|
| PHY vertices read 4 bytes late; one IVP axis is always 0 | `phy_reader.py` vertex_base | Every static prop is a zero-thickness sheet; can't stand on crates/desks. Correct = per-ledge `c_point_offset`, axes `(x, z, -y)` (vphysics ConvertPositionToHL). The "Correct Swizzle" comment in `collision.py:325` is wrong; it only looked right because of the misread. Evidence: parsed PHY bounding boxes vs `.mdl` header hull (offsets 104/116) for fence01a, oildrum001, controlroom_desk001b, lamppost03a_off, furniturecouch001a from `hl2_misc_dir.vpk`. The current code gives one axis exactly 0; vertex_base−4 with `(x, z, −y)` matches all five. |
| Displacements absent from collision | `collision.py` | Displacement terrain is not walkable. |
| Bevel planes skipped in AABB-vs-brush | `_hull_intersects_brush` | Reports solid in open space near angled brushes; flood stops early. |
| Grid clamped to entity-origin bbox ±512u | `_compute_play_bounds` | Areas far from entities never explored. |
| Eyes snapped to cell bottom; XY to cell centre | `grid_to_world` | Eye height wrong by 0–16u; gaps narrower than grid slack missed. |
| No mid-jump eyes; jump reach 3 cells | `run()` jump branch | Faces visible only while jumping, and platforms across >48u gaps, missed. |
| Teleports, water, `func_ladder`, movers not modelled; `func_useableladder` only seeded as discrete cells (no climb volume; the clearance check drops cells near walls) | `run()`, `reachability.py:391-444` | Whole regions unreached. |
| Doors / `func_brush` permanently solid and opaque | `_entity_headnodes` | Rooms behind doors under-flooded and occluded. |
| PVS filter uses only the centroid's cluster | `_worker_check_face` | Faces spanning clusters lose eyes. |
| ~13 samples per face regardless of area; eye list truncated at 2048 | `_face_sample_points`, `MAX_EYES_PER_FACE` | Large partly-hidden faces missed. |
| Prop collision used as a sight occluder | `trace_ray_full` | Chain-link fences hide what's behind them. |

Speed is architectural: per-ray recursive BSP walk in Python, prop ray tests scanning every 64u cell in the ray's AABB, PVS RLE decoded per eye-face pair, BSP re-parsed per worker.

## Shape of the new tool

A native console tool `bspwalk.exe` (C++ / CUDA / OptiX), source in `src/utils/bspwalk/`, binary in `game/bin/x64/`. Same build stack and CUDA pin as `vvis_optix`: CUDA v13.2 (13.1's cudafe++ crashes under MSVC 14.51), and clear `NoDefaultCurrentDirectoryInExePath` before MSBuild or the CUDA custom build steps fail.

- **Input:** compiled BSP after VBSP (`-emitsideids`) and VVIS, plus `-game` for model/collision loading.
- **Output:** one JSON per BSP: per face `{face, sideids, tier, reason}` plus run stats. Optional debug outputs: OBJ of C-space, walkable polygons, and eye volume, for viewing in Blender.
- **Python keeps** orchestration and VMF editing (`lmoptimizer.py`, `vis_simulator.py`), which shell out to `bspwalk.exe` and read the JSON.

Reuse, don't re-derive:
- BSP loading: `src/utils/common/bsplib.cpp/.h` (`LoadBSPFile`, dbrushes/brushsides/planes/leafs/leaffaces/dispinfo/entities). Static props come from the game lump, read the way `CVradStaticPropMgr::UnserializeStaticProps()` does (`vradstaticprops.cpp:285`).
- Prop collision: vphysics via the same path VRAD uses (`vradstaticprops.cpp:1177` `VCollideLoad`, convex access). No hand-parsed PHY. This covers `prop_static` (game lump) **and** entity-lump models: `prop_physics*`, `prop_dynamic*`, `func_pushable`, at their spawn origin/angles. Entity props count as solid for standing on (over-reach, safe) and never as sight occluders (they move or break).
- Displacement collision triangles: VBSP/VRAD displacement builders (`disp_ivp.cpp` shows the triangle source).
- Polygon clipping: the algorithms of `src/utils/common/polylib.cpp` (`ChopWinding`/`ClipWindingEpsilon`). polylib itself is single precision (`vec_t` = float), so bspwalk carries a double-precision copy of the few winding functions it needs. In float, a 0.01u sliver near ±16384 is only ~5 ULP wide.
- OptiX: a small dedicated module modelled on `vvis_optix.cpp/.cu` (~600 lines). Not `RayTraceOptiX`, which is a VRAD-coupled static singleton.

## Stage 1: collision world in configuration space

The player is an axis-aligned box that never rotates, so collision reduces exactly to a point against obstacles grown by the box (Minkowski sum). For a feet-origin point the box is `[-16,16]×[-16,16]×[0,72]` standing and `[0,36]` crouched (from the game profile, below).

- Every solid convex (world brush, solid brush entity per policy, prop convex, displacement triangle) is grown into a convex polytope. Its planes are: the source face planes pushed out by the box support, the 6 axial planes, and the edge×axis bevels. This is exact, which is what the bevel-skipping Python test was not.
- Contents filtering uses the player mask (SOLID, PLAYERCLIP, MOVEABLE, WINDOW, GRATE), with no ad-hoc class list.
- Every grown polytope is at least 32u wide horizontally, so a thin wall can never be tunnelled through. Gaps between obstacles, slivers included, stay as exact free space.

Debug output: grown solids as OBJ.

## Stage 2: walkable surfaces (exact, no grid)

The feet can rest on the upward faces (normal z ≥ 0.7) of grown polytopes. For each such face:
1. Clip away the parts inside any other grown polytope of the same posture. The spatial index is a uniform hash on polytope AABBs.
2. What remains is a set of convex polygons where the box fits standing (repeat crouched for crouch-only floor).
3. Split polygons larger than 64u so graph edges stay local.

A 32.01u corridor gives a 0.01u-wide polygon, so slivers are kept exactly. Any epsilon must be in the safe direction: shrink grown solids by 1/32u, never grow them.

Water volumes (CONTENTS_WATER leaves): all free C-space inside them is reachable, with free vertical movement. Ladders (`func_useableladder` segments, `func_ladder` and CONTENTS_LADDER volumes): the climbable volume is reachable and connects to the walkable polygons at its ends.

## Stage 3: movement graph and flood

Nodes are walkable polygons plus water and ladder volumes. Edges:
- **Walk/step:** polygons whose boundaries are within step height (18u) and touch in XY, with the connecting segment free in C-space. Exact, on the CPU.
- **Fall and jump:** ballistic arcs on the GPU (OptiX). Launch from samples along polygon edges and interiors, for a direction fan and a speed set up to the profile's max ground speed, standing and crouch-jump. Each arc is traced as short segments, as rays against the grown-solid triangle mesh, so the point trace is an exact box trace. A landing on another polygon adds an edge. Arc samples also feed the eye volume (Stage 4).
- **Teleports:** a `trigger_teleport` whose volume intersects reached C-space seeds its target entity's origin. Same for `point_teleport` triggered from reached `trigger_*` (best effort, flagged in the report).
- **Brush entities other than `func_brush`** (`func_door`, `func_door_rotating`, `func_movelinear`, `func_tracktrain`, `func_tanktrain`, `func_breakable`, `func_physbox`, `func_rotating`, …), the "tops only" rule (user decision 2026-11-02):
  - *Player-solid check first.* Skip the entity entirely if its brushes' contents miss the player mask, or a spawnflag makes it passable (e.g. `func_door` "Passable", `func_illusionary`, `func_clip_vphysics`).
  - *Sides do not exist.* A solid mover contributes **no** C-space obstacle; doors are assumed openable. A permanently locked door used as a wall makes the flood over-reach, which is the safe direction: it costs savings, never blacks a face.
  - *Tops are floors.* Its upward faces (normal z ≥ 0.7) become walkable-polygon candidates, clipped against world solids for headroom, at every pose it can hold: spawn; open (`func_door` via movedir + lip, `func_door_rotating` at its open angle); start/end (`func_movelinear`); each `path_track` (trains).
  - *Ride edges.* Consecutive poses connect, and boarding edges (18u step) connect a pose to reached ground. The swept volume between poses feeds the eye volume. Riding a top is the only way a mover relocates the player; being pushed by one is ignored, since the player can walk that ground anyway. Vertical `func_door` elevators fall out of this automatically.
  - *Never occludes sight.*
- **`func_brush`** gets its own cases, because it is toggleable and parentable:
  - `Solidity` = Never Solid → ignored.
  - Unparented, `Solidity` = Always Solid, not `StartDisabled`, never targeted by a `Disable`/`Toggle` input → treated as world: full obstacle, occluder.
  - Toggleable or `StartDisabled` → tops only, like a mover, and no occlusion (it may be invisible).
  - Parented (`parentname` set) → tops only, at the parent's poses if the parent is a mover (train or door), else spawn pose. No occlusion.
- Per-classname overrides on the CLI for all of the above.
- **Pushes:** `trigger_push`, `trigger_catapult`, `func_conveyor`, `trigger_wind`. Reached C-space inside the volume launches arcs with the push velocity added (catapult: its target trajectory). The landings become edges.
- **Ladder jump-off:** every ladder volume also launches jump arcs along its length, not only at its ends.
- **Steep slopes** (normal z < 0.7): an arc landing on one continues as a slide down the slope, traced the same way. Slide samples feed the eye volume and the slide's end is a landing.
- **Vehicles** (`prop_vehicle_*`): not simulated. Their spawn origins are seeds, and `-vehicle-free` optionally treats all free ground C-space connected to a vehicle spawn as reached. Documented as a limitation.
- **Dynamic prop stacking** (piling crates to climb): not modelled; accepted limitation.
- **Seeds:** all `info_player_*`, `info_ladder_dismount`, CLI `-seed x y z`. Zero seeds is a hard error, never an empty result.

Flood with a BFS over the graph. Output: the reached node set and an unreached-but-walkable report, which shows mappers where the sim thinks players can't go.

**Game profile** (`bspwalk_profiles.txt`, KeyValues): hull sizes, eye heights (64/28), step 18, gravity, jump impulse, crouch-jump bonus, max ground speed per game/class, and a `reach_margin` multiplier (default 1.15) that deliberately over-estimates reach. Over-reach only costs savings; under-reach blacks out faces. The margin hedges physics-constant error **only**. Bhop and air-strafe accelerate past ground speed, so the profile carries a separate `max_air_speed` per game (its bhop/air-strafe ceiling where the game caps it; a generous default, 2× run speed, where it doesn't). The arc speed set runs up to that. Routes beyond it are an accepted limitation; the unreached-but-walkable report is where they show up. Profiles: hl2, episodic, cstrike, tf (max class speed), hl2mp. The default is chosen from the gameinfo.

## Stage 4: eye volume

For each reached walkable polygon, the eye volume is the prism over it from feet+eye_crouch to feet+eye_stand+max_jump_rise, clipped to free space. Add the jump/fall arc samples from Stage 3, and water/ladder volumes offset by eye height.

**3D skybox:** if `sky_camera` exists, every eye point maps into the skybox (`sky_origin + eye / scale`). Skybox faces are evaluated from those mapped eyes, never from reachability.

**Extra viewpoints** always added: `point_camera` origins (monitors), `env_cubemap` origins (cubemaps are captured from there, independent of the player), and any `-viewpoint` from the CLI.

**Second cameras:** water reflection/refraction and `func_reflective_glass` render from mirrored origins outside the eye volume. Any reflective surface whose face is visible (or unproven) is treated as an eye-volume cluster for **both tiers**. Its cluster's PVS joins the tier A set, and its mirrored eye points join the tier B eye set. Without that, tier A would not be a proof on maps with reflective water.

## Stage 5: visibility, tiered by proof strength

Occluders: triangles of opaque brush volumes. That means all sides of CONTENTS_SOLID brushes, nodraw/sky sides included, because the brush is solid behind them. Exclude translucent, alpha-tested, window, grate and water. Props do not occlude by default (`-propocclude` opt-in, opaque materials only). Brush entities do not occlude, except an always-solid, never-toggled, unparented `func_brush` (Stage 3 policy).

- **Tier A, PVS-proven hidden:** a face whose leaves (from `dleaffaces`, *all* of them, not the centroid's) are in no cluster of the PVS of any cluster the eye volume touches. VVIS PVS is a conservative superset (even `-fast` falls back to the looser portal flood), and areaportals only ever close visibility. So this is a proof given a correct eye volume, including the second-camera clusters and extra viewpoints from Stage 4. Requires VVIS data; without it tier A is empty and the tool says so.
- **Tier B, ray-tested hidden:** two GPU passes over the rest.
  1. *Eye-driven:* sample eye points through the eye volume (density adaptive per cluster). Each shoots a Fibonacci-sphere fan; first hits mark faces visible. Cheap, and clears most faces.
  2. *Face-driven:* for each face still unmarked, sample its area on a grid of ≤ 4u spacing (edges inset 1/16u) and trace to eye points in PVS-visible clusters, chosen by angular binning (one per solid-angle bin), not by truncating a distance-sorted list. Any clear ray means visible.
  Whatever survives is tier B.
- **Visible:** everything else.

**Output policy, the key safety lever:** tier A may become nodraw or max lightmap scale. Tier B gets **max lightmap scale only** by default. A sampling miss then renders blurry, never black. `-tierb-nodraw` is an explicit opt-in.

Separately, nodraw stops a face from bouncing light, so lit areas next to it get darker. Max lightmap scale keeps the bounce. The default for tier A is therefore also max lightmap scale, with nodraw as an opt-in.

## Performance targets (to verify, not promises)

fightspace3 end to end in under 5 minutes on the dev box (RTX, CUDA 13.2):
- Stages 1–2: CPU, seconds. Polytope count ≈ brushes + prop convexes + displacement triangles.
- Stage 3: about 10⁸–10⁹ short ray segments, seconds in OptiX.
- Stage 5: the eye pass is about 10⁶ eyes × 4k rays; the face pass is bounded by the PVS and angular binning.

Log stage timings and counts in the JSON so regressions are measured.

## Validation

Ground truth comes from fixture maps whose answers are known by construction, so the tests are written first and scored against the Python walker as the baseline. Directory: `game/bin/x64/unit_test_maps/bspwalk/`. Each fixture VMF has a sidecar `*.expect.json` listing side IDs that must be visible and side IDs that must be hidden. The harness **fails** on any must-visible face classed tier A, since tier A claims a proof. It **reports with a threshold** (target 0, gate configurable) must-visible faces classed tier B, because tier B is sampled and its failure mode is blurry rather than black. It reports, without failing, must-hidden faces it missed.

Fixtures, one mechanic each:
1. Sealed void room next to a spawn room: all inner faces hidden (tier A).
2. A 32.5u corridor off-grid, room beyond it.
3. Ledge 60u up, reachable only by crouch-jump; ledge 70u up, unreachable.
4. Platform across a 200u gap (running jump) and a 400u gap (not).
5. Room reachable only by climbing crates (prop collision).
6. Displacement hill with a hut on top.
7. Room behind a closed `func_door`.
8. Teleport-only room.
9. Water-only passage; ladder-only loft.
10. Wall top visible only at jump apex.
11. Large floor seen only through a 4u slit.
12. Chain-link prop fence in front of a wall.
13. 3D skybox with a building facade.
14. Platform reachable only by riding a `func_tracktrain`; ledge reachable only via `trigger_push`.
15. Room reachable only by climbing a `prop_physics` crate.
16. Face seen only in a reflective-water reflection, and one seen only by an `env_cubemap`.

The protected behaviour is the false-hidden count on known-answer geometry. A regression in reachability, collision or face sampling breaks it.

Integration check: `vis_simulator.py --debug` on fightspace3 and d1_trainstation, painted, inspected in Hammer.

## Milestones (GitHub issues #10–#15; open decisions #16)

1. **#10 Fixtures + harness**, scored against the current Python walker (baseline numbers).
2. **#11 Skeleton + Stage 1:** loader, vphysics props, displacements, grown solids, OBJ dump.
3. **#12 Stage 2–3 CPU:** walkable polygons, walk graph, seeds, BFS (fixtures 1, 2, 5, 6, 7, 15).
4. **#13 Stage 3 GPU:** ballistic arcs, ladders, water, teleports, movers with ride edges, pushes, slides, profiles (fixtures 3, 4, 8, 9, 14).
5. **#14 Stage 4–5:** eye volume, skybox, camera and second-camera viewpoints, tier A, tier B passes (fixtures 10–13, 16).
6. **#15 Integration:** lmoptimizer/vis_simulator call `bspwalk.exe`. Tiered output policy. Delete the Python walker and its one-off `test_144/475/oracle_*/user_pos/vmf_*` scripts. README build line.

Optional interim fix before milestone 6, only if the Python walker keeps being used: correct `phy_reader.py` (vertex base + axis order) and pass VPKs from `lmoptimizer.py`.

## Open decisions (owner: user; tracked in #16)

- Tier policy defaults above (max lightmap scale for both tiers, nodraw opt-in).
- ~~Mover default~~ decided 2026-11-02: tops only for all brush entities except `func_brush`, which has its own cases (Stage 3).
- Which game profiles ship first.
- VVIS required (tier A empty without it) or an internal portal flood fallback.

## Delegation log

- 2026-11-02: design review, `review_fast` (shop26 Qwen3.8-27B + glm-flash), both completed (~17 min). 14 findings, all accepted and folded in: reflection carve-out extended to tier A (both lanes), bhop hedge replaced by `max_air_speed` (both), mover ride edges, entity props, pushes/slides/ladder jump-off/vehicles, env_cubemap viewpoints, polylib precision, harness tier split, doc accuracy nits. No escalations.
