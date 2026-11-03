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

### bspwalk usage (milestone 3)

Source `src/utils/bspwalk/` (CPU only; CUDA/OptiX join in #13 with the first `.cu`). Build: run `src/createallprojects.bat` (VPC), then from bash
`env -u NoDefaultCurrentDirectoryInExePath "/C/Program Files/Microsoft Visual Studio/18/Community/MSBuild/Current/Bin/MSBuild.exe" src/mapping_tools.sln -t:bspwalk_win64 -p:Configuration=Release -p:Platform=win64 -m` (use `-t:`/`-p:`, Git Bash rewrites `/t:`). Output `game/bin/x64/bspwalk.exe`.

`bspwalk <map.bsp> -game <dir> [-dumpobj <base>] [-checkprops] [-verify [-verifydump <file>]] [-threads N] [-seed <x y z>]... [-nowalk] [-out <json>]`, plus `-selftest` (grow and polygon-subtraction known answers, no map) and `-growdebug <file>` (grow the hull of the first `v x y z` block in a file, such as a `-verifydump` file, with every candidate decision printed). Exit code 1 on a load failure, no game directory at all, any `-verify` failure, a malformed `-seed`, or a flood with no seed (or none that maps to a polygon); in that last case no JSON is written. `-nowalk` stops after Stage 1 (and `-verify`).
- Without `-game` (or `-vproject`) the `VPROJECT` environment variable is used, as in every SDK tool, and the tool prints which game that is. Tests and agents always pass `-game` (see `AGENTS.md`).
- `-threads` values of zero or less, or non-numeric values, fall back to all cores with a warning.

- Categories (`bw_world.h`): `world`, `func_brush_solid`, `static_prop`, `disp` are grown obstacles; `entity_prop` is grown but standable only (its tops are floors, it clips nothing); `tops_only` (other solid brush entities) stays ungrown in Stage 1 and is grown per pose in Stage 2 (below). The brush-entity class/spawnflag policy is one table in `bw_world.cpp`. func_door and func_door_rotating with spawnflag 4 ("Non-solid to Player") or 8 ("Passable") are skipped entirely.
- Props: a vphysics-solid prop whose `.phy` is missing or unreadable falls back to its `.mdl` hull box, with one warning per model. VRAD instead builds hulls of the render meshes, so this fallback is coarser. Static prop entries use the struct size of their lump version (v4, v5, v6, v10). If the lump size is not count × size, every static prop is skipped with a loud warning; other versions use an inferred stride and warn.
- `-out` JSON: `walker`, `milestone`, `profile` (hull sizes, `step_height`, `max_jump_rise`), per-category `counts`, `skipped` counters, `prop_instances_hull_fallback`, `world_aabb` (null when no world brush is player-solid) vs `model0_aabb`, `reach` (below), `timings_s` (Stage 1 phases plus `walk_polygons`, `walk_stand`, `walk_edges`, `walk_flood`, `walk`), and an empty `faces` list (harness contract).
- `reach`: `seeds` (`point`, `source` = classname or `cli`, `node` or null when unmapped), `edges` (`walk`, `jump`, `drop` counts), `node_count`, `total_area`, `reached_count`, `reached_area`, `up_faces`, `mover_entities`, `mover_poses`, `mover_grow_failed`, `unreached_walkable` (`id`, `centroid`, `area`, `category` of every unreached node), and `nodes` (`id`, `category`, `convex`, `pose`, `reached`, `area`, `plane` = `[nx, ny, nz, d]` with `n . p = d`, `normal_z`, `stand_count`, `stand_area`, `verts`). The harness reads only `nodes`. On fightspace3 the file is about 780 MB (1.8M nodes), most of it `nodes`.
- `-dumpobj <base>` writes `<base>_raw.obj` (source convexes, all categories), `<base>_stand.obj`, `<base>_crouch.obj`. One `o`/`g` per convex named `<category>_<n>[_<model or classname>]`, same name in all three files. After a flood it also writes `<base>_reached.obj` and `<base>_unreached.obj`, one object per node named `<convex name>_p<pose>_n<node>`. Source units, Z up: in Blender import with Up Axis = Z, Forward Axis = Y.
- The console prints `walk: <up faces> up faces -> <nodes> nodes (<area>); edges walk/jump/drop; <seeds> seed(s) -> <reached> nodes reached (<area>); <movers> mover entities, <poses> poses` and a `walk timings (s)` line.

How Stages 2 and 3 work (`bw_walk.cpp`, milestone 3, #12):
- **Up faces.** The crouch-grown face polygons with n.z ≥ 0.7 (0.7 is `WALK_MIN_NZ`) of every grown category, plus those of each `tops_only` mover pose. Up faces are the same for both postures (for n.z > 0 the support of −B lies on its z = 0 face, which does not depend on the hull height), so nodes are crouch polygons; each node also carries its stand sub-polygons (`stand_count`, `stand_area`), the parts free of the stand-grown obstacles. Nothing is derived from plane triples: only face polygons and planes as half-spaces.
- **Subtraction.** Each up face is chopped at the 64u grid lines (`WALK_CHOP`, X then Y), then every piece has the other obstacles of the same posture subtracted: a quick "does the polygon reach the interior" test (separating plane, then a Sutherland-Hodgman clip), then a peel, one plane at a time, of the parts in front of each plane. Clippers are shrunk by 1/32u (`d -= 1/32`, `WALK_SHRINK`), the safe direction. A polytope never clips its own face. Only `CategoryIsObstacle` polytopes clip; `entity_prop` and mover tops never clip. Slivers are kept: a piece is dropped only when its area is 0 at 1e-9 of its extent squared. The spatial index is a hierarchical XY hash (cell 64·4^L, 9 levels, each polytope at the finest level where it spans at most about 3 cells).
- **Coplanar dedupe.** Two obstacles that share a coplanar up face region (a brush on a brush of the same height, both grown 16u past their edges) would give the same floor twice, since a shrunk clipper never removes the other's coplanar top. The lower obstacle index keeps the shared region: the higher one subtracts the lower one's face prism (its edge planes plus a ±1u slab). This is exact, because the region is clipped by the same obstacles either way.
- **Poses** (`tops_only` only, grown with the crouch hull at each pose; their up faces, which are the same for both hulls, are clipped by world obstacles; #13 adds the stand grow if ride edges need it). Poses are computed once per entity: spawn; `func_door` open at `movedir · (|movedir · size| − lip)` (`doors.cpp`; `size` is the dmodels size, which equals the engine's `OBBSize − 2`); `func_door_rotating` at `angles ± axis · distance` about the entity origin (spawnflag 2 reverses, 64 = roll, 128 = pitch, `distance` defaults to 90; a yaw door that is not one-way, spawnflag 16, gets both signs because it opens away from the player); `func_movelinear` at `origin − movedir · MoveDistance · StartPosition` and that plus `movedir · MoveDistance` (`func_movelinear.cpp`; a missing `MoveDistance` falls back to the door travel). Each node carries `category`, `convex` and `pose` (0 = spawn).
- **Deferred to #13** (a `TODO(#13)` in `EntityPoses`): train poses at each `path_track`, parented `func_brush` at its parent's poses, and ride edges between consecutive poses (plus the swept volume). Until then each pose's tops are separate nodes, reached only by walking, jumping or dropping onto them.
- **Edges.** For each node pair whose XY bounds touch, take the parts of A's boundary inside B's XY projection (expanded 1e-4u) and of B's inside A's: sub-segments P(s) = P0 + s (P1 − P0), s in [0, 1]. Along one, both planes are linear in s, so dz(s) = zB − zA is linear and each kind's dz band is an exact s-interval: `|dz| ≤ 18` (`PlayerProfile::stepHeight`) is a two-way walk edge; `18 ≤ dz ≤ 57` (`maxJumpRise`, 21 + 36 crouch-jump) a jump-up edge from the lower node; `dz ≥ 18` the drop edge from the higher one (band ends closed: that only adds edges, and walk wins anyway). The blocked parameters for one shrunk crouch obstacle are the s-projection of (the vertical region swept between A and B over the sub-segment) ∩ (the polytope): split at the s where zA and zB cross, each piece is a convex quad in (s, z), clipped by the polytope's planes (tolerance `FREE_TOL` in the planes, so touching is free), and its s-range is one closed interval. A kind holds where its band meets the free set (the sub-segment minus the union of those intervals), or where `|dz| ≤ 1e-6` (the surfaces meet). Speed: every sub-segment's ends and midpoint are tried first against the point test (a subset of the exact answer); the exact pass then runs only for kinds still missing, only over the span of their bands, and stops as soon as one obstacle (the last blocker first) covers that span. A pair with any walk gets only the walk edge. Known answers in `-selftest` (e): a flat floor meeting a slope whose step band lies strictly between the samples gets a walk edge; a 100u drop whose samples are all blocked gets its drop edge through the gap between two slabs; one slab over the whole boundary gives no edge. Both of the first two failed against the sampled version.
- **Seeds and flood.** Seeds are every `info_player_*` and `info_ladder_dismount` origin and each `-seed`. A seed maps to the highest node that contains it in XY (1e-3u tolerance) with plane z in `[z − 256, z + 18]` and a free vertical segment between the plane and the seed; a plane at most 2u above the seed (an origin embedded in its floor, which starts inside the shrunk floor solid) needs no segment test. An unmapped seed warns (fightspace3's four `info_ladder_dismount` at z −15489 map to nothing); zero seeds, or none mapped, is the hard error above. The flood is a BFS over walk edges both ways and jump and drop edges one way.
- **Threads.** All passes run through `RunThreadsOnIndividual` in blocks. bspwalk replaces the global `new`/`delete` with the Windows process heap (`bw_util.cpp`, `NO_MEMOVERRIDE_NEW_DELETE` in `bspwalk.vpc`): tier0's allocator takes one lock per allocation, and with it the grow took 5.8 s on 32 threads against 0.7 s on 4. `malloc` stays on tier0, so the exe has two heaps. Open item: dropping `memoverride.cpp` from bspwalk altogether (one heap, the CRT's) crashed in Stage 1 and was not investigated.
- **Spatial hash queries** return sorted, deduplicated ids by sorting the appended range (O(candidates)). The first version kept a per-thread stamp array sized to the object count: 1.8M nodes × 33 thread slots, 240 MB on fightspace3 (peak working set 2,088 MB before, 1,850 MB after).
- **fightspace3** (32 threads, 2026-11-03, exact edges): 134,453 up faces → 1,801,627 nodes (3.50e9 u²); edges walk 9,248,605, jump 68,350, drop 752,277 (the sampled version had 752,275); 13 seeds (9 mapped) → 1,216,823 nodes reached (1.27e9 u²); 132 mover entities, 158 poses. Stage 2+3 4.2–4.3 s (polygons 0.58, stand 0.11, edges 3.3, flood 0.15), grow 0.36 s, total 4.8 s, peak working set 1.85 GB; one earlier `-out` run measured 6.6 s for Stage 2+3, so expect run-to-run spread. The edge pass is the remaining cost (pure compute). The node count comes from the 64u chop over large areas, including surfaces outside the playable map (the top of the map's outer shell, above z 12288, alone is 247k unreached nodes).
- `-checkprops` prints each prop model's collision AABB (model space) against its `.mdl` `hull_min/hull_max` and flags differences over 1u.
- `-verify` checks every grown polytope against the exact sum point set `{v + c}`: every plane supports it, every face vertex is a sum point, and every corner of the plane intersection (each plane's winding clipped by all the others) lies within 6e-8·|coord| + 1e-5 (the input's float rounding) of the hull of those points, measured with GJK. GJK builds no hull, so sliver triangles cannot fake or hide a miss. A polytope that was never checked also counts as a failure. `-verifydump <file>` writes each failing convex's source points and planes as `# convex` blocks, for `-growdebug` or an offline qhull check. On fightspace3 it takes about 11 s, against 6 s for the grow.

How the grow is exact (`bw_grow.cpp`): candidate normals are the source face normals, the 6 axes and ±edge×axis for every edge. A candidate is a facet when its support set (source vertices and box corners attaining the max) spans 2-D. Facets are deduplicated by support set, never by near-equal normals: two real edges 1e-7 rad apart give two real facets, and merging them overgrew a fightspace3 prop by 0.001u. Each face polygon is the 2-D hull of its support set's sum points, so every vertex is an exact `v + c` and no plane intersections are computed. Clipping nearly parallel planes (ChopWinding) put vertices up to 1e-3u off. ChopWinding (double port) still builds the brush convexes from their planes. Prop convexes are rebuilt as the hull of their vphysics points, because vphysics' float triangulation of "planar" quads is bent and its diagonals need not be hull edges (a missing edge dropped a facet and overgrew a crate by 4.5u). The grow works on the hull of the source points (quickhull), because brush windings clipped from float planes are convex only to about 1e-4u. Candidates also include the face normals of the hull of the sum points, a safety net for the micro-chamfers that float brush planes leave. An extra candidate is always safe: its offset is the exact support, so it can only remove overgrowth. Support sets use a tolerance of 1e-9·|coord| + 2e-6. Measured overgrowth (the distance from a plane-intersection corner to the sum hull) on fightspace3 is at most 2.3e-4u, at |coord| 13316, which is below half a float ulp of the coordinates there. The 16 fixtures and all other fightspace3 convexes are within 1e-8·|coord| + 1e-5. The planned 1/32u shrink (Stage 2) absorbs it.

For #13: the grown planes include near-coplanar pairs, which come from source faces tilted by float error (6.5e-5 rad apart on convex 371). Their intersection lines cross faces far from any real vertex: on fightspace3 convex 371, one corner is 1176u from every sum point but only 4e-5u outside the hull. Use the face polygons, whose vertices are exact sum points, or the planes as half-spaces. Never derive vertices by intersecting plane triples.

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
3. Ledge 48u up, reachable only by crouch-jump; ledge 112u up, unreachable. (HL2's duck-jump limit is 21 + 36 = 57u, so the original 60/70u pair sat on the wrong side of it; 112u also clears CS:S's 93u.)
4. Platform across a 96u gap (running jump) and a 400u gap (not). (200u was at HL2's sprint-jump limit, about 202u.)
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
- 2026-11-02 (#10): `scout` toolchain survey, done. `impl` vmfgen + harness + Python adapter + f01-f04, done (found the face.side flip bug, #17). Three parallel `impl` fixture agents (f05-07/15, f08-10/14, f11-13/16), all done first try, each with a brute-force sightline check of its expectations (raw checkers saved on #19). `reviewer` on the harness core: 1 major (stale BSP under `--skip-compile`), its one unverified guess (in-air duck lifts 18u) refuted from SDK `FinishDuck` (full 36u). `impl` applied the fixes. No escalations.
- 2026-11-03 (#10 follow-up): `review_fast` on the #10 commit: shop26 found a NameError debug print in `visibility.py` (fixed, 29ca5dbd) and the dead `--no-side-flip` lead (noted on #17); glm-flash found the lax side-id check (fixed).
- 2026-11-03 (#11): `scout` build-stack survey, done (returned inline instead of writing the named file; found the CUDA 13.1 pin, #20). `impl-opus` skeleton + Stage 1, done first try (selftest written first, 17 maps verified). `reviewer` (Haiku): 1 blocking (`-threads 0` grew and verified nothing yet passed), 13 minor; `impl-opus` fixed 10. `review_fast` (shop26 + glm-flash): both timed out at the cap while building their own Python ports of the grow; no findings filed, partial notes confirm the lump offsets, disp builder, AngleMatrix port and brush-entity origin handling, and 25 fuzzed shapes matching the exact sum. Orchestrator reverted the hard `-game` requirement to a VPROJECT fallback with a notice (user call). No escalations.
- 2026-11-03 (#12): `impl-opus` Stage 2 (exact walkable polygons, mover poses) + CPU Stage 3 (walk/jump/drop edges, seeds, BFS flood), `reach` JSON, reach scoring in the harness, `expect_reach` on f01/f02/f05/f06/f07/f15: done first try (subtraction selftest written first and failed 4 checks on a stub; 6 fixtures pass 14/14 reach and 9/9 no-reach, each expectation fails when the scorer is fed all-unreached or all-reached nodes). fightspace3 Stage 2+3 went from 81 s to 4.2 s: fast intersection test, allocation-free peel/chop, coplanar dedupe, and the global new/delete moved off tier0's allocator (a cdb sample showed 31 of 32 workers in its lock; Stage 1 grow 5.8 s -> 0.36 s with it). No escalations.
- 2026-11-03 (#12 review round 1): `reviewer` (Haiku) + `review_fast` (shop26 + glm-flash), 8 findings. `impl-opus` fixed 7: exact edge test along each boundary sub-segment (selftest (e) written first; 2 of its 3 checks failed on the sampled version), embedded seeds within 2u, O(candidates) hash dedupe (peak working set 2,088 -> 1,850 MB), numeric validation of `reach` nodes in the harness, mover stand grow dropped, `EntityPoses` once per entity, memoverride open item. Refuted 1 with a measurement: the stand sub-polygons start from the already deduplicated crouch polygon, so passing the own plane to the stand pass changed no `stand_area` on the 6 fixtures or fightspace3. fightspace3 counts unchanged except drop +2; Stage 2+3 4.36 -> 4.24 s. No escalations.
- 2026-11-03 (#12 review rounds 2-3): `review_fast` on 2b513cea (shop26 + glm-flash): both approve, exact edge path fuzzed by shop26 against dense sampling with 0 misses. Orchestrator fixed 2 lows (tested seed node beats an untested embedded one; zero-travel func_door adds no duplicate pose), 57f8c39b. Refuted: "no-model func_door" (compiled BSPs always carry `*N`; the reviewer read the VMF). `review_single` glm-flash on 57f8c39b: approve, 1 low adjudicated as accepted (a tested node far below can beat an embedded one; the downward segment of an embedded seed runs through its own floor, so the window is narrow). No escalations.
