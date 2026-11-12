# VBSP -detailclip (issue #92)

Off-by-default VBSP flags that drop auto-placed detail props (from `%detailtype`
materials and the `.vbsp` file) which clip into the map:

* `-detailclip` turns it on.
* `-detailclip_shrink <fraction>` (default 0.5, clamped 0 to 0.95) is how much of
  each prop's box may overlap geometry before it counts. 0 tests the full box.

Code: `src/utils/vbsp/detailclip.cpp/.h`, hooked from `PlaceDetail` in
`detailobjects.cpp`. Static prop instances come from `s_StaticPropLump` through an
accessor in `staticprop.cpp` (entities are stripped by then). Displacement meshes
are built from `mapdispinfo[]` with `DispMapToCoreDispInfo`, as `EmitDetailModels`
does (not from `g_CoreDispInfos`, which `-onlyprops` never fills); `PlaceDetail`
is told the `mapdispinfo` index of the displacement the candidate sits on (-1 on a
brush face).

## Tests per candidate

1. **Buried:** a point 1 unit above the spawn point along the normal is inside a
   visible solid world/func_detail brush.
2. **Brush overlap:** the shrunk box (horizontal about the centre line, vertical
   toward the base, base clamped to the spawn plane), cut by the spawn surface's
   tangent plane lifted 0.1, overlaps such a brush (exact convex SAT). When the
   cut leaves nothing (a world-upright box on a downward-facing surface, standing
   up into the brush it hangs from, or a prop sunk below its spawn point) the
   uncut box is tested. Every brush edge direction is a SAT axis, however short
   (only near-parallel duplicates within ~0.08 degrees are merged). A shape that
   hangs wholly below its origin shrinks toward its top, the attached end.
   The box is framed by the lump angles, as the engine draws it: sprites with
   the normal orientation conform to the surface like models (a square footprint
   as wide as the quad); screen-aligned orientations get a world-upright box
   wide enough for any yaw.
3. **Static prop overlap:** same box against static props' visible LOD0 triangles
   (default bodygroup), via a 2D grid of prop bounds plus a BVH per model.
4. **Displacement overlap:** same box against the triangles of every displacement
   except the one the candidate was placed on and those sewn to it (a candidate
   on a brush face is tested against all of them; see "Sewn neighbours"). Each displacement is its full-resolution displaced
   mesh in world space, one BVH per displacement, found through a 2D grid of
   displacement bounds. Tested last, so brush and static prop verdicts keep
   their counts and the displacement count is exactly the props it newly drops.
   There is no buried test for displacements.

"Visible solid": `CONTENTS_SOLID`, not clip/water/slime/window/grate/ladder, at
least one side that isn't nodraw/skip/hint, world entity only (func_detail is
merged into it). Sky brushes count as obstacles on purpose: players collide with
them and a prop poking into one shows through the sky. Displacements: the map
loader drops a displacement's parent brush from the brush list altogether (the
flat base is not the visible surface), so a displacement is an obstacle through
its mesh only, and only when its parent brush is solid by the same contents test
and its displaced side is not nodraw/skip/hint.

## Invariants

* Flag off: output is identical to stock (the sprite scale draw stays after the
  blocker check, same condition).
* Flag on: runs after every `rand()` draw for the prop, so it only removes props;
  the kept set is an exact subset of the flag-off set. Exception: a map that hits
  the 65535 detail cap with the flag off can keep props past the cap with it on.
  The displacement test is last and only adds a verdict, so the brush, buried and
  static prop counts do not move.

## Sewn neighbours (read before tuning the displacement test)

Since #112 a candidate on displacement A skips every displacement sewn to A as
it skips A itself: A and its sewn neighbours are one continuous surface, and
the relief of that surface is what the tangent plane stands for. "Sewn" means
sewn along an edge: every vertex of one whole edge of A lies within 0.1 units
of B's outer ring (its edge polylines), or the same with A and B swapped. That
covers a full shared edge at any power (a coarse edge's vertices land on the
fine edge, a sewn fine edge's on the coarse segments) and a smaller neighbour
along part of a bigger one's edge. A neighbour that touches only at a corner,
or whose shared edge was left unsewn anywhere along it (even with matching
corners), is not sewn and stays an obstacle, as does any displacement that does
not touch A (an overhang, a separate piece of terrain). The log line reports the
pair count. Cost of the rule: a displacement sewn to A along an edge is skipped
whatever its angle, so a cliff wall, or an overhang rising from A's edge, built
as its own displacement is not an obstacle for props on A, the same as one
modelled inside A never was. Limit: only one side's edge vertices are tested
against the other's ring, so a coarse edge whose vertices sit on a fine
neighbour's surface counts as sewn even if the fine mesh leaves it between
them (a power-0 neighbour with matching corners always does). That needs
relief returning to the coarse samples' heights; it was shown with a harness
(review round 2), not seen in a map.

Measured after the change (scratch content, `unit_test/detailprops`):
fightspace3 enriched with a 1200-density grass type on
`nature/blenddirtgrass001a` (319 displacements, 588 sewn pairs): 3677
displacement removals before, 50 after (hits on neighbours that touch only at
a corner or are not sewn along a whole edge); buried, brush and static prop
counts unchanged (35037 / 6894 / 37); flag off byte-identical. The generator's
map with `--overhang` (four sewn power-4 displacements, 4 sewn pairs, and a flat
displacement ceiling 30 units over them, sewn to nothing): 33 displacement
removals before, 26 after; the 7 regained are all within a few units of the
x=512 or y=512 seam, in valleys, and the 21 removals under the ceiling are the
same before and after. Raising one mid-edge vertex of a floor displacement by
20 units unsews that pair (3 pairs, 29 removals). The rest of this section is
the measurement that motivated it.

Before #112, a candidate on displacement A was tested against a displacement B that shares an
edge with A, but never against A. Coplanar brush floors cannot tell the
difference (exactly coplanar means exactly touching), but displaced terrain is
bumpy, so near a seam the neighbour's triangles rise above A's tangent plane by
the local terrain relief while A's own equally bumpy triangles are not tested.
Measured on fightspace3 (enriched, below): all 92 displacement removals are
candidates on a displacement hitting a lateral neighbour that shares an edge, 69
of them within 10 units (xy) of that neighbour's mesh and 85 within 20. In each
2-unit band out to 6 units from a neighbour 9 to 12 percent of the candidates are
removed, 6 percent from 6 to 10 units, 2 percent from 10 to 20, 0.02 percent
beyond 20; testing the own displacement as well would remove 3.2 percent of
all candidates in the interior. How far the neighbour surface rises over the box
footprint, in the 92 removals: all above 0.11, 62 above 0.5, 53 above 1, 36
above 2, 21 above 4, 10 above 8 units. So most are real clipping into a
rising slope, but about a third are sub-half-unit relief a viewer would not see.
If that matters, the lever is a larger penetration tolerance for displacement
triangles than the 0.01 used for brushes (30 of the 92 rise by 0.5 or less), or
testing the own displacement with the triangle underfoot and its direct ring left
out.

## Not covered

* A buried test for displacements (a point inside the terrain mesh).
* Terrain relief inside the displacement the candidate sits on, or in one sewn
  to it (including a sewn cliff wall at the floor's edge).
* A displacement's parent block (side walls and base): the map loader drops the
  brush, so only the displaced mesh is an obstacle.
* Sewn-seam vertex snapping: VBSP snaps vertices dropped by a neighbour of
  different power into the displacement verts it writes to the BSP (and its
  core copy), but not into `mapdispinfo`. The test mesh (like placement) is
  built from `mapdispinfo`, so at such seams it can sit slightly off the drawn
  surface.
* Brush entities (func_brush, func_wall), glass, grates.
* A prop entirely inside a closed static-prop mesh (no triangle crosses its box).
* Tri/cross sprite `shape_size` fan-out; sprites are a square footprint of the quad.
* Plain card sprites are tested as a square, so as thick as they are wide (it
  covers the tri/cross shapes; a card is conservative only in that it may be
  removed for an overlap just behind its plane).
* Static prop models are all loaded up front (eager; fine so far: 22 models in
  26 ms on fightspace).

## Measured (fightspace3_unittest.vmf, hl2mp)

32351 candidates, 2047 removed at shrink 0.5 (1769 buried, 276 brush, 2 static
prop), ~20 ms to build lookups, ~10 ms testing (2041/270 before orientation-0
sprites were framed by their angles).

Displacements (#107): under plain hl2mp content fightspace3 has almost nothing to
test (the `legoj15/` materials and `detail_fightspace.vbsp` are not there: 0
candidates, or 213 if its `detailvbsp` is pointed at hl2's `detail.vbsp`). The
numbers here come from a scratch game dir that mirrors hl2mp's search paths and
overrides `nature/blenddirtgrass001a` with `%detailtype terrain_detail`
(`detail_fightspace.vbsp` copied beside it), which puts props on the 319
displacements: 41555 candidates, 2642 removed before (2334 buried, 306 brush, 2
static prop), 2734 now, the 92 extra all `displacement`. Lookups took 80 to 115 ms
(about 30 before: 319 meshes built through `DispMapToCoreDispInfo`), testing 17
to 19 ms. Flag off: every lump identical to the build before. Flag on: the kept
set is a subset of the old kept set (0 props only in the new one).

Overhang map (scratch generator on `vmfgen.py`, not kept): a 2048x1024 room with a
grass brush floor on the left half and a grass displacement floor (power 4) on the
right, each with a displacement overhang whose underside is 10 units above the
floor and another 300 units above it. Props placed well inside each footprint
(flag off / old -detailclip / new -detailclip): low over brush floor 160 / 160 /
28, low over displacement floor 145 / 145 / 24, high over either 164 / 164 / 164
and 167 / 167 / 167; open floor unchanged. Each prop under a low overhang is
removed exactly when its shrunk box (half its sprite height times its scale) is
taller than 10 units: 132 of 160 and 121 of 145 expected, 132 and 121 removed,
no disagreement. Flag off was identical to the old build except the default HDR
cubemap texture in the pak lump, which differs between two runs of the old build
too (this map has a real sky). The same map through `-onlyprops -detailclip`
also works (136 more removals than the old build, subset holds).

Ceiling check (scratch room, grass floor and ceiling, one orientation-0 and one
orientation-2 sprite): all 236 screen-aligned ceiling sprites removed, 165 of 180
hanging ceiling sprites kept (the rest touch walls); subset holds. Subset check passes. Scratch tooling
used: a `dprp` lump parser comparing two BSPs' detail props as multisets.

## Delegation log

* impl (Sonnet): built the feature, flag-off identity and subset checks, tiny-map
  check against an independent Python model; no escalation.
* reviewer (Haiku): found the displacement-brush false burial (major, real) plus
  minors (64K cap subset, shape_size, side-0 contents, comment drift).
* review_fast r1 (glm-flash, deepseek-flash stand-in for a failed shop26):
  deepseek independently found the displacement bug and the underground-slab
  shrink bug (medium, real, fixed); the rest low/accepted.
* review_fast r2 (shop26, glm-flash) on the fixes: both approve, notes only.
