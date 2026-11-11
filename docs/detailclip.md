# VBSP -detailclip (issue #92)

Off-by-default VBSP flags that drop auto-placed detail props (from `%detailtype`
materials and the `.vbsp` file) which clip into the map:

* `-detailclip` turns it on.
* `-detailclip_shrink <fraction>` (default 0.5, clamped 0 to 0.95) is how much of
  each prop's box may overlap geometry before it counts. 0 tests the full box.

Code: `src/utils/vbsp/detailclip.cpp/.h`, hooked from `PlaceDetail` in
`detailobjects.cpp`. Static prop instances come from `s_StaticPropLump` through an
accessor in `staticprop.cpp` (entities are stripped by then).

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

"Visible solid": `CONTENTS_SOLID`, not clip/water/slime/window/grate/ladder, at
least one side that isn't nodraw/skip/hint, world entity only (func_detail is
merged into it), and **no displacement sides** (the parent brush is the flat base,
not the visible surface). Sky brushes count as obstacles on purpose: players
collide with them and a prop poking into one shows through the sky.

## Invariants

* Flag off: output is identical to stock (the sprite scale draw stays after the
  blocker check, same condition).
* Flag on: runs after every `rand()` draw for the prop, so it only removes props;
  the kept set is an exact subset of the flag-off set. Exception: a map that hits
  the 65535 detail cap with the flag off can keep props past the cap with it on.

## Not covered

* Displacement terrain as an obstacle (grass under an overlapping displacement).
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
