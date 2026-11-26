# VBSP -reportdetail (issue #101)

`vbsp_lmo -reportdetail [N] -game <dir> map.vmf` lists the structural world
brushes that do not face the void and cost the most vis portals, so the mapper
can look at them and decide which should be `func_detail`. Off by default. It
changes nothing: with or without it the BSP and the `.prt` are byte for byte the
same (checked with `bsp_lumps.py diff` and `cmp`, see "Checks"). It never
promotes a brush. Structural or detail is design intent, and a wrong guess opens
a leak the mapper cannot explain.

Needs a full compile that does not leak: the void test needs the entity flood.
With `-onlyents` / `-onlyprops` (no tree) or on a leaked map it warns and writes
nothing.

## Output

Console: a table of the top N rows (default 40; `0` prints the summary and the
file only), then one summary line.

| column | meaning |
|---|---|
| rank | by portals charged, then nodes, then Hammer id (ascending), so the order depends on the map only |
| brush id | the VMF `solid` id (`mapbrush_t::id`), the brush id Hammer uses |
| portals | visportals in the `.prt` that lie on a plane of this brush's nodes |
| nodes | nodes of the final tree whose split plane this brush supplied |
| size x y z | bounding box of the brush, so a small brush with a big cost stands out |
| centre x y z | centre of that box, in Hammer coordinates |
| material | the brush's first drawn (non-nodraw) side; an env_cubemap patch name is shown as the material it patches |

The summary line gives the candidates, the structural world brushes considered,
how many of those face the void, and the share of the map's visportals charged
to candidates.

`<map>.detail.txt` beside the BSP holds every candidate (not only the top N): a
header line, then the same columns tab separated (sizes and centres with one
decimal). A script can read it directly. The brush ids in it are what to feed to
Hammer or to a VMF edit.

## How it decides

Which brushes can be listed (`IsCandidate`): world entity brushes (`entitynum 0`)
that are opaque solid, except these, which are never listed:

* brushes of other entities: func_detail (already detail), func_areaportal
  (moved into the world tree, shapes the areas), and a func_instance's world
  brushes (their ids are shifted by the main map's highest id, so they are not
  the ids in the instance's VMF);
* contents water, slime, window, grate, translucent (volumes, glass and grates;
  the entity flood passes through them), blocklos, areaportal, playerclip,
  monsterclip, origin, ladder, detail;
* a brush with any side marked hint, skip, sky or trigger (the mapper put those
  splits and seals there on purpose);
* a brush whose every side is nodraw (invisible walls, seal and vis shapers).

(a) Does the brush face the void? `DetailReport_FindVoidFacing` runs in
`ProcessWorldModel` after `FloodEntities` and before `FillOutside`, on the tree
that is kept (the second pass, or the first with `-noopt`). `FillOutside` makes
every leaf the flood did not reach solid, which erases the answer, so it has to be
asked first. A brush faces the void when a solid leaf holding a piece of it
(`node_t::brushlist`, each fragment points back to its map brush) has a portal to
an empty leaf with `occupied == 0`, or to the world edge. Such a brush may be part
of the seal and is never listed. A brush that only touches other brushes and
reached space is a candidate. Unreached empty pockets (a closed closet with no
entity in it) count as void, so the walls around them are never listed either.

(b) What does it cost? Every tree node records the index of the map brush that
supplied its split side (`node_t::sideBrush`, set in `SelectSplitSide` beside the
copied texinfo and surf from #136, so it survives the per-block trees being merged
under `BlockTree`). The detail tree adds no nodes (`MergeDetailTree` only files
brush pieces into leaves), and `PruneNodes` demotes nodes to leaves, so the count
walks the final tree and skips leaves. In `WritePortalFile`, after the portal list
is built, each listed portal is charged to the brush of the node it lies on
(`portal_t::onnode` is the node the portal was cut on and its pieces keep it).
The portals counted are exactly the ones in the `.prt`. Brushes with no node
charged are not listed.

## Read it as advice

* **A shared plane is credited to one brush.** A floor made of many brushes on one
  plane, or a pillar standing on the floor, gives the plane to the brush whose
  side `SelectSplitSide` met first at the best score (the brush list is reversed
  at every split, so which brush that is can change with the level). The numbers
  are not exact per brush, and two brushes that share planes split the credit
  arbitrarily. Look at the neighbours.
* **A detail brush does not block visibility.** A listed interior wall or floor
  slab between two rooms may well be the thing hiding one room from the other;
  as func_detail it stops doing that. The report ranks cost, not benefit. Keep
  big occluders structural; detail the clutter (pillars, beams, trim, stairs,
  small blocks in a room).
* **The numbers are not additive, and not a forecast.** Converting a brush changes
  the tree and the planes that others were credited for, and a pillar also
  fragments the portals that lie on other brushes' planes, which the pillar is not
  charged for. Measured: three free-standing pillars in a box room were charged 28
  of the 45 portals and moving them to func_detail dropped the `.prt` from 21
  clusters / 45 portals to 7 / 10. On d1_trainstation_03 (1010 clusters, 2499
  portals) the top 15 were charged 287 portals; moved to func_detail the `.prt`
  went to 971 / 2383. The top 40 were charged 506; moved, it went to only
  1001 / 2488. Recompile and read the new counts; do not add the column up.
* A brush that faces the void can still be a poor structural brush; it is left out
  because the report cannot tell it from part of the seal.
* Displacement brushes are listed like any other brush.

## Cost and determinism

The void test and the count are one walk of the leaves and one of the nodes:
0.02 s on d1_trainstation_03 (the line prints it). The tree is the one the block
pass builds, identical at any `-threads` (#103), so the table and the file are
too. `node_t::sideBrush` fills the padding before the first pointer after
`sideSurf`, so a node is no larger.

## Code

* `src/utils/vbsp/detailreport.cpp/.h`: the two entry points
  `DetailReport_FindVoidFacing` and `DetailReport_Emit`, the exclusion rules.
* `brushbsp.cpp`: `SelectSplitSide` sets `node->sideBrush`; `AllocNode` starts it
  at -1.
* `vbsp.cpp`: `-reportdetail` parsing and usage; the void test call before
  `FillOutside`.
* `prtfile.cpp`: `WritePortalFile` hands the portal list to `DetailReport_Emit`
  after the file is written.

## Checks

Compile a map with the previous build, with this build, and with this build plus
`-reportdetail`; `python -I game/bin/x64/scripts/bsp_lumps.py diff a.bsp b.bsp`
must say IDENTICAL for all and `cmp a.prt b.prt` must be silent. Then compile with
`-threads 1` and `cmp` the two `.detail.txt` files. A small test of the idea: a
sealed box room with a few free-standing structural pillars on the floor, one
pillar that pokes through the ceiling, and one func_detail pillar; the report must
list the free-standing pillars only, and moving them into func_detail must drop
the cluster and portal counts in the `.prt` header.
