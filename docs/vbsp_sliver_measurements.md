# Sliver leaves and portals in VBSP: measurements (issue #104)

Measured 2026-11-12. Question: VBSP builds its portals with single-precision windings and planes. Does that produce sliver leaves and portals, what do they cost in the PVS, and would building the portal path in double remove them?

Answer: no. The slivers are in the brush planes, not in the winding arithmetic. A double rebuild of every map's portals has the same thin portals and the same small leaves, and its PVS differs from VBSP's by the one VVIS bug below plus an amount no larger than what rotating each winding's starting corner does. Double-precision portalization was not built.

## Tools (all read-only on the map)

- `experiments/prtvis/sliver_stats.py <map.prt> --bsp <map.bsp>`: counts of thin, small, short-edged and degenerate portals from the `.prt`, and the volume of every leaf's convex cell rebuilt in double from the BSP tree.
- `experiments/prtvis/double_portals.py <map.bsp> <map.prt> [--eps E] [--write out.prt]`: rebuilds the portals in double from the BSP's node planes (what a double-precision winding path would write), compares with the `.prt`, and can save the rebuild as a portal file in the `.prt`'s own order.
- `experiments/prtvis/prt_variants.py <in.prt> <out.prt> <edit>`: edited copies of a portal file (rotate the start corner, remove short edges, drop thin portals).
- `.agents/scripts/vvis_pvs_compare.py --run label=<bin dir>@<prt dir>`: runs vvis on each variant and lists the cluster pairs gained and lost.

Maps: ten compiled 2026-11-05 with this repository's vbsp (the HL2 maps d1_trainstation_02_hrcs, d2_lostcoast_d, d3_c17_02_camera; cm_laststand, ddd_market, fof_robertlee_12, fof_sawmill_12, l4d_hospital01_apartment, l4d_smalltown01_caves, test_hardware_d) and gm_fightspace_redux without func_viscluster, compiled for this measurement with `-game hl2mp` (3,737 clusters, 9,580 portals, coordinates to 16,341).

## Counts

Portals, from the `.prt`. "Thin" is the smallest extent across any edge; "tiny" is VBSP's own WindingIsTiny rule (fewer than three edges over 0.2).

| map | portals | max coord | repeated point | edge < 0.01 | edge < 0.2 | first 3 on a line | tiny | width < 0.5 | width < 1 | area < 1 |
|---|---|---|---|---|---|---|---|---|---|---|
| gm_fightspace_redux (no viscluster) | 9,580 | 16,341 | 0 | 0 | 0 | 0 | 0 | 0 | 4 | 0 |
| d1_trainstation_02_hrcs | 3,018 | 6,400 | 0 | 4 | 4 | 0 | 0 | 19 | 43 | 5 |
| d2_lostcoast_d | 2,943 | 13,312 | 0 | 3 | 7 | 0 | 0 | 7 | 52 | 35 |
| d3_c17_02_camera | 2,275 | 12,800 | 1 | 7 | 9 | 2 | 0 | 1 | 2 | 1 |
| cm_laststand | 439 | 5,968 | 0 | 0 | 5 | 0 | 0 | 1 | 1 | 0 |
| ddd_market | 3,712 | 3,696 | 0 | 5 | 5 | 0 | 0 | 3 | 7 | 0 |
| fof_robertlee_12 | 2,366 | 3,072 | 1 | 8 | 11 | 0 | 0 | 5 | 32 | 8 |
| fof_sawmill_12 | 2,380 | 6,848 | 0 | 0 | 1 | 0 | 0 | 10 | 10 | 0 |
| l4d_hospital01_apartment | 2,920 | 6,656 | 0 | 0 | 0 | 0 | 0 | 1 | 1 | 0 |
| l4d_smalltown01_caves | 2,545 | 16,256 | 1 | 1 | 4 | 1 | 0 | 26 | 65 | 9 |
| test_hardware_d | 7,321 | 13,312 | 0 | 14 | 31 | 0 | 0 | 7 | 78 | 55 |

No portal is bent by more than 0.01 from its own plane and none has a reflex corner over 0.01.

Leaves, from the BSP (non-solid leaves; a leaf's cell is the world box cut by the node planes on the way down, in double):

| map | open leaves | empty cell | volume < 64 | volume < 512 | thickness < 1 |
|---|---|---|---|---|---|
| gm_fightspace_redux (no viscluster) | 3,737 | 0 | 9 | 28 | 12 |
| d1_trainstation_02_hrcs | 1,042 | 0 | 8 | 36 | 25 |
| d2_lostcoast_d | 1,264 | 0 | 21 | 48 | 46 |
| d3_c17_02_camera | 882 | 0 | 0 | 4 | 1 |
| cm_laststand | 203 | 0 | 1 | 5 | 0 |
| ddd_market | 1,386 | 0 | 26 | 64 | 47 |
| fof_robertlee_12 | 825 | 0 | 27 | 52 | 34 |
| fof_sawmill_12 | 780 | 0 | 4 | 20 | 26 |
| l4d_hospital01_apartment | 1,578 | 0 | 0 | 2 | 0 |
| l4d_smalltown01_caves | 890 | 0 | 8 | 29 | 32 |
| test_hardware_d | 2,626 | 0 | 46 | 103 | 81 |

"Empty cell" is the one that would show a precision artifact: a leaf whose bounding planes leave no volume between them in double. There is none on any map, and no leaf under 1 cubic unit. The small leaves are real cells of the brush planes.

## The double rebuild against the `.prt`

Same cluster pairs and the same number of portals on every map: no pair VBSP wrote is missing from the rebuild. The rebuild finds a few more shared faces that VBSP did not write, 2 to 72 cluster pairs a map. On eight of the eleven maps most are specks under 0.05 square units (VBSP's tiny cull); the larger ones (63 of fof_sawmill_12's 72) are faces VBSP culled or keeps closed for other reasons, and were not broken down further.

Short edges and thin portals, `.prt` against the rebuild with VBSP's own split distance (`--eps 0.001`) and with an exact one (`--eps 1e-7`):

| map | edge < 0.01: .prt | rebuild 0.001 | rebuild 1e-7 | width < 1: .prt | rebuild 0.001 | rebuild 1e-7 |
|---|---|---|---|---|---|---|
| gm_fightspace_redux (no viscluster) | 0 | 0 | 2 | 4 | 4 | 4 |
| d1_trainstation_02_hrcs | 4 | 0 | 52 | 43 | 46 | 47 |
| d3_c17_02_camera | 7 | 3 | 28 | 2 | 2 | 2 |
| l4d_smalltown01_caves | 1 | 7 | 27 | 65 | 70 | 70 |
| test_hardware_d | 14 | 22 | 86 | 78 | 97 | 97 |

Thin portals do not go away in double; there are slightly more, because VBSP's cull removes a few. Short edges go up, not down, when the clip is exact: the node planes are stored in single precision and do not meet in points, so an exact clip cuts a corner VBSP's 0.001 tolerance would have left alone. With VBSP's tolerance the double rebuild has fewer on two maps and more on two. Short edges are a matter of how consistent the planes are and which tolerance the clip uses, not of winding precision. Getting rid of them would take double planes from the VMF's points on, through the plane table and the brush splitter, which is a much larger change than the portal path.

The portal areas agree closely: 0 to 65 pairs a map differ by more than one square unit (fightspace: at most 9 on a portal of 1.1 million; cm_laststand: 4 on one of 22,000). The likely source is the 0.1 distance MakeNodePortal chops against sibling portals with; that was not traced.

## What it costs in the PVS

vvis at the #110 cleanup (4fcfb7cf, before the plane fix below; default flow, `-mergeflow` on fightspace) on the `.prt` and on edited copies. Unordered cluster pairs gained and lost against the `.prt`:

| map | clusters | visible pairs | rotate start corner | first-3 fixed | short edges removed (0.01) | double rebuild | thin portals dropped (< 0.5) | thin dropped (< 1) |
|---|---|---|---|---|---|---|---|---|
| d3_c17_02_camera | 871 | 98,575 | +3 | +2 | +2 | +2 | 0 | 0 |
| l4d_smalltown01_caves | 890 | 84,193 | +43 / -5 | +43 | +43 | +43 | -212 | -1,345 |
| d1_trainstation_02_hrcs | 1,011 | 152,286 | -1 | 0 | -1 | -1 | -520 | -1,059 |
| gm_fightspace_redux (no viscluster) | 3,737 | 3,963,844 | +16 / -13 | no such portal | no such edges | +5 / -30 | none that thin | +1 / -2,140 |

- **Rotate start corner** is the control: the same portals with every winding started one point later. It moves 1 to 29 pairs, so anything of that size is not a signal.
- **First-3 fixed** is the one real effect, and it is a VVIS bug, not a VBSP precision problem: three portals in these maps start with a repeated point or three collinear points, VVIS took the plane from the first three points, got none, and closed the portal. Fixed in vvis (2bfe6f88): d3_c17_02_camera gains 2 pairs, l4d_smalltown01_caves 43, all through a 930 by 300 unit portal. Valve's SDK vvis has the same bug.
- **Short edges removed** and **double rebuild** give the PVS of "first-3 fixed" exactly on d3_c17_02_camera and l4d_smalltown01_caves (the same visibility lump, byte for byte), and the PVS of "rotate" exactly on d1_trainstation_02_hrcs (the one pair lost, 996<->999, is the same pair). On fightspace the rebuild moves 35 pairs where the control moves 29.
- **Thin portals** carry real pairs: up to 1.6% of a map's visible pairs are seen only through portals under 1 unit wide (none on d3_c17_02_camera; fightspace: 2,140 pairs, 0.05%, through four triangles of 7.6 square units). They are real openings in the brush geometry and they are there in double as well. Culling them is a visibility trade (a real slit stops showing what is behind it), not a precision fix.

Small leaves take part in 0.4% to 7.5% of a map's visible pairs (clusters under 512 cubic units: d3_c17_02_camera 4 clusters in 388 pairs, l4d_smalltown01_caves 29 in 3,258, d1_trainstation_02_hrcs 33 in 11,484, fightspace 28 in 65,167).

## Conclusion

- Sliver leaves and portals exist, in small numbers (portals under 1 unit wide: 0.03% to 2.6% of a map's portals; leaves under 512 cubic units: 0.1% to 6.3% of its open leaves), and they come from the brush planes. Double winding arithmetic leaves them as they are.
- The PVS a double portal path would give equals today's plus the VVIS plane fix, to within the noise of the winding's starting corner.
- So the change that was worth making was in VVIS, and it is made. Double-precision portalization would change every map's output for no measured gain.

Not measured:
- Float against double timing of the vvis flow (left open by #89). It needs a quiet machine; the lab box has no game directory to run the tools with.
- Leaks and in-game behaviour. Nothing here was looked at in game.
- The rebuild is a model of a double portal path (same planes, same tree, double arithmetic), not a build of one. VBSP's exact order of clips and its handling of a split whose one side is tiny (below) are not reproduced.

## Oddities seen on the way (not fixed here)

- `SplitNodePortals` (`src/utils/vbsp/portals.cpp`): when one side of a split portal is tiny, the portal keeps its whole unclipped winding on the other side; when both are, the portal is dropped without being freed.
- `c_tinyportals` is counted and never printed, so a compile gives no sign of how many portals the tiny cull removed.
- `WriteFloat` (`prtfile.cpp`) prints a coordinate within 0.001 of an integer as that integer, which is one way two close points become one repeated point in the `.prt`.
