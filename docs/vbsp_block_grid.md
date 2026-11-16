# VBSP block grid: what it buys and what it costs

Second look at #96, #105 and #119, measured 2026-11-14 on the dev box (shared with other sessions: read vis times to a factor of two). Branch `claude/issues-105-119-review-fe906e`.

## How it was measured

- `.agents/scripts/blockgrid_sweep.py`: one compile, one vvis run and one `pvs_cost.py` measurement per map and grid variant. `--remeasure` re-reads the BSPs a run left behind.
- `game/bin/x64/scripts/pvs_cost.py` now has a second viewpoint model and counts static props.
  - **Floor-weighted** (`floor_*`): the viewpoint is uniform over ground a player can stand on: drawn world faces and displacements that face up, without sky, water, nodraw and tool surfaces. `floor_seen_pct` is the share of that ground in the PVS. This is the number to read.
  - **Volume-weighted** (`expected_*`): the model the #96 and #105 tables used. It follows the largest leaf. d2_prison_05 reads 90 % of its volume visible while its mean cluster sees 14 % of the clusters, and its volume-weighted triangles fall 41 % between 1024 and 2048 while what the ground sees moves 0.2 %.
  - Static props are counted as props in view, not triangles. Fade distances are ignored.
- `vbsp_lmo -blockoffset X Y` moves the grid lines. Running one size at several offsets gives the spread that comes from where the lines fall, with nothing else changed.
- `-blocksize 512` is now accepted (it was 1024 and up).
- Exact vvis on the HL2 maps (HL2-MapPack VMFs), `-mergeflow` on gm_fightspace_redux. Its 208 func_viscluster entities are all hidden in the VMF, and vbsp skips hidden blocks, so it compiles with no visclusters as it is (3737 clusters, the number #96 and #105 report for the viscluster-less map). d2_coast_07 is the one measured HL2 map with func_viscluster entities (three); it was compiled with them, as authored. `-game` is the SDK Base 2013 Multiplayer `hl2mp`.
- The stock grid is unchanged by these edits: d1_trainstation_03 and d2_coast_03 compile to the same BSP (48 lumps, `bsp_lumps.py diff`) and `.prt` as the main checkout's build.

Not covered: nothing was looked at in game. The floor model counts roofs and ground outside the playable area. A viewpoint model built on bspwalk's reachable space would be better. Ten of the twelve maps were measured at one grid position.

A trap that cost one round here: counting every upward-facing face as floor. Sky, nodraw and water are 40 to 80 % of the upward face area of an HL2 map (the sky box alone is 79 % on d1_trainstation_03), and with them in, the floor numbers followed the sky the same way the volume numbers follow the largest leaf. The first set of tables from this work had that error; the ones below do not.

## Findings

### 1. A coarser grid never tightens the PVS; it loosens it on open maps that have occluders, and nowhere else

2048 against 1024, floor-weighted. d1_trainstation_03 and d2_coast_03 are means over six and four grid positions per size; the rest are one position.

| map | clusters | ground seen | static props in view | world triangles in view | vvis s |
|---|---|---|---|---|---|
| d1_trainstation_03 | 971 -> 956 | +1.9 % | +0.5 % | +0.9 % | 3 to 30 at both sizes |
| d2_coast_03 | 1946 -> 1050 | **+5.1 %** | **+13.2 %** | -5.8 % | 30 -> 8 |
| gm_fightspace_redux | 3737 -> 1743 | +1.8 % | +1.2 % | -2.1 % | 52 -> 10 |
| d1_canals_07 | 1229 -> 869 | +2.8 % | **+9.3 %** | +3.0 % | 6 -> 1 |
| d1_canals_11 | 1126 -> 806 | **+5.9 %** | **+6.6 %** | +1.9 % | 4 -> 1 |
| d1_town_01 | 1278 -> 1213 | +0.8 % | +2.2 % | +0.7 % | 14 -> 10 |
| d1_eli_01 | 1420 -> 1353 | +1.4 % | +1.7 % | -5.0 % | 49 -> 27 |
| d2_coast_07 | 1284 -> 765 | +2.4 % | +1.1 % | -18.5 % | 31 -> 6 |
| d2_coast_11 | 1277 -> 808 | -2.0 % | +0.5 % | -17.9 % | 1015 -> 50 |
| d2_prison_05 | 1407 -> 1381 | +0.2 % | +2.1 % | -9.3 % | 13 -> 39 |
| d3_c17_07 | 694 -> 719 | +0.2 % | -0.3 % | +0.4 % | 15 -> 49 |
| d3_citadel_03 | 517 -> 438 | +2.9 % | +1.6 % | +2.3 % | under 1 |

The spread from grid position alone is about 3 % on these numbers (trainstation_03: ground seen 36.3 to 37.4 % at 1024 and 36.3 to 38.6 % at 2048; props 35.6 to 38.3 at both). Read against that, nine maps show no difference between 1024 and 2048, and three open maps (d2_coast_03, both canals maps) show 7 to 13 % more props in view at 2048. On d2_coast_03 the two sizes do not overlap at any position: props 33.3 to 34.1 at 1024, 37.5 to 38.5 at 2048.

4096 against 1024 widens the same three (props +33 %, +17 %, +11 %) and leaves the rest within 7 %.

The whole ladder on d2_coast_03 (137 static props):

| grid | clusters | ground seen | props in view | world triangles in view | world triangles in the map | vvis s |
|---|---|---|---|---|---|---|
| 512 | 4873 | 28.6 % | 31.3 | 4999 | 52,574 | 495 |
| 1024 | 1857 to 1988 | 29.7 to 30.4 % | 33.3 to 34.1 | 3547 to 3641 | 43,263 to 43,715 | 25 to 33 |
| 2048 | 1015 to 1086 | 30.5 to 32.3 % | 37.5 to 38.5 | 3279 to 3493 | 40,767 to 40,964 | 6 to 12 |
| 4096 | 832 | 34.9 % | 44.7 | 3804 | 40,142 | 5 |
| off | 676 | 39.1 % | 65.9 | 4598 | 40,042 | 4 |

gm_fightspace_redux (138 static props, no visclusters, `-mergeflow`). The grid buys little here, because over half the ground sees itself whatever the grid:

| grid | clusters | ground seen | props in view | world triangles in view | world triangles in the map | vis lump | vvis s |
|---|---|---|---|---|---|---|---|
| 1024 | 3737 | 56.3 % | 74.6 | 61,417 | 152,221 | 3.20 MB | 52 |
| 2048 | 1743 | 57.3 % | 75.5 | 60,100 | 147,909 | 0.71 MB | 10 |
| 4096 | 1164 | 59.5 % | 78.4 | 61,271 | 146,869 | 0.32 MB | 6 |
| off | 799 | 57.3 % | 79.5 | 60,488 | 146,533 | 0.15 MB | 4 |

512 against stock 1024:

| map | clusters | ground seen | props in view | world triangles in view | vvis s |
|---|---|---|---|---|---|
| d1_trainstation_03 | 1010 -> 1204 | -1.7 % | -3.3 % | +9.7 % | 7 -> 34 |
| d2_coast_03 | 1857 -> 4873 | -6.1 % | -8.2 % | +38.1 % | 25 -> 495 |
| d1_canals_07 | 1229 -> 2644 | -9.3 % | -8.8 % | +13.2 % | 6 -> 45 |
| d1_canals_11 | 1126 -> 2199 | -8.8 % | -6.4 % | +12.2 % | 4 -> 272 |
| d1_town_01 | 1278 -> 1490 | -2.0 % | -1.7 % | +0.8 % | 14 -> 22 |
| d3_c17_07 | 694 -> 886 | -9.0 % | -3.9 % | -2.3 % | 15 -> 46 |

### 2. A coarser grid means fewer world triangles, because it cuts fewer faces

Every grid line cuts the faces it crosses. d2_coast_03 has 43,263 world triangles at 1024, 40,840 at 2048 and 52,574 at 512. So fewer land in any PVS at 2048 even where the PVS covers more of the map: on d2_coast_03 the 2048 grid has 6 % fewer world triangles in view and 13 % more static props. The saving is largest on the two maps with big flat faces and open sight (d2_coast_07 and d2_coast_11, 18 % each, 1000 to 2600 triangles) and is the real part of what #119 saw. The other part was the volume weighting: the -4.7 % on d1_trainstation_03 is +0.9 % from the ground.

### 3. Where the lines fall decides vis time on dense maps

d1_trainstation_03, the stock grid and the 2048 grid at six positions each:

| grid, offset | clusters | ground seen | props in view | world triangles in view | volume-weighted triangles | vvis s |
|---|---|---|---|---|---|---|
| 1024 | 1010 | 36.34 % | 36.8 | 2634 | 753 | 7 |
| 1024 +256,256 | 946 | 36.57 % | 36.6 | 2639 | 732 | 6 |
| 1024 +512,0 | 1013 | 37.40 % | 38.3 | 2785 | 789 | 7 |
| 1024 +0,512 | 972 | 36.52 % | 36.7 | 2649 | 766 | 25 |
| 1024 +512,512 | 956 | 36.91 % | 37.9 | 2730 | 774 | 25 |
| 1024 +768,768 | 930 | 37.00 % | 35.6 | 2674 | 757 | 30 |
| 2048 | 991 | 36.27 % | 36.7 | 2738 | 718 | 30 |
| 2048 +512,512 | 951 | 37.39 % | 37.7 | 2777 | 739 | 30 |
| 2048 +1024,0 | 934 | 37.81 % | 38.3 | 2784 | 717 | 21 |
| 2048 +0,1024 | 1001 | 37.33 % | 36.7 | 2677 | 723 | 7 |
| 2048 +1024,1024 | 917 | 37.50 % | 36.1 | 2582 | 696 | 3 |
| 2048 +1536,1536 | 939 | 38.55 % | 37.6 | 2703 | 738 | 27 |
| 512 | 1204 | 35.72 % | 35.6 | 2890 | 986 | 34 |
| 4096 | 979 | 36.21 % | 36.4 | 2723 | 719 | 36 |
| off | 1039 | 37.82 % | 37.7 | 2855 | 866 | 38 |

- Vis time is 3 to 7 s or 21 to 38 s at every size. Which one follows the y position of the lines (1024 with y offset 0 or 256 and 2048 with y offset 1024 are fast, the rest slow), so one spot in the map decides it. The "2 to 4 times slower at 2048" in #96 and #119 is this, not the block size: three of the six 1024 positions are as slow as stock 2048, and two 2048 positions are faster than stock 1024. Filed as #135.
- The same lottery shows on the other maps whose cluster count barely moves with the grid: d2_prison_05 13 -> 39 s, d3_c17_07 15 -> 49 s. Where the grid makes most of the clusters, a coarser grid is reliably faster (d2_coast_03, d2_coast_07, both canals maps, fightspace). d2_coast_11 takes 1015 s at stock 1024 and 50 s at 2048 with the exact flow; one position each, not explained.
- On an indoor map the grid size hardly matters to what is seen: ground seen stays within 35.7 to 38.6 % from 512 to off.
- Moving the lines can break an areaportal. d2_coast_03 at 1024 with x offset 256 compiles with "Areaportal leak" (two areaportal brushes no longer touch two areas); the stock position and the other seven shifted grids do not, and neither does any 512, 2048, 4096 or off compile of any map here. The PVS does not depend on areas, so that row's numbers stand, but it is a reason to treat `-blockoffset` as a measuring aid and to read the vbsp log after changing the grid on a map with areaportals. The sweep marks such a row. (Only x offsets fit on d2_coast_03: the map reaches the edge of the coordinate range in y.)

## What this means

- **#119**: keep 1024 as the default. Against 1024, 2048 is the same on nine maps and shows 7 to 13 % more props on the three open maps where the grid does work; it saves up to 18 % of the world triangles in view, a few thousand at most; and it cuts vis time 3 to 20 times on open maps. That makes it a per-map compile setting. As a default it would change the cuts of every existing map for no gain in what is drawn.
- **#105 and adaptive grids**: the one thing a rule has to tell apart is open space that occluders break up (d2_coast_03, where the grid halves the props in view) from open space where most things see each other anyway (fightspace, where it is worth 6 %). That is a fact about visibility. Brush counts, leaf sizes and densities do not carry it, and indoor maps do not care about the grid in either direction. Getting it before the BSP exists means writing a second, approximate vis. Getting it afterwards means running the vis the rule was meant to save. Within one map, `func_viscluster` already is the per-region tool.
- **A per-map choice gets what there is to get.** For a map like fightspace, 2048 costs nothing that can be measured (vis 5 times faster, half the clusters), and `-blocksize 0` is defensible (vis 13 times faster, 4.7 times fewer clusters, a vis lump 20 times smaller, 7 % more props in view). For a map like d2_coast_03, 1024 stays, and 512 is there for a mapper who wants 6 to 9 % less in view and accepts 2 to 70 times the vis time and more cut faces.
- **A picker script** should rank by props and ground in view, not triangles; walk from coarse to fine, because the coarse compiles are the cheap ones (d2_coast_03: 4, 5, 8, 30 s); allow for about 3 % of position noise before calling one size better; and store its choice beside the VMF, because the search costs more vis time than it saves on any single compile.
