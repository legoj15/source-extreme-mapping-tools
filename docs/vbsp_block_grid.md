# VBSP block grid: what it buys and what it costs

Second look at #96, #105 and #119, measured 2026-11-14 on the dev box (shared with other sessions: read vis times to a factor of two). Branch `claude/issues-105-119-review-fe906e`.

## How it was measured

- `.agents/scripts/blockgrid_sweep.py`: one compile, one vvis run and one `pvs_cost.py` measurement per map and grid variant. `--remeasure` re-reads the BSPs a run left behind.
- `game/bin/x64/pvs_cost.py` now has a second viewpoint model and counts static props.
  - **Floor-weighted** (`floor_*`): the viewpoint is uniform over upward-facing world faces. `floor_seen_pct` is the share of the map's floor in the PVS. This is the number to read.
  - **Volume-weighted** (`expected_*`): the model the #96 and #105 tables used. It follows the largest leaf. d2_prison_05 reads 90 % of its volume visible while its mean cluster sees 14 % of the clusters, and its volume-weighted triangles fall 41 % between 1024 and 2048 with no change in what the floor sees.
  - Static props are counted as props in view, not triangles. Fade distances are ignored.
- `vbsp_lmo -blockoffset X Y` moves the grid lines. Running one size at several offsets gives the spread that comes from where the lines fall, with nothing else changed.
- `-blocksize 512` is now accepted (it was 1024 and up).
- Exact vvis on the HL2 maps (HL2-MapPack VMFs), `-mergeflow` on gm_fightspace_redux with its func_viscluster entities stripped. `-game` is the SDK Base 2013 Multiplayer `hl2mp`.
- The stock grid is unchanged by these edits: d1_trainstation_03 and d2_coast_03 compile to the same BSP (48 lumps, `lumpdiff.py`) and `.prt` as the main checkout's build.

Not covered: nothing was looked at in game. The floor model counts roofs and ground outside the playable area. A viewpoint model built on bspwalk's reachable space would be better.

## Findings

### 1. A finer grid gives a tighter PVS on every map. The size of the effect is the map-dependent part

2048 against 1024, floor-weighted. d1_trainstation_03 and d2_coast_03 are means over six and four grid positions per size; the rest are one position.

| map | clusters | floor seen | static props in view | world triangles in view | vvis s |
|---|---|---|---|---|---|
| d1_trainstation_03 | 971 -> 956 | +0.9 % | -0.3 % | -1.7 % | 3 to 30 at both sizes |
| d2_coast_03 | 1947 -> 1050 | **+25.9 %** | **+25.2 %** | -4.3 % | 30 -> 8 |
| gm_fightspace_redux | 3737 -> 1743 | +0.7 % | +0.7 % | -2.4 % | 52 -> 10 |
| d1_canals_07 | 1229 -> 869 | +3.6 % | +9.2 % | -0.7 % | 6 -> 1 |
| d1_canals_11 | 1126 -> 806 | +6.0 % | +6.0 % | +1.5 % | 4 -> 1 |
| d1_town_01 | 1278 -> 1213 | +1.5 % | +1.3 % | +1.6 % | 14 -> 10 |
| d1_eli_01 | 1420 -> 1353 | +0.6 % | +1.5 % | -5.4 % | 49 -> 27 |
| d2_coast_07 | 1284 -> 765 | +8.7 % | +3.8 % | -17.2 % | 31 -> 6 |
| d2_coast_11 | 1277 -> 808 | +3.4 % | +1.7 % | -18.5 % | 1015 -> 50 |
| d2_prison_05 | 1407 -> 1381 | 0.0 % | +1.6 % | -20.4 % | 13 -> 39 |
| d3_c17_07 | 694 -> 719 | +4.7 % | +0.9 % | +4.5 % | 15 -> 49 |
| d3_citadel_03 | 517 -> 438 | +2.9 % | +1.6 % | +2.3 % | under 1 |

2048 shows more of the floor from the average spot on eleven of twelve maps and the same on the twelfth. It is never tighter.

The whole ladder on d2_coast_03 (137 static props):

| grid | clusters | floor seen | props in view | world triangles in view | world triangles in the map | vvis s |
|---|---|---|---|---|---|---|
| 512 | 4873 | 27.7 % | 32.2 | 5932 | 52,574 | 495 |
| 1024 | 1857 to 1988 | 30.6 to 30.7 % | 35.6 to 35.8 | 3742 to 3821 | 43,263 to 43,715 | 25 to 33 |
| 2048 | 1015 to 1086 | 38.5 to 38.8 % | 44.4 to 44.9 | 3598 to 3637 | 40,767 to 40,964 | 6 to 12 |
| 4096 | 832 | 45.6 % | 53.2 | 4035 | 40,142 | 5 |
| off | 676 | 72.4 % | 91.7 | 6601 | 40,042 | 4 |

gm_fightspace_redux (138 static props, no visclusters, `-mergeflow`). Here the grid buys almost nothing, because four fifths of the floor already sees itself:

| grid | clusters | floor seen | props in view | world triangles in view | world triangles in the map | vis lump | vvis s |
|---|---|---|---|---|---|---|---|
| 1024 | 3737 | 80.5 % | 88.6 | 73,913 | 152,221 | 3.20 MB | 52 |
| 2048 | 1743 | 81.0 % | 89.2 | 72,139 | 147,909 | 0.71 MB | 10 |
| 4096 | 1164 | 82.3 % | 91.1 | 72,662 | 146,869 | 0.32 MB | 6 |
| off | 799 | 82.7 % | 92.1 | 72,785 | 146,533 | 0.15 MB | 4 |

512 against 1024:

| map | clusters | floor seen | props in view | world triangles in view | vvis s |
|---|---|---|---|---|---|
| d1_trainstation_03 | 1010 -> 1204 | -1.6 % | -4.9 % | +15.4 % | 7 -> 34 |
| d2_coast_03 | 1857 -> 4873 | -9.8 % | -9.6 % | +56.9 % | 25 -> 495 |
| d1_canals_07 | 1229 -> 2644 | -3.5 % | -7.2 % | +31.5 % | 6 -> 45 |
| d1_canals_11 | 1126 -> 2199 | -4.9 % | -4.6 % | +11.4 % | 4 -> 272 |
| d1_town_01 | 1278 -> 1490 | -2.4 % | -2.9 % | -1.3 % | 14 -> 22 |
| d3_c17_07 | 694 -> 886 | -8.9 % | -8.8 % | -2.5 % | 15 -> 46 |

### 2. "2048 has fewer triangles in the PVS" was the face count, not visibility

Every grid line cuts the faces it crosses. A coarser grid leaves fewer world triangles in the map (d2_coast_03: 43,263 at 1024, 40,840 at 2048, 52,574 at 512), so fewer land in any PVS even when the PVS covers more of the map. On d2_coast_03 the 2048 grid has 4 % fewer world triangles in view and 25 % more static props and floor. The numbers behind #119 (-4.7 %, -3.4 %, -2.5 %) are this effect plus the volume weighting. World triangles in view are 1,700 to 14,000 on the HL2 maps, so the saving is small in absolute terms as well.

### 3. Where the lines fall moves the old triangle metric by several percent, and decides vis time on dense maps

d1_trainstation_03, the stock grid and the 2048 grid at six positions each:

| grid, offset | clusters | floor seen | props in view | world triangles in view | volume-weighted triangles | vvis s |
|---|---|---|---|---|---|---|
| 1024 | 1010 | 35.56 % | 26.8 | 1723 | 753 | 7 |
| 1024 +256,256 | 946 | 35.10 % | 25.4 | 1668 | 732 | 6 |
| 1024 +512,0 | 1013 | 35.74 % | 27.2 | 1796 | 789 | 7 |
| 1024 +0,512 | 972 | 35.66 % | 26.8 | 1732 | 766 | 25 |
| 1024 +512,512 | 956 | 36.00 % | 27.4 | 1792 | 774 | 25 |
| 1024 +768,768 | 930 | 36.06 % | 26.1 | 1758 | 757 | 30 |
| 2048 | 991 | 35.90 % | 25.9 | 1743 | 718 | 30 |
| 2048 +512,512 | 951 | 36.05 % | 26.3 | 1762 | 739 | 30 |
| 2048 +1024,0 | 934 | 36.07 % | 26.4 | 1713 | 717 | 21 |
| 2048 +0,1024 | 1001 | 35.84 % | 26.9 | 1715 | 723 | 7 |
| 2048 +1024,1024 | 917 | 36.29 % | 26.8 | 1673 | 696 | 3 |
| 2048 +1536,1536 | 939 | 35.85 % | 27.0 | 1688 | 739 | 27 |
| 512 | 1204 | 34.99 % | 25.5 | 1989 | 986 | 34 |
| 4096 | 979 | 36.36 % | 25.9 | 1743 | 719 | 36 |
| off | 1039 | 36.56 % | 27.4 | 1906 | 867 | 38 |

- Vis time is 3 to 7 s or 21 to 38 s at every size. Which one follows the y position of the lines (1024 with y offset 0 or 256 and 2048 with y offset 1024 are fast, the rest slow), so one spot in the map decides it. The "2 to 4 times slower at 2048" in #96 and #119 is this, not the block size: three of the six 1024 positions are as slow as stock 2048, and two 2048 positions are faster than stock 1024.
- The same lottery shows on the other maps whose cluster count barely moves with the grid: d2_prison_05 13 -> 39 s, d3_c17_07 15 -> 49 s. Where the grid makes most of the clusters, a coarser grid is reliably faster (d2_coast_03, d2_coast_07, both canals maps, fightspace). d2_coast_11 takes 1015 s at stock 1024 and 50 s at 2048 with the exact flow; one position each, not explained.
- On an indoor map the grid size hardly matters to what is seen: floor seen stays within 35.0 to 36.6 % from 512 to off.
- On d2_coast_03 the x position of the lines moves floor seen by 0.3 % and props by 0.5 %, against 25 % between sizes. (Only x offsets fit: the map reaches the edge of the coordinate range in y.)

## What this means

- **#119**: keep 1024 as the default. 2048 is a compile-time setting (3 to 20 times faster vis on open maps), paid for with a looser PVS on every map measured.
- **#105 and adaptive grids**: the one thing a rule has to tell apart is open space that occluders break up (d2_coast_03, where the grid is worth 25 % per halving) from open space where everything sees everything (fightspace, where it is worth under 3 % in total). That is a fact about visibility. Brush counts, leaf sizes and densities do not carry it, and indoor maps do not care about the grid in either direction. Getting it before the BSP exists means writing a second, approximate vis. Getting it afterwards means running the vis the rule was meant to save. Within one map, `func_viscluster` already is the per-region tool.
- **A per-map choice gets what there is to get.** For a map like fightspace, `-blocksize 0` or 4096 is a sound final setting: vis 13 times faster, 4.7 times fewer clusters, a vis lump 20 times smaller, 4 % fewer world triangles in the map, for 3 % more floor and 4 % more props in view. For a map like d2_coast_03, 1024 stays, and 512 is there for a mapper who wants a tighter PVS and accepts the vis time.
- **A picker script** should rank by floor seen and props in view, not triangles; walk from coarse to fine, because the coarse compiles are the cheap ones (d2_coast_03: 4, 5, 8, 30 s); and store its choice beside the VMF, because the search costs more vis time than it saves on any single compile.
