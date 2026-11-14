# func_viscluster: optional, and what was tried in its place (#100)

Measured 2026-11-14 at fc306f0f on `fightspace3_unittest` (open map, 208 hand-placed `func_viscluster`) and stock `d1_trainstation_03`, `d2_coast_03`, `d2_prison_01`. Every compile used `-game <SDK Base 2013 Multiplayer>/hl2mp`; vis is `vvis_optix -mergeflow` unless a row says exact. Wall times are from a loaded 16C/32T box.

## Guidance

- `func_viscluster` is optional. It existed so that VVIS could finish on open maps; with `vvis_optix -mergeflow` a viscluster-less open map finishes in about a minute.
- Dropping visclusters does not cost frame time on the map measured: expected triangles in the PVS go down slightly (86,254 with, 83,192 without).
- What it costs is vis lump size, which grows with the square of the cluster count: 45 KB with visclusters, 3.67 MB without. The format limit is 16 MB (`MAX_MAP_VISIBILITY`).
- For an open map, compile with `vbsp_lmo -blocksize 0` (#96). That alone takes the viscluster-less map to 848 clusters and a 175 KB lump, vis takes 4 s, and the exact flow finishes (403 s), which it does not on the hand-visclustered compile within 15 minutes. Do not use `-blocksize 0` on indoor maps: #96 measured about 15% more drawn there.
- Exact-flow time follows portal count, not cluster count: 421 clusters with 4,076 portals times out, 848 clusters with 2,614 portals finishes.

## Fightspace variants

| Variant | Clusters | Portals | Vis s | Vis lump B | Expected triangles | Worst triangles |
|---|---|---|---|---|---|---|
| 208 hand visclusters | 421 | 4,076 | 15 | 44,901 | 86,254 | 89,948 |
| none | 3,981 | 10,726 | 57 | 3,667,683 | 83,192 | 85,842 |
| none, `-blocksize 0` | 848 | 2,614 | 4 | 174,612 | 81,855 | 83,287 |

Expected and worst triangles are from `game/bin/x64/pvs_cost.py` (run it without `-I`).

## Option 1, tried: merge clusters with like PVS rows after vis

Not built into VVIS. Measured with `.agents/scripts/vis_row_merge_stats.py`, whose size model reproduces the real lump byte for byte on the two fightspace compiles.

- Identical rows only (lossless): the lump shrinks 1 to 12%. Fightspace without visclusters 3,667,683 to 3,292,667 B, and 170 of the 298 clusters removed are one group. Not worth a pass.
- With a tolerance (a cluster joins a group whose union row differs from its own by at most t of its set bits; the group sees the union), viscluster-less fightspace:

| t | Clusters | Vis lump B | Leaf pairs visible |
|---|---|---|---|
| 0 | 3,683 | 3,292,667 | +0% |
| 0.01 | 2,284 | 1,232,124 | +0.9% |
| 0.02 | 1,582 | 574,992 | +2.8% |
| 0.05 | 880 | 170,908 | +8.7% |
| 0.15 | 448 | 44,930 | +20.1% |

- At the hand-placed map's lump size the automatic merge costs +20.1% leaf pairs; the 208 hand-placed brushes cost +23.2%. So a tolerance merge is at least as good as the manual work, but `-blocksize 0` gets within 2% of the t = 0.05 size (174,612 B) with no loss and a faster vis.
- Indoor maps have small lumps to begin with (178 to 565 KB); t = 0.02 saves 20 to 46% of that for +0.3 to +1.0% leaf pairs.
- If a map ever nears the 16 MB limit and cannot use `-blocksize 0`, this is the pass to build. It would rewrite `dleaf_t.cluster`, the vis lump and `dworldlight_t.cluster`; VRAD and bspwalk already handle many leaves per cluster (func_viscluster produces them).

## Option 2, tried and reverted: automatic visclusters in VBSP

`vbsp_lmo -autoviscluster [maxsize]` is at commit 49996fc6 and reverted by the commit after it. It merged every BSP subtree of open space (one area, one contents value, no hint plane, solid leaves under 10% of the volume) into one cluster.

- The rule as first designed, subtrees with no solid leaf at all, matches nothing: every split plane comes from a brush side, so every node has a solid leaf below it.
- Fightspace without visclusters: 3,981 to 3,122 clusters, lump 2.21 MB. Size caps of 1024 to 4096 units leave 3,756 to 3,845. Allowing any solid fraction still leaves 2,937.
- On top of `-blocksize 0`: 848 to 581 clusters, lump 84 KB, exact flow 403 s to 112 s. This is the one place it paid.
- HL2 maps: 6 to 24% fewer clusters for 0.8% (prison), 5% (trainstation) and 6% (coast) more expected triangles.
- The merged compiles lost a few leaf pairs against the unmerged ones (0 to 4,374 of up to 9.95 million; 0 to 4 under the exact flow). The ones looked at were sight lines grazing leaves 12 to 15 units thick. Whether merging or the renumbering alone causes them was not established.
- Dropped because the gain is small beside `-blocksize 0`, it costs culling indoors, and it is 244 lines in VBSP.

## Found on the way

- `node_t::side` in VBSP points into a brush that `BuildTree_r` has freed. Stock code only reads it in a warning. Do not use it to tell what made a split; the experiment compared node planes with the hint sides' planes instead.
- VVIS's "Total clusters visible" log line counts a cluster's own bit twice when its portals see back into it, which happens on visclustered maps (`ClusterMerge`, `vvis.cpp`). The stored PVS is right.
