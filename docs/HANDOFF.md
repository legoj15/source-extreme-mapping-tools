# Handoff, 2026-11-11 (wave 1C: static prop fixes)

Branch `claude/wave1-staticprop-fixes-f0d7f2`, not merged, not pushed. One commit per issue, then review fixes.

## Done
- #70 (560d2161): LoadStudioModel runs Proplm_ValidateMdl before IsStaticProp; GetCollisionModel checks the VVD (id, version, checksum, finite verts, fixups, mesh ranges) before the hull. A NaN VVD vertex crashed (rc 139) and a misaligned studiohdr2index Errored (rc 1); both now warn, drop the prop, rc 0. ValidateMdl also checks mesh->model back-pointers, the name field, keyvalue text and (9ab43055) the bone table.
- #71 (1b893667): a failed decompile now falls through to flag patching. Checked with a flag-cleared fridge and no studiomdl: pak gains the patched MDL, VVD, VTX, PHY.
- #108 (b4aeba6e): real. 400 concentric nodraw rings with doorways at -blocksize 0 give a 1206-deep tree; old build died 0xC0000409, new one places the prop. Node path is a CUtlVector, plane list on the heap. d1_trainstation_03 is 27 deep at -blocksize 0; a chain of 1100 water brushes only 17 (brush order is not spatial).
- #62 (051a7d56): WriteSMDFromModel uses the shared loader. SMDs byte-identical for fridge, train001, combine_interface001, door01_left. trainstation_02_hrcs sprp unchanged apart from offsets 31 and 44-55, which also differ base-vs-base (#106); plm2 identical. VRAD left alone: its walkers use the SDK's Studio_LoadVertexes, not a copy.
- #63 (680a2d39): ParseGameLump inflates LZMA sub-lumps (bounded by the game lump); LoadBSPFile Errors clearly on whole-lump compression. Stock `bspzip -repack` compresses nothing; `-repack -compress` compresses every lump, which no tool here loads. With only plm2 compressed (stock blob spliced), old VRAD said "version 1095588428", new one bakes the same VTF as the plain map.
- Proplm suite (uv2, rebake, vrad) passes on the final build.

## Open / follow-up candidates (not filed)
- VBSP's CacheVertexData keeps the raw VVD and the stock hull reads it without fixups, so models with VVD fixups get their per-mesh hulls from the wrong vertices (pre-existing, traced by the Haiku reviewer, not run).
- ComputeConvexHull's stackalloc(numvertices * 8) is unbounded (needs a corrupt or >131k-vertex mesh).

## Work files (scratchpad, not kept)
Fuzz game dirs, ring/chain VMF generators, stand-in studiomdl for SMD capture, spliced compressed-plm2 BSP. Recipes are in the commit messages.

## Delegation log
- reviewer (Haiku) on 1420cff7..680a2d39: pass with nits; one real gap (bone table), one stale doc; one misread (bRequireTrilists is false in the SMD writer). Fixed in 9ab43055.
- review_fast on 1420cff7..680a2d39: glm-flash only (shop26 busy, deepseek-flash stand-in failed to launch), 3 lows, two overlapping Haiku's; all fixed (9ab43055, 5a9d8206).
- review_fast on 680a2d39..5a9d8206 (final sha): shop26 (qwen3.8-27b) approve with two notes, glm-flash approve, no findings. Round closed.
- Implementation by the orchestrator. No escalations.

---

# Handoff, 2026-11-09 (VBSP visibility session)

## Done
- Surveyed what VBSP can do for in-game performance and PVS quality; filed #93-#105.
- #96 shipped and closed: `vbsp_lmo -blocksize N` (power of two 1024..16384) and `-blocksize 0` (no grid). Merged to master as e7ffd982 and pushed. Default output is byte-identical to the previous build (.prt, d1_trainstation_03).
- `game/bin/x64/pvs_cost.py`: volume-weighted expected leaves/faces/triangles in a BSP's PVS, for comparing compiles that cut space differently.
- Measurements are in the #96 comments. Short form: on open gm_fightspace_nocl the grid buys nothing (off: 4.7x fewer clusters, 20x smaller vis lump, same triangles in the PVS); on indoor d1_trainstation_03 off is about 15% worse than stock and 2048-4096 about 5% better.

## In progress
- Nothing mid-edit. Master and the worktree branch `claude/vbsp-visibility-alternatives-69ac36` are clean apart from this file.

## Next
1. #108 (new, from the final review): the static prop leaf walk has a fixed 1024-deep node buffer that a -blocksize 0 tree could exceed. Unverified static claim; verify, measure real tree depths, fix.
2. #93: func_viscluster picks the lowest-indexed overlapping cluster (confirmed by reading; one-line fix, changes PRT on maps with overlapping visclusters).
3. #105: adaptive block grid, the follow-up the #96 numbers argue for.
4. #97: automatic areaportals at portal-graph chokepoints (first real feature; start with an offline .prt prototype).

## Final review of #96 (ran after the merge)
- glm-5.3: no blocking defect. Two notes, not acted on: `-blocksize 0 -noprune` emits the fragmented exterior and prints "node without a volume" warnings; `-blocksize 00` / `+0` are rejected with a message that lists 0 as valid.
- kimi (stand-in, dgx was busy): one high, filed as #108.

## Open questions
- No -blocksize variant has been looked at in game.
- Exact vvis on d1_trainstation_03 went from 6 s at 1024 to 15-26 s at every other size, repeatably. Not investigated.
- #94 (hint gate reads only the last brush tested) is still an unverified lead.
- #100: what replaces func_viscluster once viscluster-less vis is fast. Decision for the user.
- pvs_cost.py does not count static props.

## Environment notes
- From our bin, `-game` pointing at `Half-Life 2/hl2` or `Source SDK Base 2013 Singleplayer/hl2` fails with "Can't load MaterialSystem.dll" (same symptom as #90). `Source SDK Base 2013 Multiplayer/sourcetest` works.
- Pass the `.vmf` extension explicitly when the path has a dotted directory such as `.claude`; without it vbsp cannot open the file.
- Master's generated .vcxproj files were stale after the -detailclip merge (link errors); `createallprojects.bat` was re-run in the main checkout on 2026-11-09.
- Work files: `.claude/worktrees/vbsp-visibility-alternatives-69ac36/game/bin/x64/unit_test/unit_test_output/vbsp_blocksize/` (ignored).

## Delegation log
- scout (VBSP survey): ok. reviewer (Haiku): one valid high on round 1. review_fast x2, review_final x1: see #96 comments. Implementation by the orchestrator. No escalations.

---

# Handoff, 2026-11-09 (VBSP detail prop clipping session, #92)

## Done
- #92 shipped and closed: `vbsp_lmo -detailclip` / `-detailclip_shrink <f>` (default 0.5) drops auto-placed detail props buried in, or whose shrunk box overlaps, visible solid world/func_detail brushes or static prop LOD0 triangles. Displacement brushes are excluded. Flag off = unchanged output; flag on only removes props. Merged to master as 1b2aebd5, **not pushed**. Design and limits: docs/detailclip.md.
- Follow-ups filed: #106 (detail/static prop lumps not byte-reproducible: unset padding), #107 (displacement terrain as an obstacle).

## Final review of #92 (finished after the stop)
- glm-5.3: approve. dgx: comment, no blocking defect. Both confirm flag-off identity and the subset invariant; glm-5.3 ported the SAT and grid to Python and matched 16 scenarios plus 12,000 random queries.
- Low findings, not yet acted on (all flag-on only):
  - Sprites/billboards on downward-facing surfaces (ceilings) are never overlap-tested: the cut removes the whole upright box. Fall back to the uncut box when nothing survives the cut.
  - Copy `normal` before the DetailClips call in PlaceDetail; the old BUGBUG comment says the optimizer trashes it there (measured runs suggest it is stale).
  - Brush edges under 0.05 units are left out of the SAT axes, so slivers can cause false removals. Add the direction anyway.
  - Sky brushes count as obstacles; document or carve out.
- Notes only: axis dedup tolerance, IDAG models skipped, summary candidate count excludes invalid models.

## Follow-up session, 2026-11-10
- The four lows are fixed: 158f550f (uncut-box fallback, sliver edge axes, `normal` copy, sky brushes documented as obstacles on purpose).
- Haiku review of 158f550f found a real major: sprites were always tested as a world-upright box, but the engine draws orientation-0 sprites in the lump-angle frame, so a ceiling sprite hangs down. The fallback then removed them. Fixed in 25a9e71a: orientation-0 sprites now get a framed square box; screen-aligned sprites keep the upright box. Hanging shapes now shrink toward their top.
- Checked: fightspace removes 2047 (was 2041) and the subset still holds. A grass floor+ceiling test room removes all screen-aligned ceiling sprites and keeps 165/180 hanging ones. The generator is not kept; it's a 40-line scratch script.
- vbsp_lmo.exe in game/bin/x64 is rebuilt from master (has -detailclip).
- Delegation: reviewer (Haiku) found the orientation major (the two fast-gate lanes missed it); review_fast on 158f550f: shop26 + glm-flash, lows only (fallback reaches sunk shapes on floors, now documented).

## Next
1. Push master when the user says so (#96, #92 and these follow-ups all sit on master).

## Open questions
- -detailclip not yet checked in game.
- Worktree `claude/vbsp-detail-prop-collision-95559d` is merged and clean; can be archived.
