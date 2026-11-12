# Handoff, 2026-11-11 (wave 1A: VBSP small fixes, branch claude/wave1-vbsp-fixes-4262d5)

## Done (on the branch, not merged, not pushed)
- #106 47723de1: detail and static prop lump entries are zeroed before filling. Before: two runs of one exe on fightspace3_unittest differed in 67 dprp and 152 sprp bytes. After: every lump identical.
- 060c85c1 (refs #106): the default HDR cubemap's spheremap face is zeroed after GenerateSpheremap (prebuilt vtf.lib leaves uninitialised texels). Old build: 7 of 20 runs of a sky map put 355 or 1408 garbage bytes in cubemapdefault.hdr.vtf (pak lump). New: 0 of 20. With both fixes VBSP output is lump-for-lump reproducible, which #103 depends on.
- #93 157d009f: GetVisCluster keeps the largest-overlap func_viscluster (was the lowest-indexed one over 10%). Changes the .prt of maps where a leaf is covered >10% by two visclusters. Checked on a hint-cut test room: the cluster portal moved from x=696 to x=304 as predicted.
- #95 685c401b + 6efdea39: value options are bounds-checked (and never take the map file), usage text matches the parser, -dumpstaticprops accepted, -threads and -tmpout warn and are ignored, dead subdivide code deleted.
- #94 d788739e: confirmed (TestBrushToPlanenum clears *hintsplit on entry, so the gate saw only the last brush). The OR-fix is opt-in as `-hintprotect` because it measured no better: trainstation_03 and gm_fightspace_redux byte-identical, trainstation_05 neutral, d2_prison_01 +18 clusters, +5.6% faces, +0.7% expected triangles and 9 new FindPortalSide warnings. Default output unchanged.
- #107 5bb10d09 (impl, Sonnet): -detailclip tests candidates against displaced triangles (from mapdispinfo, so -onlyprops works), skipping the candidate's own displacement. Flag off: identical. Flag on: subset of the old output. The old "displacement brushes excluded" note in the #92 section below is now only a guard (the loader already drops displacement parent brushes).

## Follow-ups filed
- #111 -onlyprops places ~half the auto detail candidates of a full compile (pre-existing).
- #112 -detailclip seam false positives against sewn neighbour displacements.
- #113 detail model names of 128+ chars are written unterminated into the dict lump.
- Not filed (outside VBSP): vrad's usage text describes vbsp's -threads (vrad.cpp ~3514); Stripify in src/utils/common/mstristrip.cpp has no caller.

## Environment notes
- HL2 VMFs: E:/legoj/Documents/SourceModding/HL2-MapPack; gm_fightspace_redux: E:/legoj/Documents/SourceModding/gmod projects/gm_fightspace_redux/mapsrc. Compiled with -game Source SDK Base 2013 Multiplayer/hl2mp.
- Exact vvis on gm_fightspace_redux (3737 clusters) ran over 4 h without finishing; use vvis_optix -mergeflow (45 s).
- fightspace3_unittest under plain hl2mp has no detail props (its .vbsp and legoj15 materials are missing); the #107 numbers used a scratch game dir giving nature/blenddirtgrass001a a %detailtype.
- The Bash tool processes backslashes inside quoted heredocs; write Python edit scripts with the Write tool instead.

## Delegation log
- impl (Sonnet) #107: done in one attempt, no escalation; good report (found the -onlyprops gap and the cubemap nondeterminism).
- reviewer (Haiku) on 1420cff7..d788739e: PASS WITH NITS; one valid medium (-hintprotect text overclaimed vs water/slime), lows acted on in 6efdea39.
- review_fast (shop26 + glm-flash) on the same range: lows only, both found the orphaned AddWindingToPrimverts; shop26's -insert_search_path claim was wrong (filesystem_init.cpp reads it).
- Round 2 (d788739e..060c85c1): reviewer (Haiku) PASS WITH NITS (index spaces, mesh layout, lifetime, -onlyprops all confirmed); acted on: the HDR cube called GenerateSpheremap on a non-DEFAULT format (now skipped, spheremap zeroed), docs "Not covered" gaps. Not acted on (low): every displacement is meshed up front (~28 KB per power-4 disp, estimate), pre-existing unguarded mapdispinfo index in EmitDetailModels under -onlyprops. glm-flash approve, no findings; shop26 timed out at 2700 s with no findings in its partial.
- Round 3 (ca1d00cd): glm-flash approve, deepseek-flash (stand-in, shop26 busy) approve; both flagged the same sewn-seam wording in docs/detailclip.md, fixed in the last docs commit without another round (wording only, after round three).

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
