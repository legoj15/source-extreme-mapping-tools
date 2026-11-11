# Handoff, 2026-11-11 (wave 1 E: Python tooling, branch claude/wave1-python-tooling-d7bb25)

## Done
- #80 3976d258 (+ GUI string fix 79a2d972): `--max-lightmap-dim` reaches every vbsp_lmo call, is in the cache key, validated 2..125, warned about at the end of a non-default run.
- #79 78865473: `--chop` never carves texlight sides.
- #86 f29edd92: under `--chop`, never-visible BSP faces are promotable in the carver, so the hidden part of a partly visible side is carved off and maxed. Policy: carving, not area weighting (area weighting coarsens faces a player sees). docs/lightmap_carving.md has the section.
- #66 f20377c9: one VBSP re-count after strict unification / floors change a scale; that count is what the budget check reads.
- #75 9b3fb656: `_coplanar_key` + `_edge_components` are the one grouping definition; the fallback unifier and `--strict-coplanar` use them.
- #91: no code change needed. All eight lows were fixed on 2026-11-08 (6f40cc2c, 52111d02, cad9d5db), before the issue was filed from the older df8a8031 snapshot. Checked 2026-11-11 by running lmo_report: the skipped text names "a stage outside the solver"; stranding says "Nothing came after it" when skip-ahead used 0 calls and "carried on" only when it used some; the summary header is "moved outside the solver"; `bspwalk_args` is in `_REPORT_ARGS`; `over_budget_scale64` is gone from the docs and report script (an over-budget run writes no report); before_solver is explained as auto-compile's up-front raise, which is the only way it arises; the --vis-debug material is covered by test_geometry_extents (cad9d5db).

- #83 0abf4c2e: report-and-warn, never cap. `vrad_patches.py` replays VBSP's SubdivideFace and VRAD's SubdividePatch on the baseline BSP. Own-scale counts are within 0.06% of VRAD on trainstation, and the rescaled prediction is within 2% (transfer growth 3.3x vs 3.25x measured). It warns at a 2x transfer growth. A VMF-only estimate was tried and dropped (24% off).
- c96a399a: review follow-ups. Default-dimension runs keep the pre-#80 cache key, and the --chop hidden-face exposure is documented.
- #32 ec86c463: vmt_checker and the InstancePath lookup go through gameinfo_search. Same %detailtype set on trainstation/hl2mp. InstancePath searches only the GAME entries (9e686915).
- Review fixes to the #83 counter: 9e686915, eb8c7d08, 48bc727d, c797ac29.
  - Every nonzero-area face gets a patch.
  - Carved pieces are counted at their own scales.
  - Face flags decide VBSP chop and VRAD subdivision on every path.
  - Never-chopped faces are counted per BSP face.
  - Own-scale counts on trainstation: 2,492,933 / 4,635,717 against VRAD's 2,492,962 / 4,635,888.

## Not done / follow-ups
- Haiku nit, not acted on: `count_vertices`/`compile_bsp` default `max_lightmap_dim=None` (VBSP's 32), so a future caller that forgets it reintroduces #80 silently. Making it required touches callers outside lmoptimizer.
- `vis_simulator.py` compiles at VBSP's default dimension and has no option for it.
- #85 (whether bspwalk's hidden verdicts on partly visible sides are right) now also bounds #86's exposure.
- vrad_patches approximations, adjudicated after review round 3:
  - A plain or NOCHOP side recounted coarser is counted from its VMF polygon, so BSP leaf splits are not modelled.
  - A NOLIGHT displacement that shares a texinfo with an emitting brush face is counted unsplit.
  - With version-0 side-ID lumps, the geometric match can add a coplanar side that is not on the face. This is not simply "high": it moves the before and after counts together and can hide growth. lmoptimizer's own baselines write version 1.

## Delegation log
- reviewer (Haiku) on 3976d258, 78865473, f29edd92: PASS WITH NITS. Its one high, the GUI parse error, was already fixed in 79a2d972. A medium on the #86 exposure was documented (c96a399a). A low on the cache key was fixed (c96a399a). Nits were adjudicated: one kept as a follow-up, the rest are text only. Useful: it ran a 1,500-layout carver differential.
- reviewer (Haiku) on f20377c9..ec86c463: PASS WITH NITS. Fixed: the InstancePath BASE_PATH/platform leak and the VRAD sky/nodraw/displacement rules (9e686915). Kept as-is: the strict unifier's missing tool/detail exclusions (pre-existing) and the VPK failure cache.
- review_fast round 1 on 1420cff7..ec86c463: all three lanes failed to launch. The 711 MB repo snapshot timed out unpacking on s1, an infrastructure failure. Retried on a 2 MB slice repo in the scratchpad (the touched Python plus vrad.cpp, faces.cpp and map.cpp): glm-flash approved with 3 lows; deepseek-flash (shop26 was busy) requested changes with 1 medium (ran it; real) and 2 lows. All fixed in eb8c7d08, except the cost low, which became the no-change skip.
- Fix round 2 (eb8c7d08): shop26 and glm-flash approved (notes taken in 48bc727d). Haiku FAIL: carved NOCHOP pieces were subdivided (ran; real; fixed in 48bc727d).
- Fix round 3 (48bc727d): shop26 and glm-flash approved. Haiku FAIL: a NOLIGHT side split over several faces was counted once (ran on trainstation; real; fixed in c797ac29). Round three was the limit, so the remaining items were adjudicated by the orchestrator (above). c797ac29 was not re-reviewed: it is a two-condition gate, checked by restoring Haiku's per-face trainstation totals and its synthetic split-face case.
- Haiku was the most useful reviewer this session: every round it ran harnesses and found real value bugs. DeepSeek also ran its finding. shop26 found nothing of substance.

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
