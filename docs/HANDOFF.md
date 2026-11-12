# Handoff, 2026-11-12 (wave 1 merged into master, wave 2 started)

The batching campaign: open issues grouped into waves of parallel worktree sessions (plan in the "Open issues batch grouping" session, 2026-11-10). Wave 1 ran 2026-11-11 as sessions A-F; its coordinator stalled before merging anything. This session merged it and started wave 2.

## Done
- Wave 1 is on master. Merge commits: A a172c307 (VBSP fixes), C 6fc25379 (static props), B 3038abd7 (VVIS), E ef19476b (Python tooling), D d4ee0865 (bspwalk). Tag `pre-wave1-merge` is the master before them (e12e52aa). The only conflict was this file; every section is kept below. F (#90) was diagnosis only, posted on the issue.
- Pushed to origin on 2026-11-12 at 70a81a33 (the user's call). That closed the 28 wave 1 issues with `Closes` lines. #77 was closed too, by an older commit (3e9035fd), and was reopened: its remainder is in session I's queue. #38 and #90 stay open as intended.
- Rebuilt in the main checkout after the merge (`createallprojects.bat`, then `buildallprojects.bat`): 0 errors. The binaries in `game/bin/x64` are from merged master.
- Suites on merged master, all pass: bspwalk selftest 175 checks; bspwalk fixtures 29 of 29 with `-vischeck` (must_visible 309 of 309, must_hidden 205 of 205); lmoptimizer 100 passed (the two real-compile tests ran); proplm 3 passed; vvis_sightline 2 of 2; vvis_mergeflow 2 of 2.
- New: `.agents/scripts/run_suites.sh` (those six suites, one line each) and `.agents/scripts/review_extract.sh` (small extract repo for third-party reviews; the full 711 MB snapshot does not launch).

## Final review of wave 1 (review_final on the merged ranges, from extract repos)
Every finding was static reasoning unless it says "ran". D had its own final review on its branch.

Python tooling (E, plus D's runner changes): dgx and glm-5.3, 8 lows.
- Fixed: `vbsp_chop` recursed once per cut and raised RecursionError past about 31k luxels, which dropped the transfer-growth warning (reproduced; ba80f5c0, guard added in 300b72d0). `vis_simulator.py` had no `--max-lightmap-dim` (b8b2efc5).
- Void: "fog faces are counted but VRAD makes no patch for them". `STATIC_FOG` is defined nowhere, so VRAD does make them.
- Kept as known approximations: the rectangle fast path over-counts a sheared face by about 0.3% (glm-5.3 ran a differential); displacement patches are modelled as a box tree.
- Not acted on: vmt_checker's new search path set differs from the old lookup in the direction of what the engine does.

VVIS (B): dgx and glm-5.3, lows only. Sent to session H for #110: dead VMPI entries in `vvis_dll.vpc`; `-mpi` is now an unknown-option failure where `-cuda` kept a warn-and-ignore branch; `fp_contract(off)` is not restored after the pop in `vvis.cpp`; no recursion depth check (about 1,450 levels per DFS at worst nesting). Checked clean by both: the fp pinning, thread independence by reading, `-trace`.

VBSP (A + C): dgx, glm-5.3, and Kimi as the overtime stand-in ($1.07). 15 findings.
- Real, all three agree: the `-forcedynamicpropsasstatic` fallback rotates vertex and tangent data in a VVD buffer nobody validated (it is not the file #70's vertex check loads), so a corrupt VVD writes out of bounds. Being fixed with three smaller items (MDL magic read before a size check, unclamped bone count in the SMD writer, a temp file left on the failure path).
- Being measured, not yet decided: dgx's claim that #70's validators are stricter than the formats, so real props could be dropped. A corpus tool that feeds every mounted model through VBSP is being written to answer it.
- Sent to session I with #37: `ParseGameLump` copies from unvalidated sub-lump offsets (stock behaviour; #63's cannot-inflate branch clamps with a wrong bound).
- Void: the keyvalue-check gap. `KeyValueText()` is guarded on `keyvaluesize`.
- Filed: #125, the SMD writer reads a mesh's material as a texture index while the shared walker calls it a skinref index.
- Not acted on: a heap vector per leaf test in the static prop leaf walk (unmeasured); `-threads` warning on every compile (session G is making the option real); `-tmpout` no longer redirects glview output (intended by #95).

## In progress when this was written
- Two implementers in their own worktrees: the VBSP hardening fixes above, and `prop_corpus_check.py`.
- `review_fast` round 2 on 300b72d0.
- `run_vvis_tests.ps1 -All -SkipVisualCheck`: first run of the three-way harness since #89 made the flow double. Not finished after more than an hour.
- Wave 2 sessions, each in its own worktree cut from d4ee0865 or later, none allowed to merge or push:
  - G, VBSP block pass: #103, #105. Owns wall-clock timing on the dev box.
  - H, precision: #110, then #104.
  - I, bspwalk: #47, #114, #39, #41, #44, #43, #37, #36, #48, #85, #115, #77, #38, then #34, #50.
  - J, prop lightmaps under #54: #56, #61, #67, #68, #69, #64.
  - K, VBSP detail props: #113, #111, #112 (proposed by the coordinator, started by the user).

## Next
1. Take the two implementers' branches: check the VBSP hardening against its crafted-input runs, read the corpus result and decide dgx's validator claim, rebuild the main checkout, rerun the suites, gate the fix commits.
2. Merge wave 2 as its sessions finish (G and H both edit `brushbsp.cpp`; merge G first).
3. Not batched: #99, #98, #97 (research, after #103); #101; #102; #100 (user decision); #90 (C++ fallback not implemented); #15 last.
4. The six wave 1 worktrees and branches are merged and can be removed (`wave1-bspwalk-b` holds only line-ending diffs on expectation files).

## Decisions made in wave 1 that nobody has confirmed
- #89 was fixed by moving the whole portal flow to double, so the default PVS is no longer bit-identical to Valve's float vvis.
- The MPI path is deleted (#51).
- bspwalk's default is intended movement; the 1.25 x air-speed factor is the session's judgment (`intended_air_speed` in the profile).
- #94's fix is opt-in (`-hintprotect`) because it measured no better.

## Delegation log
- Merge, build and suites by the orchestrator (Fable).
- review_final x3 on extracts: all six base legs ran to a verdict (glm-5.3 in 16 to 26 min, dgx in 54 to 79 min). The Kimi overtime hedge fired on the VBSP group only; the other two were skipped (one paid reviewer at a time per session) or not needed. Kimi rated two items high that the others rated medium or low; none of the three ran anything on the C++ extracts, so every C++ finding had to be confirmed by reading here. dgx was the only one to notice that #70 validates a different VVD file than the fallback rotates.
- Python fixes by the orchestrator (three small commits; tightly scoped, no delegation).
- reviewer (Haiku) on ba80f5c0..b8b2efc5: pass with nits. It ran old-versus-new differentials and found the one real problem, a hang on a no-progress split, with a reproducer. Its nit (the 2 to 125 check is repeated in two front ends) is not acted on: `lmoptimizer.py` is session I's this wave.
- review_fast on the same range: shop26 `lane_busy`, deepseek-flash stood in. glm-flash and deepseek-flash both approved and both raised the same hang plus a test that did not pin order. Fixed in 300b72d0. Round 2 on 300b72d0: launched, glm-flash only so far.
- impl (Sonnet) x2, isolated worktrees: VBSP hardening from the review; `prop_corpus_check.py`. Launched, results pending.

---

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

# Handoff, 2026-11-11 (wave 1 B: VVIS, #51 #89 #109)

Branch `claude/wave1-vvis-509f38`, not merged, not pushed.

## Done
- #51 (d5934b57, d6e9f5e2): one portal-flow recursion. `-trace` runs through `RunPortalFlowMT(numSorted)`; the stock `RecursiveLeafFlow` / `PortalFlow` and the WIN32-only MPI path (`mpivis.*`, the `MPI` blocks) are deleted. PVS byte-identical on cm_laststand, d1_trainstation_02_hrcs, ddd_market, validation, validation_harder; `-threads 1 -trace` linefiles identical to the old build for 8 cluster pairs, three of them dead-end stop clusters. `-trace` now stops at the first chain found.
- #89 (4e15f079): the default flow's geometry is double from the parsed `.prt` on (`visvec_t` in `vis.h`; `flow.cpp` and the two geometry helpers in `vvis.cpp` under a pinned floating-point model). Rules and the 0.01 epsilon unchanged.
- #109 (ed7361a2, b3911b63): `run_mergeflow_tests.py` checks restated; README and `-mergeflow` help no longer overclaim.

## #89 measurements (for #104)
- Reproduction: `python -I game/bin/x64/unit_test/vvis_sightline/run_sightline_tests.py`. Two chains cut from viscluster-less gm_fightspace_redux (32 portals / 17,300 units, 21 portals / 13,600 units), each with a segment proven in exact arithmetic to pass inside every portal. The float build leaves the far cluster out; the double build keeps it.
- The two chains named in the issue (7123->12874, 11260->11919) do NOT fail in vvis itself, only in the prtvis float port: prtvis builds `/fp:precise`, vvis `/fp:fast`, and the windows drift apart by step 8. Whether a float chain survives is close to a coin flip on rounding; a prtvis float trace does not predict vvis per chain.
- Rate: about 5,000 near-grazing ray chains from bases 7123 and 11260 gave 13 that float loses; 11,200 chains from random bases gave none.
- Where float loses them: the second `ClipToSeperators` call at the last portal. The stored window is already wrong (base 7123: a 0.0098-unit kink, 10 float ulps at z = 13165, amplified 29 times by the lever arm, so `fliptest` picks the wrong side). Exact arithmetic on the same float inputs makes the same wrong decision, so widening one step's maths would not have fixed it.
- Superset check against the float build: four maps byte-identical, d1_trainstation_02_hrcs gains one cluster pair (457 <-> 635) and loses none. Thread independence holds (`-threads 1` equals the default on four maps).
- Cost: not resolvable on this box today. Interleaved medians of flow time, float vs double: validation_harder 22.5 / 23.2 s, ddd_market 36.2 / 40.1 s, d1_trainstation_02_hrcs 115.6 / 105.8 s; same-round ratios ran 0.84 to 1.41 with another session's vvis holding the CPU at 100%. Read it as "a few percent either way"; measure again on a quiet box before #104 relies on it.
- Two rule risks double does not remove (candidates for #104): `fliptest` trusts the first source vertex outside the epsilon band, which assumes a convex window; and a separator the pass straddles by less than 0.01 is accepted, after which `flipclip` cuts the target further along the ray. The base-11260 fixture passes in double only because its micro-edge comes out with another orientation.

## Next
1. The final third-party review ran on the Kimi stand-in only (dgx and glm-5.3 `launch_failed`); a dgx or glm-5.3 pass over 1420cff7..HEAD is still owed if those lanes matter for the merge.
2. #110 holds every leftover: the redundant `g_pMergePortals` copy, `vvis_dll.vpc` MPI entries, dead declarations, garbled usage text, the fixed stack reserve, the `qsort` tie-break, correcting #89's text.
3. Re-run `vvis_test_harness.ps1`; it has not run since #89.
4. Re-time float vs double on a quiet box (scratch binaries are gone with the session; build d6e9f5e2 and HEAD).
5. Not done from #109: fixtures that exercise the hull limit, the rectangle fallback and 64-point windings; a fixture generator.

## Open questions
- "Exact" for VVIS used to mean equal to `-threads 1` stock float vvis. After #89 the default flow is its own reference (same at any thread count), and differs from Valve's by the pairs float dropped. `vvis_tests.psd1`'s ref-cpu leg compares with a tolerance, so it still runs; nobody has re-run that harness.
- Nothing here was looked at in game.

## Environment notes
- A `.vpc` edit makes the pre-build `vpc -crc2` step fail (MSB3073) until `createallprojects.bat` is re-run; after merging this branch, re-run it in the main checkout.
- `\n` inside a Bash-tool heredoc reaches Python as a real newline; write edit scripts with the Write tool.

## Delegation log
- impl-opus (#89 reproduction): ok, found that the issue's chains do not reproduce in vvis and searched for ones that do. Same agent continued for the fix: ok, first attempt passed every check. No escalation.
- #51 and #109 implemented by the orchestrator (serial, small).
- reviewer (Haiku) on #51: pass with nits, one real low (trace pre-pass ordering) that both third-party lanes also raised as medium; fixed in d6e9f5e2. reviewer on #89/#109: pass with nits, one medium (README line made false by #89); fixed in b3911b63.
- review_fast on d5934b57: shop26 + glm-flash, same two findings as Haiku. review_fast round 2 on d6e9f5e2: all three lanes `launch_failed` (infrastructure), no verdict; that commit is covered by the final review range instead.
- review_fast on d6e9f5e2..ed7361a2 (#89, #109): shop26 approve (1 low, 2 notes), glm-flash approve (1 note). Acted on: the mergeflow test's "tighter" check is now asserted even when -fast adds nothing; one comment reworded. Not acted on: stack reserve is a fixed 32 MB while `pstack_t` grew about 500 bytes a frame (the theoretical worst case already exceeded the reserve before this change); a zero-area portal from a corrupt .prt now gets a zero normal where stock gave (0,0,1).
- final-review (Fable, low) on 1420cff7..461dfdb5: ship with follow-ups, nothing blocking. Acted on: the sight-line test also runs `-threads 1` and compares rows; `vvis_tests.psd1` header; `-trace` usage mentions `-threads 1`. The rest is #110.
- review_final on 1420cff7..461dfdb5: dgx and glm-5.3 both `launch_failed`; the Kimi stand-in ran: request-changes, 1 medium, 4 low. Medium (static, arithmetic checked): `pstack_t` grew to 9,168 bytes, so the 32 MB stack reserve held about 5% fewer recursion levels than before; reserve raised to 64 MB (address space only), still no depth check (#110). Lows: the stop-only trace item skipped the INT_MAX item check (fixed); default-flow thread comparison in the sight-line test (already added in 53428d84); no `-trace` test (not added: a debugging aid, checked by hand against the old build); vpc and dead-declaration leftovers (#110).
- review_fast on 461dfdb5..5fcb1e85: shop26 approve, glm-flash approve; both raised the same low (the sight-line thread check did not compare the two runs' cluster counts), fixed in the commit after. That last commit is a three-line test change and was not gated again.

---

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
