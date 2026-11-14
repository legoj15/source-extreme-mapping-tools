# Handoff, 2026-11-14 (#77: d1 door panel, on master)

## Done
- 9434e6e1, 6c223d35: `FeetColumn` returns no piece when its prism reaches more than 0.01u past the floor polygon's XY box. A near-zero-area sliver node (d1 node 51038, prop top 2e-7u wide) built a prism 13,500u long whose eyes sat inside brush 1295. Selftest h1s (exact node 51038 vertices) failed first, passes now. d1 hrcs, hl2 profile: tier A 0 -> 100, tier B 402 -> 764. All six suites pass.
- `-traceeye x y z`: names the eye piece (source, node, vertices, node polygon at full precision) that emits a sample at a point.

## Not verified
- No corpus or other-map A/B of the prism guard beyond d1 and the suites.

## Next (#77)
- 1159/1160 are still visible (hl2 profile, CPU face pass): eyes at (-2238,-1700,466) on street node 11031 and (-2624,-1240,328) on node 5546, raised by fall columns. 438u rise = z 381 + jump apex: the launch is the top of the uphill slide-edge climb on props 2253-2257 (`SlideExitAllowed` + per-node reach credit). User decision 2026-11-14: uphill slide chains are surfing, behind `-exploits`. Fixing that is the next step.
- Under the gmod profile the store roof (node 14389) is also reached through that climb and through arcs that need gmod's base-class speeds (run 400 / sprint 600; sandbox is 200 / 400).

## Delegation log
- Orchestrator implemented it: the fix is about 20 lines found while tracing (diagnostic plus guard); writing a brief would have cost more. review_fast on 9434e6e1 (shop26 lane_busy -> deepseek-flash stand-in, glm-flash): five lows; fixed the vacuous h1s assertion (both agreed), left the -traceeye debug-print lows. review_single glm-flash on 6c223d35: approve; its note that the box check does not fire for h1s is contradicted by the pre-fix run (1 piece -> 0 with only the guard added).

# Handoff, 2026-11-14 (#132: -keepstalezip, branch claude/issue-132-analysis-3d3a19)

## Done
- 89502213: `LoadBSPFile_FileSystemOnly` loads the pak lump with no version check (it asked for 1; the toolchain writes 0), so `vbsp -keepstalezip` works. Built on the dev box. `proplm_stale_vtf_check.py`: the base build exits 1 on the `-keepstalezip` case and the fix exits 0, keeping the old pak (`0.vtf` with the same CRC, cubemap defaults, `stale.txt`) and dropping the stale `proplm/1.vtf`. All six suites pass.

## Not merged
- The branch is not merged to master or pushed.

## Delegation log
- Orchestrator implemented it: one changed line, smaller than a brief. review_fast on 89502213 (shop26 + glm-flash): three lows, all pre-existing. Not fixed: the pak load skips `ActivateByteSwapping` (benign, PC-only). Rejected: "mapRevision reset to 0" (`LoadMapFile` sets it from the VMF afterwards, map_shared.cpp:52).

# Handoff, 2026-11-13 (wave 4: prop lightmap leftovers, built and checked on Server1)

Same session and branch as wave 3 (`claude/issue-clearing-campaign-716af2`), cut from master c39c3201. Merged to master at 217acae5 (tag `pre-wave4-merge` before it) and pushed on 2026-11-13 at the user's word; merged master builds with 0 errors and passes all six suites on Server1. `review_final` (dgx and glm-5.3) on the merged range was still running at the push. The user asked to keep compute off the dev box, so every build and tool run for this wave happened on Server1 through `.agents/scripts/server1.sh`.

## Done
- **Server1 as the build and test box.** `server1.sh setup | sync [rev] | run '<cmd>' | fetch`: a clone at `C:\Dev\semt\repo`, synced to any commit (pushed or not), full build in about 40 s, all six suites pass there. Twelve harness scripts had `E:\Steam` as a literal; their default game dirs now follow `SEMT_STEAM_COMMON`. Limits: no HL2DM client (the SDK Base 2013 `hl2mp` stands in, set as `PROPLM_TEST_GAME` and `LMOPT_TEST_GAME`), no Black Mesa or other corpus content, and its GTX TITAN X does not run the CUDA kernels (bspwalk reports 0 GPU checks, VRAD bounces on the CPU). `pytest` was installed there with `pip --user`.
- **#126** fd0511bd, then b1cb00f5 (`vradstaticprops.cpp`): the per-texel bake also rasterizes the lower LODs a model still draws. An inside sample from a lower LOD replaces a margin sample; LOD 0's inside samples are never replaced. Texels inside a lower-LOD triangle and no LOD 0 triangle that were never listed for lighting: laundry_dryer001 971 of 3,516 after the first patch, 0 after the second; gear01 2,188 to 0; gear02 1,542 to 0; prison_wallpile002a 281 to 0; kitchen_fridge001a 512 to 0. refrigerator01a (no kept LODs) bakes byte-identically to the old build. Texels farther than one texel from a newly covered one are byte-identical old against new; the 12 to 66 texels per model that differ are the newly covered ones and their neighbours through the encoder's 3x3 filter.
- **#131** ef4f2554 (`vradstaticprops.cpp`): bounded VVD path, heap hull points, and the VVD beside the MDL first when its checksum matches, as VBSP does since #128. Checked with a Counter-Strike: Source prop copied under another name into a scratch game dir: the old VRAD exits 1 with "Unable to load vertex data"; the new one lights it, and the result is byte-identical to the same model under its own path.
- **#116, second half** 4fef4bab (`proplm_uv2.cpp`): VBSP removes `materials/maps/<map>/proplm/<id>.vtf` entries whose id no prop carries. After `-onlyents` with one of two props removed: the old VBSP keeps both VTFs, the new one removes the stale one and says so; with every prop removed both go; with the VMF unchanged nothing is removed.
- **#127 follow-up** 4fef4bab (`proplm_rebake.cpp`), from the wave 3 final review (GLM-5.3, Kimi and dgx all raised it): a model whose dx80 and dx90 VTX files differ is refused unless both draw the same LOD 0 triangles. Rebake pak lumps on the fixture models are unchanged.
- One constant for the rebake's "LOD switched off" switch point (`src/public/proplm.h`), used by both tools.
- New check scripts (1f13da69): `proplm_ppl_ab.py` (old and new VRAD on one BSP, texel by texel, with a coverage model that predicts VRAD's own counts exactly), `renamed_model_check.py`, `proplm_stale_vtf_check.py`.
- Suites on Server1 at the final commit: proplm 6 passed, lmoptimizer 101 passed; full build 0 errors. All six passed at fd0511bd.
- Wave 3's `review_final` collected: GLM-5.3 one low, Kimi one medium (the same rebake LOD 0 gap, fixed above), dgx one medium (VRAD's VVD lookup, which is #131, fixed above) and notes. Not acted on from dgx's notes: the fixup-applied VVD copy keeps header fields for blocks it does not carry (only the hull reads it); a too-long dict name is refused after a `-forcedynamicpropsasstatic` recompile may already have embedded the model.

## Not verified
- Nothing was looked at in game: lower-LOD lighting, the renamed model, the cleaned pak.
- GPU bakes: Server1 bakes on the CPU, so #126 was not run through the GPU path. The sample generation it changes is shared by both.
- #131 with a VVD beside the MDL whose checksum does not match (should fall back to the internal name).
- The #127 refusal has no model that triggers it.
- The dev box binaries are not rebuilt for wave 4. Run `bash .agents/scripts/worktree_build_setup.sh --build` in the main checkout before using them.

## Decisions nobody has confirmed
- **#126 also changes ordinary static props**, not only rebaked ones: any per-texel prop whose lower LODs reach texels LOD 0 does not. Of six stock props tried, radiator01a gained 467 lit texels and trashdumpster01a 44; texels already lit did not change. Kept, because those texels were black before. It costs one more rasterization pass per drawn LOD (not timed).
- **#116, first half, is still open**: should a converted proplm prop honour `lightingorigin` (it would flatten per-texel lighting) or keep the warning? The issue stays open for that.
- The pak cleanup treats the whole `materials/maps/<map>/proplm/` folder as the tool's: anything there that is not `<id>.vtf` for a current id is removed.
- After `-onlyents`, `vrad -fast` writes no prop lightmaps, so kept VTFs can belong to an earlier layout (a warning is printed). Dropping the whole folder under `-onlyents` would be stricter; not done.

## Next
1. #132 (`-keepstalezip` fails on every BSP, since the initial import) and #133 (unsynchronised first VVD load on VRAD's lighting threads; two reviewers, not reproduced).
2. #121 (float against double vvis timing on a quiet machine): Server1 is quiet and has the SDK game dir, so it can run there now.
3. A VBSP access violation was seen once on Server1 with a scratch game dir that mounted no material VPKs, on the old and the new build; dumps are in `C:\Dev\semt\verify\crash`. Not isolated, not filed.
4. Research and design: #97, #98, #99, #101, #102, #54, #50, #15 last. Questions for the user: #100, #119, #122, #39, #77, #38, and #116's lightingorigin.
5. `run_vvis_tests.ps1` production test still looks for d1_trainstation_02_hrcs.vmf in the wrong folder; the `.ps1` harnesses still carry `E:\Steam` literals.

## Delegation log
- scout (Haiku) x3: accurate; the rebake scout established that VRAD writes the proplm VTFs and VBSP's `Proplm_WriteGameLump` is the place to clean them.
- `implement_single` glm-5.3, `no_build`, x3 in parallel from an extract: #131 and the rebake/pak change compiled on the first Server1 build and behave as briefed. #126 compiled and was safe but lit only half to three quarters of the texels it should (first touch decided a texel). Escalated once.
- impl (Sonnet), verification on Server1: wrote the three check scripts and found the #126 shortfall by measurement; also showed the base VBSP already had #128, that stock HL2 models cannot test #131 (every internal name matches its path), and #132.
- impl (Sonnet), #126 second tier: fixed the claim rule in one pass against the measurement; never-listed went to 0 on all five models.
- reviewer (Haiku) on f828ce8e..fd0511bd: FAIL on #126, with a replica of the claim rule; this was right and the two third-party lanes missed it. Its other items: comment corrections and the shared constant (done), a dangling thread-local (done), #133.
- review_fast round 1 (shop26 and glm-flash both ran): approve, approve, by reading. shop26's low (stale docs) is fixed; glm-flash's low (VRAD reads only the dx80 VTX for the drawn test) is left as a corner case that needs a hand-made VTX pair.
- review_fast round 2 on fd0511bd..d85386de (shop26 and glm-flash both ran, each with a replica of the claim predicate): approve, approve, no findings. This is the final code sha.
- Orchestrator (Fable): `server1.sh`, the harness path change, applying patches, adjudication.

---

# Handoff, 2026-11-13 (wave 3: review leftovers and static prop bugs)

One session, branch `claude/issue-clearing-campaign-716af2`, cut from master 93028e29. Five batches on disjoint files, built and checked together in this worktree. Merged to master at 343dbaf7 (tag `pre-wave3-merge` before it) and pushed on 2026-11-13 at the user's word; merged master builds with 0 errors and all six suites pass. `review_final` (dgx and glm-5.3) on the merged range was still running at the push; its findings go in as follow-up commits.

## Done
- **#120** 7212d4f8 (`portals.cpp`): a portal whose two split pieces are both tiny is freed (it leaked); `-v` prints "N tiny portal pieces culled" per tree. The issue's first suggestion was tried and dropped: giving the portal the clipped piece when one side is tiny moved 7 of 3,018 portals on d1_trainstation_02_hrcs (a repeated point, two sliver edges) and changed faces and vertices (20 lumps there, 17 on fightspace3_unittest). With the commit as it is, both maps are byte-identical to the build before it, `.prt` included.
- **#124** closed without a change: 8da6a1fb (wave 2 G) already zeroes the dispinfo padding.
- **#123, #118, #117, #125, #128** e2dfef9d (`staticprop.cpp`): a prop_static model name of 128+ characters is refused with one warning per name; hull points are on the heap; `CacheVertexData` caches the LOD0 vertex list with VVD fixups applied, so each mesh's hull is built from its own vertices; the SMD writer goes through skin family 0; the VVD beside the MDL is used when its checksum matches the MDL, else the internal-name path as before (`FindVvdPath`, one helper for the check and the loader).
  - HL2DM corpus (2,225 models) against a baseline from the build before: no change.
  - Black Mesa (8,559 models), before and after: exactly the 17 models #128 lists went from refused to placed; nothing else changed class.
- **#127** 042f7a29, 58892a02 (`proplm_rebake.cpp`): lower-LOD decisions gather from both VTX files; a model with one file, or two byte-identical ones, takes the old path. `proplm_rebake_ab.py` old against new: pak lumps byte-identical.
- **#129** f391ad92 (`vradstaticprops.cpp`): each model's UV overlap is measured once on the main thread before the lighting threads start. The addon's modelscale message no longer says the bake ignores it.
- **#130** af92048a, dcfe4105, 5aa5263c, 56d257e4 (`vvis.cpp`, `docs/vbsp_sliver_measurements.md`): `-radius_override` refuses anything that is not a positive number (before: `0`, `abc`, `-5` each gave 0 visible portal pairs and no warning); the `-mpi` branch matches only the old VMPI names and takes a value only for the six that had one; `-mergeflow` measured against 70a81a33: d3_c17_02_camera gains 2 cluster pairs, l4d_smalltown01_caves 44, none lost, and with the plane fallback disabled both flows are byte-identical to 70a81a33, so #110's cleanup itself moved nothing; the write-up's answer sentence and tables are corrected.
- New: `.agents/scripts/worktree_build_setup.sh` (copies the untracked libs, DLLs and test maps into a fresh worktree, generates the projects, builds all or one project). A full build here takes about a minute.
- All six suites pass on the final build (selftest 177; lmoptimizer 101; proplm 6; vvis_sightline 4 cases, the new one checks option parsing by exit code and whether the map was reached).

## Not verified
- Nothing was looked at in game.
- #123: no map with a 128-character prop_static name was compiled; the guard was traced by three reviewers.
- #117: no before-and-after of leaf lists for one of the 174 stock props with reordered fixups. The corpus loads all of them without error.
- #125: no model with a non-identity skin family 0 was found to compare SMD material names.
- #129: no compile with several instances of one overlapping model to count the warnings.
- #127: no model whose dx80 and dx90 differ exists to test with.
- gm_fightspace_redux rows in the sliver write-up are from the 2026-11-12 run (its compiled files are gone).

## Decisions nobody has confirmed
- #120 is closed with the unclipped winding kept (stock behaviour), on the measurement above.
- #128: which VVD the engine loads for a renamed model was not confirmed from engine source or in game; the evidence is that the 21 shipped models work in their games and each has an intact, checksum-matching VVD beside it.
- `-mpi_file`, `-mpi_filebase` and the `-mpi_Patch*` options are now unknown options (VMPI put them on worker command lines itself; vvis never accepted `-PatchHost`). An unknown `-mpi…` name prints usage instead of being ignored.
- Review notes left alone as corner cases: a second word after a value-taking `-mpi` option can become the map (invalid command, as before); the rebake's texture sets ignore a material drawn only by the other VTX file's LOD0 (studiomdl does not write such pairs); the `-forcedynamicpropsasstatic` rotation fallback takes the VVD beside the model without the checksum test.

## Next
1. Still open from the review leftovers: #126 (texels only kept lower LODs use are unlit; same file as #129, so it waited), #116 (lightingorigin decision, stale proplm VTFs after `-onlyents`), #121 (timing on a quiet machine).
2. Research and design: #97, #98, #99, #101, #102, #54, #50, #15 last.
3. Questions for the user: #100, #119, #122, and session I's #39, #77, #38.
4. `run_vvis_tests.ps1` production test still looks for d1_trainstation_02_hrcs.vmf in the wrong folder.
5. #131: `src/utils/vrad/vradstaticprops.cpp` has the same unbounded `strcat` VVD path (about line 2914) and `_alloca` hull (about line 488) that VBSP had, and looks the VVD up by internal name only, so check it on the models #128 now places.

## Delegation log
- scout (Haiku) x3, one per GLM brief: accurate file:line facts; the static prop scout also noted that line 944 in the issue text is not a name guard and found the vrad twins in Next 5.
- `implement_single` glm-5.3, `no_build`, from a 2.8 MB extract (the full repo snapshot does not unpack on s1), three runs in parallel through tpr-courier: static props (21 KB patch), VRAD overlap, rebake LODs. All three applied cleanly, compiled with 0 errors on the first host build and passed their checks. No escalation.
- impl (Sonnet) in its own worktree, #130: started there, not at `implement_single`, because item 3 is a measurement against an old build and items 1 and 2 were to be shown by runs. One pass, no escalation. It found the two real maps in an old session scratchpad.
- Orchestrator (Fable): #120 (17 lines, needed compile-and-compare), the review fixes, merges.
- reviewer (Haiku) on 93028e29..875a899b: pass with nits, 13 items, several by replica. Acted on: radius square, dead threaded guard and two wrong comments, message pins in the new test, setup script path comparison. It and glm-flash both raised the rebake's replacement scan bound (fixed, 58892a02).
- review_fast round 1 (shop26 and glm-flash both ran): approve, approve. Round 2 on the fixes: approve, approve; one low from both (a constant's name in a message), fixed in a4460d0f, which is four lines of message and docstring and was not sent round again.
- For the GLM-written changes the glm-flash leg is discounted; shop26 and Haiku are the second opinions.

---

# Handoff, 2026-11-12 (wave 2 merged; review follow-ups)

Coordinator's record. The sessions' own sections follow below, each as its session wrote it; where they say "not merged", this section is the newer word.

## Done
- Wave 2 merges on master, at the user's instruction to merge each branch as it finished: K 9c718cd6 (detail props #113 #111 #112), G 5e584c42 (threaded block pass #103; #105 closed by measurement), H 78528488 (VVIS cleanup #110; #104 closed by measurement), J c5412415 (prop lightmaps #56 #61 #69 #68 #67, refs #64). Tag `pre-wave2-merge` is the master before K. Conflicts: this file and the Scripts list in `AGENTS.md`, both sides kept. **Pushed to origin on 2026-11-12 at 51cb9eeb** (the user's call), with everything listed in this section; that closed #56, #61, #67, #68, #69, #103, #104, #105, #110, #111, #112 and #113. #64, #77 and #38 stay open as intended.
- Session I merged last, 2e6f36d6 (bspwalk #47 #114 #41 #44 #43 #37 #36 #48 #85; refs #115 #50 #77 #38 #39), after the push above, and pushed in its turn on 2026-11-12 at 69a81254 (the user's call); that closed #114, #36, #37, #41, #43, #44, #47, #48 and #85. #115, #50, #77, #38, #39 and #64 stay open as intended. Tag `pre-wave2i-merge` is the master before it. On that build (full rebuild, 0 errors): all six suites pass (selftest now 177 checks, 38 GPU-only), the HL2DM corpus is unchanged against its baseline, and `-threads 1` against `-threads 8` is still identical on fightspace3_unittest. Its bsplib change reaches every tool: a BSP whose lumps or game sub-lumps lie outside the file, or whose lump is longer than its fixed array, is refused at load (7 of 2,641 installed maps by its scan; details in its section).
- Not merged, on purpose: `claude/wave2-105-adaptive-experiment` (G's experiment, marked do not merge).
- On the merged build (`createallprojects.bat` then a full build, 0 errors): all six suites pass (selftest 175; fixtures 29 of 29; lmoptimizer 101; proplm 6; vvis_sightline 3 of 3; vvis_mergeflow 2 of 2). `vbsp_repro.py` with `-threads 1` against `-threads 8`: fightspace3_unittest 4 compiles and d1_trainstation_02_hrcs 2 compiles, all identical. On these maps the block pass is 0.2 to 0.3 s of a 6 to 7 s compile, so the wall time does not move; G's own figures on larger maps are in its section.
- VBSP hardening from the wave 1 final review, five commits (e6e8a516, 630933d8, 99a698dd, cc0e97a1, 02bcf5c0), three gate rounds, closed: the `-forcedynamicpropsasstatic` flag-patching fallback validates the VVD it rotates (header, blocks after the header and inside the file, not overlapping) and drops the prop when the file is bad or missing, instead of writing out of bounds or embedding a half-converted model; `LoadStudioModel` no longer reads the magic of a file under 4 bytes; the SMD writer clamps the bone count; the failure path removes its physics SMD; `Proplm_LoadVvdVertices` requires the vertex block to start after the header. Left as is: the fixup table offset has the same loose bound (in bounds, wrong data from an already invalid file).
- `game/bin/x64/prop_corpus_check.py` (035b6fbd through 365a1293, three gate rounds, closed): feeds every mounted model through VBSP's `prop_static` path, fails on anything refused, crashing, unexplained or unread, compares against an earlier report (`--baseline`), mounts another game's files on a working game directory (`--content`), and checks VBSP's static-prop verdict against each MDL's header flag. Results:
  - Valve content (HL2 Deathmatch 2,225 models, Counter-Strike: Source 3,817, Day of Defeat: Source 3,009): none refused by validation, none crashing, flag check 0 disagree. Rerun on the final build with baselines: unchanged.
  - Nine other content sets through `--content` (about 26,000 models; the implementer's runs): **two shipped Black Mesa models were wrongly refused** by #70's name check (66-character internal names; stock VBSP places them). Fixed in 1581b83a, which also bounds the VVD path the stock collision loader built with `strcat` into 260 bytes; both models are placed on the main checkout's build. 21 models across four games are dropped because the VVD is looked up by internal name (#128; stock aborts the compile on them). One Black Mesa model exits VBSP inside the physics library, as stock does. The final review's two specific worries (fixup coverage, material index bound) found nothing.
  - Not covered: workshop `.gma` files. The user's `models/legoj15/pneuma_sword.mdl` is studio version 49, which this VBSP does not read.
- Python review fixes, two gate rounds, closed: ba80f5c0, 300b72d0 (`vbsp_chop` without recursion, with a no-progress guard), b8b2efc5 (`vis_simulator.py --max-lightmap-dim`).
- `lumpdiff.py` (9bf40fd4) now also compares the whole file; `-threads 1` and `-threads 8` BSPs have the same sha256 on both test maps.
- `review_final` on the merged wave 2 VBSP range (G + K): dgx and Kimi (stand-in for glm-5.3, refused by the quota gate; $0.70). No race found in the threaded block pass; the detail prop fixes read as correct. Both raised the areaportal-in-water fixup change (see decisions); the lumpdiff looseness is fixed; not acted on: zeroed per-block node statistics in the threaded log, and Kimi's static claim that a hand-packed material named `*_wvt_patch` would be mistaken for a generated one under `-onlyprops`.
- #34 closed by session I's measurement (binning is 0.003 s of a 240 s vis stage).

- Gate on 1581b83a, closed: reviewer (Haiku, with a replica) pass with nits; shop26 and glm-flash approve. All three noted that `BuildVvdPath` refuses a name over 248 characters though a few more would fit; left as is and documented (8077acf3, 51cb9eeb).
- `review_final` on the prop lightmap range (J), complete: glm-5.3 approve (it compiled `uvoverlap.cpp` alone, fuzzed about 17,000 cases and checked the #67 renumbering on 3,000 random models), Kimi approve with no findings, dgx comment with two lows. Nothing acted on in code; #129 holds dgx's unsynchronized lazy overlap analysis in VRAD and the smaller notes.

- H's own final review finished after its session ended and was collected here: Kimi's two findings were already handled by H in 51182199; the dgx leg (one medium, three lows, notes; all static) is filed as #130. The item to act on first is `-radius_override` accepting 0 or garbage and writing a near-empty PVS without a warning. dgx also points out that `-mergeflow` output changes on maps with a first-three-degenerate portal (the same fix the default flow got, unmeasured), and that the sliver write-up's headline is true by construction; #104's conclusion stands on the thin-portal table.

## In progress when this was written
- Nothing. Every wave 2 session has finished, is merged and pushed, and every review launched for either wave has been collected.

## Next
1. #130 (VVIS follow-ups from H's final review), #128 (VVD looked up by internal name) and #129 (VRAD lazy overlap analysis) are the review leftovers worth a session.
2. Wave 3 has no plan yet. Held from the original grouping: #99, #98, #97 (research, now that #103 is in), #101, #102, #15 last; #100, #119 and #122 are questions for the user.
3. Decisions session I left for the user: #39 (sample budget), #77 (is the rooftop reach legitimate), #38 (fall columns); what is left of #50 and #115 is listed on those issues.
4. `run_vvis_tests.ps1`: the `production` test looks for d1_trainstation_02_hrcs.vmf in `bsp_unit_tests`; the file is in `unit_test_maps`. Fix the path or the manifest, then run the production tests on a quiet box.
5. Done 2026-11-12 at the user's word: the eleven wave sessions are archived and all thirteen worktrees are removed (ten session worktrees, `wave1-bspwalk-b`, two implementer worktrees). Branches are kept; all are merged except `claude/wave2-105-adaptive-experiment`, which is kept on purpose.

## Decisions waiting for the user
- From H: portals whose first three points are collinear are now opened, so the default PVS gains pairs on maps that have one; the sort tie-break adds one pair on ddd_market. #122: cull portals under about one unit wide?
- From G: default thread cap 8; #119: make `-blocksize 2048` the default? Also from G, confirmed by two final reviewers: the threaded block pass applies every block's areaportal-in-water fixup before any block runs, so output can differ from the old serial result when an areaportal spans blocks (single-pass builds: `-noopt` or leaked maps). New thread counts agree with each other; no map checked hit it.
- From J: the rebake refuses a material with a texture key it does not rebuild; modelscale applies to the proplm bake.
- Hardening: under `-forcedynamicpropsasstatic`, a model whose VVD beside it is bad or missing is now deleted with a warning.
- Wave 1's unconfirmed decisions are listed in the section "wave 1 merged into master" below.

## Delegation log
- impl (Sonnet), VBSP hardening: three passes in one worktree. Pass 1 met the brief; the brief (mine) did not require the blocks to lie outside the header, and the Haiku gate found that hole by tracing it. Pass 2 confirmed it by running the reviewer's 92-byte file (0xc0000005 on the pass 1 build) and fixed it; pass 3 added the no-VVD refusal two reviewers asked for. Good reports: before and after for every crafted input.
- impl (Sonnet), `prop_corpus_check.py`: first pass ran the corpus and also read every file to check the two claims independently. Its bisect treated "VBSP cannot start" as a crash in every batch (found when Fortress Forever cost 2,162 runs in six minutes); fixed with a canary compile on request. Exit-code holes found by the gate are with it now.
- reviewer (Haiku) x4: found the `vbsp_chop` hang (ran it), the header-overlap hole in the hardening (traced; a run confirmed it), and the corpus tool's exit-code holes (ran them). On the hardening, glm-flash approved the same diff after a 3.6M-case fuzz replica that missed the hole.
- review_fast x5 from extracts. shop26 was `lane_busy` on four of the five; deepseek-flash stood in twice. glm-flash and deepseek-flash both raised the unblamed-batch exit code on the corpus tool; glm-flash's claim that `--explain` reads headers at the wrong offsets is unverified and looks wrong against `studio.h`.
- Merges, builds, suites and the three Python fixes by the orchestrator (Fable).
- Later the same day:
  - impl (Sonnet), hardening, passes 4 and 5: the round 2 leftovers (overlap refusal, one size, wording), then the Black Mesa name fix, where it also found the unbounded `strcat` in `CacheVertexData` by reading every consumer of the name. No escalation in five passes.
  - impl (Sonnet), corpus tool, passes 3 to 6: `--content` and the nine content runs (it compared each refusal category against Valve's own vbsp.exe, which is what separated the two real over-refusals from the rest), the two gate rounds of exit-code fixes, and the flag cross-check it proposed itself. No escalation.
  - reviewer (Haiku), three rounds on the corpus tool, FAIL each time on a reproduced exit-0 path (refusals classed "other"; a crash that passes on retry and an unlistable folder; an unreadable header in the flag check). The last one was fixed by the orchestrator after round three (365a1293). It also settled the `--explain` offsets against `studio.h`: all correct, the glm-flash claim was wrong.
  - reviewer (Haiku) rounds 2 and on the hardening: pass with nits; a g++ replica and 300k fuzzed headers.
  - review_fast: hardening rounds 2 and 3 (shop26 ran both; glm-flash once, then refused by quota with deepseek-flash standing in): approve. Corpus tool round 2 (glm-flash approve, shop26 request-changes on a non-ASCII over-length name being classed a crash, a false alarm in the failing direction, not acted on). `vbsp_chop` guard round 2: glm-flash and deepseek-flash approve.
  - The Z.ai 5-hour window ran out mid-afternoon (98% used with five sessions and the coordinator drawing on it); glm legs were refused from then until the 02:00 UTC reset, except where the gate still passed.

---

# Handoff, 2026-11-12 (wave 2 K: VBSP detail props, branch claude/objective-golick-f86a4e)

Cut from master 70a81a33. Not merged, not pushed.

## Done
- #113 5f382927: detail model names of 128+ characters are rejected in IsModelValid (error once per name, props skipped) instead of being written unterminated into the dict lump. Old build: 128-char name unterminated, 172 props on it; new: those 172 dropped, the other 1338 identical.
- #111 57be14e8: cause was not in the -onlyprops path of vbsp.cpp but in material patching. A finished BSP's texdata names VBSP's patch materials (`_wvt_patch`, cubemap patches, chained); the patched->original table lived only in memory and the material system cannot see the BSP pak, so patched faces lost `%detailtype`. `RestoreMaterialPatchTranslationsFromPak` (materialpatch.cpp) rebuilds it from the pak. Old/new -onlyprops props vs the full compile's: WVT test map 759/1479 of 1479, with an env_cubemap 0/1479, d2_coast_01 16692/16708 of 16708 (now exact matches).
- #112 774d196a + 534243fb: -detailclip skips displacements sewn to the candidate's own along a whole edge, as it skips its own. Decision and cost recorded in docs/detailclip.md and on the issue. fightspace3 enriched: displacement removals 3677 -> 50, buried/brush/static prop counts unchanged; flag off byte-identical.
- Review fixes 534243fb, 30f881d8, 17fcc4a7 (docs only).
- New: `game/bin/x64/unit_test/detailprops/detailprop_map.py` (gen a scratch game dir + test VMF with `--longname` / `--overhang`; dump and diff of the dprp lump). Needs the Python `vpk` package for `--longname`, and the SDK path at the top of the file.
- `run_suites.sh` on the final build: all six pass.

## Merge notes
- vbsp.cpp (session G's file): one include (`materialpatch.h`) and one call in the -onlyprops branch. Nothing else.
- Full compiles are byte-identical to the base build on the test maps. d2_coast_01 differs only in ddispinfo_t padding, which also differs between two base runs (#124).

## Filed
- #123: same unterminated-name bug for static props (staticprop.cpp, session J's file).
- #124: ddispinfo_t padding uninitialised, LUMP_DISPINFO differs run to run (matters for #103).

## Open
- review_final (dgx + glm-5.3) not run; it belongs after the merge. shop26 was busy for both fast rounds (deepseek-flash stood in).
- #112 limits documented, not fixed: an edge-sewn cliff wall or overhang is skipped like own relief; a vertex-poor coarse edge can count as sewn over a fine neighbour that diverges between its samples.

## Delegation log
- Implementation by the orchestrator (Opus): small, Windows-toolchain-bound changes verified by real compiles; no implementer delegations.
- reviewer (Haiku) on 70a81a33..774d196a: PASS WITH NITS. Real: the one-vertex sewn rule (fixed in 534243fb), lows on materialpatch (fixed), the static prop twin (#123). Wrong: claimed a 64K-cap abort, but the flag-on set is still a subset of the flag-off set.
- review_fast r1 (glm-flash + deepseek-flash stand-in, shop26 lane_busy): one high was wrong (it said the game lump is 32; bspfile.h has 35). Lows acted on in 30f881d8.
- review_fast r2 on 774d196a..30f881d8 (glm-flash + deepseek-flash, shop26 lane_busy): both approve; one low documented in 17fcc4a7.

---

# Handoff, 2026-11-12 (wave 2G: VBSP block pass, branch claude/wave2-vbsp-block-2b5d08)

Not merged, not pushed. Cut from master 70a81a33.

## Done
- Step 0: VBSP output was already reproducible on this tree. 5 of 5 repeat compiles were identical in every lump on fightspace3_unittest and on d1_trainstation_03 (sky map, HDR default cubemaps in the pak lump). The wave 1 scratch lumpdiff is now `game/bin/x64/lumpdiff.py`, and `game/bin/x64/vbsp_repro.py` drives it: interleaved variants, BSP and `.prt` compared with the first compile (18dbb770, hardened in d9b06b3f, 86feb1da, e1ad59b7).
- 8da6a1fb (refs #106): the dispinfo lump's two padding bytes per entry (offsets 38-39) were heap garbage, and gm_fightspace_redux differed there run to run. Now zeroed. This is the only output difference from the pre-wave-2 binary on every map checked.
- #103 33e79441 + d9b06b3f: the block pass runs on `-threads` threads with identical output. The default is one thread per CPU, at most 8; `-threads 0` means the default. A serial PrepareBlockPass creates every plane the threads look up, in the old serial order, and makes the shared areaportal-in-water fixup writes. The threads run with planes frozen (CreateNewFloatPlane Errors). Clip-plane numbers are thread_local; node and brush id counters are atomic. Evidence: 105 threaded and serial compiles across 4 maps, all identical (counts in the commit). The speedup is real but small: the block pass is 1-5% of VBSP time (fightspace_redux 0.46 -> 0.18 s of a 15 s compile).
- #105: measured. A brush-count adaptive grid (`-adaptiveblocks N`) wins indoors (trainstation_03 -12% PVS triangles) and loses 18-63% on d2_coast_03, so it is rejected as a default. Tables are on #105. The code lives only on branch `claude/wave2-105-adaptive-experiment` (8f98de84), **do not merge it**. Closed by this handoff commit.
- Filed #119 (question): `-blocksize 2048` beats stock on all three maps measured (-2.5% to -4.7% triangles, vis 4x faster on the open maps), but exact vvis on trainstation_03 gets 2-4x slower. The default change is the user's call.
- `run_suites.sh` on this build: all six pass (bspwalk selftest 175, fixtures, lmoptimizer 100, proplm 3, vvis_sightline 2/2, vvis_mergeflow 2/2).

## Decisions nobody has confirmed
- In pass 0 every block now sees every block's areaportal-in-water fixup; before, a block saw only those of earlier blocks. This only matters for an areaportal in water that spans blocks. No map checked hit it (canals_07 has both and is unchanged).
- The default thread cap is 8. The basis is a noisy sweep on this box: 4-8 threads best, 16-32 slower.
- #105 is closed as "not worth it" for the density criterion. An occlusion-aware criterion was not tried.

## Merge notes for the coordinator
- Commits: 18dbb770, 8da6a1fb, 33e79441, d9b06b3f, 86feb1da, e1ad59b7, and this handoff. Do not merge `claude/wave2-105-adaptive-experiment`.
- Touches `vbsp.cpp`, `csg.cpp`/`csg.h`, `brushbsp.cpp` (BoundsPlanes split out of BrushFromBounds; atomic ids; a stats reset gated), `map.cpp` (g_bPlanesFrozen), `vbsp.h`, `disp_vbsp.cpp`. Session H edits `brushbsp.cpp` and `portals.cpp` later; my brushbsp.cpp changes are near lines 200-230, 310-360 and 1460. `portals.cpp` is untouched.
- vbsp's log now prints `Block pass: X s (of which serial prepass Y s), N threads`.
- After merge, rebuild vbsp; `vbsp_repro.py` is the identity check.

## Not verified
- No in-game check (output matches the old binary except the zeroed padding, so none is needed for #103).
- No timing on shop26; dev-box wall clock only, with base-versus-base noise stated in the commit.

## Delegation log
- Implementation (#103, the padding fix, the #105 experiment, tools) by the orchestrator (Opus): tightly scoped serial work in files I was reading; the container cannot build MSVC. No implementer delegations.
- reviewer (Haiku) on 70a81a33..33e79441: PASS WITH NITS. Acted on: duplicated brush predicate, `-threads 0`, header/version blind spots in lumpdiff, `.prt` not compared, vbsp_repro path handling. Not acted on: the atomic ids vary run to run (not in output).
- review_fast round 1 (extract, same range): glm-flash approve. Its lows: game-lump flags/version (fixed) and timer wording (fixed). shop26 timed out at 2700 s with no findings; its partial raised one lead, whether other RunThreadsOn callers change with the thread default. Checked: the block pass is the only one in vbsp and the common files it links.
- Round 2 on d9b06b3f (shop26 busy, deepseek-flash stood in): glm-flash approve, deepseek comment. Both flagged, by running synthetic BSPs, that lumpdiff keyed game lumps by id, so order changes and duplicates were invisible. Fixed in 86feb1da.
- Round 3 on 86feb1da: glm-flash approve, deepseek request-changes (2 medium: bytes in lump 35 outside the entries not compared; OSError exits 1). I adjudicated after round three and fixed both in e1ad59b7, verified by running (real lump 35 has no uncovered bytes, so nothing was hidden in practice). e1ad59b7 is tools-only and was not reviewed again.
- review_final (dgx + glm-5.3) not run: it belongs after merge, per CLAUDE.md.

---

# Handoff, 2026-11-12 (wave 2 H: precision, #110 and #104)

Branch `claude/wave2-precision-5371cf`, cut from 70a81a33. Not merged, not pushed. Commits: 4fcfb7cf, 2bfe6f88, c0c762fa, 03b85c6e, 02ce87a4 (code), then three with this file and review follow-ups.

## Done
- **#110, cleanup (4fcfb7cf), no PVS change.** `g_pMergePortals` is gone (mergeflow builds from `portals[2i].winding`); MPI, MySQL and the empty `WaterDist.cpp` are out of `vvis_dll.vpc`; dead declarations, counters and uncalled functions removed; usage text fixed; an option missing its value says so (it used to read past `argv`). PVS byte-identical to 70a81a33 on cm_laststand, d1_trainstation_02_hrcs, ddd_market, validation, validation_harder (default flow; `-threads 1` on the four other than d1; `-mergeflow` against master's `-mergeflow` on all five).
- **#110, stack (4fcfb7cf).** The flow's reserve is derived from `sizeof(pstack_t)` (62 MB), a waiting thread only helps with half of it left, and a chain deeper than its thread holds stops with an Error. Checked with a synthetic corridor and `-trace`: 5,000 deep completes, 8,000 stops at 7,011 with a message, master exits 139. One level is 9,341 bytes. Real maps: deepest chain 7 to 48 portals, under 1 MB (`vvis -v` prints it; the number varies by a portal between runs).
- **#110, portal planes (2bfe6f88).** Not what the issue asked. It asked to reject a portal whose plane comes out zero as corrupt input. Three portals in eleven real maps are like that (first two points equal, or first three on a line) and have real area, one of them 930 by 300 units; vvis, and Valve's SDK vvis, closed them both ways. They now take the plane of the whole polygon. d3_c17_02_camera gains 2 cluster pairs, l4d_smalltown01_caves 43, none lost; the five maps above are unchanged. New fixture `smalltown_repeated_point` in `vvis_sightline` (fails before the change, passes after). `mergeflow.cpp` copies plane and sphere from `LoadPortals`; prtvis's double build has the same rule.
- **#110, sort tie-break (03b85c6e).** Equal might-see counts go in file order. Four maps identical, ddd_market gains one cluster pair (155<->1235 of 271,880) and loses none. `-threads 1` equals the default on four maps. Ties are common (ddd_market: 4,498 of the 7,424 portals vvis flows, which is its 3,712 file portals taken both ways).
- **#110, the rest.** #89's text corrected on GitHub. `-mpi...` options warn and are ignored again, `-threads -1` is accepted again (02ce87a4, from review).
- **#104 (c0c762fa, 02ce87a4): measured, not built.** `docs/vbsp_sliver_measurements.md`. Slivers come from the brush planes, not from float winding arithmetic: no leaf has an empty cell in double on any of eleven maps; a double rebuild of every map's portals has the same cluster pairs, slightly more thin portals, and more short edges when its clip is exact (with VBSP's 0.001 tolerance: fewer on two of the five maps compared, more on two); its PVS is the `.prt`'s with the plane bug fixed, give or take what starting a winding at another corner moves (0, 0, 1, 35 pairs against controls of 1, 1 to 5, 1 to 3, 29). Tools: `experiments/prtvis/sliver_stats.py`, `double_portals.py`, `prt_variants.py`, `.agents/scripts/vvis_pvs_compare.py`, `make_corridor_prt.py`.
- `run_suites.sh`: all six pass on the build at 03b85c6e; `vvis_sightline` and `vvis_mergeflow` again at 02ce87a4.

## Not done
- Float against double timing of the vvis flow. shop26 has no `gameinfo.txt` on either drive, so nothing was run there. Filed as #121.
- `run_vvis_tests.ps1` was not run on this branch (the coordinator's run on master passed `basic` and `harder` and did not get through the production maps). One data point for its open question: the SDK's float `vvis.exe` and this tree's double flow, before the plane fix, write the same cluster PVS on d3_c17_02_camera (871 clusters) and l4d_smalltown01_caves (890), no pair on either side only.
- Nothing was looked at in game.
- Which two tied portals swap for ddd_market's new pair was not traced.
- The help gate in `WaitPortalDone` never engaged in any run, even with the reserve cut to 1.5 MB in a scratch build.

## Decisions nobody has confirmed
- Portals with a zero first-three cross product are opened, not rejected. Default PVS changes (gains only) on maps that have one.
- The tie-break changes ddd_market's PVS by one pair.
- #104 is closed by a "do not build it" result. `Closes #104` is in c0c762fa.
- Follow-ups filed: #120 (VBSP `SplitNodePortals` keeps an unclipped winding when one split side is tiny, drops the portal unfreed when both are, and never prints the tiny count), #121 (the timing), #122 (question for the user: cull portals under about 1 unit wide? they carry up to 1.6% of visible pairs).

## Merge notes
- No overlap with session G: no file under `src/utils/vbsp` is touched. The wave plan below expected H to edit `brushbsp.cpp` for #104 and so said to merge G first; with #104 measured and not built, that ordering no longer binds H. `AGENTS.md` gets two lines in its Scripts list.
- `vvis_dll.vpc` changed and `WaterDist.cpp` is deleted: run `createallprojects.bat` after merging, or the pre-build step fails.
- Erratum: 2bfe6f88's message says the eleven maps have 37,499 portals. They have 39,499.
- Default output changes in two ways, gains only on the maps tested: the PVS of any map with a first-three-collinear portal (`sliver_stats.py <map.prt>` prints the count as `first3_on_line`), and, from the sort tie-break, possibly a pair or so on any map (of five, only ddd_market moved, by one pair, and it has no such portal). Compare before and after with `.agents/scripts/vvis_pvs_compare.py`.

## Environment notes
- The pristine BSP+PRT pairs used for PVS comparisons live in session scratchpads and will be cleaned up: five in `%TEMP%\claude\796c264bc82f8b5e\...\scratchpad\maps`, ten in `%TEMP%\claude\16ad1aec95a8045d\...\scratchpad\maps`. To rebuild a set: compile the VMF with `vbsp_lmo -game <dir>` and keep the `.bsp` and `.prt` before vvis touches them.
- gm_fightspace_redux without func_viscluster: `experiments/prtvis/vmf_viscluster.py strip`, then vbsp with `-game hl2mp` gives 3,737 clusters and 9,580 portals in 14 s; `vvis_optix -mergeflow` takes about a minute.
- Piping a build script through `Select-Object -First N` ends the script when N lines are out, before its later steps run (a copy step was skipped that way).
- A static claim from review that was run and is not so: `#pragma fp_contract(off)` does not outlive `#pragma float_control(pop)` under `/fp:fast` (a probe contracts again after the pop with `/arch:AVX2`), and the vvis project builds without `/arch` anyway.

## Delegation log
- scout (Haiku): map of VBSP's portal and winding code for #104, one attempt, accurate (the two claims that went into docs were re-read in the source).
- Implementation by the orchestrator (Fable), no `implement_single` or `impl`: every change needed the MSVC build and repeated vvis runs on local maps to check, and the measurement scripts were written against results as they came in.
- reviewer (Haiku) on 70a81a33..03b85c6e: pass with nits, no code defect; one valid medium (the write-up's control was confounded), fixed in 02ce87a4.
- review_fast on the vvis range: glm-flash, and deepseek-flash standing in for shop26 (lane_busy). Lows only: the stack comment (both, and Haiku), `-threads -1`, a comment. All fixed in 02ce87a4.
- review_fast on the tooling: glm-flash only (shop26 busy, the stand-in refused as session_busy). Two mediums on the write-up (the control again, and the intro), five script lows. All fixed in 02ce87a4.
- From the coordinator's final review of the wave 1 VVIS range (GLM-5.3 and dgx): `-mpi` hard failure (confirmed by a run, fixed); dead VMPI entries and the missing depth check (already done here); `fp_contract` (run, not so); two-sided sight-line fixtures (not done: over-inclusion is covered by the mergeflow test and the reference comparison).
- Round 2 on 02ce87a4: shop26 (qwen3.8-27b) and glm-flash, both ran. No code finding. One finding on the write-up from both (a range in the conclusion with no table behind it), fixed in the commit that adds this section. Not acted on: `vvis -mpi 4` with no map takes "4" as the map (it failed before as well, with a clearer message), and an `-mpi_...` value that itself starts with `-` is still an unknown option.
- Round 3 on 1637d84d (docs only): glm-flash, and deepseek-flash standing in for shop26 (lane_busy). Two mediums and five lows, all wording in this section and in the write-up (the merge note left out the tie-break; one sentence inferred a double build's output from the exact rebuild's areas; the 7,424 portal count is directed portals). Fixed in the commit after 1637d84d and not sent round again: after round three the orchestrator adjudicates.
- review_final on 70a81a33..02ce87a4: the glm-5.3 leg was refused by the quota gate (5-hour window at 92%) and not forced; Kimi stood in ($0.90): request-changes, one medium and one low. Medium, reproduced on a real map with one portal removed: `double_portals.py --write` silently left out a rebuilt portal that had no `.prt` portal to pair with. Fixed (such portals are appended and counted). The four files the PVS runs used come out byte-identical from the fixed script, so no measurement changes. Low: the plane fallback also runs when the cross product is too small to have a length, which no portal file can produce; kept, and the comment now says so. The dgx leg was still running after 50 minutes when this was written; its findings, if any, go to the coordinator.

---

# Handoff, 2026-11-12 (wave 2J: prop lightmaps under #54, branch claude/wave2-proplm-84e225)

Cut from master 70a81a33. Not merged, not pushed. Design notes for everything below are in `docs/proplm.md`.

## Done (on the branch)
- #56 a28b1a75, fixes 81401319 and e587626e: one exact UV overlap detector (`src/utils/common/uvoverlap.cpp/.h`) for both VBSP and VRAD. Rules: cell centres, an exact tie rule, and fold/repeat/other categories. It lives in utils/common because that is the only tree both tools compile; it reads no files. `vrad -uvoverlap <model|.uvmesh> [-uvoverlapres N]` prints the report. Test written first: `test_proplm_uvoverlap.py`, with hand-drawn fixtures and an exact Python reference; the old detectors failed it, e.g. mirrored 0% where 100% is right. e587626e restores the rebake's pre-#56 decisions: self-overlap is extra claims over the island's unfolded cells, and the skip rule (`movableCells == 0`) was checked equal to the old one.
- #61 09f0fa7b: proplm props are in VRAD's tracer, so they cast and receive shadows. They honour modelscale, disableshadows, disableselfshadowing (and -nossprops) and ignorenormals. Trace id: TRACE_ID_STATICPROP | (0x800000 + job). Test in `test_proplm_vrad.py`: before the fix a blocker cube left the lit faces at 578.1; after, they darken below 0.7x.
- #69 4c54d688 (+$tooltexture in a1a97e75): any other `$` key that names an existing VTF fails the material with a per-prop Warning. Checked with $basetexture2 in a scratch game dir: the fridge keeps its model, gear01 still rebakes.
- #68 113bb421: byte-identical rebaked VTFs are shared, found by CRC and length, then a full compare. On stock content no two distinct models share: gear01 and gear02 use different layouts. A renamed copy of gear01 cuts the pak lump from 1,445,419 to 1,095,551 bytes.
- #67 7385beb2: lower LODs are kept. The corners of a spanning lower-LOD triangle get vertex copies, and the VVD (with fixups), the MDL counts and offsets and both VTX files are rewritten and read back. fridge keeps 1 lower LOD (before, LOD 1+ were off); gear01, gear02, wallpile002a and laundry_dryer001 keep 4. The rebake pak grows from 2,814,848 to 2,823,958 bytes. This also fixes an existing heap corruption: a material that failed part-way freed uninitialised VMT handles (combine_monitorbay).
- #64 280f8079 (Refs only; the issue stays open for the in-game checks noted on it): the five Lua leads were each confirmed under a LuaJIT mock, then fixed.
- Filed from session C's leads, both confirmed: #117 (hull vertices read without VVD fixups: 174 stock $staticprop models get wrong per-mesh hulls), #118 (ComputeConvexHull stackalloc is unbounded). From reviews: #126 (texels used only by kept lower LODs stay unlit, 0-6% of their samples), #127 (lower-LOD decisions read only the dx90 VTX).
- `.agents/scripts/proplm_rebake_ab.py`: compiles the staticpropbounce fixture with rebake props; A/B of two VBSP builds, pak diff.
- Suites on e587626e: all six pass (proplm 6 passed). Re-run after a1a97e75: proplm passes, and the 4-model A/B pak is unchanged.

## Not done / not verified
- Nothing was looked at in game: shadows from proplm props, kept lower LODs, the Lua fixes (#64 lists the checks), and shared VTFs.
- #67's fixup path is covered by one model (laundry_dryer001). Multi-bodypart static props with lower LODs: none were mounted to test.
- Baked lighting is GPU and not bit-deterministic; the #61 test uses ratios.

## Decisions nobody has confirmed
- uvoverlap lives in utils/common. Overlap percentages are now distinct cells, so the logged figures changed (fridge 36.9%, gear01 65.5%, gear02 31.6%); the decisions did not.
- #69 refuses unknown texture keys by default instead of leaving them as they are.
- modelscale is applied to the proplm bake. proplm shadows are always triangles (no -StaticPropPolys gating).
- proplm trace id base is 0x800000. Hit consumers only test the TRACE_ID_STATICPROP flag.
- Under -fast the per-entity proplm warnings now print, because the shadow triangles are added under -fast too.
- #64's draw-failure latch keeps a prop stock until proplm_reload, even after a transient failure.

## Merge notes
- Files: src/utils/common/uvoverlap.*, the vbsp/vrad .vpc files (re-run createallprojects after merging), src/utils/vbsp/proplm_rebake.cpp, src/utils/vrad/vradproplm.*, vradstaticprops.cpp, vrad.cpp (two small hunks: -uvoverlap and the PropLm_AddPolysForRayTrace call), game/addons/proplm/lua, game/bin/x64/unit_test/proplm, docs/proplm.md, .gitignore (uvmesh fixtures), AGENTS.md (one script line).
- Not touched: bsplib.cpp, vbsp.cpp, brushbsp.cpp, map.cpp, vvis, bspwalk.
- Untracked A/B binaries in game/bin/x64 (vbsp_base.exe, vbsp_69.exe, vbsp_68.exe) are scratch; delete them.

## Reviews
- #56: Haiku reviewer (FAIL on the gate metric, now fixed) and review_fast (shop26 + glm-flash; same gate finding plus lows); fixes in 81401319 and e587626e.
- Round 1, #61/#69/#68/#64 and the uvoverlap fixes: glm-flash only, shop26 timed out. Four lows; three fixed in a1a97e75, one accepted (-fast warnings).
- Round 2, #67 and the gate: glm-flash and deepseek-flash approve (shop26 lane_busy); one pre-existing low filed as #127. Haiku reviewer: PASS with nits, its main point filed as #126.
- Round 3, on a1a97e75: shop26 and glm-flash both approve, no findings. This is the final code sha.

## Delegation log
- #64: `impl` (Sonnet). All five leads confirmed and fixed under a mock; first try.
- #67: `impl-opus`, started there as a hard class (VVD/VTX/MDL rewrite). Succeeded first try; it also found and fixed the heap corruption.
- #56 review: `reviewer` (Haiku). Caught the gate decision change, which both third-party lanes also flagged.
- #67 review: `reviewer` (Haiku). PASS with nits; it tested 4 new models.
- Not delegated: #56, #61, #68 and #69 implementation (orchestrator; small and tightly coupled to code already read), and the session C lead checks (scratch scripts over the VPKs).
- Third-party: review_fast x4 (one #56 group, rounds 1-3). shop26 timed out once and was busy once; deepseek-flash stood in once.

---

# Handoff, 2026-11-12 (wave 2 session I: bspwalk queue, branch claude/wave2-bspwalk-dec8c4)

Cut from master 70a81a33. Not merged, not pushed. One commit per issue; `Closes` where done, `Refs` plus an issue comment where not.

## Done
- #47 9d28a485: launcher and trigger-target arcs in pair order, not thread order (deterministic arc lists).
- #44 1e5e4d9b: the GPU visibility selftest checks count as GPU-only. #48 354acaa0: the fixture harness refuses two fixtures with one number. #41 62c61acf: `-arccheck`'s discarded GPU-only flood makes no surf-flight eyes.
- #114 77df85c3: the GPU sweep kernel lists clear pairs and the one-way occluder re-check runs on the CPU in parallel; same tiers and hidden set. No speedup figure is claimed: on this shared box the same d1b face pass ran 9 s to 254 s run to run.
- #43 eb83fa33: load-time rejection of malformed node, brush, leaf, leaf-brush and texdata indices. #115 items 4, 6 c33e8b99: a node graph that is not a tree (or deeper than 4096) is refused; `narrow_*` are null under `-exploits`.
- #37 6656c632: `LumpFits` before every fixed-array `CopyLump` in bsplib (a lump longer than its array is refused instead of overrunning). 4f758ebe: every lump read and every game sub-lump is bounded by the file size, the game-lump dictionary is checked (coordinator lead; master segfaulted on all three corrupt cases).
- #36 ac685a68: why `memoverride.cpp` must stay in bspwalk (reproduced the RtlFreeHeap fault without it), vrad disp radial leak fixed.
- #85 526a724e: `-traceface` replays a GPU verdict on the CPU to name the eye; `seen_from` in the JSON. Mirror pass added in 38a713eb (f17 faces 1 and 8 named).
- #115 item 9 d1ae921c: `bspwalk_runner.py` / `lmoptimizer.py --vphysics` pass `-vphysics` through. Item 1 156ede15: a seam filler whose thick region is empty tries the thin pass (selftest TestSeamSloped, 177 checks).
- Review round 1 fixes 38a713eb: sweep pairs checked in 2^18 windows, shared helper-thread budget in `DirFreeGroups`, `LumpFits` for overlay fades, `LUMP_MAP_FLAGS` bounded (corrupt case `map_flags`: master segfaults, now refused).
- #50 (partial) 1534bffc: mirrors set up lazily under `call_once` when the face pass first reads one. fightspace3 with vis data: vis 240 s -> 116 s, same verdicts.
- dcbacc97 (final review fixes): `s_FaceCapped` atomic; bsplib's legacy ambient-light lumps bounded (Refs #37, #50).
- #34 closed by measurement (coordinator closed it): the face pass is 0.003 s of a 240 s vis stage on vis'd fightspace3; no code change.
- 98a30b81 (final review fix, Refs #43): a map whose leaves name clusters past the vis lump's count is no longer refused; it warns and is walked as one without vis data (no tier A, only more visible). A cluster below -1 is still refused.

## Not done
- #39: options posted for the user (sample budget; the memory guard makes the sample count nondeterministic). Needs a decision.
- #115 items 2, 3, 5, 7, 8: listed on the issue.
- #77: the eye at (-2704,-3400,768) is on rooftop node 14389, reached by a prop arc + slide chain; the vis is right given that reach. Needs a reach decision. Comment posted.
- #38: fall-column flood measured (de_dust: tier A 19, B 321; fightspace3 80.9 s for 1.49M of 1.57M nodes). Comment posted, not implemented.
- #50 remainder: the mirror cull over large vis clusters (the 73 s now inside the face pass on fightspace3) still wants the 512u cube split the issue proposes.

## Decisions nobody has confirmed
- `memoverride.cpp` stays in bspwalk (#36).
- Seam fillers fall through to the thin pass when the thick region is empty, but an empty inset region still ends the pair (safe direction: fewer fillers).
- A leaf/vis cluster disagreement drops the vis data for the whole map rather than refusing it or trusting the PVS for the other leaves.
- Node trees deeper than 4096 are refused (a guess; no installed map comes near).
- The loader now refuses BSPs whose lumps or game sub-lumps lie outside the file. A scan of 2,641 installed BSPs: 7 v19/v20 maps rejected (6 Arctic Adventure, ep1_citadel_00_demo); master already refused ep1_citadel_00_demo and the one Arctic Adventure map run. 1,569 non-LZMA v19/v20 maps (the 38a713eb and 98a30b81 messages say 1,992: that total wrongly included 423 LZMA maps that were never checked): no flags lump over 4 bytes, one map (ep1_citadel_00_demo) with leaf clusters past its vis lump, and none of the 141 on bsplib's legacy ambient path has a short or out-of-file ambient lump.
- With lazy mirrors, `capped_faces` counts only mirrors that were set up (fightspace3 900 -> 110).

## Verification
- `run_suites.sh` all six pass on dcbacc97 (selftest 177, fixtures, lmoptimizer 100, proplm 3, vvis_sightline 2, vvis_mergeflow 2).
- Same tier A/B counts and hidden sets on d1_trainstation_02_hrcs (A 100, B 274), d1b (A 100, B 171) and fightspace3 vis'd and unvis'd, after every performance change.
- Not verified: wall-clock speedups are single runs on a shared box (session G owns timing); nothing was run on shop26.

## Merge notes
- Master has moved (K, G, H, J merged) but nothing on master touches `src/utils/bspwalk`, and `bsplib.cpp`, `lmoptimizer.py`, `bspwalk_runner.py` are unchanged on master since 70a81a33 (coordinator), so this should merge cleanly. bsplib changes affect every tool: rebuild the whole solution.

## Delegation log
- All implementation by the orchestrator (Opus): every change needed the MSVC/OptiX build and GPU runs inside the loop, so no `implement_single` or `impl`.
- `reviewer` (Haiku) on the first commits: pass with seven nits, all addressed in 38a713eb or noted above.
- `review_fast` round 1 (70a81a33..4f758ebe extract): glm-flash request-changes (overlay fades and map flags guards, sweep memory, thread fan-out), deepseek-flash stand-in (shop26 lane busy) agreeing plus the mirror seen_from gap; all fixed in 38a713eb.
- `review_fast` round 2 (4f758ebe..1534bffc): shop26 approve (ran a g++ model of the windowed OR and the budget), glm-flash approve (ran models of both); one low and three notes, fixed in 51a872c6 (comments and usage only).
- `review_final` (70a81a33..51a872c6 extract): glm-5.3 comment, dgx request-changes, kimi (the hedge, dgx past 50 min) request-changes. Adjudicated by the orchestrator (round 3):
  - Leaf-cluster gate refusing a map the face-cluster path tolerates (glm-5.3 and dgx, medium): fixed in 98a30b81 (drop the vis data instead); its own glm-flash `review_single`: approve.
  - `s_FaceCapped` byte-write race from lazy mirrors (kimi medium, glm-5.3 low, shop26 and glm-flash notes in round 2): made atomic in dcbacc97 (glm-flash `review_single`: approve, no findings).
  - Legacy ambient-light lumps not bounded (kimi medium): `LumpInFile` plus a length check against the leaf count, dcbacc97.
  - Refuted: DirFreeGroups' budget from a -1 thread sentinel (dgx low): `ThreadSetDefault` replaces -1 with the processor count before the face pass.
  - Accepted, not changed: the first large re-check takes the whole helper budget (dgx low, performance only); `dirGroups`, `segs` and `blocked` keep their peak capacity (dgx note, bounded by the 2^18 window); the listing sweep kernel no longer stops at the first clear pair (dgx note, by design: the list is needed for the one-way re-check); traced faces made visible by a capped GPU sweep get no `seen_from` (kimi medium, diagnostic only: noted on #85).

---

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
- `run_vvis_tests.ps1 -All -SkipVisualCheck`, the first run of the three-way harness since #89 made the flow double, was stopped by hand after 84 minutes because it held every core on the box session G times on. What it showed: `basic` and `harder` pass, with `vvis_optix -threads 1` byte-identical both to the SDK's `vvis.exe` and to the default thread count (so on those two maps double changed nothing). `production` (d1_trainstation_02_hrcs) could not start: its VMF is not in `bsp_unit_tests`. `production_harder` (fightspace3_unittest) never left phase 1: the SDK reference `vvis.exe` was still running at 84 minutes. `visibility` was not reached. Whether the reference comparison still holds within tolerance on a production map is therefore still unanswered; run those two tests on a quiet box.
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

## #39 session, 2026-11-14 (branch `claude/issue-39-optimization-ba61de`, not merged, not pushed)
Goal (user): bspwalk's tier B eye set must not depend on free RAM, and must not trade quality for memory. Five steps, posted on #39: (1) the rule, (2) smaller eye set (shell, clipped path chords), (3) smaller sample, (4) samples on demand behind a cache, (5) spend the headroom on quality (refine the eyes of faces still hidden).

- Stage A, e526a428 + 16814726: shell sampling from per-line spans, path chords clipped to outside the poly pieces, `-eyeconfirm` default 24. d1_trainstation_02 at 8u: 25.6M samples to 4.2M; at 4u 116.9M to 14.8M. Tiers the same at any thread count.
- Stage B part 1, 364702db + 4cf8c1d8: `EyeStore` (`bw_eyestore.*`), tiles made on demand, LRU cache (`-eyecache`, default 4096 MB), 64-bit eye ids, `-eyeverify`, suite `bspwalk_cache`. Tiers identical to the build before the store at four cache sizes and two thread counts.
- Measured with the guard switched off (throwaway build): fightspace3 at a true 8u runs in 502 s, 8.9 GB peak, 261.8M samples (the full lattice estimate is 11.0 billion); the old code widened to 30.5u. The face pass is 350 s of that: every face scans every straddling tile twice (count, then pick), bound by memory bandwidth.
- Known quality item, to settle at the end: under the shell three faces of d1 turn hidden besides the ten leak faces of #134 (1604 at 8u; 2608 and 7154 at 4u). Each was seen only from an interior lattice point through nodraw brush sides. Step 5 is meant to recover them.
- Issues filed: #134 (a grazing ray inside one brush's 1/32u shrink margin counts as a sight line), #137 (a sliver node's eye hull runs thousands of units past its polygon).
- Tools: `.agents/scripts/bspwalk_tier_diff.py` (tier diff between runs or builds; `--same`).

### Delegation log (#39)
- `scout` (Haiku) x2, eye volume and eye consumers fact sheets: accurate; the consumers scout also found the worktree lacked vbsp/vvis binaries.
- `impl-opus`, stage A: started at impl-opus, not `implement_single` (a rewrite of the eye sampler that has to iterate against the MSVC/CUDA build, the GPU and corpus runs). Passed every check first attempt, one context reset; found and fixed its own tile/node-box bug (26 clusters lost every sample), root cause filed as #137.
- `reviewer` (Haiku), stage A: no real defect; verdict FAIL on an item the brief deferred to stage B; useful minors (doc and comment errors).
- `review_fast`, stage A: shop26 `lane_busy`, deepseek-flash stood in; glm-flash and deepseek-flash each rebuilt the functions in standalone harnesses; one medium (perf) and lows, all carried forward.
- `impl-opus`, stage B part 1: same reason for the tier. Passed every check first attempt; three mutants; deviated once, on purpose and reported (cluster facts need every tile, so `ComputeAll` up front).
- `reviewer` (Haiku), stage B part 1: found an undisclosed rule change (a sample that rounds into a cluster no piece touches no longer adds that cluster) and a wrong comment about it; zero such samples on any measured map, rule kept and documented. Also a copyable pin set (latent double release) and a test that matched message wording. All fixed in 4cf8c1d8.
- `review_fast`, stage B part 1: shop26 (Qwen3.8-27B) and glm-flash both finished, no stand-in. Two lows: the cluster rule change (weighed, accepted) and no guard on the store's pin count (sent to the part 2a implementer). glm-flash stress-tested a standalone model of the Pin/Evict protocol under ASan/TSan and simulated the spare-sample compaction at different cadences: no defect.
- `impl` (Sonnet), stage B part 2a (guard removed, complete tile lists, needed clusters only): started at `impl`, not `implement_single` (the bar is tier identity on corpus runs with the MSVC/CUDA build and the GPU, which the container lacks). Passed every check first attempt, no escalation, four mutants each caught by a named check; also fixed a usage line that printed `5%` wrong.
- `reviewer` (Haiku), stage B part 2a: pass with nits, no defect. It traced the tile-list claim through VBSP's leaf bounds (`portals.cpp`, `writebsp.cpp`) and the needed-cluster set against every reader. Worth acting on: a sample outside its cluster's tile list now stops the run; falling back to generating every tile, with a warning, is kinder and as safe (to do after part 2b).
- `review_fast`, stage B part 2a: shop26 and glm-flash both finished, no stand-in; one low between them, the same point as the Haiku reviewer (the tile lists rest on VBSP leaf boxes holding their cell). glm-flash brute-forced the tile-range arithmetic (9.6M cases) and re-implemented the facts merge (400 random trials): no failure. Checked here against the runs: 0 samples outside their cluster's list on the fixtures, d1 at 8u and 4u, and all 261.8M samples of fightspace3.
- `impl-opus`, stage B part 2b (batched, tile-major face pass; sweep kernel with a rank buffer): impl-opus because it restructures the face pass and changes OptiX device code. Tiers identical to part 2a at every cache, batch and thread setting, first attempt, no escalation; two kernel mutants caught. It corrected my brief twice: fightspace3's 350 s face pass was the per-mirror setup, not the faces, and the small-cache memory peak came from the generator's scratch, not from pinned tiles. One target missed and explained: d1 at 4u face pass 3.6 s against 3.4 s, bound by the CPU re-test of the kernel's clear pairs against one-way occluders.
- Measured after part 2b (2026-11-14): fightspace3 at 8u, 154 s total (539 s after part 2a, and 30.5u before #39), face pass under 1 s, 9.2 GB peak; with `-eyecache 512` 293 s and 6.7 GB, same tiers. d1 at 8u unchanged (9.7 s), at 4u 12.1 s (13.6 s).
- `reviewer` (Haiku), stage B part 2b: pass with nits. It ran things: tier diffs on all 29 fixtures at the default and at `-eyespacing 2` (which reaches thinning and the sweep), batch sizes 1 to 3, `-eyecache 1`, `-cpuvis` against the GPU kernel, all identical. Found a gap older than this work (a capped displacement mirror whose picks all fall behind its plane was skipped instead of swept), a memory comment that overclaimed and a write before a check; all sent to the stage C implementer.
- `review_fast`, stage B part 2b: shop26 and glm-flash both finished, no stand-in. shop26: no finding. glm-flash: one medium, static, that ran nothing: the sweep kernel launches (rays, eyes, 1) and "CUDA's grid Y limit of 65,535" would fail every chunk over that. Refuted by the run: d1 at 4u sweeps 6.8M eyes in chunks of up to 262,144 with `gpu_launch_failures` 0 (an OptiX launch size is not a CUDA grid; the fan kernel launches the same shape with millions of eyes).
- `impl-opus`, stage C (refinement levels, `-eyerefine`, default 2): impl-opus because the generator's spacing had to become per store and the face pass re-entrant per level. Every check passed first attempt, no escalation; one mutant. It found that the half-spacing lattice does not contain the coarser one where under-cover chords are concerned and fixed it (a level takes the coarser levels' chords too), and it reported the cost plainly: the generator's scratch, not the cache, sets the memory peak at fine levels.
- Measured after stage C (2026-11-14), d1_trainstation_02 at the default: 160 tier B hidden (187 with `-eyerefine 0`); level 1 (4u) sees 24 of 187 faces in 5.6 s, level 2 (2u) 3 of 163 in 15 s, level 3 none. 30 s and 5.5 GB peak in all, against 9.7 s and 1.8 GB without levels. Against the tool before #39 with `-eyeconfirm 24`: at 8u (11 s, 2.4 GB) and at 4u (40 s, 5.4 GB) the new default hides only the ten leak faces of #134 and 8478 (which the old GPU fan marked through its margins; its CPU replay crosses it with no ray), and sees 27 and 4 faces more. With `-eyerefine 1` face 1604 is lost as well.
- Open after stage C: the three faces level 2 adds on d1 (1604, 5567, 8628) and 2609 at level 1 are seen along rays within 0.0001u of a brush edge where brushes stack (the #134 kind); the peak at fine levels is the tile generator's per-thread scratch (15 GB at `-eyerefine 3` on d1); d1 at 2u has 112 samples in tile 322 that lie in no cluster an eye piece touches.
- `final-review` (Fable low), whole branch at 52af091c: ship with follow-ups, read only. It found what four fast gates had not: the shell's proof needs the eye volume to hold no occluder, which fails in sky-camera layers (the mapped copy is not carved by skybox brushes) and where a piece reaches across an occluder; refinement cannot recover that. Also: the levels' early stop trades quality for time, `-eyeverify` checks level 0 only, unplaced samples never reach the JSON, a few leftovers. All sent to a stage D implementer; fightspace3 at the default was run after it asked (159 s, 9.0 GB, no level runs). Its view on the default: keep `-eyerefine 2`.
- Follow-ups filed during #39: #138 (`-eyemaxsamples` estimate), #139 (rays from the face into the eye volume), #140 (capped-mirror sweep and mirror eye memory), #141 (one-way occluders on the GPU), #142 (generator scratch sets the peak at fine levels), #143 (samples in a cluster no eye piece is marked for).
- `reviewer` (Haiku), stage C: pass with nits. Ran d1 at `-eyespacing 32` across the levels (179, 178, 177 hidden, never a face added to the hidden set), `-eyerefine 0` against the previous build on d1 and 12 fixtures, and the cache, thread and batch variants: all as required. One doc claim was false as written (the shell at S/2 does not hold every sample the shell at S keeps; the lattice does, and the cluster rule covers the rest); sent to stage D with two naming and comment nits.
- `review_fast`, stage C: shop26 and glm-flash both finished, no stand-in; two lows. shop26 found the same false containment sentence as the Haiku reviewer, with a cleaner counter-example (two reviewers agreeing); glm-flash found `-eyerefine` missing from one option list in the doc. Both sent to stage D.
- `review_final`, whole branch at 52af091c: dgx (Qwen3.8-flash, 39 min) and glm-5.3 (15 min) both finished; the Kimi hedge was not needed. One medium, static: the shell on/off decision is written in three places that disagree for a base spacing between 16u and 32u, and the selftest's refinement check takes the path production does not. Six lows: stale names and a comment that overclaims on eviction, `EyePinSet` used by the selftest only, the thinning path never reached by the cache suite, a no-vis diagnostics question (my fightspace3 runs do not show it), and the tile-list residual risk already documented. All sent to stage D.
- `impl-opus`, stage D (the shell's gap next to occluders inside the eye volume, plus the final-review items): test first, it built fixture f30 (a sky room behind a slab inside the mapped eye volume) that the stage C build fails and the tool before #39 passes, then the keep rule; every check passed. The rule as I briefed it ("within one lattice step of any occluder") was too wide: fightspace3 went from 261.8M to 350.4M samples and out of the default cache (141 s to 265 s). It did not trace why although the brief asked; resumed with a follow-up to attribute the keeps and to count only occluders that reach into the eye volume.
- Stage D follow-up, same `impl-opus`: it attributed the keeps as asked. Not what I expected: 94% of fightspace3's and 86% of d1's lie more than 1u inside a solid, because a few huge eye pieces run through solids (water volume hulls, the #137 node piece, arcs through brushes whose water material is not mounted). So the penetration filter I had planned was dropped, and the rule became a skin: within one lattice step of an occluder's surface, either side. The over-reaching pieces are filed as #144.
- Stage D, final form (same `impl-opus`, third round): the skin rule. Every check passed; no escalation. Measured: fightspace3 271,979,401 samples (261,756,589 without the rule), no tile made again, same tiers; d1 4,224,150 samples at 8u (4,155,568), same tiers as stage C at every cache, thread and batch setting; against the tool before #39 at 8u and 4u still only the ten leak faces and 8478; `-eyeverify` passes at levels 0, 1 and 2; fixture f30 fails on the stage C build (faces 4 and 5 hidden) and passes now.
- `reviewer` (Haiku), stage D: verdict FAIL on a test-only helper (`SampleEyePiece` passed the spacing where the layer scale now goes, so the older selftest checks sampled with the shell off; no production caller, no tier affected). A real slip and a one-token fix, with a selftest check added that pins the helper to the shell. Also right: the skin's argument overclaimed for pencils narrower than a step (it measured f30 hidden at 12u without levels), and `-eyeverify` shares the keep rule's arithmetic with what it checks. It ran base, stage C and stage D over all 30 fixtures: nothing the old tool or stage C sees is hidden by stage D. Fixed by the orchestrator (under 30 lines); d1 tiers, sample counts and ray counts unchanged by the fix.
- `review_fast`, stage D: shop26 and glm-flash both finished, no stand-in; three lows. glm-flash found the same `SampleEyePiece` slip as the Haiku reviewer (two reviewers agreeing) and stale numbers in the f30 fixture's prose; shop26 noted that the keep rule treats a face tilted under 1e-9 as horizontal (unreachable, left as it is). Fixed with the Haiku findings.

### State at the end of the #39 session (2026-11-14)
- Branch `claude/issue-39-optimization-ba61de`, ten commits over fc306f0f; not merged, not pushed. The finishing commit carries `Closes #39`.
- What bspwalk does now: the tier B eye set is the shell of the lattice plus a one-step skin around occluders inside the eye volume, made per 256u tile on demand behind `-eyecache` (default 4096 MB); nothing reads free memory; only the clusters undecided faces can be seen from are generated; the face pass is batched and tile-major; faces still hidden are tested again at half and a quarter of the spacing (`-eyerefine`, default 2); `-eyeverify` checks every tile at every level.
- Numbers to hold later work against (RTX 3090, 32 threads, quiet machine): d1_trainstation_02 at the default 160 tier B hidden, 30 s, 5.5 GB peak, 4,224,150 samples at 8u; `-eyerefine 0` 187 hidden, 9.7 s, 1.8 GB. fightspace3 at the default: 271,979,401 samples, every face visible, about 150 s, 9.4 GB.
- Suites: `run_suites.sh --only bspwalk_selftest,bspwalk_fixtures,bspwalk_cache,lmoptimizer` (selftest 204 checks, 30 fixtures, cache 37 checks). The fixtures suite takes about a minute now that both levels always run.
- Decisions the user may want to revisit: the default of two levels (level 2 costs 15 s and 2.5 GB on d1 for three faces of the #134 kind; `-eyerefine 1` loses face 1604, which the old tool saw); the levels no longer stop at an empty level (the approved wording said they would).
- Next, in order of value: #144 (eye pieces through solids) and #137, then re-check #134's faces; #143 (unmarked cluster); #142 (generator scratch); #141; #140; #139; #138.
- `reviewer` (Haiku), the finishing commit 7bcb8d50: pass with nits. It checked that the wider grown side can only keep more heights, reasoned that the new selftest check fails on the code before the fix (729 against 386), and re-measured f30 at seven base spacings. Three prose numbers in the fixtures were still off (one of them mine); corrected.
- `review_fast`, the finishing commit: shop26 and glm-flash both finished; three lows, text only (the keep rule's comment and doc named the rounding margin on one side only; the f30 prose again, where the generator's constant is 85 and the walker's own highest jump eye is 85.33, so the prose now gives both). Corrected in the commit after it; comment, docs and fixture prose only, so no further gate.

### #39 branch after merging master (cc840818), 2026-11-14
- Master had gained the sliver prism fix (#77: a prism wider than its floor is no piece) and `-traceeye`. Conflicts in `bspwalk.cpp` and `bw_eyes.cpp`, resolved by the stage D `impl-opus`: both sides kept; `-traceeye` now tests the piece itself (the branch makes no per-piece samples), same printed lines. It found and fixed a false match of its own first version (a flat hull's planes do not bound its flat side).
- New comparison baseline: master's own build (full lattice, guard, sliver fix). The numbers in "State at the end of the #39 session" above are from before the merge; these replace them.
- d1_trainstation_02 at the default: tier A 36, tier B 275 (level 1 sees 25 of 301, level 2 sees 1 of 276), 3,030,274 samples at 8u in 404 tiles, 82 s, 4.6 GB peak. Master at 8u with `-eyeconfirm 24`: 36 + 289, 147 s, 2.7 GB; at 4u: 36 + 266, 42 s, 5.3 GB. The merged build hides only the ten #134 leak faces that master at 8u sees (and 2431, the same kind, against 4u), and sees 24 and 2 faces more. Same tiers with `-eyecache 8`, `-threads 4`, batch 1 and on a rerun; `-eyeverify` clean at levels 0, 1, 2.
- Why d1 is slower than before the merge (30 s): the sliver piece's false eyes kept about 150 faces visible; they are hidden now, and a hidden face is the expensive kind (every candidate swept). Master's own 8u run went the same way.
- Of the four grazing-ray faces noted on #134: 2609 and 5567 are hidden now and 8628 is tier A (their eyes came from the removed piece); 1604 is still seen at level 2, 0.00002u outside brush 992.
- fightspace3 at the default: 271,981,047 samples, every face visible, 154 s, 9.4 GB.
- Suites: selftest 205 checks (master's h1s included), 30 fixtures, cache 37, lmoptimizer.
- Reviews of the merge (07f32636): `reviewer` (Haiku) pass with nits; it ran `-traceeye` on the merged build and on master's at a real eye position (the same four pieces), at a far point (nothing) and at the tolerance's edge, and compared the JSON with and without the option (identical). `review_fast`: shop26 and glm-flash, three lows. All in the log-only trace: its lines could interleave across threads (two reviewers), the wording said "would give a sample" where it tests the piece, and the doc's option list lacked it. Fixed by the orchestrator in the commit after the merge: one block per piece, the node index cleared when its node is done, the wording. Suites pass; d1 tiers unchanged.
- 2026-11-14, on the user's "Merge, don't push": the #39 branch is merged into master with a merge commit (tag `pre-issue39-merge` marks master before it). Pushed to origin the same day on the user's "Push master", with the four commits of the #77 sliver fix that were waiting on master; `Closes #39` was the only closing keyword in the range.
