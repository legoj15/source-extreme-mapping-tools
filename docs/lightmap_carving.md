# Lightmap carving redesign (#52)

Results and decisions live on issue #52. This file holds the method and the delegation log.

## Step 0: measuring the ceiling

`game/bin/x64/carve_ceiling.py EVIDENCE.bsp --cost OPTIMIZED.bsp [--json out.json]` (method in its docstring).

- EVIDENCE: the lit baseline lmoptimizer caches in `<map dir>/.lmopt_cache/`.
- OPTIMIZED: the lmoptimizer output VMF compiled with `vbsp_lmo -emitsideids -game <dir>`. VBSP only, no VIS/RAD needed.
- Envelope check: set every `"lightmapscale"` in the optimized VMF to 64 and run `vbsp_lmo -countverts -game <dir>`. The difference from the optimized count is the most any scale change can remove.
- Maps used on 2026-11-07, `-game` = `Half-Life 2 Deathmatch/hl2mp` (GarrysMod has no bin/x64; lmoptimizer refuses it): `unit_test_maps/d1_trainstation_02_hrcs.vmf` (cached baseline) and `bsp_unit_tests/fightspace3_unittest.vmf` (no cached baseline; an uncached compile hits #53, worked around with short dotless scratch names).
- Runtime is about 45 s on trainstation and a few minutes on fightspace3, all in Python.

## Step 2: per-side evidence scale (opt-in)

The plain solver raises a whole side only when its baseline lightmap is exactly flat (or, opt-in, a monotonic gradient) AND dark. Step 0 showed many lit, smooth sides reproduced at a coarser scale. `--smooth-tolerance` lets the solver use that.

- `game/bin/x64/lightmap_evidence.py`: `coarsest_reproducible_scale(ld, tolerance)` takes a `bsp_reader.FaceLightmapData` and returns b*k for the largest factor k in (2, 4, 8, 16, 32) at which EVERY luxel of EVERY sub-face is within `tolerance` (display space, out of 255) of the bilinear reconstruction from every k-th luxel. b is the finest sub-face scale. 0 means no evidence: nothing passes, never-visible, no sub-faces, or a sub-face with missing/wrong-length luxels or no `lightmap_vecs`. `evidence_scales(face_data, tolerance)` maps it over a dict and keeps the non-zero ones. `_display` and `_recon_error` moved here from `carve_ceiling.py`, which imports them.
- `lmoptimizer.py --smooth-tolerance [T]` (default off; bare flag = 1.0, strict; 4.0 is loose; the GUI field "Smooth tol." sets it, 0 = off): computes the dict from `face_data` just before `solve_with_vbsp(evidence_scales=...)` and prints one summary line. The calibration fallback (`budget_solver.py`) ignores it and warns.
- `vbsp_solver.solve_with_vbsp(evidence_scales=)`: per solver entry the evidence is `min(evidence, max_useful_scale)`, 0 for an emissive side, and for a coplanar group the minimum over the group's solver entries (an entry without evidence gives the whole group 0). It is a floor on entries that are neither uniform nor gradient (`max(scale, min_scale, evidence_scale)` in the Phase 1 scan, the scale=1 early exit and the Phase 2 assignments). Entries with a floor are not Phase 2 promotion candidates.
- `VBSPSolverResult.faces_evidence_raised`: entries whose evidence floor ends above `max(baseline scale, min_scale)`, i.e. the evidence changed their scale. Shown in the results box as "Evidence raised" whenever the flag is set.
- Test: `unit_test/lmoptimizer/test_smooth_evidence.py` (the reconstruction rule on synthetic sub-faces, and the solver floor with `count_vertices` mocked).

## Delegation log

- 2026-11-07 reviewer (Haiku): carve_ceiling.py. FAIL, one major: sides promoted finer than the baseline were credited to carving. Fixed. 6 minors, fixed or noted. It ran the tool and cross-checked decode, indexing and scales against bsp_reader and the VMF. Useful.
- 2026-11-07 review_fast (shop26 + glm-flash): same file. Two mediums (early break in the solver-room loop; already found by Haiku), lows on spanning faces, unlit union, int truncation and zero-divide. All fixed. Both lanes built numpy shims and ran synthetic cases.
- 2026-11-07 review_fast round 2 (9052efb2): shop26 medium "leafface count wrong" was false. It was static only; the count matches VBSP's LEAFFACECOUNT exactly on both maps. glm-flash medium "group scale pooled at min" was real in principle but measured at 0 mixed-scale groups on both maps. A warning now covers it (14c7e6cf). shop26's low on the mixed_face_verts basis was left as is: the field is informational.
- 2026-11-07 reviewer (Haiku): the 14c7e6cf guard. PASS WITH NITS (warning wording on degenerate scale-0 or unmeasured groups); adjudicated as not worth a round 4.
