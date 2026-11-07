# Lightmap carving redesign (#52)

Results and decisions live on issue #52. This file holds the method and the delegation log.

## Step 0: measuring the ceiling

`game/bin/x64/carve_ceiling.py EVIDENCE.bsp --cost OPTIMIZED.bsp [--json out.json]` (method in its docstring).

- EVIDENCE: the lit baseline lmoptimizer caches in `<map dir>/.lmopt_cache/`.
- OPTIMIZED: the lmoptimizer output VMF compiled with `vbsp_lmo -emitsideids -game <dir>`. VBSP only, no VIS/RAD needed.
- Envelope check: set every `"lightmapscale"` in the optimized VMF to 64 and run `vbsp_lmo -countverts -game <dir>`. The difference from the optimized count is the most any scale change can remove.
- Maps used on 2026-11-07, `-game` = `Half-Life 2 Deathmatch/hl2mp` (GarrysMod has no bin/x64; lmoptimizer refuses it): `unit_test_maps/d1_trainstation_02_hrcs.vmf` (cached baseline) and `bsp_unit_tests/fightspace3_unittest.vmf` (no cached baseline; an uncached compile hits #53, worked around with short dotless scratch names).
- Runtime is about 45 s on trainstation and a few minutes on fightspace3, all in Python.

## Delegation log

- 2026-11-07 reviewer (Haiku): carve_ceiling.py. FAIL, one major: sides promoted finer than the baseline were credited to carving. Fixed. 6 minors, fixed or noted. It ran the tool and cross-checked decode, indexing and scales against bsp_reader and the VMF. Useful.
- 2026-11-07 review_fast (shop26 + glm-flash): same file. Two mediums (early break in the solver-room loop; already found by Haiku), lows on spanning faces, unlit union, int truncation and zero-divide. All fixed. Both lanes built numpy shims and ran synthetic cases.
- 2026-11-07 review_fast round 2 (9052efb2): shop26 medium "leafface count wrong" was false. It was static only; the count matches VBSP's LEAFFACECOUNT exactly on both maps. glm-flash medium "group scale pooled at min" was real in principle but measured at 0 mixed-scale groups on both maps. A warning now covers it (14c7e6cf). shop26's low on the mixed_face_verts basis was left as is: the field is informational.
- 2026-11-07 reviewer (Haiku): the 14c7e6cf guard. PASS WITH NITS (warning wording on degenerate scale-0 or unmeasured groups); adjudicated as not worth a round 4.
