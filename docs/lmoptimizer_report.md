# lmoptimizer scale report (#81)

lmoptimizer decides every side's lightmapscale. The report records why, so "why is side X / brush Y at scale Z" is answered from a file instead of by rebuilding the pipeline (the motivating case: d1_trainstation_02 side 108783, a 672x1672 floor, stayed at baseline 2 because the skip-ahead pass stepped over it; promoting it cost about 890 chop cells).

## The file

`<output stem>.lmo_report.json`, written next to the output VMF, atomically, right after the VMF is published. Not written on `--dry-run` or on any failure. Writing it never fails the run: a problem prints `WARNING: could not write the scale report ...` to stderr and removes an older report beside the output (it would describe a different run); if that removal fails too, a second warning says the older report is still there and does not match. A successful write prints `Scale report: <path>`.

Layout: one JSON object with `run` (indented header) and `sides` (one line per side, keyed by the side id as a string, sorted by id). One side per line means `grep '^"108783":'` works on a multi-megabyte file.

Sides listed: every side of the final tree that has a face extent (so tool and entity sides appear, with decision `skipped`), plus sides WITHOUT a face extent that have a floor or whose scale a stage changed: chiefly a displacement whose CSG polygon vanished, which is floored at 16 (`disp_no_geometry`) and written to the tree though the solver never saw it. Those carry `flags.no_geometry`, no `extent_s`/`extent_t` and no decision keys. Untouched sides without an extent are not listed. Ids are those of the OUTPUT VMF. With `--chop`, a carved brush gets new side ids; each piece carries `flags.carved_from` = the original id and inherits that side's `input_scale`.

## `run`

`report_version`, `created` (UTC), `input`, `output`, `mode` (`auto-compile` or `bsp`), `bsp` (the `--bsp` file, null in auto-compile), `solver` (`vbsp` or `calibration`), `args` (the flags that change a side's scale: vertex_budget, headroom, max_lightmap_dim, light_budget, dark_threshold, detail_min_scale, smooth_tolerance, coplanar, chop, visibility_check, bspwalk_tiers, promote_order, skip_ahead_calls, texture_shadows, rtx, allow_unlit), `result`.

`result` is the solver result's plain fields. VBSP solver: `baseline_scale`, `vertex_budget` (after headroom), `vbsp_vertex_count`, `vbsp_leafface_count`, `vbsp_face_count`, `binding_limit`, `vbsp_calls`, `faces_promoted`, `faces_at_baseline`, `faces_pre_promoted`, `faces_evidence_raised`, `promote_order`, `skip_ahead_cap` (the `--skip-ahead-calls` given), `skip_ahead_used` (calls it spent), `promotion_units` (units ranked), `prefix_units` (units the prefix search promoted before skip-ahead). Calibration fallback: the budget solver's counts.

## A side record

| key | meaning |
|---|---|
| `brush_id`, `entity` | owning brush; entity classname, `""` = worldspawn |
| `material`, `extent_s`, `extent_t`, `has_displacement` | what the solver saw |
| `input_scale` | the scale in the input VMF as parsed, before auto-compile applied its baseline (16 if the key was missing) |
| `solver_scale` | what the solver chose (null if it never saw the side) |
| `final_scale` | the tree just before publishing: includes floors, strict unification and light-budget raises |
| `changed_by` | stages after the solver that moved it: `strict_unify`, `seam_unify` (calibration only), `floors`, `light_budget`. On a side the solver did not assign whose final differs from its input with none of those, `before_solver`: auto-compile applied the floor to the tree up front (vanished or skipped displacements) |
| `floor`, `floor_kinds` | the lowest scale any stage may leave it at, and why: `detailtype`, `displacement`, `texlight`, `disp_no_geometry`, `culled` |
| `flags` | only the true ones: `never_visible`, `raw_never_visible`, `culled`, `texlight`, `carve_promoted`, `no_geometry` (no face extent; see above), and `carved_from` (original side id) |
| `decision` + the keys below | the solver's reasoning |

Solver keys, on every side the VBSP solver saw: `decision`, `priority` (lighting detail x visibility), `variance`, `luminance_range`, `min_scale` (the solver's minimum: detailtype, displacement extent, floors), `evidence_scale` (0 = none), `max_useful_scale` (coarsest scale at which it is not chopped), `is_uniform`.

Added for sides in a promotion unit (Phase 2): `scale_at_baseline`, `scale_if_promoted`, `chop_cells_at_baseline`, `chop_cells_promoted` (sub-faces VBSP chops the side into; verts and leaffaces grow with them, so this is the cost unit), `rank` (1-based place of the unit in the final promotion order; by value/cost under `--promote-order value`, by each unit's best-member priority under `priority`; the script words it from `run.result.promote_order`), `unit_size`, `value`, `cost` (this side's promotion value and estimated added cells plus a 0.25 base), `unit_value`, `unit_cost` (the unit's sums, which is what ranking uses).

Added for `stepped_over`: `headroom_at_skip` (vertex budget minus the worst count of what was promoted at that moment), `binding_limit` (verts, leaffaces, faces, or `vbsp_rejected`, from the trial that did not fit), `trial_worst_count` (that trial's worst count), `stranding` (true on the unit the prefix search stopped at). Added for `no_gain`: `unit_decision` (what happened to the unit it rides in).

## Decision values

This list is the vocabulary; the solver writes the strings in `vbsp_solver.py` (`_base_record` and the decision loop at the end of `solve_with_vbsp`), and `skipped` is added by lmoptimizer for sides the solver never saw.

- `uniform`: pre-promoted to its max useful scale (perfectly uniform and dark baseline lighting).
- `evidence_held`: an evidence floor (`--smooth-tolerance`) at or above the baseline holds it; no promotion slot.
- `no_gain`: a minimum scale holds it at or above the baseline; it rides along with its unit but never moves.
- `promoted_prefix`: its unit was inside the prefix of the promotion order that fit the budget.
- `promoted_skip_ahead`: its unit was added by the skip-ahead pass.
- `stepped_over`: its unit did not fit on top of what was promoted and stays at baseline. Either the stranding unit (the prefix ended here) or one the skip-ahead pass passed over.
- `not_reached`: skip-ahead stopped (call cap used up, or `--skip-ahead-calls 0`) before its unit was tried. Left at baseline, not proven not to fit.
- `all_fit_scale1`: every side fits at scale 1, so no promotion ran.
- `over_budget_scale64`: no global scale up to 64 fits; the last counted assignment is returned.
- `skipped`: tool material or non-geometry entity; the solver never saw it (`skip_reason`). Calibration reports have only this value; their other sides carry no `decision`.

## Query script

`game/bin/x64/lmo_report.py REPORT <command>` (standard library only; the module docstring has the same usage):

- `side ID`: explains one side in plain English (decision, cost, budget left, rank, later stages, floor, flags). An id that `--chop` carved away lists its pieces; a `no_geometry` side is explained as floored without a solver decision.
- `brush ID`: every side of the brush.
- `summary`: run header and counts per decision, per final scale and per stage outside the solver (`changed_by`, including `before_solver`).
- `skipped [--top N]`: `stepped_over` sides by rank, with cost and headroom. These are the "why was this not promoted" candidates.

Exit 1 on an unreadable report or an unknown side or brush.

## Caveats

- `headroom_at_skip` and `binding_limit` describe the moment the unit was passed over, not the final state; later units may have used the headroom.
- Costs are the solver's chop-cell estimate, not VBSP counts. VBSP counts decide fits; the estimate only ranks.
- Everything in `decisions` is bookkeeping: it does not change which scales the solver picks or how many VBSP calls it makes.
