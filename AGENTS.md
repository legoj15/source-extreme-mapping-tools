# Agent notes

More rules live in `.agents/rules/` and workflows in `.agents/workflows/`.

## Scripts

- `bash .agents/scripts/run_suites.sh [--only a,b]`: the quick regression suites (bspwalk selftest and fixtures, lmoptimizer, proplm, vvis sight-line and mergeflow) against the binaries in `game/bin/x64`, one pass/fail line each. Run it after a merge or a rebuild.
- `bash .agents/scripts/review_extract.sh <out_dir> <base> <head> <regex>...`: a small two-commit repo of just the files a third-party review needs. Point `review_fast` / `review_final` at it (`diff_range xbase..xhead`); the full repository snapshot is too large to launch.
- `python -I .agents/scripts/vvis_pvs_compare.py --maps <dir> --out <dir> --run base=<bin dir> --run new=<bin dir>[,args]`: runs vvis builds (or edited portal files, `label=<bin dir>@<prt dir>`) on copies of the same BSP+PRT pairs and says whether the visibility lump is byte-identical, listing the cluster pairs gained and lost when it is not. Use it for any change under `src/utils/vvis`.
- `python -I .agents/scripts/make_corridor_prt.py <out_dir> <clusters>`: a synthetic straight-corridor portal file for a portal chain of a known depth (vvis's depth check).

## Testing tools

Always pass `-game <dir>` when running any tool here (vbsp_lmo, vvis_optix, vrad, bspwalk) in a test, harness or script. Without it the tools fall back to the `VPROJECT` environment variable, which on a dev box can point at an unrelated game. The result is missing models or a vphysics.dll load failure that looks like a DLL problem. The bspwalk fixture harness's default game is set in `game/bin/x64/unit_test/bspwalk/run_bspwalk_tests.py` (`--game`).
