# Agent notes

More rules live in `.agents/rules/` and workflows in `.agents/workflows/`.

## Scripts

- `bash .agents/scripts/run_suites.sh [--only a,b]`: the quick regression suites (bspwalk selftest and fixtures, lmoptimizer, proplm, vvis sight-line and mergeflow) against the binaries in `game/bin/x64`, one pass/fail line each. Run it after a merge or a rebuild.
- `python -I .agents/scripts/proplm_rebake_ab.py [--vbsp A.exe --vbsp B.exe] [--keep DIR] model...`: compiles the staticpropbounce fixture with one `proplm "rebake"` prop per model and prints the rebake log lines and the pak files it added (sizes, CRC); with two VBSP builds it diffs the pak lumps. A second build must sit in `game/bin/x64` (it needs the DLLs beside it).
- `bash .agents/scripts/review_extract.sh <out_dir> <base> <head> <regex>...`: a small two-commit repo of just the files a third-party review needs. Point `review_fast` / `review_final` at it (`diff_range xbase..xhead`); the full repository snapshot is too large to launch.

## Testing tools

Always pass `-game <dir>` when running any tool here (vbsp_lmo, vvis_optix, vrad, bspwalk) in a test, harness or script. Without it the tools fall back to the `VPROJECT` environment variable, which on a dev box can point at an unrelated game. The result is missing models or a vphysics.dll load failure that looks like a DLL problem. The bspwalk fixture harness's default game is set in `game/bin/x64/unit_test/bspwalk/run_bspwalk_tests.py` (`--game`).
