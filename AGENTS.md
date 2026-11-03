# Agent notes

More rules live in `.agents/rules/` and workflows in `.agents/workflows/`.

## Testing tools

Always pass `-game <dir>` when running any tool here (vbsp_lmo, vvis_optix, vrad, bspwalk) in a test, harness or script. Without it the tools fall back to the `VPROJECT` environment variable, which on a dev box can point at an unrelated game. The result is missing models or a vphysics.dll load failure that looks like a DLL problem. The bspwalk fixture harness's default game is set in `game/bin/x64/unit_test/bspwalk/run_bspwalk_tests.py` (`--game`).
