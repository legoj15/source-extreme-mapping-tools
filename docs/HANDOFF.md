# Handoff (2026-11-02, written at the 9 PM stop)

## Branch state
- `vrad-overhaul` (5 commits ahead of origin, never merged back): bspwalk design doc, lmoptimizer BSP-mode fixes, fixture maps + scoring harness, plus a WIP commit of vrad_nextgen changes made at wrap-up.
- `master` has diverged: it carries bot PRs #1-#6 (Python bsp_reader unpack speedups, VRAD lightmap gather optimization, vbsp command-injection fix) that vrad-overhaul does not have. Decision pending: merge vrad-overhaul into master, or cherry-pick only the lmoptimizer/bspwalk commits (06742269, 4fa9a3fc, 92758679, 539938b6, e9929056) so the VMF optimizer / BSPwalker work can proceed on master without dragging in unfinished vrad_nextgen.

## WIP commit contents (vrad_nextgen, NOT verified)
- optix_kernels.cu / optix_kernels_ptx.h / vrad_nextgen.cpp: large uncommitted rewrite (about 1200 lines changed); never built or run this session. The PTX header is generated; regenerate with compile_ptx.bat before trusting it.
- compile_ptx.bat pinned to CUDA v13.2; vrad_nextgen.vpc gains filesystem_tools.cpp and filesystem_init.cpp.
- vbsp.cpp: warning when -maxlightmapdim > 35 (non-GMod games crash or glitch).
- .mcp.json (nvidia-cuda-docs) and .agents/rules/use-latest-nvidia-docs.md added.

## Not committed
- src/lib/common/x64/2015/release/lzma.lib shows modified (rebuilt binary, provenance unknown); stashed as "lzma.lib rebuild" rather than discarded. Drop it if a clean rebuild reproduces it.

## Next
- Resolve the master/vrad-overhaul question above, then continue bspwalk (issues #10-#16 on the legoj15 fork; see docs/bsp_walker_rebuild.md).
- Build and validate the vrad_nextgen WIP (per-face denoiser batching is the known 64% GPU-time hotspot).

## Delegations this session
None.
