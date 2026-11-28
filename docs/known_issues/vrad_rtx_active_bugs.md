# VRAD-RTX Active Bugs

This document tracks identified bugs inside the engine that currently require script-side mitigations or future code-level resolutions.

---

## 1. The "Lock Trap" Deadlock (Bounce #13 Hang) — **FIXED (engine-side, 2026-07-09)**

### Symptoms
When running `vrad_rtx.exe -cuda` in specific automated or piped environments (like Windows PowerShell 5.1 tests), the process will completely hang during the Multithreaded Radiosity phase (e.g., at `Bounce #13`).
- **Log Signature**: Print progress stops exactly halfway through a `0...1...2...x` block inside `RunThreadsOn(uiPatchCount, true, GatherLight)`.

### Root Cause
An architectural deadlock involving the Windows IO Pipe Buffer and the Source engine thread dispatcher.
1. The global thread dispatcher `GetThreadWork()` holds a critical section: `ThreadLock()`.
2. Inside that lock, it calls `UpdatePacifier()` to print progress dots.
3. `UpdatePacifier` performs blocking synchronous I/O (`Msg()`).
4. By Bounce #12 or #13, the process has dumped enough logging (especially during GPU Visibility phases) to **saturate the 4KB/64KB OS Pipe Buffer**.
5. Once the OS buffer is full, the next thread to call `Msg()` blocks indefinitely, waiting for the receiving script to drain the pipe.
6. Because the thread is blocked while holding the `crit` section, all other threads stall attempting to grab `GetThreadWork()`. The main thread stalls on `WaitForMultipleObjects(INFINITE)`, creating a permanent deadlock.

### Resolution (Engine-Side — APPLIED)
`UpdatePacifier` has been decoupled from the thread dispatcher lock in `src/utils/common/threads.cpp` (`GetThreadWork()`):
1. The work counters (`dispatch`/`workcount`) are snapshotted inside `ThreadLock()`, then the lock is released **before** any I/O occurs. A thread blocked writing to a full pipe can no longer stall work dispatch.
2. The pacifier draw itself is guarded by a separate dedicated critical section acquired with `TryEnterCriticalSection`: if another worker is currently drawing (or blocked on a full pipe), the update is simply skipped instead of queuing behind it. Because the pacifier only moves forward, the next successful update (or `EndPacifier`) redraws any skipped dots, so console output is unchanged in practice.

This fix lives in `src/utils/common`, so it applies to vbsp, vvis, and vrad alike.

### Residual Limitation
If the consuming process **never** drains stdout at all, any `Msg()` call anywhere in the tools can still block that one thread forever — no engine-side change can print into a pipe nobody reads. The fix guarantees the dispatcher and the other workers keep running. Aggressive async stream draining in harness scripts (`Register-ObjectEvent` / PowerShell 7 background readers) remains good practice but is no longer required to avoid the deadlock.

---

## 2. CPU and `-cuda` lighting: what the kernel mirrors, and the two known leftovers (#175, closed 2026-11-28)

The CPU path is the reference: `-cuda` must give the same lightmaps. The comparison suite (`tests/compare/run_compare_tests.ps1 -Tool vrad`, 15 maps) holds the two within 0.005 to 0.01 % of the decoded light wherever no tracked CPU-side issue stands (#172, #173), and both paths are bit-deterministic run to run (#174).

Rules of the CPU gather that the kernel (`src/raytrace/optix_kernels.cu`, `__raygen__direct_lighting`) and its host code copy on purpose. Change one side and the other must follow:

- **Packs of four.** The CPU gathers four samples at a time (`GatherSampleLightAt4Points`; for a supersampled luxel the four sky-ambient sub-positions, `SupersampleLightAtPoint`). `GatherSampleAmbientSkySSE` skips a sky direction only when none of the four faces it, so a direction a pack-mate faces still lights the other lanes' bump directions, uncounted. The kernel reads the pack-mates' normals (primary samples by index, sub-positions through the `GPU_SAMPLE_GROUPED` flags) wherever lanes can have different normals: displacements, and brush faces VRAD smooths (any two faces under 45 degrees apart that share an edge, `smoothing_threshold`).
- **The flat gate.** A sample whose flat result for a light is not above zero gets nothing on its bump directions either (`GatherSampleLightSSE`).
- **The sky-ambient threshold** is `EQUAL_EPSILON` (0.001) on both sides (`SKY_DOT_EPSILON`).
- **Bump directions per sample** on displacements and smoothed faces (`GetBumpNormalsPerSample`, the CPU's `GetBumpNormals`), normalised as `VectorNormalize` does (1e-10 added to the squared length).
- **3D skybox**: a sun or sky ray that reaches the sky is traced again in the skybox, unless the ray starts in a sky camera's area or `-noskyboxrecurse` is given (`TestLine_DoesHitSky`; `GPU_SAMPLE_NO_SKY_RECURSE`).
- **Clusters and lights**: a sample takes the lights of the cluster of its point on the face (not of the point one unit off it); a sample in no cluster takes every light (`PVSCheck`); a cluster with an empty light list gives none. The CPU's per-cluster lists are a shortcut for the stock per-sample PVS test and must stay equivalent to it.
- **Light styles**: four slots per sample, given to the lights that reach the sample in the cluster list's order.
- **Supersampling**: the passes' samples are decided once per pass, whatever the batch size (`.agents/scripts/vrad_ss_batch_check.py`); 4x4 sub-positions for everything but the sky ambient, 2x2 for it, each averaged over its own count.

Known leftovers, neither measured as a difference on any map (file an issue with the map if one shows it):

1. **A face lit by more than four light styles.** The CPU keeps four styles per face and warns; the kernel keeps four per sample or sub-position, so on such a face a supersampling sub-position can give a slot to a style the face dropped and lose one it kept.
2. **The any-hit program ignores `allowBackface` under front-face culling**, and there is no single-any-hit flag. Measured harmless on the comparison maps.

To find a new difference: compare per face and per block (flat, bump 1 to 3), not the whole-map figure; split by light (sun, sky ambient, point) and by `-noextra` and `-bounce 0`; a stock-against-ours difference that appears only with bounces is a sample-to-patch tie or a form factor, not the direct light (#178, #181, #183).
