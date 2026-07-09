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
