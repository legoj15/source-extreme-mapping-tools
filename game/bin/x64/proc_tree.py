"""
proc_tree -- launch a child so that Cancel (or closing the window) takes its whole process tree down.

The GUIs run lmoptimizer.py / vis_simulator.py, which run bspwalk, vbsp, vvis and vrad.  On Windows
Popen.terminate() is TerminateProcess on the one Python process: its children keep running (and keep
the CPU/GPU busy) while the GUI reports "Cancelled".

    proc = launch(cmd, stdout=subprocess.PIPE, ...)   # a Popen, started inside a job object
    kill_tree(proc)                                   # Windows: taskkill + job terminate; POSIX: signals. Caller waits
    kill_tree_async(proc)                             # same, on a worker thread (use from a GUI thread)
    kill_pending(proc)                                # is that worker still killing?
    release(proc)                                     # the run ended: close the job (kills stragglers)

Windows: the child is assigned to a job object with JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE, so every
descendant dies if the GUI itself is killed (the OS closes the job handle) and kill_tree() ends
whatever the job holds.  kill_tree() also runs `taskkill /T /F` on the still-live root first, which
reaches children that escaped the job and covers a launch whose job could not be set up.
POSIX: the child gets its own session and the whole process group is signalled.
"""
import os
import signal
import subprocess
import sys
import threading
from typing import Optional

if sys.platform == "win32":
    import ctypes
    from ctypes import wintypes

    _k32 = ctypes.WinDLL("kernel32", use_last_error=True)
    _k32.CreateJobObjectW.restype = wintypes.HANDLE
    _k32.CreateJobObjectW.argtypes = [wintypes.LPVOID, wintypes.LPCWSTR]
    _k32.SetInformationJobObject.restype = wintypes.BOOL
    _k32.SetInformationJobObject.argtypes = [wintypes.HANDLE, ctypes.c_int, wintypes.LPVOID, wintypes.DWORD]
    _k32.AssignProcessToJobObject.restype = wintypes.BOOL
    _k32.AssignProcessToJobObject.argtypes = [wintypes.HANDLE, wintypes.HANDLE]
    _k32.TerminateJobObject.restype = wintypes.BOOL
    _k32.TerminateJobObject.argtypes = [wintypes.HANDLE, wintypes.UINT]
    _k32.OpenProcess.restype = wintypes.HANDLE
    _k32.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
    _k32.CloseHandle.restype = wintypes.BOOL
    _k32.CloseHandle.argtypes = [wintypes.HANDLE]

    _JOB_OBJECT_EXTENDED_LIMIT_INFORMATION_CLASS = 9
    _JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE = 0x2000
    _PROCESS_TERMINATE = 0x0001
    _PROCESS_SET_QUOTA = 0x0100

    class _IO_COUNTERS(ctypes.Structure):
        _fields_ = [(n, ctypes.c_ulonglong) for n in (
            "ReadOperationCount", "WriteOperationCount", "OtherOperationCount",
            "ReadTransferCount", "WriteTransferCount", "OtherTransferCount")]

    class _BASIC_LIMITS(ctypes.Structure):
        _fields_ = [("PerProcessUserTimeLimit", ctypes.c_int64), ("PerJobUserTimeLimit", ctypes.c_int64),
                    ("LimitFlags", wintypes.DWORD), ("MinimumWorkingSetSize", ctypes.c_size_t),
                    ("MaximumWorkingSetSize", ctypes.c_size_t), ("ActiveProcessLimit", wintypes.DWORD),
                    ("Affinity", ctypes.c_size_t), ("PriorityClass", wintypes.DWORD),
                    ("SchedulingClass", wintypes.DWORD)]

    class _EXTENDED_LIMITS(ctypes.Structure):
        _fields_ = [("BasicLimitInformation", _BASIC_LIMITS), ("IoInfo", _IO_COUNTERS),
                    ("ProcessMemoryLimit", ctypes.c_size_t), ("JobMemoryLimit", ctypes.c_size_t),
                    ("PeakProcessMemoryUsed", ctypes.c_size_t), ("PeakJobMemoryUsed", ctypes.c_size_t)]

    def _make_job(pid: int) -> Optional[int]:
        """A kill-on-close job holding process `pid`; None if any step fails (taskkill fallback)."""
        job = proc_handle = None
        assigned = False
        try:
            job = _k32.CreateJobObjectW(None, None)
            if not job:
                return None
            info = _EXTENDED_LIMITS()
            info.BasicLimitInformation.LimitFlags = _JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE
            if not _k32.SetInformationJobObject(job, _JOB_OBJECT_EXTENDED_LIMIT_INFORMATION_CLASS,
                                                ctypes.byref(info), ctypes.sizeof(info)):
                return None
            proc_handle = _k32.OpenProcess(_PROCESS_SET_QUOTA | _PROCESS_TERMINATE, False, pid)
            if not proc_handle:
                return None
            assigned = bool(_k32.AssignProcessToJobObject(job, proc_handle))
            return job if assigned else None
        except Exception:                       # any ctypes surprise: no job, not a lost child
            return None
        finally:
            if proc_handle:
                _k32.CloseHandle(proc_handle)
            if job and not assigned:
                _k32.CloseHandle(job)

    def _taskkill_tree(pid: int) -> None:
        """Walk the tree from the live root `pid` (it cannot be walked once the root is gone)."""
        try:
            subprocess.run(["taskkill", "/PID", str(pid), "/T", "/F"], capture_output=True,
                           timeout=10, creationflags=subprocess.CREATE_NO_WINDOW)
        except (OSError, subprocess.SubprocessError):
            pass


def python_child_env() -> dict:
    """Environment for a Python child whose pipe the GUI reads: unbuffered, UTF-8 (the GUI decodes UTF-8)."""
    return dict(os.environ, PYTHONUNBUFFERED="1", PYTHONIOENCODING="utf-8")


def launch(cmd, **popen_kwargs) -> subprocess.Popen:
    """subprocess.Popen(cmd, **popen_kwargs) wired for kill_tree().  No console window on Windows."""
    if sys.platform == "win32":
        popen_kwargs["creationflags"] = popen_kwargs.get("creationflags", 0) | subprocess.CREATE_NO_WINDOW
    else:
        popen_kwargs.setdefault("start_new_session", True)
    proc = subprocess.Popen(cmd, **popen_kwargs)
    proc._proc_tree_job = None                  # set first: kill_tree works whatever happens next
    proc._proc_tree_killer = None
    if sys.platform == "win32":
        try:
            proc._proc_tree_job = _make_job(proc.pid)
        except BaseException:                   # e.g. Ctrl+C right here: do not leak the child
            kill_tree(proc)
            raise
    return proc


def kill_tree(proc: Optional[subprocess.Popen]) -> None:
    """Kill `proc` and every process it started.  Safe to call twice or on a finished process."""
    if proc is None:
        return
    if sys.platform == "win32":
        # 1. While the root lives, walk the tree by parent links: that also reaches a child that
        #    left the job (the Microsoft Store python.exe alias launcher does).
        if proc.poll() is None:
            _taskkill_tree(proc.pid)
        # 2. Then the job: anything re-parented or spawned since is still a member.
        job = getattr(proc, "_proc_tree_job", None)
        if job and not _k32.TerminateJobObject(job, 1) and proc.poll() is None:
            _taskkill_tree(proc.pid)                    # the job refused: walk the tree again
        if proc.poll() is None:
            try:
                proc.kill()
            except OSError:
                pass
    else:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except OSError:
            return
        t = threading.Timer(3.0, lambda: _killpg_quiet(proc.pid))
        t.daemon = True
        t.start()


def kill_tree_async(proc: Optional[subprocess.Popen]) -> Optional[threading.Thread]:
    """kill_tree() on a worker thread, so a GUI thread never waits on taskkill (up to 10 s).

    The thread is not a daemon on purpose: if the program exits while it runs, Python waits for
    the kill to finish, so nothing survives.  Poll kill_pending() to learn when it is done.
    """
    if proc is None:
        return None
    killer = getattr(proc, "_proc_tree_killer", None)
    if killer is None or not killer.is_alive():
        killer = threading.Thread(target=kill_tree, args=(proc,), name="proc_tree-kill")
        proc._proc_tree_killer = killer
        killer.start()
    return killer


def kill_pending(proc: Optional[subprocess.Popen]) -> bool:
    """True while a kill_tree_async() worker for `proc` is still running."""
    killer = getattr(proc, "_proc_tree_killer", None)
    return killer is not None and killer.is_alive()


def release(proc: Optional[subprocess.Popen]) -> None:
    """The run is over: close the job handle (any straggler left in the tree dies with it)."""
    if proc is None or sys.platform != "win32":
        return
    job = getattr(proc, "_proc_tree_job", None)
    if job:
        proc._proc_tree_job = None
        _k32.CloseHandle(job)


def _killpg_quiet(pgid: int) -> None:
    try:
        os.killpg(pgid, signal.SIGKILL)
    except OSError:
        pass
