"""
bspwalk_runner -- run bspwalk.exe on a compiled BSP and read its face tiers (#15).

    nv_faces, result = run_bspwalk(bsp_path, game_dir)

bspwalk (docs/bsp_walker_rebuild.md) classifies every BSP face as tier A (PVS-proven hidden),
tier B (ray-sampled hidden) or visible.  This returns the set of BSP face indices whose tier is
in `tiers` (default both: the #16 proposal gives both tiers max lightmap scale) and the parsed
JSON.  The BSP needs VVIS data for tier A.  Side ids (the JSON's per-face `sideids`, and the
face-to-side mapping callers build from lumps 22/23) come from the caller's `vbsp -emitsideids`
compile; this module compiles nothing.

Raises BspwalkError for every way the run can fail: an explicit exe path that does not exist (no
silent fallback to the default exe), a missing/invalid game dir, an empty or unknown tier list,
a start-up failure, a timeout, a non-zero exit, or no usable JSON.  A caller therefore never
mistakes a failed run for "every face visible".

bspwalk's console output is streamed to `log` line by line while it runs (default: print).

Game profile: bspwalk picks the player-movement profile from the last component of -game, else it
falls back to hl2 (bspwalk_profiles.txt).  A fallback is reported loudly (stderr WARNING), because
a game whose real jump or speed exceeds hl2's gets an under-approximated reach and so false
"hidden" faces.  Pass profile="cstrike" (or "-profile cstrike" in extra_args) to choose one, or
fail_on_profile_fallback=True to refuse the run instead of warning.

validate_setup() runs the same checks without starting anything: call it right after argument
parsing so a bad setup is reported before any minutes-long compile.
"""
import collections
import json
import locale
import math
import os
import queue
import re
import shlex
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path
from typing import Callable, Iterable, List, Optional, Sequence, Set, Tuple, Union

import proc_tree

_THIS_DIR = Path(__file__).resolve().parent
DEFAULT_EXE = _THIS_DIR / "bspwalk.exe"
PROFILES_NAME = "bspwalk_profiles.txt"
VALID_TIERS = ("A", "B")
# Offered when bspwalk_profiles.txt cannot be read; the file is the source of truth.
FALLBACK_PROFILES = ("hl2", "episodic", "hl2mp", "cstrike", "tf")
_PIPE_GRACE = 5.0           # s the pipe may stay open after bspwalk itself has exited
_NOT_GIVEN = object()
_EOF = object()

# bspwalk prints ANSI-code-page bytes (paths, entity names); decode them leniently for display.
_CONSOLE_ENCODING = locale.getpreferredencoding(False) or "utf-8"
# "bspwalk: profile hl2 (file, fallback; game directory "garrysmod"): reach ..." (bw_profile.cpp)
_PROFILE_RE = re.compile(r'bspwalk: profile (.+?) \((\w+), ([^;)]+); game directory "([^"]*)"\)')
# The pacifier (src/utils/common/pacifier.cpp) prints a digit then up to three dots per step
# ("0...1...2..."), without a newline, so its ticks prefix the next message: drop exactly that.
_TICKS_RE = re.compile(r"^(?:\d{1,2}\.{1,3}){3,}")


class BspwalkError(RuntimeError):
    pass


def find_bspwalk(explicit: Optional[str] = None) -> Optional[Path]:
    """The bspwalk.exe to use: an explicit path, else the one next to this script.

    An explicit path that is not a file raises BspwalkError: a typo must not quietly run another
    binary.  None is returned only when nothing was asked for and the default is absent (the
    caller's run_bspwalk/validate_setup then reports that).
    """
    if explicit:
        p = Path(explicit).expanduser().resolve()
        if not p.is_file():
            raise BspwalkError(f"bspwalk.exe not found at the path given: {explicit}")
        return p
    return DEFAULT_EXE if DEFAULT_EXE.is_file() else None


def split_args(text: Union[str, Sequence[str], None]) -> List[str]:
    """Split an extra-args field the way a shell would, so quoted paths survive.

    A sequence is returned as a list unchanged.  On Windows shlex runs non-POSIX (a backslash in
    C:\\my dir\\x stays a backslash) and the quotes it leaves on a token are removed.
    """
    if text is None:
        return []
    if not isinstance(text, str):
        return list(text)
    try:
        parts = shlex.split(text, posix=(os.name != "nt"))
    except ValueError as e:
        raise BspwalkError(f"cannot split the bspwalk arguments {text!r}: {e}") from None
    if os.name == "nt":
        parts = [p[1:-1] if len(p) >= 2 and p[0] == p[-1] and p[0] in "\"'" else p for p in parts]
    return parts


def join_args(args: Sequence[str]) -> str:
    """The inverse of split_args: one string that split_args turns back into exactly `args`."""
    if os.name != "nt":
        return shlex.join(args)
    out = []
    for a in args:
        if a and not any(c.isspace() for c in a):
            out.append(a)
        elif '"' not in a:
            out.append(f'"{a}"')
        elif "'" not in a:
            out.append(f"'{a}'")
        else:
            raise BspwalkError(f"cannot quote the bspwalk argument {a!r}")
    return " ".join(out)


def parse_tiers(tiers: Union[str, Iterable[str]]) -> List[str]:
    """Tiers as an upper-case list of A/B ("A,B" or ["a", "B"]); an empty or unknown tier raises."""
    if isinstance(tiers, str):
        tiers = tiers.split(",")
    want: List[str] = []
    for t in tiers:
        t = str(t).strip().upper()
        if t and t not in want:
            want.append(t)
    if not want:
        raise BspwalkError("no bspwalk tier given; use A, B or A,B (an empty list would "
                           "silently disable the visibility optimization)")
    bad = [t for t in want if t not in VALID_TIERS]
    if bad:
        raise BspwalkError(f"unknown tier(s) {sorted(bad)}; use A and/or B")
    return want


def known_profiles(exe: Optional[Path] = None) -> List[str]:
    """Profile names defined in bspwalk_profiles.txt (next to the exe); [] when unreadable."""
    path = (Path(exe).parent if exe else _THIS_DIR) / PROFILES_NAME
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return []
    return re.findall(r'^[ \t]+"([^"]+)"[ \t]*$', text, re.M)    # blocks alone on a line, indented


def profile_choices(exe: Optional[Path] = None) -> Tuple[str, ...]:
    """What a GUI dropdown offers: "auto" (bspwalk picks from the game dir), then every profile."""
    return ("auto", *(known_profiles(exe) or FALLBACK_PROFILES))


def _has_profile_arg(args: Sequence[str]) -> bool:
    return any(a.lower() == "-profile" for a in args)


def validate_setup(game_dir, exe: Optional[Path] = None, tiers: Union[str, Iterable[str]] = ("A", "B"),
                   bsp_path=_NOT_GIVEN) -> Tuple[Path, Path, List[str]]:
    """Check everything run_bspwalk needs before anything starts.

    Returns (exe, game_dir, tiers) resolved; raises BspwalkError naming what is wrong.  Call it
    right after argument parsing, ahead of any compile.  Leave `bsp_path` out while the BSP does
    not exist yet; once given, None or a missing file raises.
    """
    exe_path = Path(exe) if exe else find_bspwalk()
    if not exe_path or not exe_path.is_file():
        raise BspwalkError(f"bspwalk.exe not found ({exe_path or DEFAULT_EXE})")
    if game_dir is None or not str(game_dir).strip():
        raise BspwalkError("bspwalk needs a game directory (-game, the folder holding gameinfo.txt); "
                           "none was given")
    game = Path(game_dir).expanduser().resolve()
    if not game.is_dir():
        raise BspwalkError(f"game directory not found: {game}")
    if not (game / "gameinfo.txt").is_file():
        raise BspwalkError(f"no gameinfo.txt in {game} (-game must be the mod folder, e.g. .../hl2)")
    if bsp_path is not _NOT_GIVEN:
        if bsp_path is None or not str(bsp_path).strip():
            raise BspwalkError("no BSP given to run bspwalk on")
        if not Path(bsp_path).is_file():
            raise BspwalkError(f"BSP not found: {bsp_path}")
    return exe_path.resolve(), game, parse_tiers(tiers)


def describe_exit(rc: int) -> str:
    """'3' for a plain code, '0xC0000005' for a Windows crash (NTSTATUS in the high bits)."""
    return f"0x{rc & 0xFFFFFFFF:08X}" if rc < 0 or rc >= 0x80000000 else str(rc)


def _default_log(line: str) -> None:
    try:
        print(f"  {line}", flush=True)
    except UnicodeEncodeError:                     # a cp1252 pipe cannot carry U+FFFD
        enc = sys.stdout.encoding or "ascii"
        print(f"  {line}".encode(enc, "replace").decode(enc), flush=True)


def run_bspwalk(bsp_path, game_dir, exe: Optional[Path] = None, tiers: Union[str, Iterable[str]] = ("A", "B"),
                extra_args: Union[str, Sequence[str]] = (), threads: int = 0, timeout: Optional[float] = None,
                verbose: bool = False, profile: Optional[str] = None,
                fail_on_profile_fallback: bool = False, bin_root: Optional[str] = None,
                log: Optional[Callable[[str], None]] = None) -> Tuple[Set[int], dict]:
    """Run bspwalk on `bsp_path` and return ({hidden BSP face indices}, parsed JSON).

    exe        explicit bspwalk.exe (must exist); None = the one next to this script.
    tiers      "A,B" / ["A"] / ...; the tiers whose faces count as hidden.  Must be non-empty.
    extra_args a list, or one string split like a shell (quotes keep a path together).
    timeout    seconds; if bspwalk is still running then, its whole process tree is killed and
               BspwalkError raised.  None or 0 = no limit; negative raises.
    verbose    also echo the command line (bspwalk's own output is always streamed).
    profile    -profile name; else the game-directory name picks one, else hl2 (warned).
    bin_root   the engine root holding bin/x64 (the --binroot VBSP/VVIS/VRAD get): forwarded as
               -binroot so vphysics.dll loads when the game dir is not under the engine folder.
    log        receives each console line as it arrives (default: print, indented).
    """
    exe_path, game_dir, want = validate_setup(game_dir, exe=exe, tiers=tiers, bsp_path=bsp_path)
    bsp_path = Path(bsp_path).resolve()
    if timeout is not None and (not math.isfinite(timeout) or timeout < 0):
        raise BspwalkError(f"bspwalk timeout must be a number of seconds >= 0 (0 = no limit), got {timeout!r}")
    args = split_args(extra_args)
    explicit_profile = bool(profile) or _has_profile_arg(args)
    emit = log or _default_log

    fd, out = tempfile.mkstemp(prefix="bspwalk_", suffix=".json")
    os.close(fd)
    proc: Optional[subprocess.Popen] = None
    try:
        # -game always (AGENTS.md); -reachslim: the node vertices are not needed here
        cmd = [str(exe_path), str(bsp_path), "-game", str(game_dir), "-reachslim", "-out", out]
        if profile and not _has_profile_arg(args):
            cmd += ["-profile", profile]
        cmd += args
        if bin_root and not any(a.lower() == "-binroot" for a in args):
            cmd += ["-binroot", str(Path(bin_root).resolve())]
        if threads > 0:
            cmd += ["-threads", str(threads)]
        if verbose:
            emit("bspwalk: " + " ".join(cmd))
        try:
            proc = proc_tree.launch(cmd, cwd=str(exe_path.parent), stdin=subprocess.DEVNULL,
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        except OSError as e:
            raise BspwalkError(f"cannot start {exe_path}: {e}") from e

        # A reader thread feeds the pipe to this one, so a process that still holds the pipe
        # after bspwalk exited can never block the loop below.
        lines: "queue.Queue" = queue.Queue()

        def _pump():
            try:
                for raw in iter(proc.stdout.readline, b""):
                    lines.put(raw)
            except (OSError, ValueError):
                pass
            finally:
                lines.put(_EOF)

        threading.Thread(target=_pump, daemon=True).start()

        deadline = time.monotonic() + timeout if timeout else None
        tail = collections.deque(maxlen=15)
        timed_out, refused, eof, root_gone_at = False, "", False, None
        try:
            while True:
                raw = None
                if eof:
                    if proc.poll() is not None:
                        break
                    time.sleep(0.05)
                else:
                    try:
                        raw = lines.get(timeout=0.2)
                    except queue.Empty:
                        pass
                    else:
                        if raw is _EOF:
                            eof, raw = True, None
                if raw is not None:
                    line = _TICKS_RE.sub("", raw.decode(_CONSOLE_ENCODING, errors="replace").rstrip("\r\n"))
                    if line.strip():
                        tail.append(line)
                        emit(line)
                        m = _PROFILE_RE.search(line)
                        if m and m.group(3).strip() == "fallback" and not explicit_profile:
                            names = ", ".join(known_profiles(exe_path)) or "see bspwalk_profiles.txt"
                            msg = (f'bspwalk used the {m.group(1)} movement profile as a FALLBACK: game '
                                   f'directory "{m.group(4)}" matches no profile. A game with a higher jump or '
                                   f'speed than {m.group(1)} gets under-approximated reach and false "hidden" '
                                   f'faces. Choose one with -profile ({names}).')
                            if fail_on_profile_fallback:
                                refused = msg
                                proc_tree.kill_tree(proc)
                                break
                            print(f"  WARNING: {msg}", file=sys.stderr, flush=True)
                if proc.poll() is not None:
                    if root_gone_at is None:
                        root_gone_at = time.monotonic()
                        proc_tree.release(proc)        # kills a straggler that still holds the pipe
                    elif not eof and time.monotonic() - root_gone_at > _PIPE_GRACE:
                        proc_tree.kill_tree(proc)      # POSIX has no job: end the pipe holder
                        break                         # pipe never closed: stop waiting for it
                elif deadline is not None and not timed_out and time.monotonic() >= deadline:
                    timed_out = True                   # still running at the deadline
                    proc_tree.kill_tree(proc)
        finally:
            if proc.poll() is None:                    # Ctrl+C, an emit error: never leave it running
                proc_tree.kill_tree(proc)
            proc.wait()
            proc_tree.release(proc)
        rc = proc.returncode

        if refused:
            raise BspwalkError(refused)
        text_tail = "\n".join(tail)
        if timed_out:
            raise BspwalkError(f"bspwalk timed out after {timeout:g} s and was killed:\n{text_tail}")
        if rc != 0:
            raise BspwalkError(f"bspwalk exited {describe_exit(rc)}:\n{text_tail}")
        try:
            # bspwalk writes strings raw (ANSI paths, entity names): never fail on a non-UTF-8 byte.
            result = json.loads(Path(out).read_bytes().decode("utf-8", errors="replace"))
        except (OSError, ValueError) as e:
            raise BspwalkError(f"bspwalk wrote no readable JSON: {e}") from e
    finally:
        if proc is not None and proc.poll() is None:
            proc_tree.kill_tree(proc)
            proc.wait()
        try:
            os.remove(out)
        except OSError:
            pass
    if not isinstance(result, dict):
        raise BspwalkError("bspwalk JSON root is not an object")
    faces = result.get("faces")
    if not isinstance(faces, list) or not faces:
        raise BspwalkError("bspwalk JSON has no face tiers (was it run with -novis?)")
    try:
        hidden = {int(f["face"]) for f in faces if f.get("tier") in want}
    except (KeyError, TypeError, ValueError, AttributeError) as e:
        raise BspwalkError(f"bspwalk JSON has a malformed face entry: {e!r}") from e
    return hidden, result


def summary(result: dict) -> str:
    vis = result.get("vis", {})
    reach = result.get("reach", {})
    prof = result.get("profile", {})
    return (f"bspwalk: {vis.get('tier_a', '?')} tier A, {vis.get('tier_b_hidden', '?')} tier B, "
            f"{vis.get('visible', '?')} visible of {vis.get('faces', '?')} faces "
            f"(PVS {'yes' if vis.get('pvs') else 'no'}, rays on the {vis.get('path', '?')}); "
            f"{reach.get('reached_count', '?')} of {reach.get('node_count', '?')} walkable polygons reached; "
            f"profile {prof.get('name', '?')}; "
            f"{result.get('timings_s', {}).get('total', 0):.1f} s")
