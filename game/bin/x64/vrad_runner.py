"""
VRAD Runner — invoke VRAD with -countlights and parse the surface light count.

This module provides a clean interface for the optimizer to call VRAD
as an external process and get the exact surface light count for a BSP file.

Usage:
    from vrad_runner import count_lights
    result = count_lights(vrad_exe, bsp_path, game_dir)
    print(f"Lights: {result.count}, exceeded: {result.exceeded}")
"""
from __future__ import annotations

import os
import re
import subprocess
import sys
import threading
from pathlib import Path
from typing import Optional
from collections import namedtuple


# ─── Exceptions ───────────────────────────────────────────────────────────────

class VRADError(Exception):
    """Raised when VRAD execution fails."""
    pass


class VRADNotFoundError(VRADError):
    """Raised when the VRAD executable is not found."""
    pass


class VRADCompileError(VRADError):
    """Raised when VRAD reports an error during compilation."""
    pass


class VRADParseError(VRADError):
    """Raised when the LIGHTCOUNT output cannot be parsed."""
    pass


# ─── Result Types ─────────────────────────────────────────────────────────────

# Result of a light count operation
LightCount = namedtuple('LightCount', [
    'count',      # Total surface light count
    'exceeded',   # True if count > 32767
])

# Default GMod light limit
GMOD_LIGHT_LIMIT = 32767

# VRAD's own hard limit (MAX_MAP_WORLDLIGHTS, bspfile.h): past it VRAD
# Error()s out ('Too many lights') without writing any lighting data. It is
# the same number as the GMod limit, but it is a property of the compiler,
# not a user budget -- callers that only need a compile to survive use this.
VRAD_HARD_LIGHT_LIMIT = 32767

# ─── Regex ────────────────────────────────────────────────────────────────────

_LIGHTCOUNT_RE = re.compile(r'^LIGHTCOUNT:\s*(\d+)', re.MULTILINE)
_EXCEEDED_RE = re.compile(r'^LIGHTCOUNT_EXCEEDED:\s*true', re.MULTILINE)

# Fallback: parse "N direct lights" from verbose VRAD output (works even
# without the -countlights patch, e.g. when VRAD Error()'s out early).
_DIRECT_LIGHTS_RE = re.compile(r'(\d+)\s+direct lights', re.MULTILINE)
# Fallback: parse "Too many lights (N / M)" error message
_TOO_MANY_RE = re.compile(r'Too many lights\s*\((\d+)\s*/\s*(\d+)\)', re.MULTILINE)

# VRAD executable name
_VRAD_EXE_NAME = 'vrad_rtx.exe'

# Development override: when vrad_rtx.exe is not co-located with the scripts,
# use this path.  Set to None for production (co-located) use.
_VRAD_DEV_PATH = Path(r'E:\GitHub\vrad-rtx\game\bin\x64')


# ─── Public API ───────────────────────────────────────────────────────────────

def find_vrad(search_dir: Optional[Path] = None) -> Optional[Path]:
    """Find vrad_rtx.exe, searching in the given directory or alongside this script.
    
    Search order:
        1. Explicit search_dir if provided
        2. Same directory as this script (game/bin/x64/)
        3. Development override path (_VRAD_DEV_PATH)
    """
    candidates = []
    
    if search_dir:
        candidates.append(Path(search_dir) / _VRAD_EXE_NAME)
    
    # Same directory as this script
    script_dir = Path(__file__).resolve().parent
    candidates.append(script_dir / _VRAD_EXE_NAME)
    
    # Development override
    if _VRAD_DEV_PATH:
        candidates.append(_VRAD_DEV_PATH / _VRAD_EXE_NAME)
    
    for path in candidates:
        if path.is_file():
            return path
    
    return None


def count_lights(vrad_exe: Path,
                 bsp_path: Path,
                 game_dir: Path,
                 bin_root: Path = None,
                 timeout: int = 300,
                 verbose: bool = False,
                 max_retries: int = 2,
                 lights_rad: Path | None = None) -> LightCount:
    """Run VRAD with -countlights and return the surface light count.
    
    This runs the full VRAD patch creation pipeline (loads lights.rad,
    creates patches, subdivides, creates direct lights) then prints the
    count and exits — without performing any actual lighting computation.
    
    Args:
        vrad_exe: Path to vrad_rtx.exe (must have -countlights support)
        bsp_path: Path to the compiled BSP file to analyze
        game_dir: Game directory for VRAD's -game flag
        bin_root: Bin root directory for VRAD's -binroot flag
        timeout: Maximum seconds to wait for VRAD (default: 300)
        verbose: If True, print VRAD's full output
        max_retries: Number of attempts (default: 2, covers Steam init failures)
        lights_rad: Optional path to a custom .rad lights file
        
    Returns:
        LightCount namedtuple with count and exceeded fields
        
    Raises:
        VRADNotFoundError: If vrad_exe doesn't exist
        VRADCompileError: If VRAD exits with an error after all retries
        VRADParseError: If LIGHTCOUNT line is not found in output
        TimeoutError: If VRAD exceeds the timeout
    """
    vrad_exe = Path(vrad_exe).resolve()
    bsp_path = Path(bsp_path).resolve()
    game_dir = Path(game_dir).resolve()
    
    if not vrad_exe.is_file():
        raise VRADNotFoundError(f"VRAD executable not found: {vrad_exe}")
    
    if not bsp_path.is_file():
        raise VRADError(f"BSP file not found: {bsp_path}")
    
    if not game_dir.is_dir():
        raise VRADError(f"Game directory not found: {game_dir}")
    
    # VRAD expects the BSP path without extension
    bsp_no_ext = str(bsp_path.with_suffix(''))
    
    # Build command line
    cmd = [
        str(vrad_exe),
        '-countlights',
        '-game', str(game_dir),
    ]
    if bin_root:
        cmd.extend(['-binroot', str(Path(bin_root).resolve())])
    if lights_rad:
        cmd.extend(['-lights', str(Path(lights_rad).resolve())])
    cmd.append(bsp_no_ext)
    
    env = os.environ.copy()
    if bin_root:
        bin_path_x64 = str((Path(bin_root) / 'bin' / 'x64').resolve())
        bin_path_32 = str((Path(bin_root) / 'bin').resolve())
        env['PATH'] = f"{bin_path_x64}{os.pathsep}{bin_path_32}{os.pathsep}{env.get('PATH', '')}"
    
    if verbose:
        print(f"  VRAD: {' '.join(cmd)}", flush=True)
    
    last_error = None
    for attempt in range(1, max_retries + 1):
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=timeout,
                cwd=str(vrad_exe.parent),
                env=env,
            )
        except subprocess.TimeoutExpired:
            raise TimeoutError(
                f"VRAD timed out after {timeout}s on {bsp_path.name}")
        except FileNotFoundError:
            raise VRADNotFoundError(f"Could not execute: {vrad_exe}")
        
        # Combine stdout and stderr for parsing
        full_output = result.stdout + '\n' + result.stderr
        
        if verbose:
            for line in result.stdout.splitlines():
                print(f"    [vrad] {line}", flush=True)
        
        # Always try to parse LIGHTCOUNT first (even on non-zero exit)
        match = _LIGHTCOUNT_RE.search(full_output)
        if match:
            count = int(match.group(1))
            exceeded = bool(_EXCEEDED_RE.search(full_output))
            return LightCount(count, exceeded)
        
        # Non-zero exit without LIGHTCOUNT — try fallback parsing before giving up.
        # Older VRAD builds Error() out before printing LIGHTCOUNT, but still
        # print "N direct lights" and/or "Too many lights (N / M)" to stderr.
        if result.returncode != 0:
            # Try "Too many lights (N / M)" first (most specific)
            too_many = _TOO_MANY_RE.search(full_output)
            if too_many:
                count = int(too_many.group(1))
                return LightCount(count, exceeded=True)
            
            # Try "N direct lights"
            direct = _DIRECT_LIGHTS_RE.search(full_output)
            if direct:
                count = int(direct.group(1))
                return LightCount(count, exceeded=(count > GMOD_LIGHT_LIMIT))
            
            output_tail = full_output.strip()[-500:]
            last_error = VRADCompileError(
                f"VRAD exited with code {result.returncode} "
                f"(attempt {attempt}/{max_retries}):\n{output_tail}")
            if attempt < max_retries:
                if verbose:
                    print(f"  VRAD attempt {attempt} failed, retrying...",
                          flush=True)
                continue
            raise last_error
        
        # Zero exit but no LIGHTCOUNT — shouldn't happen with -countlights
        raise VRADParseError(
            f"LIGHTCOUNT not found in VRAD output. "
            f"Is this VRAD built with -countlights support?\n"
            f"Output (last 500 chars): {full_output[-500:]}")
    
    # Should never reach here
    raise last_error or VRADError("VRAD failed after all retries")


# ─── Fast-Compile Constants ───────────────────────────────────────────────────

# Recommended VRAD flags for fast but accurate-enough lighting data.
# Used by auto-compile mode to generate lighting priority information.
FAST_VRAD_ARGS = [
    '-noextra',          # Skip supersampling (huge speedup)
    '-bounce', '2',      # Only 2 bounces (sufficient for priority classification)
    '-staticproppolys',  # Use polygon collision for static props
    '-textureshadows',   # Enable texture-based shadows (more accurate shadows)
]


def compile_rad(vrad_exe: Path,
                bsp_path: Path,
                game_dir: Path,
                bin_root: Path = None,
                timeout: int = 600,
                verbose: bool = False,
                max_retries: int = 2,
                lights_rad: Path | None = None,
                rtx: bool = False,
                extra_args: list | None = None,
                stream_output: bool = False) -> Path:
    """Run a full VRAD compile to produce lighting data in the BSP.
    
    Unlike count_lights() which uses -countlights (early exit),
    this performs a complete VRAD lighting pass so the resulting BSP
    has real lightmap data for face priority classification.
    
    Args:
        vrad_exe: Path to vrad_rtx.exe
        bsp_path: Path to the compiled BSP file
        game_dir: Game directory for VRAD's -game flag
        bin_root: Bin root directory for VRAD's -binroot flag
        timeout: Maximum seconds to wait for VRAD (default: 600)
        verbose: If True, print VRAD's full output
        max_retries: Number of attempts (default: 2)
        lights_rad: Optional path to a custom .rad lights file
        rtx: If True, add -rtx flag for GPU-accelerated lighting
        extra_args: Additional command-line arguments for VRAD
        
    Returns:
        Path to the lit BSP file
        
    Raises:
        VRADNotFoundError: If vrad_exe doesn't exist
        VRADCompileError: If VRAD exits with an error after all retries
        TimeoutError: If VRAD exceeds the timeout
    """
    vrad_exe = Path(vrad_exe).resolve()
    bsp_path = Path(bsp_path).resolve()
    game_dir = Path(game_dir).resolve()
    
    if not vrad_exe.is_file():
        raise VRADNotFoundError(f"VRAD executable not found: {vrad_exe}")
    
    if not bsp_path.is_file():
        raise VRADError(f"BSP file not found: {bsp_path}")
    
    if not game_dir.is_dir():
        raise VRADError(f"Game directory not found: {game_dir}")
    
    # VRAD expects the BSP path without extension
    bsp_no_ext = str(bsp_path.with_suffix(''))
    
    # Build command line with fast compile flags
    cmd = [
        str(vrad_exe),
        '-game', str(game_dir),
    ]
    if bin_root:
        cmd.extend(['-binroot', str(Path(bin_root).resolve())])
        
    cmd.extend(FAST_VRAD_ARGS)
    if rtx:
        cmd.append('-rtx')
    if lights_rad:
        cmd.extend(['-lights', str(Path(lights_rad).resolve())])
    if extra_args:
        cmd.extend(extra_args)
    cmd.append(bsp_no_ext)
    
    env = os.environ.copy()
    if bin_root:
        bin_path_x64 = str((Path(bin_root) / 'bin' / 'x64').resolve())
        bin_path_32 = str((Path(bin_root) / 'bin').resolve())
        env['PATH'] = f"{bin_path_x64}{os.pathsep}{bin_path_32}{os.pathsep}{env.get('PATH', '')}"
    
    if verbose:
        print(f"  VRAD compile: {' '.join(cmd)}", flush=True)
    
    def _echo(line: str) -> None:
        if stream_output:
            print(line, end='', flush=True)
        elif verbose:
            print(f"    [vrad] {line}", end='', flush=True)

    last_error = None
    for attempt in range(1, max_retries + 1):
        returncode, output_lines = _run_streaming(
            cmd, cwd=str(vrad_exe.parent), env=env, timeout=timeout,
            on_line=_echo, what=f"VRAD on {bsp_path.name}")
        
        # A nonzero exit is a failure even though a BSP exists: VBSP wrote it
        # long before VRAD ran, so its presence says nothing about whether
        # VRAD finished ('Too many lights' and every other Error() exit
        # nonzero without touching the file).
        if returncode == 0:
            return bsp_path
        
        full_output = ''.join(output_lines)
        output_tail = full_output.strip()[-500:]
        last_error = VRADCompileError(
            f"VRAD exited with code {returncode} "
            f"(attempt {attempt}/{max_retries}):\n{output_tail}")
        if attempt < max_retries:
            if verbose:
                print(f"  VRAD attempt {attempt} failed, retrying...",
                      flush=True)
            continue
        raise last_error

    raise last_error or VRADError("VRAD failed after all retries")


def _run_streaming(cmd: list, cwd: str, env: dict, timeout: float,
                   on_line, what: str) -> tuple:
    """Run cmd, forwarding merged stdout/stderr line by line, with a real timeout.

    The output is drained on a reader thread while the caller waits on the
    process with a wall-clock deadline: reading stdout to EOF on the calling
    thread (the old loop) blocks for as long as the child keeps its pipe open,
    so a hung VRAD never reached the wait(timeout=) after it.

    The child is started and killed through proc_tree (a job object on Windows,
    a process group on POSIX), the one tree-kill the GUIs and bspwalk runner
    use, so a launcher or wrapper script cannot leave a grandchild holding the
    pipe or the GPU after a timeout or Cancel. proc_tree.kill_tree() only
    SIGNALS on POSIX (SIGKILL follows 3 s after SIGTERM), so this waits for the
    process with its own bounded timeout rather than assuming it is dead.

    Returns (returncode, output_lines). Raises TimeoutError after killing the
    process tree, VRADNotFoundError if the executable cannot be started.
    """
    import proc_tree
    try:
        process = proc_tree.launch(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding='utf-8',
            errors='replace',
            cwd=cwd,
            env=env,
        )
    except (FileNotFoundError, PermissionError, OSError) as e:
        raise VRADNotFoundError(f"Could not execute: {cmd[0]} ({e})")

    output_lines: list = []

    def _pump() -> None:
        try:
            for line in process.stdout:
                output_lines.append(line)
                on_line(line)
        except (OSError, ValueError):
            pass  # pipe closed under us by the kill path

    reader = threading.Thread(target=_pump, daemon=True)
    reader.start()
    try:
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            proc_tree.kill_tree(process)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                pass
            reader.join(timeout=5)
            raise TimeoutError(f"{what} timed out after {timeout}s")
        reader.join(timeout=30)
        return process.returncode, output_lines
    finally:
        proc_tree.release(process)   # closes the job: a straggler dies with it


# ─── CLI Test Harness ─────────────────────────────────────────────────────────

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Count surface lights in a BSP file using VRAD.')
    parser.add_argument('bsp_path', help='Path to the BSP file')
    parser.add_argument('game_dir', help='Game directory (--game)')
    parser.add_argument('--vrad', default=None,
                        help=f'Path to {_VRAD_EXE_NAME} (auto-detected if omitted)')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='Print VRAD output')
    
    args = parser.parse_args()
    
    # Resolve VRAD executable
    if args.vrad:
        vrad_path = Path(args.vrad)
    else:
        vrad_path = find_vrad()
        if not vrad_path:
            print(f"ERROR: Could not find {_VRAD_EXE_NAME}. "
                  f"Use --vrad to specify the path.", file=sys.stderr)
            sys.exit(1)
    
    print(f"Using VRAD: {vrad_path}")
    
    try:
        result = count_lights(
            vrad_exe=vrad_path,
            bsp_path=Path(args.bsp_path),
            game_dir=Path(args.game_dir),
            verbose=args.verbose,
        )
        
        print(f"\n{'='*50}")
        print(f"  Surface Lights: {result.count:,}")
        print(f"  GMod Limit:     {GMOD_LIGHT_LIMIT:,}")
        print(f"  Status:         {'⚠ EXCEEDED' if result.exceeded else '✓ OK'}")
        if result.count > 0:
            pct = result.count / GMOD_LIGHT_LIMIT * 100
            print(f"  Usage:          {pct:.1f}%")
        print(f"{'='*50}")
        
        sys.exit(1 if result.exceeded else 0)
        
    except VRADError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(2)
    except TimeoutError as e:
        print(f"TIMEOUT: {e}", file=sys.stderr)
        sys.exit(3)
