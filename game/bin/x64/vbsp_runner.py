"""
VBSP Runner — invoke VBSP with -countverts and parse the vertex count.

This module provides a clean interface for the optimizer to call VBSP
as an external process and get the exact vertex count for a VMF file.

Usage:
    from vbsp_runner import count_vertices
    verts = count_vertices(vbsp_exe, vmf_path, game_dir)
"""
from __future__ import annotations

import os
import re
import shutil
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Optional




class VBSPError(Exception):
    """Raised when VBSP execution fails."""
    pass


class VBSPNotFoundError(VBSPError):
    """Raised when the VBSP executable is not found."""
    pass


class VBSPCompileError(VBSPError):
    """Raised when VBSP reports an error during compilation."""
    pass


class VBSPRejectedError(VBSPCompileError):
    """VBSP refused the map at this lightmap scale because a format capacity
    limit was exceeded ('Too many edges', 'Edge with too many vertices due to
    t-junctions', ...). The only kind of VBSP failure a -countverts caller may
    read as 'this scale is too fine, try a coarser one'; every other
    VBSPCompileError (a crash, a parse error, a missing file) is a real
    failure.

    'Bad surface extents' is NOT among these: it is raised by EndBSPFile,
    which -countverts mode never reaches (vbsp.cpp returns first), so the count
    path can neither produce nor detect it. Displacement extents are guarded by
    vertex_estimator.min_scale_for_valid_extents instead."""
    pass


class VBSPParseError(VBSPError):
    """Raised when the VERTCOUNT output cannot be parsed."""
    pass


from collections import namedtuple

# Result of a vertex count operation
VertexCount = namedtuple('VertexCount', [
    'count', 'exceeded',
    'leafface_count', 'leafface_exceeded',
    'face_count', 'face_exceeded',
])

# VBSP Error() messages that mean "this lightmap scale produces more of
# something than the BSP format holds" and that CAN occur in -countverts mode
# (src/utils/vbsp: faces.cpp, writebsp.cpp, common/bsplib.cpp). Anything else
# that makes VBSP exit nonzero is not a statement about the scale. (Not here:
# 'Bad surface extents', which only fires in a full compile's EndBSPFile.)
_SCALE_REJECTION_RE = re.compile(
    r'Too many (?:unique verts|edges|faces|leaffaces|detail brush faces|BSP leaves)'
    r'|Edge with too many vertices'
    r'|MAX_MAP_(?:EDGES|FACES|LEAFFACES|VERTS|NODES|LEAFS)\b'
    r'|has too many texinfos',
    re.IGNORECASE)


def _is_crash_exit(returncode: int) -> bool:
    """A signal death (negative) or a Windows NTSTATUS crash code (0xC0000005...)."""
    return returncode < 0 or returncode >= 0x80000000


# Regex to match the machine-parseable count outputs
_VERTCOUNT_RE = re.compile(r'^VERTCOUNT:\s*(\d+)', re.MULTILINE)
_EXCEEDED_RE = re.compile(r'^VERTCOUNT_EXCEEDED:\s*true', re.MULTILINE)
_LEAFFACECOUNT_RE = re.compile(r'^LEAFFACECOUNT:\s*(\d+)', re.MULTILINE)
_LEAFFACE_EXCEEDED_RE = re.compile(r'^LEAFFACECOUNT_EXCEEDED:\s*true', re.MULTILINE)
_FACECOUNT_RE = re.compile(r'^FACECOUNT:\s*(\d+)', re.MULTILINE)
_FACE_EXCEEDED_RE = re.compile(r'^FACECOUNT_EXCEEDED:\s*true', re.MULTILINE)


def find_vbsp(search_dir: Optional[Path] = None) -> Optional[Path]:
    """Find vbsp_lmo.exe, searching in the given directory or alongside this script.
    
    Search order:
        1. Explicit search_dir if provided
        2. Same directory as this script (game/bin/x64/)
        3. Parent directories (game/bin/)
    """
    candidates = []
    
    if search_dir:
        candidates.append(Path(search_dir) / 'vbsp_lmo.exe')
    
    # Same directory as this script
    script_dir = Path(__file__).resolve().parent
    candidates.append(script_dir / 'vbsp_lmo.exe')
    
    # Parent
    candidates.append(script_dir.parent / 'vbsp_lmo.exe')
    
    for path in candidates:
        if path.is_file():
            return path
    
    return None


def count_vertices(vbsp_exe: Path,
                   vmf_path: Path,
                   game_dir: Path,
                   bin_root: Path = None,
                   timeout: int = 120,
                   verbose: bool = False,
                   max_retries: int = 2) -> VertexCount:
    """Run VBSP with -countverts to get the total vertex count of the map.
    
    Args:
        vbsp_exe: Path to vbsp_lmo.exe
        vmf_path: Path to the VMF file to check
        game_dir: Game directory for VBSP's -game flag
        bin_root: Bin root directory for VBSP's -binroot flag
        timeout: Maximum seconds to wait for VBSP (default: 120)
        verbose: If True, print VBSP's full output
        max_retries: Number of attempts (default: 2, covers Steam init failures)
        
    Returns:
        Integer vertex count from VBSP's VERTCOUNT output
        
    Raises:
        VBSPNotFoundError: If vbsp_exe doesn't exist
        VBSPCompileError: If VBSP exits with an error after all retries
        VBSPParseError: If VERTCOUNT line is not found in output
        TimeoutError: If VBSP exceeds the timeout
    """
    vbsp_exe = Path(vbsp_exe).resolve()
    vmf_path = Path(vmf_path).resolve()
    game_dir = Path(game_dir).resolve()
    
    if not vbsp_exe.is_file():
        raise VBSPNotFoundError(f"VBSP executable not found: {vbsp_exe}")
    
    if not vmf_path.is_file():
        raise VBSPError(f"VMF file not found: {vmf_path}")
    
    if not game_dir.is_dir():
        raise VBSPError(f"Game directory not found: {game_dir}")
    
    # Build command line
    cmd = [
        str(vbsp_exe),
        '-countverts',
        '-game', str(game_dir),
    ]
    if bin_root:
        cmd.extend(['-binroot', str(Path(bin_root).resolve())])
    cmd.append(str(vmf_path))
    
    env = os.environ.copy()
    if bin_root:
        # Prepend the binroot to PATH so that VBSP can load MaterialSystem.dll and other engine DLLs
        bin_path_x64 = str((Path(bin_root) / 'bin' / 'x64').resolve())
        bin_path_32 = str((Path(bin_root) / 'bin').resolve())
        env['PATH'] = f"{bin_path_x64}{os.pathsep}{bin_path_32}{os.pathsep}{env.get('PATH', '')}"
    
    if verbose:
        print(f"  VBSP: {' '.join(cmd)}", flush=True)
    
    last_error = None
    for attempt in range(1, max_retries + 1):
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=timeout,
                cwd=str(vbsp_exe.parent),
                env=env,
            )
        except subprocess.TimeoutExpired:
            raise TimeoutError(
                f"VBSP timed out after {timeout}s on {vmf_path.name}")
        except FileNotFoundError:
            raise VBSPNotFoundError(f"Could not execute: {vbsp_exe}")
        
        # Combine stdout and stderr for parsing
        full_output = result.stdout + '\n' + result.stderr
        
        if verbose:
            for line in result.stdout.splitlines():
                print(f"    [vbsp] {line}", flush=True)
        
        # Always try to parse VERTCOUNT first (even on non-zero exit)
        match = _VERTCOUNT_RE.search(full_output)
        if match:
            exceeded = bool(_EXCEEDED_RE.search(full_output))
            
            # Parse leafface count (may not be present in older builds)
            lf_match = _LEAFFACECOUNT_RE.search(full_output)
            leafface_count = int(lf_match.group(1)) if lf_match else 0
            leafface_exceeded = bool(_LEAFFACE_EXCEEDED_RE.search(full_output))
            
            # Parse face count
            fc_match = _FACECOUNT_RE.search(full_output)
            face_count = int(fc_match.group(1)) if fc_match else 0
            face_exceeded = bool(_FACE_EXCEEDED_RE.search(full_output))
            
            return VertexCount(
                int(match.group(1)), exceeded,
                leafface_count, leafface_exceeded,
                face_count, face_exceeded,
            )
        
        # Non-zero exit without VERTCOUNT — may be intermittent (e.g. Steam init)
        if result.returncode != 0:
            # Show full output for diagnosis
            output_tail = full_output.strip()[-500:]
            if (not _is_crash_exit(result.returncode)
                    and _SCALE_REJECTION_RE.search(full_output)):
                # A deterministic capacity error: retrying cannot change it
                raise VBSPRejectedError(
                    f"VBSP rejected {vmf_path.name} (exit code "
                    f"{result.returncode}):\n{output_tail}")
            last_error = VBSPCompileError(
                f"VBSP exited with code {result.returncode} "
                f"(attempt {attempt}/{max_retries}):\n{output_tail}")
            if attempt < max_retries:
                if verbose:
                    print(f"  VBSP attempt {attempt} failed, retrying...",
                          flush=True)
                continue
            raise last_error
        
        # Zero exit but no VERTCOUNT — shouldn't happen with -countverts build
        raise VBSPParseError(
            f"VERTCOUNT not found in VBSP output. "
            f"Is this VBSP built with -countverts support?\n"
            f"Output (last 500 chars): {full_output[-500:]}")
    
    # Should never reach here, but just in case
    raise last_error or VBSPError("VBSP failed after all retries")


def write_temp_vmf(root, output_path: Path) -> Path:
    """Write a temporary VMF file for VBSP vertex counting.
    
    Args:
        root: Parsed VMF KVNode tree (with modified lightmapscales)
        output_path: Path to write the temp VMF
        
    Returns:
        Path to the written VMF file
    """
    from vmf_parser import VMFWriter
    
    writer = VMFWriter()
    writer.write_file(root, output_path)
    return output_path


class ScratchVMF:
    """A private, uniquely named VMF used for trial counts and compiles.

    It lives in the SAME directory as the anchor VMF (the user's input map),
    never in a temp directory and never under the user's output name:

      * VBSP resolves func_instance files relative to the VMF being compiled,
        so a trial VMF anywhere else silently loses its instances and the
        vertex counts (and baseline lighting) are wrong.
      * Compile products (.bsp/.prt/.log/.lin) land next to it under the
        scratch name, so nothing the tool deletes can ever be a file the user
        made -- unlike artifacts named after the output VMF, which collide
        with the user's own compile of their previous result.

    Use as a context manager; close() removes the VMF and everything the
    compilers wrote next to it. If the anchor directory is not writable the
    scratch falls back to the system temp directory (instances then resolve
    through the game's InstancePath only; a warning says so).
    """

    #: Files the compilers write next to a VMF/BSP (VRAD's .r0 is incremental state).
    ARTIFACT_EXTS = ('.bsp', '.prt', '.log', '.lin', '.r0')

    def __init__(self, anchor_vmf: Path, label: str = 'scratch'):
        anchor = Path(anchor_vmf).resolve()
        #: True when the input folder was not writable and the scratch lives in
        #: the system temp dir: such a compile does not see the map's relative
        #: instances, so its products must never be cached as the map's baseline.
        self.relocated = False
        # <stem>.lmopt_<label>_h<host>_p<pid>_<random>.vmf — the owning host
        # and pid in the name are what let a later run sweep the leftovers of
        # a hard-killed one (see sweep_stale_scratch); the label is
        # letters/underscores only.
        prefix = f"{anchor.stem}.lmopt_{label}_{owner_tag()}_"
        try:
            fd, name = tempfile.mkstemp(prefix=prefix, suffix='.vmf',
                                        dir=str(anchor.parent))
        except OSError:
            print(f"  WARNING: {anchor.parent} is not writable; compiling "
                  f"scratch VMFs from the system temp directory (relative "
                  f"func_instance paths will not resolve).", file=sys.stderr,
                  flush=True)
            fd, name = tempfile.mkstemp(prefix=prefix, suffix='.vmf')
            self.relocated = True
        os.close(fd)
        self.path = Path(name)
        self._extra: list = []
        self._closed = False

    @property
    def bsp(self) -> Path:
        """Where VBSP puts the compiled BSP for this scratch VMF."""
        return self.path.with_suffix('.bsp')

    def write(self, root) -> Path:
        """Write a parsed VMF tree to the scratch file."""
        from vmf_parser import VMFWriter
        VMFWriter().write_file(root, self.path)
        return self.path

    def mirror_level_rad(self, source_rad: Optional[Path]) -> None:
        """Copy the map's implicit <map>.rad next to the scratch VMF.

        VRAD loads '<bsp stem>.rad' as the level's own light definitions; the
        scratch BSP has a different stem than the real map, so the real
        map's file has to be copied under the scratch stem to be seen.
        """
        if source_rad and Path(source_rad).is_file():
            dest = self.path.with_suffix('.rad')
            shutil.copyfile(source_rad, dest)
            self._extra.append(dest)

    def clear_artifacts(self) -> None:
        """Remove compiler products so the next compile starts from nothing."""
        for ext in self.ARTIFACT_EXTS:
            _unlink(self.path.with_suffix(ext))

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.clear_artifacts()
        for p in self._extra:
            _unlink(p)
        _unlink(self.path)

    def __enter__(self) -> 'ScratchVMF':
        return self

    def __exit__(self, *exc) -> None:
        self.close()


_MACHINE_ID: Optional[str] = None


def _machine_id() -> str:
    """The OS's per-install machine id, or '' if there is none to read:
    Windows' HKLM\\SOFTWARE\\Microsoft\\Cryptography\\MachineGuid, else
    /etc/machine-id (systemd) or /var/lib/dbus/machine-id. Read once."""
    global _MACHINE_ID
    if _MACHINE_ID is None:
        value = ''
        try:
            if sys.platform == 'win32':
                import winreg
                with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,
                                    r'SOFTWARE\Microsoft\Cryptography', 0,
                                    winreg.KEY_READ | winreg.KEY_WOW64_64KEY) as key:
                    value = str(winreg.QueryValueEx(key, 'MachineGuid')[0])
            else:
                for path in ('/etc/machine-id', '/var/lib/dbus/machine-id'):
                    try:
                        value = Path(path).read_text(encoding='ascii').strip()
                    except (OSError, ValueError):
                        continue
                    if value:
                        break
        except (OSError, ImportError):
            value = ''
        _MACHINE_ID = value.strip().lower()
    return _MACHINE_ID


def host_tag() -> str:
    """Identity of this machine for temp-file names: a readable prefix (the
    first 12 [a-z0-9] characters of the hostname) plus 8 hex digits of a hash
    of the FULL lowercase hostname and the OS machine id (_machine_id), so
    hosts that only differ late in a long name (LAB-GPU-SERVER-PRIMARY-
    MAPPING-01 / -02), and cloned VMs that kept one hostname but got their own
    machine id, never share a tag. A sweep only deletes files its own host
    made: a map folder on a network share can have a live run on another
    machine, whose pid means nothing here. Clones that share the hostname AND
    the machine id (an un-sysprepped Windows image) still share a tag."""
    import hashlib
    name = socket.gethostname().lower()
    readable = re.sub(r'[^a-z0-9]', '', name)[:12]
    identity = name + '\0' + _machine_id()
    return readable + hashlib.sha1(identity.encode('utf-8', 'replace')).hexdigest()[:8]


def owner_tag(pid: Optional[int] = None) -> str:
    """'h<host>_p<pid>': the owner stamp inside every scratch/temp file name."""
    return f"h{host_tag()}_p{os.getpid() if pid is None else pid}"


def atomic_tmp_name(target_name: str, hidden: bool) -> str:
    """Name of the temp file used to write `target_name` atomically.

    hidden=True is the form beside a published output file
    ('.<output>.lmopt_h<host>_p<pid>.tmp'); hidden=False is the form inside the
    private baseline cache directory. Both carry the owner stamp so the sweep
    can recognise exactly these and nothing else.
    """
    return f"{'.' if hidden else ''}{target_name}.lmopt_{owner_tag()}.tmp"


# Exactly the names this tool creates:
#   scratch VMF  <stem>.lmopt_<label>_h<host>_p<pid>_<8 random chars>.vmf   (map folder)
#                plus its compiler products / mirrored .rad under the same stem
#   publish temp .<output name>.lmopt_h<host>_p<pid>.tmp                    (output folder)
#   cache temp   <name>.lmopt_h<host>_p<pid>.tmp                            (cache folder)
_SCRATCH_RE = re.compile(
    r'^(?P<base>.+\.lmopt_[a-z_]+_h(?P<host>[a-z0-9]+)_p(?P<pid>\d+)_[a-z0-9_]{8})'
    r'(?P<ext>\.(?:vmf|bsp|prt|log|lin|r0|rad))$')
_TMP_RE = re.compile(
    r'^(?P<dot>\.?)(?P<target>.+)\.lmopt_h(?P<host>[a-z0-9]+)_p(?P<pid>\d+)\.tmp$')

#: A dead owner's files are only removed once they are this old (seconds).
#: A live run on another machine looks dead from here (its pid is not ours),
#: and a pid can be reused; several minutes of silence rules both out.
SWEEP_MIN_AGE_SECONDS = 600


def pid_alive(pid: int) -> bool:
    """True if a process with this id is running, or if that cannot be told.

    Best effort and conservative: PID reuse can only make a dead owner look
    alive (a sweep then skips its files), and so does every case that cannot
    be checked -- a pid outside the OS's range (a name this tool could not
    have produced) or any OS-call failure. It never raises.
    """
    if pid == os.getpid():
        return True
    if not 0 < pid <= 0xFFFFFFFF:
        return True
    try:
        if sys.platform == 'win32':
            import ctypes
            kernel32 = ctypes.WinDLL('kernel32', use_last_error=True)
            kernel32.OpenProcess.restype = ctypes.c_void_p
            handle = kernel32.OpenProcess(0x1000, False, pid)  # QUERY_LIMITED_INFORMATION
            if not handle:
                return ctypes.get_last_error() == 5            # access denied: it exists
            try:
                code = ctypes.c_ulong()
                if not kernel32.GetExitCodeProcess(ctypes.c_void_p(handle), ctypes.byref(code)):
                    return True
                return code.value == 259                       # STILL_ACTIVE
            finally:
                kernel32.CloseHandle(ctypes.c_void_p(handle))
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True
    except Exception:  # noqa: BLE001 - unverifiable means "assume alive"
        return True


def sweep_stale_scratch(directory: Path, kind='scratch',
                        min_age: float = SWEEP_MIN_AGE_SECONDS,
                        now: Optional[float] = None) -> int:
    """Delete files left by runs of THIS tool on THIS machine that were killed
    before they could clean up (a hard Cancel skips finally blocks). Returns
    how many files were removed.

    kind (one name, or a tuple to share a single pass over a big folder such
    as the system temp dir) selects which exact name shape is eligible:
      'scratch'  stamped scratch sets in the map folder: the scratch VMF, its
                 compiler products and mirrored .rad (any one of them
                 identifies the set, so products whose VMF is gone are
                 collected too)
      'publish'  hidden '.<output>.lmopt_h<host>_p<pid>.tmp' in the output folder
      'cache'    '<name>.lmopt_h<host>_p<pid>.tmp' in the baseline cache folder
    A file is removed only if its name matches that shape exactly, the host in
    the name is this machine, the pid in the name is not running, and the
    newest file of the set is at least min_age seconds old. Anything else in
    the directory -- including hidden files that merely look like temp files --
    is left alone.
    """
    kinds = (kind,) if isinstance(kind, str) else tuple(kind)
    directory = Path(directory)
    now = time.time() if now is None else now
    me = host_tag()
    removed = 0
    try:
        names = os.listdir(directory)
    except OSError:
        return 0
    for name in names:
        if '.lmopt_' not in name:      # cheap pre-filter: every stamped name has it
            continue
        path = directory / name
        for one in kinds:
            if one == 'scratch':
                m = _SCRATCH_RE.match(name)
                # The whole stamped set (VMF, compiler products, mirrored
                # .rad) goes together, found from ANY member's own name: a run
                # killed after its VMF was gone still leaves stamped products
                # behind. Members are '<stamped base><ext>' -- never the
                # user's '<stem>.bsp'.
                members = ([path.with_name(m.group('base') + e) for e in
                            ('.vmf',) + ScratchVMF.ARTIFACT_EXTS + ('.rad',)]
                           if m else [])
            else:
                m = _TMP_RE.match(name)
                if m and (m.group('dot') == '.') != (one == 'publish'):
                    m = None
                members = [path] if m else []
            if not m or m.group('host') != me or not path.is_file():
                continue
            if pid_alive(int(m.group('pid'))):
                continue
            try:
                newest = max(v.stat().st_mtime for v in members if v.is_file())
            except (OSError, ValueError):
                continue
            if now - newest < min_age:
                continue
            for victim in members:
                try:
                    if victim.is_file():
                        victim.unlink()
                        removed += 1
                except OSError:
                    pass
    return removed


def _unlink(path: Path) -> None:
    try:
        if path.is_file():
            path.unlink()
    except OSError:
        pass


def compile_bsp(vbsp_exe: Path,
                vmf_path: Path,
                game_dir: Path,
                bin_root: Path = None,
                timeout: int = 300,
                verbose: bool = False,
                max_retries: int = 2,
                extra_args: list = None) -> Path:
    """Run a full VBSP compile to produce a BSP file.
    
    Unlike count_vertices() which uses -countverts (early exit),
    this performs a complete VBSP compilation so the resulting BSP
    can be used for VRAD light counting.
    
    Args:
        vbsp_exe: Path to vbsp_lmo.exe
        vmf_path: Path to the VMF file to compile
        game_dir: Game directory for VBSP's -game flag
        bin_root: Bin root directory for VBSP's -binroot flag
        timeout: Maximum seconds to wait for VBSP (default: 300)
        verbose: If True, print VBSP's full output
        max_retries: Number of attempts (default: 2)
        extra_args: Additional command-line arguments for VBSP
        
    Returns:
        Path to the compiled BSP file
        
    Raises:
        VBSPNotFoundError: If vbsp_exe doesn't exist
        VBSPCompileError: If VBSP exits with an error after all retries
    """
    vbsp_exe = Path(vbsp_exe).resolve()
    vmf_path = Path(vmf_path).resolve()
    game_dir = Path(game_dir).resolve()
    
    if not vbsp_exe.is_file():
        raise VBSPNotFoundError(f"VBSP executable not found: {vbsp_exe}")
    
    if not vmf_path.is_file():
        raise VBSPError(f"VMF file not found: {vmf_path}")
    
    if not game_dir.is_dir():
        raise VBSPError(f"Game directory not found: {game_dir}")
    
    # Build command line
    cmd = [
        str(vbsp_exe),
        '-game', str(game_dir),
    ]
    if bin_root:
        cmd.extend(['-binroot', str(Path(bin_root).resolve())])
    
    if extra_args:
        cmd.extend(extra_args)
        
    cmd.append(str(vmf_path))
    
    env = os.environ.copy()
    if bin_root:
        # Prepend the binroot to PATH so that VBSP can load MaterialSystem.dll and other engine DLLs
        bin_path_x64 = str((Path(bin_root) / 'bin' / 'x64').resolve())
        bin_path_32 = str((Path(bin_root) / 'bin').resolve())
        env['PATH'] = f"{bin_path_x64}{os.pathsep}{bin_path_32}{os.pathsep}{env.get('PATH', '')}"
    
    if verbose:
        print(f"  VBSP compile: {' '.join(cmd)}", flush=True)
    
    # Record pre-existing BSP state so we can detect stale files
    bsp_path = vmf_path.with_suffix('.bsp')
    pre_mtime = bsp_path.stat().st_mtime if bsp_path.is_file() else None
    
    last_error = None
    for attempt in range(1, max_retries + 1):
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=timeout,
                cwd=str(vbsp_exe.parent),
                env=env,
            )
        except subprocess.TimeoutExpired:
            raise TimeoutError(
                f"VBSP timed out after {timeout}s on {vmf_path.name}")
        except FileNotFoundError:
            raise VBSPNotFoundError(f"Could not execute: {vbsp_exe}")
        
        full_output = result.stdout + '\n' + result.stderr
        
        if verbose:
            for line in result.stdout.splitlines():
                print(f"    [vbsp] {line}", flush=True)
        
        # Check for BSP file — VBSP often exits with code 1 for
        # non-fatal warnings (e.g. leaked maps: "Using unoptimized BSP
        # data on this map may cause engine crashes!") but still produces
        # a valid BSP.
        #
        # IMPORTANT: Only accept BSPs that were freshly created or modified
        # by this invocation. A stale BSP from a prior compile must not be
        # silently reused — that leads to optimizing against wrong data.
        if bsp_path.is_file():
            post_mtime = bsp_path.stat().st_mtime
            is_fresh = (pre_mtime is None or post_mtime > pre_mtime)
            
            if is_fresh:
                if result.returncode != 0 and verbose:
                    print(f"  VBSP exited with code {result.returncode} "
                          f"but BSP was produced (non-fatal warning)",
                          flush=True)
                return bsp_path
            else:
                # BSP exists but wasn't touched — stale from a previous run
                if verbose:
                    print(f"  WARNING: BSP file exists but was NOT modified "
                          f"by this VBSP invocation (stale)", flush=True)
        
        if result.returncode == 0 and not bsp_path.is_file():
            raise VBSPError(
                f"VBSP completed but BSP file not found: {bsp_path}")
        
        # Non-zero exit AND no fresh BSP produced — real failure
        output_tail = full_output.strip()[-500:]
        last_error = VBSPCompileError(
            f"VBSP exited with code {result.returncode} "
            f"(attempt {attempt}/{max_retries}):\n{output_tail}")
        if attempt < max_retries:
            if verbose:
                print(f"  VBSP attempt {attempt} failed, retrying...",
                      flush=True)
            continue
        raise last_error
    
    raise last_error or VBSPError("VBSP failed after all retries")

