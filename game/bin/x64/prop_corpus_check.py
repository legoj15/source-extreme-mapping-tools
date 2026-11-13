#!/usr/bin/env python3
"""
prop_corpus_check -- which real game models does VBSP's prop_static path refuse,
and why.

VBSP checks every model before it uses it as a static prop (the model file in
LoadStudioModel, the vertex file in GetCollisionModel; #70). A model that fails
is dropped with a warning, so a check that is stricter than the file formats
guarantee loses props from ordinary maps without anyone noticing. This tool asks
the question on real content: it lists every .mdl a game (or a content folder)
mounts, compiles generated maps that hold one prop_static per model, and sorts
each model by what VBSP said about it:

  placed       the model is in the BSP's static prop dictionary
  not_static   refused because it is not a static prop: its header lacks the
               $staticprop flag, or the keyvalues text inside the MDL has a prop_data
               block without "allowstatic 1" (VBSP's IsStaticProp reads both). Expected
               for characters, weapons, physics props and the like. The flag is read
               again from the MDL header: a model refused as "not marked" whose header
               carries the flag, or one placed without it, fails the run.
  validation   refused by VBSP's own checks. The reason text is kept: a damaged or
               unreadable MDL or VVD, a file that is not a studio model, no
               collision hull ("Bad geometry"), or a studio version this VBSP does
               not read (reason starts "studio version"; a correct refusal, listed
               apart from the others and not counted as a failure).
  crash        VBSP died, hung or wrote no BSP on this model alone (a failed batch
               is split in halves until the model is found)
  other        anything else, kept word for word: a message not listed above, or
               a model that was neither placed nor mentioned

Run it after any change to the VBSP model checks:

  python prop_corpus_check.py --game "E:/Steam/steamapps/common/Half-Life 2 Deathmatch/hl2mp" \\
      [--vbsp <path to vbsp_lmo.exe>]

  --content <dir>  repeatable. Check the models in this folder (and in every *_dir.vpk
                   in it) instead of the game's own. VBSP still runs with --game's
                   engine files: a scratch game directory is built under --out whose
                   search path is the content folders first, then --game's own paths.
                   For games whose engine DLLs VBSP cannot load.
  --match <text>   only models whose path contains the text (quick runs)
  --limit N        only the first N models after --match
  --batch N        models per generated map (default 250)
  --keep           keep every map, its BSP and both logs, not only the map and VBSP's
                   output of batches that had a refusal, a crash or an unexpected message
  --baseline <report.json>  compare with an earlier report: a model that was placed and
                   is not now fails the run; other changes are only listed
  --explain <mdl>  print the numbers behind a refusal (MDL and VVD tables) and exit

A model whose geometry lies far from its own origin can miss the test box and make VBSP
say "outside the map"; such a model is compiled again alone in a box that holds its
geometry, and only a model that still misses is reported (in "other").

It runs one VBSP at a time. Before the first batch it compiles the same box with
no prop in it. If that fails, VBSP cannot run with this game directory at all (for
example its engine DLLs do not load), so the run stops there: no model is blamed
and no report is written. Each run writes a folder under --out holding
report.json (class and reason per model, mounts and folders that could not be read,
first-attempt failures, crash dumps seen) plus the map and VBSP's output of every batch
that had a refusal, a crash or an unexpected message. Crash dumps that appear in VBSP's
own folder during a run are listed with their paths and left where they are: the tool
cannot tell which process wrote them.

Exit code:
  0  clean: no crash (a batch that failed once and passed on retry counts as one),
     nothing in "other", no validation refusal except studio versions, no flag
     mismatch, no baseline regression, no VPK or folder that exists but cannot be read
  1  at least one of those happened (the summary ends with which)
  2  bad arguments, a missing game, content folder, baseline or VBSP, or a VBSP
     that cannot be started
  3  VBSP cannot compile even the empty map with this game directory
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import struct
import subprocess
import sys
import time
from collections import Counter, OrderedDict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

_HERE = Path(__file__).resolve().parent
for _p in (_HERE, _HERE / 'unit_test' / 'bspwalk'):     # vmfgen lives with the bspwalk fixtures
    if str(_p) not in sys.path:
        sys.path.append(str(_p))

import gameinfo_search          # noqa: E402  the compile tools' search path
import proc_tree                # noqa: E402  ends VBSP and its children on a timeout
import sprp_lightmap           # noqa: E402  reads the BSP's static prop lump
import vmfgen                   # noqa: E402  the repo's VMF builder
from vmf_parser import VMFWriter  # noqa: E402
from vpk_reader import VPKReader  # noqa: E402

DEFAULT_OUT = _HERE / 'unit_test' / 'unit_test_output' / 'prop_corpus'
DEFAULT_VBSP = _HERE / 'vbsp_lmo.exe'

CLASSES = ('placed', 'not_static', 'validation', 'crash', 'other')
CLASS_TITLES = {
    'placed': 'placed',
    'not_static': 'refused: not a static prop',
    'validation': 'refused by validation',
    'crash': 'VBSP crash or failure',
    'other': 'other',
}
LIST_CAP = 100          # models printed one by one per class; report.json holds them all
DICT_NAME_BYTES = 128   # DETAIL_NAME_LENGTH: room for a model path in the static prop dictionary
STUDIOHDR_FLAGS_STATIC_PROP = 0x10   # src/public/studio.h; studiohdr_t.flags is at byte 152


class VbspLaunchError(Exception):
    """VBSP exists but the system would not start it."""


# --- mounts and listing models --------------------------------------------------

def vpks_in(folder: Path, problems: Optional[List[dict]] = None) -> List[Path]:
    """The *_dir.vpk files directly inside a folder, in name order. A folder that cannot
    be listed is added to `problems` as an unreadable mount, not passed over."""
    try:
        return sorted((p for p in folder.iterdir()
                       if p.is_file() and p.name.lower().endswith('_dir.vpk')),
                      key=lambda p: p.name.lower())
    except OSError as e:
        if problems is not None:
            problems.append({'mount': str(folder), 'kind': 'unreadable',
                             'reason': f'cannot list the folder for VPKs: {e}'})
        return []


def content_mounts(folders: List[Path], problems: Optional[List[dict]] = None) -> List[Path]:
    """Each content folder as a game would mount it: its VPKs, then the folder."""
    mounts: List[Path] = []
    for folder in folders:
        mounts.extend(vpks_in(folder, problems))
        mounts.append(folder)
    return mounts


def list_models(mounts: List[Path]) -> Tuple["OrderedDict[str, str]", List[dict]]:
    """Every .mdl the mounts hold, as {lowercase name: mount it comes from}, and the
    mounts that could not be read.

    Walks the mounts in order and the first to hold a name wins, which is the copy
    VBSP's filesystem would open. A skipped mount is {mount, kind, reason}; kind is
    'missing' (not there) or 'unreadable' (there, but cannot be parsed or listed: a
    folder inside a mount that cannot be walked is reported the same way)."""
    found: "OrderedDict[str, str]" = OrderedDict()
    skipped: List[dict] = []
    seen = set()
    for loc in mounts:
        key = os.path.normcase(os.path.normpath(str(loc)))
        if key in seen:
            continue
        seen.add(key)
        label = f"{loc.parent.name}/{loc.name}"
        names: List[str] = []
        if loc.suffix.lower() == '.vpk':
            if not loc.is_file():
                skipped.append({'mount': str(loc), 'kind': 'missing', 'reason': 'no such file'})
                continue
            try:
                names = VPKReader(loc).list_files('models/', 'mdl')
            except (ValueError, OSError, struct.error) as e:
                skipped.append({'mount': str(loc), 'kind': 'unreadable', 'reason': str(e)})
                continue
        else:
            try:
                is_folder = loc.is_dir()
                has_models = is_folder and (loc / 'models').is_dir()
            except OSError as e:
                skipped.append({'mount': str(loc), 'kind': 'unreadable', 'reason': str(e)})
                continue
            if not is_folder:
                skipped.append({'mount': str(loc), 'kind': 'missing', 'reason': 'no such folder'})
                continue
            if has_models:
                def walk_failed(err, loc=loc):
                    skipped.append({'mount': str(loc), 'kind': 'unreadable',
                                    'reason': f'cannot list {err.filename}: {err.strerror or err}'})
                for dirpath, _dirs, files in os.walk(loc / 'models', onerror=walk_failed):
                    for f in files:
                        if f.lower().endswith('.mdl'):
                            rel = (Path(dirpath) / f).relative_to(loc)
                            names.append(str(rel).replace('\\', '/').lower())
        for n in sorted(names):
            found.setdefault(n.lower(), label)
    return OrderedDict(sorted(found.items())), skipped


def _norm(p: Path) -> str:
    return os.path.normcase(os.path.normpath(str(p)))


def _gameinfo_location(p: Path) -> str:
    """How gameinfo.txt names a mount: x.vpk for x_dir.vpk, a folder as it is."""
    s = str(p).replace('\\', '/')
    if s.lower().endswith('_dir.vpk'):
        short = s[:-len('_dir.vpk')]
        # x_dir_dir.vpk would read as x_dir.vpk, which names another file: keep the full name
        if not short.lower().endswith('_dir'):
            s = short + '.vpk'
    return s


def write_scratch_game(scratch: Path, base_game: Path, mounts: List[Path],
                       binroot: Optional[Path] = None) -> List[Path]:
    """A game directory whose gameinfo.txt mounts `mounts` (absolute) after the
    scratch folder itself. Returns the mounts the gameinfo_search resolver does not
    find in it (empty when the Python listing and VBSP see the same mounts)."""
    app_id = ''
    gameinfo = gameinfo_search.read_gameinfo(base_game) or []
    for key, value in gameinfo:
        if key.lower() == 'filesystem' and isinstance(value, list):
            for k, v in value:
                if k.lower() == 'steamappid' and isinstance(v, str):
                    app_id = v
    lines = ['"GameInfo"', '{', '\tgame\t"prop_corpus_check scratch"', '\tFileSystem', '\t{']
    if app_id:
        lines.append(f'\t\tSteamAppId\t{app_id}')
    lines += ['\t\tSearchPaths', '\t\t{',
              '\t\t\tgame+mod+mod_write+default_write_path\t|gameinfo_path|.']
    lines += [f'\t\t\tgame\t"{_gameinfo_location(m)}"' for m in mounts]
    lines += ['\t\t}', '\t}', '}', '']
    scratch.mkdir(parents=True, exist_ok=True)
    (scratch / 'gameinfo.txt').write_text('\n'.join(lines), encoding='utf-8')
    resolved = {_norm(p) for p in gameinfo_search.search_paths(scratch, binroot)}
    return [m for m in mounts if _norm(m) not in resolved]


# --- generated maps -----------------------------------------------------------

ROOM_MIN, ROOM_MAX = (-4096, -4096, 0), (4096, 4096, 4096)     # the box's inside
PROP_Z = 2048           # height of every prop's origin
MAP_LIMIT = 15000       # the largest coordinate a box may reach (the map limit is 16384)


def write_batch_vmf(path: Path, models: List[str],
                    room: Optional[Tuple[tuple, tuple]] = None) -> None:
    """A sealed hollow box holding one prop_static per model, on a grid. `room` is a
    box (mins, maxs) elsewhere in the map for a model whose geometry lies far from its
    origin: that model sits at the origin 0 0 0, which VBSP's leak check ignores."""
    m = vmfgen.Map(name=path.stem)
    lo, hi = room or (ROOM_MIN, ROOM_MAX)
    m.room(lo, hi, marker=True)
    cols = max(1, math.ceil(math.sqrt(len(models))))
    step = min(256, 7000 // cols)
    x0 = -(cols - 1) * step // 2
    for i, name in enumerate(models):
        r, c = divmod(i, cols)
        if room:
            m.entity('prop_static', keys={'origin': '0 0 0', 'model': name, 'solid': '6',
                                          'skin': '0'})
        else:
            m.entity('prop_static', origin=(x0 + c * step, x0 + r * step, PROP_Z), model=name,
                     solid='6', skin='0')
    VMFWriter().write_file(m.to_kv(), path)


# --- reading a model's own files (--explain, and placing models whose geometry is
# --- far from their origin) -------------------------------------------------------

def internal_vvd_name(mdl: bytes) -> str:
    """The VVD VBSP's vertex check loads for this MDL: models/<pszName()> with the
    extension .vvd, where pszName() is the studiohdr2 name when it has one, else the
    name field (offsets of studiohdr_t in src/public/studio.h: name[64] at 12,
    studiohdr2index at 400, studiohdr2_t.sznameindex at 20)."""
    internal = mdl[12:76].split(b'\0')[0].decode('latin-1')
    if len(mdl) >= 408:
        h2, = struct.unpack_from('<i', mdl, 400)
        if h2 > 0 and h2 + 24 <= len(mdl):
            sz, = struct.unpack_from('<i', mdl, h2 + 20)
            if 0 < sz and h2 + sz < len(mdl):
                internal = mdl[h2 + sz:].split(b'\0')[0].decode('latin-1')
    return 'models/' + os.path.splitext(internal.replace('\\', '/').lstrip('/').lower())[0] + '.vvd'


def lod0_bounds(vvd: bytes) -> Optional[Tuple[list, list]]:
    """Smallest box round the finite LOD 0 vertex positions of a VVD, or None.
    vertexFileHeader_t: numLODVertexes[0] at 16, vertexDataStart at 56;
    mstudiovertex_t is 48 bytes with the position at 16."""
    if len(vvd) < 64:
        return None
    count, = struct.unpack_from('<i', vvd, 16)
    start, = struct.unpack_from('<i', vvd, 56)
    if count <= 0 or start <= 0 or start + 48 * count > len(vvd):
        return None
    pos = [struct.unpack_from('<3f', vvd, start + 48 * i + 16) for i in range(count)]
    pos = [p for p in pos if all(math.isfinite(c) for c in p)]
    if not pos:
        return None
    return ([min(p[a] for p in pos) for a in range(3)], [max(p[a] for p in pos) for a in range(3)])


def room_around(bounds: Tuple[list, list]) -> Optional[Tuple[tuple, tuple]]:
    """A box that holds the part of the geometry inside the map limit, when the prop's
    origin is 0 0 0; None when none of it is (no map can hold the model at this origin)."""
    lo, hi = [], []
    for a in range(3):
        c_lo, c_hi = max(bounds[0][a], -MAP_LIMIT), min(bounds[1][a], MAP_LIMIT)
        if c_lo > c_hi:
            return None
        lo.append(int(math.floor(c_lo)) - 128)
        hi.append(int(math.ceil(c_hi)) + 128)
    return tuple(max(v, -MAP_LIMIT) for v in lo), tuple(min(v, MAP_LIMIT) for v in hi)


# --- running VBSP and reading what it said ------------------------------------

def is_crash_exit(rc: Optional[int]) -> bool:
    """A signal death (negative) or a Windows crash code such as 0xC0000005."""
    return rc is not None and (rc < 0 or rc >= 0x80000000)


def run_vbsp(vbsp: Path, game: Path, binroot: Optional[Path], vmf: Path,
             timeout: int) -> Tuple[Optional[int], str, float]:
    """Compile one map. Returns (exit code or None on timeout, output, seconds).
    Raises VbspLaunchError when the system will not start VBSP."""
    cmd = [str(vbsp), '-game', str(game)]
    if binroot:
        cmd += ['-binroot', str(binroot)]
    cmd.append(str(vmf))
    env = os.environ.copy()
    if binroot:
        env['PATH'] = os.pathsep.join([str(Path(binroot) / 'bin' / 'x64'),
                                       str(Path(binroot) / 'bin'), env.get('PATH', '')])
    start = time.time()
    # proc_tree starts VBSP so that a timeout can end it and anything it started.
    try:
        proc = proc_tree.launch(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                cwd=str(vbsp.parent), env=env)
    except OSError as e:
        raise VbspLaunchError(f"cannot start {vbsp}: {e}") from None
    try:
        out, _ = proc.communicate(timeout=timeout)
        rc: Optional[int] = proc.returncode
    except subprocess.TimeoutExpired:
        proc_tree.kill_tree(proc)
        out, _ = proc.communicate()
        rc = None
    finally:
        proc_tree.release(proc)
    return rc, (out or b'').decode('utf-8', 'replace'), time.time() - start


# What VBSP's static prop path says when it refuses a model
# (src/utils/vbsp/staticprop.cpp: LoadStudioModel, GetCollisionModel, AddStaticPropToLump).
# The model name is always quoted, so spaces in it are fine.
_RE_VALIDATION = re.compile(r'Error! \w+ model "([^"]+)": (.*?)\.( Deleted\.)?\s*$')
_RE_NOT_MARKED = re.compile(r'Error! To use model "([^"]+)"')
_RE_DYNAMIC = re.compile(r'Error! \w+ using model "([^"]+)", which must be used on a dynamic')
_RE_LOAD_FAILED = re.compile(r'Error loading studio model "([^"]+)"!')
_RE_BAD_GEOMETRY = re.compile(r'Bad geometry on "([^"]+)"!')


def _models_named(line: str, batch: set) -> List[str]:
    """Models of the batch whose path appears in an unrecognised line (the line is
    lowercase). Not matched by pattern, so a name with spaces or an unquoted name
    still finds its model."""
    if 'models/' not in line:
        return []
    named = []
    for model in batch:
        at = line.find(model)
        while at != -1:
            before = line[at - 1] if at else ' '
            after = line[at + len(model)] if at + len(model) < len(line) else ' '
            if not (before.isalnum() or before in '_-.') and not after.isalnum():
                named.append(model)
                break
            at = line.find(model, at + 1)
    return named


def parse_log(output: str, batch: set) -> Tuple[Dict[str, list], List[str]]:
    """Events per model, and Error/Warning lines that name no model of the batch.

    An event is (kind, reason, stage, line). kind is one of validation,
    not_marked, dynamic, load_failed, bad_geometry, unknown."""
    events: Dict[str, list] = {}
    stray: List[str] = []

    def add(model, kind, reason, stage, line):
        events.setdefault(model.lower(), []).append((kind, reason, stage, line))

    for raw in output.splitlines():
        line = raw.strip()
        if not line:
            continue
        m = _RE_DYNAMIC.search(line)
        if m:
            add(m.group(1), 'dynamic', 'prop data says it must be dynamic', '', line)
            continue
        m = _RE_NOT_MARKED.search(line)
        if m:
            add(m.group(1), 'not_marked', 'not compiled with $staticprop', '', line)
            continue
        m = _RE_VALIDATION.search(line)
        if m:
            add(m.group(1), 'validation', m.group(2), 'vvd' if m.group(3) else 'mdl', line)
            continue
        m = _RE_LOAD_FAILED.search(line)
        if m:
            add(m.group(1), 'load_failed', 'could not load the model', '', line)
            continue
        m = _RE_BAD_GEOMETRY.search(line)
        if m:
            add(m.group(1), 'bad_geometry', 'VBSP built no collision hull from its vertices '
                '(it said Bad geometry)', 'hull', line)
            continue
        named = _models_named(line.lower(), batch)
        if named:
            for n in named:
                add(n, 'unknown', line, '', line)
        elif re.match(r'(?i)^(error|warning)\b', line):
            if line not in stray:
                stray.append(line)
    return events, stray


def read_placed(bsp: Path) -> Tuple[Optional[set], str]:
    """Model names in the BSP's static prop dictionary, and why not when None."""
    try:
        return {p['model'].lower() for p in sprp_lightmap.parse(bsp.read_bytes())}, ''
    except SystemExit as e:                  # the lump reader exits with its message
        return None, f"static prop lump: {e.code}"
    except (OSError, struct.error, IndexError, ValueError) as e:
        return None, f"static prop lump unreadable: {e}"


def classify(model: str, events: list, placed: set, head: Optional[bytes]) -> dict:
    """The class and reason of one model, from its log events, the BSP, and the
    first bytes of its MDL (None when the tool's search path cannot find it)."""
    kinds = {e[0] for e in events}
    for e in events:
        if e[0] == 'validation':
            return {'class': 'validation', 'reason': e[1], 'stage': e[2], 'line': e[3]}
    for e in events:
        if e[0] in ('not_marked', 'dynamic'):
            return {'class': 'not_static', 'reason': e[1], 'line': e[3], 'rule': e[0]}
    for e in events:
        if e[0] == 'bad_geometry':
            return {'class': 'validation', 'reason': e[1], 'stage': e[2], 'line': e[3]}
    unexpected = [e[3] for e in events if e[0] == 'unknown']
    if unexpected:
        return {'class': 'other', 'reason': '; '.join(unexpected), 'lines': unexpected}
    if kinds == {'load_failed'}:
        said = [e[3] for e in events]
        # VBSP says only "Error loading studio model" for a missing file and for one
        # whose first four bytes are not IDST or IDAG; the file itself tells which.
        if head is not None and head[:4] not in (b'IDST', b'IDAG'):
            what = (f"starts with {head[:4]!r}" if len(head) >= 4
                    else f"is only {len(head)} bytes long")
            return {'class': 'validation', 'stage': 'magic', 'lines': said,
                    'reason': f"not a studio model: the file {what}, not IDST or IDAG "
                              "(VBSP said only that it could not load it)"}
        return {'class': 'other', 'lines': said,
                'reason': 'VBSP could not load the model and said nothing else '
                          + ('(its file starts like a studio model)' if head is not None
                             else '(the tool cannot find the file either)')}
    if model in placed:
        return {'class': 'placed', 'reason': ''}
    # The BSP's dictionary keeps 128 bytes of a model's path, without an end mark when
    # the path is longer, so a long path reads back cut off.
    raw = model.encode('utf-8', 'replace')
    if len(raw) > DICT_NAME_BYTES and raw[:DICT_NAME_BYTES].decode('utf-8', 'ignore') in placed:
        return {'class': 'placed', 'reason': f"its path is longer than {DICT_NAME_BYTES} bytes, "
                f"so the BSP's dictionary holds only the first {DICT_NAME_BYTES}"}
    return {'class': 'other', 'reason': 'VBSP said nothing about it, yet it is not in the BSP'}


def is_version_refusal(rec: dict) -> bool:
    """A validation refusal because the MDL is a studio version VBSP does not read."""
    return rec['class'] == 'validation' and rec['reason'].startswith('studio version')


class Checker:
    """One run over a list of models: compile in batches, bisect failures."""

    def __init__(self, vbsp: Path, game: Path, binroot: Optional[Path], run_dir: Path,
                 timeout: int, keep: bool, sources: Dict[str, str], read_paths: List[Path]):
        self.vbsp, self.game, self.binroot = vbsp, game, binroot
        self.dir, self.timeout, self.keep, self.sources = run_dir, timeout, keep, sources
        self.read_paths = read_paths
        self.results: Dict[str, dict] = {}
        self.runs: List[dict] = []
        self.crashes: List[dict] = []
        self.unreproduced: List[dict] = []
        self.dumps: List[dict] = []
        self.first_failures: List[dict] = []
        self.stray: List[str] = []
        self.runs_made = 0
        self.moved_runs = 0
        # models VBSP refused as "not marked $staticprop" / placed, checked against the
        # MDL header's flag; 'unreadable' = the header could not be read to check
        self.flag_checks = {'not_marked': 0, 'placed': 0, 'unreadable': 0}

    # -- helpers -----------------------------------------------------------------
    def _head(self, model: str) -> Optional[bytes]:
        """The first bytes of the model's MDL as the tool's search path finds it."""
        try:
            found = gameinfo_search.find_in(self.read_paths, model, strict=False)
            return found.read_bytes()[:16] if found else None
        except OSError:
            return None

    def _flags(self, model: str) -> Optional[int]:
        """studiohdr_t.flags of the model's MDL as the tool's search path finds it, or None."""
        try:
            found = gameinfo_search.find_in(self.read_paths, model, strict=False)
            data = found.read_bytes() if found else None
        except OSError:
            return None
        if not data or len(data) < 156 or data[:4] not in (b'IDST', b'IDAG'):
            return None
        return struct.unpack_from('<i', data, 152)[0]

    def _cross_check_flag(self, model: str, rec: dict) -> None:
        """VBSP refuses a model as "not marked" when the header lacks the static prop flag
        and places it only when the flag is there. Read the flag from the file and say so
        when VBSP's verdict disagrees: that is a failure of the check, not of the model."""
        if rec['class'] == 'not_static' and rec.get('rule') == 'not_marked':
            kind = 'not_marked'
        elif rec['class'] == 'placed':
            kind = 'placed'
        else:
            return
        flags = self._flags(model)
        if flags is None:
            self.flag_checks['unreadable'] += 1
            rec['flag_check'] = 'the MDL header could not be read'
            return
        self.flag_checks[kind] += 1
        carries = bool(flags & STUDIOHDR_FLAGS_STATIC_PROP)
        rec['flags'] = f'0x{flags & 0xFFFFFFFF:08x}'
        if kind == 'not_marked' and carries:
            rec['mismatch'] = 'refused as not marked, but the header carries the flag'
            rec['reason'] = (f"VBSP says it is not compiled with $staticprop, but the MDL header's "
                             f"flags ({rec['flags']}) carry the static prop bit")
        elif kind == 'placed' and not carries:
            rec['mismatch'] = 'placed, but the header lacks the flag'
            rec['reason'] = (f"placed although the MDL header's flags ({rec['flags']}) do not "
                             "carry the static prop bit")

    def _dump_files(self) -> set:
        """Crash dumps in VBSP's own folder, which is where VBSP runs and writes them."""
        return set(self.vbsp.parent.glob(f'{self.vbsp.stem}_*.mdmp'))

    def _note_dumps(self, name: str, before: set, run_failed: bool) -> None:
        """List the dumps that appeared in VBSP's folder during a run, where they are. They
        are not moved: nothing ties a dump to the VBSP this run started, so another
        process's dump would be taken for ours."""
        for dump in sorted(self._dump_files() - before):
            self.dumps.append({'batch': name, 'file': str(dump), 'run_failed': run_failed})

    # -- one compile ---------------------------------------------------------
    def _compile(self, name: str, models: List[str], room=None) -> dict:
        vmf = self.dir / f'{name}.vmf'
        write_batch_vmf(vmf, models, room)
        for ext in ('.bsp', '.prt', '.lin', '.log'):
            (self.dir / f'{name}{ext}').unlink(missing_ok=True)
        dumps_before = self._dump_files()
        rc, out, secs = run_vbsp(self.vbsp, self.game, self.binroot, vmf, self.timeout)
        self.runs_made += 1
        bsp = self.dir / f'{name}.bsp'
        placed, lump_error = read_placed(bsp) if bsp.is_file() else (None, '')
        failure = ''
        if rc is None:
            failure = f'timed out after {self.timeout} s'
        elif is_crash_exit(rc):
            failure = f'crashed (exit code 0x{rc & 0xFFFFFFFF:08X})'
        elif placed is None:
            failure = f'exit code {rc} and no usable BSP' + (f' ({lump_error})' if lump_error else '')
        self._note_dumps(name, dumps_before, bool(failure))
        return {'name': name, 'models': models, 'rc': rc, 'seconds': round(secs, 1),
                'output': out, 'placed': placed or set(), 'failure': failure}

    def _cleanup(self, run: dict, keep_files: bool) -> None:
        """Delete a run's files. keep_files keeps the map and VBSP's output; --keep
        also keeps the BSP and the log VBSP wrote itself."""
        name = run['name']
        for ext in ('.prt', '.lin') + (() if self.keep else ('.bsp', '.log')):
            (self.dir / f'{name}{ext}').unlink(missing_ok=True)
        if keep_files or self.keep:
            (self.dir / f'{name}.out.txt').write_text(run['output'], encoding='utf-8')
        else:
            (self.dir / f'{name}.vmf').unlink(missing_ok=True)

    def _keep_first_attempt(self, run: dict) -> None:
        """A first attempt that failed keeps its map and VBSP's output under their own
        names, so the retry (which reuses the batch name) does not overwrite them."""
        name = run['name']
        (self.dir / f'{name}.first_attempt.out.txt').write_text(run['output'], encoding='utf-8')
        try:
            (self.dir / f'{name}.vmf').replace(self.dir / f'{name}.first_attempt.vmf')
        except OSError:
            pass

    # -- models whose geometry is far from their origin ----------------------------
    def _compile_moved(self, name: str, models: List[str], progress: str) -> None:
        """VBSP says 'outside the map' when a model's hull misses the test box, which
        happens when its geometry lies far from its own origin. Compile each such model
        again alone, in a box placed round its geometry (the prop stays at the origin
        0 0 0); the new result replaces the first. A model whose geometry lies wholly
        beyond the map limit stays in 'other', with that said in its reason."""
        for m in models:
            rec = self.results.get(m, {})
            if not (rec.get('class') == 'other'
                    and any('outside the map' in l for l in rec.get('lines', []))):
                continue
            bounds = self._geometry_bounds(m)
            room = room_around(bounds) if bounds else None
            if not room:
                if bounds:
                    rec['reason'] += (f" [its geometry, x {bounds[0][0]:.0f} to {bounds[1][0]:.0f}, "
                                      f"y {bounds[0][1]:.0f} to {bounds[1][1]:.0f}, z "
                                      f"{bounds[0][2]:.0f} to {bounds[1][2]:.0f}, lies beyond the "
                                      f"map limit from its origin, so no map holds it there]")
                continue
            print(f"  {progress}{name}: {m} was outside the test box; compiling it again in "
                  "a box round its geometry", flush=True)
            self.moved_runs += 1
            self.process(f'{name}_moved{self.moved_runs}', [m], progress, room, relocate=False)
            rec = self.results[m]
            rec['box'] = [list(room[0]), list(room[1])]
            if rec['class'] == 'placed' and not rec.get('mismatch'):
                rec['reason'] = ("its geometry is far from its origin and missed the test box; "
                                 "placed in a box round its geometry")

    # -- can VBSP compile anything with this game directory? -------------------
    def canary(self) -> Optional[dict]:
        """Compile the box with no prop. Returns None when VBSP can, or the failed
        run (its files are kept) when it cannot: nonzero exit, timeout or no BSP."""
        run = None
        for attempt in range(2):             # a first failure may be a start-up hiccup
            run = self._compile('canary', [])
            if not run['failure'] and run['rc'] != 0:
                run['failure'] = f"exit code {run['rc']}"
            if attempt == 0 and run['failure']:
                self.first_failures.append({'batch': 'canary', 'models': 0,
                                            'failure': run['failure'], 'exit_code': run['rc'],
                                            'passed_on_retry': None})
                self._keep_first_attempt(run)
                continue
            if not run['failure']:
                if self.first_failures and self.first_failures[-1]['batch'] == 'canary':
                    self.first_failures[-1]['passed_on_retry'] = True
                self._cleanup(run, False)
                return None
        if self.first_failures and self.first_failures[-1]['batch'] == 'canary':
            self.first_failures[-1]['passed_on_retry'] = False
        self._cleanup(run, True)
        return run

    # -- one batch, with bisecting -------------------------------------------
    def _geometry_bounds(self, model: str):
        """Smallest box round the model's LOD 0 vertex positions (read from the VVD VBSP
        loads), or None when it cannot be worked out."""
        try:
            found = gameinfo_search.find_in(self.read_paths, model, strict=False)
            if not found:
                return None
            vvd = gameinfo_search.find_in(self.read_paths, internal_vvd_name(found.read_bytes()),
                                          strict=False)
            return lod0_bounds(vvd.read_bytes()) if vvd else None
        except (OSError, struct.error):
            return None

    def process(self, name: str, models: List[str], progress: str = '',
                room=None, relocate: bool = True) -> List[str]:
        """Compile and classify models; returns the models VBSP failed on alone."""
        run = self._compile(name, models, room)
        retried = False
        if run['failure']:           # a first failure may be a start-up hiccup: try once more
            first = run['failure']
            first_rc = run['rc']
            self._keep_first_attempt(run)
            run = self._compile(name, models, room)
            retried = True
            self.first_failures.append({'batch': name, 'models': len(models), 'failure': first,
                                        'exit_code': first_rc,
                                        'passed_on_retry': not run['failure']})
            if run['failure']:
                run['failure'] = f"{first}; again on retry: {run['failure']}"
        if not run['failure']:
            events, stray = parse_log(run['output'], set(models))
            self.stray.extend(s for s in stray if s not in self.stray)
            counts: Counter = Counter()
            had_validation = had_other = False
            for model in models:
                evs = events.get(model, [])
                head = self._head(model) if {e[0] for e in evs} == {'load_failed'} else None
                rec = classify(model, evs, run['placed'], head)
                self._cross_check_flag(model, rec)
                rec['source'] = self.sources.get(model, '')
                rec['batch'] = name
                self.results[model] = rec
                counts[rec['class']] += 1
                had_validation |= rec['class'] == 'validation'
                had_other |= rec['class'] == 'other'
            note = ' (first attempt failed, passed on retry)' if retried else ''
            self.runs.append({'name': name, 'models': len(models), 'seconds': run['seconds'],
                              'exit_code': run['rc'], 'retried': retried,
                              'counts': dict(counts)})
            print(f"  {progress}{name}: {len(models)} models, {run['seconds']:.0f} s{note}: "
                  + ', '.join(f"{CLASS_TITLES[c]} {counts[c]}" for c in CLASSES if counts[c]),
                  flush=True)
            self._cleanup(run, had_validation or had_other)
            if relocate:
                self._compile_moved(name, models, progress)
            return []

        # The batch failed twice: keep its map and log, then split it.
        print(f"  {progress}{name}: {len(models)} models FAILED: {run['failure']}", flush=True)
        self.runs.append({'name': name, 'models': len(models), 'seconds': run['seconds'],
                          'exit_code': run['rc'], 'failure': run['failure']})
        self._cleanup(run, True)
        crash = {'batch': name, 'models': list(models), 'failure': run['failure'],
                 'exit_code': run['rc'],
                 'log_tail': [l for l in run['output'].splitlines() if l.strip()][-8:]}
        self.crashes.append(crash)
        if len(models) == 1:
            culprit = models[0]
            tail = crash['log_tail'][-1] if crash['log_tail'] else ''
            self.results[culprit] = {'class': 'crash', 'source': self.sources.get(culprit, ''),
                                     'batch': name, 'reason': run['failure']
                                     + (f'; last line: {tail}' if tail else ''),
                                     'log_tail': crash['log_tail']}
            crash['culprits'] = [culprit]
            return [culprit]
        mid = len(models) // 2
        culprits = self.process(name + '_a', models[:mid], progress, room, relocate)
        culprits += self.process(name + '_b', models[mid:], progress, room, relocate)
        crash['culprits'] = culprits
        if not culprits:
            self.unreproduced.append({'batch': name, 'models': len(models),
                                      'failure': run['failure']})
            print(f"  {name}: both halves compiled, so the failure did not repeat "
                  f"(see {name}.vmf); this counts as a failure of the run", flush=True)
        return culprits


# --- the report -----------------------------------------------------------------

def normalise_reason(reason: str) -> str:
    """The reason with numbers and names blanked, to count alike refusals together.
    A studio version refusal keeps its numbers: the version is the point."""
    if reason.startswith('studio version'):
        return reason
    reason = re.sub(r"b'[^']*'", "b'...'", re.sub(r'"[^"]*"', '"..."', reason))
    return re.sub(r'\d+', 'N', reason)


def summarise(results: Dict[str, dict]) -> dict:
    by_class = Counter(r['class'] for r in results.values())
    reasons: Dict[str, Counter] = {c: Counter() for c in CLASSES}
    by_source: Dict[str, Counter] = {}
    versions = 0
    for r in results.values():
        if r['class'] != 'placed':
            # 'other' is kept word for word, the rest are counted with the numbers blanked
            reasons[r['class']][r['reason'] if r['class'] == 'other'
                                else normalise_reason(r['reason'])] += 1
        by_source.setdefault(r['source'], Counter())[r['class']] += 1
        versions += is_version_refusal(r)
    return {'counts': {c: by_class.get(c, 0) for c in CLASSES},
            'flag_mismatches': sum(1 for r in results.values() if r.get('mismatch')),
            'version_refusals': versions,
            'other_validation_refusals': by_class.get('validation', 0) - versions,
            'reasons': {c: dict(reasons[c]) for c in CLASSES if reasons[c]},
            'by_source': {s: dict(c) for s, c in sorted(by_source.items())}}


def compare_baseline(results: Dict[str, dict], old: Dict[str, dict]) -> dict:
    """Per-model class changes against an earlier report's models.

    regressions: was placed, is now anything else (fails the run). changes: any other
    class change (information). only_old / only_new: models one side lacks (listed)."""
    regressions, changes = [], []
    for model in sorted(set(results) & set(old)):
        was, now = old[model].get('class'), results[model]['class']
        if was == now:
            continue
        row = {'model': model, 'was': was, 'now': now, 'reason': results[model]['reason']}
        (regressions if was == 'placed' else changes).append(row)
    return {'regressions': regressions, 'changes': changes,
            'only_old': sorted(set(old) - set(results)),
            'only_new': sorted(set(results) - set(old))}


def failure_reasons(results: Dict[str, dict], summary: dict, checker: Checker,
                    skipped: List[dict], baseline: Optional[dict]) -> List[str]:
    """Why the run fails, in words (empty when it is clean)."""
    why = []
    if summary['other_validation_refusals']:
        why.append(f"{summary['other_validation_refusals']} model(s) refused by validation "
                   "for a reason other than the studio version")
    if summary['counts']['crash']:
        why.append(f"{summary['counts']['crash']} model(s) crash or hang VBSP")
    if checker.unreproduced:
        why.append(f"{len(checker.unreproduced)} batch(es) failed twice but both halves "
                   "compiled, so no model was found")
    passed = [f for f in checker.first_failures if f['passed_on_retry']]
    if passed:
        why.append(f"{len(passed)} run(s) of VBSP failed on the first attempt and passed on the "
                   "retry (" + ', '.join(f['batch'] for f in passed[:5])
                   + (', ...' if len(passed) > 5 else '') + "): VBSP failed once on real content")
    if summary['flag_mismatches']:
        why.append(f"{summary['flag_mismatches']} model(s) where VBSP's verdict disagrees with "
                   "the static prop flag in the MDL header")
    if checker.flag_checks['unreadable']:
        why.append(f"{checker.flag_checks['unreadable']} model(s) whose MDL header the tool could "
                   "not read, so VBSP's verdict on them was not checked against the file")
    if summary['counts']['other']:
        why.append(f"{summary['counts']['other']} model(s) in 'other' (VBSP said something "
                   "the tool does not understand, or nothing)")
    if baseline and baseline['regressions']:
        why.append(f"{len(baseline['regressions'])} model(s) placed in the baseline are not now")
    unreadable = [s for s in skipped if s['kind'] == 'unreadable']
    if unreadable:
        why.append(f"{len(unreadable)} mounted VPK(s) or folder(s) exist but could not be read, "
                   "so part of the corpus was not checked")
    return why


def print_summary(results: Dict[str, dict], summary: dict, checker: Checker,
                  seconds: float, label: str, skipped: List[dict],
                  baseline: Optional[dict], why: List[str]) -> None:
    print()
    print(label)
    print(f"Models checked: {len(results)}   VBSP runs: {checker.runs_made}   "
          f"time: {seconds:.0f} s")
    print()
    for c in CLASSES:
        print(f"  {CLASS_TITLES[c]:<30} {summary['counts'][c]:>6}")
        if c == 'validation' and summary['counts'][c]:
            print(f"      of which studio version    {summary['version_refusals']:>6}")
            print(f"      of which any other reason  {summary['other_validation_refusals']:>6}")
    for c in CLASSES[1:]:
        if summary['reasons'].get(c):
            print(f"\nReasons: {CLASS_TITLES[c]}")
            for reason, n in sorted(summary['reasons'][c].items(), key=lambda kv: -kv[1]):
                print(f"  {n:>6}  {reason}")
    print("\nBy mount:")
    for source, counts in summary['by_source'].items():
        print(f"  {source:<40} " + ', '.join(f"{c} {counts[c]}" for c in CLASSES if counts.get(c)))
    fc = checker.flag_checks
    print(f"\nStatic prop flag in the MDL header against VBSP's verdict: {fc['not_marked']} "
          f"model(s) refused as not marked and {fc['placed']} placed model(s) checked, "
          f"{summary['flag_mismatches']} disagree"
          + (f", {fc['unreadable']} header(s) could not be read" if fc['unreadable'] else ''))
    for m, r in sorted(results.items()):
        if r.get('mismatch'):
            print(f"  MISMATCH {m}  [{r['source']}]: {r['mismatch']} (flags {r['flags']})")
    if skipped:
        print(f"\nMounts and folders that could not be read ({len(skipped)}); models in them "
              "were not checked:")
        for s in skipped:
            print(f"  {s['kind']:<10} {s['mount']}  ({s['reason']})")

    def listing(title, rows):
        if not rows:
            return
        print(f"\n{title} ({len(rows)}):")
        for m, r in rows[:LIST_CAP]:
            print(f"  {m}  [{r['source']}]\n      {r['reason']}")
        if len(rows) > LIST_CAP:
            print(f"  ... {len(rows) - LIST_CAP} more in report.json")

    validation = sorted((m, r) for m, r in results.items() if r['class'] == 'validation')
    listing("Every validation refusal that is not a studio version",
            [(m, r) for m, r in validation if not is_version_refusal(r)])
    versions = [(m, r) for m, r in validation if is_version_refusal(r)]
    if versions:
        print(f"\nStudio version refusals ({len(versions)}; a correct refusal, first 5):")
        for m, r in versions[:5]:
            print(f"  {m}  [{r['source']}]  {r['reason']}")
    listing("Every model with class 'crash'",
            sorted((m, r) for m, r in results.items() if r['class'] == 'crash'))
    listing("Every model with class 'other'",
            sorted((m, r) for m, r in results.items() if r['class'] == 'other'))
    if checker.stray:
        print(f"\nError or Warning lines that name none of the models ({len(checker.stray)}):")
        for s in checker.stray[:25]:
            print(f"  {s}")
    if checker.crashes:
        print("\nFailed batches:")
        for c in checker.crashes:
            print(f"  {c['batch']}: {c['failure']}; {len(c['models'])} models; "
                  f"culprit: {', '.join(c.get('culprits', [])) or 'not found (halves passed)'}")
    if checker.first_failures:
        print(f"\nRuns of VBSP that failed on the first attempt ({len(checker.first_failures)}):")
        for f in checker.first_failures[:LIST_CAP]:
            print(f"  {f['batch']} ({f['models']} models): {f['failure']}; "
                  + ('passed on the retry' if f['passed_on_retry']
                     else 'failed on the retry too' if f['passed_on_retry'] is False
                     else 'retry not recorded'))
    if checker.dumps:
        print(f"\nCrash dumps that appeared in VBSP's folder during the run "
              f"({len(checker.dumps)}); left where they are, because the tool cannot tell "
              "which process wrote them:")
        for d in checker.dumps[:10]:
            print(f"  during {d['batch']}"
                  f"{' (that run failed)' if d['run_failed'] else ''}: {d['file']}")
    if baseline is not None:
        print(f"\nAgainst the baseline: {len(baseline['regressions'])} placed model(s) now "
              f"refused, {len(baseline['changes'])} other change(s), "
              f"{len(baseline['only_old'])} only in the baseline, "
              f"{len(baseline['only_new'])} only in this run")
        for r in baseline['regressions'][:LIST_CAP]:
            print(f"  REGRESSION {r['model']}: placed -> {r['now']}: {r['reason']}")
        for r in baseline['changes'][:LIST_CAP]:
            print(f"  changed    {r['model']}: {r['was']} -> {r['now']}")
        if len(baseline['only_old']) or len(baseline['only_new']):
            print("  (models on one side only are not failures; the lists are in report.json)")
    print()
    if why:
        print("RESULT: FAIL")
        for w in why:
            print(f"  - {w}")
    else:
        print("RESULT: clean" + (f" ({summary['version_refusals']} studio version refusals "
                                 "do not count as failures)" if summary['version_refusals'] else ''))


# --- --explain ----------------------------------------------------------------

def _read(name: str, paths: List[Path]) -> Optional[bytes]:
    found = gameinfo_search.find_in(paths, name, strict=False)
    return found.read_bytes() if found else None


def explain(paths: List[Path], where_text: str, model: str) -> int:
    """Print the MDL and VVD numbers VBSP's checks look at, for one model."""
    name = model.replace('\\', '/').lstrip('/').lower()
    if not name.endswith('.mdl'):
        name += '.mdl'
    if not name.startswith('models/'):
        name = 'models/' + name
    where = gameinfo_search.find_in(paths, name, strict=False)
    try:
        mdl = where.read_bytes() if where else None
    except OSError as e:
        print(f"{name}: found but cannot be read: {e}")
        return 2
    if mdl is None:
        print(f"{name}: not found in {where_text}")
        return 2
    print(f"{name}\n  from {where.label}\n  MDL size {len(mdl)} bytes")
    if len(mdl) < 240:
        print("  MDL is shorter than its header")
        return 1
    u = lambda fmt, ofs: struct.unpack_from('<' + fmt, mdl, ofs)
    ident, version, checksum = u('4sii', 0)
    if ident not in (b'IDST', b'IDAG'):
        print(f"  the file starts with {ident!r}, not IDST or IDAG: not a studio model")
        return 1
    # Offsets below are those of studiohdr_t in src/public/studio.h (version 44 to 48):
    # id 0, version 4, checksum 8, name[64] 12, length 76, flags 152, numtextures 204,
    # textureindex 208, numcdtextures 212, cdtextureindex 216, numskinref 220,
    # numskinfamilies 224, skinindex 228, numbodyparts 232, bodypartindex 236,
    # studiohdr2index 400 (studiohdr2_t.sznameindex at 20).
    nm = mdl[12:76].split(b'\0')[0].decode('latin-1')
    length, = u('i', 76)
    flags, = u('i', 152)
    print(f"  id {ident!r} version {version} (VBSP reads 44 to 48) checksum "
          f"0x{checksum & 0xFFFFFFFF:08X} name {nm!r} stored length {length} flags 0x{flags:x}"
          f" ($staticprop {'yes' if flags & STUDIOHDR_FLAGS_STATIC_PROP else 'no'})")
    if not 44 <= version <= 48:
        print("  VBSP refuses this studio version before it reads any table, so none are shown")
        return 1
    if len(mdl) < 408:
        print("  MDL is shorter than a version 44 to 48 header")
        return 1
    numtex, texidx, _ncd, _cdidx, numskinref, numfam, skinidx, numbp, bpidx = u('9i', 204)
    # VBSP's vertex check loads "models/<pszName()>" with the extension .vvd, and pszName()
    # is the studiohdr2 name when it has one, else the name field.
    internal = nm
    h2, = u('i', 400)
    if h2 > 0 and h2 + 24 <= len(mdl):
        sznameindex, = u('i', h2 + 20)
        if sznameindex > 0 and h2 + sznameindex < len(mdl):
            internal = mdl[h2 + sznameindex:].split(b'\0')[0].decode('latin-1')
    print(f"  textures {numtex}, skin references {numskinref}, skin families {numfam}, "
          f"bodyparts {numbp}")
    limit = numskinref if numskinref > 0 else numtex
    print(f"  VBSP bounds a mesh's material by {limit} "
          f"({'skin references' if numskinref > 0 else 'textures'})")
    try:
        row0 = [struct.unpack_from('<h', mdl, skinidx + 2 * i)[0] for i in range(numskinref)]
        print(f"  skin family 0 maps references to textures {row0}")
    except struct.error:
        print("  skin table is outside the file")
    meshes = []
    try:
        for bp in range(numbp):
            bpofs = bpidx + 16 * bp
            nummodels, _base, modelindex = u('3i', bpofs + 4)
            for mi in range(nummodels):
                mofs = bpofs + modelindex + 148 * mi
                nummeshes, meshindex, nverts, vindex = u('4i', mofs + 72)
                for k in range(nummeshes):
                    kofs = mofs + meshindex + 116 * k
                    material, modelback, mverts, voffset = u('4i', kofs)
                    meshes.append((bp, mi, k, material, voffset, mverts, vindex // 48,
                                   kofs + modelback == mofs))
    except struct.error:
        print("  a bodypart, model or mesh table is outside the file")
    for bp, mi, k, material, voffset, mverts, vbase, back in meshes[:40]:
        flag = '' if 0 <= material < limit else '   <-- material outside the limit'
        flag += '' if back else '   <-- does not point back to its model'
        print(f"  bodypart {bp} model {mi} mesh {k}: material {material}, vertex offset "
              f"{voffset}, vertices {mverts}, model vertex base {vbase}{flag}")
    if len(meshes) > 40:
        print(f"  ... {len(meshes) - 40} more meshes")
    vvd_name = internal_vvd_name(mdl)
    beside = name[:-4] + '.vvd'
    if vvd_name != beside:
        print(f"  the MDL's own name is {internal!r}, so VBSP loads {vvd_name}, not the file "
              f"beside the MDL ({beside}); showing the one VBSP loads")
    try:
        vvd = _read(vvd_name, paths)
    except OSError as e:
        print(f"  VVD {vvd_name}: found but cannot be read: {e}")
        return 2
    if vvd is None:
        print(f"  VVD {vvd_name}: missing")
        return 1
    print(f"  VVD {vvd_name}, size {len(vvd)} bytes")
    if len(vvd) < 64:
        print(f"  the VVD is {len(vvd)} bytes, shorter than its 64-byte header")
        return 1
    # vertexFileHeader_t: id 0, version 4, checksum 8, numLODs 12, numLODVertexes[8] 16,
    # numFixups 48, fixupTableStart 52, vertexDataStart 56, tangentDataStart 60.
    # mstudiovertex_t is 48 bytes with the position at 16; vertexFileFixup_t is 12 bytes.
    vid, vver, vsum, nlods = struct.unpack_from('<4siii', vvd, 0)
    if not 1 <= nlods <= 8:
        print(f"  VVD id {vid!r} version {vver}: numLODs is {nlods}, outside 1 to 8, so the "
              "header cannot be trusted")
        return 1
    lodverts = list(struct.unpack_from('<8i', vvd, 16))
    nfix, fixstart, vstart, tstart = struct.unpack_from('<4i', vvd, 48)
    print(f"  VVD id {vid!r} version {vver} checksum 0x{vsum & 0xFFFFFFFF:08X} "
          f"({'matches' if vsum == checksum else 'DIFFERS from'} the MDL) LODs {nlods}")
    need = vstart + lodverts[0] * 48
    print(f"  vertices per LOD {lodverts[:max(nlods, 1)]}, vertex data at {vstart} needs "
          f"{need} of {len(vvd)} bytes"
          f"{'' if need <= len(vvd) else '   <-- runs past the end of the file'}")
    if 0 < vstart and 0 < lodverts[0] and need <= len(vvd):
        pos = [struct.unpack_from('<3f', vvd, vstart + 48 * i + 16) for i in range(lodverts[0])]
        finite = [p for p in pos if all(math.isfinite(c) for c in p)]
        lo = [min(p[a] for p in finite) for a in range(3)] if finite else None
        hi = [max(p[a] for p in finite) for a in range(3)] if finite else None
        print(f"  LOD 0 vertex positions: {len(pos) - len(finite)} not finite; "
              f"bounds {lo} to {hi}"
              f"{'   <-- every position is the same point' if finite and lo == hi else ''}")
    print(f"  fixups {nfix} at {fixstart}")
    fixups = []
    if nfix > 0 and fixstart > 0 and fixstart + 12 * nfix <= len(vvd):
        fixups = [struct.unpack_from('<3i', vvd, fixstart + 12 * i) for i in range(nfix)]
        for lod, src, n in fixups[:12]:
            print(f"    lod {lod}  source vertex {src}  count {n}")
        if len(fixups) > 12:
            print(f"    ... {len(fixups) - 12} more")
        for root in range(max(nlods, 1)):
            total = sum(n for lod, _s, n in fixups if lod >= root)
            print(f"  fixups reaching LOD {root} sum to {total} vertices; the VVD header says "
                  f"{lodverts[root]}{'' if total == lodverts[root] else '   <-- DIFFERENT'}")
        print(f"  VBSP's rule (every fixup with lod >= 0 must sum to the LOD 0 count): "
              f"{sum(n for lod, _s, n in fixups if lod >= 0)} of {lodverts[0]}")
        bad = [f for f in fixups if f[1] < 0 or f[2] < 0 or f[1] + f[2] > lodverts[0]]
        print(f"  fixups reading past the LOD 0 vertices: {len(bad)}")
    return 0


# --- main ----------------------------------------------------------------------

def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="Find which real game models VBSP's prop_static path refuses, and why.")
    ap.add_argument('--game', required=True,
                    help="game directory (the one with gameinfo.txt) VBSP runs with")
    ap.add_argument('--content', action='append', default=[], metavar='DIR',
                    help="check the models in this folder and its *_dir.vpk files instead "
                         "of the game's own (repeatable)")
    ap.add_argument('--vbsp', default=str(DEFAULT_VBSP),
                    help="vbsp_lmo.exe to test (default: next to this script)")
    ap.add_argument('--binroot', help="passed to VBSP as -binroot")
    ap.add_argument('--out', default=str(DEFAULT_OUT), help="where run folders go (default: "
                    "unit_test/unit_test_output/prop_corpus, git-ignored)")
    ap.add_argument('--batch', type=int, default=250, help="models per generated map")
    ap.add_argument('--match', help="only models whose path contains this text")
    ap.add_argument('--limit', type=int, help="only the first N models after --match")
    ap.add_argument('--timeout', type=int, default=600, help="seconds one VBSP run may take")
    ap.add_argument('--keep', action='store_true',
                    help="keep every map, its BSP and both logs (VBSP's own and the captured "
                         "output), not only the map and output of batches with a refusal, "
                         "a crash or an unexpected message")
    ap.add_argument('--baseline', metavar='REPORT',
                    help="an earlier report.json: fail when a model it placed is not placed now")
    ap.add_argument('--explain', metavar='MODEL',
                    help="print the MDL and VVD numbers behind a refusal, then exit")
    args = ap.parse_args(argv)
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(errors='replace')

    def bad(message: str) -> int:
        print(f"error: {message}", file=sys.stderr)
        return 2

    game = Path(args.game).resolve()
    if not (game / 'gameinfo.txt').is_file():
        return bad(f"no gameinfo.txt in {game}")
    contents = [Path(c).resolve() for c in args.content]
    for c in contents:
        if not c.is_dir():
            return bad(f"content folder not found: {c}")
    binroot = Path(args.binroot).resolve() if args.binroot else None
    if binroot and not binroot.is_dir():
        return bad(f"--binroot folder not found: {binroot}")
    base_paths = gameinfo_search.search_paths(game, binroot)    # as VBSP mounts it
    mount_problems: List[dict] = []
    cmounts = content_mounts(contents, mount_problems)
    read_paths = cmounts + base_paths            # content first, as VBSP will see it
    if args.explain:
        return explain(read_paths, str(game), args.explain)
    vbsp = Path(args.vbsp).resolve()
    if not vbsp.is_file():
        return bad(f"VBSP not found: {vbsp} (pass --vbsp)")
    if args.batch < 1:
        return bad("--batch must be at least 1")
    if args.limit is not None and args.limit < 0:
        return bad("--limit must not be negative")
    baseline_models = None
    if args.baseline:
        try:
            baseline_models = json.loads(Path(args.baseline).read_text(encoding='utf-8'))['models']
            if not isinstance(baseline_models, dict):
                raise ValueError("'models' is not a table")
        except (OSError, ValueError, KeyError, TypeError) as e:
            return bad(f"cannot read the baseline report {args.baseline}: {e}")
        for model, record in baseline_models.items():
            if not isinstance(record, dict) or record.get('class') not in CLASSES:
                return bad(f"the baseline report {args.baseline} is not a report of this tool: "
                           f"the entry for {model!r} is {str(record)[:60]!r}, not a record with "
                           f"a class ({', '.join(CLASSES)})")

    sources, skipped = list_models(cmounts if contents else base_paths)
    skipped = mount_problems + skipped
    models = [m for m in sources if not args.match or args.match.lower() in m]
    if args.limit is not None:
        models = models[:args.limit]
    if not models:
        return bad(f"no models in {', '.join(map(str, contents)) if contents else game} match")

    stamp = time.strftime('%Y%m%d_%H%M%S')
    name_source = f"content_{contents[0].parent.name}_{contents[0].name}" if contents \
        else f"{game.parent.name}_{game.name}"
    slug = re.sub(r'[^A-Za-z0-9]+', '_', name_source).strip('_')[:60]
    run_dir = Path(args.out).resolve() / f'{stamp}_{slug}'
    run_dir.mkdir(parents=True, exist_ok=True)
    compile_game = game
    if contents:
        compile_game = run_dir / 'scratch_game'
        unresolved = write_scratch_game(compile_game, game, cmounts + base_paths, binroot)
        if unresolved:
            return bad("the scratch gameinfo.txt does not resolve to these mounts, so the "
                       "listing and VBSP would disagree: " + ', '.join(map(str, unresolved)))
    batches = [models[i:i + args.batch] for i in range(0, len(models), args.batch)]
    label = (f"Content: {', '.join(map(str, contents))}\nGame (engine files): {game}" if contents
             else f"Game: {game}")
    print(f"{len(models)} models from {', '.join(map(str, contents)) if contents else game}\n"
          f"VBSP: {vbsp}\n{len(batches)} map(s) of up to {args.batch}, files in {run_dir}",
          flush=True)
    if skipped:
        print(f"{len(skipped)} mount(s) could not be read (listed at the end)", flush=True)

    checker = Checker(vbsp, compile_game, binroot, run_dir, args.timeout, args.keep, sources,
                      read_paths)
    try:
        failed = checker.canary()
        if failed:
            tail = [l for l in failed['output'].splitlines() if l.strip()][-12:]
            print(f"\nVBSP cannot compile even an empty map with this game directory "
                  f"({failed['failure']}), so no model was checked and no report was written.\n"
                  f"Last lines of VBSP's output:", file=sys.stderr)
            for line in tail:
                print(f"  {line}", file=sys.stderr)
            print(f"The empty map and VBSP's full output are in {run_dir}", file=sys.stderr)
            return 3

        start = time.time()
        for n, batch in enumerate(batches, 1):
            checker.process(f'batch_{n:03d}', batch, progress=f'[{n}/{len(batches)}] ')
        seconds = time.time() - start
    except VbspLaunchError as e:
        return bad(str(e))

    summary = summarise(checker.results)
    baseline = compare_baseline(checker.results, baseline_models) if baseline_models is not None \
        else None
    why = failure_reasons(checker.results, summary, checker, skipped, baseline)
    st = vbsp.stat()
    report = {'game': str(game), 'content': [str(c) for c in contents],
              'compile_game': str(compile_game), 'vbsp': str(vbsp), 'vbsp_bytes': st.st_size,
              'vbsp_modified': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(st.st_mtime)),
              'started': stamp, 'seconds': round(seconds, 1), 'vbsp_runs': checker.runs_made,
              'batch_size': args.batch, 'summary': summary,
              'models': {m: checker.results[m] for m in sorted(checker.results)},
              'skipped_mounts': skipped, 'failed_batches': checker.crashes,
              'unreproduced_failures': checker.unreproduced,
              'first_attempt_failures': checker.first_failures,
              'crash_dumps_seen': checker.dumps,
              'runs': checker.runs, 'unattributed_messages': checker.stray,
              'baseline': {'file': str(Path(args.baseline).resolve()), **baseline}
              if baseline is not None else None,
              'flag_cross_check': dict(checker.flag_checks,
                                       mismatches=[m for m, r in sorted(checker.results.items())
                                                   if r.get('mismatch')]),
              'failed_because': why}
    (run_dir / 'report.json').write_text(json.dumps(report, indent=1), encoding='utf-8')
    print_summary(checker.results, summary, checker, seconds, label, skipped, baseline, why)
    print(f"\nreport: {run_dir / 'report.json'}")
    return 1 if why else 0


if __name__ == '__main__':
    sys.exit(main())
