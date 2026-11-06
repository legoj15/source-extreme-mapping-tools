"""Resolve files the way the compile tools' filesystem does.

Mirrors the search path the SDK tools build (src/public/filesystem_init.cpp,
src/utils/common/filesystem_tools.cpp FileSystem_Init_Normal):

  1. BASE_PATH: -binroot if given, else the parent of the -game folder
     (FileSystem_SetBasePaths / FileSystem_GetBaseDir).
  2. gameinfo.txt FileSystem/SearchPaths, in file order
     (FileSystem_LoadSearchPaths): |gameinfo_path|, |all_source_engine_paths|
     and |appid_N| tokens, plain paths relative to the base dir, wildcard
     entries (custom/*) expanded to sorted folders and VPKs, and `x.vpk`
     meaning `x_dir.vpk`.
  3. PLATFORM: <game>/../platform (FileSystem_AddSearchPath_Platform).

Only path IDs a path-less lookup (FileExists("lights.rad")) searches are kept:
gamebin, mod, game_write, mod_write, download and executable_path are
"by request only". game_lv / game_hd are skipped (tools mount neither), as are
-insert_search_path, -tempcontent and non-English language folders, which the
optimizer never passes. A VPK whose entries live in numbered archives can be
found but not read here if those archives are missing.
"""
from __future__ import annotations

import fnmatch
import os
import re
import struct
import sys
from pathlib import Path
from typing import List, Optional, Tuple

from vpk_reader import VPKReader

#: Path IDs a lookup without an explicit path ID never searches.
REQUEST_ONLY_IDS = {'executable_path', 'gamebin', 'download', 'mod',
                    'game_write', 'mod_write'}

#: quoted string | unquoted [conditional] (may hold spaces) | brace | bare word
_TOKEN_RE = re.compile(r'"((?:[^"\\]|\\.)*)"|(\[[^\]"]*\])|(\{|\})|([^\s"{}]+)')


# ─── KeyValues (text) ─────────────────────────────────────────────────────────

def _condition_true(cond: str) -> bool:
    """KeyValues EvaluateConditional (src/tier1/KeyValues.cpp ~2218): one
    leading '!' negates; the first of the known names found anywhere in the
    text decides ($WIN32 means any PC); anything else is false. No || / &&."""
    neg = cond.startswith('!')
    up = cond.upper()
    win, mac = sys.platform == 'win32', sys.platform == 'darwin'
    for name, value in (('$DECK', False), ('$X360', False), ('$WIN32', True),
                        ('$WINDOWS', win), ('$OSX', mac),
                        ('$LINUX', not win and not mac), ('$POSIX', not win)):
        if name in up:
            return value != neg
    return False


def parse_keyvalues(text: str) -> List[Tuple[str, object]]:
    """Parse KeyValues text into nested [(key, value-or-list)] in file order.

    Handles quoted and bare tokens, // comments and [$COND] conditionals.
    Duplicate keys are kept (gameinfo SearchPaths relies on that)."""
    tokens: List[Tuple[str, str]] = []
    for line in text.splitlines():
        # strip // comments outside quotes; like KeyValues, a comment only
        # starts between tokens (`hl2//x.vpk` stays one bare token)
        out, i, inq = [], 0, False
        while i < len(line):
            c = line[i]
            if c == '"':
                inq = not inq
            elif (not inq and line.startswith('//', i)
                  and (i == 0 or line[i - 1] in ' \t{}"]')):
                break
            out.append(c)
            i += 1
        for m in _TOKEN_RE.finditer(''.join(out)):
            if m.group(1) is not None:
                tokens.append(('str', m.group(1)))
            elif m.group(2):
                tokens.append(('cond', m.group(2)[1:-1]))
            elif m.group(3):
                tokens.append((m.group(3), m.group(3)))
            else:
                tokens.append(('str', m.group(4)))

    pos = 0

    def block() -> List[Tuple[str, object]]:
        # KeyValues::RecursiveLoadFromBuffer: a section may carry its
        # conditional between key and '{' (`key [$X] { }`); a plain value
        # carries it after the value (`key value [$X]`).
        nonlocal pos
        items: List[Tuple[str, object]] = []
        while pos < len(tokens):
            kind, key = tokens[pos]
            pos += 1
            if kind == '}':
                return items
            if kind != 'str' or pos >= len(tokens):
                continue
            keep = True
            if tokens[pos][0] == 'cond':
                keep = _condition_true(tokens[pos][1])
                pos += 1
                if pos >= len(tokens) or tokens[pos][0] != '{':
                    continue  # conditional between key and value: malformed
            nkind, nval = tokens[pos]
            if nkind == '{':
                pos += 1
                value: object = block()
            elif nkind == 'str':
                pos += 1
                value = nval
                if pos < len(tokens) and tokens[pos][0] == 'cond':
                    keep = _condition_true(tokens[pos][1])
                    pos += 1
            else:
                continue
            if keep:
                items.append((key, value))
        return items

    return block()


def _find(items, key: str):
    for k, v in items:
        if k.lower() == key.lower():
            return v
    return None


def _root_block(kv) -> list:
    """Children of the file's first block. A leading `#base`/`#include` line
    parses as a plain pair, so it is passed over (its contents are not merged:
    gameinfo and the Steam files do not use them in practice)."""
    for _, value in kv:
        if isinstance(value, list):
            return value
    return []


# ─── Steam library lookup (GetAppInstallDirFromSteamVDF) ──────────────────────

def _steam_path() -> Optional[Path]:
    if sys.platform == 'win32':
        try:
            import winreg
            with winreg.OpenKey(winreg.HKEY_CURRENT_USER, r'Software\Valve\Steam') as k:
                return Path(winreg.QueryValueEx(k, 'SteamPath')[0])
        except OSError:
            return None
    home = os.environ.get('HOME')
    if not home:
        return None
    p = Path(home) / '.local/share/Steam'
    return p if p.exists() else Path(home) / '.steam/steam'


def app_install_dir(app_id: int, steam_path: Optional[Path] = None) -> Optional[Path]:
    """<library>/steamapps/common/<installdir> for an installed AppID, or None."""
    steam = steam_path or _steam_path()
    if not steam or not steam.exists():
        return None
    try:
        libs = parse_keyvalues((steam / 'steamapps' / 'libraryfolders.vdf')
                               .read_text(encoding='utf-8', errors='replace'))
    except OSError:
        return None
    root = _root_block(libs)
    for _, lib in root:
        if not isinstance(lib, list):
            continue
        lib_path = _find(lib, 'path')
        if not isinstance(lib_path, str) or not lib_path:
            continue
        lib_path = lib_path.replace('\\\\', '\\')
        manifest = Path(lib_path) / 'steamapps' / f'appmanifest_{app_id}.acf'
        if not manifest.is_file():
            continue
        try:
            kv = parse_keyvalues(manifest.read_text(encoding='utf-8', errors='replace'))
        except OSError:
            continue
        state = _root_block(kv)
        install = _find(state, 'installdir')
        if isinstance(install, str) and install:
            return Path(lib_path) / 'steamapps' / 'common' / install
    return None


# ─── Search path ──────────────────────────────────────────────────────────────

def _dir_vpk(path: Path) -> Path:
    """gameinfo names `x.vpk`; the filesystem opens `x_dir.vpk`."""
    if path.suffix.lower() == '.vpk' and not path.stem.lower().endswith('_dir'):
        return path.with_name(path.stem + '_dir.vpk')
    return path


def _expand_wildcard(pattern: Path) -> List[Path]:
    """FindFirst/FindNext over a `dir/*` entry: folders and VPKs, sorted
    case-insensitively, numbered `_nnn.vpk` packs dropped next to their
    `_dir.vpk`."""
    parent = pattern.parent
    try:
        names = [e.name for e in parent.iterdir()
                 if not e.name.startswith('.')
                 and (e.is_dir() or '.vpk' in e.name.lower())]
    except OSError:
        return []
    names = sorted((n for n in names if fnmatch.fnmatch(n.lower(), pattern.name.lower())),
                   key=str.lower)
    dir_prefixes = [n[:-len('_dir.vpk')].lower() for n in names
                    if n.lower().endswith('_dir.vpk')]
    kept = [n for n in names
            if not any(n.lower().startswith(p) and not n.lower().endswith('_dir.vpk')
                       and n.lower().endswith('.vpk') for p in dir_prefixes)]
    return [parent / n for n in kept]


def search_paths(game_dir: Path, bin_root: Optional[Path] = None) -> List[Path]:
    """Ordered folders and *_dir.vpk files a path-less lookup searches.

    Without a gameinfo.txt (the tools would refuse to run) only the game
    folder itself is returned."""
    game_dir = Path(game_dir).resolve()
    base_dir = Path(bin_root).resolve() if bin_root else game_dir.parent
    try:
        raw = (game_dir / 'gameinfo.txt').read_bytes()
    except OSError:
        return [game_dir]
    # KeyValues accepts UTF-16 LE text with a BOM (KeyValues::LoadFromFile)
    text = (raw.decode('utf-16', errors='replace') if raw.startswith(b'\xff\xfe')
            else raw.decode('utf-8', errors='replace'))
    kv = parse_keyvalues(text)
    gameinfo = _root_block(kv)
    fs = _find(gameinfo, 'FileSystem')
    sp = _find(fs, 'SearchPaths') if isinstance(fs, list) else None
    if not isinstance(sp, list):
        return [game_dir]

    out: List[Path] = [base_dir]
    for path_ids, location in sp:
        if not isinstance(location, str):
            continue
        ids = {p.strip().lower() for p in path_ids.split('+')}
        ids.discard('game_lv')
        ids.discard('game_hd')
        if not ids or ids <= REQUEST_ONLY_IDS:
            continue
        if 'webdavredirector' in location.lower():
            continue
        loc, base = location, base_dir
        low = loc.lower()
        if low.startswith('|appid_'):
            rest = loc[len('|appid_'):]
            num, bar, loc = rest.partition('|')
            if not bar or not num.strip().isdigit():
                continue
            app_dir = app_install_dir(int(num))
            if app_dir is None:
                continue  # VRAD: "not installed. Skipping mount."
            base = app_dir
        elif low.startswith('|gameinfo_path|'):
            loc, base = loc[len('|gameinfo_path|'):], game_dir
        elif low.startswith('|all_source_engine_paths|'):
            loc, base = loc[len('|all_source_engine_paths|'):], game_dir.parent
        full = Path(os.path.normpath(os.path.join(str(base), loc.replace('\\', '/'))))
        if '*' in loc or '?' in loc:
            out.extend(_dir_vpk(p) for p in _expand_wildcard(full))
        else:
            out.append(_dir_vpk(full))

    out.append(Path(os.path.normpath(str(game_dir / '..' / 'platform'))))
    return out


_VPK_CACHE: dict = {}


def _open_vpk(path: Path) -> VPKReader:
    """VPKReader for a dir VPK, reused while the file's size and mtime hold
    (parsing hl2's VPK trees takes most of a second, and the optimizer looks
    lights.rad up several times per run)."""
    st = path.stat()
    key = (str(path).lower(), st.st_size, st.st_mtime_ns)
    reader = _VPK_CACHE.get(key)
    if reader is None:
        reader = VPKReader(path)
        if len(_VPK_CACHE) > 64:
            _VPK_CACHE.clear()
        _VPK_CACHE[key] = reader
    return reader


class FoundFile:
    """A file located on the search path: loose, or an entry inside a VPK."""

    def __init__(self, label: str, loose: Optional[Path] = None,
                 vpk: Optional[Path] = None, entry: str = ''):
        self.label, self.loose, self.vpk, self.entry = label, loose, vpk, entry

    def read_bytes(self) -> bytes:
        """The file's bytes. Raises OSError if they cannot be read."""
        if self.loose is not None:
            return self.loose.read_bytes()
        try:
            data = _open_vpk(self.vpk).read_file(self.entry)
        except (ValueError, OSError, struct.error) as e:
            raise OSError(f"cannot read {self.entry} from {self.vpk}: {e}") from e
        if data is None:
            raise OSError(f"cannot read {self.entry} from {self.vpk}")
        return data

    def __repr__(self):
        return f"FoundFile({self.label!r})"


def find_file(name: str, game_dir: Path, bin_root: Optional[Path] = None,
              strict: bool = True) -> Optional[FoundFile]:
    """First hit for a game-relative file name along search_paths(), or None.

    A VPK this reader cannot parse: with strict (the default) OSError is
    raised, since whether it holds the file is unknown and callers that must
    be exact (the cache key) refuse; otherwise it is skipped and the search
    goes on, as the filesystem skips a pack it cannot mount."""
    rel = name.replace('\\', '/').lstrip('/')
    for loc in search_paths(game_dir, bin_root):
        if loc.suffix.lower() == '.vpk':
            if not loc.is_file():
                continue
            try:
                reader = _open_vpk(loc)
            except (ValueError, OSError, struct.error) as e:
                if not strict:
                    continue
                raise OSError(f"unreadable VPK on the search path: {loc}: {e}") from e
            if reader.has_file(rel):
                return FoundFile(f"{loc}:{rel}", vpk=loc, entry=rel)
        else:
            candidate = loc / rel
            if candidate.is_file():
                return FoundFile(str(candidate), loose=candidate)
    return None
