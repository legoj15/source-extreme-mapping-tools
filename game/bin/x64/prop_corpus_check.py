#!/usr/bin/env python3
"""
prop_corpus_check -- which real game models does VBSP's prop_static path refuse,
and why.

VBSP checks every model before it uses it as a static prop (the model file in
LoadStudioModel, the vertex file in GetCollisionModel; #70). A model that fails
is dropped with a warning, so a check that is stricter than the file formats
guarantee loses props from ordinary maps without anyone noticing. This tool asks
the question on real content: it lists every .mdl the game mounts, compiles
generated maps that hold one prop_static per model, and sorts each model by what
VBSP said about it:

  placed       the model is in the BSP's static prop dictionary
  not_static   refused because it is not a static prop (not compiled with
               $staticprop, or its prop data says it must be dynamic). Expected
               for characters, weapons, physics props and the like.
  validation   refused by VBSP's own file checks: the MDL or VVD is damaged or
               stricter than the format allows. The reason text is kept.
  crash        VBSP died, hung or wrote no BSP on this model alone (a failed batch
               is split in halves until the model is found)
  other        anything else, kept word for word: a message not listed above, or
               a model that was neither placed nor mentioned

Run it after any change to the VBSP model checks:

  python prop_corpus_check.py --game "E:/Steam/steamapps/common/Half-Life 2 Deathmatch/hl2mp" \\
      [--vbsp <path to vbsp_lmo.exe>]

  --match <text>   only models whose path contains the text (quick runs)
  --limit N        only the first N models after --match
  --batch N        models per generated map (default 250)
  --keep           keep every map and log, not only those that had a refusal
  --explain <mdl>  print the numbers behind a refusal (MDL and VVD tables) and exit

It runs one VBSP at a time. Each run writes a folder under --out holding
report.json (class and reason per model) plus the map and log of every batch that
had a validation refusal, a crash or an unexpected message. Exit code: 0 when
nothing was refused by validation and nothing crashed, 1 otherwise, 2 for bad
arguments.
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


# --- listing models -----------------------------------------------------------

def list_models(game_dir: Path) -> "OrderedDict[str, str]":
    """Every .mdl the game mounts, as {lowercase name: mount it comes from}.

    Walks the search path in order and the first mount to hold a name wins,
    which is the copy VBSP's filesystem would open."""
    found: "OrderedDict[str, str]" = OrderedDict()
    for loc in gameinfo_search.search_paths(game_dir):
        label = f"{loc.parent.name}/{loc.name}"
        names: List[str] = []
        if loc.suffix.lower() == '.vpk':
            if not loc.is_file():
                continue
            try:
                names = VPKReader(loc).list_files('models/', 'mdl')
            except (ValueError, OSError, struct.error) as e:
                print(f"  warning: cannot read {loc}: {e}", file=sys.stderr)
                continue
        else:
            root = loc / 'models'
            if root.is_dir():
                for dirpath, _dirs, files in os.walk(root):
                    for f in files:
                        if f.lower().endswith('.mdl'):
                            rel = (Path(dirpath) / f).relative_to(loc)
                            names.append(str(rel).replace('\\', '/').lower())
        for n in sorted(names):
            found.setdefault(n.lower(), label)
    return OrderedDict(sorted(found.items()))


# --- generated maps -----------------------------------------------------------

def write_batch_vmf(path: Path, models: List[str]) -> None:
    """A sealed hollow box holding one prop_static per model, on a grid."""
    m = vmfgen.Map(name=path.stem)
    m.room((-4096, -4096, 0), (4096, 4096, 4096), marker=True)
    cols = max(1, math.ceil(math.sqrt(len(models))))
    step = min(256, 7000 // cols)
    x0 = -(cols - 1) * step // 2
    for i, name in enumerate(models):
        r, c = divmod(i, cols)
        m.entity('prop_static', origin=(x0 + c * step, x0 + r * step, 2048),
                 model=name, solid='6', skin='0')
    VMFWriter().write_file(m.to_kv(), path)


# --- running VBSP and reading what it said ------------------------------------

def is_crash_exit(rc: Optional[int]) -> bool:
    """A signal death (negative) or a Windows crash code such as 0xC0000005."""
    return rc is not None and (rc < 0 or rc >= 0x80000000)


def run_vbsp(vbsp: Path, game: Path, binroot: Optional[Path], vmf: Path,
             timeout: int) -> Tuple[Optional[int], str, float]:
    """Compile one map. Returns (exit code or None on timeout, output, seconds)."""
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
    proc = proc_tree.launch(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            cwd=str(vbsp.parent), env=env)
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
_RE_VALIDATION = re.compile(r'^Error! \w+ model "([^"]+)": (.*?)\.( Deleted\.)?\s*$')
_RE_NOT_MARKED = re.compile(r'^Error! To use model "([^"]+)"')
_RE_DYNAMIC = re.compile(r'^Error! \w+ using model "([^"]+)", which must be used on a dynamic')
_RE_LOAD_FAILED = re.compile(r'^Error loading studio model "([^"]+)"!')
_RE_ANY_MODEL = re.compile(r'models/[^"\s]+?\.mdl')


def parse_log(output: str, batch: set) -> Tuple[Dict[str, list], List[str]]:
    """Events per model, and Error/Warning lines that name no model of the batch.

    An event is (kind, reason, stage, line). kind is one of validation,
    not_marked, dynamic, load_failed, unknown."""
    events: Dict[str, list] = {}
    stray: List[str] = []

    def add(model, kind, reason, stage, line):
        events.setdefault(model.lower(), []).append((kind, reason, stage, line))

    for raw in output.splitlines():
        line = raw.strip()
        if not line:
            continue
        m = _RE_DYNAMIC.match(line)
        if m:
            add(m.group(1), 'dynamic', 'prop data says it must be dynamic', '', line)
            continue
        m = _RE_NOT_MARKED.match(line)
        if m:
            add(m.group(1), 'not_marked', 'not compiled with $staticprop', '', line)
            continue
        m = _RE_VALIDATION.match(line)
        if m:
            add(m.group(1), 'validation', m.group(2), 'vvd' if m.group(3) else 'mdl', line)
            continue
        m = _RE_LOAD_FAILED.match(line)
        if m:
            add(m.group(1), 'load_failed', 'could not load the model', '', line)
            continue
        named = [n for n in _RE_ANY_MODEL.findall(line.lower()) if n in batch]
        if named:
            for n in named:
                add(n, 'unknown', line, '', line)
        elif re.match(r'(?i)^(error|warning)\b', line):
            if line not in stray:
                stray.append(line)
    return events, stray


def read_placed(bsp: Path) -> Optional[set]:
    """Model names in the BSP's static prop dictionary, or None if unreadable."""
    try:
        return {p['model'].lower() for p in sprp_lightmap.parse(bsp.read_bytes())}
    except (SystemExit, OSError, struct.error, IndexError, ValueError):
        return None


def classify(model: str, events: list, placed: set) -> dict:
    """The class and reason of one model, from its log events and the BSP."""
    kinds = {e[0] for e in events}
    for e in events:
        if e[0] == 'validation':
            return {'class': 'validation', 'reason': e[1], 'stage': e[2], 'line': e[3]}
    for e in events:
        if e[0] in ('not_marked', 'dynamic'):
            return {'class': 'not_static', 'reason': e[1], 'line': e[3]}
    unexpected = [e[3] for e in events if e[0] == 'unknown']
    if unexpected:
        return {'class': 'other', 'reason': '; '.join(unexpected), 'lines': unexpected}
    if kinds == {'load_failed'}:
        return {'class': 'other', 'reason': 'VBSP could not load the model and said nothing '
                'else (missing file, or not a studio model)', 'lines': [e[3] for e in events]}
    if model in placed:
        return {'class': 'placed', 'reason': ''}
    return {'class': 'other', 'reason': 'VBSP said nothing about it, yet it is not in the BSP'}


class Checker:
    """One run over a list of models: compile in batches, bisect failures."""

    def __init__(self, vbsp: Path, game: Path, binroot: Optional[Path], run_dir: Path,
                 timeout: int, keep: bool, sources: Dict[str, str]):
        self.vbsp, self.game, self.binroot = vbsp, game, binroot
        self.dir, self.timeout, self.keep, self.sources = run_dir, timeout, keep, sources
        self.results: Dict[str, dict] = {}
        self.runs: List[dict] = []
        self.crashes: List[dict] = []
        self.stray: List[str] = []
        self.runs_made = 0

    # -- one compile ---------------------------------------------------------
    def _compile(self, name: str, models: List[str]) -> dict:
        vmf = self.dir / f'{name}.vmf'
        write_batch_vmf(vmf, models)
        for ext in ('.bsp', '.prt', '.lin', '.log'):
            (self.dir / f'{name}{ext}').unlink(missing_ok=True)
        rc, out, secs = run_vbsp(self.vbsp, self.game, self.binroot, vmf, self.timeout)
        self.runs_made += 1
        bsp = self.dir / f'{name}.bsp'
        placed = read_placed(bsp) if bsp.is_file() else None
        failure = ''
        if rc is None:
            failure = f'timed out after {self.timeout} s'
        elif is_crash_exit(rc):
            failure = f'crashed (exit code 0x{rc & 0xFFFFFFFF:08X})'
        elif placed is None:
            failure = f'exit code {rc} and no usable BSP'
        return {'name': name, 'models': models, 'rc': rc, 'seconds': round(secs, 1),
                'output': out, 'placed': placed or set(), 'failure': failure}

    def _cleanup(self, run: dict, keep_files: bool) -> None:
        name = run['name']
        for ext in ('.bsp', '.prt', '.lin', '.log'):
            (self.dir / f'{name}{ext}').unlink(missing_ok=True)
        if keep_files:
            (self.dir / f'{name}.out.txt').write_text(run['output'], encoding='utf-8')
        else:
            (self.dir / f'{name}.vmf').unlink(missing_ok=True)

    # -- one batch, with bisecting -------------------------------------------
    def process(self, name: str, models: List[str], progress: str = '') -> List[str]:
        """Compile and classify models; returns the models VBSP failed on alone."""
        run = self._compile(name, models)
        retried = False
        if run['failure']:           # a first failure may be a start-up hiccup: try once more
            first = run['failure']
            run = self._compile(name, models)
            retried = True
            if run['failure']:
                run['failure'] = f"{first}; again on retry: {run['failure']}"
        if not run['failure']:
            events, stray = parse_log(run['output'], set(models))
            self.stray.extend(s for s in stray if s not in self.stray)
            counts: Counter = Counter()
            had_validation = had_other = False
            for model in models:
                rec = classify(model, events.get(model, []), run['placed'])
                rec['source'] = self.sources.get(model, '')
                rec['batch'] = name
                self.results[model] = rec
                counts[rec['class']] += 1
                had_validation |= rec['class'] == 'validation'
                had_other |= rec['class'] == 'other'
            note = ' (passed on retry)' if retried else ''
            self.runs.append({'name': name, 'models': len(models), 'seconds': run['seconds'],
                              'exit_code': run['rc'], 'retried': retried,
                              'counts': dict(counts)})
            print(f"  {progress}{name}: {len(models)} models, {run['seconds']:.0f} s{note}: "
                  + ', '.join(f"{CLASS_TITLES[c]} {counts[c]}" for c in CLASSES if counts[c]),
                  flush=True)
            self._cleanup(run, self.keep or had_validation or had_other)
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
        culprits = self.process(name + '_a', models[:mid], progress)
        culprits += self.process(name + '_b', models[mid:], progress)
        crash['culprits'] = culprits
        if not culprits:
            print(f"  {name}: both halves compiled, so the failure did not repeat "
                  f"(see {name}.vmf)", flush=True)
        return culprits


# --- the report -----------------------------------------------------------------

def normalise_reason(reason: str) -> str:
    """The reason with numbers and names blanked, to count alike refusals together."""
    return re.sub(r'\d+', 'N', re.sub(r'"[^"]*"', '"..."', reason))


def summarise(results: Dict[str, dict]) -> dict:
    by_class = Counter(r['class'] for r in results.values())
    reasons: Dict[str, Counter] = {c: Counter() for c in CLASSES}
    by_source: Dict[str, Counter] = {}
    for r in results.values():
        if r['class'] != 'placed':
            reasons[r['class']][normalise_reason(r['reason'])] += 1
        by_source.setdefault(r['source'], Counter())[r['class']] += 1
    return {'counts': {c: by_class.get(c, 0) for c in CLASSES},
            'reasons': {c: dict(reasons[c]) for c in CLASSES if reasons[c]},
            'by_source': {s: dict(c) for s, c in sorted(by_source.items())}}


def print_summary(results: Dict[str, dict], summary: dict, checker: Checker,
                  seconds: float, game: Path) -> None:
    print()
    print(f"Game: {game}")
    print(f"Models checked: {len(results)}   VBSP runs: {checker.runs_made}   "
          f"time: {seconds:.0f} s")
    print()
    for c in CLASSES:
        print(f"  {CLASS_TITLES[c]:<30} {summary['counts'][c]:>6}")
    for c in CLASSES[1:]:
        if summary['reasons'].get(c):
            print(f"\nReasons: {CLASS_TITLES[c]}")
            for reason, n in sorted(summary['reasons'][c].items(), key=lambda kv: -kv[1]):
                print(f"  {n:>6}  {reason}")
    print("\nBy mount:")
    for source, counts in summary['by_source'].items():
        print(f"  {source:<40} " + ', '.join(f"{c} {counts[c]}" for c in CLASSES if counts.get(c)))
    for c in ('validation', 'crash', 'other'):
        rows = sorted((m, r) for m, r in results.items() if r['class'] == c)
        if rows:
            print(f"\nEvery model with class '{c}' ({len(rows)}):")
            for m, r in rows:
                print(f"  {m}  [{r['source']}]\n      {r['reason']}")
    if checker.stray:
        print(f"\nError or Warning lines that name none of the models ({len(checker.stray)}):")
        for s in checker.stray[:25]:
            print(f"  {s}")
    if checker.crashes:
        print("\nFailed batches:")
        for c in checker.crashes:
            print(f"  {c['batch']}: {c['failure']}; {len(c['models'])} models; "
                  f"culprit: {', '.join(c.get('culprits', [])) or 'not found (halves passed)'}")


# --- --explain ----------------------------------------------------------------

def _read(name: str, paths: List[Path]) -> Optional[bytes]:
    found = gameinfo_search.find_in(paths, name, strict=False)
    return found.read_bytes() if found else None


def explain(game: Path, model: str) -> int:
    """Print the MDL and VVD numbers VBSP's checks look at, for one model."""
    name = model.replace('\\', '/').lstrip('/').lower()
    if not name.endswith('.mdl'):
        name += '.mdl'
    if not name.startswith('models/'):
        name = 'models/' + name
    paths = gameinfo_search.search_paths(game)
    where = gameinfo_search.find_in(paths, name, strict=False)
    mdl = where.read_bytes() if where else None
    if mdl is None:
        print(f"{name}: not found in {game}")
        return 2
    print(f"{name}\n  from {where.label}\n  MDL size {len(mdl)} bytes")
    if len(mdl) < 240:
        print("  MDL is shorter than its header")
        return 1
    u = lambda fmt, ofs: struct.unpack_from('<' + fmt, mdl, ofs)
    ident, version, checksum = u('4sii', 0)
    nm = mdl[12:76].split(b'\0')[0].decode('latin-1')
    length, = u('i', 76)
    flags, = u('i', 152)
    numtex, texidx, _ncd, _cdidx, numskinref, numfam, skinidx, numbp, bpidx = u('9i', 204)
    print(f"  id {ident!r} version {version} checksum 0x{checksum & 0xFFFFFFFF:08X} "
          f"name {nm!r} stored length {length} flags 0x{flags:x}"
          f" ($staticprop {'yes' if flags & 0x10 else 'no'})")
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
    vvd = _read(name[:-4] + '.vvd', paths)
    if vvd is None:
        print("  VVD: missing")
        return 0
    print(f"  VVD size {len(vvd)} bytes")
    if len(vvd) < 64:
        print("  VVD is shorter than its header")
        return 0
    vid, vver, vsum, nlods = struct.unpack_from('<4siii', vvd, 0)
    lodverts = list(struct.unpack_from('<8i', vvd, 16))
    nfix, fixstart, vstart, tstart = struct.unpack_from('<4i', vvd, 48)
    print(f"  VVD id {vid!r} version {vver} checksum 0x{vsum & 0xFFFFFFFF:08X} "
          f"({'matches' if vsum == checksum else 'DIFFERS from'} the MDL) LODs {nlods}")
    need = vstart + lodverts[0] * 48
    print(f"  vertices per LOD {lodverts[:max(nlods, 1)]}, vertex data at {vstart} needs "
          f"{need} of {len(vvd)} bytes"
          f"{'' if need <= len(vvd) else '   <-- runs past the end of the file'}")
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
    ap.add_argument('--game', required=True, help="game directory (the one with gameinfo.txt)")
    ap.add_argument('--vbsp', default=str(DEFAULT_VBSP),
                    help="vbsp_lmo.exe to test (default: next to this script)")
    ap.add_argument('--binroot', help="passed to VBSP as -binroot")
    ap.add_argument('--out', default=str(DEFAULT_OUT), help="where run folders go (default: "
                    "unit_test/unit_test_output/prop_corpus, git-ignored)")
    ap.add_argument('--batch', type=int, default=250, help="models per generated map")
    ap.add_argument('--match', help="only models whose path contains this text")
    ap.add_argument('--limit', type=int, help="only the first N models after --match")
    ap.add_argument('--timeout', type=int, default=600, help="seconds one VBSP run may take")
    ap.add_argument('--keep', action='store_true', help="keep every map and log")
    ap.add_argument('--explain', metavar='MODEL',
                    help="print the MDL and VVD numbers behind a refusal, then exit")
    args = ap.parse_args(argv)
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(errors='replace')

    game = Path(args.game).resolve()
    if not (game / 'gameinfo.txt').is_file():
        print(f"error: no gameinfo.txt in {game}", file=sys.stderr)
        return 2
    if args.explain:
        return explain(game, args.explain)
    vbsp = Path(args.vbsp).resolve()
    if not vbsp.is_file():
        print(f"error: VBSP not found: {vbsp} (pass --vbsp)", file=sys.stderr)
        return 2
    if args.batch < 1:
        print("error: --batch must be at least 1", file=sys.stderr)
        return 2

    sources = list_models(game)
    models = [m for m in sources if not args.match or args.match.lower() in m]
    if args.limit is not None:
        models = models[:args.limit]
    if not models:
        print(f"error: no models in {game} match", file=sys.stderr)
        return 2

    stamp = time.strftime('%Y%m%d_%H%M%S')
    slug = re.sub(r'[^A-Za-z0-9]+', '_', f"{game.parent.name}_{game.name}").strip('_')[:60]
    run_dir = Path(args.out).resolve() / f'{stamp}_{slug}'
    run_dir.mkdir(parents=True, exist_ok=True)
    batches = [models[i:i + args.batch] for i in range(0, len(models), args.batch)]
    print(f"{len(models)} models from {game}\nVBSP: {vbsp}\n"
          f"{len(batches)} map(s) of up to {args.batch}, files in {run_dir}", flush=True)

    checker = Checker(vbsp, game, Path(args.binroot).resolve() if args.binroot else None,
                      run_dir, args.timeout, args.keep, sources)
    start = time.time()
    for n, batch in enumerate(batches, 1):
        checker.process(f'batch_{n:03d}', batch, progress=f'[{n}/{len(batches)}] ')
    seconds = time.time() - start

    summary = summarise(checker.results)
    st = vbsp.stat()
    report = {'game': str(game), 'vbsp': str(vbsp), 'vbsp_bytes': st.st_size,
              'vbsp_modified': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(st.st_mtime)),
              'started': stamp, 'seconds': round(seconds, 1), 'vbsp_runs': checker.runs_made,
              'batch_size': args.batch, 'summary': summary,
              'models': {m: checker.results[m] for m in sorted(checker.results)},
              'failed_batches': checker.crashes, 'runs': checker.runs,
              'unattributed_messages': checker.stray}
    (run_dir / 'report.json').write_text(json.dumps(report, indent=1), encoding='utf-8')
    print_summary(checker.results, summary, checker, seconds, game)
    print(f"\nreport: {run_dir / 'report.json'}")
    refused = summary['counts']['validation'] + summary['counts']['crash']
    return 1 if refused else 0


if __name__ == '__main__':
    sys.exit(main())
