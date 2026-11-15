#!/usr/bin/env python3
"""
bspwalk_view -- one self-contained HTML page showing why bspwalk marked each BSP face
visible or hidden: a 3D, Hammer-viewport-like debug view you open locally (file://, no
server).  The map data is inline; three.js is loaded from cdnjs, so the page needs the
network the first time the browser opens it.  Think Hammer's leak pointfile, but
explorable: click a face, see its ids, its tier and reason, and the ray that made it
visible, drawn in 3D.

    python -I bspwalk_view.py --bsp d1v.bsp --game <dir> --faces 437,17128,16625 --out view.html

The BSP must be compiled with vbsp_lmo -emitsideids (side id lumps).  --game is always
passed to bspwalk as -game (never VPROJECT).  bspwalk runs in two passes: run 1 traces
the faces you name (-traceface, 1200 faces per bspwalk run), run 2 traces every eye those
faces were seen from (-traceeye, -notierb, 600 eyes per run), which names the eye piece
and walk chain behind each eye.  --reuse DIR reads
an earlier run's walk.json / trace1.json / trace2.json instead of running bspwalk again;
--keep DIR keeps the working files for exactly that.
"""
from __future__ import annotations

import argparse
import contextlib
import datetime
import html
import json
import locale
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterator, List, Optional, Sequence, Set, Tuple

# The tool runs under python -I, so the script's directory is not on sys.path.
_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

from bsp_reader import BSPReader  # noqa: E402
from vmf_parser import KVNode, VMFParser, extract_brushes  # noqa: E402
from bspwalk_runner import find_bspwalk, split_args  # noqa: E402

EYE_BATCH = 600        # -traceeye points per bspwalk run (about 27k characters; see run_passes)
FACE_BATCH = 1200      # -traceface faces per bspwalk run (about 22k characters with 6-digit ids)
THREE_URL ="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"

# Drawing categories, in draw (and legend) order: hidden tiers cool, visible reasons
# warm, the traced-eye categories magenta/violet.  Index order is CATS order.
CATS: List[Tuple[str, str, str]] = [
    ("A",            "tier A - PVS hidden",               "#2e6fdb"),
    ("B",            "tier B - rays hidden",              "#15c2b0"),
    ("fan",          "visible - fan",                     "#ff922b"),
    ("ray",          "visible - ray",                     "#ffd21f"),
    ("mirror",       "visible - mirror",                  "#ff5d8f"),
    ("no_eye",       "visible - no eye",                  "#c8a15a"),
    ("brush_entity", "visible - brush entity",            "#9fdd3c"),
    ("solid_view",   "visible - solid view",              "#ff7043"),
    ("other",        "visible - other (no_tierb, bad_lump)", "#a0a4a8"),
    ("cubemap",      "visible - seen from a cubemap",     "#e24ce0"),
    ("nonnode",      "visible - seen from a non-node eye", "#8f6bff"),
]
CAT = {key: i for i, (key, _, _) in enumerate(CATS)}
# tier+reason of a visible face -> category key; anything else is "other".
_VISIBLE_REASON_CAT = {"fan": "fan", "ray": "ray", "mirror": "mirror", "no_eye": "no_eye",
                       "brush_entity": "brush_entity", "solid_view": "solid_view"}


@dataclass
class FaceRow:
    """One BSP face's payload data, before the category sort."""
    fid: int                      # BSP face index
    vidx: List[int]               # polygon loop, as indices into the merged vertex array
    sideids: List[int]
    cat: int
    tier: str
    reason: str
    disp: bool


def _rewrite_extra(argv: List[str]) -> List[str]:
    """`--extra -flag` -> `--extra=-flag`: argparse would otherwise take the value (a
    bspwalk option, so it starts with -) for an option of its own."""
    out: List[str] = []
    i = 0
    while i < len(argv):
        if argv[i] == "--extra" and i + 1 < len(argv):
            out.append("--extra=" + argv[i + 1])
            i += 2
            continue
        out.append(argv[i])
        i += 1
    return out


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    argv = list(sys.argv[1:] if argv is None else argv)
    p = argparse.ArgumentParser(
        description="Build a self-contained 3D HTML debug view of bspwalk's face tiers.")
    p.add_argument("--bsp", required=True, help="compiled BSP (needs vbsp_lmo -emitsideids)")
    p.add_argument("--game", required=True,
                   help="game directory, always passed to bspwalk as -game (the folder "
                        "holding gameinfo.txt)")
    p.add_argument("--vmf", default=None,
                   help="VMF for brush ids (default: the BSP's path with .vmf; if missing, "
                        "brush ids are shown as unknown)")
    p.add_argument("--profile", default="hl2", help="bspwalk movement profile (-profile)")
    p.add_argument("--faces", default="", help="comma-separated BSP face ids to trace and "
                                               "pre-select, e.g. 437,17128")
    p.add_argument("--sides", default="", help="comma-separated VMF side ids; every BSP face "
                                              "whose side id list holds one is traced and "
                                              "pre-selected")
    p.add_argument("--out", default=None, help="output HTML file (default: <map>_bspwalk.html)")
    p.add_argument("--bspwalk", default=None, help="bspwalk.exe (default: beside this script)")
    p.add_argument("--extra", default="",
                   help='extra bspwalk arguments, e.g. "-exploits"; split like a shell')
    p.add_argument("--keep", default=None,
                   help="keep bspwalk's working files (walk.json, trace*.json) in this "
                        "directory (created), for a later --reuse")
    p.add_argument("--reuse", default=None,
                   help="read walk.json / trace1.json [/trace2.json] from this directory "
                        "instead of running bspwalk")
    return p.parse_args(_rewrite_extra(argv))


def _int_list(text: str, what: str) -> List[int]:
    """A comma-separated list of non-negative ints ("437,17128"); empty parts ignored."""
    vals: List[int] = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        if not part.isdigit():
            raise SystemExit(f"error: --{what} wants comma-separated ids >= 0, got {part!r}")
        vals.append(int(part))
    return vals


@contextlib.contextmanager
def work_dir(keep: Optional[str], reuse: Optional[str]) -> Iterator[Path]:
    """The directory bspwalk writes its JSONs to: --reuse (read only), --keep (created,
    left in place) or a temp dir removed at the end."""
    if reuse:
        yield Path(reuse).resolve()
        return
    if keep:
        p = Path(keep).resolve()   # bspwalk runs in its own folder: give it absolute paths
        p.mkdir(parents=True, exist_ok=True)
        yield p
        return
    tmp = tempfile.mkdtemp(prefix="bspwalk_view_")
    try:
        yield Path(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def _decode(data: bytes) -> str:
    enc = locale.getpreferredencoding(False) or "utf-8"
    return data.decode(enc, errors="replace")


def run_logged(cmd: Sequence[str], cwd: Path) -> None:
    """Print the command, run it quietly; on a non-zero exit print its last 40 lines and
    stop the tool.  Nothing is streamed while it runs (a run takes about a minute on a
    big map)."""
    print(" ".join(str(c) for c in cmd), flush=True)
    try:
        proc = subprocess.run([str(c) for c in cmd], cwd=str(cwd), stdin=subprocess.DEVNULL,
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    except OSError as e:  # a missing exe, or a command line past Windows' 32767 characters
        n = len(subprocess.list2cmdline([str(c) for c in cmd]))
        sys.exit(f"could not start bspwalk: {e}" + (f" (a {n}-character command line)" if n > 32000 else ""))
    if proc.returncode != 0:
        lines = _decode(proc.stdout).strip().splitlines()
        print(f"bspwalk exited {proc.returncode}; last {min(40, len(lines))} lines of output:",
              file=sys.stderr, flush=True)
        for line in lines[-40:]:
            print(f"  {line}", file=sys.stderr, flush=True)
        sys.exit(1)


def _has_profile_arg(extra: Sequence[str]) -> bool:
    return any(str(a).lower() == "-profile" for a in extra)


def _load_json(path: Path, what: str) -> dict:
    try:
        obj = json.loads(path.read_bytes().decode("utf-8", errors="replace"))
    except OSError as e:
        raise SystemExit(f"error: cannot read the {what} {path}: {e}") from None
    except ValueError as e:
        raise SystemExit(f"error: the {what} {path} is not valid JSON: {e}") from None
    if not isinstance(obj, dict):
        raise SystemExit(f"error: the {what} {path} is not a JSON object")
    return obj


def collect_eyes(trace_faces: List[dict]) -> List[Tuple[str, str, str]]:
    """Every distinct eye position (witness eye_pos, then fan_rays[].eye_pos), rounded to
    4 decimals and deduped, in first-seen order."""
    seen: Set[Tuple[str, str, str]] = set()
    out: List[Tuple[str, str, str]] = []
    for t in trace_faces:
        points = [(t.get("witness") or {}).get("eye_pos")]
        points += [r.get("eye_pos") for r in t.get("fan_rays") or []]
        for p in points:
            if not isinstance(p, list) or len(p) != 3:
                continue
            try:
                key = tuple(f"{float(c):.4f}" for c in p)
            except (TypeError, ValueError):
                continue
            if key not in seen:
                seen.add(key)
                out.append(key)
    return out


def run_passes(args: argparse.Namespace, exe: Path, workdir: Path,
               selected: List[int]) -> Tuple[dict, dict, Optional[dict]]:
    """Run bspwalk for the face traces (batched) and, when they name any eye, for the eye
    traces (batched); return (walk.json, trace1.json, trace2.json or None)."""
    bsp = Path(args.bsp).resolve()
    game = str(Path(args.game).resolve())
    extra = split_args(args.extra)
    walk_path = workdir / "walk.json"
    trace1_path = workdir / "trace1.json"
    # An earlier run's JSONs in a --keep folder must not survive into this one: run 2 is
    # skipped when no traced face names an eye, and a later --reuse would read the old
    # trace2.json. Done here, after the exe and game checks, so a run that stops on those
    # leaves the folder as it was.
    for pat in ("walk.json", "trace1*.json", "trace2*.json"):
        for old in workdir.glob(pat):
            try:
                old.unlink()
            except OSError as e:
                raise SystemExit(f"error: cannot remove {old} from an earlier run: {e}")

    # Run 1 in batches of FACE_BATCH faces (about 18 characters each; each batch is a whole
    # bspwalk run), the first one also writing -out. Every option goes before the trace
    # lists: bspwalk reads -traceface / -traceeye from argv, but tier0's CommandLine()
    # keeps only the first few hundred parameters, and an -out, -tracejson or --extra
    # option past that is dropped without a word.
    trace1: Optional[dict] = None
    for b in range(0, max(len(selected), 1), FACE_BATCH):
        part = trace1_path if len(selected) <= FACE_BATCH else workdir / f"trace1_{b // FACE_BATCH}.json"
        cmd = [str(exe), str(bsp), "-game", game]
        if args.profile and not _has_profile_arg(extra):
            cmd += ["-profile", args.profile]
        if b == 0:
            cmd += ["-out", str(walk_path)]
        cmd += ["-tracejson", str(part)] + extra
        for f in selected[b:b + FACE_BATCH]:
            cmd += ["-traceface", str(f)]
        run_logged(cmd, exe.parent)
        got = _load_json(part, "bspwalk trace JSON (-tracejson)")
        if trace1 is None:
            trace1 = got
        else:
            trace1["faces"] = (trace1.get("faces") or []) + (got.get("faces") or [])
    if len(selected) > FACE_BATCH:
        trace1_path.write_text(json.dumps(trace1), encoding="utf-8")

    walk = _load_json(walk_path, "bspwalk JSON (-out)")
    trace2: Optional[dict] = None

    eyes = collect_eyes(trace1.get("faces") or [])
    # Run 2 in batches of EYE_BATCH eyes: about 45 characters each, and Windows refuses
    # a command line past 32767. The batches' eyes and node chains are merged into one
    # trace2.json, the shape a single run writes.
    for b in range(0, len(eyes), EYE_BATCH):
        part = workdir / f"trace2_{b // EYE_BATCH}.json"
        cmd = [str(exe), str(bsp), "-game", game]
        if args.profile and not _has_profile_arg(extra):
            cmd += ["-profile", args.profile]
        cmd += ["-notierb", "-tracejson", str(part)] + extra
        for x, y, z in eyes[b:b + EYE_BATCH]:
            cmd += ["-traceeye", x, y, z]
        run_logged(cmd, exe.parent)
        got = _load_json(part, "bspwalk eye-trace JSON")
        if trace2 is None:
            trace2 = {"bsp": got.get("bsp"), "faces": [], "eyes": [], "nodes": []}
        trace2["eyes"] += got.get("eyes") or []
        have = {n.get("node") for n in trace2["nodes"]}
        trace2["nodes"] += [n for n in got.get("nodes") or [] if n.get("node") not in have]
    if trace2 is not None:
        (workdir / "trace2.json").write_text(json.dumps(trace2), encoding="utf-8")
    return walk, trace1, trace2


# ─── BSP geometry and selection ──────────────────────────────────────────────

def read_side_ids(reader: BSPReader, walk_faces: Dict[int, dict], nfaces: int) -> List[List[int]]:
    """Per-face VMF side ids from the side id lumps. Without them (no -emitsideids) the
    walk.json sideids are read, which come from the same lumps and so are empty too:
    every face then shows no side ids."""
    lumps = reader.read_face_side_ids()
    if lumps is not None:
        return [lumps[i] if i < len(lumps) else [] for i in range(nfaces)]
    return [list(walk_faces.get(i, {}).get("sideids") or []) for i in range(nfaces)]


def face_category(tier: Optional[str], reason: Optional[str],
                  witness: Optional[dict]) -> int:
    """The drawing category of one face.  A traced face with a witness from a cubemap (or
    any non-node eye) gets its own category so it can be picked out in the legend."""
    if witness:
        source = witness.get("source")
        if source == "cubemap":
            return CAT["cubemap"]
        if source != "node":
            return CAT["nonnode"]
    if tier == "A":
        return CAT["A"]
    if tier == "B":
        return CAT["B"]
    if tier == "visible":
        return CAT[_VISIBLE_REASON_CAT.get(str(reason), "other")]
    return CAT["other"]


def build_rows(reader: BSPReader, faces: List, walk_faces: Dict[int, dict],
               trace_faces: Dict[int, dict]) -> Tuple[List[FaceRow], List[float], Set[int]]:
    """One FaceRow per BSP face (polygon loops as indices into the merged, deduped vertex
    array), plus that flat vertex array (rounded to 0.01) and the set of side ids used."""
    vertexes = reader.read_vertexes()
    edges = reader.read_edges()
    surfedges = reader.read_surfedges()
    side_ids = read_side_ids(reader, walk_faces, len(faces))

    flat_verts: List[float] = []
    vindex: Dict[Tuple[float, float, float], int] = {}
    rows: List[FaceRow] = []
    used_sides: Set[int] = set()

    for fi, face in enumerate(faces):
        poly = reader.get_face_vertices(face, vertexes, edges, surfedges)
        vidx: List[int] = []
        for p in poly:
            key = (p[0], p[1], p[2])
            j = vindex.get(key)
            if j is None:
                j = len(flat_verts) // 3
                flat_verts.extend((round(p[0], 2), round(p[1], 2), round(p[2], 2)))
                vindex[key] = j
            vidx.append(j)
        entry = walk_faces.get(fi) or {}
        trace = trace_faces.get(fi) or {}
        sids = side_ids[fi] if fi < len(side_ids) else []
        used_sides.update(sids)
        rows.append(FaceRow(
            fid=fi, vidx=vidx, sideids=sids,
            cat=face_category(entry.get("tier"), entry.get("reason"), trace.get("witness")),
            tier=str(entry.get("tier") or "?"),
            reason=str(entry.get("reason") or "?"),
            disp=face.dispinfo >= 0))
    return rows, flat_verts, used_sides


def selected_face_ids(wanted_faces: List[int], wanted_sides: List[int],
                      side_ids: List[List[int]], nfaces: int,
                      warn_missing: bool = True) -> List[int]:
    """--faces plus every face holding one of the --sides ids, in CLI order, deduped."""
    out: List[int] = []
    have: Set[int] = set()

    def add(fid: int) -> None:
        if fid not in have:
            have.add(fid)
            out.append(fid)

    for f in wanted_faces:
        if f >= nfaces:
            raise SystemExit(f"error: face {f} is past the BSP's {nfaces} faces")
        add(f)
    faces_of: Dict[int, List[int]] = {}   # side id -> faces, built once
    if wanted_sides:
        for fi, sids in enumerate(side_ids[:nfaces]):
            for s in sids:
                faces_of.setdefault(s, []).append(fi)
    for sid in wanted_sides:
        hits = faces_of.get(sid, [])
        if not hits and warn_missing:
            print(f"  note: VMF side id {sid} is on no BSP face "
                  f"(the side id lumps list only faces VBSP emitted ids for)",
                  file=sys.stderr)
        for fi in hits:
            add(fi)
    return out


# ─── VMF: side id -> brush id / entity classname ────────────────────────────

def load_side_brushes(vmf_path: Path, wanted: Set[int]) -> Tuple[List[List[object]], List[List[object]]]:
    """([sideId, brush index], ...) pairs and the brush table ([[brushId, classname], ...];
    classname "" = world brush) for the side ids the BSP names."""
    root = VMFParser().parse_file(vmf_path)
    side2brush: Dict[int, int] = {}
    brushes: List[List[object]] = []
    for top in root.children:
        if not isinstance(top, KVNode) or top.name not in ("world", "entity"):
            continue
        classname = "" if top.name == "world" else (top.get_property("classname") or "?")
        for brush in extract_brushes(top):
            bidx = len(brushes)
            brushes.append([brush.id, classname])
            for side in brush.sides:
                if side.id in wanted and side.id not in side2brush:
                    side2brush[side.id] = bidx
    pairs = [[sid, bidx] for sid, bidx in sorted(side2brush.items())]
    return pairs, brushes


# ─── trace data: eyes, walk chains ───────────────────────────────────────────

def pick_eyes_nodes(primary: Optional[dict], secondary: Optional[dict]
                    ) -> Tuple[List[dict], List[dict]]:
    """The eyes/nodes lists to embed: the eye-trace file's (run 2) when it has any, else
    whatever the face-trace file (run 1) carries."""
    for source in (primary, secondary):
        if not isinstance(source, dict):
            continue
        eyes = [e for e in source.get("eyes") or [] if isinstance(e, dict)]
        nodes = [n for n in source.get("nodes") or [] if isinstance(n, dict)]
        if eyes or nodes:
            return eyes, nodes
    return [], []


def match_eye(eyes: List[dict], pos) -> int:
    """Index of the eye whose point is within 0.01u of pos on every axis, else -1."""
    if not isinstance(pos, list) or len(pos) != 3:
        return -1
    for i, eye in enumerate(eyes):
        p = eye.get("point")
        if isinstance(p, list) and len(p) == 3:
            try:
                if max(abs(float(p[k]) - float(pos[k])) for k in range(3)) <= 0.01:
                    return i
            except (TypeError, ValueError):
                continue
    return -1


def build_traces(trace_faces: List[dict], eyes: List[dict]) -> List[dict]:
    """Payload entries for the traced faces: the log, witness and fan rays as bspwalk
    wrote them, plus the eyes[] each one was seen from."""
    out: List[dict] = []
    for t in trace_faces:
        if not isinstance(t, dict):
            continue
        entry = {
            "face": t.get("face"),
            "log": t.get("log") or "",
            "witness": t.get("witness"),
            "fanRays": t.get("fan_rays") or [],
            "eyes": [],
        }
        matched: List[int] = []
        w = t.get("witness") or {}
        i = match_eye(eyes, w.get("eye_pos"))
        if i >= 0:
            matched.append(i)
        for ray in t.get("fan_rays") or []:
            i = match_eye(eyes, (ray or {}).get("eye_pos"))
            if i >= 0 and i not in matched:
                matched.append(i)
        entry["eyes"] = matched
        out.append(entry)
    return out


def build_payload_eyes(eyes: List[dict], nodes: List[dict]) -> List[dict]:
    """Eyes with their pieces and, when a piece names a walk node, that node's chain."""
    node_index = {int(n.get("node")): i for i, n in enumerate(nodes)
                  if isinstance(n.get("node"), int)}
    out: List[dict] = []
    for eye in eyes:
        chain = -1
        for piece in eye.get("pieces") or []:
            n = piece.get("node") if isinstance(piece, dict) else None
            if isinstance(n, int) and n >= 0 and n in node_index:
                chain = node_index[n]
                break
        out.append({"point": eye.get("point") or [], "pieces": eye.get("pieces") or [],
                    "chain": chain})
    return out


# ─── payload and page ────────────────────────────────────────────────────────

def build_payload(args: argparse.Namespace, rows: List[FaceRow], flat_verts: List[float],
                  traces: List[dict], eyes_payload: List[dict], nodes: List[dict],
                  side_brush_pairs: List[List[int]], brushes: List[List[object]],
                  selected: List[int]) -> dict:
    """The whole page's data, compact: one flat vertex array, per-face index ranges,
    category / tier / side-id tables, faces sorted by category so each category is one
    contiguous run of triangles."""
    rows = sorted(rows, key=lambda r: (r.cat, r.fid))
    by_id = {r.fid: i for i, r in enumerate(rows)}

    fvert: List[int] = []
    franges: List[int] = []
    fsids: List[int] = []
    fsidr: List[int] = []
    fids: List[int] = []
    fcat: List[int] = []
    ftr: List[int] = []
    fdisp: List[int] = []
    tkeys: List[str] = []
    tkey_index: Dict[Tuple[str, str], int] = {}

    cat_ranges: List[List[int]] = [[0, 0] for _ in CATS]
    tri = 0
    cur_cat = -1
    for r in rows:
        n = len(r.vidx)
        franges += [len(fvert), n]
        fvert += r.vidx
        fsidr += [len(fsids), len(r.sideids)]
        fsids += r.sideids
        fids.append(r.fid)
        fcat.append(r.cat)
        key = (r.tier, r.reason)
        if key not in tkey_index:
            tkey_index[key] = len(tkeys)
            tkeys.append(f"{r.tier}|{r.reason}")
        ftr.append(tkey_index[key])
        fdisp.append(1 if r.disp else 0)
        tris = max(0, n - 2)
        if r.cat != cur_cat:
            cat_ranges[r.cat][0] = tri
            cur_cat = r.cat
        cat_ranges[r.cat][1] += tris
        tri += tris

    bsp_path = Path(args.bsp)
    made = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    try:
        bsp_made = datetime.datetime.fromtimestamp(bsp_path.stat().st_mtime).strftime(
            "%Y-%m-%d %H:%M:%S")
    except OSError:
        bsp_made = "?"
    witness_cmd = witness_command(args)

    return {
        "map": bsp_path.stem,
        "bsp": str(bsp_path),
        "bspMade": bsp_made,
        "made": made,
        "cats": [{"key": k, "label": lab, "color": col} for k, lab, col in CATS],
        "catRanges": cat_ranges,
        "tkeys": tkeys,
        "verts": flat_verts,
        "fvert": fvert,
        "franges": franges,
        "fids": fids,
        "fcat": fcat,
        "ftr": ftr,
        "fdisp": fdisp,
        "fsids": fsids,
        "fsidr": fsidr,
        "s2b": side_brush_pairs,
        "brushes": brushes,
        "traces": traces,
        "eyes": eyes_payload,
        "nodes": nodes,
        "pre": [by_id[f] for f in selected if f in by_id],   # payload face indices
        "preIds": [f for f in selected if f in by_id],       # the BSP face ids, same order
        "cmd": witness_cmd,
    }


def witness_command(args: argparse.Namespace) -> str:
    """The exact command that traces one face's witness; {F} is the face id."""
    bsp = Path(args.bsp)
    out = Path(args.out) if args.out else Path(bsp.stem + "_bspwalk.html")
    parts = ["python", "-I", "bspwalk_view.py", "--bsp", str(bsp), "--game", str(args.game)]
    if args.vmf:
        parts += ["--vmf", str(args.vmf)]
    if args.profile != "hl2":
        parts += ["--profile", args.profile]
    if args.bspwalk:
        parts += ["--bspwalk", str(args.bspwalk)]
    if args.extra:
        parts += ["--extra", args.extra]
    parts += ["--faces", "{F}", "--out", str(out)]
    return subprocess.list2cmdline(parts)


def render_page(payload: dict) -> str:
    # No '<' in the data block at all ("</script" or "<!--" in map text would end or
    # derail it); the JSON escape for it is the same character to JSON.parse.
    data = json.dumps(payload, separators=(",", ":")).replace("<", "\\u003c")
    title = html.escape(f"bspwalk view – {payload['map']}")
    # Fill the template's own placeholders first and insert the payload last, so map text
    # that happens to spell a placeholder is never substituted.
    head, tail = PAGE_TEMPLATE.split("__PAYLOAD__", 1)

    def fill(part: str) -> str:
        return part.replace("__THREE_URL__", THREE_URL).replace("__TITLE__", title)
    return fill(head) + data + fill(tail)


def main(argv: Optional[Sequence[str]] = None) -> None:
    args = parse_args(argv)
    bsp_path = Path(args.bsp)
    if not bsp_path.is_file():
        raise SystemExit(f"error: BSP not found: {bsp_path}")
    vmf_path = Path(args.vmf) if args.vmf else bsp_path.with_suffix(".vmf")

    # Geometry and the side-id lumps are needed before the runs: they decide which
    # faces --sides selects and get -traceface.
    reader = BSPReader(bsp_path)
    reader.read()
    faces = reader.read_faces()
    nfaces = len(faces)
    wanted_faces = _int_list(args.faces, "faces")
    wanted_sides = _int_list(args.sides, "sides")
    if wanted_sides and reader.read_face_side_ids() is None:
        raise SystemExit("error: --sides needs the BSP's side id lumps; compile it with "
                         "vbsp_lmo -emitsideids")
    selected = selected_face_ids(wanted_faces, wanted_sides,
                                 read_side_ids(reader, {}, nfaces), nfaces)

    walk = trace1 = trace2 = None
    with work_dir(args.keep, args.reuse) as workdir:
        if args.reuse:
            walk = _load_json(workdir / "walk.json", "bspwalk JSON (-out)")
            trace1 = _load_json(workdir / "trace1.json", "bspwalk trace JSON (-tracejson)")
            if (workdir / "trace2.json").is_file():
                trace2 = _load_json(workdir / "trace2.json", "bspwalk eye-trace JSON")
        else:
            try:
                exe = find_bspwalk(args.bspwalk)
            except Exception as e:            # BspwalkError, without importing its type
                raise SystemExit(f"error: {e}") from None
            if exe is None:
                raise SystemExit("error: bspwalk.exe not found (pass --bspwalk)")
            if not Path(args.game).is_dir():
                raise SystemExit(f"error: game directory not found: {args.game}")
            walk, trace1, trace2 = run_passes(args, exe, workdir, selected)

    walk_faces: Dict[int, dict] = {}
    for f in walk.get("faces") or []:
        if isinstance(f, dict) and isinstance(f.get("face"), int):
            walk_faces[f["face"]] = f
    trace_face_list = [t for t in trace1.get("faces") or [] if isinstance(t, dict)]

    rows, flat_verts, used_sides = build_rows(reader, faces, walk_faces,
                                              {t.get("face"): t for t in trace_face_list})
    if reader.read_face_side_ids() is None:
        print("  note: the BSP has no side id lumps (compile with vbsp_lmo -emitsideids); "
              "side and brush ids are unknown", file=sys.stderr)

    side_brush_pairs: List[List[int]] = []
    brushes: List[List[object]] = []
    if vmf_path.is_file():
        try:
            side_brush_pairs, brushes = load_side_brushes(vmf_path, used_sides)
        except Exception as e:
            print(f"  note: cannot read the VMF {vmf_path} ({e}); brush ids are unknown",
                  file=sys.stderr)
    elif args.vmf:
        raise SystemExit(f"error: --vmf not found: {vmf_path}")

    eyes, nodes = pick_eyes_nodes(trace2, trace1)
    eyes_payload = build_payload_eyes(eyes, nodes)
    traces = build_traces(trace_face_list, eyes)

    payload = build_payload(args, rows, flat_verts, traces, eyes_payload, nodes,
                            side_brush_pairs, brushes, selected)
    out_path = Path(args.out) if args.out else Path(bsp_path.stem + "_bspwalk.html")
    out_path.write_text(render_page(payload), encoding="utf-8")
    traced = len(payload["traces"])
    print(f"wrote {out_path} ({len(rows)} faces, "
          f"{sum(max(0, n - 2) for n in payload['franges'][1::2])} triangles, "
          f"{traced} traced)")


# ─── the page ────────────────────────────────────────────────────────────────
# Plain string, placeholders __TITLE__ / __PAYLOAD__ replaced in render_page().

PAGE_TEMPLATE = r"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>__TITLE__</title>
<style>
  html, body { margin: 0; padding: 0; height: 100%; background: #1b1d20;
               color: #d8dade; font: 13px/1.45 "Segoe UI", sans-serif; overflow: hidden; }
  #app { position: absolute; inset: 0; }
  #app canvas { display: block; }
  #panel { position: absolute; top: 0; left: 0; bottom: 0; width: 420px;
           background: #222529; border-right: 1px solid #3a3e44; overflow-y: auto;
           padding: 10px 12px 30px 12px; box-sizing: border-box; z-index: 10; }
  #panel h1 { font-size: 15px; margin: 0 0 4px 0; }
  #panel .meta { color: #9aa0a8; font-size: 11px; margin-bottom: 8px;
                 word-break: break-all; }
  #panel .section { margin-top: 12px; border-top: 1px solid #34383e; padding-top: 8px; }
  #panel .section > b { color: #b7bcc4; font-size: 11px; text-transform: uppercase;
                        letter-spacing: 0.06em; }
  #search { width: 66%; box-sizing: border-box; background: #191b1e; color: #d8dade;
            border: 1px solid #3a3e44; padding: 4px 6px; }
  #searchgo { background: #2d3137; color: #d8dade; border: 1px solid #3a3e44;
              padding: 4px 10px; cursor: pointer; }
  #searchhint { color: #8a9098; font-size: 11px; margin-top: 3px; }
  .opts label { display: block; }
  .legend-row { display: flex; align-items: center; gap: 6px; padding: 1px 0; }
  .legend-row .swatch { width: 12px; height: 12px; border: 1px solid #000;
                        flex: none; }
  .legend-row .cnt { color: #9aa0a8; margin-left: auto; }
  .legend-note { color: #8a9098; font-size: 11px; margin-top: 4px; }
  .facebtn { display: inline-block; margin: 2px 4px 2px 0; padding: 2px 8px;
             background: #2d3137; border: 1px solid #454a51; cursor: pointer; }
  .facebtn.active { border-color: #fff; }
  table.kv { border-collapse: collapse; margin: 4px 0; }
  table.kv td { padding: 1px 10px 1px 0; vertical-align: top; }
  table.kv td:first-child { color: #9aa0a8; white-space: nowrap; }
  table.rows { border-collapse: collapse; margin: 4px 0; font-size: 12px; }
  table.rows th { text-align: left; color: #9aa0a8; font-weight: normal;
                  border-bottom: 1px solid #34383e; padding: 0 10px 1px 0; }
  table.rows td { padding: 0 10px 1px 0; }
  pre.log { background: #191b1e; border: 1px solid #34383e; padding: 6px; height: 200px;
            overflow: auto; font: 11px/1.4 Consolas, monospace; white-space: pre;
            margin: 4px 0; }
  code.cmd { display: block; background: #191b1e; border: 1px solid #34383e;
             padding: 6px; font: 11px Consolas, monospace; word-break: break-all;
             white-space: pre-wrap; }
  .warn { color: #ffb454; }
  #help { position: absolute; left: 432px; bottom: 8px; z-index: 10; color: #9aa0a8;
          background: rgba(27,29,32,0.7); padding: 3px 10px; font-size: 11px;
          border-radius: 3px; }
</style>
</head>
<body>
<div id="app"></div>
<div id="panel">
  <h1>bspwalk view</h1>
  <div class="meta" id="meta"></div>
  <div>
    <input id="search" placeholder="face id, s&lt;sideid&gt; or b&lt;brushid&gt;"
           autocomplete="off">
    <button id="searchgo">go</button>
    <div id="searchhint">e.g. 437, s113898, b2142</div>
  </div>
  <div class="section opts">
    <label><input type="checkbox" id="xray"> X-ray (world 25%, overlays on top)</label>
    <label><input type="checkbox" id="hideHidden"> hide hidden tiers (A + B)</label>
    <label><input type="checkbox" id="allWit"> show all witnesses (not just selected)</label>
  </div>
  <div class="section" id="preSection"><b>traced / selected faces</b><div id="preList"></div></div>
  <div class="section"><b>categories</b><div id="legend"></div>
    <div class="legend-note">displacement faces are drawn as their base polygon</div>
  </div>
  <div class="section" id="detail"><b>face</b><div id="detailBody">click a face</div></div>
</div>
<div id="help">RMB drag / Z: look &middot; W A S D: move &middot; Q/E or Space/Ctrl: up/down
&middot; Shift: fast &middot; wheel: dolly &middot; MMB drag: pan &middot; Alt+LMB drag:
orbit &middot; F: frame &middot; click: pick a face</div>
<script type="application/json" id="bspwalk-data">__PAYLOAD__</script>
<script src="__THREE_URL__"></script>
<script>
(function () {
'use strict';
var D = JSON.parse(document.getElementById('bspwalk-data').textContent);

// ── scene ────────────────────────────────────────────────────────────────
var container = document.getElementById('app');
var renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
container.appendChild(renderer.domElement);
var canvas = renderer.domElement;
var scene = new THREE.Scene();
scene.background = new THREE.Color(0x232529);
var camera = new THREE.PerspectiveCamera(65, 1, 1, 80000);
camera.up.set(0, 0, 1);                      // Source is Z-up
scene.add(camera);

// ── world mesh: one merged BufferGeometry, vertex colours ────────────────
var nf = D.fids.length;
var catVisible = D.cats.map(function () { return true; });
var idxById = {};                            // BSP face id -> payload face index
for (var i = 0; i < nf; i++) idxById[D.fids[i]] = i;
var traceByFace = {};                        // BSP face id -> trace entry
D.traces.forEach(function (t) { traceByFace[t.face] = t; });

var corners = D.fvert.length;                // one entry per polygon corner
var positions = new Float32Array(corners * 3);
var colors = new Float32Array(corners * 3);
var totalTris = 0;
for (var i = 0; i < nf; i++) totalTris += Math.max(0, D.franges[i * 2 + 1] - 2);
var rawIndex = new Uint32Array(totalTris * 3);
var triFace = new Uint32Array(totalTris);    // global triangle -> payload face
var LIGHT = new THREE.Vector3(0.35, 0.25, 0.9).normalize();

function polyNormal(ofs, cnt) {              // Newell over the corner loop
  var nx = 0, ny = 0, nz = 0;
  for (var k = 0; k < cnt; k++) {
    var a = (D.fvert[ofs + k]) * 3, b = (D.fvert[ofs + (k + 1) % cnt]) * 3;
    nx += (D.verts[a + 1] - D.verts[b + 1]) * (D.verts[a + 2] + D.verts[b + 2]);
    ny += (D.verts[a + 2] - D.verts[b + 2]) * (D.verts[a] + D.verts[b]);
    nz += (D.verts[a] - D.verts[b]) * (D.verts[a + 1] + D.verts[b + 1]);
  }
  var len = Math.sqrt(nx * nx + ny * ny + nz * nz) || 1;
  return [nx / len, ny / len, nz / len];
}

var tri = 0, cornerBase = 0;
for (var i = 0; i < nf; i++) {
  var ofs = D.franges[i * 2], cnt = D.franges[i * 2 + 1];
  var cat = D.cats[D.fcat[i]];
  var shade = 1;                          // a face of under 3 corners has no normal (and no triangles)
  if (cnt >= 3) {
    var n = polyNormal(ofs, cnt);
    shade = 0.45 + 0.55 * Math.abs(n[0] * LIGHT.x + n[1] * LIGHT.y + n[2] * LIGHT.z);
  }
  var col = new THREE.Color(cat.color).multiplyScalar(shade);
  for (var k = 0; k < cnt; k++) {
    var v = D.fvert[ofs + k] * 3, c = (cornerBase + k) * 3;
    positions[c] = D.verts[v]; positions[c + 1] = D.verts[v + 1]; positions[c + 2] = D.verts[v + 2];
    colors[c] = col.r; colors[c + 1] = col.g; colors[c + 2] = col.b;
  }
  for (var k = 1; k + 1 < cnt; k++) {
    rawIndex[tri * 3] = cornerBase;
    rawIndex[tri * 3 + 1] = cornerBase + k;
    rawIndex[tri * 3 + 2] = cornerBase + k + 1;
    triFace[tri] = i;
    tri++;
  }
  cornerBase += cnt;
}

var geometry = new THREE.BufferGeometry();
geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
var visibleIndex = new Uint32Array(totalTris * 3);
var visTri = new Uint32Array(totalTris);     // drawn triangle -> global triangle
var material = new THREE.MeshBasicMaterial({ vertexColors: true,
                                             side: THREE.DoubleSide });
var mesh = new THREE.Mesh(geometry, material);
mesh.frustumCulled = false;
scene.add(mesh);

// Faces are sorted by category, so each category is one contiguous run of
// triangles: showing/hiding one copies its runs into the index buffer.
function applyCategoryVisibility() {
  var idx = 0;
  for (var c = 0; c < D.cats.length; c++) {
    if (!catVisible[c]) continue;
    var start = D.catRanges[c][0], count = D.catRanges[c][1];
    for (var t = 0; t < count; t++) {
      var g = start + t;
      visibleIndex[idx * 3] = rawIndex[g * 3];
      visibleIndex[idx * 3 + 1] = rawIndex[g * 3 + 1];
      visibleIndex[idx * 3 + 2] = rawIndex[g * 3 + 2];
      visTri[idx] = g;
      idx++;
    }
  }
  var buf = new Uint32Array(idx * 3);
  buf.set(visibleIndex.subarray(0, idx * 3));
  // visTri keeps its full size (its first idx entries are the drawn triangles): a
  // copy cut to idx would drop the entries of a category shown again later.
  geometry.setIndex(new THREE.BufferAttribute(buf, 1));
  geometry.boundingSphere = worldSphere();
  geometry.index.needsUpdate = true;
}

var _sphere = null;
function worldSphere() {
  if (_sphere) return _sphere;
  _sphere = new THREE.Sphere(new THREE.Vector3(), 1);
  geometry.computeBoundingSphere();
  _sphere = geometry.boundingSphere.clone();
  return _sphere;
}

// ── small face helpers ───────────────────────────────────────────────────
function faceCorners(i) {
  var ofs = D.franges[i * 2], cnt = D.franges[i * 2 + 1], pts = [];
  for (var k = 0; k < cnt; k++) {
    var v = D.fvert[ofs + k] * 3;
    pts.push(new THREE.Vector3(D.verts[v], D.verts[v + 1], D.verts[v + 2]));
  }
  return pts;
}
function faceCenter(i) {
  var pts = faceCorners(i), c = new THREE.Vector3();
  if (!pts.length) return c;
  pts.forEach(function (p) { c.add(p); });
  return c.multiplyScalar(1 / pts.length);
}
function faceSideIds(i) {
  var ofs = D.fsidr[i * 2], cnt = D.fsidr[i * 2 + 1], out = [];
  for (var k = 0; k < cnt; k++) out.push(D.fsids[ofs + k]);
  return out;
}
function faceBrushes(i) {                     // [[brushId, classname], ...]
  var out = [];
  faceSideIds(i).forEach(function (sid) {
    var b = sideToBrush.get(sid);
    if (b !== undefined && !out.some(function (e) { return e[0] === b; }))
      out.push([D.brushes[b][0], D.brushes[b][1]]);
  });
  out.sort(function (a, b) { return a[0] - b[0]; });
  return out;
}
var sideToBrush = new Map();
D.s2b.forEach(function (p) { sideToBrush.set(p[0], p[1]); });

function bboxOfFaces(list) {
  var box = new THREE.Box3(), v = new THREE.Vector3();
  list.forEach(function (i) {
    var ofs = D.franges[i * 2], cnt = D.franges[i * 2 + 1];
    for (var k = 0; k < cnt; k++) {
      var p = D.fvert[ofs + k] * 3;
      v.set(D.verts[p], D.verts[p + 1], D.verts[p + 2]);
      box.expandByPoint(v);
    }
  });
  if (box.isEmpty()) box.set(new THREE.Vector3(-64, -64, -64), new THREE.Vector3(64, 64, 64));
  var sphere = box.getBoundingSphere(new THREE.Sphere());
  return { box: box, center: sphere.center, radius: Math.max(sphere.radius, 16) };
}

// ── camera controls (Hammer-like fly + orbit) ────────────────────────────
var camPos = new THREE.Vector3(0, 0, 512);
var yaw = 0, pitch = -0.5;
var orbit = null;                             // {target, radius} while Alt-dragging
var focusPoint = new THREE.Vector3();         // what pan/wheel scale themselves to
var keys = {};

function forward() {
  var cp = Math.cos(pitch);
  return new THREE.Vector3(cp * Math.cos(yaw), cp * Math.sin(yaw), Math.sin(pitch));
}
function rightVec() { return new THREE.Vector3().setFromMatrixColumn(camera.matrixWorld, 0); }
function upVec() { return new THREE.Vector3().setFromMatrixColumn(camera.matrixWorld, 1); }
function updateCamera() {
  camera.position.copy(camPos);
  camera.lookAt(camPos.clone().add(forward()));
  camera.updateMatrixWorld();
}
// 'dir': the side the camera sits on, seen from the center (it then looks back along
// -dir); none keeps the current heading and backs the camera off along it.
function frame(center, radius, dir) {
  var look = dir ? dir.clone().normalize().negate() : forward();
  camPos.copy(center).addScaledVector(look, -radius * 2.2);
  yaw = Math.atan2(look.y, look.x);
  pitch = Math.asin(THREE.MathUtils.clamp(look.z, -1, 1));
  focusPoint.copy(center);
  sceneScale = Math.max(32, radius);           // marker / line sizes
  updateCamera();
}
// A sidelong view of face i's witness ray (eye -> face), from above its middle: a
// camera behind the eye tends to sit inside the wall the eye is near. False when
// the face has no witness. 'xray': turn X-ray on as well.
function frameWitness(i, xray) {
  var tr = traceByFace[D.fids[i]];
  var w = tr && tr.witness;
  var eye = w ? vec(w.eye_pos || (w.path || [])[0] || [0, 0, 0]) : null;
  if (!eye) return false;
  var c = faceCenter(i);
  var seg = c.clone().sub(eye);
  if (seg.length() <= 1) return false;
  var mid = eye.clone().add(c).multiplyScalar(0.5);
  var side = new THREE.Vector3(0, 0, 1).cross(seg);
  if (side.lengthSq() < 1e-6) side.set(1, 0, 0);
  side.normalize().add(new THREE.Vector3(0, 0, 0.6)).normalize();
  camPos.copy(mid).addScaledVector(side, Math.max(seg.length() * 1.3, 128));
  if (xray && !ui.xray.checked) {
    ui.xray.checked = true;
    ui.xray.dispatchEvent(new Event('change'));
  }
  var dir = mid.clone().sub(camPos).normalize();
  yaw = Math.atan2(dir.y, dir.x);
  pitch = Math.asin(THREE.MathUtils.clamp(dir.z, -1, 1));
  focusPoint.copy(mid);
  sceneScale = Math.max(32, camPos.distanceTo(c));
  updateCamera();
  return true;
}
function frameSelection() {
  if (selected.length === 1 && frameWitness(selected[0], false)) return;
  if (selected.length) {
    var b = bboxOfFaces(selected);
    frame(b.center, b.radius, null);
  } else {
    frameAll();
  }
}
function frameAll() {
  var b = bboxOfFaces(allFaces());
  frame(b.center, b.radius * 1.4,
        new THREE.Vector3(0.55, -0.75, 0.65).normalize());
}
function allFaces() {
  var out = [];
  for (var i = 0; i < nf; i++) out.push(i);
  return out;
}

// ── picking ──────────────────────────────────────────────────────────────
var raycaster = new THREE.Raycaster();
function pickAt(clientX, clientY) {
  var r = canvas.getBoundingClientRect();
  var ndc = new THREE.Vector2(((clientX - r.left) / r.width) * 2 - 1,
                               -((clientY - r.top) / r.height) * 2 + 1);
  raycaster.setFromCamera(ndc, camera);
  var hits = raycaster.intersectObject(mesh);
  if (!hits.length) return -1;
  return triFace[visTri[hits[0].faceIndex]] || 0;
}
function centerHit() {
  raycaster.setFromCamera(new THREE.Vector2(0, 0), camera);
  var hits = raycaster.intersectObject(mesh);
  return hits.length ? hits[0].point : null;
}

// ── overlays ─────────────────────────────────────────────────────────────
var overlays = new THREE.Group();              // per-selection overlays
scene.add(overlays);
var allWitness = new THREE.Group();            // every traced face's witness path
scene.add(allWitness);
var sceneScale = 64;                           // set after the first framing
var SOURCE_COLORS = {
  node: '#35d461', cubemap: '#e24ce0', ladder: '#d4c542', water: '#3c9ee0',
  arc: '#ff922b', ride: '#d46ecf', teleport: '#8f6bff', point_camera: '#e0563c',
  point_viewcontrol: '#e08b3c', cli: '#ffffff', sky_camera: '#7fd4e0', surf: '#50e0b0'
};
var PIECE_KINDS = ['point', 'segment', 'polyhedron'];   // EyePieceKind
function sourceColor(s) { return SOURCE_COLORS[s] || '#9a9a9a'; }

function lineMat(color, opacity) {
  return new THREE.LineBasicMaterial({ color: color, transparent: true,
                                       opacity: opacity,
                                       depthTest: !ui.xray.checked });
}
function clearGroup(g) {
  while (g.children.length) {
    var c = g.children.pop();
    g.remove(c);
    if (c.geometry) c.geometry.dispose();
    if (c.material) c.material.dispose();
  }
}
function addLine(g, pts, color, opacity) {
  if (pts.length < 2) return;
  var geo = new THREE.BufferGeometry().setFromPoints(pts);
  g.add(new THREE.Line(geo, lineMat(color, opacity)));
}
function addLineLoop(g, pts, color, opacity) {
  if (pts.length < 2) return;
  var geo = new THREE.BufferGeometry().setFromPoints(pts);
  g.add(new THREE.LineLoop(geo, lineMat(color, opacity)));
}
function addThickSegment(g, a, b, color, radius) {
  var dir = b.clone().sub(a), len = dir.length();
  if (len < 1e-4) return;
  var geo = new THREE.CylinderGeometry(radius, radius, len, 6, 1, true);
  var mat = new THREE.MeshBasicMaterial({ color: color, transparent: true, opacity: 0.95,
                                          depthTest: !ui.xray.checked });
  var m = new THREE.Mesh(geo, mat);
  m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.normalize());
  m.position.copy(a).addScaledVector(dir, len / 2);
  g.add(m);
}
function addMarker(g, p, color, size) {
  var geo = new THREE.SphereGeometry(size, 10, 8);
  var mat = new THREE.MeshBasicMaterial({ color: color, depthTest: !ui.xray.checked });
  var m = new THREE.Mesh(geo, mat);
  m.position.copy(p);
  g.add(m);
}
function addCubeMarker(g, p, color, size) {
  var geo = new THREE.BoxGeometry(size, size, size);
  var mat = new THREE.MeshBasicMaterial({ color: color, depthTest: !ui.xray.checked });
  var m = new THREE.Mesh(geo, mat);
  m.position.copy(p);
  g.add(m);
}
function vec(x) { return new THREE.Vector3(x[0] || 0, x[1] || 0, x[2] || 0); }
function vecs(list) { return (list || []).map(vec); }

// An eye piece polyhedron (its vertices are a welded set, not an edge loop): eye
// node columns are vertical prisms over a polygon, so draw, per distinct XY, the lowest and the
// highest vertex as a bottom and a top loop (sorted by angle) joined by uprights.
function drawPolyhedron(pv, color) {
  var cols = {};
  pv.forEach(function (v) {
    var k = Math.round(v.x * 8) + ',' + Math.round(v.y * 8);
    var c = cols[k] || (cols[k] = { lo: v, hi: v });
    if (v.z < c.lo.z) c.lo = v;
    if (v.z > c.hi.z) c.hi = v;
  });
  var list = Object.keys(cols).map(function (k) { return cols[k]; });
  // Not a vertical prism (a ride or volume hull, a tilted one): the JSON has no
  // faces to draw its edges from, so join every pair of vertices (a few dozen lines).
  if (list.length < 3 || list.length * 2 !== pv.length ||
      list.some(function (c) { return c.hi.z <= c.lo.z; })) {
    for (var a = 0; a < pv.length; a++)
      for (var b = a + 1; b < pv.length; b++) addLine(overlays, [pv[a], pv[b]], color, 0.35);
    return;
  }
  var cx = 0, cy = 0;
  list.forEach(function (c) { cx += c.lo.x; cy += c.lo.y; });
  cx /= list.length; cy /= list.length;
  list.sort(function (a, b) {
    return Math.atan2(a.lo.y - cy, a.lo.x - cx) - Math.atan2(b.lo.y - cy, b.lo.x - cx);
  });
  if (list.length >= 3) {
    addLineLoop(overlays, list.map(function (c) { return c.lo; }), color, 0.9);
    addLineLoop(overlays, list.map(function (c) { return c.hi; }), color, 0.9);
  }
  list.forEach(function (c) {
    if (c.hi.z > c.lo.z) addLine(overlays, [c.lo, c.hi], color, 0.6);
  });
}

function rebuildOverlays() {
  clearGroup(overlays);
  buildAllWitnessGroup();
  if (activeFace < 0) return;
  var i = activeFace;
  var tr = traceByFace[D.fids[i]];

  if (ui.ovOutline.checked) {                  // the face itself, filled and outlined
    var fc = faceCorners(i), fill = [];
    for (var t = 1; t + 1 < fc.length; t++) fill.push(fc[0], fc[t], fc[t + 1]);
    var fg = new THREE.BufferGeometry().setFromPoints(fill);
    overlays.add(new THREE.Mesh(fg, new THREE.MeshBasicMaterial({
      color: '#ffffff', transparent: true, opacity: 0.45, side: THREE.DoubleSide,
      depthTest: !ui.xray.checked, polygonOffset: true, polygonOffsetFactor: -2,
      polygonOffsetUnits: -2 })));
    addLineLoop(overlays, fc, '#ffffff', 0.95);
    selected.forEach(function (j) {
      if (j !== i) addLineLoop(overlays, faceCorners(j), D.cats[D.fcat[j]].color, 0.8);
    });
  }
  if (!tr) return;
  var w = tr.witness;
  if (w && ui.ovPath.checked) {                // eye -> face point, bright and thick
    var pts = vecs(w.path);
    for (var k = 1; k < pts.length; k++)
      addThickSegment(overlays, pts[k - 1], pts[k], '#ffe14d',
                      Math.max(0.75, sceneScale * 0.006));
    if (pts.length >= 2) addCubeMarker(overlays, pts[pts.length - 1], '#ffe14d',
                                       Math.max(1.5, sceneScale * 0.012));  // the face point
  }
  if (w && ui.ovEye.checked && w.eye_pos)
    addMarker(overlays, vec(w.eye_pos), sourceColor(w.source),
              Math.max(2.5, sceneScale * 0.02));
  if (ui.ovFan.checked && tr.fanRays.length) { // the other fan rays, thinner and dimmer
    var segs = [];
    tr.fanRays.forEach(function (r) {
      if (r && r.eye_pos && r.hit) segs.push(vec(r.eye_pos), vec(r.hit));
    });
    if (segs.length)
      overlays.add(new THREE.LineSegments(
        new THREE.BufferGeometry().setFromPoints(segs), lineMat('#c8b04a', 0.4)));
  }
  if (ui.ovPieces.checked || ui.ovChain.checked) {
    tr.eyes.forEach(function (ei) {
      var eye = D.eyes[ei];
      if (!eye) return;
      if (ui.ovPieces.checked) eye.pieces.forEach(function (piece) {
        var pv = vecs(piece.verts);
        if (pv.length >= 3) drawPolyhedron(pv, sourceColor(piece.source));
        else if (pv.length === 2) addLine(overlays, pv, sourceColor(piece.source), 0.9);
        else if (pv.length === 1) addMarker(overlays, pv[0], sourceColor(piece.source),
                                            Math.max(1.5, sceneScale * 0.01));
      });
      if (ui.ovChain.checked && eye.chain >= 0) {
        var node = D.nodes[eye.chain];
        var centroids = [], seedPoint = null;
        (node.steps || []).forEach(function (step) {
          var pv = vecs(step.poly);
          if (pv.length >= 3) {
            addLineLoop(overlays, pv, '#4dd2ff', 0.5);
            var c = new THREE.Vector3();
            pv.forEach(function (p) { c.add(p); });
            centroids.push(c.multiplyScalar(1 / pv.length));
          }
          if (step.seed && step.seed.point) seedPoint = vec(step.seed.point);
        });
        addLine(overlays, centroids, '#4dd2ff', 0.85);
        if (seedPoint) addMarker(overlays, seedPoint, '#ffd21f',
                                 Math.max(2.5, sceneScale * 0.018));
      }
    });
  }
}
function buildAllWitnessGroup() {
  clearGroup(allWitness);
  if (!ui.allWit.checked) return;
  D.traces.forEach(function (tr) {
    if (!tr.witness || !tr.witness.path) return;
    var pts = vecs(tr.witness.path);
    for (var k = 1; k < pts.length; k++)
      addThickSegment(allWitness, pts[k - 1], pts[k], sourceColor(tr.witness.source),
                      Math.max(0.6, sceneScale * 0.004));
  });
  allWitness.visible = true;
}

// ── panel ────────────────────────────────────────────────────────────────
var ui = {
  xray: document.getElementById('xray'),
  hideHidden: document.getElementById('hideHidden'),
  allWit: document.getElementById('allWit'),
  // defaults stand in until renderDetail() creates the real checkboxes
  ovOutline: { checked: true }, ovPath: { checked: true }, ovEye: { checked: true },
  ovFan: { checked: true }, ovPieces: { checked: true }, ovChain: { checked: true }
};
function esc(s) {
  return String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
}
function fmtVec(v) {
  if (!v || v.length !== 3) return '?';
  return '(' + v.map(function (c) { return Math.round(c * 100) / 100; }).join(' ') + ')';
}

function buildLegend() {
  var counts = D.cats.map(function () { return 0; });
  for (var i = 0; i < nf; i++) counts[D.fcat[i]]++;
  var el = document.getElementById('legend');
  el.innerHTML = '';
  D.cats.forEach(function (cat, c) {
    var row = document.createElement('div');
    row.className = 'legend-row';
    var cb = document.createElement('input');
    cb.type = 'checkbox';
    cb.checked = true;
    cb.addEventListener('change', function () {
      catVisible[c] = cb.checked;
      ui.hideHidden.checked = D.cats.every(function (k, j) {
        return (k.key !== 'A' && k.key !== 'B') || !catVisible[j];
      });
      applyCategoryVisibility();
    });
    var sw = document.createElement('span');
    sw.className = 'swatch';
    sw.style.background = cat.color;
    var lab = document.createElement('span');
    lab.textContent = cat.label;
    var cnt = document.createElement('span');
    cnt.className = 'cnt';
    cnt.textContent = counts[c].toLocaleString();
    row.appendChild(cb); row.appendChild(sw); row.appendChild(lab); row.appendChild(cnt);
    el.appendChild(row);
    cat._cb = cb;
  });
}
function buildPreList() {
  var el = document.getElementById('preList');
  el.innerHTML = '';
  D.pre.forEach(function (i, k) {
    var b = document.createElement('span');
    b.className = 'facebtn';
    b.textContent = 'face ' + D.preIds[k] +
        (traceByFace[D.preIds[k]] ? ' (' + D.cats[D.fcat[i]].key + ')' : ' (untraced)');
    b.addEventListener('click', function () { setSelected([i], true); });
    el.appendChild(b);
  });
  if (!D.pre.length) el.innerHTML = '<span class="legend-note">none (pass --faces / --sides)</span>';
}

function tierText(i) {
  return D.tkeys[D.ftr[i]].split('|').join(' / ');
}
function renderDetail() {
  var el = document.getElementById('detailBody');
  if (activeFace < 0) { el.innerHTML = 'click a face'; return; }
  var i = activeFace;
  var h = [];
  var tr = traceByFace[D.fids[i]];
  h.push('<div><b style="color:' + D.cats[D.fcat[i]].color + '">face ' + D.fids[i] +
         '</b> &mdash; ' + esc(tierText(i)) + '</div>');
  var sides = faceSideIds(i);
  var brushes = faceBrushes(i);
  h.push('<table class="kv">');
  h.push('<tr><td>BSP face id</td><td>' + D.fids[i] + '</td></tr>');
  h.push('<tr><td>VMF side ids</td><td>' +
         (sides.length ? esc(sides.join(', ')) : '<span class="warn">unknown</span>') +
         '</td></tr>');
  h.push('<tr><td>VMF brushes</td><td>' +
         (brushes.length ? brushes.map(function (b) {
             return b[0] + (b[1] ? ' (' + esc(b[1]) + ')' : ' (world)');
           }).join(', ')
           : (sides.length ? 'not in the VMF' : '<span class="warn">unknown</span>')) +
         '</td></tr>');
  if (D.fdisp[i]) h.push('<tr><td>displacement</td><td>drawn as its base polygon</td></tr>');
  h.push('</table>');

  if (!tr) {
    h.push('<p class="warn">this face was not traced; run bspwalk with -traceface for it:</p>');
    h.push('<code class="cmd">' + esc(D.cmd.replace('{F}', String(D.fids[i]))) + '</code>');
    el.innerHTML = h.join('');
    return;
  }

  var w = tr.witness;
  if (w) {
    var plen = 0, path = w.path || [];
    for (var k = 1; k < path.length; k++) {
      var dx = path[k][0] - path[k - 1][0], dy = path[k][1] - path[k - 1][1],
          dz = path[k][2] - path[k - 1][2];
      plen += Math.sqrt(dx * dx + dy * dy + dz * dz);
    }
    h.push('<table class="kv">');
    h.push('<tr><td>witness pass</td><td>' + esc(w.pass || '?') + '</td></tr>');
    h.push('<tr><td>eye source</td><td><span style="color:' +
           sourceColor(w.source) + '">' + esc(w.source || '?') + '</span></td></tr>');
    h.push('<tr><td>eye pos</td><td>' + esc(fmtVec(w.eye_pos)) + '</td></tr>');
    if (w.cluster !== undefined) h.push('<tr><td>cluster</td><td>' + w.cluster + '</td></tr>');
    if (w.level !== undefined) h.push('<tr><td>level</td><td>' + w.level + '</td></tr>');
    if (w.sky !== undefined) h.push('<tr><td>sky</td><td>' + w.sky + '</td></tr>');
    h.push('<tr><td>path</td><td>' + path.length + ' points, ' +
           Math.round(plen) + ' u</td></tr>');
    h.push('</table>');
  } else {
    h.push('<p class="warn">no witness: bspwalk kept this face ' + esc(tierText(i)) +
           ' without a ray that saw it.</p>');
  }

  // the overlay toggles keep their state from one selected face to the next
  var ov = function (id, text) {
    return '<label><input type="checkbox" id="' + id + '"' + (ui[id].checked ? ' checked' : '') +
           '> ' + text + '</label>';
  };
  h.push('<div style="margin-top:6px"><b style="font-size:11px;color:#9aa0a8">' +
         'OVERLAYS</b><br>' +
         ov('ovOutline', 'face outline') + '<br>' +
         ov('ovPath', 'witness path + face point') + '<br>' +
         ov('ovEye', 'witness eye') + '<br>' +
         ov('ovFan', 'other fan rays') + '<br>' +
         ov('ovPieces', 'eye pieces') + '<br>' +
         ov('ovChain', 'walk chain + seed') +
         '</div>');
  if (tr.fanRays.length)
    h.push('<div class="legend-note">' + tr.fanRays.length + ' fan rays (' +
           tr.fanRays.filter(function (r) { return r && r.hit; }).length + ' hit)</div>');

  if (tr.eyes.length) {
    h.push('<div style="margin-top:6px"><b style="font-size:11px;color:#9aa0a8">EYE PIECES</b>');
    h.push('<table class="rows"><tr><th>source</th><th>node</th><th>kind</th></tr>');
    tr.eyes.forEach(function (ei) {
      var eye = D.eyes[ei];
      if (!eye) return;
      (eye.pieces || []).forEach(function (p) {
        h.push('<tr><td style="color:' + sourceColor(p.source) + '">' + esc(p.source || '?') +
               '</td><td>' + (p.node >= 0 ? p.node : '-') + '</td><td>' +
               esc(PIECE_KINDS[p.kind] || String(p.kind)) + '</td></tr>');
      });
    });
    h.push('</table></div>');
    var chain = -1;
    tr.eyes.forEach(function (ei) {
      if (chain < 0 && D.eyes[ei] && D.eyes[ei].chain >= 0) chain = D.eyes[ei].chain;
    });
    if (chain >= 0) {
      var node = D.nodes[chain];
      h.push('<div style="margin-top:6px"><b style="font-size:11px;color:#9aa0a8">' +
             'WALK CHAIN (node ' + node.node + (node.reached ? '' : ', unreached') +
             (node.narrow ? ', narrow' : '') + ')</b>');
      h.push('<table class="rows"><tr><th>node</th><th>how / edge</th><th>z range</th></tr>');
      (node.steps || []).forEach(function (s) {
        var how = (s.how || '?') + (s.edge ? ' / ' + s.edge : '');
        if (s.node === -1) {
          h.push('<tr><td>special ' + (s.special !== undefined ? s.special : '?') + '</td><td>' +
                 esc(s.label || how) + '</td><td></td></tr>');
          return;
        }
        var zr = (s.mins && s.maxs) ? Math.round(s.mins[2]) + ' .. ' + Math.round(s.maxs[2]) : '';
        h.push('<tr><td>' + s.node + '</td><td>' + esc(how) + '</td><td>' + zr + '</td></tr>');
        if (s.seed)
          h.push('<tr><td></td><td colspan="2">seed: ' + esc(s.seed.source || '?') + ' at ' +
                 fmtVec(s.seed.point) + '</td></tr>');
      });
      h.push('</table></div>');
    }
  }
  h.push('<div style="margin-top:6px"><b style="font-size:11px;color:#9aa0a8">BSPWALK LOG</b></div>');
  h.push('<pre class="log">' + esc(tr.log || '(empty)') + '</pre>');
  el.innerHTML = h.join('');
  ['ovOutline', 'ovPath', 'ovEye', 'ovFan', 'ovPieces', 'ovChain'].forEach(function (id) {
    ui[id] = document.getElementById(id);
    ui[id].addEventListener('change', rebuildOverlays);
  });
}

// ── selection ────────────────────────────────────────────────────────────
var selected = [];                             // payload face indices
var activeFace = -1;
function setSelected(list, doFrame) {
  selected = list.filter(function (i) { return i >= 0 && i < nf; });
  activeFace = selected.length ? selected[0] : -1;
  Array.prototype.forEach.call(document.querySelectorAll('.facebtn'), function (b, k) {
    b.classList.toggle('active', D.pre[k] !== undefined && selected.indexOf(D.pre[k]) >= 0);
  });
  renderDetail();
  rebuildOverlays();
  if (doFrame) frameSelection();
}

// ── search ───────────────────────────────────────────────────────────────
function searchFaces(q) {
  q = q.trim().toLowerCase();
  var out = [];
  function addBy(pred) {
    for (var i = 0; i < nf; i++) if (pred(i)) out.push(i);
  }
  if (/^s\d+$/.test(q)) {
    var sid = +q.slice(1);
    addBy(function (i) { return faceSideIds(i).indexOf(sid) >= 0; });
  } else if (/^b-?\d+$/.test(q)) {
    var bid = +q.slice(1);
    addBy(function (i) {
      return faceBrushes(i).some(function (b) { return b[0] === bid; });
    });
  } else if (/^\d+$/.test(q)) {
    var i = idxById[+q];
    if (i !== undefined) out.push(i);
  } else {
    return null;
  }
  return out;
}
function runSearch() {
  var res = searchFaces(document.getElementById('search').value);
  var hint = document.getElementById('searchhint');
  if (res === null) {
    hint.textContent = 'not understood: use a face id, s<sideid> or b<brushid>';
    return;
  }
  if (!res.length) {
    hint.textContent = 'no matching face';
    return;
  }
  hint.textContent = res.length + ' face(s)';
  setSelected(res.slice(0, 200), true);
}
document.getElementById('searchgo').addEventListener('click', runSearch);
document.getElementById('search').addEventListener('keydown', function (e) {
  if (e.key === 'Enter') { runSearch(); e.stopPropagation(); }
});

// ── options ──────────────────────────────────────────────────────────────
ui.xray.addEventListener('change', function () {
  material.transparent = ui.xray.checked;
  material.opacity = ui.xray.checked ? 0.25 : 1.0;
  material.depthWrite = !ui.xray.checked;
  material.needsUpdate = true;
  rebuildOverlays();
});
ui.hideHidden.addEventListener('change', function () {
  var show = !ui.hideHidden.checked;
  D.cats.forEach(function (cat, c) {
    if (cat.key === 'A' || cat.key === 'B') {
      catVisible[c] = show;
      cat._cb.checked = show;
    }
  });
  applyCategoryVisibility();
});
ui.allWit.addEventListener('change', rebuildOverlays);

// ── input: mouse ─────────────────────────────────────────────────────────
var drag = null;                               // {mode, x, y}
canvas.addEventListener('contextmenu', function (e) { e.preventDefault(); });
canvas.addEventListener('mousedown', function (e) {
  if (e.button === 2) drag = { mode: 'look', x: e.clientX, y: e.clientY };
  else if (e.button === 1) { e.preventDefault(); drag = { mode: 'pan', x: e.clientX, y: e.clientY }; }
  else if (e.button === 0 && e.altKey) {
    var target = (activeFace >= 0) ? faceCenter(activeFace).clone() : centerHit();
    if (!target) target = camPos.clone().addScaledVector(forward(), 512);
    orbit = { target: target, radius: target.distanceTo(camPos) };
    focusPoint.copy(target);
    drag = { mode: 'orbit', x: e.clientX, y: e.clientY };
  } else if (e.button === 0)
    drag = { mode: 'click', x: e.clientX, y: e.clientY, sx: e.clientX, sy: e.clientY };
});
window.addEventListener('mousemove', function (e) {
  if (document.pointerLockElement === canvas) {           // Z toggled mouselook
    yaw -= e.movementX * 0.0032;
    pitch = THREE.MathUtils.clamp(pitch - e.movementY * 0.0032, -1.55, 1.55);
    if (orbit) camPos.copy(orbit.target).addScaledVector(forward(), -orbit.radius);
    updateCamera();
    return;
  }
  if (!drag) return;
  var dx = e.clientX - drag.x, dy = e.clientY - drag.y;
  drag.x = e.clientX; drag.y = e.clientY;
  if (drag.mode === 'look') {
    yaw -= dx * 0.005;                         // yaw grows to the left (Z-up): drag right turns right
    pitch = THREE.MathUtils.clamp(pitch - dy * 0.005, -1.55, 1.55);
    if (orbit) camPos.copy(orbit.target).addScaledVector(forward(), -orbit.radius);
  } else if (drag.mode === 'pan') {
    var k = Math.max(16, camPos.distanceTo(focusPoint)) * 0.0016;
    camPos.addScaledVector(rightVec(), -dx * k).addScaledVector(upVec(), dy * k);
    orbit = null;
  } else if (drag.mode === 'orbit') {
    yaw -= dx * 0.005;
    pitch = THREE.MathUtils.clamp(pitch - dy * 0.005, -1.55, 1.55);
    camPos.copy(orbit.target).addScaledVector(forward(), -orbit.radius);
  } else if (drag.mode === 'click') {
    if (Math.abs(e.clientX - drag.sx) + Math.abs(e.clientY - drag.sy) > 4) drag.mode = 'dead';
  }
  updateCamera();
});
window.addEventListener('mouseup', function (e) {
  if (drag && drag.mode === 'click' && e.button === 0) {
    var f = pickAt(e.clientX, e.clientY);
    if (f >= 0) {
      focusPoint.copy(faceCenter(f));
      setSelected([f], false);
    }
  }
  orbit = null;                                // orbit only lives while the button is down
  drag = null;
});
canvas.addEventListener('wheel', function (e) {
  e.preventDefault();
  var step = Math.sign(e.deltaY) * Math.max(2, camPos.distanceTo(focusPoint) * 0.12);
  camPos.addScaledVector(forward(), -step);
  orbit = null;
  updateCamera();
}, { passive: false });

// ── input: keyboard ──────────────────────────────────────────────────────
window.addEventListener('keydown', function (e) {
  if (document.activeElement && document.activeElement.tagName === 'INPUT') return;
  keys[e.code] = true;
  if (e.code === 'KeyZ') {
    if (document.pointerLockElement === canvas) document.exitPointerLock();
    else canvas.requestPointerLock();
  }
  if (e.code === 'KeyF') frameSelection();
  if (['Space', 'ControlLeft', 'ControlRight', 'KeyW', 'KeyA', 'KeyS', 'KeyD',
       'KeyQ', 'KeyE'].indexOf(e.code) >= 0) e.preventDefault();
});
window.addEventListener('keyup', function (e) { keys[e.code] = false; });

// ── main loop ────────────────────────────────────────────────────────────
var clock = new THREE.Clock();
function animate() {
  requestAnimationFrame(animate);
  var dt = Math.min(clock.getDelta(), 0.1);
  var move = new THREE.Vector3();
  if (keys.KeyW) move.add(forward());
  if (keys.KeyS) move.sub(forward());
  if (keys.KeyD) move.add(rightVec());
  if (keys.KeyA) move.sub(rightVec());
  if (keys.KeyQ || keys.Space) move.z += 1;
  if (keys.KeyE || keys.ControlLeft || keys.ControlRight) move.z -= 1;
  if (move.lengthSq() > 0) {
    move.normalize().multiplyScalar((keys.ShiftLeft || keys.ShiftRight ? 1500 : 450) * dt);
    camPos.add(move);
    orbit = null;
    updateCamera();
  }
  renderer.render(scene, camera);
}
function resize() {
  camera.aspect = container.clientWidth / container.clientHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(container.clientWidth, container.clientHeight);
}
window.addEventListener('resize', resize);

// ── boot ─────────────────────────────────────────────────────────────────
document.getElementById('meta').innerHTML =
  'BSP: ' + esc(D.bsp) + ' (made ' + esc(D.bspMade) + ')<br>page made ' + esc(D.made) +
  ' &middot; ' + nf.toLocaleString() + ' faces &middot; ' +
  totalTris.toLocaleString() + ' triangles &middot; ' + D.traces.length + ' traced';
buildLegend();
buildPreList();
applyCategoryVisibility();
resize();
setSelected(D.pre.slice(0, 1), false);
initialCamera();
updateCamera();
animate();

function initialCamera() {
  // The first selected face, framed with its witness ray (X-ray on, so the ray
  // shows through walls); with no selection, the whole map.
  if (!D.pre.length) { frameAll(); return; }
  if (!frameWitness(D.pre[0], true))
    frame(faceCenter(D.pre[0]), Math.max(bboxOfFaces([D.pre[0]]).radius, 128), null);
}
})();
</script>
</body>
</html>
"""

if __name__ == "__main__":
    main()
