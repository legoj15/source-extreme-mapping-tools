"""Real-binary guard: the shared UV overlap detector (issue #56,
src/utils/common/uvoverlap.cpp), through `vrad_rtx -uvoverlap`.

Run:  python -I game/bin/x64/unit_test/proplm/test_proplm_uvoverlap.py [--game DIR]
  or: pytest game/bin/x64/unit_test/proplm
Needs vrad_rtx.exe in game/bin/x64; the real-model check also needs a game
directory that mounts props_wasteland/kitchen_fridge001a (default HL2
Deathmatch's hl2mp, or PROPLM_TEST_GAME). Missing either is SKIP.

What it protects: the overlap the detector reports on five hand-made meshes in
uvoverlap/ (disjoint quads, mirrored halves with one normal, a quad tiled 1.5
times, a quad creased along a diagonal that runs through cell centers, a flap
folded back over its neighbour) must equal, cell for cell and per category,
what an exact rational-arithmetic reference of the same rules computes here.
Breaks if mirrored halves are treated as one island or exempted by normal, if
cells are not folded into one tile, if a shared edge's cell centers are
claimed by both triangles, if a fold inside an island is missed, or if the
categories are mixed up. The real model must report the same island count as
the VBSP rebake (89 islands for the fridge, docs/proplm.md).
"""
import os
import re
import subprocess
import sys
from fractions import Fraction
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'lmoptimizer'))
from _testkit import BIN_DIR, check, run_all

VRAD = BIN_DIR / 'vrad_rtx.exe'
FIXTURES = Path(__file__).resolve().parent / 'uvoverlap'
DEFAULT_GAME = r"E:/Steam/steamapps/common/Half-Life 2 Deathmatch/hl2mp"
GAME = Path(os.environ.get('PROPLM_TEST_GAME', DEFAULT_GAME))
RES = 8

# fixture -> (islands, overlap cells, fold, repeat, other, spanning) that the
# meshes were drawn to give; the reference must agree with these too.
EXPECTED_SHAPE = {
    'disjoint': (2, 0, 0, 0, 0, 0),
    'mirrored': (2, 64, 0, 0, 64, 0),
    'tiled': (1, 32, 0, 32, 0, 2),
    'crease': (1, 0, 0, 0, 0, 0),
    'fold': (1, None, None, 0, 0, 0),  # fold == overlap > 0, from the reference
}


def _skip(reason):
    try:
        import pytest
        pytest.skip(reason)
    except ImportError:
        print(f"  SKIP  {reason}")


def read_mesh(path):
    verts, tris = [], []
    for line in path.read_text().splitlines():
        f = line.split()
        if not f or f[0].startswith('#'):
            continue
        if f[0] == 'v':
            pos = tuple(float(x) for x in f[1:4])
            uv = (Fraction(f[4]), Fraction(f[5]))
            sign = int(f[6]) if len(f) > 6 else 1
            verts.append((pos, uv, sign))
        elif f[0] == 'f':
            tris.append(tuple(int(x) for x in f[1:4]))
    return verts, tris


def reference(verts, tris, res):
    """The rules of uvoverlap.h in exact arithmetic."""
    parent = list(range(len(tris)))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    owner_of_vert, edge_owner = {}, {}
    for t, tri in enumerate(tris):
        for v in tri:
            if v in owner_of_vert:
                parent[find(t)] = find(owner_of_vert[v])
            else:
                owner_of_vert[v] = t
        for e in range(3):
            a, b = verts[tri[e]], verts[tri[(e + 1) % 3]]
            if a == b:
                continue
            key = tuple(sorted([a, b]))
            if key in edge_owner:
                parent[find(t)] = find(edge_owner[key])
            else:
                edge_owner[key] = t
    island_of_root, island = {}, []
    for t in range(len(tris)):
        island.append(island_of_root.setdefault(find(t), len(island_of_root)))

    claims = {}  # folded cell -> list of (tri, tile)
    spanning = 0
    for t, tri in enumerate(tris):
        p = [(verts[v][1][0] * res, verts[v][1][1] * res) for v in tri]
        area = (p[1][0] - p[0][0]) * (p[2][1] - p[0][1]) - (p[1][1] - p[0][1]) * (p[2][0] - p[0][0])
        if area == 0:
            continue
        if area < 0:
            p[1], p[2] = p[2], p[1]
        us = [verts[v][1][0] for v in tri]
        vs = [verts[v][1][1] for v in tri]
        if max(us) - min(us) > 1 or max(vs) - min(vs) > 1:
            spanning += 1
        xs = [q[0] for q in p]
        ys = [q[1] for q in p]
        for y in range(int(min(ys)) - 1, int(max(ys)) + 2):
            for x in range(int(min(xs)) - 1, int(max(xs)) + 2):
                c = (Fraction(2 * x + 1, 2), Fraction(2 * y + 1, 2))
                inside = True
                for e in range(3):
                    a, b = p[e], p[(e + 1) % 3]
                    d = (b[0] - a[0], b[1] - a[1])
                    ev = d[0] * (c[1] - a[1]) - d[1] * (c[0] - a[0])
                    # on the edge: the point is nudged by (-1, -eta)
                    if ev < 0 or (ev == 0 and not (d[1] > 0 or (d[1] == 0 and d[0] < 0))):
                        inside = False
                        break
                if inside:
                    cell = (x % res, y % res)
                    claims.setdefault(cell, []).append((t, (x // res, y // res)))
    n_isl = len(island_of_root)
    per = [dict(fold=0, repeat=0, other=0) for _ in range(n_isl)]
    total = dict(cells=len(claims), overlap=0, fold=0, repeat=0, other=0)
    for cell, cl in claims.items():
        fold, repeat, isl_here = set(), set(), {island[t] for t, _ in cl}
        for i in range(len(cl)):
            for j in range(i + 1, len(cl)):
                (ti, ki), (tj, kj) = cl[i], cl[j]
                if island[ti] != island[tj]:
                    continue
                if ki != kj:
                    repeat.add(island[ti])
                elif ti != tj:
                    fold.add(island[ti])
        other = isl_here if len(isl_here) > 1 else set()
        for s, name in ((fold, 'fold'), (repeat, 'repeat'), (other, 'other')):
            for i in s:
                per[i][name] += 1
            total[name] += 1 if s else 0
        total['overlap'] += 1 if (fold or repeat or other) else 0
    return n_isl, total, spanning, per


SUMMARY = re.compile(r'uvoverlap: .*: (\d+) triangles, (\d+) islands, grid (\d+)x(\d+) per UV unit, '
                     r'(\d+) cells covered, (\d+) overlapped')
CATS = re.compile(r'uvoverlap: .*: fold (\d+), repeat (\d+), other (\d+) cells; (\d+) triangles span')
ISLAND = re.compile(r'uvoverlap: .*: island (\d+): (\d+) triangles, (\d+) cells, fold (\d+), repeat (\d+), '
                    r'other (\d+), spanning (\d+)')


def run_vrad(target, res):
    proc = subprocess.run([str(VRAD), '-game', str(GAME), '-uvoverlap', str(target), '-uvoverlapres', str(res)],
                          capture_output=True, text=True, errors='replace', timeout=300)
    out = proc.stdout + proc.stderr
    return proc.returncode, out


def test_fixtures_match_exact_reference():
    if not VRAD.is_file():
        return _skip(f"not built: {VRAD.name}")
    for name, shape in EXPECTED_SHAPE.items():
        verts, tris = read_mesh(FIXTURES / f'{name}.uvmesh')
        n_isl, total, spanning, per = reference(verts, tris, RES)
        e_isl, e_ov, e_fold, e_rep, e_oth, e_span = shape
        got_shape = (n_isl, total['overlap'], total['fold'], total['repeat'], total['other'], spanning)
        want_shape = tuple(g if w is None else w for g, w in zip(got_shape, shape))
        check(f'{name}: the reference gives the shape the mesh was drawn for', got_shape == want_shape,
              f'reference {got_shape}, drawn {shape}')
        if name == 'fold':
            check('fold: the reference finds a fold', total['fold'] > 0 and total['fold'] == total['overlap'])

        rc, out = run_vrad(FIXTURES / f'{name}.uvmesh', RES)
        m, c = SUMMARY.search(out), CATS.search(out)
        check(f'{name}: vrad -uvoverlap prints its report', rc == 0 and m and c, out[-400:])
        got = (int(m.group(2)), int(m.group(5)), int(m.group(6)),
               int(c.group(1)), int(c.group(2)), int(c.group(3)), int(c.group(4)))
        want = (n_isl, total['cells'], total['overlap'], total['fold'], total['repeat'], total['other'], spanning)
        check(f'{name}: islands, cells, overlap, fold, repeat, other, spanning match the reference {want}',
              got == want, f'got {got}')
        islands = {int(g[0]): tuple(int(x) for x in g[3:6]) for g in ISLAND.findall(out)}
        for i, cats in enumerate(per):
            if any(cats.values()):
                check(f'{name}: island {i} categories', islands.get(i) == (cats['fold'], cats['repeat'], cats['other']),
                      f'got {islands.get(i)}, want {cats}')


def test_real_model_islands_match_rebake():
    if not VRAD.is_file():
        return _skip(f"not built: {VRAD.name}")
    if not (GAME / 'gameinfo.txt').is_file():
        return _skip(f"no gameinfo.txt in {GAME}")
    rc, out = run_vrad('models/props_wasteland/kitchen_fridge001a.mdl', 512)
    m = SUMMARY.search(out)
    if rc != 0 and 'Unable to load model' in out:
        return _skip('kitchen_fridge001a is not mounted')
    check('fridge: vrad -uvoverlap reports', rc == 0 and m, out[-400:])
    check('fridge: 89 islands, as the VBSP rebake finds', int(m.group(2)) == 89, m.group(0))
    check('fridge: its islands overlap', int(m.group(6)) > 0, m.group(0))


TESTS = [test_fixtures_match_exact_reference, test_real_model_islands_match_rebake]


if __name__ == '__main__':
    if '--game' in sys.argv:
        GAME = Path(sys.argv[sys.argv.index('--game') + 1])
    sys.exit(run_all('test_proplm_uvoverlap', TESTS))
