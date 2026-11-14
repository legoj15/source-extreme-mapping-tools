"""Compare the per-texel static prop lightmaps (texelslighting_N.ppl) that
different VRAD builds bake from the same BSP.

  python -I .agents/scripts/proplm_ppl_ab.py --game DIR --out DIR
         --vrad LABEL=BINDIR [--vrad LABEL=BINDIR ...] [--vbsp EXE] [--vbsp-arg=ARG]
         [--lmres N] [--bounce N] [--vvis] [--lights INTENSITY] [--coverage]
         model[:plain] [model[:plain] ...]

Compiles the staticpropbounce fixture once with VBSP, one prop_static per model
(each marked proplm "rebake" unless written model:plain; plain stock props need
--vbsp-arg=-forcedynamicpropsasstatic). Then, for every --vrad build, copies that
BSP, runs BINDIR/vrad_rtx.exe on it with -StaticPropLighting (and -bounce N),
reads the LOD 0 texel image of every mesh of every prop from the output BSP's pak
(texelslighting_N.ppl) and compares every pair of builds:

  * texels non-black in each build, texels lit only in one build, and for the
    texels lit in both the largest and the mean per-channel byte difference
    (the .ppl bytes, 0..255) and how many differ at all;
  * the same split by distance: a texel within one texel of a texel lit only in
    the later build ("touching": the 3x3 filter of the encoder reaches it) and
    the rest ("clear");
  * the "[proptexel]" lines VRAD prints (texels it lit per prop).

Label one build twice (new, new2 with the same BINDIR) to get the run-to-run
noise floor of the bake: direct light only is deterministic, but with --vvis and
--bounce 2 a few texels flip by one level between runs of the same build.
--vvis runs vvis_optix.exe from the VBSP's folder on the BSP first (without it VRAD
says "No vis information, direct lighting only"). --lights adds point lights on all
sides of every prop: with the fixture's one light most texels of a prop face away from
it and stay black, so "non-black" would say little about which texels got a sample.
--coverage re-derives the texels VRAD lists for lighting from the rebaked model in the
pak (LOD 0 only, then with the lower LODs, #126), checks the counts against what VRAD
printed and the baked difference against the predicted newly listed texels, and says how
many texels only lower-LOD triangles cover are still never listed. Needs numpy.
Default VBSP is game/bin/x64/vbsp_lmo.exe of this checkout. SEMT_REPO overrides the
checkout root (to run a copy of this script elsewhere).
"""
import argparse
import io
import os
import re
import struct
import subprocess
import sys
import zipfile
from pathlib import Path

import numpy as np

ROOT = Path(os.environ.get('SEMT_REPO') or Path(__file__).resolve().parents[2])
BIN = ROOT / 'game' / 'bin' / 'x64'
sys.path.insert(0, str(BIN))  # python -I drops this script's folder
import sprp_lightmap  # noqa: E402  (parse() lists a BSP's static props)

FIXTURE = BIN / 'bsp_unit_tests' / 'validation_vrad_staticpropbounce.vmf'
LUMP_PAKFILE = 40
STEP_X = 96
MAP = 'proplm_ppl_ab'


def entity_block(text, classname):
    i = text.index(f'"classname" "{classname}"')
    start = text.rfind('entity', 0, i)
    depth, j = 0, text.index('{', start)
    while True:
        if text[j] == '{':
            depth += 1
        elif text[j] == '}':
            depth -= 1
            if depth == 0:
                return start, j + 1
        j += 1


LIGHT_OFFSETS = [(70, 0, 40), (-70, 0, 40), (0, 70, 40), (0, -70, 40), (0, 0, 130)]


def light_entity(ident, x, y, z, intensity):
    return ['entity', '{', f'\t"id" "{ident}"', '\t"classname" "light"', f'\t"_light" "255 255 255 {intensity}"',
            '\t"_quadratic_attn" "1"', f'\t"origin" "{x} {y} {z}"', '}']


def make_vmf(dst, specs, lmres, lights):
    text = FIXTURE.read_bytes().decode('utf-8')
    nl = '\r\n' if '\r\n' in text else '\n'
    a, b = entity_block(text, 'prop_static')
    block = text[a:b]
    out = []
    for k, (model, rebake) in enumerate(specs):
        e = block
        if rebake:
            e = e.replace('"classname" "prop_static"', '"classname" "prop_static"' + nl + '\t"proplm" "rebake"')
        e = re.sub(r'"model" "[^"]*"', f'"model" "{model}"', e)
        e = re.sub(r'"origin" "[^"]*"', f'"origin" "{-404 + STEP_X * k} 431 0"', e)
        e = re.sub(r'"id" "\d+"', f'"id" "{900000 + k}"', e, count=1)
        if lmres:
            e = re.sub(r'"lightmapresolution([xy])" "[^"]*"', rf'"lightmapresolution\1" "{lmres}"', e)
        out.append(e)
    text = text[:a] + nl.join(out) + text[b:]
    if lights:  # point lights around every prop, so no face of a prop stays unlit
        extra = [line for k in range(len(specs)) for n, (dx, dy, dz) in enumerate(LIGHT_OFFSETS)
                 for line in light_entity(800000 + 10 * k + n, -404 + STEP_X * k + dx, 431 + dy, dz, lights)]
        text += nl + nl.join(extra) + nl
    dst.write_bytes(text.encode('utf-8'))


def run(cmd, log, cwd=None, timeout=1800):
    p = subprocess.run([str(c) for c in cmd], cwd=cwd, capture_output=True, text=True, errors='replace',
                       timeout=timeout)
    text = p.stdout + p.stderr
    Path(log).write_text(text, encoding='utf-8')
    return p.returncode, text


def read_ppls(bsp_path):
    """{prop index: [(w, h, ndarray HxWx3 uint8) per LOD 0 mesh entry]}, and the sprp props."""
    data = bsp_path.read_bytes()
    ofs, length = struct.unpack_from('<ii', data, 8 + LUMP_PAKFILE * 16)
    z = zipfile.ZipFile(io.BytesIO(data[ofs:ofs + length]))
    files = {n.lower(): n for n in z.namelist()}
    props = sprp_lightmap.parse(data)
    out = {}
    for pr in props:
        name = f'texelslighting_{pr["index"]}.ppl'
        if name not in files:
            continue
        buf = z.read(files[name])
        version, checksum, fmt, nmesh = struct.unpack_from('<iIIi', buf, 0)
        assert fmt == 2, f'{name}: texel format {fmt}, not RGB888'
        imgs = []
        for k in range(nmesh):
            lod, o, nbytes, w, h = struct.unpack_from('<5I', buf, 32 + k * 32)
            if lod == 0:
                imgs.append((w, h, np.frombuffer(buf, np.uint8, w * h * 3, o).reshape(h, w, 3).copy()))
        out[pr['index']] = imgs
    return out, props


def dilate(mask, n):
    out = mask.copy()
    for _ in range(n):
        p = np.pad(out, 1)
        out = np.zeros_like(mask)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                out |= p[1 + dy:1 + dy + mask.shape[0], 1 + dx:1 + dx + mask.shape[1]]
    return out


def zone_stats(diff_max, diff_mean, sel):
    n = int(sel.sum())
    if not n:
        return 'none'
    dm = diff_max[sel]
    return (f'{n} texels, max {int(dm.max())}, mean {float(diff_mean[sel].mean()):.4f}, '
            f'{int((dm > 0).sum())} differ')


def compare(a, b):
    """Per-mesh comparison text lines of two LOD 0 images (a = earlier build, b = later)."""
    lit_a, lit_b = a.any(axis=2), b.any(axis=2)
    only_b, only_a, both = lit_b & ~lit_a, lit_a & ~lit_b, lit_a & lit_b
    ai, bi = a.astype(np.int16), b.astype(np.int16)
    d = np.abs(ai - bi)
    diff_max, diff_mean = d.max(axis=2), d.mean(axis=2)
    touching = dilate(only_b, 1)
    return [
        f'lit: {int(lit_a.sum())} -> {int(lit_b.sum())}; only later {int(only_b.sum())}, only earlier {int(only_a.sum())}, '
        f'both {int(both.sum())}',
        f'both, all:      {zone_stats(diff_max, diff_mean, both)}',
        f'both, touching: {zone_stats(diff_max, diff_mean, both & touching)}  (within 1 texel of an only-later texel)',
        f'both, clear:    {zone_stats(diff_max, diff_mean, both & ~touching)}',
        f'newly lit texels, channel max: {int(b[only_b].max()) if only_b.any() else 0}, '
        f'mean of the three channels {float(b[only_b].mean()) if only_b.any() else 0.0:.2f}',
    ]


# ---- independent coverage model of VRAD's texel rasterization (--coverage) -------------------
# Re-derives, from the rebaked copy model in the BSP's pak, which texels
# GenerateLightmapSamplesForMesh marks valid / possibly interesting / lists for
# lighting, before (LOD 0 only) and after (#126: the lower LODs too), so the
# counts VRAD prints can be checked without trusting VRAD.
LOD_OFF_SWITCH = 1.0e7


def parse_mdl_meshes(b):
    """[[(vertex index base of the model, vertexoffset of the mesh) per mesh] per model], in (bodypart, model) order."""
    numbody, bodyidx = struct.unpack_from('<2i', b, 232)
    out = []
    for bp in range(numbody):
        o = bodyidx + bp * 16
        _, nummodels, _, modelidx = struct.unpack_from('<4i', b, o)
        for m in range(nummodels):
            mo = o + modelidx + m * 148
            nummeshes, meshidx, _, vertidx = struct.unpack_from('<4i', b, mo + 72)
            out.append([(vertidx // 48, struct.unpack_from('<i', b, mo + meshidx + k * 116 + 12)[0])
                        for k in range(nummeshes)])
    return out


def parse_vvd_uv(b):
    """UV per flat LOD 0 vertex index."""
    lod0, = struct.unpack_from('<i', b, 16)
    numfix, fixstart, vertstart, _ = struct.unpack_from('<4i', b, 48)
    uv = [struct.unpack_from('<2f', b, vertstart + i * 48 + 40) for i in range(lod0)]
    flat = []
    for f in range(numfix):
        _, src, cnt = struct.unpack_from('<3i', b, fixstart + f * 12)
        flat.extend(range(src, src + cnt))
    return uv, flat or list(range(lod0))


def mesh_lod_triangles(mdl, vvd, vtx):
    """[{lod: (switch point, [triangle UVs in VTX order])} per mesh], VRAD's traversal."""
    models = parse_mdl_meshes(mdl)
    uv, flat = parse_vvd_uv(vvd)
    result = [dict() for model in models for _ in model]
    num_body, body_ofs = struct.unpack_from('<ii', vtx, 28)
    ordinal = 0
    model_no = 0
    for bp in range(num_body):
        body = body_ofs + bp * 8
        num_models, model_ofs = struct.unpack_from('<ii', vtx, body)
        for mi in range(num_models):
            model = body + model_ofs + mi * 8
            num_lods, lod_ofs = struct.unpack_from('<ii', vtx, model)
            first = ordinal
            for lod_i in range(num_lods):
                lod = model + lod_ofs + lod_i * 12
                num_meshes, mesh_ofs = struct.unpack_from('<ii', vtx, lod)
                switch, = struct.unpack_from('<f', vtx, lod + 8)
                for k in range(min(num_meshes, len(models[model_no]))):
                    base, voff = models[model_no][k]
                    mesh = lod + mesh_ofs + k * 9
                    num_groups, group_ofs = struct.unpack_from('<ii', vtx, mesh)
                    tris = []
                    for g in range(num_groups):
                        group = mesh + group_ofs + g * 25
                        _, vo, _, io_, ns, so = struct.unpack_from('<6i', vtx, group)
                        for s in range(ns):
                            strip = group + so + s * 27
                            sni, sio = struct.unpack_from('<ii', vtx, strip)
                            if not vtx[strip + 18] & 1:
                                continue
                            for t in range(0, sni - 2, 3):
                                corners = []
                                for c in range(3):
                                    idx, = struct.unpack_from('<H', vtx, group + io_ + (sio + t + c) * 2)
                                    orig, = struct.unpack_from('<H', vtx, group + vo + idx * 9 + 4)
                                    corners.append(uv[flat[base + voff + orig]])
                                tris.append(corners)
                    result[first + k][lod_i] = (switch, tris)
            ordinal += len(models[model_no])
            model_no += 1
    return result


def raster_triangle(corners, res):
    """(iy0, ix0, inside mask over the visited box) of VRAD's Rasterizer for one triangle, or None."""
    (x0, y0), (x1, y1), (x2, y2) = corners
    fminx, fmaxx = min(x0, x1, x2), max(x0, x1, x2)
    fminy, fmaxy = min(y0, y1, y2), max(y0, y1, y2)
    if fminx == fmaxx or fminy == fmaxy:
        return None
    cross2 = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0)
    if abs(cross2) * res * res < 1e-6:
        return None
    fminx, fminy, fmaxx, fmaxy = max(0.0, fminx), max(0.0, fminy), min(1.0, fmaxx), min(1.0, fmaxy)
    ix0, iy0 = max(0, int(fminx * res) - 1), max(0, int(fminy * res) - 1)
    ix1, iy1 = min(int(fmaxx * res) + 2, res - 1), min(int(fmaxy * res) + 2, res - 1)
    if ix1 < ix0 or iy1 < iy0:
        return None
    px = (np.arange(ix0, ix1 + 1) + 0.5) / res
    py = (np.arange(iy0, iy1 + 1) + 0.5) / res
    X, Y = np.meshgrid(px, py)
    ax, ay, bx, by = x1 - x0, y1 - y0, x2 - x0, y2 - y0
    daa, dab, dbb = ax * ax + ay * ay, ax * bx + ay * by, bx * bx + by * by
    inv = 1.0 / (daa * dbb - dab * dab)
    cx, cy = X - x0, Y - y0
    dca, dcb = cx * ax + cy * ay, cx * bx + cy * by
    by_ = (dbb * dca - dab * dcb) * inv
    bz = (daa * dcb - dab * dca) * inv
    bx_ = 1.0 - by_ - bz
    inside = (bx_ >= 0) & (bx_ <= 1) & (by_ >= 0) & (by_ <= 1) & (bz >= 0) & (bz <= 1)
    return iy0, ix0, inside


def simulate_texels(lods, res, with_lower):
    """(valid, possibly interesting, listed for lighting) masks, after LOD 0 and, when with_lower, the lower LODs."""
    valid = np.zeros((res, res), bool)
    interesting = np.zeros((res, res), bool)
    for lod_i in sorted(lods):
        switch, tris = lods[lod_i]
        if lod_i and (not with_lower or switch < 0.0 or switch >= LOD_OFF_SWITCH):
            continue
        for corners in tris:
            r = raster_triangle(corners, res)
            if r is None:
                continue
            y0, x0, inside = r
            h, w = inside.shape
            v, p = valid[y0:y0 + h, x0:x0 + w], interesting[y0:y0 + h, x0:x0 + w]
            if lod_i == 0:
                v |= inside          # a valid texel is never touched again; the rest gains validity inside
                p[:] = True
            else:
                free = ~p            # only texels no pass has written yet
                v[free] = inside[free]
                p[free] = True
    listed = valid | (interesting & dilate(valid, 1))
    return valid, interesting, listed


def coverage_report(bsp_path, props, specs, printed):
    data = bsp_path.read_bytes()
    ofs, length = struct.unpack_from('<ii', data, 8 + LUMP_PAKFILE * 16)
    z = zipfile.ZipFile(io.BytesIO(data[ofs:ofs + length]))
    files = {n.lower(): n for n in z.namelist()}
    print('\n=== independent coverage model (texels listed for lighting; VRAD printed in brackets)')
    sims = {}
    for pr in props:
        spec = specs[round((pr['origin'][0] + 404) / STEP_X)]
        base = pr['model'][:-4].lower()
        if f'{base}.dx80.vtx' not in files:
            print(f'prop {pr["index"]} {spec[0]}: model {pr["model"]} is not in the pak (plain prop), skipped')
            continue
        mdl, vvd, vtx = (z.read(files[f'{base}{ext}']) for ext in ('.mdl', '.vvd', '.dx80.vtx'))
        res = pr['res'][0]
        meshes = mesh_lod_triangles(mdl, vvd, vtx)
        tot_old = tot_new = 0
        for m, lods in enumerate(meshes):
            v0, p0, l0 = simulate_texels(lods, res, False)
            v1, p1, l1 = simulate_texels(lods, res, True)
            tot_old += int(l0.sum())
            tot_new += int(l1.sum())
            sims[(pr['index'], m)] = (l0, l1)
            only_lower = np.zeros_like(v0)  # texels inside a lower-LOD triangle and inside no LOD 0 triangle
            for lod_i, (switch, tris) in lods.items():
                if lod_i == 0 or switch < 0.0 or switch >= LOD_OFF_SWITCH:
                    continue
                for corners in tris:
                    r = raster_triangle(corners, res)
                    if r is not None:
                        y0, x0, inside = r
                        only_lower[y0:y0 + inside.shape[0], x0:x0 + inside.shape[1]] |= inside
            only_lower &= ~v0
            never = only_lower & ~l1
            print(f'prop {pr["index"]} {spec[0]} mesh {m}: LODs {sorted(lods)} switch points '
                  f'{[round(lods[i][0], 1) for i in sorted(lods)]}')
            print(f'    listed: LOD 0 only {int(l0.sum())}, with lower LODs {int(l1.sum())} '
                  f'(+{int(l1.sum() - l0.sum())}); texels inside a lower-LOD triangle and no LOD 0 triangle: '
                  f'{int(only_lower.sum())}, listed before {int((only_lower & l0).sum())}, '
                  f'listed after {int((only_lower & l1).sum())}, never listed {int(never.sum())} '
                  f'({100.0 * int(never.sum()) / max(1, int(only_lower.sum())):.1f}%)')
            print(f'    never listed: {int((never & p0).sum())} lie in the box (+1 texel) of a LOD 0 triangle, '
                  f'{int((never & ~p0).sum())} were claimed first by a lower-LOD triangle that does not contain them')
        want = printed.get(pr['index'])
        print(f'  prop {pr["index"]} total listed: model old {tot_old}, new {tot_new}; '
              f'VRAD printed {want if want else "?"}')
    return sims


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--game', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--vrad', action='append', required=True, help='LABEL=BINDIR holding vrad_rtx.exe')
    ap.add_argument('--vbsp', default=str(BIN / 'vbsp_lmo.exe'))
    ap.add_argument('--vbsp-arg', action='append', default=[], help='extra VBSP argument, e.g. -forcedynamicpropsasstatic')
    ap.add_argument('--lmres', type=int, default=0)
    ap.add_argument('--bounce', type=int, default=0)
    ap.add_argument('--vvis', action='store_true')
    ap.add_argument('--coverage', action='store_true',
                    help='also re-derive the listed texels from the model in the pak and check them against '
                         'what VRAD printed and what the first two builds baked')
    ap.add_argument('--lights', type=int, default=0, metavar='INTENSITY',
                    help='add five quadratic point lights of this intensity around every prop (e.g. 2000)')
    ap.add_argument('models', nargs='+')
    a = ap.parse_args()

    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    specs = []
    for m in a.models:
        model, _, mode = m.partition(':')
        specs.append((model, mode != 'plain'))
    vmf = out / f'{MAP}.vmf'
    make_vmf(vmf, specs, a.lmres, a.lights)
    rc, text = run([a.vbsp, '-game', a.game, *a.vbsp_arg, vmf], out / 'vbsp.log')
    print(f'VBSP {a.vbsp}: exit {rc}')
    for line in text.splitlines():
        if 'proplm' in line and ('rebake' in line or 'Warning' in line):
            print('  ' + line.strip()[:240])
    if rc:
        return 1
    bsp0 = out / f'{MAP}.bsp'
    if a.vvis:
        vvis = Path(a.vbsp).parent / 'vvis_optix.exe'
        rc, text = run([vvis, '-game', a.game, bsp0], out / 'vvis.log', cwd=vvis.parent)
        print(f'VVIS {vvis}: exit {rc}')
        if rc:
            return 1

    builds = []  # (label, images, props)
    printed = {}  # label -> {prop index: texels VRAD says it lit}
    for spec in a.vrad:
        label, _, bindir = spec.partition('=')
        bsp = out / f'{label}.bsp'
        bsp.write_bytes(bsp0.read_bytes())
        exe = Path(bindir) / 'vrad_rtx.exe'
        rc, text = run([exe, '-game', a.game, '-StaticPropLighting', '-bounce', a.bounce, bsp], out / f'{label}.log',
                       cwd=bindir)
        lines = [l.strip() for l in text.splitlines() if l.startswith('[proptexel]')]
        printed[label] = {int(m.group(1)): int(m.group(2)) for m in
                          (re.match(r'\[proptexel\] prop (\d+) .*?, (\d+) texels lit', l) for l in lines) if m}
        print(f'VRAD {label} ({exe}): exit {rc}')
        for l in lines:
            print('  ' + l[:200])
        if rc:
            print(text[-1500:])
            return 1
        imgs, props = read_ppls(bsp)
        builds.append((label, imgs, props))

    props = builds[0][2]
    for i in range(len(builds)):
        for j in range(i + 1, len(builds)):
            la, ia, _ = builds[i]
            lb, ib, _ = builds[j]
            print(f'\n=== {la} (earlier) vs {lb} (later)')
            for pr in props:
                k = pr['index']
                if k not in ia or k not in ib:
                    print(f'prop {k} {pr["model"]}: no .ppl in {"both" if k not in ia and k not in ib else la if k not in ia else lb}')
                    continue
                spec = specs[round((pr['origin'][0] + 404) / STEP_X)]
                print(f'prop {k} {spec[0]} ({"rebake" if spec[1] else "plain"}) '
                      f'{pr["res"][0]}x{pr["res"][1]}, {len(ia[k])} LOD 0 mesh entries')
                if len(ia[k]) != len(ib[k]):
                    print(f'  mesh entry counts differ: {len(ia[k])} vs {len(ib[k])}')
                    continue
                for m, ((w, h, ta), (w2, h2, tb)) in enumerate(zip(ia[k], ib[k])):
                    if (w, h) != (w2, h2):
                        print(f'  mesh {m}: size differs')
                        continue
                    for line in compare(ta, tb):
                        print(f'  mesh {m}  {line}')

    if a.coverage:
        first, second = builds[0][0], builds[1][0]
        say = {k: f'{first} {printed[first].get(k)}, {second} {printed[second].get(k)}' for k in printed[first]}
        sims = coverage_report(out / f'{first}.bsp', props, specs, say)
        print(f'\n=== baked difference {first} -> {second} against the predicted newly listed texels')
        for (k, m), (l0, l1) in sorted(sims.items()):
            ta, tb = builds[0][1][k][m][2], builds[1][1][k][m][2]
            only_b = tb.any(axis=2) & ~ta.any(axis=2)
            new = l1 & ~l0
            print(f'prop {k} mesh {m}: predicted newly listed {int(new.sum())}, black before and non-black after '
                  f'{int(only_b.sum())}, of which predicted {int((only_b & new).sum())}; '
                  f'predicted but still black after {int((new & ~tb.any(axis=2)).sum())}; '
                  f'listed before but black before and after (unlit by the lights): {int((l0 & ~ta.any(axis=2) & ~tb.any(axis=2)).sum())}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
