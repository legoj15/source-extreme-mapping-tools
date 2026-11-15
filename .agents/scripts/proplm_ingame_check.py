"""In-game check of static prop lighting routes on a stock Source engine.

Builds a two-room test map: room A holds each model with no proplm key, room B
the same models at the same spots (rooms and lights identical), marked by
--mark-b (default proplm "rebake"). Compiles it with the given VBSP arguments,
vvis and vrad (-StaticPropLighting unless --no-static-prop-lighting), then runs
it on the SDK's hl2_win64.exe and takes one screenshot per model per room from
the same relative camera, plus an overview per room, saved to --out as
<label>_<room>_<n>_<model>.tga.

  python -I .agents/scripts/proplm_ingame_check.py --sdk <SDK root> --mod <mod dir>
      --out DIR --label NAME [--vbsp EXE] [--vbsp-args "..."] [--fullbright]
      [--lod N] [--mark-b rebake|none] [--no-game] model[:camera distance]...

Source SDK Base 2013 Multiplayer ships no game DLLs, so --mod names a mod folder
to run it with: a gameinfo.txt that mounts the SDK's hl2mp content and puts the
folder itself on the GAME path, and bin/x64/client.dll + server.dll (HL2DM's
work). The game is driven with -hijack (one command batch per shot, real-time
pauses), then killed. The map is compiled against <sdk>/hl2mp; nothing is
written into the SDK. Tools come from game/bin/x64 of this checkout (--bin).
"""
import argparse
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BIN = ROOT / 'game' / 'bin' / 'x64'

ROOM_HALF_X = 650      # interior half width of each room
ROOM_Y = (-416, 416)
ROOM_Z = (0, 400)
WALL = 16
CENTER = {'a': -700, 'b': 700}
PROP_Y = 150
SPACING = 200

_id = [100]


def nid():
    _id[0] += 1
    return _id[0]


def box(x0, y0, z0, x1, y1, z1, mat):
    faces = [
        ((x0, y1, z1), (x1, y1, z1), (x1, y0, z1), '[1 0 0 0] 0.25', '[0 -1 0 0] 0.25'),
        ((x0, y0, z0), (x1, y0, z0), (x1, y1, z0), '[1 0 0 0] 0.25', '[0 -1 0 0] 0.25'),
        ((x0, y1, z1), (x0, y0, z1), (x0, y0, z0), '[0 1 0 0] 0.25', '[0 0 -1 0] 0.25'),
        ((x1, y1, z0), (x1, y0, z0), (x1, y0, z1), '[0 1 0 0] 0.25', '[0 0 -1 0] 0.25'),
        ((x1, y1, z1), (x0, y1, z1), (x0, y1, z0), '[1 0 0 0] 0.25', '[0 0 -1 0] 0.25'),
        ((x1, y0, z0), (x0, y0, z0), (x0, y0, z1), '[1 0 0 0] 0.25', '[0 0 -1 0] 0.25'),
    ]
    out = [f'\tsolid\n\t{{\n\t\t"id" "{nid()}"\n']
    for a, b, c, u, v in faces:
        pts = ' '.join('(%g %g %g)' % p for p in (a, b, c))
        out.append(f'\t\tside\n\t\t{{\n\t\t\t"id" "{nid()}"\n\t\t\t"plane" "{pts}"\n'
                   f'\t\t\t"material" "{mat}"\n\t\t\t"uaxis" "{u}"\n\t\t\t"vaxis" "{v}"\n'
                   f'\t\t\t"rotation" "0"\n\t\t\t"lightmapscale" "16"\n'
                   f'\t\t\t"smoothing_groups" "0"\n\t\t}}\n')
    out.append('\t}\n')
    return ''.join(out)


def entity(kv):
    body = ''.join(f'\t"{k}" "{v}"\n' for k, v in kv.items())
    return f'entity\n{{\n\t"id" "{nid()}"\n{body}}}\n'


def prop_x(room, i, n):
    return CENTER[room] + (i - (n - 1) / 2.0) * SPACING


def make_vmf(models, mark_b, lmres):
    wall = 'DEV/DEV_MEASUREWALL01A'
    floor = 'DEV/DEV_MEASUREGENERIC01B'
    x0, x1 = CENTER['a'] - ROOM_HALF_X, CENTER['b'] + ROOM_HALF_X
    y0, y1 = ROOM_Y
    z0, z1 = ROOM_Z
    solids = [
        box(x0 - WALL, y0 - WALL, z0 - WALL, x1 + WALL, y1 + WALL, z0, floor),
        box(x0 - WALL, y0 - WALL, z1, x1 + WALL, y1 + WALL, z1 + WALL, wall),
        box(x0 - WALL, y0 - WALL, z0, x0, y1 + WALL, z1, wall),
        box(x1, y0 - WALL, z0, x1 + WALL, y1 + WALL, z1, wall),
        box(x0, y0 - WALL, z0, x1, y0, z1, wall),
        box(x0, y1, z0, x1, y1 + WALL, z1, wall),
        # the divider: the rooms do not light each other
        box(CENTER['a'] + ROOM_HALF_X, y0, z0, CENTER['b'] - ROOM_HALF_X, y1, z1, wall),
    ]
    world = ('world\n{\n\t"id" "1"\n\t"mapversion" "1"\n\t"classname" "worldspawn"\n'
             '\t"skyname" "sky_day01_01"\n' + ''.join(solids) + '}\n')
    ents = []
    for room in ('a', 'b'):
        c = CENTER[room]
        ents.append(entity({'classname': 'light', 'origin': f'{c} -250 300',
                            '_light': '160 180 255 120', '_quadratic_attn': '1'}))
        for i, (m, dist) in enumerate(models):
            # a warm key light low and to the right of each prop: strong
            # gradients across the prop make a scrambled lightmap obvious
            ents.append(entity({'classname': 'light',
                                'origin': f'{prop_x(room, i, len(models)) + dist * 0.5:g} '
                                          f'{PROP_Y - dist * 0.6:g} {dist * 0.5:g}',
                                '_light': '255 200 150 %d' % max(40, int(dist * 0.9)),
                                '_quadratic_attn': '1'}))
            kv = {'classname': 'prop_static', 'angles': '0 270 0',
                  'origin': f'{prop_x(room, i, len(models)):g} {PROP_Y} 0',
                  'model': m, 'solid': '0', 'skin': '0', 'generatelightmaps': '1',
                  'lightmapresolutionx': str(lmres), 'lightmapresolutiony': str(lmres),
                  'disableshadows': '1', 'fademindist': '-1', 'fademaxdist': '0'}
            if room == 'b' and mark_b != 'none':
                kv['proplm'] = mark_b
            ents.append(entity(kv))
    ents.append(entity({'classname': 'info_player_deathmatch', 'origin': f'{CENTER["a"]} -300 8',
                        'angles': '0 90 0'}))
    ents.append(entity({'classname': 'info_player_start', 'origin': f'{CENTER["a"]} -300 8',
                        'angles': '0 90 0'}))
    return world + ''.join(ents)


def shot_list(models):
    """(room, index, model, setpos, setang) per screenshot."""
    out = []
    for room in ('a', 'b'):
        for i, (m, dist) in enumerate(models):
            x = prop_x(room, i, len(models))
            out.append((room, i, m, f'{x:g} {PROP_Y - dist:g} {dist * 0.55:g}', '25 90 0'))
    for room in ('a', 'b'):
        out.append((room, 99, 'overview', f'{CENTER[room]} -400 220', '18 90 0'))
    return out


def run(cmd, log):
    with open(log, 'ab') as f:
        f.write(('\n$ ' + ' '.join(map(str, cmd)) + '\n').encode())
        f.flush()
        r = subprocess.run([str(c) for c in cmd], stdout=f, stderr=subprocess.STDOUT)
    if r.returncode:
        sys.exit(f'{cmd[0]} failed ({r.returncode}); see {log}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sdk', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--label', required=True)
    ap.add_argument('--bin', default=str(BIN))
    ap.add_argument('--vbsp', default=None, help='VBSP exe (default <bin>/vbsp_lmo.exe)')
    ap.add_argument('--vbsp-args', default='')
    ap.add_argument('--no-static-prop-lighting', action='store_true')
    ap.add_argument('--fullbright', action='store_true')
    ap.add_argument('--lod', type=int, default=-1, help='r_lod for the shots (-1 = auto)')
    ap.add_argument('--mark-b', default='rebake')
    ap.add_argument('--lmres', type=int, default=256)
    ap.add_argument('--timeout', type=int, default=300)
    ap.add_argument('--no-game', action='store_true', help='compile only')
    ap.add_argument('--mod', help='mod folder to run (gameinfo + bin/x64 game DLLs)')
    ap.add_argument('models', nargs='+')
    a = ap.parse_args()

    sdk = Path(a.sdk)
    game = sdk / 'hl2mp'
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    mapname = 'plm_' + a.label
    work = out / mapname
    work.mkdir(exist_ok=True)
    vmf = work / f'{mapname}.vmf'
    log = work / 'compile.log'
    if log.exists():
        log.unlink()
    # model[:camera distance], e.g. props_wasteland/gear01:40
    models = []
    for spec in a.models:
        m, _, d = spec.partition(':')
        m = m if m.startswith('models/') else f'models/{m}'
        m = m if m.endswith('.mdl') else m + '.mdl'
        models.append((m, float(d) if d else 120.0))
    vmf.write_text(make_vmf(models, a.mark_b, a.lmres))

    binp = Path(a.bin)
    vbsp = Path(a.vbsp) if a.vbsp else binp / 'vbsp_lmo.exe'
    run([vbsp, '-game', game, *a.vbsp_args.split(), vmf], log)
    run([binp / 'vvis_optix.exe', '-game', game, '-fast', vmf.with_suffix('.bsp')], log)
    vrad = [binp / 'vrad_rtx.exe', '-game', game]
    if not a.no_static_prop_lighting:
        vrad.append('-StaticPropLighting')
    run(vrad + [vmf.with_suffix('.bsp')], log)
    if a.no_game:
        return

    # The SDK base ships no game DLLs: run it on --mod, a mod folder whose
    # gameinfo mounts the SDK content and whose bin/x64 holds client/server.
    mod = Path(a.mod)
    (mod / 'maps').mkdir(exist_ok=True)
    (mod / 'cfg').mkdir(exist_ok=True)
    shutil.copy2(vmf.with_suffix('.bsp'), mod / 'maps' / f'{mapname}.bsp')
    sdir = mod / 'screenshots'
    sdir.mkdir(exist_ok=True)
    for p in sdir.glob(f'{mapname}*.tga'):
        p.unlink()
    con = mod / 'console.log'
    con.unlink(missing_ok=True)
    exe = str(sdk / 'hl2_win64.exe')
    base = [exe, '-game', str(mod)]

    def send(cfg_lines):
        (mod / 'cfg' / 'plm_hijack.cfg').write_text('\n'.join(cfg_lines) + '\n')
        subprocess.run(base + ['-hijack', '+exec', 'plm_hijack'], cwd=str(sdk),
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=60)

    proc = subprocess.Popen(base + ['-novid', '-windowed', '-noborder', '-w', '1280', '-h', '720',
                                    '-condebug', '-insecure', '+maxplayers', '1',
                                    '+map', mapname], cwd=str(sdk))
    plan = shot_list(models)
    got = 0
    try:
        t0 = time.time()
        while time.time() - t0 < a.timeout:
            time.sleep(2)
            if con.exists() and 'Redownloading all lightmaps' in con.read_text(errors='replace'):
                break
        else:
            sys.exit('the map did not load; see ' + str(con))
        time.sleep(8)
        send(['sv_cheats 1'])
        time.sleep(2)
        send(['cl_drawhud 0', 'r_drawviewmodel 0', 'crosshair 0', 'hidepanel all',
              'hideconsole', 'noclip', 'mat_fullbright %d' % (1 if a.fullbright else 0),
              'r_lod %d' % a.lod])
        time.sleep(3)
        for room, i, m, pos, ang in plan:
            send([f'setpos {pos}', f'setang {ang}'])
            time.sleep(3)
            send(['screenshot'])
            time.sleep(2)
            got += 1
    finally:
        subprocess.run(['taskkill', '/F', '/T', '/PID', str(proc.pid)],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1)
    if con.exists():
        shutil.copy2(con, work / 'console.log')
    shots = sorted(sdir.glob(f'{mapname}*.tga'))
    for p, (room, i, m, _, _) in zip(shots, plan):
        shutil.move(str(p), str(out / f'{a.label}_{room}_{i:02d}_{Path(m).stem}.tga'))
    (mod / 'maps' / f'{mapname}.bsp').unlink(missing_ok=True)
    print(f'{len(shots)} of {len(plan)} screenshots in {out}')


if __name__ == '__main__':
    main()
