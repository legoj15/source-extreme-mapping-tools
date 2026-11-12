"""Detail prop test maps for VBSP (#111, #112, #113).

  gen <out_dir> [--longname]   write a scratch game dir and a test VMF
  dump <bsp>                   print the detail prop lump (dict + objects)
  diff <a.bsp> <b.bsp>         compare two maps' detail props as multisets

`gen` writes <out_dir>/game (gameinfo.txt mounting Source SDK Base 2013 MP's
hl2mp content by absolute path, a %detailtype material, detail.vbsp) and
<out_dir>/detailtest.vmf: a sealed 2048x1024 room whose floor is a grass brush
on the left half and four sewn power-4 grass displacements on the right half.
The displacements' heights are one function of world xy, so neighbours meet
exactly at seams. --longname adds a grass patch whose detail type names a model
path of 128+ characters (the model files are extracted from hl2_misc_dir.vpk).

Compile with -game <out_dir>/game.
"""
import argparse
import collections
import math
import os
import struct
import sys

SDK = r"E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer"
GRASS_MDL = "models/perftest/grass_tuft_001a"
# One directory padded so the model name is exactly DETAIL_NAME_LENGTH (128)
# characters: the boundary case, and short enough for MAX_PATH from a deep dir.
LONG_DIR = "models/" + "detail_name_length_test_".ljust(101, "x")

# ------------------------------------------------------------------ game dir


def write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="\n") as f:
        f.write(text)


def write_game(game, longname):
    sdk = SDK.replace("\\", "/")
    paths = [
        "hl2mp/hl2mp_english.vpk", "hl2mp/hl2mp_pak.vpk",
        "hl2/hl2_textures.vpk", "hl2/hl2_misc.vpk",
        "platform/platform_misc.vpk", "hl2mp", "hl2", "platform",
    ]
    lines = ["\t\t\tgame+mod\t|gameinfo_path|."]
    lines += ['\t\t\tgame\t"%s/%s"' % (sdk, p) for p in paths]
    write(os.path.join(game, "gameinfo.txt"),
          '"GameInfo"\n{\n\tgame "detailtest"\n\tFileSystem\n\t{\n'
          "\t\tSteamAppId 243750\n\t\tSearchPaths\n\t\t{\n"
          + "\n".join(lines) + "\n\t\t}\n\t}\n}\n")

    def vmt(detailtype):
        # WorldVertexTransition with an env_cubemap, as real blend grass can
        # be: VBSP patches brush faces with it twice (maps/<map>/..._wvt_patch,
        # then a cubemap patch of that), which is what #111 tripped on.
        return ('"WorldVertexTransition"\n{\n\t"$basetexture" '
                '"nature/blendgrassgrass001a_tooltexture"\n\t"$basetexture2" '
                '"nature/blendgrassgrass001a_tooltexture"\n'
                '\t"$envmap" "env_cubemap"\n'
                '\t"%%detailtype" "%s"\n}\n' % detailtype)

    write(os.path.join(game, "materials/detailtest/grass.vmt"), vmt("testgrass"))
    write(os.path.join(game, "materials/detailtest/grass_long.vmt"),
          vmt("testlong"))

    def dtype(name, model, density):
        return ('\t%s\n\t{\n\t\t"density" "%s"\n\t\tGroup1\n\t\t{\n'
                '\t\t\t"alpha" "1.0"\n\t\t\tModel1\n\t\t\t{\n'
                '\t\t\t\t"model" "%s.mdl"\n\t\t\t\t"amount" "0.6"\n'
                '\t\t\t}\n\t\t}\n\t}\n' % (name, density, model))

    long_mdl = LONG_DIR + "/grass_tuft_001a"
    write(os.path.join(game, "detail.vbsp"),
          "detail\n{\n" + dtype("testgrass", GRASS_MDL, 1200)
          + dtype("testlong", long_mdl, 1200) + "}\n")
    if longname:
        import vpk
        pak = vpk.open(os.path.join(SDK, "hl2", "hl2_misc_dir.vpk"))
        for ext in (".mdl", ".vvd", ".dx90.vtx", ".phy"):
            try:
                data = pak.get_file(GRASS_MDL + ext).read()
            except KeyError:
                continue
            out = os.path.join(game, long_mdl + ext)
            os.makedirs(os.path.dirname(out), exist_ok=True)
            with open(out, "wb") as f:
                f.write(data)
        print("long model name: %d chars" % len(long_mdl + ".mdl"))

# ------------------------------------------------------------------ VMF


class Ids:
    def __init__(self):
        self.n = 0

    def __call__(self):
        self.n += 1
        return self.n


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def fmt(p):
    return "(%g %g %g)" % p


def face_points(lo, hi, axis, positive):
    """Three points on a box face, ordered so (p0-p1)x(p2-p1) points out."""
    c = [hi[axis] if positive else lo[axis]] * 1
    u, v = [a for a in range(3) if a != axis]
    corners = []
    for cu, cv in ((lo[u], lo[v]), (lo[u], hi[v]), (hi[u], hi[v])):
        p = [0, 0, 0]
        p[axis], p[u], p[v] = c[0], cu, cv
        corners.append(tuple(p))
    n = cross(sub(corners[0], corners[1]), sub(corners[2], corners[1]))
    if (n[axis] > 0) != positive:
        corners.reverse()
    return corners


AXES = {0: ("[0 1 0 0] 0.25", "[0 0 -1 0] 0.25"),
        1: ("[1 0 0 0] 0.25", "[0 0 -1 0] 0.25"),
        2: ("[1 0 0 0] 0.25", "[0 -1 0 0] 0.25")}


def side(ids, pts, mat, axis, disp=""):
    ua, va = AXES[axis]
    return ('\t\tside\n\t\t{\n\t\t\t"id" "%d"\n\t\t\t"plane" "%s %s %s"\n'
            '\t\t\t"material" "%s"\n\t\t\t"uaxis" "%s"\n\t\t\t"vaxis" "%s"\n'
            '\t\t\t"rotation" "0"\n\t\t\t"lightmapscale" "16"\n'
            '\t\t\t"smoothing_groups" "0"\n%s\t\t}\n'
            % (ids(), fmt(pts[0]), fmt(pts[1]), fmt(pts[2]), mat, ua, va, disp))


def box(ids, lo, hi, mats, disp=""):
    """mats: dict (axis, positive) -> material, default nodraw."""
    s = "\tsolid\n\t{\n\t\t\"id\" \"%d\"\n" % ids()
    for axis in range(3):
        for positive in (False, True):
            mat = mats.get((axis, positive), "TOOLS/TOOLSNODRAW")
            d = disp if (axis, positive) == (2, True) else ""
            s += side(ids, face_points(lo, hi, axis, positive), mat, axis, d)
    return s + "\t}\n"


PERIOD = 256.0
AMP = 12.0


def height(x, y):
    return AMP * (math.sin(2 * math.pi * x / PERIOD)
                  + math.sin(2 * math.pi * y / PERIOD))


def dispinfo(x0, y0, size, power):
    """Displacement on the top face (z=0) of a box with min corner x0,y0.

    x0 and y0 are multiples of PERIOD, so the heights are symmetric in row and
    column and the result does not depend on which edge Hammer calls a row.
    """
    n = (1 << power) + 1
    step = size / (n - 1)
    rows = {k: [] for k in ("normals", "distances", "offsets",
                            "offset_normals", "alphas", "triangle_tags")}
    for r in range(n):
        ds = [height(x0 + c * step, y0 + r * step) for c in range(n)]
        rows["normals"].append(" ".join("0 0 1" for _ in ds))
        rows["distances"].append(" ".join("%g" % d for d in ds))
        rows["offsets"].append(" ".join("0 0 0" for _ in ds))
        rows["offset_normals"].append(" ".join("0 0 1" for _ in ds))
        rows["alphas"].append(" ".join("0" for _ in ds))
        if r < n - 1:
            rows["triangle_tags"].append(" ".join("9" for _ in range(2 * (n - 1))))
    s = ('\t\t\tdispinfo\n\t\t\t{\n\t\t\t\t"power" "%d"\n'
         '\t\t\t\t"startposition" "[%g %g 0]"\n\t\t\t\t"flags" "0"\n'
         '\t\t\t\t"elevation" "0"\n\t\t\t\t"subdiv" "0"\n' % (power, x0, y0))
    for key, vals in rows.items():
        s += "\t\t\t\t%s\n\t\t\t\t{\n" % key
        for i, v in enumerate(vals):
            s += '\t\t\t\t\t"row%d" "%s"\n' % (i, v)
        s += "\t\t\t\t}\n"
    s += ('\t\t\t\tallowed_verts\n\t\t\t\t{\n\t\t\t\t\t"10" "%s"\n'
          '\t\t\t\t}\n\t\t\t}\n' % " ".join(["-1"] * 10))
    return s


def write_vmf(path, longname):
    ids = Ids()
    wall = "DEV/DEV_MEASUREWALL01A"
    grass = "DETAILTEST/GRASS"
    solids = []
    X0, X1, Y0, Y1, Z1 = -1024, 1024, 0, 1024, 512
    # walls, ceiling, under-floor slab
    solids.append(box(ids, (X0 - 16, Y0, -64), (X0, Y1, Z1), {(0, True): wall}))
    solids.append(box(ids, (X1, Y0, -64), (X1 + 16, Y1, Z1), {(0, False): wall}))
    solids.append(box(ids, (X0 - 16, Y0 - 16, -64), (X1 + 16, Y0, Z1),
                      {(1, True): wall}))
    solids.append(box(ids, (X0 - 16, Y1, -64), (X1 + 16, Y1 + 16, Z1),
                      {(1, False): wall}))
    solids.append(box(ids, (X0 - 16, Y0 - 16, Z1), (X1 + 16, Y1 + 16, Z1 + 16),
                      {(2, False): wall}))
    solids.append(box(ids, (X0, Y0, -64), (X1, Y1, -48), {}))
    # brush floor, left half (a strip of it is the long-name patch)
    if longname:
        solids.append(box(ids, (X0, Y0, -48), (X0 + 256, Y1, 0),
                          {(2, True): "DETAILTEST/GRASS_LONG"}))
        solids.append(box(ids, (X0 + 256, Y0, -48), (0, Y1, 0), {(2, True): grass}))
    else:
        solids.append(box(ids, (X0, Y0, -48), (0, Y1, 0), {(2, True): grass}))
    # four sewn displacements, right half
    for x0 in (0, 512):
        for y0 in (0, 512):
            solids.append(box(ids, (x0, y0, -48), (x0 + 512, y0 + 512, 0),
                              {(2, True): grass}, dispinfo(x0, y0, 512, 4)))
    world = ('world\n{\n\t"id" "%d"\n\t"mapversion" "1"\n'
             '\t"classname" "worldspawn"\n\t"skyname" "sky_day01_01"\n'
             % ids()) + "".join(solids) + "}\n"
    ent = ('entity\n{\n\t"id" "%d"\n\t"classname" "info_player_start"\n'
           '\t"origin" "-512 512 64"\n\t"angles" "0 0 0"\n}\n' % ids())
    ent += ('entity\n{\n\t"id" "%d"\n\t"classname" "env_cubemap"\n'
            '\t"origin" "0 512 128"\n\t"cubemapsize" "0"\n}\n' % ids())
    write(path, 'versioninfo\n{\n\t"editorversion" "400"\n\t"mapversion" "1"\n'
          '\t"formatversion" "100"\n\t"prefab" "0"\n}\n' + world + ent)

# ------------------------------------------------------------------ dprp


def read_dprp(path):
    with open(path, "rb") as f:
        data = f.read()
    # lump 35 = game lump
    ofs, length = struct.unpack_from("<ii", data, 8 + 35 * 16)
    count = struct.unpack_from("<i", data, ofs)[0]
    p = ofs + 4
    for _ in range(count):
        lid, flags, ver, fofs, flen = struct.unpack_from("<iHHii", data, p)
        p += 16
        if lid == 0x64707270:  # 'dprp'
            break
    else:
        return [], []
    q = fofs
    n = struct.unpack_from("<i", data, q)[0]
    q += 4
    names = [data[q + 128 * i:q + 128 * (i + 1)] for i in range(n)]
    q += 128 * n
    n = struct.unpack_from("<i", data, q)[0]
    q += 4
    sprites = [struct.unpack_from("<8f", data, q + 32 * i) for i in range(n)]
    q += 32 * n
    n = struct.unpack_from("<i", data, q)[0]
    q += 4
    objs = []
    for i in range(n):
        # origin, angles, model, leaf, lighting, styles, style count, sway,
        # shape angle, shape size, orientation, pad, type, pad, scale
        o = struct.unpack_from("<3f3fHH4sIBBBBB3sB3sf", data, q + 52 * i)
        objs.append(o)
    read_dprp.sprites = sprites
    return names, objs


def dump(bsp):
    names, objs = read_dprp(bsp)
    for i, nm in enumerate(names):
        term = b"\0" in nm
        print("dict %d: %r%s" % (i, nm.split(b"\0")[0].decode("latin-1"),
                                 "" if term else "  (NOT TERMINATED)"))
    print("objects: %d" % len(objs))
    for o in objs[:5]:
        print("  origin (%.2f %.2f %.2f) model %d" % (o[0], o[1], o[2], o[6]))


def keys(bsp):
    """Objects as (origin, model): dict order differs between maps, so a model
    is its name and a sprite its type plus sprite dict entry."""
    names, objs = read_dprp(bsp)
    sprites = read_dprp.sprites
    return objs, collections.Counter(
        (round(o[0], 3), round(o[1], 3), round(o[2], 3),
         names[o[6]] if o[16] == 0 else (o[16],) + sprites[o[6]])
        for o in objs)


def diff(a, b):
    oa, ca = keys(a)
    ob, cb = keys(b)
    only_a, only_b = ca - cb, cb - ca
    print("%s: %d  %s: %d  common: %d  only first: %d  only second: %d"
          % (os.path.basename(a), len(oa), os.path.basename(b), len(ob),
             sum((ca & cb).values()), sum(only_a.values()), sum(only_b.values())))

    def region(k):
        return "brush" if k[0] < 0 else "disp"
    for label, c in (("only first", only_a), ("only second", only_b)):
        by = collections.Counter()
        for k, v in c.items():
            by[region(k)] += v
        if by:
            print("  %s by test map floor (x<0 brush): %s" % (label, dict(by)))
    return 0 if not only_a and not only_b else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest="cmd", required=True)
    g = sp.add_parser("gen")
    g.add_argument("out")
    g.add_argument("--longname", action="store_true")
    d = sp.add_parser("dump")
    d.add_argument("bsp")
    df = sp.add_parser("diff")
    df.add_argument("a")
    df.add_argument("b")
    a = ap.parse_args()
    if a.cmd == "gen":
        write_game(os.path.join(a.out, "game"), a.longname)
        write_vmf(os.path.join(a.out, "detailtest.vmf"), a.longname)
        print("wrote", a.out)
        return 0
    if a.cmd == "dump":
        dump(a.bsp)
        return 0
    return diff(a.a, a.b)


if __name__ == "__main__":
    sys.exit(main())
