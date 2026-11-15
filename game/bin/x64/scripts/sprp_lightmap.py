"""List or rewrite per-texel lightmap settings on a BSP's static props.

Static prop lump v10 only (the version this repo's VBSP writes). Rewriting
changes fields in place, so the lump size and every offset stay the same.

  python sprp_lightmap.py list map.bsp
  python sprp_lightmap.py set map.bsp out.bsp --res 1024 [--props 0,3] [--only]

`set` clears STATIC_PROP_NO_PER_TEXEL_LIGHTING and sets the resolution on the
chosen props (default: all). `--only` turns per-texel lighting off on every
other prop, so a timing run measures just the chosen ones.
"""
import argparse
import struct
import sys

LUMP_GAME = 35
SPRP = 0x73707270
NO_PER_TEXEL = 0x100
PROP_SIZE_V10 = 72
FLAGS_OFS, RESX_OFS, RESY_OFS = 64, 68, 70


def locate_sprp(data):
    ofs, length = struct.unpack_from("<ii", data, 8 + LUMP_GAME * 16)
    (count,) = struct.unpack_from("<i", data, ofs)
    for i in range(count):
        gid, gflags, gver, gofs, glen = struct.unpack_from("<iHHii", data, ofs + 4 + i * 16)
        if gid == SPRP:
            if gflags & 1:
                sys.exit("sprp lump is compressed; not supported")
            if gver != 10:
                sys.exit(f"sprp version {gver}; only v10 is supported")
            return gofs
    sys.exit("no sprp game lump")


def parse(data):
    p = locate_sprp(data)
    (ndict,) = struct.unpack_from("<i", data, p)
    names = [data[p + 4 + i * 128: p + 4 + (i + 1) * 128].split(b"\0")[0].decode() for i in range(ndict)]
    p += 4 + ndict * 128
    (nleaf,) = struct.unpack_from("<i", data, p)
    p += 4 + nleaf * 2
    (nprops,) = struct.unpack_from("<i", data, p)
    p += 4
    props = []
    for i in range(nprops):
        base = p + i * PROP_SIZE_V10
        origin = struct.unpack_from("<3f", data, base)
        (ptype,) = struct.unpack_from("<H", data, base + 24)
        (flags,) = struct.unpack_from("<I", data, base + FLAGS_OFS)
        rx, ry = struct.unpack_from("<HH", data, base + RESX_OFS)
        props.append(dict(index=i, base=base, model=names[ptype], origin=origin, flags=flags, res=(rx, ry)))
    return props


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    ls = sub.add_parser("list")
    ls.add_argument("bsp")
    st = sub.add_parser("set")
    st.add_argument("bsp")
    st.add_argument("out")
    st.add_argument("--res", type=int, required=True)
    st.add_argument("--props", help="comma-separated prop indices (default: all)")
    st.add_argument("--only", action="store_true")
    a = ap.parse_args()

    data = bytearray(open(a.bsp, "rb").read())
    props = parse(data)
    if a.cmd == "list":
        for pr in props:
            texel = "off" if pr["flags"] & NO_PER_TEXEL else f"{pr['res'][0]}x{pr['res'][1]}"
            o = pr["origin"]
            print(f"{pr['index']:4d}  texel={texel:10s} flags=0x{pr['flags']:03x}  "
                  f"({o[0]:.0f},{o[1]:.0f},{o[2]:.0f})  {pr['model']}")
        return

    if not 1 <= a.res <= 65535:
        sys.exit("--res must fit in an unsigned short")
    chosen = set(range(len(props))) if not a.props else {int(x) for x in a.props.split(",")}
    for pr in props:
        b = pr["base"]
        if pr["index"] in chosen:
            struct.pack_into("<I", data, b + FLAGS_OFS, pr["flags"] & ~NO_PER_TEXEL)
            struct.pack_into("<HH", data, b + RESX_OFS, a.res, a.res)
        elif a.only:
            struct.pack_into("<I", data, b + FLAGS_OFS, pr["flags"] | NO_PER_TEXEL)
    open(a.out, "wb").write(data)
    print(f"wrote {a.out}: {len(chosen)} prop(s) at {a.res}x{a.res}")


if __name__ == "__main__":
    main()
