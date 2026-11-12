"""corrupt_bsp.py <in.bsp> <out.bsp> <what>: writes a copy of a BSP with one index broken.
what: node_plane | node_child | brushside_plane | brush_sides | leafbrush | texdata_name | leaf_cluster | node_cycle |
      big_lump (claims a LUMP_LEAFBRUSHES length past the 65536-entry array, #37)"""
import struct, sys

src, dst, what = sys.argv[1:4]
b = bytearray(open(src, 'rb').read())
assert b[:4] == b'VBSP'

def lump(i):
    ofs, ln, ver, _ = struct.unpack_from('<iiii', b, 8 + 16 * i)
    return ofs, ln

LUMP_PLANES, LUMP_TEXDATA, LUMP_NODES, LUMP_LEAFS = 1, 2, 5, 10
LUMP_LEAFBRUSHES, LUMP_BRUSHES, LUMP_BRUSHSIDES = 17, 18, 19
if what == 'node_plane':
    ofs, _ = lump(LUMP_NODES)
    struct.pack_into('<i', b, ofs, 999999)          # dnode_t.planenum (node 0)
elif what == 'node_child':
    ofs, _ = lump(LUMP_NODES)
    struct.pack_into('<i', b, ofs + 4, 999999)      # dnode_t.children[0]
elif what == 'brushside_plane':
    ofs, _ = lump(LUMP_BRUSHSIDES)
    struct.pack_into('<H', b, ofs, 65000)           # dbrushside_t.planenum
elif what == 'brush_sides':
    ofs, _ = lump(LUMP_BRUSHES)
    struct.pack_into('<i', b, ofs + 4, 1 << 24)     # dbrush_t.numsides
elif what == 'leafbrush':
    ofs, _ = lump(LUMP_LEAFBRUSHES)
    struct.pack_into('<H', b, ofs, 65000)
elif what == 'texdata_name':
    ofs, _ = lump(LUMP_TEXDATA)
    struct.pack_into('<i', b, ofs + 12, 999999)     # dtexdata_t.nameStringTableID
elif what == 'leaf_cluster':
    ofs, _ = lump(LUMP_LEAFS)
    struct.pack_into('<h', b, ofs + 4 + 32, 30000)  # leaf 1 (dleaf_t is 32 bytes): cluster after contents
elif what == 'node_cycle':
    ofs, _ = lump(LUMP_NODES)
    struct.pack_into('<i', b, ofs + 4, 0)           # node 0's front child is node 0 (#115)
elif what == 'big_lump':
    struct.pack_into('<i', b, 8 + 16 * LUMP_LEAFBRUSHES + 4, 2 * 65537)  # lump_t.filelen
else:
    sys.exit('unknown ' + what)
open(dst, 'wb').write(b)
