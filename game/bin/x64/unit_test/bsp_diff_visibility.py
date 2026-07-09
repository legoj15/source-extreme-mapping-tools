import struct
import sys

# Lump ID
LUMP_VISIBILITY = 4

class BSPVisibility:
    def __init__(self, filename):
        self.filename = filename
        with open(filename, "rb") as f:
            self.data = f.read()
        
        # Parse Header
        ident, _ = struct.unpack_from("<II", self.data, 0)
        if ident != 0x50534256:
            raise ValueError(f"Invalid BSP ident: {hex(ident)}")
        
        # Get Visibility Lump Info
        lump_info = struct.unpack_from("<IIII", self.data, 8 + LUMP_VISIBILITY * 16)
        fileofs, filelen = lump_info[0], lump_info[1]
        
        if filelen == 0:
            self.num_clusters = 0
            self.pvs_offsets = []
            return

        vis_data = self.data[fileofs : fileofs + filelen]
        self.num_clusters = struct.unpack_from("<I", vis_data, 0)[0]
        self.pvs_offsets = []
        for i in range(self.num_clusters):
            # Each cluster has PVS and PAS offset
            pvs_off, pas_off = struct.unpack_from("<ii", vis_data, 4 + i * 8)
            self.pvs_offsets.append(pvs_off)
        
        self.vis_data = vis_data

    def decompress_pvs(self, cluster_idx):
        if cluster_idx >= len(self.pvs_offsets):
            return None
        
        offset = self.pvs_offsets[cluster_idx]
        if offset <= 0: # -1 or 0 means no visibility data for this cluster
            return bytearray((self.num_clusters + 7) // 8)

        row_size = (self.num_clusters + 7) // 8
        out = bytearray(row_size)
        
        in_ptr = offset
        out_ptr = 0
        
        while out_ptr < row_size:
            val = self.vis_data[in_ptr]
            if val == 0:
                count = self.vis_data[in_ptr + 1]
                out_ptr += count
                in_ptr += 2
            else:
                out[out_ptr] = val
                out_ptr += 1
                in_ptr += 1
        
        return out

def compare_visibility(bsp_file1, bsp_file2, tolerance=0.0):
    vis1 = BSPVisibility(bsp_file1)
    vis2 = BSPVisibility(bsp_file2)

    if vis1.num_clusters != vis2.num_clusters:
        print(f"Warning: Cluster count mismatch! {vis1.num_clusters} vs {vis2.num_clusters}")
    
    num_clusters = min(vis1.num_clusters, vis2.num_clusters)
    diff_clusters = []
    
    total_visible1 = 0
    total_visible2 = 0

    for i in range(num_clusters):
        pvs1 = vis1.decompress_pvs(i)
        pvs2 = vis2.decompress_pvs(i)

        if pvs1 != pvs2:
            diff_clusters.append(i)
        
        # Count set bits for statistics
        if pvs1: total_visible1 += sum(bin(b).count('1') for b in pvs1)
        if pvs2: total_visible2 += sum(bin(b).count('1') for b in pvs2)

    print("-" * 40)
    print(f"Total clusters: {num_clusters}")
    print(f"Clusters with visibility differences: {len(diff_clusters)}")
    
    total_bits = max(total_visible1, total_visible2, 1)
    bit_diff = abs(total_visible1 - total_visible2)
    error_rate = (bit_diff / total_bits) * 100.0

    if diff_clusters:
        print(f"First 10 differing clusters: {diff_clusters[:10]}")
        print(f"Avg visibility (bits set): {total_visible1/num_clusters:.2f} vs {total_visible2/num_clusters:.2f}")
        print(f"Total bit disparity: {bit_diff} bits ({error_rate:.4f}%)")
        
        if error_rate <= tolerance:
            print(f"Visibility data meets tolerance (<={tolerance}%).")
            sys.exit(0)
    elif vis1.num_clusters == vis2.num_clusters:
        print("Visibility data is identical.")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python bsp_diff_visibility.py <map1.bsp> <map2.bsp> [tolerance]")
        sys.exit(1)
    
    tolerance = 0.0
    if len(sys.argv) >= 4:
        tolerance = float(sys.argv[3])

    compare_visibility(sys.argv[1], sys.argv[2], tolerance)
