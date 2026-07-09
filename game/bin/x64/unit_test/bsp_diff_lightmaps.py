import struct
import sys
import os
import argparse

# Lump IDs
LUMP_FACES = 7
LUMP_LIGHTING = 8
LUMP_FACES_HDR = 58
LUMP_LIGHTING_HDR = 53

HEADER_SIZE = 1036
LUMP_COUNT = 64
LUMP_STRUCT_SIZE = 16

class BSPFile:
    def __init__(self, filename):
        self.filename = filename
        with open(filename, "rb") as f:
            self.data = f.read()
        
        # Parse Header
        ident, version = struct.unpack_from("<II", self.data, 0)
        if ident != 0x50534256: # "VBSP"
            raise ValueError(f"Invalid BSP ident: {hex(ident)}")
        
        self.lumps = []
        for i in range(LUMP_COUNT):
            # int fileofs, filelen, version, uncompressedSize
            lump_info = struct.unpack_from("<IIII", self.data, 8 + i * LUMP_STRUCT_SIZE)
            self.lumps.append({
                "fileofs": lump_info[0],
                "filelen": lump_info[1],
                "version": lump_info[2],
                "uncompressedSize": lump_info[3]
            })

    def get_lump(self, lump_id):
        lump = self.lumps[lump_id]
        return self.data[lump["fileofs"]:lump["fileofs"] + lump["filelen"]]

def compare_lightmaps(file1, file2, threshold=0.1):
    try:
        bsp1 = BSPFile(file1)
        bsp2 = BSPFile(file2)
    except Exception as e:
        print(f"Error reading BSP files: {e}")
        return 1

    # Prefer HDR lumps if they exist in both
    hdr1 = bsp1.lumps[LUMP_LIGHTING_HDR]["filelen"] > 0
    hdr2 = bsp2.lumps[LUMP_LIGHTING_HDR]["filelen"] > 0
    
    if hdr1 and hdr2:
        print("Comparing HDR lighting...")
        face_lump_id = LUMP_FACES_HDR
        light_lump_id = LUMP_LIGHTING_HDR
    else:
        print("Comparing LDR lighting...")
        face_lump_id = LUMP_FACES
        light_lump_id = LUMP_LIGHTING

    faces1_raw = bsp1.get_lump(face_lump_id)
    faces2_raw = bsp2.get_lump(face_lump_id)
    light1_raw = bsp1.get_lump(light_lump_id)
    light2_raw = bsp2.get_lump(light_lump_id)

    # dface_t is 56 bytes in VBSP 20
    FACE_SIZE = 56
    num_faces1 = len(faces1_raw) // FACE_SIZE
    num_faces2 = len(faces2_raw) // FACE_SIZE

    if num_faces1 != num_faces2:
        print(f"Warning: Number of faces differs! {num_faces1} vs {num_faces2}")
    
    total_faces = min(num_faces1, num_faces2)
    diff_count = 0
    total_pixels_compared = 0
    total_error = 0
    max_error = 0
    
    # Track discrete mismatches for manual inspection context
    structural_mismatch = False

    for i in range(total_faces):
        off = i * FACE_SIZE
        # Read lightofs (offset 20), styles (offset 16), and size (offset 36)
        # struct dface_t { ... byte styles[4]; int lightofs; ... int size[2]; ... }
        styles = struct.unpack_from("BBBB", faces1_raw, off + 16)
        lightofs1 = struct.unpack_from("<i", faces1_raw, off + 20)[0]
        size1 = struct.unpack_from("<ii", faces1_raw, off + 36)
        
        styles2 = struct.unpack_from("BBBB", faces2_raw, off + 16)
        lightofs2 = struct.unpack_from("<i", faces2_raw, off + 20)[0]
        size2 = struct.unpack_from("<ii", faces2_raw, off + 36)

        if lightofs1 == -1 or lightofs2 == -1:
            if lightofs1 != lightofs2:
                # print(f"Face {i}: One has no lighting, the other does!")
                diff_count += 1
                structural_mismatch = True
            continue

        if size1 != size2:
            # print(f"Face {i}: Size differs! {size1} vs {size2}")
            diff_count += 1
            structural_mismatch = True
            continue

        num_styles = 0
        for s in styles:
            if s != 255: num_styles += 1
        
        # Source lightmap size includes border
        luxels = (size1[0] + 1) * (size1[1] + 1)
        bytes_per_face = num_styles * luxels * 4
        
        data1 = light1_raw[lightofs1 : lightofs1 + bytes_per_face]
        data2 = light2_raw[lightofs2 : lightofs2 + bytes_per_face]

        if data1 != data2:
            diff_count += 1
            # Calculate pixel-level stats
            if len(data1) == len(data2):
                # We iterate byte-by-byte now to include exponent/alpha data
                for k in range(len(data1)):
                    p1 = data1[k]
                    p2 = data2[k]
                    err = abs(p1 - p2)
                    total_error += err
                    if err > max_error:
                        max_error = err
                
                total_pixels_compared += len(data1) 
            else:
                # print(f"Face {i}: Data length mismatch! {len(data1)} vs {len(data2)}")
                structural_mismatch = True

    print("-" * 40)
    print(f"Total faces compared: {total_faces}")
    print(f"Faces with differences: {diff_count}")
    
    percent_diff = 0.0
    
    if structural_mismatch:
        print("Note: Structural differences found (face sizes or lighting presence differed).")
    
    if total_pixels_compared > 0:
        # Normalize error to percentage
        # Max possible error per byte is 255
        max_possible_error = total_pixels_compared * 255.0
        percent_diff = (total_error / max_possible_error) * 100.0
        
        avg_err = total_error / total_pixels_compared
        print(f"Average byte difference: {avg_err:.4f} / 255")
        print(f"Max byte difference: {max_error}")
    elif diff_count == 0 and not structural_mismatch:
        percent_diff = 0.0
    elif structural_mismatch: 
        # If we have structural mismatches but no pixels compared (unlikely unless all differ in size), 
        # we still flag a failure.
        percent_diff = 100.0 

    print(f"Lightmap Difference: {percent_diff:.4f}%")
    
    if percent_diff == 0:
        print("All lightmaps are identical.")
        return 0
    elif percent_diff <= threshold:
        print(f"All lightmaps are within tolerance. (Difference {percent_diff:.4f}% <= Threshold {threshold}%)")
        return 0
    else:
        print(f"Substantial lightmap difference detected! (Difference {percent_diff:.4f}% > Threshold {threshold}%)")
        return 1

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Compare lightmaps between two BSP files.")
    parser.add_argument("map1", help="First BSP file")
    parser.add_argument("map2", help="Second BSP file")
    parser.add_argument("--threshold", type=float, default=0.0, help="Pass threshold percentage (default: 0.0)")
    
    args = parser.parse_args()
    
    sys.exit(compare_lightmaps(args.map1, args.map2, args.threshold))
