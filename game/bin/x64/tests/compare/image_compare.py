"""Image and lightmap comparison for the vrad and vvis comparison harnesses.

    python -I image_compare.py lightmaps A.bsp B.bsp [--threshold PCT] [--worst N]
    python -I image_compare.py ssim A.tga B.tga OUT_PREFIX
    python -I image_compare.py tga2png FILE.tga [FILE.tga ...]

lightmaps  Compares the lighting lumps of two BSPs face by face (HDR when both
           have it, LDR otherwise) and prints the share of the decoded light that
           differs: each luxel is decoded to rgb * 2^exponent, and the figure is
           the sum of the absolute differences over the sum of the light (the
           mean of the two files), in percent. Exit 0 when it is within
           --threshold (default 0); 1 when it is not, when the two files do not
           have the same faces lit at the same sizes with the same set of light
           styles, or when a file cannot be read. Lightmaps are matched by light
           style, not by their order in the face (two builds can store the same
           two styles the other way round), and a bumped face's three bump
           lightmaps are compared along with the flat one. --worst N lists the N
           faces that carry most of the difference. The raw byte figure is printed for information only: it counts the
           exponent byte, so one exponent step with a halved mantissa (the same
           colour) reads as a large difference (#161).
ssim       Compares two screenshots (same size) with SSIM on grayscale and
           prints "Difference: N%" (100 minus the similarity), then writes
           OUT_PREFIX-alpha.png (absolute difference) and
           OUT_PREFIX-alphacontrast.png (the same, 20x brighter). Needs numpy,
           opencv-python and Pillow. Exit 1 on an unreadable or mismatched image.
tga2png    Writes FILE.png beside each TGA (Pillow reads the Source engine TGAs
           that cv2.imread refuses).

Used by tests/compare/compare_harness.ps1. The harness's own tolerance rules
live there; this script only measures.
"""
import argparse
import importlib
import os
import struct
import sys

# Lump IDs
LUMP_TEXINFO = 6
LUMP_FACES = 7
LUMP_LIGHTING = 8
LUMP_FACES_HDR = 58
LUMP_LIGHTING_HDR = 53

TEXINFO_SIZE = 72           # texinfo_t: two 2x4 float matrices, then int flags, int texdata
TEXINFO_FLAGS_OFS = 64
SURF_BUMPLIGHT = 0x0800     # the face has three bump lightmaps after the flat one, per style

HEADER_SIZE = 1036
LUMP_COUNT = 64
LUMP_STRUCT_SIZE = 16


# ---------------------------------------------------------------------------
# lightmaps
# ---------------------------------------------------------------------------

class BSPFile:
    def __init__(self, filename):
        self.filename = filename
        with open(filename, "rb") as f:
            self.data = f.read()

        # Parse Header
        ident, version = struct.unpack_from("<II", self.data, 0)
        if ident != 0x50534256:  # "VBSP"
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


def face_is_bumped(faces_raw, off, texinfo_raw):
    """True when the face's texinfo has SURF_BUMPLIGHT (dface_t.texinfo is the short at +10)."""
    texinfo = struct.unpack_from("<h", faces_raw, off + 10)[0]
    at = texinfo * TEXINFO_SIZE + TEXINFO_FLAGS_OFS
    if texinfo < 0 or at + 4 > len(texinfo_raw):
        return False
    return bool(struct.unpack_from("<i", texinfo_raw, at)[0] & SURF_BUMPLIGHT)


def style_bytes(light_raw, lightofs, k, per_style):
    """The k-th style of a face: its average colour (4 bytes), then its lightmap block(s)."""
    avg_at = lightofs - 4 * (k + 1)
    avg = light_raw[avg_at: avg_at + 4] if avg_at >= 0 else b""
    return avg + light_raw[lightofs + k * per_style: lightofs + (k + 1) * per_style]


def compare_lightmaps(file1, file2, threshold=0.1, worst=0):
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
    tex1_raw = bsp1.get_lump(LUMP_TEXINFO)
    tex2_raw = bsp2.get_lump(LUMP_TEXINFO)

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
    light_total = 0.0
    light_error = 0.0
    face_rows = []  # (light error, light total, face, width, height) of each face whose light differs

    # A face lit in one file only, at another size or with other light styles; so is a face one
    # file does not have at all.
    structural_mismatch = num_faces1 != num_faces2

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

        # A face's lightmaps are stored style by style, in the order of its style table, and that
        # order is not part of the lighting: two builds can list the same two named lights the
        # other way round (it follows which light reached the face first). So the styles are
        # matched by value, and only another SET of styles is a structural difference, like
        # another size or a bumped face against an unbumped one.
        used1 = [s for s in styles if s != 255]
        used2 = [s for s in styles2 if s != 255]
        bumped1 = face_is_bumped(faces1_raw, off, tex1_raw)
        bumped2 = face_is_bumped(faces2_raw, off, tex2_raw)
        if size1 != size2 or sorted(used1) != sorted(used2) or len(set(used1)) != len(used1) or bumped1 != bumped2:
            diff_count += 1
            structural_mismatch = True
            continue

        # Source lightmap size includes border. A bumped face holds four lightmaps per style (the
        # flat one, then the three bump directions); all of them are compared.
        luxels = (size1[0] + 1) * (size1[1] + 1)
        per_style = luxels * 4 * (4 if bumped1 else 1)

        # Each style also has one average colour, stored backwards before lightofs (style k at
        # lightofs - 4 * (k + 1)); VRAD's detail prop lighting reads it, so it is compared too.
        data1 = b"".join(style_bytes(light1_raw, lightofs1, used1.index(s), per_style) for s in sorted(used1))
        data2 = b"".join(style_bytes(light2_raw, lightofs2, used2.index(s), per_style) for s in sorted(used2))

        # Decoded light: each luxel is ColorRGBExp32 (r, g, b, signed exponent), colour = rgb * 2^exp.
        # Summed over every lit face, identical or not, so the share below is of all the light.
        if len(data1) == len(data2):
            face_total = 0.0
            face_error = 0.0
            for k in range(0, len(data1) - 3, 4):
                s1 = 2.0 ** (data1[k + 3] - 256 if data1[k + 3] > 127 else data1[k + 3])
                s2 = 2.0 ** (data2[k + 3] - 256 if data2[k + 3] > 127 else data2[k + 3])
                for c in range(3):
                    v1, v2 = data1[k + c] * s1, data2[k + c] * s2
                    face_total += 0.5 * (v1 + v2)
                    face_error += abs(v1 - v2)
            light_total += face_total
            light_error += face_error
            if face_error > 0.0:
                face_rows.append((face_error, face_total, i, size1[0] + 1, size1[1] + 1))

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

    if total_pixels_compared > 0:
        # Information only. Raw RGBE bytes, exponent byte included: one exponent step with a halved
        # mantissa (the same colour) counts as up to 255 here, which is why this is not the verdict.
        print(f"Raw bytes: {100.0 * total_error / (total_pixels_compared * 255.0):.4f} percent of the differing faces' "
              f"bytes, average {total_error / total_pixels_compared:.4f} / 255, max {max_error}")

    # The verdict: the share of the decoded light that differs (#161).
    percent_diff = 100.0 * light_error / light_total if light_total > 0 else 0.0
    print(f"Lightmap Difference: {percent_diff:.4f}% of the decoded light"
          + (" (only of the faces both files light the same way: see below)" if structural_mismatch else ""))

    if worst > 0 and face_rows:
        face_rows.sort(reverse=True)
        print(f"Faces carrying the most difference (of {len(face_rows)} whose light differs):")
        for err, tot, face, w, h in face_rows[:worst]:
            print(f"  face {face:6d}  {w}x{h}  differs {100.0 * err / tot if tot > 0 else 0.0:8.4f}% of its own light, "
                  f"{100.0 * err / light_error:6.2f}% of all the difference")

    if structural_mismatch:
        # Not a matter of tolerance.
        print("Structural difference: the two files do not have the same faces lit, at the same sizes, "
              "with the same light styles.")
        return 1
    if diff_count == 0:
        print("All lightmaps are identical.")
        return 0
    if percent_diff <= threshold:
        print(f"All lightmaps are within tolerance. (Difference {percent_diff:.4f}% <= Threshold {threshold}%)")
        return 0
    print(f"Substantial lightmap difference detected! (Difference {percent_diff:.4f}% > Threshold {threshold}%)")
    return 1


def cmd_lightmaps(args):
    return compare_lightmaps(args.map1, args.map2, args.threshold, args.worst)


# ---------------------------------------------------------------------------
# ssim and tga2png: the imaging libraries are imported only here, so the
# lightmaps comparison runs without them.
# ---------------------------------------------------------------------------

def _import(name):
    """Import a third-party module. python -I leaves the user site-packages
    (pip install --user) off sys.path, so look there once before giving up."""
    try:
        return importlib.import_module(name)
    except ImportError:
        import site
        user = site.getusersitepackages()
        if user in sys.path or not os.path.isdir(user):
            raise
        sys.path.append(user)
        return importlib.import_module(name)


def compute_ssim(cv2, np, img1_gray, img2_gray):
    # Standard SSIM constants based on dynamic range of 255
    C1 = (0.01 * 255)**2
    C2 = (0.03 * 255)**2

    img1 = img1_gray.astype(np.float64)
    img2 = img2_gray.astype(np.float64)

    # 11x11 Gaussian kernel, sigma=1.5
    kernel = cv2.getGaussianKernel(11, 1.5)
    window = np.outer(kernel, kernel.transpose())

    mu1 = cv2.filter2D(img1, -1, window)[5:-5, 5:-5]
    mu2 = cv2.filter2D(img2, -1, window)[5:-5, 5:-5]
    mu1_sq = mu1**2
    mu2_sq = mu2**2
    mu1_mu2 = mu1 * mu2

    sigma1_sq = cv2.filter2D(img1**2, -1, window)[5:-5, 5:-5] - mu1_sq
    sigma2_sq = cv2.filter2D(img2**2, -1, window)[5:-5, 5:-5] - mu2_sq
    sigma12 = cv2.filter2D(img1 * img2, -1, window)[5:-5, 5:-5] - mu1_mu2

    ssim_map = ((2 * mu1_mu2 + C1) * (2 * sigma12 + C2)) / ((mu1_sq + mu2_sq + C1) * (sigma1_sq + sigma2_sq + C2))
    return ssim_map.mean()


def generate_diff_images(cv2, img1_color, img2_color, out_prefix):
    # Calculate absolute difference on grayscale to mimic old alpha representation
    img1_gray = cv2.cvtColor(img1_color, cv2.COLOR_BGR2GRAY)
    img2_gray = cv2.cvtColor(img2_color, cv2.COLOR_BGR2GRAY)

    abs_diff = cv2.absdiff(img1_gray, img2_gray)

    # Save base alpha diff
    alpha_path = f"{out_prefix}-alpha.png"
    cv2.imwrite(alpha_path, abs_diff)

    # Save contrast boosted diff (20x amplification)
    contrast_diff = cv2.convertScaleAbs(abs_diff, alpha=20.0, beta=0.0)
    contrast_path = f"{out_prefix}-alphacontrast.png"
    cv2.imwrite(contrast_path, contrast_diff)


def cmd_ssim(args):
    try:
        cv2 = _import("cv2")
        np = _import("numpy")
        Image = _import("PIL.Image")
    except ImportError as e:
        print(f"Error: {e}. Needs numpy, opencv-python and pillow (pip install numpy opencv-python pillow).")
        return 1

    file1 = args.image1
    file2 = args.image2
    out_prefix = args.out_prefix

    if not os.path.exists(file1) or not os.path.exists(file2):
        print(f"Error: Could not find inputs {file1} or {file2}")
        return 1

    try:
        # Read with PIL as cv2.imread fails on Source Engine TGAs
        pil_img1 = Image.open(file1).convert('RGB')
        pil_img2 = Image.open(file2).convert('RGB')

        # Convert PIL image to OpenCV format (RGB -> BGR)
        img1_color = cv2.cvtColor(np.array(pil_img1), cv2.COLOR_RGB2BGR)
        img2_color = cv2.cvtColor(np.array(pil_img2), cv2.COLOR_RGB2BGR)
    except Exception as e:
        print(f"Error: Could not decode images - {e}")
        return 1

    if img1_color is None or img2_color is None:
        print("Error: Could not decode images")
        return 1

    if img1_color.shape != img2_color.shape:
        print(f"Error: Size mismatch {img1_color.shape} vs {img2_color.shape}")
        return 1

    img1_gray = cv2.cvtColor(img1_color, cv2.COLOR_BGR2GRAY)
    img2_gray = cv2.cvtColor(img2_color, cv2.COLOR_BGR2GRAY)

    # Calculate SSIM
    s = compute_ssim(cv2, np, img1_gray, img2_gray)

    # Map 0.0 - 1.0 back to a difference percentage to match the "Difference: N%" line
    # the harness parses (tgadiff prints the same line)
    diff_percent = (1.0 - s) * 100.0
    print(f"Difference: {diff_percent:.4f}%")

    # Generate the visual differential PNGs directly without TGA intermediate
    generate_diff_images(cv2, img1_color, img2_color, out_prefix)
    return 0


def convert_tga_to_png(Image, file_path):
    if not os.path.exists(file_path):
        return

    try:
        # Open the TGA file
        with Image.open(file_path) as img:
            # Construct png path
            base, ext = os.path.splitext(file_path)
            png_path = f"{base}.png"

            # Save as PNG
            img.save(png_path)
            print(f"Converted: {file_path} -> {png_path}")

    except Exception as e:
        print(f"Failed to convert {file_path}: {e}")


def cmd_tga2png(args):
    try:
        Image = _import("PIL.Image")
    except ImportError:
        print("Error: PIL/Pillow not installed. Please run 'pip install pillow'")
        return 1
    for path in args.files:
        convert_tga_to_png(Image, path)
    return 0


# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("lightmaps", help="Compare the lightmaps of two BSP files.")
    p.add_argument("map1", help="First BSP file")
    p.add_argument("map2", help="Second BSP file")
    p.add_argument("--threshold", type=float, default=0.0,
                   help="Pass threshold: percent of the decoded light that may differ (default: 0.0)")
    p.add_argument("--worst", type=int, default=0, help="List the N faces that carry the most difference")
    p.set_defaults(func=cmd_lightmaps)

    p = sub.add_parser("ssim", help="SSIM difference of two screenshots, plus difference PNGs.")
    p.add_argument("image1", help="First image (.tga)")
    p.add_argument("image2", help="Second image (.tga)")
    p.add_argument("out_prefix", help="Prefix of the -alpha.png and -alphacontrast.png files")
    p.set_defaults(func=cmd_ssim)

    p = sub.add_parser("tga2png", help="Convert TGA files to PNG next to the originals.")
    p.add_argument("files", nargs="+", help="TGA files")
    p.set_defaults(func=cmd_tga2png)

    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
