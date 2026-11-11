"""
BSP Lightmap Reader — extract per-face lightmap data from compiled Source Engine BSP files.

Parses the BSP binary format to read lightmap samples for each face,
computes luminance variance, and maps BSP faces back to VMF sides using
two strategies:
  1. LUMP_FACEIDS hammerfaceid (direct, but truncated to uint16)
  2. Plane normal + distance + material matching (fallback for high IDs)

BSP Lump Layout (relevant lumps):
    LUMP_PLANES          (1)  — dplane_t (20 bytes)
    LUMP_TEXDATA         (2)  — dtexdata_t (32 bytes)
    LUMP_TEXINFO         (6)  — texinfo_t (72 bytes)
    LUMP_FACES           (7)  — dface_t (56 bytes)
    LUMP_LIGHTING        (8)  — ColorRGBExp32 (4 bytes each)
    LUMP_FACEIDS         (11) — dfaceid_t (2 bytes)
    LUMP_TEXDATA_STRING_DATA  (43) — null-terminated strings
    LUMP_TEXDATA_STRING_TABLE (44) — int32 (4 bytes)
    LUMP_LIGHTING_HDR    (53) — ColorRGBExp32 (4 bytes each)
    LUMP_FACES_HDR       (58) — dface_t (56 bytes)
"""
from __future__ import annotations

import math
import os
import struct
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Union


# ─── BSP Constants ────────────────────────────────────────────────────────────

IDBSPHEADER = 0x50534256  # 'VBSP' in little-endian
HEADER_LUMPS = 64

# Lump indices
LUMP_ENTITIES = 0
LUMP_PLANES = 1
LUMP_TEXDATA = 2
LUMP_VERTEXES = 3
LUMP_VISIBILITY = 4
LUMP_NODES = 5
LUMP_TEXINFO = 6
LUMP_FACES = 7
LUMP_LIGHTING = 8
LUMP_LEAFS = 10
LUMP_FACEIDS = 11
LUMP_EDGES = 12
LUMP_SURFEDGES = 13
LUMP_MODELS = 14
LUMP_LEAFBRUSHES = 17
LUMP_BRUSHES = 18
LUMP_BRUSHSIDES = 19
LUMP_FACE_SIDEIDS_INDEX = 22   # Per-face index into LUMP_FACE_SIDEIDS_DATA
LUMP_FACE_SIDEIDS_DATA = 23    # Packed int32 VMF side IDs
LUMP_GAME_LUMP = 35
LUMP_TEXDATA_STRING_DATA = 43   # Null-terminated material name strings
LUMP_TEXDATA_STRING_TABLE = 44  # int32 offsets into string data
LUMP_LIGHTING_HDR = 53
LUMP_FACES_HDR = 58

# Content flags (from bspflags.h)
CONTENTS_SOLID       = 0x1
CONTENTS_WINDOW      = 0x2
CONTENTS_GRATE       = 0x8
CONTENTS_MOVEABLE    = 0x4000
CONTENTS_PLAYERCLIP  = 0x10000
CONTENTS_MONSTERCLIP = 0x20000
CONTENTS_MONSTER     = 0x2000000

# MASK_PLAYERSOLID = SOLID|MOVEABLE|PLAYERCLIP|WINDOW|MONSTER|GRATE
MASK_PLAYERSOLID = (CONTENTS_SOLID | CONTENTS_MOVEABLE | CONTENTS_PLAYERCLIP |
                    CONTENTS_WINDOW | CONTENTS_MONSTER | CONTENTS_GRATE)

# MASK_VISIBLE — opaque geometry only (for visibility ray traces).
# Excludes PLAYERCLIP, MONSTERCLIP, GRATE (see-through), MONSTER, and WINDOW (glass).
CONTENTS_OPAQUE = 0x80
MASK_VISIBLE = (CONTENTS_SOLID | CONTENTS_MOVEABLE | CONTENTS_OPAQUE)

# Surface flags (from bspflags.h) — for filtering non-renderable faces
SURF_NODRAW = 0x80
SURF_SKY2D  = 0x2   # public/bspflags.h; 0x8 is SURF_WARP (water), not sky
SURF_SKY    = 0x4
SURF_WARP   = 0x8
SURF_SKIP   = 0x200
SURF_NOLIGHT = 0x400
_SURF_SKIP_VIS = SURF_NODRAW | SURF_SKY | SURF_SKY2D | SURF_SKIP

# Game lump IDs
GAMELUMP_STATIC_PROPS = 0x73707270  # 'sprp' as little-endian int

MAXLIGHTMAPS = 4


# ─── BSP Data Structures ─────────────────────────────────────────────────────

@dataclass
class BSPLump:
    """BSP lump descriptor from the header."""
    fileofs: int
    filelen: int
    version: int
    uncompressed_size: int


@dataclass
class BSPPlane:
    """Parsed dplane_t structure (20 bytes)."""
    normal: Tuple[float, float, float]
    dist: float
    type: int


@dataclass
class BSPFace:
    """Parsed dface_t structure (56 bytes)."""
    planenum: int
    side: int              # 0 or 1: which side of the plane
    on_node: int
    firstedge: int
    numedges: int
    texinfo: int
    dispinfo: int
    lightofs: int          # Offset into lighting lump (-1 = no lightmap)
    lightmap_mins: Tuple[int, int]   # m_LightmapTextureMinsInLuxels
    lightmap_size: Tuple[int, int]   # m_LightmapTextureSizeInLuxels
    styles: Tuple[int, ...]          # Light styles (up to 4)
    area: float
    orig_face: int         # Index of the original (unsplit) face
    smoothing_groups: int


@dataclass
class BSPTexInfo:
    """Parsed texinfo_t structure (72 bytes)."""
    texture_vecs: Tuple[Tuple[float, ...], ...]   # [2][4]
    lightmap_vecs: Tuple[Tuple[float, ...], ...]   # [2][4]
    flags: int
    texdata: int


@dataclass
class BSPTexData:
    """Parsed dtexdata_t structure (32 bytes)."""
    reflectivity: Tuple[float, float, float]
    name_string_table_id: int
    width: int
    height: int
    view_width: int
    view_height: int


@dataclass
class BSPFaceData:
    """Per-BSP-face lightmap data with geometric info for matching."""
    face_index: int
    hammer_id: int                     # Primary hammer face ID (from LUMP_FACEIDS)
    plane_normal: Tuple[float, float, float]
    plane_dist: float
    face_side: int          # Which side of the plane this face is on
    material: str
    luminances: List[float]
    luxel_count: int
    area: float
    # Spatial data for carving (lightmap coordinate → world mapping)
    lightmap_mins: Tuple[int, int] = (0, 0)
    lightmap_size: Tuple[int, int] = (0, 0)
    lightmap_vecs: Optional[Tuple[Tuple[float, ...], ...]] = None  # [2][4] from texinfo
    all_hammer_ids: Optional[List[int]] = None  # All merged VMF side IDs (from new lumps)


_LUMP_DIR_BYTES = 8 + HEADER_LUMPS * 16


def _read_lump_entry(header: bytes, lump_id: int) -> Tuple[int, int, int]:
    """(fileofs, filelen, uncompressed_size) of one lump directory entry.

    The fourth lump_t field is the LZMA uncompressed size (0 = stored
    plain); BSPReader.read() unpacks the same field as BSPLump.uncompressed_size.
    """
    ofs = 8 + lump_id * 16
    fileofs, filelen, _ver, uncomp = struct.unpack_from('<4i', header, ofs)
    return fileofs, filelen, uncomp


def bsp_lighting_problem(bsp_path) -> Optional[str]:
    """Why this BSP's lighting cannot be used for classification, or None.

    Reads only the lump directory — no lumps are loaded. Mirrors exactly
    which lump BSPReader reads (HDR when it is non-empty, else LDR) and what
    it can decode, so a True from bsp_has_lighting() means BSPReader will
    really yield luminances:

      * both lighting lumps empty -> VRAD never lit the map (failure, timeout
        or exit-0-without-lighting); classifying such a BSP produced the
        'everything is uniformly black' scale garbage
      * the lump is LZMA-compressed -> its filelen is the compressed size
        and BSPReader would slice compressed bytes as ColorRGBExp32 noise
      * the lump runs past the end of the file (truncated BSP)
    """
    try:
        size = os.path.getsize(bsp_path)
        with open(bsp_path, 'rb') as f:
            header = f.read(_LUMP_DIR_BYTES)
    except OSError as e:
        return f"cannot read BSP ({e})"
    if len(header) < _LUMP_DIR_BYTES or header[:4] != b'VBSP':
        return "not a valid VBSP file (bad ident or truncated lump directory)"

    ofs, length, uncomp = _read_lump_entry(header, LUMP_LIGHTING_HDR)
    if length <= 0:
        ofs, length, uncomp = _read_lump_entry(header, LUMP_LIGHTING)
    if length <= 0:
        return ("it contains no lightmap samples (VRAD never lit it); compile "
                "the BSP with VRAD first")
    if uncomp != 0:
        return ("its lighting lump is LZMA-compressed, which this tool cannot "
                "decode; decompress the BSP first")
    if ofs < 0 or ofs + length > size:
        return "its lighting lump extends past the end of the file (truncated BSP)"
    return None


def bsp_has_lighting(bsp_path) -> bool:
    """True if BSPReader can read real lightmap samples from this BSP.

    See bsp_lighting_problem(); callers use this to abort on unlit baselines
    and to reject poisoned cache entries.
    """
    return bsp_lighting_problem(bsp_path) is None


@dataclass
class BSPSubFaceInfo:
    """Per-BSP-sub-face info preserved for carving spatial analysis."""
    face_index: int
    lightmap_mins: Tuple[int, int]
    lightmap_size: Tuple[int, int]
    lightmap_vecs: Optional[Tuple[Tuple[float, ...], ...]]  # [2][4]
    luminances: List[float]
    is_uniform: bool


@dataclass
class FaceLightmapData:
    """Aggregated lightmap data for one VMF side (potentially from multiple BSP faces)."""
    vmf_side_id: int
    material: str
    bsp_face_indices: List[int]
    luminances: List[float]          # All luxel luminances across all BSP sub-faces
    lightmap_total_luxels: int = 0
    sub_faces: List[BSPSubFaceInfo] = field(default_factory=list)  # Per-sub-face spatial data
    is_never_visible: bool = False   # Set by VisibilityOracle to bypass caches
    # (luxel count, min, max, mean, variance). luminances is append-only, so the
    # count is enough to tell when _add_to_result has grown it since the last pass.
    _stats: Optional[Tuple[int, float, float, float, float]] = field(
        default=None, init=False, repr=False, compare=False)

    def _luminance_stats(self) -> Tuple[int, float, float, float, float]:
        """One pass over the luxels, shared by every statistic below.

        The solvers read variance, mean, range and priority per face several
        times; recomputing them was the single largest cost on big maps.
        """
        lums = self.luminances
        n = len(lums)
        if self._stats is None or self._stats[0] != n:
            mean = sum(lums) / n
            var = sum([(x - mean) ** 2 for x in lums]) / n if n >= 2 else 0.0
            self._stats = (n, min(lums), max(lums), mean, var)
        return self._stats

    @property
    def max_luminance(self) -> float:
        return self._luminance_stats()[2] if self.luminances and not self.is_never_visible else 0.0

    @property
    def min_luminance(self) -> float:
        return self._luminance_stats()[1] if self.luminances and not self.is_never_visible else 0.0

    @property
    def mean_luminance(self) -> float:
        return self._luminance_stats()[3] if self.luminances and not self.is_never_visible else 0.0

    @property
    def variance(self) -> float:
        if self.is_never_visible or len(self.luminances) < 2:
            return 0.0
        return self._luminance_stats()[4]

    @property
    def luminance_range(self) -> float:
        """Max - min luminance. A simpler measure of variation than variance."""
        if self.is_never_visible or not self.luminances:
            return 0.0
        return self.max_luminance - self.min_luminance

    @property
    def is_perfectly_uniform(self) -> bool:
        """True if every luxel has exactly the same luminance (zero variation).

        Faces with NO samples are unknown, not uniform: an unlit baseline BSP
        (VRAD failed or never ran) extracts every face with an empty
        luminance list, and treating those as 'perfectly uniform, mean 0'
        made the solver pre-promote them all to max_useful_scale. Only real
        data counts — including a single real luxel, which is trivially
        constant.
        """
        if self.is_never_visible:
            return True
        if not self.luminances:
            return False
        if len(self.luminances) < 2:
            return True
        # min == max is the exact test. The variance clause this used to
        # carry is float noise for constant data: mean = sum/n is off by an
        # ulp for values like 0.1, giving variance ~1e-34 != 0.0 and
        # demoting genuinely flat faces to "varying".
        return self.luminance_range == 0.0

    @property
    def perceptual_priority(self) -> float:
        """Priority score for face promotion: detail richness × visibility.
        
        Combines standard deviation (how much lighting detail exists) with
        mean luminance (how visible that detail is to the player).
        Shadows in well-lit areas score higher than equivalent shadows in
        darkness, because dark-area detail is invisible in-game.
        
        Formula: std_dev × mean_luminance
        
        Using linear mean (not log) gives strong brightness weighting:
        a face with mean=200 gets 40× the weight of mean=5, properly
        penalizing dark walls where detail is invisible.
        """
        if self.is_never_visible or len(self.luminances) < 2:
            return 0.0
        std_dev = self.variance ** 0.5
        mean = self.mean_luminance
        return std_dev * mean


def _decode_color_rgbexp32(r: int, g: int, b: int, exponent: int) -> Tuple[float, float, float]:
    """Decode a ColorRGBExp32 to linear RGB floats.
    
    Format: value = byte_value * 2^exponent
    The exponent is a signed byte (int8).
    """
    scale = 2.0 ** exponent
    return (r * scale, g * scale, b * scale)


def _luminance(r: float, g: float, b: float) -> float:
    """Compute perceptual luminance from linear RGB."""
    return 0.2126 * r + 0.7152 * g + 0.0722 * b


class _LuxelLuminanceCache(dict):
    """Packed little-endian ColorRGBExp32 word -> luminance, decoded on first use.

    Lightmaps repeat a small set of words heavily, so mapping the lump through
    this cache keeps the per-luxel work in C (map + dict hit) instead of a
    Python unpack/decode per luxel.
    """

    def __missing__(self, word: int) -> float:
        exp = word >> 24
        if exp >= 128:
            exp -= 256
        lum = _luminance(*_decode_color_rgbexp32(
            word & 0xFF, (word >> 8) & 0xFF, (word >> 16) & 0xFF, exp))
        self[word] = lum
        return lum


def _quantize_float(val: float, precision: int = 2) -> float:
    """Round a float to a given decimal precision for fuzzy matching."""
    return round(val, precision)


def _canonicalize_plane(nx: float, ny: float, nz: float, dist: float
                        ) -> Tuple[float, float, float, float]:
    """Canonicalize a plane so the first nonzero normal component is positive.
    
    VMF computes normals from vertex winding (cross product), which can point
    either direction. BSP stores planes canonically with a face_side flag.
    This ensures both representations produce the same key for the same plane.
    
    Example: (-1, 0, 0, 3192) → (1, 0, 0, -3192)
    """
    for c in (nx, ny, nz):
        if abs(c) > 1e-6:
            if c < 0:
                return -nx, -ny, -nz, -dist
            break
    return nx, ny, nz, dist


def _make_plane_key(normal: Tuple[float, float, float], dist: float, 
                     face_side: int, material: str) -> tuple:
    """Create a hashable plane key for matching BSP faces to VMF sides.
    
    The face_side flips the plane normal — BSP stores which side of the plane
    the face is on, so we need to account for that to get the actual facing
    direction. After flipping, we canonicalize so the first nonzero normal
    component is always positive.
    """
    nx, ny, nz = normal
    if face_side:
        nx, ny, nz = -nx, -ny, -nz
        dist = -dist
    nx, ny, nz, dist = _canonicalize_plane(nx, ny, nz, dist)
    return (
        _quantize_float(nx, 4),
        _quantize_float(ny, 4),
        _quantize_float(nz, 4),
        _quantize_float(dist, 1),
        material.upper(),
    )


class BSPReader:
    """Reads a compiled BSP file and extracts per-face lightmap data."""

    def __init__(self, filepath: Union[str, Path]):
        self.filepath = Path(filepath)
        self._data: bytes = b''
        self._lumps: List[BSPLump] = []
        self._version: int = 0

    def read(self) -> None:
        """Read and parse the BSP file header."""
        self._data = self.filepath.read_bytes()

        # Parse header
        if len(self._data) < 8:
            raise ValueError(f"File too small to be a BSP: {len(self._data)} bytes")

        ident, version = struct.unpack_from('<II', self._data, 0)
        if ident != IDBSPHEADER:
            raise ValueError(f"Not a VBSP file (ident=0x{ident:08X}, expected 0x{IDBSPHEADER:08X})")

        self._version = version

        # Parse lump directory (64 lumps × 16 bytes each, starting at offset 8)
        self._lumps = []
        for i in range(HEADER_LUMPS):
            offset = 8 + i * 16
            fileofs, filelen, lump_ver, uncomp = struct.unpack_from('<4i', self._data, offset)
            self._lumps.append(BSPLump(fileofs, filelen, lump_ver, uncomp))

    def _get_lump_data(self, lump_id: int) -> bytes:
        """Get raw bytes for a lump."""
        lump = self._lumps[lump_id]
        return self._data[lump.fileofs:lump.fileofs + lump.filelen]

    def read_vertex_count(self) -> int:
        """Read the actual compiled vertex count from LUMP_VERTEXES.
        
        Each dvertex_t is 12 bytes (3 × float32: x, y, z).
        This is the ground-truth unique vertex count after VBSP deduplication.
        """
        data = self._get_lump_data(LUMP_VERTEXES)
        return len(data) // 12

    def get_bsp_lightmapscale(self, faces=None, texinfos=None) -> int:
        """Detect the lightmapscale used when the BSP was compiled.
        
        Reads per-face lightmapscale from texinfo.lightmap_vecs:
          luxelsPerWorldUnit = magnitude(lightmap_vecs[axis][:3])
          lightmapscale = 1.0 / luxelsPerWorldUnit
        
        Returns the most common lightmapscale across all renderable faces.
        """
        if faces is None:
            faces = self.read_faces()
        if texinfos is None:
            texinfos = self.read_texinfos()
        
        from collections import Counter
        import math
        
        scale_counts = Counter()
        for face in faces:
            if face.texinfo < 0 or face.texinfo >= len(texinfos):
                continue
            if face.lightofs < 0:  # skip unlit faces
                continue
            
            ti = texinfos[face.texinfo]
            # lightmap_vecs[0] = (sx, sy, sz, offset) — luxels per world unit
            sv = ti.lightmap_vecs[0]
            mag = math.sqrt(sv[0]**2 + sv[1]**2 + sv[2]**2)
            if mag > 1e-6:
                lm_scale = round(1.0 / mag)
                if lm_scale >= 1:
                    scale_counts[lm_scale] += 1
        
        if not scale_counts:
            return 16  # VBSP default
        
        return scale_counts.most_common(1)[0][0]

    # ─── Lump Parsers ─────────────────────────────────────────────────────────

    def read_planes(self) -> List[BSPPlane]:
        """Read all dplane_t structs (20 bytes each)."""
        data = self._get_lump_data(LUMP_PLANES)
        count = len(data) // 20
        return [
            BSPPlane(normal=(vals[0], vals[1], vals[2]), dist=vals[3], type=vals[4])
            for vals in struct.iter_unpack('<4fi', data[:count * 20])
        ]

    def read_faces(self, use_hdr: bool = True) -> List[BSPFace]:
        """Read all dface_t structs from LUMP_FACES or LUMP_FACES_HDR."""
        # Prefer HDR faces if available and requested
        lump_id = LUMP_FACES
        if use_hdr and self._lumps[LUMP_FACES_HDR].filelen > 0:
            lump_id = LUMP_FACES_HDR

        data = self._get_lump_data(lump_id)
        count = len(data) // 56

        return [
            BSPFace(
                planenum=vals[0],
                side=vals[1],
                on_node=vals[2],
                firstedge=vals[3],
                numedges=vals[4],
                texinfo=vals[5],
                dispinfo=vals[6],
                lightofs=vals[12],
                lightmap_mins=(vals[14], vals[15]),
                lightmap_size=(vals[16], vals[17]),
                styles=(vals[8], vals[9], vals[10], vals[11]),
                area=vals[13],
                orig_face=vals[18],
                smoothing_groups=vals[21],
            )
            for vals in struct.iter_unpack('<HBBihhhh4BifiiiiiHHI', data[:count * 56])
        ]

    def read_face_ids(self) -> List[int]:
        """Read LUMP_FACEIDS — hammerfaceid for each face."""
        data = self._get_lump_data(LUMP_FACEIDS)
        count = len(data) // 2
        return [vals[0] for vals in struct.iter_unpack('<H', data[:count * 2])]

    def read_face_side_ids(self) -> Optional[List[List[int]]]:
        """Read the new LUMP_FACE_SIDEIDS_INDEX + LUMP_FACE_SIDEIDS_DATA lumps.
        
        Returns a list (one entry per face) of lists of VMF side IDs,
        or None if the lumps are not present (older BSP without -emitsideids).
        """
        idx_data = self._get_lump_data(LUMP_FACE_SIDEIDS_INDEX)
        sid_data = self._get_lump_data(LUMP_FACE_SIDEIDS_DATA)
        # Every emitted face writes at least one id, so an empty data lump
        # beside a non-empty index is malformed: treat it as absent rather
        # than as "no face names any side".
        if len(idx_data) == 0 or len(sid_data) == 0:
            return None

        # Each index entry is 8 bytes: int32 firstId, int32 numIds
        num_faces = len(idx_data) // 8
        result: List[List[int]] = []
        for i in range(num_faces):
            first_id, num_ids = struct.unpack_from('<ii', idx_data, i * 8)
            ids = []
            for j in range(num_ids):
                ofs = (first_id + j) * 4
                if ofs + 4 <= len(sid_data):
                    ids.append(struct.unpack_from('<i', sid_data, ofs)[0])
            result.append(ids)
        return result

    def face_side_ids_version(self) -> int:
        """Version of the side-ID lumps (vbsp_lmo's LUMP_FACE_SIDEIDS_VERSION).

        1 or more: every side merged into a face is listed. 0 (or absent): a
        merged detail face lists only its first side (#73), so a side missing
        from the lumps may still be on a face.
        """
        return self._lumps[LUMP_FACE_SIDEIDS_INDEX].version

    def read_texinfos(self) -> List[BSPTexInfo]:
        """Read all texinfo_t structs (72 bytes each)."""
        data = self._get_lump_data(LUMP_TEXINFO)
        count = len(data) // 72

        return [
            BSPTexInfo(
                texture_vecs=(vals[0:4], vals[4:8]),
                lightmap_vecs=(vals[8:12], vals[12:16]),
                flags=vals[16],
                texdata=vals[17],
            )
            for vals in struct.iter_unpack('<8f8fii', data[:count * 72])
        ]

    def read_texdata(self) -> List[BSPTexData]:
        """Read all dtexdata_t structs (32 bytes each)."""
        data = self._get_lump_data(LUMP_TEXDATA)
        count = len(data) // 32

        return [
            BSPTexData(
                reflectivity=(vals[0], vals[1], vals[2]),
                name_string_table_id=vals[3],
                width=vals[4],
                height=vals[5],
                view_width=vals[6],
                view_height=vals[7],
            )
            for vals in struct.iter_unpack('<3f5i', data[:count * 32])
        ]

    def read_material_names(self) -> List[str]:
        """Read material names via texdata → string table → string data chain."""
        table_data = self._get_lump_data(LUMP_TEXDATA_STRING_TABLE)
        table_count = len(table_data) // 4
        offsets = [struct.unpack_from('<i', table_data, i * 4)[0]
                   for i in range(table_count)]

        string_data = self._get_lump_data(LUMP_TEXDATA_STRING_DATA)

        names = []
        for ofs in offsets:
            end = string_data.index(b'\0', ofs) if ofs < len(string_data) else ofs
            names.append(string_data[ofs:end].decode('ascii', errors='replace'))

        return names

    # ─── Geometry Lump Parsers ─────────────────────────────────────────────────

    def read_vertexes(self) -> List[Tuple[float, float, float]]:
        """Read all vertex positions (12 bytes each: 3 × float32)."""
        data = self._get_lump_data(LUMP_VERTEXES)
        count = len(data) // 12
        return [
            (vals[0], vals[1], vals[2])
            for vals in struct.iter_unpack('<3f', data[:count * 12])
        ]

    def read_edges(self) -> List[Tuple[int, int]]:
        """Read all dedge_t structs (4 bytes each: 2 × uint16)."""
        data = self._get_lump_data(LUMP_EDGES)
        count = len(data) // 4
        return [
            (vals[0], vals[1])
            for vals in struct.iter_unpack('<2H', data[:count * 4])
        ]

    def read_surfedges(self) -> List[int]:
        """Read LUMP_SURFEDGES — signed int32 edge indices.
        
        Positive = edge traversed v0→v1, negative = reversed (v1→v0).
        """
        data = self._get_lump_data(LUMP_SURFEDGES)
        count = len(data) // 4
        return [vals[0] for vals in struct.iter_unpack('<i', data[:count * 4])]

    def read_visibility(self) -> Optional[Tuple[int, List[Tuple[int, int]], bytes]]:
        """Read LUMP_VISIBILITY — PVS compressed bitsets.
        
        Returns (num_clusters, cluster_offsets, raw_data) or None if empty.
        cluster_offsets[i] = (pvs_offset, pas_offset).
        """
        data = self._get_lump_data(LUMP_VISIBILITY)
        if len(data) < 4:
            return None
        num_clusters = struct.unpack_from('<i', data, 0)[0]
        if num_clusters <= 0:
            return None
        offsets = []
        for i in range(num_clusters):
            ofs = 4 + i * 8
            if ofs + 8 > len(data):
                break
            pvs_ofs, pas_ofs = struct.unpack_from('<2i', data, ofs)
            offsets.append((pvs_ofs, pas_ofs))
        return (num_clusters, offsets, data)

    def get_face_vertices(self, face: BSPFace,
                          vertexes: List[Tuple[float, float, float]],
                          edges: List[Tuple[int, int]],
                          surfedges: List[int]) -> List[Tuple[float, float, float]]:
        """Reconstruct face polygon from the edge loop.
        
        Each face references a range of surfedges. Each surfedge is a signed
        index into the edge array: positive means v0→v1, negative means v1→v0.
        We collect the first vertex of each directed edge to form the polygon.
        """
        verts = []
        for se_idx in range(face.firstedge, face.firstedge + face.numedges):
            if se_idx >= len(surfedges):
                break
            se = surfedges[se_idx]
            if se >= 0:
                edge = edges[se]
                verts.append(vertexes[edge[0]])
            else:
                edge = edges[-se]
                verts.append(vertexes[edge[1]])
        return verts

    # ─── Lump Parsers ──────────────────────────────────────────────────────────

    def read_lighting(self, use_hdr: bool = True) -> bytes:
        """Read raw lighting lump data.

        Raises ValueError for an LZMA-compressed lump: its filelen is the
        compressed size and this reader has no decoder, so slicing it would
        silently yield garbage luminances (bsp_has_lighting() reports the
        same condition as unusable).
        """
        lump_id = LUMP_LIGHTING
        if use_hdr and self._lumps[LUMP_LIGHTING_HDR].filelen > 0:
            lump_id = LUMP_LIGHTING_HDR
        if self._lumps[lump_id].uncompressed_size != 0:
            raise ValueError(
                f"{self.filepath.name}: lighting lump {lump_id} is "
                f"LZMA-compressed; decompress the BSP first")
        return self._get_lump_data(lump_id)

    # ─── High-Level Extraction ────────────────────────────────────────────────

    def extract_all_face_data(self, verbose: bool = False) -> List[BSPFaceData]:
        """Extract per-BSP-face lightmap data with plane+material info.
        
        Returns a list of BSPFaceData, one per renderable BSP face that has
        valid lightmap data. Each entry includes the plane normal, distance,
        material name, and decoded luminance samples needed for matching and
        classification.
        """
        use_hdr = self._lumps[LUMP_LIGHTING_HDR].filelen > 0
        faces = self.read_faces(use_hdr=use_hdr)
        face_ids = self.read_face_ids()
        face_side_ids = self.read_face_side_ids()  # New multi-ID lumps (may be None)
        planes = self.read_planes()
        lighting = self.read_lighting(use_hdr=use_hdr)
        texinfos = self.read_texinfos()
        texdata_list = self.read_texdata()
        material_names = self.read_material_names()

        lump_type = "HDR" if use_hdr else "LDR"
        if verbose:
            if face_side_ids is not None:
                print(f"  BSP: {len(faces)} faces, {len(planes)} planes, "
                      f"{len(lighting)} bytes {lump_type} lighting, "
                      f"multi-ID lumps present ({len(face_side_ids)} entries)", flush=True)
            else:
                print(f"  BSP: {len(faces)} faces, {len(planes)} planes, "
                      f"{len(lighting)} bytes {lump_type} lighting", flush=True)

        def get_material(face: BSPFace) -> str:
            if face.texinfo < 0 or face.texinfo >= len(texinfos):
                return "UNKNOWN"
            ti = texinfos[face.texinfo]
            if ti.texdata < 0 or ti.texdata >= len(texdata_list):
                return "UNKNOWN"
            td = texdata_list[ti.texdata]
            if td.name_string_table_id < 0 or td.name_string_table_id >= len(material_names):
                return "UNKNOWN"
            return material_names[td.name_string_table_id]

        results: List[BSPFaceData] = []

        has_lighting = len(lighting) > 0
        # Each luxel is r, g, b, exp bytes; read them as one uint32 word apiece.
        # The word layout assumes a little-endian host with a 4-byte C int;
        # anything else takes the slow path.
        lighting_words = memoryview(lighting)[:len(lighting) // 4 * 4].cast('I')
        fast_decode = sys.byteorder == 'little' and lighting_words.itemsize == 4
        lum_cache = _LuxelLuminanceCache()

        for fi, face in enumerate(faces):
            # Skip displacement surfaces
            if face.dispinfo >= 0:
                continue
                
            ti = texinfos[face.texinfo] if 0 <= face.texinfo < len(texinfos) else None
            
            # If VRAD ran, lightofs < 0 reliably filters unlit/nodraw faces.
            # If VRAD didn't run, lightofs is ALWAYS -1, so we MUST use surface flags.
            if has_lighting:
                if face.lightofs < 0:
                    continue
            else:
                if ti and (ti.flags & (_SURF_SKIP_VIS | SURF_NOLIGHT)):
                    continue

            # Compute luxel count
            width = face.lightmap_size[0] + 1
            height = face.lightmap_size[1] + 1
            luxel_count = width * height
            if luxel_count <= 0:
                continue

            # Read lightmap samples for the first lightstyle
            sample_ofs = face.lightofs
            luminances = []
            
            if has_lighting and sample_ofs >= 0 and sample_ofs + luxel_count * 4 <= len(lighting):
                if fast_decode and sample_ofs % 4 == 0:
                    first = sample_ofs // 4
                    luminances = list(map(lum_cache.__getitem__,
                                          lighting_words[first:first + luxel_count]))
                else:  # VRAD always aligns lightofs; keep the slow path for odd files
                    for li in range(luxel_count):
                        r, g, b, exp = struct.unpack_from('BBBb', lighting, sample_ofs + li * 4)
                        luminances.append(_luminance(*_decode_color_rgbexp32(r, g, b, exp)))

            # Get plane info
            plane = planes[face.planenum] if face.planenum < len(planes) else None
            if plane is None:
                continue

            # Get hammer face ID (may be truncated for IDs > 65535)
            hammer_id = face_ids[fi] if fi < len(face_ids) else 0
            
            # Get all merged side IDs from the new lumps (if available)
            all_ids = None
            if face_side_ids is not None and fi < len(face_side_ids):
                all_ids = face_side_ids[fi] if face_side_ids[fi] else None

            material = get_material(face)

            # Get lightmap vecs from texinfo for spatial mapping
            ti = texinfos[face.texinfo] if 0 <= face.texinfo < len(texinfos) else None
            lm_vecs = ti.lightmap_vecs if ti else None

            results.append(BSPFaceData(
                face_index=fi,
                hammer_id=hammer_id,
                plane_normal=plane.normal,
                plane_dist=plane.dist,
                face_side=face.side,
                material=material,
                luminances=luminances,
                luxel_count=luxel_count,
                area=face.area,
                lightmap_mins=face.lightmap_mins,
                lightmap_size=face.lightmap_size,
                lightmap_vecs=lm_vecs,
                all_hammer_ids=all_ids,
            ))

        if verbose:
            print(f"  Extracted lightmap data for {len(results)} renderable faces",
                  flush=True)

        return results


def match_bsp_to_vmf(bsp_faces: List[BSPFaceData],
                      vmf_sides: dict,
                      verbose: bool = False) -> Dict[int, FaceLightmapData]:
    """Match BSP faces to VMF sides using two-phase strategy.
    
    Phase 1: Direct hammerfaceid match (fast, reliable for IDs ≤ 65535)
    Phase 2: Plane normal + distance + material match (fallback for truncated IDs)
    
    Args:
        bsp_faces: Per-BSP-face data from extract_all_face_data()
        vmf_sides: Dict of VMF side ID → dict with keys:
                   'normal': (nx, ny, nz), 'dist': float, 'material': str
        verbose: Print matching statistics
        
    Returns:
        Dict mapping VMF side ID → FaceLightmapData
    """
    result: Dict[int, FaceLightmapData] = {}
    matched_bsp = set()  # Track which BSP faces have been matched
    
    vmf_id_set = set(vmf_sides.keys())
    
    # ─── Phase 1: Direct hammerfaceid match ───────────────────────────────────
    for bfd in bsp_faces:
        # Prefer the full merged side ID list from the new lumps
        ids_to_check = bfd.all_hammer_ids if bfd.all_hammer_ids else (
            [bfd.hammer_id] if bfd.hammer_id > 0 else [])
        for hid in ids_to_check:
            if hid > 0 and hid in vmf_id_set:
                matched_bsp.add(bfd.face_index)
                _add_to_result(result, hid, bfd)
    
    phase1_matched = len(matched_bsp)
    
    # ─── Phase 2: Plane + material matching for unmatched faces ───────────────
    # Build a lookup from VMF side's plane key → list of VMF side IDs
    vmf_plane_index: Dict[tuple, List[int]] = {}
    for sid, info in vmf_sides.items():
        if sid in result:
            continue  # Already matched in phase 1
        key = (
            _quantize_float(info['normal'][0], 4),
            _quantize_float(info['normal'][1], 4),
            _quantize_float(info['normal'][2], 4),
            _quantize_float(info['dist'], 1),
            info['material'].upper(),
        )
        # Canonicalize so both VMF and BSP produce the same key for the same plane
        cnx, cny, cnz, cdist = _canonicalize_plane(
            key[0], key[1], key[2], key[3])
        key = (cnx, cny, cnz, cdist, key[4])
        vmf_plane_index.setdefault(key, []).append(sid)
    
    phase2_matched = 0
    for bfd in bsp_faces:
        if bfd.face_index in matched_bsp:
            continue
        
        key = _make_plane_key(bfd.plane_normal, bfd.plane_dist,
                               bfd.face_side, bfd.material)
        candidates = vmf_plane_index.get(key, [])
        if candidates:
            # If multiple VMF sides share the same plane+material, assign to all
            # (they're the same surface, split in the VMF for organizational reasons)
            for vmf_id in candidates:
                _add_to_result(result, vmf_id, bfd)
            matched_bsp.add(bfd.face_index)
            phase2_matched += 1
    
    unmatched = len(bsp_faces) - len(matched_bsp)
    
    if verbose:
        print(f"  Phase 1 (face ID):  {phase1_matched} BSP faces matched",
              flush=True)
        print(f"  Phase 2 (plane):    {phase2_matched} BSP faces matched",
              flush=True)
        print(f"  Unmatched:          {unmatched} BSP faces", flush=True)
    
    print(f"  Matched {len(result)} VMF sides from {len(bsp_faces)} BSP faces "
          f"({len(matched_bsp)} matched)", flush=True)
    
    return result


def _add_to_result(result: Dict[int, FaceLightmapData],
                    vmf_id: int, bfd: BSPFaceData) -> None:
    """Add a BSP face's lightmap data to the result dict."""
    # Build per-sub-face info for spatial analysis. Nothing mutates a sub-face's
    # luminances, so it shares the BSP face's list instead of copying it.
    sub_lums = bfd.luminances
    # An empty list is UNKNOWN (truncated sample range), not uniform.
    sub_uniform = bool(sub_lums) and (len(sub_lums) < 2 or
                                      (max(sub_lums) - min(sub_lums) == 0.0))
    sub_info = BSPSubFaceInfo(
        face_index=bfd.face_index,
        lightmap_mins=bfd.lightmap_mins,
        lightmap_size=bfd.lightmap_size,
        lightmap_vecs=bfd.lightmap_vecs,
        luminances=sub_lums,
        is_uniform=sub_uniform,
    )

    if vmf_id in result:
        entry = result[vmf_id]
        entry.bsp_face_indices.append(bfd.face_index)
        entry.luminances.extend(bfd.luminances)
        entry._stats = None
        entry.lightmap_total_luxels += bfd.luxel_count
        entry.sub_faces.append(sub_info)
    else:
        result[vmf_id] = FaceLightmapData(
            vmf_side_id=vmf_id,
            material=bfd.material,
            bsp_face_indices=[bfd.face_index],
            luminances=list(bfd.luminances),
            lightmap_total_luxels=bfd.luxel_count,
            sub_faces=[sub_info],
        )
