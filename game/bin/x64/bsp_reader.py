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
    LUMP_TEXDATA_STRING_TABLE (43) — int32 (4 bytes)
    LUMP_TEXDATA_STRING_DATA  (44) — null-terminated strings
    LUMP_LIGHTING_HDR    (53) — ColorRGBExp32 (4 bytes each)
    LUMP_FACES_HDR       (58) — dface_t (56 bytes)
"""
from __future__ import annotations

import math
import struct
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
SURF_SKY    = 0x4
SURF_SKY2D  = 0x8
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
class BSPNode:
    """Parsed dnode_t structure (32 bytes)."""
    planenum: int
    children: Tuple[int, int]   # negative = -(leafs+1), not nodes
    mins: Tuple[int, int, int]
    maxs: Tuple[int, int, int]
    firstface: int
    numfaces: int
    area: int


@dataclass
class BSPLeaf:
    """Parsed dleaf_t structure (32 bytes, version 1)."""
    contents: int
    cluster: int
    area: int
    flags: int
    mins: Tuple[int, int, int]
    maxs: Tuple[int, int, int]
    firstleafface: int
    numleaffaces: int
    firstleafbrush: int
    numleafbrushes: int
    leaf_water_data_id: int


@dataclass
class BSPBrush:
    """Parsed dbrush_t structure (12 bytes)."""
    firstside: int
    numsides: int
    contents: int


@dataclass
class BSPBrushSide:
    """Parsed dbrushside_t structure (8 bytes)."""
    planenum: int
    texinfo: int
    dispinfo: int
    bevel: int


@dataclass
class BSPModel:
    """Parsed dmodel_t structure (48 bytes)."""
    mins: Tuple[float, float, float]
    maxs: Tuple[float, float, float]
    origin: Tuple[float, float, float]
    headnode: int
    firstface: int
    numfaces: int


@dataclass
class BSPStaticProp:
    """Parsed static prop from the game lump."""
    origin: Tuple[float, float, float]
    angles: Tuple[float, float, float]
    prop_type: int     # Index into model name dictionary
    model_name: str    # Resolved model name
    solid: int         # Solidity flag (0=not solid, 2=bounding box, 6=vphysics)
    first_leaf: int
    leaf_count: int
    skin: int
    flags: int


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

    @property
    def max_luminance(self) -> float:
        return max(self.luminances) if self.luminances and not self.is_never_visible else 0.0

    @property
    def min_luminance(self) -> float:
        return min(self.luminances) if self.luminances and not self.is_never_visible else 0.0

    @property
    def mean_luminance(self) -> float:
        return sum(self.luminances) / len(self.luminances) if self.luminances and not self.is_never_visible else 0.0

    @property
    def variance(self) -> float:
        if self.is_never_visible or len(self.luminances) < 2:
            return 0.0
        mean = self.mean_luminance
        return sum((x - mean) ** 2 for x in self.luminances) / len(self.luminances)

    @property
    def luminance_range(self) -> float:
        """Max - min luminance. A simpler measure of variation than variance."""
        if self.is_never_visible or not self.luminances:
            return 0.0
        return self.max_luminance - self.min_luminance

    @property
    def is_perfectly_uniform(self) -> bool:
        """True if every luxel has exactly the same luminance (zero variation)."""
        if self.is_never_visible or len(self.luminances) < 2:
            return True
        return self.variance == 0.0 and self.luminance_range == 0.0

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

    def is_monotonic_gradient(self, tolerance: float = 0.5) -> bool:
        """True if the lightmap has a smooth monotonic gradient along S or T.
        
        Args:
            tolerance: Maximum luminance difference between adjacent luxels
                       in the "wrong" direction before monotonicity is broken.
        
        Returns False if already perfectly uniform (handled separately).
        Returns False if fewer than 4 luxels (not enough for a gradient).
        For multi-sub-face entries, all sub-faces must be monotonic with
        a consistent gradient direction along the same axis.
        """
        if self.is_never_visible:
            return False
        if self.is_perfectly_uniform:
            return False
        if len(self.luminances) < 4:
            return False
        
        # Use sub-face spatial data if available
        if self.sub_faces:
            for sf in self.sub_faces:
                if sf.is_uniform:
                    continue  # Uniform sub-faces are compatible with any gradient
                w = sf.lightmap_size[0] + 1
                h = sf.lightmap_size[1] + 1
                if len(sf.luminances) != w * h:
                    return False
                if not _check_monotonic_2d(sf.luminances, w, h, tolerance):
                    return False
            return True
        
        # Fallback: no sub-face data, cannot determine 2D layout
        return False


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


def _is_monotonic_1d(values: List[float], tolerance: float = 0.5) -> bool:
    """Check if a 1D sequence is monotonically non-decreasing or non-increasing.
    
    Adjacent values may differ by up to `tolerance` in the "wrong" direction
    without breaking monotonicity (absorbs ColorRGBExp32 quantization noise).
    """
    if len(values) < 2:
        return True
    increasing = True
    decreasing = True
    for i in range(1, len(values)):
        diff = values[i] - values[i - 1]
        if diff < -tolerance:
            increasing = False
        if diff > tolerance:
            decreasing = False
        if not increasing and not decreasing:
            return False
    return True


def _check_monotonic_2d(luminances: List[float], width: int, height: int,
                        tolerance: float = 0.5) -> bool:
    """Check if a 2D lightmap grid has a gentle monotonic gradient along at least one axis.
    
    Reshapes the flat luminance list into a height×width grid, then checks:
      - S-axis gradient: every row is monotonic in the same direction
      - T-axis gradient: every column is monotonic in the same direction
    
    Additionally applies a steepness check: if any window of 3 consecutive
    luxels along any row or column has a total luminance change > 1.0, the
    gradient is considered too steep to safely downscale (e.g. sharp shadow
    edges that happen to be monotonic).
    
    Returns True if either axis has a consistent monotonic pattern AND the
    gradient is gentle enough.
    """
    if len(luminances) != width * height:
        return False
    if width < 2 and height < 2:
        return False  # Need at least 2 luxels along one axis
    
    # Build 2D grid: grid[row][col]
    grid = []
    for r in range(height):
        row = luminances[r * width : (r + 1) * width]
        grid.append(row)
    
    # ── Steepness check: reject if any 3-luxel window changes > 1.0 ──────────
    # Check rows
    for row in grid:
        for i in range(len(row) - 2):
            if abs(row[i + 2] - row[i]) > 1.0:
                return False
    # Check columns
    for c in range(width):
        for r in range(height - 2):
            if abs(grid[r + 2][c] - grid[r][c]) > 1.0:
                return False
    
    # Check S-axis (rows): every row must be monotonic in the same direction
    if width >= 2:
        s_ok = True
        # Determine direction from first row with a meaningful span
        s_direction = 0  # 0=undecided, 1=increasing, -1=decreasing
        for row in grid:
            if not _is_monotonic_1d(row, tolerance):
                s_ok = False
                break
            span = row[-1] - row[0]
            if abs(span) > tolerance:
                row_dir = 1 if span > 0 else -1
                if s_direction == 0:
                    s_direction = row_dir
                elif row_dir != s_direction:
                    s_ok = False
                    break
        if s_ok and s_direction != 0:
            return True
    
    # Check T-axis (columns): every column must be monotonic in the same direction
    if height >= 2:
        t_ok = True
        t_direction = 0
        for c in range(width):
            col = [grid[r][c] for r in range(height)]
            if not _is_monotonic_1d(col, tolerance):
                t_ok = False
                break
            span = col[-1] - col[0]
            if abs(span) > tolerance:
                col_dir = 1 if span > 0 else -1
                if t_direction == 0:
                    t_direction = col_dir
                elif col_dir != t_direction:
                    t_ok = False
                    break
        if t_ok and t_direction != 0:
            return True
    
    return False


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
        planes = []
        for i in range(count):
            ofs = i * 20
            nx, ny, nz, dist, ptype = struct.unpack_from('<4fi', data, ofs)
            planes.append(BSPPlane(normal=(nx, ny, nz), dist=dist, type=ptype))
        return planes

    def read_faces(self, use_hdr: bool = True) -> List[BSPFace]:
        """Read all dface_t structs from LUMP_FACES or LUMP_FACES_HDR."""
        # Prefer HDR faces if available and requested
        lump_id = LUMP_FACES
        if use_hdr and self._lumps[LUMP_FACES_HDR].filelen > 0:
            lump_id = LUMP_FACES_HDR

        data = self._get_lump_data(lump_id)
        count = len(data) // 56
        faces = []

        for i in range(count):
            ofs = i * 56
            (planenum, side, on_node, firstedge, numedges, texinfo,
             dispinfo, _fog, s0, s1, s2, s3, lightofs, area,
             lm_min_s, lm_min_t, lm_size_s, lm_size_t,
             orig_face, _nprims, _firstprim, smoothing) = struct.unpack_from(
                '<HBBihhhh4BifiiiiiHHI', data, ofs)

            faces.append(BSPFace(
                planenum=planenum,
                side=side,
                on_node=on_node,
                firstedge=firstedge,
                numedges=numedges,
                texinfo=texinfo,
                dispinfo=dispinfo,
                lightofs=lightofs,
                lightmap_mins=(lm_min_s, lm_min_t),
                lightmap_size=(lm_size_s, lm_size_t),
                styles=(s0, s1, s2, s3),
                area=area,
                orig_face=orig_face,
                smoothing_groups=smoothing,
            ))

        return faces

    def read_face_ids(self) -> List[int]:
        """Read LUMP_FACEIDS — hammerfaceid for each face."""
        data = self._get_lump_data(LUMP_FACEIDS)
        count = len(data) // 2
        return [struct.unpack_from('<H', data, i * 2)[0] for i in range(count)]

    def read_face_side_ids(self) -> Optional[List[List[int]]]:
        """Read the new LUMP_FACE_SIDEIDS_INDEX + LUMP_FACE_SIDEIDS_DATA lumps.
        
        Returns a list (one entry per face) of lists of VMF side IDs,
        or None if the lumps are not present (older BSP without -emitsideids).
        """
        idx_data = self._get_lump_data(LUMP_FACE_SIDEIDS_INDEX)
        sid_data = self._get_lump_data(LUMP_FACE_SIDEIDS_DATA)
        if len(idx_data) == 0:
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

    def read_texinfos(self) -> List[BSPTexInfo]:
        """Read all texinfo_t structs (72 bytes each)."""
        data = self._get_lump_data(LUMP_TEXINFO)
        count = len(data) // 72
        texinfos = []

        for i in range(count):
            ofs = i * 72
            vals = struct.unpack_from('<8f8fii', data, ofs)
            tex_vecs = (vals[0:4], vals[4:8])
            lm_vecs = (vals[8:12], vals[12:16])
            flags = vals[16]
            texdata = vals[17]
            texinfos.append(BSPTexInfo(
                texture_vecs=tex_vecs,
                lightmap_vecs=lm_vecs,
                flags=flags,
                texdata=texdata,
            ))

        return texinfos

    def read_texdata(self) -> List[BSPTexData]:
        """Read all dtexdata_t structs (32 bytes each)."""
        data = self._get_lump_data(LUMP_TEXDATA)
        count = len(data) // 32
        texdata_list = []

        for i in range(count):
            ofs = i * 32
            vals = struct.unpack_from('<3f5i', data, ofs)
            texdata_list.append(BSPTexData(
                reflectivity=(vals[0], vals[1], vals[2]),
                name_string_table_id=vals[3],
                width=vals[4],
                height=vals[5],
                view_width=vals[6],
                view_height=vals[7],
            ))

        return texdata_list

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
        verts = []
        for i in range(count):
            x, y, z = struct.unpack_from('<3f', data, i * 12)
            verts.append((x, y, z))
        return verts

    def read_edges(self) -> List[Tuple[int, int]]:
        """Read all dedge_t structs (4 bytes each: 2 × uint16)."""
        data = self._get_lump_data(LUMP_EDGES)
        count = len(data) // 4
        edges = []
        for i in range(count):
            v0, v1 = struct.unpack_from('<2H', data, i * 4)
            edges.append((v0, v1))
        return edges

    def read_surfedges(self) -> List[int]:
        """Read LUMP_SURFEDGES — signed int32 edge indices.
        
        Positive = edge traversed v0→v1, negative = reversed (v1→v0).
        """
        data = self._get_lump_data(LUMP_SURFEDGES)
        count = len(data) // 4
        return [struct.unpack_from('<i', data, i * 4)[0] for i in range(count)]

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

    # ─── Collision Lump Parsers ────────────────────────────────────────────────

    def read_nodes(self) -> List[BSPNode]:
        """Read all dnode_t structs (32 bytes each)."""
        data = self._get_lump_data(LUMP_NODES)
        count = len(data) // 32
        nodes = []
        for i in range(count):
            ofs = i * 32
            vals = struct.unpack_from('<3i3h3h2Hh2x', data, ofs)
            nodes.append(BSPNode(
                planenum=vals[0],
                children=(vals[1], vals[2]),
                mins=(vals[3], vals[4], vals[5]),
                maxs=(vals[6], vals[7], vals[8]),
                firstface=vals[9],
                numfaces=vals[10],
                area=vals[11],
            ))
        return nodes

    def read_leafs(self) -> List[BSPLeaf]:
        """Read all dleaf_t structs (32 bytes each, version 1).
        
        The area (9 bits) and flags (7 bits) are packed into a single short
        via a bitfield in the C++ struct.
        """
        data = self._get_lump_data(LUMP_LEAFS)
        count = len(data) // 32
        leafs = []
        for i in range(count):
            ofs = i * 32
            (contents, cluster, area_flags,
             min0, min1, min2, max0, max1, max2,
             firstleafface, numleaffaces,
             firstleafbrush, numleafbrushes,
             leaf_water) = struct.unpack_from('<ih h 3h3h 4Hh', data, ofs)
            # Unpack the bitfield: lower 9 bits = area, upper 7 bits = flags
            area = area_flags & 0x1FF
            flags = (area_flags >> 9) & 0x7F
            leafs.append(BSPLeaf(
                contents=contents,
                cluster=cluster,
                area=area,
                flags=flags,
                mins=(min0, min1, min2),
                maxs=(max0, max1, max2),
                firstleafface=firstleafface,
                numleaffaces=numleaffaces,
                firstleafbrush=firstleafbrush,
                numleafbrushes=numleafbrushes,
                leaf_water_data_id=leaf_water,
            ))
        return leafs

    def read_brushes(self) -> List[BSPBrush]:
        """Read all dbrush_t structs (12 bytes each)."""
        data = self._get_lump_data(LUMP_BRUSHES)
        count = len(data) // 12
        brushes = []
        for i in range(count):
            ofs = i * 12
            firstside, numsides, contents = struct.unpack_from('<3i', data, ofs)
            brushes.append(BSPBrush(
                firstside=firstside,
                numsides=numsides,
                contents=contents,
            ))
        return brushes

    def read_brushsides(self) -> List[BSPBrushSide]:
        """Read all dbrushside_t structs (8 bytes each)."""
        data = self._get_lump_data(LUMP_BRUSHSIDES)
        count = len(data) // 8
        sides = []
        for i in range(count):
            ofs = i * 8
            planenum, texinfo, dispinfo, bevel = struct.unpack_from('<Hhhh', data, ofs)
            sides.append(BSPBrushSide(
                planenum=planenum,
                texinfo=texinfo,
                dispinfo=dispinfo,
                bevel=bevel,
            ))
        return sides

    def read_models(self) -> List[BSPModel]:
        """Read all dmodel_t structs (48 bytes each)."""
        data = self._get_lump_data(LUMP_MODELS)
        count = len(data) // 48
        models = []
        for i in range(count):
            ofs = i * 48
            vals = struct.unpack_from('<9f3i', data, ofs)
            models.append(BSPModel(
                mins=(vals[0], vals[1], vals[2]),
                maxs=(vals[3], vals[4], vals[5]),
                origin=(vals[6], vals[7], vals[8]),
                headnode=vals[9],
                firstface=vals[10],
                numfaces=vals[11],
            ))
        return models

    def read_leafbrushes(self) -> List[int]:
        """Read LUMP_LEAFBRUSHES — unsigned short indices into brush array."""
        data = self._get_lump_data(LUMP_LEAFBRUSHES)
        count = len(data) // 2
        return [struct.unpack_from('<H', data, i * 2)[0] for i in range(count)]

    def read_entities(self) -> List[dict]:
        """Parse the entity lump text into a list of key-value dicts.
        
        Each entity is enclosed in braces. Keys and values are quoted strings.
        Returns a list of dicts, one per entity.
        """
        raw = self._get_lump_data(LUMP_ENTITIES)
        text = raw.decode('ascii', errors='replace').rstrip('\x00')
        
        entities = []
        current = None
        
        for line in text.split('\n'):
            line = line.strip()
            if line == '{':
                current = {}
            elif line == '}':
                if current is not None:
                    entities.append(current)
                    current = None
            elif current is not None and line.startswith('"'):
                # Parse "key" "value" format
                parts = line.split('"')
                if len(parts) >= 5:  # "", key, " ", value, ""
                    key = parts[1]
                    value = parts[3]
                    current[key] = value
        
        return entities

    def read_static_props(self) -> List[BSPStaticProp]:
        """Read static props from the GAME_LUMP ('sprp').
        
        The game lump contains a directory of sub-lumps. The 'sprp' sub-lump
        has three sections:
            1. Model dictionary: count + N × 128-byte null-terminated strings
            2. Leaf array: count + N × unsigned short leaf indices  
            3. Static prop array: count + N × StaticPropLump_t entries
        
        The struct size varies by version (v4=56, v5=60, v6=64, v7+=68+).
        We parse the common fields that exist in all versions.
        """
        game_lump_data = self._get_lump_data(LUMP_GAME_LUMP)
        if len(game_lump_data) < 4:
            return []
        
        # Parse game lump header: count of sub-lumps
        lump_count = struct.unpack_from('<i', game_lump_data, 0)[0]
        
        # Find the 'sprp' sub-lump in the directory
        sprp_id = GAMELUMP_STATIC_PROPS
        sprp_offset = None
        sprp_size = None
        sprp_version = None
        
        # Each directory entry: id(4) + flags(2) + version(2) + fileofs(4) + filelen(4) = 16 bytes
        for i in range(lump_count):
            entry_ofs = 4 + i * 16
            if entry_ofs + 16 > len(game_lump_data):
                break
            lump_id, lump_flags, lump_ver, lump_fileofs, lump_filelen = \
                struct.unpack_from('<iHHii', game_lump_data, entry_ofs)
            if lump_id == sprp_id:
                # fileofs is absolute file offset, need to convert to relative
                main_lump = self._lumps[LUMP_GAME_LUMP]
                sprp_offset = lump_fileofs - main_lump.fileofs
                sprp_size = lump_filelen
                sprp_version = lump_ver
                break
        
        if sprp_offset is None or sprp_size is None:
            return []
        
        sprp_data = game_lump_data[sprp_offset:sprp_offset + sprp_size]
        return self._parse_static_prop_lump(sprp_data, sprp_version)
    
    def _parse_static_prop_lump(self, data: bytes, version: int) -> List[BSPStaticProp]:
        """Parse the interior of the 'sprp' game lump."""
        if len(data) < 4:
            return []
        
        offset = 0
        
        # 1. Model dictionary
        dict_count = struct.unpack_from('<i', data, offset)[0]
        offset += 4
        model_names = []
        for i in range(dict_count):
            if offset + 128 > len(data):
                break
            name_bytes = data[offset:offset + 128]
            null_idx = name_bytes.find(b'\x00')
            name = name_bytes[:null_idx].decode('ascii', errors='replace') if null_idx >= 0 else name_bytes.decode('ascii', errors='replace')
            model_names.append(name)
            offset += 128
        
        # 2. Leaf array
        if offset + 4 > len(data):
            return []
        leaf_count = struct.unpack_from('<i', data, offset)[0]
        offset += 4
        offset += leaf_count * 2  # Skip leaf indices (unsigned short each)
        
        # 3. Static prop entries
        if offset + 4 > len(data):
            return []
        prop_count = struct.unpack_from('<i', data, offset)[0]
        offset += 4
        
        # Determine struct size based on version
        # V4: 56B, V5: 60B, V6: 64B, V7/V10: 68B+
        if prop_count == 0:
            return []
        remaining = len(data) - offset
        entry_size = remaining // prop_count if prop_count > 0 else 0
        
        # Fallback: calculate from version
        if entry_size < 44:  # Minimum viable size
            version_sizes = {4: 56, 5: 60, 6: 64, 7: 68, 10: 76}
            entry_size = version_sizes.get(version, 68)
        
        props = []
        for i in range(prop_count):
            ofs = offset + i * entry_size
            if ofs + 44 > len(data):  # Need at least 44 bytes for core fields
                break
            
            # Common fields across all versions (first 44 bytes):
            # Vector origin (12), QAngle angles (12), 
            # ushort propType (2), ushort firstLeaf (2), ushort leafCount (2),
            # uchar solid (1), uchar flags (1),
            # int skin (4), float fadeMinDist (4), float fadeMaxDist (4)
            ox, oy, oz, ax, ay, az, prop_type, first_leaf, leaf_count, \
                solid, flags_byte, skin = struct.unpack_from(
                    '<6f3HBBi', data, ofs)
            
            model_name = model_names[prop_type] if prop_type < len(model_names) else ''
            
            props.append(BSPStaticProp(
                origin=(ox, oy, oz),
                angles=(ax, ay, az),
                prop_type=prop_type,
                model_name=model_name,
                solid=solid,
                first_leaf=first_leaf,
                leaf_count=leaf_count,
                skin=skin,
                flags=flags_byte,
            ))
        
        return props

    def point_in_leaf(self, point: Tuple[float, float, float],
                      nodes: Optional[List[BSPNode]] = None,
                      planes: Optional[List[BSPPlane]] = None) -> int:
        """Walk the BSP tree to find which leaf contains a point.
        
        Returns the leaf index (non-negative).
        """
        if nodes is None:
            nodes = self.read_nodes()
        if planes is None:
            planes = self.read_planes()
        
        node_idx = 0  # Start at root
        while node_idx >= 0:
            node = nodes[node_idx]
            plane = planes[node.planenum]
            # dot(point, normal) - dist
            px, py, pz = point
            nx, ny, nz = plane.normal
            d = px * nx + py * ny + pz * nz - plane.dist
            if d >= 0:
                node_idx = node.children[0]  # Front
            else:
                node_idx = node.children[1]  # Back
        
        # Convert negative index to leaf index: -(node_idx + 1)
        return -(node_idx + 1)

    # ─── Original Lump Parsers ─────────────────────────────────────────────────

    def read_lighting(self, use_hdr: bool = True) -> bytes:
        """Read raw lighting lump data."""
        lump_id = LUMP_LIGHTING
        if use_hdr and self._lumps[LUMP_LIGHTING_HDR].filelen > 0:
            lump_id = LUMP_LIGHTING_HDR
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
                # Optimize: Instead of calling struct.unpack_from in a slow python for loop,
                # use struct.iter_unpack on a slice and process with a list comprehension.
                # Inlines decoding and luminance calculation for maximum performance (~4x faster).
                lighting_slice = lighting[sample_ofs : sample_ofs + luxel_count * 4]
                luminances = [
                    (0.2126 * r + 0.7152 * g + 0.0722 * b) * (2.0 ** exp)
                    for r, g, b, exp in struct.iter_unpack('BBBb', lighting_slice)
                ]

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
    # Build per-sub-face info for spatial analysis
    sub_lums = list(bfd.luminances)
    sub_uniform = (len(sub_lums) < 2 or
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
