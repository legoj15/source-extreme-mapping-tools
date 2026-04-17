"""
PHY Reader — parse Source Engine .phy VPhysics collision model files.

Extracts convex hull vertices and triangles from the IVP compact surface format
stored inside .phy files. These files are loaded from VPK archives or loose files
alongside the corresponding .mdl.

Binary layout (per phyfile.h + studiobyteswap.cpp):
    phyheader_t (16 bytes):
        size        (int32)  — always 16 (sizeof header)
        id          (int32)  — format identifier
        solidCount  (int32)  — number of collision solids
        checkSum    (int32)  — matches .mdl checksum

    Per solid (× solidCount):
        size        (int32)  — byte size of the solid blob (excluding this field)
        <solid blob of `size` bytes>

    Each solid blob starts with a "compact surface header" (28 bytes):
        vphysicsID  (int32)  — 'VPHY' = 0x56504859
        version     (int16)  — should be 3 or higher
        modelType   (int16)  — 0 = COLLIDE_POLY, 1 = COLLIDE_MOPP
        surfaceSize (int32)  — size of the IVP compact surface data
        dragAxisAreas (float[3]) — drag axis areas
        axisMapSize (int32)  — size of the axis map

    After the compact surface header, the IVP compact surface data:
        Starts with an IVP_Compact_Surface header:
            mass_center        (float[3])   — center of mass
            rotation_inertia   (float[3])   — rotation inertia tensor diagonal
            upper_limit_radius (float)      — bounding sphere radius
            max_factor_surface_deviation (int32) — packed max deviation
            byte_size          (int32)      — total byte size of compact surface (including this header)
            offset_ledgetree_root (int32)   — offset from this header to root ledge tree node
            dummy[3]           (int32)      — reserved
            
        The ledge tree is a binary tree. Each node:
            offset_right_node  (int32) — relative offset to right child (0 = leaf/ledge)
            offset_compact_ledge (int32) — relative offset to the ledge data
            center/radius packed (16 bytes)

        Each compact ledge (convex hull):
            c_point_offset  (int32) — offset back to the compact surface start (for points lookup)
            (header fields)
            n_triangles     (int32) — at ledge+0x14
            (other fields)
            Followed by `n_triangles` compact triangles, each 16 bytes:
                tri_index (uint32) — packed: 3 vertex indices × 8 bits each + edge flags
                (other packed data)

        Vertices are stored as:
            IVP_Compact_Poly_Point (16 bytes each):
                x, y, z, dummy (float × 4)
"""
from __future__ import annotations

import struct
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

Vec3 = Tuple[float, float, float]


@dataclass
class PHYSolid:
    """A single solid (convex decomposition) from a .phy file."""
    vertices: List[Vec3] = field(default_factory=list)
    triangles: List[Tuple[int, int, int]] = field(default_factory=list)


@dataclass
class PHYModel:
    """Parsed .phy file containing one or more collision solids."""
    solid_count: int = 0
    checksum: int = 0
    solids: List[PHYSolid] = field(default_factory=list)
    key_values: str = ""


class PHYReader:
    """Parse .phy VPhysics collision models into convex hull geometry.
    
    The IVP compact surface format stores convex hulls as a binary tree of
    "ledges" (convex hull definitions). Each ledge contains triangles that
    reference vertices stored in the compact surface's point array.
    
    The vertex layout is IVP_Compact_Poly_Point: (x, y, z, dummy) each float32.
    Triangle indices are packed into 32-bit words with 8 bits per vertex index.
    """

    VPHY_MAGIC = 0x59485056  # ASCII 'VPHY' read as little-endian uint32
    
    def __init__(self, data: bytes):
        self.data = data
        self.model = PHYModel()
        self._parse()
    
    def _parse(self):
        """Parse the complete .phy file."""
        if len(self.data) < 16:
            raise ValueError(f"PHY file too small: {len(self.data)} bytes")
        
        # Parse phyheader_t
        size, id_val, solid_count, checksum = struct.unpack_from('<4i', self.data, 0)
        
        if size != 16:
            raise ValueError(f"Invalid PHY header size: {size} (expected 16)")
        if solid_count <= 0:
            raise ValueError(f"Invalid solid count: {solid_count}")
        
        self.model.solid_count = solid_count
        self.model.checksum = checksum
        
        # Parse each solid
        offset = size  # Start after phyheader_t (at offset 16)
        
        for i in range(solid_count):
            if offset + 4 > len(self.data):
                break
            
            # Each solid starts with a 4-byte size prefix
            solid_size = struct.unpack_from('<i', self.data, offset)[0]
            offset += 4
            
            if solid_size <= 0 or offset + solid_size > len(self.data):
                break
            
            solid_data = self.data[offset:offset + solid_size]
            solid = self._parse_solid(solid_data)
            if solid:
                self.model.solids.append(solid)
            
            offset += solid_size
        
        # After all solids, the remainder is key-value text data
        if offset < len(self.data):
            try:
                self.model.key_values = self.data[offset:].decode('ascii', errors='replace')
            except Exception:
                pass
    
    def _parse_solid(self, data: bytes) -> Optional[PHYSolid]:
        """Parse a single collision solid from its binary blob.
        
        Layout: VPHY header (28 bytes) + IVP preamble (~44 bytes) + IVPS block.
        The IVPS block contains compact ledges with triangles.
        Vertices are stored right AFTER the IVPS block.
        """
        if len(data) < 28:
            return None
        
        # Parse VPHY compact surface header (28 bytes)
        vphy_id = struct.unpack_from('<I', data, 0)[0]
        version, model_type = struct.unpack_from('<hh', data, 4)
        
        if vphy_id != self.VPHY_MAGIC:
            return None
        
        if model_type != 0:  # Only handle COLLIDE_POLY (type 0)
            return None
        
        # Find the IVPS magic within the solid data
        ivps_ofs = data.find(b'IVPS')
        if ivps_ofs < 0 or ivps_ofs + 24 > len(data):
            return None
        
        # IVPS block: magic(4) + byte_size(4) + payload
        ivps_byte_size = struct.unpack_from('<i', data, ivps_ofs + 4)[0]
        if ivps_byte_size <= 0 or ivps_ofs + 8 + ivps_byte_size > len(data):
            return None
        
        # Vertex data (IVP_Compact_Poly_Point array) starts after IVPS block
        vertex_base = ivps_ofs + 8 + ivps_byte_size
        
        solid = PHYSolid()
        
        # Scan the IVPS block for compact ledges
        # Each compact ledge has a 24-byte header followed by n_triangles × 16 bytes
        # The header has n_triangles at offset +16 (uint16) and c_point_offset at +0
        self._extract_ledges_from_ivps(data, ivps_ofs, ivps_byte_size, 
                                        vertex_base, solid)
        
        return solid if solid.triangles else None
    
    def _extract_ledges_from_ivps(self, data: bytes, ivps_ofs: int, 
                                    ivps_byte_size: int, vertex_base: int,
                                    solid: PHYSolid):
        """Scan the IVPS block to find and extract all compact ledges.
        
        The IVPS block contains a mix of ledge-tree nodes and compact ledges.
        We identify compact ledges by scanning for the pattern:
        - Offset +16 (uint16): n_triangles (1..500)
        - Followed by n_triangles valid 16-byte compact triangle entries
        - Triangle vertex indices should all be within a reasonable range
        
        Using the flat scan + validation approach from the IVPS+24 layout.
        """
        ivps_end = ivps_ofs + 8 + ivps_byte_size
        
        # Count available vertices after IVPS block
        remaining_bytes = len(data) - vertex_base
        max_vertices = remaining_bytes // 16
        
        if max_vertices <= 0:
            return
        
        # Optimize: Instead of calling struct.unpack_from in a slow python for loop,
        # use struct.iter_unpack on a slice and process with a list comprehension.
        # This bypasses python loop overhead and accelerates parsing significantly.
        # Ensure we truncate data cleanly to a multiple of the struct format size.
        byte_limit = vertex_base + max_vertices * 16
        if byte_limit > len(data):
            byte_limit = vertex_base + ((len(data) - vertex_base) // 16) * 16

        vertices = [
            (x, y, z)
            for x, y, z, _dummy in struct.iter_unpack('<4f', data[vertex_base:byte_limit])
        ]
        
        if not vertices:
            return
        
        # The vertices array is shared across all compact ledges
        # Copy all vertices into solid
        solid.vertices = list(vertices)
        
        # Now scan for compact ledge headers in the IVPS block
        # A compact ledge header is 24 bytes with n_triangles at +16
        # We look for entries where reading n_triangles at +16 gives a valid
        # count AND the subsequent triangle data has valid vertex indices
        
        # Strategy: try every 208-byte boundary first (common spacing for 
        # ledges with 12 triangles: 24 header + 12×16 = 208 bytes), 
        # then fall back to scanning every 4-byte boundary
        found_ledges = set()  # Track (offset, n_tri) tuples we've extracted
        
        self._scan_for_ledges(data, ivps_ofs + 8, ivps_end, 
                               len(vertices), solid, found_ledges)
    
    def _scan_for_ledges(self, data: bytes, scan_start: int, scan_end: int,
                          n_vertices: int, solid: PHYSolid, 
                          found_ledges: set):
        """Scan for compact ledge headers and extract triangles.
        
        IVP_Compact_Ledge header (24 bytes):
            +0:  int c_point_offset  (offset back to the point array)
            +4:  short has_children_flag
            +6:  short dummy1
            +8:  int size_div16  (probably ledge size / 16)
            +12: int dummy2
            +16: short n_triangles
            +18: short dummy3
            +20: int dummy4
        
        Followed by n_triangles × IVP_Compact_Triangle (16 bytes each):
            +0:  uint16 start_point_index_0
            +2:  uint16 opposite_index_0
            +4:  uint16 start_point_index_1
            +6:  uint16 opposite_index_1
            +8:  uint16 start_point_index_2
            +10: uint16 opposite_index_2
            +12: int material_index
        """
        offset = scan_start
        while offset + 24 <= scan_end:
            if offset + 18 > len(data):
                break
            
            # Read candidate n_triangles at +16
            n_tri = struct.unpack_from('<H', data, offset + 16)[0]
            
            if n_tri < 1 or n_tri > 500:
                offset += 4
                continue
            
            # Check that triangles fit within the IVPS block
            tri_start = offset + 24
            tri_end = tri_start + n_tri * 16
            if tri_end > scan_end or tri_end > len(data):
                offset += 4
                continue
            
            # Validate: all vertex indices must be valid
            all_valid = True
            max_idx = 0
            for t in range(n_tri):
                tofs = tri_start + t * 16
                idx0 = struct.unpack_from('<H', data, tofs)[0]
                idx1 = struct.unpack_from('<H', data, tofs + 4)[0]
                idx2 = struct.unpack_from('<H', data, tofs + 8)[0]
                m = max(idx0, idx1, idx2)
                if m >= n_vertices:
                    all_valid = False
                    break
                max_idx = max(max_idx, m)
            
            if not all_valid or max_idx == 0:
                offset += 4
                continue
            
            # Check for duplicate ledge (same offset+n_tri)
            key = (offset, n_tri)
            if key in found_ledges:
                offset += 4
                continue
            
            # Additional validation: vertex indices should form distinct triangles
            # (not all zeros or all same index)
            has_diverse = False
            for t in range(min(n_tri, 3)):
                tofs = tri_start + t * 16
                idx0 = struct.unpack_from('<H', data, tofs)[0]
                idx1 = struct.unpack_from('<H', data, tofs + 4)[0]
                idx2 = struct.unpack_from('<H', data, tofs + 8)[0]
                if idx0 != idx1 or idx1 != idx2:
                    has_diverse = True
                    break
            
            if not has_diverse:
                offset += 4
                continue
            
            # Valid ledge! Extract triangles
            found_ledges.add(key)
            # Optimize: Replace python loop and struct.unpack_from with a bulk iter_unpack
            # extraction of the triangles array. Format '<HxxHxxHxxxxxx' skips unused fields
            # and extracts exactly the 3 vertex indices from each 16 byte IVP_Compact_Triangle.
            solid.triangles.extend(
                (idx0, idx1, idx2)
                for idx0, idx1, idx2 in struct.iter_unpack('<HxxHxxHxxxxxx', data[tri_start:tri_end])
            )
            
            # Skip past this ledge's data
            offset = tri_end
            continue
        
    
    def get_model(self) -> PHYModel:
        """Return the parsed PHY model."""
        return self.model
    
    def get_all_triangles(self) -> List[Tuple[Vec3, Vec3, Vec3]]:
        """Get all collision triangles across all solids as world-space vertex triples."""
        triangles = []
        for solid in self.model.solids:
            for i0, i1, i2 in solid.triangles:
                if i0 < len(solid.vertices) and i1 < len(solid.vertices) and i2 < len(solid.vertices):
                    triangles.append((solid.vertices[i0], solid.vertices[i1], solid.vertices[i2]))
        return triangles
    
    def get_all_vertices(self) -> List[Vec3]:
        """Get all unique collision vertices across all solids."""
        vertices = []
        for solid in self.model.solids:
            vertices.extend(solid.vertices)
        return vertices
    
    @staticmethod
    def transform_point(point: Vec3, origin: Vec3, angles: Tuple[float, float, float]) -> Vec3:
        """Transform a point by origin + angles (pitch, yaw, roll in degrees).
        
        Source Engine uses (pitch, yaw, roll) = (X, Y, Z) rotation order.
        For static props, angles are the QAngle from the BSP.
        """
        px, py, pz = point
        
        # Convert angles to radians
        pitch = math.radians(angles[0])  # X rotation
        yaw = math.radians(angles[1])    # Y rotation (around Z axis)
        roll = math.radians(angles[2])   # Z rotation
        
        # Source Engine rotation order: Yaw → Pitch → Roll
        # Build rotation matrix columns
        cy, sy = math.cos(yaw), math.sin(yaw)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cr, sr = math.cos(roll), math.sin(roll)
        
        # Forward, right, up vectors (Source Engine convention)
        m00 = cp * cy
        m01 = cp * sy
        m02 = -sp
        
        m10 = sr * sp * cy - cr * sy
        m11 = sr * sp * sy + cr * cy
        m12 = sr * cp
        
        m20 = cr * sp * cy + sr * sy
        m21 = cr * sp * sy - sr * cy
        m22 = cr * cp
        
        # Apply rotation then translation
        ox, oy, oz = origin
        x = m00 * px + m10 * py + m20 * pz + ox
        y = m01 * px + m11 * py + m21 * pz + oy
        z = m02 * px + m12 * py + m22 * pz + oz
        
        return (x, y, z)


def load_phy_from_file(filepath: str) -> Optional[PHYModel]:
    """Load and parse a .phy file from disk."""
    try:
        with open(filepath, 'rb') as f:
            data = f.read()
        reader = PHYReader(data)
        return reader.get_model()
    except Exception as e:
        print(f"  Warning: Failed to load PHY '{filepath}': {e}")
        return None


def load_phy_from_vpk(vpk_reader, model_path: str) -> Optional[PHYModel]:
    """Load and parse a .phy file from a VPK archive.
    
    Args:
        vpk_reader: A VPKReader instance
        model_path: The model path (e.g., 'models/props/barrel.mdl')
                    Will be converted to .phy extension automatically.
    """
    # Convert .mdl path to .phy path
    phy_path = model_path.rsplit('.', 1)[0] + '.phy'
    
    try:
        data = vpk_reader.read_file(phy_path)
        if data is None:
            return None
        reader = PHYReader(data)
        return reader.get_model()
    except Exception as e:
        print(f"  Warning: Failed to load PHY '{phy_path}' from VPK: {e}")
        return None
