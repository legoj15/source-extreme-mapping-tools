"""
Brush Carver — split VMF brushes along uniformity boundaries for aggressive
lightmapscale optimization.

When a single brush face spans both uniformly-lit and variably-lit regions,
the entire face must use the lower lightmapscale. By carving the brush along
the uniformity boundary, each piece gets an independent scale.

VBSP merges coplanar faces with matching material + lightmapscale + texture
alignment, so the carve is transparent when both halves match, and saves
vertices when they don't.

Supported geometry: axis-aligned rectangular brushes only (normals ±X/±Y/±Z).
"""
from __future__ import annotations

import copy
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

from vmf_parser import KVNode, KVPair, VMFBrush, VMFSide

Vec3 = Tuple[float, float, float]

# Axis names for reporting
AXIS_NAMES = ['X', 'Y', 'Z']


# ─── Data structures ────────────────────────────────────────────────────────

@dataclass
class SplitPlane:
    """An axis-aligned split plane."""
    axis: int        # 0=X, 1=Y, 2=Z
    value: float     # World coordinate of the split


@dataclass
class CarveCandidate:
    """A face that could benefit from carving."""
    side_id: int
    brush_id: int
    brush_node: KVNode
    side_node: KVNode
    parent_node: KVNode    # worldspawn or entity node containing the brush
    face_normal_axis: int  # Which axis the face normal is on (0=X, 1=Y, 2=Z)
    face_normal_sign: int  # +1 or -1
    uniform_regions: list  # BSPSubFaceInfo list for uniform sub-faces
    varied_regions: list   # BSPSubFaceInfo list for varied sub-faces
    face_bbox: Tuple[Vec3, Vec3]  # Face world-space bounding box


@dataclass
class CarveResult:
    """Result of a single brush carve."""
    original_brush_id: int
    original_side_id: int
    split_planes: List[SplitPlane]
    new_brush_count: int
    estimated_savings: int  # Vertex savings from the carve
    material: str


# ─── Axis detection ──────────────────────────────────────────────────────────

def _is_axis_aligned_normal(normal: Vec3, tolerance: float = 0.01) -> Optional[Tuple[int, int]]:
    """Check if a normal vector is axis-aligned. Returns (axis, sign) or None."""
    for axis in range(3):
        if abs(abs(normal[axis]) - 1.0) < tolerance:
            sign = 1 if normal[axis] > 0 else -1
            return (axis, sign)
    return None


def _is_axis_aligned_brush(brush: VMFBrush) -> bool:
    """Check if all sides of a brush have axis-aligned normals."""
    from geometry import Plane
    for side in brush.sides:
        if len(side.plane_points) < 3:
            return False
        p = Plane.from_three_points(*side.plane_points)
        if _is_axis_aligned_normal(p.normal) is None:
            return False
    return True


def _get_brush_bbox(brush: VMFBrush) -> Optional[Tuple[Vec3, Vec3]]:
    """Get axis-aligned bounding box of a brush from vertices_plus data."""
    all_verts = []
    for side in brush.sides:
        if side.vertices:
            all_verts.extend(side.vertices)
    if not all_verts:
        return None
    mins = (min(v[0] for v in all_verts),
            min(v[1] for v in all_verts),
            min(v[2] for v in all_verts))
    maxs = (max(v[0] for v in all_verts),
            max(v[1] for v in all_verts),
            max(v[2] for v in all_verts))
    return (mins, maxs)


# ─── Spatial analysis ────────────────────────────────────────────────────────

def _subface_world_range(sub_face, in_plane_axes: Tuple[int, int]
                          ) -> Optional[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """Compute approximate world-space range of a BSP sub-face.
    
    Uses lightmap_vecs to determine which world axis corresponds to
    lightmap S and T, then converts lightmap_mins/size to world coords.
    
    Args:
        sub_face: BSPSubFaceInfo with lightmap spatial data
        in_plane_axes: The two axes that lie in the face plane (e.g. (0,1) for Z-face)
    
    Returns:
        ((axis0_min, axis0_max), (axis1_min, axis1_max)) in world space,
        or None if spatial data is unavailable.
    """
    if sub_face.lightmap_vecs is None:
        return None
    
    lm_vecs = sub_face.lightmap_vecs
    lm_mins = sub_face.lightmap_mins
    lm_size = sub_face.lightmap_size
    
    # For each lightmap axis (S/T), find the dominant world axis
    ranges = {}  # world_axis -> (min, max) in world units
    
    for lm_axis in range(2):  # 0=S, 1=T
        vec = lm_vecs[lm_axis][:3]  # (vx, vy, vz)
        offset = lm_vecs[lm_axis][3]
        
        # Find dominant axis
        dominant = max(range(3), key=lambda a: abs(vec[a]))
        scale = vec[dominant]  # luxels per world unit
        
        if abs(scale) < 1e-8:
            continue
        
        # Convert luxel range to world range
        # luxel = scale * world_coord + offset
        # world_coord = (luxel - offset) / scale
        luxel_min = lm_mins[lm_axis]
        luxel_max = lm_mins[lm_axis] + lm_size[lm_axis]
        
        world_min = (luxel_min - offset) / scale
        world_max = (luxel_max - offset) / scale
        
        if world_min > world_max:
            world_min, world_max = world_max, world_min
        
        ranges[dominant] = (world_min, world_max)
    
    # Return ranges for the two in-plane axes
    a0, a1 = in_plane_axes
    r0 = ranges.get(a0, None)
    r1 = ranges.get(a1, None)
    
    if r0 is None or r1 is None:
        return None
    
    return (r0, r1)


def _cluster_1d(values: List[float], gap_threshold: float = 64.0
                ) -> List[Tuple[float, float]]:
    """Cluster sorted 1D values by gaps. Returns list of (min, max) per cluster."""
    if not values:
        return []
    vals = sorted(values)
    clusters = []
    c_start = vals[0]
    c_end = vals[0]
    for v in vals[1:]:
        if v - c_end > gap_threshold:
            clusters.append((c_start, c_end))
            c_start = v
        c_end = v
    clusters.append((c_start, c_end))
    return clusters


def _find_split_planes(uniform_regions, varied_regions, 
                        in_plane_axes: Tuple[int, int],
                        face_bbox: Tuple[Vec3, Vec3],
                        allow_multi: bool = False) -> List[SplitPlane]:
    """Find axis-aligned split planes that separate uniform from varied regions.
    
    Single-cut mode: best single axis-aligned split.
    
    Multi-cut mode: spatially clusters dark sub-faces, then places boundary
    cuts around EACH cluster independently. Separate shadow regions get
    separate bounding boxes.
    """
    a0, a1 = in_plane_axes
    
    # Compute world-space ranges of each sub-face
    uniform_ranges = []  # list of ((a0_min,a0_max), (a1_min,a1_max))
    
    for sf in uniform_regions:
        r = _subface_world_range(sf, in_plane_axes)
        if r:
            uniform_ranges.append(r)
    
    if not uniform_ranges:
        return []
    
    # ─── Multi-cut: cluster dark sub-faces and bound each cluster ─────────
    # For each in-plane axis, project dark sub-face midpoints, find gaps,
    # and generate lo/hi boundary cuts per cluster.
    all_cuts: List[SplitPlane] = []
    
    for local_idx, world_axis in enumerate(in_plane_axes):
        f_min = face_bbox[0][world_axis]
        f_max = face_bbox[1][world_axis]
        margin = (f_max - f_min) * 0.02
        
        # Get midpoints of dark sub-faces on this axis
        midpoints = [(r[local_idx][0] + r[local_idx][1]) / 2 for r in uniform_ranges]
        
        if allow_multi:
            # Cluster by gaps — separate shadow regions get separate bounds
            # Use gap threshold proportional to sub-face size (~32 world units)
            sub_sizes = [r[local_idx][1] - r[local_idx][0] for r in uniform_ranges]
            avg_size = sum(sub_sizes) / len(sub_sizes) if sub_sizes else 32.0
            gap_thresh = avg_size * 2.5  # Gap > 2.5× sub-face size = new cluster
            clusters = _cluster_1d(midpoints, gap_thresh)
        else:
            # Single cluster for single-cut mode
            clusters = [(min(midpoints), max(midpoints))]
        
        for c_min, c_max in clusters:
            # Find the actual extent of sub-faces in this cluster
            c_lo = min(r[local_idx][0] for r in uniform_ranges
                       if (r[local_idx][0] + r[local_idx][1]) / 2 >= c_min - 1
                       and (r[local_idx][0] + r[local_idx][1]) / 2 <= c_max + 1)
            c_hi = max(r[local_idx][1] for r in uniform_ranges
                       if (r[local_idx][0] + r[local_idx][1]) / 2 >= c_min - 1
                       and (r[local_idx][0] + r[local_idx][1]) / 2 <= c_max + 1)
            
            # Low boundary cut
            if c_lo > f_min + margin:
                cut_val = _best_grid_snap(c_lo, f_min, 'below')
                if cut_val is not None and cut_val > f_min + 1 and cut_val < f_max - 1:
                    all_cuts.append(SplitPlane(axis=world_axis, value=cut_val))
            
            # High boundary cut
            if c_hi < f_max - margin:
                cut_val = _best_grid_snap(c_hi, f_max, 'above')
                if cut_val is not None and cut_val > f_min + 1 and cut_val < f_max - 1:
                    all_cuts.append(SplitPlane(axis=world_axis, value=cut_val))
    
    # Deduplicate cuts that are very close together (within 4 units)
    # Sort by axis then value for O(N log N) sorting + O(N) deduplication
    all_cuts.sort(key=lambda x: (x.axis, x.value))
    deduped: List[SplitPlane] = []
    for sp in all_cuts:
        if not deduped or sp.axis != deduped[-1].axis or abs(sp.value - deduped[-1].value) >= 4:
            deduped.append(sp)
    
    if not deduped:
        return []
    
    if allow_multi:
        # Cap to MAX_CUTS_PER_AXIS per axis to prevent brush count explosion.
        # 2 cuts per axis × 2 axes = up to 9 pieces per brush.
        MAX_CUTS_PER_AXIS = 2
        from collections import defaultdict
        per_axis: dict = defaultdict(list)
        for sp in deduped:
            per_axis[sp.axis].append(sp)
        capped: List[SplitPlane] = []
        for axis, cuts in per_axis.items():
            if len(cuts) <= MAX_CUTS_PER_AXIS:
                capped.extend(cuts)
            else:
                # Keep the cuts that separate the most dark sub-faces
                local_idx = 0 if axis == in_plane_axes[0] else 1
                def _score(sp):
                    above = sum(1 for r in uniform_ranges if r[local_idx][0] >= sp.value)
                    below = sum(1 for r in uniform_ranges if r[local_idx][1] <= sp.value)
                    return max(above, below)
                cuts.sort(key=_score, reverse=True)
                capped.extend(cuts[:MAX_CUTS_PER_AXIS])
        return capped
    else:
        # Single cut: pick the best single boundary
        best = None
        best_score = -1
        for sp in deduped:
            local_idx = 0 if sp.axis == in_plane_axes[0] else 1
            # Score by how many dark sub-faces are on one side
            above = sum(1 for r in uniform_ranges if r[local_idx][0] >= sp.value)
            below = sum(1 for r in uniform_ranges if r[local_idx][1] <= sp.value)
            score = max(above, below)
            if score > best_score:
                best_score = score
                best = sp
        return [best] if best else []


def _best_grid_snap(boundary: float, face_edge: float, direction: str,
                    ) -> Optional[float]:
    """Find the integer-snapped cut position just outside a cluster boundary.
    
    The cut is placed between the face edge and the cluster boundary,
    snapped to the nearest integer to avoid fractional coordinates.
    
    Args:
        boundary: World coordinate of the dark cluster edge
        face_edge: World coordinate of the brush face edge (opposite side)
        direction: 'below' = cut below cluster's low edge (snap toward face_edge),
                   'above' = cut above cluster's high edge (snap toward face_edge)
    
    Returns:
        Integer-snapped cut position, or None if no valid snap found.
    """
    # Minimum distance from face edge to avoid degenerate sliver brushes
    MIN_EDGE_DIST = 4.0
    
    if direction == 'below':
        # Snap to integer at or below the cluster's low edge
        snap = math.floor(boundary)
        # Must be strictly between face edge and boundary (with margin from edge)
        if face_edge + MIN_EDGE_DIST <= snap <= boundary:
            return float(snap)
    else:
        # Snap to integer at or above the cluster's high edge
        snap = math.ceil(boundary)
        if boundary <= snap <= face_edge - MIN_EDGE_DIST:
            return float(snap)
    
    return None


def _grid_snaps_in_range(lo: float, hi: float, 
                          face_lo: float, face_hi: float,
                          grid_sizes: Tuple[int, ...] = (64, 32, 16, 8, 4, 1)
                          ) -> List[float]:
    """Find grid-snapped values within a range, preferring larger grid sizes."""
    results = []
    for grid in grid_sizes:
        snap_lo = math.ceil(lo / grid) * grid
        snap_hi = math.floor(hi / grid) * grid
        val = snap_lo
        while val <= snap_hi:
            # Don't split at the face boundary itself
            if val > face_lo + 1 and val < face_hi - 1:
                if val not in results:
                    results.append(val)
            val += grid
    return results


# ─── VMF brush carving ───────────────────────────────────────────────────────

def _next_id(vmf_root: KVNode) -> int:
    """Find the next available ID in the VMF (max existing ID + 1)."""
    max_id = 0
    
    def scan(node):
        nonlocal max_id
        id_val = node.get_property('id')
        if id_val:
            try:
                max_id = max(max_id, int(id_val))
            except ValueError:
                pass
        for child in node.children:
            if isinstance(child, KVNode):
                scan(child)
    
    scan(vmf_root)
    return max_id + 1


def _make_plane_string(p1: Vec3, p2: Vec3, p3: Vec3) -> str:
    """Format three points as a VMF plane string: (x1 y1 z1) (x2 y2 z2) (x3 y3 z3)"""
    def fmt(v):
        parts = []
        for c in v:
            if c == int(c):
                parts.append(str(int(c)))
            else:
                parts.append(f"{c:.6g}")
        return ' '.join(parts)
    return f"({fmt(p1)}) ({fmt(p2)}) ({fmt(p3)})"


def _make_clip_side(axis: int, value: float, sign: int, next_id: int) -> KVNode:
    """Create a new VMF side node on an axis-aligned clip plane.
    
    Args:
        axis: 0=X, 1=Y, 2=Z
        value: World coordinate of the clip plane
        sign: +1 = faces positive direction, -1 = faces negative
        next_id: ID to assign to this side
    
    Returns:
        A KVNode representing the new side.
    """
    # Build three points on the plane
    # The plane normal points in the direction of 'sign' along 'axis'
    # We need three points whose cross product gives the correct normal
    
    v = value
    big = 16384  # Large enough to span any brush
    
    if axis == 0:  # X-axis plane
        if sign > 0:
            p1 = (v, -big, big)
            p2 = (v, big, big)
            p3 = (v, big, -big)
        else:
            p1 = (v, big, big)
            p2 = (v, -big, big)
            p3 = (v, -big, -big)
        uaxis_str = f'[0 1 0 0] 0.25'
        vaxis_str = f'[0 0 -1 0] 0.25'
    elif axis == 1:  # Y-axis plane
        if sign > 0:
            p1 = (big, v, big)
            p2 = (-big, v, big)
            p3 = (-big, v, -big)
        else:
            p1 = (-big, v, big)
            p2 = (big, v, big)
            p3 = (big, v, -big)
        uaxis_str = f'[1 0 0 0] 0.25'
        vaxis_str = f'[0 0 -1 0] 0.25'
    else:  # Z-axis plane
        if sign > 0:
            p1 = (-big, -big, v)
            p2 = (-big, big, v)
            p3 = (big, big, v)
        else:
            p1 = (-big, big, v)
            p2 = (-big, -big, v)
            p3 = (big, -big, v)
        uaxis_str = f'[1 0 0 0] 0.25'
        vaxis_str = f'[0 -1 0 0] 0.25'
    
    plane_str = _make_plane_string(p1, p2, p3)
    
    side_node = KVNode(name='side', children=[
        KVPair(key='id', value=str(next_id)),
        KVPair(key='plane', value=plane_str),
        KVPair(key='material', value='TOOLS/TOOLSNODRAW'),
        KVPair(key='uaxis', value=uaxis_str),
        KVPair(key='vaxis', value=vaxis_str),
        KVPair(key='rotation', value='0'),
        KVPair(key='lightmapscale', value='16'),
        KVPair(key='smoothing_groups', value='0'),
    ])
    
    return side_node


def _deep_copy_brush_node(brush_node: KVNode) -> KVNode:
    """Deep copy a brush KVNode, preserving all children."""
    return copy.deepcopy(brush_node)


def _strip_vertices_plus(brush_node: KVNode) -> None:
    """Remove all vertices_plus blocks from a brush's sides.
    
    After carving, vertices_plus data is invalid — VBSP recomputes
    vertex positions from the plane definitions.
    """
    for child in brush_node.children:
        if isinstance(child, KVNode) and child.name == 'side':
            child.children = [c for c in child.children
                             if not (isinstance(c, KVNode) and c.name == 'vertices_plus')]


def _reassign_ids(brush_node: KVNode, id_counter: List[int]) -> None:
    """Assign new unique IDs to a brush and all its sides."""
    brush_node.set_property('id', str(id_counter[0]))
    id_counter[0] += 1
    
    for child in brush_node.children:
        if isinstance(child, KVNode) and child.name == 'side':
            child.set_property('id', str(id_counter[0]))
            id_counter[0] += 1


def carve_brush(brush_node: KVNode, split_planes: List[SplitPlane],
                parent_node: KVNode, id_counter: List[int]) -> List[KVNode]:
    """Carve a brush along one or more axis-aligned split planes.
    
    For each piece, MOVES the existing face planes to the interval boundaries
    rather than adding clip planes. This avoids redundant parallel faces
    that Hammer can't resolve.
    
    For N1 cuts on axis A and N2 cuts on axis B, produces (N1+1)×(N2+1) pieces.
    
    Args:
        brush_node: The original brush KVNode to carve
        split_planes: List of SplitPlane defining where to cut
        parent_node: The parent node (worldspawn or entity) containing this brush
        id_counter: Mutable list[int] for generating unique IDs
    
    Returns:
        List of new brush KVNodes (the original is NOT modified — caller
        should replace it with these).
    """
    if not split_planes:
        return [_deep_copy_brush_node(brush_node)]
    
    # Group cuts by axis and sort values
    from collections import defaultdict
    cuts_by_axis: Dict[int, List[float]] = defaultdict(list)
    for sp in split_planes:
        cuts_by_axis[sp.axis].append(sp.value)
    
    # Sort and deduplicate per axis
    for axis in cuts_by_axis:
        cuts_by_axis[axis] = sorted(set(cuts_by_axis[axis]))
    
    # Identify which existing sides correspond to each axis direction
    # For an axis-aligned brush, there should be one side for each of the
    # 6 directions: +X, -X, +Y, -Y, +Z, -Z
    axis_sides = _identify_axis_sides(brush_node)
    
    # Build intervals per axis
    axis_intervals: List[List[Tuple[int, Optional[float], Optional[float]]]] = []
    
    for axis, values in sorted(cuts_by_axis.items()):
        intervals = []
        intervals.append((axis, None, values[0]))
        for i in range(len(values) - 1):
            intervals.append((axis, values[i], values[i + 1]))
        intervals.append((axis, values[-1], None))
        axis_intervals.append(intervals)
    
    # Cartesian product of intervals across axes
    import itertools
    piece_specs = list(itertools.product(*axis_intervals))
    
    pieces = []
    for spec in piece_specs:
        piece = _deep_copy_brush_node(brush_node)
        
        # For each interval, move the appropriate face planes
        for axis, lo, hi in spec:
            # In Source VMF, the cross-product normal from the plane points
            # is the INWARD normal (toward brush interior). So:
            #   axis_sides[(axis, +1)] = face at low position (normal points toward +axis interior)
            #   axis_sides[(axis, -1)] = face at high position (normal points toward -axis interior)
            lo_side_idx = axis_sides.get((axis, +1))  # Face at low position
            hi_side_idx = axis_sides.get((axis, -1))  # Face at high position
            
            if lo is not None and lo_side_idx is not None:
                # Move the low-position face to the new lo boundary
                _offset_side_plane(piece.children[lo_side_idx], axis, lo)
            
            if hi is not None and hi_side_idx is not None:
                # Move the high-position face to the new hi boundary
                _offset_side_plane(piece.children[hi_side_idx], axis, hi)
        
        _strip_vertices_plus(piece)
        _reassign_ids(piece, id_counter)
        pieces.append(piece)
    
    return pieces


def _identify_axis_sides(brush_node: KVNode) -> Dict[Tuple[int, int], int]:
    """Identify which child index corresponds to each axis direction.
    
    Returns a dict: (axis, sign) → child index
    Where axis is 0/1/2 for X/Y/Z and sign is +1 or -1.
    
    The sign represents the direction the face's outward normal points.
    For a box brush:
      - (0, -1) = left face (normal points -X, brush interior is to the right)
      - (0, +1) = right face (normal points +X, brush interior is to the left)
    """
    result = {}
    
    for idx, child in enumerate(brush_node.children):
        if not (isinstance(child, KVNode) and child.name == 'side'):
            continue
        
        plane_str = child.get_property('plane')
        if not plane_str:
            continue
        
        # Parse the three plane points
        pts = _parse_plane_points(plane_str)
        if not pts:
            continue
        
        # Compute face normal
        p1, p2, p3 = pts
        e1 = (p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2])
        e2 = (p3[0]-p1[0], p3[1]-p1[1], p3[2]-p1[2])
        nx = e1[1]*e2[2] - e1[2]*e2[1]
        ny = e1[2]*e2[0] - e1[0]*e2[2]
        nz = e1[0]*e2[1] - e1[1]*e2[0]
        
        # Find dominant axis and sign
        normal = (nx, ny, nz)
        abs_n = (abs(nx), abs(ny), abs(nz))
        dom_axis = abs_n.index(max(abs_n))
        dom_sign = 1 if normal[dom_axis] > 0 else -1
        
        result[(dom_axis, dom_sign)] = idx
    
    return result


def _parse_plane_points(plane_str: str) -> Optional[List[Tuple[float, float, float]]]:
    """Parse a VMF plane string '(x y z) (x y z) (x y z)' into three tuples."""
    import re
    matches = re.findall(r'\(([^)]+)\)', plane_str)
    if len(matches) != 3:
        return None
    
    pts = []
    for m in matches:
        parts = m.split()
        if len(parts) != 3:
            return None
        pts.append((float(parts[0]), float(parts[1]), float(parts[2])))
    return pts


def _offset_side_plane(side_node: KVNode, axis: int, new_value: float) -> None:
    """Move a side's plane so that all three points are shifted to new_value on the given axis.
    
    For an axis-aligned face, all three plane points share the same coordinate
    on one axis. This replaces that coordinate with new_value.
    """
    plane_str = side_node.get_property('plane')
    if not plane_str:
        return
    
    pts = _parse_plane_points(plane_str)
    if not pts:
        return
    
    # Shift all three points to new_value on the given axis
    new_pts = []
    for p in pts:
        p_list = list(p)
        p_list[axis] = new_value
        new_pts.append(tuple(p_list))
    
    new_plane_str = _make_plane_string(new_pts[0], new_pts[1], new_pts[2])
    side_node.set_property('plane', new_plane_str)


# ─── Main carving pipeline ───────────────────────────────────────────────────

def find_carve_candidates(
    lighting_data: Dict[int, object],
    brushes: list,
    side_map: Dict[int, KVNode],
    brush_map: Dict[int, KVNode],
    face_extents: list,
    allow_entities: bool = True,
    verbose: bool = False,
) -> List[CarveCandidate]:
    """Find faces that span both uniform and non-uniform BSP sub-faces.
    
    Only considers axis-aligned faces on axis-aligned brushes.
    Requires a meaningful luminance contrast between the uniform and varied
    groups — a truly dark region next to a lit region, not just coincidentally
    constant patches within an evenly-lit area.
    """
    from geometry import Plane
    from vertex_estimator import min_scale_no_subdivision, estimate_face_vertices, DEFAULT_MAX_LIGHTMAP_DIM
    
    # Minimum luminance contrast between uniform and varied group means
    # to consider the face a meaningful carve candidate.
    # A dark shadow (lum ~0) next to lit area (lum ~160) has contrast ~160.
    # Coincidentally-constant patches in lit area have contrast ~5.
    MIN_CONTRAST = 20.0
    
    # Minimum fraction of sub-faces that must be uniform for a viable carve
    MIN_UNIFORM_FRACTION = 0.01  # At least 1% uniform
    
    candidates = []
    
    # Build brush lookup: side_id → VMFBrush
    brush_by_side = {}
    for brush in brushes:
        for side in brush.sides:
            brush_by_side[side.id] = brush
    
    for side_id, ld in lighting_data.items():
        # Need at least 2 sub-faces to have a mixed face
        if len(ld.sub_faces) < 2:
            continue
        
        # Check uniformity mix
        uniform_subs = [sf for sf in ld.sub_faces if sf.is_uniform]
        varied_subs = [sf for sf in ld.sub_faces if not sf.is_uniform]
        
        if not uniform_subs or not varied_subs:
            continue  # Not mixed — all uniform or all varied
        
        # Check uniform fraction
        uniform_frac = len(uniform_subs) / len(ld.sub_faces)
        if uniform_frac < MIN_UNIFORM_FRACTION:
            continue
        
        # ─── Dark-uniform cluster detection ───────────────────────────────
        # The goal: find uniform sub-faces that are significantly DARKER than
        # the face's overall mean. These are shadow regions (under void boxes,
        # in occluded areas) that genuinely benefit from higher lightmapscale.
        # Bright uniform sub-faces (similar to the varied ones) are noise.
        
        # Compute face-wide mean luminance
        all_lums = []
        for sf in ld.sub_faces:
            if sf.luminances:
                all_lums.append(sum(sf.luminances) / len(sf.luminances))
        
        if not all_lums:
            continue
        
        face_mean = sum(all_lums) / len(all_lums)
        
        # Dark threshold: sub-faces with mean luminance below this are "dark"
        # Use half the face mean, clamped to a reasonable minimum
        dark_threshold = max(face_mean * 0.3, 5.0)
        
        # Filter uniform sub-faces into "dark uniform" (shadow regions)
        dark_uniform = [sf for sf in uniform_subs
                        if sf.luminances and 
                        sum(sf.luminances) / len(sf.luminances) < dark_threshold]
        
        if not dark_uniform:
            if verbose:
                uniform_mean = sum(sum(sf.luminances)/len(sf.luminances) 
                                   for sf in uniform_subs if sf.luminances) / max(len(uniform_subs), 1)
                print(f"    Skip side {side_id} (brush {brush_by_side.get(side_id, type('', (), {'id': '?'})()).id}): "
                      f"no dark uniform sub-faces "
                      f"(face_mean={face_mean:.1f}, dark_thresh={dark_threshold:.1f}, "
                      f"uniform_mean={uniform_mean:.1f})",
                      flush=True)
            continue
        
        # Replace uniform_subs with just the dark cluster for spatial analysis
        uniform_subs = dark_uniform
        
        # Look up the brush for this side
        brush = brush_by_side.get(side_id)
        if brush is None:
            continue
        
        # Check that the brush is axis-aligned
        if not _is_axis_aligned_brush(brush):
            continue
        
        # Find this specific side
        side = None
        for s in brush.sides:
            if s.id == side_id:
                side = s
                break
        if side is None:
            continue
        
        # Get the face normal axis
        if len(side.plane_points) < 3:
            continue
        p = Plane.from_three_points(*side.plane_points)
        axis_info = _is_axis_aligned_normal(p.normal)
        if axis_info is None:
            continue
        
        face_axis, face_sign = axis_info
        
        # Get brush bounding box for the face
        bbox = _get_brush_bbox(brush)
        if bbox is None:
            continue
        
        # Get parent node (worldspawn or entity containing this brush)
        brush_node = brush._node
        side_node = side._node
        
        if brush_node is None or side_node is None:
            continue
        
        if verbose:
            dark_mean = sum(sum(sf.luminances)/len(sf.luminances) 
                           for sf in dark_uniform if sf.luminances) / max(len(dark_uniform), 1)
            print(f"  Candidate: side {side_id} (brush {brush.id}): "
                  f"{len(dark_uniform)} dark-uniform sub-faces "
                  f"(dark_mean={dark_mean:.1f}, face_mean={face_mean:.1f}) + "
                  f"{len(varied_subs)} varied",
                  flush=True)
        
        candidates.append(CarveCandidate(
            side_id=side_id,
            brush_id=brush.id,
            brush_node=brush_node,
            side_node=side_node,
            parent_node=None,  # Will be set during apply
            face_normal_axis=face_axis,
            face_normal_sign=face_sign,
            uniform_regions=uniform_subs,
            varied_regions=varied_subs,
            face_bbox=bbox,
        ))
    
    if verbose and candidates:
        print(f"  Carver: found {len(candidates)} candidate faces for carving",
              flush=True)
    
    return candidates


def _get_piece_extent(piece: KVNode) -> Dict[int, Tuple[float, float]]:
    """Get the axis-aligned extent of a carved piece by parsing its plane data.
    
    Returns {axis: (min_val, max_val)} for each axis that has two
    opposing faces (i.e., the axis is fully bounded).
    """
    import re
    # Collect all plane point coordinates per axis
    axis_coords: Dict[int, List[float]] = {0: [], 1: [], 2: []}
    
    for child in piece.children:
        if not (isinstance(child, KVNode) and child.name == 'side'):
            continue
        plane_str = child.get_property('plane')
        if not plane_str:
            continue
        pts = _parse_plane_points(plane_str)
        if not pts:
            continue
        # Find which axis is constant (the face's normal axis)
        for ax in range(3):
            vals = set(p[ax] for p in pts)
            if len(vals) == 1:
                axis_coords[ax].append(list(vals)[0])
                break
    
    result = {}
    for ax, vals in axis_coords.items():
        if len(vals) >= 2:
            result[ax] = (min(vals), max(vals))
    return result


def _promote_uniform_pieces(
    new_brushes: List[KVNode],
    candidate: 'CarveCandidate',
    in_plane_axes: Tuple[int, int],
    splits: List[SplitPlane],
    verbose: bool = False,
) -> Set[int]:
    """Identify carved pieces that lie within dark-uniform regions.
    
    Simple approach: compute each piece's center on the in-plane axes, then
    check if that center falls within any dark sub-face's world-space range.
    This directly uses the BSP lightmap data that was already analyzed.
    
    Returns the set of side IDs (from ALL faces of promoted pieces) that
    should be treated as perfectly uniform by the solver.
    """
    promoted_ids: Set[int] = set()
    
    # Get world-space ranges of all dark-uniform sub-faces
    uniform_ranges = []
    for sf in candidate.uniform_regions:
        r = _subface_world_range(sf, in_plane_axes)
        if r:
            uniform_ranges.append(r)
    
    if not uniform_ranges:
        return promoted_ids
    
    a0, a1 = in_plane_axes
    promoted = 0
    
    for piece in new_brushes:
        extent = _get_piece_extent(piece)
        
        if a0 not in extent or a1 not in extent:
            continue
        
        # Compute piece center on each in-plane axis
        center_0 = (extent[a0][0] + extent[a0][1]) / 2
        center_1 = (extent[a1][0] + extent[a1][1]) / 2
        
        # Check if center overlaps any dark sub-face
        in_dark = any(
            r[0][0] <= center_0 <= r[0][1] and
            r[1][0] <= center_1 <= r[1][1]
            for r in uniform_ranges
        )
        
        if in_dark:
            # Collect all side IDs from this dark-uniform piece
            for child in piece.children:
                if isinstance(child, KVNode) and child.name == 'side':
                    sid_str = child.get_property('id')
                    if sid_str:
                        promoted_ids.add(int(sid_str))
            promoted += 1
    
    if verbose and promoted > 0:
        print(f"    Promoted {promoted} dark-uniform pieces ({len(promoted_ids)} side IDs)",
              flush=True)
    
    return promoted_ids


def apply_carves(
    vmf_root: KVNode,
    candidates: List[CarveCandidate],
    face_extents: list,
    allow_multi: bool = False,
    verbose: bool = False,
) -> Tuple[List[CarveResult], Set[int]]:
    """Apply brush carves to the VMF tree.
    
    For each candidate, determines split planes, carves the brush,
    and replaces it in the VMF tree.
    """
    from vertex_estimator import min_scale_no_subdivision, estimate_face_vertices, DEFAULT_MAX_LIGHTMAP_DIM
    
    results = []
    all_promoted_ids: Set[int] = set()
    id_counter = [_next_id(vmf_root)]
    
    # Build a set of brush node object IDs already processed
    # (a brush may contribute multiple candidate faces)
    processed_brushes: Set[int] = set()
    
    # Find worldspawn and entity nodes that contain brushes
    world_node = None
    for child in vmf_root.children:
        if isinstance(child, KVNode) and child.name == 'world':
            world_node = child
            break
    
    # Map brush node id() → parent node
    parent_map: Dict[int, KVNode] = {}
    if world_node:
        for child in world_node.children:
            if isinstance(child, KVNode) and child.name == 'solid':
                parent_map[id(child)] = world_node
    
    # Also scan entities for func_detail brushes
    for child in vmf_root.children:
        if isinstance(child, KVNode) and child.name == 'entity':
            classname = child.get_property('classname') or ''
            if classname.lower() == 'func_detail':
                for ent_child in child.children:
                    if isinstance(ent_child, KVNode) and ent_child.name == 'solid':
                        parent_map[id(ent_child)] = child
    
    # Accumulate old → new side ID mapping across all carves
    # For each old side ID, we map it to the list of new side IDs from all
    # carved pieces that inherited that face.
    side_id_remap: Dict[int, List[int]] = {}

    for candidate in candidates:
        brush_obj_id = id(candidate.brush_node)
        
        # Skip if this brush was already carved (multi-face on same brush)
        if brush_obj_id in processed_brushes:
            continue
        
        # Set parent_node
        parent = parent_map.get(brush_obj_id)
        if parent is None:
            if verbose:
                print(f"    Skipping brush {candidate.brush_id}: no parent found",
                      flush=True)
            continue
        candidate.parent_node = parent
        
        # Determine in-plane axes (the two axes NOT the face normal)
        in_plane = tuple(a for a in range(3) if a != candidate.face_normal_axis)
        
        # Find split planes
        splits = _find_split_planes(
            candidate.uniform_regions,
            candidate.varied_regions,
            in_plane,
            candidate.face_bbox,
            allow_multi=allow_multi,
        )
        
        if not splits:
            continue
        
        # Estimate vertex savings
        # The uniform piece's face can go to max_useful_scale
        # Look up the face extent for this side
        fe = None
        for f in face_extents:
            if f.side_id == candidate.side_id:
                fe = f
                break
        
        savings = 0
        if fe:
            # Current vertices at scale=1
            verts_at_1 = estimate_face_vertices(
                fe.extent_s, fe.extent_t, 1, fe.num_vertices, DEFAULT_MAX_LIGHTMAP_DIM)
            # Max useful scale
            max_scale = min_scale_no_subdivision(
                fe.extent_s, fe.extent_t, DEFAULT_MAX_LIGHTMAP_DIM)
            verts_at_max = estimate_face_vertices(
                fe.extent_s, fe.extent_t, max_scale, fe.num_vertices, DEFAULT_MAX_LIGHTMAP_DIM)
            
            # Rough savings: proportion of face that is uniform × vertex reduction
            total_subs = len(candidate.uniform_regions) + len(candidate.varied_regions)
            uniform_frac = len(candidate.uniform_regions) / total_subs if total_subs > 0 else 0
            savings = int((verts_at_1 - verts_at_max) * uniform_frac)
        
        if verbose:
            for sp in splits:
                print(f"  Chop: brush {candidate.brush_id}, side {candidate.side_id} "
                      f"({candidate.varied_regions[0].lightmap_vecs is not None and 'spatial' or 'no-spatial'}) "
                      f"along {AXIS_NAMES[sp.axis]}={sp.value:.0f}",
                      flush=True)
                print(f"    {len(candidate.uniform_regions)} uniform + "
                      f"{len(candidate.varied_regions)} varied sub-faces, "
                      f"est. savings: ~{savings} verts", flush=True)
        
        # Record original side IDs (by child index) BEFORE carving
        original_side_ids = []
        for child in candidate.brush_node.children:
            if isinstance(child, KVNode) and child.name == 'side':
                sid = child.get_property('id')
                if sid:
                    original_side_ids.append(int(sid))
                else:
                    original_side_ids.append(-1)
        
        # Perform the carve
        new_brushes = carve_brush(
            candidate.brush_node, splits, parent, id_counter)
        
        # Build old → new side ID mapping from all carved pieces
        # Each piece is a deep copy with the same child structure, so side
        # children are at the same indices. _reassign_ids has already run
        # inside carve_brush, so each piece now has unique new IDs.
        for piece in new_brushes:
            side_idx = 0
            for child in piece.children:
                if isinstance(child, KVNode) and child.name == 'side':
                    if side_idx < len(original_side_ids):
                        old_id = original_side_ids[side_idx]
                        if old_id >= 0:
                            if old_id not in side_id_remap:
                                side_id_remap[old_id] = []
                            new_id = child.get_property('id')
                            if new_id:
                                side_id_remap[old_id].append(int(new_id))
                    side_idx += 1
        
        # ─── Promote lightmapscale on dark-uniform pieces ────────────────
        # BSP face matching ran BEFORE carving, so carved pieces have no BSP
        # data for the solver to pre-promote them. We know which pieces are
        # dark-uniform from the carving analysis — promote them here.
        promoted_ids = _promote_uniform_pieces(
            new_brushes, candidate, in_plane, splits, verbose)
        all_promoted_ids.update(promoted_ids)
        
        # Replace original brush in parent
        new_children = []
        replaced = False
        for child in parent.children:
            if child is candidate.brush_node:
                new_children.extend(new_brushes)
                replaced = True
            else:
                new_children.append(child)
        
        if replaced:
            parent.children = new_children
            processed_brushes.add(brush_obj_id)
            
            results.append(CarveResult(
                original_brush_id=candidate.brush_id,
                original_side_id=candidate.side_id,
                split_planes=splits,
                new_brush_count=len(new_brushes),
                estimated_savings=savings,
                material=candidate.varied_regions[0].luminances is not None 
                         and str(len(candidate.uniform_regions)) or 'unknown',
            ))
        elif verbose:
            print(f"    WARNING: Could not find brush {candidate.brush_id} in parent",
                  flush=True)
    
    # ─── Update info_overlay side references ─────────────────────────────────
    # After all carves, remap overlay side IDs so they reference the new
    # carved pieces instead of the deleted originals.
    if side_id_remap:
        overlay_count = 0
        for child in vmf_root.children:
            if not (isinstance(child, KVNode) and child.name == 'entity'):
                continue
            classname = (child.get_property('classname') or '').lower()
            if classname not in ('info_overlay', 'info_overlay_transition'):
                continue
            
            sides_str = child.get_property('sides')
            if not sides_str:
                continue
            
            old_ids = sides_str.split()
            new_ids = []
            changed = False
            for sid_str in old_ids:
                try:
                    sid = int(sid_str)
                except ValueError:
                    new_ids.append(sid_str)
                    continue
                
                if sid in side_id_remap:
                    new_ids.extend(str(x) for x in side_id_remap[sid])
                    changed = True
                else:
                    new_ids.append(sid_str)
            
            if changed:
                child.set_property('sides', ' '.join(new_ids))
                overlay_count += 1
        
        if overlay_count > 0 and verbose:
            print(f"  Updated {overlay_count} overlay(s) with new side IDs "
                  f"({len(side_id_remap)} sides remapped)", flush=True)
    
    return results, all_promoted_ids

