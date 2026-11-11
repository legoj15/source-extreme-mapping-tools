"""
Brush Carver — split VMF brushes along uniformity boundaries for aggressive
lightmapscale optimization.

When one brush face spans both a flat-black shadow (every luxel identical, so
a coarse lightmap loses nothing) and lit, varied area, the whole face has to
keep the fine lightmapscale the lit part needs. Carving the brush around each
dark cluster lets the dark piece's face go to max scale on its own.

A carve is made only when at least one piece lies wholly in dark sub-faces;
only that piece's copy of the dark face is promoted. Every other side of the
carved brush keeps its original side's lighting data (see apply_carves'
remap), so the solver still sees the lit parts as lit.

Uniformity is judged per BSP sub-face (VBSP's lightmap-size chops), not per
luxel, so a shadow smaller than a sub-face is never found.

Info_overlay side lists are remapped onto the carved pieces.

Supported geometry: axis-aligned rectangular brushes only (normals ±X/±Y/±Z).
"""
from __future__ import annotations

import copy
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

from vmf_parser import KVNode, VMFBrush
from vertex_estimator import DEFAULT_DARK_THRESHOLD

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
    """Axis-aligned bounding box of an axis-aligned brush.

    Uses vertices_plus when present (Hammer++); stock Hammer does not write
    it, so otherwise the box comes from the axis-aligned side planes.
    """
    all_verts = []
    for side in brush.sides:
        if side.vertices:
            all_verts.extend(side.vertices)
    if not all_verts:
        if brush._node is None:
            return None
        ext = _get_piece_extent(brush._node)
        if len(ext) != 3:
            return None
        return ((ext[0][0], ext[1][0], ext[2][0]),
                (ext[0][1], ext[1][1], ext[2][1]))
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
        # A rotated texture tilts this axis across both in-plane world axes;
        # its luxel box is then a rotated box no axis range describes.
        if any(abs(vec[a]) > 1e-6 * abs(scale)
               for a in in_plane_axes if a != dominant):
            return None
        
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


def _luxel_world_size(sub_face, world_axis: int) -> Optional[float]:
    """World units per luxel along world_axis, from the lightmap axis on it."""
    if sub_face.lightmap_vecs is None:
        return None
    for lm_axis in range(2):
        vec = sub_face.lightmap_vecs[lm_axis][:3]
        dominant = max(range(3), key=lambda a: abs(vec[a]))
        if dominant == world_axis and abs(vec[dominant]) > 1e-8:
            return 1.0 / abs(vec[dominant])
    return None


def _find_split_planes(uniform_regions,
                       in_plane_axes: Tuple[int, int],
                       face_bbox: Tuple[Vec3, Vec3]) -> List[SplitPlane]:
    """Axis-aligned cuts that box in clusters of dark sub-faces.

    Per in-plane axis, dark sub-face ranges that touch or overlap form one
    cluster. Each cluster gets a cut just inside its low and high edge
    (a cluster reaching the face edge needs none there).

    Cuts sit one luxel INSIDE the cluster, snapped further inward to whole
    units: VBSP rounds every sub-face's luxel window outward
    (CalcFaceExtents floors the mins and ceils the maxs), so a recovered
    dark range overshoots its true edge by a and the lit neighbour's range
    reaches back across it by b, with a + b = 1 luxel at a shared edge.

    At most MAX_CUTS_PER_AXIS cuts per axis, so a brush becomes at most
    3 x 3 pieces. Clusters are taken whole, most dark sub-faces first, since
    a cluster with only one of its edges cut isolates nothing (a cut within
    4u of one already taken is shared, not added). Whether a resulting piece
    is really all dark is decided by _promote_uniform_pieces.
    """
    MAX_CUTS_PER_AXIS = 2

    dark = []  # (range, sub_face)
    for sf in uniform_regions:
        r = _subface_world_range(sf, in_plane_axes)
        if r:
            dark.append((r, sf))
    if not dark:
        return []

    result: List[SplitPlane] = []
    for local_idx, world_axis in enumerate(in_plane_axes):
        f_min = face_bbox[0][world_axis]
        f_max = face_bbox[1][world_axis]
        luxel = max((_luxel_world_size(sf, world_axis) or 0.0) for _r, sf in dark)
        if luxel <= 0:
            continue

        # Touching-interval clusters on this axis
        spans = sorted((r[local_idx][0], r[local_idx][1]) for r, _sf in dark)
        clusters = []  # [lo, hi, count]
        for lo, hi in spans:
            if clusters and lo <= clusters[-1][1] + 1e-3:  # touching, not near
                clusters[-1][1] = max(clusters[-1][1], hi)
                clusters[-1][2] += 1
            else:
                clusters.append([lo, hi, 1])

        candidates = []  # (count, [cut values])
        for c_lo, c_hi, count in clusters:
            cuts = []
            if c_lo > f_min + 0.5:
                cut_val = _best_grid_snap(c_lo + luxel, f_min, 'inward_lo')
                if cut_val is not None and f_min + 1 < cut_val < f_max - 1:
                    cuts.append(cut_val)
            if c_hi < f_max - 0.5:
                cut_val = _best_grid_snap(c_hi - luxel, f_max, 'inward_hi')
                if cut_val is not None and f_min + 1 < cut_val < f_max - 1:
                    cuts.append(cut_val)
            if len(cuts) == 2 and cuts[1] - cuts[0] < 4:
                continue  # nothing left between the cuts
            if cuts:
                candidates.append((count, cuts))

        chosen: List[float] = []
        for _count, cuts in sorted(candidates, key=lambda c: -c[0]):
            new = [v for v in cuts if all(abs(v - u) >= 4 for u in chosen)]
            if len(chosen) + len(new) <= MAX_CUTS_PER_AXIS:
                chosen.extend(new)
        result.extend(SplitPlane(axis=world_axis, value=v) for v in sorted(chosen))
    return result


def _best_grid_snap(boundary: float, face_edge: float, direction: str,
                    ) -> Optional[float]:
    """Snap a cut to a whole unit, toward the inside of the dark cluster.

    'inward_lo': the cut at a cluster's low edge (face_edge is the face's
    low edge), snapped up. 'inward_hi': the cut at a cluster's high edge
    (face_edge is the face's high edge), snapped down. Returns None when the
    cut would be within 4u of the face edge (a sliver brush).
    """
    MIN_EDGE_DIST = 4.0
    if direction == 'inward_lo':
        snap = math.ceil(boundary)
        if snap >= face_edge + MIN_EDGE_DIST:
            return float(snap)
    else:
        snap = math.floor(boundary)
        if snap <= face_edge - MIN_EDGE_DIST:
            return float(snap)
    return None


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
    
    The sign is that of (p2-p1) x (p3-p1), which for VMF winding is the
    INWARD normal. For a box brush:
      - (0, +1) = the face at the low X end (interior lies toward +X)
      - (0, -1) = the face at the high X end
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
    dark_floor: float = DEFAULT_DARK_THRESHOLD,
    verbose: bool = False,
    exclude_sides: Optional[Set[int]] = None,
) -> List[CarveCandidate]:
    """Find faces with both dark-uniform and varied BSP sub-faces.

    Only considers axis-aligned faces on axis-aligned brushes without
    displacements. "Dark" is a uniform sub-face whose mean is below
    max(0.3 x the face mean, dark_floor). dark_floor is --dark-threshold,
    in raw linear lightmap units.

    exclude_sides are never candidates: the texlight sides, which the
    solvers never promote, so carving one would only add brushes (#79).
    """
    from geometry import Plane

    # Minimum fraction of sub-faces that must be uniform for a viable carve
    MIN_UNIFORM_FRACTION = 0.01  # At least 1% uniform
    
    candidates = []
    
    # Build brush lookup: side_id → VMFBrush
    brush_by_side = {}
    for brush in brushes:
        for side in brush.sides:
            brush_by_side[side.id] = brush
    
    for side_id, ld in lighting_data.items():
        if exclude_sides and side_id in exclude_sides:
            continue
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
        
        # Sub-faces with mean luminance below this are "dark": 30% of the
        # face mean, never below the --dark-threshold floor.
        dark_threshold = max(face_mean * 0.3, dark_floor)
        
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
        
        # Spatial analysis uses the dark cluster; every other sub-face,
        # bright-uniform ones included, counts as "not dark" for promotion.
        uniform_subs = dark_uniform
        dark_ids = {id(sf) for sf in dark_uniform}
        varied_subs = [sf for sf in ld.sub_faces if id(sf) not in dark_ids]
        
        # Look up the brush for this side
        brush = brush_by_side.get(side_id)
        if brush is None:
            continue
        
        # Check that the brush is axis-aligned
        if not _is_axis_aligned_brush(brush):
            continue

        # Displacements: carving moves the side planes but each copy would
        # keep the whole dispinfo, which no longer fits its face.
        if any(s._node is not None and s._node.get_children_by_name('dispinfo')
               for s in brush.sides):
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
    face_child_idx: int,
    verbose: bool = False,
) -> Set[int]:
    """Side IDs of the carved copies of the candidate face that are all dark.

    A piece qualifies when its in-plane rectangle has its centre inside a
    dark-uniform sub-face and overlaps no other sub-face of the face at all
    (candidate.varied_regions holds every sub-face that is not dark-uniform).
    If any of those has no spatial data, nothing qualifies: its position is
    unknown. Only the piece's copy of the candidate face (child index
    face_child_idx) is returned; the piece's other sides are not on that
    face and say nothing about its lighting.
    """
    promoted_ids: Set[int] = set()

    dark_ranges = []
    for sf in candidate.uniform_regions:
        r = _subface_world_range(sf, in_plane_axes)
        if r:
            dark_ranges.append(r)
    lit_ranges = []
    for sf in candidate.varied_regions:
        r = _subface_world_range(sf, in_plane_axes)
        if r is None:
            return promoted_ids
        lit_ranges.append(r)
    if not dark_ranges:
        return promoted_ids

    a0, a1 = in_plane_axes
    # Touching is not overlap (EPS only absorbs float noise). Cuts sit a
    # luxel inside the dark cluster (see _find_split_planes), which absorbs
    # VBSP's outward luxel rounding.
    EPS = 1e-3

    for piece in new_brushes:
        extent = _get_piece_extent(piece)
        if a0 not in extent or a1 not in extent:
            continue
        (p0lo, p0hi), (p1lo, p1hi) = extent[a0], extent[a1]
        center_0 = (p0lo + p0hi) / 2
        center_1 = (p1lo + p1hi) / 2

        in_dark = any(r[0][0] <= center_0 <= r[0][1] and r[1][0] <= center_1 <= r[1][1]
                      for r in dark_ranges)
        touches_lit = any(r[0][0] < p0hi - EPS and r[0][1] > p0lo + EPS and
                          r[1][0] < p1hi - EPS and r[1][1] > p1lo + EPS
                          for r in lit_ranges)
        if not in_dark or touches_lit:
            continue

        child = piece.children[face_child_idx]
        sid_str = child.get_property('id') if isinstance(child, KVNode) else None
        if sid_str:
            promoted_ids.add(int(sid_str))

    if verbose and promoted_ids:
        print(f"    Promoted {len(promoted_ids)} dark piece face(s)", flush=True)
    return promoted_ids


def apply_carves(
    vmf_root: KVNode,
    candidates: List[CarveCandidate],
    face_extents: list,
    verbose: bool = False,
) -> Tuple[List[CarveResult], Set[int], Dict[int, List[int]]]:
    """Apply brush carves to the VMF tree.

    For each candidate, finds split planes, carves the brush, and replaces it
    in the VMF tree, but only when at least one piece's face is promoted;
    a carve that promotes nothing only adds brushes and edges.

    Returns (results, promoted side IDs, remap). remap maps each original
    side ID of a carved brush to the new side IDs that inherit it, promoted
    ones included; the caller copies lighting data along it so the lit
    pieces keep being treated as lit.
    """
    from vertex_estimator import min_scale_no_subdivision, estimate_face_vertices, DEFAULT_MAX_LIGHTMAP_DIM

    results = []
    all_promoted_ids: Set[int] = set()
    id_counter = [_next_id(vmf_root)]

    # A brush may contribute several candidate faces; carve it once.
    processed_brushes: Set[int] = set()

    world_node = None
    for child in vmf_root.children:
        if isinstance(child, KVNode) and child.name == 'world':
            world_node = child
            break

    # Map brush node id() -> parent node (worldspawn or func_detail)
    parent_map: Dict[int, KVNode] = {}
    if world_node:
        for child in world_node.children:
            if isinstance(child, KVNode) and child.name == 'solid':
                parent_map[id(child)] = world_node
    for child in vmf_root.children:
        if isinstance(child, KVNode) and child.name == 'entity':
            classname = child.get_property('classname') or ''
            if classname.lower() == 'func_detail':
                for ent_child in child.children:
                    if isinstance(ent_child, KVNode) and ent_child.name == 'solid':
                        parent_map[id(ent_child)] = child

    side_id_remap: Dict[int, List[int]] = {}
    extent_by_side = {f.side_id: f for f in face_extents}

    for candidate in candidates:
        brush_obj_id = id(candidate.brush_node)
        if brush_obj_id in processed_brushes:
            continue

        parent = parent_map.get(brush_obj_id)
        if parent is None:
            if verbose:
                print(f"    Skipping brush {candidate.brush_id}: no parent found",
                      flush=True)
            continue
        candidate.parent_node = parent

        in_plane = tuple(a for a in range(3) if a != candidate.face_normal_axis)
        splits = _find_split_planes(candidate.uniform_regions, in_plane,
                                    candidate.face_bbox)
        if not splits:
            continue

        # Original side IDs by side order, and the candidate face's child index
        original_side_ids = []
        face_child_idx = None
        for idx, child in enumerate(candidate.brush_node.children):
            if isinstance(child, KVNode) and child.name == 'side':
                sid = child.get_property('id')
                original_side_ids.append(int(sid) if sid else -1)
                if sid and int(sid) == candidate.side_id:
                    face_child_idx = idx
        if face_child_idx is None:
            continue

        saved_counter = id_counter[0]
        new_brushes = carve_brush(candidate.brush_node, splits, parent, id_counter)

        promoted_ids = _promote_uniform_pieces(
            new_brushes, candidate, in_plane, face_child_idx, verbose)
        if not promoted_ids:
            id_counter[0] = saved_counter
            if verbose:
                print(f"    Skip brush {candidate.brush_id}: no piece is wholly dark",
                      flush=True)
            continue

        # Rough savings: the dark share of the face, scale 1 -> max useful
        savings = 0
        fe = extent_by_side.get(candidate.side_id)
        if fe:
            verts_at_1 = estimate_face_vertices(
                fe.extent_s, fe.extent_t, 1, fe.num_vertices, DEFAULT_MAX_LIGHTMAP_DIM)
            max_scale = min_scale_no_subdivision(
                fe.extent_s, fe.extent_t, DEFAULT_MAX_LIGHTMAP_DIM)
            verts_at_max = estimate_face_vertices(
                fe.extent_s, fe.extent_t, max_scale, fe.num_vertices, DEFAULT_MAX_LIGHTMAP_DIM)
            total = len(candidate.uniform_regions) + len(candidate.varied_regions)
            savings = int((verts_at_1 - verts_at_max) * len(candidate.uniform_regions) / total)

        if verbose:
            cuts = ', '.join(f"{AXIS_NAMES[sp.axis]}={sp.value:.0f}" for sp in splits)
            print(f"  Carve: brush {candidate.brush_id}, side {candidate.side_id} "
                  f"at {cuts} -> {len(new_brushes)} pieces "
                  f"({len(candidate.uniform_regions)} dark + "
                  f"{len(candidate.varied_regions)} other sub-faces, "
                  f"est. savings ~{savings} verts)", flush=True)

        # Old -> new side ID mapping; pieces are deep copies, so side order matches
        for piece in new_brushes:
            side_idx = 0
            for child in piece.children:
                if isinstance(child, KVNode) and child.name == 'side':
                    if side_idx < len(original_side_ids) and original_side_ids[side_idx] >= 0:
                        new_id = child.get_property('id')
                        if new_id:
                            side_id_remap.setdefault(original_side_ids[side_idx], []).append(int(new_id))
                    side_idx += 1

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
            all_promoted_ids.update(promoted_ids)
            results.append(CarveResult(
                original_brush_id=candidate.brush_id,
                original_side_id=candidate.side_id,
                split_planes=splits,
                new_brush_count=len(new_brushes),
                estimated_savings=savings,
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
    
    return results, all_promoted_ids, side_id_remap

