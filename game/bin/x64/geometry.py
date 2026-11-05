"""
Geometry Builder — converts VMF brush definitions into polygon soup.

For each brush, we use the vertices_plus data (preferred) or clip infinite
windings by half-planes to produce face polygons. These polygons are used
for ray-brush intersection during the lighting simulation.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

try:
    from .vmf_parser import VMFBrush, VMFSide, KVNode
except ImportError:
    from vmf_parser import VMFBrush, VMFSide, KVNode

Vec3 = Tuple[float, float, float]

# ─── Vector math utilities ────────────────────────────────────────────────────

def vec_sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])

def vec_add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])

def vec_scale(v: Vec3, s: float) -> Vec3:
    return (v[0] * s, v[1] * s, v[2] * s)

def vec_dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

def vec_cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )

def vec_length(v: Vec3) -> float:
    return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)

def vec_normalize(v: Vec3) -> Vec3:
    l = vec_length(v)
    if l < 1e-10:
        return (0.0, 0.0, 0.0)
    return (v[0] / l, v[1] / l, v[2] / l)

def vec_lerp(a: Vec3, b: Vec3, t: float) -> Vec3:
    return (
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    )


# ─── Plane utilities ─────────────────────────────────────────────────────────

@dataclass
class Plane:
    """A plane defined by normal · point = dist."""
    normal: Vec3
    dist: float

    @staticmethod
    def from_three_points(p0: Vec3, p1: Vec3, p2: Vec3) -> Plane:
        """Create a plane from three points (VMF winding order)."""
        e1 = vec_sub(p1, p0)
        e2 = vec_sub(p2, p0)
        normal = vec_normalize(vec_cross(e1, e2))
        dist = vec_dot(normal, p0)
        return Plane(normal=normal, dist=dist)

    def distance_to(self, point: Vec3) -> float:
        """Signed distance from point to plane."""
        return vec_dot(self.normal, point) - self.dist


# ─── Face polygon data ────────────────────────────────────────────────────────

@dataclass
class AABB:
    """Axis-aligned bounding box for fast ray rejection."""
    mins: Vec3
    maxs: Vec3

    @staticmethod
    def from_points(points: List[Vec3]) -> AABB:
        if not points:
            return AABB((0, 0, 0), (0, 0, 0))
        min_x = min(p[0] for p in points)
        min_y = min(p[1] for p in points)
        min_z = min(p[2] for p in points)
        max_x = max(p[0] for p in points)
        max_y = max(p[1] for p in points)
        max_z = max(p[2] for p in points)
        return AABB((min_x, min_y, min_z), (max_x, max_y, max_z))

    def ray_intersects(self, origin: Vec3, inv_dir: Vec3,
                       max_dist: float) -> bool:
        """Slab-method ray-AABB test. inv_dir = 1/direction per component."""
        # Handle each axis
        t_min = 0.0
        t_max = max_dist
        for i in range(3):
            if abs(inv_dir[i]) > 1e30:  # ray parallel to slab
                if origin[i] < self.mins[i] or origin[i] > self.maxs[i]:
                    return False
            else:
                t1 = (self.mins[i] - origin[i]) * inv_dir[i]
                t2 = (self.maxs[i] - origin[i]) * inv_dir[i]
                if t1 > t2:
                    t1, t2 = t2, t1
                t_min = max(t_min, t1)
                t_max = min(t_max, t2)
                if t_min > t_max:
                    return False
        return True


@dataclass
class Face:
    """A face polygon with associated metadata."""
    vertices: List[Vec3]
    normal: Vec3
    plane: Plane
    area: float
    material: str
    lightmapscale: int
    side_id: int
    brush_id: int
    bbox: Optional[AABB] = None
    # Reference to the KVNode for in-place lightmapscale modification
    _side_node: Optional[KVNode] = field(default=None, repr=False)

    def centroid(self) -> Vec3:
        """Compute the centroid of the face polygon."""
        n = len(self.vertices)
        if n == 0:
            return (0.0, 0.0, 0.0)
        cx = sum(v[0] for v in self.vertices) / n
        cy = sum(v[1] for v in self.vertices) / n
        cz = sum(v[2] for v in self.vertices) / n
        return (cx, cy, cz)


def _polygon_area(vertices: List[Vec3], normal: Vec3) -> float:
    """Compute the area of a convex polygon using the Newell method."""
    if len(vertices) < 3:
        return 0.0
    # Sum cross products of edges from v0
    total = (0.0, 0.0, 0.0)
    v0 = vertices[0]
    for i in range(1, len(vertices) - 1):
        e1 = vec_sub(vertices[i], v0)
        e2 = vec_sub(vertices[i + 1], v0)
        c = vec_cross(e1, e2)
        total = vec_add(total, c)
    return abs(vec_dot(total, normal)) * 0.5


# ─── Winding clip (CSG) ──────────────────────────────────────────────────────

CLIP_EPSILON = 0.01


def clip_winding_by_plane(winding: List[Vec3], plane: Plane,
                          keep_front: bool = True) -> List[Vec3]:
    """Clip a convex polygon by a plane.
    
    If keep_front is True, keeps the side where distance_to > 0 (front).
    If keep_front is False, keeps the back side.
    """
    if not winding:
        return []

    dists = [plane.distance_to(v) for v in winding]
    n = len(winding)
    result: List[Vec3] = []

    for i in range(n):
        j = (i + 1) % n
        di = dists[i]
        dj = dists[j]

        # Determine which side each vertex is on
        if keep_front:
            vi_inside = di >= -CLIP_EPSILON
            vj_inside = dj >= -CLIP_EPSILON
        else:
            vi_inside = di <= CLIP_EPSILON
            vj_inside = dj <= CLIP_EPSILON

        if vi_inside:
            result.append(winding[i])

        # Check for edge crossing
        if (di > CLIP_EPSILON and dj < -CLIP_EPSILON) or \
           (di < -CLIP_EPSILON and dj > CLIP_EPSILON):
            # Compute intersection point
            t = di / (di - dj)
            t = max(0.0, min(1.0, t))
            intersection = vec_lerp(winding[i], winding[j], t)
            result.append(intersection)

    return result


def _make_base_winding(plane: Plane, size: float = 65536.0) -> List[Vec3]:
    """Create a large winding on the given plane.
    
    Generates a huge quad aligned to the plane for subsequent clipping.
    """
    n = plane.normal
    # Find the axis most perpendicular to the normal
    ax, ay, az = abs(n[0]), abs(n[1]), abs(n[2])
    if az >= ax and az >= ay:
        up = (1.0, 0.0, 0.0)
    elif ax >= ay:
        up = (0.0, 0.0, 1.0)
    else:
        up = (0.0, 0.0, 1.0)

    # Build tangent and bitangent
    tangent = vec_normalize(vec_cross(up, n))
    bitangent = vec_cross(n, tangent)

    # Center point on the plane
    center = vec_scale(n, plane.dist)

    # Create a large quad
    t_scaled = vec_scale(tangent, size)
    b_scaled = vec_scale(bitangent, size)

    # Counter-clockwise ring around the normal (tangent x bitangent = normal).
    # The order matters: an out-of-order quad is a self-intersecting bowtie,
    # and clipping a bowtie by the brush's other planes collapses faces
    # (typically z-normal floors/ceilings) into degenerate slivers.
    return [
        vec_sub(vec_add(center, t_scaled), b_scaled),   # (+t, -b)
        vec_add(vec_add(center, t_scaled), b_scaled),   # (+t, +b)
        vec_add(vec_sub(center, t_scaled), b_scaled),   # (-t, +b)
        vec_sub(vec_sub(center, t_scaled), b_scaled),   # (-t, -b)
    ]


def build_brush_faces(brush: VMFBrush) -> List[Face]:
    """Convert a VMF brush into a list of Face polygons.
    
    If vertices_plus data is available, use it directly.
    Otherwise, clip an infinite winding by all other planes.
    """
    faces: List[Face] = []

    # First, compute all planes
    planes: List[Plane] = []
    for side in brush.sides:
        if len(side.plane_points) >= 3:
            p = Plane.from_three_points(*side.plane_points)
            planes.append(p)
        else:
            planes.append(Plane(normal=(0, 0, 1), dist=0))

    for i, side in enumerate(brush.sides):
        # Prefer vertices_plus if available
        if side.vertices and len(side.vertices) >= 3:
            verts = list(side.vertices)
        else:
            # CSG: start with a large winding on this plane, clip by all others
            winding = _make_base_winding(planes[i])
            for j, other_plane in enumerate(planes):
                if i == j:
                    continue
                # Keep the front side (brush interior is where n·p > dist)
                winding = clip_winding_by_plane(winding, other_plane, keep_front=True)
                if len(winding) < 3:
                    break
            verts = winding

        if len(verts) < 3:
            continue

        normal = planes[i].normal
        area = _polygon_area(verts, normal)

        faces.append(Face(
            vertices=verts,
            normal=normal,
            plane=planes[i],
            area=area,
            material=side.material,
            lightmapscale=side.lightmapscale,
            side_id=side.id,
            brush_id=brush.id,
            bbox=AABB.from_points(verts),
            _side_node=side._node,
        ))

    return faces


def build_all_faces(brushes: List[VMFBrush]) -> List[Face]:
    """Build face polygons for all brushes."""
    all_faces: List[Face] = []
    for brush in brushes:
        all_faces.extend(build_brush_faces(brush))
    return all_faces


# ─── Ray-triangle intersection ────────────────────────────────────────────────

def ray_triangle_intersect(origin: Vec3, direction: Vec3,
                           v0: Vec3, v1: Vec3, v2: Vec3,
                           max_dist: float = 1e30) -> Optional[float]:
    """Möller–Trumbore ray-triangle intersection.
    
    Returns the distance t if hit, or None if no intersection.
    """
    e1 = vec_sub(v1, v0)
    e2 = vec_sub(v2, v0)
    h = vec_cross(direction, e2)
    a = vec_dot(e1, h)

    if -1e-8 < a < 1e-8:
        return None  # Parallel

    f = 1.0 / a
    s = vec_sub(origin, v0)
    u = f * vec_dot(s, h)
    if u < 0.0 or u > 1.0:
        return None

    q = vec_cross(s, e1)
    v = f * vec_dot(direction, q)
    if v < 0.0 or u + v > 1.0:
        return None

    t = f * vec_dot(e2, q)
    if t > 1e-6 and t < max_dist:
        return t
    return None


def ray_faces_intersect(origin: Vec3, direction: Vec3,
                        faces: List[Face],
                        max_dist: float = 1e30,
                        skip_face_id: int = -1) -> Optional[float]:
    """Test a ray against all face polygons (triangulated).
    
    Uses AABB pre-filtering to skip faces the ray can't possibly hit.
    Returns the nearest hit distance, or None if no hit.
    """
    nearest = max_dist
    hit = False

    # Precompute inverse direction for AABB slab test
    inv_dir = (
        1.0 / direction[0] if abs(direction[0]) > 1e-10 else 1e31,
        1.0 / direction[1] if abs(direction[1]) > 1e-10 else 1e31,
        1.0 / direction[2] if abs(direction[2]) > 1e-10 else 1e31,
    )

    for face in faces:
        if face.side_id == skip_face_id:
            continue
        # AABB early-rejection
        if face.bbox is not None and not face.bbox.ray_intersects(
                origin, inv_dir, nearest):
            continue
        # Triangulate the face polygon (fan from vertex 0)
        verts = face.vertices
        for i in range(1, len(verts) - 1):
            t = ray_triangle_intersect(
                origin, direction, verts[0], verts[i], verts[i + 1], nearest)
            if t is not None and t < nearest:
                nearest = t
                hit = True

    return nearest if hit else None
