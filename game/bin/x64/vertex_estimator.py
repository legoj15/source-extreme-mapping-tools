"""
Vertex Estimator — estimate VBSP vertex count for map faces.

Replicates VBSP's SubdivideFace logic to predict how many vertices each face
will produce at a given lightmapscale. Used by the budget solver to find
optimal scales that fit within MAX_MAP_VERTS (65536).

VBSP formula (from faces.cpp):
    luxel_extent = world_extent / lightmapscale
    if luxel_extent > g_maxLightmapDimension (32):
        subdivisions = ceil(luxel_extent / (g_maxLightmapDimension - 1))
    else:
        subdivisions = 1

Each face produces roughly (subdivisions_s * subdivisions_t * 4) vertices.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

Vec3 = Tuple[float, float, float]

# VBSP default (from faces.cpp line 62)
DEFAULT_MAX_LIGHTMAP_DIM = 32

# Safe lightmap span budget in luxels. VBSP's CalcFaceExtents calls Error()
# when ceil(maxs) - floor(mins) > MAX_DISP_LIGHTMAP_DIM_WITHOUT_BORDER + 1
# (= 126). The ceil/floor rounding can add one luxel over the true span, so
# the span itself must stay <= 125 luxels.
MAX_SAFE_SPAN_LUXELS = 125


@dataclass
class FaceExtent:
    """World-space extents of a face along its lightmap S/T axes."""
    side_id: int
    extent_s: float       # World-unit span along S axis
    extent_t: float       # World-unit span along T axis
    s_min: float          # Minimum projection on S axis
    s_max: float          # Maximum projection on S axis
    t_min: float          # Minimum projection on T axis
    t_max: float          # Maximum projection on T axis
    s_axis: Vec3          # S axis direction (normalized)
    t_axis: Vec3          # T axis direction (normalized)
    lightmapscale: int    # Current lightmapscale from VMF
    material: str
    num_vertices: int     # Number of vertices in the original polygon
    brush_entity: str     # "" for worldspawn, entity classname otherwise
    has_displacement: bool = False  # True if face has a dispinfo block


def parse_vmf_axis(axis_str: str) -> Tuple[Vec3, float]:
    """Parse a VMF uaxis/vaxis string into (direction, texture_scale).
    
    Format: "[nx ny nz offset] scale"
    Example: "[1 0 0 0] 0.25"
    
    Returns the normalized direction vector and the texture scale.
    The texture scale affects texel density but NOT lightmap density —
    lightmap density is controlled solely by lightmapscale.
    """
    # Match [nx ny nz offset] scale
    m = re.match(r'\[([^\]]+)\]\s+([\d.eE+-]+)', axis_str)
    if not m:
        return ((1.0, 0.0, 0.0), 1.0)
    
    parts = m.group(1).split()
    if len(parts) < 3:
        return ((1.0, 0.0, 0.0), 1.0)
    
    nx, ny, nz = float(parts[0]), float(parts[1]), float(parts[2])
    scale = float(m.group(2))
    
    # Normalize direction
    length = math.sqrt(nx*nx + ny*ny + nz*nz)
    if length < 1e-10:
        return ((1.0, 0.0, 0.0), scale)
    
    return ((nx/length, ny/length, nz/length), scale)


def compute_face_extent(vertices: List[Vec3],
                         uaxis_str: str,
                         vaxis_str: str,
                         lightmapscale: int,
                         side_id: int,
                         material: str,
                         brush_entity: str = "",
                         has_displacement: bool = False) -> FaceExtent:
    """Compute the world-space extent of a face along its lightmap axes.
    
    Projects all polygon vertices onto the S (uaxis) and T (vaxis) directions
    to find the min/max span in each axis.
    """
    s_dir, _ = parse_vmf_axis(uaxis_str)
    t_dir, _ = parse_vmf_axis(vaxis_str)
    
    # Project vertices onto S and T axes
    s_projs = [v[0]*s_dir[0] + v[1]*s_dir[1] + v[2]*s_dir[2] for v in vertices]
    t_projs = [v[0]*t_dir[0] + v[1]*t_dir[1] + v[2]*t_dir[2] for v in vertices]
    
    s_min, s_max = min(s_projs), max(s_projs)
    t_min, t_max = min(t_projs), max(t_projs)
    extent_s = s_max - s_min
    extent_t = t_max - t_min
    
    return FaceExtent(
        side_id=side_id,
        extent_s=extent_s,
        extent_t=extent_t,
        s_min=s_min,
        s_max=s_max,
        t_min=t_min,
        t_max=t_max,
        s_axis=s_dir,
        t_axis=t_dir,
        lightmapscale=lightmapscale,
        material=material,
        num_vertices=len(vertices),
        brush_entity=brush_entity,
        has_displacement=has_displacement,
    )


def estimate_subdivisions(extent: float, lightmapscale: int,
                            max_lm_dim: int = DEFAULT_MAX_LIGHTMAP_DIM) -> int:
    """Estimate VBSP subdivisions for one axis.
    
    Replicates SubdivideFace logic: if luxel_extent > g_maxLightmapDimension,
    split at (mins + maxDim - 1) / luxelsPerWorldUnit, recursively.
    """
    if extent < 1e-6 or lightmapscale < 1:
        return 1
    
    # luxelsPerWorldUnit = 1.0 / lightmapscale
    # luxel_extent = world_extent * luxelsPerWorldUnit = world_extent / lightmapscale
    luxel_extent = extent / lightmapscale
    
    if luxel_extent <= max_lm_dim:
        return 1
    
    # VBSP splits at (mins + maxDim - 1) in luxel space, then recurses
    # This is effectively: ceil(luxel_extent / (maxDim - 1))
    # But because it splits at (maxDim - 1) from the start each time:
    return math.ceil(luxel_extent / (max_lm_dim - 1))


def estimate_face_vertices(extent_s: float, extent_t: float,
                             lightmapscale: int, num_polygon_verts: int = 4,
                             max_lm_dim: int = DEFAULT_MAX_LIGHTMAP_DIM) -> int:
    """Estimate total vertices a face will produce after VBSP subdivision.
    
    VBSP subdivides a face into a grid of subs_s × subs_t sub-polygons.
    Vertices at grid boundaries are shared (deduplicated by VBSP's hash),
    so the unique vertex count is (subs_s + 1) * (subs_t + 1) for the grid.
    """
    subs_s = estimate_subdivisions(extent_s, lightmapscale, max_lm_dim)
    subs_t = estimate_subdivisions(extent_t, lightmapscale, max_lm_dim)
    
    if subs_s == 1 and subs_t == 1:
        # No subdivision — just the original polygon
        return num_polygon_verts
    
    # Grid of sub-faces: shared vertices at split boundaries
    # A subs_s × subs_t grid has (subs_s+1) × (subs_t+1) unique vertices
    return (subs_s + 1) * (subs_t + 1)


def min_scale_no_subdivision(extent_s: float, extent_t: float,
                               max_lm_dim: int = DEFAULT_MAX_LIGHTMAP_DIM) -> int:
    """Find the minimum integer lightmapscale that avoids subdivision entirely."""
    if extent_s < 1e-6 and extent_t < 1e-6:
        return 1
    
    # Need: extent / scale <= max_lm_dim
    # scale >= extent / max_lm_dim
    min_s = math.ceil(extent_s / max_lm_dim) if extent_s > 0 else 1
    min_t = math.ceil(extent_t / max_lm_dim) if extent_t > 0 else 1
    
    return max(1, max(min_s, min_t))


def min_scale_for_valid_extents(extent_s: float, extent_t: float) -> int:
    """Minimum lightmapscale that avoids VBSP's fatal 'Bad surface extents' error.

    VBSP's CalcFaceExtents (bsplib.cpp) calls Error() when
    ceil(maxs) - floor(mins) exceeds MAX_DISP_LIGHTMAP_DIM_WITHOUT_BORDER + 1
    = 126 luxels. The floor/ceil rounding can add one luxel over the true
    span, so the span must stay <= 125 luxels.
    """
    if extent_s < 1e-6 and extent_t < 1e-6:
        return 1
    
    min_s = math.ceil(extent_s / MAX_SAFE_SPAN_LUXELS) if extent_s > 0 else 1
    min_t = math.ceil(extent_t / MAX_SAFE_SPAN_LUXELS) if extent_t > 0 else 1
    
    return max(1, max(min_s, min_t))


def should_skip_face(material: str, entity_class: str) -> bool:
    """Determine if a face should be excluded from vertex estimation.
    
    Skips:
    - Tool materials (clip, trigger, skip, invisible, nodraw, etc.)
    - Non-geometry entities (triggers, areaportals, etc.)
    But KEEPS func_detail since they contribute to the vertex budget.
    """
    mat_upper = material.upper()
    
    # Skip all tool materials
    if mat_upper.startswith('TOOLS/'):
        return True
    
    # Skip non-geometry entities
    skip_entities = {
        'trigger_once', 'trigger_multiple', 'trigger_push',
        'trigger_hurt', 'trigger_look', 'trigger_proximity',
        'trigger_teleport', 'trigger_transition', 'trigger_gravity',
        'trigger_soundscape', 'trigger_vphysics_motion',
        'func_areaportal', 'func_areaportalwindow',
        'func_viscluster', 'func_occluder',
        'func_clip_vphysics', 'func_monitor',
    }
    if entity_class.lower() in skip_entities:
        return True
    
    return False


def estimate_map_vertices(face_extents: List[FaceExtent],
                           scales: Optional[Dict[int, int]] = None,
                           max_lm_dim: int = DEFAULT_MAX_LIGHTMAP_DIM,
                           calibration_factor: float = 1.0,
                           ) -> Tuple[int, Dict[int, int]]:
    """Estimate total map vertices from all face extents.
    
    Args:
        face_extents: List of FaceExtent for all eligible faces
        scales: Optional dict of side_id → lightmapscale overrides.
                If None, uses the VMF's current lightmapscale.
        max_lm_dim: VBSP's g_maxLightmapDimension
        calibration_factor: Multiply raw total by this to account for
                            VBSP's global vertex deduplication. Computed as
                            bsp_actual_verts / raw_estimate_at_compile_scales.
        
    Returns:
        (total_verts, per_face_verts) where per_face_verts maps
        side_id → estimated vertex count (raw, uncalibrated)
    """
    per_face = {}
    total = 0
    
    for fe in face_extents:
        if should_skip_face(fe.material, fe.brush_entity):
            continue
        
        scale = scales.get(fe.side_id, fe.lightmapscale) if scales else fe.lightmapscale
        verts = estimate_face_vertices(
            fe.extent_s, fe.extent_t, scale,
            fe.num_vertices, max_lm_dim)
        per_face[fe.side_id] = verts
        total += verts
    
    return int(total * calibration_factor), per_face
