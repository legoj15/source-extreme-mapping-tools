"""
Budget Solver — constrained optimization of lightmapscale values.

Given face extents and lighting complexity scores (from BSP lightmap data),
finds optimal lightmapscale values that:
  1. Maximize the number of faces at lightmapscale=1
  2. Keep total estimated vertex count ≤ vertex budget (default 65536)
  3. Degrade the most uniform faces first, shadow/gradient faces last

Algorithm:
  - Start all faces at lightmapscale=1
  - Compute total estimated vertices
  - While over budget:
      - Pick the face with the LOWEST lighting complexity (most uniform)
      - Raise its lightmapscale by 1 (linear ladder: 1→2→3→4...)
      - Cap at the scale that eliminates subdivision (no further vertex savings)
      - Recompute total vertices
  - Faces with high lighting variance are the last to be touched
"""
from __future__ import annotations

import math
import heapq
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from vertex_estimator import (
    FaceExtent,
    estimate_face_vertices,
    estimate_map_vertices,
    min_scale_no_subdivision,
    min_scale_for_valid_extents,
    should_skip_face,
    DEFAULT_MAX_LIGHTMAP_DIM,
)

MAX_MAP_VERTS = 65536


@dataclass
class FaceBudgetEntry:
    """A face with its lighting complexity and scale state."""
    side_id: int
    extent: FaceExtent
    variance: float          # Lighting variance from BSP data (higher = more detail)
    luminance_range: float   # Max-min luminance
    max_luminance: float     # Peak luminance
    perceptual_priority: float  # std_dev × mean luminance: detail × visibility
    current_scale: int       # Current assigned lightmapscale
    max_useful_scale: int    # Scale beyond which no more vertex savings
    current_verts: int       # Estimated vertices at current scale
    min_scale: int = 1       # Minimum allowed scale (e.g. 5 for %detailtype faces)
    
    @property
    def is_maxed(self) -> bool:
        """True if further scale increases won't save vertices."""
        return self.current_scale >= self.max_useful_scale
    
    @property
    def has_lighting_detail(self) -> bool:
        """True if this face has shadows, gradients, or radiosity variation."""
        return self.variance > 0.5 or self.luminance_range > 2.0


@dataclass
class SolverResult:
    """Result of the budget solver."""
    scales: Dict[int, int]            # side_id → optimized lightmapscale
    initial_verts: int                # Estimated verts with VMF's current scales
    all_at_one_verts: int             # Estimated verts if everything is scale=1
    optimized_verts: int              # Estimated verts after optimization
    vertex_budget: int
    faces_at_scale_1: int             # How many faces kept at scale=1
    faces_degraded: int               # How many faces had scale raised
    faces_skipped: int                # How many faces were filtered out
    max_scale_assigned: int           # Highest scale assigned
    iterations: int                   # Number of solver iterations
    degraded_detail_faces: int        # Faces with lighting detail that HAD to be degraded
    faces_clamped_by_detail: int = 0  # Faces held at min_scale due to %detailtype
    faces_pre_promoted: int = 0       # Faces pre-promoted to max_useful_scale (perfectly uniform)
    faces_gradient_promoted: int = 0  # Faces pre-promoted due to monotonic gradient


def solve_vertex_budget(
    face_extents: List[FaceExtent],
    lighting_data: Dict[int, 'FaceLightmapData'],
    vertex_budget: int = MAX_MAP_VERTS,
    max_lm_dim: int = DEFAULT_MAX_LIGHTMAP_DIM,
    calibration_factor: float = 1.0,
    detail_type_materials: set = None,
    detail_min_scale: int = 5,
    gradient_tolerance: float = 0.5,
    emissive_sides: set = None,
    uniform_max_luminance: float = 0.0,
    verbose: bool = False,
) -> SolverResult:
    """Solve for optimal lightmapscales within vertex budget.
    
    Args:
        face_extents: Per-face world-space extents
        lighting_data: BSP lightmap data, keyed by VMF side ID
        vertex_budget: Maximum allowed vertices (default 65536)
        max_lm_dim: VBSP's g_maxLightmapDimension (default 32)
        calibration_factor: Ratio of BSP actual verts / raw estimate.
                            Used to scale raw estimates to match VBSP reality.
        verbose: Print per-step progress
        
    Returns:
        SolverResult with optimized scales and statistics
    """
    # Work in raw vertex space — inflate the budget by 1/factor
    # so we don't have to scale every individual vertex estimate
    raw_budget = int(vertex_budget / calibration_factor) if calibration_factor > 0 else vertex_budget
    
    # ─── Build entries for eligible faces ─────────────────────────────────────
    entries: List[FaceBudgetEntry] = []
    skipped = 0
    detail_set = detail_type_materials or set()
    
    for fe in face_extents:
        if should_skip_face(fe.material, fe.brush_entity):
            skipped += 1
            continue
        
        # Get lighting data if available
        ld = lighting_data.get(fe.side_id)
        variance = ld.variance if ld else 0.0
        lum_range = ld.luminance_range if ld else 0.0
        max_lum = ld.max_luminance if ld else 0.0
        priority = ld.perceptual_priority if ld else 0.0
        
        # Compute max useful scale (beyond this, no more subdivision savings)
        max_useful = min_scale_no_subdivision(fe.extent_s, fe.extent_t, max_lm_dim)
        
        # Determine minimum scale for this face
        mat_lower = fe.material.lower().replace('\\', '/')
        face_min_scale = detail_min_scale if mat_lower in detail_set else 1
        # Enforce VBSP fatal extent limit (126 luxels max per dimension)
        # Only applies to displacement faces — brush faces are subdivided first
        if fe.has_displacement:
            face_min_scale = max(face_min_scale,
                                 min_scale_for_valid_extents(fe.extent_s, fe.extent_t))
        start_scale = max(1, face_min_scale)
        
        # Compute vertices at starting scale
        verts_at_start = estimate_face_vertices(
            fe.extent_s, fe.extent_t, start_scale, fe.num_vertices, max_lm_dim)
        
        entries.append(FaceBudgetEntry(
            side_id=fe.side_id,
            extent=fe,
            variance=variance,
            luminance_range=lum_range,
            max_luminance=max_lum,
            perceptual_priority=priority,
            current_scale=start_scale,
            max_useful_scale=max_useful,
            current_verts=verts_at_start,
            min_scale=face_min_scale,
        ))
    
    if not entries:
        return SolverResult(
            scales={}, initial_verts=0, all_at_one_verts=0,
            optimized_verts=0, vertex_budget=vertex_budget,
            faces_at_scale_1=0, faces_degraded=0, faces_skipped=skipped,
            max_scale_assigned=1, iterations=0, degraded_detail_faces=0,
        )
    
    # ─── Compute initial vertex count (at VMF's current scales) ───────────────
    raw_initial = 0
    for fe in face_extents:
        if should_skip_face(fe.material, fe.brush_entity):
            continue
        v = estimate_face_vertices(
            fe.extent_s, fe.extent_t, fe.lightmapscale, fe.num_vertices, max_lm_dim)
        raw_initial += v
    
    # ─── Compute all-at-1 vertex count ────────────────────────────────────────
    raw_all_at_one = sum(e.current_verts for e in entries)
    
    # ─── Pre-promote perfectly-uniform faces to max_useful_scale ──────────────
    # Faces with zero lighting variation lose nothing from a higher scale, so we
    # push them to max_useful_scale upfront to free vertex budget for faces with
    # actual lighting detail.
    pre_promoted = 0
    pre_promoted_vert_savings = 0
    emissive = emissive_sides or set()
    for entry in entries:
        ld = lighting_data.get(entry.side_id)
        if (ld and ld.is_perfectly_uniform and not entry.is_maxed
                and entry.side_id not in emissive
                and ld.mean_luminance <= uniform_max_luminance):
            old_verts = entry.current_verts
            entry.current_scale = entry.max_useful_scale
            new_verts = estimate_face_vertices(
                entry.extent.extent_s, entry.extent.extent_t,
                entry.current_scale, entry.extent.num_vertices, max_lm_dim)
            entry.current_verts = new_verts
            savings = old_verts - new_verts
            pre_promoted_vert_savings += savings
            pre_promoted += 1
    
    # Recompute all-at-1 total after pre-promotion
    raw_all_at_one_after = sum(e.current_verts for e in entries)
    
    # ─── Pre-promote monotonic-gradient faces to max_useful_scale ──────────
    # Faces with smooth, monotonic lighting transitions lose very little
    # quality from a higher scale, so we push them up to free vertex budget.
    # Only enabled when --gradient-tolerance is explicitly provided.
    gradient_promoted = 0
    gradient_vert_savings = 0
    if gradient_tolerance is not None:
        for entry in entries:
            if entry.is_maxed:
                continue
            if entry.side_id in emissive:
                continue
            ld = lighting_data.get(entry.side_id)
            if (ld and ld.is_monotonic_gradient(gradient_tolerance)
                    and ld.mean_luminance <= uniform_max_luminance):
                old_verts = entry.current_verts
                entry.current_scale = entry.max_useful_scale
                new_verts = estimate_face_vertices(
                    entry.extent.extent_s, entry.extent.extent_t,
                    entry.current_scale, entry.extent.num_vertices, max_lm_dim)
                entry.current_verts = new_verts
                savings = old_verts - new_verts
                gradient_vert_savings += savings
                gradient_promoted += 1
    
    # Recompute total after gradient promotion
    if gradient_promoted > 0:
        raw_all_at_one_after = sum(e.current_verts for e in entries)
    
    # Calibrated values for display
    cal = calibration_factor
    cal_initial = int(raw_initial * cal)
    cal_all_at_one = int(raw_all_at_one * cal)
    
    if verbose:
        print(f"  Solver: {len(entries)} eligible faces, {skipped} skipped",
              flush=True)
        if pre_promoted > 0:
            cal_savings = int(pre_promoted_vert_savings * cal)
            print(f"  Pre-promoted {pre_promoted} perfectly-uniform faces "
                  f"(saved ~{cal_savings:,} verts)", flush=True)
        if gradient_promoted > 0:
            cal_grad_savings = int(gradient_vert_savings * cal)
            print(f"  Pre-promoted {gradient_promoted} monotonic-gradient faces "
                  f"(saved ~{cal_grad_savings:,} verts)", flush=True)
        if cal != 1.0:
            print(f"  Calibration factor: {cal:.4f} "
                  f"(raw budget: {raw_budget:,})", flush=True)
        print(f"  Vertex estimates: current VMF = {cal_initial:,}, "
              f"all-at-1 = {cal_all_at_one:,}, budget = {vertex_budget:,}",
              flush=True)
    
    # ─── If already under budget at scale=1 (with pre-promotions), we're done ─
    if raw_all_at_one_after <= raw_budget:
        scales = {e.side_id: e.current_scale for e in entries}
        cal_optimized = int(raw_all_at_one_after * cal)
        return SolverResult(
            scales=scales,
            initial_verts=cal_initial,
            all_at_one_verts=cal_all_at_one,
            optimized_verts=cal_optimized,
            vertex_budget=vertex_budget,
            faces_at_scale_1=sum(1 for e in entries if e.current_scale == 1),
            faces_degraded=sum(1 for e in entries if e.current_scale > 1),
            faces_skipped=skipped,
            max_scale_assigned=max(e.current_scale for e in entries),
            iterations=0,
            degraded_detail_faces=0,
            faces_pre_promoted=pre_promoted,
            faces_gradient_promoted=gradient_promoted,
        )
    
    # ─── Greedy solver: raise most uniform faces first ────────────────────────
    heap: List[Tuple[float, int, int]] = []
    entry_map: Dict[int, int] = {}
    
    for idx, entry in enumerate(entries):
        entry_map[entry.side_id] = idx
        if not entry.is_maxed:
            heapq.heappush(heap, (entry.perceptual_priority, entry.side_id, idx))
    
    current_raw_total = raw_all_at_one_after
    iterations = 0
    
    while current_raw_total > raw_budget and heap:
        variance, sid, idx = heapq.heappop(heap)
        entry = entries[idx]
        
        if entry.is_maxed:
            continue
        
        old_scale = entry.current_scale
        new_scale = old_scale + 1
        
        if new_scale > entry.max_useful_scale:
            new_scale = entry.max_useful_scale
        
        if new_scale == old_scale:
            continue
        
        old_verts = entry.current_verts
        new_verts = estimate_face_vertices(
            entry.extent.extent_s, entry.extent.extent_t,
            new_scale, entry.extent.num_vertices, max_lm_dim)
        
        savings = old_verts - new_verts
        
        entry.current_scale = new_scale
        entry.current_verts = new_verts
        current_raw_total -= savings
        iterations += 1
        
        if not entry.is_maxed:
            heapq.heappush(heap, (entry.perceptual_priority, entry.side_id, idx))
        
        if verbose and iterations % 500 == 0:
            cal_current = int(current_raw_total * cal)
            print(f"    iter {iterations}: ~{cal_current:,} verts, "
                  f"budget = {vertex_budget:,}", flush=True)
    
    # ─── Build result (calibrated values) ─────────────────────────────────────
    cal_optimized = int(current_raw_total * cal)
    
    scales = {e.side_id: e.current_scale for e in entries}
    faces_at_1 = sum(1 for e in entries if e.current_scale == 1)
    faces_degraded = sum(1 for e in entries if e.current_scale > 1)
    max_assigned = max(e.current_scale for e in entries)
    degraded_detail = sum(1 for e in entries 
                          if e.current_scale > 1 and e.has_lighting_detail)
    clamped_by_detail = sum(1 for e in entries if e.min_scale > 1)
    
    result = SolverResult(
        scales=scales,
        initial_verts=cal_initial,
        all_at_one_verts=cal_all_at_one,
        optimized_verts=cal_optimized,
        vertex_budget=vertex_budget,
        faces_at_scale_1=faces_at_1,
        faces_degraded=faces_degraded,
        faces_skipped=skipped,
        max_scale_assigned=max_assigned,
        iterations=iterations,
        degraded_detail_faces=degraded_detail,
        faces_clamped_by_detail=clamped_by_detail,
        faces_pre_promoted=pre_promoted,
        faces_gradient_promoted=gradient_promoted,
    )
    
    if verbose:
        _print_solver_summary(result)
    
    return result


def _print_solver_summary(result: SolverResult) -> None:
    """Print a summary of solver results."""
    under = result.optimized_verts <= result.vertex_budget
    status = "UNDER BUDGET" if under else "OVER BUDGET"
    
    print(f"\n  Solver: {status} after {result.iterations} iterations", flush=True)
    print(f"  Vertices: {result.initial_verts:,} (VMF current) → "
          f"{result.all_at_one_verts:,} (all scale=1) → "
          f"{result.optimized_verts:,} (optimized)", flush=True)
    print(f"  Budget: {result.vertex_budget:,} "
          f"({'OK' if under else f'OVER by {result.optimized_verts - result.vertex_budget:,}'})",
          flush=True)
    print(f"  Faces at scale=1: {result.faces_at_scale_1}", flush=True)
    print(f"  Faces degraded: {result.faces_degraded} "
          f"(max scale: {result.max_scale_assigned})", flush=True)
    if result.faces_pre_promoted > 0:
        print(f"  Pre-promoted (uniform): {result.faces_pre_promoted}", flush=True)
    if result.faces_gradient_promoted > 0:
        print(f"  Pre-promoted (gradient): {result.faces_gradient_promoted}", flush=True)
    if result.degraded_detail_faces > 0:
        print(f"  WARNING: {result.degraded_detail_faces} faces with "
              f"lighting detail had to be degraded", flush=True)
