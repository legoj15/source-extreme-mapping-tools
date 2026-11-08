"""
VBSP-in-the-Loop Solver — bottom-up strategy with ground-truth vertex counts.

Instead of starting at max quality and degrading, this solver:
  1. Finds the lowest global lightmapscale that fits ALL BSP budgets
     (vertices, leaffaces, faces — all capped at 65536)
  2. Promotes high-priority faces (shadow detail) back to scale=1

This produces much better results: the worst-case face is at the baseline
(e.g. scale=2), not scale 16 or 27. Faces with rich lighting detail get
promoted to scale=1 where they benefit most.

Promotion order: units go in descending value per estimated cost (promote_order
"value", the default): a unit's value is the sum of its sides' priority x area
x luxel gain, its cost the extra VBSP chop cells promotion adds. "priority" keeps the older
order, a unit ranked by its single best member and cost ignored.

Typical cost: ~3 VBSP calls for baseline + ~15 for the promotion prefix
search + up to skip_ahead_calls (default 40) for the skip-ahead pass;
d1_trainstation_02 takes 53 calls, about 2 minutes.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Set, Tuple

from vmf_parser import KVPair
from vertex_estimator import (
    FaceExtent,
    estimate_subdivisions,
    should_skip_face,
    min_scale_no_subdivision,
    min_scale_for_valid_extents,
    DEFAULT_MAX_LIGHTMAP_DIM,
)

# BSP format limits (all use unsigned short indexing)
BSP_LIMIT = 65536


def set_lightmapscale(side_node, value) -> None:
    """Set a side's lightmapscale, appending the key if the side lacks it.

    KVNode.set_property only updates EXISTING pairs and silently returns
    False otherwise, so a VMF whose sides omit the key (script-generated or
    third-party exports) would never receive a scale: the solver would count
    VBSP at the implicit 16 and write nothing back. This is the single
    scale-writing primitive; lmoptimizer._set_lightmapscale delegates here.
    """
    if not side_node.set_property('lightmapscale', str(value)):
        side_node.children.append(KVPair('lightmapscale', str(value)))


@dataclass
class VBSPSolverResult:
    """Result of the VBSP-in-the-loop solver."""
    scales: Dict[int, int]            # side_id → optimized lightmapscale
    vbsp_vertex_count: int            # Ground-truth vertex count from VBSP
    vbsp_leafface_count: int          # Ground-truth leafface count from VBSP
    vbsp_face_count: int              # Ground-truth face count from VBSP
    vertex_budget: int                # Applied vertex budget
    baseline_scale: int               # Lowest global scale that fits (no fit: the last one VBSP counted)
    faces_promoted: int               # Faces promoted from baseline to 1
    faces_at_baseline: int            # Faces left where the baseline puts them: at its scale, or held at or above it by a minimum scale or an equal evidence floor
    faces_skipped: int                # Tool/trigger faces excluded
    vbsp_calls: int                   # Number of VBSP invocations
    solve_time: float                 # Total solve time in seconds
    faces_clamped_by_detail: int = 0  # Faces held at min_scale due to %detailtype
    binding_limit: str = "verts"      # Which limit is the tightest constraint
    faces_pre_promoted: int = 0       # Faces pre-promoted to max_useful_scale (perfectly uniform)
    faces_evidence_raised: int = 0    # Faces an evidence floor holds above the baseline (evidence_scales)


@dataclass
class _FaceEntry:
    """Internal face tracking for the solver."""
    side_id: int
    extent: FaceExtent
    variance: float
    luminance_range: float
    has_detail: bool
    perceptual_priority: float = 0.0  # std_dev × mean luminance: detail × visibility
    min_scale: int = 1       # Minimum allowed scale (e.g. 5 for %detailtype)
    is_uniform: bool = False # True if perfectly uniform (pre-promoted)
    max_useful_scale: int = 1  # Scale beyond which no more vertex savings
    evidence_scale: int = 0  # Floor from the baseline lightmap (0 = none, never 1); see evidence_scales


@dataclass
class _CountResult:
    """Result from a VBSP count call."""
    verts: int
    leaffaces: int
    faces: int
    
    def fits_budget(self, budget: int) -> bool:
        """Check if ALL counts are within the budget."""
        return (self.verts <= budget and
                self.leaffaces <= budget and
                self.faces <= budget)
    
    def worst_count(self) -> int:
        """Return the highest count (the binding constraint)."""
        return max(self.verts, self.leaffaces, self.faces)
    
    def binding_name(self, budget: int) -> str:
        """Return which limit is binding (closest to or over budget)."""
        items = [("verts", self.verts), ("leaffaces", self.leaffaces),
                 ("faces", self.faces)]
        return max(items, key=lambda x: x[1])[0]


PROMOTE_ORDERS = ("value", "priority")

# Added to every side's estimated promotion cost, in chop cells. A promotion
# that adds no cell still adds luxels and lightmap data, and the constant keeps
# value/cost finite and breaks ties between units that cost the same cells.
_PROMOTE_BASE_COST = 0.25


def _chop_cells(fe: FaceExtent, scale: int, max_lm_dim: int) -> int:
    """Sub-faces VBSP chops the face into at `scale`. Verts and leaffaces both
    grow with this count, so it is the one cost unit. A displacement is not
    chopped (SubdivideFaceList never sees its faces): always 1."""
    if fe.has_displacement:
        return 1
    return (estimate_subdivisions(fe.extent_s, scale, max_lm_dim)
            * estimate_subdivisions(fe.extent_t, scale, max_lm_dim))


def _promotion_cost(e: "_FaceEntry", baseline_scale: int, max_lm_dim: int) -> float:
    """Estimated cost, in chop cells, of promoting one side from where the
    baseline leaves it to where Phase 2 puts it (the same two scales
    _promotion_scales uses)."""
    at_baseline = max(baseline_scale, e.min_scale)
    promoted = max(1, e.min_scale, e.evidence_scale)
    added = (_chop_cells(e.extent, promoted, max_lm_dim)
             - _chop_cells(e.extent, at_baseline, max_lm_dim))
    return max(0, added) + _PROMOTE_BASE_COST


def _promotion_value(e: "_FaceEntry", baseline_scale: int) -> float:
    """What promoting one side buys: its lighting detail times the luxels it
    gains. Luxel density goes as 1/scale^2, so the gain per world unit^2 is
    1/promoted^2 - 1/baseline^2: the same factor for every side promoted to 1
    at one baseline, but 0 for a side a minimum scale (%detailtype, texlight,
    displacement extent) holds at or above the baseline, and less for one an
    evidence floor stops short of 1. Those sides would otherwise rank first on
    area alone, as promotions that change nothing."""
    at_baseline = max(baseline_scale, e.min_scale)
    promoted = max(1, e.min_scale, e.evidence_scale)
    gain = max(0.0, 1.0 / (promoted * promoted) - 1.0 / (at_baseline * at_baseline))
    return e.perceptual_priority * e.extent.extent_s * e.extent.extent_t * gain


def solve_with_vbsp(
    face_extents: List[FaceExtent],
    lighting_data: Dict[int, object],
    vmf_root: object,
    side_map: Dict[int, object],
    vbsp_exe: Path,
    game_dir: Path,
    input_vmf: Path,
    bin_root: Path = None,
    vertex_budget: int = 65536,
    max_lm_dim: int = DEFAULT_MAX_LIGHTMAP_DIM,
    detail_type_materials: Optional[Set[str]] = None,
    detail_min_scale: int = 5,
    coplanar_groups: Optional[List[List[int]]] = None,
    emissive_sides: Optional[Set[int]] = None,
    uniform_max_luminance: float = 0.0,
    side_min_scales: Optional[Dict[int, int]] = None,
    evidence_scales: Optional[Dict[int, int]] = None,
    evidence_groups: Optional[List[List[int]]] = None,
    skip_ahead_calls: int = 40,
    promote_order: str = "value",
    verbose: bool = False,
) -> VBSPSolverResult:
    """Solve for optimal lightmapscales using real VBSP counts.
    
    Bottom-up strategy:
      Phase 1 — Find the lowest global scale that fits ALL budgets.
      Phase 2 — Promote high-priority faces from baseline to scale=1: the
                largest prefix of the promotion order that fits, then a
                skip-ahead pass over the units after it.

    promote_order: the order Phase 2 takes promotion units (coplanar groups,
    or single sides) in. "value" (default) ranks a unit by its sides' summed
    priority x area per estimated cost, the chop cells promotion adds
    (_promotion_cost); a group of mostly flat faces with one detailed member,
    or one that costs thousands of leaffaces, falls behind cheap detailed
    units. "priority" ranks by the unit's single best member and ignores cost.
    In value order, ties go to the higher best-member priority, then the lower
    side id; priority order keeps the stable sort of the best-member ranking.

    skip_ahead_calls: cap on the extra VBSP calls of that skip-ahead pass
    (0 = off, the prefix alone). One unit too large for the margin ends the
    prefix and strands what is left of the budget; the pass keeps the promotion
    order but steps over a unit that does not fit and adds the next runs of
    units that do, each run counted by VBSP, so the result is always a set
    VBSP counted under every budget.

    side_min_scales: optional side_id -> minimum scale (raise-only floors on
    top of the %detailtype and displacement-extent minimums), e.g. the scale
    the baseline light-budget ladder needed for texlight faces.

    evidence_scales: optional side_id -> the coarsest scale at which the whole
    side's baseline lightmap is still reproduced (lightmap_evidence). Lit
    smooth sides need it because the uniform test only accepts
    dark ones. Only sides that are not uniform or emissive take
    it. Over each coplanar group (evidence_groups, else coplanar_groups) the
    eligible members share their lowest RAW evidence; an eligible member
    with no evidence gives the group none. Each member then caps that at its
    own max useful scale (so a small member can stay finer than its siblings,
    at no vertex cost: it is unchopped at either scale), and a capped
    value of 1 is no floor. A floor that meets or beats max(baseline scale,
    min scale) keeps the side at max(baseline, min scale, floor) and out of
    Phase 2 promotion (finer is wasted); faces_evidence_raised counts only
    floors strictly above it, an equal floor counts as at baseline. A lower
    floor leaves the side promotable, to max(1, min scale, floor) instead
    of 1.

    evidence_groups: the groups for the evidence minimum only, when they
    differ from coplanar_groups (the promotion units); lmoptimizer's
    --coplanar strict passes no promotion groups, and its seam groups only
    with --smooth-tolerance; --coplanar off passes neither.

    VBSP failures: a VBSPRejectedError (VBSP's own capacity errors) at one
    trial is treated like 'does not fit' (the same tolerance as lmoptimizer's
    baseline scan); any other failure (missing exe, crash, timeout,
    unparsable output) propagates, as does a rejection if NO trial produced
    a count.
    """
    from vbsp_runner import count_vertices, ScratchVMF, VBSPRejectedError
    
    if promote_order not in PROMOTE_ORDERS:
        raise ValueError(f"promote_order must be one of {PROMOTE_ORDERS}, "
                         f"got {promote_order!r}")
    
    t_start = time.perf_counter()
    vbsp_calls = 0
    
    # ─── Build face list ──────────────────────────────────────────────────────
    entries: List[_FaceEntry] = []
    skipped = 0
    detail_set = detail_type_materials or set()
    
    for fe in face_extents:
        if should_skip_face(fe.material, fe.brush_entity):
            skipped += 1
            continue
        
        ld = lighting_data.get(fe.side_id)
        variance = ld.variance if ld else 0.0
        lum_range = ld.luminance_range if ld else 0.0
        has_detail = variance > 0.5 or lum_range > 2.0
        priority = ld.perceptual_priority if ld else 0.0
        
        # Check if face is perfectly uniform (zero variation)
        # But never pre-promote emissive (texlight) faces — their lightmap
        # resolution controls light emission quality.
        # Also, only pre-promote faces that are uniformly DARK. Fast VRAD
        # doesn't trace bounced/indirect light, so bright faces may appear
        # uniform when they actually have detail in a full compile.
        face_is_emissive = (emissive_sides and fe.side_id in emissive_sides)
        face_mean_lum = ld.mean_luminance if ld else 0.0
        face_is_dark_enough = (face_mean_lum <= uniform_max_luminance)
        face_uniform = ((ld.is_perfectly_uniform if ld else False)
                        and not face_is_emissive
                        and face_is_dark_enough)
        
        # Determine minimum scale for this face
        mat_lower = fe.material.lower().replace('\\', '/')
        face_min_scale = detail_min_scale if mat_lower in detail_set else 1
        
        # Enforce VBSP fatal extent limit (126 luxels max per dimension)
        # Only applies to displacement faces — brush faces are subdivided first
        if fe.has_displacement:
            face_min_scale = max(face_min_scale,
                                 min_scale_for_valid_extents(fe.extent_s, fe.extent_t))
        if side_min_scales:
            face_min_scale = max(face_min_scale,
                                 side_min_scales.get(fe.side_id, 1))
        
        # Compute max useful scale for this face
        face_max_useful = min_scale_no_subdivision(
            fe.extent_s, fe.extent_t, max_lm_dim)
        
        entries.append(_FaceEntry(
            side_id=fe.side_id,
            extent=fe,
            variance=variance,
            luminance_range=lum_range,
            has_detail=has_detail,
            perceptual_priority=priority,
            min_scale=face_min_scale,
            is_uniform=face_uniform,
            max_useful_scale=face_max_useful,
        ))

    
    if not entries:
        return VBSPSolverResult(
            scales={}, vbsp_vertex_count=0, vbsp_leafface_count=0,
            vbsp_face_count=0, vertex_budget=vertex_budget,
            baseline_scale=1, faces_promoted=0, faces_at_baseline=0,
            faces_skipped=skipped, vbsp_calls=0, solve_time=0.0,
        )
    
    # Count pre-promoted faces
    uniform_entries = [e for e in entries if e.is_uniform]
    pre_promoted = len(uniform_entries)
    
    # ─── Evidence floors (opt-in) ─────────────────────────────────────────────
    if evidence_scales:
        # Uniform sides are already at max useful scale and emissive
        # ones never take evidence, so neither has a say in a group's floor.
        eligible = {e.side_id: e for e in entries
                    if not (e.is_uniform
                            or (emissive_sides and e.side_id in emissive_sides))}
        raw = {sid: evidence_scales.get(sid, 0) for sid in eligible}
        floors = dict(raw)
        # Each group shares one floor: the eligible members take the
        # group's lowest RAW evidence (before the cap, so a small member
        # capped at 1 does not erase a big sibling's floor), and one with no
        # evidence at all (e.g. a carve-created side) zeroes it for all.
        # The per-member cap below can leave a small member finer than its
        # siblings (it is unchopped either way, so this costs no vertices).
        groups = evidence_groups if evidence_groups is not None else coplanar_groups
        for group in groups or ():
            members = [sid for sid in group if sid in raw]
            if members:
                low = min(raw[sid] for sid in members)
                for sid in members:
                    floors[sid] = min(floors[sid], low)
        for sid, e in eligible.items():
            capped = min(floors[sid], e.max_useful_scale)
            e.evidence_scale = capped if capped > 1 else 0  # 1 is no floor

    def _floored(base: int, or_equal: bool = False) -> List[_FaceEntry]:
        """Faces whose evidence floor beats (or_equal: matches or beats)
        where the baseline puts them."""
        if or_equal:
            return [e for e in entries
                    if e.evidence_scale and e.evidence_scale >= max(base, e.min_scale)]
        return [e for e in entries if e.evidence_scale > max(base, e.min_scale)]

    if verbose:
        print(f"  VBSP Solver: {len(entries)} eligible faces, {skipped} skipped",
              flush=True)
        n_evidence = sum(1 for e in entries if e.evidence_scale > 0)
        if n_evidence:
            print(f"  {n_evidence} faces have an evidence floor "
                  f"(baseline lighting reproduced at a coarser scale)",
                  flush=True)
        if len(uniform_entries) > 0:
            print(f"  Pre-promoting {len(uniform_entries)} perfectly-uniform faces to "
                  f"max_useful_scale", flush=True)
    
    # ─── Helper: apply scales to VMF and count everything ─────────────────────
    # The trial VMF is a uniquely named sibling of the input map so VBSP
    # resolves func_instance files exactly as it does for the real compile.
    scratch = ScratchVMF(input_vmf, 'solve')
    
    def _count_with_scales(scale_assignments: Dict[int, int]) -> _CountResult:
        """Apply scales to VMF, write temp file, call VBSP, return all counts."""
        nonlocal vbsp_calls
        
        for sid, scale in scale_assignments.items():
            node = side_map.get(sid)
            if node:
                set_lightmapscale(node, scale)
        
        scratch.write(vmf_root)
        # Count before running: a capacity rejection raises, and it still
        # cost a VBSP run (the skip-ahead cap counts attempts too).
        vbsp_calls += 1
        result = count_vertices(vbsp_exe, scratch.path, game_dir, bin_root=bin_root, verbose=False)
        
        return _CountResult(
            verts=result.count,
            leaffaces=result.leafface_count,
            faces=result.face_count,
        )
    
    # Capacity rejections seen at trial scales that were tolerated as 'too fine'
    tolerated_error: Optional[Exception] = None
    
    def _try_count(scale_assignments: Dict[int, int]) -> Optional[_CountResult]:
        """_count_with_scales, mapping a VBSP capacity rejection to None."""
        nonlocal tolerated_error
        try:
            return _count_with_scales(scale_assignments)
        except VBSPRejectedError as e:
            tolerated_error = e
            if verbose:
                print(f"    VBSP failed at this trial: {e}", flush=True)
            return None
    
    try:
        # ═══════════════════════════════════════════════════════════════════════
        #  PHASE 1: Find baseline scale (linear scan 1, 2, 3, ...)
        # ═══════════════════════════════════════════════════════════════════════
        baseline_scale = 0
        baseline_counts = _CountResult(0, 0, 0)
        counted_any = False
        last_counts = baseline_counts
        last_scales: Dict[int, int] = {}
        last_scale = 64
        
        for scale in range(1, 65):  # 1 through 64
            # Detail-type faces get max(scale, min_scale)
            # Uniform faces get max_useful_scale (pre-promoted)
            all_at_scale = {}
            for e in entries:
                if e.is_uniform:
                    all_at_scale[e.side_id] = max(e.max_useful_scale, e.min_scale)
                else:
                    all_at_scale[e.side_id] = max(scale, e.min_scale,
                                                  e.evidence_scale)
            
            if verbose:
                print(f"  [VBSP call {vbsp_calls + 1}] "
                      f"Testing global scale={scale}...", flush=True)
            
            counts = _try_count(all_at_scale)
            if counts is None:
                continue  # VBSP rejected this trial — try the next scale
            counted_any = True
            last_counts = counts
            last_scales = all_at_scale
            last_scale = scale
            
            if verbose:
                under = counts.fits_budget(vertex_budget)
                print(f"    Verts: {counts.verts:,}  "
                      f"Leaffaces: {counts.leaffaces:,}  "
                      f"Faces: {counts.faces:,}  "
                      f"({'under' if under else 'OVER'} budget)",
                      flush=True)
            
            if counts.fits_budget(vertex_budget):
                baseline_scale = scale
                baseline_counts = counts
                break
        
        if baseline_scale == 0 and not counted_any:
            # VBSP never produced a count at any scale: that is an
            # environment/map failure, not 'too many brushes'.
            raise tolerated_error
        
        if baseline_scale == 0:
            # Even scale=64 doesn't fit — map has too many brushes
            if verbose:
                print(f"  ✗ Map exceeds budget even at scale=64. "
                      f"Too many brushes.", flush=True)
            
            # Return the last assignment VBSP counted (a minimum scale or
            # evidence floor can hold a side above 64), and leave the tree on
            # it too: a later trial VBSP rejected may have written its own.
            for sid, scale in last_scales.items():
                node = side_map.get(sid)
                if node:
                    set_lightmapscale(node, scale)
            t_end = time.perf_counter()
            return VBSPSolverResult(
                scales=dict(last_scales),
                vbsp_vertex_count=last_counts.verts,
                vbsp_leafface_count=last_counts.leaffaces,
                vbsp_face_count=last_counts.faces,
                vertex_budget=vertex_budget,
                baseline_scale=last_scale,
                faces_promoted=0,
                faces_at_baseline=len(entries),
                faces_skipped=skipped,
                vbsp_calls=vbsp_calls,
                solve_time=t_end - t_start,
                binding_limit=last_counts.binding_name(vertex_budget),
            )
        
        if baseline_scale == 1:
            # Already under budget at scale=1 — apply pre-promotions for uniform faces
            if verbose:
                if pre_promoted > 0:
                    print(f"  ✓ All faces fit at scale=1! "
                          f"({pre_promoted} uniform faces pre-promoted)",
                          flush=True)
                else:
                    print(f"  ✓ All faces fit at scale=1! No degradation needed.",
                          flush=True)
            
            # Respect uniform faces' max_useful_scale
            early_scales = {}
            for e in entries:
                if e.is_uniform:
                    early_scales[e.side_id] = max(e.max_useful_scale, e.min_scale)
                else:
                    early_scales[e.side_id] = max(1, e.min_scale, e.evidence_scale)
            
            # Apply to VMF tree
            for sid, scale in early_scales.items():
                node = side_map.get(sid)
                if node:
                    set_lightmapscale(node, scale)
            
            n_floored = len(_floored(1))
            n_held = len(_floored(1, or_equal=True)) - n_floored  # equal floor: at baseline
            # A minimum scale above 1 (and no floor at or above it) keeps the
            # side where it is: at its baseline, not promoted to 1.
            n_min_held = sum(1 for e in entries
                             if not e.is_uniform
                             and e.min_scale > 1 and e.evidence_scale < e.min_scale)
            t_end = time.perf_counter()
            return VBSPSolverResult(
                scales=early_scales,
                vbsp_vertex_count=baseline_counts.verts,
                vbsp_leafface_count=baseline_counts.leaffaces,
                vbsp_face_count=baseline_counts.faces,
                vertex_budget=vertex_budget,
                baseline_scale=1,
                faces_promoted=len(entries) - pre_promoted - n_floored - n_held - n_min_held,
                faces_at_baseline=n_held + n_min_held,
                faces_skipped=skipped,
                vbsp_calls=vbsp_calls,
                solve_time=t_end - t_start,
                faces_pre_promoted=len(uniform_entries),
                faces_evidence_raised=n_floored,
                faces_clamped_by_detail=sum(1 for e in entries if e.min_scale > 1),
            )
        
        if verbose:
            binding = baseline_counts.binding_name(vertex_budget)
            headroom = vertex_budget - baseline_counts.worst_count()
            print(f"\n  Baseline: scale={baseline_scale} → "
                  f"verts={baseline_counts.verts:,}  "
                  f"leaffaces={baseline_counts.leaffaces:,}  "
                  f"faces={baseline_counts.faces:,}  "
                  f"(binding: {binding}, headroom: {headroom:,})\n", flush=True)
        
        # ═══════════════════════════════════════════════════════════════════════
        #  PHASE 2: Promote high-priority faces to scale=1
        # ═══════════════════════════════════════════════════════════════════════
        
        # Sort by priority: highest variance first → promoted first
        # Exclude pre-promoted uniform faces from promotion candidates,
        # and faces whose floor is at or above the baseline: the evidence
        # says the baseline (or coarser) already reproduces them, so a
        # promotion would spend budget for nothing. A floor below the
        # baseline only caps how far a promotion goes.
        floored_entries = _floored(baseline_scale, or_equal=True)
        n_raised = len(_floored(baseline_scale))
        n_held = len(floored_entries) - n_raised  # floor == baseline: at baseline
        floored_ids = {e.side_id for e in floored_entries}
        promotable = [e for e in entries if not e.is_uniform
                      and e.side_id not in floored_ids]
        promotable.sort(key=lambda e: e.perceptual_priority, reverse=True)
        
        # ─── Build promotion units from coplanar groups ────────────────────────
        # Faces sharing a coplanar group are promoted/demoted together.
        # Ungrouped faces are singleton units.
        if coplanar_groups:
            # Build side_id → group_index mapping
            sid_to_group = {}
            for gi, group in enumerate(coplanar_groups):
                for sid in group:
                    sid_to_group[sid] = gi
            
            # Build units: collect promotable entries per group
            group_entries: Dict[int, List[_FaceEntry]] = {}
            singleton_entries: List[_FaceEntry] = []
            for e in promotable:
                gi = sid_to_group.get(e.side_id)
                if gi is not None:
                    group_entries.setdefault(gi, []).append(e)
                else:
                    singleton_entries.append(e)
            
            # Each group's priority is the max priority among its members
            promotion_units: List[List[_FaceEntry]] = []
            for gi, elist in group_entries.items():
                promotion_units.append(elist)
            for e in singleton_entries:
                promotion_units.append([e])
            
            promotion_units.sort(
                key=lambda unit: max(e.perceptual_priority for e in unit),
                reverse=True)
        else:
            # No groups — each face is its own unit
            promotion_units = [[e] for e in promotable]
        
        # Value order: the units above are ranked by their best member, which
        # lets one detailed face carry a 64-face group, and cost never enters.
        # Rank by what the whole unit buys per cell it adds instead. The sort
        # is total (best-member priority, then lowest side id) so a run is
        # repeatable; "priority" keeps the order above untouched.
        unit_value_cost: Dict[int, Tuple[float, float]] = {}
        if promote_order == "value":
            ranked = []
            for unit in promotion_units:
                value = sum(_promotion_value(e, baseline_scale) for e in unit)
                cost = sum(_promotion_cost(e, baseline_scale, max_lm_dim)
                           for e in unit)
                ranked.append((unit, value, cost))
            ranked.sort(key=lambda r: (-(r[1] / r[2]),
                                       -max(e.perceptual_priority for e in r[0]),
                                       min(e.side_id for e in r[0])))
            promotion_units = [r[0] for r in ranked]
            unit_value_cost = {i: (r[1], r[2]) for i, r in enumerate(ranked)}
        
        # Flatten for total face count
        total_promotable_faces = sum(len(u) for u in promotion_units)

        # A side a minimum scale holds at or above the baseline rides along
        # with its unit but never changes scale: it is not a promoted face.
        def _moves(e):
            return max(1, e.min_scale, e.evidence_scale) < max(baseline_scale, e.min_scale)
        def _moved_faces(units):
            return sum(1 for u in units for e in u if _moves(e))
        
        if verbose:
            # Show top few candidates
            multi_unit_count = sum(1 for u in promotion_units if len(u) > 1)
            if promote_order == "value":
                print("  Promotion order: value per cost (summed priority x area x "
                      "luxel gain per estimated added chop cell)", flush=True)
                n_noop = sum(1 for v, _c in unit_value_cost.values() if v == 0)
                print(f"  ({n_noop} units gain nothing from promotion, a minimum "
                      f"scale or no lighting detail, and rank last)", flush=True)
                print(f"  Top promotion candidates (by value per cost):", flush=True)
            else:
                print("  Promotion order: priority (each unit's best member, "
                      "cost ignored)", flush=True)
                print(f"  Top promotion candidates (by perceptual priority):", flush=True)
            shown = 0
            for ui, unit in enumerate(promotion_units[:5]):
                for e in unit[:3]:
                    shown += 1
                    print(f"    #{shown}: side={e.side_id} priority={e.perceptual_priority:.1f} "
                          f"variance={e.variance:.2f} lum_range={e.luminance_range:.1f}"
                          f"{' [group of '+str(len(unit))+']' if len(unit)>1 else ''}",
                          flush=True)
                if ui in unit_value_cost:
                    value, cost = unit_value_cost[ui]
                    print(f"        unit {ui + 1}: value={value:,.0f} cost={cost:,.2f} "
                          f"cells value/cost={value / cost:,.0f}", flush=True)
            if len(promotion_units) > 5:
                print(f"    ... and {len(promotion_units) - 5} more units "
                      f"({total_promotable_faces - shown} faces)", flush=True)
            if pre_promoted > 0:
                print(f"  ({pre_promoted} uniform faces excluded from promotion)",
                      flush=True)
            if floored_entries:
                print(f"  ({len(floored_entries)} faces held at or above the baseline by "
                      f"an evidence floor, excluded from promotion)", flush=True)
            if multi_unit_count > 0:
                print(f"  ({multi_unit_count} coplanar groups treated as "
                      f"atomic promotion units)", flush=True)
            
            print(flush=True)
        
        def _promotion_scales(promoted) -> Dict[int, int]:
            """The scale assignment with the promotion units whose index is in
            `promoted` at scale=1 (or their floor) and the rest at baseline.
            The prefix search, the skip-ahead and the final result all build
            from here, so what VBSP counted is what gets returned."""
            scales = {}
            # Uniform faces always stay at max_useful_scale
            for e in uniform_entries:
                scales[e.side_id] = max(e.max_useful_scale, e.min_scale)
            for e in floored_entries:
                scales[e.side_id] = max(baseline_scale, e.min_scale, e.evidence_scale)
            for i, unit in enumerate(promotion_units):
                for entry in unit:
                    if i in promoted:
                        # Promote, but respect the min_scale and evidence floors
                        scales[entry.side_id] = max(1, entry.min_scale,
                                                    entry.evidence_scale)
                    else:
                        scales[entry.side_id] = max(baseline_scale, entry.min_scale)
            return scales
        
        # Binary search: find the largest N (units) such that promoting
        # units[0..N-1] to scale=1 (rest at baseline) stays under ALL budgets.
        lo, hi = 0, len(promotion_units)
        best_n = 0
        best_counts = baseline_counts
        
        if verbose:
            print(f"  Binary search: promoting units to scale=1 "
                  f"(0..{len(promotion_units)} units, "
                  f"~{max(1, len(promotion_units)).bit_length()} steps)", flush=True)
        
        while lo <= hi:
            mid = (lo + hi) // 2
            
            if mid == 0:
                # No promotions = baseline (already known to be under budget)
                lo = mid + 1
                continue
            
            # Promote the top-mid units to their best scale, rest at baseline.
            scales = _promotion_scales(range(mid))
            
            promoted_faces = _moved_faces(promotion_units[:mid])
            if verbose:
                print(f"  [VBSP call {vbsp_calls + 1}] "
                      f"Promote {mid}/{len(promotion_units)} units "
                      f"({promoted_faces} faces) to scale=1...",
                      flush=True)
            
            counts = _try_count(scales)
            
            if counts is not None and verbose:
                under = counts.fits_budget(vertex_budget)
                binding = counts.binding_name(vertex_budget)
                print(f"    Verts: {counts.verts:,}  "
                      f"Leaffaces: {counts.leaffaces:,}  "
                      f"({'under' if under else 'OVER'}, binding: {binding})",
                      flush=True)
            
            if counts is not None and counts.fits_budget(vertex_budget):
                # Under budget — try promoting more
                best_n = mid
                best_counts = counts
                lo = mid + 1
            else:
                # Over budget — promote fewer
                hi = mid - 1
        
        prefix_promoted_faces = _moved_faces(promotion_units[:best_n])
        if verbose:
            print(f"\n  Binary search result: {best_n} units promoted to scale=1 "
                  f"({prefix_promoted_faces} faces), "
                  f"{total_promotable_faces - prefix_promoted_faces} left at or above scale={baseline_scale}, "
                  f"{pre_promoted} pre-promoted (uniform) "
                  f"→ verts={best_counts.verts:,} "
                  f"leaffaces={best_counts.leaffaces:,}\n", flush=True)
        
        # ─── Skip-ahead: fill the margin the prefix search left ────────────────
        # The prefix stops at the first unit that overflows, however much
        # budget is left, and VBSP is the only judge of a fit. Unit best_n
        # (the stranding one) is already known not to fit on top of the
        # prefix. From the next unit on, add the longest run of consecutive
        # units that still fits (galloping 1, 2, 4... to find where it breaks,
        # then bisecting), step over the unit that breaks it and go on. The
        # accepted set only grows through a trial VBSP counted under budget,
        # so best_counts always belongs to it.
        n_units = len(promotion_units)
        accepted = set(range(best_n))
        skip_calls = 0
        n_stepped_over = 0
        
        def _skip_trial(first: int, length: int) -> Optional[_CountResult]:
            """VBSP counts for accepted + units[first:first+length]; those
            counts if every budget holds, else None (over, or VBSP rejected)."""
            nonlocal skip_calls
            run = range(first, first + length)
            if verbose:
                print(f"  [VBSP call {vbsp_calls + 1}] "
                      f"Skip-ahead: add units {first + 1}-{first + length} "
                      f"({length} units, "
                      f"{sum(len(promotion_units[i]) for i in run)} faces) "
                      f"to the {len(accepted)} promoted...", flush=True)
            skip_calls += 1
            counts = _try_count(_promotion_scales(accepted.union(run)))
            if counts is None:
                return None
            fits = counts.fits_budget(vertex_budget)
            if verbose:
                print(f"    Verts: {counts.verts:,}  "
                      f"Leaffaces: {counts.leaffaces:,}  "
                      f"({'under' if fits else 'OVER'}, "
                      f"binding: {counts.binding_name(vertex_budget)})",
                      flush=True)
            return counts if fits else None
        
        if best_n < n_units and skip_ahead_calls > 0:
            pos = best_n + 1
            n_stepped_over = 1  # the stranding unit
            while pos < n_units and skip_calls < skip_ahead_calls:
                # good: the longest run known to fit (0 = none yet);
                # bad: the shortest run known not to (None = none yet).
                good, good_counts, bad = 0, None, None
                step = 1
                while skip_calls < skip_ahead_calls:
                    length = min(step, n_units - pos)
                    counts = _skip_trial(pos, length)
                    if counts is None:
                        bad = length
                        break
                    good, good_counts = length, counts
                    if pos + length >= n_units:
                        break  # the run reached the end of the list
                    step *= 2
                while (bad is not None and bad - good > 1
                       and skip_calls < skip_ahead_calls):
                    mid = (good + bad) // 2
                    counts = _skip_trial(pos, mid)
                    if counts is None:
                        bad = mid
                    else:
                        good, good_counts = mid, counts
                if good:
                    accepted.update(range(pos, pos + good))
                    best_counts = good_counts
                pos += good
                if bad is not None and bad - good == 1:
                    # The unit right after the run does not fit on top of it:
                    # leave it at baseline and go on from the next one.
                    pos += 1
                    n_stepped_over += 1
        
        best_promoted_faces = _moved_faces(promotion_units[i] for i in accepted)
        if verbose and skip_calls:
            prefix_faces = _moved_faces(promotion_units[:best_n])
            print(f"  Skip-ahead: +{len(accepted) - best_n} units "
                  f"({best_promoted_faces - prefix_faces} faces), "
                  f"{n_stepped_over} units skipped, "
                  f"{skip_calls} VBSP calls → verts={best_counts.verts:,} "
                  f"leaffaces={best_counts.leaffaces:,}\n", flush=True)
        
        if verbose:
            # The same measures under either order, so two runs compare. The
            # raw sum counts a side a minimum scale holds in place at full area.
            def _raw(e):
                return e.perceptual_priority * e.extent.extent_s * e.extent.extent_t
            promoted_units = [promotion_units[i] for i in sorted(accepted)]
            promoted_raw = sum(_raw(e) for u in promoted_units for e in u)
            all_raw = sum(_raw(e) for u in promotion_units for e in u)
            promoted_value = sum(_promotion_value(e, baseline_scale)
                                 for u in promoted_units for e in u)
            all_value = sum(_promotion_value(e, baseline_scale)
                            for u in promotion_units for e in u)
            promoted_cost = sum(_promotion_cost(e, baseline_scale, max_lm_dim)
                                for u in promoted_units for e in u)
            print(f"  Promoted value ({promote_order} order, "
                  f"{best_promoted_faces} promoted sides): "
                  f"priority x area {promoted_raw:,.0f} of {all_raw:,.0f}; "
                  f"x luxel gain {promoted_value:,.0f} of {all_value:,.0f}; "
                  f"estimated cost {promoted_cost:,.0f} cells\n", flush=True)
        
        # ═══════════════════════════════════════════════════════════════════════
        #  Apply final scales and return
        # ═══════════════════════════════════════════════════════════════════════
        # Promoted units go to scale=1 (or their floor), rest at baseline
        final_scales = _promotion_scales(accepted)
        
        # Apply to VMF tree
        for sid, scale in final_scales.items():
            node = side_map.get(sid)
            if node:
                set_lightmapscale(node, scale)
        
        t_end = time.perf_counter()
        
        clamped_by_detail = sum(1 for e in entries if e.min_scale > 1)
        
        result = VBSPSolverResult(
            scales=final_scales,
            vbsp_vertex_count=best_counts.verts,
            vbsp_leafface_count=best_counts.leaffaces,
            vbsp_face_count=best_counts.faces,
            vertex_budget=vertex_budget,
            baseline_scale=baseline_scale,
            faces_promoted=best_promoted_faces,
            faces_at_baseline=total_promotable_faces - best_promoted_faces + n_held,
            faces_skipped=skipped,
            vbsp_calls=vbsp_calls,
            solve_time=t_end - t_start,
            faces_clamped_by_detail=clamped_by_detail,
            binding_limit=best_counts.binding_name(vertex_budget),
            faces_pre_promoted=len(uniform_entries),
            faces_evidence_raised=n_raised,
        )
        
        if verbose:
            _print_vbsp_solver_summary(result)
        
        return result
    
    finally:
        scratch.close()


def _print_vbsp_solver_summary(result: VBSPSolverResult) -> None:
    """Print a summary of VBSP solver results."""
    under = (result.vbsp_vertex_count <= result.vertex_budget and
             result.vbsp_leafface_count <= result.vertex_budget and
             result.vbsp_face_count <= result.vertex_budget)
    status = "UNDER BUDGET" if under else "OVER BUDGET"
    
    print(f"\n  VBSP Solver: {status} after {result.vbsp_calls} VBSP calls "
          f"({result.solve_time:.1f}s)", flush=True)
    print(f"  Vertices:  {result.vbsp_vertex_count:,}", flush=True)
    print(f"  Leaffaces: {result.vbsp_leafface_count:,}", flush=True)
    print(f"  Faces:     {result.vbsp_face_count:,}", flush=True)
    print(f"  Budget:    {result.vertex_budget:,} "
          f"(binding: {result.binding_limit})", flush=True)
    print(f"  Baseline scale: {result.baseline_scale}", flush=True)
    print(f"  Faces promoted to scale=1: {result.faces_promoted}", flush=True)
    print(f"  Faces left at baseline (scale={result.baseline_scale} or a minimum above it): "
          f"{result.faces_at_baseline}", flush=True)
    if result.faces_pre_promoted > 0:
        print(f"  Faces pre-promoted (uniform): {result.faces_pre_promoted}",
              flush=True)
    if result.faces_evidence_raised > 0:
        print(f"  Faces raised by evidence: {result.faces_evidence_raised}",
              flush=True)
