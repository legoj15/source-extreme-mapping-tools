#!/usr/bin/env python3
"""
Visibility Oracle — determine which BSP faces are visible from reachable positions.

Given a BSP file and a reachability JSON, this module classifies every
renderable face as either VISIBLE or NEVER-VISIBLE by casting rays from
reachable eye positions to face sample points.

Usage:
    python visibility.py --bsp map.bsp --reachability reach.json [--output vis.json] [--verbose]

Output:
    JSON mapping face indices to visibility status and metadata.
"""
from __future__ import annotations

import argparse
import json
import math
import multiprocessing as mp
import os
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

_this_dir = os.path.dirname(os.path.abspath(__file__))
if _this_dir not in sys.path:
    sys.path.insert(0, _this_dir)

from bsp_reader import BSPReader, BSPFace, BSPPlane, CONTENTS_SOLID, _SURF_SKIP_VIS
from collision import CollisionWorld, MASK_PLAYERSOLID, MASK_VISIBLE
from reachability import ReachabilityMap

# Ensure UTF-8 output on Windows
if sys.platform == 'win32':
    try:
        if hasattr(sys.stdout, 'reconfigure'):
            sys.stdout.reconfigure(encoding='utf-8', errors='replace')
        if hasattr(sys.stderr, 'reconfigure'):
            sys.stderr.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

Vec3 = Tuple[float, float, float]

# Maximum distance for visibility rays (units)
MAX_VIS_DISTANCE = 16384.0

# Maximum number of eye positions to test per face  
MAX_EYES_PER_FACE = 2048

# Spatial-grid cell size for eye lookups (units).  Larger cells mean fewer
# cell probes per query; the squared-distance test in query_nearby keeps
# results exact regardless of cell size.
EYE_GRID_CELL_SIZE = 2048.0

# A visibility ray "reaches" its target if it stops within this many world
# units of it.  This is an ABSOLUTE clearance (distance-invariant), unlike a
# fractional threshold whose slop grows with ray length.  Must exceed the 1u
# sample offset plus the collision tracer's DIST_EPSILON.
HIT_CLEARANCE = 2.0

# ─── Face sample points ──────────────────────────────────────────────────────

# Probe distances (units along face normal) for detecting visibility
# through overhangs and around occluders.  These "stick out" past
# nearby geometry so a ground-level eye can see the space in front of
# the face even when the surface itself is ray-blocked.
_PROBE_OFFSETS = (64.0, 128.0)

def _face_sample_points(vertices: List[Vec3],
                        normal: Optional[Vec3] = None,
                        ) -> Tuple[List[Vec3], List[Vec3]]:
    """Generate sample points on a face polygon.
    
    Returns:
        (surface_samples, probe_samples)
        surface_samples — centroid + edge midpoints + inset vertices
        probe_samples   — centroid projected outward along *normal* at
                          several distances.  Empty if normal is None.
    """
    if len(vertices) < 3:
        return [], []
    
    surface = []
    
    # Centroid
    n = len(vertices)
    cx = sum(v[0] for v in vertices) / n
    cy = sum(v[1] for v in vertices) / n
    cz = sum(v[2] for v in vertices) / n
    surface.append((cx, cy, cz))
    
    # Edge midpoints
    for i in range(n):
        j = (i + 1) % n
        mx = (vertices[i][0] + vertices[j][0]) * 0.5
        my = (vertices[i][1] + vertices[j][1]) * 0.5
        mz = (vertices[i][2] + vertices[j][2]) * 0.5
        surface.append((mx, my, mz))
    
    # Vertex positions slightly inset
    for v in vertices:
        for inset in (0.02, 0.1):
            sx = v[0] + inset * (cx - v[0])
            sy = v[1] + inset * (cy - v[1])
            sz = v[2] + inset * (cz - v[2])
            surface.append((sx, sy, sz))
    
    # Probe samples — extended outward along normal
    probes: List[Vec3] = []
    if normal is not None:
        for dist in _PROBE_OFFSETS:
            probes.append((
                cx + normal[0] * dist,
                cy + normal[1] * dist,
                cz + normal[2] * dist,
            ))
    
    return surface, probes


def _vec_sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])

def _vec_dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

def _vec_length(v: Vec3) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])

def _vec_normalize(v: Vec3) -> Vec3:
    l = _vec_length(v)
    if l < 1e-10:
        return (0.0, 0.0, 0.0)
    return (v[0] / l, v[1] / l, v[2] / l)


# ─── Spatial grid for eye position lookup ─────────────────────────────────────

class SpatialGrid:
    """Simple spatial hash for fast nearby-eye-position queries."""
    
    def __init__(self, positions: List[Vec3], cell_size: float = EYE_GRID_CELL_SIZE):
        self.cell_size = cell_size
        self.positions = positions
        self.cells: Dict[Tuple[int,int,int], List[int]] = {}

        for idx, pos in enumerate(positions):
            cell = self._hash(pos)
            if cell not in self.cells:
                self.cells[cell] = []
            self.cells[cell].append(idx)
    
    def _hash(self, pos: Vec3) -> Tuple[int, int, int]:
        return (
            int(math.floor(pos[0] / self.cell_size)),
            int(math.floor(pos[1] / self.cell_size)),
            int(math.floor(pos[2] / self.cell_size)),
        )
    
    def query_nearby(self, pos: Vec3, radius: float) -> List[int]:
        """Return indices of positions within *radius* of pos.

        The +1 cell margin guarantees no in-radius position is missed
        regardless of where pos sits inside its cell, and the squared-
        distance test trims the scanned cell-cube down to an actual
        sphere (dropping anything beyond radius), so the result is exact
        rather than merely cell-block-bounded.
        """
        r_cells = int(math.ceil(radius / self.cell_size)) + 1
        cx, cy, cz = self._hash(pos)
        positions = self.positions
        px, py, pz = pos
        r_sq = radius * radius

        result = []
        for dx in range(-r_cells, r_cells + 1):
            for dy in range(-r_cells, r_cells + 1):
                for dz in range(-r_cells, r_cells + 1):
                    bucket = self.cells.get((cx + dx, cy + dy, cz + dz))
                    if not bucket:
                        continue
                    for idx in bucket:
                        ex, ey, ez = positions[idx]
                        ddx = ex - px
                        ddy = ey - py
                        ddz = ez - pz
                        if ddx*ddx + ddy*ddy + ddz*ddz <= r_sq:
                            result.append(idx)
        return result


# ─── Multiprocessing worker ──────────────────────────────────────────────────

# Module-level globals for worker processes
_worker_world: Optional[CollisionWorld] = None
_worker_eye_positions: Optional[List[Vec3]] = None
_worker_eye_clusters: Optional[List[int]] = None
_worker_eye_grid: Optional[SpatialGrid] = None


def _worker_init(bsp_path: str, eye_positions: List[Vec3], vpk_paths=None):
    """Initialize a CollisionWorld in each worker process.
    
    Called once per worker process by Pool's initializer.
    Each worker gets its own independent collision world.
    """
    global _worker_world, _worker_eye_positions, _worker_eye_clusters, _worker_eye_grid
    
    bsp = BSPReader(bsp_path)
    bsp.read()
    
    vpk_readers = []
    if vpk_paths:
        from vpk_reader import VPKReader
        from pathlib import Path
        for vp in vpk_paths:
            try:
                r = VPKReader(Path(vp))
                vpk_readers.append(r)
            except Exception as e:
                print(f"VPK Init Worker Error: {e}", flush=True)

    _worker_world = CollisionWorld(bsp, vpk_readers=vpk_readers, verbose=False)
    _worker_eye_positions = eye_positions
    
    # Pre-compute eye clusters
    _worker_eye_clusters = []
    for pos in eye_positions:
        _worker_eye_clusters.append(_worker_world.leaf_cluster(pos))
    
    # Build spatial grid
    _worker_eye_grid = SpatialGrid(eye_positions)


def _worker_check_face(work_item):
    """Worker function: check visibility of a single face.
    
    Args:
        work_item: (face_idx, offset_samples, n_surface, normal)
            n_surface — number of leading surface samples.
            Remaining samples are probes (require centroid confirmation).
            normal — side-corrected outward face normal (backface cull).

    Returns:
        (face_idx, is_visible, min_distance)
    """
    face_idx, offset_samples, n_surface, normal = work_item
    
    world = _worker_world
    eye_positions = _worker_eye_positions
    eye_clusters = _worker_eye_clusters
    eye_grid = _worker_eye_grid
    
    min_dist = MAX_VIS_DISTANCE
    
    if not offset_samples:
        return (face_idx, False, -1.0)
    
    # Get face cluster for PVS filtering
    face_cluster = world.leaf_cluster(offset_samples[0])
    
    # Find candidate eye positions within range
    centroid = offset_samples[0]
    nearby_indices = eye_grid.query_nearby(centroid, MAX_VIS_DISTANCE)
    
    if not nearby_indices:
        return (face_idx, False, -1.0)
    
    # PVS pre-filter: rigidly restrict to VBSP portal visibility clusters
    if face_cluster >= 0:
        pvs_ok = []
        for idx in nearby_indices:
            ec = eye_clusters[idx]
            if ec < 0 or world.is_cluster_visible(ec, face_cluster):
                pvs_ok.append(idx)
        nearby_indices = pvs_ok
    
    if not nearby_indices:
        return (face_idx, False, -1.0)
    
    # Stratified eye selection: pick from multiple distance bands
    # to ensure angular diversity (avoids all eyes from one direction).
    if len(nearby_indices) > MAX_EYES_PER_FACE:
        dists = []
        for idx in nearby_indices:
            eye = eye_positions[idx]
            d = _vec_sub(eye, centroid)
            dist = _vec_length(d)
            dists.append((dist, idx))
        dists.sort()
        
        n_close = MAX_EYES_PER_FACE // 2
        n_mid = MAX_EYES_PER_FACE // 4
        n_far = MAX_EYES_PER_FACE - n_close - n_mid
        
        total = len(dists)
        mid_start = total // 3
        far_start = 2 * total // 3
        
        # 1. Take the absolute closest eyes without skipping any (vital for tight angles)
        selected = [idx for _, idx in dists[:n_close]]
        
        # 2. Sample from mid-range
        mid_pool = dists[mid_start:far_start]
        if mid_pool:
            step = max(1, len(mid_pool) // n_mid)
            selected += [idx for _, idx in mid_pool[::step]][:n_mid]
            
        # 3. Sample from far-range
        far_pool = dists[far_start:]
        if far_pool:
            step = max(1, len(far_pool) // n_far)
            selected += [idx for _, idx in far_pool[::step]][:n_far]
            
        nearby_indices = selected
    
    # Surface centroid for probe confirmation rays
    surface_centroid = offset_samples[0] if offset_samples else None
    
    for eye_idx in nearby_indices:
        eye = eye_positions[eye_idx]
        
        # Backface culling: Eye must be in front of the face
        to_eye_global = _vec_sub(eye, centroid)
        if _vec_dot(normal, to_eye_global) <= 0.0:
            continue
            
        for si, sample in enumerate(offset_samples):
            to_eye = _vec_sub(eye, sample)
            
            # Backface culling early out
            if normal is not None:
                if to_eye[0]*normal[0] + to_eye[1]*normal[1] + to_eye[2]*normal[2] < 0:
                    continue
                    
            dist = _vec_length(to_eye)
            if dist < 1.0 or dist > MAX_VIS_DISTANCE:
                continue
            
            hit = world.trace_ray_full(eye, sample, MASK_VISIBLE)

            if (1.0 - hit.fraction) * dist <= HIT_CLEARANCE:
                # Probe hit: confirm the face centroid is reachable.
                # Probes project 64-128u from the surface and can peek
                # through windows that the face itself can't be seen from.
                if si >= n_surface and surface_centroid is not None:
                    confirm = world.trace_ray_full(eye, surface_centroid, MASK_VISIBLE)
                    confirm_dist = _vec_length(_vec_sub(surface_centroid, eye))
                    if (1.0 - confirm.fraction) * confirm_dist > HIT_CLEARANCE:
                        continue  # Face not visible, only probe space
                if dist < min_dist:
                    min_dist = dist
                return (face_idx, True, round(min_dist, 1))
    
    return (face_idx, False, -1.0)


# ─── Visibility Oracle ────────────────────────────────────────────────────────

class VisibilityOracle:
    """Classify BSP faces as visible or never-visible from reachable positions."""

    def __init__(self, bsp: BSPReader, world: CollisionWorld,
                 eye_positions: List[Vec3], verbose: bool = False,
                 exclude_faces: Optional[Set[int]] = None):
        self.bsp = bsp
        self.world = world
        self.eye_positions = eye_positions
        self.verbose = verbose
        self.exclude_faces: Set[int] = exclude_faces or set()
        
        # Build spatial grid for eye positions
        self.eye_grid = SpatialGrid(eye_positions)
        
        # Pre-compute eye clusters for PVS filtering
        self.eye_clusters: List[int] = []
        for pos in eye_positions:
            self.eye_clusters.append(world.leaf_cluster(pos))
        
        # Load face geometry data
        self.faces = bsp.read_faces()
        self.planes = bsp.read_planes()
        self.vertexes = bsp.read_vertexes()
        self.edges = bsp.read_edges()
        self.surfedges = bsp.read_surfedges()
        self.texinfos = bsp.read_texinfos()
        
        if verbose:
            print(f"  VisibilityOracle: {len(self.faces)} faces, "
                  f"{len(eye_positions)} eye positions"
                  f"{f', {len(self.exclude_faces)} excluded' if self.exclude_faces else ''}")

    def _prepare_face_work_items(self):
        """Pre-compute face work items for parallel processing.
        
        Returns:
            work_items: list of (face_idx, offset_samples, n_surface, normal)
            skipped_count: number of faces skipped (no lightmap, etc.)
        """
        work_items = []
        skipped_count = 0
        
        for fi, face in enumerate(self.faces):
            # Skip explicitly excluded faces (e.g. texlight faces)
            if fi in self.exclude_faces:
                skipped_count += 1
                continue
            
            # Skip faces with no lightmap
            if face.lightofs < 0:
                skipped_count += 1
                continue
            
            # Skip tool faces (nodraw, sky, skip) — non-renderable
            if 0 <= face.texinfo < len(self.texinfos):
                surf_flags = self.texinfos[face.texinfo].flags
                if surf_flags & _SURF_SKIP_VIS:
                    skipped_count += 1
                    continue
            
            # Get face geometry
            verts = self.bsp.get_face_vertices(face, self.vertexes,
                                                self.edges, self.surfedges)
            if len(verts) < 3:
                skipped_count += 1
                continue
            
            # Get face normal — flip by face.side to get the true outward
            # (renderable) direction.  BSP stores the plane canonically; a
            # face with side==1 faces the OPPOSITE way.  Without this flip the
            # +normal sample offset pushes into solid and the backface cull is
            # inverted, marking visible side==1 faces NEVER-VISIBLE (their
            # lightmaps then get destroyed).  Same convention as
            # reachability._build_face_floor_map.
            if face.planenum < len(self.planes):
                normal = self.planes[face.planenum].normal
                if face.side:
                    normal = (-normal[0], -normal[1], -normal[2])
            else:
                skipped_count += 1
                continue
            
            # Generate surface sample points (probe points removed)
            surface_pts, _ = _face_sample_points(verts, normal=normal)
            
            offset_surface = []
            for s in surface_pts:
                offset_surface.append((
                    s[0] + normal[0],
                    s[1] + normal[1],
                    s[2] + normal[2]))
            
            n_surface = len(offset_surface)
            all_samples = offset_surface # probe_pts removed
            
            work_items.append((fi, all_samples, n_surface, normal))
        
        return work_items, skipped_count

    def classify_faces(self, num_workers: int = 0, custom_work_items: Optional[List[Tuple]] = None) -> Dict[int, dict]:
        """Classify visibility of all (or custom) faces using multiprocessing.
        
        Args:
            num_workers: Number of parallel worker processes.
            custom_work_items: Optional list of (id, samples, n_surface, normal) tuples.
        
        Returns dict: face_index (or id) → {"visible": bool, "min_distance": float}
        """
        # Guard: with zero eyes EVERY face would be classified never-visible,
        # which downstream destroys every lightmap.  Return empty so the
        # consumer treats all faces as visible (the safe direction).
        if not self.eye_positions and custom_work_items is None:
            if self.verbose:
                print("  ⚠ No eye positions — cannot classify visibility; "
                      "returning empty (all faces treated as visible).", flush=True)
            return {}

        if num_workers <= 0:
            num_workers = max(1, (os.cpu_count() or 4) - 2)
        
        t0 = time.perf_counter()
        if custom_work_items is not None:
            work_items = custom_work_items
            skipped_count = 0
            t_prep = time.perf_counter() - t0
        else:
            work_items, skipped_count = self._prepare_face_work_items()
            t_prep = time.perf_counter() - t0
            
        total_work = len(work_items)
        
        if self.verbose:
            print(f"  Prepared {total_work} face work items in {t_prep:.2f}s "
                  f"(skipped {skipped_count} non-renderable)")
        
        if total_work == 0:
            return {}
        
        # Use serial mode if only 1 worker or very few faces
        if num_workers == 1 or total_work < 50:
            return self._classify_serial(work_items, skipped_count, t0)
        
        # ─── Parallel mode ────────────────────────────────────────────
        bsp_path = self.bsp.filepath
        
        if self.verbose:
            print(f"  Dispatching to {num_workers} worker processes...",
                  flush=True)
        
        results = {}
        visible_count = 0
        never_visible_count = 0
        completed = 0
        last_print = time.perf_counter()
        
        try:
            with mp.Pool(
                processes=num_workers,
                initializer=_worker_init,
                initargs=(str(bsp_path), self.eye_positions, self.world.vpk_paths),
            ) as pool:
                for face_idx, is_visible, min_dist in pool.imap_unordered(
                    _worker_check_face, work_items, chunksize=16
                ):
                    results[face_idx] = {
                        "visible": is_visible,
                        "min_distance": min_dist if min_dist >= 0 else -1.0,
                    }
                    if is_visible:
                        visible_count += 1
                    else:
                        never_visible_count += 1
                    completed += 1
                    
                    # Progress reporting
                    if self.verbose and completed % 100 == 0:
                        now = time.perf_counter()
                        if now - last_print > 2.0:
                            pct = completed / total_work * 100
                            elapsed = now - t0
                            rate = completed / elapsed if elapsed > 0 else 0
                            eta = (total_work - completed) / rate if rate > 0 else 0
                            print(f"  [{pct:5.1f}%] {completed}/{total_work} "
                                  f"(vis={visible_count}, nv={never_visible_count}) "
                                  f"[{rate:.0f} faces/s, ETA {eta:.0f}s]",
                                  flush=True)
                            last_print = now
        
        except Exception as e:
            if self.verbose:
                print(f"  ⚠ Parallel mode failed: {e}")
                print(f"  Falling back to serial mode...", flush=True)
            return self._classify_serial(work_items, skipped_count, t0)
        
        elapsed = time.perf_counter() - t0
        
        if self.verbose:
            print(f"\n  Classification complete in {elapsed:.2f}s "
                  f"({num_workers} workers)")
            print(f"  Visible: {visible_count}")
            print(f"  Never-visible: {never_visible_count}")
            print(f"  Skipped (no lightmap): {skipped_count}")
            print(f"  Total renderable: {visible_count + never_visible_count}")
            rate = total_work / elapsed if elapsed > 0 else 0
            print(f"  Rate: {rate:.0f} faces/s")
        
        return results

    def _classify_serial(self, work_items, skipped_count: int,
                          t0: float) -> Dict[int, dict]:
        """Fallback serial classification (original behavior)."""
        results = {}
        visible_count = 0
        never_visible_count = 0
        total_work = len(work_items)
        last_print = t0
        
        if self.verbose:
            print(f"  Running serial mode ({total_work} faces)...",
                  flush=True)
        
        for i, (fi, offset_samples, n_surface, normal) in enumerate(work_items):
            # Progress reporting
            if self.verbose and i % 100 == 0:
                now = time.perf_counter()
                if now - last_print > 2.0 or i == 0:
                    pct = i / total_work * 100 if total_work > 0 else 0
                    print(f"  [{pct:5.1f}%] Face {i}/{total_work} "
                          f"(vis={visible_count}, nv={never_visible_count})")
                    last_print = now
            
            is_visible, min_dist = self._check_face_visibility(
                offset_samples, normal, n_surface=n_surface)
            
            results[fi] = {
                "visible": is_visible,
                "min_distance": round(min_dist, 1) if min_dist < MAX_VIS_DISTANCE else -1.0,
            }
            
            if is_visible:
                visible_count += 1
            else:
                never_visible_count += 1
        
        elapsed = time.perf_counter() - t0
        
        if self.verbose:
            print(f"\n  Classification complete in {elapsed:.2f}s (serial)")
            print(f"  Visible: {visible_count}")
            print(f"  Never-visible: {never_visible_count}")
            print(f"  Skipped (no lightmap): {skipped_count}")
            print(f"  Total renderable: {visible_count + never_visible_count}")
        
        return results

    def _check_face_visibility(self, samples: List[Vec3],
                                normal: Vec3,
                                n_surface: int = -1) -> Tuple[bool, float]:
        """Check if any eye position can see any sample point on a face.

        Uses PVS pre-filtering, plus backface culling against the
        side-corrected face normal (an eye behind the surface cannot see
        it).  Mirrors the parallel worker (_worker_check_face) so serial
        and parallel runs classify identically.

        n_surface — number of leading surface samples.  Remaining
                    samples are probes (require centroid confirmation).
                    -1 = all samples are surface samples.
        
        Returns (is_visible, min_distance_to_nearest_eye).
        """
        min_dist = MAX_VIS_DISTANCE
        if n_surface < 0:
            n_surface = len(samples)
        
        # Get face cluster for PVS filtering
        if len(samples) > 0:
            face_cluster = self.world.leaf_cluster(samples[0])
        else:
            return False, min_dist
        
        # Find candidate eye positions within range
        centroid = samples[0]
        nearby_indices = self.eye_grid.query_nearby(centroid, MAX_VIS_DISTANCE)
        
        if not nearby_indices:
            return False, min_dist
        
        # PVS pre-filter: rigidly restrict to VBSP portal visibility clusters
        if face_cluster >= 0:
            pvs_ok = []
            for idx in nearby_indices:
                ec = self.eye_clusters[idx]
                if ec < 0 or self.world.is_cluster_visible(ec, face_cluster):
                    pvs_ok.append(idx)
            nearby_indices = pvs_ok
        
        if not nearby_indices:
            return False, min_dist
        
        # Limit eye positions tested for performance while preserving critical local viewpoints.
        # Instead of purely angular/stratifed culling (which can accidentally cull the only
        # ladder point able to see a tricky face), we prioritize the absolute closest eyes
        # first, then sample the remaining distance bands.
        if len(nearby_indices) > MAX_EYES_PER_FACE:
            dists = []
            for idx in nearby_indices:
                eye = self.eye_positions[idx]
                d = _vec_sub(eye, centroid)
                dist = _vec_length(d)
                dists.append((dist, idx))
            dists.sort()
            
            n_close = MAX_EYES_PER_FACE // 2
            n_mid = MAX_EYES_PER_FACE // 4
            n_far = MAX_EYES_PER_FACE - n_close - n_mid
            
            total = len(dists)
            mid_start = total // 3
            far_start = 2 * total // 3
            
            # 1. Take the absolute closest eyes without skipping any (vital for tight angles)
            selected = [idx for _, idx in dists[:n_close]]
            
            # 2. Sample from mid-range
            mid_pool = dists[mid_start:far_start]
            if mid_pool:
                step = max(1, len(mid_pool) // n_mid)
                selected += [idx for _, idx in mid_pool[::step]][:n_mid]
                
            # 3. Sample from far-range
            far_pool = dists[far_start:]
            if far_pool:
                step = max(1, len(far_pool) // n_far)
                selected += [idx for _, idx in far_pool[::step]][:n_far]
                
            nearby_indices = selected
        
        # Surface centroid for probe confirmation rays
        surface_centroid = samples[0] if samples else None
        
        for eye_idx in nearby_indices:
            eye = self.eye_positions[eye_idx]

            # Backface culling early out: eye must be in front of the face.
            to_eye_global = _vec_sub(eye, centroid)
            if normal is not None and _vec_dot(normal, to_eye_global) <= 0.0:
                continue

            for si, sample in enumerate(samples):
                to_eye = _vec_sub(eye, sample)
                
                # Backface culling early out
                if normal is not None:
                    if to_eye[0]*normal[0] + to_eye[1]*normal[1] + to_eye[2]*normal[2] < 0:
                        continue
                        
                dist = _vec_length(to_eye)
                if dist < 1.0 or dist > MAX_VIS_DISTANCE:
                    continue
                
                hit = self.world.trace_ray_full(eye, sample, MASK_VISIBLE)

                if (1.0 - hit.fraction) * dist <= HIT_CLEARANCE:
                    # Probe hit: confirm the face centroid is reachable.
                    if si >= n_surface and surface_centroid is not None:
                        confirm = self.world.trace_ray_full(eye, surface_centroid, MASK_VISIBLE)
                        confirm_dist = _vec_length(_vec_sub(surface_centroid, eye))
                        if (1.0 - confirm.fraction) * confirm_dist > HIT_CLEARANCE:
                            continue  # Face not visible, only probe space
                    if dist < min_dist:
                        min_dist = dist
                    return True, min_dist
        
        return False, min_dist


# ─── Output ──────────────────────────────────────────────────────────────────

def save_visibility(results: Dict[int, dict], path: str, 
                    verbose: bool = False) -> None:
    """Save visibility results to JSON."""
    visible = sum(1 for r in results.values() if r["visible"])
    never_visible = sum(1 for r in results.values() if not r["visible"])
    
    data = {
        "total_faces": len(results),
        "visible_count": visible,
        "never_visible_count": never_visible,
        "faces": {str(k): v for k, v in results.items()},
    }
    
    with open(path, 'w') as f:
        json.dump(data, f, separators=(',', ':'))
    
    size_kb = os.path.getsize(path) / 1024
    if verbose:
        print(f"  Saved {path} ({size_kb:.1f} KB)")


def load_visibility(path: str) -> Dict[int, dict]:
    """Load visibility data from JSON."""
    with open(path, 'r') as f:
        data = json.load(f)
    return {int(k): v for k, v in data["faces"].items()}


# ─── CLI ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Classify BSP faces as visible or never-visible")
    parser.add_argument("--bsp", required=True, help="Path to BSP file")
    parser.add_argument("--reachability", "-r", required=True,
                        help="Reachability JSON from reachability.py")
    parser.add_argument("--output", "-o", default=None,
                        help="Output JSON file (default: <bsp>_vis.json)")
    parser.add_argument("--workers", "-w", type=int, default=0,
                        help="Number of worker processes (0=auto, 1=serial)")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Print detailed progress")
    args = parser.parse_args()

    if not os.path.exists(args.bsp):
        print(f"ERROR: BSP file not found: {args.bsp}")
        sys.exit(1)
    if not os.path.exists(args.reachability):
        print(f"ERROR: Reachability file not found: {args.reachability}")
        sys.exit(1)

    if args.output is None:
        base = os.path.splitext(args.bsp)[0]
        args.output = base + "_vis.json"

    print(f"\n{'='*60}")
    print(f"  Visibility Oracle — {os.path.basename(args.bsp)}")
    print(f"{'='*60}\n")

    t0 = time.perf_counter()

    # Load BSP and collision world
    bsp = BSPReader(args.bsp)
    bsp.read()
    world = CollisionWorld(bsp, verbose=args.verbose)

    # Load reachability data
    eye_positions = ReachabilityMap.load_eye_positions(args.reachability)
    if not eye_positions:
        print(f"ERROR: No eye positions in {args.reachability}. Aborting — "
              f"classifying with zero eyes would mark EVERY face never-visible "
              f"and destroy all lightmaps.")
        sys.exit(1)
    if args.verbose:
        print(f"  Loaded {len(eye_positions)} eye positions from "
              f"{os.path.basename(args.reachability)}")

    # Run visibility classification
    oracle = VisibilityOracle(bsp, world, eye_positions, verbose=True)
    results = oracle.classify_faces(num_workers=args.workers)

    # Save results
    save_visibility(results, args.output, verbose=True)

    elapsed = time.perf_counter() - t0
    print(f"\n  Total: {elapsed:.2f}s")
    print(f"{'='*60}\n")


if __name__ == "__main__":
    main()
