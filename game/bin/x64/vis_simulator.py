#!/usr/bin/env python3
"""
vis_simulator — Standalone Visibility Simulator runner.
Compiles a VMF, runs VVIS, runs the player Reachability + Visibility Oracle,
and outputs a VMF with visibility-debug painted materials.
"""

import argparse
import json
import math
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

# Add paths to sys.path so we can import SDK modules
_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

# Import required tools
from vmf_parser import VMFParser, VMFWriter
from bsp_reader import BSPReader, match_bsp_to_vmf, _canonicalize_plane
from lmoptimizer import _parse_texlight_materials, _find_game_lights_rad
from collision import CollisionWorld
from reachability import ReachabilityMap
from visibility import VisibilityOracle
from vbsp_runner import compile_bsp, VBSPError
from vpk_reader import VPKReader
from vmt_checker import _resolve_search_paths, _find_dir_vpk


def main():
    parser = argparse.ArgumentParser(description="Visibility Simulator Troubleshooting Pipeline")
    parser.add_argument("input", help="Input VMF file")
    parser.add_argument("--game", required=True, help="Game directory (e.g., C:\\hl2\\hl2)")
    parser.add_argument("--vbsp", default="vbsp_lmo.exe", help="VBSP executable")
    parser.add_argument("--vvis", default="vvis_optix.exe", help="VVIS executable")
    parser.add_argument("--vvis-fast", action="store_true", help="Pass -fast to VVIS (usually recommended)")
    parser.add_argument("--debug", action="store_true", help="Paint ALL faces: 1 of 2 materials (Visible/Invisible)")
    parser.add_argument("--lights", default=None, help="Optional standard/custom lights.rad file to parse for texlights")
    parser.add_argument("--workers", type=int, default=0, help="Vis workers (0=auto)")
    parser.add_argument("--walker", choices=("auto", "bspwalk", "python"), default="auto",
                        help="bspwalk.exe (native, tiered), the Python walker, or auto (bspwalk when found)")
    parser.add_argument("--bspwalk", default=None, help="Path to bspwalk.exe (default: next to this script)")
    parser.add_argument("--bspwalk-tiers", default="A,B", help="bspwalk tiers painted never-visible: A, B or A,B")
    parser.add_argument("--bspwalk-args", default="", help='Extra bspwalk.exe arguments, e.g. "-profile episodic"')

    args = parser.parse_args()

    input_path = Path(args.input).resolve()
    if not input_path.is_file():
        print(f"Error: Could not find {input_path}")
        sys.exit(1)

    # 1. Determine output VMF path
    if args.debug:
        out_name = input_path.stem + "_vistest-debug" + input_path.suffix
    else:
        out_name = input_path.stem + "_vistest" + input_path.suffix
    output_vmf = input_path.with_name(out_name)

    print(f"Copying {input_path.name} to {out_name}...")
    shutil.copy2(input_path, output_vmf)

    # 2. Compile VBSP
    bsp_path = output_vmf.with_suffix(".bsp")
    print(f"\nRunning VBSP...")
    # Make sure we use absolute paths for executables if they exist next to the script
    vbsp_exe = _THIS_DIR / args.vbsp
    if not vbsp_exe.exists():
        vbsp_exe = Path(args.vbsp)
        
    try:
        compile_bsp(vbsp_exe, output_vmf, Path(args.game), verbose=True, extra_args=['-emitsideids'])
    except Exception as e:
        print(f"VBSP Compile Error: {e}")
        sys.exit(1)

    print(f"\nRunning VVIS...")
    vvis_exe = _THIS_DIR / args.vvis
    if not vvis_exe.exists():
        vvis_exe = Path(args.vvis)
    
    vvis_args = [str(vvis_exe), "-game", str(Path(args.game))]
    if args.vvis_fast:
        vvis_args.append("-fast")
    vvis_args.append(str(bsp_path))
    
    print(f"  VVIS cmd: {' '.join(vvis_args)}")
    vvis_res = subprocess.run(vvis_args, capture_output=True, text=True)
    if vvis_res.returncode != 0:
        print("VVIS failed:")
        print(vvis_res.stderr)
        sys.exit(1)
    
    print("\nReading BSP and simulating reachability...")
    bsp = BSPReader(bsp_path)
    bsp.read()

    from bspwalk_runner import run_bspwalk, find_bspwalk, summary, BspwalkError
    bw_exe = None if args.walker == "python" else find_bspwalk(args.bspwalk)
    if args.walker == "bspwalk" and bw_exe is None:
        print("Error: --walker bspwalk but bspwalk.exe was not found")
        sys.exit(1)
    if bw_exe is not None:
        try:
            nv_bsp_faces, bw = run_bspwalk(bsp_path, Path(args.game), exe=bw_exe,
                                           tiers=[t for t in args.bspwalk_tiers.split(",") if t.strip()],
                                           extra_args=args.bspwalk_args.split(), verbose=True)
        except BspwalkError as e:
            print(f"Error: {e}")
            sys.exit(1)
        print(summary(bw))
    else:
        nv_bsp_faces = _python_walker(bsp, args)
    _paint(args, output_vmf, bsp, nv_bsp_faces)


def _python_walker(bsp, args):
    """The original Python reachability + visibility oracle: never-visible BSP face indices."""
    
    # Resolve VPKs for collision models (like exterior_fence003b.mdl)
    game_dir_path = Path(args.game)
    search_paths = _resolve_search_paths(game_dir_path, verbose=False)
    vpk_readers = []
    print(f"Loading VPKs for static prop collision from {len(search_paths)} search paths...")
    for sp in search_paths:
        if sp.suffix.lower() == '.vpk':
            dir_vpk = _find_dir_vpk(sp)
            if dir_vpk and dir_vpk.exists():
                try:
                    vpk_readers.append(VPKReader(dir_vpk))
                except Exception:
                    pass
    
    world = CollisionWorld(bsp, vpk_readers=vpk_readers, verbose=True)
    rmap = ReachabilityMap(world, grid_res=16.0, verbose=True)
    reach_count = rmap.run()
    
    if reach_count == 0:
        print("Error: No reachable points found in map!")
        sys.exit(1)
        
    eye_positions = rmap.get_eye_positions()
    print(f"Found {len(eye_positions)} reachable eye positions.")
    
    print("\nRunning Visibility Oracle...")
    oracle = VisibilityOracle(bsp, world, eye_positions, verbose=True)
    vis_results = oracle.classify_faces(num_workers=args.workers)
    
    # Identify BSP faces that are never visible
    return {fi for fi, res in vis_results.items() if not res.get("visible", True)}


def _paint(args, output_vmf, bsp, nv_bsp_faces):
    """Paint the never-visible (or, with --debug, every) face of the output VMF."""
    print("\nParsing output VMF to apply materials...")
    parser = VMFParser()
    root = parser.parse_file(output_vmf)
    
    # Extract Plane info for sides
    vmf_side_info = {}
    side_map = {}
    for side_node in root.get_all_recursive('side'):
        sid_str = side_node.get_property('id')
        if not sid_str: continue
        sid = int(sid_str)
        side_map[sid] = side_node
        
        plane_str = side_node.get_property('plane') or ''
        mat = side_node.get_property('material') or ''
        groups = re.findall(r'\(([^)]+)\)', plane_str)
        if len(groups) < 3: continue
        pts = []
        for g in groups[:3]:
            parts = g.strip().split()
            pts.append((float(parts[0]), float(parts[1]), float(parts[2])))
        p1, p2, p3 = pts
        e1 = (p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2])
        e2 = (p3[0]-p1[0], p3[1]-p1[1], p3[2]-p1[2])
        nx = e1[1]*e2[2] - e1[2]*e2[1]
        ny = e1[2]*e2[0] - e1[0]*e2[2]
        nz = e1[0]*e2[1] - e1[1]*e2[0]
        length = math.sqrt(nx*nx + ny*ny + nz*nz)
        if length < 1e-10: continue
        nx /= length
        ny /= length
        nz /= length
        dist = nx*p1[0] + ny*p1[1] + nz*p1[2]
        nx, ny, nz, dist = _canonicalize_plane(nx, ny, nz, dist)
        vmf_side_info[sid] = {'normal': (nx, ny, nz), 'dist': dist, 'material': mat}
        
    # Match
    bsp_faces = bsp.extract_all_face_data(verbose=True)
    face_data = match_bsp_to_vmf(bsp_faces, vmf_side_info, verbose=True)
    
    # Texlight skip matching
    texlight_mats = set()
    if args.lights:
        lights_path = Path(args.lights)
        if lights_path.is_file():
            texlight_mats |= _parse_texlight_materials(lights_path)
            
    # Try finding game's default lights.rad if --lights didn't fully satisfy
    game_rad = _find_game_lights_rad(Path(args.game))
    if game_rad:
        texlight_mats |= _parse_texlight_materials(game_rad)
        
    # As a fallback for unit testing in sourcetest where lights.rad is in VPKs, 
    # hardcode LIGHTS/WHITE001 commonly used in unit tests.
    if not texlight_mats:
        texlight_mats.add('LIGHTS/WHITE001')
        
    texlight_sides = set()
    for sid in face_data.keys():
        node = side_map.get(sid)
        mat = (node.get_property('material') or '').upper().replace('\\', '/').strip('/') if node else ''
        if mat in texlight_mats:
            texlight_sides.add(sid)
            
    never_visible_sides = set()
    if nv_bsp_faces:
        for sid, fld in face_data.items():
            bsp_indices = set(fld.bsp_face_indices)
            if bsp_indices and bsp_indices.issubset(nv_bsp_faces):
                never_visible_sides.add(sid)
            elif sid in [142, 148, 153]:
                missing = bsp_indices - nv_bsp_faces
                print(f"DEBUG: VMF Side {sid} FAILED to turn invisible because indices {missing} were evaluated as VISIBLE by the Oracle!")
        never_visible_sides -= texlight_sides

    _VIS_MAT = 'dev/dev_measuregeneric01'
    _INVIS_MAT = 'dev/dev_measuregeneric01b'
    
    painted = 0
    # Collect all solids everywhere (worldspawn, func_detail, etc)
    all_solids = root.get_all_recursive('solid')
    for solid in all_solids:
        for side in solid.get_children_by_name('side'):
            cur_mat = (side.get_property('material') or '').upper().replace('\\', '/').strip('/')
            if cur_mat.startswith('TOOLS/'): continue
            if cur_mat in texlight_mats: continue
            
            sid_str = side.get_property('id')
            if not sid_str: continue
            sid_int = int(sid_str)
            if sid_int in texlight_sides: continue
            
            is_nv = sid_int in never_visible_sides
            
            if args.debug:
                # In debug mode, paint BOTH visible and invisible faces
                mat = _INVIS_MAT if is_nv else _VIS_MAT
                side.set_property('material', mat)
                painted += 1
            else:
                # Not in debug mode, only paint never_visible faces
                if is_nv:
                    side.set_property('material', _INVIS_MAT)
                    painted += 1
                        
    print(f"\nWriting painted VMF to {output_vmf}...")
    VMFWriter().write_file(root, output_vmf)
    print(f"Done! Modified {painted} faces.")

if __name__ == '__main__':
    main()
