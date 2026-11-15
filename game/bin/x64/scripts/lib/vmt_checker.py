"""
VMT Checker — detect materials with %detailtype defined.

Looks each map material's VMT up along the Source engine search path (loose
files + VPKs, gameinfo_search) to identify materials that define
%detailtype. These materials need a minimum lightmapscale (typically ≥ 5) to
ensure VBSP generates detail props correctly.

Usage:
    from vmt_checker import scan_materials
    detail_mats = scan_materials(material_names, game_dir)
    # detail_mats is a set of material names (lowercase) that have %detailtype
"""
from __future__ import annotations

from pathlib import Path
from typing import List, Optional, Set

from gameinfo_search import find_in, search_paths


def scan_materials(
    material_names: Set[str] | List[str],
    game_dir: Path,
    verbose: bool = False,
    bin_root: Optional[Path] = None,
) -> Set[str]:
    """Scan materials for %detailtype across the game's search path.

    Args:
        material_names: Set of material names as they appear in the VMF
                        (e.g. 'nature/blendgroundtograss001a'). Case-insensitive.
        game_dir: Game directory (the --game dir, e.g. game/mod_hl2mp).
        verbose: Print detailed progress.
        bin_root: The compile's -binroot (the search path's base), if any.

    Each VMT is the first hit along gameinfo_search.search_paths, in the
    order the compile tools' filesystem searches. A VPK that cannot be read
    is skipped, as the filesystem skips a pack it cannot mount.

    Returns:
        Set of material names (lowercase) that have %detailtype defined.
    """
    materials_to_check = {m.lower().replace('\\', '/') for m in material_names}
    if not materials_to_check:
        return set()

    paths = search_paths(game_dir, bin_root)
    if verbose:
        print(f"    Resolved {len(paths)} search paths")

    found_detail: Set[str] = set()
    not_found = 0
    for mat_name in materials_to_check:
        hit = find_in(paths, f"materials/{mat_name}.vmt", strict=False)
        if hit is None:
            not_found += 1
            continue
        try:
            data = hit.read_bytes()
        except OSError:
            continue
        if _vmt_has_detailtype(data):
            found_detail.add(mat_name)

    if verbose and not_found:
        print(f"    {not_found} materials not found in any search path",
              flush=True)
    return found_detail


def _vmt_has_detailtype(data: bytes) -> bool:
    """Check if VMT file data contains a %detailtype definition."""
    try:
        text = data.decode('utf-8', errors='replace').lower()
    except Exception:
        return False

    # Match: "%detailtype" or $detailtype (some VMTs use $ instead of %)
    # Must appear as a key, not inside a comment
    for line in text.splitlines():
        line = line.strip()
        if line.startswith('//'):
            continue
        if '%detailtype' in line or '$detailtype' in line:
            return True
    return False
