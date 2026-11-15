"""
VMF KeyValues Parser — lossless round-trip parser for Valve Map Format files.

VMF files use Valve's KeyValues format: nested blocks of key-value pairs
enclosed in braces, with string values quoted. This parser preserves the
complete structure for safe modification and re-serialization.

Example VMF structure:
    world
    {
        "id" "1"
        "classname" "worldspawn"
        solid
        {
            "id" "2"
            side
            {
                "id" "3"
                "plane" "(0 0 0) (1 0 0) (1 1 0)"
                "lightmapscale" "16"
            }
        }
    }
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple, Union


@dataclass
class KVNode:
    """A node in the KeyValues tree.
    
    Each node has a name (the block type, e.g. 'world', 'solid', 'side')
    and contains an ordered list of children, which can be either:
      - KVPair: a key-value pair like "lightmapscale" "16"
      - KVNode: a nested block like side { ... }
    """
    name: str
    children: List[Union[KVPair, KVNode]] = field(default_factory=list)

    def get_property(self, key: str) -> Optional[str]:
        """Get the value of a key-value pair by key name (case-insensitive)."""
        key_lower = key.lower()
        for child in self.children:
            if isinstance(child, KVPair) and child.key.lower() == key_lower:
                return child.value
        return None

    def set_property(self, key: str, value: str) -> bool:
        """Set the value of an existing key-value pair. Returns True if found."""
        key_lower = key.lower()
        for child in self.children:
            if isinstance(child, KVPair) and child.key.lower() == key_lower:
                child.value = value
                return True
        return False

    def get_children_by_name(self, name: str) -> List[KVNode]:
        """Get all child nodes with the given name."""
        return [c for c in self.children
                if isinstance(c, KVNode) and c.name == name]

    def get_all_recursive(self, name: str) -> List[KVNode]:
        """Recursively find all descendant nodes with the given name."""
        results = []
        for child in self.children:
            if isinstance(child, KVNode):
                if child.name == name:
                    results.append(child)
                results.extend(child.get_all_recursive(name))
        return results


@dataclass
class KVPair:
    """A key-value pair in the KeyValues tree."""
    key: str
    value: str


class VMFParseError(Exception):
    """Raised when the VMF parser encounters invalid syntax."""
    pass


class VMFParser:
    """Parses VMF (KeyValues) files into a tree of KVNode and KVPair objects."""

    def __init__(self):
        # Regex to match a quoted string: "content"
        self._quoted_re = re.compile(r'"([^"]*)"')

    def parse_file(self, filepath: Union[str, Path]) -> KVNode:
        """Parse a VMF file and return the root node."""
        filepath = Path(filepath)
        text = filepath.read_text(encoding='utf-8', errors='replace')
        return self.parse_string(text, str(filepath))

    def parse_string(self, text: str, source: str = "<string>") -> KVNode:
        """Parse a VMF string and return a root node containing all top-level blocks."""
        root = KVNode(name="__root__")
        lines = text.splitlines()
        idx = 0
        while idx < len(lines):
            idx, result = self._parse_next(lines, idx, source)
            if result is not None:
                root.children.append(result)
        return root

    def _skip_whitespace_lines(self, lines: List[str], idx: int) -> int:
        """Skip blank lines and return the next non-blank line index."""
        while idx < len(lines) and lines[idx].strip() == '':
            idx += 1
        return idx

    def _parse_next(self, lines: List[str], idx: int,
                    source: str) -> Tuple[int, Optional[Union[KVNode, KVPair]]]:
        """Parse the next element (block or key-value pair) starting at line idx."""
        idx = self._skip_whitespace_lines(lines, idx)
        if idx >= len(lines):
            return idx, None

        line = lines[idx].strip()

        # Skip empty lines and comments
        if line == '' or line.startswith('//'):
            return idx + 1, None

        # Check if this is a closing brace
        if line == '}':
            return idx + 1, None

        # Check for a key-value pair: "key" "value"
        matches = self._quoted_re.findall(lines[idx])
        if len(matches) >= 2:
            return idx + 1, KVPair(key=matches[0], value=matches[1])

        # Otherwise this should be a block name (possibly with { on same or next line)
        # Handle: blockname\n{  or  blockname {
        block_name = line.rstrip('{').strip().strip('"')
        if not block_name:
            # Line is just '{' — shouldn't happen at top level
            raise VMFParseError(
                f"{source}:{idx + 1}: unexpected '{{' without block name")

        # Find the opening brace
        if line.endswith('{'):
            idx += 1
        else:
            idx += 1
            idx = self._skip_whitespace_lines(lines, idx)
            if idx < len(lines) and lines[idx].strip() == '{':
                idx += 1
            else:
                # Treat as a bare name with no block — shouldn't happen in valid VMF
                raise VMFParseError(
                    f"{source}:{idx + 1}: expected '{{' after block name '{block_name}'")

        # Parse block contents until closing brace
        node = KVNode(name=block_name)
        while idx < len(lines):
            peek_idx = self._skip_whitespace_lines(lines, idx)
            if peek_idx >= len(lines):
                break
            if lines[peek_idx].strip() == '}':
                idx = peek_idx + 1
                break
            idx, child = self._parse_next(lines, idx, source)
            if child is not None:
                node.children.append(child)

        return idx, node


class VMFWriter:
    """Serializes a KVNode tree back to VMF format."""

    def write_file(self, root: KVNode, filepath: Union[str, Path]) -> None:
        """Write the KVNode tree to a file."""
        filepath = Path(filepath)
        filepath.parent.mkdir(parents=True, exist_ok=True)
        text = self.write_string(root)
        filepath.write_text(text, encoding='utf-8')

    def write_string(self, root: KVNode) -> str:
        """Serialize the KVNode tree to a string."""
        lines: List[str] = []
        if root.name == "__root__":
            for child in root.children:
                self._write_element(child, lines, depth=0)
        else:
            self._write_element(root, lines, depth=0)
        return '\n'.join(lines) + '\n'

    def _write_element(self, element: Union[KVNode, KVPair],
                       lines: List[str], depth: int) -> None:
        indent = '\t' * depth
        if isinstance(element, KVPair):
            lines.append(f'{indent}"{element.key}" "{element.value}"')
        elif isinstance(element, KVNode):
            lines.append(f'{indent}{element.name}')
            lines.append(f'{indent}{{')
            for child in element.children:
                self._write_element(child, lines, depth + 1)
            lines.append(f'{indent}}}')


# ─── Convenience data classes for extracted VMF data ──────────────────────────

@dataclass
class VMFSide:
    """Extracted face/side data from a VMF brush."""
    id: int
    plane_points: List[Tuple[float, float, float]]  # 3 points defining the plane
    material: str
    lightmapscale: int
    uaxis: str  # raw uaxis string
    vaxis: str  # raw vaxis string
    vertices: Optional[List[Tuple[float, float, float]]] = None  # from vertices_plus
    smoothing_groups: int = 0
    # Reference back to the KVNode for in-place modification
    _node: Optional[KVNode] = field(default=None, repr=False)


@dataclass
class VMFBrush:
    """A brush (solid) from the VMF."""
    id: int
    sides: List[VMFSide]
    _node: Optional[KVNode] = field(default=None, repr=False)


@dataclass
class VMFLight:
    """A light entity from the VMF."""
    classname: str  # 'light', 'light_spot', 'light_environment'
    origin: Tuple[float, float, float]
    color: Tuple[float, float, float]
    intensity: float
    # Attenuation (for point/spot lights)
    constant_attn: float = 0.0
    linear_attn: float = 0.0
    quadratic_attn: float = 1.0
    # Spot light cone angles (degrees)
    inner_cone: float = 30.0
    outer_cone: float = 45.0
    # Sun direction (for light_environment)
    sun_direction: Optional[Tuple[float, float, float]] = None
    # Ambient (for light_environment)
    ambient_color: Optional[Tuple[float, float, float]] = None
    ambient_intensity: float = 0.0
    # Spot light direction
    spot_direction: Optional[Tuple[float, float, float]] = None


def _parse_vector(s: str) -> Tuple[float, float, float]:
    """Parse a space-separated vector string like '128 64 32'."""
    parts = s.strip().split()
    return (float(parts[0]), float(parts[1]), float(parts[2]))


def _parse_plane_points(s: str) -> List[Tuple[float, float, float]]:
    """Parse a plane definition like '(0 0 0) (128 0 0) (128 128 0)'."""
    # Find all groups within parentheses
    groups = re.findall(r'\(([^)]+)\)', s)
    return [_parse_vector(g) for g in groups[:3]]


def _parse_light_value(s: str) -> Tuple[Tuple[float, float, float], float]:
    """Parse a _light value like '255 255 255 200' -> ((r,g,b), intensity)."""
    parts = s.strip().split()
    r, g, b = float(parts[0]), float(parts[1]), float(parts[2])
    intensity = float(parts[3]) if len(parts) > 3 else 1.0
    return ((r, g, b), intensity)


def _angles_to_direction(angles_str: str, pitch: Optional[str] = None
                          ) -> Tuple[float, float, float]:
    """Convert Hammer angles + pitch to a direction vector.
    
    angles is 'pitch yaw roll' but for light_environment the pitch key
    overrides the pitch in angles. The pitch convention is:
    -90 = straight down, 0 = horizontal, 90 = straight up.
    The yaw is standard: 0 = +X, 90 = +Y.
    """
    import math
    parts = angles_str.strip().split()
    # angles format: "pitch yaw roll"
    ang_pitch = float(parts[0]) if len(parts) > 0 else 0.0
    ang_yaw = float(parts[1]) if len(parts) > 1 else 0.0

    # pitch key overrides
    if pitch is not None:
        ang_pitch = float(pitch)

    # Convert to radians (Source uses degrees)
    pitch_rad = math.radians(ang_pitch)
    yaw_rad = math.radians(ang_yaw)

    # Direction vector (pitch is negative = downward in Source)
    dx = math.cos(pitch_rad) * math.cos(yaw_rad)
    dy = math.cos(pitch_rad) * math.sin(yaw_rad)
    dz = math.sin(pitch_rad)

    return (dx, dy, dz)


def extract_brushes(root: KVNode) -> List[VMFBrush]:
    """Extract all brush (solid) data from a parsed VMF tree."""
    brushes = []
    for solid_node in root.get_all_recursive('solid'):
        solid_id = int(solid_node.get_property('id') or '0')
        sides = []
        for side_node in solid_node.get_children_by_name('side'):
            side_id = int(side_node.get_property('id') or '0')
            plane_str = side_node.get_property('plane') or ''
            material = side_node.get_property('material') or ''
            lm_scale = int(float(side_node.get_property('lightmapscale') or '16'))
            uaxis = side_node.get_property('uaxis') or ''
            vaxis = side_node.get_property('vaxis') or ''
            sg = int(side_node.get_property('smoothing_groups') or '0')

            # Parse vertices_plus if present
            vertices = None
            vp_nodes = side_node.get_children_by_name('vertices_plus')
            if vp_nodes:
                vertices = []
                for child in vp_nodes[0].children:
                    if isinstance(child, KVPair) and child.key == 'v':
                        vertices.append(_parse_vector(child.value))

            sides.append(VMFSide(
                id=side_id,
                plane_points=_parse_plane_points(plane_str) if plane_str else [],
                material=material,
                lightmapscale=lm_scale,
                uaxis=uaxis,
                vaxis=vaxis,
                vertices=vertices,
                smoothing_groups=sg,
                _node=side_node,
            ))

        if sides:
            brushes.append(VMFBrush(id=solid_id, sides=sides, _node=solid_node))
    return brushes


def extract_lights(root: KVNode) -> List[VMFLight]:
    """Extract all light entities from a parsed VMF tree."""
    lights = []
    for entity_node in root.get_children_by_name('entity'):
        classname = entity_node.get_property('classname') or ''
        if classname not in ('light', 'light_spot', 'light_environment'):
            continue

        origin_str = entity_node.get_property('origin') or '0 0 0'
        origin = _parse_vector(origin_str)

        light_str = entity_node.get_property('_light') or '255 255 255 200'
        color, intensity = _parse_light_value(light_str)

        light = VMFLight(
            classname=classname,
            origin=origin,
            color=color,
            intensity=intensity,
        )

        # Attenuation
        c = entity_node.get_property('_constant_attn')
        if c is not None:
            light.constant_attn = float(c)
        l = entity_node.get_property('_linear_attn')
        if l is not None:
            light.linear_attn = float(l)
        q = entity_node.get_property('_quadratic_attn')
        if q is not None:
            light.quadratic_attn = float(q)

        # Spot light properties
        if classname == 'light_spot':
            ic = entity_node.get_property('_inner_cone')
            if ic is not None:
                light.inner_cone = float(ic)
            oc = entity_node.get_property('_cone')
            if oc is not None:
                light.outer_cone = float(oc)
            # Spot direction from angles
            angles = entity_node.get_property('angles') or '0 0 0'
            pitch = entity_node.get_property('pitch')
            light.spot_direction = _angles_to_direction(angles, pitch)

        # Environment light properties
        if classname == 'light_environment':
            angles = entity_node.get_property('angles') or '0 0 0'
            pitch = entity_node.get_property('pitch')
            light.sun_direction = _angles_to_direction(angles, pitch)

            ambient_str = entity_node.get_property('_ambient')
            if ambient_str:
                amb_color, amb_intensity = _parse_light_value(ambient_str)
                light.ambient_color = amb_color
                light.ambient_intensity = amb_intensity

        lights.append(light)
    return lights
