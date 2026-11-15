"""
VPK Reader — read files from Valve VPK archives.

Supports VPK v1 and v2 directory files (*_dir.vpk).
Zero external dependencies — uses only struct for binary parsing.

Usage:
    vpk = VPKReader("hl2/hl2_textures_dir.vpk")
    data = vpk.read_file("materials/nature/blendgroundtograss001a.vmt")
    if data is not None:
        text = data.decode('utf-8', errors='replace')
"""
from __future__ import annotations

import os
import struct
from pathlib import Path
from typing import Dict, Optional, Tuple

# VPK signature: 0x55AA1234
VPK_SIGNATURE = 0x55AA1234

# Special archive index: file data is in the directory VPK itself
DIRECTORY_INDEX = 0x7FFF


class VPKEntry:
    """A single file entry in the VPK directory tree."""
    __slots__ = ('crc', 'preload_data', 'archive_index', 'offset', 'length')

    def __init__(self, crc: int, preload_data: bytes,
                 archive_index: int, offset: int, length: int):
        self.crc = crc
        self.preload_data = preload_data
        self.archive_index = archive_index
        self.offset = offset
        self.length = length


class VPKReader:
    """Read-only interface to a VPK archive.

    Opens the *_dir.vpk file and parses the directory tree on construction.
    File data is read on demand from the appropriate archive file.
    """

    def __init__(self, dir_vpk_path: str | Path):
        self._dir_path = Path(dir_vpk_path)
        if not self._dir_path.exists():
            raise FileNotFoundError(f"VPK dir not found: {self._dir_path}")

        # entries: lowercase path → VPKEntry
        self._entries: Dict[str, VPKEntry] = {}
        # After header + tree, the data section starts at this offset
        # (for files stored in the dir VPK itself, archive_index == 0x7FFF)
        self._data_offset = 0

        self._parse_dir()

    def _parse_dir(self) -> None:
        """Parse the VPK directory tree."""
        with open(self._dir_path, 'rb') as f:
            # Read header
            sig, version, tree_size = struct.unpack('<III', f.read(12))
            if sig != VPK_SIGNATURE:
                raise ValueError(f"Invalid VPK signature: 0x{sig:08X}")

            if version == 2:
                # v2 has extra header fields we can skip
                f.read(16)  # data_size, archive_md5_size, other_md5_size, sig_size

            tree_start = f.tell()
            self._data_offset = tree_start + tree_size

            # Parse tree: extension → path → filename hierarchy
            # Each level is a sequence of null-terminated strings, ended by empty string
            while True:
                extension = self._read_string(f)
                if extension == '':
                    break

                while True:
                    path = self._read_string(f)
                    if path == '':
                        break

                    while True:
                        filename = self._read_string(f)
                        if filename == '':
                            break

                        # Read entry data: CRC, preload_bytes, archive_index, offset, length
                        entry_data = f.read(18)
                        crc, preload_bytes, archive_index, entry_offset, entry_length, terminator = \
                            struct.unpack('<IHHIIH', entry_data)

                        # Read preload data if present
                        preload_data = b''
                        if preload_bytes > 0:
                            preload_data = f.read(preload_bytes)

                        # Build the full path
                        if path == ' ':
                            full_path = f"{filename}.{extension}"
                        else:
                            full_path = f"{path}/{filename}.{extension}"

                        self._entries[full_path.lower()] = VPKEntry(
                            crc=crc,
                            preload_data=preload_data,
                            archive_index=archive_index,
                            offset=entry_offset,
                            length=entry_length,
                        )

    @staticmethod
    def _read_string(f) -> str:
        """Read a null-terminated string from the file."""
        chars = []
        while True:
            c = f.read(1)
            if c == b'' or c == b'\x00':
                break
            chars.append(c)
        return b''.join(chars).decode('utf-8', errors='replace')

    def _get_archive_path(self, archive_index: int) -> Path:
        """Compute the path to a numbered archive file.

        Given a dir VPK like 'hl2_textures_dir.vpk',
        archive 3 would be 'hl2_textures_003.vpk'.
        """
        stem = self._dir_path.stem  # e.g. 'hl2_textures_dir'
        if stem.endswith('_dir'):
            base = stem[:-4]  # 'hl2_textures'
        else:
            base = stem
        archive_name = f"{base}_{archive_index:03d}.vpk"
        return self._dir_path.parent / archive_name

    def has_file(self, path: str) -> bool:
        """Check if a file exists in this VPK."""
        return path.lower().replace('\\', '/') in self._entries

    def read_file(self, path: str) -> Optional[bytes]:
        """Read a file's contents from the VPK.

        Returns None if the file doesn't exist in this VPK.
        """
        key = path.lower().replace('\\', '/')
        entry = self._entries.get(key)
        if entry is None:
            return None

        # Combine preload data + archive data
        result = bytearray(entry.preload_data)

        if entry.length > 0:
            if entry.archive_index == DIRECTORY_INDEX:
                # Data is in the directory VPK itself, after the tree
                with open(self._dir_path, 'rb') as f:
                    f.seek(self._data_offset + entry.offset)
                    result.extend(f.read(entry.length))
            else:
                # Data is in a numbered archive file
                archive_path = self._get_archive_path(entry.archive_index)
                with open(archive_path, 'rb') as f:
                    f.seek(entry.offset)
                    result.extend(f.read(entry.length))

        return bytes(result)

    def list_files(self, prefix: str = '', extension: str = '') -> list[str]:
        """List all files matching an optional prefix and/or extension.

        Both prefix and extension are matched case-insensitively.
        Extension should NOT include the dot.
        """
        prefix_l = prefix.lower().replace('\\', '/')
        ext_l = ('.' + extension.lower()) if extension else ''

        results = []
        for path in self._entries:
            if prefix_l and not path.startswith(prefix_l):
                continue
            if ext_l and not path.endswith(ext_l):
                continue
            results.append(path)
        return results

    def __contains__(self, path: str) -> bool:
        return self.has_file(path)

    def __len__(self) -> int:
        return len(self._entries)

    def __repr__(self) -> str:
        return f"VPKReader({self._dir_path.name!r}, {len(self._entries)} files)"
