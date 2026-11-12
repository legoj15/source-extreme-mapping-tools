"""
bspwalk fixture registry.

One module per fixture, named `fNN_<slug>.py`, exposing:

    NAME         str   file stem of the VMF/expect.json, must equal the module name
    DESCRIPTION  str   one line, goes into expect.json
    build(m)     fn    adds geometry/entities to a vmfgen.Map and marks sides with
                       m.expect_visible(...) / m.expect_hidden(...)

Fixtures are found by globbing `fNN_*.py` in this directory; adding a file is
all it takes to register one.
"""
from __future__ import annotations

import importlib
import re
from pathlib import Path
from typing import Iterable, List, Optional

_FIXTURE_RE = re.compile(r"^f\d\d_[a-z0-9_]+$")


def fixture_names() -> List[str]:
    """Module names of every fixture file, ordered; nothing is imported."""
    here = Path(__file__).resolve().parent
    names = []
    by_number = {}
    for path in sorted(here.glob("f[0-9][0-9]_*.py")):
        if not _FIXTURE_RE.match(path.stem):
            raise ValueError(f"fixture file name {path.name} must match fNN_lowercase_slug.py")
        # fixtures are picked by number (--only f11, the mutants' table): two with one number would
        # silently stand in for each other (#48)
        number = path.stem[:3]
        if number in by_number:
            raise ValueError(f"fixtures {by_number[number]} and {path.stem} share the number {number}; renumber one")
        by_number[number] = path.stem
        names.append(path.stem)
    return names


def discover(names: Optional[Iterable[str]] = None) -> List:
    """Import and return the fixture modules called `names` (default: every one), ordered by name.

    Only the named modules are imported, so a broken fixture cannot block building another.
    """
    wanted = set(fixture_names() if names is None else names)
    mods = []
    for stem in fixture_names():
        if stem not in wanted:
            continue
        mod = importlib.import_module(f"{__package__}.{stem}")
        for attr in ("NAME", "DESCRIPTION", "build"):
            if not hasattr(mod, attr):
                raise ValueError(f"fixture {stem} is missing {attr}")
        if mod.NAME != stem:
            raise ValueError(f"fixture {stem}: NAME is {mod.NAME!r}, must equal the module name")
        mods.append(mod)
    return mods
