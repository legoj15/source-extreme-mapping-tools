#!/usr/bin/env python3
"""
make_fixtures -- write the bspwalk fixture VMFs and expect.json sidecars.

    python make_fixtures.py               # every fixture
    python make_fixtures.py --only f03    # one (comma separated; matches the fNN prefix or full name)

Output: game/bin/x64/unit_test_maps/bspwalk/<NAME>.vmf and <NAME>.expect.json.
Deterministic: same input, same ids, byte-identical files.
Exit code: 0 written, 2 usage error (unknown fixture id).  Only the selected fixture modules are
imported, so a broken fixture cannot block building another.
"""
import argparse
import sys
from pathlib import Path

_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

import vmfgen                      # noqa: E402
import fixtures                    # noqa: E402

X64_DIR = _THIS_DIR.parents[1]
MAPS_DIR = X64_DIR / "unit_test_maps" / "bspwalk"


def select(names, only):
    """Module names that equal or start with one of the comma separated ids in `only`.

    Works on names so the caller imports nothing it was not asked for.  Raises ValueError on
    a selection that names no fixture.
    """
    if not only:
        return names
    wanted = [w.strip() for w in only.split(",") if w.strip()]
    if not wanted:
        raise ValueError(f"--only {only!r} names no fixture")
    missing = [w for w in wanted if not any(n.startswith(w) for n in names)]
    if missing:
        raise ValueError(f"no fixture matches: {', '.join(missing)}; have: {', '.join(names)}")
    return [n for n in names if any(n.startswith(w) for w in wanted)]


def build_fixture(mod) -> vmfgen.Map:
    m = vmfgen.Map(name=mod.NAME, description=mod.DESCRIPTION)
    mod.build(m)
    return m


def write_fixture(mod, out_dir: Path = MAPS_DIR) -> vmfgen.Map:
    m = build_fixture(mod)
    out_dir.mkdir(parents=True, exist_ok=True)
    m.write(out_dir / f"{mod.NAME}.vmf", out_dir / f"{mod.NAME}.expect.json")
    return m


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--only", help="comma separated fixture ids/names, e.g. f03 or f01,f04")
    ap.add_argument("--out", type=Path, default=MAPS_DIR, help=f"output directory (default {MAPS_DIR})")
    args = ap.parse_args(argv)

    try:
        picked = select(fixtures.fixture_names(), args.only)
    except ValueError as exc:
        print(f"usage error: {exc}", file=sys.stderr)
        return 2
    if not picked:
        print("usage error: no fixtures found", file=sys.stderr)
        return 2
    try:
        mods = fixtures.discover(picked)
    except ValueError as exc:
        print(f"usage error: {exc}", file=sys.stderr)
        return 2
    for mod in mods:
        m = write_fixture(mod, args.out)
        e = m.expect_dict()
        print(f"{mod.NAME}: {len(m.solids)} solids, {len(m.all_sides())} sides, "
              f"{len(m.entities)} entities, {len(e['must_visible'])} must_visible, "
              f"{len(e['must_hidden'])} must_hidden -> {args.out / (mod.NAME + '.vmf')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
