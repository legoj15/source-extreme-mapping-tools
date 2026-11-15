"""Where things live in game/bin/x64, derived from this file's own location.

This file is game/bin/x64/scripts/lib/binpaths.py. Every script, test and
harness that needs the bin folder, the scripts, the tests or the repo root
takes it from here instead of counting `..` from its own __file__.

  BIN        game/bin/x64: the tool binaries (exe, dll, ptx) and
             bspwalk_profiles.txt. The tools load their PTX from the working
             directory, so run them with cwd=BIN.
  SCRIPTS    BIN/scripts: command-line tools
  LIB        BIN/scripts/lib: modules that are only imported
  GUI        BIN/gui
  TESTS      BIN/tests: suites, fixture maps (maps/, compare/maps/), harnesses
  TESTS_OUT  TESTS/out: git-ignored logs and run products
  REPO       the repository root (or worktree root) holding BIN
  exe(name)  BIN/name, the path of a tool binary: exe('vbsp_lmo.exe')

Entry points run under `python -I`, which leaves the script's own folder off
sys.path, so each one first puts scripts/lib on sys.path from its __file__
(the one-line preamble at the top of each script) and then imports this.
"""
from pathlib import Path

LIB = Path(__file__).resolve().parent
SCRIPTS = LIB.parent
BIN = SCRIPTS.parent
GUI = BIN / 'gui'
TESTS = BIN / 'tests'
TESTS_OUT = TESTS / 'out'
REPO = BIN.parents[2]


def exe(name: str) -> Path:
    """The path of a tool binary in BIN; name includes its extension."""
    return BIN / name
