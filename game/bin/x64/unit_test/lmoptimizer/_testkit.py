"""Shared helpers for the lmoptimizer tests. Not collected (no test_ prefix).

Each test module is both a script (`python test_x.py`, exit 0 = all pass) and
a pytest module (`pytest game/bin/x64/unit_test/lmoptimizer`). check() makes
both work: a failed check raises AssertionError, which fails the pytest test
and, in script mode, is recorded by run_all() so the process exits nonzero
after the remaining tests have still run.
"""
import struct
import sys
import traceback
from pathlib import Path

# Make game/bin/x64 importable
BIN_DIR = Path(__file__).resolve().parents[2]
if str(BIN_DIR) not in sys.path:
    sys.path.insert(0, str(BIN_DIR))

FAILURES = []


class CheckFailed(AssertionError):
    """A failed check(): already printed and recorded."""


def check(name, cond, detail=""):
    if cond:
        print(f"  ok    {name}")
        return
    print(f"  FAIL  {name}  {detail}")
    FAILURES.append(name)
    raise CheckFailed(f"{name} {detail}".strip())


def run_all(label, tests) -> int:
    """Script entry point: run every test (continuing past failures) and
    return the process exit code, 1 if anything failed."""
    # The tool under test prints box-drawing characters; a piped Windows
    # console is cp1252. (lmoptimizer.main() does this itself; the tests call
    # its internals directly.)
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding='utf-8', errors='replace')
        except (AttributeError, ValueError, OSError):
            pass
    print(f"{label}:")
    for test in tests:
        try:
            test()
        except CheckFailed:
            pass  # check() already printed and recorded it
        except Exception:  # includes a raw `assert` (AssertionError) in a test
            traceback.print_exc()
            FAILURES.append(test.__name__)
    if FAILURES:
        print(f"\n{len(FAILURES)} FAILED: {', '.join(FAILURES)}")
        return 1
    print("\nAll tests passed.")
    return 0


def run_main(argv) -> int:
    """lmoptimizer.main(argv): every pipeline failure is a return value (the
    library raises PipelineAbort and main() turns it into 1), so nothing here
    catches SystemExit. Only argparse usage errors still exit that way."""
    import lmoptimizer
    return lmoptimizer.main(list(argv))


# Source SDK lump ids, hard-coded on purpose: importing them from bsp_reader
# would make a wrong id in bsp_reader invisible to the tests.
SDK_LUMP_LIGHTING = 8
SDK_LUMP_LIGHTING_HDR = 53


def write_bsp(path, ldr_len=0, hdr_len=0, ldr_comp=0, hdr_comp=0):
    """Minimal BSP: VBSP ident + version + 64 lump directory entries, followed
    by zero-filled payload for each non-empty lighting lump (so the lump
    really lies inside the file). *_comp is the lump_t fourCC field, the LZMA
    uncompressed size (0 = stored plain)."""
    header_size = 8 + 64 * 16
    lumps = [(0, 0, 0, 0)] * 64
    ofs = header_size
    payload = b''
    for lump_id, length, comp in ((SDK_LUMP_LIGHTING, ldr_len, ldr_comp),
                                  (SDK_LUMP_LIGHTING_HDR, hdr_len, hdr_comp)):
        if length:
            lumps[lump_id] = (ofs, length, 0, comp)
            payload += bytes(length)
            ofs += length
    with open(path, 'wb') as f:
        f.write(b'VBSP')
        f.write(struct.pack('<i', 20))
        for entry in lumps:
            f.write(struct.pack('<4i', *entry))
        f.write(payload)


# A small map: one 1088x464 displacement (needs scale >= 9 for VBSP's extent
# check) plus five nodraw sides, and an info_player_start.
MINI_VMF = """versioninfo
{
"editorversion" "400"
"editorbuild" "8864"
"mapversion" "1"
"formatversion" "100"
"prefab" "0"
}
visgroups
{
}
world
{
"id" "1"
"mapversion" "1"
"classname" "worldspawn"
"skyname" "sky_day01_01"
solid
{
"id" "100"
side
{
"id" "1"
"plane" "(-4224 -3248 0) (-4224 -3712 0) (-5312 -3712 0)"
"material" "NATURE/RED_GRASS"
"uaxis" "[1 0 0 0] 0.25"
"vaxis" "[0 1 0 -64] 0.25"
"rotation" "0"
"lightmapscale" "16"
"smoothing_groups" "0"
dispinfo
{
"power" "3"
"startposition" "[-5312 -3712 0]"
"flags" "0"
"elevation" "0"
"subdiv" "0"
}
}
side
{
"id" "2"
"plane" "(-5312 -3712 -16) (-4224 -3712 -16) (-4224 -3248 -16)"
"material" "TOOLS/TOOLSNODRAW"
"uaxis" "[1 0 0 0] 0.25"
"vaxis" "[0 1 0 0] 0.25"
"rotation" "0"
"lightmapscale" "16"
"smoothing_groups" "0"
}
side
{
"id" "3"
"plane" "(-5312 -3248 0) (-5312 -3712 0) (-5312 -3712 -16)"
"material" "TOOLS/TOOLSNODRAW"
"uaxis" "[0 1 0 0] 0.25"
"vaxis" "[0 0 -1 0] 0.25"
"rotation" "0"
"lightmapscale" "16"
"smoothing_groups" "0"
}
side
{
"id" "4"
"plane" "(-4224 -3248 -16) (-4224 -3712 -16) (-4224 -3712 0)"
"material" "TOOLS/TOOLSNODRAW"
"uaxis" "[0 1 0 0] 0.25"
"vaxis" "[0 0 -1 0] 0.25"
"rotation" "0"
"lightmapscale" "16"
"smoothing_groups" "0"
}
side
{
"id" "5"
"plane" "(-5312 -3248 -16) (-4224 -3248 -16) (-4224 -3248 0)"
"material" "TOOLS/TOOLSNODRAW"
"uaxis" "[1 0 0 0] 0.25"
"vaxis" "[0 0 -1 0] 0.25"
"rotation" "0"
"lightmapscale" "16"
"smoothing_groups" "0"
}
side
{
"id" "6"
"plane" "(-5312 -3712 0) (-4224 -3712 0) (-4224 -3712 -16)"
"material" "TOOLS/TOOLSNODRAW"
"uaxis" "[1 0 0 0] 0.25"
"vaxis" "[0 0 -1 0] 0.25"
"rotation" "0"
"lightmapscale" "16"
"smoothing_groups" "0"
}
}
}
entity
{
"id" "200"
"classname" "info_player_start"
"origin" "-4768 -3480 24"
"angles" "0 0 0"
}
cameras
{
"activecamera" "0"
}
cordon
{
"mins" "(-8192 -8192 -8192)"
"maxs" "(8192 8192 8192)"
"active" "0"
}
"""
