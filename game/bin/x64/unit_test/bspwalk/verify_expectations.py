#!/usr/bin/env python3
"""
verify_expectations -- check every fixture's must_visible / must_hidden claims by brute force.

    python verify_expectations.py               # every fixture
    python verify_expectations.py --only f11    # some (comma separated fNN ids or names)
    python verify_expectations.py --mutants     # also prove the checker can fail (see below)

Independent of any walker: it builds the fixture's vmfgen.Map and uses only the map's own
solids and the eye regions the fixture declares (Map.verify_floor / verify_eye_box /
verify_viewpoint / verify_sky / verify_mirror, issue #19).

Eyes.  A declared floor is a rectangle at height z; its eyes are a lattice over it (16u or
finer, both ends and the middle of each axis included, and the ends inset by 16u, half a hull,
so a position flush to a wall is always sampled) at heights z + 28 .. z + 85.33 (crouch eye to
the highest jump eye at jump speed 160, gravity 600) and z + 64 (the standing eye).  A declared
box is a lattice over the box.  A lattice eye is dropped when a crouched hull there (32 x 32,
feet 28 below the eye, 36 tall) would overlap a player-solid solid, so the declarations can be
whole floors and rooms.  Viewpoints are single eyes.
Dead eyes.  A declared floor also gives a dead player's view, z + 14 (HL2 VEC_DEAD_VIEWHEIGHT)
up to the crouch eye, over the same x/y lattice: bspwalk adds that layer over every reached
floor.  The z + 14 eye would put a crouched hull in the floor, so these skip the hull test at
their own height and take its footprint test at the crouch eye over the same x/y instead (a
body lies where a crouched hull fits).  With a sky declaration every eye E also gives
origin + E / scale.  A mirror at height z over a rectangle adds the reflected paths: E -> Q on
the mirror -> P, with both segments clear and P above the mirror.

Occluders: world solids (not brush entities), except playerclip, water, trigger and nodraw-
only tool brushes, solids with a displaced side, and verify_skip_solid ones.  Props never
occlude.  Player-solid (for the hull test): the same plus playerclip.

A side is seen from an eye when the eye is strictly in front of its plane and the segment from
the eye to a point of the side (lifted 0.1u off it, edges inset 0.05u) passes through no
occluder's open interior.  Face points: an n x n grid over the side (n grows with its size,
at most 12u apart for must_hidden) plus its centroid.

Verdicts per claim:
  must_visible  OK when some exact eye sees some face point (direct, through a mirror, or
                from a viewpoint).
  must_hidden   OK when no eye of either role sees any face point.
Fixtures that declare no eyes are reported as "not declared" and skipped.

--mutants runs deliberately wrong variants that each must produce failures: an eye placed in
f01's void room, f11 with an eye in the hall, f16 without its cubemap viewpoint, f13 without
the sky mapping, f21 without the dead-eye layer.  Exit code: 0 all claims hold (and every
mutant failed), 1 otherwise, 2 usage.
"""
import argparse
import contextlib
import io
import math
import multiprocessing
import os
import sys
from pathlib import Path

try:
    import numpy as np
except ImportError:  # pragma: no cover
    print("verify_expectations needs numpy (pip install numpy)", file=sys.stderr)
    sys.exit(2)

_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

import vmfgen          # noqa: E402
import fixtures        # noqa: E402

EYE_LOW = vmfgen.EYE_DUCK
EYE_DEAD = vmfgen.EYE_DEAD
HULL_HALF = vmfgen.HULL_STAND[0] / 2         # 16: the hull's reach from its centre
EYE_HIGH = vmfgen.EYE_STAND + vmfgen.JUMP_SPEED ** 2 / (2 * vmfgen.SV_GRAVITY)     # 85.33
LATTICE = 16.0
FACE_STEP_HIDDEN = 12.0
FACE_N_VISIBLE = 9
LIFT = 0.1
INSET = 0.05
NON_OCCLUDING = ("TOOLS/TOOLSPLAYERCLIP", "TOOLS/TOOLSTRIGGER", "TOOLS/TOOLSCLIP")


def _is_water(mat: str) -> bool:
    return "WATER" in mat.upper()


class World:
    """Convex solids as plane sets, for segment tests."""

    def __init__(self, solids):
        self.planes = []
        self.boxes = []
        # biggest first: a blocked segment skips every later solid, and big walls block most
        def extent(s):
            lo, hi = s.mins, s.maxs
            d = sorted(hi[i] - lo[i] for i in range(3))
            return d[1] * d[2]
        for s in sorted(solids, key=extent, reverse=True):
            ns, ds = [], []
            for sd in s.sides:
                n = np.array(sd.unit_normal, float)
                ns.append(n)
                ds.append(float(np.dot(n, np.array(sd.verts[0], float))))
            self.planes.append((np.array(ns), np.array(ds)))
            self.boxes.append((np.array(s.mins, float), np.array(s.maxs, float)))

    def blocked(self, A, B, eps=1e-6):
        """True where the open segment A -> B passes through some solid's interior."""
        A = np.asarray(A, float)
        B = np.asarray(B, float)
        out = np.zeros(len(A), bool)
        if not len(A):
            return out
        lo = np.minimum(A, B)
        hi = np.maximum(A, B)
        D = B - A
        for (ns, ds), (bmin, bmax) in zip(self.planes, self.boxes):
            near = np.all(hi >= bmin, axis=1) & np.all(lo <= bmax, axis=1) & ~out
            idx = np.nonzero(near)[0]
            if not len(idx):
                continue
            a, d = A[idx], D[idx]
            tmin = np.zeros(len(idx))
            tmax = np.ones(len(idx))
            ok = np.ones(len(idx), bool)
            for n, dist in zip(ns, ds):
                num = dist - a @ n
                den = d @ n
                with np.errstate(divide="ignore", invalid="ignore"):
                    t = num / den
                pos = den > 1e-12
                neg = den < -1e-12
                par = ~(pos | neg)
                tmax = np.where(pos, np.minimum(tmax, t), tmax)
                tmin = np.where(neg, np.maximum(tmin, t), tmin)
                ok &= ~(par & (num <= 0))
            hit = ok & ((tmax - tmin) * np.linalg.norm(d, axis=1) > eps)
            out[idx[hit]] = True
        return out

    def hull_hits(self, E):
        """True where a crouched hull with its eye at E overlaps a solid's interior (boxes only:
        every fixture solid that blocks the player is axis-aligned or a wedge, so its box is a
        superset and the hull test errs toward dropping an eye; wedges get the exact test)."""
        E = np.asarray(E, float)
        lo = E + np.array([-HULL_HALF, -HULL_HALF, -EYE_LOW])
        hi = E + np.array([HULL_HALF, HULL_HALF, vmfgen.HULL_DUCK[2] - EYE_LOW])
        out = np.zeros(len(E), bool)
        for (ns, ds), (bmin, bmax) in zip(self.planes, self.boxes):
            over = np.all(hi > bmin + 1e-6, axis=1) & np.all(lo < bmax - 1e-6, axis=1)
            if not over.any():
                continue
            axial = all(np.count_nonzero(np.abs(n) > 1e-9) == 1 for n in ns)
            if axial:
                out |= over
                continue
            # exact: the box overlaps the convex unless a plane of the convex separates it
            idx = np.nonzero(over)[0]
            sep = np.zeros(len(idx), bool)
            for n, dist in zip(ns, ds):
                # support of the hull box along n: its lowest point along n
                c = np.where(n >= 0, lo[idx], hi[idx]) @ n
                sep |= c >= dist - 1e-6
            out[idx[~sep]] = True
        return out


def _lattice(a: float, b: float, step: float):
    if b - a < 1e-9:
        return [a]
    n = max(2, int(math.ceil((b - a) / step)) + 1)
    vals = list(np.linspace(a, b, n))
    vals.append((a + b) / 2)
    return sorted(set(round(v, 6) for v in vals))


def _xy_lattice(a: float, b: float):
    """_lattice over a declared extent plus the ends of its hull-feasible range (inset by half
    a hull), so the extreme legal eyes (flush against a wall) are always sampled."""
    vals = _lattice(a, b, LATTICE)
    if b - a >= 2 * HULL_HALF:
        vals += [a + HULL_HALF, b - HULL_HALF]
    return sorted(set(round(v, 6) for v in vals))


def declared_eyes(spec, player_world, role, dead=True):
    """Eye points of the declarations with this role ('exact', or 'all' for exact + hidden).

    dead=False leaves out the dead-view layer (the --mutants check that a fixture needs it)."""
    pts = []
    dead_pts = []       # (x, y, eye z, floor z) of a dead view over a declared floor
    for (x0, y0, x1, y1, z), r, _why in spec.get("floors", []):
        if role == "exact" and r != "exact":
            continue
        # the crouch eye up to the highest jump eye, plus the standing eye
        zs = sorted(set(_lattice(z + EYE_LOW, z + EYE_HIGH, LATTICE)) | {round(z + vmfgen.EYE_STAND, 6)})
        # a dead view, below the crouch eye (that one is in zs)
        dead_zs = _lattice(z + EYE_DEAD, z + EYE_LOW, LATTICE)[:-1]
        for x in _xy_lattice(x0, x1):
            for y in _xy_lattice(y0, y1):
                for ez in zs:
                    pts.append((x, y, ez))
                for ez in dead_zs:
                    dead_pts.append((x, y, ez, z))
    for mins, maxs, r, _why in spec.get("boxes", []):
        if role == "exact" and r != "exact":
            continue
        for x in _xy_lattice(mins[0], maxs[0]):
            for y in _xy_lattice(mins[1], maxs[1]):
                for ez in _lattice(mins[2], maxs[2], LATTICE):
                    pts.append((x, y, ez))
    E = np.array(pts, float).reshape(-1, 3)
    if len(E):
        E = E[~player_world.hull_hits(E)]
    if dead and dead_pts:
        # A dead view is not a hull position (14u up, the crouch hull would be in the floor), so it
        # skips the hull test at its own height; the player died on a floor spot a crouched hull
        # fits, though, so it takes the footprint test of the crouch eye over that spot.
        D = np.array(dead_pts, float).reshape(-1, 4)
        foot = D[:, :3].copy()
        foot[:, 2] = D[:, 3] + EYE_LOW
        E = np.vstack([E, D[~player_world.hull_hits(foot), :3]])
    vp = [p for p, _why in spec.get("viewpoints", [])]
    V = np.array(vp, float).reshape(-1, 3)
    sky = spec.get("sky")
    if sky is not None and len(E):
        origin, scale = np.array(sky[0], float), sky[1]
        E = np.vstack([E, origin + E / scale])
    return np.vstack([E, V]) if len(V) else E


def face_points(side, step=None, n=None):
    V = np.array(side.verts, float)
    nrm = np.array(side.unit_normal, float)
    c = V.mean(axis=0)
    e1 = V[1] - V[0]
    e1 -= nrm * np.dot(e1, nrm)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(nrm, e1)
    P2 = np.stack([(V - c) @ e1, (V - c) @ e2], axis=1)
    ang = np.arctan2(P2[:, 1], P2[:, 0])
    Q = P2[np.argsort(ang)]
    lo, hi = P2.min(axis=0), P2.max(axis=0)
    if n is None:
        n = max(3, min(129, int(math.ceil(max(hi - lo) / step)) + 1))
    us, vs = np.linspace(lo[0], hi[0], n), np.linspace(lo[1], hi[1], n)
    U, W = np.meshgrid(us, vs)
    cand = np.stack([U.ravel(), W.ravel()], axis=1)
    inside = np.ones(len(cand), bool)
    for i in range(len(Q)):
        a, b = Q[i], Q[(i + 1) % len(Q)]
        ed = b - a
        L = np.linalg.norm(ed)
        if L < 1e-9:
            continue
        cr = ed[0] * (cand[:, 1] - a[1]) - ed[1] * (cand[:, 0] - a[0])
        inside &= cr / L >= INSET
    pts = c + cand[inside, 0:1] * e1 + cand[inside, 1:2] * e2
    return np.vstack([pts, c[None, :]]) + nrm * LIFT, nrm


def seen(world, E, P, nrm, mirrors, stop_at_first):
    """Number of clear (eye, point) pairs, direct or through a mirror."""
    total = 0
    chunk = max(1, 400000 // max(1, len(P)))
    for i in range(0, len(E), chunk):
        Eg = E[i:i + chunk]
        EE = np.repeat(Eg, len(P), axis=0)
        PP = np.tile(P, (len(Eg), 1))
        front = (EE - PP) @ nrm > 1e-6
        idx = np.nonzero(front)[0]
        if len(idx):
            total += int((~world.blocked(EE[idx], PP[idx])).sum())
        for z, x0, y0, x1, y1 in mirrors:
            Er = EE.copy()
            Er[:, 2] = 2 * z - EE[:, 2]
            above = (PP[:, 2] > z + 1e-6) & (EE[:, 2] > z + 1e-6)
            frontr = (Er - PP) @ nrm > 1e-6
            d = PP - Er
            with np.errstate(divide="ignore", invalid="ignore"):
                t = (z - Er[:, 2]) / d[:, 2]
            Q = Er + d * t[:, None]
            inw = (Q[:, 0] > x0) & (Q[:, 0] < x1) & (Q[:, 1] > y0) & (Q[:, 1] < y1) & (t > 0) & (t < 1)
            idx = np.nonzero(above & frontr & inw)[0]
            if len(idx):
                clear = ~(world.blocked(EE[idx], Q[idx]) | world.blocked(Q[idx], PP[idx]))
                total += int(clear.sum())
        if stop_at_first and total:
            return total
    return total


def worlds(m):
    skip = m.verify_spec.get("skip", set())
    occ, player = [], []
    for s in m.solids:
        if s.owner is not None or s.id in skip:
            continue
        mats = [sd.material.upper() for sd in s.sides]
        if any(sd.disp for sd in s.sides):
            continue
        if any(_is_water(x) for x in mats):
            continue
        if all(x in NON_OCCLUDING for x in mats):
            if any(x == "TOOLS/TOOLSPLAYERCLIP" for x in mats):
                player.append(s)
            continue
        occ.append(s)
        player.append(s)
    return World(occ), World(player)


def verify_map(m, label, quiet=False, dead_eyes=True):
    """Returns (claims checked, claims failed); (0, 0) when the fixture declares no eyes."""
    spec = m.verify_spec
    if not (spec.get("floors") or spec.get("boxes") or spec.get("viewpoints")):
        print(f"{label}: not declared (no verify_* eye regions), skipped")
        return 0, 0
    occ, player = worlds(m)
    exact = declared_eyes(spec, player, "exact", dead_eyes)
    every = declared_eyes(spec, player, "all", dead_eyes)
    mirrors = [mr for mr, _why in spec.get("mirrors", [])]
    sides = m.all_sides()
    exp = m.expect_dict()
    bad = checked = 0
    for kind in ("must_visible", "must_hidden"):
        for e in exp[kind]:
            sd = sides[e["side"]]
            checked += 1
            if kind == "must_visible":
                P, nrm = face_points(sd, n=FACE_N_VISIBLE)
                k = seen(occ, exact, P, nrm, mirrors, True)
                ok = k > 0
            else:
                P, nrm = face_points(sd, step=FACE_STEP_HIDDEN)
                k = seen(occ, every, P, nrm, mirrors, False)
                ok = k == 0
            bad += not ok
            if not ok or not quiet:
                print(f"  {'ok ' if ok else 'BAD'} {kind:12s} side {sd.id:5d} clear pairs {k:7d}  {e['why'][:80]}")
    print(f"{label}: {checked} claims, {bad} failed ({len(exact)} exact eyes, {len(every)} with hidden-only regions)")
    return checked, bad


def _build(mod):
    m = vmfgen.Map(name=mod.NAME, description=mod.DESCRIPTION)
    mod.build(m)
    return m


def run_mutants():
    """Each variant must produce at least one failed claim."""
    mods = {mod.NAME[:3]: mod for mod in fixtures.discover()}
    results = []

    m = _build(mods["f01"])
    m.verify_viewpoint((672, 256, 96), "mutant: an eye inside the east void room")
    results.append(("f01 + eye in the void room", verify_map(m, "mutant f01", quiet=True)[1]))

    m = _build(mods["f11"])
    m.verify_viewpoint((256, 900, 64), "mutant: an eye inside the hall")
    results.append(("f11 + eye in the hall", verify_map(m, "mutant f11", quiet=True)[1]))

    m = _build(mods["f16"])
    m.verify_spec.pop("viewpoints", None)
    results.append(("f16 without its cubemap", verify_map(m, "mutant f16", quiet=True)[1]))

    m = _build(mods["f13"])
    m.verify_spec.pop("sky", None)
    results.append(("f13 without the sky mapping", verify_map(m, "mutant f13", quiet=True)[1]))

    m = _build(mods["f16"])
    m.verify_spec.pop("mirrors", None)
    results.append(("f16 without its mirror", verify_map(m, "mutant f16", quiet=True)[1]))

    m = _build(mods["f21"])
    results.append(("f21 without the dead-eye layer", verify_map(m, "mutant f21", quiet=True, dead_eyes=False)[1]))

    ok = True
    for name, failed in results:
        print(f"mutant {name}: {failed} failed claim(s) -> {'caught' if failed else 'NOT CAUGHT'}")
        ok &= failed > 0
    return ok


def _verify_one(name):
    """Worker: (name, failed, printed text)."""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        mod = fixtures.discover([name])[0]
        failed = verify_map(_build(mod), mod.NAME, quiet=_QUIET)[1]
    return name, failed, buf.getvalue()


_QUIET = False


def _init_worker(quiet):
    global _QUIET
    _QUIET = quiet


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--only", help="comma separated fixture ids/names")
    ap.add_argument("--mutants", action="store_true", help="also run the mutation checks")
    ap.add_argument("--quiet", action="store_true", help="print only failed claims and totals")
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 1, help="fixtures checked in parallel")
    args = ap.parse_args(argv)
    names = fixtures.fixture_names()
    if args.only:
        wanted = [w.strip() for w in args.only.split(",") if w.strip()]
        bad = [w for w in wanted if not any(n.startswith(w) for n in names)]
        if bad:
            print(f"usage error: no fixture matches {', '.join(bad)}", file=sys.stderr)
            return 2
        names = [n for n in names if any(n.startswith(w) for w in wanted)]
    failed = 0
    jobs = max(1, min(args.jobs, len(names)))
    if jobs == 1:
        _init_worker(args.quiet)
        results = [_verify_one(n) for n in names]
    else:
        with multiprocessing.Pool(jobs, initializer=_init_worker, initargs=(args.quiet,)) as pool:
            results = pool.map(_verify_one, names)
    for _name, bad, text in results:
        sys.stdout.write(text)
        failed += bad
    ok = failed == 0
    if args.mutants:
        ok &= run_mutants()
    print("verify_expectations:", "all claims hold" if ok else f"{failed} claim(s) failed")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
