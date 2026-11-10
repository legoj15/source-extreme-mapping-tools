"""Shape of a Source .prt portal graph: clusters, portals per cluster, winding sizes.

Usage: python prt_stats.py <file.prt>
"""
import re
import sys
from collections import Counter


def load(path):
    with open(path, "r") as f:
        magic = f.readline().strip()
        nclusters = int(f.readline())
        nportals = int(f.readline())
        portals = []
        for line in f:
            line = line.strip()
            if not line:
                continue
            head, _, rest = line.partition("(")
            n, c0, c1 = (int(x) for x in head.split())
            pts = [tuple(float(v) for v in m.split())
                   for m in re.findall(r"\(([^)]*)\)", "(" + rest)]
            assert len(pts) == n, (len(pts), n)
            portals.append((c0, c1, pts))
    assert magic == "PRT1" and len(portals) == nportals
    return nclusters, portals


def area(pts):
    ax = ay = az = 0.0
    x0, y0, z0 = pts[0]
    for i in range(1, len(pts) - 1):
        ux, uy, uz = pts[i][0] - x0, pts[i][1] - y0, pts[i][2] - z0
        vx, vy, vz = pts[i + 1][0] - x0, pts[i + 1][1] - y0, pts[i + 1][2] - z0
        ax += uy * vz - uz * vy
        ay += uz * vx - ux * vz
        az += ux * vy - uy * vx
    return 0.5 * (ax * ax + ay * ay + az * az) ** 0.5


def pct(sorted_vals, q):
    return sorted_vals[min(len(sorted_vals) - 1, int(q * len(sorted_vals)))]


def main(path):
    nclusters, portals = load(path)
    deg = Counter()
    for c0, c1, _ in portals:
        deg[c0] += 1
        deg[c1] += 1
    degs = sorted(deg.get(c, 0) for c in range(nclusters))
    npts = sorted(len(p[2]) for p in portals)
    areas = sorted(area(p[2]) for p in portals)
    print(f"{path}")
    print(f"clusters {nclusters}  portals {len(portals)}  directed {2 * len(portals)}")
    print(f"portals per cluster: min {degs[0]} median {pct(degs, .5)} p90 {pct(degs, .9)} "
          f"p99 {pct(degs, .99)} max {degs[-1]}  mean {sum(degs) / len(degs):.1f}")
    top = sorted(deg.items(), key=lambda kv: -kv[1])[:12]
    print("largest clusters (cluster:portals): " + ", ".join(f"{c}:{d}" for c, d in top))
    big = sum(d for d in degs if d >= 100)
    print(f"portal ends in clusters with >=100 portals: {big} of {sum(degs)} "
          f"({100.0 * big / sum(degs):.1f}%)")
    # second-level work items of the MT flow: sum over directed portals of deg(leaf) - ish
    print(f"sum of degree^2 (passages if every portal pair in a cluster is one): "
          f"{sum(d * d for d in degs):,}")
    print(f"winding points: median {pct(npts, .5)} p90 {pct(npts, .9)} max {npts[-1]}")
    print(f"portal area (sq units): median {pct(areas, .5):.0f} p90 {pct(areas, .9):.0f} "
          f"max {areas[-1]:.0f}")
    xs = [v[0] for p in portals for v in p[2]]
    ys = [v[1] for p in portals for v in p[2]]
    zs = [v[2] for p in portals for v in p[2]]
    print(f"bounds x {min(xs):.0f}..{max(xs):.0f}  y {min(ys):.0f}..{max(ys):.0f}  "
          f"z {min(zs):.0f}..{max(zs):.0f}")


if __name__ == "__main__":
    main(sys.argv[1])
