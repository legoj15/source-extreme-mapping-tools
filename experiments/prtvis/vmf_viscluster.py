"""Report func_viscluster state in a VMF, or write a copy with them toggled.

Usage:
  python vmf_viscluster.py report <in.vmf>
  python vmf_viscluster.py strip  <in.vmf> <out.vmf>   # drop every func_viscluster

Hammer stores a hidden object inside a `hidden { ... }` block, which VBSP skips.
"""
import sys
from collections import Counter


def blocks(lines):
    """Yield (start, end, name, depth, hidden_ancestor) for every block."""
    stack = []  # (name, start_line)
    pending = None
    for i, raw in enumerate(lines):
        s = raw.strip()
        if s == "{":
            stack.append((pending, i - 1))
            pending = None
        elif s == "}":
            name, start = stack.pop()
            hidden = any(n == "hidden" for n, _ in stack)
            yield start, i, name, len(stack), hidden
        elif s and not s.startswith('"'):
            pending = s
    assert not stack


def entity_info(lines, start, end):
    cls = None
    depth = 0
    solids = hidden_solids = 0
    in_hidden = []
    pending = None
    for raw in lines[start:end + 1]:
        s = raw.strip()
        if s == "{":
            depth += 1
            in_hidden.append(pending == "hidden" or (in_hidden[-1] if in_hidden else False))
            if pending == "solid":
                solids += 1
                if in_hidden[-1]:
                    hidden_solids += 1
            pending = None
        elif s == "}":
            depth -= 1
            in_hidden.pop()
        elif s.startswith('"classname"') and depth == 1:
            cls = s.split('"')[3]
        elif s and not s.startswith('"'):
            pending = s
    return cls, solids, hidden_solids


def main(argv):
    mode, path = argv[1], argv[2]
    with open(path, "r", encoding="utf-8", errors="surrogateescape", newline="") as f:
        lines = f.readlines()
    ents = [(s, e, d, h) for s, e, n, d, h in blocks(lines) if n == "entity"]
    state = Counter()
    drop = []
    for s, e, d, hidden_anc in ents:
        cls, solids, hidden_solids = entity_info(lines, s, e)
        if cls != "func_viscluster":
            continue
        if hidden_anc:
            state["hidden (whole entity)"] += 1
        elif solids and hidden_solids == solids:
            state["hidden (all solids)"] += 1
        elif hidden_solids:
            state["partly hidden"] += 1
        else:
            state["visible"] += 1
        drop.append((s, e))
    if mode == "report":
        print(path)
        for k, v in sorted(state.items()):
            print(f"  func_viscluster {k}: {v}")
        if not state:
            print("  no func_viscluster")
        return
    out = argv[3]
    kill = set()
    for s, e in drop:
        kill.update(range(s, e + 1))
    with open(out, "w", encoding="utf-8", errors="surrogateescape", newline="") as f:
        f.writelines(l for i, l in enumerate(lines) if i not in kill)
    print(f"wrote {out}: dropped {len(drop)} func_viscluster entities")


if __name__ == "__main__":
    main(sys.argv)
