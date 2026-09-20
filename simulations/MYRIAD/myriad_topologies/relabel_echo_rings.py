#!/usr/bin/env python3
"""Move echo rings into the reserved id range, in place, in existing ring plans.

A router cannot tell a genuine two-node (echo) ring from a ring-neighbour that
stayed down while it was rebooting: both look like "one neighbour claims this
ring". Guessing wrong closes a lap that never walked the ring, which invents a
member_count of 2 and mis-prices every chain across that ring. Encoding the
answer in the id removes the guess — see MyriadPv.ned echoRingIdMax.

Echo rings are identifiable from the labels alone (exactly two members), so this
renumbers them without touching the topology graphs: the .txt files stay
byte-identical and only ringlabels/*.ini change. Non-echo ids are left alone;
they descend from RING_ID_BASE = 1,000,000 and stay far above the boundary.

    relabel_echo_rings.py <ini-or-dir> [more…] [--apply]

Idempotent: a plan already in the convention is reported as ok and left alone.
"""
import argparse
import re
import sys
from collections import Counter, defaultdict
from pathlib import Path

from topo2rings import ECHO_ID_BASE, RING_ID_BASE

LABEL = re.compile(r'^(\*\*\.Node\d+\.\*\.ringLabel = ")([^"]*)(")\s*$')


def plan_of(path):
    """node -> [ring ids], preserving file order."""
    out = {}
    for line in path.read_text().splitlines():
        m = LABEL.match(line)
        if m:
            out[line] = [int(r) for r in m.group(2).split(",") if r]
    return out


def violations(members):
    """Rings breaking the convention: a two-member ring outside the reserved
    range, or anything else inside it."""
    return [(r, c) for r, c in members.items() if (c == 2) != (r <= ECHO_ID_BASE)]


def remap_for(members):
    """Old id -> new id for every echo ring, lowest original id first.

    Only two-member rings move. Their new ids descend from ECHO_ID_BASE, so
    `id <= ECHO_ID_BASE` is exactly "this is an echo ring" to a router.
    """
    echo = sorted(r for r, c in members.items() if c == 2)
    big = [r for r, c in members.items() if c != 2]
    if any(r <= ECHO_ID_BASE for r in big):
        sys.exit(f"non-echo ring id already inside the reserved range "
                 f"(<= {ECHO_ID_BASE}); the ranges would overlap")
    if len(echo) > ECHO_ID_BASE:
        sys.exit(f"{len(echo)} echo rings exceed the reserved range")
    return {r: ECHO_ID_BASE - i for i, r in enumerate(echo)}


def relabel(path, apply):
    text = path.read_text()
    members = Counter()
    for _, ids in plan_of(path).items():
        members.update(ids)
    if not members:
        return None
    # Idempotent: a plan already in the convention keeps its ids, rather than
    # being reshuffled into a different but equally valid numbering.
    if not violations(members):
        return (0, len(members))
    remap = remap_for(members)
    moved = {o: n for o, n in remap.items() if o != n}
    if not moved:
        return (0, len(members))

    out = []
    for line in text.splitlines():
        m = LABEL.match(line)
        if not m:
            out.append(line)
            continue
        ids = [remap.get(int(r), int(r)) for r in m.group(2).split(",") if r]
        out.append(f"{m.group(1)}{','.join(str(i) for i in ids)}{m.group(3)}")
    if apply:
        path.write_text("\n".join(out) + "\n")
    return (len(moved), len(members))


def verify(path):
    """Every two-member ring inside the range, nothing else in it."""
    members = Counter()
    for _, ids in plan_of(path).items():
        members.update(ids)
    return violations(members)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="+")
    ap.add_argument("--apply", action="store_true",
                    help="write the files (default: report only)")
    args = ap.parse_args()

    inis = []
    for p in map(Path, args.paths):
        inis += sorted(p.rglob("*.ini")) if p.is_dir() else [p]

    moved_total = clean = 0
    for ini in inis:
        res = relabel(ini, args.apply)
        if res is None:
            continue
        moved, total = res
        if moved:
            moved_total += 1
            print(f"  {ini.name}: {moved} of {total} rings -> reserved range")
        else:
            clean += 1
        if args.apply:
            bad = verify(ini)
            if bad:
                sys.exit(f"{ini}: convention violated after relabel: {bad[:3]}")
    print(f"{'relabelled' if args.apply else 'would relabel'} {moved_total} "
          f"plan(s); {clean} already in the convention; "
          f"boundary {ECHO_ID_BASE} (non-echo ids descend from {RING_ID_BASE})")


if __name__ == "__main__":
    main()
