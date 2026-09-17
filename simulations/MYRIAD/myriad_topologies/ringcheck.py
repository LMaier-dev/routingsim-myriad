#!/usr/bin/env python3
"""Validate a ring plan against the topology it labels.

MYRIAD GP reconstructs a ring by asking "which of my neighbours carry this ring's
label" (MyriadGp.cc otherRingNeighbor), so the plan is only walkable if every
member of ring R has EXACTLY TWO neighbours carrying R. A third one -- a chord,
i.e. two members of R joined by an edge that belongs to some other ring -- makes
"forward to the other one" ambiguous: the lap can fork onto a sub-cycle that
never reaches its originator and, since nothing else ends an in-ring lap, circles
until the hop cap kills it. Fewer than two means the ring isn't closed at all.

Works for native plans (ringpipe/export ship csv/<sub>/<stem>.txt beside
ringlabels/<sub>/<stem>.ini) and for generated ones (topo2rings.py): both are
just `**.NodeN.*.ringLabel = "a,b"` lines.

    ringcheck.py <topo.txt> <labels.ini>   # exits 1 if the plan is unwalkable
    ringcheck.py                           # self-check
"""
import collections
import re
import sys

LABEL_RE = re.compile(r'\*\*\.Node(\d+)\.\*\.ringLabel\s*=\s*"([^"]*)"')


def load(topo_path, ini_path):
    """(ring -> member set, node -> neighbour set) from a topology + label ini."""
    rings = collections.defaultdict(set)
    with open(ini_path) as f:
        for line in f:
            m = LABEL_RE.match(line.strip())
            if m:
                for r in m.group(2).split(","):
                    if r.strip():
                        rings[int(r)].add(int(m.group(1)))
    adj = collections.defaultdict(set)
    with open(topo_path) as f:
        for line in f:
            p = line.split()
            if len(p) >= 2 and p[0].isdigit() and p[1].isdigit():
                adj[int(p[0])].add(int(p[1]))
                adj[int(p[1])].add(int(p[0]))
    return rings, adj


def offenders(rings, adj):
    """ring -> [(node, ring-neighbour count)] for members that don't have exactly
    two. 2-member rings are exempt: GP echoes those back to the sender."""
    bad = {}
    for r, mem in rings.items():
        if len(mem) <= 2:
            continue
        off = [(v, len(adj[v] & mem)) for v in sorted(mem)
               if len(adj[v] & mem) != 2]
        if off:
            bad[r] = off
    return bad


def check(topo_path, ini_path):
    """Convenience wrapper: the offenders dict for this topology/plan pair."""
    return offenders(*load(topo_path, ini_path))


def check_echo_ids(topo_path, ini_path):
    """ring -> member count for every ring breaking the echo-id convention.

    A two-node (echo) ring must be numbered inside the reserved block and
    nothing else may be, because a router reads echo-ness off the id alone (see
    topo2rings.ECHO_ID_BASE and MyriadGp.ned echoRingIdMax). It cannot infer it:
    after a reboot a ring-neighbour that stayed down looks exactly like a ring
    that only ever had two members. A plan breaking this mis-simulates silently,
    so callers should treat a non-empty result as fatal.
    """
    from topo2rings import ECHO_ID_BASE   # lazy: keeps this script dependency-free
    rings, _ = load(topo_path, ini_path)
    return {r: len(m) for r, m in rings.items()
            if (len(m) == 2) != (r <= ECHO_ID_BASE)}


def _selfcheck():
    plain = {0: {1, 3}, 1: {0, 2}, 2: {1, 3}, 3: {0, 2}}      # a square
    square = {1: {0, 1, 2, 3}}
    assert not offenders(square, collections.defaultdict(set, plain))
    diag = {k: set(v) for k, v in plain.items()}               # + a diagonal
    diag[0].add(2)
    diag[2].add(0)
    assert offenders(square, collections.defaultdict(set, diag)) == {
        1: [(0, 3), (2, 3)]}
    open_ = {k: set(v) for k, v in plain.items()}              # - one edge
    open_[0].discard(3)
    open_[3].discard(0)
    assert offenders(square, collections.defaultdict(set, open_)) == {
        1: [(0, 1), (3, 1)]}
    print("OK")


if __name__ == "__main__":
    if len(sys.argv) == 1:
        _selfcheck()
    elif len(sys.argv) == 3:
        bad = check(sys.argv[1], sys.argv[2])
        for r, off in sorted(bad.items()):
            print(f"ring {r}: ring-neighbour counts {off} (want 2)")
        print(f"{len(bad)} unwalkable ring(s)")
        sys.exit(1 if bad else 0)
    else:
        sys.exit(__doc__)
