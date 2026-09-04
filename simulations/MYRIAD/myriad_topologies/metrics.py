#!/usr/bin/env python3
"""Score a topology + ring assignment on the quantities MyriadGP's control plane
actually turns on. Import `score`; run this file for the family sweep.

    python3 metrics.py --sizes 100 250 500 --families ringmesh gabriel rgg
"""
import argparse
import math
import os
import sys
from collections import Counter, defaultdict

import networkx as nx

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from topo_lib import FAMILIES


def rings_to_membership(g, rings, echo=True):
    """node -> set of ring indices, adding 2-node echo rings for any edge whose
    endpoints share no ring (else that edge carries no ring and is unroutable)."""
    member = defaultdict(set)
    for i, c in enumerate(rings):
        for n in c:
            member[n].add(i)
    nxt = len(rings)
    n_echo = 0
    if echo:
        for u, v in g.edges():
            if member[u].isdisjoint(member[v]):
                member[u].add(nxt); member[v].add(nxt)
                nxt += 1; n_echo += 1
    for n in g.nodes():
        member.setdefault(n, set())
    return member, n_echo


def validate_rings(g, rings):
    """Reasons `rings` is not a usable ring set, or [] if it is.

    A GP ring is walked hop by hop, so consecutive members must be ADJACENT and
    a ring must be a SIMPLE cycle. Fan-out/spof are computed from membership
    sets, which stay happily finite for a bogus ring -- a spof-merging post-pass
    scored a 20% fan-out win here purely by splicing rings into walks that no
    packet can traverse. Cheap to check, so nothing is scored unchecked.
    """
    bad = []
    for i, c in enumerate(rings):
        if len(c) < 3:
            bad.append(f"ring {i}: {len(c)} nodes, not a cycle")
        if len(set(c)) != len(c):
            bad.append(f"ring {i}: revisits a node (not simple)")
        miss = [(c[j], c[(j + 1) % len(c)]) for j in range(len(c))
                if not g.has_edge(c[j], c[(j + 1) % len(c)])]
        if miss:
            bad.append(f"ring {i}: {len(miss)} consecutive pair(s) not edges, e.g. {miss[0]}")
    return bad


def score(g, rings, echo=True, check=True):
    """All metrics in one pass. fan-out is the headline; spof% and ring-graph
    connectivity are the constraints it must not buy itself out of."""
    if check:
        bad = validate_rings(g, rings)
        if bad:
            raise ValueError(f"invalid ring set ({len(bad)} problems): " + "; ".join(bad[:3]))
    member, n_echo = rings_to_membership(g, rings, echo)
    n = g.number_of_nodes()
    m = g.number_of_edges()
    k = [len(member[v]) for v in g.nodes()]
    fan = sum(x * (x - 1) for x in k)

    bridges = Counter()
    for v in g.nodes():
        rs = sorted(member[v])
        for i in range(len(rs)):
            for j in range(i + 1, len(rs)):
                bridges[(rs[i], rs[j])] += 1
    n_spof = sum(1 for c in bridges.values() if c == 1)

    rg = nx.Graph()
    rg.add_nodes_from(range(len(rings) + n_echo))
    rg.add_edges_from(bridges)
    comp = nx.number_connected_components(rg) if rg.number_of_nodes() else 0

    sizes = [len(c) for c in rings] + [2] * n_echo
    ringless = sum(1 for v in g.nodes() if not member[v])

    # Robustness: the network must survive any single site loss, and the ring
    # graph must stay connected too (else routing partitions even if IP doesn't).
    biconn = nx.is_biconnected(g) if n > 2 else False
    degs = [d for _, d in g.degree()]

    return dict(
        n=n, m=m, avgdeg=2 * m / n if n else 0, maxdeg=max(degs) if degs else 0,
        mindeg=min(degs) if degs else 0,
        rings=len(rings), echo=n_echo,
        fan=fan, fan_n=fan / n if n else 0,
        maxk=max(k) if k else 0, meank=sum(k) / n if n else 0,
        spof=n_spof, pairs=len(bridges),
        spofpct=100.0 * n_spof / len(bridges) if bridges else 0.0,
        comp=comp, ringless=ringless, biconn=biconn,
        maxring=max(sizes) if sizes else 0,
        meanring=sum(sizes) / len(sizes) if sizes else 0,
        diam=nx.diameter(g) if n <= 3000 and nx.is_connected(g) else -1,
        apl=nx.average_shortest_path_length(g) if n <= 1200 and nx.is_connected(g) else -1,
    )


def cover_rings(g, cap=10, basis="tree"):
    """Mine rings when the family has no native plan: cycle_basis ("tree",
    cheap) or minimum_cycle_basis ("min", small/slow). Rings larger than `cap`
    are dropped (cap=0 keeps all); their edges fall back to echo rings in
    rings_to_membership. cycle_basis yields walkable simple cycles."""
    cycles = nx.minimum_cycle_basis(g) if basis == "min" else nx.cycle_basis(g)
    return [c for c in cycles if not cap or len(c) <= cap]


HDR = (f"{'family':11} {'n':>5} {'deg':>5} {'rings':>5} {'echo':>4} {'fan':>7} "
       f"{'fan/n':>6} {'maxk':>4} {'spof%':>6} {'cmp':>3} {'2con':>4} "
       f"{'ring':>5} {'diam':>5} {'apl':>6}")


def row(name, s):
    return (f"{name:11} {s['n']:5} {s['avgdeg']:5.2f} {s['rings']:5} {s['echo']:4} "
            f"{s['fan']:7} {s['fan_n']:6.2f} {s['maxk']:4} {s['spofpct']:6.1f} "
            f"{s['comp']:3} {'Y' if s['biconn'] else 'n':>4} "
            f"{s['meanring']:5.1f} {s['diam']:5} {s['apl']:6.2f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sizes", type=int, nargs="+", default=[100, 250, 500])
    ap.add_argument("--families", nargs="+", default=list(FAMILIES))
    ap.add_argument("--cap", type=int, default=10)
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()

    print(HDR); print("-" * len(HDR))
    for n in a.sizes:
        for fam in a.families:
            try:
                g, rings = FAMILIES[fam](n, seed=a.seed)
                native = rings is not None
                if not native:
                    rings = cover_rings(g, a.cap)
                s = score(g, rings)
                print(row(f"{fam}{'*' if native else ''}", s))
            except Exception as e:                      # keep the sweep going
                print(f"{fam:11} {n:5}  FAILED: {type(e).__name__}: {e}")
        print()
    print("* = native ring plan (built from rings); others mined by cover_rings (cycle_basis)")


if __name__ == "__main__":
    main()

