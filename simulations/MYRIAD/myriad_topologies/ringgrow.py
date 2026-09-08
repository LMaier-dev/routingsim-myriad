#!/usr/bin/env python3
"""Ring topologies for a TRANSITION sweep: fixed ring count, fixed node count,
one knob -- how many ring pairs meet.

ringpipe.py sweeps fan-out and solves for everything else. If the control plane
scales with TRANSITIONS (ring pairs sharing a node) rather than fan-out, that
machinery is unnecessary: transitions can just be added one at a time to a plan
whose ring count is already fixed.

    A rings of L nodes each, all filler        add a transition: two adjacent
                                               filler nodes of P take over two
      P: a-b-c-d-e-f-       Q: u-v-w-x-y-z-    of Q's, so P and Q now share them
                            ==>
      P: a-b-c-d-e-f-       Q: a-b-w-x-y-z-    (n drops by 2, lengths unchanged)

Every pair is merged at most once and each shared run is two CONSECUTIVE nodes
in both rings, so the plan is chord-free by construction -- the property
ringpipe needs a clique cover to get (see its docstring, and ringcheck.py).

WHAT IS AND IS NOT HELD FIXED
-----------------------------
Ring count A and node count n are exact. Everything else follows from the
identity sum(ring lengths) = n + sharing surplus, which for width-2 corridors is

    sum(ring lengths) = n + 2*tr - s  ->   mean ring length = that / A
    edges             = n + tr        ->   mean degree      = 2*(n + tr) / n
    fan-out           = 4*tr - 2*s    (every node is in at most 2 rings)

with s = the --spof transitions, which share one node instead of two.

So ring length and mean degree MOVE with tr, and no setting can stop them: a
transition is two nodes gaining a second ring, and that is what both quantities
count. They are the price of the x axis, not a confound to be solved away -- but
they are known in closed form, which is what ringpipe's --hold-len solver was
buying at much greater cost.

    python3 ringgrow.py --rings 150 --tr 400 --target-n 1000
    python3 ringgrow.py --sweep-tr 200 300 450 700 --rings 150 --target-n 1000
    python3 ringgrow.py --selfcheck

Files land under csv/, ringlabels/ and gallery/ named exactly as ringpipe's are
(_rl/_tr/_fan/_n), so `eval_scaling --dir ... --x-key tr` reads them unchanged.
"""
import argparse
import math
import os
import sys
from collections import Counter

import numpy as np
import networkx as nx

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import ringpipe as rp
from metrics import score


def _pair_order(family, pts, rng, seed):
    """Ring pairs in the order they should be merged.

    The site graph's spanning tree first, so the ring graph is connected at the
    lowest tr the sweep can ask for; then the rest of the site edges, then every
    remaining pair -- nearest first throughout, so a denser point looks like a
    denser city and not like a random graph.
    """
    sg = nx.Graph()
    sg.add_nodes_from(range(len(pts)))
    for u, v in rp._raw_site_edges(family, pts, seed):
        sg.add_edge(u, v, w=float(np.linalg.norm(pts[u] - pts[v])))
    if not nx.is_connected(sg):             # rgg/powerlaw can fragment
        sg = nx.compose(sg, nx.random_spanning_tree(nx.complete_graph(len(pts)),
                                                    seed=seed))
    dist = lambda e: float(np.linalg.norm(pts[e[0]] - pts[e[1]]))
    tree = {frozenset(e) for e in nx.minimum_spanning_tree(sg, weight="w").edges()}
    site = {frozenset(e) for e in sg.edges()} - tree
    rest = ({frozenset((u, v)) for u in range(len(pts)) for v in range(u + 1, len(pts))}
            - tree - site)
    out = []
    for group in (tree, site, rest):
        out += sorted((tuple(sorted(e)) for e in group), key=dist)
    return out


def _graph_of(rings_l):
    """The router graph a plan describes: every ring is a cycle over its nodes."""
    g = nx.Graph()
    for r in rings_l:
        for j in range(len(r)):
            g.add_edge(r[j], r[(j + 1) % len(r)])
    return g


def build(rings=150, seed=0, ring_size=8, tr=0, target_n=0, family="gabriel",
          spof=0.0, stages=None):
    """A rings, `tr` transitions between them, padded to exactly target_n nodes.

    stages: optional dict, filled with the intermediate states (pts, pairs, and
    the plan after each pass) for ringgrow_explain.py. Purely an observer -- the
    build is identical whether it is passed or not, which is what stops the
    picture from drifting away from the generator."""
    if tr < rings - 1:
        raise ValueError(f"--tr {tr} below {rings - 1}: fewer transitions than "
                         f"the ring graph needs to stay connected")
    rng = np.random.default_rng(seed)
    cap = rings * (rings - 1) // 2
    if tr > cap:
        raise ValueError(
            f"--tr {tr} above the {cap} pairs {rings} rings even have "
            f"(A*(A-1)/2). Raise --rings: A={math.ceil((1 + math.sqrt(1 + 8 * tr)) / 2)} "
            f"is the smallest that has {tr} pairs.")
    if ring_size < 3:
        raise ValueError(f"--ring-size {ring_size}: a ring needs 3 nodes")
    pts = rng.random((rings, 2))
    pairs = _pair_order(family, pts, rng, seed)[:tr]
    deg = Counter(r for p in pairs for r in p)

    # Two slots per incident transition plus two filler nodes kept at the tail,
    # so the wrap edge always joins filler to filler and can never close a shared
    # run into a loop (ringpipe's chord case).
    length = {r: max(ring_size, 2 * deg[r] + 2) for r in range(rings)}
    plan, nxt = {}, 0
    for r in range(rings):
        plan[r] = list(range(nxt, nxt + length[r]))
        nxt += length[r]

    snap = ((lambda k: stages.__setitem__(k, [list(plan[r]) for r in range(rings)]))
            if stages is not None else (lambda k: None))
    if stages is not None:
        stages.update(pts=pts, pairs=pairs)
    snap("laid")            # A disjoint rings, all filler

    slot, made = dict.fromkeys(range(rings), 0), []
    for a, b in pairs:
        # a's next two (still filler, since slots only ever advance) move into
        # b's next two positions; b's originals are dropped.
        i, j = slot[a], slot[b]
        plan[b][j:j + 2] = plan[a][i:i + 2]
        slot[a], slot[b] = i + 2, j + 2
        made.append((b, j))

    snap("merged")          # tr transitions made, each two nodes wide

    # A SPOF transition is one the two rings meet at a SINGLE node, so losing it
    # severs them. Narrow one by handing b back its own copy of the run's first
    # node: the pair then shares only the second. Splitting only ever removes
    # adjacency, so it cannot introduce a chord.
    for k in rng.permutation(len(made))[:int(round(spof * len(made)))]:
        b, j = made[k]
        plan[b][j] = nxt
        nxt += 1

    snap("narrowed")        # the --spof share squeezed to one shared node

    rings_l = [plan[r] for r in range(rings)]
    g = _graph_of(rings_l)
    idx = {v: k for k, v in enumerate(sorted(g.nodes()))}
    h = nx.Graph()
    h.add_edges_from((idx[u], idx[v]) for u, v in g.edges())
    rings_l = [[idx[v] for v in r] for r in rings_l]
    if target_n:
        if target_n < h.number_of_nodes():
            # Not a solver limitation: a width-2 transition is two nodes sitting
            # in two rings, so tr transitions need 2*tr - s memberships and a
            # node can supply at most one spare. Plus two filler per ring to keep
            # the wrap edge safe, the floor is 2*tr + 2*A + s exactly.
            raise ValueError(
                f"--target-n {target_n} below this point's floor "
                f"{h.number_of_nodes()} (A={rings}, tr={tr}, spof={spof:g}); "
                f"padding only adds. n = 2*tr + 2*A + spof is an identity, not a "
                f"knob: at n={target_n} and A={rings} the most transitions "
                f"possible is {max(0, (target_n - 2 * rings) // 2)}, and tr={tr} "
                f"needs n >= {h.number_of_nodes()}.")
        h, rings_l = rp._pad_uniform(h, rings_l, target_n, rng)
    # Positions for the gallery picture: each ring drawn as a circle round its
    # site, shared nodes keeping the first ring that placed them. After padding,
    # since filler added there would otherwise have none.
    for r, cyc in enumerate(rings_l):
        for j, v in enumerate(cyc):
            ang = 2 * math.pi * j / len(cyc)
            h.nodes[v].setdefault("pos", (
                pts[r][0] + 0.35 * math.cos(ang) / math.sqrt(rings),
                pts[r][1] + 0.35 * math.sin(ang) / math.sqrt(rings)))
    return h, rings_l, [tuple(p) for p in pts]


def export(rings, tr, seed=0, ring_size=8, target_n=0, family="gabriel",
           spof=0.0, sub=None, quiet=False):
    g, plan, rpos = build(rings, seed, ring_size, tr, target_n, family, spof)
    s = score(g, plan)
    stem = (f"ringgrow_{family}_A{rings}_spof{spof:g}_rl{rp._median_len(plan)}"
            f"_seed{seed}_tr{s['pairs']}_fan{s['fan']}_n{s['n']}")
    sub = sub or f"{family}_grow{target_n or s['n']}_A{rings}_spof{spof:g}"
    prov = (f"myriad_topologies/ringgrow.py family={family} rings(A)={rings} "
            f"tr_target={tr} spof_target={spof:g} target_n={target_n} seed={seed}")
    s, topo, ini, png = rp.write_out(g, plan, rpos, stem, sub, prov)
    rl = rp._median_len(plan)
    if not quiet:
        print(f"A={rings:4} tr={s['pairs']:6} n={s['n']:6} ringlen={rl:3} "
              f"fan={s['fan']:7} deg={s['avgdeg']:5.2f} "
              f"spof={s['spofpct']:5.1f}% diam={s['diam']:4}")
        print(f"  {topo}\n  {ini}\n  {png}")
    return s, rl, topo, ini, png


def floor_n(rings, tr, **kw):
    """Nodes this point cannot go below: the plan before any padding."""
    return build(rings=rings, tr=tr, **kw)[0].number_of_nodes()


def _floor_point(rings, t, kw):
    return floor_n(rings, t, **kw)


def _point(rings, t, target_n, sub, kw):
    return export(rings, t, target_n=target_n, sub=sub, quiet=True, **kw)


def sweep(trs, rings, target_n=0, jobs=1, **kw):
    """One topology per transition count, all at the same A and the same n.

    Points share nothing but the folder they write into, so --jobs runs them on
    separate processes; the parent prints in sweep order once they land."""
    trs = sorted(trs)
    floors = dict(zip(trs, rp._pmap(_floor_point, jobs,
                                    [(rings, t, kw) for t in trs])))
    if not target_n:
        target_n = max(floors.values())
        print(f"  --target-n not given, using the largest floor: {target_n}")
    sub = (f"{kw.get('family', 'gabriel')}_grow{target_n}_A{rings}"
           f"_spof{kw.get('spof', 0.0):g}")
    for t in trs:
        if floors[t] > target_n:
            ident = 2 * t + 2 * rings + round(kw.get("spof", 0.0) * t)
            raise ValueError(
                f"tr={t} floors at n={floors[t]}, above --target-n {target_n}; "
                f"raise it to {max(floors.values())}, drop the top tr, or lower "
                f"--ring-size: the identity floor here is 2*tr + 2*A + spof = "
                f"{ident}, and the remaining {floors[t] - ident} is --ring-size "
                f"padding rings that carry too few transitions to need it")
    for t, (s, rl, *_) in zip(trs, rp._pmap(
            _point, jobs, [(rings, t, target_n, sub, kw) for t in trs])):
        print(f"A={rings:4} tr={s['pairs']:6} n={s['n']:6} ringlen={rl:3} "
              f"fan={s['fan']:7} deg={s['avgdeg']:5.2f} "
              f"spof={s['spofpct']:5.1f}% diam={s['diam']:4}")


def sweep_n(counts, rings, tr, jobs=1, **kw):
    """One topology per NODE COUNT, at a fixed ring AND transition count.

    fan-out = 4*tr - 2*s and edges = n + tr are identities (see the docstring),
    so with A and tr pinned the ring GRAPH is the same at every point and n only
    moves the ring length, (n + 2*tr - s)/A. That is the axis per-router state
    scales on: LS holds a row per node, PV holds its rings, and nothing else
    about the topology is moving underneath the curve.

    The floor is 2*tr + 2*A + s, the same for every point, so one check does."""
    counts = sorted(counts)
    floor = floor_n(rings, tr, **kw)
    if counts[0] < floor:
        raise ValueError(
            f"n={counts[0]} is below the floor {floor} this A/tr forces "
            f"(2*tr + 2*A + spof); padding only adds. Drop the low counts, or "
            f"lower --tr/--rings.")
    sub = (f"{kw.get('family', 'gabriel')}_nsweep_A{rings}_tr{tr}"
           f"_spof{kw.get('spof', 0.0):g}")
    for n, (s, rl, *_) in zip(counts, rp._pmap(
            _point, jobs, [(rings, tr, n, sub, kw) for n in counts])):
        print(f"A={rings:4} tr={s['pairs']:6} n={s['n']:6} ringlen={rl:3} "
              f"fan={s['fan']:7} deg={s['avgdeg']:5.2f} "
              f"spof={s['spofpct']:5.1f}% diam={s['diam']:4}")


def _selfcheck():
    for fam in ("gabriel", "delaunay"):
        for tr in (60, 150, 300):
            for sp in (0.0, 0.4):
                g, plan, _ = build(rings=50, seed=1, tr=tr, target_n=1000,
                                   family=fam, spof=sp)
                case = (fam, tr, sp)
                s = score(g, plan)                  # validates walkability
                assert nx.is_connected(g), case
                assert not rp.ringcheck.offenders(
                    {i: set(r) for i, r in enumerate(plan)},
                    {v: set(g[v]) for v in g}), case
                assert s["n"] == 1000 and s["rings"] == 50, (case, s["n"])
                assert s["comp"] == 1, (case, s["comp"])       # never partitions
                # the knob lands exactly, and everything it drags with it follows
                # the closed forms in the docstring
                assert s["pairs"] == tr, (case, s["pairs"])
                assert abs(s["spofpct"] / 100 - sp) < 0.01, (case, s["spofpct"])
                assert s["fan"] == 4 * tr - 2 * s["spof"], (case, s["fan"])
                assert s["m"] == 1000 + tr, (case, s["m"])
                assert (sum(len(r) for r in plan)
                        == 1000 + 2 * tr - s["spof"]), case
    # --sweep-n's premise: at a fixed A and tr the ring graph does not move with
    # n, only the ring length does.
    base = None
    for n in (600, 1200, 2400):
        g, plan, _ = build(rings=50, seed=1, tr=150, target_n=n,
                           family="delaunay", spof=0.3)
        s = score(g, plan)
        assert s["n"] == n and s["pairs"] == 150, (n, s["n"], s["pairs"])
        assert (s["fan"], s["spof"]) == (base or (s["fan"], s["spof"])), (n, s["fan"])
        base = (s["fan"], s["spof"])
        assert rp._median_len(plan) == (n + 2 * 150 - s["spof"]) // 50, n
    print("ringgrow selfcheck ok")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--family", choices=rp.SITE_FAMILIES, default="gabriel",
                    help="site graph deciding WHICH pairs meet first; the count "
                         "is --tr regardless")
    ap.add_argument("--rings", type=int, default=150, help="A: ring count, exact")
    ap.add_argument("--tr", type=int, default=0,
                    help="transitions (ring pairs sharing nodes), exact; at least "
                         "A-1, at most A*(A-1)/2")
    ap.add_argument("--target-n", type=int, default=0,
                    help="exact node count (0 = this point's floor, or the "
                         "largest floor in a sweep)")
    ap.add_argument("--ring-size", type=int, default=8,
                    help="minimum ring length; a ring carrying many transitions "
                         "gets a longer one, and padding equalises afterwards")
    ap.add_argument("--spof", type=float, default=0.0, metavar="FRAC",
                    help="fraction of transitions narrowed to a SINGLE shared "
                         "node, so losing that node severs the pair; exact")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--sweep-tr", type=int, nargs="+", metavar="TR",
                    help="one topology per transition count, all at the same A "
                         "and --target-n")
    ap.add_argument("--sweep-n", type=int, nargs="+", metavar="N",
                    help="one topology per NODE count, all at the same A and "
                         "--tr. fan = 4*tr - 2*spof, so fan and the ring graph "
                         "stay put and only the ring length follows n -- the "
                         "axis per-router state and link load scale on")
    ap.add_argument("--jobs", type=int, default=1, metavar="N",
                    help="build a sweep's points on N processes (default 1)")
    ap.add_argument("--selfcheck", action="store_true")
    a = ap.parse_args()
    if a.selfcheck:
        return _selfcheck()
    kw = dict(family=a.family, seed=a.seed, ring_size=a.ring_size, spof=a.spof)
    if a.sweep_n:
        if not a.tr:
            ap.error("--sweep-n needs --tr: it is what holds fan-out fixed")
        return sweep_n(a.sweep_n, a.rings, a.tr, a.jobs, **kw)
    if a.sweep_tr:
        return sweep(a.sweep_tr, a.rings, a.target_n, a.jobs, **kw)
    if not a.tr:
        ap.error("--tr, --sweep-tr or --sweep-n")
    export(a.rings, a.tr, target_n=a.target_n, **kw)


if __name__ == "__main__":
    main()
