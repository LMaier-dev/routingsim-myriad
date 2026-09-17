#!/usr/bin/env python3
"""Generate ring topologies for the MYRIAD evals: a ring plan plus the graph
that realises it.

WHAT A PLAN IS
--------------
A site graph (Delaunay/Gabriel/RGG/BA over random points) gives A sites; each
site becomes one ring. Rings meet at INTERFACES -- a run of nodes carried by
every ring in that interface:

      ring P:  p0 - p1 - [ X - Y ] - p2 - p3 -+        interface {X,Y}
               |                              |        is IN both rings,
               +------------------------------+        contiguous in each

      ring Q:  q0 - [ X - Y ] - q1 - q2 - q3 -+
               |                              |
               +------------------------------+

Interfaces are edge-disjoint CLIQUES of sites, so any two rings meet exactly
once. That is not cosmetic: MyriadGP infers ring adjacency from shared
ringLabels, so if P and Q met in two separate places, the edge bounding one
place would join two P-members that are not adjacent in P -- a CHORD, and GP's
lap forks onto a sub-cycle that never closes (see ringcheck.py). Chord-freedom
here is by construction, not by repair.

WHAT ENDS UP ON THE EVAL'S X AXIS
---------------------------------
fan-out   sum over nodes of k(k-1), k = rings the node is in. An interface of m
          rings and width w contributes w * m(m-1). --share raises m
          (quadratic), --width-max raises w (linear).
n         node count. Held exactly by padding rings with FILLER -- nodes in one
          ring only, which carry no ring pairs and so cost no fan-out.
length    median ring length -- MyriadGP's lap length, so it drives message
          counts directly.

These three are tied by an identity, and only TWO can be pinned at once:

      sum(ring lengths) = n + sharing surplus
                              ^^^^^^^^^^^^^^^ a node in k rings is counted k
                                              times on the left, once on the right

Raising --share IS raising that surplus, so at fixed n and fixed ring count the
rings must grow (13 -> 21 over a 0..1 sweep). --hold-len spends the ring count
to stop that.

    python3 ringpipe.py --family gabriel --rings 100 --spof 0.1 --share 0.5
    python3 ringpipe.py --sweep-share 0 0.5 1 --rings 150 --target-n 1000
    python3 ringpipe.py --sweep-share 0 0.5 1 --rings 150 --target-n 1000 --hold-len
    python3 ringpipe.py --selfcheck

For a TRANSITION sweep at a fixed ring and node count, see ringgrow.py: none of
the solving below is needed once every interface is a plain two-node corridor.

Each run writes three files under csv/, ringlabels/ and gallery/, named with the
knobs AND the achieved figures (_rl<median length>, _tr<transitions>,
_fan<fan-out>, _n<nodes>) so eval_scaling.py can read the axis straight off the
file names.
"""
import argparse
import concurrent.futures as cf
import math
import os
import sys

import numpy as np
import networkx as nx

HERE = os.path.dirname(os.path.abspath(__file__))
MYRIAD = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, MYRIAD)

from topo_lib import _check_ring_size, _cap_degree, _delaunay_edges
from metrics import score, rings_to_membership
import export as exp
import ringcheck
import topo2rings as t2r

SITE_FAMILIES = ("gabriel", "delaunay", "powerlaw", "rgg")
WIDTH_MAX = 2          # nodes per interface; 1 makes every transition a SPOF
GALLERY = os.path.join(HERE, "gallery")
CSV_DIR = os.path.join(HERE, "csv")            # topology .txt files
RINGLABELS_DIR = os.path.join(HERE, "ringlabels")   # ring-label .ini files


def _raw_site_edges(family, pts, seed):
    if family == "gabriel":
        from scipy.spatial import cKDTree
        tree = cKDTree(pts)
        e = set()
        for u, v in _delaunay_edges(pts):
            mid = (pts[u] + pts[v]) / 2.0
            r = float(np.linalg.norm(pts[u] - pts[v])) / 2.0
            if not [w for w in tree.query_ball_point(mid, r - 1e-9) if w not in (u, v)]:
                e.add((u, v))
        return e
    if family == "delaunay":
        return _delaunay_edges(pts)
    if family == "powerlaw":
        # Positions are for layout/nonplanar distance only -- BA's edge rule
        # is degree-preferential, not geometric.
        ba = nx.barabasi_albert_graph(len(pts), 2, seed=seed)
        return {tuple(sorted(e)) for e in ba.edges()}
    if family == "rgg":
        # Radius chosen for ~4.5 avg degree, as topo_lib.rgg does.
        from scipy.spatial import cKDTree
        r = math.sqrt(4.5 / (math.pi * len(pts)))
        return cKDTree(pts).query_pairs(r)
    raise ValueError(f"unknown family {family!r}; choices: {SITE_FAMILIES}")


def _hard_cap(sg, max_deg):
    """_cap_degree's connectivity repair can push a node above max_deg
    ("connectivity beats the degree cap"). Harmless for Gabriel/Delaunay, but
    powerlaw's hubs trigger it. Trim the longest excess edges, skipping any whose
    removal would disconnect."""
    for v in sorted(sg.nodes(), key=lambda v: -sg.degree(v)):
        for u, w, d in sorted(sg.edges(v, data=True), key=lambda e: -e[2]["w"]):
            if sg.degree(v) <= max_deg:
                break
            sg.remove_edge(u, w)
            if not nx.is_connected(sg):
                sg.add_edge(u, w, **d)
    return sg


def _clique_cover_geo(sg, pts, rng, m_max):
    """Edge-disjoint cliques of sites, each of which becomes one interface.

    Cliques are grown from geometric neighbourhoods and are NOT limited to site
    edges. That matters for fan-out: merely folding together sites the site graph
    already joined is fan-neutral -- a triangle's three 2-ring corridors (3 x 2
    nodes at k=2) and one 3-ring interface (2 nodes at k=3) are both fan 12 --
    which would pin fan at 2*width*|E| whatever --share said. Letting an
    interface CREATE ring adjacency is what lifts it: m rings meeting at one
    interface put every one of its nodes in m rings, so fan grows as m(m-1) per
    node while the node count grows only with the width.

        m=2 (a corridor)        m=4 (a hub)
        P --- Q                 P   Q       one interface serves all
                                  +-+       C(4,2) = 6 ring pairs, and
                                  +-+       every node in it is in 4
                                R   S       rings -> fan 4*3 per node

    `met` is the whole invariant: a pair already sharing an interface is never
    put in another, so no two rings can meet twice. The site MST is covered
    first, so the ring graph comes out connected whatever the greedy does after.
    """
    met = set()
    cliques = []

    def grow(seed):
        c = list(seed)
        if any(frozenset((x, y)) in met for i, x in enumerate(c) for y in c[i + 1:]):
            return None
        # Nearest sites first: an interface is a physical location (a street
        # cabinet, an exchange), so the rings meeting there should be
        # neighbours rather than scattered across the plane.
        near = sorted(sg.nodes(), key=lambda w: min(
            float(np.linalg.norm(pts[w] - pts[x])) for x in c))
        for w in near:
            if len(c) >= m_max:
                break
            if w in c or any(frozenset((w, x)) in met for x in c):
                continue
            c.append(w)
        met.update(frozenset((x, y)) for i, x in enumerate(c) for y in c[i + 1:])
        return c

    for u, v in nx.minimum_spanning_tree(sg, weight="w").edges():
        c = grow((u, v))
        if c:
            cliques.append(c)
    rest = [tuple(sorted(e)) for e in sg.edges()]
    rng.shuffle(rest)
    for u, v in rest:
        c = grow((u, v))
        if c:
            cliques.append(c)
    return cliques


def _plan_cliques(sg, pts, rng, spof, share, ring_size, width_max, max_meet=0):
    """Lay the clique cover out as rings: the plan, and the graph realising it.

    Each ring is its interfaces written one after another around the loop, then
    padded with filler to --ring-size:

        ring = [ iface A ][ iface B ][ filler ... ]
                 each run unbroken          the loop wraps back to the start

    Contiguity is what keeps the plan chord-free: a pair of rings shares exactly
    one interface, and that run is unbroken in both, so no edge of one ring joins
    two members of the other non-adjacently.

    --share sets how many rings meet at an interface (2 .. max_meet), which is
    the fan-out lever; --width-max sets how many nodes each interface spans,
    which is the second one.
    """
    if not max_meet:
        max_meet = max(4, round(math.sqrt(sg.number_of_nodes())))
    m_max = 2 + int(round(share * (max_meet - 2)))
    cliques = _clique_cover_geo(sg, pts, rng, m_max)
    # Width is the second fan lever, linear where share is quadratic: a wider
    # interface adds more nodes at the same k, a bigger clique raises k itself.
    # Drawn per interface from [2, width_max] rather than fixed, so a topology
    # carries a MIX of transition widths like a real one does -- which means fan
    # tracks the MEAN width, ~(2+width_max)/2, not the cap.
    wide = max(1, width_max)
    n_spof = int(round(spof * len(cliques)))
    # Width 1 = a SPOF: the two rings meet at a single node, so losing it severs
    # the transition. Spent on the SMALLEST cliques first -- narrowing a big
    # interface would strand all C(m,2) of its pairs at once, which is not what
    # one failed link means.
    idx = sorted(range(len(cliques)), key=lambda k: (len(cliques[k]), k))
    width = {k: 1 if r < n_spof else (2 if wide < 2 else int(rng.integers(2, wide + 1)))
             for r, k in enumerate(idx)}
    if wide < 2:                          # --width-max 1: every transition a SPOF
        width = {k: 1 for k in width}

    at = {}                              # site -> its interfaces, in ring order
    for k, c in enumerate(cliques):
        for i in c:
            at.setdefault(i, []).append(k)
    nxt = 0
    nodes_of = {}                        # clique -> its shared nodes
    for k, c in enumerate(cliques):
        nodes_of[k] = list(range(nxt, nxt + width[k]))
        nxt += width[k]

    g = nx.Graph()
    rings, coord, site = [], {}, []
    n_sites = len(pts)
    for i in sg.nodes():
        mine = at.get(i, [])
        cyc = []
        for k in mine:
            cyc += nodes_of[k]           # the interface, contiguous by placement
        # A ring built from ONE interface closes that run into a loop, and the
        # wrap edge then joins the run's two ends:
        #     [ X - Y - Z ]  ->  X - Y - Z - X    <- the X-Z edge exists here
        # The other rings carrying {X,Y,Z} hold it as a straight run, so for them
        # X and Z are adjacent in the graph but not in their cycle: a chord. One
        # filler node breaks the wrap. With two or more interfaces the wrap joins
        # nodes of DIFFERENT cliques, which share only this ring, so it is safe.
        if len(mine) < 2:
            cyc.append(nxt)
            nxt += 1
        while len(cyc) < ring_size:      # filler: single-ring nodes pad the loop
            cyc.append(nxt)
            nxt += 1
        if len(cyc) < 3:
            continue
        rings.append(cyc)
        site.append(i)
        for j in range(len(cyc)):
            g.add_edge(cyc[j], cyc[(j + 1) % len(cyc)])
            ang = 2 * math.pi * j / len(cyc)
            coord.setdefault(cyc[j],
                             (pts[i][0] + 0.35 * math.cos(ang) / math.sqrt(n_sites),
                              pts[i][1] + 0.35 * math.sin(ang) / math.sqrt(n_sites)))
    idx = {v: k for k, v in enumerate(sorted(g.nodes()))}
    h = nx.Graph()
    h.add_edges_from((idx[u], idx[v]) for u, v in g.edges())
    for v in g.nodes():
        h.nodes[idx[v]]["pos"] = coord.get(v, (0.0, 0.0))
    return h, [[idx[v] for v in r] for r in rings], [tuple(pts[i]) for i in site]


def _pad_uniform(g, rings, target, rng):
    """Grow the rings with filler until the graph has exactly `target` nodes,
    spread evenly so ring lengths stay comparable.

    This is what makes n an independent knob: filler sits in ONE ring, so it
    carries no ring pairs and costs no fan-out. A --share sweep can therefore
    hold n fixed while fan-out moves, which is the whole point of the sweep.

    Where a node may be inserted is not free. Splitting an edge two rings share
    would leave its endpoints non-adjacent here but still adjacent there -- a
    chord. Safe edges are the ones only this ring has: the junction between two
    interfaces, or an existing filler edge.

        [ iface A ][ iface B ]        splitting inside A: NO  (A is shared)
                  ^                   splitting at the junction: OK
                  safe                (A and B share only this ring)
    """
    extra = target - g.number_of_nodes()
    if extra <= 0:
        return g, rings
    memb = {}
    for k, r in enumerate(rings):
        for v in r:
            memb.setdefault(v, set()).add(k)

    def safe_at(r, k):
        """Index j in ring k such that the edge after j belongs to ring k alone."""
        return next((j for j in range(len(r))
                     if len(memb[r[j]] & memb[r[(j + 1) % len(r)]]) == 1), None)

    spots = [(k, safe_at(r, k)) for k, r in enumerate(rings)]
    spots = [(k, j) for k, j in spots if j is not None]
    if not spots:
        return g, rings
    rng.shuffle(spots)
    # Even split, remainder handed to the first few of a shuffled order so no
    # ring is systematically favoured.
    per, rem = divmod(extra, len(spots))
    nxt = max(g.nodes()) + 1
    for i, (k, j) in enumerate(spots):
        add = per + (1 if i < rem else 0)
        if not add:
            continue
        new = list(range(nxt, nxt + add))
        nxt += add
        rings[k] = rings[k][:j + 1] + new + rings[k][j + 1:]
    h = nx.Graph()
    h.add_nodes_from((v, dict(g.nodes[v])) for v in g.nodes())
    for r in rings:
        for j in range(len(r)):
            h.add_edge(r[j], r[(j + 1) % len(r)])
    return h, rings


def _spof_final(g, rings, frac, rng):
    """Narrow a fraction of the plan's ring pairs to a single shared node (SPOF).

    Runs on the FINISHED plan because the denominator -- how many ring pairs
    exist at all -- is not known until the cover has been laid out, and --spof is
    a fraction of that.

    A pair sharing m>=2 nodes is narrowed by SPLITTING m-1 of them: the shared
    node becomes two, one left in each ring, each keeping that ring's neighbours.

        P: - X - Y -            P: - X - Y -      pair now meets only at Y,
        Q: - X - Y -    ==>     Q: - X'- Y -      so losing Y severs it

    The last shared node is never split, so a transition is only narrowed, never
    destroyed -- the ring graph keeps its connectivity, which the self-check
    pins. Splitting only ever REMOVES adjacency, so it cannot create a chord.
    """
    rsets = [set(r) for r in rings]
    memb = {}
    for i, r in enumerate(rsets):
        for v in r:
            memb.setdefault(v, set()).add(i)
    pairs = {}
    for v, rs in memb.items():
        for a in rs:
            for b in rs:
                if a < b:
                    pairs.setdefault((a, b), set()).add(v)
    if not pairs:
        return g, rings

    target = int(round(frac * len(pairs)))
    cur = sum(1 for s in pairs.values() if len(s) == 1)
    cand = [p for p, s in pairs.items() if len(s) > 1]
    rng.shuffle(cand)
    nxt = max(g.nodes()) + 1 if g.number_of_nodes() else 0

    def splittable(v, b):
        # Moving v out of ring b costs every OTHER ring at v one shared node with
        # b. Only allow it where those pairs can spare one, so narrowing never
        # becomes a disconnection.
        return all(len(pairs.get((min(b, c), max(b, c)), ())) >= 2
                   for c in memb[v] if c != b)

    for a, b in cand:
        if cur >= target:
            break
        shared = pairs[(a, b)]
        if len(shared) < 2:
            continue
        keep = next(iter(shared))
        movers = [v for v in shared if v != keep]
        # b gives up its copy of every shared node but one; all-or-nothing,
        # since a partial split leaves the pair safe and only churns the graph.
        if not all(splittable(v, b) for v in movers):
            continue
        for v in movers:
            v2, nxt = nxt, nxt + 1
            g.add_node(v2, **dict(g.nodes[v]))
            rings[b] = [v2 if x == v else x for x in rings[b]]
            rsets[b].discard(v); rsets[b].add(v2)
            memb[v].discard(b); memb[v2] = {b}
            for c in list(memb[v]):
                pairs[(min(b, c), max(b, c))].discard(v)
        cur = sum(1 for s in pairs.values() if len(s) == 1)

    # g is exactly the union of the ring cycles, so rebuilding it is how the
    # split nodes pick up their edges (and the old ones drop theirs).
    h = nx.Graph()
    h.add_nodes_from((v, dict(g.nodes[v])) for v in g.nodes())
    for r in rings:
        for j in range(len(r)):
            h.add_edge(r[j], r[(j + 1) % len(r)])
    h.remove_nodes_from([v for v in list(h.nodes()) if h.degree(v) == 0])
    idx = {v: k for k, v in enumerate(sorted(h.nodes()))}
    return (nx.relabel_nodes(h, idx, copy=True),
            [[idx[v] for v in r] for r in rings])


def _prune_solo(g, rings, frac, rng):
    """Splice out filler -- nodes in exactly ONE ring. They carry no ring pairs,
    so they add nothing to fan-out and only lengthen the lap.

        - a - f - b -   ==>   - a --- b -     f gone, ring still a loop

    Removing one outright would break the loop, so its two neighbours are joined
    instead. Used to reach a plan's FLOOR (prune_solo=1.0), which is where the
    sweeps build every point before padding back up to --target-n.

    Never below 3 members, and only degree-2 nodes. A splice is also skipped when
    the new a-b edge would join two members of some OTHER ring that are not
    adjacent there -- that edge would be a chord (see _plan_cliques).
    """
    memb = {}
    in_rings = {}
    for k, r in enumerate(rings):
        for v in r:
            memb[v] = memb.get(v, 0) + 1
            in_rings.setdefault(v, set()).add(k)
    adj_in = [{frozenset((r[j], r[(j + 1) % len(r)])) for j in range(len(r))}
              for r in rings]

    def chords(a, b, this):
        """Would joining a-b leave them non-adjacent in a ring they both carry?
        `this` is the ring being spliced, where the join makes them adjacent."""
        return any(k != this and frozenset((a, b)) not in adj_in[k]
                   for k in in_rings.get(a, ()) & in_rings.get(b, ()))
    solo = [v for v in g.nodes() if memb.get(v, 0) == 1 and g.degree(v) == 2]
    rng.shuffle(solo)
    solo = set(solo[:int(round(frac * len(solo)))])

    out = []
    for this, r in enumerate(rings):
        keep, i = list(r), 0
        while i < len(keep) and len(keep) > 3:
            v = keep[i]
            nbrs = list(g[v]) if v in solo and v in g else []
            if len(nbrs) == 2 and not chords(*nbrs, this):
                g.remove_node(v)
                g.add_edge(*nbrs)      # splice: the ring closes over the gap
                keep.pop(i)
                continue
            i += 1
        out.append(keep)

    idx = {v: k for k, v in enumerate(sorted(g.nodes()))}
    h = nx.relabel_nodes(g, idx, copy=True)
    return h, [[idx[v] for v in r] for r in out]      # ring count unchanged


def build(family="gabriel", rings=100, seed=0, ring_size=8,
          spof=0.0, share=0.0, width_max=WIDTH_MAX,
          ring_size_max=0, prune_solo=0.0, target_n=0, max_meet=0, **kw):
    """The whole pipeline: site graph -> clique cover -> rings -> post-passes.

        sites          _plan_cliques      _prune_solo   _spof_final  _pad_uniform
        (A points) --> rings + plan  -->  down to    -> narrow some -> up to
                                          the floor    transitions    target_n

    rings   A, the site/ring count.
    share   how many rings meet at one interface (2 .. max_meet): the fan lever.
    spof    fraction of transitions narrowed to a single shared node.
    prune_solo  fraction of filler spliced out; 1.0 gives the plan's floor.
    target_n    pad back up to exactly this many nodes. Filler costs no fan-out,
                so this is how n is held fixed across a sweep.

    ring_size sets the minimum ring length AND, via ring_size//2, the cap on site
    degree -- which is what limits interfaces per site, and so fan-out. Once
    target_n is padding, that degree cap is the only thing ring_size still
    decides. ring_size_max, if set, sizes each ring to its site's degree within
    [ring_size, ring_size_max] so busy sites get the big rings."""
    s = _check_ring_size(ring_size)
    s_max = _check_ring_size(ring_size_max) if ring_size_max else s
    if s_max < s:
        raise ValueError(f"--ring-size-max {s_max} < --ring-size {s}")
    rng = np.random.default_rng(seed)
    pts = rng.random((rings, 2))
    sg = nx.Graph(); sg.add_nodes_from(range(rings))
    for u, v in _raw_site_edges(family, pts, seed):
        sg.add_edge(u, v, w=float(np.linalg.norm(pts[u] - pts[v])))
    # Cap against the LARGEST ring: a site may keep up to size//2 neighbours,
    # and the big rings are handed to the high-degree sites below.
    sg = _hard_cap(_cap_degree(sg, s_max // 2), s_max // 2)
    if s_max > s:
        # Exactly the nodes this site's interfaces need (2 each), clamped to
        # [s, s_max]. Slack beyond that is pure filler: measured against a
        # random bump toward s_max, and against uniform +1/+2 spare nodes, on
        # all four families -- fan-out reach was identical within seed noise
        # while the bump generated ~25-35% more single-ring nodes.
        size = {v: min(max(s, 2 * sg.degree(v)), s_max) for v in sg.nodes()}
    else:
        size = {v: s for v in sg.nodes()}
    g, plan, rpos = _plan_cliques(sg, pts, rng, spof, share, s, width_max, max_meet)
    if prune_solo > 0:
        g, plan = _prune_solo(g, plan, prune_solo, rng)
    if spof > 0:
        # Last, so the fraction is measured against the pairs that actually
        # exist after share, pruning and splicing. Its own splits only ever
        # remove adjacency, so they cannot reintroduce a chord.
        g, plan = _spof_final(g, plan, spof, rng)
    if target_n:
        # Last: --spof splits nodes, so padding before it would overshoot.
        g, plan = _pad_uniform(g, plan, target_n, rng)
    return g, plan, rpos


def write_out(g, plan, rpos, stem, sub, prov):
    """Write one built topology's three files: the graph (csv/), the ring labels
    MyriadGP reads (ringlabels/) and a picture (gallery/). Shared with
    ringgrow.py, which builds its plans differently but ships the same triple."""
    s = score(g, plan)          # raises if any ring isn't hop-walkable
    # Walkable is not enough: GP also needs every member to have exactly TWO
    # ring-neighbours, or its lap has a fork it cannot resolve. The planner
    # cannot break that, but the post-passes reshape the graph afterwards, so it
    # is checked on the way out -- shipping a chorded plan would silently ruin
    # every eval run built on it.
    bad = ringcheck.offenders({i: set(r) for i, r in enumerate(plan)},
                              {v: set(g[v]) for v in g})
    if bad:
        raise AssertionError(f"chorded rings in the shipped plan: {sorted(bad)}")

    # .native.ring marks a topology that SHIPS its ring plan (use --basis
    # native); the suffix makes that visible in a directory listing.
    os.makedirs(os.path.join(CSV_DIR, sub), exist_ok=True)
    os.makedirs(os.path.join(RINGLABELS_DIR, sub), exist_ok=True)
    topo = os.path.join(CSV_DIR, sub, f"{stem}.native.ring.txt")
    ini = os.path.join(RINGLABELS_DIR, sub, f"{stem}.native.ring.ini")
    exp.write_topology(g, topo, prov)
    hdr = [f"ringLabel lines generated from {stem}.native.ring.txt",
           prov,
           f"nodes={s['n']} edges={s['m']} avgdeg={s['avgdeg']:.2f} rings={s['rings']} echo={s['echo']}",
           f"fan-out(sum k*(k-1))={s['fan']} fan/n={s['fan_n']:.2f} max rings/node={s['maxk']}",
           f"transitions: {s['pairs']} ring pairs, {s['spof']} spof ({s['spofpct']:.1f}%)",
           f"ring-graph components={s['comp']} biconnected={s['biconn']} diameter={s['diam']}"]
    exp.write_ringlabels(g, plan, ini, hdr)

    member, n_echo = rings_to_membership(g, plan)
    n_rings = len(plan) + n_echo
    ring_of = {v: [t2r.ring_id(i, len(plan)) for i in sorted(member[v])] for v in g.nodes()}
    ring_ids = [t2r.ring_id(i, len(plan)) for i in range(n_rings)]
    stats = t2r.topology_stats(g, ring_of, n_rings, n_echo, 0, f"{stem}  {prov}")
    os.makedirs(os.path.join(GALLERY, sub), exist_ok=True)
    png = os.path.join(GALLERY, sub, f"{stem}.png")
    # Echo rings have no site, so they get no fixed position -- draw() falls
    # back to a spring layout for the whole panel if any ring is missing one.
    ring_pos = {t2r.ring_id(i, len(plan)): p for i, p in enumerate(rpos)}
    t2r.draw(g, ring_of, ring_ids, png, stats, ring_pos)
    return s, topo, ini, png


def export_and_plot(family, rings, seed=0, ring_size=8,
                    spof=0.0, share=0.0, width_max=WIDTH_MAX,
                    ring_size_max=0, prune_solo=0.0, quiet=False, target_n=0,
                    sub=None, max_meet=0):
    """Build one topology from the ringpipe knobs and write it out."""
    g, plan, rpos = build(family, rings, seed, ring_size, spof, share, width_max,
                          ring_size_max, prune_solo, target_n, max_meet)
    # The name carries what the run ACHIEVED, not just what was asked for, since
    # eval_scaling reads its axis off these names:
    #   _rl<N>   the median ring length this seed actually got
    #   _tr<N>   transitions: ring pairs sharing at least one node -- the count
    #            of inter-ring hand-offs the control plane has to maintain,
    #            which fan-out weights but does not itself report
    #   _fan/_n  fan-out and node count achieved
    tag = (f"_rl{_median_len(plan)}"
           + (f"_rs{ring_size}-{ring_size_max}" if ring_size_max else "")
           + (f"_ps{prune_solo:g}" if prune_solo else ""))
    s0 = score(g, plan)
    stem = (f"ringpipe_{family}_A{rings}_spof{spof:g}_share{share:g}{tag}"
            f"_seed{seed}_tr{s0['pairs']}_fan{s0['fan']}_n{s0['n']}")
    # One subfolder per family so a sweep's outputs stay together and
    # `eval_scaling --dir` can take the folder as the sweep. Keyed by A normally;
    # sweeps that SOLVE for --rings pass their own `sub`, since their points have
    # different ring counts and would otherwise scatter one folder each.
    sub = sub or f"{family}_A{rings}_spof{spof:g}"
    prov = (f"myriad_topologies/ringpipe.py family={family} rings(A)={rings} "
            f"spof_target={spof:g} share_target={share:g} width_max={width_max} "
            f"seed={seed}")
    s, topo, ini, png = write_out(g, plan, rpos, stem, sub, prov)

    if not quiet:
        print(f"{family:9} A={rings:4} spof->{spof:4g} share->{share:4g} "
              f"n={s['n']:5} fan/n={s['fan_n']:6.2f} spof={s['spofpct']:5.1f}% "
              f"maxk={s['maxk']:2} diam={s['diam']:4}")
        print(f"  {topo}\n  {ini}\n  {png}")
    return s, topo, ini, png


AUTO_RING_SIZES = (6, 8, 10, 12, 14, 16, 20, 24, 32, 48)


def _floor_n(**kw):
    """Nodes left once every spliceable filler node is gone -- the smallest plan
    these settings can make, and so the lowest n a run can be padded from."""
    return build(prune_solo=1.0, **kw)[0].number_of_nodes()


def _fit_ring_size(target, tol, sizes=AUTO_RING_SIZES, **kw):
    """Pick the ring size giving the MOST fan-out at `target` nodes.

    Ring size does not decide n any more -- _pad_uniform does, exactly -- but it
    still decides fan, because it caps site degree at ring_size//2 and that cap
    limits how many interfaces a site can carry. More interfaces per site means
    more ring memberships per node, and fan is quadratic in those."""
    best = None
    for rs in sizes:
        floor = _floor_n(ring_size=rs, **kw)
        if floor > target + tol:
            continue                      # can only pad UP from the floor
        s = score(*build(ring_size=rs, prune_solo=1.0,
                         target_n=max(target, floor), **kw)[:2])
        if best is None or s["fan"] > best[0]:
            best = (s["fan"], rs)
    if best is None:
        lows = ", ".join(f"rs{rs}:{_floor_n(ring_size=rs, **kw)}" for rs in sizes[:4])
        raise ValueError(
            f"every ring size floors above n={target} (+{tol}) at "
            f"--rings {kw.get('rings')}, share={kw.get('share')} ({lows}). "
            f"The floor scales with the site count -- lower --rings, or raise "
            f"--target-n.")
    return best[1]


def _design(kw):
    """The same kw with seed 0 forced.

    Anything a sweep SOLVES for -- --rings for a target length, --share for a
    target fan -- is part of the experiment's design, not of the draw, so it is
    solved once on seed 0 and reused by every seed. Solving per seed instead lets
    the solver's own jitter dominate: --hold-len landed on 102..161 rings for the
    same share=1 point, and fan (which grows with the ring count, and with
    max_meet ~ sqrt(rings)) came out 10926..27934 across seeds."""
    return {**kw, "seed": 0}


def _sweep_max_meet(counts):
    """One rings-per-interface ceiling for a whole sweep.

    max_meet defaults to sqrt(--rings) per BUILD, which is fine when --rings is
    fixed. But a sweep that SOLVES for --rings (--hold-len)
    then moves the ceiling underneath itself: --hold-len landed on

        share     0    0.2   0.4   0.6   0.8   1.0
        rings    72     74    78    78    89   139
        max_meet  8      9     9     9     9    12   <- the swept knob's own cap

    and --share means "scale from 2 up to max_meet", so the last point got a
    bigger cap AND more rings at once -- a 3.2x step in fan that the x axis
    attributes to share alone. Pinning one ceiling removes that and reaches more
    fan besides (41206 vs 20700 at share=1 here).

    Sized from the LARGEST solved ring count, not the median or the mean: the
    ceiling only binds where the sweep actually spends it -- the high-share, or
    short-ring, end -- and sizing it to a middling point throttles exactly the
    point the sweep is built to reach (median gave 12744 against 41206)."""
    return max(4, round(math.sqrt(max(counts))))


def _median_len(plan):
    L = sorted(len(r) for r in plan)
    return L[len(L) // 2]


def _fit_rings(target_len, target_n, hint=0, **kw):
    """The --rings count whose rings run ~target_len nodes long at n=target_n.

    From the identity in the module docstring, sum(ring lengths) = n + surplus:
    with n pinned, the more rings there are the SHORTER each must be. Monotone,
    hence bisection. This is the ONLY way to move ring length once n is fixed --
    --ring-size cannot, because _pad_uniform equalises the rings anyway."""
    # A hint (the count that fitted at a nearby point) narrows the search to a
    # band around it, which is what keeps the joint solves affordable. An answer
    # AT the band edge means the real one is outside it, so that case re-solves
    # over the full range -- share can move the surplus far enough in one step
    # that a band around the last answer misses badly.
    full = (3, max(8, target_n // 3))
    lo, hi = ((max(3, int(hint * 0.6)), max(8, int(hint * 1.7))) if hint else full)
    banded = bool(hint)
    while hi > lo and _floor_n(rings=hi, **kw) > target_n:
        hi = hi * 3 // 4          # that many rings already floors above target_n
    if hi <= lo:                  # hint band collapsed: nothing to bisect over
        return max(3, hi)
    best = None
    while lo < hi:
        mid = (lo + hi) // 2
        m = _median_len(build(rings=mid, prune_solo=1.0, target_n=target_n, **kw)[1])
        best = min(best or (10 ** 9, mid), (abs(m - target_len), mid))
        if m > target_len:
            lo = mid + 1          # rings too long -> cut them by making more
        else:
            hi = mid
    got = best[1] if best else max(3, lo)
    if banded and not (max(3, int(hint * 0.6)) < got < int(hint * 1.7) - 1):
        return _fit_rings(target_len, target_n, hint=0, **kw)   # answer was outside
    return got


def _pmap(fn, jobs, args):
    """[fn(*a) for a in args], spread over `jobs` PROCESSES.

    Points of a sweep are independent -- each writes its own three files -- so
    the only shared state is the console, and the parent prints once they all
    land, in order. Processes rather than threads: the work is numpy/networkx
    graph building, which would just queue behind the GIL."""
    if jobs <= 1 or len(args) < 2:
        return [fn(*a) for a in args]
    with cf.ProcessPoolExecutor(max_workers=jobs) as ex:
        return [f.result() for f in [ex.submit(fn, *a) for a in args]]


def _floor_point(sh, auto, kw):
    return (min(_floor_n(share=sh, ring_size=rs, **kw) for rs in AUTO_RING_SIZES)
            if auto else _floor_n(share=sh, **kw))


def _share_point(sh, n, tol, auto, kw):
    rs = _fit_ring_size(n, tol, share=sh, **kw) if auto else None
    s, *_ = export_and_plot(share=sh, prune_solo=1.0, target_n=n, quiet=True,
                            **({"ring_size": rs} if rs else {}), **kw)
    return s, rs


def _held_point(sh, a, target_n, sub, kw):
    s, *_ = export_and_plot(rings=a, share=sh, prune_solo=1.0, target_n=target_n,
                            quiet=True, sub=sub, **kw)
    got = _median_len(build(rings=a, share=sh, prune_solo=1.0,
                            target_n=target_n, **kw)[1])
    return s, got


def sweep_share(shares, target_n=0, tol=25, auto_ring_size=False, hold_len=None,
                jobs=1, **kw):
    """One topology per --share value, all at exactly target_n nodes.

    Every point is built at its FLOOR (prune_solo=1.0, all spliceable filler
    gone) and padded back up to the target. Filler costs no fan-out, so n is held
    without touching the thing being swept.

    Default target = the LARGEST floor in the sweep, since padding can only add:
    that is the lowest n every point can reach.

    hold_len pins the mean RING LENGTH too, by solving --rings per point. Without
    it the rings grow along with fan-out -- 13 -> 21 over a 0..1 sweep -- because
    sum(ring lengths) = n + surplus and --share IS the surplus, so with n and the
    ring count both fixed the length is the only thing left to absorb it. A curve
    from such a sweep mixes two causes; with --hold-len it is fan-out alone.
    0 = anchor on the shortest length the highest share can manage."""
    if auto_ring_size and hold_len is not None:
        raise ValueError("--auto-ring-size picks --ring-size for maximum fan and "
                         "--hold-len picks --rings for a fixed length; they fight "
                         "over the same trade. Use one.")
    if auto_ring_size:
        kw.pop("ring_size", None)
        kw.pop("ring_size_max", None)     # size is chosen per point now
    if hold_len is not None:
        return _sweep_share_holding_len(shares, target_n, hold_len, jobs, **kw)
    floors = dict(zip(shares, _pmap(_floor_point, jobs,
                                    [(sh, auto_ring_size, kw) for sh in shares])))
    if not target_n:
        target_n = max(floors.values())
        print(f"  target-n not given, using the largest floor in the sweep: {target_n}")
    for sh in shares:
        if floors[sh] > target_n + tol:
            raise ValueError(
                f"share={sh:g} floors at n={floors[sh]}, above --target-n "
                f"{target_n} (+{tol}); padding can only add. Raise --target-n to "
                f"{max(floors.values())} or lower --rings.")
    for sh, (s, rs) in zip(shares, _pmap(
            _share_point, jobs,
            [(sh, max(target_n, floors[sh]), tol, auto_ring_size, kw)
             for sh in shares])):
        size = f"ring_size={rs:3} " if rs else ""
        print(f"share={sh:4g} {size}n={s['n']:5} fan={s['fan']:7} "
              f"fan/n={s['fan_n']:6.2f} maxk={s['maxk']:3} "
              f"spof={s['spofpct']:5.1f}%")


def _sweep_share_holding_len(shares, target_n, hold_len, jobs=1, **kw):
    """--sweep-share with the ring length pinned too: --rings solved per point
    (see sweep_share's hold_len, and _fit_rings). The ring count varies across
    points, so the files go to ONE folder rather than a per-ring-count one.

    --jobs only parallelises the export: the solver is a chain, each point
    bisecting in a band around its predecessor's answer, and cutting that chain
    to parallelise it would change which ring counts the sweep lands on."""
    if not target_n:
        raise ValueError("--hold-len needs --target-n: with the length pinned "
                         "too, n is the only anchor left to solve --rings against")
    rings0 = kw.pop("rings", None)
    shares = sorted(shares)
    if not hold_len:
        # Anchor on the SHORTEST length the highest share can manage. That point
        # is the hard one: its surplus is the biggest, and max_meet grows with
        # sqrt(--rings), so adding rings adds surplus about as fast as it adds
        # room -- its floor length is what every lower share can still meet.
        # Anchored on seed 0 whatever --seed this run uses: the anchor defines the
        # experiment, so every seed of a sweep must solve for the SAME length, or
        # they are not measuring one thing (and their files, which carry the
        # anchor in the folder name, would not even land together).
        hard, anch = max(shares), _design(kw)
        a = _fit_rings(2, target_n, share=hard, **anch)    # 2 = "as short as it goes"
        hold_len = _median_len(build(rings=a, share=hard, prune_solo=1.0,
                                     target_n=target_n, **anch)[1])
        print(f"  --hold-len given no value, anchoring on the shortest rings "
              f"share={hard:g} can reach: {hold_len}")
    sub = f"{kw['family']}_shr{target_n}_rl{hold_len:g}_spof{kw.get('spof', 0):g}"

    def solve_all(k):
        # Ascending: _fit_rings bisects in a band around the previous point's
        # answer, so a point's ring count depends on its predecessor. Neighbours
        # in share are neighbours in ring count; an out-of-order list hands a
        # point a hint from a share nowhere near it.
        out, hint = [], rings0 or 0
        for sh in shares:
            hint = _fit_rings(hold_len, target_n, hint=hint, share=sh, **_design(k))
            out.append(hint)
        return out

    if not kw.get("max_meet"):
        # First pass only to see the ring counts, so the ceiling can be fixed for
        # the sweep; the counts are then re-solved under it, since pinning the
        # ceiling changes the surplus and so the count a length needs.
        kw = {**kw, "max_meet": _sweep_max_meet(solve_all(kw))}
        print(f"  --max-meet not given, pinning the sweep to "
              f"{kw['max_meet']} rings per interface")
    counts = solve_all(kw)
    for (sh, a), (s, got) in zip(zip(shares, counts), _pmap(
            _held_point, jobs,
            [(sh, a, target_n, sub, kw) for sh, a in zip(shares, counts)])):
        flag = "" if abs(got - hold_len) <= 1 else f"  (len {got}, wanted {hold_len})"
        print(f"share={sh:4g} rings={a:5} ringlen={got:3} n={s['n']:5} "
              f"fan={s['fan']:7} fan/n={s['fan_n']:6.2f} maxk={s['maxk']:3} "
              f"spof={s['spofpct']:5.1f}%{flag}")


def _selfcheck():
    for fam in SITE_FAMILIES:
        for spof in (0.0, 0.3):
            for share in (0.0, 0.3):
                g, plan, _ = build(fam, rings=30, seed=0, spof=spof, share=share)
                assert nx.is_connected(g), (fam, spof, share)
                s = score(g, plan)         # validates ring walkability
                assert s["n"] > 0
    # varied ring sizes: rings must stay walkable (score checks) and actually
    # vary.
    for fam in SITE_FAMILIES:
        g, plan, _ = build(fam, rings=40, seed=1, share=0.5, ring_size=8, ring_size_max=14)
        assert nx.is_connected(g), fam
        score(g, plan)
    g, plan, _ = build("delaunay", rings=40, seed=1, share=0.5, ring_size=8, ring_size_max=14)
    assert len({len(r) for r in plan}) > 1, "ring sizes did not vary"
    # --hold-len's solver: --rings is solved per length, n stays pinned, and the
    # rings really do come out the requested length (see _fit_rings).
    for L in (8, 16):
        a = _fit_rings(L, 600, family="delaunay", seed=1, ring_size=8,
                       spof=0.1, share=0.3)
        g, plan, _ = build("delaunay", rings=a, seed=1, ring_size=8, spof=0.1,
                           share=0.3, prune_solo=1.0, target_n=600)
        assert g.number_of_nodes() == 600, (L, a, g.number_of_nodes())
        assert abs(_median_len(plan) - L) <= 2, (L, a, _median_len(plan))
    # spof lands on target regardless of share, and never partitions the ring
    # graph (a narrowed transition is still a transition).
    for sp in (0.1, 0.33):
        for sh in (0.0, 0.9):
            g, plan, _ = build("delaunay", rings=60, seed=2, spof=sp, share=sh)
            s = score(g, plan)
            assert abs(s["spofpct"] / 100 - sp) < 0.05, (sp, sh, s["spofpct"])
            assert s["comp"] == 1 and nx.is_connected(g), (sp, sh)
    # prune-solo: rings stay walkable (score checks), the graph stays connected,
    # nodes actually go away, and fan-out is untouched -- filler carries none.
    for fam in SITE_FAMILIES:
        base = build(fam, rings=40, seed=1, share=0.3)[:2]
        g, plan, _ = build(fam, rings=40, seed=1, share=0.3, prune_solo=1.0)
        assert nx.is_connected(g), fam
        s0, s1 = score(*base), score(g, plan)
        assert s1["n"] < s0["n"], fam
        assert s1["fan"] == s0["fan"], (fam, s0["fan"], s1["fan"])
    print("ringpipe selfcheck ok")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--family", choices=SITE_FAMILIES, default="gabriel")
    ap.add_argument("--rings", type=int, default=100, help="A: base ring/site count")
    ap.add_argument("--spof", type=float, default=0.0,
                    help="target fraction of transitions that are single-node (SPOF)")
    ap.add_argument("--share", type=float, default=0.0,
                    help="how many rings meet at one interface: 0 -> 2 (a plain "
                         "corridor), 1 -> --max-meet. THE fan-out lever, since "
                         "every node of an m-ring interface sits in m rings and "
                         "fan per node is m(m-1)")
    ap.add_argument("--max-meet", type=int, default=0, metavar="M",
                    help="ceiling on how many rings meet at one interface; "
                         "--share scales from 2 up to this. The strongest fan-out "
                         "lever: every node of an m-ring interface sits in m rings "
                         "and fan per node is m(m-1), quadratic, where --width-max "
                         "is linear. Default 0 picks ~sqrt(--rings), near the peak "
                         "-- higher backfires, since a clique of m eats C(m,2) ring "
                         "pairs and the cover runs out of candidates")
    ap.add_argument("--width-max", type=int, default=WIDTH_MAX,
                    help="cap on nodes per interface; each one gets a width drawn "
                         "from [2, this], so a topology carries a MIX of transition "
                         "widths. A second fan-out lever alongside --share: fan "
                         "tracks the mean width (~(2+N)/2) and is linear in it, "
                         "where --share is quadratic. 1 makes every transition a SPOF")
    ap.add_argument("--prune-solo", type=float, default=0.0, metavar="FRAC",
                    help="fraction of filler (single-ring) nodes to splice out; "
                         "they add no fan-out, only lap length. 1 = the plan's "
                         "floor, which is what the sweeps build from before "
                         "padding up to --target-n")
    ap.add_argument("--ring-size", type=int, default=8,
                    help="ring size (even, >=6); the minimum when --ring-size-max is set")
    ap.add_argument("--ring-size-max", type=int, default=0,
                    help="if set, ring sizes vary in [--ring-size, this], the "
                         "biggest rings going to the busiest sites; 0 = all rings "
                         "start equal")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--hold-len", type=float, nargs="?", const=0, default=None,
                    metavar="L",
                    help="with --sweep-share: solve --rings per point so the mean "
                         "RING LENGTH stays put, leaving fan-out the only variable. "
                         "sum(ring lengths) = n + sharing surplus and --share IS "
                         "that surplus, so at a fixed n and ring count the rings "
                         "grow instead (13->21 over 0..1). Give a length, or pass "
                         "it bare to anchor on the shortest the top share can reach")
    ap.add_argument("--sweep-share", nargs="+", type=float, metavar="F",
                    help="one topology per share value, each padded to --target-n "
                         "nodes with filler (which costs no fan-out), so the points "
                         "differ in fan-out at a fixed node count. Add --hold-len "
                         "to stop ring length drifting up along with it")
    ap.add_argument("--target-n", type=int, default=0,
                    help="exact node count for every point of a sweep (0 = the "
                         "smallest n every point can reach). Required by "
                         "--hold-len, which solves against it")
    ap.add_argument("--target-tol", type=int, default=25,
                    help="how far a point's FLOOR may sit above --target-n before "
                         "the sweep gives up (padding can only add nodes). The "
                         "count itself is exact, so this is not an error bar")
    ap.add_argument("--auto-ring-size", action="store_true",
                    help="in --sweep-share: pick --ring-size per point for the most "
                         "fan-out. Ring size no longer moves n (padding does), but "
                         "it caps site degree at ring_size//2, which limits "
                         "interfaces per site and so fan-out")
    ap.add_argument("--jobs", type=int, default=1, metavar="N",
                    help="build a sweep's points on N processes (default 1). "
                         "--hold-len's ring-count solver stays serial: it is a "
                         "chain, and breaking it would move the sweep's points")
    ap.add_argument("--selfcheck", action="store_true")
    a = ap.parse_args()
    if a.selfcheck:
        return _selfcheck()
    if a.sweep_share:
        kw = dict(family=a.family, rings=a.rings, seed=a.seed,
                  ring_size=a.ring_size, spof=a.spof, width_max=a.width_max,
                  ring_size_max=a.ring_size_max, max_meet=a.max_meet)
        return sweep_share(a.sweep_share, a.target_n, a.target_tol,
                           auto_ring_size=a.auto_ring_size, hold_len=a.hold_len,
                           jobs=a.jobs,
                           **kw)
    export_and_plot(a.family, a.rings, a.seed, a.ring_size, a.spof, a.share,
                    a.width_max, a.ring_size_max, a.prune_solo,
                    max_meet=a.max_meet)


if __name__ == "__main__":
    main()
