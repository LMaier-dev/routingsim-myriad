#!/usr/bin/env python3
"""Build a ring plan from a chosen SITE GRAPH, then emit it as a topology.
Generalizes ringmesh (Gabriel sites, single-edge glue) with a few knobs:

  --family     which site-graph rule to start from
  --rings (A)  number of sites = number of base rings
  --spof       fraction of ring-to-ring links guarded by a SINGLE shared node
               (a SPOF). Spent on non-backbone links first, so the important
               backbone links keep their redundancy longest.
  --share      fraction of links that reuse a site's already-shared node instead
               of a fresh one, so ONE node lands in 3+ rings. This is the real
               fan-out lever: a node's cost jumps k=2->3 (+6) or 3->4 (+8), so
               piling links onto repeat hubs matters more than adding links.

    python3 ringpipe.py --family gabriel --rings 100 --spof 0.1 --share 0.1
    python3 ringpipe.py --family rgg --rings 200
    python3 ringpipe.py --selfcheck
"""
import argparse
import math
import os
import sys

import numpy as np
import networkx as nx

HERE = os.path.dirname(os.path.abspath(__file__))
MYRIAD = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, MYRIAD)

from topo_lib import _check_ring_size, _cap_degree, _delaunay_edges, _DSU
from metrics import score, rings_to_membership
import export as exp
import topo2rings as t2r

SITE_FAMILIES = ("gabriel", "delaunay", "powerlaw", "rgg")
WIDTH_MAX = 3          # default max transition width (shared nodes = width+1)
HUB_CAP = 3            # share edges one hub anchor absorbs before a fresh one opens
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
    ("connectivity beats the slot budget"). Harmless for Gabriel/Delaunay, but
    powerlaw's hubs trigger it, and _glue_widths forces tree edges to width 1 --
    dropping one silently splits the graph. Trim the longest excess edges,
    skipping any whose removal would disconnect."""
    for v in sorted(sg.nodes(), key=lambda v: -sg.degree(v)):
        for u, w, d in sorted(sg.edges(v, data=True), key=lambda e: -e[2]["w"]):
            if sg.degree(v) <= max_deg:
                break
            sg.remove_edge(u, w)
            if not nx.is_connected(sg):
                sg.add_edge(u, w, **d)
    return sg


def _share_pairs(i, j, a, b, shares, si, sj):
    """Union-find pairs for one glue: `shares` nodes shared between ring i (from
    slot a) and ring j (from slot b). shares=1 -> SPOF, 2 -> normal glue, >=3 ->
    a wide corridor -- all three are this one formula at different `shares`.
    Each side wraps around its OWN ring size (si/sj), which differ once ring
    sizes are randomized."""
    return [((i, (a + da) % si), (j, (b + (shares - 1) - da) % sj)) for da in range(shares)]


def _glue_widths(sg, pts, size, rng, spof=0.0, share=0.0, width_max=WIDTH_MAX,
                 hub_cap=HUB_CAP, prune_ring=0.0):
    """Glue the site graph into rings. Like the old fixed single-edge glue, but
    with knobs:

    spof: target fraction of links sharing exactly 1 node -- NOT decided here
    (the total link count isn't known until `share` has invented its extra
    links); _spof_final does it afterwards on the finished plan.
    share: target fraction of links that reuse a site's hub slot instead of a
    fresh one, building 3+-ring hubs.

    Each glue costs 1 slot per side (a hub reuse costs 0 on the reusing side),
    except a wide corridor (shares>=3) which reserves shares-1 slots -- paid only
    where a site has spare room. Backbone (tree) links always take the minimal
    shares=2, so connectivity never depends on spare capacity.

    `size` = each site's slot count. A site's glue capacity is size//2, so
    bigger rings are what let a high --share actually fit.
    """
    free = {i: list(range(0, size[i], 2)) for i in sg.nodes()}
    for i in free:
        rng.shuffle(free[i])
    hub = {}            # site -> slot reused to build it into a 3+-ring hub
    hub_use = {}        # site -> share edges the current anchor has absorbed
    dsu = _DSU()
    tree = {tuple(sorted(e)) for e in nx.minimum_spanning_tree(sg, weight="w").edges()}
    edges = list(sg.edges())
    non_tree = [tuple(sorted(e)) for e in edges if tuple(sorted(e)) not in tree]
    rng.shuffle(non_tree)

    # spof isn't chosen here (see _spof_final): the denominator -- total ring
    # pairs -- isn't known yet, because share adds extra links later.
    spof_set = set()
    share_pool = [tuple(sorted(e)) for e in edges]
    rng.shuffle(share_pool)

    # share: build hubs (>=2 links at the SAME site) by ROUND-ROBIN over sites,
    # not by draining the busiest first -- draining piles every hub into one
    # cluster (in a geometric family the busy sites sit together, so their
    # mutual edges get picked by both ends and tangle up). Draw from ALL edges
    # (minus spof's picks); holding back tree edges capped share far too low
    # (66% on delaunay), since a hub reuse is never costlier than the plain glue
    # it replaces.
    touch = {}
    for key in share_pool:
        touch.setdefault(key[0], []).append(key)
        touch.setdefault(key[1], []).append(key)
    sites = sorted(touch, key=lambda v: -len(touch[v]))
    ptr = {v: 0 for v in sites}
    n_share = int(round(share * len(edges)))
    share_set = set()
    hub_m = {}          # site -> count of share edges it drives (its hub reuse count)
    progress = True
    while len(share_set) < n_share and progress:
        progress = False
        for v in sites:
            if len(share_set) >= n_share:
                break
            lst = touch[v]
            while ptr[v] < len(lst) and lst[ptr[v]] in share_set:
                ptr[v] += 1
            if ptr[v] < len(lst):
                share_set.add(lst[ptr[v]])
                hub_m[v] = hub_m.get(v, 0) + 1
                ptr[v] += 1
                progress = True

    # prune-ring: drop a fraction of links entirely (the two rings just don't
    # meet there). Only from non-tree links -- pruning the backbone would split
    # the graph. Fewer links = sparser ring graph, the opposite of share.
    n_prune = int(round(prune_ring * len(edges)))
    prunable = [k for k in non_tree if k not in spof_set and k not in share_set]
    prune_set = set(prunable[:max(0, min(n_prune, len(prunable)))])

    # Tree edges processed first and always minimal, so a backbone glue is
    # never starved of its one required slot by an optional edge widening.
    order = sorted(edges, key=lambda e: tuple(sorted(e)) not in tree)

    def slot(i, reuse):
        # Returns (slot, kind). "reuse" = an existing hub anchor (no capacity
        # spent), "fresh" = a new even slot from free, "degrade" = fell back to
        # the hub anchor with no free slot left. Hub bookkeeping happens in the
        # loop, after it decides whether to accept a reuse.
        if (reuse and i in hub and (not hub_cap or hub_use.get(i, 0) < hub_cap)
                and hub[i] not in free[i]):
            return hub[i], "reuse"
        if free[i]:
            return free[i][-1], "fresh"
        return (hub[i], "degrade") if i in hub else (None, None)

    def even_span(i, base, shares):
        """The OTHER even slots a corridor at `base` covers on site i. shares<=2
        covers none (which is why a plain 1-edge glue never collides); every
        even slot is a glue anchor another link could still be handed."""
        return [(base + d) % size[i] for d in range(2, shares) if d % 2 == 0]

    for i, j in order:
        key = tuple(sorted((i, j)))
        if key in prune_set:            # dropped transition: never glued
            continue
        want_share = key in share_set
        a, ka = slot(i, want_share)
        b, kb = slot(j, want_share)
        if a is None or b is None:
            continue
        # A share link reusing a hub on BOTH ends would fuse the two hubs into
        # one node (collapsing both sites' rings onto it, chaining hub-to-hub).
        # Keep hubs as stars: at most one end reuses, the other opens a fresh slot.
        if ka == "reuse" and kb == "reuse" and free[j]:
            b, kb = free[j][-1], "fresh"
        elif ka == "reuse" and kb == "reuse" and free[i]:
            a, ka = free[i][-1], "fresh"
        for site, k in ((i, ka), (j, kb)):
            if k == "reuse":
                hub_use[site] = hub_use.get(site, 0) + 1
        if ka == "fresh":
            free[i].pop()
        if kb == "fresh":
            free[j].pop()
        # Only a share link creates a hub anchor. Recording one for EVERY glue
        # let a later share link reuse a slot a plain glue had already consumed,
        # fusing unrelated rings into a runaway mega-hub (k=45, degree 84).
        if want_share:
            for site, anchor, k in ((i, a, ka), (j, b, kb)):
                if k == "fresh" and hub.get(site) != anchor:  # fresh anchor: new hub
                    hub[site] = anchor
                    hub_use[site] = 1

        if key in spof_set:
            shares = 1
        elif want_share:
            # Hub reuse. shares=2 alone ISN'T safe: the OTHER rings already at
            # this hub would meet the new one only at the anchor node -- C(m,2)
            # single-node (SPOF) pairs. So also share the anchor's odd neighbour
            # slot across partners: every ring at the hub shares the whole EDGE
            # (a, a+1) pairwise, not just node a. Odd slots are corridor interior,
            # so this costs no capacity.
            shares = 2
        elif key in tree:
            shares = 2
        else:
            # Widen only while the even slots the corridor would cover are still
            # free on both sides. free[] holds even slots only, so a blind pop()
            # let a corridor overrun a neighbour's anchor and fuse two rings
            # (showed up as spurious spof even at share=0).
            shares = 2
            while shares < width_max + 1:
                want_i, want_j = even_span(i, a, shares + 1), even_span(j, b, shares + 1)
                if not (all(x in free[i] for x in want_i)
                        and all(y in free[j] for y in want_j)):
                    break
                shares += 1
            for x in even_span(i, a, shares):
                free[i].remove(x)
            for y in even_span(j, b, shares):
                free[j].remove(y)
        for x, y in _share_pairs(i, j, a, b, shares, size[i], size[j]):
            dsu.union(x, y)

    g = nx.Graph(); rings = []; coord = {}; site = []
    n_sites = len(pts)
    for i in sg.nodes():
        seen, cyc = set(), []
        for v in (dsu.find((i, j)) for j in range(size[i])):
            if v not in seen:
                seen.add(v); cyc.append(v)
        if len(cyc) < 3:
            continue
        rings.append(cyc)
        # Remember which site each ring came from: its coordinate is fixed by
        # (rings, seed), so the ring-graph panel can be drawn at the real site
        # positions instead of a spring layout that reshuffles on every change.
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


def _spof_final(g, rings, frac, rng):
    """Turn a fraction of the FINAL plan's ring pairs into SPOFs.

    Can't be done at glue time: the denominator (total ring pairs) isn't known
    until share has added its links. Here every pair exists, so the fraction is
    exact.

    A pair sharing m>=2 nodes is narrowed to 1 by SPLITTING m-1 of them -- the
    shared node becomes two, one left in each ring, each keeping that ring's
    neighbours. The guard below never splits the last shared node, so no link is
    destroyed, only narrowed -- the ring graph keeps its connectivity.
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


def _prune_nodes(g, rings, rpos, frac, rng):
    """Remove ~frac of nodes at random (keeping the graph connected) and DROP
    every ring that lost a member -- a loop minus a node isn't walkable, so it
    can't be a ring label. Survivors keep their other rings; any left in no ring
    are reachable only via the echo rings score() adds (or reported unroutable).
    Rings that lost nobody keep every edge. Node ids are re-indexed to 0..n-1
    afterwards so the topology writer sees a dense range."""
    n_target = int(round(frac * g.number_of_nodes()))
    removed = set()
    cand = list(g.nodes()); rng.shuffle(cand)
    for v in cand:
        if len(removed) >= n_target:
            break
        nbrs, attr = list(g[v]), dict(g.nodes[v])
        g.remove_node(v)
        if g.number_of_nodes() and nx.is_connected(g):
            removed.add(v)
        else:                                   # cut vertex: removal partitions
            g.add_node(v, **attr)
            g.add_edges_from((v, u) for u in nbrs)
    keep = [k for k, r in enumerate(rings) if not (set(r) & removed)]
    idx = {v: k for k, v in enumerate(sorted(g.nodes()))}
    h = nx.relabel_nodes(g, idx, copy=True)
    return (h, [[idx[v] for v in rings[k]] for k in keep], [rpos[k] for k in keep])


def _prune_solo(g, rings, frac, rng):
    """Splice out nodes that are in exactly ONE ring: they carry no ring-to-ring
    link and add nothing to fan-out, just make the loop longer. Removing one
    outright would break the loop, so its two ring neighbours are joined instead
    -- the ring stays a walkable loop, just shorter.

    Never taken below 3 members, and only degree-2 nodes are spliced (after
    --prune-node a node can lose its ring but keep edges; rewiring that isn't a
    splice).
    """
    memb = {}
    for r in rings:
        for v in r:
            memb[v] = memb.get(v, 0) + 1
    solo = [v for v in g.nodes() if memb.get(v, 0) == 1 and g.degree(v) == 2]
    rng.shuffle(solo)
    solo = set(solo[:int(round(frac * len(solo)))])

    out = []
    for r in rings:
        keep, i = list(r), 0
        while i < len(keep) and len(keep) > 3:
            v = keep[i]
            nbrs = list(g[v]) if v in solo and v in g else []
            if len(nbrs) == 2:
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
          spof=0.0, share=0.0, width_max=WIDTH_MAX, hub_cap=HUB_CAP,
          prune_ring=0.0, prune_node=0.0, ring_size_max=0, prune_solo=0.0, **kw):
    """A=rings (site/ring count). spof=fraction of links sharing 1 node.
    share=fraction reusing a hub slot to build 3+-ring nodes (the fan-out lever).
    hub_cap=links one anchor absorbs before a fresh node opens (lower = more,
    smaller hubs -> lower peak k). prune_ring=links dropped entirely (sparser).
    prune_node=nodes removed (may leave some ringless). prune_solo=single-ring
    filler nodes spliced out (0=keep all, the default, so node counts stay
    comparable with older runs).

    ring_size_max: if set, each site's ring is sized to its DEGREE (2 slots per
    link) within [ring_size, ring_size_max] -- busy sites get the big rings,
    leaf sites stay small, and nobody carries slots no link will use.
    ring_size_max also caps site degree (ring_size_max//2), so it is really a
    knob on how many links a site may host: raise it for more share
    opportunities, not for bigger rings."""
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
        # Exactly the slots this site's links need (2 per link), clamped to
        # [s, s_max]. Slack beyond that is pure filler: measured against a
        # random bump toward s_max, and against uniform +1/+2 spare slots, on
        # all four families -- fan-out reach was identical within seed noise
        # while the bump generated ~25-35% more single-ring nodes.
        size = {v: min(max(s, 2 * sg.degree(v)), s_max) for v in sg.nodes()}
    else:
        size = {v: s for v in sg.nodes()}
    g, plan, rpos = _glue_widths(sg, pts, size, rng, spof, share, width_max,
                                 hub_cap, prune_ring)
    if prune_node > 0:
        g, plan, rpos = _prune_nodes(g, plan, rpos, prune_node, rng)
    if prune_solo > 0:
        g, plan = _prune_solo(g, plan, prune_solo, rng)
    if spof > 0:
        # Last, so the fraction is measured against the pairs that actually
        # exist after share, pruning and splicing.
        g, plan = _spof_final(g, plan, spof, rng)
    return g, plan, rpos


def export_and_plot(family, rings, seed=0, ring_size=8,
                    spof=0.0, share=0.0, width_max=WIDTH_MAX, hub_cap=HUB_CAP,
                    prune_ring=0.0, prune_node=0.0, ring_size_max=0,
                    prune_solo=0.0, quiet=False):
    g, plan, rpos = build(family, rings, seed, ring_size, spof, share, width_max,
                          hub_cap, prune_ring, prune_node, ring_size_max, prune_solo)
    s = score(g, plan)          # raises if any ring isn't hop-walkable

    tag = ((f"_rs{ring_size}-{ring_size_max}" if ring_size_max else "")
           + (f"_pr{prune_ring:g}" if prune_ring else "")
           + (f"_pn{prune_node:g}" if prune_node else "")
           + (f"_ps{prune_solo:g}" if prune_solo else ""))
    stem = (f"ringpipe_{family}_A{rings}_spof{spof:g}_share{share:g}{tag}"
            f"_seed{seed}_fan{s['fan']}_n{s['n']}")
    # Group a run's three files under one subfolder keyed by family/A/spof, so a
    # sweep's outputs don't fan out flat across csv/ringlabels/gallery.
    sub = f"{family}_A{rings}_spof{spof:g}"
    # .native.ring marks a topology that SHIPS its ring plan (use --basis
    # native); the suffix makes that visible in a directory listing.
    os.makedirs(os.path.join(CSV_DIR, sub), exist_ok=True)
    os.makedirs(os.path.join(RINGLABELS_DIR, sub), exist_ok=True)
    topo = os.path.join(CSV_DIR, sub, f"{stem}.native.ring.txt")
    ini = os.path.join(RINGLABELS_DIR, sub, f"{stem}.native.ring.ini")
    exp.write_topology(g, topo,
        f"myriad_topologies/ringpipe.py family={family} rings(A)={rings} "
        f"spof_target={spof} share_target={share} width_max={width_max} seed={seed}")
    hdr = [f"ringLabel lines generated by myriad_topologies/ringpipe.py from {stem}.native.ring.txt",
           f"family={family} rings(A)={rings} spof_target={spof} share_target={share} width_max={width_max}",
           f"nodes={s['n']} edges={s['m']} avgdeg={s['avgdeg']:.2f} rings={s['rings']} echo={s['echo']}",
           f"fan-out(sum k*(k-1))={s['fan']} fan/n={s['fan_n']:.2f} max rings/node={s['maxk']}",
           f"transitions: {s['pairs']} ring pairs, {s['spof']} spof ({s['spofpct']:.1f}%)",
           f"ring-graph components={s['comp']} biconnected={s['biconn']} diameter={s['diam']}"]
    exp.write_ringlabels(g, plan, ini, hdr)

    member, n_echo = rings_to_membership(g, plan)
    n_rings = len(plan) + n_echo
    ring_of = {v: [t2r.RING_ID_BASE - i for i in sorted(member[v])] for v in g.nodes()}
    ring_ids = [t2r.RING_ID_BASE - i for i in range(n_rings)]
    stats = t2r.topology_stats(g, ring_of, n_rings, n_echo, 0,
        f"{stem}  fam={family} A={rings} spof_target={spof:g} share_target={share:g}")
    os.makedirs(os.path.join(GALLERY, sub), exist_ok=True)
    png = os.path.join(GALLERY, sub, f"{stem}.png")
    # Echo rings have no site, so they get no fixed position -- draw() falls
    # back to a spring layout for the whole panel if any ring is missing one.
    ring_pos = {t2r.RING_ID_BASE - i: p for i, p in enumerate(rpos)}
    t2r.draw(g, ring_of, ring_ids, png, stats, ring_pos)

    if not quiet:
        print(f"{family:9} A={rings:4} spof->{spof:4g} share->{share:4g} "
              f"n={s['n']:5} fan/n={s['fan_n']:6.2f} spof={s['spofpct']:5.1f}% "
              f"maxk={s['maxk']:2} diam={s['diam']:4}")
        print(f"  {topo}\n  {ini}\n  {png}")
    return s, topo, ini, png


def _fit_solo(target, tol, **kw):
    """Find the prune_solo that lands within `tol` of `target` nodes, by
    bisection. Sweeping --share alone changes n (sharing fuses nodes), so runs
    aren't comparable. Splicing solo filler changes n WITHOUT touching fan-out
    (see _prune_solo), so trade it against share to hold n fixed. n falls as
    prune_solo rises, hence plain bisection."""
    lo, hi = 0.0, 1.0
    n_hi = build(prune_solo=hi, **kw)[0].number_of_nodes()   # smallest reachable
    n_lo = build(prune_solo=lo, **kw)[0].number_of_nodes()   # largest reachable
    if not n_hi - tol <= target <= n_lo + tol:
        raise ValueError(f"target {target} outside reachable n [{n_hi}, {n_lo}]")
    best = (abs(n_lo - target), lo) if abs(n_lo - target) < abs(n_hi - target) \
        else (abs(n_hi - target), hi)
    for _ in range(20):
        if best[0] <= tol:
            break
        mid = (lo + hi) / 2
        n = build(prune_solo=mid, **kw)[0].number_of_nodes()
        best = min(best, (abs(n - target), mid))
        lo, hi = (mid, hi) if n > target else (lo, mid)
    return best[1], best[0]


AUTO_RING_SIZES = (6, 8, 10, 12, 14, 16, 20, 24, 32, 48)


def _fit_ring_size(target, tol, sizes=AUTO_RING_SIZES, **kw):
    """Pick the ring size (with its prune_solo) giving the MOST fan-out at
    ~target nodes.

    Ring size trades floor against fan-out, and the balance flips across a share
    sweep, so no single --ring-size is right for the whole thing. At share=0 it
    sets the reachable floor (~3.4*A nodes at 6, ~7.5*A at 16), which is what
    decides whether `target` is even reachable. At share=1 it sets how far hub
    reuse can concentrate ring memberships onto few nodes -- and fan is
    quadratic in k, so concentration, not membership count, is what pays."""
    best = None
    for rs in sizes:
        try:
            ps, err = _fit_solo(target, tol, ring_size=rs, **kw)
        except ValueError:
            continue                      # target outside this size's n range
        if err > tol:
            continue
        s = score(*build(ring_size=rs, prune_solo=ps, **kw)[:2])
        if abs(s["n"] - target) <= tol and (best is None or s["fan"] > best[0]):
            best = (s["fan"], rs, ps)
    if best is None:
        # At share=0 a small ring has almost no filler to trim, so each size
        # only spans a few percent of n -- and sizes step 6, 8, 10..., so the
        # bands have gaps a target can fall straight through. n scales with the
        # site count though, so report the --rings that would land on it.
        a = kw.get("rings") or 1
        bands = []
        for rs in sizes:
            lo = build(ring_size=rs, prune_solo=1.0, **kw)[0].number_of_nodes()
            hi = build(ring_size=rs, prune_solo=0.0, **kw)[0].number_of_nodes()
            if hi > lo:
                bands.append(f"--ring-size {rs} wants --rings "
                             f"{round(target * a / hi)}-{round(target * a / lo)}")
        raise ValueError(
            f"no ring size reaches n~{target} (+/-{tol}) at --rings {a}, "
            f"share={kw.get('share')}. n is pinned to the site count -- use one "
            f"of:\n    " + "\n    ".join(bands[:4])
            + f"\n  (or raise --target-tol, currently {tol})")
    return best[1], best[2]


def sweep_share(shares, target_n=0, tol=25, auto_ring_size=False, **kw):
    """Export one topology per share value, all at ~target_n nodes. Default
    target = the smallest unpruned n in the sweep (high share fuses nodes, so
    that's the ceiling); every other share is pruned DOWN to it.

    auto_ring_size: choose --ring-size per point for maximum fan-out spread
    instead of holding one size across the sweep (see _fit_ring_size). Default
    target then becomes the share=0 floor at the smallest size, which is the
    anchor that makes the span widest -- so only --rings and --target-n are left
    to pick."""
    if auto_ring_size:
        kw.pop("ring_size", None)
        kw.pop("ring_size_max", None)     # size is chosen per point now
        if not target_n:
            target_n = build(share=0.0, ring_size=AUTO_RING_SIZES[0],
                             prune_solo=1.0, **kw)[0].number_of_nodes()
            print(f"  target-n not given, using the share=0 floor: {target_n}")
        for sh in shares:
            rs, ps = _fit_ring_size(target_n, tol, share=sh, **kw)
            s, *_ = export_and_plot(ring_size=rs, share=sh, prune_solo=ps,
                                    quiet=True, **kw)
            print(f"share={sh:4g} ring_size={rs:3} prune_solo={ps:.3f} "
                  f"n={s['n']:5} fan={s['fan']:7} fan/n={s['fan_n']:6.2f} "
                  f"maxk={s['maxk']:3} spof={s['spofpct']:5.1f}%")
        return
    if not target_n:
        target_n = min(build(share=sh, prune_solo=0.0, **kw)[0].number_of_nodes()
                       for sh in shares)
    for sh in shares:
        ps, err = _fit_solo(target_n, tol, share=sh, **kw)
        s, *_ = export_and_plot(share=sh, prune_solo=ps, quiet=True, **kw)
        flag = "" if err <= tol else f"  (off by {err}, tol {tol})"
        print(f"share={sh:4g} prune_solo={ps:.3f} n={s['n']:5} "
              f"fan={s['fan']:6} fan/n={s['fan_n']:5.2f} "
              f"spof={s['spofpct']:5.1f}%{flag}")


def _selfcheck():
    for fam in SITE_FAMILIES:
        for spof in (0.0, 0.3):
            for share in (0.0, 0.3):
                g, plan, _ = build(fam, rings=30, seed=0, spof=spof, share=share)
                assert nx.is_connected(g), (fam, spof, share)
                s = score(g, plan)         # validates ring walkability
                assert s["n"] > 0
    # prune: connectivity must hold and every surviving ring stays valid
    # (score raises otherwise). prune-node also must actually shrink the graph.
    for pr, pn in ((0.3, 0.0), (0.0, 0.3), (0.2, 0.2)):
        g0 = build("delaunay", rings=40, seed=1)[0]
        g, plan, _ = build("delaunay", rings=40, seed=1, prune_ring=pr, prune_node=pn)
        assert nx.is_connected(g), (pr, pn)
        score(g, plan)
        assert g.number_of_nodes() < g0.number_of_nodes() if pn else True
    # varied ring sizes: rings must stay walkable (score checks) and actually
    # vary, and the extra slots must let a high share beat the fixed-size run.
    for fam in SITE_FAMILIES:
        g, plan, _ = build(fam, rings=40, seed=1, share=0.5, ring_size=8, ring_size_max=14)
        assert nx.is_connected(g), fam
        score(g, plan)
    g, plan, _ = build("delaunay", rings=40, seed=1, share=0.5, ring_size=8, ring_size_max=14)
    assert len({len(r) for r in plan}) > 1, "ring sizes did not vary"
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
                    help="target fraction of transitions reusing a hub slot -> 3+-ring nodes (fan-out lever)")
    ap.add_argument("--width-max", type=int, default=WIDTH_MAX,
                    help="cap on shared nodes for a non-spof, non-hub, non-backbone transition")
    ap.add_argument("--hub-cap", type=int, default=HUB_CAP,
                    help="share links one hub anchor absorbs before a fresh node "
                         "opens (lower = hubs spread over more nodes, lower peak k; "
                         "0 = unlimited)")
    ap.add_argument("--prune-ring", type=float, default=0.0,
                    help="fraction of links to drop entirely (non-backbone only, "
                         "so the graph stays connected)")
    ap.add_argument("--prune-node", type=float, default=0.0,
                    help="fraction of nodes to remove (staying connected); every "
                         "ring a removed node was in is dropped -- may leave nodes ringless")
    ap.add_argument("--prune-solo", type=float, default=0.0, metavar="FRAC",
                    help="fraction of single-ring nodes to splice out of their ring "
                         "(they add no fan-out, only lap length). 0 = off, 1 = strip all")
    ap.add_argument("--ring-size", type=int, default=8,
                    help="ring size (even, >=6); the minimum when --ring-size-max is set")
    ap.add_argument("--ring-size-max", type=int, default=0,
                    help="if set, ring sizes vary in [--ring-size, this], biggest "
                         "rings to the busiest sites (more slots -> higher --share "
                         "fits; 0 = all rings equal)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--sweep-share", nargs="+", type=float, metavar="F",
                    help="export one topology per share value, each fitted to "
                         "--target-n nodes via --prune-solo (which moves n without "
                         "moving fan-out), so runs differ in fan-out only")
    ap.add_argument("--target-n", type=int, default=0,
                    help="node count to hit in --sweep-share (0 = smallest n the "
                         "sweep can all reach)")
    ap.add_argument("--target-tol", type=int, default=25, help="allowed node-count error")
    ap.add_argument("--auto-ring-size", action="store_true",
                    help="in --sweep-share: pick --ring-size per point for the "
                         "widest fan-out span (small rings at low share keep n "
                         "reachable, big ones at high share concentrate rings "
                         "onto hubs). Leaves only --rings and --target-n to pick")
    ap.add_argument("--selfcheck", action="store_true")
    a = ap.parse_args()
    if a.selfcheck:
        return _selfcheck()
    if a.sweep_share:
        kw = dict(family=a.family, rings=a.rings, seed=a.seed,
                  ring_size=a.ring_size, spof=a.spof, width_max=a.width_max,
                  hub_cap=a.hub_cap, prune_ring=a.prune_ring,
                  prune_node=a.prune_node, ring_size_max=a.ring_size_max)
        return sweep_share(a.sweep_share, a.target_n, a.target_tol,
                           auto_ring_size=a.auto_ring_size, **kw)
    export_and_plot(a.family, a.rings, a.seed, a.ring_size, a.spof, a.share,
                    a.width_max, a.hub_cap, a.prune_ring, a.prune_node,
                    a.ring_size_max, a.prune_solo)


if __name__ == "__main__":
    main()
