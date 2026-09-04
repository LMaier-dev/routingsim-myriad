#!/usr/bin/env python3
"""Topology families for a geographically spread-out energy-grid network:
flat, road-like, no giant hubs, always in one connected piece.

Each builder returns (graph, rings). Here rings is always None -- meaning "find
the rings later" (metrics.py does that). Families that SHIP their own ring plan
live in ringpipe.py instead.

Goal: low fan-out (sum k*(k-1) over each node's rings), so every family is
deliberately sparse.
"""
import math
import numpy as np
import networkx as nx
from scipy.spatial import Delaunay, cKDTree

# ---------------------------------------------------------------- helpers

def _giant(g):
    g = g.subgraph(max(nx.connected_components(g), key=len)).copy()
    return nx.convert_node_labels_to_integers(g, label_attribute="old")

def _delaunay_edges(pts):
    tri = Delaunay(pts)
    e = set()
    for s in tri.simplices:
        for a, b in ((s[0], s[1]), (s[1], s[2]), (s[0], s[2])):
            e.add((a, b) if a < b else (b, a))
    return e

def _attach_pos(g, pts):
    # Keep coordinates: the .txt writer emits them so downstream tooling can
    # draw a geographic layout instead of a spring layout.
    for n in g.nodes():
        old = g.nodes[n].get("old", n)
        g.nodes[n]["pos"] = tuple(float(x) for x in pts[old])
    return g

def _trim_to(g, n_target):
    """Shrink to n_target without disconnecting: repeatedly drop the lowest-degree
    non-articulation node."""
    while g.number_of_nodes() > n_target:
        arts = set(nx.articulation_points(g))
        cand = [v for v in sorted(g, key=lambda x: g.degree(x)) if v not in arts]
        if not cand:
            break
        g.remove_node(cand[0])
    return nx.convert_node_labels_to_integers(g, label_attribute="old2")

# ---------------------------------------------------------------- helpers shared with ringpipe

def _check_ring_size(s):
    """Ring size must be EVEN and >= 6. Glue points sit on the even slots
    (0,2,..,s-2), and only even s>=6 leaves room for them:
      odd s -- slots 0 and s-1 touch, so two glue points land on ONE node and
               that ring-to-ring link becomes a SPOF (~32% measured at s=5);
      s = 4 -- only 2 glue slots, too few for a 3-neighbour site (network splits).
    Both are outright breakage, not tuning knobs, so reject them.
    """
    if s < 6 or s % 2:
        raise ValueError(f"ring_size must be even and >= 6, got {s}")
    return s


def _cap_degree(sg, max_deg):
    """A connected spanning subgraph of `sg` with every node <= max_deg edges.

    Built UP, not torn down: start from a spanning tree and add spare edges
    (shortest first) while both ends still have room. Tearing edges out of a
    dense graph can't keep both the cap AND connectivity -- it used to quietly
    ship split graphs.
    """
    if max_deg < 2:
        raise ValueError(f"max_deg {max_deg} < 2 cannot span; use ring_size >= 6")
    mst = nx.minimum_spanning_tree(sg, weight="w")
    out = nx.Graph(); out.add_nodes_from(sg.nodes())
    # A tree can already exceed the cap at a hub, so keep a tree edge only while
    # both ends have room, then repair whatever that left disconnected.
    for u, v, d in sorted(mst.edges(data=True), key=lambda e: e[2]["w"]):
        if out.degree(u) < max_deg and out.degree(v) < max_deg:
            out.add_edge(u, v, **d)
    # Repair: reconnect leftover pieces with the shortest legal edge; if both
    # ends are full, use any edge to a node with room -- staying connected beats
    # the slot budget.
    while nx.number_connected_components(out) > 1:
        comps = list(nx.connected_components(out))
        home = {v: i for i, c in enumerate(comps) for v in c}
        best = None
        for u, v, d in sg.edges(data=True):
            if home[u] == home[v]:
                continue
            legal = out.degree(u) < max_deg and out.degree(v) < max_deg
            key = (not legal, d["w"])
            if best is None or key < best[0]:
                best = (key, u, v, d)
        if best is None:                       # sg itself was disconnected
            for i in range(len(comps) - 1):
                out.add_edge(min(comps[i]), min(comps[i + 1]), w=1.0)
            break
        _, u, v, d = best
        out.add_edge(u, v, **d)
    for u, v, d in sorted(sg.edges(data=True), key=lambda e: e[2]["w"]):
        if not out.has_edge(u, v) and out.degree(u) < max_deg and out.degree(v) < max_deg:
            out.add_edge(u, v, **d)            # spare capacity -> extra redundancy
    return out

class _DSU:
    def __init__(self): self.p = {}
    def find(self, x):
        self.p.setdefault(x, x)
        while self.p[x] != x:
            self.p[x] = self.p[self.p[x]]; x = self.p[x]
        return x
    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb: self.p[rb] = ra

# ---------------------------------------------------------------- families

def gabriel(n, seed=0, **kw):
    """Gabriel graph: edge uv iff the disk with diameter uv is empty. Planar,
    degree ~4, a standard road-network model and much sparser than Delaunay."""
    pts = np.random.default_rng(seed).random((n, 2))
    tree = cKDTree(pts)
    g = nx.Graph(); g.add_nodes_from(range(n))
    for u, v in _delaunay_edges(pts):
        mid = (pts[u] + pts[v]) / 2.0
        r = float(np.linalg.norm(pts[u] - pts[v])) / 2.0
        # 1e-9 slack: u and v sit exactly on the circle.
        if not [w for w in tree.query_ball_point(mid, r - 1e-9) if w not in (u, v)]:
            g.add_edge(u, v)
    return _attach_pos(_giant(g), pts), None

def rgg(n_target, seed=0, avg_degree=4.5, **kw):
    for over in (1.0, 1.3, 1.7, 2.2, 3.0, 4.0):
        n = int(n_target * over)
        r = math.sqrt(avg_degree / (math.pi * n))
        g = nx.random_geometric_graph(n, r, seed=seed)
        g = _giant(g)
        if g.number_of_nodes() >= n_target:
            return _trim_to(g, n_target), None
    raise SystemExit("rgg: unreachable size")

def delaunay(n, seed=0, **kw):
    pts = np.random.default_rng(seed).random((n, 2))
    g = nx.Graph(); g.add_nodes_from(range(n))
    g.add_edges_from(_delaunay_edges(pts))
    return _attach_pos(_giant(g), pts), None

FAMILIES = {
    "gabriel":  gabriel,
    "rgg":      rgg,
    "delaunay": delaunay,
}


def _selfcheck():
    """Every family must yield a connected graph. Native-plan validation lives
    with the families that ship one (ringpipe.py); these are all mined."""
    for name, fn in FAMILIES.items():
        for n in (120, 400):
            g, rings = fn(n, seed=0)
            assert g.number_of_nodes() > 0, name
            assert nx.is_connected(g), f"{name} n={n}: disconnected"
            assert min(d for _, d in g.degree()) >= 1, f"{name}: isolated node"
            assert rings is None, f"{name}: expected mined (rings=None)"
    # ring_size guard still rejects unsound sizes (used by ringpipe).
    for bad in (3, 4, 5, 7):
        try:
            _check_ring_size(bad)
        except ValueError:
            continue
        raise AssertionError(f"_check_ring_size({bad}) should raise")
    print("topo_lib selfcheck ok")


if __name__ == "__main__":
    _selfcheck()
