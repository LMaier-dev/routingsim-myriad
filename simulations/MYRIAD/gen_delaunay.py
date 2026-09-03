#!/usr/bin/env python3
"""Generate Delaunay triangulation topologies in routingsim .txt format.

Unlike RGG (mean rings/node flat as n grows), Delaunay's mean degree is fixed
near 6 but the ring load climbs with n and hubs land on many rings — which is
what stresses MyriadGP's inter-ring fan-out. n points are drawn, triangulated,
and every simplex edge becomes a graph edge. Result is always connected.

    python3 gen_delaunay.py 750
    python3 gen_delaunay.py 750 --distribution clustered --clusters 8

Tune later via --distribution (point placement) / --clusters / --seed.
"""
import argparse
import datetime
import numpy as np
import networkx as nx
from scipy.spatial import Delaunay

def points(n, distribution, clusters, rng):
    if distribution == "uniform":
        return rng.random((n, 2))
    if distribution == "clustered":
        # Gaussian blobs → uneven density → high-degree hubs where blobs meet.
        centers = rng.random((clusters, 2))
        idx = rng.integers(0, clusters, n)
        return np.clip(centers[idx] + rng.normal(0, 0.08, (n, 2)), 0, 1)
    raise SystemExit(f"unknown distribution: {distribution}")

def delaunay(n, distribution, clusters, seed):
    rng = np.random.default_rng(seed)
    tri = Delaunay(points(n, distribution, clusters, rng))
    g = nx.Graph()
    g.add_nodes_from(range(n))
    for s in tri.simplices:                     # each triangle: 3 undirected edges
        g.add_edges_from(((s[0], s[1]), (s[1], s[2]), (s[0], s[2])))
    # clustered points can coincide after clipping and get dropped from the
    # triangulation, leaving isolated nodes — keep the giant component.
    g = g.subgraph(max(nx.connected_components(g), key=len)).copy()
    return nx.convert_node_labels_to_integers(g)

def write(g, path, distribution, seed):
    n, m = g.number_of_nodes(), g.number_of_edges()
    sp = nx.average_shortest_path_length(g) if n <= 2000 else None
    with open(path, "w") as f:
        f.write("# : generated with SciPy scipy.spatial.Delaunay\n")
        f.write(f"# : n = {n}; distribution = {distribution}; seed = {seed}\n")
        f.write(f"# time: {datetime.datetime.now()}\n")
        f.write(f"# size: {n}\n# edges: {m}\n# connected: {nx.is_connected(g)}\n")
        f.write(f"# average degree: {2 * m / n}\n")
        f.write(f"# density: {nx.density(g)}\n")
        f.write(f"# radius: {nx.radius(g)}\n# diameter: {nx.diameter(g)}\n")
        f.write(f"# average clustering: {nx.average_clustering(g)}\n")
        f.write(f"# average shortest path length: {sp if sp else 'n/a'}\n")
        f.write(f"{n}\t{m}\n")
        for u, v in g.edges():          # both directions, as the other topologies do
            f.write(f"{u}\t{v}\n{v}\t{u}\n")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("n", type=int)
    ap.add_argument("--distribution", choices=("uniform", "clustered"), default="uniform")
    ap.add_argument("--clusters", type=int, default=6)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    g = delaunay(a.n, a.distribution, a.clusters, a.seed)
    out = a.out or f"delaunay_n{g.number_of_nodes()}_{a.distribution}_seed{a.seed}.txt"
    write(g, out, a.distribution, a.seed)
    print(f"{out}: V={g.number_of_nodes()} E={g.number_of_edges()} "
          f"avgdeg={2*g.number_of_edges()/g.number_of_nodes():.2f}")

if __name__ == "__main__":
    main()
