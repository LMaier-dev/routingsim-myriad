#!/usr/bin/env python3
"""Generate random geometric topologies in routingsim .txt format.

RGG keeps mean rings/node flat (~5) as the network grows, unlike powerlaw or
Delaunay where it climbs with n and a hub ends up on 150+ rings — which is what
drives MyriadGP's inter-ring fan-out. Radius is solved for a target average
degree so density, not size, sets the ring load.

    python3 gen_rgg.py 750 --avg-degree 4.5
"""
import argparse
import datetime
import math
import random
import networkx as nx


def rgg(n_target, avg_degree, seed):
    """Giant component of an RGG with `n_target` nodes at ~avg_degree.

    Oversamples: near the connectivity threshold a chunk of the points fall in
    small components, so generating exactly n_target would return far fewer.
    """
    for over in (1.0, 1.3, 1.7, 2.2, 3.0, 4.0):
        n = int(n_target * over)
        r = math.sqrt(avg_degree / (math.pi * n))
        g = nx.random_geometric_graph(n, r, seed=seed)
        g = g.subgraph(max(nx.connected_components(g), key=len)).copy()
        if g.number_of_nodes() >= n_target:
            # Trim to size, keeping the graph connected: drop the node whose
            # removal isn't a cut vertex, repeatedly.
            while g.number_of_nodes() > n_target:
                arts = set(nx.articulation_points(g))
                cand = [x for x in sorted(g, key=lambda v: g.degree(v)) if x not in arts]
                if not cand:
                    break
                g.remove_node(cand[0])
            return nx.convert_node_labels_to_integers(g)
    raise SystemExit(f"could not reach {n_target} connected nodes at degree {avg_degree}")


def write(g, path, avg_degree, seed):
    n, m = g.number_of_nodes(), g.number_of_edges()
    sp = nx.average_shortest_path_length(g) if n <= 2000 else None
    with open(path, "w") as f:
        f.write("# : generated with NetworkX, networkx.random_geometric_graph"
                " (giant component)\n")
        f.write(f"# : n = {n}; target average degree = {avg_degree}; seed = {seed}\n")
        f.write(f"# time: {datetime.datetime.now()}\n")
        f.write(f"# size: {n}\n# edges: {m}\n# connected: True\n")
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
    ap.add_argument("--avg-degree", type=float, default=4.5)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    g = rgg(a.n, a.avg_degree, a.seed)
    out = a.out or f"rgg_n{g.number_of_nodes()}_d{a.avg_degree:g}_seed{a.seed}.txt"
    write(g, out, a.avg_degree, a.seed)
    print(f"{out}: V={g.number_of_nodes()} E={g.number_of_edges()} "
          f"avgdeg={2*g.number_of_edges()/g.number_of_nodes():.2f}")


if __name__ == "__main__":
    main()
