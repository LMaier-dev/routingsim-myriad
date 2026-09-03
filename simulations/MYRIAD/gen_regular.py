#!/usr/bin/env python3
"""Generate random d-regular graph topologies in routingsim .txt format.

Every node has exactly `--degree` neighbors — no geometry, no hubs. This is the
degree-controlled null model: ring load stays flat because degree is uniform,
so it isolates what MyriadGP does when fan-out is homogeneous (contrast RGG's
density-driven load and Delaunay's climbing hub load).

    python3 gen_regular.py 750 --degree 6

Tune later via --degree / --seed. Requires n*degree even.
"""
import argparse
import datetime
import networkx as nx

def regular(n, degree, seed):
    if (n * degree) % 2:
        raise SystemExit(f"n*degree must be even (n={n}, degree={degree})")
    # d-regular graphs are a.s. connected for d>=3 but not guaranteed; a bad
    # draw is rare, so just walk seeds until one connects.
    for s in range(seed, seed + 100):
        g = nx.random_regular_graph(degree, n, seed=s)
        if nx.is_connected(g):
            return nx.convert_node_labels_to_integers(g)
    raise SystemExit(f"no connected {degree}-regular graph on {n} nodes in 100 seeds")

def write(g, path, degree, seed):
    n, m = g.number_of_nodes(), g.number_of_edges()
    sp = nx.average_shortest_path_length(g) if n <= 2000 else None
    with open(path, "w") as f:
        f.write("# : generated with NetworkX, networkx.random_regular_graph\n")
        f.write(f"# : n = {n}; degree = {degree}; seed = {seed}\n")
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
    ap.add_argument("--degree", type=int, default=6)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    g = regular(a.n, a.degree, a.seed)
    out = a.out or f"regular_n{g.number_of_nodes()}_d{a.degree}_seed{a.seed}.txt"
    write(g, out, a.degree, a.seed)
    print(f"{out}: V={g.number_of_nodes()} E={g.number_of_edges()} "
          f"avgdeg={2*g.number_of_edges()/g.number_of_nodes():.2f}")

if __name__ == "__main__":
    main()
