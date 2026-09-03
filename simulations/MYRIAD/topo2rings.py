#!/usr/bin/env python3
# Derive MyriadGP ring memberships from an EXISTING routingsim topology .txt
#   python3 topo2rings.py ../topologies/grid_10,10.txt

import argparse
import os
import networkx as nx
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches

from generate_ring_membership import get_ring_per_node

# Ring ids = RING_ID_BASE - ring_index (distinguishability)
RING_ID_BASE = 1_000_000
# Above this the pie-per-node drawing is unreadable and spring_layout is slow.
NODE_PNG_CAP = 250
# One shared gallery, so a sweep's pictures land together instead of scattered
# beside each label file (where every eval caller passes --no-png anyway).
PNG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "eval", "topology_pngs")


def read_topology(path):
    g = nx.Graph()
    header_seen = False
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if not header_seen:
                header_seen = True
                continue
            parts = line.split()
            if len(parts) >= 2 and parts[0] != "node":
                g.add_edge(int(parts[0]), int(parts[1]))
    return g


def topology_stats(g, ring_of, n_rings, n_echo, n_dropped, label):
    """Ring-decomposition stats, shared by the .ini header and the PNG caption
    so the two can never disagree. Beyond plain sizes these are the quantities
    MyriadGP's cost actually turns on:

    spof transitions -- ring pairs bridged by exactly ONE node. That router is a
      single point of failure for every chain crossing there, and it is what
      rankUseSpof ranks against; a decomposition with many of them forces the
      spof-free wave onto long detours.
    fan-out -- a node in k rings relays each learned chain into k-1 others, so
      sum k*(k-1) is the inter-ring propagation work the topology implies.
    ring-graph components -- rings linked by shared members. More than one
      component means chains cannot cross between them: partitioned routing.
    """
    nodes = sorted(g.nodes())
    per_ring = {}
    for n in nodes:
        for rid in ring_of[n]:
            per_ring.setdefault(rid, []).append(n)
    memb = [len(ring_of[n]) for n in nodes]
    empty = [n for n in nodes if not ring_of[n]]

    # Bridges between ring pairs, keyed by the pair; 1 member = SPOF transition.
    bridges = {}
    for n in nodes:
        rs = sorted(ring_of[n])
        for i in range(len(rs)):
            for j in range(i + 1, len(rs)):
                bridges.setdefault((rs[i], rs[j]), 0)
                bridges[(rs[i], rs[j])] += 1
    n_spof = sum(1 for c in bridges.values() if c == 1)

    # Ring graph connectivity: rings as nodes, shared members as edges.
    rg = nx.Graph()
    rg.add_nodes_from(per_ring)
    rg.add_edges_from(bridges)
    n_comp = nx.number_connected_components(rg) if rg.number_of_nodes() else 0

    fanout = sum(k * (k - 1) for k in memb)
    hist = sorted((len(v) for v in per_ring.values()), reverse=True)

    stats = [label,
             f"nodes={g.number_of_nodes()} edges={g.number_of_edges()} "
             f"rings={n_rings} (echo={n_echo}, dropped-oversize={n_dropped})"]
    if per_ring:
        stats += [
            f"ring size: max={hist[0]} mean={sum(hist)/len(hist):.2f} top10={hist[:10]}",
            f"rings/node: max={max(memb)} mean={sum(memb)/len(nodes):.2f} "
            f"fan-out(sum k*(k-1))={fanout}",
            f"transitions: {len(bridges)} ring pairs, {n_spof} spof "
            f"({100.0*n_spof/len(bridges):.1f}%) | ring-graph components={n_comp}",
        ]
    if n_comp > 1:
        stats.append(f"WARNING: ring graph in {n_comp} components — routing is partitioned")
    if empty:
        stats.append(f"WARNING: {len(empty)} node(s) in no ring (unroutable): "
                     f"{empty[:10]}{' ...' if len(empty) > 10 else ''}")
    return stats


def draw(g, ring_of, ring_ids, path, stats=()):
    # Each node is a pie of the colors of every ring it belongs to.
    n_rings = len(ring_ids)
    cmap = plt.get_cmap("hsv")
    color = {rid: cmap(i / max(1, n_rings)) for i, rid in enumerate(ring_ids)}

    pos = nx.spring_layout(g, seed=1, k=0.5, iterations=200)
    fig, ax = plt.subplots(figsize=(14, 14))
    nx.draw_networkx_edges(g, pos, ax=ax, alpha=0.3)

    r = 0.035
    for n in g.nodes():
        rings = ring_of[n]
        x, y = pos[n]
        if not rings:  # node in no cycle (bridge/leaf) -- draw hollow
            ax.add_patch(matplotlib.patches.Circle((x, y), r, facecolor="white",
                         edgecolor="black", linewidth=0.5, zorder=2))
        else:
            step = 360.0 / len(rings)
            for i, rid in enumerate(rings):
                ax.add_patch(matplotlib.patches.Wedge(
                    (x, y), r, i * step, (i + 1) * step,
                    facecolor=color[rid], edgecolor="black", linewidth=0.5, zorder=2))
        ax.text(x, y, str(n), ha="center", va="center", fontsize=7, zorder=3)
        
    # Monospace + left-aligned: the stats are columnar, so proportional font
    # and centering make them unreadable.
    ax.set_title(f"{os.path.basename(path)}  (multi-color = multi-ring node)\n"
                 + "\n".join(stats),
                 fontsize=9, family="monospace", loc="left")
    ax.set_aspect("equal")
    ax.axis("off")
    ax.autoscale_view()
    plt.tight_layout()
    plt.savefig(path, dpi=100)
    plt.close()


def main():
    ap = argparse.ArgumentParser(description="Ring memberships + viz from a routingsim .txt")
    ap.add_argument("topology", help="path to a routingsim self-defined .txt")
    ap.add_argument("--out-dir", default=os.path.dirname(os.path.abspath(__file__)),
                    help="where to write .ringlabels.ini / .png (default: next to this script)")
    ap.add_argument("--out", default=None,
                    help="exact output path for .ringlabels.ini (overrides --out-dir naming)")
    ap.add_argument("--no-png", action="store_true", help="skip the drawing")
    ap.add_argument("--basis", choices=["min", "tree", "short", "all"], default="tree",
                    help="ring basis: tree=cycle_basis (spanning-tree, default, big rings), "
                         "min=minimum_cycle_basis (small, slow), "
                         "short=greedy shortest-cycle-per-edge (small, cheap); "
                         "'all' is a legacy alias for tree")
    ap.add_argument("--max-ring-size", type=int, default=20,
                    help="drop rings with more than this many members (0 = keep all); "
                         "members of a dropped ring fall back to echo rings")
    ap.add_argument("--min-ring-size", type=int, default=0,
                    help="basis=short only: target floor on ring size. The cover "
                         "prefers rings >= this (after coverage), allowing smaller "
                         "outliers where no bigger local cycle exists. 0 = no floor.")
    ap.add_argument("--no-echo", action="store_true",
                    help="omit 2-node echo rings; may leave nodes in no ring at all")
    args = ap.parse_args()

    g = read_topology(args.topology)
    stem = os.path.splitext(os.path.basename(args.topology))[0]

    # runs the generate_ring_membership.py by daniel
    rings_idx, n_echo, n_dropped = get_ring_per_node(
        g, args.basis, max_ring_size=args.max_ring_size, echo=not args.no_echo,
        min_ring_size=args.min_ring_size)
    n_rings = max((max(v) + 1 for v in rings_idx.values() if v), default=0)
    def ring_id(idx):
        return RING_ID_BASE - idx
    ring_of = {n: [ring_id(i) for i in rings_idx[n]] for n in g.nodes()}
    ring_ids = [ring_id(i) for i in range(n_rings)]

    nodes = sorted(g.nodes())
    if args.out:
        ini = args.out
    else:
        ini = os.path.join(args.out_dir, f"{stem}.{args.basis}.ringlabels.ini")
    stats = topology_stats(
        g, ring_of, n_rings, n_echo, n_dropped,
        f"topology: {stem}.txt  basis={args.basis} "
        f"ring-size={args.min_ring_size or 0}..{args.max_ring_size or 'unlimited'} "
        f"echo={not args.no_echo}")

    with open(ini, "w") as f:
        f.write(f"# ringLabel lines generated by topo2rings.py from {stem}.txt\n")
        for s in stats:
            f.write(f"# {s}\n")
        for n in nodes:
            labels = ",".join(str(rid) for rid in ring_of[n])
            f.write(f'**.Node{n}.*.ringLabel = "{labels}"\n')

    for s in stats:
        print(f"  {s}")
    print(f"  wrote {ini}")

    if args.no_png or g.number_of_nodes() > NODE_PNG_CAP:
        return
    # Basis and cap in the name: the same topology under different ring
    # settings is a different picture, and they must not overwrite each other.
    tag = f"{args.basis}.m{args.max_ring_size}" if args.max_ring_size else args.basis
    if args.min_ring_size:
        tag += f".lo{args.min_ring_size}"
    os.makedirs(PNG_DIR, exist_ok=True)
    png = os.path.join(PNG_DIR, f"{stem}.{tag}.png")
    draw(g, ring_of, ring_ids, png, stats)
    print(f"  wrote {png}")


if __name__ == "__main__":
    main()
