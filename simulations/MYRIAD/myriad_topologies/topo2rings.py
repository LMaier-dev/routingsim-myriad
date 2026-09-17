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
from metrics import score

# Ring ids = RING_ID_BASE - ring_index (distinguishability)
RING_ID_BASE = 1_000_000
# Echo (two-node) rings are numbered from their own block instead, so a router
# reads echo-ness off the id and never has to infer it from its neighbour table
# — which it cannot do after a reboot, when a ring-neighbour that stayed down is
# indistinguishable from a ring that only ever had two members. Keep in step
# with MyriadGp.ned echoRingIdMax. Echo rings exist for evaluation topologies
# only; a deployment ring plan has none.
ECHO_ID_BASE = 900_000


def ring_id(idx, n_planned):
    """Ring index -> id. Planned rings descend from RING_ID_BASE, echo rings
    (every index from n_planned on, which is how the membership builders order
    them) descend from ECHO_ID_BASE, so `id <= ECHO_ID_BASE` means echo ring."""
    return (RING_ID_BASE - idx if idx < n_planned
            else ECHO_ID_BASE - (idx - n_planned))
# Above this the pie-per-node drawing is unreadable and spring_layout is slow.
NODE_PNG_CAP = 250
# One shared gallery, so a sweep's pictures land together instead of scattered
# beside each label file (where every eval caller passes --no-png anyway).
PNG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gallery")


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
    """Ring-decomposition stats for the .ini header and the PNG caption (so the
    two never disagree). The quantities MyriadGP's cost turns on -- fan-out,
    spof transitions, ring-graph components -- are computed by metrics.score;
    see it for what each means. ring_of already carries the echo rings, so score
    must not add its own; echo rings are 2-node and unordered here, so the
    walkability check is skipped."""
    nodes = sorted(g.nodes())
    per_ring = {}
    for n in nodes:
        for rid in ring_of[n]:
            per_ring.setdefault(rid, []).append(n)
    empty = [n for n in nodes if not ring_of[n]]

    s = score(g, list(per_ring.values()), echo=False, check=False)
    hist = sorted((len(v) for v in per_ring.values()), reverse=True)

    stats = [label,
             f"nodes={g.number_of_nodes()} edges={g.number_of_edges()} "
             f"rings={n_rings} (echo={n_echo}, dropped-oversize={n_dropped})"]
    if per_ring:
        stats += [
            f"ring size: max={hist[0]} mean={sum(hist)/len(hist):.2f} top10={hist[:10]}",
            f"rings/node: max={s['maxk']} mean={s['meank']:.2f} "
            f"fan-out(sum k*(k-1))={s['fan']}",
            f"transitions: {s['pairs']} ring pairs, {s['spof']} spof "
            f"({s['spofpct']:.1f}%) | ring-graph components={s['comp']}",
        ]
    if s["comp"] > 1:
        stats.append(f"WARNING: ring graph in {s['comp']} components — routing is partitioned")
    if empty:
        stats.append(f"WARNING: {len(empty)} node(s) in no ring (unroutable): "
                     f"{empty[:10]}{' ...' if len(empty) > 10 else ''}")
    return stats


def draw(g, ring_of, ring_ids, path, stats=(), ring_pos=None):
    # Each node is a pie of the colors of every ring it belongs to.
    n_rings = len(ring_ids)
    cmap = plt.get_cmap("hsv")
    color = {rid: cmap(i / max(1, n_rings)) for i, rid in enumerate(ring_ids)}

    # Two panels: the ring graph on the left (rings as nodes, shared members as
    # edges -- this is the graph MyriadGP's inter-ring propagation actually runs
    # on) and the node graph on the right.
    fig, (axr, ax) = plt.subplots(1, 2, figsize=(26, 14),
                                  gridspec_kw={"width_ratios": [1, 1.3]})

    # Same reasoning as the ring panel: if the graph carries its own geometry
    # (ringpipe sets a 'pos' per node from the site layout), draw there so the
    # picture is stable across runs; otherwise fall back to a spring solve.
    if all("pos" in g.nodes[n] for n in g.nodes()):
        pos = {n: g.nodes[n]["pos"] for n in g.nodes()}
    else:
        pos = nx.spring_layout(g, seed=1, k=0.5, iterations=200)
    nx.draw_networkx_edges(g, pos, ax=ax, alpha=0.3)

    # Radius follows the layout's extent (spring spans ~[-1,1], site coords
    # ~[0,1]), else the pies are twice too big on a supplied geometry.
    span = max(max(x for x, _ in pos.values()) - min(x for x, _ in pos.values()),
               max(y for _, y in pos.values()) - min(y for _, y in pos.values()), 1e-9)
    r = 0.0175 * span
    # Size by ring membership: a single-ring node is plain ring filler carrying
    # no transition, so it is drawn small and unlabelled -- at scale those are
    # most of the nodes and at full size they bury the multi-ring nodes that
    # actually decide fan-out. k>=2 keeps the full radius.
    for n in g.nodes():
        rings = ring_of[n]
        x, y = pos[n]
        rn = r if len(rings) >= 2 else r * 0.35
        if not rings:  # node in no cycle (bridge/leaf) -- draw hollow
            ax.add_patch(matplotlib.patches.Circle((x, y), rn, facecolor="white",
                         edgecolor="black", linewidth=0.5, zorder=2))
        else:
            step = 360.0 / len(rings)
            for i, rid in enumerate(rings):
                ax.add_patch(matplotlib.patches.Wedge(
                    (x, y), rn, i * step, (i + 1) * step,
                    facecolor=color[rid], edgecolor="black", linewidth=0.5,
                    zorder=2 if len(rings) < 2 else 4))
        if len(rings) >= 2:
            ax.text(x, y, str(n), ha="center", va="center", fontsize=7, zorder=5)

    _draw_ring_graph(axr, ring_of, color, ring_pos)

    # Monospace + left-aligned: the stats are columnar, so proportional font
    # and centering make them unreadable.
    ax.set_title(f"{os.path.basename(path)}  (multi-color = multi-ring node)\n"
                 + "\n".join(stats),
                 fontsize=9, family="monospace", loc="left")
    ax.set_aspect("equal")
    ax.axis("off")
    ax.autoscale_view()
    plt.tight_layout()
    plt.savefig(path, dpi=200)
    plt.close()


def _draw_ring_graph(ax, ring_of, color, ring_pos=None):
    """The ring graph: one node per ring (drawn in that ring's color), one edge
    per ring pair that shares a member. RED edge = shares exactly one node (a
    SPOF transition -- lose that node and the two rings can't reach each other);
    grey = shares >=2 (a redundant, safe transition). This is the structure that
    determines fan-out and single-point-of-failure risk, so it is worth seeing
    directly rather than only as summary counts.

    ring_pos: optional {ring id -> (x, y)}. Ring plans built from sited rings
    (myriad_topologies/ringpipe.py) know where each ring belongs, and passing that in
    pins the panel so two runs are comparable by eye. Without it the layout is
    a spring solve, which re-converges differently on any structural change --
    the same ring lands somewhere else and nothing lines up between pictures."""
    from collections import Counter
    shared = Counter()
    for rs in ring_of.values():
        rs = sorted(set(rs))
        for i in range(len(rs)):
            for j in range(i + 1, len(rs)):
                shared[(rs[i], rs[j])] += 1
    rg = nx.Graph()
    for rs in ring_of.values():
        rg.add_nodes_from(rs)
    rg.add_edges_from(shared)
    if rg.number_of_nodes() == 0:
        ax.axis("off"); return
    if ring_pos and all(n in ring_pos for n in rg.nodes()):
        pos = {n: ring_pos[n] for n in rg.nodes()}
    else:
        pos = nx.spring_layout(rg, seed=1, k=0.8, iterations=200)
    spof = [e for e in rg.edges() if shared[tuple(sorted(e))] == 1]
    safe = [e for e in rg.edges() if shared[tuple(sorted(e))] >= 2]
    # Edge width = how many nodes bridge the two rings (the "width" of the
    # transition): a 1-node bridge is a hairline SPOF, a shared 3-node corridor
    # is a fat, hard-to-sever link.
    sw = [1.0 + 1.3 * (shared[tuple(sorted(e))] - 2) for e in safe]
    nx.draw_networkx_edges(rg, pos, ax=ax, edgelist=safe, edge_color="0.5", width=sw)
    nx.draw_networkx_edges(rg, pos, ax=ax, edgelist=spof, edge_color="red", width=1.5)
    nx.draw_networkx_nodes(rg, pos, ax=ax, node_size=90,
                           node_color=[color.get(n, "grey") for n in rg.nodes()],
                           edgecolors="black", linewidths=0.5)
    n_spof = len(spof)
    n_comp = nx.number_connected_components(rg)
    ax.set_title(f"ring graph: {rg.number_of_nodes()} rings, {rg.number_of_edges()} "
                 f"transitions, {n_spof} spof (red), {n_comp} component(s)",
                 fontsize=9, family="monospace", loc="left")
    ax.set_aspect("equal")
    ax.axis("off")


def main():
    ap = argparse.ArgumentParser(description="Ring memberships + viz from a routingsim .txt")
    ap.add_argument("topology", help="path to a routingsim self-defined .txt")
    ap.add_argument("--out-dir", default=os.path.join(
                        os.path.dirname(os.path.abspath(__file__)), "ringlabels"),
                    help="where to write .ring.ini / .png (default: ./ringlabels)")
    ap.add_argument("--out", default=None,
                    help="exact output path for .ring.ini (overrides --out-dir naming)")
    ap.add_argument("--no-png", action="store_true", help="skip the drawing")
    ap.add_argument("--basis", choices=["min", "tree", "all"], default="tree",
                    help="ring basis: tree=cycle_basis (spanning-tree, default, big rings), "
                         "min=minimum_cycle_basis (small, slow); "
                         "'all' is a legacy alias for tree")
    ap.add_argument("--max-ring-size", type=int, default=20,
                    help="drop rings with more than this many members (0 = keep all); "
                         "members of a dropped ring fall back to echo rings")
    ap.add_argument("--no-echo", action="store_true",
                    help="omit 2-node echo rings; may leave nodes in no ring at all")
    args = ap.parse_args()

    g = read_topology(args.topology)
    stem = os.path.splitext(os.path.basename(args.topology))[0]

    # runs the generate_ring_membership.py by daniel
    rings_idx, n_echo, n_dropped = get_ring_per_node(
        g, args.basis, max_ring_size=args.max_ring_size, echo=not args.no_echo)
    n_rings = max((max(v) + 1 for v in rings_idx.values() if v), default=0)
    # get_ring_per_node numbers the cycle rings first and appends the echo ones.
    n_planned = n_rings - n_echo
    ring_of = {n: [ring_id(i, n_planned) for i in rings_idx[n]] for n in g.nodes()}
    ring_ids = [ring_id(i, n_planned) for i in range(n_rings)]

    nodes = sorted(g.nodes())
    if args.out:
        ini = args.out
    else:
        ini = os.path.join(args.out_dir, f"{stem}.{args.basis}.ring.ini")
    stats = topology_stats(
        g, ring_of, n_rings, n_echo, n_dropped,
        f"topology: {stem}.txt  basis={args.basis} "
        f"max-ring-size={args.max_ring_size or 'unlimited'} "
        f"echo={not args.no_echo}")

    os.makedirs(os.path.dirname(ini) or ".", exist_ok=True)
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
    os.makedirs(PNG_DIR, exist_ok=True)
    png = os.path.join(PNG_DIR, f"{stem}.{tag}.png")
    draw(g, ring_of, ring_ids, png, stats)
    print(f"  wrote {png}")


if __name__ == "__main__":
    main()
