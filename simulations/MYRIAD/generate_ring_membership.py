import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path
import networkx as nx

# Bounded cycles examined per uncovered edge before picking the best. The DFS is
# lazy, so this bounds the work; higher = less redundant cover, slower.
CANDIDATES = 500

def bounded_cycle_cover(graph, max_ring_size=0, min_ring_size=0):
    """Greedy ring cover. For each still-uncovered edge, sample the first
    CANDIDATES simple cycles through it with <= max_ring_size nodes (drop the
    edge, DFS with cutoff, re-add) and keep the best by:
        (new edges covered, fewest new SPOF transitions, more safe overlaps,
         size in [min,max], smaller size)
    Coverage is the primary objective (keeps memberships/fan-out down); the SPOF
    term is the tiebreak UNDER it. A SPOF transition is a pair of rings sharing
    exactly one node -- lose that node and the two rings disconnect. Among
    equal-coverage candidates we prefer the cycle that touches existing rings on
    >=2 nodes (a `safe` 2-connected transition) over one that grazes a ring at a
    single vertex, so the cover trims single-node bridges without inflating ring
    count. Measured on an RGG this cuts SPOF pairs ~53%->37% at ~0 fan-out cost;
    making SPOF the PRIMARY objective cuts further (~11%) but explodes fan-out
    6x (a swarm of small redundant rings), so it stays a tiebreak.
    The size range is a preference layered under both; most rings land in
    [min,max] while outliers are still allowed both ways:
      - below min: only when no sampled cycle reaches min (e.g. a triangle is
        the only local cycle) -- taken because coverage still wins;
      - above max: only when NO cycle <= max exists at all, via the shortest-
        cycle fallback (the flower's inner 10-ring under a cap of 5).
    So [min,max] is a target band, not a hard bound. min=0 disables the floor
    (identical to the plain <=max behaviour). Not a basis; cheaper than
    minimum_cycle_basis and, unlike cycle_basis, only exceeds the band where the
    graph forces it."""
    cap = max_ring_size or graph.number_of_nodes()
    lo = min_ring_size
    ekey = lambda a, b: (a, b) if a <= b else (b, a)
    cycles, covered = [], set()
    member = defaultdict(set)   # node -> indices of cycles it's in (SPOF scoring)
    for u, v in list(graph.edges()):
        if ekey(u, v) in covered:
            continue
        graph.remove_edge(u, v)
        # Best of the first CANDIDATES cycles through (u,v). shortest_simple_paths
        # (Yen) yields in increasing length and is polynomial per path, so we can
        # walk up to the cap without all_simple_paths' exponential backtracking
        # (a cutoff of 9 on a degree-8 graph took minutes). Stop at the first
        # path over the cap -- being length-ordered, none after it fits either.
        best, best_key = None, None
        if nx.has_path(graph, u, v):
            for i, p in enumerate(nx.shortest_simple_paths(graph, u, v)):
                if len(p) > cap:
                    break
                gain = sum(ekey(p[j], p[(j + 1) % len(p)]) not in covered
                           for j in range(len(p)))
                # SPOF score: count how many existing rings this cycle would
                # share exactly one node with (a single-node bridge = SPOF) vs
                # >=2 nodes (a redundant, safe transition). Prefer few SPOFs.
                shared = Counter()
                for nd in p:
                    shared.update(member[nd])
                spof = sum(1 for c in shared.values() if c == 1)
                safe = sum(1 for c in shared.values() if c >= 2)
                shortfall = max(0, lo - len(p))   # 0 once size >= lo (<= cap already)
                key = (gain, -spof, safe, -shortfall, -len(p))
                if best_key is None or key > best_key:
                    best, best_key = p, key
                if i + 1 >= CANDIDATES:
                    break
        path = best
        # No cycle fits the cap: fall back to the shortest cycle (an above-max
        # outlier) rather than surrendering the edge to a per-edge echo ring.
        if not path and nx.has_path(graph, u, v):
            path = nx.shortest_path(graph, u, v)
        graph.add_edge(u, v)
        if not path:
            continue
        ridx = len(cycles)
        cycles.append(path)
        for i in range(len(path)):
            covered.add(ekey(path[i], path[(i + 1) % len(path)]))
        for nd in path:
            member[nd].add(ridx)
    return cycles


def get_ring_per_node(graph, basis="tree", max_ring_size=0, echo=True,
                      min_ring_size=0):
    # basis: "min" = minimum_cycle_basis (shortest independent cycles, slow);
    # "tree" = cycle_basis (spanning-tree fundamental basis, cheap, but yields
    # mostly LARGE rings); "short" = greedy bounded-cycle cover targeting the
    # [min_ring_size, max_ring_size] band (see bounded_cycle_cover), no
    # minimum_cycle_basis cost. "all" is a legacy alias for "tree".
    if basis == "short":
        cycles = bounded_cycle_cover(graph, max_ring_size, min_ring_size)
    else:
        cycles = (nx.minimum_cycle_basis(graph) if basis == "min"
                  else nx.cycle_basis(graph))
    # A long cycle costs every member a COLLECT lap over the whole ring and adds
    # itself to each member's inter-ring fan-out, so a handful of oversized rings
    # dominates control-plane cost. Dropping them keeps the short local rings and
    # leaves the discarded members covered by echo rings (below) instead.
    # Not for "short": it already honoured the cap per edge and only exceeds it
    # where the graph left no alternative, so dropping here would undo the
    # fallback and hand those edges right back to echo rings.
    n_dropped = 0
    if max_ring_size and basis != "short":
        keep = [c for c in cycles if len(c) <= max_ring_size]
        n_dropped = len(cycles) - len(keep)
        cycles = keep
    rings_per_node = {}
    current_ring_id = 0
    for node in graph.nodes():
        rings_per_node[node] = []
    for cycle in cycles:
        for node in cycle:
            rings_per_node[node].append(current_ring_id)
        current_ring_id += 1
    # Echo rings: every edge whose endpoints share no cycle ring becomes a 2-node
    # ring, so bridges/leaves/tree parts are covered and the ring graph stays
    # connected wherever the node graph is (each such ring is closed by echo).
    # Chords and intra-cycle edges self-skip (endpoints already share a ring).
    n_echo = 0
    if echo:
        for u, v in graph.edges():
            if set(rings_per_node[u]).isdisjoint(rings_per_node[v]):
                rings_per_node[u].append(current_ring_id)
                rings_per_node[v].append(current_ring_id)
                current_ring_id += 1
                n_echo += 1
    return rings_per_node, n_echo, n_dropped


def ring_coverage(rings_per_node):
    """Nodes in no ring at all. Without echo rings a pruned basis can strand a
    node entirely — it has no ring to announce itself on, so it is unroutable."""
    return [n for n, r in rings_per_node.items() if not r]

def read_self_defined_topo(filename):
    graph = nx.Graph()
    with open(filename) as f:
        while f.readline().startswith('#'):
            next(f)
        next(f) # skip number of nodes and edges
        for line in f:
            print(line)
            graph.add_edge(line.split()[0], line.split()[1])
    return graph

def main():
    parser = argparse.ArgumentParser(description='Read graphml file and generate ring memberships')
    parser.add_argument('-i', '--input', type=str, help='input file')
    parser.add_argument('-o', '--output', type=str, help='output file', default="")
    parser.add_argument('-t', '--type', type=str, help='type of input file', choices=['gml', 'sd'])
    args = parser.parse_args()

    input_file = Path(args.input)
    output_file = args.output
    if not input_file.is_file():
        print("Input file", input_file ," not found!")
        return
    if args.type == "gml":
        graph = nx.read_gml(input_file, label="id")
    elif args.type == "sd":
        graph = read_self_defined_topo(input_file)
    else:
        print("Invalid input type: ", args.type)
        return
    rings_per_node, n_echo, _ = get_ring_per_node(graph)
    if output_file == "":
        output_file = args.input + "_ring_memberships.json"
    with open(output_file, 'w') as outfile:
        json.dump(rings_per_node, outfile, indent=4, sort_keys=True)

if __name__ == '__main__':
    main()