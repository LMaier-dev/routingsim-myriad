import argparse
import json
from pathlib import Path
import networkx as nx

def get_ring_per_node(graph, basis="tree", max_ring_size=0, echo=True):
    # basis: "min" = minimum_cycle_basis (shortest independent cycles, slow);
    # "tree" = cycle_basis (spanning-tree fundamental basis, cheap). Same ring
    # COUNT either way (the cyclomatic number); they differ only in ring shape.
    # "all" accepted as a legacy alias for "tree".
    cycles = (nx.minimum_cycle_basis(graph) if basis == "min"
              else nx.cycle_basis(graph))
    # A long cycle costs every member a COLLECT lap over the whole ring and adds
    # itself to each member's inter-ring fan-out, so a handful of oversized rings
    # dominates control-plane cost. Dropping them keeps the short local rings and
    # leaves the discarded members covered by echo rings (below) instead.
    n_dropped = 0
    if max_ring_size:
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