import argparse
import json
from pathlib import Path
import networkx as nx

def get_ring_per_node(graph):
    # cycles = nx.cycle_basis(graph)
    # cycle_basis returns a *fundamental* basis (one cycle per non-tree edge via
    # its spanning-tree path), so cycles can be very long and heavily overlap:
    # on a 10x10 grid it puts up to 13 of 81 rings on a single hub node, whose
    # COLLECT/UPDATE traffic then explodes (6.6M UPDATEs by t=4s in MyriadGP).
    # minimum_cycle_basis returns the shortest independent cycles instead — for a
    # grid these are the unit squares (every ring length 4, every node in <=4),
    # which is the topology's natural ring structure and keeps the sim tractable.
    cycles = nx.minimum_cycle_basis(graph)
    rings_per_node = {}
    current_ring_id = 0
    for node in graph.nodes():
        rings_per_node[node] = []
    for cycle in cycles:
        for node in cycle:
            rings_per_node[node].append(current_ring_id)
        current_ring_id += 1
    return rings_per_node

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
    rings_per_node = get_ring_per_node(graph)
    if output_file == "":
        output_file = args.input + "_ring_memberships.json"
    with open(output_file, 'w') as outfile:
        json.dump(rings_per_node, outfile, indent=4, sort_keys=True)

if __name__ == '__main__':
    main()