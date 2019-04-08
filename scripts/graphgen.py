#!/usr/bin/env python3
"""
Tool to generate topologies for RoutingSim
"""
import argparse
import sys
from datetime import datetime
import itertools
import concurrent.futures

import networkx


def main():
    parser = argparse.ArgumentParser(description='Tool to generate topologies for RoutingSim')
    parser.add_argument('--seed', type=int, help='seed for randomness', default=0)
    parser.add_argument('-n', '--nostats', action='store_true', help='dont generatop stats')
    parser.add_argument('-d', '--dir', help='directory for output file', default='.')
    subparsers = parser.add_subparsers(
        help='choose the graph you want to generate',
        dest='type')
    parser_plc = subparsers.add_parser('plc', help='power law cluster graph (Holme-Kim)')
    parser_plc.add_argument('N', type=int, help='network size')
    parser_plc.add_argument('M', type=int, help='added edges per node')
    parser_plc.add_argument('C', type=float, help='clustering parameter')

    parser_cft = subparsers.add_parser('cft', help='clos network fat tree')
    parser_cft.add_argument('K', type=int, help='node degree')
    parser_cft.add_argument('--hosts', help='include hosts (degree 1)', action='store_true')

    parser_plc = subparsers.add_parser('tree', help='perfectly balance R-tree with height H')
    parser_plc.add_argument('R', type=int, help='tree degree')
    parser_plc.add_argument('H', type=int, help='tree height')

    opt = parser.parse_args()

    print("generating graph...")
    if(opt.type == 'plc'):
        G = networkx.powerlaw_cluster_graph(opt.N, opt.M, opt.C, seed=opt.seed)
        G.graph['info'] = [
            ["", f"generated with NetworkX {networkx.__version__}, networkx.powerlaw_cluster_graph"],
            ["", f"n = {opt.N}; m = {opt.M}; clustering param = {opt.C}; seed = {opt.seed}"],
            ["time", datetime.now()]]
        G.graph['filename'] = f"powerlaw_cluster_graph_n{opt.N}_m{opt.M}_p{opt.C}_seed{opt.seed}.txt"
    elif(opt.type == 'cft'):
        G = clos_fat_tree(opt.K, opt.hosts)
        G.graph['info'] = [
            ["", f"generated with NetworkX {networkx.__version__}"],
            ["", f"fat tree clos network, k = {opt.K}, with{'out' if not opt.hosts else ''} hosts"],
            ["", f"\t{(opt.K//2) **2} core nodes, {opt.K} pods with {opt.K} nodes each"],
            ["", f"\tmakes {(opt.K//2)**2 + opt.K**2} nodes and up to {opt.K**3 // 4} hosts"],
            ["time", datetime.now()]]

        G.graph['filename'] = f"clos_fat_tree_{opt.K}{'_with_hosts' if opt.hosts else ''}.txt"
    elif(opt.type == 'tree'):
        G = networkx.balanced_tree(opt.R, opt.H)
        G.graph['info'] = [
            ["", f"generated with NetworkX {networkx.__version__}, networkx.balanced tree"],
            ["", f"r = {opt.R}; h = {opt.H}"],
            ["time", datetime.now()]]

        G.graph['filename'] = f"tree_r{opt.R}_h{opt.H}.txt"
    else:
        exit()
    if not opt.nostats:
        print("generating stats...")
        add_stats(G, seed=opt.seed)
    else:
        print("skipping stats.")
    print_info(G, sys.stdout)
    print("writing file...")
    path = f"{opt.dir}/{G.graph['filename']}"
    print(f"path: {path}")
    write_graph(G, path)



def write_graph(G, filename=None):
    if filename is None:
        filename = G.graph['filename']
    with open(filename, 'w') as f:
        print_info(G, f)
        print_graph(G, f)

def print_graph(G, stream):
    stream.write('{}\t{}\n'.format(len(G.nodes), len(G.edges)))
    for a, b in G.edges:
        stream.write(f'{a}\t{b}\n{b}\t{a}\n')

def print_info(G, stream):
    for k, v in G.graph.get('info', []):
        print(f"# {k}: {v}", file=stream)

def add_stats(G, seed=0):
    assert(networkx.is_connected(G))
    exe = concurrent.futures.ProcessPoolExecutor()
    ecc = exe.submit(networkx.eccentricity, G)
    ac = lambda g: networkx.average_clustering(g)
    f1 = [['radius', exe.submit(networkx.radius, G, ecc.result())],
        ['diameter', exe.submit(networkx.diameter, G, ecc.result())],
        ['average clustering', exe.submit(networkx.average_clustering, G)],
        ['average shortest path length (sampled)', 
        exe.submit(sampling_aspl, G, seed=seed)]]
    if 'info' not in G.graph:
        G.graph['info'] = []
    stats =  G.graph['info']
    stats.extend([
        ['size', networkx.number_of_nodes(G)],
        ['edges', networkx.number_of_edges(G)],
        ['connected', True],
        ['average degree', networkx.number_of_edges(G)*2 / networkx.number_of_nodes(G)],
        ['density', networkx.density(G)]])
    stats.extend([k, v.result()] for k, v in f1)

@networkx.utils.py_random_state(2)
def sampling_aspl(G, count=1000, seed=None):
    return sum(networkx.shortest_path_length(G, *seed.sample(G.nodes(), 2))
               for _ in range(count)) / count

def my_range(start, length):
    return range(start, start+length)

def clos_fat_tree(k, hosts=False):
    assert k % 2  == 0
    # we have:
    # - (k//2) ** 2 core nodes
    # - k pods with k nodes each
    G = networkx.empty_graph(k**2 + (k//2)**2)
    # pod node id is: pod*k + node
    # core node id is (with x,y core coordinates): k**2 + x*k//2 + y
    core_offset = k**2

    # initialize connections inside pods
    for i in range(0, k**2, k):
        G.add_edges_from(itertools.product(
            my_range(i, k//2),
            my_range(i + k//2, k//2)))
    # initialize connections from pods to core
    for i in range(k//2):
        G.add_edges_from(itertools.product(
            my_range(core_offset + k//2*i, k//2),
            range(i, k**2, k)))
    if hosts:
        # initialize host components
        host_offset = G.size()
        G.add_nodes_from(my_range(host_offset, k**3 // 4), type='host')
        for pod in range(k):
            for node in range(k//2):
                node_id = pod*k + k//2 + node
                assert(len(G.edges(node_id)) == k//2)
                for host in range(k//2):
                    G.add_edge(node_id, host_offset + pod*k**2//4 + node*k//2 + host)
                assert(len(G.edges(node_id)) == k)
    return G


if __name__ == '__main__':
    main()
