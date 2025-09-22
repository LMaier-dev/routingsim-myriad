#!/usr/bin/env python3
"""
Tool to generate topologies for RoutingSim
"""
import argparse
import sys
from datetime import datetime
import itertools
import concurrent.futures
import os
import pathlib
import json
from random import seed as rd_seed, choice, choices, sample

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

    parser_plc = subparsers.add_parser('ws', help='connected watts strogatz graph')
    parser_plc.add_argument('N', type=int, help='network size')
    parser_plc.add_argument('K', type=int, help='each node is joined with its k nearest neighbors in a ring topology.')
    parser_plc.add_argument('P', type=float, help='The probability of rewiring each edge')

    parser_rg = subparsers.add_parser('rg', help='random geometric graph')
    parser_rg.add_argument('N', type=int, help='network size')
    parser_rg.add_argument('R', type=float, help='radius')

    parser_cft = subparsers.add_parser('cft', help='clos network fat tree')
    parser_cft.add_argument('K', type=int, help='node degree')
    parser_cft.add_argument('--hosts', help='include hosts (degree 1)', action='store_true')

    parser_plc = subparsers.add_parser('tree', help='perfectly balance R-tree with height H')
    parser_plc.add_argument('R', type=int, help='tree degree')
    parser_plc.add_argument('H', type=int, help='tree height')

    parser_plc = subparsers.add_parser('gnp', help='Erdos Reni Model G(n,p)')
    parser_plc.add_argument('N', type=int, help='number or nodes')
    parser_plc.add_argument('P', type=int, help='edge probability')

    parser_plc = subparsers.add_parser('gnm', help='G(n,m) model: n nodes, m edges')
    parser_plc.add_argument('N', type=int, help='number or nodes')
    parser_plc.add_argument('M', type=int, help='number of edges')

    parser_plc = subparsers.add_parser('gnd', help='G(n,m) model: n nodes, d*n/2 edges')
    parser_plc.add_argument('N', type=int, help='number or nodes')
    parser_plc.add_argument('D', type=int, help='average degree')

    parser_plc = subparsers.add_parser('grid', help='grid graph with given dimensions')
    parser_plc.add_argument('dim', help='dimensions, e.g. 1,2 or 4,8,9')

    parser_plc = subparsers.add_parser('hexgrid', help='hexgrid graph with given dimensions')
    parser_plc.add_argument('X', type=int, help='')
    parser_plc.add_argument('Y', type=int, help='')

    parser_f16 = subparsers.add_parser('f16', help='facebooks f16 fabric topology: https://engineering.fb.com/2019/03/14/data-center-engineering/f16-minipack/')
    parser_f16.add_argument('S', type=int, choices=(range(1,117)), help='how many slices of ToR switches (max 116)')
    parser_f16.add_argument('T', type=int, choices=(range(1, 113)), help='how many ToRs per slice (max 112)')
    parser_f16.add_argument('I', type=int, choices=(range(0, 113)), help='how many instances per ToR (max 112)')

    parser_plc = subparsers.add_parser('graphml', help='read graphml file')
    parser_plc.add_argument('PATH', help='path to graphml file')

    gml_plc = subparsers.add_parser('gml', help='read graphml file')
    gml_plc.add_argument('PATH', help='path to gml file')

    parser_plc_cc = subparsers.add_parser('plc_cc', help='two power law cluster graphs (Holme-Kim), connected with D edges')
    parser_plc_cc.add_argument('N', type=int, help='number of nodes')
    parser_plc_cc.add_argument('M', type=int, help='power law: added edges per node')
    parser_plc_cc.add_argument('C', type=float, help='power law: clustering parameter')
    parser_plc_cc.add_argument("D", type=int, help="number of connections between clusters", default=1)

    parser_fcc = subparsers.add_parser('fcc', help='full mesh clusters with sparse interconnect')
    parser_fcc.add_argument('N', type=int, help='number of nodes')
    parser_fcc.add_argument('C', type=int, help='power law: clustering parameter')

    opt = parser.parse_args()

    print("generating graph...")
    # RANDOM MODELS
    if(opt.type == 'plc'):
        G = networkx.powerlaw_cluster_graph(opt.N, opt.M, opt.C, seed=opt.seed)
        G.graph['info'] = [
            ["", f"generated with NetworkX {networkx.__version__}, networkx.powerlaw_cluster_graph"],
            ["", f"n = {opt.N}; m = {opt.M}; clustering param = {opt.C}; seed = {opt.seed}"],
            ["time", datetime.now()]]
        G.graph['filename'] = f"powerlaw_cluster_graph_n{opt.N}_m{opt.M}_p{opt.C}_seed{opt.seed}.txt"
    elif(opt.type == 'ws'):
        G = networkx.connected_watts_strogatz_graph(opt.N, opt.K, opt.P, seed=opt.seed)
        G.graph['info'] = [
            ["", f"generated with NetworkX {networkx.__version__}, networkx.connected_watts_strogatz_graph"],
            ["", f"n = {opt.N}; k = {opt.K}; p = {opt.P}; seed = {opt.seed}"],
            ["time", datetime.now()]]
        G.graph['filename'] = f"watts_strogatz_graph_n{opt.N}_k{opt.K}_p{opt.P}_seed{opt.seed}.txt"
    elif(opt.type == 'rg'):
        G = networkx.random_geometric_graph(opt.N, opt.R, seed=opt.seed)
        if (networkx.is_connected(G) == False):
            complement = list(networkx.k_edge_augmentation(G,k=1))
            if (len(complement) > 0):
                G.add_edges_from(complement)
                print(f"augmenting graph by {len(complement)} edges to let it be connected")
        G.graph['info'] = [
            ["", f"generated with NetworkX {networkx.__version__}, networkx.random_geometric_graph"],
            ["", f"n = {opt.N}; r = {opt.R}; seed = {opt.seed}"],
            ["time", datetime.now()]]
        G.graph['filename'] = f"random_geometric_n{opt.N}_r{opt.R}_seed{opt.seed}.txt"
    elif(opt.type == 'gnp'):
        G = networkx.fast_gnp_random_graph(opt.N, opt.P, seed=opt.seed)
        if (networkx.is_connected(G) == False):
            complement = list(networkx.k_edge_augmentation(G,k=1))
            if (len(complement) > 0):
                G.add_edges_from(complement)
                print(f"augmenting graph by {len(complement)} edges to let it be connected")
        G.graph['info'] = [
            ["", f"generated with NetworkX {networkx.__version__}, networkx.fast_gnp_random_graph"],
            ["", f"n = {opt.N}; p = {opt.P}; seed = {opt.seed}"],
            ["time", datetime.now()]]
        G.graph['filename'] = f"gnp_n{opt.N}_m{opt.P}_seed{opt.seed}.txt"
    elif(opt.type == 'gnm'):
        G = networkx.gnm_random_graph(opt.N, opt.M, seed=opt.seed)
        if (networkx.is_connected(G) == False):
            complement = list(networkx.k_edge_augmentation(G,k=1))
            if (len(complement) > 0):
                G.add_edges_from(complement)
                print(f"augmenting graph by {len(complement)} edges to let it be connected")
        G.graph['info'] = [
            ["", f"generated with NetworkX {networkx.__version__}, networkx.gnm_random_graph"],
            ["", f"n = {opt.N}; m = {opt.M}; seed = {opt.seed}"],
            ["time", datetime.now()]]
        G.graph['filename'] = f"gnm_n{opt.N}_m{opt.M}_seed{opt.seed}.txt"
    elif(opt.type == 'gnd'):
        G = networkx.gnm_random_graph(opt.N, opt.D * opt.N/2, seed=opt.seed)
        if (networkx.is_connected(G) == False):
            complement = list(networkx.k_edge_augmentation(G,k=1))
            if (len(complement) > 0):
                G.add_edges_from(complement)
                print(f"augmenting graph by {len(complement)} edges to let it be connected")
        G.graph['info'] = [
            ["", f"generated with NetworkX {networkx.__version__}, networkx.gnm_random_graph"],
            ["", f"n = {opt.N}; m = {opt.D * opt.N/2}; seed = {opt.seed}"],
            ["time", datetime.now()]]
        G.graph['filename'] = f"gnd_n{opt.N}_d{opt.D}_seed{opt.seed}.txt"
    elif(opt.type == 'plc_cc'):
        G, edge_str = generate_plc_cc(opt.N, opt.M, opt.C, opt.D, seed=opt.seed)
        G.graph['info'] = [
            ["", f"generated with NetworkX {networkx.__version__}, networkx.powerlaw_cluster_graph"],
            ["", f"n = {opt.N}; m = {opt.M}; clustering param = {opt.C}; connections between clusters = {opt.D}; seed = {opt.seed}"],
            ["", f"Connecting edges: {edge_str}"],
            ["time", datetime.now()]]
        G.graph['filename'] = f"connected_powerlaw_cluster_graphs_n{opt.N}_m{opt.M}_p{opt.C}_d{opt.D}_seed{opt.seed}.txt"

    elif(opt.type == 'fcc'):
        G = generate_plc_fcc(opt.N, opt.C, seed=opt.seed)
        G.graph['info'] = [
            ["time", datetime.now()]]
        G.graph['filename'] = f"full_mesh_clusters_sparse_interconnect_n{opt.N}_cs{opt.C}_seed{opt.seed}.txt"

    # NON RANDOM MODELS
    elif(opt.type == 'cft'):
        G = clos_fat_tree(opt.K, opt.hosts)
        G.graph['info'] = [
            ["", f"generated with NetworkX {networkx.__version__}"],
            ["", f"fat tree clos network, k = {opt.K}, with{'out' if not opt.hosts else ''} hosts"],
            ["", f"\t{(opt.K//2) **2} core nodes, {opt.K} pods with {opt.K} nodes each"],
            ["", f"\tmakes {(opt.K//2)**2 + opt.K**2} nodes and up to {opt.K**3 // 4} hosts"],
            ["time", datetime.now()]]
        G.graph['filename'] = f"clos_fat_tree_{opt.K}_with{'out' if not opt.hosts else ''}_hosts.txt"
    elif(opt.type == 'tree'):
        G = networkx.balanced_tree(opt.R, opt.H)
        G.graph['info'] = [
            ["", f"generated with NetworkX {networkx.__version__}, networkx.balanced_tree"],
            ["", f"r = {opt.R}; h = {opt.H}"],
            ["time", datetime.now()]]
        G.graph['filename'] = f"tree_r{opt.R}_h{opt.H}.txt"
    elif(opt.type == 'grid'):
        dim = json.loads(f"[{opt.dim}]")
        G = networkx.grid_graph(dim)
        G = networkx.convert_node_labels_to_integers(G)
        G.graph['info'] = [
            ["", f"generated with NetworkX {networkx.__version__}, networkx.grid_graph"],
            ["", f"dimensions = {dim}"],
            ["time", datetime.now()]]
        G.graph['filename'] = f"grid_{','.join(str(x) for x in dim)}.txt"
    elif(opt.type == 'hexgrid'):
        G = networkx.Graph()
        x=opt.X
        y=opt.Y
        for i in range(x):
            for j in range(y):
                if i < x-1:
                    G.add_edge(i+x*j,i+x*j+1)
                if j < y-1:
                    G.add_edge(i+x*j,i+x*j+x)
                if i < x-1 and j < y-1:
                    G.add_edge(i+x*j,i+x*j+x+1)
        G = networkx.convert_node_labels_to_integers(G)
        G.graph['info'] = [["time", datetime.now()]]
        G.graph['filename'] = f"f16_{','.join(str(a) for a in [x,y])}.txt"
    elif(opt.type == 'f16'):
        G = networkx.Graph()
        slices=opt.S
        tors=opt.T
        instances=opt.I
        #Spine switches
        for i in range(256):
            G.add_node(i)

        for slice in range(slices):
            # fabric switches connected to 16 spine switches each
            for i in range(16):
                for j in range(16):
                    G.add_edge(256+slice*16+i, 16*i + j)
            for tor in range(tors):
                for i in range(16):
                    G.add_edge(256+slices*16+slice*tors+tor, 256+slice*16+i) 
                for instance in range(instances):
                    G.add_edge(256+slices*16+slice*tors+tor, 256+slices*16+slices*tors+slice*tors*instances+tor*instances+instance) 
        G = networkx.convert_node_labels_to_integers(G)
        G.graph['info'] = [["time", datetime.now()],
            ["", f"f16 with 256 spine switches in 16 planes"],
            ["", f"\t{16*slices} fabric switches in {slices} slices"],
            ["", f"\t{tors*slices} tors"],
            ["", f"\t{tors*slices*instances} instances"],
            ["", f"In total {len(G.nodes)} nodes and {len(G.edges)} edges."]]
        G.graph['filename'] = f"f16_slices{slices}_tors{tors}_instances{instances}.txt"
    # CONVERSION
    elif(opt.type == 'graphml'):
        G = networkx.Graph(networkx.read_graphml(opt.PATH, node_type=int))
        p = pathlib.Path(opt.PATH)
        G.graph['info'] = [
            ["", f"converted with NetworkX {networkx.__version__}, networkx.read_graphml"],
            ["", f"input file: {p}"],
            ["time", datetime.now()]]
        G.graph['filename'] = f"{p.name}.txt"
    elif(opt.type == 'gml'):
        G = networkx.Graph(networkx.read_gml(opt.PATH))
        p = pathlib.Path(opt.PATH)
        G.graph['info'] = [
            ["", f"converted with NetworkX {networkx.__version__}, networkx.read_gml"],
            ["", f"input file: {p}"],
            ["time", datetime.now()]]
        G.graph['filename'] = f"{p.name}.txt"
    else:
        print("unknown generator")
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
    exe = concurrent.futures.ProcessPoolExecutor(max_workers=min(os.cpu_count(), 5))
    ecc = exe.submit(networkx.eccentricity, G)
    ac = lambda g: networkx.average_clustering(g)
    f1 = [['radius', exe.submit(networkx.radius, G, ecc.result())],
        ['diameter', exe.submit(networkx.diameter, G, ecc.result())],
        ['average clustering', exe.submit(networkx.average_clustering, G)]]
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
    # - k pods with k nodes each, i.e., k**2 pods
    # - k**3 // 4 hosts
    G = networkx.empty_graph(k**2 + (k//2)**2)
    # pod node id is: pod*k + node
    # core node id is (with x,y core coordinates): k**2 + x*k//2 + y
    core_offset = k**2
    # nodetypedict
    node_type_attrs= dict()

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

    for pod in range(k):
            for node in range(k):
                    node_type_attrs[pod*k + node]={"type" : "pod"}
    for i in range(k//2):
        for j in my_range(core_offset + k//2*i, k//2):
            node_type_attrs[j]={"type" : "core"}

    networkx.set_node_attributes(G,node_type_attrs)

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
    G = networkx.convert_node_labels_to_integers(G)
    return G

def generate_plc_fcc(n, cs, seed):
    num_full_clusters = n // cs 
    num_clusters = num_full_clusters
    if n % cs != 0:
        num_clusters += 1
    G = networkx.generators.random_graphs.powerlaw_cluster_graph(num_clusters, 1, 0.2, seed)
    target_edges = 3 * n
    interconnect_edges = len(G.edges)
    random_edges = target_edges - interconnect_edges - num_clusters * (cs - 1) 
    random_edges_per_cluster = random_edges // num_clusters
    print(f"Graph connected: {networkx.is_connected(G)}")
    
    node_number = max(int(n) for n in G.nodes) + 1
    rd_seed(seed)
    nodes = [c_i for c_i in G.nodes]
    for c_i in nodes:
        # create new cluster
        new_nodes = [i for i in range(node_number, node_number + cs)]
        # connect nodes in ring
        networkx.add_cycle(G, new_nodes)

        # add random edges within cluster
        random_edge_counter = 0
        while random_edge_counter < random_edges_per_cluster:
           a, b = choices(new_nodes, k=2)
           if (a, b) not in G and (b, a) not in G:
               G.add_edge(a,b)
               random_edge_counter += 1
        
        # add connection to neighboring clusters
        neighbors = networkx.all_neighbors(G, c_i)
        for neigh in neighbors:
            #create edge to random node in cluster
            G.add_edge(choice(new_nodes), neigh)

        #remove old node - was replaced by cluster
        G.remove_node(c_i)
        node_number += cs
        print(f"Graph connected: {networkx.is_connected(G)}")

    nodes = list(G.nodes())
    print(len(G.edges))
    # add remaining edges as triangles
    while len(G.edges) < target_edges:
        start = choice(nodes)
        potential_neighbors = [n for n, l in networkx.single_source_dijkstra_path_length(G, start, cutoff=2).items() if l == 2]
        if len(potential_neighbors) > 0:
            end = choice(potential_neighbors)
            G.add_edge(start, end)
    print(f"Graph connected: {networkx.is_connected(G)}")

    return networkx.convert_node_labels_to_integers(G)

def generate_plc_cc(n, m, c, d, seed):
    # generate two powerlaw clusters
    F = networkx.generators.random_graphs.powerlaw_cluster_graph(n // 2,m, c, seed)
    H = networkx.generators.random_graphs.powerlaw_cluster_graph(n // 2,m, c, seed + 1)

    # add connections
    rd_seed(seed)
    G_tmp = networkx.union(F,H, rename=('F-', 'H-'))
    connections = []
    for d in range(d):
        random_f = choice(list(F.nodes))
        random_h = choice(list(H.nodes))
        G_tmp.add_edge("F-" + str(random_f), "H-" + str(random_h))
        connections.append(("F-" + str(random_f), "H-" + str(random_h)))

    # Rename nodes to get continuous numbers
    # does not use convert_node_labels_to_integers, because we also want to return connection between clisters
    mapping = {}
    for x in range(n // 2):
        mapping["F-" + str(x)] = x

    for y in range(n // 2):
        mapping["H-" + str(y)] = y + (n // 2)
    G = networkx.relabel_nodes(G_tmp, mapping)

    edge_str = ""
    for c in connections:
        edge = str(mapping[c[0]]) + "-" + str(mapping[c[1]]) + "; "
        edge_str += edge
    return G, edge_str


if __name__ == '__main__':
    main()
