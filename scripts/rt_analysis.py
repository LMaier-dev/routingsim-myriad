#! /usr/bin/env python3
import networkx as nx
import json
import statistics
from multiprocessing import Pool
from random import Random
from dataclasses import dataclass
import mmap

class RoutingTableAnalyzer:
    """
    Multiple Analysis functions using routing table state exported from KIRA simulations
    """

    def __init__(self, id_okey_mapping, topology, node_info, key_length=112):
        
        # Calculate the maximum id for later use
        self.max_id = 2 ** key_length - 1 
        self.key_length = key_length

        # mapping of RoutingSim nodeid -> KIRA OverlayKey 
        self.mapping = id_okey_mapping
        # same thing but as a dict
        self.mapping_dict = dict((k,int(v, 16)) for k, v in id_okey_mapping)


        # the reverse of the above
        self.reverse_mapping = [(int(key, 16), nid) for nid, key in self.mapping] 
        self.reverse_mapping.sort(key=lambda item: item[0])

        # the network topology as a networkx graph
        self.topology = topology

        # the routing table of each node
        self.routing_tables = node_info

        # a simple list of node ids
        self.node_list = list(nid for nid, _ in self.mapping)

    def calculate_route(self, from_nid, to_nid):
        overlay_hops = [from_nid]
        path = [from_nid]

        while overlay_hops[-1] != to_nid:
           _, path_segment, overlay_hop = self.get_closest_node_from_rt(overlay_hops[-1], to_nid) 
           path += path_segment[1:]
           overlay_hops.append(overlay_hop)

        return (path, overlay_hops)

    def calculate_answer_route(self, from_nid, to_nid):
        path, _ = self.calculate_route(from_nid, to_nid)
        G = nx.Graph()
        G.add_edges_from(zip(path[:-1], path[1:]))
        return nx.shortest_path(G, from_nid, to_nid)

    def calculate_later_route(self, from_nid, to_nid):
        # is there something more optimal than this?
        path = self.calculate_answer_route(from_nid, to_nid)
        G = nx.Graph()
        G.add_edges_from(zip(path[:-1], path[1:]))
        G.add_edges_from(self.get_edges_from_rt(from_nid))
        return nx.shortest_path(G, from_nid, to_nid)

    def calculate_foreign_pid_entries(self):
        size = len(self.mapping)
        pid_destinations = [set() for _ in range(size)]

        for nid in range(size):
            for bucket in self.routing_tables[nid]['buckets']:
                for contact in bucket:
                    # no routes needed
                    if len(contact[1]) < 4:
                        continue
                    for hop in contact[1][1:-3]:
                        pid_destinations[hop].add(contact[1][-1])
        return [len(pids) for pids in pid_destinations]

    def calculate_foreign_pid_entries_optimized(self):
        size = len(self.mapping)
        pid_destinations = [set() for _ in range(size)]

        for nid in range(size):
            for bucket in self.routing_tables[nid]['buckets']:
                for contact in bucket:
                    # no routes needed
                    if len(contact[1]) <= 4:
                        continue
                    for hop in contact[1][1:-5]:
                        pid_destinations[hop].add(contact[1][-3])
        return [len(pids) for pids in pid_destinations]

    def get_closest_node_from_rt(self, nid, to_nid):
        target = self.mapping_dict[to_nid]
        closest = (1 << 112, [0]*1000, None)
        for bucket in self.routing_tables[nid]['buckets']:
            for contact in bucket:
                xor_distance = self.mapping_dict[contact[0]] ^ target
                if xor_distance < closest[0]:
                    closest = (xor_distance, contact[1], contact[0])
                elif xor_distance == closest[0] and len(contact[1]) < len(closest[1]):
                    closest = (xor_distance, contact[1], contact[0])
        return closest

        
    def get_edges_from_rt(self, nid, max_distance=None, non_full_only=False):
        edges = list()

        starting_bucket = 0
        if non_full_only:
            starting_bucket = min(self.get_routing_table_depth(nid), self.get_last_full_bucket_depth(nid))
        for bucket in self.routing_tables[nid]['buckets'][starting_bucket:]:
            for contact in bucket:
                for i in range(len(contact[1]) - 1):
                    if max_distance != None and max_distance <= i:
                        break
                    edges.append((contact[1][i], contact[1][i+1]))

        return edges

    def get_nodes_in_range(self, start, end):
        start_index = 0
        for key, _ in self.reverse_mapping:
            if key >= start:
                break
            start_index += 1

        end_index = len(self.reverse_mapping) - 1
        for key, _ in self.reverse_mapping[::-1]:
            if key < end:
                break
            end_index -= 1

        return [nid for _, nid in self.reverse_mapping[start_index:end_index]]


    def get_routing_table_depth(self, nid):
        return len(self.routing_tables[nid]['buckets']) - 1

    def get_closest_nodes_by_xor(self, overlay_keys):

        result = [(mykey, self.reverse_mapping[0][0], self.reverse_mapping[0][0] ^ mykey, self.reverse_mapping[0][1]) for mykey in overlay_keys]

        for key, nid in self.reverse_mapping:
            for i in range(len(result)):
                dist = key ^ result[i][0]
                if  dist < result[i][2]:
                    result[i] = (result[i][0], key, dist, nid)

        return [(r[0],r[3]) for r in result]


    def get_closest_nodes(self, overlay_keys):
        sorted_keys = sorted(overlay_keys)
        results = list()
        
        map_index = 1
        for key in sorted_keys:
            # skip until k+1 higher than key
            while map_index < len(self.reverse_mapping) - 1 and \
                key > self.reverse_mapping[map_index][0]:
                    map_index += 1
                    
            keya = self.reverse_mapping[map_index-1]
            keyb = self.reverse_mapping[map_index]

            #insert result
            if abs(key - keya[0]) > abs(key - keyb[0]):
                results.append((key, keyb[1]))
            else:
                results.append((key, keya[1]))

        return results
        
def parse_topology(file):
    # We use a weird format for topologies and need to skip 
    # the first non-comment line
    while True:
        line = next(file)
        if line[0] != '#':
            break;
    G = nx.read_edgelist(file, nodetype=int)
    file.close()

    return G            

def parse_routing_tables(file):
    routing_tables = list()
    mfile = mmap.mmap(file.fileno(), 0)
    lines = list()
    while True:
        line = mfile.readline()
        if len(line) == 0:
            break
        lines.append(line)

    data = [json.loads(line) for line in lines]
    routing_tables = [None] * len(data)
    for d in data:
       nodeid = int(d['nodeid'])
       routing_tables[nodeid] = d

    file.close()
    return routing_tables



def parse_mapping(file):
    mapping = json.load(file)
    file.close()
    return mapping

def parse_all(topo_file_path, map_file_path, rt_file_path):
    rts = parse_routing_tables(open(rt_file_path,mode="r+b"))
    mapping = parse_mapping(open(map_file_path))
    if topo_file_path == None:
        return RoutingTableAnalyzer(mapping, None, rts)

    topo = parse_topology(open(topo_file_path))
    return RoutingTableAnalyzer(mapping, topo, rts)
