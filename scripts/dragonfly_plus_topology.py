import networkx as nx
import matplotlib.pyplot as plt
import random
import numpy as np


class DragonflyPlusTopology:
    """
    Dragonfly+ topology generator and analyzer.

    The topology has three levels:
    - Servers connected to Leaf routers
    - Routers (Spine+Leaf) within groups fully connected (intra-group links)
    - Spine routers connect groups (inter-group links) with different number of links
    - So host -> leaf -> spine -> spine
    - each leaf router is connected to p hosts and s spine routers (up links) within a group
    - each spine router is connected to l leaf routers in its group (down links) and to h spine routers of other groups
    - there is p = l = s = h and p = h = k/2, k= router radix
    """

    def __init__(self, num_groups, leaf_routers_per_group, inter_group_type, add_hosts):
        """
        Initialize the Dragonfly+ topology parameters.

        Args:
            num_groups (int): Number of groups
            leaf_routers_per_group (int): Number of leaf routers per group, there will be the same number of spine routers added,
                                          i.e., overall we get twice the number of leaf routers in the group
            inter_group_type (string): "large", "medium" (small is not supported)
                                       large: each pair of groups is connected by a single global link
                                       medium: every spine router is connected to every other group by a single global link
            add_hosts (bool): True: add host nodes to leaf routers, False: do not add host nodes
        """
        self.num_groups = num_groups
        self.leaf_routers_per_group = leaf_routers_per_group # this is the same as k/2
        self.spine_routers_per_group = leaf_routers_per_group # this is the same as k/2
        self.routers_per_group = self.leaf_routers_per_group + self.spine_routers_per_group
        self.hosts_per_group = self.leaf_routers_per_group * self.leaf_routers_per_group

        # Calculate total nodes
        self.total_routers = self.num_groups * 2 * self.leaf_routers_per_group
        self.total_hosts = self.num_groups * self.hosts_per_group
        self.total_nodes = self.total_routers + self.total_hosts

        self.inter_group_type = inter_group_type
        self.add_hosts = add_hosts

        # sanity check w.r.t. the number of ports, the invariant of equal number of up and down ports cannot be violated
        if inter_group_type == 'large':
            if (self.spine_routers_per_group * self.spine_routers_per_group + 1) < self.num_groups:
                raise ValueError("Number of groups does not match with number of routers (not enough ports available)")
        elif inter_group_type == 'medium':
            if self.spine_routers_per_group + 1 < self.num_groups:
                raise ValueError("Number of groups does not match with number of routers (not enough ports available)")

        # Create the graph
        self.graph = nx.Graph()
        self._build_topology()


    def _build_topology(self):
        """Construct the Dragonfly+ topology."""
        # Add routers first and interconnect leaf/spine routers first
        for group_id in range(self.num_groups):
            self._create_group(group_id)

        # create inter-group links
        if self.inter_group_type == 'large': #
            for first_group_idx in range(self.num_groups):
                for second_group_idx in range(first_group_idx+1,self.num_groups):
                    self.graph.add_edge(self._get_spine_router_id(first_group_idx, (second_group_idx-1) // self.spine_routers_per_group),
                                        self._get_spine_router_id(second_group_idx, first_group_idx // self.spine_routers_per_group),
                                        link_type='inter-group')
                    #print(f"G{first_group_idx} S{(second_group_idx-1) // self.spine_routers_per_group}->G{second_group_idx} S{first_group_idx // self.spine_routers_per_group} N{self._get_spine_router_id(first_group_idx, (second_group_idx-1) // self.spine_routers_per_group)}->N{self._get_spine_router_id(second_group_idx, first_group_idx // self.spine_routers_per_group)}")
        elif self.inter_group_type == 'medium':
            for first_group_idx in range(self.num_groups):
                for spine_router_idx in range (self.spine_routers_per_group):
                    for second_group_idx in range(self.num_groups):
                        self.graph.add_edge(self._get_spine_router_id(first_group_idx, spine_router_idx),
                                            self._get_spine_router_id(second_group_idx, first_group_idx % self.spine_routers_per_group),
                                            link_type='inter-group')
        else:
            raise ValueError("Unknown type of inter group connections")

        if self.add_hosts == True:
            host_idx_start = len(self.graph.nodes)
            host_idx = host_idx_start
            for group_idx in range(self.num_groups):
                for leaf_router_idx in range(self.leaf_routers_per_group):
                    # number of hosts per leaf router is the same as the number of leaf routers
                    for h in range(self.leaf_routers_per_group):
                        self.graph.add_node(host_idx, node_type="host")
                        self.graph.add_edge(self._get_leaf_router_id(group_idx, leaf_router_idx),
                                            host_idx,
                                            link_type='host-leaf')
                        host_idx += 1
            assert (host_idx-host_idx_start) == self.total_hosts, "number of hosts does not match expected number"

    def _create_group(self, group_idx):
        """Create a group of leaf and spine routers with internal connections (all bipartite links)"""
        # we start with leaf routers at lower indices
        for leaf_router_idx in range(self.leaf_routers_per_group):
            self.graph.add_node(self._get_leaf_router_id(group_idx,leaf_router_idx), node_type='leaf', group_id=group_idx)
        for spine_router_idx in range(self.spine_routers_per_group):
            self.graph.add_node(self._get_spine_router_id(group_idx,spine_router_idx), node_type='spine', group_id=group_idx)
        # create links between every leaf and spine router, i.e., full bipartite mesh
        for leaf_router_idx in range(self.leaf_routers_per_group):
            for spine_router_idx in range(self.spine_routers_per_group):
                self.graph.add_edge(self._get_leaf_router_id(group_idx, leaf_router_idx),
                                    self._get_spine_router_id(group_idx, spine_router_idx),
                                    link_type='intra-group'
                )
                #print(f"G{group_idx} L{leaf_router_idx}->S{spine_router_idx} N{self._get_leaf_router_id(group_idx, leaf_router_idx)}->N{self._get_spine_router_id(group_idx, spine_router_idx)}")


    def _get_spine_router_id(self, group_idx, spine_router_idx):
        """Get the router id of a spine router in a certain group"""
        if spine_router_idx >= self.spine_routers_per_group:
            raise ValueError(f"Spine router index too large {spine_router_idx}, max allowed: {self.spine_routers_per_group-1}")
        else:
            return group_idx * self.routers_per_group + self.leaf_routers_per_group + spine_router_idx

    def _get_leaf_router_id(self, group_idx, leaf_router_idx):
        """Get the router id of a leaf router in a certain group"""
        if leaf_router_idx >= self.leaf_routers_per_group:
            raise ValueError(f"leaf router index too large {leaf_router_idx}, max allowed: {self.leaf_routers_per_group-1}")
        else:
            return group_idx * self.routers_per_group + leaf_router_idx

    def _get_router_id(self, group_idx, router_idx):
        """Get the global router ID from group and router indices."""
        if router_idx >= self.routers_per_group:
            raise ValueError("Router index too large")
        else:
            return group_idx * self.routers_per_group + router_idx

    def _get_router_group(self, router_id):
        """Get the group index for a given router ID."""
        return router_id // self.routers_per_group

    def is_connected(self):
        """Check if the topology is connected."""
        return nx.is_connected(self.graph)

    def get_connected_components(self):
        """Return the number of connected components."""
        return nx.number_connected_components(self.graph)

    def get_statistics(self):
        """Return topology statistics."""
        return {
            'total_nodes': self.total_nodes,
            'total_routers': self.total_routers,
            'total_servers': self.total_hosts,
            'num_groups': self.num_groups,
            'routers_per_group': self.leaf_routers_per_group + self.spine_routers_per_group,
            'servers_per_group': self.hosts_per_group,
            'edges': self.graph.number_of_edges(),
            'is_connected': self.is_connected(),
            'num_components': self.get_connected_components(),
            'average_degree': sum(dict(self.graph.degree()).values()) / self.total_nodes,
            'diameter': nx.diameter(self.graph) if self.is_connected() else float('inf'),
            #'shortest paths from 0' : dict(nx.single_source_all_shortest_paths(self.graph,0))
        }

    def draw_topology(self, show_servers=False, ax=None):
        """Draw the topology visualization."""
        if ax is None:
            fig, ax = plt.subplots(figsize=(12, 8))

        # Position nodes
        pos = {}

        # Position groups in a circle
        group_positions = {}
        for group_idx in range(self.num_groups):
            angle = 2 * 3.14159 * group_idx / self.num_groups
            group_center = (3 * np.cos(angle), 3 * np.sin(angle))
            group_positions[group_idx] = group_center

        # Position routers within groups
        for group_idx in range(self.num_groups):
            group_center = group_positions[group_idx]
            for router_idx in range(self.routers_per_group):
                angle = 2 * 3.14159 * router_idx / self.routers_per_group
                router_pos = (group_center[0] + 0.5 * np.cos(angle),
                             group_center[1] + 0.5 * np.sin(angle))
                router_id = self._get_router_id(group_idx, router_idx)
                pos[router_id] = router_pos

                # Position servers around the router
                for server_idx in range(self.leaf_routers_per_group):
                    server_angle = 2 * 3.14159 * server_idx / self.leaf_routers_per_group + angle
                    server_pos = (router_pos[0] + 0.3 * np.cos(server_angle),
                                 router_pos[1] + 0.3 * np.sin(server_angle))
                    server_id = self.total_routers + (group_idx * self.routers_per_group + router_idx) * self.leaf_routers_per_group + server_idx
                    if show_servers:
                        pos[server_id] = server_pos

        # Node colors based on type
        node_colors = []
        for node in self.graph.nodes():
            if self.graph.nodes[node].get('node_type') == 'spine':
                node_colors.append('red')
            elif self.graph.nodes[node].get('node_type') == 'leaf':
                node_colors.append('blue')
            else:
                node_colors.append('green')

        # Draw the graph
        if show_servers:
            nx.draw(self.graph, pos, ax=ax, node_color=node_colors,
                   node_size=150, with_labels=True, font_size=6)
        else:
            # Draw only routers for cleaner visualization
            router_nodes = list(range(self.total_routers))
            router_subgraph = self.graph.subgraph(router_nodes)
            nx.draw(router_subgraph, pos, ax=ax, node_color=node_colors[:self.total_routers],
                   node_size=300, with_labels=True, font_size=8)

        ax.set_title(f'Dragonfly+ Topology (groups={self.num_groups}, routers per group={self.routers_per_group}')
        return ax


def test_dragonfly_plus_connectivity():
    """Test connectivity for various Dragonfly+ configurations."""
    test_cases = [
        # (a, b, c, description)
        (2, 2, 'large', False, "Small Dragonfly+ with 2 groups, 4 routers/group, large"),
        (3, 2, 'large', False, "Medium Dragonfly+ with 3 groups, 4 routers/group, large"),
        (5, 2, 'large', False, "Medium Dragonfly+ with 5 groups, 4 routers/group, large"),
        (5, 4, 'medium', False, "Larger Dragonfly+ with 5 groups, 8 routers/group, medium"),
        (17, 4, 'large', False, "Larger Dragonfly+ with 17 groups, 8 routers/group, large"),
        (65, 8, 'large', False, "Complex Dragonfly+ with 65 groups, 16 routers/group, large")
    ]

    print("Dragonfly+ Topology Connectivity Test Results:")
    print("=" * 60)

    for n_groups, n_leaf, t_type, add_hosts, description in test_cases:
        topology = DragonflyPlusTopology(n_groups, n_leaf, t_type, add_hosts)
        stats = topology.get_statistics()

        print(f"\nTest Case: {description}")
        print(f"  Groups: {n_groups}")
        print(f"  Routers/Group: {2*n_leaf}")
        print(f"  Servers: {stats['total_servers']}")
        print(f"  Total Nodes: {stats['total_nodes']}")
        print(f"  Total Edges: {stats['edges']}")
        print(f"  Connected: {stats['is_connected']}")
        print(f"  Components: {stats['num_components']}")
        print(f"  Average Degree: {stats['average_degree']:.2f}")

        if stats['is_connected']:
            print(f"  Network Diameter: {stats['diameter']}")

        # Verify specific connectivity conditions
        assert stats['is_connected'], f"Topology with is not connected!"
        assert stats['num_components'] == 1, f"Topology has {stats['num_components']} components instead of 1!"

        # Test router degrees
        # router_degrees = topology.get_router_degrees()
        # for key, degree in router_degrees.items():
        #     if 'global' in key:
        #         expected_degree = (b - 1) + (a - 1)  # Intra-group + inter-group
        #     else:
        #         expected_degree = b - 1  # Only intra-group
        #     assert degree == expected_degree, f"Degree mismatch for {key}: expected {expected_degree}, got {degree}"

    print("\n" + "=" * 60)
    print("All connectivity tests passed successfully!")
    return True


def generate_custom_dragonfly_plus():
    """Generate a custom Dragonfly+ topology based on user input."""
    print("\nCustom Dragonfly+ Topology Generator")
    print("-" * 30)

    n_g = int(input("Enter number of groups: "))
    n_l = int(input("Enter leaf routers per group: "))
    t_t = input("Interconnection: (l)arge/(m)edium: ")

    if n_g < 1 or n_l < 1 or not (t_t in set(['m','l'])):
        print("Error: All parameters must be positive integers and interconnection either l or m!")
        return None

    if t_t == 'm':
        t_type = 'medium'
    elif t_t == 'l':
        t_type = 'large'

    topology = DragonflyPlusTopology(n_g, n_l, t_type, False)

    print(f"\nGenerated Dragonfly+ Topology Statistics:")
    stats = topology.get_statistics()
    for key, value in stats.items():
        print(f"  {key}: {value}")

        # Draw the topology
    try:
        fig, axes = plt.subplots(1, 2, figsize=(16, 8))

        # Without servers (routers only)
        topology.draw_topology(show_servers=False, ax=axes[0])
        axes[0].set_title(f'Router-only View (groups={n_g}, k={2*n_l}, {t_type})')

        # With servers
        topology.draw_topology(show_servers=True, ax=axes[1])
        axes[1].set_title(f'Full Topology View (groups={n_g}, k={2*n_l}, {t_type})')

        plt.tight_layout()
        plt.savefig(f'dragonfly_plus_groups{n_g}_k{2*n_l}_{t_type}.png', dpi=300, bbox_inches='tight')
        print(f"\nTopology visualization saved as 'dragonfly_plus_groups{n_g}_k{2*n_l}_{t_type}.png'")
        plt.show()

    except ImportError:
        print("Note: Matplotlib not available for visualization")

    return topology



# Main execution
if __name__ == "__main__":
    # Run connectivity tests first
    test_dragonfly_plus_connectivity()

    # Optional: Generate custom topology
    #custom_topology = generate_custom_dragonfly_plus()
    print("End.")
