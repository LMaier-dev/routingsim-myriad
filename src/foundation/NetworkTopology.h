//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef __ROUTINGSIM_NETWORKTOPOLOGY_H_
#define __ROUTINGSIM_NETWORKTOPOLOGY_H_

#include <omnetpp.h>
#include <string>
#include <vector>
#include <ctime>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/strong_components.hpp>
#include <boost/graph/floyd_warshall_shortest.hpp>
#include <boost/graph/johnson_all_pairs_shortest.hpp>

#include <Node.h>

namespace routingsim {

using namespace boost;
using std::string;
using std::vector;

/**
 * NetworkTopology provides a data structure to represent network topologies
 * and corresponding methods for their manipulation as well as the runtime
 * state.
 *
 * The boost graph library is used to represent the graph and to perform calculations
 * on it
 *
 * @author Christoph Werle <werle@kit.edu>
 */

class NetworkTopology {
protected:
	typedef boost::unordered_map<string,string> properties_t;

	NetworkTopology &operator=(NetworkTopology const &n);
	NetworkTopology(NetworkTopology const &n);

	// Vertex Properties
	struct BGLNode {
		bool active = false; // mark status of used nodes
		bool disabled = false; // mark unused (non-existing) nodes
		int maxRTentries = 0;
		properties_t properties = {}; // store properties from config file
	};
	// Edge Properties
	struct BGLEdge {
		bool active = false;
		float weight = 1.0;
	};
	// BGL Definitions
	// to enforce the absence of multi-edges, vertexlist might be set to setS,
	// currently this needs to be ensured in the topology file for efficiency
	typedef adjacency_list<setS, vecS, bidirectionalS, BGLNode, BGLEdge> topologyGraph;
	typedef graph_traits<topologyGraph>::vertex_descriptor BGLVertexDesc;
	typedef graph_traits<topologyGraph>::vertex_iterator BGLVertexIt;
	typedef graph_traits<topologyGraph>::edge_descriptor BGLEdgeDesc;
	typedef graph_traits<topologyGraph>::edge_iterator BGLEdgeIt;
	typedef graph_traits<topologyGraph>::out_edge_iterator BGLOutEdgeIt;
	typedef graph_traits<topologyGraph>::in_edge_iterator BGLInEdgeIt;
	using Vertex = BGLVertexDesc;
	using Edge = BGLEdgeDesc;


	// filters
	template <typename GraphType>
	class activeNodes {
		public:
			activeNodes() : graph(nullptr) {};
			activeNodes( GraphType &g) : graph(&g) {};
			template <typename VertexType>
			bool operator() (const VertexType &v) const { return (*graph)[v].active; }
		protected:
			GraphType *graph;
	};

	template <typename GraphType>
	class activeEdges {
		public:
			activeEdges() : graph(nullptr) {};
			activeEdges( GraphType &g) : graph(&g) {};
			template <typename EdgeType>
			bool operator() (const EdgeType &e) const { return (*graph)[e].active; }
		protected:
			GraphType *graph;
	};

	typedef filtered_graph<topologyGraph, activeEdges<topologyGraph>, activeNodes<topologyGraph> > FilteredGraph;
	typedef graph_traits<FilteredGraph>::vertex_iterator ActiveBGLVertexIt;
	typedef graph_traits<FilteredGraph>::edge_iterator ActiveBGLEdgeIt;
	typedef graph_traits<FilteredGraph>::adjacency_iterator ActiveBGLAdjacencyIt;

	// Visitor to record the predecessors
    template <typename PredecessorMap>
	class bfs_visitor_bidirectional : public default_bfs_visitor {
		public:
			bfs_visitor_bidirectional(PredecessorMap& pred_map) : pred_map(pred_map) {}

			template <typename Edge, typename Graph>
			void tree_edge(Edge e, const Graph& g) const {
				Vertex u = source(e, g);
				Vertex v = target(e, g);
				pred_map[v] = u;
			}

		private:
			PredecessorMap& pred_map;
	};

	/// our topology as (directed) bgl graph
	topologyGraph* BGLtopology = nullptr;

	cModule *networkManager = nullptr;
	/// Bool to keep track of whether the topology was changed since last
	/// shortest path calculation; needs to be set to true by topology-
	/// modifying functions
	bool topologyAltered = true;

	unsigned long spathMatrixSize; // only the number of rows (= number of columns)
	bool spathAvailable  = false;
    float **spath = nullptr; // two dimensional array of floats as distance (float is usually 4 bytes)

	enum shortestPathCalculationMode_t : uint8_t {
	  fullDistanceMatrix = 1,
	  onDemandDijkstra = 2,
	  onDemandBidirectionalBFS = 3 // only useful for identical weights/hops as metric
	} shortestPathCalculationMode;

	/**
	 * Allocate Memory for the shortest path matrix
	 *
	 **/
	float** allocateSPM(unsigned long numberOfNodes, float **spath);

	std::vector<Vertex>
	reconstruct_path(Vertex start, Vertex goal, Vertex meeting_point,
					 const std::vector<Vertex> &pred_map_start,
					 const std::vector<Vertex> &pred_map_goal) const;

	protected:
        /// link between the graph and the omnet network
		std::unordered_map<nodeid_t, cModule *> moduleMap;

	public:
		//bool compareSPM();
		/**
		 * Constructs an empty network topology.
		 */
		explicit NetworkTopology();

		/**
		 * Destroys the network topology
		 */
		~NetworkTopology();

		/**
		 * Read network topology from filename into data structure
		 * This function dispatches the actual reading to specialized functions
		 * depending on topologytype (for possible values see NetworkManager.ned)
		 */
		void readTopology(const string& topologyFileName, string topologytype = "SELF_DEFINED", bool useOnlyLargestConnectedComponent=true, bool makeLinksBidirectional=true, bool allowNodeIdRemapping=false, bool compressIDs=false);

		/**
		 * Read our own
		 */
		void readSelfDefinedTopology(std::ifstream &topologyFile, std::set<nodeid_t> &nodeset, vector< std::pair<nodeid_t, nodeid_t> > &edges, boost::unordered_map < nodeid_t, properties_t > &nodeproperties);

		/**
		 * Read a dimes as edges topology; by default, bidirectional edges are
		 * assumed. If this is not the intended behaviour, set makeBidirectional
		 * to false.
		 */
		void readDimesTopology(std::ifstream &topologyFile, std::set<nodeid_t> &nodeset, vector< std::pair<nodeid_t, nodeid_t> > &edges, boost::unordered_map < nodeid_t, properties_t > &nodeproperties, bool makeBidirectional=true);

		/**
		 * Read shortest paths from file with shortest-path pre-calculations
		 *
		 * @return False, if file not exists or parsing goes wrong
		 */
		bool readShortestPathMatrix(const string& shortestPathFileName);

		/**
		 * Write shortest paths to file with shortest-path pre-calculations
		 *
		 */
		void writeShortestPathMatrix(const string& shortestPathFileName);

		/**
		 * Builds the network topology with given modules.
		 *
		 * NetworkManager should be passed as the parent module.
		 */
		void createNetwork(cModule *networkManager, std::string modTypeStr = "routingsim.foundation.NodeWrapper");

		/**
		 * Extract a constructed network topology built from a .ned file
		 *
		 * NetworkManager (or another module of the network) should be
		 * passed as  cModule
		 */
		void extractFromNetwork(const cModule *networkManager);

		/**
		 * Returns a random node from the set of active nodes
		 *
		 * @param rng a random number generator.
		 * @return a random node identifier.
		 */
		nodeid_t getRandomNode(cRNG *rng) const;

		/// Return k random distinct pairs of active nodes in a vector (-> Sampling)
		/// Random Edge Sampling ("Sampling from Large Graphs", Jure Leskovec and Christos Faloutsos, 2006)
		vector<std::pair<nodeid_t, nodeid_t> > getRandomNodePairs( cRNG *rng, size_t k ) const;

		/// Return a random edge selected from the set of active edges (activeLinkNumber-th edge)
		std::pair<nodeid_t, nodeid_t> getRandomEdge(size_t activeLinkNumber) const;


		/// Returns the node degree, i.e., number of links to ACTIVE neighbors
		/// If the node itself is not active, zero is returned
		size_t getNodeDegree( nodeid_t node ) const;

		/// return in the parameter retDegreeMap the list of active nodes and
		/// their degree, resp. out-degree in case of a directed graph
		/// Return value: Size of retDegreeMap
		size_t getNodeDegrees( std::map<nodeid_t, unsigned long> &retDegreeMap) const;

		/// returns the FILTERED set of ACTIVE nodeids contained in the topology
		/// in the parameter v
		void getNodeIDs(vector<nodeid_t> &v) const;

		/// return the active neighbors of a node in the vector neighborIDs
		void getNeighborNodeIDs(nodeid_t fromID, vector<nodeid_t> &neighborIDs) const;

		// return the node ids of the count highest degree nodes
		vector<nodeid_t> getHighestDegreeNodes( unsigned int count ) const;

		/**
		 * Adds an active bidirectional link between nodes node1 and node2
		 * TODO: Properties?
		 */
		void addLink(nodeid_t node1, nodeid_t node2);

		/// Disables the *unidirectional* link between node1 and node2
		void disableLink(nodeid_t node1, nodeid_t node2);

		/// Enables the *unidirectional* link between node1 and node2
		void enableLink(nodeid_t node1, nodeid_t node2);

		/// Disable all links of the node to and from its neighbors
		void disableNode(nodeid_t node);

		/// Re-enable all links of the node to its ACTIVE neighbors
		/// links that were disabled individually before between node and
		/// active neighbors are also! re-enabled
		void enableNode(nodeid_t node);

		/// returns status of node
		bool isActive(nodeid_t node) const { return (*BGLtopology)[node].active; }

		// are the two nodes directly connected with an active link
		bool isConnected(nodeid_t node1, nodeid_t node2) const;

        // does an edge from node1 to node 2 exist
        bool edgeExists(nodeid_t node1, nodeid_t node2) const;

		/// Returns true, if the network is connected
		bool isConnected() const;

		/// Returns the current number of partitions in the network
		unsigned long getNumPartitions() const;

		/// Returns the number of ACTIVE nodes in the network
		unsigned long getNetworkSize() const { 	return getActiveNumberNodes(); }

		/// Returns the TOTAL number of nodes irregarding their state (active/inactive)
		inline unsigned long getNumberNodes() const { return num_vertices(*BGLtopology); }

		/// Returns the TOTAL number of links irregarding their state (active/inactive)
		inline unsigned long getNumberLinks() const { return num_edges(*BGLtopology); }

		/// Returns the number of ACTIVE nodes in the network
		unsigned long getActiveNumberNodes() const;

		/// Returns the number of ACTIVE links in the network
		unsigned long getActiveNumberLinks() const;

		template <typename GraphType>
		std::vector<Vertex> bidirectional_bfs(const GraphType &fg, Vertex start, Vertex goal) const;

		/// return the shortest path distance between two nodes
		/// if not calculated already (or the graph has changed),
		/// calculate it on the fly and return result
		float getDistance(nodeid_t from, nodeid_t to);

		/// get the shortest path between from and to in the full, unfiltered topology
		vector<nodeid_t> getFullTopologyPath(nodeid_t from, nodeid_t to) const;

		/// get the shortest path between from and to in the ACTIVE topology respecting failed nodes and edges
		vector<nodeid_t> getActiveTopologyPath(nodeid_t from, nodeid_t to) const;

		/// return true if either a link or a node of the shortest path within the
		/// unaltered original topology failed; false otherwise
		bool fullTopologyPathFailed(nodeid_t from, nodeid_t to) const;

		/// reset all protocols on the Node identified by nodeid n
		void resetNode(nodeid_t n);

		void setShortestPathCalculationMode(string mode);

		inline Node *getNodeModule(nodeid_t nid) { return Node::getNode(moduleMap[nid]); }
};

// Bidirectional BFS function, returns path including start and goal vertex
// if only the goal vertex is given as result, it means that no path has been found
template <typename GraphType>
std::vector<NetworkTopology::Vertex>
NetworkTopology::bidirectional_bfs(const GraphType &g, Vertex start, Vertex goal) const {

	// handle special case (otherwise an empty vector is returned)
	if (start == goal or g[start].active == false or g[goal].active == false)
		return { goal };
	// maps that record visited vertices (predecessors)
	std::vector<Vertex> pred_map_start(num_vertices(g), graph_traits<topologyGraph>::null_vertex());
	std::vector<Vertex> pred_map_goal(num_vertices(g), graph_traits<topologyGraph>::null_vertex());

	std::queue<Vertex> queue_start, queue_goal;
	queue_start.push(start);
	queue_goal.push(goal);

	pred_map_start[start] = start;
	pred_map_goal[goal] = goal;

	auto start_top_level_elements= queue_start.size();
	auto goal_top_level_elements= queue_goal.size();
	while (not queue_start.empty() and not queue_goal.empty()) {
		// BFS from start vertex
		while (not queue_start.empty() and start_top_level_elements > 0) {
			// visiting vertex u
			Vertex u = queue_start.front();
			// has the vertex been discovered from the goal side already?
			if (pred_map_goal[u] != graph_traits<topologyGraph>::null_vertex()) {
				return reconstruct_path(start, goal, u, pred_map_start, pred_map_goal); // Path found
			}
			queue_start.pop();
			start_top_level_elements--;

			// add/discover all neighbor nodes of u for the next level
			for (auto edge : make_iterator_range(out_edges(u, g))) {
				Vertex v = target(edge, g);
				// v will be ignored if already discovered
				if (pred_map_start[v] == graph_traits<topologyGraph>::null_vertex()) {
					pred_map_start[v] = u;
					queue_start.push(v);
				}
			}
		} // end while not finished the top level vertices
		// remember this for the next rounds
		start_top_level_elements = queue_start.size();

		// BFS from goal vertex
		while (not queue_goal.empty() and goal_top_level_elements > 0) {
			// visiting vertex u
			Vertex u = queue_goal.front();
			// has the vertex been discovered from the start side already?
			if (pred_map_start[u] != graph_traits<topologyGraph>::null_vertex()) {
				return reconstruct_path(start, goal, u, pred_map_start, pred_map_goal); // Path found
			}
			queue_goal.pop();
			goal_top_level_elements--;

			// add all neighbor nodes of u for the next level
			for (auto edge : make_iterator_range(out_edges(u, g))) {
				Vertex v = target(edge, g);
				if (pred_map_goal[v] == graph_traits<topologyGraph>::null_vertex()) {
					pred_map_goal[v] = u;
					queue_goal.push(v);
				}
			}
		} // end while not finished the top level vertices

		// remember number of top elements for the next rounds
		goal_top_level_elements= queue_goal.size();
	} // end outer while

	return { goal }; // No path found
}

} // namespace RoutingSim

#endif /* __ROUTINGSIM_NETWORKTOPOLOGY_H_ */
