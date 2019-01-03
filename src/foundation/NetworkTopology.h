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

#ifndef NETWORKTOPOLOGY_H_
#define NETWORKTOPOLOGY_H_

#include <omnetpp.h>
#include <string>
#include <vector>

#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>

/// forward declaration
namespace routingsim {
class NetworkTopology;
}

#include <Node.h>

namespace routingsim {

using namespace std;
using namespace boost;

typedef float cost_t;
#define INVALID_DISTANCE 	0xff;

/**
 * NetworkTopology provides a data structure to represent network topologies
 * and corresponding methods for their manipulation as well as the runtime
 * state.
 *
 * @author Christoph Werle <werle@kit.edu>
 * @author Sebastian Mies <mies@kit.edu>
 */
class NetworkTopology {
public:
	/**
	 * Constructs an empty network topology.
	 */
	NetworkTopology();

	/**
	 * Destroyes the network topology
	 */
	~NetworkTopology();

	/**
	 * Read network topology from filename into data structure
	 */
	void readTopology(string topologyFile);

	/**
	 * Read shortest paths from file with shortest-path pre-calculations.
	 *
	 * @return False, if file not exists or parsing goes wrong
	 */
	bool readShortestPaths(string shortestPathsFile);

	/**
	 * Builds the network topology with given modules.
	 *
	 * NetworkManager should be passed as the parent module.
	 */
	void createNetwork(cModule *networkManager);

	/**
	 * Extract a constructed network topology built from a .ned file
	 *
	 * NetworkManager (or another module of the network) should be
	 * passed as  cModule
	 */
	void extractFromNetwork(cModule *networkManager);

	/**
	 * Returns a random node identifier.
	 *
	 * @param rndvalue a random value.
	 * @return a random node identifier.
	 */
	nodeid_t getRandomNode( size_t rndvalue ) const;

	/**
	 * Returns the maximal set of connected nodes.
	 */
	unordered_set<nodeid_t>* getGiantComponent(unordered_set<nodeid_t>& disabled ) const;

	/**
	 * Returns a node's degree, i.e., number of links.
	 */
	size_t getNodeDegree( nodeid_t node ) const;

	/**
	 * Returns all node ID's
	 */
	vector<nodeid_t>* getNodeIDs() const;

	/**
	 * Adds a bidirectional link between nodes node1 and node2
	 */
	void addLink(nodeid_t node1, nodeid_t node2);

	/**
	 * Disables the bidirectional link between node1 and node2
	 * in the Node multiplexer
	 */
	void disableLink(nodeid_t node1, nodeid_t node2);

	/**
	 * Enables the bidirectional link between node1 and node2
	 * in the Node multiplexer
	 */
	void enableLink(nodeid_t node1, nodeid_t node2);

	/**
	 * Disable all links of the node to its neighbors
	 */
	void disableNode(nodeid_t node);

	/**
	 * Re-enable all links of the node to its neighbors
	 */
	void enableNode(nodeid_t node);

	/**
	 * Enable or disable a link between two nodes
	 */
	void setConnected(nodeid_t node1, nodeid_t node2, bool connected);

	/**
	 * Returns true, if nodes are connected by a link
	 */
	bool isConnected(nodeid_t node1, nodeid_t node2);

	/**
	 * Returns true, if the network is connected
	 */
	bool isConnected();

	/**
	 * returns the number of nodes in the network
	 * TODO: consider node status
	 */
	int getNetworkSize();

	/**
	 * Returns the distance as sum of the link weights on the path
	 */
	cost_t getDistance(nodeid_t from, nodeid_t to);

	/**
	 * Returns the distance as sum of the link weights on the path
	 */
	cost_t getDistance(cModule *from, cModule *to);

	/**
	 * Helper-function mapping a cModule to the corresponding nodeid_t
	 */
	nodeid_t moduleToNodeID(cModule *cmod);

	/**
	 * Helper-function mapping a nodeid_t to the corresponding cModule
	 */
	cModule *NodeIDToModule(nodeid_t node);

	/**
	 * Returns the total number of nodes
	 */
	uint getNumberNodes() const {
		return numberNodes;
	}

	/**
	 * Returns the total number of links
	 */
	uint getNumberLinks() const {
		return numberLinks;
	}

public:
	class TopologyNode;
	TopologyNode *getLinkset(nodeid_t);

private:
	/// List of adjacent nodes + link properties
	typedef unordered_map<nodeid_t, TopologyNode*> adjacencylist_t;
	adjacencylist_t adjacencyList;

	/// mapping from a node in the graph topology to the corresponding
	/// Node module
	typedef unordered_map<nodeid_t, cModule*> modulemap_t;
	modulemap_t moduleMap;

	/// are we aware of the shortest paths, i.e., is the mapping sp valid?
	bool spath_available;
	uint8_t* spath_costs;
	unordered_map<nodeid_t, size_t> spath_map;

	/// file from which the topology was read
	string inputFile;

	uint numberNodes;
	uint numberLinks;

};

} // namespace RoutingSim

#endif /* NETWORKTOPOLOGY_H_ */
