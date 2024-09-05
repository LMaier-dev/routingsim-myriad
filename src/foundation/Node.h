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

#ifndef __ROUTINGSIM_NODE_H_
#define __ROUTINGSIM_NODE_H_

#include <omnetpp.h>
#include <cdisplaystring.h>
#include <unordered_map>

namespace routingsim {
class Node;
}

#include "NodePkt_m.h"
#include "BaseTypes.h"
#include "Protocol.h"

namespace routingsim {

/**
 * TODO - Generated class
 */
class Node: public cSimpleModule {
public:
	typedef unsigned long long linkcount_t;
	/// returns the node, that is either a submodule, the node of a protocol, etc
	static Node* getNode(cModule *mod);
	/// returns the node on the other side of this outgoing gate
	static Node* getOtherNode(cGate *gate);

	/// returns the node_id of this node
	nodeid_t getNodeId() const { return id; }

	/// returns the physical neighbors of the node
	const vector_set<nodeid_t>& getNeighbors() const { return neighbor_set; }

	/// getNeighborMap()
	const std::unordered_map<nodeid_t, neighbor_t> getNeighborMap() const { return neighbor_map; }

	/// getLinkUsageMap()
	const std::unordered_map<nodeid_t, linkcount_t> getLinkUsageMap() const { return linkusage_map; }

	/// Binds a module to the given packet type
	void bind( int type, cModule* mod );

	/// Unbind the module
	void unbind( int type, cModule* mod );

	/// sets the color of the display string of this Node (and its wrapper).
	void setColor(const char *color);

protected:
	friend class NetworkTopology;
	virtual void initialize(int stage);
	virtual int numInitStages() const { return 2; };

	virtual void handleMessage(cMessage *msg);
	virtual void doBuildInside();
	virtual ~Node();
	/// Disable link to neighbor mod in gate vector gatename
	bool disableLink(Node *mod, const char *gatename);
	/// Enable link to neighbor mod in gate vector gatename
	bool enableLink(Node *mod, const char *gatename);
        /// Set link datarate to neighbor mod
        bool setLinkDatarate(Node *mod, const char *gatename, double datarate);
	/// sendNotify is called from the context of NetworkManager
	void sendNotify(NodePkt *packet, cGate *dg);

protected:
	/// my own id (only local scope, corresponding to mac address)
	nodeid_t id;

	/// the bindings of the node modules
	/// maps type int to gate index
	std::unordered_map<int, int> bindings;

	/// Neighbors of the node
	vector_set<nodeid_t> neighbor_set;
	std::unordered_map<nodeid_t, neighbor_t> neighbor_map;
	/// need separate map for link counters as neighbor_map will remove nodes
	std::unordered_map<nodeid_t, linkcount_t> linkusage_map;

	/// Broadcast a message among all node modules
	void sendToModules( NodePkt* np );

	/// Broadcast a message among neighbors
	void sendToNeighbors( NodePkt* np );

	/// Send a message to a specific neighbor
	void sendToNode( nodeid_t id, NodePkt* np );

private:
	/// dynamically disabled links, used to check whether we have to
	/// send a packet in sendToNode (which is also used by sendToNeighbors)
	std::vector<cGate *> disabledLinks;

	/// add a gate to disabledLinks
	void disableGate(cGate *);

	/// remove a gate from disabledLinks
	void enableGate(cGate *);

	/// check whether a gate is marked disabled
	bool gateIsDisabled(const cGate *g) const;

	/// Initialize neighbors
	virtual void initNeighbors();

	/// On hello message
	virtual void onHello( NodePkt* pkt );

	/// On goodbye message
	virtual void onGoodbye( NodePkt* pkt );

	int minDelay, maxDelay; // in milliseconds

        cGate *findGate(Node* mod, const char *gatename);
};

} //namespace

#endif
