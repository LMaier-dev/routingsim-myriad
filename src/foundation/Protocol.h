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

#ifndef __ROUTINGSIM_PROTOCOL_H_
#define __ROUTINGSIM_PROTOCOL_H_

#include <omnetpp.h>
#include <memory>

namespace routingsim {
class Protocol;
}

#include "BaseTypes.h"
#include "Node.h"

namespace routingsim {
using NodePkt_p = std::unique_ptr<NodePkt>;

/**
 * Class that models a protocol unit.
 * @author ?
 */
class Protocol: public cSimpleModule {
	friend class NetworkManager;
	friend class NetworkTopology;

	// ------------------------------------------------------ helper methods --
protected:
	/// Returns the parent node of the module
	inline Node& node() {
		return *parent;
	}

	/// Bind module to node to receive messages of the given type
	void bind(int type);

	/// Unbind module to node to receive messages of the given type no more
	void unbind(int type);

	/// sends a packet
	inline void sendToNode( nodeid_t nid, NodePkt_p pkt ){ sendToNode(nid, pkt.release());};
	/// sends a packet
	void sendToNode( nodeid_t nid, NodePkt* pkt );

	/// sends a packet to all neighbors
	inline void sendToNeighbors( NodePkt_p pkt ){ sendToNeighbors(pkt.release());};
	/// sends a packet to all neighbors
	void sendToNeighbors( NodePkt* pkt );

	// ------------------------------------------------ methods to implement --
protected:
	/// called at initialization time of the Protocol
	virtual void onInitialize() = 0;

	/// called when a new node is connected
	virtual void onConnect( nodeid_t nid ) = 0;

	/// called when a node is disconnected
	virtual void onDisconnect( nodeid_t nid ) = 0;

	/// on message
	virtual void onMessage( NodePkt_p pkt ) = 0;

	/// reset state of Protocol to initial state (Node Failure)
	virtual void reset() = 0;

	// ------------------------------------------------------------- private --
private:
	Node* parent;
	virtual void initialize();
	virtual void handleMessage(cMessage *msg);
};

/**
 * Protocol class for legacy code that needs unwraped pointers.
 * @author Jakob Thomas
 */
class LegacyProtocol : public Protocol {
	/// on message
	virtual void onMessage( NodePkt_p pkt ) { onMessage(pkt.release());}
	/// on message for code that needs unwraped pointers
	virtual void onMessage( NodePkt* pkt ) = 0;
};

} //namespace

#endif
