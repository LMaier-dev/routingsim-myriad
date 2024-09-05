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

#include "Protocol.h"

namespace routingsim {

//Define_Module(Protocol);
Register_Abstract_Class(Protocol);

void Protocol::initialize(int stage) {

	// Protocol entity is created at initialization stage 1
	if (stage == 1) {
		parent = Node::getOtherNode(gate("toNode$o"));
		assert(parent!=NULL);
		this->onInitialize();
	}
}

void Protocol::handleMessage(cMessage *msg) {
	if (msg->isSelfMessage()) {
		onSelfMessage(std::unique_ptr<cMessage>(msg));
		return;
	}

#ifdef NDEBUG
	NodePkt* np = reinterpret_cast<NodePkt*>(msg);
#else
	NodePkt* np = check_and_cast<NodePkt*>(msg);
#endif
	switch( np->getType() ) {
	case PAYLOAD_HELLO:
		this->onConnect(np->getHop().from);
		delete np;
		return;
	case PAYLOAD_GOODBYE:
		this->onDisconnect(np->getHop().from);
		delete np;
		return;
	default:
		break;
	}
	this->onMessage(std::unique_ptr<NodePkt>(np));
}

/// Bind module to node to receive messages of the given type
void Protocol::bind(int type) {
	node().bind(type, this);
}

/// Unbind module to node to receive messages of the given type no more
void Protocol::unbind(int type) {
	node().unbind(type, this);
}

/// sends a packet
void Protocol::sendToNode( nodeid_t nid, NodePkt* pkt, simtime_t delay) {
	pkt->getHopForUpdate().to = nid;

	if (delay == 0)
		cSimpleModule::send(pkt, "toNode$o");
	else
		cSimpleModule::sendDelayed(pkt, delay, "toNode$o");
}


/// sends a packet
void Protocol::sendToNeighbors( NodePkt* pkt) {
	pkt->getHopForUpdate().to = neighbor_cast;
	cSimpleModule::send(pkt, "toNode$o");
}

} // namespace
