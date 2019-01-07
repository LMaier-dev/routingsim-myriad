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

#include "R2Kademlia.h"
#include "BaseTypes.h"

namespace routingsim {

Define_Module(R2Kademlia);

void R2Kademlia::onInitialize() {
	EV << "R2Kademlia::onInitialize()" << endl;
}

void R2Kademlia::onConnect( nodeid_t nid ) {
	EV << "Now I am connected to " << nid << endl;
}

void R2Kademlia::onDisconnect( nodeid_t nid ) {
	EV << "Now I am disconnected from " << nid << endl;
}

void R2Kademlia::onMessage( NodePkt_p pkt ) {
	EV << "received msg" << endl;
	//TODO assert type
	KadPacket_p packet = \
		std::unique_ptr<KadPacket>(check_and_cast<KadPacket*>(pkt->decapsulate()));
	// check if recipient
	if(packet->getTarget() == myOverlayKey)
		handlePacket(std::move(packet));
	else if (true /* source route end*/)
		handleStrandedPacket(std::move(packet));
	else // route packet according to source route
		routePacket(std::move(packet), std::move(pkt));
}

void R2Kademlia::reset() {
	EV << "resetting state of R2Kademlia" << endl;
}


void R2Kademlia::routePacket(KadPacket_p packet, NodePkt_p nodePkt) {
	OverlayKey& target = packet->getTarget();
	nodeid_t nid = getNeighbor(target);
	if(nid == undefined_node) {
		onMissingNeighbor(std::move(packet), std::move(nodePkt));
		return;
	}
	//TODO set type
	nodePkt->encapsulate(packet.release());
	sendToNode(nid, std::move(nodePkt));
}

} //namespace
