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
	myOverlayKey = OverlayKey::random(getRNG(0));
	bind(PAYLOAD_TYPE);
	sendToNeighbors(makeHelloPacket());
}

void R2Kademlia::onConnect( nodeid_t nid ) {
	sendToNode(nid, makeHelloPacket());
}

void R2Kademlia::onDisconnect( nodeid_t nid ) {
	neighborTable.removeNeighbor(nid);
	//TODO remove neighbor from routing table
}

void R2Kademlia::onMessage( NodePkt_p pkt ) {
	EV << "received msg" << endl;
	ASSERT(pkt->getType() == PAYLOAD_TYPE);
	KadPacket_p packet = \
		std::unique_ptr<KadPacket>(check_and_cast<KadPacket*>(pkt->decapsulate()));
	// check if recipient
	if(packet->getTarget() == myOverlayKey || packet->getType() == HELLO)
		handlePacket(std::move(packet));
	else if (true /* source route end*/)
		handleStrandedPacket(std::move(packet));
	else // route packet according to source route
		routePacket(std::move(packet), std::move(pkt));
}

void R2Kademlia::reset() {
	EV << "resetting state of R2Kademlia" << endl;
	neighborTable.clear();
	//TODO routing table clear
}

void R2Kademlia::handlePacket(KadHello_p packet) {
	EV << "recieved hello packet from " << packet->getSourceId() << endl;
	neighborTable.addNeighbor(packet->getSourceId(), packet->getSource());
	//TODO add neighbor to routing table
}

void R2Kademlia::handlePacket(KadPacket_p packet) {
	EV_ERROR << "dropping unhandled packet..." << endl;
}

void R2Kademlia::routePacket(KadPacket_p packet, NodePkt_p nodePkt) {
	//TODO forward using source routing
	ASSERT(packet->getType() != HELLO);
	OverlayKey& target = packet->getTarget();
	nodeid_t nid = neighborTable.getNodeId(target);
	if(nid == undefined_node) {
		onMissingNeighbor(std::move(packet), std::move(nodePkt));
		return;
	}
	//TODO set type
	nodePkt->setType(PAYLOAD_TYPE);
	nodePkt->encapsulate(packet.release());
	sendToNode(nid, std::move(nodePkt));
}


NodePkt_p R2Kademlia::makeHelloPacket()
{
	KadHello_p hello = std::make_unique<KadHello>();
	hello->setTarget(0);
	hello->setSource(myOverlayKey);
	hello->setSourceId(node().getNodeId());
	hello->setNonce(0);
	NodePkt_p nodePkt = std::make_unique<NodePkt>();
	nodePkt->setType(PAYLOAD_TYPE);
	nodePkt->encapsulate(hello.release());
	return nodePkt;
}

} //namespace
