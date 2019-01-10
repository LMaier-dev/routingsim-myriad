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
	// hello packets do not include my key
	if (handleIfFitting<KadHello>(packet)) return;
	// other packets should do so
	ASSERT(packet->getCurrentHop() == myOverlayKey);
	if (packet->hasRemainingHops()){
		// we are not the recipient
		packet->advanceProgress(); // increase hop count
		routePacket(std::move(packet));
		return;
	}
	else {
		// no hops left, packet is adressed to me
		if (handleIfFitting<KadAnswer>(packet)) return;
		if (handleIfFitting<KadPing>(packet)) return;
	}
	handleStrandedPacket(std::move(packet));
}

void R2Kademlia::reset() {
	EV << "resetting state of R2Kademlia" << endl;
	neighborTable.clear();
	//TODO routing table clear
}

void R2Kademlia::handlePacket(KadHello_p packet) {
	EV << "recieved hello packet from " << packet->getSourceId() << endl;
	neighborTable.addNeighbor(packet->getSourceId(), packet->getSource());
	ping(packet->getSource());
	//TODO add neighbor to routing table
}

void R2Kademlia::handlePacket(KadPing_p packet) {
	EV << "recieved ping packet from " << packet->getSource() << endl;
	KadPong_p pong = std::make_unique<KadPong>();
	fillAnswer(*packet, *pong);
	routePacket(std::move(pong));
}

void R2Kademlia::handlePacket(KadAnswer_p packet) {
	nonce_t nonce = packet->getNonce();
	handleAnswer(nonce, std::move(packet));
}

void R2Kademlia::handlePacket(KadPacket_p packet) {
	EV_ERROR << "dropping unhandled packet..." << endl;
}

void R2Kademlia::routePacket(KadPacket_p packet, NodePkt_p nodePkt) {
	// we do not route hello packets!
	ASSERT(packet->getType() != KAD_HELLO);
	//TODO we could check if the target is a neighbor...
	const OverlayKey& nextHop = packet->getNextHop();
	nodeid_t nid = neighborTable.getNodeId(nextHop);
	if (nid != undefined_node) {
		nodePkt->setType(PAYLOAD_TYPE);
		nodePkt->encapsulate(packet.release());
		sendToNode(nid, std::move(nodePkt));
	}
	else {
		onMissingNeighbor(std::move(packet), std::move(nodePkt));
	}
}
// -------------------- rpc methods and classes ---------------------------


class R2Kademlia::PingRPCState : public RPCProtocol::RPCState {
	void handleAnswer(cPacket_p answer) override {
		KadPong_p pong = packet_cast<KadPong>(answer);
		EV << "got ping answer from " << pong->getSource();
		EV << " with nonce " << nonce << endl;
	}
};

void R2Kademlia::ping(const OverlayKey& key) {
	auto ping = std::make_unique<KadPing>();
	ping->setNonce(registerState(std::make_unique<PingRPCState>()));
	ping->appendHop(key);
	ping->setSource(myOverlayKey);
	routePacket(std::move(ping));
	EV << "ping sent";
}

// ------------------------- policy foo --------------------------------------

void R2Kademlia::onMissingNeighbor(KadPacket_p packet, NodePkt_p pkt) {
	EV_WARN << "missing neighbor" << packet->getTarget() << endl;
};
void R2Kademlia::onUnknownNonce(nonce_t nonce, cPacket_p answer) {
	EV_WARN << "recieved answer packet with unknown nonce" << endl;
};

// -------------------- private helper methods --------------------
NodePkt_p R2Kademlia::makeHelloPacket()
{
	KadHello_p hello = std::make_unique<KadHello>();
	OverlayKey k(0);
	hello->appendHop(k);
	hello->setSource(myOverlayKey);
	hello->setSourceId(node().getNodeId());
	hello->setNonce(0);
	NodePkt_p nodePkt = std::make_unique<NodePkt>();
	nodePkt->setType(PAYLOAD_TYPE);
	nodePkt->encapsulate(hello.release());
	return nodePkt;
}

void R2Kademlia::fillAnswer(KadPacket& original, KadPacket& answer) {
	answer.setReturnPathFrom(original);
	ASSERT(answer.getSource() == myOverlayKey);
	answer.setNonce(original.getNonce());
}

template<typename T> bool R2Kademlia::handleIfFitting(KadPacket_p& packet) {
	ASSERT(packet); // no nullptr here!
	KadPacket *raw_ptr = packet.get();
	std::unique_ptr<T> tPacket(dynamic_cast<T*>(raw_ptr));
	// if cast failed we get nullptr -> skip the rest, return false
	if(not tPacket) return false;
	// invalidate the original unique_ptr
	packet.release();
	// delegate responsibility to handlePacket method
	handlePacket(std::move(tPacket));
	return true;
}

template<typename T> std::unique_ptr<T> packet_cast(cPacket_p& packet) {
	ASSERT(packet); // no nullptr here!
	cPacket *raw_ptr = packet.get();
	std::unique_ptr<T> tPacket(dynamic_cast<T*>(raw_ptr));
	// if tPacket is not nullptr the cast succeded
	// -> we need to invalidate the original pointer
	if(tPacket)
		packet.release();
	return tPacket;
}
} //namespace
