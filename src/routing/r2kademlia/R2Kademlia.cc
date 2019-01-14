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
// for std::bind with callback functions
using namespace std::placeholders;

Define_Module(R2Kademlia);

void R2Kademlia::onInitialize() {
	EV << "R2Kademlia::onInitialize()" << endl;
	myOverlayKey = OverlayKey::random(getRNG(0));
	bind(PAYLOAD_TYPE);
	//TODO parameters for bucketSize and b (accelerated lookup parameter)
	routingTable = std::make_unique<KadRoutingTable>(myOverlayKey);
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
	ASSERT(pkt->getType() == PAYLOAD_TYPE);
	KadPacket_p packet = \
		std::unique_ptr<KadPacket>(check_and_cast<KadPacket*>(pkt->decapsulate()));
	EV_TRACE << "received KadPacket of type " << (int)packet->getType() << endl;
	EV_TRACE << "my key " << myOverlayKey << endl;
	EV_TRACE << "source " << packet->getSource() << endl;
	EV_TRACE << "target " << packet->getTarget() << endl;
	// hello packets do not include my key
	if (handleIfFitting<KadHello>(packet)) return;
	// other packets should do so
	packet->advanceProgress(); // increase hop count
	ASSERT(packet->getCurrentHop() == myOverlayKey);
	if (packet->hasRemainingHops()){
		// we are not the recipient
		routePacket(std::move(packet), std::move(pkt));
		return;
	}
	else {
		// no hops left, packet is adressed to me
		//TODO learn route from packet
		if (handleIfFitting<KadAnswer>(packet)) return;
		if (handleIfFitting<KadPing>(packet)) return;
		if (handleIfFitting<KadGetNeighborsCall>(packet)) return;
	}
	handleStrandedPacket(std::move(packet));
}

void R2Kademlia::reset() {
	EV << "resetting state of R2Kademlia" << endl;
	neighborTable.clear();
	routingTable = std::make_unique<KadRoutingTable>(myOverlayKey);
	//TODO optimization: implement clear function in routing table
}

void R2Kademlia::handlePacket(KadHello_p packet) {
	EV << "recieved hello packet from " << packet->getSourceId() << endl;
	neighborTable.addNeighbor(packet->getSourceId(), packet->getSource());
	//TODO special treatment for neighbors?
	routingTable->onDiscovery(std::make_shared<KadContact>(packet->getSource()));
}

void R2Kademlia::handlePacket(KadAnswer_p packet) {
	handleAnswer(std::move(packet));
}

void R2Kademlia::handlePacket(KadPacket_p packet) {
	EV_ERROR << "dropping unhandled packet..." << endl;
}

void R2Kademlia::routePacket(KadPacket_p packet, NodePkt_p nodePkt) {
	// we do not route hello packets!
	ASSERT(packet->getType() != KAD_HELLO);
	ASSERT(packet->hasRemainingHops());
	//TODO optimization: we could check if the target is a neighbor...
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
// -------------------- rpc methods ---------------------------


// PING
void R2Kademlia::sendPing(KadContact_p contact) {
	sendRpcCall(makePacket<KadPing>(contact), std::bind(&R2Kademlia::handlePingAnswer, this, _1, _2));
	EV_TRACE << "ping sent to" << contact->getKey() << endl;
}

void R2Kademlia::handlePacket(KadPing_p packet) {
	EV_TRACE << "recieved ping packet from " << packet->getSource() << endl;
	sendPacket(makeAnswer<KadPong>(*packet));
}

void R2Kademlia::handlePingAnswer(KadPacket_p answer, RPCState& rpcState) {
	if (rpcState.status != RPC_SUCCESS) return;
	ASSERT(answer->getSource() == rpcState.original_packet->getTarget());
	KadPong_p pong = safe_packet_cast<KadPong>(answer);
	EV_TRACE << "got ping answer from " << pong->getSource();
	EV_TRACE << " with nonce " << pong->getNonce() << endl;
	//TODO add contact to routing table and mark seen
	const OverlayKey& key = answer->getSource();
	KadContact_p contact = routingTable->getContact(key);
	//TODO check if return path was the same
	kad_path path = rpcState.original_packet->getIntermediatePath();
	if (contact){
		// update contact
		contact->seen();
		if (contact->isStale() or (contact->getPathLength() > path.size()))
			contact->setPath(path);
		contact->setStale(false);
	}
	else {
		// create new contact
		contact = std::make_shared<KadContact>(key, path);
		contact->seen();
		routingTable->addContact(contact);
	}
}

// GET NEIGHBOR
void R2Kademlia::sendGetNeighbors(KadContact_p contact)
{
	sendRpcCall(
		makePacket<KadGetNeighborsCall>(contact),
		std::bind(&R2Kademlia::handleGetNeighborsAnswer, this, _1, _2)
	);
}

void R2Kademlia::handlePacket(std::unique_ptr<KadGetNeighborsCall> packet) {
	auto answer = makeAnswer<KadGetNeighborsAnswer>(*packet);
	std::vector<OverlayKey> neighbors = neighborTable.getNeighbors();
	answer->setNeighborsArraySize(neighbors.size());
	for (uint i = 0; i < neighbors.size(); i++)
		answer->setNeighbors(i, neighbors[i]);
	sendPacket(std::move(answer));
}

void R2Kademlia::handleGetNeighborsAnswer(KadPacket_p answer, RPCState& rpcState)
{
	if (rpcState.status != RPC_SUCCESS) return;
	auto answ = safe_packet_cast<KadGetNeighborsAnswer>(answer);
	EV_TRACE << "recieved get neighbors answer" << endl;
	for (uint i=0; i < answ->getNeighborsArraySize(); i++) {
		const OverlayKey& key = answ->getNeighbors(i);
		KadContact_p contact = routingTable->getContact(key);
		if (contact.get() == nullptr) {
			// contact is not known
			kad_path path = routingTable->getContact(answ->getSource())->getPath();
			path.push_back(answ->getSource());
			routingTable->onDiscovery(std::make_shared<KadContact>(key, path));
		}
		else if (contact->getPathLength() > answ->getRouteArraySize()-1) {
			// contact is known, but we found a better path
			kad_path path = routingTable->getContact(answ->getSource())->getPath();
			path.push_back(answ->getSource());
			contact->setPath(path);
		}
	}
}

// ------------------------- policy foo --------------------------------------

void R2Kademlia::onMissingNeighbor(KadPacket_p packet, NodePkt_p pkt) {
	returnToSender(KAD_NEXT_HOP_UNAVAILABLE, std::move(packet), std::move(pkt));
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

template<typename T>
std::unique_ptr<T> R2Kademlia::makePacket(KadContact_p contact) {
	auto packet = std::make_unique<T>();
	// TODO optimization: do this in one go
	packet->setSource(myOverlayKey);
	packet->appendRoute(contact->getPath());
	packet->appendHop(contact->getKey());
	return packet;
}

template<typename T>
std::unique_ptr<T> R2Kademlia::makeAnswer(KadPacket& original) {
	auto answer = std::make_unique<T>();
	answer->setReturnPathFrom(original);
	ASSERT(answer->getSource() == myOverlayKey);
	answer->setNonce(original.getNonce());
	return answer;
}

void R2Kademlia::returnToSender(uint8_t errorType, KadPacket_p packet, NodePkt_p nodePkt) {
	KadError_p answer = makeAnswer<KadError>(*packet);
	answer->setErrorType(errorType);
	answer->encapsulate(packet.release());
	routePacket(std::move(answer), std::move(nodePkt));
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

template<typename T> std::unique_ptr<T> safe_packet_cast(KadPacket_p& packet) {
	ASSERT(packet); // no nullptr here!
	// this will throw cRuntimeError if the cast fails
	return std::unique_ptr<T>(check_and_cast<T*>(packet.release()));
}
} //namespace
