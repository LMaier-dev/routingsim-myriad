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
#include "KadRoutingTable.h"

namespace routingsim {
// for std::bind with callback functions
using namespace std::placeholders;

Define_Module(R2Kademlia);

void R2Kademlia::onInitialize() {
	EV << "R2Kademlia::onInitialize()" << endl;
	myOverlayKey = OverlayKey::random(getRNG(0));
	bind(PAYLOAD_TYPE);
	//TODO parameters for bucketSize and accelerated lookup parameter
	routingTable.init(myOverlayKey, this);
	sendToNeighbors(makeHelloPacket());
	WATCH(myOverlayKey);
	WATCH(neighborTable);
	WATCH_OBJ(routingTable);
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
		EV_TRACE << "i will route this..." << endl;
		// TODO learn routes form passing packets
		// TODO learn about failiures from passing packets
		// we are not the recipient
		routePacket(std::move(packet), std::move(pkt));
		return;
	}
	else {
		// no hops left, packet is adressed to me
		//TODO learn route from packet
		EV_TRACE << "this is for me!" << endl;
		if (handleIfFitting<KadAnswer>(packet)) return;
		if (handleIfFitting<KadPing>(packet)) return;
		if (handleIfFitting<KadGetNeighborsCall>(packet)) return;
		EV_ERROR << "dropping unhandled packet of type " << packet->getType() << endl;
	}
}

void R2Kademlia::reset() {
	EV << "resetting state of R2Kademlia" << endl;
	neighborTable.clear();
	routingTable = KadRoutingTable();
	//TODO optimization: implement clear function in routing table
}

void R2Kademlia::handlePacket(KadHello_p packet) {
	EV << "recieved hello packet from " << packet->getSourceId() << endl;
	neighborTable.addNeighbor(packet->getSourceId(), packet->getSource());
	EV_TRACE << "neighbor table: "<< neighborTable << endl;
	//TODO special treatment for neighbors?
	onDiscovery(packet->getSource(), kad_path());
}

void R2Kademlia::handlePacket(KadAnswer_p packet) {
	EV << "received answer packet" << endl;
	if (packet->getType() == KAD_ERROR)
		onErrorPacket(*check_and_cast<KadError *>(packet.get()));
	handleAnswer(std::move(packet));
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
void R2Kademlia::sendPacket(KadPacket_p packet) {
	ASSERT(packet->getSource() == myOverlayKey);
	routePacket(std::move(packet), std::make_unique<NodePkt>(packet->getName()));
}

// -------------------- rpc methods ---------------------------


// PING
void R2Kademlia::sendPing(KadContact_p contact) {
	sendRpcCall(makePacket<KadPing>(contact, "KadPing"), std::bind(&R2Kademlia::handlePingAnswer, this, _1, _2));
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
	const OverlayKey& key = pong->getSource();
	KadContact_p contact = routingTable.getContact(key);
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
		routingTable.addContact(contact);
	}
}

// GET NEIGHBOR
void R2Kademlia::sendGetNeighbors(KadContact_p contact)
{
	EV_TRACE << "sending KadGetNeighbors to " << *contact << endl;
	sendRpcCall(
		makePacket<KadGetNeighborsCall>(contact, "KadGetNeighbors"),
		std::bind(&R2Kademlia::handleGetNeighborsAnswer, this, _1, _2)
	);
}

void R2Kademlia::handlePacket(std::unique_ptr<KadGetNeighborsCall> packet) {
	EV_TRACE << "recieved KadGetNeighbors from " << packet->getSource() << endl;
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
	EV_TRACE << "recieved " << answ->getNeighborsArraySize() << " neighbors from " << answ->getSource().toString().substr(6) << endl;
	for (uint i=0; i < answ->getNeighborsArraySize(); i++) {
		const OverlayKey& key = answ->getNeighbors(i);
		if(key == myOverlayKey) continue;
		else onPathDiscovery(key, kad_path(), answ->getSource());
	}
}

// FIND NODE
void R2Kademlia::sendFindNode(const OverlayKey& key) {
	auto contacts = routingTable.getClosestContacts(key, 1);
	// TODO select more and choose
	ASSERT(contacts.size() == 1);
	KadContact_p contact = contacts.front();
	auto packet = makePacket<KadFindNodeCall>(contact, "FindNode");
	packet->setQuery(key);
	sendRpcCall(
		std::move(packet),
		std::bind(&R2Kademlia::handleFindNodeAnswer, this, _1, _2)
	);
}
void R2Kademlia::handlePacket(std::unique_ptr<KadFindNodeCall> packet) {
	auto answer = makeAnswer<KadFindNodeAnswer>(*packet);
	if (packet->getQuery() == myOverlayKey) {
		sendPacket(std::move(answer));
		return;
	}
	auto contacts = routingTable.getClosestContacts(packet->getQuery(), findNodeNumber);
	answer->setContactArraySize(contacts.size());
	answer->setPathArraySize(contacts.size());
	for(size_t i = 0; i < contacts.size(); i++) {
		answer->setContact(i, contacts[i]->getKey());
		answer->setPath(i, contacts[i]->getPath());
	}
	sendPacket(std::move(answer));
}
void R2Kademlia::handleFindNodeAnswer(KadPacket_p answer_, RPCState& rpcState) {
	if (rpcState.status != RPC_SUCCESS) return;
	auto answer = safe_packet_cast<KadFindNodeAnswer>(answer_);
	ASSERT(answer->getContactArraySize() == answer->getPathArraySize());
	for (size_t i = 0; i < answer->getContactArraySize(); i++) {
		onPathDiscovery(
			answer->getContact(i),
			std::move(answer->getPath(i)),
			answer->getSource()
		);
	}
	// TODO try again
}
// ------------------------- policy foo --------------------------------------

void R2Kademlia::onMissingNeighbor(KadPacket_p packet, NodePkt_p pkt) {
	returnToSender(KAD_NEXT_HOP_UNAVAILABLE, std::move(packet), std::move(pkt));
}

void R2Kademlia::onErrorPacket(KadError& error) {
	if (error.getErrorType() == KAD_NEXT_HOP_UNAVAILABLE) {
		KadPacket* original = check_and_cast<KadPacket *>(error.getEncapsulatedPacket());
		ASSERT(original->getNonce() == error.getNonce());
		ASSERT(original->getSource() == myOverlayKey);
		EV_INFO << "Removing contact " << original->getTarget() << " because of error packet" << endl;
		routingTable.removeContact(original->getTarget());
	}
	// currently there are no more error types
}

void R2Kademlia::onDiscovery(const OverlayKey& key, kad_path path) {
	KadContact_p contact = routingTable.getContact(key);
	if (contact) {
		EV << "double discovery " << contact << endl;
		//TODO update contact
	}
	else
		routingTable.addCandidate(std::make_shared<KadContact>(key, std::move(path)));
}

void R2Kademlia::onPathDiscovery(const OverlayKey& key, kad_path path, const OverlayKey& via) {
	// find via contact
	KadContact_p viaContact{nullptr};
	viaContact = routingTable.getContact(via);
	if(!viaContact)
		throw cRuntimeError("VIA contact not known");

	// find contact
	KadContact_p contact = routingTable.getContact(key);
	if(contact) {
		// contact known
		if(contact->getPathLength() > viaContact->getPathLength() + 1 + path.size()) {
			contact->setPath(concatViaPath(viaContact->getPath(), via, path));
		}
	}
	else {
		// contact unknown
		onDiscovery(key, concatViaPath(viaContact->getPath(), via, path));
	}
}
kad_path R2Kademlia::concatViaPath(const kad_path& viaPath, const OverlayKey& viaKey, const kad_path& path) {
	kad_path newPath{};
	newPath.reserve(viaPath.size() + 1 + path.size());
	std::copy(viaPath.begin(), viaPath.end(), std::back_inserter(newPath));
	newPath.push_back(viaKey);
	std::copy(path.begin(), path.end(), std::back_inserter(newPath));
	return newPath;
}

void R2Kademlia::onBucketSpace(KadBucket& bucket) {
	KadContact_p replacement = bucket.contactReplacements.back();
	bucket.contactReplacements.pop_back();
	routingTable.addContact(replacement);
	// TODO remove this
	sendGetNeighbors(replacement);
}
void R2Kademlia::onBucketEmpty(KadBucket& bucket) {
	EV << "bucket empty" << endl;
	// TODO probe
}
void R2Kademlia::onBucketRefresh(KadBucket& bucket) {
	// TODO probe?
}
void R2Kademlia::onBucketFull(KadBucket& bucket, KadContact_p contact) {
	EV << "bucket full, dropping contact " << contact << endl;
	// TODO contact replacement
}

// -------------------- private helper methods --------------------
NodePkt_p R2Kademlia::makeHelloPacket()
{
	KadHello_p hello = std::make_unique<KadHello>();
	OverlayKey k(0);
	hello->setName("KadHello");
	hello->appendHop(k);
	hello->setSource(myOverlayKey);
	hello->setSourceId(node().getNodeId());
	hello->setNonce(0);
	NodePkt_p nodePkt = std::make_unique<NodePkt>(hello->getName());
	nodePkt->setType(PAYLOAD_TYPE);
	nodePkt->encapsulate(hello.release());
	return nodePkt;
}

template<typename T>
std::unique_ptr<T> R2Kademlia::makePacket(KadContact_p contact, const char *name) {
	auto packet = std::make_unique<T>();
	packet->setName(name);
	// TODO optimization: do this in one go
	packet->setSource(myOverlayKey);
	packet->appendRoute(contact->getPath());
	packet->appendHop(contact->getKey());
	return packet;
}

template<typename T>
std::unique_ptr<T> R2Kademlia::makeAnswer(KadPacket& original) {
	auto answer = std::make_unique<T>();
	answer->setName((std::string(original.getName()) + std::string("Answer")).c_str());
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
