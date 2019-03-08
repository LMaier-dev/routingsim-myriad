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

#include <BaseTypes.h>

#include "Statistics.h"

#include "R2Kademlia.h"
#include "KadRoutingTable.h"

namespace routingsim {
// for std::bind with callback functions
using namespace std::placeholders;

Define_Module(R2Kademlia);

void R2Kademlia::onInitialize() {
	EV << "R2Kademlia::onInitialize()" << endl;
	myOverlayKey = OverlayKey::random(getRNG(0));
	bind(PAYLOAD_TYPE);

	ASSERT(par("accelerationFactor").intValue() > 0);
	ASSERT(par("bucketSize").intValue() > 1);
	routingTable.init(
		myOverlayKey, this,
		par("accelerationFactor").intValue(),
		par("bucketSize").intValue());

	ASSERT(par("discoveryRadius").intValue() > 0);
	discoveryRadius = (uint)par("discoveryRadius").intValue();
	ASSERT(par("findNodeNumber").intValue() > 0);
	findNodeNumber = (size_type)par("findNodeNumber").intValue();

	getStatistics().addSample(this);
	addTimer(4, std::bind(&R2Kademlia::findRandom, this));
	WATCH(myOverlayKey);
	WATCH(neighborTable);
	WATCH_OBJ(routingTable);

	sendToNeighbors(makeHelloPacket());
}

void R2Kademlia::onConnect( nodeid_t nid ) {
	sendToNode(nid, makeHelloPacket());
}

void R2Kademlia::onDisconnect( nodeid_t nid ) {
	// if this node is taken down, the packets arrive after the reset
	if(neighborTable.size() == 0) return;
	OverlayKey key = neighborTable.getOverlayKey(nid);
	ASSERT(!key.isUnspecified());
	ASSERT(neighborTable.getNodeId(key) == nid );

	neighborTable.removeNeighbor(nid);
	ASSERT(neighborTable.getNodeId(key) == undefined_node);
	if(!routingTable.getContact(key)) {
		//TODO why does this happen?????
		EV_ERROR << "missing contact" << nid << " " << key << " on node " << node().getNodeId() << endl;
		return;
	}
	routingTable.removeContact(key);
	//TODO remove contacts from routing table with this neighbor as next hop
}

void R2Kademlia::onMessage( NodePkt_p pkt ) {
	ASSERT(pkt->getType() == PAYLOAD_TYPE);
	KadPacket_p packet = \
		std::unique_ptr<KadPacket>(check_and_cast<KadPacket*>(pkt->decapsulate()));
	// hello packets do not include my key
	if (handleIfFitting<KadHello>(packet)) return;
	// other packets should do so
	packet->advanceProgress(); // increase hop count
	if (packet->getCurrentHop() != myOverlayKey) {
		EV_ERROR << "dropping packet, current hop does not fit!" << endl;
		//TODO how does this happen?
		droppedPackets++;
		return;
	}
	if (packet->hasRemainingHops()){
		// TODO learn routes form passing packets
		// TODO learn about failiures from passing packets
		// we are not the recipient
		forwardPacket(std::move(packet), std::move(pkt));
		return;
	}
	else {
		packet->setControlInfo(pkt.release());
		// no hops left, packet is adressed to me
		//TODO learn route from packet
		// these types may forward packets, so they count themselves
		if (handleIfFitting<KadFindNodeReq>(packet)) return;
		// these types never forward packets
		countRecieved(*packet);
		if (handleIfFitting<KadQueryRouteReq>(packet)) return;
		if (handleIfFitting<KadAnswer>(packet)) return;
		if (handleIfFitting<KadProbeReq>(packet)) return;
		EV_ERROR << "dropping unhandled packet of type " << (int)packet->getType() << endl;
		droppedPackets++;
	}
}

void R2Kademlia::onSelfMessage(cMessage_p msg) {
	// handle timer
	short timerId = msg->getKind();
	ASSERT(timerId >= 0);
	ASSERT((size_t)timerId < timerDuration.size());
	ASSERT(timerMsg[timerId] == msg.get());
	timerCallback[timerId]();
	// reschedule timer
	simtime_t time = uniform(timerDuration[timerId]/2, timerDuration[timerId]/2*3);
	scheduleAt(simTime() + time, msg.release());
}

void R2Kademlia::reset() {
	Enter_Method_Silent();
	EV << "resetting state of R2Kademlia" << endl;
	neighborTable.clear();
	routingTable = KadRoutingTable();
	unbind(PAYLOAD_TYPE);
	onInitialize();
	//TODO optimization: implement clear function in routing table
	//TODO clear timers
	//TODO clear rpc states
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
	if (packet->getType() == KAD_Error)
		onErrorPacket(*check_and_cast<KadError *>(packet.get()));
	handleAnswer(std::move(packet));
}

void R2Kademlia::forwardPacket(KadPacket_p packet, NodePkt_p nodePkt) {
	// we do not route hello packets!
	ASSERT(packet->getType() != KAD_PNHello);
	//TODO do stuff with hop count
	ASSERT(packet->hasRemainingHops());
	ASSERT(packet->getControlInfo() == nullptr);
	//TODO optimization: we could check if the target is a neighbor...
	const OverlayKey& nextHop = packet->getNextHop();
	nodeid_t nid = neighborTable.getNodeId(nextHop);
	if (nid != undefined_node) {
		nodePkt->setType(PAYLOAD_TYPE);
		nodePkt->encapsulate(packet.release());
		sendToNode(nid, std::move(nodePkt));
		forwardedPackets++;
	}
	else {
		onMissingNeighbor(std::move(packet), std::move(nodePkt));
	}
}

void R2Kademlia::resendPacket(KadPacket_p packet) {
	NodePkt_p nodePkt = NodePkt_p(check_and_cast<NodePkt *>(packet->removeControlInfo()));
	forwardPacket(std::move(packet), std::move(nodePkt));
}

void R2Kademlia::sendPacket(KadPacket_p packet) {
	ASSERT(packet->getSource() == myOverlayKey);
	ASSERT(packet->getControlInfo() == nullptr);
	auto nodePkt = std::make_unique<NodePkt>(packet->getName());
	getStatistics().registerMessage(nodePkt.get());
	sentPackets++;
	forwardPacket(std::move(packet), std::move(nodePkt));
}

// -------------------- rpc methods ---------------------------


// PING
void R2Kademlia::sendProbe(KadContact_p contact) {
	sendRpcCall(
		makePacket<KadProbeReq>(contact, "KadProbeReq"),
		std::bind(&R2Kademlia::handleProbeResponse, this, _1, _2));
}

void R2Kademlia::handlePacket(std::unique_ptr<KadProbeReq> packet) {
	sendPacket(makeAnswer<KadProbeRsp>(*packet));
}

bool R2Kademlia::handleProbeResponse(KadPacket_p answer, RPCState& rpcState) {
	if (rpcState.status != RPC_SUCCESS) return true;
	ASSERT(answer->getSource() == rpcState.original_packet->getTarget());
	std::unique_ptr<KadProbeRsp> probeRsp = safe_packet_cast<KadProbeRsp>(answer);
	//TODO add contact to routing table and mark seen
	const OverlayKey& key = probeRsp->getSource();
	KadContact_p contact = routingTable.getContact(key);
	//TODO check if return path was the same
	kad_path path = rpcState.original_packet->getIntermediatePath();
	if (contact){
		// update contact
		contact->seen();
		if (contact->isStale() or (contact->getPathLength() > path.size()))
			contact->setPath(std::move(path));
		contact->setStale(false);
	}
	else {
		// create new contact
		contact = std::make_shared<KadContact>(key, std::move(path));
		contact->seen();
		routingTable.addContact(contact);
	}
	return true;
}

// GET NEIGHBOR
void R2Kademlia::sendGetNeighbors(KadContact_p contact)
{
	EV_TRACE << "sending query route request to " << *contact << endl;
	auto req = makePacket<KadQueryRouteReq>(contact, "KadQueryRouteReq");
	req->setNeighborhood(1);
	req->setContactsOnly(true);
	sendRpcCall(
		std::move(req),
		std::bind(&R2Kademlia::handleQueryRouteResponse, this, _1, _2)
	);
}

void R2Kademlia::handlePacket(std::unique_ptr<KadQueryRouteReq> packet) {
	EV_TRACE << "recieved KadQueryRouteRsp from " << packet->getSource() << endl;
	ASSERT(packet->getNeighborhood() == 1);
	ASSERT(packet->getContactsOnly());
	auto response = makeAnswer<KadQueryRouteRsp>(*packet);
	std::vector<OverlayKey> neighbors = neighborTable.getNeighbors();
	response->setContactArraySize(neighbors.size());
	for (uint i = 0; i < neighbors.size(); i++)
		response->setContact(i, neighbors[i]);
	sendPacket(std::move(response));
}

bool R2Kademlia::handleQueryRouteResponse(KadPacket_p answer, RPCState& rpcState)
{
	if (rpcState.status != RPC_SUCCESS) return true;
	auto response = safe_packet_cast<KadQueryRouteRsp>(answer);
	ASSERT(response->getPathArraySize() == 0);
	EV_TRACE << "recieved " << response->getContactArraySize() << " neighbors from " << response->getSource().toString().substr(0, 6) << endl;
	for (uint i=0; i < response->getContactArraySize(); i++) {
		onPathDiscovery(response->getContact(i), kad_path(), response->getSource());
	}
	return true;
}

// FIND NODE
void R2Kademlia::sendFindNode(const OverlayKey& key) {
	EV_TRACE << "sendFindNode w/ " << key << endl;
	auto contacts = routingTable.getClosestContacts(key, 1);
	// TODO select more and choose
	ASSERT(contacts.size() == 1);
	KadContact_p contact = contacts.front();
	EV_TRACE << "closest contact found: " << *contact << endl;
	auto packet = makePacket<KadFindNodeReq>(contact, "FindNode");
	packet->setQuery(key);
	sendRpcCall(
		std::move(packet),
		std::bind(&R2Kademlia::handleFindNodeResponse, this, _1, _2)
	);
}
void R2Kademlia::handlePacket(std::unique_ptr<KadFindNodeReq> packet) {
	auto answer = makeAnswer<KadFindNodeRsp>(*packet);
	OverlayKey& query = packet->getQuery();
	if (query == myOverlayKey) {
		//TODO what if not exact needed, do we send contacts?
		countRecieved(*packet);
		sendPacket(std::move(answer));
		return;
	}
	KadContact_p contact = routingTable.getClosestContact(
		packet->getQuery(), packet->getSource());
	if (contact->getKey() == packet->getSource()) {
		countRecieved(*packet);
		// TODO error packet
		return;
	}
	//TODO if recursive routing
	if (routingTable.distance(query) < routingTable.distance(contact->getKey(), query)) {
		auto contacts = routingTable.getClosestContacts(packet->getQuery(), findNodeNumber);
		answer->setContactArraySize(contacts.size());
		answer->setPathArraySize(contacts.size());
		for(size_t i = 0; i < contacts.size(); i++) {
			answer->setContact(i, contacts[i]->getKey());
			answer->setPath(i, contacts[i]->getPath());
		}
		countRecieved(*packet);
		EV_TRACE << "KadFindNodeReq revieved query: " << packet->getQuery() << " answering with list size" << contacts.size() << endl;
		sendPacket(std::move(answer));
	}
	else {
		// forward request
		EV_TRACE << "KadFindNodeReq revieved query: " << packet->getQuery() << endl << " next hop is " << *contact << endl;
		packet->appendRoute(contact->getPath());
		packet->appendHop(contact->getKey());
		resendPacket(std::move(packet));
	}
}
bool R2Kademlia::handleFindNodeResponse(KadPacket_p answer_, RPCState& rpcState) {
	if (rpcState.status != RPC_SUCCESS) return true;
	auto answer = safe_packet_cast<KadFindNodeRsp>(answer_);
	ASSERT(answer->getContactArraySize() == answer->getPathArraySize());
	EV_TRACE << "recieved FindNodeRsp for " << check_and_cast<KadFindNodeReq *>(rpcState.original_packet.get())->getQuery();
	EV_TRACE << " from " << answer->getSource() << " with " << answer->getContactArraySize() << " contacts"<< endl;
	onPathDiscovery(answer->getSource(), answer->reversePath(answer->getIntermediatePath()));
	for (size_t i = 0; i < answer->getContactArraySize(); i++) {
		onPathDiscovery(
			answer->getContact(i),
			std::move(answer->getPath(i)),
			answer->getSource()
		);
	}
	// TODO try again
	return true;
}
// ------------------------- policy foo --------------------------------------

void R2Kademlia::onMissingNeighbor(KadPacket_p packet, NodePkt_p pkt) {
//  	getStatistics().dropMessage(pkt, undefined_node);
	droppedPackets++;
	if(packet->getType() == KAD_Error) {
		EV << "dropping error packet" << endl;
	}
	else
		returnToSender(KAD_NEXT_HOP_UNAVAILABLE, std::move(packet));
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
		if(path.size() == 0) {
			// neighbor discovery
			contact->setPath(path);
		}
		else{
			// this should never happen, path updates should be handled by
			// onPathDiscovery
			EV_ERROR << "double discovery " << *contact << endl;
		}
	}
	else {
		contact = std::make_shared<KadContact>(key, std::move(path));
		routingTable.addCandidate(contact);
		if(contact->getPathLength() < discoveryRadius) {
			sendGetNeighbors(contact);
		}
	}
}

void R2Kademlia::onPathDiscovery(const OverlayKey& key, kad_path path, const OverlayKey& via) {
	if (key == myOverlayKey) return; // nothing to do here
	// find via contact
	KadContact_p viaContact{nullptr};
	if (!via.isUnspecified()) {
		viaContact = routingTable.getContact(via);
		if(!viaContact) {
			EV_ERROR << "Failed to hande path discovery, via contact " << via << " was not found" << endl;
			return;
		}
	}
	// find contact
	KadContact_p contact = routingTable.getContact(key);
	if(contact) {
		// contact known
		size_t viaPathLength = viaContact ? viaContact->getPathLength() + 1 : 0;
		if(contact->getPathLength() > viaPathLength + path.size()) {
			if (viaContact)
				contact->setPath(concatViaPath(viaContact->getPath(), via, path));
			else
				contact->setPath(path);
		}
	}
	else {
		// contact unknown
		if (viaContact)
			onDiscovery(key, concatViaPath(viaContact->getPath(), via, path));
		else
			onDiscovery(key, path);
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


void R2Kademlia::sampleStatistics (Statistics &stats) {
	stats.value("forwardedPackets", forwardedPackets-sentPackets);
	stats.value("sentPackets", sentPackets);
	stats.value("receivedPackets", recievedPackets);
	stats.value("routingTableContacts", routingTable.getNumContacts());
	stats.value("routingTableBuckets", routingTable.getNumBuckets());
}
// -------------- kad policy -------------

void R2Kademlia::onBucketSpace(KadBucket& bucket) {
	ASSERT(bucket.replacementSize() > 0);
	KadContact_p replacement = bucket.contactReplacements.back();
	bucket.contactReplacements.pop_back();
	routingTable.addContact(replacement);
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


void R2Kademlia::addTimer(simtime_t duration, std::function<void()> callback) {
	ASSERT(timerDuration.size() == timerCallback.size());
	ASSERT(timerDuration.size() < std::numeric_limits<short>::max());
	ASSERT(duration > 0);
	timerDuration.push_back(duration);
	timerCallback.push_back(callback);
	cMessage *msg = new cMessage("TimerMsg", timerDuration.size()-1);
	timerMsg.push_back(msg);
	simtime_t initialOffset = uniform(0, duration);
	scheduleAt(simTime() + duration + initialOffset, msg);
	EV_TRACE << "sheduled timer msg " << msg << " with duration " << duration <<  endl;
}

void R2Kademlia::findRandom() {
	if (neighborTable.size() == 0) return; // we are isolated
	uint32_t index = intuniform(0, myOverlayKey.getLength()-1);
	OverlayKey probeKey = myOverlayKey;
	probeKey[index] ^= 1;
	sendFindNode(probeKey);
}

void R2Kademlia::countRecieved(KadPacket& packet) {
	recievedPackets++;
	getStatistics().deliveredMessage(
		check_and_cast<cMessage *>(packet.getControlInfo()),
		packet.getProgress()
	);
}

void R2Kademlia::returnToSender(uint8_t errorType, KadPacket_p packet) {
	KadError_p answer = makeAnswer<KadError>(*packet);
	answer->setErrorType(errorType);
	answer->encapsulate(packet.release());
	if (answer->hasRemainingHops())
		sendPacket(std::move(answer));
	else // we sent this packet!
		handleAnswer(std::move(answer));
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
