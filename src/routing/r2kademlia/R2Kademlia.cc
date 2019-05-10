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
#include "KadObserver.h"

namespace routingsim {
// for std::bind with callback functions
namespace ph = std::placeholders;

Define_Module(R2Kademlia);

void R2Kademlia::onInitialize() {
	EV << "R2Kademlia::onInitialize()" << endl;
	myOverlayKey = OverlayKey::random(getRNG(0));
	getKadObserver().registerUp(myOverlayKey, node().getNodeId());
	bind(PAYLOAD_TYPE);

	ASSERT(par("accelerationFactor").intValue() > 0);
	ASSERT(par("bucketSize").intValue() > 1);
	routingTable.init(
		myOverlayKey, this,
		par("accelerationFactor").intValue(),
		par("bucketSize").intValue(),
		par("proximityNeighborSelection").boolValue(),
		par("pathSimplification").boolValue());

	ASSERT(par("discoveryRadius").intValue() > 0);
	discoveryRadius = (uint)par("discoveryRadius").intValue();
	ASSERT(par("findNodeNumber").intValue() > 0);
	findNodeNumber = (size_type) par("findNodeNumber").intValue();
	proximityRouting = par("proximityRouting").boolValue();
	hopLimit = (uint) par("hopLimit").intValue();
	pathSnooping = par("pathSnooping").boolValue();
	probeStrategy = (uint) par("probeStrategy").intValue();
	detailedRoutingTableStats = par("detailedRoutingTableStats").boolValue();

	getStatistics().addSample(this);
	simtime_t probeInterval = (simtime_t) par("probeInterval").doubleValue();
	addTimer(probeInterval*1.5, probeInterval, std::bind(&R2Kademlia::doProbe, this));
	addTimer(probeInterval*1.5, probeInterval, std::bind(&R2Kademlia::doReachabilityTest, this));
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
		// this should only happen for nodes with very large degree
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
		countDropped(*packet, packet->getTarget());
		return;
	}
	if (packet->hasRemainingHops()){
		// we are not the recipient
		if (pathSnooping) {
			// aka PATH OVERHEARING
			//TODO optimization use isPathInteresting
			routingTable.addPath(packet->getSource(), packet->reversePath(packet->getPath(1, packet->getProgress())));
			routingTable.addPath(packet->getTarget(), packet->getPath(packet->getProgress()+1, packet->getRouteArraySize()-1));
		}
		//TODO ERROR OVERHEARING, more overhearing
		forwardPacket(std::move(packet), std::move(pkt));
		return;
	}
	else { // no hops left, packet is addressed to me
		packet->setControlInfo(pkt.release());
		// learn route from packet
		//TODO optimization use isPathInteresting
		routingTable.addPath(packet->getSource(), packet->reversePath(packet->getIntermediatePath())) ;

		// these types may forward packets, so they count themselves
		if (handleIfFitting<KadFindNodeReq>(packet)) return;
		// these types never forward packets
		countReceived(*packet);
		if (handleIfFitting<KadQueryRouteReq>(packet)) return;
		if (handleIfFitting<KadAnswer>(packet)) return;
		if (handleIfFitting<KadProbeReq>(packet)) return;
		EV_ERROR << "dropping unhandled packet of type " << (int)packet->getType() << endl;
		countDropped(*packet, packet->getTarget());
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
	getKadObserver().registerDown(myOverlayKey, node().getNodeId());
	neighborTable.clear();
	routingTable = KadRoutingTable();
	unbind(PAYLOAD_TYPE);
	onInitialize();
	//TODO optimization: implement clear function in routing table
	//TODO clear timers
	//TODO clear rpc states
}

void R2Kademlia::handlePacket(KadHello_p packet) {
	EV << "received hello packet from " << packet->getSourceId() << endl;
	neighborTable.addNeighbor(packet->getSourceId(), packet->getSource());
	EV_TRACE << "neighbor table: "<< neighborTable << endl;
	routingTable.addPath(packet->getSource(), kad_path());
}

void R2Kademlia::handlePacket(KadAnswer_p packet) {
	EV_DEBUG << "received answer packet" << endl;
	if (packet->getType() == KAD_Error)
		onErrorPacket(*check_and_cast<KadError *>(packet.get()));
	handleAnswer(std::move(packet));
}

void R2Kademlia::forwardPacket(KadPacket_p packet, NodePkt_p nodePkt) {
	// we do not route hello packets!
	ASSERT(packet->getType() != KAD_PNHello);
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
	getKadObserver().registerPacketOverlayHop(*packet);
	NodePkt_p nodePkt = NodePkt_p(check_and_cast<NodePkt *>(packet->removeControlInfo()));
	forwardPacket(std::move(packet), std::move(nodePkt));
}

void R2Kademlia::sendPacket(KadPacket_p packet) {
	ASSERT(packet->getSource() == myOverlayKey);
	ASSERT(packet->getControlInfo() == nullptr);
	auto nodePkt = std::make_unique<NodePkt>(packet->getName());
	if(packet->getType() == KAD_FindNodeReq && check_and_cast<KadFindNodeReq*>(packet.get())->getExact()) {
		getKadObserver().registerPacket(*packet);
		getStatistics().registerMessage(nodePkt.get());
	}
	sentPackets++;
	forwardPacket(std::move(packet), std::move(nodePkt));
}

const KadContact& R2Kademlia::chooseNextHop(const KadContactList& contacts, const OverlayKey& target, const OverlayKey& source) {
	if (proximityRouting && contacts.size() > 1) {
		const KadContact* contact = contacts.front();
		int dprefix = (contact->getKey() ^ target).log_2();
		size_t dproxMin = contact->getPathLength();
		if ((myOverlayKey ^ target).log_2() == dprefix) {
			// if we have no prefix progress,
			// we take the closest contact by XOR
			return *contact;
		}
		for (auto it = contacts.begin()+1; it != contacts.end(); it++) {
			if (((*it)->getKey() ^ target).log_2() > dprefix) break;
			if ((*it)->getPathLength() < dproxMin) {
				dproxMin = (*it)->getPathLength();
				contact = *it;
			}
		}
		return *contact;
	}
	else {
		// no proximity routing
		ASSERT(contacts.front()->getKey() != target);
		return *(contacts.front());
	}
}

// -------------------- RPC methods ---------------------------


// PING
void R2Kademlia::sendProbe(KadContact_p contact) {
	sendRpcCall(
		makePacket<KadProbeReq>(contact, "KadProbeReq"),
		std::bind(&R2Kademlia::handleProbeResponse, this, ph::_1, ph::_2));
}

void R2Kademlia::handlePacket(std::unique_ptr<KadProbeReq> packet) {
	sendPacket(makeAnswer<KadProbeRsp>(*packet));
}

bool R2Kademlia::handleProbeResponse(KadPacket_p answer, RPCState& rpcState) {
	if (rpcState.status != RPC_SUCCESS) return true;
	ASSERT(answer->getSource() == rpcState.original_packet->getTarget());
	std::unique_ptr<KadProbeRsp> probeRsp = safe_packet_cast<KadProbeRsp>(answer);
	return true;
}

// GET NEIGHBOR
void R2Kademlia::sendGetNeighbors(KadContact_p contact)
{
	EV_TRACE << "sending query route request to " << *contact << endl;
	auto req = makePacket<KadQueryRouteReq>(contact, "KadQueryRouteReq");
	req->setNeighborhood(1);
	req->setContactsOnly(false);
	sendRpcCall(
		std::move(req),
		std::bind(&R2Kademlia::handleQueryRouteResponse, this, ph::_1, ph::_2)
	);
}

void R2Kademlia::handlePacket(std::unique_ptr<KadQueryRouteReq> packet) {
	EV_TRACE << "received KadQueryRouteRsp from " << packet->getSource() << endl;
	ASSERT(packet->getNeighborhood() == 1);
	ASSERT(!packet->getContactsOnly());
	auto response = makeAnswer<KadQueryRouteRsp>(*packet);
	std::vector<OverlayKey> neighbors = neighborTable.getNeighbors();
	response->setContactArraySize(neighbors.size());
	response->setPathArraySize(neighbors.size());
	for (uint i = 0; i < neighbors.size(); i++)
		response->setContact(i, neighbors[i]);
	sendPacket(std::move(response));
}

bool R2Kademlia::handleQueryRouteResponse(KadPacket_p answer, RPCState& rpcState)
{
	if (rpcState.status != RPC_SUCCESS) return true;
	auto response = safe_packet_cast<KadQueryRouteRsp>(answer);
	EV_TRACE << "received " << response->getContactArraySize() << " neighbors from " << response->getSource().toString().substr(0, 6) << endl;
	readRTableRsp(std::move(response));
	return true;
}

// FIND NODE
void R2Kademlia::sendFindNode(const OverlayKey& key, bool exact) {
	EV_TRACE << "sendFindNode w/ " << key << endl;
	auto contacts = routingTable.getClosestContacts(key, findNodeNumber);
	ASSERT(contacts.size() > 0);
	const KadContact& contact = chooseNextHop(contacts, key, myOverlayKey);
	auto packet = makePacket<KadFindNodeReq>(contact, "FindNode");
	packet->setQuery(key);
	packet->setExact(exact);
	sendRpcCall(
		std::move(packet),
		std::bind(&R2Kademlia::handleFindNodeResponse, this, ph::_1, ph::_2)
	);
}
void R2Kademlia::handlePacket(std::unique_ptr<KadFindNodeReq> packet) {
	OverlayKey& query = packet->getQuery();
	size_t contactsLen = std::max((size_t)findNodeNumber, routingTable.getSplitSize());
	KadContactList contacts = routingTable.getClosestContacts(packet->getQuery(), contactsLen);
	ASSERT(contacts.size() > 0); // routing table has at least one entry
	if (query != myOverlayKey) {
		if (contacts.front()->getKey() == packet->getSource()) {
			if(contacts.size() == 1) {
				// the only contact we know is the source
				countDropped(*packet, query);
				returnToSender(KAD_NodeUnreachable, std::move(packet));
				return;
			}
			else
				contacts.erase(contacts.begin());
		}
		//TODO optimization: we should check if any contacts (not just front) are close enough
		if ((contacts.front()->getKey() ^ query) < (myOverlayKey ^ query)
				and contacts.front()->getPathLength() < packet->getHopLimit()) {
			// we have found a closer contact that is close enough
			// => forward request
			const KadContact& contact = chooseNextHop(contacts, query, packet->getSource());
			EV_TRACE << "KadFindNodeReq received query: " << packet->getQuery() << endl << " next hop is " << contact << endl;
			ASSERT((contact.getKey() ^ query).log_2() <= (myOverlayKey ^ query).log_2());
			if((contact.getKey() ^ query).log_2() <= (myOverlayKey ^ query).log_2()) {
				EV_WARN << "no prefix progress made" << endl;
			}
			ASSERT((contact.getKey() ^ query) < (myOverlayKey ^ query));
			packet->appendRoute(contact);
			ASSERT(packet->getRouteArraySize() - packet->getProgress() <= packet->getHopLimit());
			resendPacket(std::move(packet));
			return;
		}
		else if (packet->getExact()) {
			countDropped(*packet, query);
			//TODO: if we find a closer contact too far away, we should send HOP LIMIT EXCEEDED
			returnToSender(KAD_NodeUnreachable, std::move(packet));
			return;
		}
	}
	// answer request
	auto answer = makeAnswer<KadFindNodeRsp>(*packet);
	fillRTableRsp(*answer, contacts);
	countReceived(*packet);
	EV_TRACE << "KadFindNodeReq received query: " << packet->getQuery() << " answering with list size" << contacts.size() << endl;
	sendPacket(std::move(answer));
}

bool R2Kademlia::handleFindNodeResponse(KadPacket_p answer_, RPCState& rpcState) {
	if (rpcState.status != RPC_SUCCESS) return true;
	auto answer = safe_packet_cast<KadFindNodeRsp>(answer_);
	ASSERT(answer->getContactArraySize() == answer->getPathArraySize());
	EV_TRACE << "received FindNodeRsp for " << check_and_cast<KadFindNodeReq *>(rpcState.original_packet.get())->getQuery();
	EV_TRACE << " from " << answer->getSource() << " with " << answer->getContactArraySize() << " contacts"<< endl;
	readRTableRsp(std::move(answer));
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
		returnToSender(KAD_SegmentFailure, std::move(packet));
}

void R2Kademlia::onErrorPacket(KadError& error) {
	if (error.getErrorType() == KAD_SegmentFailure) {
		KadPacket* original = check_and_cast<KadPacket *>(error.getEncapsulatedPacket());
		ASSERT(original->getNonce() == error.getNonce());
		ASSERT(original->getSource() == myOverlayKey);
		EV_INFO << "Removing contact " << original->getTarget() << " because of error packet" << endl;
		routingTable.removeContact(original->getTarget());
	}
	// currently there are no more error types
}

void R2Kademlia::sampleStatistics (Statistics &stats) {
	stats.value("forwardedPackets", forwardedPackets-sentPackets);
	stats.value("sentPackets", sentPackets);
	stats.value("receivedPackets", receivedPackets);
	stats.value("routingTableContacts", routingTable.getNumContacts());
	stats.value("routingTableBuckets", routingTable.getNumBuckets());
	stats.value("routingTableUpdates", routingTable.getUpdateCount());
	stats.value("routingTableReplacements", routingTable.getReplacementCount());
	if(detailedRoutingTableStats) {
		stats.value("routingTablePathLength", routingTable.getAveragePathLength());
		stats.value("routingTablePathStretch", routingTable.getAveragePathStretch());
	}
}
// -------------- kad policy -------------

void R2Kademlia::onBucketEmpty(KadBucket& bucket) {
	EV << "bucket empty" << endl;
}
void R2Kademlia::onNewContact(KadContact_p contact) {
	if (contact->getPathLength() < discoveryRadius)
		sendGetNeighbors(contact);
}

void R2Kademlia::recordScalars() {
	recordScalar("forwardedPackets", forwardedPackets);
	recordScalar("droppedPackets", droppedPackets);
	recordScalar("sentPackets", sentPackets);
	recordScalar("receivedPackets", receivedPackets);
	recordScalar("routingTableContacts", routingTable.getNumContacts());
	recordScalar("routingTableBuckets", routingTable.getNumBuckets());
	recordScalar("routingTablePathLength", routingTable.getAveragePathLength());
	recordScalar("routingTablePathStretch", routingTable.getAveragePathStretch());
	recordScalar("routingTableUpdates", routingTable.getUpdateCount());
	recordScalar("routingTableReplacements", routingTable.getReplacementCount());
	nodeid_t nid =  Node::getNode(this)->getNodeId();
	recordScalar("nodeId", nid);
	recordScalar("degree", getNetworkManager()->getTopology().getNodeDegree(nid));
	recordScalar("popularity", getKadObserver().getPopularity(myOverlayKey));
}

void R2Kademlia::recordRoutingTable() {
	// these submodules are a hack to record scalars for the contacts
	// they are only used for scalar recording
	cModuleType* modType = cModuleType::get("routingsim.routing.r2kademlia.Dummy");
	auto allContacts = routingTable.getAllContacts();
	std::string contactStr{"contact"};
	for (size_type i; i < allContacts.size(); i++) {
		KadContact_p contact = allContacts[i];
		cModule* mod = modType->create((contactStr + std::to_string(i)).c_str(), this);
		mod->recordScalar("pathLength", contact->getPathLength()+1);
		mod->recordScalar("distance", getKadObserver().getPhysicalDistance(myOverlayKey, contact->getKey()));
		mod->recordScalar("commonPrefix", myOverlayKey.sharedPrefixLength(contact->getKey()));
	}
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
std::unique_ptr<T> R2Kademlia::makePacket(const KadContact* contact, const char *name) {
	auto packet = std::make_unique<T>();
	packet->setName(name);
	// TODO optimization: do this in one go
	packet->setSource(myOverlayKey);
	packet->appendRoute(contact);
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


void R2Kademlia::fillRTableRsp(KadRTableRsp& packet, const KadContactList& contacts, bool includePaths){
	ASSERT(packet.getContactArraySize() == 0);
	ASSERT(packet.getPathArraySize() == 0);
	ASSERT(contacts.size() > 0);
	packet.setContactArraySize(contacts.size());
	if (includePaths) {
		packet.setPathArraySize(contacts.size());
	}
	for (size_t i = 0; i < contacts.size(); i++) {
		packet.setContact(i, contacts[i]->getKey());
		if (includePaths) {
			packet.setPath(i, contacts[i]->getPath());
		}
	}
}

void R2Kademlia::readRTableRsp(std::unique_ptr<KadRTableRsp> packet) {
	ASSERT(packet->getContactArraySize() == packet->getPathArraySize());
	kad_path viaPath = packet->reversePath(packet->getIntermediatePath());
	viaPath.push_back(packet->getSource());
	for (size_t i = 0; i < packet->getContactArraySize(); i++) {
		// TODO optimization: isPathInteresting
		routingTable.addPath(
			packet->getContact(i),
			std::move(packet->getPath(i)),
			viaPath
		);
	}
}


void R2Kademlia::addTimer(simtime_t initialOffset, simtime_t duration, std::function<void()> callback) {
	ASSERT(timerDuration.size() == timerCallback.size());
	ASSERT(timerDuration.size() < std::numeric_limits<short>::max());
	ASSERT(duration > 0);
	timerDuration.push_back(duration);
	timerCallback.push_back(callback);
	cMessage *msg = new cMessage("TimerMsg", timerDuration.size()-1);
	timerMsg.push_back(msg);
	simtime_t scatter = uniform(0, duration);
	scheduleAt(simTime() + initialOffset - duration/2 + scatter, msg);
	EV_TRACE << "scheduled timer message " << msg << " with duration " << duration <<  endl;
}

void R2Kademlia::doProbe() {
	if (neighborTable.size() == 0) return; // we are isolated
	OverlayKey probeKey;
	bool exact = false;
	uint32_t index = 0;
	switch (probeStrategy) {
		case 0:
			return;
		case 1:
			probeKey = OverlayKey::random(getRNG(0));
			break;
		case 2:
			probeKey = getKadObserver().getRandomNodeKey();
			break;
		case 3:
			index = std::min<size_type>(intuniform(1, 1 + routingTable.getDepth()*2),
					myOverlayKey.getLength());
			probeKey = myOverlayKey;
			probeKey[myOverlayKey.getLength() - index] ^= 1;
			break;
		case 4:
			index = routingTable.getDepth();
			probeKey = myOverlayKey.randomSuffix(myOverlayKey.getLength() - index, getRNG(0));
			break;
		case 5:
			index = std::min<size_type>(intuniform(1, 1 + routingTable.getDepth()*2),
					myOverlayKey.getLength());
			probeKey = myOverlayKey.randomSuffix(myOverlayKey.getLength() - index, getRNG(0));
			break;
		case 6:
			index = intuniform(1, 1 + routingTable.getDepth());
			probeKey = myOverlayKey.randomSuffix(myOverlayKey.getLength() - index, getRNG(0));
			break;
		default:
			throw cRuntimeError("unknown probe strategy %i, values from 0 to 6 are allowed");
	}
	if (probeKey == myOverlayKey) probeKey[0] ^= 1;
	sendFindNode(probeKey, exact);
}

void R2Kademlia::doReachabilityTest() {
	if (neighborTable.size() == 0) return; // we are isolated
	const OverlayKey* key;
	do {
		key = &getKadObserver().getRandomNodeKey();
	} while(*key == myOverlayKey);
	sendFindNode(*key, true);
}

void R2Kademlia::countReceived(KadPacket& packet) {
	receivedPackets++;
	getKadObserver().registerPacketDelivered(packet);
	getStatistics().deliveredMessage(
		check_and_cast<cMessage *>(packet.getControlInfo()),
		packet.getProgress()
	);
}

void R2Kademlia::countDropped(KadPacket& packet, const OverlayKey& target) {
	droppedPackets++;
	getKadObserver().registerPacketDropped(packet, myOverlayKey, target);
	nodeid_t nid = undefined_node;
	if (getKadObserver().exists(target))
		nid = getKadObserver().getNodeId(target);
	getStatistics().dropMessage(
		check_and_cast<cMessage *>(packet.getControlInfo()), nid
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
