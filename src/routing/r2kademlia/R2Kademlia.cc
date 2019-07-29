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
	rediscoveryWaitTime = (simtime_t) (par("rediscoveryWaitTime").doubleValue() * 0.001);
	updateWaitTime = (simtime_t) (par("updateWaitTime").doubleValue() * 0.001);

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
	EV_TRACE << "onDisconnect " << nid << endl;
	// if this node is taken down, the packets arrive after the reset
	if(neighborTable.size() == 0) return;
	OverlayKey key = neighborTable.getOverlayKey(nid);
	ASSERT(!key.isUnspecified());
	ASSERT(neighborTable.getNodeId(key) == nid );

	neighborTable.removeNeighbor(nid);
	ASSERT(neighborTable.getNodeId(key) == undefined_node);
	// set contact invalid (was routingTable.removeContact(key))
	if (!routingTable.setContactInvalid(key)) {
		// this should only happen for nodes with very large degree
		EV_ERROR << "missing contact " << nid << " " << key << " on node " << node().getNodeId() << endl;
		return;
	}
	
	// Try to rediscover that contact
	// after grace period (otherwise more links may fail immediately after this)
	scheduleRediscovery(key, key, myOverlayKey);

	// contacts from routing table with this neighbor as next hop are not usable until rediscovered
	routingTable.invalidateContactsPN(key);

	// send UpdateRouteReq w/ failed link to ID-wise closest Nodes
	scheduleUpdate(key, KAD_Unreachable);
}


void R2Kademlia::onMessage( NodePkt_p pkt ) {
	ASSERT(pkt->getType() == PAYLOAD_TYPE);

	// we should check whether the link via this message travelled is
	// still active, otherwise we should drop this message as it was affected
	// by the link failure, e.g., more precisely using (getDuration(), getArrivalTime())
	const hop_t taken_hop= pkt->getHop();
	//ASSERT(Node::getNode(this)->getNodeId() == taken_hop.to);

	if (!getNetworkManager()->getTopology().isConnected(taken_hop.from,taken_hop.to))
	{
		KadPacket_p packet = std::unique_ptr<KadPacket>(check_and_cast<KadPacket*>(pkt->decapsulate()));
		EV_WARN << "received packet (msg id: " << packet->getId() << ") via an inactive link, thus will discard" << endl;
		if (!packet->getControlInfo())
			packet->setControlInfo(pkt.release());

		countDropped(*packet, packet->getTarget());
		return;
	}
		
	
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

			// avoid learning about supposed physical Neighbors
			kad_path path_of_interest(packet->reversePath(packet->getSrcPath(1, packet->getProgress())));
			if (path_of_interest.size() > 0)
				routingTable.addPath(packet->getSource(), path_of_interest);
			
			path_of_interest= packet->getSrcPath(packet->getProgress()+1, packet->getRouteArraySize()-1);
			if (path_of_interest.size() > 0)
				routingTable.addPath(packet->getTarget(), path_of_interest);
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
		if (handleIfFitting<KadUpdateRouteReq>(packet)) return;
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
	// reschedule timer if duration > 0, otherwise it was a one shot timer
	if (timerDuration[timerId] > 0)
	{
		simtime_t time = uniform(timerDuration[timerId]/2, timerDuration[timerId]/2*3);
		scheduleAt(simTime() + time, msg.release());
	}
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
	EV << "received hello packet from " << packet->getSource().shortStr() << endl;
	neighborTable.addNeighbor(packet->getSourceId(), packet->getSource());
	EV_TRACE << "neighbor table: "<< neighborTable << endl;
	routingTable.addPath(packet->getSource(), kad_path());
}


void R2Kademlia::handlePacket(KadAnswer_p packet) {
	EV_DEBUG << "received answer packet from " << packet->getSource().shortStr() << endl;
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
		// Trying to send to a next hop that is not a physical next hop
		// That will not fly...
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


/**
 * Assumption is that the contact list is sorted by XOR and distance to target  */
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
		// no proximity routing or only single contact
		//ASSERT(contacts.front()->getKey() != target); // this assertion seems to be wrong

		if (contacts.front()->getKey() == target)
		{
			EV_WARN << "Next hop is the target, number of contacts: " << contacts.size()
				 << "contact: " << *(contacts.front()) << endl;
		}

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
	EV_TRACE << "received KadQueryRouteRsp from " << packet->getSource().shortStr() << endl;
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


// UPDATEROUTEREQ
void R2Kademlia::handlePacket(std::unique_ptr<KadUpdateRouteReq> packet) {
	EV_TRACE << "received KadUpdateRouteReq from " << packet->getSource().shortStr() << endl;

	// check for malformed packet
	ASSERT(packet->getContactArraySize() == packet->getPathArraySize());
	ASSERT(packet->getContactArraySize() == packet->getActionArraySize());

	// path to origin of this KadUpdateRouteReq
	kad_path source_path= packet->reversePath(packet->getSrcPath());
	EV_TRACE << "path to origin: <";
	for (const OverlayKey& k : source_path)
		EV_TRACE << k.shortStr() << ' ';
	EV_TRACE << '>' << endl;

	// this is route update information only, no answer required
	unsigned int numContacts= packet->getContactArraySize();
	for (unsigned int i= 0; i < numContacts; i++)
	{
		// get contact and corresponding path info
		const auto& contactkey= packet->getContact(i);
		const auto& path= packet->getPath(i);
		auto action= packet->getAction(i);

		if (contactkey == myOverlayKey) {
			// someone is telling me news about myself -> ignore
			EV_TRACE << "ignoring update about me" << endl;
			continue;
		}
		
		EV_TRACE << "reported path to contact " << contactkey.shortStr() << " <";
		for (const auto& k : path)
			EV_TRACE << k.shortStr() << ' ';
		EV_TRACE << '>' << endl;

		switch (action) {
			case KAD_Announce:
				EV_TRACE << "UpdateReq Announce for contact " << contactkey.shortStr() << endl;
			        routingTable.addPath(contactkey, path, source_path);
				break;

			case KAD_Withdraw:
				EV_TRACE << "UpdateReq Withdraw for contact " << contactkey.shortStr() << endl;
				//TODO implement this
				break;

			case KAD_Change:
				EV_TRACE << "UpdateReq Change for contact " << contactkey.shortStr() << endl;
			        routingTable.addPath(contactkey, path, source_path);
				break;

			case KAD_Unreachable:
				// unreachable node
				ASSERT(path.size() == 0);

				EV_TRACE << "Unreachable contact: " << contactkey.shortStr() << ", link: < "
					 << packet->getSource().shortStr() << ',' << contactkey.shortStr() << " >" << endl;
				// now invalidate all contacts with this link
				routingTable.invalidateContacts(contactkey, packet->getSource());
				break;

			default:
				EV_ERROR << "unknown action type: " << action << endl;
				break;
		}
	}

}

	
bool R2Kademlia::handleQueryRouteResponse(KadPacket_p answer, RPCState& rpcState)
{
	if (rpcState.status != RPC_SUCCESS) return true;
	auto response = safe_packet_cast<KadQueryRouteRsp>(answer);
	EV_TRACE << "received " << response->getContactArraySize() << " neighbors from " << response->getSource().shortStr() << endl;
	readRTableRsp(std::move(response));
	return true;
}


// FIND NODE
void R2Kademlia::sendFindNode(const OverlayKey& key, bool exact, const OverlayKey* notvia_unreachable_node, const OverlayKey* notvia_failednode_neighbor) {
	EV_TRACE << "sendFindNode w/ " << key.shortStr() << (notvia_unreachable_node ? " (Rediscovery)" : "") << endl;
	auto contacts = routingTable.getClosestContacts(key, findNodeNumber);
	//ASSERT(contacts.size() > 0);
	if (contacts.size() > 0)
	{
		const KadContact& contact = chooseNextHop(contacts, key, myOverlayKey);
		ASSERT(contact.isValid());
		ASSERT(!contact.getKey().isUnspecified());
		if (contact.getKey() == myOverlayKey)
		{
			EV_ERROR << "I'm the next hop for the destination. Will not send anything." << endl;
			return;
		}
		auto packet = makePacket<KadFindNodeReq>(contact, "FindNode");
		packet->setQuery(key);
		packet->setExact(exact);
		if (notvia_unreachable_node &&  notvia_failednode_neighbor)
		{ // this is an immediate rediscovery packet
			ASSERT(exact);
			packet->setNot_via(true);
			packet->setUnreachableNode(*notvia_unreachable_node);
			packet->setDetectingNode(*notvia_failednode_neighbor);
			sentRediscovery++;
			EV_TRACE << "Rediscovery sending ... msg id=" << packet->getId()
				 << " via " << contact << endl;
		}
		sendRpcCall(
			std::move(packet),
			std::bind(&R2Kademlia::handleFindNodeResponse, this, ph::_1, ph::_2)
			);
	}
	else
	{ // no contacts to choose from (usually only possible for isolated nodes)
		EV_WARN << "no contacts found, cannot send FindNodeReq" << endl;
	}
}


void R2Kademlia::handlePacket(std::unique_ptr<KadFindNodeReq> packet) {
	OverlayKey& query = packet->getQuery(); // query is the destination ID

	EV_TRACE << "KadFindNodeReq received query for: " << query.shortStr() << endl;
	bool is_rediscovery= false;
	// check for presence of "not via" setting
	if (packet->getNot_via())
	{
		EV_TRACE << "FindNodeReq contains not via <"
			 << packet->getUnreachableNode().shortStr() << ' '
			 << packet->getDetectingNode().shortStr() << '>' << endl;
		// we should invalidate all contacts with that link in the path to them before answering
		routingTable.invalidateContacts(packet->getUnreachableNode(), packet->getDetectingNode());
		is_rediscovery= true;
	}
	
	size_t contactsLen = std::max((size_t)findNodeNumber, routingTable.getSplitSize());
	KadContactList contacts = routingTable.getClosestContacts(query, contactsLen);

	if (contacts.size() == 0)
	{
		// send back Route Failure: no route toward destination known
		// the only contact we know is the source
		countDropped(*packet, query);
		returnToSender(KAD_RouteFailureDeadEnd, std::move(packet));

		if (is_rediscovery)
			droppedRediscovery++;

		return;
	}

	// routing table has at least one entry
	if (query != myOverlayKey) { // someone else routing via us

		// the first contact is closest to query key
		// if it is the source, we would route it back in a loop
		if (contacts.front()->getKey() == packet->getSource()) {
			if (contacts.size() == 1) {
				// the only contact we know is the source
				countDropped(*packet, query);
				returnToSender(KAD_RouteFailureDeadEnd, std::move(packet));

				if (is_rediscovery)
					droppedRediscovery++;

				return;
			}
			else // remove source from answer
				contacts.erase(contacts.begin());
		}

		/*
		EV_TRACE << "Found contacts:";
		for (const auto& thiscontact : contacts)
		{
			EV_TRACE << *thiscontact;
		}
		EV_TRACE << endl;
		*/
		
		// if contact is closer to destination than we, it's ok
		if ((contacts.front()->getKey() ^ query) < (myOverlayKey ^ query)
				and contacts.front()->getPathLength() < packet->getHopLimit()) {
			// we have found a closer contact that is close enough
			// => forward request
			const KadContact& contact = chooseNextHop(contacts, query, packet->getSource());
			EV_TRACE << "KadFindNodeReq received query: " << packet->getQuery().shortStr() << endl << " next hop is " << contact << endl;
			// note: the log_2 will be smaller for a longer common prefix
			ASSERT((contact.getKey() ^ query).log_2() <= (myOverlayKey ^ query).log_2());
			if((contact.getKey() ^ query).log_2() > (myOverlayKey ^ query).log_2()) {
				EV_WARN << "no prefix progress made" << endl;
			}
			ASSERT((contact.getKey() ^ query) < (myOverlayKey ^ query));
			packet->appendRoute(contact);
			ASSERT(packet->getRouteArraySize() - packet->getProgress() <= packet->getHopLimit());
			resendPacket(std::move(packet));
			if (is_rediscovery)
				forwardedRediscovery++;
			return;
		}
		else
		{
			// the best contact we know is further away from the target than we are
			// we must find the node exactly, however, if there is no progress from here
			// we should send back an error message
			if (packet->getExact()) {

				const KadContact& contact = chooseNextHop(contacts, query, packet->getSource());
				EV_WARN << "best neighbor now: " << contact << endl;
				EV_TRACE << "distance for contact=" << (contact.getKey() ^ query).log_2()
					 << " own distance=" << (myOverlayKey ^ query).log_2() << endl;
				if ((contact.getKey() ^ query).log_2() > (myOverlayKey ^ query).log_2()) {
					EV_WARN << "no prefix progress made - dropping request" << endl;
				}
				// since we need an exact match and cannot make any progress, we are lost here
				countDropped(*packet, query);
				//TODO: if we find a closer contact too far away, we should send HOP LIMIT EXCEEDED
				returnToSender(KAD_RouteFailureDeadEnd, std::move(packet));

				if (is_rediscovery)
					droppedRediscovery++;
				return;

			} // if exact flag is set

			// otherwise it's ok to report what we know

		} // end else
	}
	// answer request
	auto answer = makeAnswer<KadFindNodeRsp>(*packet);
	fillRTableRsp(*answer, contacts);
	countReceived(*packet);
	if (is_rediscovery)
		succeededRediscovery++;
	
	EV_TRACE << "KadFindNodeReq query for " << query.shortStr() << " answering with list size " << contacts.size() << endl;
	sendPacket(std::move(answer));
}


bool R2Kademlia::handleFindNodeResponse(KadPacket_p answer_, RPCState& rpcState) {
	if (rpcState.status != RPC_SUCCESS) return true;
	auto answer = safe_packet_cast<KadFindNodeRsp>(answer_);
	ASSERT(answer->getContactArraySize() == answer->getPathArraySize());
	EV_TRACE << "received FindNodeRsp for " << check_and_cast<KadFindNodeReq *>(rpcState.original_packet.get())->getQuery().shortStr();
	EV_TRACE << " from " << answer->getSource().shortStr() << " with " << answer->getContactArraySize() << " contacts"<< endl;
	readRTableRsp(std::move(answer));
	return true;
}
// ------------------------- policy foo --------------------------------------

void R2Kademlia::onMissingNeighbor(KadPacket_p packet, NodePkt_p pkt) {
//  	getStatistics().dropMessage(pkt, undefined_node);
	EV_WARN << "onMissingNeighbor called - packet next hop should be " << packet->getNextHop().shortStr() << endl;
	droppedPackets++;
	// do not generate Error packets for Error packets
	if (packet->getType() == KAD_Error) {
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
		EV_INFO << "Removing contact " << original->getTarget().shortStr() << " because of error packet" << endl;
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


void R2Kademlia::onImprovement(const KadContact& contact) {
	// path got improved for this contact -> potentially send UpdateRouteReq
	EV_TRACE << "onImprovement(" << contact.getKey().shortStr() << ") called" << endl;

	// TODO: wait grace period
	if ( neighborTable.isNeighbor(contact.getKey()) )
	{ // this is true after re-enabling a physical neighbor
		scheduleUpdate(contact.getKey(), KAD_Announce);
	}
	else
		scheduleUpdate(contact.getKey(), KAD_Change);
}


void R2Kademlia::recordScalars() {
	recordScalar("forwardedPackets", forwardedPackets);
	recordScalar("droppedPackets", droppedPackets);
	recordScalar("sentPackets", sentPackets);
	recordScalar("receivedPackets", receivedPackets);
	recordScalar("rediscoverySent", sentRediscovery);
	recordScalar("rediscoverySuccess", succeededRediscovery);
	recordScalar("rediscoveryFailed", droppedRediscovery);
	recordScalar("rediscoveryForwarded", forwardedRediscovery);
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
	hello->setHopLimit(hopLimit);
	hello->setSourceId(node().getNodeId());
	hello->setNonce(0);
	NodePkt_p nodePkt = std::make_unique<NodePkt>(hello->getName());
	nodePkt->setType(PAYLOAD_TYPE);
	nodePkt->encapsulate(hello.release());
	return nodePkt;
}


/// send all updates
void R2Kademlia::sendUpdates()
{
	// the queue (map) will be traversed
	for (auto notifyContact : updateQueue)
	{
		auto packet = makePacket<KadUpdateRouteReq>(notifyContact.first, "UpdateRoute");

		for (auto& contact : notifyContact.second)
		{
		}
		/*
		packet->setContactArraySize(1);
		packet->setContact(0, affectedContactKey);
		packet->setPathArraySize(1);
		
		packet->setPath(0, affectedContact->getPath());
		packet->setActionArraySize(1);
		packet->setAction(0, action);
		// send UpdateRouteReq w/ failed link
		sendPacket(std::move(packet));
		*/
	}


}



// send a find node request excluding paths that include the failed link
void R2Kademlia::scheduleRediscovery(const OverlayKey& contactkey, const OverlayKey& notvia_unreachable_node, const OverlayKey& notvia_failednode_neighbor)
{
	KadContact_p rt_contact= routingTable.getContact(contactkey);
	if (rt_contact && rt_contact->isRediscovering())
	{
		// do not schedule for rediscovery if there is something still in progress
		EV_WARN << "Rediscovery not scheduled for contact " << contactkey.shortStr()
			<< " because it is already in rediscovering state" << endl;
		return;
	}
	
	EV_TRACE << "Rediscovery scheduled for contact " << contactkey.shortStr() << " <"
		<< notvia_unreachable_node.shortStr() << ' '
		<< notvia_failednode_neighbor.shortStr() << '>' << endl;

	auto rediscoverContact=
		std::make_unique<const KadContact>(contactkey, vector<OverlayKey>{notvia_unreachable_node, notvia_failednode_neighbor});
	
	rediscoverQueue.push_back(std::move(rediscoverContact));

	if (!rediscoveryScheduled)
	{
		addTimer(rediscoveryWaitTime, 0, std::bind(&R2Kademlia::rediscoverContacts, this));
		rediscoveryScheduled= true;
	}
	
}


void R2Kademlia::scheduleUpdate(const OverlayKey& affectedContactKey, KadRouteUpdateAction action)
{
	// we collect all update information for neighbors
	// send UpdateRouteReq to ID-wise closest Nodes
	size_t contactsLen = std::max((size_t)findNodeNumber, routingTable.getSplitSize());
	KadContactList closeContacts(routingTable.getClosestContacts(affectedContactKey, contactsLen));
	
	KadContact_p affectedContact= routingTable.getContact(affectedContactKey);
	ASSERT(affectedContact);
	
	if (closeContacts.size() > 0)
	{ // if there are contacts left to notify
		for (auto notifyContact : closeContacts)
		{ // each ID-wise close contact gets a UpdateRouteReq from us

			// except when it is the contact itself
			if (affectedContactKey == notifyContact->getKey())
				continue;

			auto entry= updateQueue.find(affectedContactKey);
			if ( entry == updateQueue.end() ) {
				// need to create new entry
				updateQueue[notifyContact->getKey()]= { std::make_pair(affectedContactKey, action) };
			}
			else
			{
				// entry exists, so we need to append this update to the list
				entry->second.emplace_back(std::make_pair(affectedContactKey, action));
			}
		} // end for each contact
			
	} // endif
	
	if (!updateScheduled)
	{
		addTimer(updateWaitTime, 0, std::bind(&R2Kademlia::sendUpdates, this));
		updateScheduled= true;
	}

}

template<typename T>
std::unique_ptr<T> R2Kademlia::makePacket(const KadContact* contact, const char *name) {
	auto packet = std::make_unique<T>();
	packet->setName(name);
	// TODO optimization: do this in one go
	packet->setSource(myOverlayKey);
	packet->setHopLimit(hopLimit);
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

/* if duration is 0, this is a one shot timer */
void R2Kademlia::addTimer(simtime_t initialOffset, simtime_t duration, std::function<void()> callback) {
	ASSERT(timerDuration.size() == timerCallback.size());
	ASSERT(timerDuration.size() < std::numeric_limits<short>::max());
	ASSERT(duration >= 0);
	timerDuration.push_back(duration);
	timerCallback.push_back(callback);
	cMessage *msg = new cMessage("TimerMsg", timerDuration.size()-1);
	timerMsg.push_back(msg);
	simtime_t scatter = uniform(0, duration);
	scheduleAt(simTime() + initialOffset - duration/2 + scatter, msg);
	EV_TRACE << "scheduled timer message " << msg << ( (duration != 0) ? " with duration " : " after ")
		 <<   ((duration != 0) ? duration : initialOffset) <<  endl;
}


void R2Kademlia::rediscoverContacts() {
	// traverse list of scheduled contacts for rediscovery
	// recheck status and set to rediscover
	EV_TRACE << "rediscoverContacts called, Queued: " << rediscoverQueue.size() << endl;
	
	while (!rediscoverQueue.empty())
	{
		auto& contact= rediscoverQueue.front();
		// check for current status of scheduled contact
		auto rt_contact= routingTable.getContact(contact->getKey());
		if (rt_contact) { // so contact still exists
			// contact may have been recovered meanwhile
			// do not rediscover contact which is already Rediscovering
			if (rt_contact->isInvalid() && !rt_contact->isRediscovering())
			{
				// set contact state to rediscovering
				const_cast<KadContact*>(rt_contact)->setRediscovering();
			        const kad_path& failedLink(contact->getPath());
				ASSERT(failedLink.size() == 2);
				sendFindNode(rt_contact->getKey(), true, &failedLink[0], &failedLink[1]);
			}
		}
		rediscoverQueue.pop_front();
	}

	// next call of this procedure will be scheduled on demand
	rediscoveryScheduled= false;
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
	
	EV_TRACE << "returnToSender() - return path:";
	for (const auto& c : answer->getSrcPath())
		EV_TRACE << ' ' << c.shortStr();
	EV_TRACE << endl;		
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
