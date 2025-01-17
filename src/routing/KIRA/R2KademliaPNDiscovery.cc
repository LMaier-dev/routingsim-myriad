#include "KIRA.h"

/*********************************************************
 * This file contains code that is related to the discovery
 * of the physical neighbors and vicinity from the 
 * R2Kademlia main class
 *********************************************************/

namespace routingsim {


NodePkt_p R2Kademlia::makeHelloPacket()
{
	KadHello_p hello = std::make_unique<KadHello>();
	hello->setName(PDUNames.at(KAD_ULNHello));
	hello->setSource(myOverlayKey);
	hello->setHopLimit(hopLimit);
	hello->setSourceId(node().getNodeId());
	hello->setSrcNodeDegree(neighborTable.size());
	hello->setNonce(0);
	hello->setStateSeqNo(getLSSeqNo());
	NodePkt_p nodePkt = std::make_unique<NodePkt>(hello->getName());
	nodePkt->setType(KAD_CTRL_PACKET_TYPE);
	nodePkt->encapsulate(hello.release());

	return nodePkt;
}


// everything starts here: discovering a neighbor by seeing a PNHello packet
// the node is added to the neighborTable, but not added as contact until
// bidirectional reachability is confirmed
// in case the link was temporarily down: rediscover neighbor
void R2Kademlia::handlePacket(KadHello_p packet) {
	EV << "received hello packet from " << packet->getSource().shortStr() << " SSeq#" << packet->getStateSeqNo() << endl;

	countReceived(*packet);

	KadContact neighborcontact(myOverlayKey, packet->getSource(), kad_path());

	// it could be that the node has been entered into the vicinity already
	if (neighborTable.isNeighbor(packet->getSource()))
	{ // known physical neighbor: check for newer sequence number

		if (auto nodep= vicinity.getNode(packet->getSource()); nodep != nullptr) {
			if (nodep->stateseqno < packet->getStateSeqNo()) {
				EV_TRACE << "check for resync" << endl;
				// already known neighbor, just check for seqno synchronization (also updates lastseen)
				checkForResync(packet->getSource(), packet->getStateSeqNo());
                        }
			else {
				// update lastseen nevertheless
				if (KadContact *contact_p = const_cast<KadContact *>(routingTable.getContact(packet->getSource()));
				    contact_p != nullptr and contact_p->isPhysicalNeighbor()) {
					contact_p->updateLastSeen();
					contact_p->updateLastDirectULContact();
				}
			}
                } else {
			// update lastseen
			if (KadContact *contact_p = const_cast<KadContact *>(routingTable.getContact(packet->getSource()));
			    contact_p != nullptr and contact_p->isPhysicalNeighbor()) {
				contact_p->updateLastSeen();
				contact_p->updateLastDirectULContact();
			}
		}
		return;
	}
	else
	{ // new underlay/physical neighbor, first contact: either wait for ULNDiscReq or send one

                // connected or reconnected to first neighbor reset join timer
                if (neighborTable.empty()) {
                    resetTimer(probeTimerId, SimTime(ProbeSelfMinInterval,SIMTIME_MS));
                }

		// we need to add the node to our neighbortable, because we need to send a Req
		neighborTable.addNeighbor(packet->getSource(), packet->getSourceId());

		// add node to vicinity
		auto [node_added, link_added, nodeptr]= vicinity.addNode(myOverlayKey, // parent
						  packet->getSource(), // new child
						  1);

		if (usePathIDs and (node_added or link_added)) {
			fwdTablePathID.addPrecalcEntry({neighborcontact.getKey()});
			vicinityPathIDCompletion(neighborcontact);
		}

		// use this deterministic strategy to save redundant ULNDiscReq/Rsp exchanges
		// do not use full ID, otherwise large IDs will always "loose", use mod 2^32 comparison
		// small collision chance: but just in case, full nodeID will be a tie breaker
		uint32_t ownModID(myOverlayKey.getBitRange(0,32));
		uint32_t otherModID(packet->getSource().getBitRange(0,32));
		uint32_t delta((uint32_t) (otherModID-ownModID));

		if ( (delta < 0x80000000UL and delta != 0 ) or
		     ((delta == 0 or delta == 0x80000000UL) and myOverlayKey < packet->getSource())
			) {
			// schedule ULNDiscReq
			schedulePNDiscReq({myOverlayKey, packet->getSource(), {}, stateseqno_t::invalidSeqNo});
		}
	}

	//EV_TRACE << "handlePacket: "<< static_cast<void*>(packet.get()) << endl;
}


// Request Physical Neighbors from contact
// @returns a delay value for the next call of this method
simtime_t R2Kademlia::sendGetPhysicalNeighbors(const KadContact& contact, RequestQueueElementActionType action)
{
	EV_TRACE << "sending query route request (PN" << ((action & RequestQueueElementActions::get_OverlayNeighborsSource) ? "+ON)" : ")") << " to " << contact << endl;
	auto req = makePacket<KadQueryRouteReq>(contact, "QueryRouteReq");
	req->setNeighborhood(1); // get only physical neighbors (1 hop away)
	req->setContactsOnly(false);
	req->setOverlayNeighbors(false);
	// if true, return also the Overlay Neighbors
	req->setOverlayNeighborsSource( (action & RequestQueueElementActions::get_OverlayNeighborsSource) != 0 );

	simtime_t delay(0);

	static const simtime_t mindelay(SimTime(GetPNWaitTime,SIMTIME_US)); // was 20000
	if  ( simTime() < LastPNQueryRouteReq ) {
		// last was scheduled in future, schedule it mindelay behind that
		delay= LastPNQueryRouteReq-simTime()+mindelay;
	}
	else { // last send in past
		if (simTime()-LastPNQueryRouteReq < mindelay) {
			// we are too early, so delay until mindelay is fulfilled
			delay= LastPNQueryRouteReq+mindelay-simTime();
		}
	}

	sendRpcCall(
		std::move(req),
		std::bind(&R2Kademlia::handleQueryRouteResponse, this, ph::_1, ph::_2),
		std::bind(&R2Kademlia::handleQueryRouteResponse, this, ph::_1, ph::_2),
		std::bind(&R2Kademlia::handleRPCTimeOut, this, ph::_1, ph::_2),
		0
	);
	LastPNQueryRouteReq= simTime()+delay;

	// update vicinity to avoid future redundant requests
	auto [nodeAdded, linkAdded, nodeptr]= vicinity.addNode(contact.getPath().empty() ? myOverlayKey : contact.getPath().back(), // parent
					       contact.getKey(), // new child
					       contact.getPathLength()+1);
	if (usePathIDs and (nodeAdded or linkAdded)) {
		auto newpath(contact.getPath());
		newpath.push_back(contact.getKey());
		fwdTablePathID.addPrecalcEntry(newpath);
		vicinityPathIDCompletion(contact);
	}
	// return delay for next call
	return delay;
}



// Request k nearest Overlay Neighbors from contact
// the 2nd parameter is currently ignored (only required by unified interface)
// @returns a delay value for the next call of this method
simtime_t R2Kademlia::sendGetOverlayNeighbors(const KadContact& contact, RequestQueueElementActionType action)
{
	EV_TRACE << "sending query route request (ON) to " << contact << endl;
	auto req = makePacket<KadQueryRouteReq>(contact, "QueryRouteReq");
	req->setNeighborhood(0); // no physical neighbors
	req->setOverlayNeighborsSource( (action & RequestQueueElementActions::get_OverlayNeighborsSource) != 0 ); // get known overlay neighbors for my ID
	req->setContactsOnly(false); // get also the routes

	// we rate limit the requests
	simtime_t delay(0);
	static const simtime_t mindelay(SimTime(GetONWaitTime,SIMTIME_US)); // was 40000
	if  ( simTime() < LastONQueryRouteReq ) {
		// last was scheduled in future, schedule it mindelay behind that
		delay= LastONQueryRouteReq-simTime()+mindelay;
	}
	else { // last send in past
		if (simTime()-LastONQueryRouteReq < mindelay) {
			// we are too early, so delay until mindelay is fulfilled
			delay= LastONQueryRouteReq+mindelay-simTime();
		}
		// otherwise no extra delay necessary
	}

	sendRpcCall(
		std::move(req),
		std::bind(&R2Kademlia::handleQueryRouteResponse, this, ph::_1, ph::_2),
		std::bind(&R2Kademlia::handleQueryRouteResponse, this, ph::_1, ph::_2),
		std::bind(&R2Kademlia::handleRPCTimeOut, this, ph::_1, ph::_2),
		0
	);

	LastONQueryRouteReq= simTime()+delay;

	return delay;
}


// Send ULNDiscReq
// the 2nd parameter is currently ignored (only required by unified interface)
// @returns a delay value for the next call of this method
simtime_t R2Kademlia::sendDiscoveryReq(const KadContact& contact, RequestQueueElementActionType _unused_arg)
{
	EV_TRACE << "sending ULNDiscReq to " << contact << " mySSeq#: " << getLSSeqNo();

	// need to brute force the path as the contact may be known via a detour before
	KadContact pncontact(myOverlayKey, contact.getKey(),kad_path());
	auto req = makePacket<KadULNDiscReq>(&pncontact);
	req->setSourceId(node().getNodeId());
	req->setNeighborhood(1); // get only physical neighbors (1 hop away)
	req->setContactsOnly(false);
	// send my own physical neighbors
	if (auto neighbors = neighborTable.getTable(); neighbors.size() > 0) {

		// reserve enough space in the vectors
		req->setPhyNeighborArraySize(neighbors.size());
		req->setPhyNeighborSeqNoArraySize(neighbors.size());
		req->setPhyNeighborAgeArraySize(neighbors.size());
		req->setPhyNeighborDegreeArraySize(neighbors.size());
		uint effective_size(0);
		for (auto it= neighbors.cbegin(); it != neighbors.cend(); it++) {
			if (auto contactp= routingTable.getContact(it->first); contactp != nullptr and contactp->getPathLength()==0) {
				req->setPhyNeighbor(effective_size, it->first);
				req->setPhyNeighborSeqNo(effective_size, contactp->getStateSeqNo());
				req->setPhyNeighborAge(effective_size, contactp->getPathAge());
				req->setPhyNeighborDegree(effective_size, contactp->getNodeDegree());
				effective_size++;
			}
		}
		// shrink size to what has been actually written
		req->setPhyNeighborArraySize(effective_size);
		req->setPhyNeighborSeqNoArraySize(effective_size);
		req->setPhyNeighborAgeArraySize(effective_size);
		req->setPhyNeighborDegreeArraySize(effective_size);
		req->addBitLength(effective_size*(OverlayKey::getLength()+16+32+16));
	}
	//req->setHopLimit(1);
	// we rate limit the requests
	simtime_t delay(0);

	// default (see R2Kademlia.h): 10000µs is 10ms, so the rate limit is 100 ULNDiscReqs/s
	static const simtime_t mindelay(SimTime(PNDiscoverWaitTime,SIMTIME_US));
	if  ( simTime() < LastPNDiscReq ) {
		// last was scheduled in future, schedule it mindelay behind that
		delay= LastPNDiscReq-simTime()+mindelay;
	}
	else { // last send in past
		if (simTime()-LastPNDiscReq < mindelay) {
			// we are too early, so delay until mindelay is fulfilled
			delay= LastPNDiscReq+mindelay-simTime();
		}
		// otherwise no extra delay necessary
	}

	EV_TRACE << " delay: " << delay << endl;
	sendRpcCall(
		std::move(req),
		std::bind(&R2Kademlia::handlePNDiscResponse, this, ph::_1, ph::_2),
		std::bind(&R2Kademlia::handlePNDiscResponse, this, ph::_1, ph::_2),
		std::bind(&R2Kademlia::handleRPCTimeOut, this, ph::_1, ph::_2),
		0
	);

	LastPNDiscReq= simTime()+delay;

	return delay;
}


// handle ULNDiscReq
void R2Kademlia::handlePacket(std::unique_ptr<KadULNDiscReq> packet) {
	EV_TRACE << "received ULNDiscReq from " << packet->getSource().shortStr()
		 << " SSeq#" <<  packet->getStateSeqNo()
		 << " w/ " << packet->getPhyNeighborArraySize() << " neighbors" << endl;

	countReceived(*packet);

	// since this is a physical neighbor try to add it to our neighbortable
	neighborTable.addNeighbor(packet->getSource(), packet->getSourceId());

	// add physical neighbor to my vicinity
	auto [node_added, link_added, nodeptr]= vicinity.addNode(myOverlayKey, packet->getSource(), 1);
	if (nodeptr != nullptr) {
		// set or update the sequence number
		nodeptr->stateseqno= packet->getStateSeqNo();
	}
	KadContact pncontact(myOverlayKey, packet->getSource(), kad_path());
	if (usePathIDs and (node_added or link_added)) {
		fwdTablePathID.addPrecalcEntry({pncontact.getKey()});
		vicinityPathIDCompletion(pncontact);
	}

	// add physical contact to routing table or update sequence number, last seen, and LastDirectULContact
	routingTable.addPNContact(packet->getSource(), packet->getStateSeqNo(), packet->getSrcNodeDegree());

	// check for physical neighbors sent with
	kad_path path{packet->getSource()};
	if (packet->getPhyNeighborArraySize() > 0) {
		for (uint i = 0; i < packet->getPhyNeighborArraySize(); i++) {

			// this addPath cannot be trusted, because the PN's neighbor may have not been contacted recently
			if (not routingTable.addPath(packet->getPhyNeighbor(i), path,
						     packet->getPhyNeighborSeqNo(i),
						     packet->getPhyNeighborAge(i),
						     packet->getPhyNeighborDegree(i),
						     false)) {
				// if not put into routing table, add PN's neighbor to vicinity nevertheless
				auto [node_added, link_added, nodeptr]= vicinity.addNode(packet->getSource(),
								  packet->getPhyNeighbor(i),
								  2);
				if (usePathIDs and (node_added or link_added)) {
					KadContact nncontact(myOverlayKey, packet->getPhyNeighbor(i), { {packet->getSource()} });
					auto newpath(path);
					newpath.push_back(nncontact.getKey());
					fwdTablePathID.addPrecalcEntry(newpath);
					vicinityPathIDCompletion(nncontact);
				}
			}
		}
	}

	EV_TRACE << "neighbor table: "<< neighborTable << endl;

	// Update syncPNSeqNo if suitable
	if (KadContact* contact_p= const_cast<KadContact*>(routingTable.getContact(packet->getSource())); contact_p != nullptr) {

		// if we got a list of PNs, set syncPNSeqNo to avoid too frequent resync
		if (packet->getPhyNeighborArraySize() > 0) {
			if (packet->getStateSeqNo() > contact_p->getSyncPNSeqNo())
				contact_p->setSyncPNSeqNo(packet->getStateSeqNo());
		}
		else {
			// if no PN list given we may want to resync
			checkForResync(packet->getSource(), packet->getStateSeqNo());
		}
	}

	// TODO currently only getting physical neighbors is implemented
	ASSERT(packet->getNeighborhood() == 1);
	ASSERT(not packet->getContactsOnly());
	auto response = makeAnswer<KadULNDiscRsp>(*packet);
	auto neighbors = neighborTable.getTable();

	response->setContactArraySize(neighbors.size());
	response->setPathArraySize(neighbors.size());
	response->setSeqNoArraySize(neighbors.size());
	response->setPathInfoAgeArraySize(neighbors.size());
	response->setNodeDegreeArraySize(neighbors.size());
	uint effective_size(0);
	for (auto it= neighbors.cbegin(); it != neighbors.cend(); it++) {
		// some neighbors may not yet be in the routing table (or only indirectly via other nodes)
		if (auto contactp= routingTable.getContact(it->first); contactp != nullptr and contactp->getPathLength()==0) {
			response->setContact(effective_size, it->first);
			response->setSeqNo(effective_size, contactp->getStateSeqNo());
			response->setPathInfoAge(effective_size, contactp->getPathAge());
			response->setNodeDegree(effective_size, contactp->getNodeDegree());

			effective_size++;
		}
	}
	response->setContactArraySize(effective_size);
	response->setPathArraySize(effective_size);
	response->setSeqNoArraySize(effective_size);
	response->setPathInfoAgeArraySize(effective_size);
	response->setNodeDegreeArraySize(effective_size);
	response->addBitLength(effective_size*(OverlayKey::getLength()+16+32+16));

	sendPacket(std::move(response));
}



void R2Kademlia::handlePacket(std::unique_ptr<KadQueryRouteReq> packet)
{
	EV_TRACE << "received KadQueryRouteReq from " << packet->getSource().shortStr()
		 << " SSeq#" <<  packet->getStateSeqNo() << endl;

	// update StateSeqNo and lastseen of origin contact
	if (KadContact* contact_p= const_cast<KadContact*>(routingTable.getContact(packet->getSource())); contact_p != nullptr) {
		if (contact_p->getStateSeqNo() <= packet->getStateSeqNo()) {
			contact_p->setStateSeqNo(packet->getStateSeqNo());
			contact_p->updateLastSeen();
			contact_p->setNodeDegree(packet->getSrcNodeDegree());
		}
		// EV_TRACE << "contact found" << endl;
	}

	//  packet->getContactsOnly() not implemented yet
	ASSERT(not packet->getContactsOnly());
	auto response = makeAnswer<KadQueryRouteRsp>(*packet);

	// need to check for the number of overlay neighbors that need to be returned
	KadContactList ovlyneighbors;
	if (packet->getOverlayNeighbors() or packet->getOverlayNeighborsSource())
	{
		ovlyneighbors= routingTable.getClosestContacts(packet->getOverlayNeighbors() ? myOverlayKey : packet->getSource(), routingTable.getSplitSize(), true);
		routingTable.addRandomGratuitousContacts(ovlyneighbors);
	}

	uint effective_size= 0;
	// distance of 1 means physical neighbors
	if (packet->getNeighborhood() == 1)
	{
		auto neighbors= neighborTable.getTable();
		response->setContactArraySize(neighbors.size() + ovlyneighbors.size());
		response->setPathArraySize(neighbors.size() + ovlyneighbors.size());
		response->setSeqNoArraySize(neighbors.size() + ovlyneighbors.size());
		response->setPathInfoAgeArraySize(neighbors.size() + ovlyneighbors.size());
		response->setNodeDegreeArraySize(neighbors.size() + ovlyneighbors.size());
		for (auto it= neighbors.cbegin(); it != neighbors.cend(); it++) {
			if (auto contactp= routingTable.getContact(it->first); contactp != nullptr and contactp->getPathLength()==0)
			{
				response->setContact(effective_size, it->first);
				response->setSeqNo(effective_size, contactp->getStateSeqNo());
				response->setPathInfoAge(effective_size, contactp->getPathAge());
				response->setNodeDegree(effective_size, contactp->getNodeDegree());
				effective_size++;
			}
		}
		response->setPNListDelivered(true);
	}
	else
	if (packet->getNeighborhood() == 63) // distance 63 means to get full routing table
	{
		KadContactList allcontacts= routingTable.getAllContacts();
		response->setContactArraySize(allcontacts.size() + ovlyneighbors.size());
		response->setPathArraySize(allcontacts.size() + ovlyneighbors.size());
		response->setSeqNoArraySize(allcontacts.size() + ovlyneighbors.size());
		response->setPathInfoAgeArraySize(allcontacts.size() + ovlyneighbors.size());
		response->setNodeDegreeArraySize(allcontacts.size() + ovlyneighbors.size());
		for (uint i = 0; i < allcontacts.size(); i++)
		{
			response->setContact(effective_size, allcontacts[i]->getKey());
			response->setPath(effective_size, allcontacts[i]->getPath());
			response->setSeqNo(effective_size, allcontacts[i]->getStateSeqNo());
			response->setPathInfoAge(effective_size, allcontacts[i]->getPathAge());
			response->setNodeDegree(effective_size, allcontacts[i]->getNodeDegree());
			effective_size++;
		}
		response->setPNListDelivered(true);
	}

	if (packet->getOverlayNeighbors() or packet->getOverlayNeighborsSource())
	{
		// if the previous methods did not allocate the array, do it now
		if (effective_size == 0) {
			response->setContactArraySize(ovlyneighbors.size());
			response->setPathArraySize(ovlyneighbors.size());
			response->setSeqNoArraySize(ovlyneighbors.size());
			response->setPathInfoAgeArraySize(ovlyneighbors.size());
			response->setNodeDegreeArraySize(ovlyneighbors.size());
		}

		for (uint i = 0; i < ovlyneighbors.size(); i++)
		{
			response->setContact(effective_size, ovlyneighbors[i]->getKey());
			response->setPath(effective_size, ovlyneighbors[i]->getPath());
			response->setSeqNo(effective_size, ovlyneighbors[i]->getStateSeqNo());
			response->setPathInfoAge(effective_size, ovlyneighbors[i]->getPathAge());
			response->setNodeDegree(effective_size, ovlyneighbors[i]->getNodeDegree());
			effective_size++;
		}
	}

	response->setContactArraySize(effective_size);
	response->setPathArraySize(effective_size);
	response->setSeqNoArraySize(effective_size);
	response->setPathInfoAgeArraySize(effective_size);
	response->setNodeDegreeArraySize(effective_size);

	sendPacket(std::move(response));
}


bool R2Kademlia::handlePNDiscResponse(KadPacket_p answer, RPCState& rpcState)
{
	if (rpcState.status != RPC_SUCCESS) return true;
	auto response = safe_packet_cast<KadULNDiscRsp>(answer);
	EV_TRACE << response->getName() << " from " << response->getSource().shortStr() << " received " << response->getContactArraySize()
		 << " neighbors SSeq#" << response->getStateSeqNo() << endl;

	if (response->getRouteArraySize() != 2)
	  EV_ERROR << response->getName() << " not received via direct link" << endl;

	// update sequence number or restore direct link
	vicinity.updateNodeStateSeqNo(response->getSource(), response->getStateSeqNo());
	routingTable.addPNContact(response->getSource(), response->getStateSeqNo(), response->getSrcNodeDegree());

	// update PNsyncSeqNo
	if (KadContact* contact_p= const_cast<KadContact*>(routingTable.getContact(response->getSource())); contact_p != nullptr) {
		contact_p->setSyncPNSeqNo(response->getStateSeqNo());
	}
	else
		EV_ERROR << "PN not found as contact in RT!" << endl;

	// now read PNs from neighbor
	readRTableRsp(*response);
	return true;
}


bool R2Kademlia::handleQueryRouteResponse(KadPacket_p answer, RPCState& rpcState)
{
	ASSERT(rpcState.original_packet.get() != nullptr);
	if (rpcState.status != RPC_SUCCESS) {
		return true;
	}
	auto response = safe_packet_cast<KadQueryRouteRsp>(answer);
	EV_TRACE << "received " << response->getContactArraySize() << " neighbors from " << response->getSource().shortStr() << endl;
	if (response->getPNListDelivered()) {
		// update state seq no in vicinity
		vicinity.updateNodeStateSeqNo(response->getSource(), response->getStateSeqNo());
	}

	readRTableRsp(*response);
	return true;
}


/// schedule sending ULNDiscReq for contacts, because meanwhile more neighbors will be discovered
/// also record expected new seqno, so if state had been updated meanwhile we can avoid resync
/// this will avoid triggering frequent resynchronizations
/// note that the given contact is a temporary contact that does not need to actually exist in the routing table
ContactRequestQueue::iterator
R2Kademlia::scheduleRequest(ContactRequestQueue& q, KadContact&& tmpcontact)
{
	auto it= std::lower_bound(q.begin(), q.end(), tmpcontact, [&](const auto& lhs, const auto& rhs) -> bool { return lhs.getKey() < rhs.getKey(); });
	if (it != q.end() and it->getKey() == tmpcontact.getKey()) {
		// entry exists already
		stateseqno_t expected_seqno(tmpcontact.getStateSeqNo());
		// same neighbor/contact already present, but
		// probably need to update the state seq no
		if (not expected_seqno.isInvalid()) {
			// only update when actually larger
			if (expected_seqno > it->getStateSeqNo())
				it->setStateSeqNo(expected_seqno);
		}
		EV_TRACE << "scheduleRequest(" << q.getDescription() << "): " << tmpcontact.getKey().shortStr() << " already queued, expected SSeq#" << expected_seqno << endl;
		return it;
	}
	else
	{ // it == q.end() or it->getKey() != tmpcontact.getKey()
                // entry does not exist yet, so insert it at the right place
		it= q.insert(it, std::move(tmpcontact));
	}

	EV_TRACE << "scheduleRequest(" << q.getDescription() << "): "  << it->getKey().shortStr() << " queued, expected  SSeq#" << it->getStateSeqNo() << endl;

	if (not q.isScheduled())
	{
	  simtime_t wait_time(uniform(q.initialScheduleDelay/2, 3*q.initialScheduleDelay/2));
		EV_TRACE << "scheduling " << q.getDescription() << " for " << it->getKey().shortStr()
			 << " in " << wait_time << 's' << endl;
		addOneShotTimer(wait_time, q.sendscheduledcall);
		q.setScheduled();
	}
	return it;
}


// this schedules a request for getting the overlay neighbors
// if the node is in the GetPNQueue, we simply set an additional flag in the contact copy inside the queue
void
R2Kademlia::scheduleGetONReq(const KadContact& tmpcontact) {
	// check first for same node in GetPNQueue (so node is in vicinity)
	auto it= std::lower_bound(GetPNQueue.begin(), GetPNQueue.end(), tmpcontact,
				  [&](const auto& lhs, const auto& rhs) -> bool { return lhs.getKey() < rhs.getKey(); });
	if (it != GetPNQueue.end() and it->getKey() == tmpcontact.getKey()) {
		EV_TRACE << "scheduleGetONReq() found entry for " << tmpcontact.getKey().shortStr() << " in GetPNQueue, setting get_ON flag" << endl;
		it->setState(KadContact::State::_get_ON);
		// delete it from GetONQueue
		// if (auto ONit= std::lower_bound(GetONQueue.begin(), GetONQueue.end(), tmpcontact,
		// 		  [&](const auto& lhs, const auto& rhs) -> bool { return lhs.getKey() < rhs.getKey(); });
		//     ONit != GetONQueue.end()) {
		// 	GetONQueue.erase(ONit);
		// }
	}
	else {
		// put into GetONQueue (as copy)
		auto it= scheduleRequest(GetONQueue, KadContact(tmpcontact));
		ASSERT(it != GetONQueue.end());
		if (it != GetONQueue.end()) {
			it->setState(KadContact::State::_get_ON);
		}
	}
};


bool
R2Kademlia::sendScheduledRequests(ContactRequestQueue& q) {
	simtime_t delay_next_call(0);
	EV_TRACE << "sendScheduledRequests(" << q.getDescription() << ") called, queued requests:" << q.size() << endl;
	while (not q.empty())
	{ // for getPN getON we have no neighbors and probably also no contacts in the routing table available
		// normally we would use front(), but for vectors back() and pop_back() is easier
		// due to the sorted order
		auto& tmpcontact= q.back();
		bool isPN= (tmpcontact.getPath().size() == 0); // Physical neighbor

		stateseqno_t lastseenseqno(stateseqno_t::invalidSeqNo);
		auto rt_contact= routingTable.getContact(tmpcontact.getKey());

		// check for current status of scheduled contact if locally known
		// note: contact may have been invalidated or changed meanwhile
		if (rt_contact != nullptr) { // so contact still exists
			lastseenseqno= rt_contact->getSyncPNSeqNo();
		}
		else { // try to find node in vicinity
			// e.g., if we only received a PNHello, PN is not a contact yet, but it is in the vicinity
			// furthermore, other nodes may never be a contact due to bucket size restrictions
			if (tmpcontact.getPath().size() < nodeVicinityGraph::vicinityDiscoveryRadius) {
				if (auto vnodep= vicinity.getNode(tmpcontact.getKey()); vnodep != nullptr) {
					lastseenseqno= vnodep->stateseqno;
					EV_TRACE << tmpcontact.getKey().shortStr() << " found in vicinity, lastseenseqno==" << lastseenseqno << endl;
				}
			}
		}

		// temporary contact object in case we don't have a real one yet
		if (rt_contact == nullptr)
			rt_contact= &tmpcontact;

		auto expected_seqno(tmpcontact.getStateSeqNo());
		auto getOverlayNeighborsSource(tmpcontact.getState() == KadContact::State::_get_ON ? true : false);
		// check whether update is needed (invalidSeqNo means unconditionally update)
		// this makes no sense for querying overlay neighbors as this is not reflected in sequence numbers
		if (getOverlayNeighborsSource or lastseenseqno.isInvalid() or expected_seqno.isInvalid() or expected_seqno > lastseenseqno) {
			if (isPN and not neighborTable.isNeighbor(tmpcontact.getKey())) {
				EV_ERROR << q.getDescription() << " cannot be sent, because contact " << tmpcontact.getKey().shortStr() << " not in neighbor table anymore" << endl;
			}
			else {
				// send the request (ULNDiscReq, QueryRouteReq(PN), QueryRouteReq(ON))
				// e.g., see R2Kademlia::sendDiscoveryReq(KadContact_p contact)
				delay_next_call= q.sendrequestcall(*rt_contact, getOverlayNeighborsSource ? get_OverlayNeighborsSource : unset);
			}
		}
		else {
			EV_TRACE << "Obsolete " << q.getDescription() << " for " << rt_contact->getKey().shortStr() << " avoided due to current seqno "
				 << lastseenseqno << " expected: "
				 <<  expected_seqno << endl;
		}
		// remove the scheduled request now
		q.pop_back();

		if (not q.empty() and delay_next_call > 0) {
			// schedule next call
			addOneShotTimer(delay_next_call, q.sendscheduledcall);
			q.setScheduled();
			EV_TRACE << "scheduling " << q.getDescription() << " in " << delay_next_call << 's' << endl;

			return false;
		}
	}

	// queue is empty now
	// next call of this procedure will be scheduled on demand
        q.clearScheduled();
	return true;
}


// check whether we need to resync with physical neighbor, update lastseen
// normal stateSeqNo not sufficient, because it may have been updated by another message already
// getSyncPNSeqNo() tells us when we received the last full PN list from this PN
bool R2Kademlia::checkForResync(const OverlayKey& contact, stateseqno_t newseqno) {
	if (KadContact* contact_p= const_cast<KadContact*>(routingTable.getContact(contact)); contact_p != nullptr and contact_p->isPhysicalNeighbor()) {
		contact_p->updateLastSeen();
		// if state seqno of PN has changed resync
		if (contact_p->getSyncPNSeqNo().isInvalid() or newseqno > contact_p->getSyncPNSeqNo()) {
			EV_TRACE << "resync required for " << contact_p->getKey().shortStr() << " new: "
				 << newseqno << " old: " << contact_p->getSyncPNSeqNo() << endl;

			reSyncPN(*contact_p, newseqno);

			return true;
		}
	}

	return false;
}


} // end namespace
