#include "KIRA.h"

namespace routingsim {


// sendPathProbe along proposed routing path to check for path validity on demand
void R2Kademlia::scheduleOnDemandPathProbeReq(KeyRequestQueue& q, const KadContact& tmpcontact, RequestQueueElementActionType action)
{
	// the following compare is a bit hacky:
	// we create a new ID by shifting the OverlayKey 8 bits to the left and using the action field for the lower 8 bit
	// therefore, we get different entries for different actions
	auto it= std::lower_bound(q.begin(), q.end(), tmpcontact,
				  [&](const auto& lhs, const auto& rhs) -> bool { return ((lhs.first.toUInt() << 8) | lhs.second) < ((rhs.getKey().toUInt() << 8) | action); });
	if (it != q.end() and *it == std::make_pair(tmpcontact.getKey(),action)) {
		// entry exists already
		return;
	}
	else
	{ // it == q.end() or it->getKey() != tmpcontact.getKey()
                // entry does not exist yet, so insert it at the right place
		q.insert(it, std::make_pair(tmpcontact.getKey(),action));
	}

	EV_TRACE << "scheduleRequest(" << q.getDescription() << "): "  << tmpcontact.getKey().shortStr() << " queued, expected  seq#" << tmpcontact.getStateSeqNo() << " action: " << ((action & probeProposedPath) ? "proposed" : "active") << ((action & pathSetupCheck) ? " pathsetup" : "") << endl;

	if (not q.isScheduled())
	{
		simtime_t wait_time(uniform(q.initialScheduleDelay/2, 3*q.initialScheduleDelay/2));
		EV_TRACE << "scheduling " << q.getDescription() << " for " << tmpcontact.getKey().shortStr()
			 << " in " << wait_time << 's' << endl;
		addOneShotTimer(wait_time, q.sendscheduledcall);
		q.setScheduled();
	}
}


// sendPathProbe along full source routing path to existing contact w/ given key
// called by periodic refresh function doProbeNeighbors() and doProbeContacts()
void R2Kademlia::sendPathProbe(const OverlayKey& contactkey)
{
	if (auto contactp(routingTable.getContact(contactkey)); contactp != nullptr)
		sendPathProbeToContact(*contactp, pathSetupCheck);
	else {
		EV_TRACE << "not sending ProbeReq from " << myOverlayKey.shortStr() << " to " << contactkey.shortStr() << " because of invalid contact" << endl;
	}
}



// sendPathProbe along full source routing path, only works with existing contacts
// called by periodic refresh function doProbeNeighbors() and doProbeContacts()
simtime_t R2Kademlia::sendOnDemandPathProbe(KeyRequestQueue::value_type keyactionpair, RequestQueueElementActionType unused_arg)
{
        // we rate limit the requests
	simtime_t delay(0);

	auto& [contactkey, action]= keyactionpair;
	auto contactp= routingTable.getContact(contactkey);
	if (contactp == nullptr)
		return delay;

	// default (see R2Kademlia.h): 50000µs is 50ms, so the rate limit is 20 Reqs/s
	static const simtime_t mindelay(SimTime(OnDemandPathProbeReqWaitTime,SIMTIME_US));
	if  ( simTime() < LastOnDemandProbeReq ) {
		// last was scheduled in future, schedule it mindelay behind that
		delay= LastOnDemandProbeReq-simTime()+mindelay;
	}
	else { // last send in past
		if (simTime()-LastOnDemandProbeReq < mindelay) {
			// we are too early, so delay until mindelay is fulfilled
			delay= LastOnDemandProbeReq+mindelay-simTime();
		}
		// otherwise no extra delay necessary
	}

	EV_TRACE << "sendOnDemandPathProbe(" << contactkey.shortStr() << ") next delay: " << delay << endl;

	sendPathProbeToContact(*contactp, action);

	LastOnDemandProbeReq= simTime()+delay;

	return delay;

}


void
R2Kademlia::sendScheduledPathProbeRequests(KeyRequestQueue& q) {
	simtime_t delay_next_call(0);
	EV_TRACE << "sendScheduledRequests(" << q.getDescription() << ") called, queued requests:" << q.size() << endl;
	while (not q.empty())
	{ // for getPN getON we have no neighbors and probably also no contacts in the routing table available
		// normally we would use front(), but for vectors back() and pop_back() is easier
		// due to the sorted order
		auto keyactionpair= q.back();

		// send the path probe request (second argument is ignored, because the pair already contains the action)
		delay_next_call= q.sendrequestcall(keyactionpair, unset);

		// remove the scheduled request now
		q.pop_back();

		if (not q.empty() and delay_next_call > 0) {
			// schedule next call
			addOneShotTimer(delay_next_call, q.sendscheduledcall);
			q.setScheduled();
			EV_TRACE << "scheduling " << q.getDescription() << " in " << delay_next_call << 's' << endl;

			return;
		}
	}

	// queue is empty now
	// next call of this procedure will be scheduled on demand
	q.clearScheduled();
}


// sendPathProbe along full source routing path of temporary contact
simtime_t R2Kademlia::sendPathProbeToContactCandidate(const KadContact& tmpcontact, RequestQueueElementActionType action)
{
        // we rate limit the requests
	simtime_t delay(0);

	// default (see KIRA.h): 100'000µs is 100ms, so the rate limit is 10 Reqs/s
	static const simtime_t mindelay(SimTime(OnDemandPathProbeReqWaitTime,SIMTIME_US));
	if  ( simTime() < LastContactCandidateProbeReq ) {
		// last was scheduled in future, schedule it mindelay behind that
		delay= LastContactCandidateProbeReq-simTime()+mindelay;
	}
	else { // last send in past
		if (simTime()-LastContactCandidateProbeReq < mindelay) {
			// we are too early, so delay until mindelay is fulfilled
			delay= LastContactCandidateProbeReq+mindelay-simTime();
		}
		// otherwise no extra delay necessary
	}

	EV_TRACE << "sendPathProbeToContactCandidate(" << tmpcontact.getKey().shortStr() << ") next delay: " << delay << endl;

	sendPathProbeToContact(tmpcontact, action);

	LastContactCandidateProbeReq= simTime()+delay;

	return delay;

}


void
R2Kademlia::sendScheduledContactCandidateProbeReqs(ContactRequestQueue& q) {
	simtime_t delay_next_call(0);
	EV_TRACE << "sendScheduledRequests(" << q.getDescription() << ") called, queued requests:" << q.size() << endl;
	while (not q.empty())
	{
		// normally we would use front(), but for vectors back() and pop_back() is easier
		// due to the sorted order
                auto contact = q.back();
		// check whether it is a contact already (no need to check then)
		if (not routingTable.isKnown(contact.getKey())) {
			// send the path probe request (second argument is ignored, because the pair already contains the action)
                        delay_next_call= q.sendrequestcall(contact, pathSetupCheck);
                }
                else { // skip this element: remove it from the queue and do not send anything
			delay_next_call = 0;
			EV_TRACE << "skipped contact candidate " << contact.getKey().shortStr() << " as it is already a contact" << endl;
		}

                // remove the scheduled request now
		q.pop_back();

		if (not q.empty() and delay_next_call > 0) {
			// schedule next call
			addOneShotTimer(delay_next_call, q.sendscheduledcall);
			q.setScheduled();
			EV_TRACE << "scheduling " << q.getDescription() << " in " << delay_next_call << 's' << endl;

			return;
		}
	}

	// queue is empty now
	// next call of this procedure will be scheduled on demand
	q.clearScheduled();
}


// Path Probing: sends a ProbeReq to check for validity of an existing path
// the ProbeReq will travel the full path to its final destination if the path is intact
// if pathsetupcheck is true it will also refresh PathIDs that have been installed in intermediate nodes
void R2Kademlia::sendPathProbeToContact(const KadContact& contact, RequestQueueElementActionType action)
{
	// must be a valid contact
	if ( contact.getKey().isUnspecified() or contact.isInvalid() ) {
		EV_TRACE << "not sending ProbeReq from " << myOverlayKey.shortStr() << " to " << contact.getKey().shortStr() << " because of invalid contact" << endl;
		return;
	}
	if ( (action & probeProposedPath) and not contact.proposedPathDefined() ) {
		EV_TRACE << "not sending ProbeReq from " << myOverlayKey.shortStr() << " to " << contact.getKey().shortStr() << " because proposed path not defined anymore" << endl;
		return;
	}

	EV_TRACE << "sending ProbeReq from " << myOverlayKey.shortStr() << " to " << contact.getKey().shortStr();
	std::unique_ptr<KadProbeReq> probereq(nullptr);
	// if action says so and proposed path is present, we probe for this
	if ( (action & probeProposedPath) and contact.proposedPathDefined()) {
		probereq= makePacket<KadProbeReq>(contact, "ProbeReq", false);
		probereq->appendRoute(contact.getRawProposedPath(), contact.getKey());
		EV_TRACE << " (Proposed Path): " << contact.getRawProposedPath();
	}
	else {
		probereq= makePacket<KadProbeReq>(contact, "ProbeReq");
		EV_TRACE << " (Active Path): " << contact.getRawActivePath();
	}

	// need to set the PathCheck flag?
	if (action & pathSetupCheck) {
		probereq->setPathCheck(true);
		EV_TRACE << " (PathSetupCheck)";
	}
	EV_TRACE << endl;
	// adding failed local links to NotVia list
	auto failedPNList(routingTable.getFailedPNs());
	probereq->setNotVia(true);
	probereq->setFailedLinkArraySize(failedPNList.size());
	probereq->setFailedLinkAgeArraySize(failedPNList.size());
	unsigned long i= 0UL;
	for ( auto& failedPN : failedPNList ) {
		probereq->setFailedLink(i,{myOverlayKey,failedPN});
		if (auto pncontact= routingTable.getContact(failedPN); pncontact != nullptr) {
			probereq->setFailedLinkAge(i,pncontact->getPathAge());
		}
		else { // this should not happen
			ASSERT(false);
			// use timer min value for estimation
			probereq->setFailedLinkAge(i,notifyLinkFailureWaitTime/2);
		}
		i++;
	}

	sendRpcCall(std::move(probereq),
		std::bind(&R2Kademlia::handleProbeResponse, this, ph::_1, ph::_2),
		std::bind(&R2Kademlia::handleProbeResponse, this, ph::_1, ph::_2),
		std::bind(&R2Kademlia::handleRPCTimeOut, this, ph::_1, ph::_2));
}


// sends back an answer for a path probe
void R2Kademlia::handlePacket(std::unique_ptr<KadProbeReq> packet) {
	// update StateSeqNo and lastseen of origin contact if path is the same
	// since paths may be asymmetric, this ProbeReq may have taken a different path
	if (KadContact* contact_p= const_cast<KadContact*>(routingTable.getContact(packet->getSource())); contact_p != nullptr) {
		const auto takenpath(packet->reversePath(packet->getIntermediatePath()));
		if (takenpath == contact_p->getPath()) { // if it is the same path that we use, update contact
			if (contact_p->getStateSeqNo() <= packet->getStateSeqNo()) {
				// update contact data
				contact_p->setStateSeqNo(packet->getStateSeqNo());
				contact_p->updateLastSeen();
				contact_p->setNodeDegree(packet->getSrcNodeDegree());
				// note: also if path was working we should not use updateLastPathRefresh(), because PIDs are unidirectional
			}
			else
				EV_WARN << "State sequence number of message smaller (" << packet->getStateSeqNo() << ") than for contact (" << contact_p->getStateSeqNo() << ")" << endl;

		}
	}

	sendPacket(makeAnswer<KadProbeRsp>(*packet));
}


bool R2Kademlia::handleProbeResponse(KadPacket_p answer, RPCState& rpcState) {
	// callback will be invoked by RPCProtocol::handleAnswer()
	// in case the response contained an error it will be handled by onErrorPacket first
	// in case of failure we need to return true since this RPC is finished then
	if (rpcState.status != RPC_SUCCESS) return true;
	ASSERT(answer->getSource() == rpcState.original_packet->getTarget());
	std::unique_ptr<KadProbeRsp> probeRsp = safe_packet_cast<KadProbeRsp>(answer);

	// update StateSeqNo and lastseen of origin contact
	if (KadContact* contact_p= const_cast<KadContact*>(routingTable.getContact(probeRsp->getSource())); contact_p != nullptr) {
		// usually this should not be necessary as a path update is done before
		if (contact_p->getStateSeqNo() <= probeRsp->getStateSeqNo()) {
			contact_p->setStateSeqNo(probeRsp->getStateSeqNo());
			//contact_p->setValid();
			contact_p->updateLastSeen();
			contact_p->updateLastPathRefresh();
			contact_p->setNodeDegree(probeRsp->getSrcNodeDegree());
		}
		else
			EV_WARN << "State sequence number of message smaller (" << probeRsp->getStateSeqNo() << ") than for contact (" << contact_p->getStateSeqNo() << ")" << endl;
	}

	return true;
}

} // namespace
