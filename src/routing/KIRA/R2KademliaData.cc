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


#include "KIRA.h"
#include "KadPacket_m.h"
#include "KadObserver.h"
#include "PathIDUtils.h"

namespace routingsim {




bool R2Kademlia::doDataProbe(short timerID) {
	if (neighborTable.empty()) return false; // we are isolated
	const OverlayKey* probeKey(nullptr);
	unsigned int maxtries= 10;
	bool nodefound= false;
	do {
		// during periods of failed links we want to preferably test reachability for affected nodes
		if (failureFocussedProbing and getNetworkManager()->getFailedNodes().size() > 0) {
			// select random nodes out of set of failed nodes
			auto nodeit= getNetworkManager()->getFailedNodes().cbegin();

			for (unsigned int index= intuniform(0, getNetworkManager()->getFailedNodes().size()-1); index > 0; index--)
				nodeit++;

			if (nodeit != getNetworkManager()->getFailedNodes().cend()) {
				nodeid_t nodecandidate(*nodeit);
				// if node is active and not isolated
				if (getNetworkManager()->getTopology().isActive(nodecandidate) == true and
				    getNetworkManager()->getTopology().getNodeDegree(nodecandidate) > 0) {
					if (getNetworkManager()->getFailedNodes().size() > 1) { // not the only choice
						// try this failed node
						probeKey = &getKadObserver().getNodeKey(nodecandidate);
						nodefound= true;
					}
				}
			}
		}

		if (not nodefound) { // try different non affected node as alternative
			probeKey = &getKadObserver().getRandomNodeKey();
		}

		// check whether destination node is actually reachable
		if (probeKey != nullptr) {
			if (getKadObserver().getPhysicalDistance(myOverlayKey, *probeKey) == 0) {
				EV_TRACE << "Node " << probeKey->shortStr() << " not reachable" << endl;
				nodefound= false;
			}
			else { // node is reachable
				nodefound= true;
			}
		}
		else {
			nodefound= false;
		}

		maxtries--;
	} while ( nodefound == false and maxtries > 0 );

	if (nodefound and probeKey != nullptr)
		sendData(*probeKey);
	return false;
}


void R2Kademlia::handleDataPacket(NodePkt_p pkt) {
	ASSERT( pkt != nullptr );
	EV_TRACE << "handleDataPacket() - ";

	KadDataPacket_p packet = std::unique_ptr<KadDataPacket>(check_and_cast<KadDataPacket*>(pkt->decapsulate()));
	
	EV_TRACE << "received data packet from "
	   << packet->getSrcNodeID().shortStr() << " to "
	   << packet->getDstNodeID().shortStr() << " overlaysrc: " << packet->getPathSrcNodeID().shortStr()
	   << " HL:" << packet->getHopLimit() << endl;

	// we should check whether the link via this message travelled is
	// still active, otherwise we should drop this message as it was affected
	// by the link failure, e.g., more precisely using (getDuration(), getArrivalTime())
	const hop_t taken_hop= pkt->getHop();
	//ASSERT(Node::getNode(this)->getNodeId() == taken_hop.to);

	if (not getNetworkManager()->getTopology().isConnected(taken_hop.from,taken_hop.to))
	{
		EV_WARN << "received packet (msg id: " << packet->getId() << ") via an inactive link, thus will discard" << endl;
		if (packet->getControlInfo() == nullptr)
			packet->setControlInfo(pkt.release());
		
		countDropped(*packet, packet->getDstNodeID(), travelledDisabledLink);
		return;
	}

	// this should either forward to next hop or deliver locally
	forwardDataPacket(std::move(packet), std::move(pkt));
}



// uses the PID of the packet to determine the next hop and transmits it
// @returns true in case of successful forwarding/transmission to next hop or false in case of dropped packet
bool R2Kademlia::forwardDataPacketToNextHop(KadDataPacket_p& packet, NodePkt_p& nodePkt)
{
	ASSERT(nodePkt != nullptr);
	bool packetorigin(packet->getSrcNodeID() == myOverlayKey);
	// lookup outgoing PathID (next hop) based on incoming PathID

	auto fwdentry= fwdTablePathID.PIDFwdTable.find(packet->getFirstPathID());
	if (fwdentry != fwdTablePathID.PIDFwdTable.end())
	{ // entry found
		FwdTableEntry& nh= fwdentry->second;
		if (nh.isValid() or nh.isObsolete() or nh.isInvalid())
		{
			if (nh.isInvalid()) {
				EV_ERROR << "invalid PathID entry hit: " << packet->getFirstPathID() << endl;
			}

			if (nh.isObsolete()) { // obsolete, but maybe still good enough to forward
				EV_WARN << "obsolete PathID entry hit: " << packet->getFirstPathID() << endl;
			}

			if (nh.fwdAction == FwdTableEntry::Action::fwdNextPathID)
			{ // set new outgoing PathID
				packet->setFirstPathID(nh.outgoingPathID);
				EV_TRACE << "Outgoing PathID: " << nh.outgoingPathID << endl;
			}
			else
			if (nh.fwdAction == FwdTableEntry::Action::fwdNextNodeId)
			{ // next hop will be the next hop where the path ends
				// either next PID will be used or next overlay hop will determine next segment
				// basically pop the First PID
				packet->setFirstPathID(packet->getSecondPathID());
				packet->setSecondPathID(PathID::UndefinedPathID);
				EV_TRACE << "Forward to Next NodeID: " << nh.nextHopNode.shortStr() << endl;
				EV_TRACE << "Outgoing PID: " << packet->getFirstPathID() << endl;
			}
			else
			if (nh.fwdAction == FwdTableEntry::Action::fwdlocal)
			{
				EV_TRACE << "Forward action: local delivery" << endl;
			}
		}
		else
			EV_ERROR << "hit forwarding table entry with undefined state" << endl;

	       	nodeid_t nid = neighborTable.getNodeId(nh.nextHopNode);
		if (nid != undefined_node) {
			nh.incrementUsageCount();
			packet->advanceProgress(); // decrease hop limit
			nodePkt->encapsulate(packet.release());
			sendToNode(nid, std::move(nodePkt));
			if (not packetorigin) {
				forwardedDataPackets++;
			}
			EV_TRACE << "send to next hop nid=" << nid << endl;
			// true means success
			return true;
		}
		else {
			// Trying to send to a next hop that is not (available as) a physical next hop
			// That will not fly...

			// in case the link to the next hop has failed,
			// try to reroute if possible,
			// send back segment failure to inform the sender about the problem
			// TODO: send also known replacement path with the error indication
			if (auto nexthop_contact_p= routingTable.getContact(nh.nextHopNode);
			    nexthop_contact_p != nullptr and nexthop_contact_p->isPhysicalNeighbor()) {

				reasoncode dropreason(missingNeighbor);
				// several cases possible:
				// 1) we know the final destination as contact and have a valid route to it;
				//    this is preferrable to the next options, as we can fully replace first and second PID
				// 2) we can detour to the next hop by inserting a new PID (i.e., second segment PID must be empty)
				// 3) we try to reroute to the next hop in case the path should have ended there
				//    (same as case 2, but guaranteed to work as the 2nd segment is not present)
				if (auto dst_contact_p = routingTable.getContact(packet->getDstNodeID());
				    dst_contact_p != nullptr and
				    dst_contact_p->isValid()) {

					EV_TRACE << "forwardDataPacketToNextHop: link to " << nh.nextHopNode.shortStr() << " not available, try to reroute to final destination "
						 << dst_contact_p->getKey().shortStr() << " directly via " << dst_contact_p->getPath() << endl;
					if ( auto pidentry= fwdTablePathID.NIDPIDTable.find(dst_contact_p->getKey());
					     pidentry != fwdTablePathID.NIDPIDTable.cend() )
					{ // pidentry.second is incomingPathID
						// this will be used to lookup the outgoing PID in forwardDataPacket()
						packet->setFirstPathID(pidentry->second);
						// in case that we have a longer path, a second PID for the last path segment needs to be specified
						if ( auto secondpidentry= fwdTablePathID.NIDPIDTableLastSeg.find(dst_contact_p->getKey());
						     secondpidentry != fwdTablePathID.NIDPIDTableLastSeg.cend() ) {
							packet->setSecondPathID(secondpidentry->second);
						}

						// send back error message (requires copy of packet) to inform about broken link in source route
						// even if we are able to reroute the data packet
						onMissingNeighbor(std::make_unique<KadDataPacket>(*packet), nh.nextHopNode);
						// need to check now that this next hop is available, otherwise drop to avoid endless recursion
						if (auto newfwdentry= fwdTablePathID.PIDFwdTable.find(packet->getFirstPathID());
						    newfwdentry != fwdTablePathID.PIDFwdTable.end())
						{ // entry found
							FwdTableEntry& new_nh= newfwdentry->second;
						        if (neighborTable.getNodeId(new_nh.nextHopNode) != undefined_node) {
								// recursive call
								return forwardDataPacketToNextHop(packet, nodePkt);
							}
							else {
								EV_TRACE << "reroute not successful, next hop " << new_nh.nextHopNode.shortStr() << " also not available" << endl;
							}

							packet->setControlInfo(nodePkt.release());
							countDropped(*packet, packet->getDstNodeID(), dropreason);

							return false;
						}
					}
					else { // no PID found (usually for PNs, but this is not reachable)
						// this can also occur when the path has not been setup yet
						EV_ERROR << "No PID found for destination contact " << dst_contact_p->getKey() << endl;

						// send back error message (requires copy of packet)
						onMissingNeighbor(std::make_unique<KadDataPacket>(*packet), nh.nextHopNode);

						packet->setControlInfo(nodePkt.release());
						countDropped(*packet, packet->getDstNodeID(), dropreason);

						return false;
					}
				}

				// do we have another path to the next hop available that we can insert?
				// this is only useful if the contact is in state valid
				if (auto pidentry= fwdTablePathID.NIDPIDTable.find(nh.nextHopNode);
				    nexthop_contact_p->isValid()
				    and pidentry != fwdTablePathID.NIDPIDTable.cend()
				    and pidentry->second != PathID::UndefinedPathID)
				{
					ASSERT(pidentry->second != packet->getFirstPathID());

					EV_TRACE << "forwardDataPacketToNextHop: link to " << nh.nextHopNode.shortStr() << " not available, try to reroute to next hop via " << nexthop_contact_p->getPath() << endl;

					// must check for 2nd segment PID
					auto lastsegpidentry= fwdTablePathID.NIDPIDTableLastSeg.find(nh.nextHopNode);
					if ( lastsegpidentry != fwdTablePathID.NIDPIDTableLastSeg.cend() ) {
						// second segment required for detour to failex next hop (path is at least 4 hops)
						if (packet->getFirstPathID() == PathID::UndefinedPathID) {
							// if there is no second segment used yet (*), we can reroute now
							// otherwise we do not want to introduce more nested PID stacking
							// (to avoid overwriting the 2nd segment) and packet will be dropped instead
							// (*) note that the current FirstPathID is the former SecondPathID
							packet->setFirstPathID(pidentry->second);
							packet->setSecondPathID(lastsegpidentry->second);

							// send back error message (requires copy of packet)
							onMissingNeighbor(std::make_unique<KadDataPacket>(*packet), nh.nextHopNode);
							if (auto newfwdentry= fwdTablePathID.PIDFwdTable.find(packet->getFirstPathID());
							    newfwdentry != fwdTablePathID.PIDFwdTable.end())
							{ // entry found
								FwdTableEntry& new_nh= newfwdentry->second;
								if (neighborTable.getNodeId(new_nh.nextHopNode) != undefined_node) {
									// recursive call
									return forwardDataPacketToNextHop(packet, nodePkt);
								}
								else
									EV_TRACE << "reroute not successful, next hop " << new_nh.nextHopNode.shortStr() << " also not available" << endl;
							}
						}
						else
							dropreason= rerouteDataSegmentLimitExceeded;
					}
					else { // no second segment required for detour
						// move second segment to where it belongs and
						// insert new PID for detour as first PID
						packet->setSecondPathID(packet->getFirstPathID());
						packet->setFirstPathID(pidentry->second);

						// send back error message (requires copy of packet)
						onMissingNeighbor(std::make_unique<KadDataPacket>(*packet), nh.nextHopNode);

						// recursive call TODO: Stack Overflow in mobility
						// return forwardDataPacketToNextHop(packet, nodePkt);
					}
				} // endif PathID to next hop available

				packet->setControlInfo(nodePkt.release());
				countDropped(*packet, packet->getDstNodeID(), dropreason);

				// send back segment failure
				onMissingNeighbor(std::move(packet), nh.nextHopNode);
				return false;
			}
			else { // else: next hop not found (TODO: this cannot happen during data packet forwarding)
				EV_ERROR << "forwardDataPacketToNextHop: wrong next hop"
					 << nh.nextHopNode.shortStr() << endl;

				packet->setControlInfo(nodePkt.release());
				countDropped(*packet, packet->getDstNodeID(), nextHopUnknownDataFwd);
				return false;
			}

		} // end else (next hop not available)

	} // endif forwarding entry found
	else {
		EV_WARN << "forwardDataPacket: incoming PathID " << packet->getFirstPathID() << " not found in FwdPIDTable. Dropping packet" << endl;
		inconsistentPathHit++;

		packet->setControlInfo(nodePkt.release());
		countDropped(*packet, packet->getDstNodeID(), missingPID);
		notifySender(KAD_PathIDUnknown, std::move(packet), OverlayKey::UNSPECIFIED_KEY);
		return false;
	}

	// false means forwarding failure
	return false;
}


void R2Kademlia::forwardDataPacket(KadDataPacket_p packet, NodePkt_p nodePkt) {

	ASSERT(packet != nullptr);
	ASSERT(packet->getType() == KAD_Data);
	ASSERT(packet->getControlInfo() == nullptr);
        //ASSERT(packet->hasRemainingHops());
	ASSERT(nodePkt != nullptr);

	const auto& finaldest(packet->getDstNodeID());
	EV_TRACE << "forwardDATAPacket: finaldest=" << finaldest.shortStr()
		 << " overlaysrc=" << packet->getPathSrcNodeID().shortStr()
		 << " 1st PID=" << packet->getFirstPathID() << " 2nd PID=" << packet->getSecondPathID()  << endl;
	
	if (packet->hasRemainingHops() == false)
	{ // Loop detected: this should not occur
		if (packet->getControlInfo() == nullptr) {
			packet->setControlInfo(nodePkt.release());
		}
		countDropped(*packet, finaldest, hopLimitExceeded);
		notifySender(KAD_HopLimitExceeded, std::move(packet), OverlayKey::UNSPECIFIED_KEY);
		return;
	}

	// should be just my own key or direct neighbor!
	if (finaldest == myOverlayKey)
	{ // destined to myself
		EV_TRACE << "next hop is me (after " << getHopLimitDefault()-packet->getHopLimit() << " hops / " << packet->getOverlayHops() <<  " overlayhops )" << endl;
		// successfully delivered! consume packet
		packet->setControlInfo(nodePkt.release());

		countReceived(*packet);
        handleIfFitting<KadFloodPacket>(packet);
		return;
	}

	//  own originated packets will possess the right (incoming) DestPathID, see makeDataPacket()
	PathID incomingPathID(packet->getFirstPathID());

	// incomingPathID is only undefined for packets whose PathID determined Path ends here
	if (incomingPathID == PathID::UndefinedPathID)
	{ // next hop (intermediate) destination ID of this path (final destination may be elsewhere)
		// increase overlayHop count
		packet->countOverlayHop();
		uint8_t reasoncode(unspecified);
		// Since our NIDPIDTable only allows for exact matches, we use our routing table for lookup
		auto contacts = routingTable.getClosestContacts(finaldest, findNodeNumber, true);
		if (contacts.size() > 0)
		{
			// we need to check every contact for a valid NID entry
			for (auto cit(contacts.cbegin()); cit != contacts.cend(); cit++)
			{
				cit = chooseNextHop(contacts, finaldest, cit);
				auto& contact(**cit);
				ASSERT(contact.isValid());
				ASSERT(!contact.getKey().isUnspecified());
				
				// next hop is not closer than this node
				if ((contact.getKey() ^ finaldest) > (myOverlayKey ^ finaldest)) {
					EV_WARN << "next overlay hop is " << contact.getKey().shortStr()
						<< " - cannot get closer, dropping packet" << endl;
					packet->setControlInfo(nodePkt.release());
					countDropped(*packet, finaldest, routeFailureDeadEnd);
					return;
				}

				// set new source for the next segment
				packet->setPathSrcNodeID(myOverlayKey);
				EV_TRACE << "next overlay hop is " << contact.getKey().shortStr()
					 << " (" << contact.getPathLength()+1 << " hops)" << endl;

				// now try to find a PathID to the next overlay hop

				if (auto pidentry= fwdTablePathID.NIDPIDTable.find(contact.getKey());
				    pidentry != fwdTablePathID.NIDPIDTable.end() ) { // pidentry.second is incomingPathID
					// TODO check whether PID/FwdEntry is valid

					// NID -> PID mapping exists and is valid
					// this will be used to lookup the outgoing PID in forwardDataPacket()
					packet->setFirstPathID(pidentry->second);

					EV_TRACE << "new inPID for dest " << contact.getKey().shortStr() << ": " << pidentry->second << endl;
					if (auto secondpidentry= fwdTablePathID.NIDPIDTableLastSeg.find(contact.getKey());
					    secondpidentry != fwdTablePathID.NIDPIDTableLastSeg.end() ) {
						ASSERT(packet->getSecondPathID() == PathID::UndefinedPathID);
						packet->setSecondPathID(secondpidentry->second);
						EV_TRACE << "2nd segment PID: " << secondpidentry->second << endl;
					}

					if (not forwardDataPacketToNextHop(packet, nodePkt))
					{
						EV_WARN << "PID based forwarding failed while trying to forward to next hop." << endl;
						return;
					}
					else // forwardDataPacketToNextHop had success
						return;
				}
				else {
					// no PID found this may be the case when PathSetup has been completely finished
					// try next contact instead
					EV_TRACE << "No NID entry found for " << contact.getKey().shortStr() << " trying next contact" << endl;
				}
			} // end for

				EV_ERROR << "forwardDataPacket: No usable contact found in " << contacts.size() << " contacts – NID and PID missing – Dropping packet" << endl;
				reasoncode= missingPID;
		}
		else
		{
			// Trying to send to a next hop that is not known
			// That will not fly...
			EV_ERROR << "forwardDataPacket: next hop for destination " << finaldest.shortStr() << " cannot be found (0 contacts)" << endl;
			// forwarding failed
			reasoncode= routeFailureDeadEnd;
		}

		// this is an error condition
		packet->setControlInfo(nodePkt.release());
		countDropped(*packet, finaldest, reasoncode);
		return;
	} // if incoming PathID is undefined

	// incomingPathID is present, simply look it up and forward accordingly
	packet->setFirstPathID(incomingPathID);
	// forwardDataPacketToNextHop will either forward (returns true) or drop the packet and report an error msg back
	forwardDataPacketToNextHop(packet, nodePkt);
}


void R2Kademlia::sendPacket(KadDataPacket_p packet) {
	//ASSERT(packet->getSource() == myOverlayKey);
	ASSERT(packet->getControlInfo() == nullptr);
	auto nodePkt = std::make_unique<NodePkt>(packet->getName());
	nodePkt->setType(KAD_DATA_PACKET_TYPE);
	//nodePkt->setAttributes(nodePkt->getAttributes() | NODEPKT_ATTR_DATA_PLANE_PROCESSING_DELAY);
	if (packet->getType() == KAD_Data) {
		getKadObserver().registerPacket(*packet);
		getStatistics().registerMessage(nodePkt.get());
	}
		
	sentDataPackets++;
	forwardDataPacket(std::move(packet), std::move(nodePkt));
}


void R2Kademlia::sendData(const OverlayKey& finaldestkey) {
	EV_TRACE << "sendData to " << finaldestkey.shortStr() << endl;

	// find closest contact toward final destination NodeID
	auto contacts = routingTable.getClosestContacts(finaldestkey, findNodeNumber, true);
	//ASSERT(contacts.size() > 0);
	if (contacts.size() > 0)
	{
		const KadContact& contact = chooseNextHop(contacts, finaldestkey);
		ASSERT(contact.isValid());
		ASSERT(!contact.getKey().isUnspecified());
		if (contact.getKey() == myOverlayKey)
		{
			EV_ERROR << "I'm the next hop for the destination. Will not send anything." << endl;
			return;
		}
		auto packet = makeDataPacket<KadDataPacket>(finaldestkey, &contact, "KadDataPacket");
		EV_TRACE << "sending KadDataPacket id=" << packet->getId() << " via " << contact.getKey().shortStr() << " to " << finaldestkey.shortStr()
			 << " 1st PID=" << packet->getFirstPathID() << " 2nd PID=" << packet->getSecondPathID() << endl;
		// note that makeDataPacket sets DestPathID accordingly!
		sendPacket(std::move(packet));
	}
	else
	{ // no contacts to choose from (usually only possible for isolated nodes)
		EV_WARN << "no contacts found, cannot send KadDataPacket" << endl;
	}
}


void R2Kademlia::onMissingNeighbor(KadDataPacket_p packet, const OverlayKey& missingNeighborNodeID) {
//  	getStatistics().dropMessage(pkt, undefined_node);
	EV_WARN << "onMissingNeighbor(data) called - packet next hop should be ";
	EV_WARN << "NID: " << missingNeighborNodeID.shortStr();
	if (packet->getFirstPathID() != PathID::UndefinedPathID) {
		//EV_WARN << " PID: " << packet->getFirstPathID() << endl;
	}
	else
		EV_WARN << endl;

	if (packet->getSrcNodeID() == myOverlayKey)
	{
		EV_DEBUG << "I'm the source, so not returning any message" << endl;
		return;
	}
	else {
		notifySender(KAD_SegmentFailure, std::move(packet), missingNeighborNodeID);
	}
}


void R2Kademlia::countReceived(KadDataPacket& packet) {
	receivedDataPackets++;
	getKadObserver().registerPacketDelivered(packet);
	getStatistics().deliveredMessage(
		check_and_cast<cMessage *>(packet.getControlInfo()),
		getHopLimitDefault()-packet.getHopLimit()
	);
}


// drops packet and counts dropped packet
// controlInfo needs to be set properly
void R2Kademlia::countDropped(KadDataPacket& packet, const OverlayKey& target, uint8_t reason) {
	droppedDataPackets++;
	getKadObserver().registerPacketDropped(packet, myOverlayKey, target, reason);
	EV_TRACE << "+DROPPED_DATA+ reason:" << (unsigned int) reason << endl;
	nodeid_t nid = undefined_node;
	if (getKadObserver().exists(target))
		nid = getKadObserver().getNodeId(target);
	getStatistics().dropMessage(
		check_and_cast<cMessage *>(packet.getControlInfo()), nid
	);
}


void R2Kademlia::notifySender(uint8_t errorType, KadDataPacket_p origpacket, const OverlayKey& failedNextHopNodeID) {

	EV_TRACE << "notifySender(): to " << origpacket->getPathSrcNodeID().shortStr() << endl;
	// find closest contact toward last overlay source NodeID
	auto contacts = routingTable.getClosestContacts(origpacket->getPathSrcNodeID(), findNodeNumber, true);
	//ASSERT(contacts.size() > 0);
	if (contacts.size() > 0)
	{
		const KadContact& contact = chooseNextHop(contacts, origpacket->getPathSrcNodeID());
		ASSERT(contact.isValid());
		ASSERT(!contact.getKey().isUnspecified());
		if (contact.getKey() == myOverlayKey)
		{
			EV_ERROR << "I'm the next hop for the destination. Will not send anything." << endl;
			return;
		}
		// generate error message to inform the source overlay hop about the broken link
		KadError_p packet = std::make_unique<KadError>();
		packet->setErrorType(errorType);
		packet->setFailedNextHopNodeID(failedNextHopNodeID);
		packet->setOrigDstNodeID(origpacket->getDstNodeID());
		packet->setName("KadErrorInd");
		packet->setDstNodeID(origpacket->getPathSrcNodeID());
		packet->setSource(myOverlayKey);
		packet->setHopLimit(hopLimit);
		packet->appendRoute(contact);
		packet->setStateSeqNo(getLSSeqNo());
		packet->setSrcNodeDegree(neighborTable.size());

		EV_TRACE << "Sending error packet " << (int) errorType  << " back to " << origpacket->getPathSrcNodeID().shortStr()
			 << " via " << contact.getKey().shortStr() << endl;
		sendPacket(std::move(packet));
	}
}

} // end namespace

