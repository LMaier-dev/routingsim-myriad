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
#include "KadObserver.h"

namespace routingsim {
// for std::bind with callback functions
namespace ph = std::placeholders;



template<typename T>
std::unique_ptr<T> R2Kademlia::makeDataPacket(const OverlayKey& dstNodeID, const KadContact* viacontact, const char *name) {
	auto packet = std::make_unique<T>();
	packet->setName(name);
	packet->setType(KAD_Data);
	packet->setSrcNodeID(myOverlayKey);
	packet->setDstNodeID(dstNodeID);
	packet->setHopLimit(hopLimit);

	// when creating a packet we set everything so that we can forward the packet to the next overlay hop
	auto pidentry= fwdTablePathID.NIDPIDTable.find(viacontact->getKey());
	if ( pidentry != fwdTablePathID.NIDPIDTable.end() )
	{ // pidentry.second is incomingPathID
		// this will be used to lookup the outgoing PID in forwardDataPacket()
		packet->setDestPathID(pidentry->second);
	}
	else
	{
		EV_ERROR << "makeDataPacket() - no PathID found for via destination " << viacontact->getKey().shortStr() << endl;
	}
	return packet;
}



void R2Kademlia::doDataProbe(short timerID) {
	if (neighborTable.size() == 0) return; // we are isolated
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
	
	sendData(*probeKey);
}


void R2Kademlia::handleDataPacket(NodePkt_p pkt) {
	EV_TRACE << "handleDataPacket() - ";
	
	KadDataPacket_p packet = std::unique_ptr<KadDataPacket>(check_and_cast<KadDataPacket*>(pkt->decapsulate()));
	
	EV_TRACE << "received data packet from "
	   << packet->getSrcNodeID().shortStr() << " to "
	   << packet->getDstNodeID().shortStr() << " HL:" << packet->getHopLimit() << endl;

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
// @returns true in case of successful forwarding/transmission to next hop or dropped packet
bool R2Kademlia::forwardDataPacketToNextHop(KadDataPacket_p& packet, NodePkt_p& nodePkt)
{
	bool packetorigin(packet->getSrcNodeID() == myOverlayKey);
	// lookup outgoing PathID (next hop) based on incoming PathID
	auto fwdentry= fwdTablePathID.PIDFwdTable.find(packet->getDestPathID());
	if (fwdentry != fwdTablePathID.PIDFwdTable.end())
	{ // entry found
		FwdTableEntry& nh= fwdentry->second;
		if (nh.isValid() or nh.isObsolete() or nh.isInvalid())
		{
			if (nh.isInvalid()) {
				EV_ERROR << "invalid PathID entry hit " << packet->getDestPathID() << endl;
			}

			if (nh.isObsolete()) { // obsolete, but maybe still good enough to forward
				EV_WARN << "obsolete PathID entry hit: " << packet->getDestPathID() << endl;
			}

			if (nh.fwdAction == FwdTableEntry::Action::fwdNextPathID)
			{ // set new outgoing PathID
				packet->setDestPathID(nh.outgoingPathID);
				EV_TRACE << "New outgoing PathID: " << nh.outgoingPathID << endl;
			}
			else
			if (nh.fwdAction == FwdTableEntry::Action::fwdNextKadId)
			{ // next hop will be destination NID
				packet->setDestPathID(PathID::UndefinedPathID);
				EV_TRACE << "Forward to Next KadID: " << nh.nextHopNode.shortStr() << endl;
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
			// Trying to send to a next hop that is not a physical next hop
			// That will not fly...

			// in case the link to the next hop has failed, send back segment failure
			// to inform the sender about the problem
			if (auto contact_p= routingTable.getContact(nh.nextHopNode);
			    contact_p != nullptr and contact_p->isPhysicalNeighbor()) {
				packet->setControlInfo(nodePkt.release());
				countDropped(*packet, packet->getDstNodeID(), missingNeighbor);
				// send back segment failure
				onMissingNeighbor(std::move(packet), std::move(nodePkt));
			}
			else { // else: wrong source route
				EV_ERROR << "forwardDataPacketToNextHop: wrong next hop"
					 << nh.nextHopNode.shortStr() << endl;

				packet->setControlInfo(nodePkt.release());
				countDropped(*packet, packet->getDstNodeID(), wrongNextHop);
			}

			return true;
		}

	}

	// false means failure
	return false;
}


void R2Kademlia::forwardDataPacket(KadDataPacket_p packet, NodePkt_p nodePkt) {

	ASSERT(packet->getType() == KAD_Data);
	ASSERT(packet->getControlInfo() == nullptr);
        ASSERT(packet->hasRemainingHops());

	auto& finaldest(packet->getDstNodeID());

	if (packet->hasRemainingHops() == false)
	{ // Loop detected
		countDropped(*packet, finaldest, hopLimitExceeded);
		returnToSender(KAD_HopLimitExceeded, std::move(packet));
		return;
	}

	//  own originated packets will possess the right (incoming) DestPathID, see makeDataPacket()
	PathID incomingPathID(packet->getDestPathID());
	
	// incomingPathID is only undefined for last hop packets	
	if (incomingPathID == PathID::UndefinedPathID)
	{ // next hop (intermediate) destination ID of this path (final destination may be elsewhere)
		// increase overlayHop count
		packet->countOverlayHop();

		// should be just my own key or direct neighbor!
		if (finaldest == myOverlayKey)
		{ // destined to myself
			EV_TRACE << "next hop is me (after " << getHopLimitDefault()-packet->getHopLimit() << " hops / " << packet->getOverlayHops() <<  " overlayhops )" << endl;
			// successfully delivered! consume packet
			packet->setControlInfo(nodePkt.release());

			countReceived(*packet);
			return;
		}
		else
		{ // we must find next overlay hop according to DStNodeID

			auto contacts = routingTable.getClosestContacts(finaldest, findNodeNumber, true);
			if (contacts.size() > 0)
			{
				const KadContact& contact = chooseNextHop(contacts, finaldest);
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

				EV_TRACE << "next overlay hop is " << contact.getKey().shortStr()
					 << " (" << contact.getPathLength()+1 << " hops)" << endl;

				// now try to find a PathID to the next overlay hop
				auto pidentry= fwdTablePathID.NIDPIDTable.find(contact.getKey());
				if ( pidentry != fwdTablePathID.NIDPIDTable.end() )
				{ // pidentry.second is incomingPathID
					// this will be used to lookup the outgoing PID in forwardDataPacket()
					packet->setDestPathID(pidentry->second);
					EV_TRACE << "new PID for dest " << contact.getKey().shortStr() << ": " << pidentry->second << endl;

					if (not forwardDataPacketToNextHop(packet, nodePkt))
					{
						EV_WARN << "PID based forwarding failed while trying to forward to next hop. Dropping packet" << endl;
						countDropped(*packet, finaldest, missingPID);
						returnToSender(KAD_PathIDUnknown, std::move(packet));

						return;
					}
					else // forwardDataPacketToNextHop had success or dropped
						return;
				}
				else
				{
					EV_ERROR << "No PID found for contact in Routing Table: Dropping packet" << endl;
					// this is an error condition
					packet->setControlInfo(nodePkt.release());
					countDropped(*packet, finaldest, missingPID);
					return;
				}
			}
			else
			{
				// Trying to send to a next hop that is not known
				// That will not fly...
				EV_ERROR << "forwardDataPacket: next hop for destination " << finaldest.shortStr() << " cannot be found (0 contacts)" << endl;

				// forwarding failed
				packet->setControlInfo(nodePkt.release());
				countDropped(*packet, finaldest, routeFailureDeadEnd);
				return;
			}
		} // end else (must find next overlay hop)

		// we should never get here
		ASSERT(false);
		return;

	} // if incoming PathID is undefined

	packet->setDestPathID(incomingPathID);
	if (not forwardDataPacketToNextHop(packet, nodePkt))
	{ // incoming PathID not found
		// may be the case if entry already deleted/overwritten, or
		// has never existed due to shortcut route heard from other nodes
		EV_WARN << "forwardDataPacket: incoming PathID " << incomingPathID << " not found in FwdPIDTable" << endl;
		inconsistentPathHit++;

		// 2nd chance in case for inconsistent paths (e.g., due to overhearing/shortening):
		// lookup path ID to destination ID
		// this would avoid routing via the overlay (which should be the last resort)
		auto pidentry= fwdTablePathID.NIDPIDTable.find(finaldest);
		if ( pidentry != fwdTablePathID.NIDPIDTable.end() )
		{ // pidentry.second is incomingPathID
			// this will be used to lookup the outgoing PID in forwardDataPacket()
			packet->setDestPathID(pidentry->second);
			EV_TRACE << "new PID for dest " << finaldest << ": " << pidentry->second << endl;

			if (not forwardDataPacketToNextHop(packet, nodePkt))
			{
				EV_WARN << "2nd chance forwarding failed while trying to forward. Dropping packet" << endl;
				countDropped(*packet, finaldest, secondChanceFailed);
				//returnToSender(KAD_PathIDUnknown, std::move(packet));

				return;
			}
			else
			{ // 2nd chance had success or dropped
				return;
			}
		}
		else
		{
			EV_WARN << "No PID found for contact: 2nd chance forwarding failed. Dropping packet" << endl;
		
			// if we are here 2nd chance failed
			packet->setControlInfo(nodePkt.release());
			countDropped(*packet, finaldest, secondChanceFailed);
			returnToSender(KAD_PathIDUnknown, std::move(packet));
		}
	}
	// else: Packet was sent off or dropped by function
}


void R2Kademlia::sendPacket(KadDataPacket_p packet) {
	//ASSERT(packet->getSource() == myOverlayKey);
	ASSERT(packet->getControlInfo() == nullptr);
	auto nodePkt = std::make_unique<NodePkt>(packet->getName());
	nodePkt->setType(KAD_DATA_PACKET_TYPE);
	if (packet->getType() == KAD_Data) {
		getKadObserver().registerPacket(*packet);
		getStatistics().registerMessage(nodePkt.get());
	}
		
	sentDataPackets++;
	forwardDataPacket(std::move(packet), std::move(nodePkt));
}


void R2Kademlia::sendData(const OverlayKey& finaldestkey) {
	EV_TRACE << "sendData to " << finaldestkey.shortStr() << endl;
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
		EV_TRACE << "sending KadDataPacket id=" << packet->getId() << " via " << contact.getKey().shortStr() << " to " << finaldestkey.shortStr() <<  endl;
		// note that makeDataPacket sets DestPathID accordingly!
		sendPacket(std::move(packet));
	}
	else
	{ // no contacts to choose from (usually only possible for isolated nodes)
		EV_WARN << "no contacts found, cannot send KadDataPacket" << endl;
	}
}


void R2Kademlia::onMissingNeighbor(KadDataPacket_p packet, NodePkt_p pkt) {
//  	getStatistics().dropMessage(pkt, undefined_node);
	EV_WARN << "onMissingNeighbor(data) called - packet next hop should be ";
	if (packet->getDestPathID() != PathID::UndefinedPathID) {
		EV_WARN << "PID: " << packet->getDestPathID() << endl;
	}
	else {
		EV_WARN << "NID: " << packet->getDstNodeID() << endl;
	}

	if (packet->getSrcNodeID() == myOverlayKey)
	{
		EV_DEBUG << "I'm the source, so not returning any message" << endl;
		return;
	}
	else {
		returnToSender(KAD_SegmentFailure, std::move(packet));
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


void R2Kademlia::returnToSender(uint8_t errorType, KadDataPacket_p packet) {
	// KadError_p answer = makeAnswer<KadError>(*packet);
	// answer->setErrorType(errorType);
	
	// EV_TRACE << "returnToSender() - return path:";
	// for (const auto& c : answer->getSrcPath())
	// 	EV_TRACE << ' ' << c.shortStr();
	// EV_TRACE << endl;		
	// answer->encapsulate(packet.release());
	// if (answer->hasRemainingHops())
	// 	sendPacket(std::move(answer));
	// else // we sent this packet!
	// 	handleAnswer(std::move(answer));
}

} // end namespace

