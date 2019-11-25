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

#include "FwdTable.h"

namespace routingsim {
// for std::bind with callback functions
namespace ph = std::placeholders;



template<typename T>
std::unique_ptr<T> R2Kademlia::makeDataPacket(const KadContact* contact, const char *name) {
	auto packet = std::make_unique<T>();
	packet->setName(name);
	packet->setType(KAD_Data);
	packet->setSrcNodeID(myOverlayKey);
	packet->setDstNodeID(contact->getKey());
	packet->setHopLimit(hopLimit);

	// when creating a packet we set everything so that we can forward the packet
	auto pidentry= fwdTablePathID.NIDPIDTable.find(contact->getKey());
	if ( pidentry != fwdTablePathID.NIDPIDTable.end() )
	{ // pidentry.second is incomingPathID
		// this will be used to lookup the outgoing PID in forwardDataPacket()
		packet->setDestPathID(pidentry->second);
	}
	else
	{
		EV_ERROR << "makeDataPacket() - no PathID found for destination " << contact->getKey().shortStr() << endl;
	}
	return packet;
}



void R2Kademlia::doDataProbe() {
	if (neighborTable.size() == 0) return; // we are isolated
	const OverlayKey* probeKey(nullptr);
	do {
		probeKey = &getKadObserver().getRandomNodeKey();
	} while(*probeKey == myOverlayKey);
	
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
	// lookup outgoing PathID (next hop) based on incoming PathID
	auto fwdentry= fwdTablePathID.PIDFwdTable.find(packet->getDestPathID());
	if (fwdentry != fwdTablePathID.PIDFwdTable.end())
	{
		FwdTableEntry& nh= fwdentry->second;
		if (nh.fwdState == FwdTableEntry::State::valid)
		{ // 
			if (nh.fwdAction == FwdTableEntry::Action::fwdNextPathID)
			{ // set new outgoing PathID
				packet->setDestPathID(nh.outgoingPathID);
				EV_TRACE << "New outgoing PathID: " << nh.outgoingPathID << endl;
			}
			else
			if (nh.fwdAction == FwdTableEntry::Action::fwdNextKadId)
			{ // next hop will be destination NID
				packet->setDestPathID(PathID::UndefinedPathID);
				EV_TRACE << "Forward to Next KadID: " << packet->getDstNodeID().shortStr() << endl;
			}
			else
			if (nh.fwdAction == FwdTableEntry::Action::fwdlocal)
			{
				EV_TRACE << "Forward action: local delivery" << endl;
			}
		}
		else
		if (nh.fwdState == FwdTableEntry::State::obsolete)
		{ // obsolete, but maybe still good enough to forward
			EV_WARN << "obsolete PathID entry hit" << endl;
		}
		else
		if (nh.fwdState == FwdTableEntry::State::invalid)
		{
			EV_ERROR << "invalid PathID entry hit" << endl;
		}
		else
			EV_ERROR << "hit forwarding table entry with undefined state" << endl;

	       	nodeid_t nid = neighborTable.getNodeId(nh.nextHopNode);
		if (nid != undefined_node) {
			packet->advanceProgress(); // decrease hop limit
			nodePkt->encapsulate(packet.release());
			sendToNode(nid, std::move(nodePkt));
			forwardedDataPackets++;
			EV_TRACE << "send to next hop nid=" << nid << endl;
			// true means success
			return true;
		}
		else {
			// Trying to send to a next hop that is not a physical next hop
			// That will not fly...
			EV_ERROR << "forwardDataPacket: wrong next hop" << endl;
			//TODO onMissingNeighbor(std::move(packet), std::move(nodePkt));
			packet->setControlInfo(nodePkt.release());
			countDropped(*packet, packet->getDstNodeID(), wrongNextHop);

			return true;
		}

	}

	// false means failure
	return false;
}


void R2Kademlia::forwardDataPacket(KadDataPacket_p packet, NodePkt_p nodePkt) {

	ASSERT(packet->getType() == KAD_Data);
	ASSERT(packet->hasRemainingHops());
	ASSERT(packet->getControlInfo() == nullptr);

	//  own originated packets will possess the right (incoming) DestPathID, see makeDataPacket()
	PathID incomingPathID(packet->getDestPathID());
	
	// incomingPathID is only undefined for last hop packets	
	if (incomingPathID == PathID::UndefinedPathID)
	{ // next hop is destination ID
		// should be just my own key or direct neighbor!
		if (packet->getDstNodeID() == myOverlayKey)
		{ // destined to myself
			EV_TRACE << "next hop is me (after " << getHopLimitDefault()-packet->getHopLimit() << " hops)" << endl;
			// successfully delivered! consume packet
			packet->setControlInfo(nodePkt.release());

			countReceived(*packet);
			return;
		}
		else
		{ // if forwarding incoming packet it is destined to next hop that should be PN
			EV_TRACE << "next hop is " << packet->getDstNodeID().shortStr() << " (PN)" << endl;

			nodeid_t nid= neighborTable.getNodeId(packet->getDstNodeID());
			if (nid != undefined_node)
			{ // just forward
				packet->advanceProgress(); // reduce hop limit
				nodePkt->encapsulate(packet.release());
				sendToNode(nid, std::move(nodePkt));
				forwardedDataPackets++;
			}
			else
			{
				// Trying to send to a next hop that is not a physical next hop
				// That will not fly...
				EV_ERROR << "forwardDataPacket: next hop is not a physical node" << endl;
				//TODO onMissingNeighbor(std::move(packet), std::move(nodePkt));

				// forwarding failed
				packet->setControlInfo(nodePkt.release());
				countDropped(*packet, packet->getDstNodeID(), missingNeighbor);
			}

			return;
		}

	}

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
		auto pidentry= fwdTablePathID.NIDPIDTable.find(packet->getDstNodeID());
		if ( pidentry != fwdTablePathID.NIDPIDTable.end() )
		{ // pidentry.second is incomingPathID
			// this will be used to lookup the outgoing PID in forwardDataPacket()
			packet->setDestPathID(pidentry->second);
			EV_TRACE << "new PID for dest " << packet->getDstNodeID() << ": " << pidentry->second << endl;

			if (not forwardDataPacketToNextHop(packet, nodePkt))
			{
				//TODO send error message back
				EV_WARN << "2nd chance forwarding failed while trying to forward. TODO send error message back. Dropping packet" << endl;
				countDropped(*packet, packet->getDstNodeID(), secondChanceFailed);
				return;
			}
			else
			{ // 2nd chance had success or dropped
				return;
			}
		}
		else
		{
			EV_WARN << "No PID found for contact: 2nd chance forwarding failed. TODO send error message back. Dropping packet" << endl;
		
			// if we are here 2nd chance failed
			packet->setControlInfo(nodePkt.release());
			countDropped(*packet, packet->getDstNodeID(), secondChanceFailed);
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


void R2Kademlia::sendData(const OverlayKey& key) {
	EV_TRACE << "sendData to " << key.shortStr() << endl;
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
		auto packet = makeDataPacket<KadDataPacket>(&contact, "KadDataPacket");
		EV_TRACE << "sending KadDataPacket id=" << packet->getId() << endl;
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
	EV_WARN << "onMissingNeighbor(data) called - packet next hop should be PID:" << packet->getDestPathID() << endl;
	droppedDataPackets++;
	// do not generate Error packets for Error or UpdateRoute packets
	switch (packet->getType()) {
		case KAD_Error:
		case KAD_UpdateRouteReq:
		case KAD_QueryRouteRsp:
		case KAD_FindNodeRsp:	
			EV << "not generating error packet for packet type " << packet->getName() << "(" << packet->getType() << ")" << endl;
			break;
		default:
		{
			if (packet->getSrcNodeID() == myOverlayKey)
			{
				EV_DEBUG << "I'm the source, so not returning any message" << endl;
				return;
			}
			else
				returnToSender(KAD_SegmentFailure, std::move(packet));
		}
	} // end switch
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
	getKadObserver().registerPacketDropped(packet, myOverlayKey, target);
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

