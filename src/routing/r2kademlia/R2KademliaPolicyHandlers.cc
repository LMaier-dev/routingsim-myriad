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

namespace routingsim {
// for std::bind with callback functions
namespace ph = std::placeholders;


// -------------- kad policy -------------

void R2Kademlia::onMissingNeighbor(KadPacket_p packet, NodePkt_p pkt) {
//  	getStatistics().dropMessage(pkt, undefined_node);
	EV_WARN << "onMissingNeighbor called - packet next hop should be " << packet->getNextHop().shortStr() << endl;
	droppedPackets++;
	// do not generate Error packets for Error or UpdateRoute packets
	switch (packet->getType()) {
		case KAD_Error:
		case KAD_UpdateRouteReq:
		case KAD_QueryRouteRsp:
		case KAD_FindNodeRsp:	
			EV << "not generating error packet for packet type " << packet->getName() << "(" << (unsigned int) packet->getType() << ")" << endl;
			break;
		default:
		{
			if (packet->getSource() == myOverlayKey)
			{
				EV_DEBUG << "I'm the source, so not returning any message" << endl;
				return;
			}
			else
				returnToSender(KAD_SegmentFailure, std::move(packet));
		}
	} // end switch
}




void R2Kademlia::onErrorPacket(KadError& error) {

	EV_INFO << "received ERROR packet (" << (unsigned int) error.getErrorType() << ") ";
	switch (error.getErrorType()) {
		case KAD_NoError:
			EV_INFO << "No Error";
			break;
		case KAD_HopLimitExceeded:
			EV_INFO << "HopLimitExceeded";
			break;
		case KAD_SegmentFailure:
			EV_INFO << "SegmentFailure";
			break;
		case KAD_NodeUnreachable:
			EV_INFO << "NodeUnreachable";
			break;
		case KAD_PathIDUnknown:
			EV_INFO << "PathIDUnkown";
			break;
		case KAD_RouteFailureDeadEnd:
			EV_INFO << "RouteFailureDeadEnd";
			break;
	}
	EV_INFO	<< endl;
	
	ASSERT(error.getEncapsulatedPacket() != nullptr);
	KadPacket* original = check_and_cast<KadPacket *>(error.getEncapsulatedPacket());
	ASSERT(original->getNonce() == error.getNonce());
	ASSERT(original->getSource() == myOverlayKey);
	switch (error.getErrorType()) {

		// the following errors indicate a problem with the route to the contact
		case KAD_SegmentFailure:
		case KAD_NodeUnreachable:
		{ // Invalidate contact as the error information is current

			// find out the failed link
		        auto reportingNode(error.getSource());
		        const auto originalPath(original->getSrcPath());
			// use progress in original path to find the correct link
			// just finding the reporting node in the source path isn't safe due to cycles in forwarding path!
		        short unsigned int index(original->getProgress());
			// the following should be true, otherwise we are getting
			// an error from some off path node
			ASSERT(originalPath[index] == reportingNode and index+1U < originalPath.size());
			// the next hop is the one that failed
			if (index+1U < originalPath.size()) {
				auto failedNextHop = originalPath[index+1];
				link_t failedlink(reportingNode, failedNextHop);
				EV_INFO << "Check for invalid contacts because of failed link " << failedlink << " in error packet" << endl;
				// this segment or node error is current and should invalidate all affected contacts
				// a rediscovery will be triggered automatically if necessary
				routingTable.invalidateContacts(failedlink, (simTime()-error.getCreationTime()).inUnit(SIMTIME_MS));
			}
			else
				EV_ERROR << "onErrorPacket: failed link collapsed to node " << reportingNode << " path:" << originalPath << endl;
		};
		break;

		case KAD_PathIDUnknown:
		{ // KAD_PathIDUnknown: we should try to resetup the path
			auto affectedContact= routingTable.getContact(original->getTarget());
			if (affectedContact and affectedContact->isValid() and affectedContact->getPathLength() > discoveryRadius)
			{
				schedulePathSetup(affectedContact->getKey());
			}
		};
		break;

		default:
			// currently there are no more error types to handle
			break;
	} // end switch
}


void R2Kademlia::onBucketEmpty(KadBucket& bucket) {
	EV << "bucket empty" << endl;
}


// this is called before deletion of the contact
void R2Kademlia::onContactRemoval(KadContact_p contact)
{
	//EV_TRACE << "onContactRemoval(" << ((contact != nullptr) ? contact->getKey().shortStr() : "nullptr") << ") called" << endl;
	if (contact == nullptr)
		return;

	// delete from Forwarding table PID
	// TODO: this can be scheduled for happening later
	if (usePathIDs)
		fwdTablePathID.delEntry(contact);

	// TODO: the following is probably not useful
	// (node may still be present in vicinity, but was evicted from bucket)
	// remove from vicinity node set (in case of later rendezvous again)
	if (contact->getPathLength() < discoveryRadius and contact->getState() == KadContact::State::dead)
		vicinity.removeNode(contact->getKey());
}


// this is called after invalidation of the contact
void R2Kademlia::onContactInvalidation(KadContact_p contact)
{
	if (contact == nullptr)
		return;

	if (usePathIDs)
		fwdTablePathID.invalidateEntry(contact);
}

void R2Kademlia::onNewContact(KadContact_p contact) {

	EV_TRACE << "onNewContact(" << contact->getKey().shortStr() << ") called, SeqNo #" << contact->getStateSeqNo() << endl;
	//ASSERT( getKadObserver().checkSeqNo(contact->getKey(),contact->getStateSeqNo()) == true );

	// physical neighbor, may have been heard of its existence by some third node!
	// avoid further actions until we hear a HELLO
	if (contact->getPathLength() == 0) 
	{ 	if (not neighborTable.isNeighbor(contact->getKey()))
		{
			EV_DEBUG << "onNewContact: will ignore supposed physical neighbor not being in Neighbor table yet" << endl;
			return;
		}
	}

	// ask for physical neighbors if new contact is within discovery radius
	// contact->getPathLength() excludes the destination, so path length = 0 
	// holds for PhyNeighbors (their Physical Neighbors are already discovered by PNDiscReq/Rsp)
	// getPathLength()+1 corresponds to the number of hops from me to the contact.
	// Thus, contact->getPathLength()+1 < discoveryRadius is correct, because we ask
	// those contacts about their PNs which is even one hop further away
	if (contact->getPathLength() > 0 and contact->getPathLength()+1 < discoveryRadius)
	{
		// sends a QueryRouteReq requesting their 1 hop neighborhood
		sendGetPhysicalNeighbors(contact);
	}

	if (contact->isValid())
	{
		// request info about overlay neighbors near my node ID from nodes in our deepest bucket
		if (routingTable.isInLastBucket(contact->getKey()))
		{
			//EV_TRACE << "New contact is in deepest bucket, will ask for contacts close to me" << endl;
			sendGetOverlayNeighbors(contact);
		}

		if (usePathIDs) {
			if (contact->getPathLength() <= discoveryRadius) {
				// it is required to add all PIDs to nodes within the discovery radius
				// (and without signaling one can extend this by one hop)
				// since the contact path excludes the contact itself the above condition includes nodes <= discoveryRadius+1
				if (auto incomingPID= fwdTablePathID.addEntry(contact, false, true); incomingPID != PathID::UndefinedPathID) {
					// PID was added, it can directly be used, because no path setup necessary within discovery
					fwdTablePathID.setNIDtoPIDmapping(contact->getKey(), incomingPID);
					vicinityPathIDCompletion(*contact);
				}
			}
			else
			if (contact->getPathLength() > discoveryRadius)
			{  // send a path setup request packet for installing PIDs in intermediate nodes for this route
				// only do this for paths longer than (discovery radius+1) (e.g., for dr=3 -> contacts 5 hops away) hops:
				// the next node precalculated already all path IDs within its own discovery radius and
				// we are just one hop away, so only required setup for paths longer than (discovery radius+1)
				// note that the contact->getPath() excludes the contact itself
				EV_TRACE << "Triggering path setup to " << contact->getKey().shortStr() << ": " << contact->getPath() << endl;
				schedulePathSetup(contact->getKey());
			}
		}
	}

	scheduleUpdate(*contact, KAD_Announce);
}


void R2Kademlia::onImprovement(const KadContact& contact) {
	// path got improved for this contact -> potentially send UpdateRouteReq
	EV_TRACE << "onImprovement(" << contact.getKey().shortStr() << ") called, SeqNo #" << contact.getStateSeqNo() << ' ' << contact.getPath() << endl;

	//ASSERT( getKadObserver().checkSeqNo(contact.getKey(),contact.getStateSeqNo()) == true );

	if (usePathIDs and contact.isValid()) {

		// no need to setup routes within discovery radius + 1 distance
		// (note/remember: getPathLength() does not include the last hop)
		if ( contact.getPathLength() > discoveryRadius ) {
			// send a path setup request packet for installing PIDs in intermediate nodes for this route
			//EV_TRACE << "Triggering path setup to " << contact.getKey().shortStr() << endl;
			// modentry() will be called by handlePathSetupResponse
			schedulePathSetup(contact.getKey());
		}
		else {
			// for paths within discovery radius: update the PID entry
			fwdTablePathID.modEntry(&contact, false);
		}
	}

	// TODO: wait grace period
	if ( neighborTable.isNeighbor(contact.getKey()) )
	{ // this is true after re-enabling a physical neighbor
		if (not contact.isPhysicalNeighbor())
			EV_WARN << "reenabling physical contact is not flagged as being neighbor" << endl;

		scheduleUpdate(contact, KAD_Announce);
	}
	else
		scheduleUpdate(contact, KAD_Change);
}


void R2Kademlia::recheckPhysicalNeighbors(const KadContact& contact) {
	// simply recheck the physical Neighbors
	EV_TRACE << "recheckPhysicalNeighbors triggered for " << contact.getKey().shortStr() << endl;
	sendGetPhysicalNeighbors(&contact);
}


void R2Kademlia::reSyncPN(const KadContact& contact) {
	// simply trigger a PNDiscReq to update any sequence number
	EV_TRACE << "reSyncPN triggered for " << contact.getKey().shortStr() << endl;
	schedulePNDiscReq(contact.getKey());
}



void R2Kademlia::onContactCheck(const KadContact& contact) {
	EV_TRACE << "contact check triggered for " << contact.getKey().shortStr() << endl;
	sendProbe(&contact);
}

} // end namespace
