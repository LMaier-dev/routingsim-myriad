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
#include "PathIDUtils.h"

namespace routingsim {

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
			EV_INFO << "PathIDUnknown";
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
		{ // KAD_PathIDUnknown: we should try to setup the path
			auto affectedContact= routingTable.getContact(original->getTarget());
			if (affectedContact and affectedContact->isValid() and affectedContact->getPathLength() > nodeVicinityGraph::vicinityDiscoveryRadius)
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

	// delete from Forwarding table PID (not for vicinity nodes)
	// TODO: this can be scheduled for happpening later
	if (usePathIDs and contact->getPathLength() > 2*nodeVicinityGraph::vicinityDiscoveryRadius)
	{ // this condition must be aligned with the sendPathSetup
		// check for pending/scheduled PathSetup first
		if (std::find(pathSetupQueue.cbegin(),pathSetupQueue.cend(),contact->getKey()) == pathSetupQueue.cend())
			sendPathTeardown(contact);

		fwdTablePathID.delEntry(contact);
	}

	// TODO: the following is probably not useful
	// (node may still be present in vicinity, but was evicted from bucket)
	// remove from vicinity node set (in case of later rendezvous again)
	if (contact->getPathLength() < nodeVicinityGraph::vicinityDiscoveryRadius and contact->getState() == KadContact::State::dead)
		vicinity.removeNode(contact->getKey());
}


// this is called after invalidation of the contact, i.e., path is known to be broken
void R2Kademlia::onContactInvalidation(KadContact_p contact)
{
	if (contact == nullptr)
		return;

	if (usePathIDs)
		fwdTablePathID.invalidateEntry(contact);
}



void R2Kademlia::onNewContact(KadContact_p contact) {

	EV_TRACE << "onNewContact(" << contact->getKey().shortStr() << ") called, SeqNo #" << contact->getStateSeqNo() << " (" << contact->getNodeDegree() << ')' << endl;
	//ASSERT( getKadObserver().checkSeqNo(contact->getKey(),contact->getStateSeqNo()) == true );

	// physical neighbor, may have been heard of its existence by some third node!
	// avoid further actions until we hear a HELLO or get a PNDiscReq
	if (contact->getPathLength() == 0) 
	{
		if (not neighborTable.isNeighbor(contact->getKey()))
		{
			EV_DEBUG << "onNewContact: will ignore supposed physical neighbor not being in Neighbor table yet" << endl;
			return;
		}
		else { // actually a new neighbor -> increase sequence number
			incLSSeqNo();
		}
	}

	// ask for physical neighbors if new contact is within discovery radius
	// contact->getPathLength() excludes the destination, so path length = 0 
	// holds for PhyNeighbors (their Physical Neighbors are already discovered by PNDiscReq/Rsp)
	// getPathLength()+1 corresponds to the number of hops from me to the contact.
	// Thus, contact->getPathLength()+1 < discoveryRadius is correct, because we ask
	// those contacts about their PNs which is even one hop further away
	if (contact->getPathLength() > 0)
	{
		if (contact->getPathLength()+1 < discoveryRadius) {
			// sends a QueryRouteReq requesting their 1 hop neighborhood
			// sendGetPhysicalNeighbors() adds contact to vicinity to keep track of already queried nodes in vicinity
			scheduleGetPNReq(*contact); // was sendGetPhysicalNeighbors(contact);
		}
		else // maybe just on the radius, check to add contact for vicinity
		if (contact->getPathLength()+1 == nodeVicinityGraph::vicinityDiscoveryRadius) {
			vicinity.addNode(contact->getPath().back(), // parent, path not empty, see precondition of outer if
					 contact->getKey(), // new child
					 contact->getStateSeqNo(),
					 contact->getPathLength()+1);
		}
	}

	if (contact->isValid())
	{
		// request info about overlay neighbors near my node ID from nodes in our deepest bucket
		if (routingTable.isInLastBucket(contact->getKey()))
		{
			//EV_TRACE << "New contact is in deepest bucket, will ask for contacts close to me" << endl;
			scheduleGetONReq(*contact);// was sendGetOverlayNeighbors(contact);
		}

		if (usePathIDs) {
			// assume this is node (x) and the contact is (hi), vicinityDiscoveryRadius==2:
			// (x)-(h1)-(h2)-(h3)-(h4)-(h5)-(h6)-(h7)-(h8)
			// n: hop distance p: path length of contact
			// n: 1 p: 0 (h1) -> PID out: NID h1, , precalculated -> see above
			// n: 2 p: 1 (h2) -> PID out: PID H(h1,h2), , precalculated -> see above
			// n: 3 p: 2 (h3) -> PID out: PID H(h1,h2,h3) (no path setup required)
			// n: 4 p: 3 (h4) -> PID out: PID H(h1,h2,h3) (no path setup required), 2ndPID H(h3,h4), no local PID installation required
			// n: 5 p: 4 (h5) -> PID out: PID H(h1,h2,h3) (no path setup required), 2ndPID H(h3,h4,h5), no local PID installation required
			// n: 6 p: 5 (h6) -> PID out: PID H(h1,h2,h3) (path setup required, h3 needs to install inPID(h3,h4,h5,h6)->outPID(h4,h5,h6))
			// n: 7 p: 6 (h7) -> PID out: PID H(h1,h2,h3,h4) (path setup required in h1, h4 needs to install inPID(h4,h5,h6,h7)->outPID(h5,h6,h7), setup ends at h4
			// n: 8 p: 7 (h8) -> PID out: PID H(h1,h2,h3,h4,h5) (path setup required in h1+h2, h5 needs to install inPID(h5,h6,h7,h8)->outPID(h6,h7,h8) ), setup ends at h5
			// and so on

			// note: the contact path excludes the contact itself and path length = hops + 1
			// paths with less than 6 hops do not need a path setup, so PIDs can be directly installed here
			if (contact->getPathLength() <= 2*nodeVicinityGraph::vicinityDiscoveryRadius) {
				// for paths up to 3 hops
				// for paths with 4 or 5 hops: stop 2 hops before end, the PID of the 2nd segment has been precomputed there
				kad_path newpath(contact->getPath());
				newpath.push_back(contact->getKey());

				auto [firstSegmentPID,secondSegmentPID]=  getSegmentPathIDs(myOverlayKey,newpath);
				if (newpath.size() > 2*nodeVicinityGraph::vicinityDiscoveryRadius)
					newpath.resize(nodeVicinityGraph::vicinityDiscoveryRadius+2);
				else
				if (newpath.size() == 2*nodeVicinityGraph::vicinityDiscoveryRadius)
					newpath.resize(nodeVicinityGraph::vicinityDiscoveryRadius+1);

				if (auto [added,incomingPID]= fwdTablePathID.addLocalEntry(newpath, contact->getPathLength()<nodeVicinityGraph::vicinityDiscoveryRadius); incomingPID != PathID::UndefinedPathID) {
					ASSERT(incomingPID == firstSegmentPID);
					// PID was added, it can directly be used, because no path setup necessary within 5 hops
					EV_TRACE << "added local NID " << contact->getKey() << " and PID entry " << incomingPID << " 2ndPID: " << secondSegmentPID <<  endl;
					fwdTablePathID.setNIDtoPIDmapping(contact->getKey(), incomingPID, secondSegmentPID);
					if (contact->getPathLength() < nodeVicinityGraph::vicinityDiscoveryRadius)
						vicinityPathIDCompletion(*contact);
				}
			}
			else {
				// send a path setup request packet for installing PIDs in intermediate nodes for this route

				// only do this for paths longer than 2*(vicinityDiscoveryRadius+1)-1 (e.g., for vdr=2 -> contacts 6 hops away) hops:
				// the next node precalculated already all path IDs within its own discovery radius and
				// we are just one hop away, so only required setup for paths longer than (discovery radius+1)
				// note that the contact->getPath() excludes the contact itself
				EV_TRACE << "Triggering path setup for contact " << contact->getKey().shortStr() << ": " << contact->getPath() << endl;
				schedulePathSetup(contact->getKey());
			}
		} // endif usePathIDs
	}

	scheduleUpdate(*contact, KAD_Announce);
}


void R2Kademlia::onImprovement(const KadContact& contact) {
	// path got improved for this contact -> potentially send UpdateRouteReq
	EV_TRACE << "onImprovement(" << contact.getKey().shortStr() << ") called, SeqNo #" << contact.getStateSeqNo() << ' ' << contact.getPath() << endl;

	//ASSERT( getKadObserver().checkSeqNo(contact.getKey(),contact.getStateSeqNo()) == true );

	if (usePathIDs and contact.isValid()) {

		// no need to setup paths for paths that are shorter than 6 hops
		// (note/remember: getPathLength() does not include the last hop)
		if ( contact.getPathLength() > 2*nodeVicinityGraph::vicinityDiscoveryRadius ) {
			// send a path setup request packet for installing PIDs in intermediate nodes for this route
			//EV_TRACE << "Triggering path setup to " << contact.getKey().shortStr() << endl;
			// modentry() will be called by handlePathSetupResponse
			schedulePathSetup(contact.getKey());
		}
		else {  // 5 hops or less
			
			// for these paths update the PID entry (max to 3rd hop)
			auto newpath(contact.getPath());
			if (contact.getPathLength() <= nodeVicinityGraph::vicinityDiscoveryRadius)
				newpath.push_back(contact.getKey());
			else
				newpath.resize(nodeVicinityGraph::vicinityDiscoveryRadius+1);

			fwdTablePathID.modEntry(contact.getKey(), newpath, false);
		}
	}

	// TODO: wait grace period
	if ( neighborTable.isNeighbor(contact.getKey()) and contact.getPath().empty())
	{ // this is true after re-enabling a physical neighbor
		if (not contact.isPhysicalNeighbor()) {
			EV_WARN << "reenabling physical contact is not flagged as being neighbor" << endl;
		}
		// now increase the sequence number
		incLSSeqNo();
		scheduleUpdate(contact, KAD_Announce);
	}
	else
		scheduleUpdate(contact, KAD_Change);
}


// this is different from onImprovement(): path has maybe the same length but is different (e.g., due to higher seq no)
// no UpdateRouteReq is sent, but possibly PathSetup needs to be initiated or PID needs to change
void R2Kademlia::onPathChange(const KadContact& contact) {
	EV_TRACE << "onPathChange(" << contact.getKey().shortStr() << ") called, SeqNo #" << contact.getStateSeqNo() << ' ' << contact.getPath() << endl;

	//ASSERT( getKadObserver().checkSeqNo(contact.getKey(),contact.getStateSeqNo()) == true );

	if (usePathIDs and contact.isValid()) {

		// no need to setup routes within discovery radius + 1 distance
		// (note/remember: getPathLength() does not include the last hop)
		if ( contact.getPathLength() > 2*nodeVicinityGraph::vicinityDiscoveryRadius ) {
			// send a path setup request packet for installing PIDs in intermediate nodes for this route
			//EV_TRACE << "Triggering path setup to " << contact.getKey().shortStr() << endl;
			// modentry() will be called by handlePathSetupResponse
			schedulePathSetup(contact.getKey());
		}
		else { // 5 hops or less

			// for these paths update the PID entry (max to 3rd hop)
			auto newpath(contact.getPath());
			if (contact.getPathLength() <= nodeVicinityGraph::vicinityDiscoveryRadius)
				newpath.push_back(contact.getKey());
			else
				newpath.resize(nodeVicinityGraph::vicinityDiscoveryRadius+1);
			
			fwdTablePathID.modEntry(contact.getKey(), newpath, false);
		}
	}

}

void R2Kademlia::recheckPhysicalNeighbors(const KadContact& contact) {
	// simply recheck the physical Neighbors
	EV_TRACE << "recheckPhysicalNeighbors triggered for " << contact.getKey().shortStr() << endl;
	scheduleGetPNReq(contact);
}


void R2Kademlia::reSyncPN(const KadContact& contact, stateseqno_t expected_seqno) {
	// simply trigger a PNDiscReq to update any sequence number
	EV_TRACE << "reSyncPN scheduled for " << contact.getKey().shortStr() << " expected seqno: " << expected_seqno << endl;

	schedulePNDiscReq({contact.getKey(), {}, expected_seqno});
}


void R2Kademlia::onPathCheck(const KadContact& contact) {
	EV_TRACE << "path check triggered for " << contact.getKey().shortStr() << endl;
	// check whether the current path is still working properly
	sendPathProbe(contact.getKey());
}

} // end namespace
