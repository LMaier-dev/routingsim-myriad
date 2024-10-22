#include "KIRA.h"
//#include "KadRoutingTable.h"
//#include "FwdTable.h"
#include "PathIDUtils.h"

/*********************************************************
 * This file contains PathSetup and PathTeardown related code from the 
 * R2Kademlia main class
 *********************************************************/

namespace routingsim {
// for std::bind with callback functions
namespace ph = std::placeholders;

// Path Setup Request/Response
void R2Kademlia::sendPathSetup(KadContact_p contact) {
	if ( contact->getPathLength() > 2*nodeVicinityGraph::vicinityDiscoveryRadius ) { // p >= 5 (6 hops as destination is not included)
		// send a path setup request packet for installing PIDs in intermediate nodes for this route
		// note that this will usually stop three hops before the destination node
		// we only need to start for paths that have 6 hops in total as then
		// there is only the 3rd hop that needs to install an NID->PID mapping 
		EV_TRACE << "sending PathSetupReq from " << myOverlayKey.shortStr() << " to " << contact->getKey().shortStr() << endl;
		sentPathSetup++;

		std::unique_ptr<KadPathSetupReq> packet(makePacket<KadPathSetupReq>(contact, "KadPathSetupReq"));
		packet->setContactLastSeen(contact->getLastSeen());
		packet->setContactSeqNo(contact->getStateSeqNo());
		sendRpcCall(std::move(packet),
			    std::bind(&R2Kademlia::handlePathSetupResponse, this, ph::_1, ph::_2),
			    std::bind(&R2Kademlia::handlePathSetupResponse, this, ph::_1, ph::_2),
			    std::bind(&R2Kademlia::handleRPCTimeOut, this, ph::_1, ph::_2));
	}
	else {
		EV_WARN << "PathSetupReq scheduled, but not sent, because path too short: " << *contact << endl;
	}
}


// Path Teardown Request
void R2Kademlia::sendPathTeardown(KadContact_p contact) {
	EV_TRACE << "sending PathTeardownReq from " << myOverlayKey.shortStr() << " to " << contact->getKey().shortStr() << endl;
	sentPathTeardown++;
	std::unique_ptr<KadPathTeardownReq> packet(makePacket<KadPathTeardownReq>(contact, "KadPathTeardownReq"));

	sendPacket(std::move(packet));
}


// this will never be called, because a node two hops before should have answered the PathSetupReq due to precomputed PathIDs
void R2Kademlia::handlePacket(std::unique_ptr<KadPathSetupReq> packet) {
	EV_TRACE << "received PathSetupReq from " << packet->getSource().shortStr() << " to " << packet->getTarget().shortStr()
		 << " sending back answer" << endl;
	receivedPathSetup++;
	sendPacket(makeAnswerKadPathSetupRsp(*packet));
}


// this installs a NID/PID mapping and PID entry
bool R2Kademlia::handlePathSetupResponse(KadPacket_p answer, RPCState& rpcState) {
	if (rpcState.status != RPC_SUCCESS) return true;

	std::unique_ptr<KadPathSetupRsp> pathSetupRsp = safe_packet_cast<KadPathSetupRsp>(answer);

	// originally complete path to contact should be set in answer
	ASSERT(pathSetupRsp->getFwdSourcePathArraySize() > 2*(nodeVicinityGraph::vicinityDiscoveryRadius+1));

	// construct original contact path that was used as source path for PathSetupReq
	kad_path originalpath;
	// the setup path ends earlier: three hops before the target!
	kad_path setuppath;
	kad_path lastsegmentpath(nodeVicinityGraph::vicinityDiscoveryRadius+2);
	lastsegmentpath.at(0)=pathSetupRsp->getFwdSourcePath(pathSetupRsp->getFwdSourcePathArraySize()-(nodeVicinityGraph::vicinityDiscoveryRadius+2));
	OverlayKey target(pathSetupRsp->getFwdSourcePath(pathSetupRsp->getFwdSourcePathArraySize()-1));
	// note that FwdSourcePath includes the source and the target node
	// pathSetupRsp->getFwdSourcePathArraySize()+1 is number hops and that must be at least 6
	if (pathSetupRsp->getFwdSourcePathArraySize() > 2*(nodeVicinityGraph::vicinityDiscoveryRadius+1)) {
		// original contact path excludes source and target node
		originalpath.resize(pathSetupRsp->getFwdSourcePathArraySize()-2);
		// shorten setuppath to what is expected (by 2 hops) (also remove source and target nodes)
		setuppath.resize(pathSetupRsp->getFwdSourcePathArraySize()-1-(nodeVicinityGraph::vicinityDiscoveryRadius+1));
		for (unsigned int i= 1; i<pathSetupRsp->getFwdSourcePathArraySize()-1; i++) {
			originalpath[i-1]= pathSetupRsp->getFwdSourcePath(i);
			if (i < pathSetupRsp->getFwdSourcePathArraySize()-(nodeVicinityGraph::vicinityDiscoveryRadius+1))
				setuppath[i-1]= pathSetupRsp->getFwdSourcePath(i);
			else
				lastsegmentpath.at(i-pathSetupRsp->getFwdSourcePathArraySize()+nodeVicinityGraph::vicinityDiscoveryRadius+2)= pathSetupRsp->getFwdSourcePath(i);
		}
		lastsegmentpath.at(nodeVicinityGraph::vicinityDiscoveryRadius+1)= target;
	}
	else
	{
		EV_ERROR << "Received PathSetupRsp for a too short path (# of hops: " << pathSetupRsp->getFwdSourcePathArraySize() << ")" << endl;
		ASSERT(pathSetupRsp->getFwdSourcePathArraySize() > 2*(nodeVicinityGraph::vicinityDiscoveryRadius+1));
		return false;
	}

	EV_TRACE << "Received PathSetupRsp, setup path: " << setuppath << " to " << target.shortStr()
		 << " original path: " << originalpath << " last segment:" << lastsegmentpath << endl;

	KadContact tmpcontact(myOverlayKey, target, std::move(originalpath));

	bool addthispath(false);
	// meanwhile the contact may have been replaced or deleted etc.
	// also the pathID may have been updated for the contact in the meantime
	// otherwise actually install the path
	if (auto contactp= const_cast<KadContact*>(routingTable.getContact(tmpcontact.getKey())); contactp != nullptr) {
		// check if contact has been changed meanwhile
		// e.g., maybe without path setup as precalc, so this action would override the correct better path...
		if (contactp->getStateSeqNo() < pathSetupRsp->getContactSeqNo() or
		    (contactp->getStateSeqNo() == pathSetupRsp->getContactSeqNo() and
		     tmpcontact.getLastSeen() < pathSetupRsp->getContactLastSeen())) {
                    if (contactp->isValid() and contactp->getPathLength() < tmpcontact.getPathLength()) {
                        // do nothing we have a valid shorter path in our contact
			// next path setup should have been scheduled then, or PID was precomputed
                    } else {
			addthispath= true;
                    }
		}
		else {
			// Last Seen of current contact is more recent
			// check whether path has changed, then we should better ignore this PathSetup as obsolete and ignore
			if (contactp->getPathLength() != tmpcontact.getPath().size()+nodeVicinityGraph::vicinityDiscoveryRadius or contactp->getPath() != tmpcontact.getPath()) {
				EV_WARN << "PathSetupRsp: ignoring, because Last Seen of current contact is more recent and path has changed meanwhile:" << tmpcontact.getLastSeen() << endl;
				// TODO: initiate Teardown of the obsolete path
			}
			else {
				addthispath= true;
			}
		}
	}
	else {
		EV_TRACE << "PathSetupRsp: ignoring, because contact " << tmpcontact.getKey().shortStr() << " does not exist anymore" << endl;
		// TODO: initiate Teardown of the obsolete path? or has this been triggered already by contact replacement/removal?
	}

	if (addthispath) {
		if (auto pidentry= fwdTablePathID.NIDPIDTable.find(target); pidentry != fwdTablePathID.NIDPIDTable.end()) {
			// contact to PID mapping exists already, so call modEntry()
			fwdTablePathID.modEntry(target, tmpcontact.getPath(), false);
		}
		else {
			// add new PID and mapping
			// for addLocalEntry() the path needs to include the target node
			if (lastsegmentpath.empty())
				setuppath.push_back(tmpcontact.getKey());
			if (auto [added, incomingPID]= fwdTablePathID.addLocalEntry(setuppath, false); incomingPID != PathID::UndefinedPathID) {
				if (not lastsegmentpath.empty())
					fwdTablePathID.setNIDtoPIDmapping(target, incomingPID, getPathID(lastsegmentpath));
				else
					fwdTablePathID.setNIDtoPIDmapping(target, incomingPID);
			}
		}
	}

	return true;
}


// send a Path Setup Request
void R2Kademlia::schedulePathSetup(const OverlayKey& contactkey)
{
	KadContact_p rt_contact= routingTable.getContact(contactkey);
	if (rt_contact and rt_contact->isInvalid())
	{
		// do not schedule for path setup if contact not valid
		EV_WARN << "Path Setup not scheduled for contact " << contactkey.shortStr()
			<< " because it is in invalid state" << endl;
		return;
	}

	// do not insert the same contact key twice
	if (std::find(pathSetupQueue.begin(), pathSetupQueue.end(), contactkey) == pathSetupQueue.end()) {
		EV_TRACE << "PathSetup scheduled for contact " << contactkey.shortStr() << endl;

		pathSetupQueue.push_back(contactkey);
	}
	else
		EV_TRACE << "PathSetup already scheduled for contact " << contactkey.shortStr() << endl;

	if (not pathSetupScheduled)
	{
	  addOneShotTimer(uniform(pathSetupWaitTime/2,3*pathSetupWaitTime/2), std::bind(&R2Kademlia::sendScheduledPathSetups, this));
		pathSetupScheduled= true;
	}
}


bool R2Kademlia::sendScheduledPathSetups() {
	// traverse list of scheduled contact keys for path setup
	// recheck status and set to rediscover
	EV_TRACE << "sendScheduledPathSetups() called, Queued: " << pathSetupQueue.size() << endl;
	
	while (!pathSetupQueue.empty())
	{
		auto& contactkey= pathSetupQueue.front();
		// check for current status of scheduled contact
		if (auto rt_contact= routingTable.getContact(contactkey); rt_contact != nullptr) { // so contact still exists
			// note: contact may have been invalidated or changed meanwhile
			if (rt_contact->isValid() and rt_contact->getPathLength() > 2*nodeVicinityGraph::vicinityDiscoveryRadius)
			{
				// send PathSetupReq to this contact unless it falls under the limit
				sendPathSetup(rt_contact);
			}
		}
		pathSetupQueue.pop_front();
	}

	// next call of this procedure will be scheduled on demand
	pathSetupScheduled= false;
	return false;
}


void R2Kademlia::handlePacket(std::unique_ptr<KadPathTeardownReq> packet) {
	EV_TRACE << "received PathTeardownReq from " << packet->getSource().shortStr() << " to " << packet->getTarget().shortStr() << endl;

	receivedPathTeardown++;
	// no need to send back an answer
	//sendPacket(makeAnswer<KadPathTeardownRsp>(*packet));
}


std::unique_ptr<KadPathSetupRsp>
R2Kademlia::makeAnswerKadPathSetupRsp(const KadPathSetupReq& original)
{
	auto responsepacket= makeAnswer<KadPathSetupRsp>(original);
	// copy complete(!) source path from KadPathSetupReq
	// the PathSetupReq usually returns much earlier and does not reach the target
	responsepacket->setFwdSourcePathArraySize(original.getRouteArraySize());
	for (unsigned int i= 0; i < original.getRouteArraySize(); i++)
		responsepacket->setFwdSourcePath(i, original.getRoute(i));

	// this echoes the contact state at the originator
	// so that the originator can check whether this Rsp is obsolete when received
	responsepacket->setContactLastSeen(original.getContactLastSeen());
	responsepacket->setContactSeqNo(original.getContactSeqNo());
	return responsepacket;
}


} // end namespace
