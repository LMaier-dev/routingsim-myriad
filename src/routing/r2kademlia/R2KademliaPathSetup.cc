#include "R2Kademlia.h"
//#include "KadRoutingTable.h"
//#include "FwdTable.h"

/*********************************************************
 * This file contains PathSetup and PathTeardown related code from the 
 * R2Kademlia main class
 *********************************************************/

namespace routingsim {
// for std::bind with callback functions
namespace ph = std::placeholders;

// Path Setup Request/Response
void R2Kademlia::sendPathSetup(KadContact_p contact) {
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


void R2Kademlia::handlePacket(std::unique_ptr<KadPathSetupReq> packet) {
	EV_TRACE << "received PathSetupReq from " << packet->getSource().shortStr() << " to " << packet->getTarget().shortStr()
		 << " sending back answer" << endl;
	receivedPathSetup++;
	sendPacket(makeAnswerKadPathSetupRsp(*packet));
}


bool R2Kademlia::handlePathSetupResponse(KadPacket_p answer, RPCState& rpcState) {
	if (rpcState.status != RPC_SUCCESS) return true;

	std::unique_ptr<KadPathSetupRsp> pathSetupRsp = safe_packet_cast<KadPathSetupRsp>(answer);

	// original intended path for setup should be set in answer
	ASSERT(pathSetupRsp->getFwdSourcePathArraySize()>0);

	kad_path setuppath;
	OverlayKey target(pathSetupRsp->getFwdSourcePath(pathSetupRsp->getFwdSourcePathArraySize()-1));
	// note that FwdSourcePath includes the source and the target node
	if (pathSetupRsp->getFwdSourcePathArraySize() > 2) {
		setuppath.resize(pathSetupRsp->getFwdSourcePathArraySize()-2);
		for (unsigned int i= 1; i<pathSetupRsp->getFwdSourcePathArraySize()-1; i++)
			setuppath[i-1]= pathSetupRsp->getFwdSourcePath(i);

	}
	KadContact tmpcontact(target, setuppath);

	EV_TRACE << "Received PathSetupRsp, source path: " << setuppath << " to " << target.shortStr() << endl;

	// meanwhile the contact may have been replaced or deleted etc.
	// also the pathID may have been updated for the contact in the meantime
	// otherwise actually install the path
	if (auto contactp= routingTable.getContact(tmpcontact.getKey()); contactp != nullptr) {
		// check if contact has been changed meanwhile
		// e.g., maybe without path setup as precalc, so this action would override the correct better path...
		if (contactp->getStateSeqNo() < pathSetupRsp->getContactSeqNo() or
		    (contactp->getStateSeqNo() == pathSetupRsp->getContactSeqNo() and
		     tmpcontact.getLastSeen() < pathSetupRsp->getContactLastSeen())) {
			if (auto pidentry= fwdTablePathID.NIDPIDTable.find(tmpcontact.getKey()); pidentry != fwdTablePathID.NIDPIDTable.end()) {
				// contact to PID mapping exists already, then call modEntry()
				fwdTablePathID.modEntry(&tmpcontact, false);
			}
			else {
				if (auto incomingPID= fwdTablePathID.addEntry(&tmpcontact, false, false); incomingPID != PathID::UndefinedPathID) {
					fwdTablePathID.setNIDtoPIDmapping(tmpcontact.getKey(),incomingPID);
				}
			}
		}
		else
			EV_WARN << "PathSetupRsp: ignoring, because Last Seen of current contact is more recent:" << tmpcontact.getLastSeen() << endl;
	}
	else
		EV_TRACE << "PathSetupRsp: ignoring, because contact " << tmpcontact.getKey().shortStr() << " does not exist anymore" << endl;

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

	EV_TRACE << "PathSetup scheduled for contact " << contactkey.shortStr() << endl;

	// do not insert the same contact key twice
	if (std::find(pathSetupQueue.begin(), pathSetupQueue.end(), contactkey) == pathSetupQueue.end())
		pathSetupQueue.push_back(contactkey);

	if (not pathSetupScheduled)
	{
		addTimer(pathSetupWaitTime, 0, std::bind(&R2Kademlia::sendScheduledPathSetups, this));
		pathSetupScheduled= true;
	}
}


void R2Kademlia::sendScheduledPathSetups() {
	// traverse list of scheduled contact keys for path setup
	// recheck status and set to rediscover
	EV_TRACE << "sendScheduledPathSetups() called, Queued: " << pathSetupQueue.size() << endl;
	
	while (!pathSetupQueue.empty())
	{
		auto& contactkey= pathSetupQueue.front();
		// check for current status of scheduled contact
		if (auto rt_contact= routingTable.getContact(contactkey); rt_contact != nullptr) { // so contact still exists
			// note: contact may have been invalidated or changed meanwhile
			if (rt_contact->isValid() and rt_contact->getPathLength() > discoveryRadius)
			{
				// send PathSetupReq to this contact unless it falls under the limit
				sendPathSetup(rt_contact);
			}
		}
		pathSetupQueue.pop_front();
	}

	// next call of this procedure will be scheduled on demand
	pathSetupScheduled= false;
}


void R2Kademlia::handlePacket(std::unique_ptr<KadPathTeardownReq> packet) {
	EV_TRACE << "received PathTeardownReq from " << packet->getSource().shortStr() << " to " << packet->getTarget().shortStr()
		 << " sending back answer" << endl;
	receivedPathTeardown++;
	sendPacket(makeAnswer<KadPathTeardownRsp>(*packet));
}


std::unique_ptr<KadPathSetupRsp>
R2Kademlia::makeAnswerKadPathSetupRsp(const KadPathSetupReq& original)
{
	auto responsepacket= makeAnswer<KadPathSetupRsp>(original);
	// copy source path from KadPathSetupReq
	responsepacket->setFwdSourcePathArraySize(original.getRouteArraySize());
	for (unsigned int i= 0; i < original.getRouteArraySize(); i++)
		responsepacket->setFwdSourcePath(i, original.getRoute(i));
	// just reflect the time stamp
	responsepacket->setContactLastSeen(original.getContactLastSeen());
	responsepacket->setContactSeqNo(original.getContactSeqNo());
	return responsepacket;
}


} // end namespace
