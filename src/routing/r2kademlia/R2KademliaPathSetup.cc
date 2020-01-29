#include "R2Kademlia.h"
//#include "KadRoutingTable.h"
//#include "FwdTable.h"

/*********************************************************
 * This file contains PathSetup related code from the 
 * R2Kademlia main class
 *********************************************************/

namespace routingsim {
// for std::bind with callback functions
namespace ph = std::placeholders;

// Path Setup Request/Response
void R2Kademlia::sendPathSetup(KadContact_p contact) {
	EV_TRACE << "sending PathSetupReq from " << myOverlayKey.shortStr() << " to " << contact->getKey().shortStr() << endl;
	sentPathSetup++;
	sendRpcCall(
		makePacket<KadPathSetupReq>(contact, "KadPathSetupReq"),
		std::bind(&R2Kademlia::handlePathSetupResponse, this, ph::_1, ph::_2),
		std::bind(&R2Kademlia::handlePathSetupResponse, this, ph::_1, ph::_2),
		std::bind(&R2Kademlia::handleRPCTimeOut, this, ph::_1, ph::_2));
}

void R2Kademlia::handlePacket(std::unique_ptr<KadPathSetupReq> packet) {
	EV_TRACE << "received PathSetupReq from " << packet->getSource().shortStr() << " to " << packet->getTarget().shortStr()
		 << " sending back answer" << endl;
	receivedPathSetup++;
	sendPacket(makeAnswer<KadPathSetupRsp>(*packet));
}


bool R2Kademlia::handlePathSetupResponse(KadPacket_p answer, RPCState& rpcState) {
	if (rpcState.status != RPC_SUCCESS) return true;
	ASSERT(answer->getSource() == rpcState.original_packet->getTarget());
	std::unique_ptr<KadPathSetupRsp> pathSetupRsp = safe_packet_cast<KadPathSetupRsp>(answer);

	// must check for modified path in Response, because it may have taken a shortcut
	KadContact contact(pathSetupRsp->getSource(), pathSetupRsp->reversePath(pathSetupRsp->getIntermediatePath()));
	EV_TRACE << "Received PathSetupRsp, source path: <";
	for (auto& k : pathSetupRsp->getSrcPath())
		EV_TRACE << ' ' << k.shortStr();
	EV_TRACE << " >" << endl;

	fwdTablePathID.addEntry(&contact, true);

	return true;
}


// send a Path Setup Request
void R2Kademlia::schedulePathSetup(const OverlayKey& contactkey)
{
	KadContact_p rt_contact= routingTable.getContact(contactkey);
	if (rt_contact and rt_contact->isInvalid())
	{
		// do not schedule for path setup if contact not 
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
		auto rt_contact= routingTable.getContact(contactkey);
		if (rt_contact) { // so contact still exists
			// contact may have been invalidated meanwhile
			if (rt_contact->isValid())
			{
				// send PathSetupReq to this contact
				sendPathSetup(rt_contact);
			}
		}
		pathSetupQueue.pop_front();
	}

	// next call of this procedure will be scheduled on demand
	pathSetupScheduled= false;
}

} // end namespace
