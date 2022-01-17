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

// callback function type
using cbfunc_t = std::function<void(short timerId)>;

// add a timer that calls callback every duration period (randomized)
// if duration is 0, this is a one shot timer, consider using addOneShotTimer() instead
short R2Kademlia::addTimer(simtime_t initialOffset, simtime_t duration, std::function<void(short timerId)> callback) {
	ASSERT(timerDuration.size() == timerMsg.size());
	ASSERT(timerDuration.size() < (size_type) std::numeric_limits<short int>::max());
	ASSERT(duration >= 0);
	if (duration > 0)
		timerDuration.push_back(duration);

	short int ntimerId(duration == 0 ? -1 : timerDuration.size()-1);
	cMessage *msg = new cMessage("TimerMsg", ntimerId);
	msg->setContextPointer( reinterpret_cast<void *>( new auto(callback) ) );

	if (duration > 0)
		timerMsg.push_back(msg);

	simtime_t scatter = uniform(0, duration);
	scheduleAt(simTime() + initialOffset - duration/2 + scatter, msg);
	//EV_TRACE << "scheduled timer message " << msg << ( (duration != 0) ? " with duration " : " after ")
	//	 <<   ((duration != 0) ? duration : initialOffset) <<  endl;
	return ntimerId;
}


void R2Kademlia::stopTimer(short timerId)
{
	if (timerId > 0 and (size_type) timerId < timerMsg.size()) {
		cMessage* timermsg= timerMsg[timerId];
		timerMsg[timerId]= nullptr;
		// cancel potentially scheduled message (nothing happens if it was not)
		cancelEvent(timermsg);
		// remove callback function
		cbfunc_t* cbfunc_p(static_cast<cbfunc_t*>(timermsg->getContextPointer()));
		delete(cbfunc_p);
		timermsg->setContextPointer(nullptr);
		delete timermsg;
	}
}


void R2Kademlia::resetTimer(short timerId, simtime_t duration)
{
	if (timerId > 0 and (size_type) timerId < timerMsg.size()) {
		cMessage* timermsg= timerMsg[timerId];
		// save callback function pointer
		cbfunc_t* cbfunc_p(static_cast<cbfunc_t*>(timermsg->getContextPointer()));
		// cancel potentially scheduled message (nothing happens if it was not)
		timerMsg[timerId]= nullptr;
		cancelEvent(timermsg);
		if (duration > 0) {
			timerDuration[timerId]= duration;
			cMessage *msg = new cMessage("TimerMsg", timerId);
			msg->setContextPointer( reinterpret_cast<void *>( cbfunc_p ) );
			scheduleAt(simTime() + uniform(duration, duration/2*3), msg);
		}
	}
}


// this handles timer self messages, if the timer triggers
void R2Kademlia::onSelfMessage(cMessage_p msg) {
	std::unique_ptr<RPCtimeoutMsg> rpctimeout(dynamic_cast<RPCtimeoutMsg *>(msg.get()));
	if (rpctimeout)
	{
		msg.release(); 		// no double deletion

		//EV_TRACE << "RPC Timeout Message for nonce " << rpctimeout->nonce << endl;

		handleTimeOut(rpctimeout->nonce);

		// timeout message will be dismissed
		return;
	}

	// handle timer
	// timerId == -1 is a one shot timer
	short timerId = msg->getKind();

	if (cbfunc_t* timerCallback(static_cast<cbfunc_t *>(msg->getContextPointer()));
	    timerCallback != nullptr) {
		// now call the callback function
		(*timerCallback)(timerId);

		// delete the callback function for one shot messages
		if (timerId == -1) {
			delete timerCallback;
			msg->setContextPointer(nullptr);
		}
	}

	// reschedule timer message if duration > 0, otherwise it was a one shot timer
	if (timerId  >= 0)
	{
		ASSERT((size_t)timerId < timerDuration.size());
		ASSERT(timerMsg[timerId] == msg.get());
		if (timerDuration[timerId] > 0)
		{
			simtime_t time = uniform(timerDuration[timerId]/2, timerDuration[timerId]/2*3);
			scheduleAt(simTime() + time, msg.release());
		}
	}
}



// periodically send a Hello packet to keep state seq no in synch
void R2Kademlia::doSendHello(short timerId) {
	if (neighborTable.size() == 0)
		return; // node is isolated

	ASSERT(timerId >= 0);
	static const simtime_t maxDuration(30); // check every 30 seconds

	// exponential backoff double duration until we reached maxDuration
	if (timerDuration[timerId] < maxDuration)
	{ // double waiting time for next call until maxDuration is reached
		timerDuration[timerId] = std::min(2 * timerDuration[timerId], maxDuration);
	}

	auto hellopkt(makeHelloPacket());
	sentVolume+= neighborTable.size() * hellopkt->getByteLength();
	sendToNeighbors(std::move(hellopkt));
	sentPackets+= neighborTable.size();
}


// Contact Probing: probe for closest (overlay/ID-wise) neighbors, i.e., those in the deepest buckets
// contact probing uses FindNodeReq (i.e., the overlay routing strategy) to check reachability of
// the contact, this may also detect changes in routes or even better routes
void R2Kademlia::doProbeNeighbors(short timerId) {
	if (neighborTable.size() == 0)
		return; // node is isolated

	if ( probeNeighborQueue.empty() )
	{
		// fill queue with current neighbors
		auto contacts= routingTable.getClosestContacts(myOverlayKey, routingTable.getSplitSize(), false);
		for (auto contactp : contacts)
			probeNeighborQueue.push_back(contactp->getKey());
	}

	bool probed(false);
	while (probeNeighborQueue.size() > 0 and !probed) {
		// check for reachability of neighbors
		// do not probe in case contact not found or too young
		if (auto contactp= routingTable.getContact(probeNeighborQueue.front());
		    contactp != nullptr and contactp->getAge() > R2Kademlia::PathInfoAgeThreshold) {
			EV_TRACE << "probing for neighbor: " << probeNeighborQueue.front() << endl;
			sendFindNode(probeNeighborQueue.front(), true);
			probed= true;
		}
		probeNeighborQueue.pop_front();
	} // end while
}



// check for contact validity
// contact probing uses FindNodeReq (i.e., the overlay routing strategy) to check reachability of
// the contact, this may also detect changes in routes or even better routes
void R2Kademlia::doProbeContacts(short timerId) {
	if (neighborTable.size() == 0)
		return; // node is isolated

	if ( probeContactQueue.empty() )
	{
		// fill queue with current oldest neighbors (2 for each bucket)
		routingTable.getOldestValidContacts(probeContactQueue);
	}

	// check for reachability of neighbors
	bool probed(false);
	while (probeContactQueue.size() > 0 and !probed)
	{
		// do not probe in case contact not found or too young
		if (auto contactp= routingTable.getContact(probeContactQueue.front());
		    contactp != nullptr and contactp->getAge() > R2Kademlia::PathInfoAgeThreshold) {
			EV_TRACE << "probing for contact: " << probeContactQueue.front() << endl;
			sendFindNode(probeContactQueue.front(), true);
			probed= true;
		}
		else
			EV_TRACE << "skipped contact: " << probeContactQueue.front() << endl; 

		probeContactQueue.pop_front();
	};
}


void R2Kademlia::doSendExplorePathPacket(short timerId) {
	if (neighborTable.size() == 0)
		return; // node is isolated

	// check for alternative paths to contacts
	if (auto contactp= routingTable.getRandomContact();
	    contactp != nullptr) {
		EV_TRACE << "explore path for contact: " << *contactp << endl;

		if (auto contacts = routingTable.getClosestContacts(contactp->getKey(), 2*findNodeNumber, true); not contacts.empty()) {

			// choose random via contact
			unsigned int random_index = intuniform(0,contacts.size()-1);
			auto viacontactp(contacts[random_index]);
			EV_TRACE << "chose via contact #" << random_index << " of " << contacts.size() << endl;
			sendFindNodeVia(contactp->getKey(), viacontactp, true, false);
		}
		else
			EV_WARN << "no via contact found?!" << endl;
	}
	else
		EV_WARN << "no contact found for path exploration" << endl;

}


void R2Kademlia::doProbe(short timerId) {
	if (neighborTable.size() == 0) return; // we are isolated
	OverlayKey probeKey;
	bool exact = false;
	uint32_t index = 0;
	switch (probeStrategy) {
		case 0:
			return;
		case 1:
			probeKey = OverlayKey::random(getRNG(0));
			break;
		case 2:
			probeKey = getKadObserver().getRandomNodeKey();
			break;
		case 3:
			index = std::min<size_type>(intuniform(1, 1 + routingTable.getDepth()*2),
					myOverlayKey.getLength());
			probeKey = myOverlayKey;
			probeKey[myOverlayKey.getLength() - index] ^= 1;
			break;
		case 4:
			index = routingTable.getDepth();
			probeKey = myOverlayKey.randomSuffix(myOverlayKey.getLength() - index, getRNG(0));
			break;
		case 5:
			index = std::min<size_type>(intuniform(1, 1 + routingTable.getDepth()*2),
					myOverlayKey.getLength());
			probeKey = myOverlayKey.randomSuffix(myOverlayKey.getLength() - index, getRNG(0));
			break;
		case 6:
			index = intuniform(1, 1 + routingTable.getDepth());
			probeKey = myOverlayKey.randomSuffix(myOverlayKey.getLength() - index, getRNG(0));
			break;
		default:
			throw cRuntimeError("unknown probe strategy %u, values from 0 to 6 are allowed", probeStrategy);
	}
	if (probeKey == myOverlayKey) probeKey[0] ^= 1;
	sendFindNode(probeKey, exact);
}


void R2Kademlia::doProbeSelf(short timerId) {
	ASSERT(timerId >= 0);
	static const simtime_t maxDuration(300); // check every 5 minutes

	// exponential backoff double duration until we reached maxDuration
	if (timerDuration[timerId] < maxDuration)
	{ // double waiting time for next call until maxDuration is reached
		timerDuration[timerId] = std::min(2 * timerDuration[timerId], maxDuration);
	}

	if (neighborTable.size() == 0) return; // we are isolated

	sendFindNode(myOverlayKey, false);
}


void R2Kademlia::doReachabilityTest(short timerID) {
	if (neighborTable.size() == 0) return; // we are isolated
	const OverlayKey* key(nullptr);
	unsigned int maxtries= 10;
	bool nodefound= false;
	do {
		// during periods of failed links we want to preferably test reachability for affected nodes
		if (failureFocussedProbing and getNetworkManager()->getFailedNodes().size() > 0) {
			// select random nodes out of set of failed nodes
			auto nodeit= getNetworkManager()->getFailedNodes().cbegin();
			for (unsigned int index= intuniform(0, getNetworkManager()->getFailedNodes().size()-1); index > 0; index--)
				nodeit++;

			if (nodeit != getNetworkManager()->getFailedNodes().end()) {
				nodeid_t nodecandidate(*nodeit);
				// if node is active and not isolated
				if (getNetworkManager()->getTopology().isActive(nodecandidate) == true and
				    getNetworkManager()->getTopology().getNodeDegree(nodecandidate) > 0) {
					if (getNetworkManager()->getFailedNodes().size() > 1) { // not the only choice
						// try this failed node
						key = &getKadObserver().getNodeKey(nodecandidate);
						nodefound= true;
					}
				}
			}
		}

		if (not nodefound) { // try different non affected node as alternative
			key = &getKadObserver().getRandomNodeKey();
		}

		// check whether destination node is actually reachable
		if (key != nullptr) {
			if (getKadObserver().getPhysicalDistance(myOverlayKey, *key) == 0) {
				EV_TRACE << "Node " << key->shortStr() << " not reachable" << endl;
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

	if (key != nullptr and nodefound == true) {
		EV_TRACE << "Reachability Test to " << key->shortStr() << endl;
		sendFindNode(*key, true, true);
	}
	else
		EV_WARN << "Reachability Test cannot be performed (key==nullptr)" << endl;
}

} // end namespace
