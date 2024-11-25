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
#include "BaseTypes.h"
#include "KIRA.h"
#include "KadObserver.h"
#include "cexception.h"
#include "routing/KIRA/KadTypes.h"


namespace routingsim {


// add a timer that calls callback every duration period in average (randomized over [0.5*duration,1.5*duration])
// note: the first timer will fire in the interval now+[initialoffset - duration/2, initialoffset + duration/2]
// therefore initialoffset must be at least duration/2!
// if duration is 0, this is a one shot timer, consider using addOneShotTimer() instead
short
R2Kademlia::addRandomizedTimer(simtime_t initialOffset, simtime_t duration, TimerMsg::cbfunc_t callback) {
	ASSERT(randomTimerDuration.size() == randomTimerMsg.size());
	ASSERT(randomTimerDuration.size() < (size_type) fixedTimerIdBase-1);
	ASSERT(duration >= omnetpp::SimTime::ZERO);

	if (duration/2 > initialOffset)
		throw cRuntimeError("initialOffset %s too small for duration %s",initialOffset.str().c_str(),duration.str().c_str());

	short int ntimerId(duration == omnetpp::SimTime::ZERO ? oneShotTimerId : randomTimerDuration.size());
	auto msg= new TimerMsg(ntimerId, callback);

	if (duration > omnetpp::SimTime::ZERO) {
		randomTimerDuration.push_back(duration);
		randomTimerMsg.push_back(msg);
	}

        simtime_t scatter = uniform(0, duration);
	scheduleAt(simTime() + initialOffset - duration/2 + scatter, msg);
	// EV_TRACE << "scheduled timer message (randomized) " << ntimerId << ( (duration != 0) ? " with duration " : " after ")
	// 	 <<   ((duration != 0) ? duration : initialOffset) <<  endl;
	return ntimerId;
}


// add a timer that calls callback exactly every duration period
// if duration is 0, this is a one shot timer, consider using addOneShotTimer() instead
short R2Kademlia::addFixedTimer(simtime_t initialOffset, simtime_t duration, TimerMsg::cbfunc_t callback) {
	ASSERT(fixedTimerDuration.size() == fixedTimerMsg.size());
	ASSERT(fixedTimerDuration.size() < (size_type) std::numeric_limits<short int>::max()-1-fixedTimerIdBase);
        ASSERT(duration >= omnetpp::SimTime::ZERO);
	// for fixed timers the timerId starts at fixedTimerIdBase
	short int ntimerId((duration == omnetpp::SimTime::ZERO) ? oneShotTimerId :  fixedTimerIdBase+fixedTimerDuration.size());
	auto msg= new TimerMsg(ntimerId, callback);
	if (duration > omnetpp::SimTime::ZERO) {
		fixedTimerDuration.push_back(duration);
                fixedTimerMsg.push_back(msg);
	}

	scheduleAt(simTime() + initialOffset, msg);
        // EV_TRACE << "scheduled timer message (fixed) " << ntimerId << " TimerIdx=" << ntimerId - fixedTimerIdBase
	// 	 << ( (duration != 0) ? " with duration " : " after ")
	//  	 <<   ((duration != 0) ? duration : initialOffset) <<  endl;
	return ntimerId;
}


// delete_timermsg should not be set in case this is called from within onMessage() that is a unique_ptr
void R2Kademlia::stopTimer(short timerId, bool delete_timermsg)
{
	// handle randomized periodic timers
	if (timerId >= 0 and timerId < fixedTimerIdBase and (size_type) timerId < randomTimerMsg.size()) {
		TimerMsg* timermsg(randomTimerMsg[timerId]);
		randomTimerMsg[timerId]= nullptr;
		randomTimerDuration[timerId]= omnetpp::SimTime::ZERO;
		// cancel potentially scheduled message (nothing happens if it was not)
		if (timermsg != nullptr) {
		  cancelEvent(timermsg);
                  if (delete_timermsg)
			  delete timermsg;
		}
	}
	else // handle fixed timers
	if (timerId >= fixedTimerIdBase and timerId < oneShotTimerId) {
		if (size_type timerIdx= timerId-fixedTimerIdBase; timerIdx < fixedTimerMsg.size()) {
			TimerMsg* timermsg(fixedTimerMsg[timerIdx]);
                        fixedTimerMsg[timerIdx] = nullptr;
			fixedTimerDuration[timerIdx] = omnetpp::SimTime::ZERO;
			// cancel potentially scheduled message (nothing happens if it was not)
			if (timermsg != nullptr) {
				cancelEvent(timermsg);
				if (delete_timermsg)
					delete timermsg;
			}
		}
        }
        else {
		throw cRuntimeError("stopTimer(): invalid timerId %d", timerId);
	}
}


// if duration is omnetpp::SimTime::ZERO, timer is stopped
void R2Kademlia::resetTimer(short timerId, simtime_t duration)
{
	//EV_TRACE << "resetTimer() called: timerId=" << timerId << " duration " << duration.str() << endl;
	// periodic timer
	if (timerId >= 0 and timerId < fixedTimerIdBase and (size_type) timerId < randomTimerMsg.size()) {
		TimerMsg* timermsg(randomTimerMsg[timerId]);
		if (timermsg == nullptr)
			return;

		// cancel potentially scheduled message (nothing happens if it was not)
		cancelEvent(timermsg); // calling with nullptr is safe

		if (duration > omnetpp::SimTime::ZERO) { // set new duration and reschedule
			randomTimerDuration[timerId]= duration;
			scheduleAt(simTime() + uniform(duration, duration/2*3), timermsg);
		}
		else {
			randomTimerMsg[timerId]= nullptr;
			randomTimerDuration[timerId]= omnetpp::SimTime::ZERO;
			delete timermsg;
		}
	}
	else  // fixed timer
	if (timerId >= fixedTimerIdBase and timerId < oneShotTimerId)
		if (auto timerIdx= timerId-fixedTimerIdBase; (size_type) timerIdx < fixedTimerMsg.size()) {
		TimerMsg* timermsg(fixedTimerMsg[timerIdx]);

		// cancel potentially scheduled message (nothing happens if it was not)
		cancelEvent(timermsg);
		if (timermsg == nullptr)
			return;

		if (duration > omnetpp::SimTime::ZERO) { // set new duration and reschedule
			fixedTimerDuration[timerIdx]= duration;
			scheduleAt(simTime() + duration, timermsg);
		}
		else {
			fixedTimerMsg[timerIdx]= nullptr;
                        fixedTimerDuration[timerIdx] = omnetpp::SimTime::ZERO;
			delete timermsg;
		}
	}
}


// this handles timer self messages, if the timer triggers
void R2Kademlia::onSelfMessage(cMessage_p msg)
{
	std::unique_ptr<RPCtimeoutMsg> rpctimeout(dynamic_cast<RPCtimeoutMsg *>(msg.get()));
	if (rpctimeout != nullptr)
	{
		// cast to rpctimeout was successful
		(void) msg.release(); // no double deletion, but also don't need to original pointer

		//EV_TRACE << "RPC Timeout Message for nonce " << rpctimeout->nonce << endl;

		handleTimeOut(rpctimeout->nonce);

		// timeout message will be dismissed
		return;
	}

	// handle timer
	// timerId == oneShotTimerId is a one shot timer
	// timerId >= 0 and < fixedTimerIdBase: randomized timer
	// timerId >= fixedTimerIdBase < oneShotTimerId: fixed timer
	short timerId = msg->getKind();

	if (auto timerCallback(static_cast<TimerMsg::cbfunc_t *>(msg->getContextPointer()));
	    timerCallback != nullptr) {
		// now call the callback function
		bool stop_timer= (*timerCallback)(timerId);

		// delete the callback function for one shot messages or if the callback returns true
		if (timerId == oneShotTimerId  or stop_timer) {
			delete timerCallback;
                        msg->setContextPointer(nullptr);
                        if (stop_timer) {
				if (timerId >= 0 and timerId < fixedTimerIdBase and (size_type) timerId < randomTimerMsg.size()) {
					randomTimerMsg[timerId] = nullptr;
					randomTimerDuration[timerId]= omnetpp::SimTime::ZERO;
				}
				else
				if (timerId >= fixedTimerIdBase and timerId < oneShotTimerId) {
					if (size_type timerIdx = timerId - fixedTimerIdBase; timerIdx < fixedTimerMsg.size()) {
						fixedTimerMsg[timerIdx]= nullptr;
						fixedTimerDuration[timerIdx]= omnetpp::SimTime::ZERO;
					}
                                }
                        }
			return;
		}
	}

	// reschedule timer message if duration > 0, otherwise it was a one shot timer
	if (timerId  >= 0 and timerId < fixedTimerIdBase)
	{ // these timers draw their next time period randomly from the interval
	  // [randomTimerDuration[timerId]/2, randomTimerDuration[timerId]/2*3]
		ASSERT((size_t)timerId < randomTimerDuration.size());
		ASSERT(randomTimerMsg.at(timerId) == msg.get());
		if (randomTimerDuration[timerId] > omnetpp::SimTime::ZERO)
		{
			simtime_t time = uniform(randomTimerDuration[timerId]/2, randomTimerDuration[timerId]/2*3);
			scheduleAt(simTime() + time, msg.release());
		}
	}
	else if (timerId >= fixedTimerIdBase and timerId < oneShotTimerId)
	{
		size_type timerIdx = timerId - fixedTimerIdBase;
		if (not (timerIdx < fixedTimerDuration.size()))
			EV_ERROR << "TimerIdx= " << timerIdx << " TimerId=" << timerId << endl;
		ASSERT(timerIdx < fixedTimerDuration.size());
		ASSERT(fixedTimerMsg.at(timerIdx) == msg.get());
		if (fixedTimerDuration[timerIdx] > omnetpp::SimTime::ZERO)
		{
			scheduleAt(simTime() + fixedTimerDuration.at(timerIdx), msg.release());
		}
        }
}



// periodically send a Hello packet to keep state seq no in synch
bool R2Kademlia::doSendHello(short timerId)
{
	ASSERT(timerId >= 0 and timerId < fixedTimerIdBase);
        // existing neighbors are not known when LinkLayerIndications are not
        // used, so in this case we need to send a Hello to all potential underlay neighbors
	if (useLinkLayerIndication and neighborTable.empty())
		return false; // node is isolated

	if (lastPNHelloStateSeqNo.isInvalid() or lastPNHelloStateSeqNo != getLSSeqNo()) {
		// if local state has changed within the last interval, reset to minInterval
		randomTimerDuration[timerId]= minHelloInterval;
	}
	else {
		// exponential backoff double duration until we reached maxDuration
		if (randomTimerDuration[timerId] < maxHelloInterval)
		{ // double waiting time for next call until maxDuration is reached
			randomTimerDuration[timerId] = std::min(2 * randomTimerDuration[timerId], maxHelloInterval);
		}
	}

	auto hellopkt(makeHelloPacket());
	sentVolume+= neighborTable.size() * hellopkt->getByteLength();
	sendToNeighbors(std::move(hellopkt));
	sentPackets+= neighborTable.size();
        lastPNHelloStateSeqNo = getLSSeqNo();

	return false;
}


// Overlay Neighbor Probing: probe for closest (overlay/ID-wise) neighbors, i.e., those in the deepest buckets
// sends at most one probe per call
// contact probing uses ProbeReq to check for reachability of the contact,
// this detects any broken links along the current path
bool R2Kademlia::doProbeNeighbors(short timerId)
{
	ASSERT(timerId >= 0 and timerId < fixedTimerIdBase);
	if (neighborTable.empty())
		return false; // node is isolated

	// if no contacts selected for probing, construct a list first
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
		    contactp != nullptr and contactp->getPathAge() > R2Kademlia::PathInfoAgeThreshold) {
			EV_TRACE << "probing for neighbor: " << probeNeighborQueue.front() << endl;
			//sendFindNode(probeNeighborQueue.front(), true);
			sendPathProbe(probeNeighborQueue.front());
			probed= true;
		}
		probeNeighborQueue.pop_front();
        } // end while

	return false;
}



// Contact Probing: check for contact validity (see also doProbeNeighbors), does not check for PNContacts
// sends at most one probe per call
// contact probing uses ProbeReq to check for reachability of the contact,
// this detects any broken links along the current path
bool R2Kademlia::doProbeContacts(short timerId)
{
	ASSERT(timerId >= 0 and timerId < fixedTimerIdBase);
	if (neighborTable.empty())
		return false; // node is isolated

	// if no contacts selected for probing, construct a list first
	// note that doPathRefresh() also fills contacts into this queue (consecutively at the front) 
	if ( probeContactQueue.empty() )
	{
		// fill queue with current oldest neighbors (2 for each bucket)
		// where we have no information about the path liveness/validity
		routingTable.getOldestValidContacts(probeContactQueue);
	}

	// check for reachability of neighbors
	bool probed(false);
	// randomoffset is used to avoid sending refresh in a bulk since most nodes startup at the same time
	auto randomoffset= intuniform(0,R2Kademlia::PathRefreshThreshold/2);
	while (probeContactQueue.size() > 0 and not probed)
	{
		// do not probe in case contact not found or too young
		if (auto contactp= routingTable.getContact(probeContactQueue.front());
		    contactp != nullptr
		    and
		    contactp->getPathLength() > 0 // not necessary to probe PNs
		    and (contactp->getPathAge() > R2Kademlia::PathInfoAgeThreshold or
                        contactp->getLastPathRefreshAge() > R2Kademlia::PathRefreshThreshold-randomoffset)) {
			EV_TRACE << "probing for contact: " << probeContactQueue.front() << " path age:" << contactp->getPathAge()
				 << " last path refresh age:" << contactp->getLastPathRefreshAge() << endl;
                            sendPathProbe(probeContactQueue.front());
			    // send multiple probes in case path refreshes are required (they are all at the front)
			    if (contactp->getLastPathRefreshAge() < R2Kademlia::PathRefreshThreshold-randomoffset) {
				    probed= true;
			    }
		}
		else
			EV_TRACE << "skipped contact: " << probeContactQueue.front() << endl; 

		probeContactQueue.pop_front();
        };
	return false;
}


bool R2Kademlia::doSendExplorePathPacket(short timerId)
{
	ASSERT(timerId >= 0 and timerId < fixedTimerIdBase);
	if (neighborTable.empty())
		return false; // node is isolated

	// check for alternative paths to contacts
	if (auto contactp= routingTable.getRandomContact();
	    contactp != nullptr) {
		EV_TRACE << "explore path for contact: " << *contactp << endl;

                if (auto contacts = routingTable.getClosestContacts(
                        contactp->getKey(), 2 * findNodeNumber, true);
                    not contacts.empty()) {

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

	return false; // do not stop timer
}


bool R2Kademlia::doCheckPNs(short timerId) {
	ASSERT(timerId >= 0 and timerId < fixedTimerIdBase);
        std::list<OverlayKey> resultlist;
	// get all PNs that have not sent a Hello or other message for a long time (assume 2 messages within 1.5 maxHelloInterval + additional 100ms)
        getRoutingTable().getDeadPNs(3 * maxHelloInterval.inUnit(SIMTIME_MS)+100, resultlist);
        for (auto it = resultlist.cbegin(); it != resultlist.cend(); it++) {
		if (auto nid = neighborTable.getNodeId(*it); nid != undefined_node) {
			nodeDisconnect(nid);
		}
                else {
			EV_WARN << "DeadPN " << it->shortStr() << ", but not present in neighbor table" << endl;
		}
        }
	return false; // do not stop timer
}


bool R2Kademlia::doProbeRandomly(short timerId)
{
	ASSERT(timerId >= 0 and timerId < fixedTimerIdBase);
	if (neighborTable.empty()) return false; // we are isolated
	OverlayKey probeKey;
	bool exact = false;
	uint32_t index = 0;
	switch (probeStrategy) {
		case 0:
			return false;
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

	return false;
}


bool R2Kademlia::doProbeSelf(short timerId)
{
	ASSERT(timerId >= 0 and timerId < fixedTimerIdBase);
	static const simtime_t maxDuration(SimTime(300,SIMTIME_S)); // check every 5 minutes

	// exponential backoff double duration until we reached maxDuration
	if (randomTimerDuration[timerId] < maxDuration)
	{ // double waiting time for next call until maxDuration is reached
		randomTimerDuration[timerId] = std::min(2 * randomTimerDuration[timerId], maxDuration);
	}

        if (neighborTable.empty())
          return false; // we are isolated

        sendFindNode(myOverlayKey, false);
	return false; // do not stop timer
}


// this is for testing/evaluation only and not required by R²/Kad
bool R2Kademlia::doReachabilityTest(short timerId)
{
	ASSERT(timerId >= 0 and timerId < fixedTimerIdBase);
        if (neighborTable.empty())
          return false; // we are isolated
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

	return false; // do not stop timer
}


bool R2Kademlia::doPathRefresh(short timerId)
{
	std::list<OverlayKey> contactlist;
	// refresh paths of contacts in the forwarding tier by sending a ProbeReq
	routingTable.getContactsforPathRefresh(contactlist);
	EV_TRACE << "doPathRefresh: number of contacts " << contactlist.size() << endl;
	for (const auto& key: contactlist)
	{
		// check if contact already in the list
		if (std::find(probeContactQueue.cbegin(),probeContactQueue.cend(),key) == probeContactQueue.cend()) {
			EV_TRACE << "adding contact " << key.shortStr() << " to probeContactQueue for path refresh" << endl;
			probeContactQueue.push_front(key);
		}
	}
	// note that the queue will be served by doProbeContacts

        return false; // do not stop timer
}


bool R2Kademlia::doGarbageCollectPathIDs(short timerId)
{
	ASSERT(timerId >= fixedTimerIdBase and timerId < oneShotTimerId); // assume fixed timer
	EV_TRACE << "PathID garbage collection called" << endl;
        fwdTablePathID.garbageCollect(notifyFirstSegmentSources,
                                      3 * R2Kademlia::PathRefreshThreshold);
	return false; // do not stop timer after callback
}

} // end namespace
