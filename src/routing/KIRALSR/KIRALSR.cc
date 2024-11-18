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

#include "KIRALSR.h"
#include "KIRALSR_Packet.h"
#include "clog.h"
#include "routing/KIRA/KIRA.h"
#include "routing/KIRA/TimerMsg.h"
#include "simtime.h"
#include <memory>

namespace routingsim {

Define_Module(KIRALSR);

void KIRALSR::onInitialize() {
	EV << "KIRALSR::onInitialize()" << endl;
    if (auto proto = this->node().getProtocol<R2Kademlia>(); proto != nullptr) {
        EV << "R2Kademlia Protocol Found!" << proto->getOverlayKey() << endl;
        kira = proto;
    } else {
        EV_WARN << "No R2Kademlia Protocol Found!" << endl;
    }
    
	bind(KIRALSR_PACKET_TYPE);
}

void KIRALSR::onConnect( nodeid_t nid ) {
	EV << "Now I am connected to " << nid << endl;
    addOneShotTimer(SimTime(10, SIMTIME_S), [this, nid](short i) {
        auto nodePkt = std::make_unique<NodePkt>("KIRALSR");
        auto pkt = makePacket<KIRALSRPacket>("KIRALSR");
        nodePkt->encapsulate(pkt.release());
        nodePkt->setType(KIRALSR_PACKET_TYPE);
        EV_INFO << "Sending Flood Packet" << nid << endl;
        this->kira->sendFloodPacket(std::move(nodePkt));
        return false;
    });
}

void KIRALSR::onDisconnect( nodeid_t nid ) {
	EV << "Now I am disconnected from " << nid << endl;
}

void KIRALSR::onMessage( NodePkt_p pkt ) {
	EV << "received msg" << endl;
	switch (pkt->getType()) {
    case KIRALSR_PACKET_TYPE:
		EV << "Got KIRALSR packet!" << endl;

	default:
		EV << "unhandled packet: " << pkt->getType() << endl;
	}
}

// this handles timer self messages, if the timer triggers
void KIRALSR::onSelfMessage(cMessage_p msg)
{
	// handle timer
	// timerId == -1 is a one shot timer
	// timerId >= 0: randomized timer
	// timerId <= -2: fixed timer
	short timerId = msg->getKind();

	if (auto timerCallback(static_cast<TimerMsg::cbfunc_t *>(msg->getContextPointer()));
	    timerCallback != nullptr) {
		// now call the callback function
		bool stop_timer= (*timerCallback)(timerId);
		delete timerCallback;
        msg->setContextPointer(nullptr);

		// delete the callback function for one shot messages or if the callback returns true
		//if (timerId == -1  or stop_timer) {
		//	delete timerCallback;
        //                msg->setContextPointer(nullptr);
        //                if (stop_timer) {
		//	  if (timerId >= 0 and (size_type) timerId < randomTimerMsg.size()) {
        //                    randomTimerMsg[timerId]= nullptr;
		//	    randomTimerDuration[timerId]= omnetpp::SimTime::ZERO;
        //                  }
		//	  else
		//	  if (timerId < 0) {
        //                    if (size_type timerIdx = -2 - timerId; timerIdx < fixedTimerMsg.size()) {
        //                      fixedTimerMsg[timerIdx]= nullptr;
		//	      fixedTimerDuration[timerIdx]= omnetpp::SimTime::ZERO;
        //                    }
        //                  }
        //                }
		//	return;
		//}
	}

	//// reschedule timer message if duration > 0, otherwise it was a one shot timer
	//if (timerId  >= 0)
	//{ // these timers draw their next time period randomly from the interval
	//  // [randomTimerDuration[timerId]/2, randomTimerDuration[timerId]/2*3]
	//	ASSERT((size_t)timerId < randomTimerDuration.size());
	//	ASSERT(randomTimerMsg[timerId] == msg.get());
	//	if (randomTimerDuration[timerId] > omnetpp::SimTime::ZERO)
	//	{
	//		simtime_t time = uniform(randomTimerDuration[timerId]/2, randomTimerDuration[timerId]/2*3);
	//		scheduleAt(simTime() + time, msg.release());
	//	}
	//}
	//else if (timerId < -1)
	//{
	//	size_type timerIdx= -2-timerId;
	//	ASSERT(timerIdx < fixedTimerDuration.size());
	//	ASSERT(fixedTimerMsg[timerIdx] == msg.get());
	//	if (fixedTimerDuration[timerIdx] > omnetpp::SimTime::ZERO)
	//	{
	//		scheduleAt(simTime() + fixedTimerDuration[timerIdx], msg.release());
	//	}
	//}
}

void KIRALSR::reset() {
	EV << "resetting state of KIRALSR" << endl;
}

} //namespace
