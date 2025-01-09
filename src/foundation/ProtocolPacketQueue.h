#ifndef __ROUTINGSIM_ProtocolPacketQueue_H
#define __ROUTINGSIM_ProtocolPacketQueue_H

#include "NodePkt_m.h"
#include "csimulation.h"
#include "simtime.h"
#include <memory>
#include <queue>
#include <utility>
#include <functional>

namespace routingsim {

using NodePkt_p = std::unique_ptr<NodePkt>;

/*
 * @class ProtocolPacketQueue
 * this class is used for modeling per packet processing times
 * NodePkts are enqueued if necessary
 */
class ProtocolPacketQueue {
public:
	ProtocolPacketQueue(std::function<simtime_t(void)> processingTimeFunc, size_t maxQueueLengthSpec)
		: processingTimeFunc(processingTimeFunc), maxQueueLength(maxQueueLengthSpec) { ASSERT(maxQueueLength>0); };
	/// returns true if packet was added and false, if it was not
        bool addPacket(NodePkt_p p);
	/// returns the first packet (including timestamp) and removes it from the queue
        std::pair<simtime_t, NodePkt_p> popPacket();
        /// returns the processing time for the current packet
        simtime_t getProcessingTime() const { return processingTimeFunc(); }
	/// sets the FinishingTime of the currently served packet (set randomly by default using the processingTimeFunc())
        void setFinishingTime(simtime_t tp = SIMTIME_ZERO) {
          finishingTimeCurrentlyServedPacket = (tp == SIMTIME_ZERO) ? simTime()+processingTimeFunc() : tp; }
	simtime_t getFinishingTime() const { return finishingTimeCurrentlyServedPacket; }
	/// returns true if packets waiting for being served or current packet not yet finished
        bool isBusy() const {return not packetQueue.empty() or simTime() <= finishingTimeCurrentlyServedPacket;};
	bool empty() const { return packetQueue.empty(); }
	size_t size() const { return packetQueue.size(); }
	void setScheduled(bool state) { scheduled = state; }
        bool isScheduled() const { return scheduled; }
	// timerSelfMsg is typically a ProtocolTimerMsg
        void setTimerMsg(cMessage* setTimerSelfMsg) { timerSelfMsg.reset(setTimerSelfMsg); }
	cMessage* getTimerMsg() const { return timerSelfMsg.get(); }
	unsigned int getOverloadDiscardedPackets() const { return overloadDiscardedPackets; }
        simtime_t getMaxSojournTime() const { return maxSojournTime; }
	//std::queue< std::pair<simtime_t, NodePkt_p> >& getRawQueue() { return packetQueue; } // only for debugging
private:
	std::queue< std::pair<simtime_t, NodePkt_p> > packetQueue;
	std::function<simtime_t(void)> processingTimeFunc;
        bool enqueue_condition(); // TODO later: make this variable using const std::function<bool(void)>& func
        bool scheduled = false;
	std::unique_ptr<cMessage> timerSelfMsg = nullptr;
        simtime_t finishingTimeCurrentlyServedPacket = SIMTIME_ZERO;
        unsigned int overloadDiscardedPackets = 0UL;
        simtime_t maxSojournTime = 0UL;
	size_t maxQueueLength = 10000;
};


inline
bool
ProtocolPacketQueue::addPacket(NodePkt_p p) {
	//EV_TRACE << "ProtocolPacketQueue::addPacket() called, current queue size=" << packetQueue.size();
	if (enqueue_condition()) {
		packetQueue.push(std::make_pair(simTime(),std::move(p)));
                return true;
        } else {
		overloadDiscardedPackets++;
		return false;
	}
}


inline
std::pair<simtime_t, NodePkt_p>
ProtocolPacketQueue::popPacket() {
  if (not packetQueue.empty()) {
	  auto return_val = std::make_pair( packetQueue.front().first, std::move(packetQueue.front().second));
          auto sojournTime = simTime() - return_val.first;
	  ASSERT(sojournTime >= SIMTIME_ZERO);
          if (sojournTime > maxSojournTime)
		  maxSojournTime= sojournTime;
          //EV_TRACE << "popPacket(): waited " << sojournTime << "s"; 
          packetQueue.pop();
	  return return_val;
  }
  else
	  return std::make_pair(simTime(),nullptr);
}


// default policy: do not enqueue packets 1s after the first packet
// returns true if packet should be enqueued, false if not
inline
bool
ProtocolPacketQueue::enqueue_condition() {
	if (not packetQueue.empty() and packetQueue.size() >= maxQueueLength)
		return false;
        else
		return true;
};

}
#endif
