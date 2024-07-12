#ifndef __ROUTINGSIM_RateLimitedQueue_H_
#define __ROUTINGSIM_RateLimitedQueue_H_

#include <functional>
#include <omnetpp/simtime_t.h>
#include "KadPacket.h"

namespace routingsim {


using namespace omnetpp;

using KadPacket_p = std::unique_ptr<KadPacket>;
  using KadPacketQueue = std::list<std::pair<simtime_t,KadPacket_p>>;

class RateLimitedQueue : public KadPacketQueue {
public:
  RateLimitedQueue(const std::string descriptiontxt,
                   unsigned long initialScheduleDelay,
                   unsigned long nextCallPauseTime,
		   std::function<void(simtime_t initialOffset, simtime_t duration, std::function<bool(short timerId)> callback)> addfixedtimer,
                   std::function<void(simtime_t,KadPacket_p)> sendscheduledcall)
    : initialScheduleDelay(initialScheduleDelay), nextCallPauseTime(nextCallPauseTime), description(descriptiontxt),
    scheduled(false), scheduletimer(addfixedtimer), sendscheduledcall(sendscheduledcall)
    {};
  const std::string& getDescription() { return description; }
  bool isScheduled() const { return scheduled; }
  void setScheduled() { scheduled= true; }
  void clearScheduled() { scheduled = false; }
  void addPacket(KadPacket_p p);
  size_t getMaxLength() { return maxLength; }
private:
  unsigned long initialScheduleDelay= 0; // in µs
  unsigned long nextCallPauseTime = 0;   // in µs
  size_t maxLength= 0;
  const std::string description;
  bool scheduled = false;
  std::function<void(simtime_t initialOffset, simtime_t duration, std::function<bool(short int timerId)> callback)> scheduletimer;
  std::function<void(simtime_t,KadPacket_p)> sendscheduledcall;
};


inline
void
RateLimitedQueue::addPacket(KadPacket_p p)
{
  //EV_TRACE << description << " addPacket called - scheduled=" << scheduled << endl;
  push_back(std::make_pair(simTime(), std::move(p)));
  if (size() > maxLength)
    maxLength= size();

  if (not scheduled) {
    //EV_TRACE << description << " initialScheduleDelay " << initialScheduleDelay << "µs, nextCallPauseTime " << nextCallPauseTime << "µs" << endl;
    scheduletimer(SimTime(initialScheduleDelay, SIMTIME_US),
                  SimTime(nextCallPauseTime, SIMTIME_US),
                  [this](short int timerID){
		      EV_TRACE << description << ": timer " << timerID << " callback for rate limited sending called" << endl;
                      if (not this->empty()) {
			EV_TRACE << "sendscheduled call" << endl;
			sendscheduledcall(this->front().first,std::move(this->front().second));
                        this->pop_front();
                        if (not this->empty())
			  return false;
                      }

                      scheduled = false;
		      return true; // stop timer after this callback
		  });
    scheduled= true;
  };
}

} // end namespace

#endif
