#ifndef __ROUTINGSIM_RequestQueue_H_
#define __ROUTINGSIM_RequestQueue_H_

#include<vector>
#include<functional>
#include<omnetpp/simtime_t.h>

namespace routingsim {

using namespace omnetpp;

typedef uint8_t RequestQueueElementActionType;

template<typename T>
class RequestQueue : public std::vector<T> {
public:
	RequestQueue(const std::string descriptiontxt,
		     std::function<simtime_t(const T&, RequestQueueElementActionType)> sendrequestcall,
		     std::function<void()> sendscheduledcall,
		     unsigned long initialScheduleDelay) :
		sendrequestcall(sendrequestcall),
		sendscheduledcall(sendscheduledcall),
		initialScheduleDelay(initialScheduleDelay),
		description(descriptiontxt),
		scheduled(false) {};
	const std::string& getDescription() { return description; }
	bool isScheduled() const { return scheduled; }
	void setScheduled() { scheduled= true; }
	void clearScheduled() { scheduled= false; }
	std::function<simtime_t(const T&, RequestQueueElementActionType)> sendrequestcall;
	std::function<void()> sendscheduledcall;
	unsigned long initialScheduleDelay = 0; // in µs
private:
	const std::string description;
	bool scheduled = false;
};



enum RequestQueueElementActions : uint8_t {
        unset = 0,   // unset == probe active path only
	probeProposedPath = 1, // if unset probe active path only
	pathsetupcheck = 2,     // setup PathID during PathProbe
	get_OverlayNeighborsSource = 4
};


} // end namespace

#endif
