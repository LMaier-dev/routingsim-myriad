#ifndef __ROUTINGSIM_TimerMsg_H_
#define __ROUTINGSIM_TimerMsg_H_


#include "cmessage.h"
#include <functional>

namespace routingsim {

class TimerMsg : public omnetpp::cMessage {
public:
	// callback function type: should return true if timer should stop
	using cbfunc_t = std::function<bool(short timerId)>;

	TimerMsg(short timerId, cbfunc_t callback) : cMessage("TimerMsg", timerId) {
		setContextPointer( reinterpret_cast<void *>( new auto(callback) ) );
	};

        TimerMsg(const TimerMsg& other) = delete;
        virtual ~TimerMsg() { delete static_cast<cbfunc_t*>(getContextPointer()); setContextPointer(nullptr); }
};

} // namespace routingsim


#endif
