#ifndef __ROUTINGSIM_TimerMsg_H_
#define __ROUTINGSIM_TimerMsg_H_

#include "omnetpp.h"
#include <functional>

namespace routingsim {

class TimerMsg : public cMessage {
public:
	// callback function type
	using cbfunc_t = std::function<void(short timerId)>;

	TimerMsg(short timerId, cbfunc_t callback) : cMessage("TimerMsg", timerId) {
		setContextPointer( reinterpret_cast<void *>( new auto(callback) ) );
	};

        TimerMsg(const TimerMsg& other) = delete;
	virtual ~TimerMsg() { delete static_cast<cbfunc_t*>(getContextPointer()); setContextPointer(nullptr); }
};

} // namespace routingsim


#endif
