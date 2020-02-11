#include <simtime.h>
#include <csimulation.h>

using namespace omnetpp;

namespace routingsim {

	
class TrickleTimer {
	SimTime Imin= SimTime(100,SIMTIME_MS); // minimum interval size
	unsigned int Imax= 5; // maximum interval doubling
	SimTime maxInterval= 0; // maximum interval size
	unsigned int k= 1; // redundancy constant
	
	SimTime currentInterval= 0; // current interval
	SimTime currentTime= 0;
	unsigned int counter= 0;
public:
        TrickleTimer(SimTime minInterval, unsigned int maxDoubling, unsigned int redundancy= 5)
		: Imin(minInterval), Imax(maxDoubling), maxInterval(maxDoubling*minInterval), k(redundancy)
	{
		currentInterval= getSimulation()->getContext()->uniform(Imin, maxInterval);
	};
	inline void reinit() { currentInterval= getSimulation()->getContext()->uniform(Imin, maxInterval); counter= 0; }
	inline void begin() { currentTime= getSimulation()->getContext()->uniform(currentInterval/2, currentInterval); counter= 0; }
	inline SimTime getCurrentTime() const { return currentTime; }
	// doubles the interval if interval expires
	inline void expired() { currentInterval= (2*currentInterval < maxInterval) ? 2*currentInterval : maxInterval; }
	// increment the counter (whenever Trickle hears a transmission that is "consistent")
	inline void increment() { counter++; }
	// If Trickle hears a transmission that is "inconsistent" and I is greater than Imin,
	// it resets the Trickle timer.
	void reset();
	// returns true if transmission is allowed
	inline bool transmit() const { return counter < k; }
	inline unsigned int getImax() const { return Imax; }
	inline SimTime getMaxInterval() const { return maxInterval; }
	inline SimTime getImin() const { return Imin; }
};


inline
void TrickleTimer::reset() {
	if (currentInterval > Imin) { currentInterval= Imin; begin(); }
	// If I is equal to Imin when Trickle hears an "inconsistent" transmission, Trickle does nothing.
}

} // end namespace
